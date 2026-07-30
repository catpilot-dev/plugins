import pytest
import sys
import os

# Add plugin dir to path so bmw package is importable
_PLUGIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PLUGIN_DIR not in sys.path:
  sys.path.insert(0, _PLUGIN_DIR)

from bmw.dcc_map import (expected_accel, gap_for_accel, accel_envelope,
                         select_cruise_command, MS_TO_KPH, V_ERROR_DEADZONE)
from bmw.dcc_map_table import GAP_BPS, V_BPS, A_TABLE


def test_table_columns_are_monotone():
  for j in range(len(V_BPS)):
    col = [A_TABLE[i][j] for i in range(len(GAP_BPS))]
    assert all(b >= a for a, b in zip(col, col[1:])), f"column {j} not monotone"


def test_expected_accel_hits_table_nodes():
  for i, g in enumerate(GAP_BPS):
    for j, v in enumerate(V_BPS):
      assert expected_accel(g, v) == pytest.approx(A_TABLE[i][j], abs=1e-9)


def test_expected_accel_clamps_outside_table():
  v = V_BPS[2]
  assert expected_accel(GAP_BPS[0] - 5.0, v) == pytest.approx(expected_accel(GAP_BPS[0], v))
  assert expected_accel(GAP_BPS[-1] + 5.0, v) == pytest.approx(expected_accel(GAP_BPS[-1], v))


def test_negative_gap_gives_negative_accel():
  for v in V_BPS:
    assert expected_accel(-2.0, v) < 0.0
    assert expected_accel(+1.5, v) > expected_accel(-1.0, v)


def test_inversion_round_trips_inside_the_envelope():
  for v in V_BPS:
    lo, hi = accel_envelope(v)
    for frac in (0.2, 0.5, 0.8):
      a = lo + (hi - lo) * frac
      g = gap_for_accel(a, v)
      assert expected_accel(g, v) == pytest.approx(a, abs=0.02)


def test_inversion_clamps_beyond_authority():
  v = V_BPS[2]
  lo, hi = accel_envelope(v)
  assert gap_for_accel(hi + 5.0, v) == pytest.approx(GAP_BPS[-1])
  assert gap_for_accel(lo - 5.0, v) == pytest.approx(GAP_BPS[0])


def test_gap_never_escapes_table_bounds():
  for v in (1.0, V_BPS[0], V_BPS[-1], 60.0):
    for a in (-9.0, -1.0, 0.0, 1.0, 9.0):
      assert GAP_BPS[0] <= gap_for_accel(a, v) <= GAP_BPS[-1]


def test_envelope_is_ordered():
  for v in V_BPS:
    lo, hi = accel_envelope(v)
    assert lo < hi


# ---- command selection ----

def cmd(a_target, v_ego, setpoint, v_target, min_setpoint=5.0):
  return select_cruise_command(a_target, v_ego, setpoint, v_target, min_setpoint)


def test_deadband_emits_nothing():
  v, a = 20.0, 0.0
  sp = v + gap_for_accel(a, v)
  assert cmd(a, v, sp, v_target=v + 5.0) is None


def test_accel_request_below_target_emits_plus():
  v = 20.0
  assert cmd(+0.3, v, setpoint=v, v_target=v + 5.0) in ("plus1", "plus5")


def test_large_setpoint_error_uses_step5():
  v = 20.0
  _, a_max = accel_envelope(v)
  assert cmd(a_max, v, setpoint=v, v_target=v + 10.0) == "plus5"


def test_small_setpoint_error_uses_step1():
  v = 20.0
  a = 0.1   # small positive a_target: 0.0 would fail the direction gate (needs a_target > 0)
  sp = v + gap_for_accel(a, v) - (2.0 / 3.6)   # 2 km/h short -> one tick
  assert cmd(a, v, setpoint=sp, v_target=v + 10.0) == "plus1"


def test_setpoint_never_commanded_above_v_target():
  v, v_target = 20.0, 20.5
  assert cmd(+0.5, v, setpoint=v_target, v_target=v_target) is None


def test_decel_request_emits_minus():
  v = 25.0
  assert cmd(-0.5, v, setpoint=v, v_target=v - 5.0, min_setpoint=5.0) in ("minus1", "minus5")


def test_beyond_authority_saturates_not_escalates():
  v = 20.0
  _, a_max = accel_envelope(v)
  assert cmd(a_max + 5.0, v, v, v + 20.0) == cmd(a_max, v, v, v + 20.0)


def test_commanded_setpoint_never_goes_below_min():
  """The desired-setpoint clamp is what protects min cruise speed.

  A minus tick moves 1 or 5 km/h and is only emitted when at least that much
  error exists above the floor, so it can never carry the setpoint under it.
  """
  v, min_sp = 10.0, 8.0
  for excess_kph in (0.2, 0.9, 1.5, 4.0, 6.0, 12.0):
    sp = min_sp + excess_kph / 3.6
    out = cmd(-1.0, v, setpoint=sp, v_target=0.0, min_setpoint=min_sp)
    if out is None:
      continue
    step_kph = 5.0 if out == "minus5" else 1.0
    assert sp - step_kph / 3.6 >= min_sp - 1e-9, \
        f"{out} from setpoint {sp:.3f} would cross the {min_sp} m/s floor"


# ---- safety regression tests ----

def test_conflict_setpoint_below_floor_blocks_accel():
  """When v_target < min_setpoint and decel is requested, the min-speed floor
  must not trigger an ACCELERATION command.

  Regression: with naive clamp(desired, min_setpoint) after clamp(desired, v_target),
  desired could be pushed above v_target, causing acceleration when both the
  planner and the controller ask for decel. This is a hard safety violation:
  it overrides the planner while the car is descending, leaving an unsafe
  target if openpilot disengages.
  """
  # Simulates: v_ego=10 m/s, setpoint=8.9 m/s, v_target=4.0 m/s (near stop),
  # min_setpoint=9.72 m/s (35 km/h cruise floor), a_target=-0.8 m/s^2 (decel requested).
  # The brief's code would return 'plus1' here.
  assert cmd(-0.8, 10.0, 8.9, 4.0, 9.72) is None


def test_conflict_setpoint_above_floor_allows_decel():
  """When v_target < min_setpoint but setpoint is still above the floor,
  deceleration toward the floor must still work.
  """
  # Simulates: same scenario but setpoint=13.0 m/s is still above floor.
  # This should allow a decel command toward the floor.
  assert cmd(-0.8, 13.9, 13.0, 4.0, 9.72) == "minus5"


def test_nan_guards():
  """NaN in any argument must return None, not propagate through interpolation."""
  import math
  v, sp, vt, min_sp = 20.0, 20.0, 25.0, 5.0
  assert cmd(math.nan, v, sp, vt, min_sp) is None
  assert cmd(0.0, math.nan, sp, vt, min_sp) is None
  assert cmd(0.0, v, math.nan, vt, min_sp) is None
  assert cmd(0.0, v, sp, math.nan, min_sp) is None
  assert cmd(0.0, v, sp, vt, math.nan) is None


def test_inf_guard():
  """Infinity in any argument must also return None (finiteness guard, not just NaN)."""
  v, sp, vt, min_sp = 20.0, 20.0, 25.0, 5.0
  assert cmd(float('inf'), v, sp, vt, min_sp) is None
  assert cmd(float('-inf'), v, sp, vt, min_sp) is None


def test_floor_never_raises_setpoint_when_desired_raw_is_below_it():
  """The min-speed floor may hold the setpoint up but must never RAISE it above
  what the planner actually asked for (desired_raw).

  Regression: v_target >= min_setpoint but v_ego + gap_for_accel(...) <
  min_setpoint (i.e. desired_raw < min_setpoint). The old guard
  `desired > v_target and err_kph > 0` never fired here because desired was
  clamped to <= v_target, yet the min_setpoint floor still pushed `desired`
  above the current setpoint, emitting an upward tick while a_target was
  negative — fighting a planner that is actively decelerating.
  """
  assert cmd(-1.0, 11.0, 9.17, 10.5, 9.72) is None


def test_floor_raise_invariant_sweep():
  """Invariant sweep over the grid where v_target >= min_setpoint but the raw
  desired setpoint (v_ego + gap_for_accel(...), before the floor clamp) sits
  below min_setpoint — i.e. the floor is actually engaging. Within that grid,
  whenever a_target < 0 and desired_raw < setpoint, no plus command may ever
  be emitted: the floor must hold the setpoint, never raise it against a
  planner that is asking for less than what's already commanded.
  """
  min_sp = 9.72  # 35 km/h cruise floor
  checked = 0
  for v_ego in [8.0, 8.5, 9.0, 9.5, 10.0, 11.0, 12.0]:
    for v_target in [min_sp, min_sp + 0.5, min_sp + 2.0, min_sp + 5.0]:
      for a_target in [-3.0, -2.0, -1.0, -0.5, -0.1]:
        desired_raw = min(v_ego + gap_for_accel(a_target, v_ego), v_target)
        if desired_raw >= min_sp:
          continue  # floor isn't actually engaging on this grid point
        for setpoint in [min_sp - 0.5, min_sp, min_sp + 0.5, min_sp + 1.0, min_sp + 2.0]:
          if not (a_target < 0 and desired_raw < setpoint):
            continue  # outside the invariant's precondition
          checked += 1
          result = cmd(a_target, v_ego, setpoint, v_target, min_sp)
          assert result not in ("plus1", "plus5"), \
              f"floor raised setpoint against planner: a_target={a_target}, " \
              f"v_ego={v_ego}, setpoint={setpoint:.2f}, v_target={v_target}, " \
              f"desired_raw={desired_raw:.3f} < setpoint={setpoint:.2f} -> {result}"
  assert checked > 0, "sweep grid produced no cases matching the invariant precondition"


def test_floor_never_causes_accel_above_v_target():
  """Invariant: when v_target < min_setpoint, never emit 'plus1' or 'plus5'.

  The min-speed floor is an availability concern, not a safety override.
  When the planner's target is below the floor, we may need to hold the floor
  (for deceleration toward it), but we must never raise the setpoint above
  what the planner asked for.
  """
  min_sp = 9.72  # 35 km/h cruise floor
  for v_ego in [8.0, 10.0, 15.0, 20.0]:
    for v_target in [0.0, 2.0, 4.0, 5.0]:
      if v_target >= min_sp:
        continue  # not testing this case
      for a_target in [-2.0, -1.0, -0.5, 0.0, 0.5]:
        for setpoint in [min_sp - 0.5, min_sp, min_sp + 0.5, min_sp + 1.0]:
          result = cmd(a_target, v_ego, setpoint, v_target, min_sp)
          assert result not in ("plus1", "plus5"), \
              f"floor override: a_target={a_target}, v_ego={v_ego}, setpoint={setpoint:.2f}, " \
              f"v_target={v_target}, min_sp={min_sp} -> {result}"


def test_floor_never_pushes_setpoint_past_v_target_when_setpoint_at_or_below_it():
  """Second hole in the planner-intent guard alone: when v_target < min_setpoint
  and the current setpoint sits at or below v_target, desired_raw == v_target
  (min() picks v_target), so `desired_raw < setpoint` is False and does not
  fire — yet `desired` is still floored up to min_setpoint, which is above
  v_target, so a plus tick was emitted and drove the setpoint above v_target.
  This is exactly the case the original `desired > v_target` guard protected,
  and it is not implied by `desired_raw < setpoint`. Both guards are required.
  """
  assert cmd(-0.5, 15.0, 7.0, 8.0, 9.72) is None    # setpoint 7.00 < v_target 8.0
  assert cmd(-0.5, 15.0, 8.0, 8.0, 9.72) is None    # setpoint == v_target 8.0 (equality edge)


@pytest.mark.parametrize("a_target,v_ego,setpoint,v_target,min_setpoint,expected", [
    (-1.0, 11.0, 9.17, 10.5, 9.72, None),
    (-0.8, 10.0, 8.9,  4.0,  9.72, None),
    (-0.8, 13.9, 13.0, 4.0,  9.72, 'minus5'),
    ( 0.4, 10.0, 9.0,  15.0, 9.72, 'plus5'),
    ( 0.3, 20.0, 20.0, 25.0, 5.0,  'plus1'),
    (-0.5, 25.0, 25.0, 20.0, 5.0,  'minus5'),
])
def test_canonical_cases_unchanged(a_target, v_ego, setpoint, v_target, min_setpoint, expected):
  """Locks in the hand-verified reference cases from the direction-gate spec.
  These must not move when the gate is added: three already return None on
  pre-existing guards, and the other three have a_target and v_error agreeing
  on direction, so the new gate is a no-op on them."""
  assert cmd(a_target, v_ego, setpoint, v_target, min_setpoint) == expected


# ---- direction gate: v_error and a_target must agree ----

def test_gate_suppresses_plus_inside_deadzone():
  """v_error is inside the deadzone even though err_kph > 0 and a_target > 0."""
  assert cmd(0.3, 20.0, 18.0, 20.1, 5.0) is None


def test_gate_now_allows_minus_inside_old_symmetric_deadzone():
  """Behaviour change: v_error = -0.1 (v_target=19.9, v_ego=20.0) sits inside
  the OLD symmetric deadzone [-V_ERROR_DEADZONE, +V_ERROR_DEADZONE], so this
  used to be suppressed. The gate is now asymmetric -- braking only requires
  v_error < +V_ERROR_DEADZONE, which -0.1 satisfies -- so a minus must fire."""
  assert cmd(-0.3, 20.0, 22.0, 19.9, 5.0) == 'minus5'


def test_gate_plus_still_requires_strict_positive_deadzone():
  """The plus branch is unchanged: a small-positive v_error (0 < v_error <
  V_ERROR_DEADZONE) must still suppress a raise even with a_target > 0 and
  err_kph > 0."""
  assert cmd(0.5, 20.0, 18.0, 20.1, 5.0) is None


def test_gate_minus_allowed_with_small_positive_v_error():
  """New asymmetric behaviour: a small-positive v_error (0 < v_error <
  V_ERROR_DEADZONE) must still allow a brake when a_target < 0 and the
  setpoint error clearly calls for it (err_kph <= -1)."""
  result = cmd(-0.5, 20.0, 22.0, 20.1, 5.0)
  assert result in ("minus1", "minus5")


def test_gate_suppresses_plus_when_a_target_disagrees():
  """v_error is strongly positive and err_kph > 0, but a_target is negative
  (noisy modelV2 disagreement) -- the gate must block the raise."""
  assert cmd(-0.1, 20.0, 18.0, 25.0, 5.0) is None


def test_gate_suppresses_minus_when_a_target_disagrees():
  """v_error is strongly negative and err_kph < 0, but a_target is positive
  (noisy modelV2 disagreement) -- the gate must block the lower."""
  assert cmd(0.1, 25.0, 22.0, 15.0, 5.0) is None


def test_gate_conjunction_sweep():
  """Sweep a grid of (a_target, v_ego, setpoint, v_target, min_setpoint) and
  assert the direction gate's conjunction holds on every emitted command:
  every plus has v_error > V_ERROR_DEADZONE and a_target > 0; every minus has
  v_error < V_ERROR_DEADZONE and a_target < 0 (the gate is asymmetric: both
  branches pivot on the same +V_ERROR_DEADZONE threshold, favouring braking)."""
  a_targets = [-2.0, -1.0, -0.5, -0.1, 0.0, 0.1, 0.5, 1.0, 2.0]
  v_egos = [6.0, 9.0, 11.0, 15.0, 20.0, 28.0]
  setpoints = [8.0, 9.2, 10.0, 12.0, 15.0, 20.0, 25.0]
  v_targets = [4.0, 8.0, 10.5, 15.0, 19.9, 20.0, 20.1, 22.0, 30.0]
  min_setpoints = [5.0, 9.72]

  checked = 0
  emitted = 0
  for a_target in a_targets:
    for v_ego in v_egos:
      for setpoint in setpoints:
        for v_target in v_targets:
          for min_sp in min_setpoints:
            checked += 1
            result = cmd(a_target, v_ego, setpoint, v_target, min_sp)
            if result is None:
              continue
            emitted += 1
            v_error = v_target - v_ego
            if result in ("plus1", "plus5"):
              assert v_error > V_ERROR_DEADZONE and a_target > 0, \
                  f"plus emitted without agreement: v_error={v_error}, a_target={a_target}"
            else:
              assert v_error < V_ERROR_DEADZONE and a_target < 0, \
                  f"minus emitted without agreement: v_error={v_error}, a_target={a_target}"
  assert checked == len(a_targets) * len(v_egos) * len(setpoints) * len(v_targets) * len(min_setpoints)
  assert emitted > 0, "sweep produced no emitted commands to check"


def test_full_invariant_sweep_plus_and_minus_and_planner_intent():
  """Brute-force sweep over a, v_ego, setpoint, v_target, min_setpoint, checking
  every one of select_cruise_command's core safety invariants on every emitted
  command:

    1. A plus tick must never land the setpoint above v_target if the current
       setpoint started at or below v_target.
    2. A minus tick must never land the setpoint below min_setpoint if the
       current setpoint started at or above min_setpoint.
    3. A plus must never be emitted when desired_raw (the planner's actual
       ask, before the floor clamp) is below the current setpoint.

  This is the grid that would have caught both the original min-speed-floor
  bug and the second hole where the `desired > v_target` guard was wrongly
  deleted — so it is deliberately exhaustive rather than minimal.
  """
  a_targets = [-1.5, -1.0, -0.5, -0.1, 0.0, 0.2, 0.5]
  v_egos = [6.0, 9.0, 11.0, 15.0, 20.0, 28.0]
  setpoints = [8.0, 9.2, 10.0, 12.0, 15.0, 20.0, 25.0]
  v_targets = [4.0, 8.0, 10.5, 15.0, 22.0, 30.0]
  min_setpoints = [5.0, 9.72]

  violations = []
  checked = 0
  for a_target in a_targets:
    for v_ego in v_egos:
      for setpoint in setpoints:
        for v_target in v_targets:
          for min_sp in min_setpoints:
            checked += 1
            result = cmd(a_target, v_ego, setpoint, v_target, min_sp)
            if result is None:
              continue
            step_kph = 5.0 if result in ("plus5", "minus5") else 1.0
            step_ms = step_kph / MS_TO_KPH

            if result in ("plus1", "plus5"):
              new_setpoint = setpoint + step_ms
              # Invariant 1: never land above v_target if we started at or below it.
              if setpoint <= v_target and new_setpoint > v_target + 1e-9:
                violations.append(
                    ("v_target ceiling breached", a_target, v_ego, setpoint, v_target, min_sp, result))
              # Invariant 3: desired_raw is the planner's own ask; a plus must
              # never fire when that ask is already below the current setpoint.
              desired_raw = min(v_ego + gap_for_accel(a_target, v_ego), v_target)
              if desired_raw < setpoint - 1e-9:
                violations.append(
                    ("planner-intent guard breached", a_target, v_ego, setpoint, v_target, min_sp, result))

            elif result in ("minus1", "minus5"):
              new_setpoint = setpoint - step_ms
              # Invariant 2: never land below min_setpoint if we started at or above it.
              if setpoint >= min_sp and new_setpoint < min_sp - 1e-9:
                violations.append(
                    ("min_setpoint floor breached", a_target, v_ego, setpoint, v_target, min_sp, result))

  assert checked == len(a_targets) * len(v_egos) * len(setpoints) * len(v_targets) * len(min_setpoints)
  assert not violations, f"{len(violations)} invariant violation(s), first few: {violations[:5]}"
