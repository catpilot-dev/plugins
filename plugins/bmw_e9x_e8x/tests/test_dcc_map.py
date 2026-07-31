import pytest
import sys
import os

# Add plugin dir to path so bmw package is importable
_PLUGIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PLUGIN_DIR not in sys.path:
  sys.path.insert(0, _PLUGIN_DIR)

from bmw.dcc_map import (expected_accel, gap_for_accel, accel_envelope,
                         select_cruise_command, MS_TO_KPH, SETPOINT_DEADBAND_KPH)
from bmw.dcc_map_table import GAP_BPS, V_BPS, A_TABLE


# ---- expected_accel / gap_for_accel / accel_envelope ----
# These are retained in bmw.dcc_map for OFFLINE ANALYSIS ONLY (tools/dcc_study/)
# and are no longer part of the control path -- select_cruise_command below no
# longer calls any of them. These tests just confirm the table plumbing itself
# still works.

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
#
# select_cruise_command no longer inverts the map or splits into an
# acceleration/braking branch with a direction gate. The whole body is now:
#
#   desired = max(v_target, min_setpoint)
#   err_kph = (desired - setpoint) * MS_TO_KPH
#   if abs(err_kph) < SETPOINT_DEADBAND_KPH: return None
#   if err_kph > 0:
#     if a_target <= 0: return None      # model veto, raise side only
#     return 'plus1'
#   return 'minus5' if -err_kph >= 5.0 else 'minus1'
#
# There is deliberately NO veto on the braking side: lowering the setpoint
# toward vTarget can only reduce the commanded speed, so it is always safe.

def cmd(a_target, v_ego, setpoint, v_target, min_setpoint=5.0):
  return select_cruise_command(a_target, v_ego, setpoint, v_target, min_setpoint)


def test_deadband_emits_nothing():
  v = 20.0
  assert cmd(0.0, v, setpoint=v, v_target=v + 0.2) is None


def test_accel_request_below_target_emits_plus():
  v = 20.0
  assert cmd(+0.3, v, setpoint=v, v_target=v + 5.0) == "plus1"


def test_raise_is_always_plus1_never_plus5():
  """The old map-inversion branch could emit 'plus5' for a large gap; the new
  logic never does -- a raise is always 'plus1' (20 Hz), regardless of how
  large err_kph is, per the spec: 'plus1 only, 20 Hz -- smooth'."""
  v = 20.0
  assert cmd(+2.0, v, setpoint=v, v_target=v + 50.0) == "plus1"


def test_small_setpoint_error_uses_step1():
  """A raise is always 'plus1', even for a small (2 km/h) setpoint error."""
  v = 20.0
  assert cmd(0.1, v, setpoint=v, v_target=v + 2.0 / MS_TO_KPH) == "plus1"


def test_setpoint_never_commanded_above_v_target():
  v, v_target = 20.0, 20.5
  assert cmd(+0.5, v, setpoint=v_target, v_target=v_target) is None


def test_decel_request_emits_minus():
  v = 25.0
  assert cmd(-0.5, v, setpoint=v, v_target=v - 5.0, min_setpoint=5.0) in ("minus1", "minus5")


def test_large_decel_error_uses_step5():
  v = 25.0
  assert cmd(-0.5, v, setpoint=v, v_target=v - 10.0, min_setpoint=5.0) == "minus5"


def test_small_decel_error_uses_step1():
  v = 25.0
  assert cmd(-0.5, v, setpoint=v, v_target=v - 2.0 / MS_TO_KPH, min_setpoint=5.0) == "minus1"


def test_commanded_setpoint_never_goes_below_min():
  """The desired-setpoint clamp is what protects min cruise speed.

  A minus tick moves 1 or 5 km/h and is only emitted when at least that much
  error exists above max(v_target, min_setpoint), so it can never carry the
  setpoint under that floor.
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


def test_no_veto_on_braking_side():
  """Deliberate design point: lowering the setpoint toward vTarget is always
  safe, so there is NO a_target gate on the braking side -- unlike the old
  direction gate, a positive a_target (disagreeing with the speed error) must
  not block a minus command."""
  assert cmd(+2.0, 25.0, setpoint=25.0, v_target=5.0, min_setpoint=5.0) == "minus5"


# ---- safety regression tests (still-relevant floor/veto cases) ----

def test_conflict_setpoint_below_floor_blocks_accel():
  """When v_target < min_setpoint, the min-speed floor makes `desired` sit
  above `setpoint` here, which is technically a "raise" -- but a_target is
  negative, so the model veto still blocks it. Regression: a naive floor
  clamp must not sneak an acceleration command past the veto."""
  assert cmd(-0.8, 10.0, 8.9, 4.0, 9.72) is None


def test_conflict_setpoint_above_floor_allows_decel():
  """When v_target < min_setpoint but setpoint is still above the floor,
  deceleration toward the floor must still work (no veto on the braking
  side)."""
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


@pytest.mark.parametrize("a_target,v_ego,setpoint,v_target,min_setpoint,expected", [
    (-1.0, 11.0, 9.17, 10.5, 9.72, None),
    (-0.8, 10.0, 8.9,  4.0,  9.72, None),
    (-0.8, 13.9, 13.0, 4.0,  9.72, 'minus5'),
    ( 0.4, 10.0, 9.0,  15.0, 9.72, 'plus1'),
    ( 0.3, 20.0, 20.0, 25.0, 5.0,  'plus1'),
    (-0.5, 25.0, 25.0, 20.0, 5.0,  'minus5'),
])
def test_canonical_cases_unchanged(a_target, v_ego, setpoint, v_target, min_setpoint, expected):
  """Locks in the hand-verified reference cases from the vTarget-tracking spec.
  These values happen to coincide with the pre-simplification reference set
  (the old two-branch/gate logic and the new desired = max(v_target,
  min_setpoint) logic agree on all six), but the reasoning behind each is now
  just: compute desired, check the deadband, then (for a raise) the a_target
  sign veto."""
  assert cmd(a_target, v_ego, setpoint, v_target, min_setpoint) == expected


# ---- acceleration: model veto only, no magnitude trust ----

def test_real_world_sluggish_case_now_accelerates():
  """The motivating field observation: car held at 80 km/h (v_ego=24.08 m/s)
  with a 97 km/h target (v_target=27.11 m/s) because a_target was a barely
  positive +0.015 -- under the old aTarget-driven setpoint, gap_for_accel of
  that tiny a_target put the setpoint only ~1.3 km/h above v_ego, below the
  1.0 km/h deadband, so nothing was ever sent. Tracking vTarget directly
  fixes this."""
  assert cmd(0.015, 24.08, 24.44, 27.11, 9.72) == 'plus1'


def test_brake_veto_blocks_acceleration():
  """a_target negative means the model does not agree with accelerating, even
  though the setpoint error clearly calls for it -- conflicting signals must
  coast (None), not fight the model."""
  assert cmd(-0.5, 20.0, 20.0, 30.0, 5.0) is None


def test_brake_veto_boundary_just_below_zero_still_blocks():
  """a_target just below zero (-0.29) still blocks: the gate is a strict sign
  requirement (a_target > 0), not a magnitude threshold."""
  assert cmd(-0.29, 20.0, 20.0, 30.0, 5.0) is None


def test_accel_veto_boundary_exactly_zero_blocks():
  """a_target of exactly 0.0 does not accelerate: the gate requires strict
  positivity (a_target > 0). A large setpoint error alone must not be
  enough."""
  assert cmd(0.0, 20.0, 20.0, 30.0, 5.0) is None


def test_accel_veto_boundary_barely_positive_still_accelerates():
  """a_target barely positive (+0.01) still satisfies a_target > 0, so a
  large setpoint error must still produce a raise."""
  assert cmd(0.01, 20.0, 20.0, 30.0, 5.0) == 'plus1'


def test_deadband_blocks_even_with_large_setpoint_gap_to_floor():
  """Setpoint already within 1 km/h of desired (= max(v_target, min_setpoint))
  must emit nothing, even with a large positive a_target."""
  v_ego, v_target = 20.0, 25.0
  setpoint = v_target - 0.5 / MS_TO_KPH   # 0.5 km/h short of v_target
  assert cmd(0.5, v_ego, setpoint, v_target, 5.0) is None


# ---- brute-force invariant sweep ----
#
# The literal "never above v_target" / "never below min_setpoint" framing
# used during design collapses, once the min-speed floor is folded in, into a
# single reference point: desired = max(v_target, min_setpoint). When
# v_target >= min_setpoint (the common case) this is just v_target; when
# v_target < min_setpoint (near-stop targets below the enable floor) this is
# min_setpoint, and a plus1 raising the setpoint UP TO that floor is the
# intended "never strand below min cruise" behaviour -- not a bug. So the
# sweep checks against `desired`, not against v_target/min_setpoint in
# isolation.

def test_full_invariant_sweep():
  """Brute-force sweep checking, on every emitted command:

    1. A plus1 tick never carries the setpoint above desired = max(v_target,
       min_setpoint), and only ever moves exactly 1 km/h.
    2. A minus tick never carries the setpoint below desired, and only moves
       5 km/h when the error is >= 5 km/h, else 1 km/h.
    3. Acceleration (a plus command) never happens when a_target <= 0.
    4. (covered separately by test_nan_guards / test_inf_guard) non-finite
       input always returns None.
  """
  a_targets = [-1.5, -1.0, -0.5, -0.1, 0.0, 0.01, 0.2, 0.5, 2.0]
  v_egos = [6.0, 9.0, 11.0, 15.0, 20.0, 28.0]
  setpoints = [8.0, 9.2, 10.0, 12.0, 15.0, 20.0, 25.0]
  v_targets = [4.0, 8.0, 10.5, 15.0, 22.0, 30.0]
  min_setpoints = [5.0, 9.72]

  violations = []
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
            desired = max(v_target, min_sp)
            step_kph = 5.0 if result in ("plus5", "minus5") else 1.0
            step_ms = step_kph / MS_TO_KPH
            err_kph = (desired - setpoint) * MS_TO_KPH

            if result == "plus1":
              if a_target <= 0:
                violations.append(("accel without a_target veto", a_target, v_ego, setpoint, v_target, min_sp, result))
              new_setpoint = setpoint + step_ms
              if new_setpoint > desired + 1e-9:
                violations.append(("plus overshoots desired", a_target, v_ego, setpoint, v_target, min_sp, result))
              if err_kph < SETPOINT_DEADBAND_KPH - 1e-9:
                violations.append(("plus emitted inside deadband", a_target, v_ego, setpoint, v_target, min_sp, result))
            elif result in ("minus1", "minus5"):
              new_setpoint = setpoint - step_ms
              if new_setpoint < desired - 1e-9:
                violations.append(("minus undershoots desired", a_target, v_ego, setpoint, v_target, min_sp, result))
              if result == "minus5" and -err_kph < 5.0 - 1e-9:
                violations.append(("minus5 used below 5 kph error", a_target, v_ego, setpoint, v_target, min_sp, result))
            else:
              violations.append(("unexpected command", a_target, v_ego, setpoint, v_target, min_sp, result))

  assert checked == len(a_targets) * len(v_egos) * len(setpoints) * len(v_targets) * len(min_setpoints)
  assert emitted > 0, "sweep produced no emitted commands to check"
  assert not violations, f"{len(violations)} invariant violation(s), first few: {violations[:5]}"
