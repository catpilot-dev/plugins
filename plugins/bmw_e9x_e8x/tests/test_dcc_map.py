import pytest
import sys
import os

# Add plugin dir to path so bmw package is importable
_PLUGIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PLUGIN_DIR not in sys.path:
  sys.path.insert(0, _PLUGIN_DIR)

from bmw.dcc_map import (expected_accel, gap_for_accel, accel_envelope,
                         select_cruise_command, MS_TO_KPH, SETPOINT_DEADBAND_KPH,
                         STEP5_RAISE_KPH, STEP5_LOWER_KPH, BRAKE_LEAD_GAIN,
                         DECEL_STEP5_ATARGET)
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
#   if v_target < v_ego:      # slowing: lead the setpoint below v_target
#     desired = v_target - BRAKE_LEAD_GAIN * (v_ego - v_target)
#   else:                     # holding/accelerating: no lead
#     desired = v_target
#   desired = max(desired, min_setpoint)
#   err_kph = (desired - setpoint) * MS_TO_KPH
#   if abs(err_kph) < SETPOINT_DEADBAND_KPH: return None
#   if err_kph > 0:
#     if a_target <= 0: return None      # model veto, raise side only
#     return 'plus5' if err_kph >= STEP5_RAISE_KPH else 'plus1'
#   urgent = (-err_kph >= STEP5_LOWER_KPH) or (-a_target >= DECEL_STEP5_ATARGET)
#   floor_ok = (setpoint - 10.0 / MS_TO_KPH) >= min_setpoint
#   return 'minus5' if (urgent and floor_ok) else 'minus1'
#
# DECEL_STEP5_ATARGET (0.9) restores the previous production controller's
# aTarget-based urgency trigger, as a second, independent way to reach the
# minus5 branch -- NOT as an input to `desired`. aTarget anticipates the
# setpoint error (route 3d6 seg 26: a_target read -1.03 while the setpoint
# gap was still under a km/h), so it upgrades minus1 -> minus5 earlier than
# the error alone or the brake lead above can. The floor guard is unchanged
# and applies identically to both triggers.
#
# BRAKE_LEAD_GAIN (K=1.0) exists because tracking v_target exactly means the
# commanded gap (setpoint - v_ego) can never open faster than v_target itself
# falls -- route 3d6 seg 26 showed this crawling at minus1's rate while vEgo
# ran 9.6 km/h above v_target and the driver had to take over. The lead is
# one-sided: it only pulls desired below v_target when slowing, since
# undershooting while braking self-cancels as v_ego converges (safe), but
# leading above v_target while holding/accelerating would exceed the
# planner's target speed (unsafe).
#
# The two step5 thresholds are deliberately asymmetric:
#
# STEP5_RAISE_KPH (10.0) exists because a measured sample of isolated +-5
# commands landed a median of 2.00 ticks (never 1) at both 20 Hz and 40 Hz
# cadence -- the car's cruise module auto-repeats and can deliver a second
# tick after we stop transmitting. Overshoot on acceleration is the UNSAFE
# direction (the car ends above the planner's target speed), so plus5 is
# only used once two ticks' (10 km/h) worth of error exists -- a 2-tick
# landing from there does not overshoot desired.
#
# STEP5_LOWER_KPH (5.0) is lower than the raise threshold because overshoot
# on braking is the SAFE direction (the car ends slightly slower and
# self-corrects) -- route 3d5 showed the old single 10.0 km/h threshold left
# minus5 unused for 65 segments, capping braking slew at minus1's rate while
# vTarget dropped faster, producing up to +7.41 km/h of lag. Responsiveness
# is favored on this side. But a minus5's ~10 km/h (2-tick) move is not
# reflexively safe against the min-cruise floor the way a single tick is --
# see the floor guard below, which the v_target-based `desired` clamp alone
# does not provide (desired can equal min_setpoint exactly).

def cmd(a_target, v_ego, setpoint, v_target, min_setpoint=5.0):
  return select_cruise_command(a_target, v_ego, setpoint, v_target, min_setpoint)


def test_deadband_emits_nothing():
  v = 20.0
  assert cmd(0.0, v, setpoint=v, v_target=v + 0.2) is None


def test_accel_request_below_target_emits_plus():
  v = 20.0
  # 5 km/h of error -- below STEP5_RAISE_KPH, so plus1.
  assert cmd(+0.3, v, setpoint=v, v_target=v + 5.0 / MS_TO_KPH) == "plus1"


def test_raise_error_just_below_step5_threshold_uses_plus1():
  """Just under STEP5_RAISE_KPH (10 km/h) of error, a plus1 is used -- a
  plus5 here would risk a 2-tick landing overshooting past v_target, which is
  the unsafe direction when accelerating."""
  v = 20.0
  err_kph = STEP5_RAISE_KPH - 0.1
  assert cmd(+0.5, v, setpoint=v, v_target=v + err_kph / MS_TO_KPH) == "plus1"


def test_raise_error_at_or_above_step5_threshold_uses_plus5():
  """At or above STEP5_RAISE_KPH (10 km/h) of error, a plus5 is used -- a
  2-tick (10 km/h) landing cannot overshoot v_target from here."""
  v = 20.0
  assert cmd(+0.5, v, setpoint=v, v_target=v + STEP5_RAISE_KPH / MS_TO_KPH) == "plus5"
  err_kph = STEP5_RAISE_KPH + 0.1
  assert cmd(+0.5, v, setpoint=v, v_target=v + err_kph / MS_TO_KPH) == "plus5"


def test_small_setpoint_error_uses_step1():
  """A small (2 km/h) setpoint error stays well under STEP5_RAISE_KPH, so
  plus1 is used."""
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


def test_decel_error_just_below_step5_threshold_uses_step1():
  """Just under STEP5_LOWER_KPH (5 km/h) of error, a minus1 is used.

  With the brake lead, setpoint == v_ego means
  desired = v_ego - (1 + BRAKE_LEAD_GAIN) * (v_ego - v_target), so the delta
  that lands err_kph just under the threshold is smaller than the plain
  (v_ego - v_target) gap would suggest -- solve for it directly rather than
  assuming err_kph == v_ego - v_target."""
  v = 25.0
  err_kph = STEP5_LOWER_KPH - 0.1
  delta = err_kph / (MS_TO_KPH * (1 + BRAKE_LEAD_GAIN))
  assert cmd(-0.5, v, setpoint=v, v_target=v - delta, min_setpoint=5.0) == "minus1"


def test_decel_error_at_or_above_step5_threshold_uses_step5():
  """At or above STEP5_LOWER_KPH (5 km/h) of error, with ample room above
  min_setpoint, a minus5 is used."""
  v = 25.0
  assert cmd(-0.5, v, setpoint=v, v_target=v - STEP5_LOWER_KPH / MS_TO_KPH, min_setpoint=5.0) == "minus5"
  err_kph = STEP5_LOWER_KPH + 0.1
  assert cmd(-0.5, v, setpoint=v, v_target=v - err_kph / MS_TO_KPH, min_setpoint=5.0) == "minus5"


def test_atarget_trigger_alone_yields_minus5():
  """A small setpoint error (well under STEP5_LOWER_KPH) still fires minus5
  when a_target alone clears DECEL_STEP5_ATARGET. This is the point of the
  restored trigger: aTarget anticipates the speed error and upgrades minus1
  to minus5 before the setpoint error itself would."""
  v = 20.0
  assert cmd(-DECEL_STEP5_ATARGET, v, setpoint=v, v_target=v - 1.0 / MS_TO_KPH,
             min_setpoint=5.0) == "minus5"


def test_atarget_just_above_threshold_uses_minus1():
  """Just inside DECEL_STEP5_ATARGET (a_target=-0.89, magnitude 0.89 < 0.9),
  with the same small setpoint error, stays minus1 -- the trigger is a strict
  magnitude threshold, not a loose one."""
  v = 20.0
  assert cmd(-0.89, v, setpoint=v, v_target=v - 1.0 / MS_TO_KPH,
             min_setpoint=5.0) == "minus1"


def test_atarget_trigger_blocked_by_floor_guard():
  """aTarget alone clears the urgency trigger (-2.0, well past
  DECEL_STEP5_ATARGET), but the setpoint only has 8 km/h of room above
  min_setpoint -- less than the 10 km/h a minus5's 2-tick landing needs -- so
  minus1 is used instead, NOT minus5. The floor guard applies identically
  regardless of which trigger (setpoint error or aTarget) fired."""
  min_sp = 9.72
  setpoint = min_sp + 8.0 / MS_TO_KPH
  assert cmd(-2.0, 12.5, setpoint, 12.0, min_sp) == "minus1"


def test_atarget_trigger_does_not_bypass_deadband():
  """Even a strongly urgent a_target (-2.0) does not bypass the 1.0 km/h
  deadband: the aTarget trigger only upgrades a decrease we were already
  going to command (minus1 -> minus5), it does not create a command from
  nothing. err_kph here is -0.36, inside the deadband."""
  v = 20.0
  assert cmd(-2.0, v, setpoint=v, v_target=v - 0.05, min_setpoint=5.0) is None


def test_decel_step5_blocked_by_floor_guard():
  """err_kph clears STEP5_LOWER_KPH, but the setpoint only has 8 km/h of room
  above min_setpoint -- less than the 10 km/h a minus5's 2-tick landing
  needs -- so minus1 is used instead, NOT minus5. This is the floor guard the
  v_target-based `desired` clamp alone does not provide: `desired` can equal
  min_setpoint exactly, and a minus5 fired from just above that floor would
  still land two ticks and cross under it, risking DCC disengagement."""
  min_sp = 9.72
  setpoint = min_sp + 8.0 / MS_TO_KPH
  # v_target far below the floor so desired collapses to min_setpoint, and
  # err_kph is driven purely by (setpoint - min_setpoint) = 8 km/h >= 5.0.
  assert cmd(-0.8, 20.0, setpoint, 0.0, min_sp) == "minus1"


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
  safe in direction, so there is NO a_target gate on the braking side --
  unlike the old direction gate, a positive a_target (disagreeing with the
  speed error) must not block a minus command."""
  assert cmd(+2.0, 25.0, setpoint=25.0, v_target=5.0, min_setpoint=5.0) == "minus5"


# ---- brake lead (route 3d6 near-collision fix) ----
#
# Tracking v_target exactly means the commanded gap (setpoint - v_ego) can
# never open faster than v_target itself falls. On route 3d6 seg 26 the
# setpoint tracked v_target faithfully (error never exceeded 2 km/h), so
# minus5 was never eligible and braking crawled at minus1's ~3 km/h/s rate
# while vEgo ran up to 9.6 km/h above v_target -- the driver had to take
# over. BRAKE_LEAD_GAIN fixes this by leading the setpoint below v_target,
# in proportion to the speed error, only when slowing.

def test_route_3d6_seg26_near_collision_regression():
  """The actual field samples from the near-collision, min_setpoint=9.72
  (measured DCC enable floor). Under the old vTarget-follow law all four of
  these stayed at minus1 (error never exceeded 2 km/h against a bare
  v_target). With the brake lead, the last three clear STEP5_LOWER_KPH and
  command minus5, opening the gap fast enough that this braking event would
  no longer require driver takeover.

  The first sample now ALSO resolves to minus5, not minus1: a_target=-1.53
  clears DECEL_STEP5_ATARGET (0.9) on its own, via the aTarget urgency
  trigger reintroduced alongside the brake lead -- see
  test_route_3d6_seg26_atarget_urgency_regression below, which covers the
  full aTarget-vs-setpoint-error timeline for this incident."""
  min_sp = 9.72
  assert cmd(-1.53, 17.67, setpoint=17.22, v_target=17.03, min_setpoint=min_sp) == "minus5"
  assert cmd(-2.15, 17.42, setpoint=16.39, v_target=15.83, min_setpoint=min_sp) == "minus5"
  assert cmd(-2.09, 16.86, setpoint=15.00, v_target=14.67, min_setpoint=min_sp) == "minus5"
  assert cmd(-2.69, 16.14, setpoint=13.89, v_target=13.47, min_setpoint=min_sp) == "minus5"


def test_route_3d6_seg26_atarget_urgency_regression():
  """The aTarget urgency trigger (DECEL_STEP5_ATARGET) this change restores.
  Real seg-26 field samples, min_setpoint=9.72. aTarget anticipates the speed
  error: at t+35.5 the setpoint error was only ~1.3 km/h (well under
  STEP5_LOWER_KPH, so the brake lead alone would still say minus1) while
  a_target already read -1.03 -- over the 0.9 m/s2 threshold -- so this fires
  minus5 immediately. That is ~1.3s earlier than the setpoint-error trigger
  alone reaches minus5 in this same braking event (t+36.8 per the incident
  timeline), and the previous production controller fired at this same
  aTarget-driven point."""
  min_sp = 9.72
  assert cmd(-1.03, 17.81, setpoint=17.78, v_target=17.61, min_setpoint=min_sp) == "minus5"
  assert cmd(-1.53, 17.67, setpoint=17.22, v_target=17.03, min_setpoint=min_sp) == "minus5"
  assert cmd(-2.15, 17.42, setpoint=16.39, v_target=15.83, min_setpoint=min_sp) == "minus5"
  assert cmd(-2.69, 16.14, setpoint=13.89, v_target=13.47, min_setpoint=min_sp) == "minus5"


def test_no_lead_when_holding_or_accelerating():
  """When v_target >= v_ego (holding or speeding up), desired must equal
  v_target exactly -- no brake lead. Verified by placing the setpoint 0.5
  km/h short of v_target (inside the 1.0 km/h deadband): if any lead leaked
  into this branch, desired would shift and the deadband would no longer
  swallow the error."""
  for v_ego, v_target in [(20.0, 20.0), (20.0, 22.0), (10.0, 30.0)]:
    setpoint = v_target - 0.5 / MS_TO_KPH
    assert cmd(0.5, v_ego, setpoint, v_target, min_setpoint=5.0) is None


def test_brake_lead_is_proportional_to_speed_error():
  """The lead is proportional to the speed error. Pinning setpoint == v_target
  makes err_kph measure (desired - v_target) directly (the "amount desired
  sits below v_target"), which equals -BRAKE_LEAD_GAIN * (v_ego - v_target).
  Doubling (v_ego - v_target) must therefore exactly double that amount:
  delta1 is chosen to land just under STEP5_LOWER_KPH (minus1), and 2*delta1
  lands at exactly double the error, comfortably over the threshold
  (minus5) -- only possible if the lead scales linearly with the speed
  error."""
  v_target, min_sp = 15.0, 5.0
  setpoint = v_target
  delta1 = (STEP5_LOWER_KPH - 0.5) / (MS_TO_KPH * BRAKE_LEAD_GAIN)
  delta2 = 2 * delta1
  assert cmd(-0.5, v_target + delta1, setpoint, v_target, min_sp) == "minus1"
  assert cmd(-0.5, v_target + delta2, setpoint, v_target, min_sp) == "minus5"


def test_desired_floored_at_min_setpoint_despite_large_brake_lead():
  """A large lead can push v_target - BRAKE_LEAD_GAIN * (v_ego - v_target)
  far below min_setpoint (here, unclamped, desired would be 0.0 - 1.0 *
  (50.0 - 0.0) = -50.0 m/s). desired must still clamp to min_setpoint exactly
  like the plain v_target case, giving the same outcome as the no-lead floor
  guard scenario: only 8 km/h of room above the floor, less than the 10 km/h
  a minus5 2-tick landing needs, so minus1."""
  min_sp = 9.72
  setpoint = min_sp + 8.0 / MS_TO_KPH
  assert cmd(-0.8, 50.0, setpoint, 0.0, min_sp) == "minus1"


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
    ( 0.4, 10.0, 9.0,  15.0, 9.72, 'plus5'),
    ( 0.3, 20.0, 20.0, 25.0, 5.0,  'plus5'),
    (-0.5, 25.0, 25.0, 20.0, 5.0,  'minus5'),
])
def test_canonical_cases_unchanged(a_target, v_ego, setpoint, v_target, min_setpoint, expected):
  """Locks in the hand-verified reference cases from the vTarget-tracking
  spec. Two of the six (cases 4 and 5) now resolve to 'plus5' rather than the
  old always-plus1 result: both have >= STEP5_RAISE_KPH (10 km/h) of error
  and a_target > 0, which is exactly the new raise-side step5 condition. The
  other four cases are unaffected by the asymmetric-threshold change."""
  assert cmd(a_target, v_ego, setpoint, v_target, min_setpoint) == expected


# ---- acceleration: model veto only, no magnitude trust ----

def test_real_world_sluggish_case_now_accelerates():
  """The motivating field observation: car held at 80 km/h (v_ego=24.08 m/s)
  with a 97 km/h target (v_target=27.11 m/s) because a_target was a barely
  positive +0.015 -- under the old aTarget-driven setpoint, gap_for_accel of
  that tiny a_target put the setpoint only ~1.3 km/h above v_ego, below the
  1.0 km/h deadband, so nothing was ever sent. Tracking vTarget directly
  fixes this. The resulting error here (~9.6 km/h) is still under
  STEP5_RAISE_KPH, so it lands as plus1."""
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
  setpoint error must still produce a raise. Error is kept under
  STEP5_RAISE_KPH so this stays focused on the veto boundary rather than the
  step5 magnitude branch."""
  v = 20.0
  assert cmd(0.01, v, setpoint=v, v_target=v + 8.0 / MS_TO_KPH, min_setpoint=5.0) == 'plus1'


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

    1. A plus tick never carries the setpoint above desired (see below), and
       only moves 5 km/h (plus5) when the error is >= STEP5_RAISE_KPH, else
       1 km/h (plus1).
    2. A minus1 tick never carries the setpoint below desired (minus5 is
       exempt from this -- see below). A minus tick only moves 5 km/h
       (minus5) when (the error is >= STEP5_LOWER_KPH OR a_target magnitude
       is >= DECEL_STEP5_ATARGET) AND the setpoint has at least 10 km/h of
       room above min_setpoint (the floor guard), else 1 km/h (minus1).
    3. Acceleration (a plus command) never happens when a_target <= 0.
    4. (covered separately by test_nan_guards / test_inf_guard) non-finite
       input always returns None.

  Why minus5 is exempt from the "never undershoots desired" check that
  minus1 is held to: even before the aTarget trigger, a minus5 lands ~2
  ticks (10 km/h) in practice, not the nominal 5 km/h this sweep uses for
  the check -- see test_two_tick_landing_never_overshoots_target, which
  deliberately checks the 2-tick landing against min_setpoint only, not
  against desired, because overshooting past desired while braking is the
  documented SAFE direction (self-corrects as v_ego converges). The aTarget
  trigger makes this structural: it is designed to fire minus5 BEFORE the
  setpoint error grows to STEP5_LOWER_KPH, so by construction the
  setpoint-to-desired gap can be smaller than the nominal step at the moment
  it fires. That is anticipation working as intended, not an overshoot bug.

  desired mirrors select_cruise_command's own formula: when slowing
  (v_target < v_ego) it leads v_target down by BRAKE_LEAD_GAIN * (v_ego -
  v_target); otherwise it is v_target unchanged. Either way it is floored at
  min_setpoint.
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
            if v_target < v_ego:
              desired = v_target - BRAKE_LEAD_GAIN * (v_ego - v_target)
            else:
              desired = v_target
            desired = max(desired, min_sp)
            step_kph = 5.0 if result in ("plus5", "minus5") else 1.0
            step_ms = step_kph / MS_TO_KPH
            err_kph = (desired - setpoint) * MS_TO_KPH

            if result in ("plus1", "plus5"):
              if a_target <= 0:
                violations.append(("accel without a_target veto", a_target, v_ego, setpoint, v_target, min_sp, result))
              new_setpoint = setpoint + step_ms
              if new_setpoint > desired + 1e-9:
                violations.append(("plus overshoots desired", a_target, v_ego, setpoint, v_target, min_sp, result))
              if err_kph < SETPOINT_DEADBAND_KPH - 1e-9:
                violations.append(("plus emitted inside deadband", a_target, v_ego, setpoint, v_target, min_sp, result))
              if result == "plus5" and err_kph < STEP5_RAISE_KPH - 1e-9:
                violations.append(("plus5 used below step5 raise threshold", a_target, v_ego, setpoint, v_target, min_sp, result))
            elif result in ("minus1", "minus5"):
              new_setpoint = setpoint - step_ms
              # minus5 is deliberately exempt here -- see the docstring above:
              # both its real ~2-tick landing and the aTarget trigger's
              # anticipation are expected to land it below `desired`, which
              # is the documented-safe overshoot direction, not a violation.
              if result == "minus1" and new_setpoint < desired - 1e-9:
                violations.append(("minus1 undershoots desired", a_target, v_ego, setpoint, v_target, min_sp, result))
              if result == "minus5":
                urgent = (-err_kph >= STEP5_LOWER_KPH - 1e-9) or (-a_target >= DECEL_STEP5_ATARGET - 1e-9)
                if not urgent:
                  violations.append(("minus5 used without either urgency trigger", a_target, v_ego, setpoint, v_target, min_sp, result))
                if setpoint - 10.0 / MS_TO_KPH < min_sp - 1e-9:
                  violations.append(("minus5 used despite floor guard", a_target, v_ego, setpoint, v_target, min_sp, result))
            else:
              violations.append(("unexpected command", a_target, v_ego, setpoint, v_target, min_sp, result))

  assert checked == len(a_targets) * len(v_egos) * len(setpoints) * len(v_targets) * len(min_setpoints)
  assert emitted > 0, "sweep produced no emitted commands to check"
  assert not violations, f"{len(violations)} invariant violation(s), first few: {violations[:5]}"


def test_two_tick_landing_never_overshoots_target():
  """Overshoot-safety regression for the bug these thresholds fix: measured
  telemetry showed a +-5 typically lands 2 accepted ticks (median 2.00 at
  both cadences, never 1). This sweeps a grid of (setpoint, v_target,
  min_setpoint) and checks the 2-tick (10 km/h) landing of whichever of
  minus5/plus5 is emitted:

    - minus5: the landing must not cross min_setpoint (the DCC-disengage
      floor guard) -- this is deliberately checked against min_setpoint, not
      desired, since overshooting past v_target while braking is the SAFE
      direction and is not itself a violation.
    - plus5: the landing must not cross desired = max(v_target, min_setpoint)
      -- overshoot while accelerating is the UNSAFE direction.

  Both branches are reachable under the asymmetric thresholds (plus5 via
  STEP5_RAISE_KPH, minus5 via STEP5_LOWER_KPH + the floor guard), so this
  exercises both.
  """
  a_targets = [-1.5, -1.0, -0.5, -0.1, 0.0, 0.01, 0.2, 0.5, 2.0]
  v_egos = [6.0, 9.0, 11.0, 15.0, 20.0, 28.0]
  setpoints = [8.0, 9.2, 10.0, 12.0, 15.0, 20.0, 25.0]
  v_targets = [4.0, 8.0, 10.5, 15.0, 22.0, 30.0]
  min_setpoints = [5.0, 9.72]
  two_tick_ms = 10.0 / MS_TO_KPH   # a 2-tick landing moves 10 km/h, not 5

  checked_minus5 = 0
  checked_plus5 = 0
  for a_target in a_targets:
    for v_ego in v_egos:
      for setpoint in setpoints:
        for v_target in v_targets:
          for min_sp in min_setpoints:
            result = cmd(a_target, v_ego, setpoint, v_target, min_sp)
            desired = max(v_target, min_sp)
            if result == "minus5":
              checked_minus5 += 1
              assert setpoint - two_tick_ms >= min_sp - 1e-9, \
                  f"minus5 2-tick landing would cross min_setpoint={min_sp} from setpoint={setpoint}"
            elif result == "plus5":
              checked_plus5 += 1
              assert setpoint + two_tick_ms <= desired + 1e-9, \
                  f"plus5 2-tick landing would overshoot desired={desired} from setpoint={setpoint}"

  assert checked_minus5 > 0, "sweep produced no minus5 commands to check"
  assert checked_plus5 > 0, "sweep produced no plus5 commands to check"
