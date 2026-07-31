"""Measured DCC response map, and the setpoint-tracking controller that drives
the cruise stalk.

The car's acceleration is a function of the *setpoint gap*
(cruiseState.speed - vEgo), not of which stalk command produced it — see
docs/superpowers/specs/2026-07-29-dcc-response-findings.md.

select_cruise_command no longer inverts this map to get a target acceleration.
aTarget is unreliable for magnitude in BOTH directions (upstream's
`a_target = min(e2e_model, mpc)` understates acceleration ~87% and overstates
braking ~3x), the achievable deceleration is DCC's to decide, not ours, and
DCC's response latency is out of our control too. The controller's only job
is to track the cruise setpoint to vTarget -- the smooth MPC signal, and the
only thing we can actually chase.

expected_accel, gap_for_accel, accel_envelope, and the dcc_map_table import
below are retained for OFFLINE ANALYSIS ONLY (tools/dcc_study/) and are no
longer part of the control path.
"""
import math

from bmw.dcc_map_table import GAP_BPS, V_BPS, A_TABLE

MS_TO_KPH = 3.6                # local literal: this module must not import opendbc
SETPOINT_DEADBAND_KPH = 1.0    # below one tick there is nothing to send
STEP5_RAISE_KPH = 10.0         # a +-5 command typically lands 2 ticks (measured
                                # median 2.00, 96% >= 2). Raising must not overshoot
                                # v_target (unsafe direction), so plus5 is only used
                                # once two ticks of error exist.
STEP5_LOWER_KPH = 5.0          # braking overshoot is the safe direction (car ends
                                # slightly slower, self-corrects), so the threshold
                                # for lowering is relaxed for responsiveness -- but
                                # see the floor guard in select_cruise_command below.
BRAKE_LEAD_GAIN = 1.0          # when slowing, lead the setpoint BELOW vTarget in
                                # proportion to the speed error, so the commanded gap
                                # becomes (1 + K) * (v_target - v_ego). K=1.0 was chosen
                                # from route data: it leaves normal cruising untouched
                                # (median commanded gap unchanged) while the 1st-percentile
                                # gap reaches -11.9 km/h, which is exactly where DCC
                                # saturates (~-1.2 m/s2). K=1.5 overshoots that for nothing.
DECEL_STEP5_ATARGET = 0.9      # m/s2 — the previous production controller's
                                # DECEL_STEP5_THRESHOLD. aTarget anticipates the speed
                                # error, so it upgrades minus1 -> minus5 earlier than
                                # the setpoint error alone can. Measured cost: only
                                # ~1.1% additional minus5 firings in normal driving.


def _clamp(x, lo, hi):
  return lo if x < lo else (hi if x > hi else x)


def _interp(x, xs, ys):
  x = _clamp(x, xs[0], xs[-1])
  for i in range(len(xs) - 1):
    if x <= xs[i + 1]:
      x0, x1 = xs[i], xs[i + 1]
      y0, y1 = ys[i], ys[i + 1]
      if x1 == x0:
        return y0
      return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
  return ys[-1]


def _column(v_ego):
  """Response curve vs gap at this speed (interpolated across V_BPS)."""
  return [_interp(v_ego, V_BPS, [A_TABLE[i][j] for j in range(len(V_BPS))])
          for i in range(len(GAP_BPS))]


def expected_accel(gap, v_ego):
  """Acceleration DCC is expected to produce at this gap and speed (m/s^2).

  OFFLINE ANALYSIS ONLY -- not used by select_cruise_command.
  """
  return _interp(gap, GAP_BPS, _column(v_ego))


def accel_envelope(v_ego):
  """(a_min, a_max) reachable at this speed — DCC's authority limits.

  OFFLINE ANALYSIS ONLY -- not used by select_cruise_command.
  """
  col = _column(v_ego)
  return col[0], col[-1]


def gap_for_accel(a_target, v_ego):
  """Setpoint gap (m/s) that produces a_target, clamped to what DCC can do.

  OFFLINE ANALYSIS ONLY -- not used by select_cruise_command.
  """
  col = _column(v_ego)
  return _interp(_clamp(a_target, col[0], col[-1]), col, GAP_BPS)


def select_cruise_command(a_target, v_ego, setpoint, v_target, min_setpoint):
  """Which stalk command closes the gap between the current DCC setpoint and
  vTarget. Returns a CruiseStalk member name or None.

  The controller's only job is to track the cruise setpoint to vTarget, the
  smooth MPC signal -- not to invert an acceleration map. aTarget is
  unreliable for magnitude in both directions (upstream's
  min(e2e_model, mpc) understates acceleration ~87% and overstates braking
  ~3x, per docs/superpowers/specs/2026-07-29-dcc-response-findings.md), the
  achievable deceleration is DCC's to decide, not ours, and DCC's response
  latency is out of our control too.

  aTarget is therefore used only as a sign veto on the acceleration side: it
  can block a raise but never supplies its magnitude. There is deliberately
  NO veto on the braking side -- lowering the setpoint toward vTarget can
  only reduce the commanded speed, so it is always safe. aTarget IS used on
  the braking side, but only as a second urgency trigger selecting step size
  (minus1 vs minus5, see DECEL_STEP5_ATARGET below) -- never as an input to
  `desired`, which stays vTarget plus the brake lead regardless of aTarget.

  Tracking vTarget exactly, however, means the commanded gap (setpoint -
  v_ego) can never open faster than vTarget itself falls -- on route 3d6 seg
  26 this left the car braking at minus1's ~3 km/h/s crawl while vEgo ran up
  to 9.6 km/h above vTarget, and the driver had to take over. So when slowing
  (v_target < v_ego), the setpoint leads vTarget DOWNWARD in proportion to
  the speed error (BRAKE_LEAD_GAIN), opening the gap faster than vTarget can
  fall on its own. This lead is deliberately one-sided: undershooting
  vTarget while braking merely brakes harder and self-cancels as v_ego
  converges (safe), whereas leading ABOVE vTarget while holding/accelerating
  would make the car exceed the planner's target speed (unsafe) -- so there
  is no lead on that side, only the plain v_target follow.
  """
  # Guard against non-finite inputs (NaN or +/-inf)
  if any(not math.isfinite(x) for x in [a_target, v_ego, setpoint, v_target, min_setpoint]):
    return None

  if v_target < v_ego:                       # slowing: lead the setpoint down
    desired = v_target - BRAKE_LEAD_GAIN * (v_ego - v_target)
  else:                                      # holding or speeding up: no lead,
    desired = v_target                       # overshooting ABOVE v_target is unsafe
  desired = max(desired, min_setpoint)        # never strand below min cruise
  err_kph = (desired - setpoint) * MS_TO_KPH

  if abs(err_kph) < SETPOINT_DEADBAND_KPH:
    return None

  if err_kph > 0:                                # raise the setpoint
    if a_target <= 0:
      return None                                # model veto: do not speed up against it
    # plus5 lands ~2 ticks like minus5 (measured), and overshoot is UNSAFE
    # when accelerating (car would end above v_target), so it is only used
    # once two ticks of error exist -- same reasoning as the lower branch,
    # just without the floor guard (there is no upper-side analogue of
    # min_setpoint to protect against).
    return 'plus5' if err_kph >= STEP5_RAISE_KPH else 'plus1'

  # lower the setpoint: always safe in magnitude, it can only reduce commanded
  # speed -- but a minus5 typically moves 10 km/h (2-tick landing), so we only
  # permit it when the setpoint has at least that much room above
  # min_setpoint. This protects against DCC disengaging on a too-low setpoint,
  # which the v_target-based `desired` clamp alone does not guarantee: desired
  # can equal min_setpoint exactly, and a minus5 fired from just above that
  # floor would still land two ticks and cross under it.
  #
  # urgent is an OR of two independent triggers: the setpoint error (as
  # before) and aTarget magnitude. aTarget anticipates the setpoint error --
  # on route 3d6 seg 26 it read -1.03 m/s2 while the setpoint gap was still
  # under a km/h, well before STEP5_LOWER_KPH could fire -- so it upgrades
  # minus1 -> minus5 earlier when the model already sees hard braking coming.
  # This is a trigger only: it selects step SIZE, never `desired` (the
  # setpoint destination), which stays vTarget plus the brake lead above.
  # The floor guard applies identically regardless of which trigger fired --
  # a minus5 lands ~2 ticks (~10 km/h) whatever provoked it, so the same
  # floor headroom is required either way.
  urgent = (-err_kph >= STEP5_LOWER_KPH) or (-a_target >= DECEL_STEP5_ATARGET)
  floor_ok = (setpoint - 10.0 / MS_TO_KPH) >= min_setpoint
  return 'minus5' if (urgent and floor_ok) else 'minus1'
