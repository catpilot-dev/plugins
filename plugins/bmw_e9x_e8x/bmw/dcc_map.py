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
STEP5_THRESHOLD_KPH = 10.0     # a +-5 command typically lands 2 ticks (measured
                                # median 2.00, 96% >= 2), so only use it when at
                                # least two ticks of error exist


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
  only reduce the commanded speed, so it is always safe.
  """
  # Guard against non-finite inputs (NaN or +/-inf)
  if any(not math.isfinite(x) for x in [a_target, v_ego, setpoint, v_target, min_setpoint]):
    return None

  desired = max(v_target, min_setpoint)          # never strand below min cruise
  err_kph = (desired - setpoint) * MS_TO_KPH

  if abs(err_kph) < SETPOINT_DEADBAND_KPH:
    return None

  if err_kph > 0:                                # raise the setpoint
    if a_target <= 0:
      return None                                # model veto: do not speed up against it
    return 'plus1'                                # plus1 only, 20 Hz -- smooth
  # lower the setpoint: always safe, it can only reduce commanded speed
  # +-1 is preferred below STEP5_THRESHOLD_KPH: a short burst lands ~1 tick,
  # while +-5 lands ~2 ticks (measured median 2.00, 96% >= 2), so +-5 is only
  # used once at least two ticks of error exist.
  return 'minus5' if -err_kph >= STEP5_THRESHOLD_KPH else 'minus1'
