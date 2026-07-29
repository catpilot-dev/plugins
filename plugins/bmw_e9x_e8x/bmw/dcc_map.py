"""Measured DCC response map and its inverse.

The car's acceleration is a function of the *setpoint gap*
(cruiseState.speed - vEgo), not of which stalk command produced it — see
docs/superpowers/specs/2026-07-29-dcc-response-findings.md. This module turns a
requested acceleration into the gap that delivers it.

Open-loop by design: nothing here consumes measured aEgo.
"""
from bmw.dcc_map_table import GAP_BPS, V_BPS, A_TABLE

MS_TO_KPH = 3.6                # local literal: this module must not import opendbc
SETPOINT_DEADBAND_KPH = 1.0    # below one tick there is nothing to send


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
  """Acceleration DCC is expected to produce at this gap and speed (m/s^2)."""
  return _interp(gap, GAP_BPS, _column(v_ego))


def accel_envelope(v_ego):
  """(a_min, a_max) reachable at this speed — DCC's authority limits."""
  col = _column(v_ego)
  return col[0], col[-1]


def gap_for_accel(a_target, v_ego):
  """Setpoint gap (m/s) that produces a_target, clamped to what DCC can do."""
  col = _column(v_ego)
  return _interp(_clamp(a_target, col[0], col[-1]), col, GAP_BPS)


def select_cruise_command(a_target, v_ego, setpoint, v_target, min_setpoint):
  """Which stalk command closes the gap between the current DCC setpoint and
  where the measured map says it should be. Returns a CruiseStalk member name
  or None.

  Open-loop on the map: measured aEgo is deliberately not an input.
  """
  desired = v_ego + gap_for_accel(a_target, v_ego)
  desired = min(desired, v_target)        # never target above the planner's speed
  desired = max(desired, min_setpoint)    # never strand the car below min cruise
  err_kph = (desired - setpoint) * MS_TO_KPH

  if abs(err_kph) < SETPOINT_DEADBAND_KPH:
    return None
  if err_kph > 0:
    return 'plus5' if err_kph >= 5.0 else 'plus1'
  # No separate min-speed headroom check is needed: `desired` is already floored
  # at min_setpoint, so a tick is only emitted when at least that step of error
  # exists above the floor, and it can never carry the setpoint under it.
  return 'minus5' if -err_kph >= 5.0 else 'minus1'
