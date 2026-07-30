"""Measured DCC response map and its inverse.

The car's acceleration is a function of the *setpoint gap*
(cruiseState.speed - vEgo), not of which stalk command produced it — see
docs/superpowers/specs/2026-07-29-dcc-response-findings.md. This module turns a
requested acceleration into the gap that delivers it.

Open-loop by design: nothing here consumes measured aEgo.
"""
import math

from bmw.dcc_map_table import GAP_BPS, V_BPS, A_TABLE

MS_TO_KPH = 3.6                # local literal: this module must not import opendbc
SETPOINT_DEADBAND_KPH = 1.0    # below one tick there is nothing to send
V_ERROR_DEADZONE = 0.5 / 3.6   # m/s (~0.5 km/h) — speed-error direction deadzone
ACCEL_TRIGGER_KPH = 1.0        # speed shortfall that puts us in acceleration mode


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

  Split into two branches because aTarget cannot be trusted symmetrically.
  Upstream's longitudinal_planner.py takes
  min(output_a_target_e2e, output_a_target_mpc), so the noisy vision model
  can only ever VETO acceleration, never braking — vTarget, by contrast,
  comes from the MPC alone. Measured against what the plant gain implies
  from the speed error, aTarget is only ~0.13x the needed value when
  accelerating but ~3.0-3.3x when braking. Deriving the setpoint from aTarget
  therefore makes acceleration hopeless (observed: car held at 80 km/h with
  a 97 km/h target because aTarget was +0.015). So acceleration is driven
  from vTarget directly (aTarget only vetoes it), while braking keeps using
  aTarget, where it is conservative and safe.

  Open-loop on the map: measured aEgo is deliberately not an input.
  """
  # Guard against non-finite inputs (NaN or +/-inf)
  if any(not math.isfinite(x) for x in [a_target, v_ego, setpoint, v_target, min_setpoint]):
    return None

  v_error = v_target - v_ego

  if v_error * MS_TO_KPH > ACCEL_TRIGGER_KPH:
    # ACCELERATION: trust the MPC's speed target for the destination (aTarget
    # is only ~0.13x the needed value here, per the module docstring, so it
    # cannot supply the magnitude). aTarget still has to agree in SIGN before
    # we accelerate -- the vision model retains its veto over a positive-sign
    # command -- so a_target must be strictly positive, matching what the
    # previous production controller required (`accel > 0`). Measured cost of
    # this gate: 70.7% duty cycle on real data, blocked stretches median
    # 0.31 s; since DCC accepts only ~3 ticks/s while we offer 20 commands/s,
    # the climb rate is essentially unaffected. Only ever 'plus1' -- never
    # 'plus5' -- both because plus1 at 20 Hz gives smoother acceleration and
    # because this branch may need to close a large gap gradually rather than
    # in one jump.
    if a_target <= 0:
      return None
    err_kph = (v_target - setpoint) * MS_TO_KPH
    if err_kph < SETPOINT_DEADBAND_KPH:
      return None
    # Can never overshoot v_target: a plus1 tick moves exactly 1 km/h and is
    # only emitted while err_kph >= 1.0 (i.e. setpoint + 1 km/h <= v_target).
    return 'plus1'

  # BRAKING / HOLDING: unchanged from before the acceleration split above.
  desired_raw = min(v_ego + gap_for_accel(a_target, v_ego), v_target)
  desired = max(desired_raw, min_setpoint)
  err_kph = (desired - setpoint) * MS_TO_KPH

  if abs(err_kph) < SETPOINT_DEADBAND_KPH:
    return None
  # The min-speed floor may hold the setpoint up, but it must never RAISE it
  # past either of two ceilings — neither guard below implies the other:
  #  - desired_raw < setpoint: the planner has already asked for something
  #    lower than what's currently commanded, so an upward tick would fight it.
  #  - desired > v_target: the floor has pushed the commanded setpoint past
  #    the planner's target speed (this happens when v_target < min_setpoint
  #    and the current setpoint is already at or below v_target, so
  #    desired_raw == v_target is not < setpoint, yet the floor clamp still
  #    lifts `desired` above v_target).
  if err_kph > 0 and (desired_raw < setpoint or desired > v_target):
    return None

  # Direction gate: the smooth speed error (v_target - v_ego) decides the
  # DIRECTION of the command; the noisy a_target only shapes the magnitude via
  # the map above. Requiring both signals to agree before emitting anything is
  # what the previous production controller did, and it's what keeps modelV2
  # noise (a_target reverses sign far more often than the speed error) from
  # manufacturing spurious commands.
  #
  # The two branches are deliberately asymmetric, both pivoting on the same
  # +V_ERROR_DEADZONE threshold: acceleration requires the speed error to
  # CLEARLY call for speeding up (v_error > V_ERROR_DEADZONE); braking only
  # requires that it is NOT clearly calling for speeding up
  # (v_error < V_ERROR_DEADZONE). Measurement showed the symmetric gate
  # blocked 75.5% of wanted braking commands, leaving the setpoint at or
  # above vEgo in 94% of those cases — the car physically cannot decelerate
  # from there. Favouring braking is the safe direction: a wanted brake that
  # gets suppressed is worse than one that fires slightly early.
  if err_kph > 0:
    if not (v_error > V_ERROR_DEADZONE and a_target > 0):
      return None
    return 'plus5' if err_kph >= 5.0 else 'plus1'
  if not (v_error < V_ERROR_DEADZONE and a_target < 0):
    return None
  # No separate min-speed headroom check is needed: `desired` is already floored
  # at min_setpoint, so a tick is only emitted when at least that step of error
  # exists above the floor, and it can never carry the setpoint under it.
  return 'minus5' if -err_kph >= 5.0 else 'minus1'
