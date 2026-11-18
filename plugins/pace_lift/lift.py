"""Pace lift law: the lift target and its gate from one tick's inputs.

Pure: no I/O, no state, no openpilot imports. hook.py owns state, rates,
hard resets that depend on the car, the lead hold and telemetry.
See docs/superpowers/specs/2026-11-19-pace-lift-design.md (R3-R6).
"""
import math

STRENGTH_DEFAULT = {'aggressive': 0.40, 'standard': 0.20, 'relaxed': 0.0}   # m/s^2
T_FOLLOW = {'aggressive': 1.25, 'standard': 1.45, 'relaxed': 1.75}          # s

ASK_DECEL = -0.10            # m/s^2, a_e2e at or below this is the model braking
GAP_RAMP = (1.0, 4.0)        # m/s, v_cruise - v_ego -> [0, 1]
SPEED_FADE = (13.89, 16.67)  # m/s, v_ego -> [0, 1] (50-60 km/h)
ASK_CAP_RANGE = (-0.10, 0.0)  # m/s^2, a_e2e -> [0, 1]
PLAN_HORIZON_S = 6.0
PLAN_DECEL_MARGIN = 1.0      # m/s below v_ego
CURVE_LAT_ACCEL = 1.5        # m/s^2
LEAD_MARGIN_S = 0.5
LEAD_BASE_GAP = 6.0          # m
LEAD_CLOSING_VREL = -0.5     # m/s


def _ramp(x, lo, hi):
  return min(1.0, max(0.0, (x - lo) / (hi - lo)))


def ask_cap(a_e2e, strength):
  """Largest lift the current ask allows: a model easing off cuts the lift."""
  return strength * _ramp(a_e2e, *ASK_CAP_RANGE)


def lift_target(a_e2e, v_ego, v_cruise, strength, t_follow, plan_t, plan_v, plan_yaw_rate,
                should_stop, lead_present, lead_d, lead_vrel):
  """Return (target lift m/s^2, gate). target is 0 for every gate but 'active'."""
  try:
    scalars = (a_e2e, v_ego, v_cruise, strength, t_follow, lead_d, lead_vrel)
    if not all(math.isfinite(x) for x in scalars):
      return 0.0, 'no_model'
    n = len(plan_t)
    if n < 2 or len(plan_v) != n or len(plan_yaw_rate) != n:
      return 0.0, 'no_model'
    if not all(math.isfinite(x) for x in (*plan_t, *plan_v, *plan_yaw_rate)):
      return 0.0, 'no_model'
  except (TypeError, ValueError):
    return 0.0, 'no_model'

  if should_stop:
    return 0.0, 'stop'
  if a_e2e <= ASK_DECEL:
    return 0.0, 'ask_decel'

  idx = [i for i in range(n) if plan_t[i] <= PLAN_HORIZON_S]
  if not idx:
    return 0.0, 'no_model'
  if min(plan_v[i] for i in idx) < v_ego - PLAN_DECEL_MARGIN:
    return 0.0, 'plan_decel'
  if max(abs(plan_v[i] * plan_yaw_rate[i]) for i in idx) >= CURVE_LAT_ACCEL:
    return 0.0, 'curve'
  if lead_present:
    if lead_d < t_follow * v_ego + LEAD_BASE_GAP + LEAD_MARGIN_S * v_ego:
      return 0.0, 'lead_gap'
    if lead_vrel < LEAD_CLOSING_VREL:
      return 0.0, 'lead_closing'

  target = strength * _ramp(v_cruise - v_ego, *GAP_RAMP) * _ramp(v_ego, *SPEED_FADE)
  if target <= 0.0:
    return 0.0, 'none_needed'
  return target, 'active'
