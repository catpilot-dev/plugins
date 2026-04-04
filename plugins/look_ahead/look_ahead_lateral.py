"""Look Ahead Lateral Control — compute curvature from longer preview distance.

Stock openpilot uses curvature at t=actuator_delay (~0.5s), which amplifies
model prediction noise into steering oscillation. Human drivers look 2-5s
ahead, naturally smoothing out near-field noise.

This module recomputes desired curvature from the model's orientation
prediction at a longer lookahead time:
  lookahead_t = clamp(lookahead_dist / v_ego, MIN_T, MAX_T)

At 80 km/h with 80m lookahead: 3.6s ahead (vs stock 0.5s).
The model's far-field predictions are geometrically smoother because
road curves are large-radius at distance.

With a lead vehicle, lookahead distance is capped to the lead's distance —
like a human driver focusing on the car ahead rather than the horizon.
"""
import numpy as np

LOOKAHEAD_DISTANCE = 80.0   # meters — model is reliable up to ~100m
MIN_LOOKAHEAD_T = 1.0       # seconds — floor at low speed
MAX_LOOKAHEAD_T = 4.0       # seconds — model reliability drops beyond ~5s
MIN_SPEED = 5.0             # m/s — below this, don't override

# T_IDXS from ModelConstants (copied to avoid import dependency)
_T_IDXS = [10.0 * (i / 32) ** 2 for i in range(33)]


def _curv_from_plan(yaws, yaw_rates, v_ego, action_t):
  """Same formula as openpilot's curv_from_psis but standalone."""
  v = max(v_ego, MIN_SPEED)
  psi_target = np.interp(action_t, _T_IDXS, yaws)
  psi_rate = yaw_rates[0]
  curv_from_psi = psi_target / (v * action_t)
  return 2 * curv_from_psi - psi_rate / v


_radar_sm = None


def _get_lead_distance():
  """Get lead vehicle distance from radarState, or None if no lead."""
  global _radar_sm
  try:
    if _radar_sm is None:
      import cereal.messaging as messaging
      _radar_sm = messaging.SubMaster(['radarState'])
    _radar_sm.update(0)
    lead = _radar_sm['radarState'].leadOne
    if lead.status and lead.dRel > 5.0:
      return lead.dRel
  except Exception:
    pass
  return None


def compute_lookahead_curvature(model_v2, v_ego):
  """Compute curvature from model orientation at lookahead distance.

  Lookahead distance is 80m, or lead vehicle distance if closer.
  Returns (curvature, lookahead_t) or (None, 0) if data unavailable.
  """
  if v_ego < MIN_SPEED:
    return None, 0

  lookahead_dist = LOOKAHEAD_DISTANCE
  lead_dist = _get_lead_distance()
  if lead_dist is not None and lead_dist < lookahead_dist:
    lookahead_dist = lead_dist

  lookahead_t = lookahead_dist / v_ego
  lookahead_t = max(MIN_LOOKAHEAD_T, min(MAX_LOOKAHEAD_T, lookahead_t))

  try:
    yaws = list(model_v2.orientation.z)
    yaw_rates = list(model_v2.orientationRate.z)
  except (AttributeError, IndexError):
    return None, 0

  if len(yaws) < 20 or len(yaw_rates) < 2:
    return None, 0

  curvature = _curv_from_plan(yaws, yaw_rates, v_ego, lookahead_t)
  return float(curvature), lookahead_t


def _is_enabled():
  try:
    import os
    _dir = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(_dir, 'data', 'LookAheadEnabled')) as f:
      return f.read().strip() != '0'
  except (FileNotFoundError, OSError):
    return True  # default on


def on_curvature_correction(default_curvature, model_v2, v_ego, lane_changing):
  """Hook callback for controls.curvature_correction.

  Replaces the stock short-lookahead curvature with a longer-lookahead
  version. During lane changes, falls back to stock to preserve responsiveness.
  """
  if not _is_enabled() or lane_changing:
    return default_curvature

  curvature, _ = compute_lookahead_curvature(model_v2, v_ego)
  if curvature is None:
    return default_curvature

  return curvature
