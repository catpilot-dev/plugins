"""Look Ahead Lateral Control — compute curvature from longer preview distance.

Stock openpilot uses curvature at t=actuator_delay (~0.5s), which amplifies
model prediction noise into steering oscillation. Human drivers look 2-5s
ahead, naturally smoothing out near-field noise.

This module recomputes desired curvature from the model's orientation
prediction at a longer lookahead time:
  lookahead_t = clamp(lookahead_dist / v_ego, MIN_T, MAX_T)

At 80 km/h with 50m lookahead: 2.25s ahead (vs stock 0.5s).
The model's far-field predictions are geometrically smoother because
road curves are large-radius at distance.

With a lead vehicle, lookahead distance is capped to the lead's distance —
like a human driver focusing on the car ahead rather than the horizon.

Also estimates steering angle offset on straight roads:
- Publishes to plugin bus topic 'steer_angle_offset' for carstate
- Persists to data/SteerAngleOffset across reboots
"""
import os
import numpy as np

_PLUGIN_DIR = os.path.dirname(os.path.abspath(__file__))
_DATA_DIR = os.path.join(_PLUGIN_DIR, 'data')

MIN_LOOKAHEAD_DIST = 20.0   # meters — floor for confidence-based distance
MAX_LOOKAHEAD_DIST = 100.0  # meters — cap even if model is very confident
CONFIDENCE_THRESHOLD = 0.6      # use model predictions up to where yStd confidence > 60%
MIN_LOOKAHEAD_T = 1.0       # seconds — floor at low speed
MAX_LOOKAHEAD_T = 3.0       # seconds — model reliability drops beyond ~5s
MIN_SPEED = 5.0             # m/s — below this, don't override
STRAIGHT_THRESHOLD = 0.002  # 1/m (~500m radius) — stock must be straight to activate
BLEND_RATE = 2.0            # blend factor change per second (0→1 in 0.5s)

# Longitudinal: confidence-based speed cap
PREVIEW_TIME = 3.0          # seconds — minimum forward visibility time at current speed
MAX_LAT_ACCEL_CAP = 1.5     # m/s² — comfortable lateral acceleration limit for speed cap
MIN_CAP_SPEED = 20.0        # km/h — don't cap below this (already crawling)
MAX_CAP_SPEED = 120.0       # km/h — above this, no cap needed

# Steering angle offset estimation
OFFSET_MIN_SPEED = 15.0     # m/s — only estimate on highway-like roads
OFFSET_MAX = 3.0            # deg — sanity clamp, real offsets are typically < 2°
OFFSET_BLOCK_DURATION = 5.0 # seconds — minimum consecutive straight driving per block
OFFSET_MIN_BLOCKS = 10      # blocks needed for a valid estimate
OFFSET_PUB_INTERVAL = 1.0   # seconds between plugin bus publishes

# T_IDXS from ModelConstants (copied to avoid import dependency)
_T_IDXS = [10.0 * (i / 32) ** 2 for i in range(33)]


def _curv_from_plan(yaws, yaw_rates, v_ego, action_t):
  """Same formula as openpilot's curv_from_psis but standalone."""
  v = max(v_ego, MIN_SPEED)
  psi_target = np.interp(action_t, _T_IDXS, yaws)
  psi_rate = yaw_rates[0]
  curv_from_psi = psi_target / (v * action_t)
  return 2 * curv_from_psi - psi_rate / v


_sm = None


_blend_factor = 0.0  # 0 = stock, 1 = look ahead
_blend_last_time = 0.0

# Longitudinal speed cap state
_speed_cap_pub = None


def _get_sm():
  """Lazy init SubMaster for radarState, carState, and deviceState."""
  global _sm
  if _sm is None:
    import cereal.messaging as messaging
    _sm = messaging.SubMaster(['radarState', 'carState', 'deviceState'])
  _sm.update(0)
  return _sm


def _get_lead_distance():
  """Get lead vehicle distance from radarState, or None if no lead."""
  try:
    sm = _get_sm()
    lead = sm['radarState'].leadOne
    if lead.status and lead.dRel > 5.0:
      return lead.dRel
  except Exception:
    pass
  return None


# --- Steering angle offset estimator ---
_offset_estimate = None
_block_samples = []        # current block being collected
_block_start = 0.0         # monotonic time when current block started
_block_medians = []        # median of each completed block this drive
_offset_last_pub = 0.0
_offset_pub = None
_prev_started = True


def _load_offset():
  global _offset_estimate
  if _offset_estimate is not None:
    return
  try:
    with open(os.path.join(_DATA_DIR, 'SteerAngleOffset')) as f:
      _offset_estimate = max(-OFFSET_MAX, min(OFFSET_MAX, float(f.read().strip())))
  except (FileNotFoundError, OSError, ValueError):
    _offset_estimate = 0.0


def _save_offset():
  """Compute median of block medians and save if enough valid blocks.

  Called once on onroad→offroad transition. Each block is 5+ seconds of
  consecutive straight highway driving. Requires 10 valid blocks (~50s total).
  Median-of-medians is robust to individual block outliers (road camber, wind).
  """
  global _offset_estimate, _block_medians, _block_samples
  _block_samples.clear()

  if len(_block_medians) < OFFSET_MIN_BLOCKS:
    _block_medians.clear()
    return

  new_offset = float(np.median(_block_medians))
  new_offset = max(-OFFSET_MAX, min(OFFSET_MAX, new_offset))
  _offset_estimate = new_offset
  _block_medians.clear()

  try:
    os.makedirs(_DATA_DIR, exist_ok=True)
    with open(os.path.join(_DATA_DIR, 'SteerAngleOffset'), 'w') as f:
      f.write('%.4f' % _offset_estimate)
  except OSError:
    pass


def _publish_offset(now):
  global _offset_last_pub, _offset_pub
  if now - _offset_last_pub < OFFSET_PUB_INTERVAL:
    return
  _offset_last_pub = now
  try:
    if _offset_pub is None:
      from openpilot.selfdrive.plugins.plugin_bus import PluginPub
      _offset_pub = PluginPub('steer_angle_offset')
    _offset_pub.send({'offset': _offset_estimate})
  except Exception:
    pass


def _update_offset_estimate(desired_curvature, v_ego):
  """Collect steering angle samples in consecutive blocks on straight roads.

  Each block requires 5+ seconds of uninterrupted straight highway driving.
  If the car enters a curve or slows down, the current block is discarded.
  At route end, the median of all block medians is the offset estimate.
  Requires 10 valid blocks (~50s of straight highway) to update.

  The offset is a physical sensor property — once converged it shouldn't
  change between drives.
  """
  global _prev_started, _block_samples, _block_start
  import time
  _load_offset()

  now = time.monotonic()

  # Detect onroad→offroad transition → compute median-of-medians and save
  try:
    sm = _get_sm()
    started = sm['deviceState'].started
    if _prev_started and not started:
      _save_offset()
    _prev_started = started
  except Exception:
    pass

  # Always publish current estimate (even if not updating)
  _publish_offset(now)

  # Check if conditions are met for sampling
  on_straight = v_ego >= OFFSET_MIN_SPEED and abs(desired_curvature) < 0.0005

  if not on_straight:
    # Conditions broken — discard current block
    _block_samples.clear()
    _block_start = 0.0
    return

  try:
    sm = _get_sm()
    # carState.steeringAngleDeg has the offset already subtracted,
    # so add it back to get the raw sensor value for estimation.
    angle = sm['carState'].steeringAngleDeg + _offset_estimate
  except Exception:
    return

  # Start new block
  if not _block_samples:
    _block_start = now

  _block_samples.append(angle)

  # Complete block after 5 seconds of consecutive data
  if now - _block_start >= OFFSET_BLOCK_DURATION:
    block_median = float(np.median(_block_samples))
    _block_medians.append(block_median)

    # Update live estimate once we have enough blocks
    if len(_block_medians) >= OFFSET_MIN_BLOCKS:
      _offset_estimate = float(np.median(_block_medians))
      _offset_estimate = max(-OFFSET_MAX, min(OFFSET_MAX, _offset_estimate))
    _block_samples.clear()
    _block_start = 0.0


# --- Curvature computation ---

def _confidence_distance(model_v2):
  """Find the farthest distance where model lateral confidence > threshold.

  Uses position.yStd from the model: confidence = 1 / (1 + yStd).
  """
  threshold = CONFIDENCE_THRESHOLD
  try:
    pos = model_v2.position
    x = pos.x
    yStd = pos.yStd
    if len(x) < 5 or len(yStd) < 5:
      return MIN_LOOKAHEAD_DIST
    # Walk backwards to find last point above threshold
    for i in range(len(x) - 1, -1, -1):
      conf = 1.0 / (1.0 + yStd[i])
      if conf > threshold:
        return max(MIN_LOOKAHEAD_DIST, min(MAX_LOOKAHEAD_DIST, x[i]))
  except (AttributeError, IndexError):
    pass
  return MIN_LOOKAHEAD_DIST


def compute_lookahead_curvature(model_v2, v_ego):
  """Compute curvature from model orientation at lookahead distance.

  Lookahead distance is dynamic based on model confidence,
  or lead vehicle distance if closer.
  Returns (curvature, lookahead_t) or (None, 0) if data unavailable.
  """
  if v_ego < MIN_SPEED:
    return None, 0

  lookahead_dist = _confidence_distance(model_v2)

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
    with open(os.path.join(_DATA_DIR, 'LookAheadEnabled')) as f:
      return f.read().strip() != '0'
  except (FileNotFoundError, OSError):
    return True  # default on


def _boundary_curvature(model_v2, conf_dist):
  """Estimate curvature at the confidence boundary distance."""
  try:
    pos = model_v2.position
    x = pos.x
    y = pos.y
    if len(x) < 5 or len(y) < 5:
      return 0.0
    import numpy as np
    px = np.array(x)
    py = np.array(y)
    idx = np.searchsorted(px, conf_dist)
    if idx < 2 or idx >= len(px) - 1:
      return 0.0
    dx = px[idx+1] - px[idx-1]
    dy = py[idx+1] - py[idx-1]
    d2y = py[idx+1] - 2*py[idx] + py[idx-1]
    if abs(dx) < 0.1:
      return 0.0
    dydx = dy / dx
    d2x = dx / 2
    d2ydx2 = d2y / (d2x * dx)
    return abs(d2ydx2) / (1 + dydx**2)**1.5
  except (AttributeError, IndexError):
    return 0.0


def _compute_speed_cap(model_v2, v_ego):
  """Compute safe speed from confidence distance and boundary curvature.

  Two signals:
  1. Visibility: don't drive faster than confidence_distance / PREVIEW_TIME
  2. Curvature at boundary: v_safe = sqrt(MAX_LAT_ACCEL / curvature)
  Returns speed cap in km/h, or 0 if no constraint.
  """
  if v_ego < MIN_SPEED:
    return 0

  conf_dist = _confidence_distance(model_v2)
  needed_dist = min(v_ego * PREVIEW_TIME, MAX_LOOKAHEAD_DIST)

  # No constraint if we can see far enough ahead (or at the model's range limit)
  if conf_dist >= needed_dist:
    return 0

  # Visibility-based cap
  vis_safe_ms = conf_dist / PREVIEW_TIME
  vis_safe_kph = vis_safe_ms * 3.6

  # Curvature at the confidence boundary
  boundary_curv = _boundary_curvature(model_v2, conf_dist)
  if boundary_curv > 0.001:
    curv_safe_ms = (MAX_LAT_ACCEL_CAP / boundary_curv) ** 0.5
    curv_safe_kph = curv_safe_ms * 3.6
    safe_kph = min(vis_safe_kph, curv_safe_kph)
  else:
    safe_kph = vis_safe_kph

  if safe_kph >= MAX_CAP_SPEED:
    return 0
  if safe_kph < MIN_CAP_SPEED:
    safe_kph = MIN_CAP_SPEED

  return int(safe_kph)


def _publish_speed_cap(speed_cap_kph):
  """Publish speed cap to speedlimitd via plugin bus."""
  global _speed_cap_pub
  try:
    if _speed_cap_pub is None:
      from openpilot.selfdrive.plugins.plugin_bus import PluginPub
      _speed_cap_pub = PluginPub('lookahead_speed_cap')
    _speed_cap_pub.send({'speed_cap': speed_cap_kph})
  except Exception:
    pass


LOOKAHEAD_T = 1.5  # seconds — fixed look-ahead for curvature smoothing

def on_curvature_correction(default_curvature, model_v2, v_ego, lane_changing, **kwargs):
  """Hook callback for controls.curvature_correction.

  Fixed 1.5s look-ahead: replaces stock curvature (at ~0.5s) with
  curvature from model trajectory at t=1.5s. Smooths model noise —
  at 120 km/h, 1.5s = 50m ahead, 3x further than stock.

  Applied always (straights, curves, lane changes). controlsd's
  clip_curvature handles safety limiting afterward.

  Also estimates steering angle offset and publishes speed cap.
  """
  # Offset estimation always runs
  _update_offset_estimate(default_curvature, v_ego)

  # Longitudinal: confidence-based speed cap
  if not lane_changing:
    speed_cap = _compute_speed_cap(model_v2, v_ego)
    _publish_speed_cap(speed_cap)
  else:
    _publish_speed_cap(0)

  if not _is_enabled() or v_ego < MIN_SPEED:
    return default_curvature

  # Only look ahead on straights — in curves, stock 0.5s is more accurate.
  # At curve exit, 1.5s would see the straight too early → cut the turn.
  if abs(default_curvature) > STRAIGHT_THRESHOLD:
    return default_curvature

  try:
    yaws = list(model_v2.orientation.z)
    yaw_rates = list(model_v2.orientationRate.z)
    if len(yaws) >= 20 and len(yaw_rates) >= 2:
      return float(_curv_from_plan(yaws, yaw_rates, v_ego, LOOKAHEAD_T))
  except (AttributeError, IndexError):
    pass

  return default_curvature
