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

LOOKAHEAD_DISTANCE = 50.0   # meters — reduced from 80m to avoid lane-line hugging
MIN_LOOKAHEAD_T = 1.0       # seconds — floor at low speed
MAX_LOOKAHEAD_T = 3.0       # seconds — model reliability drops beyond ~5s
MIN_SPEED = 5.0             # m/s — below this, don't override
CURVE_THRESHOLD = 0.002     # 1/m (~500m radius) — fall back to stock in curves

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
    _block_samples.clear()
    _block_start = 0.0


# --- Curvature computation ---

def compute_lookahead_curvature(model_v2, v_ego):
  """Compute curvature from model orientation at lookahead distance.

  Lookahead distance is 50m, or lead vehicle distance if closer.
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
    with open(os.path.join(_DATA_DIR, 'LookAheadEnabled')) as f:
      return f.read().strip() != '0'
  except (FileNotFoundError, OSError):
    return True  # default on


def on_curvature_correction(default_curvature, model_v2, v_ego, lane_changing):
  """Hook callback for controls.curvature_correction.

  Replaces the stock short-lookahead curvature with a longer-lookahead
  version. Falls back to stock curvature in curves (where lane centering
  needs the accurate short-lookahead) and during lane changes.

  Also estimates steering angle offset on straight roads.
  """
  # Offset estimation always runs (independent of Look Ahead toggle)
  _update_offset_estimate(default_curvature, v_ego)

  if not _is_enabled() or lane_changing:
    return default_curvature

  # In curves, fall back to stock — look ahead sees past the apex and
  # fights lane centering's correction for the current curve position.
  if abs(default_curvature) > CURVE_THRESHOLD:
    return default_curvature

  curvature, _ = compute_lookahead_curvature(model_v2, v_ego)
  if curvature is None:
    return default_curvature

  return curvature
