#!/usr/bin/env python3
"""
Speed Limit Middleware — merges mapd, YOLO, and road-type inference
into a single SpeedLimitState message at 5 Hz.

Three-tier priority:
  1. YOLO speed sign detection (direct sign reading, highest confidence)
  2. mapd suggestedSpeed (comprehensive: visionCurveSpeed + speed limit + road type)
  3. Vision-inferred speed (lane count + road type, own fallback when mapd has no data)
"""
import math
import os
import time
import tomllib
import cereal.messaging as messaging
from openpilot.common.realtime import Ratekeeper

# Load speed tables from per-country TOML files
SPEED_TABLES_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'speed_tables')


def load_speed_table(country: str) -> tuple[dict, dict, int, list]:
  """Load urban/nonurban speed tables, fallback, and lane_width class table.

  Returns (urban_table, nonurban_table, default_fallback, lane_width_class).
  lane_width_class is a list of {'min': float, 'type': str} dicts sorted by
  `min` descending (so the first match on lane_width ≥ min wins).
  """
  path = os.path.join(SPEED_TABLES_DIR, f'{country}.toml')
  with open(path, 'rb') as f:
    data = tomllib.load(f)

  urban = {k: dict(v) for k, v in data.get('urban', {}).items()}
  nonurban = {k: dict(v) for k, v in data.get('nonurban', {}).items()}
  fallback = data.get('default_fallback', 40)
  lane_width_class = sorted(
    [dict(e) for e in data.get('lane_width_class', []) if 'min' in e and 'type' in e],
    key=lambda e: e['min'], reverse=True,
  )
  return urban, nonurban, fallback, lane_width_class


def classify_by_width(lane_width: float, table: list) -> str:
  """Pick a road-type hint from observed lane_width via the cn.toml table.

  Returns '' if no table entry matches (unconfigured country) or width is
  non-positive.
  """
  if lane_width <= 0.0 or not table:
    return ''
  for entry in table:
    if lane_width >= entry['min']:
      return entry['type']
  return ''


def load_country_bboxes() -> list[tuple[str, list]]:
  """Load bounding boxes from all country TOML files.

  Returns list of (country_code, [min_lat, max_lat, min_lon, max_lon]).
  """
  bboxes = []
  for fname in os.listdir(SPEED_TABLES_DIR):
    if not fname.endswith('.toml'):
      continue
    with open(os.path.join(SPEED_TABLES_DIR, fname), 'rb') as f:
      data = tomllib.load(f)
    bbox = data.get('bbox')
    if bbox and len(bbox) == 4:
      bboxes.append((fname[:-5], bbox))
  return bboxes


def country_from_gps(lat: float, lon: float, bboxes: list) -> str | None:
  """Match lat/lon to a country code via bounding box lookup."""
  for code, (min_lat, max_lat, min_lon, max_lon) in bboxes:
    if min_lat <= lat <= max_lat and min_lon <= lon <= max_lon:
      return code
  return None


# Default to China; overridden by GPS auto-detection at runtime
SPEED_TABLE_URBAN, SPEED_TABLE_NONURBAN, DEFAULT_FALLBACK_SPEED, LANE_WIDTH_CLASS_TABLE = load_speed_table('cn')

# Standard speed limit values used in China (GB 5768)
_STANDARD_SPEEDS = [30, 40, 50, 60, 80, 100, 120]


def snap_to_standard_speed(speed: int) -> int:
  """Snap a computed speed to the nearest standard speed limit value.

  mapd visionCurveSpeed produces raw values like 47, 75, 83 km/h.
  Speed limit signs always display standard values, so we snap for
  clean display and consistent planner behaviour.
  """
  return min(_STANDARD_SPEEDS, key=lambda s: abs(s - speed))


# Gradual transition timing (seconds per step)
_STEP_DOWN_INTERVAL = 3.0  # downgrade: 80 → 60 → 50 → 40 (3s per step)
_STEP_UP_INTERVAL = 2.0    # upgrade:   40 → 50 → 60 → 80 (2s per step)


def _step_speed_limit(current: int, target: int) -> int:
  """Move current one step toward target in _STANDARD_SPEEDS.

  Returns the next standard speed in the direction of target,
  or target itself if already adjacent or equal.
  """
  if current == target or current == 0:
    return target

  if target < current:
    # Step down: find the next lower standard speed
    lower = [s for s in _STANDARD_SPEEDS if s < current]
    return max(lower) if lower else target
  else:
    # Step up: find the next higher standard speed
    higher = [s for s in _STANDARD_SPEEDS if s > current]
    return min(higher) if higher else target


def _near_road_edge(model_msg) -> tuple[bool, bool]:
  """Check if the car is near the left or right road edge.

  When the outermost visible lane line is close to the road edge (within
  one lane width ~3.5m) and the road edge is detected with high confidence
  (low std), the car is in an edge lane and vision likely undercounts by 1.

  Returns (near_left_edge, near_right_edge).
  """
  if not hasattr(model_msg, 'roadEdges') or len(model_msg.roadEdges) < 2:
    return False, False
  if not hasattr(model_msg, 'roadEdgeStds') or len(model_msg.roadEdgeStds) < 2:
    return False, False

  probs = model_msg.laneLineProbs
  re_stds = model_msg.roadEdgeStds
  EDGE_STD_THRESH = 0.5  # confident road edge detection
  LANE_WIDTH = 3.5  # meters — gap between outermost line and edge must be < 1 lane

  # y positions at ~10m ahead (index 2)
  try:
    ll_y = [model_msg.laneLines[i].y[2] for i in range(4)]
    re_y = [model_msg.roadEdges[i].y[2] for i in range(2)]
  except (IndexError, AttributeError):
    return False, False

  # Left edge: use leftmost visible lane line (index 0 if visible, else 1)
  left_line_idx = 0 if probs[0] > 0.3 else 1
  near_left = (re_stds[0] < EDGE_STD_THRESH and probs[left_line_idx] > 0.3 and
               abs(ll_y[left_line_idx] - re_y[0]) < LANE_WIDTH)

  # Right edge: use rightmost visible lane line (index 3 if visible, else 2)
  right_line_idx = 3 if probs[3] > 0.3 else 2
  near_right = (re_stds[1] < EDGE_STD_THRESH and probs[right_line_idx] > 0.3 and
                abs(re_y[1] - ll_y[right_line_idx]) < LANE_WIDTH)

  return near_left, near_right


def infer_lane_count(model_msg) -> int:
  """Infer lane count from modelV2 laneLineProbs and roadEdges.

  The model outputs 4 lane lines (indices 0-3) and 2 road edges.
  Lane lines form lane boundaries; N visible lines = up to N-1 lanes
  on the visible side of the road.

  When the car is in an edge lane (close to a road edge), the far side
  of the road is harder to see, so we boost the count by 1 to compensate
  for the likely unseen lane(s) on the opposite side.

  Returns estimated total lane count (1-6+).
  """
  if not hasattr(model_msg, 'laneLineProbs') or len(model_msg.laneLineProbs) < 4:
    return 1

  probs = model_msg.laneLineProbs
  # Count lane lines with reasonable confidence
  visible_lines = sum(1 for p in probs if p > 0.3)

  # visible_lines → lane estimate:
  #   4 lines = 3 lane gaps visible, likely 4+ lane road
  #   3 lines = 2 lane gaps, likely 3-4 lane road
  #   2 lines (inner pair) = our lane + neighbors, at least 2 lanes
  #   1 or 0 = single lane
  if visible_lines >= 4:
    base_count = 4
  elif visible_lines >= 3:
    base_count = 3
  elif visible_lines >= 2:
    base_count = 2
  else:
    base_count = 1

  # Edge lane boost: if the car is next to a road edge, vision likely
  # misses a lane on the far side. Boost by 1, capped at 4.
  if base_count >= 2:
    near_left, near_right = _near_road_edge(model_msg)
    if near_left or near_right:
      base_count = min(base_count + 1, 4)

  return base_count


# --- Lateral-acceleration params (2026-07-28 layer contract) ---------------
# speedlimitd owns lateral acceleration via vEgo. Two knobs, both read from the
# persisted plugin param store (same store mapd_runner uses) and parsed as a raw
# m/s² value per the README contract:
#   MapdCurveTargetLatAccel — proactive vision-curve target a_y (default 1.5)
#   MapdReactLatAccel       — reactive measured-a_y threshold (default 2.5, 0=off)
CURVE_LAT_ACCEL_DEFAULT = 1.5
CURVE_LAT_ACCEL_MIN = 1.0
CURVE_LAT_ACCEL_MAX = 3.0
REACT_LAT_ACCEL_DEFAULT = 2.5
REACT_LAT_ACCEL_MIN = 1.8
REACT_LAT_ACCEL_MAX = 3.0

# Reactive measured-a_y cap tuning.
REACT_TAU = 0.3            # s — low-pass time constant on measured a_y
REACT_ENGAGE_S = 0.5      # s — |a_y| must exceed threshold this long to engage
REACT_QUIET_S = 2.0       # s — |a_y| must stay quiet this long before release
REACT_QUIET_MARGIN = 0.3  # m/s² — release deadband below threshold
REACT_HYST_MS = 1.0       # m/s — extra reduction below the sqrt() cap
REACT_RELEASE_RATE = 1.0  # m/s per s — cap ramp-up rate during release


def _parse_lat_accel(raw, default: float, lo: float, hi: float,
                     zero_disables: bool = False) -> float:
  """Parse a lateral-accel param string into a clamped m/s² value.

  Unset (''), unparseable, or non-finite → `default`. A literal 0 → `default`
  unless `zero_disables` (then 0.0, meaning the feature is off). Otherwise the
  value is clamped to [lo, hi].
  """
  try:
    val = float(raw)
  except (TypeError, ValueError):
    return default
  if not math.isfinite(val):
    return default
  if val == 0.0:
    return 0.0 if zero_disables else default
  return min(max(val, lo), hi)


def curvature_speed_cap(model_msg, max_lat_accel: float = 1.5) -> int:
  """Cap speed based on max predicted path curvature within reliable vision.

  Looks at the model's predicted yaw rate and velocity over a horizon
  bounded by:
    - time:     T_IDXS index 30 (≈ 8.8 s, model's prediction limit)
    - distance: model's confidence boundary (yStd-based, capped at 100 m)
  Whichever bound is tighter wins. Beyond the confidence boundary the
  model extrapolates 'straight ahead' and predictions are noise.

  Computes max κ within that range and maps to a comfort-limited safe
  speed. The target lateral acceleration is `max_lat_accel` (m/s²), wired
  from the MapdCurveTargetLatAccel param by the middleware (default 1.5,
  clamped [1.0, 3.0]) — previously hardcoded at 1.5 (2026-07-28: the layer
  contract makes speedlimitd solely responsible for lateral accel via vEgo,
  so this proactive target is now a tunable rather than a magic number).

  Replaced two separate functions (curvature_speed_cap looking at +5s
  yaw rate and confidence_speed_cap sampling boundary κ) — both were
  computing the same physical quantity (path curvature within vision).

  Returns speed cap in km/h, or 0 if no meaningful constraint.
  """
  if not hasattr(model_msg, 'orientationRate') or not hasattr(model_msg, 'velocity'):
    return 0

  try:
    yaw_rates = list(model_msg.orientationRate.z)
    velocities = list(model_msg.velocity.x)
    positions_x = list(model_msg.position.x)
    yStd = list(model_msg.position.yStd)
  except Exception:
    return 0

  if len(yaw_rates) < 10 or len(velocities) < 10 or len(positions_x) < 10:
    return 0

  # Confidence-based vision distance, capped at 100 m
  CONFIDENCE_THRESHOLD = 0.6
  MAX_VISION = 100.0
  conf_dist = MAX_VISION
  for i in range(min(len(positions_x), len(yStd)) - 1, -1, -1):
    conf = 1.0 / (1.0 + yStd[i])
    if conf > CONFIDENCE_THRESHOLD:
      conf_dist = min(MAX_VISION, positions_x[i])
      break

  # T_IDXS = 10 * (i/32)^2:  i=10 → 1.0s   i=22 → 4.7s   i=30 → 8.8s
  # Iterate within both time AND distance bounds.
  max_curvature = 0.0
  for i in range(5, min(31, len(yaw_rates), len(positions_x))):
    if positions_x[i] > conf_dist:
      break  # past confident vision — extrapolation noise
    v = max(velocities[i], 5.0)  # floor at 5 m/s to avoid division issues
    curvature = abs(yaw_rates[i]) / v
    max_curvature = max(max_curvature, curvature)

  if max_curvature < 0.003:  # negligible curvature (~330m radius)
    return 0

  # v = sqrt(a_lat_max / curvature). max_lat_accel defaults to 1.5 m/s²
  # (below the 3 m/s² EU envelope) to give the BMW DCC's −1 m/s² decel limit
  # time to bleed speed before the curve; MapdCurveTargetLatAccel tunes it.
  safe_speed_kph = ((max_lat_accel / max_curvature) ** 0.5) * 3.6

  if safe_speed_kph >= 100:
    return 0  # no meaningful constraint

  return snap_to_standard_speed(int(safe_speed_kph))


def vision_speed_cap(model_msg) -> int:
  """Cap speed when vision confidently sees a narrow road (≤2 lanes).

  When both inner lane lines are detected with high confidence (>0.6),
  the vision model has a clear view of the road. If only ≤2 lanes are
  visible, the road is likely a link/ramp — cap speed accordingly:
    1 lane  → 30 km/h
    2 lanes → 40 km/h (2 × 20)
  Returns 0 if no cap applies (low confidence or wide road).
  """
  if not hasattr(model_msg, 'laneLineProbs') or len(model_msg.laneLineProbs) < 4:
    return 0

  probs = model_msg.laneLineProbs
  # Inner pair = indices 1, 2 (left and right of ego lane)
  inner_confident = sum(1 for i in (1, 2) if probs[i] > 0.6)
  if inner_confident == 0:
    return 0  # not confident enough

  # Count visible lines: inner pair at 0.3, outer pair (indices 0, 3) at 0.5.
  # The higher outer threshold prevents faint echoes of an adjacent main road
  # from counting as a visible lane when entering a link/ramp.
  visible_lines = sum(1 for i, p in enumerate(probs)
                      if p > (0.5 if i in (0, 3) else 0.3))

  if inner_confident >= 2 and visible_lines <= 2:
    return 40  # 2 lanes — link/ramp
  elif inner_confident >= 1 and visible_lines <= 1:
    return 30  # 1 lane — single-lane ramp
  return 0


def infer_speed_from_road_type(highway_type: str, lane_count: int, road_context: str,
                               width_class: str = '', osm_type: str = '') -> int:
  """Look up fallback speed from road context + highway type + lane count + width.

  For narrow roads (≤2 lanes), vision cannot distinguish a through road from
  a link/ramp, so road-type tables are not used — speed is derived directly
  from lane count: 2 lanes → 40 km/h, 1 lane → 30 km/h.

  For wider roads (≥3 lanes), lane count and lane-width class both infer a
  road class; the higher-ranked class wins when OSM's highway_type is weak.

  width_class is a road-type hint derived from observed lane_width (via the
  lane_width_class table in the country TOML); '' if unavailable.

  osm_type is the OSM highway=* classification from offline_hw tiles ('' if
  unavailable). Unlike OSM maxspeed (sparse/stale in China), the highway
  classification is structural and reliably mapped, so when present it is
  trusted over the vision-inferred class votes. Vision keeps two safety nets:
  the narrow-road shortcut above, and the ≤2-lane vision cap in update().
  """
  # Narrow roads: use lane count directly, skip table lookup
  if lane_count <= 1:
    return 30
  if lane_count == 2:
    return 40

  # Secondary and below are almost never nonurban high-speed roads (especially
  # in China). Override mapd's roadContext to urban when highway type is low.
  URBAN_ONLY_TYPES = {'secondary', 'tertiary', 'residential', 'unclassified', 'living_street', 'service'}
  osm_demotes_context = osm_type in URBAN_ONLY_TYPES and highway_type not in ('motorway', 'trunk')
  if road_context == 'freeway' and (highway_type in URBAN_ONLY_TYPES or osm_demotes_context):
    road_context = 'city'

  if road_context == 'freeway':
    table = SPEED_TABLE_NONURBAN
  else:
    table = SPEED_TABLE_URBAN

  # Infer road class from lane count.
  # Motorway requires freeway context — a 4-lane urban arterial (e.g. 中环路) is trunk, not motorway.
  # Only freeways (expressways with controlled access) can be motorway-grade.
  if lane_count >= 4:
    lane_class = 'motorway' if road_context == 'freeway' else 'trunk'
  elif lane_count >= 3:
    lane_class = 'trunk' if road_context == 'freeway' else 'primary'
  else:
    lane_class = ''

  # When highway type comes from a known G/S expressway ref, trust it directly —
  # don't let lane count or width promote beyond the ref classification.
  # For inferred/lower types (secondary, primary, etc.), voting still applies.
  EXPRESSWAY_REFS = {'motorway', 'trunk'}
  if highway_type in EXPRESSWAY_REFS:
    effective_type = highway_type
  elif osm_type:
    # Trusted OSM classification. 'motorway' without a G/S ref is an urban
    # elevated expressway (中环路-style) — trunk-grade (80), not 100/120.
    effective_type = 'trunk' if osm_type == 'motorway' else osm_type
  else:
    rank = {'motorway': 4, 'trunk': 3, 'primary': 2, 'secondary': 1, 'tertiary': 0, 'residential': -1}
    hw_rank = rank.get(highway_type, -2)
    lane_rank = rank.get(lane_class, -2)
    width_rank = rank.get(width_class, -2)
    # Highest-ranked voter wins; width breaks ties between OSM and lane_count
    # voters so a 3-lane road with 3.0 m lanes settles at secondary rather
    # than being promoted to primary by lane_count alone.
    voters = [(hw_rank, highway_type), (lane_rank, lane_class), (width_rank, width_class)]
    _, effective_type = max(voters, key=lambda v: v[0])

  entry = table.get(effective_type)
  if entry:
    return entry['multi']  # lane_count >= 3, always multi-lane

  return DEFAULT_FALLBACK_SPEED


class SpeedLimitMiddleware:
  def __init__(self):
    # livePose (2026-07-28): lightest single, vehicle-agnostic source of a
    # measured lateral accel — forward speed (velocityDevice.x) and yaw rate
    # (angularVelocityDevice.z) both come from the localizer, no car sensor or
    # steering ratio involved. a_y_meas = v · yaw_rate.
    self.sm = messaging.SubMaster(['modelV2', 'gpsLocationExternal', 'livePose'])
    from openpilot.selfdrive.plugins.plugin_bus import PluginPub
    self._sl_pub = PluginPub('speedLimitState')

    # OSM tile reader — reads offline tiles directly, no mapd binary needed
    _pkg_dir = os.path.dirname(os.path.abspath(__file__))
    if _pkg_dir not in __import__('sys').path:
      __import__('sys').path.insert(0, _pkg_dir)
    from osm_query import OsmTileReader
    self._osm = OsmTileReader()
    self._osm_query_interval = 5.0  # seconds between tile queries (0.2 Hz)
    self._osm_last_query_t = 0.0

    self.country_bboxes = load_country_bboxes()
    self.country_detected = False

    # State
    self.last_yolo_speed: float = 0.0
    self.last_highway_type: str = ''
    self.last_road_name: str = ''
    self.last_road_id: str = ''        # roadName or wayRef — stable road identity
    self.last_road_context: str = 'unknown'
    self.last_way_ref: str = ''
    self.last_osm_hwtype: str = ''     # OSM highway=* class from offline_hw tiles
    self.lane_count: int = 1
    self.lane_count_stable: int = 1
    self.lane_count_stable_since: float = 0.0
    self.lane_count_locked: bool = False  # True once vision has a 2 s stable reading
    self.lane_conf: float = 0.0           # smoothed lane line confidence (0.0–1.0)
    self.vision_cap: int = 0
    self.vision_cap_stable: int = 0
    self.vision_cap_stable_since: float = 0.0
    self.curvature_cap: int = 0
    self._curvature_cap_hold_until: float = 0.0  # monotonic time to hold current cap
    self._curvature_cap_relax_step_t: float = 0.0  # last step time during relax phase

    # Lateral-accel params (2026-07-28 layer contract). Read at startup and
    # refreshed periodically so a UI change takes effect without a restart.
    self.curve_target_lat_accel: float = CURVE_LAT_ACCEL_DEFAULT
    self.react_lat_accel_threshold: float = REACT_LAT_ACCEL_DEFAULT
    self._params_last_read_t: float = 0.0
    self._read_params()

    # Reactive measured-a_y cap state (2026-07-28). A_y ownership is
    # longitudinal: proactive vision capping (curvature_speed_cap) can't see
    # late-appearing curves or plant amplification (route seg-15 measured
    # 3.6 m/s² where vision had capped nothing), and the BMW lateral controller
    # no longer ISO-cancels in curves — so the ISO 3.0 m/s² defense lives HERE
    # now, as a reactive backstop on the *measured* lateral accel.
    self._ay_filt: float = 0.0            # low-pass |a_y_meas| (m/s²)
    self._react_cap_ms: float = 0.0       # engaged reactive cap (m/s), 0 = off
    self._react_over_since: float | None = None   # |a_y| first exceeded threshold
    self._react_quiet_since: float | None = None  # |a_y| first went quiet
    self._react_last_t: float = 0.0       # monotonic time of last reactive tick

    # Gradual speed limit transition — step through standard speeds one level
    # at a time instead of jumping directly (e.g. 80 → 60 → 50 → 40).
    self._displayed_speed_limit: int = 0
    self._last_step_time: float = 0.0

    # GPS state
    self._gps_lat: float = 0.0
    self._gps_lon: float = 0.0
    self._gps_valid: bool = False

    # Confirmation state — starts confirmed so speed limit is active immediately
    self.confirmed: bool = True
    self.confirmed_value: float = 0.0
    self._confirm_debounce_until: float = 0.0

    # Plugin bus: receive toggle commands from carstate/UI
    # Messages buffered before _cmd_init_t are stale (from a previous session)
    try:
      from openpilot.selfdrive.plugins.plugin_bus import PluginSub
      self._cmd_sub = PluginSub(['speedlimit_cmd_car', 'speedlimit_cmd_ui'])
      self._cmd_init_t = time.monotonic()
    except ImportError:
      self._cmd_sub = None

    # Plugin bus: subscribe to lane_centering_state for lane_width fusion
    try:
      from openpilot.selfdrive.plugins.plugin_bus import PluginSub
      self._lc_sub = PluginSub(['lane_centering_state'])
    except ImportError:
      self._lc_sub = None
    self.lane_width: float = 0.0       # smoothed m, 0 = no observation yet
    self.lane_width_class: str = ''    # road-type hint from lane_width_class table

    # YOLO detection state (placeholder for future integration)
    self.yolo_speed: int = 0
    self.yolo_last_seen: float = 0.0
    self.yolo_timeout: float = 120.0  # seconds before YOLO detection expires

  def _read_params(self):
    """Refresh lateral-accel params from the persisted plugin store.

    Uses the same store mapd_runner reads (config.read_plugin_param, env-
    overridable). Values are interpreted as raw m/s² per the README contract.
    Import is lazy + guarded so a missing config module just keeps the current
    (default) values rather than crashing the daemon.
    """
    try:
      from config import read_plugin_param
    except Exception:
      return
    self.curve_target_lat_accel = _parse_lat_accel(
      read_plugin_param('speedlimitd', 'MapdCurveTargetLatAccel', ''),
      CURVE_LAT_ACCEL_DEFAULT, CURVE_LAT_ACCEL_MIN, CURVE_LAT_ACCEL_MAX)
    self.react_lat_accel_threshold = _parse_lat_accel(
      read_plugin_param('speedlimitd', 'MapdReactLatAccel', ''),
      REACT_LAT_ACCEL_DEFAULT, REACT_LAT_ACCEL_MIN, REACT_LAT_ACCEL_MAX,
      zero_disables=True)

  def _update_reactive_cap(self, a_y_meas, v_ego: float, threshold: float,
                           now: float, dt: float) -> float:
    """Reactive measured-a_y speed cap (2026-07-28 layer contract).

    Low-passes |a_y_meas| (~0.3 s). When it exceeds `threshold` continuously
    for REACT_ENGAGE_S, engages a cap v_react = v_ego·sqrt(threshold/|a_y|)
    minus a 1 m/s hysteresis margin. While engaged the cap may only move DOWN
    or hold — it never chases a_y upward. Release ramps the cap back up at
    REACT_RELEASE_RATE once |a_y| < threshold−REACT_QUIET_MARGIN for
    REACT_QUIET_S. `threshold <= 0` disables the feature entirely.

    State lives on self; returns the current cap (m/s, 0 = disengaged). Kept a
    pure function of its inputs so it is unit-testable without livePose msgs.
    """
    if a_y_meas is not None and math.isfinite(a_y_meas):
      mag = abs(a_y_meas)
      alpha = dt / (REACT_TAU + dt) if dt > 0.0 else 1.0
      self._ay_filt += (mag - self._ay_filt) * alpha
    ay = self._ay_filt

    if threshold <= 0.0:  # disabled
      self._react_cap_ms = 0.0
      self._react_over_since = None
      self._react_quiet_since = None
      return 0.0

    # Engage debounce (over threshold) and release debounce (quiet).
    if ay > threshold:
      if self._react_over_since is None:
        self._react_over_since = now
    else:
      self._react_over_since = None
    if ay < threshold - REACT_QUIET_MARGIN:
      if self._react_quiet_since is None:
        self._react_quiet_since = now
    else:
      self._react_quiet_since = None

    if self._react_cap_ms <= 0.0:
      # Not engaged — engage after sustained over-threshold.
      if (self._react_over_since is not None
          and now - self._react_over_since >= REACT_ENGAGE_S
          and ay > 0.0 and v_ego > 0.0):
        v_react = v_ego * math.sqrt(threshold / ay) - REACT_HYST_MS
        self._react_cap_ms = max(v_react, 0.0)
    else:
      # Engaged.
      if (self._react_quiet_since is not None
          and now - self._react_quiet_since >= REACT_QUIET_S):
        # Sustained quiet — ramp the cap back up, disengage once it no longer
        # constrains (reaches current speed).
        self._react_cap_ms += REACT_RELEASE_RATE * dt
        if self._react_cap_ms >= max(v_ego, 0.0):
          self._react_cap_ms = 0.0
      elif ay > 0.0 and v_ego > 0.0:
        # Still cornering (or not quiet long enough): monotonic-down / hold.
        v_react = v_ego * math.sqrt(threshold / ay) - REACT_HYST_MS
        self._react_cap_ms = min(self._react_cap_ms, max(v_react, 0.0))
    return self._react_cap_ms

  def _ingest_osm_result(self, result: dict | None):
    """Update road identity state from an OSM tile query result.

    Accepts any matched way that carries identity or classification — refs
    (G2), names (白城路), or a bare highwayType (unnamed service roads).
    """
    if result and (result['wayRef'] or result['roadName'] or result.get('highwayType')):
      way_ref = result['wayRef']
      self.last_way_ref = way_ref
      self.last_road_name = result['roadName']
      self.last_osm_hwtype = result.get('highwayType', '')

      # Road context
      if result['roadContext'] == 0:
        self.last_road_context = 'freeway'
      elif result['roadContext'] == 1:
        self.last_road_context = 'city'

      # Highway type from wayRef.
      # G = national expressway (120 km/h), S1-S99 = provincial expressway (100 km/h).
      # S100+ are provincial general roads, not expressways.
      road_id = result['roadName'] or way_ref
      hw = ''
      if way_ref.startswith('G'):
        hw = 'motorway'
      elif way_ref.startswith('S') and len(way_ref[1:]) <= 2 and way_ref[1:].isdigit():
        hw = 'trunk'
      hw_rank = {'motorway': 4, 'trunk': 3}
      if road_id != self.last_road_id:
        self.last_road_id = road_id
        self.last_highway_type = hw
      elif hw_rank.get(hw, -1) > hw_rank.get(self.last_highway_type, -1):
        self.last_highway_type = hw
    else:
      self.last_way_ref = ''
      self.last_road_name = ''
      self.last_osm_hwtype = ''

  def update(self):
    global SPEED_TABLE_URBAN, SPEED_TABLE_NONURBAN, DEFAULT_FALLBACK_SPEED, LANE_WIDTH_CLASS_TABLE
    self.sm.update(0)

    now = time.monotonic()

    # --- Refresh lateral-accel params (5 s cadence, UI-change friendly) ---
    if now - self._params_last_read_t >= 5.0:
      self._params_last_read_t = now
      self._read_params()

    # --- Auto-detect country from GPS ---
    if self.sm.updated.get('gpsLocationExternal', False):
      gps = self.sm['gpsLocationExternal']
      if gps.flags % 2 == 1:  # valid fix
        self._gps_lat = gps.latitude
        self._gps_lon = gps.longitude
        self._gps_valid = True
        if not self.country_detected:
          country = country_from_gps(gps.latitude, gps.longitude, self.country_bboxes)
          if country:
            try:
              SPEED_TABLE_URBAN, SPEED_TABLE_NONURBAN, DEFAULT_FALLBACK_SPEED, LANE_WIDTH_CLASS_TABLE = load_speed_table(country)
            except FileNotFoundError:
              pass
          self.country_detected = True

    # --- Query OSM tiles at 0.2 Hz ---
    if self._gps_valid and now - self._osm_last_query_t >= self._osm_query_interval:
      self._osm_last_query_t = now
      try:
        result = self._osm.query(self._gps_lat, self._gps_lon)
      except Exception:
        result = None

      self._ingest_osm_result(result)

    # --- Read lane data from vision model ---
    if self.sm.updated['modelV2']:
      model = self.sm['modelV2']
      raw_lane_count = infer_lane_count(model)

      # Adaptive demotion hysteresis based on predicted curvature.
      # Straight road: drops are likely lane-change occlusion → 5s to filter.
      # Curved road: road is genuinely narrowing → 2s for quick response.
      curving = self.curvature_cap > 0  # curvature_speed_cap detected upcoming curve
      if raw_lane_count != self.lane_count:
        self.lane_count = raw_lane_count
        self.lane_count_stable_since = now
      else:
        going_down = raw_lane_count < self.lane_count_stable
        demotion_window = 2.0 if curving else 5.0
        stability_window = demotion_window if going_down else 1.5
        if now - self.lane_count_stable_since > stability_window:
          self.lane_count_stable = self.lane_count
          self.lane_count_locked = True

      # Lane line confidence: sum of all probs divided by line count.
      # Scales with both the number of visible lines and their individual strength.
      probs = list(model.laneLineProbs) if hasattr(model, 'laneLineProbs') else []
      if probs:
        raw_conf = sum(min(p, 1.0) for p in probs) / len(probs)
        # Exponential smoothing (α=0.2) — fast enough to track real changes,
        # slow enough to suppress single-frame noise.
        self.lane_conf = 0.8 * self.lane_conf + 0.2 * raw_conf

      # Vision speed cap for narrow roads (links/ramps)
      raw_cap = vision_speed_cap(model)
      if raw_cap != self.vision_cap:
        self.vision_cap = raw_cap
        self.vision_cap_stable_since = now
      elif now - self.vision_cap_stable_since > 1.0:
        self.vision_cap_stable = self.vision_cap

      # Curvature lookahead cap from model predicted path.
      # The model's curvature prediction is noisy — the cap can flicker between
      # 0 and a valid value frame-to-frame. Smooth by:
      #   1) holding the lowest recent cap for 3 seconds (locks in the tightest
      #      reading against single-frame dropouts),
      #   2) when the hold expires and raw cap stays low, step-relaxing up the
      #      standard ladder (60 → 80 → 100 → off) instead of snapping to 0.
      # Re-detection at any point during the relax walks back to the tighter cap.
      raw_curv_cap = curvature_speed_cap(model, self.curve_target_lat_accel)
      if raw_curv_cap > 0 and (raw_curv_cap <= self.curvature_cap or self.curvature_cap == 0):
        # Curve actively detected (tighter, equal, or first) — apply and refresh hold
        self.curvature_cap = raw_curv_cap
        self._curvature_cap_hold_until = now + 3.0
        self._curvature_cap_relax_step_t = 0.0  # cancel any in-progress relax
      elif now < self._curvature_cap_hold_until:
        pass  # Hold current cap during hold period
      elif self.curvature_cap > 0:
        # Hold expired, raw is looser or 0 — step-relax up the standard ladder.
        # _STANDARD_SPEEDS = [30, 40, 50, 60, 80, 100, 120]; curvature_speed_cap
        # returns 0 above 100, so we treat reaching > 80 as "release to off".
        RELAX_STEP_INTERVAL = 2.0  # s per rung
        if self._curvature_cap_relax_step_t == 0.0:
          self._curvature_cap_relax_step_t = now
        elif now - self._curvature_cap_relax_step_t >= RELAX_STEP_INTERVAL:
          higher = [s for s in _STANDARD_SPEEDS if s > self.curvature_cap]
          if higher and min(higher) <= 80:
            self.curvature_cap = min(higher)
            self._curvature_cap_relax_step_t = now
          else:
            self.curvature_cap = 0
            self._curvature_cap_relax_step_t = 0.0

    # --- Reactive measured-a_y cap (2026-07-28 layer contract) ---
    # Runs every tick (not gated on modelV2) off the localizer's measured yaw.
    # a_y_meas = v · yaw_rate; both from livePose (vehicle-agnostic, no steering
    # ratio). Catches curves the proactive vision cap missed and plant
    # amplification — the ISO 3.0 m/s² defense now lives here (see __init__).
    react_dt = min(max(now - self._react_last_t, 0.0), 1.0) if self._react_last_t else 0.2
    self._react_last_t = now
    a_y_meas = None
    v_ego_meas = 0.0
    if self.sm.updated.get('livePose', False):
      try:
        lp = self.sm['livePose']
        av = lp.angularVelocityDevice
        vd = lp.velocityDevice
        if getattr(av, 'valid', True) and getattr(vd, 'valid', True):
          v_ego_meas = abs(float(vd.x))
          a_y_meas = v_ego_meas * float(av.z)
      except Exception:
        a_y_meas = None
        v_ego_meas = 0.0
    self._update_reactive_cap(a_y_meas, v_ego_meas, self.react_lat_accel_threshold,
                              now, react_dt)

    # --- Lane width observation from lane_centering plugin ---
    # Smoothed (EMA) across 5 Hz drain so a single noisy frame can't swing
    # the road-class vote. lane_width_learned=False → fall back to default
    # width from lane_centering; we ignore those to avoid false confidence.
    if self._lc_sub is not None:
      lc = self._lc_sub.drain()
      if lc is not None:
        _, data = lc
        if isinstance(data, dict) and data.get('lane_width_learned'):
          w = data.get('lane_width')
          if isinstance(w, (int, float)) and w > 0:
            if self.lane_width == 0.0:
              self.lane_width = float(w)
            else:
              self.lane_width = 0.8 * self.lane_width + 0.2 * float(w)
            self.lane_width_class = classify_by_width(self.lane_width, LANE_WIDTH_CLASS_TABLE)

    # --- YOLO timeout ---
    if self.yolo_speed > 0 and (now - self.yolo_last_seen) > self.yolo_timeout:
      self.yolo_speed = 0

    # --- Priority cascade ---
    yolo_speed = self.yolo_speed

    # Urban expressways without a G/S highway ref (like 中环路, 北翟高架路) are classified
    # as 'freeway' by mapd but their actual speed limit is trunk-class (80 km/h), not
    # motorway-class (120 km/h). Treat them as 'city' for inference.
    road_ctx_for_infer = self.last_road_context
    if not self.last_way_ref and road_ctx_for_infer == 'freeway':
      road_ctx_for_infer = 'city'

    inferred_speed = infer_speed_from_road_type(
      self.last_highway_type, self.lane_count_stable, road_ctx_for_infer,
      width_class=self.lane_width_class, osm_type=self.last_osm_hwtype,
    )

    # Vision cap: when vision confidently sees ≤2 lanes, cap inferred speed.
    # Only apply when lane_count_stable < 3 — on confirmed multi-lane roads the
    # outermost lane line probability naturally fluctuates below the cap threshold
    # without implying a narrow road, so the cap would fire spuriously.
    if self.vision_cap_stable > 0 and self.lane_count_stable < 3:
      inferred_speed = min(inferred_speed, self.vision_cap_stable)

    MIN_SPEED_LIMIT = 30   # km/h — no real road is below this

    # OSM maxSpeed is unreliable in China — use OSM only for road context,
    # highway classification (G/S ref), and road name.

    # Reactive measured-a_y cap (2026-07-28): enters the SAME min() path as the
    # proactive curve cap, as a safety-class source (source 4). That inheritance
    # is deliberate — planner_hook's gas-override suspends it and, crucially, its
    # lead-override protection (route 2fd) does NOT bypass safety-class caps, so
    # a faster lead can never lift the reactive curve cap.
    react_cap_kph = self._react_cap_ms * 3.6 if self._react_cap_ms > 0.0 else 0.0
    react_active = react_cap_kph >= MIN_SPEED_LIMIT

    # Take minimum across all available sources — most conservative valid reading wins.
    candidates = []
    if yolo_speed >= MIN_SPEED_LIMIT:
      candidates.append((float(yolo_speed), 1, 0.8))    # yoloDetection
    if self.curvature_cap >= MIN_SPEED_LIMIT:
      candidates.append((float(self.curvature_cap), 4, 0.7))  # curvatureLookahead
    if react_active:
      candidates.append((react_cap_kph, 4, 0.7))  # reactiveLatAccel (safety class)
    candidates.append((float(max(inferred_speed, MIN_SPEED_LIMIT)), 2, round(self.lane_conf, 2)))  # roadTypeInference

    speed_limit, source, confidence = min(candidates, key=lambda x: x[0])

    # --- Gradual speed limit transition ---
    # Curvature cap bypasses gradual transition — it's safety-critical and must
    # apply immediately. The gradual ramp only applies to road-type / YOLO changes.
    target = snap_to_standard_speed(int(speed_limit))
    if self._displayed_speed_limit == 0:
      # First reading — set immediately
      self._displayed_speed_limit = target
      self._last_step_time = now
    elif target != self._displayed_speed_limit:
      interval = _STEP_DOWN_INTERVAL if target < self._displayed_speed_limit else _STEP_UP_INTERVAL
      if now - self._last_step_time >= interval:
        self._displayed_speed_limit = _step_speed_limit(self._displayed_speed_limit, target)
        self._last_step_time = now

    # Safety cap override — clamp displayed limit immediately (bypass gradual
    # transition) so a tightening curve cap takes effect without lag. Both the
    # proactive curve cap and the reactive measured-a_y cap are safety-critical.
    if self.curvature_cap >= MIN_SPEED_LIMIT:
      cap_snapped = snap_to_standard_speed(self.curvature_cap)
      if cap_snapped < self._displayed_speed_limit:
        self._displayed_speed_limit = cap_snapped
    if react_active:
      react_snapped = snap_to_standard_speed(int(react_cap_kph))
      if react_snapped < self._displayed_speed_limit:
        self._displayed_speed_limit = react_snapped

    # safetyCapped: True whenever a safety cap (proactive curve OR reactive
    # measured-a_y) is the active (lowest) source OR is at/below the displayed
    # limit (i.e., the limit IS being constrained by a safety cap, regardless of
    # gradual-transition state). planner_hook uses this to skip the comfort
    # offset AND to keep the cap in the lead-override-protected class.
    safety_capped = source == 4 or (
      self.curvature_cap >= MIN_SPEED_LIMIT
      and snap_to_standard_speed(self.curvature_cap) <= self._displayed_speed_limit
    ) or (
      react_active
      and snap_to_standard_speed(int(react_cap_kph)) <= self._displayed_speed_limit
    )

    # --- Confirmation management ---
    # Process toggle commands from carstate resume button / UI tap via plugin bus.
    # Confirmed state is sticky — only changes on explicit user toggle.
    # Never auto-reset on speed limit change, disengage, or process restart.
    if self._cmd_sub is not None:
      cmd = self._cmd_sub.drain()
      if cmd is not None and time.monotonic() - self._cmd_init_t > 2.0:
        _, data = cmd
        if isinstance(data, dict) and data.get('action') == 'toggle_confirm' and now > self._confirm_debounce_until:
          self.confirmed = not self.confirmed
          self._confirm_debounce_until = now + 1.0  # 1s debounce
          try:
            from openpilot.common.swaglog import cloudlog
            cloudlog.info(f"speedlimitd: confirmed toggled to {self.confirmed}")
          except Exception:
            pass

    # Track current limit for the planner (uses displayed limit after gradual transition)
    if self.confirmed:
      self.confirmed_value = self._displayed_speed_limit
    else:
      self.confirmed_value = 0.0

    # --- Publish ---
    self._sl_pub.send({
      'speedLimit': self._displayed_speed_limit,
      'source': source,
      'confirmed': self.confirmed,
      'confidence': confidence,
      'yoloSpeed': yolo_speed,
      'inferredSpeed': inferred_speed,
      'highwayType': self.last_highway_type,
      'osmHwType': self.last_osm_hwtype,
      'wayRef': self.last_way_ref,
      'roadName': self.last_road_name,
      'laneCount': self.lane_count_stable,
      'laneWidth': round(self.lane_width, 2),
      'laneWidthClass': self.lane_width_class,
      'curvatureCap': self.curvature_cap,
      'safetyCapped': safety_capped,
      # Reactive measured-a_y cap telemetry (2026-07-28).
      'reactCapEngaged': self._react_cap_ms > 0.0,
      'reactCap': round(react_cap_kph, 1),          # km/h, 0 = disengaged
      'reactLatAccel': round(self._ay_filt, 2),     # measured filtered |a_y| (m/s²)
    })


def main():
  middleware = SpeedLimitMiddleware()
  rk = Ratekeeper(5, print_delay_threshold=None)  # 5 Hz

  while True:
    middleware.update()
    rk.keep_time()


if __name__ == "__main__":
  main()
