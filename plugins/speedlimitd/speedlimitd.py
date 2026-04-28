#!/usr/bin/env python3
"""
Speed Limit Middleware — merges mapd, YOLO, and road-type inference
into a single SpeedLimitState message at 5 Hz.

Three-tier priority:
  1. YOLO speed sign detection (direct sign reading, highest confidence)
  2. mapd suggestedSpeed (comprehensive: visionCurveSpeed + speed limit + road type)
  3. Vision-inferred speed (lane count + road type, own fallback when mapd has no data)
"""
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


def curvature_speed_cap(model_msg) -> int:
  """Cap speed based on predicted path curvature lookahead.

  Uses the model's predicted orientation rate (yaw rate) and velocity
  over a horizon bounded by both time (8.8s ceiling, T_IDXS[30]) AND
  distance (~100m, the model's reliable vision range). Beyond 100m the
  model extrapolates 'straight ahead' from current heading and any κ
  prediction is noise — we'd rather miss a curve than chase a phantom.
  Adaptive horizon means: at city speed (40 kph) we use the full 8.8s
  window; at highway speed (>72 kph) the 100m distance limit kicks in
  and the time horizon shrinks. For curves at high speed beyond 100m,
  OSM map data is the right path (not model-based prediction).

  Returns speed cap in km/h, or 0 if no constraint.
  """
  if not hasattr(model_msg, 'orientationRate') or not hasattr(model_msg, 'velocity'):
    return 0

  try:
    yaw_rates = list(model_msg.orientationRate.z)
    velocities = list(model_msg.velocity.x)
    positions_x = list(model_msg.position.x)
  except Exception:
    return 0

  if len(yaw_rates) < 10 or len(velocities) < 10 or len(positions_x) < 10:
    return 0

  # T_IDXS = 10 * (i/32)^2:  i=10 → 1.0s   i=22 → 4.7s   i=30 → 8.8s
  # Vision-bounded horizon: stop iterating once predicted x exceeds VISION_M.
  VISION_M = 100.0
  max_curvature = 0.0
  for i in range(5, min(31, len(yaw_rates), len(positions_x))):
    if positions_x[i] > VISION_M:
      break  # past reliable vision — model extrapolates, predictions are noise
    v = max(velocities[i], 5.0)  # floor at 5 m/s to avoid division issues
    curvature = abs(yaw_rates[i]) / v
    max_curvature = max(max_curvature, curvature)

  if max_curvature < 0.003:  # negligible curvature (~330m radius)
    return 0

  # v = sqrt(a_lat_max / curvature)
  # Use 1.5 m/s² (not the physical limit of ~3-4 m/s²) because the model
  # underestimates apex curvature when looking ahead from the approach.
  # 1.5 compensates for this, producing 60 km/h caps ~3s earlier.
  MAX_LAT_ACCEL = 1.5
  safe_speed_ms = (MAX_LAT_ACCEL / max_curvature) ** 0.5
  safe_speed_kph = safe_speed_ms * 3.6

  if safe_speed_kph >= 100:
    return 0  # no meaningful constraint

  return snap_to_standard_speed(int(safe_speed_kph))


def confidence_speed_cap(model_msg, v_ego) -> int:
  """Cap speed based on curvature visible at the confidence boundary.

  The model's reliable vision is ~100 m. At highway speed, BMW DCC's
  −1 m/s² decel limit cannot bleed enough speed to handle a tight curve
  emerging from past-vision distance — but only IF a curve is actually
  there. The previous time-based visibility check (PREVIEW_TIME) capped
  speed any time vision didn't extend v_ego·PREVIEW_TIME ahead, which
  triggered on perfectly straight highway and felt over-conservative.

  Switched to a curvature-gated cap: look at the predicted curvature AT
  the visibility boundary. If there's curvature there, cap to a speed
  that handles it (using MAX_LAT_ACCEL_CAP as the comfort target). If
  the road appears straight at the boundary (κ < threshold), no cap —
  trust until evidence shows otherwise.

  Without reliable OSM data in China this is the only forward-curve
  safety net at high speed; it accepts a residual risk on roads where
  a tight curve hides past straight-road visibility (rare in practice).
  """
  CONFIDENCE_THRESHOLD = 0.6
  MIN_DIST = 20.0
  MAX_DIST = 100.0
  BOUNDARY_CURV_THRESHOLD = 0.004   # 1/m (radius ≥ 250 m). Below ≈ straight at boundary.
  MAX_LAT_ACCEL_CAP = 1.5    # m/s² target for cap (same as curvature_speed_cap)
  MIN_CAP_SPEED = 30.0       # km/h floor
  MAX_CAP_SPEED = 100.0      # km/h — above this, treat as no constraint

  if v_ego < 5.0:
    return 0

  # Find confidence distance — farthest point where yStd confidence > threshold
  conf_dist = MIN_DIST
  try:
    pos = model_msg.position
    x = list(pos.x)
    yStd = list(pos.yStd)
    if len(x) >= 5 and len(yStd) >= 5:
      for i in range(len(x) - 1, -1, -1):
        conf = 1.0 / (1.0 + yStd[i])
        if conf > CONFIDENCE_THRESHOLD:
          conf_dist = max(MIN_DIST, min(MAX_DIST, x[i]))
          break
  except (AttributeError, IndexError):
    pass

  # Curvature at confidence boundary
  boundary_curv = 0.0
  try:
    pos = model_msg.position
    px, py = list(pos.x), list(pos.y)
    if len(px) >= 5 and len(py) >= 5:
      import numpy as np
      pxa, pya = np.array(px), np.array(py)
      idx = np.searchsorted(pxa, conf_dist)
      if 2 <= idx < len(pxa) - 1:
        dx = pxa[idx+1] - pxa[idx-1]
        dy = pya[idx+1] - pya[idx-1]
        d2y = pya[idx+1] - 2*pya[idx] + pya[idx-1]
        if abs(dx) >= 0.1:
          dydx = dy / dx
          d2ydx2 = d2y / ((dx/2) * dx)
          boundary_curv = abs(d2ydx2) / (1 + dydx**2)**1.5
  except (AttributeError, IndexError):
    pass

  # No cap if road appears straight at the boundary
  if boundary_curv < BOUNDARY_CURV_THRESHOLD:
    return 0

  # Cap based on boundary curvature
  safe_speed_kph = ((MAX_LAT_ACCEL_CAP / boundary_curv) ** 0.5) * 3.6

  if safe_speed_kph >= MAX_CAP_SPEED:
    return 0
  return int(max(MIN_CAP_SPEED, safe_speed_kph))


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
                               width_class: str = '') -> int:
  """Look up fallback speed from road context + highway type + lane count + width.

  For narrow roads (≤2 lanes), vision cannot distinguish a through road from
  a link/ramp, so road-type tables are not used — speed is derived directly
  from lane count: 2 lanes → 40 km/h, 1 lane → 30 km/h.

  For wider roads (≥3 lanes), lane count and lane-width class both infer a
  road class; the higher-ranked class wins when OSM's highway_type is weak.

  width_class is a road-type hint derived from observed lane_width (via the
  lane_width_class table in the country TOML); '' if unavailable.
  """
  # Narrow roads: use lane count directly, skip table lookup
  if lane_count <= 1:
    return 30
  if lane_count == 2:
    return 40

  # Secondary and below are almost never nonurban high-speed roads (especially
  # in China). Override mapd's roadContext to urban when highway type is low.
  URBAN_ONLY_TYPES = {'secondary', 'tertiary', 'residential', 'unclassified', 'living_street', 'service'}
  if road_context == 'freeway' and highway_type in URBAN_ONLY_TYPES:
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
    self.sm = messaging.SubMaster(['modelV2', 'gpsLocationExternal'])
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

    self.lookahead_cap: int = 0
    self._lookahead_cap_hold_until: float = 0.0

    # YOLO detection state (placeholder for future integration)
    self.yolo_speed: int = 0
    self.yolo_last_seen: float = 0.0
    self.yolo_timeout: float = 120.0  # seconds before YOLO detection expires

  def update(self):
    global SPEED_TABLE_URBAN, SPEED_TABLE_NONURBAN, DEFAULT_FALLBACK_SPEED, LANE_WIDTH_CLASS_TABLE
    self.sm.update(0)

    now = time.monotonic()

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

      if result and result['wayRef']:
        way_ref = result['wayRef']
        self.last_way_ref = way_ref
        self.last_road_name = result['roadName']

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
      # 0 and a valid value frame-to-frame. Smooth by holding the lowest recent
      # cap for 3 seconds, and only releasing when the raw cap exceeds it.
      raw_curv_cap = curvature_speed_cap(model)
      if raw_curv_cap > 0 and (raw_curv_cap < self.curvature_cap or self.curvature_cap == 0):
        # Tighter cap detected — apply immediately
        self.curvature_cap = raw_curv_cap
        self._curvature_cap_hold_until = now + 3.0
      elif now < self._curvature_cap_hold_until:
        pass  # Hold current cap during hold period
      else:
        # Hold expired — allow cap to relax
        self.curvature_cap = raw_curv_cap

    # --- Confidence-based speed cap (moved from look_ahead plugin) ---
    if self.sm.updated['modelV2']:
      try:
        v_ego = max(list(model.velocity.x)[0], 0.0) if hasattr(model, 'velocity') else 0.0
      except (IndexError, AttributeError):
        v_ego = 0.0
      raw_la_cap = confidence_speed_cap(model, v_ego)
      if raw_la_cap > 0 and (raw_la_cap < self.lookahead_cap or self.lookahead_cap == 0):
        self.lookahead_cap = raw_la_cap
        self._lookahead_cap_hold_until = now + 3.0
      elif now < self._lookahead_cap_hold_until:
        pass  # Hold current cap
      else:
        self.lookahead_cap = raw_la_cap

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
      width_class=self.lane_width_class,
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

    # Take minimum across all available sources — most conservative valid reading wins.
    candidates = []
    if yolo_speed >= MIN_SPEED_LIMIT:
      candidates.append((float(yolo_speed), 1, 0.8))    # yoloDetection
    if self.curvature_cap >= MIN_SPEED_LIMIT:
      candidates.append((float(self.curvature_cap), 4, 0.7))  # curvatureLookahead
    if self.lookahead_cap >= MIN_SPEED_LIMIT:
      candidates.append((float(self.lookahead_cap), 5, 0.7))  # lookaheadConfidence
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

    # Safety caps override — clamp displayed limit immediately (bypass gradual
    # transition) so a tightening curve cap takes effect without lag.
    for cap in (self.curvature_cap, self.lookahead_cap):
      if cap >= MIN_SPEED_LIMIT:
        cap_snapped = snap_to_standard_speed(cap)
        if cap_snapped < self._displayed_speed_limit:
          self._displayed_speed_limit = cap_snapped

    # safetyCapped: True whenever a curve cap is the active (lowest) source
    # OR is at/below the displayed limit (i.e., the limit IS being constrained
    # by a safety cap, regardless of gradual-transition state). planner_hook
    # uses this to skip the +15% comfort offset on curve approaches — applying
    # the offset to a curve-derived speed defeats the purpose of the cap.
    safety_capped = source in (4, 5) or any(
      snap_to_standard_speed(c) <= self._displayed_speed_limit
      for c in (self.curvature_cap, self.lookahead_cap)
      if c >= MIN_SPEED_LIMIT
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
      'wayRef': self.last_way_ref,
      'roadName': self.last_road_name,
      'laneCount': self.lane_count_stable,
      'laneWidth': round(self.lane_width, 2),
      'laneWidthClass': self.lane_width_class,
      'curvatureCap': self.curvature_cap,
      'lookaheadCap': self.lookahead_cap,
      'safetyCapped': safety_capped,
    })


def main():
  middleware = SpeedLimitMiddleware()
  rk = Ratekeeper(5, print_delay_threshold=None)  # 5 Hz

  while True:
    middleware.update()
    rk.keep_time()


if __name__ == "__main__":
  main()
