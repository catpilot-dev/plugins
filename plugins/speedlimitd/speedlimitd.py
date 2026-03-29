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


def load_speed_table(country: str) -> tuple[dict, dict, int]:
  """Load urban/nonurban speed tables and fallback from a country TOML file.

  Returns (urban_table, nonurban_table, default_fallback).
  """
  path = os.path.join(SPEED_TABLES_DIR, f'{country}.toml')
  with open(path, 'rb') as f:
    data = tomllib.load(f)

  urban = {k: dict(v) for k, v in data.get('urban', {}).items()}
  nonurban = {k: dict(v) for k, v in data.get('nonurban', {}).items()}
  fallback = data.get('default_fallback', 40)
  return urban, nonurban, fallback


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
SPEED_TABLE_URBAN, SPEED_TABLE_NONURBAN, DEFAULT_FALLBACK_SPEED = load_speed_table('cn')

# Standard speed limit values used in China (GB 5768)
_STANDARD_SPEEDS = [30, 40, 60, 80, 100, 120]


def snap_to_standard_speed(speed: int) -> int:
  """Snap a computed speed to the nearest standard speed limit value.

  mapd visionCurveSpeed produces raw values like 47, 75, 83 km/h.
  Speed limit signs always display standard values, so we snap for
  clean display and consistent planner behaviour.
  """
  return min(_STANDARD_SPEEDS, key=lambda s: abs(s - speed))


def infer_lane_count(model_msg) -> int:
  """Infer lane count from modelV2 laneLineProbs and roadEdges.

  The model outputs 4 lane lines (indices 0-3) and 2 road edges.
  Lane lines form lane boundaries; N visible lines = up to N-1 lanes
  on the visible side of the road. Road edges beyond the outermost
  lane lines suggest additional lanes.

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
    return 4
  elif visible_lines >= 3:
    return 3
  elif visible_lines >= 2:
    return 2
  return 1


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


def infer_speed_from_road_type(highway_type: str, lane_count: int, road_context: str) -> int:
  """Look up fallback speed from road context + highway type + lane count.

  For narrow roads (≤2 lanes), vision cannot distinguish a through road from
  a link/ramp, so road-type tables are not used — speed is derived directly
  from lane count: 2 lanes → 40 km/h, 1 lane → 30 km/h.

  For wider roads (≥3 lanes), lane count infers road class and the
  appropriate speed table is consulted.
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
  # don't let lane count promote beyond the ref classification.
  # For inferred/lower types (secondary, primary, etc.), lane-count promotion still applies.
  EXPRESSWAY_REFS = {'motorway', 'trunk'}
  if highway_type in EXPRESSWAY_REFS:
    effective_type = highway_type
  else:
    rank = {'motorway': 4, 'trunk': 3, 'primary': 2, 'secondary': 1, 'tertiary': 0}
    hw_rank = rank.get(highway_type, -1)
    lane_rank = rank.get(lane_class, -1)
    effective_type = lane_class if lane_rank > hw_rank else highway_type

  entry = table.get(effective_type)
  if entry:
    return entry['multi']  # lane_count >= 3, always multi-lane

  return DEFAULT_FALLBACK_SPEED


class SpeedLimitMiddleware:
  def __init__(self):
    from cereal.services import SERVICE_LIST
    subs = ['modelV2', 'gpsLocationExternal']
    if 'mapdOut' in SERVICE_LIST:
      subs.append('mapdOut')
    self._has_mapd = 'mapdOut' in subs
    self.sm = messaging.SubMaster(subs)
    from openpilot.selfdrive.plugins.plugin_bus import PluginPub
    self._sl_pub = PluginPub('speedLimitState')

    self.country_bboxes = load_country_bboxes()
    self.country_detected = False

    # State
    self.last_osm_speed: float = 0.0
    self.last_mapd_suggested: float = 0.0   # mapd suggestedSpeed (comprehensive)
    self.last_yolo_speed: float = 0.0
    self.last_highway_type: str = ''
    self.last_road_name: str = ''
    self.last_road_id: str = ''        # roadName or wayRef — stable road identity
    self.last_road_context: str = 'unknown'
    self.last_way_ref: str = ''
    self.osm_lanes: int = 0
    self.lane_count: int = 1
    self.lane_count_stable: int = 1
    self.lane_count_stable_since: float = 0.0
    self.lane_count_locked: bool = False  # True once vision has a 2 s stable reading
    self.lane_conf: float = 0.0           # smoothed lane line confidence (0.0–1.0)
    self.vision_cap: int = 0
    self.vision_cap_stable: int = 0
    self.vision_cap_stable_since: float = 0.0

    # Confirmation state — confirmed by default on engage; user can toggle off
    self.confirmed: bool = False
    self.confirmed_value: float = 0.0

    # Plugin bus: receive toggle commands from carstate/UI
    try:
      from openpilot.selfdrive.plugins.plugin_bus import PluginSub
      self._cmd_sub = PluginSub(['speedlimit_cmd_car', 'speedlimit_cmd_ui'])
    except ImportError:
      self._cmd_sub = None

    # YOLO detection state (placeholder for future integration)
    self.yolo_speed: int = 0
    self.yolo_last_seen: float = 0.0
    self.yolo_timeout: float = 120.0  # seconds before YOLO detection expires

  def update(self):
    global SPEED_TABLE_URBAN, SPEED_TABLE_NONURBAN, DEFAULT_FALLBACK_SPEED
    self.sm.update(0)

    now = time.monotonic()

    # --- Auto-detect country from GPS (once) ---
    if not self.country_detected and self.sm.updated.get('gpsLocationExternal', False):
      gps = self.sm['gpsLocationExternal']
      if gps.flags % 2 == 1:  # valid fix
        country = country_from_gps(gps.latitude, gps.longitude, self.country_bboxes)
        if country:
          try:
            SPEED_TABLE_URBAN, SPEED_TABLE_NONURBAN, DEFAULT_FALLBACK_SPEED = load_speed_table(country)
          except FileNotFoundError:
            pass
        self.country_detected = True

    # --- Read mapd data (if available) ---
    if self._has_mapd and self.sm.updated['mapdOut']:
      mapd = self.sm['mapdOut']
      # mapd publishes speeds in m/s, convert to km/h
      self.last_osm_speed = mapd.speedLimit * 3.6 if mapd.speedLimit > 0 else 0.0
      self.last_mapd_suggested = mapd.suggestedSpeed * 3.6 if mapd.suggestedSpeed > 0 else 0.0
      self.last_road_name = mapd.roadName

      # Road context from mapd (0=freeway, 1=city)
      road_ctx = mapd.roadContext
      if road_ctx == 0:
        self.last_road_context = 'freeway'
      elif road_ctx == 1:
        self.last_road_context = 'city'
      else:
        self.last_road_context = 'unknown'

      self.osm_lanes = mapd.lanes
      self.last_way_ref = mapd.wayRef

      # Highway type from wayRef — only expressway-grade refs are reliable.
      # G (national expressway) → motorway (120 km/h), S (provincial expressway) → trunk (100 km/h).
      # Lower-grade refs (X=county, etc.) are ignored: GPS cannot distinguish
      # an elevated expressway from the ground-level road beneath it when they
      # share the same name, so county-road classification is left to vision
      # lane count which reads the actual road geometry.
      way_ref = mapd.wayRef
      road_id = mapd.roadName or way_ref
      if road_id:
        hw = ''
        if way_ref:
          if way_ref.startswith('G'):
            hw = 'motorway'   # national expressway — 120 km/h
          elif way_ref.startswith('S'):
            hw = 'trunk'      # provincial expressway — 100 km/h
        hw_rank = {'motorway': 4, 'trunk': 3}
        if road_id != self.last_road_id:
          self.last_road_id = road_id
          self.last_highway_type = hw
        elif hw_rank.get(hw, -1) > hw_rank.get(self.last_highway_type, -1):
          # Same road, higher-rank ref seen — promote (never demote)
          self.last_highway_type = hw

    # --- Read lane data from vision model ---
    if self.sm.updated['modelV2']:
      model = self.sm['modelV2']
      raw_lane_count = infer_lane_count(model)

      # Directional hysteresis: fast to promote (2 s), slow to demote (5 s).
      # Once vision confirms a wide road, a brief occlusion won't drop the count.
      if raw_lane_count != self.lane_count:
        self.lane_count = raw_lane_count
        self.lane_count_stable_since = now
      else:
        going_down = raw_lane_count < self.lane_count_stable
        stability_window = 5.0 if going_down else 2.0
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

    # --- YOLO timeout ---
    if self.yolo_speed > 0 and (now - self.yolo_last_seen) > self.yolo_timeout:
      self.yolo_speed = 0

    # --- Priority cascade ---
    osm_speed = round(self.last_osm_speed) if self.last_osm_speed > 0 else 0
    yolo_speed = self.yolo_speed

    # Urban expressways without a G/S highway ref (like 中环路, 北翟高架路) are classified
    # as 'freeway' by mapd but their actual speed limit is trunk-class (80 km/h), not
    # motorway-class (120 km/h). Treat them as 'city' for inference.
    road_ctx_for_infer = self.last_road_context
    if not self.last_way_ref and road_ctx_for_infer == 'freeway':
      road_ctx_for_infer = 'city'

    inferred_speed = infer_speed_from_road_type(
      self.last_highway_type, self.lane_count_stable, road_ctx_for_infer
    )

    # Vision cap: when vision confidently sees ≤2 lanes, cap inferred speed.
    # Only apply when lane_count_stable < 3 — on confirmed multi-lane roads the
    # outermost lane line probability naturally fluctuates below the cap threshold
    # without implying a narrow road, so the cap would fire spuriously.
    if self.vision_cap_stable > 0 and self.lane_count_stable < 3:
      inferred_speed = min(inferred_speed, self.vision_cap_stable)

    MIN_SPEED_LIMIT = 30   # km/h — no real road is below this
    MAPD_UNCONSTRAINED = 130  # km/h — mapd's default ceiling when no OSM/curve data

    raw_suggested = round(self.last_mapd_suggested) if self.last_mapd_suggested > 0 else 0
    # Treat mapd's unconstrained ceiling as "no data" — fall through to own inference.
    # Also ignore suggestedSpeed when mapd has no road data (no wayRef, no OSM lane count):
    # in that case mapd is just outputting its default ceiling, not a meaningful constraint.
    # Only trust mapd's suggestedSpeed when we have a real OSM way reference.
    # mapd reports osm_lanes > 0 even on roads it detects visually (no OSM data),
    # so osm_lanes alone is not a reliable indicator. Without wayRef, mapd has no
    # speed limit data and its visionCurveSpeed can oscillate wildly on curves.
    mapd_has_road_data = bool(self.last_way_ref)
    mapd_suggested = raw_suggested if raw_suggested < MAPD_UNCONSTRAINED and mapd_has_road_data else 0

    # Take minimum across all available sources — most conservative valid reading wins.
    # mapd contributes curve constraints, inference contributes road-type knowledge,
    # YOLO contributes sign readings. No single source is trusted exclusively.
    candidates = []
    if yolo_speed >= MIN_SPEED_LIMIT:
      candidates.append((float(yolo_speed), 1, 0.8))    # yoloDetection
    if mapd_suggested >= MIN_SPEED_LIMIT:
      candidates.append((float(mapd_suggested), 3, 0.6)) # mapdSuggested
    candidates.append((float(max(inferred_speed, MIN_SPEED_LIMIT)), 2, round(self.lane_conf, 2)))  # roadTypeInference

    speed_limit, source, confidence = min(candidates, key=lambda x: x[0])

    # --- Confirmation management ---
    # Process toggle commands from carstate resume button / UI tap via plugin bus
    if self._cmd_sub is not None:
      cmd = self._cmd_sub.drain()
      if cmd is not None:
        _, data = cmd
        if data.get('action') == 'toggle_confirm':
          self.confirmed = not self.confirmed

    # Track current limit for the planner (always follows detected limit)
    if self.confirmed:
      self.confirmed_value = speed_limit
    else:
      self.confirmed_value = 0.0

    # --- Publish ---
    self._sl_pub.send({
      'speedLimit': snap_to_standard_speed(int(speed_limit)),
      'source': source,
      'confirmed': self.confirmed,
      'confidence': confidence,
      'osmMaxspeed': osm_speed,
      'yoloSpeed': yolo_speed,
      'inferredSpeed': inferred_speed,
      'mapdSuggested': mapd_suggested,
      'highwayType': self.last_highway_type,
      'roadName': self.last_road_name,
      'laneCount': self.lane_count_stable,
    })


def main():
  middleware = SpeedLimitMiddleware()
  rk = Ratekeeper(5, print_delay_threshold=None)  # 5 Hz

  while True:
    middleware.update()
    rk.keep_time()


if __name__ == "__main__":
  main()
