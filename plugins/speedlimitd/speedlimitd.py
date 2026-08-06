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
import re
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

  # --- Fix G: bounded-road demote — "3 lines with both edges → 2 lanes" -------
  # Canonical rule (user decision, 2026-08-05): when EXACTLY 3 lane lines are
  # visible AND both road edges are "bounded" — each edge nearly-confident (std
  # below DEMOTE_FAR_STD_MAX) AND hugging the outermost visible line on its side
  # (|edge y - outer line y| below DEMOTE_EDGE_GAP_MAX, both read at y-index 2,
  # ~10 m) — the road is physically closed and holds exactly 2 lanes: the count
  # is 2 (→ 40 km/h) and the edge boost below is naturally unreachable (base
  # 2 < 3). Side selection / y-index / visibility mirror _near_road_edge exactly
  # (the replay gate replicated it): outermost left = idx 0 if probs[0]>0.3 else
  # 1; outermost right = idx 3 if probs[3]>0.3 else 2; positions at y-index 2.
  # Config DEMOTE_FAR_STD_MAX=0.9 / DEMOTE_EDGE_GAP_MAX=1.5 is the replay-gated
  # grid point s0.9/g1.5.
  #
  # ACCEPTED TRADE / DESIRED BEHAVIOR (user decision 2026-08-05, made with the
  # replay-gate numbers in view — same precedent style as the edge-boost gate
  # below). The gate catches ~83% of the driver-audited 3e5 ramp exemplar but
  # also fires on barrier/wall-adjacent 3-line highway stretches, in runs that
  # can exceed the 3 s narrow-confirmation threshold (~+5 narrow commits per
  # ~4.5 h battery, each briefly showing 40). Per the user: a wall or barrier
  # hugging the outer line IS a real
  # road edge — most often a construction zone occupying lanes, where the human
  # response is to slow down, so demoting to 40 there is the CORRECT read, not a
  # false positive. The genuine residual cost is only the subset where the
  # barrier is a permanent sound wall / median divider at full lane count; that
  # remains gas-overridable. (Field audits 2026-08-05: both replay-flagged
  # "false fire" specimens — 3d0 seg34 connector, seg56 frontage — proved to be
  # genuine 2-lane roads; the practical residual is smaller than gated.) Pinned in
  # test_demote_accepted_wide_road_characterization AS desired construction-
  # squeeze behavior (with the permanent-barrier residual noted) so any future
  # change to it is a conscious one.
  DEMOTE_FAR_STD_MAX = 0.9   # each edge std must be below this to count as bounded
  DEMOTE_EDGE_GAP_MAX = 1.5  # edge must hug its outermost line within this (m) @ ~10 m
  if (visible_lines == 3 and hasattr(model_msg, 'roadEdges') and
      len(model_msg.roadEdges) >= 2 and hasattr(model_msg, 'roadEdgeStds') and
      len(model_msg.roadEdgeStds) >= 2):
    re_stds = model_msg.roadEdgeStds
    try:
      ll_y = [model_msg.laneLines[i].y[2] for i in range(4)]
      re_y = [model_msg.roadEdges[i].y[2] for i in range(2)]
    except (IndexError, AttributeError):
      ll_y = None
    if ll_y is not None:
      left_idx = 0 if probs[0] > 0.3 else (1 if probs[1] > 0.3 else None)
      right_idx = 3 if probs[3] > 0.3 else (2 if probs[2] > 0.3 else None)
      left_bounded = (left_idx is not None and re_stds[0] < DEMOTE_FAR_STD_MAX and
                      abs(re_y[0] - ll_y[left_idx]) < DEMOTE_EDGE_GAP_MAX)
      right_bounded = (right_idx is not None and re_stds[1] < DEMOTE_FAR_STD_MAX and
                       abs(re_y[1] - ll_y[right_idx]) < DEMOTE_EDGE_GAP_MAX)
      if left_bounded and right_bounded:
        return 2

  # Edge boost: when the car is driving next to the road edge and the vision lane
  # count is >= 3, vision likely misses the far-side lane(s) — assume actual =
  # vision_lane_count + 1 (cap 4). At <= 2 visible lanes next to an edge the
  # evidence indicates a genuinely narrow road/ramp, not a wide road with an
  # unseen far side — no boost (user definition, 2026-08-03). Route 3de seg 19: a
  # >=2 boost inflated an exit link's honest 2-line reading, defeating the narrow
  # confirmation, the lane≤2 G/S escape, and the ramp-40.
  #
  # ACCEPTED TRADE (user decision 2026-08-03, made with the replay + review
  # numbers in view). Deploy-gate replay: gating to >=3 adds ~1 wide-road
  # spurious-40 per ~2 h (龙东大道-class, gas-overridable), and — via a ≥3 s
  # sustained edge-lane MISREAD reading base 2 on a genuine expressway — can
  # release a 100/120 G/S hold to 40 (review F1). Both are accepted in exchange
  # for the counter reporting what vision actually sees on narrow roads; the
  # release edge is damped by the 3 s narrow accumulator + the display step
  # ladder. See the F1 characterization test, asserted AS accepted behavior so a
  # future change to it is a conscious one.
  if base_count >= 3:
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
REACT_MIN_SPEED = 30.0 / 3.6     # m/s (~29 km/h) — below this the cap defers to the
                          # driver: prevents ratchet-to-zero when measured a_y
                          # is speed-independent (banking, yaw bias, a
                          # tightening spiral) and kills parking-speed noise
                          # engagement.
REACT_LIVEPOSE_STALE_S = 1.0  # s — no livePose update for longer than this
                               # while engaged forces the release ramp: a
                               # stalled localizer must not latch a speed cap
                               # indefinitely. Held-slow is safe briefly; an
                               # unbounded latch is not.


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


# --- Distance-aware curve approach + tight-curve a_y derating (route 3d0) ---
COMFORT_BRAKE = 0.8  # m/s² — matches DCC's comfortable decel envelope so the
                     # commanded speed profile is actually ACHIEVABLE. Route 3d0
                     # seg 61: the first hairpin leg was entered at 44 km/h
                     # against a 29 km/h cap because nothing planned the
                     # deceleration POINT — DCC bleeds only ~1 m/s², so a
                     # late-seen sharp curve arrives over-speed. v_now =
                     # sqrt(v_curve² + 2·a·d) is the speed at which braking at
                     # COMFORT_BRAKE over the remaining distance d still makes
                     # the curve; the cap thus tightens as the curve nears
                     # instead of issuing an un-followable one-shot drop.
TIGHT_CURVE_FACTOR = 0.75  # a_y target derate at hairpin-class curvature. Route
                           # 3d0 seg 60: even at the comfort target the model
                           # cuts toward the inner line (gap 0.05 m at a correct
                           # 32 km/h); a lower a_y target buys lateral margin and
                           # driver reaction time. The inner-line cut itself is
                           # model-level (0.11.2 is the yardstick) — this only
                           # buys margin around it.


def _interp(x: float, xp: list, fp: list) -> float:
  """Clamped 1-D linear interpolation for a single scalar (no numpy dep)."""
  if x <= xp[0]:
    return fp[0]
  if x >= xp[-1]:
    return fp[-1]
  for i in range(1, len(xp)):
    if x < xp[i]:
      t = (x - xp[i - 1]) / (xp[i] - xp[i - 1])
      return fp[i - 1] + t * (fp[i] - fp[i - 1])
  return fp[-1]


def curvature_speed_cap(model_msg, max_lat_accel: float = 1.5, kappa_meas: float | None = None) -> int:
  """Cap speed based on predicted path curvature within reliable vision.

  Looks at the model's predicted yaw rate and velocity over a horizon
  bounded by:
    - time:     T_IDXS index 30 (≈ 8.8 s, model's prediction limit)
    - distance: model's confidence boundary (yStd-based, capped at 100 m)
  Whichever bound is tighter wins. Beyond the confidence boundary the
  model extrapolates 'straight ahead' and predictions are noise.

  Distance-aware approach (route 3d0): rather than mapping the single max κ
  to a comfort speed, it makes a per-point pass over the confidence-gated
  path. For each meaningfully-curved point i it computes the curve speed
  there — v_curve_i = sqrt(target_ay_i / |κ_i|), with target_ay_i derated on
  tight curvature (Change 2) — and the speed allowed NOW so a COMFORT_BRAKE
  deceleration still bleeds down to it by the time the curve is reached:
  v_now_i = sqrt(v_curve_i² + 2·COMFORT_BRAKE·d_i), d_i = position.x[i]. The
  binding cap is the minimum v_now over points, so a near mild curve and a
  far sharp curve compete on equal (achievable-profile) footing, and the cap
  tightens as a curve nears. A point at d≈0 reduces to v_now = v_curve — the
  original distance-less behaviour — exactly.

  The target lateral acceleration is `max_lat_accel` (m/s²), wired from the
  MapdCurveTargetLatAccel param by the middleware (default 1.5, clamped
  [1.0, 3.0]).

  The return value is the RAW ramp value (rounded to 1 km/h, floored at
  30 km/h) — it is NOT snapped to a standard speed. Snapping the enforcement
  value quantized the smooth distance ramp away in the gaps between standard
  speeds (e.g. a real, tightening 92 km/h constraint snapped to 100 — which
  reads as "no constraint yet" — and then cliffed straight to 80 once the
  curve was almost on top of the car, instead of gently tightening 92 → 80).
  Callers that need a display-clean value must call snap_to_standard_speed()
  themselves at the point where the value is shown/published — see
  SpeedLimitMiddleware.update().

  `kappa_meas` (route 3d0 seg 60 apex fix): the driver's OWN measured
  curvature, |yaw_rate|/max(v_ego, 0.1) from the same livePose signal the
  reactive cap already consumes, passed in only when that reading is fresh
  and valid. When given and above CURVE_GATE it is folded in as a virtual
  d=0 path point (no braking-headroom term — the apex itself is the most
  confident path point there is) using the SAME derate interp and floor as
  every other point, and only ever tightens the result (min semantics).

  Returns speed cap in km/h, or 0 if no meaningful constraint (including when
  the raw, unsnapped value is >= 100 km/h — the curve is far/mild enough that
  no slowdown is needed yet).
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

  CURVE_GATE = 0.003  # negligible curvature (~330 m radius)

  # T_IDXS = 10 * (i/32)^2:  i=10 → 1.0s   i=22 → 4.7s   i=30 → 8.8s
  # Per-point pass within both time AND distance bounds. d_i is position.x[i]
  # (forward distance along the path — adequate at ≤100 m, see note below).
  cap_ms = None
  for i in range(5, min(31, len(yaw_rates), len(positions_x), len(velocities))):
    d_i = positions_x[i]
    if d_i > conf_dist:
      break  # past confident vision — extrapolation noise
    v = max(velocities[i], 5.0)  # floor at 5 m/s to avoid division issues
    kappa = abs(yaw_rates[i]) / v
    if kappa < CURVE_GATE:
      continue  # this point is effectively straight
    # Change 2 — derate the comfort a_y target on tight (hairpin) curvature.
    target_ay = max_lat_accel * _interp(kappa, [0.02, 0.035], [1.0, TIGHT_CURVE_FACTOR])
    v_curve = math.sqrt(target_ay / kappa)
    # Change 1 — speed allowed now so COMFORT_BRAKE still makes the curve at
    # d_i. d≈0 → v_now = v_curve (the original distance-less behaviour).
    v_now = math.sqrt(v_curve * v_curve + 2.0 * COMFORT_BRAKE * max(d_i, 0.0))
    cap_ms = v_now if cap_ms is None else min(cap_ms, v_now)

  # 2026-07-28 (route 3d0 seg 60 apex fix): the sharpest path point the model
  # sees always sits at d>0, so the distance term (2·COMFORT_BRAKE·d_i) keeps
  # inflating v_now above the derated target — the car never actually reaches
  # the intended derated speed until the apex is basically behind the
  # confidence horizon (seg 60: distance term held the cap ~4 km/h above the
  # derated apex target — delivered ~31, intended ~27). The apex itself is
  # the most confident path point there is: measured curvature delivers the
  # derated target exactly where the model's cutting makes margin matter.
  # Division of labor stays clean — this is still the vision/curve cap
  # (target-a_y-based, floored 30); the reactive cap (2.5 m/s² threshold)
  # remains the separate excursion backstop.
  if kappa_meas is not None and kappa_meas > CURVE_GATE:
    target_ay_meas = max_lat_accel * _interp(kappa_meas, [0.02, 0.035], [1.0, TIGHT_CURVE_FACTOR])
    v_virtual = math.sqrt(target_ay_meas / kappa_meas)  # d=0 — no headroom term
    cap_ms = v_virtual if cap_ms is None else min(cap_ms, v_virtual)

  if cap_ms is None:  # no meaningfully-curved point and no virtual apex point
    return 0

  safe_speed_kph = cap_ms * 3.6

  if safe_speed_kph >= 100:
    return 0  # curve far/mild enough that no slowdown is needed yet — RAW check

  # Enforcement floor: no real road curve constraint is below the lowest
  # standard speed (30). This used to be provided incidentally by
  # snap_to_standard_speed (30 is the nearest standard for anything below
  # 35) — now that the return value is unsnapped, clamp explicitly.
  CURVE_CAP_FLOOR_KPH = 30
  return max(CURVE_CAP_FLOOR_KPH, round(safe_speed_kph))


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


# --- Lane-count-first speed-limit inference (2026-07-28, route 3d0) ----------
# Route 3d0 segs 41–49 (elevated ring road, official 80): the OSM road_id flipped
# 37 times in 8 min among stacked ways — trunk 80 / primary 60 / residential 50
# / trunk_link 40, surface streets 0.2–13 m beneath the elevated way in 2D — while
# the vision lane count sat rock-stable at 4. The offline tiles carry no
# layer/bridge/altitude to disambiguate the stack, so OSM road-type inference is
# pure noise on stacked/elevated geometry. Redesign: the vision lane count drives
# the limit on ordinary roads; OSM is trusted ONLY for G/S expressways, where the
# EXISTING promote mechanism (wayRef class x lane count via
# infer_speed_from_road_type — G+≥3 lanes → 120, S+≥3 lanes → 100) is preserved
# verbatim and made sticky for GS_STICKY_S to ride out momentary flips to a
# stacked non-G/S way (the 100→80 transient).
GS_STICKY_S = 30.0          # absolute ceiling on the sticky expressway hold
GS_RELEASE_CONT_S = 10.0    # release once non-G/S matches have been CONTINUOUS
                            # this long — a genuine exit connector accumulates it
                            # in one run; an alternating stacked flicker keeps
                            # resetting it (stickiness preserved). Bounds a stale
                            # 100/120 hold well under the 30 s ceiling (a wide
                            # gentle exit must not defeat safety caps for 30 s).
GS_RELEASE_MARGIN_M = 8.0   # release when the matched non-G/S way is this much
                            # CLOSER than the held G/S way — the car is decisively
                            # ON another road. An absolute distance gate on the
                            # held way does NOT work: fresh 3de seg 19 tile
                            # measurement shows the held S1 polyline only 13.5 m
                            # away at ramp entry (diverging 0.84 m/s, never past
                            # 21 m in the segment), so a 25 m gate would fire
                            # later than the 10 s timer. The generic
                            # discriminator is instead the MARGIN between the
                            # matched way and the held way: on a genuine exit the
                            # car sits on the ramp (matched ≈ 0.6 m) while the
                            # held expressway recedes (margin ≈ 13 m); a stacked
                            # mis-match matches a way essentially co-located with
                            # the held one (margin 0.2-5 m, 3d0 forensics). Held
                            # way ABSENT from candidates ⇒ margin +inf.
GS_RELEASE_MARGIN_QUERIES = 2  # consecutive OSM queries (~5 s cadence) the
                            # margin must hold before releasing — one divergent
                            # query could be a single mis-query; two in a row is
                            # the car committed to the other road. Still bounded
                            # well under the 10 s absence timer and 30 s ceiling.
GS_LANE_DROP_S = 1.5        # release path 2 (route 3de user addition): while the
                            # held G/S ref has stopped matching (ref-empty absence
                            # run active), a RAW lane count (infer_lane_count) that
                            # holds ≤2 continuously this long is the second,
                            # independent exit signal — two signals agreeing = an
                            # unambiguous exit onto a narrow ramp, needing NO OSM
                            # candidate-distance data. Deliberately HALF the
                            # general 3 s narrow confirmation (NARROW_CONFIRM_S):
                            # that 3 s must reject noise from vision ALONE; here
                            # the ref-empty corroboration is a second independent
                            # signal, so a shorter window is justified. NOTE: on
                            # 3de seg 19 this path would NOT have fired — the
                            # edge-aware lane count read ≥3 on that wide ramp for
                            # ~15 s; the margin rule (path 1) covers that geometry.
                            # This path covers genuinely-narrow exits and is robust
                            # when candidate distances are unavailable.
LANE_COUNT_LIMIT_3 = 60     # 3 confident lanes → 60 km/h
LANE_COUNT_LIMIT_4 = 80     # ≥4 confident lanes → 80 km/h

# Noise-tolerant narrow-band (≤2 lane) confirmation (route 3d3 seg 16 / 3d1 seg 29).
# A single directional debounce timer resets on ANY raw-count change, so on a
# genuine 2-lane ramp with brief 3↔4 occlusion spikes the demotion window keeps
# restarting and lane_count_stable never commits to 2 (seg 16: raw ≤2 for 24 s
# continuous, never committed). Instead we integrate a LEAKY "time-in-narrow"
# accumulator: it ADDS dt while raw ≤2 and bleeds back at NARROW_DECAY·dt while
# raw ≥3 (asymmetric — narrow evidence sticks longer than a spike erases it),
# clamped to [0, cap]. Reaching NARROW_CONFIRM_S commits lane_count_stable to the
# sustained narrow value. A genuine ramp (raw ≤2 for seconds, occasional
# single-frame occlusion blip) climbs to the threshold in ~3-4 s; a sub-3 s
# transient dip (occlusion reads cluster at ~0.1 s, 95% of all ≤2 reads) never
# reaches it and is rejected. 3 s cleanly separates the two classes (genuine
# ramps run ≥4 s). Only the DEMOTION-into-narrow path uses this; promotion back
# to ≥3 lanes and 3↔4 transitions keep the existing directional debounce.
NARROW_CONFIRM_S = 3.0      # sustained (leaky) time-in-narrow before committing ≤2
NARROW_ACCUM_CAP = 3.0      # accumulator clamp (== confirm threshold; no banking)
NARROW_DECAY = 0.5          # bleed fraction of dt while raw ≥3 (occlusion spike)

# China expressway ref grammar (国家高速 / 省级高速 numbering system):
#   [GS]\d{1,2}  national/provincial expressway TRUNKS — G2, G15, S1, S20
#   [GS]\d{4}    regional ring / spur expressways      — G1501 (Shenyang ring)
#   [GS]\d{3}    ordinary guodao/shengdao SURFACE highways — G312, S203 — these
#                are NOT controlled-access expressways (official ~80 km/h) and
#                MUST route through lane-count mode, not the OSM promote. The
#                3-digit exclusion is the whole point: it keeps the noisy OSM
#                road-type path away from ordinary national/provincial roads.
_GS_EXPRESSWAY_RE = re.compile(r'^[GS](\d{1,2}|\d{4})$')


def is_gs_expressway_ref(way_ref: str) -> bool:
  """True iff way_ref is a controlled-access G/S expressway ref (see grammar).

  1–2 digit and 4-digit G/S refs are expressways; a 3-digit ref is an ordinary
  guodao/shengdao surface highway and is deliberately excluded.
  """
  return bool(_GS_EXPRESSWAY_RE.match(way_ref))


def lane_count_limit(lane_count: int) -> int:
  """Speed limit from the confident vision lane count (non-expressway roads).

  ≤2 lanes: vision can't tell a through road from a link/ramp, so the speed is
  the EXISTING narrow-road sub-table, UNCHANGED — delegated verbatim to
  infer_speed_from_road_type's lane-count shortcut (2 → 40, 1 → 30). 3 → 60,
  ≥4 → 80. See the GS_* block above for the route-3d0 rationale.
  """
  if lane_count <= 2:
    return infer_speed_from_road_type('', lane_count, 'city')
  if lane_count == 3:
    return LANE_COUNT_LIMIT_3
  return LANE_COUNT_LIMIT_4


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
    # G/S expressway stickiness (2026-07-28, route 3d0): once a G/S match is
    # seen, hold expressway classification (and its promote-derived limit) for
    # GS_STICKY_S even if the instantaneous match flips to a stacked non-G/S way.
    self._gs_last_seen_t: float = 0.0  # monotonic time of last G/S match
    self._gs_limit_kph: int = 0        # promote-derived limit held during stickiness
    self._gs_absent_since: float | None = None  # start of the current continuous
                                                 # non-G/S run (None ⇒ last match
                                                 # was G/S)
    # Distance-guarded fast release (route 3de seg 19). Held-way identity + the
    # two independent divergence signals that let the sticky hold drop before
    # the 10 s absence timer when the car has clearly left the expressway.
    self._gs_held_ref: str = ''         # ref of the currently-held G/S way
                                        # ('' ⇒ none, or a ref-less motorway hold
                                        # with no trackable identity)
    self._gs_force_release: bool = False  # margin-rule (path 1) divergence latch;
                                          # sticky until a genuine G/S re-match
    self._gs_margin_count: int = 0        # consecutive divergent-margin OSM queries
    self._gs_lane_drop_since: float | None = None  # start of the continuous RAW
                                                    # ≤2 lane run (path 2); None ⇒
                                                    # last raw read was ≥3
    self.inference_mode: str = 'lane_count'  # 'gs_osm' | 'lane_count'
    self.lane_count: int = 1
    self.lane_count_stable: int = 1
    self.lane_count_stable_since: float = 0.0
    self.lane_count_locked: bool = False  # True once vision has a 2 s stable reading
    # Leaky narrow-band (≤2) confirmation state (see NARROW_* constants).
    self._narrow_accum: float = 0.0       # net time-in-narrow, seconds, [0, cap]
    self._lane_last_t: float = 0.0        # last modelV2 tick time, for the accumulator dt
    self.lane_conf: float = 0.0           # smoothed lane line confidence (0.0–1.0)
    self.vision_cap: int = 0
    self.vision_cap_stable: int = 0
    self.vision_cap_stable_since: float = 0.0
    self.curvature_cap: int = 0  # RAW enforcement value (unsnapped, see curvature_speed_cap);
                                  # snapped to a standard speed only where displayed/published
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
    self._react_livepose_last_t: float = 0.0  # monotonic time livePose last updated

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
                           now: float, dt: float,
                           livepose_updated: bool = True) -> float:
    """Reactive measured-a_y speed cap (2026-07-28 layer contract).

    Low-passes |a_y_meas| (~0.3 s). When it exceeds `threshold` continuously
    for REACT_ENGAGE_S (and v_ego is at least REACT_MIN_SPEED), engages a cap
    v_react = v_ego·sqrt(threshold/|a_y|) minus a 1 m/s hysteresis margin.
    While engaged the cap may only move DOWN or hold — it never chases a_y
    upward — and is floored at REACT_MIN_SPEED (below that the cap defers to
    the driver rather than ratcheting toward zero; see the constant comment).
    Release ramps the cap back up at REACT_RELEASE_RATE once
    |a_y| < threshold−REACT_QUIET_MARGIN for REACT_QUIET_S, or immediately if
    livePose has gone stale (no update for REACT_LIVEPOSE_STALE_S) while
    engaged — a stalled localizer must not latch a cap indefinitely.
    `threshold <= 0` disables the feature entirely.

    State lives on self; returns the current cap (m/s, 0 = disengaged). Kept a
    pure function of its inputs so it is unit-testable without livePose msgs;
    `livepose_updated` defaults True so existing pure-function callers/tests
    are unaffected.
    """
    if livepose_updated:
      self._react_livepose_last_t = now

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

    # A livePose that has stopped updating while a cap is latched must not be
    # allowed to hold that cap forever — force the release path immediately.
    stale = (self._react_cap_ms > 0.0
             and self._react_livepose_last_t > 0.0
             and now - self._react_livepose_last_t > REACT_LIVEPOSE_STALE_S)

    if self._react_cap_ms <= 0.0:
      # Not engaged — engage after sustained over-threshold, but never below
      # REACT_MIN_SPEED: at parking-lot speeds the driver, not this cap,
      # should be in charge.
      if (self._react_over_since is not None
          and now - self._react_over_since >= REACT_ENGAGE_S
          and ay > 0.0 and v_ego >= REACT_MIN_SPEED):
        v_react = v_ego * math.sqrt(threshold / ay) - REACT_HYST_MS
        self._react_cap_ms = max(v_react, REACT_MIN_SPEED)
    else:
      # Engaged.
      if stale or (self._react_quiet_since is not None
          and now - self._react_quiet_since >= REACT_QUIET_S):
        # Sustained quiet, or a stale localizer — ramp the cap back up,
        # disengage once it no longer constrains (reaches current speed).
        self._react_cap_ms += REACT_RELEASE_RATE * dt
        if self._react_cap_ms >= max(v_ego, 0.0):
          self._react_cap_ms = 0.0
      elif ay > 0.0 and v_ego > 0.0:
        # Still cornering (or not quiet long enough): monotonic-down / hold,
        # floored at REACT_MIN_SPEED while engaged.
        v_react = v_ego * math.sqrt(threshold / ay) - REACT_HYST_MS
        self._react_cap_ms = min(self._react_cap_ms, max(v_react, REACT_MIN_SPEED))
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

      # Highway type from wayRef — ONLY true G/S expressways promote (same
      # grammar as the gs_mode gate, see is_gs_expressway_ref): G expressway →
      # motorway (120 km/h), S expressway → trunk (100 km/h). Ordinary 3-digit
      # guodao/shengdao (G312, S203) classify as '' so gs_mode never sees them.
      road_id = result['roadName'] or way_ref
      hw = ''
      if is_gs_expressway_ref(way_ref):
        hw = 'motorway' if way_ref[0] == 'G' else 'trunk'
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

    # Per-query margin-rule evaluation (route 3de seg 19). _ingest_osm_result is
    # the once-per-OSM-query hook (both on-device and in tests), so the
    # consecutive-query margin count is tracked here, not in the 5 Hz update().
    self._eval_gs_margin_release(result)

  def _eval_gs_margin_release(self, result: dict | None):
    """Margin-rule fast release for the sticky G/S hold (route 3de seg 19).

    Called once per OSM query. While a G/S hold is active and THIS query's best
    match is non-G/S, the car has decisively moved onto another road when the
    matched way is GS_RELEASE_MARGIN_M closer than the held G/S way — or the
    held way has dropped out of the candidate set entirely (margin +inf) — for
    GS_RELEASE_MARGIN_QUERIES consecutive such queries. An absolute distance
    gate on the held way does not work: at seg-19 ramp entry the held S1
    polyline is only ~13.5 m off (a 25 m gate would fire later than the 10 s
    timer). The MARGIN is the generic discriminator — a genuine exit sits the
    car on the ramp (matched ≈ 0.6 m) while the held expressway recedes
    (margin ≈ 13 m); a stacked mis-match matches a way co-located with the held
    one (margin 0.2-5 m, 3d0 forensics).

    Sets the sticky _gs_force_release latch (cleared only by a genuine G/S
    re-match, below). Distance unavailable (no refDistances / no result / no
    matched distance) ⇒ leave the count and latch untouched: the 10 s absence
    timer is the fallback.
    """
    current_is_gs = (is_gs_expressway_ref(self.last_way_ref)
                     or self.last_osm_hwtype == 'motorway')
    if current_is_gs:
      # A genuine G/S match (re-)establishes the held identity and clears all
      # margin-divergence evidence — the hold is fresh. A ref-less motorway hold
      # has no trackable ref, so the margin path is disabled for it ('' held
      # ref) and the timer/lane-drop paths govern instead.
      self._gs_held_ref = self.last_way_ref if is_gs_expressway_ref(self.last_way_ref) else ''
      self._gs_force_release = False
      self._gs_margin_count = 0
      return
    if not self._gs_held_ref:
      return  # no trackable held identity — margin path disabled
    ref_dists = result.get('refDistances') if result else None
    matched_dist = result.get('distance') if result else None
    if ref_dists is None or matched_dist is None:
      return  # distance unavailable → timer fallback, evidence untouched
    held_dist = ref_dists.get(self._gs_held_ref)
    margin = math.inf if held_dist is None else held_dist - matched_dist
    if margin > GS_RELEASE_MARGIN_M:
      self._gs_margin_count += 1
      if self._gs_margin_count >= GS_RELEASE_MARGIN_QUERIES:
        self._gs_force_release = True
    else:
      self._gs_margin_count = 0

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

    # --- Reactive measured-a_y cap + measured-curvature apex point (2026-07-28) ---
    # Read ahead of the modelV2 block below so this tick's measured curvature
    # is available to curvature_speed_cap() as a same-tick virtual apex point
    # (route 3d0 seg 60 apex fix — see kappa_meas there). Runs every tick (not
    # gated on modelV2) off the localizer's measured yaw. a_y_meas = v · yaw_rate;
    # both from livePose (vehicle-agnostic, no steering ratio). Catches curves
    # the proactive vision cap missed and plant amplification — the ISO
    # 3.0 m/s² defense lives here (see __init__).
    react_dt = min(max(now - self._react_last_t, 0.0), 1.0) if self._react_last_t else 0.2
    self._react_last_t = now
    a_y_meas = None
    v_ego_meas = 0.0
    kappa_meas = None  # |yaw_rate| / max(v_ego, 0.1) — reused below as the
                        # curvature_speed_cap() virtual apex point; stays None
                        # (no virtual point) unless livePose is fresh and valid
                        # THIS tick — reusing the exact same freshness/validity
                        # check as a_y_meas, no separate staleness tracking added.
    livepose_updated = self.sm.updated.get('livePose', False)
    if livepose_updated:
      try:
        lp = self.sm['livePose']
        av = lp.angularVelocityDevice
        vd = lp.velocityDevice
        if getattr(av, 'valid', True) and getattr(vd, 'valid', True):
          v_ego_meas = abs(float(vd.x))
          a_y_meas = v_ego_meas * float(av.z)
          kappa_meas = abs(float(av.z)) / max(v_ego_meas, 0.1)
      except Exception:
        a_y_meas = None
        v_ego_meas = 0.0
        kappa_meas = None
    self._update_reactive_cap(a_y_meas, v_ego_meas, self.react_lat_accel_threshold,
                              now, react_dt, livepose_updated)

    # --- Read lane data from vision model ---
    if self.sm.updated['modelV2']:
      model = self.sm['modelV2']
      raw_lane_count = infer_lane_count(model)

      # Per-tick dt for the leaky narrow-confirmation accumulator below. Clamp to
      # guard a stalled tick; 0 on the first sample.
      dt = min(max(now - self._lane_last_t, 0.0), 0.5) if self._lane_last_t > 0.0 else 0.0
      self._lane_last_t = now

      # Release path 2 (route 3de user addition) — RAW-lane-drop run timer. Track
      # a continuous raw ≤2 run off infer_lane_count directly (NOT the debounced
      # lane_count_stable), reset on any raw ≥3. Consumed only in conjunction with
      # the ref-empty absence condition in the G/S release block below; see
      # GS_LANE_DROP_S for why RAW + a 1.5 s window is safe here.
      if raw_lane_count <= 2:
        if self._gs_lane_drop_since is None:
          self._gs_lane_drop_since = now
      else:
        self._gs_lane_drop_since = None

      # Committing UP / between wide counts: the existing directional debounce.
      # A steady raw reading commits after a stability window — 1.5 s going up, and
      # the demotion window (2 s curving / 5 s straight) for wide→less-wide drops
      # that stay ≥3. Demotion INTO the narrow band (≤2) is handled by the leaky
      # accumulator below instead: a single debounce timer resets on any raw
      # flicker and so never commits on a noisy 2-lane ramp (route 3d3 seg 16).
      curving = self.curvature_cap > 0  # curvature_speed_cap detected upcoming curve
      if raw_lane_count != self.lane_count:
        self.lane_count = raw_lane_count
        self.lane_count_stable_since = now
      elif raw_lane_count >= 3:
        going_down = raw_lane_count < self.lane_count_stable
        demotion_window = 2.0 if curving else 5.0
        stability_window = demotion_window if going_down else 1.5
        if now - self.lane_count_stable_since > stability_window:
          self.lane_count_stable = self.lane_count
          self.lane_count_locked = True
          # A committed widening is positive evidence the narrow section ended —
          # zero the leaky narrow accumulator (route 3e0 seg 33, 2026-08-04).
          # Without this the residual (it only drains 0.5·dt during wide
          # stretches) lets a stray ≤2 edge-lane dip re-commit narrow in ~2-3 s;
          # on the 五洲大道 exit that repeatedly yanked the climbing limit back
          # to 40-60, gating acceleration for 20.8 s (car crept 59→77). With the
          # reset, a re-narrow after a committed widening needs a fresh full
          # NARROW_CONFIRM_S (3.0 s). Accepted trade: a post-Y-fork re-narrow on a
          # genuine ramp arrives ~1.5 s later (3d4 seg 7 class — the apex curve
          # cap covers it).
          self._narrow_accum = 0.0

      # Committing DOWN into the narrow band (≤2): leaky NARROW_CONFIRM_S confirmation
      # (see NARROW_* constants). ADD dt while raw ≤2, bleed NARROW_DECAY·dt while
      # raw ≥3, clamp [0, cap]. A genuine 2-lane ramp climbs to the threshold in
      # ~3-4 s and commits; a sub-3 s transient dip never reaches it.
      # Commit to a FIXED 2 (→40), NOT the min raw seen while narrow: the commit
      # fires during the noisiest lane-loss moment, where a single occluded frame
      # can read raw=1, and both verified ramps (3d3 seg16 / 3d1 seg29) commit on
      # exactly such a frame — committing to that raw would read 30 instead of 40
      # on the very ramps this fixes. A true 1-lane→30 road would need its own
      # separate confirmation; 2→40 is the correct choice for the 2-lane target.
      if raw_lane_count <= 2:
        self._narrow_accum = min(self._narrow_accum + dt, NARROW_ACCUM_CAP)
      else:
        self._narrow_accum = max(self._narrow_accum - NARROW_DECAY * dt, 0.0)
      if self._narrow_accum >= NARROW_CONFIRM_S:
        self.lane_count_stable = 2
        self.lane_count_locked = True

      # Fix F post-narrow ceiling (592d39f) removed 2026-08-05 per user decision —
      # exit-release hold cost outweighed ghost-link protection (see project
      # record); ghost-link false-80s are accepted modelV2 artifacts pending a
      # better model.

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
      raw_curv_cap = curvature_speed_cap(model, self.curve_target_lat_accel, kappa_meas)
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

    # --- Base speed-limit inference — lane-count-first (2026-07-28, route 3d0) ---
    # OSM road-type inference is REMOVED for ordinary roads: on stacked/elevated
    # geometry the matched way churns 37×/8min while the vision lane count is
    # rock-stable, so the lane count drives the limit. OSM is trusted ONLY for
    # G/S expressways, where the EXISTING promote mechanism
    # (infer_speed_from_road_type — G/S ref × lane count) is preserved verbatim
    # and made sticky. A way is in G/S mode iff its ref is 'G'/'S'+digit OR its
    # OSM highway class is 'motorway'. G/S classification is sticky for
    # GS_STICKY_S so a momentary flip to a stacked non-G/S way can't drop the
    # expressway limit under the car (the 100→80 transient).
    is_gs_now = (is_gs_expressway_ref(self.last_way_ref)
                 or self.last_osm_hwtype == 'motorway')
    if is_gs_now:
      # Urban elevated expressways without a G/S ref (中环路, 北翟高架路) are tagged
      # 'freeway' by mapd but are trunk-class (80), not motorway (120) — demote
      # their context to 'city' for the promote, exactly as before.
      road_ctx_for_infer = self.last_road_context
      if not self.last_way_ref and road_ctx_for_infer == 'freeway':
        road_ctx_for_infer = 'city'
      self._gs_limit_kph = infer_speed_from_road_type(
        self.last_highway_type, self.lane_count_stable, road_ctx_for_infer,
        width_class=self.lane_width_class, osm_type=self.last_osm_hwtype,
      )
      self._gs_last_seen_t = now
      self._gs_absent_since = None        # a G/S match resets the absence run
      self._gs_lane_drop_since = None     # ...and the path-2 lane-drop run
    elif self._gs_last_seen_t > 0.0 and self._gs_absent_since is None:
      self._gs_absent_since = now         # first non-G/S tick of a new run

    # Path 2 (route 3de user addition) — ref-empty + narrow-drop conjunction:
    # the held G/S ref has stopped matching (absence run active) AND the RAW lane
    # count has held ≤2 for GS_LANE_DROP_S. Two independent signals agreeing = an
    # unambiguous exit onto a narrow ramp, with NO dependence on OSM candidate
    # distances (robust when refDistances is unavailable). See GS_LANE_DROP_S for
    # why RAW lane count and the half-length 1.5 s window are justified, and why
    # this path would NOT have fired on the (wide) 3de seg 19 ramp — the margin
    # rule (path 1, _gs_force_release) covers that geometry.
    gs_lane_drop = (self._gs_absent_since is not None
                    and self._gs_lane_drop_since is not None
                    and now - self._gs_lane_drop_since >= GS_LANE_DROP_S)

    # Release the sticky expressway hold on ANY of:
    #   - absolute ceiling (GS_STICKY_S) since the last G/S match;
    #   - the margin rule (path 1): the matched way is decisively closer than the
    #     held G/S way for GS_RELEASE_MARGIN_QUERIES queries (_gs_force_release);
    #   - the ref-empty + narrow-drop conjunction (path 2, gs_lane_drop above);
    #   - non-G/S matches CONTINUOUS for GS_RELEASE_CONT_S — a genuine exit
    #     accumulates it in one run; an alternating stacked flicker keeps
    #     resetting _gs_absent_since so it never accumulates (stickiness kept).
    #     This is the fallback when candidate distances are unavailable;
    #   - a narrow section (lane_count_stable ≤ 2, already debounced): a ramp/
    #     exit off the expressway must obey the narrow-road limit immediately,
    #     never a stale 100/120.
    gs_released = (
        self._gs_last_seen_t == 0.0
        or now - self._gs_last_seen_t > GS_STICKY_S
        or self._gs_force_release
        or gs_lane_drop
        or (self._gs_absent_since is not None
            and now - self._gs_absent_since >= GS_RELEASE_CONT_S)
        or self.lane_count_stable <= 2)
    gs_mode = not gs_released

    if gs_mode:
      # Hold the last promote-derived limit through momentary non-G/S flips.
      inferred_speed = self._gs_limit_kph
      self.inference_mode = 'gs_osm'
    else:
      # Lane-count debounce is the EXISTING lane_count_stable directional
      # hysteresis (up 1.5 s, down 2 s curving / 5 s straight) — the limit only
      # moves after a lane-count change has held, so a momentary lane-prob dip
      # can't flicker it.
      inferred_speed = lane_count_limit(self.lane_count_stable)
      self.inference_mode = 'lane_count'

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
    candidates.append((float(max(inferred_speed, MIN_SPEED_LIMIT)), 2, round(self.lane_conf, 2)))  # base inference (lane-count / gs_osm), source 2

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
      # Base-inference mode: 'gs_osm' (G/S expressway promote, sticky) or
      # 'lane_count' (vision lane-count table). Telemetry/observability only.
      'inferenceMode': self.inference_mode,
      'yoloSpeed': yolo_speed,
      'inferredSpeed': inferred_speed,
      'highwayType': self.last_highway_type,
      'osmHwType': self.last_osm_hwtype,
      'wayRef': self.last_way_ref,
      'roadName': self.last_road_name,
      'laneCount': self.lane_count_stable,
      'laneWidth': round(self.lane_width, 2),
      'laneWidthClass': self.lane_width_class,
      # self.curvature_cap is the RAW enforcement value (unsnapped); the
      # published/displayed cap is snapped to a standard speed at this one
      # publish site, same as speedLimit.
      'curvatureCap': snap_to_standard_speed(self.curvature_cap) if self.curvature_cap >= MIN_SPEED_LIMIT else 0,
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
