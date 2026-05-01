import numpy as np
from config import read_plugin_param
from openpilot.common.realtime import DT_MDL
try:
  from openpilot.common.swaglog import cloudlog
except ImportError:
  import logging
  cloudlog = logging.getLogger(__name__)
from openpilot.selfdrive.controls.lib.drive_helpers import smooth_value


class LaneCenteringCorrection:
  """Curvature correction to center car in lane during turns.

  v10: Adds kP compensation, rate limiting, and derivative damping to
  prevent oscillation caused by latcontrol_torque's speed-dependent kP.

  Usage:
    lcc = LaneCenteringCorrection()
    correction = lcc.update(model_v2, v_ego)
  """

  # Curvature-dependent K: sharper turns need stronger correction. Extended
  # to curvature=0 (straights) so the correction engages whenever lateral
  # offset exceeds threshold, not only in curves.
  K_BP = [0.000, 0.002, 0.005, 0.008, 0.012, 0.020]  # Curvature breakpoints (1/m)
  K_V  = [0.150, 0.150, 0.350, 0.400, 0.500, 0.650]   # K for straight matches gentle curves (~2s closure)

  # Curvature-adaptive minimum confidence: lane line confidence drops in tight
  # turns due to perspective distortion at the lookahead point — detections
  # are at stable positions but model marks them low confidence. Linearly
  # relax MIN_PROB from straight (full noise filter) to a low floor (tight
  # turns), so we keep using detected-but-low-confidence lines when the
  # alternative is bailing entirely (route 2b1 seg 24 case: both probs at
  # 0.15-0.25 during κ=0.011 apex with positions varying by < 5cm).
  #   MIN_PROB(κ) = max(MIN_PROB_FLOOR, MIN_PROB_STRAIGHT − SLOPE · |κ|)
  MIN_PROB_STRAIGHT     = 0.5    # full noise filtering on straights; also "strong" gate for selection
  MIN_PROB_FLOOR        = 0.15   # minimum confidence even in tightest turns
  MIN_PROB_CURV_SLOPE   = 30.0   # 1/m → drops 0.5 → 0.15 over κ ∈ [0, 0.012]
  MIN_SPEED = 9.0          # m/s - disable at low speed
  # Curvature-adaptive hysteresis. Tight turns produce larger lookahead-point
  # offsets from geometric outside-swing (CG cuts inside the rear-axle path)
  # and from perception uncertainty (lane lines dim in turns — see
  # MIN_PROB_CURV_SLOPE). Straights stay tight to catch real drift; curves
  # relax to avoid fighting geometry. Linearly interpolated on |κ|:
  #   |κ| ≤ 0.001 (straight)   → threshold = 0.20 m
  #   |κ| ≥ 0.010 (tight)      → threshold = 0.30 m
  #   tolerance = threshold / 2     (2:1 hysteresis)
  OFFSET_THRESHOLD_KAPPA = [0.001, 0.010]
  OFFSET_THRESHOLD_M     = [0.20,  0.30]
  SMOOTH_TAU = 0.5         # seconds - correction smoothing
  WINDDOWN_TAU = 1.0       # seconds - slower wind-down when exiting turns
  MEASUREMENT_TAU = 0.2    # seconds - lane center measurement smoothing
  # Speed-dependent MAX_JUMP for lane_center change rejection. At high speed
  # perception noise at lookahead is bigger, AND any tracked change implies
  # higher lateral acceleration if applied — so reject smaller jumps. At low
  # speed allow bigger jumps since corrections are gentler. Linear between:
  #   v ≤ 30 kph (8.33 m/s):  0.40 m  (more permissive)
  #   v ≥ 120 kph (33.33 m/s): 0.15 m  (more conservative)
  # Matches current fixed 0.30 m near v ≈ 65 kph (typical operating speed).
  MAX_JUMP_LOW_V  = 0.40   # m at v_low
  MAX_JUMP_HIGH_V = 0.15   # m at v_high
  V_FOR_JUMP_LOW  = 30.0 / 3.6   # 8.33 m/s
  V_FOR_JUMP_HIGH = 120.0 / 3.6  # 33.33 m/s

  # Lane width estimation — per China standard (GB 50647):
  #   highway / city expressway ≥60 km/h: 3.75 m
  #   city general / mixed:               3.25–3.5 m
  #   intersection:                       2.8–3.5 m
  #   toll / narrow:                      2.5 m
  # Accept measurements in [MIN=2.5, MAX=3.75]. Outside this range, fall back
  # to DEFAULT (3.5, a reasonable middle for mixed city driving).
  LANE_WIDTH_DEFAULT = 3.5   # m - fallback when measurement out of range
  LANE_WIDTH_MIN = 2.5       # m - toll / narrow lane lower bound
  LANE_WIDTH_MAX = 3.75      # m - highway upper bound (China GB standard)
  LANE_WIDTH_SMOOTH_TAU = 2.0  # seconds - width estimation smoothing

  # latcontrol_torque kP curve — used to normalize our correction
  # so effective gain is flat across speeds
  KP_SPEEDS = [1, 1.5, 2.0, 3.0, 5, 7.5, 10, 15, 30]
  KP_VALUES = [250, 120, 65, 30, 11.5, 5.5, 3.5, 2.0, 0.8]
  KP_NOMINAL = 2.0  # normalize to this kP (roughly 15 m/s highway cruising)

  # Rate limiting — max correction change per model frame (prevents sudden
  # jumps). update() is called at modelV2's 20 Hz rate (gated by frameId in
  # on_curvature_correction), so 0.0005 / 0.05s ≈ 0.01 1/m per second.
  MAX_CORRECTION_RATE = 0.0005  # 1/m per model frame (20 Hz)

  # Derivative damping — reduce correction when offset is improving
  KD = 0.5  # damping factor on offset rate of change

  # Sample the offset at modelV2's planning horizon (lat_action_t) — this is
  # where desired_curvature is evaluated, so reading offset at the same
  # lookahead makes our correction intrinsically consistent with what it
  # modifies. Fixed 0.5s matches modelV2.action_t; removes dependency on
  # the separately-learned liveDelay service.
  MODEL_LAT_ACTION_T = 0.5

  # Reference-frame shift: model uses CAR_ROTATION_RADIUS=0 (rear axle), but
  # in turns the front swings outside and the rear cuts inside while the CG
  # follows the path the driver perceives. Sample lane offset at delay_dist
  # + CG_OFFSET so the measurement approximates CG perspective. On straights
  # this is a no-op (rear and CG sit on the same line); in turns the shift
  # captures the geometric outside-swing of the body. CG_OFFSET ≈ rear-axle
  # to CG distance, ~1.4 m for a typical mid-size sedan.
  CG_OFFSET = 1.4

  def __init__(self):
    self.prev_correction = 0.0
    self.active = False
    self.prev_lane_center = None
    self.smoothed_lane_center = None
    self.estimated_lane_width = None
    self.prev_offset = 0.0
    # Diagnostics (published to plugin bus by hook)
    self.diag = {}

  def _reset_lane_tracking(self):
    self.active = False
    self.prev_lane_center = None
    self.smoothed_lane_center = None

  def _smooth_correction(self, target, winding_down=False):
    tau = self.WINDDOWN_TAU if winding_down else self.SMOOTH_TAU
    self.prev_correction = smooth_value(target, self.prev_correction, tau, DT_MDL)
    return self.prev_correction

  def update(self, model_v2, v_ego):
    self.diag = {}

    # Skip if lane detection data not available
    if len(model_v2.laneLineProbs) < 3:
      self._reset_lane_tracking()
      return self._smooth_correction(0.0, winding_down=True)

    right_prob = model_v2.laneLineProbs[2]
    left_prob = model_v2.laneLineProbs[1]
    curvature = model_v2.action.desiredCurvature

    # Curvature-adaptive minimum confidence: relax linearly with |κ|. Keep
    # MIN_PROB_STRAIGHT as the "strong" gate for selection / width learning,
    # so width estimates aren't polluted by low-confidence detections.
    min_prob = max(self.MIN_PROB_FLOOR,
                   self.MIN_PROB_STRAIGHT - self.MIN_PROB_CURV_SLOPE * abs(curvature))

    # Need at least one lane with adaptive confidence
    if right_prob < min_prob and left_prob < min_prob:
      self._reset_lane_tracking()
      return self._smooth_correction(0.0, winding_down=True)

    if v_ego < self.MIN_SPEED:
      self._reset_lane_tracking()
      return self._smooth_correction(0.0, winding_down=True)

    # Check array lengths (idx=0 should always exist)
    if (len(model_v2.position.y) == 0 or
        len(model_v2.laneLines[1].y) == 0 or
        len(model_v2.laneLines[2].y) == 0):
      self._reset_lane_tracking()
      return self._smooth_correction(0.0, winding_down=True)

    # Read lane positions at modelV2.action_t lookahead, shifted by CG_OFFSET
    # so the offset is measured at CG perspective rather than rear axle.
    delay_dist = v_ego * self.MODEL_LAT_ACTION_T + self.CG_OFFSET
    X_IDXS = [192.0 * (i / 32) ** 2 for i in range(33)]
    idx = min(range(len(X_IDXS)), key=lambda i: abs(X_IDXS[i] - delay_dist))
    idx = min(idx, len(model_v2.position.y) - 1, len(model_v2.laneLines[1].y) - 1, len(model_v2.laneLines[2].y) - 1)

    path_y = model_v2.position.y[idx]
    right_y = model_v2.laneLines[2].y[idx]
    left_y = model_v2.laneLines[1].y[idx]

    # Dynamic lane width estimation when both lanes are confident.
    # When measured width is out of valid range (e.g. > 4.5m due to turn
    # projection distortion or merge ramp pulling the lane lines apart),
    # fall back to standard width AND flag the measurement as anomalous so
    # the lane-center selection below knows the raw lane line positions
    # are unreliable.
    width_anomalous = False
    if right_prob >= self.MIN_PROB_STRAIGHT and left_prob >= self.MIN_PROB_STRAIGHT:
      measured_width = right_y - left_y
      if self.LANE_WIDTH_MIN <= measured_width <= self.LANE_WIDTH_MAX:
        if self.estimated_lane_width is None:
          self.estimated_lane_width = measured_width
        else:
          self.estimated_lane_width = smooth_value(measured_width, self.estimated_lane_width, self.LANE_WIDTH_SMOOTH_TAU)
        lane_width = self.estimated_lane_width
      else:
        lane_width = self.LANE_WIDTH_DEFAULT
        width_anomalous = True
    else:
      lane_width = self.estimated_lane_width if self.estimated_lane_width is not None else self.LANE_WIDTH_DEFAULT
    half_width = lane_width / 2

    # Publish lane width unconditionally — speedlimitd fuses it as road-type
    # context (3.75 → highway, 3.25 → city general, 2.5 → toll/narrow) so it
    # must be available whenever we have a measurement, not only when the
    # correction itself is active.
    self.diag['lane_width'] = round(lane_width, 2)
    self.diag['lane_width_learned'] = self.estimated_lane_width is not None

    # Lane-center reference selection.
    # Width-anomaly override: if measured lane width was outside valid range
    # (likely merge ramp / split / model confusion), the raw lane-line y
    # positions can't be trusted to define the ego lane.
    #   With history: freeze lane_center to last good smoothed value.
    #   Without history: trust the model's path prediction — assume the car
    #     is on its planned path → lane_center = path_y → offset = 0. Seeds
    #     smoothed_lane_center for the next frame.
    if width_anomalous:
      lane_center = self.smoothed_lane_center if self.smoothed_lane_center is not None else path_y
    else:
      # Confidence-based selection — no closest-lane flip-flop, no explicit
      # turn/straight branching:
      #   Both strong (≥ MIN_PROB_STRAIGHT): midpoint = (left + right)/2.
      #     Stable (no flip when car drifts across the midline) and exact —
      #     doesn't depend on learned half_width.
      #   One strong, other weak: use the strong one + estimated half_width.
      #     In tight turns where outside drops below MIN_PROB_STRAIGHT, this
      #     naturally yields inside-lane reference (the inside is the strong one).
      #   Neither strong, at least one OK (curvature-relaxed gate): use higher.
      right_strong = right_prob >= self.MIN_PROB_STRAIGHT
      left_strong  = left_prob  >= self.MIN_PROB_STRAIGHT
      right_ok = right_prob >= min_prob
      left_ok  = left_prob  >= min_prob
      if not right_ok and not left_ok:
        self.active = False
        return self._smooth_correction(0.0, winding_down=True)

      if right_strong and left_strong:
        lane_center = (left_y + right_y) / 2.0
      elif left_strong:
        lane_center = left_y + half_width
      elif right_strong:
        lane_center = right_y - half_width
      else:
        # Both weak (only in turns with relaxed gate). Pick higher-confidence.
        if right_prob >= left_prob:
          lane_center = right_y - half_width
        else:
          lane_center = left_y + half_width

    # Reject sudden lane center jumps — threshold tighter at high speed where
    # noise is bigger and tracking would imply higher a_y.
    t_jump = max(0.0, min(1.0, (v_ego - self.V_FOR_JUMP_LOW) / (self.V_FOR_JUMP_HIGH - self.V_FOR_JUMP_LOW)))
    max_jump = self.MAX_JUMP_LOW_V + t_jump * (self.MAX_JUMP_HIGH_V - self.MAX_JUMP_LOW_V)
    if self.prev_lane_center is not None:
      if abs(lane_center - self.prev_lane_center) > max_jump:
        self.active = False
        self.prev_lane_center = lane_center
        self.smoothed_lane_center = None
        return self._smooth_correction(0.0, winding_down=True)
    self.prev_lane_center = lane_center

    # Smooth lane center measurement
    if self.smoothed_lane_center is None:
      self.smoothed_lane_center = lane_center
    else:
      self.smoothed_lane_center = smooth_value(lane_center, self.smoothed_lane_center, self.MEASUREMENT_TAU)

    offset = path_y - self.smoothed_lane_center

    # Curvature-adaptive hysteresis — relax in turns (geometric outside-swing
    # + perception noise on dim lane lines) and tighten on straights.
    offset_threshold = float(np.interp(abs(curvature), self.OFFSET_THRESHOLD_KAPPA, self.OFFSET_THRESHOLD_M))
    offset_tolerance = offset_threshold / 2.0

    # Hysteresis on offset only — runs regardless of curvature.
    if not self.active:
      if abs(offset) >= offset_threshold:
        self.active = True
    else:
      if abs(offset) < offset_tolerance:
        self.active = False

    if self.active:
      k = float(np.interp(abs(curvature), self.K_BP, self.K_V))

      # Derivative damping: offset rate of change (m/frame)
      d_offset = offset - self.prev_offset
      # If offset is shrinking (d_offset has opposite sign to offset), damp the correction
      damping = 1.0 - self.KD * np.clip(-np.sign(offset) * d_offset / max(abs(offset), 0.05), 0.0, 1.0)

      # kP compensation: normalize out the controller's speed-dependent gain
      kp_at_speed = float(np.interp(v_ego, self.KP_SPEEDS, self.KP_VALUES))
      kp_scale = self.KP_NOMINAL / max(kp_at_speed, 0.1)

      raw_correction = -k * offset * damping * kp_scale / (v_ego ** 2)

      # Rate limiting: clamp correction change per frame
      prev = self.prev_correction
      clamped = float(np.clip(raw_correction, prev - self.MAX_CORRECTION_RATE, prev + self.MAX_CORRECTION_RATE))

      self.prev_offset = offset
      self.diag = {
        'offset': round(offset, 3),
        'd_offset': round(d_offset, 4),
        'damping': round(damping, 3),
        'kp_scale': round(kp_scale, 3),
        'k': round(k, 3),
        'raw': round(raw_correction, 6),
        'clamped': round(clamped, 6),
        'v_ego': round(v_ego, 1),
        'curvature': round(curvature, 5),
      }

      return self._smooth_correction(clamped, winding_down=False)
    else:
      self.prev_offset = offset
      return self._smooth_correction(0.0, winding_down=True)


_lcc = None
_enabled = None   # None = unread; read once from LaneCenteringEnabled param
_prev_active = False
_lcc_pub = None   # publishes lane_centering_state (active + diagnostics)
_last_frame_id = None    # last model_v2.frameId we computed against
_last_correction = 0.0   # cached correction; reused on duplicate frames


def on_curvature_correction(curvature, model_v2, v_ego, lane_changing, **kwargs):
  global _lcc, _enabled, _prev_active, _lcc_pub, _last_frame_id, _last_correction

  # Feature toggle (Driving panel) — independent of plugin lifecycle
  if _enabled is None:
    val = read_plugin_param('lane_centering', 'LaneCenteringEnabled')
    _enabled = val != '0'  # enabled by default; only explicit '0' disables

  if not _enabled:
    return curvature
  if _lcc is None:
    _lcc = LaneCenteringCorrection()
  if lane_changing:
    _last_frame_id = None  # force recompute on next frame
    return curvature

  # Recompute only on a new model frame (20 Hz). controlsd calls this hook at
  # 100 Hz, but model_v2 only refreshes when modeld publishes. Smoothing /
  # rate-limit time constants (SMOOTH_TAU, MAX_CORRECTION_RATE, etc.) are
  # written assuming 20 Hz cadence — running them at 100 Hz makes them
  # converge 5× too fast.
  frame_id = model_v2.frameId
  if frame_id != _last_frame_id:
    _last_correction = _lcc.update(model_v2, v_ego)
    _last_frame_id = frame_id

    # Publish active state and diagnostics to plugin bus (also at 20 Hz)
    try:
      from openpilot.selfdrive.plugins.plugin_bus import PluginPub
      if _lcc_pub is None:
        _lcc_pub = PluginPub('lane_centering_state')
      msg = {'active': _lcc.active}
      if _lcc.diag:
        msg.update(_lcc.diag)
      _lcc_pub.send(msg)
    except Exception as e:
      cloudlog.warning(f"lane_centering: publish error: {e}")

  return curvature + _last_correction


def on_health_check(acc, **kwargs):
  return {**acc, "lane-centering": {"status": "ok"}}


