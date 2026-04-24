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

  MIN_PROB = 0.5           # Minimum lane detection confidence
  MIN_SPEED = 9.0          # m/s - disable at low speed
  OFFSET_THRESHOLD = 0.15  # m - activate when offset exceeds this (still OK on straight lane)
  OFFSET_TOLERANCE = 0.05  # m - deactivate when offset within tolerance
  SMOOTH_TAU = 0.5         # seconds - correction smoothing
  WINDDOWN_TAU = 1.0       # seconds - slower wind-down when exiting turns
  MEASUREMENT_TAU = 0.2    # seconds - lane center measurement smoothing
  MAX_JUMP = 0.3           # m - max lane center change per frame

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

  # Rate limiting — max correction change per frame (prevents sudden jumps)
  MAX_CORRECTION_RATE = 0.0005  # 1/m per frame at 20 Hz

  # Derivative damping — reduce correction when offset is improving
  KD = 0.5  # damping factor on offset rate of change

  # Sample the offset at modelV2's planning horizon (lat_action_t) — this is
  # where desired_curvature is evaluated, so reading offset at the same
  # lookahead makes our correction intrinsically consistent with what it
  # modifies. Fixed 0.5s matches modelV2.action_t; removes dependency on
  # the separately-learned liveDelay service.
  MODEL_LAT_ACTION_T = 0.5

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

    # Need at least one lane with good confidence
    if right_prob < self.MIN_PROB and left_prob < self.MIN_PROB:
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

    curvature = model_v2.action.desiredCurvature

    # Read lane positions at modelV2.action_t lookahead (not t=0) so the
    # offset is sampled at the same point in space where desired_curvature
    # is evaluated. Keeps our correction intrinsically time-aligned.
    delay_dist = v_ego * self.MODEL_LAT_ACTION_T
    X_IDXS = [192.0 * (i / 32) ** 2 for i in range(33)]
    idx = min(range(len(X_IDXS)), key=lambda i: abs(X_IDXS[i] - delay_dist))
    idx = min(idx, len(model_v2.position.y) - 1, len(model_v2.laneLines[1].y) - 1, len(model_v2.laneLines[2].y) - 1)

    path_y = model_v2.position.y[idx]
    right_y = model_v2.laneLines[2].y[idx]
    left_y = model_v2.laneLines[1].y[idx]

    # Dynamic lane width estimation when both lanes are confident.
    # When measured width is out of valid range (e.g. > 4.5m due to turn
    # projection distortion), fall back to standard width immediately rather
    # than using a stale smoothed estimate.
    if right_prob >= self.MIN_PROB and left_prob >= self.MIN_PROB:
      measured_width = right_y - left_y
      if self.LANE_WIDTH_MIN <= measured_width <= self.LANE_WIDTH_MAX:
        if self.estimated_lane_width is None:
          self.estimated_lane_width = measured_width
        else:
          self.estimated_lane_width = smooth_value(measured_width, self.estimated_lane_width, self.LANE_WIDTH_SMOOTH_TAU)
        lane_width = self.estimated_lane_width
      else:
        lane_width = self.LANE_WIDTH_DEFAULT
    else:
      lane_width = self.estimated_lane_width if self.estimated_lane_width is not None else self.LANE_WIDTH_DEFAULT
    half_width = lane_width / 2

    # Publish lane width unconditionally — speedlimitd fuses it as road-type
    # context (3.75 → highway, 3.25 → city general, 2.5 → toll/narrow) so it
    # must be available whenever we have a measurement, not only when the
    # correction itself is active.
    self.diag['lane_width'] = round(lane_width, 2)
    self.diag['lane_width_learned'] = self.estimated_lane_width is not None

    # Use closest lane line as reference for lane center. The nearer line is
    # more reliable for detection and matches how a driver references "the
    # line I'm closest to." If only one is confident enough, use it.
    right_ok = right_prob >= self.MIN_PROB
    left_ok  = left_prob  >= self.MIN_PROB
    if right_ok and left_ok:
      if abs(right_y) <= abs(left_y):
        lane_center = right_y - half_width
      else:
        lane_center = left_y + half_width
    elif right_ok:
      lane_center = right_y - half_width
    elif left_ok:
      lane_center = left_y + half_width
    else:
      self.active = False
      return self._smooth_correction(0.0, winding_down=True)

    # Reject sudden lane center jumps
    if self.prev_lane_center is not None:
      if abs(lane_center - self.prev_lane_center) > self.MAX_JUMP:
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

    # Hysteresis on offset only — runs regardless of curvature.
    if not self.active:
      if abs(offset) >= self.OFFSET_THRESHOLD:
        self.active = True
    else:
      if abs(offset) < self.OFFSET_TOLERANCE:
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


def on_curvature_correction(curvature, model_v2, v_ego, lane_changing, **kwargs):
  global _lcc, _enabled, _prev_active, _lcc_pub

  # Feature toggle (Driving panel) — independent of plugin lifecycle
  if _enabled is None:
    val = read_plugin_param('lane_centering', 'LaneCenteringEnabled')
    _enabled = val != '0'  # enabled by default; only explicit '0' disables

  if not _enabled:
    return curvature
  if _lcc is None:
    _lcc = LaneCenteringCorrection()
  if lane_changing:
    return curvature
  correction = _lcc.update(model_v2, v_ego)

  # Publish active state and diagnostics to plugin bus
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

  return curvature + correction


def on_health_check(acc, **kwargs):
  return {**acc, "lane-centering": {"status": "ok"}}


