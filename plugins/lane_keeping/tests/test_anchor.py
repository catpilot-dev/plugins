import importlib.util, os
from types import SimpleNamespace
# Load the pure control core by explicit path under a unique module name — do
# NOT insert the plugin dir on sys.path, which would shadow other plugins'
# same-named modules (e.g. bmw's `register`) when the whole suite runs together.
_PLUGIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_spec = importlib.util.spec_from_file_location('lk_anchor', os.path.join(_PLUGIN_DIR, 'anchor.py'))
_anchor = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_anchor)
AnchorConfig, LaneAnchor = _anchor.AnchorConfig, _anchor.LaneAnchor


# modelV2 device frame: +y = RIGHT, so the LEFT ego line (laneLines[1]) sits at
# NEGATIVE y and the RIGHT ego line (laneLines[2]) at POSITIVE y.
def _mv(left_y, right_y, left_p=1.0, right_p=1.0):
  return SimpleNamespace(
    laneLines=[SimpleNamespace(y=[0.0]), SimpleNamespace(y=[left_y]),
               SimpleNamespace(y=[right_y]), SimpleNamespace(y=[0.0])],
    laneLineProbs=[0.0, left_p, right_p, 0.0])


def test_gap_left_driver():
  a = LaneAnchor(AnchorConfig(driver_side='left', half_width=0.91))
  assert a.driver_idx == 1 and a.line_sign == -1.0 and a.curv_sign == 1.0
  # left line 1.75 m to the left, at y = -1.75 -> wheel gap = 1.75 - 0.91 = 0.84
  assert abs(a._gap(_mv(left_y=-1.75, right_y=1.75)) - 0.84) < 1e-9


def test_gap_right_driver():
  a = LaneAnchor(AnchorConfig(driver_side='right', half_width=0.91))
  assert a.driver_idx == 2 and a.line_sign == 1.0 and a.curv_sign == -1.0
  # right line 1.75 m to the right, at y = +1.75 -> wheel gap = 1.75 - 0.91 = 0.84
  assert abs(a._gap(_mv(left_y=-1.75, right_y=1.75)) - 0.84) < 1e-9


def test_excess_zero_in_band():
  a = LaneAnchor(AnchorConfig(gap_min=0.6, gap_max=1.0))
  assert a._excess(0.6) == 0.0
  assert a._excess(0.8) == 0.0
  assert a._excess(1.0) == 0.0


def test_excess_signs_and_clip():
  a = LaneAnchor(AnchorConfig(gap_min=0.6, gap_max=1.0, excess_max=0.5))
  # gap below band (too close to driver line) -> negative excess
  assert abs(a._excess(0.4) - (-0.2)) < 1e-9
  # gap above band (too far from driver line) -> positive excess
  assert abs(a._excess(1.3) - 0.3) < 1e-9
  # glitch far above band -> clipped to +excess_max
  assert a._excess(9.0) == 0.5
  # glitch far below band -> clipped to -excess_max
  assert a._excess(-9.0) == -0.5


def test_pursuit_magnitude_and_sign_left():
  a = LaneAnchor(AnchorConfig(driver_side='left', t_preview=1.5, kappa_bias_max=1.0))
  # excess +0.3 (car too far from left line) at 25 m/s:
  # Lp = 25*1.5 = 37.5; kappa = 2*0.3/37.5^2 = 0.000426..., positive (steer left, left-positive curvature)
  k = a._pursuit(0.3, 25.0)
  assert abs(k - (2 * 0.3 / 37.5 ** 2)) < 1e-9
  assert k > 0
  # excess -0.3 (car too close to left line) -> steer right (negative)
  assert a._pursuit(-0.3, 25.0) < 0


def test_pursuit_sign_right_driver():
  a = LaneAnchor(AnchorConfig(driver_side='right', t_preview=1.5, kappa_bias_max=1.0))
  # right driver, excess +0.3 (car too far from right line = too far left) -> steer right (negative)
  assert a._pursuit(0.3, 25.0) < 0


def test_pursuit_hard_cap():
  a = LaneAnchor(AnchorConfig(driver_side='left', t_preview=1.5, kappa_bias_max=0.002))
  # low speed inflates kappa; cap binds
  assert abs(a._pursuit(0.5, 5.0)) == 0.002


def _settle(a, mv, v=25.0, lane_changing=False, n=2000):
  out = None
  for _ in range(n):
    out, _t = a.update(0.01, mv, v, lane_changing)
  return out


def test_update_passthrough_when_no_line():
  a = LaneAnchor(AnchorConfig())
  mv = SimpleNamespace(laneLines=[], laneLineProbs=[])
  out, telem = a.update(0.0123, mv, 25.0, False)
  assert out == 0.0123              # bit-identical passthrough
  assert telem['state'] == 'model'


def test_update_passthrough_when_low_prob():
  a = LaneAnchor(AnchorConfig(prob_on=0.6))
  mv = _mv(left_y=-1.75, right_y=1.75, left_p=0.4)   # below prob_on
  out, telem = a.update(0.0123, mv, 25.0, False)
  assert out == 0.0123
  assert telem['state'] == 'model'


def test_update_no_bias_in_band():
  a = LaneAnchor(AnchorConfig())
  # left line at y=-1.75 -> gap 0.84, inside [0.6,1.0]
  out = _settle(a, _mv(left_y=-1.75, right_y=1.75))
  assert abs(out - 0.01) < 1e-6     # curvature unchanged (bias ~0)


def test_update_biases_left_when_too_far_from_left_line():
  a = LaneAnchor(AnchorConfig())
  # left line at y=-2.3 -> gap 1.39, above band -> steer left (positive bias)
  out = _settle(a, _mv(left_y=-2.3, right_y=1.2))
  assert out > 0.01 + 1e-5


def test_update_biases_right_when_too_close_to_left_line():
  a = LaneAnchor(AnchorConfig())
  # left line at y=-1.3 -> gap 0.39, below band -> steer right (negative bias)
  out = _settle(a, _mv(left_y=-1.3, right_y=2.2))
  assert out < 0.01 - 1e-5


def test_update_disabled_during_lane_change():
  a = LaneAnchor(AnchorConfig())
  out = _settle(a, _mv(left_y=-2.3, right_y=1.2), lane_changing=True)
  assert abs(out - 0.01) < 1e-6     # authority 0 -> passthrough


def test_update_lane_change_hard_zeros_established_bias():
  # Build up a real bias out-of-band, then a lane change must drop it to exactly
  # 0 on the FIRST tick (not decay it) so the anchor never fights the maneuver.
  a = LaneAnchor(AnchorConfig())
  mv = _mv(left_y=-2.3, right_y=1.2)          # gap 1.39 above band -> steer left
  _settle(a, mv)                               # establish the bias
  assert a.kappa_bias > 1e-5                   # bias is genuinely established
  out, telem = a.update(0.01, mv, 25.0, True)  # now a lane change
  assert a.kappa_bias == 0.0                   # hard-zeroed immediately, not decaying
  assert out == 0.01                           # bit-identical passthrough
  assert telem['state'] == 'model'


def test_update_rate_limited():
  a = LaneAnchor(AnchorConfig(kappa_rate_max=0.002))
  mv = _mv(left_y=-2.3, right_y=1.2)
  # first engaged tick can move at most kappa_rate_max*DT_CTRL from 0
  a.gap_filt = a._gap(mv)           # warm the filter so excess is immediate
  out, telem = a.update(0.01, mv, 25.0, False)
  assert abs(telem['kappa_bias']) <= 0.002 * 0.01 + 1e-12


def test_smoothing_lags_a_step_then_converges():
  # No line -> no position bias, so the output is purely the smoothed reference.
  a = LaneAnchor(AnchorConfig(kappa_filter_tau=0.3))
  none_mv = SimpleNamespace(laneLines=[], laneLineProbs=[])
  out1, t1 = a.update(0.02, none_mv, 25.0, False)
  assert abs(out1 - 0.02) < 1e-9          # first sample seeds the filter
  assert abs(t1['kappa_in'] - 0.02) < 1e-9
  out2, _ = a.update(0.0, none_mv, 25.0, False)   # step down
  assert 0.0 < out2 < 0.02                # lags, does not jump
  for _ in range(500):                    # ~5s >> tau
    out, _t = a.update(0.0, none_mv, 25.0, False)
  assert abs(out) < 1e-4                  # converges


def test_smoothing_bypassed_during_lane_change():
  a = LaneAnchor(AnchorConfig(kappa_filter_tau=0.3))
  none_mv = SimpleNamespace(laneLines=[], laneLineProbs=[])
  for _ in range(200):
    a.update(0.02, none_mv, 25.0, False)  # settle filter at 0.02
  out, telem = a.update(0.0, none_mv, 25.0, True)   # lane change -> raw
  assert out == 0.0                       # bit-identical passthrough of raw
  assert abs(telem['kappa_ref']) < 1e-12


def test_smoothing_applies_in_anchor_state_too():
  # smoothing is unconditional; only the position correction is ANCHOR-gated
  a = LaneAnchor(AnchorConfig(kappa_filter_tau=0.3))
  mv = _mv(left_y=-1.75, right_y=1.75)    # gap 0.84 in band -> zero bias
  a.update(0.02, mv, 25.0, False)
  out, telem = a.update(0.0, mv, 25.0, False)
  assert telem['state'] == 'anchor'
  assert 0.0 < out < 0.02                 # smoothed, bias is zero in-band


# --- predictive deadband (2026-07-23 spec) ---
# Lane lines with real geometry: y arrays over an x grid (device frame +y=right,
# so the LEFT line's y values are negative).
def _mv_geo(xs, left_ys, right_ys, left_p=1.0, right_p=1.0, plan_ys=None):
  # position = the model's planned path (y over the same x grid); default is
  # a straight-ahead plan (zeros) — the prediction is line-minus-plan.
  return SimpleNamespace(
    laneLines=[SimpleNamespace(x=[], y=[0.0]),
               SimpleNamespace(x=list(xs), y=list(left_ys)),
               SimpleNamespace(x=list(xs), y=list(right_ys)),
               SimpleNamespace(x=[], y=[0.0])],
    laneLineProbs=[0.0, left_p, right_p, 0.0],
    position=SimpleNamespace(x=list(xs),
                             y=list(plan_ys if plan_ys is not None else [0.0] * len(xs))))


_XS = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0]


def _flat(y):
  return [y] * len(_XS)


def test_pred_parallel_line_equals_current_gap():
  # straight, parallel line, zero curvature -> gap_pred == gap; in-band -> hold
  a = LaneAnchor(AnchorConfig())
  mv = _mv_geo(_XS, _flat(-1.75), _flat(1.75))    # gap 0.84 everywhere
  out = None
  for _ in range(500):
    out, t = a.update(0.0, mv, 25.0, False, lat_delay=0.6)
  assert abs(t['gap_pred'] - 0.84) < 1e-6
  assert abs(t['x_pred'] - 30.0) < 1e-9           # 25 m/s * 2*0.6 s
  assert abs(out) < 1e-6                          # no bias, no reference


def test_pred_converging_line_nudges_early():
  # In-band NOW (gap 0.84) but the left line converges: at 30 m the gap is
  # only 0.34 -> predicted below band -> nudge AWAY from the left line
  # (steer right = negative curvature) while still in-band.
  a = LaneAnchor(AnchorConfig())
  left = [-1.75 + 0.5 * (x / 30.0) for x in _XS]  # -1.75 at car -> -1.25 at 30 m
  mv = _mv_geo(_XS, left, _flat(1.75))
  out = None
  for _ in range(2000):
    out, t = a.update(0.0, mv, 25.0, False, lat_delay=0.6)
  assert t['gap_pred'] < 0.6                      # predicted out-of-band
  assert out < -1e-5                              # nudging right (away)


def test_pred_recovering_line_no_fight():
  # Current gap below band (0.5) but diverging: at 30 m the gap is 0.9 ->
  # predicted in-band -> DO NOT nudge (0.5 is above the 0.3 hard floor).
  a = LaneAnchor(AnchorConfig())
  left = [-1.41 - 0.4 * (x / 30.0) for x in _XS]  # gap 0.5 at car -> 0.9 at 30 m
  mv = _mv_geo(_XS, left, _flat(1.75))
  out = None
  for _ in range(2000):
    out, t = a.update(0.0, mv, 25.0, False, lat_delay=0.6)
  assert t['gap_filt'] < 0.6                      # currently out-of-band
  assert t['gap_pred'] > 0.6                      # predicted back in-band
  assert abs(out) < 1e-6                          # holds: no fight with recovery


def test_pred_hard_floor_low_overrides_prediction():
  # Wheel 0.2 m from the line: prediction says recovering, but 0.2 < 0.3 hard
  # floor -> correct NOW on the current gap.
  a = LaneAnchor(AnchorConfig())
  left = [-1.11 - 0.7 * (x / 30.0) for x in _XS]  # gap 0.2 at car -> 0.9 at 30 m
  mv = _mv_geo(_XS, left, _flat(1.75))
  out = None
  for _ in range(2000):
    out, t = a.update(0.0, mv, 25.0, False, lat_delay=0.6)
  assert t['gap_filt'] < 0.3
  assert t['gap_pred'] > 0.6
  assert out < -1e-5                              # corrects away regardless


def test_pred_hard_ceiling_overrides_prediction():
  # Gap 1.6 (> 1.5 hard ceiling), prediction says coming back -> correct NOW
  # toward the driver line (steer left = positive).
  a = LaneAnchor(AnchorConfig())
  left = [-2.51 + 0.7 * (x / 30.0) for x in _XS]  # gap 1.6 at car -> 0.9 at 30 m
  mv = _mv_geo(_XS, left, _flat(0.95))
  out = None
  for _ in range(2000):
    out, t = a.update(0.0, mv, 25.0, False, lat_delay=0.6)
  assert t['gap_filt'] > 1.5
  assert t['gap_pred'] < 1.0
  assert out > 1e-5


def test_pred_curve_no_phantom_drift():
  # Left curve kappa=0.004: the line curves left (y = -1.75 - k*x^2/2) and the
  # MODEL PLAN curves with it (same geometry) -> line-minus-plan cancels the
  # curvature -> predicted gap stays 0.84, no phantom excess, no bias beyond
  # the (smoothed) reference itself.
  k = 0.004
  a = LaneAnchor(AnchorConfig())
  left = [-1.75 - k * x * x / 2.0 for x in _XS]
  right = [1.75 - k * x * x / 2.0 for x in _XS]
  plan = [-k * x * x / 2.0 for x in _XS]
  mv = _mv_geo(_XS, left, right, plan_ys=plan)
  out = None
  for _ in range(2000):
    out, t = a.update(k, mv, 25.0, False, lat_delay=0.6)
  assert abs(t['gap_pred'] - 0.84) < 0.02
  assert abs(out - k) < 1e-4                      # reference passes, no nudge


def test_pred_plan_drift_toward_line_nudges():
  # Straight, parallel lines, but the PLAN drifts toward the left line
  # (plan y -> -0.5 at 30 m): the tracker will follow that plan, so the
  # predicted gap shrinks (1.25 - 0.91 = 0.34 < 0.6) -> nudge right (negative)
  # even though the current gap (0.84) is comfortably in-band.
  a = LaneAnchor(AnchorConfig())
  plan = [-0.5 * (x / 30.0) for x in _XS]
  mv = _mv_geo(_XS, _flat(-1.75), _flat(1.75), plan_ys=plan)
  out = None
  for _ in range(2000):
    out, t = a.update(0.0, mv, 25.0, False, lat_delay=0.6)
  assert t['gap_pred'] < 0.6
  assert out < -1e-5


def test_pred_missing_plan_falls_back_to_current_gap():
  # Good line geometry but NO position attr -> prediction unavailable ->
  # deadband acts on the current gap (which is in-band here -> no bias).
  a = LaneAnchor(AnchorConfig())
  left = [-1.75 + 0.5 * (x / 30.0) for x in _XS]  # converging line...
  mv = SimpleNamespace(
    laneLines=[SimpleNamespace(x=[], y=[0.0]),
               SimpleNamespace(x=list(_XS), y=list(left)),
               SimpleNamespace(x=list(_XS), y=list(_flat(1.75))),
               SimpleNamespace(x=[], y=[0.0])],
    laneLineProbs=[0.0, 1.0, 1.0, 0.0])           # ...but no position: fallback
  out = None
  for _ in range(500):
    out, t = a.update(0.0, mv, 25.0, False, lat_delay=0.6)
  assert abs(t['gap_pred'] - t['gap_filt']) < 1e-9   # fell back to current gap
  assert abs(out) < 1e-6                             # current gap in-band -> hold


def test_pred_right_driver_converging_nudges_left():
  # Right-side driver: right line converging -> nudge AWAY from the right
  # line = steer left = POSITIVE curvature.
  a = LaneAnchor(AnchorConfig(driver_side='right'))
  right = [1.75 - 0.5 * (x / 30.0) for x in _XS]
  mv = _mv_geo(_XS, _flat(-1.75), right)
  out = None
  for _ in range(2000):
    out, t = a.update(0.0, mv, 25.0, False, lat_delay=0.6)
  assert t['gap_pred'] < 0.6
  assert out > 1e-5


def test_pred_fallback_short_arrays_behaves_like_current_gap():
  # Old-style single-point lane lines (no x attr / 1-point y): prediction
  # falls back to the current gap — the pre-existing out-of-band behavior.
  a = LaneAnchor(AnchorConfig())
  out = _settle(a, _mv(left_y=-2.3, right_y=1.2))  # gap 1.39, above band
  assert out > 0.01 + 1e-5                         # still biases (via fallback)


def test_pred_lat_delay_scales_x_pred():
  a = LaneAnchor(AnchorConfig())
  mv = _mv_geo(_XS, _flat(-1.75), _flat(1.75))
  _o, t = a.update(0.0, mv, 25.0, False, lat_delay=0.4)
  assert abs(t['x_pred'] - 20.0) < 1e-9            # 25 * 2*0.4
  _o, t = a.update(0.0, mv, 25.0, False)           # no lat_delay -> fallback 0.6
  assert abs(t['x_pred'] - 30.0) < 1e-9
