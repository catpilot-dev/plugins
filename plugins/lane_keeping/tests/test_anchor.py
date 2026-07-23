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
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
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
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
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
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
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
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
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
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
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
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
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
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
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
  a = LaneAnchor(AnchorConfig(driver_side='right', pred_delay_mult=2.0))
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
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
  mv = _mv_geo(_XS, _flat(-1.75), _flat(1.75))
  _o, t = a.update(0.0, mv, 25.0, False, lat_delay=0.4)
  assert abs(t['x_pred'] - 20.0) < 1e-9            # 25 * 2*0.4
  _o, t = a.update(0.0, mv, 25.0, False)           # no lat_delay -> fallback 0.6
  assert abs(t['x_pred'] - 30.0) < 1e-9


def test_lane_change_reseeds_gap_filters():
  # The driver-side line's IDENTITY changes during a lane change (old left
  # line hands off to the new lane's left line), so smoothed gap history from
  # the old lane is meaningless. During the change the filters must track RAW
  # (re-seed every tick, mirroring kappa_filt), so the first post-change tick
  # reads the new lane cleanly - no stale re-convergence, no settle-nudge.
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
  mv_old = _mv_geo(_XS, _flat(-1.31), _flat(2.19))     # old lane: gap 0.40 (below band)
  for _ in range(2000):
    a.update(0.0, mv_old, 25.0, False, lat_delay=0.6)  # settle, bias building
  mv_new = _mv_geo(_XS, _flat(-1.75), _flat(1.75))     # new lane: gap 0.84 (in-band)
  out, t = a.update(0.0, mv_new, 25.0, True, lat_delay=0.6)   # lane-change tick
  assert abs(t['gap_filt'] - 0.84) < 1e-9              # re-seeded to raw, no lag
  assert abs(t['gap_pred'] - 0.84) < 1e-9
  assert out == 0.0                                    # fully inert during the change
  out, t = a.update(0.0, mv_new, 25.0, False, lat_delay=0.6)  # first tick after
  assert abs(t['gap_filt'] - 0.84) < 1e-6              # no stale state carried over
  assert abs(out) < 1e-6                               # in-band -> no settle-nudge


# --- integral trim (2026-07-23: route 3c0 showed P-only droop) ---

def test_trim_accumulates_below_band_correct_direction():
  # Left driver stuck below band: trim must build NEGATIVE (steer right) at
  # trim_rate, independent of the (tiny) proportional nudge.
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
  mv = _mv_geo(_XS, _flat(-1.41), _flat(2.09))     # gap 0.50, below band
  for _ in range(500):                             # 5 s (mid-ramp, below the cap)
    a.update(0.0, mv, 17.0, False, lat_delay=0.6)
  assert a.kappa_trim < -0.5e-4                    # accumulated rightward
  expected = -1e-4 * 5.0                           # trim_rate * t (before cap)
  assert abs(a.kappa_trim - expected) < 2e-5       # ~linear ramp


def test_trim_caps():
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
  mv = _mv_geo(_XS, _flat(-1.41), _flat(2.09))
  for _ in range(3000):                            # 30 s >> cap time
    a.update(0.0, mv, 17.0, False, lat_delay=0.6)
  assert abs(a.kappa_trim + 1e-3) < 1e-9           # clamped at -trim_max


def test_trim_leaks_in_band():
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
  a.kappa_trim = -8e-4
  mv = _mv_geo(_XS, _flat(-1.75), _flat(1.75))     # gap 0.84, in-band
  for _ in range(1000):                            # 10 s of leak
    a.update(0.0, mv, 17.0, False, lat_delay=0.6)
  assert -8e-4 < a.kappa_trim < -5e-4              # decaying, slowly (leak << rate)
  for _ in range(50000):                           # long in-band -> exactly zero
    a.update(0.0, mv, 17.0, False, lat_delay=0.6)
  assert a.kappa_trim == 0.0


def test_trim_unwinds_on_opposite_error_no_ratchet():
  # THE hold-bias regression test: an opposite-side error must unwind the
  # trim at full rate — anti-windup may block only windup, never unwind.
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
  below = _mv_geo(_XS, _flat(-1.41), _flat(2.09))  # gap 0.50 -> trim goes negative
  for _ in range(1000):
    a.update(0.0, below, 17.0, False, lat_delay=0.6)
  t_neg = a.kappa_trim
  assert t_neg < -0.5e-4
  above = _mv_geo(_XS, _flat(-2.11), _flat(1.39))  # gap 1.20 -> above band
  for _ in range(1500):                            # unwinds through zero
    a.update(0.0, above, 17.0, False, lat_delay=0.6)
  assert a.kappa_trim > 0.0                        # crossed zero: no ratchet


def test_trim_zeroed_on_lane_change():
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
  a.kappa_trim = -8e-4
  mv = _mv_geo(_XS, _flat(-1.75), _flat(1.75))
  a.update(0.0, mv, 17.0, True, lat_delay=0.6)
  assert a.kappa_trim == 0.0


def test_trim_added_to_output():
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
  a.kappa_trim = -6e-4
  mv = _mv_geo(_XS, _flat(-1.75), _flat(1.75))     # in-band: bias ~0
  out, t = a.update(0.01, mv, 17.0, False, lat_delay=0.6)
  assert abs(out - (0.01 - 6e-4)) < 2e-5           # kappa_ref + trim (leak negligible)
  assert abs(t['kappa_trim'] - a.kappa_trim) < 1e-12


def test_trim_right_driver_mirror():
  # Right driver too close to the right line: trim must build POSITIVE (left).
  a = LaneAnchor(AnchorConfig(driver_side='right', pred_delay_mult=2.0))
  mv = _mv_geo(_XS, _flat(-2.09), _flat(1.41))     # right gap 0.50, below band
  for _ in range(1000):
    a.update(0.0, mv, 17.0, False, lat_delay=0.6)
  assert a.kappa_trim > 0.5e-4


def test_trim_leaks_when_authority_zero():
  # Review finding: a visible-but-untrusted line (authority 0) must LEAK the
  # trim like line-loss does — never freeze DC state without a trusted
  # measurement, and never snap a stale trim back when confidence returns.
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
  a.kappa_trim = -8e-4
  mv = _mv_geo(_XS, _flat(-1.41), _flat(2.09), left_p=0.4)   # below band, prob<prob_on
  for _ in range(1000):                            # 10 s
    a.update(0.0, mv, 17.0, False, lat_delay=0.6)
  assert -7e-4 < a.kappa_trim < -5.5e-4            # leaked ~2e-4, not frozen at -8e-4


def test_trim_leaks_when_line_lost():
  # Review coverage gap: the unavailable-branch leak with a NONZERO trim.
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
  a.kappa_trim = -8e-4
  none_mv = SimpleNamespace(laneLines=[], laneLineProbs=[])
  for _ in range(1000):                            # 10 s
    a.update(0.0, none_mv, 17.0, False, lat_delay=0.6)
  assert -7e-4 < a.kappa_trim < -5.5e-4            # leaking, not frozen/integrating


def test_trim_speed_accel_clamp():
  # Review finding: flat kappa cap means v^2*trim grows quadratically. The
  # dynamic clamp bounds |v^2 * kappa_trim| <= trim_accel_max (0.3 m/s^2).
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
  a.kappa_trim = -1e-3                             # at the flat cap
  mv = _mv_geo(_XS, _flat(-1.75), _flat(1.75))     # in-band
  a.update(0.0, mv, 30.0, False, lat_delay=0.6)    # highway speed
  cap = 0.3 / 900.0                                # trim_accel_max / v^2
  assert abs(a.kappa_trim) <= cap + 1e-9           # clamped immediately
  # and at the 3c0 operating point (~17.8 m/s) the full flat cap remains usable
  a2 = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
  a2.kappa_trim = -1e-3
  a2.update(0.0, mv, 17.0, False, lat_delay=0.6)
  assert abs(a2.kappa_trim) > 0.9e-3               # 0.3/289=1.04e-3 > flat cap


def test_disabled_retires_trim_at_trim_rate():
  # A deliberate disable (cfg.enable False) retires the trim at trim_rate
  # (~10s from full), not the slow line-dropout leak (~50s).
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
  a.kappa_trim = -8e-4
  a.cfg.enable = False
  mv = _mv_geo(_XS, _flat(-1.41), _flat(2.09))     # geometry irrelevant: disabled
  for _ in range(500):                             # 5 s
    a.update(0.0, mv, 17.0, False, lat_delay=0.6)
  assert -3.5e-4 < a.kappa_trim < -2.5e-4          # ~5e-4 retired (rate 1e-4/s)


def test_disabled_keeps_smoothing():
  # Disabling the anchor must NOT disable the reference conditioning —
  # a step in kappa_des still comes out smoothed (Phase 2 safety invariant).
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
  a.cfg.enable = False
  mv = _mv_geo(_XS, _flat(-1.31), _flat(2.19))     # would nudge if enabled
  for _ in range(200):
    a.update(0.02, mv, 17.0, False, lat_delay=0.6) # settle filter at 0.02
  out, t = a.update(0.0, mv, 17.0, False, lat_delay=0.6)   # step down
  assert 0.0 < out < 0.02                          # smoothed, lagging
  assert t['kappa_bias'] == 0.0 and t['state'] == 'model'  # no position correction
