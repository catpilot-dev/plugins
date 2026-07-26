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


def test_update_disabled_during_lane_change():
  a = LaneAnchor(AnchorConfig())
  out = _settle(a, _mv(left_y=-2.3, right_y=1.2), lane_changing=True)
  assert abs(out - 0.01) < 1e-6     # authority 0 -> passthrough


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


def test_pred_fallback_short_arrays_behaves_like_current_gap():
  # Old-style single-point lane lines (no x attr / 1-point y): prediction
  # falls back to the current gap — same fallback wiring as before, but
  # under the Phase-3 pivot a CONSTANT gap (even out of the old band) is
  # conceded rather than corrected, so the outcome changes accordingly.
  a = LaneAnchor(AnchorConfig())
  mv = _mv(left_y=-2.3, right_y=1.2)  # gap 1.39, constant
  out = t = None
  for _ in range(2000):
    out, t = a.update(0.01, mv, 25.0, False)
  assert abs(t['gap_pred'] - t['gap_filt']) < 1e-9   # fell back to current gap
  assert abs(out - 0.01) < 1e-6                       # constant gap -> conceded


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
    a.update(0.0, mv_old, 25.0, False, lat_delay=0.6)  # settle filters
  mv_new = _mv_geo(_XS, _flat(-1.75), _flat(1.75))     # new lane: gap 0.84 (in-band)
  out, t = a.update(0.0, mv_new, 25.0, True, lat_delay=0.6)   # lane-change tick
  assert abs(t['gap_filt'] - 0.84) < 1e-9              # re-seeded to raw, no lag
  assert abs(t['gap_pred'] - 0.84) < 1e-9
  assert out == 0.0                                    # fully inert during the change
  out, t = a.update(0.0, mv_new, 25.0, False, lat_delay=0.6)  # first tick after
  assert abs(t['gap_filt'] - 0.84) < 1e-6              # no stale state carried over
  assert abs(out) < 1e-6                               # in-band -> no settle-nudge


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


# --- Phase 3: AC stabilizer (damp the wander, concede the line) ---

def _mv_at(gap):
  # left-driver scene with the LEFT line placed to give the requested gap
  y = -(gap + 0.91)
  return _mv_geo(_XS, _flat(y), _flat(1.75))


def _run(a, gap, n, v=17.0, lc=False):
  out = t = None
  for _ in range(n):
    out, t = a.update(0.0, _mv_at(gap), v, lc, lat_delay=0.6)
  return out, t


def test_constant_offset_is_conceded_anywhere():
  # THE anti-3c1 regression test: a static gap — even far out of the old
  # band — produces ZERO correction after seeding. The line is the model's.
  for g in (1.39, 0.45, 0.84):
    a2 = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
    out, t = _run(a2, g, 3000)
    assert abs(out) < 1e-6, f'gap {g} not conceded'
    assert abs(t['excess_ac']) < 0.02
    assert abs(t['gap_dc'] - g) < 0.05          # DC tracked the line


def test_drift_is_damped():
  # A drifting gap (the integrated sub-Hz wander) IS corrected while it
  # drifts: gap rising = car moving away from the left line (rightward)
  # -> damp with positive (leftward) curvature.
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
  _run(a, 0.84, 1000)                            # seed DC at 0.84
  out = None
  for i in range(600):                           # 6 s drift at ~0.05 m/s
    g = 0.84 + 0.05 * (i / 100.0)
    out, t = a.update(0.0, _mv_at(g), 17.0, False, lat_delay=0.6)
  assert out > 1e-5                              # damping the motion, leftward
  assert t['excess_ac'] > 0.1                    # beyond the AC deadband


def test_step_damped_then_conceded():
  # A step disturbance is resisted transiently, then forgotten with dc_tau.
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0, dc_tau=2.0))   # fast tau for test
  _run(a, 0.84, 1000)                            # settle
  out_peak = 0.0
  for _ in range(300):                           # 3 s after step to gap 1.2
    out, _t = a.update(0.0, _mv_at(1.2), 17.0, False, lat_delay=0.6)
    out_peak = max(out_peak, out)
  assert out_peak > 1e-5                         # transient resistance fired
  out, t = _run(a, 1.2, 2000)                    # 20 s >> dc_tau -> conceded
  assert abs(out) < 1e-6
  assert abs(t['gap_dc'] - 1.2) < 0.05


def test_ac_deadband_ignores_micro_noise():
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
  _run(a, 0.84, 1000)
  out = None
  for i in range(400):                           # ±0.05 m oscillation < deadband 0.10
    g = 0.84 + 0.05 * (1 if (i // 50) % 2 else -1)
    out, _t = a.update(0.0, _mv_at(g), 17.0, False, lat_delay=0.6)
  assert abs(out) < 1e-6


def test_lane_change_resets_dc_and_concedes_new_lane():
  # After a lane change the DC re-seeds: the new lane's position — whatever
  # it is — is immediately the reference. No settle-nudge, no old-lane memory.
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
  _run(a, 0.84, 1000)
  a.update(0.0, _mv_at(0.5), 17.0, True, lat_delay=0.6)    # LC tick, new lane geometry
  assert a.gap_dc is None                        # reset during LC
  out, t = _run(a, 0.5, 500)                     # post-LC: 0.5 is the new normal
  assert abs(out) < 1e-6
  assert abs(t['gap_dc'] - 0.5) < 0.05


def test_dc_freezes_when_untrusted():
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
  _run(a, 0.84, 1000)
  dc0 = a.gap_dc
  mv_low = _mv_geo(_XS, _flat(-1.75), _flat(1.75), left_p=0.3)   # authority 0
  for _ in range(500):
    a.update(0.0, mv_low, 17.0, False, lat_delay=0.6)
  assert a.gap_dc == dc0                         # frozen, not adapted/reset
  none_mv = SimpleNamespace(laneLines=[], laneLineProbs=[])
  for _ in range(500):
    a.update(0.0, none_mv, 17.0, False, lat_delay=0.6)
  assert a.gap_dc == dc0                         # dropouts keep the reference


def test_hard_floors_still_absolute():
  # At the extremes the ABSOLUTE band still governs — even though the DC
  # would concede, a 0.2 m gap keeps a sustained (best-effort) push away.
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
  out, _t = _run(a, 0.2, 3000)
  assert out < -1e-5                             # steering right, sustained
  a2 = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
  out, _t = _run(a2, 1.6, 3000)
  assert out > 1e-5                              # ceiling: steering left


def test_lane_change_hard_zeros_bias_built_from_drift():
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
  _run(a, 0.84, 1000)
  for i in range(600):                           # build bias from a drift
    a.update(0.0, _mv_at(0.84 + 0.05 * (i / 100.0)), 17.0, False, lat_delay=0.6)
  assert a.kappa_bias > 1e-5
  out, t = a.update(0.01, _mv_at(1.14), 17.0, True, lat_delay=0.6)
  assert a.kappa_bias == 0.0                     # hard-zeroed on the LC tick
  assert out == 0.01                             # bit-identical passthrough
  assert t['state'] == 'model'


def test_rate_limited_on_step():
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0, kappa_rate_max=0.002))
  _run(a, 0.84, 1000)
  a.gap_filt = 1.3                               # warm the filters into a step
  a.gap_pred_filt = 1.3
  _o, t = a.update(0.0, _mv_at(1.3), 17.0, False, lat_delay=0.6)
  assert abs(t['kappa_bias']) <= 0.002 * 0.01 + 1e-12


def test_dc_frozen_during_hard_floor_and_assists_recovery():
  # Review finding: the DC must never learn the excursion the hard floor is
  # fighting. Frozen during the floor regime, the post-floor AC term points
  # WITH the recovery (or is silent) — never back toward the floor.
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
  _run(a, 0.84, 1000)                              # seed DC at 0.84
  dc0 = a.gap_dc
  _run(a, 0.2, 1000)                               # deep in the low floor, 10 s
  # DC may creep only during the gap-filter's ~1.3 s transit into the floor
  # regime (tau=0.7 lag before gap_filt crosses 0.3); once in_floor, it is
  # frozen. The property: the excursion's BULK (0.64 m) is never learned.
  assert abs(a.gap_dc - dc0) < 0.05                # learned <8% of the excursion
  out, t = a.update(0.0, _mv_at(0.7), 17.0, False, lat_delay=0.6)  # recovering
  # gap 0.7 < dc 0.84 -> excess_ac negative -> steer right (gap-increasing):
  # assists the recovery toward the old line; must NOT push back to the floor
  for _ in range(200):
    out, t = a.update(0.0, _mv_at(0.7), 17.0, False, lat_delay=0.6)
  assert out <= 1e-6                               # never positive (toward floor)


def test_dc_seed_requires_authority():
  # Final-review fix: an untrusted line (authority 0) must not SEED the DC —
  # a stale seed would snap back as a nudge when confidence returns.
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
  mv_low = _mv_geo(_XS, _flat(-1.75), _flat(1.75), left_p=0.3)
  for _ in range(200):
    a.update(0.0, mv_low, 17.0, False, lat_delay=0.6)
  assert a.gap_dc is None                          # never seeded untrusted
  out, t = _run(a, 0.84, 200)                      # trust arrives -> seeds now
  assert abs(t['gap_dc'] - 0.84) < 0.05
  assert abs(out) < 1e-6


# --- Addendum 2026-07-27: asymmetric damping near the driver-side line ---
# asym_gap suppresses only the toward-line direction (excess > 0 -> pursuit
# steers toward the driver line, restoring the DC) when gap_filt is close to
# the line. Away-pushes (excess <= 0, sagging further toward the line) are
# always kept, at any asym_gap setting.

def _mv_at_right(gap):
  # right-driver scene with the RIGHT line placed to give the requested gap
  # (mirrors _mv_at's left-driver construction)
  y = gap + 0.91
  return _mv_geo(_XS, _flat(-1.75), _flat(y))


def test_asym_gap_near_line_escape_not_opposed():
  # Escape: gap rising away from a near-line DC (asym_gap default 0.6). The
  # toward-line direction is suppressed throughout -> bias relaxes to (stays
  # at) zero and never grows toward the line.
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))   # asym_gap=0.6 default
  _run(a, 0.45, 1000)                                 # seed a near-line DC
  max_bias = 0.0
  for i in range(600):                                # escape stays < asym_gap (0.6)
    g = 0.45 + 0.025 * (i / 100.0)
    a.update(0.0, _mv_at(g), 17.0, False, lat_delay=0.6)
    max_bias = max(max_bias, a.kappa_bias)
  assert max_bias < 1e-6                              # never grew toward the line
  assert abs(a.kappa_bias) < 1e-6                      # relaxed to zero


def test_asym_gap_near_line_sag_still_pushed_away():
  # Sag: gap falling further toward a near-line DC. The away-push direction
  # is kept, and identically so regardless of asym_gap (only excess > 0 is
  # ever touched by the gate).
  a_asym = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))               # asym_gap=0.6
  a_sym = LaneAnchor(AnchorConfig(pred_delay_mult=2.0, asym_gap=0.0))  # symmetric
  _run(a_asym, 0.45, 1000)
  _run(a_sym, 0.45, 1000)
  out_asym = out_sym = None
  for i in range(1000):                               # sag stays above the hard floor (0.3)
    g = 0.45 - 0.0145 * (i / 100.0)
    out_asym, _t = a_asym.update(0.0, _mv_at(g), 17.0, False, lat_delay=0.6)
    out_sym, _t = a_sym.update(0.0, _mv_at(g), 17.0, False, lat_delay=0.6)
  assert out_asym < -1e-6                             # pushed away (right, left-driver)
  assert out_asym == out_sym                          # bit-identical to the symmetric run


def test_asym_gap_far_from_line_unchanged():
  # Gap stays comfortably above asym_gap (0.6) throughout -> the gate never
  # engages, so asym_gap=0.6 and asym_gap=0.0 must produce bit-identical
  # bias trajectories (same scenario as test_drift_is_damped).
  a1 = LaneAnchor(AnchorConfig(pred_delay_mult=2.0, asym_gap=0.6))
  a2 = LaneAnchor(AnchorConfig(pred_delay_mult=2.0, asym_gap=0.0))
  _run(a1, 0.84, 1000)
  _run(a2, 0.84, 1000)
  biases1, biases2 = [], []
  for i in range(600):
    g = 0.84 + 0.05 * (i / 100.0)
    out1, _t1 = a1.update(0.0, _mv_at(g), 17.0, False, lat_delay=0.6)
    out2, _t2 = a2.update(0.0, _mv_at(g), 17.0, False, lat_delay=0.6)
    biases1.append(out1)
    biases2.append(out2)
  assert biases1 == biases2                           # bit-identical trajectories


def test_asym_gap_zero_disables_suppression():
  # Same near-line escape as test_asym_gap_near_line_escape_not_opposed, but
  # with asym_gap=0.0: the OLD symmetric behavior returns — a nonzero
  # toward-line bias appears, proving the switch works both ways.
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0, asym_gap=0.0))
  _run(a, 0.45, 1000)
  out = None
  for i in range(600):
    g = 0.45 + 0.025 * (i / 100.0)
    out, _t = a.update(0.0, _mv_at(g), 17.0, False, lat_delay=0.6)
  assert out > 1e-6                                   # toward-line bias, NOT suppressed


def test_asym_gap_side_agnostic_right_driver():
  # Same escape-not-opposed scenario, mirrored onto a right-side driver
  # (mirrored y values, per the existing right-side test convention).
  a = LaneAnchor(AnchorConfig(driver_side='right', pred_delay_mult=2.0))  # asym_gap=0.6
  out = t = None
  for _ in range(1000):
    out, t = a.update(0.0, _mv_at_right(0.45), 17.0, False, lat_delay=0.6)
  max_bias = 0.0
  for i in range(600):
    g = 0.45 + 0.025 * (i / 100.0)
    a.update(0.0, _mv_at_right(g), 17.0, False, lat_delay=0.6)
    max_bias = max(max_bias, abs(a.kappa_bias))
  assert max_bias < 1e-6                              # never grew toward the (right) line
  assert abs(a.kappa_bias) < 1e-6


def test_disable_resets_dc_for_fresh_ab():
  # Deliberate toggle-off forgets the reference; re-enable starts fresh on
  # the current line (clean A/B), instead of nudging toward a stale DC.
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
  _run(a, 0.84, 1000)
  a.cfg.enable = False
  a.update(0.0, _mv_at(0.84), 17.0, False, lat_delay=0.6)
  assert a.gap_dc is None
  a.cfg.enable = True
  out, t = _run(a, 1.3, 500)                       # re-enable at a NEW position
  assert abs(t['gap_dc'] - 1.3) < 0.05             # fresh seed, no old memory
  assert abs(out) < 1e-6                           # conceded immediately
