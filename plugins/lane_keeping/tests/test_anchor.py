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
