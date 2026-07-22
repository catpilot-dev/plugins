import os, sys
from types import SimpleNamespace
_PLUGIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PLUGIN_DIR not in sys.path:
  sys.path.insert(0, _PLUGIN_DIR)
from anchor import AnchorConfig, LaneAnchor


def _mv(left_y, right_y, left_p=1.0, right_p=1.0):
  return SimpleNamespace(
    laneLines=[SimpleNamespace(y=[0.0]), SimpleNamespace(y=[left_y]),
               SimpleNamespace(y=[right_y]), SimpleNamespace(y=[0.0])],
    laneLineProbs=[0.0, left_p, right_p, 0.0])


def test_gap_left_driver():
  a = LaneAnchor(AnchorConfig(driver_side='left', half_width=0.91))
  assert a.driver_idx == 1 and a.side_sign == 1.0
  # left line 1.75 m to the left (+y) -> wheel gap = 1.75 - 0.91 = 0.84
  assert abs(a._gap(_mv(left_y=1.75, right_y=-1.75)) - 0.84) < 1e-9


def test_gap_right_driver():
  a = LaneAnchor(AnchorConfig(driver_side='right', half_width=0.91))
  assert a.driver_idx == 2 and a.side_sign == -1.0
  # right line 1.75 m to the right (-y) -> wheel gap = 1.75 - 0.91 = 0.84
  assert abs(a._gap(_mv(left_y=1.75, right_y=-1.75)) - 0.84) < 1e-9


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
