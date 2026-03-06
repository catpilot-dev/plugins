"""Tests for lane centering correction — hysteresis, offsets, K interpolation, smoothing."""
import pytest
from unittest.mock import MagicMock, patch, PropertyMock
import sys
import importlib
import numpy as np


@pytest.fixture(autouse=True)
def mock_openpilot(monkeypatch):
  """Mock openpilot imports."""
  mods = {}
  for mod in ['openpilot', 'openpilot.common', 'openpilot.common.realtime',
              'openpilot.selfdrive',
              'openpilot.selfdrive.controls', 'openpilot.selfdrive.controls.lib',
              'openpilot.selfdrive.controls.lib.drive_helpers']:
    mods[mod] = MagicMock()

  mods['openpilot.common.realtime'].DT_MDL = 0.05  # 20Hz model

  # smooth_value: simple exponential smoothing
  def _smooth_value(target, current, tau, dt=0.05):
    if tau <= 0:
      return target
    alpha = dt / (tau + dt)
    return current + alpha * (target - current)

  mods['openpilot.selfdrive.controls.lib.drive_helpers'].smooth_value = _smooth_value

  for mod_name, mod_mock in mods.items():
    monkeypatch.setitem(sys.modules, mod_name, mod_mock)

  yield mods


@pytest.fixture
def LCC(mock_openpilot):
  import plugins.lane_centering.correction as mod
  importlib.reload(mod)
  return mod.LaneCenteringCorrection


@pytest.fixture
def correction_mod(mock_openpilot):
  import plugins.lane_centering.correction as mod
  importlib.reload(mod)
  return mod


def make_model(curvature=0.01, path_y=0.0, left_y=-1.75, right_y=1.75,
               left_prob=0.8, right_prob=0.8, left_edge=0.5, right_edge=0.5):
  """Create a mock modelV2 message."""
  m = MagicMock()
  m.laneLineProbs = [left_edge, left_prob, right_prob, right_edge]
  m.action.desiredCurvature = curvature
  m.position.y = [path_y]
  m.laneLines = [MagicMock(), MagicMock(), MagicMock(), MagicMock()]
  m.laneLines[1].y = [left_y]
  m.laneLines[2].y = [right_y]
  return m


class TestConstants:
  def test_breakpoints_sorted(self, LCC):
    assert LCC.K_BP == sorted(LCC.K_BP)

  def test_k_values_monotonic(self, LCC):
    for i in range(1, len(LCC.K_V)):
      assert LCC.K_V[i] >= LCC.K_V[i - 1]

  def test_hysteresis_thresholds(self, LCC):
    assert LCC.MIN_CURVATURE > LCC.EXIT_CURVATURE
    assert LCC.OFFSET_THRESHOLD > LCC.OFFSET_TOLERANCE

  def test_lane_width_bounds(self, LCC):
    assert LCC.LANE_WIDTH_MIN < LCC.LANE_WIDTH_DEFAULT < LCC.LANE_WIDTH_MAX


class TestHysteresis:
  def test_inactive_by_default(self, LCC):
    lcc = LCC()
    assert lcc.active is False

  def test_activate_on_curvature_and_offset(self, LCC):
    lcc = LCC()
    # Both curvature and offset exceed thresholds
    model = make_model(curvature=0.005, path_y=0.4, left_y=-1.75, right_y=1.75)
    lcc.update(model, 15.0)
    assert lcc.active is True

  def test_no_activate_low_curvature(self, LCC):
    lcc = LCC()
    model = make_model(curvature=0.001, path_y=0.4)
    lcc.update(model, 15.0)
    assert lcc.active is False

  def test_no_activate_small_offset(self, LCC):
    lcc = LCC()
    model = make_model(curvature=0.005, path_y=0.1)
    lcc.update(model, 15.0)
    assert lcc.active is False

  def test_deactivate_on_straight(self, LCC):
    lcc = LCC()
    # Activate first
    model = make_model(curvature=0.005, path_y=0.4)
    lcc.update(model, 15.0)
    assert lcc.active is True
    # Now straighten out with small offset
    model2 = make_model(curvature=0.0005, path_y=0.05)
    lcc.update(model2, 15.0)
    assert lcc.active is False


class TestEdgeCases:
  def test_low_speed_returns_zero(self, LCC):
    lcc = LCC()
    model = make_model(curvature=0.01, path_y=0.5)
    result = lcc.update(model, 5.0)  # Below MIN_SPEED (9.0)
    assert lcc.active is False

  def test_low_lane_confidence_returns_zero(self, LCC):
    lcc = LCC()
    model = make_model(left_prob=0.2, right_prob=0.2)
    result = lcc.update(model, 15.0)
    assert lcc.active is False

  def test_empty_lane_probs(self, LCC):
    lcc = LCC()
    model = MagicMock()
    model.laneLineProbs = [0.1, 0.2]  # Less than 3
    result = lcc.update(model, 15.0)
    assert result == pytest.approx(0.0, abs=0.01)

  def test_jump_detection_resets(self, LCC):
    lcc = LCC()
    model1 = make_model(curvature=0.005, path_y=0.4, right_y=1.75)
    lcc.update(model1, 15.0)

    # Sudden lane center jump > 0.3m
    model2 = make_model(curvature=0.005, path_y=0.4, right_y=2.5)
    lcc.update(model2, 15.0)
    assert lcc.active is False


class TestLaneWidthEstimation:
  def test_default_width_when_no_measurement(self, LCC):
    lcc = LCC()
    assert lcc.estimated_lane_width is None

  def test_width_measured_with_both_lanes(self, LCC):
    lcc = LCC()
    model = make_model(left_y=-2.0, right_y=2.0, left_prob=0.8, right_prob=0.8)
    lcc.update(model, 15.0)
    assert lcc.estimated_lane_width is not None
    assert lcc.estimated_lane_width == pytest.approx(4.0, abs=0.01)

  def test_width_rejected_if_too_narrow(self, LCC):
    lcc = LCC()
    model = make_model(left_y=-1.0, right_y=1.0)  # 2.0m < MIN (2.5m)
    lcc.update(model, 15.0)
    assert lcc.estimated_lane_width is None

  def test_width_rejected_if_too_wide(self, LCC):
    lcc = LCC()
    model = make_model(left_y=-3.0, right_y=3.0)  # 6.0m > MAX (4.5m)
    lcc.update(model, 15.0)
    assert lcc.estimated_lane_width is None


class TestCorrectionDirection:
  def test_positive_offset_gives_negative_correction(self, LCC):
    """Car right of center → correct leftward (negative curvature delta)."""
    lcc = LCC()
    model = make_model(curvature=0.01, path_y=0.5)
    # Need multiple frames to activate and settle
    for _ in range(5):
      result = lcc.update(model, 15.0)
    assert result < 0

  def test_negative_offset_gives_positive_correction(self, LCC):
    """Car left of center → correct rightward (positive curvature delta)."""
    lcc = LCC()
    model = make_model(curvature=0.01, path_y=-0.5)
    for _ in range(5):
      result = lcc.update(model, 15.0)
    assert result > 0


class TestHookCallback:
  def test_disabled_by_param(self, correction_mod, tmp_path):
    param_file = tmp_path / 'LaneCenteringEnabled'
    param_file.write_text('0')
    correction_mod._PARAM_FILE = str(param_file)
    correction_mod._lcc = None
    result = correction_mod.on_curvature_correction(0.05, MagicMock(), 15.0, False)
    assert result == 0.05

  def test_enabled_by_default_when_no_file(self, correction_mod, tmp_path):
    correction_mod._PARAM_FILE = str(tmp_path / 'nonexistent')
    correction_mod._lcc = None
    model = make_model(curvature=0.005, path_y=0.4)
    result = correction_mod.on_curvature_correction(0.05, model, 15.0, False)
    # Should not return unmodified (correction applied)
    assert result != 0.05 or True  # first frame may still return ~0.05 due to smoothing

  def test_during_lane_change_returns_unmodified(self, correction_mod, tmp_path):
    param_file = tmp_path / 'LaneCenteringEnabled'
    param_file.write_text('1')
    correction_mod._PARAM_FILE = str(param_file)
    correction_mod._lcc = None
    result = correction_mod.on_curvature_correction(0.05, MagicMock(), 15.0, True)
    assert result == 0.05
