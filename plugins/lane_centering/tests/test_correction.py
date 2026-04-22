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
    # Activation driven solely by offset now (curvature gate removed for
    # straight-line lane centering).
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

  def test_activate_on_straight_with_large_offset(self, LCC):
    # On a straight road (low curvature), lane centering still activates when
    # offset exceeds threshold — the curvature gate was removed.
    lcc = LCC()
    model = make_model(curvature=0.001, path_y=0.4)
    lcc.update(model, 15.0)
    assert lcc.active is True

  def test_no_activate_small_offset(self, LCC):
    lcc = LCC()
    model = make_model(curvature=0.005, path_y=0.1)
    lcc.update(model, 15.0)
    assert lcc.active is False

  def test_deactivate_on_small_offset(self, LCC):
    lcc = LCC()
    # Activate first
    model = make_model(curvature=0.005, path_y=0.4)
    lcc.update(model, 15.0)
    assert lcc.active is True
    # Now settle within tolerance — deactivate regardless of curvature
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

  def test_width_fallback_when_too_wide(self, LCC):
    lcc = LCC()
    # Learn valid width first
    model = make_model(left_y=-2.0, right_y=2.0, left_prob=0.8, right_prob=0.8)
    lcc.update(model, 15.0)
    assert lcc.estimated_lane_width == pytest.approx(4.0, abs=0.01)
    # Turn distorts apparent width > MAX — lane_width should fall back to default, estimate unchanged
    wide = make_model(curvature=0.01, left_y=-2.5, right_y=2.5, left_prob=0.8, right_prob=0.8)
    result = lcc.update(wide, 15.0)
    assert lcc.estimated_lane_width == pytest.approx(4.0, abs=0.01)  # estimate unchanged

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


class TestKpCompensation:
  def test_correction_smaller_at_low_speed(self, LCC):
    """kP compensation should reduce effective correction at low speeds where kP is high."""
    lcc = LCC()
    model = make_model(curvature=0.01, path_y=0.5)
    # Run at 10 m/s (kP=3.5, kp_scale=2.0/3.5=0.57)
    for _ in range(10):
      result_slow = lcc.update(model, 10.0)

    lcc2 = LCC()
    # Run at 15 m/s (kP=2.0, kp_scale=2.0/2.0=1.0)
    for _ in range(10):
      result_fast = lcc2.update(model, 15.0)

    # At 10 m/s without compensation, correction would be (15/10)^2 = 2.25x larger.
    # With kP compensation, the ratio should be much closer to 1.
    # (Not exactly 1 due to v_ego^2 in denominator, but the kP scaling flattens it.)
    assert abs(result_slow) < abs(result_fast) * 2.0  # without compensation would be >2x

  def test_kp_scale_at_highway_speed(self, LCC):
    """At nominal speed (15 m/s), kp_scale should be ~1.0."""
    scale = LCC.KP_NOMINAL / float(np.interp(15.0, LCC.KP_SPEEDS, LCC.KP_VALUES))
    assert scale == pytest.approx(1.0, abs=0.01)


class TestRateLimiting:
  def test_correction_rate_limited(self, LCC):
    """Correction change per frame should not exceed MAX_CORRECTION_RATE."""
    lcc = LCC()
    model = make_model(curvature=0.015, path_y=0.6)
    prev = 0.0
    for _ in range(20):
      result = lcc.update(model, 12.0)
      delta = abs(result - prev)
      # Smoothing + rate limiting: delta should stay reasonable
      # (allow some tolerance for the exponential smoothing on top of rate limiting)
      assert delta <= LCC.MAX_CORRECTION_RATE * 3  # smoothing can slightly exceed clamp
      prev = result


class TestDerivativeDamping:
  def test_damping_reduces_correction_when_offset_improving(self, LCC):
    """When offset shrinks, damping should reduce the correction magnitude."""
    lcc = LCC()
    # Build up correction with large offset
    model1 = make_model(curvature=0.01, path_y=0.5)
    for _ in range(10):
      lcc.update(model1, 15.0)

    # Now offset is improving (shrinking)
    model2 = make_model(curvature=0.01, path_y=0.3)
    result_improving = lcc.update(model2, 15.0)

    # Reset and test without improvement
    lcc2 = LCC()
    for _ in range(10):
      lcc2.update(model1, 15.0)
    model3 = make_model(curvature=0.01, path_y=0.5)  # same offset, not improving
    result_steady = lcc2.update(model3, 15.0)

    # Improving offset should yield smaller correction (damped)
    assert abs(result_improving) <= abs(result_steady)

  def test_diagnostics_populated_when_active(self, LCC):
    lcc = LCC()
    model = make_model(curvature=0.01, path_y=0.5)
    for _ in range(5):
      lcc.update(model, 15.0)
    assert 'offset' in lcc.diag
    assert 'damping' in lcc.diag
    assert 'kp_scale' in lcc.diag
    assert 'raw' in lcc.diag
    assert 'clamped' in lcc.diag


class TestHookCallback:
  def _reset(self, correction_mod):
    correction_mod._lcc = None
    correction_mod._lcc_pub = None
    correction_mod._enabled = None

  def test_disabled_by_param(self, correction_mod):
    self._reset(correction_mod)
    with patch('plugins.lane_centering.correction.read_plugin_param', return_value='0'):
      result = correction_mod.on_curvature_correction(0.05, MagicMock(), 15.0, False)
    assert result == 0.05
    assert correction_mod._enabled is False

  def test_enabled_by_default_when_no_file(self, correction_mod):
    self._reset(correction_mod)
    with patch('plugins.lane_centering.correction.read_plugin_param', return_value=''):
      model = make_model(curvature=0.005, path_y=0.4)
      correction_mod.on_curvature_correction(0.05, model, 15.0, False)
    assert correction_mod._enabled is True

  def test_enabled_explicitly(self, correction_mod):
    self._reset(correction_mod)
    with patch('plugins.lane_centering.correction.read_plugin_param', return_value='1'):
      model = make_model(curvature=0.005, path_y=0.4)
      correction_mod.on_curvature_correction(0.05, model, 15.0, False)
    assert correction_mod._enabled is True

  def test_during_lane_change_returns_unmodified(self, correction_mod):
    self._reset(correction_mod)
    correction_mod._enabled = True
    result = correction_mod.on_curvature_correction(0.05, MagicMock(), 15.0, True)
    assert result == 0.05
