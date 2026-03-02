"""Tests for speedlimitd UI overlay — speed limit sign rendering via ui.render_overlay hook."""
import math
import pytest
from unittest.mock import MagicMock, patch, PropertyMock
import pyray as rl


# Mock openpilot imports before importing the module under test
@pytest.fixture(autouse=True)
def mock_openpilot(monkeypatch):
  """Mock all openpilot dependencies so tests run without openpilot installed."""
  import sys

  # Create mock modules for openpilot imports
  mock_modules = {}
  for mod in [
    'openpilot', 'openpilot.common', 'openpilot.common.params',
    'openpilot.selfdrive', 'openpilot.selfdrive.ui', 'openpilot.selfdrive.ui.ui_state',
    'openpilot.system', 'openpilot.system.ui', 'openpilot.system.ui.lib',
    'openpilot.system.ui.lib.application', 'openpilot.system.ui.lib.text_measure',
  ]:
    mock_modules[mod] = MagicMock()

  # Set up specific mock behaviors
  mock_font_weight = MagicMock()
  mock_font_weight.BOLD = 'BOLD'
  mock_font_weight.MEDIUM = 'MEDIUM'
  mock_modules['openpilot.system.ui.lib.application'].FontWeight = mock_font_weight

  mock_gui_app = MagicMock()
  mock_font_bold = MagicMock(name='font_bold')
  mock_font_medium = MagicMock(name='font_medium')
  mock_gui_app.font.side_effect = lambda w: mock_font_bold if w == 'BOLD' else mock_font_medium
  mock_modules['openpilot.system.ui.lib.application'].gui_app = mock_gui_app

  # measure_text_cached returns a Vector2-like object
  mock_text_size = MagicMock()
  mock_text_size.x = 40.0
  mock_text_size.y = 30.0
  mock_modules['openpilot.system.ui.lib.text_measure'].measure_text_cached = MagicMock(return_value=mock_text_size)

  # Params mock
  mock_params_instance = MagicMock()
  mock_modules['openpilot.common.params'].Params = MagicMock(return_value=mock_params_instance)

  # ui_state mock with SubMaster-like sm
  mock_sm = MagicMock()
  mock_sm.recv_frame = {}
  mock_sm.__getitem__ = MagicMock(return_value=MagicMock())
  mock_ui_state = MagicMock()
  mock_ui_state.sm = mock_sm
  mock_modules['openpilot.selfdrive.ui.ui_state'].ui_state = mock_ui_state

  for mod_name, mod_mock in mock_modules.items():
    monkeypatch.setitem(sys.modules, mod_name, mod_mock)

  yield {
    'gui_app': mock_gui_app,
    'font_bold': mock_font_bold,
    'font_medium': mock_font_medium,
    'params': mock_params_instance,
    'ui_state': mock_ui_state,
    'sm': mock_sm,
    'measure_text_cached': mock_modules['openpilot.system.ui.lib.text_measure'].measure_text_cached,
  }


@pytest.fixture
def overlay(mock_openpilot):
  """Import ui_overlay fresh for each test (resets module-level state)."""
  import importlib
  import plugins.speedlimitd.ui_overlay as mod
  importlib.reload(mod)
  return mod


@pytest.fixture
def content_rect():
  return rl.Rectangle(30, 30, 1780, 1020)


class TestConstants:
  def test_sign_dimensions(self, overlay):
    assert overlay.SPEED_SIGN_RADIUS == 60
    assert overlay.SPEED_SIGN_BORDER == 8
    assert overlay.SPEED_SIGN_X == 120
    assert overlay.SPEED_SIGN_Y == 330
    assert overlay.SPEED_SIGN_FONT_SIZE == 56

  def test_source_labels(self, overlay):
    assert overlay.SOURCE_LABELS == {0: "OSM", 1: "SIGN", 2: "~"}


class TestLazyInit:
  def test_fonts_none_before_init(self, overlay):
    assert overlay._font_bold is None
    assert overlay._font_medium is None
    assert overlay._params is None

  def test_ensure_init_loads_fonts(self, overlay, mock_openpilot):
    overlay._ensure_init()
    assert overlay._font_bold is not None
    assert overlay._font_medium is not None
    assert overlay._params is not None
    mock_openpilot['gui_app'].font.assert_any_call('BOLD')
    mock_openpilot['gui_app'].font.assert_any_call('MEDIUM')

  def test_ensure_init_idempotent(self, overlay, mock_openpilot):
    overlay._ensure_init()
    overlay._ensure_init()
    # font() called exactly twice (BOLD + MEDIUM), not four times
    assert mock_openpilot['gui_app'].font.call_count == 2


class TestUpdateState:
  def test_no_speed_limit_state(self, overlay, mock_openpilot):
    """When no speedLimitState received, state stays at defaults."""
    mock_openpilot['sm'].recv_frame = {}
    overlay._update_state()
    assert overlay._speed_limit == 0.0
    assert overlay._speed_limit_source == 2
    assert overlay._speed_limit_confirmed is False

  def test_speed_limit_state_received(self, overlay, mock_openpilot):
    """When speedLimitState has been received, state is updated."""
    mock_sls = MagicMock()
    mock_sls.speedLimit = 80.0
    mock_sls.source.raw = 0  # OSM
    mock_sls.confirmed = True
    mock_openpilot['sm'].recv_frame = {"speedLimitState": 5}
    mock_openpilot['sm'].__getitem__ = MagicMock(return_value=mock_sls)

    overlay._update_state()
    assert overlay._speed_limit == 80.0
    assert overlay._speed_limit_source == 0
    assert overlay._speed_limit_confirmed is True

  def test_speed_limit_unconfirmed(self, overlay, mock_openpilot):
    mock_sls = MagicMock()
    mock_sls.speedLimit = 60.0
    mock_sls.source.raw = 1  # SIGN
    mock_sls.confirmed = False
    mock_openpilot['sm'].recv_frame = {"speedLimitState": 1}
    mock_openpilot['sm'].__getitem__ = MagicMock(return_value=mock_sls)

    overlay._update_state()
    assert overlay._speed_limit == 60.0
    assert overlay._speed_limit_confirmed is False

  def test_source_fallback_to_int(self, overlay, mock_openpilot):
    """When source has no .raw attribute, falls back to int()."""
    mock_sls = MagicMock(spec=[])  # empty spec — no auto-created attributes
    mock_sls.speedLimit = 100.0
    mock_sls.source = 2  # plain int, hasattr(source, 'raw') is False
    mock_sls.confirmed = False
    mock_openpilot['sm'].recv_frame = {"speedLimitState": 3}
    mock_openpilot['sm'].__getitem__ = MagicMock(return_value=mock_sls)

    overlay._update_state()
    assert overlay._speed_limit_source == 2


class TestHandleTap:
  def test_no_tap_does_nothing(self, overlay, mock_openpilot, content_rect):
    overlay._ensure_init()
    overlay._speed_limit = 80.0
    overlay._speed_limit_confirmed = False

    with patch('pyray.is_mouse_button_released', return_value=False):
      overlay._handle_tap(content_rect)

    assert overlay._speed_limit_confirmed is False
    mock_openpilot['params'].put.assert_not_called()

  def test_tap_inside_sign_toggles_confirmed(self, overlay, mock_openpilot, content_rect):
    overlay._ensure_init()
    overlay._speed_limit = 80.0
    overlay._speed_limit_confirmed = False

    # Tap at the sign center
    sign_cx = content_rect.x + overlay.SPEED_SIGN_X
    sign_cy = content_rect.y + overlay.SPEED_SIGN_Y
    mock_pos = MagicMock()
    mock_pos.x = sign_cx
    mock_pos.y = sign_cy

    with patch('pyray.is_mouse_button_released', return_value=True), \
         patch('pyray.get_mouse_position', return_value=mock_pos):
      overlay._handle_tap(content_rect)

    assert overlay._speed_limit_confirmed is True
    mock_openpilot['params'].put.assert_any_call("SpeedLimitConfirmed", "1")
    mock_openpilot['params'].put.assert_any_call("SpeedLimitValue", "80.0")

  def test_tap_outside_sign_no_toggle(self, overlay, mock_openpilot, content_rect):
    overlay._ensure_init()
    overlay._speed_limit = 80.0
    overlay._speed_limit_confirmed = False

    # Tap far from sign
    mock_pos = MagicMock()
    mock_pos.x = 800.0
    mock_pos.y = 600.0

    with patch('pyray.is_mouse_button_released', return_value=True), \
         patch('pyray.get_mouse_position', return_value=mock_pos):
      overlay._handle_tap(content_rect)

    assert overlay._speed_limit_confirmed is False
    mock_openpilot['params'].put.assert_not_called()

  def test_tap_on_edge_of_sign(self, overlay, mock_openpilot, content_rect):
    """Tap exactly at radius boundary should still register."""
    overlay._ensure_init()
    overlay._speed_limit = 60.0
    overlay._speed_limit_confirmed = True

    sign_cx = content_rect.x + overlay.SPEED_SIGN_X
    sign_cy = content_rect.y + overlay.SPEED_SIGN_Y
    r = overlay.SPEED_SIGN_RADIUS

    # Tap at exactly the radius distance (on the boundary)
    mock_pos = MagicMock()
    mock_pos.x = sign_cx + r  # exactly at edge
    mock_pos.y = sign_cy

    with patch('pyray.is_mouse_button_released', return_value=True), \
         patch('pyray.get_mouse_position', return_value=mock_pos):
      overlay._handle_tap(content_rect)

    # Should toggle: confirmed True → False
    assert overlay._speed_limit_confirmed is False
    mock_openpilot['params'].put.assert_any_call("SpeedLimitConfirmed", "0")

  def test_tap_just_outside_radius(self, overlay, mock_openpilot, content_rect):
    """Tap 1px beyond radius should not register."""
    overlay._ensure_init()
    overlay._speed_limit = 60.0
    overlay._speed_limit_confirmed = False

    sign_cx = content_rect.x + overlay.SPEED_SIGN_X
    sign_cy = content_rect.y + overlay.SPEED_SIGN_Y
    r = overlay.SPEED_SIGN_RADIUS

    mock_pos = MagicMock()
    mock_pos.x = sign_cx + r + 1.0  # just outside
    mock_pos.y = sign_cy

    with patch('pyray.is_mouse_button_released', return_value=True), \
         patch('pyray.get_mouse_position', return_value=mock_pos):
      overlay._handle_tap(content_rect)

    assert overlay._speed_limit_confirmed is False


class TestOnRenderOverlay:
  def test_returns_none(self, overlay, mock_openpilot, content_rect):
    """Void hook — always returns None."""
    with patch('pyray.is_mouse_button_released', return_value=False), \
         patch('pyray.draw_circle'), patch('pyray.draw_text_ex'):
      result = overlay.on_render_overlay(None, content_rect)
    assert result is None

  def test_no_draw_when_speed_limit_zero(self, overlay, mock_openpilot, content_rect):
    """When speed limit is 0, nothing should be drawn."""
    mock_openpilot['sm'].recv_frame = {}
    with patch('pyray.draw_circle') as mock_draw_circle, \
         patch('pyray.is_mouse_button_released', return_value=False):
      overlay.on_render_overlay(None, content_rect)

    mock_draw_circle.assert_not_called()

  def test_draws_when_speed_limit_active(self, overlay, mock_openpilot, content_rect):
    """When speed limit > 0, sign should be drawn."""
    mock_sls = MagicMock()
    mock_sls.speedLimit = 80.0
    mock_sls.source.raw = 0
    mock_sls.confirmed = True
    mock_openpilot['sm'].recv_frame = {"speedLimitState": 1}
    mock_openpilot['sm'].__getitem__ = MagicMock(return_value=mock_sls)

    with patch('pyray.draw_circle') as mock_draw_circle, \
         patch('pyray.draw_text_ex') as mock_draw_text, \
         patch('pyray.is_mouse_button_released', return_value=False):
      overlay.on_render_overlay(None, content_rect)

    # Should draw outer ring + inner fill = 2 draw_circle calls
    assert mock_draw_circle.call_count == 2
    # Should draw speed text + source label = 2 draw_text_ex calls
    assert mock_draw_text.call_count == 2

  def test_alpha_confirmed_vs_unconfirmed(self, overlay, mock_openpilot, content_rect):
    """Confirmed = alpha 255, unconfirmed = alpha 128."""
    mock_sls = MagicMock()
    mock_sls.speedLimit = 60.0
    mock_sls.source.raw = 1
    mock_openpilot['sm'].recv_frame = {"speedLimitState": 1}
    mock_openpilot['sm'].__getitem__ = MagicMock(return_value=mock_sls)

    for confirmed, expected_alpha in [(True, 255), (False, 128)]:
      mock_sls.confirmed = confirmed
      with patch('pyray.draw_circle') as mock_draw_circle, \
           patch('pyray.draw_text_ex'), \
           patch('pyray.is_mouse_button_released', return_value=False):
        # Reload to reset state
        import importlib
        importlib.reload(overlay)
        overlay = __import__('plugins.speedlimitd.ui_overlay', fromlist=['ui_overlay'])
        overlay.on_render_overlay(None, content_rect)

      # Check alpha on the outer ring (first draw_circle call)
      ring_color = mock_draw_circle.call_args_list[0][0][3]
      assert ring_color.a == expected_alpha


class TestDrawPositioning:
  def test_sign_position_relative_to_content_rect(self, overlay, mock_openpilot, content_rect):
    """Sign should be positioned relative to content_rect origin."""
    mock_sls = MagicMock()
    mock_sls.speedLimit = 100.0
    mock_sls.source.raw = 0
    mock_sls.confirmed = True
    mock_openpilot['sm'].recv_frame = {"speedLimitState": 1}
    mock_openpilot['sm'].__getitem__ = MagicMock(return_value=mock_sls)

    with patch('pyray.draw_circle') as mock_draw_circle, \
         patch('pyray.draw_text_ex'), \
         patch('pyray.is_mouse_button_released', return_value=False):
      overlay.on_render_overlay(None, content_rect)

    # First call is outer ring — check cx, cy
    call_args = mock_draw_circle.call_args_list[0][0]
    expected_cx = int(content_rect.x) + overlay.SPEED_SIGN_X
    expected_cy = int(content_rect.y) + overlay.SPEED_SIGN_Y
    assert call_args[0] == expected_cx
    assert call_args[1] == expected_cy
    assert call_args[2] == overlay.SPEED_SIGN_RADIUS
