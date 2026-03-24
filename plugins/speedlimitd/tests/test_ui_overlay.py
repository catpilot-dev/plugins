"""Tests for speedlimitd UI overlay — speed limit sign rendering via ui.render_overlay hook."""
import sys
import pytest
from unittest.mock import MagicMock, patch, PropertyMock

# Mock pyray at module level — it's only available on devices with GPU/display
if 'pyray' not in sys.modules:
  sys.modules['pyray'] = MagicMock()
import pyray as rl


# Mock openpilot imports before importing the module under test
@pytest.fixture(autouse=True)
def mock_openpilot(monkeypatch):
  """Mock all openpilot dependencies so tests run without openpilot installed."""
  # Create mock modules for openpilot imports
  mock_modules = {}
  for mod in [
    'openpilot', 'openpilot.common',
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

  # ui_state mock with SubMaster-like sm
  mock_sm = MagicMock()
  mock_sm.recv_frame = {}
  mock_sm.__getitem__ = MagicMock(return_value=MagicMock())
  mock_ui_state = MagicMock()
  mock_ui_state.sm = mock_sm
  mock_ui_state.is_metric = True
  mock_modules['openpilot.selfdrive.ui.ui_state'].ui_state = mock_ui_state

  # Mock plugin_bus to avoid ZMQ in tests
  mock_plugin_bus = MagicMock()
  mock_pub_instance = MagicMock()
  mock_plugin_bus.PluginPub = MagicMock(return_value=mock_pub_instance)
  mock_modules['openpilot.selfdrive.plugins'] = MagicMock()
  mock_modules['openpilot.selfdrive.plugins.plugin_bus'] = mock_plugin_bus

  for mod_name, mod_mock in mock_modules.items():
    monkeypatch.setitem(sys.modules, mod_name, mod_mock)

  yield {
    'gui_app': mock_gui_app,
    'font_bold': mock_font_bold,
    'font_medium': mock_font_medium,
    'plugin_bus_pub': mock_pub_instance,
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


class _Rect:
  """Simple rectangle with numeric attributes (rl.Rectangle is mocked)."""
  def __init__(self, x, y, w, h):
    self.x = x; self.y = y; self.width = w; self.height = h


@pytest.fixture
def content_rect():
  return _Rect(30, 30, 1780, 1020)


class TestConstants:
  def test_sign_dimensions(self, overlay):
    assert overlay.SPEED_SIGN_RADIUS_METRIC == 100   # diameter 200 = MAX block width
    assert overlay.SPEED_SIGN_RADIUS_IMPERIAL == 86   # diameter 172
    assert overlay.SPEED_SIGN_BORDER_RATIO == 0.1  # Vienna Convention: 1/10 diameter
    assert overlay.SPEED_SIGN_FONT_SIZE == 84


class TestLazyInit:
  def test_fonts_none_before_init(self, overlay):
    assert overlay._font_bold is None
    assert overlay._font_medium is None

  def test_ensure_init_loads_fonts(self, overlay, mock_openpilot):
    overlay._ensure_init()
    assert overlay._font_bold is not None
    assert overlay._font_medium is not None
    mock_openpilot['gui_app'].font.assert_any_call('BOLD')
    mock_openpilot['gui_app'].font.assert_any_call('MEDIUM')

  def test_ensure_init_idempotent(self, overlay, mock_openpilot):
    overlay._ensure_init()
    overlay._ensure_init()
    # font() called exactly twice (BOLD + MEDIUM), not four times
    assert mock_openpilot['gui_app'].font.call_count == 2


class TestStateSubscriptionsHook:
  """Test the ui.state_subscriptions hook callback."""

  def test_adds_speed_limit_state(self, overlay):
    services = ["modelV2", "controlsState", "deviceState"]
    result = overlay.on_state_subscriptions(services)
    assert 'speedLimitState' in result

  def test_does_not_duplicate(self, overlay):
    services = ["modelV2", "speedLimitState", "deviceState"]
    result = overlay.on_state_subscriptions(services)
    assert result.count('speedLimitState') == 1

  def test_preserves_existing_services(self, overlay):
    services = ["modelV2", "controlsState"]
    result = overlay.on_state_subscriptions(services)
    assert "modelV2" in result
    assert "controlsState" in result
    assert "speedLimitState" in result

  def test_returns_same_list(self, overlay):
    """Modifies in-place and returns the same list object."""
    services = ["modelV2"]
    result = overlay.on_state_subscriptions(services)
    assert result is services


class TestUpdateState:
  def test_no_speed_limit_state(self, overlay, mock_openpilot):
    """When no speedLimitState received, state stays at defaults."""
    overlay._ensure_init()
    mock_openpilot['sm'].recv_frame = {}
    overlay._update_state()
    assert overlay._speed_limit == 0.0
    assert overlay._speed_limit_source == 2
    assert overlay._speed_limit_confirmed is False

  def test_speed_limit_state_received(self, overlay, mock_openpilot):
    """When speedLimitState has been received, state is updated."""
    overlay._ensure_init()
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

  def test_ceiling_computed_when_confirmed(self, overlay, mock_openpilot):
    """Ceiling = limit * (1 + offset%) when confirmed."""
    overlay._ensure_init()
    mock_sls = MagicMock()
    mock_sls.speedLimit = 80.0
    mock_sls.source.raw = 0
    mock_sls.confirmed = True
    mock_openpilot['sm'].recv_frame = {"speedLimitState": 1}
    mock_openpilot['sm'].__getitem__ = MagicMock(return_value=mock_sls)

    overlay._update_state()
    assert overlay.speed_limit_capping is True
    assert overlay.speed_limit_ceiling == pytest.approx(88.0)  # 80 * 1.10

  def test_ceiling_uses_tiered_offset(self, overlay, mock_openpilot):
    """Ceiling uses tiered offset: >60 kph → 10%, 51-60 → 30%, ≤50 → 40%."""
    overlay._ensure_init()

    # Highway: 100 kph → 10% offset → ceiling 110
    mock_sls = MagicMock()
    mock_sls.speedLimit = 100.0
    mock_sls.source.raw = 0
    mock_sls.confirmed = True
    mock_openpilot['sm'].recv_frame = {"speedLimitState": 1}
    mock_openpilot['sm'].__getitem__ = MagicMock(return_value=mock_sls)
    overlay._update_state()
    assert overlay.speed_limit_ceiling == pytest.approx(110.0)

    # Mid speed: 60 kph → 30% offset → ceiling 78
    mock_sls.speedLimit = 60.0
    overlay._update_state()
    assert overlay.speed_limit_ceiling == pytest.approx(78.0)

    # Low speed: 40 kph → 40% offset → ceiling 56
    mock_sls.speedLimit = 40.0
    overlay._update_state()
    assert overlay.speed_limit_ceiling == pytest.approx(56.0)

  def test_ceiling_zero_when_unconfirmed(self, overlay, mock_openpilot):
    """Ceiling is 0 when not confirmed."""
    overlay._ensure_init()
    mock_sls = MagicMock()
    mock_sls.speedLimit = 80.0
    mock_sls.source.raw = 0
    mock_sls.confirmed = False
    mock_openpilot['sm'].recv_frame = {"speedLimitState": 1}
    mock_openpilot['sm'].__getitem__ = MagicMock(return_value=mock_sls)

    overlay._update_state()
    assert overlay.speed_limit_capping is False
    assert overlay.speed_limit_ceiling == 0.0

  def test_speed_limit_unconfirmed(self, overlay, mock_openpilot):
    overlay._ensure_init()
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
    overlay._ensure_init()
    mock_sls = MagicMock(spec=[])  # empty spec — no auto-created attributes
    mock_sls.speedLimit = 100.0
    mock_sls.source = 2  # plain int, hasattr(source, 'raw') is False
    mock_sls.confirmed = False
    mock_openpilot['sm'].recv_frame = {"speedLimitState": 3}
    mock_openpilot['sm'].__getitem__ = MagicMock(return_value=mock_sls)

    overlay._update_state()
    assert overlay._speed_limit_source == 2


def _mouse_release(x, y):
  """Create a mock MouseEvent with left_released=True at (x, y)."""
  pos = MagicMock()
  pos.x = x
  pos.y = y
  ev = MagicMock()
  ev.pos = pos
  ev.left_released = True
  return ev


def _mouse_noop():
  """Create a mock MouseEvent with left_released=False."""
  ev = MagicMock()
  ev.left_released = False
  return ev


class TestHandleTap:
  def test_no_tap_does_nothing(self, overlay, mock_openpilot, content_rect):
    overlay._ensure_init()
    overlay._speed_limit = 80.0
    overlay._speed_limit_confirmed = False

    mock_openpilot['gui_app'].mouse_events = [_mouse_noop()]
    overlay._handle_tap(content_rect)

    assert overlay._speed_limit_confirmed is False

  def test_tap_inside_sign_toggles_confirmed(self, overlay, mock_openpilot, content_rect):
    overlay._ensure_init()
    overlay._speed_limit = 80.0
    overlay._speed_limit_confirmed = False

    # Tap at the sign center
    cx, cy, r = overlay._sign_geometry(content_rect)
    mock_openpilot['gui_app'].mouse_events = [_mouse_release(cx, cy)]
    overlay._handle_tap(content_rect)

    assert overlay._speed_limit_confirmed is True
    mock_openpilot['plugin_bus_pub'].send.assert_called_with({'action': 'toggle_confirm'})

  def test_tap_outside_sign_no_toggle(self, overlay, mock_openpilot, content_rect):
    overlay._ensure_init()
    overlay._speed_limit = 80.0
    overlay._speed_limit_confirmed = False

    # Tap far from sign
    mock_openpilot['gui_app'].mouse_events = [_mouse_release(800.0, 600.0)]
    overlay._handle_tap(content_rect)

    assert overlay._speed_limit_confirmed is False

  def test_tap_on_edge_of_sign(self, overlay, mock_openpilot, content_rect):
    """Tap exactly at radius boundary should still register."""
    overlay._ensure_init()
    overlay._speed_limit = 60.0
    overlay._speed_limit_confirmed = True

    cx, cy, r = overlay._sign_geometry(content_rect)

    # Tap at exactly the radius distance (on the boundary)
    mock_openpilot['gui_app'].mouse_events = [_mouse_release(cx + r, cy)]
    overlay._handle_tap(content_rect)

    # Should toggle: confirmed True → False
    assert overlay._speed_limit_confirmed is False
    mock_openpilot['plugin_bus_pub'].send.assert_called_with({'action': 'toggle_confirm'})

  def test_tap_just_outside_radius(self, overlay, mock_openpilot, content_rect):
    """Tap 1px beyond radius should not register."""
    overlay._ensure_init()
    overlay._speed_limit = 60.0
    overlay._speed_limit_confirmed = False

    cx, cy, r = overlay._sign_geometry(content_rect)

    mock_openpilot['gui_app'].mouse_events = [_mouse_release(cx + r + 1.0, cy)]
    overlay._handle_tap(content_rect)

    assert overlay._speed_limit_confirmed is False


class TestOnRenderOverlay:
  def test_returns_none(self, overlay, mock_openpilot, content_rect):
    """Void hook — always returns None."""
    mock_openpilot['gui_app'].mouse_events = []
    with patch('pyray.draw_circle'), patch('pyray.draw_text_ex'):
      result = overlay.on_render_overlay(None, content_rect)
    assert result is None

  def test_no_draw_when_speed_limit_zero(self, overlay, mock_openpilot, content_rect):
    """When speed limit is 0, nothing should be drawn."""
    mock_openpilot['sm'].recv_frame = {}
    mock_openpilot['gui_app'].mouse_events = []
    with patch('pyray.draw_circle') as mock_draw_circle:
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

    mock_openpilot['gui_app'].mouse_events = []
    with patch('pyray.draw_circle') as mock_draw_circle, \
         patch('pyray.draw_text_ex') as mock_draw_text:
      overlay.on_render_overlay(None, content_rect)

    # Should draw outer ring + inner fill = 2 draw_circle calls
    assert mock_draw_circle.call_count == 2
    # Should draw speed text
    assert mock_draw_text.call_count == 1

  def test_alpha_confirmed_vs_unconfirmed(self, overlay, mock_openpilot, content_rect):
    """Confirmed = alpha 255, unconfirmed = alpha 128.

    The draw code uses: alpha = 255 if _speed_limit_confirmed else 128
    We verify by intercepting Color() calls through the mocked pyray module.
    """
    mock_sls = MagicMock()
    mock_sls.speedLimit = 60.0
    mock_sls.source.raw = 1
    mock_openpilot['sm'].recv_frame = {"speedLimitState": 1}
    mock_openpilot['sm'].__getitem__ = MagicMock(return_value=mock_sls)

    pyray_mock = sys.modules['pyray']
    for confirmed, expected_alpha in [(True, 255), (False, 128)]:
      mock_sls.confirmed = confirmed
      # Track Color calls via the pyray mock in sys.modules
      color_alphas = []
      pyray_mock.Color = MagicMock(side_effect=lambda r, g, b, a: color_alphas.append(a) or MagicMock(a=a))

      import importlib
      importlib.reload(overlay)
      overlay = __import__('plugins.speedlimitd.ui_overlay', fromlist=['ui_overlay'])
      overlay.on_render_overlay(None, content_rect)

      # All Color() calls should use the same alpha for this frame
      assert expected_alpha in color_alphas, f"Expected alpha {expected_alpha} in {color_alphas}"


class TestDrawPositioning:
  def test_sign_centered_under_max_block(self, overlay, mock_openpilot, content_rect):
    """Sign center x should match MAX block center x."""
    overlay._ensure_init()
    cx, cy, r = overlay._sign_geometry(content_rect)
    # MAX block center for metric: rect.x + 60 + 200/2 = rect.x + 160
    expected_cx = int(content_rect.x) + 60 + (172 - 200) // 2 + 200 // 2
    assert cx == expected_cx
    assert r == overlay.SPEED_SIGN_RADIUS_METRIC

  def test_sign_below_max_block_with_gap(self, overlay, mock_openpilot, content_rect):
    """Sign top should be GAP pixels below MAX block bottom."""
    overlay._ensure_init()
    cx, cy, r = overlay._sign_geometry(content_rect)
    max_bottom = int(content_rect.y) + 45 + 204
    assert cy == max_bottom + overlay.SPEED_SIGN_GAP + r

  def test_imperial_uses_smaller_radius(self, overlay, mock_openpilot, content_rect):
    """Imperial mode uses 172px diameter sign."""
    overlay._ensure_init()
    mock_openpilot['ui_state'].is_metric = False
    cx, cy, r = overlay._sign_geometry(content_rect)
    assert r == overlay.SPEED_SIGN_RADIUS_IMPERIAL

  def test_sign_position_relative_to_content_rect(self, overlay, mock_openpilot, content_rect):
    """Sign should be positioned relative to content_rect origin."""
    mock_sls = MagicMock()
    mock_sls.speedLimit = 100.0
    mock_sls.source.raw = 0
    mock_sls.confirmed = True
    mock_openpilot['sm'].recv_frame = {"speedLimitState": 1}
    mock_openpilot['sm'].__getitem__ = MagicMock(return_value=mock_sls)

    mock_openpilot['gui_app'].mouse_events = []
    with patch('pyray.draw_circle') as mock_draw_circle, \
         patch('pyray.draw_text_ex'):
      overlay.on_render_overlay(None, content_rect)

    # First call is outer ring — check cx, cy, r
    call_args = mock_draw_circle.call_args_list[0][0]
    expected_cx, expected_cy, expected_r = overlay._sign_geometry(content_rect)
    assert call_args[0] == expected_cx
    assert call_args[1] == expected_cy
    assert call_args[2] == expected_r
