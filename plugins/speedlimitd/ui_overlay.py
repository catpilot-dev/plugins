"""Speed limit sign overlay — renders Vienna-style speed limit sign on the HUD.

Registered as a ui.render_overlay hook callback. Uses stock UI lib directly:
  - pyray for drawing primitives (circles, text)
  - fonts.py shared helper for font access
  - ui_state.sm for speedLimitState data
  - plugin bus for tap-to-confirm toggle
"""
import pyray as rl
from openpilot.system.ui.lib.application import gui_app, FontWeight
from fonts import get_font, measure

# Layout constants — sign diameter matches MAX block width, centered below it
SPEED_SIGN_RADIUS_METRIC = 100    # diameter 200 = metric MAX width
SPEED_SIGN_RADIUS_IMPERIAL = 86   # diameter 172 = imperial MAX width
SPEED_SIGN_BORDER_RATIO = 0.1     # red ring = 1/10 diameter (Vienna Convention)
SPEED_SIGN_GAP = 30               # vertical gap between MAX block bottom and sign top
SPEED_SIGN_FONT_SIZE = 84

# MAX block layout (mirrors hud_renderer.py UIConfig)
_MAX_X_OFFSET = 60
_MAX_Y_OFFSET = 45
_MAX_WIDTH_METRIC = 200
_MAX_WIDTH_IMPERIAL = 172
_MAX_HEIGHT = 204


# Module-level state (initialized lazily on first render frame)
# ui_state is lazily imported in _ensure_init to avoid circular import
# when the plugin registry loads this module during ui_state.__init__.
ui_state = None
_font_bold = None
_font_medium = None
_speed_limit = 0.0
_speed_limit_source = 2  # roadTypeInference default
_speed_limit_confirmed = False
_tap_hold_until = 0.0  # Hold local confirmed state until this time (monotonic)


def _ensure_init():
  """Lazy init — imports deferred until first render frame."""
  global _font_bold, _font_medium, ui_state
  if _font_bold is None:
    font_bold = get_font(FontWeight.BOLD)
    if font_bold is None:
      return  # fonts not ready yet — retry next frame
    from openpilot.selfdrive.ui.ui_state import ui_state as _ui_state
    ui_state = _ui_state
    _font_bold = font_bold
    _font_medium = get_font(FontWeight.MEDIUM)


_sl_sub = None
_sl_data = None


def _update_state():
  """Read speedLimitState from plugin bus."""
  global _speed_limit, _speed_limit_source, _speed_limit_confirmed
  global _sl_sub, _sl_data
  import time

  # Recreate sub if socket was recycled (speedlimitd restart deletes + rebinds)
  import os
  _sl_socket_path = '/tmp/plugin_bus/speedLimitState'
  if _sl_sub is not None and not os.path.exists(_sl_socket_path):
    try:
      _sl_sub.close()
    except Exception:
      pass
    _sl_sub = None

  if _sl_sub is None and os.path.exists(_sl_socket_path):
    try:
      from openpilot.selfdrive.plugins.plugin_bus import PluginSub
      _sl_sub = PluginSub(['speedLimitState'])
    except Exception:
      pass

  if _sl_sub is not None:
    try:
      msg = _sl_sub.drain('speedLimitState')
      if msg is not None and isinstance(msg, tuple) and len(msg) == 2:
        _, _sl_data = msg
    except Exception:
      pass

  if _sl_data is not None:
    _speed_limit = _sl_data.get('speedLimit', 0)
    _speed_limit_source = _sl_data.get('source', 2)
    if time.monotonic() >= _tap_hold_until:
      _speed_limit_confirmed = _sl_data.get('confirmed', False)


def _sign_geometry(content_rect):
  """Compute sign center (cx, cy) and radius, aligned to MAX block."""
  is_metric = ui_state.is_metric
  max_w = _MAX_WIDTH_METRIC if is_metric else _MAX_WIDTH_IMPERIAL
  r = SPEED_SIGN_RADIUS_METRIC if is_metric else SPEED_SIGN_RADIUS_IMPERIAL
  # MAX block x uses imperial width for centering offset
  max_x = int(content_rect.x) + _MAX_X_OFFSET + (_MAX_WIDTH_IMPERIAL - max_w) // 2
  cx = max_x + max_w // 2
  max_bottom = int(content_rect.y) + _MAX_Y_OFFSET + _MAX_HEIGHT
  cy = max_bottom + SPEED_SIGN_GAP + r
  return cx, cy, r


def _draw_speed_limit_sign(content_rect):
  """Draw Vienna-style speed limit sign (red circle, white fill, black number).

  50% opacity when unconfirmed (suggestion), 100% when confirmed (active).
  Small source indicator below: "OSM" / "SIGN" / "~"
  """
  cx, cy, r = _sign_geometry(content_rect)

  # Register interactive zone so tapping sign doesn't toggle sidebar
  try:
    from openpilot.selfdrive.ui.onroad.overlay_zones import register_circle_zone
    register_circle_zone(cx, cy, r)
  except ImportError:
    pass
  alpha = 255 if _speed_limit_confirmed else 128

  # Red outer ring
  rl.draw_circle(cx, cy, r, rl.Color(220, 30, 30, alpha))

  # White inner fill (border = 1/10 diameter per Vienna Convention)
  border = r * SPEED_SIGN_BORDER_RATIO * 2
  rl.draw_circle(cx, cy, r - border, rl.Color(255, 255, 255, alpha))

  # Speed number (black)
  speed_text = str(round(_speed_limit))
  text_color = rl.Color(0, 0, 0, alpha)
  text_size = measure(_font_bold, speed_text, SPEED_SIGN_FONT_SIZE)
  rl.draw_text_ex(
    _font_bold,
    speed_text,
    rl.Vector2(cx - text_size.x / 2, cy - text_size.y / 2),
    SPEED_SIGN_FONT_SIZE,
    0,
    text_color,
  )



_tap_pub = None


def _handle_tap(content_rect):
  """Check for tap on speed limit sign — toggle confirmed state via plugin bus."""
  global _speed_limit_confirmed, _tap_hold_until, _tap_pub
  cx, cy, r = _sign_geometry(content_rect)
  for ev in gui_app.mouse_events:
    if not ev.left_released:
      continue
    dx = ev.pos.x - cx
    dy = ev.pos.y - cy
    if dx * dx + dy * dy <= r * r:
      import time
      _speed_limit_confirmed = not _speed_limit_confirmed
      _tap_hold_until = time.monotonic() + 2.0  # hold local state until speedlimitd catches up
      try:
        if _tap_pub is None:
          from openpilot.selfdrive.plugins.plugin_bus import PluginPub
          _tap_pub = PluginPub('speedlimit_cmd_ui')
        _tap_pub.send({'action': 'toggle_confirm'})
      except Exception:
        pass
      break


def on_state_subscriptions(services):
  """Hook callback for ui.state_subscriptions (no-op, speedLimitState moved to plugin_bus)."""
  return services


def on_hud_set_speed_override(default, max_color, set_speed_color, set_speed, is_metric):
  """Hook callback for ui.hud_set_speed_override.

  MAX block always shows user's set cruise speed. Speed limit info is
  shown in the speed limit sign overlay instead.
  """
  return default


def _show_sign_enabled():
  """Check ShowSpeedLimitSign param (cached, refreshed lazily)."""
  global _show_sign_cache, _show_sign_check_time
  import time
  now = time.monotonic()
  if not hasattr(_show_sign_enabled, '_cache_until') or now > _show_sign_enabled._cache_until:
    import os
    val = None
    try:
      param_path = '/data/plugins/speedlimitd/data/ShowSpeedLimitSign'
      if os.path.isfile(param_path):
        val = open(param_path).read().strip()
    except Exception:
      pass
    _show_sign_enabled._value = val != '0'
    _show_sign_enabled._cache_until = now + 2.0  # re-check every 2s
  return _show_sign_enabled._value


def on_render_overlay(default, content_rect):
  """Hook callback for ui.render_overlay. Called each frame inside scissor mode."""
  _ensure_init()
  _update_state()

  if _speed_limit > 0 and _show_sign_enabled():
    _draw_speed_limit_sign(content_rect)
    _handle_tap(content_rect)

  return None
