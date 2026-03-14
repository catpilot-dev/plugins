"""Speed limit sign overlay — renders Vienna-style speed limit sign on the HUD.

Registered as a ui.render_overlay hook callback. Uses stock UI lib directly:
  - pyray for drawing primitives (circles, text)
  - gui_app.font() for font access
  - measure_text_cached() for text measurement
  - ui_state.sm for speedLimitState data
  - plugin bus for tap-to-confirm toggle
"""
import pyray as rl
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached

# Layout constants — sign diameter matches MAX block width, centered below it
SPEED_SIGN_RADIUS_METRIC = 100    # diameter 200 = metric MAX width
SPEED_SIGN_RADIUS_IMPERIAL = 86   # diameter 172 = imperial MAX width
SPEED_SIGN_BORDER_RATIO = 0.1     # red ring = 1/10 diameter (Vienna Convention)
SPEED_SIGN_GAP = 30               # vertical gap between MAX block bottom and sign top
SPEED_SIGN_FONT_SIZE = 84
SOURCE_LABELS = {0: "OSM", 1: "SIGN", 2: "~"}

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
speed_limit_capping = False  # True when confirmed limit is actively capping MAX
speed_limit_ceiling = 0.0    # Effective ceiling: limit + offset (km/h)
_tap_hold_until = 0.0  # Hold local confirmed state until this time (monotonic)


def _ensure_init():
  """Lazy init — fonts and imports deferred until first render frame."""
  global _font_bold, _font_medium, ui_state
  if _font_bold is None:
    from openpilot.selfdrive.ui.ui_state import ui_state as _ui_state
    ui_state = _ui_state
    _font_bold = gui_app.font(FontWeight.BOLD)
    _font_medium = gui_app.font(FontWeight.MEDIUM)


def _update_state():
  """Read speedLimitState from SubMaster (updated each frame by ui_state)."""
  global _speed_limit, _speed_limit_source, _speed_limit_confirmed, speed_limit_capping, speed_limit_ceiling
  import time
  sm = ui_state.sm
  if sm.recv_frame.get("speedLimitState", 0) > 0:
    sls = sm['speedLimitState']
    _speed_limit = sls.speedLimit
    _speed_limit_source = sls.source.raw if hasattr(sls.source, 'raw') else int(sls.source)
    # Don't overwrite local confirmed state during tap hold period
    if time.monotonic() >= _tap_hold_until:
      _speed_limit_confirmed = sls.confirmed
    speed_limit_capping = _speed_limit_confirmed and _speed_limit > 0
    if speed_limit_capping:
      # Tiered offset matching planner_hook logic
      if _speed_limit <= 50:
        offset_pct = 40
      elif _speed_limit <= 60:
        offset_pct = 30
      else:
        offset_pct = 10
      speed_limit_ceiling = _speed_limit * (1 + offset_pct / 100.0)
    else:
      speed_limit_ceiling = 0.0
  else:
    speed_limit_capping = False
    speed_limit_ceiling = 0.0


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
  text_size = measure_text_cached(_font_bold, speed_text, SPEED_SIGN_FONT_SIZE)
  rl.draw_text_ex(
    _font_bold,
    speed_text,
    rl.Vector2(cx - text_size.x / 2, cy - text_size.y / 2),
    SPEED_SIGN_FONT_SIZE,
    0,
    text_color,
  )

  # Source indicator below the sign
  source_label = SOURCE_LABELS.get(_speed_limit_source, "?")
  source_size = measure_text_cached(_font_medium, source_label, 36)
  rl.draw_text_ex(
    _font_medium,
    source_label,
    rl.Vector2(cx - source_size.x / 2, cy + r + 10),
    36,
    0,
    rl.Color(200, 200, 200, alpha),
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
  """Hook callback for ui.state_subscriptions.

  Adds speedLimitState to the UI SubMaster so the overlay can read speed limit data.
  """
  if 'speedLimitState' not in services:
    services.append('speedLimitState')
  return services


def on_hud_set_speed_override(default, max_color, set_speed_color, set_speed, is_metric):
  """Hook callback for ui.hud_set_speed_override.

  When speed limit is actively capping cruise, dim the MAX block and show
  the ceiling speed instead of the user's set speed.
  """
  if not speed_limit_capping or speed_limit_ceiling <= 0:
    return default

  import pyray as rl
  KM_TO_MILE = 0.621371
  ceiling = speed_limit_ceiling if is_metric else speed_limit_ceiling * KM_TO_MILE
  return {
    "max_color": rl.Color(max_color.r, max_color.g, max_color.b, 128),
    "set_speed_color": rl.Color(255, 255, 255, 128),
    "set_speed_text": str(round(ceiling)),
  }


def on_render_overlay(default, content_rect):
  """Hook callback for ui.render_overlay. Called each frame inside scissor mode."""
  _ensure_init()
  _update_state()

  if _speed_limit > 0:
    _draw_speed_limit_sign(content_rect)
    _handle_tap(content_rect)

  return None
