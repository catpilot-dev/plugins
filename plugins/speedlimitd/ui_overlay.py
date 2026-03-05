"""Speed limit sign overlay — renders Vienna-style speed limit sign on the HUD.

Registered as a ui.render_overlay hook callback. Uses stock UI lib directly:
  - pyray for drawing primitives (circles, text)
  - gui_app.font() for font access
  - measure_text_cached() for text measurement
  - ui_state.sm for speedLimitState data
  - Params for tap-to-confirm persistence
"""
import math
import pyray as rl
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached

# Layout constants (same position as previous hud_renderer implementation)
SPEED_SIGN_RADIUS = 60      # px
SPEED_SIGN_BORDER = 8       # red ring thickness
SPEED_SIGN_X = 120          # center x (below MAX box)
SPEED_SIGN_Y = 330          # center y
SPEED_SIGN_FONT_SIZE = 56
SOURCE_LABELS = {0: "OSM", 1: "SIGN", 2: "~"}

# Module-level state (initialized lazily on first render frame)
# ui_state and Params are lazily imported in _ensure_init to avoid circular import
# when the plugin registry loads this module during ui_state.__init__.
ui_state = None
_font_bold = None
_font_medium = None
_params = None
_speed_limit = 0.0
_speed_limit_source = 2  # roadTypeInference default
_speed_limit_confirmed = False


def _ensure_init():
  """Lazy init — fonts and imports deferred until first render frame."""
  global _font_bold, _font_medium, _params, ui_state
  if _font_bold is None:
    from openpilot.common.params import Params
    from openpilot.selfdrive.ui.ui_state import ui_state as _ui_state
    ui_state = _ui_state
    _font_bold = gui_app.font(FontWeight.BOLD)
    _font_medium = gui_app.font(FontWeight.MEDIUM)
    _params = Params()


def _update_state():
  """Read speedLimitState from SubMaster (updated each frame by ui_state)."""
  global _speed_limit, _speed_limit_source, _speed_limit_confirmed
  sm = ui_state.sm
  if sm.recv_frame.get("speedLimitState", 0) > 0:
    sls = sm['speedLimitState']
    _speed_limit = sls.speedLimit
    _speed_limit_source = sls.source.raw if hasattr(sls.source, 'raw') else int(sls.source)
    _speed_limit_confirmed = sls.confirmed


def _draw_speed_limit_sign(content_rect):
  """Draw Vienna-style speed limit sign (red circle, white fill, black number).

  50% opacity when unconfirmed (suggestion), 100% when confirmed (active).
  Small source indicator below: "OSM" / "SIGN" / "~"
  """
  cx = int(content_rect.x) + SPEED_SIGN_X
  cy = int(content_rect.y) + SPEED_SIGN_Y
  r = SPEED_SIGN_RADIUS
  alpha = 255 if _speed_limit_confirmed else 128

  # Red outer ring
  rl.draw_circle(cx, cy, r, rl.Color(220, 30, 30, alpha))

  # White inner fill
  rl.draw_circle(cx, cy, r - SPEED_SIGN_BORDER, rl.Color(255, 255, 255, alpha))

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
  source_size = measure_text_cached(_font_medium, source_label, 28)
  rl.draw_text_ex(
    _font_medium,
    source_label,
    rl.Vector2(cx - source_size.x / 2, cy + r + 8),
    28,
    0,
    rl.Color(200, 200, 200, alpha),
  )


def _handle_tap(content_rect):
  """Check for tap on speed limit sign — toggle confirmed state."""
  global _speed_limit_confirmed
  if not rl.is_mouse_button_released(rl.MOUSE_BUTTON_LEFT):
    return
  pos = rl.get_mouse_position()
  cx = content_rect.x + SPEED_SIGN_X
  cy = content_rect.y + SPEED_SIGN_Y
  dx = pos.x - cx
  dy = pos.y - cy
  if math.sqrt(dx * dx + dy * dy) <= SPEED_SIGN_RADIUS:
    _speed_limit_confirmed = not _speed_limit_confirmed
    _params.put("SpeedLimitConfirmed", "1" if _speed_limit_confirmed else "0")
    _params.put("SpeedLimitValue", str(_speed_limit))


def on_state_subscriptions(services):
  """Hook callback for ui.state_subscriptions.

  Adds speedLimitState to the UI SubMaster so the overlay can read speed limit data.
  """
  if 'speedLimitState' not in services:
    services.append('speedLimitState')
  return services


def on_render_overlay(default, content_rect):
  """Hook callback for ui.render_overlay. Called each frame inside scissor mode."""
  _ensure_init()
  _update_state()

  if _speed_limit > 0:
    _draw_speed_limit_sign(content_rect)
    _handle_tap(content_rect)

  return None
