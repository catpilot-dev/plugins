"""BMW temperature overlay — renders coolant and oil temps on the onroad HUD.

Registered as a ui.render_overlay hook callback. Draws two stacked temperature
readings at the bottom-right corner of the onroad HUD: coolant (top) and oil (bottom).

Temps received via plugin bus topic 'bmw_temps' (published by carstate.py at 0.2 Hz).

Color thresholds match the COD dashboard:
  Blue (#3b82f6)   — cold (< 60°C)
  Green (#22c55e)   — normal operating
  Yellow (#eab308)  — warning (coolant >= 95°C, oil >= 120°C)
  Red (#ef4444)     — critical (coolant >= 105°C, oil >= 140°C)
"""
import os
import pyray as rl

_PLUGIN_DIR = os.path.dirname(os.path.abspath(__file__))

# Layout — positioned at bottom-right corner
FONT_SIZE = 56
LINE_HEIGHT = 62
RIGHT_MARGIN = 30
BOTTOM_MARGIN = 30

# COD dashboard colors
COLOR_COLD = rl.Color(59, 130, 246, 220)     # #3b82f6
COLOR_NORMAL = rl.Color(34, 197, 94, 220)    # #22c55e
COLOR_WARNING = rl.Color(234, 179, 8, 220)   # #eab308
COLOR_CRITICAL = rl.Color(239, 68, 68, 220)  # #ef4444

# Module-level state (lazy init)
_ui_state = None
_font = None
_measure = None
_temp_sub = None
_cached_coolant = 0.0
_cached_oil = 0.0


def _ensure_init():
  global _ui_state, _font, _measure
  if _font is None:
    from openpilot.selfdrive.ui.ui_state import ui_state
    from openpilot.system.ui.lib.application import gui_app, FontWeight
    from openpilot.system.ui.lib.text_measure import measure_text_cached
    _ui_state = ui_state
    _font = gui_app.font(FontWeight.SEMI_BOLD)
    _measure = measure_text_cached


def _is_enabled():
  try:
    with open(os.path.join(_PLUGIN_DIR, 'data', 'TemperatureOverlay')) as f:
      return f.read().strip() != '0'
  except (FileNotFoundError, OSError):
    return True  # default on


def _read_temps():
  """Read coolant and oil temps from plugin bus (cached)."""
  global _temp_sub, _cached_coolant, _cached_oil
  try:
    if _temp_sub is None:
      from openpilot.selfdrive.plugins.plugin_bus import PluginSub
      _temp_sub = PluginSub(['bmw_temps'])
    msg = _temp_sub.drain()
    if msg is not None:
      _, data = msg
      _cached_coolant = float(data.get('coolant', 0))
      _cached_oil = float(data.get('oil', 0))
  except Exception:
    pass
  return _cached_coolant, _cached_oil


def _coolant_color(temp):
  if temp >= 105:
    return COLOR_CRITICAL
  if temp >= 95:
    return COLOR_WARNING
  if temp >= 60:
    return COLOR_NORMAL
  return COLOR_COLD


def _oil_color(temp):
  if temp >= 140:
    return COLOR_CRITICAL
  if temp >= 120:
    return COLOR_WARNING
  if temp >= 60:
    return COLOR_NORMAL
  return COLOR_COLD


def on_render_overlay(default, content_rect):
  """Hook callback for ui.render_overlay. Called each frame inside scissor mode."""
  _ensure_init()

  if not _is_enabled():
    return None

  coolant, oil = _read_temps()

  # Skip if no data (both zero means engine off or no signal)
  if coolant == 0 and oil == 0:
    return None

  is_metric = _ui_state.is_metric
  unit = "°C" if is_metric else "°F"

  coolant_disp = coolant if is_metric else coolant * 9 / 5 + 32
  oil_disp = oil if is_metric else oil * 9 / 5 + 32

  coolant_text = f"{coolant_disp:.0f}{unit}"
  oil_text = f"{oil_disp:.0f}{unit}"

  # Position: right-aligned at bottom-right corner, stacked (coolant above oil)
  coolant_size = _measure(_font, coolant_text, FONT_SIZE)
  oil_size = _measure(_font, oil_text, FONT_SIZE)
  max_w = max(coolant_size.x, oil_size.x)
  total_h = LINE_HEIGHT + oil_size.y
  pad = 12

  x_right = int(content_rect.x + content_rect.width) - RIGHT_MARGIN
  y_top = int(content_rect.y + content_rect.height) - BOTTOM_MARGIN - total_h - pad * 2 + pad

  # 30% opacity black background
  bg_rect = rl.Rectangle(x_right - max_w - pad, y_top - pad, max_w + pad * 2, total_h + pad * 2)
  rl.draw_rectangle_rounded(bg_rect, 0.2, 10, rl.Color(0, 0, 0, 77))

  # Coolant temperature (top)
  coolant_x = x_right - coolant_size.x
  rl.draw_text_ex(_font, coolant_text, rl.Vector2(coolant_x, y_top), FONT_SIZE, 0, _coolant_color(coolant))

  # Oil temperature (below coolant)
  oil_x = x_right - oil_size.x
  rl.draw_text_ex(_font, oil_text, rl.Vector2(oil_x, y_top + LINE_HEIGHT), FONT_SIZE, 0, _oil_color(oil))

  return None
