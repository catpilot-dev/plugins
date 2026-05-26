"""Road info overlay — renders current road name and wayRef at bottom-center of HUD.

Shows the road identifier from speedlimitd's OSM tile query, useful for
verifying that the speed limit daemon is reading the correct road.

Format: "S20 外环高速" or just "外环高速" when no wayRef is available.

ALL imports MUST be lazy (inside functions, not module level) — hooks load
during __init__ mid-import, and module-level UI imports will crash.
"""
import os

_PLUGIN_DIR = os.path.dirname(os.path.abspath(__file__))

FONT_SIZE = 56
BOTTOM_MARGIN = 30

_sl_sub = None
_cached_way_ref = ''
_cached_road_name = ''


def _is_enabled():
  try:
    with open(os.path.join(_PLUGIN_DIR, 'data', 'RoadInfoOverlay')) as f:
      return f.read().strip() != '0'
  except (FileNotFoundError, OSError):
    return False  # default off


def _read_road_info():
  global _sl_sub, _cached_way_ref, _cached_road_name
  _socket_path = '/tmp/plugin_bus/speedLimitState'
  if _sl_sub is not None and not os.path.exists(_socket_path):
    try:
      _sl_sub.close()
    except Exception:
      pass
    _sl_sub = None
  try:
    if _sl_sub is None and os.path.exists(_socket_path):
      from openpilot.selfdrive.plugins.plugin_bus import PluginSub
      _sl_sub = PluginSub(['speedLimitState'])
    if _sl_sub is not None:
      msg = _sl_sub.drain('speedLimitState')
      if msg is not None:
        _, data = msg
        _cached_way_ref = data.get('wayRef', '')
        _cached_road_name = data.get('roadName', '')
  except Exception:
    pass
  return _cached_way_ref, _cached_road_name


def on_render_overlay(default, content_rect):
  if not _is_enabled():
    return None

  way_ref, road_name = _read_road_info()

  if not way_ref:
    return None

  # Lazy imports — must not be at module level
  import pyray as rl
  from openpilot.system.ui.lib.application import FontWeight
  from fonts import get_font, measure

  text = way_ref
  font = get_font(FontWeight.SEMI_BOLD)
  if font is None:
    return None
  text_size = measure(font, text, FONT_SIZE)
  pad = 10

  # Center horizontally, bottom of content rect
  cx = content_rect.x + content_rect.width / 2
  x = cx - text_size.x / 2
  y = content_rect.y + content_rect.height - BOTTOM_MARGIN - text_size.y

  # Semi-transparent background
  bg_rect = rl.Rectangle(x - pad, y - pad / 2, text_size.x + pad * 2, text_size.y + pad)
  rl.draw_rectangle_rounded(bg_rect, 0.3, 10, rl.Color(0, 0, 0, 128))

  color = rl.Color(100, 200, 255, 220)
  rl.draw_text_ex(font, text, rl.Vector2(x, y), FONT_SIZE, 0, color)

  return None
