"""Road info overlay — renders current road name and wayRef at bottom-center of HUD.

Shows the road identifier from speedlimitd's OSM tile query, useful for
verifying that the speed limit daemon is reading the correct road.

Format: "S20 外环高速" or just "外环高速" when no wayRef is available.
"""
import os
import pyray as rl

_PLUGIN_DIR = os.path.dirname(os.path.abspath(__file__))

FONT_SIZE = 56
BOTTOM_MARGIN = 30

COLOR_REF = rl.Color(100, 200, 255, 220)    # light blue for wayRef
COLOR_NAME = rl.Color(200, 200, 200, 200)   # light gray for road name

_font = None
_unifont = None
_measure = None
_sl_sub = None
_cached_way_ref = ''
_cached_road_name = ''


def _has_cjk(text):
  return any('\u4e00' <= c <= '\u9fff' or '\u3400' <= c <= '\u4dbf' for c in text)


def _ensure_init():
  global _font, _unifont, _measure
  if _font is None:
    from openpilot.system.ui.lib.application import gui_app, FontWeight
    from openpilot.system.ui.lib.text_measure import measure_text_cached
    _font = gui_app.font(FontWeight.SEMI_BOLD)
    _unifont = gui_app.font(FontWeight.UNIFONT)
    _measure = measure_text_cached


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
  _ensure_init()

  if not _is_enabled():
    return None

  way_ref, road_name = _read_road_info()

  if not way_ref and not road_name:
    return None

  # Build display text: "S20 外环高速" or just road name
  if way_ref and road_name and way_ref != road_name:
    text = f"{way_ref}  {road_name}"
  elif way_ref:
    text = way_ref
  else:
    text = road_name

  # Use unifont for CJK characters (road names are always Chinese regardless of UI language)
  font = _unifont if _has_cjk(text) else _font
  text_size = _measure(font, text, FONT_SIZE)
  pad = 10

  # Center horizontally, bottom of content rect
  cx = content_rect.x + content_rect.width / 2
  x = cx - text_size.x / 2
  y = content_rect.y + content_rect.height - BOTTOM_MARGIN - text_size.y

  # Semi-transparent background
  bg_rect = rl.Rectangle(x - pad, y - pad / 2, text_size.x + pad * 2, text_size.y + pad)
  rl.draw_rectangle_rounded(bg_rect, 0.3, 10, rl.Color(0, 0, 0, 128))

  # Draw text — use ref color if wayRef present, otherwise name color
  color = COLOR_REF if way_ref else COLOR_NAME
  rl.draw_text_ex(font, text, rl.Vector2(x, y), FONT_SIZE, 0, color)

  return None
