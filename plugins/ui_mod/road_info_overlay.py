"""Road info overlay — renders current road name and wayRef at bottom-center of HUD.

Shows the road identifier from speedlimitd's OSM tile query, useful for
verifying that the speed limit daemon is reading the correct road.

Format: "S20 外环高速", or just "外环高速" / "S20" when only one is known.

The whole label is drawn in the system's Noto Sans SC: the UI's own fonts carry
no Chinese (Inter) or only the characters of the UI translations (the shipped
Noto subset). Where that system font is missing the UI font draws what it can,
which is ASCII — the road number, and a name only if it is ASCII too.

ALL imports MUST be lazy (inside functions, not module level) — hooks load
during __init__ mid-import, and module-level UI imports will crash.
"""
import os
from collections import OrderedDict

_PLUGIN_DIR = os.path.dirname(os.path.abspath(__file__))

FONT_SIZE = 64
BOTTOM_MARGIN = 30
MAX_NAME_CHARS = 16   # longer road names are cut, to stay clear of the HUD's corners

SYSTEM_FONT = '/usr/share/fonts/NotoSansSC-Regular.otf'   # full Noto Sans SC, part of the AGNOS image
# raylib sizes a font by its line height (ascent + descent). The em is 0.676 of
# that in Noto Sans SC and 0.826 in Inter, so the system font is drawn larger to
# give the label the glyph size FONT_SIZE has in the UI font.
SYSTEM_FONT_SCALE = 1.2
ATLAS_SIZE = 96       # px the label's glyphs are rasterized at; drawn a little smaller, through mipmaps
MAX_LABEL_FONTS = 6   # recent labels whose glyphs stay loaded

_sl_sub = None
_cached_way_ref = ''
_cached_road_name = ''
_label_fonts = OrderedDict()   # label -> font holding its glyphs, or None if the system font lacks one
_system_font_ok = None         # None until the first label needs it


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


def _ui_font_has(text):
  """The UI fonts (Inter) are loaded with printable ASCII plus the UI's own strings."""
  return all(32 <= ord(c) < 127 for c in text)


def _label(way_ref, road_name):
  """The text to show: road number, then name."""
  name = road_name.strip()
  if len(name) > MAX_NAME_CHARS:
    name = name[:MAX_NAME_CHARS - 3] + '...' if _ui_font_has(name) else name[:MAX_NAME_CHARS - 1] + '…'
  return f'{way_ref} {name}'.strip()


def _label_font(rl, text):
  """A font holding just this label's glyphs, loaded from the system font. None if it can't draw it.

  One small atlas per label, the most recent few kept: a road change costs the
  rasterization of a handful of glyphs, once.
  """
  global _system_font_ok
  if text in _label_fonts:
    _label_fonts.move_to_end(text)
    return _label_fonts[text]
  if _system_font_ok is None:
    _system_font_ok = os.path.isfile(SYSTEM_FONT)
  if not _system_font_ok:
    return None

  codepoints = sorted({ord(c) for c in text})
  buf = rl.ffi.new("int[]", codepoints)
  font = rl.load_font_ex(SYSTEM_FONT, ATLAS_SIZE, rl.ffi.cast("int *", buf), len(codepoints))
  if font.texture.id == 0:  # the file could not be loaded at all
    _system_font_ok = False
    return None
  if font.glyphCount < len(codepoints):  # raylib drops glyphs the file lacks: no label with holes in it
    rl.unload_font(font)
    font = None
  else:
    rl.gen_texture_mipmaps(font.texture)
    rl.set_texture_filter(font.texture, rl.TextureFilter.TEXTURE_FILTER_TRILINEAR)

  _label_fonts[text] = font
  while len(_label_fonts) > MAX_LABEL_FONTS:
    _, old = _label_fonts.popitem(last=False)
    if old is not None:
      rl.unload_font(old)
  return font


def on_render_overlay(default, content_rect):
  if not _is_enabled():
    return None

  way_ref, road_name = _read_road_info()
  text = _label(way_ref, road_name)
  if not text:
    return None

  # Lazy imports — must not be at module level
  import pyray as rl
  from openpilot.system.ui.lib.application import FontWeight, FONT_SCALE
  from fonts import get_font, measure

  font = _label_font(rl, text)
  if font is not None:
    # Not through gui_app's text helpers: besides scaling the size by FONT_SCALE,
    # they swap in the UI's own fallback font when the UI language is Chinese.
    size = FONT_SIZE * FONT_SCALE * SYSTEM_FONT_SCALE
    text_size = rl.measure_text_ex(font, text, size, 0)
    draw = getattr(rl, '_orig_draw_text_ex', rl.draw_text_ex)
  else:
    if not _ui_font_has(text):
      text = way_ref if _ui_font_has(way_ref) else ''
    font = get_font(FontWeight.SEMI_BOLD)
    if not text or font is None:
      return None
    size = FONT_SIZE
    text_size = measure(font, text, size)
    draw = rl.draw_text_ex
  pad = 10

  # Center horizontally, bottom of content rect
  cx = content_rect.x + content_rect.width / 2
  x = cx - text_size.x / 2
  y = content_rect.y + content_rect.height - BOTTOM_MARGIN - text_size.y

  # Semi-transparent background
  bg_rect = rl.Rectangle(x - pad, y - pad / 2, text_size.x + pad * 2, text_size.y + pad)
  rl.draw_rectangle_rounded(bg_rect, 0.3, 10, rl.Color(0, 0, 0, 128))

  color = rl.Color(100, 200, 255, 220)
  draw(font, text, rl.Vector2(x, y), size, 0, color)

  return None
