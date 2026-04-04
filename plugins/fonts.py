"""Shared font access for plugin overlays.

Handles the lazy-init + retry pattern that all overlay hooks need:
gui_app.font() returns None on early render frames before fonts are
loaded. This module caches fonts and retries until ready, so individual
plugins don't need to implement their own _ensure_init guards.

Usage:
    from fonts import get_font, get_cjk_font, measure
    font = get_font(FontWeight.BOLD)
    if font is None:
        return  # not ready yet
    size = measure(font, text, 48)
"""
import os

_font_cache = {}    # FontWeight -> font or None
_cjk_font = None
_cjk_font_attempted = False
_measure = None

# CJK font config
_CJK_FONT_PATHS = [
  '/data/openpilot/selfdrive/assets/fonts/unifont.otf',
]
_CJK_FONT_SIZE = 56  # default size for atlas generation


def get_font(weight):
  """Get a system font by FontWeight, or None if not ready yet.

  Retries on each call until the font is available. Once loaded,
  returns the cached font instantly.
  """
  cached = _font_cache.get(weight)
  if cached is not None:
    return cached

  from openpilot.system.ui.lib.application import gui_app
  font = gui_app.font(weight)
  if font is not None:
    _font_cache[weight] = font
  return font


def get_cjk_font(size=None):
  """Get CJK font (unifont.otf with ASCII + CJK Unified Ideographs).

  Loaded once on first successful call. Returns None if font file
  not found. The size parameter sets the atlas resolution (default 56).
  """
  global _cjk_font, _cjk_font_attempted
  if _cjk_font is not None:
    return _cjk_font
  if _cjk_font_attempted:
    return None

  _cjk_font_attempted = True
  font_size = size or _CJK_FONT_SIZE

  font_path = None
  for p in _CJK_FONT_PATHS:
    if os.path.exists(p):
      font_path = p
      break
  if font_path is None:
    return None

  import pyray as rl
  # ASCII (32-126) + CJK Unified Ideographs (0x4E00-0x9FFF)
  codepoints = list(range(32, 127)) + list(range(0x4E00, 0xA000))
  cp_buffer = rl.ffi.new("int[]", codepoints)
  cp_ptr = rl.ffi.cast("int *", cp_buffer)
  _cjk_font = rl.load_font_ex(font_path, font_size, cp_ptr, len(codepoints))
  return _cjk_font


def measure(font, text, font_size):
  """Measure text using cached measure function. Returns Vector2."""
  global _measure
  if _measure is None:
    from openpilot.system.ui.lib.text_measure import measure_text_cached
    _measure = measure_text_cached
  return _measure(font, text, font_size)
