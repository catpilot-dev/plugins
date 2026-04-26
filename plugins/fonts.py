"""Shared font access for plugin overlays.

Handles the lazy-init + retry pattern that all overlay hooks need:
gui_app.font() returns None on early render frames before fonts are
loaded. This module caches fonts and retries until ready, so individual
plugins don't need to implement their own _ensure_init guards.

Usage:
    from fonts import get_font, measure
    font = get_font(FontWeight.BOLD)
    if font is None:
        return  # not ready yet
    size = measure(font, text, 48)
"""

_font_cache = {}    # FontWeight -> font or None
_measure = None


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


def measure(font, text, font_size):
  """Measure text using cached measure function. Returns Vector2."""
  global _measure
  if _measure is None:
    from openpilot.system.ui.lib.text_measure import measure_text_cached
    _measure = measure_text_cached
  return _measure(font, text, font_size)
