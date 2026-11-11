"""Static route map renderer using OpenStreetMap tiles and raylib.

Downloads tiles in a background thread, darkens them there with the same CSS
filter COD's map applies to light basemaps, and keeps them on disk; the main
thread loads them as raylib textures and draws a GPS trace polyline on top.
(CARTO's dark basemap, used before, now answers keyless requests with an
"API KEY REQUIRED" tile.)

Map is centered on the route endpoint at a zoom level that fills the render
rect completely — no black borders.
"""
import io
import math
import os
import threading
import time
import urllib.request

import numpy as np

from config import PLUGINS_RUNTIME_DIR

import pyray as rl

from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached

TILE_CACHE_DIR = os.path.join(PLUGINS_RUNTIME_DIR, 'map_tiles')
TILE_SIZE = 256
TILE_URL = 'https://tile.openstreetmap.org/{z}/{x}/{y}.png'
TILE_SOURCE = 'osm-dark'   # cache subdirectory: OSM tiles after darken()
ATTRIBUTION = '(c) OpenStreetMap contributors'   # the UI font has no "©"
USER_AGENT = 'catpilot/1.0'
# OSM's tile policy asks clients to cache tiles (7 days at least); repeat drives
# reuse them, and tiles unused for this long are deleted.
TILE_MAX_AGE_DAYS = 30
# Seconds between download passes while tiles are missing. At boot the network is
# down and the clock unsynced (TLS fails), so the first pass usually gets nothing.
RETRY_DELAYS_S = (10, 30, 60, 120, 300)   # the last one repeats

TRACE_COLOR = rl.Color(70, 91, 234, 255)
TRACE_WIDTH = 8.0
START_COLOR = rl.Color(76, 175, 80, 255)
END_COLOR = rl.Color(226, 44, 44, 255)
MARKER_RADIUS = 12
BG_COLOR = rl.Color(30, 30, 30, 255)
CORNER_ROUNDNESS = 0.08
CORNER_SEGMENTS = 10
CORNER_BORDER = 4
URL_BAR_HEIGHT = 48
URL_BAR_COLOR = rl.Color(0, 0, 0, 160)
URL_FONT_SIZE = 36
URL_COLOR = rl.Color(180, 180, 180, 255)
ATTRIBUTION_FONT_SIZE = 24
ATTRIBUTION_MARGIN = 12

# OSM tiles are 256 px, drawn at 2x (as a browser at devicePixelRatio 2 does)
# so the map's labels are legible on the C3 screen.
TILE_PX = 512
# Zoom limits
MAX_ZOOM = 16
MIN_ZOOM = 10
# Padding factor: trace fills this fraction of the view (rest is margin)
FIT_PADDING = 0.85


# ============================================================
# Web Mercator math
# ============================================================

def _lat_lng_to_tile_xy(lat, lng, zoom):
  n = 2 ** zoom
  x = (lng + 180.0) / 360.0 * n
  y = (1.0 - math.log(math.tan(math.radians(lat)) + 1.0 / math.cos(math.radians(lat))) / math.pi) / 2.0 * n
  return x, y


def _tiles_for_rect(center_lat, center_lng, zoom, rect_w, rect_h):
  """Compute tile range that covers rect_w x rect_h pixels centered on a GPS point."""
  cx, cy = _lat_lng_to_tile_xy(center_lat, center_lng, zoom)

  # How many tiles span the rect in each direction (in tile-coordinate units)
  half_w = (rect_w / TILE_PX) / 2
  half_h = (rect_h / TILE_PX) / 2

  tx0 = int(math.floor(cx - half_w))
  tx1 = int(math.ceil(cx + half_w))
  ty0 = int(math.floor(cy - half_h))
  ty1 = int(math.ceil(cy + half_h))

  return tx0, tx1, ty0, ty1, cx, cy


# ============================================================
# Darkening: COD's .map-tiles-dark CSS filter
#   invert(1) hue-rotate(180deg) brightness(0.95) contrast(1.1) saturate(0.15)
# ============================================================

def _hue_rotate_matrix(deg):
  c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
  return np.array([
    [0.213 + c * 0.787 - s * 0.213, 0.715 - c * 0.715 - s * 0.715, 0.072 - c * 0.072 + s * 0.928],
    [0.213 - c * 0.213 + s * 0.143, 0.715 + c * 0.285 + s * 0.140, 0.072 - c * 0.072 - s * 0.283],
    [0.213 - c * 0.213 - s * 0.787, 0.715 - c * 0.715 + s * 0.715, 0.072 + c * 0.928 + s * 0.072],
  ])


def _saturate_matrix(v):
  return np.array([
    [0.213 + 0.787 * v, 0.715 - 0.715 * v, 0.072 - 0.072 * v],
    [0.213 - 0.213 * v, 0.715 + 0.285 * v, 0.072 - 0.072 * v],
    [0.213 - 0.213 * v, 0.715 - 0.715 * v, 0.072 + 0.928 * v],
  ])


_HUE_ROTATE_180 = _hue_rotate_matrix(180)
_SATURATE_015 = _saturate_matrix(0.15)


def darken(rgb):
  """HxWx3 uint8 sRGB -> the same, filtered step by step and clamped as a browser does."""
  c = rgb.astype(np.float32) / 255.0
  c = 1.0 - c                                       # invert(1)
  c = np.clip(c @ _HUE_ROTATE_180.T, 0.0, 1.0)      # hue-rotate(180deg)
  c = np.clip(c * 0.95, 0.0, 1.0)                   # brightness(0.95)
  c = np.clip((c - 0.5) * 1.1 + 0.5, 0.0, 1.0)      # contrast(1.1)
  c = np.clip(c @ _SATURATE_015.T, 0.0, 1.0)        # saturate(0.15)
  return np.rint(c * 255.0).astype(np.uint8)


def _darken_png(data):
  """PNG bytes of a light tile -> PNG bytes of its dark version."""
  from PIL import Image
  rgb = np.asarray(Image.open(io.BytesIO(data)).convert('RGB'))
  out = io.BytesIO()
  Image.fromarray(darken(rgb), 'RGB').save(out, 'PNG')
  return out.getvalue()


# ============================================================
# RouteMapRenderer
# ============================================================

class RouteMapRenderer:
  def __init__(self):
    self._textures = {}   # (z, x, y) -> Texture2D
    self._tile_keys = []  # tiles to load
    self._zoom = 0
    self._tx0 = self._tx1 = self._ty0 = self._ty1 = 0
    self._center_tx = 0.0  # fractional tile x of center point
    self._center_ty = 0.0  # fractional tile y of center point
    self._trace = []
    self._rect_w = 0
    self._rect_h = 0
    self._gen = 0  # bumped by cleanup(): a download thread of an older trace stops

  def load_trace(self, trace, rect_w=1500, rect_h=900):
    """Set GPS trace, fit entire route in view, and start downloading tiles."""
    self.cleanup()
    if not trace or len(trace) < 2:
      return

    self._trace = trace
    self._rect_w = rect_w
    self._rect_h = rect_h

    # Compute bounding box of the entire trace
    lats = [p[0] for p in trace]
    lngs = [p[1] for p in trace]
    min_lat, max_lat = min(lats), max(lats)
    min_lng, max_lng = min(lngs), max(lngs)

    # Center on the midpoint of the bounding box
    center_lat = (min_lat + max_lat) / 2
    center_lng = (min_lng + max_lng) / 2

    # Find the best zoom level that fits the entire trace
    self._zoom = MAX_ZOOM
    for z in range(MAX_ZOOM, MIN_ZOOM - 1, -1):
      # Convert bounding box corners to tile coordinates at this zoom
      x0, y0 = _lat_lng_to_tile_xy(max_lat, min_lng, z)  # top-left
      x1, y1 = _lat_lng_to_tile_xy(min_lat, max_lng, z)  # bottom-right
      # Trace extent in pixels at this zoom
      trace_w = abs(x1 - x0) * TILE_PX
      trace_h = abs(y1 - y0) * TILE_PX
      # Check if trace fits within the padded view
      if trace_w <= rect_w * FIT_PADDING and trace_h <= rect_h * FIT_PADDING:
        self._zoom = z
        break

    tx0, tx1, ty0, ty1, cx, cy = _tiles_for_rect(
      center_lat, center_lng, self._zoom, rect_w, rect_h)

    self._tx0, self._tx1, self._ty0, self._ty1 = tx0, tx1, ty0, ty1
    self._center_tx, self._center_ty = cx, cy

    self._tile_keys = [
      (self._zoom, x, y)
      for x in range(self._tx0, self._tx1 + 1)
      for y in range(self._ty0, self._ty1 + 1)
    ]

    threading.Thread(target=self._download_tiles, args=(self._gen,), daemon=True).start()

  def render(self, rect):
    """Render map with trace into the given rect."""
    # Dark background with rounded corners
    rl.draw_rectangle_rounded(rect, CORNER_ROUNDNESS, CORNER_SEGMENTS, BG_COLOR)

    if not self._trace:
      return

    # Load any newly downloaded tiles as textures (must be on main thread)
    self._load_pending()

    # Scale: 1 tile pixel = 1 screen pixel (1:1), tiles fill the rect edge-to-edge.
    # Offset so the center GPS point maps to the center of rect.
    ox = rect.x + rect.width / 2 - self._center_tx * TILE_PX
    oy = rect.y + rect.height / 2 - self._center_ty * TILE_PX

    # Clip to rect
    rl.begin_scissor_mode(int(rect.x), int(rect.y), int(rect.width), int(rect.height))

    # Draw tiles at 1:1 pixel scale
    for key in self._tile_keys:
      tex = self._textures.get(key)
      if tex and rl.is_texture_valid(tex):
        z, tx, ty = key
        dx = ox + tx * TILE_PX
        dy = oy + ty * TILE_PX
        src = rl.Rectangle(0, 0, tex.width, tex.height)
        dst = rl.Rectangle(dx, dy, TILE_PX, TILE_PX)
        rl.draw_texture_pro(tex, src, dst, rl.Vector2(0, 0), 0, rl.WHITE)

    # Draw trace polyline (only segments visible in rect)
    for i in range(len(self._trace) - 1):
      p0 = self._to_screen(self._trace[i], ox, oy)
      p1 = self._to_screen(self._trace[i + 1], ox, oy)
      rl.draw_line_ex(p0, p1, TRACE_WIDTH, TRACE_COLOR)

    # Start and end markers
    if len(self._trace) >= 2:
      start = self._to_screen(self._trace[0], ox, oy)
      rl.draw_circle(int(start.x), int(start.y), MARKER_RADIUS, START_COLOR)
      end = self._to_screen(self._trace[-1], ox, oy)
      rl.draw_circle(int(end.x), int(end.y), MARKER_RADIUS, END_COLOR)

    # URL overlay at bottom (no background — dark tiles provide contrast)
    bar_rect = rl.Rectangle(rect.x, rect.y + rect.height - URL_BAR_HEIGHT, rect.width, URL_BAR_HEIGHT)
    url_text = "Check route details at catpilot.local"
    font = gui_app.font(FontWeight.NORMAL)
    text_size = measure_text_cached(font, url_text, URL_FONT_SIZE)
    tx = bar_rect.x + (bar_rect.width - text_size.x) / 2
    ty = bar_rect.y + (bar_rect.height - text_size.y) / 2
    rl.draw_text_ex(font, url_text, rl.Vector2(tx, ty), URL_FONT_SIZE, 0, URL_COLOR)

    # Map data credit, required by OSM, top-right clear of the trace caption
    attr_size = measure_text_cached(font, ATTRIBUTION, ATTRIBUTION_FONT_SIZE)
    ax = rect.x + rect.width - attr_size.x - ATTRIBUTION_MARGIN - CORNER_BORDER
    ay = rect.y + ATTRIBUTION_MARGIN + CORNER_BORDER
    rl.draw_text_ex(font, ATTRIBUTION, rl.Vector2(ax, ay), ATTRIBUTION_FONT_SIZE, 0, URL_COLOR)

    rl.end_scissor_mode()

    # Rounded border on top to mask square corners
    rl.draw_rectangle_rounded_lines_ex(rect, CORNER_ROUNDNESS, CORNER_SEGMENTS, CORNER_BORDER, BG_COLOR)


  def _to_screen(self, point, ox, oy):
    x, y = _lat_lng_to_tile_xy(point[0], point[1], self._zoom)
    sx = ox + x * TILE_PX
    sy = oy + y * TILE_PX
    return rl.Vector2(sx, sy)

  def _tile_path(self, z, x, y):
    return os.path.join(TILE_CACHE_DIR, TILE_SOURCE, str(z), str(x), f'{y}.png')

  @staticmethod
  def _prune_cache():
    """Delete tiles unused for TILE_MAX_AGE_DAYS, then directories left empty (a retired source's too)."""
    cutoff = time.time() - TILE_MAX_AGE_DAYS * 86400
    for root, _dirs, files in os.walk(TILE_CACHE_DIR, topdown=False):
      for name in files:
        path = os.path.join(root, name)
        try:
          if os.path.getmtime(path) < cutoff:
            os.remove(path)
        except OSError:
          pass
      if root != TILE_CACHE_DIR:
        try:
          os.rmdir(root)  # only succeeds when empty
        except OSError:
          pass

  def _download_tiles(self, gen=None):
    """Fetch the trace's tiles, passes repeated with backoff until all are there or the trace changes."""
    gen = self._gen if gen is None else gen
    keys = self._tile_keys
    self._prune_cache()
    attempt = 0
    while self._fetch_missing(keys, gen) and gen == self._gen:
      time.sleep(RETRY_DELAYS_S[min(attempt, len(RETRY_DELAYS_S) - 1)])
      attempt += 1

  def _fetch_missing(self, keys, gen):
    """One download pass. True if some tile is still missing."""
    missing = False
    for z, x, y in keys:
      if gen != self._gen:
        return True
      path = self._tile_path(z, x, y)
      if os.path.exists(path):
        try:
          if time.time() > os.path.getmtime(path):  # never back in time, when the clock is unsynced at boot
            os.utime(path)  # in use: keep it past the next prune
        except OSError:
          pass
        continue
      try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        url = TILE_URL.format(z=z, x=x, y=y)
        req = urllib.request.Request(url, headers={'User-Agent': USER_AGENT})
        with urllib.request.urlopen(req, timeout=10) as resp:
          data = _darken_png(resp.read())
        tmp = path + '.tmp'
        with open(tmp, 'wb') as f:
          f.write(data)
        os.replace(tmp, path)
      except Exception:
        missing = True
    return missing

  def _load_pending(self):
    for key in self._tile_keys:
      if key in self._textures:
        continue
      path = self._tile_path(*key)
      if os.path.exists(path):
        try:
          tex = rl.load_texture(path)
          if rl.is_texture_valid(tex):
            rl.set_texture_filter(tex, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)  # drawn at 2x
            self._textures[key] = tex
        except Exception:
          pass

  def cleanup(self):
    self._gen += 1
    for tex in self._textures.values():
      if rl.is_texture_valid(tex):
        rl.unload_texture(tex)
    self._textures.clear()
    self._trace = []
    self._tile_keys = []
