"""Static route map renderer using CartoDB dark tiles and raylib.

Downloads tiles to disk in a background thread, loads them as raylib textures
on the main thread, and draws a GPS trace polyline on top.

Map is centered on the route endpoint at a zoom level that fills the render
rect completely — no black borders.
"""
import math
import os
import random
import threading
import urllib.request

import socket

import pyray as rl

from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached

TILE_CACHE_DIR = '/data/plugins-runtime/map_tiles'
TILE_SIZE = 256
CARTODB_URL = 'https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}@2x.png'
CARTODB_SUBDOMAINS = 'abcd'
USER_AGENT = 'catpilot/1.0'

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

# @2x tiles are 512px
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
# RouteMapRenderer
# ============================================================

class RouteMapRenderer:
  def __init__(self):
    self._textures = {}   # (z, x, y) -> Texture2D
    self._tile_keys = []  # tiles to load
    self._downloading = False
    self._download_done = False
    self._zoom = 0
    self._tx0 = self._tx1 = self._ty0 = self._ty1 = 0
    self._center_tx = 0.0  # fractional tile x of center point
    self._center_ty = 0.0  # fractional tile y of center point
    self._trace = []
    self._rect_w = 0
    self._rect_h = 0

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

    self._downloading = True
    self._download_done = False
    threading.Thread(target=self._download_tiles, daemon=True).start()

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
    url_text = f"View route details at {self._get_device_ip()}:8082"
    font = gui_app.font(FontWeight.NORMAL)
    text_size = measure_text_cached(font, url_text, URL_FONT_SIZE)
    tx = bar_rect.x + (bar_rect.width - text_size.x) / 2
    ty = bar_rect.y + (bar_rect.height - text_size.y) / 2
    rl.draw_text_ex(font, url_text, rl.Vector2(tx, ty), URL_FONT_SIZE, 0, URL_COLOR)

    rl.end_scissor_mode()

    # Rounded border on top to mask square corners
    rl.draw_rectangle_rounded_lines_ex(rect, CORNER_ROUNDNESS, CORNER_SEGMENTS, CORNER_BORDER, BG_COLOR)

  @staticmethod
  def _get_device_ip():
    try:
      s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      s.connect(("8.8.8.8", 80))
      ip = s.getsockname()[0]
      s.close()
      return ip
    except Exception:
      return "<device_ip>"

  def _to_screen(self, point, ox, oy):
    x, y = _lat_lng_to_tile_xy(point[0], point[1], self._zoom)
    sx = ox + x * TILE_PX
    sy = oy + y * TILE_PX
    return rl.Vector2(sx, sy)

  def _tile_path(self, z, x, y):
    return os.path.join(TILE_CACHE_DIR, 'cartodb', str(z), str(x), f'{y}.png')

  def _download_tiles(self):
    # Wipe old tiles — we only display the last drive
    import shutil
    if os.path.isdir(TILE_CACHE_DIR):
      shutil.rmtree(TILE_CACHE_DIR, ignore_errors=True)
    os.makedirs(TILE_CACHE_DIR, exist_ok=True)
    for z, x, y in self._tile_keys:
      path = self._tile_path(z, x, y)
      if os.path.exists(path):
        continue
      try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        s = random.choice(CARTODB_SUBDOMAINS)
        url = CARTODB_URL.format(s=s, z=z, x=x, y=y)
        req = urllib.request.Request(url, headers={'User-Agent': USER_AGENT})
        with urllib.request.urlopen(req, timeout=10) as resp:
          data = resp.read()
        tmp = path + '.tmp'
        with open(tmp, 'wb') as f:
          f.write(data)
        os.replace(tmp, path)
      except Exception:
        pass
    self._download_done = True
    self._downloading = False

  def _load_pending(self):
    for key in self._tile_keys:
      if key in self._textures:
        continue
      path = self._tile_path(*key)
      if os.path.exists(path):
        try:
          tex = rl.load_texture(path)
          if rl.is_texture_valid(tex):
            self._textures[key] = tex
        except Exception:
          pass

  def cleanup(self):
    for tex in self._textures.values():
      if rl.is_texture_valid(tex):
        rl.unload_texture(tex)
    self._textures.clear()
    self._trace = []
    self._tile_keys = []
