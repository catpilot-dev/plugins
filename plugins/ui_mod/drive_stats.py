"""Drive statistics widget — vehicle info + stats from COD + last drive summary."""

import json
import os
import threading
import urllib.request

import pyray as rl

from openpilot.common.params import Params
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets import Widget

COD_BASE = "http://localhost:8082"
EMBLEMS_DIR = '/data/plugins/logos/emblems'
EMBLEM_SIZE = 180

BG_COLOR = rl.Color(51, 51, 51, 255)
LABEL_COLOR = rl.Color(128, 128, 128, 255)
VALUE_COLOR = rl.WHITE


class DriveStatsWidget(Widget):
  def __init__(self):
    super().__init__()
    dongle_id = Params().get("DongleId")
    self._dongle_id = (dongle_id if isinstance(dongle_id, str) else dongle_id.decode("utf-8")) if dongle_id else ""
    self._is_metric = Params().get_bool("IsMetric")
    self._stats = None
    self._fetching = False
    self._last_drive = None
    self._last_drive_mtime = None  # sentinel: force first check
    self.map_renderer = None  # public: shared with RouteMapWidget
    self._brand_texture = None
    self._fingerprint = ''
    self._load_vehicle_info()
    self.refresh()  # initial fetch on construction

  def _load_vehicle_info(self):
    try:
      from openpilot.selfdrive.ui.ui_state import ui_state
      CP = ui_state.CP
      if CP is not None:
        self._fingerprint = CP.carFingerprint
        brand = getattr(CP, 'brand', '')
        if brand:
          emblem_path = os.path.join(EMBLEMS_DIR, f'{brand}.png')
          if os.path.isfile(emblem_path):
            image = rl.load_image(emblem_path)
            rl.image_resize(image, EMBLEM_SIZE, EMBLEM_SIZE)
            self._brand_texture = rl.load_texture_from_image(image)
            rl.unload_image(image)
    except Exception:
      pass

  def show_event(self):
    super().show_event()
    self.refresh()

  def refresh(self):
    self._is_metric = Params().get_bool("IsMetric")
    self._maybe_reload()

  def _maybe_reload(self):
    """Reload last drive and stats only when the data file changes (offroad transition)."""
    from drive_tracker import LAST_DRIVE_FILE
    try:
      mtime = os.path.getmtime(LAST_DRIVE_FILE)
    except OSError:
      mtime = 0.0

    if mtime == self._last_drive_mtime:
      return

    self._last_drive_mtime = mtime
    try:
      from drive_tracker import get_last_drive
      drive = get_last_drive()
      if drive:
        self._last_drive = drive
        self._load_map_trace(drive)
      else:
        self._last_drive = None
        self._cleanup_map()
    except Exception:
      self._last_drive = None
      self._cleanup_map()

    # Stats also change after a drive — fetch from COD
    self._fetch_stats()

  def _load_map_trace(self, drive):
    trace = drive.get('trace', [])
    if not trace or len(trace) < 2:
      self._cleanup_map()
      return
    if self.map_renderer is None:
      from route_map import RouteMapRenderer
      self.map_renderer = RouteMapRenderer()
    self.map_renderer.load_trace(trace)

  def _cleanup_map(self):
    if self.map_renderer:
      self.map_renderer.cleanup()
      self.map_renderer = None

  def _fetch_stats(self):
    if self._fetching or not self._dongle_id:
      return
    self._fetching = True
    threading.Thread(target=self._do_fetch, daemon=True).start()

  def _do_fetch(self):
    try:
      url = f"{COD_BASE}/v1.1/devices/{self._dongle_id}/stats"
      req = urllib.request.Request(url, headers={"Accept": "application/json"})
      with urllib.request.urlopen(req, timeout=10) as resp:
        self._stats = json.loads(resp.read())
    except Exception:
      pass
    finally:
      self._fetching = False

  def _render(self, rect: rl.Rectangle):
    rl.draw_rectangle_rounded(rect, 0.025, 10, BG_COLOR)

    x = rect.x + 56
    w = rect.width - 112

    font_bold = gui_app.font(FontWeight.BOLD)
    font_normal = gui_app.font(FontWeight.NORMAL)

    # Vehicle info header — centered, takes upper portion
    if self._fingerprint:
      self._render_vehicle_header(x, rect.y + 32, w, rect, font_bold)

    # Stats blocks — bottom-aligned with padding
    # Each row: label(36) + gap(4) + value(68) + unit_gap(4) + unit(40) = ~152
    ROW_HEIGHT = 152
    PADDING = 40
    bottom = rect.y + rect.height - PADDING
    # Past 7 Days at bottom
    week_y = bottom - ROW_HEIGHT
    self._render_stats_row(x, week_y, w, "Past 7 Days", "week", font_bold, font_normal)
    # Last Drive above with same padding
    if self._last_drive:
      drive_y = week_y - PADDING - ROW_HEIGHT
      self._render_last_drive_row(x, drive_y, w, font_bold, font_normal)

  def _render_vehicle_header(self, x, y, w, rect, font_bold):
    """Render brand icon + model name, centered in upper portion."""
    # Center vertically in top 45% of widget
    zone_h = rect.height * 0.44
    center_y = y + zone_h / 2

    display_name = self._fingerprint.replace('_', ' ')

    if self._brand_texture is not None:
      # Icon above text, both centered
      icon_w = self._brand_texture.width
      icon_h = self._brand_texture.height
      text_size = measure_text_cached(font_bold, display_name, 52)
      total_h = icon_h + 16 + text_size.y
      icon_y = center_y - total_h / 2
      icon_x = x + (w - icon_w) / 2
      rl.draw_texture_pro(
        self._brand_texture,
        rl.Rectangle(0, 0, self._brand_texture.width, self._brand_texture.height),
        rl.Rectangle(icon_x, icon_y, icon_w, icon_h),
        rl.Vector2(0, 0), 0, rl.WHITE,
      )
      text_x = x + (w - text_size.x) / 2
      text_y = icon_y + icon_h + 16
      rl.draw_text_ex(font_bold, display_name, rl.Vector2(text_x, text_y), 52, 0, VALUE_COLOR)
    else:
      text_size = measure_text_cached(font_bold, display_name, 52)
      text_x = x + (w - text_size.x) / 2
      text_y = center_y - text_size.y / 2
      rl.draw_text_ex(font_bold, display_name, rl.Vector2(text_x, text_y), 52, 0, VALUE_COLOR)

  def _render_stats_row(self, x, y, w, label, stats_key, font_bold, font_normal) -> float:
    """Render a compact stats row with label. Returns y for next row."""
    rl.draw_text_ex(font_normal, tr(label), rl.Vector2(x, y), 36, 0, LABEL_COLOR)
    y += 40

    if self._stats is None:
      rl.draw_text_ex(font_normal, tr("loading..."), rl.Vector2(x, y), 36, 0, LABEL_COLOR)
      return y + 80

    data = self._stats.get(stats_key, {})
    routes = data.get("routes", 0)
    distance_mi = data.get("distance", 0)
    minutes = data.get("minutes", 0)
    if self._is_metric:
      distance = distance_mi * 1.60934
      dist_unit = tr("km")
    else:
      distance = distance_mi
      dist_unit = tr("mi")

    hours = minutes / 60
    stats_row = [
      (str(routes), tr("Drives")),
      (f"{distance:.0f}", dist_unit),
      (f"{hours:.0f}", tr("Hours")),
    ]

    self._draw_stat_cols(x, y, w, stats_row, font_bold, font_normal)
    return y + 110

  def _render_last_drive_row(self, x, y, w, font_bold, font_normal) -> float:
    """Render last drive stats as a compact row. Returns y for next row."""
    rl.draw_text_ex(font_normal, tr("Last Drive"), rl.Vector2(x, y), 36, 0, LABEL_COLOR)
    y += 40

    d = self._last_drive
    distance_m = d.get('distance_m', 0)
    duration_s = d.get('duration_s', 0)
    engaged_s = d.get('engaged_s', 0)

    if self._is_metric:
      dist_val = distance_m / 1000
      dist_unit = tr("km")
    else:
      dist_val = distance_m / 1609.344
      dist_unit = tr("mi")

    dur_min = duration_s / 60
    if dur_min >= 60:
      dur_text = f"{dur_min / 60:.1f}"
      dur_unit = tr("Hours")
    else:
      dur_text = f"{dur_min:.0f}"
      dur_unit = tr("Min")

    pct_text = f"{min(engaged_s / duration_s, 1.0) * 100:.0f}%" if duration_s > 0 else "—"

    stats_row = [
      (f"{dist_val:.1f}", dist_unit),
      (dur_text, dur_unit),
      (pct_text, tr("engaged")),
    ]

    self._draw_stat_cols(x, y, w, stats_row, font_bold, font_normal)
    return y + 110

  @staticmethod
  def _draw_stat_cols(x, y, w, cols, font_bold, font_normal):
    from openpilot.system.ui.lib.text_measure import measure_text_cached
    col_w = w / len(cols)
    for i, (value, unit) in enumerate(cols):
      col_center = x + (i + 0.5) * col_w
      val_size = measure_text_cached(font_bold, value, 68)
      rl.draw_text_ex(font_bold, value, rl.Vector2(col_center - val_size.x / 2, y), 68, 0, VALUE_COLOR)
      unit_size = measure_text_cached(font_normal, unit, 40)
      rl.draw_text_ex(font_normal, unit, rl.Vector2(col_center - unit_size.x / 2, y + 72), 40, 0, LABEL_COLOR)
