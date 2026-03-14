"""Route map widget for the home screen left column.

Renders the last drive's GPS trace on a dark tile map, filling the full area.
Shares the RouteMapRenderer instance from DriveStatsWidget.
"""
import pyray as rl
from openpilot.system.ui.widgets import Widget

BG_COLOR = rl.Color(30, 30, 30, 255)


class RouteMapWidget(Widget):
  def __init__(self, drive_stats):
    super().__init__()
    self._drive_stats = drive_stats

  def _render(self, rect):
    renderer = self._drive_stats.map_renderer
    if renderer and renderer._trace:
      renderer.render(rect)
    else:
      rl.draw_rectangle_rounded(rect, 0.025, 10, BG_COLOR)
