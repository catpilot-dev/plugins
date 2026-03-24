"""Driving panel for Settings UI — runtime driving feature tuning.

Items only appear when the corresponding plugin is enabled. Toggling here
controls runtime behaviour (e.g. skip curvature correction), not the plugin
lifecycle — that's managed in the Plugins panel.
"""
import json
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import PLUGINS_RUNTIME_DIR, read_plugin_param, write_plugin_param, write_param
from cereal import log
from openpilot.common.params import Params
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.list_view import toggle_item, multiple_button_item
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.selfdrive.ui.ui_state import ui_state

PLUGINS_DIR = PLUGINS_RUNTIME_DIR
PERSONALITY_TO_INT = log.LongitudinalPersonality.schema.enumerants

LAT_ACCEL_VALUES = [1.5, 2.0, 2.5, 3.0]   # m/s² — indexed by pill selection


def _plugin_enabled(plugin_id):
  return (os.path.isdir(os.path.join(PLUGINS_DIR, plugin_id)) and
          not os.path.exists(os.path.join(PLUGINS_DIR, plugin_id, '.disabled')))




def _sync_mapd_settings():
  """Regenerate MapdSettings JSON for mapd Go binary (/data/params/d/)."""
  enabled = read_plugin_param('speedlimitd', 'MapdSpeedLimitControlEnabled') == '1'

  try:
    lat_idx = int(read_plugin_param('speedlimitd', 'MapdCurveTargetLatAccel') or '0')
  except ValueError:
    lat_idx = 0
  lat_accel = LAT_ACCEL_VALUES[lat_idx] if 0 <= lat_idx < len(LAT_ACCEL_VALUES) else 1.5

  settings = {
    'speed_limit_control_enabled': enabled,
    'map_curve_speed_control_enabled': True,   # always on — placeholder for future OSM curve data
    'vision_curve_speed_control_enabled': True, # always on — toggle removed by design
    'speed_limit_offset': 0.0,   # no offset — planner_hook applies tiered offset
    'map_curve_target_lat_a': lat_accel,
    'vision_curve_target_lat_a': lat_accel,
  }
  write_param('MapdSettings', json.dumps(settings))


class DrivingLayout(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()
    self._scroller = None
    self._needs_rebuild = True

  def _build_scroller(self):
    items = []

    # --- Driving Personality (always present) ---
    self._personality = multiple_button_item(
      lambda: tr("Personality"),
      lambda: tr("Standard is recommended. In aggressive mode, openpilot will follow lead cars closer and be more aggressive with the gas and brake. "
                 "In relaxed mode openpilot will stay further away from lead cars. On supported cars, you can cycle through these personalities with "
                 "your steering wheel distance button."),
      buttons=[lambda: tr("Aggressive"), lambda: tr("Standard"), lambda: tr("Relaxed")],
      button_width=255,
      callback=self._set_personality,
      selected_index=self._params.get("LongitudinalPersonality", return_default=True),
    )
    items.append(self._personality)

    # --- Lane Centering in Turns (if plugin enabled) ---
    if _plugin_enabled('lane_centering'):
      current = read_plugin_param('lane_centering', 'LaneCenteringEnabled') != '0'
      self._lane_centering = toggle_item(
        "Lane Centering in Turns",
        "Adjusts steering curvature in turns to keep the car centered between lane lines.",
        current,
        callback=self._on_lane_centering,
      )
      items.append(self._lane_centering)

    # --- Speedlimitd items (if plugin enabled) ---
    if _plugin_enabled('speedlimitd'):
      # Conditional Speed Control
      map_speed_enabled = read_plugin_param('speedlimitd', 'MapdSpeedLimitControlEnabled') == '1'
      self._map_speed = toggle_item(
        "Conditional Speed Control",
        "Limit cruise speed to detected speed limit when confirmed.",
        map_speed_enabled,
        callback=self._on_map_speed,
        enabled=lambda: _plugin_enabled('mapd'),
      )
      items.append(self._map_speed)

      # Curve Comfort (depends on Conditional Speed Control)
      current_curve = read_plugin_param('speedlimitd', 'MapdCurveTargetLatAccel')
      try:
        curve_idx = int(current_curve) if current_curve else 0
      except ValueError:
        curve_idx = 0
      self._curve_comfort = multiple_button_item(
        "Curve Comfort",
        "Target lateral acceleration in curves (m/s²).",
        buttons=["1.5", "2.0", "2.5", "3.0"],
        selected_index=curve_idx,
        button_width=150,
        callback=self._on_curve_comfort,
      )
      self._curve_comfort.set_visible(lambda: read_plugin_param('speedlimitd', 'MapdSpeedLimitControlEnabled') == '1')
      items.append(self._curve_comfort)

    self._scroller = Scroller(items, line_separator=True, spacing=0)

  def _set_personality(self, button_index):
    self._params.put("LongitudinalPersonality", button_index)

  def _on_lane_centering(self, state):
    write_plugin_param('lane_centering', 'LaneCenteringEnabled', '1' if state else '0')

  def _on_map_speed(self, state):
    write_plugin_param('speedlimitd', 'MapdSpeedLimitControlEnabled', '1' if state else '0')
    _sync_mapd_settings()

  def _on_curve_comfort(self, idx):
    write_plugin_param('speedlimitd', 'MapdCurveTargetLatAccel', str(idx))
    _sync_mapd_settings()

  def _update_state(self):
    if not hasattr(self, '_personality'):
      return

    # Sync personality from car state (e.g. steering wheel button change)
    if ui_state.sm.updated["selfdriveState"]:
      personality = PERSONALITY_TO_INT[ui_state.sm["selfdriveState"].personality]
      if personality != ui_state.personality and ui_state.started:
        self._personality.action_item.set_selected_button(personality)
      ui_state.personality = personality

    # Disable personality when no longitudinal control (stock behaviour)
    if ui_state.CP is not None:
      self._personality.action_item.set_enabled(ui_state.has_longitudinal_control)

  def show_event(self):
    super().show_event()
    self._needs_rebuild = True

  def _render(self, rect):
    if self._needs_rebuild:
      self._build_scroller()
      self._needs_rebuild = False

    if self._scroller:
      self._scroller.render(rect)
