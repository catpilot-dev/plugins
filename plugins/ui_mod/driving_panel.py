"""Driving panel for Settings UI — driving behavior + vehicle-specific settings.

Combines:
- Stock driving personality (aggressive/standard/relaxed)
- Plugin-provided driving toggles (lane centering, speed limit sign)
- Vehicle-specific settings populated via 'ui.vehicle_settings' hook
  (e.g. BMW lane change behavior, steering tuning)

Vehicle heading (brand icon + fingerprint) shown when a car is detected.
"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import PLUGINS_RUNTIME_DIR, PLUGINS_REPO_DIR, read_plugin_param, write_plugin_param
from cereal import log
import pyray as rl
from openpilot.common.params import Params
from openpilot.selfdrive.plugins.hooks import hooks
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.list_view import toggle_item, multiple_button_item
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.lib.multilang import tr

PLUGINS_DIR = PLUGINS_RUNTIME_DIR
LOGOS_DIR = os.path.join(PLUGINS_REPO_DIR, 'logos', 'icons')
PERSONALITY_TO_INT = log.LongitudinalPersonality.schema.enumerants
HEADING_HEIGHT = 80
ICON_SIZE = 64


def _plugin_enabled(plugin_id):
  return (os.path.isdir(os.path.join(PLUGINS_DIR, plugin_id)) and
          not os.path.exists(os.path.join(PLUGINS_DIR, plugin_id, '.disabled')))


class DrivingLayout(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()
    self._scroller = None
    self._needs_rebuild = True
    self._brand_texture = None
    self._fingerprint = ''
    self._font = gui_app.font(FontWeight.MEDIUM)

  def _load_brand_icon(self, brand):
    icon_path = os.path.join(LOGOS_DIR, f'{brand}.png')
    if os.path.isfile(icon_path):
      image = rl.load_image(icon_path)
      rl.image_resize(image, ICON_SIZE, ICON_SIZE)
      texture = rl.load_texture_from_image(image)
      rl.unload_image(image)
      return texture
    return None

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

    # --- Vehicle-specific settings (populated by car plugins) ---
    CP = ui_state.CP
    if CP is not None:
      self._fingerprint = CP.carFingerprint
      brand = getattr(CP, 'brand', '')
      if brand and self._brand_texture is None:
        self._brand_texture = self._load_brand_icon(brand)
      items = hooks.run('ui.vehicle_settings', items, CP)

    self._scroller = Scroller(items, line_separator=True, spacing=0)

  def _set_personality(self, button_index):
    self._params.put("LongitudinalPersonality", button_index)

  def _on_lane_centering(self, state):
    write_plugin_param('lane_centering', 'LaneCenteringEnabled', '1' if state else '0')

  def _update_state(self):
    if not hasattr(self, '_personality'):
      return

    if ui_state.sm.updated["selfdriveState"]:
      personality = PERSONALITY_TO_INT[ui_state.sm["selfdriveState"].personality]
      if personality != ui_state.personality and ui_state.started:
        self._personality.action_item.set_selected_button(personality)
      ui_state.personality = personality

    if ui_state.CP is not None:
      self._personality.action_item.set_enabled(ui_state.has_longitudinal_control)

  def show_event(self):
    super().show_event()
    self._needs_rebuild = True

  def _render(self, rect):
    if self._needs_rebuild:
      self._build_scroller()
      self._needs_rebuild = False

    # Draw vehicle heading with brand icon + fingerprint
    heading_offset = 0
    if self._fingerprint:
      heading_offset = HEADING_HEIGHT
      heading_rect = rl.Rectangle(rect.x, rect.y, rect.width, HEADING_HEIGHT)

      if self._brand_texture is not None:
        icon_y = heading_rect.y + (HEADING_HEIGHT - self._brand_texture.height) / 2
        rl.draw_texture_pro(
          self._brand_texture,
          rl.Rectangle(0, 0, self._brand_texture.width, self._brand_texture.height),
          rl.Rectangle(heading_rect.x, icon_y, self._brand_texture.width, self._brand_texture.height),
          rl.Vector2(0, 0), 0, rl.WHITE,
        )

      text_size = measure_text_cached(self._font, self._fingerprint, 55)
      text_x = heading_rect.x + heading_rect.width - text_size.x
      text_y = heading_rect.y + (HEADING_HEIGHT - text_size.y) / 2
      rl.draw_text_ex(self._font, self._fingerprint, rl.Vector2(text_x, text_y), 55, 0, rl.WHITE)

      sep_y = heading_rect.y + HEADING_HEIGHT - 1
      rl.draw_line_ex(rl.Vector2(rect.x, sep_y), rl.Vector2(rect.x + rect.width, sep_y), 1, rl.Color(64, 64, 64, 255))

    if self._scroller:
      content_rect = rl.Rectangle(rect.x, rect.y + heading_offset, rect.width, rect.height - heading_offset)
      self._scroller.render(content_rect)
