"""Vehicle panel for Settings UI — vehicle-specific parameters.

Only visible when a car is fingerprinted (ui_state.CP is not None).
Items are populated by car plugins via the 'ui.vehicle_settings' hook.

Hook signature:
  callback(items: list, CP: CarParams) -> items
  Plugins append toggle_item/multiple_button_item widgets to the list.
"""
import os
from config import PLUGINS_REPO_DIR

import pyray as rl
from openpilot.selfdrive.plugins.hooks import hooks
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.scroller_tici import Scroller

HEADING_HEIGHT = 80
ICON_SIZE = 64
LOGOS_DIR = os.path.join(PLUGINS_REPO_DIR, 'logos', 'icons')


class VehicleLayout(Widget):
  def __init__(self):
    super().__init__()
    self._scroller = None
    self._needs_rebuild = True
    self._brand_texture = None
    self._fingerprint = ''
    self._font = gui_app.font(FontWeight.MEDIUM)

  def _load_brand_icon(self, brand):
    """Load brand icon from plugins logos directory."""
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

    CP = ui_state.CP
    if CP is not None:
      self._fingerprint = CP.carFingerprint
      brand = getattr(CP, 'brand', '')
      if brand and self._brand_texture is None:
        self._brand_texture = self._load_brand_icon(brand)
      items = hooks.run('ui.vehicle_settings', items, CP)

    self._scroller = Scroller(items, line_separator=True, spacing=0) if items else None

  def show_event(self):
    super().show_event()
    self._needs_rebuild = True

  def _render(self, rect):
    if self._needs_rebuild:
      self._build_scroller()
      self._needs_rebuild = False

    # Draw heading with brand icon (left) and fingerprint (right)
    if self._fingerprint:
      heading_rect = rl.Rectangle(rect.x, rect.y, rect.width, HEADING_HEIGHT)

      # Brand icon — left aligned, vertically centered
      if self._brand_texture is not None:
        icon_y = heading_rect.y + (HEADING_HEIGHT - self._brand_texture.height) / 2
        rl.draw_texture_pro(
          self._brand_texture,
          rl.Rectangle(0, 0, self._brand_texture.width, self._brand_texture.height),
          rl.Rectangle(heading_rect.x, icon_y, self._brand_texture.width, self._brand_texture.height),
          rl.Vector2(0, 0), 0, rl.WHITE,
        )

      # Fingerprint — right aligned, vertically centered
      text_size = measure_text_cached(self._font, self._fingerprint, 55)
      text_x = heading_rect.x + heading_rect.width - text_size.x
      text_y = heading_rect.y + (HEADING_HEIGHT - text_size.y) / 2
      rl.draw_text_ex(self._font, self._fingerprint, rl.Vector2(text_x, text_y), 55, 0, rl.WHITE)

      # Separator line
      sep_y = heading_rect.y + HEADING_HEIGHT - 1
      rl.draw_line_ex(rl.Vector2(rect.x, sep_y), rl.Vector2(rect.x + rect.width, sep_y), 1, rl.Color(64, 64, 64, 255))

    # Scroller below heading
    if self._scroller:
      content_y = rect.y + (HEADING_HEIGHT if self._fingerprint else 0)
      content_rect = rl.Rectangle(rect.x, content_y, rect.width, rect.height - (HEADING_HEIGHT if self._fingerprint else 0))
      self._scroller.render(content_rect)
