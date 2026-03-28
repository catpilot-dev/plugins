"""Experimental mode button — vehicle emblem with state indicators.

Replaces stock steering wheel / atomic icons with the vehicle brand emblem.
  - Normal: white icon on dark background
  - Experimental: color emblem on dark background
  - Lane centering active: green ring around the button
"""
import os
import sys
import time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import PLUGINS_REPO_DIR
import pyray as rl
from openpilot.common.params import Params
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget

ICONS_DIR = os.path.join(PLUGINS_REPO_DIR, 'logos', 'icons')
EMBLEMS_DIR = os.path.join(PLUGINS_REPO_DIR, 'logos', 'emblems')

# Colors
BG_COLOR = rl.Color(0, 0, 0, 77)               # 30% opacity black
RING_LANE_CENTERING = rl.Color(76, 175, 80, 255)  # green
RING_WIDTH = 12.0


class ExpButton(Widget):
  def __init__(self, button_size: int, icon_size: int):
    super().__init__()
    self._params = Params()
    self._experimental_mode: bool = False
    self._engageable: bool = False
    self._icon_size = icon_size

    # State hold mechanism
    self._hold_duration = 2.0  # seconds
    self._held_mode: bool | None = None
    self._hold_end_time: float | None = None

    self._white_color: rl.Color = rl.Color(255, 255, 255, 255)
    self._txt_icon: rl.Texture | None = None      # white-on-transparent
    self._txt_emblem: rl.Texture | None = None     # color emblem
    self._textures_loaded: bool = False
    self._lane_centering_active: bool = False
    self._lcc_sub = None
    self._rect = rl.Rectangle(0, 0, button_size, button_size)

  def set_rect(self, rect: rl.Rectangle) -> None:
    self._rect.x, self._rect.y = rect.x, rect.y

  def _load_textures(self):
    """Load white icon and color emblem for the vehicle brand, resized to icon_size."""
    self._textures_loaded = True
    CP = ui_state.CP
    if CP is None:
      return
    brand = getattr(CP, 'brand', '')
    if not brand:
      return
    # White icon (normal mode)
    icon_path = os.path.join(ICONS_DIR, f'{brand}.png')
    if os.path.isfile(icon_path):
      try:
        self._txt_icon = self._load_and_resize(icon_path)
      except Exception:
        pass
    # Color emblem (experimental mode)
    emblem_path = os.path.join(EMBLEMS_DIR, f'{brand}.png')
    if os.path.isfile(emblem_path):
      try:
        self._txt_emblem = self._load_and_resize(emblem_path)
      except Exception:
        pass

  def _load_and_resize(self, path: str) -> rl.Texture:
    """Load image, resize to fit button with 12px padding (maintaining aspect ratio), return texture."""
    img = rl.load_image(path)
    # 12px gap between emblem edge and circle boundary
    target = int(self._rect.width) - 24
    scale = target / max(img.width, img.height)
    new_w = int(img.width * scale)
    new_h = int(img.height * scale)
    rl.image_resize(img, new_w, new_h)
    tex = rl.load_texture_from_image(img)
    rl.unload_image(img)
    return tex

  def _update_state(self) -> None:
    selfdrive_state = ui_state.sm["selfdriveState"]
    self._experimental_mode = selfdrive_state.experimentalMode
    self._engageable = selfdrive_state.engageable or selfdrive_state.enabled

    # Load textures lazily (needs CP to be available)
    if not self._textures_loaded:
      self._load_textures()

    # Check lane centering state via plugin bus (live) or pluginBusLog cereal (replay)
    try:
      if self._lcc_sub is None:
        from openpilot.selfdrive.plugins.plugin_bus import PluginSub
        self._lcc_sub = PluginSub(['lane_centering_state'])
      msg = self._lcc_sub.drain()
      if msg is not None:
        _, data = msg
        self._lane_centering_active = data.get('active', False)
    except Exception:
      pass

    # Fallback: read from pluginBusLog (replayed from rlog)
    if not self._lane_centering_active:
      try:
        sm = ui_state.sm
        if sm.updated.get('pluginBusLog', False):
          for entry in sm['pluginBusLog'].entries:
            if entry.topic == 'lane_centering_state':
              import json
              data = json.loads(entry.json)
              self._lane_centering_active = data.get('active', False)
      except Exception:
        pass

  def _handle_mouse_release(self, _):
    super()._handle_mouse_release(_)
    if self._is_toggle_allowed():
      new_mode = not self._experimental_mode
      self._params.put_bool("ExperimentalMode", new_mode)

      # Hold new state temporarily
      self._held_mode = new_mode
      self._hold_end_time = time.monotonic() + self._hold_duration

  def _render(self, rect: rl.Rectangle) -> None:
    center_x = int(self._rect.x + self._rect.width // 2)
    center_y = int(self._rect.y + self._rect.height // 2)

    radius = self._rect.width / 2

    # Dark background always
    rl.draw_circle(center_x, center_y, radius, BG_COLOR)

    # Green ring when lane centering active
    if self._lane_centering_active:
      rl.draw_ring(rl.Vector2(center_x, center_y), radius - RING_WIDTH, radius, 0, 360, 36, RING_LANE_CENTERING)

    # Experimental mode: color emblem; normal mode: white icon
    alpha = 180 if self.is_pressed or not self._engageable else 255
    tint = rl.Color(255, 255, 255, alpha)
    if self._held_or_actual_mode() and self._txt_emblem and rl.is_texture_valid(self._txt_emblem):
      tex = self._txt_emblem
    elif self._txt_icon and rl.is_texture_valid(self._txt_icon):
      tex = self._txt_icon
    else:
      tex = None

    if tex:
      rl.draw_texture_ex(tex, rl.Vector2(center_x - tex.width // 2, center_y - tex.height // 2), 0.0, 1.0, tint)

  def _held_or_actual_mode(self):
    now = time.monotonic()
    if self._hold_end_time and now < self._hold_end_time:
      return self._held_mode

    if self._hold_end_time and now >= self._hold_end_time:
      self._hold_end_time = self._held_mode = None

    return self._experimental_mode

  def _is_toggle_allowed(self):
    if not self._params.get_bool("ExperimentalModeConfirmed"):
      return False

    # Mirror exp mode toggle using persistent car params
    return ui_state.has_longitudinal_control
