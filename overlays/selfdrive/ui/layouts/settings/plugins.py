"""Plugins panel for Settings UI — plugin lifecycle management (enable/disable).

Runtime driving feature tuning is in the Driving panel.
"""
import json
import os

from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget, DialogResult
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
from openpilot.system.ui.widgets.list_view import toggle_item
from openpilot.system.ui.widgets.scroller_tici import Scroller

PLUGINS_DIR = '/data/plugins'
BUILD_HASH_FILE = '/tmp/plugin_build_hash'
IS_C3 = os.path.exists('/TICI')

# Sort order matching COD
SORT_ORDER = {
  'model_selector': -3, 'lane_centering': -2, 'speedlimitd': -1,
  'mapd': 0, 'bmw_e9x_e8x': 1, 'c3_compat': 2,
}


class _PluginEntry:
  """Lightweight struct for a discovered plugin."""
  __slots__ = ('id', 'name', 'description', 'enabled')

  def __init__(self, id, name, description, enabled):
    self.id = id
    self.name = name
    self.description = description
    self.enabled = enabled


class PluginsLayout(Widget):
  def __init__(self):
    super().__init__()
    self._scroller = None
    self._entries = []
    self._needs_rebuild = True

  def _scan_plugins(self):
    """Scan /data/plugins/ for plugin manifests, return sorted list of _PluginEntry."""
    if not os.path.isdir(PLUGINS_DIR):
      return []

    entries = []
    for name in sorted(os.listdir(PLUGINS_DIR)):
      plugin_dir = os.path.join(PLUGINS_DIR, name)
      if not os.path.isdir(plugin_dir):
        continue
      manifest_path = os.path.join(plugin_dir, 'plugin.json')
      if not os.path.exists(manifest_path):
        continue
      try:
        with open(manifest_path) as f:
          manifest = json.load(f)
      except (json.JSONDecodeError, OSError):
        continue

      # Device filter
      device_filter = manifest.get('device_filter')
      if device_filter:
        device_type = 'tici' if IS_C3 else 'unknown'
        if device_type not in device_filter:
          continue

      # Skip enforced plugins — always on, no toggle needed
      if os.path.exists(os.path.join(plugin_dir, '.enforced')):
        continue

      disabled = os.path.exists(os.path.join(plugin_dir, '.disabled'))
      entries.append(_PluginEntry(
        id=name,
        name=manifest.get('name', name),
        description=manifest.get('description', ''),
        enabled=not disabled,
      ))

    entries.sort(key=lambda e: (SORT_ORDER.get(e.id, 0), e.id))
    return entries

  def _build_scroller(self):
    """Build widget list from plugin entries."""
    self._entries = self._scan_plugins()
    items = []

    for entry in self._entries:
      header = toggle_item(
        entry.name,
        entry.description,
        entry.enabled,
        callback=lambda state, e=entry: self._toggle_plugin(state, e),
        enabled=True,
      )
      items.append(header)

    self._scroller = Scroller(items, line_separator=True, spacing=0)

  def _toggle_plugin(self, state, entry):
    """Enable/disable a plugin via .disabled marker."""
    marker = os.path.join(PLUGINS_DIR, entry.id, '.disabled')
    if state:
      try:
        os.remove(marker)
      except FileNotFoundError:
        pass
      entry.enabled = True
    else:
      try:
        with open(marker, 'w') as f:
          f.write('')
      except OSError:
        pass
      entry.enabled = False

    # Force builder rebuild on next boot
    try:
      os.remove(BUILD_HASH_FILE)
    except FileNotFoundError:
      pass

    # Show reboot dialog
    dlg = ConfirmDialog('Reboot required to apply changes.', 'OK', cancel_text='')
    gui_app.set_modal_overlay(dlg, callback=lambda _: None)

    self._needs_rebuild = True

  def show_event(self):
    super().show_event()
    self._needs_rebuild = True

  def _render(self, rect):
    if self._needs_rebuild:
      self._build_scroller()
      self._needs_rebuild = False

    if self._scroller:
      self._scroller.render(rect)
