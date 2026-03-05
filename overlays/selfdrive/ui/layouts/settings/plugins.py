"""Plugins panel for Settings UI — on-device plugin management."""
import json
import os

from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget, DialogResult
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
from openpilot.system.ui.widgets.list_view import toggle_item, multiple_button_item, ListItem
from openpilot.system.ui.widgets.scroller_tici import Scroller

PLUGINS_DIR = '/data/plugins'
PARAMS_DIR = '/data/params/d'
BUILD_HASH_FILE = '/tmp/plugin_build_hash'
IS_C3 = os.path.exists('/TICI')

# Sort order matching COD
SORT_ORDER = {
  'model_selector': -3, 'lane_centering': -2, 'speedlimitd': -1,
  'mapd': 0, 'bmw_e9x_e8x': 1, 'c3_compat': 2,
}

# Mapd params that require MapdSettings JSON regeneration
MAPD_PARAM_KEYS = {'MapdSpeedLimitControlEnabled', 'MapdSpeedLimitOffsetPercent', 'MapdCurveTargetLatAccel'}


def _read_param(key):
  try:
    with open(os.path.join(PARAMS_DIR, key)) as f:
      return f.read().strip()
  except (FileNotFoundError, OSError):
    return ''


def _write_param(key, value):
  try:
    with open(os.path.join(PARAMS_DIR, key), 'w') as f:
      f.write(value)
  except OSError:
    pass


def _sync_mapd_settings():
  """Regenerate MapdSettings JSON from individual params."""
  enabled = _read_param('MapdSpeedLimitControlEnabled') == '1'
  try:
    offset_pct = int(_read_param('MapdSpeedLimitOffsetPercent') or '10')
  except ValueError:
    offset_pct = 10
  try:
    lat_idx = int(_read_param('MapdCurveTargetLatAccel') or '1')
  except ValueError:
    lat_idx = 1
  lat_vals = [1.5, 2.0, 2.5, 3.0]
  lat_accel = lat_vals[lat_idx] if 0 <= lat_idx < 4 else 2.0

  settings = {
    'speed_limit_control_enabled': enabled,
    'map_curve_speed_control_enabled': enabled,
    'vision_curve_speed_control_enabled': enabled,
    'speed_limit_offset': offset_pct / 100.0,
    'map_curve_target_lat_a': lat_accel,
    'vision_curve_target_lat_a': lat_accel,
  }
  _write_param('MapdSettings', json.dumps(settings))


class _PluginEntry:
  """Lightweight struct for a discovered plugin."""
  __slots__ = ('id', 'name', 'description', 'enabled', 'locked', 'params')

  def __init__(self, id, name, description, enabled, locked, params):
    self.id = id
    self.name = name
    self.description = description
    self.enabled = enabled
    self.locked = locked
    self.params = params


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

      locked = (name == 'c3_compat' and IS_C3)
      enabled = True if locked else not os.path.exists(os.path.join(plugin_dir, '.disabled'))

      # Collect visible params (those with 'desc' field)
      params = []
      for key, meta in manifest.get('params', {}).items():
        if 'desc' not in meta:
          continue
        params.append({
          'key': key,
          'type': meta.get('type', 'string'),
          'label': meta.get('label', key),
          'desc': meta['desc'],
          'default': meta.get('default'),
          'options': meta.get('options'),
          'suffix': meta.get('suffix', ''),
          'dependsOn': meta.get('dependsOn'),
          'requiresPlugin': meta.get('requiresPlugin'),
        })

      entries.append(_PluginEntry(
        id=name,
        name=manifest.get('name', name),
        description=manifest.get('description', ''),
        enabled=enabled,
        locked=locked,
        params=params,
      ))

    entries.sort(key=lambda e: (SORT_ORDER.get(e.id, 0), e.id))
    return entries

  def _build_scroller(self):
    """Build widget list from plugin entries."""
    self._entries = self._scan_plugins()
    items = []

    for entry in self._entries:
      # Plugin header toggle
      header = toggle_item(
        entry.name,
        entry.description,
        entry.enabled,
        callback=lambda state, e=entry: self._toggle_plugin(state, e),
        enabled=not entry.locked,
      )
      items.append(header)

      # Param items (only visible when plugin is enabled)
      for param in entry.params:
        item = self._build_param_item(entry, param)
        if item:
          item.set_visible(lambda e=entry: e.enabled)
          items.append(item)

    self._scroller = Scroller(items, line_separator=True, spacing=0)

  def _build_param_item(self, entry, param):
    """Build a ListItem for a single param definition."""
    key = param['key']
    ptype = param['type']

    # Compute enabled state: dependsOn + requiresPlugin
    def is_enabled(p=param, e=entry):
      if p.get('requiresPlugin'):
        req = p['requiresPlugin']
        if os.path.exists(os.path.join(PLUGINS_DIR, req, '.disabled')):
          return False
      if p.get('dependsOn'):
        return _read_param(p['dependsOn']) == '1'
      return True

    if ptype == 'bool':
      current = _read_param(key)
      initial = current == '1' if current else bool(param.get('default', False))
      return toggle_item(
        param['label'],
        param['desc'],
        initial,
        callback=lambda state, k=key: self._on_param_bool(k, state),
        enabled=is_enabled,
      )

    elif ptype == 'pills':
      options = param.get('options', [])
      suffix = param.get('suffix', '')
      buttons = [f"{opt}{suffix}" for opt in options]
      current = _read_param(key)
      try:
        selected = int(current) if current else param.get('default', 0)
      except ValueError:
        selected = param.get('default', 0)
      return multiple_button_item(
        param['label'],
        param['desc'],
        buttons=buttons,
        selected_index=selected,
        button_width=150,
        callback=lambda idx, k=key: self._on_param_pills(k, idx),
      )

    return None

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

  def _on_param_bool(self, key, state):
    _write_param(key, '1' if state else '0')
    if key in MAPD_PARAM_KEYS:
      _sync_mapd_settings()

  def _on_param_pills(self, key, index):
    _write_param(key, str(index))
    if key in MAPD_PARAM_KEYS:
      _sync_mapd_settings()

  def show_event(self):
    super().show_event()
    self._needs_rebuild = True

  def _render(self, rect):
    if self._needs_rebuild:
      self._build_scroller()
      self._needs_rebuild = False

    if self._scroller:
      self._scroller.render(rect)
