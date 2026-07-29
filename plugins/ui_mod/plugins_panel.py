"""Plugins panel for Settings UI — plugin lifecycle management (enable/disable).

Runtime driving feature tuning is in the Driving panel.
"""
import json
import math
import os
import subprocess
import threading
import time

from config import PLUGINS_RUNTIME_DIR, PLUGINS_REPO_DIR, OPENPILOT_DIR
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget, DialogResult
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
from openpilot.system.ui.widgets.list_view import button_item, toggle_item
from openpilot.system.ui.widgets.scroller_tici import Scroller

PLUGINS_DIR = PLUGINS_RUNTIME_DIR
PLUGINS_REPO = PLUGINS_REPO_DIR
BUILD_HASH_FILE = '/tmp/plugin_build_hash'
IS_C3 = os.path.exists('/TICI')

# Essential plugins — always enforced (non-toggleable)
ESSENTIAL_PLUGINS = {'bus_logger'}

# Sort order: user-facing plugins first, essential + c3_compat at the bottom
SORT_ORDER = {
  'model_selector': -3, 'lane_centering': -2, 'network_settings': -1,
  'bmw_e9x_e8x': 0,
  'speedlimitd': 10, 'bus_logger': 11, 'mapd': 12, 'c3_compat': 13,
}


class _PluginEntry:
  """Lightweight struct for a discovered plugin."""
  __slots__ = ('id', 'name', 'description', 'enabled', 'enforced')

  def __init__(self, id, name, description, enabled, enforced=False):
    self.id = id
    self.name = name
    self.description = description
    self.enabled = enabled
    self.enforced = enforced


class PluginsLayout(Widget):
  def __init__(self):
    super().__init__()
    self._entries = []

    # Update state
    self._update_commits_behind = 0
    self._show_reboot = False
    self._last_check_time = None  # monotonic timestamp of last successful check
    self._check_state = 'idle'  # idle, checking, checked, failed, updating, updated, update_failed
    self._cached_hash = None  # cached git hash, computed once

    self._update_btn = button_item(
      'Plugin Updates',
      'CHECK',
      callback=self._on_update_click,
    )

    # Fix float32 truncation: ceil + extra padding for rl.Rectangle precision loss
    _orig_hint = self._update_btn.action_item.get_width_hint
    self._update_btn.action_item.get_width_hint = lambda: math.ceil(_orig_hint()) + 10

    # Build scroller once (like Software panel)
    self._scroller = self._build_scroller()

  def _scan_plugins(self):
    """Scan /data/plugins-runtime/ for plugin manifests, return sorted list of _PluginEntry."""
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

      # Hidden plugins (panel: false)
      if manifest.get('panel') is False:
        continue

      # Device filter
      device_filter = manifest.get('device_filter')
      if device_filter:
        device_type = 'tici' if IS_C3 else 'unknown'
        if device_type not in device_filter:
          continue

      enforced = name in ESSENTIAL_PLUGINS or os.path.exists(os.path.join(plugin_dir, '.enforced'))
      disabled = not enforced and os.path.exists(os.path.join(plugin_dir, '.disabled'))
      entries.append(_PluginEntry(
        id=name,
        name=manifest.get('name', name),
        description=manifest.get('description', ''),
        enabled=not disabled,
        enforced=enforced,
      ))

    entries.sort(key=lambda e: (SORT_ORDER.get(e.id, 0), e.id))
    return entries

  def _get_current_hash(self):
    """Return short hash of currently installed plugins commit (cached)."""
    if self._cached_hash is None:
      try:
        self._cached_hash = subprocess.check_output(
          ['git', '-C', PLUGINS_REPO, 'rev-parse', '--short', 'HEAD'],
          stderr=subprocess.DEVNULL, timeout=5,
        ).decode().strip()
      except Exception:
        self._cached_hash = ''
    return self._cached_hash

  @staticmethod
  def _get_target_branch():
    """Get target branch — aligned to catpilot's branch."""
    try:
      return subprocess.check_output(
        ['git', '-C', OPENPILOT_DIR, 'rev-parse', '--abbrev-ref', 'HEAD'],
        stderr=subprocess.DEVNULL, timeout=5,
      ).decode().strip()
    except Exception:
      pass
    try:
      return subprocess.check_output(
        ['git', '-C', PLUGINS_REPO, 'rev-parse', '--abbrev-ref', 'HEAD'],
        stderr=subprocess.DEVNULL, timeout=5,
      ).decode().strip()
    except Exception:
      return 'main'

  def _on_update_click(self):
    if self._check_state in ('idle', 'checked', 'failed', 'update_failed'):
      self._check_state = 'checking'
      threading.Thread(target=self._check_update, daemon=True).start()
    elif self._check_state == 'available':
      self._check_state = 'updating'
      threading.Thread(target=self._apply_update, daemon=True).start()

  def _time_ago(self):
    if self._last_check_time is None:
      return 'never'
    diff = int(time.monotonic() - self._last_check_time)
    if diff < 60:
      return 'now'
    if diff < 3600:
      m = diff // 60
      return f'{m} minute{"s" if m != 1 else ""} ago'
    if diff < 86400:
      h = diff // 3600
      return f'{h} hour{"s" if h != 1 else ""} ago'
    d = diff // 86400
    return f'{d} day{"s" if d != 1 else ""} ago'

  def _check_update(self):
    try:
      branch = self._get_target_branch()
      subprocess.check_call(
        ['git', '-C', PLUGINS_REPO, 'fetch', 'origin', branch, '--quiet'],
        timeout=30, stderr=subprocess.DEVNULL,
        env={**os.environ, 'GIT_SSL_NO_VERIFY': '1'},
      )
      behind = subprocess.check_output(
        ['git', '-C', PLUGINS_REPO, 'rev-list', '--count', f'HEAD..origin/{branch}'],
        stderr=subprocess.DEVNULL, timeout=5,
      ).decode().strip()
      n = int(behind)
      self._update_commits_behind = n
      self._last_check_time = time.monotonic()
      self._check_state = 'available' if n > 0 else 'checked'
    except Exception:
      self._check_state = 'failed'

  def _apply_update(self):
    try:
      branch = self._get_target_branch()
      subprocess.check_call(
        ['git', '-C', PLUGINS_REPO, 'reset', '--hard', f'origin/{branch}'],
        timeout=60, stderr=subprocess.DEVNULL,
        env={**os.environ, 'GIT_SSL_NO_VERIFY': '1'},
      )
      subprocess.check_call(
        ['bash', os.path.join(PLUGINS_REPO, 'install.sh'), '--target', OPENPILOT_DIR],
        timeout=120, stderr=subprocess.DEVNULL,
      )
      try:
        os.remove(BUILD_HASH_FILE)
      except FileNotFoundError:
        pass
      self._cached_hash = None  # invalidate cached hash
      self._check_state = 'updated'
      self._show_reboot = True
    except Exception:
      self._check_state = 'update_failed'

  def _build_scroller(self):
    """Build widget list from plugin entries and return Scroller."""
    self._entries = self._scan_plugins()
    items = []

    # Update button at top (only if repo exists on device)
    if os.path.isdir(os.path.join(PLUGINS_REPO, '.git')):
      items.append(self._update_btn)

    for entry in self._entries:
      if entry.enforced:
        header = toggle_item(
          entry.name,
          entry.description,
          initial_state=True,
          enabled=False,
        )
      else:
        header = toggle_item(
          entry.name,
          entry.description,
          entry.enabled,
          callback=lambda state, e=entry: self._toggle_plugin(state, e),
          enabled=True,
        )
      items.append(header)

    return Scroller(items, line_separator=True, spacing=0)

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
    gui_app.push_widget(ConfirmDialog('Reboot required to apply changes.', 'OK', cancel_text=''))

  def show_event(self):
    super().show_event()
    self._scroller.show_event()

  def _update_state(self):
    btn = self._update_btn
    if self._check_state == 'checking':
      btn.action_item.set_value('checking...')
      btn.action_item.set_text('CHECK')
      btn.action_item.set_enabled(False)
    elif self._check_state == 'updating':
      btn.action_item.set_value('updating...')
      btn.action_item.set_text('UPDATE')
      btn.action_item.set_enabled(False)
    elif self._check_state == 'available':
      n = self._update_commits_behind
      btn.action_item.set_value(f'{n} new commit{"s" if n != 1 else ""} available')
      btn.action_item.set_text('UPDATE')
      btn.action_item.set_enabled(True)
    elif self._check_state == 'checked':
      btn.action_item.set_value(f'up to date, last checked {self._time_ago()}')
      btn.action_item.set_text('CHECK')
      btn.action_item.set_enabled(True)
    elif self._check_state == 'failed':
      btn.action_item.set_value('check failed')
      btn.action_item.set_text('CHECK')
      btn.action_item.set_enabled(True)
    elif self._check_state == 'updated':
      btn.action_item.set_value('updated, reboot to apply')
      btn.action_item.set_text('CHECK')
      btn.action_item.set_enabled(False)
    elif self._check_state == 'update_failed':
      btn.action_item.set_value('update failed')
      btn.action_item.set_text('CHECK')
      btn.action_item.set_enabled(True)
    else:
      btn.action_item.set_value('up to date, last checked never')
      btn.action_item.set_text('CHECK')
      btn.action_item.set_enabled(True)

  def _render(self, rect):
    if self._show_reboot:
      self._show_reboot = False
      self._prompt_reboot()

    self._scroller.render(rect)

  def _prompt_reboot(self):
    def _on_reboot(result):
      if result == DialogResult.CONFIRM:
        try:
          with open('/data/params/d/DoReboot', 'w') as f:
            f.write('1')
        except OSError:
          pass
    gui_app.push_widget(ConfirmDialog('Plugin update installed. Reboot now?', 'Reboot', cancel_text='Later', callback=_on_reboot))
