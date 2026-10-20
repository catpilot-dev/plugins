"""Tests for BMW plugin hook handlers — interface registration, cruise ceiling."""
import os
import sys
import pytest
from unittest.mock import MagicMock, patch
from types import SimpleNamespace

# Add plugin dir to path so register module is importable
_PLUGIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PLUGIN_DIR not in sys.path:
  sys.path.insert(0, _PLUGIN_DIR)

from test_helpers import install_all_mocks


@pytest.fixture(autouse=True)
def mock_deps(monkeypatch):
  """Mock opendbc/cereal imports for standalone testing."""
  install_all_mocks(monkeypatch)


@pytest.fixture
def param_dir(tmp_path, monkeypatch):
  """Set up a temp data dir for plugin params."""
  import register
  data_dir = tmp_path / 'data'
  data_dir.mkdir()
  monkeypatch.setattr(register, '_PLUGIN_DIR', str(tmp_path))
  return data_dir


# ============================================================
# Interface Registration (opendbc patching)
# ============================================================

class TestRegisterInterfaces:
  def test_patches_interfaces_at_load(self, mock_deps):
    """_register_interfaces runs at module load and patches car_helpers.interfaces."""
    from opendbc.car.car_helpers import interfaces
    import importlib
    import register
    importlib.reload(register)
    assert any('E82' in str(k) for k in interfaces)
    assert any('E90' in str(k) for k in interfaces)

  def test_preserves_existing_interfaces(self, mock_deps):
    from opendbc.car.car_helpers import interfaces
    mock_iface = MagicMock()
    interfaces['HONDA_CIVIC'] = mock_iface
    import importlib
    import register
    importlib.reload(register)
    assert interfaces['HONDA_CIVIC'] is mock_iface

  def test_patches_torque_params(self, mock_deps):
    import importlib
    import opendbc.car.interfaces as _intf
    original_params = {'HONDA_CIVIC': {'LAT_ACCEL_FACTOR': 1.0}}
    _intf.get_torque_params = lambda: dict(original_params)

    import register
    importlib.reload(register)

    patched_params = _intf.get_torque_params()
    assert 'HONDA_CIVIC' in patched_params
    bmw_keys = [k for k in patched_params if 'BMW' in k.upper()]
    assert len(bmw_keys) >= 1

  def test_torque_params_toml_exists(self):
    """torque_params.toml exists and is parseable."""
    import tomllib
    toml_path = os.path.join(_PLUGIN_DIR, 'torque_params.toml')
    assert os.path.exists(toml_path)
    with open(toml_path, 'rb') as f:
      data = tomllib.load(f)
    assert 'legend' in data
    assert any('BMW' in k.upper() for k in data if k != 'legend')


# ============================================================
# Lateral controller module (split out of register.py 2026-07-03)
# ============================================================

class TestLateralControllerModule:
  def test_module_exposes_hook(self, mock_deps):
    """bmw/latcontroller.py loads the way the registry loads it (file-level, and exposes the hook target
    canonical name) and exposes the hook plugin.json points at."""
    import importlib.util
    path = os.path.join(_PLUGIN_DIR, 'bmw', 'latcontroller.py')
    spec = importlib.util.spec_from_file_location('plugins.bmw_e9x_e8x.bmw.latcontroller', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    assert callable(mod.on_lat_controller_init)

  def test_plugin_json_points_at_module(self):
    import json
    with open(os.path.join(_PLUGIN_DIR, 'plugin.json')) as f:
      hooks = json.load(f)['hooks']
    lat = hooks['controls.lat_controller_init']
    assert lat['module'] == 'bmw.latcontroller'
    assert lat['function'] == 'on_lat_controller_init'


# ============================================================
# ISO accel/jerk cancel guard — REMOVED 2026-07-28 (lateral never gives up in
# a turn; a_y is bounded at the system level by speedlimitd's curve-speed
# capping); there is no accel guard or cancel machinery here. The no-cancel
# behaviour is covered by tests/test_latcontroller.py.
# ============================================================


# ============================================================
# Cruise Ceiling Memory
# ============================================================

class TestCruiseCeilingMemory:
  def test_restores_last_cruise(self, param_dir):
    import register
    helper = SimpleNamespace(v_cruise_kph=105, v_cruise_kph_last=80, v_cruise_cluster_kph=105)
    register.on_cruise_initialized(None, helper, None)
    assert helper.v_cruise_kph == 80
    assert helper.v_cruise_cluster_kph == 80

  def test_no_restore_on_first_engage(self, param_dir):
    import register
    helper = SimpleNamespace(v_cruise_kph=105, v_cruise_kph_last=0, v_cruise_cluster_kph=105)
    register.on_cruise_initialized(None, helper, None)
    assert helper.v_cruise_kph == 105

  def test_disabled_by_param(self, param_dir):
    (param_dir / 'CruiseCeilingMemory').write_text('0')
    import register
    helper = SimpleNamespace(v_cruise_kph=105, v_cruise_kph_last=80, v_cruise_cluster_kph=105)
    register.on_cruise_initialized(None, helper, None)
    assert helper.v_cruise_kph == 105

  def test_enabled_by_param(self, param_dir):
    (param_dir / 'CruiseCeilingMemory').write_text('1')
    import register
    helper = SimpleNamespace(v_cruise_kph=105, v_cruise_kph_last=80, v_cruise_cluster_kph=105)
    register.on_cruise_initialized(None, helper, None)
    assert helper.v_cruise_kph == 80

  def test_enabled_by_default_no_file(self, param_dir):
    """Default enabled when param file doesn't exist."""
    import register
    helper = SimpleNamespace(v_cruise_kph=105, v_cruise_kph_last=80, v_cruise_cluster_kph=105)
    register.on_cruise_initialized(None, helper, None)
    assert helper.v_cruise_kph == 80


# ============================================================
# Vehicle Settings (ui.vehicle_settings hook)
# ============================================================

class TestVehicleSettingsRows:
  """The Driving-panel rows this plugin contributes (ui.vehicle_settings)."""

  def _rows(self, monkeypatch, param_dir):
    import register

    def fake_toggle_item(title, description, initial_state=False, callback=None, enabled=True):
      return SimpleNamespace(title=title, description=description,
                             initial_state=initial_state, callback=callback,
                             enabled=enabled)

    lv = MagicMock()
    lv.toggle_item = fake_toggle_item
    monkeypatch.setitem(sys.modules, 'openpilot.system.ui.widgets.list_view', lv)
    return register.on_vehicle_settings([], SimpleNamespace(brand='bmw'))

  def test_exact_row_set(self, mock_deps, monkeypatch, param_dir):
    """Retirement pin (2026-08-14): the 2-degree steering-push toggle failed
    its route-3f4 A/B and was deleted with the feature. Pinned as an EXACT
    row list rather than a not-in check so the retired row cannot return
    under any title.

    'Stall Breakaway' (2026-08-15) is a DIFFERENT mechanism and is listed
    here deliberately: the refuted one bounded torque on degrees moved, this
    one trips on rack state, which is what the retirement note requires of
    any replacement. The retired row's own title stays pinned out below."""
    titles = [r.title for r in self._rows(monkeypatch, param_dir)]
    assert titles == ['Temperature Overlay', 'Stall Breakaway', 'Slope Compensation',
                      'Resume Button Repurposed']
    assert 'Steering Push Budget' not in titles

  def test_non_bmw_gets_no_rows(self, mock_deps, monkeypatch, param_dir):
    import register
    items = register.on_vehicle_settings([], SimpleNamespace(brand='toyota'))
    assert items == []

  def _row(self, monkeypatch, param_dir, title):
    rows = [r for r in self._rows(monkeypatch, param_dir) if r.title == title]
    assert len(rows) == 1
    return rows[0]

  def test_temperature_overlay_row_reflects_the_param(self, mock_deps, monkeypatch, param_dir):
    """Default-ON polarity (`!= '0'`): absent file and '1' both read as on,
    only an explicit '0' reads as off. The adjacent Stall Breakaway row uses
    the opposite (`== '1'`) predicate, so this pins that the two did not get
    mixed up — copying this row's predicate onto that one would silently arm
    a safety net on fresh installs."""
    assert self._row(monkeypatch, param_dir, 'Temperature Overlay').initial_state is True
    (param_dir / 'TemperatureOverlay').write_text('0')
    assert self._row(monkeypatch, param_dir, 'Temperature Overlay').initial_state is False
    (param_dir / 'TemperatureOverlay').write_text('1')
    assert self._row(monkeypatch, param_dir, 'Temperature Overlay').initial_state is True

  def test_toggle_callback_persists_via_write_param(self, mock_deps, monkeypatch, param_dir):
    """`_write_param` is the only persistence path this plugin's panel has —
    it creates the data dir and writes the file that `_read_param` (and the
    plugin's own consumers) later read. A no-op `_write_param` leaves the row
    working in the UI and silently loses every setting at restart, so pin the
    round trip through a real callback: flip it, read the FILE back, and
    confirm the rebuilt row reflects it."""
    row = self._row(monkeypatch, param_dir, 'Temperature Overlay')

    row.callback(False)
    assert (param_dir / 'TemperatureOverlay').read_text() == '0'
    assert self._row(monkeypatch, param_dir, 'Temperature Overlay').initial_state is False

    row.callback(True)
    assert (param_dir / 'TemperatureOverlay').read_text() == '1'
    assert self._row(monkeypatch, param_dir, 'Temperature Overlay').initial_state is True

  def test_stall_breakaway_defaults_off(self, mock_deps, monkeypatch, param_dir):
    """No param file: the row must be OFF. Guards the `== '1'` predicate —
    the adjacent TemperatureOverlay row uses `!= '0'` (default-ON), and
    copying that here would silently arm the mechanism on fresh installs,
    contradicting latcontroller's own default-OFF read."""
    assert self._row(monkeypatch, param_dir, 'Stall Breakaway').initial_state is False

  def test_stall_breakaway_reflects_existing_param(self, mock_deps, monkeypatch, param_dir):
    """'1' is the only enabling value, matching latcontroller's
    `read_plugin_param(...) == '1'`; anything else reads as off."""
    (param_dir / 'StallBreakaway').write_text('1')
    assert self._row(monkeypatch, param_dir, 'Stall Breakaway').initial_state is True
    (param_dir / 'StallBreakaway').write_text('0')
    assert self._row(monkeypatch, param_dir, 'Stall Breakaway').initial_state is False

  def test_stall_breakaway_callback_writes_the_param(self, mock_deps, monkeypatch, param_dir):
    """The param FILE is the only channel to controlsd — there is no bus
    topic or heartbeat for this toggle, so a callback that does not persist
    leaves the UI looking right and changes nothing at the next drive."""
    row = self._row(monkeypatch, param_dir, 'Stall Breakaway')

    row.callback(True)
    assert (param_dir / 'StallBreakaway').read_text() == '1'
    assert self._row(monkeypatch, param_dir, 'Stall Breakaway').initial_state is True

    row.callback(False)
    assert (param_dir / 'StallBreakaway').read_text() == '0'
    assert self._row(monkeypatch, param_dir, 'Stall Breakaway').initial_state is False

  def test_stall_breakaway_absent_for_non_bmw(self, mock_deps, monkeypatch, param_dir):
    """This is a BMW rack behaviour; the hook runs for every brand's Driving
    panel, so the brand guard must keep the row off other cars."""
    import register
    items = register.on_vehicle_settings([], SimpleNamespace(brand='toyota'))
    assert [r for r in items if getattr(r, 'title', None) == 'Stall Breakaway'] == []

  def test_slope_compensation_row_state_and_callback(self, mock_deps, monkeypatch, param_dir):
    """Default-OFF (`== '1'`), persisted to the param file latcontroller
    reads at each engagement."""
    assert self._row(monkeypatch, param_dir, 'Slope Compensation').initial_state is False
    (param_dir / 'SlopeCompensation').write_text('1')
    assert self._row(monkeypatch, param_dir, 'Slope Compensation').initial_state is True
    (param_dir / 'SlopeCompensation').write_text('0')
    row = self._row(monkeypatch, param_dir, 'Slope Compensation')
    assert row.initial_state is False

    row.callback(True)
    assert (param_dir / 'SlopeCompensation').read_text() == '1'
    row.callback(False)
    assert (param_dir / 'SlopeCompensation').read_text() == '0'

  def test_slope_compensation_row_follows_stall_breakaway(self, mock_deps, monkeypatch, param_dir):
    titles = [r.title for r in self._rows(monkeypatch, param_dir)]
    assert titles.index('Slope Compensation') == titles.index('Stall Breakaway') + 1

  def test_slope_compensation_absent_for_non_bmw(self, mock_deps, monkeypatch, param_dir):
    import register
    items = register.on_vehicle_settings([], SimpleNamespace(brand='toyota'))
    assert [r for r in items if getattr(r, 'title', None) == 'Slope Compensation'] == []

  def test_write_param_creates_a_missing_data_dir(self, mock_deps, monkeypatch, tmp_path):
    """`_write_param` does `os.makedirs(..., exist_ok=True)` — on a fresh
    install /data/plugins/<id>/data does not exist yet, and without that the
    first toggle would raise instead of persisting."""
    import register
    monkeypatch.setattr(register, '_PLUGIN_DIR', str(tmp_path))
    assert not (tmp_path / 'data').exists()
    register._write_param('TemperatureOverlay', '0')
    assert (tmp_path / 'data' / 'TemperatureOverlay').read_text() == '0'
    assert register._read_param('TemperatureOverlay') == '0'

  def test_state_tick_hook_targets_existing_function(self):
    """Originally a retirement pin (2026-08-14) asserting `ui.state_tick` was
    gone with the steering-push hot toggle. The hook returned 2026-08-16 for
    the LKA override-grey border, now in ui_overlay — the hazard the pin
    guarded (a manifest entry pointing at a missing function breaks plugin
    load) is asserted directly instead: the entry must target a function that
    exists, and the retired register-module handler must stay gone."""
    import json as _json
    import register
    with open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                           'plugin.json')) as f:
      manifest = _json.load(f)
    entry = manifest['hooks']['ui.state_tick']
    assert entry['module'] == 'ui_overlay'
    assert entry['function'] == 'on_ui_state_tick'
    ov_src = open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                               'ui_overlay.py')).read()
    assert 'def on_ui_state_tick(' in ov_src
    assert not hasattr(register, 'on_ui_state_tick')
