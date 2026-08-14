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
# hold_factor — curvature hold gate (2026-07-27: moved from |kappa_des| to
# commanded lateral accel v²·|kappa_des|; route 3ca seg 23 hunting fix).
# Pure function, module-level, no controller construction needed.
# ============================================================

class TestHoldFactor:
  def test_3ca_seg23_mild_fast_curve_now_holds(self, mock_deps):
    """19.4 m/s, kappa 0.0033 -> a_y 1.25: above HOLD_AY_BP[1], full hold.
    This is the case that was broken under the old kappa-only gate
    (HOLD_KAPPA_BP[0] = 0.004 > 0.0033 -> hold_f was 0 -> drain -> hunting)."""
    from bmw.latcontroller import hold_factor
    assert hold_factor(19.4, 0.0033) == 1.0

  def test_tight_slow_curve_holds(self, mock_deps):
    """9.0 m/s, kappa 0.012 -> a_y 0.972: at/above HOLD_AY_BP[1], full hold
    (route 380/384 hairpin fix operating point)."""
    from bmw.latcontroller import hold_factor
    assert hold_factor(9.0, 0.012) == 1.0

  def test_reference_mild_curve_still_drains(self, mock_deps):
    """12.4 m/s, kappa 0.0023 -> a_y 0.354: below HOLD_AY_BP[0], drains to 0
    (the reference case that damps fine with drain — must stay drained)."""
    from bmw.latcontroller import hold_factor
    assert hold_factor(12.4, 0.0023) == 0.0

  def test_fast_straight_drains(self, mock_deps):
    """19.4 m/s, kappa 0.0008 -> a_y 0.301: near-straight at highway speed,
    below HOLD_AY_BP[0], drains to 0."""
    from bmw.latcontroller import hold_factor
    assert hold_factor(19.4, 0.0008) == 0.0

  def test_parking_speed_tight_kappa_drains(self, mock_deps):
    """3.0 m/s, kappa 0.05 -> a_y 0.45: tight kappa but parking-speed slow,
    SAT far below stiction -> drains to 0 despite the large kappa. This is
    the case a kappa-only gate would get wrong in the other direction."""
    from bmw.latcontroller import hold_factor
    assert hold_factor(3.0, 0.05) == 0.0

  def test_transition_sliver_is_partial(self):
    """14.0 m/s, kappa 0.0035 -> a_y 0.686: strictly inside (HOLD_AY_BP[0],
    HOLD_AY_BP[1]) -> partial hold factor, neither 0 nor 1."""
    from bmw.latcontroller import hold_factor
    f = hold_factor(14.0, 0.0035)
    assert 0.0 < f < 1.0
    # a_y = 14.0**2 * 0.0035 = 0.686; interp over [0.5, 0.9] -> (0.686-0.5)/0.4
    assert f == pytest.approx((14.0 * 14.0 * 0.0035 - 0.5) / 0.4, abs=1e-9)

  def test_boundary_bp0_is_zero(self):
    from bmw.latcontroller import hold_factor, HOLD_AY_BP
    v = 10.0
    kappa = HOLD_AY_BP[0] / (v * v)
    assert hold_factor(v, kappa) == pytest.approx(0.0)

  def test_boundary_bp1_is_one(self):
    from bmw.latcontroller import hold_factor, HOLD_AY_BP
    v = 10.0
    kappa = HOLD_AY_BP[1] / (v * v)
    assert hold_factor(v, kappa) == pytest.approx(1.0)


# ============================================================
# ISO accel/jerk cancel guard — REMOVED 2026-07-28 (lateral never gives up in
# a turn; a_y is bounded at the system level by speedlimitd's curve-speed
# capping). accel_guard_threshold() and the cancel machinery no longer exist,
# so the TestAccelGuardThreshold suite that used to sit here was removed. The
# no-cancel behaviour and the surviving cancel_tol boundary-hygiene path are
# covered by tests/test_latcontroller.py.
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
  """The Vehicle-panel rows this plugin contributes (ui.vehicle_settings)."""

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
    under any title."""
    titles = [r.title for r in self._rows(monkeypatch, param_dir)]
    assert titles == ['Temperature Overlay', 'Resume Button Repurposed']

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
    only an explicit '0' reads as off. The retired steering-push row used the
    opposite (`== '1'`) predicate, so this pins that they did not get mixed up
    when that row was removed from between them."""
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

  def test_state_tick_hook_is_gone(self):
    """Retirement pin (2026-08-14): `ui.state_tick` existed only to heartbeat
    the retired steering-push hot toggle. With that feature gone there is
    nothing to publish, so both the hook entry and its handler are deleted —
    a re-added manifest entry pointing at a missing function would break
    plugin load."""
    import json as _json
    import register
    with open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                           'plugin.json')) as f:
      manifest = _json.load(f)
    assert 'ui.state_tick' not in manifest['hooks']
    assert not hasattr(register, 'on_ui_state_tick')
