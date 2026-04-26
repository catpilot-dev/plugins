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
