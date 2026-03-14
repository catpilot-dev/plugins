"""Integration tests — require full openpilot environment (C3 or catpilot checkout).

Run with: python -m pytest plugins/bmw_e9x_e8x/tests/test_integration.py -v
Skip reason: 'opendbc.car not available' when running standalone.

The TestBmwUpstream class delegates to the upstream test_car_interfaces.py test body
via hypothesis.inner_test, so any assertions upstream adds in future releases (e.g.
v0.14.0) are automatically picked up without modifying this file.
"""
import os
import sys
import pytest

_PLUGIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

try:
  from opendbc.car import structs
  from opendbc.car.car_helpers import interfaces
  from opendbc.car.values import PLATFORMS
  HAS_OPENPILOT = True
except ImportError:
  HAS_OPENPILOT = False

pytestmark = pytest.mark.skipif(not HAS_OPENPILOT, reason="opendbc.car not available")

BMW_PLATFORMS = ['BMW_E82', 'BMW_E90']


# ============================================================
# Fixtures
# ============================================================

@pytest.fixture(scope="session", autouse=True)
def register_bmw():
  """Inject BMW into opendbc before any tests run."""
  if _PLUGIN_DIR not in sys.path:
    sys.path.insert(0, _PLUGIN_DIR)
  import importlib
  import register
  importlib.reload(register)  # re-run _register_interfaces() to patch interfaces


# ============================================================
# Upstream CarInterface tests — delegated, not copied
# ============================================================

class TestBmwUpstream:
  """Run upstream test_car_interfaces assertions for BMW platforms.

  Instead of duplicating the assertion block, we import the upstream test's
  inner function (unwrapped from hypothesis/parametrize decorators) and call
  it directly. This means any new checks upstream adds are automatically
  applied to BMW without modifying this file.
  """

  @staticmethod
  def _get_upstream_test_fn():
    from opendbc.car.tests.test_car_interfaces import TestCarInterfaces
    # hypothesis stores the unwrapped function at .hypothesis.inner_test
    return TestCarInterfaces.test_car_interfaces.hypothesis.inner_test

  @pytest.mark.parametrize("car_name", BMW_PLATFORMS)
  def test_car_interfaces(self, car_name):
    """Delegate to upstream test_car_interfaces with fuzzy inputs."""
    import hypothesis.strategies as st
    from hypothesis import Phase, given, settings as h_settings

    upstream_fn = self._get_upstream_test_fn()

    # Run the upstream test body through hypothesis with BMW-only parametrization
    @h_settings(max_examples=5, deadline=None,
                phases=(Phase.reuse, Phase.generate, Phase.shrink))
    @given(data=st.data())
    def _run(data):
      upstream_fn(self, car_name, data)

    _run()


# ============================================================
# BMW platform registration checks
# ============================================================

class TestBmwPlatformRegistration:
  """Verify BMW is properly integrated into opendbc globals."""

  def test_platforms_registered(self):
    for name in BMW_PLATFORMS:
      assert name in PLATFORMS, f"{name} not in PLATFORMS"

  def test_interfaces_registered(self):
    for name in BMW_PLATFORMS:
      assert name in interfaces, f"{name} not in interfaces"

  def test_torque_params_available(self):
    from opendbc.car.interfaces import get_torque_params
    params = get_torque_params()
    for name in BMW_PLATFORMS:
      assert name in params, f"{name} not in torque params"
      assert params[name]['LAT_ACCEL_FACTOR'] > 0

  def test_fingerprints_registered(self):
    from opendbc.car.fingerprints import _FINGERPRINTS
    for name in BMW_PLATFORMS:
      assert name in _FINGERPRINTS, f"{name} not in _FINGERPRINTS"

  def test_fw_versions_registered(self):
    from opendbc.car.fingerprints import FW_VERSIONS
    for name in BMW_PLATFORMS:
      assert name in FW_VERSIONS, f"{name} not in FW_VERSIONS"

  def test_fw_query_config_registered(self):
    from opendbc.car.fw_versions import FW_QUERY_CONFIGS
    assert 'bmw' in FW_QUERY_CONFIGS

  def test_model_to_brand_mapping(self):
    from opendbc.car.fw_versions import MODEL_TO_BRAND
    for name in BMW_PLATFORMS:
      assert MODEL_TO_BRAND.get(name) == 'bmw'


# ============================================================
# BMW-specific checks (analogous to test_toyota.py)
# ============================================================

class TestBmwSpecific:

  @pytest.mark.parametrize("car_name", BMW_PLATFORMS)
  def test_safety_config(self, car_name):
    """BMW must use SafetyModel.bmw."""
    CarInterface = interfaces[car_name]
    CP = CarInterface.get_params(car_name, {i: {} for i in range(7)}, [],
                                 alpha_long=False, is_release=False, docs=False)
    cp = CP.as_reader()
    assert len(cp.safetyConfigs) >= 1
    assert cp.safetyConfigs[0].safetyModel == structs.CarParams.SafetyModel.bmw

  @pytest.mark.parametrize("car_name", BMW_PLATFORMS)
  def test_brand_is_bmw(self, car_name):
    CarInterface = interfaces[car_name]
    CP = CarInterface.get_params(car_name, {i: {} for i in range(7)}, [],
                                 alpha_long=False, is_release=False, docs=False)
    assert CP.as_reader().brand == "bmw"

  @pytest.mark.parametrize("car_name", BMW_PLATFORMS)
  def test_longitudinal_control_enabled(self, car_name):
    CarInterface = interfaces[car_name]
    CP = CarInterface.get_params(car_name, {i: {} for i in range(7)}, [],
                                 alpha_long=False, is_release=False, docs=False)
    cp = CP.as_reader()
    assert cp.openpilotLongitudinalControl is True
    assert cp.radarUnavailable is True

  @pytest.mark.parametrize("car_name", BMW_PLATFORMS)
  def test_dbc_paths_resolve(self, car_name):
    """All DBC files referenced in platform config must exist on disk."""
    from bmw.values import DBC
    dbc_dict = DBC[car_name]
    for bus, path in dbc_dict.items():
      assert os.path.isfile(path), f"DBC file missing for bus {bus}: {path}"

  def test_vin_fuzzy_match_e90(self):
    from bmw.values import match_fw_to_car_fuzzy
    from opendbc.car.fingerprints import FW_VERSIONS
    result = match_fw_to_car_fuzzy({}, 'LBVPH18059SC20723', FW_VERSIONS)
    assert result == {'BMW_E90'}

  def test_vin_fuzzy_match_e82(self):
    from bmw.values import match_fw_to_car_fuzzy
    from opendbc.car.fingerprints import FW_VERSIONS
    result = match_fw_to_car_fuzzy({}, 'WBAUF1C50BVM12345', FW_VERSIONS)
    assert result == {'BMW_E82'}

  def test_vin_unknown_returns_empty(self):
    from bmw.values import match_fw_to_car_fuzzy
    from opendbc.car.fingerprints import FW_VERSIONS
    result = match_fw_to_car_fuzzy({}, 'WBAXX1C50BVM12345', FW_VERSIONS)
    assert result == set()
