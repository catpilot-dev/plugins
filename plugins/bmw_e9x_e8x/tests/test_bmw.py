"""Tests for BMW E9x/E8x plugin — VIN detection, CAN checksums, DBC paths."""
import pytest
from unittest.mock import MagicMock, patch
import sys
import os

# Add plugin dir to path so bmw package is importable
_PLUGIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PLUGIN_DIR not in sys.path:
  sys.path.insert(0, _PLUGIN_DIR)


@pytest.fixture(autouse=True)
def mock_opendbc(monkeypatch):
  """Mock opendbc imports so tests run without openpilot installed.

  Key: @dataclass subclasses need REAL base classes (not MagicMock) because
  the dataclass decorator iterates __mro__ which MagicMock doesn't support.
  """
  from dataclasses import dataclass
  from enum import Enum

  # Real stub classes needed for @dataclass inheritance in bmw/values.py
  @dataclass
  class CarSpecs:
    mass: float = 0.0
    wheelbase: float = 0.0
    steerRatio: float = 0.0
    tireStiffnessFactor: float = 0.0

  @dataclass
  class CarDocs:
    make_model_years: str = ""
    package: str = ""
    footnotes: list = None
    car_parts: object = None
    def init_make(self, CP): pass

  class CarFootnote:
    def __init__(self, text, column): self.text = text; self.column = column

  class Column:
    FSR_STEERING = 'fsr_steering'
    FSR_LONGITUDINAL = 'fsr_longitudinal'
    PACKAGE = 'package'
    AUTO_RESUME = 'auto_resume'
    HARDWARE = 'hardware'

  class CarHarness(Enum):
    custom = 'custom'

  class CarParts:
    @staticmethod
    def common(harnesses): return harnesses

  class Bus:
    pt = 0; chassis = 1; body = 2; alt = 3

  class Platforms(Enum):
    @classmethod
    def create_dbc_map(cls): return {}

  @dataclass
  class PlatformConfig:
    car_docs: list = None
    specs: object = None
    dbc_dict: dict = None
    def __init__(self, car_docs=None, specs=None, **kw):
      self.car_docs = car_docs; self.specs = specs; self.dbc_dict = kw.get('dbc_dict', {})

  class DbcDict(dict): pass

  mods = {}
  for mod in [
    'opendbc', 'opendbc.car', 'opendbc.car.structs', 'opendbc.car.docs_definitions',
    'opendbc.car.common', 'opendbc.car.common.conversions', 'opendbc.car.fw_query_definitions',
    'opendbc.can',
  ]:
    mods[mod] = MagicMock()

  # Wire up real classes for dataclass-inheriting code
  mods['opendbc.car'].Bus = Bus
  mods['opendbc.car'].Platforms = Platforms
  mods['opendbc.car'].CarSpecs = CarSpecs
  mods['opendbc.car'].PlatformConfig = PlatformConfig
  mods['opendbc.car'].DbcDict = DbcDict
  mods['opendbc.car'].STD_CARGO_KG = 136
  mods['opendbc.car.common.conversions'].Conversions.LB_TO_KG = 0.453592
  mods['opendbc.car.docs_definitions'].CarDocs = CarDocs
  mods['opendbc.car.docs_definitions'].CarFootnote = CarFootnote
  mods['opendbc.car.docs_definitions'].CarHarness = CarHarness
  mods['opendbc.car.docs_definitions'].CarParts = CarParts
  mods['opendbc.car.docs_definitions'].Column = Column

  for mod_name, mod_mock in mods.items():
    monkeypatch.setitem(sys.modules, mod_name, mod_mock)


# ============================================================
# VIN Detection
# ============================================================

class TestVINDetection:
  def _get_match_fn(self):
    import importlib
    import bmw.values as mod
    importlib.reload(mod)
    return mod.match_fw_to_car_fuzzy

  def test_e90_vin(self, mock_opendbc):
    match = self._get_match_fn()
    # Real E90 VIN — model code PH1 at positions 4-6
    result = match({}, 'LBVPH18059SC20723', {})
    assert result == {'BMW_E90'}

  def test_e82_vin(self, mock_opendbc):
    match = self._get_match_fn()
    result = match({}, 'WBAUF1C50BVM12345', {})
    assert result == {'BMW_E82'}

  def test_all_e90_codes(self, mock_opendbc):
    match = self._get_match_fn()
    e90_codes = ['PH1', 'PH2', 'PK1', 'PK2', 'PM1', 'PM2', 'PN1']
    for code in e90_codes:
      vin = f'LBV{code}8059SC20723'
      result = match({}, vin, {})
      assert result == {'BMW_E90'}, f"Failed for model code {code}"

  def test_all_e82_codes(self, mock_opendbc):
    match = self._get_match_fn()
    e82_codes = ['UF1', 'UF2', 'UH1']
    for code in e82_codes:
      vin = f'WBA{code}C50BVM12345'
      result = match({}, vin, {})
      assert result == {'BMW_E82'}, f"Failed for model code {code}"

  def test_unknown_model_code(self, mock_opendbc):
    match = self._get_match_fn()
    result = match({}, 'WBAXX1C50BVM12345', {})
    assert result == set()

  def test_empty_vin(self, mock_opendbc):
    match = self._get_match_fn()
    assert match({}, '', {}) == set()
    assert match({}, None, {}) == set()

  def test_short_vin(self, mock_opendbc):
    match = self._get_match_fn()
    assert match({}, 'LBVPH', {}) == set()

  def test_offline_fw_filtering(self, mock_opendbc):
    """When offline_fw_versions provided, only return if model is in it."""
    match = self._get_match_fn()
    # E90 detected but not in offline versions
    result = match({}, 'LBVPH18059SC20723', {'BMW_E82': {}})
    assert result == set()
    # E90 detected and in offline versions
    result = match({}, 'LBVPH18059SC20723', {'BMW_E90': {}})
    assert result == {'BMW_E90'}


# ============================================================
# CAN Checksums
# ============================================================

class TestCANChecksums:
  def _get_checksums(self):
    from bmw.bmwcan import calc_checksum_8bit, calc_checksum_4bit, calc_checksum_cruise
    return calc_checksum_8bit, calc_checksum_4bit, calc_checksum_cruise

  def test_checksum_8bit_zero_data(self, mock_opendbc):
    calc_8bit, _, _ = self._get_checksums()
    result = calc_8bit(bytearray([0, 0, 0, 0]), 0)
    assert result == 0

  def test_checksum_8bit_with_msg_id(self, mock_opendbc):
    calc_8bit, _, _ = self._get_checksums()
    # msg_id 0xA8 with zero data
    result = calc_8bit(bytearray([0, 0, 0, 0]), 0xA8)
    assert result == 0xA8

  def test_checksum_8bit_overflow_wraps(self, mock_opendbc):
    calc_8bit, _, _ = self._get_checksums()
    # 0xFF * 4 = 0x3FC, msg_id = 0 → (0xFC + 0x03) & 0xFF = 0xFF
    result = calc_8bit(bytearray([0xFF, 0xFF, 0xFF, 0xFF]), 0)
    assert result == 0xFF

  def test_checksum_8bit_carry(self, mock_opendbc):
    calc_8bit, _, _ = self._get_checksums()
    # Test carry from upper byte: sum > 0xFF
    result = calc_8bit(bytearray([0x80, 0x80]), 0)
    assert result == (0x00 + 0x01) & 0xFF  # 0x100 → carry 1 + 0x00 = 1
    assert result == 1

  def test_checksum_4bit(self, mock_opendbc):
    _, calc_4bit, _ = self._get_checksums()
    result = calc_4bit(bytearray([0, 0, 0, 0]), 0)
    assert result == 0

  def test_checksum_4bit_nibble_wrap(self, mock_opendbc):
    _, calc_4bit, _ = self._get_checksums()
    result = calc_4bit(bytearray([0, 0, 0, 0]), 0x130)
    # 0x130 → (0x30 + 0x01) = 0x31 → (0x1 + 0x3) = 0x4
    assert result == 4

  def test_checksum_cruise_uses_zero_init(self, mock_opendbc):
    calc_8bit, _, calc_cruise = self._get_checksums()
    data = bytearray([0x10, 0x20, 0x30])
    assert calc_cruise(data) == calc_8bit(data, 0)

  def test_checksum_8bit_deterministic(self, mock_opendbc):
    calc_8bit, _, _ = self._get_checksums()
    data = bytearray([0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE])
    r1 = calc_8bit(data, 0xA8)
    r2 = calc_8bit(data, 0xA8)
    assert r1 == r2


# ============================================================
# Steering / Cruise Enums
# ============================================================

class TestEnums:
  def test_steering_modes(self, mock_opendbc):
    from bmw.bmwcan import SteeringModes
    assert SteeringModes.Off.value == 0
    assert SteeringModes.TorqueControl.value == 1
    assert SteeringModes.AngleControl.value == 2
    assert SteeringModes.SoftOff.value == 3

  def test_cruise_stalk_values(self, mock_opendbc):
    from bmw.bmwcan import CruiseStalk
    expected = {'plus1', 'plus5', 'minus1', 'minus5', 'cancel', 'resume', 'cancel_lever_up'}
    actual = {s.value for s in CruiseStalk}
    assert actual == expected


# ============================================================
# DBC Path Resolution
# ============================================================

class TestDBCPaths:
  def test_dbc_dict_has_all_buses(self, mock_opendbc):
    """All Bus entries (pt, chassis, body, alt) resolve to plugin-local DBC files."""
    import importlib
    import bmw.values as mod
    importlib.reload(mod)
    assert os.path.isabs(mod.PLUGIN_DBC_DIR)
    dbc_dict = mod.BmwPlatformConfig([], mod.CarSpecs()).dbc_dict
    for bus_key in ['pt', 'chassis', 'body', 'alt']:
      bus_val = getattr(mod.Bus, bus_key) if hasattr(mod.Bus, bus_key) else bus_key
      path = dbc_dict.get(bus_val)
      if path is None:
        path = dbc_dict.get({'pt': 0, 'chassis': 1, 'body': 2, 'alt': 3}[bus_key])
      assert path is not None, f"Bus.{bus_key} not in dbc_dict"
      assert mod.PLUGIN_DBC_DIR in path, f"Bus.{bus_key} path not in plugin dir: {path}"

  def test_ocelot_controls_dbc_exists(self, mock_opendbc):
    """ocelot_controls.dbc exists in plugin dbc directory."""
    import importlib
    import bmw.values as mod
    importlib.reload(mod)
    ocelot_path = os.path.join(mod.PLUGIN_DBC_DIR, 'ocelot_controls.dbc')
    assert os.path.exists(ocelot_path), f"Missing: {ocelot_path}"

  def test_bmw_dbc_exists(self, mock_opendbc):
    """bmw_e9x_e8x.dbc exists in plugin dbc directory."""
    import importlib
    import bmw.values as mod
    importlib.reload(mod)
    bmw_path = os.path.join(mod.PLUGIN_DBC_DIR, 'bmw_e9x_e8x.dbc')
    assert os.path.exists(bmw_path), f"Missing: {bmw_path}"


# ============================================================
# Platform Config
# ============================================================

class TestPlatformConfig:
  def test_controller_params(self, mock_opendbc):
    from bmw.values import CarControllerParams
    p = CarControllerParams(None)
    assert p.STEER_MAX == 12
    assert p.STEER_STEP == 1
    assert p.STEER_DELTA_UP == 0.1
    assert p.STEER_DELTA_DOWN == 1.0

  def test_bmw_flags(self, mock_opendbc):
    from bmw.values import BmwFlags
    # Flags are distinct powers of 2
    assert BmwFlags.STEPPER_SERVO_CAN == 1
    assert BmwFlags.NORMAL_CRUISE_CONTROL == 2
    assert BmwFlags.DYNAMIC_CRUISE_CONTROL == 4
    # Can combine flags
    combined = BmwFlags.STEPPER_SERVO_CAN | BmwFlags.DYNAMIC_CRUISE_CONTROL
    assert BmwFlags.STEPPER_SERVO_CAN in combined
    assert BmwFlags.NORMAL_CRUISE_CONTROL not in combined

  def test_can_bus_assignments(self, mock_opendbc):
    from bmw.values import CanBus
    assert CanBus.PT_CAN == 0
    assert CanBus.SERVO_CAN == 1
    assert CanBus.F_CAN == 1
    assert CanBus.AUX_CAN == 2
