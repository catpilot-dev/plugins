"""Shared mock helpers for BMW plugin tests.

Provides opendbc and cereal stubs so tests run without openpilot installed.
Key: @dataclass subclasses need REAL base classes (not MagicMock) because
the dataclass decorator iterates __mro__ which MagicMock doesn't support.
"""
from dataclasses import dataclass
from enum import Enum, IntEnum
from unittest.mock import MagicMock


# ============================================================
# opendbc stubs — real classes for @dataclass inheritance
# ============================================================

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


# ============================================================
# cereal stubs — lane change / desire enums
# ============================================================

class LaneChangeState(IntEnum):
  off = 0
  preLaneChange = 1
  laneChangeStarting = 2
  laneChangeFinishing = 3


class LaneChangeDirection(IntEnum):
  none = 0
  left = 1
  right = 2


class Desire(IntEnum):
  none = 0
  laneChangeLeft = 1
  laneChangeRight = 2


# ============================================================
# Module patching helpers
# ============================================================

def make_opendbc_mocks() -> dict:
  """Build a dict of module_name → mock/stub for opendbc packages."""
  mods = {}
  for mod in [
    'opendbc', 'opendbc.car', 'opendbc.car.structs', 'opendbc.car.docs_definitions',
    'opendbc.car.common', 'opendbc.car.common.conversions', 'opendbc.car.fw_query_definitions',
    'opendbc.car.interfaces', 'opendbc.car.lateral', 'opendbc.car.fingerprints',
    'opendbc.car.fw_versions', 'opendbc.car.values', 'opendbc.car.car_helpers',
    'opendbc.can',
  ]:
    mods[mod] = MagicMock()

  # car_helpers.interfaces must be a real dict for monkey-patching
  mods['opendbc.car.car_helpers'].interfaces = {}

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

  return mods


def make_cereal_mocks() -> dict:
  """Build a dict of module_name → mock/stub for cereal packages."""
  log_mock = MagicMock()
  log_mock.LaneChangeState = LaneChangeState
  log_mock.LaneChangeDirection = LaneChangeDirection
  log_mock.Desire = Desire

  cereal_mock = MagicMock()
  cereal_mock.log = log_mock
  messaging_mock = MagicMock()

  return {
    'cereal': cereal_mock,
    'cereal.log': log_mock,
    'cereal.messaging': messaging_mock,
  }


def install_all_mocks(monkeypatch):
  """Install both opendbc and cereal mocks into sys.modules."""
  for mod_name, mod_mock in make_opendbc_mocks().items():
    monkeypatch.setitem(__import__('sys').modules, mod_name, mod_mock)
  for mod_name, mod_mock in make_cereal_mocks().items():
    monkeypatch.setitem(__import__('sys').modules, mod_name, mod_mock)
