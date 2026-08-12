"""Shared mock helpers for BMW plugin tests.

Provides opendbc and cereal stubs so tests run without openpilot installed.
Key: @dataclass subclasses need REAL base classes (not MagicMock) because
the dataclass decorator iterates __mro__ which MagicMock doesn't support.
"""
import os
import sys
from dataclasses import dataclass
from enum import Enum, IntEnum
from unittest.mock import MagicMock

# bmw.latcontroller does `from config import read_plugin_param` inside
# on_lat_controller_init (function scope, review fix Important 2 — every
# other config.read_plugin_param consumer in this repo imports it at call
# time, not module scope, so a missing config.py just defaults the param off
# instead of failing the whole hook module's import) — on device the shared
# plugins/ dir (config.py, services.py, ...) is already on sys.path;
# replicate that here or the import raises ModuleNotFoundError the first
# time a test constructs a controller (and tests that monkeypatch
# config.read_plugin_param need the real `config` module importable at all).
# Lives here (not in test_latcontroller.py) because both test_latcontroller.py
# AND test_hooks.py import bmw.latcontroller, and both already import this
# module — a fix that lived in only one of them left the other broken when
# run in isolation (review fix, Minor 10). Same fix, same reason, as
# speedlimitd/tests/test_speedlimitd.py's _PLUGINS_DIR insert.
_PLUGIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PLUGIN_DIR not in sys.path:
  sys.path.insert(0, _PLUGIN_DIR)
_PLUGINS_DIR = os.path.dirname(_PLUGIN_DIR)
if _PLUGINS_DIR not in sys.path:
  sys.path.insert(0, _PLUGINS_DIR)


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

  # ISO comfort ceilings as real floats (not MagicMock). latcontroller.py no
  # longer imports these — the ISO accel/jerk cancel guard was removed
  # 2026-07-28 (a_y is bounded at the system level by speedlimitd) — but other
  # opendbc.car.lateral consumers may read them, so keep realistic stubs.
  mods['opendbc.car.lateral'].ISO_LATERAL_ACCEL = 3.0
  mods['opendbc.car.lateral'].ISO_LATERAL_JERK = 5.0

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
