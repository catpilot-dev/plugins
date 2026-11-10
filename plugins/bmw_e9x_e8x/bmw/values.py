import os
from dataclasses import dataclass, field
from enum import Enum, IntFlag
from opendbc.car import Bus, Platforms, CarSpecs, PlatformConfig, DbcDict
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarFootnote, CarHarness, CarDocs, CarParts, Column
from opendbc.car.fw_query_definitions import LiveFwVersions, OfflineFwVersions, FwQueryConfig

# Plugin-local DBC directory (resolved at import time)
_PLUGIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PLUGIN_DBC_DIR = os.path.join(_PLUGIN_DIR, 'dbc')

class CarControllerParams:  # steer torque limits; controls run at 100 Hz
  STEER_STEP = 1 # 100Hz
  STEER_MAX = 12  # Nm
  STEER_DELTA_UP = 0.1       # Nm/10ms
  STEER_DELTA_DOWN = 0.1     # Nm/10ms
  # No distance-to-measured limit in apply_dist_to_meas_limits: the panda's
  # check is mirrored by CarController._servo_ceiling instead.
  STEER_ERROR_MAX = 999

  def __init__(self, CP):
    pass


class BmwSafetyFlags(IntFlag):
  # safetyParam bits — must match safety/bmw.h
  STEPPER_SERVO = 1  # BMW_PARAM_STEPPER_SERVO: RX-check 0x22F, allow 0x22E


class BmwFlags(IntFlag):
  # Detected Flags
  STEPPER_SERVO_CAN = 2 ** 0
  NORMAL_CRUISE_CONTROL = 2 ** 1          # CC  $540
  DYNAMIC_CRUISE_CONTROL = 2 ** 2         # DCC $544
  ACTIVE_CRUISE_CONTROL = 2 ** 3          # ACC $541 - LDM and ACC sensor - #! not supported
  ACTIVE_CRUISE_CONTROL_NO_ACC = 2 ** 4   # no ACC module - DSC, DME, KOMBI coded to $541, LDM coded to $544
  ACTIVE_CRUISE_CONTROL_NO_LDM = 2 ** 5   # no LDM/ACC - DSC, DME, KOMBI coded to $541


class CruiseSettings:
  CLUSTER_OFFSET = 2 # kph
  MIN_SPEED_BUFFER = 1.0  # km/h - add to minEnableSpeed to avoid disengagement
  MIN_ENABLE_SPEED_KPH = 30.  # CC and DCC both refuse to engage below this

class CanBus:
  # Panda bus = harness CAN pair: CAN0 PT-CAN, CAN1 F-CAN. DESIGN.md: "CAN bus layout".
  PT_CAN = 0
  SERVO_CAN = 1  # required for steering (STEPPER_SERVO can be on this bus)
  F_CAN = 1  # required for DYNAMIC_CRUISE_CONTROL or optional for logging
  # Alternative bus for STEPPER_SERVO messages (matches BMW_AUX_CAN in bmw.h).
  # CAN2 is the harness relay's camera-side pair: it mirrors PT-CAN whenever the
  # relay is at rest, so it is a standalone servo bus only with that path removed.
  AUX_CAN = 2


class Footnote(Enum):
  StepperServoCAN = CarFootnote(
    "Requires StepperServoCAN",
    Column.FSR_STEERING)
  DCC = CarFootnote(
    "Minimum speed with CC or DCC is 30 kph",
    Column.FSR_LONGITUDINAL)
  CC = CarFootnote(
    "Normal cruise control should work but was not tested in a while. Code in DCC instead or provide a fix",
    Column.PACKAGE)
  ACC = CarFootnote(
    "ACC is required. Also LDM module to take over when OP is off.",
    Column.AUTO_RESUME)
  DIY = CarFootnote(
    "For CC and DCC only a diy USB-C and a resistor is required or a harness box DIY connector",
    Column.HARDWARE)


@dataclass
class BmwCarDocs(CarDocs):
  package: str = "Cruise Control - VO540, VO544, VO541"
  footnotes: list[Enum] = field(default_factory=lambda: [Footnote.StepperServoCAN, Footnote.DCC, Footnote.CC, Footnote.ACC, Footnote.DIY])

  def init_make(self, CP: CarParams):
      self.car_parts = CarParts.common([CarHarness.custom])


@dataclass
class BmwPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: os.path.join(PLUGIN_DBC_DIR, 'bmw_e9x_e8x.dbc'),
    Bus.chassis: os.path.join(PLUGIN_DBC_DIR, 'bmw_e9x_e8x.dbc'),
    Bus.body: os.path.join(PLUGIN_DBC_DIR, 'bmw_e9x_e8x.dbc'),
    Bus.alt: os.path.join(PLUGIN_DBC_DIR, 'ocelot_controls.dbc'),
    })


class CAR(Platforms):
  BMW_E82 = BmwPlatformConfig(
    [BmwCarDocs("BMW E82 2004-13")],
    CarSpecs(wheelbase=2.66, steerRatio=16.00, tireStiffnessFactor=0.8)
  )
  BMW_E90 = BmwPlatformConfig(
    [BmwCarDocs("BMW E90 2005-11")],
    CarSpecs(wheelbase=2.76, steerRatio=19.0, tireStiffnessFactor=0.8)
  )


DBC = CAR.create_dbc_map()


def match_fw_to_car_fuzzy(live_fw_versions: LiveFwVersions, vin: str, offline_fw_versions: OfflineFwVersions) -> set[str]:
  """BMW VIN-based model detection to distinguish E82 from E90"""
  if not vin or len(vin) != 17:
    return set()

  # BMW VIN structure: positions 4-6 contain model code
  model_code = vin[3:6]

  # BMW model code mapping for E8x/E9x series
  vin_to_model = {
    # E82 1-Series Coupe/Convertible
    'UF1': 'BMW_E82', 'UF2': 'BMW_E82', 'UH1': 'BMW_E82',
    # E90/E91/E92/E93 3-Series (all use E90 fingerprint)
    'PH1': 'BMW_E90', 'PH2': 'BMW_E90', 'PK1': 'BMW_E90',
    'PK2': 'BMW_E90', 'PM1': 'BMW_E90', 'PM2': 'BMW_E90', 'PN1': 'BMW_E90',
  }

  detected_model = vin_to_model.get(model_code)

  # Only return models that exist in offline_fw_versions (if provided)
  if detected_model and offline_fw_versions:
    if detected_model in offline_fw_versions:
      return {detected_model}
    else:
      return set()
  elif detected_model:
    return {detected_model}
  else:
    return set()


FW_QUERY_CONFIG = FwQueryConfig(
  requests=[],  # No firmware queries needed for VIN-based detection
  fw_version_regex=br"\x00BMW_E(?:82|90)_DUMMY",  # fingerprints.py's placeholder entries
  match_fw_to_car_fuzzy=match_fw_to_car_fuzzy,
  extra_ecus=[],
  non_essential_ecus={},
)
