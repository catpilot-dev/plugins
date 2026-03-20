#!/usr/bin/env python3
from opendbc.car import structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car import get_safety_config
from bmw.values import CanBus, BmwFlags
from opendbc.car.interfaces import CarInterfaceBase
from bmw.carcontroller import CarController
from bmw.carstate import CarState

TransmissionType = structs.CarParams.TransmissionType


def detect_stepper_override(steer_cmd, steer_act, v_ego, centering_coeff, steer_friction_torque):
  release_angle = steer_friction_torque / (max(v_ego, 1) ** 2 * centering_coeff)

  override = False
  margin_value = 1
  if abs(steer_cmd) > release_angle:
    if steer_cmd > 0:
      override |= steer_act - steer_cmd > margin_value
      override |= steer_act < 0
    else:
      override |= steer_act - steer_cmd < -margin_value
      override |= steer_act > 0
  return override


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  def __init__(self, CP, *args, **kwargs):
    super().__init__(CP, *args, **kwargs)

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, alpha_long, is_release, docs):
    ret.brand = "bmw"

    # Detect cruise type by ECU status messages first (authoritative),
    # then fall back to stalk bus location if ECU is slow to wake.
    #   NCC ($540): CruiseControlStatus 0x200 on PT_CAN
    #   DCC ($544): CruiseControlStatus 0x193 on PT_CAN
    # Stalk (0x194) can appear on both buses, so it's only used as fallback.
    has_ncc_ecu = 0x200 in fingerprint.get(CanBus.PT_CAN, {})
    has_dcc_ecu = 0x193 in fingerprint.get(CanBus.PT_CAN, {})
    stalk_on_fcan = 0x194 in fingerprint.get(CanBus.F_CAN, {})
    has_ldm = 0x0D5 in fingerprint.get(CanBus.PT_CAN, {})

    if (0x22F in fingerprint.get(CanBus.SERVO_CAN, {}) or
        0x22F in fingerprint.get(CanBus.AUX_CAN, {})):
      ret.flags |= BmwFlags.STEPPER_SERVO_CAN.value

    ret.openpilotLongitudinalControl = True
    ret.radarUnavailable = True
    ret.pcmCruise = False

    ret.autoResumeSng = False
    if has_ncc_ecu:
      ret.flags |= BmwFlags.NORMAL_CRUISE_CONTROL.value
    elif has_dcc_ecu or stalk_on_fcan:
      if not has_ldm:
        ret.flags |= BmwFlags.DYNAMIC_CRUISE_CONTROL.value
      else:
        ret.flags |= BmwFlags.ACTIVE_CRUISE_CONTROL_NO_ACC.value
        ret.autoResumeSng = True
    else:
      ret.flags |= BmwFlags.ACTIVE_CRUISE_CONTROL_NO_LDM.value
      ret.autoResumeSng = True

    if 0xb8 in fingerprint.get(CanBus.PT_CAN, {}) or 0xb5 in fingerprint.get(CanBus.PT_CAN, {}):
      ret.transmissionType = TransmissionType.automatic
    else:
      ret.transmissionType = TransmissionType.manual

    if 0xbc in fingerprint.get(CanBus.PT_CAN, {}):
      ret.steerRatio = 18.5

    if ret.flags & BmwFlags.DYNAMIC_CRUISE_CONTROL:
      ret.minEnableSpeed = 30. * CV.KPH_TO_MS
    if ret.flags & BmwFlags.NORMAL_CRUISE_CONTROL:
      ret.minEnableSpeed = 30. * CV.KPH_TO_MS

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.bmw)]
    ret.safetyConfigs[0].safetyParam = 0

    ret.steerControlType = structs.CarParams.SteerControlType.torque
    ret.steerActuatorDelay = 0.2
    ret.steerLimitTimer = 0.4

    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning, steering_angle_deadzone_deg=0.0)

    ret.longitudinalActuatorDelay = 0.3

    ret.centerToFront = ret.wheelbase * 0.44

    ret.startAccel = 0.0

    return ret
