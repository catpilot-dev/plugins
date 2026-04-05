import numpy as np
from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, structs, create_button_events
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from bmw.values import DBC, CanBus, BmwFlags, CruiseSettings
import cereal.messaging as messaging

ButtonType = structs.CarState.ButtonEvent.Type

# Resume button hold duration thresholds (in frames at 100Hz = 10ms per frame)
# Short press minimum: 10ms (1 frame)
# Long press threshold: 500ms (49 frames) — triggers gap adjust / personality cycle
RESUME_SHORT_PRESS_MIN_FRAMES = 1 
RESUME_LONG_PRESS_FRAMES = 49


_sl_pub = None


def toggle_speed_limit_confirm():
  """Send toggle command to speedlimitd via plugin bus."""
  global _sl_pub
  try:
    if _sl_pub is None:
      from openpilot.selfdrive.plugins.plugin_bus import PluginPub
      _sl_pub = PluginPub('speedlimit_cmd_car')
    _sl_pub.send({'action': 'toggle_confirm'})
  except Exception:
    pass


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.shifter_values = can_define.dv["TransmissionDataDisplay"]['ShiftLeverPosition']
    self.gas_kickdown = False

    self.cluster_min_speed = CruiseSettings.CLUSTER_OFFSET

    self.is_metric = None
    self.cruise_stalk_speed = 0
    self.cruise_stalk_resume = False
    self.cruise_stalk_cancel = False
    self.cruise_stalk_cancel_up = False
    self.cruise_stalk_cancel_dn = False
    self.cruise_stalk_counter = 0
    self.prev_cruise_stalk_speed = 0
    self.prev_cruise_stalk_resume = self.cruise_stalk_resume
    self.prev_cruise_stalk_cancel = self.cruise_stalk_cancel
    self.prev_cruise_enabled = False  # Track previous openpilot cruise state for resume button logic
    self.resume_button_hold_frames = 0  # Track how many frames resume button has been held (v4 duration-based logic)
    self.steer_fault_counter = 0  # Debounce: consecutive frames with raw fault bits set
    self.steer_angle_offset = self._load_steer_angle_offset()
    self._offset_sub = None

    self.right_blinker_pressed = False
    self.left_blinker_pressed = False
    self.other_buttons = False
    self.prev_other_buttons = False
    self.prev_gas_pressed = False
    self.cruise_state_enabled = False
    self.dtc_mode = False

    # Subscribe to radarState, liveDelay
    self.sm = messaging.SubMaster(['radarState', 'liveDelay'])
    from openpilot.selfdrive.plugins.plugin_bus import PluginSub
    self._sl_sub = PluginSub(['speedLimitState'])
    self._sl_data = None

  def update(self, can_parsers) -> structs.CarState:
    cp_PT = can_parsers[Bus.pt]
    cp_F = can_parsers[Bus.body]
    cp_aux = can_parsers[Bus.alt]

    # Update offset from look_ahead plugin bus (published at 1 Hz)
    self._update_steer_angle_offset()

    ret = structs.CarState()

    # set these prev states at the beginning because they are used outside the update()
    self.prev_cruise_stalk_speed = self.cruise_stalk_speed
    self.prev_cruise_stalk_resume = self.cruise_stalk_resume
    self.prev_cruise_stalk_cancel = self.cruise_stalk_cancel

    ret.doorOpen = False
    ret.seatbeltUnlatched = False

    ret.brakePressed = cp_PT.vl["EngineAndBrake"]['BrakePressed'] != 0
    ret.parkingBrake = cp_PT.vl["Status_contact_handbrake"]["Handbrake_pulled_up"] != 0
    ret.gasPressed = cp_PT.vl['AccPedal']["AcceleratorPedalPressed"] != 0 or cp_PT.vl['AccPedal']["KickDownPressed"] != 0
    self.gas_kickdown = cp_PT.vl['AccPedal']["KickDownPressed"] != 0

    ret.vEgoRaw = cp_PT.vl['Speed']["VehicleSpeed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.vEgoCluster = ret.vEgo + CruiseSettings.CLUSTER_OFFSET * CV.KPH_TO_MS
    ret.standstill = not cp_PT.vl['Speed']["MovingForward"] and not cp_PT.vl['Speed']["MovingReverse"]
    ret.yawRate = cp_PT.vl['Speed']["YawRate"] * CV.DEG_TO_RAD
    ret.steeringRateDeg = cp_PT.vl["SteeringWheelAngle"]['SteeringSpeed']
    can_gear = int(cp_PT.vl["TransmissionDataDisplay"]['ShiftLeverPosition'])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    blinker_on = cp_PT.vl["TurnSignals"]['TurnSignalActive'] != 0 and cp_PT.vl["TurnSignals"]['TurnSignalIdle'] == 0
    ret.leftBlinker = blinker_on and cp_PT.vl["TurnSignals"]['LeftTurn'] != 0
    ret.rightBlinker = blinker_on and cp_PT.vl["TurnSignals"]['RightTurn'] != 0
    self.right_blinker_pressed = not blinker_on and cp_PT.vl["TurnSignals"]['RightTurn'] != 0
    self.left_blinker_pressed = not blinker_on and cp_PT.vl["TurnSignals"]['LeftTurn'] != 0

    self.dtc_mode = cp_PT.vl['StatusDSC_KCAN']['DTC_on'] != 0

    self.other_buttons = \
      cp_PT.vl["SteeringButtons"]['Volume_DOWN'] != 0 or cp_PT.vl["SteeringButtons"]['Volume_UP'] != 0 or \
      cp_PT.vl["SteeringButtons"]['Previous_down'] != 0 or cp_PT.vl["SteeringButtons"]['Next_up'] != 0 or \
      cp_PT.vl["SteeringButtons"]['VoiceControl'] != 0 or \
      self.prev_gas_pressed and not ret.gasPressed

    ret.steeringPressed = cp_PT.vl["SteeringButtons"]['VoiceControl'] != 0 or ret.gasPressed
    if ret.steeringPressed and ret.leftBlinker:
      ret.steeringTorque = 1
    elif ret.steeringPressed and ret.rightBlinker:
      ret.steeringTorque = -1
    else:
      ret.steeringTorque = 0

    ret.espDisabled = cp_PT.vl['StatusDSC_KCAN']['DSC_full_off'] != 0
    ret.cruiseState.available = not ret.espDisabled
    ret.cruiseState.nonAdaptive = False

    cruise_control_stal_msg = cp_PT.vl["CruiseControlStalk"]
    if self.CP.flags & BmwFlags.DYNAMIC_CRUISE_CONTROL:
      ret.steeringAngleDeg = cp_F.vl['SteeringWheelAngle_DSC']['SteeringPosition'] - self.steer_angle_offset
      ret.cruiseState.speed = cp_PT.vl["DynamicCruiseControlStatus"]['CruiseControlSetpointSpeed'] * (CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS)
      ret.cruiseState.enabled = cp_PT.vl["DynamicCruiseControlStatus"]['CruiseActive'] != 0
      cruise_control_stal_msg = cp_F.vl["CruiseControlStalk"]
    elif self.CP.flags & BmwFlags.NORMAL_CRUISE_CONTROL:
      ret.steeringAngleDeg = cp_PT.vl['SteeringWheelAngle']['SteeringPosition'] - self.steer_angle_offset
      ret.cruiseState.speed = cp_PT.vl["CruiseControlStatus"]['CruiseControlSetpointSpeed'] * (CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS)
      ret.cruiseState.enabled = cp_PT.vl["CruiseControlStatus"]['CruiseControlActiveFlag'] != 0
    ret.cruiseState.speedCluster = ret.cruiseState.speed + CruiseSettings.CLUSTER_OFFSET * CV.KPH_TO_MS
    if cruise_control_stal_msg['plus1'] != 0:
      self.cruise_stalk_speed = 1
    elif cruise_control_stal_msg['minus1'] != 0:
      self.cruise_stalk_speed = -1
    elif cruise_control_stal_msg['plus5'] != 0:
      self.cruise_stalk_speed = 5
    elif cruise_control_stal_msg['minus5'] != 0:
      self.cruise_stalk_speed = -5
    else:
      self.cruise_stalk_speed = 0
    self.cruise_stalk_resume = cruise_control_stal_msg['resume'] != 0
    self.cruise_stalk_cancel = cruise_control_stal_msg['cancel'] != 0
    self.cruise_stalk_cancel_up = cruise_control_stal_msg['cancel_lever_up'] != 0
    self.cruise_stalk_counter = cruise_control_stal_msg['Counter_0x194']
    self.cruise_stalk_cancel_dn = self.cruise_stalk_cancel and not self.cruise_stalk_cancel_up

    if self.is_metric is None and ret.cruiseState.enabled and ret.vEgo > 5:
      speed_ratio = ret.cruiseState.speed / ret.vEgo
      if 0.8 < speed_ratio < 1.2:
        self.is_metric = False
      elif 0.8 * CV.MPH_TO_KPH < speed_ratio < 1.2 * CV.MPH_TO_KPH:
        self.is_metric = True
        cruise_msg = "DynamicCruiseControlStatus" if (self.CP.flags & BmwFlags.DYNAMIC_CRUISE_CONTROL) else "CruiseControlStatus"
        ret.cruiseState.speed = cp_PT.vl[cruise_msg]['CruiseControlSetpointSpeed'] * CV.KPH_TO_MS
      else:
        ret.accFaulted = True

    ret.genericToggle = self.dtc_mode

    # Temps are not in stock CarState capnp — share via plugin bus for UI overlay
    # Publish at 0.2 Hz (every 5s) — temps change slowly
    import time
    now = time.monotonic()
    if not hasattr(self, '_last_temp_write') or now - self._last_temp_write >= 5.0:
      self._last_temp_write = now
      try:
        if not hasattr(self, '_temp_pub'):
          from openpilot.selfdrive.plugins.plugin_bus import PluginPub
          self._temp_pub = PluginPub('bmw_temps')
        self._temp_pub.send({
          'coolant': round(cp_PT.vl["EngineData"]["TEMP_ENG"]),
          'oil': round(cp_PT.vl["EngineData"]["TEMP_EOI"]),
        })
      except Exception:
        pass

    if self.CP.flags & BmwFlags.STEPPER_SERVO_CAN:
      ret.steeringTorqueEps = cp_aux.vl['STEERING_STATUS']['STEERING_TORQUE']
      ret.steeringAngleOffsetDeg = ret.steeringAngleDeg - cp_aux.vl['STEERING_STATUS']['STEERING_ANGLE']
      raw_fault = (int(cp_aux.vl['STEERING_STATUS']['DEBUG_STATES']) & 0xE0) != 0 or \
                  (int(cp_aux.vl['STEERING_STATUS']['CONTROL_STATUS']) & 0x4) != 0
      self.steer_fault_counter = self.steer_fault_counter + 1 if raw_fault else 0
      ret.steerFaultTemporary = self.steer_fault_counter >= 10

    self.prev_gas_pressed = ret.gasPressed

    # Resume button duration-based logic (v6)
    resume_button_events = []

    if self.cruise_stalk_resume:
      if not self.prev_cruise_stalk_resume:
        self.resume_button_hold_frames = 0
      else:
        self.resume_button_hold_frames += 1

    if not self.cruise_stalk_resume and self.prev_cruise_stalk_resume:
      if self.resume_button_hold_frames >= RESUME_LONG_PRESS_FRAMES:
        resume_button_events.append(structs.CarState.ButtonEvent(
          pressed=True,
          type=ButtonType.gapAdjustCruise
        ))
        resume_button_events.append(structs.CarState.ButtonEvent(
          pressed=False,
          type=ButtonType.gapAdjustCruise
        ))
      elif self.resume_button_hold_frames < RESUME_SHORT_PRESS_MIN_FRAMES:
        pass  
      elif self.cruise_state_enabled:
        # Short press while engaged: toggle speed limit confirm
        msg = self._sl_sub.drain('speedLimitState')
        if msg is not None:
          _, self._sl_data = msg
        toggle_speed_limit_confirm()
      else:
        resume_button_events.append(structs.CarState.ButtonEvent(
          pressed=True,
          type=ButtonType.resumeCruise
        ))
        resume_button_events.append(structs.CarState.ButtonEvent(
          pressed=False,
          type=ButtonType.resumeCruise
        ))

    ret.buttonEvents = [
      *create_button_events(self.cruise_stalk_speed > 0, self.prev_cruise_stalk_speed > 0, {1: ButtonType.accelCruise}),
      *create_button_events(self.cruise_stalk_speed < 0, self.prev_cruise_stalk_speed < 0, {1: ButtonType.decelCruise}),
      *create_button_events(self.cruise_stalk_cancel, self.prev_cruise_stalk_cancel, {1: ButtonType.cancel}),
      *create_button_events(self.other_buttons, self.prev_other_buttons, {1: ButtonType.altButton2}),
      *resume_button_events
      ]

    self.cruise_state_enabled = ret.cruiseState.enabled
    self.prev_cruise_enabled = ret.cruiseState.enabled
    self.prev_other_buttons = self.other_buttons
    return ret

  def update_button_enable(self, buttonEvents: list[structs.CarState.ButtonEvent]):
    if self.cruise_state_enabled and not self.out.cruiseState.enabled:
      return True
    return False

  @staticmethod
  def _load_steer_angle_offset():
    """Load persisted offset from look_ahead plugin data. Falls back to 0."""
    try:
      import os
      path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                          '..', 'look_ahead', 'data', 'SteerAngleOffset')
      with open(os.path.normpath(path)) as f:
        return float(f.read().strip())
    except (FileNotFoundError, OSError, ValueError):
      return 0.0

  def _update_steer_angle_offset(self):
    """Update offset from look_ahead plugin bus (steer_angle_offset topic)."""
    try:
      import os
      socket_path = '/tmp/plugin_bus/steer_angle_offset'
      if self._offset_sub is not None and not os.path.exists(socket_path):
        try:
          self._offset_sub.close()
        except Exception:
          pass
        self._offset_sub = None
      if self._offset_sub is None and os.path.exists(socket_path):
        from openpilot.selfdrive.plugins.plugin_bus import PluginSub
        self._offset_sub = PluginSub(['steer_angle_offset'])
      if self._offset_sub is not None:
        msg = self._offset_sub.drain('steer_angle_offset')
        if msg is not None:
          _, data = msg
          self.steer_angle_offset = float(data.get('offset', self.steer_angle_offset))
    except Exception:
      pass

  @staticmethod
  def get_can_parsers(CP):
    # Always subscribe to both DCC and NCC cruise messages (with nan = no timeout).
    # Fingerprinting may misclassify cruise type if the ECU is slow to wake;
    # subscribing to both prevents canValid failures from unsubscribed messages
    # being lazily accessed via vl[] in update().
    pt_messages = [
      ("TurnSignals", float('nan')),
      ("DynamicCruiseControlStatus", float('nan')),
      ("CruiseControlStatus", float('nan')),
      ("CruiseControlStalk", float('nan')),
    ]
    fcan_messages = [
      ("CruiseControlStalk", float('nan')),
      ("SteeringWheelAngle_DSC", float('nan')),
    ]
    servo_can_messages = []

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, CanBus.PT_CAN),
      Bus.body: CANParser(DBC[CP.carFingerprint][Bus.body], fcan_messages, CanBus.F_CAN),
      Bus.alt: CANParser(DBC[CP.carFingerprint][Bus.alt], servo_can_messages, CanBus.SERVO_CAN),
    }
