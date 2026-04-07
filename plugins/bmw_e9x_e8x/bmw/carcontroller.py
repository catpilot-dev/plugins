from opendbc.car import Bus, DT_CTRL, apply_hysteresis
from opendbc.car.lateral import apply_dist_to_meas_limits
from bmw import bmwcan
from bmw.bmwcan import SteeringModes, CruiseStalk
from bmw.values import CarControllerParams, CanBus, BmwFlags, CruiseSettings
from opendbc.car.interfaces import CarControllerBase
from opendbc.can import CANPacker
from opendbc.car.common.conversions import Conversions as CV


# DO NOT CHANGE: Cruise control step size
CC_STEP = 1

# BMW Stock DCC CAN Frequencies
CRUISE_STALK_IDLE_TICK_STOCK = 0.2    # 5Hz
CRUISE_STALK_SINGLE_TICK_STOCK = 0.05 # 20Hz
CRUISE_STALK_HOLD_TICK_STOCK = 0.025  # 40Hz

# Openpilot DCC Emulation - Frequency-based command rates
SINGLE_TICK = 0.05    # 20Hz — 5 × DT_CTRL, matches model update rate
HOLD_TICK = 0.02     # 50Hz — 2 × DT_CTRL, faster than stock 40Hz so DCC recognizes as hold

# DCC command selection thresholds
V_ERROR_DEADZONE = 0.5 / 3.6   # m/s (~0.5 km/h) — deadzone for entry and burst cancellation
ACCEL_HOLD_THRESHOLD = 0.2     # m/s² — use HOLD_TICK above this, SINGLE_TICK below
ACCEL_STEP5_THRESHOLD = 0.6    # m/s² — use +5 above this, +1 below (midpoint of 0.4–1.2)
DECEL_STEP5_THRESHOLD = 0.9    # m/s² — use -5 above this, -1 below (midpoint of 0.6–1.2)

# DCC Calibration
# PLUS1 + HOLD_TICK = 0.4 m/s2
# PLUS5 + HOLD_TICK = 1.2 m/s2
# MINUS1 + HOLD_TICK = -0.6 m/s2
# MINUS5 + HOLD_TICK = -1.2 m/s2

class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP):
    super().__init__(dbc_name, CP)
    self.flags = CP.flags
    self.min_cruise_speed = CP.minEnableSpeed
    self.min_cruise_setpoint = self.min_cruise_speed + CruiseSettings.MIN_SPEED_BUFFER * CV.KPH_TO_MS
    self.cruise_units = None

    self.cruise_cancel = False
    self.cruise_enabled_prev = False
    self.apply_torque_last = 0
    self.last_cruise_rx_timestamp = 0
    self.last_cruise_tx_timestamp = 0
    self.tx_cruise_stalk_counter_last = 0
    self.rx_cruise_stalk_counter_last = -1

    self.cruise_bus = CanBus.PT_CAN
    if CP.flags & BmwFlags.DYNAMIC_CRUISE_CONTROL:
      self.cruise_bus = CanBus.F_CAN

    self.packer = CANPacker(dbc_name[Bus.pt])
    self.torque_steady = 0.0

    self.dcc_last_tick_time = 0

  def update(self, CC, CS, now_nanos):

    actuators = CC.actuators
    can_sends = []

    self.cruise_units = (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)

    v_target = actuators.speed

    v_current = CS.out.vEgo
    v_error = v_target - v_current

    accel = actuators.accel

    if CS.cruise_stalk_counter != self.rx_cruise_stalk_counter_last:
      # Resync to stock counter, but never go backwards — if we already
      # sent past the stock counter, keep ours to avoid duplicate counters.
      stock = CS.cruise_stalk_counter
      fwd = (stock - self.tx_cruise_stalk_counter_last) % 15
      if fwd <= 7:  # stock is ahead or same (within half-ring)
        self.tx_cruise_stalk_counter_last = stock
      self.last_cruise_rx_timestamp = now_nanos
    self.rx_cruise_stalk_counter_last = CS.cruise_stalk_counter

    def cruise_cmd(cmd, tick_interval):
      time_since_cruise_sent = (now_nanos - self.last_cruise_tx_timestamp) / 1e9 + DT_CTRL / 10
      time_since_cruise_received = (now_nanos - self.last_cruise_rx_timestamp) / 1e9 + DT_CTRL / 10
      send = time_since_cruise_sent > tick_interval \
        and time_since_cruise_received > CRUISE_STALK_HOLD_TICK_STOCK/2 - DT_CTRL \
        and time_since_cruise_received < CRUISE_STALK_IDLE_TICK_STOCK/2 + DT_CTRL
      if send:
        tx_cruise_stalk_counter = (self.tx_cruise_stalk_counter_last + 1) % 15
        can_sends.append(bmwcan.create_accel_command(self.packer, cmd, self.cruise_bus, tx_cruise_stalk_counter))
        self.tx_cruise_stalk_counter_last = tx_cruise_stalk_counter
        self.last_cruise_tx_timestamp = now_nanos
      return send

    if not CC.enabled and self.cruise_enabled_prev:
      self.cruise_cancel = True
    if (CS.out.cruiseState.speedCluster - self.min_cruise_speed) < 0.1 \
      and CS.out.vEgoCluster - self.min_cruise_speed < 0.4:
      self.cruise_cancel = True
    if not CS.out.cruiseState.enabled:
      self.cruise_cancel = False

    cruise_stalk_human_pressing = CS.cruise_stalk_resume or CS.cruise_stalk_cancel or CS.cruise_stalk_speed != 0

    if not cruise_stalk_human_pressing and CS.out.cruiseState.enabled:
      if self.cruise_cancel:
        cruise_cmd(CruiseStalk.cancel, SINGLE_TICK)
      elif CC.enabled:
        if CS.out.gasPressed:
          cruise_cmd(CruiseStalk.plus1, SINGLE_TICK)
        else:
          current_time = now_nanos / 1e9

          setpoint_error = v_target - CS.out.cruiseState.speed

          if v_error > V_ERROR_DEADZONE and accel > 0 and setpoint_error > 0:
            cmd = CruiseStalk.plus5 if accel >= ACCEL_STEP5_THRESHOLD else CruiseStalk.plus1
            interval = HOLD_TICK if accel >= ACCEL_HOLD_THRESHOLD else SINGLE_TICK
            if current_time - self.dcc_last_tick_time >= interval:
              if cruise_cmd(cmd, interval):
                self.dcc_last_tick_time = current_time

          elif v_error < -V_ERROR_DEADZONE and accel < 0 and setpoint_error < 0 and CS.out.cruiseState.speed > self.min_cruise_setpoint:
            headroom_kmh = (CS.out.cruiseState.speed - self.min_cruise_setpoint) * 3.6
            cmd = CruiseStalk.minus5 if -accel >= DECEL_STEP5_THRESHOLD else CruiseStalk.minus1
            interval = HOLD_TICK if -accel >= ACCEL_HOLD_THRESHOLD else SINGLE_TICK
            step = 5 if cmd == CruiseStalk.minus5 else 1
            if headroom_kmh >= step and current_time - self.dcc_last_tick_time >= interval:
              if cruise_cmd(cmd, interval):
                self.dcc_last_tick_time = current_time

    if self.flags & BmwFlags.STEPPER_SERVO_CAN:
      if CC.enabled and CC.latActive:
        new_steer = actuators.torque * CarControllerParams.STEER_MAX
        apply_torque = apply_dist_to_meas_limits(new_steer, self.apply_torque_last, CS.out.steeringTorqueEps,
                                           CarControllerParams.STEER_DELTA_UP, CarControllerParams.STEER_DELTA_DOWN,
                                           CarControllerParams.STEER_ERROR_MAX, CarControllerParams.STEER_MAX)
        # Hysteresis: suppress small torque oscillations (servo buzz) without phase lag.
        # Holds last output until input moves beyond ±hyst_gap, preserving mean offset.
        self.apply_torque_pre_hyst = apply_torque
        self.torque_steady = apply_hysteresis(apply_torque, self.torque_steady, CarControllerParams.STEER_TORQUE_HYST)
        apply_torque = self.torque_steady
        can_sends.append(bmwcan.create_steer_command(self.frame, SteeringModes.TorqueControl, apply_torque))
      elif not CS.cruise_stalk_cancel and not CS.out.brakePressed and not CS.out.gasPressed and self.apply_torque_last != 0:
        can_sends.append(bmwcan.create_steer_command(self.frame, SteeringModes.SoftOff, self.apply_torque_last))
        apply_torque = CS.out.steeringTorqueEps
      else:
        apply_torque = 0
        can_sends.append(bmwcan.create_steer_command(self.frame, SteeringModes.Off))
      self.apply_torque_last = apply_torque

    self.cruise_enabled_prev = CC.enabled

    new_actuators = actuators.as_builder()
    new_actuators.torque = getattr(self, 'apply_torque_pre_hyst', 0.0) / CarControllerParams.STEER_MAX  # pre-hysteresis (normalized)
    new_actuators.torqueOutputCan = self.apply_torque_last  # post-hysteresis (Nm)

    new_actuators.speed = v_target

    self.frame += 1
    return new_actuators, can_sends
