from opendbc.car import Bus, DT_CTRL
from opendbc.car.lateral import apply_dist_to_meas_limits
from bmw import bmwcan
from bmw.bmwcan import SteeringModes, CruiseStalk
from bmw.values import CarControllerParams, CanBus, BmwFlags, CruiseSettings
from opendbc.car.interfaces import CarControllerBase
from opendbc.can import CANPacker
from opendbc.car.common.conversions import Conversions as CV


# DO NOT CHANGE: Cruise control step size
CC_STEP = 1

# BMW stock cruise stalk idle cadence — 5 Hz when no physical stalk press.
# When a button is held, stock accelerates to 20 Hz (single) / 40 Hz (hold);
# DCC infers single-vs-hold (and acceleration magnitude) from this cadence.
CRUISE_STALK_IDLE_TICK_STOCK = 0.2

# Inject emulated cruise commands inside stock's 200 ms idle window at our
# chosen cadence (single 20 Hz or hold 40 Hz). The last injection of each slot
# is placed within PRE_TICK_LEAD before stock's predicted next tick: it carries
# the counter that stock would have emitted, lands first on PT-CAN, and stock's
# duplicate same-counter idle frame arriving ~10 ms later is dropped by DCC's
# counter-must-advance check. This avoids DTC 5ECE while preserving the cadence
# DCC needs to interpret accel magnitude.
HOLD_INTERVAL = 0.025         # 40 Hz — used when commanded accel ≥ ACCEL_HOLD_THRESHOLD
SINGLE_INTERVAL = 0.050       # 20 Hz — single-press cadence
PRE_TICK_LEAD = 0.010         # last inject lands 10 ms before stock's predicted tick

# DCC command selection thresholds
V_ERROR_DEADZONE = 0.5 / 3.6   # m/s (~0.5 km/h) — deadzone for entry and burst cancellation
ACCEL_HOLD_THRESHOLD = 0.2     # m/s² — use HOLD_INTERVAL above this, SINGLE_INTERVAL below
ACCEL_STEP5_THRESHOLD = 0.6    # m/s² — use +5 above this, +1 below (midpoint of 0.4–1.2)
DECEL_STEP5_THRESHOLD = 0.9    # m/s² — use -5 above this, -1 below (midpoint of 0.6–1.2)

# DCC Calibration
# PLUS1 + HOLD = +0.4 m/s²
# PLUS5 + HOLD = +1.2 m/s²
# MINUS1 + HOLD = -0.6 m/s²
# MINUS5 + HOLD = -1.2 m/s²

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
    self.rx_cruise_stalk_counter_last = -1
    self.tx_cruise_stalk_counter = -1

    self.cruise_bus = CanBus.PT_CAN
    if CP.flags & BmwFlags.DYNAMIC_CRUISE_CONTROL:
      self.cruise_bus = CanBus.F_CAN

    self.packer = CANPacker(dbc_name[Bus.pt])

  def update(self, CC, CS, now_nanos):

    actuators = CC.actuators
    can_sends = []

    self.cruise_units = (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)

    v_target = actuators.speed

    v_current = CS.out.vEgo
    v_error = v_target - v_current

    accel = actuators.accel

    # Anchor stock's idle phase. Update on counter advance whenever a recent TX
    # echo can't account for it (RX more than one OP cycle since our TX), so the
    # phase stays locked to the stock module's actual 5 Hz idle clock and not to
    # our injection cadence.
    if CS.cruise_stalk_counter != self.rx_cruise_stalk_counter_last:
      if (now_nanos - self.last_cruise_tx_timestamp) > 2 * DT_CTRL * 1e9:
        self.last_cruise_rx_timestamp = now_nanos
    self.rx_cruise_stalk_counter_last = CS.cruise_stalk_counter

    def cruise_cmd(cmd, interval):
      if self.last_cruise_rx_timestamp == 0:
        return False

      # Position in stock's open-loop 200 ms slot phase. Modulo handles long
      # bursts where the rx anchor isn't refreshed every slot.
      slot_period_ns = CRUISE_STALK_IDLE_TICK_STOCK * 1e9
      elapsed_in_slot_ns = (now_nanos - self.last_cruise_rx_timestamp) % slot_period_ns

      # Block TX in the final PRE_TICK_LEAD of the slot — covers ~10 ms cycle
      # quantization in rx_timestamp (set on the OP cycle that observes stock's
      # tick) plus a small bus-separation margin. The cycle just before this
      # block is the "overwrite" frame: it lands at predicted - PRE_TICK_LEAD,
      # carries the counter stock would emit, and DCC drops stock's duplicate.
      if elapsed_in_slot_ns > slot_period_ns - PRE_TICK_LEAD * 1e9:
        return False

      # Throttle to the chosen cadence (HOLD 40 Hz / SINGLE 20 Hz) — DCC infers
      # accel magnitude from this rate.
      dt_tx = (now_nanos - self.last_cruise_tx_timestamp) / 1e9
      if dt_tx < interval - DT_CTRL / 2:
        return False

      # Sync TX counter from RX on burst start (after a long pause); within a
      # burst, carry our independent sequence forward so DCC's "must advance"
      # check is satisfied even if stock's intermittent ticks have rotated rx.
      if self.tx_cruise_stalk_counter < 0 or dt_tx > 0.5:
        self.tx_cruise_stalk_counter = self.rx_cruise_stalk_counter_last
      self.tx_cruise_stalk_counter = (self.tx_cruise_stalk_counter + 1) % 15
      can_sends.append(bmwcan.create_accel_command(self.packer, cmd, self.cruise_bus, self.tx_cruise_stalk_counter))
      self.last_cruise_tx_timestamp = now_nanos
      return True

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
        cruise_cmd(CruiseStalk.cancel, SINGLE_INTERVAL)
      elif CC.enabled:
        if CS.out.gasPressed:
          cruise_cmd(CruiseStalk.plus1, SINGLE_INTERVAL)
        else:
          setpoint_error = v_target - CS.out.cruiseState.speed

          if v_error > V_ERROR_DEADZONE and accel > 0 and setpoint_error > 0:
            cmd = CruiseStalk.plus5 if accel >= ACCEL_STEP5_THRESHOLD else CruiseStalk.plus1
            interval = HOLD_INTERVAL if accel >= ACCEL_HOLD_THRESHOLD else SINGLE_INTERVAL
            cruise_cmd(cmd, interval)

          elif v_error < -V_ERROR_DEADZONE and accel < 0 and setpoint_error < 0 and CS.out.cruiseState.speed > self.min_cruise_setpoint:
            headroom_kmh = (CS.out.cruiseState.speed - self.min_cruise_setpoint) * 3.6
            cmd = CruiseStalk.minus5 if -accel >= DECEL_STEP5_THRESHOLD else CruiseStalk.minus1
            interval = HOLD_INTERVAL if -accel >= ACCEL_HOLD_THRESHOLD else SINGLE_INTERVAL
            step = 5 if cmd == CruiseStalk.minus5 else 1
            if headroom_kmh >= step:
              cruise_cmd(cmd, interval)

    if self.flags & BmwFlags.STEPPER_SERVO_CAN:
      if CC.enabled and CC.latActive:
        new_steer = actuators.torque * CarControllerParams.STEER_MAX
        apply_torque = apply_dist_to_meas_limits(new_steer, self.apply_torque_last, CS.out.steeringTorqueEps,
                                           CarControllerParams.STEER_DELTA_UP, CarControllerParams.STEER_DELTA_DOWN,
                                           CarControllerParams.STEER_ERROR_MAX, CarControllerParams.STEER_MAX)
        self.apply_torque_last = apply_torque
        can_sends.append(bmwcan.create_steer_command(self.frame, SteeringModes.TorqueControl, apply_torque))
      elif not CS.cruise_stalk_cancel and not CS.out.brakePressed and not CS.out.gasPressed and self.apply_torque_last != 0:
        can_sends.append(bmwcan.create_steer_command(self.frame, SteeringModes.SoftOff, self.apply_torque_last))
        self.apply_torque_last = CS.out.steeringTorqueEps
      else:
        self.apply_torque_last = 0
        can_sends.append(bmwcan.create_steer_command(self.frame, SteeringModes.Off))

    self.cruise_enabled_prev = CC.enabled

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / CarControllerParams.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last

    new_actuators.speed = v_target

    self.frame += 1
    return new_actuators, can_sends
