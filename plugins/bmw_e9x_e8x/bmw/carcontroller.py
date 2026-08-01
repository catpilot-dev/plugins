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
# chosen cadence (single 20 Hz or hold 40 Hz). Force a final "overwrite" frame
# within PRE_TICK_LEAD of stock's predicted next tick: our counter lands first
# on PT-CAN, advancing DCC's accepted counter past stock's pending value. When
# stock's idle frame arrives ~10 ms later carrying a same-or-earlier counter,
# DCC drops it as stale. This avoids DTC 5ECE while preserving the cadence DCC
# uses to interpret accel magnitude.
#
# Counter overwrite mechanism: SZL emits its 0x194 counter open-loop, +1 per
# 200 ms slot regardless of what DCC accepts. To keep DCC's accepted counter
# ahead of SZL through a burst, EVERY in-burst stock idle slot must be
# overwritten in its lead window — we cannot rely on DCC catching up.
# Per-slot drift = (frames_per_slot − 1): HOLD drifts +7/slot, SINGLE +3/slot.
# At burst end, DCC accepts SZL's resumption only if (1 + M − K) mod 15 ∈ [1,7]
# where M = slots overwritten, K = total frames. Until that holds, keep
# transmitting (with neutral act=0 if openpilot has stopped commanding).
HOLD_INTERVAL = 0.025         # 40 Hz — used when commanded accel ≥ ACCEL_HOLD_THRESHOLD
SINGLE_INTERVAL = 0.050       # 20 Hz — single-press cadence
PRE_TICK_LEAD = 0.015         # lead window 15 ms — wide enough to catch ≥1 OP cycle (10 ms) with phase jitter
BURST_LIVE_WINDOW = 0.5       # s — burst considered "live" until this long without TX

# DCC command selection thresholds
V_ERROR_DEADZONE = 0.5 / 3.6   # m/s (~0.5 km/h) — deadzone for entry and burst cancellation
ACCEL_HOLD_THRESHOLD = 0.2     # m/s² — use HOLD_INTERVAL above this, SINGLE_INTERVAL below
ACCEL_STEP5_THRESHOLD = 0.6    # m/s² — use +5 above this, +1 below (midpoint of 0.4–1.2)
DECEL_HOLD_THRESHOLD = 0.4
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
    # Burst counter-overwrite tracking: M slots, K frames, last cadence used.
    self.cruise_burst_slots = 0
    self.cruise_burst_frames = 0
    self.cruise_burst_interval = SINGLE_INTERVAL
    self.cruise_in_lead_window_prev = False

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
      in_lead_window = elapsed_in_slot_ns >= slot_period_ns - PRE_TICK_LEAD * 1e9

      # Track lead-window edge BEFORE any early returns so M counter stays
      # correct across throttled OP cycles that don't TX.
      crossing_into_lead = in_lead_window and not self.cruise_in_lead_window_prev
      self.cruise_in_lead_window_prev = in_lead_window

      # Force an "overwrite" frame in the final PRE_TICK_LEAD of each slot: our
      # next counter lands first on PT-CAN and advances DCC's accepted counter
      # past stock's pending value. Stock's idle frame (same-or-earlier counter)
      # arrives ~10 ms later and DCC drops it as stale. Outside the lead window,
      # throttle to the chosen cadence (HOLD 40 Hz / SINGLE 20 Hz) — DCC infers
      # accel magnitude from that rate.
      dt_tx = (now_nanos - self.last_cruise_tx_timestamp) / 1e9
      if in_lead_window:
        if dt_tx < DT_CTRL / 2:
          return False
      elif dt_tx < interval - DT_CTRL / 2:
        return False

      # Sync TX counter from RX on burst start (after a long pause); within a
      # burst, carry our independent sequence forward so DCC's "must advance"
      # check is satisfied even if stock's intermittent ticks have rotated rx.
      burst_dead = self.tx_cruise_stalk_counter < 0 or dt_tx > BURST_LIVE_WINDOW
      if burst_dead:
        self.tx_cruise_stalk_counter = self.rx_cruise_stalk_counter_last
        self.cruise_burst_slots = 0
        self.cruise_burst_frames = 0
      self.tx_cruise_stalk_counter = (self.tx_cruise_stalk_counter + 1) % 15
      self.cruise_burst_frames += 1
      # Count one slot per lead-window entry edge — the lead-window TX is the
      # frame that overwrites that slot's stock idle.
      if crossing_into_lead:
        self.cruise_burst_slots += 1
      # Track cadence for trailing-frame replication after commanding ends.
      if cmd is not None:
        self.cruise_burst_interval = interval
      can_sends.append(bmwcan.create_accel_command(self.packer, cmd, self.cruise_bus, self.tx_cruise_stalk_counter))
      self.last_cruise_tx_timestamp = now_nanos
      return True

    def cruise_burst_release_safe():
      # DCC accepts SZL's resumption as forward iff (1 + M − K) mod 15 ∈ [1, 7].
      # Requires at least one overwritten slot (M ≥ 1) so SZL is observed to be
      # behind DCC's accepted counter at handoff.
      if self.cruise_burst_slots < 1:
        return False
      delta = (1 + self.cruise_burst_slots - self.cruise_burst_frames) % 15
      return 1 <= delta <= 7

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
            interval = HOLD_INTERVAL if -accel >= DECEL_HOLD_THRESHOLD else SINGLE_INTERVAL
            step = 5 if cmd == CruiseStalk.minus5 else 1
            if headroom_kmh >= step:
              cruise_cmd(cmd, interval)

    # Trailing counter overwrite. If commanding stopped (or is briefly idle in
    # a deadzone) but the burst is still live, keep transmitting at the burst's
    # cadence with neutral act=0 frames until SZL's natural counter has caught
    # up enough that handoff back to stock is a forward step. Yields the bus
    # immediately when the driver is on the stalk.
    burst_alive = self.tx_cruise_stalk_counter >= 0 \
                  and (now_nanos - self.last_cruise_tx_timestamp) / 1e9 < BURST_LIVE_WINDOW
    if burst_alive and not cruise_stalk_human_pressing and not cruise_burst_release_safe():
      cruise_cmd(None, self.cruise_burst_interval)

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
