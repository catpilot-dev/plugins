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
# Counter law. Our counter advances by exactly CRUISE_SLOT_STEPS per stock idle
# slot rather than +1 per frame. 16 = 1 (mod 15), so the lead over SZL is
# stationary from slot to slot instead of walking, and 16*15 = 0 (mod 15) so it
# stays consistent across the counter's own wrap. Advancing +1 per frame makes
# the lead grow by (frames_per_slot - 1) each slot; the moment it passes 7,
# SZL's idle frame looks fresh to DCC again and the stock stalk takes the bus
# back mid-ramp. No cadence avoids that: only 1 or 16 frames per slot (5 Hz /
# 80 Hz) hold the lead fixed, and 80 Hz needs a 12.5 ms spacing the 100 Hz
# control loop cannot place. Measured on route 444 segs 19-22 with +1 per frame,
# DCC accepted 72% of our command frames and SZL won 50% of in-burst slots.
CRUISE_SLOT_STEPS = 16
# Where we sit inside DCC's rejection band. The delta SZL sees at each tick is
# (1 - CRUISE_LEAD_OFFSET - k) mod 15 with k = floor(CRUISE_SLOT_STEPS * phase)
# of our last frame; the PRE_TICK_LEAD frame pins k to 14-15, which is why that
# frame is load-bearing for this law and not just for freshness. Offsets 1..8
# all keep SZL stale; 5 is the middle and measured best on replay (99% of our
# commands accepted, stock takeovers 50% -> 9%).
CRUISE_LEAD_OFFSET = 5

HOLD_INTERVAL = 0.025         # 40 Hz — used when commanded accel ≥ ACCEL_HOLD_THRESHOLD
SINGLE_INTERVAL = 0.050       # 20 Hz — single-press cadence
PRE_TICK_LEAD = 0.015         # lead window 15 ms — wide enough to catch ≥1 OP cycle (10 ms) with phase jitter
BURST_LIVE_WINDOW = 0.5       # s — burst considered "live" until this long without TX

# DCC command selection thresholds
V_ERROR_DEADZONE = 0.5 / 3.6   # m/s (~0.5 km/h) — deadzone for entry and burst cancellation
ACCEL_HOLD_THRESHOLD = 0.3     # m/s² — use HOLD_INTERVAL above this, SINGLE_INTERVAL below
ACCEL_STEP5_THRESHOLD = 0.6    # m/s² — use +5 above this, +1 below (midpoint of 0.4–1.2)
DECEL_HOLD_THRESHOLD = 0.3
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
    # Burst anchors for the slot counter law: SZL's counter when the burst
    # started, and the counter we opened on.
    self.cruise_burst_rx_start = 0
    self.cruise_burst_c_start = 0
    self.cruise_burst_interval = SINGLE_INTERVAL
    # Handoff latch: set once DCC has been observed to take SZL's counter back
    # (see the handoff block at the end of update). Forces the next command to
    # start a fresh burst resynced from RX instead of resuming a stale sequence.
    self.cruise_burst_released = False

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

    tx_this_cycle = False

    def cruise_cmd(cmd, interval):
      nonlocal tx_this_cycle
      if self.last_cruise_rx_timestamp == 0:
        return False

      # Position in stock's open-loop 200 ms slot phase. Modulo handles long
      # bursts where the rx anchor isn't refreshed every slot.
      slot_period_ns = CRUISE_STALK_IDLE_TICK_STOCK * 1e9
      elapsed_in_slot_ns = (now_nanos - self.last_cruise_rx_timestamp) % slot_period_ns
      in_lead_window = elapsed_in_slot_ns >= slot_period_ns - PRE_TICK_LEAD * 1e9

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

      # Sync TX counter from RX on burst start; within a burst, carry our
      # independent sequence forward so DCC's "must advance" check is satisfied
      # even if stock's intermittent ticks have rotated rx. A burst is over
      # either because nothing was sent for BURST_LIVE_WINDOW, or because the
      # handoff latch fired — the latter is the one that matters in practice,
      # since BURST_LIVE_WINDOW (0.5 s) outlives SZL's 200 ms idle slot.
      burst_dead = self.tx_cruise_stalk_counter < 0 or dt_tx > BURST_LIVE_WINDOW \
                   or self.cruise_burst_released
      if burst_dead:
        # Open on SZL's counter plus the lead offset: a forward step for DCC,
        # which is still following SZL at this point.
        self.cruise_burst_rx_start = self.rx_cruise_stalk_counter_last
        self.cruise_burst_c_start = (self.cruise_burst_rx_start + CRUISE_LEAD_OFFSET) % 15
        self.tx_cruise_stalk_counter = self.cruise_burst_c_start
        self.cruise_burst_released = False
      else:
        # Target = anchor + 16 per elapsed slot + the fraction of this slot so
        # far. Slot count is taken from SZL's own counter, and wraps harmlessly
        # because 16*15 = 0 (mod 15).
        slots = (self.rx_cruise_stalk_counter_last - self.cruise_burst_rx_start) % 15
        phase = elapsed_in_slot_ns / slot_period_ns
        target = (self.cruise_burst_c_start + CRUISE_SLOT_STEPS * slots
                  + int(CRUISE_SLOT_STEPS * phase)) % 15
        # Every frame still has to be a forward step inside DCC's [1, 7] window:
        # never repeat a counter (two frames inside one 1/16 slot bucket), never
        # jump further than DCC will accept.
        step = (target - self.tx_cruise_stalk_counter) % 15
        step = 1 if step == 0 else min(step, 7)
        self.tx_cruise_stalk_counter = (self.tx_cruise_stalk_counter + step) % 15
      # Track cadence for trailing-frame replication after commanding ends.
      if cmd is not None:
        self.cruise_burst_interval = interval
      can_sends.append(bmwcan.create_accel_command(self.packer, cmd, self.cruise_bus, self.tx_cruise_stalk_counter))
      self.last_cruise_tx_timestamp = now_nanos
      tx_this_cycle = True
      return True

    def cruise_handoff_counter():
      # Under the slot law our lead is stationary, so SZL's idle frames stay
      # stale for as long as we transmit — they never become a forward step on
      # their own the way the old M/K drift arithmetic waited for. The handoff
      # therefore has to be made deliberately: emit one frame carrying SZL's
      # OWN current counter, and its next tick is +1 from DCC's accepted value,
      # a clean forward step. Only reachable while that lands inside DCC's
      # [1, 7] window, which is about 40% of each slot, so the caller keeps the
      # neutral overwrite running until it opens.
      handoff = self.rx_cruise_stalk_counter_last
      if 1 <= (handoff - self.tx_cruise_stalk_counter) % 15 <= 7:
        return handoff
      return None

    if not CC.enabled and self.cruise_enabled_prev:
      self.cruise_cancel = True
    if (CS.out.cruiseState.speedCluster - self.min_cruise_speed) < 0.1 \
      and CS.out.vEgoCluster - self.min_cruise_speed < 0.4:
      self.cruise_cancel = True
    if not CS.out.cruiseState.enabled:
      self.cruise_cancel = False

    cruise_stalk_human_pressing = CS.cruise_stalk_resume or CS.cruise_stalk_cancel or CS.cruise_stalk_speed != 0

    commanding = False
    if not cruise_stalk_human_pressing and CS.out.cruiseState.enabled:
      if self.cruise_cancel:
        cruise_cmd(CruiseStalk.cancel, SINGLE_INTERVAL)
        commanding = True
      elif CC.enabled:
        if CS.out.gasPressed:
          cruise_cmd(CruiseStalk.plus1, SINGLE_INTERVAL)
          commanding = True
        else:
          setpoint_error = v_target - CS.out.cruiseState.speed

          if v_error > V_ERROR_DEADZONE and accel > 0 and setpoint_error > 0:
            cmd = CruiseStalk.plus5 if accel >= ACCEL_STEP5_THRESHOLD else CruiseStalk.plus1
            interval = HOLD_INTERVAL if accel >= ACCEL_HOLD_THRESHOLD else SINGLE_INTERVAL
            cruise_cmd(cmd, interval)
            commanding = True

          elif v_error < -V_ERROR_DEADZONE and accel < 0 and setpoint_error < 0 and CS.out.cruiseState.speed > self.min_cruise_setpoint:
            headroom_kmh = (CS.out.cruiseState.speed - self.min_cruise_setpoint) * 3.6
            cmd = CruiseStalk.minus5 if -accel >= DECEL_STEP5_THRESHOLD else CruiseStalk.minus1
            interval = HOLD_INTERVAL if -accel >= DECEL_HOLD_THRESHOLD else SINGLE_INTERVAL
            step = 5 if cmd == CruiseStalk.minus5 else 1
            if headroom_kmh >= step:
              cruise_cmd(cmd, interval)
              commanding = True

    # Trailing overwrite and handoff. While commanding is paused but the burst
    # is still live, keep the neutral act=0 frames going so DCC stays on our
    # sequence, and take the first opening to hand the bus back deliberately.
    # Yields immediately when the driver is on the stalk: we stop transmitting,
    # our counter freezes, and SZL's own frames catch up within a few ticks.
    burst_alive = self.tx_cruise_stalk_counter >= 0 \
                  and (now_nanos - self.last_cruise_tx_timestamp) / 1e9 < BURST_LIVE_WINDOW
    if burst_alive and not self.cruise_burst_released:
      if cruise_stalk_human_pressing:
        self.cruise_burst_released = True
      elif not commanding:
        handoff = cruise_handoff_counter()
        if handoff is None:
          cruise_cmd(None, self.cruise_burst_interval)
        else:
          can_sends.append(bmwcan.create_accel_command(self.packer, None, self.cruise_bus, handoff))
          self.tx_cruise_stalk_counter = handoff
          self.last_cruise_tx_timestamp = now_nanos
          self.cruise_burst_released = True

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
