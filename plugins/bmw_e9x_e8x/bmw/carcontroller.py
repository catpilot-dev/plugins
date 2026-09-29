from collections import deque

from opendbc.car import Bus, DT_CTRL
from opendbc.car.lateral import apply_dist_to_meas_limits
from bmw import bmwcan
from bmw.bmwcan import SteeringModes, CruiseStalk
from bmw.values import CarControllerParams, CanBus, BmwFlags, CruiseSettings
from opendbc.car.interfaces import CarControllerBase
from opendbc.can import CANPacker
from opendbc.car.common.conversions import Conversions as CV


# DCC cruise-stalk control. The rationale and the measurements behind every
# constant below live in ../DESIGN.md, "DCC cruise-stalk control"; section
# names are cited as DESIGN.md: "<heading>".

# 0x194 counter machinery — DESIGN.md: "How DCC accepts 0x194" and "The
# counter machinery". Field-proven; do not tune without on-car verification.
# DCC suppresses SZL's frame by TIMING when ours lands just ahead of it, and
# the surviving stream must step exactly +1.
CRUISE_STALK_IDLE_TICK_STOCK = 0.2  # s — SZL's open-loop 5 Hz idle slot
HOLD_INTERVAL = 0.025         # nominal 40 Hz, measures 48 Hz on the 10 ms grid
SINGLE_INTERVAL = 0.050       # 20 Hz
PRE_TICK_LEAD = 0.015         # s — overwrite window before SZL's tick, >= 1 control cycle with jitter
BURST_LIVE_WINDOW = 0.5       # s — burst considered live until this long without TX

# Accel side — DESIGN.md: "DCC cruise-stalk control" (cadence encodes magnitude)
V_ERROR_DEADZONE = 0.5 / 3.6   # m/s — accel-side entry gate
ACCEL_HOLD_THRESHOLD = 0.3     # m/s² — HOLD above, SINGLE below
ACCEL_STEP5_THRESHOLD = 0.6    # m/s² — plus5 offered above; its landing test decides

# Decel setpoint inverts the plant — DESIGN.md: "The setpoint is a torque
# request, not a speed". K_DCC is stiffer than the measured 0.0935 so every ask
# lands shallow of the plant; 0.12 over the first-cut 0.1 is a comfort choice.
K_DCC = 0.12                   # m/s² of DCC response per km/h of setpoint gap
SETPOINT_BIAS_MAX = 12.0       # km/h below v_target — plant floor −1.12 m/s²

# Command gate: a Schmitt trigger — the full deadzone opens a braking episode,
# the narrow one keeps it open; restore always pays the full width.
# DESIGN.md: "Command selection" (one deadzone, both directions) and "The
# command gate is a Schmitt trigger".
SETPOINT_DEADZONE = 3.0        # km/h — how far off target before we command
SETPOINT_HOLD_DEADZONE = 1.0   # km/h — once an episode is under way

# Concordance gate: accel and vTarget's trend must agree in sign to take on or
# give up the braking bias; disagreement holds state. Only the sign of the
# vTarget delta is read. DV_WINDOW is the deque length, not a tuning knob.
# DESIGN.md: "Concordance gate — two estimates must agree".
DV_WINDOW = 0.30               # s — 6 modelV2 frames at 20 Hz

# Command selection: one decision per SZL slot, keyed on the setpoint error
# left after what is still in flight (setpoint_pending). minus1 is a 1 km/h
# step; minus5/plus5 snap to a 10 km/h grid and are used only when the landing
# point does not pass the target. DESIGN.md: "Command selection — one decision
# per SZL slot", "±1 is a step; ±5 snaps to a grid", "±5 needs no threshold".
STEP5_GRID_KMH = 10.0         # DCC snaps to this grid on plus5 and minus5
STEP5_MIN_USEFUL_KMH = 2.0    # below this it is the ±1 command's job anyway
# Measured grid position in cruiseState.speed units (speed ≡ 8 mod 10). NOT
# CruiseSettings.CLUSTER_OFFSET, which is cosmetic — a test pins them apart.
STEP5_GRID_PHASE = 2.0        # km/h
# Held longer, DCC auto-repeats minus5 into a second grid step. Counted in
# frames, not seconds. DESIGN.md: "minus5 is released after two frames".
MINUS5_ASSERT_FRAMES = 2      # transmitted frames, then release
MINUS1_YIELD_KMH = 1.0
PENDING_TIMEOUT = 0.5         # s — forget what was sent, or an unresponsive DCC silences us for good


def minus5_landing_kmh(setpoint_kmh):
  """Where a minus5 will actually put the setpoint, in the same units as
  CS.out.cruiseState.speed (km/h).

  DCC snaps down to the next multiple of STEP5_GRID_KMH strictly below the
  current value. STEP5_GRID_PHASE places that grid in these units. Exact on 54
  of 54 measured landings.
  """
  raw = round(setpoint_kmh + STEP5_GRID_PHASE)
  land = STEP5_GRID_KMH * ((raw - 1) // STEP5_GRID_KMH)
  return land - STEP5_GRID_PHASE


def plus5_landing_kmh(setpoint_kmh):
  """The mirror: plus5 snaps UP to the next multiple of STEP5_GRID_KMH strictly
  above. Exact on 32 of 32 measured landings, gain 1-10 km/h — from 49 you get
  1, from 50 you get 10, same command."""
  raw = round(setpoint_kmh + STEP5_GRID_PHASE)
  land = STEP5_GRID_KMH * ((raw // STEP5_GRID_KMH) + 1)
  return land - STEP5_GRID_PHASE


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP):
    super().__init__(dbc_name, CP)
    self.flags = CP.flags
    self.min_cruise_speed = CP.minEnableSpeed
    self.min_cruise_setpoint = self.min_cruise_speed + CruiseSettings.MIN_SPEED_BUFFER * CV.KPH_TO_MS

    self.cruise_cancel = False
    self.cruise_enabled_prev = False
    self.apply_torque_last = 0
    self.last_cruise_rx_timestamp = 0
    self.last_cruise_tx_timestamp = 0
    self.rx_cruise_stalk_counter_last = -1
    self.tx_cruise_stalk_counter = -1
    # Burst counter-overwrite tracking: M slots, K frames, last cadence used.
    # DESIGN.md: "The counter machinery".
    self.cruise_burst_slots = 0
    self.cruise_burst_frames = 0
    self.cruise_burst_interval = SINGLE_INTERVAL
    self.cruise_in_lead_window_prev = False
    # Handoff latch: set once SZL has taken the counter back. Forces the next
    # command to start a fresh burst resynced from RX.
    self.cruise_burst_released = False
    self.cruise_release_rx_cnt = -1

    # Concordance state: vTarget history for its trend, and the braking latch.
    self.v_target_hist = deque(maxlen=int(round(DV_WINDOW / DT_CTRL)) + 1)
    self.setpoint_braking = False
    # True once this braking episode has sent a down command: narrows the gate
    # to SETPOINT_HOLD_DEADZONE until the episode ends.
    self.setpoint_commanding = False
    # Frames actually transmitted for the current slot's command, so minus5 can
    # be released after a counted number rather than a wall-clock window.
    self.slot_cmd_frames = 0

    # Per-slot command selection: what was decided this slot, and how much
    # setpoint we have asked for but not yet seen arrive.
    self.slot_cmd = None
    self.slot_decided_ns = 0
    self.setpoint_pending = 0.0
    self.setpoint_pending_ns = 0
    self.setpoint_last_seen = None

    # Debt ledger: the driver's set speed in km/h, latched while openpilot is
    # driving (v_cruise is not live otherwise). Every downward bias is borrowed
    # from it and repaid. Zero means nothing is owed. DESIGN.md: "The debt
    # ledger" (in "The command gate is a Schmitt trigger").
    self.setpoint_handback_kmh = 0.0
    self.setpoint_debt = 0.0     # derived each cycle, kept for logging and tests

    self.cruise_bus = CanBus.PT_CAN
    if CP.flags & BmwFlags.DYNAMIC_CRUISE_CONTROL:
      self.cruise_bus = CanBus.F_CAN

    self.packer = CANPacker(dbc_name[Bus.pt])

  def update(self, CC, CS, now_nanos):

    actuators = CC.actuators
    can_sends = []

    v_target = actuators.speed

    v_current = CS.out.vEgo
    v_error = v_target - v_current

    accel = actuators.accel

    # Anchor SZL's idle phase on an RX counter advance our own TX can't explain.
    if CS.cruise_stalk_counter != self.rx_cruise_stalk_counter_last:
      if (now_nanos - self.last_cruise_tx_timestamp) > 2 * DT_CTRL * 1e9:
        self.last_cruise_rx_timestamp = now_nanos
    self.rx_cruise_stalk_counter_last = CS.cruise_stalk_counter

    tx_this_cycle = False

    def cruise_cmd(cmd, interval):
      nonlocal tx_this_cycle
      if self.last_cruise_rx_timestamp == 0:
        return False

      # Position in SZL's 200 ms slot; modulo covers an anchor not refreshed.
      slot_period_ns = CRUISE_STALK_IDLE_TICK_STOCK * 1e9
      elapsed_in_slot_ns = (now_nanos - self.last_cruise_rx_timestamp) % slot_period_ns
      in_lead_window = elapsed_in_slot_ns >= slot_period_ns - PRE_TICK_LEAD * 1e9

      # Track the lead-window edge BEFORE any early return so M stays correct.
      crossing_into_lead = in_lead_window and not self.cruise_in_lead_window_prev
      self.cruise_in_lead_window_prev = in_lead_window

      # In the lead window, always send: ours lands first and SZL's idle frame
      # is suppressed by timing. Outside it, throttle to the cadence.
      dt_tx = (now_nanos - self.last_cruise_tx_timestamp) / 1e9
      if in_lead_window:
        if dt_tx < DT_CTRL / 2:
          return False
      elif dt_tx < interval - DT_CTRL / 2:
        return False

      # Resync from RX on burst start, else carry our own +1 sequence. A burst
      # ends on BURST_LIVE_WINDOW of silence or, in practice, the handoff latch.
      burst_dead = self.tx_cruise_stalk_counter < 0 or dt_tx > BURST_LIVE_WINDOW \
                   or self.cruise_burst_released
      if burst_dead:
        self.tx_cruise_stalk_counter = self.rx_cruise_stalk_counter_last
        self.cruise_burst_slots = 0
        self.cruise_burst_frames = 0
        self.cruise_burst_released = False
        self.cruise_release_rx_cnt = -1
      self.tx_cruise_stalk_counter = (self.tx_cruise_stalk_counter + 1) % 15
      self.cruise_burst_frames += 1
      # One overwritten slot per lead-window entry edge.
      if crossing_into_lead:
        self.cruise_burst_slots += 1
      # Track cadence for trailing-frame replication after commanding ends.
      if cmd is not None:
        self.cruise_burst_interval = interval
      can_sends.append(bmwcan.create_accel_command(self.packer, cmd, self.cruise_bus, self.tx_cruise_stalk_counter))
      self.last_cruise_tx_timestamp = now_nanos
      tx_this_cycle = True
      return True

    def cruise_burst_release_safe():
      # Legacy (1 + M − K) mod 15 ∈ [1, 7] test. Its rationale was retracted
      # 2026-09-06 but the trailing behaviour it gates is field-proven — see
      # DESIGN.md: "How DCC accepts 0x194" before changing it.
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

    # Concordance gate — DESIGN.md: "Concordance gate — two estimates must agree".
    self.v_target_hist.append(v_target)
    if len(self.v_target_hist) == self.v_target_hist.maxlen:
      dv_target = v_target - self.v_target_hist[0]
    else:
      # No full window yet means no second estimate: hold (never fall back to
      # accel alone — that is route 454's bare sign test).
      dv_target = 0.0
    if not CC.enabled:
      self.setpoint_braking = False
      self.v_target_hist.clear()
    elif accel < 0 and dv_target < 0:
      self.setpoint_braking = True
    elif accel > 0 and dv_target > 0:
      self.setpoint_braking = False
    if not self.setpoint_braking:
      self.setpoint_commanding = False
    # else: the two estimates disagree — hold the state we are already in.

    # A changed reading is a fresh 0x193 report: nothing is in flight any more.
    if CS.out.cruiseState.speed != self.setpoint_last_seen:
      self.setpoint_last_seen = CS.out.cruiseState.speed
      self.setpoint_pending = 0.0
    elif (now_nanos - self.setpoint_pending_ns) / 1e9 > PENDING_TIMEOUT:
      self.setpoint_pending = 0.0
    if not self.setpoint_braking:
      self.setpoint_pending = 0.0
      self.slot_cmd = None

    cruise_stalk_human_pressing = CS.cruise_stalk_resume or CS.cruise_stalk_cancel or CS.cruise_stalk_speed != 0

    # Latch the hand-back point while we still have the car (range-checked
    # like register.py's cruise-ceiling memory).
    if CC.enabled and CS.out.cruiseState.enabled and 30.0 <= CS.out.vCruise <= 145.0:
      self.setpoint_handback_kmh = CS.out.vCruise

    # Drop the claim when the driver takes the stalk (their value stands) or
    # DCC loses availability (the setpoint memory goes with it).
    if cruise_stalk_human_pressing or not CS.out.cruiseState.available:
      self.setpoint_handback_kmh = 0.0

    # Debt is measured off the setpoint readback, never counted from frames.
    self.setpoint_debt = 0.0
    if self.setpoint_handback_kmh > 0.0:
      self.setpoint_debt = min(SETPOINT_BIAS_MAX,
                               self.setpoint_handback_kmh - CS.out.cruiseState.speed * 3.6)

    if not cruise_stalk_human_pressing and CS.out.cruiseState.enabled:
      if self.cruise_cancel:
        cruise_cmd(CruiseStalk.cancel, SINGLE_INTERVAL)
      elif CC.enabled:
        if CS.out.gasPressed:
          cruise_cmd(CruiseStalk.plus1, SINGLE_INTERVAL)
        else:
          # Setpoint target: v_target (planner vTarget ~0.5 s ahead, not the
          # driver's set speed), deepened by the plant-inverting bias during a
          # braking episode. Only the decel side inverts — DESIGN.md: "Only the
          # decel side inverts the plant".
          sp_target = v_target
          if self.setpoint_braking:
            # min(accel, 0): a positive blip holds the bias; the latch releases.
            bias = max(min(accel, 0.0) / K_DCC, -SETPOINT_BIAS_MAX) * CV.KPH_TO_MS
            sp_target = min(v_target, v_current + bias)

          setpoint_error = sp_target - CS.out.cruiseState.speed
          active_deadzone_kmh = (SETPOINT_HOLD_DEADZONE if self.setpoint_commanding
                                 else SETPOINT_DEADZONE)
          active_deadzone = active_deadzone_kmh * CV.KPH_TO_MS
          setpoint_deadzone = SETPOINT_DEADZONE * CV.KPH_TO_MS

          # No v_error term on the decel side — the setpoint deadzone replaces it.
          decel_gate = self.setpoint_braking and setpoint_error < -active_deadzone

          if v_error > V_ERROR_DEADZONE and accel > 0 and setpoint_error > 0:
            cmd = CruiseStalk.plus1
            if accel >= ACCEL_STEP5_THRESHOLD:
              # plus5 only when its grid landing does not pass the target.
              sp_now_kmh = CS.out.cruiseState.speed * 3.6
              gain_kmh = plus5_landing_kmh(sp_now_kmh) - sp_now_kmh
              if STEP5_MIN_USEFUL_KMH <= gain_kmh <= setpoint_error * 3.6:
                cmd = CruiseStalk.plus5
            interval = HOLD_INTERVAL if accel >= ACCEL_HOLD_THRESHOLD else SINGLE_INTERVAL
            cruise_cmd(cmd, interval)

          elif accel < 0 and decel_gate and CS.out.cruiseState.speed > self.min_cruise_setpoint:
            headroom_kmh = (CS.out.cruiseState.speed - self.min_cruise_setpoint) * 3.6
            # One decision per SZL slot, held for the slot.
            if (now_nanos - self.slot_decided_ns) / 1e9 >= CRUISE_STALK_IDLE_TICK_STOCK:
              self.slot_decided_ns = now_nanos
              self.slot_cmd_frames = 0
              err_kmh = -setpoint_error * 3.6 - self.setpoint_pending
              # minus5's landing, from the setpoint expected once pending lands.
              sp_eff_kmh = CS.out.cruiseState.speed * 3.6 - self.setpoint_pending
              m5_land_kmh = minus5_landing_kmh(sp_eff_kmh)
              m5_drop_kmh = sp_eff_kmh - m5_land_kmh
              # The whole overshoot guard: minus5 only if it lands at or above
              # the target (and the floor).
              if (STEP5_MIN_USEFUL_KMH <= m5_drop_kmh <= err_kmh
                  and m5_land_kmh >= self.min_cruise_setpoint * 3.6):
                self.slot_cmd = CruiseStalk.minus5
                self.setpoint_pending += m5_drop_kmh
                self.setpoint_pending_ns = now_nanos
                self.setpoint_commanding = True
              elif err_kmh >= active_deadzone_kmh and headroom_kmh >= 1:
                self.slot_cmd = CruiseStalk.minus1
                self.setpoint_pending += MINUS1_YIELD_KMH
                self.setpoint_pending_ns = now_nanos
                self.setpoint_commanding = True
              else:
                self.slot_cmd = None
            if self.slot_cmd is not None:
              # minus5 releases after MINUS5_ASSERT_FRAMES; minus1 holds the slot.
              if (self.slot_cmd is not CruiseStalk.minus5
                  or self.slot_cmd_frames < MINUS5_ASSERT_FRAMES):
                if cruise_cmd(self.slot_cmd, SINGLE_INTERVAL):
                  self.slot_cmd_frames += 1

          # Restore: walk a parked setpoint back up with plus1 at SINGLE
          # (<= 0.19 m/s²; plus5 would lurch). DESIGN.md: "The setpoint is a
          # torque request, not a speed" (Restore).
          elif setpoint_error > setpoint_deadzone:
            cruise_cmd(CruiseStalk.plus1, SINGLE_INTERVAL)

      # Repay the debt while openpilot is not driving — in practice on the DCC
      # rising edge after a disengage/brake standby. Threshold is one step, not
      # the deadzone. DESIGN.md: "The debt ledger".
      elif self.setpoint_debt >= MINUS1_YIELD_KMH:
        cruise_cmd(CruiseStalk.plus1, SINGLE_INTERVAL)

    # Trailing overwrite: neutral act=0 frames until release is safe; yields
    # to the driver. DESIGN.md: "The counter machinery".
    burst_alive = self.tx_cruise_stalk_counter >= 0 \
                  and (now_nanos - self.last_cruise_tx_timestamp) / 1e9 < BURST_LIVE_WINDOW
    if burst_alive and not cruise_stalk_human_pressing and not cruise_burst_release_safe():
      cruise_cmd(None, self.cruise_burst_interval)

    # Handoff latch: arm on the first silent cycle, fire once SZL's counter
    # moves while we are still silent, so a resume resyncs instead of breaking
    # +1 (route 444). DESIGN.md: "The counter machinery".
    if tx_this_cycle:
      self.cruise_release_rx_cnt = -1
    elif self.tx_cruise_stalk_counter >= 0 and (cruise_stalk_human_pressing or cruise_burst_release_safe()):
      if self.cruise_release_rx_cnt < 0:
        self.cruise_release_rx_cnt = self.rx_cruise_stalk_counter_last
      elif self.rx_cruise_stalk_counter_last != self.cruise_release_rx_cnt:
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
