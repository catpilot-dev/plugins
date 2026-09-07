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
ACCEL_HOLD_THRESHOLD = 0.3     # m/s² — use HOLD_INTERVAL above this, SINGLE_INTERVAL below
ACCEL_STEP5_THRESHOLD = 0.6    # m/s² — use +5 above this, +1 below (midpoint of 0.4–1.2)
DECEL_HOLD_THRESHOLD = 0.3
DECEL_STEP5_THRESHOLD = 0.9    # m/s² — use -5 above this, -1 below (midpoint of 0.6–1.2)

# DCC Calibration
# PLUS1 + HOLD = +0.4 m/s²
# PLUS5 + HOLD = +1.2 m/s²
# MINUS1 + HOLD = -0.6 m/s²
# MINUS5 + HOLD = -1.2 m/s²

# Accel-derived setpoint (decel side).
#
# DCC's response is a clean linear function of the setpoint gap. Measured over
# routes 452 + 453 (25.6 engaged min): a_ego = 0.0935 * (setpoint − vEgo) in
# km/h on the decel side, 0.0912 on the accel side — one symmetric constant,
# linear out to a −11 km/h gap and saturating near −1.4 m/s².
#
# DCC's own speed loop is slow (implied horizon ≈ 2.8 s), so driving the
# setpoint to v_target — what this file did before — asks for a gap 1.8–2.9x
# too shallow across the entire decel range. The measured closed loop was
# a_ego = 0.611 * a_cmd: we delivered 61% of the demanded deceleration, and the
# best a_cmd → a_ego correlation sat at a 2.0 s lag. Worse, the old
# `setpoint_error < 0` gate is a *clamp*: once the setpoint reaches v_target we
# stop, whatever the demand. It blocked 82–92% of the time below a_cmd −0.9,
# and the setpoint reached that clamp in a median of 0.00 s — so neither step
# size nor cadence could ever buy authority. Matched on demand, the only times
# the car braked properly were minus5 overshoots that punched through it
# (a_cmd −0.8: clamped −0.27 m/s², deep −0.95).
#
# So invert the plant instead: ask for the gap the demanded accel needs.
#
# K_DCC is deliberately 0.1, not the measured 0.0935. That makes every ask ~7%
# shallow, landing at a flat 92–93% of demand across the whole linear range
# rather than overshooting wherever the plant is stiffer than measured.
K_DCC = 0.1                    # m/s² of DCC response per km/h of setpoint gap
SETPOINT_BIAS_MAX = 12.0       # km/h below v_target — plant floor −1.12 m/s²
SETPOINT_DEADZONE = 1.0        # km/h — one whole step; below this, don't command

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
    # Handoff latch: set once DCC has been observed to take SZL's counter back
    # (see the handoff block at the end of update). Forces the next command to
    # start a fresh burst resynced from RX instead of resuming a stale sequence.
    self.cruise_burst_released = False
    self.cruise_release_rx_cnt = -1

    # CruiseCadence — debug A/B knob, default off. 'hold' or 'single' pins the
    # stalk cadence regardless of demanded accel; anything else keeps the normal
    # accel-driven choice. It exists because openpilot picks the command AND the
    # cadence from the same demanded accel, so HOLD and SINGLE cells can never
    # be matched observationally — 46 decel bursts over 25 segments left the
    # question open (one gap bin showed HOLD much stronger, two did not, and the
    # cells differed in speed). Pinning it breaks that entanglement: drive the
    # same road once pinned 'hold' and once pinned 'single', then compare decel
    # at matched setpoint gaps.
    #
    # This changes frame SPACING only. Counter steps stay +1, so it carries none
    # of the 5ECE exposure that the 16-per-slot counter law did (see DESIGN.md).
    #
    # Restart-scoped, read once here, matching HoldHysteresis/StallBreakaway:
    # controlsd is onroad-only so the file is re-read at every drive start, and
    # a mid-drive flip would contaminate the very A/B this exists for. Import at
    # function scope so a partial deploy missing config.py just defaults to off.
    try:
      from config import read_plugin_param
      self.cruise_cadence_pin = read_plugin_param('bmw_e9x_e8x', 'CruiseCadence', '').strip().lower()
    except Exception:
      self.cruise_cadence_pin = ''
    if self.cruise_cadence_pin not in ('hold', 'single'):
      self.cruise_cadence_pin = ''
    if self.cruise_cadence_pin:
      print(f"[bmw] CruiseCadence pinned to {self.cruise_cadence_pin.upper()} - debug A/B, not for normal driving")

    # SetpointBias — accel-derived setpoint on the decel side. Default ON;
    # `echo 0 > /data/plugins-runtime/bmw_e9x_e8x/data/SetpointBias` falls back
    # to the old v_target setpoint without a redeploy, which matters because
    # this raises 0x194 commanding duty ~7x and counter faults (5ECE/CD95)
    # latch DCC off until an OBD clear. Restart-scoped like the params above.
    try:
      from config import read_plugin_param
      self.setpoint_bias_on = read_plugin_param('bmw_e9x_e8x', 'SetpointBias', '') != '0'
    except Exception:
      self.setpoint_bias_on = True
    if not self.setpoint_bias_on:
      print("[bmw] SetpointBias disabled - setpoint tracks v_target (pre-2026-09 behaviour)")

    # Debt ledger — km/h of setpoint we have taken and not yet given back.
    # Every downward bias is borrowed from the driver's set speed and has to be
    # repaid, because a setpoint left low keeps braking: the plant is symmetric.
    # Accounted at the call sites rather than inferred from cruiseState.speed,
    # so a step DCC silently drops is still owed. Capped at SETPOINT_BIAS_MAX
    # so no accounting slip can accumulate into a large repay.
    self.setpoint_debt = 0.0

    self.cruise_bus = CanBus.PT_CAN
    if CP.flags & BmwFlags.DYNAMIC_CRUISE_CONTROL:
      self.cruise_bus = CanBus.F_CAN

    self.packer = CANPacker(dbc_name[Bus.pt])

  def pin_cadence(self, interval):
    """Apply the CruiseCadence debug override to a demand-chosen interval."""
    if self.cruise_cadence_pin == 'hold':
      return HOLD_INTERVAL
    if self.cruise_cadence_pin == 'single':
      return SINGLE_INTERVAL
    return interval

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

      # Sync TX counter from RX on burst start; within a burst, carry our
      # independent sequence forward so DCC's "must advance" check is satisfied
      # even if stock's intermittent ticks have rotated rx. A burst is over
      # either because nothing was sent for BURST_LIVE_WINDOW, or because the
      # handoff latch fired — the latter is the one that matters in practice,
      # since BURST_LIVE_WINDOW (0.5 s) outlives SZL's 200 ms idle slot.
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
      # Count one slot per lead-window entry edge — the lead-window TX is the
      # frame that overwrites that slot's stock idle.
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

    # The ledger is only ours while the setpoint is only ours. A driver on the
    # stalk is setting the speed themselves, and repaying over that would fight
    # them — note v_cruise does NOT track their presses while openpilot is
    # disengaged (_update_v_cruise_non_pcm returns early when not enabled), so
    # there is no live signal to reconcile against. Drop the debt and let their
    # value stand. Losing cruiseState.available (ignition, main switch) means
    # the setpoint memory is gone too, so nothing is owed.
    if cruise_stalk_human_pressing or not CS.out.cruiseState.available:
      self.setpoint_debt = 0.0

    if not cruise_stalk_human_pressing and CS.out.cruiseState.enabled:
      if self.cruise_cancel:
        cruise_cmd(CruiseStalk.cancel, SINGLE_INTERVAL)
      elif CC.enabled:
        if CS.out.gasPressed:
          cruise_cmd(CruiseStalk.plus1, self.pin_cadence(SINGLE_INTERVAL))
        else:
          # Setpoint target. Decel side only: whenever accel >= 0 this is
          # v_target, so the accel branch below is bit-identical to before.
          #
          # v_target is long_plan.vTarget — the MPC trajectory speed at
          # action_t (~0.5 s ahead), not the driver's set speed (that is
          # CS.out.vCruise). It sits close to vEgo: median +1.1 km/h, p10 −2.1,
          # p90 +3.7. So min(v_target, ...) is a ceiling just above current
          # speed, and because the new target is min'd against the old one this
          # law can only ever ask for a *deeper* setpoint than before, never a
          # shallower one. Staying under the driver's set speed is still the
          # planner's job, exactly as before: it caps v_target at v_cruise.
          #
          # The min_cruise_setpoint floor stays with the branch guard below.
          sp_target = v_target
          if self.setpoint_bias_on and accel < 0:
            bias = max(accel / K_DCC, -SETPOINT_BIAS_MAX) * CV.KPH_TO_MS
            sp_target = min(v_target, v_current + bias)

          setpoint_error = sp_target - CS.out.cruiseState.speed
          setpoint_deadzone = SETPOINT_DEADZONE * CV.KPH_TO_MS

          # Dropping v_error from the decel gate is part of the new law, not a
          # tidy-up: it used to block 51% of the a_cmd −0.4..−0.3 band, because
          # sitting near the plan's target speed is not a reason to ignore a
          # planner asking for deceleration. The setpoint deadzone replaces it
          # — it asks the question that actually matters (is a whole step of
          # setpoint worth moving?) and is what keeps a noisy accel from
          # churning commands. With the bias off this must reduce to exactly
          # the pre-2026-09 gate, v_error term included, so that
          # SetpointBias=0 is a true rollback and not a third behaviour.
          if self.setpoint_bias_on:
            decel_gate = setpoint_error < -setpoint_deadzone
          else:
            decel_gate = v_error < -V_ERROR_DEADZONE and setpoint_error < 0

          if v_error > V_ERROR_DEADZONE and accel > 0 and setpoint_error > 0:
            cmd = CruiseStalk.plus5 if accel >= ACCEL_STEP5_THRESHOLD else CruiseStalk.plus1
            interval = self.pin_cadence(HOLD_INTERVAL if accel >= ACCEL_HOLD_THRESHOLD else SINGLE_INTERVAL)
            if cruise_cmd(cmd, interval):
              self.setpoint_debt = max(0.0, self.setpoint_debt - (5 if cmd == CruiseStalk.plus5 else 1))

          elif accel < 0 and decel_gate and CS.out.cruiseState.speed > self.min_cruise_setpoint:
            headroom_kmh = (CS.out.cruiseState.speed - self.min_cruise_setpoint) * 3.6
            cmd = CruiseStalk.minus5 if -accel >= DECEL_STEP5_THRESHOLD else CruiseStalk.minus1
            interval = self.pin_cadence(HOLD_INTERVAL if -accel >= DECEL_HOLD_THRESHOLD else SINGLE_INTERVAL)
            step = 5 if cmd == CruiseStalk.minus5 else 1
            if headroom_kmh >= step and cruise_cmd(cmd, interval):
              self.setpoint_debt = min(SETPOINT_BIAS_MAX, self.setpoint_debt + step)

          # Restore. The bias is a debt: the setpoint is parked below where the
          # demand now justifies, and every path out of a decel leaves it there
          # (the plant is symmetric, so a setpoint left low keeps braking).
          # Walking back with plus1 at SINGLE holds the setpoint <= 1-2 km/h
          # above vEgo, so the restore transient is <= 0.19 m/s² — imperceptible
          # — and costs 1.0 s at the p90 debt of 5.2 km/h, 3.2 s at the worst
          # observed 16 km/h. plus5 would repay it in 0.2 s but park the
          # setpoint ~12 km/h high on the way, a +1.1 m/s² lurch.
          #
          # This is the whole recovery mechanism for the 96% of decel episodes
          # that end normally: sp_target rises back to v_target on its own as
          # demand releases, and this branch follows it. Only exits that stop
          # us commanding entirely (disengage, brake) need the debt ledger.
          elif self.setpoint_bias_on and setpoint_error > setpoint_deadzone:
            if cruise_cmd(CruiseStalk.plus1, self.pin_cadence(SINGLE_INTERVAL)):
              self.setpoint_debt = max(0.0, self.setpoint_debt - 1)

      # Repay while openpilot is not driving. The branches above only run with
      # CC.enabled, so every exit that stops us commanding — openpilot
      # disengaging, the driver braking — parks the debt in DCC's setpoint
      # memory and leaves it there. The driver then resumes expecting their set
      # speed and gets one up to SETPOINT_BIAS_MAX low, with the car braking
      # into it.
      #
      # Repaying is not a new action, it is undoing one of ours, so it is not
      # the kind of button use the HMI rule forbids. It is bounded by the
      # ledger, gentle (plus1 at SINGLE, <=0.19 m/s2), and yields immediately:
      # the enclosing guard drops it the moment the driver touches the stalk.
      #
      # In practice this fires on the DCC rising edge. An openpilot disengage
      # raises cruise_cancel, which takes DCC to standby before we can repay
      # anything, so the debt waits — it survives standby because only losing
      # cruiseState.available clears it — and is settled when the driver
      # brings DCC back.
      elif self.setpoint_debt >= SETPOINT_DEADZONE and self.setpoint_bias_on:
        if cruise_cmd(CruiseStalk.plus1, SINGLE_INTERVAL):
          self.setpoint_debt = max(0.0, self.setpoint_debt - 1)

    # Trailing counter overwrite. If commanding stopped (or is briefly idle in
    # a deadzone) but the burst is still live, keep transmitting at the burst's
    # cadence with neutral act=0 frames until SZL's natural counter has caught
    # up enough that handoff back to stock is a forward step. Yields the bus
    # immediately when the driver is on the stalk.
    burst_alive = self.tx_cruise_stalk_counter >= 0 \
                  and (now_nanos - self.last_cruise_tx_timestamp) / 1e9 < BURST_LIVE_WINDOW
    if burst_alive and not cruise_stalk_human_pressing and not cruise_burst_release_safe():
      cruise_cmd(None, self.cruise_burst_interval)

    # Burst handoff detection. cruise_burst_release_safe() means "if SZL's idle
    # frame lands now, DCC accepts it as a forward step". So the moment we fall
    # silent with release safe — or yield the bus to the driver — SZL's next
    # tick becomes DCC's accepted counter, and our private sequence is left
    # BEHIND it. Resuming on that stale sequence is a counter rollback, the
    # 5ECE case: measured 14 times in 4 minutes on route 444 (e.g. a 280 ms
    # pause where SZL had reached 9 and we resumed at 3, delta 9 — outside
    # DCC's accepted [1, 7] window). BURST_LIVE_WINDOW alone cannot catch this,
    # because at 0.5 s it outlives SZL's 200 ms idle slot.
    #
    # Two-step latch: arm on the first silent cycle, fire once SZL's counter
    # actually moves while we are still silent. Arming is cleared by any TX, so
    # the mid-burst gaps between our own 20-40 Hz frames never trip it — only a
    # real silence spanning an SZL tick does.
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
