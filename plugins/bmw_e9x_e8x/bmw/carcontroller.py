from collections import deque

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
DECEL_STEP5_THRESHOLD = 0.9    # m/s² — use -5 above this, -1 below (midpoint of 0.6–1.2).
                               # Only the SetpointBias=0 rollback path uses this and
                               # DECEL_HOLD_THRESHOLD now; the bias path is minus1-only.

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

# Concordance gate on entering/leaving the braking bias.
#
# Route 454 drove the bias off a bare `accel < 0` sign test and it chattered:
# the setpoint changed direction 19.3 times a minute against the old law's 4.6,
# burned 2.75x the bus, and delivered *less* deceleration (50% of demand against
# the old law's 69%) because minus1 and plus1 bursts cancelled — 640 against 810
# in half an hour. a_cmd crosses zero 22 times a minute, and every crossing
# swapped the target between `vEgo + bias` and `v_target`.
#
# Two independent estimates of the same intent are available:
#   accel        actuators.accel, LongControl's output and already filtered —
#                sd 0.046 m/s² against a 2 s centred mean of itself
#   a_dv         the raw plan's vTarget differentiated over DV_WINDOW —
#                sd 0.221, noisier, but its noise comes from somewhere else
# They disagree in sign on 18.8% of samples yet agree ~100% once accel < -0.3:
# they diverge where the noise is and converge where the demand is real. That
# is what makes requiring agreement work here, and it is why the earlier
# attempt with v_error did not — v_error carries our own braking back through
# vEgo, so gating on it is negative feedback on the thing being sustained.
#
# a_dv is ONLY the concordance check. The bias magnitude stays raw accel:
# taking min(accel, a_dv) simulates better still (75% against 61%) but that is
# a deliberate brake-to-the-more-pessimistic-estimate policy, not noise
# rejection, and it is not being smuggled in as a tuning win.
#
# The rule needs no thresholds. Both negative -> brake; both positive ->
# accelerate; disagreement -> hold whatever state we are in. The hysteresis
# falls out of the disagreement region, which is exactly the band where the
# noise lives, so it is self-sizing rather than tuned.
#
# Modelled on 454: flips 18.6 -> 8.9/min, commanding 1658 -> 1487 moves/min,
# gain 61% -> 57% of demand, unwanted braking -0.002 m/s².
DV_WINDOW = 0.30               # s — 6 modelV2 frames at 20 Hz

# Command selection: pick the largest step whose measured yield fits the error
# that is left, one decision per SZL slot.
#
# What a burst is worth was measured per burst, not per frame or per second —
# neither cadence nor hold length moves it. Over 4 routes, a minus1 burst drops
# the setpoint 1 km/h and a minus5 burst 10 km/h (median; 2-frame and 3-frame
# bursts both land there, and a 1-frame burst has n=1, so the 10 is not dialable
# down by shortening it).
#
# Deciding once per 200 ms slot matters because 0x193 reports the setpoint back
# at only ~5 Hz: without it a 10 km/h error draws minus5 twice before the first
# one is visible and overshoots by a whole yield, 0.9 m/s² of braking nobody
# asked for. setpoint_pending carries what has been commanded since the last
# fresh report for the same reason.
#
# Closed-loop bench over 53 real episodes from 452/453/454 (planner surrogate,
# measured plant, 5 Hz observation), validated by minus1-only reproducing the
# 50% of demand actually measured on route 454:
#
#   minus1 only              50% of demand   median overshoot 1.6 km/h (0.15 m/s²)
#   minus5 at err >= 5      *68%*                             3.5 km/h (0.33)
#   minus5 at err >= 10      54%                              1.6 km/h (0.15)
#
# 5 km/h is chosen knowing it overshoots by construction — minus5's yield is
# 10 km/h against a 12 km/h total bias budget, so any threshold low enough to be
# useful overshoots. A threshold of 10 is overshoot-free and recovers almost
# nothing. The overshoot is in the safe direction, is pulled back by the restore
# branch, and minus5 fires only about once a minute, so it stays a rare
# intervention for the hard cases rather than a routine command.
DECEL_STEP5_KMH = 5.0          # km/h of remaining error at or above which minus5 is used
MINUS5_YIELD_KMH = 10.0        # measured median setpoint drop from one minus5 burst
MINUS1_YIELD_KMH = 1.0
PENDING_TIMEOUT = 0.5          # s — give up on what was sent and re-command.
                               # Without it, a DCC that stops acting on us never
                               # moves the setpoint, so the reading never changes,
                               # so pending never clears and we fall silent for
                               # good. 0.5 s is 2-3 report periods: anything sent
                               # has either landed or been lost by then.

# Step size keys on the setpoint error, not on accel. Under the old clamped
# setpoint the two were nearly the same question, because the setpoint could
# never get further from v_target than the plan already was. This law breaks
# that: the setpoint can already be deep (nothing to do) while demand is large,
# or sitting high (10 km/h to go) while demand is mild. Measured against what
# the setpoint actually had to move, the accel-keyed rule agreed only 35.7% of
# the time and was too timid in 64.1% of cases — minus1 where >= 3 km/h was
# needed — against 0.2% the other way.
#
# It also costs duty, which is the counter-overwrite exposure that matters
# here: 7.88 presses to close the gap accel-keyed against 2.41 error-keyed.
#
# The decel bias is built with minus1 alone, asserted at HOLD.
#
# minus5 is parked, not deleted, and the numbers for bringing it back are here.
# Measured over routes 452/453/44b, simulating each option's slew limit against
# the real a_cmd traces:
#
#                       delivered   blind-window commit   sustains   builds 9.1 km/h
#   old clamped law         61%            --                --            --
#   minus1 only             68%      0.9 km/h = 0.09 m/s2   1.28 m/s2      2.0 s
#   minus1 + minus5         78%      6.7 km/h = 0.63 m/s2   9.17 m/s2      0.3 s
#
# minus5 buys transient response, not ceiling: minus1 alone already sustains
# 1.28 m/s², above SETPOINT_BIAS_MAX's 1.12. What it costs is overshoot — the
# setpoint is only reported back on 0x193 at ~5 Hz, so we command blind for up
# to 200 ms, and minus5 is accepted on ~67% of frames at 5 km/h each. minus1's
# exposure over the same window is 7x smaller, and carries no model risk on
# that acceptance figure.
#
# HOLD rather than SINGLE is a deliberate call. DCC's minus1 step rate does NOT
# track our frame rate — measured 4.76 steps/s at 15-25 Hz, 4.10 at 38-60 Hz,
# and what predicts the step count is how long the bit is ASSERTED (R2 0.82)
# rather than how many frames carry it (R2 0.73). So HOLD costs ~2x the frames
# on the 0x194 counter-overwrite axis for no measured gain in slew; it is taken
# for robustness of the assertion against dropped frames, which is not
# something the logs can settle either way.

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

    # Concordance state: vTarget history for a_dv, and the braking latch.
    self.v_target_hist = deque(maxlen=int(round(DV_WINDOW / DT_CTRL)) + 1)
    self.setpoint_braking = False

    # Per-slot command selection: what was decided this slot, and how much
    # setpoint we have asked for but not yet seen arrive.
    self.slot_cmd = None
    self.slot_decided_ns = 0
    self.setpoint_pending = 0.0
    self.setpoint_pending_ns = 0
    self.setpoint_last_seen = None

    # Hand-back point — the driver's set speed in km/h, latched while openpilot
    # is driving. Every downward bias is borrowed from it and has to be given
    # back, because a setpoint left low keeps braking: the plant is symmetric.
    #
    # Latched rather than read live because v_cruise only tracks the driver's
    # stalk presses while openpilot is enabled (_update_v_cruise_non_pcm returns
    # early otherwise), so the value is only trustworthy at the moment we still
    # have the car. Zero means nothing is owed.
    self.setpoint_handback_kmh = 0.0
    self.setpoint_debt = 0.0     # derived each cycle, kept for logging and tests

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

    # Concordance gate. Both estimates must agree before the bias is taken on
    # or given up. Until DV_WINDOW of history exists a_dv is unavailable, so
    # fall back to accel alone rather than refusing to brake.
    self.v_target_hist.append(v_target)
    if len(self.v_target_hist) == self.v_target_hist.maxlen:
      a_dv = (v_target - self.v_target_hist[0]) / DV_WINDOW
    else:
      a_dv = accel
    if not CC.enabled:
      self.setpoint_braking = False
      self.v_target_hist.clear()
    elif accel < 0 and a_dv < 0:
      self.setpoint_braking = True
    elif accel > 0 and a_dv > 0:
      self.setpoint_braking = False
    # else: the two estimates disagree — hold the state we are already in.

    # A changed setpoint reading is a fresh 0x193 report, and it already
    # contains everything we have sent, so nothing is in flight any more.
    if CS.out.cruiseState.speed != self.setpoint_last_seen:
      self.setpoint_last_seen = CS.out.cruiseState.speed
      self.setpoint_pending = 0.0
    elif (now_nanos - self.setpoint_pending_ns) / 1e9 > PENDING_TIMEOUT:
      self.setpoint_pending = 0.0
    if not self.setpoint_braking:
      self.setpoint_pending = 0.0
      self.slot_cmd = None

    cruise_stalk_human_pressing = CS.cruise_stalk_resume or CS.cruise_stalk_cancel or CS.cruise_stalk_speed != 0

    # Latch the hand-back point while we still have the car. Range-checked the
    # same way register.py's cruise-ceiling memory does, so an unset or garbage
    # v_cruise can never become a repay target.
    if CC.enabled and CS.out.cruiseState.enabled and 30.0 <= CS.out.vCruise <= 145.0:
      self.setpoint_handback_kmh = CS.out.vCruise

    # The claim is only ours while the setpoint is only ours. A driver on the
    # stalk is setting the speed themselves and repaying over that would fight
    # them, and there is no live v_cruise to reconcile against once openpilot is
    # out — so drop the claim and let their value stand. Losing
    # cruiseState.available (ignition, main switch) takes the setpoint memory
    # with it, so nothing is owed either.
    if cruise_stalk_human_pressing or not CS.out.cruiseState.available:
      self.setpoint_handback_kmh = 0.0

    # Debt is *measured*, not accounted. An earlier version counted transmitted
    # frames, which pinned it to the cap after 4 frames (60 ms) because DCC only
    # steps the setpoint once per 200 ms slot, and made the repay declare itself
    # settled after 0.6 s having actually returned about 3 km/h. Reading the
    # setpoint back instead is self-correcting: a step DCC drops is still
    # visible as debt, and the repay simply continues.
    self.setpoint_debt = 0.0
    if self.setpoint_bias_on and self.setpoint_handback_kmh > 0.0:
      self.setpoint_debt = min(SETPOINT_BIAS_MAX,
                               self.setpoint_handback_kmh - CS.out.cruiseState.speed * 3.6)

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
          if self.setpoint_bias_on and self.setpoint_braking:
            # min(accel, 0) so a positive blip while latched holds the bias
            # rather than releasing it — the latch is what decides to release.
            bias = max(min(accel, 0.0) / K_DCC, -SETPOINT_BIAS_MAX) * CV.KPH_TO_MS
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
            decel_gate = self.setpoint_braking and setpoint_error < -setpoint_deadzone
          else:
            decel_gate = v_error < -V_ERROR_DEADZONE and setpoint_error < 0

          if v_error > V_ERROR_DEADZONE and accel > 0 and setpoint_error > 0:
            cmd = CruiseStalk.plus5 if accel >= ACCEL_STEP5_THRESHOLD else CruiseStalk.plus1
            interval = self.pin_cadence(HOLD_INTERVAL if accel >= ACCEL_HOLD_THRESHOLD else SINGLE_INTERVAL)
            cruise_cmd(cmd, interval)

          elif accel < 0 and decel_gate and CS.out.cruiseState.speed > self.min_cruise_setpoint:
            headroom_kmh = (CS.out.cruiseState.speed - self.min_cruise_setpoint) * 3.6
            if self.setpoint_bias_on:
              # One decision per SZL slot; hold it for the rest of the slot so
              # the burst is long enough for DCC to act on (a sub-0.06 s
              # assertion produced nothing 80% of the time).
              if (now_nanos - self.slot_decided_ns) / 1e9 >= CRUISE_STALK_IDLE_TICK_STOCK:
                self.slot_decided_ns = now_nanos
                err_kmh = -setpoint_error * 3.6 - self.setpoint_pending
                if err_kmh >= DECEL_STEP5_KMH and headroom_kmh >= 5:
                  self.slot_cmd = CruiseStalk.minus5
                  self.setpoint_pending += MINUS5_YIELD_KMH
                  self.setpoint_pending_ns = now_nanos
                elif err_kmh >= SETPOINT_DEADZONE and headroom_kmh >= 1:
                  self.slot_cmd = CruiseStalk.minus1
                  self.setpoint_pending += MINUS1_YIELD_KMH
                  self.setpoint_pending_ns = now_nanos
                else:
                  self.slot_cmd = None
              if self.slot_cmd is not None:
                cruise_cmd(self.slot_cmd, self.pin_cadence(SINGLE_INTERVAL))
            else:
              use_step5 = -accel >= DECEL_STEP5_THRESHOLD
              cmd = CruiseStalk.minus5 if use_step5 else CruiseStalk.minus1
              interval = self.pin_cadence(HOLD_INTERVAL if -accel >= DECEL_HOLD_THRESHOLD else SINGLE_INTERVAL)
              step = 5 if use_step5 else 1
              if headroom_kmh >= step:
                cruise_cmd(cmd, interval)

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
            cruise_cmd(CruiseStalk.plus1, self.pin_cadence(SINGLE_INTERVAL))

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
      elif self.setpoint_debt >= SETPOINT_DEADZONE:
        cruise_cmd(CruiseStalk.plus1, SINGLE_INTERVAL)

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
