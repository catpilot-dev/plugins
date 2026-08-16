"""BMW E9x/E8x lateral controller — front-wheel-angle plant-inversion.

Hook target for controls.lat_controller_init (see plugin.json): monkey-patches
lac.update with the custom controller. Design & tuning reference:
LATERAL_CONTROLLER.md in this directory.

Split out of register.py 2026-07-03; register.py keeps interface
registration and the small hooks; all lateral control lives here.

Lives in bmw/ with the rest of the port for now. Planned future refactor:
extract into a separate lateral *curvature* control plugin, complementing
upstream's lateral angle and torque controllers.

Loaded by the plugin registry as `plugins.bmw_e9x_e8x.bmw.latcontroller`
(plugin.json nested-module mapping). Do not also import it as
`bmw.latcontroller` from other modules — that would create a second module
instance with its own state dict (registry module-identity lesson).
"""
import os
import sys
from collections import deque

# Ensure the plugin root is importable so `from bmw.values import ...` works
# regardless of module load order (this file lives in bmw/, so the plugin
# root is the parent directory).
_PLUGIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PLUGIN_DIR not in sys.path:
  sys.path.insert(0, _PLUGIN_DIR)

import numpy as np

# SAFETY ARCHITECTURE (2026-07-28): the lateral controller NEVER gives up in a
# turn — its contract is to track the commanded curvature, always. Keeping
# lateral acceleration within ISO 11270 / comfort limits is handled at the
# SYSTEM level by speedlimitd, which caps vEgo for curves (a_y = v²·κ, so it
# owns the v² factor). Draining torque mid-turn to respect an a_y ceiling here
# converted a comfort exceedance into a trajectory failure — the car ran wide.
# The ISO accel/jerk cancel machinery that used to live in this file
# (accel_guard_threshold, ACCEL_GUARD_*, the overshoot gate, cancel_accel /
# cancel_jerk drains, ISO_LATERAL_ACCEL/JERK) was removed on this date. The
# incident record shows every cancel firing caused harm and none prevented any:
#   route 326                — spurious cancels, torque churn per lane change
#   route 385 seg 27         — over-latch → cancel_accel dump → deep unwind cycle
#   route 2ba seg 22         — cancel zeroed torque under-tracking → 1.29 m off-lane
#   route 3ce (segs 15/26/31) — drain-rebuild hunting, 54% zero-torque in curves
#   route 3cf seg 15         — cancel firing through 75% of a sharp curve
# Remaining lateral bounds, all of which track-toward-command rather than
# abandon it: the P-law naturally REVERSES on overshoot (tracking back to the
# commanded κ IS the correction), the STEP_MAX per-decision jerk bound, the
# panda STEER_MAX hard limit, and driver supervision. a_y is not this layer's
# concern; it belongs to speedlimitd's curve-speed capping.

# Hold gate, lateral-acceleration-dependent (2026-07-27, route 3ca seg 23).
# Whether "drain to zero and let stiction hold" is safe on-target depends on
# self-aligning torque (SAT), which scales as v²·κ — NOT on κ alone. The
# original gate (2026-07-03) used |κ_des| directly, encoding the ~12 m/s
# tuning speed of the route 380/384 hairpin fix into a curvature threshold
# that silently assumed a fixed speed. Route 3ca seg 23 broke that
# assumption: 19.4 m/s, κ 0.0033, a_y 1.25 sat BELOW the old
# HOLD_KAPPA_BP[0]=0.004 → hold_f=0 → drain → 0.6 Hz-class hunting (30%
# zero-torque decisions in a sustained turn, 1.8× command-wobble
# amplification) — a mild-but-fast curve with plenty of SAT to unwind the
# wheel, misclassified as "straight" by the κ-only gate.
# Reference points:
#   route 3ca seg 23  (19.4 m/s, κ 0.0033, a_y 1.25)  → must hold (was broken)
#   reference mild curve (12.4 m/s, κ 0.0023, a_y 0.35) → damps fine with drain
#   route 380/384 hairpin fix operating point (a_y ≥ 0.97) → must keep FULL hold
# HOLD_AY_BP = [0.5, 0.9] preserves full hold at the 380/384 operating point,
# fixes the 3ca seg 23 mild-fast-curve drain, keeps drain on straights and
# gentle-slow curves (a_y < 0.5), and drops hold at parking speeds where SAT
# is far below stiction and drain is safe regardless of κ.
# hold_f is functionally near-binary in practice: the held target is
# re-derived from state['torque'] every 100 ms-class decision tick, so
# mid-range (partial) hold_f values only persist for the one decision they're
# computed on — they don't accumulate or hold state, and decay sub-second.
HOLD_AY_BP = [0.5, 0.9]  # m/s² of commanded a_y = v²·|kappa_des|


def hold_factor(v_ego, kappa_des_abs):
  """Curvature-hold blend factor [0, 1], gated on commanded lateral accel.

  0 → pure stiction-hold (drain torque to 0, on-target).
  1 → keep the standing torque (it approximates self-aligning torque, which
      beats rack stiction at this loading).
  Pure function of (v_ego, |kappa_des|) — no state — so it's directly
  testable without constructing the controller.
  """
  return float(np.interp(v_ego * v_ego * kappa_des_abs, HOLD_AY_BP, [0.0, 1.0]))


# Stall/breakaway v2 (2026-08-15, route 3f2+3f4 replay-validated — see
# LATERAL_CONTROLLER.md dated section and stall-breakaway-v2-design.md).
# A stalled rack that releases sweeps the wheel far past what the model is
# asking for while the controller still pushes on a stale kappa-space error
# (every kappa-derived signal lags the rack by the vehicle response). The
# release signature is DISPLACEMENT-SINCE-BREAKAWAY plus RATE — nothing
# absolute. The stall context is what makes that sufficient: arming already
# proves windup exists, and the same stall condition balances the tire-slip
# term that an absolute target comparison would have to model (user ruling:
# no upstream vehicle model here, too many unknown parameters).
# Everything is measured against the wheel's own breakaway state, so there
# is NO absolute angle/curvature polarity convention anywhere in this
# machine — sb_dir is the observed motion direction, self-referenced.
# Provenance of the two trip thresholds:
#   SB_TRIP_DISP_DEG 2.0  — user-specified ("additional 2 degrees of
#     steering wheel motion" past the point the rack broke free).
#   SB_TRIP_RATE_DPS 30.0 — rate sweep on 3f4 (86 segs, 85 min ordinary
#     driving): >=25 leaves 0.31 trips/min, >=30 leaves 0.153, and adding
#     the deep-curve gate below brings it to 0.071/min. Ordinary
#     post-stick corrections cluster below 30 deg/s; the 3f2 release
#     sweeps 31-70.
#   SB_TRIP_KAPPA_MAX 0.010 — same value as the deep-curve doctrine
#     threshold (RELAX_DWELL_KAPPA). In a deep curve SAT is strong enough
#     to self-arrest a release, measured: hairpin segments tracked fine and
#     dominated the false trips before this gate. The 3f2 lurch was a MILD
#     curve (kappa_des -0.0035) where SAT could not arrest it.
# Replay: 3f2 seg 10 trips once at t=664.20, crossing at 31 deg/s, 0.30 s
# before the 24.8 deg peak; every other 3f2 segment trips zero times.
ANGLE_QUANTUM_DEG = 0.04395   # steerAngleDeg LSB
SB_FROZEN_TICKS = 40          # 0.4 s @100 Hz, span < 2 quanta = frozen (arm)
SB_MOVE_TICKS = 20            # 0.2 s window: breakaway + rate measurement
SB_MOVE_QUANTA = 3            # >= 3 quanta advance = breakaway (creep is 1-2)
SB_EPISODE_TICKS = 200        # 2 s max episode after breakaway
SB_TRIP_DISP_DEG = 2.0        # wheel travel past the breakaway point
SB_TRIP_RATE_DPS = 30.0       # sweep speed over the 0.2 s window
SB_TRIP_KAPPA_MAX = 0.010     # |kappa_des| above this: SAT self-arrests, stay out
SB_TRIP_MIN_TORQUE = 0.12   # frac (~1.4 Nm): a release without windup worth shedding
                            # must not trip — 8 of 9 on-car benign trips (3fa/3fb)
                            # were corner-exit unwinds at 0.007-0.088; the 3f2 real
                            # event carried 0.293. (2026-08-16)
SB_SHED_FRAMES = 10           # drain torque -> 0 over 100 ms on trip
SB_BLOCK_TICKS = 50           # 0.5 s same-side push suppression after trip


def on_lat_controller_init(result, lac, CP):
  """Plant-inversion at 500 ms horizon in front-wheel-angle space.

  BMW E90 hydraulic rack has high breakaway friction and no alignment-torque
  self-centering — the wheel holds its angle at zero torque *near center*.
  In curves, self-aligning torque exceeds rack stiction above ~40° wheel
  (route 380/384), so on-target there requires standing torque. So:
    - On-target, low commanded a_y (v²·|κ_des| < HOLD_AY_BP[0]): drive
      torque → 0 and let stiction hold. No chatter. Gated on a_y, not κ
      alone — SAT scales as v²·κ (2026-07-27, route 3ca seg 23).
    - On-target, higher commanded a_y: hold hold_f·torque — the torque that
      achieved on-target ≈ the self-aligning torque. Re-derived each
      decision from state['torque']; bounded by the P-term's own command,
      no ratchet.
    - Off-target: compute the torque that would move the front wheel by
      δ_err over 500 ms (plant-inversion accounting for first-order lag), ramp
      to it over one 250 ms decision.

  Error in rear-axle bicycle-model front-wheel-angle space:
    δ_des  = atan(κ_des  · L)        L = CP.wheelbase
    δ_meas = atan(κ_meas · L)        κ_meas = yawRate / v_ego
    δ_err  = δ_des − δ_meas

  On-target trigger (Phase 2, 2026-07-22): HOLD_BAND = 0.001 rad, fixed —
  not the speed-adaptive kinematic DRIFT_M deadzone it replaced. It exists
  only to decide when the rack has arrived so the curvature hold can take
  over. It is simply a dead band of front-wheel error, sized at the error
  signal's measured noise floor (~1σ; see the HOLD_BAND constant's comment
  for the numbers — the old "sized by rack breakaway" story was a fiction
  retired 2026-08-13), not by allowed lateral drift. modelV2 noise handling
  and lateral-drift correction live upstream in the lane_keeping position
  loop. Since 2026-08-13 entry and settle are split (hysteresis):
    leave rest:   |δ_err| > HOLD_BAND_ENTER  (0.0015)
             or:  |EMA(2 s) δ_err| > HOLD_EMA_ESCAPE (0.0012) — the
                  persistent-lean escape added 2026-08-14, which bounds
                  how long a constant (road-crown) pull can sit inside
                  the entry gap uncorrected.
    return:       |δ_err| ≤ HOLD_BAND        (0.001)

  Plant-inversion target torque, angle domain (linear tire regime). P acts
  on the FULL error — no tolerance subtraction (Phase 2: it was attenuating
  the lane_keeping position correction upstream):
    τ_Nm_target = T_CAP_SLOPE · v² · δ_err
    Clamp to ±T_CAP(v, δ):
      T_CAP_NM = min(STEER_MAX, T_CAP_BASE + T_CAP_SLOPE · v²·|δ_des|)
    Same slope drives both target and cap.
    Sub-breakaway targets are commanded as-is (they don't move the rack —
    a deliberate soft actuation deadband; stiction special-casing removed
    2026-07-03). BASE is the hydraulic rack's stiction floor. Hard stop at
    STEER_MAX (panda limit) preserves lane authority during transient
    over-envelope events before speedlimitd trims v.

  Ramp: ramp_step = (T_peak − state['torque']) / spread_frames, applied
  per CAN frame for spread_frames frames; panda enforces wire-rate.
  (spread_frames = round((model_action_t/2)/DT_CAN_TICK), per-tick dynamic.)

  SAFETY ARCHITECTURE (2026-07-28): this controller NEVER abandons a turn.
  There is no ISO accel/jerk cancel here — that machinery was removed. Keeping
  lateral acceleration within ISO 11270 / comfort limits is a SYSTEM-level
  responsibility owned by speedlimitd, which caps vEgo for curves (a_y = v²·κ).
  Draining torque mid-turn to respect an a_y ceiling turned a comfort
  exceedance into a trajectory failure (the car ran wide); see the module-level
  SAFETY ARCHITECTURE note for the incident list (326, 385 seg27, 2ba 1.29 m
  off-lane, 3ce hunting, 3cf seg15) that proves every cancel was net-harmful.
  The remaining bounds all keep tracking the command rather than give up: the
  P-law naturally REVERSES on overshoot (tracking back to κ_des IS the
  correction), STEP_MAX bounds per-decision jerk, panda STEER_MAX is the hard
  limit, and the driver supervises. a_y_meas / jerk_pred are still computed but
  are telemetry-only now — nothing reads them to make a control decision.

  Relax-dwell (2026-07-12): in a measured deep curve (|κ_meas| > 0.010),
  an overshoot-side error must persist 1.0 s before the relax path may
  command below current torque — bridges modelV2's transient mid-turn κ_des
  dips (the "gives up mid-way" mechanism, route 3a0 seg 8). κ_des sign flips
  (S-curves) abort it instantly.

  cancel_tol (every livePose tick) — NOT ISO machinery, it is HOLD_BAND
  boundary hygiene: if |δ_err| drops into the on-target band (1.2 × HOLD_BAND)
  mid push-ramp, drain torque to the sign-guarded, capped hold (0 on
  straights). This tracks the command more tightly, it does not abandon it —
  without it the in-flight ramp keeps pushing toward a stale target until the
  next 250 ms cadence notices.

  2026-07-03 simplification: all stiction special-casing removed (breakaway
  ±FRICTION amplification, brake_zero reverse pulse, cancel reverse-FRICTION
  pulses). Route 384 telemetry showed ~40% of in-turn decisions were friction
  pulses — torque reversals that churned rather than corrected. The straight-
  line wobble those mechanisms targeted is modelV2 vision noise; as of Phase 2
  (2026-07-22) that's handled upstream by the lane_keeping plugin (κ_des
  low-pass + position loop), not by anything in this controller.

  No online adaptation: plant behavior is fully described by T_CAP_SLOPE,
  and T_CAP_BASE_NM. Tune these offline from route data; there's
  no scale_by_bin or shadow estimator anymore.
  """
  import math
  from cereal import log
  from cereal import messaging
  from bmw.values import CarControllerParams as CCP

  # Plugin params are read ONCE here, never per-tick and never on a periodic
  # cache expiry (review fix, Important 4): a Path.read_text() on controlsd's
  # 100 Hz RT thread can cost a control frame under eMMC contention, and even
  # a cache-timer check burns work on every tick for no benefit. The init read
  # is the BOOT state; controlsd is onroad-only, so the param file is re-read
  # at every drive start and stays authoritative across drives.
  #
  # Import at function scope, not module scope (review fix, Important 2):
  # every other config.read_plugin_param consumer in this repo does this
  # (bmw/carstate.py's _load_steer_angle_offset, speedlimitd's _read_params)
  # so a missing config.py only defaults these params instead of failing the
  # whole hook module's import — install.sh copies config.py to the
  # plugins-runtime root as a separate, later step, so a partial deploy can
  # transiently be missing it. A module-scope import failing here would
  # silently fall the car back to stock LatControlTorque, which is worse than
  # just defaulting a toggle.
  #
  # HoldHysteresis kill-switch (2026-08-13, route 3f4 data): default ON —
  # '0' is the rollback to the legacy single threshold. No bus topic, no
  # heartbeat, no hot toggle: a restart-scoped kill-switch is enough. If the
  # import above failed, read_plugin_param is undefined here and the except
  # branch defaults hysteresis ON, matching that polarity. See
  # HOLD_BAND_ENTER below and LATERAL_CONTROLLER.md.
  #
  # StallBreakaway kill-switch (2026-08-15, stall/breakaway v2): default OFF —
  # '1' enables. Same restart-scoped polarity as the retired AngleBudget: no
  # bus topic, no heartbeat, no hot toggle (a rare-event safety net does not
  # need mid-drive flipping; the A/B is "did a windup release get caught",
  # read from telemetry). Applies at the NEXT drive start, since controlsd is
  # onroad-only. See the SB_* constants at module scope.
  try:
    from config import read_plugin_param
    _hold_hyst_on = read_plugin_param('bmw_e9x_e8x', 'HoldHysteresis', '') != '0'
    _stall_v2_on = read_plugin_param('bmw_e9x_e8x', 'StallBreakaway', '') == '1'
  except Exception:
    _hold_hyst_on = True
    _stall_v2_on = False

  # Decision cadence & CAN-rate spreading — both subscribe to model_action_t
  # per tick, sized to one half of the model's action horizon so exactly two
  # decision-and-ramp cycles fit within one horizon. This keeps the
  # controller's correction bandwidth matched to the model's planning
  # bandwidth: when CP.steerActuatorDelay changes, both (cadence, ramp) move
  # together — no parallel-constant drift. (model_action_t also sets the
  # jerk_pred telemetry horizon; jerk_pred no longer gates anything.)
  #
  #   action_cadence_ticks = round( (model_action_t / 2) / DT_LIVEPOSE )
  #   spread_frames        = round( (model_action_t / 2) / DT_CAN_TICK  )
  #
  # Each ramp completes when the next decision lands; no overlapping ramps.
  # No internal rate cap — panda enforces wire-rate (STEER_DELTA_UP =
  # 0.1 Nm/frame). For typical deltas (≤ 2.5 Nm), ramp_step ≤ 0.1 Nm/frame
  # and demand tracks rack reality. For large transients, panda clips and
  # state['torque'] briefly leads the rack — accepted; the controller keeps
  # tracking the command and catches up.
  #
  # Computed inside update() from model_action_t — see action_cadence_ticks
  # / spread_frames below. Fallbacks here are used only for state-init at
  # construction time, before any lat_delay has arrived.
  DT_LIVEPOSE = 0.05                        # one livePose tick (s)
  DT_CAN_TICK = 0.01                        # one CAN tick at 100 Hz (s)
  ACTION_CADENCE_TICKS_FALLBACK = 5         # state-init only (= 250 ms cadence)
  SPREAD_FRAMES_FALLBACK = 25               # state-init only (= 250 ms ramp)
  # T_CAP_SLOPE: aligning-torque gain (κ-independent). Linear tire regime:
  #     τ_Nm_hold = T_CAP_SLOPE · v² · δ                (aligning torque)
  # Drives both authority cap and target torque:
  #   T_CAP(v, δ)  = T_CAP_BASE_NM + T_CAP_SLOPE · v² · |δ_des|   (≤ STEER_MAX)
  #   target_Nm    = T_CAP_SLOPE · v² · delta_err   (Phase 2: full error, no soft-deadband)
  # BASE covers the speed- and angle-independent stiction floor.
  # T_CAP_SLOPE_BASE = 1.0: gentle baseline gain on straights. A curvature-
  # dependent scale T_CAP_SCALE(|κ_des|) bumps it up to 3.0× on tight curves
  # (linear interp 0.001..0.01 1/m). Rationale: small κ_des needs gentle gain
  # to avoid ringing on near-straight sections (seg-14 evidence); tight κ_des
  # needs enough authority to chase the planner without lag (seg-6 evidence).
  # The per-tick cancel_tol guard (HOLD_BAND-based) handles boundary
  # smoothness; there is no soft-deadband since Phase 2 (2026-07-22).
  T_CAP_BASE_NM = 2.0
  T_CAP_SLOPE_BASE = 1.0
  T_CAP_SCALE_KAPPA = [0.001, 0.01, 0.02]        # |κ_des| breakpoints (1/m)
  T_CAP_SCALE_BP    = [1.0, 2.5, 3.0]           # scale factor on T_CAP_SLOPE_BASE
  # Plant prediction horizon — sourced per-tick from controlsd's lat_delay
  # (= liveDelay.lateralDelay + LAT_SMOOTH_SECONDS) and matched to where
  # modeld samples κ_des: lat_action_t = lat_delay + DT_MDL (modeld.py:391).
  # Used by the hold-cap's fixed-reference drift formula (HOLD_CAP_DRIFT_M
  # block below) and by the jerk_pred telemetry (log-only since 2026-07-28;
  # no longer an overshoot guard).
  #
  # For BMW: lagd never converges (it correlates on latcontrol_torque
  # telemetry that our front-wheel-angle plant-inversion controller doesn't
  # produce), so liveDelay.lateralDelay is permanently pinned at initial_lag
  # = CP.steerActuatorDelay + 0.2 (lagd.py:181). CP.steerActuatorDelay in
  # bmw/interface.py is therefore the single knob controlling both modeld's
  # lat_action_t AND our model_action_t — change it there to retune both.
  DT_MDL = 0.05                                  # openpilot model dt (common/realtime.py)
  MODEL_ACTION_T_FALLBACK = 0.55                 # used only if lat_delay arrives as 0/None

  # Curvature-dependent hold: SEE HOLD_AY_BP / hold_factor() at module scope
  # (top of file). The gate moved from |κ_des| to commanded lateral accel
  # (v²·|κ_des|) on 2026-07-27 — SAT that determines whether stiction can
  # safely hold the wheel scales with v²·κ, not κ alone; see the module-level
  # comment for the route 3ca seg 23 evidence and reference operating points.
  # Reference drift scale for the hold-torque cap ONLY (decoupled from the
  # kinematic DRIFT_M deadzone on 2026-07-04 — see the hold_cap block in
  # update()). That DRIFT_M deadzone was deleted entirely in Phase 2
  # (2026-07-22); this fixed reference is unaffected and unchanged.
  HOLD_CAP_DRIFT_M = 0.10

  # Per-decision torque step cap (2026-07-03, route 385 seg 27 review).
  # Human-style gradual steering: each cadence decision moves the target at
  # most step_max from the current torque, the plant responds (~2.5τ per
  # cadence), the next decision re-measures and steps again. Design rule:
  # never apply excessive steering torque abruptly. 0.10 frac = 1.2 Nm per
  # 300 ms ≈ 4 Nm/s max slew (panda wire limit is 10 Nm/s). Full authority
  # builds in ~1.5 s instead of 0.3 s — accepted: speedlimitd slows for
  # curves and the P-law reverses its TARGET the moment the plant overshoots
  # κ_des (the executed unwind is STEP_MAX-rate-limited — deliberately slower
  # than the removed drain, which is what ran the car wide; see §7)
  # (tracking back to the command). First knob to revisit if curve entries
  # ever feel late.
  #
  # 2026-07-09 (route 39b seg 18, user safety call): SPEED-SCALED. A slight
  # highway left showed sudden back-and-forth wheel motion; aggressive
  # per-decision steps are riskier at highway speed (lane margin consumed
  # faster, less time to react). Full 0.10 up to 15 m/s (curves keep their
  # entry authority; speedlimitd owns curve-entry speed), tapering to 0.05
  # at/above 28 m/s (100 km/h) — highway slew halves to ~2 Nm/s.
  # (Historical note: at the time this was written, the deadzone was the
  # 1/v²-tight kinematic DRIFT_M, deliberately not tightened further at
  # speed. Phase 2 (2026-07-22) deleted that deadzone entirely; HOLD_BAND
  # is now a FIXED 0.001 rad with no speed adaptation — at v=28 m/s it is
  # roughly 3× WIDER than the old tol_kin was, not tighter. Speed-adaptive
  # drift correction now lives in lane_keeping's position loop, not here.)
  STEP_MAX_V  = [15.0, 28.0]     # vEgo breakpoints (m/s)
  STEP_MAX_BP = [0.10, 0.05]     # per-decision torque step cap (frac)

  # Relax-dwell (2026-07-12, route 3a0 seg 8): in a measured deep curve, an
  # overshoot-side error must persist this long before the relax path may
  # command below current torque — bridges modelV2's transient mid-turn
  # κ_des dips (40-50% for ~1 s, κ_meas steady). See deep_relax in update().
  RELAX_DWELL_TICKS = 20      # livePose ticks (1.0 s at 20 Hz)
  RELAX_DWELL_KAPPA = 0.010   # |κ_meas| defining a deep curve (1/m)

  # (FRICTION = 0.05 retired 2026-08-13. It claimed to be the rack's breakaway
  # torque; the measured breakaway is 2.0-2.75 Nm — 4x higher — and is not a
  # constant at all (it spans wider than the usable torque range across load,
  # speed and surface; route 3f2 study). Its two surviving epsilon-gates —
  # cancel_tol's |target_frac| > FRICTION and deep_relax's |torque| > FRICTION
  # — were user-ruled redundant: the first was HOLD_BAND expressed in torque
  # coordinates through the P gain, the second was vacuous under the
  # deep-curve gate (SAT-scale torque ~0.19 frac in any |κ_meas| > 0.010
  # curve). Breakaway is OBSERVED, never predicted. Do not reintroduce a
  # friction constant; nothing in this controller should pretend to model the
  # rack.)

  # Dead band of front-wheel error. It shall be a constant; 0.001 is a good
  # choice. (User's definition, 2026-08-13 — this one line IS the design.)
  # Why 0.001 is good, as measured footnotes, not derivation:
  #   - It sits at ~1σ of the error signal's band-passed noise floor
  #     (σ = 0.00081 rad on route 3f4's clean straights, 1.23σ; the 12-route
  #     2026-07-19 study gave σ·L = 0.00099, 1.0σ). Resting tighter would be
  #     resting on noise. The noise is speed-independent, which is why the
  #     band is correctly FIXED rather than 1/v².
  #   - At 25 m/s it tolerates 0.23 m/s² of lateral-accel error — about the
  #     driver perception threshold.
  #   - It is small enough not to attenuate the lane_keeping position
  #     correction — the Phase 1 failure mode, where a 0.0012–0.0021 rad
  #     tolerance ate 44% of the anchor's command (route 3bf).
  # k_sigma is live telemetry: if a future model upgrade halves the wander,
  # the telemetry itself will show this band has headroom to tighten.
  HOLD_BAND = 0.001        # rad of front-wheel-angle error treated as on-target

  # Entry/settle hysteresis (2026-08-13, route 3f4 data). One shared threshold
  # made the controller flicker across the band boundary ~89x/min on straights
  # (error noise sigma = 0.00081 rad, so HOLD_BAND is only 1.23 sigma),
  # starting ~22.6 sub-breakaway ramp episodes/min. Leaving rest now requires
  # clearing the noise core (> HOLD_BAND_ENTER); settling back keeps the
  # original tight point (<= HOLD_BAND), so a growing lane_keeping correction
  # still lands at 0.001 — plain widening was measured on route 3f4 and
  # rejected (-18% activations only, and it re-enters the Phase-1 regime
  # where a 0.0012-0.0021 tolerance ate 44% of the anchor's commands).
  # Replay: -63% activation episodes. Kill-switch: HoldHysteresis param
  # ('0' disables -> both thresholds = HOLD_BAND -> exact legacy behaviour).
  HOLD_BAND_ENTER = 0.0015

  # Persistent-lean escape (2026-08-14, route 3f8 on-car verdict). The entry
  # gap (0.001, 0.0015) rejects symmetric noise but also hid a constant
  # road-crown pull: slow |err| > 0.002 grew to 8.2% of straight time
  # (0.3% baseline speed-matched) and the user felt a left-hug with slow
  # correction. A second leave-rest condition on the SLOW error bounds the
  # lean latency: escape when |EMA(2 s) of delta_err| > HOLD_EMA_ESCAPE.
  # Flicker cannot move a 2-s average (replayed cost: +1.4 entries/min,
  # still ~30% under legacy); a sustained bias of 0.002 escapes in ~1.8 s,
  # 0.0014 in ~4 s, <= 0.0012 never (that zone IS the noise floor).
  # The EMA is deliberately NOT re-primed on settle (route 3f9, 2026-08-15):
  # re-priming starved the escape to zero fires, and a still-high EMA that
  # re-exits right after a settle is the intended behaviour on a crowned road,
  # where the correction itself drags the EMA back down. Gated on the same
  # HoldHysteresis kill-switch ('0' disables escape AND hysteresis -> exact
  # legacy behaviour).
  HOLD_EMA_TAU = 2.0        # s
  HOLD_EMA_ESCAPE = 0.0012  # rad; strict > so a bias exactly at threshold never escapes

  # SAFETY ARCHITECTURE (2026-07-28): NO ISO accel/jerk cancel guard. It used
  # to live here (a κ-indexed BMW_LATERAL_JERK table + a commanded-a_y
  # accel_guard_threshold, both gated on an `overshooting` predicate, draining
  # torque to 0 when they fired). All of it was removed: bounding lateral
  # acceleration is speedlimitd's job at the system level (it caps vEgo for
  # curves — a_y = v²·κ). This controller's contract is to track the commanded
  # curvature and never abandon a turn. See the module-level SAFETY
  # ARCHITECTURE note (top of file) and the docstring for the incident record
  # (326, 385 seg27, 2ba 1.29 m off-lane, 3ce hunting, 3cf seg15) showing the
  # drains were net-harmful. a_y_meas / jerk_pred are still computed in update()
  # for telemetry / log analysis, but nothing reads them for control.

  # Rear-axle bicycle-model wheelbase (m). Used for κ ↔ δ conversion.
  L = float(CP.wheelbase)

  _sm = messaging.SubMaster(['livePose'])

  state = {
    'torque': 0.0,             # current commanded torque fraction (advances by ramp_step each CAN tick)
    'target_frac': 0.0,        # plant-inversion target set each decision (cadence ≈ model_action_t/2)
    'ramp_step': 0.0,          # per-frame torque increment = (target − torque) / spread_frames
    'ramp_frames': 0,          # CAN frames left in current ramp
    'tick_count': ACTION_CADENCE_TICKS_FALLBACK,  # primed so first livePose tick fires cadence immediately (no engagement gap)
    'action': 'init',          # debug: hold_zero / hold_curve / ramp / relax_dwell / cancel_tol / idle
    #                            (cancel_accel / cancel_jerk removed 2026-07-28 — no ISO cancel)
    'delta_err': 0.0,          # debug: front-wheel-angle error (rad), what controller acts on
    'lat_pub': None,
    'desired': 0.0, 'measured': 0.0,
    'a_y_meas': 0.0,              # debug: v²·κ_meas (m/s²)
    'jerk_pred': 0.0,             # debug: v²·κ_err/τ (m/s³)
    'hold_f': 0.0,                # debug: lateral-accel hold factor [0,1] (gated on v²·|kappa_des|)
    'hold_cap': 0.0,              # debug: cap on held torque (P value at the HOLD_CAP_DRIFT_M reference drift, frac)
    'relax_ticks': 0,             # consecutive livePose ticks of overshoot-side error while deep in a curve
    'at_rest': True,   # hysteresis state: True = holding, False = correcting
    'derr_ema': 0.0,   # slow (HOLD_EMA_TAU) EMA of delta_err, livePose rate
    # Stall/breakaway v2 (see the SB_* constants at module scope). Entirely
    # separate from the EMA escape / hysteresis above: different state,
    # different trigger, different action — the two never interact.
    'sb_ring': deque(maxlen=SB_FROZEN_TICKS + 1),  # 100 Hz steerAngleDeg, active only
    'sb_state': 0,          # 0=idle, 1=armed(stall), 2=breakaway episode
    'sb_brk_angle': 0.0,    # steerAngleDeg at breakaway (displacement reference)
    'sb_dir': 0.0,          # +1/-1: breakaway MOTION direction (self-referenced)
    'sb_arm_kappa': 0.0,    # |kappa_des| LATCHED at ARM (internal, not telemetry)
    'sb_episode_ticks': 0,  # countdown in state 2
    'sb_block': 0,          # same-side push suppression countdown after a trip
    'sb_trips': 0,          # cumulative (telemetry)
  }

  def update(active, CS, VM, params, steer_limited_by_safety, desired_curvature, curvature_limited, lat_delay):
    pid_log = log.ControlsState.LateralTorqueState.new_message()
    pid_log.version = 11

    # getattr guard: CS is a stub in some test paths. steeringAngleDeg carries
    # a constant ~-1.58 deg alignment offset — DELTAS ONLY, never the
    # absolute. The stall/breakaway v2 state machine below is its consumer and
    # reads nothing but deltas (the angle stream is the only usable rack-motion
    # sensor here; angRate reads 0 through the whole route 3f2 creep phase),
    # which is why that offset never has to be known or estimated.
    _angle = float(getattr(CS, 'steeringAngleDeg', 0.0))

    # ---- Stall/breakaway v2 state machine (100 Hz) -----------------------
    # See the SB_* constants at module scope for the principle and the replay
    # provenance. ARM on a stalled rack (frozen angle while ramping) ->
    # BREAKAWAY when it finally moves, latching where it was and which way it
    # went -> TRIP when the freed wheel has travelled SB_TRIP_DISP_DEG past
    # that point, in that direction, at >= SB_TRIP_RATE_DPS, outside a deep
    # curve. Every comparison is relative to the wheel's own breakaway state,
    # so this machine has NO absolute angle or curvature convention to get
    # wrong (the alignment offset in steeringAngleDeg cancels out of every
    # difference, and sb_dir is observed rather than assumed).
    # Runs before the livePose branch: a trip's shed can still be overridden
    # by a same-tick cadence decision, which is what the shed-rate
    # enforcement below exists to undo — except when that decision is a
    # counter-steer, which always wins (see both sign gates below).
    if active:
      state['sb_ring'].append(_angle)
    else:
      state['sb_ring'].clear(); state['sb_state'] = 0; state['sb_block'] = 0
    if state['sb_block'] > 0:
      state['sb_block'] -= 1
      # Shed-rate enforcement (review finding, 2026-08-15). The trip writes a
      # SB_SHED_FRAMES drain, but a cadence decision landing inside the shed
      # window recomputes ramp_step over spread_frames (~40 frames), stretching
      # the 100 ms drain to ~400 ms — a 6x slowdown of the one action the trip
      # exists to perform, and the route 3f2 event peaked 0.30 s AFTER the trip
      # point, so drain RATE is the whole point. While the block is up and the
      # standing torque is still a same-side push, re-assert the fast drain
      # whenever the in-flight ramp is not draining toward zero at least that
      # fast.
      # Sign pairing: torque is OPPOSITE in sign to angle (torque negative =
      # left), so the push that drove the sb_dir motion has
      # torque * (-sb_dir) > 0.
      #
      # STAND DOWN FOR COUNTER-STEER (review finding (c), 2026-08-15). An
      # earlier version of this comment claimed an opposite-side ramp "already
      # satisfies the drain test and is never touched". That was FALSE: such a
      # ramp passes the DIRECTION conjunct but a counter-steer spread over the
      # normal ~40-frame cadence ramp has a per-frame step far below
      # |torque|/SB_SHED_FRAMES, so it fails the MAGNITUDE conjunct, is
      # misclassified as "not draining", and gets overwritten — and the
      # overwrite sets target_frac = 0, destroying the correction's
      # DESTINATION, not merely its rate. Measured: standing torque -0.27 with
      # a fresh +0.20 counter-steer decision gives ramp_step +0.01175 vs a
      # 0.027 drain bar. So the enforcement now explicitly yields whenever the
      # in-flight ramp is headed to the opposite side: the controller may stop
      # pushing, it must never give up correcting.
      _to_counter = state['target_frac'] * (-state['sb_dir']) < 0.0
      if state['torque'] * (-state['sb_dir']) > 0.0 and not _to_counter:
        _drain = abs(state['torque']) / SB_SHED_FRAMES
        _draining = (state['ramp_frames'] > 0
                     and state['ramp_step'] * state['torque'] < 0.0
                     and abs(state['ramp_step']) >= _drain)
        if not _draining:
          state['target_frac'] = 0.0
          state['ramp_step'] = -state['torque'] / SB_SHED_FRAMES
          state['ramp_frames'] = SB_SHED_FRAMES
    if _stall_v2_on and active:
      ring = state['sb_ring']
      if state['sb_state'] == 0:
        # ARM: a full 0.4 s ring inside 2 quanta while the controller is
        # pushing = the rack is not following the command. Common and
        # harmless on its own (926 arms/85 min on 3f4); arming does nothing.
        if state['action'] == 'ramp' and len(ring) == ring.maxlen \
           and (max(ring) - min(ring)) < 2 * ANGLE_QUANTUM_DEG:
          state['sb_state'] = 1
          # LATCH the arming curvature (2026-08-16). Never updated in state 1
          # or 2 — that is the point: arming mid-corner latches the IN-corner
          # kappa, so a corner-exit decay cannot sneak the episode under the
          # deep-curve gate later. Re-latched on every fresh arm.
          state['sb_arm_kappa'] = abs(state['desired'])
      elif state['sb_state'] == 1:
        if state['action'] != 'ramp':
          state['sb_state'] = 0
        elif len(ring) > SB_MOVE_TICKS \
             and abs(ring[-1] - ring[-1 - SB_MOVE_TICKS]) >= SB_MOVE_QUANTA * ANGLE_QUANTUM_DEG:
          # BREAKAWAY: the stalled rack moved. Latch the reference the trip
          # measures displacement from, and the direction it broke free in.
          # sb_dir is the OBSERVED motion direction over the breakaway
          # window — not derived from any angle-sign convention — so a
          # displacement "past the breakaway point" means the same thing in
          # both directions with nothing to invert.
          state['sb_state'] = 2
          state['sb_episode_ticks'] = SB_EPISODE_TICKS
          state['sb_brk_angle'] = ring[-1]
          state['sb_dir'] = 1.0 if ring[-1] >= ring[-1 - SB_MOVE_TICKS] else -1.0
      elif state['sb_state'] == 2:
        state['sb_episode_ticks'] -= 1
        # SIGNED by the breakaway direction (review finding (b), 2026-08-15).
        # An unsigned rate made the trip a REBOUND detector: a slow drift out
        # (below the gate, so no trip) followed by a fast snap BACK toward the
        # breakaway point reads as a fast crossing while displacement is still
        # decaying through the 2 deg mark. Reviewer repro: 39 ticks outbound at
        # 21.975 deg/s, then 20 return ticks at 30.765 deg/s -> tripped at
        # displacement 2.197 deg while the wheel was travelling the WRONG way.
        # Positive _rate now means "advancing in the direction it broke free",
        # which is the only motion a release can produce. No effect on 3f2 —
        # its sweep is in-direction throughout.
        _rate = (ring[-1] - ring[-1 - SB_MOVE_TICKS]) * state['sb_dir'] / (SB_MOVE_TICKS * DT_CAN_TICK) \
                if len(ring) > SB_MOVE_TICKS else 0.0
        if state['sb_episode_ticks'] <= 0 or state['action'] in ('hold_zero', 'hold_curve'):
          state['sb_state'] = 0
        # The kappa gate is applied BOTH instantaneous and latched-at-arm
        # (2026-08-16, drives 3fa/3fb). The instantaneous gate alone is
        # defeated by the hairpin-exit case: on 3fb t=1569 kappa_des decayed
        # 0.019 -> 0.0096 DURING the episode, slipping under
        # SB_TRIP_KAPPA_MAX while the wheel was still at -43 deg, so a plain
        # corner-exit unwind read as a release. The arm-time latch pins the
        # episode to the curvature it actually started in.
        # SB_TRIP_MIN_TORQUE: no standing windup, nothing to shed — 8 of the
        # 9 benign on-car trips were corner-exit unwinds at 0.007-0.088 frac.
        elif state['action'] != 'relax_dwell' \
             and abs(state['torque']) >= SB_TRIP_MIN_TORQUE \
             and abs(state['desired']) <= SB_TRIP_KAPPA_MAX \
             and state['sb_arm_kappa'] <= SB_TRIP_KAPPA_MAX \
             and (ring[-1] - state['sb_brk_angle']) * state['sb_dir'] >= SB_TRIP_DISP_DEG \
             and _rate >= SB_TRIP_RATE_DPS:
          # TRIP: the released rack has swept SB_TRIP_DISP_DEG past where it
          # broke free, in the direction it broke free, fast, in a curve mild
          # enough that SAT cannot arrest it, with the push still applied.
          # The deep-curve conjunct is not a comfort gate: above
          # SB_TRIP_KAPPA_MAX the self-aligning torque self-arrests a release
          # (hairpins tracked fine and dominated the false trips before it),
          # and shedding there would be the give-up-mid-turn failure mode.
          # Shed: drain to zero over SB_SHED_FRAMES (jerk-safe, same-side
          # decrease with the wheel already moving that way) and suppress
          # same-side pushes for SB_BLOCK_TICKS so the stale-error P law
          # cannot immediately rebuild the surplus.
          #
          # THE SHED IS SIGN-GATED (review finding (c), 2026-08-15). It used
          # to run unconditionally, so if the standing torque at the trip tick
          # was already OPPOSITE-side — the controller counter-steering the
          # release, exactly what it should be doing — the shed drained that
          # correction to zero, contradicting the invariant asserted three
          # lines down at the block. There is also nothing to shed in that
          # case: the surplus this mechanism exists to dump is same-side
          # torque. So with counter-steer (or zero) standing torque the event
          # is still RECORDED (sb_trips increments, the episode closes — it
          # happened, and the telemetry must show it) but nothing is drained
          # and no block is armed.
          if state['torque'] * (-state['sb_dir']) > 0.0:
            state['target_frac'] = 0.0
            state['ramp_step'] = -state['torque'] / SB_SHED_FRAMES
            state['ramp_frames'] = SB_SHED_FRAMES
            state['sb_block'] = SB_BLOCK_TICKS
          state['sb_trips'] += 1
          state['sb_state'] = 0
    else:
      state['sb_state'] = 0

    _sm.update(0)
    lp = _sm['livePose']
    livepose_updated = _sm.updated['livePose']

    # livePose tick (20 Hz): update measured + desired every tick; plant-
    # inversion decision only every action_cadence_ticks (= model_action_t/2,
    # currently ~325 ms at SAD=0.4) — gives plant time to respond to the
    # previous correction (≥ 2.5τ → ~92% response for BMW rack τ ≈ 130 ms).
    # CAN tick (100 Hz): apply ramp_step toward target_frac.
    if livepose_updated:
      # Plant prediction horizon — match modeld's lat_action_t (where the
      # model targets κ). lat_delay is controlsd's liveDelay.lateralDelay +
      # LAT_SMOOTH_SECONDS; modeld adds DT_MDL on top (modeld.py:391) for the
      # actual sample point, so we do the same here. For BMW this is pinned at
      # CP.steerActuatorDelay + 0.2 + DT_MDL since lagd never converges — see
      # MODEL_ACTION_T_FALLBACK block above.
      model_action_t = (lat_delay + DT_MDL) if lat_delay and lat_delay > 0 else MODEL_ACTION_T_FALLBACK

      # Decision cadence and CAN ramp length — both equal half of the model
      # action horizon (see constants block). 2 decision-and-ramp cycles fit
      # per horizon; the controller's correction bandwidth tracks the model's.
      # spread_frames derived from action_cadence_ticks (× 5 CAN ticks per
      # livePose tick) so ramp duration matches cadence period exactly — no
      # rounding mismatch between the two units (e.g. at model_action_t=0.65,
      # half=0.325s → cadence=6 ticks=300ms → spread=30 frames=300ms, equal).
      half_horizon = 0.5 * model_action_t
      action_cadence_ticks = max(1, round(half_horizon / DT_LIVEPOSE))
      spread_frames = action_cadence_ticks * int(DT_LIVEPOSE / DT_CAN_TICK)

      # This controller applies no filter of its own to κ_des — but as of
      # Phase 2 (2026-07-22) `desired_curvature` is no longer the raw model
      # output at the system level: the `lane_keeping` plugin low-passes
      # modelV2 κ_des upstream (`KAPPA_FILTER_TAU`, currently 0.15 s) before
      # this controller ever sees it. So `state['desired']` — and everything
      # downstream that reads it (`kappa_scale`, `hold_f`, `t_cap`, and the
      # `jerk_pred` telemetry) — sees that smoothed reference, not raw
      # modelV2 output. Keeping this controller filter-free means no held bias
      # here (the drift failure mode of the prior κ_des-hysteresis attempt);
      # noise-induced lag is now `lane_keeping`'s to own, per its position loop.
      state['desired'] = float(desired_curvature)

      # LKA mode (2026-08-14) runs lateral below DCC's 30 km/h floor down to
      # standstill, so low speed is now LIVE territory — the two speed roles
      # must split (user ruling 2026-08-16):
      #   v_true — the MEASUREMENT divisor. κ_meas = yawRate / v_true must use
      #     actual speed or the loop settles at κ_des·(8.5/v_true): ~2× inside
      #     the corner at 15 km/h. Floored at 1.0 m/s only against
      #     div-by-near-zero (controlsd drops latActive below 0.3 m/s anyway).
      #   v — the TORQUE-PARAMETER reference. All gains/caps (P target, t_cap,
      #     hold_cap, hold_factor gate, lookahead, step_max interp) keep
      #     8.5 m/s ≈ 30 kph as their floor: below 30 the torque calibration
      #     references 30 km/h rather than extrapolating v² toward zero.
      v_true = max(float(lp.velocityDevice.x) if _sm.seen['livePose'] else CS.vEgo, 1.0)
      v = max(v_true, 8.5)
      state['measured'] = float(lp.angularVelocityDevice.z) / v_true

      # Lateral-accel-dependent hold factor (2026-07-27; see HOLD_AY_BP /
      # hold_factor() at module scope). In a curve, "on-target" is achieved
      # WITH torque applied — that standing torque ≈ the self-aligning
      # torque (SAT), which scales as v²·κ and above HOLD_AY_BP[1] exceeds
      # rack stiction (route 380/384: hold-at-zero let the wheel unwind →
      # 0.6 Hz limit cycle). hold_f scales the on-target / cancel_tol
      # target: 0 below HOLD_AY_BP[0] (pure stiction-hold — straights and
      # gentle-slow curves, where SAT can't beat stiction anyway), 1 at/above
      # HOLD_AY_BP[1] (keep what got us on-target). Gated on commanded a_y
      # (v²·|κ_des|), not |κ_des| alone — a κ-only gate misses mild-but-fast
      # curves where SAT is already large (route 3ca seg 23: 19.4 m/s,
      # κ 0.0033, a_y 1.25 — plenty of SAT, but below the old κ-only
      # threshold). The held value is bounded by what the P-term was already
      # commanding and re-derives from state['torque'] every decision — no
      # learning state, no ratchet (the hold-bias integral failure mode).
      hold_f = hold_factor(v, abs(state['desired']))
      state['hold_f'] = hold_f

      # Front-wheel-angle error (rear-axle bicycle model).
      delta_des = math.atan(state['desired'] * L)
      delta_meas = math.atan(state['measured'] * L)
      delta_err_raw = delta_des - delta_meas

      # Phase 2: no filtering here. modelV2 noise is handled upstream by
      # lane_keeping (it low-passes kappa_des and closes the position loop),
      # so this controller tracks whatever reference it is given, faithfully.
      delta_err = delta_err_raw
      state['delta_err'] = delta_err

      # Slow-error EMA for the persistent-lean escape (see HOLD_EMA_TAU).
      state['derr_ema'] += (DT_LIVEPOSE / HOLD_EMA_TAU) * (delta_err - state['derr_ema'])

      # Lookahead is still needed for the hold cap below.
      lookahead_m = v * model_action_t

      # Curvature scale for the P-term and caps (computed per tick since the
      # hold cap below needs it too; keyed on raw |κ_des|, see T_CAP_SCALE
      # constants block). 2026-05-22 (route 326): near-straight scale 1.5 → 1.0.
      kappa_scale = float(np.interp(abs(state['desired']),
                                    T_CAP_SCALE_KAPPA, T_CAP_SCALE_BP))

      # Held torque for the on-target hold / cancel_tol (route 385 seg 27
      # review, 2026-07-03). "Keep what you have" needs two qualifiers:
      #   - sign-guard: never hold torque opposing the commanded curve — on a
      #     rack with no self-centering that actively drives the wrong way
      #     (seg 27: 400 ms counter-curve holds during overshoot recovery,
      #     63 vs 38 deg/s subsequent rate bursts).
      #   - magnitude cap: anything above what P would command while "almost
      #     on-target" is arrival momentum, not holding torque (seg 27
      #     latched 0.588 where steady SAT ≈ 0.15 → then-present ISO
      #     cancel_accel dumped it → deep unwind cycle; the cap keeps the
      #     latch from building in the first place, and outlives that cancel).
      # 2026-07-04: the cap was DECOUPLED from the kinematic DRIFT_M deadzone
      # that existed at the time. It was originally slope·kappa_scale·v²·
      # tolerance, but when the κ-widened DRIFT_M was reverted (deadzone back
      # to 0.02 m), deriving the cap from the now-tight tolerance would have
      # shrunk it ~5× (0.31 → 0.06 at the route-384 operating point) — BELOW
      # the measured steady SAT (0.08–0.15 frac), breaking hold_curve. The
      # cap keeps its own fixed reference drift (HOLD_CAP_DRIFT_M = 0.10 m —
      # the field-verified scale from the seg-27 fix), so it tunes
      # independently. That DRIFT_M deadzone was deleted entirely in Phase 2
      # (2026-07-22); this fixed reference is unaffected and unchanged.
      hold_cap = (T_CAP_SLOPE_BASE * kappa_scale * v * v
                  * (2.0 * HOLD_CAP_DRIFT_M * L / (lookahead_m ** 2))) / CCP.STEER_MAX
      state['hold_cap'] = hold_cap
      held = hold_f * state['torque']
      if held * delta_des < 0.0:
        held = 0.0
      held_target = float(np.clip(held, -hold_cap, hold_cap))

      # Relax-dwell (route 3a0 seg 8, 2026-07-12). modelV2's κ_des dips
      # 40–50% for ~1 s MID-hairpin and recovers (three times in one turn),
      # while κ_meas stays steady — the reference lies, the plant doesn't.
      # Following the dip down surrenders held torque; SAT flings the freed
      # wheel ~20° out of the turn ("gives up mid-way"), and the step-capped
      # rebuild takes 1.5–2 s against SAT. Unwinding is instant and free,
      # rebuilding is slow and fought — so bridge transient dips: while
      # DEEPLY in a measured curve, an overshoot-side error must persist
      # for RELAX_DWELL_TICKS before the relax path may command below the
      # current (capped) torque. Gates on MEASURED κ (steady through the
      # dips), requires κ_des still same-side (S-curve sign flips abort
      # instantly), and torque above breakaway. (Historically an ISO overshoot
      # cancel could interrupt the dwell; that machinery was removed 2026-07-28
      # — the dwell now runs uninterrupted, and genuine overshoot is corrected
      # by the P-law reversing as it tracks back to κ_des.)
      # True exits (κ_des drops and stays) proceed after the dwell: cost is
      # ≤1 s of extra curvature (~0.2 m lateral at 9 m/s).
      # (|torque| > FRICTION conjunct deleted 2026-08-13, user ruling: vacuous
      # — a tracked |κ_meas| > 0.010 curve implies SAT-scale held torque. If
      # torque somehow IS ~0 here, the dwell bridges at ~0, indistinguishable
      # from not arming.)
      deep_relax = (abs(state['measured']) > RELAX_DWELL_KAPPA
                    and state['desired'] * state['measured'] > 0.0
                    and delta_err * delta_des < 0.0)
      state['relax_ticks'] = state['relax_ticks'] + 1 if deep_relax else 0
      dwelling = 0 < state['relax_ticks'] <= RELAX_DWELL_TICKS

      # a_y_meas / jerk_pred: TELEMETRY ONLY since 2026-07-28. The ISO accel/
      # jerk cancel guard that used to read these (the `overshooting` predicate
      # + accel_guard_threshold + the drain-to-0) was removed — a_y is bounded
      # at the system level by speedlimitd's curve-speed capping, and this
      # controller never abandons a turn (see the SAFETY ARCHITECTURE notes at
      # module scope and in the docstring). We still compute them for log
      # analysis / field-health telemetry; nothing here reads them for control.
      a_y_meas = v * v * state['measured']
      jerk_pred = v * v * (state['desired'] - state['measured']) / model_action_t
      state['a_y_meas'] = a_y_meas
      state['jerk_pred'] = jerk_pred

      # (|target_frac| > FRICTION conjunct deleted 2026-08-13, user ruling:
      # redundant — it was HOLD_BAND in torque coordinates through the P gain.
      # Consequence: terminal push ramps with small stale targets now drain to
      # the held target on arrival instead of completing; less residual torque
      # at rest entry.)
      if (state['action'] == 'ramp' and abs(delta_err) <= 1.2*HOLD_BAND
            and state['ramp_frames'] > 0):
        # cancel_tol — HOLD_BAND boundary hygiene, NOT an ISO cancel (this is
        # the only "cancel_"-named path left; it tracks the command tighter, it
        # does not abandon the turn). Error fell into the on-target band (1.2x
        # HOLD_BAND) while a PUSH ramp is still in flight. Without this, the
        # ramp keeps driving torque toward a stale target until the next 250 ms
        # cadence.
        # Gated on action=='ramp' (route 385 review, 2026-07-03): hold
        # ramps also set ramp_frames, and un-gated this branch fired on
        # them every in-band tick — pinning tick_count (cadence stretched
        # 300→550 ms) and flooding telemetry with phantom cancel_tol
        # (~10 of 11 in-curve ticks), corrupting the action field-health
        # metric. Gating also removes the float-equality re-arm fragility
        # in the blend region. Drain to the sign-guarded, capped hold
        # (0 on straights; ≈steady SAT in curves — "stop the ramp, keep
        # what you have" without keeping arrival momentum).
        unwind_target = held_target
        if state['target_frac'] != unwind_target:
          state['target_frac'] = unwind_target
          state['ramp_step'] = (unwind_target - state['torque']) / spread_frames
          state['ramp_frames'] = spread_frames
        state['action'] = 'cancel_tol'
        state['tick_count'] = 0
      else:
        state['tick_count'] += 1
        # Expire transient action labels once their ramp completes (parked
        # fix, folded in 2026-07-19): between cadence decisions the label
        # used to persist after the ramp finished, so naive telemetry
        # occupancy counts over-attributed ramp/relax states (transition-
        # level counting was the workaround). Holds keep their label — they
        # re-fire every cadence and their occupancy IS meaningful. The
        # cancel_tol gate is unaffected: it requires ramp_frames > 0, and
        # in-flight ramps (ramp_frames > 0) never expire here.
        if state['ramp_frames'] == 0 and state['action'] in (
            'ramp', 'relax_dwell', 'cancel_tol'):
          state['action'] = 'idle'

      if state['tick_count'] >= action_cadence_ticks:
        state['tick_count'] = 0

        # Plant-inversion target torque in angle domain — the steady-state
        # aligning torque required to hold δ_err. Phase 2 (2026-07-22): P
        # acts on the FULL δ_err, no soft-deadband subtraction — see the
        # target_nm formula below and the 2026-07-22 header note in
        # LATERAL_CONTROLLER.md for why (it was attenuating the upstream
        # lane_keeping position correction).
        #   τ_Nm = T_CAP_SLOPE · v² · δ_err
        # On-target (|δ_err| ≤ HOLD_BAND) → stiction holds; no chatter at
        # the boundary (see the hold_curve / hold_zero branch below).
        # Entry/settle hysteresis: leave rest only past HOLD_BAND_ENTER,
        # return only at/below HOLD_BAND. With the kill-switch off both
        # thresholds are HOLD_BAND and this reduces exactly to the legacy
        # per-decision comparison (leave on >, return on <=).
        _enter = HOLD_BAND_ENTER if _hold_hyst_on else HOLD_BAND
        if state['at_rest']:
          if abs(delta_err) > _enter or \
             (_hold_hyst_on and abs(state['derr_ema']) > HOLD_EMA_ESCAPE):
            state['at_rest'] = False
        else:
          if abs(delta_err) <= HOLD_BAND:
            state['at_rest'] = True
            # NO re-prime on settle (route 3f9, 2026-08-15): re-priming to the
            # settle-point error starved the escape — instantaneous noise wins
            # the race to HOLD_BAND_ENTER within ~1-2 decisions of any lean,
            # and a freshly re-primed 2-s EMA can never reach HOLD_EMA_ESCAPE
            # during rest (measured: 0 escape fires in 15 straight-min, all
            # 190 rest-exits threshold-driven). A still-high EMA re-exiting
            # right after settle is the escape working: a crowned road needs
            # continuous correction, and the correction itself drags the EMA
            # down, so re-fires are self-limiting (replayed on 3f9:
            # 1.54 fires/min, +1.4 entries/min, no churn loop).
        if state['at_rest']:
          # On-target. Low commanded a_y (hold_f=0 — straights and gentle-
          # slow curves): drain to 0, stiction holds. Higher commanded a_y
          # (hold_f→1): keep the torque that achieved on-target — it
          # approximates the self-aligning torque, which beats stiction at
          # that loading; target-0 here was the 0.6 Hz limit cycle's driver
          # (route 380/384) and, before the a_y gate, also the route 3ca
          # seg 23 mild-fast-curve hunting. Re-derived from state['torque']
          # each decision: bounded by the P-term's own command, no ratchet.
          # (brake_zero one-shot reverse pulse removed 2026-07-03 —
          # residual on-target-entry momentum is left to rack friction.)
          # held_target is sign-guarded (never counter-curve) and capped at
          # the P value implied by HOLD_CAP_DRIFT_M — see the hold_cap
          # block above.
          target_frac = held_target
          state['action'] = 'hold_curve' if held_target != 0.0 else 'hold_zero'
        else:
          # kappa_scale computed per tick above (shared with hold_cap):
          # 1.0 on straights rising to 3.0 at |κ_des| ≥ 0.02.
          # Phase 2: P acts on the FULL error — the tolerance subtraction is
          # gone. It was the term that attenuated the lane_keeping position
          # correction (Phase 1, route 3bf: 44% of correcting ticks produced no
          # action at all). Nothing here shrinks the commanded curvature now.
          target_nm = T_CAP_SLOPE_BASE * kappa_scale * v * v * delta_err
          target_frac = target_nm / CCP.STEER_MAX
          # Sub-breakaway targets are commanded as-is (breakaway ±FRICTION
          # amplification removed 2026-07-03): commands below the rack's
          # breakaway torque don't move the wheel — an intentional soft
          # actuation deadband; the wheel moves once the P-term grows past
          # friction naturally.
          state['action'] = 'ramp'
          # v²·|δ|-scaled cap, clipped at STEER_MAX (panda hard limit).
          # Authority grows with commanded a_y_des — straights stay near BASE,
          # tight turns can reach STEER_MAX (transient over-envelope; speedlimitd
          # bleeds v).
          t_cap_nm = min(CCP.STEER_MAX,
                         T_CAP_BASE_NM + T_CAP_SLOPE_BASE * kappa_scale * v * v * abs(delta_des))
          t_cap_frac = t_cap_nm / CCP.STEER_MAX
          target_frac = float(np.clip(target_frac, -t_cap_frac, t_cap_frac))
          # Hold-floor (route 393 segs 7/8, 2026-07-06): a same-direction
          # push never commands LESS than the held torque — "keep holding
          # while adding trim; don't ease off while still understeering."
          # Route 393 showed 15–31% of in-curve ramp ticks commanded
          # sub-friction targets: with the wheel held at e.g. 0.08 and the
          # error drifting just outside the band, P computed ~0.04 and the
          # ramp DROPPED torque below the holding level — SAT unwound the
          # wheel and accelerated the very error being corrected. The floor
          # applies only when P and the (sign-guarded, capped) hold point
          # the same way; opposite-sign P (overshoot correction) is
          # untouched, so torque reduction still happens the moment the
          # error flips sides. Bounded by hold_cap, STEP_MAX, and t_cap.
          if target_frac * held_target > 0.0:
            target_frac = math.copysign(max(abs(target_frac), abs(held_target)), target_frac)
          # Relax-dwell bridge (see the deep_relax block above): during the
          # dwell window, an overshoot-side reference dip may NOT command
          # below the current (capped) torque — keep the curve and wait the
          # dip out. Persisting overshoot (> dwell) falls through to the
          # normal relax staircase (the ISO cancel that used to bypass this
          # is gone as of 2026-07-28).
          if dwelling:
            target_frac = math.copysign(min(abs(state['torque']), hold_cap), state['torque'])
            state['action'] = 'relax_dwell'
          # Per-decision step cap (route 385 seg 27 review, 2026-07-03):
          # human-style gradual steering — move at most STEP_MAX toward the
          # P target per decision, let the plant respond one cadence, then
          # re-measure and step again. Never apply excessive torque
          # abruptly: seg 27 showed single decisions swinging Δ0.69 frac
          # (8.3 Nm) which drove 150 deg/s wheel bursts and the
          # over-latch → (then-present ISO) cancel_accel dump → deep-unwind
          # cycle — the step cap prevents the over-latch at the source. Max
          # slew is now ~0.33 frac/s (~4 Nm/s); the P-law reverses on
          # overshoot, speedlimitd handles curve-entry speed (a_y bound).
          step_max = float(np.interp(v, STEP_MAX_V, STEP_MAX_BP))
          step = float(np.clip(target_frac - state['torque'], -step_max, step_max))
          target_frac = float(np.clip(state['torque'] + step, -t_cap_frac, t_cap_frac))
          # Post-trip block: no same-side re-push while the release is
          # still settling. Opposite-side correction passes untouched —
          # this must never block counter-steer (SAFETY ARCHITECTURE:
          # the controller may stop pushing, never give up correcting).
          # Sign pairing: sb_dir is the wheel's breakaway MOTION direction
          # in ANGLE space (+ = leftward) and torque is OPPOSITE in sign to
          # angle (torque negative = left), so the push driving that motion
          # is target_frac * (-sb_dir) > 0. Pinned in both directions by
          # TestStallBreakawayV2.test_block_suppresses_same_side_only.
          # Gated on action == 'ramp' (review Minor): by this point the
          # relax-dwell bridge may have replaced target_frac with a HOLD of
          # the current torque, which is not a push and must not be zeroed —
          # zeroing it is the give-up-mid-turn failure mode relax_dwell
          # exists to prevent.
          if state['action'] == 'ramp' and state['sb_block'] > 0 \
             and target_frac * (-state['sb_dir']) > 0.0:
            target_frac = 0.0

        state['target_frac'] = target_frac
        state['ramp_step'] = (target_frac - state['torque']) / spread_frames
        state['ramp_frames'] = spread_frames

    # Apply per-frame ramp step. Panda enforces wire-rate (STEER_DELTA_UP)
    # downstream; large ramp_step (Δ > 5 Nm spread over 50 frames) gets
    # clipped at the gateway.
    if state['ramp_frames'] > 0:
      state['torque'] = float(np.clip(state['torque'] + state['ramp_step'], -1.0, 1.0))
      state['ramp_frames'] -= 1

    err = state['desired'] - state['measured']  # for logging only
    output = 0.0 if not active else float(np.clip(state['torque'], -1.0, 1.0))

    pid_log.actualLateralAccel = float(state['measured'])
    pid_log.desiredLateralAccel = float(state['desired'])
    pid_log.error = float(err)
    pid_log.active = active
    pid_log.output = float(output)
    pid_log.saturated = bool(abs(output) > 0.99)

    # Telemetry: publish at livePose rate (20 Hz). Most fields only change on
    # livePose ticks; per-CAN-tick publish would burn ~1300 dict-key inserts/sec
    # for the same observable signal.
    if livepose_updated:
      try:
        if state['lat_pub'] is None:
          from openpilot.selfdrive.plugins.plugin_bus import PluginPub
          state['lat_pub'] = PluginPub('bmw_lat_control')
        payload = {
          'desired': float(state['desired']),
          'desired_raw': float(desired_curvature),     # bit-identical to 'desired' (Phase 2: kept for telemetry schema back-compat)
          'measured': float(state['measured']),
          'err': float(err),
          'delta_err': float(state['delta_err']),             # what controller acts on (Phase 2: raw, no filter)
          'target_frac': float(state['target_frac']),
          'ramp_step': float(state['ramp_step']),
          'ramp_frames': int(state['ramp_frames']),
          'action': state['action'],
          'torque': float(state['torque']),
          'output': float(output),
          'vEgo': float(CS.vEgo),
          'active': active,
          'a_y_meas': float(state['a_y_meas']),
          'jerk_pred': float(state['jerk_pred']),
          'hold_f': float(state['hold_f']),
          'hold_cap': float(state['hold_cap']),
          'hold_band': float(HOLD_BAND),                      # stiction hold trigger (rad)
          'hb_enter': float(HOLD_BAND_ENTER if _hold_hyst_on else HOLD_BAND),  # leave-rest threshold (rad); drive self-label: 0.0015=hysteresis, 0.001=kill-switched, absent=old build
          'derr_ema': float(state['derr_ema']),               # slow (2 s) error EMA driving the persistent-lean escape (rad)
          'at_rest': bool(state['at_rest']),                  # hysteresis decision state (observable even when action is cancel_tol/idle, where action is not a proxy for it)
          'relax_ticks': int(state['relax_ticks']),
          'sb_state': int(state['sb_state']),                 # stall/breakaway v2: 0=idle, 1=armed, 2=episode
          'sb_trips': int(state['sb_trips']),                 # cumulative trips this drive
          'sb_block': int(state['sb_block']),                 # same-side push suppression ticks left
          'sb_on': bool(_stall_v2_on),                        # StallBreakaway param, drive self-label
        }
        state['lat_pub'].send(payload)
      except Exception:
        pass

    return -output, 0.0, pid_log

  lac.update = update
  return result
