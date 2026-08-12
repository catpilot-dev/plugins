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
  over; it is sized by rack breakaway (below it P commands less than
  friction, so the wheel can't move anyway), not by allowed lateral drift.
  modelV2 noise handling and lateral-drift correction now live upstream in
  the lane_keeping plugin's position loop:
    on_target = |δ_err| ≤ HOLD_BAND

  Plant-inversion target torque, angle domain (linear tire regime). P acts
  on the FULL error — no tolerance subtraction (Phase 2: it was attenuating
  the lane_keeping position correction upstream):
    τ_Nm_target = T_CAP_SLOPE · v² · δ_err
    Clamp to ±T_CAP(v, δ):
      T_CAP_NM = min(STEER_MAX, T_CAP_BASE + T_CAP_SLOPE · v²·|δ_des|)
    Same slope drives both target and cap.
    Sub-friction targets are commanded as-is (they don't move the rack —
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
  T_CAP_BASE_NM, and FRICTION. Tune these offline from route data; there's
  no scale_by_bin or shadow estimator anymore.
  """
  import math
  from cereal import log
  from cereal import messaging
  from bmw.values import CarControllerParams as CCP

  # AngleBudget param — read ONCE here, not per-tick or on any periodic
  # cache-expiry (review fix, Important 4): this used to re-check a 5 s cache
  # every CAN tick, which still means a Path.read_text() on controlsd's 100 Hz
  # RT thread whenever the cache lapsed — under eMMC contention that read can
  # cost a control frame, and the toggle-off default path paid for the
  # monotonic-clock check on every tick for no benefit. Applying or rolling
  # back the toggle therefore requires restarting controlsd (an offroad
  # reboot, not a UI restart) — see LATERAL_CONTROLLER.md § 12, "To toggle
  # AngleBudget", for the exact procedure and how to verify from telemetry
  # that a restart actually took. See BUDGET_DEG in the constants block below.
  #
  # Import at function scope, not module scope (review fix, Important 2):
  # every other config.read_plugin_param consumer in this repo does this
  # (bmw/carstate.py's _load_steer_angle_offset, speedlimitd's _read_params)
  # so a missing config.py only defaults this one param off instead of
  # failing the whole hook module's import — install.sh copies config.py to
  # the plugins-runtime root as a separate, later step, so a partial deploy
  # can transiently be missing it. A module-scope import failing here would
  # silently fall the car back to stock LatControlTorque, which is worse
  # than just defaulting the toggle off.
  try:
    from config import read_plugin_param
    _angle_budget_on = read_plugin_param('bmw_e9x_e8x', 'AngleBudget', '') == '1'
  except Exception:
    _angle_budget_on = False

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

  # Steering-wheel movement one open-loop push may cause before feedback takes
  # over. Human-style: push harder until the wheel moves, then stop pushing and
  # ease off. 2 deg is 0.11 deg of front wheel (curvature 0.00070 /m, ~1440 m
  # radius) — a real steering input, and 45 quanta of the 0.04395 deg angle
  # signal, so no noise can spend it (0.04395 is the confirmed quantum; see
  # LATERAL_CONTROLLER.md § 11 for the reconciliation against the deleted v1
  # rack_motion.py's differently-inferred value). Route 3f2 seg 10: spent at
  # 2.8 Nm against the 3.75 Nm the ramp actually reached.
  BUDGET_DEG = 2.0

  # Relax-dwell (2026-07-12, route 3a0 seg 8): in a measured deep curve, an
  # overshoot-side error must persist this long before the relax path may
  # command below current torque — bridges modelV2's transient mid-turn
  # κ_des dips (40-50% for ~1 s, κ_meas steady). See deep_relax in update().
  RELAX_DWELL_TICKS = 20      # livePose ticks (1.0 s at 20 Hz)
  RELAX_DWELL_KAPPA = 0.010   # |κ_meas| defining a deep curve (1/m)

  # Rack breakaway torque fraction (stiction floor). 2026-07-03: no longer
  # used to amplify sub-friction commands or emit reverse pulses (stiction
  # special-casing removed — the pulses were churn, not correction; the
  # straight-line wobble they targeted is modelV2 vision noise, now handled
  # upstream by lane_keeping since Phase 2, 2026-07-22). Retained only as
  # the cancel_tol boundary-hygiene threshold: ramps below this can't move
  # the rack, so there is nothing to stop.
  FRICTION = 0.05

  # Stiction hold trigger (Phase 2, 2026-07-22). Replaces the DRIFT_M kinematic
  # deadzone, which existed only because there was no position feedback — that
  # job now belongs to the lane_keeping position loop, which also owns modelV2
  # noise. This band exists ONLY to decide when the rack is "on target" so the
  # curvature hold can take over; it is sized by STICTION, not by drift: below
  # the error where the P term commands less than rack breakaway
  # (FRICTION·STEER_MAX / (T_CAP_SLOPE·kappa_scale·v²) ≈ 0.001 rad at 25 m/s)
  # the wheel cannot move anyway. Small enough that it does not meaningfully
  # attenuate the lane_keeping position correction — the Phase 1 failure mode,
  # where the old 0.0012–0.0021 rad tolerance ate 44% of the anchor's command.
  HOLD_BAND = 0.001        # rad of front-wheel-angle error treated as on-target

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
    'push_ref': None,        # steering angle when this push began (deg)
    'push_moved': 0.0,       # debug: signed deg moved since then
    'budget_spent': False,   # debug: 2 deg moved in the commanded direction
  }

  def update(active, CS, VM, params, steer_limited_by_safety, desired_curvature, curvature_limited, lat_delay):
    pid_log = log.ControlsState.LateralTorqueState.new_message()
    pid_log.version = 11

    # Push budget. Deltas only — steeringAngleDeg carries a constant ~-1.58 deg
    # physical alignment offset which cancels against the captured reference.
    # getattr guard: CS is a stub in some test paths.
    # Gated on `active` (review fix): update() runs regardless of engagement
    # and the decision state machine (which sets state['action']) is not
    # itself gated on active, so without this, driver steering while
    # disengaged/hands-on would accrue into push_moved and a push could begin
    # already spent. CS.steeringPressed is NOT usable as a substitute gate on
    # this car — it is a voice-control button ORed with gasPressed.
    _angle = float(getattr(CS, 'steeringAngleDeg', 0.0))
    if active and state['action'] == 'ramp':
      if state['push_ref'] is None:
        state['push_ref'] = _angle
      state['push_moved'] = _angle - state['push_ref']
    else:
      state['push_ref'] = None
      state['push_moved'] = 0.0
    # Torque is NEGATIVE for left, angle POSITIVE for left, so the product of
    # push_moved and -torque is positive when the wheel moved the way we asked.
    state['budget_spent'] = (_angle_budget_on
                             and abs(state['push_moved']) >= BUDGET_DEG
                             and state['push_moved'] * -state['torque'] > 0.0)

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

      # 8.5 m/s = ~30 kph, BMW DCC minimum engagement speed. Below this the
      # controller is never active, so the floor only protects κ_meas from
      # div-by-near-zero during disengaged crawl.
      v = max(float(lp.velocityDevice.x) if _sm.seen['livePose'] else CS.vEgo, 8.5)
      state['measured'] = float(lp.angularVelocityDevice.z) / v

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
      deep_relax = (abs(state['measured']) > RELAX_DWELL_KAPPA
                    and state['desired'] * state['measured'] > 0.0
                    and abs(state['torque']) > FRICTION
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

      if (state['action'] == 'ramp' and abs(delta_err) <= 1.2*HOLD_BAND
            and state['ramp_frames'] > 0 and abs(state['target_frac']) > FRICTION):
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
        if abs(delta_err) <= HOLD_BAND:
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
          # Sub-friction targets are commanded as-is (breakaway ±FRICTION
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
          # Once the wheel has moved its 2 deg, stop pushing harder and shed
          # torque at whatever rate the P law asks. Winding up is fought by the
          # rack; unwinding is free (self-aligning torque does it). A symmetric
          # STEP_MAX is right while ramping blind, but afterwards it needed
          # 0.65 s to unwind route 3f2 seg 10 while the overshoot took 0.4 s.
          # The budget lets the controller stop pushing and let go — it must
          # NEVER let it push harder, in either direction (review fix, this
          # replaced a sign-blind |target|>|torque| comparison that either
          # froze on an overshoot reversal — the controller giving up mid-turn,
          # the invariant the module-level SAFETY ARCHITECTURE note forbids —
          # or, when the counter-target had smaller magnitude, applied no cap
          # at all, up to a single-decision Δ0.578 frac swing). Same-side
          # target: clamp toward torque, never past it (freeze, don't push
          # harder). Then shed toward zero unthrottled — that's free, SAT does
          # it for us — and allow at most one step_max past zero, since
          # anything past zero is a NEW push in the other direction and gets
          # rate-limited like any other.
          if state['budget_spent']:
            if target_frac * state['torque'] > 0.0:
              target_frac = math.copysign(min(abs(target_frac), abs(state['torque'])),
                                          state['torque'])
            lo = min(0.0, state['torque']) - step_max
            hi = max(0.0, state['torque']) + step_max
            target_frac = float(np.clip(target_frac, lo, hi))
            step = target_frac - state['torque']
          else:
            step = float(np.clip(target_frac - state['torque'], -step_max, step_max))
          target_frac = float(np.clip(state['torque'] + step, -t_cap_frac, t_cap_frac))

        state['target_frac'] = target_frac
        state['ramp_step'] = (target_frac - state['torque']) / spread_frames
        state['ramp_frames'] = spread_frames
        if active and state['action'] == 'ramp':
          # Re-arm the push budget every decision (review fix, Critical 1).
          # push_ref used to be captured only on the first tick of a push and
          # then held for the push's entire lifetime — so once budget_spent
          # latched true, it stayed true for as long as action == 'ramp'
          # (which is exactly as long as |delta_err| > HOLD_BAND), pinning
          # the same-side clamp and locking the push out of torque authority
          # for the rest of the turn: the "controller gives up mid-turn"
          # state this file's SAFETY ARCHITECTURE header forbids. A driver
          # does not stop pushing forever after one 2-degree movement — they
          # ease off, feel whether the wheel is still moving, and push again
          # if not. Re-measuring the reference at this same cadence (the one
          # that just set target_frac/ramp_step/ramp_frames above) gives
          # exactly that stepping behaviour: each ~model_action_t/2 decision
          # gets its own fresh BUDGET_DEG allowance, so a push that spends
          # the budget and then sticks (wheel static) recovers full torque
          # authority at the very next decision, while a push whose wheel
          # keeps moving fast keeps re-spending it decision after decision.
          # Gated on active/action=='ramp' exactly like the capture above,
          # for the same disengagement reason.
          state['push_ref'] = _angle

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
          'relax_ticks': int(state['relax_ticks']),
          'push_moved': float(state['push_moved']),
          'budget_spent': bool(state['budget_spent']),
        }
        state['lat_pub'].send(payload)
      except Exception:
        pass

    return -output, 0.0, pid_log

  lac.update = update
  return result
