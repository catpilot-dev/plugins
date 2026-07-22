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


def on_lat_controller_init(result, lac, CP):
  """Plant-inversion at 500 ms horizon in front-wheel-angle space.

  BMW E90 hydraulic rack has high breakaway friction and no alignment-torque
  self-centering — the wheel holds its angle at zero torque *near center*.
  In curves, self-aligning torque exceeds rack stiction above ~40° wheel
  (route 380/384), so on-target there requires standing torque. So:
    - Inside tolerance, straight (|κ_des| < HOLD_KAPPA_BP[0]): drive torque
      → 0 and let stiction hold. No chatter.
    - Inside tolerance, curve: hold hold_f·torque — the torque that achieved
      on-target ≈ the self-aligning torque. Re-derived each decision from
      state['torque']; bounded by the P-term's own command, no ratchet.
    - Outside tolerance: compute the torque that would move the front wheel by
      δ_err over 500 ms (plant-inversion accounting for first-order lag), ramp
      to it over one 250 ms decision.

  Error in rear-axle bicycle-model front-wheel-angle space:
    δ_des  = atan(κ_des  · L)        L = CP.wheelbase
    δ_meas = atan(κ_meas · L)        κ_meas = yawRate / v_ego
    δ_err  = δ_des − δ_meas

  Tolerance (physical DRIFT_M = 0.02 m over model_action_t, speed-adaptive
  1/v²; the 2026-07-03 κ-widening was reverted 2026-07-04 — allowed drift
  must not grow with curvature):
    tolerance = 2 · DRIFT_M · L / (v · model_action_t)²

  Plant-inversion target torque, angle domain (linear tire regime):
    τ_Nm_target = T_CAP_SLOPE · v² · (δ_err − tolerance·sign(δ_err))
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

  ISO 11270 half-comfort guard (every livePose tick): cancel ramping if
  |a_y_meas| > 1.5 m/s² OR predicted jerk |v²·(κ_des−κ_meas)/0.5| > 2.5 m/s³,
  AND only when plant has actually overshot ((κ_des−κ_meas)·κ_meas < 0).
  Under-tracking (plant lagging in a hard curve) is left to the controller
  to chase. When cancel fires, drain the ramp to 0 — with torque relaxed,
  tire aligning forces unwind the wheel (at guard-firing angles aligning
  torque exceeds rack stiction; route 380/384 evidence).

  Relax-dwell (2026-07-12): in a measured deep curve (|κ_meas| > 0.010),
  an overshoot-side error must persist 1.0 s before the relax path may
  command below current torque — bridges modelV2's transient mid-turn κ_des
  dips (the "gives up mid-way" mechanism, route 3a0 seg 8). ISO cancels
  bypass the dwell; κ_des sign flips (S-curves) abort it instantly.

  Tolerance-cancel (every livePose tick): if |δ_err| drops into the success
  band mid push-ramp, drain torque to the sign-guarded, capped hold (0 on
  straights). Without this, the in-flight ramp keeps pushing toward a stale
  target until the next 250 ms cadence notices.

  2026-07-03 simplification: all stiction special-casing removed (breakaway
  ±FRICTION amplification, brake_zero reverse pulse, cancel reverse-FRICTION
  pulses). Route 384 telemetry showed ~40% of in-turn decisions were friction
  pulses — torque reversals that churned rather than corrected. The straight-
  line wobble those mechanisms targeted is modelV2 vision noise, handled by
  the blended delta_err box filter instead (blend since 2026-07-04).

  No online adaptation: plant behavior is fully described by T_CAP_SLOPE,
  T_CAP_BASE_NM, and FRICTION. Tune these offline from route data; there's
  no scale_by_bin or shadow estimator anymore.
  """
  import math
  import numpy as np
  from cereal import log
  from cereal import messaging
  from bmw.values import CarControllerParams as CCP
  from opendbc.car.lateral import ISO_LATERAL_ACCEL, ISO_LATERAL_JERK

  # Decision cadence & CAN-rate spreading — both subscribe to model_action_t
  # per tick, sized to one half of the model's action horizon so exactly two
  # decision-and-ramp cycles fit within one horizon. This keeps the
  # controller's correction bandwidth matched to the model's planning
  # bandwidth: when CP.steerActuatorDelay changes, all three (cadence,
  # ramp, jerk_pred horizon) move together — no parallel-constant drift.
  #
  #   action_cadence_ticks = round( (model_action_t / 2) / DT_LIVEPOSE )
  #   spread_frames        = round( (model_action_t / 2) / DT_CAN_TICK  )
  #
  # Each ramp completes when the next decision lands; no overlapping ramps.
  # No internal rate cap — panda enforces wire-rate (STEER_DELTA_UP =
  # 0.1 Nm/frame). For typical deltas (≤ 2.5 Nm), ramp_step ≤ 0.1 Nm/frame
  # and demand tracks rack reality. For large transients, panda clips and
  # state['torque'] briefly leads the rack — accepted; cancel logic still
  # produces correct intent.
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
  #   target_Nm    = T_CAP_SLOPE · v² · effective_err
  # BASE covers the speed- and angle-independent stiction floor.
  # T_CAP_SLOPE_BASE = 1.0: gentle baseline gain on straights. A curvature-
  # dependent scale T_CAP_SCALE(|κ_des|) bumps it up to 3.0× on tight curves
  # (linear interp 0.001..0.01 1/m). Rationale: small κ_des needs gentle gain
  # to avoid ringing on near-straight sections (seg-14 evidence); tight κ_des
  # needs enough authority to chase the planner without lag (seg-6 evidence).
  # The soft-deadband and per-tick tolerance-cancel handle the boundary
  # smoothness.
  T_CAP_BASE_NM = 2.0
  T_CAP_SLOPE_BASE = 1.0
  T_CAP_SCALE_KAPPA = [0.001, 0.01, 0.02]        # |κ_des| breakpoints (1/m)
  T_CAP_SCALE_BP    = [1.0, 2.5, 3.0]           # scale factor on T_CAP_SLOPE_BASE
  # Plant prediction horizon — sourced per-tick from controlsd's lat_delay
  # (= liveDelay.lateralDelay + LAT_SMOOTH_SECONDS) and matched to where
  # modeld samples κ_des: lat_action_t = lat_delay + DT_MDL (modeld.py:391).
  # Used by jerk_pred (the ISO-jerk overshoot guard) and by the kinematic
  # deadzone formula (DRIFT_M_BP block below) — same horizon for both.
  #
  # For BMW: lagd never converges (it correlates on latcontrol_torque
  # telemetry that our front-wheel-angle plant-inversion controller doesn't
  # produce), so liveDelay.lateralDelay is permanently pinned at initial_lag
  # = CP.steerActuatorDelay + 0.2 (lagd.py:181). CP.steerActuatorDelay in
  # bmw/interface.py is therefore the single knob controlling both modeld's
  # lat_action_t AND our model_action_t — change it there to retune both.
  DT_MDL = 0.05                                  # openpilot model dt (common/realtime.py)
  MODEL_ACTION_T_FALLBACK = 0.55                 # used only if lat_delay arrives as 0/None

  # Curvature-dependent hold (2026-07-03): inside the tolerance band the
  # target is hold_f·torque instead of 0, hold_f = interp(|κ_des|, BP, [0,1]).
  # Below BP[0] (straights; above the ±0.003 modelV2 wobble amplitude) the
  # target is 0 as before. Between BP[0] and BP[1] the partial factor gives
  # a geometric relax (each decision multiplies held torque by hold_f);
  # at/above BP[1] the torque that achieved on-target is held outright.
  # Rationale: in a curve, on-target is reached WITH torque applied — that
  # torque ≈ SAT, and above ~40° wheel SAT beats rack stiction, so target-0
  # lets the wheel unwind (route 380/384 0.6 Hz limit cycle).
  HOLD_KAPPA_BP = [0.004, 0.010]
  # Reference drift scale for the hold-torque cap ONLY (decoupled from the
  # deadzone DRIFT_M on 2026-07-04 — see the hold_cap block in update()).
  HOLD_CAP_DRIFT_M = 0.10

  # Per-decision torque step cap (2026-07-03, route 385 seg 27 review).
  # Human-style gradual steering: each cadence decision moves the target at
  # most step_max from the current torque, the plant responds (~2.5τ per
  # cadence), the next decision re-measures and steps again. Design rule:
  # never apply excessive steering torque abruptly. 0.10 frac = 1.2 Nm per
  # 300 ms ≈ 4 Nm/s max slew (panda wire limit is 10 Nm/s). Full authority
  # builds in ~1.5 s instead of 0.3 s — accepted: speedlimitd slows for
  # curves and the ISO guards still cancel overshoot instantly. First knob
  # to revisit if curve entries ever feel late.
  #
  # 2026-07-09 (route 39b seg 18, user safety call): SPEED-SCALED. A slight
  # highway left showed sudden back-and-forth wheel motion; aggressive
  # per-decision steps are riskier at highway speed (lane margin consumed
  # faster, less time to react). Full 0.10 up to 15 m/s (curves keep their
  # entry authority; speedlimitd owns curve-entry speed), tapering to 0.05
  # at/above 28 m/s (100 km/h) — highway slew halves to ~2 Nm/s. Deadzone
  # (DRIFT_M) deliberately NOT tightened at speed: it is already 1/v²-tight
  # (~0.02° at 28 m/s, ~10× below the vision-noise floor) and tightening
  # would add chase pressure, not calm.
  STEP_MAX_V  = [15.0, 28.0]     # vEgo breakpoints (m/s)
  STEP_MAX_BP = [0.10, 0.05]     # per-decision torque step cap (frac)

  # κ_meas magnitude below which its SIGN is yaw-rate noise (route 39b
  # seg 18 observed ±0.0002 at 30 m/s). The ISO overshoot gate requires
  # |κ_meas| above this floor — see the overshooting comment in update().
  KMEAS_SIGN_FLOOR = 0.0005

  # Relax-dwell (2026-07-12, route 3a0 seg 8): in a measured deep curve, an
  # overshoot-side error must persist this long before the relax path may
  # command below current torque — bridges modelV2's transient mid-turn
  # κ_des dips (40-50% for ~1 s, κ_meas steady). See deep_relax in update().
  RELAX_DWELL_TICKS = 20      # livePose ticks (1.0 s at 20 Hz)
  RELAX_DWELL_KAPPA = 0.010   # |κ_meas| defining a deep curve (1/m)

  # Rack breakaway torque fraction (stiction floor). 2026-07-03: no longer
  # used to amplify sub-friction commands or emit reverse pulses (stiction
  # special-casing removed — the pulses were churn, not correction; the
  # straight-line wobble they targeted is modelV2 vision noise, handled by
  # the delta_err box filter). Retained only as the tolerance-cancel guard:
  # ramps below this can't move the rack, so there is nothing to cancel.
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

  # ISO 11270 comfort guard, κ-dependent. At small κ (near-straight), tighter
  # half-ISO; ramps up to full ISO at tight curves (where larger accel/jerk
  # are part of normal driving).
  #   ISO_LATERAL_ACCEL = 3.0 m/s²    →  BMW_LATERAL_ACCEL [1.5..3.0]
  #   ISO_LATERAL_JERK  = 5.0 m/s³    →  BMW_LATERAL_JERK  [1.5..5.0]
  # Cancel the ramp when either exceeded, drain torque to 0
  #   |a_y_meas| > BMW_LATERAL_ACCEL — current loading already at limit;
  #     don't push deeper. Uses κ_meas (measured outcome).
  #   |jerk_pred| > BMW_LATERAL_JERK — predicted jerk = v²·(κ_des−κ_meas)/T
  #     using model_action_t (= lat_delay + DT_MDL, sourced per-tick from
  #     liveDelay) as the prediction horizon — matches modeld's lat_action_t
  #     so we predict jerk over the exact horizon the model targets κ over.
  #     Predictive — catches ringing setup
  #     ~100 ms before it appears in κ_meas. Validated against route 2b8
  #     seg 14: at t=848.5s during overshoot, κ_des reversed while κ_meas
  #     still on the wrong side, jerk_pred = 4.8 m/s³ → would have
  #     cancelled the counter-torque ramp that produced the 15.7 m/s³
  #     measured jerk.
  # Bug fix 2026-05-22: LATERAL_CURVATURE second value was 0.05 (out of order),
  # which made np.interp non-monotonic — cancel guards were stuck at the small-κ
  # value (2.0) all the way through |κ|=0.02, then jumped discontinuously to ISO
  # at the boundary. Corrected to 0.005 so the table ramps as designed.
  # Small-κ thresholds also tightened 2.0 → 1.5 — at near-straight, brief
  # κ_des excursions or lateral disturbances should cancel the ramp earlier
  # (more conservative on small-κ overshoot).
  LATERAL_CURVATURE = [0.001, 0.005, 0.01, 0.02]
  LATERAL_ACCEL_BP = [1.5, 1.5, 2.5, ISO_LATERAL_ACCEL]
  LATERAL_JERK_BP = [1.5, 1.5, 3.0, ISO_LATERAL_JERK]

  # Rear-axle bicycle-model wheelbase (m). Used for κ ↔ δ conversion.
  L = float(CP.wheelbase)

  _sm = messaging.SubMaster(['livePose'])

  state = {
    'torque': 0.0,             # current commanded torque fraction (advances by ramp_step each CAN tick)
    'target_frac': 0.0,        # plant-inversion target set each decision (cadence ≈ model_action_t/2)
    'ramp_step': 0.0,          # per-frame torque increment = (target − torque) / spread_frames
    'ramp_frames': 0,          # CAN frames left in current ramp
    'tick_count': ACTION_CADENCE_TICKS_FALLBACK,  # primed so first livePose tick fires cadence immediately (no engagement gap)
    'action': 'init',          # debug: hold_zero / hold_curve / ramp / relax_dwell / cancel_tol / cancel_accel / cancel_jerk / idle
    'delta_err': 0.0,          # debug: front-wheel-angle error (rad), what controller acts on
    'lat_pub': None,
    'desired': 0.0, 'measured': 0.0,
    'a_y_meas': 0.0,              # debug: v²·κ_meas (m/s²)
    'jerk_pred': 0.0,             # debug: v²·κ_err/τ (m/s³)
    'hold_f': 0.0,                # debug: curvature hold factor [0,1]
    'hold_cap': 0.0,              # debug: cap on held torque (deadzone-edge P value, frac)
    'relax_ticks': 0,             # consecutive livePose ticks of overshoot-side error while deep in a curve
  }

  def update(active, CS, VM, params, steer_limited_by_safety, desired_curvature, curvature_limited, lat_delay):
    pid_log = log.ControlsState.LateralTorqueState.new_message()
    pid_log.version = 11

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

      # Reference is the raw model κ_des — no filter on κ_des itself.
      # State['desired'] feeds δ_des, kappa_scale, jerk_pred, ISO guards, and
      # telemetry. Keeping it raw means no held bias (the drift failure mode
      # of the prior κ_des-hysteresis attempt) and downstream magnitude-
      # sensitive logic sees the true model intent.
      state['desired'] = float(desired_curvature)

      # Curvature-dependent hold factor (2026-07-03). In a curve, "on-target"
      # is achieved WITH torque applied — that standing torque ≈ the self-
      # aligning torque, which above ~40° wheel exceeds rack stiction
      # (route 380/384: hold-at-zero let the wheel unwind → 0.6 Hz limit
      # cycle). hold_f scales the deadzone / tolerance-cancel target:
      # 0 on straights (pure stiction-hold, below-BP includes the ±0.003
      # modelV2 wobble band), 1 in real curves (keep what got us on-target).
      # The held value is bounded by what the P-term was already commanding
      # and re-derives from state['torque'] every decision — no learning
      # state, no ratchet (the hold-bias integral failure mode).
      hold_f = float(np.interp(abs(state['desired']), HOLD_KAPPA_BP, [0.0, 1.0]))
      state['hold_f'] = hold_f

      # 8.5 m/s = ~30 kph, BMW DCC minimum engagement speed. Below this the
      # controller is never active, so the floor only protects κ_meas from
      # div-by-near-zero during disengaged crawl.
      v = max(float(lp.velocityDevice.x) if _sm.seen['livePose'] else CS.vEgo, 8.5)
      state['measured'] = float(lp.angularVelocityDevice.z) / v

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

      # Held torque for the deadzone / tolerance-cancel (route 385 seg 27
      # review, 2026-07-03). "Keep what you have" needs two qualifiers:
      #   - sign-guard: never hold torque opposing the commanded curve — on a
      #     rack with no self-centering that actively drives the wrong way
      #     (seg 27: 400 ms counter-curve holds during overshoot recovery,
      #     63 vs 38 deg/s subsequent rate bursts).
      #   - magnitude cap: anything above what P would command while "almost
      #     on-target" is arrival momentum, not holding torque (seg 27
      #     latched 0.588 where steady SAT ≈ 0.15 → cancel_accel dump →
      #     deep unwind cycle).
      # 2026-07-04: the cap is DECOUPLED from the deadzone tolerance. It was
      # originally slope·kappa_scale·v²·tolerance, but when the κ-widened
      # DRIFT_M was reverted (deadzone back to 0.02 m), deriving the cap from
      # the now-tight tolerance would shrink it ~5× (0.31 → 0.06 at the
      # route-384 operating point) — BELOW the measured steady SAT
      # (0.08–0.15 frac), breaking hold_curve. The cap keeps its own fixed
      # reference drift (HOLD_CAP_DRIFT_M = 0.10 m — the field-verified
      # scale from the seg-27 fix), so the deadzone and the hold cap tune
      # independently.
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
      # instantly), and torque above breakaway. ISO guards are untouched —
      # genuine measured overshoot (a_y/jerk) still cancels immediately.
      # True exits (κ_des drops and stays) proceed after the dwell: cost is
      # ≤1 s of extra curvature (~0.2 m lateral at 9 m/s).
      deep_relax = (abs(state['measured']) > RELAX_DWELL_KAPPA
                    and state['desired'] * state['measured'] > 0.0
                    and abs(state['torque']) > FRICTION
                    and delta_err * delta_des < 0.0)
      state['relax_ticks'] = state['relax_ticks'] + 1 if deep_relax else 0
      dwelling = 0 < state['relax_ticks'] <= RELAX_DWELL_TICKS

      # ISO 11270 half-comfort guard, gated on plant overshoot. Fires only
      # when (κ_des − κ_meas)·κ_meas < 0 — i.e., plant has turned more than
      # the planner asked for (or to the wrong side of zero). During
      # legitimate under-tracking (plant lagging κ_des in a hard curve),
      # the guard stays silent so the controller can keep tracking. Route
      # 2ba seg 22 evidence: a_y_meas crept above 1.5 during chassis catch-
      # up while still under-tracking (κ_meas < κ_des); the un-gated guard
      # zeroed step_remaining, the controller couldn't apply torque, and
      # the car drifted 1.29 m outside the lane.
      #
      # When overshoot is real, drain τ toward 0 (reverse-FRICTION unwind
      # pulse removed 2026-07-03): with torque relaxed, tire aligning forces
      # return the wheel — at the angles where these guards fire, aligning
      # torque exceeds rack stiction (route 380/384 evidence). No active
      # counter-push, so the cancel can't create the seg-14 ringing pattern
      # in reverse.
      a_y_meas = v * v * state['measured']
      jerk_pred = v * v * (state['desired'] - state['measured']) / model_action_t
      state['a_y_meas'] = a_y_meas
      state['jerk_pred'] = jerk_pred
      # 2026-07-09 (route 39b seg 18): overshoot requires |κ_meas| above the
      # yaw-noise floor — near zero the SIGN of κ_meas is pure noise
      # (±0.0002 observed), so the wrong-side test degenerated: a gentle
      # highway-left build was repeatedly cancel_jerk'd at κ_meas = +0.0002
      # ("wrong side" by noise) while the car was under-turning. The error
      # then grew until a late, big correction swung the wheel back and
      # forth at 108 km/h. Below the floor the guard stays silent — the
      # step-capped push (≤ 0.7 m/s³ actual jerk at highway) is already
      # gentler than the guard's own threshold.
      overshooting = ((state['desired'] - state['measured']) * state['measured'] < 0
                      and abs(state['measured']) > KMEAS_SIGN_FLOOR)
      cancel_reason = None
      BMW_LATERAL_ACCEL = float(np.interp(abs(state['desired']),
                                          LATERAL_CURVATURE, LATERAL_ACCEL_BP))
      BMW_LATERAL_JERK = float(np.interp(abs(state['desired']),
                                          LATERAL_CURVATURE, LATERAL_JERK_BP))
      if overshooting:
        if abs(a_y_meas) > BMW_LATERAL_ACCEL:
          cancel_reason = 'cancel_accel'
        elif abs(jerk_pred) > BMW_LATERAL_JERK:
          cancel_reason = 'cancel_jerk'
      if cancel_reason:
        # Cancel preempts the cadence decision this tick — reset window so
        # the next plant-inversion decision is one full cycle after the drain.
        # Drain to 0 (reverse-FRICTION unwind pulse removed 2026-07-03 with
        # the rest of the stiction special-casing): torque relaxes and tire
        # aligning forces return the wheel — at the angles where the
        # overshoot guards fire, aligning torque exceeds rack stiction
        # (route 380/384 evidence). Only re-arm the ramp if the target
        # changed, so continuous overshoot doesn't restart the drain window
        # every tick (exponential-decay bug guarded against as before).
        unwind_target = 0.0
        if state['target_frac'] != unwind_target:
          state['target_frac'] = unwind_target
          state['ramp_step'] = (unwind_target - state['torque']) / spread_frames
          state['ramp_frames'] = spread_frames
        state['action'] = cancel_reason
        state['tick_count'] = 0
      elif (state['action'] == 'ramp' and abs(delta_err) <= 1.2*HOLD_BAND
            and state['ramp_frames'] > 0 and abs(state['target_frac']) > FRICTION):
        # Tolerance-cancel: error fell into the success band while a PUSH
        # ramp is still in flight. Without this, the ramp keeps driving
        # torque toward a stale target until the next 250 ms cadence.
        # Gated on action=='ramp' (route 385 review, 2026-07-03): hold
        # ramps also set ramp_frames, and un-gated this branch fired on
        # them every in-band tick — pinning tick_count (cadence stretched
        # 300→550 ms) and flooding telemetry with phantom cancel_tol
        # (~10 of 11 in-curve ticks), corrupting the cancel_* field-health
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
        # occupancy counts over-attributed ramp/cancel states (transition-
        # level counting was the workaround). Holds keep their label — they
        # re-fire every cadence and their occupancy IS meaningful. The
        # cancel_tol gate is unaffected: it requires ramp_frames > 0, and
        # in-flight ramps (ramp_frames > 0) never expire here.
        if state['ramp_frames'] == 0 and state['action'] in (
            'ramp', 'relax_dwell', 'cancel_tol', 'cancel_accel', 'cancel_jerk'):
          state['action'] = 'idle'

      if state['tick_count'] >= action_cadence_ticks:
        state['tick_count'] = 0

        # Plant-inversion target torque in angle domain — the steady-state
        # aligning torque required to hold δ_err. Soft-deadband subtracts
        # the tolerance from |δ_err| so τ_Nm starts at 0 when crossing the
        # boundary instead of stepping to T_CAP_SLOPE·v²·tolerance (~1.1 Nm
        # at 120 kph) — without it, the boundary crossing would dominate
        # the torque profile and feel like a discrete step.
        #   τ_Nm = T_CAP_SLOPE · v² · (δ_err − tolerance·sign(δ_err))
        # Inside tolerance → 0 (stiction holds; no chatter at the boundary).
        if abs(delta_err) <= HOLD_BAND:
          # On-target. Straights (hold_f=0): drain to 0, stiction holds.
          # Curves (hold_f→1): keep the torque that achieved on-target —
          # it approximates the self-aligning torque, which beats stiction
          # at curve angles; target-0 here was the 0.6 Hz limit cycle's
          # driver (route 380/384). Re-derived from state['torque'] each
          # decision: bounded by the P-term's own command, no ratchet.
          # (brake_zero one-shot reverse pulse removed 2026-07-03 —
          # residual deadzone-entry momentum is left to rack friction.)
          # held_target is sign-guarded (never counter-curve) and capped at
          # the deadzone-edge P value — see the hold_cap block above.
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
          # normal relax staircase; ISO cancels bypass this entirely.
          if dwelling:
            target_frac = math.copysign(min(abs(state['torque']), hold_cap), state['torque'])
            state['action'] = 'relax_dwell'
          # Per-decision step cap (route 385 seg 27 review, 2026-07-03):
          # human-style gradual steering — move at most STEP_MAX toward the
          # P target per decision, let the plant respond one cadence, then
          # re-measure and step again. Never apply excessive torque
          # abruptly: seg 27 showed single decisions swinging Δ0.69 frac
          # (8.3 Nm) which drove 150 deg/s wheel bursts and the
          # over-latch → cancel_accel dump → deep-unwind cycle. Max slew
          # is now ~0.33 frac/s (~4 Nm/s); ISO guards still cancel
          # overshoot instantly, speedlimitd handles curve-entry speed.
          step_max = float(np.interp(v, STEP_MAX_V, STEP_MAX_BP))
          step = float(np.clip(target_frac - state['torque'], -step_max, step_max))
          target_frac = float(np.clip(state['torque'] + step, -t_cap_frac, t_cap_frac))

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
          'desired_raw': float(desired_curvature),     # pre-filter, for box-filter diagnostics
          'measured': float(state['measured']),
          'err': float(err),
          'delta_err': float(state['delta_err']),             # filtered (what controller acts on)
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
        }
        state['lat_pub'].send(payload)
      except Exception:
        pass

    return -output, 0.0, pid_log

  lac.update = update
  return result
