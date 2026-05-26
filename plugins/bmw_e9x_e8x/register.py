"""BMW car interface registration — monkey-patches opendbc at plugin load time.

Injects BMW E82/E90 into opendbc's interfaces, fingerprints, and platforms
when the plugin is enabled. When disabled, BMW is not in the system.

This runs at module exec time (during registry.load_plugin), before card.py
starts fingerprinting. No opendbc fork needed — we mutate the dicts in-place.
"""
import os
import sys

# Ensure the plugin's bmw/ package is importable
_PLUGIN_DIR = os.path.dirname(os.path.abspath(__file__))
if _PLUGIN_DIR not in sys.path:
  sys.path.insert(0, _PLUGIN_DIR)


def _register_interfaces():
  """Monkey-patch BMW into opendbc's car interfaces system.

  Mutates interfaces/fingerprints/platforms dicts in-place at module load time.
  Since card.py holds a reference to the same dict objects, BMW becomes visible.
  """
  from bmw.interface import CarInterface
  from bmw.values import CAR

  # Patch interfaces dict
  try:
    from opendbc.car.car_helpers import interfaces
    interfaces[CAR.BMW_E82] = CarInterface
    interfaces[CAR.BMW_E90] = CarInterface
  except ImportError:
    pass

  # Patch global fingerprints
  try:
    from opendbc.car.fingerprints import _FINGERPRINTS, FW_VERSIONS as GLOBAL_FW
    from bmw.fingerprints import FINGERPRINTS as BMW_FP, FW_VERSIONS as BMW_FW
    _FINGERPRINTS.update({str(k): v for k, v in BMW_FP.items()})
    GLOBAL_FW.update({str(k): v for k, v in BMW_FW.items()})
  except (ImportError, AttributeError):
    pass

  # Patch fw_versions globals (FW_QUERY_CONFIGS, VERSIONS, MODEL_TO_BRAND, REQUESTS)
  try:
    from opendbc.car.fw_versions import FW_QUERY_CONFIGS, VERSIONS, MODEL_TO_BRAND, REQUESTS
    from bmw.fingerprints import FW_VERSIONS as BMW_FW
    from bmw.values import FW_QUERY_CONFIG as BMW_FW_CONFIG
    FW_QUERY_CONFIGS['bmw'] = BMW_FW_CONFIG
    VERSIONS['bmw'] = {str(k): v for k, v in BMW_FW.items()}
    for model in BMW_FW:
      MODEL_TO_BRAND[str(model)] = 'bmw'
    for r in BMW_FW_CONFIG.requests:
      REQUESTS.append(('bmw', BMW_FW_CONFIG, r))
  except (ImportError, AttributeError):
    pass

  # Patch get_torque_params to include BMW models
  try:
    import opendbc.car.interfaces as _intf
    _orig_get_torque = _intf.get_torque_params
    import tomllib
    with open(os.path.join(_PLUGIN_DIR, 'torque_params.toml'), 'rb') as f:
      toml = tomllib.load(f)
    legend = toml.pop('legend', ['LAT_ACCEL_FACTOR', 'MAX_LAT_ACCEL_MEASURED', 'FRICTION'])
    torque = {model: dict(zip(legend, vals)) for model, vals in toml.items()}
    def _patched_get_torque_params():
      params = _orig_get_torque()
      for model, values in torque.items():
        if model not in params:
          params[model] = values
      return params
    _intf.get_torque_params = _patched_get_torque_params
  except (ImportError, AttributeError):
    pass

  # Patch global platforms
  try:
    from opendbc.car.values import PLATFORMS
    PLATFORMS[str(CAR.BMW_E82)] = CAR.BMW_E82
    PLATFORMS[str(CAR.BMW_E90)] = CAR.BMW_E90
  except (ImportError, AttributeError):
    pass


# Run at module load time — triggered by registry.load_plugin() -> exec_module()
_register_interfaces()



def on_post_actuators(default, actuators, CS, long_plan):
  """Hook callback: inject vTarget from longitudinal planner into actuators.speed.

  long_plan.vTarget is the trajectory speed at action_t (longitudinalActuatorDelay
  + DT_MDL), time-aligned with long_plan.aTarget. Both come from the same
  get_accel_from_plan call in the planner. Using vTarget (instead of speeds[0],
  which is the MPC's filtered initial state and can drift above v_ego after
  sustained +accel intent) makes carcontroller's v_error gate consistent with
  the slope encoded in aTarget.
  """
  actuators.speed = long_plan.vTarget
  return None


def on_cruise_initialized(result, v_cruise_helper, CS):
  """Hook callback: restore last cruise ceiling on re-engagement.

  Stock openpilot resets cruise speed to V_CRUISE_INITIAL on every engagement
  for BMW because engagement is a state transition (not a resume button press).
  This restores the user's last-adjusted ceiling within the same onroad session.
  """
  if _read_param('CruiseCeilingMemory') == '0':
    return result

  if 30 <= v_cruise_helper.v_cruise_kph_last <= 145:
    v_cruise_helper.v_cruise_kph = v_cruise_helper.v_cruise_kph_last
    v_cruise_helper.v_cruise_cluster_kph = v_cruise_helper.v_cruise_kph_last
  return result


def _read_param(key):
  try:
    with open(os.path.join(_PLUGIN_DIR, 'data', key)) as f:
      return f.read().strip()
  except (FileNotFoundError, OSError):
    return ''


def _write_param(key, value):
  data_dir = os.path.join(_PLUGIN_DIR, 'data')
  os.makedirs(data_dir, exist_ok=True)
  with open(os.path.join(data_dir, key), 'w') as f:
    f.write(value)



def on_vehicle_settings(items, CP):
  """Hook callback: populate Vehicle panel with BMW-specific toggles."""
  if CP.brand != 'bmw':
    return items

  from openpilot.system.ui.widgets.list_view import toggle_item

  items.append(toggle_item(
    "Temperature Overlay",
    "Show coolant and oil temperature at the bottom-right corner of the onroad HUD.",
    _read_param('TemperatureOverlay') != '0',
    callback=lambda state: _write_param('TemperatureOverlay', '1' if state else '0'),
  ))

  items.append(toggle_item(
    "Resume Button Repurposed",
    "Short press: resume (disengaged) or toggle speed limit confirm (engaged). Long press: cycle follow distance.",
    initial_state=True,
    enabled=False,
  ))

  return items


def on_lat_controller_init(result, lac, CP):
  """Plant-inversion at 500 ms horizon in front-wheel-angle space.

  BMW E90 hydraulic rack has high breakaway friction and no alignment-torque
  self-centering — the wheel holds its angle at zero torque. So:
    - Inside tolerance: drive torque → 0 and let stiction hold. No chatter.
    - Outside tolerance: compute the torque that would move the front wheel by
      δ_err over 500 ms (plant-inversion accounting for first-order lag), ramp
      to it over one 250 ms decision.

  Error in rear-axle bicycle-model front-wheel-angle space:
    δ_des  = atan(κ_des  · L)        L = CP.wheelbase
    δ_meas = atan(κ_meas · L)        κ_meas = yawRate / v_ego
    δ_err  = δ_des − δ_meas

  Tolerance (physical 0.05 m drift over 0.5 s, speed-adaptive, scales 1/v²):
    tolerance = 2 · 0.05 · L / (v² · 0.5²)

  Plant-inversion target torque, angle domain (linear tire regime):
    τ_Nm_target = T_CAP_SLOPE · v² · (δ_err − tolerance·sign(δ_err))
    Clamp to ±T_CAP(v, δ):
      T_CAP_NM = min(STEER_MAX, T_CAP_BASE + T_CAP_SLOPE · v²·|δ_des|)
    Same slope drives both target and cap.
    If |target_frac| < FRICTION, push to ±FRICTION to break stiction.
    BASE is the hydraulic rack's stiction floor. Hard stop at STEER_MAX
    (panda limit) preserves lane authority during transient over-envelope
    events before speedlimitd trims v.

  Ramp: ramp_step = (T_peak − state['torque']) / spread_frames, applied
  per CAN frame for spread_frames frames; panda enforces wire-rate.
  (spread_frames = round((model_action_t/2)/DT_CAN_TICK), per-tick dynamic.)

  ISO 11270 half-comfort guard (every livePose tick): cancel ramping if
  |a_y_meas| > 1.5 m/s² OR predicted jerk |v²·(κ_des−κ_meas)/0.5| > 2.5 m/s³,
  AND only when plant has actually overshot ((κ_des−κ_meas)·κ_meas < 0).
  Under-tracking (plant lagging in a hard curve) is left to the controller
  to chase. When cancel fires, redirect the ramp toward −FRICTION·sign(κ_meas)
  so the BMW hydraulic rack can unwind via tire aligning forces (won't
  self-center under standing torque).

  Tolerance-cancel (every livePose tick): if |δ_err| drops into the success
  band mid-ramp, redirect torque to 0 (or −FRICTION·sign(δ_err) if plant
  has momentum past the goal). Without this, the in-flight ramp keeps
  pushing toward a stale target until the next 250 ms cadence notices.

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
  # The soft-deadband, FRICTION breakaway, and per-tick tolerance-cancel
  # handle the boundary smoothness.
  T_CAP_BASE_NM = 2.0
  T_CAP_SLOPE_BASE = 1.0
  T_CAP_SCALE_KAPPA = [0.001, 0.01, 0.02]        # |κ_des| breakpoints (1/m)
  T_CAP_SCALE_BP    = [1.0, 2.5, 3.0]           # scale factor on T_CAP_SLOPE_BASE
  # Plant prediction horizon — sourced per-tick from controlsd's lat_delay
  # (= liveDelay.lateralDelay + LAT_SMOOTH_SECONDS) and matched to where
  # modeld samples κ_des: lat_action_t = lat_delay + DT_MDL (modeld.py:391).
  # Used by jerk_pred (the ISO-jerk overshoot guard) and by the kinematic
  # deadzone formula (DRIFT_M block below) — same horizon for both.
  #
  # For BMW: lagd never converges (it correlates on latcontrol_torque
  # telemetry that our front-wheel-angle plant-inversion controller doesn't
  # produce), so liveDelay.lateralDelay is permanently pinned at initial_lag
  # = CP.steerActuatorDelay + 0.2 (lagd.py:181). CP.steerActuatorDelay in
  # bmw/interface.py is therefore the single knob controlling both modeld's
  # lat_action_t AND our model_action_t — change it there to retune both.
  DT_MDL = 0.05                                  # openpilot model dt (common/realtime.py)
  MODEL_ACTION_T_FALLBACK = 0.55                 # used only if lat_delay arrives as 0/None

  # Feedback deadzone: engage only when δ_err would cause ≥ DRIFT_M
  # lateral drift within model_action_t (per-tick horizon).
  #   drift(T) = ½ · δ_err / L · v² · T²  ⇒  δ_tol = 2 · DRIFT_M · L / (v·T)²
  # The 1/v² factor gives natural speed adaptation (tighter at high v).
  # 2026-05-20: a constant-angle reformulation (TOL_DEG_CONST = 0.35°) was
  # tried and reverted — the wide highway-speed deadzone allowed sustained
  # δ_err inside the band to accumulate into 1.3–1.7 m lateral drift when
  # lane_centering was off (route 31b seg 8/15). The 1/v² scaling is what
  # keeps the κ-controller actively tracking at highway speed when it's the
  # sole loop closing against drift.
  DRIFT_M = 0.02           # m of allowed drift over model_action_t

  # κ-gated box filter on delta_err. Bare-model κ_des on near-straight
  # (lane_centering off) produces high-rate sign-flips that propagate into
  # delta_err. Each delta_err sign-flip across the tolerance band triggers
  # a cancel_tol / brake_zero cycle with a counter-direction FRICTION pulse
  # — the actual felt swaying isn't κ_des amplitude, it's the rate of
  # those unwind pulses.
  #
  # Filter target is delta_err (not raw κ_des) — keeps state['desired']
  # raw so:
  #   - the reference is always fresh, no held bias → no drift accumulation
  #     (the failure mode of the prior hysteresis-on-κ_des attempt, daef207,
  #     which held κ_des for up to 44 s and produced 32% time |offset|>0.5m
  #     on route 322 with no controller correction)
  #   - kappa_scale, jerk_pred, ISO guards all see raw κ_des magnitude
  #     (no smoothed-magnitude distortion)
  # The smoothed view is given only to the cadence decision's error band
  # check — that's where sign-flips matter for cancel events.
  #
  # Gate on |raw κ_des| < KD_GATE: filter active only in the near-straight
  # wobble regime. Real curves bypass (no filter lag during curve entry).
  # Lane changes (modelV2.meta.laneChangeState != off) also bypass
  # (canonical signal stock controlsd uses to gate lane_centering).
  # de_filter_window subscribes to action_cadence_ticks for single-knob
  # coupling: at SAD=0.4 → cadence=6 → window=6 (300 ms box, 150 ms group
  # delay).
  #
  # 2026-05-22 (post-route-322 tuning): KD_GATE = 0.002 (was 0.005).
  # Rationale: vision-noise κ_des wobble itself reaches ±0.003, so a wider
  # gate would smooth wobble peaks (which are real model κ excursions,
  # noisy but committed) and could compromise the controller's response
  # to small-amplitude real corrections needed for lane-keeping on
  # straights. Narrowing to 0.002 focuses the filter on zero-crossing
  # windows (where delta_err sign-flips actually happen) and lets
  # wobble peaks pass through raw so the controller can respond. Expected
  # sign-flip reduction is more modest than at 0.005 but no compromise
  # to real-correction capability.
  KD_GATE = 0.002                           # |raw κ_des| at which delta_err filter bypasses

  # Breakaway torque fraction (rack stiction floor). Sub-friction commands
  # don't move the hydraulic rack, so the controller pushes target to ±friction
  # to break stiction. Initial estimate from memory; tune if needed via a
  # dedicated stop-and-ramp experiment (not online — see shadow-plant notes).
  FRICTION = 0.05

  # ISO 11270 comfort guard, κ-dependent. At small κ (near-straight), tighter
  # half-ISO; ramps up to full ISO at tight curves (where larger accel/jerk
  # are part of normal driving).
  #   ISO_LATERAL_ACCEL = 3.0 m/s²    →  BMW_LATERAL_ACCEL [1.5..3.0]
  #   ISO_LATERAL_JERK  = 5.0 m/s³    →  BMW_LATERAL_JERK  [1.5..5.0]
  # Cancel the ramp when either exceeded, redirect toward FRICTION-level
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

  # modelV2 subscribed for meta.laneChangeState (filter-disable gate, see below).
  # Future modelV2 fields (lane line confidence, position uncertainty, frameId,
  # desire predictions, etc.) accessible via the same subscription.
  _sm = messaging.SubMaster(['livePose', 'modelV2'])

  state = {
    'torque': 0.0,             # current commanded torque fraction (advances by ramp_step each CAN tick)
    'target_frac': 0.0,        # plant-inversion target set each decision (cadence ≈ model_action_t/2)
    'ramp_step': 0.0,          # per-frame torque increment = (target − torque) / spread_frames
    'ramp_frames': 0,          # CAN frames left in current ramp
    'tick_count': ACTION_CADENCE_TICKS_FALLBACK,  # primed so first livePose tick fires cadence immediately (no engagement gap)
    'action': 'init',          # debug: hold_zero / brake_zero / breakaway / ramp / cancel_accel / cancel_jerk
    'delta_err': 0.0,          # debug: filtered front-wheel-angle error (rad), what controller acts on
    'delta_err_raw': 0.0,      # debug: pre-filter delta_err (rad)
    'lat_pub': None,
    'desired': 0.0, 'measured': 0.0,
    'de_buffer': [],            # rolling window of recent delta_err for box filter
    'a_y_meas': 0.0,              # debug: v²·κ_meas (m/s²)
    'jerk_pred': 0.0,             # debug: v²·κ_err/τ (m/s³)
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

      # 8.5 m/s = ~30 kph, BMW DCC minimum engagement speed. Below this the
      # controller is never active, so the floor only protects κ_meas from
      # div-by-near-zero during disengaged crawl.
      v = max(float(lp.velocityDevice.x) if _sm.seen['livePose'] else CS.vEgo, 8.5)
      state['measured'] = float(lp.angularVelocityDevice.z) / v

      # Front-wheel-angle error (rear-axle bicycle model).
      delta_des = math.atan(state['desired'] * L)
      delta_meas = math.atan(state['measured'] * L)
      delta_err_raw = delta_des - delta_meas

      # κ-gated box filter on delta_err (see KD_GATE block above).
      # Smooths the controller's view of the error signal so wobble-induced
      # delta_err sign-flips don't repeatedly cross the tolerance band and
      # trigger cancel_tol / brake_zero unwind pulses. Filter bypassed when:
      #   - |κ_des| ≥ KD_GATE: real curve, controller needs the raw error
      #   - lane change active: prevents filter lag across the mid-lane-
      #     change κ_des zero-crossing (route 31d seg 7 evidence)
      # Filter window = action_cadence_ticks (cadence-coupled, single-knob).
      # state['delta_err'] holds the FILTERED value (used downstream).
      # Telemetry publishes both raw and filtered for verification.
      lane_change_active = (_sm['modelV2'].meta.laneChangeState != log.LaneChangeState.off)
      if abs(state['desired']) >= KD_GATE or lane_change_active:
        state['de_buffer'].clear()
        delta_err = delta_err_raw
      else:
        state['de_buffer'].append(delta_err_raw)
        if len(state['de_buffer']) > action_cadence_ticks:
          state['de_buffer'].pop(0)
        delta_err = sum(state['de_buffer']) / len(state['de_buffer'])
      state['delta_err'] = delta_err
      state['delta_err_raw'] = delta_err_raw

      # Speed-scaled tolerance: DRIFT_M drift over model_action_t, 1/v² scaling.
      # Restored 2026-05-20 after route 31b seg 8/15 showed the constant-angle
      # 0.35° deadzone (commit 715114d, now reverted) allowed 1.3–1.7 m lateral
      # drift at 85 kph when lane_centering is off: δ_err of 0.2–0.35° sat
      # inside the 0.35° band for seconds while stiction held the rack at zero,
      # and the car coasted ~half a lane out of position with 98% torque=0.
      # The kinematic 1/v² formula keeps the deadzone tight at highway speed
      # (~0.05° at 85 kph) where the κ-controller is the only loop closing
      # against drift in the lane_centering-off configuration. Re-evaluate the
      # constant-angle reformulation only with lane_centering active.
      lookahead_m = v * model_action_t
      tolerance = 2.0 * DRIFT_M * L / (lookahead_m ** 2)

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
      # Reverse-breakaway unwind: when overshoot is real, drain τ toward
      # −FRICTION·sign(κ_meas). The BMW hydraulic rack has high stiction
      # and won't self-center under standing torque; that small counter-
      # direction torque (~0.6 Nm) breaks stiction so tire aligning forces
      # can return the wheel toward center. FRICTION-level (not full
      # counter-correction) prevents the cancel from creating the seg-14
      # ringing pattern in reverse.
      a_y_meas = v * v * state['measured']
      jerk_pred = v * v * (state['desired'] - state['measured']) / model_action_t
      state['a_y_meas'] = a_y_meas
      state['jerk_pred'] = jerk_pred
      overshooting = (state['desired'] - state['measured']) * state['measured'] < 0
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
        # overshooting=True implies κ_meas != 0; unwind toward opposite sign.
        # Cancel preempts the cadence decision this tick — reset window so
        # the next plant-inversion decision is one full cycle after the unwind.
        # Only re-arm the ramp if the unwind target changed (first cancel, or
        # κ_meas flipped sign). If we're already ramping toward this same
        # unwind target, leave it alone — re-arming on every continuous-
        # overshoot tick would restart the 250 ms window from current torque
        # each time, producing exponential decay (slower unwind the harder
        # the plant fights, the opposite of what a safety guard should do).
        unwind_target = -math.copysign(FRICTION, state['measured'])
        if state['target_frac'] != unwind_target:
          state['target_frac'] = unwind_target
          state['ramp_step'] = (unwind_target - state['torque']) / spread_frames
          state['ramp_frames'] = spread_frames
        state['action'] = cancel_reason
        state['tick_count'] = 0
      elif abs(delta_err) <= 1.2*tolerance and state['ramp_frames'] > 0 and abs(state['target_frac']) > FRICTION:
        # Tolerance-cancel: error fell into the success band while a push
        # ramp is still in flight. Without this, the ramp keeps driving
        # torque toward a stale target until the next 250 ms cadence. If
        # plant has momentum in the error direction, brake with reverse
        # FRICTION; otherwise drain to 0. Idempotent like the ISO cancel.
        if state['torque'] * delta_err > 0:
          unwind_target = -math.copysign(FRICTION, delta_err)
        else:
          unwind_target = 0.0
        if state['target_frac'] != unwind_target:
          state['target_frac'] = unwind_target
          state['ramp_step'] = (unwind_target - state['torque']) / spread_frames
          state['ramp_frames'] = spread_frames
        state['action'] = 'cancel_tol'
        state['tick_count'] = 0
      else:
        state['tick_count'] += 1

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
        # Sub-breakaway commands won't move the rack → push to ±FRICTION.
        prev_action = state['action']
        if abs(delta_err) <= tolerance:
          # Brake-to-hold: if we just exited a ramp into deadzone with τ
          # still loading toward δ_err, plant has momentum that would
          # cross zero. Set target to FRICTION-level reverse torque to
          # actively decelerate (BMW rack is sticky; small reverse torque
          # is enough to halt residual motion). One-shot per ramp →
          # deadzone transition: next decision sees prev_action='brake_zero'
          # and falls through to hold_zero (target=0) so τ relaxes.
          if prev_action == 'ramp' and state['torque'] * delta_err > 0:
            target_frac = -math.copysign(FRICTION, delta_err)
            state['action'] = 'brake_zero'
          else:
            target_frac = 0.0
            state['action'] = 'hold_zero'
        else:
          # Curvature scale: 1.0 on straights (|κ_des| ≤ 0.001) rising linearly
          # to 2.5 at |κ_des| = 0.01 to 3.0 on tight curves (|κ_des| ≥ 0.02).
          # Inlined into both target and cap formulas so the κ_des-dependence
          # is visible at the math. 2026-05-22 (route 326): reduced near-
          # straight scale 1.5 → 1.0 (matches T_CAP_SLOPE_BASE) to suppress
          # over-correction the user felt at the small-κ regime; "feels much
          # better" verbatim on route 326 with the change in place.
          kappa_scale = float(np.interp(abs(state['desired']),
                                        T_CAP_SCALE_KAPPA, T_CAP_SCALE_BP))
          effective_err = delta_err - math.copysign(tolerance, delta_err)
          target_nm = T_CAP_SLOPE_BASE * kappa_scale * v * v * effective_err
          target_frac = target_nm / CCP.STEER_MAX
          if abs(target_frac) < FRICTION:
            target_frac = math.copysign(FRICTION, delta_err)
            state['action'] = 'breakaway'
          else:
            state['action'] = 'ramp'
          # v²·|δ|-scaled cap, clipped at STEER_MAX (panda hard limit).
          # Authority grows with commanded a_y_des — straights stay near BASE,
          # tight turns can reach STEER_MAX (transient over-envelope; speedlimitd
          # bleeds v).
          t_cap_nm = min(CCP.STEER_MAX,
                         T_CAP_BASE_NM + T_CAP_SLOPE_BASE * kappa_scale * v * v * abs(delta_des))
          t_cap_frac = t_cap_nm / CCP.STEER_MAX
          target_frac = float(np.clip(target_frac, -t_cap_frac, t_cap_frac))

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
          'delta_err_raw': float(state['delta_err_raw']),     # pre-filter, for diagnostics
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
        }
        state['lat_pub'].send(payload)
      except Exception:
        pass

    return -output, 0.0, pid_log

  lac.update = update
  return result


def on_health_check(acc, **kwargs):
  try:
    from opendbc.car.car_helpers import interfaces
    from bmw.values import CAR
    registered = CAR.BMW_E90 in interfaces or str(CAR.BMW_E90) in interfaces
  except Exception:
    registered = False
  result = {"status": "ok" if registered else "warning", "interfaces_registered": registered}
  if not registered:
    result["warnings"] = ["BMW interfaces not registered in opendbc"]
  return {**acc, "bmw-e9x-e8x": result}
