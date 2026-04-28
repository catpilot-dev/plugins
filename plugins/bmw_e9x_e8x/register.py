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
  """Hook callback: inject vTarget from longitudinal planner into actuators.speed."""
  if len(long_plan.speeds):
    actuators.speed = long_plan.speeds[0]
  return None


def on_cruise_initialized(result, v_cruise_helper, CS):
  """Hook callback: restore last cruise ceiling on re-engagement.

  Stock openpilot resets cruise speed to V_CRUISE_INITIAL on every engagement
  for BMW because engagement is a state transition (not a resume button press).
  This restores the user's last-adjusted ceiling within the same onroad session.
  """
  try:
    with open(os.path.join(_PLUGIN_DIR, 'data', 'CruiseCeilingMemory')) as f:
      if f.read().strip() == '0':
        return result
  except (FileNotFoundError, OSError):
    pass  # default: enabled

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

  Tolerance (physical 0.025 m drift over 0.5 s, speed-adaptive, scales 1/v²):
    tolerance = 2 · 0.025 · L / (v² · 0.5²)

  Plant-inversion target torque, angle domain (linear tire regime):
    τ_Nm_target = slope_eff · v² · δ_err
    Clamp to ±T_CAP(v, δ):
      T_CAP_NM = min(STEER_MAX, T_CAP_BASE + slope_eff · v²·|δ_des|)
    slope_eff is gain-scheduled on |κ_des|:
      slope_eff = T_CAP_SLOPE_LO + clip01((|κ|−κ_LO)/(κ_HI−κ_LO)) · (HI − LO)
    Low slope on near-straights → gentle, no plant overshoot / ringing.
    High slope at tight turns (|κ_des| ≥ KAPPA_TIGHT) → full authority.
    Same slope_eff drives both target and cap.
    If |target_frac| < FRICTION, push to ±FRICTION to break stiction.
    BASE is the hydraulic rack's stiction floor. Hard stop at STEER_MAX
    (panda limit) preserves lane authority during transient over-envelope
    events before speedlimitd trims v.

  Ramp: step_remaining = T_peak − state['torque'], drained over 25 CAN frames.

  ISO 11270 half-comfort guard (every livePose tick): cancel ramping if
  |a_y_meas| > 1.5 m/s² OR predicted jerk |v²·(κ_des−κ_meas)/0.5| > 2.5 m/s³,
  AND only when plant has actually overshot ((κ_des−κ_meas)·κ_meas < 0).
  Under-tracking (plant lagging in a hard curve) is left to the controller
  to chase. When cancel fires, redirect the ramp toward −FRICTION·sign(κ_meas)
  so the BMW hydraulic rack can unwind via tire aligning forces (won't
  self-center under standing torque).

  No online adaptation: plant behavior is fully described by T_CAP_SLOPE_*,
  T_CAP_BASE_NM, and FRICTION. Tune these offline from route data; there's
  no scale_by_bin or shadow estimator anymore.
  """
  import math
  from cereal import log
  from cereal import messaging
  from bmw.values import CarControllerParams as CCP

  # Decision cadence & CAN-rate spreading.
  # ACTION_CADENCE_TICKS = 5 livePose ticks × 50 ms = 250 ms decision period.
  # SPREAD_FRAMES is speed-dependent (linearly interpolated):
  #   v=30 kph (8.33 m/s):  spread=10 → 100 ms ramp (agile, sub-cycle)
  #   v=120 kph (33.3 m/s): spread=25 → 250 ms ramp (fills the decision cycle)
  # Capping SPREAD_FRAMES_MAX at 25 keeps ramps within the 250 ms decision
  # window — each cycle's ramp completes before the next decision fires.
  # Higher v gets gentler ramps without de-syncing from cadence.
  ACTION_CADENCE_TICKS = 5
  SPREAD_FRAMES_MIN = 10                  # at v ≤ 30 kph (8.33 m/s)
  SPREAD_FRAMES_MAX = 25                  # at v ≥ 120 kph (33.33 m/s) — fills 250 ms cycle
  V_FOR_SPREAD_MIN = 30.0 / 3.6           # 8.33 m/s
  V_FOR_SPREAD_MAX = 120.0 / 3.6          # 33.33 m/s
  # T_CAP slope gain-schedule (κ_des-adaptive). Linear tire regime:
  #     τ_Nm_hold = slope_eff · v² · δ                 (aligning torque)
  #     slope_eff = lerp(LO → HI, (|κ_des|−κ_LO)/(κ_HI−κ_LO))   clamped
  # Used for both authority and target:
  #   T_CAP(v, δ)  = T_CAP_BASE_NM + slope_eff · v² · |δ_des|   (≤ STEER_MAX)
  #   target_Nm    = slope_eff · v² · δ_err
  # BASE covers the speed- and angle-independent stiction floor.
  # Prior fixed SLOPE=2.0 (route 2b8 baseline): seg-14 ringing on small κ_des
  # (overshoot), seg-6 under-tracking on tight κ_des (insufficient authority).
  # Curvature schedule resolves both: gentle on near-straights (less ringing
  # ingredients), full authority on tight turns. v-independent — same SLOPE
  # multiplier whether κ=0.01 is at parking-lot speed or highway speed.
  T_CAP_BASE_NM = 1.25
  T_CAP_SLOPE_LO  = 1.5      # at |κ_des| ≤ KAPPA_STRAIGHT — gentle
  T_CAP_SLOPE_HI  = 2.5      # at |κ_des| ≥ KAPPA_TIGHT    — full authority
  KAPPA_STRAIGHT  = 0.001    # m⁻¹ — below this, treat as straight
  KAPPA_TIGHT     = 0.010    # m⁻¹ — above this, tight turn
  # Range narrowed (was 1.0..3.0) after route 2b9 seg 9: HI=3.0 made the
  # tight-turn entry over-aggressive (τ ramped to 4.9 Nm in 600 ms, plant
  # overshot, ISO cancel could not undo standing torque → disengagement).
  # 1.5..2.5 keeps near-straight gentleness vs. seg-14 ringing while never
  # exceeding the proven 2b8-baseline authority on tight turns.
  # Default 2.0 (previous constant) recovered at |κ_des| ≈ 0.0055 (mild curve).
  # STEP_PER_FRAME is computed per decision from speed-dependent SPREAD_FRAMES:
  #   step = T_CAP_BASE_NM / STEER_MAX / spread_frames(v)
  # At v=30 kph (spread=10): 0.0104 frac/frame = 0.125 Nm/frame — exceeds the
  # wire STEER_DELTA_UP limit (0.1 Nm/frame), so wire clamps to ~0.1 Nm/frame.
  # At v=120 kph (spread=50): 0.00208 frac/frame = 0.025 Nm/frame — well under
  # wire limit, internal-paced gentle drain.

  # Feedback deadzone: engage only when δ_err would cause ≥ drift_tol_m
  # lateral drift within DRIFT_EVAL_HORIZON_S (= model's lat_action_t).
  #   drift(T) = ½ · δ_err / L · v² · T²  ⇒  δ_tol = 2 · drift_m · L / (v·T)²
  # drift_m DECREASES with speed: at low v allow larger drift (comfort,
  # corrections take longer to be felt); at high v tighten drift (precision,
  # any error matters more because v² scales the felt impact).
  # Combined with the 1/v² factor in tolerance, both effects compound:
  # tolerance shrinks faster with v than the +slope variant did.
  DRIFT_LOW_V_M  = 0.040  # m at v ≤ 30 kph (8.33 m/s) — permissive
  DRIFT_HIGH_V_M = 0.020  # m at v ≥ 120 kph (33.33 m/s) — tighter
  DRIFT_EVAL_HORIZON_S = 0.5

  # Breakaway torque fraction (rack stiction floor). Sub-friction commands
  # don't move the hydraulic rack, so the controller pushes target to ±friction
  # to break stiction. Initial estimate from memory; tune if needed via a
  # dedicated stop-and-ramp experiment (not online — see shadow-plant notes).
  FRICTION = 0.1

  # ISO 11270 comfort guard. Half-ISO targets:
  #   ISO_LATERAL_ACCEL = 3.0 m/s²    →  BMW_LATERAL_ACCEL = 1.5
  #   ISO_LATERAL_JERK  = 5.0 m/s³    →  BMW_LATERAL_JERK  = 2.5
  # Cancel step_remaining (and fast_ramp_remaining) when either exceeded.
  #   |a_y_meas| > BMW_LATERAL_ACCEL — current loading already at limit;
  #     don't push deeper. Uses κ_meas (measured outcome).
  #   |jerk_pred| > BMW_LATERAL_JERK — predicted jerk = v²·(κ_des−κ_meas)/τ
  #     where τ = JERK_PRED_TAU = 0.5 s matches the controller's plant
  #     settling horizon. Predictive — catches ringing setup ~100 ms
  #     before it appears in κ_meas. Validated against route 2b8 seg 14:
  #     at t=848.5s during overshoot, κ_des reversed while κ_meas still on
  #     the wrong side, jerk_pred = 4.8 m/s³ → would have cancelled the
  #     counter-torque ramp that produced the 15.7 m/s³ measured jerk.
  BMW_LATERAL_ACCEL = 1.5
  BMW_LATERAL_JERK = 2.5
  JERK_PRED_TAU = 0.5

  # Rear-axle bicycle-model wheelbase (m). Used for κ ↔ δ conversion.
  L = float(CP.wheelbase)

  _sm = messaging.SubMaster(['livePose'])

  state = {
    'torque': 0.0,             # current commanded torque fraction (ramps toward target_frac)
    'target_frac': 0.0,        # plant-inversion target set each 250 ms decision
    'step_remaining': 0.0,     # target_frac - torque, drained at CAN rate
    'tick_count': 0,           # livePose tick counter; decide every ACTION_CADENCE_TICKS
    'action': 'init',          # debug: hold_zero / brake_zero / breakaway / ramp / cancel_accel / cancel_jerk
    'delta_err': 0.0,          # debug: front-wheel-angle error (rad)
    'fast_ramp_remaining': 0,  # CAN frames left in breakaway sign-flip fast ramp
    'fast_ramp_step': 0.0,     # per-frame step during fast ramp (target_frac / 5)
    'step_per_frame': T_CAP_BASE_NM / CCP.STEER_MAX / 25,  # per-frame drain rate (set per decision)
    'lat_pub': None,
    'desired': 0.0, 'measured': 0.0,
    'slope_eff': T_CAP_SLOPE_LO,  # debug: effective gain-scheduled T_CAP_SLOPE
    'a_y_meas': 0.0,              # debug: v²·κ_meas (m/s²)
    'jerk_pred': 0.0,             # debug: v²·κ_err/τ (m/s³)
  }

  def update(active, CS, VM, params, steer_limited_by_safety, desired_curvature, curvature_limited, lat_delay):
    pid_log = log.ControlsState.LateralTorqueState.new_message()
    pid_log.version = 11

    _sm.update(0)
    lp = _sm['livePose']

    state['desired'] = float(desired_curvature)

    # livePose tick (20 Hz): update measured every tick; plant-inversion
    # decision only every ACTION_CADENCE_TICKS (250 ms) — gives plant time to
    # respond to previous correction (2.5τ → ~92% response).
    # CAN tick (100 Hz): drain step_remaining toward T_peak_frac target.
    if _sm.updated['livePose']:
      v = max(float(lp.velocityDevice.x) if _sm.seen['livePose'] else CS.vEgo, 5.0)
      state['measured'] = float(lp.angularVelocityDevice.z) / v

      # κ_des-adaptive aligning-torque slope. Gentle on near-straights (less
      # overshoot ingredients on small κ_des), full authority on tight turns.
      #   t = clip((|κ_des| − KAPPA_STRAIGHT) / (KAPPA_TIGHT − KAPPA_STRAIGHT), 0, 1)
      #   slope_eff = T_CAP_SLOPE_LO + t · (T_CAP_SLOPE_HI − T_CAP_SLOPE_LO)
      # v-independent — only the path geometry drives authority allocation.
      kappa_abs = abs(state['desired'])
      k_t = max(0.0, min(1.0, (kappa_abs - KAPPA_STRAIGHT) / (KAPPA_TIGHT - KAPPA_STRAIGHT)))
      slope_eff = T_CAP_SLOPE_LO + k_t * (T_CAP_SLOPE_HI - T_CAP_SLOPE_LO)
      state['slope_eff'] = slope_eff

      # Front-wheel-angle error (rear-axle bicycle model).
      delta_des = math.atan(state['desired'] * L)
      delta_meas = math.atan(state['measured'] * L)
      delta_err = delta_des - delta_meas
      state['delta_err'] = delta_err

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
      jerk_pred = v * v * (state['desired'] - state['measured']) / JERK_PRED_TAU
      state['a_y_meas'] = a_y_meas
      state['jerk_pred'] = jerk_pred
      overshooting = (state['desired'] - state['measured']) * state['measured'] < 0
      cancel_reason = None
      if overshooting:
        if abs(a_y_meas) > BMW_LATERAL_ACCEL:
          cancel_reason = 'cancel_accel'
        elif abs(jerk_pred) > BMW_LATERAL_JERK:
          cancel_reason = 'cancel_jerk'
      if cancel_reason:
        state['fast_ramp_remaining'] = 0
        # overshooting=True implies κ_meas != 0; unwind toward opposite sign.
        unwind_target = -FRICTION if state['measured'] > 0 else FRICTION
        state['target_frac'] = unwind_target
        state['step_remaining'] = unwind_target - state['torque']
        state['action'] = cancel_reason

      state['tick_count'] += 1

      if state['tick_count'] >= ACTION_CADENCE_TICKS:
        state['tick_count'] = 0

        # Speed-dependent ramp window: linearly interpolate spread frames
        # between low-speed (agile, spread=10) and high-speed (gentle, spread=50).
        spread_t = max(0.0, min(1.0, (v - V_FOR_SPREAD_MIN) / (V_FOR_SPREAD_MAX - V_FOR_SPREAD_MIN)))
        spread_frames = SPREAD_FRAMES_MIN + spread_t * (SPREAD_FRAMES_MAX - SPREAD_FRAMES_MIN)
        state['step_per_frame'] = T_CAP_BASE_NM / CCP.STEER_MAX / spread_frames

        # Speed-adaptive tolerance: 0.025 m lateral drift over 0.5 s horizon.
        # δ_tol = 2·M·L / (v·T)²  — scales 1/v², matches natural correction authority.
        lookahead_m = v * DRIFT_EVAL_HORIZON_S
        # drift_m linearly interpolated between low-v (permissive) and high-v (tighter)
        drift_t = max(0.0, min(1.0, (v - V_FOR_SPREAD_MIN) / (V_FOR_SPREAD_MAX - V_FOR_SPREAD_MIN)))
        drift_m = DRIFT_LOW_V_M + drift_t * (DRIFT_HIGH_V_M - DRIFT_LOW_V_M)
        tolerance = 2.0 * drift_m * L / (lookahead_m ** 2)

        # Plant-inversion target torque in angle domain — the steady-state
        # aligning torque required to hold δ_err. Soft deadband: deduct
        # tolerance from δ_err so commanded torque starts at 0 (not at
        # SLOPE·v²·tolerance) when crossing the boundary — smooth transition,
        # no pulse on tolerance crossing.
        #   effective_err = sign(δ_err) · (|δ_err| − tolerance)
        #   τ_Nm = slope_eff · v² · effective_err
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
            target_frac = -FRICTION if delta_err > 0 else FRICTION
            state['action'] = 'brake_zero'
          else:
            target_frac = 0.0
            state['action'] = 'hold_zero'
        else:
          effective_err = delta_err - tolerance * (1.0 if delta_err > 0 else -1.0)
          target_nm = slope_eff * v * v * effective_err
          target_frac = target_nm / CCP.STEER_MAX
          if abs(target_frac) < FRICTION:
            target_frac = FRICTION * (1.0 if delta_err > 0 else -1.0)
            state['action'] = 'breakaway'
          else:
            state['action'] = 'ramp'
          # v²·|δ|-scaled cap, clipped at STEER_MAX (panda hard limit).
          # Cap also uses the gain-scheduled slope so authority grows with
          # commanded a_y_des — straights stay near BASE, tight turns can
          # reach STEER_MAX (transient over-envelope; speedlimitd bleeds v).
          t_cap_nm = min(CCP.STEER_MAX,
                         T_CAP_BASE_NM + slope_eff * v * v * abs(delta_des))
          t_cap_frac = t_cap_nm / CCP.STEER_MAX
          target_frac = max(-t_cap_frac, min(t_cap_frac, target_frac))

        state['target_frac'] = target_frac

        # Breakaway sign-flip fast ramp: the normal drain would crawl from
        # ±friction through zero to ∓friction at the per-frame step rate,
        # sitting in the stiction zone for many frames and buzzing the
        # actuator. Reset torque to 0 and ramp to the new target over 5
        # frames (50 ms) regardless of speed-dependent SPREAD_FRAMES — the
        # 5-frame fast ramp is a dedicated stiction-crossing mechanism.
        if state['action'] == 'breakaway' and state['torque'] * target_frac < 0.0:
          state['torque'] = 0.0
          state['fast_ramp_remaining'] = 5
          state['fast_ramp_step'] = target_frac / 5.0
          state['step_remaining'] = 0.0
        else:
          state['step_remaining'] = target_frac - state['torque']

    # Apply CAN-rate step: fraction of remaining per 100 Hz tick.
    # Fast ramp (breakaway sign flip) takes priority for its first 5 frames.
    if state['fast_ramp_remaining'] > 0:
      state['torque'] = max(-1.0, min(1.0, state['torque'] + state['fast_ramp_step']))
      state['fast_ramp_remaining'] -= 1
    elif abs(state['step_remaining']) > 1e-9:
      step_per_frame = state['step_per_frame']
      step_this_tick = max(-step_per_frame, min(step_per_frame, state['step_remaining']))
      state['torque'] = max(-1.0, min(1.0, state['torque'] + step_this_tick))
      state['step_remaining'] -= step_this_tick

    err = state['desired'] - state['measured']  # for logging only
    output = 0.0 if not active else max(-1.0, min(1.0, state['torque']))

    pid_log.actualLateralAccel = float(state['measured'])
    pid_log.desiredLateralAccel = float(state['desired'])
    pid_log.error = float(err)
    pid_log.active = active
    pid_log.output = float(output)
    pid_log.saturated = bool(abs(output) > 0.99)

    try:
      if state['lat_pub'] is None:
        from openpilot.selfdrive.plugins.plugin_bus import PluginPub
        state['lat_pub'] = PluginPub('bmw_lat_control')
      payload = {
        'desired': float(state['desired']),
        'measured': float(state['measured']),
        'err': float(err),
        'delta_err': float(state['delta_err']),
        'target_frac': float(state['target_frac']),
        'step_remaining': float(state['step_remaining']),
        'action': state['action'],
        'torque': float(state['torque']),
        'output': float(output),
        'vEgo': float(CS.vEgo),
        'active': active,
        'slope_eff': float(state['slope_eff']),
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
