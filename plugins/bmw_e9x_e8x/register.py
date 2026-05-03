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

  Ramp: ramp_step = (T_peak − state['torque']) / SPREAD_FRAMES, applied
  per CAN frame for SPREAD_FRAMES (25) frames; panda enforces wire-rate.

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

  # Decision cadence & CAN-rate spreading.
  # ACTION_CADENCE_TICKS = 5 livePose ticks × 50 ms = 250 ms decision period.
  # SPREAD_FRAMES = 25 → 250 ms ramp matches the cadence. Each ramp completes
  # before the next decision lands; no overlapping ramps. Cadence sets
  # target_frac and ramp_step = (target − torque) / SPREAD_FRAMES; CAN ticks
  # apply ramp_step until SPREAD_FRAMES have fired.
  #
  # No internal rate cap — panda enforces wire-rate (STEER_DELTA_UP =
  # 0.1 Nm/frame). For typical deltas (≤ 2.5 Nm), ramp_step ≤ 0.1 Nm/frame
  # and demand tracks rack reality. For large transients, panda clips and
  # state['torque'] briefly leads the rack — accepted; cancel logic still
  # produces correct intent.
  ACTION_CADENCE_TICKS = 5
  SPREAD_FRAMES = 25                       # 250 ms ramp (matches cadence)
  # T_CAP_SLOPE: aligning-torque gain (κ-independent). Linear tire regime:
  #     τ_Nm_hold = T_CAP_SLOPE · v² · δ                (aligning torque)
  # Drives both authority cap and target torque:
  #   T_CAP(v, δ)  = T_CAP_BASE_NM + T_CAP_SLOPE · v² · |δ_des|   (≤ STEER_MAX)
  #   target_Nm    = T_CAP_SLOPE · v² · effective_err
  # BASE covers the speed- and angle-independent stiction floor.
  # T_CAP_SLOPE_BASE = 1.0: gentle baseline gain on straights. A curvature-
  # dependent scale T_CAP_SCALE(|κ_des|) bumps it up to 2.5× on tight curves
  # (linear interp 0.001..0.01 1/m). Rationale: small κ_des needs gentle gain
  # to avoid ringing on near-straight sections (seg-14 evidence); tight κ_des
  # needs enough authority to chase the planner without lag (seg-6 evidence).
  # The soft-deadband, FRICTION breakaway, and per-tick tolerance-cancel
  # handle the boundary smoothness.
  T_CAP_BASE_NM = 2.0
  T_CAP_SLOPE_BASE = 1.0
  T_CAP_SCALE_KAPPA = [0.001, 0.01]        # |κ_des| breakpoints (1/m)
  T_CAP_SCALE_BP    = [1.0, 2.5]           # scale factor on T_CAP_SLOPE_BASE
  # Model action horizon — the time over which the model expects desired
  # curvature to be achieved (= lat_action_t). Used both for the feedback
  # deadzone (drift integration window) and for predicted jerk (ISO guard).
  MODEL_ACTION_T = 0.5

  # Feedback deadzone: engage only when δ_err would cause ≥ DRIFT_M
  # lateral drift within MODEL_ACTION_T.
  #   drift(T) = ½ · δ_err / L · v² · T²  ⇒  δ_tol = 2 · DRIFT_M · L / (v·T)²
  # The 1/v² factor in tolerance already gives natural speed adaptation
  # (tighter at high v); the prior speed-adaptive drift_m interpolation
  # added a second-order tweak that wasn't measurably useful.
  DRIFT_M = 0.05           # m of allowed drift over MODEL_ACTION_T

  # Breakaway torque fraction (rack stiction floor). Sub-friction commands
  # don't move the hydraulic rack, so the controller pushes target to ±friction
  # to break stiction. Initial estimate from memory; tune if needed via a
  # dedicated stop-and-ramp experiment (not online — see shadow-plant notes).
  FRICTION = 0.05

  # ISO 11270 comfort guard. Half-ISO targets:
  #   ISO_LATERAL_ACCEL = 3.0 m/s²    →  BMW_LATERAL_ACCEL = 1.5
  #   ISO_LATERAL_JERK  = 5.0 m/s³    →  BMW_LATERAL_JERK  = 2.5
  # Cancel the ramp when either exceeded, redirect toward FRICTION-level
  #   |a_y_meas| > BMW_LATERAL_ACCEL — current loading already at limit;
  #     don't push deeper. Uses κ_meas (measured outcome).
  #   |jerk_pred| > BMW_LATERAL_JERK — predicted jerk = v²·(κ_des−κ_meas)/T
  #     using MODEL_ACTION_T as the prediction horizon (matches the
  #     controller's plant settling). Predictive — catches ringing setup
  #     ~100 ms before it appears in κ_meas. Validated against route 2b8
  #     seg 14: at t=848.5s during overshoot, κ_des reversed while κ_meas
  #     still on the wrong side, jerk_pred = 4.8 m/s³ → would have
  #     cancelled the counter-torque ramp that produced the 15.7 m/s³
  #     measured jerk.
  BMW_LATERAL_ACCEL = ISO_LATERAL_ACCEL / 2
  BMW_LATERAL_JERK = ISO_LATERAL_JERK / 2

  # Rear-axle bicycle-model wheelbase (m). Used for κ ↔ δ conversion.
  L = float(CP.wheelbase)

  _sm = messaging.SubMaster(['livePose'])

  state = {
    'torque': 0.0,             # current commanded torque fraction (advances by ramp_step each CAN tick)
    'target_frac': 0.0,        # plant-inversion target set each 250 ms decision
    'ramp_step': 0.0,          # per-frame torque increment = (target − torque) / SPREAD_FRAMES
    'ramp_frames': 0,          # CAN frames left in current ramp
    'tick_count': ACTION_CADENCE_TICKS,  # primed so first livePose tick fires cadence immediately (no 250 ms engagement gap)
    'action': 'init',          # debug: hold_zero / brake_zero / breakaway / ramp / cancel_accel / cancel_jerk
    'delta_err': 0.0,          # debug: front-wheel-angle error (rad)
    'lat_pub': None,
    'desired': 0.0, 'measured': 0.0,
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
    # inversion decision only every ACTION_CADENCE_TICKS (250 ms) — gives
    # plant time to respond to previous correction (2.5τ → ~92% response).
    # CAN tick (100 Hz): apply ramp_step toward target_frac.
    if livepose_updated:
      state['desired'] = float(desired_curvature)
      # 8.5 m/s = ~30 kph, BMW DCC minimum engagement speed. Below this the
      # controller is never active, so the floor only protects κ_meas from
      # div-by-near-zero during disengaged crawl.
      v = max(float(lp.velocityDevice.x) if _sm.seen['livePose'] else CS.vEgo, 8.5)
      state['measured'] = float(lp.angularVelocityDevice.z) / v

      # Front-wheel-angle error (rear-axle bicycle model).
      delta_des = math.atan(state['desired'] * L)
      delta_meas = math.atan(state['measured'] * L)
      delta_err = delta_des - delta_meas
      state['delta_err'] = delta_err

      # Speed-scaled tolerance: DRIFT_M drift over MODEL_ACTION_T, 1/v² scaling.
      # Computed every livePose tick (50 ms) so the tolerance-cancel below
      # and the cadence decision both see the same value.
      lookahead_m = v * MODEL_ACTION_T
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
      jerk_pred = v * v * (state['desired'] - state['measured']) / MODEL_ACTION_T
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
          state['ramp_step'] = (unwind_target - state['torque']) / SPREAD_FRAMES
          state['ramp_frames'] = SPREAD_FRAMES
        state['action'] = cancel_reason
        state['tick_count'] = 0
      elif abs(delta_err) <= tolerance and state['ramp_frames'] > 0 and abs(state['target_frac']) > FRICTION:
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
          state['ramp_step'] = (unwind_target - state['torque']) / SPREAD_FRAMES
          state['ramp_frames'] = SPREAD_FRAMES
        state['action'] = 'cancel_tol'
        state['tick_count'] = 0
      else:
        state['tick_count'] += 1

      if state['tick_count'] >= ACTION_CADENCE_TICKS:
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
          # to 2.5 on tight curves (|κ_des| ≥ 0.01). Inlined into both target
          # and cap formulas so the κ_des-dependence is visible at the math.
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
        state['ramp_step'] = (target_frac - state['torque']) / SPREAD_FRAMES
        state['ramp_frames'] = SPREAD_FRAMES

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
          'measured': float(state['measured']),
          'err': float(err),
          'delta_err': float(state['delta_err']),
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
