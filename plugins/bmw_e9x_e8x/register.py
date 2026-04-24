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
    τ_Nm_target = T_CAP_SLOPE · v² · δ_err / PLANT_500MS_RESPONSE · scale[bin]
    If |target_frac| < FRICTION, push to ±FRICTION to break stiction.
    Clamp to ±T_CAP(v, δ):
      T_CAP_NM = min(STEER_MAX, T_CAP_BASE + T_CAP_SLOPE · v²·|δ_des|)
    Same T_CAP_SLOPE drives both target and cap — one plant characteristic
    (τ_align = T_CAP_SLOPE · v² · δ in linear tire regime, a_y ≤ 3 m/s²).
    BASE is the hydraulic rack's stiction floor. Hard stop at STEER_MAX
    (panda limit) preserves lane authority during transient over-envelope
    events before speedlimitd trims v.

  Ramp: step_remaining = T_peak − state['torque'], drained over 25 CAN frames.

  Scale factor (per-speed-bin) adapts via iterative learning from
  commanded-vs-measured δ ratio. Persistent across drives via LatAdaptive.json.
  """
  import math
  from cereal import log
  from cereal import messaging
  from bmw.values import CarControllerParams as CCP

  # Decision cadence & CAN-rate spreading.
  # ACTION_CADENCE_TICKS = 5 livePose ticks × 50 ms = 250 ms decision period.
  # SPREAD_FRAMES = 25 CAN frames × 10 ms = 250 ms ramp window; step_remaining
  # drains into state['torque'] gradually, respecting STEER_DELTA_UP rate limit.
  ACTION_CADENCE_TICKS = 5
  SPREAD_FRAMES = 25
  # T_CAP_SLOPE is the single plant/aligning-torque characteristic, in the
  # front-wheel-angle (δ) domain. Linear tire regime (a_y ≤ 3 m/s² per
  # EU/UN-R79):
  #     τ_Nm_hold = T_CAP_SLOPE · v² · δ                 (aligning torque)
  #     angle_plant_gain(v) = STEER_MAX / (T_CAP_SLOPE · v²)  (δ_ss per τ_frac)
  #
  # Used for both authority and target:
  #   T_CAP(v, δ)     = T_CAP_BASE_NM + T_CAP_SLOPE · v² · |δ_des|      (capped STEER_MAX)
  #   target_Nm       = T_CAP_SLOPE · v² · |δ_err| / PLANT_500MS_RESPONSE · scale
  #
  # BASE covers the speed- and angle-independent stiction floor.
  # SLOPE [Nm · s²/(m²·rad)] sized from route 000002a4 segs 8/18: at v=12.8
  # m/s, δ=0.0304 rad the controller needed ~2.5 Nm vs the old fixed 1.25 Nm,
  # giving SLOPE = 1.25 / (12.8² · 0.0304) ≈ 0.25. Residual plant mismatch
  # (derived 1/angle_plant_gain vs observed) is absorbed by scale_by_bin.
  T_CAP_BASE_NM = 1.25
  T_CAP_SLOPE = 0.25                                # Nm · s² / (m² · rad)
  # STEP_PER_FRAME stays sized to BASE so per-frame rate (0.00417 frac =
  # 0.05 Nm/frame) remains well under the STEER_DELTA_UP wire limit (0.1
  # Nm/frame). Larger targets under tight-corner T_CAP simply drain across
  # more 250 ms decision cycles — fine since we enter corners gradually.
  STEP_PER_FRAME = T_CAP_BASE_NM / CCP.STEER_MAX / SPREAD_FRAMES

  # Plant-inversion horizon: we command the torque that would move the front
  # wheel by δ_err within 500 ms. For a first-order plant with τ=100 ms
  # and a 250 ms ramp → 250 ms hold input, the output at t=500 ms is:
  #   y(0.25) = 4·[0.15 + 0.1·e^(−2.5)] = 0.633   (end of ramp)
  #   y(0.50) = 1 + (0.633 − 1)·e^(−2.5) = 0.970   (end of hold)
  # Factor appears in the denominator of T_peak to compensate for this lag.
  PLANT_500MS_RESPONSE = 0.970

  # Feedback deadzone: engage only when δ_err would cause ≥DRIFT_TOLERANCE_M
  # lateral drift within DRIFT_EVAL_HORIZON_S (= model's lat_action_t).
  #   drift(T) = ½ · δ_err / L · v² · T²  ⇒  δ_tol = 2 · M · L / (v·T)²
  # 0.025 m in 0.5 s = 0.05 m/s drift — below driver perception in a 3.5 m lane.
  # Upstream lane_centering uses a 0.1 m hysteresis band (0.2 m activate, 0.1 m
  # deactivate) — 4× this tolerance — so layer hand-offs are clean: activation
  # reliably triggers controller response, deactivation leaves the car inside
  # our no-action zone. Preserve the ≥ 4× ratio when tuning either layer.
  DRIFT_TOLERANCE_M = 0.025
  DRIFT_EVAL_HORIZON_S = 0.5

  # Breakaway torque fraction (rack stiction floor). Sub-friction commands
  # don't move the hydraulic rack, so the controller pushes target to ±friction
  # to break stiction. Initial estimate from memory; tune if needed via a
  # dedicated stop-and-ramp experiment (not online — see shadow-plant notes).
  FRICTION = 0.05

  # Rear-axle bicycle-model wheelbase (m). Used for κ ↔ δ conversion.
  L = float(CP.wheelbase)

  _sm = messaging.SubMaster(['livePose'])

  # Per-speed-bin responsiveness scale factor (adaptive). At each 250 ms decision,
  # compare desired vs measured δ change over window → ratio tells us how much
  # of observed motion was commanded (vs disturbance / plant mismatch).
  # Applied multiplicatively to target (scale > 1 → target bigger → more aggressive).
  # Bins: 30 km/h to 120 km/h in 5 km/h steps = 18 bins.
  MIN_VEGO_MS = 30.0 / 3.6
  BIN_WIDTH_MS = 5.0 / 3.6
  _SCALE_N = 18
  _scale_by_bin = [1.0] * _SCALE_N
  SCALE_MIN, SCALE_MAX = 0.5, 2.0
  SCALE_EMA_ALPHA = 0.2

  # Persistent adaptive state — survives reboot and plugin updates
  # (install.sh preserves data/ subdirectory). Only scale_by_bin is persisted;
  # shadow fit is diagnostic-only (see bmw/shadow_plant.py), live K/b/friction
  # stay at calibrated seeds.
  import json
  import time
  _ADAPTIVE_PATH = os.path.join(_PLUGIN_DIR, 'data', 'LatAdaptive.json')
  _SAVE_EVERY_TICKS = 1000  # ~50 seconds at 20 Hz
  _ADAPTIVE_VERSION = 2     # v2: shadow values dropped (diagnostic-only)

  try:
    with open(_ADAPTIVE_PATH) as _f:
      _saved = json.load(_f)
    if _saved.get('version') == _ADAPTIVE_VERSION:
      _saved_bins = _saved.get('scale_by_bin', [])
      if len(_saved_bins) == _SCALE_N:
        _scale_by_bin = list(_saved_bins)
  except (FileNotFoundError, json.JSONDecodeError, OSError, ValueError):
    pass  # fresh start with defaults

  def _save_adaptive():
    try:
      data_dir = os.path.dirname(_ADAPTIVE_PATH)
      os.makedirs(data_dir, exist_ok=True)
      tmp = _ADAPTIVE_PATH + '.tmp'
      with open(tmp, 'w') as f:
        json.dump({
          'version': _ADAPTIVE_VERSION,
          'scale_by_bin': list(_scale_by_bin),
          'updated_ts': time.time(),
        }, f)
      os.replace(tmp, _ADAPTIVE_PATH)  # atomic on same fs
    except OSError:
      pass

  state = {
    'torque': 0.0,             # current commanded torque fraction (ramps toward target_frac)
    'target_frac': 0.0,        # plant-inversion target set each 250 ms decision
    'step_remaining': 0.0,     # target_frac - torque, drained at CAN rate
    'prev_desired': 0.0,       # desired curvature at last decision (for adaptive scale)
    'prev_measured': 0.0,      # measured curvature at last decision (for adaptive scale)
    'tick_count': 0,           # livePose tick counter; decide every ACTION_CADENCE_TICKS
    'save_counter': 0,         # periodic persistence counter
    'action': 'init',          # debug: hold_zero / breakaway / ramp
    'last_scale': 1.0,         # debug: scale factor used in last decision
    'delta_err': 0.0,          # debug: front-wheel-angle error (rad)
    'fast_ramp_remaining': 0,  # CAN frames left in breakaway sign-flip fast ramp
    'fast_ramp_step': 0.0,     # per-frame step during fast ramp (target_frac / 5)
    'lat_pub': None,
    'desired': 0.0, 'measured': 0.0,
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

      # Front-wheel-angle error (rear-axle bicycle model).
      delta_des = math.atan(state['desired'] * L)
      delta_meas = math.atan(state['measured'] * L)
      delta_err = delta_des - delta_meas
      state['delta_err'] = delta_err

      # Periodic persistence — save adaptive state to disk every ~50s
      state['save_counter'] += 1
      if state['save_counter'] >= _SAVE_EVERY_TICKS:
        state['save_counter'] = 0
        _save_adaptive()

      state['tick_count'] += 1
      if state['tick_count'] >= ACTION_CADENCE_TICKS:
        state['tick_count'] = 0

        # Speed-adaptive tolerance: 0.025 m lateral drift over 0.5 s horizon.
        # δ_tol = 2·M·L / (v·T)²  — scales 1/v², matches natural correction authority.
        tolerance = 2.0 * DRIFT_TOLERANCE_M * L / ((v * DRIFT_EVAL_HORIZON_S) ** 2)

        # Update per-speed-bin responsiveness scale (iterative learning). Use
        # δ deltas over the 250 ms window; ratio = desired / measured tells us
        # whether the plant over- or under-responded vs our commanded change.
        _dw_des = delta_des - math.atan(state['prev_desired']  * L)
        _dw_meas = delta_meas - math.atan(state['prev_measured'] * L)
        if abs(_dw_des) > tolerance and abs(_dw_meas) > tolerance:
          ratio = abs(_dw_des) / abs(_dw_meas)
          ratio = max(SCALE_MIN, min(SCALE_MAX, ratio))
          if MIN_VEGO_MS <= v < MIN_VEGO_MS + _SCALE_N * BIN_WIDTH_MS:
            b_idx = int((v - MIN_VEGO_MS) / BIN_WIDTH_MS)
            _scale_by_bin[b_idx] = (1 - SCALE_EMA_ALPHA) * _scale_by_bin[b_idx] + SCALE_EMA_ALPHA * ratio
        state['prev_desired'] = state['desired']
        state['prev_measured'] = state['measured']

        # Look up scale factor for current speed bin (default 1.0 if outside)
        if MIN_VEGO_MS <= v < MIN_VEGO_MS + _SCALE_N * BIN_WIDTH_MS:
          b_idx = int((v - MIN_VEGO_MS) / BIN_WIDTH_MS)
          scale = _scale_by_bin[b_idx]
        else:
          scale = 1.0
        state['last_scale'] = scale

        # Plant-inversion target torque in angle domain. τ needed to move δ
        # by δ_err within 500 ms, given aligning-torque physics and first-order
        # plant lag (0.970 asymptote at +500 ms):
        #   τ_Nm_steady = T_CAP_SLOPE · v² · δ_err
        #   τ_Nm_command = τ_Nm_steady / PLANT_500MS_RESPONSE · scale
        # Inside tolerance → 0 (stiction holds; no chatter at the boundary).
        # Sub-breakaway commands won't move the rack → push to ±FRICTION.
        if abs(delta_err) <= tolerance:
          target_frac = 0.0
          state['action'] = 'hold_zero'
        else:
          target_nm = T_CAP_SLOPE * v * v * delta_err / PLANT_500MS_RESPONSE * scale
          target_frac = target_nm / CCP.STEER_MAX
          if abs(target_frac) < FRICTION:
            target_frac = FRICTION * (1.0 if delta_err > 0 else -1.0)
            state['action'] = 'breakaway'
          else:
            state['action'] = 'ramp'
          # v²·|δ|-scaled cap, clipped at STEER_MAX (panda hard limit).
          # Normal ops stay within EU 3 m/s² → T_CAP in the 1.25-3.3 Nm
          # range; transient over-envelope events allowed up to STEER_MAX
          # so the car doesn't drift while speedlimitd catches up.
          t_cap_nm = min(CCP.STEER_MAX,
                         T_CAP_BASE_NM + T_CAP_SLOPE * v * v * abs(delta_des))
          t_cap_frac = t_cap_nm / CCP.STEER_MAX
          target_frac = max(-t_cap_frac, min(t_cap_frac, target_frac))

        state['target_frac'] = target_frac

        # Breakaway sign-flip fast ramp: the normal drain would crawl from
        # ±friction through zero to ∓friction at STEP_PER_FRAME, sitting in
        # the stiction zone for ~24 frames and buzzing the actuator. Reset
        # torque to 0 and ramp to the new target over 5 frames (50 ms).
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
      step_this_tick = max(-STEP_PER_FRAME, min(STEP_PER_FRAME, state['step_remaining']))
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
        'scale': float(state['last_scale']),
        'torque': float(state['torque']),
        'output': float(output),
        'vEgo': float(CS.vEgo),
        'active': active,
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
