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
  """Delta-error micro-stepping + friction FF — curvature tracking at 20 Hz.

  Feedback logic (per livePose tick):
    err       = desired − measured
    delta_err = err − prev_err
    ─────────────────────────────────────────────────────────
    |err| < deadzone     :  HOLD       (step = 0)
    sign changed         :  FULL STEP  (step = err / plant_gain)
    worsening same-sign  :  DELTA STEP (step = delta_err / plant_gain)
    improving same-sign  :  HOLD       (step = 0)  ← lets plant respond
    ─────────────────────────────────────────────────────────
    Step clamped to ±MAX_STEP, spread over SPREAD_FRAMES CAN frames (500 ms).

  Why hold-when-improving handles 0.5 s plant delay gracefully:
    during the delay window, err appears "stuck" at its pre-correction value.
    Delta-logic doesn't trigger more action because err isn't WORSENING — it's
    frozen. Once plant starts responding, err decreases → still holding. Torque
    settles at the right level without command backlog or windup.

  Friction FF (separate, fast, on top of micro-stepped torque):
    proportional ramp: scales from 0 → ±FRICTION_TORQUE over |err| 0 → FRICTION_ERR_SAT
    added to output directly (not spread)
  """
  from cereal import log
  from cereal import messaging
  from bmw.shadow_plant import ShadowPlantEstimator
  from bmw.values import CarControllerParams as CCP

  # Plant gain: K/v² + b, fit from route 00000266 regression
  #   desired·v² = K·(−torque)/v² + b·(−torque),  K=0.68, b=0.0073, R²=0.44
  # These are the INITIAL values. A shadow estimator runs online against the
  # same 2-param model, and once validated (R² > 0.4, stable across refits,
  # coverage across speed bins), promotes its fit to the live plant_gain.
  PLANT_GAIN_K = 0.68
  PLANT_GAIN_B = 0.0073

  # Micro-stepping cadence:
  #   Delta-error decision runs every ACTION_CADENCE_TICKS livePose ticks (= 250 ms).
  #   SPREAD_FRAMES=25 CAN frames (= 250 ms) so each step fully applies before the
  #   next decision; no overlap. Matches plant first-order time const ≈100 ms →
  #   at 2.5τ = 250 ms, plant has responded ~91.8% to previous correction.
  MAX_STEP = CCP.STEER_DELTA_UP * 5 / CCP.STEER_MAX          # 0.04167
  SPREAD_FRAMES = 25
  STEP_PER_FRAME = MAX_STEP / SPREAD_FRAMES
  ACTION_CADENCE_TICKS = 5   # 5 × 50 ms = 250 ms

  # Friction feedforward — step torque in sign(err) direction, proportionally
  # ramped to avoid bang-bang at small err. Applied on top of micro-stepped torque.
  FRICTION_TORQUE = 0.05      # ±0.6 Nm
  FRICTION_ERR_SAT = 0.0002   # |err| ≥ this saturates friction at full authority
  # Feedback deadzone from physical criterion: engage P/I/friction only when
  # the curvature error would cause ≥DRIFT_TOLERANCE_M lateral drift within
  # DRIFT_EVAL_HORIZON_S seconds (horizon aligned with desired_curvature's
  # lookahead, lat_action_t ≈ 0.5s).
  #   offset(T) = ½ · delta · v² · T²  ⇒  delta_threshold = 2·M / (v·T)²
  # 0.025m over 0.5s = 0.05 m/s drift rate — below driver perception in a 3.5m lane.
  DRIFT_TOLERANCE_M = 0.025
  DRIFT_EVAL_HORIZON_S = 0.5

  _sm = messaging.SubMaster(['livePose'])
  _shadow = ShadowPlantEstimator(PLANT_GAIN_K, PLANT_GAIN_B, FRICTION_TORQUE)

  # Torque history for shadow estimator lag compensation. measured(t) reflects
  # plant response to torque ~250 ms ago. Pair measured(t) with torque(t-250 ms)
  # for correct plant-gain fit. Buffer size matches ACTION_CADENCE_TICKS.
  from collections import deque as _deque
  _torque_history = _deque(maxlen=ACTION_CADENCE_TICKS)   # 5 livePose ticks = 250 ms

  state = {
    'torque': 0.0,             # micro-stepping accumulator (output, pre-friction)
    'step_remaining': 0.0,     # pending correction delta to apply (CAN-rate spread)
    'prev_err': 0.0,           # err at last action (for delta-err logic, 250 ms ago)
    'tick_count': 0,           # livePose tick counter; action every ACTION_CADENCE_TICKS
    'action': 'init',          # debug: last action taken (hold/full/delta)
    'lat_pub': None,
    'desired': 0.0, 'measured': 0.0,
    'plant_gain': 0.0,
  }

  def update(active, CS, VM, params, steer_limited_by_safety, desired_curvature, curvature_limited, lat_delay):
    pid_log = log.ControlsState.LateralTorqueState.new_message()
    pid_log.version = 10

    _sm.update(0)
    lp = _sm['livePose']

    state['desired'] = float(desired_curvature)

    # livePose tick (20 Hz): update measured/shadow every tick; delta-error
    # decision only every ACTION_CADENCE_TICKS (250 ms) — gives plant time to
    # respond to previous correction (2.5τ → ~92% response).
    # CAN tick (100 Hz): apply fraction of pending step toward state['torque'].
    friction_ff = 0.0
    if _sm.updated['livePose']:
      v = max(float(lp.velocityDevice.x) if _sm.seen['livePose'] else CS.vEgo, 5.0)
      state['plant_gain'] = _shadow.plant_gain(v)
      state['measured'] = float(lp.angularVelocityDevice.z) / v

      # Shadow sample: pair measured(t) with torque from 250 ms ago (ACTION_CADENCE_TICKS
      # livePose ticks earlier) so the plant's first-order response is captured correctly.
      _torque_history.append(state['torque'])
      if active and len(_torque_history) == ACTION_CADENCE_TICKS:
        lagged_torque = _torque_history[0]   # oldest, ~250 ms old
        _shadow.add_sample(v, lagged_torque, state['measured'])

      state['tick_count'] += 1
      if state['tick_count'] >= ACTION_CADENCE_TICKS:
        state['tick_count'] = 0

        err = state['desired'] - state['measured']
        deadzone = (2.0 * DRIFT_TOLERANCE_M / (DRIFT_EVAL_HORIZON_S ** 2)) / (v ** 2)
        prev_err = state['prev_err']

        if abs(err) < deadzone:
          step = 0.0
          state['action'] = 'hold_deadzone'
        else:
          # same_sign ≥ 0 allows prev_err=0 to trigger worsening on first sample
          same_sign = err * prev_err >= 0
          sign_changed = prev_err != 0 and not same_sign
          worsening = same_sign and abs(err) > abs(prev_err)

          if sign_changed:
            step = err / state['plant_gain']              # full correction
            state['action'] = 'full'
          elif worsening:
            step = (err - prev_err) / state['plant_gain'] # delta correction only
            state['action'] = 'delta'
          else:
            step = 0.0                                     # HOLD — plant responding
            state['action'] = 'hold_improving'

        state['step_remaining'] = max(-MAX_STEP, min(MAX_STEP, step))
        state['prev_err'] = err

    # Apply CAN-rate step: fraction of remaining per 100 Hz tick
    if abs(state['step_remaining']) > 1e-9:
      step_this_tick = max(-STEP_PER_FRAME, min(STEP_PER_FRAME, state['step_remaining']))
      state['torque'] = max(-1.0, min(1.0, state['torque'] + step_this_tick))
      state['step_remaining'] -= step_this_tick

    # Friction FF (proportional ramp), added on top of micro-stepped torque
    err = state['desired'] - state['measured']  # fresh each CAN tick for friction + logging
    if abs(err) > 1e-6:
      friction_scale = min(1.0, abs(err) / FRICTION_ERR_SAT)
      friction_ff = _shadow.friction() * friction_scale * (1.0 if err > 0 else -1.0)

    output = 0.0 if not active else max(-1.0, min(1.0, state['torque'] + friction_ff))

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
        'step_remaining': float(state['step_remaining']),
        'action': state['action'],
        'plant_gain': float(state['plant_gain']),
        'friction_ff': float(friction_ff),
        'torque': float(state['torque']),
        'output': float(output),
        'vEgo': float(CS.vEgo),
        'active': active,
      }
      payload.update(_shadow.debug_state())
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
