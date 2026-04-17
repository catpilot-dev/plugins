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
  """Incremental P torque controller — self-contained curvature tracking.

  Both desired and measured curvature computed from modelV2 orientation
  using curv_from_psis. No dependency on controlsd's desired curvature.

  - measured: curv_from_psis at t=0.01s (current vehicle state)
  - desired: curv_from_psis at t=LOOKAHEAD_T (1.0s on straights, 0.5s in curves)
  Same formula, same source — error is purely the curvature change needed.

  Runs even when disengaged (shadow torque) for seamless engage transition.
  """
  from cereal import log
  from cereal import messaging
  import numpy as np
  from bmw.values import CarControllerParams as CCP

  _lat_pub = None
  _sm = messaging.SubMaster(['modelV2'])
  _T_IDXS = [10.0 * (i / 32) ** 2 for i in range(33)]

  # Look-ahead time: 0.5s (stock timing). With hysteresis 0.001, noise is
  # already filtered — longer lookahead adds prediction error without benefit.
  # 0.5s tested with 54 ZC vs 110 ZC at 1.0s on straights (route 00000263).
  LOOKAHEAD_T = 0.5

  # Max torque change per model frame (CAN safety limit, 5 CAN frames at 20Hz)
  MAX_STEP = CCP.STEER_DELTA_UP * 5 / CCP.STEER_MAX  # 0.04167 per model frame

  # Spread correction across 20 CAN frames (0.2s / 4 model frames)
  # 4.8s to saturate (12Nm) — smooth lane changes, no abrupt jerk
  # Straight-lane buzz handled by hysteresis, not spreading
  SPREAD_FRAMES = 20
  STEP_PER_FRAME = MAX_STEP / SPREAD_FRAMES  # 0.00208 per CAN frame

  # Plant gain: 2/v² — mirrors vehicle dynamics (curvature response ∝ 1/v²).
  # At higher speed, same torque produces less curvature change because lateral
  # force scales with v². At 100 kph: 0.0026, at 50 kph: 0.0104.
  PLANT_GAIN_COEFF = 2.0  # gain = PLANT_GAIN_COEFF / v²

  # Two-layer noise suppression (model output is already clean):
  # 1. DELTA_THRESHOLD: min |delta_error| to trigger worsening — ignores noise below P99
  # 2. STEPPER_DEADZONE: min |error| to trigger sign_change — stepper ignores small errors
  # Result: 83 actions vs 364 (hysteresis) on route 263, 16x fewer than raw.
  DELTA_THRESHOLD = 0.0004   # catches real error trends early
  STEPPER_DEADZONE = 0.0008  # stepper only acts on significant errors

  # Comfort limits on desired curvature (stricter than ISO/stock)
  MAX_LATERAL_JERK = 2.5      # m/s³ (stock: 5.0)
  MAX_LATERAL_ACCEL = 1.0     # m/s² (stock: 3.0)
  DT_MDL = 0.05               # 20Hz model frame

  state = {
    'torque': 0.0, 'step_remaining': 0,
    # Debug: last model frame values
    'desired': 0.0, 'measured': 0.0, 'error': 0.0,
    'delta_error': 0.0, 'log_prev_error': 0.0,
    'plant_gain': 0.0, 'action': 'hold',
  }

  def update(active, CS, VM, params, steer_limited_by_safety, desired_curvature, curvature_limited, lat_delay):
    pid_log = log.ControlsState.LateralTorqueState.new_message()
    pid_log.version = 10

    _sm.update(0)

    # Runs every frame, even when disengaged — shadow-computes torque so
    # engage transition is seamless (no ramp-up from zero in curves).
    if _sm.updated['modelV2']:
      m = _sm['modelV2']
      try:
        yaws = list(m.orientation.z)
        yaw_rates = list(m.orientationRate.z)
        if len(yaws) >= 20 and len(yaw_rates) >= 2:
          v = max(CS.vEgo, 5.0)
          psi_rate = yaw_rates[0]

          # Measured curvature at t≈0.01s
          action_t = 0.01
          state['measured'] = 2.0 * yaws[1] / (v * action_t) - psi_rate / v

          # Desired curvature at lookahead time
          psi_la = float(np.interp(LOOKAHEAD_T, _T_IDXS, yaws))
          raw_desired = 2.0 * psi_la / (v * LOOKAHEAD_T) - psi_rate / v

          # Comfort jerk + accel limits on desired curvature
          max_curv_rate = MAX_LATERAL_JERK / (v ** 2)
          clipped = float(np.clip(raw_desired,
                                  state['desired'] - max_curv_rate * DT_MDL,
                                  state['desired'] + max_curv_rate * DT_MDL))
          max_curv = MAX_LATERAL_ACCEL / (v ** 2)
          state['desired'] = float(np.clip(clipped, -max_curv, max_curv))
      except (AttributeError, IndexError):
        pass

      state['log_prev_error'] = state['error']
      state['error'] = state['desired'] - state['measured']

      prev_error = state['log_prev_error']
      same_sign = state['error'] * prev_error > 0
      state['delta_error'] = state['error'] - prev_error
      error_worsening = same_sign and abs(state['delta_error']) > DELTA_THRESHOLD and abs(state['error']) > abs(prev_error)
      error_sign_changed = prev_error != 0 and not same_sign and abs(state['error']) > STEPPER_DEADZONE

      plant_gain = PLANT_GAIN_COEFF / (v ** 2)
      state['plant_gain'] = plant_gain

      if error_worsening:
        correction = state['delta_error'] / plant_gain
        state['step_remaining'] = max(-MAX_STEP, min(MAX_STEP, correction))
        state['action'] = 'worsening'
      elif error_sign_changed:
        correction = state['error'] / plant_gain
        state['step_remaining'] = max(-MAX_STEP, min(MAX_STEP, correction))
        state['action'] = 'sign_change'
      else:
        state['step_remaining'] = 0
        state['action'] = 'hold'

    # Every control frame (100Hz): apply micro-step to shadow torque
    if state['step_remaining'] != 0:
      small_step = max(-STEP_PER_FRAME, min(STEP_PER_FRAME, state['step_remaining']))
      state['torque'] += small_step
      state['step_remaining'] -= small_step

    output = max(-1.0, min(1.0, state['torque']))

    if not active or CS.vEgo < 5.0:
      pid_log.active = False
      pid_log.error = 0.0
      return 0.0, 0.0, pid_log

    pid_log.actualLateralAccel = float(state['measured'])
    pid_log.desiredLateralAccel = float(state['desired'])
    pid_log.error = float(state['error'])
    pid_log.active = True
    pid_log.output = float(output)
    pid_log.saturated = bool(abs(output) > 0.99)

    # Log to plugin bus (all debug fields for offline analysis)
    nonlocal _lat_pub
    try:
      if _lat_pub is None:
        from openpilot.selfdrive.plugins.plugin_bus import PluginPub
        _lat_pub = PluginPub('bmw_lat_control')
      _lat_pub.send({
        'desired': float(state['desired']),
        'measured': float(state['measured']),
        'error': float(state['error']),
        'prev_error': float(state['log_prev_error']),
        'delta_error': float(state['delta_error']),
        'plant_gain': float(state['plant_gain']),
        'step': float(state['step_remaining']),
        'torque': float(state['torque']),
        'output': float(output),
        'vEgo': float(CS.vEgo),
        'action': state['action'],
      })
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
