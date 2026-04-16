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
  """Incremental P torque controller.

  Computes correction once per model frame (20Hz) using plant gain to
  convert curvature error directly to torque, then spreads across 5
  control frames (100Hz) for smooth CAN output.

  correction_torque = curvature_error / PLANT_GAIN
  PLANT_GAIN = 0.006 (median from route data, fixed — rate limiter dominates)

  - desired: from controlsd (curv_from_psis at t+0.5s, safety-limited)
  - measured: curv_from_psis at t+0.01s (same formula, same model source)
  Same formula, same domain — error is purely the curvature change needed.
  """
  from cereal import log
  from cereal import messaging
  from bmw.values import CarControllerParams as CCP

  _lat_pub = None
  _sm = messaging.SubMaster(['modelV2'])

  # Max torque change per model frame (CAN safety limit, 5 CAN frames at 20Hz)
  MAX_STEP = CCP.STEER_DELTA_UP * 5 / CCP.STEER_MAX  # 0.04167 per model frame

  # Spread correction across 20 CAN frames (0.2s / 4 model frames)
  # 4.8s to saturate (12Nm) — smooth lane changes, no abrupt jerk
  # Straight-lane buzz handled by hysteresis, not spreading
  SPREAD_FRAMES = 20
  STEP_PER_FRAME = MAX_STEP / SPREAD_FRAMES  # 0.00208 per CAN frame

  # Plant gain: curvature change per unit normalized torque
  # Median from route data across all speeds: 0.006
  # Speed-dependent gain (2.0/v²) not needed — correction is rate-limited
  # by STEP_PER_FRAME anyway, and delta-error prevents accumulation.
  PLANT_GAIN = 0.006

  # Hysteresis on raw error: suppresses zero crossings (stepper direction reversals)
  # from model noise without muting actions like a deadzone does.
  # Gap 0.001: 63% fewer ZC than 0.0004 on straights, MAE +25% but less busy steering.
  HYST_GAP = 0.001
  from opendbc.car import apply_hysteresis

  state = {
    'torque': 0.0, 'step_remaining': 0, 'prev_error': 0.0,
    'error_steady': 0.0,  # hysteresis-filtered error
    # Debug: last model frame values
    'measured': 0.0, 'error': 0.0, 'delta_error': 0.0, 'log_prev_error': 0.0,
    'plant_gain': 0.0, 'action': 'hold',
  }

  def update(active, CS, VM, params, steer_limited_by_safety, desired_curvature, curvature_limited, lat_delay):
    pid_log = log.ControlsState.LateralTorqueState.new_message()
    pid_log.version = 10

    _sm.update(0)

    if not active or CS.vEgo < 5.0:
      state['torque'] = 0.0
      state['step_remaining'] = 0
      state['prev_error'] = 0.0
      state['error_steady'] = 0.0
      pid_log.active = False
      pid_log.error = 0.0
      return 0.0, 0.0, pid_log

    # On modelV2 update (20Hz): compute measured curvature using same formula as desired
    # desired_curvature uses curv_from_psis(psi_target, psi_rate, v, action_t) at t+0.5s
    # measured uses the same formula at T_IDXS[1] ≈ t+0.01s (current state)
    if _sm.updated['modelV2']:
      m = _sm['modelV2']
      try:
        yaws = list(m.orientation.z)
        yaw_rates = list(m.orientationRate.z)
        if len(yaws) > 1 and len(yaw_rates) > 0:
          v = max(CS.vEgo, 5.0)
          # T_IDXS[1] = 0.01s
          psi_target = yaws[1]
          psi_rate = yaw_rates[0]
          action_t = 0.01
          state['measured'] = 2.0 * psi_target / (v * action_t) - psi_rate / v
      except (AttributeError, IndexError):
        pass

      raw_error = desired_curvature - state['measured']
      state['error_steady'] = apply_hysteresis(raw_error, state['error_steady'], HYST_GAP)
      state['error'] = state['error_steady']

      prev_error = state['prev_error']
      state['log_prev_error'] = prev_error
      state['prev_error'] = state['error']

      same_sign = state['error'] * prev_error > 0
      state['delta_error'] = state['error'] - prev_error
      error_worsening = same_sign and abs(state['delta_error']) > 1e-6 and abs(state['error']) > abs(prev_error)
      error_sign_changed = prev_error != 0 and not same_sign and abs(state['error']) > 1e-6

      state['plant_gain'] = PLANT_GAIN

      if error_worsening:
        correction = state['delta_error'] / PLANT_GAIN
        state['step_remaining'] = max(-MAX_STEP, min(MAX_STEP, correction))
        state['action'] = 'worsening'
      elif error_sign_changed:
        correction = state['error'] / PLANT_GAIN
        state['step_remaining'] = max(-MAX_STEP, min(MAX_STEP, correction))
        state['action'] = 'sign_change'
      else:
        state['step_remaining'] = 0
        state['action'] = 'hold'

    # Every control frame (100Hz): apply 1/5 of the correction
    if state['step_remaining'] != 0:
      small_step = max(-STEP_PER_FRAME, min(STEP_PER_FRAME, state['step_remaining']))
      state['torque'] += small_step
      state['step_remaining'] -= small_step

    output = max(-1.0, min(1.0, state['torque']))

    pid_log.actualLateralAccel = float(state['measured'])
    pid_log.desiredLateralAccel = float(desired_curvature)
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
        'desired': float(desired_curvature),
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
