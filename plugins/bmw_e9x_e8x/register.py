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
  """Incremental P torque controller — curvature rate tracking.

  - desired:  controlsd's desired_curvature (modelV2 plan at lat_action_t ~0.5s)
  - measured: livePose yaw rate / vEgo

  Correction signal is a backward 50ms difference on both sources:
    delta_desired  = desired[t]  - desired[t-50ms]
    delta_measured = measured[t] - measured[t-50ms]
    delta_of_delta = delta_desired - delta_measured

  Taking differences within each pipeline cancels constant bias between model
  and gyro sources. Integrating delta_of_delta via micro-stepping still drives
  measured trajectory to follow desired trajectory, without sensitivity to a
  fixed offset between the two signals.
  """
  from cereal import log
  from cereal import messaging
  from bmw.values import CarControllerParams as CCP

  _sm = messaging.SubMaster(['livePose'])

  # Max torque change per measurement frame (CAN safety limit, 5 CAN frames at 20Hz)
  MAX_STEP = CCP.STEER_DELTA_UP * 5 / CCP.STEER_MAX

  # Spread correction across 5 CAN frames (50ms) — one model/livePose cycle,
  # so each correction fully applies before the next delta_of_delta arrives
  SPREAD_FRAMES = 5
  STEP_PER_FRAME = MAX_STEP / SPREAD_FRAMES

  # Plant gain: UNDERSTEER_MARGIN × (K/v² + b).
  # K, b fitted to route data as desired·v² = K·(−torque)/v² + b·(−torque) — the
  # K/v²+b form captures that actual plant_gain asymptotes to a floor at high v
  # instead of decaying to zero (R² 0.44 vs 0.36 for pure K/v²).
  # UNDERSTEER_MARGIN=1.3 inflates controller's plant_gain above fitted truth so
  # base torque deliberately undershoots; stepper closes the remainder additively.
  PLANT_GAIN_K = 0.68
  PLANT_GAIN_B = 0.0073
  UNDERSTEER_MARGIN = 1.3

  # Output torque deadzone — 1% of max (~0.12 Nm). Route analysis shows
  # straight-line raw torque P90 ≈ 0.09; 0.01 cuts only 21% of small action
  # while keeping buzz ≤0.15 Hz (15× quieter than stock)
  STEPPER_DEADZONE = 0.01

  # Friction feedforward — static torque step in the direction that closes the
  # desired-measured error. Compensates for steering-column Coulomb friction
  # that absorbs small stepper-accumulated torque and causes undershoot in
  # curves and slow drift on straights. Stock uses 0.15; starting at 0.10.
  FRICTION_TORQUE = 0.10
  FRICTION_DEADZONE = 0.0001   # curvature error below which friction is off

  state = {
    'torque': 0.0, 'step_remaining': 0.0,
    'lat_pub': None,
    'desired': 0.0, 'measured': 0.0,
    'desired_prev': 0.0, 'measured_prev': 0.0,
    'delta_desired': 0.0, 'delta_measured': 0.0,
    'plant_gain': 0.0,
  }

  def update(active, CS, VM, params, steer_limited_by_safety, desired_curvature, curvature_limited, lat_delay):
    pid_log = log.ControlsState.LateralTorqueState.new_message()
    pid_log.version = 10

    _sm.update(0)

    v = max(CS.vEgo, 5.0)
    state['plant_gain'] = UNDERSTEER_MARGIN * (PLANT_GAIN_K / (v ** 2) + PLANT_GAIN_B)
    state['desired'] = float(desired_curvature)

    if _sm.updated['livePose']:
      state['measured'] = float(_sm['livePose'].angularVelocityDevice.z) / v

    # Friction feedforward: step torque in the direction that closes error.
    # err > 0 means we want more curvature (more left) than we have → base
    # torque needs to increase (more +state['torque'] → more -car_torque →
    # more left). friction_ff is same sign as err.
    err = state['desired'] - state['measured']
    if abs(err) > FRICTION_DEADZONE:
      friction_ff = FRICTION_TORQUE if err > 0 else -FRICTION_TORQUE
    else:
      friction_ff = 0.0

    # Base torque from desired curvature (true feedforward) + friction
    # Using desired (not measured) so the base term tracks the plan instead
    # of sustaining the current state. Drift correction is handled by friction
    # FF and micro-stepping; base commits to the target.
    state['torque'] = max(-1.0, min(1.0, state['desired'] / state['plant_gain'] + friction_ff))

    if _sm.updated['livePose']:
      state['delta_desired'] = state['desired'] - state['desired_prev']
      state['delta_measured'] = state['measured'] - state['measured_prev']
      state['desired_prev'] = state['desired']
      state['measured_prev'] = state['measured']
      delta_of_delta = state['delta_desired'] - state['delta_measured']
      correction = delta_of_delta / state['plant_gain']
      state['step_remaining'] = max(-MAX_STEP, min(MAX_STEP, correction))

    if state['step_remaining'] != 0:
      small_step = max(-STEP_PER_FRAME, min(STEP_PER_FRAME, state['step_remaining']))
      state['torque'] += small_step
      state['step_remaining'] -= small_step

    output = max(-1.0, min(1.0, state['torque']))

    # Output 0 when disengaged or torque below perception threshold
    if not active or abs(output) < STEPPER_DEADZONE:
      output = 0.0

    pid_log.actualLateralAccel = float(state['measured'])
    pid_log.desiredLateralAccel = float(state['desired'])
    pid_log.error = float(state['desired'] - state['measured'])
    pid_log.active = active
    pid_log.output = float(output)
    pid_log.saturated = bool(abs(output) > 0.99)

    # Log to plugin bus (all debug fields for offline analysis)
    try:
      if state['lat_pub'] is None:
        from openpilot.selfdrive.plugins.plugin_bus import PluginPub
        state['lat_pub'] = PluginPub('bmw_lat_control')
      state['lat_pub'].send({
        'desired': float(state['desired']),
        'measured': float(state['measured']),
        'delta_desired': float(state['delta_desired']),
        'delta_measured': float(state['delta_measured']),
        'delta_of_delta': float(state['delta_desired'] - state['delta_measured']),
        'plant_gain': float(state['plant_gain']),
        'step': float(state['step_remaining']),
        'friction_ff': float(friction_ff),
        'torque': float(state['torque']),
        'output': float(output),
        'vEgo': float(CS.vEgo),
        'active': active,
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
