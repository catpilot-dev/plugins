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
  """PI controller with feedforward + friction — curvature tracking at 100 Hz.

    curvature_cmd = measured + Kp·(desired − measured) + Ki·integral
    torque       = curvature_cmd / plant_gain + friction_ff

  - desired:  controlsd's desired_curvature (modelV2 plan at lat_action_t ~0.5s, right-positive)
  - measured: livePose yaw rate / velocity (Kalman-filtered, bias-corrected, 100 Hz
              after locationd fork that drives the loop on gyroscope). Uses
              velocityDevice.x rather than noisy CS.vEgo for a self-consistent
              kinematic measurement from a single filter state.
  - Kp < 1 bakes in the understeer margin (base under-commits; I closes gap)
  - Integral accumulates err every CAN tick, anti-windup clamped
  """
  from cereal import log
  from cereal import messaging

  # Plant gain: K/v² + b, fit from route 00000266 regression
  #   desired·v² = K·(−torque)/v² + b·(−torque),  K=0.68, b=0.0073, R²=0.44
  # No multiplicative understeer margin — Kp < 1 provides it.
  PLANT_GAIN_K = 0.68
  PLANT_GAIN_B = 0.0073

  # PI gains. Kp = 0.8 commits 80% to desired (20% understeer margin).
  # Ki at 100 Hz — tuned so 200 ms of steady err = 0.001 adds ~0.004 curvature
  # contribution (≈30% torque at v=15), a modest plant-mismatch correction.
  KP = 0.8
  KI = 0.02
  I_MAX = 0.005   # curvature, caps sustained I-contribution to ~40% torque at v=15

  # Friction feedforward — step torque in sign(err) direction, breaks Coulomb
  # stiction in the steering column. Stock uses 0.15; we run 0.10 (1.2 Nm).
  FRICTION_TORQUE = 0.10
  # Feedback deadzone from physical criterion: engage P/I/friction only when
  # the curvature error would cause ≥DRIFT_TOLERANCE_M lateral drift within
  # DRIFT_EVAL_HORIZON_S seconds (horizon aligned with desired_curvature's
  # lookahead, lat_action_t ≈ 0.5s).
  #   offset(T) = ½ · delta · v² · T²  ⇒  delta_threshold = 2·M / (v·T)²
  # 0.025m over 0.5s = 0.05 m/s drift rate — below driver perception in a 3.5m lane.
  DRIFT_TOLERANCE_M = 0.025
  DRIFT_EVAL_HORIZON_S = 0.5

  _sm = messaging.SubMaster(['livePose'])

  state = {
    'torque': 0.0,
    'integral': 0.0,
    'lat_pub': None,
    'desired': 0.0, 'measured': 0.0,
    'plant_gain': 0.0,
  }

  def update(active, CS, VM, params, steer_limited_by_safety, desired_curvature, curvature_limited, lat_delay):
    pid_log = log.ControlsState.LateralTorqueState.new_message()
    pid_log.version = 10

    _sm.update(0)
    lp = _sm['livePose']

    # velocityDevice.x is the Kalman-filtered forward velocity (m/s); fall back
    # to CS.vEgo at stop or before filter initializes
    v = max(float(lp.velocityDevice.x) if _sm.seen['livePose'] else CS.vEgo, 5.0)
    state['plant_gain'] = PLANT_GAIN_K / (v ** 2) + PLANT_GAIN_B
    state['desired'] = float(desired_curvature)
    # livePose angularVelocityDevice.z is right-positive (matches desiredCurvature)
    state['measured'] = float(lp.angularVelocityDevice.z) / v

    err = state['desired'] - state['measured']

    # Deadzone from physical criterion — curvature error that would cause
    # DRIFT_TOLERANCE_M lateral drift within DRIFT_EVAL_HORIZON_S seconds.
    # Below this, the controller takes NO feedback action — pure FF on desired.
    # Above it, P + I + friction engage to close the error.
    deadzone = (2.0 * DRIFT_TOLERANCE_M / (DRIFT_EVAL_HORIZON_S ** 2)) / (v ** 2)
    active_err = err if abs(err) > deadzone else 0.0

    # Integral accumulates only when outside deadzone (freezes when tracking well)
    state['integral'] += active_err
    state['integral'] = max(-I_MAX, min(I_MAX, state['integral']))

    # Friction FF only engages outside deadzone
    friction_ff = (FRICTION_TORQUE if err > 0 else -FRICTION_TORQUE) if abs(err) > deadzone else 0.0

    # Curvature command: FF on desired + (gated) P·err + I·integral
    curvature_cmd = state['desired'] + KP * active_err + KI * state['integral']
    state['torque'] = max(-1.0, min(1.0, curvature_cmd / state['plant_gain'] + friction_ff))

    output = 0.0 if not active else state['torque']

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
      state['lat_pub'].send({
        'desired': float(state['desired']),
        'measured': float(state['measured']),
        'err': float(err),
        'integral': float(state['integral']),
        'plant_gain': float(state['plant_gain']),
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
