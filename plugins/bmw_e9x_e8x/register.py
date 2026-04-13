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


def on_state_subscriptions(services):
  """Hook callback: add liveTorqueParameters and liveDelay to UI SubMaster."""
  for svc in ('liveTorqueParameters', 'liveDelay'):
    if svc not in services:
      services.append(svc)
  return services


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


_torque_cache = {"val": "Not calibrated", "t": 0.0}

def _torque_value():
  import time
  now = time.monotonic()
  if now - _torque_cache["t"] < 10.0:
    return _torque_cache["val"]
  _torque_cache["t"] = now
  try:
    lt = None
    from openpilot.selfdrive.ui.ui_state import ui_state
    sm = ui_state.sm
    if sm.recv_frame.get('liveTorqueParameters', 0) > 0:
      lt = sm['liveTorqueParameters']
    else:
      from openpilot.common.params import Params
      from cereal import log
      data = Params().get('LiveTorqueParameters')
      if data:
        with log.Event.from_bytes(data) as evt:
          lt = evt.liveTorqueParameters
    if lt:
      status = "Estimated" if lt.useParams and lt.liveValid else f"Estimating {lt.calPerc}%"
      _torque_cache["val"] = f"{status} | F={lt.latAccelFactorFiltered:.2f} f={lt.frictionCoefficientFiltered:.3f}"
  except Exception:
    pass
  return _torque_cache["val"]


_delay_cache = {"val": "Not calibrated", "t": 0.0}

def _delay_value():
  import time
  now = time.monotonic()
  if now - _delay_cache["t"] < 10.0:
    return _delay_cache["val"]
  _delay_cache["t"] = now
  try:
    ld = None
    from openpilot.selfdrive.ui.ui_state import ui_state
    sm = ui_state.sm
    if sm.recv_frame.get('liveDelay', 0) > 0:
      ld = sm['liveDelay']
    else:
      from openpilot.common.params import Params
      from cereal import log
      data = Params().get('LiveDelay')
      if data:
        with log.Event.from_bytes(data) as evt:
          ld = evt.liveDelay
    if ld:
      s = str(ld.status).split('.')[-1]
      if s == 'estimated':
        status = "Estimated"
      elif s == 'invalid':
        status = "Invalid"
      else:
        status = f"Estimating {ld.calPerc}%"
      _delay_cache["val"] = f"{status} | {ld.lateralDelay:.2f}s"
  except Exception:
    pass
  return _delay_cache["val"]


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
  """Incremental torque controller with curvature-dependent gain + online learning.

  Two components:
  1. Steady-state feedforward: torque = desired_curvature × gain(|curvature|)
     Prior gain table from route data. Handles sustained curves.
  2. Incremental correction: frame-to-frame delta adjusts torque for transitions.

  Online learner: when delta ≈ 0 (model satisfied), record the actual
  curvature→torque relationship and update the gain table.
  """
  from cereal import log
  import numpy as np
  from bmw.values import CarControllerParams as CCP

  _lat_pub = None

  # Prior gain table from route data analysis (|curvature| → |normalized torque gain|)
  # Sign is handled separately — gain is always positive here.
  # At small curvature, more gain needed (overcoming friction).
  # At large curvature, less gain (hydraulic assist kicks in).
  GAIN_CURV = [0.0002, 0.001, 0.003, 0.01]
  GAIN_VAL  = [100.0,  55.0,  35.0,  20.0]

  # Incremental parameters
  MAX_STEP = CCP.STEER_DELTA_UP * 5 / CCP.STEER_MAX  # 0.5 Nm per model frame
  MAX_DELTA = 0.0008
  DEADZONE_PCT = 0.05
  DEADZONE_MIN = 0.0001

  # Online learning: exponential moving average of observed gain
  LEARN_RATE = 0.02  # ~10s to converge during steady curves
  GAIN_FILE = os.path.join(_PLUGIN_DIR, 'data', 'learned_gain.json')

  def _load_learned_gain():
    try:
      import json
      with open(GAIN_FILE) as f:
        vals = json.load(f)
      if len(vals) == len(GAIN_VAL):
        return vals
    except (FileNotFoundError, OSError, ValueError):
      pass
    return list(GAIN_VAL)

  def _save_learned_gain(gains):
    try:
      import json
      os.makedirs(os.path.dirname(GAIN_FILE), exist_ok=True)
      with open(GAIN_FILE, 'w') as f:
        json.dump(gains, f)
    except (OSError, TypeError):
      pass

  learned_gain = _load_learned_gain()
  _save_counter = [0]  # save every ~10s (1000 frames at 100Hz)

  state = {'torque': 0.0, 'prev_desired': 0.0, 'correction': 0.0}

  def _get_gain(curv_abs):
    """Interpolate gain from the (possibly learned) table."""
    return float(np.interp(curv_abs, GAIN_CURV, learned_gain))

  def update_incremental(active, CS, VM, params, steer_limited_by_safety, desired_curvature, curvature_limited, lat_delay):
    pid_log = log.ControlsState.LateralTorqueState.new_message()
    pid_log.version = 1

    prev = state['prev_desired']
    delta = desired_curvature - prev
    state['prev_desired'] = desired_curvature

    if not active or CS.vEgo < 5.0:
      state['torque'] = 0.0
      state['correction'] = 0.0
      state['prev_desired'] = 0.0
      pid_log.active = False
      pid_log.error = 0.0
      return 0.0, 0.0, pid_log

    # --- Steady-state feedforward ---
    # desired_curvature is left-positive, internal torque is right-positive
    curv_abs = abs(desired_curvature)
    gain = _get_gain(curv_abs)
    ff = -desired_curvature * gain  # negate: left-pos curvature → right-pos torque

    # --- Incremental correction for transitions ---
    # delta is left-positive, correction is right-positive → negate
    deadzone = max(curv_abs * DEADZONE_PCT, DEADZONE_MIN)
    if abs(delta) > deadzone:
      scale = min(abs(delta) / MAX_DELTA, 1.0)
      step = scale * MAX_STEP
      state['correction'] += -step if delta > 0 else step  # negate: left-pos delta → right-pos correction

    # Decay correction toward zero — ff handles steady state
    state['correction'] *= 0.995

    # Combined output
    state['torque'] = ff + state['correction']
    output = max(-1.0, min(1.0, state['torque']))

    # --- Online learning ---
    # When model demand is stable (delta ≈ 0) and curvature is significant,
    # the current output torque is the right steady-state for this curvature.
    # Update the gain table toward the observed gain.
    if abs(delta) < deadzone and curv_abs > 0.0005 and abs(output) > 0.01:
      observed_gain = abs(output) / curv_abs
      current_gain = _get_gain(curv_abs)
      # Find nearest table entry and nudge it
      idx = int(np.searchsorted(GAIN_CURV, curv_abs))
      idx = max(0, min(idx, len(learned_gain) - 1))
      learned_gain[idx] += LEARN_RATE * (observed_gain - learned_gain[idx])

    # Persist learned gains periodically (~10s)
    _save_counter[0] += 1
    if _save_counter[0] >= 1000:
      _save_learned_gain(learned_gain)
      _save_counter[0] = 0

    pid_log.actualLateralAccel = float(curv_abs)
    pid_log.desiredLateralAccel = float(desired_curvature * CS.vEgo**2)
    pid_log.error = float(delta)
    pid_log.active = True
    pid_log.output = float(-output)
    pid_log.p = float(ff)                      # feedforward
    pid_log.i = float(state['correction'])      # incremental correction
    pid_log.f = float(gain)                     # current gain
    pid_log.saturated = bool(abs(output) > 0.99)

    # Log to plugin bus
    nonlocal _lat_pub
    try:
      if _lat_pub is None:
        from openpilot.selfdrive.plugins.plugin_bus import PluginPub
        _lat_pub = PluginPub('bmw_lat_control')
      _lat_pub.send({
        'desired': float(desired_curvature),
        'ff': float(ff),
        'correction': float(state['correction']),
        'output': float(output),
        'gain': float(gain),
        'learnedGain': list(learned_gain),
      })
    except Exception:
      pass

    # Internal convention: right-positive (same as stock latcontrol_torque)
    # actuators.torque convention: left-positive → negate at return
    return -output, 0.0, pid_log

  lac.update = update_incremental
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
