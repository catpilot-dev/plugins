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


def on_torqued_allowed_cars(allowed_cars):
  """Hook callback: add BMW to torqued's live torque learning allowlist."""
  if 'bmw' not in allowed_cars:
    allowed_cars.append('bmw')
  return allowed_cars


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


# --- Consecutive lane change state (per-process, used by desire hooks) ---

class _ConsecutiveLCState:
  prev_steering_button = False
  consecutive_requested = False
  desire_gap = 0

_clc = _ConsecutiveLCState()


def _is_consecutive_enabled():
  return _read_param('ConsecutiveLaneChange') != '0'


def on_pre_lane_change(result, dh, carstate):
  """Handle consecutive lane change before state machine runs.

  Two jobs:
  1. When desire_gap counts down to 0, reset state to laneChangeStarting
     with ll_prob=1.0 so the state machine fades normally.
  2. When consecutive_requested and ll_prob faded, set desire_gap BEFORE
     the state machine can transition to laneChangeFinishing.
  """
  if not _is_consecutive_enabled():
    return result

  from cereal import log

  # Job 1: after gap frame, reset to fresh laneChangeStarting
  if _clc.desire_gap > 0:
    _clc.desire_gap -= 1
    if _clc.desire_gap == 0:
      dh.lane_change_state = log.LaneChangeState.laneChangeStarting
      dh.lane_change_ll_prob = 1.0
      dh.lane_change_timer = 0.0
    return result

  # Job 2: intercept before state machine transitions starting→finishing
  # The state machine fades ll_prob by 2*DT_MDL per frame, then checks < 0.01.
  # We must intercept BEFORE the fade crosses 0.01, so check < 0.11 (0.01 + 2*0.05).
  # This gives us one frame of margin to set ll_prob=1.0 before the state machine
  # can transition to finishing.
  if _clc.consecutive_requested \
      and dh.lane_change_state == log.LaneChangeState.laneChangeStarting \
      and dh.lane_change_ll_prob < 0.11:
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    if one_blinker:
      _clc.desire_gap = 1
      _clc.consecutive_requested = False
      dh.lane_change_ll_prob = 1.0  # block state machine's starting→finishing transition

  return result


def on_post_lane_change(result, dh, carstate, one_blinker, below_lane_change_speed, lane_change_prob):
  """Detect consecutive lane change triggers after state machine."""
  if not _is_consecutive_enabled():
    _clc.prev_steering_button = False
    return result

  from cereal import log

  # BMW uses VoiceControl button (steeringPressed) but not gas pedal for consecutive trigger
  steering_button = carstate.steeringPressed and not carstate.gasPressed
  rising_edge = steering_button and not _clc.prev_steering_button
  _clc.prev_steering_button = steering_button

  if dh.lane_change_state in (log.LaneChangeState.off, log.LaneChangeState.preLaneChange):
    _clc.consecutive_requested = False
    _clc.desire_gap = 0

  elif dh.lane_change_state == log.LaneChangeState.laneChangeStarting:
    # Only count a press as consecutive when the lane change is already committed (ll_prob < 0.5,
    # i.e. >0.25s in). The initiating press fires at ll_prob=1.0 and must NOT be counted —
    # otherwise it immediately schedules a second lane change the user never requested.
    if rising_edge and one_blinker and dh.lane_change_ll_prob < 0.5:
      _clc.consecutive_requested = True
    # Note: the actual desire_gap trigger for ll_prob < 0.01 is in on_pre_lane_change
    # (runs BEFORE state machine) to prevent the starting→finishing race condition.

  elif dh.lane_change_state == log.LaneChangeState.laneChangeFinishing:
    # consecutive_requested may have been set during laneChangeStarting but the
    # state machine transitioned to finishing in the same frame (race condition).
    # Honor it here — no need for a fresh rising_edge.
    if (_clc.consecutive_requested or (rising_edge and not carstate.gasPressed)) \
        and one_blinker and not below_lane_change_speed:
      _clc.desire_gap = 1
      _clc.consecutive_requested = False

  return result


def on_desire_post_update(desire, lane_change_state, lane_change_direction, carstate):
  """Override desire to none during consecutive gap frame for model rising edge."""
  if _clc.desire_gap > 0:
    from cereal import log
    return log.Desire.none
  return desire


def on_lat_controller_init(result, lac, CP):
  """Replace stock speed-dependent KP with BMW hydraulic steering schedule.

  Stock KP is tuned for EPS cars with high low-speed gain. BMW's hydraulic
  power steering has strong self-centering and assist at low speed — the
  stock KP overcompensates, causing oscillation in tight curves (2.8 Hz
  at 50 kph with stock KP=2.33, reduced to 1.38 with BMW schedule).

  Highway KP is unchanged (0.8 at 120+ kph).
  """
  # Flat KP=0.85 — proven for BMW hydraulic steering in openpilot 0.10.1.
  # Speed-dependent KP is designed for EPS cars; hydraulic assist provides
  # its own speed-dependent gain, so flat KP works correctly.
  lac.pid._k_p = [[0], [0.85]]
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
