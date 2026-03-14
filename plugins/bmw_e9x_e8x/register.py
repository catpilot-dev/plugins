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


_torque_cache = {"btn": "INACTIVE", "desc": "Online torque learning from driving data.", "t": 0.0}

def _torque_update():
  import time
  now = time.monotonic()
  if now - _torque_cache["t"] < 10.0:
    return
  _torque_cache["t"] = now
  try:
    from openpilot.selfdrive.ui.ui_state import ui_state
    sm = ui_state.sm
    if sm.recv_frame.get('liveTorqueParameters', 0) > 0:
      lt = sm['liveTorqueParameters']
      _torque_cache["btn"] = "ACTIVE" if lt.useParams and lt.liveValid else "INACTIVE"
      _torque_cache["desc"] = f"{lt.calPerc}% | F={lt.latAccelFactorFiltered:.2f} f={lt.frictionCoefficientFiltered:.3f}"
  except Exception:
    pass

def _torque_button_text():
  _torque_update()
  return _torque_cache["btn"]

def _torque_desc():
  _torque_update()
  return _torque_cache["desc"]


_delay_cache = {"btn": "INACTIVE", "desc": "Online lateral delay estimation from steering response.", "t": 0.0}

def _delay_update():
  import time
  now = time.monotonic()
  if now - _delay_cache["t"] < 10.0:
    return
  _delay_cache["t"] = now
  try:
    from openpilot.selfdrive.ui.ui_state import ui_state
    sm = ui_state.sm
    if sm.recv_frame.get('liveDelay', 0) > 0:
      ld = sm['liveDelay']
      _delay_cache["btn"] = "ACTIVE" if str(ld.status).split('.')[-1] == 'applied' else "INACTIVE"
      _delay_cache["desc"] = f"{ld.calPerc}% | {ld.lateralDelay:.2f}s"
  except Exception:
    pass

def _delay_button_text():
  _delay_update()
  return _delay_cache["btn"]

def _delay_desc():
  _delay_update()
  return _delay_cache["desc"]


def on_vehicle_settings(items, CP):
  """Hook callback: populate Vehicle panel with BMW-specific toggles."""
  if CP.brand != 'bmw':
    return items

  from openpilot.system.ui.widgets.list_view import toggle_item, button_item

  items.append(toggle_item(
    "Cruise Speed Memory",
    "Remember cruise speed ceiling across disengage/re-engage within the same drive.",
    _read_param('CruiseCeilingMemory') != '0',
    callback=lambda state: _write_param('CruiseCeilingMemory', '1' if state else '0'),
  ))

  items.append(toggle_item(
    "Consecutive Lane Changes",
    "Press steering button during an active lane change to chain the next one immediately for fluid multi-lane merges.",
    _read_param('ConsecutiveLaneChange') != '0',
    callback=lambda state: _write_param('ConsecutiveLaneChange', '1' if state else '0'),
  ))

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

  items.append(button_item(
    "Live Torque",
    _torque_button_text,
    _torque_desc,
    enabled=False,
  ))

  items.append(button_item(
    "Lateral Delay",
    _delay_button_text,
    _delay_desc,
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
  """Handle desire gap countdown before state machine runs."""
  if not _is_consecutive_enabled():
    return result

  if _clc.desire_gap > 0:
    _clc.desire_gap -= 1
    if _clc.desire_gap == 0:
      from cereal import log
      dh.lane_change_state = log.LaneChangeState.laneChangeStarting
      dh.lane_change_ll_prob = 1.0
      dh.lane_change_timer = 0.0
      _clc.consecutive_requested = False
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
    if rising_edge and one_blinker:
      _clc.consecutive_requested = True
    # Re-trigger as soon as car is committed (ll_prob faded ~0.5s) — skip waiting for model
    if _clc.consecutive_requested and one_blinker and not below_lane_change_speed \
        and dh.lane_change_ll_prob < 0.01:
      _clc.desire_gap = 1

  elif dh.lane_change_state == log.LaneChangeState.laneChangeFinishing:
    if rising_edge and one_blinker and not below_lane_change_speed:
      _clc.desire_gap = 1

  return result


def on_desire_post_update(desire, lane_change_state, lane_change_direction, carstate):
  """Override desire to none during consecutive gap frame for model rising edge."""
  if _clc.desire_gap > 0:
    from cereal import log
    return log.Desire.none
  return desire
