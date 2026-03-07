"""BMW car interface registration hook.

Injects BMW E82/E90 into opendbc's interfaces, fingerprints, and platforms
when the plugin is enabled. When disabled, BMW is not in the system.
"""
import os
import sys

# Ensure the plugin's bmw/ package is importable
_PLUGIN_DIR = os.path.dirname(os.path.abspath(__file__))
if _PLUGIN_DIR not in sys.path:
  sys.path.insert(0, _PLUGIN_DIR)


def on_register_interfaces(interfaces):
  """Hook callback: inject BMW into the car interfaces system.

  Called by car_helpers.py via hooks.run('car.register_interfaces', interfaces).
  Modifies interfaces dict in-place AND patches fingerprints/platforms globals.
  """
  from bmw.interface import CarInterface
  from bmw.values import CAR

  # Register BMW interfaces
  interfaces[CAR.BMW_E82] = CarInterface
  interfaces[CAR.BMW_E90] = CarInterface

  # Patch global fingerprints
  try:
    from opendbc.car.fingerprints import _FINGERPRINTS, FW_VERSIONS as GLOBAL_FW
    from bmw.fingerprints import FINGERPRINTS as BMW_FP, FW_VERSIONS as BMW_FW
    _FINGERPRINTS.update({str(k): v for k, v in BMW_FP.items()})
    GLOBAL_FW.update({str(k): v for k, v in BMW_FW.items()})
  except (ImportError, AttributeError):
    pass  # fingerprints module not available in all contexts

  # Patch global platforms
  try:
    from opendbc.car.values import PLATFORMS
    PLATFORMS[str(CAR.BMW_E82)] = CAR.BMW_E82
    PLATFORMS[str(CAR.BMW_E90)] = CAR.BMW_E90
  except (ImportError, AttributeError):
    pass

  return interfaces


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

  if v_cruise_helper.v_cruise_kph_last > 0:
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
