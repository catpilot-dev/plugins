import time

from openpilot.common.constants import CV

# Lead vehicle override: if lead is traveling above the speed limit,
# the OSM/inferred limit is likely wrong — skip capping until lead slows down.
LEAD_OVERRIDE_THRESHOLD = 0.10  # 10% above speed limit
LEAD_MIN_STATUS = True  # lead must be tracked (status=True)

# Gentle ramp for inferred (road-type, source==2) limits. When lane lines get
# faint the inferred limit can jitter downward; enforce the reduction as a soft
# coast (bounded deceleration) instead of a brake, and let the driver override
# it momentarily with the gas pedal. See
# docs/superpowers/specs/2026-07-09-speedlimitd-gentle-ramp-gas-override-design.md
SOURCE_ROAD_TYPE_INFERENCE = 2  # _sl_data['source'] value for inferred limits
RAMP_DECEL_MS2 = 0.5            # max deceleration the speed-limit cap may impose
DT_CLAMP_S = 0.2               # clamp per-cycle dt so a long gap can't jump the ramp

_sl_sub = None
_sl_data = None

# Ramp state (source==2 only): the currently enforced cap and last cycle time.
_eff_cap_ms = None
_last_t = None


def _get_sl_data():
  """Update _sl_data from plugin bus if available."""
  global _sl_sub, _sl_data
  import os
  _sl_socket_path = '/tmp/plugin_bus/speedLimitState'

  # Recreate sub if socket was recycled (speedlimitd restart deletes + rebinds)
  if _sl_sub is not None and not os.path.exists(_sl_socket_path):
    try:
      _sl_sub.close()
    except Exception:
      pass
    _sl_sub = None

  if _sl_sub is None and os.path.exists(_sl_socket_path):
    try:
      from openpilot.selfdrive.plugins.plugin_bus import PluginSub
      _sl_sub = PluginSub(['speedLimitState'])
    except Exception:
      return
  if _sl_sub is None:
    return
  try:
    msg = _sl_sub.drain('speedLimitState')
    if msg is not None and isinstance(msg, tuple) and len(msg) == 2:
      _, _sl_data = msg
  except Exception:
    pass


def _effective_offset_percent(speed_limit_kph):
  """Tiered offset: +15% for limits < 80 km/h, +10% for limits >= 80 km/h."""
  if speed_limit_kph < 80:
    return 15
  else:
    return 10


def _lead_overrides_limit(sm, speed_limit_kph):
  """Return True if lead vehicle speed suggests the speed limit data is wrong."""
  try:
    lead = sm['radarState'].leadOne
    if not lead.status:
      return False
    # lead.vLead is absolute speed in m/s
    lead_kph = lead.vLead * CV.MS_TO_KPH
    return lead_kph > speed_limit_kph * (1 + LEAD_OVERRIDE_THRESHOLD)
  except Exception:
    return False


def _gas_pressed(sm) -> bool:
  try:
    return bool(sm['carState'].gasPressed)
  except Exception:
    return False


def _reset_ramp():
  global _eff_cap_ms, _last_t
  _eff_cap_ms = None
  _last_t = None


def _ramp_cap(eff_cap_ms, target_ms, dt, gas, v_ego):
  """Advance the enforced speed-limit cap one cycle.

  Returns (new_eff_cap_ms, enforced_ms) where enforced_ms is None when the cap
  is suspended (driver overriding). Downward movement is limited to
  RAMP_DECEL_MS2; upward movement (limit rose) is immediate.
  """
  if eff_cap_ms is None:
    # Init to current speed so activation never caps below what we're doing.
    eff_cap_ms = max(target_ms, v_ego)

  if gas:
    # Suspend the cap and let it float up with the driver, so releasing the
    # pedal resumes the glide from the actual current speed.
    eff_cap_ms = max(eff_cap_ms, v_ego)
    return eff_cap_ms, None

  if target_ms < eff_cap_ms:
    eff_cap_ms = max(target_ms, eff_cap_ms - RAMP_DECEL_MS2 * dt)
  else:
    eff_cap_ms = target_ms
  return eff_cap_ms, eff_cap_ms


def on_v_cruise(v_cruise, v_ego, sm):
  global _eff_cap_ms, _last_t
  _get_sl_data()  # update from plugin bus
  if _sl_data is None:
    _reset_ramp()
    return v_cruise

  confirmed = _sl_data.get('confirmed', False)
  speed_limit = _sl_data.get('speedLimit', 0)
  if not (confirmed and speed_limit > 0):
    _reset_ramp()
    return v_cruise

  safety_capped = _sl_data.get('safetyCapped', True)
  source = _sl_data.get('source')

  # Inferred road-type limit, not safety-capped: soft coast + gas override.
  if source == SOURCE_ROAD_TYPE_INFERENCE and not safety_capped:
    offset_pct = _effective_offset_percent(speed_limit)
    target_ms = speed_limit * (1 + offset_pct / 100.0) * CV.KPH_TO_MS
    # A fast lead means the inferred limit is likely wrong — treat it like a
    # driver override (suspend the cap) rather than glide down to a bad value.
    gas = _gas_pressed(sm) or _lead_overrides_limit(sm, speed_limit)
    now = time.monotonic()
    dt = 0.0 if _last_t is None else min(max(now - _last_t, 0.0), DT_CLAMP_S)
    _last_t = now
    _eff_cap_ms, enforced = _ramp_cap(_eff_cap_ms, target_ms, dt, gas, v_ego)
    if enforced is None:
      return v_cruise
    return min(v_cruise, enforced)

  # All other sources (YOLO, curvature, OSM-confirmed) and any safety cap:
  # existing immediate enforcement.
  _reset_ramp()
  # Skip speed limit if lead vehicle is much faster — limit data likely wrong.
  # Does NOT apply to safety caps: a fast lead doesn't make a curve less tight.
  if not safety_capped and _lead_overrides_limit(sm, speed_limit):
    return v_cruise

  offset_pct = 0 if safety_capped else _effective_offset_percent(speed_limit)
  v_limit = speed_limit * (1 + offset_pct / 100.0) * CV.KPH_TO_MS
  if v_limit < v_cruise:
    return v_limit

  return v_cruise


def _pid_alive(name: str) -> bool:
  import os as _os
  try:
    pid = int(open(f'/data/plugins-runtime/.pids/{name}.pid').read().strip())
    _os.kill(pid, 0)
    return True
  except Exception:
    return False


def on_health_check(acc, **kwargs):
  alive = _pid_alive("speedlimitd")
  result = {"status": "ok" if alive else "warning", "process_alive": alive}
  if not alive:
    result["warnings"] = ["speedlimitd process not running"]
  return {**acc, "speedlimitd": result}
