import time

from openpilot.common.constants import CV

# Lead vehicle override: if lead is traveling above the speed limit,
# the OSM/inferred limit is likely wrong — skip capping until lead slows down.
LEAD_OVERRIDE_THRESHOLD = 0.10  # 10% above speed limit
LEAD_MIN_STATUS = True  # lead must be tracked (status=True)

# Driver-intent speed-limit enforcement. speedlimitd assists but never fights
# the driver: a limit change may slow or hold the car but never speed it up, and
# it never brakes below current speed except for a real, un-overridden reduction.
# Inferred (road-type, source==2) limits jitter downward when lane lines get
# faint; a "hold floor" (always <= v_ego) rejects those spurious drops, and the
# gas pedal suspends enforcement entirely and holds the driver's speed on
# release. See
# docs/superpowers/specs/2026-07-10-speedlimitd-driver-intent-enforcement-design.md
SOURCE_ROAD_TYPE_INFERENCE = 2  # _sl_data['source'] value for inferred limits
RAMP_DECEL_MS2 = 0.5            # max deceleration the speed-limit cap may impose
DT_CLAMP_S = 0.2               # clamp per-cycle dt so a long gap can't jump the ramp

_sl_sub = None
_sl_data = None

# Enforcement state.
_baseline_ms = None   # inferred running-max target on the current road (floor)
_gas_floor_ms = None  # driver-override hold floor (all sources), set post gas
_road_id = ''         # last non-empty OSM road identity
_eff_cap_ms = None    # ramp state (inferred path)
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


def _reset_all():
  """Clear ramp + floors (road identity is kept across brief invalid limits)."""
  global _baseline_ms, _gas_floor_ms
  _reset_ramp()
  _baseline_ms = None
  _gas_floor_ms = None


def _ramp_cap(eff_cap_ms, target_ms, dt, v_ego):
  """Advance the enforced cap one cycle toward target_ms.

  Downward movement is deceleration-limited to RAMP_DECEL_MS2; upward is
  immediate. The result is clamped to max(target_ms, v_ego) so a stale-high cap
  can never permit acceleration on a drop or hold. Returns (new_eff, enforced).
  """
  if eff_cap_ms is None:
    eff_cap_ms = max(target_ms, v_ego)

  if target_ms < eff_cap_ms:
    eff_cap_ms = max(target_ms, eff_cap_ms - RAMP_DECEL_MS2 * dt)
  else:
    eff_cap_ms = target_ms

  eff_cap_ms = min(eff_cap_ms, max(target_ms, v_ego))
  return eff_cap_ms, eff_cap_ms


def on_v_cruise(v_cruise, v_ego, sm):
  global _baseline_ms, _gas_floor_ms, _road_id, _eff_cap_ms, _last_t
  _get_sl_data()  # update from plugin bus
  if _sl_data is None:
    _reset_all()
    return v_cruise

  confirmed = _sl_data.get('confirmed', False)
  speed_limit = _sl_data.get('speedLimit', 0)
  if not (confirmed and speed_limit > 0):
    _reset_all()
    return v_cruise

  safety_capped = _sl_data.get('safetyCapped', True)
  source = _sl_data.get('source')
  road_id = _sl_data.get('roadName') or _sl_data.get('wayRef') or ''
  inferred = (source == SOURCE_ROAD_TYPE_INFERENCE and not safety_capped)

  offset_pct = 0 if safety_capped else _effective_offset_percent(speed_limit)
  target_ms = speed_limit * (1 + offset_pct / 100.0) * CV.KPH_TO_MS

  # New (non-empty) road: drop carried floors and re-anchor. A transient empty
  # road_id (OSM tile gap on the same road) is NOT a change.
  if road_id != '' and road_id != _road_id:
    _road_id = road_id
    _baseline_ms = None
    _gas_floor_ms = None
    _reset_ramp()

  # Gas pedal: universal suspend (all sources, incl. safety caps). Raise the
  # hold floor to current speed so enforcement resumes from here on release.
  if _gas_pressed(sm):
    _gas_floor_ms = v_ego
    _reset_ramp()
    return v_cruise

  # Ratchet the gas floor down with the driver; clear once eased to the limit.
  if _gas_floor_ms is not None:
    _gas_floor_ms = min(_gas_floor_ms, v_ego)
    if _gas_floor_ms <= target_ms:
      _gas_floor_ms = None

  # Baseline (road-continuity) floor — inferred limits only, always active.
  baseline_floor = None
  if inferred:
    _baseline_ms = target_ms if _baseline_ms is None else max(_baseline_ms, target_ms)
    baseline_floor = min(_baseline_ms, v_ego)
  else:
    _baseline_ms = None

  floors = [f for f in (baseline_floor, _gas_floor_ms) if f is not None]
  effective_floor = max(floors) if floors else None
  floored_target = target_ms if effective_floor is None else max(target_ms, effective_floor)

  if inferred:
    now = time.monotonic()
    dt = 0.0 if _last_t is None else min(max(now - _last_t, 0.0), DT_CLAMP_S)
    _last_t = now
    _eff_cap_ms, enforced = _ramp_cap(_eff_cap_ms, floored_target, dt, v_ego)
    return min(v_cruise, enforced)

  # Non-inferred (YOLO, curvature, OSM-confirmed): immediate enforcement.
  _reset_ramp()
  # Fast lead suggests the limit is wrong — skip. Never for safety caps (a fast
  # lead doesn't make a curve less tight), nor when a gas hold is active.
  if not safety_capped and _gas_floor_ms is None and _lead_overrides_limit(sm, speed_limit):
    return v_cruise

  if floored_target < v_cruise:
    return floored_target
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
