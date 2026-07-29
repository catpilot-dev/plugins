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
# release. The cap is enforced directly — DCC comfort-limits the deceleration
# (no artificial ramp). See
# docs/superpowers/specs/2026-07-10-speedlimitd-driver-intent-enforcement-design.md
SOURCE_ROAD_TYPE_INFERENCE = 2  # _sl_data['source'] value for inferred limits

# Large inferred-drop gate (route 3d1 seg 29). The lane-count-first inference is
# steadier than the old OSM flicker, so a spurious low read now persists long
# enough to *enforce*: an inferred lane-count=40 limit at v_ego 85 km/h commanded
# a real 85->45 hard slowdown on an unnamed road (road_id='' => no baseline floor
# today). Fix: an INFERRED limit whose enforcement would require a large
# deceleration must persist continuously before it lowers v_cruise. Display is
# unaffected (the driver sees the number immediately). Reference is v_ego (the
# decel magnitude ~ v_ego - limit), NOT the previous limit. This gate NEVER
# applies to safety caps (source==4 / safetyCapped -- the a_y layer must slow
# promptly), manual/confirmed limits (the driver's explicit choice), or small
# drops (< LARGE_DROP_KPH below v_ego -- enforced immediately as before).
LARGE_DROP_KPH = 20.0     # a limit this far below v_ego is a "large drop" (hard brake)
LARGE_DROP_GATE_S = 3.0   # inferred large drop must persist this long before it enforces

# Narrow-band lane-count limits are DISPLAY-ONLY (route 3d1 seg 29). speedlimitd
# publishes a ≤2-confident-lane inferred limit (40/30) for DISPLAY but flags it
# laneCountNarrow=True. Lane count is a poor predictor of a LOW limit -- a 2-lane
# road is anywhere from a 40 link to an 80 rural highway -- so a narrow read must
# never command a hard slowdown (seg 29: a ≤2-lane read → 40 on a wide 85 km/h
# road, SUSTAINED 13.4 s; the 3 s large-drop gate only delayed it). This is a
# permanent exclusion from the enforcement path, complementary to (not a weakening
# of) the large-drop gate above: everything else stays enforcing -- ≥3-lane limits
# (60/80, informative wide→faster), G/S expressway promotes, safety caps
# (source 4), and manual/confirmed limits.

_sl_sub = None
_sl_data = None

# Enforcement state.
_baseline_ms = None   # inferred running-max target on the current road (floor)
_gas_floor_ms = None  # driver-override hold floor (all sources), set post gas
_road_id = ''         # last non-empty OSM road identity

# Large inferred-drop gate state.
_large_drop_target_ms = None  # pending large-drop candidate target (m/s), for observability
_large_drop_since = None      # monotonic time the large drop first appeared continuously


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


def _reset_all():
  """Clear the floors (road identity is kept across brief invalid limits)."""
  global _baseline_ms, _gas_floor_ms, _large_drop_target_ms, _large_drop_since
  _baseline_ms = None
  _gas_floor_ms = None
  # No confirmed limit => no pending large-drop candidate.
  _large_drop_target_ms = None
  _large_drop_since = None


def on_v_cruise(v_cruise, v_ego, sm):
  global _baseline_ms, _gas_floor_ms, _road_id, _large_drop_target_ms, _large_drop_since
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

  # Display-only guard (route 3d1 seg 29). A narrow-band lane-count guess is
  # published for display but must never lower v_cruise. It is inherently
  # source==2 / not-safety-capped (see speedlimitd.lane_count_narrow), so this
  # never touches safety caps, YOLO signs, or manual/confirmed limits. Drop any
  # carried inferred floor / large-drop candidate so nothing stale re-enforces
  # when the road later widens; road identity is left intact for change detection.
  if inferred and _sl_data.get('laneCountNarrow', False):
    _baseline_ms = None
    _large_drop_target_ms = None
    _large_drop_since = None
    return v_cruise

  offset_pct = 0 if safety_capped else _effective_offset_percent(speed_limit)
  target_ms = speed_limit * (1 + offset_pct / 100.0) * CV.KPH_TO_MS

  # Large inferred-drop gate. Track, continuously, whether an INFERRED limit sits
  # >= LARGE_DROP_KPH below v_ego (a hard-brake candidate). Updated every cycle
  # (before the gas/floor logic) so the persistence timer is truly continuous;
  # the road_id is deliberately NOT consulted here (unnamed-road/road_id churn is
  # exactly the case this gate covers -- a persistent large drop must still be
  # able to enforce despite churn). Reset the timer the moment the drop is no
  # longer large (limit rose back above the v_ego - LARGE_DROP line) so a
  # transient vision glitch never brakes.
  large_drop = inferred and (v_ego * CV.MS_TO_KPH - speed_limit) >= LARGE_DROP_KPH
  if large_drop:
    if _large_drop_since is None:
      _large_drop_since = time.monotonic()
    _large_drop_target_ms = target_ms
    drop_gated = (time.monotonic() - _large_drop_since) < LARGE_DROP_GATE_S
  else:
    _large_drop_since = None
    _large_drop_target_ms = None
    drop_gated = False

  # New (non-empty) road: drop carried floors. A transient empty road_id (OSM
  # tile gap on the same road) is NOT a change.
  if road_id != '' and road_id != _road_id:
    _road_id = road_id
    _baseline_ms = None
    _gas_floor_ms = None

  # Gas pedal: universal suspend (all sources, incl. safety caps). Raise the
  # hold floor to current speed so enforcement resumes from here on release.
  if _gas_pressed(sm):
    _gas_floor_ms = v_ego
    return v_cruise

  # Ratchet the gas floor down with the driver; clear once eased to the limit.
  if _gas_floor_ms is not None:
    _gas_floor_ms = min(_gas_floor_ms, v_ego)
    if _gas_floor_ms <= target_ms:
      _gas_floor_ms = None

  # Baseline (road-continuity) floor — inferred limits only, and only when we
  # actually have an OSM road identity. Without one (unnamed ramp/link, e.g. an
  # interchange motorway_link) we can't assert "same road", so the hold is
  # invalid — let the inferred/vision cap control and slow the car instead.
  baseline_floor = None
  if inferred and road_id != '':
    _baseline_ms = target_ms if _baseline_ms is None else max(_baseline_ms, target_ms)
    baseline_floor = min(_baseline_ms, v_ego)
  else:
    _baseline_ms = None

  floors = [f for f in (baseline_floor, _gas_floor_ms) if f is not None]
  effective_floor = max(floors) if floors else None
  floored_target = target_ms if effective_floor is None else max(target_ms, effective_floor)

  # Withhold a not-yet-persisted large inferred drop. This is an ADDITIONAL gate
  # layered on the inferred path; it never fires for safety caps, manual limits,
  # or small drops (drop_gated is False for those). Honor any existing floor
  # (gas hold / named-road baseline continuity) so those keep capping, but do NOT
  # let the unproven large drop itself lower v_cruise. On an unnamed road there is
  # no baseline floor (road_id='' => effective_floor is None), which is precisely
  # the seg-29 case the existing floor misses: hold v_cruise unchanged. Once the
  # drop persists LARGE_DROP_GATE_S (drop_gated flips False), enforcement falls
  # through to the normal cap below and the floor lowers as usual.
  if drop_gated:
    if effective_floor is not None and effective_floor < v_cruise:
      return effective_floor
    return v_cruise

  # Fast lead suggests a non-safety confirmed limit is wrong — skip. Not for
  # inferred limits (the baseline floor handles those), safety caps (a fast lead
  # doesn't make a curve less tight — route 2fd), or when a gas hold is active.
  if not inferred and not safety_capped and _gas_floor_ms is None \
      and _lead_overrides_limit(sm, speed_limit):
    return v_cruise

  # Enforce the cap directly; DCC comfort-limits the deceleration. floored_target
  # is <= v_ego whenever the limit dropped, so the cap never commands accel.
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
