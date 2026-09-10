import math
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

# --- Jerk-limited allowed-speed ceiling ---
# The enforced target does not jump between limits; it slides under a
# trapezoidal acceleration profile so both its value and its slope are
# continuous. Setpoint GAP is what drives DCC deceleration on this car
# (corr +0.746), so a step change in the target is a brake spike — three of
# them for an 80 -> 40 drop under the old fixed-time ladder.
# Only the DESCENT is shaped. A rising limit is a release, not a manoeuvre —
# the hook only ever lowers v_cruise, so handing the ceiling straight to a
# higher target just stops capping, and DCC's own ~+0.5 m/s² envelope shapes
# the acceleration from there. Ramping the release only delayed it.
CEIL_A_DOWN = 0.5    # m/s²  peak descent rate — deliberately gentler than
                     #       speedlimitd's COMFORT_BRAKE (0.8), so the comfort ramp
                     #       is always the gentler of the two and a safety cap
                     #       still wins simply by being steeper
CEIL_J_DOWN = 0.5    # m/s³  jerk limit on the descent
CEIL_DT_MAX = 0.2    # s     dt clamp (plannerd ticks at DT_MDL = 0.05)

_sl_sub = None
_sl_data = None

# Enforcement state.
_baseline_ms = None   # inferred running-max target on the current road (floor)
_gas_floor_ms = None  # driver-override hold floor (all sources), set post gas
_road_id = ''         # last non-empty OSM road identity
_ceiling_ms = None    # jerk-limited allowed-speed ceiling (m/s); None = uninitialised
_ceiling_rate = 0.0   # its current slope (m/s², signed)
_last_t = None        # monotonic timestamp of the last ceiling advance
_prev_target_ms = None  # previous tick's raw target, to detect a fresh drop


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


def _advance_ceiling(ceiling_ms, rate_ms2, target_ms, dt):
  """Advance the allowed-speed ceiling one tick toward target_ms.

  Returns (ceiling_ms, rate_ms2). A pure function of its arguments and the
  CEIL_* constants — no vehicle state, no clock — so the trajectory for a
  given limit change is always the same.

  A RISING target is applied at once: the hook only ever lowers v_cruise, so
  raising the ceiling merely stops capping, and DCC's own acceleration
  envelope shapes what follows. Only the descent is shaped, under a
  trapezoidal profile: the slope ramps in at the jerk limit, holds at the
  peak, then ramps back out so the ceiling arrives with zero slope. Every tick
  changes the slope by at most CEIL_J_DOWN·dt, including the last one — an
  arrival that zeroes a leftover slope is itself the brake spike this profile
  exists to remove.
  """
  err = target_ms - ceiling_ms
  if err >= 0.0:
    return target_ms, 0.0
  if dt <= 0.0:
    return ceiling_ms, rate_ms2

  dj = CEIL_J_DOWN * dt

  # The fastest slope we may still carry and bleed to zero inside the error
  # that will remain AFTER this tick's travel. Budgeting against the error we
  # have *now* is optimistic by one tick, and the shortfall compounds: the
  # bleed starts late, and the ceiling lands on the target with slope still on
  # it (measured: −0.15 m/s² left at arrival, 6× the jerk step).
  reach = math.sqrt(2.0 * CEIL_J_DOWN * max(0.0, -err - abs(rate_ms2) * dt))
  want = -min(CEIL_A_DOWN, reach)

  rate_ms2 += max(-dj, min(dj, want - rate_ms2))
  ceiling_ms += rate_ms2 * dt

  # Discrete integration can still step past the target. Pin the value there
  # and let the slope bleed out over the following ticks rather than zeroing
  # it outright.
  if ceiling_ms <= target_ms:
    ceiling_ms = target_ms
    if abs(rate_ms2) <= dj:
      rate_ms2 = 0.0
  return ceiling_ms, rate_ms2


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
  """Clear the floors and the ceiling (road identity is kept across brief
  invalid limits). Clearing the ceiling means the next valid limit is applied
  immediately rather than ramped from a stale value."""
  global _baseline_ms, _gas_floor_ms, _ceiling_ms, _ceiling_rate, _last_t
  global _prev_target_ms
  _baseline_ms = None
  _gas_floor_ms = None
  _ceiling_ms = None
  _ceiling_rate = 0.0
  _last_t = None
  _prev_target_ms = None


def on_v_cruise(v_cruise, v_ego, sm):
  global _baseline_ms, _gas_floor_ms, _road_id, _ceiling_ms, _ceiling_rate, _last_t
  global _prev_target_ms
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

  # New (non-empty) road: drop carried floors. A transient empty road_id (OSM
  # tile gap on the same road) is NOT a change.
  if road_id != '' and road_id != _road_id:
    _road_id = road_id
    _baseline_ms = None
    _gas_floor_ms = None

  # Advance the jerk-limited ceiling toward the target. This runs before the
  # gas early-return on purpose: the ceiling is a pure function of the limit,
  # so a gas hold must not freeze it — the gas FLOOR is what suspends
  # enforcement. Safety caps assign the target directly; a tightening curve
  # cannot wait out a 16 s ramp.
  now = time.monotonic()
  if _ceiling_ms is None or safety_capped:
    _ceiling_ms, _ceiling_rate = target_ms, 0.0
  else:
    # A FRESH drop anchors the ceiling to where the car actually is. Starting
    # every descent at the old limit + offset meant the ceiling spent seconds
    # in dead air above the car before the cap reached it and anything
    # happened — on 80 -> 40 at 55 km/h, ~20 s and ~300 m into the 40 zone.
    # This is a step in the CAP, not in the setpoint GAP: clamping 88 -> 55
    # while the car is doing 55 leaves zero error, so there is nothing for DCC
    # to react to. Never below the new target (that would drag the driver down
    # and hold them there) and never upward (min).
    #
    # Fires only on the tick the target drops — re-anchoring every tick would
    # ratchet the cap down with a decelerating car. It costs the ceiling's
    # pure-function property at exactly one instant; the ramp is a pure
    # function of the limit from there on.
    if _prev_target_ms is not None and target_ms < _prev_target_ms - 1e-9:
      anchored = min(_ceiling_ms, max(v_ego, target_ms))
      if anchored < _ceiling_ms:
        _ceiling_ms, _ceiling_rate = anchored, 0.0
    dt = min(max(now - _last_t, 0.0), CEIL_DT_MAX) if _last_t is not None else 0.0
    _ceiling_ms, _ceiling_rate = _advance_ceiling(_ceiling_ms, _ceiling_rate, target_ms, dt)
  _last_t = now
  _prev_target_ms = target_ms

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

  # The ramped ceiling — not the raw target — is what gets enforced. The floors
  # above still track the RAW target: they are statements about the limit
  # ("highest seen on this road", "driver's held speed"), not about the ramp.
  floors = [f for f in (baseline_floor, _gas_floor_ms) if f is not None]
  effective_floor = max(floors) if floors else None
  floored_target = _ceiling_ms if effective_floor is None else max(_ceiling_ms, effective_floor)

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
