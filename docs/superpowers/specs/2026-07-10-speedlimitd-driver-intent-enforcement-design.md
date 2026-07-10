# Speedlimitd: Driver-Intent Speed-Limit Enforcement

**Date:** 2026-07-10
**Component:** `plugins/speedlimitd/planner_hook.py`
**Status:** Design approved, pending implementation plan
**Supersedes:** `2026-07-09-speedlimitd-gentle-ramp-gas-override-design.md`
(that spec's gentle ramp is retained as a sub-part; the momentary gas override
and the source==2 immediate-cap gate are replaced by the model below.)

## Guiding Principle

**speedlimitd assists but never fights the driver.** If the car's behavior is
fine the driver never intervenes; the instant the driver presses the gas, the
system yields to that intent. Two corollaries, both required by the design:

1. **Never command acceleration** because a speed limit changed — a limit *drop*
   may only slow the car or hold it, never speed it up.
2. **Never brake the car below its current speed** except for a *real,
   corroborated* reduction the driver has not overridden — a genuine lower limit
   on a newly-entered road, a curvature/safety cap, or a detected sign, when the
   driver is not on the gas.

## Problem

The inferred road-type limit (`_sl_data['source'] == 2`, confidence =
`lane_conf`) jitters downward when lane lines get faint (weather, a passing
truck occluding the markings). Because the OSM road identity has not changed,
the true limit almost certainly has not dropped — so following that drop and
braking is both wrong and jarring. Separately, when the driver deliberately
accelerates past *any* limit with the gas pedal, the system should hold their
chosen speed on release rather than braking back.

## Model: one hold floor, two activations

A **hold floor** is a speed the enforced cap is not allowed to fall below. It is
always `≤ v_ego`, so it can never command acceleration. The enforced target is:

```
floored_target = max(limit_target, effective_floor)
```

Two independent floors combine (`effective_floor = max` of whichever are
active):

### 1. Baseline floor — road-continuity, INFERRED (`source==2`) only, requires a road identity
- **Only active when `road_id != ''`.** The hold means "same road ⇒ this drop is
  spurious"; without an OSM identity we cannot assert continuity, so the hold is
  invalid and the inferred/vision cap controls (the car slows via the ramp).
  This was added after route 3a1: the interchange is unnamed `motorway_link`
  ways (`road_id == ''`, the named S20 mainline is ~47 m away and never matched),
  so the baseline held the car at ~65 and suppressed the correct 40 km/h ramp
  vision cap. An empty `road_id` disables the hold but does **not** clear
  `_road_id`, so returning to the same named road is not a false road change.
- `baseline` = running max of the inferred target since entering the current
  `road_id` (`roadName or wayRef`). Reset when `road_id` changes.
- **`road_id` change = a new *non-empty* identity that differs from the last
  non-empty one.** A transient empty `road_id` (OSM tile gap while physically on
  the same road) does **not** count as a change and does not reset the floors —
  otherwise a momentary map dropout would let a spurious drop through.
- `baseline_floor = min(baseline, v_ego)`.
- Effect: on a stable road, an uncorroborated inferred drop is held at the
  established speed (never below current speed); a **road_id change** resets the
  baseline so a genuinely lower limit on a new road *does* slow the car.

### 2. Gas floor — driver override, ALL sources, active only after a gas press
- While `gasPressed`: cap fully suspended (return `v_cruise`); `gas_floor = v_ego`.
- After release: `gas_floor` persists and **ratchets down** with the driver:
  `gas_floor = min(gas_floor, v_ego)`.
- Cleared (normal enforcement resumes) when `gas_floor ≤ limit_target` (driver
  has eased back to the limit) or `road_id` changes.
- Effect: after the driver accelerates over any limit — inferred **or a
  curve/safety cap** — their speed is held on release and followed down as they
  ease off, instead of braking back.

## Per-source behavior

| Source | Baseline floor | Gas floor | Ramp (0.5 m/s²) | No-gas default |
|---|---|---|---|---|
| Inferred `2` (not safety) | yes (always) | yes | yes | hold vs spurious drop; slow only on road_id change |
| Curvature/safety `4` (`safetyCapped`) | no | yes | **no** (immediate) | brake to cap promptly |
| YOLO `1`, OSM confirmed | no | yes | no (immediate) | brake to cap |

- **Safety caps keep prompt (immediate) braking** when the driver is not on the
  gas — the 0.5 m/s² ramp is *not* applied to them, so a tight curve is not
  approached too slowly. The ramp smooths only the inferred path.
- The automatic **lead-vehicle override** stays non-safety-only and applies only
  when no gas floor is active (unchanged from today; route-2fd protection).

## Never-speed-up guarantee

`effective_floor ≤ v_ego` always (both floors are `≤ v_ego`). Therefore
`floored_target = max(target, effective_floor)`:
- if `target > v_ego` (limit is *above* current speed) → `target`: the car may
  accelerate toward it — that is a limit *rise*, not a drop, and is allowed;
- if `target ≤ v_ego` → `effective_floor ≤ v_ego`: the cap is at or below current
  speed, so the car holds or slows but never accelerates.

## Per-cycle algorithm (`on_v_cruise(v_cruise, v_ego, sm)`)

```
_get_sl_data()
if no valid confirmed limit:  reset all state; return v_cruise

source        = _sl_data['source']
safety_capped = _sl_data['safetyCapped']
speed_limit   = _sl_data['speedLimit']
road_id       = _sl_data['roadName'] or _sl_data['wayRef'] or ''
gas           = carState.gasPressed
inferred      = (source == 2 and not safety_capped)
offset%       = 0 if safety_capped else _effective_offset_percent(speed_limit)
target        = speed_limit * (1 + offset%/100) * KPH_TO_MS

if road_id != '' and road_id != _road_id:   # new *non-empty* road only
    baseline = None; gas_floor = None; reset ramp; _road_id = road_id

if gas:                        # universal suspend
    gas_floor = v_ego; reset ramp
    return v_cruise

# ratchet + clear gas floor
if gas_floor is not None:
    gas_floor = min(gas_floor, v_ego)
    if gas_floor <= target: gas_floor = None

# baseline floor (inferred only)
baseline_floor = None
if inferred:
    baseline = target if baseline is None else max(baseline, target)
    baseline_floor = min(baseline, v_ego)
else:
    baseline = None

floors = [f for f in (baseline_floor, gas_floor) if f is not None]
effective_floor = max(floors) if floors else None
floored_target  = target if effective_floor is None else max(target, effective_floor)

if inferred:                   # gentle ramp only on the inferred path
    dt = clamp(now - _last_t, 0..DT_CLAMP_S); _last_t = now
    eff_cap, enforced = _ramp_cap(eff_cap, floored_target, dt, v_ego)
    return min(v_cruise, enforced)
else:                          # immediate enforcement (prompt for safety caps)
    reset ramp
    if not safety_capped and gas_floor is None and _lead_overrides_limit(sm, speed_limit):
        return v_cruise
    return min(v_cruise, floored_target)
```

`_ramp_cap(eff_cap, target, dt, v_ego)` (gas branch removed vs the prior spec):
init `eff_cap = max(target, v_ego)`; glide down at ≤ `RAMP_DECEL_MS2` (0.5),
restore up immediately; returns `(new_eff_cap, enforced)`.

## Walk-through

| Case | Behavior |
|---|---|
| Spurious inferred drop, same road, no gas | `baseline_floor≈v_ego` → hold current speed, no brake |
| Real lower limit, new road (road_id change), no gas | baseline reset → ramp glides down to new limit |
| Inferred limit recovers (rises) | `target` rises above floor → accelerate toward it |
| Ramp inferred 40, gas→60, release | during gas: suspend; release `gas_floor=60` → hold 60; ease off → follow down, settle at 40 |
| Curve cap 40, gas→60, release | hold 60 on release (gas floor, all sources); ease off → follow down |
| Curve cap 40 approached at 100, **no gas** | `gas_floor` inactive, no baseline (safety) → immediate cap → brake to 40 |
| Any limit, gas held | return `v_cruise` (fully suspended) |

## Safety note (on record, per user decision)

Because the gas floor applies to safety caps, after the driver accelerates over
a curve cap and keeps that speed (foot off, ACC holding), a *new, tighter* curve
will not be slowed for until the driver eases off — the driver is the authority
in that window. The lateral controller still steers throughout. This is a
deliberate "trust driver intention" choice.

## Structure & Testability

- State (module-level, reset on invalid limit): `_baseline_ms`, `_gas_floor_ms`,
  `_road_id`, `_eff_cap_ms`, `_last_t`.
- `_ramp_cap` stays a pure helper (deterministic `dt`, no clock).
- The floor bookkeeping is pure arithmetic over the state and inputs — unit
  tests drive it through `on_v_cruise` with a mock `sm` (carState.gasPressed,
  radarState.leadOne) and a patched `time.monotonic`, plus direct `_ramp_cap`
  tests.

## Tests

1. `_ramp_cap`: glide-down rate = `RAMP_DECEL_MS2`, clamp at target, immediate
   up-restore, init holds current speed.
2. Spurious inferred drop, same road → holds current speed (no brake).
3. Real inferred drop on road_id change → slows to new limit.
4. Inferred recovery (rise) → allows acceleration.
5. Never-speed-up: inferred drop while below the limit does not raise the cap.
6. Gas pressed (inferred / curve / any) → returns `v_cruise`.
7. Inferred gas→higher→release → holds release speed, then follows v_ego down,
   clears at target.
8. Curve (safety) gas→higher→release → holds release speed (gas floor on a
   safety cap).
9. Curve (safety), no gas → immediate cap, prompt braking, no ramp.
10. road_id change clears both `gas_floor` and `baseline`.
11. Lead override: non-safety, no gas floor → suppresses; ignored under safety
    cap and when a gas floor is active.
12. Invalid / unconfirmed limit → passes `v_cruise` through, state reset.

## Files Touched
- `plugins/speedlimitd/planner_hook.py`
- `plugins/speedlimitd/tests/test_speedlimitd.py`
