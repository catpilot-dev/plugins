# Speedlimitd: Jerk-Limited Allowed-Speed Ceiling

**Date:** 2026-09-10
**Component:** `plugins/speedlimitd/planner_hook.py`, `plugins/speedlimitd/speedlimitd.py`
**Status:** Design approved, pending implementation plan
**Amends:** `2026-07-10-speedlimitd-driver-intent-enforcement-design.md`
(the hold-floor model is unchanged; this spec replaces the *shape* of the
enforced target's motion — the "no artificial ramp" clause of that spec no
longer holds for non-safety limit drops.)

## Problem

A limit drop brakes the car harshly. The cause is not the average deceleration
— it is the **discontinuity**.

Today the transition is a fixed-time ladder inside the daemon
(`_step_speed_limit`, `_STEP_DOWN_INTERVAL = 3 s`): the published limit moves
one standard rung at a time, and `planner_hook` converts each rung into an
instantaneous `v_cruise` value. An 80 → 40 km/h drop is therefore three step
changes in the setpoint:

| t | displayed | enforced target (+offset) | step |
|---|-----------|---------------------------|------|
| 0 s | 80 | 88 km/h | — |
| 3 s | 60 | 69 km/h | −5.3 m/s |
| 6 s | 50 | 57.5 km/h | −3.2 m/s |
| 9 s | 40 | 46 km/h | −3.2 m/s |

Each step opens a large instantaneous setpoint gap. On this car that is exactly
the quantity that drives DCC deceleration (see
`project_dcc_setpoint_bias`: setpoint gap → DCC accel, corr **+0.746**), so
each rung produces a brake spike followed by a coast — three times per
transition. The average rate over the 9 s is only ~1.2 m/s²; the *peaks* are
what the driver feels.

Braking jerk is the complaint. Acceleration jerk is not — per the user, brisk
acceleration is fine and enjoyable.

## Model: ramp the ceiling, not the vehicle state

The quantity that ramps is the **allowed-speed ceiling** — the limit plus its
comfort offset, in m/s. It is a pure function of the limit history: it does not
read, track, or manipulate `v_ego` or the driver's `v_cruise`.

```
speed limit 80 km/h  →  +10% offset  →  ceiling 88 km/h
speed limit 40 km/h  →  +15% offset  →  ceiling 46 km/h
```

The ceiling slides continuously from 88 to 46 under a trapezoidal acceleration
profile — its value *and* its slope are continuous, so DCC never sees a step.
`on_v_cruise` then does what it already does:

```
if ceiling < v_cruise:  return ceiling
else:                   return v_cruise
```

Because the ceiling is decoupled from vehicle state, the ramp is fully
deterministic: the same limit change always produces the same ceiling
trajectory regardless of what the car is doing.

**Offset tiers are unchanged** (`_effective_offset_percent`): +15% below
80 km/h, +10% at/above, +0% when `safetyCapped`. The offset is applied to the
*target* limit; the ceiling ramps toward the finished value, so an offset-tier
crossing mid-ramp introduces no discontinuity.

## Trapezoidal profile

State: `_ceiling_ms` (m/s), `_ceiling_rate` (m/s², signed), `_last_t`.

```
CEIL_A_DOWN  = 0.5   # m/s²  peak descent rate — kept below COMFORT_BRAKE (0.8)
CEIL_J_DOWN  = 0.5   # m/s³  jerk limit on the descent; this is the whole point
CEIL_DT_MAX  = 0.2   # s     dt clamp (plannerd runs at DT_MDL = 0.05)
```

Per tick, after `target_ms` is computed and **before** the hold floors:

```python
if _ceiling_ms is None or safety_capped:
    _ceiling_ms, _ceiling_rate = target_ms, 0.0     # immediate — safety bypasses the ramp
else:
    dt = clamp(now - _last_t, 0.0, CEIL_DT_MAX)
    _ceiling_ms, _ceiling_rate = _advance_ceiling(_ceiling_ms, _ceiling_rate, target_ms, dt)
```

and `_advance_ceiling` is:

```python
err = target_ms - ceiling_ms
if err >= 0.0:
    return target_ms, 0.0            # a rising limit is a release — immediate

dj = CEIL_J_DOWN * dt
reach = sqrt(2 * CEIL_J_DOWN * max(0.0, -err - abs(rate) * dt))
want  = -min(CEIL_A_DOWN, reach)

rate    += clamp(want - rate, -dj, +dj)
ceiling += rate * dt

if ceiling <= target_ms:             # discrete integration overshoot
    ceiling = target_ms              # pin the value…
    if abs(rate) <= dj:
        rate = 0.0                   # …and only zero the slope once it is within one jerk step
```

### Why the stop budget looks one tick ahead

`reach` is the fastest slope we may still carry and bleed to zero inside the
error that will remain **after** this tick's travel — note the
`abs(err) - abs(rate)*dt`.

The textbook form, `sqrt(2·j·|err|)`, budgets against the error we have *now*.
That is optimistic by exactly one tick, and the shortfall compounds: the bleed
starts late, the ceiling reaches the target with slope still on it, and the
arrival has to zero that slope in a single step. Measured on an 88 → 46 km/h
descent at dt = 0.05: the ceiling landed carrying −0.15 m/s², a **6×** jerk-limit
violation on the final tick — the exact discontinuity this whole design exists
to remove, relocated to the end of the ramp. A half-jerk-step correction
(`sqrt(2j|err| + (j·dt)²/4) − j·dt/2`) narrows it to 1.7× but does not close it.

The look-ahead form is jerk-compliant on every tick, verified across 80→40,
40→80, 80→60, 90→40 and a 1 km/h nudge, at both dt = 0.05 and the dt = 0.2
clamp. It costs ~0.2 s of ramp length.

Pinning the value on overshoot while letting the slope bleed out over the
following ticks is the same principle applied to the other end: a hard
`rate = 0` on arrival is a discontinuity even when the value is already correct.

Resulting 80 → 40 km/h (Δv = 11.1 m/s): **~24 s**, ramp-in 1.0 s, hold at
0.5 m/s², ramp-out 1.0 s. That is ~410 m of road; a 120 → 40 drop is ~49 s /
~1 km. The descent rate is the comfort knob — it shipped at 0.8 and was
lowered to 0.5 after route 45f confirmed the ramp binding on real drops. Slower than today's 9 s, but with no spikes — the
peak demand falls from ~5.3 m/s of instantaneous gap to a steady 0.8 m/s².

### The ascent is not shaped at all

A rising limit is a **release**, not a manoeuvre. The hook only ever lowers
`v_cruise`, so handing the ceiling straight to a higher target merely stops
capping — DCC's own ~+0.5 m/s² envelope shapes the acceleration from there.

This was initially built as a brisk ramp (`CEIL_A_UP = 1.5`, `CEIL_J_UP = 1.0`)
on the reasoning that a step change in the setpoint should be avoided in both
directions. That was wrong: since DCC caps real acceleration near +0.5 m/s²,
any ascent rate above ~0.6 produces identical car behaviour, so the ramp bought
nothing and only postponed the release — most visibly when a safety cap lifts
at the end of a curve and the car stays held down while the ramp catches up.
Braking jerk is the complaint; acceleration is not.

## Interaction with existing enforcement

The hold-floor model from the 2026-07-10 spec is **unchanged in structure**.
`_ceiling_ms` simply replaces `target_ms` at the downstream use sites:

- **Baseline floor** keeps tracking the *raw* `target_ms`, not the ceiling —
  `_baseline_ms` means "the highest limit seen on this road", which is a
  property of the limit, not of the ramp.
- **Gas floor**, **lead override**, and the final `< v_cruise` comparison all
  consume `_ceiling_ms`.
- **Never-speed-up guarantee** is preserved: the hook still only ever lowers or
  holds `v_cruise`.

### Safety caps bypass the ramp

When `safetyCapped` is true — proactive curvature cap or reactive measured-a_y
cap — the ceiling is assigned the target immediately and the rate is zeroed.
A tightening curve must not wait 15 s. This preserves the existing invariant
that safety caps always win, and it is why the descent rate (0.8 m/s²) is set
equal to `COMFORT_BRAKE`: a safety cap's own distance-aware profile is never
gentler than the comfort ramp, so the two can never fight.

### Reset semantics

- `_reset_all()` clears `_ceiling_ms` to `None`, so the next valid limit
  initialises immediately rather than ramping from a stale ceiling.
- A new `road_id` does **not** reset the ceiling — that would reintroduce
  exactly the jump this spec removes. It clears the hold floors only, as today.
- The ceiling continues ramping while the gas is pressed. It is a pure function
  of the limit; the gas floor is what suspends enforcement.

## Daemon change: the display ladder is removed

The ladder currently lives in `speedlimitd.py` and does double duty — it sets
both what the sign shows and how fast the car brakes. Splitting those is the
point of this design:

- **`speedlimitd`** (5 Hz) is perception and fusion: *what the limit is*.
- **`planner_hook`** (20 Hz, DT_MDL) is enforcement: *how we get there*.

Removed: `_STEP_DOWN_INTERVAL`, `_STEP_UP_INTERVAL`, `_step_speed_limit()`,
`self._last_step_time`, and the gradual-transition block. `_displayed_speed_limit`
is assigned the target directly. The CN-ladder snap and the OSM ±5 rounding
stay — they are display formatting, not transition timing. The safety-cap clamp
and the `safetyCapped` computation are unchanged.

Consequence: the HUD sign flips 80 → 40 at detection while the car is still
bleeding speed, so the driver briefly sees themselves over the displayed limit.
This is accepted — the sign now tells the truth about the road, and the ramp is
an enforcement concern.

## Accepted trade-off: slower to bite below the limit

Because the ceiling ignores vehicle state, it takes longer to affect a driver
who is *already* below the old limit. Driver at 70 km/h in an 80 zone (ceiling
88), limit drops to 40:

- The ceiling spends ~6 s descending 88 → 70 with no effect on the car.
- It then captures the car and slows it 70 → 46 over ~9 s.

Today's ladder reaches 69 after 3 s and captures that driver sooner. This is a
deliberate consequence of the decoupled design.

**Mitigation, if a drive shows the delay is objectionable:** initialise the
ceiling at `min(_ceiling_ms, v_cruise)` when a descent begins. This *reads*
`v_cruise` without manipulating it and preserves determinism given the same
driver setpoint. Not implemented initially — add only on evidence.

## Structure & Testability

The profile integrator is a pure function of `(ceiling, rate, target, dt)` and
the four constants. It is extracted as a module-level helper:

```python
def _advance_ceiling(ceiling_ms, rate_ms2, target_ms, dt) -> tuple[float, float]
```

so the trajectory can be tested without a plugin bus, a `sm`, or a clock.
`on_v_cruise` keeps the I/O and calls it once per tick.

## Tests

New `tests/test_planner_hook.py` (none exists today):

1. **Jerk bound** — over a simulated 80 → 40, `|Δrate|` never exceeds `j·dt` on
   any tick.
2. **Continuity** — no single-tick change in `_ceiling_ms` exceeds `a·dt`.
3. **No overshoot** — the ceiling never falls below the target, and arrives with
   `rate ≈ 0`.
4. **Timing** — 80 → 40 completes within an expected window (13–17 s).
5. **Asymmetry** — 40 → 80 completes materially faster than 80 → 40.
6. **Safety bypass** — with `safetyCapped` true, the ceiling equals the target on
   the first tick, no ramp.
7. **Mid-ramp retarget** — a second drop (80 → 60 → 40) during an active ramp
   keeps the state continuous, with no step in value or slope.
8. **Floors preserved** — baseline floor, gas floor, gas ratchet, and lead
   override behave as in the 2026-07-10 spec, now against the ramped ceiling.
9. **Reset** — an invalid/unconfirmed limit clears the ceiling; the next valid
   limit is applied immediately, not ramped from stale state.
10. **Never speeds up** — the returned value is never above the incoming
    `v_cruise`, in any of the above.

Existing tests that assert removed ladder behaviour and must be reworked:

- `tests/test_speedlimitd.py:2591` `test_oscillation_ladder_damps_displayed_limit`
  — the damping intent moves to the hook; the daemon-side assertion goes away.
- `tests/test_speedlimitd.py:2977` `test_osm_step_is_plus_minus_10` — the ±10
  stepping no longer exists; the OSM ±5 *rounding* assertion should be kept.

## Files Touched

| File | Change |
|------|--------|
| `plugins/speedlimitd/planner_hook.py` | `_advance_ceiling` helper, ceiling state, constants, wire into `on_v_cruise` |
| `plugins/speedlimitd/speedlimitd.py` | remove ladder constants, `_step_speed_limit`, `_last_step_time`, transition block |
| `plugins/speedlimitd/tests/test_planner_hook.py` | new |
| `plugins/speedlimitd/tests/test_speedlimitd.py` | rework the two ladder tests |
| `plugins/speedlimitd/DESIGN.md` | §"Temporal accumulators & the display ladder", §"Enforcement & gas override" |

`README.md` needs no change: its only mentions of "ramp" are the curve-approach
profile and the road-type on/off-ramp class, neither of which this spec touches.

## Verification

This is a longitudinal-behaviour change. Bench tests are necessary but not
sufficient — it is not done until an on-car drive confirms that a non-safety
limit drop no longer produces a brake spike, and that safety caps still bite
immediately.
