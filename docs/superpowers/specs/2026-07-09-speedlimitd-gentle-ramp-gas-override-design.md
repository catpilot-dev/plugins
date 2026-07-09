# Speedlimitd: Gentle Ramp + Momentary Gas Override for Inferred Limits

**Date:** 2026-07-09
**Component:** `plugins/speedlimitd/planner_hook.py`
**Status:** Design approved, pending implementation plan

## Problem

`speedlimitd` fuses several speed-limit sources. One of them —
`roadTypeInference` (`source == 2`) — infers the limit from road type and
lane geometry, with a confidence (`self.lane_conf`) that tracks how clearly
the model sees the lane lines. On a highway with no lead car and faint lane
lines, this inferred limit can drop significantly and spuriously (e.g.
100 → 60 km/h). Today that reduction is enforced quickly (subject only to
`speedlimitd`'s coarse display stepping), producing a **temporary hard brake**
before the limit recovers. This is uncomfortable and, because the drop is
usually a false alarm, unwarranted.

## Goals

1. When the **inferred** limit drops, slow the car **gently** — a bounded
   deceleration of ~0.2 m/s², a coast rather than a brake.
2. Let the driver **override** the slowdown by pressing the gas pedal, and
   resume gentle enforcement of the *actual* current limit when the pedal is
   released.
3. **Do not touch** safety caps or confirmed detections — curvature/vision
   safety caps (`source 4` / `safetyCapped`), YOLO sign detections
   (`source 1`), and OSM-confirmed limits keep their current immediate
   enforcement.

## Non-Goals

- No timed "hold" of a captured set-point (an earlier idea, dropped). The
  override lasts exactly as long as the pedal is held.
- No change to `speedlimitd.py`. The perception/fusion daemon keeps reporting
  the true fused limit; all new behavior is an enforcement-policy layer.
- No "significant drop" threshold. The 0.2 m/s² ramp smooths every reduction
  uniformly, so a magnitude gate is unnecessary.

## Scope / Gate

New behavior applies **only when all** of:
- `_sl_data['source'] == 2` (`roadTypeInference`),
- `_sl_data['confirmed']` is true and `speedLimit > 0`, and
- `_sl_data['safetyCapped']` is **False**.

Every other case falls through to the **existing** `on_v_cruise` logic
unchanged:
- `source == 1` (YOLO), `source == 4` (curvature): immediate cap.
- `safetyCapped == True`: immediate cap, no offset (existing behavior) — even
  when `source == 2`, because a curve cap can sit at/below an inferred limit
  (`safetyCapped` true while `source == 2`); a genuine curve slowdown must
  never be softened or gas-overridable.
- Not confirmed / no limit: pass `v_cruise` through.

`source` and `confirmed` are already published in `speedLimitState`, so no
producer change is required.

## Design

### Signals (all already available in the hook)
- `_sl_data['speedLimit']` (km/h), `_sl_data['source']`, `_sl_data['confirmed']`,
  `_sl_data['safetyCapped']`
- `sm['carState'].gasPressed`
- `v_ego`, `v_cruise` (hook arguments)

### State (module-level; reset whenever the gate is false)
- `eff_cap_ms` — the currently enforced speed-limit cap, in m/s. `None` when
  inactive.
- `_last_t` — monotonic timestamp of the previous cycle, for `dt`.

### Per-cycle logic (only when gate is true)

```
offset%   = _effective_offset_percent(speed_limit_kph)   # +15% <80, +10% >=80
target_ms = speed_limit_kph * (1 + offset%/100) * KPH_TO_MS
dt        = clamp(now - _last_t, 0 .. DT_CLAMP_S)

if eff_cap_ms is None:
    eff_cap_ms = max(target_ms, v_ego)     # init: never cap below current speed

if gasPressed or lead_overrides_limit(sm, speed_limit):
    eff_cap_ms = max(eff_cap_ms, v_ego)    # float up with the driver
    enforced   = None                      # cap fully suspended -> return v_cruise
else:
    if target_ms < eff_cap_ms:
        eff_cap_ms = max(target_ms, eff_cap_ms - RAMP_DECEL_MS2 * dt)  # glide DOWN <=0.2 m/s^2
    else:
        eff_cap_ms = target_ms             # restore UP immediately
    enforced   = min(v_cruise, eff_cap_ms)

return v_cruise if enforced is None else enforced
```

### Resulting behavior
- Inferred limit drops → cap glides down at ≤0.2 m/s² → ACC decelerates
  gently, no brake jolt.
- Driver presses gas → cap suspended, car accelerates freely; `eff_cap_ms`
  tracks up with `v_ego`.
- Driver releases → the actual current inferred limit is re-enforced,
  resuming from current speed at the same 0.2 m/s² glide.
- Lane lines return / inferred limit rises → cap restores immediately (no
  artificial lag upward).
- Lead-override (existing mechanism: lead faster than limit ⇒ limit likely
  wrong) continues to suppress the cap, folded into the gas branch.

### Constants (tunable, module-level)
- `RAMP_DECEL_MS2 = 0.2`
- `DT_CLAMP_S = 0.2`

### Interaction with existing gradual step in `speedlimitd`
`speedlimitd` still steps its *displayed* limit (for the HUD sign) via
`_step_speed_limit`. The hook ramps toward the published (stepped) value; since
the hook's 0.2 m/s² glide is far slower than the display step, the hook cap is
the binding constraint and governs the enforced trajectory. No conflict, and
the HUD sign behavior is unchanged.

## Structure (for testability)

Extract the ramp math into a **pure** helper so tests need no clock mocking:

```
def _ramp_cap(eff_cap_ms, target_ms, dt, gas, v_ego):
    """Return (new_eff_cap_ms, enforced_ms_or_None)."""
```

`on_v_cruise` computes the gate and `dt` (from `time.monotonic()`), then calls
`_ramp_cap`. The non-`source==2` paths remain exactly as today.

## Testing

Unit tests in the existing `tests/test_speedlimitd.py` planner_hook section:

1. **Glide-down rate:** repeated `_ramp_cap` calls with `gas=False`,
   `target < eff_cap` reduce `eff_cap` by ≤ `0.2 * dt` per step; never below
   `target`.
2. **Immediate up-restore:** `target > eff_cap` sets `eff_cap = target` in one
   step.
3. **Gas suspends:** `gas=True` returns `enforced=None` and floats `eff_cap`
   up to at least `v_ego`.
4. **Release resumes:** after a gas interval, `gas=False` resumes the glide
   from the (higher) current cap toward `target`.
5. **Init guard:** first activation with `v_ego > target` does not cap below
   `v_ego`.
6. **Gate — source 1 (YOLO):** immediate cap, no ramp (existing behavior).
7. **Gate — source 4 / safetyCapped:** immediate cap, path unchanged —
   including the `source == 2` **and** `safetyCapped == True` case (curve cap
   at/below the inferred limit): no ramp, no gas override.
8. **Lead override:** with a fast lead, cap is suppressed for `source == 2`.
9. **Not confirmed / no limit:** `v_cruise` returned unchanged; state resets.

## Files Touched
- `plugins/speedlimitd/planner_hook.py` — add `_ramp_cap`, state, and the
  `source == 2` branch in `on_v_cruise`.
- `plugins/speedlimitd/tests/test_speedlimitd.py` — add ramp/override tests.
