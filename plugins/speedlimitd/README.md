# speedlimitd — Conditional Speed Control

**Type**: hybrid (process + hook)
**Dependency**: mapd plugin
**Process**: speedlimitd at 5 Hz (only_onroad)
**Hooks**: `planner.v_cruise`, `planner.subscriptions`, `ui.hud_set_speed_override`, `ui.render_overlay`, `ui.state_subscriptions`

## What it does

Fuses multiple speed limit sources into a single `speedLimitState` message,
and enforces confirmed limits by capping cruise speed in the longitudinal planner.

Even when no speed signs are visible and OSM has no data, road type inference
provides a safety floor — the car won't exceed reasonable speeds for the road
context (e.g. 30 km/h residential, 60 km/h secondary, 120 km/h motorway).

### Three-tier priority

| Priority | Source | Confidence | Description |
|----------|--------|------------|-------------|
| 1 (highest) | OSM maxspeed | 0.95 | From mapdOut.speedLimit |
| 2 | YOLO detection | 0.80 | Speed sign recognition (placeholder) |
| 3 (lowest) | Road type inference | 0.50 | Highway type + lane count lookup |

### Road type inference

Speed limits are inferred from:

- **OSM highway type**: motorway, trunk, primary, secondary, etc.
- **OSM wayRef**: G-roads = trunk, S-roads = primary, X-roads = secondary (China)
- **Road context**: freeway or city (from mapd)
- **Vision lane count**: counts visible lane lines from modelV2 (probability > 0.3 threshold, 1–4 lines)

Lane count promotes road class when it suggests a higher classification:
- 6+ lanes → motorway
- 3+ lanes → trunk
- 2+ lanes → primary

The higher of OSM type vs lane-inferred class wins. Lane detection requires
2 seconds of stability before applying. OSM lane count is preferred over vision
when OSM reports 3+ lanes.

#### Per-country speed tables

Speed tables are stored as TOML files in `speed_tables/`, one per country:

```
speed_tables/
  cn.toml   # China — GB 5768
  de.toml   # Germany — StVO
  au.toml   # Australia — Australian Road Rules
```

Each file defines urban/nonurban tables and a bounding box for GPS detection:

```toml
bbox = [18, 54, 73, 135]  # [min_lat, max_lat, min_lon, max_lon]
default_fallback = 40

[urban]
motorway = { multi = 100, single = 100 }
trunk    = { multi = 80,  single = 60 }
primary  = { multi = 60,  single = 60 }
# ...

[nonurban]
motorway = { multi = 120, single = 120 }
# ...
```

Adding a new country requires only a new TOML file — no code changes.

#### Auto-detection

On startup, speedlimitd subscribes to `gpsLocationExternal`. On first valid
GPS fix, it matches the coordinates against bounding boxes in all TOML files
and loads the matching country's speed tables. No internet connection required.

### Confirmation

- **Auto-confirmed on engage** — speed limit enforcement is active by default when openpilot goes onroad
- Toggle off/on via steering wheel resume button (short press) or UI speed sign tap
- **Sticky** — persists across speed limit changes; only manual toggle resets it

### Speed limit enforcement

The `planner.v_cruise` hook caps cruise speed when confirmed:

```
v_limit = speedLimit * (1 + offset) * KPH_TO_MS
```

Speed-dependent comfort offset:
- ≤ 50 km/h zones: +40%
- 51–60 km/h zones: +30%
- > 60 km/h zones: +10%

**Lead override**: if a tracked lead vehicle is traveling > 10% above the speed
limit, capping is skipped to maintain traffic flow.

Only enforced when `v_limit < v_cruise` (never increases cruise speed).

### HUD overlay

A Vienna-style speed limit sign is rendered on the onroad HUD:

- **50% opacity**: suggestion (unconfirmed) — display only
- **100% opacity**: confirmed — actively capping cruise speed
- **MAX indicator dims** to 50% when speed limit is enforcing a cap
- **Tap the sign** to confirm/cancel enforcement
- **Source label** below the sign: "OSM" / "SIGN" / "~" (inference)

## Cereal messages

| Message | Direction | Frequency | Description |
|---------|-----------|-----------|-------------|
| mapdOut | subscribe | 20 Hz | OSM speed limit, road context, wayRef, lanes |
| modelV2 | subscribe | 20 Hz | Lane line probs for lane count inference |
| gpsLocationExternal | subscribe | 1 Hz | GPS fix for country auto-detection |
| speedLimitState | publish | 5 Hz | Fused speed limit + source + confirmation |

## Key files

```
speedlimitd/
  plugin.json        # Plugin manifest
  speedlimitd.py     # SpeedLimitMiddleware process
  planner_hook.py    # planner.v_cruise hook
  ui_overlay.py      # HUD overlay rendering + tap handler
  speed_tables/      # Per-country speed limit TOML files
    cn.toml
    de.toml
    au.toml
```

## Params

| Param | Type | Description |
|-------|------|-------------|
| ShowSpeedLimitSign | bool | Show speed limit sign on HUD |
| MapdSpeedLimitControlEnabled | bool | Enable conditional speed control |
| MapdCurveTargetLatAccel | pills | Target lateral acceleration in curves (m/s²) |
