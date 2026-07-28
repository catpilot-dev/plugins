# speedlimitd — Conditional Speed Control

**Type**: hybrid (process + hook) | **Dependency**: mapd | **Process**: 5 Hz, only_onroad

## What it does

Fuses multiple speed limit sources into a single `speedLimitState` message and enforces confirmed limits by capping cruise speed. Even when no speed signs are visible and OSM has no data, road type inference provides a safety floor.

### Three-tier priority

| Priority | Source | Confidence | Description |
|----------|--------|------------|-------------|
| 1 (highest) | OSM maxspeed | 0.95 | From mapdOut.speedLimit |
| 2 | YOLO detection | 0.80 | Speed sign recognition (placeholder) |
| 3 (lowest) | Road type inference | 0.50 | Highway type + lane count + vision |

### Road type inference

Speed limits are inferred from:

- **OSM highway type**: motorway, trunk, primary, secondary, etc.
- **OSM wayRef**: G-roads = trunk, S-roads = primary, X-roads = secondary (China)
- **Road context**: freeway or city (from mapd). Secondary and below roads force urban context regardless of mapd output.
- **Vision lane count**: counts visible lane lines from modelV2 (inner pair confidence > 0.6, outer > 0.3). Always used instead of OSM lane count for better accuracy.

Lane count promotes road class when it suggests a higher classification:
- 6+ lanes → motorway
- 3+ lanes → trunk
- 2+ lanes → primary

### Vision speed cap

When vision confidently detects a narrow road (both inner lane lines > 0.6 confidence), speed is capped for safety:
- 1 visible lane → 30 km/h
- 2 visible lanes → 40 km/h

Only applied on secondary roads and below (not highways).

### Lateral-acceleration ownership (2026-07-28 layer contract)

speedlimitd is solely responsible for lateral acceleration via `vEgo`. Two mechanisms defend the comfort/ISO envelope:

- **Proactive curve cap** (`curvature_speed_cap`): caps speed from the model's predicted path curvature within confident vision. The target lateral acceleration is `MapdCurveTargetLatAccel` (m/s², default 1.5, clamped [1.0, 3.0]).
- **Reactive measured-a_y cap**: a backstop on the *measured* lateral accel (`a_y = vEgo · yawRate`, from `livePose`, low-passed ~0.3 s). Proactive capping cannot see late-appearing curves or plant amplification (one recorded curve measured 3.6 m/s² where vision had capped nothing). When `|a_y|` exceeds `MapdReactLatAccel` (m/s², default 2.5, clamp [1.8, 3.0], 0 disables) continuously for 0.5 s it engages a cap `v = vEgo·sqrt(threshold/|a_y|) − 1 m/s`; while engaged the cap only moves down or holds, and releases by ramping back up (~1 m/s per second) after 2 s of quiet. The ISO 3.0 m/s² defense now lives here.

Both feed the same `min()` combination as a safety-class source: they inherit the driver-gas suspend and are in the lead-override-protected class (a faster lead never lifts a curve cap). Telemetry: `reactCapEngaged`, `reactCap` (km/h), `reactLatAccel` (measured m/s²).

### Per-country speed tables

TOML files in `speed_tables/`, auto-detected from GPS coordinates:

```
speed_tables/
  cn.toml   # China — GB 5768
  de.toml   # Germany — StVO
  au.toml   # Australia — Australian Road Rules
```

Adding a new country requires only a new TOML file — no code changes.

### Confirmation & enforcement

- **Auto-confirmed on engage** — active by default when openpilot goes onroad
- Toggle via steering wheel resume button (short press) or UI speed sign tap
- Speed-dependent comfort offset: +40% (≤50 km/h), +30% (51-60 km/h), +10% (>60 km/h)
- **Lead override**: if lead vehicle travels > 10% above limit, capping is skipped

### HUD overlay

Vienna-style speed limit sign on the onroad HUD:
- 50% opacity = suggestion (unconfirmed), 100% = enforcing
- MAX indicator dims when speed limit is capping cruise
- Tap sign to confirm/cancel
- Source label: "OSM" / "SIGN" / "~" (inference)

## Hooks

| Hook | Description |
|------|-------------|
| `planner.subscriptions` | Add speedLimitState to planner |
| `planner.v_cruise` | Cap cruise speed when confirmed |
| `ui.hud_set_speed_override` | Override HUD speed display |
| `ui.render_overlay` | Speed limit sign + tap handler |
| `ui.state_subscriptions` | Subscribe to speedLimitState in UI |

## Cereal Messages

| Message | Direction | Frequency |
|---------|-----------|-----------|
| mapdOut | subscribe | 20 Hz |
| modelV2 | subscribe | 20 Hz |
| gpsLocationExternal | subscribe | 1 Hz |
| livePose | subscribe | 20 Hz |
| speedLimitState | publish | 5 Hz |

`livePose` is the (lightest, vehicle-agnostic) source of measured lateral accel — forward speed and yaw rate both come from the localizer, no car sensor or steering ratio.

## Params

| Param | Description |
|-------|-------------|
| ShowSpeedLimitSign | Show speed limit sign on HUD |
| MapdSpeedLimitControlEnabled | Enable conditional speed control |
| MapdCurveTargetLatAccel | Proactive curve-cap target lateral acceleration (m/s², default 1.5, clamp [1.0, 3.0]) |
| MapdReactLatAccel | Reactive measured-a_y cap threshold (m/s², default 2.5, clamp [1.8, 3.0], 0 disables) |
