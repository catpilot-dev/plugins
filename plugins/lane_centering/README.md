# Lane Centering Correction

Corrects openpilot's tendency to hug the inside of turns. The driving model's path prediction cuts corners — up to **1.5m toward the inside edge** in sharp turns compared to manual driving. This plugin uses real-time lane line detection to measure the offset and applies a curvature correction that centers the car in the lane.

## The Problem

openpilot's `desiredCurvature` output biases toward the inside of turns. The bias is curvature-dependent — barely noticeable on gentle curves, uncomfortable in sharp turns:

| Turn type | Inside bias vs manual driving |
|-----------|-------------------------------|
| Straight | ~0 m |
| Gentle curve | +0.5 m |
| Moderate turn | +1.4 m |
| Sharp turn | +1.5 m |

*Data from route comparison: manual driving (68k samples) vs openpilot (1.2k samples)*

## How It Works

1. Pick the higher-confidence lane line (left or right, >= 0.5 probability)
2. Estimate lane center using that lane + dynamically measured lane width
3. Compute lateral offset between the car's position and lane center
4. Apply curvature-dependent gain: `correction = -K * offset / v_ego²`

The correction is **curvature-gated** — only activates in turns where inside bias is actually a problem, leaving straight-road behavior untouched.

**Single-lane mode**: When entering a turn, the outside lane often becomes invisible (confidence drops below 0.5). The plugin uses the inside lane — which stays reliable throughout the turn — plus an estimated lane width to maintain correction. This solved a critical gap where earlier versions lost 90% of correction during the most important phase of a turn.

### Activation (hysteresis)

- **Activates** when curvature > 0.002 (1/m) AND offset > 0.3 m
- **Deactivates** when curvature < 0.001 (1/m) AND offset < 0.15 m
- **Disabled** during lane changes, below 9 m/s, or when lane confidence < 0.5
- **Steering only** — longitudinal planner uses raw curvature for conservative speed limiting

### Safety

- Jump rejection: ignores lane center changes > 0.3 m per frame
- Smooth wind-down (1.0s tau) when deactivating
- Dynamic lane width estimation (2.5–4.5 m range, 3.5 m default)
- Fails safe: returns unmodified curvature on any data issue

## Results

92% reduction in inside bias during moderate-to-sharp turns. In the critical turn entry zone (2–10s), correction coverage went from 9% to 94% after the single-lane mode improvement.

## Plugin Details

**Type**: hook | **Hook**: `controls.curvature_correction` | **Toggle**: `LaneCenteringCorrection` (default off)

```
lane_centering/
  plugin.json      # Plugin manifest
  correction.py    # LaneCenteringCorrection class + hook callback
```

## Tuning Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| K_BP | [0.002, 0.005, 0.008, 0.012, 0.020] | Curvature breakpoints (1/m) |
| K_V | [0.03, 0.35, 0.40, 0.50, 0.65] | Gain at each breakpoint |
| MIN_CURVATURE | 0.002 | Activation threshold (~500m radius) |
| EXIT_CURVATURE | 0.001 | Deactivation threshold (~1000m radius) |
| OFFSET_THRESHOLD | 0.3 m | Minimum offset to activate |
| SMOOTH_TAU | 0.5 s | Correction smoothing time constant |
| WINDDOWN_TAU | 1.0 s | Deactivation smoothing time constant |
