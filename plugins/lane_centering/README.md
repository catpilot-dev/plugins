# lane_centering — Lane Centering Correction

**Type**: hook | **Hook**: `controls.curvature_correction` | **Toggle**: Settings > Driving > Lane Centering in Turns

## What it does

Corrects openpilot's tendency to hug the inside of turns. The driving model's path prediction cuts corners — up to 1.5m toward the inside edge in sharp turns. This plugin uses real-time lane line detection to measure the offset and applies a curvature correction that centers the car in the lane.

## The Problem

openpilot's `desiredCurvature` biases toward the inside of turns:

| Turn type | Inside bias |
|-----------|-------------|
| Straight | ~0 m |
| Gentle curve | +0.5 m |
| Moderate turn | +1.4 m |
| Sharp turn | +1.5 m |

## How It Works

1. Pick the higher-confidence lane line (left or right, >= 0.5 probability)
2. Estimate lane center using that lane + dynamically measured lane width
3. Compute lateral offset between car position and lane center
4. Apply curvature-dependent gain with kP compensation: `correction = -K * offset / v_ego²`
5. Rate limiting and derivative damping prevent oscillation from latcontrol_torque's speed-dependent kP

**Single-lane mode**: When entering a turn, the outside lane often becomes invisible. The plugin uses the inside lane + estimated lane width to maintain correction throughout the turn.

### Activation (hysteresis)

- **Speed-dependent threshold**: `T(v) = 0.15 + 0.01 · (v · 0.5)` m (lookahead-scaled — noise projects farther at higher speed)
- **Activates** when offset > `T(v)` (e.g. 0.20 m at 10 m/s, 0.30 m at 30 m/s)
- **Deactivates** when offset < `T(v) / 2`
- **Disabled** during lane changes, below 9 m/s, or when lane confidence < 0.5

### Safety

- Jump rejection: ignores lane center changes > 0.3 m per frame
- Smooth wind-down (1.0s tau) when deactivating
- Dynamic lane width estimation (2.5–4.5 m range, 3.5 m default)
- Fails safe: returns unmodified curvature on any data issue

## Results

92% reduction in inside bias during moderate-to-sharp turns. Correction coverage in turn entry zone: 9% → 94% after single-lane mode improvement.

## Tuning Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| K_BP | [0.002, 0.005, 0.008, 0.012, 0.020] | Curvature breakpoints (1/m) |
| K_V | [0.03, 0.35, 0.40, 0.50, 0.65] | Gain at each breakpoint |
| KD | 0.5 | Derivative damping coefficient |
| SMOOTH_TAU | 0.5 s | Correction smoothing time constant |
| WINDDOWN_TAU | 1.0 s | Deactivation smoothing time constant |

## Key Files

```
lane_centering/
  plugin.json      # Plugin manifest
  correction.py    # LaneCenteringCorrection class + hook callback
```
