# BMW E90 Lateral Micro-Stepping Control

## Overview

Incremental P torque controller that corrects curvature error through micro-stepping. Instead of computing absolute torque from a plant model, the controller steps torque proportionally to the curvature error between desired (t+0.5s) and measured (t≈0), spreading each correction across 20 CAN frames (0.2s) for smooth response.

### Key Design Principles

1. **Same-source curvature**: Both desired and measured use `curv_from_psis` formula from the model trajectory — no cross-source bias
2. **Delta-error correction**: Only correct new error (worsening); hold torque when improving — respects actuator delay
3. **Micro-stepping**: Spread correction across 20 CAN frames (0.2s, human reaction time) instead of instant
4. **Rate-limited**: MAX_STEP = 0.5 Nm per model frame (CAN safety), STEP_PER_FRAME = 0.025 Nm per CAN frame

### Controller Parameters

| Parameter | Value | Source |
|-----------|-------|--------|
| PLANT_GAIN | 0.006 | Route data median (delta_curv / delta_torque at +0.5s) |
| MAX_STEP | 0.04167 | CAN rate limit: 0.1 Nm/10ms × 5 frames / 12 Nm max |
| STEP_PER_FRAME | 0.00208 | MAX_STEP / 20 (spread across 0.2s) |
| DEADZONE | 0.0001 | Noise filter on curvature error |

### Curvature Sources

- **desired_curvature**: from controlsd → `curv_from_psis(yaws, yaw_rates, vEgo, lat_delay + DT_MDL)` at t≈+0.5s, safety-limited by `clip_curvature`
- **measured_curvature**: from modelV2 → `curv_from_psis(yaws[1], yaw_rates[0], vEgo, 0.01)` at t≈0.01s

### Error Correction Logic

```
error = desired_curvature - measured_curvature
delta_error = error - prev_error

if error worsening (same sign, |delta_error| > DEADZONE, |error| > |prev|):
    correction = delta_error / PLANT_GAIN    # incremental only
elif error sign changed (overshot or new curve):
    correction = error / PLANT_GAIN          # full reset
else:
    correction = 0                           # hold, let existing torque work

step = clamp(correction, -MAX_STEP, MAX_STEP)
torque += step / 20 per CAN frame           # spread across 0.2s
```

### Plant Gain

Measured from route data: `delta_curvature[t+0.5s] / delta_torque[t]`

Filter: |delta_torque| > 0.017 (0.2 Nm / 12 Nm) per model frame to exclude noise.

Only same-sign pairs (positive torque → positive curvature change) are valid.

**Plant gain ∝ 1/v²** (R²=0.98) due to Servotronic hydraulic assist, but with micro-stepping the rate limiter dominates, so a fixed median value is used.

| Percentile | Gain | Inverse |
|---|---|---|
| P10 | 0.0007 | 1429 |
| P25 | 0.0020 | 500 |
| Median | 0.0046 | 216 |
| Mean | 0.0060 | 167 |
| P75 | 0.0080 | 125 |
| P90 | 0.0106 | 94 |

**By speed (from all routes):**

| Speed | Gain | Inverse |
|---|---|---|
| 10-15 m/s | 0.015 | 65 |
| 15-20 m/s | 0.008 | 133 |
| 20-25 m/s | 0.003 | 345 |
| 25-35 m/s | 0.002 | 461 |

## Test Drives

### Route 0000025d — First Test Drive

**Config**: Incremental P controller with delta-error, fixed PLANT_GAIN=0.006, look_ahead plugin disabled, 20-frame spreading

**Overall: 24,422 engaged samples**

| Metric | Value |
|--------|-------|
| Error MAE | 0.00122 |
| Error P95 | 0.00350 |
| Error P99 | 0.00549 |
| Correlation (desired vs measured) | +0.812 |
| Torque range | [-0.762, +0.911] |
| Torque MAE | 0.144 |
| Speed range | 8.3 - 26.9 m/s |

**By speed:**

| Speed | MAE | Std |
|-------|-----|-----|
| 5-10 m/s (30-36 km/h) | 0.00289 | 0.00346 |
| 10-15 m/s (36-54 km/h) | 0.00160 | 0.00203 |
| 15-20 m/s (54-72 km/h) | 0.00105 | 0.00135 |
| 20-25 m/s (72-90 km/h) | 0.00071 | 0.00090 |
| 25-35 m/s (90-126 km/h) | 0.00054 | 0.00069 |

**By curvature:**

| Curvature | MAE | Std |
|-----------|-----|-----|
| < 0.0005 (straight) | 0.00065 | 0.00092 |
| 0.0005 - 0.001 | 0.00086 | 0.00113 |
| 0.001 - 0.003 | 0.00139 | 0.00168 |
| 0.003 - 0.010 | 0.00224 | 0.00273 |
| > 0.010 | 0.00214 | 0.00262 |

**Straight-lane oscillation (|desired| < 0.001):**

| Speed | Steer std | Osc freq | Error MAE |
|-------|-----------|----------|-----------|
| 48 km/h | 4.45° | 15.7 Hz | 0.00110 |
| 65 km/h | 3.52° | 8.3 Hz | 0.00079 |
| 76 km/h | 2.70° | 10.2 Hz | 0.00059 |
| 83 km/h | 2.29° | 8.3 Hz | 0.00047 |
| 86 km/h | 3.51° | 11.3 Hz | 0.00068 |
| 91 km/h | 2.38° | 9.3 Hz | 0.00045 |

**Observations:**
- Lane changing: near perfect, no abrupt jerk
- Tight curves: no torque saturation (max ±0.91 vs ±1.0 limit)
- Straight lanes: oscillation 8-16 Hz at 2-4.5° std — needs damping
- Lower speed → more oscillation (Servotronic higher gain at low speed)

**Comparison with stock PID (route 00000230):**

| | Stock PID (00000230) | Micro-stepping (0000025d) |
|---|---|---|
| Straight err_std | 0.0544 | — |
| Straight angle_std | 2.05° | 2.3-4.5° |
| Straight osc Hz | 1.9 | 8-16 |
| Overall err_std | 0.066 | 0.00167 |

**TODO:**
- [ ] Tune straight-lane damping — consider torque decay when error is within deadzone
- [ ] Evaluate with look_ahead plugin re-enabled (noise reduction on straights)
- [ ] Test PLANT_GAIN sensitivity: try 0.004 (median) vs 0.006 (mean)
- [ ] Measure plant gain online for adaptive tuning
