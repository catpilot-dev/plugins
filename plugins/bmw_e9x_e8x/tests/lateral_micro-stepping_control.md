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
| PLANT_GAIN | 2/v² | Vehicle dynamics: curvature response ∝ 1/v². 0.0026 at 100kph, 0.0104 at 50kph |
| MAX_STEP | 0.04167 | CAN rate limit: 0.1 Nm/10ms × 5 frames / 12 Nm max |
| STEP_PER_FRAME | 0.00208 | MAX_STEP / 20 (spread across 0.2s) |
| HYST_GAP | 0.001 | Hysteresis on curvature error — suppresses zero crossings without muting actions |
| LOOKAHEAD_T | 0.5 | Fixed 0.5s — with hysteresis, noise is already filtered; longer adds prediction error |
| MAX_LATERAL_JERK | 2.5 | m/s³ comfort limit on desired curvature rate (stock: 5.0) |
| MAX_LATERAL_ACCEL | 1.0 | m/s² comfort limit on desired curvature magnitude (stock: 3.0) |

### Curvature Sources

Self-contained — both computed directly from modelV2, no controlsd dependency:
- **desired_curvature**: `curv_from_psis(yaws, yaw_rates, vEgo, 0.5)` at t=0.5s, rate-limited by comfort jerk/accel limits
- **measured_curvature**: `curv_from_psis(yaws[1], yaw_rates[0], vEgo, 0.01)` at t≈0.01s
- Shadow torque runs continuously even when disengaged for seamless engage transition

### Error Correction Logic

```
raw_error = desired_curvature - measured_curvature
error = apply_hysteresis(raw_error, error_steady, HYST_GAP)
delta_error = error - prev_error

if error worsening (same sign, |delta_error| > 0, |error| > |prev|):
    correction = delta_error / (2/v²)         # incremental only
elif error sign changed (overshot or new curve):
    correction = error / (2/v²)              # full reset
else:
    correction = 0                           # hold, let existing torque work

step = clamp(correction, -MAX_STEP, MAX_STEP)
torque += step / 20 per CAN frame           # spread across 0.2s
```

### Plant Gain

Measured from route data: `delta_curvature[t+0.5s] / delta_torque[t]`

Filter: |delta_torque| > 0.017 (0.2 Nm / 12 Nm) per model frame to exclude noise.

Only same-sign pairs (positive torque → positive curvature change) are valid.

**Plant gain ∝ 1/v²** (R²=0.98) due to vehicle dynamics (lateral force ∝ v²). Now used directly as speed-dependent gain: PLANT_GAIN = 2/v².

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
- Lower speed → more oscillation (higher plant gain at low speed, vehicle dynamics ∝ 1/v²)

**Comparison with stock PID (route 00000230):**

| | Stock PID (00000230) | Micro-stepping (0000025d) |
|---|---|---|
| Straight err_std | 0.0544 | — |
| Straight angle_std | 2.05° | 2.3-4.5° |
| Straight osc Hz | 1.9 | 8-16 |
| Overall err_std | 0.066 | 0.00167 |

**Lane change performance (single, non-consecutive, laneChangeStarting→off):**

| LC | Direction | Speed | Duration | Error MAE |
|----|-----------|-------|----------|-----------|
| 1 | left | 70 km/h | 7.2s | 0.00109 |
| 2 | right | 87 km/h | 6.8s | 0.00093 |
| 5 | left | 41 km/h | 6.0s | 0.00157 |
| 6 | left | 69 km/h | 15.3s | 0.00091 |
| 7 | left | 76 km/h | 3.8s | 0.00073 |
| 10 | right | 70 km/h | 6.4s | 0.00073 |
| 11 | right | 47 km/h | 6.0s | 0.00199 |
| 12 | left | 60 km/h | 6.0s | 0.00084 |
| 15 | right | 88 km/h | 6.0s | 0.00125 |
| 16 | left | 48 km/h | 4.9s | 0.00089 |

- Mean MAE: **0.00099**, Best: **0.00073** (76 km/h), Worst: 0.00199 (47 km/h)
- Mean duration: 6.7s — 0.2s spreading is only 3% of total maneuver
- Near perfect path following, no abrupt jerk
- Higher speed → tighter tracking (smaller curvature errors at speed)

**TODO:**
- [x] ~~Test drive with 0.2s spreading~~ → 0.1s tested in 0000025f, reverted to 0.2s (lane change smoothness)
- [x] ~~Tune straight-lane damping~~ → deadzone replaced by hysteresis 0.0004 (89% fewer zero crossings, no phase lag; 4-frame window tried but adds phase lag)
- [x] ~~Evaluate with look_ahead plugin re-enabled~~ → fixed 1.0s lookahead (39% fewer actions vs stock 0.5s, 3% more MAE). Dynamic confidence-based (1.0-3.0s) tested but model prediction error dominates beyond 1.0s. 1.5s adds prediction error without reducing zero crossings.
- [ ] Test PLANT_GAIN sensitivity: try 0.004 (median) vs 0.006 (mean)
- [ ] Test drive with hysteresis + 1.0s lookahead combined (next route)

### Route 0000025f — Second Test Drive

**Config**: Incremental P controller, PLANT_GAIN=0.006, SPREAD_FRAMES=10 (0.1s), DEADZONE=0.0004, look_ahead plugin disabled

**Changes from 0000025d**: SPREAD_FRAMES 20→10 (0.2s→0.1s), DEADZONE 0.0001→0.0004

**Overall: 135,105 engaged samples**

| Metric | Value |
|--------|-------|
| Error MAE | 0.00033 |
| Error P95 | 0.00127 |
| Error P99 | 0.00296 |
| Correlation (desired vs measured) | +0.948 |
| Torque range | [-0.805, +1.000] |
| Torque MAE | 0.081 |
| Speed range | 8.7 - 29.3 m/s |

**By speed:**

| Speed | MAE | Std |
|-------|-----|-----|
| 5-10 m/s (18-36 km/h) | 0.00048 | 0.00066 |
| 10-15 m/s (36-54 km/h) | 0.00078 | 0.00105 |
| 15-20 m/s (54-72 km/h) | 0.00033 | 0.00044 |
| 20-25 m/s (72-90 km/h) | 0.00016 | 0.00027 |
| 25-35 m/s (90-126 km/h) | 0.00014 | 0.00018 |

**By curvature:**

| Curvature | MAE | Std |
|-----------|-----|-----|
| < 0.0005 (straight) | 0.00017 | 0.00026 |
| 0.0005 - 0.001 | 0.00034 | 0.00041 |
| 0.001 - 0.003 | 0.00068 | 0.00084 |
| 0.003 - 0.010 | 0.00105 | 0.00108 |
| > 0.010 | 0.00114 | 0.00109 |

**Straight-lane oscillation (|desired| < 0.001):**

| Speed | Steer std | Osc freq | Error MAE |
|-------|-----------|----------|-----------|
| 40-55 km/h | 2.64° | 1.9 Hz | 0.00043 |
| 55-70 km/h | 2.03° | 2.2 Hz | 0.00021 |
| 70-80 km/h | 2.04° | 2.6 Hz | 0.00015 |
| 80-90 km/h | 2.15° | 2.6 Hz | 0.00013 |
| 90-100 km/h | 1.28° | 2.4 Hz | 0.00013 |

**Lane change performance (23 lane changes, laneChangeStarting→off):**

| LC | Direction | Speed | Duration | Error MAE |
|----|-----------|-------|----------|-----------|
| 1 | left | 61 km/h | 6.1s | 0.00088 |
| 2 | right | 64 km/h | 6.1s | 0.00058 |
| 3 | left | 83 km/h | 5.4s | 0.00054 |
| 4 | left | 69 km/h | 6.0s | 0.00064 |
| 5 | right | 74 km/h | 6.1s | 0.00052 |
| 6 | right | 38 km/h | 6.1s | 0.00216 |
| 7 | right | 44 km/h | 6.1s | 0.00147 |
| 8 | right | 79 km/h | 6.0s | 0.00070 |
| 9 | left | 56 km/h | 6.1s | 0.00085 |
| 10 | left | 52 km/h | 1.7s | 0.00175 |
| 11 | left | 90 km/h | 6.0s | 0.00045 |
| 12 | left | 103 km/h | 5.7s | 0.00031 |
| 13 | right | 80 km/h | 6.1s | 0.00044 |
| 14 | left | 100 km/h | 6.0s | 0.00022 |
| 15 | right | 104 km/h | 5.8s | 0.00031 |
| 16 | left | 77 km/h | 6.1s | 0.00047 |
| 17 | right | 36 km/h | 6.1s | 0.00282 |
| 18 | left | 67 km/h | 6.0s | 0.00078 |
| 19 | left | 69 km/h | 6.0s | 0.00066 |
| 20 | right | 86 km/h | 6.0s | 0.00030 |
| 21 | left | 88 km/h | 6.0s | 0.00037 |
| 22 | right | 88 km/h | 6.0s | 0.00030 |
| 23 | right | 88 km/h | 2.7s | 0.00055 |

- Mean MAE: **0.00079**, Best: **0.00022** (100 km/h left), Worst: 0.00282 (36 km/h right)
- Mean duration: 5.7s
- Higher speed → dramatically tighter tracking (consistent with 0000025d)

**Comparison with 0000025d:**

| | 0000025d (0.2s, DZ=0.0001) | 0000025f (0.1s, DZ=0.0004) | Change |
|---|---|---|---|
| Samples | 24,422 | 135,105 | 5.5× more data |
| Error MAE | 0.00122 | 0.00033 | **-73%** |
| Error P95 | 0.00350 | 0.00127 | **-64%** |
| Error P99 | 0.00549 | 0.00296 | **-46%** |
| Correlation | +0.812 | +0.948 | **+0.136** |
| Straight osc Hz | 8-16 | 1.9-2.6 | **-80%** |
| Straight steer std | 2.3-4.5° | 1.3-2.6° | **-40%** |
| LC mean MAE | 0.00099 | 0.00079 | **-20%** |
| Torque MAE | 0.144 | 0.081 | **-44%** |

**Observations:**
- DEADZONE 4× increase eliminated high-frequency oscillation (8-16Hz → 2Hz) — the dominant issue from 0000025d
- Faster spreading (0.1s vs 0.2s) improved responsiveness without introducing jerk
- Torque hit 1.000 max (saturation) vs 0.911 in 0000025d — tighter curves attempted
- Low-speed (<55 km/h) lane changes remain weakest: MAE 0.00147-0.00282 vs <0.00055 at 80+ km/h

### Post-0000025f Tuning (offline analysis on route data)

**Spreading**: Reverted 10→20 frames (0.1s→0.2s). 0.1s caused abrupt lane changes despite better straight-lane metrics.

**Noise filter comparison** (straights, 1.5s lookahead):
- Deadzone 0.0004: mutes actions (204) but zero crossings unchanged (2294)
- 4-frame window: adds phase lag, increases actions (24213) — worse
- **Hysteresis 0.0004**: 89% fewer zero crossings (2294→242), 1057 actions — no phase lag

**Lookahead comparison** (straights, route 00000263 with hysteresis 0.001):

| Metric | 0.5s | 0.75s | 1.0s | 1.5s |
|---|---|---|---|---|
| Zero crossings | **54** | 94 | 110 | 122 |
| Total actions | **235** | 470 | 633 | 751 |
| MAE | **0.000218** | 0.000256 | 0.000295 | 0.000351 |

With hysteresis 0.001, noise is already filtered — longer lookahead only adds model prediction error. 0.5s (stock timing) is optimal.

**Current config**: SPREAD_FRAMES=20, PLANT_GAIN=2/v², hysteresis 0.001, lookahead 0.5s, comfort limits (jerk 2.5, accel 1.0), self-contained curvature, shadow torque

### Route 00000262 — Third Test Drive (hysteresis + 1.0s lookahead)

**Config**: Incremental P controller, PLANT_GAIN=0.006, SPREAD_FRAMES=20 (0.2s), hysteresis 0.0004, look_ahead 1.0s fixed

**Overall: 115,826 engaged samples**

| Metric | Value |
|--------|-------|
| Error MAE | 0.00036 |
| Error P95 | 0.00155 |
| Error P99 | 0.00337 |
| Correlation (desired vs measured) | +0.962 |
| Torque range | [-0.856, +0.630] |
| Torque MAE | 0.080 |
| Speed range | 9.6 - 25.0 m/s |

**By speed:**

| Speed | MAE | Std |
|-------|-----|-----|
| 5-10 m/s (18-36 km/h) | 0.00236 | 0.00186 |
| 10-15 m/s (36-54 km/h) | 0.00086 | 0.00102 |
| 15-20 m/s (54-72 km/h) | 0.00036 | 0.00052 |
| 20-25 m/s (72-90 km/h) | 0.00018 | 0.00028 |

**By curvature:**

| Curvature | MAE | Std |
|-----------|-----|-----|
| < 0.0005 (straight) | 0.00018 | 0.00031 |
| 0.0005 - 0.001 | 0.00036 | 0.00051 |
| 0.001 - 0.003 | 0.00069 | 0.00077 |
| 0.003 - 0.010 | 0.00093 | 0.00102 |
| > 0.010 | 0.00232 | 0.00185 |

**Straight-lane oscillation (|desired| < 0.001):**

| Speed | Steer std | Osc freq | Error MAE |
|-------|-----------|----------|-----------|
| 40-55 km/h | 3.76° | 2.0 Hz | 0.00057 |
| 55-70 km/h | 2.39° | 2.3 Hz | 0.00026 |
| 70-80 km/h | 2.00° | 2.5 Hz | 0.00016 |
| 80-90 km/h | 1.85° | 2.6 Hz | 0.00015 |

**Lane change performance (26 lane changes):**

- Mean MAE: **0.00080**, Best: **0.00039** (87 km/h right), Worst: 0.00152 (55 km/h left)
- Mean duration: 5.8s

**Comparison across all routes:**

| | 0000025d | 0000025f | 00000262 |
|---|---|---|---|
| Config | DZ=0.0001, no LA | DZ=0.0004, no LA | Hyst 0.0004, LA 1.0s |
| Error MAE | 0.00122 | 0.00033 | 0.00036 |
| Error P95 | 0.00350 | 0.00127 | 0.00155 |
| Correlation | +0.812 | +0.948 | **+0.962** |
| Straight osc Hz | 8-16 | 1.9-2.6 | 2.0-2.6 |
| LC mean MAE | 0.00099 | 0.00079 | 0.00080 |
| Driving feel | oscillation on straights | smooth, abrupt LC (10-frame) | smooth straights + smooth LC |

**Observations:**
- Best correlation yet (+0.962) — hysteresis + 1.0s lookahead complement each other
- No torque saturation (max 0.856 vs 1.0 limit)
- Low-speed (<55 km/h) lane changes remain the weakest area

### Route 00000263 — Fourth Test Drive (self-contained, comfort limits, PG=0.004)

**Config**: Self-contained curvature (both from modelV2), PLANT_GAIN=0.004 fixed, SPREAD_FRAMES=20, hysteresis 0.001, lookahead 1.0s→0.5s (curves), comfort limits (jerk 2.5 m/s³, accel 1.0 m/s²)

**Changes from 00000262**: Self-contained curvature (no controlsd dependency), PLANT_GAIN 0.006→0.004, hysteresis 0.0004→0.001, comfort jerk/accel limits added, shadow torque for seamless engage

**Overall: 91,368 engaged samples**

| Metric | Value |
|--------|-------|
| Error MAE | 0.00042 |
| Error P95 | 0.00144 |
| Error P99 | 0.00249 |
| Correlation (desired vs measured) | +0.945 |
| Torque range | [-0.684, 1.000] |
| Torque MAE | 0.089 |
| Speed range | 8.1 - 24.4 m/s |

**By speed:**

| Speed | MAE | Std |
|-------|-----|-----|
| 5-10 m/s (18-36 km/h) | 0.00109 | 0.00131 |
| 10-15 m/s (36-54 km/h) | 0.00065 | 0.00065 |
| 15-20 m/s (54-72 km/h) | 0.00031 | 0.00042 |
| 20-25 m/s (72-90 km/h) | 0.00017 | 0.00023 |

**By curvature:**

| Curvature | MAE | Std |
|-----------|-----|-----|
| < 0.0005 (straight) | 0.00022 | 0.00028 |
| 0.0005 - 0.001 | 0.00036 | 0.00038 |
| 0.001 - 0.003 | 0.00074 | 0.00071 |
| 0.003 - 0.010 | 0.00094 | 0.00099 |

**Straight-lane oscillation (|desired| < 0.001):**

| Speed | Steer std | Osc freq | Error MAE |
|-------|-----------|----------|-----------|
| 40-55 km/h | 2.80° | 2.2 Hz | 0.00042 |
| 55-70 km/h | 1.88° | 2.6 Hz | 0.00024 |
| 70-80 km/h | 1.62° | 2.2 Hz | 0.00015 |
| 80-90 km/h | 1.89° | 2.8 Hz | 0.00013 |

**Lane change performance (12 lane changes):**

- Mean MAE: **0.00072**, Best: **0.00029** (85 km/h right), Worst: 0.00123 (50 km/h right)
- Mean duration: 6.0s

**Comparison 00000262 vs 00000263:**

| | 00000262 | 00000263 | Change |
|---|---|---|---|
| Config | Hyst 0.0004, PG 0.006, controlsd | Hyst 0.001, PG 0.004, self-contained | |
| Error MAE | 0.00036 | 0.00042 | +17% (gentler controller) |
| Error P95 | 0.00155 | 0.00144 | **-7%** |
| Error P99 | 0.00337 | 0.00249 | **-26%** |
| LC mean MAE | 0.00080 | **0.00072** | **-10%** |
| Steer std 70-80kph | 2.00° | **1.62°** | **-19%** |
| Steer std 40-55kph | 3.76° | **2.80°** | **-26%** |

**Observations:**
- Lane changes much smoother — comfort limits (1.0 m/s², 2.5 m/s³) + lower plant gain
- P99 error dropped 26% — fewer extreme corrections
- Steer std improved across all speeds — hysteresis 0.001 less busy
- Tight turns smooth — lower plant gain ramps torque more gradually
- Slight oscillation remains at 50 kph — vehicle dynamics: same torque produces more curvature change at low speed (∝ 1/v²) → addressed by speed-dependent plant gain (2/v²) in next route
- MAE slightly higher (+17%) — expected, controller is intentionally gentler
- Torque hit 1.000 saturation once — worth monitoring

**Post-route analysis** (offline simulation on route 00000263 data):
- Lookahead 0.5s optimal: 54 ZC vs 110 ZC at 1.0s on straights. With hysteresis 0.001, noise already filtered — longer lookahead only adds prediction error.
- Speed-dependent plant gain (2/v²) equalizes correction across speeds. Fixed 0.004 over-corrected 4x at 40 kph vs 80 kph.

**Current config (post-263 tuning)**: SPREAD_FRAMES=20, PLANT_GAIN=2/v², hysteresis 0.001, lookahead 0.5s fixed, comfort limits (jerk 2.5, accel 1.0), self-contained curvature from modelV2, shadow torque

**Evaluation scripts**: `tests/eval_micro_stepping.py` (curvature metrics, speed bins, oscillation, lane changes), `tests/analyze_lateral.py` (PID term decomposition, per-segment timeline, calibration convergence)
