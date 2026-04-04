# Look Ahead Lateral Control

## The Problem

Stock openpilot computes desired steering curvature from the model's prediction at `t = actuator_delay` (~0.5s). At 80 km/h, that's only 11 meters ahead — like driving while staring at the hood. Every frame of model prediction noise becomes a steering command, causing visible steering wheel oscillation (±5° on BMW E90, ~2 Hz).

This short lookahead also makes the controller sensitive to exact calibration of `steerActuatorDelay` and `latAccelFactor`. Small parameter errors are amplified frame-to-frame into steering jitter.

## The Solution

Human drivers look 2-5 seconds ahead. The model already predicts road geometry out to 10 seconds — we just need to use it.

Look Ahead Lateral replaces the stock short-lookahead curvature with one computed from a longer preview distance:

```
lookahead_t = clamp(80m / v_ego, 1.0s, 4.0s)
```

| Speed | Lookahead time | Lookahead distance |
|-------|---------------|-------------------|
| 30 km/h | 4.0s (capped) | 33m |
| 60 km/h | 4.0s (capped) | 67m |
| 80 km/h | 3.6s | 80m |
| 120 km/h | 2.4s | 80m |

The model's orientation prediction at 80m describes the road's large-scale geometry, which doesn't change frame-to-frame. Near-field noise cancels out naturally.

### Lead Vehicle Adaptation

When a lead vehicle is detected (via `radarState.leadOne`), the lookahead distance is capped to the lead's distance. Like a human driver, the controller focuses on the car ahead rather than the horizon when following:

| Scenario | Lookahead |
|----------|-----------|
| Open road | 80m (full preview) |
| Lead at 40m | 40m |
| Close following (15m) | 15m → 1.0s floor |

### Lane Changes

During lane changes, Look Ahead falls back to stock curvature for quick lateral response.

## Measured Results

Replay analysis of route 00000230 (BMW E90, 43 segments):

**Straight lane curvature noise reduction:**

| Speed | Stock (0.51s) | Look Ahead (80m) | Reduction |
|-------|--------------|-------------------|-----------|
| 40-60 km/h | 0.001022 | 0.000489 | 52% |
| 60-80 km/h | 0.000529 | 0.000392 | 26% |
| 80-120 km/h | 0.000368 | 0.000338 | 8% |

**Frame-to-frame jitter in straights: 69% reduction**

**Steering angle equivalent: 1.51° → 0.95° std (37% smoother)**

## Why It Works

1. **Far-field smoothness** — road geometry at 80m is an arc, not noise. The model's orientation prediction at 3.6s averages over a large road segment.

2. **Self-adapting with speed** — faster speed = shorter time lookahead for the same 80m. Tighter attention at highway speed, more relaxed in urban driving.

3. **Robust to calibration** — with a 3.6s lookahead, a 50ms error in actuator delay is 1.4% shift (vs 10% at 0.5s). Small errors in `latAccelFactor` produce gentle steady-state offsets that the integrator corrects, instead of frame-to-frame jitter.

4. **Pairs with rate limiting** — smooth curvature commands + symmetric torque rate limits (`STEER_DELTA_UP=0.1`, `STEER_DELTA_DOWN=0.2`) give smooth build-up into curves and gradual unwind out.

## Hook Chain

1. **look_ahead** (priority 40) — computes curvature from 80m preview
2. **lane_centering** (priority 50) — adjusts for lane offset in turns

## Configuration

Toggle in Settings > Driving > "Look Ahead Steering" (default ON).
