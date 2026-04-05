# BMW E90 Lateral Tuning Analysis

## Summary

The BMW E90 uses a stepper servo + hydraulic power steering system with fundamentally different characteristics from modern EPS cars. The key finding: **latAccelFactor is the single most important parameter** — stock KP schedule works correctly when feedforward is accurate.

Three additional improvements eliminated straight-line steering oscillation:
1. **Look Ahead Lateral Control** — 50m preview curvature instead of 0.5s actuator delay
2. **Symmetric torque rate limits** — STEER_DELTA_DOWN reduced from 1.0 to 0.2 Nm/10ms
3. **Steering angle offset** — auto-calibrated SZL sensor zero offset (0.30°)

## Current Configuration

| Parameter | Value | Source |
|-----------|-------|--------|
| steerActuatorDelay | 0.26s | lagd (total 0.46s with +0.2s smooth) |
| latAccelFactor | 4.50 | torqued raw estimate across multiple drives |
| friction | 0.13 | stable across all drives |
| longitudinalActuatorDelay | 0.5s | DCC cruise control in the loop |
| KP schedule | stock speed-dependent | works when FF is accurate |
| STEER_DELTA_UP | 0.1 Nm/10ms | stock |
| STEER_DELTA_DOWN | 0.2 Nm/10ms | reduced from 1.0 for smooth unwinding |
| Look Ahead distance | 50m | ~2.25s at 80 kph (plugin: look_ahead) |
| Steer angle offset | auto (0.30°) | median of straight highway samples |

## Performance History

### Overall

| Route | Config | Error std | Error p95 | I mean | Osc Hz |
|-------|--------|-----------|-----------|--------|--------|
| 0000021b | F=2.80, delay=0.40, DD=1.0 | 0.112 | 0.239 | 0.225 | 2.0 |
| 0000021e | F=2.80, delay=0.40, DD=1.0 | 0.177 | 0.371 | 0.206 | 1.5 |
| 0000021f | F=4.50, delay=0.40, DD=1.0 | 0.194 | 0.354 | 0.058 | 1.3 |
| 00000222 | F=4.50, delay=0.40, DD=1.0 | **0.080** | **0.142** | 0.144 | 2.1 |
| 0000022f | F=4.50, delay=0.26, DD=1.0 | 0.080 | 0.152 | 0.230 | 1.9 |
| 00000230 | F=4.50, delay=0.26, DD=0.2 | 0.066 | 0.125 | 0.244 | 1.8 |
| 00000235 | DD=0.2, LA 80m | 0.090 | 0.129 | 0.145 | 2.2 |
| 00000236 | DD=0.2, LA 50m, offset=0.30 | 0.099 | 0.177 | 0.185 | 2.0 |

### Straight Lane Performance

| Route | Config | err_std | angle_std | osc Hz |
|-------|--------|---------|-----------|--------|
| 00000230 | stock curvature | 0.0544 | 2.05° | 1.9 |
| 00000235 | LA 80m | 0.0531 | 1.47° | 2.2 |
| 00000236 | LA 50m + offset | 0.0636 | 1.64° | 2.2 |

### Straight Lane by Speed

| Speed | 00000230 (stock) | 00000235 (LA 80m) | 00000236 (LA 50m) |
|-------|-----------------|-------------------|-------------------|
| 40-60 kph | 0.058 | 0.047 | 0.063 |
| 60-80 kph | 0.052 | 0.052 | 0.065 |
| 80-120 kph | 0.057 | 0.056 | 0.063 |
| angle_std 80-120 | — | 1.41° | **1.19°** |

## Look Ahead Lateral Control

Standalone plugin `plugins/look_ahead/` — platform-independent.

**Problem**: Stock openpilot computes curvature at t=actuator_delay (~0.5s = 11m at 80 kph). Frame-to-frame model noise becomes steering jitter.

**Solution**: `lookahead_t = clamp(50m / v_ego, 1.0s, 3.0s)`. Falls back to stock in curves (curvature > 0.002) so lane centering can work accurately.

**Lead vehicle**: Caps lookahead to `radarState.leadOne.dRel` when lead is closer than 50m.

**Measured noise reduction** (replay of route 00000230 data with 80m lookahead):
- 40-60 kph: 52% reduction
- 60-80 kph: 26% reduction
- Frame-to-frame jitter in straights: 69% reduction

**Why it works**: At 3.6s lookahead, a 50ms actuator delay error is 1.4% shift (vs 10% at 0.5s). The controller becomes robust to calibration uncertainty.

## STEER_DELTA_DOWN

Reduced from 1.0 to 0.2 Nm/10ms (both `values.py` and `safety/bmw.h`).

| | DELTA_UP | DELTA_DOWN | Full release | Down/Up ratio |
|---|---|---|---|---|
| Old | 0.1 | 1.0 | 120ms | 10x |
| New | 0.1 | 0.2 | 600ms | 2x |

Most platforms use 1.5-2.5x ratio. The old 10x was the most aggressive in the fleet. Near-symmetric rate limits prevent sawtooth oscillation: the controller can't dump torque 10x faster than it applies it.

## Steering Angle Offset

The SZL module's zero doesn't match the rack's center position. `paramsd`'s Kalman filter can't learn this because its zero-observation regularizer on `ANGLE_OFFSET_FAST` assumes the sensor is factory-calibrated.

**Auto-calibration** (in `look_ahead` plugin):
- Collects `steeringAngleDeg` samples on straight highways (speed > 15 m/s, curvature < 0.0005)
- Computes median at route end (requires 3000+ samples = ~30s of straight highway)
- Persists to `look_ahead/data/SteerAngleOffset`, publishes via plugin bus
- BMW carstate subscribes and applies the offset
- Clamped to ±3° for safety

**Result**: Converged to 0.30° (initially seeded at 0.80°). I-term mean dropped from 0.244 to 0.185.

## Why latAccelFactor=4.50

- torqued raw estimate: 3.8-4.6 across drives (varies due to nonlinear hydraulic assist)
- F=2.80 caused integrator buildup (I=0.206) and oscillation
- F=4.50 gives accurate feedforward, P+I only correct residual
- The gap between statistical fit (~2.1) and working value (4.50) is due to the hydraulic system's nonlinear gain

## LiveDelay

- Stock lagd thresholds (NCC>=0.95, confidence>=0.70) are too strict for BMW
- BMW's hydraulic steering produces broad NCC peaks (0.83-0.96) and low confidence (0.40-0.55)
- LiveDelay never converges — stays at fallback 0.46s (steerActuatorDelay=0.26 + 0.2)
- With Look Ahead, exact delay calibration matters much less

## LiveTorqueParameters

- At 63% calibration as of route 00000236
- Raw estimates: F=3.96 (trending up toward 4.50), friction=0.15 (stable)
- Once converged, will refine feedforward within ±30% of TOML value

## Steering System Characteristics

### Nonlinearity
- Torque-to-lateral-acceleration relationship is highly nonlinear
- Per-bucket least squares R^2 ~ 0 within narrow torque ranges
- Overall R^2 ~ 0.42 with optimal delay alignment (0.6-0.8s)
- SVD fit is unstable per-segment (range: -355 to +52)

### Delay
- CAN YawRate vs livePose: correlation -0.93 (sign convention difference), zero latency
- BMW CAN uses left-positive, openpilot uses right-positive yaw convention
- livePose adds no additional latency to yaw rate signal
- Actuator delay is purely mechanical (servo -> hydraulics -> rack)

### CAN Signals (message 416 "Speed" from DSC)
- `LatlAcc`: 12-bit signed, 0.025 m/s^2 — includes road camber/superelevation
- `YawRate`: 12-bit signed, 0.05 deg/s — same as carState.yawRate
- `VehicleSpeed`: 12-bit signed, 0.103 kph
- CAN LatlAcc is 5-10x larger than kinematic estimate (yaw_rate x v) due to road geometry

## Analysis Scripts

All scripts run on C3 device against recorded route data:

| Script | Purpose |
|--------|---------|
| `analyze_lateral.py` | Comprehensive per-segment lateral performance |
| `check_torque.py` | LiveTorqueParameters status and bucket distribution |
| `eval_buckets.py` | Per-segment torque bucket distribution and piecewise slopes |
| `eval_svd.py` | Per-segment SVD fit (torqued algorithm) |
| `eval_delayed.py` | Torque vs lat_accel with delay sweep (livePose) |
| `eval_piecewise.py` | Per-bucket least squares fit |
| `eval_can_la.py` | CAN LatlAcc vs livePose comparison |
| `eval_can_vs_pose.py` | CAN vs pose lateral acceleration |
| `eval_yawrate.py` | CAN YawRate vs livePose cross-correlation |
| `replay_lateral.py` | Route lateral performance replay |
| `replay_lanechange.py` | Lane change behavior replay |
| `sim_step_response.py` | PID step response simulation |
| `sim_pid_optimize.py` | KP/KI optimization for BMW delay |
| `sim_adaptive_gains.py` | Universal adaptive gain formula design |
| `validate_adaptive_gains.py` | Cross-platform validation against CI routes |

Usage: `ssh c3 "cd /data/openpilot && source /usr/local/venv/bin/activate && PYTHONPATH=/data/openpilot python3 /data/tmp/<script>.py"`
