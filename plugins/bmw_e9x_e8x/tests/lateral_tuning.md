# BMW E90 Lateral Tuning Analysis

## Summary

The BMW E90 uses a stepper servo + hydraulic power steering system with fundamentally different characteristics from modern EPS cars. The key finding: **latAccelFactor is the single most important parameter** — stock KP schedule works correctly when feedforward is accurate.

## Final Configuration

| Parameter | Value | Source |
|-----------|-------|--------|
| steerActuatorDelay | 0.4s | lagd anchor point (initial_lag = 0.6s) |
| latAccelFactor | 4.50 | torqued raw estimate across multiple drives |
| friction | 0.13 | stable across all drives |
| longitudinalActuatorDelay | 0.5s | DCC cruise control in the loop |
| KP schedule | stock speed-dependent | reverted from adaptive — works when FF is accurate |

## Performance Comparison

| Route | latAccelFactor | Error std | Error p95 | I term mean | Oscillation |
|-------|---------------|-----------|-----------|-------------|-------------|
| 0000021b | 2.80 (old) | 0.112 | 0.239 | 0.225 | 2.0 Hz |
| 0000021e | 2.80 (old) | 0.177 | 0.371 | 0.206 | 1.5 Hz |
| 0000021f | 4.50 | 0.194 | 0.354 | 0.058 | 1.3 Hz |
| 00000222 | 4.50 | **0.080** | **0.142** | 0.144 | 2.1 Hz |

## Why latAccelFactor=4.50

- torqued raw estimate: 3.8-4.6 across drives (varies due to nonlinear hydraulic assist)
- F=2.80 caused integrator buildup (I=0.206) and oscillation
- F=4.50 gives accurate feedforward, P+I only correct residual
- The gap between statistical fit (~2.1) and working value (4.50) is due to the hydraulic system's nonlinear gain

## LiveDelay

- Stock lagd thresholds (NCC>=0.95, confidence>=0.70) are too strict for BMW
- BMW's hydraulic steering produces broad NCC peaks (0.83-0.96) and low confidence (0.40-0.55)
- LiveDelay never converges — stays at fallback 0.6s (steerActuatorDelay=0.4 + 0.2)
- The 0.6s fallback is acceptable for driving

## LiveTorqueParameters

- Stuck at 55-59% calibration — outer torque buckets [-0.5,-0.3) and [+0.3,+0.5) fill slowly
- Requires sustained hard turns in both directions to fill
- Raw estimates: F=3.8-4.6 (varies), friction=0.12-0.13 (stable)
- Once converged, will refine feedforward within ±30% of TOML value

## Steering System Characteristics

### Nonlinearity
- Torque-to-lateral-acceleration relationship is highly nonlinear
- Per-bucket least squares R² ≈ 0 within narrow torque ranges
- Overall R² ≈ 0.42 with optimal delay alignment (0.6-0.8s)
- SVD fit is unstable per-segment (range: -355 to +52)

### Delay
- CAN YawRate vs livePose: correlation -0.93 (sign convention difference), zero latency
- BMW CAN uses left-positive, openpilot uses right-positive yaw convention
- livePose adds no additional latency to yaw rate signal
- Actuator delay is purely mechanical (servo → hydraulics → rack)

### CAN Signals (message 416 "Speed" from DSC)
- `LatlAcc`: 12-bit signed, 0.025 m/s² — includes road camber/superelevation
- `YawRate`: 12-bit signed, 0.05 deg/s — same as carState.yawRate
- `VehicleSpeed`: 12-bit signed, 0.103 kph
- CAN LatlAcc is 5-10x larger than kinematic estimate (yaw_rate × v) due to road geometry

## Adaptive KP Exploration (reverted)

Explored KP = 0.08/delay formula, validated in simulation:
- Toyota 0.32s delay: KP=0.250, 19% overshoot
- BMW 0.60s delay: KP=0.133, 15% overshoot

Reverted because the real issue was latAccelFactor, not KP. With accurate feedforward, PID contributes <4% of control output — KP doesn't matter.

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
