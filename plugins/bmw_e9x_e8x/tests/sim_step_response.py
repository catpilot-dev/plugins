#!/usr/bin/env python3
"""Step response simulation: stock vs BMW PID, with and without delay compensation.

Models the lateral control loop as:
  Controller (PID + friction FF) → Plant delay (τ) → First-order plant → Measurement

Compares:
  A) Naive PID:  error = desired_now - actual_now
  B) Delay-compensated PID: error = desired_τ_ago - actual_now  (matches latcontrol_torque.py)
"""
import numpy as np
from collections import deque

DT = 0.01
SIM_TIME = 3.0
STEPS = int(SIM_TIME / DT)

# Stock PID schedule
INTERP_SPEEDS = [1, 1.5, 2.0, 3.0, 5, 7.5, 10, 15, 30]
KP_INTERP = [250, 120, 65, 30, 11.5, 5.5, 3.5, 2.0, 0.8]
STOCK_KI = 0.15

# BMW flat PID
BMW_KP = 0.8
BMW_KI = 0.15

# Plant parameters
FRICTION = 0.13
FRICTION_THRESHOLD = 0.2
PLANT_TAU = 0.08


def get_stock_kp(v_ego):
  return float(np.interp(v_ego, INTERP_SPEEDS, KP_INTERP))


def get_friction(error, friction):
  if abs(error) < FRICTION_THRESHOLD:
    return error * friction / FRICTION_THRESHOLD
  return friction * np.sign(error)


def simulate(kp, ki, plant_delay_s, compensated_delay_s=0.0, step_magnitude=1.0):
  """Simulate step response.

  Args:
    plant_delay_s: actual plant delay
    compensated_delay_s: delay used for error compensation (0 = naive, >0 = delay-compensated)
  """
  plant_delay_frames = int(round(plant_delay_s / DT))
  comp_delay_frames = int(round(compensated_delay_s / DT)) + 1 if compensated_delay_s > 0 else 0

  integral = 0.0
  plant_state = 0.0
  output_buffer = [0.0] * (plant_delay_frames + 1)
  request_buffer = deque([0.0] * max(comp_delay_frames + 1, 1), maxlen=max(comp_delay_frames + 1, 1))
  measurement = 0.0

  times, desired_hist, actual_hist, torque_hist, error_hist = [], [], [], [], []

  for i in range(STEPS):
    t = i * DT
    desired = step_magnitude if t >= 0.2 else 0.0

    # Delay-compensated error: compare against what we asked for τ ago
    if comp_delay_frames > 0:
      setpoint = request_buffer[-comp_delay_frames] if comp_delay_frames <= len(request_buffer) else 0.0
    else:
      setpoint = desired

    error = setpoint - measurement
    integral += error * DT
    integral = np.clip(integral, -1.0, 1.0)

    # Feedforward uses current desired (not delayed) + friction on error
    ff = desired + get_friction(error, FRICTION)
    pid_output = ff + kp * error + ki * integral

    # Record desired in request buffer (for delay compensation)
    request_buffer.append(desired)

    # Plant: pure delay → first-order
    output_buffer.append(pid_output)
    delayed_input = output_buffer.pop(0)
    plant_state += (delayed_input - plant_state) * DT / PLANT_TAU
    measurement = plant_state

    times.append(t)
    desired_hist.append(desired)
    actual_hist.append(measurement)
    torque_hist.append(pid_output)
    error_hist.append(error)

  return times, desired_hist, actual_hist, torque_hist, error_hist


def metrics(actual, desired, step_idx):
  post = actual[step_idx:]
  target = desired[step_idx]
  if target == 0:
    return 0, 0, 0

  peak = max(post)
  overshoot = (peak - target) / target * 100

  t90 = next((i for i, v in enumerate(post) if v >= 0.9 * target), len(post) - 1)
  rise_time = t90 * DT

  settled_idx = 0
  for i in range(len(post) - 1, -1, -1):
    if abs(post[i] - target) > 0.05 * target:
      settled_idx = i + 1
      break
  settling_time = settled_idx * DT

  return rise_time, overshoot, settling_time


def main():
  speeds_kph = [30, 50, 60, 80, 100, 120]
  plant_delay = 0.4

  configs = [
    ("Stock naive",           lambda v: get_stock_kp(v), STOCK_KI, 0.0),
    ("Stock delay-comp",      lambda v: get_stock_kp(v), STOCK_KI, plant_delay),
    ("BMW KP=0.8 naive",      lambda v: BMW_KP,          BMW_KI,   0.0),
    ("BMW KP=0.8 delay-comp", lambda v: BMW_KP,          BMW_KI,   plant_delay),
  ]

  print(f"Plant: first-order tau={PLANT_TAU}s, delay={plant_delay}s, friction={FRICTION}")
  print(f"\n{'Config':<26s} | {'Speed':>5s} | {'KP':>5s} | {'Rise':>6s} | {'Overshoot':>9s} | {'Settle':>7s}")
  print(f"{'-'*26} | {'-'*5} | {'-'*5} | {'-'*6} | {'-'*9} | {'-'*7}")

  for name, kp_fn, ki, comp_delay in configs:
    for kph in speeds_kph:
      v = kph / 3.6
      kp = kp_fn(v)
      _, d, a, _, _ = simulate(kp, ki, plant_delay, comp_delay)
      step_idx = next(i for i, x in enumerate(d) if x > 0)
      rise, os, settle = metrics(a, d, step_idx)
      os_str = f"{os:.1f}%" if abs(os) < 1000 else f"{os:.0f}%"
      print(f"{name:<26s} | {kph:>4.0f}  | {kp:>5.1f} | {rise:>5.2f}s | {os_str:>9s} | {settle:>6.2f}s")
    print()

  # Time-series at 60 km/h
  v = 60 / 3.6
  print(f"\n{'='*100}")
  print(f"Time-series at 60 km/h, delay={plant_delay}s")
  print(f"{'='*100}")
  print(f"{'time':>6s} | {'desired':>7s} | {'stk_nv':>7s} {'stk_dc':>7s} {'bmw_nv':>7s} {'bmw_dc':>7s} | "
        f"{'tq_sn':>6s} {'tq_sd':>6s} {'tq_bn':>6s} {'tq_bd':>6s}")

  results = []
  for name, kp_fn, ki, comp_delay in configs:
    kp = kp_fn(v)
    results.append(simulate(kp, ki, plant_delay, comp_delay))

  for i in range(0, STEPS, 5):
    t = results[0][0][i]
    if 0.15 <= t <= 2.0:
      d = results[0][1][i]
      a = [r[2][i] for r in results]
      tq = [r[3][i] for r in results]
      print(f"{t:>6.2f} | {d:>7.2f} | {a[0]:>7.3f} {a[1]:>7.3f} {a[2]:>7.3f} {a[3]:>7.3f} | "
            f"{tq[0]:>6.2f} {tq[1]:>6.2f} {tq[2]:>6.2f} {tq[3]:>6.2f}")


if __name__ == "__main__":
  main()
