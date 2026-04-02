#!/usr/bin/env python3
"""Design adaptive KP/KI from LiveDelay.

For each delay in [0.05, 1.0], find optimal KP/KI via grid search,
then fit a formula that generalizes across all cars.
"""
import numpy as np
from collections import deque

DT = 0.01
SIM_TIME = 4.0
STEPS = int(SIM_TIME / DT)
PLANT_TAU = 0.08
FRICTION_THRESHOLD = 0.2


def get_friction(error, friction):
  if abs(error) < FRICTION_THRESHOLD:
    return error * friction / FRICTION_THRESHOLD
  return friction * np.sign(error)


def simulate(kp, ki, plant_delay, friction=0.13):
  plant_delay_frames = int(round(plant_delay / DT))
  comp_delay_frames = int(round(plant_delay / DT)) + 1

  integral = 0.0
  plant_state = 0.0
  output_buffer = [0.0] * (plant_delay_frames + 1)
  request_buffer = deque([0.0] * (comp_delay_frames + 1), maxlen=comp_delay_frames + 1)
  measurement = 0.0
  actual_hist, desired_hist = [], []

  for i in range(STEPS):
    t = i * DT
    desired = 1.0 if t >= 0.2 else 0.0
    setpoint = request_buffer[-comp_delay_frames]
    error = setpoint - measurement
    integral += error * DT
    integral = np.clip(integral, -2.0, 2.0)
    ff = desired + get_friction(error, friction)
    pid_output = ff + kp * error + ki * integral
    request_buffer.append(desired)
    output_buffer.append(pid_output)
    delayed_input = output_buffer.pop(0)
    plant_state += (delayed_input - plant_state) * DT / PLANT_TAU
    measurement = plant_state
    actual_hist.append(measurement)
    desired_hist.append(desired)

  return desired_hist, actual_hist


def evaluate(actual, desired):
  step_idx = next(i for i, d in enumerate(desired) if d > 0)
  post = actual[step_idx:]
  target = desired[step_idx]

  peak = max(post)
  overshoot = (peak - target) / target * 100
  t90 = next((i for i, v in enumerate(post) if v >= 0.9 * target), len(post) - 1)
  rise_time = t90 * DT

  settled_idx = 0
  for i in range(len(post) - 1, -1, -1):
    if abs(post[i] - target) > 0.02 * target:
      settled_idx = i + 1
      break
  settling_time = settled_idx * DT
  ss_error = abs(post[-1] - target) / target * 100

  return rise_time, overshoot, settling_time, ss_error


def score(rise, os, settle, ss):
  return os * 2 + settle * 10 + rise * 5 + ss * 50


def main():
  delays = [0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.40, 0.50, 0.60, 0.80, 1.00]
  frictions = [0.08, 0.13, 0.20]  # low, medium, high friction cars
  kp_range = np.arange(0.02, 2.0, 0.02)
  ki_range = np.arange(0.01, 0.5, 0.01)

  print("="*110)
  print("Phase 1: Find optimal KP/KI for each delay (friction=0.13)")
  print("="*110)
  print(f"{'Delay':>6s} | {'Best KP':>7s} | {'Best KI':>7s} | {'Rise':>6s} | {'OS%':>6s} | {'Settle':>7s} | {'SS%':>5s} | {'KP*delay':>8s}")
  print(f"{'-'*6} | {'-'*7} | {'-'*7} | {'-'*6} | {'-'*6} | {'-'*7} | {'-'*5} | {'-'*8}")

  optimal_kp = []
  optimal_ki = []

  for delay in delays:
    best = (1e9, 0, 0, 0, 0, 0, 0)
    for kp in kp_range:
      for ki in ki_range:
        d, a = simulate(kp, ki, delay)
        r, o, s, ss = evaluate(a, d)
        sc = score(r, o, s, ss)
        if sc < best[0]:
          best = (sc, kp, ki, r, o, s, ss)

    _, kp, ki, r, o, s, ss = best
    optimal_kp.append(kp)
    optimal_ki.append(ki)
    print(f"{delay:>5.2f}s | {kp:>7.2f} | {ki:>7.2f} | {r:>5.2f}s | {o:>5.1f}% | {s:>6.2f}s | {ss:>4.1f}% | {kp*delay:>8.4f}")

  # Fit KP = a / delay
  kp_delay_products = [kp * d for kp, d in zip(optimal_kp, delays)]
  a_kp = np.mean(kp_delay_products)

  # Fit KI = b / delay
  ki_delay_products = [ki * d for ki, d in zip(optimal_ki, delays)]
  a_ki = np.mean(ki_delay_products)

  print(f"\nFitted: KP = {a_kp:.4f} / delay    (KP * delay products: {[f'{x:.4f}' for x in kp_delay_products]})")
  print(f"Fitted: KI = {a_ki:.4f} / delay    (KI * delay products: {[f'{x:.4f}' for x in ki_delay_products]})")

  # Validate fitted formula
  print(f"\n{'='*110}")
  print(f"Phase 2: Validate fitted formula KP = {a_kp:.4f}/delay, KI = {a_ki:.4f}/delay")
  print(f"{'='*110}")
  print(f"{'Delay':>6s} | {'Fit KP':>7s} | {'Fit KI':>7s} | {'Rise':>6s} | {'OS%':>6s} | {'Settle':>7s} | {'SS%':>5s} | {'vs optimal':>10s}")
  print(f"{'-'*6} | {'-'*7} | {'-'*7} | {'-'*6} | {'-'*6} | {'-'*7} | {'-'*5} | {'-'*10}")

  for i, delay in enumerate(delays):
    kp_fit = np.clip(a_kp / delay, 0.05, 1.0)
    ki_fit = np.clip(a_ki / delay, 0.01, 0.20)

    d, a = simulate(kp_fit, ki_fit, delay)
    r, o, s, ss = evaluate(a, d)

    d2, a2 = simulate(optimal_kp[i], optimal_ki[i], delay)
    r2, o2, s2, ss2 = evaluate(a2, d2)

    diff = o - o2
    print(f"{delay:>5.2f}s | {kp_fit:>7.3f} | {ki_fit:>7.3f} | {r:>5.2f}s | {o:>5.1f}% | {s:>6.2f}s | {ss:>4.1f}% | {diff:>+9.1f}%")

  # Validate across friction range
  print(f"\n{'='*110}")
  print(f"Phase 3: Robustness across friction range")
  print(f"{'='*110}")
  print(f"{'Delay':>6s} | {'Friction':>8s} | {'KP':>5s} | {'KI':>5s} | {'OS%':>6s} | {'Settle':>7s} | {'SS%':>5s}")
  print(f"{'-'*6} | {'-'*8} | {'-'*5} | {'-'*5} | {'-'*6} | {'-'*7} | {'-'*5}")

  for delay in [0.05, 0.15, 0.40, 0.80]:
    kp_fit = np.clip(a_kp / delay, 0.05, 1.0)
    ki_fit = np.clip(a_ki / delay, 0.01, 0.20)
    for friction in frictions:
      d, a = simulate(kp_fit, ki_fit, delay, friction)
      r, o, s, ss = evaluate(a, d)
      print(f"{delay:>5.2f}s | {friction:>8.2f} | {kp_fit:>5.2f} | {ki_fit:>5.2f} | {o:>5.1f}% | {s:>6.2f}s | {ss:>4.1f}%")

  # Physical bounds summary
  print(f"\n{'='*110}")
  print("Phase 4: Physical bounds for self-tuning lateral control")
  print(f"{'='*110}")
  print(f"""
  Parameter          Range          Source
  ─────────────────  ─────────────  ──────────────────────────────────
  lateralDelay       0.05 - 1.0s   lagd cross-correlation (MIN_LAG, MAX_LAG)
  KP                 0.05 - 1.0    = {a_kp:.4f} / delay, clamped
  KI                 0.01 - 0.20   = {a_ki:.4f} / delay, clamped
  latAccelFactor     1.5 - 4.0     torqued SVD fit (vehicle mass, tire grip)
  friction           0.05 - 0.30   torqued spread estimate (steering column, rack)
  latAccelOffset     -0.5 - 0.5    torqued (roll misalignment compensation)

  Convergence order:
    1. lagd bootstraps delay (relaxed thresholds, 15-20 min)
    2. KP/KI computed from delay (immediate, no learning)
    3. torqued learns latAccelFactor + friction (clean PID data, faster convergence)
    4. lagd refines delay with stable control (tightened thresholds)

  Formula:  KP = {a_kp:.4f} / lateralDelay
            KI = {a_ki:.4f} / lateralDelay
  """)

  # Compare against known cars
  print(f"{'='*110}")
  print("Phase 5: Known car validation")
  print(f"{'='*110}")
  known_cars = [
    ("Toyota TSS2",  0.05, 0.12),
    ("Rivian",       0.15, 0.10),
    ("Hyundai",      0.20, 0.10),
    ("Honda",        0.15, 0.15),
    ("VW MQB",       0.10, 0.10),
    ("BMW E90",      0.40, 0.13),
    ("Subaru (angle)", 0.40, 0.10),
    ("GM Bolt",      0.50, 0.15),
  ]

  print(f"{'Car':>18s} | {'Delay':>6s} | {'Fric':>5s} | {'Adapt KP':>8s} | {'Adapt KI':>8s} | {'OS%':>6s} | {'Settle':>7s} | {'SS%':>5s}")
  print(f"{'-'*18} | {'-'*6} | {'-'*5} | {'-'*8} | {'-'*8} | {'-'*6} | {'-'*7} | {'-'*5}")

  for name, delay, friction in known_cars:
    kp = np.clip(a_kp / delay, 0.05, 1.0)
    ki = np.clip(a_ki / delay, 0.01, 0.20)
    d, a = simulate(kp, ki, delay, friction)
    r, o, s, ss = evaluate(a, d)
    print(f"{name:>18s} | {delay:>5.2f}s | {friction:>5.2f} | {kp:>8.3f} | {ki:>8.3f} | {o:>5.1f}% | {s:>6.2f}s | {ss:>4.1f}%")


if __name__ == "__main__":
  main()
