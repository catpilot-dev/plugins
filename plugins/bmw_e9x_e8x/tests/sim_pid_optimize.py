#!/usr/bin/env python3
"""Optimize KP/KI for BMW E90 with delay-compensated PID.

Grid search over KP and KI, targeting minimum overshoot with acceptable
rise time. Plant: 0.4-0.5s delay, friction=0.12-0.13, tau=0.08s.
"""
import numpy as np
from collections import deque

DT = 0.01
SIM_TIME = 4.0
STEPS = int(SIM_TIME / DT)

FRICTION = 0.13
FRICTION_THRESHOLD = 0.2
PLANT_TAU = 0.08


def get_friction(error, friction):
  if abs(error) < FRICTION_THRESHOLD:
    return error * friction / FRICTION_THRESHOLD
  return friction * np.sign(error)


def simulate(kp, ki, plant_delay, comp_delay):
  plant_delay_frames = int(round(plant_delay / DT))
  comp_delay_frames = int(round(comp_delay / DT)) + 1

  integral = 0.0
  plant_state = 0.0
  output_buffer = [0.0] * (plant_delay_frames + 1)
  request_buffer = deque([0.0] * (comp_delay_frames + 1), maxlen=comp_delay_frames + 1)
  measurement = 0.0

  actual_hist = []
  desired_hist = []

  for i in range(STEPS):
    t = i * DT
    desired = 1.0 if t >= 0.2 else 0.0

    setpoint = request_buffer[-comp_delay_frames]
    error = setpoint - measurement
    integral += error * DT
    integral = np.clip(integral, -2.0, 2.0)

    ff = desired + get_friction(error, FRICTION)
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
    if abs(post[i] - target) > 0.02 * target:  # 2% band
      settled_idx = i + 1
      break
  settling_time = settled_idx * DT

  # Steady-state error
  ss_error = abs(post[-1] - target) / target * 100

  return rise_time, overshoot, settling_time, ss_error


def main():
  kp_range = np.arange(0.1, 1.6, 0.1)
  ki_range = np.arange(0.05, 0.5, 0.05)
  delays = [(0.4, 0.4), (0.5, 0.5)]  # (plant_delay, comp_delay)

  for plant_delay, comp_delay in delays:
    print(f"\n{'='*100}")
    print(f"Plant delay = {plant_delay}s, compensation delay = {comp_delay}s, friction = {FRICTION}")
    print(f"{'='*100}")
    print(f"{'KP':>5s} {'KI':>5s} | {'Rise':>6s} | {'OS%':>7s} | {'Settle':>7s} | {'SS_err':>6s} | {'Score':>6s}")
    print(f"{'-'*5} {'-'*5} | {'-'*6} | {'-'*7} | {'-'*7} | {'-'*6} | {'-'*6}")

    results = []
    for kp in kp_range:
      for ki in ki_range:
        d, a = simulate(kp, ki, plant_delay, comp_delay)
        rise, os, settle, ss = evaluate(a, d)

        # Score: penalize overshoot heavily, moderate penalty for slow rise/settle
        # Lower is better
        score = os * 2 + settle * 10 + rise * 5 + ss * 50
        results.append((score, kp, ki, rise, os, settle, ss))

    results.sort()

    # Print top 20
    for score, kp, ki, rise, os, settle, ss in results[:20]:
      marker = " <-- best" if results[0] == (score, kp, ki, rise, os, settle, ss) else ""
      print(f"{kp:>5.1f} {ki:>5.2f} | {rise:>5.2f}s | {os:>6.1f}% | {settle:>6.2f}s | {ss:>5.1f}% | {score:>6.1f}{marker}")

    # Show current BMW config for comparison
    print(f"\n  Current BMW (KP=0.8, KI=0.15):")
    d, a = simulate(0.8, 0.15, plant_delay, comp_delay)
    rise, os, settle, ss = evaluate(a, d)
    score = os * 2 + settle * 10 + rise * 5 + ss * 50
    print(f"  0.8  0.15 | {rise:>5.2f}s | {os:>6.1f}% | {settle:>6.2f}s | {ss:>5.1f}% | {score:>6.1f}")

  # Detailed comparison of top candidate vs current
  print(f"\n\n{'='*100}")
  print("Comparison across speed range (delay=0.45s, average)")
  print(f"{'='*100}")

  # Find best from 0.45s delay
  best_results = []
  for kp in kp_range:
    for ki in ki_range:
      scores = []
      for pd in [0.4, 0.45, 0.5]:
        d, a = simulate(kp, ki, pd, pd)
        rise, os, settle, ss = evaluate(a, d)
        scores.append(os * 2 + settle * 10 + rise * 5 + ss * 50)
      best_results.append((np.mean(scores), kp, ki))

  best_results.sort()
  best_kp, best_ki = best_results[0][1], best_results[0][2]

  print(f"\nBest averaged: KP={best_kp:.1f}, KI={best_ki:.2f}")
  print(f"Current:       KP=0.8, KI=0.15\n")

  print(f"{'Delay':>6s} | {'Config':>15s} | {'Rise':>6s} | {'OS%':>7s} | {'Settle':>7s} | {'SS_err':>6s}")
  print(f"{'-'*6} | {'-'*15} | {'-'*6} | {'-'*7} | {'-'*7} | {'-'*6}")

  for pd in [0.35, 0.40, 0.45, 0.50, 0.55]:
    d1, a1 = simulate(0.8, 0.15, pd, pd)
    d2, a2 = simulate(best_kp, best_ki, pd, pd)
    r1, o1, s1, ss1 = evaluate(a1, d1)
    r2, o2, s2, ss2 = evaluate(a2, d2)
    print(f"{pd:>5.2f}s | {'Current':>15s} | {r1:>5.2f}s | {o1:>6.1f}% | {s1:>6.2f}s | {ss1:>5.1f}%")
    print(f"{'':>6s} | {'Optimized':>15s} | {r2:>5.2f}s | {o2:>6.1f}% | {s2:>6.2f}s | {ss2:>5.1f}%")


if __name__ == "__main__":
  main()
