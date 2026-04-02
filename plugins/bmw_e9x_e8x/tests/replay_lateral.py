#!/usr/bin/env python3
"""Analyze lateral control oscillation across a route."""
import sys
import os
import glob
import zstandard
import numpy as np
from cereal import log as caplog, car

MS_TO_KPH = 3.6
ROUTE = "/data/media/0/realdata/0000021b--bb850498a7--"

def read_rlog(path):
  dctx = zstandard.ZstdDecompressor()
  with open(path, 'rb') as f:
    raw = dctx.decompress(f.read(), max_output_size=200 * 1024 * 1024)
  return caplog.Event.read_multiple_bytes(raw)

def analyze_segment(seg_id):
  rlog_path = os.path.join(f"{ROUTE}{seg_id}", "rlog.zst")
  if not os.path.exists(rlog_path):
    return None

  events = read_rlog(rlog_path)
  sorted_events = sorted(events, key=lambda e: e.logMonoTime)

  t0 = None
  samples = []
  lat_active = False
  v_ego = 0

  for evt in sorted_events:
    which = evt.which()
    t = evt.logMonoTime * 1e-9
    if t0 is None:
      t0 = t

    if which == "carState":
      v_ego = evt.carState.vEgo
    elif which == "carControl":
      lat_active = evt.carControl.latActive
    elif which == "controlsState":
      cs = evt.controlsState
      if lat_active and v_ego > 3:
        lac = getattr(cs.lateralControlState, cs.lateralControlState.which())
        samples.append({
          't': t - t0,
          'v_ego': v_ego * MS_TO_KPH,
          'curvature': cs.curvature,
          'desired_curvature': cs.desiredCurvature,
          'steer_torque': lac.output if hasattr(lac, 'output') else 0,
          'error': lac.error if hasattr(lac, 'error') else 0,
          'saturated': lac.saturated if hasattr(lac, 'saturated') else False,
          'p': lac.p if hasattr(lac, 'p') else 0,
          'i': lac.i if hasattr(lac, 'i') else 0,
          'f': lac.f if hasattr(lac, 'f') else 0,
          'actual_lat_accel': lac.actualLateralAccel if hasattr(lac, 'actualLateralAccel') else 0,
          'desired_lat_accel': lac.desiredLateralAccel if hasattr(lac, 'desiredLateralAccel') else 0,
        })

  return samples

def main():
  segs = [int(s) for s in sys.argv[1:]] if len(sys.argv) > 1 else list(range(0, 45))

  # Collect all samples across segments
  all_samples = []
  for seg_id in segs:
    samples = analyze_segment(seg_id)
    if samples:
      all_samples.extend(samples)

  if not all_samples:
    print("No data found")
    return

  # Bin by speed and compute oscillation metrics
  speed_bins = [(20, 40), (40, 60), (60, 80), (80, 100)]

  print(f"\n{'Speed bin':>12s} | {'samples':>7s} | {'err_std':>8s} | {'err_p95':>8s} | {'torq_std':>9s} | "
        f"{'P_std':>8s} | {'I_std':>8s} | {'F_std':>8s} | {'sat%':>5s} | {'curv_err_std':>12s}")
  print(f"{'-'*12} | {'-'*7} | {'-'*8} | {'-'*8} | {'-'*9} | {'-'*8} | {'-'*8} | {'-'*8} | {'-'*5} | {'-'*12}")

  for lo, hi in speed_bins:
    bin_samples = [s for s in all_samples if lo <= s['v_ego'] < hi]
    if len(bin_samples) < 100:
      print(f"{lo:>3d}-{hi:<3d} km/h | {len(bin_samples):>7d} | {'(insufficient)':>8s}")
      continue

    errors = np.array([s['error'] for s in bin_samples])
    torques = np.array([s['steer_torque'] for s in bin_samples])
    ps = np.array([s['p'] for s in bin_samples])
    Is = np.array([s['i'] for s in bin_samples])
    fs = np.array([s['f'] for s in bin_samples])
    saturated = sum(1 for s in bin_samples if s['saturated'])
    curv_errs = np.array([s['desired_curvature'] - s['curvature'] for s in bin_samples])

    print(f"{lo:>3d}-{hi:<3d} km/h | {len(bin_samples):>7d} | {errors.std():>8.4f} | {np.percentile(np.abs(errors), 95):>8.4f} | "
          f"{torques.std():>9.4f} | {ps.std():>8.4f} | {Is.std():>8.4f} | {fs.std():>8.4f} | "
          f"{100*saturated/len(bin_samples):>4.1f}% | {curv_errs.std():>12.6f}")

  # Time-series dump for a few interesting windows
  print("\n\n=== Low-speed sample (first 30-50 km/h window) ===")
  low_speed = [s for s in all_samples if 30 <= s['v_ego'] < 50]
  if low_speed:
    # Find a 5-second window with high error variance
    window = 100  # ~5s at 20Hz
    max_var_idx = 0
    max_var = 0
    errors = [s['error'] for s in low_speed]
    for i in range(len(errors) - window):
      v = np.std(errors[i:i+window])
      if v > max_var:
        max_var = v
        max_var_idx = i

    print(f"{'time':>7s} | {'v_ego':>6s} | {'error':>8s} | {'torque':>8s} | {'P':>8s} | {'I':>8s} | {'F':>8s} | {'des_la':>8s} | {'act_la':>8s}")
    for s in low_speed[max_var_idx:max_var_idx + window:5]:  # every 5th sample
      print(f"{s['t']:7.1f} | {s['v_ego']:5.1f} | {s['error']:>8.4f} | {s['steer_torque']:>8.4f} | "
            f"{s['p']:>8.4f} | {s['i']:>8.4f} | {s['f']:>8.4f} | {s['desired_lat_accel']:>8.3f} | {s['actual_lat_accel']:>8.3f}")

  print("\n\n=== High-speed sample (first 70-90 km/h window) ===")
  high_speed = [s for s in all_samples if 70 <= s['v_ego'] < 90]
  if high_speed:
    window = 100
    max_var_idx = 0
    max_var = 0
    errors = [s['error'] for s in high_speed]
    for i in range(len(errors) - window):
      v = np.std(errors[i:i+window])
      if v > max_var:
        max_var = v
        max_var_idx = i

    print(f"{'time':>7s} | {'v_ego':>6s} | {'error':>8s} | {'torque':>8s} | {'P':>8s} | {'I':>8s} | {'F':>8s} | {'des_la':>8s} | {'act_la':>8s}")
    for s in high_speed[max_var_idx:max_var_idx + window:5]:
      print(f"{s['t']:7.1f} | {s['v_ego']:5.1f} | {s['error']:>8.4f} | {s['steer_torque']:>8.4f} | "
            f"{s['p']:>8.4f} | {s['i']:>8.4f} | {s['f']:>8.4f} | {s['desired_lat_accel']:>8.3f} | {s['actual_lat_accel']:>8.3f}")

if __name__ == "__main__":
  main()
