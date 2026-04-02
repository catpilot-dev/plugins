#!/usr/bin/env python3
"""Validate adaptive PID gains against real route data from CI.

For each route:
  1. Extract CarParams and lateral control quality
  2. Read liveDelay messages for actual delay estimate
  3. Compute adaptive KP/KI
  4. Compare stock vs adaptive
"""
import sys
import os
import zstandard
import bz2
import numpy as np

CATPILOT = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'catpilot')
sys.path.insert(0, os.path.abspath(CATPILOT))

from cereal import log as caplog

# Stock PID schedule
INTERP_SPEEDS = [1, 1.5, 2.0, 3.0, 5, 7.5, 10, 15, 30]
KP_INTERP = [250, 120, 65, 30, 11.5, 5.5, 3.5, 2.0, 0.8]

def adaptive_kp(delay): return np.clip(0.008 / delay, 0.05, 1.0)
def adaptive_ki(delay): return np.clip(0.009 / delay, 0.01, 0.20)


def read_rlog(path):
  if path.endswith('.zst'):
    dctx = zstandard.ZstdDecompressor()
    with open(path, 'rb') as f:
      raw = dctx.decompress(f.read(), max_output_size=200 * 1024 * 1024)
  elif path.endswith('.bz2'):
    with open(path, 'rb') as f:
      raw = bz2.decompress(f.read())
  else:
    raise ValueError(f"Unknown format: {path}")
  return caplog.Event.read_multiple_bytes(raw)


def analyze_route(name, rlog_path):
  events = read_rlog(rlog_path)
  sorted_events = sorted(events, key=lambda e: e.logMonoTime)

  CP = None
  for evt in sorted_events:
    if evt.which() == 'carParams':
      CP = evt.carParams
      break

  if CP is None:
    print(f"  {name}: No carParams found")
    return

  print(f"\n{'='*100}")
  print(f"  {name}")
  print(f"  Car: {CP.carFingerprint}, brand: {CP.brand}")
  print(f"  steerActuatorDelay: {CP.steerActuatorDelay:.3f}s")
  if CP.lateralTuning.which() == 'torque':
    t = CP.lateralTuning.torque
    print(f"  latAccelFactor: {t.latAccelFactor:.3f}, friction: {t.friction:.3f}")
  print(f"{'='*100}")

  # Collect all relevant data in one pass
  lat_active = False
  v_ego = 0
  errors, torques, speeds = [], [], []
  p_terms, i_terms, f_terms = [], [], []
  desired_la, actual_la = [], []
  delay_values, delay_statuses = [], []

  for evt in sorted_events:
    which = evt.which()
    if which == "carControl":
      lat_active = evt.carControl.latActive
    elif which == "carState":
      v_ego = evt.carState.vEgo
    elif which == "controlsState" and lat_active and v_ego > 5:
      cs = evt.controlsState
      lac = getattr(cs.lateralControlState, cs.lateralControlState.which())
      if hasattr(lac, 'error') and hasattr(lac, 'output'):
        errors.append(lac.error)
        torques.append(lac.output)
        speeds.append(v_ego)
        p_terms.append(lac.p if hasattr(lac, 'p') else 0)
        i_terms.append(lac.i if hasattr(lac, 'i') else 0)
        f_terms.append(lac.f if hasattr(lac, 'f') else 0)
        if hasattr(lac, 'actualLateralAccel') and hasattr(lac, 'desiredLateralAccel'):
          desired_la.append(lac.desiredLateralAccel)
          actual_la.append(lac.actualLateralAccel)
    elif which == "liveDelay":
      ld = evt.liveDelay
      delay_values.append(ld.lateralDelay)
      delay_statuses.append(str(ld.status).split('.')[-1])

  if not errors:
    print("  No lateral control data found")
    return

  errors = np.array(errors)
  torques = np.array(torques)
  speeds = np.array(speeds)
  p_terms = np.array(p_terms)
  i_terms = np.array(i_terms)
  f_terms = np.array(f_terms)

  # --- Stock control quality ---
  print(f"\n  Stock lateral control ({len(errors)} samples, {speeds.min()*3.6:.0f}-{speeds.max()*3.6:.0f} km/h):")
  print(f"    error:   std={errors.std():.4f}  p95={np.percentile(np.abs(errors), 95):.4f}  mean={errors.mean():.4f}")
  print(f"    torque:  std={torques.std():.4f}")
  print(f"    P term:  std={p_terms.std():.4f}")
  print(f"    I term:  std={i_terms.std():.4f}  mean={i_terms.mean():.4f}")
  print(f"    F term:  std={f_terms.std():.4f}")
  if desired_la:
    dla = np.array(desired_la)
    ala = np.array(actual_la)
    tracking_err = dla - ala
    print(f"    lat_accel tracking: err_std={tracking_err.std():.4f}  err_p95={np.percentile(np.abs(tracking_err), 95):.4f}")

  # --- LiveDelay from recorded data ---
  if delay_values:
    last_delay = delay_values[-1]
    last_status = delay_statuses[-1]
    print(f"\n  Recorded LiveDelay ({len(delay_values)} msgs):")
    print(f"    final: {last_delay:.3f}s ({last_status})")
    print(f"    range: {min(delay_values):.3f} - {max(delay_values):.3f}s")
  else:
    last_delay = CP.steerActuatorDelay + 0.2
    last_status = 'none'
    print(f"\n  No LiveDelay in rlog, using fallback: {last_delay:.3f}s")

  # --- Adaptive gains ---
  effective_delay = last_delay if last_status == 'estimated' else max(CP.steerActuatorDelay + 0.2, 0.5)
  akp = adaptive_kp(effective_delay)
  aki = adaptive_ki(effective_delay)
  avg_speed = speeds.mean()
  stock_kp_avg = np.interp(avg_speed, INTERP_SPEEDS, KP_INTERP)

  print(f"\n  Adaptive gains (delay={effective_delay:.3f}s):")
  print(f"    KP = {akp:.3f}  (stock at {avg_speed*3.6:.0f}kph: {stock_kp_avg:.2f})")
  print(f"    KI = {aki:.3f}  (stock: 0.15)")

  # --- Per-speed-bin analysis ---
  speed_bins = [(20, 40), (40, 60), (60, 80), (80, 120)]
  print(f"\n  {'Speed':>10s} | {'N':>6s} | {'Stk KP':>7s} | {'Adp KP':>7s} | {'ratio':>6s} | {'err_std':>8s} | {'P_std':>8s} | {'F_std':>8s}")
  print(f"  {'-'*10} | {'-'*6} | {'-'*7} | {'-'*7} | {'-'*6} | {'-'*8} | {'-'*8} | {'-'*8}")

  for lo, hi in speed_bins:
    mask = (speeds * 3.6 >= lo) & (speeds * 3.6 < hi)
    n = mask.sum()
    if n < 50:
      continue
    avg_v = speeds[mask].mean()
    stock_kp = np.interp(avg_v, INTERP_SPEEDS, KP_INTERP)
    ratio = stock_kp / akp
    print(f"  {lo:>3d}-{hi:<3d}kph | {n:>6d} | {stock_kp:>7.2f} | {akp:>7.3f} | {ratio:>5.1f}x | "
          f"{errors[mask].std():>8.4f} | {p_terms[mask].std():>8.4f} | {f_terms[mask].std():>8.4f}")

  # --- Oscillation analysis: error zero-crossings per second ---
  if len(errors) > 100:
    crossings = np.sum(errors[1:] * errors[:-1] < 0)
    duration = len(errors) * 0.01  # ~100Hz
    osc_freq = crossings / duration / 2  # half-cycles per second
    print(f"\n  Oscillation: {crossings} zero-crossings in {duration:.1f}s = {osc_freq:.1f} Hz")


def main():
  routes = {
    "Toyota Corolla TSS2 (regen 2025)": "/home/oxygen/catpilot-dev/route_cache/corolla_tss2_regen/rlog_0.zst",
    "Toyota Prius (regen 2025)": "/home/oxygen/catpilot-dev/route_cache/prius_regen/rlog_0.zst",
    "Hyundai Sonata (regen 2025)": "/home/oxygen/catpilot-dev/route_cache/hyundai_sonata_regen/rlog_0.zst",
    "Kia EV6 (regen 2025)": "/home/oxygen/catpilot-dev/route_cache/kia_ev6_regen/rlog_0.zst",
  }

  # Also check for BMW route on C3 cache
  bmw_path = "/home/oxygen/catpilot-dev/route_cache/bmw_e90"
  if os.path.exists(bmw_path):
    routes["BMW E90"] = bmw_path

  for name, path in routes.items():
    if os.path.exists(path):
      try:
        analyze_route(name, path)
      except Exception as e:
        print(f"  {name}: Error - {e}")
        import traceback
        traceback.print_exc()
    else:
      print(f"  {name}: not found at {path}")

  print(f"\n\n{'='*100}")
  print("Summary: Adaptive PID formula")
  print(f"{'='*100}")
  print(f"  KP = 0.008 / lateralDelay   clamped [0.05, 1.0]")
  print(f"  KI = 0.009 / lateralDelay   clamped [0.01, 0.20]")
  print(f"")
  print(f"  For Toyota at 0.12s delay + 0.2 offset = 0.32s:")
  print(f"    KP = {adaptive_kp(0.32):.3f}, KI = {adaptive_ki(0.32):.3f}")
  print(f"  For Toyota at 0.05s actual (if lagd converges):")
  print(f"    KP = {adaptive_kp(0.05):.3f}, KI = {adaptive_ki(0.05):.3f}")
  print(f"  For BMW at 0.4s actual:")
  print(f"    KP = {adaptive_kp(0.4):.3f}, KI = {adaptive_ki(0.4):.3f}")


if __name__ == "__main__":
  main()
