#!/usr/bin/env python3
"""Empirically fit PLANT_GAIN_COEFF K in: desired_curvature · v² = K · torque_output.

Pairs the stock controller's commanded torque(t) with the desired curvature
the plan wanted at t+0.5s (which IS what cs.desiredCurvature represents —
modelV2 plan at lat_action_t≈0.5s). The lookahead is baked into the desired
signal, so the pairing is a direct plant-gain measurement with no manual lag
correction.

Assumes the stock controller is tracking well enough that commanded torque
produces the intended future curvature (i.e., steady-state consistency).
"""
import glob, os, zstandard, numpy as np
from cereal import log as caplog

ROUTE = '/data/media/0/realdata/00000266--1f1715a873--'
MS_TO_KPH = 3.6


def read_rlog(path):
  dctx = zstandard.ZstdDecompressor()
  with open(path, 'rb') as f:
    return caplog.Event.read_multiple_bytes(dctx.decompress(f.read(), max_output_size=200*1024*1024))


def main():
  segments = sorted(glob.glob(f'{ROUTE}*'), key=lambda d: int(d.rsplit('--', 1)[-1]))
  print(f'Found {len(segments)} segments')

  lat_active = False
  v_ego = 0.0
  samples = []  # (v, torque_out, desired_curv, measured_curv_log)

  for seg_dir in segments:
    rlog_path = os.path.join(seg_dir, 'rlog.zst')
    if not os.path.exists(rlog_path): continue
    events = sorted(read_rlog(rlog_path), key=lambda e: e.logMonoTime)

    for evt in events:
      w = evt.which()
      if w == 'carControl':
        lat_active = evt.carControl.latActive
      elif w == 'carState':
        v_ego = evt.carState.vEgo
      elif w == 'controlsState' and lat_active and v_ego > 5:
        cs = evt.controlsState
        lac = getattr(cs.lateralControlState, cs.lateralControlState.which())
        torque_out = lac.output if hasattr(lac, 'output') else 0.0
        if abs(torque_out) > 0.02:
          samples.append((v_ego, torque_out, cs.desiredCurvature, cs.curvature))

  if not samples:
    print('No data')
    return

  arr = np.array(samples)
  v, t, des, meas = arr[:,0], arr[:,1], arr[:,2], arr[:,3]
  N = len(arr)
  print(f'\n{N} engaged samples (|torque| > 0.02, lat_active, vEgo > 5)\n')

  # Fit: desired_curv · v² = K · torque_output (with sign determination)
  print(f'{"="*80}')
  print('FIT: desired_curv(t+0.5s) · v(t)² = K · torque_output(t)')
  print(f'{"="*80}')

  y_des = des * v * v
  for sign, name in [(+1, 'des = +K·t/v²'), (-1, 'des = -K·t/v²')]:
    x = sign * t
    K = float(np.sum(x * y_des) / np.sum(x * x))
    resid = y_des - K * x
    r2 = 1 - np.sum(resid ** 2) / np.sum((y_des - y_des.mean()) ** 2)
    print(f'  {name}:  K = {K:>7.3f}   R² = {r2:>6.3f}')

  print(f'  By speed bin (K via des = -K·t/v²):')
  print(f'    {"Speed":>14s} | {"N":>6s} | {"K":>7s} | {"R²":>6s} | {"plant_gain at center":>22s}')
  for lo, hi in [(5,10),(10,15),(15,20),(20,25),(25,30)]:
    m = (v >= lo) & (v < hi)
    n = m.sum()
    if n < 50: continue
    x = -t[m]; yy = des[m] * v[m] * v[m]
    K = float(np.sum(x * yy) / np.sum(x * x))
    r2 = 1 - np.sum((yy - K * x) ** 2) / np.sum((yy - yy.mean()) ** 2)
    v_c = (lo + hi) / 2
    print(f'    {lo:>2d}-{hi:<2d} m/s     | {n:>6d} | {K:>7.3f} | {r2:>6.3f} | {K / v_c**2:>22.5f}')

  print(f'  High-torque subset (|torque| > 0.1):')
  m = np.abs(t) > 0.1
  if m.sum() > 50:
    x = -t[m]; yy = des[m] * v[m] ** 2
    K = float(np.sum(x * yy) / np.sum(x * x))
    r2 = 1 - np.sum((yy - K * x) ** 2) / np.sum((yy - yy.mean()) ** 2)
    print(f'    N={m.sum()}  K = {K:.3f}  R² = {r2:.3f}')

  # Sanity: compare to measured-curvature fit (same pairing, no lag — worse by construction)
  print(f'\n{"="*80}')
  print('SANITY: measured_curv(t) · v² = K · torque(t) — same pairing, no lag correction')
  print(f'{"="*80}')
  for sign, name in [(+1, 'meas = +K·t/v²'), (-1, 'meas = -K·t/v²')]:
    x = sign * t; yy = meas * v * v
    K = float(np.sum(x * yy) / np.sum(x * x))
    r2 = 1 - np.sum((yy - K * x) ** 2) / np.sum((yy - yy.mean()) ** 2)
    print(f'  {name}:  K = {K:>7.3f}   R² = {r2:>6.3f}')


if __name__ == '__main__':
  main()
