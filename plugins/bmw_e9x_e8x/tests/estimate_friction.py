#!/usr/bin/env python3
"""Estimate the friction-feedforward magnitude stock applies on route 00000266.

For each engaged sample:
  plant_torque  = desired_curv · v² / LAT_ACCEL_FACTOR   (what K/v² plant model predicts)
  residual      = torque_stock − plant_torque            (extra torque: friction+PID+roll)
  friction_est  = mean(residual · sign(error))           (aligned w/ error direction)

Grouping by |error| lets us separate friction (constant step above deadzone)
from PID proportional response (scales with error).
"""
import glob, os, zstandard, numpy as np
from cereal import log as caplog

ROUTE = '/data/media/0/realdata/00000266--1f1715a873--'
LAT_ACCEL_FACTOR = 2.5


def read_rlog(path):
  dctx = zstandard.ZstdDecompressor()
  with open(path, 'rb') as f:
    return caplog.Event.read_multiple_bytes(dctx.decompress(f.read(), max_output_size=200*1024*1024))


def main():
  segments = sorted(glob.glob(f'{ROUTE}*'), key=lambda d: int(d.rsplit('--', 1)[-1]))
  print(f'Found {len(segments)} segments')

  lat_active = False
  v_ego = 0.0
  samples = []

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
        samples.append((v_ego, cs.desiredCurvature, cs.curvature, torque_out))

  if not samples:
    print('No data'); return

  arr = np.array(samples)
  v, des, meas, t = arr[:,0], arr[:,1], arr[:,2], arr[:,3]
  err = des - meas
  # In register.py sign convention: output_torque = -measured/plant_gain
  # Stock lac.output is also pre-negated. Plant torque in that convention:
  plant_torque = -des * v * v / LAT_ACCEL_FACTOR
  residual = t - plant_torque
  N = len(arr)
  print(f'{N} engaged samples (lat_active, vEgo > 5)\n')

  print(f'{"="*80}')
  print(f'FRICTION ESTIMATE (residual = torque_stock − plant_torque, grouped by |error|)')
  print(f'{"="*80}')
  print(f'  Sign-aligned residual: positive = residual pushes in sign(error) direction')
  print(f'  {"Error band":>22s} | {"N":>6s} | {"mean aligned res":>17s} | {"median aligned res":>19s}')
  # Bin by |error|. Friction should be a constant saturating near a threshold;
  # PID contribution should scale with error.
  for lo, hi, lbl in [
      (0.0,    0.00005, '|err| < 5e-5 (noise)'),
      (0.00005,0.0001,  '5e-5 — 1e-4'),
      (0.0001, 0.0003,  '1e-4 — 3e-4'),
      (0.0003, 0.001,   '3e-4 — 1e-3'),
      (0.001,  0.003,   '1e-3 — 3e-3'),
      (0.003,  1.0,     '> 3e-3 (large)'),
  ]:
    m = (np.abs(err) >= lo) & (np.abs(err) < hi)
    n = m.sum()
    if n < 30: continue
    aligned = residual[m] * np.sign(err[m])  # positive means residual aids error-closing
    # Compensate sign flip: our sign convention means torque opposes error, so align with -err
    # Verify: in stock at |err|>0, if car is undershooting (meas < des, err > 0 right turn),
    # stock applies more torque. In our convention torque and curvature have opposite signs,
    # so aiding error means residual has opposite sign to err. Use -sign(err).
    aligned_neg = residual[m] * (-np.sign(err[m]))
    print(f'  {lbl:>22s} | {n:>6d} | {aligned_neg.mean():>+17.4f} | {np.median(aligned_neg):>+19.4f}')

  # Now check: does the |residual| vs |error| curve saturate? (Friction → plateau; pure PID → linear)
  print(f'\n{"="*80}')
  print('IS IT FRICTION OR P-GAIN? Aligned residual vs |error|')
  print(f'{"="*80}')
  print(f'  A saturating curve = friction; a linear one = proportional gain')
  print(f'  {"|err| bin":>22s} | {"N":>6s} | {"mean |residual|":>16s} | {"P50 aligned":>12s} | {"P90 aligned":>12s}')
  for lo, hi, lbl in [
      (0.0,    0.00005, '< 5e-5'),
      (0.00005,0.0001,  '5e-5 — 1e-4'),
      (0.0001, 0.0002,  '1e-4 — 2e-4'),
      (0.0002, 0.0005,  '2e-4 — 5e-4'),
      (0.0005, 0.001,   '5e-4 — 1e-3'),
      (0.001,  0.003,   '1e-3 — 3e-3'),
      (0.003,  0.01,    '3e-3 — 1e-2'),
  ]:
    m = (np.abs(err) >= lo) & (np.abs(err) < hi)
    n = m.sum()
    if n < 30: continue
    aligned_neg = residual[m] * (-np.sign(err[m]))
    print(f'  {lbl:>22s} | {n:>6d} | {np.abs(residual[m]).mean():>16.4f} | {np.median(aligned_neg):>+12.4f} | {np.percentile(aligned_neg, 90):>+12.4f}')


if __name__ == '__main__':
  main()
