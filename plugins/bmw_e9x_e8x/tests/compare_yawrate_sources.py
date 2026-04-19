#!/usr/bin/env python3
"""Compare measured curvature from CS.yawRate (BMW DSC @ 100 Hz, sign-flipped)
vs livePose.angularVelocityDevice.z (Kalman-filtered @ 20 Hz).

Measures the difference in magnitude, the correlation, and where the two
diverge (noise amplitude, lag, sign issues).
"""
import glob, os, sys, zstandard, numpy as np
from cereal import log as caplog

DEFAULT_ROUTE = '/data/media/0/realdata/00000266--1f1715a873--'
if len(sys.argv) > 1:
  arg = sys.argv[1]
  ROUTE = arg if arg.startswith('/') else (glob.glob(f'/data/media/0/realdata/{arg}*')[0].rsplit('--', 1)[0] + '--')
else:
  ROUTE = DEFAULT_ROUTE
print(f'Route: {ROUTE}')


def read_rlog(path):
  dctx = zstandard.ZstdDecompressor()
  raw = b''
  try:
    with open(path, 'rb') as f:
      with dctx.stream_reader(f) as reader:
        raw = reader.read()
  except zstandard.ZstdError:
    pass
  events = []
  it = caplog.Event.read_multiple_bytes(raw)
  while True:
    try:
      events.append(next(it))
    except StopIteration: break
    except Exception: break
  return events


def main():
  segments = sorted(glob.glob(f'{ROUTE}*'), key=lambda d: int(d.rsplit('--', 1)[-1]))
  print(f'Found {len(segments)} segments')

  v_ego = 0.0
  yr_cs = 0.0
  yr_lp = 0.0
  lat_active = False
  samples = []  # (v, desired, -CS.yawRate, +livePose.z)

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
        yr_cs = evt.carState.yawRate
      elif w == 'livePose':
        yr_lp = evt.livePose.angularVelocityDevice.z
      elif w == 'controlsState' and lat_active and v_ego > 5:
        des = evt.controlsState.desiredCurvature
        samples.append((v_ego, des, -yr_cs, yr_lp))   # signs aligned to desired's convention

  if not samples:
    print('No engaged samples'); return

  arr = np.array(samples)
  v = arr[:,0]; des = arr[:,1]
  curv_cs = arr[:,2] / np.maximum(v, 5.0)   # -CS.yawRate / v
  curv_lp = arr[:,3] / np.maximum(v, 5.0)   # livePose.z / v
  diff = curv_cs - curv_lp
  N = len(arr)
  print(f'\n{N} engaged samples\n')

  print(f'{"="*80}')
  print('OVERALL — -CS.yawRate/v  vs  livePose.angularVelocityDevice.z/v')
  print(f'{"="*80}')
  print(f'  N            = {N}')
  print(f'  mean(CS)     = {curv_cs.mean():+.6f}    std = {curv_cs.std():.6f}')
  print(f'  mean(LP)     = {curv_lp.mean():+.6f}    std = {curv_lp.std():.6f}')
  print(f'  mean(diff)   = {diff.mean():+.6f}    std(diff) = {diff.std():.6f}')
  print(f'  correlation  = {np.corrcoef(curv_cs, curv_lp)[0,1]:.4f}')
  print(f'  |diff| mean  = {np.abs(diff).mean():.6f}')
  print(f'  |diff| P50   = {np.percentile(np.abs(diff),50):.6f}')
  print(f'  |diff| P95   = {np.percentile(np.abs(diff),95):.6f}')
  print(f'  |diff| P99   = {np.percentile(np.abs(diff),99):.6f}')

  # slope: if CS reports X% more/less than LP
  slope = float(np.sum(curv_cs * curv_lp) / np.sum(curv_lp * curv_lp))
  print(f'  linear slope (CS = slope × LP): {slope:.4f}')

  print(f'\n{"="*80}')
  print('BY CURVATURE BIN')
  print(f'{"="*80}')
  print(f'  {"|desired|":>20s} | {"N":>6s} | {"mean CS":>10s} | {"mean LP":>10s} | {"mean diff":>10s} | {"|diff| P95":>11s}')
  for lo, hi, lbl in [(0, 0.0005, 'straight <5e-4'),
                       (0.0005, 0.003, 'mild'),
                       (0.003, 0.01, 'curve'),
                       (0.01, 1.0, 'sharp')]:
    m = (np.abs(des) >= lo) & (np.abs(des) < hi)
    if m.sum() < 30: continue
    d = diff[m]
    print(f'  {lbl:>20s} | {m.sum():>6d} | {curv_cs[m].mean():>+10.6f} | {curv_lp[m].mean():>+10.6f} | {d.mean():>+10.6f} | {np.percentile(np.abs(d),95):>11.6f}')

  print(f'\n{"="*80}')
  print('BY SPEED BIN')
  print(f'{"="*80}')
  print(f'  {"Speed":>12s} | {"N":>6s} | {"mean CS":>10s} | {"mean LP":>10s} | {"|diff| mean":>12s} | {"|diff| P95":>11s}')
  for lo, hi in [(5,10),(10,15),(15,20),(20,25)]:
    m = (v >= lo) & (v < hi)
    if m.sum() < 30: continue
    d = diff[m]
    print(f'  {lo:>2d}-{hi:<2d} m/s   | {m.sum():>6d} | {curv_cs[m].mean():>+10.6f} | {curv_lp[m].mean():>+10.6f} | {np.abs(d).mean():>12.6f} | {np.percentile(np.abs(d),95):>11.6f}')


if __name__ == '__main__':
  main()
