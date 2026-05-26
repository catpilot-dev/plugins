#!/usr/bin/env python3
"""Sweep friction deadzone based on "lateral offset in 1s" criterion."""
import glob, os, sys, zstandard, numpy as np
from cereal import log as caplog

ROUTE = '/data/media/0/realdata/00000266--1f1715a873--'


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
    try: events.append(next(it))
    except StopIteration: break
    except Exception: break
  return events


def main():
  segs = sorted(glob.glob(f'{ROUTE}*'), key=lambda d: int(d.rsplit('--', 1)[-1]))
  print(f'Found {len(segs)} segments')
  lat_active=False; v=0; yaw_cs=0
  samples = []
  for s in segs:
    p = os.path.join(s, 'rlog.zst')
    if not os.path.exists(p): continue
    for evt in sorted(read_rlog(p), key=lambda e: e.logMonoTime):
      w = evt.which()
      if w=='carControl':
        lat_active = evt.carControl.latActive
      elif w=='carState':
        v = evt.carState.vEgo
        yaw_cs = -evt.carState.yawRate
      elif w=='controlsState' and lat_active and v > 5:
        des = evt.controlsState.desiredCurvature
        samples.append((v, des, yaw_cs / max(v, 5.0)))

  arr = np.array(samples)
  v = arr[:,0]; des = arr[:,1]; meas = arr[:,2]
  err = des - meas
  N = len(arr)
  print(f'Engaged samples: {N}\n')

  for offset_m in [0.25, 0.10, 0.05, 0.02, 0.01]:
    threshold = 2 * offset_m / (1.0 ** 2) / (v ** 2)
    fires = np.abs(err) > threshold
    print(f'{"="*80}')
    print(f'OFFSET_M = {offset_m:.2f}  (threshold = {2*offset_m}/v²)')
    print(f'{"="*80}')
    print(f'  Overall fire rate: {100*fires.mean():.1f}%')
    print(f'  By curvature bin:')
    for lbl, m in [('straight (|des|<5e-4)', np.abs(des) < 0.0005),
                   ('mild (5e-4 - 3e-3)',   (np.abs(des) >= 0.0005) & (np.abs(des) < 0.003)),
                   ('curve (|des|>3e-3)',    np.abs(des) > 0.003)]:
      if m.sum() < 10: continue
      print(f'    {lbl:>24s}: fire {100*fires[m].mean():>5.1f}%   N={m.sum()}')
    print(f'  By speed bin:')
    for lo, hi in [(5,10),(10,15),(15,20),(20,25)]:
      m = (v >= lo) & (v < hi)
      if m.sum() < 10: continue
      t_center = 2 * offset_m / (((lo+hi)/2) ** 2)
      print(f'    {lo}-{hi} m/s (th={t_center:.5f}): fire {100*fires[m].mean():>5.1f}%   N={m.sum()}')


if __name__ == '__main__':
  main()
