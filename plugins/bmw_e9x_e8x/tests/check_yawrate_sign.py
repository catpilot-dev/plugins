#!/usr/bin/env python3
"""Verify sign convention: CS.yawRate vs desiredCurvature vs livePose z-axis.

Check which sign of CS.yawRate matches desiredCurvature's convention, and
confirm it's the OPPOSITE of livePose.angularVelocityDevice.z (which the
current register.py uses — so we flip the sign for CS.yawRate).
"""
import glob, os, sys, zstandard, numpy as np
from cereal import log as caplog

DEFAULT_ROUTE = '/data/media/0/realdata/00000266--1f1715a873--'
if len(sys.argv) > 1:
  arg = sys.argv[1]
  if arg.startswith('/'):
    ROUTE = arg
  else:
    found = glob.glob(f'/data/media/0/realdata/{arg}*')
    if not found:
      print(f'No segments matching {arg}'); sys.exit(1)
    ROUTE = found[0].rsplit('--', 1)[0] + '--'
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
  if not raw: return []
  events = []
  it = caplog.Event.read_multiple_bytes(raw)
  while True:
    try:
      events.append(next(it))
    except StopIteration:
      break
    except Exception:
      break
  return events


def main():
  segments = sorted(glob.glob(f'{ROUTE}*'), key=lambda d: int(d.rsplit('--', 1)[-1]))
  print(f'Found {len(segments)} segments')

  v_ego = 0.0
  yaw_rate_carstate = 0.0
  yaw_rate_livepose = 0.0
  lat_active = False
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
        yaw_rate_carstate = evt.carState.yawRate
      elif w == 'livePose':
        yaw_rate_livepose = evt.livePose.angularVelocityDevice.z
      elif w == 'controlsState' and lat_active and v_ego > 5:
        cs = evt.controlsState
        samples.append((v_ego, cs.desiredCurvature, cs.curvature,
                        yaw_rate_carstate, yaw_rate_livepose))

  if not samples:
    print('No engaged samples')
    return

  arr = np.array(samples)
  v, des, meas_log = arr[:,0], arr[:,1], arr[:,2]
  yr_cs, yr_lp = arr[:,3], arr[:,4]
  N = len(arr)
  print(f'\nEngaged samples: {N}\n')

  # Derived curvatures from each source
  curv_from_carstate = yr_cs / np.maximum(v, 5.0)
  curv_from_livepose = yr_lp / np.maximum(v, 5.0)

  # Focus on clear-turn samples so sign is unambiguous
  m_turn = np.abs(des) > 0.003
  print(f'{"="*80}')
  print(f'SIGN MATCH (in clear turns |desired| > 0.003, N={m_turn.sum()})')
  print(f'{"="*80}')
  print(f'  Source                           | same-sign as desired | corr vs desired')
  for lbl, x in [('log measured (cs.curvature)',  meas_log[m_turn]),
                 ('+CS.yawRate/v',                curv_from_carstate[m_turn]),
                 ('-CS.yawRate/v',                -curv_from_carstate[m_turn]),
                 ('+livePose.angVelDev.z/v',      curv_from_livepose[m_turn]),
                 ('-livePose.angVelDev.z/v',      -curv_from_livepose[m_turn])]:
    d = des[m_turn]
    same = (x * d > 0).mean() * 100
    c = np.corrcoef(x, d)[0,1]
    print(f'  {lbl:<32s} |     {same:>5.1f}%           |   {c:+.3f}')

  print(f'\n{"="*80}')
  print('MAGNITUDE CHECK — do CS.yawRate and livePose agree in magnitude?')
  print(f'{"="*80}')
  # Assume correct signs chosen above; compare magnitudes in clear turns
  # If both are legit yaw rates, they should be within a small factor
  m = m_turn
  print(f'  mean |CS.yawRate/v|         = {np.abs(curv_from_carstate[m]).mean():.5f}')
  print(f'  mean |livePose z/v|         = {np.abs(curv_from_livepose[m]).mean():.5f}')
  print(f'  mean |cs.curvature (log)|   = {np.abs(meas_log[m]).mean():.5f}')
  print(f'  mean |desired|              = {np.abs(des[m]).mean():.5f}')

  print(f'\n{"="*80}')
  print('RELATIVE SIGN — is CS.yawRate opposite to livePose?')
  print(f'{"="*80}')
  same = (yr_cs[m] * yr_lp[m] > 0).mean() * 100
  print(f'  same-sign(CS.yawRate, livePose.z) in turns: {same:.1f}%')
  print(f'  → If <50%: OPPOSITE convention (confirms need to flip CS.yawRate)')
  print(f'  → If >50%: SAME convention (no flip needed)')


if __name__ == '__main__':
  main()
