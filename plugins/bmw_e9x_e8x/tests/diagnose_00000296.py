#!/usr/bin/env python3
"""Diagnose drift / tracking symptoms on an engaged route:
  1. Straight-line drift (measured_curvature mean bias vs desired)
  2. In-turn tracking (|measured| / |desired| distribution)
  3. Torque output bias, steady-state error by speed

Usage:  diagnose.py                              # default route 00000296
        diagnose.py 00000297                    # by route prefix
        diagnose.py /data/media/0/realdata/00000297--2b9bca7287--
"""
import glob, os, sys, zstandard, numpy as np
from cereal import log as caplog

_DEFAULT = '/data/media/0/realdata/00000296--6dfdb1bbbd--'
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
  ROUTE = _DEFAULT
print(f'Route: {ROUTE}')
MS_TO_KPH = 3.6


def read_rlog(path):
  # Tolerant of both truncated zst frames and truncated capnp messages at tail
  # (live-recording segments).
  dctx = zstandard.ZstdDecompressor()
  raw = b''
  try:
    with open(path, 'rb') as f:
      with dctx.stream_reader(f) as reader:
        raw = reader.read()
  except zstandard.ZstdError:
    pass
  if not raw:
    return []
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

  lat_active = False
  v_ego = 0.0
  steer_angle = 0.0
  yaw_rate = 0.0
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
        steer_angle = evt.carState.steeringAngleDeg
      elif w == 'livePose':
        yaw_rate = evt.livePose.angularVelocityDevice.z
      elif w == 'controlsState' and lat_active and v_ego > 5:
        cs = evt.controlsState
        lac = getattr(cs.lateralControlState, cs.lateralControlState.which())
        torque_out = lac.output if hasattr(lac, 'output') else 0.0
        meas_from_gyro = yaw_rate / max(v_ego, 5.0)
        samples.append({
          'v': v_ego,
          'des_curv': cs.desiredCurvature,
          'meas_curv_log': cs.curvature,
          'meas_curv_gyro': meas_from_gyro,
          'torque': torque_out,
          'steer_angle': steer_angle,
        })

  if not samples:
    print('No engaged data')
    return

  arr = np.array([(s['v'], s['des_curv'], s['meas_curv_log'], s['meas_curv_gyro'],
                   s['torque'], s['steer_angle']) for s in samples])
  v = arr[:,0]; des = arr[:,1]; meas = arr[:,2]; meas_g = arr[:,3]
  torque = arr[:,4]; steer = arr[:,5]
  N = len(arr)
  print(f'\nTotal engaged samples: {N}')

  # ========== STRAIGHT-LINE DRIFT ==========
  print(f'\n{"="*80}')
  print('STRAIGHT-LINE BEHAVIOR (|desired| < 0.0005)')
  print(f'{"="*80}')
  m_str = np.abs(des) < 0.0005
  if m_str.sum() > 20:
    print(f'  N = {m_str.sum()}')
    print(f'  desired   mean={des[m_str].mean():+.6f}  std={des[m_str].std():.6f}')
    print(f'  measured (log)  mean={meas[m_str].mean():+.6f}  std={meas[m_str].std():.6f}')
    print(f'  measured (gyro) mean={meas_g[m_str].mean():+.6f}  std={meas_g[m_str].std():.6f}')
    print(f'  des - meas (log)  mean={(des[m_str]-meas[m_str]).mean():+.6f}')
    print(f'  des - meas (gyro) mean={(des[m_str]-meas_g[m_str]).mean():+.6f}')
    print(f'  steering angle mean={steer[m_str].mean():+.3f}° (+ is left in OP convention?)')
    print(f'  torque_out mean={torque[m_str].mean():+.4f}  std={torque[m_str].std():.4f}')
    print(f'  torque_out P25={np.percentile(torque[m_str],25):+.4f}  P75={np.percentile(torque[m_str],75):+.4f}')
    print(f'\n  → Convention check: openpilot curvature is LEFT-POSITIVE. If desired≈0')
    print(f'    but measured < 0 on average, car is drifting RIGHT. Measured > 0 means LEFT.')
    left_drift_frac = (meas[m_str] > 0.0002).mean()
    right_drift_frac = (meas[m_str] < -0.0002).mean()
    print(f'    Samples with measured > 0.0002 (drifting left):  {100*left_drift_frac:.1f}%')
    print(f'    Samples with measured < -0.0002 (drifting right): {100*right_drift_frac:.1f}%')

  # ========== TURN BEHAVIOR — is measured overshooting desired? ==========
  print(f'\n{"="*80}')
  print('TURN BEHAVIOR — measured vs desired magnitude')
  print(f'{"="*80}')
  m_curve = np.abs(des) > 0.003
  if m_curve.sum() > 20:
    ratio = np.abs(meas[m_curve]) / np.maximum(np.abs(des[m_curve]), 1e-5)
    print(f'  N = {m_curve.sum()} (|desired| > 0.003)')
    print(f'  |measured| / |desired|  mean = {ratio.mean():.3f}  median = {np.median(ratio):.3f}')
    print(f'  overshoot (|meas| > |des| * 1.1):  {100*(ratio > 1.1).mean():.1f}%')
    print(f'  undershoot (|meas| < |des| * 0.9): {100*(ratio < 0.9).mean():.1f}%')
    # Signed: is the car turning MORE than requested, in the same direction?
    same_sign = des[m_curve] * meas[m_curve] > 0
    print(f'  same-sign samples: {100*same_sign.mean():.1f}% (car and plan agree on direction)')

  # ========== TORQUE BIAS — does controller output favor one direction? ==========
  print(f'\n{"="*80}')
  print('TORQUE OUTPUT BIAS')
  print(f'{"="*80}')
  for lbl, m in [('all engaged', np.ones_like(v, dtype=bool)),
                 ('straight (|des|<0.0005)', np.abs(des) < 0.0005),
                 ('curving (|des|>0.003)', np.abs(des) > 0.003)]:
    if m.sum() < 20: continue
    t = torque[m]
    print(f'  {lbl:<28s} N={m.sum():>6d}  mean={t.mean():+.4f}  frac(t>0)={100*(t>0).mean():.1f}%  frac(t<0)={100*(t<0).mean():.1f}%')

  # ========== SPEED-BINNED ERROR ==========
  print(f'\n{"="*80}')
  print('STEADY-STATE ERROR vs SPEED (|desired| < 0.0005)')
  print(f'{"="*80}')
  print(f'  {"Speed":>12s} | {"N":>5s} | {"mean des":>9s} | {"mean meas":>9s} | {"mean(des-meas)":>14s} | {"mean torque":>12s}')
  for lo, hi in [(5,10),(10,15),(15,20),(20,25)]:
    m = (v >= lo) & (v < hi) & (np.abs(des) < 0.0005)
    if m.sum() < 30: continue
    print(f'  {lo:>2d}-{hi:<2d} m/s   | {m.sum():>5d} | {des[m].mean():>+9.6f} | {meas[m].mean():>+9.6f} | {(des[m]-meas[m]).mean():>+14.6f} | {torque[m].mean():>+12.4f}')


if __name__ == '__main__':
  main()
