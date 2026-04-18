#!/usr/bin/env python3
"""Empirically fit PLANT_GAIN_COEFF K in: measured_curvature(t) ≈ -(K/v²) · torque(t-LAG).

Accounts for actuator + tire response lag: 0.3 s steering actuator + 0.2 s tire
slip transient = 0.5 s total.

Reads route 00000266 logs, builds per-segment time series at controlsState rate
(~100 Hz), pairs torque[i-LAG_FRAMES] with measurement[i], regresses through
origin: measured_curv·v² = K · (-torque_lagged).
"""
import glob, os, zstandard, numpy as np
from cereal import log as caplog

ROUTE = '/data/media/0/realdata/00000266--1f1715a873--'
MS_TO_KPH = 3.6
LAG_S = 0.5             # 0.3 actuator + 0.2 tire response
CONTROLS_HZ = 100
LAG_FRAMES = int(LAG_S * CONTROLS_HZ)


def read_rlog(path):
  dctx = zstandard.ZstdDecompressor()
  with open(path, 'rb') as f:
    return caplog.Event.read_multiple_bytes(dctx.decompress(f.read(), max_output_size=200*1024*1024))


def main():
  segments = sorted(glob.glob(f'{ROUTE}*'), key=lambda d: int(d.rsplit('--', 1)[-1]))
  print(f'Found {len(segments)} segments  |  LAG = {LAG_S}s ({LAG_FRAMES} frames @ {CONTROLS_HZ}Hz)')

  # per-segment time series, then pair with lag
  all_pairs = []  # (v, torque_lagged, c_log, c_yaw, active_lagged)

  for seg_dir in segments:
    rlog_path = os.path.join(seg_dir, 'rlog.zst')
    if not os.path.exists(rlog_path): continue
    events = sorted(read_rlog(rlog_path), key=lambda e: e.logMonoTime)

    lat_active = False
    v_ego = 0.0
    yaw_rate = 0.0
    yaw_rate_valid = False
    seg_series = []  # one row per controlsState event

    for evt in events:
      w = evt.which()
      if w == 'carControl':
        lat_active = evt.carControl.latActive
      elif w == 'carState':
        v_ego = evt.carState.vEgo
      elif w == 'livePose':
        yaw_rate = evt.livePose.angularVelocityDevice.z
        yaw_rate_valid = True
      elif w == 'controlsState' and yaw_rate_valid:
        cs = evt.controlsState
        lac = getattr(cs.lateralControlState, cs.lateralControlState.which())
        torque_out = lac.output if hasattr(lac, 'output') else 0.0
        seg_series.append((v_ego, torque_out, cs.curvature,
                           yaw_rate / max(v_ego, 5.0), lat_active))

    if len(seg_series) <= LAG_FRAMES:
      continue
    arr = np.array(seg_series, dtype=float)
    # pair measurement at i with torque at i-LAG_FRAMES
    v_meas    = arr[LAG_FRAMES:, 0]
    t_lagged  = arr[:-LAG_FRAMES, 1]
    c_log     = arr[LAG_FRAMES:, 2]
    c_yaw     = arr[LAG_FRAMES:, 3]
    act_lag   = arr[:-LAG_FRAMES, 4]
    act_now   = arr[LAG_FRAMES:, 4]
    # only keep samples where lat was active for the entire torque->measurement window
    keep = (act_lag > 0.5) & (act_now > 0.5) & (v_meas > 5) & (np.abs(t_lagged) > 0.02)
    for i in np.where(keep)[0]:
      all_pairs.append((v_meas[i], t_lagged[i], c_log[i], c_yaw[i]))

  if not all_pairs:
    print('No data')
    return

  arr = np.array(all_pairs)
  v, t, c_log, c_yaw = arr[:,0], arr[:,1], arr[:,2], arr[:,3]
  N = len(arr)
  print(f'\n{N} lagged-paired samples (|torque(t-{LAG_S}s)| > 0.02, lat active across window, vEgo > 5)\n')

  for label, c in [('log curvature (cs.curvature)', c_log),
                   ('yaw_rate / vEgo (livePose)', c_yaw)]:
    print(f'{"="*80}')
    print(f'Source: {label}')
    print(f'{"="*80}')

    y = c * v * v
    for sign, name in [(+1, 'curv = +K·t_lag/v²'), (-1, 'curv = -K·t_lag/v²')]:
      x = sign * t
      K = float(np.sum(x * y) / np.sum(x * x))
      resid = y - K * x
      r2 = 1 - np.sum(resid ** 2) / np.sum((y - y.mean()) ** 2)
      print(f'  {name}:  K = {K:>7.3f}   R² = {r2:>6.3f}')

    print(f'  By speed bin (K via curv = -K·t_lag/v²):')
    print(f'    {"Speed":>14s} | {"N":>6s} | {"K":>7s} | {"R²":>6s} | {"plant_gain at center":>22s}')
    for lo, hi in [(5,10),(10,15),(15,20),(20,25),(25,30)]:
      m = (v >= lo) & (v < hi)
      n = m.sum()
      if n < 50: continue
      x = -t[m]; yy = c[m] * v[m] * v[m]
      K = float(np.sum(x * yy) / np.sum(x * x))
      r2 = 1 - np.sum((yy - K * x) ** 2) / np.sum((yy - yy.mean()) ** 2)
      v_c = (lo + hi) / 2
      print(f'    {lo:>2d}-{hi:<2d} m/s     | {n:>6d} | {K:>7.3f} | {r2:>6.3f} | {K / v_c**2:>22.5f}')
    print()

  print(f'{"="*80}')
  print(f'HIGH-TORQUE SUBSET (|torque(t-{LAG_S}s)| > 0.1) using livePose yaw_rate')
  print(f'{"="*80}')
  m = np.abs(t) > 0.1
  if m.sum() > 50:
    x = -t[m]; yy = c_yaw[m] * v[m] ** 2
    K = float(np.sum(x * yy) / np.sum(x * x))
    r2 = 1 - np.sum((yy - K * x) ** 2) / np.sum((yy - yy.mean()) ** 2)
    print(f'  N={m.sum()}  K = {K:.3f}  R² = {r2:.3f}')


if __name__ == '__main__':
  main()
