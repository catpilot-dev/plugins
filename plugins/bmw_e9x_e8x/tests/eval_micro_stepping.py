#!/usr/bin/env python3
"""Extended micro-stepping evaluation: curvature metrics, straight-lane oscillation, lane changes."""
import glob, os, zstandard, numpy as np
from cereal import log as caplog

ROUTE = '/data/media/0/realdata/0000025f--9568620447--'
MS_TO_KPH = 3.6

def read_rlog(path):
  dctx = zstandard.ZstdDecompressor()
  with open(path, 'rb') as f:
    return caplog.Event.read_multiple_bytes(dctx.decompress(f.read(), max_output_size=200*1024*1024))

def main():
  segments = sorted(glob.glob(f'{ROUTE}*'), key=lambda d: int(d.rsplit('--', 1)[-1]))
  print(f'Found {len(segments)} segments')

  lat_active = False
  v_ego = 0
  steer_angle = 0
  all_samples = []
  # Lane change tracking
  lc_state = 'off'
  lc_start_time = None
  lc_dir = 'none'
  lc_samples = []
  all_lc_events = []

  for seg_dir in segments:
    rlog_path = os.path.join(seg_dir, 'rlog.zst')
    if not os.path.exists(rlog_path): continue
    seg_id = seg_dir.rsplit('--', 1)[-1]
    events = sorted(read_rlog(rlog_path), key=lambda e: e.logMonoTime)

    for evt in events:
      w = evt.which()
      t = evt.logMonoTime * 1e-9

      if w == 'carControl':
        lat_active = evt.carControl.latActive
      elif w == 'carState':
        v_ego = evt.carState.vEgo
        steer_angle = evt.carState.steeringAngleDeg
      elif w == 'drivingModelData':
        dm = evt.drivingModelData
        meta = dm.meta
        new_state = str(meta.laneChangeState).split('.')[-1]
        new_dir = str(meta.laneChangeDirection).split('.')[-1]

        if new_state == 'laneChangeStarting' and lc_state != 'laneChangeStarting':
          lc_start_time = t
          lc_dir = new_dir
          lc_samples = []
        elif new_state in ('off', 'preLaneChange') and lc_state in ('laneChangeStarting', 'laneChangeFinishing'):
          if lc_start_time and lc_samples:
            errs = [s['curv_err'] for s in lc_samples]
            mae = np.mean(np.abs(errs))
            spd = np.mean([s['v'] for s in lc_samples]) * MS_TO_KPH
            all_lc_events.append({'seg': seg_id, 'dir': lc_dir, 'speed': spd,
                                  'duration': t - lc_start_time, 'mae': mae, 'n': len(lc_samples)})
          lc_start_time = None
        lc_state = new_state
      elif w == 'controlsState' and lat_active and v_ego > 3:
        cs = evt.controlsState
        lac = getattr(cs.lateralControlState, cs.lateralControlState.which())
        if hasattr(lac, 'error') and hasattr(lac, 'output'):
          sample = {
            't': t,
            'v': v_ego,
            'des_curv': cs.desiredCurvature,
            'meas_curv': cs.curvature,
            'curv_err': cs.desiredCurvature - cs.curvature,
            'lac_error': lac.error,
            'torque': lac.output,
            'steer_angle': steer_angle,
            'p': lac.p if hasattr(lac, 'p') else 0,
            'i': lac.i if hasattr(lac, 'i') else 0,
            'f': lac.f if hasattr(lac, 'f') else 0,
          }
          all_samples.append(sample)
          if lc_start_time:
            lc_samples.append(sample)

  if not all_samples:
    print('No data found')
    return

  # Convert to arrays
  N = len(all_samples)
  v = np.array([s['v'] for s in all_samples])
  des_curv = np.array([s['des_curv'] for s in all_samples])
  meas_curv = np.array([s['meas_curv'] for s in all_samples])
  curv_err = np.array([s['curv_err'] for s in all_samples])
  lac_err = np.array([s['lac_error'] for s in all_samples])
  torque = np.array([s['torque'] for s in all_samples])
  steer = np.array([s['steer_angle'] for s in all_samples])

  # ========== OVERALL ==========
  abs_err = np.abs(curv_err)
  print(f'\n{"="*80}')
  print(f'OVERALL ({N} engaged samples)')
  print(f'{"="*80}')
  print(f'  Error MAE:     {abs_err.mean():.5f}')
  print(f'  Error P95:     {np.percentile(abs_err, 95):.5f}')
  print(f'  Error P99:     {np.percentile(abs_err, 99):.5f}')
  print(f'  Correlation:   {np.corrcoef(des_curv, meas_curv)[0,1]:.3f}')
  print(f'  Torque range:  [{torque.min():.3f}, {torque.max():.3f}]')
  print(f'  Torque MAE:    {np.abs(torque).mean():.3f}')
  print(f'  Speed range:   {v.min():.1f} - {v.max():.1f} m/s')

  # ========== BY SPEED (m/s) ==========
  print(f'\n{"="*80}')
  print('BY SPEED')
  print(f'{"="*80}')
  print(f'  {"Speed":>25s} | {"N":>6s} | {"MAE":>8s} | {"Std":>8s}')
  for lo, hi in [(5,10),(10,15),(15,20),(20,25),(25,35)]:
    mask = (v >= lo) & (v < hi)
    n = mask.sum()
    if n < 20: continue
    ae = np.abs(curv_err[mask])
    print(f'  {lo:>2d}-{hi:<2d} m/s ({lo*MS_TO_KPH:.0f}-{hi*MS_TO_KPH:.0f} km/h) | {n:>6d} | {ae.mean():>8.5f} | {ae.std():>8.5f}')

  # ========== BY CURVATURE ==========
  print(f'\n{"="*80}')
  print('BY CURVATURE')
  print(f'{"="*80}')
  print(f'  {"Curvature":>22s} | {"N":>6s} | {"MAE":>8s} | {"Std":>8s}')
  for lo, hi, lbl in [(0, 0.0005, '< 0.0005 (straight)'),
                       (0.0005, 0.001, '0.0005 - 0.001'),
                       (0.001, 0.003, '0.001 - 0.003'),
                       (0.003, 0.01, '0.003 - 0.010'),
                       (0.01, 1.0, '> 0.010')]:
    mask = (np.abs(des_curv) >= lo) & (np.abs(des_curv) < hi)
    n = mask.sum()
    if n < 20: continue
    ae = np.abs(curv_err[mask])
    print(f'  {lbl:>22s} | {n:>6d} | {ae.mean():>8.5f} | {ae.std():>8.5f}')

  # ========== STRAIGHT-LANE OSCILLATION ==========
  print(f'\n{"="*80}')
  print('STRAIGHT-LANE OSCILLATION (|desired| < 0.001)')
  print(f'{"="*80}')
  print(f'  {"Speed":>8s} | {"N":>6s} | {"Steer std":>9s} | {"Osc freq":>8s} | {"Error MAE":>9s}')
  for lo, hi in [(40,55),(55,70),(70,80),(80,90),(90,100)]:
    mask = (v*MS_TO_KPH >= lo) & (v*MS_TO_KPH < hi) & (np.abs(des_curv) < 0.001)
    n = mask.sum()
    if n < 50: continue
    e = lac_err[mask]
    ae = np.abs(curv_err[mask])
    sa = steer[mask]
    osc = np.sum(e[1:] * e[:-1] < 0) / (len(e) * 0.01) / 2 if len(e) > 10 else 0
    print(f'  {lo:>3d}-{hi:<3d}kph | {n:>6d} | {sa.std():>8.2f}d | {osc:>7.1f}Hz | {ae.mean():>9.5f}')

  # ========== LANE CHANGES ==========
  print(f'\n{"="*80}')
  print(f'LANE CHANGES ({len(all_lc_events)} found)')
  print(f'{"="*80}')
  if all_lc_events:
    print(f'  {"LC":>3s} | {"Direction":>9s} | {"Speed":>8s} | {"Duration":>8s} | {"Error MAE":>10s}')
    for i, lc in enumerate(all_lc_events):
      print(f'  {i+1:>3d} | {lc["dir"]:>9s} | {lc["speed"]:>6.0f}kph | {lc["duration"]:>7.1f}s | {lc["mae"]:>10.5f}')
    maes = [lc['mae'] for lc in all_lc_events]
    print(f'\n  Mean MAE: {np.mean(maes):.5f}, Best: {min(maes):.5f}, Worst: {max(maes):.5f}')
    print(f'  Mean duration: {np.mean([lc["duration"] for lc in all_lc_events]):.1f}s')

if __name__ == '__main__':
  main()
