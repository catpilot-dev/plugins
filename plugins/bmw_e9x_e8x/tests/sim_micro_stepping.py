#!/usr/bin/env python3
"""Simulate micro-stepping controller against route logs; compare to stock latcontrol_torque."""
import glob, os, zstandard, numpy as np
from cereal import log as caplog

ROUTE = '/data/media/0/realdata/00000266--1f1715a873--'
MS_TO_KPH = 3.6

# Micro-stepping constants (mirror plugins/bmw_e9x_e8x/register.py)
STEER_MAX = 12.0
STEER_DELTA_UP = 0.1
MAX_STEP = STEER_DELTA_UP * 5 / STEER_MAX     # 0.04167 per measurement frame
SPREAD_FRAMES = 5
STEP_PER_FRAME = MAX_STEP / SPREAD_FRAMES     # 0.00833 per CAN frame
PLANT_GAIN_K = 0.68
PLANT_GAIN_B = 0.0073
UNDERSTEER_MARGIN = 1.3
STEPPER_DEADZONE = 0.01


class MicroStepping:
  def __init__(self):
    self.torque = 0.0
    self.step_remaining = 0.0
    self.desired = 0.0
    self.measured = 0.0
    self.desired_prev = 0.0
    self.measured_prev = 0.0
    self.delta_desired = 0.0
    self.delta_measured = 0.0
    self.plant_gain = 0.0

  def update(self, active, v_ego, desired_curv, yaw_rate, livepose_updated):
    v = max(v_ego, 5.0)
    self.plant_gain = UNDERSTEER_MARGIN * (PLANT_GAIN_K / (v ** 2) + PLANT_GAIN_B)
    self.desired = float(desired_curv)

    if livepose_updated:
      self.measured = float(yaw_rate) / v

    # base torque from measured curvature
    self.torque = max(-1.0, min(1.0, self.measured / self.plant_gain))

    if livepose_updated:
      self.delta_desired = self.desired - self.desired_prev
      self.delta_measured = self.measured - self.measured_prev
      self.desired_prev = self.desired
      self.measured_prev = self.measured
      delta_of_delta = self.delta_desired - self.delta_measured
      correction = delta_of_delta / self.plant_gain
      self.step_remaining = max(-MAX_STEP, min(MAX_STEP, correction))

    if self.step_remaining != 0.0:
      small = max(-STEP_PER_FRAME, min(STEP_PER_FRAME, self.step_remaining))
      self.torque += small
      self.step_remaining -= small

    out = max(-1.0, min(1.0, self.torque))
    self.torque_raw = out  # pre-deadzone torque for deadzone analysis
    if not active or abs(out) < STEPPER_DEADZONE:
      out = 0.0
    return -out


def read_rlog(path):
  dctx = zstandard.ZstdDecompressor()
  with open(path, 'rb') as f:
    return caplog.Event.read_multiple_bytes(dctx.decompress(f.read(), max_output_size=200*1024*1024))


def main():
  segments = sorted(glob.glob(f'{ROUTE}*'), key=lambda d: int(d.rsplit('--', 1)[-1]))
  print(f'Found {len(segments)} segments')

  ms = MicroStepping()
  lat_active = False
  v_ego = 0.0
  steer_angle = 0.0
  yaw_rate = 0.0
  livepose_dirty = False
  samples = []

  for seg_dir in segments:
    rlog_path = os.path.join(seg_dir, 'rlog.zst')
    if not os.path.exists(rlog_path):
      continue
    events = sorted(read_rlog(rlog_path), key=lambda e: e.logMonoTime)

    for evt in events:
      w = evt.which()
      t = evt.logMonoTime * 1e-9

      if w == 'carControl':
        lat_active = evt.carControl.latActive
      elif w == 'carState':
        v_ego = evt.carState.vEgo
        steer_angle = evt.carState.steeringAngleDeg
      elif w == 'livePose':
        yaw_rate = evt.livePose.angularVelocityDevice.z
        livepose_dirty = True
      elif w == 'controlsState' and v_ego > 3:
        cs = evt.controlsState
        des_curv = cs.desiredCurvature
        meas_curv = cs.curvature
        lac = getattr(cs.lateralControlState, cs.lateralControlState.which())
        stock_out = lac.output if hasattr(lac, 'output') else 0.0

        sim_out = ms.update(lat_active, v_ego, des_curv, yaw_rate, livepose_dirty)
        livepose_dirty = False

        if lat_active:
          samples.append({
            't': t, 'v': v_ego, 'des_curv': des_curv, 'meas_curv': meas_curv,
            'curv_err': des_curv - meas_curv,
            'stock_out': stock_out, 'sim_out': sim_out,
            'sim_raw': ms.torque_raw,
            'sim_meas': ms.measured,
            'sim_err': ms.desired - ms.measured,
            'sim_dod': ms.delta_desired - ms.delta_measured,
            'steer_angle': steer_angle,
          })

  if not samples:
    print('No data')
    return

  v = np.array([s['v'] for s in samples])
  des = np.array([s['des_curv'] for s in samples])
  meas = np.array([s['meas_curv'] for s in samples])
  curv_err = np.array([s['curv_err'] for s in samples])
  stock = np.array([s['stock_out'] for s in samples])
  sim = np.array([s['sim_out'] for s in samples])
  sim_err = np.array([s['sim_err'] for s in samples])
  sim_dod = np.array([s['sim_dod'] for s in samples])
  sim_raw = np.array([s['sim_raw'] for s in samples])
  steer = np.array([s['steer_angle'] for s in samples])
  N = len(samples)

  print(f'\n{"="*80}')
  print(f'OVERALL ({N} engaged samples)')
  print(f'{"="*80}')
  print(f'  Stock torque:  range [{stock.min():.3f}, {stock.max():.3f}]  MAE {np.abs(stock).mean():.3f}')
  print(f'  Sim torque:    range [{sim.min():.3f}, {sim.max():.3f}]  MAE {np.abs(sim).mean():.3f}')
  print(f'  Stock-Sim:     MAE {np.abs(stock-sim).mean():.3f}  RMSE {np.sqrt(np.mean((stock-sim)**2)):.3f}  corr {np.corrcoef(stock, sim)[0,1]:.3f}')
  print(f'  Curv err MAE:  log-stock {np.abs(curv_err).mean():.5f}  sim-internal {np.abs(sim_err).mean():.5f}')
  print(f'  Speed range:   {v.min():.1f} - {v.max():.1f} m/s')

  print(f'\n{"="*80}')
  print('TORQUE BY SPEED')
  print(f'{"="*80}')
  print(f'  {"Speed":>14s} | {"N":>6s} | {"stock MAE":>9s} | {"sim MAE":>9s} | {"Δ MAE":>7s} | {"corr":>5s}')
  for lo, hi in [(5,10),(10,15),(15,20),(20,25),(25,35)]:
    mask = (v >= lo) & (v < hi)
    n = mask.sum()
    if n < 20: continue
    c = np.corrcoef(stock[mask], sim[mask])[0,1] if n > 1 else 0
    print(f'  {lo:>2d}-{hi:<2d} m/s     | {n:>6d} | {np.abs(stock[mask]).mean():>9.3f} | {np.abs(sim[mask]).mean():>9.3f} | {np.abs(stock[mask]-sim[mask]).mean():>7.3f} | {c:>5.2f}')

  print(f'\n{"="*80}')
  print('TORQUE BY CURVATURE')
  print(f'{"="*80}')
  print(f'  {"Curvature":>22s} | {"N":>6s} | {"stock MAE":>9s} | {"sim MAE":>9s} | {"Δ MAE":>7s}')
  for lo, hi, lbl in [(0, 0.0005, '< 0.0005 (straight)'),
                       (0.0005, 0.001, '0.0005 - 0.001'),
                       (0.001, 0.003, '0.001 - 0.003'),
                       (0.003, 0.01, '0.003 - 0.010'),
                       (0.01, 1.0, '> 0.010')]:
    mask = (np.abs(des) >= lo) & (np.abs(des) < hi)
    n = mask.sum()
    if n < 20: continue
    print(f'  {lbl:>22s} | {n:>6d} | {np.abs(stock[mask]).mean():>9.3f} | {np.abs(sim[mask]).mean():>9.3f} | {np.abs(stock[mask]-sim[mask]).mean():>7.3f}')

  # Bias check — during straight-line engagement, desired-measured should average to ~0
  # if there's no pipeline bias. Non-zero mean indicates model-vs-gyro offset.
  print(f'\n{"="*80}')
  print('SIGNAL BIAS (desired − measured) — validates the delta-of-delta redesign')
  print(f'{"="*80}')
  print(f'  {"Segment":>22s} | {"N":>6s} | {"mean":>9s} | {"std":>9s} | {"p05":>9s} | {"p95":>9s}')
  for lbl, mask in [
      ('all engaged',           np.ones_like(des, dtype=bool)),
      ('|des|<0.0005 straight', np.abs(des) < 0.0005),
      ('|des|<0.001  straight', np.abs(des) < 0.001),
      ('|des|>0.003  curving',  np.abs(des) > 0.003),
  ]:
    n = mask.sum()
    if n < 20: continue
    e = curv_err[mask]
    print(f'  {lbl:>22s} | {n:>6d} | {e.mean():>+9.6f} | {e.std():>9.6f} | {np.percentile(e,5):>+9.6f} | {np.percentile(e,95):>+9.6f}')

  # delta_of_delta signal magnitude — is it big enough for stepper to act on?
  print(f'\n{"="*80}')
  print('DELTA_OF_DELTA SIGNAL MAGNITUDE (ΔΔ = Δdesired − Δmeasured, 50ms)')
  print(f'{"="*80}')
  print(f'  {"Segment":>22s} | {"N":>6s} | {"|ΔΔ| p50":>9s} | {"|ΔΔ| p95":>9s} | {"|ΔΔ| p99":>9s}')
  for lbl, mask in [
      ('all engaged',           np.ones_like(des, dtype=bool)),
      ('|des|<0.0005 straight', np.abs(des) < 0.0005),
      ('|des|>0.003  curving',  np.abs(des) > 0.003),
  ]:
    n = mask.sum()
    if n < 20: continue
    dd = np.abs(sim_dod[mask])
    print(f'  {lbl:>22s} | {n:>6d} | {np.percentile(dd,50):>9.6f} | {np.percentile(dd,95):>9.6f} | {np.percentile(dd,99):>9.6f}')

  # Deadzone sizing — on straight segments, what does the RAW sim torque look like
  # before the deadzone cuts it? Pick threshold so P95 of straight-line buzz → 0.
  print(f'\n{"="*80}')
  print('STEPPER DEADZONE ANALYSIS — raw sim torque on straights (|desired| < 0.0005)')
  print(f'{"="*80}')
  mask_str = np.abs(des) < 0.0005
  raw_abs = np.abs(sim_raw[mask_str])
  print(f'  N={mask_str.sum()}  mean |raw| = {raw_abs.mean():.4f}  std = {raw_abs.std():.4f}')
  print(f'  |raw| percentiles: '
        f'p50={np.percentile(raw_abs,50):.4f}  '
        f'p75={np.percentile(raw_abs,75):.4f}  '
        f'p90={np.percentile(raw_abs,90):.4f}  '
        f'p95={np.percentile(raw_abs,95):.4f}  '
        f'p99={np.percentile(raw_abs,99):.4f}')

  # How each candidate deadzone suppresses straight-line action
  print(f'\n  Candidate deadzone (fraction of 12Nm) vs straight-line survival & zero-crossings:')
  print(f'    {"deadzone":>10s} | {"Nm":>5s} | {"% cut":>7s} | {"% survives":>10s} | {"zc Hz (all)":>11s} | {"zc Hz (50-90kph)":>17s}')
  mask_mid = (np.abs(des) < 0.0005) & (v*MS_TO_KPH >= 50) & (v*MS_TO_KPH < 90)
  raw_all = sim_raw[mask_str]
  raw_mid = sim_raw[mask_mid]
  for dz in [0.00, 0.01, 0.02, 0.03, 0.05, 0.07, 0.10]:
    out_all = np.where(np.abs(raw_all) < dz, 0.0, raw_all)
    out_mid = np.where(np.abs(raw_mid) < dz, 0.0, raw_mid)
    pct_cut = 100.0 * np.mean(np.abs(raw_all) < dz)
    pct_surv = 100.0 - pct_cut
    zc_all = np.sum(out_all[1:] * out_all[:-1] < 0) / (len(out_all) * 0.01) / 2 if len(out_all) > 10 else 0
    zc_mid = np.sum(out_mid[1:] * out_mid[:-1] < 0) / (len(out_mid) * 0.01) / 2 if len(out_mid) > 10 else 0
    print(f'    {dz:>10.3f} | {dz*12:>5.2f} | {pct_cut:>6.1f}% | {pct_surv:>9.1f}% | {zc_all:>10.2f} | {zc_mid:>16.2f}')

  # Check that the selected deadzone does NOT eat into legitimate curving action
  print(f'\n  Effect of deadzone on CURVING samples (|desired| > 0.003) — want %cut near 0:')
  raw_curv = sim_raw[np.abs(des) > 0.003]
  print(f'    {"deadzone":>10s} | {"% curving cut":>14s}')
  for dz in [0.02, 0.03, 0.05, 0.07, 0.10]:
    pct_cut_curv = 100.0 * np.mean(np.abs(raw_curv) < dz)
    print(f'    {dz:>10.3f} | {pct_cut_curv:>13.1f}%')

  print(f'\n{"="*80}')
  print('STRAIGHT-LANE OUTPUT JITTER (|desired| < 0.001)')
  print(f'{"="*80}')
  print(f'  {"Speed":>10s} | {"N":>6s} | {"stock std":>9s} | {"sim std":>9s} | {"stock zc":>9s} | {"sim zc":>8s}')
  for lo, hi in [(40,55),(55,70),(70,80),(80,90),(90,100)]:
    mask = (v*MS_TO_KPH >= lo) & (v*MS_TO_KPH < hi) & (np.abs(des) < 0.001)
    n = mask.sum()
    if n < 50: continue
    s_st = stock[mask]; s_si = sim[mask]
    zc_st = np.sum(s_st[1:] * s_st[:-1] < 0) / (len(s_st) * 0.01) / 2 if len(s_st) > 10 else 0
    zc_si = np.sum(s_si[1:] * s_si[:-1] < 0) / (len(s_si) * 0.01) / 2 if len(s_si) > 10 else 0
    print(f'  {lo:>3d}-{hi:<3d}kph | {n:>6d} | {s_st.std():>9.3f} | {s_si.std():>9.3f} | {zc_st:>7.1f}Hz | {zc_si:>6.1f}Hz')


if __name__ == '__main__':
  main()
