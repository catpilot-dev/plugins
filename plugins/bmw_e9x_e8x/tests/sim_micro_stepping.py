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
SPREAD_FRAMES = 20
STEP_PER_FRAME = MAX_STEP / SPREAD_FRAMES     # 0.00208 per CAN frame
PLANT_GAIN_COEFF = 2.0
DELTA_PCT = 0.10
STEPPER_DEADZONE = 0.05


class MicroStepping:
  def __init__(self):
    self.torque = 0.0
    self.step_remaining = 0.0
    self.measured = 0.0
    self.desired = 0.0
    self.error = 0.0
    self.prev_error = 0.0
    self.plant_gain = 0.0

  def update(self, active, v_ego, desired_curv, yaw_rate, livepose_updated):
    v = max(v_ego, 5.0)
    self.plant_gain = PLANT_GAIN_COEFF / (v ** 2)
    self.desired = float(desired_curv)

    if livepose_updated:
      self.measured = float(yaw_rate) / v

    # base torque from measured curvature
    self.torque = max(-1.0, min(1.0, self.measured / self.plant_gain))

    if livepose_updated:
      self.prev_error = self.error
      self.error = self.desired - self.measured
      d_err = self.error - self.prev_error
      same_sign = self.error * self.prev_error > 0
      worsening = same_sign and abs(d_err) > DELTA_PCT * abs(self.prev_error) and abs(self.error) > abs(self.prev_error)
      sign_changed = self.prev_error != 0 and not same_sign

      if worsening:
        self.step_remaining = max(-MAX_STEP, min(MAX_STEP, d_err / self.plant_gain))
      elif sign_changed:
        self.step_remaining = max(-MAX_STEP, min(MAX_STEP, self.error / self.plant_gain))
      else:
        self.step_remaining = 0.0

    if self.step_remaining != 0.0:
      small = max(-STEP_PER_FRAME, min(STEP_PER_FRAME, self.step_remaining))
      self.torque += small
      self.step_remaining -= small

    out = max(-1.0, min(1.0, self.torque))
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
            'sim_meas': ms.measured, 'sim_err': ms.error,
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
