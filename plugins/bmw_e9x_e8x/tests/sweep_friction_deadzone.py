#!/usr/bin/env python3
"""Sweep FRICTION_DEADZONE on route 00000266 — measure how often friction fires
and its effect on sim torque (especially straight-line activity).
"""
import glob, os, zstandard, numpy as np
from cereal import log as caplog

ROUTE = '/data/media/0/realdata/00000266--1f1715a873--'
PLANT_GAIN_K = 0.68
PLANT_GAIN_B = 0.0073
KP = 0.8
KI = 0.02
I_MAX = 0.005
FRICTION_TORQUE = 0.10


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


def sim_one_deadzone(FRICTION_DEADZONE, samples):
  """samples = list of (v, desired, measured, active). Returns sim torques and stats."""
  torque_arr = []
  integral = 0.0
  fire_count = 0
  straight_fire_count = 0
  straight_total = 0
  for (v, des, meas, lat_active) in samples:
    v = max(v, 5.0)
    plant_gain = PLANT_GAIN_K / (v ** 2) + PLANT_GAIN_B
    err = des - meas
    integral += err
    integral = max(-I_MAX, min(I_MAX, integral))
    if abs(err) > FRICTION_DEADZONE:
      friction_ff = FRICTION_TORQUE if err > 0 else -FRICTION_TORQUE
      fire_count += 1
      if abs(des) < 0.0005:
        straight_fire_count += 1
    else:
      friction_ff = 0.0
    if abs(des) < 0.0005:
      straight_total += 1
    curvature_cmd = meas + KP * err + KI * integral
    torque = max(-1.0, min(1.0, curvature_cmd / plant_gain + friction_ff))
    if lat_active:
      torque_arr.append((v, des, meas, torque, friction_ff))
  return torque_arr, fire_count, straight_fire_count, straight_total


def main():
  segments = sorted(glob.glob(f'{ROUTE}*'), key=lambda d: int(d.rsplit('--', 1)[-1]))
  print(f'Found {len(segments)} segments')

  lat_active = False
  v_ego = 0.0
  yaw_rate = 0.0
  samples = []   # (v, desired, measured, active)

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
        yaw_rate = -evt.carState.yawRate   # flip sign, same as register.py
      elif w == 'controlsState' and v_ego > 3:
        cs = evt.controlsState
        samples.append((v_ego, cs.desiredCurvature, yaw_rate / max(v_ego, 5.0), lat_active))

  print(f'Total samples: {len(samples)}  engaged: {sum(1 for s in samples if s[3])}\n')

  print(f'{"="*90}')
  print(f'FRICTION_DEADZONE sweep — effect on torque stats and firing frequency')
  print(f'{"="*90}')
  print(f'  {"DZ":>10s} | {"fire %":>7s} | {"straight fire %":>15s} | {"sim MAE":>8s} | {"straight abs t":>15s} | {"curve abs t":>12s}')
  for dz in [0.00000, 0.00001, 0.00005, 0.0001, 0.0002, 0.0003]:
    out, fire, str_fire, str_n = sim_one_deadzone(dz, samples)
    if not out: continue
    arr = np.array(out)
    v = arr[:,0]; des = arr[:,1]; meas = arr[:,2]; torque = arr[:,3]
    m_str = np.abs(des) < 0.0005
    m_cur = np.abs(des) > 0.003
    str_pct = 100.0 * fire / len(samples)
    str_fire_pct = 100.0 * str_fire / max(str_n, 1)
    sim_mae = np.abs(torque).mean()
    straight_t = np.abs(torque[m_str]).mean() if m_str.sum() else 0
    curve_t = np.abs(torque[m_cur]).mean() if m_cur.sum() else 0
    print(f'  {dz:>10.5f} | {str_pct:>6.1f}% | {str_fire_pct:>14.1f}% | {sim_mae:>8.4f} | {straight_t:>15.4f} | {curve_t:>12.4f}')


if __name__ == '__main__':
  main()
