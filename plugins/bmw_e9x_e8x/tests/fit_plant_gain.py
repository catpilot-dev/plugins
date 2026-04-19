#!/usr/bin/env python3
"""Empirically fit PLANT_GAIN_COEFF K in: desired_curvature · v² = K · torque_output.

Pairs the stock controller's commanded torque(t) with the desired curvature
the plan wanted at t+0.5s (which IS what cs.desiredCurvature represents —
modelV2 plan at lat_action_t≈0.5s). The lookahead is baked into the desired
signal, so the pairing is a direct plant-gain measurement with no manual lag
correction.

Assumes the stock controller is tracking well enough that commanded torque
produces the intended future curvature (i.e., steady-state consistency).
"""
import glob, os, zstandard, numpy as np
from cereal import log as caplog

ROUTE = '/data/media/0/realdata/00000266--1f1715a873--'
MS_TO_KPH = 3.6


def read_rlog(path):
  dctx = zstandard.ZstdDecompressor()
  with open(path, 'rb') as f:
    return caplog.Event.read_multiple_bytes(dctx.decompress(f.read(), max_output_size=200*1024*1024))


def main():
  segments = sorted(glob.glob(f'{ROUTE}*'), key=lambda d: int(d.rsplit('--', 1)[-1]))
  print(f'Found {len(segments)} segments')

  lat_active = False
  v_ego = 0.0
  samples = []  # (v, torque_out, desired_curv, measured_curv_log)

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
        if abs(torque_out) > 0.02:
          samples.append((v_ego, torque_out, cs.desiredCurvature, cs.curvature))

  if not samples:
    print('No data')
    return

  arr = np.array(samples)
  v, t, des, meas = arr[:,0], arr[:,1], arr[:,2], arr[:,3]
  N = len(arr)
  print(f'\n{N} engaged samples (|torque| > 0.02, lat_active, vEgo > 5)\n')

  # Fit: desired_curv · v² = K · torque_output (with sign determination)
  print(f'{"="*80}')
  print('FIT: desired_curv(t+0.5s) · v(t)² = K · torque_output(t)')
  print(f'{"="*80}')

  y_des = des * v * v
  for sign, name in [(+1, 'des = +K·t/v²'), (-1, 'des = -K·t/v²')]:
    x = sign * t
    K = float(np.sum(x * y_des) / np.sum(x * x))
    resid = y_des - K * x
    r2 = 1 - np.sum(resid ** 2) / np.sum((y_des - y_des.mean()) ** 2)
    print(f'  {name}:  K = {K:>7.3f}   R² = {r2:>6.3f}')

  print(f'  By speed bin (K via des = -K·t/v²):')
  print(f'    {"Speed":>14s} | {"N":>6s} | {"K":>7s} | {"R²":>6s} | {"plant_gain at center":>22s}')
  for lo, hi in [(5,10),(10,15),(15,20),(20,25),(25,30)]:
    m = (v >= lo) & (v < hi)
    n = m.sum()
    if n < 50: continue
    x = -t[m]; yy = des[m] * v[m] * v[m]
    K = float(np.sum(x * yy) / np.sum(x * x))
    r2 = 1 - np.sum((yy - K * x) ** 2) / np.sum((yy - yy.mean()) ** 2)
    v_c = (lo + hi) / 2
    print(f'    {lo:>2d}-{hi:<2d} m/s     | {n:>6d} | {K:>7.3f} | {r2:>6.3f} | {K / v_c**2:>22.5f}')

  print(f'  High-torque subset (|torque| > 0.1):')
  m = np.abs(t) > 0.1
  if m.sum() > 50:
    x = -t[m]; yy = des[m] * v[m] ** 2
    K = float(np.sum(x * yy) / np.sum(x * x))
    r2 = 1 - np.sum((yy - K * x) ** 2) / np.sum((yy - yy.mean()) ** 2)
    print(f'    N={m.sum()}  K = {K:.3f}  R² = {r2:.3f}')

  # Two-parameter fit: plant_gain = K/v² + b, i.e. desired = (K/v² + b) · (-torque)
  # Linear regression: desired = K · (-torque/v²) + b · (-torque)
  print(f'\n{"="*80}')
  print('TWO-PARAM FIT: plant_gain = K/v² + b   (desired = K·(-t)/v² + b·(-t))')
  print(f'{"="*80}')
  y_all = des
  X = np.column_stack([-t / (v * v), -t])
  coef, _, _, _ = np.linalg.lstsq(X, y_all, rcond=None)
  K_2p, b_2p = float(coef[0]), float(coef[1])
  resid = y_all - X @ coef
  r2_2p = 1 - np.sum(resid ** 2) / np.sum((y_all - y_all.mean()) ** 2)
  print(f'  All samples:       K = {K_2p:>7.3f}  b = {b_2p:>+9.6f}   R² = {r2_2p:.3f}')

  # High-torque subset
  m = np.abs(t) > 0.1
  if m.sum() > 50:
    X_hi = np.column_stack([-t[m] / (v[m] * v[m]), -t[m]])
    y_hi = des[m]
    coef_hi, _, _, _ = np.linalg.lstsq(X_hi, y_hi, rcond=None)
    K_hi, b_hi = float(coef_hi[0]), float(coef_hi[1])
    resid_hi = y_hi - X_hi @ coef_hi
    r2_hi = 1 - np.sum(resid_hi ** 2) / np.sum((y_hi - y_hi.mean()) ** 2)
    print(f'  |torque| > 0.1:    K = {K_hi:>7.3f}  b = {b_hi:>+9.6f}   R² = {r2_hi:.3f}')
    # Sample the fitted plant_gain at common speeds for the controller
    print(f'  Fitted plant_gain at common speeds:')
    for v_test in [8, 12, 15, 18, 22]:
      pg = K_hi / v_test**2 + b_hi
      pg_old = 2.5 / v_test**2
      print(f'    v={v_test} m/s: plant_gain = {pg:.5f}  (current K=2.5/v² → {pg_old:.5f})')

  # 3-param fit: plant_gain = K/v² + B/v + C
  print(f'\n{"="*80}')
  print('THREE-PARAM FIT: plant_gain = K/v² + B/v + C   (des = K·(-t)/v² + B·(-t)/v + C·(-t))')
  print(f'{"="*80}')
  m = np.abs(t) > 0.1
  X3 = np.column_stack([-t[m] / (v[m] * v[m]), -t[m] / v[m], -t[m]])
  y3 = des[m]
  coef3, _, _, _ = np.linalg.lstsq(X3, y3, rcond=None)
  K3, B3, C3 = float(coef3[0]), float(coef3[1]), float(coef3[2])
  r2_3 = 1 - float(np.sum((y3 - X3 @ coef3) ** 2)) / float(np.sum((y3 - y3.mean()) ** 2))
  print(f'  |torque| > 0.1:   K = {K3:>8.3f}  B = {B3:>+9.4f}  C = {C3:>+9.6f}   R² = {r2_3:.3f}')

  # Apples-to-apples: predict desired_curv directly, compare R² of all models
  print(f'\n{"="*80}')
  print('R² COMPARISON — all models predicting desired_curvature directly')
  print(f'{"="*80}')
  y = des[m]; y_var = float(np.sum((y - y.mean()) ** 2))
  pred_1 = (2.203 / (v[m] ** 2)) * (-t[m])
  pred_2 = (K_hi / (v[m] ** 2) + b_hi) * (-t[m])
  pred_3 = (K3 / (v[m] ** 2) + B3 / v[m] + C3) * (-t[m])
  r2_1 = 1 - float(np.sum((y - pred_1) ** 2)) / y_var
  r2_2 = 1 - float(np.sum((y - pred_2) ** 2)) / y_var
  r2_3c = 1 - float(np.sum((y - pred_3) ** 2)) / y_var
  print(f'  1-param (K/v², K=2.203):                          R² = {r2_1:.3f}')
  print(f'  2-param (K/v²+b, K={K_hi:.3f}, b={b_hi:+.5f}):              R² = {r2_2:.3f}')
  print(f'  3-param (K/v²+B/v+C, K={K3:.2f}, B={B3:+.4f}, C={C3:+.5f}): R² = {r2_3c:.3f}')
  print(f'\n  Per-speed RMSE of desired prediction:')
  print(f'    {"Speed":>14s} | {"N":>6s} | {"1-param":>9s} | {"2-param":>9s} | {"3-param":>9s} | {"best":>7s}')
  for lo, hi in [(5,10),(10,15),(15,20),(20,25)]:
    mm = (v[m] >= lo) & (v[m] < hi)
    if mm.sum() < 50: continue
    r1 = np.sqrt(np.mean((y[mm] - pred_1[mm]) ** 2))
    r2 = np.sqrt(np.mean((y[mm] - pred_2[mm]) ** 2))
    r3 = np.sqrt(np.mean((y[mm] - pred_3[mm]) ** 2))
    best = '3-param' if r3 <= min(r1,r2) else ('2-param' if r2 <= r1 else '1-param')
    print(f'    {lo:>2d}-{hi:<2d} m/s     | {mm.sum():>6d} | {r1:>9.6f} | {r2:>9.6f} | {r3:>9.6f} | {best:>7s}')

  print(f'\n  Fitted 3-param plant_gain at common speeds:')
  for v_test in [8, 12, 15, 18, 22]:
    pg3 = K3 / v_test**2 + B3 / v_test + C3
    pg2 = K_hi / v_test**2 + b_hi
    pg1 = 2.5 / v_test**2
    print(f'    v={v_test} m/s: 3-param={pg3:.5f}  2-param={pg2:.5f}  K=2.5/v²={pg1:.5f}')

  # Sanity: compare to measured-curvature fit (same pairing, no lag — worse by construction)
  print(f'\n{"="*80}')
  print('SANITY: measured_curv(t) · v² = K · torque(t) — same pairing, no lag correction')
  print(f'{"="*80}')
  for sign, name in [(+1, 'meas = +K·t/v²'), (-1, 'meas = -K·t/v²')]:
    x = sign * t; yy = meas * v * v
    K = float(np.sum(x * yy) / np.sum(x * x))
    r2 = 1 - np.sum((yy - K * x) ** 2) / np.sum((yy - yy.mean()) ** 2)
    print(f'  {name}:  K = {K:>7.3f}   R² = {r2:>6.3f}')


if __name__ == '__main__':
  main()
