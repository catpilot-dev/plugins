#!/usr/bin/env python3
"""Comprehensive lateral performance analysis for a route."""
import glob, os, zstandard, numpy as np
from cereal import log as caplog

ROUTE = '/data/media/0/realdata/0000021e--'
MS_TO_KPH = 3.6

def read_rlog(path):
  dctx = zstandard.ZstdDecompressor()
  with open(path, 'rb') as f:
    return caplog.Event.read_multiple_bytes(dctx.decompress(f.read(), max_output_size=200*1024*1024))

def main():
  segments = sorted(glob.glob(f'{ROUTE}*'), key=lambda d: int(d.rsplit('--', 1)[-1]))
  print(f'Found {len(segments)} segments')

  # Get CarParams
  events = read_rlog(os.path.join(segments[0], 'rlog.zst'))
  CP = None
  for evt in events:
    if evt.which() == 'carParams':
      CP = evt.carParams
      break
  print(f'Car: {CP.carFingerprint}, steerActuatorDelay: {CP.steerActuatorDelay}')

  lat_active = False
  v_ego = 0

  # Per-segment tracking
  seg_data = []
  # Global tracking
  all_errors, all_torques, all_speeds = [], [], []
  all_p, all_i, all_f = [], [], []
  all_des_la, all_act_la = [], []
  delay_trace = []
  torque_trace = []

  for seg_dir in segments:
    rlog_path = os.path.join(seg_dir, 'rlog.zst')
    if not os.path.exists(rlog_path): continue
    seg_id = seg_dir.rsplit('--', 1)[-1]

    events = sorted(read_rlog(rlog_path), key=lambda e: e.logMonoTime)
    seg_errors, seg_speeds = [], []

    for evt in events:
      w = evt.which()
      if w == 'carControl':
        lat_active = evt.carControl.latActive
      elif w == 'carState':
        v_ego = evt.carState.vEgo
      elif w == 'controlsState' and lat_active and v_ego > 3:
        cs = evt.controlsState
        lac = getattr(cs.lateralControlState, cs.lateralControlState.which())
        if hasattr(lac, 'error') and hasattr(lac, 'output'):
          e, t, s = lac.error, lac.output, v_ego
          all_errors.append(e); all_torques.append(t); all_speeds.append(s)
          seg_errors.append(e); seg_speeds.append(s)
          all_p.append(lac.p if hasattr(lac, 'p') else 0)
          all_i.append(lac.i if hasattr(lac, 'i') else 0)
          all_f.append(lac.f if hasattr(lac, 'f') else 0)
          if hasattr(lac, 'actualLateralAccel') and hasattr(lac, 'desiredLateralAccel'):
            all_des_la.append(lac.desiredLateralAccel)
            all_act_la.append(lac.actualLateralAccel)
      elif w == 'liveDelay':
        ld = evt.liveDelay
        delay_trace.append({'seg': seg_id, 'delay': ld.lateralDelay,
                           'est': ld.lateralDelayEstimate, 'std': ld.lateralDelayEstimateStd,
                           'blocks': ld.validBlocks, 'cal': ld.calPerc,
                           'status': str(ld.status).split('.')[-1]})
      elif w == 'liveTorqueParameters':
        lt = evt.liveTorqueParameters
        torque_trace.append({'seg': seg_id, 'valid': lt.liveValid, 'cal': lt.calPerc,
                            'pts': lt.totalBucketPoints,
                            'F_raw': lt.latAccelFactorRaw, 'F_filt': lt.latAccelFactorFiltered,
                            'f_raw': lt.frictionCoefficientRaw, 'f_filt': lt.frictionCoefficientFiltered,
                            'resets': lt.maxResets})

    if seg_errors:
      se = np.array(seg_errors)
      ss = np.array(seg_speeds)
      osc = np.sum(se[1:] * se[:-1] < 0) / (len(se) * 0.01) / 2 if len(se) > 10 else 0
      seg_data.append((seg_id, len(se), ss.mean()*MS_TO_KPH, se.std(), np.percentile(np.abs(se), 95), osc))

  errors = np.array(all_errors)
  torques = np.array(all_torques)
  speeds = np.array(all_speeds)
  p_arr = np.array(all_p)
  i_arr = np.array(all_i)
  f_arr = np.array(all_f)

  # === Overall stats ===
  print(f'\n{"="*100}')
  print(f'OVERALL LATERAL PERFORMANCE ({len(errors)} samples)')
  print(f'{"="*100}')
  print(f'  error:   std={errors.std():.4f}  p95={np.percentile(np.abs(errors), 95):.4f}  mean={errors.mean():.4f}')
  print(f'  torque:  std={torques.std():.4f}')
  print(f'  P term:  std={p_arr.std():.4f}  mean={p_arr.mean():.4f}')
  print(f'  I term:  std={i_arr.std():.4f}  mean={i_arr.mean():.4f}')
  print(f'  F term:  std={f_arr.std():.4f}')
  if all_des_la:
    te = np.array(all_des_la) - np.array(all_act_la)
    print(f'  tracking: err_std={te.std():.4f}  err_p95={np.percentile(np.abs(te), 95):.4f}')
  osc_total = np.sum(errors[1:] * errors[:-1] < 0) / (len(errors) * 0.01) / 2
  print(f'  oscillation: {osc_total:.1f} Hz')

  # === Speed bins ===
  print(f'\n  {"Speed":>10s} | {"N":>6s} | {"err_std":>8s} | {"err_p95":>8s} | {"P_std":>8s} | {"I_mean":>8s} | {"F_std":>8s} | {"KP_eff":>7s}')
  print(f'  {"-"*10} | {"-"*6} | {"-"*8} | {"-"*8} | {"-"*8} | {"-"*8} | {"-"*8} | {"-"*7}')
  for lo, hi in [(20,40),(40,60),(60,80),(80,120)]:
    mask = (speeds*MS_TO_KPH >= lo) & (speeds*MS_TO_KPH < hi)
    n = mask.sum()
    if n < 50: continue
    kp_eff = p_arr[mask].std() / errors[mask].std() if errors[mask].std() > 0.001 else 0
    print(f'  {lo:>3d}-{hi:<3d}kph | {n:>6d} | {errors[mask].std():>8.4f} | {np.percentile(np.abs(errors[mask]), 95):>8.4f} | '
          f'{p_arr[mask].std():>8.4f} | {i_arr[mask].mean():>8.4f} | {f_arr[mask].std():>8.4f} | {kp_eff:>7.3f}')

  # === Per-segment timeline ===
  print(f'\n{"="*100}')
  print('PER-SEGMENT TIMELINE')
  print(f'{"="*100}')
  print(f'  {"seg":>3s} | {"N":>5s} | {"speed":>6s} | {"err_std":>8s} | {"err_p95":>8s} | {"osc_Hz":>6s} | {"delay":>8s} | {"torque_cal":>10s}')
  print(f'  {"-"*3} | {"-"*5} | {"-"*6} | {"-"*8} | {"-"*8} | {"-"*6} | {"-"*8} | {"-"*10}')

  for seg_id, n, spd, estd, ep95, osc in seg_data:
    # Find matching delay/torque for this segment
    d = next((x for x in reversed(delay_trace) if x['seg'] == seg_id), None)
    t = next((x for x in reversed(torque_trace) if x['seg'] == seg_id), None)
    delay_str = f'{d["delay"]:.3f}s {d["status"][:3]}' if d else '—'
    torque_str = f'{t["cal"]:.0f}% {"V" if t["valid"] else "X"}' if t else '—'
    marker = ' <<<' if estd > 0.15 else ''
    print(f'  {seg_id:>3s} | {n:>5d} | {spd:>5.1f} | {estd:>8.4f} | {ep95:>8.4f} | {osc:>5.1f} | {delay_str:>8s} | {torque_str:>10s}{marker}')

  # === LiveDelay convergence ===
  print(f'\n{"="*100}')
  print('LIVEDELAY CONVERGENCE')
  print(f'{"="*100}')
  if delay_trace:
    first = delay_trace[0]
    last = delay_trace[-1]
    print(f'  Start: {first["delay"]:.3f}s ({first["status"]}) blocks={first["blocks"]}')
    print(f'  End:   {last["delay"]:.3f}s ({last["status"]}) blocks={last["blocks"]} cal={last["cal"]}%')
    # Find transitions
    prev_status = None
    for d in delay_trace:
      if d['status'] != prev_status:
        print(f'  seg {d["seg"]}: {prev_status} -> {d["status"]} delay={d["delay"]:.3f}s blocks={d["blocks"]}')
        prev_status = d['status']

  # === LiveTorqueParameters convergence ===
  print(f'\n{"="*100}')
  print('LIVETORQUEPARAMETERS CONVERGENCE')
  print(f'{"="*100}')
  if torque_trace:
    first = torque_trace[0]
    last = torque_trace[-1]
    print(f'  Start: cal={first["cal"]:.0f}% valid={first["valid"]} pts={first["pts"]:.0f} F={first["F_filt"]:.3f} f={first["f_filt"]:.3f}')
    print(f'  End:   cal={last["cal"]:.0f}% valid={last["valid"]} pts={last["pts"]:.0f} F={last["F_filt"]:.3f} f={last["f_filt"]:.3f}')
    if last['F_raw'] > 0:
      print(f'  Raw:   F={last["F_raw"]:.3f} f={last["f_raw"]:.3f}')
    print(f'  Resets: {last["resets"]:.0f}')

if __name__ == '__main__':
  main()
