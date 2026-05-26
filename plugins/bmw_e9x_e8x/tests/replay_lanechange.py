#!/usr/bin/env python3
"""Analyze lane change behavior in a specific segment."""
import sys
import os
import zstandard
from cereal import log as caplog

MS_TO_KPH = 3.6
ROUTE = "/data/media/0/realdata/0000021b--bb850498a7--"

def read_rlog(path):
  dctx = zstandard.ZstdDecompressor()
  with open(path, 'rb') as f:
    raw = dctx.decompress(f.read(), max_output_size=200 * 1024 * 1024)
  return caplog.Event.read_multiple_bytes(raw)

def analyze_segment(seg_id):
  rlog_path = os.path.join(f"{ROUTE}{seg_id}", "rlog.zst")
  if not os.path.exists(rlog_path):
    print(f"  Segment {seg_id}: rlog not found")
    return

  events = read_rlog(rlog_path)
  sorted_events = sorted(events, key=lambda e: e.logMonoTime)

  t0 = None
  last_v_ego = 0
  last_lc_state = None
  last_lc_dir = None
  last_desire = None
  last_blinker_l = False
  last_blinker_r = False
  last_curvature = 0
  last_steering_pressed = False
  last_gas_pressed = False
  last_lat_active = None

  print(f"\n{'='*120}")
  print(f"SEGMENT {seg_id}")
  print(f"{'='*120}")
  print(f"{'time':>7s} | {'v_ego':>6s} | {'event'}")
  print(f"{'-'*7} | {'-'*6} | {'-'*105}")

  for evt in sorted_events:
    which = evt.which()
    t = evt.logMonoTime * 1e-9
    if t0 is None:
      t0 = t
    rel_t = t - t0

    if which == "carState":
      cs = evt.carState
      last_v_ego = cs.vEgo * MS_TO_KPH
      bl = cs.leftBlinker
      br = cs.rightBlinker
      if bl != last_blinker_l or br != last_blinker_r:
        print(f"{rel_t:7.1f} | {last_v_ego:5.1f} | BLINKER: L={bl} R={br}")
        last_blinker_l = bl
        last_blinker_r = br
      sp = cs.steeringPressed
      gp = cs.gasPressed
      if sp != last_steering_pressed:
        print(f"{rel_t:7.1f} | {last_v_ego:5.1f} | STEERING_PRESSED: {sp}  (torque={cs.steeringTorque:.1f})")
        last_steering_pressed = sp
      if gp != last_gas_pressed:
        print(f"{rel_t:7.1f} | {last_v_ego:5.1f} | GAS_PRESSED: {gp}")
        last_gas_pressed = gp

    elif which == "carControl":
      cc = evt.carControl
      if cc.latActive != last_lat_active:
        print(f"{rel_t:7.1f} | {last_v_ego:5.1f} | LAT_ACTIVE: {cc.latActive}")
        last_lat_active = cc.latActive

    elif which == "modelV2":
      m = evt.modelV2
      meta = m.meta
      lc_state = str(meta.laneChangeState)
      lc_dir = str(meta.laneChangeDirection)

      if lc_state != last_lc_state or lc_dir != last_lc_dir:
        probs = list(m.laneLineProbs) if hasattr(m, 'laneLineProbs') else []
        prob_str = ' '.join(f'{p:.2f}' for p in probs)
        print(f"{rel_t:7.1f} | {last_v_ego:5.1f} | MODEL: lc_state={lc_state:<20s} lc_dir={lc_dir:<10s} probs=[{prob_str}]")
        last_lc_state = lc_state
        last_lc_dir = lc_dir

    elif which == "drivingModelData":
      dm = evt.drivingModelData
      meta = dm.meta
      lc_state = str(meta.laneChangeState)
      lc_dir = str(meta.laneChangeDirection)
      desire = str(dm.desire) if hasattr(dm, 'desire') else '?'

      if desire != last_desire:
        print(f"{rel_t:7.1f} | {last_v_ego:5.1f} | DESIRE: {desire:<20s} lc_state={lc_state:<20s} lc_dir={lc_dir}")
        last_desire = desire

    elif which == "controlsState":
      cs = evt.controlsState
      last_curvature = cs.desiredCurvature

    elif which == "selfdriveState":
      sd = evt.selfdriveState
      # Check for lat/long active changes
      pass

    elif which == "driverAssistance":
      da = evt.driverAssistance
      if hasattr(da, 'leftLaneDeparture') or hasattr(da, 'rightLaneDeparture'):
        left_dep = da.leftLaneDeparture if hasattr(da, 'leftLaneDeparture') else False
        right_dep = da.rightLaneDeparture if hasattr(da, 'rightLaneDeparture') else False
        if left_dep or right_dep:
          print(f"{rel_t:7.1f} | {last_v_ego:5.1f} | LANE_DEPART: L={left_dep} R={right_dep}")

def main():
  segs = [int(s) for s in sys.argv[1:]] if len(sys.argv) > 1 else [9]
  for seg_id in segs:
    analyze_segment(seg_id)

if __name__ == "__main__":
  main()
