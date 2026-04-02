import glob, os, zstandard, numpy as np
from cereal import log as caplog
from collections import deque

ROUTE = '/data/media/0/realdata/0000021f--'
segments = sorted(glob.glob(ROUTE + '*'), key=lambda d: int(d.rsplit('--', 1)[-1]))
DT = 0.05  # 20Hz
HIST_LEN = int(5.0 / DT)  # 5 second history

def read_rlog(path):
  dctx = zstandard.ZstdDecompressor()
  with open(path, 'rb') as f:
    return list(caplog.Event.read_multiple_bytes(dctx.decompress(f.read(), max_output_size=200*1024*1024)))

# Test multiple delay values
delays_to_test = [0.0, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8]

for test_delay in delays_to_test:
  torque_times = deque(maxlen=HIST_LEN)
  torque_vals = deque(maxlen=HIST_LEN)
  lat_active = False
  v_ego = 0
  pairs_t = []
  pairs_la = []

  for seg_dir in segments:
    rlog_path = os.path.join(seg_dir, 'rlog.zst')
    if not os.path.exists(rlog_path): continue
    events = sorted(read_rlog(rlog_path), key=lambda e: e.logMonoTime)
    for evt in events:
      w = evt.which()
      t = evt.logMonoTime * 1e-9
      if w == 'carControl':
        lat_active = evt.carControl.latActive
      elif w == 'carState':
        v_ego = evt.carState.vEgo
      elif w == 'carOutput' and lat_active and v_ego > 15/3.6:
        steer = -evt.carOutput.actuatorsOutput.torque
        torque_times.append(t + test_delay)
        torque_vals.append(steer)
      elif w == 'livePose' and lat_active and v_ego > 15/3.6 and len(torque_times) > 10:
        pose = evt.livePose
        if pose.angularVelocityDevice.valid:
          yaw = pose.angularVelocityDevice.z
          la = yaw * v_ego
          steer = float(np.interp(t, list(torque_times), list(torque_vals)))
          if abs(steer) > 0.02 and abs(la) <= 2.0:
            pairs_t.append(steer)
            pairs_la.append(la)

  t_arr = np.array(pairs_t)
  la_arr = np.array(pairs_la)

  # Overall least squares
  A = np.column_stack([t_arr, np.ones(len(t_arr))])
  result = np.linalg.lstsq(A, la_arr, rcond=None)
  slope, offset = result[0]
  la_pred = slope * t_arr + offset
  ss_res = np.sum((la_arr - la_pred)**2)
  ss_tot = np.sum((la_arr - la_arr.mean())**2)
  r2 = 1 - ss_res / ss_tot if ss_tot > 0 else 0

  # Per bucket R^2 for mid range
  mid_mask = (np.abs(t_arr) >= 0.1) & (np.abs(t_arr) < 0.3)
  if mid_mask.sum() > 50:
    A_m = np.column_stack([t_arr[mid_mask], np.ones(mid_mask.sum())])
    res_m = np.linalg.lstsq(A_m, la_arr[mid_mask], rcond=None)
    sl_m = res_m[0][0]
    pred_m = sl_m * t_arr[mid_mask] + res_m[0][1]
    r2_m = 1 - np.sum((la_arr[mid_mask] - pred_m)**2) / np.sum((la_arr[mid_mask] - la_arr[mid_mask].mean())**2)
  else:
    sl_m = 0
    r2_m = 0

  print('delay=' + str(test_delay) + 's | N=' + str(len(t_arr)) + ' | F=' + str(round(slope, 3)) +
        ' R2=' + str(round(r2, 4)) + ' | mid F=' + str(round(sl_m, 3)) + ' R2=' + str(round(r2_m, 4)))
