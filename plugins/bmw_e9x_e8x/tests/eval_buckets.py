import glob, os, zstandard, numpy as np
from cereal import log as caplog

ROUTE = '/data/media/0/realdata/0000021f--'
segments = sorted(glob.glob(ROUTE + '*'), key=lambda d: int(d.rsplit('--', 1)[-1]))

def read_rlog(path):
  dctx = zstandard.ZstdDecompressor()
  with open(path, 'rb') as f:
    return list(caplog.Event.read_multiple_bytes(dctx.decompress(f.read(), max_output_size=200*1024*1024)))

lat_active = False
v_ego = 0
lag = 0.0

bounds = [(-0.5,-0.3),(-0.3,-0.2),(-0.2,-0.1),(-0.1,0),(0,0.1),(0.1,0.2),(0.2,0.3),(0.3,0.5)]

print('seg | speed |  [-0.5,-0.3) [-0.3,-0.2) [-0.2,-0.1)  [-0.1,0)    [0,0.1)   [0.1,0.2)  [0.2,0.3)  [0.3,0.5) | slope_lo slope_mid slope_hi')

for seg_dir in segments:
  rlog_path = os.path.join(seg_dir, 'rlog.zst')
  if not os.path.exists(rlog_path): continue
  seg_id = seg_dir.rsplit('--', 1)[-1]

  events = sorted(read_rlog(rlog_path), key=lambda e: e.logMonoTime)

  torques = []
  lat_accels = []
  speeds = []

  for evt in events:
    w = evt.which()
    if w == 'carControl':
      lat_active = evt.carControl.latActive
    elif w == 'carState':
      v_ego = evt.carState.vEgo
    elif w == 'liveDelay':
      lag = evt.liveDelay.lateralDelay
    elif w == 'carOutput' and lat_active and v_ego > 15/3.6:
      torques.append(-evt.carOutput.actuatorsOutput.torque)
    elif w == 'livePose' and lat_active and v_ego > 15/3.6:
      # lat_accel = yaw_rate * v_ego
      pose = evt.livePose
      if pose.angularVelocityDevice.valid:
        yaw = pose.angularVelocityDevice.z
        la = yaw * v_ego
        lat_accels.append(la)
        speeds.append(v_ego)

  if len(torques) < 100 or len(lat_accels) < 100:
    continue

  t = np.array(torques[:len(lat_accels)])
  la = np.array(lat_accels[:len(t)])
  s = np.array(speeds[:len(t)])

  # Bucket counts
  counts = []
  for lo, hi in bounds:
    counts.append(int(np.sum((t >= lo) & (t < hi))))

  # Piecewise slope: lat_accel / torque for different ranges
  def slope(mask):
    if mask.sum() < 20:
      return 0.0
    return float(np.abs(la[mask]).mean() / np.abs(t[mask]).mean()) if np.abs(t[mask]).mean() > 0.01 else 0.0

  lo_mask = (np.abs(t) >= 0.02) & (np.abs(t) < 0.1)
  mid_mask = (np.abs(t) >= 0.1) & (np.abs(t) < 0.2)
  hi_mask = (np.abs(t) >= 0.2) & (np.abs(t) < 0.5)

  s_lo = slope(lo_mask)
  s_mid = slope(mid_mask)
  s_hi = slope(hi_mask)

  avg_speed = s.mean() * 3.6

  c_str = ' '.join(str(c).rjust(10) for c in counts)
  print(str(seg_id).rjust(3) + ' | ' + str(round(avg_speed, 0)).rjust(5) + ' | ' + c_str + ' | ' +
        str(round(s_lo, 2)).rjust(8) + ' ' + str(round(s_mid, 2)).rjust(8) + ' ' + str(round(s_hi, 2)).rjust(8))
