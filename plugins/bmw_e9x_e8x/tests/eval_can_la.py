import glob, os, zstandard, numpy as np, struct
from cereal import log as caplog

ROUTE = '/data/media/0/realdata/0000021f--'
segments = sorted(glob.glob(ROUTE + '*'), key=lambda d: int(d.rsplit('--', 1)[-1]))

def read_rlog(path):
  dctx = zstandard.ZstdDecompressor()
  with open(path, 'rb') as f:
    return list(caplog.Event.read_multiple_bytes(dctx.decompress(f.read(), max_output_size=200*1024*1024)))

def parse_speed_msg(dat):
  """Parse CAN message 416 (Speed) from DSC. DBC:
    LatlAcc: start=28, length=12, signed, factor=0.025, offset=0
    YawRate: start=40, length=12, signed, factor=0.05, offset=0
  """
  if len(dat) < 8:
    return None, None
  d = bytes(dat)
  # LatlAcc: bits 28-39 (byte3[4:8] + byte4[0:8]) little-endian signed
  raw = ((d[3] >> 4) & 0xF) | (d[4] << 4)
  if raw >= 2048: raw -= 4096  # 12-bit signed
  lat_acc = raw * 0.025

  # YawRate: bits 40-51
  raw_yr = (d[5]) | ((d[6] & 0xF) << 8)
  if raw_yr >= 2048: raw_yr -= 4096
  yaw_rate = raw_yr * 0.05  # deg/s

  return lat_acc, yaw_rate

lat_active = False
v_ego = 0

can_la_list = []
can_yr_list = []
pose_la_list = []
torque_list = []
speed_list = []

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
    elif w == 'can' and lat_active and v_ego > 15/3.6:
      for msg in evt.can:
        if msg.address == 416 and msg.src == 0:
          la, yr = parse_speed_msg(msg.dat)
          if la is not None:
            can_la_list.append(la)
            can_yr_list.append(yr)
    elif w == 'livePose' and lat_active and v_ego > 15/3.6:
      pose = evt.livePose
      if pose.angularVelocityDevice.valid:
        pose_la_list.append(pose.angularVelocityDevice.z * v_ego)
    elif w == 'carOutput' and lat_active and v_ego > 15/3.6:
      torque_list.append(-evt.carOutput.actuatorsOutput.torque)
      speed_list.append(v_ego)

can_la = np.array(can_la_list)
pose_la = np.array(pose_la_list)
torque = np.array(torque_list)

print('Samples: CAN_la=' + str(len(can_la)) + ' pose_la=' + str(len(pose_la)) + ' torque=' + str(len(torque)))

n = min(len(can_la), len(pose_la))
if n > 100:
  c = can_la[:n]
  p = pose_la[:n]
  corr = np.corrcoef(c, p)[0, 1]
  diff = c - p
  print('')
  print('CAN vs Pose lateral acceleration:')
  print('  CAN:  mean=' + str(round(c.mean(), 4)) + ' std=' + str(round(c.std(), 4)))
  print('  Pose: mean=' + str(round(p.mean(), 4)) + ' std=' + str(round(p.std(), 4)))
  print('  Correlation: ' + str(round(corr, 4)))
  print('  Diff: mean=' + str(round(diff.mean(), 4)) + ' std=' + str(round(diff.std(), 4)))

# Torque vs CAN lat_accel delay sweep
print('')
print('Torque vs CAN LatlAcc (delay sweep):')
n_t = min(len(torque), len(can_la))
t_arr = torque[:n_t]
cla_arr = can_la[:n_t]

for delay_frames in [0, 5, 10, 15, 20, 25, 30, 40, 50, 60, 80, 100]:
  if delay_frames >= n_t - 100:
    continue
  if delay_frames == 0:
    t_d = t_arr
    la_d = cla_arr
  else:
    t_d = t_arr[:-delay_frames]
    la_d = cla_arr[delay_frames:]

  mask = (np.abs(t_d) > 0.02) & (np.abs(la_d) <= 2.0)
  if mask.sum() < 100:
    continue

  A = np.column_stack([t_d[mask], np.ones(mask.sum())])
  result = np.linalg.lstsq(A, la_d[mask], rcond=None)
  slope = result[0][0]
  offset = result[0][1]
  pred = slope * t_d[mask] + offset
  ss_res = np.sum((la_d[mask] - pred)**2)
  ss_tot = np.sum((la_d[mask] - la_d[mask].mean())**2)
  r2 = 1 - ss_res / ss_tot if ss_tot > 0 else 0

  delay_s = delay_frames * 0.01
  print('  delay=' + str(round(delay_s, 2)).ljust(5) + 's | F=' + str(round(slope, 3)).ljust(7) + ' R2=' + str(round(r2, 4)).ljust(7) + ' N=' + str(mask.sum()))
