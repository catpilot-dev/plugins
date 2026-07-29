import glob, os, zstandard, numpy as np
from cereal import log as caplog
from opendbc.can import CANParser
from opendbc.car import Bus

DBC_FILE = '/data/plugins-runtime/bmw_e9x_e8x/dbc/bmw_e9x_e8x.dbc'
ROUTE = '/data/media/0/realdata/0000021f--'
segments = sorted(glob.glob(ROUTE + '*'), key=lambda d: int(d.rsplit('--', 1)[-1]))

def read_rlog(path):
  dctx = zstandard.ZstdDecompressor()
  with open(path, 'rb') as f:
    return list(caplog.Event.read_multiple_bytes(dctx.decompress(f.read(), max_output_size=200*1024*1024)))

# Create CAN parser for Speed message (0x1A0 = 416)
parser = CANParser(DBC_FILE, [("Speed", 50)], Bus.pt)

lat_active = False
v_ego = 0

can_la_all = []
pose_la_all = []
torque_all = []
speed_all = []

for seg_dir in segments:
  rlog_path = os.path.join(seg_dir, 'rlog.zst')
  if not os.path.exists(rlog_path): continue
  seg_id = seg_dir.rsplit('--', 1)[-1]

  events = sorted(read_rlog(rlog_path), key=lambda e: e.logMonoTime)

  seg_can_la = []
  seg_pose_la = []

  for evt in events:
    w = evt.which()
    if w == 'carControl':
      lat_active = evt.carControl.latActive
    elif w == 'carState':
      v_ego = evt.carState.vEgo
    elif w == 'can' and lat_active and v_ego > 15/3.6:
      # Parse CAN messages
      can_strings = []
      for msg in evt.can:
        can_strings.append((msg.address, msg.dat, msg.src))
      parser.update_strings([bytes([m[2]]) + m[1] for m in can_strings if m[2] == 0])
      can_lat = parser.vl.get('Speed', {}).get('LatlAcc', None)
      if can_lat is not None:
        seg_can_la.append(can_lat)
    elif w == 'livePose' and lat_active and v_ego > 15/3.6:
      pose = evt.livePose
      if pose.angularVelocityDevice.valid:
        pose_la = pose.angularVelocityDevice.z * v_ego
        seg_pose_la.append(pose_la)
    elif w == 'carOutput' and lat_active and v_ego > 15/3.6:
      torque_all.append(-evt.carOutput.actuatorsOutput.torque)
      speed_all.append(v_ego)

  # Align by resampling to same length
  n = min(len(seg_can_la), len(seg_pose_la))
  if n > 50:
    can_la_all.extend(seg_can_la[:n])
    pose_la_all.extend(seg_pose_la[:n])

can_la = np.array(can_la_all)
pose_la = np.array(pose_la_all)

print('CAN LatlAcc vs livePose lat_accel:')
print('  Samples: ' + str(len(can_la)))
print('  CAN:  mean=' + str(round(can_la.mean(), 4)) + ' std=' + str(round(can_la.std(), 4)) + ' range=[' + str(round(can_la.min(), 3)) + ', ' + str(round(can_la.max(), 3)) + ']')
print('  Pose: mean=' + str(round(pose_la.mean(), 4)) + ' std=' + str(round(pose_la.std(), 4)) + ' range=[' + str(round(pose_la.min(), 3)) + ', ' + str(round(pose_la.max(), 3)) + ']')

# Correlation
corr = np.corrcoef(can_la, pose_la)[0, 1]
print('  Correlation: ' + str(round(corr, 4)))

# Difference
diff = can_la - pose_la
print('  Diff (CAN-Pose): mean=' + str(round(diff.mean(), 4)) + ' std=' + str(round(diff.std(), 4)))

# Now fit torque vs CAN lat_accel with various delays
print('')
print('Torque vs CAN LatlAcc (delay sweep):')

n_t = min(len(torque_all), len(can_la_all))
t_arr = np.array(torque_all[:n_t])
cla_arr = np.array(can_la_all[:n_t])

for delay_frames in [0, 2, 4, 6, 8, 10, 12, 15, 20]:
  if delay_frames >= n_t:
    continue
  # Shift: torque[:-delay] vs lat_accel[delay:]
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
  print('  delay=' + str(round(delay_s, 2)) + 's | F=' + str(round(slope, 3)) + ' offset=' + str(round(offset, 4)) + ' R2=' + str(round(r2, 4)) + ' N=' + str(mask.sum()))
