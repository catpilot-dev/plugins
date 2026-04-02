import glob, os, zstandard, numpy as np
from cereal import log as caplog

ROUTE = '/data/media/0/realdata/0000021f--'
segments = sorted(glob.glob(ROUTE + '*'), key=lambda d: int(d.rsplit('--', 1)[-1]))
FRICTION_FACTOR = 1.5

def read_rlog(path):
  dctx = zstandard.ZstdDecompressor()
  with open(path, 'rb') as f:
    return list(caplog.Event.read_multiple_bytes(dctx.decompress(f.read(), max_output_size=200*1024*1024)))

def slope2rot(slope):
  sin = np.sqrt(slope**2 / (slope**2 + 1))
  cos = np.sqrt(1 / (slope**2 + 1))
  return np.array([[cos, -sin], [sin, cos]])

def estimate_params(points):
  """Same SVD fit as torqued.py estimate_params()"""
  try:
    _, _, v = np.linalg.svd(points, full_matrices=False)
    slope, offset = -v.T[0:2, 2] / v.T[2, 2]
    _, spread = np.matmul(points[:, [0, 2]], slope2rot(slope)).T
    friction = np.std(spread) * FRICTION_FACTOR
    return slope, offset, friction
  except:
    return np.nan, np.nan, np.nan

lat_active = False
v_ego = 0

print('seg | speed |  pts | latAccelFactor |  offset  | friction | status')

for seg_dir in segments:
  rlog_path = os.path.join(seg_dir, 'rlog.zst')
  if not os.path.exists(rlog_path): continue
  seg_id = seg_dir.rsplit('--', 1)[-1]

  events = sorted(read_rlog(rlog_path), key=lambda e: e.logMonoTime)

  # Collect torque vs lat_accel pairs (same as torqued)
  raw_torques = []
  raw_lat_accels = []
  raw_vegos = []
  speeds = []

  for evt in events:
    w = evt.which()
    if w == 'carControl':
      lat_active = evt.carControl.latActive
    elif w == 'carState':
      v_ego = evt.carState.vEgo
    elif w == 'carOutput' and lat_active and v_ego > 15/3.6:
      steer = -evt.carOutput.actuatorsOutput.torque
      if abs(steer) > 0.02:
        raw_torques.append(steer)
    elif w == 'livePose' and lat_active and v_ego > 15/3.6:
      pose = evt.livePose
      if pose.angularVelocityDevice.valid:
        yaw = pose.angularVelocityDevice.z
        roll = 0  # simplified
        la = yaw * v_ego - np.sin(roll) * 9.81
        raw_lat_accels.append(la)
        speeds.append(v_ego)

  n = min(len(raw_torques), len(raw_lat_accels))
  if n < 100:
    continue

  t = np.array(raw_torques[:n])
  la = np.array(raw_lat_accels[:n])
  s = np.array(speeds[:n])
  avg_speed = s.mean() * 3.6

  # Filter: lat_accel < 1.0 (same as torqued LAT_ACC_THRESHOLD)
  mask = np.abs(la) <= 1.0
  t_f = t[mask]
  la_f = la[mask]

  if len(t_f) < 50:
    continue

  # Build points matrix: [torque, 1.0, lat_accel] — same as torqued
  points = np.column_stack([t_f, np.ones(len(t_f)), la_f])

  slope, offset, friction = estimate_params(points)

  if np.isnan(slope):
    status = 'FAILED'
  else:
    status = 'OK'

  print(str(seg_id).rjust(3) + ' | ' + str(round(avg_speed, 0)).rjust(5) + ' | ' + str(len(t_f)).rjust(4) + ' | ' +
        str(round(slope, 3)).rjust(14) + ' | ' + str(round(offset, 4)).rjust(8) + ' | ' + str(round(friction, 4)).rjust(8) + ' | ' + status)

# Also do overall estimate
all_t = []
all_la = []
for seg_dir in segments:
  rlog_path = os.path.join(seg_dir, 'rlog.zst')
  if not os.path.exists(rlog_path): continue
  events = sorted(read_rlog(rlog_path), key=lambda e: e.logMonoTime)
  lat_active = False
  v_ego = 0
  for evt in events:
    w = evt.which()
    if w == 'carControl': lat_active = evt.carControl.latActive
    elif w == 'carState': v_ego = evt.carState.vEgo
    elif w == 'carOutput' and lat_active and v_ego > 15/3.6:
      steer = -evt.carOutput.actuatorsOutput.torque
      if abs(steer) > 0.02:
        all_t.append(steer)
    elif w == 'livePose' and lat_active and v_ego > 15/3.6:
      pose = evt.livePose
      if pose.angularVelocityDevice.valid:
        la = pose.angularVelocityDevice.z * v_ego
        all_la.append(la)

n = min(len(all_t), len(all_la))
t = np.array(all_t[:n])
la = np.array(all_la[:n])
mask = np.abs(la) <= 1.0
points = np.column_stack([t[mask], np.ones(mask.sum()), la[mask]])
slope, offset, friction = estimate_params(points)
print('')
print('ALL | total | ' + str(len(points)).rjust(4) + ' | ' + str(round(slope, 3)).rjust(14) + ' | ' +
      str(round(offset, 4)).rjust(8) + ' | ' + str(round(friction, 4)).rjust(8) + ' | OVERALL')
