import glob, os, zstandard, numpy as np
from cereal import log as caplog

ROUTE = '/data/media/0/realdata/0000021f--'
segments = sorted(glob.glob(ROUTE + '*'), key=lambda d: int(d.rsplit('--', 1)[-1]))

def read_rlog(path):
  dctx = zstandard.ZstdDecompressor()
  with open(path, 'rb') as f:
    return list(caplog.Event.read_multiple_bytes(dctx.decompress(f.read(), max_output_size=200*1024*1024)))

# Collect all torque vs lat_accel pairs across route
all_t = []
all_la = []
all_v = []
lat_active = False
v_ego = 0

for seg_dir in segments:
  rlog_path = os.path.join(seg_dir, 'rlog.zst')
  if not os.path.exists(rlog_path): continue
  events = sorted(read_rlog(rlog_path), key=lambda e: e.logMonoTime)
  for evt in events:
    w = evt.which()
    if w == 'carControl': lat_active = evt.carControl.latActive
    elif w == 'carState': v_ego = evt.carState.vEgo
    elif w == 'carOutput' and lat_active and v_ego > 15/3.6:
      all_t.append(-evt.carOutput.actuatorsOutput.torque)
    elif w == 'livePose' and lat_active and v_ego > 15/3.6:
      pose = evt.livePose
      if pose.angularVelocityDevice.valid:
        all_la.append(pose.angularVelocityDevice.z * v_ego)
        all_v.append(v_ego)

n = min(len(all_t), len(all_la))
t = np.array(all_t[:n])
la = np.array(all_la[:n])
v = np.array(all_v[:n])

# Per-bucket least squares: la = slope * torque + offset
bounds = [(-0.5,-0.3),(-0.3,-0.2),(-0.2,-0.1),(-0.1,-0.05),(-0.05,0),(0,0.05),(0.05,0.1),(0.1,0.2),(0.2,0.3),(0.3,0.5)]

print('Piecewise least squares fit: la = F * torque + offset')
print('')
print('  Torque range    |   N  |    F (slope) |   offset  |  R^2   | avg_speed')
print('  --------------- | ---- | ------------ | --------- | ------ | ---------')

slopes = []
centers = []

for lo, hi in bounds:
  mask = (t >= lo) & (t < hi) & (np.abs(la) <= 2.0)
  count = mask.sum()
  if count < 30:
    print('  [' + str(lo).rjust(5) + ',' + str(hi).rjust(5) + ') | ' + str(count).rjust(4) + ' |         —    |     —     |   —    |')
    continue

  t_b = t[mask]
  la_b = la[mask]
  v_b = v[mask]

  # Least squares: la = F * torque + offset
  A = np.column_stack([t_b, np.ones(len(t_b))])
  result = np.linalg.lstsq(A, la_b, rcond=None)
  slope, offset = result[0]

  # R^2
  la_pred = slope * t_b + offset
  ss_res = np.sum((la_b - la_pred)**2)
  ss_tot = np.sum((la_b - la_b.mean())**2)
  r2 = 1 - ss_res / ss_tot if ss_tot > 0 else 0

  avg_speed = v_b.mean() * 3.6
  center = (lo + hi) / 2

  slopes.append(slope)
  centers.append(center)

  print('  [' + str(lo).rjust(5) + ',' + str(hi).rjust(5) + ') | ' + str(count).rjust(4) + ' | ' +
        str(round(slope, 3)).rjust(12) + ' | ' + str(round(offset, 4)).rjust(9) + ' | ' +
        str(round(r2, 4)).rjust(6) + ' | ' + str(round(avg_speed, 0)).rjust(5))

# Also by speed range
print('')
print('Per-speed-range fit (all torques combined):')
print('  Speed range  |   N  |    F (slope) |  offset   |  R^2')
for slo, shi in [(30,50),(50,70),(70,90),(90,120)]:
  mask = (v*3.6 >= slo) & (v*3.6 < shi) & (np.abs(t) > 0.02) & (np.abs(la) <= 2.0)
  count = mask.sum()
  if count < 50: continue
  A = np.column_stack([t[mask], np.ones(count)])
  result = np.linalg.lstsq(A, la[mask], rcond=None)
  slope, offset = result[0]
  la_pred = slope * t[mask] + offset
  ss_res = np.sum((la[mask] - la_pred)**2)
  ss_tot = np.sum((la[mask] - la[mask].mean())**2)
  r2 = 1 - ss_res / ss_tot if ss_tot > 0 else 0
  print('  ' + str(slo).rjust(3) + '-' + str(shi).rjust(3) + ' kph | ' + str(count).rjust(4) + ' | ' +
        str(round(slope, 3)).rjust(12) + ' | ' + str(round(offset, 4)).rjust(9) + ' | ' + str(round(r2, 4)).rjust(6))

# Summary
print('')
print('Summary: F varies from ' + str(round(min(slopes), 2)) + ' to ' + str(round(max(slopes), 2)))
print('Torqued single-slope estimate would average these, masking the nonlinearity.')
