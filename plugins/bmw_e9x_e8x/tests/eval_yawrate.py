import glob, os, zstandard, numpy as np
from cereal import log as caplog

ROUTE = '/data/media/0/realdata/0000021f--'
segments = sorted(glob.glob(ROUTE + '*'), key=lambda d: int(d.rsplit('--', 1)[-1]))

def read_rlog(path):
  dctx = zstandard.ZstdDecompressor()
  with open(path, 'rb') as f:
    return list(caplog.Event.read_multiple_bytes(dctx.decompress(f.read(), max_output_size=200*1024*1024)))

# Collect time-stamped yaw rates from both sources
can_yr = []  # (time, yaw_rate_rad)
pose_yr = []  # (time, yaw_rate_rad)

for seg_dir in segments[:15]:  # first 15 segments
  rlog_path = os.path.join(seg_dir, 'rlog.zst')
  if not os.path.exists(rlog_path): continue
  events = sorted(read_rlog(rlog_path), key=lambda e: e.logMonoTime)
  for evt in events:
    t = evt.logMonoTime * 1e-9
    w = evt.which()
    if w == 'carState':
      can_yr.append((t, evt.carState.yawRate))
    elif w == 'livePose':
      pose = evt.livePose
      if pose.angularVelocityDevice.valid:
        pose_yr.append((t, pose.angularVelocityDevice.z))

can_t = np.array([x[0] for x in can_yr])
can_v = np.array([x[1] for x in can_yr])
pose_t = np.array([x[0] for x in pose_yr])
pose_v = np.array([x[1] for x in pose_yr])

# Resample pose to CAN timestamps
pose_resampled = np.interp(can_t, pose_t, pose_v)

diff = can_v - pose_resampled

print('CAN YawRate vs livePose angularVelocity.yaw (z)')
print('  CAN samples:  ' + str(len(can_v)) + ' @ ~100Hz')
print('  Pose samples: ' + str(len(pose_v)) + ' @ ~20Hz')
print('')
print('  CAN:   mean=' + str(round(can_v.mean(), 5)) + ' std=' + str(round(can_v.std(), 5)))
print('  Pose:  mean=' + str(round(pose_resampled.mean(), 5)) + ' std=' + str(round(pose_resampled.std(), 5)))
print('  Diff:  mean=' + str(round(diff.mean(), 5)) + ' std=' + str(round(diff.std(), 5)))
print('  Corr:  ' + str(round(np.corrcoef(can_v, pose_resampled)[0, 1], 6)))

# Cross-correlation to find time offset
# Downsample CAN to ~20Hz for efficiency
ds = 5
c = can_v[::ds]
p = pose_resampled[::ds]
dt_ds = (can_t[-1] - can_t[0]) / len(c)

# Normalized cross-correlation
c_norm = (c - c.mean()) / (c.std() + 1e-10)
p_norm = (p - p.mean()) / (p.std() + 1e-10)

max_lag = int(1.0 / dt_ds)  # search up to 1 second
best_lag = 0
best_corr = -1
for lag in range(-max_lag, max_lag):
  if lag >= 0:
    cc = np.mean(c_norm[lag:] * p_norm[:len(c_norm)-lag]) if lag < len(c_norm) else 0
  else:
    cc = np.mean(c_norm[:len(c_norm)+lag] * p_norm[-lag:]) if -lag < len(p_norm) else 0
  if cc > best_corr:
    best_corr = cc
    best_lag = lag

lag_seconds = best_lag * dt_ds
print('')
print('Cross-correlation peak:')
print('  Lag: ' + str(round(lag_seconds, 4)) + 's (CAN leads pose by this amount)')
print('  Corr at peak: ' + str(round(best_corr, 6)))

# Show correlation at different offsets
print('')
print('Correlation at different time shifts:')
for shift_ms in [-200, -150, -100, -50, 0, 50, 100, 150, 200]:
  shift_samples = int(shift_ms / 1000.0 / dt_ds)
  if abs(shift_samples) >= len(c_norm):
    continue
  if shift_samples >= 0:
    cc = np.mean(c_norm[shift_samples:] * p_norm[:len(c_norm)-shift_samples])
  else:
    cc = np.mean(c_norm[:len(c_norm)+shift_samples] * p_norm[-shift_samples:])
  print('  shift=' + str(shift_ms).rjust(4) + 'ms | corr=' + str(round(cc, 6)))
