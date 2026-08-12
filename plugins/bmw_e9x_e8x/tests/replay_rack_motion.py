"""Replay the rack-motion detector against recorded routes.

Run ON THE C3 (it reads rlogs directly):
    ssh c3
    source /usr/local/venv/bin/activate
    cd /data/openpilot
    PYTHONPATH=/data/openpilot:/data/plugins-runtime/bmw_e9x_e8x \\
      python /tmp/replay_rack_motion.py /data/media/0/realdata/000003f2--a4bbab4676-- 0 50

Reports, per segment and in aggregate:
  - how early the detector would have flagged the stall before each release
  - false-positive rate on ordinary driving (flagged while the wheel was fine)
  - the breakaway estimate's trajectory and final value
"""
import json
import sys

import zstandard
from cereal import log

from bmw.rack_motion import RackMotion, BreakawayEstimator, MOTION_THRESHOLD_DEG_S


def read_segment(path):
    """Yield (t, steering_angle_deg, torque_frac, action) at carState rate."""
    with open(path, 'rb') as f:
        raw = zstandard.ZstdDecompressor().decompress(f.read(), max_output_size=600 * 1024 * 1024)
    cs, lat = [], []
    for e in log.Event.read_multiple_bytes(raw):
        w = e.which()
        if w == 'carState':
            m = e.carState
            cs.append((e.logMonoTime / 1e9, m.steeringAngleDeg, m.steeringPressed, m.vEgo))
        elif w == 'pluginBusLog':
            for ent in e.pluginBusLog.entries:
                if ent.topic != 'bmw_lat_control':
                    continue
                try:
                    d = json.loads(ent.json)
                except Exception:
                    continue
                lat.append((ent.monoTime / 1e9, d.get('output', 0.0), d.get('action', '')))
    cs.sort()
    lat.sort()
    j = 0
    for t, ang, pressed, v in cs:
        while j + 1 < len(lat) and lat[j + 1][0] <= t:
            j += 1
        out, action = (lat[j][1], lat[j][2]) if lat else (0.0, '')
        yield t, ang, out, action, pressed, v


def replay(prefix, lo, hi):
    est = BreakawayEstimator()
    stalls = releases = early_total = 0
    flagged_ticks = push_ticks = 0
    for n in range(lo, hi + 1):
        rm = RackMotion()
        stalled_since = None
        for t, ang, out, action, pressed, v in read_segment(f"{prefix}{n}/rlog.zst"):
            rm.update(t, ang)
            if pressed or v < 5.0 or action != 'ramp':
                stalled_since = None
                continue
            push_ticks += 1
            moving = rm.is_moving_with_torque(out)
            est.update(out, moving)
            stalled = (abs(out) > est.breakaway_frac * 0.5) and not moving
            if stalled:
                flagged_ticks += 1
                if stalled_since is None:
                    stalled_since = t
                    stalls += 1
            else:
                if stalled_since is not None and abs(rm.rate_deg_s) > 10.0:
                    releases += 1
                    early_total += (t - stalled_since)
                stalled_since = None
        print(f"seg {n:2d}: stalls={stalls} releases={releases} breakaway={est.breakaway_frac:.3f}")
    print("\n=== AGGREGATE ===")
    print(f"push ticks              : {push_ticks}")
    print(f"stall episodes flagged  : {stalls}")
    print(f"releases after a stall  : {releases}")
    if releases:
        print(f"mean warning before release: {early_total / releases:.2f} s")
    print(f"flagged fraction of pushes : {flagged_ticks / max(push_ticks, 1):.1%}")
    print(f"final breakaway estimate   : {est.breakaway_frac:.3f} frac"
          f" ({est.breakaway_frac * 12:.2f} Nm) from {est.observations} observations")
    print(f"motion threshold used      : {MOTION_THRESHOLD_DEG_S} deg/s")


if __name__ == '__main__':
    replay(sys.argv[1], int(sys.argv[2]), int(sys.argv[3]))
