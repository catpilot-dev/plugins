"""Replay the hold-bias gate + integral over a real rlog to sanity-check gating
and converged magnitude. NOT a closed-loop test (plant response isn't logged).

Usage (on C3, venv active, from /data/openpilot):
  PYTHONPATH=. python /data/plugins/plugins/bmw_e9x_e8x/tests/replay_hold_bias.py \
      /data/media/0/realdata/00000380--bc2a2ca510--6/rlog.zst
"""
import os
import sys
import math
import zstandard
from cereal import log as caplog

# import the pure functions from register (mock opendbc/cereal not needed here —
# we import only the plain-Python helpers, but importing register triggers
# _register_interfaces; run under the same interpreter that has opendbc, i.e. C3).
_PLUGIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _PLUGIN_DIR)
import register

# Constants mirror register.py HOLD_* defaults.
A_ON, A_FULL, K_ON, K_FULL = 20.0, 35.0, 0.006, 0.012
KI, B_MAX, LEAK, SAT = 0.8, 0.20, 0.10, 0.99
L = 2.76  # BMW E90 wheelbase (m)


def load(path):
  with open(path, 'rb') as f:
    raw = zstandard.ZstdDecompressor().decompress(f.read(), max_output_size=200 * 1024 * 1024)
  return list(caplog.Event.read_multiple_bytes(raw))


def main(path):
  events = load(path)
  # latest carState / controlsState / livePose sampled at livePose rate
  angle = kdes = kmeas = pressed = 0.0
  t0 = None
  rows = []
  for e in events:
    w = e.which()
    if w == 'carState':
      angle = e.carState.steeringAngleDeg
      pressed = 1.0 if e.carState.steeringPressed else 0.0
    elif w == 'controlsState':
      kdes = e.controlsState.desiredCurvature
      kmeas = e.controlsState.curvature
    elif w == 'livePose':
      t = e.logMonoTime / 1e9
      if t0 is None:
        t0 = t
      d_err = math.atan(kdes * L) - math.atan(kmeas * L)
      rows.append((t - t0, angle, kdes, kmeas, d_err, pressed))

  b = 0.0
  bmax_turn = 0.0
  bmax_straight = 0.0
  for (t, ang, kd, km, d_err, pr) in rows:
    g = register.hold_gate(ang, kd, A_ON, A_FULL, K_ON, K_FULL)
    overshoot = (kd - km) * km < 0.0
    learn_ok, release = register.hold_learn_flags(g, 'ramp' if g > 0 else 'hold_zero',
                                                  bool(pr), overshoot, False)
    b = register.hold_bias_step(b, g, d_err, learn_ok, release, KI, B_MAX, LEAK)
    if 10.0 <= t <= 35.0:
      bmax_turn = max(bmax_turn, abs(b))
    if t < 6.0 or t > 50.0:
      bmax_straight = max(bmax_straight, abs(b))

  print(f"samples={len(rows)}  hold_bias peak in turn (10-35s)={bmax_turn:.3f}  "
        f"peak on straight (<6s,>50s)={bmax_straight:.3f}")
  ok_turn = 0.05 <= bmax_turn <= 0.20
  ok_straight = bmax_straight < 0.03
  print(f"turn magnitude in [0.05,0.20]: {ok_turn}   straight < 0.03: {ok_straight}")
  if not (ok_turn and ok_straight):
    print("SANITY CHECK FAILED")
    return 1
  print("SANITY CHECK PASSED")
  return 0


if __name__ == '__main__':
  sys.exit(main(sys.argv[1]))
