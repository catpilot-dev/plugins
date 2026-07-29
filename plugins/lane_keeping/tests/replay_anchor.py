#!/usr/bin/env python3
"""Offline replay of the lane-keeping anchor over recorded rlogs (run on C3).

Feeds recorded modelV2 lane lines through LaneAnchor and reports ANCHOR/MODEL
occupancy, in-band vs biasing fraction, and a T_PREVIEW sweep of the resulting
curvature-bias magnitude. Open-loop detector validation + tuning aid (the
closed-loop centering cannot be measured offline).

Usage (on C3):
  source /usr/local/venv/bin/activate
  PYTHONPATH=/data/openpilot:/data/plugins-runtime/lane_keeping \
    python replay_anchor.py 000003b7--977baff7b6 000003bb--12fe0dc6be
"""
import sys, glob
import zstandard
from cereal import log as capnp_log
from anchor import AnchorConfig, LaneAnchor

BASE = '/data/media/0/realdata'


def load_model(route, seg):
  raw = zstandard.ZstdDecompressor().decompress(
    open(f'{BASE}/{route}--{seg}/rlog.zst', 'rb').read(), max_output_size=2**31)
  frames = []
  v = 0.0
  for evt in capnp_log.Event.read_multiple_bytes(raw):
    w = evt.which()
    if w == 'carState':
      v = float(evt.carState.vEgo)
    elif w == 'modelV2':
      m = evt.modelV2
      lane_changing = str(m.meta.laneChangeState) != 'off'
      frames.append((m, v, lane_changing))
  return frames


def run_route(route, t_preview):
  segs = sorted(int(p.rsplit('--', 1)[1]) for p in glob.glob(f'{BASE}/{route}--*'))
  a = LaneAnchor(AnchorConfig(t_preview=t_preview))
  n = anchor_n = biasing = 0
  max_bias = 0.0
  for seg in segs:
    try:
      frames = load_model(route, seg)
    except (FileNotFoundError, zstandard.ZstdError):
      continue
    for m, v, lc in frames:
      _c, telem = a.update(0.0, m, v, lc)
      n += 1
      if telem['state'] == 'anchor':
        anchor_n += 1
        if abs(telem['excess']) > 1e-6:
          biasing += 1
      max_bias = max(max_bias, abs(telem['kappa_bias']))
  if n == 0:
    return
  print(f'  {route}: n={n} ANCHOR={anchor_n/n*100:.0f}% biasing={biasing/max(anchor_n,1)*100:.0f}% '
        f'max|kappa_bias|={max_bias:.5f}')


if __name__ == '__main__':
  routes = sys.argv[1:] or ['000003b7--977baff7b6', '000003bb--12fe0dc6be']
  for tp in (1.0, 1.5, 2.0):
    print(f'T_PREVIEW={tp}s:')
    for r in routes:
      run_route(r, tp)
