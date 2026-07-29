#!/usr/bin/env python3
"""DC_TAU sweep for the AC stabilizer (run on C3, non-activating).

Per route x tau: (1) sustained-correction residue — mean |bias| over 30 s
windows on out-of-band stretches (the anti-arm-wrestle gate; the trim sat
pinned at cap here); (2) wander capture — how much of the gap's 0.03-0.16 Hz
band variance appears in excess_ac; (3) correction occupancy on quiet
in-band stretches (must not exceed a few percent).

Usage: PYTHONPATH=/data/openpilot:/tmp/lkp3 python replay_ac.py ROUTE [...]
"""
import sys, glob
import numpy as np
import zstandard
from cereal import log as capnp_log
import anchor as anchor_mod
anchor_mod.DT_CTRL = 0.05                 # replay steps at modelV2 ~20 Hz — exact fix
from anchor import AnchorConfig, LaneAnchor

BASE = '/data/media/0/realdata'
TICK = 0.05

def frames(route):
  for seg in sorted(int(p.rsplit('--', 1)[1]) for p in glob.glob(f'{BASE}/{route}--*')):
    try:
      raw = zstandard.ZstdDecompressor().decompress(
        open(f'{BASE}/{route}--{seg}/rlog.zst', 'rb').read(), max_output_size=2**31)
    except (FileNotFoundError, zstandard.ZstdError):
      continue
    v = 0.0
    for evt in capnp_log.Event.read_multiple_bytes(raw):
      w = evt.which()
      if w == 'carState':
        v = float(evt.carState.vEgo)
      elif w == 'modelV2':
        m = evt.modelV2
        yield m, v, str(m.meta.laneChangeState) != 'off'

def band_var(x, lo=0.03, hi=0.16):
  x = np.asarray(x) - np.mean(x)
  f = np.fft.rfftfreq(len(x), TICK)
  X = np.abs(np.fft.rfft(x)) ** 2
  sel = (f >= lo) & (f <= hi)
  return X[sel].sum() / len(x)

for route in sys.argv[1:]:
  print(f'\n===== {route} =====')
  for tau in (10.0, 20.0, 30.0):
    a = LaneAnchor(AnchorConfig(dc_tau=tau))
    rows = []
    for m, v, lc in frames(route):
      out, t = a.update(float(m.action.desiredCurvature), m, v, lc, lat_delay=0.6)
      rows.append((t['gap_filt'], t['excess_ac'], t['kappa_bias'], t['state'] == 'anchor'))
    gf = np.array([r[0] for r in rows]); ac = np.array([r[1] for r in rows])
    kb = np.array([r[2] for r in rows]); anc = np.array([r[3] for r in rows])
    if not anc.any():
      print(f'  tau={tau}: no anchor ticks'); continue
    # 1. sustained residue on out-of-band stretches (old-band definition)
    oob = anc & ((gf < 0.6) | (gf > 1.0))
    W = int(30.0 / TICK)
    res = [np.abs(np.mean(kb[i:i+W])) for i in range(0, len(kb) - W, W)
           if oob[i:i+W].mean() > 0.8]
    # 2. wander capture on long anchored runs
    runs, s = [], None
    for i, f_ in enumerate(anc):
      if f_ and s is None: s = i
      elif not f_ and s is not None:
        if i - s > 1200: runs.append((s, i))
        s = None
    caps = [band_var(ac[s:e]) / max(band_var(gf[s:e]), 1e-12) for s, e in runs]
    # 3. quiet occupancy
    inb = anc & (gf >= 0.6) & (gf <= 1.0)
    occ = np.mean(np.abs(kb[inb]) > 1e-5) if inb.any() else 0.0
    print(f'  tau={tau}: residue(30s windows) p50={np.median(res) if res else 0:.6f} '
          f'max={max(res) if res else 0:.6f} (gate <2e-4)  '
          f'wander-capture p50={np.median(caps) if caps else 0:.2f}  '
          f'in-band correction occupancy={occ*100:.1f}%')
