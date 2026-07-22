#!/usr/bin/env python3
"""Replay gate for the predictive deadband (run on C3, non-activating).

Feeds recorded modelV2 lane lines + vEgo through LaneAnchor and reports:
  1. prediction accuracy: gap_pred(t) vs realized gap_filt(t + pred_t),
     compared against the trivial predictor gap_filt(t);
  2. a PRED_DELAY_MULT sweep {1.5, 2.0, 3.0};
  3. decision quality vs the current-gap deadband (computed from the same
     telemetry): nudge-onset lead at band exits, ease-off during recoveries,
     added-nudge fraction (prediction nudges, current holds, and no excursion
     follows within pred_t — false alarms).

Usage (on C3):
  source /usr/local/venv/bin/activate
  PYTHONPATH=/data/openpilot:/tmp/lkp python replay_pred.py 000003bf--47fd55882c [...]
"""
import sys, glob
import numpy as np
import zstandard
from cereal import log as capnp_log
from anchor import AnchorConfig, LaneAnchor
import anchor as anchor_mod

BASE = '/data/media/0/realdata'
GAP_MIN, GAP_MAX = 0.6, 1.0
LAT_DELAY = 0.6          # replay approximation of the live liveDelay value
TICK = 0.05              # modelV2 ~20 Hz in the rlog
# The live hook runs at 100 Hz (DT_CTRL=0.01) but this replay steps once per
# modelV2 frame (~20 Hz). Patch the module dt so every EMA (gap, gap_pred,
# kappa filters) and the bias slew limit see the true per-call cadence —
# exact fix via the exponential-filter identity, not an approximation.
anchor_mod.DT_CTRL = TICK


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


def deadband(g):
  return g - min(max(g, GAP_MIN), GAP_MAX)


def run(route, mult):
  a = LaneAnchor(AnchorConfig(pred_delay_mult=mult))
  rows = []  # (gap_filt, gap_pred, v, anchored)
  for m, v, lc in frames(route):
    # feed the RECORDED desired curvature so the path-compensation term is
    # active in curves, as it will be live
    kin = float(m.action.desiredCurvature)
    _c, t = a.update(kin, m, v, lc, lat_delay=LAT_DELAY)
    rows.append((t['gap_filt'], t['gap_pred'], v, t['state'] == 'anchor'))
  return rows


for route in (sys.argv[1:] or ['000003bf--47fd55882c']):
  print(f'\n===== {route} =====')
  for mult in (1.5, 2.0, 3.0):
    rows = run(route, mult)
    gf = np.array([r[0] for r in rows]); gp = np.array([r[1] for r in rows])
    v = np.array([r[2] for r in rows]); anc = np.array([r[3] for r in rows])
    # 1. prediction accuracy: realized gap pred_t later (pred_t varies with v
    #    only through the x_pred clip; use time shift pred_t = mult*LAT_DELAY)
    pred_t = mult * LAT_DELAY
    shift = max(1, int(round(pred_t / TICK)))
    if len(rows) <= shift:
      print(f'  mult={mult}: no data'); continue
    unclipped = (v[:-shift] * pred_t > 5.0 + 1e-6) & (v[:-shift] * pred_t < 50.0 - 1e-6)
    ok = anc[:-shift] & anc[shift:] & unclipped
    if not ok.any():
      print(f'  mult={mult}: no unclipped anchor ticks'); continue
    err_pred = gp[:-shift][ok] - gf[shift:][ok]
    err_triv = gf[:-shift][ok] - gf[shift:][ok]
    print(f'  mult={mult}: n={ok.sum()}  pred RMSE={np.sqrt(np.mean(err_pred**2)):.3f} m'
          f'  trivial RMSE={np.sqrt(np.mean(err_triv**2)):.3f} m'
          f'  (improvement {100*(1 - np.sqrt(np.mean(err_pred**2))/max(np.sqrt(np.mean(err_triv**2)),1e-9)):.0f}%)'
          f'  clip-excluded={100 * (1 - ok.sum() / max((anc[:-shift] & anc[shift:]).sum(), 1)):.0f}%')
    if mult != 2.0:
      continue
    # 2. decision quality at mult=2.0
    cur_nudge = np.array([abs(deadband(g)) > 1e-9 for g in gf])
    prd_nudge = np.array([abs(deadband(g)) > 1e-9 for g in gp])
    # onset lead: for each current-gap band exit, how much earlier did the
    # predictor start nudging?
    leads = []
    prev_exit = 0
    exits = np.where((~cur_nudge[:-1]) & cur_nudge[1:] & anc[1:] & anc[:-1])[0] + 1
    for i in exits:
      j = i
      while j > prev_exit and prd_nudge[j - 1] and anc[j - 1]:
        j -= 1
      leads.append((i - j) * TICK)
      prev_exit = i
    # ease-off: ticks where current is out-of-band but predictor already holds
    rec = cur_nudge & (~prd_nudge) & anc
    # false alarms: predictor nudges, current holds, and no current-gap exit
    # follows within pred_t
    po = prd_nudge & (~cur_nudge) & anc
    e2 = np.diff(np.concatenate([[0], po.astype(int), [0]]))
    starts = np.where(e2 == 1)[0]; ends = np.where(e2 == -1)[0]
    fa = 0
    for s_, e_ in zip(starts, ends):
      # a run is a false alarm only if NO current-gap nudge occurs from its
      # start through shift ticks past its end (tails of genuine early onsets
      # and lingering post-exit ticks are not separate alarms)
      if not cur_nudge[s_:min(len(cur_nudge), e_ + shift)].any():
        fa += 1
    tot_pn = len(starts)
    print(f'    exits={len(exits)}  onset lead p50={np.median(leads) if leads else 0:.2f}s'
          f' p90={np.percentile(leads, 90) if leads else 0:.2f}s')
    print(f'    ease-off ticks (current out, predictor holds): {rec.sum()}'
          f' ({100 * rec.sum() / max(cur_nudge.sum(), 1):.0f}% of out-of-band time)')
    print(f'    predictor-only nudge RUNS: {tot_pn}  false-alarm (no exit follows): '
          f'{fa} ({100 * fa / max(tot_pn, 1):.0f}%)')
