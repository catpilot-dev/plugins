#!/usr/bin/env python3
"""Route-wide sanity check for the BMW lateral controller's push budget.

Independently reproduces, OFFLINE, the push-budget rule shipped in
bmw/latcontroller.py's update() (BUDGET_DEG block) and reports how often it
spends, how fast, at what torque, and how much extra torque the historical
(unbudgeted) ramp piled on afterward. This is a SANITY CHECK, not a gate: it
exists to catch the rule cutting normal steering short (median torque at
spend near zero, or almost every push spending instantly), not to tune
BUDGET_DEG.

Deliberately NOT imported from the plugin tree: the deployed runtime under
/data/plugins-runtime predates this feature (its bmw_lat_control payloads
carry no push_moved/budget_spent keys at all -- confirmed empirically before
writing this script), and the point of this script is an INDEPENDENT replica
to sanity-check the shipped rule, not a call-through to it. The rule itself
is five lines, reproduced verbatim in spirit below:

    push_ref = steeringAngleDeg on the first tick of a run where action ==
    'ramp'; push_moved = steeringAngleDeg - push_ref each tick thereafter;
    spent = abs(push_moved) >= BUDGET_DEG and push_moved * -torque > 0.0.

Sign convention (see bmw/latcontroller.py): steeringAngleDeg positive =
LEFT, torque (the controller's `output`, a fraction of STEER_MAX) negative =
LEFT -- opposite conventions, hence the -torque in the direction check.

Data sources per rlog: carState (steeringAngleDeg, steeringPressed, vEgo)
and the pluginBusLog topic "bmw_lat_control" (JSON payload keys `output`,
`action`), published once per livePose tick (20 Hz) and batched into
pluginBusLog cereal messages at 5 Hz by bus_logger.py -- each Entry keeps its
own individual monoTime, which is what this script times pushes on.

Timeline choice: the bus stream (20 Hz, one row per bmw_lat_control entry)
drives the tick sequence; carState is the fast signal (~100 Hz) and is
backward-filled onto each bus tick (most recent carState at or before the
bus tick's time) for steeringAngleDeg/steeringPressed/vEgo. This matches the
grain of the coarser signal (torque/action only change at 20 Hz) and was
verified against the hand-analysis checkpoint (route 3f2 seg 10: spend
t~=663.87 torque~=2.8 Nm, peak 3.75 Nm at t~=664.30) before being adopted --
see task-2-report.md for the comparison against a carState-driven join,
which was consistently farther off on both axes.

peak-after-spend note: once a push has spent, torque is tracked past the
literal end of the 'ramp' action label (through subsequent bus ticks,
whatever their action) until it strictly decreases for the first time. This
matters: in the route 3f2 seg 10 example, the true peak (3.754 Nm) is
recorded on the FIRST tick already labeled 'cancel_tol' -- the ramp's last
committed step is still being applied when the next decision's label
changes, so gating the peak search on action == 'ramp' would clip it one
tick early (3.727 Nm) and understate the headroom this change recovers.

Run ON THE C3 (it reads rlogs directly), read-only, one segment at a time
-- the C3 CPU is weak and 51 segments takes several minutes:

    ssh c3
    source /usr/local/venv/bin/activate
    cd /data/openpilot
    PYTHONPATH=/data/openpilot python /tmp/replay_push_budget.py \\
      /data/media/0/realdata/000003f2--a4bbab4676-- 0 50

(Stage this file under /tmp on the device -- the runtime deploy predates the
feature it's checking, so there's nothing under /data/plugins-runtime worth
importing from; this script is self-contained.)
"""
import json
import sys

import numpy as np
import zstandard
from cereal import log

# Must match bmw/latcontroller.py's BUDGET_DEG and bmw/values.py's
# CarControllerParams.STEER_MAX. NOT tuned here -- this script sanity-checks
# the shipped value, it does not choose it.
BUDGET_DEG = 2.0    # deg of steeringAngleDeg one push may spend before easing off
STEER_MAX = 12.0    # Nm, full torque authority (output fraction 1.0)


def read_events(path):
  with open(path, 'rb') as f:
    raw = zstandard.ZstdDecompressor().decompress(f.read(), max_output_size=600 * 1024 * 1024)
  return log.Event.read_multiple_bytes(raw)


def build_ticks(seg_path):
  """Bus-tick-driven (20 Hz) timeline for one segment.

  Returns a list of (t, steeringAngleDeg, output, action) tuples, one per
  bmw_lat_control bus entry, in time order, with carState backward-filled
  onto each bus tick's time and ticks already dropped where
  steeringPressed or vEgo < 5.0 ("skip ticks", applied here so the push
  state machine below never sees them at all).
  """
  cs, lat = [], []
  for e in read_events(seg_path):
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
  cs.sort(key=lambda r: r[0])
  lat.sort(key=lambda r: r[0])

  ticks = []
  i = 0
  for t, out, action in lat:
    while i + 1 < len(cs) and cs[i + 1][0] <= t:
      i += 1
    ang, pressed, v = cs[i][1], cs[i][2], cs[i][3]
    if pressed or v < 5.0:
      continue
    ticks.append((t, ang, out, action))
  return ticks


def find_pushes(ticks):
  """The rule, verbatim: a push is a run of ticks with action == 'ramp'.

  Latches on first crossing -- once spent, later same-push ticks don't
  re-evaluate the spend condition (only the first crossing is "when it
  spent").
  """
  pushes = []
  push = None
  for idx, (t, ang, out, action) in enumerate(ticks):
    if action == 'ramp':
      if push is None:
        push = {'ref': ang, 'start_t': t, 'start_idx': idx, 'spent': False}
      push_moved = ang - push['ref']
      # Angle positive=LEFT, torque(out) negative=LEFT -- opposite
      # conventions, so "moved the way we pushed" is push_moved * -out > 0.
      if not push['spent'] and abs(push_moved) >= BUDGET_DEG and push_moved * -out > 0.0:
        push['spent'] = True
        push['spent_t'] = t
        push['spent_idx'] = idx
        push['spent_torque_nm'] = abs(out * STEER_MAX)
    else:
      if push is not None:
        pushes.append(push)
        push = None
  if push is not None:
    pushes.append(push)
  return pushes


def measure_peak_after(ticks, pushes):
  """For each spent push, how far past the spend torque the ramp still ran.

  Scans forward from the spend tick (exclusive) -- across the action-label
  boundary, see module docstring -- tracking the running max |torque|,
  stopping at the first strict decrease, the next push's start, or the end
  of the segment's ticks, whichever comes first. Sets peak_after_nm /
  peak_after_t on each spent push dict; headroom = peak_after_nm -
  spent_torque_nm is "how much more torque the ramp added after spend."
  """
  for pi, p in enumerate(pushes):
    if not p['spent']:
      continue
    next_start = pushes[pi + 1]['start_idx'] if pi + 1 < len(pushes) else len(ticks)
    peak = p['spent_torque_nm']
    peak_t = p['spent_t']
    for idx in range(p['spent_idx'] + 1, next_start):
      torque_nm = abs(ticks[idx][2] * STEER_MAX)
      if torque_nm > peak:
        peak = torque_nm
        peak_t = ticks[idx][0]
      else:
        break
    p['peak_after_nm'] = peak
    p['peak_after_t'] = peak_t


def replay_segment(seg_path):
  ticks = build_ticks(seg_path)
  pushes = find_pushes(ticks)
  measure_peak_after(ticks, pushes)
  return pushes


def main():
  prefix, lo, hi = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])

  all_pushes = []
  seg10_pushes = []
  for n in range(lo, hi + 1):
    seg_path = f"{prefix}{n}/rlog.zst"
    try:
      pushes = replay_segment(seg_path)
    except FileNotFoundError:
      print(f"seg {n:3d}: rlog not found, skipping", flush=True)
      continue
    except Exception as exc:
      print(f"seg {n:3d}: FAILED ({exc!r}), skipping", flush=True)
      continue

    spent = [p for p in pushes if p['spent']]
    print(f"seg {n:3d}: pushes={len(pushes):4d} spent={len(spent):4d}", flush=True)
    for p in spent:
      headroom = p['peak_after_nm'] - p['spent_torque_nm']
      print(f"    push@{p['start_t']:9.2f}s spent@{p['spent_t']:9.2f}s "
            f"(+{p['spent_t'] - p['start_t']:5.2f}s) torque={p['spent_torque_nm']:5.2f}Nm "
            f"peak_after={p['peak_after_nm']:5.2f}Nm@{p['peak_after_t']:9.2f}s "
            f"headroom=+{headroom:5.2f}Nm", flush=True)

    all_pushes.extend(pushes)
    if n == 10:
      seg10_pushes = spent

  n_pushes = len(all_pushes)
  spent_pushes = [p for p in all_pushes if p['spent']]
  n_spent = len(spent_pushes)

  print("\n=== AGGREGATE ===")
  print(f"segments processed         : {lo}-{hi}")
  print(f"total pushes                : {n_pushes}")
  print(f"pushes that spent budget    : {n_spent} ({100.0 * n_spent / max(n_pushes, 1):.1f}%)")

  if spent_pushes:
    times = np.array([p['spent_t'] - p['start_t'] for p in spent_pushes])
    torques = np.array([p['spent_torque_nm'] for p in spent_pushes])
    headrooms = np.array([p['peak_after_nm'] - p['spent_torque_nm'] for p in spent_pushes])
    print(f"time to spend    median/p90 : {np.median(times):.3f}s / {np.percentile(times, 90):.3f}s")
    print(f"torque at spend  median/p90 : {np.median(torques):.2f}Nm / {np.percentile(torques, 90):.2f}Nm")
    print(f"headroom after   median/p90 : {np.median(headrooms):.2f}Nm / {np.percentile(headrooms, 90):.2f}Nm")
    print(f"headroom after   mean       : {np.mean(headrooms):.2f}Nm")
  else:
    print("no pushes spent the budget")

  print("\n=== SEGMENT 10 CHECK (hand-analysis: spend t~=663.87 torque~=2.8Nm, "
        "peak 3.75Nm @ t~=664.30) ===")
  if seg10_pushes:
    for p in seg10_pushes:
      headroom = p['peak_after_nm'] - p['spent_torque_nm']
      print(f"  push@{p['start_t']:.2f}s spent@{p['spent_t']:.2f}s torque={p['spent_torque_nm']:.3f}Nm "
            f"peak_after={p['peak_after_nm']:.3f}Nm@{p['peak_after_t']:.2f}s headroom=+{headroom:.3f}Nm")
  else:
    print("  no spending pushes found in segment 10 (or segment 10 not in range)")


if __name__ == '__main__':
  main()
