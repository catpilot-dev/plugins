"""Evaluate one route's DCC setpoint control against a baseline set of routes.

Replaces the pile of throwaway per-route scripts: every number here comes from
the extracted .npz only, so `eval_route.py --route X` is reproducible.

Two measurement traps this tool exists to avoid, both learned the hard way:
  * acceptance rates are meaningless on bursts that had another command 0.05 s
    away — one route showed a "45 % rejection rate" for minus5 that was purely
    contamination, so acceptance is reported on an ISOLATED subset and refuses
    to print a percentage below ISOLATED_MIN_N bursts.
  * np.interp clamps instead of extrapolating, so reading the setpoint past the
    end of a segment silently returns the last sample — which is a disabled/
    sentinel value. That produced a 252 km/h setpoint reading. The settle time
    is clamped into the segment and the burst dropped if too little is left.
"""
import argparse
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

from common import CMD_STEP, EXTRACTED_DIR
# Segmentation is shared with bursts.py on purpose: importing is side-effect
# free there (its CLI is behind __main__) and a second copy of the constants
# would drift.
from bursts import find_bursts, GAP_S, HOLD_MAX_INTERVAL

CMDS4 = ("plus1", "plus5", "minus1", "minus5")
CADENCES = ("hold", "single")

V_MIN = 3.0             # m/s: below this the DCC is not meaningfully in control
MIN_CLEAN = 200         # engaged-clean carState samples for a segment to count
MIN_TOTAL = 500         # total carState samples for a segment to count
LOW_SPEED = 12.0        # m/s: the low-speed braking subset
BRAKE_ATARGET = -0.3    # m/s^2: planner is asking for real deceleration
LOW_SPEED_MIN_N = 50    # fewer samples than this -> "insufficient"
CLOSE_S = 2.0           # two commands this close together count as churn
ISOLATION_S = 2.0       # clear of every other burst by this much on BOTH sides
ISOLATED_MIN_N = 10     # fewer isolated bursts than this -> UNMEASURABLE
SHORT_BURST_S = 0.10    # bursts at or under this are single-frame-ish taps
SETTLE_S = 0.5          # setpoint read this long after the burst ends
SETTLE_MIN_S = 0.3      # ...but never on less than this much real segment data
PRE_S = 0.1             # setpoint baseline read this long before the burst
SP_EPS = 0.05           # m/s: smaller setpoint steps are float32 noise
V_DEADZONE = 0.5 / 3.6  # m/s: speed error inside this counts as "at target"
A_DEADZONE = 0.05       # m/s^2: accel request inside this counts as "coasting"
BURST_MATCH_S = 0.2     # burst start must be this close to a clean sample


# ---------------------------------------------------------------- pure helpers

def clean_mask(seg):
  """Engaged, no pedals, moving: the only samples any metric here trusts."""
  return (seg["cruiseEnabled"] > 0) & (seg["gas"] == 0) & \
         (seg["brake"] == 0) & (seg["vEgo"] > V_MIN)


def segment_usable(seg):
  n_total = len(seg["cs_t"])
  if n_total < MIN_TOTAL or len(seg["ctrl_t"]) < 2:
    return False
  return int(np.count_nonzero(clean_mask(seg))) >= MIN_CLEAN


def sample_dt(t):
  """Median sample spacing; the log rate is nominal but never exactly uniform."""
  return float(np.median(np.diff(t))) if len(t) > 1 else 0.0


def on_cs_grid(seg, key):
  """A carControl channel resampled onto the carState grid."""
  return np.interp(seg["cs_t"], seg["ctrl_t"], seg[key])


def isolated_flags(bursts, gap=ISOLATION_S):
  """True where no other burst comes within `gap` seconds on either side.

  A missing neighbour (first/last burst of the segment) counts as clear. Bursts
  need not arrive sorted; the returned flags follow the input order.
  """
  order = sorted(range(len(bursts)), key=lambda i: bursts[i].t_start)
  flags = [False] * len(bursts)
  for k, i in enumerate(order):
    b = bursts[i]
    before = k == 0 or b.t_start - bursts[order[k - 1]].t_end > gap
    after = k == len(order) - 1 or bursts[order[k + 1]].t_start - b.t_end > gap
    flags[i] = before and after
  return flags


def burst_ticks(burst, cs_t, setpoint):
  """Accepted ticks in the commanded direction, or None if unreadable.

  None means the segment does not hold enough data around the burst to read a
  settled setpoint: np.interp would clamp to an edge sample and invent a number.
  """
  if len(cs_t) < 2:
    return None
  t_settle = min(burst.t_end + SETTLE_S, float(cs_t[-1]))
  if t_settle - burst.t_end < SETTLE_MIN_S:
    return None
  t_base = burst.t_start - PRE_S
  if t_base < float(cs_t[0]):
    return None
  sp0 = float(np.interp(t_base, cs_t, setpoint))
  sp1 = float(np.interp(t_settle, cs_t, setpoint))
  return int(round((sp1 - sp0) * 3.6 / CMD_STEP[burst.cmd]))


def burst_is_clean(burst, cs_t, clean):
  """True when the burst starts inside an engaged-clean stretch."""
  if not len(cs_t) or not np.any(clean):
    return False
  i = int(np.argmin(np.abs(cs_t - burst.t_start)))
  return bool(clean[i]) and abs(float(cs_t[i]) - burst.t_start) < BURST_MATCH_S


def deadzone_sign(x, dz):
  """+1 / -1 outside the deadzone, 0 inside it."""
  return np.where(x > dz, 1.0, np.where(x < -dz, -1.0, 0.0))


def sign_change_count(x, dz):
  """Transitions between + and - ignoring the deadzone, i.e. real reversals."""
  s = deadzone_sign(np.asarray(x, dtype=np.float64), dz)
  s = s[s != 0]
  return int(np.count_nonzero(np.diff(s) != 0)) if len(s) > 1 else 0


def direction_agreement(v_error, a_target, v_dz=V_DEADZONE, a_dz=A_DEADZONE):
  """(n compared, n disagreeing) for sign(v_error) vs sign(aTarget).

  Samples inside either deadzone carry no direction and are excluded. Counts,
  not a fraction, so that per-segment results add up exactly.
  """
  v_error = np.asarray(v_error, dtype=np.float64)
  a_target = np.asarray(a_target, dtype=np.float64)
  m = (np.abs(v_error) > v_dz) & (np.abs(a_target) > a_dz)
  n = int(np.count_nonzero(m))
  if n == 0:
    return 0, 0
  return n, int(np.count_nonzero(np.sign(v_error[m]) != np.sign(a_target[m])))


def clean_runs(mask):
  """Contiguous [start, stop) index ranges where `mask` is True."""
  idx = np.flatnonzero(mask)
  if not len(idx):
    return []
  parts = np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1)
  return [(int(g[0]), int(g[-1]) + 1) for g in parts]


def sign_changes_masked(x, mask, dz):
  """sign_change_count over contiguous clean runs only.

  Run boundaries are dropped so a gap between two engaged stretches cannot fake
  a reversal.
  """
  return sum(sign_change_count(x[lo:hi], dz) for lo, hi in clean_runs(mask))


def setpoint_churn(setpoint, mask, eps=SP_EPS):
  """(total |travel|, |net change|, reversals) of the setpoint over clean runs.

  Travel far above net change means the setpoint was pumped up and down to end
  up nowhere, which is exactly the churn the setpoint-error law is meant to kill.
  """
  travel = net = 0.0
  reversals = 0
  for lo, hi in clean_runs(mask):
    sp = np.asarray(setpoint[lo:hi], dtype=np.float64)
    if len(sp) < 2:
      continue
    d = np.diff(sp)
    d = d[np.abs(d) > eps]
    if not len(d):
      continue
    travel += float(np.sum(np.abs(d)))
    net += abs(float(np.sum(d)))
    reversals += int(np.count_nonzero(np.diff(np.sign(d)) != 0))
  return travel, net, reversals


def label_bias(median_err):
  """Sign convention: aEgo - aTarget > 0 while braking = not braking enough."""
  if np.isnan(median_err):
    return "n/a"
  return "under-braking" if median_err > 0 else "over-braking"


def _pct(v):
  return f"{100.0 * v:5.1f}%" if not np.isnan(v) else "  n/a"


def _delta(new, base):
  if np.isnan(new) or np.isnan(base) or abs(base) < 1e-9:
    return "    n/a"
  return f"{100.0 * (new - base) / abs(base):+6.1f}%"


def _med(a):
  return float(np.median(a)) if len(a) else float("nan")


# ------------------------------------------------------------------ evaluation

def iter_segments(extracted, prefix):
  """Yield (name, seg) for every extracted .npz whose filename starts prefix.

  A generator so a 500-segment baseline set never sits in memory at once.
  """
  for p in sorted(Path(extracted).glob(prefix + "*.npz")):
    with np.load(p) as npz:
      yield p.stem, {k: npz[k] for k in npz.files}


def evaluate(segments):
  """All metrics for one set of segments. `segments` is an iterable of segs."""
  m = {"n_used": 0, "n_skipped": 0, "minutes": 0.0, "n_clean": 0,
       "have_vtarget": True}
  abs_err, sgn_err, ls_err = [], [], []
  n_bursts = n_close = n_reversal = 0
  durations = []
  travel = net = 0.0
  sp_reversals = 0
  cells = defaultdict(lambda: {"ticks": [], "iso_ticks": [], "itvl": [],
                               "n_iso": 0})
  da_n = da_bad = 0
  a_flips = v_flips = 0

  for _name, seg in segments:
    if not segment_usable(seg):
      m["n_skipped"] += 1
      continue
    m["n_used"] += 1
    cs_t, clean = seg["cs_t"], clean_mask(seg)
    n_clean = int(np.count_nonzero(clean))
    m["n_clean"] += n_clean
    m["minutes"] += n_clean * sample_dt(cs_t) / 60.0

    # --- tracking: planner accel request vs achieved accel
    a_tgt = on_cs_grid(seg, "aTarget")
    err = seg["aEgo"][clean] - a_tgt[clean]
    abs_err.append(np.abs(err))
    sgn_err.append(err)

    low = clean & (seg["vEgo"] < LOW_SPEED) & (a_tgt < BRAKE_ATARGET)
    if np.any(low):
      ls_err.append(seg["aEgo"][low] - a_tgt[low])

    # --- churn: isolation/adjacency judged against ALL bursts, counted on the
    # clean ones, because a contaminating neighbour contaminates regardless.
    bursts = find_bursts(seg)
    bursts.sort(key=lambda b: b.t_start)
    iso = isolated_flags(bursts)
    for i, b in enumerate(bursts):
      if not burst_is_clean(b, cs_t, clean):
        continue
      n_bursts += 1
      durations.append(b.duration)
      if i + 1 < len(bursts) and bursts[i + 1].t_start - b.t_end <= CLOSE_S:
        n_close += 1
        if CMD_STEP[b.cmd] * CMD_STEP[bursts[i + 1].cmd] < 0:
          n_reversal += 1
      ticks = burst_ticks(b, cs_t, seg["setpoint"])
      if ticks is None:
        continue
      cell = cells[(b.cmd, b.cadence)]
      cell["ticks"].append(ticks)
      cell["itvl"].append(b.duration / (b.n_frames - 1))
      if iso[i]:
        cell["n_iso"] += 1
        cell["iso_ticks"].append(ticks)

    tv, nt, rv = setpoint_churn(seg["setpoint"], clean)
    travel += tv
    net += nt
    sp_reversals += rv

    # --- direction agreement: does the planner accelerate toward its own target
    if "vTarget" not in seg:
      m["have_vtarget"] = False
      continue
    v_err = on_cs_grid(seg, "vTarget") - seg["vEgo"]
    n, bad = direction_agreement(v_err[clean], a_tgt[clean])
    da_n += n
    da_bad += bad
    a_flips += sign_changes_masked(a_tgt, clean, A_DEADZONE)
    v_flips += sign_changes_masked(v_err, clean, V_DEADZONE)

  if not m["n_used"]:
    return None

  per_min = (lambda n: n / m["minutes"]) if m["minutes"] > 0 else (lambda n: float("nan"))
  a = np.concatenate(abs_err) if abs_err else np.array([])
  s = np.concatenate(sgn_err) if sgn_err else np.array([])
  ls = np.concatenate(ls_err) if ls_err else np.array([])
  d = np.asarray(durations)

  m["trk_med"] = _med(a)
  m["trk_p75"] = float(np.percentile(a, 75)) if len(a) else float("nan")
  m["trk_p90"] = float(np.percentile(a, 90)) if len(a) else float("nan")
  m["trk_signed"] = _med(s)
  m["ls_n"] = int(len(ls))
  m["ls_med"] = _med(ls)
  m["bursts_per_min"] = per_min(n_bursts)
  m["n_bursts"] = n_bursts
  m["close_per_min"] = per_min(n_close)
  m["rev_per_min"] = per_min(n_reversal)
  m["dur_med"] = _med(d)
  m["dur_frac_short"] = float(np.mean(d <= SHORT_BURST_S)) if len(d) else float("nan")
  m["sp_travel"] = travel
  m["sp_net"] = net
  m["churn_ratio"] = travel / net if net > 1e-9 else float("nan")
  m["sp_rev_per_min"] = per_min(sp_reversals)
  m["cells"] = cells
  m["da_n"] = da_n
  m["da_frac"] = da_bad / da_n if da_n else float("nan")
  m["a_flips_per_min"] = per_min(a_flips)
  m["v_flips_per_min"] = per_min(v_flips)
  return m


# -------------------------------------------------------------------- printing

def print_set(label, m):
  print(f"\n=== {label} ===")
  print(f"segments: {m['n_used']} used, {m['n_skipped']} skipped "
        f"(<{MIN_CLEAN} clean or <{MIN_TOTAL} total samples)")
  print(f"engaged clean: {m['minutes']:.1f} min, {m['n_clean']} samples")

  print(f"\nTRACKING |aEgo - aTarget| (m/s^2): median {m['trk_med']:.3f}  "
        f"p75 {m['trk_p75']:.3f}  p90 {m['trk_p90']:.3f}")
  print(f"  signed median (aEgo - aTarget): {m['trk_signed']:+.3f}")

  print(f"\nLOW-SPEED BRAKING (vEgo < {LOW_SPEED:.0f} m/s, "
        f"aTarget < {BRAKE_ATARGET}): n={m['ls_n']}")
  if m["ls_n"] < LOW_SPEED_MIN_N:
    print(f"  insufficient (n < {LOW_SPEED_MIN_N})")
  else:
    print(f"  median signed error {m['ls_med']:+.3f} m/s^2 "
          f"-> {label_bias(m['ls_med'])}")

  print(f"\nCHURN ({m['n_bursts']} clean bursts)")
  print(f"  bursts/min                 {m['bursts_per_min']:6.2f}")
  print(f"  changes within {CLOSE_S:.0f}s /min      {m['close_per_min']:6.2f}")
  print(f"  ...direction reversals/min {m['rev_per_min']:6.2f}")
  print(f"  burst duration median      {m['dur_med']:6.3f} s, "
        f"{_pct(m['dur_frac_short'])} <= {SHORT_BURST_S:.2f} s")
  print(f"  setpoint travel {m['sp_travel'] * 3.6:.1f} kph vs net "
        f"{m['sp_net'] * 3.6:.1f} kph -> churn ratio {m['churn_ratio']:.2f}")
  print(f"  setpoint direction reversals/min {m['sp_rev_per_min']:6.2f}")

  print(f"\nACCEPTANCE (settle {SETTLE_S:.1f}s, isolation {ISOLATION_S:.0f}s "
        f"both sides)")
  print(f"  {'cmd':8s}{'cadence':8s}{'n':>5s}{'iso':>5s}{'itvl_ms':>9s}"
        f"{'ticks':>7s}{'>=1tick':>9s}{'wrong':>8s}")
  for cmd in CMDS4:
    for cadence in CADENCES:
      cell = m["cells"].get((cmd, cadence))
      if not cell or not cell["ticks"]:
        continue
      t = np.asarray(cell["ticks"], dtype=np.float64)
      itvl = _med(np.asarray(cell["itvl"])) * 1000.0
      head = (f"  {cmd:8s}{cadence:8s}{len(t):5d}{cell['n_iso']:5d}"
              f"{itvl:9.1f}{np.mean(t):7.2f}")
      if cell["n_iso"] < ISOLATED_MIN_N:
        # Contaminated acceptance numbers mislead badly (see module docstring).
        print(head + f"   UNMEASURABLE (n isolated = {cell['n_iso']})")
        continue
      iso = np.asarray(cell["iso_ticks"], dtype=np.float64)
      print(head + f"{_pct(float(np.mean(iso >= 1))):>9s}"
                   f"{_pct(float(np.mean(iso <= -1))):>8s}")
  print("  (>=1tick/wrong from the isolated subset; n = all clean bursts)")

  print("\nDIRECTION AGREEMENT (vTarget - vEgo vs aTarget)")
  if not m["have_vtarget"]:
    print("  SKIPPED: vTarget missing from at least one .npz — "
          "re-run extract.py to backfill it")
  elif not m["da_n"]:
    print("  no samples outside the deadzones")
  else:
    print(f"  n={m['da_n']}  disagreeing {_pct(m['da_frac'])}")
    print(f"  sign changes/min: aTarget {m['a_flips_per_min']:6.2f}, "
          f"v_error {m['v_flips_per_min']:6.2f}")


HEADLINES = (("tracking |err| median", "trk_med", "{:.3f}"),
             ("low-speed braking med", "ls_med", "{:+.3f}"),
             ("bursts/min", "bursts_per_min", "{:.2f}"),
             ("direction reversals/min", "rev_per_min", "{:.2f}"),
             ("churn ratio", "churn_ratio", "{:.2f}"))


def print_comparison(route_label, route, base_label, base):
  print("\n" + "=" * 66)
  print(f"HEADLINE  route={route_label}  baseline={base_label}")
  print(f"  {'metric':24s}{'route':>10s}{'baseline':>12s}{'delta':>10s}")
  for name, key, fmt in HEADLINES:
    r, b = route.get(key, float("nan")), base.get(key, float("nan"))
    rs = fmt.format(r) if not np.isnan(r) else "n/a"
    bs = fmt.format(b) if not np.isnan(b) else "n/a"
    print(f"  {name:24s}{rs:>10s}{bs:>12s}{_delta(r, b):>10s}")
  if route["ls_n"] < LOW_SPEED_MIN_N or base["ls_n"] < LOW_SPEED_MIN_N:
    print(f"  NOTE: low-speed braking n = {route['ls_n']} / {base['ls_n']}; "
          f"below {LOW_SPEED_MIN_N} it is insufficient, not an improvement")


def main():
  p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  p.add_argument("--route", required=True,
                 help="extracted .npz filename prefix, e.g. 000003d4")
  p.add_argument("--baseline", default="2026-07-2",
                 help="baseline prefix (default: the pre-change date-named routes)")
  p.add_argument("--extracted", default=EXTRACTED_DIR, type=Path)
  args = p.parse_args()

  print(f"burst segmentation: gap {GAP_S:.2f}s, "
        f"hold cadence <= {HOLD_MAX_INTERVAL * 1000:.0f}ms inter-frame "
        f"(shared with bursts.py)")
  out = {}
  for kind, prefix in (("route", args.route), ("baseline", args.baseline)):
    m = evaluate(iter_segments(args.extracted, prefix))
    if m is None:
      sys.exit(f"no usable segments for {kind} prefix {prefix!r} "
               f"under {args.extracted}")
    print_set(f"{kind.upper()} {prefix}", m)
    out[kind] = m
  print_comparison(args.route, out["route"], args.baseline, out["baseline"])


if __name__ == "__main__":
  main()
