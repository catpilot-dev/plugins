"""bursts.csv -> plots + coverage/acceptance summary for the phase gate."""
import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from common import DATA_DIR, REPORT_DIR

CMDS4 = ("plus1", "plus5", "minus1", "minus5")
CADENCE_COLOR = {"hold": "tab:red", "single": "tab:blue"}
PITCH_FLAT = 0.017  # rad, ~1 deg / ~2 % grade — primary-fit filter (spec)

_INT = {"segment", "n_frames", "ticks_accepted"}
_STR = {"route", "cmd", "cadence"}


def load_bursts(csv_path):
  rows = []
  with open(csv_path) as f:
    for r in csv.DictReader(f):
      rows.append({k: (v if k in _STR else int(v) if k in _INT else float(v))
                   for k, v in r.items()})
  return rows


def delta_a(row):
  return row["steady_delta_a"] if not math.isnan(row["steady_delta_a"]) \
      else row["peak_delta_a"]


def speed_bin(v_mps):
  return int(v_mps * 3.6 // 10) * 10


def _bin_medians(rows):
  """(cmd, cadence, speed_bin) -> median delta_a, flat-pitch rows only."""
  groups = defaultdict(list)
  for r in rows:
    if not math.isnan(r["pitch_rad"]) and abs(r["pitch_rad"]) < PITCH_FLAT:
      groups[(r["cmd"], r["cadence"], speed_bin(r["v_start_mps"]))].append(delta_a(r))
  return {k: float(np.median(v)) for k, v in groups.items()}


def generate(csv_path, out_dir):
  rows = load_bursts(csv_path)
  out_dir = Path(out_dir)
  out_dir.mkdir(parents=True, exist_ok=True)
  medians = _bin_medians(rows)

  # response vs speed, one panel per command
  fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharex=True)
  for ax, cmd in zip(axes.flat, CMDS4):
    for cadence, color in CADENCE_COLOR.items():
      sel = [r for r in rows if r["cmd"] == cmd and r["cadence"] == cadence]
      ax.scatter([r["v_start_mps"] * 3.6 for r in sel], [delta_a(r) for r in sel],
                 s=12, alpha=0.4, color=color, label=f"{cadence} (n={len(sel)})")
      pts = sorted((sb + 5, m) for (c, cad, sb), m in medians.items()
                   if c == cmd and cad == cadence)
      if pts:
        ax.plot(*zip(*pts), color=color, marker="o")
    ax.set_title(cmd)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
  fig.supxlabel("vEgo at burst start (km/h)")
  fig.supylabel("achieved Δaccel (m/s²)")
  fig.savefig(out_dir / "response_vs_speed.png", dpi=120)
  plt.close(fig)

  # residuals vs setpoint gap and pitch
  for field, fname in (("setpoint_gap_mps", "residual_vs_setpoint_gap.png"),
                       ("pitch_rad", "residual_vs_pitch.png")):
    fig, ax = plt.subplots(figsize=(8, 5))
    xs, ys = [], []
    for r in rows:
      key = (r["cmd"], r["cadence"], speed_bin(r["v_start_mps"]))
      if key in medians and not math.isnan(r[field]):
        xs.append(r[field])
        ys.append(delta_a(r) - medians[key])
    ax.scatter(xs, ys, s=12, alpha=0.4)
    ax.axhline(0, color="k", lw=0.5)
    ax.set_xlabel(field)
    ax.set_ylabel("Δaccel residual vs bin median (m/s²)")
    ax.grid(alpha=0.3)
    fig.savefig(out_dir / fname, dpi=120)
    plt.close(fig)

  # summary tables
  lines = ["DCC response study summary", "=" * 40, "",
           "Coverage (bursts per cmd x cadence x 10 km/h speed bin):"]
  bins = sorted({speed_bin(r["v_start_mps"]) for r in rows})
  lines.append(f"{'cmd':8s}{'cadence':8s}" + "".join(f"{b:>7d}" for b in bins))
  counts = defaultdict(int)
  for r in rows:
    counts[(r["cmd"], r["cadence"], speed_bin(r["v_start_mps"]))] += 1
  for cmd in CMDS4:
    for cadence in ("hold", "single"):
      row = [counts.get((cmd, cadence, b), 0) for b in bins]
      lines.append(f"{cmd:8s}{cadence:8s}" + "".join(f"{n:>7d}" for n in row))
  lines += ["", "Median response, flat pitch (m/s²):"]
  for (cmd, cadence, sb), m in sorted(medians.items()):
    lines.append(f"  {cmd:8s}{cadence:8s}{sb:>4d}-{sb + 10:d} km/h: {m:+.3f}")
  lines += ["", "Acceptance (mean accepted ticks per burst):"]
  acc = defaultdict(list)
  for r in rows:
    acc[(r["cmd"], r["cadence"])].append(r["ticks_accepted"])
  for (cmd, cadence), v in sorted(acc.items()):
    lines.append(f"  {cmd:8s}{cadence:8s}: {np.mean(v):5.1f} ticks over {len(v)} bursts")
  (out_dir / "summary.txt").write_text("\n".join(lines) + "\n")
  print("\n".join(lines))


def main():
  p = argparse.ArgumentParser()
  p.add_argument("--csv", default=DATA_DIR / "bursts.csv", type=Path)
  p.add_argument("--out", default=REPORT_DIR, type=Path)
  args = p.parse_args()
  generate(args.csv, args.out)


if __name__ == "__main__":
  main()
