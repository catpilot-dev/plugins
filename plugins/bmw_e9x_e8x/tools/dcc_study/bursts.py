"""Segment injected stalk commands into bursts and measure the car's response."""
import argparse
import csv
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from common import CMD_STEP, EXTRACTED_DIR, DATA_DIR, PROFILES_DIR
from extract import CMDS

GAP_S = 0.5              # mirrors carcontroller BURST_LIVE_WINDOW
HOLD_MAX_INTERVAL = 0.035  # median inter-frame <= this -> 40 Hz "hold"
PAD_PRE = 0.5            # s before burst: baseline window / contamination pad
PAD_POST = 1.5           # s after burst: response tail / contamination pad
HUMAN_MATCH_S = 0.05     # rx action frame within this of a tx frame = our echo
HUMAN_PAD_S = 2.0        # human press within this of the burst -> contaminated
STEADY_MIN_DUR = 1.0     # s: bursts shorter than this get no steady-state value
STEADY_SKIP = 0.7        # s: skipped at burst start before steady-state averaging


@dataclass
class Burst:
  t_start: float
  t_end: float
  cmd: str
  cadence: str
  n_frames: int
  v_start: float = float("nan")
  setpoint_gap: float = float("nan")
  pitch_mean: float = float("nan")
  a_baseline: float = float("nan")
  peak_delta_a: float = float("nan")
  steady_delta_a: float = float("nan")
  rise_time: float = float("nan")
  ticks_accepted: int = 0

  @property
  def duration(self):
    return self.t_end - self.t_start


def find_bursts(seg):
  bursts = []
  run_t, run_cmd = [], None
  speed_codes = {CMDS.index(c) for c in CMD_STEP}

  def close():
    if run_cmd is None or len(run_t) < 2:
      return
    itvl = float(np.median(np.diff(run_t)))
    bursts.append(Burst(t_start=run_t[0], t_end=run_t[-1], cmd=run_cmd,
                        cadence="hold" if itvl <= HOLD_MAX_INTERVAL else "single",
                        n_frames=len(run_t)))

  for t, code in zip(seg["tx_t"], seg["tx_cmd"]):
    if code not in speed_codes:      # neutral (-1), cancel, resume: not a command
      continue
    cmd = CMDS[code]
    if run_cmd is not None and (cmd != run_cmd or t - run_t[-1] > GAP_S):
      close()
      run_t = []
    run_t.append(float(t))
    run_cmd = cmd
  close()
  return bursts


def is_contaminated(burst, seg):
  lo, hi = burst.t_start - PAD_PRE, burst.t_end + PAD_POST
  win = (seg["cs_t"] >= lo) & (seg["cs_t"] <= hi)
  if np.any(seg["gas"][win] > 0) or np.any(seg["brake"][win] > 0):
    return True
  # Human stalk press: an rx action frame that no tx frame of ours explains.
  near = (seg["rx_t"] >= burst.t_start - HUMAN_PAD_S) & \
         (seg["rx_t"] <= burst.t_end + HUMAN_PAD_S)
  for rt in seg["rx_t"][near]:
    if len(seg["tx_t"]) == 0 or np.min(np.abs(seg["tx_t"] - rt)) > HUMAN_MATCH_S:
      return True
  return False


def _mean_in(t, y, lo, hi):
  m = (t >= lo) & (t < hi)
  return float(np.mean(y[m])) if np.any(m) else float("nan")


def measure(burst, seg):
  cs_t, aEgo = seg["cs_t"], seg["aEgo"]
  t0, t1 = burst.t_start, burst.t_end
  sign = 1.0 if CMD_STEP[burst.cmd] > 0 else -1.0

  burst.a_baseline = _mean_in(cs_t, aEgo, t0 - PAD_PRE, t0)
  burst.v_start = float(np.interp(t0, cs_t, seg["vEgo"]))
  burst.setpoint_gap = float(np.interp(t0 - 0.1, cs_t, seg["setpoint"])) - burst.v_start

  win = (cs_t >= t0) & (cs_t <= t1 + PAD_POST)
  delta = (aEgo[win] - burst.a_baseline) * sign          # response, positive = "as commanded"
  if len(delta):
    burst.peak_delta_a = float(np.max(delta)) * sign
  if burst.duration >= STEADY_MIN_DUR:
    steady = _mean_in(cs_t, aEgo, t0 + STEADY_SKIP, t1) - burst.a_baseline
    burst.steady_delta_a = steady
    target = abs(steady) * 0.63
    tw = cs_t[win]
    reached = np.nonzero(delta >= target)[0] if not np.isnan(steady) else []
    if len(reached):
      burst.rise_time = float(tw[reached[0]] - t0)

  sp0 = np.interp(t0 - 0.1, cs_t, seg["setpoint"])
  sp1 = np.interp(t1 + 0.5, cs_t, seg["setpoint"])
  burst.ticks_accepted = int(round((sp1 - sp0) * 3.6 / CMD_STEP[burst.cmd]))

  if len(seg["pose_t"]):
    burst.pitch_mean = _mean_in(seg["pose_t"], seg["pitch"], t0 - PAD_PRE, t1 + PAD_POST)
  return burst


CSV_FIELDS = ("route", "segment", "t_start", "duration_s", "cmd", "cadence",
              "n_frames", "v_start_mps", "setpoint_gap_mps", "pitch_rad",
              "a_baseline", "peak_delta_a", "steady_delta_a", "rise_time_s",
              "ticks_accepted")


def burst_row(burst, route, segment):
  return {"route": route, "segment": segment, "t_start": round(burst.t_start, 3),
          "duration_s": round(burst.duration, 3), "cmd": burst.cmd,
          "cadence": burst.cadence, "n_frames": burst.n_frames,
          "v_start_mps": round(burst.v_start, 3),
          "setpoint_gap_mps": round(burst.setpoint_gap, 3),
          "pitch_rad": round(burst.pitch_mean, 5),
          "a_baseline": round(burst.a_baseline, 4),
          "peak_delta_a": round(burst.peak_delta_a, 4),
          "steady_delta_a": round(burst.steady_delta_a, 4),
          "rise_time_s": round(burst.rise_time, 3),
          "ticks_accepted": burst.ticks_accepted}


def write_csv(rows, path):
  with open(path, "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
    w.writeheader()
    w.writerows(rows)


def main():
  p = argparse.ArgumentParser()
  p.add_argument("--extracted", default=EXTRACTED_DIR, type=Path)
  args = p.parse_args()

  PROFILES_DIR.mkdir(parents=True, exist_ok=True)
  rows, kept, dropped = [], 0, 0
  for npz in sorted(args.extracted.glob("*.npz")):
    seg = dict(np.load(npz))
    route, _, segment = npz.stem.rpartition("--")
    for i, b in enumerate(find_bursts(seg)):
      if is_contaminated(b, seg):
        dropped += 1
        continue
      measure(b, seg)
      rows.append(burst_row(b, route, segment))
      win = (seg["cs_t"] >= b.t_start - PAD_PRE) & (seg["cs_t"] <= b.t_end + PAD_POST)
      np.savez_compressed(PROFILES_DIR / f"{npz.stem}--{i}.npz",
                          t=seg["cs_t"][win] - b.t_start, aEgo=seg["aEgo"][win])
      kept += 1
  if not rows:
    sys.exit("no clean bursts found — check extraction output")
  write_csv(rows, DATA_DIR / "bursts.csv")
  print(f"{kept} bursts kept, {dropped} contaminated -> {DATA_DIR / 'bursts.csv'}")


if __name__ == "__main__":
  main()
