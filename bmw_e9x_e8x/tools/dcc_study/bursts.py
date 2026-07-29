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
