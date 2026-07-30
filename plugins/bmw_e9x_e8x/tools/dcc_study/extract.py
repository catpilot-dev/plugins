"""rlog.zst -> per-segment .npz of the channels the DCC study needs."""
import argparse
import sys
from pathlib import Path

import numpy as np
import zstandard

from common import STALK_ADDR, ROUTES_DIR, EXTRACTED_DIR, decode_stalk

CMDS = ("plus1", "plus5", "minus1", "minus5", "cancel", "resume", "cancel_lever_up")

_KEYS = ("cs_t", "vEgo", "aEgo", "setpoint", "cruiseEnabled", "gas", "brake",
         "ctrl_t", "aTarget", "vTarget", "ctrlEnabled",
         "tx_t", "tx_cmd", "rx_t", "rx_cmd", "pose_t", "pitch")


def extract_segment(rlog_path):
  from cereal import log  # heavy import kept out of module import time

  try:
    raw = zstandard.ZstdDecompressor().decompress(
      rlog_path.read_bytes(), max_output_size=2 ** 30)
    events = log.Event.read_multiple_bytes(raw, traversal_limit_in_words=2 ** 61)
    out = {k: [] for k in _KEYS}
    for evt in events:
      which = evt.which()
      t = evt.logMonoTime / 1e9
      if which == "carState":
        cs = evt.carState
        out["cs_t"].append(t)
        out["vEgo"].append(cs.vEgo)
        out["aEgo"].append(cs.aEgo)
        out["setpoint"].append(cs.cruiseState.speed)
        out["cruiseEnabled"].append(float(cs.cruiseState.enabled))
        out["gas"].append(float(cs.gasPressed))
        out["brake"].append(float(cs.brakePressed))
      elif which == "carControl":
        out["ctrl_t"].append(t)
        out["aTarget"].append(evt.carControl.actuators.accel)
        out["vTarget"].append(evt.carControl.actuators.speed)
        out["ctrlEnabled"].append(float(evt.carControl.enabled))
      elif which == "sendcan":
        for c in evt.sendcan:
          if c.address == STALK_ADDR:
            _, cmd = decode_stalk(bytes(c.dat))
            out["tx_t"].append(t)
            out["tx_cmd"].append(CMDS.index(cmd) if cmd is not None else -1)
      elif which == "can":
        for c in evt.can:
          if c.address == STALK_ADDR:
            _, cmd = decode_stalk(bytes(c.dat))
            if cmd is not None:  # only action frames matter for contamination
              out["rx_t"].append(t)
              out["rx_cmd"].append(CMDS.index(cmd))
      elif which == "livePose":
        out["pose_t"].append(t)
        out["pitch"].append(evt.livePose.orientationNED.y)
  except Exception as e:  # corrupt/truncated logs are expected occasionally
    print(f"WARNING: skipping {rlog_path}: {e}", file=sys.stderr)
    return None
  return {k: np.asarray(v, dtype=np.int8 if k in ("tx_cmd", "rx_cmd") else np.float64)
          for k, v in out.items()}


def needs_extract(dest):
  """True when `dest` is absent, unreadable, or predates a key added to _KEYS.

  Keys get added as the study needs new channels (vTarget, ...). Re-extracting
  only the stale files keeps a schema change from silently leaving a mixed set
  of .npz behind, which is far worse than the extra CPU.
  """
  if not dest.exists():
    return True
  try:
    with np.load(dest) as npz:
      missing = [k for k in _KEYS if k not in set(npz.files)]
  except Exception as e:  # truncated/partial write from an interrupted run
    print(f"WARNING: re-extracting unreadable {dest.name}: {e}", file=sys.stderr)
    return True
  if missing:
    print(f"{dest.name}: re-extracting, missing {','.join(missing)}")
    return True
  return False


def main():
  p = argparse.ArgumentParser()
  p.add_argument("--routes", default=ROUTES_DIR, type=Path)
  p.add_argument("--out", default=EXTRACTED_DIR, type=Path)
  args = p.parse_args()

  args.out.mkdir(parents=True, exist_ok=True)
  rlogs = sorted(args.routes.glob("**/rlog.zst"))
  if not rlogs:
    sys.exit(f"no rlog.zst under {args.routes} — run fetch_routes.py first")
  for rlog in rlogs:
    dest = args.out / (rlog.parent.name + ".npz")
    if not needs_extract(dest):
      continue
    seg = extract_segment(rlog)
    if seg is not None:
      np.savez_compressed(dest, **seg)
      print(f"{rlog.parent.name}: {len(seg['cs_t'])} carState, "
            f"{len(seg['tx_t'])} stalk TX")


if __name__ == "__main__":
  main()
