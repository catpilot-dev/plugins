# DCC Response Study (Phase 1) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Offline tooling that measures the BMW E9x's actual acceleration response per DCC stalk command×cadence from C3 route rlogs, producing the report reviewed at the Phase-2 gate.

**Architecture:** Four standalone scripts in `plugins/bmw_e9x_e8x/tools/dcc_study/` forming a pipeline: `fetch_routes.py` (COD API → local rlog cache) → `extract.py` (rlog.zst → per-segment .npz of time-aligned channels) → `bursts.py` (.npz → per-burst measurements in bursts.csv + profile .npz) → `report.py` (bursts.csv → plots + coverage/acceptance tables). Pure logic (stalk decode, segmentation, measurement) is separated from I/O and unit-tested with synthetic data.

**Tech Stack:** Python via catpilot's venv (`pycapnp`/cereal, `zstandard`, `numpy`, `matplotlib`, `requests`). No pandas/pyarrow — npz + csv only.

**Spec:** `docs/superpowers/specs/2026-07-29-dcc-response-mapping-design.md`

## Global Constraints

- Repo: `/home/oxygen/catpilot-dev/plugins`, branch `dev`. Commit after every task; no `Co-Authored-By` lines.
- All commands below run from `plugins/bmw_e9x_e8x/tools/dcc_study/` unless stated.
- Interpreter line (call it `$PY` below):
  `PYTHONPATH=/home/oxygen/catpilot-dev/catpilot:. /home/oxygen/catpilot-dev/catpilot/.venv/bin/python`
- Tests: `$PY -m pytest tests/ -v` from the `dcc_study` dir. The repo pre-push hook glob (`plugins/*/tests/`) does NOT collect these — always run them explicitly.
- Never hardcode the C3 address: resolve via `ssh -G c3` (currently yields `catpilot.local`). COD API base is `http://<host>:8082`, no real auth.
- `data/` under `dcc_study/` is gitignored (route rlogs, npz, csv, pngs).
- Vision-only constraint: grade proxy is `livePose` pitch only; no GPS/map inputs.
- CruiseControlStalk wire format (DBC `BO_ 404`, 4 bytes, verified against `dbc/bmw_e9x_e8x.dbc`):
  `dat[0]`=checksum, `dat[1]`=(requests<<4)|counter → counter=`dat[1]&0x0F`,
  `dat[2]` bits: 0x01 plus1, 0x02 plus5, 0x04 minus1, 0x08 minus5, 0x10 cancel, 0x40 resume, 0x80 cancel_lever_up,
  `dat[3]`=0xFC.
- Controller constants this tooling must mirror (from `bmw/carcontroller.py`): burst gap `BURST_LIVE_WINDOW = 0.5` s; cadences 40 Hz (hold) / 20 Hz (single).

---

### Task 1: Scaffold + stalk frame decoder (`common.py`)

**Files:**
- Create: `plugins/bmw_e9x_e8x/tools/dcc_study/.gitignore`
- Create: `plugins/bmw_e9x_e8x/tools/dcc_study/README.md`
- Create: `plugins/bmw_e9x_e8x/tools/dcc_study/common.py`
- Create: `plugins/bmw_e9x_e8x/tools/dcc_study/tests/conftest.py`
- Test: `plugins/bmw_e9x_e8x/tools/dcc_study/tests/test_stalk_decode.py`

**Interfaces:**
- Produces: `STALK_ADDR = 404`; `CMD_STEP = {'plus1': 1, 'plus5': 5, 'minus1': -1, 'minus5': -5}` (kph per accepted tick, signed); `decode_stalk(dat: bytes) -> tuple[int, str | None]` returning `(counter, cmd_name_or_None)` — `None` when no action bit is set (neutral/counter-overwrite frame); `DATA_DIR`, `ROUTES_DIR`, `EXTRACTED_DIR`, `PROFILES_DIR`, `REPORT_DIR` (pathlib Paths under `dcc_study/data/`).

- [ ] **Step 1: Create scaffold files**

`.gitignore`:
```
data/
__pycache__/
```

`tests/conftest.py` (makes `common` etc. importable from tests):
```python
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
```

`README.md`:
```markdown
# DCC Response Study

Measures actual vehicle acceleration per DCC stalk command×cadence from C3 rlogs.
Spec: docs/superpowers/specs/2026-07-29-dcc-response-mapping-design.md

## Run (dev machine, from this directory)

    PY="PYTHONPATH=/home/oxygen/catpilot-dev/catpilot:. /home/oxygen/catpilot-dev/catpilot/.venv/bin/python"
    # 1. pull rlogs of engaged routes from the C3 via COD API
    $PY fetch_routes.py
    # 2. rlog.zst -> per-segment npz
    $PY extract.py
    # 3. npz -> data/bursts.csv (+ data/profiles/*.npz)
    $PY bursts.py
    # 4. bursts.csv -> data/report/*.png + summary.txt
    $PY report.py

Tests: `$PY -m pytest tests/ -v` (NOT collected by the repo pre-push hook).
```

- [ ] **Step 2: Write the failing test**

`tests/test_stalk_decode.py`:
```python
import pytest

from common import decode_stalk, CMD_STEP


def frame(counter=0, byte2=0x00):
  return bytes([0x00, 0xF0 | (counter & 0x0F), byte2, 0xFC])


@pytest.mark.parametrize("byte2,cmd", [
  (0x01, "plus1"), (0x02, "plus5"), (0x04, "minus1"), (0x08, "minus5"),
  (0x10, "cancel"), (0x40, "resume"), (0x80, "cancel_lever_up"),
])
def test_decodes_each_command(byte2, cmd):
  counter, decoded = decode_stalk(frame(counter=7, byte2=byte2))
  assert counter == 7
  assert decoded == cmd


def test_neutral_frame_decodes_to_none():
  counter, decoded = decode_stalk(frame(counter=14))
  assert counter == 14
  assert decoded is None


def test_cmd_step_signs():
  assert CMD_STEP == {"plus1": 1, "plus5": 5, "minus1": -1, "minus5": -5}
```

- [ ] **Step 3: Run test to verify it fails**

Run: `$PY -m pytest tests/test_stalk_decode.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'common'`

- [ ] **Step 4: Write `common.py`**

```python
"""Shared constants and CruiseControlStalk (0x194) frame decoding.

Wire format (DBC BO_ 404, 4 bytes):
  dat[0] checksum, dat[1] = (requests << 4) | counter,
  dat[2] action bits (see _CMD_BITS), dat[3] = 0xFC.
"""
from pathlib import Path

STALK_ADDR = 404  # 0x194

_CMD_BITS = (
  (0x01, "plus1"),
  (0x02, "plus5"),
  (0x04, "minus1"),
  (0x08, "minus5"),
  (0x10, "cancel"),
  (0x40, "resume"),
  (0x80, "cancel_lever_up"),
)

# kph moved per accepted tick, signed. Only these four are speed commands.
CMD_STEP = {"plus1": 1, "plus5": 5, "minus1": -1, "minus5": -5}

DATA_DIR = Path(__file__).resolve().parent / "data"
ROUTES_DIR = DATA_DIR / "routes"
EXTRACTED_DIR = DATA_DIR / "extracted"
PROFILES_DIR = DATA_DIR / "profiles"
REPORT_DIR = DATA_DIR / "report"


def decode_stalk(dat: bytes) -> tuple[int, str | None]:
  """Return (counter, command name or None for a neutral frame)."""
  counter = dat[1] & 0x0F
  for bit, name in _CMD_BITS:
    if dat[2] & bit:
      return counter, name
  return counter, None
```

- [ ] **Step 5: Run test to verify it passes**

Run: `$PY -m pytest tests/test_stalk_decode.py -v`
Expected: 9 passed

- [ ] **Step 6: Commit**

```bash
git add plugins/bmw_e9x_e8x/tools/dcc_study
git commit -m "dcc_study: scaffold + stalk frame decoder"
```

---

### Task 2: rlog extraction (`extract.py`)

**Files:**
- Create: `plugins/bmw_e9x_e8x/tools/dcc_study/extract.py`
- Test: `plugins/bmw_e9x_e8x/tools/dcc_study/tests/test_extract.py`

**Interfaces:**
- Consumes: `common.STALK_ADDR`, `common.decode_stalk`, `common.ROUTES_DIR`, `common.EXTRACTED_DIR`.
- Produces: `extract_segment(rlog_path: Path) -> dict[str, np.ndarray]` with keys (all float64 unless noted):
  `cs_t, vEgo, aEgo, setpoint, cruiseEnabled, gas, brake` (carState, aligned on `cs_t`; setpoint in m/s; last three 0/1),
  `ctrl_t, aTarget, ctrlEnabled` (carControl),
  `tx_t, tx_cmd` (sendcan 0x194 frames; `tx_cmd` int8: index into `CMDS` list, −1 for neutral),
  `rx_t, rx_cmd` (can-bus 0x194 frames **with an action bit set only** — candidate human presses and TX echoes),
  `pose_t, pitch` (livePose pitch, rad).
  Also `CMDS = ("plus1", "plus5", "minus1", "minus5", "cancel", "resume", "cancel_lever_up")` module constant (index ↔ code).
- CLI: `$PY extract.py [--routes DIR] [--out DIR]` walks `DIR/**/rlog.zst` (default `data/routes`), writes `data/extracted/<parent-dir-name>.npz` per segment, skips already-extracted, skips corrupt files with a warning.

- [ ] **Step 1: Write the failing test**

`tests/test_extract.py` builds a tiny synthetic rlog with cereal, exercising every channel:

```python
import numpy as np
import zstandard

from common import STALK_ADDR


def _stalk_frame(counter, byte2):
  return bytes([0x00, 0xF0 | counter, byte2, 0xFC])


def make_rlog(path):
  from cereal import log

  msgs = []

  def evt(t):
    e = log.Event.new_message()
    e.logMonoTime = int(t * 1e9)
    return e

  e = evt(1.0)
  e.init("carState")
  e.carState.vEgo = 20.0
  e.carState.aEgo = 0.125   # exactly representable in float32 (cereal stores f32)
  e.carState.cruiseState.speed = 22.0
  e.carState.cruiseState.enabled = True
  e.carState.gasPressed = False
  e.carState.brakePressed = True
  msgs.append(e)

  e = evt(1.01)
  e.init("carControl")
  e.carControl.enabled = True
  e.carControl.actuators.accel = 0.5
  msgs.append(e)

  e = evt(1.02)
  cans = e.init("sendcan", 2)
  cans[0].address = STALK_ADDR
  cans[0].dat = _stalk_frame(3, 0x01)   # plus1
  cans[1].address = 0x22E               # unrelated address -> ignored
  cans[1].dat = b"\x00" * 4
  msgs.append(e)

  e = evt(1.03)
  cans = e.init("can", 2)
  cans[0].address = STALK_ADDR
  cans[0].dat = _stalk_frame(4, 0x08)   # minus5 action frame -> recorded
  cans[1].address = STALK_ADDR
  cans[1].dat = _stalk_frame(5, 0x00)   # neutral idle frame -> NOT recorded
  msgs.append(e)

  e = evt(1.04)
  e.init("livePose")
  e.livePose.orientationNED.y = 0.02
  msgs.append(e)

  raw = b"".join(m.to_bytes() for m in msgs)
  path.write_bytes(zstandard.ZstdCompressor().compress(raw))


def test_extract_segment(tmp_path):
  from extract import extract_segment, CMDS

  rlog = tmp_path / "rlog.zst"
  make_rlog(rlog)
  seg = extract_segment(rlog)

  assert seg["cs_t"].shape == (1,)
  assert seg["vEgo"][0] == 20.0 and seg["aEgo"][0] == 0.125
  assert seg["setpoint"][0] == 22.0
  assert seg["brake"][0] == 1.0 and seg["gas"][0] == 0.0
  assert seg["aTarget"][0] == 0.5 and seg["ctrlEnabled"][0] == 1.0
  assert list(seg["tx_cmd"]) == [CMDS.index("plus1")]
  assert list(seg["rx_cmd"]) == [CMDS.index("minus5")]  # neutral rx dropped
  assert abs(seg["pitch"][0] - 0.02) < 1e-6  # tolerate float32 round-trip
  assert np.all(np.diff(seg["cs_t"]) >= 0)


def test_corrupt_rlog_returns_none(tmp_path):
  from extract import extract_segment

  bad = tmp_path / "rlog.zst"
  bad.write_bytes(b"not zstd at all")
  assert extract_segment(bad) is None
```

- [ ] **Step 2: Run test to verify it fails**

Run: `$PY -m pytest tests/test_extract.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'extract'`

- [ ] **Step 3: Write `extract.py`**

```python
"""rlog.zst -> per-segment .npz of the channels the DCC study needs."""
import argparse
import sys
from pathlib import Path

import numpy as np
import zstandard

from common import STALK_ADDR, ROUTES_DIR, EXTRACTED_DIR, decode_stalk

CMDS = ("plus1", "plus5", "minus1", "minus5", "cancel", "resume", "cancel_lever_up")

_KEYS = ("cs_t", "vEgo", "aEgo", "setpoint", "cruiseEnabled", "gas", "brake",
         "ctrl_t", "aTarget", "ctrlEnabled",
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
    if dest.exists():
      continue
    seg = extract_segment(rlog)
    if seg is not None:
      np.savez_compressed(dest, **seg)
      print(f"{rlog.parent.name}: {len(seg['cs_t'])} carState, "
            f"{len(seg['tx_t'])} stalk TX")


if __name__ == "__main__":
  main()
```

- [ ] **Step 4: Run test to verify it passes**

Run: `$PY -m pytest tests/test_extract.py -v`
Expected: 2 passed. If `read_multiple_bytes` rejects `traversal_limit_in_words`, drop to positional/`opts` per installed pycapnp version — check `python -c "import capnp; print(capnp.__version__)"` and adapt; the limit must stay ≥ default to survive large rlogs (see project memory: capnp traversal crash).

- [ ] **Step 5: Commit**

```bash
git add plugins/bmw_e9x_e8x/tools/dcc_study
git commit -m "dcc_study: rlog channel extraction"
```

---

### Task 3: Burst segmentation + contamination filters (`bursts.py` part 1)

**Files:**
- Create: `plugins/bmw_e9x_e8x/tools/dcc_study/bursts.py`
- Test: `plugins/bmw_e9x_e8x/tools/dcc_study/tests/test_bursts.py`

**Interfaces:**
- Consumes: `extract.CMDS`, `common.CMD_STEP`.
- Produces:
  ```python
  @dataclass
  class Burst:
    t_start: float
    t_end: float
    cmd: str            # plus1/plus5/minus1/minus5
    cadence: str        # "hold" (40 Hz) or "single" (20 Hz)
    n_frames: int
    # measurement fields filled by Task 4, NaN/0 until then:
    v_start: float = float("nan")
    setpoint_gap: float = float("nan")
    pitch_mean: float = float("nan")
    a_baseline: float = float("nan")
    peak_delta_a: float = float("nan")
    steady_delta_a: float = float("nan")
    rise_time: float = float("nan")
    ticks_accepted: int = 0
  ```
  `find_bursts(seg: dict) -> list[Burst]` — segmentation only;
  `is_contaminated(burst: Burst, seg: dict) -> bool`.
  Constants: `GAP_S = 0.5` (mirrors `BURST_LIVE_WINDOW`), `HOLD_MAX_INTERVAL = 0.035` (median inter-frame ≤ this → 40 Hz hold), `PAD_PRE = 0.5`, `PAD_POST = 1.5`, `HUMAN_MATCH_S = 0.05`, `HUMAN_PAD_S = 2.0`.

- [ ] **Step 1: Write the failing tests**

`tests/test_bursts.py` (segmentation/contamination half; a `seg()` helper builds minimal channel dicts):

```python
import numpy as np

from extract import CMDS
from bursts import find_bursts, is_contaminated, GAP_S

PLUS1 = CMDS.index("plus1")
PLUS5 = CMDS.index("plus5")


def seg(**over):
  base = {
    "cs_t": np.arange(0.0, 60.0, 0.01),
    "tx_t": np.array([]), "tx_cmd": np.array([], dtype=np.int8),
    "rx_t": np.array([]), "rx_cmd": np.array([], dtype=np.int8),
    "pose_t": np.array([]), "pitch": np.array([]),
  }
  n = len(base["cs_t"])
  base.update({"vEgo": np.full(n, 20.0), "aEgo": np.zeros(n),
               "setpoint": np.full(n, 22.0), "cruiseEnabled": np.ones(n),
               "gas": np.zeros(n), "brake": np.zeros(n)})
  base.update(over)
  return base


def tx(t0, n, interval, code):
  t = t0 + np.arange(n) * interval
  return t, np.full(n, code, dtype=np.int8)


def test_single_burst_hold_cadence():
  t, c = tx(10.0, 20, 0.025, PLUS5)          # 40 Hz
  bursts = find_bursts(seg(tx_t=t, tx_cmd=c))
  assert len(bursts) == 1
  b = bursts[0]
  assert (b.cmd, b.cadence, b.n_frames) == ("plus5", "hold", 20)
  assert b.t_start == 10.0


def test_single_cadence_classified():
  t, c = tx(10.0, 10, 0.05, PLUS1)           # 20 Hz
  assert find_bursts(seg(tx_t=t, tx_cmd=c))[0].cadence == "single"


def test_gap_splits_burst():
  t1, c1 = tx(10.0, 10, 0.05, PLUS1)
  t2, c2 = tx(t1[-1] + GAP_S + 0.1, 10, 0.05, PLUS1)
  bursts = find_bursts(seg(tx_t=np.concatenate([t1, t2]),
                           tx_cmd=np.concatenate([c1, c2])))
  assert len(bursts) == 2


def test_command_change_splits_burst():
  t1, c1 = tx(10.0, 10, 0.05, PLUS1)
  t2, c2 = tx(t1[-1] + 0.05, 10, 0.05, PLUS5)
  bursts = find_bursts(seg(tx_t=np.concatenate([t1, t2]),
                           tx_cmd=np.concatenate([c1, c2])))
  assert [b.cmd for b in bursts] == ["plus1", "plus5"]


def test_neutral_and_cancel_frames_ignored():
  t, c = tx(10.0, 10, 0.05, PLUS1)
  t_n = np.concatenate([t, t[-1] + np.arange(1, 6) * 0.05])
  c_n = np.concatenate([c, np.full(5, -1, dtype=np.int8)])   # trailing neutral
  bursts = find_bursts(seg(tx_t=t_n, tx_cmd=c_n))
  assert len(bursts) == 1 and bursts[0].n_frames == 10
  cancel = np.full(3, CMDS.index("cancel"), dtype=np.int8)
  assert find_bursts(seg(tx_t=t[:3], tx_cmd=cancel)) == []


def test_gas_contaminates():
  t, c = tx(10.0, 20, 0.025, PLUS1)
  s = seg(tx_t=t, tx_cmd=c)
  b = find_bursts(s)[0]
  assert not is_contaminated(b, s)
  s["gas"][(s["cs_t"] > 10.2) & (s["cs_t"] < 10.3)] = 1.0
  assert is_contaminated(b, s)


def test_brake_in_post_window_contaminates():
  t, c = tx(10.0, 20, 0.025, PLUS1)
  s = seg(tx_t=t, tx_cmd=c)
  b = find_bursts(s)[0]
  s["brake"][(s["cs_t"] > b.t_end + 1.0) & (s["cs_t"] < b.t_end + 1.2)] = 1.0
  assert is_contaminated(b, s)


def test_human_stalk_press_contaminates():
  t, c = tx(10.0, 20, 0.025, PLUS1)
  s = seg(tx_t=t, tx_cmd=c,
          rx_t=np.array([11.0]), rx_cmd=np.array([PLUS1], dtype=np.int8))
  # rx action frame 11.0 is >HUMAN_MATCH_S from every tx (last tx ~10.475)
  assert is_contaminated(find_bursts(s)[0], s)


def test_tx_echo_on_rx_is_not_human():
  t, c = tx(10.0, 20, 0.025, PLUS1)
  s = seg(tx_t=t, tx_cmd=c,
          rx_t=np.array([t[5] + 0.01]), rx_cmd=np.array([PLUS1], dtype=np.int8))
  assert not is_contaminated(find_bursts(s)[0], s)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `$PY -m pytest tests/test_bursts.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'bursts'`

- [ ] **Step 3: Write `bursts.py` (segmentation + contamination)**

```python
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
```

(Measurement, CSV output and `main()` are added in Task 4 — the module must import and the tests above must pass with just this.)

- [ ] **Step 4: Run tests to verify they pass**

Run: `$PY -m pytest tests/test_bursts.py -v`
Expected: 9 passed

- [ ] **Step 5: Commit**

```bash
git add plugins/bmw_e9x_e8x/tools/dcc_study
git commit -m "dcc_study: burst segmentation and contamination filters"
```

---

### Task 4: Response measurement + CSV pipeline (`bursts.py` part 2)

**Files:**
- Modify: `plugins/bmw_e9x_e8x/tools/dcc_study/bursts.py`
- Test: `plugins/bmw_e9x_e8x/tools/dcc_study/tests/test_measure.py`

**Interfaces:**
- Consumes: Task 3's `Burst`, `find_bursts`, `is_contaminated`; `common.CMD_STEP`, `common.PROFILES_DIR`, `common.DATA_DIR`.
- Produces: `measure(burst: Burst, seg: dict) -> Burst` (fills the NaN fields in place and returns it); `write_csv(rows: list[dict], path)`; CSV `data/bursts.csv` with header exactly:
  `route,segment,t_start,duration_s,cmd,cadence,n_frames,v_start_mps,setpoint_gap_mps,pitch_rad,a_baseline,peak_delta_a,steady_delta_a,rise_time_s,ticks_accepted`
  plus per-burst profile `data/profiles/<npz-stem>--<idx>.npz` holding `t` (relative to `t_start`) and `aEgo`.
- CLI: `$PY bursts.py [--extracted DIR]` reads every npz, prints kept/dropped counts, writes csv + profiles. Segment npz stems look like `<route>--<seg>` — split on the last `--` for the route/segment CSV columns.

- [ ] **Step 1: Write the failing tests**

`tests/test_measure.py`:
```python
import numpy as np

from extract import CMDS
from bursts import Burst, measure

PLUS5 = CMDS.index("plus5")


def step_seg(step_a=0.6, t0=10.0, dur=3.0, tau=0.4, sp_step_kph=15.0):
  """vEgo 20 m/s; aEgo first-order step of step_a at t0; setpoint ramps up."""
  cs_t = np.arange(0.0, 60.0, 0.01)
  aEgo = np.where(cs_t < t0, 0.0, step_a * (1 - np.exp(-(cs_t - t0) / tau)))
  setpoint = np.where(cs_t < t0, 22.0, 22.0 + sp_step_kph / 3.6)
  tx_t = t0 + np.arange(int(dur / 0.025)) * 0.025
  n = len(cs_t)
  return {
    "cs_t": cs_t, "vEgo": np.full(n, 20.0), "aEgo": aEgo,
    "setpoint": setpoint, "cruiseEnabled": np.ones(n),
    "gas": np.zeros(n), "brake": np.zeros(n),
    "tx_t": tx_t, "tx_cmd": np.full(len(tx_t), PLUS5, dtype=np.int8),
    "rx_t": np.array([]), "rx_cmd": np.array([], dtype=np.int8),
    "pose_t": cs_t[::10], "pitch": np.full(len(cs_t[::10]), 0.01),
  }


def _burst(s):
  from bursts import find_bursts
  return find_bursts(s)[0]


def test_steady_state_and_baseline():
  s = step_seg(step_a=0.6)
  b = measure(_burst(s), s)
  assert abs(b.a_baseline) < 0.01
  assert 0.5 < b.steady_delta_a < 0.65
  assert 0.55 < b.peak_delta_a < 0.65


def test_rise_time_near_tau():
  s = step_seg(step_a=0.6, tau=0.4)
  b = measure(_burst(s), s)
  assert 0.2 < b.rise_time < 0.6


def test_short_burst_has_no_steady_state():
  s = step_seg(dur=0.5)
  b = measure(_burst(s), s)
  assert np.isnan(b.steady_delta_a)
  assert b.peak_delta_a > 0.3          # peak still measured


def test_setpoint_gap_and_acceptance():
  s = step_seg(sp_step_kph=15.0)
  b = measure(_burst(s), s)
  assert abs(b.setpoint_gap - 2.0) < 0.1     # 22 - 20 m/s at burst start
  assert b.ticks_accepted == 3               # 15 kph / 5 kph per plus5 tick


def test_decel_peak_is_signed():
  s = step_seg(step_a=-0.8)
  s["tx_cmd"][:] = CMDS.index("minus5")
  b = measure(_burst(s), s)
  assert b.peak_delta_a < -0.6


def test_pitch_mean_recorded():
  s = step_seg()
  b = measure(_burst(s), s)
  assert abs(b.pitch_mean - 0.01) < 1e-6


def test_no_pose_gives_nan_pitch():
  s = step_seg()
  s["pose_t"] = np.array([]); s["pitch"] = np.array([])
  assert np.isnan(measure(_burst(s), s).pitch_mean)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `$PY -m pytest tests/test_measure.py -v`
Expected: FAIL — `ImportError: cannot import name 'measure'`

- [ ] **Step 3: Append measurement + pipeline to `bursts.py`**

```python
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
```

- [ ] **Step 4: Run ALL tests to verify they pass**

Run: `$PY -m pytest tests/ -v`
Expected: test_measure 7 passed; test_bursts/test_extract/test_stalk_decode still green.

- [ ] **Step 5: Commit**

```bash
git add plugins/bmw_e9x_e8x/tools/dcc_study
git commit -m "dcc_study: burst response measurement and bursts.csv pipeline"
```

---

### Task 5: COD route fetcher (`fetch_routes.py`)

**Files:**
- Create: `plugins/bmw_e9x_e8x/tools/dcc_study/fetch_routes.py`
- Test: `plugins/bmw_e9x_e8x/tools/dcc_study/tests/test_fetch.py`

**Interfaces:**
- Consumes: `common.ROUTES_DIR`.
- Produces: `route_url_name(fullname: str) -> str` (`"abc/2026-.."` → URL-quoted `"abc|2026-.."`); `extract_rlogs(tar_path, dest_dir) -> int` (pulls every `rlog.zst` member into `dest_dir/<member's parent dir name>/rlog.zst`, returns count); CLI `$PY fetch_routes.py [--host HOST] [--limit N] [--min-engagement PCT] [--route DATE ...]` — `--route` (repeatable, e.g. `--route 2026-02-20--10-47-46`) restricts to the named routes and bypasses the engagement filter (spec: "also usable with an explicit route list").
- COD endpoints used (see `connect-on-device/API.md`): `GET /v1/me/devices/` → dongle id; `GET /v1/devices/{id}/routes?limit=N`; `GET /v1/route/{name}/` → `engagement_pct`; `GET /v1/route/{name}/download?files=rlog` → tar.gz.

- [ ] **Step 1: Write the failing tests**

`tests/test_fetch.py` (pure parts only — no network):
```python
import io
import tarfile

from fetch_routes import route_url_name, extract_rlogs


def test_route_url_name_swaps_separator_and_quotes():
  assert route_url_name("abc123/2026-02-20--10-47-46") == \
      "abc123%7C2026-02-20--10-47-46"


def test_extract_rlogs_normalizes_layout(tmp_path):
  tar_path = tmp_path / "dl.tar.gz"
  with tarfile.open(tar_path, "w:gz") as tf:
    for name in ("2026-02-20--10-47-46--0/rlog.zst",
                 "2026-02-20--10-47-46--1/rlog.zst",
                 "2026-02-20--10-47-46--0/qlog.zst"):   # non-rlog ignored
      data = b"fake"
      info = tarfile.TarInfo(name)
      info.size = len(data)
      tf.addfile(info, io.BytesIO(data))
  dest = tmp_path / "routes"
  assert extract_rlogs(tar_path, dest) == 2
  assert (dest / "2026-02-20--10-47-46--0" / "rlog.zst").read_bytes() == b"fake"
  assert not (dest / "2026-02-20--10-47-46--0" / "qlog.zst").exists()
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `$PY -m pytest tests/test_fetch.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'fetch_routes'`

- [ ] **Step 3: Write `fetch_routes.py`**

```python
"""Pull rlogs of engaged routes from the C3 via the Connect-on-Device API."""
import argparse
import subprocess
import tarfile
import tempfile
import urllib.parse
from pathlib import Path

import requests

from common import ROUTES_DIR


def c3_host():
  out = subprocess.run(["ssh", "-G", "c3"], capture_output=True, text=True,
                       check=True).stdout
  for line in out.splitlines():
    if line.startswith("hostname "):
      return line.split()[1]
  raise RuntimeError("could not resolve host from `ssh -G c3`")


def route_url_name(fullname):
  return urllib.parse.quote(fullname.replace("/", "|"), safe="")


def extract_rlogs(tar_path, dest_dir):
  n = 0
  with tarfile.open(tar_path, "r:gz") as tf:
    for m in tf.getmembers():
      p = Path(m.name)
      if m.isfile() and p.name == "rlog.zst":
        seg_dir = Path(dest_dir) / p.parent.name
        seg_dir.mkdir(parents=True, exist_ok=True)
        (seg_dir / "rlog.zst").write_bytes(tf.extractfile(m).read())
        n += 1
  return n


def main():
  p = argparse.ArgumentParser()
  p.add_argument("--host", default=None, help="override `ssh -G c3` resolution")
  p.add_argument("--limit", type=int, default=100)
  p.add_argument("--min-engagement", type=int, default=1)
  p.add_argument("--route", action="append", default=None,
                 help="route date string, repeatable; bypasses engagement filter")
  args = p.parse_args()

  host = args.host or c3_host()
  base = f"http://{host}:8082"
  try:
    devices = requests.get(f"{base}/v1/me/devices/", timeout=10).json()
  except requests.ConnectionError as e:
    raise SystemExit(f"COD unreachable at {base} — is the C3 on and awake? "
                     f"(check `ssh c3`)\n{e}")
  dongle = devices[0]["dongle_id"]
  routes = requests.get(f"{base}/v1/devices/{dongle}/routes",
                        params={"limit": args.limit}, timeout=30).json()

  ROUTES_DIR.mkdir(parents=True, exist_ok=True)
  for r in routes:
    date = r["fullname"].split("/")[1]
    if args.route is not None and date not in args.route:
      continue
    name = route_url_name(r["fullname"])
    meta = requests.get(f"{base}/v1/route/{name}/", timeout=120).json()
    pct = meta.get("engagement_pct") or 0
    if args.route is None and pct < args.min_engagement:
      print(f"skip {date}: engagement {pct}%")
      continue
    n_seg = (meta.get("maxqlog") or 0) + 1
    if all((ROUTES_DIR / f"{date}--{s}" / "rlog.zst").exists() for s in range(n_seg)):
      print(f"have {date} ({n_seg} segments)")
      continue
    print(f"downloading {date} (engagement {pct}%, {n_seg} segments)...")
    with requests.get(f"{base}/v1/route/{name}/download",
                      params={"files": "rlog"}, stream=True, timeout=600) as resp:
      resp.raise_for_status()
      with tempfile.NamedTemporaryFile(suffix=".tar.gz") as tmp:
        for chunk in resp.iter_content(1 << 20):
          tmp.write(chunk)
        tmp.flush()
        print(f"  {extract_rlogs(tmp.name, ROUTES_DIR)} rlogs extracted")


if __name__ == "__main__":
  main()
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `$PY -m pytest tests/test_fetch.py -v`
Expected: 2 passed

- [ ] **Step 5: Commit**

```bash
git add plugins/bmw_e9x_e8x/tools/dcc_study
git commit -m "dcc_study: COD API route fetcher"
```

---

### Task 6: Report (`report.py`)

**Files:**
- Create: `plugins/bmw_e9x_e8x/tools/dcc_study/report.py`
- Test: `plugins/bmw_e9x_e8x/tools/dcc_study/tests/test_report.py`

**Interfaces:**
- Consumes: `data/bursts.csv` (Task 4's `CSV_FIELDS` header), `common.REPORT_DIR`, `common.DATA_DIR`.
- Produces in `data/report/`:
  - `response_vs_speed.png` — 2×2 grid (one panel per cmd), x = v_start km/h, y = Δaccel (steady where available else peak), scatter + 10 km/h-bin medians, one color per cadence.
  - `residual_vs_setpoint_gap.png`, `residual_vs_pitch.png` — residual = Δaccel − that burst's (cmd, cadence, speed-bin) median.
  - `summary.txt` — coverage table (burst counts per cmd×cadence×10 km/h bin), acceptance-rate table (ticks_accepted / expected ticks per cadence, where expected single-tick ≈ n_frames·frame-interval·cadence rate — report observed ratio only), median response per cmd×cadence.
- `load_bursts(csv_path) -> list[dict]` (typed values); `speed_bin(v_mps) -> int` (10 km/h bins); pitch filter: primary medians use rows with `abs(pitch_rad) < 0.017` (~1°); rows with NaN pitch excluded from pitch-filtered fits only (per spec).

- [ ] **Step 1: Write the failing smoke test**

`tests/test_report.py`:
```python
import csv
import math
import random

from bursts import CSV_FIELDS


def fake_rows(n=120):
  random.seed(0)
  rows = []
  for i in range(n):
    cmd = random.choice(["plus1", "plus5", "minus1", "minus5"])
    cadence = random.choice(["hold", "single"])
    v = random.uniform(8, 35)
    a = (0.4 if "plus" in cmd else -0.6) * (1.5 if "5" in cmd else 1.0)
    rows.append({"route": "r", "segment": i % 8, "t_start": i * 10.0,
                 "duration_s": 2.0, "cmd": cmd, "cadence": cadence,
                 "n_frames": 40, "v_start_mps": round(v, 2),
                 "setpoint_gap_mps": 1.0, "pitch_rad": 0.001,
                 "a_baseline": 0.0, "peak_delta_a": round(a * 1.1, 3),
                 "steady_delta_a": round(a, 3) if i % 3 else "nan",
                 "rise_time_s": 0.5, "ticks_accepted": 4})
  return rows


def test_report_generates_outputs(tmp_path):
  import report

  csv_path = tmp_path / "bursts.csv"
  with open(csv_path, "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
    w.writeheader()
    w.writerows(fake_rows())

  out = tmp_path / "report"
  report.generate(csv_path, out)

  assert (out / "response_vs_speed.png").exists()
  assert (out / "residual_vs_setpoint_gap.png").exists()
  assert (out / "residual_vs_pitch.png").exists()
  text = (out / "summary.txt").read_text()
  assert "coverage" in text.lower()
  assert "plus5" in text


def test_load_bursts_types(tmp_path):
  import report

  csv_path = tmp_path / "bursts.csv"
  with open(csv_path, "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
    w.writeheader()
    w.writerows(fake_rows(5))
  rows = report.load_bursts(csv_path)
  assert isinstance(rows[0]["v_start_mps"], float)
  assert isinstance(rows[0]["n_frames"], int)
  nan_rows = [r for r in rows if isinstance(r["steady_delta_a"], float)
              and math.isnan(r["steady_delta_a"])]
  assert nan_rows  # "nan" strings parsed to float nan
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `$PY -m pytest tests/test_report.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'report'`

- [ ] **Step 3: Write `report.py`**

```python
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
```

- [ ] **Step 4: Run ALL tests to verify they pass**

Run: `$PY -m pytest tests/ -v`
Expected: all green (≈22 tests).

- [ ] **Step 5: Commit**

```bash
git add plugins/bmw_e9x_e8x/tools/dcc_study
git commit -m "dcc_study: phase-gate report generation"
```

---

### Task 7: End-to-end run on real routes + findings doc

**Requires the C3 online (`ssh c3` reachable). This task is operator-assisted — if the device is unreachable, stop and report; do not fake data.**

**Files:**
- Create: `docs/superpowers/specs/2026-07-29-dcc-response-findings.md` (findings, committed)
- Uses: all four scripts.

- [ ] **Step 1: Fetch routes**

Run: `$PY fetch_routes.py` (from `dcc_study/`).
Expected: engaged routes downloaded under `data/routes/<date>--<seg>/rlog.zst`; skips print for low-engagement routes. If the device response shapes differ from `API.md` (e.g. `dongle_id` key name), adapt `fetch_routes.py` minimally and note it in the commit.

- [ ] **Step 2: Extract + segment + report**

Run:
```bash
$PY extract.py
$PY bursts.py
$PY report.py
```
Expected: non-zero clean bursts; `data/report/` contains 3 pngs + summary.txt. Sanity checks before trusting output: hold-cadence bursts should show larger |Δaccel| than single at the same speed; plus5 > plus1; acceptance ticks > 0 on most bursts (validates counter-overwrite). If any sanity check fails, investigate (systematic-debugging) before writing findings.

- [ ] **Step 3: Write findings doc**

`docs/superpowers/specs/2026-07-29-dcc-response-findings.md` — actual numbers only (no template values): coverage table verdict per spec gate criteria (enough bursts per cmd×cadence across ≥3 speed bins?), speed-dependence verdict (are binned medians monotone/flat?), setpoint-gap saturation observation, pitch-filter adequacy, acceptance-rate result, and a recommendation: approach B / approach A / targeted calibration drives. Attach the summary.txt content inline.

- [ ] **Step 4: Commit findings**

```bash
git add docs/superpowers/specs/2026-07-29-dcc-response-findings.md
git commit -m "dcc_study: phase-1 findings from on-device routes"
```

- [ ] **Step 5: Phase gate**

STOP. Present findings + plots to the user for the joint B-vs-A decision. Phase 2 is a separate plan written only after that decision.
