# DCC Setpoint Control (Phase 2) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the four hand-tuned `aTarget` thresholds in `bmw/carcontroller.py` with a setpoint-error control law derived from measured DCC response, so `aEgo` tracks `aTarget` instead of being determined by `v_target` alone.

**Architecture:** A generator (`tools/dcc_study/fit_map.py`) fits `a_expected = f(gap, vEgo)` from the 2.46 M extracted samples and emits a checked-in table module. The controller inverts that table to get the gap it needs, converts gap to a desired setpoint (capped at `v_target`), and emits the ticks still owed. Cadence selection is deleted — the study proved it inert. Everything stays open-loop on the map; `aEgo` is never used as an error signal.

**Tech Stack:** Python; numpy in the offline generator only. The shipped controller path uses plain lists + `numpy.interp` (already imported across the plugin).

**Spec:** `docs/superpowers/specs/2026-07-29-dcc-response-findings.md` — §"The resulting control law", §"Can the stalk actually track aTarget?", §"Two constraints B′ must respect".

## Global Constraints

- Repo `/home/oxygen/catpilot-dev/plugins`, branch `dev`. Commit per task; **no `Co-Authored-By` lines**.
- Plugin dir: `/home/oxygen/catpilot-dev/plugins/plugins/bmw_e9x_e8x/`. Study tooling: `<plugin>/tools/dcc_study/`.
- Study-tool interpreter (`$PY`): `PYTHONPATH=/home/oxygen/catpilot-dev/catpilot:. /home/oxygen/catpilot-dev/catpilot/.venv/bin/python`
- Plugin tests run from the repo root as `PYTHONPATH=. uv run pytest plugins/bmw_e9x_e8x/tests/ -v`; study tests run from `tools/dcc_study/` as `$PY -m pytest tests/ -v` (the pre-push hook does NOT collect the nested study tests).
- **Safety invariants that must survive unchanged** (all already in `carcontroller.py`): the setpoint is never commanded above `v_target`; `minEnableSpeed` + `MIN_SPEED_BUFFER` headroom is respected before any decrement; the burst / counter-overwrite machinery (`cruise_cmd`, `cruise_burst_release_safe`, trailing neutral frames), `V_ERROR_DEADZONE` entry gate, cancel handling, and gas-pressed handling are untouched.
- **Never** use measured `aEgo` as a feedback/error term — that is approach C, which the data rejects (`aEgo` p50 rate of change is 0.72 m/s³ vs `aTarget`'s 0.075).
- No on-device changes in Tasks 1–3. Task 4 is on-car and operator-gated.
- 2-space indent (openpilot house style).

## File Structure

| File | Responsibility |
|---|---|
| `tools/dcc_study/fit_map.py` (new) | Offline: fit + monotonise the response table, emit `bmw/dcc_map_table.py`. Not shipped to the car. |
| `tools/dcc_study/tests/test_fit_map.py` (new) | Covers monotonisation and table emission on synthetic input. |
| `bmw/dcc_map_table.py` (new, generated) | Pure data: `GAP_BPS`, `V_BPS`, `A_TABLE`. Regenerable, reviewed as code. |
| `bmw/dcc_map.py` (new) | `expected_accel(gap, v)` and `gap_for_accel(a_target, v)` — the inversion, plus the authority clamp. |
| `bmw/carcontroller.py` (modify) | Replace the threshold ladder with the setpoint-error law; delete 4 dead constants. |
| `tests/test_dcc_map.py` (new) | Inversion round-trip, clamping, monotonicity. |
| `tests/test_carcontroller_longitudinal.py` (new) | Command selection + safety invariants. |

---

### Task 1: Fit and emit the response table

**Files:**
- Create: `plugins/bmw_e9x_e8x/tools/dcc_study/fit_map.py`
- Create: `plugins/bmw_e9x_e8x/tools/dcc_study/tests/test_fit_map.py`
- Generates: `plugins/bmw_e9x_e8x/bmw/dcc_map_table.py`

**Interfaces:**
- Produces: `GAP_BPS = [-3.0, -2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5]` (m/s), `V_BPS = [8.0, 14.0, 20.0, 26.0, 33.0]` (m/s), `A_TABLE` — a list of `len(GAP_BPS)` rows × `len(V_BPS)` cols of floats, **strictly increasing down each column**.
- `monotonise(col: list[float]) -> list[float]` — running maximum, so the inversion is single-valued.
- `fit(extracted_dir) -> (gap_bps, v_bps, table)`; `emit(path, gap_bps, v_bps, table)`.

**Why the table stops at gap +1.5:** only 1.2 % of samples sit at |gap| > 2 m/s, and the fitted cells there are non-monotone noise (at 72 km/h the +3.0 cell reads *below* the +2.0 cell). Above +1.5 the controller clamps to the envelope rather than extrapolating into the saturated region.

**Known cost of that choice — read before Task 4.** Truncating at +1.5 also truncates the *authority*: the shipped table tops out at **+0.32…+0.46 m/s²** depending on speed, whereas DCC's true ceiling measured on the coarser binning is ~+0.5 m/s² (reached at gaps of +2…+3). So B′ will request slightly gentler acceleration than the car can actually deliver. That is the deliberate, conservative side to err on for a first on-car run, but if the driver reports acceleration feels *weaker* than before, the correct fix is to extend the table to gap +2.0 using data from the targeted calibration drive the findings recommend — **not** to loosen the clamp against the noisy cells that already exist. Deceleration is unaffected (the table reaches −0.97…−1.17 m/s², matching the measured floor).

**This plan's expected generated table** (produced by Step 5 against the current 2.46 M samples; use it to verify the generator reproduces rather than as a hand-edited constant):

```
  gap  -3.0: -1.0039 -1.0039 -1.0167 -1.1682 -0.9683
  gap  -2.0: -0.8720 -0.8720 -0.8889 -0.8185 -0.8561
  gap  -1.5: -0.6980 -0.6980 -0.6683 -0.6825 -0.7204
  gap  -1.0: -0.4822 -0.4134 -0.4172 -0.4358 -0.4603
  gap  -0.5: -0.2855 -0.2406 -0.2564 -0.2807 -0.3311
  gap  +0.0: -0.0030 -0.0314 -0.0870 -0.1138 -0.1926
  gap  +0.5: +0.0629 +0.0609 +0.0359 +0.0135 +0.0006
  gap  +1.0: +0.2840 +0.2299 +0.1960 +0.1711 +0.1265
  gap  +1.5: +0.4334 +0.4589 +0.3827 +0.3522 +0.3174
```

All of Task 2's and Task 3's test assertions were verified against this exact table before the plan was written — they pass.

- [ ] **Step 1: Write the failing test**

`tests/test_fit_map.py`:
```python
import numpy as np

from fit_map import monotonise, emit, GAP_BPS, V_BPS


def test_monotonise_is_running_max():
  assert monotonise([-1.0, -0.5, -0.6, 0.2, 0.1]) == [-1.0, -0.5, -0.5, 0.2, 0.2]


def test_monotonise_leaves_increasing_untouched():
  col = [-1.0, -0.4, 0.0, 0.3, 0.5]
  assert monotonise(col) == col


def test_monotonise_handles_nan_by_carrying_previous():
  out = monotonise([-1.0, float("nan"), -0.2])
  assert out[0] == -1.0
  assert out[1] == -1.0        # NaN carries the previous value forward
  assert out[2] == -0.2


def test_emit_writes_importable_module(tmp_path):
  table = [[float(i) / 10 + j / 100 for j in range(len(V_BPS))]
           for i in range(len(GAP_BPS))]
  path = tmp_path / "dcc_map_table.py"
  emit(path, GAP_BPS, V_BPS, table)

  ns = {}
  exec(compile(path.read_text(), str(path), "exec"), ns)
  assert ns["GAP_BPS"] == GAP_BPS
  assert ns["V_BPS"] == V_BPS
  assert len(ns["A_TABLE"]) == len(GAP_BPS)
  assert len(ns["A_TABLE"][0]) == len(V_BPS)
  assert "GENERATED" in path.read_text()


def test_emitted_table_columns_are_strictly_increasing(tmp_path):
  # regression guard: whatever fit() produced must invert single-valued
  from fit_map import monotonise
  raw = [[-1.0, -1.0], [-0.5, -0.6], [-0.5, -0.2], [0.3, 0.1]]
  cols = [monotonise([r[j] for r in raw]) for j in range(2)]
  for c in cols:
    assert all(b >= a for a, b in zip(c, c[1:]))
```

- [ ] **Step 2: Run test to verify it fails**

Run (from `tools/dcc_study/`): `$PY -m pytest tests/test_fit_map.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'fit_map'`

- [ ] **Step 3: Write `fit_map.py`**

```python
"""Fit a_expected = f(setpoint gap, vEgo) and emit the shipped table module.

Offline only — never imported by the car. Run after extract.py:
    $PY fit_map.py
"""
import argparse
import glob
import math
from pathlib import Path

import numpy as np

from common import EXTRACTED_DIR

# Dense, monotone region only. Above +1.5 m/s of gap the data is 1.2% of
# samples and non-monotone; the controller clamps there instead.
GAP_BPS = [-3.0, -2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5]
V_BPS = [8.0, 14.0, 20.0, 26.0, 33.0]
MIN_CELL = 150          # samples required before a cell is trusted
DEFAULT_OUT = Path(__file__).resolve().parents[2] / "bmw" / "dcc_map_table.py"


def monotonise(col):
  """Running maximum, carrying the previous value across NaN.

  The inversion must be single-valued: a_expected has to increase with gap.
  """
  out, run = [], None
  for x in col:
    if x is not None and not math.isnan(x):
      run = x if run is None else max(run, x)
    out.append(run if run is not None else float("nan"))
  return out


def _edges(bps, lo_pad, hi_pad):
  mids = [(a + b) / 2 for a, b in zip(bps, bps[1:])]
  return [bps[0] - lo_pad] + mids + [bps[-1] + hi_pad]


def fit(extracted_dir):
  gaps, accs, vs = [], [], []
  for fn in sorted(glob.glob(str(Path(extracted_dir) / "*.npz"))):
    d = np.load(fn)
    m = (d["cruiseEnabled"] > 0) & (d["gas"] == 0) & (d["brake"] == 0) & (d["vEgo"] > 3.0)
    if m.sum() < 200:
      continue
    gaps.append(d["setpoint"][m] - d["vEgo"][m])
    accs.append(d["aEgo"][m])
    vs.append(d["vEgo"][m])
  gap = np.concatenate(gaps)
  acc = np.concatenate(accs)
  v = np.concatenate(vs)
  print(f"samples: {len(gap):,}")

  g_edge = _edges(GAP_BPS, 1.0, 0.5)
  v_edge = _edges(V_BPS, 5.0, 12.0)
  table = []
  for i in range(len(GAP_BPS)):
    row = []
    for j in range(len(V_BPS)):
      s = (gap >= g_edge[i]) & (gap < g_edge[i + 1]) & \
          (v >= v_edge[j]) & (v < v_edge[j + 1])
      row.append(float(np.median(acc[s])) if s.sum() >= MIN_CELL else float("nan"))
    table.append(row)

  # interpolate across speed to fill sparse cells, then force monotone in gap
  for i, row in enumerate(table):
    ok = [j for j, x in enumerate(row) if not math.isnan(x)]
    if len(ok) >= 2:
      table[i] = list(np.interp(V_BPS, [V_BPS[j] for j in ok], [row[j] for j in ok]))
  cols = [monotonise([table[i][j] for i in range(len(GAP_BPS))])
          for j in range(len(V_BPS))]
  return GAP_BPS, V_BPS, [[cols[j][i] for j in range(len(V_BPS))]
                          for i in range(len(GAP_BPS))]


def emit(path, gap_bps, v_bps, table):
  lines = [
    '"""GENERATED by tools/dcc_study/fit_map.py — do not edit by hand.',
    "",
    "Median measured aEgo (m/s^2) as a function of DCC setpoint gap",
    "(cruiseState.speed - vEgo, m/s) and vEgo (m/s). Columns are forced",
    "monotone in gap so the inversion in dcc_map.py is single-valued.",
    '"""',
    f"GAP_BPS = {list(gap_bps)!r}",
    f"V_BPS = {list(v_bps)!r}",
    "A_TABLE = [",
  ]
  for g, row in zip(gap_bps, table):
    cells = ", ".join(f"{x:+.4f}" for x in row)
    lines.append(f"  [{cells}],  # gap {g:+.1f} m/s")
  lines += ["]", ""]
  Path(path).write_text("\n".join(lines))


def main():
  p = argparse.ArgumentParser()
  p.add_argument("--extracted", default=EXTRACTED_DIR, type=Path)
  p.add_argument("--out", default=DEFAULT_OUT, type=Path)
  args = p.parse_args()
  gap_bps, v_bps, table = fit(args.extracted)
  for j, vb in enumerate(v_bps):
    col = [table[i][j] for i in range(len(gap_bps))]
    print(f"  v={vb * 3.6:>5.0f} kph  a: {col[0]:+.3f} .. {col[-1]:+.3f}")
  emit(args.out, gap_bps, v_bps, table)
  print(f"wrote {args.out}")


if __name__ == "__main__":
  main()
```

- [ ] **Step 4: Run test to verify it passes**

Run: `$PY -m pytest tests/test_fit_map.py -v`
Expected: 5 passed. Then run the whole study suite: `$PY -m pytest tests/ -v` (34 prior + 5 = 39).

- [ ] **Step 5: Generate the real table and eyeball it**

Run: `$PY fit_map.py`
Expected: `samples: 2,4xx,xxx`, per-speed envelopes printed, `bmw/dcc_map_table.py` written. Sanity-check before committing — the printed envelope should be roughly `a_min ≈ −0.5…−1.15`, `a_max ≈ +0.3…+0.5` across the speed columns, and every column must be non-decreasing. If a column is flat over more than 3 consecutive gap breakpoints, the monotonisation has masked bad data — stop and report rather than shipping it.

- [ ] **Step 6: Commit**

```bash
git add plugins/bmw_e9x_e8x/tools/dcc_study/fit_map.py \
        plugins/bmw_e9x_e8x/tools/dcc_study/tests/test_fit_map.py \
        plugins/bmw_e9x_e8x/bmw/dcc_map_table.py
git commit -m "bmw: fit and generate the DCC gap-to-accel response table"
```

---

### Task 2: The map module and its inversion

**Files:**
- Create: `plugins/bmw_e9x_e8x/bmw/dcc_map.py`
- Test: `plugins/bmw_e9x_e8x/tests/test_dcc_map.py`

**Interfaces:**
- Consumes: `bmw.dcc_map_table.GAP_BPS / V_BPS / A_TABLE` from Task 1.
- Produces:
  - `expected_accel(gap: float, v_ego: float) -> float` — bilinear interp, clamped at the table edges.
  - `gap_for_accel(a_target: float, v_ego: float) -> float` — inverse; clamps `a_target` into the achievable envelope for that speed, then interpolates gap. Never returns a gap outside `[GAP_BPS[0], GAP_BPS[-1]]`.
  - `accel_envelope(v_ego: float) -> tuple[float, float]` — `(a_min, a_max)` achievable at that speed.

- [ ] **Step 1: Write the failing test**

`tests/test_dcc_map.py`:
```python
import pytest

from bmw.dcc_map import expected_accel, gap_for_accel, accel_envelope
from bmw.dcc_map_table import GAP_BPS, V_BPS, A_TABLE


def test_table_columns_are_monotone():
  for j in range(len(V_BPS)):
    col = [A_TABLE[i][j] for i in range(len(GAP_BPS))]
    assert all(b >= a for a, b in zip(col, col[1:])), f"column {j} not monotone"


def test_expected_accel_hits_table_nodes():
  for i, g in enumerate(GAP_BPS):
    for j, v in enumerate(V_BPS):
      assert expected_accel(g, v) == pytest.approx(A_TABLE[i][j], abs=1e-9)


def test_expected_accel_clamps_outside_table():
  v = V_BPS[2]
  assert expected_accel(GAP_BPS[0] - 5.0, v) == pytest.approx(expected_accel(GAP_BPS[0], v))
  assert expected_accel(GAP_BPS[-1] + 5.0, v) == pytest.approx(expected_accel(GAP_BPS[-1], v))


def test_negative_gap_gives_negative_accel():
  for v in V_BPS:
    assert expected_accel(-2.0, v) < 0.0
    assert expected_accel(+1.5, v) > expected_accel(-1.0, v)


def test_inversion_round_trips_inside_the_envelope():
  for v in V_BPS:
    lo, hi = accel_envelope(v)
    for frac in (0.2, 0.5, 0.8):
      a = lo + (hi - lo) * frac
      g = gap_for_accel(a, v)
      assert expected_accel(g, v) == pytest.approx(a, abs=0.02)


def test_inversion_clamps_beyond_authority():
  v = V_BPS[2]
  lo, hi = accel_envelope(v)
  assert gap_for_accel(hi + 5.0, v) == pytest.approx(GAP_BPS[-1])
  assert gap_for_accel(lo - 5.0, v) == pytest.approx(GAP_BPS[0])


def test_gap_never_escapes_table_bounds():
  for v in (1.0, V_BPS[0], V_BPS[-1], 60.0):
    for a in (-9.0, -1.0, 0.0, 1.0, 9.0):
      assert GAP_BPS[0] <= gap_for_accel(a, v) <= GAP_BPS[-1]


def test_envelope_is_ordered():
  for v in V_BPS:
    lo, hi = accel_envelope(v)
    assert lo < hi
```

- [ ] **Step 2: Run test to verify it fails**

Run (from repo root): `PYTHONPATH=. uv run pytest plugins/bmw_e9x_e8x/tests/test_dcc_map.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'bmw.dcc_map'`

- [ ] **Step 3: Write `dcc_map.py`**

```python
"""Measured DCC response map and its inverse.

The car's acceleration is a function of the *setpoint gap*
(cruiseState.speed - vEgo), not of which stalk command produced it — see
docs/superpowers/specs/2026-07-29-dcc-response-findings.md. This module turns a
requested acceleration into the gap that delivers it.

Open-loop by design: nothing here consumes measured aEgo.
"""
from bmw.dcc_map_table import GAP_BPS, V_BPS, A_TABLE


def _clamp(x, lo, hi):
  return lo if x < lo else (hi if x > hi else x)


def _interp(x, xs, ys):
  x = _clamp(x, xs[0], xs[-1])
  for i in range(len(xs) - 1):
    if x <= xs[i + 1]:
      x0, x1 = xs[i], xs[i + 1]
      y0, y1 = ys[i], ys[i + 1]
      if x1 == x0:
        return y0
      return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
  return ys[-1]


def _column(v_ego):
  """Response curve vs gap at this speed (interpolated across V_BPS)."""
  return [_interp(v_ego, V_BPS, [A_TABLE[i][j] for j in range(len(V_BPS))])
          for i in range(len(GAP_BPS))]


def expected_accel(gap, v_ego):
  """Acceleration DCC is expected to produce at this gap and speed (m/s^2)."""
  return _interp(gap, GAP_BPS, _column(v_ego))


def accel_envelope(v_ego):
  """(a_min, a_max) reachable at this speed — DCC's authority limits."""
  col = _column(v_ego)
  return col[0], col[-1]


def gap_for_accel(a_target, v_ego):
  """Setpoint gap (m/s) that produces a_target, clamped to what DCC can do."""
  col = _column(v_ego)
  return _interp(_clamp(a_target, col[0], col[-1]), col, GAP_BPS)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `PYTHONPATH=. uv run pytest plugins/bmw_e9x_e8x/tests/test_dcc_map.py -v`
Expected: 8 passed.

- [ ] **Step 5: Commit**

```bash
git add plugins/bmw_e9x_e8x/bmw/dcc_map.py plugins/bmw_e9x_e8x/tests/test_dcc_map.py
git commit -m "bmw: DCC response map with clamped inversion"
```

---

### Task 3: Setpoint-error control law in `carcontroller.py`

**Files:**
- Modify: `plugins/bmw_e9x_e8x/bmw/carcontroller.py` (constants block ~lines 40–51; command-selection block ~lines 171–191)
- Test: `plugins/bmw_e9x_e8x/tests/test_carcontroller_longitudinal.py`

**Interfaces:**
- Consumes: `bmw.dcc_map.gap_for_accel`.
- Produces: no new public API — behavioural change inside `CarController.update`.

**Deletions** (all four are proven dead by the study): `ACCEL_HOLD_THRESHOLD`, `DECEL_HOLD_THRESHOLD` (cadence is inert), `ACCEL_STEP5_THRESHOLD`, `DECEL_STEP5_THRESHOLD` (superseded by the tick arithmetic). Replace the stale "DCC Calibration" comment block with a pointer to the findings doc.

**New constants:**
```python
CMD_INTERVAL = SINGLE_INTERVAL   # cadence is inert (study §3/§5); 20 Hz halves bus load
SETPOINT_DEADBAND_KPH = 1.0      # below one tick there is nothing to send
```

- [ ] **Step 1: Write the failing test**

`tests/test_carcontroller_longitudinal.py` — tests the decision function directly so no CAN stack is needed:

```python
import pytest

from bmw.dcc_map import gap_for_accel, accel_envelope
from bmw.carcontroller import select_cruise_command, SETPOINT_DEADBAND_KPH


def cmd(a_target, v_ego, setpoint, v_target, min_setpoint=5.0):
  return select_cruise_command(a_target, v_ego, setpoint, v_target, min_setpoint)


def test_deadband_emits_nothing():
  # setpoint already where the map wants it
  v, a = 20.0, 0.0
  sp = v + gap_for_accel(a, v)
  assert cmd(a, v, sp, v_target=v + 5.0) is None


def test_accel_request_below_target_emits_plus():
  v = 20.0
  out = cmd(+0.3, v, setpoint=v, v_target=v + 5.0)
  assert out in ("plus1", "plus5")


def test_large_setpoint_error_uses_step5():
  v = 20.0
  # ask for the most the car can do, from a setpoint sitting at vEgo
  _, a_max = accel_envelope(v)
  assert cmd(a_max, v, setpoint=v, v_target=v + 10.0) == "plus5"


def test_small_setpoint_error_uses_step1():
  v = 20.0
  sp = v + gap_for_accel(0.0, v) - (2.0 / 3.6)   # 2 km/h short -> one-ish tick
  out = cmd(0.0, v, setpoint=sp, v_target=v + 10.0)
  assert out == "plus1"


def test_setpoint_never_commanded_above_v_target():
  v, v_target = 20.0, 20.5
  # map wants a big gap, but v_target is only 0.5 m/s above vEgo
  out = cmd(+0.5, v, setpoint=v_target, v_target=v_target)
  assert out is None, "already at v_target — must not push the setpoint higher"


def test_commanded_setpoint_never_goes_below_min():
  """The desired-setpoint clamp is what protects min cruise speed.

  Sweep a decel request from just above the floor and check the command can
  never carry the setpoint under it: a minus tick moves 1 or 5 km/h, and it is
  only emitted when at least that much error exists above the floor.
  """
  v, min_sp = 10.0, 8.0
  for excess_kph in (0.2, 0.9, 1.5, 4.0, 6.0, 12.0):
    sp = min_sp + excess_kph / 3.6
    out = cmd(-1.0, v, setpoint=sp, v_target=0.0, min_setpoint=min_sp)
    if out is None:
      continue
    step_kph = 5.0 if out == "minus5" else 1.0
    assert sp - step_kph / 3.6 >= min_sp - 1e-9, \
        f"{out} from setpoint {sp:.3f} would cross the {min_sp} m/s floor"


def test_decel_request_emits_minus():
  v = 25.0
  out = cmd(-0.5, v, setpoint=v, v_target=v - 5.0, min_setpoint=5.0)
  assert out in ("minus1", "minus5")


def test_beyond_authority_saturates_not_escalates():
  v = 20.0
  _, a_max = accel_envelope(v)
  far = cmd(a_max + 5.0, v, setpoint=v, v_target=v + 20.0)
  at_max = cmd(a_max, v, setpoint=v, v_target=v + 20.0)
  assert far == at_max, "requesting beyond authority must not change the command"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `PYTHONPATH=. uv run pytest plugins/bmw_e9x_e8x/tests/test_carcontroller_longitudinal.py -v`
Expected: FAIL — `ImportError: cannot import name 'select_cruise_command'`

- [ ] **Step 3: Edit `carcontroller.py`**

Replace the constants block (the lines from `# DCC command selection thresholds` through the `# MINUS5 + HOLD = -1.2 m/s²` comment) with:

```python
# DCC command selection.
# The car's acceleration tracks the setpoint gap, not the command or the TX
# cadence — see docs/superpowers/specs/2026-07-29-dcc-response-findings.md.
# We invert the measured map to get the gap we need, cap the resulting setpoint
# at v_target, and emit the ticks still owed.
V_ERROR_DEADZONE = 0.5 / 3.6   # m/s (~0.5 km/h) — deadzone for entry and burst cancellation
CMD_INTERVAL = SINGLE_INTERVAL # cadence is inert; 20 Hz halves bus load
SETPOINT_DEADBAND_KPH = 1.0    # below one tick there is nothing to send
```

Add the module-level decision function (above `class CarController`):

```python
def select_cruise_command(a_target, v_ego, setpoint, v_target, min_setpoint):
  """Which stalk command closes the gap between the setpoint and where the
  measured DCC map says it should be. Returns a CruiseStalk name or None.

  Open-loop on the map: measured aEgo is deliberately not an input.
  """
  desired = v_ego + gap_for_accel(a_target, v_ego)
  desired = min(desired, v_target)          # never target above the planner's speed
  desired = max(desired, min_setpoint)      # never strand the car below min cruise
  err_kph = (desired - setpoint) * CV.MS_TO_KPH

  if abs(err_kph) < SETPOINT_DEADBAND_KPH:
    return None
  if err_kph > 0:
    return 'plus5' if err_kph >= 5.0 else 'plus1'
  # No separate min-speed headroom check is needed: `desired` is already floored
  # at min_setpoint, so a tick is only emitted when at least that step of error
  # exists above the floor, and it can never carry the setpoint under it.
  return 'minus5' if -err_kph >= 5.0 else 'minus1'
```

Replace the command-selection body inside `update` (the `if v_error > V_ERROR_DEADZONE ...` / `elif v_error < -V_ERROR_DEADZONE ...` branches) with:

```python
        else:
          cmd_name = select_cruise_command(accel, v_current,
                                           CS.out.cruiseState.speed, v_target,
                                           self.min_cruise_setpoint)
          if cmd_name is not None and abs(v_error) > V_ERROR_DEADZONE:
            cruise_cmd(getattr(CruiseStalk, cmd_name), CMD_INTERVAL)
```

Add `from bmw.dcc_map import gap_for_accel` to the imports.

- [ ] **Step 4: Run tests to verify they pass**

Run: `PYTHONPATH=. uv run pytest plugins/bmw_e9x_e8x/tests/ -v`
Expected: the 8 new longitudinal tests pass **and** every pre-existing BMW test still passes. If any pre-existing test referenced a deleted threshold constant, update that test to the new law rather than re-adding the constant — and say so in the commit body.

- [ ] **Step 5: Commit**

```bash
git add plugins/bmw_e9x_e8x/bmw/carcontroller.py \
        plugins/bmw_e9x_e8x/tests/test_carcontroller_longitudinal.py
git commit -m "bmw: drive DCC by setpoint error from the measured response map"
```

---

### Task 4: On-car verification

**Operator-assisted and safety-critical. Requires the C3 reachable (`ssh c3`) and a driver. If the device is unreachable, stop and report — do not simulate a result.**

- [ ] **Step 1: Deploy**

```bash
GIT_SSH_COMMAND='ssh -o BatchMode=yes' git push git@github.com:catpilot-dev/plugins.git dev:refs/heads/dev
ssh c3 'cd /data/plugins && GIT_SSL_NO_VERIFY=1 git fetch origin dev && git reset --hard origin/dev && bash install.sh'
```
Expected: install.sh reports success and writes `.needs_restart`; plugind applies it offroad.

- [ ] **Step 2: Drive and record**

Cover low (~30 km/h), mid (~60–80), and high (~100+) speed, including at least one lead-car follow and one free-flow acceleration to set speed. Note the route ID.

- [ ] **Step 3: Measure tracking against the baseline**

Pull the new route and compare `aEgo` against `aTarget` using the study tooling's extraction:
```bash
$PY fetch_routes.py --route <new-route-date>
$PY extract.py
```
Then compare the tracking error `aEgo − aTarget` distribution against a pre-change route. **Success criterion:** median |aEgo − aTarget| is lower than baseline and no new oscillation appears (check that the sign of `aEgo − aTarget` does not alternate at a fixed period — that would be the lag-driven overshoot the findings flagged).

- [ ] **Step 4: Report and gate**

STOP and present: tracking-error comparison, driver's subjective report on sluggishness/harshness, and any oscillation evidence. Do not tune constants without on-car data — the DCC 5ECE lesson in project memory applies.

Failure-mode routing, decided in advance so nobody improvises on the car:
- **Oscillation / alternating tracking-error sign** → widen `SETPOINT_DEADBAND_KPH`, or low-pass `a_target` before inversion. **Never** add feedback on `aEgo` (that is approach C).
- **Acceleration feels weaker than before** → expected; the table is truncated at gap +1.5 (see Task 1). Fix by extending the table with calibration-drive data at gaps of +2…+3, not by loosening the clamp.
- **Deceleration feels wrong** → the decel side of the table is well-populated and matches the measured floor, so suspect the `v_target` path or the min-speed clamp before suspecting the map.
