# Driver-Side Lane Keeping Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a standalone `lane_keeping` plugin that anchors the car's driver-side wheel-to-line gap in a [0.6, 1.0] m deadband via a bounded pure-pursuit curvature bias, robust to modelV2 sub-Hz curvature noise, coexisting with the existing DRIFT_M controller as fallback (Phase 1).

**Architecture:** A pure-Python control core (`anchor.py`, no cereal/opendbc deps) holds all state and math; a thin `register.py` loads config from the plugin `data/` dir, runs the core on the `controls.curvature_correction` hook, and publishes telemetry. Two states: ANCHOR (confident driver-side line → add bias) and MODEL (else → passthrough).

**Tech Stack:** Python 3.11, plugin framework (`controls.curvature_correction` hook), `PluginPub` plugin-bus telemetry, pytest (`PYTHONPATH=. uv run pytest`).

## Global Constraints

- Plugin params live in the plugin's `data/` dir, NEVER `/data/params/d/` (clearAll wipes unknown keys).
- Core control module `anchor.py` must have NO cereal/opendbc/zmq imports (pure `math` only) so it is unit-testable standalone.
- Curvature sign convention (openpilot): positive curvature = left turn (+y = left in device frame).
- Control loop / hook rate: 100 Hz, `DT_CTRL = 0.01` s.
- `modelV2.laneLines[1]` = left ego line, `[2]` = right ego line; `.y[0]` = lateral offset at car; `laneLineProbs[idx]` = confidence.
- MODEL state must return the input `curvature` bit-identical (no bias residual once released).
- Commit after every task. No `Co-Authored-By` lines.
- Branch: `lane_keeping` (off `dev` @ af3be5f).

---

### Task 1: Plugin scaffolding + passthrough hook

**Files:**
- Create: `plugins/lane_keeping/plugin.json`
- Create: `plugins/lane_keeping/register.py`
- Create: `plugins/lane_keeping/data/.gitkeep`
- Create: `plugins/lane_keeping/README.md`
- Test: `plugins/lane_keeping/tests/test_register.py`

**Interfaces:**
- Produces: `register.on_curvature_correction(curvature, model_v2, v_ego, lane_changing, lat_delay=None) -> float` (Task 1: pure passthrough; wired to the anchor in Task 7).

- [ ] **Step 1: Write the failing test**

`plugins/lane_keeping/tests/test_register.py`:
```python
import os, sys
from types import SimpleNamespace
_PLUGIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PLUGIN_DIR not in sys.path:
  sys.path.insert(0, _PLUGIN_DIR)


def test_passthrough_returns_input_curvature():
  import register
  mv = SimpleNamespace(laneLines=[], laneLineProbs=[])
  out = register.on_curvature_correction(0.0123, mv, 25.0, False, lat_delay=0.45)
  assert out == 0.0123
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd plugins && PYTHONPATH=. uv run pytest lane_keeping/tests/test_register.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'register'`

- [ ] **Step 3: Write minimal implementation**

`plugins/lane_keeping/register.py`:
```python
"""Driver-side lane keeping — hook entry, config loading, telemetry.

Registers on controls.curvature_correction. Phase 1: coexists with the
existing DRIFT_M controller; MODEL state is a literal passthrough.
"""
import os
import sys

_PLUGIN_DIR = os.path.dirname(os.path.abspath(__file__))
if _PLUGIN_DIR not in sys.path:
  sys.path.insert(0, _PLUGIN_DIR)


def on_curvature_correction(curvature, model_v2, v_ego, lane_changing, lat_delay=None):
  # Task 7 replaces this passthrough with the anchor + telemetry.
  return curvature
```

`plugins/lane_keeping/plugin.json`:
```json
{
  "id": "lane_keeping",
  "name": "Driver-Side Lane Keeping",
  "version": "1.0.0",
  "type": "hook",
  "author": "catpilot",
  "description": "Position outer-loop anchoring the driver-side wheel-to-line gap in a comfort band via a bounded pure-pursuit curvature bias. Robust to modelV2 sub-Hz curvature noise.",
  "min_openpilot": "0.10.0",
  "max_openpilot": "0.11.99",
  "conflicts": [],
  "dependencies": [],
  "params": {
    "LaneKeepEnable": {
      "default": true,
      "description": "Enable driver-side lane keeping (position anchor)"
    },
    "LaneKeepDriverSide": {
      "default": "left",
      "description": "Which ego lane line to anchor to: left or right"
    }
  },
  "hooks": {
    "controls.curvature_correction": {
      "module": "register",
      "function": "on_curvature_correction",
      "priority": 50
    }
  }
}
```

`plugins/lane_keeping/README.md`:
```markdown
# Driver-Side Lane Keeping

Standalone plugin on `controls.curvature_correction`. Anchors the car's
driver-side wheel-to-line gap in a `[GAP_MIN, GAP_MAX]` deadband via a bounded
pure-pursuit curvature bias. When no confident driver-side line is present it
is a literal passthrough (the existing controller runs unchanged).

Design spec: `docs/superpowers/specs/2026-07-22-driver-side-lane-keeping-design.md`.
Control core: `anchor.py` (pure, unit-tested). Hook + telemetry: `register.py`.
Params: files in `data/` (see `_load_config` in `register.py`).
```

Create the data dir marker: `plugins/lane_keeping/data/.gitkeep` (empty file).

- [ ] **Step 4: Run test to verify it passes**

Run: `cd plugins && PYTHONPATH=. uv run pytest lane_keeping/tests/test_register.py -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
cd plugins
git add lane_keeping/
git commit -m "lane_keeping: plugin scaffolding + passthrough hook"
```

---

### Task 2: AnchorConfig + signal extraction

**Files:**
- Create: `plugins/lane_keeping/anchor.py`
- Test: `plugins/lane_keeping/tests/test_anchor.py`

**Interfaces:**
- Produces:
  - `anchor.DT_CTRL = 0.01`
  - `anchor._clip(x, lo, hi) -> float`, `anchor._interp(x, [x0,x1], [f0,f1]) -> float`
  - `anchor.AnchorConfig` dataclass with fields: `enable: bool=True`, `driver_side: str='left'`, `half_width: float=0.91`, `gap_min: float=0.6`, `gap_max: float=1.0`, `t_preview: float=1.5`, `excess_max: float=0.5`, `kappa_bias_max: float=0.002`, `kappa_rate_max: float=0.002`, `filter_tau: float=0.7`, `prob_on: float=0.6`, `prob_fade: float=0.1`
  - `anchor.LaneAnchor(config)` with `.driver_idx` (1 left / 2 right), `.side_sign` (+1 / −1), `.gap_filt=None`, `.kappa_bias=0.0`, `.state='model'`, and `._gap(model_v2) -> float` returning `side_sign*laneLines[idx].y[0] - half_width`.

- [ ] **Step 1: Write the failing test**

Append to `plugins/lane_keeping/tests/test_anchor.py`:
```python
import os, sys
from types import SimpleNamespace
_PLUGIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PLUGIN_DIR not in sys.path:
  sys.path.insert(0, _PLUGIN_DIR)
from anchor import AnchorConfig, LaneAnchor


def _mv(left_y, right_y, left_p=1.0, right_p=1.0):
  return SimpleNamespace(
    laneLines=[SimpleNamespace(y=[0.0]), SimpleNamespace(y=[left_y]),
               SimpleNamespace(y=[right_y]), SimpleNamespace(y=[0.0])],
    laneLineProbs=[0.0, left_p, right_p, 0.0])


def test_gap_left_driver():
  a = LaneAnchor(AnchorConfig(driver_side='left', half_width=0.91))
  assert a.driver_idx == 1 and a.side_sign == 1.0
  # left line 1.75 m to the left (+y) -> wheel gap = 1.75 - 0.91 = 0.84
  assert abs(a._gap(_mv(left_y=1.75, right_y=-1.75)) - 0.84) < 1e-9


def test_gap_right_driver():
  a = LaneAnchor(AnchorConfig(driver_side='right', half_width=0.91))
  assert a.driver_idx == 2 and a.side_sign == -1.0
  # right line 1.75 m to the right (-y) -> wheel gap = 1.75 - 0.91 = 0.84
  assert abs(a._gap(_mv(left_y=1.75, right_y=-1.75)) - 0.84) < 1e-9
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd plugins && PYTHONPATH=. uv run pytest lane_keeping/tests/test_anchor.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'anchor'`

- [ ] **Step 3: Write minimal implementation**

`plugins/lane_keeping/anchor.py`:
```python
"""Driver-side lane keeping — pure control core (no cereal/zmq imports).

Anchors the driver-side wheel-to-line gap in a [gap_min, gap_max] deadband
via a bounded pure-pursuit curvature bias. See design spec
docs/superpowers/specs/2026-07-22-driver-side-lane-keeping-design.md.
"""
import math
from dataclasses import dataclass

DT_CTRL = 0.01  # openpilot control-loop period (s); the hook runs at 100 Hz


def _clip(x, lo, hi):
  return lo if x < lo else hi if x > hi else x


def _interp(x, xp, fp):
  # two-point clamped linear interpolation (xp ascending, len 2)
  if x <= xp[0]:
    return fp[0]
  if x >= xp[1]:
    return fp[1]
  t = (x - xp[0]) / (xp[1] - xp[0])
  return fp[0] + t * (fp[1] - fp[0])


@dataclass
class AnchorConfig:
  enable: bool = True
  driver_side: str = 'left'      # 'left' or 'right'
  half_width: float = 0.91       # car half-width (m); E90 ~1.817 m
  gap_min: float = 0.6           # driver-wheel-to-line comfort band (m)
  gap_max: float = 1.0
  t_preview: float = 1.5         # pure-pursuit look-ahead time (s)
  excess_max: float = 0.5        # max deadband excess acted on (m)
  kappa_bias_max: float = 0.002  # hard cap on curvature bias (1/m)
  kappa_rate_max: float = 0.002  # bias slew (1/m per second)
  filter_tau: float = 0.7        # gap low-pass time constant (s)
  prob_on: float = 0.6           # driver-side line confidence to engage
  prob_fade: float = 0.1         # fade width above prob_on


class LaneAnchor:
  def __init__(self, config: AnchorConfig):
    self.cfg = config
    self.driver_idx = 1 if config.driver_side == 'left' else 2
    self.side_sign = 1.0 if config.driver_side == 'left' else -1.0
    self.gap_filt = None
    self.kappa_bias = 0.0
    self.state = 'model'

  def _gap(self, model_v2):
    line_y = float(model_v2.laneLines[self.driver_idx].y[0])
    return self.side_sign * line_y - self.cfg.half_width
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd plugins && PYTHONPATH=. uv run pytest lane_keeping/tests/test_anchor.py -v`
Expected: PASS (2 tests)

- [ ] **Step 5: Commit**

```bash
cd plugins
git add lane_keeping/anchor.py lane_keeping/tests/test_anchor.py
git commit -m "lane_keeping: AnchorConfig + driver-side gap extraction"
```

---

### Task 3: Deadband + glitch clip

**Files:**
- Modify: `plugins/lane_keeping/anchor.py`
- Test: `plugins/lane_keeping/tests/test_anchor.py`

**Interfaces:**
- Produces: `LaneAnchor._excess(gap_filt) -> float` = `clip(gap_filt - clip(gap_filt, gap_min, gap_max), -excess_max, excess_max)`. Zero inside the band; signed by which side of the band; clipped to ±`excess_max`.

- [ ] **Step 1: Write the failing test**

Append to `plugins/lane_keeping/tests/test_anchor.py`:
```python
def test_excess_zero_in_band():
  a = LaneAnchor(AnchorConfig(gap_min=0.6, gap_max=1.0))
  assert a._excess(0.6) == 0.0
  assert a._excess(0.8) == 0.0
  assert a._excess(1.0) == 0.0


def test_excess_signs_and_clip():
  a = LaneAnchor(AnchorConfig(gap_min=0.6, gap_max=1.0, excess_max=0.5))
  # gap below band (too close to driver line) -> negative excess
  assert abs(a._excess(0.4) - (-0.2)) < 1e-9
  # gap above band (too far from driver line) -> positive excess
  assert abs(a._excess(1.3) - 0.3) < 1e-9
  # glitch far above band -> clipped to +excess_max
  assert a._excess(9.0) == 0.5
  # glitch far below band -> clipped to -excess_max
  assert a._excess(-9.0) == -0.5
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd plugins && PYTHONPATH=. uv run pytest lane_keeping/tests/test_anchor.py -k excess -v`
Expected: FAIL with `AttributeError: 'LaneAnchor' object has no attribute '_excess'`

- [ ] **Step 3: Write minimal implementation**

Add to `LaneAnchor` in `plugins/lane_keeping/anchor.py`:
```python
  def _excess(self, gap_filt):
    cfg = self.cfg
    excess = gap_filt - _clip(gap_filt, cfg.gap_min, cfg.gap_max)
    return _clip(excess, -cfg.excess_max, cfg.excess_max)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd plugins && PYTHONPATH=. uv run pytest lane_keeping/tests/test_anchor.py -k excess -v`
Expected: PASS (2 tests)

- [ ] **Step 5: Commit**

```bash
cd plugins
git add lane_keeping/anchor.py lane_keeping/tests/test_anchor.py
git commit -m "lane_keeping: deadband excess + glitch clip"
```

---

### Task 4: Pure-pursuit bias + hard cap

**Files:**
- Modify: `plugins/lane_keeping/anchor.py`
- Test: `plugins/lane_keeping/tests/test_anchor.py`

**Interfaces:**
- Produces: `LaneAnchor._pursuit(excess, v_ego) -> float` = `clip(side_sign * 2 * excess / (max(v_ego*t_preview, 1.0)**2), -kappa_bias_max, kappa_bias_max)`. Look-ahead floored at 1 m to avoid div-by-zero at standstill.

- [ ] **Step 1: Write the failing test**

Append to `plugins/lane_keeping/tests/test_anchor.py`:
```python
def test_pursuit_magnitude_and_sign_left():
  a = LaneAnchor(AnchorConfig(driver_side='left', t_preview=1.5, kappa_bias_max=1.0))
  # excess +0.3 (car too far from left line) at 25 m/s:
  # Lp = 25*1.5 = 37.5; kappa = 2*0.3/37.5^2 = 0.000426..., positive (steer left)
  k = a._pursuit(0.3, 25.0)
  assert abs(k - (2 * 0.3 / 37.5 ** 2)) < 1e-9
  assert k > 0
  # excess -0.3 (car too close to left line) -> steer right (negative)
  assert a._pursuit(-0.3, 25.0) < 0


def test_pursuit_sign_right_driver():
  a = LaneAnchor(AnchorConfig(driver_side='right', t_preview=1.5, kappa_bias_max=1.0))
  # right driver, excess +0.3 (car too far from right line = too far left) -> steer right (negative)
  assert a._pursuit(0.3, 25.0) < 0


def test_pursuit_hard_cap():
  a = LaneAnchor(AnchorConfig(driver_side='left', t_preview=1.5, kappa_bias_max=0.002))
  # low speed inflates kappa; cap binds
  assert abs(a._pursuit(0.5, 5.0)) == 0.002
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd plugins && PYTHONPATH=. uv run pytest lane_keeping/tests/test_anchor.py -k pursuit -v`
Expected: FAIL with `AttributeError: ... '_pursuit'`

- [ ] **Step 3: Write minimal implementation**

Add to `LaneAnchor` in `plugins/lane_keeping/anchor.py`:
```python
  def _pursuit(self, excess, v_ego):
    cfg = self.cfg
    lp = max(v_ego * cfg.t_preview, 1.0)   # look-ahead floor avoids div0 at standstill
    kappa = self.side_sign * 2.0 * excess / (lp * lp)
    return _clip(kappa, -cfg.kappa_bias_max, cfg.kappa_bias_max)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd plugins && PYTHONPATH=. uv run pytest lane_keeping/tests/test_anchor.py -k pursuit -v`
Expected: PASS (3 tests)

- [ ] **Step 5: Commit**

```bash
cd plugins
git add lane_keeping/anchor.py lane_keeping/tests/test_anchor.py
git commit -m "lane_keeping: pure-pursuit curvature bias + hard cap"
```

---

### Task 5: Full update — filter, authority, rate limit, passthrough

**Files:**
- Modify: `plugins/lane_keeping/anchor.py`
- Test: `plugins/lane_keeping/tests/test_anchor.py`

**Interfaces:**
- Produces: `LaneAnchor.update(curvature, model_v2, v_ego, lane_changing) -> (new_curvature: float, telem: dict)`. Telemetry dict keys: `prob, line_y, gap, gap_filt, excess, kappa_bias, authority, state, v_ego`. Applies: availability guard → gap low-pass (`alpha = 1 - exp(-DT_CTRL/filter_tau)`) → deadband → pursuit → authority fade (`_interp(prob, [prob_on, prob_on+prob_fade], [0,1])`, ×0 if `lane_changing`) → single rate-limit path (`max_step = kappa_rate_max * DT_CTRL`) that also smoothly releases the bias to 0 in MODEL state. `state='anchor'` iff available and `authority>0`, else `'model'`. On unavailable line, resets `gap_filt=None`.

- [ ] **Step 1: Write the failing test**

Append to `plugins/lane_keeping/tests/test_anchor.py`:
```python
def _settle(a, mv, v=25.0, lane_changing=False, n=2000):
  out = None
  for _ in range(n):
    out, _t = a.update(0.01, mv, v, lane_changing)
  return out


def test_update_passthrough_when_no_line():
  a = LaneAnchor(AnchorConfig())
  mv = SimpleNamespace(laneLines=[], laneLineProbs=[])
  out, telem = a.update(0.0123, mv, 25.0, False)
  assert out == 0.0123              # bit-identical passthrough
  assert telem['state'] == 'model'


def test_update_passthrough_when_low_prob():
  a = LaneAnchor(AnchorConfig(prob_on=0.6))
  mv = _mv(left_y=1.75, right_y=-1.75, left_p=0.4)   # below prob_on
  out, telem = a.update(0.0123, mv, 25.0, False)
  assert out == 0.0123
  assert telem['state'] == 'model'


def test_update_no_bias_in_band():
  a = LaneAnchor(AnchorConfig())
  # left line 1.75 -> gap 0.84, inside [0.6,1.0]
  out = _settle(a, _mv(left_y=1.75, right_y=-1.75))
  assert abs(out - 0.01) < 1e-6     # curvature unchanged (bias ~0)


def test_update_biases_left_when_too_far_from_left_line():
  a = LaneAnchor(AnchorConfig())
  # left line 2.3 -> gap 1.39, above band -> steer left (positive bias)
  out = _settle(a, _mv(left_y=2.3, right_y=-1.2))
  assert out > 0.01 + 1e-5


def test_update_biases_right_when_too_close_to_left_line():
  a = LaneAnchor(AnchorConfig())
  # left line 1.3 -> gap 0.39, below band -> steer right (negative bias)
  out = _settle(a, _mv(left_y=1.3, right_y=-2.2))
  assert out < 0.01 - 1e-5


def test_update_disabled_during_lane_change():
  a = LaneAnchor(AnchorConfig())
  out = _settle(a, _mv(left_y=2.3, right_y=-1.2), lane_changing=True)
  assert abs(out - 0.01) < 1e-6     # authority 0 -> passthrough


def test_update_rate_limited():
  a = LaneAnchor(AnchorConfig(kappa_rate_max=0.002))
  mv = _mv(left_y=2.3, right_y=-1.2)
  # first engaged tick can move at most kappa_rate_max*DT_CTRL from 0
  a.gap_filt = a._gap(mv)           # warm the filter so excess is immediate
  out, telem = a.update(0.01, mv, 25.0, False)
  assert abs(telem['kappa_bias']) <= 0.002 * 0.01 + 1e-12
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd plugins && PYTHONPATH=. uv run pytest lane_keeping/tests/test_anchor.py -k update -v`
Expected: FAIL with `AttributeError: ... 'update'`

- [ ] **Step 3: Write minimal implementation**

Add to `LaneAnchor` in `plugins/lane_keeping/anchor.py`:
```python
  def _telem(self, prob, line_y, gap, excess, authority, v_ego):
    return {
      'prob': float(prob), 'line_y': float(line_y), 'gap': float(gap),
      'gap_filt': float(self.gap_filt) if self.gap_filt is not None else 0.0,
      'excess': float(excess), 'kappa_bias': float(self.kappa_bias),
      'authority': float(authority), 'state': self.state, 'v_ego': float(v_ego),
    }

  def update(self, curvature, model_v2, v_ego, lane_changing):
    cfg = self.cfg
    prob = line_y = gap = excess = authority = 0.0
    available = (cfg.enable and model_v2 is not None
                 and len(model_v2.laneLineProbs) > self.driver_idx
                 and len(model_v2.laneLines) > self.driver_idx
                 and len(model_v2.laneLines[self.driver_idx].y) > 0)
    if available:
      prob = float(model_v2.laneLineProbs[self.driver_idx])
      line_y = float(model_v2.laneLines[self.driver_idx].y[0])
      gap = self.side_sign * line_y - cfg.half_width
      if self.gap_filt is None:
        self.gap_filt = gap
      else:
        alpha = 1.0 - math.exp(-DT_CTRL / cfg.filter_tau)
        self.gap_filt += alpha * (gap - self.gap_filt)
      excess = self._excess(self.gap_filt)
      authority = _interp(prob, [cfg.prob_on, cfg.prob_on + cfg.prob_fade], [0.0, 1.0])
      if lane_changing:
        authority = 0.0
      kappa_target = authority * self._pursuit(excess, v_ego)
    else:
      self.gap_filt = None
      kappa_target = 0.0
    # single rate-limit path (also smoothly releases bias to 0 in MODEL state)
    max_step = cfg.kappa_rate_max * DT_CTRL
    self.kappa_bias = _clip(kappa_target, self.kappa_bias - max_step, self.kappa_bias + max_step)
    self.state = 'anchor' if (available and authority > 0.0) else 'model'
    return curvature + self.kappa_bias, self._telem(prob, line_y, gap, excess, authority, v_ego)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd plugins && PYTHONPATH=. uv run pytest lane_keeping/tests/test_anchor.py -v`
Expected: PASS (all tests, ~13)

- [ ] **Step 5: Commit**

```bash
cd plugins
git add lane_keeping/anchor.py lane_keeping/tests/test_anchor.py
git commit -m "lane_keeping: full update loop — filter, authority fade, rate limit, passthrough"
```

---

### Task 6: Config loading from data dir

**Files:**
- Modify: `plugins/lane_keeping/register.py`
- Test: `plugins/lane_keeping/tests/test_register.py`

**Interfaces:**
- Produces: `register._read_param(key, default) -> str`, `register._load_config() -> AnchorConfig`. Reads `data/<Key>` files; missing/empty → the `AnchorConfig` default. Bool keys: `LaneKeepEnable`. String: `LaneKeepDriverSide`. Float: `LaneKeepHalfWidth`, `LaneKeepGapMin`, `LaneKeepGapMax`, `LaneKeepTPreview`, `LaneKeepExcessMax`, `LaneKeepKappaBiasMax`, `LaneKeepKappaRateMax`, `LaneKeepFilterTau`, `LaneKeepProbOn`, `LaneKeepProbFade`.

- [ ] **Step 1: Write the failing test**

Append to `plugins/lane_keeping/tests/test_register.py`:
```python
import pytest


@pytest.fixture
def data_dir(tmp_path, monkeypatch):
  import register
  d = tmp_path / 'data'
  d.mkdir()
  monkeypatch.setattr(register, '_PLUGIN_DIR', str(tmp_path))
  return d


def test_load_config_defaults(data_dir):
  import register
  cfg = register._load_config()
  assert cfg.enable is True
  assert cfg.driver_side == 'left'
  assert cfg.gap_min == 0.6 and cfg.gap_max == 1.0
  assert cfg.t_preview == 1.5


def test_load_config_overrides(data_dir):
  import register
  (data_dir / 'LaneKeepDriverSide').write_text('right')
  (data_dir / 'LaneKeepGapMin').write_text('0.5')
  (data_dir / 'LaneKeepEnable').write_text('0')
  cfg = register._load_config()
  assert cfg.driver_side == 'right'
  assert cfg.gap_min == 0.5
  assert cfg.enable is False
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd plugins && PYTHONPATH=. uv run pytest lane_keeping/tests/test_register.py -k config -v`
Expected: FAIL with `AttributeError: module 'register' has no attribute '_load_config'`

- [ ] **Step 3: Write minimal implementation**

Add to `plugins/lane_keeping/register.py` (above `on_curvature_correction`):
```python
def _read_param(key, default=''):
  try:
    with open(os.path.join(_PLUGIN_DIR, 'data', key)) as f:
      return f.read().strip()
  except (FileNotFoundError, OSError):
    return default


def _load_config():
  from anchor import AnchorConfig
  d = AnchorConfig()

  def fget(key, dflt):
    v = _read_param(key)
    return float(v) if v else dflt

  def sget(key, dflt):
    v = _read_param(key)
    return v if v else dflt

  def bget(key, dflt):
    v = _read_param(key)
    return dflt if v == '' else v not in ('0', 'false', 'False')

  return AnchorConfig(
    enable=bget('LaneKeepEnable', d.enable),
    driver_side=sget('LaneKeepDriverSide', d.driver_side),
    half_width=fget('LaneKeepHalfWidth', d.half_width),
    gap_min=fget('LaneKeepGapMin', d.gap_min),
    gap_max=fget('LaneKeepGapMax', d.gap_max),
    t_preview=fget('LaneKeepTPreview', d.t_preview),
    excess_max=fget('LaneKeepExcessMax', d.excess_max),
    kappa_bias_max=fget('LaneKeepKappaBiasMax', d.kappa_bias_max),
    kappa_rate_max=fget('LaneKeepKappaRateMax', d.kappa_rate_max),
    filter_tau=fget('LaneKeepFilterTau', d.filter_tau),
    prob_on=fget('LaneKeepProbOn', d.prob_on),
    prob_fade=fget('LaneKeepProbFade', d.prob_fade),
  )
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd plugins && PYTHONPATH=. uv run pytest lane_keeping/tests/test_register.py -v`
Expected: PASS (passthrough + 2 config tests)

- [ ] **Step 5: Commit**

```bash
cd plugins
git add lane_keeping/register.py lane_keeping/tests/test_register.py
git commit -m "lane_keeping: config loading from data dir"
```

---

### Task 7: Wire hook to anchor + telemetry

**Files:**
- Modify: `plugins/lane_keeping/register.py`
- Test: `plugins/lane_keeping/tests/test_register.py`

**Interfaces:**
- Consumes: `LaneAnchor.update`, `_load_config` (Tasks 5–6).
- Produces: `on_curvature_correction` now lazily builds a module-level `LaneAnchor` from config, runs it, best-effort publishes telemetry via `PluginPub('lane_keeping')`, returns the biased curvature. Telemetry failures never break the control path.

- [ ] **Step 1: Write the failing test**

Append to `plugins/lane_keeping/tests/test_register.py`:
```python
def test_hook_applies_bias_and_survives_pub_failure(data_dir, monkeypatch):
  import importlib, register
  importlib.reload(register)
  monkeypatch.setattr(register, '_PLUGIN_DIR', str(data_dir.parent))
  # force telemetry publish to raise — control path must still return a value
  monkeypatch.setattr(register, '_publish', lambda telem: (_ for _ in ()).throw(RuntimeError('no bus')))
  register._anchor = None
  mv = SimpleNamespace(
    laneLines=[SimpleNamespace(y=[0.0]), SimpleNamespace(y=[2.3]),
               SimpleNamespace(y=[-1.2]), SimpleNamespace(y=[0.0])],
    laneLineProbs=[0.0, 1.0, 1.0, 0.0])
  out = None
  for _ in range(2000):
    out = register.on_curvature_correction(0.01, mv, 25.0, False, lat_delay=0.45)
  assert out > 0.01           # biased left (too far from left line), pub error swallowed


def test_hook_passthrough_when_disabled(data_dir, monkeypatch):
  import importlib, register
  importlib.reload(register)
  monkeypatch.setattr(register, '_PLUGIN_DIR', str(data_dir.parent))
  (data_dir / 'LaneKeepEnable').write_text('0')
  register._anchor = None
  mv = SimpleNamespace(laneLines=[], laneLineProbs=[])
  assert register.on_curvature_correction(0.0123, mv, 25.0, False) == 0.0123
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd plugins && PYTHONPATH=. uv run pytest lane_keeping/tests/test_register.py -k hook -v`
Expected: FAIL (bias not applied — hook is still a passthrough; `_publish` missing)

- [ ] **Step 3: Write minimal implementation**

Replace the body of `on_curvature_correction` and add helpers in `plugins/lane_keeping/register.py`:
```python
_anchor = None
_pub = None


def _publish(telem):
  global _pub
  if _pub is None:
    from openpilot.selfdrive.plugins.plugin_bus import PluginPub
    _pub = PluginPub('lane_keeping')
  _pub.send(telem)


def on_curvature_correction(curvature, model_v2, v_ego, lane_changing, lat_delay=None):
  global _anchor
  if _anchor is None:
    from anchor import LaneAnchor
    _anchor = LaneAnchor(_load_config())
  if not _anchor.cfg.enable:
    return curvature
  new_curvature, telem = _anchor.update(curvature, model_v2, v_ego, lane_changing)
  try:
    _publish(telem)
  except Exception:
    pass  # telemetry must never break the control path
  return new_curvature
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd plugins && PYTHONPATH=. uv run pytest lane_keeping/tests/ -v`
Expected: PASS (all register + anchor tests)

- [ ] **Step 5: Commit**

```bash
cd plugins
git add lane_keeping/register.py lane_keeping/tests/test_register.py
git commit -m "lane_keeping: wire hook to anchor + best-effort telemetry"
```

---

### Task 8: Offline replay validation + T_PREVIEW sweep

**Files:**
- Create: `plugins/lane_keeping/tests/replay_anchor.py`

**Interfaces:**
- Consumes: `anchor.AnchorConfig`, `anchor.LaneAnchor`.
- Produces: a standalone script (NOT pytest-collected — no `test_` prefix) run on the C3 that replays the anchor over recorded rlogs and reports, per route: ANCHOR-vs-MODEL occupancy, in-band vs biasing fraction, and a `T_PREVIEW` sweep of resulting `kappa_bias` magnitude. Validates the detector open-loop and informs the `T_PREVIEW` choice before on-car (per spec §4.2, §7).

- [ ] **Step 1: Write the replay script**

`plugins/lane_keeping/tests/replay_anchor.py`:
```python
#!/usr/bin/env python3
"""Offline replay of the lane-keeping anchor over recorded rlogs (run on C3).

Feeds recorded modelV2 lane lines through LaneAnchor and reports ANCHOR/MODEL
occupancy, in-band vs biasing fraction, and a T_PREVIEW sweep of the resulting
curvature-bias magnitude. Open-loop detector validation + tuning aid (the
closed-loop centering cannot be measured offline).

Usage (on C3):
  source /usr/local/venv/bin/activate
  PYTHONPATH=/data/openpilot:/data/plugins-runtime/lane_keeping \
    python replay_anchor.py 000003b7--977baff7b6 000003bb--12fe0dc6be
"""
import sys, glob
import zstandard
from cereal import log as capnp_log
from anchor import AnchorConfig, LaneAnchor

BASE = '/data/media/0/realdata'


def load_model(route, seg):
  raw = zstandard.ZstdDecompressor().decompress(
    open(f'{BASE}/{route}--{seg}/rlog.zst', 'rb').read(), max_output_size=2**31)
  frames = []
  v = 0.0
  for evt in capnp_log.Event.read_multiple_bytes(raw):
    w = evt.which()
    if w == 'carState':
      v = float(evt.carState.vEgo)
    elif w == 'modelV2':
      m = evt.modelV2
      lane_changing = str(m.meta.laneChangeState) != 'off'
      frames.append((m, v, lane_changing))
  return frames


def run_route(route, t_preview):
  segs = sorted(int(p.rsplit('--', 1)[1]) for p in glob.glob(f'{BASE}/{route}--*'))
  a = LaneAnchor(AnchorConfig(t_preview=t_preview))
  n = anchor_n = biasing = 0
  max_bias = 0.0
  for seg in segs:
    try:
      frames = load_model(route, seg)
    except (FileNotFoundError, zstandard.ZstdError):
      continue
    for m, v, lc in frames:
      _c, telem = a.update(0.0, m, v, lc)
      n += 1
      if telem['state'] == 'anchor':
        anchor_n += 1
        if abs(telem['excess']) > 1e-6:
          biasing += 1
      max_bias = max(max_bias, abs(telem['kappa_bias']))
  if n == 0:
    return
  print(f'  {route}: n={n} ANCHOR={anchor_n/n*100:.0f}% biasing={biasing/max(anchor_n,1)*100:.0f}% '
        f'max|kappa_bias|={max_bias:.5f}')


if __name__ == '__main__':
  routes = sys.argv[1:] or ['000003b7--977baff7b6', '000003bb--12fe0dc6be']
  for tp in (1.0, 1.5, 2.0):
    print(f'T_PREVIEW={tp}s:')
    for r in routes:
      run_route(r, tp)
```

- [ ] **Step 2: Deploy to C3 and run**

```bash
cd plugins
GIT_SSH_COMMAND='ssh -o BatchMode=yes' git push git@github.com:catpilot-dev/plugins.git lane_keeping:refs/heads/lane_keeping
ssh c3 'cd /data/plugins && GIT_SSL_NO_VERIFY=1 git fetch origin lane_keeping && git checkout lane_keeping && git reset --hard origin/lane_keeping && bash install.sh'
ssh c3 'source /usr/local/venv/bin/activate && cd /data/plugins-runtime/lane_keeping && PYTHONPATH=/data/openpilot:/data/plugins-runtime/lane_keeping python tests/replay_anchor.py'
```
Expected: per-route ANCHOR occupancy (prior data suggests 60–100% confident-line), biasing fraction, and `max|kappa_bias|` growing modestly as `T_PREVIEW` shrinks. Use to confirm `T_PREVIEW=1.5` keeps the bias gentle; record the numbers in the spec if they change the default.

- [ ] **Step 3: Commit**

```bash
cd plugins
git add lane_keeping/tests/replay_anchor.py
git commit -m "lane_keeping: offline replay validation + T_PREVIEW sweep"
```

---

### Task 9: On-device probe harness

**Files:**
- Create: `plugins/lane_keeping/tests/on_device_probe.py`

**Interfaces:**
- Consumes: the deployed runtime `register.on_curvature_correction` and `anchor.LaneAnchor`.
- Produces: a standalone probe script (run offroad on C3) asserting every branch against the real runtime tree: passthrough (no line / low prob / disabled), in-band no-bias, out-of-band correct-sign bias (both driver sides), lane-change disable, glitch clip, rate limit, hard cap.

- [ ] **Step 1: Write the probe harness**

`plugins/lane_keeping/tests/on_device_probe.py`:
```python
#!/usr/bin/env python3
"""On-device probe for the lane_keeping runtime (run offroad on C3).

Loads /data/plugins-runtime/lane_keeping/anchor.py directly and asserts every
control branch. No cereal pub needed — anchor.py is pure.

Usage (on C3, offroad):
  source /usr/local/venv/bin/activate
  python /data/plugins-runtime/lane_keeping/tests/on_device_probe.py
"""
import importlib.util, os, sys
from types import SimpleNamespace

PLUGIN_DIR = '/data/plugins-runtime/lane_keeping'
spec = importlib.util.spec_from_file_location('lk_anchor', os.path.join(PLUGIN_DIR, 'anchor.py'))
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
AnchorConfig, LaneAnchor = mod.AnchorConfig, mod.LaneAnchor

PASS = FAIL = 0
def check(name, cond, detail=''):
  global PASS, FAIL
  print(f'  [{"PASS" if cond else "FAIL"}] {name} {detail}')
  PASS, FAIL = PASS + cond, FAIL + (not cond)

def mv(left_y, right_y, lp=1.0, rp=1.0):
  return SimpleNamespace(
    laneLines=[SimpleNamespace(y=[0.0]), SimpleNamespace(y=[left_y]),
               SimpleNamespace(y=[right_y]), SimpleNamespace(y=[0.0])],
    laneLineProbs=[0.0, lp, rp, 0.0])

def settle(a, m, v=25.0, lc=False, n=3000):
  out = None
  for _ in range(n):
    out, _t = a.update(0.01, m, v, lc)
  return out

print('probe 1: passthrough')
a = LaneAnchor(AnchorConfig())
o, t = a.update(0.0123, SimpleNamespace(laneLines=[], laneLineProbs=[]), 25.0, False)
check('no line -> bit-identical passthrough', o == 0.0123 and t['state'] == 'model')
o, t = a.update(0.0123, mv(1.75, -1.75, lp=0.4), 25.0, False)
check('low prob -> passthrough', o == 0.0123 and t['state'] == 'model')

print('probe 2: in-band no bias')
check('gap 0.84 in [0.6,1.0] -> no bias', abs(settle(LaneAnchor(AnchorConfig()), mv(1.75, -1.75)) - 0.01) < 1e-6)

print('probe 3: out-of-band bias, left driver')
check('too far from left line -> steer left', settle(LaneAnchor(AnchorConfig()), mv(2.3, -1.2)) > 0.01 + 1e-5)
check('too close to left line -> steer right', settle(LaneAnchor(AnchorConfig()), mv(1.3, -2.2)) < 0.01 - 1e-5)

print('probe 4: out-of-band bias, right driver')
check('right driver too far from right line -> steer right',
      settle(LaneAnchor(AnchorConfig(driver_side='right')), mv(-1.2, -2.3)) < 0.01 - 1e-5)

print('probe 5: lane-change disable')
check('lane change -> passthrough', abs(settle(LaneAnchor(AnchorConfig()), mv(2.3, -1.2), lc=True) - 0.01) < 1e-6)

print('probe 6: glitch clip + hard cap')
a = LaneAnchor(AnchorConfig())
a.gap_filt = 9.0
_o, t = a.update(0.01, mv(9.9, -1.2), 25.0, False)
check('huge gap -> excess clipped to excess_max', abs(t['excess']) == 0.5)

print('probe 7: rate limit')
a = LaneAnchor(AnchorConfig(kappa_rate_max=0.002))
m = mv(2.3, -1.2); a.gap_filt = a._gap(m)
_o, t = a.update(0.01, m, 25.0, False)
check('first tick bias <= rate*DT', abs(t['kappa_bias']) <= 0.002 * 0.01 + 1e-12)

print(f'\n{PASS} passed, {FAIL} failed')
sys.exit(1 if FAIL else 0)
```

- [ ] **Step 2: Deploy and run on C3 (offroad)**

```bash
cd plugins
GIT_SSH_COMMAND='ssh -o BatchMode=yes' git push git@github.com:catpilot-dev/plugins.git lane_keeping:refs/heads/lane_keeping
ssh c3 'cd /data/plugins && GIT_SSL_NO_VERIFY=1 git fetch origin lane_keeping && git reset --hard origin/lane_keeping && bash install.sh'
ssh c3 'source /usr/local/venv/bin/activate && python /data/plugins-runtime/lane_keeping/tests/on_device_probe.py'
```
Expected: `7 ... passed, 0 failed` (probe count may differ; all PASS).

- [ ] **Step 3: Commit**

```bash
cd plugins
git add lane_keeping/tests/on_device_probe.py
git commit -m "lane_keeping: on-device probe harness"
```

---

## Post-plan: on-car verification (manual, not a code task)

After Task 9 passes on-device, deploy and drive a structured highway route, then
run the 3b7/3bb lane-offset pipeline against the `lane_keeping` telemetry to
confirm: ANCHOR engages on clear-line stretches; driver-side gap holds in
[GAP_MIN, GAP_MAX]; position drift is **lower** than the DRIFT_M-only baseline
(the wander-rejection claim); no new churn or snap at prob transitions. This is
the Phase 1.5 evidence gate that authorizes Phase 2 (DRIFT_M removal, separate spec).

## Self-review notes

- **Spec coverage:** §3.1 hook wiring → Task 7; §3.2 signals → Task 2; §3.3 control law (filter/deadband/pursuit/authority/rate) → Tasks 3–5; §3.4 safety (glitch clip, cap, fade, lane-change, passthrough, release) → Tasks 3–5; §3.5 config → Task 6; §3.6 telemetry → Tasks 5,7; §4 testing layers 1–3 → Tasks 2–9 (layer 4 on-car = post-plan); §5 phasing → post-plan note.
- **Type consistency:** `update() -> (float, dict)` used consistently in Tasks 5, 7, 8, 9; `AnchorConfig` field names match between `anchor.py`, `_load_config`, and tests.
- **relax-dwell / DRIFT_M removal:** out of scope (Phase 2, separate spec) — this plan touches no existing controller code.
