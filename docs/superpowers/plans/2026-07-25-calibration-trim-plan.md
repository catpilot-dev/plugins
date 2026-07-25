# Calibration Trim Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task.

**Goal:** Perception-side DC position compensation — a slow, bounded yaw bias on the camera-warp calibration, closed on the anchor's gap_dc, with calibrationd kept provably blind.

**Architecture:** Two hook call sites in catpilot's modeld.py (warp bias + cameraOdometry de-bias); pure trim law + file transport + modeld-side reader in the lane_keeping plugin. Spec: `docs/superpowers/specs/2026-07-25-calibration-trim-design.md` (authoritative for all values).

**Tech stack:** Python, pytest, catpilot hook framework (`hooks.run`, fail-safe default), plugin data-dir params.

## Global Constraints

- Repos: Task 1 in `/home/oxygen/catpilot-dev/catpilot` (branch `dev`); Tasks 2–4 in `/home/oxygen/catpilot-dev/plugins` (branch `dev`). NOTHING is deployed to the C3 in this plan.
- modeld hard clamp ±1.0°; law cap `CalibTrimMaxDeg` default 0.8°; slew `CalibTrimSlewDegS` default 0.02 °/s; params exactly as spec §5 table.
- `modeld.calib_bias` hook body in the modeld process = cached float file read ONLY (no anchor/trim imports there).
- All plugin UI/hook imports lazy; hook never short-circuited; data-dir params only (never /data/params/d).
- File transport: `data/CalibTrimYawDeg`, atomic tmp + `os.replace`, 1 Hz, skip unchanged at 0.001° resolution.
- The de-bias touches ONLY cameraOdometry.trans/rot (R_z(−b)); zero bias must be a bit-exact no-op at both call sites.
- Pure law module: math/dataclasses only, module-level `DT_CTRL = 0.01` (replay harnesses patch it — standing rule).
- Plugins test command: `PYTHONPATH=. uv run pytest`; catpilot tests: `pytest selfdrive/modeld/tests/test_calib_bias.py` from repo root.

---

### Task 1: catpilot modeld call sites + de-bias helper

**Files:**
- Create: `selfdrive/modeld/calib_bias.py` (catpilot repo)
- Modify: `selfdrive/modeld/modeld.py` (~lines 22–29 imports, ~348–351 call site A, ~407–410 call site B)
- Test: `selfdrive/modeld/tests/test_calib_bias.py`

**Interfaces:**
- Produces: `hooks.run('modeld.calib_bias', 0.0)` call site (plugins register against this name); helper `apply_pose_debias(trans, rot, yaw_bias_deg) -> (trans, rot)` and `clamp_bias_deg(v) -> float`.

- [ ] **Step 1: failing tests** — `selfdrive/modeld/tests/test_calib_bias.py`:

```python
import numpy as np
from openpilot.selfdrive.modeld.calib_bias import apply_pose_debias, clamp_bias_deg


def _rot_z(b):
  c, s = np.cos(b), np.sin(b)
  return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def test_debias_inverts_warp_rotation():
  # pose head reports vectors in a frame rotated by +b; de-bias must restore truth
  true_trans = np.array([20.0, 0.3, -0.1])
  true_rot = np.array([0.01, -0.02, 0.005])
  b = np.radians(0.8)
  reported_trans = _rot_z(b).T @ true_trans  # frame rotated by +b => components by R^T
  reported_rot = _rot_z(b).T @ true_rot
  out_trans, out_rot = apply_pose_debias(list(reported_trans), list(reported_rot), 0.8)
  assert np.allclose(out_trans, true_trans, atol=1e-6)
  assert np.allclose(out_rot, true_rot, atol=1e-6)


def test_zero_bias_identity():
  t, r = [1.0, 2.0, 3.0], [0.1, 0.2, 0.3]
  out_t, out_r = apply_pose_debias(t, r, 0.0)
  assert out_t == t and out_r == r


def test_z_components_untouched():
  out_t, out_r = apply_pose_debias([10.0, 1.0, 0.5], [0.0, 0.0, 0.2], 0.8)
  assert out_t[2] == 0.5 and out_r[2] == 0.2


def test_clamp():
  assert clamp_bias_deg(2.5) == 1.0
  assert clamp_bias_deg(-2.5) == -1.0
  assert clamp_bias_deg(0.3) == 0.3
```

- [ ] **Step 2:** run, expect FAIL (module missing).
- [ ] **Step 3:** implement `selfdrive/modeld/calib_bias.py`:

```python
"""Perception-side yaw bias helpers (catpilot calibration-trim feature).

A yaw bias b added to the device_from_calib euler rotates the calibrated
frame the model perceives. The model's pose head, running on the biased
warp, therefore reports device-frame vectors expressed in a frame rotated
by +b about device z. Rotating the reported components by R_z applied as
the inverse frame change (see spec 2026-07-25-calibration-trim-design.md
section 4) restores the true device frame, keeping calibrationd's
observations bias-invariant (its yaw observation is atan2(trans[1],
trans[0]); pitch/height use z components, untouched here).
"""
import math

BIAS_HARD_CLAMP_DEG = 1.0


def clamp_bias_deg(v: float) -> float:
  return max(-BIAS_HARD_CLAMP_DEG, min(BIAS_HARD_CLAMP_DEG, float(v)))


def apply_pose_debias(trans, rot, yaw_bias_deg: float):
  if yaw_bias_deg == 0.0:
    return trans, rot
  b = math.radians(yaw_bias_deg)
  c, s = math.cos(b), math.sin(b)
  def _undo(vec):
    x, y = vec[0], vec[1]
    return [c * x - s * y, s * x + c * y, vec[2]]
  return _undo(trans), _undo(rot)
```

  NOTE to implementer: derive the sign inside `_undo` from the test, not
  from this listing — the test encodes the frame-change convention
  (`reported = R^T @ true` ⇒ `true = R @ reported`). If the test fails on
  sign, fix the implementation, never the test.

- [ ] **Step 4:** run tests, expect PASS.
- [ ] **Step 5:** wire modeld.py. Import block (near line 22, after existing openpilot imports):

```python
from openpilot.selfdrive.plugins.hooks import hooks
from openpilot.selfdrive.modeld.calib_bias import apply_pose_debias, clamp_bias_deg
```

  Call site A — replace line 348 area:

```python
      device_from_calib_euler = np.array(sm["liveCalibration"].rpyCalib, dtype=np.float32)
      yaw_bias_deg = clamp_bias_deg(hooks.run('modeld.calib_bias', 0.0))
      if yaw_bias_deg != 0.0:
        device_from_calib_euler = device_from_calib_euler.copy()
        device_from_calib_euler[2] += np.float32(np.radians(yaw_bias_deg))
```

  IMPORTANT: `yaw_bias_deg` must be a main-loop variable initialized to 0.0 before the loop and refreshed ONLY inside the `if sm.updated["liveCalibration"] ...` block shown, so call site B always uses the same value the warp was built with (the warp matrices persist between calibration updates; the bias must persist identically).

  Call site B — after `fill_pose_msg(...)`, before `pm.send('cameraOdometry', posenet_send)`:

```python
      if yaw_bias_deg != 0.0:
        od = posenet_send.cameraOdometry
        t, r = apply_pose_debias(list(od.trans), list(od.rot), yaw_bias_deg)
        od.trans = t
        od.rot = r
```

- [ ] **Step 6:** run test file + `python -c "import selfdrive.modeld.modeld"` sanity (imports resolve; full modeld won't run off-device — do not try).
- [ ] **Step 7:** commit (catpilot repo, dev): `feat(modeld): calib_bias hook call sites for perception-side yaw trim`

### Task 2: trim law — `plugins/lane_keeping/calib_trim.py`

**Files:** Create `plugins/lane_keeping/calib_trim.py`; Test `plugins/lane_keeping/tests/test_calib_trim.py`

**Interfaces:**
- Produces: `TrimConfig` (fields: `mode:int=0, fixed_deg:float=0.0, max_deg:float=0.8, slew_deg_s:float=0.02, yaw_sign:int=0, ki:float=0.04, gap_lo:float=0.6, gap_hi:float=1.0`), `CalibTrim` with `update(gap_dc, authority, lane_changing, v_ego, enabled) -> (delta_deg, telem)`; module-level `DT_CTRL = 0.01`, `IN_BAND_DWELL_S = 5.0`, `HOLD_MIN_SPEED = 5.0`.

- [ ] **Step 1: failing tests** (complete file):

```python
import importlib.util
import os

_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
spec = importlib.util.spec_from_file_location('lk_calib_trim', os.path.join(_DIR, 'calib_trim.py'))
ct = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ct)


def _run(trim, n, **kw):
  args = dict(gap_dc=0.8, authority=1.0, lane_changing=False, v_ego=15.0, enabled=True)
  args.update(kw)
  out = None
  for _ in range(n):
    out = trim.update(**args)
  return out


def test_mode0_stays_zero():
  trim = ct.CalibTrim(ct.TrimConfig(mode=0))
  d, _ = _run(trim, 1000)
  assert d == 0.0


def test_mode1_slews_to_fixed_and_respects_cap():
  trim = ct.CalibTrim(ct.TrimConfig(mode=1, fixed_deg=0.3))
  d1, _ = trim.update(0.8, 1.0, False, 15.0, True)
  assert 0 < d1 <= 0.02 * ct.DT_CTRL + 1e-12          # first tick slew-limited
  d, _ = _run(trim, 100 * 60)                          # 60 s
  assert abs(d - 0.3) < 1e-6
  trim2 = ct.CalibTrim(ct.TrimConfig(mode=1, fixed_deg=5.0, max_deg=0.8))
  d, _ = _run(trim2, 100 * 120)
  assert abs(d - 0.8) < 1e-6                           # capped


def test_mode1_rate_never_exceeds_slew():
  trim = ct.CalibTrim(ct.TrimConfig(mode=1, fixed_deg=0.8))
  prev = 0.0
  for _ in range(2000):
    d, _ = trim.update(0.8, 1.0, False, 15.0, True)
    assert abs(d - prev) <= 0.02 * ct.DT_CTRL + 1e-12
    prev = d


def test_mode2_inert_without_sign():
  trim = ct.CalibTrim(ct.TrimConfig(mode=2, yaw_sign=0))
  d, _ = _run(trim, 100 * 30, gap_dc=0.2)              # far out of band
  assert d == 0.0


def test_mode2_integrates_toward_band_both_signs():
  for sign in (1, -1):
    trim = ct.CalibTrim(ct.TrimConfig(mode=2, yaw_sign=sign))
    d, t = _run(trim, 100 * 10, gap_dc=0.3)            # err = -0.3 (too close)
    assert t['integrating']
    assert d != 0.0
    # convention: positive err moves gap down; err<0 must push delta the
    # direction that (per yaw_sign) raises the gap: d has sign +yaw_sign*? —
    # pin the ALGEBRA, not intuition: dδ = clip(-ki*err*yaw_sign, ±slew)*DT
    exp_sign = 1.0 if (-(-0.3) * sign) > 0 else -1.0
    assert (d > 0) == (exp_sign > 0)


def test_mode2_in_band_decays_after_dwell():
  trim = ct.CalibTrim(ct.TrimConfig(mode=2, yaw_sign=1))
  _run(trim, 100 * 20, gap_dc=0.3)                     # build up some delta
  d_built, _ = trim.update(0.3, 1.0, False, 15.0, True)
  assert d_built != 0.0
  d_after_dwell, t = _run(trim, 100 * 5, gap_dc=0.8)   # in band, dwell running
  assert abs(d_after_dwell - d_built) <= 0.02 * 0.05 + 1e-9  # no decay yet (first 5 s)
  d_decayed, _ = _run(trim, 100 * 30, gap_dc=0.8)
  assert abs(d_decayed) < abs(d_built)                 # decaying at slew/2
  d_final, _ = _run(trim, 100 * 120, gap_dc=0.8)
  assert d_final == 0.0


def test_hold_on_untrusted_lc_slow():
  trim = ct.CalibTrim(ct.TrimConfig(mode=2, yaw_sign=1))
  _run(trim, 100 * 20, gap_dc=0.3)
  d0, _ = trim.update(0.3, 1.0, False, 15.0, True)
  for kw in (dict(authority=0.0), dict(lane_changing=True), dict(v_ego=3.0)):
    d, t = _run(trim, 100 * 30, gap_dc=0.3, **kw)
    assert d == d0 and not t['integrating']            # held exactly


def test_disabled_decays_to_zero():
  trim = ct.CalibTrim(ct.TrimConfig(mode=2, yaw_sign=1))
  _run(trim, 100 * 20, gap_dc=0.3)
  d, _ = _run(trim, 100 * 200, enabled=False)
  assert d == 0.0


def test_frozen_gap_dc_accepted():
  trim = ct.CalibTrim(ct.TrimConfig(mode=2, yaw_sign=1))
  d, t = _run(trim, 100 * 10, gap_dc=0.43)             # frozen below band
  assert t['integrating'] and d != 0.0
```

- [ ] **Step 2:** run → FAIL. **Step 3:** implement:

```python
"""Calibration trim law — slow yaw-bias integrator on gap_dc.

Pure math; no I/O. Spec: docs/superpowers/specs/
2026-07-25-calibration-trim-design.md section 5. delta_deg is the ONLY
state; every transition is slew-limited; the value is written to
data/CalibTrimYawDeg by register.py and applied inside modeld via the
modeld.calib_bias hook.
"""
from dataclasses import dataclass

DT_CTRL = 0.01
IN_BAND_DWELL_S = 5.0
HOLD_MIN_SPEED = 5.0


def _clip(v, lo, hi):
  return max(lo, min(hi, v))


@dataclass
class TrimConfig:
  mode: int = 0
  fixed_deg: float = 0.0
  max_deg: float = 0.8
  slew_deg_s: float = 0.02
  yaw_sign: int = 0
  ki: float = 0.04
  gap_lo: float = 0.6
  gap_hi: float = 1.0


class CalibTrim:
  def __init__(self, cfg: TrimConfig):
    self.cfg = cfg
    self.delta_deg = 0.0
    self._in_band_ticks = 0

  def _slew_toward(self, target, rate):
    step = rate * DT_CTRL
    self.delta_deg += _clip(target - self.delta_deg, -step, step)

  def update(self, gap_dc, authority, lane_changing, v_ego, enabled):
    cfg = self.cfg
    integrating = False
    if not enabled or cfg.mode == 0 or (cfg.mode == 2 and cfg.yaw_sign not in (1, -1)):
      self._slew_toward(0.0, cfg.slew_deg_s)
      self._in_band_ticks = 0
    elif cfg.mode == 1:
      self._slew_toward(_clip(cfg.fixed_deg, -cfg.max_deg, cfg.max_deg), cfg.slew_deg_s)
    else:  # mode 2, sign valid
      gate = (gap_dc is not None and authority > 0.0
              and not lane_changing and v_ego >= HOLD_MIN_SPEED)
      if gate:
        if gap_dc < cfg.gap_lo:
          err = gap_dc - cfg.gap_lo
        elif gap_dc > cfg.gap_hi:
          err = gap_dc - cfg.gap_hi
        else:
          err = 0.0
        if err != 0.0:
          self._in_band_ticks = 0
          integrating = True
          self.delta_deg += _clip(-cfg.ki * err * cfg.yaw_sign,
                                  -cfg.slew_deg_s, cfg.slew_deg_s) * DT_CTRL
        else:
          self._in_band_ticks += 1
          if self._in_band_ticks * DT_CTRL > IN_BAND_DWELL_S:
            self._slew_toward(0.0, cfg.slew_deg_s / 2.0)
      # gate False: hold (no integrate, no decay)
    self.delta_deg = _clip(self.delta_deg, -cfg.max_deg, cfg.max_deg)
    telem = {'delta_deg': self.delta_deg, 'mode': cfg.mode, 'integrating': integrating}
    return self.delta_deg, telem
```

- [ ] **Step 4:** run → PASS. **Step 5:** commit `lane_keeping: calibration trim law (calib_trim.py)`.

### Task 3: register wiring — writer + modeld-side reader

**Files:** Modify `plugins/lane_keeping/register.py`; Test `plugins/lane_keeping/tests/test_register_trim.py`

**Interfaces:**
- Consumes: `CalibTrim`/`TrimConfig` (Task 2, loaded by explicit path like `_anchor_module()`); anchor telemetry keys `gap_dc`, plus authority via existing config/prob logic — pass what `on_curvature_correction` already has: use telem `gap_dc`, `state`=='anchor' as the authority>0 proxy is NOT sufficient — thread the anchor's actual `authority` into its telemetry dict if not already present (add key `authority` in anchor.py telem; one-line, covered by an assertion in tests).
- Produces: `data/CalibTrimYawDeg` file; `modeld.calib_bias` registered hook `on_calib_bias(default)`.

- [ ] Steps: failing tests → implement → pass → commit `lane_keeping: trim wiring (writer + modeld.calib_bias reader)`. Requirements the tests must pin:
  1. `_load_trim_config()` reads all §5 params with `fget`/int casts, defaults from `TrimConfig()`.
  2. In `on_curvature_correction`: after anchor update, `_trim.update(telem.get('gap_dc'), telem.get('authority', 0.0), lane_changing, v_ego, cfg.enable)`; every 100 ticks write via `_write_yaw_file(delta)`: `tmp = path + '.tmp'`; write `f"{delta:.3f}"`; `os.replace(tmp, path)`; skip when `round(delta,3)` unchanged. Trim exceptions swallowed like telemetry (`try/except` — trim must never break the control path).
  3. `on_calib_bias(default)`: module-level cache `{val, calls}`; re-read file every 100 calls; missing/invalid → 0.0; clamp to a module-level constant `_CLAMP_DEG_DEFAULT = 0.8` (mirrors TrimConfig.max_deg — NO trim-module import in this path, per spec §6); registered declaratively in plugin.json under `modeld.calib_bias`.
  4. Test: writer atomicity (no partial file mid-write — assert tmp never left behind), round-trip float, reader cache refresh at call 100, reader 0.0 on garbage file, reader clamp, and `authority` key present in anchor telemetry (import anchor by path, run one update, assert key).

### Task 4: probes, docs, ledger

**Files:** Modify `plugins/lane_keeping/tests/on_device_probe.py`, `plugins/lane_keeping/README.md`, `plugins/lane_keeping/plugin.json` (mention trim, mode 0 default); append `.superpowers/sdd/progress.md`.

- [ ] Probe additions (mirroring existing probe style): (p22) trim file round-trip through `_write_yaw_file` + `on_calib_bias` fresh-cache read; (p23) `on_calib_bias` returns 0.0 with file deleted; (p24) trim law slew/cap smoke (mode 1, 0.3°, 2000 ticks → monotone, ≤slew); (p25) mode 2 with yaw_sign=0 stays 0.0. Run full plugins suite; commit `lane_keeping: calibration trim probes + docs`.

## Self-review notes

- Spec §5 params all appear in Task 2/3; §4 call sites verbatim in Task 1; §8 catpilot test = Task 1 Step 1; probe additions cover §8 device items except modelExecutionTime (needs the C3 — deferred to deploy step 3 per spec §9, noted in probe file as TODO comment).
- Type consistency: `update(gap_dc, authority, lane_changing, v_ego, enabled)` uniform across Tasks 2/3; `delta_deg` float degrees everywhere; file format `%.3f` degrees.
- No deploy tasks — spec §9 gates deployment on the user's OFF drive and explicit go.
