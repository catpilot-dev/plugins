# Phase 2 — Lateral Controller Simplification Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move all modelV2 noise handling out of the BMW lateral controller and into the `lane_keeping` layer, reducing the controller to a faithful curvature→torque tracker.

**Architecture:** `lane_keeping` becomes the reference conditioner — it low-passes the model's κ_des (kills fast chatter) and adds the driver-side position correction (cancels sub-Hz drift), handing down one clean curvature. The BMW controller deletes DRIFT_M/tolerance, the box filter, the σ-observer and the persist gate; its P term acts on the **full** `delta_err`, and the stiction hold retriggers on a small friction-sized `HOLD_BAND`.

**Tech Stack:** Python 3.11, openpilot plugin framework, pytest (`PYTHONPATH=. uv run pytest`), on-device probe harnesses run on the C3.

## Global Constraints

- Spec: `docs/superpowers/specs/2026-07-22-lateral-controller-simplification-design.md`.
- `plugins/lane_keeping/anchor.py` stays pure (`math` + `dataclasses` only — no cereal/opendbc/zmq).
- modelV2 device frame is **`+y = right`** (left ego line `laneLines[1]` at NEGATIVE y); curvature is **left-positive**. `line_sign` (gap: left −1 / right +1) and `curv_sign` (bias: left +1 / right −1) are already correct — do not touch them.
- `HOLD_BAND = 0.001` rad, fixed. `KAPPA_FILTER_TAU = 0.3` s.
- MODEL state must still return the input curvature bit-identical when the anchor is released and no smoothing change applies.
- Plugin params live in the plugin's `data/` dir, NEVER `/data/params/d/`.
- Do NOT change: plant-inversion P, `kappa_scale`, `t_cap`, `STEP_MAX`, `hold_curve`/hold-floor/`hold_cap`/sign-guard, ISO `cancel_accel`/`cancel_jerk`, `relax-dwell`.
- Run the FULL suite before each commit: `cd /home/oxygen/catpilot-dev/plugins && PYTHONPATH=. uv run pytest plugins/ -q` (369 passed, 20 skipped as of a0afa58).
- Commit after every task. NO `Co-Authored-By` lines.

---

### Task 1: lane_keeping — reference smoothing

**Files:**
- Modify: `plugins/lane_keeping/anchor.py`
- Test: `plugins/lane_keeping/tests/test_anchor.py`

**Interfaces:**
- Consumes: existing `AnchorConfig`, `LaneAnchor.update(curvature, model_v2, v_ego, lane_changing) -> (float, dict)`, `_clip`, `DT_CTRL`.
- Produces: `AnchorConfig.kappa_filter_tau: float = 0.3`; `LaneAnchor.kappa_filt` state (None until first sample); `update()` now returns `kappa_ref + kappa_bias` where `kappa_ref` is the low-passed input (raw during a lane change); telem gains `kappa_in` and `kappa_ref`.

- [ ] **Step 1: Write the failing tests**

Append to `plugins/lane_keeping/tests/test_anchor.py`:
```python
def test_smoothing_lags_a_step_then_converges():
  # No line -> no position bias, so the output is purely the smoothed reference.
  a = LaneAnchor(AnchorConfig(kappa_filter_tau=0.3))
  none_mv = SimpleNamespace(laneLines=[], laneLineProbs=[])
  out1, t1 = a.update(0.02, none_mv, 25.0, False)
  assert abs(out1 - 0.02) < 1e-9          # first sample seeds the filter
  assert abs(t1['kappa_in'] - 0.02) < 1e-9
  out2, _ = a.update(0.0, none_mv, 25.0, False)   # step down
  assert 0.0 < out2 < 0.02                # lags, does not jump
  for _ in range(500):                    # ~5s >> tau
    out, _t = a.update(0.0, none_mv, 25.0, False)
  assert abs(out) < 1e-4                  # converges


def test_smoothing_bypassed_during_lane_change():
  a = LaneAnchor(AnchorConfig(kappa_filter_tau=0.3))
  none_mv = SimpleNamespace(laneLines=[], laneLineProbs=[])
  for _ in range(200):
    a.update(0.02, none_mv, 25.0, False)  # settle filter at 0.02
  out, telem = a.update(0.0, none_mv, 25.0, True)   # lane change -> raw
  assert out == 0.0                       # bit-identical passthrough of raw
  assert abs(telem['kappa_ref']) < 1e-12


def test_smoothing_applies_in_anchor_state_too():
  # smoothing is unconditional; only the position correction is ANCHOR-gated
  a = LaneAnchor(AnchorConfig(kappa_filter_tau=0.3))
  mv = _mv(left_y=-1.75, right_y=1.75)    # gap 0.84 in band -> zero bias
  a.update(0.02, mv, 25.0, False)
  out, telem = a.update(0.0, mv, 25.0, False)
  assert telem['state'] == 'anchor'
  assert 0.0 < out < 0.02                 # smoothed, bias is zero in-band
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd /home/oxygen/catpilot-dev/plugins && PYTHONPATH=. uv run pytest plugins/lane_keeping/tests/test_anchor.py -k smoothing -v`
Expected: FAIL — `TypeError: AnchorConfig.__init__() got an unexpected keyword argument 'kappa_filter_tau'`

- [ ] **Step 3: Add the config field and filter state**

In `plugins/lane_keeping/anchor.py`, add to `AnchorConfig` (after `filter_tau`):
```python
  kappa_filter_tau: float = 0.3  # low-pass on the model's kappa_des (s)
```

In `LaneAnchor.__init__`, add beside `self.gap_filt = None`:
```python
    self.kappa_filt = None
```

- [ ] **Step 4: Add smoothing to `update()`**

In `plugins/lane_keeping/anchor.py`, at the TOP of `update()` (immediately after `cfg = self.cfg`), insert:
```python
    # Reference conditioning (Phase 2): low-pass the model's curvature to kill
    # the fast chatter, so the BMW controller can track it faithfully with no
    # deadzone. Safe because any lag this introduces shows up as position
    # drift, which the position correction below closes. A lane change bypasses
    # it — the model is deliberately reframing the trajectory.
    if lane_changing or self.kappa_filt is None:
      self.kappa_filt = curvature
      kappa_ref = curvature
    else:
      a_k = 1.0 - math.exp(-DT_CTRL / cfg.kappa_filter_tau)
      self.kappa_filt += a_k * (curvature - self.kappa_filt)
      kappa_ref = self.kappa_filt
```

Change the `return` at the end of `update()` from:
```python
    return curvature + self.kappa_bias, self._telem(prob, line_y, gap, excess, authority, v_ego)
```
to:
```python
    return kappa_ref + self.kappa_bias, self._telem(prob, line_y, gap, excess, authority, v_ego,
                                                    curvature, kappa_ref)
```

Change `_telem`'s signature and body to carry the new fields:
```python
  def _telem(self, prob, line_y, gap, excess, authority, v_ego, kappa_in=0.0, kappa_ref=0.0):
    return {
      'prob': float(prob), 'line_y': float(line_y), 'gap': float(gap),
      'gap_filt': float(self.gap_filt) if self.gap_filt is not None else 0.0,
      'excess': float(excess), 'kappa_bias': float(self.kappa_bias),
      'authority': float(authority), 'state': self.state, 'v_ego': float(v_ego),
      'kappa_in': float(kappa_in), 'kappa_ref': float(kappa_ref),
    }
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `cd /home/oxygen/catpilot-dev/plugins && PYTHONPATH=. uv run pytest plugins/lane_keeping/tests/ -v`
Expected: PASS (23 tests — 20 prior + 3 new)

- [ ] **Step 6: Run the full suite and commit**

Run: `cd /home/oxygen/catpilot-dev/plugins && PYTHONPATH=. uv run pytest plugins/ -q`
Expected: `372 passed, 20 skipped`

```bash
cd /home/oxygen/catpilot-dev/plugins
git add plugins/lane_keeping/anchor.py plugins/lane_keeping/tests/test_anchor.py
git commit -m "lane_keeping: low-pass the model curvature (reference conditioning)"
```

---

### Task 2: lane_keeping — filter-tau config param

**Files:**
- Modify: `plugins/lane_keeping/register.py`
- Test: `plugins/lane_keeping/tests/test_register.py`

**Interfaces:**
- Consumes: `register._load_config() -> AnchorConfig`, `register._read_param`, the `data_dir` fixture.
- Produces: `_load_config()` reads `LaneKeepKappaFilterTau` → `AnchorConfig.kappa_filter_tau`.

- [ ] **Step 1: Write the failing test**

Append to `plugins/lane_keeping/tests/test_register.py`:
```python
def test_load_config_kappa_filter_tau(data_dir):
  cfg = register._load_config()
  assert cfg.kappa_filter_tau == 0.3          # default
  (data_dir / 'LaneKeepKappaFilterTau').write_text('0.45')
  cfg2 = register._load_config()
  assert cfg2.kappa_filter_tau == 0.45
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/oxygen/catpilot-dev/plugins && PYTHONPATH=. uv run pytest plugins/lane_keeping/tests/test_register.py -k kappa_filter -v`
Expected: FAIL — `assert 0.3 == 0.45` (override not read)

- [ ] **Step 3: Read the param**

In `plugins/lane_keeping/register.py`, inside `_load_config()`'s `AnchorConfig(...)` call, add after the `filter_tau=` line:
```python
    kappa_filter_tau=fget('LaneKeepKappaFilterTau', d.kappa_filter_tau),
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /home/oxygen/catpilot-dev/plugins && PYTHONPATH=. uv run pytest plugins/lane_keeping/tests/ -v`
Expected: PASS (24 tests)

- [ ] **Step 5: Run the full suite and commit**

Run: `cd /home/oxygen/catpilot-dev/plugins && PYTHONPATH=. uv run pytest plugins/ -q`
Expected: `373 passed, 20 skipped`

```bash
cd /home/oxygen/catpilot-dev/plugins
git add plugins/lane_keeping/register.py plugins/lane_keeping/tests/test_register.py
git commit -m "lane_keeping: LaneKeepKappaFilterTau config param"
```

---

### Task 3: bmw — rewrite the on-device probe for the simplified controller (RED)

**Files:**
- Modify: `plugins/bmw_e9x_e8x/tests/on_device_probe.py` (full rewrite of the probe list)

**Interfaces:**
- Consumes: the runtime `bmw/latcontroller.py` via `importlib`, `on_lat_controller_init(result, lac, CP)`, the closure `state` dict.
- Produces: a probe harness asserting the **post-Task-4** behavior. It is EXPECTED TO FAIL against the current controller — that is the TDD red step.

**Note for the implementer:** this task only rewrites the probe. Do NOT touch `latcontroller.py`. Running the probe at the end of this task is expected to FAIL; record which probes fail.

- [ ] **Step 1: Rewrite the probe file**

Replace the whole of `plugins/bmw_e9x_e8x/tests/on_device_probe.py` with:
```python
#!/usr/bin/env python3
"""On-device probe for the SIMPLIFIED bmw latcontroller (run offroad on C3).

Phase 2: the controller is a faithful tracker — P acts on the FULL delta_err
(no tolerance subtraction), the stiction hold retriggers on a small fixed
HOLD_BAND, and all modelV2 noise machinery (box filter, sigma-observer,
persist gate, DRIFT_M tolerance) is gone.

Usage (on C3, offroad), LK-style overridable plugin dir:
  source /usr/local/venv/bin/activate
  BMW_PLUGIN_DIR=/data/plugins-runtime/bmw_e9x_e8x python <this file>
"""
import importlib.util, math, os, sys, time
from types import SimpleNamespace

sys.path.insert(0, '/data/openpilot')
PLUGIN_DIR = os.environ.get('BMW_PLUGIN_DIR', '/data/plugins-runtime/bmw_e9x_e8x')
sys.path.insert(0, PLUGIN_DIR)

import numpy as np
import cereal.messaging as messaging
from openpilot.common.params import Params

if Params().get_bool('IsOnroad'):
  print('ABORT: device is onroad'); sys.exit(1)

spec = importlib.util.spec_from_file_location(
  'plugins.bmw_e9x_e8x.bmw.latcontroller', os.path.join(PLUGIN_DIR, 'bmw', 'latcontroller.py'))
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

pm = messaging.PubMaster(['livePose'])
time.sleep(0.3)

PASS = FAIL = 0
def check(name, cond, detail=''):
  global PASS, FAIL
  print(f'  [{"PASS" if cond else "FAIL"}] {name} {detail}')
  PASS, FAIL = PASS + cond, FAIL + (not cond)

def fresh():
  lac = SimpleNamespace()
  mod.on_lat_controller_init({}, lac, SimpleNamespace(wheelbase=2.76, steerActuatorDelay=0.4))
  st = None
  for c in (lac.update.__closure__ or []):
    try: v = c.cell_contents
    except ValueError: continue
    if isinstance(v, dict) and 'torque' in v and 'action' in v: st = v
  assert st is not None, 'state dict not found in closure'
  return lac, st

def tick(lac, kappa_des, v=25.0, yaw=0.0, n_can=1, active=True):
  msg = messaging.new_message('livePose')
  msg.livePose.velocityDevice.x = v
  msg.livePose.angularVelocityDevice.z = yaw
  pm.send('livePose', msg)
  time.sleep(0.004)
  out = None
  cs = SimpleNamespace(vEgo=v)
  for _ in range(n_can):
    out = lac.update(active, cs, None, None, False, kappa_des, False, 0.4)
  return out

L = 2.76
HOLD_BAND = 0.001
STEER_MAX = 12.0

print('probe 1: loads, and the noise machinery is GONE')
lac, st = fresh()
check('module loads, update patched', callable(lac.update))
gone = [k for k in ('k_sigma', 'de_dc', 'persist_w', 'de_w', 'de_buffer',
                    'kn_ema_f', 'kn_ema_s', 'kn_var', 'delta_err_raw', 'tolerance')
        if k in st]
check('no noise-machinery state keys remain', not gone, f'found={gone}')

print('probe 2: P acts on the FULL delta_err (no tolerance subtraction)')
# v=15: STEP_MAX=0.10 so the step cap does NOT bind; kappa_scale=1.0 at small kappa.
# kappa_des chosen so delta_err = atan(k*L) = 0.002 rad = 2*HOLD_BAND.
k_des = math.tan(0.002) / L
lac2, st2 = fresh()
tick(lac2, k_des, v=15.0, yaw=0.0)
expected = (1.0 * 1.0 * 15.0 * 15.0 * 0.002) / STEER_MAX   # full-error P = 0.0375
check('ramp fired (old code would HOLD: 0.002 < old tol 0.0024)', st2['action'] == 'ramp',
      f"action={st2['action']}")
check('target == full-error P', abs(st2['target_frac'] - expected) < 1e-4,
      f"target={st2['target_frac']:.5f} expected={expected:.5f}")

print('probe 3: HOLD_BAND is the hold trigger')
k_small = math.tan(0.0005) / L      # delta_err 0.0005 < HOLD_BAND
lac3, st3 = fresh()
tick(lac3, k_small, v=15.0, yaw=0.0)
check('inside HOLD_BAND -> hold_zero', st3['action'] == 'hold_zero', f"action={st3['action']}")

print('probe 4: delta_err is RAW (box filter deleted)')
lac4, st4 = fresh()
for _ in range(10):
  tick(lac4, math.tan(0.004) / L, v=15.0, yaw=0.0)   # settle a would-be filter high
tick(lac4, k_des, v=15.0, yaw=0.0)                    # step down to 0.002
check('delta_err equals the instantaneous value (no filter lag)',
      abs(st4['delta_err'] - 0.002) < 1e-4, f"delta_err={st4['delta_err']:.5f}")

print('probe 5: STEP_MAX still caps (regression)')
lac5, st5 = fresh()
tick(lac5, math.tan(0.02) / L, v=25.0, yaw=0.0)       # big error at 25 m/s
check('first step <= STEP_MAX(25 m/s) ~0.0615', abs(st5['target_frac']) <= 0.0625,
      f"target={st5['target_frac']:.4f}")

print('probe 6: hold_curve still holds torque in a curve (regression)')
lac6, st6 = fresh()
for _ in range(30):
  tick(lac6, 0.015, v=10.0, yaw=10.0 * 0.010, n_can=10)
built = st6['torque']
for _ in range(8):
  tick(lac6, 0.015, v=10.0, yaw=10.0 * 0.0149, n_can=10)
check('torque built in curve', built > 0.05, f'tq={built:.3f}')
check('hold engaged on-target', st6['action'] in ('hold_curve', 'cancel_tol'), f"action={st6['action']}")
check('held torque not drained', st6['torque'] > 0.03, f"tq={st6['torque']:.3f}")

print(f'\n{PASS} passed, {FAIL} failed')
sys.exit(1 if FAIL else 0)
```

- [ ] **Step 2: Deploy the probe to C3 and run it (expect FAILURES)**

Copy the WHOLE plugin dir — `latcontroller.py` imports `bmw.values`, so a
partial copy fails. This is non-activating: it touches `/tmp` only, never
`/data/plugins-runtime`.
```bash
cd /home/oxygen/catpilot-dev/plugins
ssh c3 'rm -rf /tmp/bmwp && mkdir -p /tmp/bmwp'
scp -r plugins/bmw_e9x_e8x/. c3:/tmp/bmwp/
ssh c3 'source /usr/local/venv/bin/activate && BMW_PLUGIN_DIR=/tmp/bmwp python /tmp/bmwp/tests/on_device_probe.py'
```
Expected: probes 1–4 FAIL (noise state keys present; `hold_zero` instead of `ramp`; target reduced by the tolerance subtraction; filtered `delta_err`). Probes 5–6 PASS. **This failure is the point of the task** — record the output in your report.

- [ ] **Step 3: Commit**

```bash
cd /home/oxygen/catpilot-dev/plugins
git add plugins/bmw_e9x_e8x/tests/on_device_probe.py
git commit -m "bmw: rewrite on-device probe for the simplified controller (red)"
```

---

### Task 4: bmw — simplify the lateral controller (GREEN)

**Files:**
- Modify: `plugins/bmw_e9x_e8x/bmw/latcontroller.py`

**Interfaces:**
- Consumes: the probe from Task 3.
- Produces: `HOLD_BAND = 0.001` module constant; `state` without any noise keys; `delta_err` computed raw; P on full `delta_err`; telemetry without `tolerance`/`delta_err_raw`/`de_w`/`k_sigma`/`de_dc`/`persist_w`, with `hold_band`.

- [ ] **Step 1: Delete the noise constants**

In `plugins/bmw_e9x_e8x/bmw/latcontroller.py`:
- Delete the `DRIFT_M = 0.02` line and its preceding comment block (the "Feedback deadzone" paragraph).
- Delete the entire `KN_*` block (from the comment starting "Online modelV2 noise observer + tolerance noise-floor" through `KN_DRIFT_CAP_M = 0.08`).
- Delete the entire sign-persistence block (from "Sign-persistence gate on the noise floor" through `KN_PERSIST_BP = [0.7, 1.3]`).
- Delete the `KD_BLEND_BP = [0.002, 0.004]` line and its preceding "κ-gated box filter on delta_err" comment block.

Add in their place (next to `FRICTION`):
```python
  # Stiction hold trigger (Phase 2, 2026-07-22). Replaces the DRIFT_M kinematic
  # deadzone, which existed only because there was no position feedback — that
  # job now belongs to the lane_keeping position loop, which also owns modelV2
  # noise. This band exists ONLY to decide when the rack is "on target" so the
  # curvature hold can take over; it is sized by STICTION, not by drift: below
  # the error where the P term commands less than rack breakaway
  # (FRICTION·STEER_MAX / (T_CAP_SLOPE·kappa_scale·v²) ≈ 0.001 rad at 25 m/s)
  # the wheel cannot move anyway. Small enough that it does not meaningfully
  # attenuate the lane_keeping position correction — the Phase 1 failure mode,
  # where the old 0.0012–0.0021 rad tolerance ate 44% of the anchor's command.
  HOLD_BAND = 0.001        # rad of front-wheel-angle error treated as on-target
```

- [ ] **Step 2: Delete the box filter, the observer, and the tolerance computation**

Replace this block (the blend filter + observer + tolerance, currently ~lines 480–565) — everything from the `# Blended box filter on delta_err` comment through `state['tolerance'] = tolerance` — with:
```python
      # Phase 2: no filtering here. modelV2 noise is handled upstream by
      # lane_keeping (it low-passes kappa_des and closes the position loop),
      # so this controller tracks whatever reference it is given, faithfully.
      delta_err = delta_err_raw
      state['delta_err'] = delta_err

      # Lookahead is still needed for the hold cap below.
      lookahead_m = v * model_action_t
```

Keep the lines immediately before it (`delta_des`, `delta_meas`, `delta_err_raw`) and everything after (`kappa_scale`, `hold_cap`, …) unchanged.

- [ ] **Step 3: Fix the three tolerance consumers**

(a) `cancel_tol` — change:
```python
      elif (state['action'] == 'ramp' and abs(delta_err) <= 1.2*tolerance
```
to:
```python
      elif (state['action'] == 'ramp' and abs(delta_err) <= 1.2*HOLD_BAND
```

(b) hold trigger — change:
```python
        if abs(delta_err) <= tolerance:
```
to:
```python
        if abs(delta_err) <= HOLD_BAND:
```

(c) full-error P — change:
```python
          effective_err = delta_err - math.copysign(tolerance, delta_err)
          target_nm = T_CAP_SLOPE_BASE * kappa_scale * v * v * effective_err
```
to:
```python
          # Phase 2: P acts on the FULL error — the tolerance subtraction is
          # gone. It was the term that attenuated the lane_keeping position
          # correction (Phase 1, route 3bf: 44% of correcting ticks produced no
          # action at all). Nothing here shrinks the commanded curvature now.
          target_nm = T_CAP_SLOPE_BASE * kappa_scale * v * v * delta_err
```

- [ ] **Step 4: Delete the dead state keys and the modelV2 subscription**

In the `state = {...}` dict, delete these entries: `'delta_err_raw'`, `'de_buffer'`, `'de_w'`, `'tolerance'`, `'kn_ema_f'`, `'kn_ema_s'`, `'kn_var'`, `'k_sigma'`, `'de_dc'`, `'persist_w'`.

Change the SubMaster (modelV2 was only used for the now-deleted `lane_change_active`):
```python
  _sm = messaging.SubMaster(['livePose'])
```

- [ ] **Step 5: Clean up telemetry**

In the telemetry payload, delete these keys: `'delta_err_raw'`, `'de_w'`, `'tolerance'`, `'k_sigma'`, `'de_dc'`, `'persist_w'`. Add:
```python
          'hold_band': float(HOLD_BAND),                      # stiction hold trigger (rad)
```

- [ ] **Step 6: Verify no dangling references**

Run:
```bash
cd /home/oxygen/catpilot-dev/plugins
grep -n "tolerance\|DRIFT_M\|KN_\|k_sigma\|de_dc\|persist_w\|de_w\|de_buffer\|KD_BLEND\|delta_err_raw\|lane_change_active" plugins/bmw_e9x_e8x/bmw/latcontroller.py | grep -v "^\s*#" | grep -v HOLD_CAP_DRIFT_M
```
Expected: no output except comment/doc lines. `HOLD_CAP_DRIFT_M` must REMAIN (it feeds `hold_cap`, which is kept).

Then: `python3 -m py_compile plugins/bmw_e9x_e8x/bmw/latcontroller.py` → no output.

- [ ] **Step 7: Run the probe on C3 — now GREEN**

```bash
cd /home/oxygen/catpilot-dev/plugins
scp plugins/bmw_e9x_e8x/bmw/latcontroller.py c3:/tmp/bmwp/bmw/
ssh c3 'source /usr/local/venv/bin/activate && BMW_PLUGIN_DIR=/tmp/bmwp python /tmp/bmwp/tests/on_device_probe.py'
```
Expected: `10 passed, 0 failed`

- [ ] **Step 8: Run the full suite and commit**

Run: `cd /home/oxygen/catpilot-dev/plugins && PYTHONPATH=. uv run pytest plugins/ -q`
Expected: `373 passed, 20 skipped`

```bash
cd /home/oxygen/catpilot-dev/plugins
git add plugins/bmw_e9x_e8x/bmw/latcontroller.py
git commit -m "bmw: simplify lateral controller — noise handling offloaded to lane_keeping

Deletes DRIFT_M/tolerance, the KD_BLEND box filter, the sigma-observer noise
floor and the sign-persistence gate. P now acts on the FULL delta_err (the
tolerance subtraction was what attenuated the lane_keeping position correction
- Phase 1 route 3bf: no action on 44% of correcting ticks). The stiction hold
retriggers on a small friction-sized HOLD_BAND instead of a drift-sized
deadzone. modelV2 subscription dropped (only fed the deleted filter gate)."
```

---

### Task 5: docs — LATERAL_CONTROLLER.md

**Files:**
- Modify: `plugins/bmw_e9x_e8x/LATERAL_CONTROLLER.md`

**Interfaces:**
- Consumes: the implemented behavior from Task 4.
- Produces: doc that matches the shipped controller.

- [ ] **Step 1: Add the Phase 2 header note**

Append after the final existing `>` header note (the 2026-07-19 sign-persistence note), before the `---`:
```markdown
> **2026-07-22 — Phase 2: noise handling offloaded to lane_keeping.** Route 3bf
> proved the position anchor cannot work through a deadzone: its bias was only
> 1.98× the tolerance and the controller took no action on 44% of correcting
> ticks (a 75 s excursion had the bias saturated, correctly signed, and never
> recovering). The controller is now a faithful tracker. **Deleted:** `DRIFT_M`
> + kinematic `tolerance`, the `KD_BLEND` box filter, the σ-observer noise floor
> (`KN_*`, `k_sigma`), and the sign-persistence gate (`de_dc`, `persist_w`).
> **P acts on the full `delta_err`** — the tolerance subtraction is gone, and it
> was the attenuator. The stiction hold retriggers on a fixed
> `HOLD_BAND = 0.001` rad, sized by rack breakaway (below it the P term commands
> less than friction, so the wheel cannot move) rather than by allowed drift.
> modelV2 noise now lives entirely in `lane_keeping`, which low-passes κ_des
> (`KAPPA_FILTER_TAU = 0.3 s`) and closes the position loop — filtering is safe
> there because its lag becomes position drift, which that loop cancels. The
> modelV2 subscription is dropped (it only fed the deleted filter's lane-change
> gate). Telemetry drops `tolerance`/`delta_err_raw`/`de_w`/`k_sigma`/`de_dc`/
> `persist_w` and gains `hold_band`.
```

- [ ] **Step 2: Update §5 (the kinematic deadzone section)**

Replace the body of `## 5. The kinematic deadzone` with:
```markdown
**DELETED 2026-07-22 (Phase 2).** The kinematic `DRIFT_M` deadzone, the
σ-observer noise floor and the sign-persistence gate are gone. They existed
because the controller was the only loop closing against drift; that job now
belongs to the `lane_keeping` position loop, which also owns modelV2 noise.

What remains is `HOLD_BAND = 0.001` rad — **not** a deadzone. P acts on the full
`delta_err` at all times; `HOLD_BAND` only decides when the rack is "on target"
so the curvature hold (§ hold_curve) can take over. It is sized by stiction:
below `FRICTION·STEER_MAX / (T_CAP_SLOPE·kappa_scale·v²)` ≈ 0.001 rad at 25 m/s
the P term commands less than rack breakaway, so the wheel cannot move anyway.

Historical note: a constant-angle deadzone was tried in 2026-05 and reverted
(route 31b, 1.3–1.7 m drift) precisely because no position-feedback layer
existed. That constraint is now satisfied — but note the resolution was to
delete the deadzone, not to widen it.
```

- [ ] **Step 3: Update the §8 action table and §9 telemetry table**

In §8, leave the action states as-is (unchanged by Phase 2) but change the `hold_zero`/`hold_curve` rows' condition from `|delta_err| ≤ tolerance` to `|delta_err| ≤ HOLD_BAND`.

In §9, delete the `tolerance`, `delta_err_raw`, `de_w`, `k_sigma`, `de_dc`, `persist_w` rows and add:
```markdown
| `hold_band` | (2026-07-22) fixed stiction hold trigger (rad) — not a deadzone; P acts on full error |
```

- [ ] **Step 4: Commit**

```bash
cd /home/oxygen/catpilot-dev/plugins
git add plugins/bmw_e9x_e8x/LATERAL_CONTROLLER.md
git commit -m "docs: Phase 2 lateral controller simplification"
```

---

## Post-plan: on-car verification (manual, not a code task)

Deploy (merge to `dev` — `install.sh` force-aligns the plugins branch to
catpilot's `dev`, so a feature branch will not reach the car), drive a
structured highway route, then evaluate against BOTH baselines with the
existing scripts (`lk_oncar.py`, the lane-offset pipeline):

| criterion | target | reference |
|---|---|---|
| driver-side gap in-band | ≫ 24% | Phase 1 (3bf) |
| lateral position std | ≤ 0.36 m | 3b7 DRIFT_M era |
| lane touches (<5 cm) | ≈ 0% | 3bf: 4.7% / 1.8% |
| torque reversals | ≤ 34/min | 3bf |
| curve residual p2p | within verified band | 393/3a9 (9.9–13.8°, best 4.9°) |
| give-ups / overturns | 0 | 3a9 |

Most likely follow-up: raising the anchor's authority (`KAPPA_BIAS_MAX`,
`T_PREVIEW`) — Phase 1 had it saturated and losing, and deleting the tolerance
subtraction only roughly doubles it. Tune from this drive's telemetry.

## Self-review notes

- **Spec coverage:** §3 reference conditioner → Tasks 1–2; §4.1 deletions →
  Task 4 steps 1/2/4/5; §4.2 three consumers → Task 4 step 3; §4.3 kept
  mechanisms → guarded by probes 5–6 and the Global Constraints; §4.4 telemetry
  → Task 4 step 5; §6 testing → Tasks 1–4 + post-plan; §5 risks → post-plan note.
- **Type consistency:** `_telem(...)` gains `kappa_in`/`kappa_ref` in Task 1 and
  is called with them from the same task; `HOLD_BAND` defined in Task 4 step 1
  before use in step 3; probe (Task 3) asserts exactly the names Task 4 produces.
- **`relax-dwell` and `hold_cap`/`HOLD_CAP_DRIFT_M` are explicitly retained** —
  Task 4 step 6's grep guards against deleting them by accident.
