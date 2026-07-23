# Phase 3 — AC Stabilizer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Convert the lane_keeping anchor from a gap *positioner* into a gap *stabilizer*: a slow DC tracker concedes the model's chosen line; the pursuit machinery damps only the AC (wander) around it; the integral trim and the absolute band as decision variable are deleted.

**Architecture:** All changes live in `plugins/lane_keeping/` (anchor.py control law, register.py params) plus its tests/probes and a new replay sweep script. The Phase-2 tracker, smoothing, prediction, floors, toggle, and ring are untouched in behavior except where the spec says otherwise.

**Tech Stack:** Python 3.11, pytest (`PYTHONPATH=. uv run pytest`), C3 replay/probe scripts (controller-run, non-activating).

## Global Constraints

- Spec: `docs/superpowers/specs/2026-07-23-ac-stabilizer-design.md`. Work on branch `dev` (current tip 1a8d4bc).
- `anchor.py` stays pure (`math` + `dataclasses` only). Sign conventions untouched (`line_sign` left −1/right +1; `curv_sign` left +1/right −1; +y = right; curvature left-positive).
- Exact new values: `dc_tau = 20.0` s (param `LaneKeepDcTau`), `ac_deadband = 0.10` m (param `LaneKeepAcDeadband`).
- DELETE completely: `kappa_trim` state, `trim_rate/trim_max/trim_leak/trim_accel_max` config fields, `LaneKeepTrim*` param reads, the trim accel-clamp block, trim telemetry, all `test_trim_*` tests, probe 11.
- The absolute band `[gap_min, gap_max]` survives ONLY inside the hard-floor override (`gap_filt < 0.3` or `> 1.5` → `_excess(gap_filt)` as today).
- DC tracker discipline (spec §2.1): seed from first anchor sample of `gap_pred_filt`; adapt only when `authority > 0` and not lane-changing; FREEZE on unavailable/authority-0 (dropouts keep the reference); RESET (`None`) during lane change.
- **Behavioral pivot the tests must pin:** a STATIC scene — any constant gap, in band or out — produces ZERO correction after seeding (concession; the anti-3c1 property). Only CHANGES in the gap (drift, steps, wander) produce correction, which then decays with `dc_tau`.
- Unchanged and must stay green: MODEL bit-identical passthrough, lane-change hard-zero + filter re-seed, hard floors, smoothing, prediction fallbacks, authority fade (`prob_on 0.5`), rate limit, toggle live re-read, enforced plugin.
- Full suite before each commit: `cd /home/oxygen/catpilot-dev/plugins && PYTHONPATH=. uv run pytest plugins/ -q` (baseline `401 passed, 20 skipped`; count CHANGES as trim tests go and AC tests arrive — each task states its expected count).
- C3 guardrails: `ssh c3` alias only; probes/replay run NON-ACTIVATING from `/tmp` copies; controller (not implementers) runs anything on the C3.
- Commit after every task. NO `Co-Authored-By` lines.

---

### Task 1: anchor.py — DC tracker + AC excess, trim deletion, test rewrite

**Files:**
- Modify: `plugins/lane_keeping/anchor.py`
- Modify: `plugins/lane_keeping/tests/test_anchor.py`

**Interfaces:**
- Consumes: existing `AnchorConfig`, `LaneAnchor` (`gap_filt`, `gap_pred_filt`, `_excess`, `_pursuit`, `_telem`, `update`), `_clip`, `DT_CTRL`.
- Produces: `AnchorConfig.dc_tau=20.0`, `AnchorConfig.ac_deadband=0.10` (trim fields gone); `LaneAnchor.gap_dc` state (None until seeded; `kappa_trim` gone); telem keys `gap_dc`, `excess_ac` (key `kappa_trim` gone); `update()` returns `kappa_ref + kappa_bias` (no trim term).

- [ ] **Step 1: Write the new failing tests (and delete the obsolete ones)**

In `plugins/lane_keeping/tests/test_anchor.py`:

(a) DELETE these tests entirely (they pin the deleted positioner/trim behavior):
`test_trim_accumulates_below_band_correct_direction`, `test_trim_caps`,
`test_trim_leaks_in_band`, `test_trim_unwinds_on_opposite_error_no_ratchet`,
`test_trim_zeroed_on_lane_change`, `test_trim_added_to_output`,
`test_trim_right_driver_mirror`, `test_trim_leaks_when_authority_zero`,
`test_trim_leaks_when_line_lost`, `test_trim_speed_accel_clamp`,
`test_disabled_retires_trim_at_trim_rate`,
`test_update_biases_left_when_too_far_from_left_line`,
`test_update_biases_right_when_too_close_to_left_line`,
`test_pred_converging_line_nudges_early`,
`test_pred_recovering_line_no_fight`,
`test_pred_right_driver_converging_nudges_left`,
`test_pred_plan_drift_toward_line_nudges`.

(b) MODIFY `test_update_lane_change_hard_zeros_established_bias` and
`test_update_rate_limited` — they built bias from a constant out-of-band gap,
which now concedes. Replace both with the versions inside the new block below
(`test_lane_change_hard_zeros_bias_built_from_drift`, `test_rate_limited_on_step`).

(c) APPEND this block (uses a fast `dc_tau` where waiting for 20 s is impractical):

```python
# --- Phase 3: AC stabilizer (damp the wander, concede the line) ---

def _mv_at(gap):
  # left-driver scene with the LEFT line placed to give the requested gap
  y = -(gap + 0.91)
  return _mv_geo(_XS, _flat(y), _flat(1.75))


def _run(a, gap, n, v=17.0, lc=False):
  out = t = None
  for _ in range(n):
    out, t = a.update(0.0, _mv_at(gap), v, lc, lat_delay=0.6)
  return out, t


def test_constant_offset_is_conceded_anywhere():
  # THE anti-3c1 regression test: a static gap — even far out of the old
  # band — produces ZERO correction after seeding. The line is the model's.
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
  for g in (1.39, 0.45, 0.84):
    a2 = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
    out, t = _run(a2, g, 3000)
    assert abs(out) < 1e-6, f'gap {g} not conceded'
    assert abs(t['excess_ac']) < 0.02
    assert abs(t['gap_dc'] - g) < 0.05          # DC tracked the line


def test_drift_is_damped():
  # A drifting gap (the integrated sub-Hz wander) IS corrected while it
  # drifts: gap rising = car moving away from the left line (rightward)
  # -> damp with positive (leftward) curvature.
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
  _run(a, 0.84, 1000)                            # seed DC at 0.84
  out = None
  for i in range(600):                           # 6 s drift at ~0.05 m/s
    g = 0.84 + 0.05 * (i / 100.0)
    out, t = a.update(0.0, _mv_at(g), 17.0, False, lat_delay=0.6)
  assert out > 1e-5                              # damping the motion, leftward
  assert t['excess_ac'] > 0.1                    # beyond the AC deadband


def test_step_damped_then_conceded():
  # A step disturbance is resisted transiently, then forgotten with dc_tau.
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0, dc_tau=2.0))   # fast tau for test
  _run(a, 0.84, 1000)                            # settle
  out_peak = 0.0
  for _ in range(300):                           # 3 s after step to gap 1.2
    out, _t = a.update(0.0, _mv_at(1.2), 17.0, False, lat_delay=0.6)
    out_peak = max(out_peak, out)
  assert out_peak > 1e-5                         # transient resistance fired
  out, t = _run(a, 1.2, 2000)                    # 20 s >> dc_tau -> conceded
  assert abs(out) < 1e-6
  assert abs(t['gap_dc'] - 1.2) < 0.05


def test_ac_deadband_ignores_micro_noise():
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
  _run(a, 0.84, 1000)
  out = None
  for i in range(400):                           # ±0.05 m oscillation < deadband 0.10
    g = 0.84 + 0.05 * (1 if (i // 50) % 2 else -1)
    out, _t = a.update(0.0, _mv_at(g), 17.0, False, lat_delay=0.6)
  assert abs(out) < 1e-6


def test_lane_change_resets_dc_and_concedes_new_lane():
  # After a lane change the DC re-seeds: the new lane's position — whatever
  # it is — is immediately the reference. No settle-nudge, no old-lane memory.
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
  _run(a, 0.84, 1000)
  a.update(0.0, _mv_at(0.5), 17.0, True, lat_delay=0.6)    # LC tick, new lane geometry
  assert a.gap_dc is None                        # reset during LC
  out, t = _run(a, 0.5, 500)                     # post-LC: 0.5 is the new normal
  assert abs(out) < 1e-6
  assert abs(t['gap_dc'] - 0.5) < 0.05


def test_dc_freezes_when_untrusted():
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
  _run(a, 0.84, 1000)
  dc0 = a.gap_dc
  mv_low = _mv_geo(_XS, _flat(-1.75), _flat(1.75), left_p=0.3)   # authority 0
  for _ in range(500):
    a.update(0.0, mv_low, 17.0, False, lat_delay=0.6)
  assert a.gap_dc == dc0                         # frozen, not adapted/reset
  none_mv = SimpleNamespace(laneLines=[], laneLineProbs=[])
  for _ in range(500):
    a.update(0.0, none_mv, 17.0, False, lat_delay=0.6)
  assert a.gap_dc == dc0                         # dropouts keep the reference


def test_hard_floors_still_absolute():
  # At the extremes the ABSOLUTE band still governs — even though the DC
  # would concede, a 0.2 m gap keeps a sustained (best-effort) push away.
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
  out, _t = _run(a, 0.2, 3000)
  assert out < -1e-5                             # steering right, sustained
  a2 = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
  out, _t = _run(a2, 1.6, 3000)
  assert out > 1e-5                              # ceiling: steering left


def test_lane_change_hard_zeros_bias_built_from_drift():
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0))
  _run(a, 0.84, 1000)
  for i in range(600):                           # build bias from a drift
    a.update(0.0, _mv_at(0.84 + 0.05 * (i / 100.0)), 17.0, False, lat_delay=0.6)
  assert a.kappa_bias > 1e-5
  out, t = a.update(0.01, _mv_at(1.14), 17.0, True, lat_delay=0.6)
  assert a.kappa_bias == 0.0                     # hard-zeroed on the LC tick
  assert out == 0.01                             # bit-identical passthrough
  assert t['state'] == 'model'


def test_rate_limited_on_step():
  a = LaneAnchor(AnchorConfig(pred_delay_mult=2.0, kappa_rate_max=0.002))
  _run(a, 0.84, 1000)
  a.gap_filt = 1.3                               # warm the filters into a step
  a.gap_pred_filt = 1.3
  _o, t = a.update(0.0, _mv_at(1.3), 17.0, False, lat_delay=0.6)
  assert abs(t['kappa_bias']) <= 0.002 * 0.01 + 1e-12
```

- [ ] **Step 2: Run to verify the new tests fail**

Run: `cd /home/oxygen/catpilot-dev/plugins && PYTHONPATH=. uv run pytest plugins/lane_keeping/tests/test_anchor.py -k "conceded or drift or step or deadband_ignores or resets_dc or freezes_when or floors_still or built_from_drift or rate_limited_on_step" -v`
Expected: FAILURES — `AnchorConfig` has no `dc_tau`; telem has no `gap_dc`/`excess_ac`; constant out-of-band gaps still produce bias.

- [ ] **Step 3: Implement in `plugins/lane_keeping/anchor.py`**

(a) `AnchorConfig`: DELETE the four `trim_*` fields and their comment block; ADD after `kappa_filter_tau`'s block:
```python
  dc_tau: float = 20.0           # DC-tracker time constant (s) — the forgetting time.
                                 # The stabilizer concedes any disagreement older than
                                 # ~this: zero-mean correction by construction, so the
                                 # 3c1 arm-wrestle (model counter-steers a sustained
                                 # bias to a stalemate) is structurally impossible.
  ac_deadband: float = 0.10      # AC excess ignored below this (m) — micro-noise
```

(b) `__init__`: replace `self.kappa_trim = 0.0` with `self.gap_dc = None`.

(c) `_telem`: remove the `'kappa_trim'` entry; add parameters `gap_dc=0.0, excess_ac=0.0` (after `x_pred=0.0`) and entries:
```python
      'gap_dc': float(gap_dc), 'excess_ac': float(excess_ac),
```

(d) In `update()`, the available branch: REPLACE everything from the hard-floor
`if`/`else` through the end of the trim integrate/leak block with (note the
authority computation MOVES ABOVE this block — cut its two lines from below
and place them after the `gap_pred_filt` update):
```python
      authority = _interp(prob, [cfg.prob_on, cfg.prob_on + cfg.prob_fade], [0.0, 1.0])
      if lane_changing:
        authority = 0.0
      # DC tracker (Phase 3): adiabatically follow the model's chosen line.
      # Seeds on the first anchor sample; adapts only while the measurement is
      # trusted; FROZEN on low authority (dropouts keep the reference); RESET
      # by a lane change (new lane, new line identity, new DC).
      if lane_changing:
        self.gap_dc = None
      elif self.gap_dc is None:
        self.gap_dc = self.gap_pred_filt
      elif authority > 0.0:
        a_dc = 1.0 - math.exp(-DT_CTRL / cfg.dc_tau)
        self.gap_dc += a_dc * (self.gap_pred_filt - self.gap_dc)
      # Decision: hard floors are ABSOLUTE (best-effort at the extremes);
      # otherwise damp only the AC — the deviation from the tracked line.
      # Zero-mean by construction: a static scene, at ANY gap, concedes.
      excess_ac = (self.gap_pred_filt - self.gap_dc) if self.gap_dc is not None else 0.0
      if self.gap_filt < cfg.gap_hard_lo or self.gap_filt > cfg.gap_hard_hi:
        excess = self._excess(self.gap_filt)
      else:
        excess = excess_ac - _clip(excess_ac, -cfg.ac_deadband, cfg.ac_deadband)
        excess = _clip(excess, -cfg.excess_max, cfg.excess_max)
      kappa_target = authority * self._pursuit(excess, v_ego)
```
Also initialize `excess_ac = 0.0` and `gap_dc_t = 0.0` alongside the other
zeroed locals at the top of `update()` is NOT needed — instead, for telemetry,
pass at the return site: `gap_dc=self.gap_dc if self.gap_dc is not None else 0.0`
and `excess_ac=excess_ac` (define `excess_ac = 0.0` in the top-of-update zeroed
locals so the unavailable path has it).

(e) Unavailable branch: delete the trim leak/retire lines (keep the gap-filter
resets and `kappa_target = 0.0`). Do NOT touch `self.gap_dc` there (freeze).

(f) Delete the trim speed-clamp block (`cap_eff = ...` two lines + comment)
after the branches.

(g) Lane-change hard-zero: remove `self.kappa_trim = 0.0` (keep the bias zero).

(h) Return: `return kappa_ref + self.kappa_bias, self._telem(prob, line_y, gap, excess, authority, v_ego, curvature, kappa_ref, x_pred=x_pred, gap_dc=self.gap_dc if self.gap_dc is not None else 0.0, excess_ac=excess_ac)`

(i) Verify no dangling trim references:
`grep -n "trim" plugins/lane_keeping/anchor.py` → only prose in comments if any; no code.

- [ ] **Step 4: Run the suite**

Run: `PYTHONPATH=. uv run pytest plugins/lane_keeping/tests/test_anchor.py -v` — ALL pass (pre-existing minus 17 deleted, plus 9 new).
Then `PYTHONPATH=. uv run pytest plugins/ -q` — expect a FAILURE only in `test_register.py` (`test_load_config_trim_params`, and possibly `test_live_toggle_disables_and_releases` timing) — those are Task 2's to fix; note the count.

- [ ] **Step 5: Commit**

```bash
git add plugins/lane_keeping/anchor.py plugins/lane_keeping/tests/test_anchor.py
git commit -m "lane_keeping: AC stabilizer — damp the wander, concede the line (trim deleted)"
```

---

### Task 2: register.py — params swap + test updates

**Files:**
- Modify: `plugins/lane_keeping/register.py`
- Modify: `plugins/lane_keeping/tests/test_register.py`

**Interfaces:**
- Consumes: Task 1's `AnchorConfig` (`dc_tau`, `ac_deadband`; no trim fields).
- Produces: `_load_config()` reads `LaneKeepDcTau → dc_tau`, `LaneKeepAcDeadband → ac_deadband`; the four `LaneKeepTrim*` reads are gone.

- [ ] **Step 1: Update the tests**

In `plugins/lane_keeping/tests/test_register.py`:
- DELETE `test_load_config_trim_params`.
- APPEND:
```python
def test_load_config_ac_params(data_dir):
  cfg = register._load_config()
  assert cfg.dc_tau == 20.0 and cfg.ac_deadband == 0.10
  (data_dir / 'LaneKeepDcTau').write_text('30')
  assert register._load_config().dc_tau == 30.0
```
- REWRITE `test_live_toggle_disables_and_releases` (its constant out-of-band
scene no longer builds bias). Replace its body with:
```python
def test_live_toggle_disables_and_releases(data_dir, monkeypatch):
  monkeypatch.setattr(register, '_PLUGIN_DIR', str(data_dir.parent))
  monkeypatch.setattr(register, '_publish', lambda telem: None)
  def mv_at(gap):
    y = -(gap + 0.91)
    xs = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0]
    return SimpleNamespace(
      laneLines=[SimpleNamespace(x=[], y=[0.0]), SimpleNamespace(x=xs, y=[y]*6),
                 SimpleNamespace(x=xs, y=[1.75]*6), SimpleNamespace(x=[], y=[0.0])],
      laneLineProbs=[0.0, 1.0, 1.0, 0.0],
      position=SimpleNamespace(x=xs, y=[0.0]*6))
  for _ in range(1000):                          # seed DC at 0.84
    register.on_curvature_correction(0.0, mv_at(0.84), 17.0, False, lat_delay=0.45)
  out = None
  for i in range(600):                           # drift -> stabilizer damps
    out = register.on_curvature_correction(0.0, mv_at(0.84 + 0.05*(i/100.0)), 17.0, False, lat_delay=0.45)
  assert out > 1e-5
  (data_dir / 'LaneKeepEnable').write_text('0')
  for i in range(500):                           # toggle off mid-drift: releases
    out = register.on_curvature_correction(0.0, mv_at(1.14 + 0.05*(i/100.0)), 17.0, False, lat_delay=0.45)
  assert abs(out) < 1e-6
```

- [ ] **Step 2: Verify current failure, implement**

Run: `PYTHONPATH=. uv run pytest plugins/lane_keeping/tests/test_register.py -v` → the new/rewritten tests FAIL (params not wired).
In `_load_config()`: DELETE the four `trim_*=fget('LaneKeepTrim*', ...)` lines; ADD:
```python
    dc_tau=fget('LaneKeepDcTau', d.dc_tau),
    ac_deadband=fget('LaneKeepAcDeadband', d.ac_deadband),
```

- [ ] **Step 3: Full suite green, commit**

Run: `PYTHONPATH=. uv run pytest plugins/ -q` — ALL green (record the new total).
```bash
git add plugins/lane_keeping/register.py plugins/lane_keeping/tests/test_register.py
git commit -m "lane_keeping: LaneKeepDcTau/AcDeadband params, trim params removed"
```

---

### Task 3: on-device probe rework (author only — controller runs it)

**Files:**
- Modify: `plugins/lane_keeping/tests/on_device_probe.py`

**Scope limit: edit + AST check + pytest-non-collection check + commit ONLY. No ssh.**

- [ ] **Step 1: Replace probe 11 (trim) with the AC probes**

DELETE the whole `probe 11: integral trim` block. In its place:
```python
print('probe 11: AC stabilizer (concede the line, damp the wander)')
def mv_at(gap):
  y = -(gap + 0.91)
  return mv_geo(flat(y), flat(1.75))
a = LaneAnchor(AnchorConfig())
out = None
for _ in range(3000):
  out, t = a.update(0.0, mv_at(1.39), 17.0, False, lat_delay=0.6)
check('constant out-of-band gap is CONCEDED (anti-3c1)', abs(out) < 1e-6 and abs(t['gap_dc'] - 1.39) < 0.05,
      f"out={out:.6f} dc={t['gap_dc']:.2f}")
for i in range(600):
  out, t = a.update(0.0, mv_at(1.39 + 0.05 * (i / 100.0)), 17.0, False, lat_delay=0.6)
check('drift is damped (positive/leftward vs rightward drift)', out > 1e-5, f'out={out:.6f}')
a2 = LaneAnchor(AnchorConfig())
out, _t = None, None
for _ in range(3000):
  out, _t = a2.update(0.0, mv_at(0.2), 17.0, False, lat_delay=0.6)
check('hard floor still absolute at 0.2 m', out < -1e-5, f'out={out:.6f}')
```

- [ ] **Step 2: Local gates + commit**

`python3 -c "import ast; ast.parse(open('plugins/lane_keeping/tests/on_device_probe.py').read()); print('syntax ok')"` → `syntax ok`; full suite unchanged from Task 2's count.
```bash
git add plugins/lane_keeping/tests/on_device_probe.py
git commit -m "lane_keeping: AC-stabilizer probes replace trim probe"
```

---

### Task 4: replay sweep script (author only — controller runs it on the C3)

**Files:**
- Create: `plugins/lane_keeping/tests/replay_ac.py`

**Scope limit: write + AST check + non-collection check + commit ONLY. No ssh.**

- [ ] **Step 1: Write the script**

`plugins/lane_keeping/tests/replay_ac.py`:
```python
#!/usr/bin/env python3
"""DC_TAU sweep for the AC stabilizer (run on C3, non-activating).

Per route x tau: (1) sustained-correction residue — mean |bias| over 30 s
windows on out-of-band stretches (the anti-arm-wrestle gate; the trim sat
pinned at cap here); (2) wander capture — how much of the gap's 0.03-0.16 Hz
band variance appears in excess_ac; (3) correction occupancy on quiet
in-band stretches (must not exceed a few percent).

Usage: PYTHONPATH=/data/openpilot:/tmp/lkp3 python replay_ac.py ROUTE [...]
"""
import sys, glob
import numpy as np
import zstandard
from cereal import log as capnp_log
import anchor as anchor_mod
anchor_mod.DT_CTRL = 0.05                 # replay steps at modelV2 ~20 Hz — exact fix
from anchor import AnchorConfig, LaneAnchor

BASE = '/data/media/0/realdata'
TICK = 0.05

def frames(route):
  for seg in sorted(int(p.rsplit('--', 1)[1]) for p in glob.glob(f'{BASE}/{route}--*')):
    try:
      raw = zstandard.ZstdDecompressor().decompress(
        open(f'{BASE}/{route}--{seg}/rlog.zst', 'rb').read(), max_output_size=2**31)
    except (FileNotFoundError, zstandard.ZstdError):
      continue
    v = 0.0
    for evt in capnp_log.Event.read_multiple_bytes(raw):
      w = evt.which()
      if w == 'carState':
        v = float(evt.carState.vEgo)
      elif w == 'modelV2':
        m = evt.modelV2
        yield m, v, str(m.meta.laneChangeState) != 'off'

def band_var(x, lo=0.03, hi=0.16):
  x = np.asarray(x) - np.mean(x)
  f = np.fft.rfftfreq(len(x), TICK)
  X = np.abs(np.fft.rfft(x)) ** 2
  sel = (f >= lo) & (f <= hi)
  return X[sel].sum() / len(x)

for route in sys.argv[1:]:
  print(f'\n===== {route} =====')
  for tau in (10.0, 20.0, 30.0):
    a = LaneAnchor(AnchorConfig(dc_tau=tau))
    rows = []
    for m, v, lc in frames(route):
      out, t = a.update(float(m.action.desiredCurvature), m, v, lc, lat_delay=0.6)
      rows.append((t['gap_filt'], t['excess_ac'], t['kappa_bias'], t['state'] == 'anchor'))
    gf = np.array([r[0] for r in rows]); ac = np.array([r[1] for r in rows])
    kb = np.array([r[2] for r in rows]); anc = np.array([r[3] for r in rows])
    if not anc.any():
      print(f'  tau={tau}: no anchor ticks'); continue
    # 1. sustained residue on out-of-band stretches (old-band definition)
    oob = anc & ((gf < 0.6) | (gf > 1.0))
    W = int(30.0 / TICK)
    res = [np.abs(np.mean(kb[i:i+W])) for i in range(0, len(kb) - W, W)
           if oob[i:i+W].mean() > 0.8]
    # 2. wander capture on long anchored runs
    runs, s = [], None
    for i, f_ in enumerate(anc):
      if f_ and s is None: s = i
      elif not f_ and s is not None:
        if i - s > 1200: runs.append((s, i))
        s = None
    caps = [band_var(ac[s:e]) / max(band_var(gf[s:e]), 1e-12) for s, e in runs]
    # 3. quiet occupancy
    inb = anc & (gf >= 0.6) & (gf <= 1.0)
    occ = np.mean(np.abs(kb[inb]) > 1e-5) if inb.any() else 0.0
    print(f'  tau={tau}: residue(30s windows) p50={np.median(res) if res else 0:.6f} '
          f'max={max(res) if res else 0:.6f} (gate <2e-4)  '
          f'wander-capture p50={np.median(caps) if caps else 0:.2f}  '
          f'in-band correction occupancy={occ*100:.1f}%')
```

- [ ] **Step 2: Local gates + commit**

AST check `syntax ok`; suite count unchanged.
```bash
git add plugins/lane_keeping/tests/replay_ac.py
git commit -m "lane_keeping: DC_TAU replay sweep for the AC stabilizer"
```

---

## Post-plan (controller-run): validation gate → deploy on user approval

1. Copy `plugins/lane_keeping/` to `c3:/tmp/lkp3` (non-activating); run
   `on_device_probe.py` (all probes) and `replay_ac.py` on **3c1** (the
   adversarial case), 3c0, 3bf, and a quiet route (3b7).
2. Gates: 3c1 out-of-band residue ≈ 0 (< 2e-4 in every 30 s window — where the
   trim sat pinned at 78% of cap); wander capture meaningful (> ~0.5 at the
   chosen τ); quiet-route in-band occupancy ≤ a few %. Pick `DC_TAU` from the
   sweep; update the default if ≠ 20.
3. Review pass on the full diff (standing reviewer), then present the numbers
   to the user; merge/push/deploy only on their go; the drive A/B (toggle)
   is the closed-loop gate per spec §5.

## Self-review notes

- **Spec coverage:** §2 law/DC discipline → Task 1; §2.2 floors → Task 1 (+test); §2.3 deletions → Tasks 1–3; §3 params/telemetry → Tasks 1–2; §4 sweep/metrics → Task 4 + post-plan; §5 on-car gate → post-plan.
- **Behavioral pivot pinned:** concession (static-anywhere), damping (drift), forgetting (step→decay), floors-absolute, LC reset, freeze rules — each has a dedicated test; the anti-3c1 concession test is the headline.
- **Type consistency:** `dc_tau`/`ac_deadband` names match across config/register/tests/replay; telem `gap_dc`/`excess_ac` produced in Task 1, consumed in Task 4's replay; `_mv_at` helper defined in the test block that uses it; probe's `mv_at` defined locally.
- **Known lesson applied:** replay patches `DT_CTRL` to the 20 Hz cadence (the Task-3-round-1 Critical from the predictive deadband).
