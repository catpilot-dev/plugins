# Predictive Deadband Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The lane_keeping anchor decides on the *predicted* driver-side gap (line geometry at the point the car reaches after ~2× the lateral delay, minus its own commanded path) instead of the current gap — hold in-band, gentle nudge out-of-band, hard floors so prediction can defer but never mask.

**Architecture:** All control logic lands in the pure core `anchor.py` (`LaneAnchor.update` gains a `lat_delay` parameter, already plumbed through the `curvature_correction` hook but unused until now). `register.py` gains three config params and passes `lat_delay` through. A replay script (run on the C3 by the controller, non-activating) is the validation gate before the combined Phase-2 deploy.

**Tech Stack:** Python 3.11, pytest (`PYTHONPATH=. uv run pytest`), C3 replay/probe scripts (not pytest-collected).

## Global Constraints

- Spec: `docs/superpowers/specs/2026-07-23-predictive-deadband-design.md` (amends the Phase 2 spec). Branch: `phase2_simplify`.
- `plugins/lane_keeping/anchor.py` stays pure: `math` + `dataclasses` only — NO cereal/opendbc/zmq/numpy.
- Sign conventions (hard-won, do not touch): device frame `+y = right`, left ego line `laneLines[1]` at NEGATIVE y; curvature LEFT-positive; `line_sign` (left −1 / right +1), `curv_sign` (left +1 / right −1). Left-positive κ → path displacement toward **−y**: `y_path = −κ·x²/2`.
- Exact values: `pred_delay_mult = 2.0`, `LAT_DELAY_FALLBACK = 0.6` s, `X_PRED_MIN/MAX = 5.0/50.0` m, `gap_hard_lo = 0.3` m, `gap_hard_hi = 1.5` m. Param keys: `LaneKeepPredDelayMult`, `LaneKeepGapHardLo`, `LaneKeepGapHardHi`.
- Prediction must degrade gracefully: line arrays missing/short/not covering `x_pred` → `gap_pred = gap` (current-gap deadband). All pre-existing tests use single-point lane lines and MUST keep passing unchanged via this fallback.
- Existing behaviors that must not change: pure-pursuit gain/caps/rate limit/authority fade, κ_des smoothing (`kappa_filter_tau = 0.15`), lane-change hard-zero + filter bypass, MODEL-state bit-identical passthrough.
- C3 guardrails: `ssh c3` (alias only); scripts run NON-ACTIVATING from `/tmp` copies; NEVER write `/data/plugins-runtime/`, never run `install.sh`. The C3 replay run is done by the CONTROLLER, not implementer subagents.
- Full suite green before each commit: `cd /home/oxygen/catpilot-dev/plugins && PYTHONPATH=. uv run pytest plugins/ -q` (baseline `373 passed, 20 skipped`; grows with each task's new tests).
- Commit after every task. NO `Co-Authored-By` lines.

---

### Task 1: anchor.py — predicted gap, hard floors, lat_delay

**Files:**
- Modify: `plugins/lane_keeping/anchor.py`
- Test: `plugins/lane_keeping/tests/test_anchor.py`

**Interfaces:**
- Consumes: existing `AnchorConfig`, `LaneAnchor` (`line_sign`, `curv_sign`, `gap_filt`, `kappa_filt`, `_excess`, `_pursuit`, `_telem`, `update`), `_clip`, `DT_CTRL`.
- Produces:
  - `AnchorConfig` fields: `pred_delay_mult: float = 2.0`, `gap_hard_lo: float = 0.3`, `gap_hard_hi: float = 1.5`.
  - Module constants: `LAT_DELAY_FALLBACK = 0.6`, `X_PRED_MIN = 5.0`, `X_PRED_MAX = 50.0`.
  - `_interp_arr(x, xs, ys) -> float | None` (general clamped-left linear interp; `None` past the end or on bad arrays).
  - `LaneAnchor.gap_pred_filt` state (None until seeded; reset with `gap_filt`).
  - `update(curvature, model_v2, v_ego, lane_changing, lat_delay=None) -> (float, dict)`; telem gains `gap_pred`, `x_pred`.

- [ ] **Step 1: Write the failing tests**

Append to `plugins/lane_keeping/tests/test_anchor.py`:
```python
# --- predictive deadband (2026-07-23 spec) ---
# Lane lines with real geometry: y arrays over an x grid (device frame +y=right,
# so the LEFT line's y values are negative).
def _mv_geo(xs, left_ys, right_ys, left_p=1.0, right_p=1.0):
  return SimpleNamespace(
    laneLines=[SimpleNamespace(x=[], y=[0.0]),
               SimpleNamespace(x=list(xs), y=list(left_ys)),
               SimpleNamespace(x=list(xs), y=list(right_ys)),
               SimpleNamespace(x=[], y=[0.0])],
    laneLineProbs=[0.0, left_p, right_p, 0.0])


_XS = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0]


def _flat(y):
  return [y] * len(_XS)


def test_pred_parallel_line_equals_current_gap():
  # straight, parallel line, zero curvature -> gap_pred == gap; in-band -> hold
  a = LaneAnchor(AnchorConfig())
  mv = _mv_geo(_XS, _flat(-1.75), _flat(1.75))    # gap 0.84 everywhere
  out = None
  for _ in range(500):
    out, t = a.update(0.0, mv, 25.0, False, lat_delay=0.6)
  assert abs(t['gap_pred'] - 0.84) < 1e-6
  assert abs(t['x_pred'] - 30.0) < 1e-9           # 25 m/s * 2*0.6 s
  assert abs(out) < 1e-6                          # no bias, no reference


def test_pred_converging_line_nudges_early():
  # In-band NOW (gap 0.84) but the left line converges: at 30 m the gap is
  # only 0.34 -> predicted below band -> nudge AWAY from the left line
  # (steer right = negative curvature) while still in-band.
  a = LaneAnchor(AnchorConfig())
  left = [-1.75 + 0.5 * (x / 30.0) for x in _XS]  # -1.75 at car -> -1.25 at 30 m
  mv = _mv_geo(_XS, left, _flat(1.75))
  out = None
  for _ in range(2000):
    out, t = a.update(0.0, mv, 25.0, False, lat_delay=0.6)
  assert t['gap_pred'] < 0.6                      # predicted out-of-band
  assert out < -1e-5                              # nudging right (away)


def test_pred_recovering_line_no_fight():
  # Current gap below band (0.5) but diverging: at 30 m the gap is 0.9 ->
  # predicted in-band -> DO NOT nudge (0.5 is above the 0.3 hard floor).
  a = LaneAnchor(AnchorConfig())
  left = [-1.41 - 0.4 * (x / 30.0) for x in _XS]  # gap 0.5 at car -> 0.9 at 30 m
  mv = _mv_geo(_XS, left, _flat(1.75))
  out = None
  for _ in range(2000):
    out, t = a.update(0.0, mv, 25.0, False, lat_delay=0.6)
  assert t['gap_filt'] < 0.6                      # currently out-of-band
  assert t['gap_pred'] > 0.6                      # predicted back in-band
  assert abs(out) < 1e-6                          # holds: no fight with recovery


def test_pred_hard_floor_low_overrides_prediction():
  # Wheel 0.2 m from the line: prediction says recovering, but 0.2 < 0.3 hard
  # floor -> correct NOW on the current gap.
  a = LaneAnchor(AnchorConfig())
  left = [-1.11 - 0.7 * (x / 30.0) for x in _XS]  # gap 0.2 at car -> 0.9 at 30 m
  mv = _mv_geo(_XS, left, _flat(1.75))
  out = None
  for _ in range(2000):
    out, t = a.update(0.0, mv, 25.0, False, lat_delay=0.6)
  assert t['gap_filt'] < 0.3
  assert t['gap_pred'] > 0.6
  assert out < -1e-5                              # corrects away regardless


def test_pred_hard_ceiling_overrides_prediction():
  # Gap 1.6 (> 1.5 hard ceiling), prediction says coming back -> correct NOW
  # toward the driver line (steer left = positive).
  a = LaneAnchor(AnchorConfig())
  left = [-2.51 + 0.7 * (x / 30.0) for x in _XS]  # gap 1.6 at car -> 0.9 at 30 m
  mv = _mv_geo(_XS, left, _flat(0.95))
  out = None
  for _ in range(2000):
    out, t = a.update(0.0, mv, 25.0, False, lat_delay=0.6)
  assert t['gap_filt'] > 1.5
  assert t['gap_pred'] < 1.0
  assert out > 1e-5


def test_pred_curve_compensation_no_phantom_drift():
  # Left curve kappa=0.004: the line curves left (y_line = -1.75 - k*x^2/2)
  # and the car's commanded path curves with it -> predicted gap stays 0.84,
  # no phantom excess, no bias beyond the (smoothed) reference itself.
  k = 0.004
  a = LaneAnchor(AnchorConfig())
  left = [-1.75 - k * x * x / 2.0 for x in _XS]
  right = [1.75 - k * x * x / 2.0 for x in _XS]
  mv = _mv_geo(_XS, left, right)
  out = None
  for _ in range(2000):
    out, t = a.update(k, mv, 25.0, False, lat_delay=0.6)
  assert abs(t['gap_pred'] - 0.84) < 0.02
  assert abs(out - k) < 1e-4                      # reference passes, no nudge


def test_pred_right_driver_converging_nudges_left():
  # Right-side driver: right line converging -> nudge AWAY from the right
  # line = steer left = POSITIVE curvature.
  a = LaneAnchor(AnchorConfig(driver_side='right'))
  right = [1.75 - 0.5 * (x / 30.0) for x in _XS]
  mv = _mv_geo(_XS, _flat(-1.75), right)
  out = None
  for _ in range(2000):
    out, t = a.update(0.0, mv, 25.0, False, lat_delay=0.6)
  assert t['gap_pred'] < 0.6
  assert out > 1e-5


def test_pred_fallback_short_arrays_behaves_like_current_gap():
  # Old-style single-point lane lines (no x attr / 1-point y): prediction
  # falls back to the current gap — the pre-existing out-of-band behavior.
  a = LaneAnchor(AnchorConfig())
  out = _settle(a, _mv(left_y=-2.3, right_y=1.2))  # gap 1.39, above band
  assert out > 0.01 + 1e-5                         # still biases (via fallback)


def test_pred_lat_delay_scales_x_pred():
  a = LaneAnchor(AnchorConfig())
  mv = _mv_geo(_XS, _flat(-1.75), _flat(1.75))
  _o, t = a.update(0.0, mv, 25.0, False, lat_delay=0.4)
  assert abs(t['x_pred'] - 20.0) < 1e-9            # 25 * 2*0.4
  _o, t = a.update(0.0, mv, 25.0, False)           # no lat_delay -> fallback 0.6
  assert abs(t['x_pred'] - 30.0) < 1e-9
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd /home/oxygen/catpilot-dev/plugins && PYTHONPATH=. uv run pytest plugins/lane_keeping/tests/test_anchor.py -k pred -v`
Expected: FAIL — `TypeError: update() got an unexpected keyword argument 'lat_delay'` (and/or missing `gap_pred` telem key).

- [ ] **Step 3: Implement in `plugins/lane_keeping/anchor.py`**

(a) Module constants, after `DT_CTRL`:
```python
LAT_DELAY_FALLBACK = 0.6  # s; used when the hook passes no lat_delay
X_PRED_MIN = 5.0          # m; keep a meaningful prediction at crawl
X_PRED_MAX = 50.0         # m; stay inside the model's reliable line region
```

(b) General interp helper, after `_interp`:
```python
def _interp_arr(x, xs, ys):
  # clamped-left linear interpolation over ascending xs; None past the end or
  # on malformed arrays (caller falls back to the current-gap deadband)
  n = len(xs)
  if n < 2 or n != len(ys):
    return None
  if x <= xs[0]:
    return float(ys[0])
  for i in range(1, n):
    if x <= xs[i]:
      t = (x - xs[i - 1]) / (xs[i] - xs[i - 1])
      return float(ys[i - 1]) + t * (float(ys[i]) - float(ys[i - 1]))
  return None
```

(c) `AnchorConfig` fields, after `prob_fade`:
```python
  pred_delay_mult: float = 2.0   # prediction horizon = mult × lateral delay
  gap_hard_lo: float = 0.3       # current-gap floor: prediction may not defer below (m)
  gap_hard_hi: float = 1.5       # current-gap ceiling: prediction may not defer above (m)
```

(d) `__init__`: add `self.gap_pred_filt = None` beside `self.gap_filt = None`.

(e) `update` signature: `def update(self, curvature, model_v2, v_ego, lane_changing, lat_delay=None):`

(f) Inside the `if available:` branch, REPLACE the block from `if self.gap_filt is None:` through `excess = self._excess(self.gap_filt)` with:
```python
      alpha = 1.0 - math.exp(-DT_CTRL / cfg.filter_tau)
      if self.gap_filt is None:
        self.gap_filt = gap
      else:
        self.gap_filt += alpha * (gap - self.gap_filt)
      # Predictive deadband (2026-07-23 spec): evaluate the driver-side line at
      # the point the car reaches after pred_delay_mult × lat_delay (~1.2 s for
      # BMW), subtract the path its commanded curvature will trace, and decide
      # on THAT gap. A correction commanded now takes ~one lat_delay to act, so
      # 2× = one delay to act + one to observe — the human rhythm. Falls back
      # to the current gap when the line geometry can't cover x_pred.
      pred_t = cfg.pred_delay_mult * (lat_delay if lat_delay else LAT_DELAY_FALLBACK)
      x_pred = _clip(v_ego * pred_t, X_PRED_MIN, X_PRED_MAX)
      line = model_v2.laneLines[self.driver_idx]
      xs = getattr(line, 'x', [])
      y_line = _interp_arr(x_pred, [float(p) for p in xs], [float(p) for p in line.y]) \
        if len(xs) == len(line.y) else None
      if y_line is None:
        gap_pred = gap
      else:
        # left-positive curvature displaces the path toward −y (+y = right)
        y_path = -self.kappa_filt * x_pred * x_pred / 2.0 if self.kappa_filt is not None else 0.0
        gap_pred = self.line_sign * (y_line - y_path) - cfg.half_width
      if self.gap_pred_filt is None:
        self.gap_pred_filt = gap_pred
      else:
        self.gap_pred_filt += alpha * (gap_pred - self.gap_pred_filt)
      # Hard floors: the prediction may DEFER a correction, never MASK one —
      # on the paint (or drifting far toward the opposite line), correct NOW.
      if self.gap_filt < cfg.gap_hard_lo or self.gap_filt > cfg.gap_hard_hi:
        excess = self._excess(self.gap_filt)
      else:
        excess = self._excess(self.gap_pred_filt)
```

(g) In the `else:` (unavailable) branch, add `self.gap_pred_filt = None` beside the existing `self.gap_filt = None`, and set `x_pred = 0.0` before the branch so telem always has it (initialize `x_pred = 0.0` with the other zeroed locals at the top of `update`).

(h) `_telem`: add a keyword parameter `x_pred=0.0` (AFTER the existing
`kappa_in=0.0, kappa_ref=0.0`) and dict entries:
```python
      'gap_pred': float(self.gap_pred_filt) if self.gap_pred_filt is not None else 0.0,
      'x_pred': float(x_pred),
```
and change the `update` return line's call to pass it BY KEYWORD (the two
positional trailing args stay exactly as they are):
```python
    return kappa_ref + self.kappa_bias, self._telem(prob, line_y, gap, excess, authority, v_ego,
                                                    curvature, kappa_ref, x_pred=x_pred)
```

- [ ] **Step 4: Run the lane_keeping tests, then the full suite**

Run: `cd /home/oxygen/catpilot-dev/plugins && PYTHONPATH=. uv run pytest plugins/lane_keeping/tests/ -v`
Expected: ALL pass — the 24 pre-existing (fallback keeps them byte-identical in behavior) + 9 new.
Run: `PYTHONPATH=. uv run pytest plugins/ -q`
Expected: all green (`382 passed, 20 skipped`).

- [ ] **Step 5: Commit**

```bash
cd /home/oxygen/catpilot-dev/plugins
git add plugins/lane_keeping/anchor.py plugins/lane_keeping/tests/test_anchor.py
git commit -m "lane_keeping: predictive deadband — decide on the gap ~2x lat_delay ahead"
```

---

### Task 2: register.py — params + lat_delay passthrough

**Files:**
- Modify: `plugins/lane_keeping/register.py`
- Test: `plugins/lane_keeping/tests/test_register.py`

**Interfaces:**
- Consumes: `_load_config()` (`fget`, `d = AnchorConfig()`), the hook `on_curvature_correction(curvature, model_v2, v_ego, lane_changing, lat_delay=None)` which currently calls `_anchor.update(curvature, model_v2, v_ego, lane_changing)`.
- Produces: config keys `LaneKeepPredDelayMult → pred_delay_mult`, `LaneKeepGapHardLo → gap_hard_lo`, `LaneKeepGapHardHi → gap_hard_hi`; the hook passes `lat_delay` through to `update`.

- [ ] **Step 1: Write the failing tests**

Append to `plugins/lane_keeping/tests/test_register.py`:
```python
def test_load_config_predictive_params(data_dir):
  cfg = register._load_config()
  assert cfg.pred_delay_mult == 2.0
  assert cfg.gap_hard_lo == 0.3 and cfg.gap_hard_hi == 1.5
  (data_dir / 'LaneKeepPredDelayMult').write_text('3.0')
  (data_dir / 'LaneKeepGapHardLo').write_text('0.4')
  cfg2 = register._load_config()
  assert cfg2.pred_delay_mult == 3.0
  assert cfg2.gap_hard_lo == 0.4


def test_hook_passes_lat_delay_through(data_dir, monkeypatch):
  monkeypatch.setattr(register, '_PLUGIN_DIR', str(data_dir.parent))
  seen = {}
  class FakeAnchor:
    cfg = type('C', (), {'enable': True})()
    def update(self, curvature, model_v2, v_ego, lane_changing, lat_delay=None):
      seen['lat_delay'] = lat_delay
      return curvature, {}
  register._anchor = FakeAnchor()
  monkeypatch.setattr(register, '_publish', lambda telem: None)
  mv = SimpleNamespace(laneLines=[], laneLineProbs=[])
  register.on_curvature_correction(0.0, mv, 25.0, False, lat_delay=0.55)
  assert seen['lat_delay'] == 0.55
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd /home/oxygen/catpilot-dev/plugins && PYTHONPATH=. uv run pytest plugins/lane_keeping/tests/test_register.py -k "predictive or lat_delay" -v`
Expected: FAIL (params not read; `lat_delay` not forwarded → `seen['lat_delay'] is None`).

- [ ] **Step 3: Implement**

In `_load_config()`'s `AnchorConfig(...)` call, add after `prob_fade=`:
```python
    pred_delay_mult=fget('LaneKeepPredDelayMult', d.pred_delay_mult),
    gap_hard_lo=fget('LaneKeepGapHardLo', d.gap_hard_lo),
    gap_hard_hi=fget('LaneKeepGapHardHi', d.gap_hard_hi),
```

In `on_curvature_correction`, change the update call to:
```python
  new_curvature, telem = _anchor.update(curvature, model_v2, v_ego, lane_changing, lat_delay)
```

- [ ] **Step 4: Run the full suite**

Run: `cd /home/oxygen/catpilot-dev/plugins && PYTHONPATH=. uv run pytest plugins/ -q`
Expected: all green (`384 passed, 20 skipped`).

- [ ] **Step 5: Commit**

```bash
cd /home/oxygen/catpilot-dev/plugins
git add plugins/lane_keeping/register.py plugins/lane_keeping/tests/test_register.py
git commit -m "lane_keeping: predictive-deadband params + lat_delay passthrough"
```

---

### Task 3: replay validation script (authored only — controller runs it on the C3)

**Files:**
- Create: `plugins/lane_keeping/tests/replay_pred.py`

**Interfaces:**
- Consumes: `AnchorConfig`, `LaneAnchor` (with Task 1's `update(..., lat_delay=)` and telem keys `gap_filt`, `gap_pred`, `state`, `excess`).
- Produces: a standalone script (NO `test_` prefix; needs cereal + rlogs, C3-only) reporting, per route: prediction accuracy (predicted vs realized gap `pred_t` later, vs the trivial predictor), a `PRED_DELAY_MULT` sweep {1.5, 2.0, 3.0}, and decision quality (nudge-onset lead at band exits, ease-off during recoveries, added-nudge fraction on quiet segments).

**Scope limit for the implementer: write the script + AST syntax check + commit ONLY. Do NOT ssh to the C3, do NOT run it — the controller runs it as the validation gate.**

- [ ] **Step 1: Write the script**

`plugins/lane_keeping/tests/replay_pred.py`:
```python
#!/usr/bin/env python3
"""Replay gate for the predictive deadband (run on C3, non-activating).

Feeds recorded modelV2 lane lines + vEgo through LaneAnchor and reports:
  1. prediction accuracy: gap_pred(t) vs realized gap_filt(t + pred_t),
     compared against the trivial predictor gap_filt(t);
  2. a PRED_DELAY_MULT sweep {1.5, 2.0, 3.0};
  3. decision quality vs the current-gap deadband (computed from the same
     telemetry): nudge-onset lead at band exits, ease-off during recoveries,
     added-nudge fraction (prediction nudges, current holds, and no excursion
     follows within pred_t — false alarms).

Usage (on C3):
  source /usr/local/venv/bin/activate
  PYTHONPATH=/data/openpilot:/tmp/lkp python replay_pred.py 000003bf--47fd55882c [...]
"""
import sys, glob
import numpy as np
import zstandard
from cereal import log as capnp_log
from anchor import AnchorConfig, LaneAnchor

BASE = '/data/media/0/realdata'
GAP_MIN, GAP_MAX = 0.6, 1.0
LAT_DELAY = 0.6          # replay approximation of the live liveDelay value
TICK = 0.05              # modelV2 ~20 Hz in the rlog


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


def deadband(g):
  return g - min(max(g, GAP_MIN), GAP_MAX)


def run(route, mult):
  a = LaneAnchor(AnchorConfig(pred_delay_mult=mult))
  rows = []  # (gap_filt, gap_pred, v, anchored)
  for m, v, lc in frames(route):
    # feed the RECORDED desired curvature so the path-compensation term is
    # active in curves, as it will be live
    kin = float(m.action.desiredCurvature)
    _c, t = a.update(kin, m, v, lc, lat_delay=LAT_DELAY)
    rows.append((t['gap_filt'], t['gap_pred'], v, t['state'] == 'anchor'))
  return rows


for route in (sys.argv[1:] or ['000003bf--47fd55882c']):
  print(f'\n===== {route} =====')
  for mult in (1.5, 2.0, 3.0):
    rows = run(route, mult)
    gf = np.array([r[0] for r in rows]); gp = np.array([r[1] for r in rows])
    v = np.array([r[2] for r in rows]); anc = np.array([r[3] for r in rows])
    # 1. prediction accuracy: realized gap pred_t later (pred_t varies with v
    #    only through the x_pred clip; use time shift pred_t = mult*LAT_DELAY)
    shift = max(1, int(round(mult * LAT_DELAY / TICK)))
    ok = anc[:-shift] & anc[shift:]
    err_pred = gp[:-shift][ok] - gf[shift:][ok]
    err_triv = gf[:-shift][ok] - gf[shift:][ok]
    print(f'  mult={mult}: n={ok.sum()}  pred RMSE={np.sqrt(np.mean(err_pred**2)):.3f} m'
          f'  trivial RMSE={np.sqrt(np.mean(err_triv**2)):.3f} m'
          f'  (improvement {100*(1 - np.sqrt(np.mean(err_pred**2))/max(np.sqrt(np.mean(err_triv**2)),1e-9)):.0f}%)')
    if mult != 2.0:
      continue
    # 2. decision quality at mult=2.0
    cur_nudge = np.array([abs(deadband(g)) > 1e-9 for g in gf])
    prd_nudge = np.array([abs(deadband(g)) > 1e-9 for g in gp])
    # onset lead: for each current-gap band exit, how much earlier did the
    # predictor start nudging?
    leads = []
    exits = np.where((~cur_nudge[:-1]) & cur_nudge[1:] & anc[1:])[0] + 1
    for i in exits:
      j = i
      while j > 0 and prd_nudge[j - 1] and anc[j - 1]:
        j -= 1
      leads.append((i - j) * TICK)
    # ease-off: ticks where current is out-of-band but predictor already holds
    rec = cur_nudge & (~prd_nudge) & anc
    # false alarms: predictor nudges, current holds, and no current-gap exit
    # follows within pred_t
    fa = 0; tot_pn = 0
    for i in np.where(prd_nudge & (~cur_nudge) & anc)[0]:
      tot_pn += 1
      if not cur_nudge[i:i + shift].any():
        fa += 1
    print(f'    exits={len(exits)}  onset lead p50={np.median(leads) if leads else 0:.2f}s'
          f' p90={np.percentile(leads, 90) if leads else 0:.2f}s')
    print(f'    ease-off ticks (current out, predictor holds): {rec.sum()}'
          f' ({100 * rec.sum() / max(cur_nudge.sum(), 1):.0f}% of out-of-band time)')
    print(f'    predictor-only nudges: {tot_pn}  false-alarm (no exit follows): '
          f'{fa} ({100 * fa / max(tot_pn, 1):.0f}%)')
```

- [ ] **Step 2: Local syntax gate (no C3)**

Run: `cd /home/oxygen/catpilot-dev/plugins && python3 -c "import ast; ast.parse(open('plugins/lane_keeping/tests/replay_pred.py').read()); print('syntax ok')"`
Expected: `syntax ok`. Also confirm pytest does not collect it: `PYTHONPATH=. uv run pytest plugins/ -q` → same count as after Task 2 (`384 passed, 20 skipped`).

- [ ] **Step 3: Commit**

```bash
cd /home/oxygen/catpilot-dev/plugins
git add plugins/lane_keeping/tests/replay_pred.py
git commit -m "lane_keeping: predictive-deadband replay gate script"
```

---

### Task 4: lane_keeping on-device probe — predictive + smoothing coverage

**Files:**
- Modify: `plugins/lane_keeping/tests/on_device_probe.py`

**Interfaces:**
- Consumes: the probe's existing `check`/`mv`/`settle` helpers and `LK_PLUGIN_DIR` loading of `anchor.py`; Task 1's `update(..., lat_delay=)`, telem `gap_pred`/`x_pred`.
- Produces: probes for the predictive branches (parallel no-bias, converging early-nudge, recovery no-fight, hard floor, fallback) and the previously-uncovered κ_des smoothing — run offroad, non-activating.

- [ ] **Step 1: Append the probes**

Append to `plugins/lane_keeping/tests/on_device_probe.py`, BEFORE the final `print(f'\n{PASS} passed...')` / `sys.exit` lines:
```python
print('probe 8: kappa_des smoothing (previously uncovered)')
a = LaneAnchor(AnchorConfig())
none_mv = SimpleNamespace(laneLines=[], laneLineProbs=[])
a.update(0.02, none_mv, 25.0, False)
o, t = a.update(0.0, none_mv, 25.0, False)
check('reference lags a step (smoothing active)', 0.0 < o < 0.02, f'o={o:.4f}')
o, t = a.update(0.0, none_mv, 25.0, True)
check('lane change bypasses the filter', o == 0.0)

print('probe 9: predictive deadband')
XS = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0]
def mv_geo(left_ys, right_ys):
  return SimpleNamespace(
    laneLines=[SimpleNamespace(x=[], y=[0.0]),
               SimpleNamespace(x=list(XS), y=list(left_ys)),
               SimpleNamespace(x=list(XS), y=list(right_ys)),
               SimpleNamespace(x=[], y=[0.0])],
    laneLineProbs=[0.0, 1.0, 1.0, 0.0])
flat = lambda y: [y] * len(XS)

a = LaneAnchor(AnchorConfig())
out = None
for _ in range(500):
  out, t = a.update(0.0, mv_geo(flat(-1.75), flat(1.75)), 25.0, False, lat_delay=0.6)
check('parallel line: gap_pred==gap, x_pred=30, no bias',
      abs(t['gap_pred'] - 0.84) < 1e-3 and abs(t['x_pred'] - 30.0) < 1e-6 and abs(out) < 1e-5,
      f"gp={t['gap_pred']:.3f} xp={t['x_pred']:.1f} out={out:.5f}")

a = LaneAnchor(AnchorConfig())
conv = [-1.75 + 0.5 * (x / 30.0) for x in XS]
for _ in range(2000):
  out, t = a.update(0.0, mv_geo(conv, flat(1.75)), 25.0, False, lat_delay=0.6)
check('converging line: nudges early (right/negative)', t['gap_pred'] < 0.6 and out < -1e-5,
      f"gp={t['gap_pred']:.3f} out={out:.5f}")

a = LaneAnchor(AnchorConfig())
recov = [-1.41 - 0.4 * (x / 30.0) for x in XS]
for _ in range(2000):
  out, t = a.update(0.0, mv_geo(recov, flat(1.75)), 25.0, False, lat_delay=0.6)
check('recovering: current out-of-band but predictor holds',
      t['gap_filt'] < 0.6 and t['gap_pred'] > 0.6 and abs(out) < 1e-5,
      f"gf={t['gap_filt']:.3f} gp={t['gap_pred']:.3f} out={out:.5f}")

a = LaneAnchor(AnchorConfig())
crit = [-1.11 - 0.7 * (x / 30.0) for x in XS]
for _ in range(2000):
  out, t = a.update(0.0, mv_geo(crit, flat(1.75)), 25.0, False, lat_delay=0.6)
check('hard floor: 0.2m gap corrects despite recovering prediction',
      t['gap_filt'] < 0.3 and out < -1e-5, f"gf={t['gap_filt']:.3f} out={out:.5f}")

a = LaneAnchor(AnchorConfig())
for _ in range(3000):
  out, _t = a.update(0.01, mv(-2.3, 1.2), 25.0, False, lat_delay=0.6)
check('single-point lines fall back to current-gap deadband', out > 0.01 + 1e-5,
      f'out={out:.5f}')
```

- [ ] **Step 2: Local syntax gate**

Run: `cd /home/oxygen/catpilot-dev/plugins && python3 -c "import ast; ast.parse(open('plugins/lane_keeping/tests/on_device_probe.py').read()); print('syntax ok')"`
Expected: `syntax ok`. (The controller runs the probe on the C3 together with the replay gate.)

- [ ] **Step 3: Commit**

```bash
cd /home/oxygen/catpilot-dev/plugins
git add plugins/lane_keeping/tests/on_device_probe.py
git commit -m "lane_keeping: on-device probes for predictive deadband + smoothing"
```

---

## Post-plan (controller-run): validation gate, then the single combined deploy

1. Copy `plugins/lane_keeping/` to `c3:/tmp/lkp` (non-activating) and run:
   - `on_device_probe.py` → all probes pass;
   - `replay_pred.py` on 3bf + 3b7 + 3bb → prediction beats the trivial
     predictor, mult=2.0 well-placed, onset leads positive, ease-off present,
     false alarms low, quiet segments quiet.
2. Present the gate results to the user. On approval: merge `phase2_simplify`
   → `dev`, push, deploy to the C3 (`install.sh`), and run the combined on-car
   verification from the Phase 2 spec §6 plus `gap_pred` occupancy.

## Self-review notes

- **Spec coverage:** §2 prediction (geometry, 2×lat_delay, clip, fallback, κ_ref compensation) → Task 1; §3 decision + hard floors → Task 1; §4 params/telemetry → Tasks 1–2; §5.1 unit tests → Task 1; §5.2 replay gate → Task 3 + post-plan; §5.3 probes → Task 4 + post-plan.
- **Type consistency:** `update(..., lat_delay=None)` and telem keys `gap_pred`/`x_pred` match across Tasks 1/2/3/4; `_interp_arr` returns `None` past-the-end and Task 1's caller handles it; test helper `_mv_geo` and probe `mv_geo` construct `.x`/`.y` arrays the Task-1 guard expects.
- **Backward compat:** pre-existing tests use single-point lines with no `.x` → the `len(xs) == len(line.y)` guard fails → fallback `gap_pred = gap` → identical decisions; verified by keeping all 24 prior tests green in Task 1 Step 4.
- **kappa_filt note:** `y_path` uses `self.kappa_filt` (the smoothed reference state), which is None only before the first `update` tick — guarded with a 0.0 fallback in the same expression.
