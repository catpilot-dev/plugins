# BMW Self-Calibrating Hold-Bias Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a slow, regime-gated, anti-windup integral "hold-bias" to the BMW lateral controller that supplies the steady self-aligning torque in tight turns, killing the 0.6 Hz limit cycle seen on route 00000380 seg 6.

**Architecture:** Extract the hold-bias math into four pure, module-level functions in `register.py` (fully unit-testable, plain Python, no numpy), then wire them into the existing `update()` closure at three points (compute gate, add `g·b` in the `hold_zero`/`ramp` targets, integrate the bias each livePose tick). A real-data replay script provides magnitude/gating confidence before on-car testing.

**Tech Stack:** Python, pytest. Plugin: `plugins/bmw_e9x_e8x`. Controller: `register.py` `on_lat_controller_init` → nested `update()` closure.

**Reference spec:** `docs/superpowers/specs/2026-07-02-bmw-self-calibrating-hold-bias-design.md`

## Global Constraints

- **Branch:** `bmw-hold-bias` (already checked out on the plugins repo). Do not commit to `dev`.
- **Pure helpers are plain Python** — module-level, no `numpy`/`math` (those are imported *inside* the closure at `register.py:210-211`, unavailable at module scope). Use `max`/`min` for clamping.
- **Tuning constants live inside `on_lat_controller_init`** (near `T_CAP_*`/`FRICTION`), passed into the pure functions as arguments. Pure functions stay stateless.
- **`kappa_scale` and all existing tuning are untouched** — this is one isolated new mechanism.
- **Sign convention:** `hold_bias` is in the same frame as `state['torque']` (before the final `-output` flip at `register.py:642`); it integrates `state['delta_err']` with the same sign the P-term uses.
- **Starting constants (verbatim, tunable on-car):**
  `HOLD_ANGLE_ON_DEG = 20.0`, `HOLD_ANGLE_FULL_DEG = 35.0`, `HOLD_KAPPA_ON = 0.006`, `HOLD_KAPPA_FULL = 0.012`, `HOLD_KI = 0.8`, `HOLD_B_MAX = 0.20`, `HOLD_LEAK = 0.10`, `HOLD_SAT_FRAC = 0.99`.
- **Test invocation:** from `plugins/bmw_e9x_e8x/`, run `python -m pytest tests/<file> -v`. Baseline: `tests/test_hooks.py` + `tests/test_bmw.py` = 42 passing.
- **Commits:** do NOT add a `Co-Authored-By: Claude ...` trailer.

## File Structure

- **Modify** `plugins/bmw_e9x_e8x/register.py`
  - Task 1: add 4 pure module-level functions before `on_lat_controller_init` (line 161).
  - Task 2: wire them into the `update()` closure (constants, state, gate, targets, integral, telemetry).
- **Create** `plugins/bmw_e9x_e8x/tests/test_hold_bias.py` (Task 1) — unit tests for the pure functions.
- **Create** `plugins/bmw_e9x_e8x/tests/replay_hold_bias.py` (Task 3) — real-rlog replay confidence check.

---

### Task 1: Pure hold-bias functions + unit tests

**Files:**
- Modify: `plugins/bmw_e9x_e8x/register.py` (insert 4 functions before line 161)
- Test: `plugins/bmw_e9x_e8x/tests/test_hold_bias.py` (create)

**Interfaces:**
- Produces (module-level in `register`):
  - `hold_gate(steer_deg, kappa_des, angle_on, angle_full, kappa_on, kappa_full) -> float` in [0,1]
  - `hold_learn_flags(g, action, steering_pressed, overshooting, saturated) -> (bool learn_ok, bool release)`
  - `hold_bias_step(prev_b, g, delta_err, learn_ok, release, ki, b_max, leak) -> float`
  - `hold_applied(base_frac, g, b) -> float`

- [ ] **Step 1: Write the failing test file**

Create `plugins/bmw_e9x_e8x/tests/test_hold_bias.py`:

```python
"""Unit tests for the self-calibrating hold-bias pure functions (register.py).

These functions are plain-Python and stateless; importing `register` still needs
the opendbc/cereal mocks (module runs _register_interfaces at load).
"""
import os
import sys
import pytest

_PLUGIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PLUGIN_DIR not in sys.path:
  sys.path.insert(0, _PLUGIN_DIR)

from test_helpers import install_all_mocks


@pytest.fixture(autouse=True)
def mock_deps(monkeypatch):
  install_all_mocks(monkeypatch)


# Starting constants (mirror register.py HOLD_* defaults)
A_ON, A_FULL, K_ON, K_FULL = 20.0, 35.0, 0.006, 0.012
KI, B_MAX, LEAK = 0.8, 0.20, 0.10


class TestHoldGate:
  def test_below_both_thresholds_zero(self):
    import register
    assert register.hold_gate(10.0, 0.003, A_ON, A_FULL, K_ON, K_FULL) == 0.0

  def test_both_full_is_one(self):
    import register
    assert register.hold_gate(35.0, 0.012, A_ON, A_FULL, K_ON, K_FULL) == pytest.approx(1.0)

  def test_above_full_clamps_to_one(self):
    import register
    assert register.hold_gate(60.0, 0.02, A_ON, A_FULL, K_ON, K_FULL) == pytest.approx(1.0)

  def test_angle_midpoint_kappa_full(self):
    import register
    # ga = (27.5-20)/15 = 0.5 ; gk = 1.0 -> 0.5
    assert register.hold_gate(27.5, 0.012, A_ON, A_FULL, K_ON, K_FULL) == pytest.approx(0.5)

  def test_and_gate_low_curvature_closes(self):
    import register
    # high angle but curvature below threshold -> gate is 0 (AND of both)
    assert register.hold_gate(50.0, 0.003, A_ON, A_FULL, K_ON, K_FULL) == 0.0

  def test_uses_absolute_value(self):
    import register
    assert register.hold_gate(-50.0, -0.02, A_ON, A_FULL, K_ON, K_FULL) == pytest.approx(1.0)


class TestHoldLearnFlags:
  def test_holding_on_target_learns(self):
    import register
    assert register.hold_learn_flags(0.5, 'hold_zero', False, False, False) == (True, False)

  def test_ramp_learns(self):
    import register
    assert register.hold_learn_flags(0.8, 'ramp', False, False, False) == (True, False)

  def test_out_of_regime_releases(self):
    import register
    assert register.hold_learn_flags(0.0, 'hold_zero', False, False, False) == (False, True)

  def test_driver_pressed_releases(self):
    import register
    assert register.hold_learn_flags(0.5, 'ramp', True, False, False) == (False, True)

  def test_overshoot_freezes_no_release(self):
    import register
    assert register.hold_learn_flags(0.5, 'ramp', False, True, False) == (False, False)

  def test_saturated_freezes(self):
    import register
    assert register.hold_learn_flags(0.5, 'ramp', False, False, True) == (False, False)

  def test_cancel_action_freezes(self):
    import register
    assert register.hold_learn_flags(0.5, 'cancel_jerk', False, False, False) == (False, False)


class TestHoldBiasStep:
  def test_integrates_up(self):
    import register
    # 0 + 0.8*1.0*0.01 = 0.008
    assert register.hold_bias_step(0.0, 1.0, 0.01, True, False, KI, B_MAX, LEAK) == pytest.approx(0.008)

  def test_integrates_signed(self):
    import register
    assert register.hold_bias_step(0.0, 1.0, -0.01, True, False, KI, B_MAX, LEAK) == pytest.approx(-0.008)

  def test_clamps_positive(self):
    import register
    assert register.hold_bias_step(0.19, 1.0, 0.05, True, False, KI, B_MAX, LEAK) == pytest.approx(0.20)

  def test_clamps_negative(self):
    import register
    assert register.hold_bias_step(-0.19, 1.0, -0.05, True, False, KI, B_MAX, LEAK) == pytest.approx(-0.20)

  def test_freeze_holds_value(self):
    import register
    assert register.hold_bias_step(0.1, 0.5, 0.01, False, False, KI, B_MAX, LEAK) == pytest.approx(0.1)

  def test_release_leaks_toward_zero(self):
    import register
    # 0.1 + 0.1*(0-0.1) = 0.09
    assert register.hold_bias_step(0.1, 0.0, 0.0, False, True, KI, B_MAX, LEAK) == pytest.approx(0.09)

  def test_converges_and_clamps_over_iteration(self):
    import register
    b = 0.0
    for _ in range(200):
      b = register.hold_bias_step(b, 1.0, 0.005, True, False, KI, B_MAX, LEAK)
    assert b == pytest.approx(0.20)  # ramps to the clamp under sustained error

  def test_leak_decays_to_near_zero(self):
    import register
    b = 0.20
    for _ in range(100):
      b = register.hold_bias_step(b, 0.0, 0.0, False, True, KI, B_MAX, LEAK)
    assert abs(b) < 0.01


class TestHoldApplied:
  def test_scales_by_gate(self):
    import register
    assert register.hold_applied(0.0, 0.5, 0.15) == pytest.approx(0.075)

  def test_adds_to_base(self):
    import register
    assert register.hold_applied(0.1, 1.0, 0.15) == pytest.approx(0.25)

  def test_gate_zero_is_passthrough(self):
    import register
    assert register.hold_applied(0.1, 0.0, 0.15) == pytest.approx(0.1)

  def test_signed(self):
    import register
    assert register.hold_applied(-0.1, 1.0, -0.15) == pytest.approx(-0.25)
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `python -m pytest tests/test_hold_bias.py -q`
Expected: FAIL — `AttributeError: module 'register' has no attribute 'hold_gate'` (functions not defined yet).

- [ ] **Step 3: Implement the four pure functions in `register.py`**

Insert immediately before `def on_lat_controller_init(result, lac, CP):` (line 161):

```python
# ============================================================
# Self-calibrating hold-bias (tight-turn oscillation fix).
# Plain-Python, stateless, module-level so they are unit-testable.
# Spec: docs/superpowers/specs/2026-07-02-bmw-self-calibrating-hold-bias-design.md
# ============================================================

def hold_gate(steer_deg, kappa_des, angle_on, angle_full, kappa_on, kappa_full):
  """Regime gate g in [0,1]: 1 only when BOTH |steer| and |kappa_des| are well
  past their thresholds. Soft blend; decides WHERE the bias applies, not how much."""
  ga = (abs(steer_deg) - angle_on) / (angle_full - angle_on)
  gk = (abs(kappa_des) - kappa_on) / (kappa_full - kappa_on)
  ga = 0.0 if ga < 0.0 else (1.0 if ga > 1.0 else ga)
  gk = 0.0 if gk < 0.0 else (1.0 if gk > 1.0 else gk)
  return ga * gk


def hold_learn_flags(g, action, steering_pressed, overshooting, saturated):
  """Decide whether to integrate (learn_ok) or leak the bias to zero (release).
  Learn only while actively holding/tracking on-target, in-regime, not saturated,
  not overshooting, and the driver is not steering. Release (leak) out-of-regime
  or under driver override. Overshoot/saturation freeze WITHOUT releasing."""
  learn_ok = (g > 0.0) and (action in ('hold_zero', 'ramp')) \
      and (not steering_pressed) and (not overshooting) and (not saturated)
  release = (g <= 0.0) or bool(steering_pressed)
  return learn_ok, release


def hold_bias_step(prev_b, g, delta_err, learn_ok, release, ki, b_max, leak):
  """One livePose-tick update of the hold-bias integral. Release leaks toward 0;
  else integrate delta_err (self-limiting: over-push reverses delta_err and backs
  the integral down); else hold. Clamped to +/- b_max."""
  if release:
    return prev_b + leak * (0.0 - prev_b)
  if learn_ok:
    b = prev_b + ki * g * delta_err
    return -b_max if b < -b_max else (b_max if b > b_max else b)
  return prev_b


def hold_applied(base_frac, g, b):
  """Torque actually applied by the bias, ridden under the P-term target."""
  return base_frac + g * b


```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `python -m pytest tests/test_hold_bias.py -v`
Expected: PASS — all 25 tests green.

- [ ] **Step 5: Confirm no regression in the existing suite**

Run: `python -m pytest tests/test_hooks.py tests/test_bmw.py -q`
Expected: PASS — 42 passed (module-level additions don't touch existing behavior).

- [ ] **Step 6: Commit**

```bash
cd /home/oxygen/catpilot-dev/plugins
git add plugins/bmw_e9x_e8x/register.py plugins/bmw_e9x_e8x/tests/test_hold_bias.py
git commit -m "feat(bmw-lat): pure hold-bias functions (gate, learn-flags, integral, apply)"
```

---

### Task 2: Wire the hold-bias into the update() closure

**Files:**
- Modify: `plugins/bmw_e9x_e8x/register.py` (constants ~327, state ~379, gate ~420, hold_zero ~562, ramp ~588, integral ~593, telemetry ~636)

**Interfaces:**
- Consumes: `hold_gate`, `hold_learn_flags`, `hold_bias_step`, `hold_applied` (Task 1); closure locals `CS`, `state`, `g_hold`, `overshooting`, `v`, `delta_des`, `t_cap_frac`.
- Produces: `state['hold_bias']`, `state['gate']`; telemetry fields `hold_bias`, `gate`.

- [ ] **Step 1: Add the tuning constants**

In `register.py`, immediately after the `FRICTION = 0.05` line (~327), insert:

```python

  # --- Self-calibrating hold-bias (route 00000380 seg 6 oscillation fix) ---
  # Slow gated anti-windup integral that supplies the steady self-aligning
  # torque in tight turns (where SAT exceeds rack stiction and hold_zero fails).
  # Magnitude is self-calibrated (integrates measured delta_err) so it cannot
  # over-push. See spec 2026-07-02. All values tunable on-car.
  HOLD_ANGLE_ON_DEG   = 20.0     # |steeringAngleDeg| where the gate opens
  HOLD_ANGLE_FULL_DEG = 35.0     # full-strength angle
  HOLD_KAPPA_ON       = 0.006    # |kappa_des| where the gate opens (1/m)
  HOLD_KAPPA_FULL     = 0.012    # full-strength curvature
  HOLD_KI             = 0.8      # integral gain on delta_err (rad^-1 per livePose tick)
  HOLD_B_MAX          = 0.20     # clamp on |hold_bias| (torque frac, ~2.4 Nm)
  HOLD_LEAK           = 0.10     # per-tick leak toward 0 out-of-regime / driver-pressed
  HOLD_SAT_FRAC       = 0.99     # freeze integration when |torque| >= this (anti-windup)
```

- [ ] **Step 2: Add state fields**

In the `state = { ... }` dict, change the closing `'jerk_pred'` line (~379) from:

```python
    'jerk_pred': 0.0,             # debug: v²·κ_err/τ (m/s³)
  }
```

to:

```python
    'jerk_pred': 0.0,             # debug: v²·κ_err/τ (m/s³)
    'hold_bias': 0.0,             # self-calibrating hold-bias (torque frac, same frame as 'torque')
    'gate': 0.0,                  # debug: hold-bias regime gate g in [0,1]
  }
```

- [ ] **Step 3: Compute the gate each livePose tick**

Right after `state['desired'] = float(desired_curvature)` (~420), insert:

```python

      # Hold-bias regime gate: 1 only when steering angle AND curvature are both
      # significant (the regime where SAT overcomes stiction and hold_zero fails).
      g_hold = hold_gate(CS.steeringAngleDeg, state['desired'],
                         HOLD_ANGLE_ON_DEG, HOLD_ANGLE_FULL_DEG,
                         HOLD_KAPPA_ON, HOLD_KAPPA_FULL)
      state['gate'] = g_hold
```

- [ ] **Step 4: Apply the bias in the `hold_zero` branch**

Change the `hold_zero` assignment (~562) from:

```python
          else:
            target_frac = 0.0
            state['action'] = 'hold_zero'
```

to:

```python
          else:
            target_frac = hold_applied(0.0, g_hold, state['hold_bias'])
            state['action'] = 'hold_zero'
```

- [ ] **Step 5: Apply the bias in the `ramp` branch (before the t_cap clip)**

Change the cap/clip block (~586-589) from:

```python
          t_cap_nm = min(CCP.STEER_MAX,
                         T_CAP_BASE_NM + T_CAP_SLOPE_BASE * kappa_scale * v * v * abs(delta_des))
          t_cap_frac = t_cap_nm / CCP.STEER_MAX
          target_frac = float(np.clip(target_frac, -t_cap_frac, t_cap_frac))
```

to:

```python
          t_cap_nm = min(CCP.STEER_MAX,
                         T_CAP_BASE_NM + T_CAP_SLOPE_BASE * kappa_scale * v * v * abs(delta_des))
          t_cap_frac = t_cap_nm / CCP.STEER_MAX
          target_frac = hold_applied(target_frac, g_hold, state['hold_bias'])
          target_frac = float(np.clip(target_frac, -t_cap_frac, t_cap_frac))
```

- [ ] **Step 6: Integrate the bias each livePose tick**

After the decision block ends (the `state['ramp_frames'] = spread_frames` at ~593), still inside the `if livepose_updated:` block, change from:

```python
        state['target_frac'] = target_frac
        state['ramp_step'] = (target_frac - state['torque']) / spread_frames
        state['ramp_frames'] = spread_frames

    # Apply per-frame ramp step. Panda enforces wire-rate (STEER_DELTA_UP)
```

to:

```python
        state['target_frac'] = target_frac
        state['ramp_step'] = (target_frac - state['torque']) / spread_frames
        state['ramp_frames'] = spread_frames

      # Self-calibrating hold-bias integral (every livePose tick). Learn the
      # steady holding torque from delta_err while on-target in-regime; leak out
      # of regime or under driver override. Runs after the decision so
      # state['action'] reflects this tick.
      saturated = abs(state['torque']) >= HOLD_SAT_FRAC
      learn_ok, release = hold_learn_flags(g_hold, state['action'],
                                           bool(CS.steeringPressed), overshooting, saturated)
      state['hold_bias'] = hold_bias_step(state['hold_bias'], g_hold, state['delta_err'],
                                          learn_ok, release, HOLD_KI, HOLD_B_MAX, HOLD_LEAK)

    # Apply per-frame ramp step. Panda enforces wire-rate (STEER_DELTA_UP)
```

Note: `overshooting` is computed earlier in the same block (~490) and is in scope.

- [ ] **Step 7: Publish telemetry**

In the `bmw_lat_control` payload dict, change the `jerk_pred` line (~636) from:

```python
          'a_y_meas': float(state['a_y_meas']),
          'jerk_pred': float(state['jerk_pred']),
        }
```

to:

```python
          'a_y_meas': float(state['a_y_meas']),
          'jerk_pred': float(state['jerk_pred']),
          'hold_bias': float(state['hold_bias']),
          'gate': float(state['gate']),
        }
```

- [ ] **Step 8: Verify register imports and the full standalone suite passes**

Run: `python -m pytest tests/test_hooks.py tests/test_bmw.py tests/test_hold_bias.py -q`
Expected: PASS — 67 passed (42 baseline + 25 hold-bias). A syntax/scope error in the wiring surfaces here as an import failure.

- [ ] **Step 9: Byte-compile check (catches indentation/scope errors the tests can't reach in the closure)**

Run: `python -c "import py_compile; py_compile.compile('register.py', doraise=True); print('OK')"`
Expected: `OK`

- [ ] **Step 10: Commit**

```bash
cd /home/oxygen/catpilot-dev/plugins
git add plugins/bmw_e9x_e8x/register.py
git commit -m "feat(bmw-lat): wire self-calibrating hold-bias into update() closure"
```

---

### Task 3: Real-rlog replay confidence check

**Files:**
- Create: `plugins/bmw_e9x_e8x/tests/replay_hold_bias.py`

**Interfaces:**
- Consumes: `register.hold_gate`, `register.hold_bias_step` (Task 1); an rlog path (arg).
- Produces: printed `hold_bias` trajectory summary; non-zero exit if the sanity assertions fail.

This runs against the real route on C3 (`/data/media/0/realdata/00000380--bc2a2ca510--6/rlog.zst`). It confirms the gate fires only in the turn and `hold_bias` converges to the measured ~0.08–0.15 there and decays to ~0 on the straights. It does NOT model closed-loop plant response (not in the log) — that is the on-car test.

- [ ] **Step 1: Write the replay script**

Create `plugins/bmw_e9x_e8x/tests/replay_hold_bias.py`:

```python
"""Replay the hold-bias gate + integral over a real rlog to sanity-check gating
and converged magnitude. NOT a closed-loop test (plant response isn't logged).

Usage (on C3, venv active, from /data/openpilot):
  PYTHONPATH=. python /data/plugins/plugins/bmw_e9x_e8x/tests/replay_hold_bias.py \
      /data/media/0/realdata/00000380--bc2a2ca510--6/rlog.zst
"""
import os
import sys
import math
import zstandard
from cereal import log as caplog

# import the pure functions from register (mock opendbc/cereal not needed here —
# we import only the plain-Python helpers, but importing register triggers
# _register_interfaces; run under the same interpreter that has opendbc, i.e. C3).
_PLUGIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _PLUGIN_DIR)
import register

# Constants mirror register.py HOLD_* defaults.
A_ON, A_FULL, K_ON, K_FULL = 20.0, 35.0, 0.006, 0.012
KI, B_MAX, LEAK, SAT = 0.8, 0.20, 0.10, 0.99
L = 2.76  # BMW E90 wheelbase (m)


def load(path):
  with open(path, 'rb') as f:
    raw = zstandard.ZstdDecompressor().decompress(f.read(), max_output_size=200 * 1024 * 1024)
  return list(caplog.Event.read_multiple_bytes(raw))


def main(path):
  events = load(path)
  # latest carState / controlsState / livePose sampled at livePose rate
  angle = kdes = kmeas = pressed = 0.0
  t0 = None
  rows = []
  for e in events:
    w = e.which()
    if w == 'carState':
      angle = e.carState.steeringAngleDeg
      pressed = 1.0 if e.carState.steeringPressed else 0.0
    elif w == 'controlsState':
      kdes = e.controlsState.desiredCurvature
      kmeas = e.controlsState.curvature
    elif w == 'livePose':
      t = e.logMonoTime / 1e9
      if t0 is None:
        t0 = t
      d_err = math.atan(kdes * L) - math.atan(kmeas * L)
      rows.append((t - t0, angle, kdes, kmeas, d_err, pressed))

  b = 0.0
  bmax_turn = 0.0
  bmax_straight = 0.0
  for (t, ang, kd, km, d_err, pr) in rows:
    g = register.hold_gate(ang, kd, A_ON, A_FULL, K_ON, K_FULL)
    overshoot = (kd - km) * km < 0.0
    learn_ok, release = register.hold_learn_flags(g, 'ramp' if g > 0 else 'hold_zero',
                                                  bool(pr), overshoot, False)
    b = register.hold_bias_step(b, g, d_err, learn_ok, release, KI, B_MAX, LEAK)
    if 10.0 <= t <= 35.0:
      bmax_turn = max(bmax_turn, abs(b))
    if t < 6.0 or t > 50.0:
      bmax_straight = max(bmax_straight, abs(b))

  print(f"samples={len(rows)}  hold_bias peak in turn (10-35s)={bmax_turn:.3f}  "
        f"peak on straight (<6s,>50s)={bmax_straight:.3f}")
  ok_turn = 0.05 <= bmax_turn <= 0.20
  ok_straight = bmax_straight < 0.03
  print(f"turn magnitude in [0.05,0.20]: {ok_turn}   straight < 0.03: {ok_straight}")
  if not (ok_turn and ok_straight):
    print("SANITY CHECK FAILED")
    return 1
  print("SANITY CHECK PASSED")
  return 0


if __name__ == '__main__':
  sys.exit(main(sys.argv[1]))
```

- [ ] **Step 2: Run it on C3 against route 380 seg 6**

Run:
```bash
ssh c3 'source /usr/local/venv/bin/activate && cd /data/openpilot && \
  GIT_SSL_NO_VERIFY=1 python /data/plugins/plugins/bmw_e9x_e8x/tests/replay_hold_bias.py \
  /data/media/0/realdata/00000380--bc2a2ca510--6/rlog.zst'
```
(Deploy the branch to `/data/plugins` first, or scp the two files.) Expected: `SANITY CHECK PASSED`, with turn peak ~0.08–0.15 and straight peak ~0.

If the turn peak is far outside [0.05, 0.20], stop and revisit `HOLD_KI` / gate thresholds before any on-car test — the replay is the cheapest place to catch a mis-scaled bias.

- [ ] **Step 3: Commit**

```bash
cd /home/oxygen/catpilot-dev/plugins
git add plugins/bmw_e9x_e8x/tests/replay_hold_bias.py
git commit -m "test(bmw-lat): rlog replay sanity check for hold-bias gate + magnitude"
```

---

## On-car verification (post-implementation, manual)

Deploy the `bmw-hold-bias` branch to C3 and drive a route with a tight low-speed turn (route 380-like). Watch the `bmw_lat_control` telemetry and feel:
- **Primary:** the 0.6 Hz wheel oscillation in the sharp turn is gone; `hold_bias` rises to ~0.08–0.15 through the turn and decays to 0 after.
- **Regression watch (the reverted-FF failure signatures):** no over-rotation into the turn, no sustained `cancel_jerk`/`cancel_accel` bursts.
- **Elsewhere unchanged:** straights and gentle curves feel identical (gate ≈ 0, bias ≈ 0).
- **Driver override:** grabbing the wheel mid-turn yields cleanly (bias decays, no fight).
- **S-curves:** on an S with no straight between lobes, confirm `hold_bias` re-converges to the new sign without a lingering wrong-direction hold.

Tuning recipe is in the spec §9.

---

## Self-Review

**Spec coverage:**
- §5.1 applied torque (ride-on-top) → Task 2 Steps 4–5 (`hold_applied` in `hold_zero`/`ramp`). ✓
- §5.2 gate → Task 1 `hold_gate` + Task 2 Step 3. ✓
- §5.3 integral law + `learn_ok` → Task 1 `hold_bias_step`/`hold_learn_flags` + Task 2 Step 6. ✓
- §5.4 release/decay/driver-override → `release` flag in `hold_learn_flags`, leak in `hold_bias_step`, `CS.steeringPressed` wired in Task 2 Step 6. ✓
- §6 insertion points → Task 2 Steps 1–7 map to the cited lines. ✓
- §8 validation → Task 1 unit tests, Task 3 replay, on-car section. ✓
- Constants (§5.3) → Task 2 Step 1, values verbatim from Global Constraints. ✓

**Placeholder scan:** no TBD/TODO; every code and command step is concrete. ✓

**Type consistency:** function names/signatures identical across Task 1 (definition), the tests, Task 2 (call sites), and Task 3 (replay). `hold_bias`/`gate` state keys consistent. ✓

**Gap note:** the `update()` closure itself is not unit-tested (it needs a livePose `SubMaster` the existing suite never mocks). Mitigated by: all logic lives in the four tested pure functions (Task 1), the closure is thin plumbing verified by no-regression + byte-compile (Task 2 Steps 8–9), and the real-data replay (Task 3). This matches the plugin's existing test architecture.
