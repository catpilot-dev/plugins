# Jerk-Limited Allowed-Speed Ceiling — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace speedlimitd's fixed-time display ladder with a jerk-limited ramp of the allowed-speed ceiling inside `planner_hook`, so a non-safety limit drop produces one smooth slowdown instead of three brake spikes.

**Architecture:** The daemon (`speedlimitd.py`, 5 Hz) stops shaping transitions and publishes the true fused limit; the planner hook (`planner_hook.py`, 20 Hz) owns a `_ceiling_ms` state that slides toward the offset-adjusted target under a trapezoidal acceleration profile. The ceiling is a pure function of the limit history — it never reads `v_ego` or `v_cruise` — so the trajectory is deterministic. Safety caps assign the ceiling directly, bypassing the ramp.

**Tech Stack:** Python 3, pytest, `uv`. No new dependencies.

**Spec:** `docs/superpowers/specs/2026-09-10-speedlimitd-jerk-limited-ceiling-design.md`

## Global Constraints

- Repo: `/home/oxygen/catpilot-dev/plugins`, branch `dev`.
- Run all tests from the repo root with a **cleared** `PYTHONPATH`: `PYTHONPATH= uv run pytest <path> -q`. A foreign `PYTHONPATH` in the session env silently shadows the repo's namespace package and tests the wrong worktree.
- Indentation in this codebase is **2 spaces**, not 4. Match the surrounding file.
- Constants, exact values, copied verbatim from the spec:
  - `CEIL_A_DOWN = 0.8` (m/s²), `CEIL_J_DOWN = 0.5` (m/s³)
  - `CEIL_A_UP = 1.5` (m/s²), `CEIL_J_UP = 1.0` (m/s³)
  - `CEIL_SNAP_MS = 0.05` (m/s), `CEIL_SNAP_RATE = 0.05` (m/s²), `CEIL_DT_MAX = 0.2` (s)
- Offset tiers are **unchanged**: +15% below 80 km/h, +10% at/above, +0% when `safetyCapped`.
- The hook must never return a value above the incoming `v_cruise`. This invariant predates this work and must survive it.
- Commit messages end with `Claude-Session: https://claude.ai/code/session_01L53Y95zG636KXqrdkaA2TQ` and carry **no** `Co-Authored-By` line.

## File Structure

| File | Responsibility |
|------|----------------|
| `plugins/speedlimitd/planner_hook.py` | Owns the ceiling. New pure helper `_advance_ceiling` (the profile integrator, no I/O) plus module state wired into `on_v_cruise`. |
| `plugins/speedlimitd/speedlimitd.py` | Perception/fusion only. Ladder machinery deleted. |
| `plugins/speedlimitd/tests/test_planner_hook.py` | **New.** Everything about the ramp: the pure profile, then its integration into `on_v_cruise`. |
| `plugins/speedlimitd/tests/test_speedlimitd.py` | Existing suite. Two daemon ladder tests reworked; the `TestPlannerHook` tests that re-target a limit mid-test need a clock. |
| `plugins/speedlimitd/DESIGN.md` | Two sections rewritten. |

`README.md` is untouched — its only "ramp" mentions are the curve-approach profile and the on/off-ramp road class.

---

### Task 1: The profile integrator (`_advance_ceiling`)

A pure function: no bus, no `sm`, no clock. This is the whole of the new maths, and it is testable in isolation.

**Files:**
- Modify: `plugins/speedlimitd/planner_hook.py` (add constants after `SOURCE_ROAD_TYPE_INFERENCE`, add helper after `_effective_offset_percent`)
- Test: `plugins/speedlimitd/tests/test_planner_hook.py` (create)

**Interfaces:**
- Consumes: nothing from earlier tasks.
- Produces: `_advance_ceiling(ceiling_ms: float, rate_ms2: float, target_ms: float, dt: float) -> tuple[float, float]` returning `(new_ceiling_ms, new_rate_ms2)`. Task 2 calls this once per tick.

- [ ] **Step 1: Write the failing tests**

Create `plugins/speedlimitd/tests/test_planner_hook.py`. The `sys.path` preamble mirrors `test_speedlimitd.py` — the tests import `plugins.speedlimitd.planner_hook` as a package, which needs the repo root on `sys.path`.

```python
"""Tests for planner_hook — the jerk-limited allowed-speed ceiling.

Spec: docs/superpowers/specs/2026-09-10-speedlimitd-jerk-limited-ceiling-design.md
"""
import os
import sys
import importlib
import pytest
from unittest.mock import MagicMock

_PLUGINS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
if _PLUGINS_DIR not in sys.path:
  sys.path.insert(0, _PLUGINS_DIR)
_REPO_ROOT = os.path.dirname(_PLUGINS_DIR)
if _REPO_ROOT not in sys.path:
  sys.path.insert(0, _REPO_ROOT)


@pytest.fixture
def ph():
  """planner_hook with openpilot's CV stubbed, module state cleared."""
  mock_cv = MagicMock()
  mock_cv.KPH_TO_MS = 1.0 / 3.6
  mock_cv.MS_TO_KPH = 3.6
  sys.modules['openpilot.common'] = MagicMock()
  sys.modules['openpilot.common.constants'] = MagicMock(CV=mock_cv)

  import plugins.speedlimitd.planner_hook as mod
  importlib.reload(mod)
  mod._sl_sub = None
  mod._sl_data = None
  mod._baseline_ms = None
  mod._gas_floor_ms = None
  mod._road_id = ''
  mod._ceiling_ms = None
  mod._ceiling_rate = 0.0
  mod._last_t = None
  return mod


DT = 0.05  # plannerd runs on modelV2 — DT_MDL


def _trace(ph, start_kph, target_kph, dt=DT, max_s=120.0):
  """Advance the ceiling to the target, returning [(t, ceiling_ms, rate_ms2)]."""
  ceiling = start_kph / 3.6
  target = target_kph / 3.6
  rate = 0.0
  out = [(0.0, ceiling, rate)]
  t = 0.0
  while t < max_s:
    ceiling, rate = ph._advance_ceiling(ceiling, rate, target, dt)
    t += dt
    out.append((t, ceiling, rate))
    if ceiling == target and rate == 0.0:
      break
  return out


class TestAdvanceCeiling:
  def test_jerk_bound_on_descent(self, ph):
    """No tick may change the rate by more than J_DOWN * dt. This is the point
    of the whole design: DCC reacts to setpoint gap, so the slope must be
    continuous, not just the value."""
    tr = _trace(ph, 88.0, 46.0)
    for (_, _, r0), (_, _, r1) in zip(tr, tr[1:]):
      assert abs(r1 - r0) <= ph.CEIL_J_DOWN * DT + 1e-9

  def test_value_continuity_on_descent(self, ph):
    """No tick may change the ceiling by more than A_DOWN * dt."""
    tr = _trace(ph, 88.0, 46.0)
    for (_, c0, _), (_, c1, _) in zip(tr, tr[1:]):
      assert abs(c1 - c0) <= ph.CEIL_A_DOWN * DT + 1e-9

  def test_descent_is_monotonic_and_does_not_overshoot(self, ph):
    tr = _trace(ph, 88.0, 46.0)
    target = 46.0 / 3.6
    for (_, c0, _), (_, c1, _) in zip(tr, tr[1:]):
      assert c1 <= c0 + 1e-9, 'ceiling rose during a descent'
    for _, c, _ in tr:
      assert c >= target - 1e-9, 'ceiling undershot the target'

  def test_arrives_with_zero_slope(self, ph):
    tr = _trace(ph, 88.0, 46.0)
    _, c_end, r_end = tr[-1]
    assert c_end == pytest.approx(46.0 / 3.6, abs=1e-9)
    assert r_end == 0.0

  def test_peak_descent_rate_respects_a_down(self, ph):
    tr = _trace(ph, 88.0, 46.0)
    assert min(r for _, _, r in tr) >= -ph.CEIL_A_DOWN - 1e-9

  def test_80_to_40_lands_in_the_expected_window(self, ph):
    """Spec: ~15 s. Ramp-in 1.6 s + 11.1/0.8 hold + ramp-out 1.6 s."""
    tr = _trace(ph, 88.0, 46.0)
    assert 13.0 <= tr[-1][0] <= 17.0

  def test_ascent_is_brisker_than_descent(self, ph):
    """Braking jerk is what annoys; acceleration is fine. A_UP > A_DOWN."""
    down = _trace(ph, 88.0, 46.0)[-1][0]
    up = _trace(ph, 46.0, 88.0)[-1][0]
    assert up < down

  def test_ascent_respects_its_own_limits(self, ph):
    tr = _trace(ph, 46.0, 88.0)
    for (_, c0, r0), (_, c1, r1) in zip(tr, tr[1:]):
      assert abs(r1 - r0) <= ph.CEIL_J_UP * DT + 1e-9
      assert c1 >= c0 - 1e-9
    assert max(r for _, _, r in tr) <= ph.CEIL_A_UP + 1e-9
    assert tr[-1][1] == pytest.approx(88.0 / 3.6, abs=1e-9)

  def test_retarget_mid_ramp_stays_continuous(self, ph):
    """80 -> 60, then 40 arrives while still ramping. No step in value or slope."""
    target_a = 69.0 / 3.6
    target_b = 46.0 / 3.6
    ceiling, rate = 88.0 / 3.6, 0.0
    prev = (ceiling, rate)
    for i in range(400):
      target = target_a if i < 40 else target_b
      ceiling, rate = ph._advance_ceiling(ceiling, rate, target, DT)
      assert abs(rate - prev[1]) <= ph.CEIL_J_DOWN * DT + 1e-9
      assert abs(ceiling - prev[0]) <= ph.CEIL_A_DOWN * DT + 1e-9
      prev = (ceiling, rate)
    assert ceiling == pytest.approx(target_b, abs=1e-9)
    assert rate == 0.0

  def test_reversal_mid_descent_is_continuous(self, ph):
    """The limit rises while the ceiling is still falling. The rate must pass
    through zero under the jerk limit, not flip sign."""
    ceiling, rate = 88.0 / 3.6, 0.0
    for _ in range(40):
      ceiling, rate = ph._advance_ceiling(ceiling, rate, 46.0 / 3.6, DT)
    assert rate < 0.0, 'precondition: still descending'
    prev_rate = rate
    for _ in range(400):
      ceiling, rate = ph._advance_ceiling(ceiling, rate, 100.0 / 3.6, DT)
      assert abs(rate - prev_rate) <= ph.CEIL_J_UP * DT + 1e-9
      prev_rate = rate
    assert ceiling == pytest.approx(100.0 / 3.6, abs=1e-9)

  def test_already_at_target_is_a_no_op(self, ph):
    target = 46.0 / 3.6
    assert ph._advance_ceiling(target, 0.0, target, DT) == (target, 0.0)

  def test_zero_dt_does_not_move_the_ceiling(self, ph):
    c, r = ph._advance_ceiling(88.0 / 3.6, 0.0, 46.0 / 3.6, 0.0)
    assert c == pytest.approx(88.0 / 3.6)
    assert r == 0.0
```

- [ ] **Step 2: Run the tests to verify they fail**

```bash
cd /home/oxygen/catpilot-dev/plugins
PYTHONPATH= uv run pytest plugins/speedlimitd/tests/test_planner_hook.py -q
```

Expected: every test in `TestAdvanceCeiling` errors with `AttributeError: module 'plugins.speedlimitd.planner_hook' has no attribute '_advance_ceiling'`.

- [ ] **Step 3: Add the constants**

In `plugins/speedlimitd/planner_hook.py`, directly after the `SOURCE_ROAD_TYPE_INFERENCE = 2` line:

```python
# --- Jerk-limited allowed-speed ceiling ---
# The enforced target does not jump between limits; it slides under a
# trapezoidal acceleration profile so both its value and its slope are
# continuous. Setpoint GAP is what drives DCC deceleration on this car
# (corr +0.746), so a step change in the target is a brake spike — three of
# them for an 80 -> 40 drop under the old fixed-time ladder.
# Descent is tightly jerk-limited (braking jerk is the complaint); ascent is
# brisk (acceleration is not). DCC caps real acceleration near +0.5 m/s², so
# CEIL_A_UP above ~0.6 simply releases the cap as fast as the car can use it.
CEIL_A_DOWN = 0.8    # m/s²  peak descent rate — matches speedlimitd's COMFORT_BRAKE
CEIL_J_DOWN = 0.5    # m/s³  jerk limit on the descent
CEIL_A_UP = 1.5      # m/s²  peak ascent rate
CEIL_J_UP = 1.0      # m/s³  jerk limit on the ascent
CEIL_SNAP_MS = 0.05  # m/s   close-enough epsilon on the value
CEIL_SNAP_RATE = 0.05  # m/s² close-enough epsilon on the slope
CEIL_DT_MAX = 0.2    # s     dt clamp (plannerd ticks at DT_MDL = 0.05)
```

- [ ] **Step 4: Implement `_advance_ceiling`**

Add directly after `_effective_offset_percent`:

```python
def _advance_ceiling(ceiling_ms, rate_ms2, target_ms, dt):
  """Advance the allowed-speed ceiling one tick toward target_ms.

  Returns (ceiling_ms, rate_ms2). A pure function of its arguments and the
  CEIL_* constants — no vehicle state, no clock — so the trajectory for a
  given limit change is always the same.

  The profile is trapezoidal in acceleration: the rate ramps in at the jerk
  limit, holds at the peak, then ramps back out so the ceiling arrives at the
  target with zero slope.
  """
  err = target_ms - ceiling_ms
  if abs(err) < CEIL_SNAP_MS and abs(rate_ms2) < CEIL_SNAP_RATE:
    return target_ms, 0.0
  if dt <= 0.0:
    return ceiling_ms, rate_ms2

  descending = err < 0.0
  a_max, j_max = (CEIL_A_DOWN, CEIL_J_DOWN) if descending else (CEIL_A_UP, CEIL_J_UP)

  # Δv consumed bleeding the current rate to zero at j_max. Once the remaining
  # error is no bigger than that, start bleeding — that is what makes the
  # arrival slope zero instead of a stop-dead discontinuity.
  stop = rate_ms2 * rate_ms2 / (2.0 * j_max)
  want = 0.0 if abs(err) <= stop else (-a_max if descending else a_max)

  step = max(-j_max * dt, min(j_max * dt, want - rate_ms2))
  rate_ms2 += step
  ceiling_ms += rate_ms2 * dt

  # Discrete integration can step past the target; clamp rather than ring.
  if (target_ms - ceiling_ms < 0.0) != descending:
    return target_ms, 0.0
  return ceiling_ms, rate_ms2
```

- [ ] **Step 5: Run the tests to verify they pass**

```bash
cd /home/oxygen/catpilot-dev/plugins
PYTHONPATH= uv run pytest plugins/speedlimitd/tests/test_planner_hook.py -q
```

Expected: PASS, 12 tests.

If `test_jerk_bound_on_descent` fails on the *final* tick only, the cause is the terminal snap zeroing a non-trivial rate. Do not weaken the assertion — make the snap in the overshoot clamp conditional, returning `(target_ms, 0.0)` only when `abs(rate_ms2) <= j_max * dt`, and otherwise letting the next tick's `stop` logic bleed it.

- [ ] **Step 6: Commit**

```bash
cd /home/oxygen/catpilot-dev/plugins
git add plugins/speedlimitd/planner_hook.py plugins/speedlimitd/tests/test_planner_hook.py
git commit -F - <<'MSG'
speedlimitd: add the jerk-limited ceiling profile integrator

Pure function, no I/O — a trapezoidal acceleration profile that slides a
value toward a target with continuous slope and zero arrival rate. Not
wired into on_v_cruise yet.

Claude-Session: https://claude.ai/code/session_01L53Y95zG636KXqrdkaA2TQ
MSG
```

---

### Task 2: Wire the ceiling into `on_v_cruise`

**Files:**
- Modify: `plugins/speedlimitd/planner_hook.py` (module state, `_reset_all`, `on_v_cruise`)
- Test: `plugins/speedlimitd/tests/test_planner_hook.py` (append a second class)

**Interfaces:**
- Consumes: `_advance_ceiling(ceiling_ms, rate_ms2, target_ms, dt) -> (float, float)` from Task 1.
- Produces: module state `_ceiling_ms: float | None`, `_ceiling_rate: float`, `_last_t: float | None`. Task 3's fixtures reset these three names.

The ceiling advances **before** the gas early-return (it is a pure function of the limit, so a gas hold must not freeze it) and **before** the floors. `_baseline_ms` and the gas-floor clear keep comparing against the **raw** `target_ms`, not the ceiling — they are statements about the limit, not about the ramp.

- [ ] **Step 1: Write the failing tests**

Append to `plugins/speedlimitd/tests/test_planner_hook.py`:

```python
class TestCeilingInOnVCruise:
  """The ceiling as on_v_cruise actually drives it, with a fake clock."""

  def _sm(self, gas=False, lead_status=False, lead_vLead=0.0):
    cs = MagicMock()
    cs.gasPressed = gas
    lead = MagicMock()
    lead.status = lead_status
    lead.vLead = lead_vLead
    radar = MagicMock()
    radar.leadOne = lead
    sm = MagicMock()

    def getitem(key):
      if key == 'carState':
        return cs
      if key == 'radarState':
        return radar
      return MagicMock()

    sm.__getitem__ = MagicMock(side_effect=getitem)
    return sm

  def _sl(self, ph, speed_limit, source=1, safety=False, confirmed=True, road='A'):
    ph._sl_data = {'confirmed': confirmed, 'speedLimit': speed_limit,
                   'safetyCapped': safety, 'source': source,
                   'roadName': road, 'wayRef': ''}

  def _clock(self, ph, monkeypatch, t0=1000.0):
    """Fake monotonic clock. `clk['t'] += DT` advances one planner tick."""
    clk = {'t': t0}
    monkeypatch.setattr(ph.time, 'monotonic', lambda: clk['t'])
    return clk

  # --- first reading is immediate, not ramped -------------------------

  def test_first_reading_applies_immediately(self, ph, monkeypatch):
    """No ramp from a null ceiling — there is nothing to ramp from."""
    self._clock(ph, monkeypatch)
    self._sl(ph, 80, source=1)
    assert ph.on_v_cruise(100 / 3.6, 25.0, self._sm()) == pytest.approx(88 / 3.6, abs=0.01)

  # --- a drop ramps ---------------------------------------------------

  def test_drop_does_not_jump(self, ph, monkeypatch):
    """80 -> 40 must not land on 46 km/h the very next tick."""
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 80, source=1)
    ph.on_v_cruise(100 / 3.6, 25.0, self._sm())
    self._sl(ph, 40, source=1)
    clk['t'] += DT
    out = ph.on_v_cruise(100 / 3.6, 25.0, self._sm())
    assert out > 80 / 3.6, 'ceiling teleported to the new limit'

  def test_drop_reaches_the_target(self, ph, monkeypatch):
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 80, source=1)
    ph.on_v_cruise(100 / 3.6, 25.0, self._sm())
    self._sl(ph, 40, source=1)
    out = None
    for _ in range(600):
      clk['t'] += DT
      out = ph.on_v_cruise(100 / 3.6, 25.0, self._sm())
    assert out == pytest.approx(46 / 3.6, abs=0.01)

  def test_drop_is_monotonic_and_never_speeds_up(self, ph, monkeypatch):
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 80, source=1)
    prev = ph.on_v_cruise(100 / 3.6, 25.0, self._sm())
    self._sl(ph, 40, source=1)
    for _ in range(600):
      clk['t'] += DT
      out = ph.on_v_cruise(100 / 3.6, 25.0, self._sm())
      assert out <= prev + 1e-9
      assert out <= 100 / 3.6 + 1e-9, 'returned above the incoming v_cruise'
      prev = out

  # --- safety caps bypass the ramp ------------------------------------

  def test_safety_cap_bypasses_the_ramp(self, ph, monkeypatch):
    """A tightening curve must bite now, not in 15 s."""
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 80, source=1)
    ph.on_v_cruise(100 / 3.6, 25.0, self._sm())
    self._sl(ph, 40, source=4, safety=True)
    clk['t'] += DT
    # safetyCapped => no offset, exact limit, immediately
    assert ph.on_v_cruise(100 / 3.6, 25.0, self._sm()) == pytest.approx(40 / 3.6, abs=0.01)

  def test_ramp_resumes_cleanly_after_a_safety_cap_releases(self, ph, monkeypatch):
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 40, source=4, safety=True)
    ph.on_v_cruise(100 / 3.6, 25.0, self._sm())
    self._sl(ph, 80, source=1)
    clk['t'] += DT
    out = ph.on_v_cruise(100 / 3.6, 25.0, self._sm())
    assert out < 88 / 3.6, 'release teleported instead of ramping up'
    assert out > 40 / 3.6

  # --- gas -------------------------------------------------------------

  def test_ceiling_keeps_ramping_while_gas_is_pressed(self, ph, monkeypatch):
    """The ceiling is a function of the limit, not of the driver. On release
    the limit must already be where it belongs, not 15 s behind."""
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 80, source=1)
    ph.on_v_cruise(100 / 3.6, 25.0, self._sm())
    self._sl(ph, 40, source=1)
    for _ in range(600):
      clk['t'] += DT
      ph.on_v_cruise(100 / 3.6, 25.0, self._sm(gas=True))
    assert ph._ceiling_ms == pytest.approx(46 / 3.6, abs=0.01)

  # --- reset -----------------------------------------------------------

  def test_invalid_limit_clears_the_ceiling(self, ph, monkeypatch):
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 80, source=1)
    ph.on_v_cruise(100 / 3.6, 25.0, self._sm())
    self._sl(ph, 80, source=1, confirmed=False)
    clk['t'] += DT
    ph.on_v_cruise(100 / 3.6, 25.0, self._sm())
    assert ph._ceiling_ms is None
    # …and the next valid limit is applied immediately, not ramped from stale state
    self._sl(ph, 40, source=1)
    clk['t'] += DT
    assert ph.on_v_cruise(100 / 3.6, 25.0, self._sm()) == pytest.approx(46 / 3.6, abs=0.01)

  def test_road_change_does_not_reset_the_ceiling(self, ph, monkeypatch):
    """Resetting on a road change would reintroduce exactly the jump we removed."""
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 80, source=1, road='A')
    ph.on_v_cruise(100 / 3.6, 25.0, self._sm())
    self._sl(ph, 40, source=1, road='B')
    clk['t'] += DT
    out = ph.on_v_cruise(100 / 3.6, 25.0, self._sm())
    assert out > 80 / 3.6, 'ceiling was reset by the road change'

  # --- dt hygiene -------------------------------------------------------

  def test_long_stall_does_not_lurch(self, ph, monkeypatch):
    """A 10 s gap between ticks (process stall) must be clamped to CEIL_DT_MAX,
    not integrated as a 10 s step."""
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 80, source=1)
    ph.on_v_cruise(100 / 3.6, 25.0, self._sm())
    self._sl(ph, 40, source=1)
    clk['t'] += 10.0
    out = ph.on_v_cruise(100 / 3.6, 25.0, self._sm())
    assert out > 87 / 3.6, 'a stalled tick integrated the whole gap'
```

- [ ] **Step 2: Run the tests to verify they fail**

```bash
cd /home/oxygen/catpilot-dev/plugins
PYTHONPATH= uv run pytest plugins/speedlimitd/tests/test_planner_hook.py::TestCeilingInOnVCruise -q
```

Expected: failures — `ph.time` does not exist yet (`AttributeError`), and the drop tests see the old instantaneous behaviour.

- [ ] **Step 3: Add `import time` and the ceiling state**

At the top of `plugins/speedlimitd/planner_hook.py`, above the existing `from openpilot.common.constants import CV`:

```python
import time
```

Then extend the enforcement-state block. Replace:

```python
# Enforcement state.
_baseline_ms = None   # inferred running-max target on the current road (floor)
_gas_floor_ms = None  # driver-override hold floor (all sources), set post gas
_road_id = ''         # last non-empty OSM road identity
```

with:

```python
# Enforcement state.
_baseline_ms = None   # inferred running-max target on the current road (floor)
_gas_floor_ms = None  # driver-override hold floor (all sources), set post gas
_road_id = ''         # last non-empty OSM road identity
_ceiling_ms = None    # jerk-limited allowed-speed ceiling (m/s); None = uninitialised
_ceiling_rate = 0.0   # its current slope (m/s², signed)
_last_t = None        # monotonic timestamp of the last ceiling advance
```

- [ ] **Step 4: Clear the ceiling in `_reset_all`**

Replace:

```python
def _reset_all():
  """Clear the floors (road identity is kept across brief invalid limits)."""
  global _baseline_ms, _gas_floor_ms
  _baseline_ms = None
  _gas_floor_ms = None
```

with:

```python
def _reset_all():
  """Clear the floors and the ceiling (road identity is kept across brief
  invalid limits). Clearing the ceiling means the next valid limit is applied
  immediately rather than ramped from a stale value."""
  global _baseline_ms, _gas_floor_ms, _ceiling_ms, _ceiling_rate, _last_t
  _baseline_ms = None
  _gas_floor_ms = None
  _ceiling_ms = None
  _ceiling_rate = 0.0
  _last_t = None
```

- [ ] **Step 5: Advance the ceiling in `on_v_cruise`**

Widen the `global` declaration on the first line of `on_v_cruise`:

```python
  global _baseline_ms, _gas_floor_ms, _road_id, _ceiling_ms, _ceiling_rate, _last_t
```

Then, immediately after the road-change block (the one ending `_gas_floor_ms = None`) and **before** the `if _gas_pressed(sm):` block, insert:

```python
  # Advance the jerk-limited ceiling toward the target. This runs before the
  # gas early-return on purpose: the ceiling is a pure function of the limit,
  # so a gas hold must not freeze it — the gas FLOOR is what suspends
  # enforcement. Safety caps assign the target directly; a tightening curve
  # cannot wait out a 15 s ramp.
  now = time.monotonic()
  if _ceiling_ms is None or safety_capped:
    _ceiling_ms, _ceiling_rate = target_ms, 0.0
  else:
    dt = min(max(now - _last_t, 0.0), CEIL_DT_MAX) if _last_t is not None else 0.0
    _ceiling_ms, _ceiling_rate = _advance_ceiling(_ceiling_ms, _ceiling_rate, target_ms, dt)
  _last_t = now
```

- [ ] **Step 6: Consume the ceiling downstream**

In the same function, replace the floors/enforcement block. Replace:

```python
  floors = [f for f in (baseline_floor, _gas_floor_ms) if f is not None]
  effective_floor = max(floors) if floors else None
  floored_target = target_ms if effective_floor is None else max(target_ms, effective_floor)
```

with:

```python
  # The ramped ceiling — not the raw target — is what gets enforced. The floors
  # above still track the RAW target: they are statements about the limit
  # ("highest seen on this road", "driver's held speed"), not about the ramp.
  floors = [f for f in (baseline_floor, _gas_floor_ms) if f is not None]
  effective_floor = max(floors) if floors else None
  floored_target = _ceiling_ms if effective_floor is None else max(_ceiling_ms, effective_floor)
```

Leave `_baseline_ms`, the gas-floor ratchet/clear, the lead override, and the final `if floored_target < v_cruise` comparison exactly as they are — they already read `target_ms` where they should.

- [ ] **Step 7: Run the new tests**

```bash
cd /home/oxygen/catpilot-dev/plugins
PYTHONPATH= uv run pytest plugins/speedlimitd/tests/test_planner_hook.py -q
```

Expected: PASS, 23 tests.

- [ ] **Step 8: Commit**

```bash
cd /home/oxygen/catpilot-dev/plugins
git add plugins/speedlimitd/planner_hook.py plugins/speedlimitd/tests/test_planner_hook.py
git commit -F - <<'MSG'
speedlimitd: enforce a ramped ceiling instead of the raw limit target

on_v_cruise now slides the allowed-speed ceiling toward the offset-adjusted
target rather than assigning it, so DCC sees a continuous setpoint instead
of a step. The ceiling advances before the gas early-return -- it is a
function of the limit, and the gas floor is what suspends enforcement. The
hold floors keep tracking the raw target; safety caps assign directly.

Claude-Session: https://claude.ai/code/session_01L53Y95zG636KXqrdkaA2TQ
MSG
```

---

### Task 3: Repair the existing hook tests that assume instant re-targeting

The 26 tests in `TestPlannerHook` (`tests/test_speedlimitd.py:684`) predate the ramp. Ones that read a limit **once** still pass — the first reading initialises the ceiling immediately. Ones that **change** the limit mid-test and assert the new cap on the next call will now see a ramping value.

The fixture must also reset the three new state names, or state leaks between tests.

**Files:**
- Modify: `plugins/speedlimitd/tests/test_speedlimitd.py:684-960` (`TestPlannerHook`)

**Interfaces:**
- Consumes: `_ceiling_ms`, `_ceiling_rate`, `_last_t` from Task 2.
- Produces: nothing.

- [ ] **Step 1: Reset the new state in the `hook` fixture**

In `TestPlannerHook.hook` (around line 696), after `mod._road_id = ''`, add:

```python
    mod._ceiling_ms = None
    mod._ceiling_rate = 0.0
    mod._last_t = None
```

- [ ] **Step 2: Make `_clock` real again and add `_settle`**

The `_clock` helper at line 731 is a no-op stub left from a previously-removed ramp. Replace the whole method with:

```python
  def _clock(self, monkeypatch, hook, t0=1000.0):
    """Fake monotonic clock for the ceiling ramp. `.tick(dt)` advances it."""
    clk = {'t': t0}
    monkeypatch.setattr(hook.time, 'monotonic', lambda: clk['t'])

    class _C:
      def tick(self, dt):
        clk['t'] += dt

    return _C()

  def _settle(self, hook, v_cruise, v_ego, sm, clk, ticks=600, dt=0.05):
    """Run the hook until the ceiling has finished ramping; return the last
    returned v_cruise. Tests that assert a STEADY-STATE cap use this; tests
    that assert ramp behaviour drive the clock themselves."""
    out = None
    for _ in range(ticks):
      clk.tick(dt)
      out = hook.on_v_cruise(v_cruise, v_ego, sm)
    return out
```

- [ ] **Step 3: Run the suite and triage**

```bash
cd /home/oxygen/catpilot-dev/plugins
PYTHONPATH= uv run pytest plugins/speedlimitd/tests/test_speedlimitd.py -q -k PlannerHook
```

Expected: some failures. Every failure falls into exactly one of two buckets — decide per test, do not blanket-apply:

- **Asserting a steady-state cap after a limit change** (e.g. `test_inferred_real_drop_new_road_slows` at :831, `test_empty_road_id_disables_baseline_hold` at :936). These are still correct in intent. Give the test a `clk = self._clock(monkeypatch, hook)`, add `monkeypatch` to the signature if absent, and replace the final `hook.on_v_cruise(...)` with `self._settle(hook, ..., clk)`. Note in a comment that the value is now reached by ramp.
- **Asserting a hold** (e.g. `test_inferred_spurious_drop_holds_speed` at :817) — the floor clamps the result regardless of the ramp, so these should already pass. If one fails, the floor wiring is wrong; fix `planner_hook.py`, not the test.

- [ ] **Step 4: Add a regression test pinning the layer split**

Append inside `TestPlannerHook`:

```python
  def test_floors_track_the_raw_limit_not_the_ramped_ceiling(self, hook, monkeypatch):
    """The baseline floor means "highest limit seen on this road". If it
    tracked the ceiling instead, a mid-ramp reading would bake the ramp's
    transient into the floor and the hold would drift."""
    clk = self._clock(monkeypatch, hook)
    self._sl(hook, 100, source=2, road='A')
    hook.on_v_cruise(40.0, 30.0, self._sm())
    assert hook._baseline_ms == pytest.approx(100 * 1.10 / 3.6, abs=0.01)
    self._sl(hook, 60, source=2, road='A')     # spurious drop, same road
    clk.tick(0.05)
    hook.on_v_cruise(40.0, 30.0, self._sm())
    assert hook._baseline_ms == pytest.approx(100 * 1.10 / 3.6, abs=0.01)
```

- [ ] **Step 5: Run the full hook suite**

```bash
cd /home/oxygen/catpilot-dev/plugins
PYTHONPATH= uv run pytest plugins/speedlimitd/tests/test_speedlimitd.py -q -k PlannerHook
PYTHONPATH= uv run pytest plugins/speedlimitd/tests/test_planner_hook.py -q
```

Expected: both PASS.

- [ ] **Step 6: Commit**

```bash
cd /home/oxygen/catpilot-dev/plugins
git add plugins/speedlimitd/tests/test_speedlimitd.py
git commit -F - <<'MSG'
speedlimitd: the hook tests assumed a limit change lands in one tick

They predate the ramp, so a re-target now returns a mid-ramp value. Tests
asserting a steady-state cap settle the ramp first; tests asserting a hold
were already correct, since the floor clamps regardless of the ramp. The
_clock helper -- a no-op stub left from an earlier removed ramp -- drives
the ceiling again.

Claude-Session: https://claude.ai/code/session_01L53Y95zG636KXqrdkaA2TQ
MSG
```

---

### Task 4: Delete the daemon's display ladder

**Files:**
- Modify: `plugins/speedlimitd/speedlimitd.py` (lines ~114-134, ~895-898, ~1637-1660)
- Modify: `plugins/speedlimitd/tests/test_speedlimitd.py:2591`, `:2977`

**Interfaces:**
- Consumes: nothing.
- Produces: `speedLimit` on the bus is now the fused limit with no transition shaping. `_step_speed_limit`, `_STEP_DOWN_INTERVAL`, `_STEP_UP_INTERVAL` and `self._last_step_time` cease to exist.

- [ ] **Step 1: Rework the two ladder tests first**

`test_osm_step_is_plus_minus_10` (:2977) asserts stepping that will not exist. Its surviving value is the OSM ±5 *rounding*. Replace the whole method with:

```python
  def test_osm_rounds_to_5_not_the_cn_ladder(self, sld):
    """OSM carries the exact posted value, so it rounds to 5 km/h — the CN
    ladder would round a US 45 mph (72 km/h) limit UP to 80. The transition
    itself is no longer shaped here; planner_hook ramps the enforced ceiling."""
    mw = self._mw_for_update(sld)
    self._arm(mw, 72.0)
    mw._displayed_speed_limit = 105   # a previous, unrelated reading
    mw.update()
    pub = mw._sl_pub.send.call_args[0][0]
    assert pub['speedLimit'] == 70    # applied at once, rounded to 5
```

`test_oscillation_ladder_damps_displayed_limit` (:2591) asserts "never more than one rung apart", which was the ladder's damping. That damping now lives in the hook. The test's other assertions — `_gs_force_release` and the `_gs_margin_since` reset on re-match — are about G/S release and are still valid. Rename it and drop only the ladder assertion:

```python
  def test_gs_oscillation_does_not_corrupt_release_state(self, sld, monkeypatch):
    """Release → genuine G/S re-match (re-promote, margin count reset) → fresh
    divergence → re-release, repeated. inferenceMode may oscillate; the release
    bookkeeping must stay coherent through it.

    This test used to also assert that _displayed_speed_limit never moved more
    than one standard rung per step interval. That ladder is gone — the daemon
    publishes the fused limit directly and planner_hook ramps the enforced
    ceiling instead, so transition damping is no longer observable here."""
    mw = self._mw(sld)
    mw.lane_count_stable = 3
    clock = {'t': 20000.0}
    monkeypatch.setattr(sld.time, 'monotonic', lambda: clock['t'])
    self._hold_s1(mw)
    for _ in range(3):
      self._diverge(mw, clock, held_dist=13.5, matched_dist=0.6)
      self._diverge(mw, clock, held_dist=17.0, matched_dist=0.6)
      assert mw._gs_force_release is True
      self._hold_s1(mw)                          # genuine re-match re-promotes
      assert mw._gs_margin_since is None            # reset on the re-match
```

- [ ] **Step 2: Run them to verify they fail**

```bash
cd /home/oxygen/catpilot-dev/plugins
PYTHONPATH= uv run pytest plugins/speedlimitd/tests/test_speedlimitd.py -q \
  -k "osm_rounds_to_5 or gs_oscillation_does_not_corrupt"
```

Expected: `test_osm_rounds_to_5_not_the_cn_ladder` FAILS (`assert 95 == 70` — the ladder still steps). The renamed G/S test should pass already; that is fine, it is a rename plus a deletion.

- [ ] **Step 3: Delete the ladder constants and stepper**

In `plugins/speedlimitd/speedlimitd.py`, delete this entire block (the comment, both constants, and the whole `_step_speed_limit` function, lines ~113-134):

```python
# Gradual transition timing (seconds per step)
_STEP_DOWN_INTERVAL = 3.0  # downgrade: 80 → 60 → 50 → 40 (3s per step)
_STEP_UP_INTERVAL = 2.0    # upgrade:   40 → 50 → 60 → 80 (2s per step)


def _step_speed_limit(current: int, target: int) -> int:
  ...
```

Keep `_STANDARD_SPEEDS` and `snap_to_standard_speed` — those are display formatting, not transition timing.

- [ ] **Step 4: Delete the `_last_step_time` state**

Replace lines ~895-898:

```python
    # Gradual speed limit transition — step through standard speeds one level
    # at a time instead of jumping directly (e.g. 80 → 60 → 50 → 40).
    self._displayed_speed_limit: int = 0
    self._last_step_time: float = 0.0
```

with:

```python
    # The published limit. Transitions are NOT shaped here — planner_hook ramps
    # the enforced ceiling. This is the sign value: what the road says, now.
    self._displayed_speed_limit: int = 0
```

- [ ] **Step 5: Replace the transition block in `update()`**

Replace the whole `--- Gradual speed limit transition ---` block (lines ~1637-1660, from the comment through `self._last_step_time = now`) with:

```python
    # --- Publish the fused limit directly ---
    # No transition shaping here. The daemon answers "what is the limit";
    # planner_hook owns "how we get there" and ramps the enforced ceiling under
    # a jerk limit. The ladder that used to live here set both, which meant the
    # sign's step interval was also the brake schedule.
    # OSM-sourced base carries the exact posted value — the CN ladder would
    # round a US 45 mph (72 km/h) limit UP to 80, so round to 5 km/h instead.
    osm_display = osm_base and source == 2
    if osm_display:
      self._displayed_speed_limit = int(round(speed_limit / 5.0) * 5)
    else:
      self._displayed_speed_limit = snap_to_standard_speed(int(speed_limit))
```

The safety-cap clamp immediately below it, and the `safetyCapped` computation below that, are unchanged.

- [ ] **Step 6: Run the full plugin suite**

```bash
cd /home/oxygen/catpilot-dev/plugins
PYTHONPATH= uv run pytest plugins/speedlimitd/tests/ -q
```

Expected: PASS. If a daemon test fails because it set `_displayed_speed_limit` to a prior value and expected stepping, it is asserting removed behaviour — rework it the way Step 1 reworked the OSM one. If a test fails referencing `_last_step_time`, delete that line from it.

- [ ] **Step 7: Commit**

```bash
cd /home/oxygen/catpilot-dev/plugins
git add plugins/speedlimitd/speedlimitd.py plugins/speedlimitd/tests/test_speedlimitd.py
git commit -F - <<'MSG'
speedlimitd: the sign's step interval was also the brake schedule

The display ladder did two jobs: what the HUD shows and how fast the car
slows. Now that planner_hook ramps the enforced ceiling, the daemon just
publishes what the road says. The CN-ladder snap and the OSM round-to-5
stay -- those are display formatting, not transition timing.

Claude-Session: https://claude.ai/code/session_01L53Y95zG636KXqrdkaA2TQ
MSG
```

---

### Task 5: Update DESIGN.md

**Files:**
- Modify: `plugins/speedlimitd/DESIGN.md` (§"Temporal accumulators & the display ladder" at :413, §"Enforcement & gas override" at :439)

**Interfaces:**
- Consumes: the final behaviour from Tasks 1-4.
- Produces: nothing.

- [ ] **Step 1: Replace the "Display step ladder" bullet**

In §"Temporal accumulators & the display ladder", replace the final bullet:

```markdown
- **Display step ladder.** The published limit changes one standard step at a
  time — `_STEP_DOWN_INTERVAL = 3 s`, `_STEP_UP_INTERVAL = 2 s` per rung — via
  `_step_speed_limit`. **Safety caps bypass this**: a tightening curve or
  reactive cap clamps the displayed limit down immediately.
```

with:

```markdown
- **No display ladder.** The published limit is the fused value, applied at
  once (CN-ladder snapped, or rounded to 5 km/h under an OSM base). Transition
  shaping lives in `planner_hook`, which ramps the *enforced* ceiling — see
  below. The ladder that used to live here set the sign's step interval and the
  brake schedule with one number; splitting them is what removed the brake
  spikes. Safety caps still clamp the published limit down immediately.
```

Retitle the section from `## Temporal accumulators & the display ladder` to `## Temporal accumulators`.

- [ ] **Step 2: Rewrite the enforcement preamble**

In §"Enforcement & gas override (`planner_hook.on_v_cruise`)", replace:

```markdown
The hook drains `speedLimitState` from the bus and returns a possibly-reduced
`v_cruise`. It **only ever lowers or holds** — `floored_target < v_cruise`
returns the target, otherwise `v_cruise` is unchanged; DCC comfort-shapes the
deceleration (no artificial ramp). Key rules:
```

with:

```markdown
The hook drains `speedLimitState` from the bus and returns a possibly-reduced
`v_cruise`. It **only ever lowers or holds** — `floored_target < v_cruise`
returns the target, otherwise `v_cruise` is unchanged. Key rules:
```

- [ ] **Step 3: Add the ceiling as the first rule**

Insert as the first bullet of that list, above **Comfort offset**:

```markdown
- **Jerk-limited ceiling.** The enforced target does not jump between limits.
  `_ceiling_ms` slides toward `limit + offset` under a trapezoidal acceleration
  profile (`_advance_ceiling`), so both its value and its slope stay continuous
  and it arrives with zero slope. Descent `CEIL_A_DOWN = 0.8` m/s² at
  `CEIL_J_DOWN = 0.5` m/s³ (~15 s for 80 → 40); ascent `CEIL_A_UP = 1.5` m/s²
  at `CEIL_J_UP = 1.0` m/s³. Setpoint *gap* is what drives DCC deceleration on
  this car (corr +0.746), so a step in the target is a brake spike — the old
  3 s ladder produced three of them per drop.

  The ceiling is a **pure function of the limit history**: it never reads
  `v_ego` or `v_cruise`, so a given limit change always produces the same
  trajectory. It keeps advancing while the gas is pressed (the gas *floor* is
  what suspends enforcement), and it survives a `road_id` change — resetting it
  there would reintroduce the jump. `_reset_all()` clears it, so the next valid
  limit is applied immediately rather than ramped from stale state.

  **Safety caps bypass the ramp** — `safetyCapped` assigns the target directly.
  A tightening curve cannot wait out a 15 s ramp. `CEIL_A_DOWN` equals
  `COMFORT_BRAKE` precisely so a safety cap's own distance-aware profile is
  never gentler than the comfort ramp, and the two can never fight.

  **Known trade-off:** because the ceiling ignores vehicle state, it is slower
  to bite on a driver already below the old limit — at 70 in an 80 zone it
  spends ~6 s descending 88 → 70 before the car feels anything. Accepted. If a
  drive shows it matters, initialise the ceiling at `min(_ceiling_ms, v_cruise)`
  when a descent begins.
```

- [ ] **Step 4: Fix the stale ramp reference in the hold-floor bullet**

Still in that list, the **Baseline (road-continuity) floor** bullet ends "the inferred/vision cap is allowed to slow the car." Confirm it does not claim there is no ramp; if the phrase "no artificial ramp" appears anywhere else in the file, remove it. Check with:

```bash
cd /home/oxygen/catpilot-dev/plugins
grep -n "artificial ramp\|_step_speed_limit\|_STEP_DOWN_INTERVAL\|_STEP_UP_INTERVAL\|_last_step_time" \
  plugins/speedlimitd/DESIGN.md plugins/speedlimitd/README.md
```

Expected: no output.

- [ ] **Step 5: Full suite + commit**

```bash
cd /home/oxygen/catpilot-dev/plugins
PYTHONPATH= uv run pytest plugins/speedlimitd/tests/ -q
git add plugins/speedlimitd/DESIGN.md
git commit -F - <<'MSG'
speedlimitd: DESIGN.md still described the ladder and "no artificial ramp"

Both are now wrong: the daemon publishes the fused limit directly and the
hook ramps the enforced ceiling under a jerk limit. Documents the pure-
function property, the safety bypass, and the accepted slower bite on a
driver already below the limit.

Claude-Session: https://claude.ai/code/session_01L53Y95zG636KXqrdkaA2TQ
MSG
```

---

## Verification

Bench tests are necessary but not sufficient. This changes longitudinal behaviour, so it is not done until an on-car drive confirms:

1. A non-safety limit drop (e.g. an 80 → 40 sign) produces one continuous slowdown, not three brake pulses.
2. A curve/safety cap still bites immediately.
3. The HUD sign flipping ahead of the car's speed is tolerable in practice.

Deploy per the standard plugins flow, then check `plugind` picked it up:

```bash
ssh c3 'cd /data/plugins && GIT_SSL_NO_VERIFY=1 git fetch origin dev && git reset --hard origin/dev && bash install.sh'
```

`planner_hook` is imported by `plannerd`, not by `plugind`, so a UI restart is not enough — the change lands when `plannerd` restarts (offroad→onroad transition). `speedlimitd.py` is a plugin process and picks up `.needs_restart` normally.
