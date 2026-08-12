# Angle-Based Breakaway — Replacing FRICTION Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the BMW lateral controller's `FRICTION` constant — which *predicts* whether a commanded torque can move the steering rack — with direct *observation* of whether the rack is moving, derived from `steeringAngleDeg`.

**Architecture:** A new pure module `bmw/rack_motion.py` provides a windowed least-squares Δangle observer and an online breakaway-torque estimator. `latcontroller.py` feeds it `CS.steeringAngleDeg` on every CAN tick and consults the estimate at the three sites that currently read `FRICTION`, plus the `HOLD_BAND` derivation which is itself FRICTION-derived. A stall-aware ramp then stops winding torque once motion is observed and backs off to the sustaining level. All control-affecting behaviour sits behind a plugin param defaulting OFF.

**Tech Stack:** Python 3.12, numpy (already a controller dependency), pytest, openpilot 0.11.x plugin framework, catpilot plugin bus for telemetry.

## Global Constraints

- Target file lives at `plugins/bmw_e9x_e8x/bmw/latcontroller.py`; the new module at `plugins/bmw_e9x_e8x/bmw/rack_motion.py`.
- **Module identity:** the registry loads the controller as `plugins.bmw_e9x_e8x.bmw.latcontroller`. `latcontroller.py` already inserts the plugin root on `sys.path` (lines 20–27) so `from bmw.rack_motion import ...` resolves. Use exactly that import form — never `import bmw.latcontroller` from elsewhere.
- **Sign convention (measured, non-negotiable):** `steeringAngleDeg` positive = LEFT. Controller `output`/`torque` fraction negative = LEFT. They are OPPOSITE. All sign handling goes through `TORQUE_TO_ANGLE_SIGN` in `rack_motion.py`; no ad-hoc flips.
- **Angle offset:** `steeringAngleDeg` carries a constant ~−1.58° physical alignment offset. Work only in deltas. Never read absolute angle in this feature.
- **Measured constants (from route 3f2, 51 segments, `action=='ramp'` ticks only):** angle LSB `0.0879` deg; window `0.16` s → `0.55` deg/s resolution; motion threshold `2.0` deg/s; breakaway knee `2.0–2.75` Nm; `STEER_MAX = 12` Nm so seed `0.20` frac = 2.4 Nm; existing `FRICTION = 0.05` frac = 0.6 Nm (≈4× too low); sustain ≈ 0.5 × breakaway.
- **Plugin params** live in the plugin data dir via `config.read_plugin_param` — never `/data/params/d/`. Param id is `bmw_e9x_e8x`.
- **Tests:** run from the plugins repo root with `PYTHONPATH=. uv run pytest`.
- **Commits:** no `Co-Authored-By` lines.
- **Do not touch:** `DRIFT_M`, `KN_*` noise-floor, `KD_*` blend, `RELAX_DWELL_TICKS/KAPPA`, `HOLD_CAP_DRIFT_M`. Those describe vision noise and lane geometry, not the rack.
- **Out of scope:** the full inner-rate-loop / angle-space cascade redesign. This plan replaces FRICTION and adds stall-aware ramping only.
- **Device access is `ssh c3` only.** Never hardcode an IP. Read-only on device except the documented deploy step.

### Verified facts about the existing code — do not re-derive these

- `latcontroller.py` imports at module scope are only `os`, `sys`, `numpy`. **`math` is imported at line 178, inside `on_lat_controller_init`** — so `math.copysign` in Task 6 works, but any new module-scope use needs its own import.
- `CCP` comes from `from bmw.values import CarControllerParams as CCP` at **line 178–181, function scope**. It is a closure variable visible inside `update()`. `CCP.STEER_MAX == 12`.
- `kappa_scale` is computed at **line 429**; the `deep_relax` block is at **line 478**. So a value derived from `kappa_scale` may be computed anywhere between them.
- **`read_plugin_param` is NOT imported** in `latcontroller.py`. Task 5 must add it.
- **`sec_since_boot` is NOT imported and must not be used.** Use `time.monotonic()` — stdlib, always present, ~50 ns.
- **Test harness signatures (`tests/test_latcontroller.py`), exact:**
  - `_make_controller(monkeypatch, wheelbase=2.66)` returns a **4-tuple** `(lac, fake_sm, mod, state)`.
  - `_call_update(lac, desired_curvature, lat_delay=0.2, v_ego=20.0, active=True)` — there is no `_tick()`.
  - `_call_update` builds `CS = SimpleNamespace(vEgo=v_ego)`, which **has no `steeringAngleDeg`**. Task 4 must extend it, or every existing test raises `AttributeError`.
  - `_set_measured(fake_sm, v, kappa_meas)` seeds livePose content.
  - `state` is reached via `_closure_state(lac.update)`.

---

### Task 1: Δangle motion observer

**Files:**
- Create: `plugins/bmw_e9x_e8x/bmw/rack_motion.py`
- Test: `plugins/bmw_e9x_e8x/tests/test_rack_motion.py`

**Interfaces:**
- Produces: `RackMotion(window_s=0.16)` with `.update(t, angle_deg) -> None`, `.rate_deg_s -> float` (NaN when insufficient span), `.is_moving(threshold_deg_s=2.0) -> bool`, `.is_moving_with_torque(torque_frac, threshold_deg_s=2.0) -> bool`, `.reset() -> None`. Module constants `ANGLE_LSB_DEG`, `WINDOW_S`, `MOTION_THRESHOLD_DEG_S`, `TORQUE_TO_ANGLE_SIGN`.

- [ ] **Step 1: Write the failing test**

Create `plugins/bmw_e9x_e8x/tests/test_rack_motion.py`:

```python
import math
from bmw.rack_motion import (RackMotion, ANGLE_LSB_DEG, WINDOW_S,
                             MOTION_THRESHOLD_DEG_S, TORQUE_TO_ANGLE_SIGN)


def _feed(rm, rate_deg_s, duration_s=0.30, dt=0.01, start_angle=0.0, quantise=True):
    """Drive the observer with a constant-rate ramp, optionally LSB-quantised."""
    t = 0.0
    angle = start_angle
    while t <= duration_s + 1e-9:
        a = angle
        if quantise:
            a = round(a / ANGLE_LSB_DEG) * ANGLE_LSB_DEG
        rm.update(t, a)
        t += dt
        angle += rate_deg_s * dt
    return rm


def test_rate_nan_before_window_fills():
    rm = RackMotion()
    rm.update(0.0, 1.0)
    rm.update(0.01, 1.0)
    assert math.isnan(rm.rate_deg_s)


def test_constant_rate_recovered_within_resolution():
    rm = _feed(RackMotion(), 20.0)
    assert abs(rm.rate_deg_s - 20.0) < 1.0


def test_stationary_reads_zero_rate():
    rm = _feed(RackMotion(), 0.0)
    assert abs(rm.rate_deg_s) < 0.55


def test_constant_offset_cancels_exactly_without_quantisation():
    """The algorithm is offset-immune: differencing removes any constant.

    Quantisation is off here so this tests the property itself. With it on,
    an offset that is not a whole number of LSBs (-1.58 / 0.0879 = 17.97)
    shifts the sampling phase, so cancellation is exact only in exact
    arithmetic — see the companion test below.
    """
    a = _feed(RackMotion(), 12.0, start_angle=0.0, quantise=False).rate_deg_s
    b = _feed(RackMotion(), 12.0, start_angle=-1.58, quantise=False).rate_deg_s
    assert abs(a - b) < 1e-9


def test_constant_offset_cancels_within_quantisation_noise():
    """With LSB quantisation the residual is bounded by the noise floor."""
    a = _feed(RackMotion(), 12.0, start_angle=0.0).rate_deg_s
    b = _feed(RackMotion(), 12.0, start_angle=-1.58).rate_deg_s
    assert abs(a - b) < 0.55          # ANGLE_LSB_DEG / WINDOW_S


def test_is_moving_threshold():
    assert not _feed(RackMotion(), 1.0).is_moving()
    assert _feed(RackMotion(), 5.0).is_moving()


def test_is_moving_with_torque_requires_matching_direction():
    # Negative torque commands LEFT; LEFT is POSITIVE steering angle.
    rm = _feed(RackMotion(), +20.0)          # wheel moving left
    assert rm.is_moving_with_torque(-0.20)   # left torque -> agrees
    assert not rm.is_moving_with_torque(+0.20)  # right torque -> disagrees


def test_zero_torque_never_counts_as_moving_with_torque():
    rm = _feed(RackMotion(), +20.0)
    assert not rm.is_moving_with_torque(0.0)


def test_reset_clears_history():
    rm = _feed(RackMotion(), 20.0)
    rm.reset()
    assert math.isnan(rm.rate_deg_s)


def test_stale_samples_are_evicted():
    rm = RackMotion()
    _feed(rm, 40.0, duration_s=0.30)
    _feed(rm, 0.0, duration_s=0.30, start_angle=100.0)
    assert abs(rm.rate_deg_s) < 0.55


def test_sign_convention_constant_is_negative():
    assert TORQUE_TO_ANGLE_SIGN == -1.0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/oxygen/catpilot-dev/plugins && PYTHONPATH=.:plugins/bmw_e9x_e8x uv run pytest plugins/bmw_e9x_e8x/tests/test_rack_motion.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'bmw.rack_motion'`

- [ ] **Step 3: Write the implementation**

Create `plugins/bmw_e9x_e8x/bmw/rack_motion.py`:

```python
"""Rack motion observation for the BMW lateral controller.

Replaces the job FRICTION used to do — PREDICTING whether a commanded torque
can move the steering rack — with OBSERVING whether it actually is moving.

Why: breakaway torque is not a constant. Measured on route 3f2 (51 segments,
`action=='ramp'` ticks only) the knee moves from under 0.25 Nm to beyond
3.9 Nm with lateral acceleration and speed, plus unobservables (tyre pressure,
road surface, rack lubrication). FRICTION = 0.05 frac (0.6 Nm) understates the
measured 2.0-2.75 Nm knee by roughly 4x, and HOLD_BAND is derived from it.

steeringAngleDeg is ground truth that the rack moved, independent of every one
of those unobservables. It carries a constant ~-1.58 deg physical alignment
offset which cancels exactly under differencing, so this module works only in
deltas and never reads absolute angle.

Loaded as plugins.bmw_e9x_e8x.bmw.rack_motion. Pure computation, no I/O.
"""
from collections import deque

# steeringAngleDeg quantisation, measured (169 distinct values over a segment).
ANGLE_LSB_DEG = 0.0879

# Rate window. Resolution = ANGLE_LSB_DEG / WINDOW_S = 0.55 deg/s.
# Shorter windows are noisier: a single-sample difference at 100 Hz is
# 8.8 deg/s of pure quantisation noise.
WINDOW_S = 0.16

# "Stuck" criterion used throughout the plant characterisation.
MOTION_THRESHOLD_DEG_S = 2.0

# Controller torque fraction is NEGATIVE for LEFT; steeringAngleDeg is
# POSITIVE for LEFT. Verified against GPS bearing, DSC yaw rate and lateral
# acceleration on route 3f2. This is the single place that conversion lives.
TORQUE_TO_ANGLE_SIGN = -1.0


class RackMotion:
  """Windowed least-squares slope of steering angle. Offset-immune."""

  def __init__(self, window_s=WINDOW_S):
    self.window_s = float(window_s)
    self._t = deque()
    self._a = deque()

  def reset(self):
    self._t.clear()
    self._a.clear()

  def update(self, t, angle_deg):
    t = float(t)
    # Non-monotonic time (log replay seek, engagement restart) invalidates the
    # window rather than producing a bogus slope.
    if self._t and t <= self._t[-1]:
      self.reset()
    self._t.append(t)
    self._a.append(float(angle_deg))
    while len(self._t) > 1 and (self._t[-1] - self._t[0]) > self.window_s:
      self._t.popleft()
      self._a.popleft()

  @property
  def rate_deg_s(self):
    n = len(self._t)
    if n < 3:
      return float('nan')
    span = self._t[-1] - self._t[0]
    if span < 0.5 * self.window_s:
      return float('nan')
    t_bar = sum(self._t) / n
    a_bar = sum(self._a) / n
    num = 0.0
    den = 0.0
    for ti, ai in zip(self._t, self._a):
      dt = ti - t_bar
      num += dt * (ai - a_bar)
      den += dt * dt
    if den <= 0.0:
      return float('nan')
    return num / den

  def is_moving(self, threshold_deg_s=MOTION_THRESHOLD_DEG_S):
    r = self.rate_deg_s
    return r == r and abs(r) >= threshold_deg_s

  def is_moving_with_torque(self, torque_frac, threshold_deg_s=MOTION_THRESHOLD_DEG_S):
    """True when the rack is moving in the direction the torque commands.

    Signed on purpose: on rough pavement or camber the wheel jiggles without
    the rack having broken free the way we asked.
    """
    r = self.rate_deg_s
    if r != r or torque_frac == 0.0:
      return False
    expected_sign = TORQUE_TO_ANGLE_SIGN * (1.0 if torque_frac > 0.0 else -1.0)
    return abs(r) >= threshold_deg_s and (r * expected_sign) > 0.0
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /home/oxygen/catpilot-dev/plugins && PYTHONPATH=.:plugins/bmw_e9x_e8x uv run pytest plugins/bmw_e9x_e8x/tests/test_rack_motion.py -v`
Expected: PASS, 11 passed

- [ ] **Step 5: Commit**

```bash
cd /home/oxygen/catpilot-dev/plugins
git add plugins/bmw_e9x_e8x/bmw/rack_motion.py plugins/bmw_e9x_e8x/tests/test_rack_motion.py
git commit -m "feat(bmw): add offset-immune Delta-angle rack motion observer"
```

---

### Task 2: Online breakaway estimator

**Files:**
- Modify: `plugins/bmw_e9x_e8x/bmw/rack_motion.py` (append `BreakawayEstimator`)
- Test: `plugins/bmw_e9x_e8x/tests/test_rack_motion.py` (append)

**Interfaces:**
- Consumes: `RackMotion.is_moving_with_torque` from Task 1.
- Produces: `BreakawayEstimator(seed_frac=0.20)` with `.update(torque_frac, moving_with_torque) -> None`, `.breakaway_frac -> float`, `.sustain_frac -> float`, `.observations -> int`, `.reset() -> None`. Module constants `BREAKAWAY_SEED_FRAC`, `BREAKAWAY_MIN_FRAC`, `BREAKAWAY_MAX_FRAC`, `BREAKAWAY_ALPHA`, `SUSTAIN_RATIO`.

- [ ] **Step 1: Write the failing test**

Append to `plugins/bmw_e9x_e8x/tests/test_rack_motion.py`:

```python
from bmw.rack_motion import (BreakawayEstimator, BREAKAWAY_SEED_FRAC,
                             BREAKAWAY_MIN_FRAC, BREAKAWAY_MAX_FRAC, SUSTAIN_RATIO)


def test_seed_is_the_measured_knee_not_the_old_friction_constant():
    # Measured knee 2.0-2.75 Nm at STEER_MAX=12 -> 0.167-0.229 frac.
    # The old FRICTION was 0.05. The seed must not be that.
    assert 0.15 <= BREAKAWAY_SEED_FRAC <= 0.25
    assert BreakawayEstimator().breakaway_frac == BREAKAWAY_SEED_FRAC


def test_no_observation_leaves_seed_untouched():
    est = BreakawayEstimator()
    for _ in range(50):
        est.update(0.30, moving_with_torque=False)
    assert est.breakaway_frac == BREAKAWAY_SEED_FRAC
    assert est.observations == 0


def test_records_torque_at_the_stationary_to_moving_transition():
    est = BreakawayEstimator()
    est.update(0.30, moving_with_torque=False)
    est.update(0.30, moving_with_torque=True)      # transition
    assert est.observations == 1
    assert est.breakaway_frac > BREAKAWAY_SEED_FRAC


def test_sustained_motion_records_only_once():
    est = BreakawayEstimator()
    est.update(0.30, moving_with_torque=False)
    for _ in range(20):
        est.update(0.30, moving_with_torque=True)
    assert est.observations == 1


def test_converges_toward_repeated_observations():
    est = BreakawayEstimator()
    for _ in range(60):
        est.update(0.30, moving_with_torque=False)
        est.update(0.30, moving_with_torque=True)
    assert abs(est.breakaway_frac - 0.30) < 0.02


def test_observation_is_clamped_to_sane_range():
    est = BreakawayEstimator()
    for _ in range(200):
        est.update(0.95, moving_with_torque=False)
        est.update(0.95, moving_with_torque=True)
    assert est.breakaway_frac <= BREAKAWAY_MAX_FRAC + 1e-9


def test_sign_is_ignored_only_magnitude_matters():
    a = BreakawayEstimator()
    b = BreakawayEstimator()
    a.update(+0.30, False); a.update(+0.30, True)
    b.update(-0.30, False); b.update(-0.30, True)
    assert abs(a.breakaway_frac - b.breakaway_frac) < 1e-9


def test_sustain_is_a_fraction_of_breakaway():
    est = BreakawayEstimator()
    assert abs(est.sustain_frac - SUSTAIN_RATIO * est.breakaway_frac) < 1e-9
    assert est.sustain_frac < est.breakaway_frac


def test_reset_restores_seed():
    est = BreakawayEstimator()
    est.update(0.30, False); est.update(0.30, True)
    est.reset()
    assert est.breakaway_frac == BREAKAWAY_SEED_FRAC
    assert est.observations == 0


def test_first_ever_sample_moving_is_not_counted_as_a_breakaway():
    """Engaging mid-turn with the wheel already moving must not record."""
    est = BreakawayEstimator()
    est.update(0.15, moving_with_torque=True)
    assert est.observations == 0
    assert est.breakaway_frac == BREAKAWAY_SEED_FRAC


def test_arming_requires_seeing_the_rack_stationary_first():
    est = BreakawayEstimator()
    est.update(0.15, moving_with_torque=True)
    est.update(0.15, moving_with_torque=True)
    assert est.observations == 0
    est.update(0.15, moving_with_torque=False)   # arms here
    est.update(0.15, moving_with_torque=True)
    assert est.observations == 1


def test_reset_disarms_the_edge_detector():
    est = BreakawayEstimator()
    est.update(0.30, moving_with_torque=False)
    est.update(0.30, moving_with_torque=True)
    est.reset()
    est.update(0.15, moving_with_torque=True)
    assert est.observations == 0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/oxygen/catpilot-dev/plugins && PYTHONPATH=.:plugins/bmw_e9x_e8x uv run pytest plugins/bmw_e9x_e8x/tests/test_rack_motion.py -v`
Expected: FAIL — `ImportError: cannot import name 'BreakawayEstimator'`

- [ ] **Step 3: Write the implementation**

Append to `plugins/bmw_e9x_e8x/bmw/rack_motion.py`:

```python
# Seed = measured knee midpoint. Route 3f2 ramp ticks: stuck fraction first
# falls below 50% at 2.50-2.75 Nm (strict action-freshness gate) or
# 2.00-2.25 Nm (permissive gate). 2.4 Nm / STEER_MAX 12 = 0.20 frac.
# The old FRICTION = 0.05 (0.6 Nm) was roughly 4x too low.
BREAKAWAY_SEED_FRAC = 0.20

# Clamps. MIN keeps a pathological low observation from disabling the gates
# that consume this; MAX keeps a stuck-rack episode from ratcheting the
# estimate into the authority cap.
BREAKAWAY_MIN_FRAC = 0.05
BREAKAWAY_MAX_FRAC = 0.40

# EMA weight per observed breakaway. At 0.10 the estimate reaches ~90% of a
# step change in ~22 observations; route 3f2 offers roughly 40 qualifying
# transitions per hour of driving, so this tracks conditions across a drive
# without chasing a single anomalous release.
BREAKAWAY_ALPHA = 0.10

# Static friction exceeds kinetic: at the instant of release the applied
# torque already exceeds what sustains motion. Measured on route 3f2 —
# breakaway 2.5-2.9 Nm, sustained unwinding motion at 1.25-1.5 Nm.
SUSTAIN_RATIO = 0.5


class BreakawayEstimator:
  """Online estimate of the rack's breakaway torque, in torque fraction.

  Records the applied torque at each observed stationary -> moving transition
  and low-passes it. Never needs to know tyre pressure, surface or temperature:
  it re-measures the threshold on every push under whatever conditions apply.
  """

  def __init__(self, seed_frac=BREAKAWAY_SEED_FRAC):
    self._seed = float(seed_frac)
    self.breakaway_frac = float(seed_frac)
    self.observations = 0
    self._was_moving = False
    self._armed = False

  def reset(self):
    self.breakaway_frac = self._seed
    self.observations = 0
    self._was_moving = False
    self._armed = False

  def update(self, torque_frac, moving_with_torque):
    moving = bool(moving_with_torque)
    # Warm-up gate: only count a transition once the rack has been SEEN
    # stationary. The controller constructs this at engagement, and the driver
    # can engage mid-turn with the wheel already moving — recording that as a
    # breakaway would capture the SUSTAIN torque (about half of breakaway) and
    # bias the estimate low, the exact failure this feature exists to remove.
    if moving and not self._was_moving and self._armed and torque_frac != 0.0:
      obs = min(max(abs(float(torque_frac)), BREAKAWAY_MIN_FRAC), BREAKAWAY_MAX_FRAC)
      self.breakaway_frac += BREAKAWAY_ALPHA * (obs - self.breakaway_frac)
      self.observations += 1
    if not moving:
      self._armed = True      # arm AFTER the check, so one sample cannot both arm and fire
    self._was_moving = moving

  @property
  def sustain_frac(self):
    return SUSTAIN_RATIO * self.breakaway_frac
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /home/oxygen/catpilot-dev/plugins && PYTHONPATH=.:plugins/bmw_e9x_e8x uv run pytest plugins/bmw_e9x_e8x/tests/test_rack_motion.py -v`
Expected: PASS, 23 passed

- [ ] **Step 5: Commit**

```bash
cd /home/oxygen/catpilot-dev/plugins
git add plugins/bmw_e9x_e8x/bmw/rack_motion.py plugins/bmw_e9x_e8x/tests/test_rack_motion.py
git commit -m "feat(bmw): add online breakaway torque estimator"
```

---

### Task 3: Offline replay validation against route 3f2

**Files:**
- Create: `plugins/bmw_e9x_e8x/tests/replay_rack_motion.py`

**Interfaces:**
- Consumes: `RackMotion`, `BreakawayEstimator` from Tasks 1–2.
- Produces: a decision record written to stdout. No controller code depends on this; it exists to set `WINDOW_S`, `MOTION_THRESHOLD_DEG_S`, `BREAKAWAY_SEED_FRAC` and `SUSTAIN_RATIO` from data before any control change ships.

**Why this task exists before any controller edit:** replaying a *detector* against logs is valid because it is a pure function of recorded signals. Replaying a *closed loop* is not — changing torque changes the angle response. So detection timing and false-positive rate are settled here, at the desk; loop behaviour needs the car (Task 7).

- [ ] **Step 1: Write the replay harness**

Create `plugins/bmw_e9x_e8x/tests/replay_rack_motion.py`:

```python
"""Replay the rack-motion detector against recorded routes.

Run ON THE C3 (it reads rlogs directly):
    ssh c3
    source /usr/local/venv/bin/activate
    cd /data/openpilot
    PYTHONPATH=/data/openpilot:/data/plugins-runtime/bmw_e9x_e8x \\
      python /tmp/replay_rack_motion.py /data/media/0/realdata/000003f2--a4bbab4676-- 0 50

Reports, per segment and in aggregate:
  - how early the detector would have flagged the stall before each release
  - false-positive rate on ordinary driving (flagged while the wheel was fine)
  - the breakaway estimate's trajectory and final value
"""
import json
import sys

import zstandard
from cereal import log

from bmw.rack_motion import RackMotion, BreakawayEstimator, MOTION_THRESHOLD_DEG_S


def read_segment(path):
    """Yield (t, steering_angle_deg, torque_frac, action) at carState rate."""
    with open(path, 'rb') as f:
        raw = zstandard.ZstdDecompressor().decompress(f.read(), max_output_size=600 * 1024 * 1024)
    cs, lat = [], []
    for e in log.Event.read_multiple_bytes(raw):
        w = e.which()
        if w == 'carState':
            m = e.carState
            cs.append((e.logMonoTime / 1e9, m.steeringAngleDeg, m.steeringPressed, m.vEgo))
        elif w == 'pluginBusLog':
            for ent in e.pluginBusLog.entries:
                if ent.topic != 'bmw_lat_control':
                    continue
                try:
                    d = json.loads(ent.json)
                except Exception:
                    continue
                lat.append((ent.monoTime / 1e9, d.get('output', 0.0), d.get('action', '')))
    cs.sort()
    lat.sort()
    j = 0
    for t, ang, pressed, v in cs:
        while j + 1 < len(lat) and lat[j + 1][0] <= t:
            j += 1
        out, action = (lat[j][1], lat[j][2]) if lat else (0.0, '')
        yield t, ang, out, action, pressed, v


def replay(prefix, lo, hi):
    est = BreakawayEstimator()
    stalls = releases = early_total = 0
    flagged_ticks = push_ticks = 0
    for n in range(lo, hi + 1):
        rm = RackMotion()
        stalled_since = None
        for t, ang, out, action, pressed, v in read_segment(f"{prefix}{n}/rlog.zst"):
            rm.update(t, ang)
            if pressed or v < 5.0 or action != 'ramp':
                stalled_since = None
                continue
            push_ticks += 1
            moving = rm.is_moving_with_torque(out)
            est.update(out, moving)
            stalled = (abs(out) > est.breakaway_frac * 0.5) and not moving
            if stalled:
                flagged_ticks += 1
                if stalled_since is None:
                    stalled_since = t
                    stalls += 1
            else:
                if stalled_since is not None and abs(rm.rate_deg_s) > 10.0:
                    releases += 1
                    early_total += (t - stalled_since)
                stalled_since = None
        print(f"seg {n:2d}: stalls={stalls} releases={releases} breakaway={est.breakaway_frac:.3f}")
    print("\n=== AGGREGATE ===")
    print(f"push ticks              : {push_ticks}")
    print(f"stall episodes flagged  : {stalls}")
    print(f"releases after a stall  : {releases}")
    if releases:
        print(f"mean warning before release: {early_total / releases:.2f} s")
    print(f"flagged fraction of pushes : {flagged_ticks / max(push_ticks, 1):.1%}")
    print(f"final breakaway estimate   : {est.breakaway_frac:.3f} frac"
          f" ({est.breakaway_frac * 12:.2f} Nm) from {est.observations} observations")
    print(f"motion threshold used      : {MOTION_THRESHOLD_DEG_S} deg/s")


if __name__ == '__main__':
    replay(sys.argv[1], int(sys.argv[2]), int(sys.argv[3]))
```

- [ ] **Step 2: Copy to device and run**

```bash
scp /home/oxygen/catpilot-dev/plugins/plugins/bmw_e9x_e8x/tests/replay_rack_motion.py c3:/tmp/replay_rack_motion.py
ssh c3 'source /usr/local/venv/bin/activate && cd /data/openpilot && \
  PYTHONPATH=/data/openpilot:/data/plugins-runtime/bmw_e9x_e8x \
  python /tmp/replay_rack_motion.py /data/media/0/realdata/000003f2--a4bbab4676-- 0 50'
```

Expected: an aggregate block. **Acceptance gates — all four must hold, or tune and re-run before proceeding:**
1. The segment-10 release (monotime 663.9–664.5) is flagged as a stall, with **≥ 0.5 s of warning** before the release.
2. Mean warning before release ≥ 0.3 s.
3. Flagged fraction of push ticks ≤ 25% — higher means the detector fires on ordinary driving and would make the controller timid.
4. Final breakaway estimate lands in **0.15–0.25 frac (1.8–3.0 Nm)**, consistent with the measured knee.

- [ ] **Step 3: Record the outcome in the plan file**

Append the aggregate output verbatim as a `## Task 3 result` section at the bottom of this plan file, and note any constant you changed in `rack_motion.py` and why.

- [ ] **Step 4: Commit**

```bash
cd /home/oxygen/catpilot-dev/plugins
git add plugins/bmw_e9x_e8x/tests/replay_rack_motion.py docs/superpowers/plans/2026-08-12-angle-based-breakaway.md
git commit -m "test(bmw): add offline rack-motion detector replay and record route 3f2 results"
```

---

### Task 4: Wire the observer into the controller — telemetry only, zero control effect

**Files:**
- Modify: `plugins/bmw_e9x_e8x/bmw/latcontroller.py`
- Test: `plugins/bmw_e9x_e8x/tests/test_latcontroller.py`

**Interfaces:**
- Consumes: `RackMotion`, `BreakawayEstimator`.
- Produces: `state['rack_motion']`, `state['breakaway_est']`, `state['rack_rate']`, `state['rack_moving']`; telemetry keys `rack_rate`, `rack_moving`, `breakaway_frac`, `breakaway_obs`.

This task must not change a single commanded torque value. It exists so the estimate can be watched on the car for a drive before it influences anything.

- [ ] **Step 1: Extend the test harness so `CS` carries a steering angle**

This must come first: `_call_update` currently builds a `CS` with no `steeringAngleDeg`, so the Step 3 change would break every existing test. Replace `_call_update` in `plugins/bmw_e9x_e8x/tests/test_latcontroller.py` (line 110) with:

```python
def _call_update(lac, desired_curvature, lat_delay=0.2, v_ego=20.0, active=True,
                 steering_angle_deg=0.0):
  CS = SimpleNamespace(vEgo=v_ego, steeringAngleDeg=steering_angle_deg)
  return lac.update(active, CS, None, None, False, desired_curvature, False, lat_delay)
```

The default of `0.0` keeps every existing call site working unchanged.

- [ ] **Step 2: Write the failing test**

Append to `plugins/bmw_e9x_e8x/tests/test_latcontroller.py`:

```python
def test_rack_motion_state_is_populated_but_torque_is_unchanged(monkeypatch):
    """Observer runs every CAN tick; commanded torque must be bit-identical."""
    from bmw.rack_motion import RackMotion, BreakawayEstimator

    lac_a, sm_a, _, state_a = _make_controller(monkeypatch)
    lac_b, sm_b, _, state_b = _make_controller(monkeypatch)

    outputs_a, outputs_b = [], []
    for i in range(200):
        angle = 2.0 + 0.05 * i
        _set_measured(sm_a, 20.0, 0.001)
        _set_measured(sm_b, 20.0, 0.001)
        outputs_a.append(_call_update(lac_a, 0.002, steering_angle_deg=angle))
        outputs_b.append(_call_update(lac_b, 0.002, steering_angle_deg=angle))

    assert outputs_a == outputs_b
    assert isinstance(state_a['rack_motion'], RackMotion)
    assert isinstance(state_a['breakaway_est'], BreakawayEstimator)
    assert state_a['rack_rate'] == state_a['rack_rate']   # not NaN after 200 ticks


def test_missing_steering_angle_degrades_to_no_motion(monkeypatch):
    """Defensive: a CS without the field must not crash, and must report
    'not moving' rather than fabricating motion from a default."""
    lac, sm, _, state = _make_controller(monkeypatch)
    CS = SimpleNamespace(vEgo=20.0)          # no steeringAngleDeg
    for _ in range(50):
        _set_measured(sm, 20.0, 0.001)
        out = lac.update(True, CS, None, None, False, 0.002, False, 0.2)
    assert out is not None
    assert state['rack_moving'] is False
    assert abs(state['rack_rate']) < 0.55    # constant 0.0 angle -> zero rate
```

- [ ] **Step 3: Run test to verify it fails**

Run: `cd /home/oxygen/catpilot-dev/plugins && PYTHONPATH=.:plugins/bmw_e9x_e8x uv run pytest plugins/bmw_e9x_e8x/tests/test_latcontroller.py -k rack_motion -v`
Expected: FAIL — `KeyError: 'rack_motion'`

- [ ] **Step 4: Write the implementation**

In `latcontroller.py`, add to the module-scope imports (after the `sys.path` block that ends at line 27, beside `import numpy as np`):

```python
import time

from bmw.rack_motion import RackMotion, BreakawayEstimator
```

`time.monotonic()` is used rather than `sec_since_boot` — the latter is not imported in this file and adds an openpilot dependency for no benefit. `math` is already imported at line 178 (function scope); do not add a second import.

Add to the `state` dict initialiser (after `'relax_ticks': 0,`):

```python
    'rack_motion': RackMotion(),   # Delta-angle observer, offset-immune
    'breakaway_est': BreakawayEstimator(),
    'rack_rate': float('nan'),     # debug: windowed steering rate (deg/s)
    'rack_moving': False,          # debug: moving in the commanded direction
```

Inside `update()`, immediately after `pid_log.version = 11` (so it runs on **every** CAN tick, not only livePose ticks):

```python
    # Rack motion observation (2026-08-12). Runs at CAN rate because the whole
    # point is to see the wheel move sooner than livePose can show it. Deltas
    # only — steeringAngleDeg carries a constant ~-1.58 deg alignment offset
    # (physical front-wheel alignment, not a sensor fault) which cancels under
    # differencing. Telemetry only in this revision — nothing reads it yet.
    # getattr guard: CS is a stub in some test paths and must not crash control.
    state['rack_motion'].update(time.monotonic(), float(getattr(CS, 'steeringAngleDeg', 0.0)))
    state['rack_rate'] = state['rack_motion'].rate_deg_s
    state['rack_moving'] = state['rack_motion'].is_moving_with_torque(state['torque'])
    state['breakaway_est'].update(state['torque'], state['rack_moving'])
```

Add to the telemetry `payload` dict (after `'relax_ticks': ...`):

```python
          'rack_rate': float(state['rack_rate']),
          'rack_moving': bool(state['rack_moving']),
          'breakaway_frac': float(state['breakaway_est'].breakaway_frac),
          'breakaway_obs': int(state['breakaway_est'].observations),
```

- [ ] **Step 5: Run the full plugin test suite**

Run: `cd /home/oxygen/catpilot-dev/plugins && PYTHONPATH=. uv run pytest plugins/bmw_e9x_e8x/tests/ -v`
Expected: PASS, all existing tests still green plus the two new ones. The `outputs_a == outputs_b` assertion is the real gate: this task must not move a single commanded torque value.

- [ ] **Step 6: Run the on-device probe harness**

```bash
scp /home/oxygen/catpilot-dev/plugins/plugins/bmw_e9x_e8x/tests/on_device_probe.py c3:/tmp/on_device_probe.py
ssh c3 'source /usr/local/venv/bin/activate && cd /data/openpilot && python /tmp/on_device_probe.py'
```
Expected: all probes PASS (this harness must be run offroad).

- [ ] **Step 7: Commit**

```bash
cd /home/oxygen/catpilot-dev/plugins
git add plugins/bmw_e9x_e8x/bmw/latcontroller.py plugins/bmw_e9x_e8x/tests/test_latcontroller.py
git commit -m "feat(bmw): observe rack motion in latcontroller (telemetry only, no control effect)"
```

- [ ] **Step 8: Deploy and observe for one drive**

```bash
GIT_SSH_COMMAND='ssh -o BatchMode=yes' git push git@github.com:catpilot-dev/plugins.git dev:refs/heads/dev
ssh c3 'cd /data/plugins && GIT_SSL_NO_VERIFY=1 git fetch origin dev && git reset --hard origin/dev && bash install.sh'
```

After the next drive, confirm from the rlog that `breakaway_frac` converged into 0.15–0.25 and that `rack_moving` tracks the angle trace. **Do not proceed to Task 5 until this has been seen on real data.**

---

### Task 5: Replace the three FRICTION consult sites and the HOLD_BAND derivation

**Files:**
- Modify: `plugins/bmw_e9x_e8x/bmw/latcontroller.py`
- Modify: `plugins/bmw_e9x_e8x/plugin.json` (add param)
- Test: `plugins/bmw_e9x_e8x/tests/test_latcontroller.py`

**Interfaces:**
- Consumes: `state['breakaway_est']` from Task 4.
- Produces: param `AngleBreakaway` (`"0"`/`"1"`, default `"0"`); local `breakaway` and `hold_band` replacing the `FRICTION` and `HOLD_BAND` constants at their consult sites.

The three sites, all in `update()`:
1. `deep_relax` gate, ~line 480: `abs(state['torque']) > FRICTION`
2. `cancel_tol` gate, ~line 498: `abs(state['target_frac']) > FRICTION`
3. `HOLD_BAND`, used at ~line 497 (`abs(delta_err) <= 1.2*HOLD_BAND`) and in the on-target branch (`abs(delta_err) <= HOLD_BAND`). Its docstring at ~line 302 derives it as `FRICTION*STEER_MAX / (T_CAP_SLOPE*kappa_scale*v**2)`.

- [ ] **Step 1: Write the failing test**

Append to `plugins/bmw_e9x_e8x/tests/test_latcontroller.py`:

```python
def test_toggle_off_reproduces_legacy_friction_behaviour(monkeypatch):
    lac, sm, mod, state = _make_controller(monkeypatch)
    monkeypatch.setattr(mod, 'read_plugin_param', lambda *a, **k: '0')
    for _ in range(50):
        _set_measured(sm, 20.0, 0.001)
        _call_update(lac, 0.002, steering_angle_deg=2.0)
    assert state['breakaway_used'] == 0.05                # legacy FRICTION
    assert abs(state['hold_band_used'] - 0.001) < 1e-9    # legacy HOLD_BAND


def test_toggle_on_uses_the_live_estimate(monkeypatch):
    lac, sm, mod, state = _make_controller(monkeypatch)
    monkeypatch.setattr(mod, 'read_plugin_param', lambda *a, **k: '1')
    state['breakaway_est'].breakaway_frac = 0.22
    for _ in range(50):
        _set_measured(sm, 20.0, 0.001)
        _call_update(lac, 0.002, steering_angle_deg=2.0)
    assert state['breakaway_used'] == 0.22
    # hold_band is derived from breakaway, so a 4.4x larger breakaway widens it
    assert state['hold_band_used'] > 0.001


def test_hold_band_is_clamped_so_it_cannot_swallow_lane_keeping(monkeypatch):
    lac, sm, mod, state = _make_controller(monkeypatch)
    monkeypatch.setattr(mod, 'read_plugin_param', lambda *a, **k: '1')
    state['breakaway_est'].breakaway_frac = 0.40          # estimator ceiling
    for _ in range(50):
        _set_measured(sm, 20.0, 0.001)
        _call_update(lac, 0.002, steering_angle_deg=2.0)
    assert state['hold_band_used'] <= mod.HOLD_BAND_MAX
```

`HOLD_BAND_MAX` is read off the module object (`mod`) because the constants live inside `on_lat_controller_init`'s scope in some revisions — reading it via `mod` fails loudly if it was placed at the wrong scope, which is the behaviour we want. If it is function-scope, hoist `HOLD_BAND_MAX` / `HOLD_BAND_MIN` to module scope in Step 3 so the test can see them.

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/oxygen/catpilot-dev/plugins && PYTHONPATH=.:plugins/bmw_e9x_e8x uv run pytest plugins/bmw_e9x_e8x/tests/test_latcontroller.py -k "toggle or hold_band" -v`
Expected: FAIL — `KeyError: 'breakaway_used'`

- [ ] **Step 3: Write the implementation**

First add the import at **module scope** in `latcontroller.py`, beside `import time` from Task 4. Module scope is required — the tests monkeypatch `mod.read_plugin_param`, which only works if the name lives on the module:

```python
from config import read_plugin_param
```

`config.py` is deployed to `/data/plugins-runtime/config.py` and the plugin root is already on `sys.path` (lines 20–27), so this resolves both in the repo and on the device.

Add `HOLD_BAND_MAX` / `HOLD_BAND_MIN` at **module scope** too (not inside `on_lat_controller_init`), so the test can read them off `mod`:

```python
  # HOLD_BAND is FRICTION-derived (see the derivation in the comment above:
  # FRICTION*STEER_MAX / (T_CAP_SLOPE*kappa_scale*v**2) ~= 0.001 rad at 25 m/s).
  # With the live breakaway estimate replacing FRICTION the band self-sizes,
  # so it needs a ceiling: at the estimator's 0.40 cap it would otherwise reach
  # ~0.008 rad and start eating the lane_keeping position correction — the
  # Phase 1 failure mode where a 0.0012-0.0021 rad tolerance ate 44% of the
  # anchor's command. 0.004 rad matches the measured breakaway at 25 m/s.
  HOLD_BAND_MAX = 0.004
  HOLD_BAND_MIN = 0.001    # never tighter than the legacy value
```

Add the param helper near the top of `on_lat_controller_init`. It re-reads at most every 5 s — the param lives on disk and a per-CAN-tick read would be 100 file opens a second:

```python
  _param_cache = {'t': 0.0, 'on': False}

  def _angle_breakaway_enabled():
    now = time.monotonic()
    if now - _param_cache['t'] >= 5.0:
      _param_cache['t'] = now
      try:
        _param_cache['on'] = read_plugin_param('bmw_e9x_e8x', 'AngleBreakaway', '') == '1'
      except Exception:
        _param_cache['on'] = False
    return _param_cache['on']
```

Note for the tests: the 5 s cache means a monkeypatched `read_plugin_param` is consulted on the first call and then not again. Each test builds a fresh controller, so the first `_call_update` picks up the patched value — but do not write a test that flips the param mid-run without also resetting `_param_cache['t']`.

Add to the `state` dict initialiser:

```python
    'breakaway_used': FRICTION,    # debug: value actually consulted this tick
    'hold_band_used': HOLD_BAND,   # debug: band actually consulted this tick
```

Inside `update()`, on the livePose tick and **before** the `deep_relax` block, compute both:

```python
      # FRICTION replacement (2026-08-12). FRICTION predicted whether a torque
      # could move the rack; the estimator observes it. Measured knee is
      # 2.0-2.75 Nm against FRICTION's 0.6 Nm, and the knee is not a constant
      # (it spans wider than the whole torque range across conditions), so a
      # re-tuned constant would be wrong in a different way.
      if _angle_breakaway_enabled():
        breakaway = float(state['breakaway_est'].breakaway_frac)
        hold_band = float(np.clip(breakaway * CCP.STEER_MAX
                                  / max(T_CAP_SLOPE_BASE * kappa_scale * v * v, 1e-6),
                                  HOLD_BAND_MIN, HOLD_BAND_MAX))
      else:
        breakaway = FRICTION
        hold_band = HOLD_BAND
      state['breakaway_used'] = breakaway
      state['hold_band_used'] = hold_band
```

Replace the three consult sites:

```python
      # was: and abs(state['torque']) > FRICTION
                    and abs(state['torque']) > breakaway
```

```python
      # was: abs(delta_err) <= 1.2*HOLD_BAND ... and abs(state['target_frac']) > FRICTION
      if (state['action'] == 'ramp' and abs(delta_err) <= 1.2*hold_band
            and state['ramp_frames'] > 0 and abs(state['target_frac']) > breakaway):
```

```python
      # on-target branch, was: if abs(delta_err) <= HOLD_BAND:
        if abs(delta_err) <= hold_band:
```

Add the telemetry keys to `payload` (replacing the fixed `'hold_band': float(HOLD_BAND)`):

```python
          'hold_band': float(state['hold_band_used']),
          'breakaway_used': float(state['breakaway_used']),
```

Add to `plugin.json` `params`:

```json
    "AngleBreakaway": {
      "default": false,
      "description": "Use observed rack motion instead of a fixed friction constant"
    }
```

- [ ] **Step 4: Run the full suite**

Run: `cd /home/oxygen/catpilot-dev/plugins && PYTHONPATH=. uv run pytest plugins/bmw_e9x_e8x/tests/ -v`
Expected: PASS. With the toggle defaulting off, every pre-existing test must be unchanged — that is the regression gate for this task.

- [ ] **Step 5: Commit**

```bash
cd /home/oxygen/catpilot-dev/plugins
git add plugins/bmw_e9x_e8x/bmw/latcontroller.py plugins/bmw_e9x_e8x/plugin.json plugins/bmw_e9x_e8x/tests/test_latcontroller.py
git commit -m "feat(bmw): consult observed breakaway instead of FRICTION, behind AngleBreakaway toggle"
```

---

### Task 6: Stall-aware ramp — stop winding, back off on breakaway

**Files:**
- Modify: `plugins/bmw_e9x_e8x/bmw/latcontroller.py`
- Test: `plugins/bmw_e9x_e8x/tests/test_latcontroller.py`

**Interfaces:**
- Consumes: `state['rack_moving']`, `state['breakaway_est']`, `_angle_breakaway_enabled()`.
- Produces: new action label `breakaway`; the ramp's back-off behaviour.

The failure this fixes: on route 3f2 segment 10 the controller ramped torque from 0.11 to 0.313 over 1.2 s while the wheel delivered 0–21% of commanded motion, then over-delivered 3× within 0.35 s while torque was still rising.

- [ ] **Step 1: Write the failing test**

Append to `plugins/bmw_e9x_e8x/tests/test_latcontroller.py`:

```python
def _drive_angle(lac, sm, mod, state, angles, torque=-0.30):
    """Hold a fixed commanded torque while feeding a steering-angle trajectory."""
    for a in angles:
        state['torque'] = torque
        state['action'] = 'ramp' if state['action'] != 'breakaway' else 'breakaway'
        _set_measured(sm, 20.0, 0.001)
        _call_update(lac, 0.002, steering_angle_deg=a)


def test_backs_off_to_sustain_when_the_rack_breaks_free(monkeypatch):
    lac, sm, mod, state = _make_controller(monkeypatch)
    monkeypatch.setattr(mod, 'read_plugin_param', lambda *a, **k: '1')
    # Negative torque commands LEFT; LEFT is POSITIVE steering angle.
    _drive_angle(lac, sm, mod, state, [2.0 + 1.0 * i for i in range(20)])
    assert state['action'] == 'breakaway'
    expected = -state['breakaway_est'].sustain_frac
    assert abs(state['target_frac'] - expected) < 1e-6


def test_does_not_back_off_while_the_rack_is_still_stuck(monkeypatch):
    lac, sm, mod, state = _make_controller(monkeypatch)
    monkeypatch.setattr(mod, 'read_plugin_param', lambda *a, **k: '1')
    _drive_angle(lac, sm, mod, state, [2.0] * 20)          # frozen wheel
    assert state['action'] != 'breakaway'


def test_motion_opposing_the_command_does_not_trigger_backoff(monkeypatch):
    """Camber or a bump moving the wheel the wrong way is not a breakaway."""
    lac, sm, mod, state = _make_controller(monkeypatch)
    monkeypatch.setattr(mod, 'read_plugin_param', lambda *a, **k: '1')
    # Commanding LEFT (negative torque) while the wheel travels RIGHT.
    _drive_angle(lac, sm, mod, state, [2.0 - 1.0 * i for i in range(20)])
    assert state['action'] != 'breakaway'


def test_toggle_off_never_produces_a_breakaway_action(monkeypatch):
    lac, sm, mod, state = _make_controller(monkeypatch)
    monkeypatch.setattr(mod, 'read_plugin_param', lambda *a, **k: '0')
    _drive_angle(lac, sm, mod, state, [2.0 + 1.0 * i for i in range(20)])
    assert state['action'] != 'breakaway'
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/oxygen/catpilot-dev/plugins && PYTHONPATH=.:plugins/bmw_e9x_e8x uv run pytest plugins/bmw_e9x_e8x/tests/test_latcontroller.py -k breakaway -v`
Expected: FAIL — `assert 'ramp' == 'breakaway'`

- [ ] **Step 3: Write the implementation**

In `update()`, immediately after the rack-observation block added in Task 4 (so it runs at CAN rate, before the ramp step is applied):

```python
    # Stall-aware ramp (2026-08-12). Static friction exceeds kinetic: at the
    # instant the rack breaks free the applied torque already exceeds what
    # sustains motion, so detecting the release is not enough — we must back
    # off. Route 3f2 seg 10: torque was still RISING (to 0.313) 0.35 s after
    # the wheel had begun over-delivering 3x. Continuous, not latched: the
    # rack can re-stick mid-movement.
    if (_angle_breakaway_enabled() and state['action'] in ('ramp', 'breakaway')
        and state['rack_moving'] and state['torque'] != 0.0):
      sustain = math.copysign(state['breakaway_est'].sustain_frac, state['torque'])
      if abs(sustain) < abs(state['torque']):
        state['target_frac'] = sustain
        state['ramp_step'] = (sustain - state['torque']) / max(spread_frames_live, 1)
        state['ramp_frames'] = max(spread_frames_live, 1)
        state['action'] = 'breakaway'
```

`spread_frames_live` must be available at CAN-tick scope. Add to the `state` dict initialiser `'spread_frames': SPREAD_FRAMES_FALLBACK,` and, inside the livePose block where `spread_frames` is computed, add `state['spread_frames'] = spread_frames`. Then use `spread_frames_live = state['spread_frames']` in the block above.

- [ ] **Step 4: Run the full suite**

Run: `cd /home/oxygen/catpilot-dev/plugins && PYTHONPATH=. uv run pytest plugins/bmw_e9x_e8x/tests/ -v`
Expected: PASS. Every pre-existing test must still pass with the toggle off.

- [ ] **Step 5: Update the design doc**

Add a dated section to `plugins/bmw_e9x_e8x/LATERAL_CONTROLLER.md` recording: the measured breakaway (2.0–2.75 Nm vs FRICTION's 0.6 Nm), that the knee is not a constant, the observer's window and threshold, the estimator's seed and EMA, the `breakaway` action, and the `AngleBreakaway` param.

- [ ] **Step 6: Commit**

```bash
cd /home/oxygen/catpilot-dev/plugins
git add plugins/bmw_e9x_e8x/bmw/latcontroller.py plugins/bmw_e9x_e8x/tests/test_latcontroller.py plugins/bmw_e9x_e8x/LATERAL_CONTROLLER.md
git commit -m "feat(bmw): back off to sustain torque when observed rack breakaway occurs"
```

---

### Task 7: On-car A/B and rollback

**Files:**
- Modify: `docs/superpowers/plans/2026-08-12-angle-based-breakaway.md` (append results)

No code changes. This task exists because the closed loop cannot be validated offline: changing torque changes the angle response, so Task 3's replay proves detection only.

- [ ] **Step 1: Deploy with the toggle OFF and drive a baseline**

```bash
GIT_SSH_COMMAND='ssh -o BatchMode=yes' git push git@github.com:catpilot-dev/plugins.git dev:refs/heads/dev
ssh c3 'cd /data/plugins && GIT_SSL_NO_VERIFY=1 git fetch origin dev && git reset --hard origin/dev && bash install.sh'
```

Drive a route with both highway straights and curves. Record the route id.

- [ ] **Step 2: Enable the toggle and drive the same route**

```bash
ssh c3 'echo -n 1 > /data/plugins-runtime/bmw_e9x_e8x/data/AngleBreakaway'
ssh c3 "pkill -f 'selfdrive.ui.ui'"
```

Verify the UI process actually restarted (new PID and etime) — the old `python.*ui_main` pattern matches nothing on 0.11.x.

- [ ] **Step 3: Compare the two routes**

Extract from both rlogs and compare:

| metric | source | pass condition |
|---|---|---|
| wheel-rate std on straights | `carState.steeringAngleDeg` windowed | no worse than baseline |
| residual p2p in curves | same | no worse than baseline |
| torque sign reversals per minute | `bmw_lat_control.output` | no worse than baseline |
| peak `output` | `bmw_lat_control.output` | lower than baseline |
| max curvature overshoot vs `desired` | `bmw_lat_control` | lower than baseline |
| `breakaway_frac` final | `bmw_lat_control` | inside 0.15–0.25 |
| lane offset distribution | `modelV2.laneLines` centre | no worse than baseline |

**The straight-line metrics are the veto.** The route-395 lesson stands: the user's seat is the authoritative sensor, and offline churn metrics are blind to small-correction phase lag.

- [ ] **Step 4: Rollback procedure if any straight-line metric regresses**

```bash
ssh c3 'echo -n 0 > /data/plugins-runtime/bmw_e9x_e8x/data/AngleBreakaway'
ssh c3 "pkill -f 'selfdrive.ui.ui'"
```

The toggle is the rollback — no revert needed. Record what regressed in this plan file before changing any constant.

- [ ] **Step 5: Append results and commit**

```bash
cd /home/oxygen/catpilot-dev/plugins
git add docs/superpowers/plans/2026-08-12-angle-based-breakaway.md
git commit -m "docs(bmw): record angle-breakaway on-car A/B results"
```

---

## Deferred — explicitly NOT in this plan

- **Inner steering-rate / angle-space cascade.** The full restructure where the outer loop hands down a desired increment and an inner loop servos it. Revisit only after Task 7 shows the observer is trustworthy on the car.
- **Relaxing `STEP_MAX`.** The angle feedback licenses a faster ramp — slow ramping was the mitigation for having no stop condition. Do not do both in one change; the A/B would be uninterpretable.
- **`T_CAP_BASE_NM = 2.0`.** Sits inside the measured breakaway range, so on near-straight roads the authority cap may land at roughly the threshold itself. Worth investigating separately.

## Task 3 result

Ran `plugins/bmw_e9x_e8x/tests/replay_rack_motion.py` on the C3 against all 51 segments of route `000003f2--a4bbab4676--`, exactly as specified in Task 3 Step 2. No constants in `rack_motion.py` were changed — none were tuned to make a gate pass, per instruction.

Full command:

```
PYTHONPATH=/data/openpilot:/tmp/rack_pkg python /tmp/replay_rack_motion.py \
  /data/media/0/realdata/000003f2--a4bbab4676-- 0 50
```

(`/tmp/rack_pkg` on the device is a minimal `bmw/` package containing only `rack_motion.py`, copied there because `/data/plugins-runtime/bmw_e9x_e8x/bmw/rack_motion.py` does not exist on the device — the runtime deploy predates Tasks 1–2 and was not redeployed for this task, per the read-only guardrail on `/data/plugins-runtime`. The imported `rack_motion.py` is byte-identical to the committed one, confirmed via the smoke-test import succeeding and matching the module's own constants in the printed output.)

Full output, verbatim:

```
seg  0: stalls=0 releases=0 breakaway=0.200
seg  1: stalls=2 releases=0 breakaway=0.110
seg  2: stalls=2 releases=0 breakaway=0.110
seg  3: stalls=2 releases=0 breakaway=0.110
seg  4: stalls=2 releases=0 breakaway=0.110
seg  5: stalls=2 releases=0 breakaway=0.110
seg  6: stalls=2 releases=0 breakaway=0.110
seg  7: stalls=13 releases=0 breakaway=0.063
seg  8: stalls=36 releases=1 breakaway=0.073
seg  9: stalls=64 releases=1 breakaway=0.091
seg 10: stalls=85 releases=1 breakaway=0.077
seg 11: stalls=113 releases=1 breakaway=0.067
seg 12: stalls=131 releases=1 breakaway=0.069
seg 13: stalls=131 releases=1 breakaway=0.069
seg 14: stalls=142 releases=1 breakaway=0.074
seg 15: stalls=157 releases=1 breakaway=0.064
seg 16: stalls=159 releases=1 breakaway=0.057
seg 17: stalls=159 releases=1 breakaway=0.057
seg 18: stalls=192 releases=2 breakaway=0.075
seg 19: stalls=232 releases=3 breakaway=0.064
seg 20: stalls=255 releases=3 breakaway=0.053
seg 21: stalls=298 releases=5 breakaway=0.062
seg 22: stalls=330 releases=6 breakaway=0.079
seg 23: stalls=340 releases=6 breakaway=0.069
seg 24: stalls=348 releases=6 breakaway=0.067
seg 25: stalls=364 releases=6 breakaway=0.064
seg 26: stalls=384 releases=6 breakaway=0.072
seg 27: stalls=393 releases=6 breakaway=0.069
seg 28: stalls=419 releases=6 breakaway=0.110
seg 29: stalls=434 releases=6 breakaway=0.086
seg 30: stalls=450 releases=6 breakaway=0.073
seg 31: stalls=478 releases=6 breakaway=0.062
seg 32: stalls=494 releases=7 breakaway=0.057
seg 33: stalls=503 releases=7 breakaway=0.063
seg 34: stalls=550 releases=8 breakaway=0.082
seg 35: stalls=570 releases=8 breakaway=0.065
seg 36: stalls=593 releases=8 breakaway=0.059
seg 37: stalls=608 releases=8 breakaway=0.060
seg 38: stalls=649 releases=9 breakaway=0.060
seg 39: stalls=682 releases=9 breakaway=0.055
seg 40: stalls=682 releases=9 breakaway=0.055
seg 41: stalls=682 releases=9 breakaway=0.055
seg 42: stalls=682 releases=9 breakaway=0.055
seg 43: stalls=689 releases=9 breakaway=0.055
seg 44: stalls=689 releases=9 breakaway=0.055
seg 45: stalls=689 releases=9 breakaway=0.055
seg 46: stalls=689 releases=9 breakaway=0.055
seg 47: stalls=689 releases=9 breakaway=0.055
seg 48: stalls=689 releases=9 breakaway=0.055
seg 49: stalls=689 releases=9 breakaway=0.055
seg 50: stalls=689 releases=9 breakaway=0.055

=== AGGREGATE ===
push ticks              : 86723
stall episodes flagged  : 689
releases after a stall  : 9
mean warning before release: 0.79 s
flagged fraction of pushes : 44.4%
final breakaway estimate   : 0.055 frac (0.66 Nm) from 606 observations
motion threshold used      : 2.0 deg/s
```

**Acceptance gates:**

1. **FAIL.** Segment 10 contributed **zero** releases (its running total stays at 1, carried in from segment 8; the per-tick trace shows no `RELEASE` event fires anywhere in segment 10). The physical release the brief cites (rack unwinding from ~663.7s to ~664.3s, angle running from ~3° to ~16.7° at up to 39°/s) is real and present in the log, but the harness's release-counting logic checks `abs(rate) > 10 deg/s` only on the single tick where `stalled` first flips to `False` — and at a genuine breakaway the rate crosses the 2 deg/s "moving" threshold gradually (it was ~2.1 deg/s at that exact tick), so the check almost always misses. By the time the rate has built up to >10 deg/s a few ticks later, `stalled_since` has already been cleared. Measuring by hand instead (last stall onset before the release to release onset): the nearest continuous stall run before 663.7s starts at 663.531s, giving ~0.17 s of warning — also below the 0.5 s bar. Gate 1 fails on both readings.
2. **PASS.** Mean warning before release = 0.79 s ≥ 0.3 s. (Caveat: this average is computed over only 9 counted releases, and gate 1's finding means the counting logic under-counts real releases and may not be representative of a "typical" release event — see analysis in the task-3 report.)
3. **FAIL.** Flagged fraction of push ticks = 44.4%, required ≤ 25%. Off by roughly 1.8x.
4. **FAIL.** Final breakaway estimate = 0.055 frac (0.66 Nm), required 0.15–0.25 frac (1.8–3.0 Nm). This is at the `BREAKAWAY_MIN_FRAC = 0.05` clamp floor — without the clamp the raw EMA would have gone lower still. Off by roughly 3–4x low.

Three of four gates fail. Root-cause analysis, evidence, and recommendation are in `.superpowers/sdd/2026-08-12-angle-based-breakaway/task-3-report.md` — summary: `steeringAngleDeg` updates in a stairstep pattern (the CAN source updates slower than the 100 Hz `carState` rate), which makes the windowed least-squares rate hover near the 2 deg/s `MOTION_THRESHOLD_DEG_S` even while the rack is genuinely stationary. Each spurious crossing is treated as a stationary→moving transition by `BreakawayEstimator`, recording whatever torque happens to be applied at that instant (often small) as a "breakaway" observation. This biases `breakaway_frac` down over hundreds of noise-driven observations, which in turn lowers the `stalled` threshold (`breakaway_frac * 0.5`) used by the harness, which flags more ticks as stalled — a self-reinforcing loop that plausibly explains both the gate-3 and gate-4 failures. No constant was changed to chase these gates; that decision is deferred to the plan owner.
