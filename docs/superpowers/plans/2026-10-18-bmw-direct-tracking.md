# BMW Direct Tracking Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A Driving-panel-switchable BMW lateral mode with no on-target rest state that follows the arc-smoothed reference continuously and takes up servo free play quickly, behind a noise guard, on each change of push direction.

**Architecture:** The free-play logic is pure and lives in a new module `bmw/takeup.py`: contact learning, the sign guard and the take-up state machine. It is unit-tested in isolation. `bmw/latcontroller.py` latches the mode at each engagement. In Direct mode it bypasses only the rest state (`at_rest`, hold_zero/hold_curve, hysteresis, persistent-lean escape, `cancel_tol`), routes the P target through the sign guard, and lets the take-up machine override torque per CAN tick. With the switch off, behaviour is bit-identical to today. `lane_keeping` forces Arc on and Lane Keeping off while Direct mode is latched or switched on.

**Tech Stack:** Python 3, numpy, pytest (`PYTHONPATH=. uv run pytest`, run from `/home/oxygen/catpilot-dev/plugins`), openpilot plugin framework (plugin params in `/data/plugins-runtime/<id>/data/`).

**Spec:** `docs/superpowers/specs/2026-10-18-bmw-direct-tracking-design.md`

## Global Constraints

- Switch param: `DirectTracking` in the `bmw_e9x_e8x` plugin data dir. **Default off.** Only an explicit `'1'` reads as on; an absent file is off.
- The mode takes effect at the next lateral engagement (active false → true). It never changes law mid-push.
- Direct mode applies at all speeds, including LKA below 30 km/h.
- Initial constants (module-level, named):
  - `TAKEUP_NM = 0.35`
  - `CONTACT_TOL_DEG = 1.5`
  - `GUARD_PERSIST_S = 0.15`
  - `SZL_CONFIRM_QUANTA = 3` (× `ANGLE_QUANTUM_DEG = 0.04395`)
  - `TAKEUP_TIMEOUT_S = 0.4`
  - `LEARN_MIN_SAMPLES = 200`
  - `LEARN_TAU_S = 60.0`
  - `LEARN_TQ_NM = 0.45`
  - `LEARN_RATE_DPS = 1.0`
  - `CONTACT_WIDTH_RANGE = (3.0, 25.0)` deg
- Sign convention: controller torque + = right. Contacts are named by torque side: `c_right` = gap under right torque. BMW `steeringAngleDeg` is left-positive, so right torque moves SZL in the **negative** direction.
- Kept in Direct mode, unchanged:
  - the hold-floor (`held_target`, `hold_f`, `hold_cap`)
  - relax-dwell
  - STEP_MAX for the desired torque
  - `t_cap`
  - stall-breakaway v2
  - `ov_*` observers
  - LKA fast-unwind boost
  - cadence and spread
- Old-side torque is shed under STEP_MAX. Take-up fires only when the commanded torque reaches zero heading to the new side; the only un-capped jump is 0 → `TAKEUP_NM`. Take-up runs per CAN tick.
- No `Co-Authored-By` lines in commit messages.

## Review Focus

1. **Switch flipped off while engaged, with the Arc switch also off.** Lane keeping must keep Arc forced on until the controller disengages. Raw curvature into a band-free tracker is documented-unsafe. *(Task 6 test)*
2. **No servo, or a dead gap signal** (`steeringAngleOffsetDeg` stays 0.0). The learner must report no contacts, so take-up never fires and Direct mode degrades to band-free P. *(Task 1 test)*
3. **Driver override or disengage mid-take-up.** The take-up resets and output follows the normal inactive path (0). *(Task 4 test)*
4. **Noise sign flips shorter than the persistence window.** There must be no reversal and no take-up; torque sheds to 0 and waits. *(Task 3 test)*
5. **Stall-breakaway block active** (`sb_block > 0`). Take-up must never fire into the blocked side. *(Task 2 test)*

---

### Task 1: Contact learner

**Files:**
- Create: `plugins/bmw_e9x_e8x/bmw/takeup.py`
- Test: `plugins/bmw_e9x_e8x/tests/test_takeup.py`

**Interfaces:**
- Produces:
  - The Global Constraints constants as module globals in `bmw/takeup.py`.
  - `parse_contacts(text: str) -> tuple[float, float] | None`: parses `"c_right,c_left"`. Returns None on malformed input or a width outside `CONTACT_WIDTH_RANGE`.
  - `format_contacts(c: tuple[float, float]) -> str`: 2 decimals, `"77.60,65.00"`.
  - `class ContactLearner(seed: tuple[float, float] | None = None, dt: float = 0.01)`, with:
    - `observe(gap_deg: float, torque_nm: float, steer_rate_dps: float, ok: bool) -> None`
    - `contacts() -> tuple[float, float] | None`, returning `(c_right, c_left)`
    - `fresh() -> bool`

- [ ] **Step 1: Write the failing tests**

```python
from bmw.takeup import ContactLearner, parse_contacts, format_contacts, LEARN_MIN_SAMPLES

def feed(L, gap, tq, n, rate=0.0, ok=True):
  for _ in range(n):
    L.observe(gap, tq, rate, ok)

def test_learns_both_sides_by_median():
  L = ContactLearner()
  feed(L, 77.6, +0.8, LEARN_MIN_SAMPLES)
  assert L.contacts() is None            # one side only
  feed(L, 65.0, -0.8, LEARN_MIN_SAMPLES)
  assert L.contacts() == pytest.approx((77.6, 65.0))
  assert L.fresh()

def test_ignores_small_torque_moving_wheel_and_not_ok():
  L = ContactLearner()
  feed(L, 10.0, +0.3, 500)               # |T| < LEARN_TQ_NM
  feed(L, 10.0, +0.8, 500, rate=5.0)     # wheel moving
  feed(L, 10.0, +0.8, 500, ok=False)
  feed(L, 10.0, -0.8, 500, ok=False)
  assert L.contacts() is None

def test_seed_used_until_fresh_and_then_replaced():
  L = ContactLearner(seed=(77.0, 64.0))
  assert L.contacts() == (77.0, 64.0) and not L.fresh()
  feed(L, 78.0, +0.8, LEARN_MIN_SAMPLES); feed(L, 65.0, -0.8, LEARN_MIN_SAMPLES)
  assert L.contacts() == pytest.approx((78.0, 65.0))

def test_dead_gap_signal_gives_no_contacts():      # Review Focus 2
  L = ContactLearner()
  feed(L, 0.0, +0.8, 500); feed(L, 0.0, -0.8, 500)
  assert L.contacts() is None                       # width 0 < CONTACT_WIDTH_RANGE[0]

def test_parse_format_roundtrip_and_rejects():
  assert parse_contacts(format_contacts((77.6, 65.0))) == pytest.approx((77.6, 65.0))
  assert parse_contacts('') is None and parse_contacts('x,y') is None
  assert parse_contacts('70.0,69.0') is None        # width 1 deg
```

- [ ] **Step 2: Run the tests to make sure they fail**

Run: `PYTHONPATH=. uv run pytest plugins/bmw_e9x_e8x/tests/test_takeup.py -v`
Expected: FAIL (`ModuleNotFoundError: bmw.takeup`)

- [ ] **Step 3: Implement the constants, `parse_contacts`, `format_contacts` and `ContactLearner` in `bmw/takeup.py`**

Learning rules:
- A sample counts only when all of these hold: `ok`, `|torque_nm| > LEARN_TQ_NM`, and `|steer_rate_dps| < LEARN_RATE_DPS`. The sign of the torque picks the side.
- For each side, buffer the first `LEARN_MIN_SAMPLES` samples and take their median. After that, update with an EMA, `alpha = dt / LEARN_TAU_S`.
- `contacts()` returns the fresh value for a side that is learned, otherwise the seed value. It returns None when either side is missing, or when `|c_right - c_left|` falls outside `CONTACT_WIDTH_RANGE`.
- `fresh()` is true only when both sides are learned this drive.

The module docstring cites the spec and the 4e5 numbers.

- [ ] **Step 4: Run the tests to make sure they pass**

Run: `PYTHONPATH=. uv run pytest plugins/bmw_e9x_e8x/tests/test_takeup.py -v`
Expected: 5 passed

- [ ] **Step 5: Commit**

```bash
git add plugins/bmw_e9x_e8x/bmw/takeup.py plugins/bmw_e9x_e8x/tests/test_takeup.py
git commit -m "bmw: servo-column contact learner for Direct Tracking"
```

---

### Task 2: Sign guard and take-up state machine

**Files:**
- Modify: `plugins/bmw_e9x_e8x/bmw/takeup.py`
- Test: `plugins/bmw_e9x_e8x/tests/test_takeup.py`

**Interfaces:**
- Consumes: `ContactLearner.contacts()` from Task 1.
- Produces:
  - `class SignGuard(persist_s: float = GUARD_PERSIST_S, dt: float = 0.05)`:
    - attributes `side: int` (+1 / −1 / 0) and `suppressed: int` (cumulative)
    - `observe(delta_err: float) -> None` (call at 20 Hz)
    - `allow(target_frac: float) -> float`
    - `reset() -> None`
  - `class FreePlayTakeUp(learner: ContactLearner, steer_max_nm: float, dt: float = 0.01)`:
    - attributes `state: str` (one of `'unknown' | 'right' | 'left' | 'taking_up'`), `fires: int`, `timeouts: int`, `last_ms: float`
    - `seat(gap_deg: float) -> int`
    - `step(torque_frac: float, ramp_step: float, gap_deg: float, angle_deg: float, blocked: bool) -> float | None`
    - `reset() -> None`

- [ ] **Step 1: Write the failing tests**

```python
from bmw.takeup import SignGuard, FreePlayTakeUp, ContactLearner, TAKEUP_NM, TAKEUP_TIMEOUT_S

def test_guard_first_push_commits_side():
  g = SignGuard(); g.observe(+0.002)
  assert g.allow(+0.1) == 0.1 and g.side == +1

def test_guard_suppresses_short_reversal_then_allows_persisted():   # Review Focus 4
  g = SignGuard(); g.observe(+0.002); g.allow(+0.1)
  g.observe(-0.002); g.observe(-0.002)                  # 0.10 s < 0.15 s
  assert g.allow(-0.1) == 0.0 and g.suppressed == 1 and g.side == +1
  g.observe(-0.002)                                     # 3 ticks = 0.15 s
  assert g.allow(-0.1) == -0.1 and g.side == -1

def test_guard_flicker_never_reverses():
  g = SignGuard(); g.observe(+0.002); g.allow(+0.1)
  for k in range(20):
    g.observe(-0.002 if k % 2 else +0.002)
    assert g.allow(-0.1) == 0.0
  assert g.side == +1

def _tu():
  L = ContactLearner(seed=(77.6, 65.0))
  return FreePlayTakeUp(L, steer_max_nm=12.0)

def test_takeup_fires_at_zero_crossing_toward_unseated_side():
  t = _tu()
  out = t.step(torque_frac=0.0, ramp_step=+0.002, gap_deg=65.0, angle_deg=0.0, blocked=False)
  assert out == pytest.approx(TAKEUP_NM / 12.0) and t.state == 'taking_up' and t.fires == 1

def test_no_takeup_when_already_seated_or_still_shedding():
  t = _tu()
  assert t.step(0.0, +0.002, 77.6, 0.0, False) is None          # seated right already
  assert t.step(+0.05, -0.002, 77.6, 0.0, False) is None        # shedding old side, not at zero yet

def test_handover_on_gap_contact():
  t = _tu(); t.step(0.0, +0.002, 65.0, 0.0, False)
  assert t.step(TAKEUP_NM / 12, +0.002, 70.0, 0.0, False) == pytest.approx(TAKEUP_NM / 12)
  assert t.step(TAKEUP_NM / 12, +0.002, 77.0, 0.0, False) is None and t.state == 'right'

def test_handover_on_szl_confirmation():
  t = _tu(); t.step(0.0, +0.002, 65.0, 10.0, False)
  assert t.step(TAKEUP_NM / 12, +0.002, 70.0, 10.0 - 0.14, False) is None   # right torque -> SZL negative, 3 quanta

def test_timeout_hands_over():
  t = _tu(); t.step(0.0, +0.002, 65.0, 0.0, False)
  for _ in range(round(TAKEUP_TIMEOUT_S / 0.01) - 1):
    assert t.step(TAKEUP_NM / 12, +0.002, 66.0, 0.0, False) is not None
  assert t.step(TAKEUP_NM / 12, +0.002, 66.0, 0.0, False) is None and t.timeouts == 1

def test_blocked_or_unknown_contacts_never_fire():                # Review Focus 5 / 2
  assert _tu().step(0.0, +0.002, 65.0, 0.0, blocked=True) is None
  t = FreePlayTakeUp(ContactLearner(), steer_max_nm=12.0)
  assert t.step(0.0, +0.002, 65.0, 0.0, False) is None
```

- [ ] **Step 2: Run the tests to make sure they fail**

Run: `PYTHONPATH=. uv run pytest plugins/bmw_e9x_e8x/tests/test_takeup.py -v`
Expected: FAIL (`ImportError: SignGuard`)

- [ ] **Step 3: Implement `SignGuard` and `FreePlayTakeUp` in `bmw/takeup.py`**

`SignGuard`:
- `observe` counts consecutive ticks whose `sign(delta_err)` is opposite to `side`. A same-sign or zero tick resets the count.
- `allow` with `side == 0` commits `sign(target)` and passes the target.
- A target on the same side passes.
- An opposite target passes and commits only once the count reaches `round(persist_s / dt)`. Before that it returns `0.0` and increments `suppressed`.

`FreePlayTakeUp`:
- `seat` returns ±1 when the gap is within `CONTACT_TOL_DEG` of `c_right` / `c_left`, else 0.
- Fire when all of the following hold. On fire, record `angle0`, zero the timer, and return `s * TAKEUP_NM / steer_max_nm`:
  - contacts are known and not `blocked`;
  - `s = sign(ramp_step) != 0`;
  - `torque_frac * s <= 0` and `(torque_frac + ramp_step) * s > 0`;
  - `seat(gap) != s`.
- While taking up, hand over (return None, set `last_ms`, set `state` from the seat) on the first of:
  - `|gap - c(s)| < CONTACT_TOL_DEG`;
  - `(angle_deg - angle0) * -s >= SZL_CONFIRM_QUANTA * 0.04395`;
  - elapsed `>= TAKEUP_TIMEOUT_S`, which also increments `timeouts`.
- Otherwise keep returning the take-up torque.
- Count the timeout in ticks (`round(TAKEUP_TIMEOUT_S / dt)`), not as a float-time sum, so the 40th tick hands over exactly.
- When not taking up, `state` is `'right'` / `'left'` / `'unknown'` from the seat.

- [ ] **Step 4: Run the tests to make sure they pass**

Run: `PYTHONPATH=. uv run pytest plugins/bmw_e9x_e8x/tests/test_takeup.py -v`
Expected: 14 passed

- [ ] **Step 5: Commit**

```bash
git add plugins/bmw_e9x_e8x/bmw/takeup.py plugins/bmw_e9x_e8x/tests/test_takeup.py
git commit -m "bmw: sign guard and free-play take-up state machine"
```

---

### Task 3: Direct mode in the decision path (latch, no rest state, sign guard)

**Files:**
- Modify: `plugins/bmw_e9x_e8x/bmw/latcontroller.py`:
  - module scope near `CONTROL_HORIZON_S`
  - `on_lat_controller_init` param block (~L387)
  - the `state` dict (~L575)
  - the `cancel_tol` gate (~L1129)
  - the rest-state block (~L1205–1243)
  - before the step cap (~L1303)
- Test: `plugins/bmw_e9x_e8x/tests/test_latcontroller.py` (new class `TestDirectTracking`)

**Interfaces:**
- Consumes: `SignGuard` from Task 2.
- Produces:
  - module-level `direct_latched() -> bool`, which reads module global `_DIRECT_LATCHED`;
  - `state['direct']: bool`;
  - `state['guard']: SignGuard`;
  - `state['was_active']: bool`.

- [ ] **Step 1: Write the failing tests**

Extend `_make_controller` with a `direct=None` kwarg. Its `_param_stub` returns `'1'`/`'0'` for key `'DirectTracking'`. Engage with one `active=False` call, then `active=True` calls.

```python
class TestDirectTracking:
  def _engaged(self, monkeypatch, direct):
    lac, fake_sm, mod, state = _make_controller(monkeypatch, direct=direct)
    _set_measured(fake_sm, 25.0, 0.0)
    _call_update(lac, 0.0, v_ego=25.0, active=False)
    _call_update(lac, 0.0, v_ego=25.0, active=True)
    return lac, fake_sm, mod, state

  def test_latch_reads_switch_at_engagement(self, monkeypatch):
    lac, fake_sm, mod, state = self._engaged(monkeypatch, True)
    assert state['direct'] and mod.direct_latched()

  def test_off_by_default_keeps_rest_state(self, monkeypatch):
    lac, fake_sm, mod, state = self._engaged(monkeypatch, None)
    assert not state['direct']
    state['tick_count'] = 999
    _call_update(lac, 0.0003, v_ego=25.0)       # inside HOLD_BAND_ENTER
    assert state['action'] == 'hold_zero'

  def test_sub_band_error_is_tracked_in_direct(self, monkeypatch):   # the 4e5 arc-only case
    lac, fake_sm, mod, state = self._engaged(monkeypatch, True)
    state['tick_count'] = 999
    _call_update(lac, 0.0003, v_ego=25.0)       # δ_err ≈ 0.0008 rad < 0.0015
    assert state['action'] == 'ramp' and state['target_frac'] > 0.0 and not state['at_rest']

  def test_no_cancel_tol_in_direct(self, monkeypatch):
    lac, fake_sm, mod, state = self._engaged(monkeypatch, True)
    state.update(action='ramp', ramp_frames=10, torque=0.05, target_frac=0.08)
    _call_update(lac, 0.0, v_ego=25.0)          # δ_err in 1.2·HOLD_BAND
    assert state['action'] != 'cancel_tol'

  def test_short_reversal_sheds_to_zero_not_across(self, monkeypatch):  # Review Focus 4
    lac, fake_sm, mod, state = self._engaged(monkeypatch, True)
    state['tick_count'] = 999; _call_update(lac, +0.002, v_ego=25.0)   # commit right
    state['tick_count'] = 999; _call_update(lac, -0.002, v_ego=25.0)   # one tick opposite
    assert state['target_frac'] >= 0.0

  def test_direct_applies_in_lka_regime(self, monkeypatch):        # user ruling: all speeds
    # same as test_sub_band_error_is_tracked_in_direct at v_ego = _set_measured v = 5.0 m/s
    ...

  def test_hold_floor_kept_in_curves(self, monkeypatch):
    # same scenario as the existing hold-floor test, with direct=True: a same-direction
    # sub-hold P target is floored at held_target
    ...
```

The executor fills in `test_direct_applies_in_lka_regime` as described in its comment, and `test_hold_floor_kept_in_curves` by copying the existing hold-floor test's setup with `direct=True` and the same assertion.

- [ ] **Step 2: Run the tests to make sure they fail**

Run: `PYTHONPATH=. uv run pytest plugins/bmw_e9x_e8x/tests/test_latcontroller.py -k DirectTracking -v`
Expected: FAIL (`KeyError: 'direct'` / `AttributeError: direct_latched`)

- [ ] **Step 3: Implement in `latcontroller.py`**

- **Engagement latch.** On the edge (`active and not state['was_active']`), read `read_plugin_param('bmw_e9x_e8x', 'DirectTracking', '') == '1'` inside a try. If the read fails, the mode is off. Set `state['direct']` and the module `_DIRECT_LATCHED`, and call `state['guard'].reset()`. Clear `_DIRECT_LATCHED` when inactive. Update `state['was_active']` every tick.
- **Rest state.** When `state['direct']`, force `state['at_rest'] = False` and skip the rest-state decision, so the else branch always computes the P target.
- **cancel_tol.** Gate the `cancel_tol` branch with `not state['direct']`.
- **Guard.** Call `state['guard'].observe(delta_err)` on every livePose tick. In the decision, after the hold-floor and relax-dwell lines and before `step_max`, apply `target_frac = state['guard'].allow(target_frac)` when direct.

With direct off, every existing path must be unchanged. The full existing file must still pass.

- [ ] **Step 4: Run the tests to make sure they pass**

Run: `PYTHONPATH=. uv run pytest plugins/bmw_e9x_e8x/tests/ -q`
Expected: all pass (existing + new)

- [ ] **Step 5: Commit**

```bash
git add plugins/bmw_e9x_e8x/bmw/latcontroller.py plugins/bmw_e9x_e8x/tests/test_latcontroller.py
git commit -m "bmw: Direct Tracking decision path - no rest state, sign-guarded reversals"
```

---

### Task 4: Take-up per CAN tick, contact learning and persistence, telemetry

**Files:**
- Modify: `plugins/bmw_e9x_e8x/bmw/latcontroller.py`:
  - init: seed from `DirectContacts`
  - the per-CAN-tick ramp application (~L1350)
  - the disengage edge
  - the telemetry payload (~L1380)
- Test: `plugins/bmw_e9x_e8x/tests/test_latcontroller.py` (`TestDirectTracking`)

**Interfaces:**
- Consumes:
  - `ContactLearner`, `FreePlayTakeUp`, `parse_contacts`, `format_contacts` (Tasks 1–2)
  - `state['direct']` (Task 3)
- Produces:
  - `state['learner']`, `state['takeup']`;
  - telemetry keys `direct`, `tu_state`, `c_right`, `c_left` (0.0 when unknown), `tu_fires`, `tu_ms`, `tu_timeouts`, `tu_guarded`.

- [ ] **Step 1: Write the failing tests**

`_call_update` gains `gap_deg=0.0`, `pressed=False` and `steer_rate=0.0` kwargs, which it puts on the CS SimpleNamespace as `steeringAngleOffsetDeg`, `steeringPressed` and `steeringRateDeg`. `_make_controller` gains a `contacts=None` kwarg for `DirectContacts`.

```python
  def test_takeup_overrides_torque_on_reversal(self, monkeypatch):
    lac, fake_sm, mod, state = self._engaged(monkeypatch, True)   # with contacts='77.60,65.00'
    state.update(torque=0.0, ramp_step=+0.002, ramp_frames=20)
    _call_update(lac, 0.002, v_ego=25.0, gap_deg=65.0)
    assert state['takeup'].state == 'taking_up'
    from bmw.takeup import TAKEUP_NM
    assert state['torque'] == pytest.approx(TAKEUP_NM / 12.0)   # CCP.STEER_MAX = 12 Nm (imported inside init, not a module attr)

  def test_takeup_off_when_not_direct(self, monkeypatch): ...      # same, direct=None -> ramp only

  def test_disengage_mid_takeup_resets(self, monkeypatch):         # Review Focus 3
    ... start take-up, then _call_update(active=False) -> output 0, takeup.state != 'taking_up'

  def test_contacts_persisted_on_disengage_when_fresh(self, monkeypatch):
    ... feed fresh samples both sides via state['learner'].observe, disengage,
    assert config.write_plugin_param called with ('bmw_e9x_e8x', 'DirectContacts', '77.60,65.00')

  def test_telemetry_has_direct_fields(self, monkeypatch): ...     # keys listed in Interfaces
```

- [ ] **Step 2: Run the tests to make sure they fail**

Run: `PYTHONPATH=. uv run pytest plugins/bmw_e9x_e8x/tests/test_latcontroller.py -k DirectTracking -v`
Expected: the new tests FAIL

- [ ] **Step 3: Implement**

- **Init.** `state['learner'] = ContactLearner(seed=parse_contacts(read_plugin_param(..., 'DirectContacts', '')))` and `state['takeup'] = FreePlayTakeUp(state['learner'], CCP.STEER_MAX)`.
- **Learning, every CAN tick.** Call `learner.observe(CS.steeringAngleOffsetDeg, state['torque'] * CCP.STEER_MAX, CS.steeringRateDeg, ok=active and not CS.steeringPressed)`. This runs in both modes, so contacts are learned before the first Direct drive. Read the fields with `getattr` defaults.
- **Take-up, per CAN tick in Direct mode, before the ramp application.** Call `takeup.step(state['torque'], state['ramp_step'], gap, _angle, blocked=state['sb_block'] > 0)`.
  - If it returns a value, set `state['torque']` to it and do not apply or decrement the ramp this tick.
  - On handover, if `ramp_frames > 0`, recompute `ramp_step = (target_frac - torque) / ramp_frames`.
- **Inactive.** Call `takeup.reset()`.
- **Disengage edge.** If `learner.fresh()`, call `write_plugin_param('bmw_e9x_e8x', 'DirectContacts', format_contacts(...))` inside a try.
- **Telemetry.** Add the keys. `tu_guarded` is `guard.suppressed`. `action` reads `'take_up'` while taking up.

- [ ] **Step 4: Run the tests to make sure they pass**

Run: `PYTHONPATH=. uv run pytest plugins/bmw_e9x_e8x/tests/ -q`
Expected: all pass

- [ ] **Step 5: Commit**

```bash
git add plugins/bmw_e9x_e8x/bmw/latcontroller.py plugins/bmw_e9x_e8x/tests/test_latcontroller.py
git commit -m "bmw: Direct Tracking free-play take-up, contact learning, telemetry"
```

---

### Task 5: Driving-panel switch

**Files:**
- Modify: `plugins/bmw_e9x_e8x/register.py` (`on_vehicle_settings`, after the Stall Breakaway row)
- Modify: `plugins/bmw_e9x_e8x/plugin.json` (declare param `DirectTracking`, default false). A manifest change needs a C3 reboot on deploy.
- Modify: `plugins/bmw_e9x_e8x/LATERAL_CONTROLLER.md`: a short "Direct Tracking mode" section linking the spec.
- Test: `plugins/bmw_e9x_e8x/tests/test_hooks.py`

**Interfaces:**
- Produces: the row title `"Direct Tracking"`, which writes `DirectTracking` as `'1'` / `'0'`.

- [ ] **Step 1: Write the failing test.** Update the existing titles assertion to `['Temperature Overlay', 'Stall Breakaway', 'Direct Tracking', 'Resume Button Repurposed']`. Add a test that the row's initial state is off when the param file is absent and on only for `'1'`, and that its callback writes `'1'` / `'0'`.

- [ ] **Step 2: Run** `PYTHONPATH=. uv run pytest plugins/bmw_e9x_e8x/tests/test_hooks.py -v`. Expected: FAIL on the titles list.

- [ ] **Step 3: Implement the row.** Copy, fixed:
  - title `"Direct Tracking"`
  - description `"Steer continuously along the arc-smoothed path with no on-target hold band, taking up servo free play quickly on each change of direction. Forces Arc Smoothing on and Lane Keeping off. Applies at the next engagement."`

- [ ] **Step 4: Run** `PYTHONPATH=. uv run pytest plugins/bmw_e9x_e8x/tests/ -q`. Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add plugins/bmw_e9x_e8x/register.py plugins/bmw_e9x_e8x/plugin.json plugins/bmw_e9x_e8x/LATERAL_CONTROLLER.md plugins/bmw_e9x_e8x/tests/test_hooks.py
git commit -m "bmw: Direct Tracking Driving-panel switch"
```

---

### Task 6: Lane keeping follows Direct mode

**Files:**
- Modify: `plugins/lane_keeping/register.py` (`on_curvature_correction`, add `_direct_tracking_on()`)
- Test: `plugins/lane_keeping/tests/test_register.py`

**Interfaces:**
- Consumes: `direct_latched()` from Task 3, looked up at call time with `sys.modules.get('plugins.bmw_e9x_e8x.bmw.latcontroller')`. Never import a mutable value (registry module-identity lesson).
- Produces: `_direct_tracking_on() -> bool`. It returns true when `direct_latched()` is true, or when the cached BMW switch value (`DirectTracking == '1'`, refreshed every 100 ticks) is true.

- [ ] **Step 1: Write the failing tests**

```python
def test_direct_forces_arc_on_and_lane_keep_off(monkeypatch): ...
  # fake latcontroller module in sys.modules with direct_latched() -> True; user switches
  # LaneKeepEnable='1', LaneKeepArcSmooth='0'; after one call: cfg.arc_smooth True, cfg.enable False

def test_switch_off_while_latched_keeps_arc(monkeypatch):    # Review Focus 1
  # param DirectTracking='0' but direct_latched() True -> arc stays forced on

def test_user_switches_respected_when_direct_off(monkeypatch): ...
  # no latch, param absent -> cfg follows LaneKeepEnable / LaneKeepArcSmooth as today
```

- [ ] **Step 2: Run** `PYTHONPATH=. uv run pytest plugins/lane_keeping/tests/test_register.py -v`. Expected: FAIL.

- [ ] **Step 3: Implement**
- Check `direct_latched()` on every call; it is cheap.
- Read the BMW switch file only in the existing 100-tick refresh. Use `config.read_plugin_param('bmw_e9x_e8x', 'DirectTracking', '')` in a try, with a function-scope import, as the BMW plugin does.
- When direct: `cfg.enable = False` and `cfg.arc_smooth = True`. Otherwise the existing switch reads apply.

- [ ] **Step 4: Run** `PYTHONPATH=. uv run pytest plugins/ -q`. Expected: the full suite passes (baseline 1119 passed, 21 skipped, plus the new tests).

- [ ] **Step 5: Commit**

```bash
git add plugins/lane_keeping/register.py plugins/lane_keeping/tests/test_register.py
git commit -m "lane_keeping: force Arc on / Lane Keeping off under BMW Direct Tracking"
```

---

Deployment to the C3 and the test drive are the user's call after review. They are not part of this plan.
