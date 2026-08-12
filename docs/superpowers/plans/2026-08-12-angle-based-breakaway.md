# Push Budget — Implementation Plan

**Goal:** Make the lateral controller behave like a driver: push harder until the wheel actually moves, then stop pushing harder and ease off as fast as needed.

**The rule, in full:** remember the steering angle when a push starts. While the wheel has moved less than **2°** in the commanded direction, ramp torque as today. Once it has moved 2°, stop increasing torque and let the P law shed it at whatever rate it asks.

**Why:** on route 3f2 segment 10 the controller wound torque 0.11 → 0.313 frac over 1.2 s into a rack that was delivering 0–21% of the commanded motion, then the rack broke free and the wheel swung to 24.8°. The 2° budget is spent at t≈663.87 with torque at 2.8 Nm, against the 3.75 Nm peak actually reached at 664.30 — and it does **not** fire during the preceding 1.4 s stall, which accumulated only 1.63°.

**Why 2°:** it is an *authority budget*, not a detection threshold — how much wheel movement the controller may cause open-loop before feedback must take over. Whether that movement was creep or breakaway is irrelevant, which is why it needs no margin. Physically 2° is 0.11° of front wheel = curvature 0.00070 /m ≈ 1440 m radius ≈ 80% of the real curve the car was in, robust across steerRatio 18–21. It is also 45 quanta of the 0.04395° angle signal, so no signal noise can spend it.

**Why not the v1 design** (deleted by this plan): v1 learned breakaway torque online and thresholded on angular *rate*. Offline validation killed both. Rate thresholding produced 1–2 tick false-motion blips at exactly the low rates that matter, and a debounce did not fix it. The estimator's observations had a median torque of 0.041 frac — the controller's own median push — so it was measuring "torque present when the wheel happens to move", not breakaway; it collapsed to its clamp floor. Edge detection and the offline stuck-fraction-knee method measure different things.

## Global Constraints

- **Sign convention (measured):** `steeringAngleDeg` positive = LEFT; torque fraction negative = LEFT. They are OPPOSITE.
- **Angle offset:** `steeringAngleDeg` carries a constant ~−1.58° *physical* alignment offset (the sensor is accurate). Work only in deltas from a captured reference; never read absolute angle.
- **`read_plugin_param` is NOT imported** in `latcontroller.py` — add it at module scope (tests monkeypatch `mod.read_plugin_param`). **`sec_since_boot` is NOT imported and must not be used** — use `time.monotonic()`.
- `math` is already imported at line 178 (function scope). `CCP.STEER_MAX == 12`.
- **Test harness, exact:** `_make_controller(monkeypatch, wheelbase=2.66)` returns `(lac, fake_sm, mod, state)`; `_call_update(lac, desired_curvature, lat_delay=0.2, v_ego=20.0, active=True)` builds `CS = SimpleNamespace(vEgo=v_ego)` with **no `steeringAngleDeg`**; `_set_measured(fake_sm, v, kappa_meas)` seeds livePose.
- **Tests:** `cd /home/oxygen/catpilot-dev/plugins && PYTHONPATH=. uv run pytest plugins/bmw_e9x_e8x/tests/ -v`. Check `echo $PYTHONPATH` first — if it names `sign_vision` or anything outside this repo, override it.
- **Commits:** no `Co-Authored-By`.
- **Do not touch** `FRICTION`, `HOLD_BAND`, `DRIFT_M`, `KN_*`, `KD_*`, `RELAX_DWELL_*`, `HOLD_CAP_DRIFT_M`. See "Deferred".
- **Device is `ssh c3` only**, read-only.

---

### Task 1: The push budget in the controller

**Files:**
- Modify: `plugins/bmw_e9x_e8x/bmw/latcontroller.py`
- Modify: `plugins/bmw_e9x_e8x/plugin.json`
- Modify: `plugins/bmw_e9x_e8x/tests/test_latcontroller.py`
- Delete: `plugins/bmw_e9x_e8x/bmw/rack_motion.py`, `plugins/bmw_e9x_e8x/tests/test_rack_motion.py`, `plugins/bmw_e9x_e8x/tests/replay_rack_motion.py` (v1 leftovers; nothing imports them — confirm with `grep -rn rack_motion plugins/`)

- [ ] **Step 1: Extend the test harness so `CS` carries a steering angle**

Must come first — otherwise Step 3 breaks every existing test. Replace `_call_update` (line 110) in `tests/test_latcontroller.py`:

```python
def _call_update(lac, desired_curvature, lat_delay=0.2, v_ego=20.0, active=True,
                 steering_angle_deg=0.0):
  CS = SimpleNamespace(vEgo=v_ego, steeringAngleDeg=steering_angle_deg)
  return lac.update(active, CS, None, None, False, desired_curvature, False, lat_delay)
```

The `0.0` default keeps every existing call site unchanged.

- [ ] **Step 2: Write the failing tests**

Append to `tests/test_latcontroller.py`:

```python
def _drive(lac, sm, state, angles, torque=-0.30, desired=0.002):
    """Hold a commanded torque while feeding a steering-angle trajectory."""
    for a in angles:
        state['torque'] = torque
        state['action'] = 'ramp'
        _set_measured(sm, 20.0, 0.001)
        _call_update(lac, desired, steering_angle_deg=a)


def test_budget_unspent_while_the_wheel_barely_moves(monkeypatch):
    lac, sm, mod, state = _make_controller(monkeypatch)
    monkeypatch.setattr(mod, 'read_plugin_param', lambda *a, **k: '1')
    _drive(lac, sm, state, [0.0, 0.4, 0.8, 1.2, 1.5, 1.8])
    assert state['budget_spent'] is False


def test_budget_spent_after_two_degrees_in_the_commanded_direction(monkeypatch):
    lac, sm, mod, state = _make_controller(monkeypatch)
    monkeypatch.setattr(mod, 'read_plugin_param', lambda *a, **k: '1')
    # Negative torque commands LEFT; LEFT is POSITIVE steering angle.
    _drive(lac, sm, state, [0.0, 0.5, 1.0, 1.5, 2.0, 2.5])
    assert state['budget_spent'] is True


def test_movement_opposing_the_command_does_not_spend_it(monkeypatch):
    """Camber or a bump moving the wheel the wrong way is not our doing."""
    lac, sm, mod, state = _make_controller(monkeypatch)
    monkeypatch.setattr(mod, 'read_plugin_param', lambda *a, **k: '1')
    _drive(lac, sm, state, [0.0, -0.5, -1.0, -1.5, -2.0, -2.5])
    assert state['budget_spent'] is False


def test_reference_resets_when_the_push_ends(monkeypatch):
    lac, sm, mod, state = _make_controller(monkeypatch)
    monkeypatch.setattr(mod, 'read_plugin_param', lambda *a, **k: '1')
    _drive(lac, sm, state, [0.0, 1.0, 2.0, 3.0])
    assert state['budget_spent'] is True
    state['action'] = 'hold_zero'
    _set_measured(sm, 20.0, 0.001)
    _call_update(lac, 0.002, steering_angle_deg=3.0)
    assert state['budget_spent'] is False
    _drive(lac, sm, state, [3.0, 3.5])          # new push from 3.0
    assert state['budget_spent'] is False


def test_offset_cancels(monkeypatch):
    """A constant alignment offset must not change when the budget is spent."""
    for base in (0.0, -1.58, +7.3):
        lac, sm, mod, state = _make_controller(monkeypatch)
        monkeypatch.setattr(mod, 'read_plugin_param', lambda *a, **k: '1')
        _drive(lac, sm, state, [base + 0.5 * i for i in range(5)])
        assert state['budget_spent'] is True


def test_toggle_off_never_spends_the_budget(monkeypatch):
    lac, sm, mod, state = _make_controller(monkeypatch)
    monkeypatch.setattr(mod, 'read_plugin_param', lambda *a, **k: '0')
    _drive(lac, sm, state, [0.0 + 0.5 * i for i in range(10)])
    assert state['budget_spent'] is False


def test_missing_steering_angle_degrades_safely(monkeypatch):
    lac, sm, mod, state = _make_controller(monkeypatch)
    CS = SimpleNamespace(vEgo=20.0)          # no steeringAngleDeg
    for _ in range(20):
        _set_measured(sm, 20.0, 0.001)
        out = lac.update(True, CS, None, None, False, 0.002, False, 0.2)
    assert out is not None
    assert state['budget_spent'] is False
```

- [ ] **Step 3: Run to verify they fail**

Run: `cd /home/oxygen/catpilot-dev/plugins && PYTHONPATH=. uv run pytest plugins/bmw_e9x_e8x/tests/test_latcontroller.py -k budget -v`
Expected: FAIL — `KeyError: 'budget_spent'`

- [ ] **Step 4: Implement**

Module-scope imports in `latcontroller.py`, beside `import numpy as np`:

```python
import time

from config import read_plugin_param
```

Constant, in the constants block beside `STEP_MAX_BP`:

```python
  # Steering-wheel movement one open-loop push may cause before feedback takes
  # over. Human-style: push harder until the wheel moves, then stop pushing and
  # ease off. 2 deg is 0.11 deg of front wheel (curvature 0.00070 /m, ~1440 m
  # radius) — a real steering input, and 45 quanta of the 0.04395 deg angle
  # signal, so no noise can spend it. Route 3f2 seg 10: spent at 2.8 Nm against
  # the 3.75 Nm the ramp actually reached.
  BUDGET_DEG = 2.0
```

Param helper near the top of `on_lat_controller_init` (the param is a file; cache it — a per-CAN-tick read would be 100 opens/second):

```python
  _param_cache = {'t': 0.0, 'on': False}

  def _budget_enabled():
    now = time.monotonic()
    if now - _param_cache['t'] >= 5.0:
      _param_cache['t'] = now
      try:
        _param_cache['on'] = read_plugin_param('bmw_e9x_e8x', 'AngleBudget', '') == '1'
      except Exception:
        _param_cache['on'] = False
    return _param_cache['on']
```

Add to the `state` dict initialiser:

```python
    'push_ref': None,        # steering angle when this push began (deg)
    'push_moved': 0.0,       # debug: signed deg moved since then
    'budget_spent': False,   # debug: 2 deg moved in the commanded direction
```

In `update()`, immediately after `pid_log.version = 11` (runs every CAN tick):

```python
    # Push budget. Deltas only — steeringAngleDeg carries a constant ~-1.58 deg
    # physical alignment offset which cancels against the captured reference.
    # getattr guard: CS is a stub in some test paths.
    _angle = float(getattr(CS, 'steeringAngleDeg', 0.0))
    if state['action'] == 'ramp':
      if state['push_ref'] is None:
        state['push_ref'] = _angle
      state['push_moved'] = _angle - state['push_ref']
    else:
      state['push_ref'] = None
      state['push_moved'] = 0.0
    # Torque is NEGATIVE for left, angle POSITIVE for left, so the product of
    # push_moved and -torque is positive when the wheel moved the way we asked.
    state['budget_spent'] = (_budget_enabled()
                             and abs(state['push_moved']) >= BUDGET_DEG
                             and state['push_moved'] * -state['torque'] > 0.0)
```

In the decision block, replace the `STEP_MAX` clamp:

```python
          step = float(np.clip(target_frac - state['torque'], -step_max, step_max))
          target_frac = float(np.clip(state['torque'] + step, -t_cap_frac, t_cap_frac))
```

with:

```python
          # Once the wheel has moved its 2 deg, stop pushing harder and shed
          # torque at whatever rate the P law asks. Winding up is fought by the
          # rack; unwinding is free (self-aligning torque does it). A symmetric
          # STEP_MAX is right while ramping blind, but afterwards it needed
          # 0.65 s to unwind route 3f2 seg 10 while the overshoot took 0.4 s.
          if state['budget_spent']:
            if abs(target_frac) > abs(state['torque']):
              target_frac = state['torque']          # no more pushing
            step = target_frac - state['torque']     # ease off unthrottled
          else:
            step = float(np.clip(target_frac - state['torque'], -step_max, step_max))
          target_frac = float(np.clip(state['torque'] + step, -t_cap_frac, t_cap_frac))
```

Telemetry — add to the `payload` dict:

```python
          'push_moved': float(state['push_moved']),
          'budget_spent': bool(state['budget_spent']),
```

`plugin.json` `params`:

```json
    "AngleBudget": {
      "default": false,
      "description": "Stop pushing and ease off once the wheel has moved 2 degrees"
    }
```

- [ ] **Step 5: Run the full suite**

Run: `cd /home/oxygen/catpilot-dev/plugins && PYTHONPATH=. uv run pytest plugins/bmw_e9x_e8x/tests/ -v`
Expected: PASS. **With the toggle off every pre-existing test must be unchanged — that is the regression gate.**

- [ ] **Step 6: On-device probe harness**

```bash
scp plugins/bmw_e9x_e8x/tests/on_device_probe.py c3:/tmp/on_device_probe.py
ssh c3 'source /usr/local/venv/bin/activate && cd /data/openpilot && python /tmp/on_device_probe.py'
```
Expected: all probes PASS (run offroad).

- [ ] **Step 7: Document and commit**

Add a dated section to `plugins/bmw_e9x_e8x/LATERAL_CONTROLLER.md`: the 2° budget and why it is a budget rather than a detection threshold, the asymmetric `STEP_MAX`, the `AngleBudget` param, and the route 3f2 seg 10 numbers.

```bash
cd /home/oxygen/catpilot-dev/plugins
git rm plugins/bmw_e9x_e8x/bmw/rack_motion.py plugins/bmw_e9x_e8x/tests/test_rack_motion.py plugins/bmw_e9x_e8x/tests/replay_rack_motion.py
git add -A plugins/bmw_e9x_e8x/
git commit -m "feat(bmw): give each push a 2-degree steering budget, then ease off"
```

---

### Task 2: Route-wide sanity check

**Files:** Create `plugins/bmw_e9x_e8x/tests/replay_push_budget.py`

Not a gate — a sanity check that the budget neither fires constantly nor never fires. Read `carState.steeringAngleDeg` and the `bmw_lat_control` bus topic (`output`, `action`) from each rlog, reproduce the Task 1 rule offline, and report across route `000003f2--a4bbab4676--` segments 0–50:

- number of pushes, and what fraction spend the budget
- median and p90 time from push start to spending it
- median and p90 torque at the moment of spending (Nm)
- how much more torque the ramp added *after* the budget was spent (this is the headroom the change recovers)

Run it on the device read-only, staging the module under `/tmp` (the runtime deploy predates it):

```bash
ssh c3 'source /usr/local/venv/bin/activate && cd /data/openpilot && \
  PYTHONPATH=/data/openpilot python /tmp/replay_push_budget.py \
  /data/media/0/realdata/000003f2--a4bbab4676-- 0 50'
```

**Expected from hand-analysis:** segment 10 spends its budget at t≈663.87 with torque ≈2.8 Nm. If the route-wide median torque at spend is near zero, or nearly every push spends instantly, stop and report — that would mean the rule is cutting normal steering short. **Do not tune `BUDGET_DEG` to change the numbers.**

Append the output to this plan file as `## Task 2 result` and commit.

---

### Task 3: On-car A/B — REQUIRES THE CAR, stop here for the user

Deploy with the toggle OFF, drive a route with straights and curves, then enable and drive the same route:

```bash
ssh c3 'echo -n 1 > /data/plugins-runtime/bmw_e9x_e8x/data/AngleBudget'
ssh c3 "pkill -f 'selfdrive.ui.ui'"     # verify a NEW pid/etime; the old python.*ui_main pattern matches nothing on 0.11.x
```

Compare: wheel-rate std on straights, residual p2p in curves, torque sign reversals per minute, peak `output`, max curvature overshoot, `budget_spent` occupancy, lane-offset distribution.

**The straight-line metrics are the veto** — route 395's lesson is that offline churn metrics are blind to small-correction phase lag, and the driver's seat is the authoritative sensor.

Rollback is the toggle: `echo -n 0 > .../AngleBudget`. If the excursion is still larger than wanted, the knob is `BUDGET_DEG` 2.0 → 1.0, which fires ~0.3 s and ~0.8 Nm earlier.

---

## Deferred

- **Replacing `FRICTION` at its three consult sites** and **re-deriving `HOLD_BAND`** (which is FRICTION-derived and therefore 3–4× tighter than the measured 2.0–2.75 Nm breakaway justifies — a strong candidate explanation for the route-3ac small-correction churn). Both affect straight-line feel and need their own A/B.
- **`T_CAP_BASE_NM = 2.0`** sits inside the measured breakaway range.
- **The inner-rate-loop / angle-space cascade redesign.**
