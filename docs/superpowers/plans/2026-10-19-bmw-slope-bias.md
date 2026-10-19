# BMW Slope Bias Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove Direct Tracking's standing straight-road error (the right-line hug) with one slow, bounded, learned torque bias.

**Architecture:** In `bmw/latcontroller.py`, a state value `sl_bias` (Nm) integrates the P term on straights at 20 Hz. It is added to Direct's P target at each decision. A Driving-panel switch `SlopeCompensation`, latched at engagement, enables it only while Direct is latched.

**Tech Stack:** Python, pytest (`PYTHONPATH=. uv run pytest plugins/ -q` from the repo root).

**Spec:** `docs/superpowers/specs/2026-10-19-bmw-slope-bias-design.md`

## Global Constraints

- Constants, at `latcontroller.py` module scope beside the Direct constants:
  - `SL_TAU_S = 20.0`
  - `SL_BIAS_MAX_NM = 0.15`
  - `SL_MIN_V = 40 / 3.6` (m/s)
  - `SL_KAPPA_DES = 1e-3`
  - `SL_KAPPA_MEAS = 1.5e-3`
- Learning law, applied each livePose tick: `sl_bias = clip(sl_bias + (P_nm / SL_TAU_S) * DT_LIVEPOSE, ±SL_BIAS_MAX_NM)`.
  - `P_nm = T_CAP_SLOPE_BASE * kappa_scale * v * v * delta_err` (P alone, without the bias).
- Learn gate. It depends on the driving situation only, never on the error's sign. All of these must hold:
  - `state['slope']`
  - `active`
  - `not CS.steeringPressed`
  - `v_true > SL_MIN_V`
  - `|state['desired']| < SL_KAPPA_DES`
  - `|state['measured']| < SL_KAPPA_MEAS`
  - `state['takeup'].state != 'taking_up'`
  - `state['sb_block'] == 0`
  - `state['sb_state'] == 0`
- Apply: when `state['slope']`, `target_nm = P_nm + state['sl_bias']` on the Direct P path (latcontroller.py ~line 1365). It is applied whether or not the gate is open.
- Lifetime:
  - `sl_bias` starts at 0.0 in the state dict at `on_lat_controller_init`;
  - it is never reset on disengage and never persisted.
- Latch: `state['slope'] = state['direct'] and read_plugin_param('bmw_e9x_e8x', 'SlopeCompensation', '') == '1'`. Set inside the existing engagement-edge try block (~line 696), and cleared with `direct` at disengage.
- Switch off (or Direct off): controller output bit-identical to dev @ 0a26023.
- Telemetry, added to the bmw_lat_control dict beside the `tu_*` fields:
  - `'sl_bias': float(state['sl_bias'])`
  - `'sl_learn': bool(<gate open this tick>)`
- Driving-panel row:
  - title "Slope Compensation";
  - description "Learn the small steady torque the car needs to drive straight (road slope, alignment) and apply it. Works only with Direct Tracking. Applies at the next engagement.";
  - default off (`== '1'`), placed right after the Direct Tracking row.

## Review Focus

1. Long curve or a slow road: the gate stays closed and `sl_bias` holds its value but is still applied. Test: `test_curve_holds_bias_without_learning`.
2. Driver press mid-learn: frozen in both directions. Test: `test_press_freezes_bias`.
3. Disengage, then re-engage: `sl_bias` is kept. Test: `test_bias_survives_reengage`.
4. Switch on with Direct off: `slope` is not latched, so there is no learning and nothing is applied. Test: `test_slope_needs_direct`.
5. Take-up in flight or a stall-breakaway block/episode: no learning. Test: `test_takeup_and_sb_close_gate`.

---

### Task 1: Learned slope bias in the controller

**Files:**
- Modify: `plugins/bmw_e9x_e8x/bmw/latcontroller.py`
- Test: `plugins/bmw_e9x_e8x/tests/test_latcontroller.py` (new class `TestSlopeBias`)

**Interfaces:**
- Produces:
  - state keys `slope` (bool) and `sl_bias` (float Nm);
  - telemetry `sl_bias`, `sl_learn`;
  - param key `SlopeCompensation`.
- Test helper: extend `_make_controller(..., slope=None)` so that `SlopeCompensation` returns '1'/'0', like `direct`.

- [ ] **Step 1: Write the failing tests** in `TestSlopeBias`. Use the `_lane_keeping_loaded` fixture pattern from `TestDirectTracking` (the module stub needs `HOOK_READY=True`). Engage at v=25 with `direct=True, slope=True`. Feed `fake_sm.updated = {'livePose': True}` every call. Assertions:
  - `test_learns_toward_standing_p_both_signs`: desired 0, measured ±2e-4.
    - After 10 ticks, `sl_bias ≈ Σ P_nm·0.05/20` (sign opposite to measured: measured +2e-4 → bias < 0).
    - After 2000 ticks, `sl_bias == -SL_BIAS_MAX_NM` (and `+` for measured −2e-4).
  - `test_bias_added_to_p_target`: set `state['sl_bias'] = -0.1` with zero error. The next decision's `target_frac*12 ≈ -0.1` (within the step cap).
  - `test_curve_holds_bias_without_learning`: with desired 3e-3, `sl_bias` is unchanged over 200 ticks and `sl_learn` is False.
  - `test_low_speed_no_learning`: at v=10 m/s, `sl_bias` is unchanged.
  - `test_press_freezes_bias`: `pressed=True` → unchanged, for both error signs.
  - `test_takeup_and_sb_close_gate`: each of `takeup.state='taking_up'`, `sb_block=5`, `sb_state=2` → unchanged.
  - `test_bias_survives_reengage`: learn, disengage one tick, re-engage → `sl_bias` keeps its value.
  - `test_slope_needs_direct`: `direct=False, slope=True` → `state['slope']` is False, `sl_bias` stays 0.0, and output equals a `slope=None` controller fed the same sequence.
  - `test_slope_off_bit_identical`: `direct=True, slope=False` vs `slope=None` over the same 300-tick mixed sequence (curves, straights, press, speed change) → identical `state['torque']` every tick.

- [ ] **Step 2: Run** `PYTHONPATH=. uv run pytest plugins/bmw_e9x_e8x/tests/test_latcontroller.py -q -k SlopeBias`. Expected: FAIL (`slope` kwarg / `sl_bias` key missing).

- [ ] **Step 3: Implement** in `latcontroller.py`:
  - the constants;
  - state keys `'slope': False, 'sl_bias': 0.0, 'sl_learn': False`;
  - the latch at the engagement edge, and `slope=False` at disengage;
  - learning in the livePose block right after `kappa_scale` is computed (it needs `v`, `kappa_scale`, `delta_err`);
  - the apply on the P path;
  - telemetry.

  One comment block cites the spec and the hold-bias lesson ("gate on situation only, never on error sign").

- [ ] **Step 4: Run** the Task-1 tests, then the full suite `PYTHONPATH=. uv run pytest plugins/ -q`. Expected: all pass (1206 + new).

- [ ] **Step 5: Commit** `bmw: Direct Tracking slope bias - slow straight-road torque bias (SlopeCompensation)`.

### Task 2: Driving-panel switch, manifest param, docs

**Files:**
- Modify: `plugins/bmw_e9x_e8x/register.py` (setter `_set_slope_compensation(state)` + row)
- Modify: `plugins/bmw_e9x_e8x/plugin.json` (param `SlopeCompensation`)
- Modify: `plugins/bmw_e9x_e8x/LATERAL_CONTROLLER.md` (subsection after "Direct Tracking mode")
- Test: `plugins/bmw_e9x_e8x/tests/test_hooks.py`

**Interfaces:**
- Consumes: param key `SlopeCompensation` (Task 1).

- [ ] **Step 1: Write the failing tests** mirroring `test_direct_tracking_row_state_and_callback` / `test_direct_tracking_absent_for_non_bmw`:
  - the row titled "Slope Compensation" exists right after "Direct Tracking";
  - its initial state follows the param file ('1' on, '0' or absent off);
  - its callback writes '1'/'0';
  - it is absent for non-BMW.
- [ ] **Step 2: Run** `PYTHONPATH=. uv run pytest plugins/bmw_e9x_e8x/tests/test_hooks.py -q -k slope`. Expected: FAIL.
- [ ] **Step 3: Implement** the setter and row, using the Global Constraints copy. Add the plugin.json entry:
  - `"SlopeCompensation": {"default": false, "description": "Learn and apply the steady straight-road torque (road slope, alignment) in Direct Tracking. Write 1 to enable. Read at each engagement."}`

  Docs subsection, under 25 lines: problem (4e9 +0.15 m hug, P-only standing error), law, gate, constants, telemetry, roll-FF deferral.
- [ ] **Step 4: Run** the full suite. Expected: all pass.
- [ ] **Step 5: Verify bit-identity.** Run the differential replay with the base set to dev @ 0a26023. Copy `git show 0a26023:plugins/bmw_e9x_e8x/bmw/latcontroller.py` over the scratchpad `rv/latcontroller_base.py`, add `SlopeCompensation: '0'` and `''` to the harness params, then run `cd plugins/bmw_e9x_e8x && PYTHONPATH=. uv run pytest <scratchpad>/rv/test_bitident_head.py -q`. Expected: all pass.
- [ ] **Step 6: Commit** `bmw: Slope Compensation Driving-panel switch`.
