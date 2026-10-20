# BMW Lateral-Stack Audit (Part B) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Delete the dead and stale lateral code (spec Part B, decisions D1, D3, D4, D6, D7, D8, D9) with the controller's output bit-identical to `dev` @ 9af3c87 in Direct mode.

**Architecture:** Direct Tracking becomes the only law in `bmw/latcontroller.py`; `lane_keeping` becomes a pure reference conditioner (Arc + low-pass); telemetry-only and persistence code goes; comments and docs are rewritten to the present tense. Every step is proven by the full suite plus a differential replay of the controller against a snapshot of 9af3c87.

**Tech Stack:** Python, pytest (`PYTHONPATH= PYTHONPATH=. uv run pytest plugins/ -q` from the repo root; the pre-push hook runs the same suite).

**Spec:** `docs/superpowers/specs/2026-10-20-bmw-curve-feedforward-and-audit-design.md` (Part B). D2 (hold machinery) is NOT in this plan: it is one behaviour change together with the Part A feed-forward and lands in that plan.

## Global Constraints

- Branch `bmw-lateral-audit` off `dev` @ 7fc8e16 (the design commit); one commit per task, messages prefixed `bmw:`, `lane_keeping:`, `ui_mod:` or `docs:`; no `Co-Authored-By`.
- Controller output bit-identical to 9af3c87 with Direct latched: the Task 0 harness passes after every task (Tasks 1–8).
- Suite green after every task.
- Plugin ids, directory names, module names and plugin-bus topic names (`bmw_lat_control`, `lane_keeping`) do not change.
- No `dependencies` entry in `bmw_e9x_e8x/plugin.json` (the registry would refuse to load the whole car plugin when `lane_keeping` is `.disabled`); the lane_keeping requirement is enforced inside `on_lat_controller_init` instead (Task 1).
- Kept mechanisms, unchanged in value and behaviour: `V_GAIN_FLOOR`, `LKA_MAX_V`/`LKA_BLEND_V`, `UNWIND_STEP_GAIN`, `STEP_MAX_*`, `T_CAP_*`, `CONTROL_HORIZON_S`, `SB_*`, hold_f/hold_cap/held_target/hold-floor/R8/relax-dwell (until Part A), the noise guard, take-up, contact learner, Slope Compensation (`SL_*`, switch).
- Slope row copy after Task 1: "Learn the small steady torque the car needs to drive straight (road slope, alignment) and apply it. Applies at the next engagement."

## Review Focus

1. `lane_keeping` is `.disabled` or failed to import on the device: the BMW car plugin still loads, `on_lat_controller_init` returns without patching `lac.update` (stock torque controller runs) and logs a warning. Test: `test_no_lane_keeping_means_stock_controller` (Task 1).
2. Slope Compensation on, with no Direct switch any more: it latches at every engagement edge. Test: `test_slope_latches_at_engagement` (Task 1).
3. Stall-breakaway episode end in the always-Direct law still uses the 0.001 rad settle. Test: `test_sb_episode_ends_when_settled` (Task 1).
4. First drive after the persistence deletion: no seed, so take-up stays inert until the learner has `LEARN_MIN_SAMPLES` per side, then fires. Test: `test_takeup_waits_for_learned_contacts` (Task 4).
5. `LaneKeepArcHorizon` > 0 still forces a fixed look-ahead with the position loop gone. Test: existing `test_fixed_horizon_override` must survive Task 6 unchanged.

---

### Task 0: Branch and the bit-identity harness

**Files:**
- Create (scratchpad, not committed): `<scratchpad>/rv5/latcontroller_base.py`, `<scratchpad>/rv5/test_bitident_audit.py`

**Interfaces:**
- Produces: the command every later task runs: `cd plugins/bmw_e9x_e8x && PYTHONPATH=. uv run pytest <scratchpad>/rv5/test_bitident_audit.py -q`.

- [ ] **Step 1:** `git checkout -b bmw-lateral-audit` from `dev` @ 7fc8e16.
- [ ] **Step 2:** `git show 9af3c87:plugins/bmw_e9x_e8x/bmw/latcontroller.py > <scratchpad>/rv5/latcontroller_base.py`.
- [ ] **Step 3:** Write `test_bitident_audit.py` from `<scratchpad>/rv3/test_bitident_slope.py`:
  - both controllers get `sys.modules['plugins.lane_keeping.register'] = SimpleNamespace(HOOK_READY=True)` before init;
  - base params `{'DirectTracking': '1', 'DirectContacts': '', 'HoldHysteresis': '', 'StallBreakaway': sb, 'SlopeCompensation': sl}`; head params `{'StallBreakaway': sb, 'SlopeCompensation': sl}`;
  - parametrize `seed` 0–5, `sb` in ('1','0'), `sl` in ('1','0');
  - two scenario generators: the existing random walk, and `curvy`: κ_des = 0.012·sin(2π·t/40 s) + noise 2e-4, v = 8…30 m/s random walk, the same `active`/`pressed` toggles — so the hold_f > 0 and relax-dwell branches run;
  - assert `oa[0] == ob[0]` every tick.
- [ ] **Step 4:** Run it against the unchanged tree. Expected: 48 passed.

### Task 1: D1 — Direct is the only law (bmw side)

**Files:**
- Modify: `plugins/bmw_e9x_e8x/bmw/latcontroller.py`, `plugins/bmw_e9x_e8x/register.py`, `plugins/bmw_e9x_e8x/plugin.json`
- Test: `plugins/bmw_e9x_e8x/tests/test_latcontroller.py`, `plugins/bmw_e9x_e8x/tests/test_hooks.py`

**Interfaces:**
- Produces: `_make_controller(monkeypatch, wheelbase=2.66, stall_v2=None, contacts=None, slope=None)` (the `hold_hyst` and `direct` kwargs are gone); a module-level autouse fixture in `test_latcontroller.py` that installs the `HOOK_READY=True` stub for `plugins.lane_keeping.register`; module constant `SB_SETTLED_RAD = 0.001` beside the `SB_*` constants.
- Removed from the module API: `direct_latched()`, `lateral_engaged()`, `_DIRECT_LATCHED`, `_LAT_ENGAGED`.

- [ ] **Step 1: Write the failing tests** (in `TestDirectTracking`, replacing the latch/switch tests):
  - `test_no_lane_keeping_means_stock_controller`: delete the stub from `sys.modules`, call `on_lat_controller_init(result, lac, CP)`; assert `lac.update` is unchanged (set `lac.update = sentinel` before) and the return value is `result`.
  - `test_half_loaded_lane_keeping_means_stock_controller`: stub without `HOOK_READY` → same assertions.
  - `test_slope_latches_at_engagement`: `slope=True`, engage → `state['slope']` is True; disengage → False; set the param stub to '0' and re-engage → False.
  - `test_sb_episode_ends_when_settled` (in `TestStallBreakawayV2`): drive the machine to `sb_state == 2`, then feed a livePose tick with `|delta_err| <= 0.001` → `sb_state == 0`.
  - `test_sub_band_error_is_tracked`: existing `test_sub_band_error_is_tracked_in_direct`, with the `at_rest` assertion dropped.
- [ ] **Step 2: Run** `PYTHONPATH= PYTHONPATH=. uv run pytest plugins/bmw_e9x_e8x/tests/test_latcontroller.py -q -k "stock_controller or slope_latches or ends_when_settled"`. Expected: FAIL (`direct` kwarg / `at_rest` / patch still applied).
- [ ] **Step 3: Implement** in `latcontroller.py`:
  - module scope: delete `_DIRECT_LATCHED`, `direct_latched`, `_LAT_ENGAGED`, `lateral_engaged`; keep `_LK_REGISTER_MODULE`, `_lane_keeping_ready`; add `SB_SETTLED_RAD = 0.001`.
  - `on_lat_controller_init`: first statement after the imports: `if not _lane_keeping_ready(): <warn>; return result` where `<warn>` is `from openpilot.common.swaglog import cloudlog; cloudlog.warning("bmw latcontroller: lane_keeping not loaded, stock lateral controller in use")` inside its own `try/except Exception: pass`.
  - delete the `HoldHysteresis` read and `_hold_hyst_on`; delete `HOLD_BAND`, `HOLD_BAND_ENTER`, `HOLD_EMA_TAU`, `HOLD_EMA_ESCAPE`.
  - state: delete `at_rest`, `derr_ema`, `direct`.
  - engagement edge: read only `SlopeCompensation` (try/except → False), `guard.reset()`; disengage edge: `slope = False`, take-up reset, action restore (the contacts write stays until Task 4). Delete the `global` statement and the mirror assignments.
  - sb machine episode end: `state['sb_episode_ticks'] <= 0 or abs(state['delta_err']) <= SB_SETTLED_RAD`.
  - livePose block: delete the `derr_ema` update; delete the whole `cancel_tol` branch so the `else` body runs unconditionally (`tick_count += 1`, label expiry over `('ramp', 'relax_dwell')`); delete `UNWIND_CADENCE_TQ`.
  - decision: delete `_enter`, the `at_rest` ladder and the `if state['at_rest']` branch; the P path runs unconditionally; `_trim_hold = hold_f > 0.0 and target_frac * held_target < 0.0`; `if not dwelling: target_frac = state['guard'].allow(target_frac)`.
  - take-up block and the ramp-end zero snap: unconditional (drop `state['direct']`).
  - telemetry: delete `hold_band`, `hb_enter`, `derr_ema`, `at_rest`, `direct`.
  - `register.py`: delete `_set_direct_tracking` and the Direct Tracking row; Slope row copy per Global Constraints. `plugin.json`: delete the `DirectTracking` and `HoldHysteresis` params.
- [ ] **Step 4: Prune the tests.** Rule: delete every test whose subject is a deleted mechanism (cancel_tol, at_rest / hold_zero / hold_curve, entry hysteresis, EMA escape, `HoldHysteresis`, the `DirectTracking` switch and latch, `direct_latched`, `lateral_engaged`); a test of a kept mechanism that only used a rest state as a quiet vehicle is rewritten on the P path with the same assertion target. Whole classes to delete: `TestCancelTolStillFires`, `TestPersistentLeanEscape`, `TestCancelTolCadenceOnReversal`; `TestFrictionGatesDeleted` and `TestNonCancelPaths` keep only their ramp / relax-dwell tests. `test_hooks.py`: `test_exact_row_set` → `['Temperature Overlay', 'Stall Breakaway', 'Slope Compensation', 'Resume Button Repurposed']`; delete the two `direct_tracking` row tests; `test_slope_compensation_row_follows_direct_tracking` → `..._follows_stall_breakaway`. Then: `grep -nE "hold_hyst|HoldHysteresis|HOLD_BAND|at_rest|derr_ema|cancel_tol|hold_zero|hold_curve|direct_latched|lateral_engaged|direct=" plugins/bmw_e9x_e8x/tests/*.py` prints nothing.
- [ ] **Step 5: Run** the suite and the harness. Expected: suite green; harness 48 passed.
- [ ] **Step 6: Commit** `bmw: Direct Tracking is the only lateral law`.

### Task 2: D1 — lane_keeping always serves the Direct reference; its switches go

**Files:**
- Modify: `plugins/lane_keeping/register.py`, `plugins/lane_keeping/plugin.json`, `plugins/ui_mod/driving_panel.py`, `plugins/ui_mod/DESIGN.md`
- Test: `plugins/lane_keeping/tests/test_register.py`

- [ ] **Step 1: Write the failing test** `test_reference_is_always_arc_and_never_position_corrected`: with `data/LaneKeepEnable` = '1' and `data/LaneKeepArcSmooth` = '0', after 150 hook calls `register._anchor.cfg.enable is False` and `cfg.arc_smooth is True`, and `register` has no attribute `_direct_tracking_on`.
- [ ] **Step 2: Run** `PYTHONPATH= PYTHONPATH=. uv run pytest plugins/lane_keeping/tests/test_register.py -q -k always_arc`. Expected: FAIL.
- [ ] **Step 3: Implement.** `register.py`: delete `_BMW_LAT_MODULE`, `_read_direct_switch`, `_bmw_lat_flag`, `_direct_latched`, `_lateral_engaged`, `_direct_tracking_on`, `_direct_switch`, `_user_enable`, `_user_arc`, `_tick`, `LANE_KEEP_DEFAULT`, `ARC_SMOOTH_DEFAULT`, the 100-tick refresh; `_load_config` passes `enable=False, arc_smooth=True`; the hook body is: build the anchor on first call, `update`, publish (publish failure swallowed), return. `plugin.json`: delete `LaneKeepEnable`, `LaneKeepArcSmooth`. `ui_mod/driving_panel.py`: delete the `_plugin_enabled('lane_keeping')` block and `_on_lane_keeping`, `_on_arc_smooth`; `ui_mod/DESIGN.md`: delete row 2 of the Driving-panel list (renumber) and the `LaneKeepEnable` table row.
- [ ] **Step 4: Prune the tests**: delete the R11 block (`_direct_env`, `_call`, the 10 tests under "BMW Direct Tracking"), `test_hook_passthrough_when_disabled`, `test_live_toggle_disables_and_releases`, `test_live_arc_toggle`, `test_live_defaults_without_param_files`; in `test_load_config_defaults`/`_overrides`/`_arc_params` drop the `enable`/`arc_smooth` assertions.
- [ ] **Step 5: Run** the suite and the harness. Expected: green; 48 passed.
- [ ] **Step 6: Commit** `lane_keeping: always the Arc reference, position-loop and Arc switches removed`.

### Task 3: D4 — delete the driver-override observers

**Files:**
- Modify: `plugins/bmw_e9x_e8x/bmw/latcontroller.py`
- Test: `plugins/bmw_e9x_e8x/tests/test_latcontroller.py`

- [ ] **Step 1:** Delete `TestOverrideObservers` whole, and add to `TestStallBreakawayV2` the test `test_no_ov_keys_in_state_or_payload`: after one engaged tick, no state key and no telemetry key starts with `ov_`.
- [ ] **Step 2: Run** `-k no_ov_keys`. Expected: FAIL.
- [ ] **Step 3: Implement**: delete `OV_*` constants (incl. `OV_PUSH_MEM_S`, `OV_RATE_TICKS`), `_ov_bump`, the block between the `# --- override observers (phase 1) ---` and `# --- end override observers ---` markers (markers included), the `ov_*` state keys and telemetry fields. `sb_ring` and the sb machine stay.
- [ ] **Step 4: Run** the suite and the harness. Expected: green; 48 passed.
- [ ] **Step 5: Commit** `bmw: driver-override observers removed`.

### Task 4: D6 — delete contacts persistence

**Files:**
- Modify: `plugins/bmw_e9x_e8x/bmw/latcontroller.py`, `plugins/bmw_e9x_e8x/bmw/takeup.py`
- Test: `plugins/bmw_e9x_e8x/tests/test_takeup.py`, `plugins/bmw_e9x_e8x/tests/test_latcontroller.py`

**Interfaces:**
- Produces: `ContactLearner(dt=0.01)` (no `seed`, no `fresh()`); test helper `_seed_learner(state, c_right, c_left)` in `test_latcontroller.py` that sets `state['learner']._val = {0: c_right, 1: c_left}`; `_make_controller(monkeypatch, wheelbase=2.66, stall_v2=None, slope=None)`.

- [ ] **Step 1: Write the failing tests**: `test_takeup_waits_for_learned_contacts` (controller level): with no seed, a clean reversal at 25 m/s produces no `take_up` action; after `_seed_learner(state, 77.6, 65.0)` the same reversal does. In `test_takeup.py` delete `test_seed_used_until_fresh_and_then_replaced` and `test_parse_format_roundtrip_and_rejects`; `test_learns_both_sides_by_median` asserts `contacts()` instead of `fresh()`.
- [ ] **Step 2: Run** `-k waits_for_learned`. Expected: FAIL (the `contacts` kwarg still seeds).
- [ ] **Step 3: Implement**: `takeup.py` — delete `parse_contacts`, `format_contacts`, the `seed` argument, `_seed`, `fresh()`; `contacts()` returns `None` until both sides are learned. `latcontroller.py` — delete the `DirectContacts` read, `_contacts_seed`, the disengage-edge write (and its `write_plugin_param` import), `ContactLearner()` without a seed. `test_latcontroller.py` — delete the `write_plugin_param` MagicMock line in `mock_deps`, the `contacts` kwarg, and seed every test that needs contacts through `_seed_learner`.
- [ ] **Step 4: Run** the suite and the harness. Expected: green; 48 passed.
- [ ] **Step 5: Commit** `bmw: DirectContacts persistence removed; contacts are learned each drive`.

### Task 5: D7 — telemetry trim

**Files:**
- Modify: `plugins/bmw_e9x_e8x/bmw/latcontroller.py`, `plugins/bmw_e9x_e8x/LATERAL_CONTROLLER.md` (§9 table only)
- Test: `plugins/bmw_e9x_e8x/tests/test_latcontroller.py`

- [ ] **Step 1: Write the failing test** `test_payload_exact_key_set`: one engaged livePose tick publishes exactly `{'desired','measured','err','delta_err','target_frac','ramp_step','ramp_frames','action','torque','output','vEgo','active','hold_f','hold_cap','relax_ticks','sb_state','sb_trips','sb_block','angle','servo_gap','steer_rate','pressed','lat_delay','tu_state','c_right','c_left','tu_fires','tu_ms','tu_timeouts','tu_guarded','sl_bias','sl_learn'}`. (`hold_f`, `hold_cap`, `relax_ticks` stay until Part A removes their mechanisms.)
- [ ] **Step 2: Run** `-k payload_exact`. Expected: FAIL.
- [ ] **Step 3: Implement**: delete `desired_raw`, `jerk_pred`, `a_y_meas`, `sb_on` from the payload and their state keys; delete `model_action_t`, `DT_MDL`, `MODEL_ACTION_T_FALLBACK` (nothing else reads them); `lat_delay` is still published raw. Rewrite the §9 table to the exact set above. Delete tests that asserted the removed keys.
- [ ] **Step 4: Run** the suite and the harness. Expected: green; 48 passed.
- [ ] **Step 5: Commit** `bmw: lateral telemetry trimmed to the fields the current law produces`.

### Task 6: D3 — lane_keeping becomes the reference conditioner

**Files:**
- Modify: `plugins/lane_keeping/anchor.py`, `plugins/lane_keeping/register.py`, `plugins/lane_keeping/plugin.json`, `plugins/lane_keeping/DESIGN.md`, `plugins/lane_keeping/README.md`, `plugins/ui_mod/exp_button.py`, `README.md` (repo root)
- Test: `plugins/lane_keeping/tests/test_anchor.py`, `plugins/lane_keeping/tests/test_register.py`

**Interfaces:**
- Produces: `AnchorConfig(kappa_filter_tau: float = 0.15, arc_horizon: float = 0.0)`; `LaneAnchor(config).update(curvature, model_v2, v_ego, lane_changing) -> (kappa_ref, telem)` with telem keys exactly `kappa_in, kappa_ref, kappa_arc, arc_on, arc_h, k_ahead, v_ego, line_l, line_r, prob_l, prob_r`; module keeps `DT_CTRL, ARC_X_MIN, ARC_H_SPEED_V, ARC_H_SPEED, ARC_H_CURVE, ARC_K_AHEAD_BP, ARC_K_AHEAD_T, _clip, _interp, _interp_arr, plan_curvature_ahead, arc_horizon, arc_curvature`. The hook `on_curvature_correction(curvature, model_v2, v_ego, lane_changing, lat_delay=None)` keeps its signature and ignores `lat_delay`.

- [ ] **Step 1: Write the failing tests**: `test_telemetry_exact_key_set` (the set above); `test_update_has_no_lat_delay_parameter` (`inspect.signature(LaneAnchor.update)` has no `lat_delay`); `test_config_has_only_the_two_fields` (`dataclasses.fields(AnchorConfig)` names == `['kappa_filter_tau', 'arc_horizon']`).
- [ ] **Step 2: Run** `-k "exact_key_set or no_lat_delay or two_fields"`. Expected: FAIL.
- [ ] **Step 3: Implement** `anchor.py`: delete the position loop (`_gap`, `_excess`, `_pursuit`, the `available` branch, DC tracker, predictive gap, hard floors, asym gate, `kappa_bias`, `state`, `gap_*`), the deleted config fields, `LAT_DELAY_FALLBACK`, `X_PRED_MIN/MAX`, `AC_DEADBAND_V`, the sign-convention block that only served the loop (`line_sign`, `curv_sign`, `driver_idx`). `update` keeps: arc (always), low-pass with lane-change bypass, lane geometry telemetry, `kappa_ref` return. `register.py`: `_load_config` reads `LaneKeepKappaFilterTau` and `LaneKeepArcHorizon` only. `plugin.json`: description "Lateral reference conditioner: an arc through the model's planned path ahead, low-passed, for the vehicle's lateral controller."; `"params": {}`. `exp_button.py`: delete the ring (docstring line, `_lane_keeping_active`, `_lcc_sub`, both polling blocks, the `draw_ring` call, `RING_WIDTH`/`RING_LANE_KEEPING` if unused).
- [ ] **Step 4: Prune the tests.** `test_anchor.py` keeps: the three smoothing tests, the arc tests except `test_arc_off_by_default_reference_is_the_model_curvature` (arc is always on), the five horizon tests, `test_fixed_horizon_override`, the two lane-geometry tests; everything on gap/excess/pursuit/pred/dc/asym/floors/driver side/`test_telemetry_logs_both_switch_positions` goes. `test_register.py` keeps `test_hook_ready_is_set_after_a_full_load`, `test_passthrough_returns_input_curvature`, `test_load_config_kappa_filter_tau`, `test_hook_passes_lat_delay_through` (rewritten: the hook accepts `lat_delay` and returns a float), the publish-failure test, and Task 2's test; the remaining `_load_config` tests go.
- [ ] **Step 5: Docs.** `lane_keeping/README.md` and `DESIGN.md` rewritten (≤ 50 lines each): what it does now (arc schedule, pull-back, low-pass, lane-change bypass), that the BMW lateral controller requires it (disabled → stock lateral), telemetry, and a "History" pointer to the specs in `docs/superpowers/specs/`. Repo `README.md`: row 17 → "Smooths the model's curvature reference (an arc through its planned path ahead) for the vehicle's lateral controller"; delete the "Lane Keeping" toggle sentence at lines 41–44.
- [ ] **Step 6: Run** the suite and the harness. Expected: green; 48 passed.
- [ ] **Step 7: Commit** `lane_keeping: reference conditioner only; position loop, switches and emblem ring removed`.

### Task 7: D9 — delete the analysis scripts

**Files:**
- Delete: in `plugins/bmw_e9x_e8x/tests/`: `analyze_lateral.py check_torque.py check_yawrate_sign.py compare_yawrate_sources.py diagnose_00000296.py estimate_friction.py eval_buckets.py eval_can_la.py eval_can_vs_pose.py eval_delayed.py eval_micro_stepping.py eval_piecewise.py eval_svd.py eval_yawrate.py fit_plant_gain.py on_device_probe.py replay_lanechange.py replay_lateral.py sim_adaptive_gains.py sim_micro_stepping.py sim_pid_optimize.py sim_step_response.py sweep_friction_deadzone.py sweep_offset_criterion.py validate_adaptive_gains.py`

- [ ] **Step 1:** `git rm` the 25 files (none is imported by a test or named in any doc — verified 2026-10-20).
- [ ] **Step 2: Run** the suite. Expected: green, same count as after Task 6.
- [ ] **Step 3: Commit** `bmw: analysis scripts for reverted designs removed`.

### Task 8: D8 — comments and docs in the present tense

**Files:**
- Modify: `plugins/bmw_e9x_e8x/bmw/latcontroller.py`, `plugins/bmw_e9x_e8x/bmw/takeup.py`, `plugins/bmw_e9x_e8x/LATERAL_CONTROLLER.md`

Rules for the code: one rationale per rule, present tense; a constant's comment is at most three lines — what it is, where the value comes from (one route or number), what breaks if it moves; no history narration (no "used to", "removed 20xx", "retired", "historical", "an earlier version", "no longer", "Phase 1/2", "review fix"); the module docstring states the law in ≤ 25 lines; the `on_lat_controller_init` docstring describes the current decision path only; the three SAFETY ARCHITECTURE notes collapse to one ≤ 8-line note at module scope. Deleted constants' comment blocks go with them.

- [ ] **Step 1:** Rewrite the comments per the rules. Verify: `grep -cE "used to|removed 20|retired|historical|HISTORICAL|an earlier version|no longer|Phase [12]|review fix" plugins/bmw_e9x_e8x/bmw/latcontroller.py` → 0; `wc -l` ≤ 800.
- [ ] **Step 2:** `LATERAL_CONTROLLER.md`: insert a "Current law (2026-10-20)" section at the top (≤ 60 lines: signal path lane_keeping → controller, the decision math with `kappa_scale`, `t_cap`, guard, take-up, STEP_MAX, hold-floor/relax-dwell as still present, `SL_*`; a constants table with values; the two switches `StallBreakaway` and `SlopeCompensation`; a pointer to §9 telemetry); a "History" heading above the first dated note; §10: replace the "Don't add a FF term on κ_des" line with "Below ~0.3 m/s² the rack holds its angle at zero torque; above it curves need steady torque (4e9–4ec, 440 windows) — see the 2026-10-20 design"; §12: delete the `HoldHysteresis` and `DirectTracking` recipes; §13: rewrite the code map to the current file.
- [ ] **Step 3: Run** the suite and the harness. Expected: green; 48 passed.
- [ ] **Step 4: Commit** `bmw: lateral controller comments and reference doc rewritten to the current law`.

### Task 9: catpilot README row

**Files:**
- Modify: `/home/oxygen/catpilot-dev/catpilot/README.md:32`

- [ ] **Step 1:** Row text → "Smooths the model's curvature reference (an arc through its planned path ahead) for the vehicle's lateral controller".
- [ ] **Step 2: Commit** in the catpilot repo on `dev`: `docs: lane_keeping row describes the reference conditioner`. No device deploy needed.
