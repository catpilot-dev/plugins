# BMW Curve Feed-Forward (Part A + D2) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Supply the steady torque curves need with a bounded feed-forward on the reference lateral acceleration, and delete the curve hold machinery it replaces (spec Part A and decision D2).

**Architecture:** In `bmw/latcontroller.py` every livePose tick computes `demand_nm = ff_nm + P_nm (+ sl_bias)`, where `ff_nm = curve_ff_nm(a_ref)` is a pure function of `a_ref = v_true² · κ_ref`; the decision takes that demand through `t_cap`, the noise guard (which now observes the sign of the whole demand), `STEP_MAX` and the stall-breakaway block. The hold factor, hold cap, hold-floor, R8 trim and relax-dwell go. On straights (`|a_ref| < CF_AY0`) nothing changes: a straight-only differential replay against dev @ 2df1b63 stays bit-identical after every task.

**Tech Stack:** Python, pytest (`PYTHONPATH= PYTHONPATH=. uv run pytest plugins/ -q` from the repo root).

**Spec:** `docs/superpowers/specs/2026-10-20-bmw-curve-feedforward-and-audit-design.md` — Part A (law, constants, verification) and Part B decision D2.

## Global Constraints

- Branch `bmw-curve-ff` off `dev` @ 2df1b63; one commit per task, messages prefixed `bmw:` or `docs:`; no `Co-Authored-By`.
- Tasks 1 and 2 are ONE behaviour change (spec: "FF + D2"). Both land before any deploy; the branch is never deployed between them.
- Constants, verbatim from the spec, at `latcontroller.py` module scope beside `SL_*`: `CF_AY0 = 0.3` (m/s²), `CF_GAIN = 1.0` (Nm per m/s²), `CF_MAX_NM = 2.0` (Nm), `SL_AY_MAX = 0.15` (m/s², the new slope-learning gate).
- Law: `a_ref = v_true * v_true * state['desired']` (the TRUE speed, not the gain reference `v`); `ff_nm = clip(CF_GAIN · sign(a_ref) · max(|a_ref| − CF_AY0, 0), ±CF_MAX_NM)`; `demand_nm = ff_nm + P_nm + (sl_bias if slope else 0)`; `P_nm = T_CAP_SLOPE_BASE · kappa_scale · v² · delta_err` unchanged and still the only term the slope bias integrates.
- The noise guard observes `demand_nm / (T_CAP_SLOPE_BASE · kappa_scale · v · v)` every livePose tick (the demand as an error-equivalent), replacing the `if state['slope']` branch.
- Decision order after Task 2: `target_nm = demand_nm` → `t_cap` clip → `guard.allow` → `_tu_demand` → `STEP_MAX` (× unwind gain) → stall-breakaway same-side block. No hold-floor, no R8 trim, no relax-dwell.
- No new switch (rollback is `CF_GAIN = 0`); no manifest change, so the deploy needs no reboot.
- Telemetry: add `ff_nm`, `a_ref`; drop `hold_f`, `hold_cap`, `relax_ticks`. `test_payload_exact_key_set` pins the set.
- Straight-only harness (Task 0) passes after every task (Tasks 1–3); the full suite is green after every task.

## Review Focus

1. Low-speed tight turn (18 km/h, κ 0.05): `a_ref` uses `v_true`, so `ff_nm` is 0.95 Nm — below the ~2.75 Nm breakaway — not the 0.39 m/s²-floored value. Test: `test_ff_uses_true_speed_not_gain_floor` (Task 2).
2. A κ_ref spike during a lane change (raw model curvature, 0.003 at 30 m/s → `a_ref` 2.7): `ff_nm` saturates at `CF_MAX_NM` and the decision still moves at most `STEP_MAX` from the current torque. Test: `test_ff_capped_and_step_limited_on_kappa_spike` (Task 2).
3. S-curve: the FF flips sign with κ_ref, the demand crosses zero, the guard holds 0 until the new sign has persisted, then the take-up fires once. Test: `test_s_curve_ff_reversal_passes_guard_then_takes_up` (Task 2).
4. Long gentle bend above the gate (κ 4e-4 at 25 m/s, `a_ref` 0.25): the slope bias is frozen while the FF (0) and P carry the bend; `sl_learn` is False although `|κ| < SL_KAPPA_DES`. Test: `test_bias_frozen_above_a_ref_gate` (Task 2).
5. Stall-breakaway block in a curve: the same-side demand including its FF is zeroed for the block; a counter-steer demand passes. Test: `test_sb_block_zeroes_ff_push` (Task 2).

---

### Task 0: Branch and the straight-only replay harness

**Files:**
- Create (scratchpad, not committed): `<scratchpad>/rv6/latcontroller_base.py`, `<scratchpad>/rv6/takeup_base.py`, `<scratchpad>/rv6/test_bitident_straight.py`

**Interfaces:**
- Produces: the command every later task runs: `cd plugins/bmw_e9x_e8x && PYTHONPATH=. uv run pytest <scratchpad>/rv6/test_bitident_straight.py -q`.

- [ ] **Step 1:** `git checkout -b bmw-curve-ff` from `dev` @ 2df1b63.
- [ ] **Step 2:** `git show 2df1b63:plugins/bmw_e9x_e8x/bmw/latcontroller.py > <scratchpad>/rv6/latcontroller_base.py`; same for `bmw/takeup.py` → `takeup_base.py`; in the base, change the import to `from takeup_base import SignGuard, ContactLearner, FreePlayTakeUp`.
- [ ] **Step 3:** Write `test_bitident_straight.py` from the reviewer's `<scratchpad>/review/test_bitident_extended.py` (side-dependent servo gap driven by the base output, stall-then-sweep angle episodes): base and head both get `{'StallBreakaway': sb, 'SlopeCompensation': sl}` and the `HOOK_READY` stub; one scenario `straight`: κ_des random walk σ 3e-5 per tick clipped to ±1.5e-4, v 12–28 m/s (so `|a_ref| ≤ 0.12 < SL_AY_MAX < CF_AY0` and commanded a_y < `HOLD_AY_BP[0]`); parametrize `seed` 0–5, `sb`, `sl`; 12,000 ticks; assert `oa[0] == ob[0]` every tick and, at the end, that take-ups fired and the sb machine armed at least once (else the run proves less than it claims).
- [ ] **Step 4:** Run it against the unchanged tree. Expected: 24 passed, with non-zero take-up and arm counts printed.

### Task 1: D2 — delete the curve hold machinery

**Files:**
- Modify: `plugins/bmw_e9x_e8x/bmw/latcontroller.py`
- Test: `plugins/bmw_e9x_e8x/tests/test_latcontroller.py`, `plugins/bmw_e9x_e8x/tests/test_hooks.py`

**Interfaces:**
- Removed from the module API: `hold_factor`, `HOLD_AY_BP`; state keys `hold_f`, `hold_cap`, `relax_ticks`; action label `relax_dwell`.
- Produces: the decision path `P → t_cap → guard.allow → _tu_demand → STEP_MAX → sb block` with `guard.allow` unconditional; the `_curve_decision(self, monkeypatch, *, v, kappa, delta_err, torque)` test helper kept (it no longer asserts a hold).

- [ ] **Step 1: Write the failing tests** (in `TestDirectTracking`):
  - `test_no_hold_floor_in_curves`: v=20, κ=0.006, standing torque 0.15 with δ_err +0.002 (P ≈ 0.122 frac, same side, below the old hold) → `state['target_frac'] == pytest.approx(0.122, abs=0.005)`; `not hasattr(mod, 'hold_factor')`; `'hold_f' not in state and 'relax_ticks' not in state`.
  - `test_overshoot_in_deep_curve_reverses_at_once`: v_true 9, κ_meas 0.012, κ_des 0.008 (overshoot side), standing torque 0.3, `tick_count=999` → the decision's `target_frac < 0.3` and `state['action'] == 'ramp'` (no dwell bridge).
  - In `TestStallBreakawayV2`: `test_block_zeroes_same_side_push_whatever_the_label`: `sb_block=10`, `sb_dir=1.0`, standing left torque −0.3, a left decision → `target_frac == 0.0`.
  - `test_payload_exact_key_set`: remove `hold_f`, `hold_cap`, `relax_ticks` from the expected set.
- [ ] **Step 2: Run** `-k "no_hold_floor or reverses_at_once or whatever_the_label or payload_exact"`. Expected: FAIL.
- [ ] **Step 3: Implement** in `latcontroller.py`: delete `HOLD_AY_BP`, `hold_factor`, `HOLD_CAP_DRIFT_M`, `RELAX_DWELL_TICKS`, `RELAX_DWELL_KAPPA`, `lookahead_m`, `hold_f`/`hold_cap`/`held`/`held_target`, `deep_relax`/`relax_ticks`/`dwell_ticks`/`dwelling`, the R8 trim, the hold-floor, the dwell bridge, the `if not dwelling` guard exemption (`guard.allow` always runs), the `'relax_dwell'` entries in the label-expiry tuple and the sb `action != 'relax_dwell'` conjunct, the sb-block `state['action'] == 'ramp'` conjunct, the three state keys and telemetry fields. `LKA_MAX_V`/`LKA_BLEND_V`/`UNWIND_STEP_GAIN` stay (the unwind boost). The module and init docstrings drop the hold/dwell sentences.
- [ ] **Step 4: Prune the tests.** Delete: `TestRelaxDwellLowSpeedGate`, `TestFrictionGatesDeleted`, the five `test_dwell_*` tests in `TestLkaSpeedBlend`, `TestControllerHorizonDecoupled.test_hold_cap_ignores_lat_delay`, `TestStallBreakawayV2.test_relax_dwell_no_trip` and `.test_block_does_not_zero_a_relax_dwell_hold`, `TestDirectTracking.test_hold_floor_kept_in_curves`, `.test_in_band_overshoot_trims_from_hold`, `.test_large_overshoot_still_reduces_torque` (both rewritten on the FF law in Task 2), `.test_relax_dwell_exempt_from_guard`; `test_hooks.py::TestHoldFactor` whole. Then `grep -nE "hold_f|hold_cap|held_target|relax|dwell|HOLD_AY|hold_factor|HOLD_CAP" plugins/bmw_e9x_e8x/tests/*.py plugins/bmw_e9x_e8x/bmw/*.py` prints nothing.
- [ ] **Step 5: Run** the suite and the harness. Expected: suite green; harness 24 passed.
- [ ] **Step 6: Commit** `bmw: curve hold machinery removed (hold factor, hold-floor, R8 trim, relax-dwell)`.

### Task 2: Curve feed-forward

**Files:**
- Modify: `plugins/bmw_e9x_e8x/bmw/latcontroller.py`
- Test: `plugins/bmw_e9x_e8x/tests/test_latcontroller.py`

**Interfaces:**
- Consumes: the Task 1 decision path.
- Produces: module constants `CF_AY0`, `CF_GAIN`, `CF_MAX_NM`, `SL_AY_MAX`; pure function `curve_ff_nm(a_ref: float) -> float`; state keys `a_ref`, `ff_nm`, `demand_nm`; telemetry `ff_nm`, `a_ref`.

- [ ] **Step 1: Write the failing tests.** Pure function (module level in `test_latcontroller.py`): `test_curve_ff_zero_inside_deadband` (`curve_ff_nm(0.29) == 0.0 == curve_ff_nm(-0.29)`), `test_curve_ff_sign_and_gain` (`curve_ff_nm(1.3) == approx(1.0)`, `curve_ff_nm(-1.3) == approx(-1.0)`), `test_curve_ff_cap` (`curve_ff_nm(5.0) == 2.0`, `curve_ff_nm(-5.0) == -2.0`). In `TestDirectTracking` (engaged at the stated v, `tick_count=999` before the decision; torque pre-set so `STEP_MAX` does not bind unless the test says so; read `state['target_frac'] * 12`):
  - `test_decision_is_ff_plus_p`: v 25, κ_ref 0.002 (`a_ref` 1.25 → ff 0.95), κ_meas chosen so `P_nm = +0.60` → target ≈ 1.55 Nm (±0.05).
  - `test_overshoot_trims_the_sum`: same ff, `P_nm = −0.40` → target ≈ 0.55 Nm, still positive, `action == 'ramp'`.
  - `test_large_overshoot_crosses_zero_under_guard`: ff 0.95, `P_nm = −1.50`, guard committed `+1` → first decision target 0.0 (`tu_guarded` +1); after `GUARD_PERSIST_S` of livePose ticks the next decision is negative.
  - `test_ff_uses_true_speed_not_gain_floor`: `v_true = 1.0` (livePose), κ_ref 0.05, κ_meas 0.05 → `state['ff_nm'] == 0.0` (a_ref 0.05) although the gain reference speed is 2.778.
  - `test_ff_low_speed_turn_bounded`: v 5.0, κ 0.05 (`a_ref` 1.25) → `state['ff_nm'] == approx(0.95)`.
  - `test_ff_capped_and_step_limited_on_kappa_spike`: v 30, κ_ref 0.003 (`a_ref` 2.7), κ_meas 0.003, torque 0.0 → `state['ff_nm'] == 2.0` and `abs(state['target_frac']) <= step_max(30) + 1e-9` (= 0.05).
  - `test_s_curve_ff_reversal_passes_guard_then_takes_up`: contacts seeded, steady right curve (ff +0.95, torque ≈ +0.08 frac, gap 77.6), then κ_ref flips to the mirror-image left curve with κ_meas following: the first decisions hold 0 (guard), the demand's sign persists, the guard releases, the ramp crosses zero and `state['takeup'].fires == 1`.
  - `test_bias_frozen_above_a_ref_gate` (in `TestSlopeBias`): κ_des 4e-4 at 25 m/s (`a_ref` 0.25), slope on, 200 ticks → `sl_learn` False and `sl_bias` unchanged; at κ_des 2e-4 (`a_ref` 0.125) → `sl_learn` True.
  - `test_guard_judges_the_full_demand` (replaces `test_guard_judges_the_biased_demand`): ff 0 (straight), bias −0.10, P +0.05 → as before the guard turns to −1 and a negative target appears; and with ff +0.95 at v 25 / κ_ref 0.002 and P −0.40 the guard side stays +1.
  - `test_sb_block_zeroes_ff_push` (in `TestStallBreakawayV2`): right curve (ff +0.95), `sb_block=10`, `sb_dir=−1.0` (a right push is same-side) → decision target 0.0; with `sb_dir=+1.0` the same decision passes (> 0).
  - `test_payload_exact_key_set`: add `ff_nm`, `a_ref`.
- [ ] **Step 2: Run** `-k "curve_ff or ff_ or trims_the_sum or crosses_zero or a_ref_gate or full_demand or payload_exact"`. Expected: FAIL (`curve_ff_nm` missing).
- [ ] **Step 3: Implement** in `latcontroller.py`: the four constants with a ≤ 3-line comment each (spec provenance: 440 steady windows on 4e9–4ec, ~1.5 Nm per m/s² beyond ~0.25; 55 % on purpose); `curve_ff_nm` at module scope beside `gain_reference_speed`; in the livePose block after `kappa_scale`: `a_ref`, `ff_nm`, `_p_nm` (P alone), the slope gate with `abs(a_ref) < SL_AY_MAX`, `state['demand_nm'] = ff_nm + _p_nm + (sl_bias if slope else 0.0)`, `guard.observe(demand_nm / (T_CAP_SLOPE_BASE * kappa_scale * v * v))`; in the decision `target_nm = state['demand_nm']`; telemetry. Update the module docstring law lines.
- [ ] **Step 4: Run** the new tests, then the suite and the harness. Expected: green; harness 24 passed (on straights ff is 0 and the gate is open, so the law is unchanged).
- [ ] **Step 5: Replay bound.** Run `python3 <scratchpad>/ffcheck.py` (steady 2-s windows on 4e9–4ec, `ff_nm` from the window's `a_ref` against the torque the rack actually needed, bias excluded). Expected: `ff / need` median 0.50–0.55; `|ff| > |need|` in ≤ 15 % of same-sign windows; over-supply p99 ≤ 0.40 Nm and max < 1.0 Nm (measured 2026-10-20 at these constants: 0.52, 14 %, +0.39, +0.79). A worse result means the constants in the code differ from the spec.
- [ ] **Step 6: Commit** `bmw: curve feed-forward on the reference lateral acceleration (CF_GAIN 1.0, deadband 0.3, cap 2 Nm)`.

### Task 3: Docs

**Files:**
- Modify: `plugins/bmw_e9x_e8x/LATERAL_CONTROLLER.md`, `docs/superpowers/specs/2026-10-20-bmw-curve-feedforward-and-audit-design.md`

- [ ] **Step 1:** `LATERAL_CONTROLLER.md` "Current law": the law block gains `a_ref`, `ff_nm`, `demand_nm`; the Decision paragraph becomes the Task 2 order (no hold, no dwell); the constants table drops `HOLD_AY_BP`, `HOLD_CAP_DRIFT_M`, `RELAX_DWELL_*` and adds `CF_AY0`, `CF_GAIN`, `CF_MAX_NM`, `SL_AY_MAX`; §8 drops the `relax_dwell` row; §9 drops `hold_f`, `hold_cap`, `relax_ticks` and adds `ff_nm` (Nm), `a_ref` (m/s²); §12's Slope gate line adds `|a_ref| < 0.15 m/s²`; §13 drops the hold/relax rows and adds the FF to the livePose row. Add a dated entry under History: "2026-10-20 — curve feed-forward; hold machinery removed (routes 4ec seg 22/23)".
- [ ] **Step 2:** Spec header: `Status: Part A implemented 2026-10-20 (plan docs/superpowers/plans/2026-10-20-bmw-curve-feedforward.md); Part B merged 2df1b63.`
- [ ] **Step 3:** `grep -nE "hold_f|hold_cap|relax_dwell|HOLD_AY|HOLD_CAP|held_target" plugins/bmw_e9x_e8x/LATERAL_CONTROLLER.md` prints only lines below the `## History` heading (dated notes) or in §1–§7.
- [ ] **Step 4: Run** the suite and the harness. Expected: green; 24 passed.
- [ ] **Step 5: Commit** `docs: lateral controller current law with the curve feed-forward`.

### First drive (not a task — the spec's verification list, for the executor's final message)

In-curve κ shortfall (κ_ref − κ_act in steady bends, 3–7e-4 on 4e9–4ec) at least halved; the seg-22-type outside drift ≤ 0.3 m; `sl_bias` swings smaller than 4ec's −0.25…−0.03; no overshoot or oscillation in low-speed turns; straights unchanged (`ff_nm` 0 there); `tu_fires`/min and `tu_guarded`/min about the same as 4ec (3.0, 12.8).
