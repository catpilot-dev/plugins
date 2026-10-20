# BMW lateral: curve feed-forward, and a lateral-stack audit (design)

Status: draft for user review, 2026-10-20. Builds on
`2026-10-18-bmw-direct-tracking-design.md` and `2026-10-19-bmw-slope-bias-design.md`.

Evidence routes: 4e9, 4ea, 4eb, 4ec (114 min Direct-active; scratchpad
`ffit.py`, `ev22.py`, `slcurve.py`, `audit_ev.py`).

## Part A — curve feed-forward

### Problem

Route 4ec (user: best drive so far) understeers in gentle bends and carries a
stale left bias into the next right bend:

- Seg 22, 13:13:10–30, two left bends (~1 m/s², 75–85 km/h): the car turned at
  −11e-4 against −20e-4 asked for ~5 s (δ_err −24e-4, −1.2…−1.9 Nm) and drifted
  +0.07 → +0.87 m and +0.26 → +1.35 m toward the outside.
- Seg 23, 13:14:11–36, a gentle right bend (+2…4e-4): it needed ~+0.3 Nm; the
  −0.2 Nm bias learned in the left bends cancelled it, the car ran near
  straight 0.3–0.64 m left, and the bias took ~25 s to unwind.

Direct is P-only, so any steady torque needs a standing error. Slope
Compensation supplies the straight-road part; nothing supplies the curve part.
440 steady 2-s windows (4e9–4ec, bias excluded):

| a_y (m/s²) | 0.1–0.3 | 0.3–0.6 | 0.6–1.0 | 1.0–1.5 | 1.5–3.0 |
|---|---|---|---|---|---|
| torque, right | +0.03 | +0.29 | +1.18 | +2.15 | +3.16 |
| torque, left | −0.30 | −0.51 | −1.19 | −2.31 | −2.82 |
| κ_ref − κ_act (e-4) | 0.2–3 | 3–4 | 3–4 | 7 | 10–18 |

Roughly `T ≈ 1.5·(|a_y| − 0.25)` Nm; below ~0.3 m/s² rack stiction holds the
angle and no torque is needed (the May 2026 "no FF on κ" ruling holds there).

The May 2026 steering-angle feed-forward (e81f1c0) is not revived: it was linear
in steering angle, fit at one speed, and over-pushed at 34 km/h / κ 0.023.

### Law

    a_ref  = v_true² · κ_ref                      (κ_ref = the controller's reference, Arc-smoothed)
    ff_nm  = clip( CF_GAIN · sign(a_ref) · max(|a_ref| − CF_AY0, 0), ±CF_MAX_NM )
    demand = ff_nm + P_nm + sl_bias              (every livePose tick)

| Constant | Value | Why |
|---|---|---|
| CF_AY0 | 0.3 m/s² | Stiction holds below this; no FF where none is needed. |
| CF_GAIN | 1.0 Nm per m/s² | ~55 % of the measured need. Under-supplied on purpose: P closes the rest, and P can never be asked to *remove* FF torque at any measured point. At the May failure point (a_y ≈ 2) it gives 1.7 Nm against a measured 3.2. |
| CF_MAX_NM | 2.0 Nm | Transient envelope; t_cap and STEER_MAX still apply. |

Decision path (Direct): `demand` → t_cap clip → noise guard (observes
`sign(demand)` at 20 Hz, replacing the bias-as-error fudge) → take-up demand →
STEP_MAX → stall-breakaway block. No hold-floor, no R8 trim, no relax-dwell:
FF is the physical floor, and an overshoot gives a P that trims the sum down
by itself.

Slope learning gate gains `|a_ref| < 0.15 m/s²` (κ < 4e-4 at 80 km/h): on 4ec
the bias change tracked the model's gentle-bend curvature (r +0.72), not roll.

No new switch (rollback: `CF_GAIN = 0`, one constant). Telemetry: `ff_nm`,
`a_ref`.

### Verification

- Unit: zero inside the deadband, sign, cap, FF+P sum, overshoot trims the sum,
  guard judges the full demand, bias frozen above the a_ref gate.
- Replay 4e9–4ec: `ff_nm` never exceeds the torque actually needed in any
  steady window.
- First drive: in-curve κ shortfall (3–7e-4 today) at least halved; the seg-22
  outside drift ≤ 0.3 m; sl_bias swings smaller; no overshoot or oscillation in
  low-speed turns; straights unchanged (FF is 0 there).

## Part B — audit of the lateral stack

Scope: `bmw/latcontroller.py`, `bmw/takeup.py`, `lane_keeping/`, the BMW
Driving-panel rows and params, their tests and docs. Not in scope: carstate,
carcontroller, DCC, lka_mode.

What the four Direct drives show (`audit_ev.py`):

| mechanism | activity | verdict |
|---|---|---|
| legacy rest state (hold_zero/curve, hysteresis, EMA escape, cancel_tol) | 0 % — Direct bypasses it | **delete** |
| relax-dwell | 0.1–0.3 % of ticks | **delete** (with FF the conflict it bridged is gone) |
| hold_f / hold-floor / R8 | hold_f > 0 for 15 % of ticks | **delete** (FF replaces it) |
| driver-override observers (ov_*) | 0–4 fires per drive, telemetry only, Phase 2 never built | **delete** |
| stall-breakaway v2 | on; 1–3 trips per drive | **keep**, re-audit the trips after FF |
| DirectContacts persistence | seed 47° off on 4e9, relearned in 3 segments | **delete** (keep the online learner) |
| Lane Keeping position loop | forced off by Direct; user keeps it off | **retire** |

### Decisions (each needs the user's ruling; recommendation in bold)

- **D1 Direct is the only law.** Delete the legacy path: `at_rest`,
  hold_zero/hold_curve, `HOLD_BAND`/`HOLD_BAND_ENTER`, `HOLD_EMA_*`/`derr_ema`,
  `cancel_tol` and its LKA cadence rule (`UNWIND_CADENCE_TQ`), the
  `HoldHysteresis` param. The `DirectTracking` switch and row go, and with them
  lane_keeping's cross-plugin Direct machinery (R11: `_direct_switch`,
  `_direct_tracking_on`, `direct_latched`/`lateral_engaged` accessors).
  R10 stays as a hard dependency: if `plugins.lane_keeping.register` is not
  `HOOK_READY` at init, the hook does not patch `lac.update` (stock controller
  runs) and logs why; `plugin.json` lists `lane_keeping` under `dependencies`.
  Cost: the A/B baseline is gone from the device; it stays in git (9af3c87).
- **D2 Hold machinery goes with the FF**: `hold_factor`/`HOLD_AY_BP`,
  `hold_cap`/`HOLD_CAP_DRIFT_M`/`lookahead_m`, `held_target`, hold-floor, R8,
  relax-dwell (`RELAX_DWELL_*`, dwell fade, R3), `relax_ticks`.
- **D3 Retire the Lane Keeping position loop.** `anchor.py` becomes the
  reference conditioner: Arc (horizon schedule, curvature pull-back), the
  0.15 s low-pass, lane-change bypass, lane-geometry telemetry. Deleted:
  gap/excess/pursuit, DC tracker, predictive deadband, asym gate, hard floors,
  `LaneKeepEnable`/`LaneKeepDriverSide` and the other position params, the
  Lane Keeping Driving-panel row. Arc Smoothing is always on (its switch goes).
  Cost: a future "position loop" starts from git, not from live code.
- **D4 Delete the driver-override observers** (`OV_*`, the marker-delimited
  block, `ov_*` state and telemetry, `TestOverrideObservers`).
- **D5 Keep stall-breakaway v2** (`SB_*`, `StallBreakaway` switch). Re-audit
  once FF has a drive: if its trips are all corner-exit unwinds, delete next.
- **D6 Delete contacts persistence**: `DirectContacts` param, seed,
  write-on-disengage, `parse_contacts`/`format_contacts`.
- **D7 Telemetry trim.** Drop `desired_raw`, `hold_band`, `hb_enter`,
  `derr_ema`, `at_rest`, `relax_ticks`, `hold_f`, `hold_cap`, `jerk_pred`,
  `a_y_meas`, `sb_on`, `ov_*`; drop `model_action_t`/`DT_MDL`/
  `MODEL_ACTION_T_FALLBACK` (they only fed `jerk_pred`). Add `ff_nm`, `a_ref`.
- **D8 Comments and docs.** One rationale per rule, in the present tense;
  history leaves the code (git and the dated log in `LATERAL_CONTROLLER.md`
  keep it). The init docstring describes the current law. `LATERAL_CONTROLLER.md`
  gets a one-page "Current law" at the top; the dated entries stay as history.
- **D9 Delete the analysis scripts in `bmw_e9x_e8x/tests/`** for reverted
  designs (adaptive gains, micro-stepping, PID sims, friction sweeps,
  plant-gain fits, `diagnose_00000296`, `replay_*`, `on_device_probe`): ~25
  files, none run by the suite.
- **D10 Keep** `V_GAIN_FLOOR`, `LKA_MAX_V`/`LKA_BLEND_V`, `UNWIND_STEP_GAIN`
  (on-car verified, small), `STEP_MAX` schedule, `t_cap`, `kappa_scale`,
  `CONTROL_HORIZON_S`, the noise guard, take-up and contact learner, Slope
  Compensation and its switch.

Expected size: `latcontroller.py` ~1700 → ~700 lines, `test_latcontroller.py`
~3100 → ~1200, `anchor.py` ~400 → ~150, `test_anchor.py` 63 → ~25 tests.

### Order and proof

1. **D1 + D4 + D6 + D7 (bit-identical).** Direct output on the 4e9–4ec rlogs
   replays identical to 9af3c87 before and after.
2. **FF + D2 (behaviour change).** Unit tests, the replay bound above, then one
   drive.
3. **D3 + D9 (bit-identical).** Arc path replays identical.
4. **D8** last, on the settled code.

Each step is its own branch and commit set; the suite is green at every step.
