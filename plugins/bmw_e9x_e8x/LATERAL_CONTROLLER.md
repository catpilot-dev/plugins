# BMW E9x/E8x Lateral Controller — Design & Tuning Reference

This document is the canonical reference for the lateral controller registered by `bmw/latcontroller.py::on_lat_controller_init` (hook: `controls.lat_controller_init`). It describes **what** the controller does, **why** it's shaped this way for the BMW hydraulic rack, **how** the single-knob timing design propagates, and **what was tried and rejected**. Future maintainers should be able to retune without re-litigating the failed experiments.

## Current law (2026-10-20)

`bmw/latcontroller.py` installs Direct Tracking as `lac.update` when the
`lane_keeping` plugin is loaded (`HOOK_READY`); without it the stock lateral
controller runs and a warning is logged. The exact field set of the
`bmw_lat_control` telemetry is in §9; the code map in §13. Everything below the
**History** heading is the dated record of how the law got here.

**Signal path.** `lane_keeping` turns the model's curvature into the reference:
an arc through the planned path 1.5–2.5 s ahead (by speed, pulled back to 1.5 s
when a bend is coming), low-passed 0.15 s. The controller tracks that reference
with no filtering of its own.

**Error and gain** (rear-axle bicycle model, `L = CP.wheelbase`):

    delta_err = atan(kappa_ref·L) − atan(kappa_meas·L)      kappa_meas = yaw rate / v_true
    v         = max(v_true, V_GAIN_FLOOR)                   gain reference speed
    P_nm      = T_CAP_SLOPE_BASE · kappa_scale(|kappa_ref|) · v² · delta_err
    a_ref     = v_true² · kappa_ref                          reference lateral accel (m/s²)
    ff_nm     = clip(CF_GAIN · sign(a_ref) · max(|a_ref| − CF_AY0, 0), ±CF_MAX_NM)
    demand    = ff_nm + P_nm (+ sl_bias)                    every livePose tick

**Decision**, every `CONTROL_HORIZON_S / 2` (6 livePose ticks, 300 ms) while active, in order:
the demand on the full error (no on-target band — a sub-friction target is a
hold; the feed-forward supplies the steady curve torque, ~55 % of the measured
need, and P closes the rest); clip to `t_cap`; relax-dwell bridge in a deep
measured curve (the current torque is held through a reference dip, within
`t_cap`); the noise guard (a sign change of the demand commands 0 until the new
sign has persisted `GUARD_PERSIST_S`);
`STEP_MAX` toward the target (×`UNWIND_STEP_GAIN` when shedding in the LKA
regime); a same-side push during a stall-breakaway block is zeroed. The torque ramps to the target over the next 300 ms, one CAN tick
at a time. The free-play take-up (`bmw/takeup.py`) commands `TAKEUP_NM` across
the servo lash on a change of push direction once the contacts are learned,
then hands back to the ramp. Inactive: no decision runs; torque, target and
ramp are 0, so every engagement ramps from 0.

**Constants**

| constant | value | role |
|---|---|---|
| `CONTROL_HORIZON_S` | 0.65 s | cadence and ramp spread are each half of it |
| `T_CAP_BASE_NM`, `T_CAP_SLOPE_BASE` | 2.0 Nm, 1.0 | `t_cap = min(STEER_MAX, BASE + SLOPE·kappa_scale·v²·|delta_ref|)` |
| `T_CAP_SCALE_KAPPA` / `_BP` | [1e-3, 1e-2, 2e-2] → [1.25, 2.5, 3.0] | gain scale on `|kappa_ref|` (straight 1.25 since 2026-10-21, trial) |
| `V_GAIN_FLOOR` | 2.778 m/s | gain reference speed floor |
| `CF_AY0`, `CF_GAIN`, `CF_MAX_NM` | 0.3 m/s², 1.0 Nm per m/s², 2.0 Nm | curve feed-forward deadband, gain, cap |
| `RELAX_DWELL_TICKS`, `RELAX_DWELL_KAPPA` | 20 ticks, 0.010 1/m | dwell length, deep-curve threshold |
| `STEP_MAX_V` / `_BP` | [15, 28] m/s → [0.10, 0.05] frac | per-decision step cap |
| `LKA_MAX_V`, `LKA_BLEND_V`, `UNWIND_STEP_GAIN` | 8.5, 12.0 m/s, 3.0 | LKA regime blend band (dwell length, unwind boost) |
| `SB_*` | see module scope | stall-breakaway v2 (`StallBreakaway` switch, default off) |
| `SL_TAU_S`, `SL_BIAS_MAX_NM`, `SL_MIN_V`, `SL_KAPPA_DES`, `SL_KAPPA_MEAS`, `SL_AY_MAX` | 20 s, 0.25 Nm, 40 km/h, 1e-3, 1.5e-3, 0.15 m/s² | Slope Compensation (`SlopeCompensation` switch, default off) |
| `GUARD_PERSIST_S`, `TAKEUP_NM`, `TAKEUP_TIMEOUT_S` | 0.15 s, 0.35 Nm, 0.4 s | noise guard, take-up (bmw/takeup.py) |

**Switches** (Driving panel, param files under `bmw_e9x_e8x/data/`):
`StallBreakaway` (read at drive start) and `SlopeCompensation` (read at each
engagement). Both default off.

**Verification.** Unit tests in `tests/test_latcontroller.py` and
`tests/test_takeup.py`; a differential replay against the previous build is the
proof for every refactor that claims no behaviour change.

**Rollback.** `CF_GAIN = 0` switches the feed-forward off inside this law (P
alone, no curve hold-floor) — a law no drive has run. The driven baseline is
`dev` @ 2df1b63 (4ec's law plus the audit); roll back by redeploying it.

---

## History

Dated notes in the order they were written. Mechanisms they describe may be
gone; the current law above is authoritative.

> **2026-07-28 — SAFETY ARCHITECTURE: the lateral controller never gives up in
> a turn. The ISO 11270 cancel machinery was removed.** The controller's
> contract is now singular: **track the commanded curvature, always.**
>
> **ISO 11270 is enforced at the SYSTEM level, not here.** Lateral acceleration
> `a_y = v²·κ`; the only term the car can trim online without leaving the
> commanded path is `v`. That is **speedlimitd's** job — it caps `vEgo` for
> upcoming curves so the commanded `a_y` stays within comfort/ISO limits. The
> lateral layer then faithfully tracks whatever curvature it is handed. This
> is a clean separation of duties: **longitudinal owns `a_y` (via `v`),
> lateral owns trajectory (via κ tracking).**
>
> **Why the in-controller cancel was net-harmful.** The removed guard drained
> steering torque to zero mid-turn whenever measured `a_y` or predicted jerk
> exceeded a threshold (gated on plant overshoot). Draining torque in a turn
> does not reduce lateral acceleration in any useful way — with a hydraulic
> rack that has no self-centering near center but strong self-aligning torque
> (SAT) at angle, releasing torque lets SAT unwind the wheel and the car
> **runs wide**, converting a comfort exceedance (transient, bounded) into a
> **trajectory failure** (the car leaves its lane). The incident record is
> unanimous — every cancel firing caused harm and none prevented any:
>
> | Route | What the cancel did |
> |---|---|
> | 326 | spurious cancels + torque churn, up to 114 `cancel_accel`/route pre-fix |
> | 385 seg 27 | over-latch → `cancel_accel` dump → deep unwind limit cycle in a hairpin |
> | 2ba seg 22 | zeroed torque while still **under**-tracking → car drifted **1.29 m** outside the lane |
> | 3ce (segs 15/26/31) | drain-rebuild hunting cycle, 54% zero-torque occupancy in curves |
> | 3cf seg 15 | cancel firing through ~75% of a single sharp curve |
>
> **What still bounds the lateral command** (all of these track *toward* the
> commanded κ, none abandon it):
> 1. **P-law reversal on overshoot** (target reverses immediately; the executed
>    unwind is STEP_MAX-rate-limited — slower than the removed drain by design;
>    the fast drain is what ran the car wide in the field record) — the moment
>    the plant turns past `κ_des`,
>    `δ_err` flips sign and the P-term commands torque the other way. Tracking
>    back to the command *is* the overshoot correction; no separate guard needed.
> 2. **`STEP_MAX`** — per-decision torque step cap (speed-scaled 0.10→0.05),
>    the jerk bound (~4 Nm/s max slew, halving to ~2 Nm/s at highway).
> 3. **Panda `STEER_MAX`** — the hard actuator limit (12 Nm), enforced at the
>    gateway.
> 4. **Driver supervision** — hands-on, the ultimate authority.
>
> **What was removed from `bmw/latcontroller.py`:** `accel_guard_threshold()`,
> `ACCEL_GUARD_FLOOR/RATIO/MARGIN`, `LATERAL_CURVATURE`, `LATERAL_JERK_BP`, the
> `overshooting` predicate (incl. its torque-direction conjunct and
> `KMEAS_SIGN_FLOOR`), `BMW_LATERAL_ACCEL`/`BMW_LATERAL_JERK`, the
> `cancel_accel`/`cancel_jerk` branch and its drain, and the module-scope
> `ISO_LATERAL_ACCEL`/`ISO_LATERAL_JERK` imports. **`cancel_tol` was kept** —
> it is *not* ISO machinery but `HOLD_BAND` boundary hygiene (it stops an
> in-flight push ramp once the error is on-target, draining to the
> sign-guarded capped hold; it tracks the command tighter, it does not abandon
> the turn). `a_y_meas` / `jerk_pred` are still computed but are **telemetry
> only** now — nothing reads them for control. **§7 (ISO 11270 comfort guards)
> below and every "ISO guard / cancel_accel / cancel_jerk" mention elsewhere in
> this document are HISTORICAL as of this date.**
>
> **2026-07-03 — stiction special-casing removed.** All FRICTION-based command
> shaping is gone: the `breakaway` ±FRICTION amplification, the `brake_zero`
> one-shot reverse pulse, and the reverse-FRICTION pulses in `cancel_tol` and
> the ISO cancels (all drains now go to 0). Route 384 telemetry showed ~40% of
> in-turn decisions were friction pulses — ±0.6 Nm torque reversals that
> churned rather than corrected. The straight-line wobble those mechanisms
> were built to manage is modelV2 vision noise, mitigated by the κ-gated
> delta_err box filter (§4) instead. Sub-friction targets are now commanded
> as-is (soft actuation deadband); cancels relax to 0 and rely on tire
> aligning torque to unwind (at guard-firing angles it exceeds rack stiction —
> route 380/384 evidence). Sections below describing `breakaway`/`brake_zero`
> or reverse-FRICTION drains are historical where they conflict with this note.
> `FRICTION` remains only as the tolerance-cancel guard threshold.
>
> **2026-08-13 — `FRICTION` fully retired (user ruling).** The constant
> claimed to be the rack's breakaway torque; the measured breakaway is
> 2.0–2.75 Nm (4× higher) and is not a constant at all — route 3f2's
> covariate study showed the knee spanning wider than the usable torque
> range across lateral load, speed and surface. Its two surviving
> ε-gates were deleted rather than re-tuned: `cancel_tol`'s
> `|target_frac| > FRICTION` was algebraically redundant with `HOLD_BAND`
> (the same threshold expressed in torque coordinates through the P gain,
> agreeing only at 25 m/s and diverging as 1/v² elsewhere), and
> `deep_relax`'s `|torque| > FRICTION` was vacuous under the deep-curve
> gate (a tracked |κ_meas| > 0.010 curve implies SAT-scale held torque).
> Behavioural consequence of the first deletion: terminal push ramps with
> small stale targets now drain to the held target on arrival instead of
> completing — less residual torque at rest entry. Breakaway is OBSERVED
> (at that date, by the push-budget machinery — retired 2026-08-14, see the
> dated note below; the ruling that it is observed, never predicted, stands),
> never predicted; `HOLD_BAND` is sized by the
> measured noise floor, not by any rack property. Every FRICTION mention
> below this date is historical. (`CP.lateralTuning.torque.friction = 0.16`
> still exists in carParams — it feeds the stock `LatControlTorque` object
> whose `update` this controller replaces; inert by construction, kept
> as-is. `interface.py`'s unused `detect_stepper_override` also kept as-is
> by the same ruling.)
>
> **Same date — curvature-dependent hold.** Inside the tolerance band the
> target is `hold_f·torque` instead of 0, with
> `hold_f = interp(|κ_des|, HOLD_KAPPA_BP=[0.004, 0.010], [0, 1])`. In a curve
> the torque that achieved on-target ≈ the self-aligning torque (which beats
> rack stiction above ~40° wheel — target-0 there was the driver of the
> route 380/384 0.6 Hz limit cycle). Straights (|κ_des| < 0.004, above the
> ±0.003 modelV2 wobble band) keep the pure stiction-hold. The held value
> re-derives from `state['torque']` each decision — bounded by the P-term's
> own command, no learning state, no ratchet (the failure mode of the
> reverted hold-bias integral). The tolerance-cancel drain targets the same
> `hold_f·torque` ("stop the ramp, keep what you have" in curves); ISO
> overshoot cancels still drain to 0.
>
> **2026-07-03 (route 385 seg 27 review batch).** Four fixes/mechanisms after
> the first on-car data of the new stack still oscillated in a hairpin:
> 1. **Held torque is sign-guarded** — never hold torque opposing the
>    commanded curve (`held·δ_des ≥ 0`); seg 27 showed 400 ms counter-curve
>    holds during overshoot recovery (63 vs 38 deg/s subsequent rate bursts).
> 2. **Held torque is magnitude-capped** at the deadzone-edge P value
>    (`hold_cap = T_CAP_SLOPE·kappa_scale·v²·tolerance`): anything above what
>    P commands while "almost on-target" is arrival momentum, not holding
>    torque (seg 27 latched 0.588 vs steady SAT ≈ 0.15 → cancel_accel dump →
>    deep unwind).
> 3. **`cancel_tol` gated to `action == 'ramp'`** — it fired on hold ramps
>    every in-band tick, stretching cadence 300→550 ms and flooding telemetry
>    with phantom `cancel_tol` (~10 of 11 in-curve ticks).
> 4. **Per-decision step cap `STEP_MAX = 0.10`** — human-style gradual
>    steering: move ≤1.2 Nm toward the P target per 300 ms decision, let the
>    plant respond, re-measure, step again. Design rule: **never apply
>    excessive steering torque abruptly.** Seg 27 had single decisions
>    swinging Δ0.69 frac (8.3 Nm) → 150 deg/s wheel bursts. Max slew now
>    ~4 Nm/s; full authority builds in ~1.5 s (accepted — speedlimitd slows
>    for curves, ISO guards still cancel instantly).
> Telemetry gains `hold_cap`; `kappa_scale` is computed per tick (shared by
> the P-term and the hold cap).
>
> **2026-07-04 (route 38b follow-up).** Straights on the new stack: "almost
> perfect" (user verdict). Two changes from the mild-turn wobble analysis
> (segs 10/11 — small 1.6–2.3° p2p wobble in a mild highway left):
> 1. **delta_err filter is BLENDED, not gated**: `w_raw = interp(|κ_des|,
>    KD_BLEND_BP=[0.002, 0.004], [0,1])`, `delta_err = w_raw·raw +
>    (1−w_raw)·filtered`. The old hard KD_GATE=0.002 sat exactly on
>    mild-turn κ, flickering on/off at ~0.5 Hz and passing raw vision noise
>    half the time. Straights unchanged (fully filtered), real curves
>    unchanged (fully raw), lane changes still bypass.
> 2. **κ-dependent DRIFT_M REVERTED to fixed 0.02 m**: the widening logic
>    was wrong — in large curvature the lane margin shrinks and position
>    error matters MORE; allowed drift must not grow with κ. (Route 38b
>    also showed the widening was a bystander in the mild-turn wobble:
>    factor 1.1× there.) Noise-chasing in tight turns is handled by
>    hold_curve + STEP_MAX instead. The **hold cap is decoupled** onto its
>    own fixed reference (`HOLD_CAP_DRIFT_M = 0.10`) so it keeps the
>    field-verified seg-27 scale — deriving it from the now-tight tolerance
>    would have shrunk it below measured steady SAT and broken hold_curve.
> Telemetry gains `de_w` (blend weight).
>
> **2026-07-06 (route 393 segs 7/8 — hold verified working).** Wheel quality
> in tight curves: rate std 7.5–13.4 deg/s, residual p2p ~14° (vs 18–37 /
> 29–54° pre-redesign); held torque 0.06–0.08 mean ≈ measured SAT; hold_cap
> binds ≤5%. One structural improvement from the data — **hold-floor**: a
> same-direction ramp push never commands less than the held torque
> ("keep holding while adding trim — don't ease off while still
> understeering"). 15–31% of in-curve ramp ticks had been commanding
> sub-friction targets that DROPPED torque below the holding level,
> letting SAT unwind the wheel mid-correction. Opposite-sign P (overshoot
> correction) is untouched — torque still reduces the moment the error
> flips sides.
>
> **Same date — filter window 6 → 12 ticks. REVERTED 2026-07-07 after one
> day on-car.** The offline replay (334 s of straights, sign-flip/crossing/
> onset metrics) predicted a free −31% churn — but route 395 segs 16–20
> felt "wobbling and lag" on straights, and the measured torque-response
> lag vs raw error rose **0.44 s → 0.66 s**. Lesson recorded: those replay
> metrics are blind to small-correction **phase lag** (low-frequency weave);
> straight-line feel is the primary regime and the user's seat is the
> authoritative sensor. Window is back to 1×cadence (300 ms box). The
> hold-floor from the same period was verified working on-car (route 395
> seg 7: tight-turn residual p2p 9.9°, best yet) and stays.
>
> **2026-07-09 — STEP_MAX speed-scaled (route 39b seg 18, safety call).**
> Sudden back-and-forth wheel motion in a slight highway left; aggressive
> steps are riskier at speed. `step_max = interp(v, [15, 28], [0.10, 0.05])`
> — highway slew halves to ~2 Nm/s, curves (< 54 km/h) keep full entry
> authority. DRIFT_M deliberately NOT tightened at speed (already 1/v²-tight,
> ~10× below the noise floor at 100 km/h — tightening adds chase pressure).
> **Same date — burst root-caused and fixed (seg-18 tick dump).** Not hold
> flicker (hold_f = 0 throughout) and not step aggression: the ISO overshoot
> gate `(κ_des−κ_meas)·κ_meas < 0` degenerates near κ_meas ≈ 0, where the
> SIGN of κ_meas is yaw noise (±0.0002 observed). A gentle highway-left
> build was repeatedly cancel_jerk'd at κ_meas = +0.0002 (wrong-side by
> noise, jerk threshold 1.5 m/s³ ≡ κ_err 0.0011 at v²=900) while the car
> under-turned; the error grew to 0.005 until a late −0.35 correction swung
> back and forth — a guard-induced limit cycle (block gentle early action →
> force late big action). Fix: `overshooting` additionally requires
> `|κ_meas| > KMEAS_SIGN_FLOOR = 0.0005`. Note jerk_pred also over-predicts
> since STEP_MAX: commanded pushes can only produce ~0.7 m/s³ at highway —
> revisit the guard's role if it misfires again.

> **2026-10-12 — model timing matched to the measured loop delay; controller
> timing decoupled.** Stage delays on straights (routes 4d4/4d5, 0.1–1 Hz
> band): target → torque 0.05–0.10 s, torque → wheel angle 0.30–0.52 s
> (servo + hydraulic rack), angle → yaw 0.33–0.37 s, total 0.90–1.0 s — but
> modeld planned for 0.65 s (`steerActuatorDelay` 0.4 + lagd's 0.2 + DT_MDL),
> so the model's corrections landed ~0.3 s late and drove the sub-Hz in-lane
> sway. `steerActuatorDelay` → 0.7 (lat_action_t 0.95 s). The controller's
> cadence/ramp spread and hold-cap lookahead no longer follow lat_delay: they
> stay on `CONTROL_HORIZON_S = 0.65`, the horizon they were tuned at (a 0.95 s
> horizon would mean a ~500 ms cadence and a hold cap at 47%, below steady
> SAT). Curves measured shorter (~0.25–0.6 s, little data), so expect slightly
> earlier turn-in; 0.6 is the fallback compromise. lane_keeping's gap
> prediction (1.5 × lat_delay) still follows lat_delay by design.
> **REVERTED to 0.4 the same day (route 4d9):** with no add-ons the drive was
> "hardly driveable" — sway-band yaw oscillation ~2× (leg average 19.7 vs
> 10.3 ×1e-4 with Arc Smoothing; 31–50 vs 13–15 on 4d4 at ~45 km/h), the
> model's own corrections overshooting. The measured 0.9–1.0 s is mostly rack
> stiction on small corrections, not a pure delay; feeding it to the model is
> the wrong lever. `CONTROL_HORIZON_S` stays (identical to the old coupled
> value at SAD 0.4).

> **2026-07-12 — relax-dwell (route 3a0 seg 8: "gives up mid-turn").**
> modelV2's κ_des dips 40–50% for ~1 s mid-hairpin and recovers (3× in one
> turn) while κ_meas stays steady. The controller faithfully followed each
> dip down the relax staircase; SAT flung the freed wheel ~20° out of the
> turn, then the step-capped rebuild took 1.5–2 s against SAT. Asymmetry:
> unwinding is instant and free, rebuilding is slow and fought. Fix: in a
> MEASURED deep curve (|κ_meas| > RELAX_DWELL_KAPPA = 0.010, κ_des still
> same-side, torque > FRICTION), an overshoot-side error must persist
> RELAX_DWELL_TICKS = 20 (1.0 s) before the relax path may command below
> the current (hold_cap-clipped) torque; during the dwell the target
> bridges at current torque (action `relax_dwell`). ISO cancels bypass the
> dwell (measured overshoot still cancels instantly); κ_des sign flips
> abort it. True exits proceed after ≤1 s (~0.2 m lateral at 9 m/s).
> hold_cap exonerated by the same investigation (pins were healthy holds).
> Telemetry gains `relax_ticks`.

> **2026-07-19 — live noise observer + tolerance noise-floor (route 3ac
> segs 20-30: straight/mild-turn "unnecessary oscillation").** The hold
> stack was exonerated (hold_f ≡ 0 across all 11 segments); the real driver
> is that at 60–90 km/h the 1/v² tolerance (κ ≈ 2–3e-4) sits 5–10× below
> modelV2's sub-Hz κ_des wander (±1–3e-3), so |delta_err| > tolerance on
> ~70–80% of ticks: torque reversed direction 25–50×/min and the wheel
> stick-slipped in 0.5–1° notches. Cross-correlation proved the chase is
> model-driven (κ_des leads κ_meas by 0.15–1.2 s). A 12-route study showed
> the noise is not speed-scheduled (it mildly *decreases* with v; the
> tolerance just shrinks faster) and grows with lead proximity and low
> laneLineProbs — so it is now observed **live**: fast(1 s)−slow(5 s) EMA
> band-pass of κ_des → 20 s running variance → σ, trained only on
> near-straight engaged ticks (|κ_des| < KN_GATE_KAPPA = 0.002, no lane
> change), frozen elsewhere. The tolerance gets a floor of
> `KN_SIGMA_MULT(1.5)·σ·L`, faded to zero over |κ_des| ∈ KN_FADE_BP
> = [0.002, 0.004] (curves keep the pure kinematic band — allowed drift
> must not grow with curvature) and capped at the kinematic tolerance ×
> KN_DRIFT_CAP_M/DRIFT_M (implied drift ≤ 0.08 m; the route-31b 0.35°
> constant band that drifted 1.3–1.7 m is 3–5× above this cap). Replay over
> 140k near-straight ticks: actionable fraction 70%→36%, error sign-flips
> −30%, cap binds 5%, 0.00% of curve ticks affected. Telemetry gains
> `k_sigma`. **Same date — stale action labels expired:** between cadence
> decisions, a transient label (ramp/relax_dwell/cancel_*) now becomes
> `idle` once its ramp completes; holds keep their label (they re-fire each
> cadence). Occupancy counts over the telemetry stream are now honest;
> the cancel_tol `action=='ramp'` gate is unaffected (in-flight ramps
> never expire).

> **2026-07-19 — sign-persistence gate on the noise floor (route 3b3 "a bit
> left hug", first on-car data of the floor).** The floor is symmetric so it
> can't create a directional bias, but it removed the gentle centering the
> tight kinematic band provided, letting a small pre-existing offset stand:
> 3b3 showed the signed lane offset shift ~0.13 m left and off-centering
> magnitude scale with the floor-widening ratio (|offset| 0.17 m at 1.0× →
> 0.32 m at ≥2×; Spearman(widen, |offset|) = +0.22, well above σ's +0.08).
> Root cause: the floor widened tolerance against the *instantaneous*
> delta_err, lumping zero-mean wander (ignore) with a sustained DC offset
> (must correct) — and the 0.08 m implied-drift cap is per-horizon, so it
> doesn't bound the *integrated* steady offset (which grows until the model's
> re-centering demand exceeds 1.5σ·L → ~0.2–0.3 m at highway). Fix: `de_dc`,
> a slow (τ≈2 s) EMA of delta_err, is a persistence detector — zero-mean
> wander averages toward 0, a sustained offset accumulates. As
> `|de_dc|/tol_kin` grows across `KN_PERSIST_BP = [0.7, 1.3]` the floor fades
> out (`persist_w → 0`), so the steady offset is bounded by `tol_kin` (tight),
> not the floor. Mirrors relax-dwell's "persistence proves it's real." τ and
> band are data-set on 3b3 (8353 lane-aligned near-straight ticks: τ=2 s
> separates centered de_dc/tol_kin ≈ 0.4 from offset ≈ 1.5; the band pulls the
> floor on 75% of offset ticks while keeping it for 90% of centered ticks).
> Telemetry gains `de_dc`, `persist_w`. Closed-loop centering recovery is
> pending on-car verification (open-loop replay validates only the detector).

> **2026-07-22 — Phase 2: noise handling offloaded to lane_keeping.** Route 3bf
> proved the position anchor cannot work through a deadzone: its bias was only
> 1.98× the tolerance and the controller took no action on 44% of correcting
> ticks (a 75 s excursion had the bias saturated, correctly signed, and never
> recovering). The controller is now a faithful tracker. **Deleted:** `DRIFT_M`
> + kinematic `tolerance`, the `KD_BLEND` box filter, the σ-observer noise floor
> (`KN_*`, `k_sigma`), and the sign-persistence gate (`de_dc`, `persist_w`).
> **P acts on the full `delta_err`** — the tolerance subtraction is gone, and it
> was the attenuator. The stiction hold retriggers on a fixed
> `HOLD_BAND = 0.001` rad, sized by rack breakaway (below it the P term commands
> less than friction, so the wheel cannot move) rather than by allowed drift.
> modelV2 noise now lives entirely in `lane_keeping`, which low-passes κ_des
> (`KAPPA_FILTER_TAU = 0.15 s`) and closes the position loop — filtering is safe
> there because its lag becomes position drift, which that loop cancels. The
> modelV2 subscription is dropped (it only fed the deleted filter's lane-change
> gate). Telemetry drops `tolerance`/`delta_err_raw`/`de_w`/`k_sigma`/`de_dc`/
> `persist_w` and gains `hold_band`. **Sections below describing the κ-gated
> delta_err box filter, the kinematic/noise-floor tolerance, or `effective_err`
> as live code are historical where they conflict with this note — see § 3, § 4,
> § 12, and § 13, all updated in place below.**
>
> **2026-07-27 — hold gate moved from curvature to lateral accel (route 3ca
> seg 23).** `hold_f` no longer gates on `|κ_des|` (`HOLD_KAPPA_BP`). Whether
> "drain to zero and let stiction hold" is safe on-target depends on
> self-aligning torque (SAT), which scales as **v²·κ**, not κ alone — the old
> gate silently baked in the ~12 m/s tuning speed of the route 380/384
> hairpin fix. Route 3ca seg 23 (19.4 m/s, κ 0.0033, a_y 1.25) sat below
> `HOLD_KAPPA_BP[0] = 0.004` → `hold_f = 0` → drain → 0.6 Hz-class hunting
> (30% zero-torque decisions in a sustained turn, 1.8× command-wobble
> amplification) — a mild-but-fast curve with plenty of SAT to unwind the
> wheel, misclassified as "straight." Now:
> `hold_f = interp(v²·|κ_des|, HOLD_AY_BP=[0.5, 0.9], [0, 1])`, extracted as
> the pure module-level function `hold_factor(v_ego, kappa_des_abs)` in
> `bmw/latcontroller.py` (unit-testable without constructing the
> controller). `[0.5, 0.9]` preserves full hold at the route 380/384
> operating point (a_y ≥ 0.97), fixes the 3ca seg 23 drain, keeps drain on
> straights and gentle-slow curves (a_y < 0.5, including the reference mild
> curve — 12.4 m/s, κ 0.0023, a_y 0.35 — which damps fine with drain), and
> drops hold at parking speeds regardless of κ (SAT there is far below
> stiction). `hold_f` remains functionally near-binary: the held target
> re-derives from `state['torque']` every ~100–300 ms decision, so partial
> values only persist for the one decision they're computed on. **§ 5 and
> § 8 below use `HOLD_KAPPA_BP`/curvature language in places where that is
> now historical — the live gate is `HOLD_AY_BP` as described here.**

> **2026-08-12 — push budget (route 3f2 seg 10: "wound to breakaway, then
> overshot"). SUPERSEDED — RETIRED 2026-08-14, see the retirement note at
> the end of this header. Everything in this entry is historical: no
> `BUDGET_DEG`, `AngleBudget` param, `angle_budget` bus topic, "Steering
> Push Budget" toggle, or `push_moved`/`budget_spent`/`budget_on` telemetry
> exists in the code any more.** The controller now remembers the steering angle when a push
> (`action == 'ramp'`) begins and tracks how far the wheel has moved from that
> reference. Once it has moved `BUDGET_DEG = 2.0` degrees **in the commanded
> direction**, the per-decision `STEP_MAX` clamp stops applying: the target
> can no longer command *more* torque than currently held, and the step to
> get there is unclamped — the P law sheds torque as fast as it asks instead
> of at the ramping rate. Human-style: **push harder until the wheel
> actually moves, then stop pushing harder and ease off.**
>
> **Why a budget, not a detection threshold.** 2 degrees is not trying to
> detect breakaway — whether the movement was creep through stiction or the
> rack actually breaking free is irrelevant to it — it is an *authority
> budget*: how much wheel movement **one push** may cause open-loop before
> feedback must take over for that push. This is a bound **per push, not per
> event or per drive** — each time `action` re-enters `'ramp'` after having
> left it, `push_ref` re-arms from wherever the wheel currently is (see
> below) and a fresh 2 degrees is available; a curve negotiated as several
> distinct pushes (interleaved with holds, `cancel_tol`, `relax_dwell`, or a
> disengagement) can accumulate open-loop travel well past 2 degrees in
> total. That framing needs no margin against false positives/negatives the
> way a detector would. Physically 2 degrees of
> steering wheel is 0.11 degrees of front wheel (curvature 0.00070 /m,
> roughly 1440 m radius, about 80% of the actual curve in route 3f2 seg 10,
> robust across steerRatio 18-21) — a real steering input — and 45 quanta of
> the 0.04395 degree angle signal, so no signal noise can spend it.
>
> **Route 3f2 seg 10 numbers.** The controller wound torque 0.11 to 0.313
> frac over 1.2 s into a rack delivering only 0-21% of the commanded motion,
> then the rack broke free and the wheel swung to 24.8 degrees. The budget is
> spent at t=663.87 with torque at 2.8 Nm, against the 3.75 Nm peak the ramp
> actually reached at t=664.30 — i.e. it would have capped the overshoot
> roughly 0.43 s and about 1.0 Nm earlier. It does *not* fire during the
> preceding 1.4 s stall (which accumulated only 1.63 degrees of movement), so
> ordinary creep against stiction is unaffected.
>
> **Asymmetric `STEP_MAX` once spent.** Winding torque up is fought by the
> rack (stiction); winding it down is free (self-aligning torque does it).
> The existing speed-scaled `STEP_MAX` clamp is the right rate limit while
> ramping blind into an unknown plant response, but it is the wrong rate
> limit for shedding torque once the plant has already told you it moved: on
> route 3f2 seg 10 the symmetric clamp needed 0.65 s to unwind an overshoot
> that took only 0.4 s to build. Once `state['budget_spent']` is true: a
> same-direction target asking for *more* torque is clamped to (never past)
> the current torque — stop pushing harder, freeze there; a same-direction
> target asking for *less* sheds toward zero unthrottled; and past zero (an
> overshoot reversal, a new push the other way) is never frozen — it is
> allowed at most one `step_max` beyond zero in that single decision, the
> same rate limit any other push gets. **Review fix (2026-08-12,
> pre-merge):** the first cut of this compared `abs(target_frac)` against
> `abs(state['torque'])` with no sign check, which on a reversal either froze
> torque at its wrong-direction value (the controller giving up mid-turn —
> forbidden by this file's own SAFETY ARCHITECTURE contract) or, when the
> counter-target was smaller in magnitude, applied no cap at all (up to a
> single-decision Δ0.578 frac / 6.9 Nm swing — exactly what `STEP_MAX` exists
> to prevent). Caught in review before deployment; never shipped.
>
> **`AngleBudget` param** (default **off**) gates the whole mechanism, read
> **once** at controller construction into a plain local (not re-read
> per-tick or on a cache timer — a first cut cached it 5 s, but any
> `Path.read_text()` on controlsd's 100 Hz RT thread risks costing a control
> frame under eMMC contention). The init read is the **boot state**; the
> Driving-panel "Steering Push Budget" toggle **hot-applies mid-drive** via
> the `angle_budget` plugin-bus topic, polled at livePose rate into
> `state['budget_on']` (tmpfs + memory — still no file I/O on the RT
> thread). The param file the toggle also writes is what each later drive's
> init read picks up. See § 12 ("To toggle `AngleBudget`") for the
> procedure and how to verify from telemetry which state a drive ran in. With the param off, `state['budget_spent']` is always
> `False` and every decision takes the pre-existing symmetric-`STEP_MAX` path
> unchanged — behaviour is bit-identical to before this date.
>
> **Gated on `active`** (review fix): the angle-capture block runs every CAN
> tick regardless of engagement, and the decision state machine that sets
> `state['action']` is not itself gated on `active` — without this, steering
> input while disengaged (hands-on override, manual parking, etc.) would
> accrue into `push_moved`, and a push beginning right after re-engagement
> could start already spent. `CS.steeringPressed` is **not** a usable
> substitute gate on this car — it is a voice-control button ORed with
> `gasPressed`, not a hands-on-wheel signal.
>
> Uses `CS.steeringAngleDeg` (positive = LEFT; torque fraction is negative =
> LEFT, the opposite sign convention — the code multiplies `push_moved` by
> `-torque` to test "moved the way we asked"). The signal carries a constant
> ~-1.58 degree physical alignment offset, cancelled by working only in
> deltas from the captured reference (`push_ref`), never the absolute angle
> — verified the budget fires identically regardless of a -1.58 or a +7.3
> degree constant offset. The reference resets to `None` (a fresh push
> re-arms from wherever the wheel is) whenever `action` leaves `'ramp'`.
> `getattr(CS, 'steeringAngleDeg', 0.0)` degrades safely (budget never
> spends) if the field is ever absent from `CS`. Telemetry gains
> `push_moved`, `budget_spent` (§ 9).
>
> **Replaces the v1 design** (`bmw/rack_motion.py`: a `RackMotion` rate
> estimator plus a `BreakawayEstimator` that learned the rack's breakaway
> torque online from edge-detected angular-rate transitions), deleted this
> date — offline validation on route 3f2 refuted it before it ever shipped
> behind a toggle. See § 11.

> **2026-08-13 — entry/settle hysteresis on the `HOLD_BAND` rest band (route
> 3f4 data).** One shared threshold for both leaving rest and returning to it
> made the controller flicker across the boundary constantly on straights:
> route 3f4 (OFF leg, 20.3 min of clean engaged straight driving) measured
> the error signal's band-passed noise at **σ = 0.00081 rad** — `HOLD_BAND`
> (0.001 rad) is only **1.23σ**, well inside the noise core — so the
> controller crossed the boundary **~89 times/min** and started **~22.6**
> sub-breakaway ramp episodes/min: stepper activations that mostly commanded
> torque below rack breakaway and barely moved the wheel.
>
> **The fix: split the threshold.**
> - **Leave rest** only when `|delta_err| > HOLD_BAND_ENTER` (new, `0.0015`
>   rad — clears the noise core with margin).
> - **Return to rest** at `|delta_err| ≤ HOLD_BAND` (existing, `0.001` rad,
>   **unchanged** — a growing `lane_keeping` position correction still lands
>   exactly where it always has).
>
> New state `state['at_rest']` (`True` = holding, `False` = correcting)
> replaces the single `if abs(delta_err) <= HOLD_BAND:` comparison that used
> to gate `hold_zero`/`hold_curve` vs. the off-target branch: while resting,
> only an error past `HOLD_BAND_ENTER` leaves rest; while correcting, only an
> error at/below `HOLD_BAND` re-settles. The `cancel_tol` gate (`1.2 ×
> HOLD_BAND`, § 8) is untouched — it is boundary hygiene for an in-flight
> **push** ramp specifically, orthogonal to this rest/correcting state.
>
> **Replay: −63% activation episodes.** Plain widening of the single
> threshold was measured and rejected — only **−18%** activations, and it
> re-enters the **Phase-1 failure mode** (§ 2026-07-19 above): a
> 0.0012–0.0021 rad tolerance band ate **44%** of the `lane_keeping` anchor's
> commands on route 3bf. Splitting the threshold avoids that because the
> *settle* point never moves — only entry gets harder, so a persistent
> correction is never absorbed once its error is real.
>
> **Open-loop caveat**: this replay validates the *detector* — that
> classifying deltas against `[HOLD_BAND, HOLD_BAND_ENTER]` cuts flicker —
> not the closed-loop feel of the wider entry gate. On-car verification is
> still pending; straight-line smoothness is the veto (see A/B guidance
> below).
>
> **`HoldHysteresis` param, kill-switch polarity (opposite of `AngleBudget`):
> default ON**, `'0'` rolls back to the legacy single threshold. Read once at
> construction (same rationale as `AngleBudget`'s init-time read — no
> per-tick/cached param-file re-read on the RT thread); **no bus topic, no
> heartbeat, no hot toggle** — unlike `AngleBudget`'s mid-drive push-budget
> switch, this doesn't need one: a restart-scoped kill-switch is enough,
> since controlsd re-reads the param at every drive start. With the
> kill-switch off, `HOLD_BAND_ENTER`'s effective value collapses to
> `HOLD_BAND` and the state machine reduces *decision-for-decision* to the
> legacy comparison, including the boundary case (`>` to leave, `<=` to
> return) — pinned by `test_killswitch_reproduces_legacy_single_threshold` in
> `tests/test_latcontroller.py`.
>
> **Telemetry** gains `hb_enter` (§ 9): the live leave-rest threshold in
> effect this tick (`0.0015` with hysteresis on, `0.001` kill-switched). Also
> doubles as each drive's self-label for A/B analysis — a build without this
> change carries no `hb_enter` key at all.
>
> **A/B guidance**: compare against **route 3f4, segments 37–85** as the
> baseline (same roads, same build minus this change). Flip `HoldHysteresis`
> at a landmark on a straight, same as the `AngleBudget` A/B procedure of
> the day (§ 12 — that procedure was removed with the push budget on
> 2026-08-14; `HoldHysteresis` is restart-scoped anyway, so compare whole
> drives). **Straight-line feel is the veto** — if the wider entry gate reads as
> sluggish re-centering or a wobblier rest band on-car, that overrides the
> replay numbers.

> **2026-08-14 — persistent-lean escape on the rest band (route 3f8 on-car
> verdict).** The hysteresis above went to the car and delivered its calm:
> activations **16.1 → 11.2/min**, `cancel_tol` occupancy **8.0% → 2.8%**,
> `at_rest` **61%** of straight time. But the user felt **"a bit left-hug and
> slow correction"**, and the data agreed: the slow (5 s EMA) error exceeded
> **0.002 rad on 8.2%** of straight time (**3.5%** speed-matched, vs **0.3%**
> on the baseline), **leftward-biased** and concentrated on **crowned
> stretches**. Diagnosis: a *constant* road-crown pull that lands inside the
> new entry gap `(0.001, 0.0015)` is never corrected on its own merits — it
> is corrected only when noise happens to walk it past `HOLD_BAND_ENTER`, so
> the lean latency is **unbounded**.
>
> **The fix (variant F): a second leave-rest condition on the SLOW error.**
> Entry stays at `HOLD_BAND_ENTER` (0.0015) and the settle branch is
> untouched; rest is additionally escaped when
> `|EMA(τ = HOLD_EMA_TAU) of delta_err| > HOLD_EMA_ESCAPE`.
>
> - `HOLD_EMA_TAU = 2.0` s — updated **every livePose tick** (α = 0.05/2.0 =
>   0.025), not at the decision cadence, so the lean estimate is independent
>   of how often the controller decides.
> - `HOLD_EMA_ESCAPE = 0.0012` rad, **strict `>`** — an EMA converging to a
>   constant 0.0012 bias approaches from below and never exceeds it, so that
>   value stays in the "hold" class by construction.
>
> **Why a 2 s average is safe**: symmetric flicker barely moves it (that is
> the whole point — the calm survives), while a one-sided lean accumulates.
> Replayed entry cost, entries/min: **A 13.4 · B 17.5 · F 14.8 · legacy
> 21.1** — the shipped variant F costs **+1.4/min** over doing nothing and
> still runs ~**30% under legacy**.
>
> **Latency profile** (constant bias → time to escape): 0.002 → **~1.8 s**;
> 0.0014 → **~4 s**; ≤ 0.0012 → **never, by design** — that zone *is* the
> measured noise floor (σ = 0.00081 rad), and chasing it would re-open the
> flicker the hysteresis just closed.
>
> **EMA re-prime on settle** — ~~`state['derr_ema']` is reset to the current
> `delta_err` on every `False → True` (settle) transition — only there, never
> on each resting tick (otherwise it could never accumulate while resting).
> Without it, a converged lean reading would survive the correction it caused
> and instantly re-exit the rest state the settle just entered.~~
> **REMOVED 2026-08-15** — this is exactly what starved the escape to zero
> on-car fires; see the route 3f9 note below.
>
> **Kill-switch unchanged**: the escape term is gated on the same
> `_hold_hyst_on`, so `HoldHysteresis='0'` still yields **exact legacy
> behaviour** — it now disables the hysteresis *and* the escape
> (`test_killswitch_disables_escape`). No new param, no plugin.json change —
> this rides the existing feature and its kill-switch.
>
> **Telemetry** gains `derr_ema` (§ 9): the live slow-error EMA driving the
> escape, so a drive's lean profile is readable straight from the bus.
>
> **Open-loop caveat**: replay prices only the **cost** side (entries/min).
> The benefit — a bounded, shrinking lean — is *structural* (a constant bias
> can no longer sit uncorrected for an unbounded time), but how it *feels*,
> and whether the left-hug actually goes away, needs the next drive.
> Straight-line calm remains the veto.

> **2026-08-15 — the escape shipped INERT; the settle re-prime is removed
> (route 3f9 on-car verdict).** The mechanism above went to the car and did
> **nothing**: **zero** escape fires in **15 straight-minutes**. All **190**
> rest-exits in that window were threshold-driven (`|delta_err| >
> HOLD_BAND_ENTER`), with `|derr_ema|` at exit sitting at a p50 of just
> **0.0005** — nowhere near the 0.0012 escape point. The latency profile in
> the note above (0.002 → ~1.8 s, 0.0014 → ~4 s) described a machine that
> never ran.
>
> **Root cause — two effects compounding, neither of them the threshold:**
>
> 1. **Noise pre-empts the escape.** Instantaneous `delta_err` carries
>    σ ≈ 0.0008, so from any resting lean the *instantaneous* signal crosses
>    `HOLD_BAND_ENTER` within ~**1–2 decisions**. The escape needs seconds of
>    accumulation; it always loses that race. The entry threshold is not a
>    wall a lean sits behind — noise walks the car over it almost at once.
> 2. **The re-prime starves what little is left.** The correction runs,
>    settles, and the settle branch resets `derr_ema` to the settle-point
>    error (≤ `HOLD_BAND` = 0.001, by definition of settling). A 2-s EMA
>    re-primed that low can never climb to 0.0012 during the short rest
>    interval before noise triggers the next threshold exit. The escape was
>    structurally unreachable, not merely rare.
>
> **The fix: delete the re-prime.** The settle branch now sets `at_rest =
> True` and leaves `derr_ema` alone. Replayed on route 3f9 without the
> re-prime: the escape fires **1.54/min**, and entries go **13.3 →
> 14.7/min** (+1.4/min — the same cost the variant-F replay predicted) with
> **no churn loop**.
>
> **Why re-firing right after a settle is correct, not thrash.** The
> re-prime was defending against an EMA that "instantly re-exits the rest
> state the settle just entered". That behaviour is the escape *working*: if
> the slow error is still high after a correction settled, the road is still
> pulling — a crowned stretch needs **continuous** correction, not one
> correction and a nap. And the re-fires are **self-limiting**, because the
> correction itself drags the EMA down; once the lean is genuinely gone the
> EMA falls under 0.0012 and rest holds. The emergent behaviour is a
> **gradient**: crowned roads trend toward the legacy near-continuous
> correction (which is what those stretches always needed), while clean roads
> keep the full hysteresis calm because their EMA never gets near the escape
> point. Pinned by `test_ema_survives_settle_and_escape_fires`, which drives
> a correction to settle with a standing `derr_ema = 0.005` and asserts both
> that the EMA survives the settle and that the next tick exits rest.
>
> **Process lesson — replay the design you actually ship.** Variant F was
> replayed and priced *before* the re-prime was added; the re-prime was then
> added on a plausible-sounding safety argument and shipped **unreplayed**.
> The as-built machine was never run against a route, so a change that
> zeroed the feature's entire benefit passed review looking like a
> refinement. When a "small safety guard" is added after the numbers were
> taken, the numbers no longer describe the build — re-run them.

> **2026-08-14 — the 2-degree push budget is RETIRED (route 3f4 A/B
> verdict).** The mechanism described in the 2026-08-12 entry above failed
> its on-car A/B and has been **deleted entirely** — constant, param, bus
> topic, panel toggle, telemetry keys and tests. What the A/B showed:
>
> - With the budget ON it clamped **~10 episodes/min** at ordinary-steering
>   torque (median torque at first spend **1.09 Nm** — nowhere near the
>   2.0–2.75 Nm rack breakaway it was meant to follow).
> - Matched-curve tracking error rose **+44%**.
> - The per-decision *displacement* distributions of normal steering and of
>   the route 3f2 seg 10 lurch **overlap**, so no `BUDGET_DEG` value can be
>   both invisible during ordinary driving and protective during a windup
>   release. The premise — that degrees-moved-per-push separates the two —
>   is refuted by the data, which is why this is a deletion and not a
>   retune.
>
> **What was deleted**: `BUDGET_DEG`, the `AngleBudget` param (plugin.json
> and the init-time read), the `angle_budget` plugin-bus topic and its
> livePose-rate subscriber poll, the "Steering Push Budget" Driving-panel
> toggle plus `_set_angle_budget`/`_publish_angle_budget`, the
> `ui.state_tick` heartbeat hook (`on_ui_state_tick`) that existed only to
> carry that toggle — hook entry included — the `push_ref`/`push_moved`/
> `budget_spent`/`budget_on`/`budget_sub` state, the asymmetric clamp branch
> (`STEP_MAX` is symmetric again for every decision, as before 2026-08-12),
> the per-decision `push_ref` re-arm, the `push_moved`/`budget_spent`/
> `budget_on` telemetry keys, and `tests/replay_push_budget.py`.
>
> **What was kept**: `_angle = float(getattr(CS, 'steeringAngleDeg', 0.0))`
> in `update()` — one harmless line, retained because the angle stream is
> the only usable rack-motion sensor on this car (`angRate` reads 0 through
> the entire route 3f2 creep phase) and the successor work will need it.
> **Deltas only**: the signal carries a constant ~−1.58° physical alignment
> offset.
>
> **The open problem is NOT closed.** Route 3f2 seg 10 (6 s stall, hold-floor
> ratchet, breakaway into a 24.8° swing) is still unmitigated; the successor
> design — arming on stall *state* rather than displacement — went back for
> revision on this date after offline replay refuted its arming predicate
> (during the frozen phase the stall inflates `delta_err`, so the pure-P
> base law tracks the wound-up torque within ±0.02 frac and no "surplus"
> appears). Do not re-propose a displacement budget; do not assume the
> replacement exists.

> **2026-08-15 — stall/breakaway v2: the displacement trip (`StallBreakaway`,
> default OFF).** The successor to the retired push budget, and the answer to
> the open problem the note above leaves standing. Replay-validated on routes
> 3f2 (7 segs) and 3f4 (86 segs, 85 min); design record in
> `.superpowers/sdd/2026-08-12-angle-based-breakaway/`.
>
> **v1's refutation, in one line:** every κ-derived signal *lags the rack by
> the vehicle response*, so at release the "torque above base-P" surplus v1
> armed on does not exist — the stall inflates `delta_err`, which inflates the
> base law in lockstep with the windup (measured ±0.02 frac through the whole
> frozen phase of 3f2 seg 10). Do not re-propose any arming predicate built on
> a κ-space error.
>
> **The absolute-target attempt is also a dead end (do not retry).** An
> intermediate design compared the wheel against the angle `κ_des` implies.
> It needs the *vehicle model's* conversion, not a kinematic one — the
> kinematic bicycle form understates the required steering angle by **+32% at
> 20 m/s and +49% at 25** (slip factor `sf = −7.88e−4`), so a perfectly
> tracking wheel reads 3–5° "past" a kinematic target in any curve at speed.
> **User ruling: no upstream vehicle model here — too many unknown
> parameters.** It also dragged in an online steering-angle-offset estimator,
> a readiness gate, and an absolute angle-polarity assumption, all of which
> are now gone.
>
> **The v2 principle.** *The stall context already proves the windup exists.*
> Arming requires a rack frozen while the controller is pushing, so by the
> time it breaks free there is nothing left to establish about whether torque
> has wound up — and (user) the **tire-slip term is already balanced by the
> stall condition**, which is why no absolute comparison is needed. The
> release signature is then **displacement since breakaway + sweep rate**,
> both measured against the wheel's own breakaway state.
>
> **No absolute convention anywhere — this is structural, not a comment.**
> Every quantity is a *difference* of `steeringAngleDeg` samples, so the
> constant ~−1.58° alignment offset cancels out and never has to be known or
> estimated; and `sb_dir` is the **observed** motion direction over the
> breakaway window, not a sign convention. The angle-polarity failure mode
> the earlier review had to characterize and argue away is therefore
> **eliminated by construction**: there is no polarity claim left in this
> machine to be wrong about.
>
> **The machine** (100 Hz, in `update()` right after the `_angle` capture,
> before the livePose branch):
>
> | state | entry | notes |
> |---|---|---|
> | 0 idle | — | |
> | 1 armed (stall) | `action == 'ramp'` ∧ the `SB_FROZEN_TICKS`+1 angle ring spans `< 2·ANGLE_QUANTUM_DEG` | common and harmless — **926 arms / 85 min** on 3f4; arming alone does nothing |
> | 2 episode | while armed, `≥ SB_MOVE_QUANTA` of advance inside `SB_MOVE_TICKS`; latches `sb_brk_angle = ring[-1]` and `sb_dir = ±1` (observed motion direction) | ends after `SB_EPISODE_TICKS`, or early on `hold_zero`/`hold_curve` |
> | **TRIP** | in state 2, **all four**: not `relax_dwell`; `\|κ_des\| ≤ SB_TRIP_KAPPA_MAX`; `(angle − sb_brk_angle)·sb_dir ≥ SB_TRIP_DISP_DEG`; wheel rate over the 0.2 s window `≥ SB_TRIP_RATE_DPS` | one trip per episode; re-arming needs a fresh 0.4 s freeze |
>
> **Constants and their provenance** (module scope, `bmw/latcontroller.py`):
>
> | constant | value | provenance |
> |---|---:|---|
> | `ANGLE_QUANTUM_DEG` | 0.04395 | confirmed `steerAngleDeg` LSB (survives both retired mechanisms — see §11) |
> | `SB_FROZEN_TICKS` | 40 | 0.4 s @ 100 Hz; ring is 41 long, span `< 2` quanta = frozen |
> | `SB_MOVE_TICKS` | 20 | 0.2 s breakaway + rate window |
> | `SB_MOVE_QUANTA` | 3 | ordinary creep is 1–2 quanta per window |
> | `SB_EPISODE_TICKS` | 200 | 2 s max episode |
> | `SB_TRIP_DISP_DEG` | **2.0** | **user-specified**: "additional 2 degrees of steering wheel motion" past the point the rack broke free |
> | `SB_TRIP_RATE_DPS` | **30.0** | rate sweep on 3f4: `≥25` leaves **0.31 trips/min**, `≥30` leaves **0.153**, and adding the deep-curve gate brings it to **0.071/min**. Ordinary post-stick corrections cluster below 30 °/s; the 3f2 release sweeps **31–70** |
> | `SB_TRIP_KAPPA_MAX` | **0.010** | the same value as the deep-curve doctrine threshold (`RELAX_DWELL_KAPPA`). Above it, SAT is strong enough to **self-arrest** a release — measured: hairpin segments tracked fine and *dominated* the false trips before this gate. The 3f2 lurch was a **mild** curve (`κ_des −0.0035`) where SAT could not arrest it |
> | `SB_SHED_FRAMES` | 10 | drain to 0 over 100 ms |
> | `SB_BLOCK_TICKS` | 50 | 0.5 s same-side push suppression |
>
> **Replay results.** 3f2 seg 10 trips **once, at t = 664.20, crossing at
> 31 °/s, 0.30 s before the 24.8° peak**; every other 3f2 segment trips
> **zero** times. 3f4 residue: **0.071 trips/min** (one per ~14 min) across
> 85 min of ordinary driving.
>
> **The trip action.** `target_frac → 0`, `ramp_step = −torque/SB_SHED_FRAMES`,
> `ramp_frames = SB_SHED_FRAMES` — an unthrottled drain to zero. Jerk-safe by
> construction: reducing same-side torque while the wheel is already moving
> that way cannot lurch. Then `sb_block = SB_BLOCK_TICKS` **suppresses
> same-side pushes only** (`target_frac · (−sb_dir) > 0 → 0`, applied at the
> very end of the ramp branch, after the step cap, **and only when the
> decision is a `ramp`**) so the stale-error P law cannot immediately rebuild
> the surplus. **Opposite-side correction passes through untouched** — this
> must never block counter-steer. The controller may stop pushing; it never
> gives up correcting (the standing SAFETY ARCHITECTURE law). The
> `action == 'ramp'` gate is part of that: by the time the block runs, the
> relax-dwell bridge may have replaced `target_frac` with a *hold* of the
> current torque, and zeroing that is precisely the give-up-mid-turn failure
> mode the dwell exists to prevent.
>
> **Shed-rate enforcement.** A cadence decision landing inside the shed window
> recomputes `ramp_step` over `spread_frames` (~40 frames), stretching the
> 100 ms drain to ~400 ms — a **6× slowdown of the one action the trip exists
> to perform**, and the 3f2 event peaked **0.30 s after** the trip point, so
> drain *rate* is the whole point. So while `sb_block` is up and the standing
> torque is still a same-side push, the per-tick machine re-asserts the fast
> drain whenever the in-flight ramp is not draining toward zero at least at
> `|torque|/SB_SHED_FRAMES` per frame. It only ever *reduces* same-side
> torque; an opposite-side counter-steer ramp already passes the drain test
> and is never touched. This is also what makes the machine's placement
> *before* the livePose branch safe.
>
> **Why the dwell gate stays**: `relax_dwell` exists precisely to bridge
> modelV2's mid-turn `κ_des` dips (route 3a0 seg 8), and a dip is exactly the
> condition under which a trip would be reading the reference rather than the
> rack. It costs nothing and it is certified doctrine.
>
> **Param polarity.** `StallBreakaway`, **default OFF**, `'1'` enables. Read
> once at controller construction, so it applies at the **next drive start**
> (controlsd is onroad-only); no bus topic, no heartbeat, no hot toggle and
> **no panel toggle** — a rare-event safety net does not need mid-drive
> flipping, and the A/B is "did a windup release get caught", read from
> telemetry rather than felt. There is no warm-up: with the param on, the
> machine can arm from the first engaged tick.
>
> **Verification plan for the first enabled drive** (telemetry, §9):
> `sb_state` excursions to 1 should be routine and harmless; `sb_trips` should
> stay ≈ 0 on an ordinary drive (budget: **~0.071/min**, i.e. about one per
> 14 min); and a trip should appear **only** on a genuine windup release —
> cross-check each one against the angle trace for a fast sweep past the
> breakaway point in a mild curve. A drive that accumulates trips at
> ordinary-steering rates is a refutation, not a tuning opportunity — the same
> standard that retired the push budget.

> **2026-08-17 — driver-override observers, phase 1: four detectors, telemetry
> only.** The mission is to make openpilot **understand driver intention
> rather than fight it**. Everything above this line is about tracking the
> commanded curvature well; this is about noticing when the *driver* has a
> different opinion. Design record: `project_driver_override_design` memory
> and `.superpowers/sdd/2026-08-12-angle-based-breakaway/`.
>
> **The hard constraint: this car has no driver-torque sensor.** There is no
> `steeringTorque` to threshold — `steeringPressed` on this platform is a
> VoiceControl button OR `gasPressed`, and is *not* a hands-on signal. So the
> **rack's own measured physics is the microphone**: what the wheel does under
> a known applied torque is the only evidence a driver is on it. Each detector
> below is a statement about motion that the plant alone cannot produce.
>
> Sign conventions used throughout: **angle + = LEFT, torque − = LEFT**, so the
> command direction in angle space is `−sign(torque)`, and the curvature intent
> in the angle domain is `−sign(κ_des)`. Rate is the signed 0.2 s difference
> over the shared `sb_ring` (deltas only — the ~−1.58° alignment offset in
> `steeringAngleDeg` cancels).
>
> | detector | signature | conjuncts | sustain |
> |---|---|---|---|
> | **EOD** | panic yank | motion **against** our push at ≥ `OV_EOD_RATE_DPS` 40 °/s with \|torque\| > `OV_EOD_MIN_TQ` 0.05; **not** toward-center from beyond `OV_SAT_SAFE_DEG` 10° (SAT does that unaided); **and `sb_block == 0`** | `OV_EOD_TICKS` 10 (100 ms) |
> | **HOLD** | rigid hold | wheel **frozen** (the sb machine's own test: full ring, span < 2 quanta) at \|torque\| ≥ `OV_HOLD_TQ` **0.25 frac ≈ 3 Nm** — static friction cannot hold past the knee (user ruling) — and \|angle\| < 6° | `OV_HOLD_TICKS` 50 (0.5 s) |
> | **CRAWL** | firm resist | \|torque\| ≥ `OV_CRAWL_TQ` 0.30 with \|rate\| < `OV_CRAWL_RATE_DPS` 8 °/s and \|angle\| < `OV_CRAWL_ANG_DEG` 6° — a free rack at 0.30 *sweeps* | `OV_CRAWL_TICKS` 50 (0.5 s) |
> | **WSD** | deliberate correction | wheel deepening onto the **wrong side of the commanded intent** (`intent = −sign(κ_des)` when \|κ_des\| > `OV_WSD_INTENT_KAPPA` 0.001; on a straight, any side past a wider floor and still moving out) at ≥ `OV_WSD_RATE_DPS` 4 °/s against a **remembered** push | `OV_WSD_TICKS` 15 (150 ms) |
>
> **`OV_CRAWL_ANG_DEG` was tightened 10 → 6** on this date: SAT plus friction
> can balance 0.30 frac on their own at 8–10°, so a crawl there proves nothing.
> `HOLD` reuses the same 6° near-center bound, and it binds harder there since
> `OV_HOLD_TQ` < `OV_CRAWL_TQ`.
>
> **The push memory (`OV_PUSH_MEM_S` = 0.3 s)** exists because `cancel_tol`
> yields ~0.3 s after the driver wins — measured on 3f2 seg 22 — which masks
> the against-torque test exactly when the override is most visible. A **new
> push in either direction re-primes it**: carrying a stale direction across a
> command reversal cost **12 false WSD fires / 3.2 h** at curve entries.
>
> **The `sb_block` stand-down on EOD** is composition with stall/breakaway v2,
> not belt-and-braces. While a trip is settling, the fast motion is the *plant*
> shedding windup (a rebound), which v2 owns; validated on 3fa seg 10, and the
> single at-speed EOD fire in the clean set coincides exactly with a v2 trip.
>
> **Validation** (six routes, ~6 h, including the new low-speed LKA regime).
> Clean-at-speed fire rates per 3.2 h: **EOD 0, HOLD 0, CRAWL 2, WSD 4.** Both
> ground-truth positives are caught: **3f2 seg 22** (driver resisting) fires
> CRAWL + WSD; **3f2 seg 10** (takeover) fires EOD + WSD. **Low-speed
> maneuvering (3–6 m/s, LKA regime) fires EOD/WSD on genuine driver steering —
> expected and wanted**: those are real overrides, and phase 2 needs them in
> the dataset.
>
> **Phase-1 semantics — read this before touching the block.** The detectors
> are **pure observers: they count and publish, and act on nothing.** There is
> **no param** (they are always-on; there is nothing to gate because there is
> nothing to do) and **no behavior change of any kind**: the block writes
> `ov_*` state keys and nothing else, and **no `ov_*` key is read anywhere in
> the actuator path**. Actuation is byte-identical with and without it — pinned
> by `TestOverrideObservers.test_no_behavior_change`, which steps two
> controllers through the same choreography with one's `ov_*` state pre-poked
> and requires exact equality of every non-`ov_` state key, the returned
> output, and every non-`ov_` telemetry field on every tick.
>
> Placement: after the sb machine (EOD reads `sb_block`, which that machine
> both arms and decrements) and before the livePose branch, so `state['torque']`
> and `state['desired']` are the values that were actually standing while the
> ring recorded the motion being judged. The block **reuses `sb_ring`
> read-only** — do not add a second ring. On disengage the sustain runs and the
> push memory reset with the ring; the **cumulative counts survive** (they are
> the drive's record).
>
> **Reading the telemetry** (§9): `ov_eod` / `ov_hold` / `ov_crawl` / `ov_wsd`
> are cumulative fires this drive, `ov_last` is the most recent detector
> (`1`=eod `2`=hold `3`=crawl `4`=wsd, `0`=none), and `ov_brake_fires` counts
> fires that landed while `brakePressed` — a fire on the brake is a different
> event from one on a trailing throttle, and that is the context phase 2 needs.
> Compare an at-speed drive against the budgets above; a clean highway hour
> that accumulates EOD or HOLD fires is a refutation of the constants, not a
> tuning opportunity. **Four caveats, all of them interpretation rather than
> defect:**
>
> 1. **`ov_eod` is only meaningful alongside `sb_on`.** The `sb_block`
>    stand-down is *dead code on a v2-off drive*: `sb_block` is only ever armed
>    inside `if _stall_v2_on`, so with `StallBreakaway` off nothing claims the
>    rebound class and **plant windup releases will be COUNTED as EOD fires**.
>    The `EOD 0 / 3.2 h` budget above was measured with v2 effectively claiming
>    them. On this car v2 *is* armed, so the budget applies as stated — but an
>    `sb_on: false` drive is not comparable, and an EOD count from one is not
>    evidence about drivers. Phase 2 revisits whether EOD needs a v2-independent
>    rebound test.
> 2. **The counts are not summable.** The detectors are independent hypotheses,
>    not a partition, and one real event can increment two counters. The exact
>    relation is **`HOLD ∧ (|torque| ≥ 0.30) ⊆ CRAWL`**: a frozen ring bounds
>    the measured rate at `2 quanta / 0.2 s = 0.44 °/s`, far inside
>    `OV_CRAWL_RATE_DPS`, and both share the same 6° bound — so a frozen wheel
>    at supra-`OV_CRAWL_TQ` torque satisfies CRAWL as well. It is **not**
>    containment in general: frozen at `0.25 ≤ |torque| < 0.30` fires **HOLD
>    alone**, which is exactly the band HOLD exists for. Report the four counts
>    separately; "total overrides" is not a number this telemetry produces.
> 3. **`ov_last` is last-*evaluated*, not last-*started*.** The detectors run in
>    block order EOD → HOLD → CRAWL → WSD and each fire overwrites the field, so
>    on a tick where several fire together **CRAWL masks HOLD and WSD masks
>    everything**. Use it as a coarse "what kind of event ended this drive"
>    hint, never as an event sequence.
> 4. **`ov_brake_fires` counts fires, not ticks.** Two detectors firing on the
>    same braked tick add **two**, so it can exceed the number of distinct
>    braked events. It is a context weight for phase 2, not an event count.
>
> **Phase 2 is NOT in this change**: yield tiers, the brake short-circuit, and
> integration with the LKA events are all designed in the memory and
> deliberately unbuilt until the phase-1 counters have been read from real
> drives.
---

> **2026-10-20 — curve feed-forward; hold machinery removed (route 4ec segs 22/23).**
> Route 4ec understeered in gentle bends (seg 22: the car drifted 0.8 m to the
> outside of a 1 m/s² bend while P carried a standing −24e-4 rad error) and
> carried a stale left slope bias into the next right bend. 440 steady windows
> on 4e9–4ec showed curves need steady torque, ~1.5 Nm per m/s² beyond ~0.25,
> that P alone paid for with a standing error. `ff_nm = clip(1.0 · (|a_ref| −
> 0.3), ±2 Nm)` on `a_ref = v_true²·κ_ref` now supplies ~55 % of it (replay:
> ff/need median 0.52, over-supply p99 0.39 Nm); P closes the rest. The curve
> hold (`hold_f`, `hold_cap`, hold-floor, R8 trim) went with it (D2 of the
> 2026-10-20 design): the feed-forward replaces the hold's level, not its
> ratchet, so a P below the standing torque now lowers it — watch for 0.3–1 Hz
> torque breathing in steady 0.5–1.5 m/s² bends. **Relax-dwell stays**: the
> design assumed the feed-forward would bridge mid-hairpin reference dips, but
> in a hairpin it sits at its cap while P drops by κ_scale·v²·L·Δκ; on 4e9–4ec
> the dwell bridged nine such dips holding 1.9–3.8 Nm (final review). Its
> bridge is now capped at `t_cap` instead of the deleted `hold_cap`. Slope
> learning stops above 0.15 m/s² of `a_ref`. Straights are unchanged (replay
> against 2df1b63 bit-identical). Rollback: `CF_GAIN = 0` is a law no drive has
> run; the driven baseline is `dev` @ 2df1b63.

---

> **2026-10-21 — straight gain 1.0 → 1.25 (trial, route 4ed).** Measured near-centre
> rack stiction: breakaway from rest at a median 0.45 Nm (p10 0.18, p90 0.93; 1,973
> events on 4e9–4ed, the same at 31–126 km/h); 1.0 Nm at 5–20° and 2.3 Nm in turns
> ≥ 20° (the older "2–2.75 Nm knee" is the turn value). In gentle bends P had to
> reach ~0.45 Nm before the wheel moved — a curvature error of ~4e-4 at 72 km/h —
> and the car drifted 0.3–0.7 m to the outside (4ed's left hug in right bends,
> with the slope bias cancelling most of P). The error needed scales with 1/gain.
> A lower feed-forward deadband was replayed and rejected: steady gentle bends
> already track, and +0.1 Nm stays inside the stiction band. 1.5 rang
> near-straight on route 326 under the pre-Direct controller.

---

> **2026-10-21 — take-up torque 0.35 → 0.3 Nm (route 4ee).** The lash sweep was
> audible. Inside the lash the servo speed is near-binary in torque: it does not
> move below ~0.15 Nm, sometimes creeps at 0.15–0.2, and from 0.2 Nm up to 1 Nm
> runs at ~200–235°/s, the motor's voltage-limited top speed (k_bemf 2561 mV/(rev/s),
> ratio 7.62). 0.3 keeps a margin over the ~0.2 Nm detent; the sweep is only
> slightly slower (~208 vs ~235°/s). The same constant is the reversal-size gate,
> so 0.30–0.35 Nm reversals now take up too. Take-ups are ~1/4 of all lash
> crossings (8.2/min on 4ee, 5.5 on 4ed); ramp reversals cross at the same speed.
> The lash keeps widening: 13.7° on 4e9 → 17.4° on 4ee.

> **2026-10-22 — take-up torque back to 0.35 Nm (route 4ef).** 0.3 Nm was not
> audibly different (user) and not measurably slower: p50 206°/s at 0.29–0.31 Nm,
> the same top speed. It was less reliable: the servo moved on 86 % of mid-lash
> ticks (100 % at 0.34–0.36), take-ups over 100 ms rose 15 → 25 %, and three timed
> out stalled 1.5–2° short of the right contact (none on 4ee). The torque does not
> set the sweep speed, so it is not the noise lever; a firmware speed cap in torque
> mode is. Lash 17.1 → 18.0° within the drive. Also on 4ef: `StallBreakaway` was
> switched on mid-drive, which does nothing until the next controlsd start (the
> param is read once at init) — `sb_trips` stayed 0 and the shadow detector found
> no would-be trips after the toggle; the quieter stretch that followed was the
> eastbound highway, calmer in 4ee too.

---

## 1. Why a custom controller (not stock latcontrol_torque)

> Superseded by **Current law** at the top of this document (2026-10-20); kept as history.

The BMW E9x/E8x uses an **AC Delco / Ocelot stepper servo on the hydraulic rack**, not an EPS motor. The plant has two properties that make stock torque controllers misbehave:

1. **High static stiction, no self-centering** — at zero applied torque the rack *holds its current angle* indefinitely. There is no aligning-torque-driven return-to-center. This is the opposite of most EPS racks, where the steering self-centers.
2. **Kinetic friction comparable to typical control torques** — small commands (sub-FRICTION) don't move the rack at all. The rack ignores them or moves in fits.

Consequence: traditional **angle/curvature-PID with feedforward on κ_des** is wrong here. FF for κ_des is the correct pattern when the plant *needs* sustained torque to maintain an angle (any EPS); it's wrong when stiction maintains the angle for free.

The custom controller is a **plant-inversion design in front-wheel-angle (δ) space**: compute the torque that would move δ by `δ_err` over the model's action horizon, ramp to it, then let stiction hold.

```
δ_des  = atan(κ_des · L)        L = CP.wheelbase (BMW E90: 2.76 m)
δ_meas = atan(κ_meas · L)        κ_meas = yawRate / v_ego
δ_err  = δ_des − δ_meas
target_nm = T_CAP_SLOPE_BASE · kappa_scale(|κ_des|) · v² · δ_err   # full error, no subtraction (Phase 2)
```

When `|δ_err| ≤ HOLD_BAND` the cadence decision holds (target → 0 on straight, or holds the last target on a curve) and stiction keeps the rack in place. Outside `HOLD_BAND`, the controller ramps to `target_nm` over `spread_frames` CAN ticks. When the plant runs past κ_des, `δ_err` flips sign and `target_nm` reverses on its own — tracking back to the command *is* the overshoot correction. (The old ISO comfort-guard drain-to-0 was removed 2026-07-28; a_y is bounded by speedlimitd — see the SAFETY ARCHITECTURE note at the top and §7.)

---

## 2. Single-knob timing design

> Superseded by **Current law** at the top of this document (2026-10-20); kept as history.

There is one knob — `CP.steerActuatorDelay` in `bmw/interface.py:87` — and every timing constant downstream subscribes to it via the liveDelay chain. Change SAD and the whole stack adapts.

```
CP.steerActuatorDelay              (bmw/interface.py — THE knob; currently 0.4)
        │
        ▼  + 0.2 (lagd.py:181 initial_lag = SAD + 0.2)
liveDelay.lateralDelay             (= 0.6 s; permanently pinned for BMW —
                                    lagd never converges because it correlates
                                    on latcontrol_torque telemetry we don't
                                    produce; status stays 'unestimated')
        │
        ├──▶ modeld lat_action_t = lat_delay + DT_MDL = 0.65 s
        │       └──▶ get_curvature_from_plan samples κ_des on the predicted
        │            trajectory at this horizon (modeld.py:391)
        │
        └──▶ controlsd lat_delay = liveDelay + LAT_SMOOTH_SECONDS = 0.6 s
                                    (LAT_SMOOTH_SECONDS = 0.0 on catpilot)
                │
                ▼  passed to LaC.update(..., lat_delay)
        bmw/latcontroller.py update() per livePose tick:
                model_action_t = lat_delay + DT_MDL = 0.65 s   ≡ modeld's lat_action_t
                half_horizon   = model_action_t / 2 = 0.325 s
                action_cadence_ticks = round(half_horizon / DT_LIVEPOSE)  = 6  (300 ms)
                spread_frames        = action_cadence_ticks × 5            = 30 (300 ms)
                jerk_pred horizon    = model_action_t                      = 0.65 s
```

**Concrete values at SAD = 0.4 (current):**

| | value |
|---|---:|
| `liveDelay.lateralDelay` | 0.6 s |
| `modeld lat_action_t` | 0.65 s |
| `model_action_t` (in update) | 0.65 s |
| `action_cadence_ticks` | 6 (= 300 ms decision period) |
| `spread_frames` | 30 (= 300 ms ramp, exactly = cadence) |
| `jerk_pred` horizon | 0.65 s |

**Plant-settling check**: BMW hydraulic rack τ ≈ 130 ms → 300 ms cadence ≈ 2.3τ → ~90% step response between decisions. Margin preserved.

**Why `spread_frames = action_cadence_ticks × 5`** rather than independently rounded: `DT_LIVEPOSE = 50 ms = 5 × DT_CAN_TICK (10 ms)`. Independent rounding can diverge by one tick (e.g., 6×50 = 300 ms vs 32×10 = 320 ms). Deriving spread from cadence enforces exact equality — ramp completes precisely when next decision lands.

**SAD retuning range**: `[0.3, 0.5]` keeps cadence in `[5, 7]` ticks (250–350 ms), all ≥ 2τ for the BMW rack. Below 0.3 risks plant-settling adequacy; above 0.5 just adds curve-entry latency without further wobble benefit.

---

## 3. Reference signal & filter chain

> Superseded by **Current law** at the top of this document (2026-10-20); kept as history.

```
modelV2.action.desiredCurvature   (raw model output, ~20 Hz)
        │
        ▼  hooks.run('controls.curvature_correction', ...)
        │     (passthrough — no curvature corrections registered in the public build)
        ▼
clip_curvature() in drive_helpers.py
        │  — rate-limited to MAX_LATERAL_JERK = 5.0 m/s³ (ISO)
        │  — clamped to ±MAX_LATERAL_ACCEL_NO_ROLL = 3.0 m/s² (ISO, + roll comp)
        │  — clamped to ±MAX_CURVATURE = 0.2 1/m
        ▼
self.LaC.update(active, CS, VM, params, steer_limited_by_safety,
                desired_curvature, curvature_limited, lat_delay)
        ▼
=== bmw/latcontroller.py update() ===
state['desired'] = float(desired_curvature)   # raw, NO filter on κ_des itself
v               = float(lp.velocityDevice.x), floored at 8.5 m/s
state['measured'] = float(lp.angularVelocityDevice.z) / v
delta_des       = atan(state['desired'] · L)
delta_meas      = atan(state['measured'] · L)
delta_err_raw   = delta_des - delta_meas
        │
        ▼  Phase 2 (2026-07-22): NO filter on delta_err either.
        │  delta_err = delta_err_raw, unconditionally — the κ-gated box
        │  filter that used to live here (§4) is deleted. modelV2 noise is
        │  handled upstream by the `lane_keeping` plugin's κ_des low-pass
        │  (KAPPA_FILTER_TAU = 0.15 s) before this controller ever sees it.
        ▼
state['delta_err'] = delta_err                 # raw, used by controller as-is
        │
        ▼  a_y_meas / jerk_pred computed here — TELEMETRY ONLY since 2026-07-28.
        │  (The ISO accel/jerk cancel guard that used to read them and drain
        │   target_frac to 0 was REMOVED — a_y is bounded by speedlimitd; see
        │   the SAFETY ARCHITECTURE note at the top and §7. cancel_tol, the
        │   HOLD_BAND boundary-hygiene drain, survives and runs just below.)
        ▼
Cadence decision (every action_cadence_ticks):
   if |delta_err| ≤ HOLD_BAND:
      hold_zero (target_frac = 0, straight)   OR   hold_curve (target_frac = held_target, curve)
   else:
      kappa_scale = interp(|κ_des|, T_CAP_SCALE_KAPPA, T_CAP_SCALE_BP)
      target_nm = T_CAP_SLOPE_BASE · kappa_scale · v² · delta_err   # FULL error, no subtraction
      target_frac = target_nm / STEER_MAX
      cap to ±t_cap_frac; hold-floor keeps |target_frac| ≥ |held_target| when same-signed
      action = 'ramp'
        ▼
Per-CAN-tick ramp: state['torque'] += ramp_step toward state['target_frac']
        ▼
return -state['torque']  (BMW carcontroller flips sign convention)
```

---

## 4. The κ-gated box filter on delta_err (DELETED 2026-07-22)

**DELETED 2026-07-22 (Phase 2).** The filter described below no longer exists
in `bmw/latcontroller.py` — `delta_err` is the raw `delta_err_raw` at all
times, `KD_GATE` and `kd_filter_window` are gone, and there is no `de_buffer`.
Its job (suppressing modelV2 κ_des wobble on near-straight) now belongs to
the `lane_keeping` plugin's κ_des low-pass (`KAPPA_FILTER_TAU = 0.15 s`),
which filters upstream of this controller instead of on `delta_err` inside
it. The rest of this section is kept for historical context only — see the
2026-07-22 header note.

**Purpose (historical)**: suppress high-rate sign-flips in `delta_err` caused by vision-only κ_des wobble on near-straight (no position-feedback layer running). Each sign-flip across the tolerance band triggers `cancel_tol` / `brake_zero` with a counter-direction FRICTION pulse — those are the felt "swaying" pulses.

**Target choice**: filter applied to `delta_err`, **not** to raw κ_des. Keeps `state['desired']` raw so:
- Reference is always fresh — no held bias → no drift accumulation
- `kappa_scale`, `jerk_pred`, ISO guards all see raw κ_des magnitude (true model intent)
- Only the cadence decision's tolerance check sees the smoothed error

**Gate**: `KD_GATE = 0.002` 1/m. Filter active only when `|raw κ_des| < KD_GATE`. Above gate (real curves) → bypass, buffer flushed. Also bypassed during `modelV2.meta.laneChangeState != off`.

**Window**: `kd_filter_window = action_cadence_ticks = 6` samples (300 ms box, 150 ms group delay).

**Output**: equal-weight uniform box average.

```python
raw_kd = float(desired_curvature)
lane_change_active = (_sm['modelV2'].meta.laneChangeState != log.LaneChangeState.off)
if abs(raw_kd) >= KD_GATE or lane_change_active:
    state['de_buffer'].clear()
    delta_err = delta_err_raw
else:
    state['de_buffer'].append(delta_err_raw)
    if len(state['de_buffer']) > action_cadence_ticks:
        state['de_buffer'].pop(0)
    delta_err = sum(state['de_buffer']) / len(state['de_buffer'])
state['delta_err']     = delta_err
state['delta_err_raw'] = delta_err_raw    # both published in telemetry
```

**Design rule (the filter exists for ONE job)**: suppress sign-flips on near-straight. Never compromise real correction. Hence gate at 0.002 (below typical wobble peak amplitude ±0.003) — the filter operates only at zero-crossing windows where sign-flips actually happen; wobble peaks pass through raw.

**Field-measured sign-flip reduction**: ~60% on highway-dominant routes.

---

## 5. The kinematic deadzone

> Superseded by **Current law** at the top of this document (2026-10-20); kept as history.

**DELETED 2026-07-22 (Phase 2).** The kinematic `DRIFT_M` deadzone, the
σ-observer noise floor and the sign-persistence gate are gone. They existed
because the controller was the only loop closing against drift; that job now
belongs to the `lane_keeping` position loop, which also owns modelV2 noise.

What remains is `HOLD_BAND = 0.001` rad — **not** a deadzone. P acts on the full
`delta_err` at all times; `HOLD_BAND` only decides when the rack is "on target"
so the curvature hold (§ hold_curve) can take over. It is sized by stiction:
below `FRICTION·STEER_MAX / (T_CAP_SLOPE·kappa_scale·v²)` ≈ 0.001 rad at 25 m/s
the P term commands less than rack breakaway, so the wheel cannot move anyway.

Historical note: a constant-angle deadzone was tried in 2026-05 and reverted
(route 31b, 1.3–1.7 m drift) precisely because no position-feedback layer
existed. That constraint is now satisfied — but note the resolution was to
delete the deadzone, not to widen it.

---

## 6. T_CAP_SCALE — curvature-dependent gain

```python
T_CAP_BASE_NM     = 2.0    # stiction floor (Nm)
T_CAP_SLOPE_BASE  = 1.0    # base aligning-torque gain
T_CAP_SCALE_KAPPA = [0.001, 0.01,  0.02]   # |κ_des| breakpoints (1/m)
T_CAP_SCALE_BP    = [1.0,   2.5,   3.0]    # multiplicative scale on T_CAP_SLOPE_BASE

kappa_scale = np.interp(|κ_des|, T_CAP_SCALE_KAPPA, T_CAP_SCALE_BP)
target_nm   = T_CAP_SLOPE_BASE · kappa_scale · v² · delta_err   # full error, Phase 2
t_cap_nm    = T_CAP_BASE_NM + T_CAP_SLOPE_BASE · kappa_scale · v² · |δ_des|  (≤ STEER_MAX=12)
```

**Design rule**: curvature awareness must enter as a **multiplicative gain scale** on `T_CAP_SLOPE_BASE`, NOT as a feedforward additive term on κ_des. The stiction-holds-at-zero plant doesn't need continuous holding torque; an additive FF would over-push and induce stiction-locked overshoots.

**Tuning history of `T_CAP_SCALE_BP[0]` (small-κ gain)**:
- Originally 1.0 (matches T_CAP_SLOPE_BASE — neutral baseline)
- Bumped to 1.5 for more aggressive small-κ tracking
- **Returned to 1.0** (commit bba4454, 2026-05-22): the 1.5 caused over-correction in the small-κ regime → overshoots → cancel_jerk pulses. Route 326: cancel_jerk per lane change dropped from 25 to 5.3 with no compromise to lane offset.

Higher-κ scale points (2.5 @ 0.01, 3.0 @ 0.02) unchanged so tight-curve tracking authority is preserved.

---

## 7. ISO 11270 comfort guards — HISTORICAL (removed 2026-07-28)

> **This entire section is historical.** The in-controller ISO accel/jerk
> cancel guard was removed on 2026-07-28 — see the SAFETY ARCHITECTURE note at
> the top of this document. ISO 11270 lateral-acceleration limiting is now a
> **system-level** responsibility of speedlimitd (curve-speed capping of
> `vEgo`), not of the lateral controller, which tracks the commanded curvature
> unconditionally. The code, constants (`accel_guard_threshold`,
> `ACCEL_GUARD_*`, `LATERAL_CURVATURE`, `LATERAL_JERK_BP`,
> `ISO_LATERAL_ACCEL/JERK`), and the `cancel_accel`/`cancel_jerk` actions
> described below **no longer exist in `bmw/latcontroller.py`**. Kept for the
> design history and the reasoning that led to the removal.

```python
ISO_LATERAL_ACCEL = 3.0    # m/s²  (from opendbc.car.lateral)
ISO_LATERAL_JERK  = 5.0    # m/s³

LATERAL_CURVATURE = [0.001, 0.005, 0.01, 0.02]
LATERAL_JERK_BP   = [1.5,   1.5,   3.0,  ISO_LATERAL_JERK]    # half-ISO at small κ, full at tight

# ACCEL_GUARD_* — module scope (bmw/latcontroller.py, near HOLD_AY_BP)
ACCEL_GUARD_FLOOR  = 1.5    # m/s² — small-a_y tightened floor (2026-05-22, route 326 class)
ACCEL_GUARD_RATIO  = 1.25   # fire only when measured exceeds commanded by 25%...
ACCEL_GUARD_MARGIN = 0.2    # ...plus 0.2 m/s² absolute headroom

def accel_guard_threshold(a_y_des_abs):
    return min(ISO_LATERAL_ACCEL, max(ACCEL_GUARD_FLOOR,
                                       ACCEL_GUARD_RATIO * a_y_des_abs + ACCEL_GUARD_MARGIN))
```

Fire only when **the plant has actually overshot** `(κ_des - κ_meas) · κ_meas < 0` AND the relevant signal exceeds its threshold:
- **jerk** — unchanged, still the κ-indexed `LATERAL_CURVATURE` / `LATERAL_JERK_BP` table (`jerk_pred` is already error-relative, so it wasn't the thing cycling).
- **accel** (2026-07-27) — `BMW_LATERAL_ACCEL = accel_guard_threshold(v² · |κ_des|)`, referenced to **commanded** lateral accel rather than a κ-indexed table. Action on fire: drain `target_frac` to 0 (reverse-FRICTION unwind pulse removed 2026-07-03 — see §8).

**Why the accel guard moved off the κ table (2026-07-27, route 3ce)**: curves whose commanded a_y sat within ~5% of the old κ-interpolated threshold were getting `cancel_accel`'d by normal ±15–20% measurement tracking noise on the overshoot side — a drain-rebuild hunting cycle at 54% zero-torque. The old table encoded a fixed curvature→threshold mapping tuned at city speed; since both SAT and achievable a_y scale with v², that mapping is outrun once v² ≥ ~210 (above ~14.5 m/s), collapsing the margin. `accel_guard_threshold()` instead keeps headroom proportional to what was actually commanded (25% ratio + 0.2 m/s² absolute margin) at any speed, floored at 1.5 m/s² (carries forward the 2026-05-22 near-straight tightening below) and capped at `ISO_LATERAL_ACCEL` (3.0) via `min()`. Reference points: route 3ce seg 26 (commanded 1.82 → threshold ≈2.475, clear of wobble reach) and seg 15 (commanded 2.34 → threshold caps at ISO 3.0).

**Bug history (worth remembering)**: `LATERAL_CURVATURE` second value was originally `0.05` (out of order), making `np.interp` non-monotonic. Effect: thresholds were stuck at the small-κ value (2.0 at the time) all the way through `|κ|=0.02`, then jumped discontinuously to ISO at the boundary. cancel_jerk / cancel_accel were firing more aggressively than designed during real moderate curves. Fixed in commit 633a146 along with the small-κ tightening (2.0 → 1.5). Verified positive on routes 32a/32d: `cancel_accel` essentially eliminated (114 on route 326 → 0/18), post-LC max torque cut from 4.3 N to 2.1 N. (This history now applies to the jerk table; the accel side carries the same 1.5 floor forward as `ACCEL_GUARD_FLOOR`.)

---

## 8. Action state machine (debug field)

`state['action']`, published in `bmw_lat_control` telemetry.

| state | when entered |
|---|---|
| `init` | controller construction |
| `ramp` | a decision ran: the torque ramps toward `target_frac` |
| `relax_dwell` | overshoot-side error in a measured deep curve, within the dwell — the target holds the current torque (within `t_cap`) |
| `take_up` | the free-play take-up owns the torque while it crosses the servo lash |
| `idle` | between decisions once a transient label's ramp completed, so telemetry occupancy counts are honest |

Stall/breakaway v2 adds no label: a trip writes `target_frac` / `ramp_step` /
`ramp_frames` directly, so the shed appears as `sb_trips` incrementing plus
`target_frac → 0`; its own state is `sb_state` (§9).

---

## 9. Telemetry (plugin_bus topic `bmw_lat_control`, 20 Hz)

Published each livePose tick. The exact field set is pinned by
`test_payload_exact_key_set`.

| field | meaning |
|---|---|
| `desired` | κ_ref the controller tracks (the lane_keeping Arc reference, 1/m) |
| `measured` | κ_meas = livePose yaw rate / v_true (1/m) |
| `err` | κ_ref − κ_meas (debug) |
| `delta_err` | front-wheel angle error (rad) — what the P law acts on |
| `target_frac` | the decision's target torque fraction (−1..1), after guard, STEP_MAX and caps |
| `ramp_step` | per-CAN-tick torque increment |
| `ramp_frames` | CAN frames remaining in the current ramp |
| `action` | `ramp` / `relax_dwell` / `take_up` / `idle` (§8) |
| `torque` | commanded torque fraction (−1..1) |
| `output` | clipped torque fraction returned to controlsd (0 while inactive) |
| `vEgo` | CS.vEgo |
| `active` | lateral active |
| `ff_nm` | curve feed-forward (Nm) |
| `a_ref` | reference lateral accel `v_true² · kappa_ref` (m/s²) |
| `relax_ticks` | consecutive overshoot-side ticks while deep in a curve (dwell counter) |
| `sb_state` | stall/breakaway v2: `0` idle, `1` armed, `2` breakaway episode |
| `sb_trips` | cumulative trips this drive (budget on ordinary driving ≈ 0) |
| `sb_block` | same-side push-suppression ticks left after a trip |
| `angle` | steering wheel angle (deg, + = LEFT) |
| `servo_gap` | column − servo angle (deg): slack / windup |
| `steer_rate` | steering rate (deg/s) |
| `pressed` | steeringPressed |
| `lat_delay` | as given by controlsd (s) |
| `tu_state` | `right` / `left` (seated) / `taking_up` / `unknown` |
| `c_right`, `c_left` | learned contacts (deg); 0.0 = not known yet this drive |
| `tu_fires`, `tu_ms`, `tu_timeouts` | cumulative take-ups, last duration, cumulative timeouts |
| `tu_guarded` | cumulative reversals held at 0 by the noise guard |
| `sl_bias` | learned straight-road torque (Nm) |
| `sl_learn` | slope-bias learn gate open this tick |

Multiply `output` or `torque` by `STEER_MAX = 12 Nm` for Nm.

---

## 10. Constraints baked into the design

- **Vision-only stack**: catpilot has no HD-map, lidar, radar fusion, or IMU-fusion for κ. All correction must come from `modelV2` outputs. Don't propose external-sensor escape hatches for vision-noise problems.
- **Reference conditioning**: `lane_keeping` is registered on `controls.curvature_correction` — it hands this controller an arc through the model's planned path, low-passed. This `latcontroller` is a faithful tracker with no noise reasoning of its own; the controller requires the plugin (R10).
- **lagd never converges for BMW**: `liveDelay.lateralDelay` is permanently pinned at `SAD + 0.2` because lagd correlates on latcontrol_torque telemetry our controller doesn't produce. `SAD` is the effective knob — not a hint to be overridden.
- **BMW hydraulic rack**: stiction holds δ at zero torque near centre, no self-centering there. Below ~0.3 m/s² of lateral acceleration no steady torque is needed; above it curves need steady torque (routes 4e9–4ec, 440 steady windows: ~1.5 Nm per m/s² beyond ~0.25) — see the 2026-10-20 curve-feed-forward design.

---

## 11. Failed experiments (do not re-litigate)

These were tried, deployed, and reverted. Each appears here so future maintainers know what's been ruled out and why.

| experiment | result | file:line | reverted by |
|---|---|---|---|
| Constant-angle deadzone (`TOL_DEG_CONST = 0.35°`) | Allowed 1.3–1.7 m drift at 85 kph with no position-feedback layer running; controller silent 98% of samples (route 31b seg 8/15). Wide angular deadzone is unsafe without a position-feedback layer. | commit 715114d | d43fb19 |
| Stiction-gated steering-angle FF (linear in \|steer\|, 20°/0.10 Nm·deg⁻¹) | Over-pushed in route 31d seg 8 right turn: 50% over-rotation, 6.22 Nm torque, sustained cancel_jerk oscillation. SAT physics aren't linear in steering angle alone — needs multi-regime (v×κ grid) calibration, not single-route fit. | commit 2c67a97 | e81f1c0 |
| Hysteresis-on-κ_des (`KD_HYST_GAP = 0.003`, `KD_GATE = 0.006`) | 95% sign-flip reduction in offline check, but route 322 had 32% time `\|offset\|>0.5 m` and 44-second frozen-κ_des stretches — controller faithfully tracked a held biased reference → drift. Structural failure of holding the reference without a position-feedback layer. | commit daef207 | 96945fc (replaced with box-on-delta_err) |
| Suggesting external sensor fusion (HD map, radar, lidar, IMU) for vision noise | Out of project scope by design — see [vision-only constraint](§10). | (recurring; do not propose) |
| 2-degree open-loop push budget (`BUDGET_DEG`, `AngleBudget` toggle: after one push moved the wheel 2° in the commanded direction, stop increasing torque and shed it unthrottled) | Deployed default-off, A/B'd on route 3f4, **refuted**. It clamped ~10 episodes/min at ordinary-steering torque (median 1.09 Nm at first spend, far below the 2.0–2.75 Nm rack breakaway it was chasing) and raised matched-curve tracking error **+44%**. Root cause is structural, not a bad threshold: the per-decision displacement distributions of ordinary steering and of the route 3f2 seg 10 windup release **overlap**, so no `BUDGET_DEG` is both invisible and protective. Deleted entirely 2026-08-14 (constant, param, `angle_budget` bus topic, panel toggle, `ui.state_tick` heartbeat, telemetry keys, tests, `replay_push_budget.py`). Do not re-propose a displacement-based bound; whatever replaces it must discriminate on rack **state**, not degrees moved. | commit 7afd501 | 2026-08-14 (this commit) |
| Stall/breakaway **v1** — arming on "torque above the pure-P base law while the rack is frozen" | Killed by replay before any code shipped. Through the entire frozen phase of route 3f2 seg 10 the commanded torque tracks the base-P law within **±0.02 frac** (surplus peak +0.019): the stall inflates `delta_err`, which inflates the base law in lockstep with the windup. **Every κ-derived signal lags the rack by the vehicle response, so all of them are stale exactly at release** — the decisive surplus only appears mid-swing, after `κ_meas` catches up. Zero snaps on the motivating event. Replaced by the displacement trip (v2, dated note in the header), which needs no κ-space quantity at release at all: the stall context already proves the windup, so the signature is wheel travel past the breakaway point plus sweep rate. Do not re-propose any arming predicate built on a κ-space error. See `.superpowers/sdd/2026-08-12-angle-based-breakaway/stall-breakaway-v2-design.md`. | never deployed | 2026-08-14 |
| Online breakaway-torque estimator + angular-rate edge detector (`bmw/rack_motion.py`: `RackMotion`, `BreakawayEstimator`) | Never deployed — killed by offline validation on route 3f2. Rate thresholding produced 1-2 tick false-motion blips at exactly the low rates that mattered; a debounce did not fix it. The estimator's own observations had a median torque of 0.041 frac — the controller's own median push, not a breakaway signature — so it measured "torque present when the wheel happens to move" and collapsed to its clamp floor. Replaced by the fixed 2-degree open-loop push budget (no online learning, no rate threshold) — see the 2026-08-12 entry above. Its `ANGLE_LSB_DEG = 0.0879` angle quantum was also wrong, exactly 2× too coarse — inferred by counting 169 distinct observed values over one segment, when the confirmed quantum is `0.04395`° (observed level gaps are integer multiples of it, and this same file's own `MOTION_CONFIRM_TICKS` comment already used 0.04395 for the true quantum without ever reconciling the two). `BUDGET_DEG` in the push budget uses the confirmed 0.04395° value (45 quanta of headroom). (The push budget that replaced it was itself retired 2026-08-14 — next row. The 0.04395° quantum finding survives both.) | commits 5bb33b4..dd3d2a2 | 2026-08-12 (this commit) |

---

## 12. Retuning recipes (current operating point)

Current configuration is field-verified stable (2026-05-24, routes 32a/32d, user: "solid in straight, agile and accurate in turns"). Only retune if a new failure mode appears.

### To change the controller's overall timing (cadence, ramp, jerk horizon, modeld lookahead all together):
- `CONTROL_HORIZON_S` (0.65 s): the decision cadence and the ramp spread are
  each half of it. It does not follow `lat_delay`. modeld's own look-ahead is
  `CP.steerActuatorDelay` in `bmw/interface.py`, a separate knob.

### To suppress more wobble at near-straight (filter):
- The reference is conditioned in `lane_keeping`: the arc look-ahead schedule
  (`ARC_H_SPEED_V` / `ARC_H_SPEED`, pulled back by `ARC_K_AHEAD_BP`) and the
  low-pass `LaneKeepKappaFilterTau` (0.15 s). Longer removes more wander and
  adds delay or early turn-in. This controller has no near-straight knob of its
  own beyond its straight gain `T_CAP_SCALE_BP[0]`.

### If lane-change overshoot reappears on the post-LC phase:
- Look at `bmw_lat_control` log for that LC: is `de_at_end_raw > 0.3°`? Is post-LC `|torque|` > 3 N?
- The mechanism is one of: spurious cancel_jerk suppressing controller during LC (small κ, |κ_meas| < 0.001 — see seg 10 of route 326 for pattern) OR rack stiction → catch-up overshoot (see seg 42 ev2 for pattern).
- Don't apply a global LC torque cap — most LCs are fine. Targeted options exist (magnitude-gated cancel, soft post-LC re-engagement); see commit history around routes 326/32a/32d for the analysis.

### If straights feel less smooth than today:
- Compare `kappa_ref` against `kappa_in` in the `lane_keeping` topic first (the
  reference's share of the model wobble), then the noise guard's `tu_guarded`
  rate and the take-ups per minute in `bmw_lat_control`.

### If real-curve tracking feels under-aggressive:
- Increase `T_CAP_SCALE_BP[1]` or `T_CAP_SCALE_BP[2]` (currently 2.5, 3.0 at `|κ| = 0.01, 0.02`).
- **Don't** increase `T_CAP_SCALE_BP[0]` past 1.0 — that's where route 326's over-correction lived (was 1.5, now back to 1.0).

### If lateral acceleration feels too high in curves:
- This is **no longer a lateral-controller tuning problem** (2026-07-28). The lateral controller tracks the commanded curvature and does not limit `a_y`. Lower the curve-speed target in **speedlimitd** (curve-speed capping) — it owns `vEgo`, hence `a_y = v²·κ`. Recipes for the old in-controller ISO cancel guard (`LATERAL_ACCEL_BP` / `LATERAL_JERK_BP` / `BMW_LATERAL_*`) are obsolete; that machinery was removed.

### ~~To toggle `AngleBudget` (the push-budget mechanism)~~ — GONE 2026-08-14
- The push budget was **retired** (route 3f4 A/B; see the dated note in the
  header and the §11 row). There is no `AngleBudget` param, no
  `angle_budget` bus topic, no "Steering Push Budget" toggle and no
  `ui.state_tick` heartbeat hook in this plugin any more — nothing to
  toggle, nothing to write, no procedure to follow.
- A leftover `AngleBudget` param file on a device is inert; delete it at
  your convenience (`rm /data/plugins-runtime/bmw_e9x_e8x/data/AngleBudget`).
- **The hot-toggle pattern itself is still the right one** should another
  mid-drive-switchable knob appear here: panel toggle → immediate
  `PluginPub` send **plus** a 1 Hz heartbeat republishing the param file
  (ZMQ PUB drops edge-triggered sends to a not-yet-connected subscriber, so
  the heartbeat is what makes it reliable), with the controller polling at
  livePose rate — tmpfs + memory only, never a param-file read on the RT
  thread. Git history around commits fbb899b / 7afd501 has the working
  implementation.

### To toggle `StallBreakaway` (stall/breakaway v2 displacement trip, 2026-08-15 entry):
- **Default OFF**, `'1'` enables.
- **Not hot**, same as above: read once at controller construction, so it
  applies at the **next drive start**. There is deliberately no bus topic, no
  heartbeat; the Driving-panel "Stall Breakaway" toggle writes the param.
- Enable: `echo -n 1 > /data/plugins-runtime/bmw_e9x_e8x/data/StallBreakaway`.
  Disable: write anything else, or delete the file.
- **Verify a drive picked it up**: with the param on the machine arms from the
  first engaged tick, so `sb_state` excursions to 1 show up early and often.
- **The A/B is telemetry, not feel**: the question is "did a windup release
  get caught", so compare `sb_trips` (expect ≈ 0 per ordinary drive) and the
  angle traces around any trip — not straight-line smoothness.
- **Rollback**: delete the param and restart the drive.

### Slope Compensation (`SlopeCompensation`, 2026-10-19 entry):
- **Why**: Direct is P-only, so the steady torque a straight needs against a
  constant pull (road slope, alignment) comes only from a standing error. On
  route 4e9 that was δ_err ≈ −2.2e-4 rad / ~0.08 Nm left, and the car sat
  +0.15 m right of the model's plan (plan at 3 s: +0.03 m).
- **Law**: `sl_bias += (P_nm / SL_TAU_S)·dt` each livePose tick while the gate
  is open, clamped to ±`SL_BIAS_MAX_NM`; `demand = ff_nm + P_nm + sl_bias`,
  applied whether or not the gate is open.
- **Gate** (situation only, never the error's sign; see the hold-bias failure,
  37e34b7): latched at engagement, active, not pressed, v > 40 km/h,
  |κ_des| < 1e-3, |κ_meas| < 1.5e-3 and |a_ref| < 0.15 m/s² (gentle bends charged
  the bias on route 4ec), no take-up in flight, no stall-breakaway
  block or episode (`sb_state` 2; the armed state 1 does not close it).
- **Constants**: `SL_TAU_S` 20 s, `SL_BIAS_MAX_NM` 0.25 (well below rack
  breakaway, ~0.5 Nm), `SL_MIN_V` 40 km/h. Was 0.15 (≈2× the 4e9 need): on
  route 4ea it pinned the bias 20% of the time on roads with roll ≥ 2.3°,
  which need 0.20–0.25 Nm, and the car stayed +0.3 m right there.
- **Route 4ea result** (switch on from seg 12, same drive): straight offset
  +0.22 → +0.07 m, sway 0.050 → 0.047 m/s², take-ups 2.0 → 2.7/min, 0 timeouts.
  Per segment the need tracks roll (43 segments, r −0.50, ≈0.14 Nm/deg, zero
  near +1.35° roll) — a candidate roll feedforward, not built yet.
- **Lifetime**: 0 at controller start (each drive), kept across engagements,
  never persisted.
- **Telemetry**: `sl_bias` (Nm), `sl_learn`.
- **Not included**: upstream-style `−g·roll` feedforward. On 4e9 roll varied
  ±0.4° between roads without tracking the pull (|r| ≤ 0.1), and the mount roll
  is unknown. Revisit if the learned `sl_bias` correlates with roll.
- **Default OFF**; Driving-panel toggle "Slope Compensation". Rollback: toggle
  off and re-engage.
- Design: `docs/superpowers/specs/2026-10-19-bmw-slope-bias-design.md`.

---

## 13. Code map (`bmw/latcontroller.py`)

| section | where | content |
|---|---|---|
| Module constants | top of file | `SB_*`, `V_GAIN_FLOOR`, LKA band, `CONTROL_HORIZON_S`, `SL_*`, `CF_*` + `curve_ff_nm`, the lane_keeping readiness check (R10) |
| `on_lat_controller_init` | module | R10 guard, `StallBreakaway` read, cadence/gain/step/dwell constants, the state dict, `update` closure |
| Engagement edge | top of `update()` | `SlopeCompensation` latch, guard reset; inactive zeroes torque/target/ramp |
| Stall/breakaway v2 | `update()`, before the livePose branch | ring / arm / breakaway / trip, shed-rate enforcement |
| livePose tick | `update()` | reference, speeds, `delta_err`, `kappa_scale`, `a_ref`/`ff_nm`, slope-bias learning, `demand_nm`, guard observe, relax-dwell |
| Decision | `update()`, every `action_cadence_ticks` while active | demand → `t_cap` → dwell bridge → guard → `STEP_MAX` → sb block |
| Contact learning, take-up | `update()`, every CAN tick | `ContactLearner.observe`, `FreePlayTakeUp.step`, handover |
| Ramp and telemetry | end of `update()` | per-frame ramp step, zero snap, `bmw_lat_control` payload (§9) |

`bmw/takeup.py`: `ContactLearner`, `SignGuard`, `FreePlayTakeUp` and their constants.

---

## 14. Quick verification recipe

To check that a deployed change behaves as expected:

```bash
# On c3:
cd /data/openpilot && source /usr/local/venv/bin/activate

# Load a route's bmw_lat_control + modelV2:
python -c "
import zstandard, json
from cereal import log
ev = list(log.Event.read_multiple_bytes(zstandard.ZstdDecompressor().decompress(
    open('/data/media/0/realdata/<ROUTE>--<SEG>/rlog.zst','rb').read(),
    max_output_size=600*1024*1024)))
# ... iterate events, e.which()=='pluginBusLog' → e.pluginBusLog.entries
#                 where en.topic=='bmw_lat_control' → json.loads(en.json)
"
```

Key metrics to compute per route:
- **Reference-smoothing efficacy** (Phase 2: this now lives in `lane_keeping`, not `latcontroller`): compare `kappa_in` vs `kappa_ref` sign-flips per second from the `lane_keeping` telemetry topic — `delta_err_raw` no longer exists in `bmw_lat_control` telemetry, since the box filter it fed was deleted from this controller.
- **Lane offset**: from `modelV2.laneLines[1].y[0]` and `modelV2.laneLines[2].y[0]` averaged. RMS should be 0.30-0.45 m on healthy drives.
- **`de_at_end_raw`**: δ_err at the moment laneChangeState flips back to `off`. Should be < 0.3° on healthy LCs.
- **`post_tq_max`**: peak `|output·STEER_MAX|` in the 1.5 s after each LC end. Should be < 2-3 Nm.

Reference baselines (field-verified, 2026-05):
- Route 31c: lane offset rms 0.33 m, filter reduction 59% — the "nearly perfect" baseline
- Routes 32a / 32d: 0 flagged LC events out of 39 LCs, filter reduction 65-66%, cancel_accel essentially eliminated — current stable operating point

> **Design law (user, 2026-07-28): the lateral controller never gives up in
> curves.** A last-resort measured-a_y backstop (fire only >3.0 m/s² sustained
> with into-turn torque) was recommended by the removal review and DECLINED —
> the review's analysis is preserved in the project records should a future
> incident reopen the question. a_y responsibility rests entirely with
> speedlimitd's curve capping + panda torque bounds + driver supervision.
