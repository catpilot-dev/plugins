# BMW E9x/E8x Lateral Controller — Design & Tuning Reference

This document is the canonical reference for the lateral controller registered by `bmw/latcontroller.py::on_lat_controller_init` (hook: `controls.lat_controller_init`). It describes **what** the controller does, **why** it's shaped this way for the BMW hydraulic rack, **how** the single-knob timing design propagates, and **what was tried and rejected**. Future maintainers should be able to retune without re-litigating the failed experiments.

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

---

## 1. Why a custom controller (not stock latcontrol_torque)

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

When `|δ_err| ≤ HOLD_BAND` the cadence decision holds (target → 0 on straight, or holds the last target on a curve) and stiction keeps the rack in place. Outside `HOLD_BAND`, the controller ramps to `target_nm` over `spread_frames` CAN ticks. ISO comfort guards (cancel_jerk / cancel_accel) gated on **actual plant overshoot** brake the ramp if the plant runs past κ_des.

---

## 2. Single-knob timing design

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
        ▼  ISO comfort guards (gated on actual overshoot)
        │  overshooting = (κ_des - κ_meas) · κ_meas < 0
        │  if overshooting and |a_y_meas| > BMW_LATERAL_ACCEL → cancel_accel
        │  elif overshooting and |jerk_pred| > BMW_LATERAL_JERK → cancel_jerk
        │  → target_frac drains to 0 (reverse-FRICTION pulse removed 2026-07-03)
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

## 7. ISO 11270 comfort guards

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

State held in `state['action']`, published in `bmw_lat_control` telemetry. Useful for forensic analysis.

| state | when entered |
|---|---|
| `init` | controller construction |
| `hold_zero` | `|delta_err| ≤ HOLD_BAND`, straight (`hold_f = 0`) — target 0, stiction holds |
| `hold_curve` | `|delta_err| ≤ HOLD_BAND`, curve (`hold_f > 0`) — target `hold_f·torque`, holds the standing torque against self-aligning torque |
| `ramp` | active plant-inversion push toward `target_nm` (sub-friction targets commanded as-is since 2026-07-03) |
| `relax_dwell` | overshoot-side error in a measured deep curve, within the 1 s dwell — target bridges at current (capped) torque |
| `cancel_tol` | error fell into the on-target band (1.2× `HOLD_BAND`) mid **push** ramp (`action=='ramp'` only); drain to the sign-guarded, capped hold (0 on straights) |
| `cancel_accel` | overshoot AND `|a_y_meas| > BMW_LATERAL_ACCEL` — drain to 0 |
| `cancel_jerk` | overshoot AND `|jerk_pred| > BMW_LATERAL_JERK` — drain to 0 |
| `idle` | (2026-07-19) between cadence decisions after a transient label's ramp completed — expires ramp/relax_dwell/cancel_* so telemetry occupancy counts are honest; holds never expire (they re-fire each cadence) |

Removed 2026-07-03 (see header note): `brake_zero`, `breakaway`. Added: `hold_curve`. Telemetry gains `hold_f`.

---

## 9. Telemetry (plugin_bus topic `bmw_lat_control`, 20 Hz)

Published each livePose tick:

| field | meaning |
|---|---|
| `desired` | κ_des actually used by controller (= raw, since no κ filter) |
| `desired_raw` | identical to `desired` (kept for back-compat with older filter designs) |
| `measured` | κ_meas from livePose yaw rate / v_ego |
| `err` | κ_des − κ_meas (debug) |
| `delta_err` | front-wheel angle error (rad) — what controller acts on |
| `target_frac` | current cadence-decision target torque fraction (−1..1) |
| `ramp_step` | per-CAN-tick torque increment |
| `ramp_frames` | CAN frames remaining in current ramp |
| `action` | state machine label (see §8) |
| `torque` | current commanded torque fraction (−1..1) |
| `output` | clipped torque fraction returned to controlsd |
| `vEgo` | from CS.vEgo |
| `active` | controller active flag |
| `a_y_meas` | `v²·κ_meas` (m/s²) |
| `jerk_pred` | `v²·(κ_des - κ_meas)/model_action_t` (m/s³) |
| `hold_f` | curvature hold factor [0, 1] |
| `hold_cap` | cap on held torque (P value implied by the fixed `HOLD_CAP_DRIFT_M` reference, frac — independent of the deleted tolerance) |
| `relax_ticks` | consecutive ticks of overshoot-side error while deep in a curve (dwell counter) |
| `hold_band` | (2026-07-22) fixed stiction hold trigger (rad) — not a deadzone; P acts on full error |

Multiply `output` or `torque` by `STEER_MAX = 12 Nm` for Nm.

---

## 10. Constraints baked into the design

- **Vision-only stack**: catpilot has no HD-map, lidar, radar fusion, or IMU-fusion for κ. All correction must come from `modelV2` outputs. Don't propose external-sensor escape hatches for vision-noise problems.
- **Position-feedback layer (2026-07-22, Phase 2)**: `lane_keeping` IS registered on `controls.curvature_correction` — it low-passes `modelV2` κ_des (killing fast chatter) and closes a driver-side lane-offset position loop (cancelling sub-Hz wander, which cannot be filtered out of the reference alone). It now owns modelV2 noise handling and lateral position entirely; this `latcontroller` is a faithful tracker with no noise reasoning of its own (§3, §5).
- **lagd never converges for BMW**: `liveDelay.lateralDelay` is permanently pinned at `SAD + 0.2` because lagd correlates on latcontrol_torque telemetry our controller doesn't produce. `SAD` is the effective knob — not a hint to be overridden.
- **BMW hydraulic rack**: stiction holds δ at zero torque, no self-centering. The plant-inversion controller is designed around this. **Don't add a FF term on κ_des** (correct for EPS, wrong here).

---

## 11. Failed experiments (do not re-litigate)

These were tried, deployed, and reverted. Each appears here so future maintainers know what's been ruled out and why.

| experiment | result | file:line | reverted by |
|---|---|---|---|
| Constant-angle deadzone (`TOL_DEG_CONST = 0.35°`) | Allowed 1.3–1.7 m drift at 85 kph with no position-feedback layer running; controller silent 98% of samples (route 31b seg 8/15). Wide angular deadzone is unsafe without a position-feedback layer. | commit 715114d | d43fb19 |
| Stiction-gated steering-angle FF (linear in \|steer\|, 20°/0.10 Nm·deg⁻¹) | Over-pushed in route 31d seg 8 right turn: 50% over-rotation, 6.22 Nm torque, sustained cancel_jerk oscillation. SAT physics aren't linear in steering angle alone — needs multi-regime (v×κ grid) calibration, not single-route fit. | commit 2c67a97 | e81f1c0 |
| Hysteresis-on-κ_des (`KD_HYST_GAP = 0.003`, `KD_GATE = 0.006`) | 95% sign-flip reduction in offline check, but route 322 had 32% time `\|offset\|>0.5 m` and 44-second frozen-κ_des stretches — controller faithfully tracked a held biased reference → drift. Structural failure of holding the reference without a position-feedback layer. | commit daef207 | 96945fc (replaced with box-on-delta_err) |
| Suggesting external sensor fusion (HD map, radar, lidar, IMU) for vision noise | Out of project scope by design — see [vision-only constraint](§10). | (recurring; do not propose) |

---

## 12. Retuning recipes (current operating point)

Current configuration is field-verified stable (2026-05-24, routes 32a/32d, user: "solid in straight, agile and accurate in turns"). Only retune if a new failure mode appears.

### To change the controller's overall timing (cadence, ramp, jerk horizon, modeld lookahead all together):
- Change `ret.steerActuatorDelay` in `bmw/interface.py:87`
- Range `[0.3, 0.5]` keeps cadence in `[5, 7]` ticks (250-350 ms). Don't go below 0.3.

### To suppress more wobble at near-straight (filter):
- `KD_GATE` and `kd_filter_window` no longer exist — the κ-gated delta_err box
  filter was deleted in Phase 2 (2026-07-22, see § 4). There is nothing to
  retune in this controller for near-straight wobble suppression.
- Reference smoothing now lives in the `lane_keeping` plugin: retune
  `KAPPA_FILTER_TAU` there (currently 0.15 s) to trade group delay for
  smoothing on its κ_des low-pass.
- In this controller, the only near-straight knob left is `HOLD_BAND`
  (currently 0.001 rad) — the fixed stiction hold-retrigger threshold. It is
  sized by rack breakaway, not by allowed wobble; widening it to fight noise
  reintroduces the drift failure mode the deleted deadzone had (§ 5).

### If lane-change overshoot reappears on the post-LC phase:
- Look at `bmw_lat_control` log for that LC: is `de_at_end_raw > 0.3°`? Is post-LC `|torque|` > 3 N?
- The mechanism is one of: spurious cancel_jerk suppressing controller during LC (small κ, |κ_meas| < 0.001 — see seg 10 of route 326 for pattern) OR rack stiction → catch-up overshoot (see seg 42 ev2 for pattern).
- Don't apply a global LC torque cap — most LCs are fine. Targeted options exist (magnitude-gated cancel, soft post-LC re-engagement); see commit history around routes 326/32a/32d for the analysis.

### If straights feel less smooth than today:
- First, check that the κ_des wobble character hasn't changed (newer model versions may have different noise).
- This controller no longer filters — check `lane_keeping`'s `KAPPA_FILTER_TAU`
  (currently 0.15 s) first; that low-pass is what shapes straight-line feel now.
  Shortening it trades smoothing for responsiveness on its position loop.

### If real-curve tracking feels under-aggressive:
- Increase `T_CAP_SCALE_BP[1]` or `T_CAP_SCALE_BP[2]` (currently 2.5, 3.0 at `|κ| = 0.01, 0.02`).
- **Don't** increase `T_CAP_SCALE_BP[0]` past 1.0 — that's where route 326's over-correction lived (was 1.5, now back to 1.0).

### If cancel events feel too frequent on moderate curves:
- Loosen `LATERAL_ACCEL_BP` / `LATERAL_JERK_BP` at the moderate-κ breakpoints (currently 2.5 / 3.0 at `|κ|=0.01`).
- Don't loosen the small-κ values (1.5) — they're the brake against small-amplitude overshoot.

### If unwind pulses are felt too often (controller pushes back too easily):
- Loosen `BMW_LATERAL_JERK_BP[0]` / `BMW_LATERAL_ACCEL_BP[0]` (currently 1.5). Cancel guards will fire less often at small κ. But verify no spurious overshoot crept in.

---

## 13. Code map (`bmw/latcontroller.py`)

| section | line range (approx) | content |
|---|---:|---|
| Constants block | 210-360 | All tuning constants with rationale comments |
| State init dict | 320-360 | Per-controller persistent state |
| `update()` function | 360+ | Per-CAN-tick body, with livePose-gated heavy logic |
| Plant horizon block | inside update | `model_action_t`, cadence/spread computed from `lat_delay` |
| Hold-cap / hold-floor block | inside update | `hold_cap` (HOLD_CAP_DRIFT_M reference), `held_target`, sign-guard |
| HOLD_BAND on-target check | inside update | Fixed stiction hold-retrigger threshold (no deadzone; § 5) |
| ISO guards | inside update | Overshoot-gated cancel_jerk / cancel_accel |
| Cadence decision | inside update | hold_zero / hold_curve / ramp / target_nm formula |
| Ramp application | inside update | Per-CAN-tick torque ramp |
| Telemetry publish | inside update | `bmw_lat_control` topic via plugin_bus |

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
- **cancel_jerk / cancel_accel counts**: per lane change (should be < ~10 / LC each)
- **`de_at_end_raw`**: δ_err at the moment laneChangeState flips back to `off`. Should be < 0.3° on healthy LCs.
- **`post_tq_max`**: peak `|output·STEER_MAX|` in the 1.5 s after each LC end. Should be < 2-3 Nm.

Reference baselines (field-verified, 2026-05):
- Route 31c: lane offset rms 0.33 m, filter reduction 59% — the "nearly perfect" baseline
- Routes 32a / 32d: 0 flagged LC events out of 39 LCs, filter reduction 65-66%, cancel_accel essentially eliminated — current stable operating point
