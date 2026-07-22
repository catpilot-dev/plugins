# Phase 2 — Lateral Controller Simplification (noise offloaded to lane centering)

**Date:** 2026-07-22
**Status:** design approved, pending implementation plan
**Predecessor:** `2026-07-22-driver-side-lane-keeping-design.md` (Phase 1)
**Baseline to modify:** BMW lateral controller @ `af3be5f` + `lane_keeping` plugin @ `a0afa58` (both on `dev`)

## 1. Motivation — what Phase 1 proved

Phase 1 deployed the driver-side position anchor *alongside* the existing
DRIFT_M controller. Route 3bf (segs 5–18) showed it is **net-negative as
deployed**, and the telemetry gives an unambiguous cause.

Measured (3bf, anchor active 88% of the drive, prob med 0.96):

| metric | 3bf (anchor + DRIFT_M) | 3b7 (DRIFT_M era) |
|---|---|---|
| lateral position std | **0.47 m** | 0.36 m |
| p95 offset | **+0.77 m** | +0.52 m |
| driver-side gap in-band | **24%** | — |
| time out-of-band | **66%** | — |
| left wheel <5 cm from line | **4.7%** | — |
| right wheel <5 cm from line | **1.8%** | — |
| torque reversals | 34/min | 25–49/min |

Excursions: 138 episodes, p90 **11.5 s**, max **75 s**. Min wheel margins
**−0.48 m** (left) and **−0.59 m** (right) — real line crossings. The gap
autocorrelation shows **no periodicity**, so this is slow drift, not a limit
cycle.

**Root cause — the inner deadzone eats the outer loop's command.** On the 9958
ticks where the anchor was actively correcting (`|excess| > 0.05`):

- anchor bias in the delta domain: p50 **0.00209 rad**
- controller deadzone (`tolerance`): p50 **0.00120 rad**
- ratio: **1.98** — and the controller took **no action on 44%** of those ticks
  (`hold_zero` 42.5%), because the resulting error sat inside its own deadzone.

The 75 s excursion is the proof end-to-end: the bias was correctly signed
(negative = steer right), pinned at the `−0.50` excess clip for the full 75 s,
while the gap sat at −0.21…0.45 m — riding on and past the left line — and never
recovered.

**Conclusion:** an outer position loop cannot work through an inner loop with a
deadzone; the deadzone attenuates its command by construction. The anchor ended
up *too weak to reach its own target but strong enough to disturb the model's
centering* — hence the worse spread. Phase 1 coexistence is architecturally
unable to validate the feature. The deadzone has to go.

## 2. Architecture — two layers, one job each

**User directive:** *"simplify the BMW lateral controller, offload the modelV2
sub-Hz noise to lane centering."*

modelV2 noise lives in two bands and each is handled where it is tractable:

- **fast chatter** → removed by filtering the *reference*
- **sub-Hz wander** → cannot be filtered out (it overlaps real road geometry);
  it is cancelled in the *position* domain, where it is directly measurable

```
        modelV2 desiredCurvature (noisy)
                   |
   +---------------v-----------------------------+
   |  lane_keeping  (curvature_correction hook)  |   <-- owns ALL noise + position
   |   1. low-pass kappa_des      (kill chatter) |
   |   2. + driver-side position correction      |
   |      (pure-pursuit deadband, existing)      |
   +---------------|-----------------------------+
                   |  one clean, position-corrected curvature
   +---------------v-----------------------------+
   |  BMW latcontroller  (faithful tracker)      |   <-- no noise reasoning at all
   |   plant-inversion P on FULL delta_err       |
   |   + stiction hold, STEP_MAX, ISO guards     |
   +---------------------------------------------+
```

**Why filtering is safe now and was not before.** Any lag or steady-state error
the low-pass introduces manifests as position drift — which the position loop
measures and cancels. Without a position loop, that same lag produced the
route-395 low-frequency weave, which is why the box-filter window could never be
lengthened. The cascade removes that constraint.

## 3. `lane_keeping` changes — reference conditioner

Add reference smoothing ahead of the existing position correction. **It lives in
`anchor.py` (`LaneAnchor.update`)**, which already receives the incoming
curvature and returns the modified one — it is control logic, so it belongs in
the pure core. `register.py` stays hook wiring, config, and telemetry only.
Filter state (`kappa_filt`) sits alongside `gap_filt` and follows the same
reset discipline.

```
kappa_in                                   # model desiredCurvature
if lane_changing:                          # model is reframing the trajectory
    kappa_ref = kappa_in                   # bypass filter (canonical signal)
    reset filter state
else:
    kappa_ref = low_pass(kappa_in, tau=KAPPA_FILTER_TAU)
return kappa_ref + authority * kappa_bias  # position correction unchanged
```

- **Smoothing applies in BOTH states** (ANCHOR and MODEL) — noise handling is
  this layer's job unconditionally. Only the *position correction* is
  ANCHOR-gated.
- **`KAPPA_FILTER_TAU` default 0.3 s.** Rationale: the reverted box filter was a
  300 ms window and was considered safe; 600 ms was not (route 395). 0.3 s is
  therefore the known-safe scale, and is now additionally protected by the
  position loop. Config param; validated on replay and first drive.
- Lane-change bypass mirrors the existing `kappa_bias` hard-zero.

## 4. BMW `latcontroller` changes — faithful tracker

### 4.1 Deleted

| item | why it existed |
|---|---|
| `DRIFT_M` + kinematic `tolerance` | drift-sized deadzone (no position feedback existed) |
| `KD_BLEND` box filter (`de_buffer`, `de_w`, `delta_err_raw` blend) | curvature-domain noise coping |
| σ-observer noise floor (`KN_EMA_*`, `KN_VAR_ALPHA`, `KN_SIGMA_*`, `KN_FADE_BP`, `KN_GATE_KAPPA`, `KN_DRIFT_CAP_M`, `k_sigma`, `kn_ema_f`, `kn_ema_s`, `kn_var`) | made the tight deadzone survive noise |
| sign-persistence gate (`KN_DC_ALPHA`, `KN_PERSIST_BP`, `de_dc`, `persist_w`) | fixed the noise floor's own side effect |

All four exist only to cope with curvature-domain noise, which is now handled
upstream.

### 4.2 The three `tolerance` consumers, resolved

1. **`effective_err = delta_err − copysign(tolerance, delta_err)`** →
   **deleted**. P acts on the **full** `delta_err`.
   *This is the single change that fixes Phase 1's failure* — it is the term
   that attenuated the anchor's command.
2. **`if abs(delta_err) <= tolerance:` → `hold_zero` / `hold_curve`** →
   replaced by a small fixed **`HOLD_BAND`**. The stiction hold is still
   required (the BMW rack unwinds at zero torque), but its trigger is now sized
   by *stiction*, not by drift.
   **Sizing:** below the error at which the P term commands less than rack
   breakaway, the wheel cannot move anyway. `e ≈ FRICTION·STEER_MAX /
   (T_CAP_SLOPE·kappa_scale·v²)` ⇒ ≈ 0.001 rad at 25 m/s. **`HOLD_BAND = 0.001`
   rad** (fixed, config-tunable). Small enough that residual attenuation of the
   anchor is negligible (previously 0.0012–0.0021 rad and speed-dependent).
3. **`cancel_tol` (`|delta_err| <= 1.2·tolerance`)** → gated on
   `1.2·HOLD_BAND`.

### 4.3 Kept (nothing to do with noise)

Plant-inversion P, `kappa_scale`, `t_cap`, `STEP_MAX` (speed-scaled slew cap),
`hold_curve` / hold-floor / `hold_cap` / sign-guard (rack-stiction holding), ISO
`cancel_accel` / `cancel_jerk` guards.

**`relax-dwell` kept, reviewed on evidence.** It bridges mid-hairpin κ_des dips;
reference smoothing may now cover that, but it catches a fast failure (~20°
unwind in ~1 s) the slow position loop will not. Decide from Phase 2 data.

### 4.4 Telemetry

Drop `k_sigma`, `de_dc`, `persist_w`, `de_w`, `delta_err_raw`, `tolerance`.
Add `hold_band` (constant, for band-occupancy forensics). `lane_keeping` gains
`kappa_in` and `kappa_ref` so the smoothing is observable.

## 5. Risks and open items

- **Anchor authority may still be insufficient.** Phase 1 had it saturated and
  losing. Deleting `effective_err` roughly doubles its effective authority, but
  the 75 s excursion suggests that may not close the gap. `KAPPA_BIAS_MAX` and
  `T_PREVIEW` are the knobs; **set them from the first Phase 2 drive rather than
  guessing now.** This is the most likely follow-up.
- **Smoothing lag on curve entry.** 0.3 s is the known-safe scale but the
  position loop now also has to absorb it. Watch curve-entry response and
  residual p2p.
- **`HOLD_BAND` sizing** is derived, not measured. If the stiction hold engages
  late (wheel unwinds before hold) or early (holds while still under-turning),
  this is the knob.
- **MODEL state (~12%) is safe.** Removing the deadzone makes the controller
  track the model *more* faithfully, not less. The route-31b 1.3–1.7 m drift
  came from a *wide* deadzone leaving error unactioned; zero deadzone is the
  opposite failure mode. That risk is retired, not re-created.
- **Single large step.** The whole noise stack goes at once. Rollback is one
  revert to the `af3be5f` controller config; `lane_keeping` can be disabled
  independently via `.disabled` or `LaneKeepEnable=0`.

## 6. Testing

1. **Unit** — `lane_keeping`: smoothing time constant, lane-change bypass +
   filter reset, smoothing applies in both states, position correction still
   ANCHOR-only. `latcontroller`: P acts on full `delta_err` (no subtraction),
   `HOLD_BAND` hold/cancel triggers, deleted machinery absent, kept mechanisms
   (STEP_MAX, hold-floor, sign-guard, ISO) unchanged.
2. **On-device probes** — rewrite the existing `bmw` probe harness: its
   noise-floor / persist-gate / tolerance probes are deleted with the feature;
   replace with `HOLD_BAND` and full-error probes.
3. **On-car (the real gate)** — a structured highway route, compared against
   both baselines:

| criterion | target | reference |
|---|---|---|
| driver-side gap in-band | **≫ 24%** | Phase 1 (3bf) |
| lateral position std | **≤ 0.36 m** | 3b7 DRIFT_M era |
| lane touches (<5 cm) | **≈ 0%** | 3bf: 4.7% / 1.8% |
| torque reversals | **≤ 34/min** | 3bf; 3ac era 25–49/min |
| curve residual p2p | within verified band | 393/3a9 (9.9–13.8°, best 4.9°) |
| give-ups / overturns | 0 | 3a9 |

Reuse the existing analysis scripts (`lk_oncar.py`, `lk_absorb.py`, the
lane-offset pipeline).

## 7. Non-goals

- Changing the anchor's control law (pure-pursuit deadband) — only its authority
  may be retuned, from data.
- `CAMERA_OFFSET` in the gap (~0.05 m) — separate follow-up.
- Any change to longitudinal, speedlimitd, or other plugins.
- Removing `relax-dwell` — evidence-gated, not part of this change.
