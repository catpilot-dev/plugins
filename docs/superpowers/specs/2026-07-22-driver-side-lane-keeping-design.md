# Driver-Side Lane Keeping — Design Spec

**Date:** 2026-07-22
**Branch:** `lane_keeping` (off `dev` @ af3be5f)
**Status:** Phase 1 design, approved for implementation
**Author:** design dialogue (oxygen + Claude)

## 1. Motivation

The BMW lateral controller tracks `modelV2`'s `desiredCurvature`, whose **sub-Hz
wander** we spent two weeks mitigating with a kinematic deadzone (`DRIFT_M`),
a `delta_err` box filter, a live σ-observer noise floor, and a sign-persistence
gate. All of that is *curvature-domain* noise coping: `DRIFT_M` is a noisy
**estimate** of lateral position drift derived from the noisy curvature error,
and the rest of the apparatus exists to keep that estimate usable.

`desiredCurvature` is a *rate* signal — its sub-Hz noise integrates into slow
lateral position drift. A driver-side lane line gives an **absolute position
reference**, and a slow loop closed on that integrated quantity rejects the
curvature wander the inner controller cannot. In a left-hand-drive car the
driver sits on the left and judges the left line most confidently; keeping a
consistent margin from that line is both the human driving style and a
noise-robust control anchor.

The controller comment on `DRIFT_M` states the unlock directly: the reverted
constant-angle deadzone (route 31b) is *"only viable on top of an external
position-feedback loop, which the public build has none of."* **Driver-side
lane keeping is that missing position-feedback loop.** It *measures* the
lateral position `DRIFT_M` only *estimates*.

## 2. End-state architecture (target — context for Phase 1)

A **two-state lateral controller** with `DRIFT_M` and its entire noise-coping
apparatus removed:

- **ANCHOR** — a confident driver-side line is present: lateral position is
  governed by the driver-side line. A deadband `[GAP_MIN, GAP_MAX]` on the
  driver-wheel-to-line gap decides when to act, *replacing* the kinematic
  curvature-error tolerance. The model still supplies the road's *shape*
  (curvature through turns); the anchor governs only *where in the lane* the
  car sits. Within the band → no correction (follow model). Outside → a
  bounded curvature bias brings it back.
- **MODEL** — no reliable driver-side line: follow `desiredCurvature` directly.

Every scenario where `DRIFT_M`'s noise problem actually bit us was a structured
highway road at 60–90 km/h, which by definition has a confident driver-side
line and therefore lives in ANCHOR state. That is why `DRIFT_M` can eventually
die (Phase 2).

### Deletion boundary (Phase 2, for reference)

**Deleted:** `DRIFT_M`/kinematic `tolerance`; `KD_BLEND` box filter on
`delta_err`; σ-observer noise floor (`k_sigma`, `KN_SIGMA_MULT`, `KN_FADE`);
sign-persistence gate (`de_dc`, `persist_w`).

**Kept:** plant-inversion P term, `kappa_scale`, `t_cap`; `STEP_MAX` slew cap;
`hold_curve`/hold-floor/`hold_cap`/sign-guard (rack-stiction holding); ISO
`cancel_accel`/`cancel_jerk` guards.

**Review on evidence:** `relax-dwell` (mid-hairpin κ_des-dip bridge) — it
catches a fast failure (~20° wheel unwind in ~1 s) the deliberately-slow anchor
may not react to. Keep through Phase 1; decide on Phase 1.5 data.

## 3. Phase 1 scope (this spec)

Build a **standalone `lane_keeping` plugin** registering on the
`controls.curvature_correction` hook, coexisting with a fully-active `DRIFT_M`.
The anchor's bias rides upstream of the existing controller; because it is
usually zero (in-band) and always gentle, the interaction is minimal and clean.
`DRIFT_M` and the noise floor remain the fallback (MODEL state = literal
passthrough). Phase 1 proves the anchor holds position and rejects wander,
on-car, before Phase 2 removes anything.

### 3.1 Hook integration

Call site (`selfdrive/controls/controlsd.py:127`, catpilot repo — no change
needed, hook already exists):

```python
new_desired_curvature = hooks.run('controls.curvature_correction',
    new_desired_curvature, model_v2, CS.vEgo, lane_changing, lat_delay=lat_delay)
```

Runs **before** `clip_curvature` and the lateral controller. Plugin callback
signature (chain hook — receives current value first, returns modified value):

```python
def on_curvature_correction(curvature, model_v2, v_ego, lane_changing, lat_delay=None):
    # returns curvature unchanged (MODEL) or curvature + authority*bias (ANCHOR)
```

Curvature sign convention (openpilot): **positive curvature = left turn**
(counterclockwise, +y), `yaw_rate = v · curvature`.

### 3.2 Signals from `modelV2`

- `laneLines[1]` = left ego lane line, `laneLines[2]` = right ego lane line.
  `.y[0]` = lateral position of the line at the car (device frame, **+y = left**).
- `laneLineProbs[idx]` = per-line confidence.
- `side_sign`: `+1` for `DRIVER_SIDE=left` (`idx=1`), `−1` for `right` (`idx=2`).
- `gap_center_to_line = side_sign · laneLines[idx].y[0]` (positive both sides).
- `driver_wheel_to_line = gap_center_to_line − HALF_WIDTH`.

### 3.3 Control law (ANCHOR)

```
prob   = laneLineProbs[idx]
gap    = side_sign * laneLines[idx].y[0] - HALF_WIDTH
gap_f  = low_pass(gap, tau=FILTER_TAU)                       # measurement-noise reject
excess = gap_f - clip(gap_f, GAP_MIN, GAP_MAX)               # the tolerance/deadband
excess = clip(excess, -EXCESS_MAX, EXCESS_MAX)               # glitch reject
Lp     = v_ego * T_PREVIEW
kappa_bias_raw = side_sign * 2.0 * excess / Lp**2            # pure-pursuit
kappa_bias = clip(kappa_bias_raw, -KAPPA_BIAS_MAX, KAPPA_BIAS_MAX)
kappa_bias = rate_limit(kappa_bias, KAPPA_RATE_MAX)          # per-tick slew
authority  = interp(prob, [PROB_ON, PROB_ON+PROB_FADE], [0,1])
if lane_changing: authority = 0.0
return curvature + authority * kappa_bias
```

**Why pure-pursuit (key design choice):** curvature proportional to position
error is an *undamped oscillator* (position is a double integral of curvature).
The `v·T_PREVIEW` look-ahead provides anticipation (damping) and speed
adaptation. The resulting lateral accel is `2·excess/T_PREVIEW²`, **independent
of speed** and naturally bounded (~0.44 m/s² at `EXCESS_MAX=0.5 m`,
`T_PREVIEW=1.5 s`). It is `1/v²`-scaled, mirroring the kinematic logic the
controller already speaks.

**Sign check.** `excess>0` = gap too large = car too far from the driver line
(toward the non-driver side) → `kappa_bias` sign `= side_sign` → steers toward
the driver side. `excess<0` = car too close to the driver line → opposite →
steers away. Verified for both `DRIVER_SIDE` values.

### 3.4 Safety envelope

- **No push to center.** Deadband makes the correction stop at the band edge;
  it never hunts a setpoint or overshoots inward. In-band → exactly zero.
- **Bounded & gentle.** Accel-bounded by construction, plus a hard
  `KAPPA_BIAS_MAX` backstop and a per-tick rate limit.
- **Glitch reject.** `EXCESS_MAX` clip means a spurious line reading (gap = 5 m)
  cannot produce a large bias.
- **Authority fade.** Smooth fade over `[PROB_ON, PROB_ON+PROB_FADE]` — no snap
  at the 0.6 threshold — and hard 0 during lane changes.
- **Fallback.** Anything short of a confident line → passthrough → the existing
  `DRIFT_M` controller runs bit-identically to today.
- **State on transition.** On leaving ANCHOR, decay `kappa_bias` to 0 via the
  rate limiter (no snap); reset the low-pass so a re-entry starts clean.

### 3.5 Config (plugin `data/` dir, never `/data/params/d/`)

| param | default | meaning |
|---|---|---|
| `ENABLE` | true | master toggle |
| `DRIVER_SIDE` | `left` | which ego line to anchor (`left`/`right`) |
| `HALF_WIDTH` | 0.91 | car half-width (m); E90 ≈ 1.817 m |
| `GAP_MIN` / `GAP_MAX` | 0.6 / 1.0 | driver-wheel-to-line comfort band (m) |
| `T_PREVIEW` | 1.5 | pure-pursuit look-ahead time (s); speed-independent correction accel = `2·excess/T_PREVIEW²` (~0.44 m/s² at full excess). **Kept at 1.5 s deliberately — shortening it makes high-speed steering more aggressive (route-39b lesson); validated in offline replay (§4.2) before on-car.** |
| `EXCESS_MAX` | 0.5 | max deadband excess acted on (m) |
| `KAPPA_BIAS_MAX` | 0.002 | hard cap on curvature bias (1/m); binds only below ~54 km/h, keeps low-speed corrections gentle |
| `KAPPA_RATE_MAX` | 0.002 /s | bias slew (1/m per second; ≈ full range in 1 s), converted to per-tick at the 100 Hz control rate; refined on replay/on-car |
| `FILTER_TAU` | 0.7 | gap low-pass time constant (s) |
| `PROB_ON` | 0.6 | driver-side line confidence to engage |
| `PROB_FADE` | 0.1 | fade width above `PROB_ON` |

Defaults ship BMW-left; params make it region- and vehicle-portable.

### 3.6 Telemetry

A `lane_keeping` plugin-bus topic (same pattern as `bmw_lat_control`, published
at the hook rate) with: `prob`, `line_y`, `gap`, `gap_filt`, `excess`,
`kappa_bias`, `authority`, `state` (`anchor`/`model`), `v_ego`. Enables the same
on-car lane-offset verification pipeline used for the 3b3/3b7/3bb evaluation.

## 4. Testing

1. **Unit (pytest, mirrors `bmw` plugin suite):** sign correctness both
   `DRIVER_SIDE` values; deadband (in-band → bias exactly 0; <`GAP_MIN` →
   correct away from line; >`GAP_MAX` → toward line); pure-pursuit magnitude and
   accel bound; `EXCESS_MAX` glitch clip; rate-limit and `KAPPA_BIAS_MAX` cap;
   authority fade across `[PROB_ON, PROB_ON+PROB_FADE]`; **MODEL passthrough
   returns input curvature bit-identical**; lane-change disable; transition
   decay/reset.
2. **Offline replay** on existing logs (3b7, 3bb, 3ac, 3b3 left-hug): run the
   anchor over recorded `modelV2` lane lines; confirm quiet in-band, correct
   bias direction out-of-band. 3b3 cross-check: the anchor must detect the
   left-hug and bias right. Validates the *detector* open-loop (closed-loop
   centering cannot be proven offline — same caveat as the persist gate). Also
   measures ANCHOR-vs-MODEL occupancy (prior data: 63–100% confident-line).
3. **On-device probe harness** (like `on_device_probe.py`): drive
   `on_curvature_correction` with synthetic `modelV2` on the real C3 runtime,
   offroad; assert every branch.
4. **On-car:** deploy; drive a structured highway route; via telemetry confirm
   ANCHOR engages on clear-line stretches, driver-side gap holds in
   `[GAP_MIN, GAP_MAX]`, position drift is **lower** than baseline (the
   wander-rejection claim), and there is no new churn or snap at prob
   transitions — using the 3b7/3bb lane-offset pipeline.

## 5. Phasing

- **Phase 1 (this spec):** plugin coexisting with active `DRIFT_M`; four test
  layers; deploy; verify hold + wander rejection.
- **Phase 1.5 (evidence gate):** with both live, measure on-car whether
  `DRIFT_M`/noise-floor is redundant on structured roads. Data gate for Phase 2.
- **Phase 2 (separate spec, gated):** delete `DRIFT_M`, box filter, σ-observer,
  persist gate; collapse to the two-state controller; decide `relax-dwell` on
  evidence. Its own brainstorm→spec→plan cycle — it rips out the freshly-
  stabilized core.

## 6. Non-goals / out of scope (Phase 1)

- Removing any existing controller mechanism (that is Phase 2).
- HD-map / lidar / radar fusion (vision-only stack — hard constraint).
- A UI panel toggle (param-file configured for Phase 1; UI is a later follow-up).
- Cross-checking both lines / road-edge fallback (rejected: "no clear line →
  follow model" is the agreed fallback).
- Non-BMW vehicle validation (params make it portable; only BMW-left is verified
  in Phase 1).

## 7. Risks / open questions

- **Closed-loop stability on-car.** Pure-pursuit + slow filter + gentle gain
  should be well-damped, but the true loop includes the rack, the existing
  controller, and filter lag. Mitigation: `KAPPA_BIAS_MAX` and rate limit bound
  the worst case to a gentle correction; on-car verify is the gate.
- **`KAPPA_RATE_MAX` / `FILTER_TAU` values** are set at implementation from the
  offline replay and refined on-car (do not over-filter — the look-ahead, not
  the filter, provides loop damping; excessive `FILTER_TAU` adds lag).
- **Lane-line noise itself.** The driver-side line has its own noise/dropout;
  the `prob>0.6` gate + fade + `EXCESS_MAX` clip + slow filter bound its effect,
  and MODEL fallback covers dropout.
- **MODEL-state noise (Phase 2 concern, not Phase 1).** With `DRIFT_M` gone,
  MODEL state would track raw model curvature. Must confirm on data that
  no-clear-line scenarios are not noise-critical before Phase 2 deletes the
  floor. Phase 1 does not touch this.
- **Interaction with persist-gate (Phase 1).** The anchor reduces sustained
  position offset, which *shrinks* `de_dc` and keeps `persist_w` high — the two
  are compatible; the anchor does not fight the gate.
