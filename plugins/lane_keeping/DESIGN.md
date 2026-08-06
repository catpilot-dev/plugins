# Lane Keeping — Design & Implementation

Standalone plugin on `controls.curvature_correction` (100 Hz, inside
controlsd, upstream of the vehicle's lateral controller). It does two jobs:
it **conditions the model's curvature reference** (a light low-pass, always
on), and it **damps the model's sub-Hz in-lane wander** — the ping-pong —
through a bounded position feedback that never fights the model for
position.

## Signal flow

```
modelV2 κ_des ──► κ_ref = LPF(κ, τ = kappa_filter_tau)          (always on)
modelV2 lane line + plan ──► gap_pred ──► AC/DC split ──► κ_bias (damper)
hook returns κ_ref + κ_bias ──► vehicle lateral controller
```

Two states: **anchor** (confident driver-side line, authority > 0 — the
damper is active) and **model** (no usable line, or a lane change in
progress — the bias releases to zero and the plugin is a smoothing
passthrough). The ui_mod emblem ring shows the anchor state.

## AC Damper

The wheel-to-line gap on the driver side is measured (plan-based prediction
at `x_pred`, bias-cancelling by construction — see below) and split into DC
and AC. The DC — where the model chooses to place the car — is **conceded**
via a slow tracker (`LaneKeepDcTau`, field-proven 5 s): the model closes its
own position loop through the camera with unbounded authority, so every
sustained output-side push loses (field-verified, repeatedly). The
AC — the wander around that line — is damped through a bounded, rate-limited
pure-pursuit curvature bias. Near the line an **asymmetric gate**
(`LaneKeepAsymGap`, 0.6 m) suppresses corrections *toward* the line so
recoveries are never opposed. The damper core is width-independent (the
constant half-width cancels in the DC/AC split); only the asym threshold
consumes `LaneKeepHalfWidth`, making the plugin vehicle-agnostic in one
parameter (plus `LaneKeepDriverSide`).

## Control law details

- **Gap measurement**: `gap = line_sign · laneLines[idx].y − half_width`
  (idx 1 = left / 2 = right). Two sign conventions are in play: the modelV2
  device frame is +y = RIGHT, while desiredCurvature is LEFT-positive — see
  the `line_sign`/`curv_sign` comment in `anchor.py`. Both the raw and the
  predicted gap are low-passed with `filter_tau` (0.7 s) against
  measurement noise.
- **Predictive gap**: evaluated at
  `x_pred = clip(v · pred_delay_mult · lat_delay, 5, 50)` m
  (`pred_delay_mult = 1.5`, sweep-measured optimum) as
  `line(x_pred) − plan(x_pred)`. The line and the plan share the vision
  frame, so their coherent wander cancels in the difference — measured RMSE
  0.13–0.14 m, beating both a trivial predictor and a κ-path extrapolation.
- **DC tracker**: EMA with `dc_tau`; seeds and adapts only from trusted
  measurements (authority > 0, not in a hard-floor excursion), freezes on
  line dropouts, resets on lane change and on deliberate disable.
- **AC deadband** (speed-dependent since 2026-08-06): the AC excess is
  deadbanded by `interp(v, [15, 25] m/s, [ac_deadband, ac_deadband_hi])` —
  0.10 m at urban speed, tightening to 0.05 m at/above 90 km/h ("higher
  speed, tighter deadband"). Safe because the lp cap (next bullet) pins the
  pursuit gain at exactly the speeds where the band tightens: a 0.05 m noise
  blip commands ~1.6e-4 κ ≈ 0.1 m/s² a_y.
- **Pure pursuit** (speed-dependent since 2026-08-06):
  `κ = curv_sign · 2 · excess / lp²` with `lp = clip(v · t_preview, 1, lp_max)`,
  scaled by the authority factor below. Uncapped, the gain collapses as
  1/v² — a highway field measurement at 82 km/h found the damper 3.7× weaker
  than at the urban speeds it was field-tuned at, cancelling only ~30% of
  the model's wander. `lp_max = 25 m` (reached at 60 km/h) keeps urban behavior
  bit-identical and restores highway authority (×1.9 @ 82, ×2.8 @ 100 km/h).
- **Authority**: `interp(prob, [prob_on, prob_on + prob_fade], [0, 1])` with
  `prob_on = 0.5`, `prob_fade = 0.1` (measured: gap noise in [0.5, 0.6)
  equals the trusted band; below 0.5 is a real, route-inconsistent quality
  cliff — do not lower).
- **Bounds**: hard cap `kappa_bias_max = 0.002` 1/m, slew
  `kappa_rate_max = 0.002` 1/m/s. Release on line loss or fading confidence
  goes through the same rate limit (no snap); a lane change hard-zeros the
  bias immediately and re-seeds the gap filters, so the new lane starts with
  no memory of the old one.
- **Hard floors** (`gap_hard_lo/hi`): absolute best-effort correction at
  extreme gaps. **Field-disabled** (±99): the floors' sustained push turned
  brief line touches into pinned stalemates (field A/B: 34 s holds vs 2.8 s
  with the model alone). Code retained, params keep them off.

## Reference conditioning

The model curvature is low-passed with `kappa_filter_tau = 0.15 s` — enough
to remove frame-to-frame jitter while the sub-Hz content deliberately
passes (filtering it there would add group delay to curve entries; the
damper handles it in the position domain instead). Rule learned the hard
way when tuning this value: compare *group delay*, never a box-filter
window length against a first-order τ.

This smoothing runs **unconditionally**, even with the toggle off — the
downstream lateral controller in this repo tracks the conditioned reference
with a deadzone-free band and would be unsafe against raw κ_des. That is why
the plugin is **enforced** (`.enforced` marker): full removal (`.disabled`)
is coupled to reverting the consuming controller. The Driving-panel toggle
(`LaneKeepEnable`, live ~1 s) gates only the position correction.

## Configuration

Params are files in `data/` (runtime:
`/data/plugins-runtime/lane_keeping/data/`), read at process start except
the live toggle. Full list in `_load_config` (`register.py`); the ones that
matter in the field:

| param | field value | note |
|---|---|---|
| `LaneKeepEnable` | 1 | **live** (~1 s): the Driving-panel toggle; gates only the position correction, never the smoothing |
| `LaneKeepDriverSide` | left | left-hand drive; the other vehicle-specific param besides half-width |
| `LaneKeepHalfWidth` | 0.91 | car half-width (m) |
| `LaneKeepDcTau` | 5 | concession time constant (s); code default 20 |
| `LaneKeepAsymGap` | 0.6 (default) | never-oppose-recovery threshold (m); 0 = symmetric |
| `LaneKeepGapHardLo` / `Hi` | **−99 / 99 (floors disabled)** | code defaults 0.3/1.5 — see hard floors above |
| `LaneKeepLpMax` | 25 | pure-pursuit aim-point cap (m); keeps damper authority at highway speed |
| `LaneKeepAcDeadband` / `Hi` | 0.10 / 0.05 (defaults) | AC deadband taper over 54–90 km/h |

## Porting to other vehicles

The plugin is vehicle-agnostic by construction: it reads modelV2, `v_ego`,
and `lat_delay`, writes a bounded bias onto `desiredCurvature`, keeps every
bound in physical units (1/m, m/s²), and its output still passes through
controlsd's ISO curvature clipping downstream. Both LHD and RHD sign paths
are implemented and tested. Checklist for a new platform:

1. **Faithful tracking (the one hard requirement).** The inner lateral
   controller must execute small `desiredCurvature` biases (≤ 0.002 1/m).
   Any deadzone or heavy reference filter in the inner loop silently eats
   the correction — a fat curvature-error tolerance once absorbed 44% of
   the damper's commands here. Stock angle control tracks near-1:1 (best
   case); stock torque control integrates small biases out through its
   measured-a_y feedback (good; friction compensation makes low-speed
   execution less crisp, but that is where the deadband is widest anyway).
2. **Set the two vehicle params**: `LaneKeepHalfWidth`,
   `LaneKeepDriverSide`.
3. **Decide the reference smoothing.** Stock controllers are tuned for raw
   κ_des and don't need `kappa_filter_tau`; it exists here because the
   consuming tracker is deadzone-free (see Reference conditioning). On a
   stock platform it is optional — and the `.enforced` coupling does not
   apply.
4. **Check `lat_delay` convergence.** The prediction horizon auto-scales
   from liveDelay; on most cars it converges and the predictive gap works
   as designed or better.
5. **Shake down on telemetry, not feel.** The `lane_keeping` topic carries
   anchored duty, bias amplitude, and — against measured curvature — the
   faithful-tracking check itself, before trusting a subjective verdict.
6. **Expect a tuning pass.** The structure ports; the deadband, `dc_tau`,
   and speed breakpoints were shaped on one car's wander spectrum and one
   region's road markings, so field values may want re-validation per
   platform.

## Telemetry

Topic `lane_keeping` on the plugin bus (recorded in rlogs via
`customReserved1`), published at 100 Hz: `prob`, `line_y`, `gap`, `gap_filt`,
`gap_pred`, `gap_dc`, `excess`, `excess_ac`, `kappa_in`, `kappa_ref`,
`kappa_bias`, `authority`, `state` (`anchor`/`model`), `x_pred`, `v_ego`.

## Design history

Specs in `docs/superpowers/specs/`, newest governs:
`2026-07-23-ac-stabilizer-design.md` (+ its 2026-07-27 asymmetric-damping
addendum) supersedes the predictive-deadband and 2026-07-22 positioner
specs. The arc — absolute band → predictive deadband → integral trim →
AC/DC stabilizer → floors removed → asymmetric gate → speed-dependent
authority (lp cap + deadband taper) — is traceable through the supersession
banners; the one-line summary is that every mechanism which held an
*opinion about position* was removed after losing to the model in the field
(the fundamental finding: the e2e model counter-steers a sustained bias to
a stalemate and wins), and what remains is a pure damper.
