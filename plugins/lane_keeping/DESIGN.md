# Lane Keeping — Design & Implementation

Standalone plugin on `controls.curvature_correction` (runs inside controlsd at
100 Hz, before the lateral controller). Damps the sub-Hz lane wander of the
e2e driving model without ever fighting the model for position. A second, dormant hook
(`modeld.calib_bias`) belongs to the retired calibration trim (see below).

## Signal flow

```
modelV2 κ_des ──► κ_ref = LPF(κ, τ=0.15 s)      (reference conditioning, always on)
modelV2 lane line + plan ──► gap_pred ──► AC/DC split ──► κ_bias (bounded pursuit)
hook returns κ_ref + κ_bias ──► BMW lateral controller
```

## The damper, in one paragraph

The wheel-to-line gap on the driver side is measured (plan-based prediction
at `x = v · pred_delay_mult · lat_delay`, bias-cancelling by construction)
and split into DC and AC. The DC — where the model chooses to place the car —
is **conceded** via a slow tracker (`LaneKeepDcTau`, field-proven 5 s): the
model closes its own position loop through the camera with unbounded
authority, so every sustained output-side push loses (field result, routes
3c1/3c3/3c5). The AC — the wander around that line — is damped through a
bounded, rate-limited pure-pursuit curvature bias. Near the line an
**asymmetric gate** (`LaneKeepAsymGap`, 0.6 m) suppresses corrections
*toward* the line so recoveries are never opposed. The damper core is
width-independent (the constant half-width cancels in the DC/AC split); only
the asym threshold consumes `LaneKeepHalfWidth`, making the plugin
vehicle-agnostic in one parameter (plus `LaneKeepDriverSide`).

## Control law details

- **Gap measurement**: `gap = line_sign · laneLines[idx].y − half_width`
  (idx 1 = left / 2 = right; modelV2 device frame is +y = RIGHT, while
  desiredCurvature is LEFT-positive — two sign conventions, see the
  `line_sign`/`curv_sign` comment in `anchor.py`).
- **Predictive gap**: evaluated at `x_pred = clip(v · 1.5 · lat_delay, 5, 50)` m
  as `line(x_pred) − plan(x_pred)` — the line and the plan share the vision
  frame, so their coherent wander cancels in the difference (measured RMSE
  0.13–0.14 m, beating both a trivial predictor and a κ-path extrapolation).
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
  `κ = curv_sign · 2 · excess / lp²` with `lp = clip(v · t_preview, 1, lp_max)`.
  Uncapped, gain collapses as 1/v² — route 3e7 (82 km/h) measured the damper
  3.7× weaker than at the urban speeds it was field-tuned at, cancelling only
  ~30% of the model's wander. `lp_max = 25 m` (reached at 60 km/h) keeps
  urban behavior bit-identical and restores highway authority (×1.9 @ 82,
  ×2.8 @ 100 km/h).
- **Bounds**: hard cap `kappa_bias_max = 0.002` 1/m, slew
  `kappa_rate_max = 0.002` 1/m/s. Release on line loss / low confidence is
  through the same rate limit (no snap); lane change hard-zeros the bias.
- **Authority**: `interp(prob, [prob_on, prob_on + prob_fade], [0, 1])`,
  `prob_on = 0.5` (measured: [0.5, 0.6) gap noise equals the trusted band;
  below 0.5 is a real, route-inconsistent quality cliff — do not lower).
- **Hard floors** (`gap_hard_lo/hi`): absolute best-effort correction at
  extreme gaps. **Field-disabled** (±99): the floors' sustained push turned
  brief line touches into pinned stalemates (3c3: 34 s holds vs model-alone
  2.8 s). Code retained, params keep them off.

## Reference conditioning

The model curvature is low-passed (`kappa_filter_tau = 0.15 s`, frame-jitter
only — sub-Hz content deliberately passes; filtering it would add group delay
to curve entries; and NOTE the rule learned the hard way: compare *group
delay*, not box-window length vs τ). This smoothing runs **unconditionally**,
even with the toggle off — the BMW controller's deadzone-free tracker
depends on it (raw κ_des against a 0.001 rad band is the documented-unsafe
rollback). That is why the plugin is **enforced** (`.enforced` marker): full
removal (`.disabled`) is coupled to reverting the BMW lateral controller.
The Driving-panel toggle (`LaneKeepEnable`, live ~1 s) gates only the
position correction.

## Configuration

Params are files in `data/` (runtime: `/data/plugins-runtime/lane_keeping/data/`),
read at process start except the live toggle. Full list in `_load_config`
(`register.py`); the ones that matter in the field:

| param | field value | note |
|---|---|---|
| `LaneKeepEnable` | 1 | **live** (~1 s): the Driving-panel toggle; gates only the position correction, never the smoothing |
| `LaneKeepDriverSide` | left | China |
| `LaneKeepHalfWidth` | 0.91 | E90 half-width (m); one of only two vehicle-specific params |
| `LaneKeepDcTau` | 5 | concession time constant (s); code default 20 |
| `LaneKeepAsymGap` | 0.6 (default) | never-oppose-recovery threshold (m); 0 = symmetric |
| `LaneKeepGapHardLo` / `Hi` | **−99 / 99 (floors disabled)** | code defaults 0.3/1.5 — see hard floors above |
| `LaneKeepLpMax` | 25 | pure-pursuit aim-point cap (m); highway authority (route 3e7) |
| `LaneKeepAcDeadband` / `Hi` | 0.10 / 0.05 (defaults) | AC deadband taper over 54–90 km/h |

## Telemetry

Topic `lane_keeping` on the plugin bus (recorded in rlogs via
`customReserved1`), published at 100 Hz: `prob`, `line_y`, `gap`, `gap_filt`,
`gap_pred`, `gap_dc`, `excess`, `excess_ac`, `kappa_in`, `kappa_ref`,
`kappa_bias`, `authority`, `state` (`anchor`/`model`), `x_pred`, `v_ego`,
plus dormant `trim_*` fields. The ui_mod emblem ring reads `state` from this
topic.

## Design history

Specs in `docs/superpowers/specs/`, newest governs:
`2026-07-23-ac-stabilizer-design.md` (+ its 2026-07-27 asymmetric-damping
addendum) supersedes the predictive-deadband and 2026-07-22 positioner
specs. The arc — absolute band → predictive deadband → integral trim →
AC/DC stabilizer → floors removed → asymmetric gate → speed-dependent
authority (lp cap + deadband taper, route 3e7) — is traceable through the
supersession banners; the one-line summary is that every mechanism which
held an *opinion about position* was removed after losing to the model in
the field (fundamental finding, route 3c1: the e2e model counter-steers a
sustained bias to a stalemate and wins), and what remains is a pure damper.

## Calibration trim (retired)

`calib_trim.py` and its `modeld.calib_bias` reader remain in the tree but
are inert: `CalibTrimMode=0` by default, and the modeld-side call sites
were never deployed (archived on the catpilot `calib-trim-parked` branch,
2026-07-29). It was a perception-side DC lever designed to move the
model's chosen line by biasing the calibration yaw — built and reviewed,
then retired when the hard-floor removal dissolved the problem it
targeted. Design record: `2026-07-25-calibration-trim-design.md`.
