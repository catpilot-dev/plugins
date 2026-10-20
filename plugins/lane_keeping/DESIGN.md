# Lateral Reference Conditioner — Design

Standalone plugin on `controls.curvature_correction` (100 Hz, inside
controlsd, upstream of the vehicle's lateral controller). It conditions the
model's curvature into the reference the controller tracks. It has one job
and no position feedback.

## Signal flow

```
modelV2 plan ──► k_ahead = max |plan curvature| within 2.5 s
             ──► H = arc_horizon(v, k_ahead)
             ──► κ_arc = 2y/(x²+y²) at the plan point H seconds ahead
κ_src = κ_arc (or the model's desiredCurvature when there is no usable plan)
κ_ref = LPF(κ_src, τ = kappa_filter_tau)           lane change: κ_ref = raw input
hook returns κ_ref ──► vehicle lateral controller
```

## Arc look-ahead

`κ_arc` is the curvature of the circle from the car, tangent to its heading,
through the plan point `H` seconds ahead. It averages over one frame's path,
so it carries no time lag (routes 4d4/4d5: 30–40 % less 0.1–1 Hz wander than
`desiredCurvature` at 1.5–2.0 s, same timing). `position.y` shares
`desiredCurvature`'s sign in this stack (RIGHT-positive, GPS-verified on
route 4d4), so no sign conversion.

`H` on straights is `interp(v, [35, 55] km/h, [1.5, 2.5] s)`: longer removes
more straight-line wander at every speed (4d4–4dc, 75–95 km/h: arc/raw 0.61 at
1.5 s → 0.47 at 2.5 s) but softens S-curve legs and turns in early, worst at
low speed. The speed edges moved from 45/75 to 35/55 km/h after route 4eb
(2026-10-20): at 60 km/h the 2.1 s look-ahead passed 0.52 of the model's wobble
against 0.46 at 2.5 s. `k_ahead`, the max plan curvature within 2.5 s, blends
`H` down to 1.5 s over [1.5e-3, 3e-3] 1/m (straights p99 1.38e-3, steady curves
p10 4.1e-3), so curves are unaffected by the speed schedule. Aim points closer
than 5 m (crawl) are ill-conditioned: the model's curvature is used instead.

## Low-pass

`kappa_filter_tau` 0.15 s. Its group delay matches the field-verified 300 ms
box filter's 0.125 s; 0.3 s lagged and wobbled (route 395). The downstream
tracker is deadzone-free, which is why the reference is smoothed here rather
than tolerated there.

## Configuration

Params are files in `data/` (runtime `/data/plugins-runtime/lane_keeping/data/`),
read at process start.

| param | default | note |
|---|---|---|
| `LaneKeepKappaFilterTau` | 0.15 | reference low-pass (s) |
| `LaneKeepArcHorizon` | 0 (auto) | arc aim point, plan seconds ahead; > 0 forces a fixed value (A/B) |

## Telemetry

Topic `lane_keeping` on the plugin bus (recorded in rlogs), 100 Hz: `kappa_in`
(the model's curvature), `kappa_ref` (the reference returned), `kappa_arc`,
`arc_on`, `arc_h`, `k_ahead` (−1 when unavailable), `v_ego`, and the ego lane
geometry at the car for drive analysis: `line_l`, `line_r` (m, +y right),
`prob_l`, `prob_r`.

## Relation to the BMW controller

The BMW lateral controller (`bmw_e9x_e8x/bmw/latcontroller.py`) is a
deadzone-free tracker of this reference and patches itself in only when this
plugin is loaded (`HOOK_READY`); without it the stock openpilot lateral
controller runs.

## History

The driver-side lane-keeping position loop this plugin was named for (gap
measurement, predictive deadband, AC/DC damper, asymmetric gate) was retired
on 2026-10-20: the BMW controller's Direct Tracking law forced it off and it
stayed off. Its design and results are in `docs/superpowers/specs/`
(2026-07-22 driver-side-lane-keeping, 2026-07-23 predictive-deadband and
ac-stabilizer) and in git before that date. The reference conditioning
(2026-07-22 lateral-controller-simplification) and the arc (2026-10-12) are
what remains.
