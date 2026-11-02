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
κ_ref = LPF(κ_src, τ = kappa_filter_tau)           lane change: κ_ref = raw input (after the onset hand-over)
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

## Lane-change onset hand-over (2026-11-02)

During a lane change (`laneChangeStarting` / `laneChangeFinishing`, read from
`modelV2.meta`) the reference is the model's raw curvature — but only once
that curvature has turned toward `laneChangeDirection`: raw must sit
`LC_HANDOVER_K` (0.2e-3 1/m) past the held Arc reference on the lane-change
side, or `LC_HANDOVER_MAX_S` (1.0 s) passes. Until then the Arc stays on.
`preLaneChange` is plain lane keeping (no desire yet), so the Arc stays on
there too. Without `meta` the hook's `lane_changing` flag switches to raw at
once, as before.

Why: the driver's nudge yaws the car toward the target lane, so for a tick or
two after the start the pre-desire model plans a correction away from the
change. The instant raw switch exposed that spike (513 seg 2: −0.64e-3 vs the
Arc's −0.17e-3 on a right change), and a BMW decision landing in that 0.1 s
stepped the 1.2 Nm lane-change torque cap the wrong way: 32 of 92 lane changes
on 510–513 moved ≥ 0.5 Nm the wrong way at the onset, the wheel p50 2.4° /
p90 7.6°. Replay over 50f–513 (109 lane changes): hand-over p50 0.06 s after
the start (max 0.07 s), onset wrong-way reference dips ≥ 0.3e-3 53 % → 19 %
(the rest is the road's own curve; random non-LC moments: 7 %), raw passes
bit-identically after the hand-over.

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
`prob_l`, `prob_r`; `lc_hold` (true while the lane-change onset hand-over
holds the Arc).

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
