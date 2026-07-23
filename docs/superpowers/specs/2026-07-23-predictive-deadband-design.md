# Predictive Deadband — anchor decides on the *predicted* gap

**Date:** 2026-07-23
**Status:** design approved (conversation 2026-07-23), pending implementation
**Amends:** `2026-07-22-lateral-controller-simplification-design.md` (Phase 2, branch `phase2_simplify`)
**Deployment:** folded into the Phase 2 branch; single combined deploy (user decision)

## 1. The model (user-directed)

Mimic how a human drives: the driver *predicts* the drift with respect to the
driver-side lane line over the next couple of seconds. If the predicted gap
stays within the comfort band [0.6, 1.0 m], he just **holds the steering
wheel**. If it leaves the band, he **nudges slightly** to correct, then watches
again.

The current anchor tests the *present* gap. This change tests the *predicted*
gap. Everything else — pure-pursuit nudge magnitude, caps, rate limit,
authority fade, reference smoothing, the Phase-2 tracker underneath — is
unchanged.

What the prediction fixes (all observed on route 3bf):

- **Late catch-up** → early gentle correction: a drift toward the line is
  corrected while still in-band, when the needed nudge is smallest.
- **Fighting a recovery** → if the car is outside the band but coming back at
  an adequate rate, the predicted gap is in-band → stop nudging. Overshoot is
  prevented by construction, and "recovers slowly" self-calibrates: the nudge
  continues only while the recovery rate is insufficient.
- **Sub-Hz κ_des noise** → it integrates into slow drift, which appears in the
  *trajectory* toward the line a couple of seconds before it becomes position
  error. The predictor acts on exactly that signal.

## 2. The prediction — pure geometry, one frame, horizon = 2× lateral delay

User-selected source: the lane line's geometry ahead (no time-differentiation
of a noisy series), evaluated at the point the car reaches after
**`PRED_T = PRED_DELAY_MULT × lat_delay`** (user decision 2026-07-23:
mult = 2, BMW lat_delay ≈ 0.6 s → **≈ 1.2 s horizon**). The car's own
commanded path is subtracted so a curving line tracked by a curving car reads
as zero drift:

```
pred_t    = PRED_DELAY_MULT · lat_delay          # lat_delay live from the hook;
                                                 # fallback LAT_DELAY_FALLBACK=0.6 s
x_pred    = clip(v_ego · pred_t, X_PRED_MIN, X_PRED_MAX)       # ≈12 m urban, ≈30 m @90 km/h
y_line    = interp(x_pred, laneLines[idx].x, laneLines[idx].y)  # line at that point
y_plan    = interp(x_pred, position.x, position.y)              # THE MODEL'S OWN PLAN
gap_pred  = line_sign · (y_line − y_plan) − HALF_WIDTH
gap_pred_f = low_pass(gap_pred, tau=FILTER_TAU)
```

> **REVISED 2026-07-23 after the replay gate (user-approved).** The original
> design computed the car's future position by constant-curvature
> extrapolation (`y_path = −κ_ref·x_pred²/2`) and explicitly avoided the
> model's planned path "to stay decoupled from plan noise." **The gate proved
> that reasoning backwards.** The κ term multiplies κ_des sub-Hz noise by
> `x_pred²/2` (~450 m² at 30 m): measured predictor RMSE 0.33–0.55 m across
> routes 3bf/3b7/3bb — 2–6× *worse* than the trivial "gap doesn't change"
> predictor (0.15–0.17 m), degrading exactly as horizon². The plan-based
> difference measures **0.128–0.138 m, beating trivial on all three routes,
> bias ≤ 0.015 m** — because the plan and the lane line come from the same
> vision frame, so their coherent wander (the 2026-07-12 "plan-coherent
> dips" finding) cancels in the difference. It is also semantically exact:
> the Phase-2 tracker faithfully follows the plan, so line-minus-plan at
> `x_pred` *is* the predicted gap. Fallback extends to the plan: if
> `position.x/y` can't cover `x_pred`, `gap_pred = gap`.

- **Why 2× the lateral delay:** a correction commanded now takes ~one
  `lat_delay` to bend the car's path — predicting less than that ahead is
  reacting to what can no longer be affected. 2× = one delay for the nudge to
  act + one to observe the result before the next decision: the human
  observe-act rhythm, scaled to the plant. This also rejoins the controller's
  single-knob timing design: the horizon derives from the same liveDelay
  chain as the tracker's cadence, and adapts if the learned delay changes.
  The `curvature_correction` hook already passes `lat_delay` (unused until
  now); `LaneAnchor.update()` gains it as a parameter.
- `X_PRED_MIN = 5 m` (meaningful prediction at crawl), `X_PRED_MAX = 50 m`
  (stay inside the model's reliable line region).
- `kappa_ref` is the smoothed reference the anchor already computes — so the
  prediction *includes the correction already in flight*. Nudging reduces the
  predicted excess, which eases the nudge off as it takes effect: the same
  anti-overshoot feedback a human uses ("I've already turned in a little").
- Availability guard: needs `laneLines[idx].x/.y` to cover `x_pred`; if not,
  fall back to `gap_pred = gap_filt` (the design degrades gracefully to the
  current-gap deadband).

## 3. The decision

```
excess = deadband(gap_pred_f, [GAP_MIN, GAP_MAX])   # in-band -> hold (zero bias)
nudge  = pure_pursuit(excess)                       # unchanged from Phase 1/2
```

**Safety floor — prediction may defer, never mask.** A human doesn't trust
"it'll come back" when he is already on the paint. If the *current* filtered
gap is outside hard limits, the deadband acts on the current gap instead:

```
if gap_filt < GAP_HARD_LO (0.3 m)  or  gap_filt > GAP_HARD_HI (1.5 m):
    excess = deadband(gap_filt, [GAP_MIN, GAP_MAX])   # correct NOW, regardless
```

Low side: wheel within 0.3 m of the driver line. High side: 1.5 m gap on a
typical lane means the far-side wheel is approaching the opposite line (3bf
touched both sides).

## 4. Config & telemetry

New params (plugin `data/`, `AnchorConfig` fields):

| param | default | meaning |
|---|---|---|
| `LaneKeepPredDelayMult` | 2.0 | prediction horizon as a multiple of the live lateral delay |
| `LaneKeepGapHardLo` | 0.3 | current-gap floor below which prediction may not defer (m) |
| `LaneKeepGapHardHi` | 1.5 | current-gap ceiling above which prediction may not defer (m) |

Hard-coded: `LAT_DELAY_FALLBACK = 0.6 s` (used when the hook passes no
`lat_delay`), `X_PRED_MIN/MAX = 5/50 m`.

Telemetry gains `gap_pred` (filtered); `gap` / `gap_filt` remain, so replay
can compare the current-gap and predicted-gap decisions tick-by-tick — this
is the attribution mitigation for the single combined deploy.

## 5. Validation gate (before the combined deploy)

1. **Unit tests:** prediction geometry both driver sides (line/path sign
   conventions); straight-line equivalence (`gap_pred → gap` when line
   parallel and κ_ref = 0); curve compensation (curving line + matching κ_ref
   → no phantom excess); hard-floor override both sides; graceful fallback on
   short line arrays; in-band hold / out-of-band nudge on *predicted* gap.
2. **Replay (C3, non-activating), routes 3bf + 3b7 + 3bb:**
   - **Prediction accuracy:** `gap_pred(t)` vs the realized gap at
     `t + pred_t` (≈1.2 s later) — error stats; must beat the trivial
     predictor (`gap(t)` itself). Sweep `PRED_DELAY_MULT` ∈ {1.5, 2.0, 3.0}
     to confirm 2.0 is well-placed.
   - **Decision quality:** around 3bf's band exits, nudge onset must LEAD the
     current-gap design; during recoveries, nudging must cease earlier; quiet
     segments must stay quiet (no added churn from prediction noise).
3. **On-device probe additions** for the new branches, then the single deploy
   (merge `phase2_simplify` → `dev`) and the combined on-car gate from the
   Phase 2 spec §6, now also watching `gap_pred` occupancy vs band.

## 6. Non-goals

- No change to nudge magnitude/shape (pure-pursuit + caps + rate limit stay).
- No change to the Phase-2 tracker.
- No multi-hypothesis prediction. (~~No use of the model's planned lateral
  path~~ — REVERSED 2026-07-23 with gate evidence, see §2: the plan IS the
  path source now; the κ-extrapolation it replaced was the noisier option.)

## 7. Integral trim (added 2026-07-23 after route 3c0 — user: "the correction is too weak")

First on-car data measured classic P-only droop: the pure-pursuit nudge is
proportional to excess, so near the band edge it is tiny and equilibrates
against a persistent disturbance (the model shading left on narrow roads) —
gap below band 52% of ticks, episodes to 90 s with zero recovery, saturation
only 13%. A proportional outer loop cannot cancel a DC disturbance.

**`kappa_trim`** — a slow, bounded, signed integrator supplying the missing DC
authority, added to the output alongside the nudge:

```
out-of-band:  kappa_trim += trim_rate·DT·authority, toward curv_sign·excess
              (SIGNED: an opposite-side excess unwinds at the same rate —
               the hold-bias lesson: block only windup, never unwind)
in-band:      kappa_trim leaks to exactly 0 at trim_leak (5× slower)
line lost:    leak only — never integrate blind
lane change:  hard-zero (with the bias)
cap:          ±trim_max
```

| param | default | meaning |
|---|---|---|
| `LaneKeepTrimRate` | 1e-4 /m/s | out-of-band slew — full cap in 10 s |
| `LaneKeepTrimMax` | 1e-3 /m | hard cap; half the pursuit cap |
| `LaneKeepTrimLeak` | 2e-5 /m/s | in-band decay |

Telemetry gains `kappa_trim`. **Validation limits:** replay verifies the trim's
mechanics on the 3c0 telemetry (direction, ramp rate, cap, no oscillation);
the closed-loop displacement it wins back CANNOT be measured offline — the
disturbance response involves the model re-planning against the car's actual
position. The next drive is the gate: gap in-band % and left-touch % on
3c0-like roads. Curve-cutting (route 3c0's second finding) is deliberately
NOT addressed here — one change at a time; re-measure curves after the DC fix.
