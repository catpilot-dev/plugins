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

## 2. The prediction — pure geometry, one frame, fixed distance

User-selected source: the lane line's geometry ahead (no time-differentiation
of a noisy series), evaluated at a **fixed distance** `PRED_DIST = 10 m`
(user decision 2026-07-23) — the classic look-ahead-error formulation
(`e + x_la·ψ`). The car's own commanded path is subtracted so a curving line
tracked by a curving car reads as zero drift:

```
y_line    = interp(PRED_DIST, laneLines[idx].x, laneLines[idx].y)  # line 10 m ahead
y_path    = -kappa_ref · PRED_DIST² / 2                            # where the car will be
                                                                   # (left-positive κ → −y in
                                                                   #  the +y=right device frame)
gap_pred  = line_sign · (y_line − y_path) − HALF_WIDTH
gap_pred_f = low_pass(gap_pred, tau=FILTER_TAU)
```

- **Why fixed distance beats a fixed time horizon:** 10 m is inside the
  model's most reliable near-field (a 2 s horizon at highway speed would read
  the line 40–60 m out, where it is noisiest); the correction geometry is
  speed-independent (same nudge per metre of road); and the path-compensation
  term is small and well-conditioned (`κ·50` vs `κ·1800` at 60 m). At urban
  speeds 10 m ≈ 1–2 s — the "couple of seconds" of the human model.
  Trade-off, accepted: at highway speed the time-anticipation is ~0.4 s;
  `PRED_DIST` is a param so replay/drive data can raise it if that lead
  proves too short.
- `kappa_ref` is the smoothed reference the anchor already computes — so the
  prediction *includes the correction already in flight*. Nudging reduces the
  predicted excess, which eases the nudge off as it takes effect: the same
  anti-overshoot feedback a human uses ("I've already turned in a little").
- Availability guard: needs `laneLines[idx].x/.y` to cover `PRED_DIST`; if
  not, fall back to `gap_pred = gap_filt` (the design degrades gracefully to
  the current-gap deadband).

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
| `LaneKeepPredDist` | 10.0 | fixed prediction distance ahead (m) |
| `LaneKeepGapHardLo` | 0.3 | current-gap floor below which prediction may not defer (m) |
| `LaneKeepGapHardHi` | 1.5 | current-gap ceiling above which prediction may not defer (m) |

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
   - **Prediction accuracy:** `gap_pred(t)` vs the realized gap once the car
     has travelled `PRED_DIST` (i.e. at `t + PRED_DIST/v`) — error stats; must
     beat the trivial predictor (`gap(t)` itself). Also report whether the
     ~0.4 s highway-speed lead is sufficient (informs raising `PRED_DIST`).
   - **Decision quality:** around 3bf's band exits, nudge onset must LEAD the
     current-gap design; during recoveries, nudging must cease earlier; quiet
     segments must stay quiet (no added churn from prediction noise).
3. **On-device probe additions** for the new branches, then the single deploy
   (merge `phase2_simplify` → `dev`) and the combined on-car gate from the
   Phase 2 spec §6, now also watching `gap_pred` occupancy vs band.

## 6. Non-goals

- No change to nudge magnitude/shape (pure-pursuit + caps + rate limit stay).
- No change to the Phase-2 tracker.
- No multi-hypothesis prediction, no use of the model's planned lateral path
  beyond the already-smoothed `kappa_ref` (keeps the predictor decoupled from
  plan noise).
