# Phase 3 — AC Stabilizer: damp the wander, concede the line

**Date:** 2026-07-23
**Status:** design approved (conversation), pending implementation plan
**Amends:** `2026-07-23-predictive-deadband-design.md` (supersedes its position-preference
decision law and §7 integral trim)
**Baseline:** `lane_keeping` @ `fb03c65` (dev, deployed)

## 1. Motivation — what route 3c1 proved

Route 3c1 settled the architecture question empirically. With the anchor on,
the trim wound to its cap in the correct direction, and **+0.0022 1/m of
sustained leftward correction applied, the car did not move**: 35 above-band
episodes, max 62 s, zero recovery, right-line crossings to −0.83 m. The
worst-episode tick dump shows why — on a sustained right bend the model
commanded −0.0016…−0.0033 while we pushed +0.002; the net was ≈ 0, yet the car
tracked the bend on the model's chosen line. **The e2e model closes its own
position loop through the camera at unbounded curvature authority; it
escalates until reality matches its intent. A bounded additive correction in
sustained disagreement is absorbed as pure command-cancellation**
(corr(κ_in, correction) = −0.43). Injection point downstream of the model is
irrelevant: the model observes the car, not our software stage.

This reinterprets the whole position-preference arc: 3c0's droop was not
weak-P (doubling authority with the trim changed the steady state by nothing);
3bf's worse spread was transient fighting with nil steady gain; curve-cutting
is the model's deliberate intent, immune to the hook.

**User directive:** *"lane keeping shall remove AC with respect to the
driver-side lane anchor, but keep DC part."*

## 2. The design — gap stabilizer, not gap positioner

Decompose the car's motion *relative to the driver-side lane line*:

- **DC** — the steady margin, wherever the model puts it (narrow-road shading,
  curve apexing). **Conceded.** The model wins this channel structurally;
  fighting it buys nothing and adds noise.
- **AC** — wander, drift and oscillation *around* that line: the integrated
  form of the modelV2 sub-Hz noise this project has fought since route 380.
  **Removed.** This is the original "offload the noise to lane centering"
  directive in its purest form — position-domain noise damping against a
  real-world reference.

```
gap_dc     = slow_EMA(gap_pred_filt, tau = DC_TAU)   # adiabatically tracks the
                                                     # model's chosen line
excess_ac  = gap_pred_filt − gap_dc                  # the WANDER, not the position
excess     = deadband(excess_ac, ±AC_DEADBAND)       # ignore micro-noise
kappa_corr = pursuit(excess)                         # existing gain/caps/rate/authority
```

**Why the arm-wrestle dissolves structurally (not by tuning):** the correction
is zero-mean by construction. Any sustained disagreement decays out of
`excess_ac` with `DC_TAU` — the stabilizer forgets it, concedes the line, and
returns to zero output. There is nothing left for the model to escalate
against. The trim was this design's exact inverse (it *accumulated*
disagreement, guaranteeing the stalemate); the stabilizer *forgets* it.
Against the model's own zero-mean wander the two loops are cooperative — both
push toward the same slowly-moving reference. Curve-cut onsets read as AC for
a few seconds, so they are transiently *softened*, then conceded: resistance
without war.

### 2.1 DC-tracker state discipline

- Seeds from the first anchor-state sample of `gap_pred_filt`.
- **Freezes** when integration is untrusted: unavailable line, authority = 0
  (visible-but-untrusted), lane change.
- **Re-seeds** (with the gap filters) during a lane change — new lane, new
  line identity, new DC.
- Persists through brief dropouts (freeze, not reset), like the trim did.

### 2.2 Hard floors — unchanged, absolute

`gap_filt < GAP_HARD_LO (0.3 m)` or `> GAP_HARD_HI (1.5 m)` still overrides
with the ABSOLUTE deadband against `[GAP_MIN, GAP_MAX]` (as deployed today):
at the extremes we resist with everything we have, transiently, even knowing a
sustained disagreement is unwinnable. Best-effort protection, honestly scoped.

### 2.3 Deleted

- **`kappa_trim` and all its machinery** (`trim_rate/max/leak/accel_max`,
  params `LaneKeepTrim*`, telemetry `kappa_trim`, its tests/probes). Measured
  on 3c1: sustained authority achieves nothing against the model. The best fix
  removes code.
- **The absolute band `[GAP_MIN, GAP_MAX]` as the primary decision variable.**
  It remains only inside the hard-floor override (2.2), so `gap_min/gap_max`
  config stays.

### 2.4 Kept unchanged

Reference smoothing (`kappa_filter_tau` — orthogonal, proven); gap + plan
prediction machinery (now feeding the AC error: predictive deadband becomes
predictive *damping*); pursuit gain/caps/`kappa_rate_max`/authority fade
(`prob_on = 0.5`); lane-change hard-zero + filter re-seed; MODEL passthrough;
Driving-panel toggle + emblem ring semantics (`anchor` state = stabilizer
live); enforced-plugin coupling; all Phase-2 tracker behavior.

## 3. Config & telemetry

| param | default | meaning |
|---|---|---|
| `LaneKeepDcTau` | 20.0 | DC-tracker time constant (s) — the forgetting time |
| `LaneKeepAcDeadband` | 0.10 | AC excess ignored below this (m) |

Removed params: `LaneKeepTrimRate/Max/Leak/AccelMax` (stale data files are
harmless — simply no longer read).

Telemetry: `kappa_trim` removed; gains `gap_dc` and `excess_ac`. Everything
else unchanged (`gap`, `gap_filt`, `gap_pred`, `x_pred`, `kappa_bias`,
`kappa_in`, `kappa_ref`, `authority`, `state`, `prob`, `v_ego`).

## 4. Choosing `DC_TAU` — the one open knob

Too short → the DC tracker follows the wander and we damp nothing. Too long →
a slow arm-wrestle re-appears. The wander band is ≈ 0.03–0.16 Hz (6–30 s
periods, July σ-observer study); the model's deliberate line moves take
~2–10 s. `DC_TAU ∈ {10, 20, 30}` s swept in replay; default 20 s pending the
sweep. Selection metrics (replay, per route):

- **Sustained-correction residue** (the anti-arm-wrestle gate): on 3c1's
  stuck-above-band stretches — where the trim sat pinned at cap — the
  stabilizer's correction must average ≈ 0 (|mean| ≪ cap, e.g. < 0.0002 over
  any 30 s window).
- **Wander tracking**: fraction of gap AC variance (0.03–0.16 Hz band) that
  appears in `excess_ac` (the signal we can act on) — higher is better.
- **Quiet-route inertness**: on 3b7/3bb in-band stretches, correction
  occupancy must not exceed today's.

**Honest validation limit (as with the trim):** open-loop replay shows the
*decision signal* is right (zero-mean, correctly sized, quiet when it should
be); the closed-loop damping — does the wheel/gap actually wander less — is
only measurable on-car.

## 5. On-car gate (the drive after deploy)

| metric | target | reference |
|---|---|---|
| sustained correction residue on 3c1-like stretches | ≈ 0 (no pinned correction) | 3c1: trim pinned at cap 78% |
| gap AC amplitude (0.03–0.16 Hz band) | reduced vs 3c1/3c0 | first measured baseline |
| torque reversals / fight-noise | ≤ current | 3c0: 38.7/min |
| right/left touches | not worse than the model alone (toggle A/B) | 3c1 hands-off 26% right |
| toggle A/B on one road | stabilized stretch visibly calmer, same average line | user's seat |

The Driving-panel toggle makes the A/B trivially cheap: same road, anchor on
vs off — with the DC conceded, the *only* difference should be less wander.

## 6. Non-goals

- Fighting the model's line choice anywhere (that is the point).
- The HPF-model/DC-ownership hybrid (Phase-3 alternative — superseded by this
  simpler design; revisit only if AC damping proves insufficient on-car).
- Curve-cutting elimination (model intent; softened transiently at best;
  real fix is model-level — 0.11.2 yardstick).
- Any change to the Phase-2 tracker.

## Addendum 2026-07-27: Asymmetric damping near the driver-side line

Soak finding (routes 3ca/3cd vs 3c5): symmetric damping cannot distinguish
the model's ESCAPE from the line from wander — both are AC — so it
partially opposes recoveries, stretching model-alone 2.8 s touches to
~10 s (narrow-shoulder matched geometry: 12.6–15% line time vs
model-alone 7.1%). The floors' removal eliminated the sustained push;
this is the milder, symmetric-damping residual.

Rule (gap-space, side-agnostic): when `gap_filt < asym_gap` (new
AnchorConfig field, default 0.6 m, param `LaneKeepAsymGap`, 0 = disabled
→ exact prior behavior), the post-deadband excess is clamped to
`min(excess, 0)` — corrections toward the driver-side line are
suppressed; away-pushes (fast sag below the conceding DC) are kept.
Removes opposition only; introduces no NEW sustained bias. Scope of that
claim (review 2026-07-27): it holds for steps and stationary deviations,
NOT for sustained ramps — a deliberate line-crossing at rate r leaves the
first-order DC tracker lagging by r*dc_tau, producing a sustained
away-push (~60-77% of cap at r=0.1 m/s) for the crossing's duration.
That ramp-lag push is PRE-EXISTING damper behavior (identical under
symmetric damping; the gate only removes the toward-line half, inactive
during a sag) and is structurally the 3c1 arms-race mechanism in the
crossing direction — the on-car soak must watch for crossing-direction
stalemates (deep sustained negative gaps), which would indict the ramp
lag, not this gate. This is the only place besides the
(param-disabled) floors where the absolute gap — and therefore
half_width — enters the law; the damper core remains width-independent.

Prediction to verify in soak: narrow-shoulder line time 13% → ~7%,
worst hold ~10 s → ~3 s, churn unchanged.
