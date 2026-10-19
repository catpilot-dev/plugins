# BMW Direct Tracking — straight-road slope bias (design)

Status: draft for user review, 2026-10-19. Builds on
`2026-10-18-bmw-direct-tracking-design.md`.

## Problem

Route 4e9 (Direct Tracking on) is the best lateral drive so far. The one
complaint is a slight right-line hug.

- On straights the car sits **+0.15 m right** of lane centre. The model's own
  plan 3 s ahead is +0.03 m, so the model is steering back to centre.
- With no torque, the column creeps right on every route since 4e2
  (−0.06..−0.16 °/s). Left pushes move it 0.15–0.6× as far as equal right
  pushes. Something outside the steering pulls the car right: road slope
  (device roll reads +1.4..+2.0° right-side-down), alignment or tyres.
- Direct Tracking is proportional only (torque = gain × error), so it can
  only make the ~0.08 Nm of standing left torque it needs from a standing error
  (δ_err mean −2.2e-4 rad on straights). That error is the offset.

Upstream's torque controller handles this with a roll feedforward, a learned
mounting offset (`latAccelOffset`) and an integrator. Our controller replaces
`lac.update` and has none of the three.

## Goal

Remove the standing error on straights so the car follows the model's plan.
Target: straight-road offset +0.15 m → ≤ +0.05 m, with no new oscillation.

Non-goals: changing curve behaviour, the take-up, the noise guard, or the
model's reference.

## Design

One slow, bounded torque bias `b` (Nm), added to Direct's P target:

    target_nm = T_CAP_SLOPE_BASE · kappa_scale · v² · δ_err + b

**Learning.** At 20 Hz, while the learn gate is open:

    b += (P_nm / TAU_BIAS) · dt,   then clamp b to ±BIAS_MAX

Here P_nm is the P term alone (without b). This is a plain integral of the
P term. When b supplies the standing torque, the standing error and P go to 0
and b stops moving.

| Constant | Value | Why |
|---|---|---|
| TAU_BIAS | 20 s | Far slower than corrections (0.1–1 Hz), so it only sees the DC part. Noise on b ≈ ±0.01 Nm. |
| BIAS_MAX | 0.25 Nm | Well below rack breakaway (~0.5 Nm), so b alone can never steer the car. First set to 0.15 (about 2× the 4e9 need of 0.08); raised after route 4ea, where roads with roll ≥ 2.3° needed 0.20–0.25. |

**Learn gate.** It depends only on the driving situation, never on the error's
sign (the hold-bias lesson: anti-windup must never block the unwind):

- Direct latched, active, not `steeringPressed`
- v > 40 km/h
- straight: |κ_desired| < 1e-3 and |κ_measured| < 1.5e-3
- no take-up in flight, no stall-breakaway block

When the gate is closed, b is held at its value and still applied. Road slope
doesn't stop at a bend, and 0.15 Nm is small next to curve torques.

**Lifetime.** b starts at 0 at each power-up. It keeps its value across
disengage/engage within a drive, and is never persisted (roads change). It
learns in about 1 minute of straight driving.

**Switch.** New Driving-panel row "Slope Compensation", default off. Active
only with Direct Tracking, and latched at engagement like Direct. Off leaves the
controller bit-identical.

**Telemetry.** `sl_bias` (Nm) and `sl_learn` (gate open).

## Roll feedforward: not in v1 (my call, reversible)

I suggested an upstream-style `−g·roll` feedforward. The 4e9 data argues
against it for now:

- Roll changes by ±0.4° between roads and did not track the pull (|r| ≤ 0.1).
  The feedforward would mostly add noise.
- The mounting part of roll is unknown: `torqued` is not valid for this car,
  and calibration assumes zero roll. A wrong offset becomes a constant wrong
  push.
- b already learns the total steady need, whatever its cause (slope, mount,
  alignment).

After the first drive I'll check whether b correlates with roll. If it does,
roll feedforward is the next step.

## Risks

- **b fights a deliberate off-centre plan** (e.g. the model shying from a
  truck). It is slow (20 s) and small (≤ 0.15 Nm), so it only removes the
  part that lasts tens of seconds.
- **Take-up and guard.** With a steady left b, the servo rests on the left
  contact more often; right corrections need P > |b| before the take-up fires.
  Watch take-ups/min.
- **Wrong b after a road change.** The clamp bounds it, and it re-learns
  within ~20 s of straight driving.

## Verification

- Unit tests:
  - b converges to a synthetic constant pull, in both signs;
  - clamp holds;
  - gate closed → b frozen in both directions;
  - press, take-up and sb_block close the gate;
  - switch off → bit-identical replay.
- First drive:
  - straight offset ≤ +0.05 m;
  - b settles about −0.05..−0.1 Nm;
  - sway ≤ 4e9's 0.046 m/s²;
  - take-ups/min about the same;
  - b vs roll correlation, to decide on roll feedforward.
