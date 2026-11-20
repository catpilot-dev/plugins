# Model Centring (Arc + slow raw correction) — spec + plan

Date: 2026-11-20. Owner: PM (bmw session). Status: rev 3. Gate 1 REVISE x2 (rev 1: D3/D4/D1; rev 2: sweep2 reset artifact). Per the 2-round rule this goes to the user with the Director's open question (turn transients) and the rev 3 answer (bend gate, D4b).

## Goal
A lateral reference for action-head models (Cheddar now, 0.11.4's model next) that keeps the
Arc's smoothness in turns and the model's own lane centring:
`kappa_ref = kappa_arc + LP(kappa_raw - kappa_arc, tc)`, behind a new BMW-only Driving-panel
toggle "Model Centring" (default OFF). Today's POP + Arc behaviour is unchanged unless the toggle
is ON.

## Background (verified 2026-11-20)
- 549 (Cheddar + Arc): car +38 cm right of centre on straights, 3 unsignalled right departures,
  6 right-line touches in 22 quiet min. Cheddar's plan follows the car, so the Arc (built on the
  plan) drops the action head's centring: on straights raw - Arc = -34e-5 1/m (toward centre).
- 54a (Cheddar, Arc Smooth OFF = raw): offset +11.5 cm (548 POP + Arc: +12), 1 line touch; but
  steering rate rms +40-50 %, tracking err up, tight-turn over-asks (seg 7 R~40 m: raw spread
  37.6 vs Arc 29.6e-4; seg 28: raw peak 124 vs Arc 102e-4, then a reversal to -35 vs -31).
  User: "close to POP+Arc", busier steering, oversteer in seg 7 and seg 28; lane changes very
  smooth. Logged telemetry (pluginBusLog lane_keeping, 100 Hz) has raw and the would-be Arc.
- Offline sweep on 54a (scratchpad sweep.py), ratios vs the would-be Arc:
  raw 0.1-1 Hz 5.7x / 1-5 Hz 4.5x; 1st-order LP on raw tau 0.2: 5.3x / 0.8x, tau 0.5: 4.2x / 0.2x
  (LP fixes jitter only, not wander or turn overshoot). Hybrid tc 4 s: 1.65x / 1.00x, seg 28 peak
  110, centring kept 0.89; tc 8 s: 1.25x / 1.00x, peak 106, centring kept 0.83. LC onset timing
  unchanged (0.20-0.22 s vs Arc 0.20). ADVERSE (ungated): seg 28 exit -41e-4 (raw -35, Arc -31),
  lag vs raw 0.58 s (Arc 0.37). The rev 2 sweep claim was an ARTIFACT (sweep2.py reset the state on
  any >0.1 s timestamp gap; pluginBusLog timestamps come in 0.2 s batches -> term zeroed every
  0.2 s). Corrected (reset on latActive only), tc 4 / A_MAX 0.3 without a bend gate: <0.1 Hz 2.09x
  Arc, 0.1-1 Hz 1.65x, lag 0.58 s, seg 28 peak 110 / exit -41 -> fails T5 turn exit. Seg 28 runs at
  42-70 km/h, so the speed gate and the cap do not touch it.
  Rev 3, with the D4b bend gate (no charging above 0.3 m/s^2 road lateral accel, decay tau 0.5 s):
  seg 7 spread 29.6 (= Arc), seg 28 peak 102 / exit -31 (= Arc), 1-5 Hz 1.01x, lag 0.56 s,
  <0.1 Hz 2.06x / 0.1-1 Hz 1.65x Arc. Cross-drive context (straights 60-90 km/h, 1e-10 m^-2):
  <0.1 Hz: 54a would-be Arc 40, hybrid 93, 548 POP+Arc target 85, raw 208; 0.1-1 Hz: Arc 43,
  hybrid 72, 548 POP+Arc 24, raw 244 (54a raw drove with car sway only 1.17x 548). 549: delivers
  ~0.94 of raw-Arc on straights, cap binding ~2 % (Director rerun). Open-loop: indicative only.
- lane_keeping hook: `register.on_curvature_correction` (Arc in anchor.py; Arc Smooth toggle
  `data/LaneKeepArcSmoothTest`, only '0' = raw, read once per process; telemetry has
  kappa_in / kappa_arc_ref / kappa_ref / arc_bypass). BMW Direct needs HOOK_READY (ruling R10).
- Lateral frozen (user 2026-11-12), but this work is the user's request (2026-11-20, "go ahead").

## Scope
In: centring term in lane_keeping (register.py or anchor.py), toggle "Model Centring" (BMW-only
`ui.vehicle_settings` row, file `data/LaneKeepModelCentring`, only exact '1' = ON, read once per
process), time constant `data/LaneKeepCentringTau` (file only, default 4.0 s, clamped 1-20),
telemetry `centring_on`, `kappa_centring` (the LP term); docs; tests.
Out: any change to the Arc law, BMW controller, clip; a UI for tc; changes to POP defaults;
frame-time work (waits for 0.11.4's tinygrad).

## Decisions
- D1: hybrid, not a plain LP on raw (sweep: LP fixes 1-5 Hz jitter only). tc default 4 s; a
  longer tc is NOT more conservative (it costs phase margin in the closed centring loop); if the
  drive shows a slow weave, shorten tc (2 s) via the data file.
- D2: precedence: Arc Smooth OFF -> raw (as today, centring ignored). Arc ON + Centring OFF ->
  Arc (today). Arc ON + Centring ON -> hybrid.
- D3 (engagement): controlsd passes the MEASURED curvature to the hook when latActive is false
  (controlsd.py:128-130), and the hook has no latActive argument. The term uses
  `model_v2.action.desiredCurvature` as raw and runs only while engaged, detected as
  `curvature == model_v2.action.desiredCurvature` (exact: both are the same capnp float32 when
  latActive and no lateralManeuverPlan). Not engaged (or maneuver plan active) -> state reset to 0,
  term 0, so the first engaged tick outputs the Arc and the term builds from 0 with tc (no engage
  step). Fixed dt = DT_CTRL (0.01 s), like the Arc.
- D3b (lane changes): use the anchor's own `lc_active` (laneChangeStarting/Finishing, not
  preLaneChange; centring keeps working while the blinker waits). From onset the term fades to 0
  with a 0.5 s time constant and the state is reset to 0 at the end of the change (the new lane
  starts fresh). DESIGN's "raw bit-identical after hand-over" becomes: raw + a fading term for
  ~1.5 s (3 tau) after onset, then bit-identical. Rationale: no reference step at onset
  (gradual-torque rule); the Arc's hand-over step is unchanged.
- D4 (bound): state clamp (anti-windup), in lateral-acceleration units: |s| <= A_MAX / v^2 with
  A_MAX = 0.3 m/s^2 (549 needed ~0.13 m/s^2 steady; 0.15 binds 26 % and delivers only ~68 %).
  Speed gate on the output: 0 below 30 km/h, linear to full at 40 km/h (low-speed raw-Arc
  differences are the tight-turn over-asks, not centring).
- D4b (bend gate, rev 3): while |kappa_arc| * v^2 > 0.3 m/s^2 the state does not charge and decays
  to 0 with tau 0.5 s (turn over-asks never reach the reference; turn exits = Arc). tc file junk/NaN -> default 4 s,
  clamped 1-20 s; NaN/inf raw or Arc -> term reset to 0, output = Arc.
- D5: default OFF, BMW-only, file not declared in plugin.json (connect-on-device would expose it).

## Requirements
R1 Centring OFF (missing/any value but '1'): output bit-identical to today in both Arc modes.
R2 ON + Arc ON: output = Arc + LP term; with raw == Arc the output equals the Arc.
R3 LP matches a reference 1st-order filter within 1e-9 on a step and a sine (tc 4 s).
R4 Bounded by D4; bend gate per D4b (test: a scripted bend above 0.3 m/s^2 decays the term, a
straight charges it); NaN/inf inputs never propagate (fallback Arc).
R7 Engage transient: on the first engaged tick output == Arc; while not engaged the term is 0
and the state stays 0 (test: measured-curvature input != action -> output == Arc). R8 LC: from
onset the term decays with 0.5 s tau, is 0 when the change ends, and preLaneChange does not affect
it. R9 Clamp is on the state (|s| <= 0.3/v^2) and the 30-40 km/h gate on the output.
R5 Read once per process; telemetry keys present in all modes; HOOK_READY stays True.
R6 UI row BMW-only, initial state from the file, callback writes '1'/'0', lazy UI import;
non-BMW no row; manifest still declares no params.

## Tasks
| # | Task | Worker | Verification |
|---|---|---|---|
| T1 | Test-first implementation per R1-R9 on a branch off origin/dev 438bc7d; docs (lane_keeping DESIGN/README, BMW latcontroller comment if it names the reference sources). | engineer | lane_keeping + bmw suites pass; PM reruns full plugins suite |
| T2 | Offline replay check: run the new term code over 54a and 549 logs and reproduce the rev 3 numbers (bend-gated law, reset on latActive only) within 5 %. | PM | script output |
| T3 | Gate 2 Director. | Director | APPROVE |
| T4 | With the user's go: push, deploy, swap to Cheddar, Arc ON + Model Centring ON, user reboot, re-seal, verify files. | PM | on-device checks |
| T5 | Drive (user, supervised) on the 549/54a roads incl. segs 7 and 28. | user | criteria below |
| T6 | Close-out: back to the daily config (POP + Arc + Pace Lift, Centring OFF) unless the user keeps Cheddar. | PM | files + rlog |

## Drive pass criteria (T5), vs 548 (POP + Arc) and 54a (raw)
Straight offset within +-15 cm; 0 unsignalled line touches; slow weave (<0.1 Hz lane-centre
power) <= 1.2x 548 (longer tc costs phase margin: if a slow weave appears, SHORTEN tc, e.g. 2 s);
turn-exit reversals (seg 28 type) no larger than 548's / the would-be Arc's; steering rate rms <= 1.15x 548;
lane-centre sway <= 1.2x 548; seg 7 / seg 28 type turns: no curb-side or next-lane excursion
(inside offset in tight turns <= 548 + 10 cm); LC onset no worse than 54a; lash <= 5.5 deg.
Frame drops / locationd alerts are recorded but are not this change's criterion.

## Risks and cost
- Closed-loop centring dynamics are untested: a slow term can wind up on a long bend where the
  model and the Arc disagree. Mitigations: D3b fade in lane changes, D4 bound, D4b bend gate, supervised drive.
- Cheddar frame time stays marginal (4 soft-disable alerts on 54a); the drive will see them again.
- Cost: one engineer task, two short Director reviews, one supervised drive. No paid APIs.
