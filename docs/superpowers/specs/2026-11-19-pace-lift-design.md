# Pace lift: gap-driven lift on the e2e accel ask, tied to personality

Date: 2026-11-19. Status: V8 built; Director gate 1 and gate 2 APPROVE; on-car drive pending (branch pace-lift).

## Problem
On the BMW E90 (always experimental/e2e, no stock FCW, `radarUnavailable = True`, so leads are
vision-only), the car runs well under the speed target and catches up slowly, on free road and
behind a lead. Evidence (routes 53b/53e/545/546, catpilot 0.11.2, plugins f571a02 on 545/546):
- `longitudinal_planner.py` takes `min(mpc, a_cruise, e2e)`. e2e wins 99–100 % of the time on free
  road and 72–80 % behind a lead. The e2e ask (`modelV2.action.desiredAcceleration`, after
  AccelBoost) is p50 +0.07…+0.13 m/s², and is at least 0.2 m/s² only 16–28 % of the time.
- Free-road gap to vCruiseOut is p50 14–20 km/h at about 80 km/h. Time more than 2 km/h under
  vCruiseOut is 47 s per free-road minute (546). About 60 % of that is vT itself low (model pace),
  which is this project's target. The other 40 % is DCC lagging vT, which the bmw session owns.
- Personality (Aggressive throughout) reaches only the lead MPC. The steady-lead time gap is
  1.2–1.5 s when the MPC wins and 2.0–3.2 s overall.
- Pulse test (19 driver gas pulses, free road, under the target): the model's raw ask does not rise
  after a pulse (+0.07 before, +0.08 at 1–4 s after, +0.04 at 4–10 s after), but the model keeps
  the speed gained (dv over the next 10 s p50 +0.2 km/h). A lift therefore gains what it pushes in,
  and the model does not undo it.
- Upstream plan-long `get_accel_boost` would add only 0.005–0.013 m/s² here (0.2 dead zone).

## Goal
When the model is not slowing for anything, lift the e2e ask in proportion to the gap to the
planner's speed target, with a strength set by personality. The car then reaches the target sooner
and, behind a lead, closes in to the MPC's personality gap plus a margin.

The speed target is the planner's `v_cruise` after the `planner.v_cruise` hook: speedlimitd's
vCruiseOut, or the driver's max set speed when speedlimitd is disabled or has no limit (user, 2026-11-19).

## Scope
In: experimental mode, engaged, free road and behind a lead. A new catpilot planner hook and a new
plugin `pace_lift`.
Out: chill mode; the DCC setpoint law and 0x194 machinery (bmw session); speedlimitd's target (the
lift never changes v_cruise); the MPC; upstream ports; a pulse-train variant (a later option if the
continuous lift feels wrong).

## Requirements
R1 Hook. catpilot `longitudinal_planner.update`, inside the existing experimental-mode branch, after
  `accel_boost.update` and before the e2e candidate is appended, calls
  `hooks.run('planner.e2e_accel', a_e2e, v_ego, v_cruise, self.a_cruise, sm)`. The result is used only
  if `float(result)` succeeds and is finite; otherwise (and with no plugin, or on any plugin error) the
  pre-hook value is kept. The hook row is added to catpilot `DESIGN.md`'s hook table.
R2 Strength by personality: aggressive 0.40, standard 0.20, relaxed 0.0 m/s²; an unknown personality
  gives 0.0. Plugin params: `PaceLiftAggressive`, `PaceLiftStandard` (declared as string params,
  parsed as float, clamped 0–0.6, defaults above), `PaceLiftEnabled` (bool, default true). Bad or
  missing strength values fall back to the defaults. A param-reader failure other than a missing file
  counts as disabled, so `PaceLiftEnabled` off cannot silently stop working.
R3 Law. gap = v_cruise − v_ego (m/s).
  target = strength × interp(gap, [1.0, 4.0], [0, 1]) × interp(v_ego, [13.89, 16.67], [0, 1])
  (the lift fades in over 50–60 km/h).
R4 Hard resets: output = a_e2e on the same tick, and the lift state is set to 0. Gate strings:
  `disabled` (param off), `off` (not enabled or not experimental), `gas`, `brake`, `standstill`,
  `stop` (`modelV2.action.shouldStop`), `ask_decel` (a_e2e ≤ −0.10), `no_model` (model arrays shorter
  than 2 or any non-finite input among a_e2e, v_ego, v_cruise, a_cruise, model arrays or lead values).
R5 Ask cap: on every tick the lift STATE is clamped to ≤ strength × interp(a_e2e, [−0.10, 0.0], [0, 1]),
  so a model easing off reduces the lift at once, and recovery goes through the R8 rise rate (ask
  noise between −0.10 and 0 is not passed through at 5× gain).
R6 Soft gates: target = 0, and the lift falls at ≤ 1.0 m/s³. Checked in this order, first match reported:
  - `plan_decel`: min `modelV2.velocity.x` over t ≤ 6 s < v_ego − 2.0;
  - `curve`: max |velocity.x × orientationRate.z| over t ≤ 6 s ≥ 1.5 m/s²;
  - `lead_gap`: leadOne.present and dRel < T_FOLLOW × v_ego + 6.0 + 0.5 × v_ego (MPC steady gap +
    0.5 s), with T_FOLLOW aggressive 1.25, standard 1.45, relaxed 1.75 s. Hysteresis: once tripped it
    clears only when dRel ≥ T_FOLLOW × v_ego + 6.0 + 0.8 × v_ego;
  - `lead_closing`: leadOne.present and vRel < −0.5 m/s. Hysteresis: once tripped it clears only
    when vRel ≥ −0.2 m/s;
  - `none_needed`: the ramps alone give 0.
  Both lead latches reset whenever leadOne.present is False. ALL soft gates (`plan_decel`, `curve`,
  `lead_gap`, `lead_closing`) have a 2.0 s hold: once tripped, the target stays 0 until no soft gate
  has tripped for 2.0 s, so a flapping plan, curve or vision lead does not pump the lift. The hold
  never delays a hard reset (R4).
R7 Bounds: output = max(a_e2e, min(a_e2e + lift, a_cruise)). The lift never pushes above the cruise
  candidate and never lowers the ask. The planner's min() with the MPC is untouched.
R8 Rate: the lift rises at ≤ 0.25 m/s³. dt comes from a monotonic clock, clamped 0–0.2 s.
R9 Telemetry: every call publishes `paceLift` on the plugin bus (lands in pluginBusLog): aIn, aOut,
  lift, target, gap, strength, gate. Publishing never raises.
R10 Loading: `hook.py` loads `lift.py` by explicit file path (the `lane_keeping/register.py` pattern),
  never from sys.path. Params are re-read at most once per second.

R11 Driving panel toggle (user, 2026-11-19): `ui_mod/driving_panel.py` shows a "Pace Lift" toggle right
  after Personality, only when `_plugin_enabled('pace_lift')`. It reads `PaceLiftEnabled != '0'` and
  writes '1'/'0' via `write_plugin_param('pace_lift', 'PaceLiftEnabled', …)` (the Lane Centering
  pattern). Description: "In experimental mode, speeds up toward the speed target when the driving
  model is slower than it. Strength follows Driving Personality: strongest on Aggressive, off on
  Relaxed." The hook's 1 s param cache means a change takes effect within 1 s, also while driving:
  off → `disabled` hard reset, on → the lift rebuilds at the R8 rise rate.

## Verification
V1 Unit tests (plugins) for the law, every gate, the hard-reset invariant, the ask cap, the bounds,
  the rates, the soft-gate hold and lead hysteresis, the personality mapping, the params and loading the plugin the way the
  registry does. Plumbing tests (catpilot) including non-finite returns. All existing suites pass.
V2 Offline open-loop replay over rlogs 53b/53e/545/546, with a_e2e taken from
  `modelV2.action.desiredAcceleration` (never from aTarget):
  - per route, free road vs lead and by speed band (< 50, 50–70, > 70 km/h): lift p50/p90, active
    share, gate shares, gate flips per minute, the ask-cap share (ticks where R5 binds), and the
    share of active-lift samples with `allowThrottle` False;
  - invariants, asserted on the OUTPUT: output == a_e2e on every hard-reset tick; output ≤
    max(a_e2e, a_cruise) + 1e-6 and ≥ a_e2e everywhere. Asserted on the TARGET: target == 0 on every soft-gate tick.
    Expected violations: 0.
  If active<->not-active flips exceed 6 per minute on free road or 8 per minute behind a lead, or
  allowThrottle-False active lift exceeds 5 %, revise the gates before the drive. (Flips between two
  soft gates do not change the lift and are not counted.)
  Replay tuning (2026-11-19): the first law (plan margin 1.0 m/s, 1.0 s hold on the lead gates only)
  flipped 47–117 per minute on free road and 19–48 behind a lead, with the plan_decel gate chattering
  tick by tick. Variants were replayed on the same four routes. V8 (plan margin 2.0 m/s, 2.0 s hold on
  all soft gates, lead hysteresis as in R6) gives free-road active share 76 → 85 %, flips 2.2–4.5/min
  free road and 4.2–7.6/min behind a lead, added ripple at most +0.17 m/s³ (p95 of the change rate of
  the output minus that of the model's own ask) and 0 invariant violations. It was chosen. Continuous
  fades on plan and curve lowered the flips less and made the lift ripple more (reversals 140–280/min).
V3 On-car, one drive, after the user's GO and the bmw session's go-ahead. Baseline 545 + 546:
  - free road: time more than 2 km/h under vCruiseOut, at 47 s per free-road minute (546; recompute
    545) → target ≤ 30;
  - behind a lead: time with a time gap more than 1.0 s above the lead gate while under vCruiseOut,
    per lead-minute → target ≥ 30 % lower than baseline;
  - hard brakes 0.14–0.20/min → must stay ≤ 0.25/min; over-vCruiseOut time on flat cruise ≤ 0.5 %;
    bmw small-ask delivery > 0.71.
  Revert if any driver brake comes within 3 s after lift > 0.1 m/s² with a lead within 60 m, or more
  than 2 such brakes per hour on free road, or hard brakes exceed 0.30/min.

## Way back
From the car: the Driving panel's Pace Lift toggle, or Relaxed personality. On the device: a `.disabled` marker in
the plugin dir, or reset to catpilot 0200bc2c9 and plugins f571a02 (current release-v0.11.2 heads).

## Risks
- The model may be slow for reasons its plan does not show (pedestrians, narrow streets).
  Mitigations: the 50–60 km/h fade-in, R4–R6, Relaxed = off, gas/brake = instant off.
- The set speed as target (speedlimitd off or no limit): in the pulse sample the target was
  100–122 km/h at 45–70 km/h. Mitigations: the fade-in, the model gates, and 0.4 m/s² at most.
- Vision-only leads: objects the lead head does not flag are protected only by the ask cap and the
  plan gate.
- Efficacy: open-loop replay cannot predict closed-loop gain. The pulse test suggests the model keeps
  pushed-in speed.
- A higher aT changes the bmw session's DCC metrics: one session's work per drive.
- Two repos: the hook must reach `/data/openpilot` as well as the plugin reaching `/data/plugins`.
