# BMW E9x/E8x — Self-Calibrating Hold-Bias for Tight-Turn Steering Oscillation

**Date:** 2026-07-02
**Plugin:** `bmw_e9x_e8x` (`register.py` lateral controller)
**Status:** Design approved; awaiting implementation plan.
**Motivating evidence:** route `00000380--bc2a2ca510--6`.

---

## 1. Problem

On route 00000380 segment 6, in a sharp left turn (wheel angle −45° to −71°, v ≈ 9–10 m/s,
κ_des ≈ 0.015–0.020), the steering wheel visibly oscillates.

Measured characterization (rlog, 100 Hz `carState`):

- **Limit cycle**, not model noise: ~18° peak-to-peak, 3.1° RMS wheel wobble; `steeringRate`
  slams ±50–100 deg/s. Dominant frequency **~0.6 Hz, with 0 % energy above 4 Hz** → slow
  closed-loop limit cycle, not high-frequency stick-slip chatter.
- **Not κ_des wobble**: `desiredCurvature` is comparatively smooth through the turn (residual
  std 0.0007 1/m). The *angle* and *commanded torque* (`torqueState.output`, −0.35↔+0.57,
  sign-flipping ~1 Hz) oscillate; actual path curvature lags and sits **below** desired
  (persistent understeer). The loop rings around a smooth reference.
- **Triggered by angle/curvature, not speed**: oscillation onset at t≈7 s while v is still
  16 m/s (same speed as the quiet preceding straight); the wheel goes quiet on exit at
  17–21 m/s. Low speed alone does not cause it — entering the high-angle regime does.

## 2. Root cause

The controller (`register.py` plant-inversion micro-stepper) is **pure proportional in torque**:

```
target_nm = T_CAP_SLOPE_BASE · kappa_scale · v² · effective_err        # register.py:575
```

This is zero when on-target, so the only way it produces a *standing* holding torque is by
running a standing `delta_err` (the measured understeer bias). Near center this is free:
rack stiction (~0.5–1.5 Nm) holds the angle at zero torque, so the P-term rests at ~0 and
the deadzone action is `hold_zero` (target → 0, `register.py:562`).

At 45–60° the self-aligning torque (~1.8 Nm, measured — see §3) **exceeds breakaway
friction**, so stiction no longer holds. `hold_zero` lets the wheel drift back, `delta_err`
grows past tolerance, a `breakaway`/`ramp` push fires and overshoots (aggravated by the
0.4 s `steerActuatorDelay` ≈ 86° phase lag at 0.6 Hz, and by `kappa_scale = 3.0` at κ≈0.02
tripling the loop gain in exactly this regime). The result is the 0.6 Hz `hold_zero → drift →
breakaway → overshoot` limit cycle.

The documented design premise — *"BMW rack stiction holds δ at zero torque, so no additive
holding feedforward"* — is **correct near center and empirically false at high angle.** The
premise's validity is regime-dependent (holds where SAT < breakaway friction).

## 3. Direct magnitude measurement

At moments the wheel is briefly steady (`|steeringRate| < 3 deg/s`) in the turn, the commanded
torque ≈ the holding torque required against SAT:

| Regime | steady angle | holding torque (`torqueState.output` frac) |
|---|---|---|
| near center (`|ang|<5°`, straight) | ~0° | **−0.013** (≈ 0 — stiction holds) |
| turn `|ang|` 40–50° | ~45° | **+0.153** (≈ 1.8 Nm) |
| turn `|ang|` 50–60° | ~55° | **+0.04–0.08** |

58 % of steady high-angle samples require `|cmd| > 0.05`. Near center it is ≈ 0. This both
confirms the mechanism and sizes the fix: the needed hold is **~0.08–0.15 torque fraction**
(~1.0–1.8 Nm) in this regime.

## 4. Prior art — the reverted open-loop FF (do not repeat)

A stiction-gated, steering-angle-**linear** holding FF was deployed (`2c67a97`, 2026-05-20)
and **reverted** (`e81f1c0`, 2026-05-21). It over-pushed 50 % on a tighter/slower turn
(route 31d seg 8) because SAT is not linear in steering angle — it scales with v²·κ (dynamic
tire) plus caster-jacking (geometric), and pneumatic trail collapses near the friction limit.
A one-regime calibration extrapolated badly and supplied ~2.3 Nm where ~1 Nm was needed;
its fixed baseline also **kept the rack from relaxing during overshoot**, sustaining the ring.

**Lesson carried into this design:** the holding magnitude must not be an open-loop *model*
of SAT. It must be *measured/closed-loop* so it cannot exceed the true SAT, and it must
vanish during unwind so it never blocks a relax.

## 5. Design — self-calibrating hold-bias `b`

A bias `b` (torque fraction) rides underneath the existing P-term. Mechanically it is a
**slow, gated, anti-windup integral term** on `delta_err`. It absorbs the standing holding
effort so the P-term can rest near zero error → the limit cycle loses its driver.

### 5.1 Applied torque (ride-on-top)

In-regime, the decision-block target becomes:

- `hold_zero` branch (`register.py:562`): `target_frac = g·b`  (was `0.0`)
- `ramp` branch (`register.py:576`): `target_frac = plant_inversion + g·b`, then existing
  `t_cap` clip (`register.py:589`) applied to the sum.
- **Not applied** in `brake_zero`, `breakaway`, `cancel_tol`, `cancel_accel`, `cancel_jerk`.
  Those are transient safety/unwind actions; the bias must never fight a relax.

`g ∈ [0,1]` is the regime gate (§5.2); `b` is the integral state (§5.3).

### 5.2 Regime gate `g` (angle AND curvature both significant)

Soft blend (no hard hysteresis needed — `b` decays smoothly out of regime):

```
g_angle = clip((|CS.steeringAngleDeg| − HOLD_ANGLE_ON_DEG)
               / (HOLD_ANGLE_FULL_DEG − HOLD_ANGLE_ON_DEG), 0, 1)
g_kappa = clip((|state['desired']| − HOLD_KAPPA_ON)
               / (HOLD_KAPPA_FULL − HOLD_KAPPA_ON), 0, 1)
g = g_angle · g_kappa
```

Because the *magnitude* is self-calibrated (§5.3), the gate decides only **where** the bias
applies, not how much — so it can be generous without over-push risk. Out of regime `g→0`,
the applied bias fades, and `b` leaks to 0 (§5.4), restoring the documented
"stiction-holds-for-free" behavior on straights and gentle curves. `state['desired']` is
`κ_des` — the same signal `kappa_scale` keys on.

Starting thresholds (tunable):

```
HOLD_ANGLE_ON_DEG   = 20.0
HOLD_ANGLE_FULL_DEG = 35.0
HOLD_KAPPA_ON       = 0.006
HOLD_KAPPA_FULL     = 0.012
```

### 5.3 Integral law (self-calibration)

Per livePose tick (20 Hz), the bias integrates the front-wheel-angle error while the
controller is actively holding/tracking in-regime:

```
if learn_ok:
    b = clip(b + HOLD_KI · g · state['delta_err'], −HOLD_B_MAX, +HOLD_B_MAX)
```

- Sign convention matches the P-term (`target_nm` is `+` for `+delta_err`), so `b` builds
  holding torque in the turn direction.
- Standard integral action ⇒ **provably self-limiting**: if `b` ever exceeds SAT the wheel
  over-rotates, `delta_err` reverses, and `b` integrates back down. It converges to
  `b·g ≈ SAT` and cannot over-push (the structural fix vs. §4).
- `state['delta_err']` is the controller's existing (box-filtered) error; in curves
  (`|κ_des| ≥ KD_GATE = 0.002`) the filter is bypassed, so `b` sees the raw error — correct.

`learn_ok` (anti-windup gate) is true only when **all** hold:

- `g > 0`
- `state['action']` ∈ {`hold_zero`, `ramp`} (not a cancel/brake/breakaway/overshoot state)
- `not CS.steeringPressed` (driver override — §5.4)
- output not at the `t_cap` clip (back-calculation anti-windup)

Starting constants (tunable — `HOLD_KI` is the primary knob):

```
HOLD_KI     = 0.8     # rad⁻¹ per livePose tick (integral gain on delta_err)
HOLD_B_MAX  = 0.20    # clamp on |b| (torque fraction ≈ 2.4 Nm) — margin over measured ~0.15
```

### 5.4 Release / decay / driver override

```
if (g == 0) or CS.steeringPressed:
    b += HOLD_LEAK · (0 − b)          # τ ≈ 0.5 s
```

- On turn exit `g→0` first fades the *applied* bias, then `b` leaks out — no held bias into
  the next (possibly opposite) turn.
- On `steeringPressed`, learning freezes and `b` decays: the hold yields to the driver
  (route 380: driver grabs the wheel at t≈36 s).

```
HOLD_LEAK = 0.10      # per-tick leak toward 0 when out-of-regime / driver-pressed
```

### 5.5 Stability rationale

The integral is deliberately slow (design target τ ≈ 1–2 s, set by `HOLD_KI`) — its corner
(~0.1 Hz) sits well below the 0.6 Hz limit-cycle band, so it adds negligible loop gain/phase
there. It supplies the DC/holding component;
the fast P-term keeps the dynamics. By removing the standing error the P-term was chasing, it
*reduces* the P-term's excursions — damping the cycle rather than adding a new one.
`kappa_scale` is left untouched so the change is a single isolated mechanism and any
limit-cycle recurrence is attributable.

## 6. Insertion points (`plugins/bmw_e9x_e8x/register.py`)

- **Constants block** (~lines 254–356): add the `HOLD_*` constants near `T_CAP_*`/`FRICTION`.
- **State dict** (~line 366): add `'hold_bias': 0.0`.
- **Decision block** (lines 550–589): add `g·b` to `target_frac` in the `hold_zero` and
  `ramp` branches only.
- **livePose-tick tail**: on **every** livePose tick (not only decision ticks), compute `g`
  from the freshly-updated `delta_err`, then run the `learn_ok`/leak update to
  `state['hold_bias']`, placed after the decision block so `learn_ok` sees the current
  `state['action']`. (The applied `g·b` enters `target_frac` only at the cadence decision and
  is ramped between, consistent with how all targets are set.)
- **Telemetry** (lines 620–637): add `'hold_bias'` and `'gate'` to the `bmw_lat_control`
  payload.

## 7. Consistency with existing design decisions

- **`feedback_bmw_kappa_scale` ("no additive FF on κ_des")** — respected. That rule forbids an
  open-loop *model* term; `b` is closed-loop integral feedback, zero wherever the rule's
  premise (stiction holds) is true, nonzero only where it is empirically false.
- **`feedback_controller_layering` ("position feedback stays in lane_centering")** — respected.
  `b` observes `delta_err` (angle/torque domain), not lateral offset. Single job: hold δ
  against SAT.
- **`feedback_bmw_steering_stiction`** — this design *is* the "an integrator is needed to
  unwind against the sticky plant" recommendation, made regime-gated so it doesn't disturb
  the near-center stiction-hold.

## 8. Validation plan

1. **Unit tests** (offline, mock `CS`/`livePose`, per the plugin test pattern): integral
   convergence to a steady `b`, `B_MAX` clamp, gate blend endpoints, freeze-on-cancel,
   driver-yield decay, out-of-regime leak, sign correctness.
2. **Offline trajectory replay** on route 380 seg 6 logged inputs: confirm `b` converges to
   ~0.08–0.15 through the turn and decays to 0 on the straights. Validates gating/magnitude
   only — closed-loop plant response is not in the log.
3. **On-car** (deploy to C3): watch specifically for the §4 failure signatures — over-rotation,
   sustained `cancel_jerk` — plus: oscillation gone, understeer bias reduced, straights
   unaffected, driver override clean.

## 9. Tuning recipe

- *Oscillation persists / hold too weak*: raise `HOLD_KI` (faster convergence) or lower
  `HOLD_ANGLE_ON_DEG`/`HOLD_KAPPA_ON` (engage earlier).
- *Any over-rotation into the turn*: lower `HOLD_B_MAX` first (hard cap), then `HOLD_KI`.
- *Bias lingers after the turn / feels dragged*: raise `HOLD_LEAK`, widen the gate-off.
- *Fires in parking-lot maneuvers*: add a `v` term to the gate (bias self-calibrates small at
  low speed, so low risk, but available if felt).

## 10. Risks / watch-items

- **Tight S-curves with no straight between lobes**: `g` may not fully drop to 0 between
  opposite turns, so `b` must re-converge to the new sign via integral action (self-corrects,
  with a brief `cancel` at the reversal freezing then resuming). Verify on an S-curve route.
- **One-regime magnitude anchor**: `B_MAX` and gate thresholds are seeded from a single route;
  the self-calibrating law removes magnitude-extrapolation risk, but the gate thresholds still
  want confirmation across more turns.
- **Integral + delay interaction**: keep `HOLD_KI` conservative; if a *new* lower-frequency
  cycle appears, reduce `HOLD_KI` before anything else.

## 11. Open items

- Exact `HOLD_KI` / `HOLD_LEAK` values are starting estimates; final values from on-car tuning.
- Decide whether to add a low-speed (`v`) gate term now or defer to §9 if felt on-car.
