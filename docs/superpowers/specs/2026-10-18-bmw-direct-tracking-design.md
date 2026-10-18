# BMW Lateral: Direct Tracking (no hold band) with Free-Play Take-Up

**Date:** 2026-10-18
**Component:** `plugins/bmw_e9x_e8x/bmw/latcontroller.py`, `plugins/bmw_e9x_e8x/register.py`
(Driving-panel row), `plugins/lane_keeping/register.py` (reference path in this mode)
**Status:** Draft — user answers folded in, pending user approval

## Problem

Route 4e5 (re-printed servo gears, loop drive on the same roads, segs 5–12 Arc
Smoothing only, segs 14–27 Arc + Lane Keeping). User: arc-only "swings back and
forth with slow correction riding on lines"; with Lane Keeping "the correction
was timely, then slightly overshoots, then slightly swings back and forth".

Measured (scratchpad `drift_attr.py`, `overshoot.py`, `lagchain.py`):

1. **Corrections start late — the hold band.** In the 2 s before each drift
   toward a line (|lane-centre offset| crossing 0.45 m), the raw model, the arc
   reference and the controller target pointed back to centre 100% of the time,
   but the request (+3.8e-4 1/m) was only 0.66× `HOLD_BAND_ENTER` (0.0015 rad
   front-wheel error). The controller sat in `hold_zero` 84% of the time at
   ~0 Nm while the car kept curving outward. Lane Keeping doubles the request
   (+7.4e-4, above the band), which is why it feels timely.
2. **Corrections end by letting go — the rest state.** When the error re-enters
   `HOLD_BAND` the controller drops torque to 0 (`hold_f = 0` on straights). This
   rack holds its angle at zero torque, so the wheel stays turned: over the next
   3 s the car kept curving the correction way 79% of the time while the
   reference asked for that only 51%; the wheel unwound only passively
   (−0.6° in 1 s). The opposite push came a median 4.8 s later → overshoot,
   then swing.
3. **Free play adds ~0.3 s per side change.** The servo↔column clearance is
   ~12–13° at column scale on 4e5 (contacts: servo−column gap ≈ +77.6…77.8° under
   right torque, +64.4…65.0° under left torque; controller sign, + = right — note
   `carOutput.actuatorsOutput.torque` carries the opposite sign). On a side change the servo
   sweeps it in ~0.1 s at only 0.13–0.25 Nm commanded (unloaded), then servo and
   wheel move 1:1. Confirmed against stock SZL 0x0C9 and DSC 0x0C4. The user
   rules the free play unavoidable in this mechanical setup (gear not slipping).

The hold band was sized for a design in which Lane Keeping performs the
position correction upstream (`latcontroller.py` docstring, Phase 2). With the
reference reduced to Arc Smoothing, sub-band corrections go unserved, and the
rest state makes every correction end passively.

## Intent

User's words: *"Lane Keeping + hold band shall go, so the model desired
curvature can be smoothed by arc smoothing, commands stepper faithfully."*

- Arc Smoothing alone shapes the reference; it is expected to absorb the model's
  sub-Hz wobble.
- The controller follows the reference continuously — no on-target rest state.
- It **actively steers back** when the reference says so; it never relies on rack
  stiction to hold or unwind (user: stiction is "too much unpredictable").
- On a change of push direction it **takes up the free play** quickly and with a
  small torque, then applies the desired torque.

## Goals

- Corrections start when the arc reference asks, not when the error has grown
  past a band.
- Corrections end with the wheel being steered back toward the reference, not
  released.
- The free-play crossing on a side change is quick, small-torque, and
  deliberate (never triggered by noise).
- Switchable against today's controller from the Driving panel for A/B drives.

## Non-goals

- Mechanical changes (user: free play unavoidable now).
- Changing Arc Smoothing's algorithm or look-ahead schedule.
- Changing torque limits, panda limits, or the safety layers listed under *What
  stays*. (LKA, sub-30 km/h, DOES change in Direct mode — user ruling below.)
- Deleting the old controller path — it remains the default and the A/B baseline.

## Anti-goals (failure even if it "works")

- **Rattle:** the servo flying across the 12° gap on noise sign-flips (audible
  clunk; the "slipping" noise the user heard is likely this). Gap crossings per
  minute must not rise above today's.
- Fast swings like route 4d9's Lane-Keeping-ON drive ("a few dangerous fast swing
  back and forth").
- Abrupt torque application — the STEP_MAX "human-style gradual torque" rule
  still holds for the *desired* torque (feedback memory: gradual torque).
- Running the deadzone-free tracker on the **raw** model curvature (documented
  unsafe in `lane_keeping/register.py`): this mode requires Arc Smoothing.

## Behaviour

### 1. Reference (in Direct Tracking mode)

- Arc Smoothing is forced ON and Lane Keeping's position correction is forced
  OFF while Direct Tracking is on, whatever their own Driving-panel switches say.
  Their switches keep working when Direct Tracking is off.
- No other change to `lane_keeping`'s smoothing (its 0.15 s LPF stays).

### 2. Tracking — no rest state

- No `at_rest`, no `hold_zero` / `hold_curve`, no entry/settle hysteresis, no
  persistent-lean escape, no `cancel_tol` in this mode. Every decision computes
  the P target from the full `delta_err` (today's plant-inversion law
  `τ = T_CAP_SLOPE_BASE · kappa_scale · v² · δ_err`, same `t_cap`, same cadence
  and spread).
- Small targets are commanded as they are, within one side. User ruling: a
  torque too small to move the column is, in effect, the same as holding — no
  band is needed to produce "hold". A change of SIGN is gated by the noise guard
  (§3), because ~0.5 Nm of noise torque at 25 m/s (1σ δ_err 0.00081 rad × v²)
  is enough to sweep the servo across the gap.
- **Steady torque in curves (user ruling):** keep the hold-floor — a same-direction
  push never commands less than `held_target` (`hold_f · torque`, sign-guarded,
  capped at `hold_cap`). `hold_f` is 0 on straights, so straights get the new
  behaviour; curves keep their proven hold. `hold_f`, `hold_cap` and `held_target`
  therefore stay in Direct mode (relax-dwell uses `hold_cap` too).
- **Speed range (user ruling):** Direct mode applies at ALL speeds, including the
  LKA regime below 30 km/h and the 30–43 km/h blend band. The LKA extras that
  ride the push path carry over unchanged: fast-unwind boost, dwell-length fade.
  The LKA-only cadence rule inside `cancel_tol` goes with `cancel_tol`.
- Active steer-back is the P law reversing when `δ_err` changes sign; the free-play
  take-up (below) makes that reversal land promptly.

### 3. Free-play take-up (on every change of push direction)

State per drive: which contact the servo is seated on, from the live servo−column
gap (`carState.steeringAngleOffsetDeg` = SZL − servo angle; servo angle =
firmware encoder ÷ 7.62, column-scale degrees).

- **Contacts are learned, not hard-coded.** Learn `C_right` / `C_left` as the
  median gap with |commanded torque| > 0.45 Nm, wheel nearly still
  (|steeringRateDeg| < 1), hands off, lat active; slow update (they drift with
  wear and re-prints: 4e2→4e5 the clearance grew 5.6°→11.6°). Persist the last
  values in the plugin data dir as the seed for the next drive. Until both
  contacts are known with enough samples, take-up is disabled (plain ramp, as
  today).
- **Noise guard (user agreed, gates the P law itself):** when the commanded torque
  would change sign, command 0 Nm instead until the new direction has persisted
  for the persistence window, judged on the per-update (20 Hz) `δ_err` sign — not
  on the ~300 ms cadence target. 0 Nm is a hold: below the detent-equivalent
  torque the servo does not move. A flip back to the old side within the window
  resumes the old side normally (counts in `tu_guarded`).
- **Seat when mid-gap (my call):** if the gap is not within tolerance of either
  contact, the seat is "unknown"; after the persistence window take-up runs
  toward the requested side as normal (the gap says how far there is to go).
- **Take-up:** command a quick, small torque in the new direction — just above
  the stepper detent torque at the column (user: detent 0.036 Nm motor ×
  7.62 ≈ 0.27 Nm; 4e5 data: sweep starts at 0.13–0.25 Nm). This bypasses
  STEP_MAX for the take-up amount only.
- **Contact signal (user agreed):** primary = gap within tolerance of the new
  contact; confirmation = stock SZL showing column motion in the new direction.
- **Sequencing (my call, builder check Q4):** old-side torque is shed under the
  normal STEP_MAX; take-up fires when the commanded torque reaches zero on the way
  to the new side, so the only un-capped jump is 0 → take-up torque. Take-up runs
  per CAN tick (out of cadence) — it is a small torque, the cadence governs the
  desired torque only.
- **Then** hand over to the normal P target, which ramps from the take-up level
  under the usual STEP_MAX.
- **Timeout:** if contact is not reached within a bound, hand over anyway (never
  stall the correction).

- **Speeds (my call):** take-up runs at all speeds — the free play is the same.

### 4. Switch

- New Driving-panel row in the BMW `on_vehicle_settings` hook: **"Direct
  Tracking"**, param `DirectTracking` in `/data/plugins-runtime/bmw_e9x_e8x/data/`,
  default **off** (absent file = off, same polarity rule as Stall Breakaway).
- Takes effect at the next lateral engagement (latActive false → true), so the
  user can A/B mid-drive by disengaging and re-engaging, and the controller never
  changes law while pushing.

### 5. Telemetry (bmw_lat_control, every publish)

Add: `direct` (mode on), `tu_state` (seated-right / seated-left / taking-up /
unknown), `c_right`, `c_left` (learned contacts), `tu_fires` (cumulative),
`tu_ms` (last take-up duration), `tu_timeouts`, `tu_guarded` (cumulative sign
flips suppressed by the noise guard). Existing fields keep their meaning; in
Direct mode `at_rest` is always false and `action` uses `ramp` / `take_up`.

## What stays (both modes)

Panda/STEER_MAX limits, `t_cap`, STEP_MAX schedule (for the desired torque),
stall-breakaway v2 (`sb_*`, StallBreakaway switch), driver-override observers
(`ov_*`), LKA mode and its blend band below 43 km/h, the LKA fast-unwind boost,
relax-dwell (curve-dip bridge — it rides the ramp path, not the rest state),
lane-change handling in `lane_keeping`, CONTROL_HORIZON_S cadence, telemetry, and
the hold-floor / `held_target` / `hold_cap` machinery (Direct mode keeps it for
curves). In Direct mode only the rest state (`at_rest`, hold_zero/hold_curve,
hysteresis, persistent-lean escape, `cancel_tol`) is bypassed.

## Constants — initial values (builder's call unless noted; all named, all in telemetry)

| Constant | Initial | Basis |
|---|---|---|
| Take-up torque | 0.35 Nm | > detent-equivalent 0.27 Nm; 4e5 sweeps at 0.13–0.25 |
| Contact tolerance | 1.5° | contacts stable ±1° within a drive (4e3/4e4/4e5) |
| Noise-guard persistence | 0.15 s (3 livePose ticks) | sign flips from noise are shorter; drift asks persist 2 s |
| SZL confirmation | 3 quanta (0.13°) | `ANGLE_QUANTUM_DEG` 0.04395 |
| Take-up timeout | 0.4 s | 4e5 crossing ~0.1 s; 0.3 s to contact median |
| Contact learn | min 200 samples per side, EMA τ ≈ 60 s | edges drift slowly |

## Success criteria (next drive, same loop roads as 4e5 where possible)

- Time > 0.3 m off the lane centre ≤ the 4e5 Lane-Keeping half (15–30% per road),
  with Lane Keeping off.
- Drift episodes (|offset| crossing 0.45 m): before the crossing, torque toward
  centre clearly non-zero (4e5 arc-only: ~0 Nm).
- After a correction: the opposite push (when the reference reverses) arrives in
  ≲ 1 s (4e5: 4–4.8 s median).
- Gap crossings per minute not above 4e5's; no audible rattle (user).
- User feel: no swing riding the lines; hands-off confidence on straights.

## Decisions log

- User: Lane Keeping + hold band go; arc smoothing smooths the reference; the
  controller follows faithfully and actively steers back.
- User: free-play take-up with a small quick torque, SZL as the "closed" indication;
  agreed: gap = primary contact signal, SZL = confirmation; agreed: noise guard.
- User: switchable from the Driving panel.
- User (builder check Q1): keep the hold-floor in curves.
- User (Q2): noise guard gates the P law's sign reversal (0 Nm until persistence).
- User (Q3): Direct mode at all speeds, including below 30 km/h.
- My calls: take-up sequencing (Q4), out-of-cadence take-up, 20 Hz persistence,
  unknown seat when mid-gap, take-up at all speeds, effect at next engagement,
  initial constants, telemetry fields, contact learning/persistence.
