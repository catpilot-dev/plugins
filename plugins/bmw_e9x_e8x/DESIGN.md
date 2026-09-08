# BMW E9x/E8x — Design & Implementation

A **car-type plugin**: it supplies a full openpilot car interface (CarState,
CarController, CarParams), a custom lateral controller, a Panda safety model,
and the UI touches (temperature overlay, Driving-panel vehicle items) for the BMW
E8x/E9x family. It carries its own DBC files and does not fork opendbc.

The lateral controller has its own canonical reference —
[LATERAL_CONTROLLER.md](LATERAL_CONTROLLER.md). This document does **not**
repeat it; it covers everything around it.

## How the car interface registers

There is **no `car_helpers` registration hook** (that hook was removed in the
0.11 rebase). Instead `register.py` **monkey-patches opendbc at plugin load
time**: `_register_interfaces()` runs at module-exec time (during
`registry.load_plugin()`, before `card.py` starts fingerprinting) and mutates
opendbc's global dicts in place:

- `opendbc.car.car_helpers.interfaces[BMW_E82 / BMW_E90] = CarInterface`
- `opendbc.car.fingerprints._FINGERPRINTS` / `FW_VERSIONS`
- `opendbc.car.fw_versions` globals (`FW_QUERY_CONFIGS`, `VERSIONS`,
  `MODEL_TO_BRAND`, `REQUESTS`)
- `opendbc.car.values.PLATFORMS`
- `opendbc.car.interfaces.get_torque_params` is **wrapped** to fold in the
  BMW rows from `torque_params.toml`

Because `card.py` holds references to the same dict objects, BMW becomes a
visible car. Disable the plugin and none of this runs, so BMW is simply not
in the system. `device.health_check` (`on_health_check`) reports `ok` only if
`BMW_E90` is present in the `interfaces` dict.

## Fingerprinting — VIN only

Fingerprints are **empty** (`fingerprints.py`: `{}` for both platforms) with
dummy FW entries, which forces opendbc's fuzzy path. `values.py::match_fw_to_car_fuzzy`
reads VIN positions 4–6 (`vin[3:6]`) as the BMW model code and maps it:

- `UF1 / UF2 / UH1` → `BMW_E82`
- `PH1 / PH2 / PK1 / PK2 / PM1 / PM2 / PN1` → `BMW_E90` (covers E90/E91/E92/E93)

Within a platform, `interface.py::_get_params` further reads the live CAN
fingerprint to detect **which cruise ECU** is present and set feature flags
(`BmwFlags`):

| Flag | Meaning | Detected from |
|---|---|---|
| `NORMAL_CRUISE_CONTROL` | NCC ($540) | `0x200` on PT-CAN |
| `DYNAMIC_CRUISE_CONTROL` | DCC ($544) | `0x193` on PT-CAN, or `0x194` stalk on F-CAN, and no LDM |
| `ACTIVE_CRUISE_CONTROL_NO_ACC` / `_NO_LDM` | ACC-module variants | LDM (`0x0D5`) presence |
| `STEPPER_SERVO_CAN` | Ocelot servo present | `0x22F` on SERVO/AUX-CAN |

Transmission type (auto vs manual) and a couple of steer-ratio tweaks are also
inferred from message presence. DCC and NCC both set `minEnableSpeed = 30 km/h`.

## Data flow

```
Panda ──CAN──► carstate.py (CarState.update)
                 parses PT-CAN / F-CAN / AUX-CAN → structs.CarState
                 publishes bmw_temps (0.2 Hz) on the plugin bus
                 resume-button state machine → ButtonEvents / speedlimit toggle
                        │
                        ▼
        controlsd  ──► LatControl (custom, controls.lat_controller_init)
                       ──► actuators.torque
                   ──► Longitudinal planner ──► actuators.speed
                       (overwritten to long_plan.vTarget by post_actuators)
                        │
                        ▼
              carcontroller.py (CarController.update)
                 torque → Ocelot STEERING_COMMAND (rate-limited)
                 v_target/accel → DCC cruise-stalk 0x194 bursts
                        │
                        ▼
                 Panda safety (bmw.h) — torque & rate limits, TX allow-list
```

### CAN bus layout

| Bus | Names in `values.py::CanBus` | Traffic |
|---|---|---|
| 0 | `PT_CAN` | engine, brakes, speed, yaw, transmission, cruise status/stalk, temps |
| 1 | `SERVO_CAN` / `F_CAN` | Ocelot stepper servo (steering); DCC cruise stalk when DCC is present |
| 2 | `AUX_CAN` / `K_CAN` | alternative servo bus; logging |

`get_can_parsers` deliberately subscribes to **both** DCC and NCC cruise
messages (with a `nan` timeout) so a slow-to-wake ECU can't cause `canValid`
failures from an unsubscribed message accessed later.

### CarState notes (`carstate.py`)

- Speed/yaw/temps/cruise are parsed from the plugin's own `bmw_e9x_e8x.dbc`.
- **Temperatures** aren't in the stock CarState schema, so coolant/oil are
  published on plugin-bus topic `bmw_temps` at 0.2 Hz for the UI overlay.
- **Resume-button repurposing** is implemented here (not in `ui_overlay.py`):
  a rising/falling-edge state machine times the hold. Long press
  (≥ `RESUME_LONG_PRESS_FRAMES` = 49 frames ≈ 500 ms) emits a
  `gapAdjustCruise` button event; short press while engaged sends
  `{'action': 'toggle_confirm'}` on plugin-bus topic `speedlimit_cmd_car`
  (and drains `speedLimitState`); short press while disengaged emits
  `resumeCruise`.
- A **steering-angle offset** can be supplied live on plugin-bus topic
  `steer_angle_offset`; it is persisted to the plugin data dir as
  `SteerAngleOffset` and subtracted from the reported steering angle.
- `is_metric` is auto-detected from the ratio of set-speed to vEgo the first
  time cruise is engaged above 5 m/s.

## DCC cruise-stalk control (`carcontroller.py`)

openpilot has no direct set-speed channel on this car, so longitudinal
control is done by **emulating cruise-stalk (0x194) presses** — `plus1`,
`plus5`, `minus1`, `minus5`, `cancel`. The controller compares
`actuators.speed` (= planner `vTarget`, injected by `post_actuators`) against
current speed and the DCC set-speed, and issues stalk pulses:

- **Command selection** — `plus5/minus5` vs `plus1/minus1` chosen by accel
  magnitude thresholds; decel is blocked below the cruise minimum + buffer.
  The **accel** side is gated by `V_ERROR_DEADZONE` (~0.5 km/h) plus accel sign
  and set-speed headroom. The **decel** side is gated by the setpoint deadzone
  instead — see *The setpoint is a torque request* below.
- **Cadence encodes magnitude** — DCC infers accel magnitude from press
  *rate*: `HOLD_INTERVAL` for large accel, `SINGLE_INTERVAL` otherwise. Note
  the constants are 25 ms / 50 ms but 25 ms is not representable on the 10 ms
  control grid: `dt_tx < interval - DT_CTRL / 2` puts HOLD's threshold exactly
  on a tick, so HOLD actually transmits at ~20 ms — measured **48 Hz on-car,
  not the 40 Hz the constant implies**. SINGLE is unaffected and measures
  20.1 Hz. The calibration table (PLUS1+HOLD ≈ +0.4 m/s², PLUS5+HOLD ≈ +1.2,
  MINUS1 ≈ −0.6, MINUS5 ≈ −1.2 m/s²) was measured against the real 48 Hz
  behaviour, so do not "correct" the cadence without re-measuring it.

### The setpoint is a torque request, not a speed

Measured over routes 452 + 453 (25.6 engaged min, 2026-09-05), DCC's response
is a clean linear function of the setpoint gap:

```
a_ego = 0.0935 * (setpoint - vEgo)   [km/h]     n = 21952, decel side
a_ego = 0.0912 * (setpoint - vEgo)               n = 103609, accel side
```

One symmetric constant, linear out to a −11 km/h gap, saturating near
−1.4 m/s². **10.7 km/h of gap buys 1 m/s².**

DCC's own speed loop is slow — implied horizon ≈ 2.8 s against the planner's
≈ 0.8 s — so a setpoint driven to `v_target` asks for a gap 1.8–2.9× too
shallow across the entire decel range. The measured closed loop was
`a_ego = 0.611 * a_cmd`: **61% of the deceleration openpilot asked for**, with
the best `a_cmd → a_ego` correlation at a 2.0 s lag.

The old `setpoint_error < 0` gate is a **clamp, not a rate limit**. It blocked
82–92% of the time below `a_cmd` −0.9, and the setpoint reached it in a median
of **0.00 s** — so no choice of `DECEL_STEP5_THRESHOLD` or
`DECEL_HOLD_THRESHOLD` could ever buy authority; step size and cadence only
control how fast you arrive at a clamp you are already sitting on. Matched on
demand, the only times the car braked properly were `minus5` overshoots that
punched through it:

| `a_cmd` | setpoint ≈ `v_target` | setpoint < `v_target` − 2 |
|---|---|---|
| −0.6 | gap −2.6 → **−0.33** | gap −5.8 → **−0.84** |
| −0.8 | gap −3.3 → **−0.27** | gap −7.9 → **−0.95** |
| −1.05 | gap −3.4 → **−0.35** | gap −9.2 → **−0.87** |

So the decel setpoint inverts the plant instead:

```
sp_target = min(v_target, vEgo + max(accel / K_DCC, -SETPOINT_BIAS_MAX))
```

`K_DCC = 0.1` is deliberately shallower than the measured 0.0935: it makes
every ask ~7% conservative, landing at a flat **92–93% of demand** rather than
overshooting wherever the plant is stiffer than measured. `SETPOINT_BIAS_MAX`
= 12 km/h caps the ask at −1.12 m/s²; above that the car under-brakes on
purpose and the driver finishes the stop.

Three things to keep straight:

- **`v_target` is not the driver's set speed.** It is `long_plan.vTarget`, the
  MPC trajectory speed at `action_t` (~0.5 s ahead), and it sits close to vEgo
  (median +1.1 km/h, p10 −2.1, p90 +3.7). The driver's set speed is
  `CS.out.vCruise`. Staying under it remains the planner's job — it caps
  `v_target` at `v_cruise` — exactly as before.
- **The new target is min'd against the old one**, so this law can only ever
  ask for a *deeper* setpoint than the pre-2026-09 code, never a shallower one.
- **The accel side is untouched.** `sp_target == v_target` whenever
  `accel >= 0`, so that branch is bit-identical.

**Restore.** The bias is a debt — a setpoint left low keeps braking, because
the plant is symmetric. As demand releases, `sp_target` rises back to
`v_target` on its own and a third branch walks the setpoint after it with
`plus1` at `SINGLE`. That keeps the setpoint ≤1–2 km/h above vEgo, so the
restore transient is **≤0.19 m/s²**, and costs 1.0 s at the p90 debt of
5.2 km/h (3.2 s at the worst observed 16.0). `plus5` would repay in 0.2 s but
park the setpoint ~12 km/h high on the way — a +1.1 m/s² lurch. This covers
the 96% of decel episodes that end normally; exits that stop us commanding
entirely (disengage, brake) are the debt ledger's job.

### The loop is blind for 200 ms — the decel bias is minus1 only

`0x193` reports the setpoint back at only **~5 Hz** (measured median gap
0.199 s / 0.131 s). That is the observation quantum: we command blind for up to
200 ms. It is **not** a limit on how fast DCC moves the setpoint, and it is not
SZL's 200 ms idle transmit period either — three unrelated 5 Hz clocks, easily
and wrongly collapsed into one.

Measured per command, over routes 452 / 453 / 44b:

| command | how DCC treats it | rate | blind-window commit |
|---|---|---|---|
| `minus1` | integrates **asserted duration** — R² 0.82 on held seconds vs 0.73 on frames sent, and rising edges explain only 0.20, so it is level-triggered and toggling gains nothing | ~4.6 km/h/s, flat across TX rate (4.76 steps/s at 15–25 Hz, 4.10 at 38–60 Hz) | 0.9 km/h = **0.09 m/s²** |
| `minus5` | accepted per frame, ~67% | up to 137 km/h/s at HOLD | 6.7 km/h = **0.63 m/s²** |

Simulating each option's slew limit against the real `a_cmd` traces:

| | delivered / demanded | sustains | builds a 9.1 km/h gap |
|---|---|---|---|
| old clamped law | 61% | — | — |
| **minus1 only** | **68%** | 1.28 m/s² | 2.0 s |
| minus1 + minus5 | 78% | 9.17 m/s² | 0.3 s |

So **the binding constraint is now setpoint slew, not the clamp.** Removing the
clamp moves 61% → 68%, not to the ~93% the static gain suggests; that figure
assumed the setpoint could *reach* `sp_target`, and it cannot.

`minus5` is parked, not deleted. It buys transient response, not ceiling —
minus1 alone already sustains 1.28 m/s², above `SETPOINT_BIAS_MAX`'s 1.12 — and
it costs 7× the blind-window exposure plus model risk on that 67% acceptance
figure. The table above is what to weigh if it is brought back.

Cadence is `HOLD`. DCC's minus1 step rate does not track our frame rate, so
HOLD costs ~2× the frames on the 0x194 counter-overwrite axis for no measured
gain in slew; it is taken for robustness of the assertion against dropped
frames, which the logs cannot settle either way. `DECEL_HOLD_THRESHOLD` and
`DECEL_STEP5_THRESHOLD` now live only on the `SetpointBias=0` rollback path.

### Route history

| route | law | gain | flips/min | tx/min | med gap | driver brakes/min |
|---|---|---|---|---|---|---|
| 452/453 | pre-bias | 69–71% | 4.3–4.8 | 212–308 | −1.9 | 0.22–0.25 |
| 454 | bias, bare sign test | **50%** | **19.3** | 724 | −1.9 | 0.20 |
| 455 | + concordance gate, per-slot selection | 67% | 13.0 | 420 | **−3.5** | **0.11** |

455 is the first drive where the bias actually established itself (median gap
−3.5 km/h) and the driver-brake rate halved. Flipping remains the open item and
is what `RESTORE_DEADZONE` targets.

### Concordance gate — two estimates must agree

Route 454 drove the bias off a bare `accel < 0` sign test and it chattered. The
setpoint changed direction **19.3 times a minute** against the old law's 4.6,
burned 2.75× the bus (724 vs 263 TX/min), and delivered **less** deceleration —
50% of demand against the old law's 69% — because minus1 and plus1 bursts
cancelled each other, 640 against 810 in half an hour. `a_cmd` crosses zero
22 times a minute, and every crossing swapped the target between `vEgo + bias`
and `v_target`.

Two estimates of the same intent are available, with largely independent noise:

| signal | what it is | noise vs a 2 s centred mean of `a_cmd` |
|---|---|---|
| `accel` | `actuators.accel` — LongControl's output, already filtered | **0.046 m/s²** |
| `a_dv` | the raw plan's `vTarget` differentiated over `DV_WINDOW` | 0.221 m/s² |

They disagree in sign on **18.8%** of samples yet agree ~100% once
`accel < −0.3`: they diverge where the noise is and converge where the demand
is real. The rule needs no thresholds:

```
both negative  -> brake
both positive  -> accelerate
disagreement   -> hold whatever state we are in
```

The hysteresis falls out of the disagreement region, which is exactly the band
where the noise lives — so it is self-sizing rather than tuned. Modelled on
454: flips **18.6 → 8.9/min**, commanding 1658 → 1487 moves/min.

`a_dv` is **only** the concordance check; the bias magnitude stays raw `accel`.
Using `min(accel, a_dv)` for the magnitude simulates better still (75% of
demand against 57%) but that is a deliberate brake-to-the-more-pessimistic-
estimate policy rather than noise rejection, and is deliberately not taken.

Two things that do **not** work, both tried on 454's trace:

- **A `v_error` deadband or concordance.** `v_error = vTarget − vEgo` carries
  our own braking back through vEgo, so gating decel on it is negative feedback
  on the quantity being sustained: gain goes to **−83%** even with a zero
  deadband, and the gap turns positive. It was safe under the old law only
  because the setpoint tracked `v_target` there, making `v_error` the genuine
  control error.
- **Filtering `a_cmd`.** The trouble is not fast noise — `a_cmd`'s residual
  against a 0.2 s low-pass is sd 0.037. It is the planner's own oscillation at
  0.1–0.3 m/s² over 0.5–2 s, which sits below any sensible filter corner, so a
  low-pass removes signal before it removes the wander (tau 1.0 s: flips 8.1
  but gain down to 49%).

### Command selection — one decision per SZL slot

What a burst is worth was measured **per burst**: neither cadence nor hold
length moves it. Over four routes a `minus1` burst drops the setpoint 1 km/h and
a `minus5` burst **10 km/h** (median; 2- and 3-frame bursts both land there, and
a 1-frame burst has n=1, so the 10 is not dialable down by shortening it).

```
err = (setpoint_observed - sp_target) - setpoint_pending      [km/h]
if   err >= DECEL_STEP5_KMH:   minus5
elif err >= SETPOINT_DEADZONE: minus1
else (err <= -deadzone):       plus1 via the restore branch
```

decided once per 200 ms slot and held for the rest of it, because an assertion
under 0.06 s produced no step 80% of the time while 0.10–0.15 s produced one
99% of the time.

**`setpoint_pending` is load-bearing.** 0x193 reports the setpoint back at ~5 Hz
and DCC's first step lands ~0.13 s after a burst starts, so without discounting
what is already asked for, a 10 km/h error draws `minus5` in two consecutive
slots and overshoots by a whole yield — measured in the bench as the setpoint
running to 54 km/h instead of 64 against a 74 km/h floor. `PENDING_TIMEOUT`
clears it after 0.5 s regardless: a DCC that has stopped acting on us never
changes the reading, so pending would never clear and the law would fall silent
for good.

Closed-loop bench over 53 real episodes from 452/453/454 — planner surrogate,
measured plant, 5 Hz observation — validated by `minus1`-only reproducing the
50% of demand actually measured on route 454:

| law | delivered | median overshoot |
|---|---|---|
| `minus1` only | 50% | 1.6 km/h (0.15 m/s²) |
| `minus5` at err ≥ 5 | **68%** | 3.5 km/h (0.33 m/s²) |
| `minus5` at err ≥ 8 | 58% | 1.7 km/h (0.16) |
| `minus5` at err ≥ 10 | 54% | 1.6 km/h (0.15) |

`DECEL_STEP5_KMH` = 10 matches minus5's yield, so it fires only with a whole
yield of room: overshoot-free by construction. That is a deliberate trade of
authority for smoothness, taken from the seat after route 455, where minus5 was
measurably the jerkiest thing the controller does — peak |d a_ego/dt| in the
0.7 s after a burst was **3.49 m/s³** median and **6.55** at p90, against
minus1's 2.08/3.90, plus1's 1.86/3.82 and a 0.71 baseline.

It is binary rather than a dial: `err` rarely exceeds 8 km/h once minus1 is
keeping up, so 8 and 10 both amount to switching minus5 off (bench 54% and 51%
of demand, against 68% at a threshold of 5, and 48% with minus5 gone). Rate
limiting instead was measured and does nothing — minus5 already fires about once
a minute, so a minimum gap of up to 3 s never binds. The cost is real: **68% →
51%**, below the 69% the pre-bias law measured. Judge it on the next drive by
the driver-brake rate, not this gain.

**Restoring is lazier than braking** (`RESTORE_DEADZONE` = 3 km/h against
`SETPOINT_DEADZONE` = 1). Route 455 still flipped the setpoint direction 13
times a minute against the old law's 4.5, and it is *not* the concordance latch
— a minimum dwell in the braking state changed nothing at all (68% / 7.3 flips
at every value from 0.3 to 2.0 s). It is the restore branch chasing v_target
back up between episodes: 769 plus1 bursts against 559 minus1 on that drive.
Widening only the upward deadzone takes flips 7.3 → 4.0/min at no cost in
delivered decel. It costs speed-return tracking (rms speed error 2.75 → 3.72),
so the car is slower to pick back up to the set speed.

Do **not** add lead compensation: `longitudinalActuatorDelay` is 0.7 for this
car, so the planner already computes vTarget/aTarget at that horizon.

**The debt ledger.** The restore branch only runs while openpilot is driving,
so every exit that stops us commanding parks the bias in DCC's setpoint memory:
the driver resumes expecting their set speed and gets one up to 12 km/h low,
braking into it. `setpoint_debt` is how many km/h the setpoint is still
below the point we borrowed it from, capped at `SETPOINT_BIAS_MAX`.

The hand-back point is `CS.out.vCruise`, **latched while openpilot is
driving**. It cannot be read live at repay time: `v_cruise` only tracks the
driver's stalk presses while openpilot is enabled
(`_update_v_cruise_non_pcm` returns early otherwise), so the value is
trustworthy only while we still have the car. It is range-checked 30–145 km/h,
the same way `register.py`'s cruise-ceiling memory does. The claim is dropped
entirely the moment the driver touches the stalk — their value stands, and
there is nothing live to reconcile against — and when `cruiseState.available`
goes away, since the setpoint memory goes with it.

Debt is **measured, not accounted**: `handback − cruiseState.speed`, capped.
An earlier version counted transmitted frames and was wrong — it pinned to the
cap after 4 frames (60 ms), because DCC only steps the setpoint once per 200 ms
slot, and the repay then declared itself settled after 0.6 s having actually
returned about 3 km/h. Reading the setpoint back is self-correcting: a step DCC
drops is still visible as debt and the repay simply continues. The tests model
DCC's one-step-per-slot behaviour for the same reason — a harness that holds
the setpoint fixed cannot see this class of bug at all.

The repay always lands **after a standby round-trip**, never during the
disengage itself: an openpilot disengage raises `cruise_cancel`, which outranks
the repay and takes DCC to standby first. The debt survives standby and is
settled when the driver brings DCC back — `CC.enabled` is still false at that
point, which is the state the repay branch is written for. It repays with
`plus1` at `SINGLE` for the same reason the restore branch does.

**Duty cycle is the risk to watch.** This raises decel commanding from 6.7% to
~47% of engaged time — 7× the 0x194 counter-overwrite exposure — while
direction flips stay flat (~4/min), so it is sustained commanding, not chatter
(an LPF on `accel` changes nothing). Every step is still +1 and every in-burst
slot is still overwritten, but sustained holds of this length are beyond
anything driven so far; the longest healthy hold on record is 6.28 s on route
44b. **Check the merged +1 rate on the first drive** — it should stay in the
94% band, not the 55% that flagged the slot law. `SetpointBias=0` rolls the
whole thing back on the car without a redeploy.

### `CruiseCadence` — debug A/B param (default off)

Pins the stalk cadence regardless of demanded accel, so a HOLD-vs-SINGLE
comparison can be driven. Read once at `CarController.__init__`, so it applies
from the next drive start.

```sh
ssh c3 'echo hold   > /data/plugins-runtime/bmw_e9x_e8x/data/CruiseCadence'  # pin 48 Hz
ssh c3 'echo single > /data/plugins-runtime/bmw_e9x_e8x/data/CruiseCadence'  # pin 20 Hz
ssh c3 'rm -f        /data/plugins-runtime/bmw_e9x_e8x/data/CruiseCadence'   # back to normal
```

It exists because openpilot picks the command **and** the cadence from the same
demanded accel, so the two cells can never be matched observationally — 46 decel
bursts over 25 segments left the question open (gap bin [0.5, 2) showed HOLD at
−0.284 m/s² vs SINGLE at −0.075, but two other bins were a tie or a slight
reversal, and the cells differed in speed by 24 km/h). Drive the same road once
pinned `hold` and once pinned `single`, then compare decel at matched setpoint
gaps.

Changes frame **spacing only** — counter steps stay +1, so it carries none of
the 5ECE exposure described below. Any unrecognised value falls back to normal
demand-driven behaviour.

### How DCC accepts 0x194 — read this before touching the counter

> **The rule.** DSC/DCC ignores the stock SZL frame when an emulated frame
> arrives a few milliseconds ahead of it. **Suppression is by TIMING, not by
> comparing counters.** The counter's only job is to increment by **exactly
> +1** on the stream that survives that timing filter.
>
> **Never emit a step other than +1 on 0x194.**

The stock SZL module emits its own 0x194 counter open-loop at 5 Hz (+1 per
200 ms slot). The controller injects a frame inside a `PRE_TICK_LEAD` (15 ms)
window at the end of each stock idle slot so ours lands first and stock's
arrives into the suppression shadow. `bmwcan.create_accel_command` builds the
frame with the special zero-initialised 0x194 checksum.

Because SZL's frames are only *sometimes* overwritten, our counter has to stay
+1 relative to whatever DCC last processed — which may be an SZL frame that got
through. That is what `cruise_burst_released` handles: after a pause long
enough for SZL to slip through, the next command resyncs from RX rather than
resuming our own sequence (see `carcontroller.py`).

**Correction, 2026-09-06.** This section previously stated that DCC accepts a
frame iff `(1 + M − K) mod 15 ∈ [1, 7]`, and that the counter-overwrite works by
keeping our counter inside a forward window. **That is not the mechanism**, and
building on it put DTC 5ECE + CD95 on the car twice (routes 44c seg 24, 450 seg
18): a change that advanced the counter 16 per slot instead of +1 per frame
dropped the merged stream's +1 rate from ~94% to ~55%, and DCC matured the fault
after ~95 s of engaged driving and latched cruise off. Reverted in `12a3f57`.

`cruise_burst_release_safe()` still carries the old `(1 + M − K) mod 15` test.
It is retained because it is what currently gates when the trailing `act=0`
overwrite stops, and that behaviour is field-proven — but its stated rationale
is wrong, and it should be revisited against the timing rule rather than
extended.

Evidence — merged 0x194 stream after timing suppression, fraction of +1 steps:

| route | build | +1 rate | outcome |
|-------|-------|---------|---------|
| 444 | pre resume-fix | 94.6% | no 5ECE |
| 44b | resume fix | 93.9% | no DTC, 11 segments |
| 44c | 16-per-slot counter | 55.9% | **5ECE + CD95** |
| 450 | 16-per-slot counter | 54.8% | **5ECE + CD95** |

## Steering command (`carcontroller.py` + `bmwcan.py`)

When `STEPPER_SERVO_CAN` is set and lateral is active, the torque fraction
from the lateral controller is scaled to Nm (`STEER_MAX = 12`), rate-limited
by `apply_dist_to_meas_limits` against the measured EPS torque
(`STEER_DELTA_UP/DOWN = 0.1 Nm/10 ms`), and sent as an Ocelot
`STEERING_COMMAND` in `TorqueControl` mode with an 8-bit checksum. On
disengage it issues `SoftOff` then `Off` frames. `bmwcan.SteeringModes` also
defines an `AngleControl` mode, but only torque control is used.

## Panda safety (`safety/bmw.h`)

Compiled C, safety model id **35** (`bmw`), declared in `plugin.json`'s
`cereal.safety_models`. Enforced independently on the Panda:

- **TX allow-list**: only `CruiseControlStalk` (0x194, PT-CAN and F-CAN) and
  `STEPPER_STEERING_COMMAND` (F-CAN / AUX-CAN).
- **Torque limits**: `TorqueMotorLimited`, max 12 Nm, **speed-scaled** down to
  8 Nm at 80 km/h and 4 Nm at 100 km/h; rate up ≤ 0.125 Nm/10 ms, rate down
  ≤ 1.0 Nm/10 ms (matches the flashed firmware; tightening to the controller's
  0.2 needs on-car validation of the disengage/SoftOff decay path first);
  RT delta 25 Nm/250 ms.
- **Firmware build/flash** (2026-08-14): `safety/build_firmware.sh` injects the
  plugin-owned `bmw.h` + `tests/test_bmw.py` into the firmware workspace
  (`~/openpilot`, dzid26 fork with the F4-capable `OxygenLiu/panda`; panda
  pinned at `a0848226`), runs the safety suite, builds
  `panda-bmw-fw/board/obj/panda.bin.signed`, and restores the workspace tree —
  no opendbc fork is maintained. Flash from the C3 with the stack stopped;
  note the device lib's `Panda.flash()` hangs on F4 re-enumeration and
  hardcodes H7 sector layout — call `Panda.flash_static(handle, code,
  mcu_type=McuType.F4)` with the panda already in bootstub instead.
- **RX checks** on brake, gas, speed and either cruise-status message, plus
  the stepper status.
- `disable_forwarding = true`; cruise-engaged state is taken from the DCC/NCC
  status messages.
- **LKA mode** (2026-08-14): DCC engaging latches `controls_allowed`; DCC
  dropping does NOT clear it (lateral continues while the driver owns
  gas/brake), and `brake_pressed` is deliberately not reported — brake means
  "drop to LKA", not "kill steering". Openpilot owns ALL disengagement
  semantics (two-stage cancel etc.); panda follows it down via the stock
  firmware heartbeat (`controls_allowed && !heartbeat_engaged` for 3 s clears
  `controls_allowed`; lost heartbeat → SILENT) and enforces the torque limits.
  See `.superpowers/sdd/2026-08-14-bmw-lka-mode/lka-mode-brief.md`.

## Hooks

From `plugin.json` — **eight** hooks:

| Hook | Function (module) | Purpose |
|---|---|---|
| `controls.lat_controller_init` | `on_lat_controller_init` (`bmw.latcontroller`) | installs the custom BMW lateral controller — see [LATERAL_CONTROLLER.md](LATERAL_CONTROLLER.md) |
| `controls.post_actuators` | `on_post_actuators` (`register`) | overwrites `actuators.speed` with `long_plan.vTarget` (time-aligned with aTarget) for the DCC v_error gate |
| `car.cruise_initialized` | `on_cruise_initialized` (`register`) | cruise-ceiling memory: restore `v_cruise_kph_last` (30–145 km/h) on re-engage |
| `ui.vehicle_settings` | `on_vehicle_settings` (`register`) | append the Temperature-Overlay toggle + Resume-Button note to the Driving panel's vehicle section (only when `CP.brand == 'bmw'`) |
| `ui.render_overlay` | `on_render_overlay` (`ui_overlay`) | draw coolant/oil temperature on the driving HUD |
| `device.health_check` | `on_health_check` (`register`) | report whether the BMW interface registered into opendbc |
| `selfdrived.events_filter` | `on_events_filter` (`lka_mode`) | LKA two-stage disengagement: strip brake/first-cancel disengage events so lateral survives DCC dropping; a cancel press that starts in LKA fully disengages; any definite gear other than Drive disengages directly (stock soft-disable only for `unknown` glitches) |
| `ui.state_tick` | `on_ui_state_tick` (`ui_overlay`) | LKA border shows the override grey (UIStatus.OVERRIDE, same as gasPressed while engaged) |

The `ui.vehicle_settings` hook is **dispatched by** the `ui_mod` plugin from
inside its Driving panel: when a car is detected, ui_mod draws a vehicle
heading (brand emblem from `logos/icons/bmw.png` via `CP.brand`, plus the
fingerprint) and runs `ui.vehicle_settings` to collect car-specific rows —
this plugin's `on_vehicle_settings` is the producer. These rows render within
the Driving panel.

## Configuration / params

Params are **files in the plugin's `data/` dir** (runtime:
`/data/plugins-runtime/bmw_e9x_e8x/data/`), read/written via `register.py`'s
`_read_param`/`_write_param`. Never `/data/params/d/`.

| param | default | live? | note |
|---|---|---|---|
| `TemperatureOverlay` | on | yes (read each frame) | coolant/oil temps on the HUD; Driving-panel toggle |
| `CruiseCeilingMemory` | on | yes (read on engage) | restore last set-speed ceiling on re-engage within a drive |
| `SteerAngleOffset` | 0.0 | yes (1 Hz) | persisted steering-angle zero offset; updated from the `steer_angle_offset` plugin-bus topic, **not** a user-facing toggle |
| `SetpointBias` | on | no (read at init) | accel-derived decel setpoint. `0` reverts to the pre-2026-09 `v_target` setpoint, v_error gate included — a true rollback, not a third behaviour |
| `CruiseCadence` | off | no (read at init) | debug A/B: `hold` / `single` pins the stalk cadence. Not for normal driving |

`torque_params.toml` (LAT_ACCEL_FACTOR / MAX_LAT_ACCEL_MEASURED / FRICTION per
platform) is folded into opendbc's torque params at load time. Lateral-timing
knobs (chiefly `steerActuatorDelay = 0.4`) live in `interface.py`; see
[LATERAL_CONTROLLER.md](LATERAL_CONTROLLER.md).

> Note: the old README listed a `ConsecutiveLaneChange` param — it does not
> exist in the code or manifest and has been dropped.

## Telemetry

Plugin-bus topics (recorded into rlogs because `install.sh` injects the plugin
cereal schemas):

| topic | rate | source | payload |
|---|---|---|---|
| `bmw_lat_control` | 20 Hz | `bmw/latcontroller.py` | full lateral-controller state — see LATERAL_CONTROLLER.md §9 |
| `bmw_temps` | 0.2 Hz | `carstate.py` | `coolant`, `oil` (°C) for the HUD overlay |
| `speedlimit_cmd_car` | on press | `carstate.py` | `{'action': 'toggle_confirm'}` to speedlimitd |
| `steer_angle_offset` | in (1 Hz) | consumed by `carstate.py` | live steering-angle offset from any publisher |
| `speedLimitState` | in | consumed by `carstate.py` | drained on resume-press to sync speed-limit state |

## Key files

```
bmw_e9x_e8x/
  plugin.json           # manifest: 6 hooks, 2 declared params, safety model bmw=35
  register.py           # monkey-patch registration + 4 hook callbacks + param IO
  ui_overlay.py         # temperature HUD overlay (ui.render_overlay)
  torque_params.toml    # per-platform lateral torque params
  bmw/
    values.py           # platforms, VIN detection, flags, CAN bus map, DBC map
    fingerprints.py     # empty fingerprints + dummy FW (forces VIN fuzzy match)
    interface.py        # CarInterface._get_params — flags, cruise type, delays
    carstate.py         # CAN parsing, resume-button SM, temps, offset
    carcontroller.py    # DCC 0x194 stalk emulation + Ocelot steering
    bmwcan.py           # CAN message builders + checksums
    latcontroller.py    # custom lateral controller (see LATERAL_CONTROLLER.md)
  dbc/
    bmw_e9x_e8x.dbc     # PT-CAN / F-CAN definitions
    ocelot_controls.dbc # stepper-servo definitions
  safety/bmw.h          # Panda safety model (C, safety id 35)
  tests/                # unit + replay/analysis scripts
```

## Known issues / notes

- **`lagd` never converges** for this car (it correlates on
  `latcontrol_torque` telemetry the custom controller doesn't produce), so
  `liveDelay.lateralDelay` is permanently pinned at `steerActuatorDelay + 0.2`.
  `steerActuatorDelay` is the effective single timing knob — details in
  LATERAL_CONTROLLER.md §2.
- **DCC is the exercised cruise path.** NCC and the ACC-module variants are
  recognized and wired but far less tested; the `Footnote` strings in
  `values.py` flag this.
- **Hardware-dependent.** Steering needs the Ocelot stepper servo
  (`STEPPER_SERVO_CAN` flag); without it, only longitudinal/DCC is available.
