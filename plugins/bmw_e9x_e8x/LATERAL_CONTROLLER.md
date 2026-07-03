# BMW E9x/E8x Lateral Controller — Design & Tuning Reference

This document is the canonical reference for the lateral controller registered by `register.py::on_lat_controller_init` (hook: `controls.lat_controller_init`). It describes **what** the controller does, **why** it's shaped this way for the BMW hydraulic rack, **how** the single-knob timing design propagates, and **what was tried and rejected**. Future maintainers should be able to retune without re-litigating the failed experiments.

> **2026-07-03 — stiction special-casing removed.** All FRICTION-based command
> shaping is gone: the `breakaway` ±FRICTION amplification, the `brake_zero`
> one-shot reverse pulse, and the reverse-FRICTION pulses in `cancel_tol` and
> the ISO cancels (all drains now go to 0). Route 384 telemetry showed ~40% of
> in-turn decisions were friction pulses — ±0.6 Nm torque reversals that
> churned rather than corrected. The straight-line wobble those mechanisms
> were built to manage is modelV2 vision noise, mitigated by the κ-gated
> delta_err box filter (§4) instead. Sub-friction targets are now commanded
> as-is (soft actuation deadband); cancels relax to 0 and rely on tire
> aligning torque to unwind (at guard-firing angles it exceeds rack stiction —
> route 380/384 evidence). Sections below describing `breakaway`/`brake_zero`
> or reverse-FRICTION drains are historical where they conflict with this note.
> `FRICTION` remains only as the tolerance-cancel guard threshold.
>
> **Same date — curvature-dependent hold.** Inside the tolerance band the
> target is `hold_f·torque` instead of 0, with
> `hold_f = interp(|κ_des|, HOLD_KAPPA_BP=[0.004, 0.010], [0, 1])`. In a curve
> the torque that achieved on-target ≈ the self-aligning torque (which beats
> rack stiction above ~40° wheel — target-0 there was the driver of the
> route 380/384 0.6 Hz limit cycle). Straights (|κ_des| < 0.004, above the
> ±0.003 modelV2 wobble band) keep the pure stiction-hold. The held value
> re-derives from `state['torque']` each decision — bounded by the P-term's
> own command, no learning state, no ratchet (the failure mode of the
> reverted hold-bias integral). The tolerance-cancel drain targets the same
> `hold_f·torque` ("stop the ramp, keep what you have" in curves); ISO
> overshoot cancels still drain to 0.

---

## 1. Why a custom controller (not stock latcontrol_torque)

The BMW E9x/E8x uses an **AC Delco / Ocelot stepper servo on the hydraulic rack**, not an EPS motor. The plant has two properties that make stock torque controllers misbehave:

1. **High static stiction, no self-centering** — at zero applied torque the rack *holds its current angle* indefinitely. There is no aligning-torque-driven return-to-center. This is the opposite of most EPS racks, where the steering self-centers.
2. **Kinetic friction comparable to typical control torques** — small commands (sub-FRICTION) don't move the rack at all. The rack ignores them or moves in fits.

Consequence: traditional **angle/curvature-PID with feedforward on κ_des** is wrong here. FF for κ_des is the correct pattern when the plant *needs* sustained torque to maintain an angle (any EPS); it's wrong when stiction maintains the angle for free.

The custom controller is a **plant-inversion design in front-wheel-angle (δ) space**: compute the torque that would move δ by `δ_err` over the model's action horizon, ramp to it, then let stiction hold.

```
δ_des  = atan(κ_des · L)        L = CP.wheelbase (BMW E90: 2.76 m)
δ_meas = atan(κ_meas · L)        κ_meas = yawRate / v_ego
δ_err  = δ_des − δ_meas
target_nm = T_CAP_SLOPE_BASE · kappa_scale(|κ_des|) · v² · (δ_err − tolerance·sign(δ_err))
```

Within the tolerance deadzone, target → 0 and stiction holds. Outside, the controller ramps to `target_nm` over `spread_frames` CAN ticks. ISO comfort guards (cancel_jerk / cancel_accel) gated on **actual plant overshoot** brake the ramp if the plant runs past κ_des.

---

## 2. Single-knob timing design

There is one knob — `CP.steerActuatorDelay` in `bmw/interface.py:87` — and every timing constant downstream subscribes to it via the liveDelay chain. Change SAD and the whole stack adapts.

```
CP.steerActuatorDelay              (bmw/interface.py — THE knob; currently 0.4)
        │
        ▼  + 0.2 (lagd.py:181 initial_lag = SAD + 0.2)
liveDelay.lateralDelay             (= 0.6 s; permanently pinned for BMW —
                                    lagd never converges because it correlates
                                    on latcontrol_torque telemetry we don't
                                    produce; status stays 'unestimated')
        │
        ├──▶ modeld lat_action_t = lat_delay + DT_MDL = 0.65 s
        │       └──▶ get_curvature_from_plan samples κ_des on the predicted
        │            trajectory at this horizon (modeld.py:391)
        │
        └──▶ controlsd lat_delay = liveDelay + LAT_SMOOTH_SECONDS = 0.6 s
                                    (LAT_SMOOTH_SECONDS = 0.0 on catpilot)
                │
                ▼  passed to LaC.update(..., lat_delay)
        register.py update() per livePose tick:
                model_action_t = lat_delay + DT_MDL = 0.65 s   ≡ modeld's lat_action_t
                half_horizon   = model_action_t / 2 = 0.325 s
                action_cadence_ticks = round(half_horizon / DT_LIVEPOSE)  = 6  (300 ms)
                spread_frames        = action_cadence_ticks × 5            = 30 (300 ms)
                jerk_pred horizon    = model_action_t                      = 0.65 s
```

**Concrete values at SAD = 0.4 (current):**

| | value |
|---|---:|
| `liveDelay.lateralDelay` | 0.6 s |
| `modeld lat_action_t` | 0.65 s |
| `model_action_t` (in update) | 0.65 s |
| `action_cadence_ticks` | 6 (= 300 ms decision period) |
| `spread_frames` | 30 (= 300 ms ramp, exactly = cadence) |
| `jerk_pred` horizon | 0.65 s |

**Plant-settling check**: BMW hydraulic rack τ ≈ 130 ms → 300 ms cadence ≈ 2.3τ → ~90% step response between decisions. Margin preserved.

**Why `spread_frames = action_cadence_ticks × 5`** rather than independently rounded: `DT_LIVEPOSE = 50 ms = 5 × DT_CAN_TICK (10 ms)`. Independent rounding can diverge by one tick (e.g., 6×50 = 300 ms vs 32×10 = 320 ms). Deriving spread from cadence enforces exact equality — ramp completes precisely when next decision lands.

**SAD retuning range**: `[0.3, 0.5]` keeps cadence in `[5, 7]` ticks (250–350 ms), all ≥ 2τ for the BMW rack. Below 0.3 risks plant-settling adequacy; above 0.5 just adds curve-entry latency without further wobble benefit.

---

## 3. Reference signal & filter chain

```
modelV2.action.desiredCurvature   (raw model output, ~20 Hz)
        │
        ▼  hooks.run('controls.curvature_correction', ...)
        │     (passthrough — no curvature corrections registered in the public build)
        ▼
clip_curvature() in drive_helpers.py
        │  — rate-limited to MAX_LATERAL_JERK = 5.0 m/s³ (ISO)
        │  — clamped to ±MAX_LATERAL_ACCEL_NO_ROLL = 3.0 m/s² (ISO, + roll comp)
        │  — clamped to ±MAX_CURVATURE = 0.2 1/m
        ▼
self.LaC.update(active, CS, VM, params, steer_limited_by_safety,
                desired_curvature, curvature_limited, lat_delay)
        ▼
=== register.py update() ===
state['desired'] = float(desired_curvature)   # raw, NO filter on κ_des itself
v               = float(lp.velocityDevice.x), floored at 8.5 m/s
state['measured'] = float(lp.angularVelocityDevice.z) / v
delta_des       = atan(state['desired'] · L)
delta_meas      = atan(state['measured'] · L)
delta_err_raw   = delta_des - delta_meas
        │
        ▼  κ-gated box filter on delta_err (only at near-straight wobble)
        │  lane_change_active = (_sm['modelV2'].meta.laneChangeState != off)
        │  if |state['desired']| ≥ KD_GATE or lane_change_active:
        │       buffer cleared, delta_err = delta_err_raw (passthrough)
        │  else:
        │       buffer.append(delta_err_raw); window = action_cadence_ticks
        │       delta_err = sum(buffer) / len(buffer)   (equal-weight box)
        ▼
state['delta_err'] = delta_err                 # filtered, used by controller
        ▼
tolerance = 2 · DRIFT_M · L / (v · model_action_t)²    (kinematic, 1/v² scaling)
        │
        ▼  ISO comfort guards (gated on actual overshoot)
        │  overshooting = (κ_des - κ_meas) · κ_meas < 0
        │  if overshooting and |a_y_meas| > BMW_LATERAL_ACCEL → cancel_accel
        │  elif overshooting and |jerk_pred| > BMW_LATERAL_JERK → cancel_jerk
        │  → target_frac = -copysign(FRICTION, κ_meas)
        ▼
Cadence decision (every action_cadence_ticks):
   if |delta_err| ≤ tolerance:
      hold_zero (target_frac = 0)   OR   brake_zero (-FRICTION reverse, one-shot)
   else:
      kappa_scale = interp(|κ_des|, T_CAP_SCALE_KAPPA, T_CAP_SCALE_BP)
      effective_err = delta_err - copysign(tolerance, delta_err)
      target_nm = T_CAP_SLOPE_BASE · kappa_scale · v² · effective_err
      target_frac = target_nm / STEER_MAX
      if |target_frac| < FRICTION: target_frac = copysign(FRICTION, delta_err)  (breakaway)
      cap to ±t_cap_frac
      action = 'ramp' (or 'breakaway')
        ▼
Per-CAN-tick ramp: state['torque'] += ramp_step toward state['target_frac']
        ▼
return -state['torque']  (BMW carcontroller flips sign convention)
```

---

## 4. The κ-gated box filter on delta_err

**Purpose**: suppress high-rate sign-flips in `delta_err` caused by vision-only κ_des wobble on near-straight (no position-feedback layer running). Each sign-flip across the tolerance band triggers `cancel_tol` / `brake_zero` with a counter-direction FRICTION pulse — those are the felt "swaying" pulses.

**Target choice**: filter applied to `delta_err`, **not** to raw κ_des. Keeps `state['desired']` raw so:
- Reference is always fresh — no held bias → no drift accumulation
- `kappa_scale`, `jerk_pred`, ISO guards all see raw κ_des magnitude (true model intent)
- Only the cadence decision's tolerance check sees the smoothed error

**Gate**: `KD_GATE = 0.002` 1/m. Filter active only when `|raw κ_des| < KD_GATE`. Above gate (real curves) → bypass, buffer flushed. Also bypassed during `modelV2.meta.laneChangeState != off`.

**Window**: `kd_filter_window = action_cadence_ticks = 6` samples (300 ms box, 150 ms group delay).

**Output**: equal-weight uniform box average.

```python
raw_kd = float(desired_curvature)
lane_change_active = (_sm['modelV2'].meta.laneChangeState != log.LaneChangeState.off)
if abs(raw_kd) >= KD_GATE or lane_change_active:
    state['de_buffer'].clear()
    delta_err = delta_err_raw
else:
    state['de_buffer'].append(delta_err_raw)
    if len(state['de_buffer']) > action_cadence_ticks:
        state['de_buffer'].pop(0)
    delta_err = sum(state['de_buffer']) / len(state['de_buffer'])
state['delta_err']     = delta_err
state['delta_err_raw'] = delta_err_raw    # both published in telemetry
```

**Design rule (the filter exists for ONE job)**: suppress sign-flips on near-straight. Never compromise real correction. Hence gate at 0.002 (below typical wobble peak amplitude ±0.003) — the filter operates only at zero-crossing windows where sign-flips actually happen; wobble peaks pass through raw.

**Field-measured sign-flip reduction**: ~60% on highway-dominant routes.

---

## 5. The kinematic deadzone

```python
DRIFT_M = 0.02                                     # m of allowed drift over model_action_t
lookahead_m = v * model_action_t
tolerance   = 2.0 * DRIFT_M * L / (lookahead_m**2) # rad, 1/v² scaling
```

**Physical meaning**: tolerance is the front-wheel-angle error that produces ≤ `DRIFT_M` of lateral position drift over `model_action_t` of forward travel. At 85 kph: tolerance ≈ 0.027°. At 30 kph: tolerance ≈ 0.36°.

**Why 1/v² and not constant-angle**: a constant 0.35° deadzone was tried (commit 715114d) and reverted (d43fb19) after route 31b seg 8/15 showed the controller silent for 98% of samples while `δ_err` sat inside the wide band, producing 1.3–1.7 m lateral drift at 85 kph with no position-feedback layer running. The 1/v² formula tightens the deadzone exactly where it matters (high speed) and is the only safe choice without a position-feedback layer.

**Used in three places** in the cadence decision:
- `if abs(delta_err) ≤ 1.2 · tolerance` → cancel_tol band (drain ramp toward FRICTION-level brake or 0)
- `if abs(delta_err) ≤ tolerance` → hold_zero / brake_zero
- `effective_err = delta_err - copysign(tolerance, delta_err)` → soft-deadband in target_nm formula

---

## 6. T_CAP_SCALE — curvature-dependent gain

```python
T_CAP_BASE_NM     = 2.0    # stiction floor (Nm)
T_CAP_SLOPE_BASE  = 1.0    # base aligning-torque gain
T_CAP_SCALE_KAPPA = [0.001, 0.01,  0.02]   # |κ_des| breakpoints (1/m)
T_CAP_SCALE_BP    = [1.0,   2.5,   3.0]    # multiplicative scale on T_CAP_SLOPE_BASE

kappa_scale = np.interp(|κ_des|, T_CAP_SCALE_KAPPA, T_CAP_SCALE_BP)
target_nm   = T_CAP_SLOPE_BASE · kappa_scale · v² · effective_err
t_cap_nm    = T_CAP_BASE_NM + T_CAP_SLOPE_BASE · kappa_scale · v² · |δ_des|  (≤ STEER_MAX=12)
```

**Design rule**: curvature awareness must enter as a **multiplicative gain scale** on `T_CAP_SLOPE_BASE`, NOT as a feedforward additive term on κ_des. The stiction-holds-at-zero plant doesn't need continuous holding torque; an additive FF would over-push and induce stiction-locked overshoots.

**Tuning history of `T_CAP_SCALE_BP[0]` (small-κ gain)**:
- Originally 1.0 (matches T_CAP_SLOPE_BASE — neutral baseline)
- Bumped to 1.5 for more aggressive small-κ tracking
- **Returned to 1.0** (commit bba4454, 2026-05-22): the 1.5 caused over-correction in the small-κ regime → overshoots → cancel_jerk pulses. Route 326: cancel_jerk per lane change dropped from 25 to 5.3 with no compromise to lane offset.

Higher-κ scale points (2.5 @ 0.01, 3.0 @ 0.02) unchanged so tight-curve tracking authority is preserved.

---

## 7. ISO 11270 comfort guards

```python
ISO_LATERAL_ACCEL = 3.0    # m/s²  (from opendbc.car.lateral)
ISO_LATERAL_JERK  = 5.0    # m/s³

LATERAL_CURVATURE = [0.001, 0.005, 0.01, 0.02]
LATERAL_ACCEL_BP  = [1.5,   1.5,   2.5,  ISO_LATERAL_ACCEL]   # half-ISO at small κ, full at tight
LATERAL_JERK_BP   = [1.5,   1.5,   3.0,  ISO_LATERAL_JERK]
```

Fire only when **the plant has actually overshot** `(κ_des - κ_meas) · κ_meas < 0` AND the relevant signal exceeds its κ-dependent threshold. Action: drop `target_frac` to `−copysign(FRICTION, κ_meas)` — small reverse pulse to brake plant momentum.

**Bug history (worth remembering)**: `LATERAL_CURVATURE` second value was originally `0.05` (out of order), making `np.interp` non-monotonic. Effect: thresholds were stuck at the small-κ value (2.0 at the time) all the way through `|κ|=0.02`, then jumped discontinuously to ISO at the boundary. cancel_jerk / cancel_accel were firing more aggressively than designed during real moderate curves. Fixed in commit 633a146 along with the small-κ tightening (2.0 → 1.5). Verified positive on routes 32a/32d: `cancel_accel` essentially eliminated (114 on route 326 → 0/18), post-LC max torque cut from 4.3 N to 2.1 N.

---

## 8. Action state machine (debug field)

State held in `state['action']`, published in `bmw_lat_control` telemetry. Useful for forensic analysis.

| state | when entered |
|---|---|
| `init` | controller construction |
| `hold_zero` | `|delta_err| ≤ tolerance`, straight (`hold_f = 0`) — target 0, stiction holds |
| `hold_curve` | `|delta_err| ≤ tolerance`, curve (`hold_f > 0`) — target `hold_f·torque`, holds the standing torque against self-aligning torque |
| `ramp` | active plant-inversion push toward `target_nm` (sub-friction targets commanded as-is since 2026-07-03) |
| `cancel_tol` | error fell into 1.2× tolerance band mid-ramp; drain to `hold_f·torque` (0 on straights) |
| `cancel_accel` | overshoot AND `|a_y_meas| > BMW_LATERAL_ACCEL` — drain to 0 |
| `cancel_jerk` | overshoot AND `|jerk_pred| > BMW_LATERAL_JERK` — drain to 0 |

Removed 2026-07-03 (see header note): `brake_zero`, `breakaway`. Added: `hold_curve`. Telemetry gains `hold_f`.

---

## 9. Telemetry (plugin_bus topic `bmw_lat_control`, 20 Hz)

Published each livePose tick:

| field | meaning |
|---|---|
| `desired` | κ_des actually used by controller (= raw, since no κ filter) |
| `desired_raw` | identical to `desired` (kept for back-compat with older filter designs) |
| `measured` | κ_meas from livePose yaw rate / v_ego |
| `err` | κ_des − κ_meas (debug) |
| `delta_err` | **filtered** front-wheel angle error (rad) — what controller acts on |
| `delta_err_raw` | pre-filter δ_err (rad) — for filter diagnostics |
| `target_frac` | current cadence-decision target torque fraction (−1..1) |
| `ramp_step` | per-CAN-tick torque increment |
| `ramp_frames` | CAN frames remaining in current ramp |
| `action` | state machine label (see §8) |
| `torque` | current commanded torque fraction (−1..1) |
| `output` | clipped torque fraction returned to controlsd |
| `vEgo` | from CS.vEgo |
| `active` | controller active flag |
| `a_y_meas` | `v²·κ_meas` (m/s²) |
| `jerk_pred` | `v²·(κ_des - κ_meas)/model_action_t` (m/s³) |

Multiply `output` or `torque` by `STEER_MAX = 12 Nm` for Nm.

---

## 10. Constraints baked into the design

- **Vision-only stack**: catpilot has no HD-map, lidar, radar fusion, or IMU-fusion for κ. All correction must come from `modelV2` outputs. Don't propose external-sensor escape hatches for vision-noise problems.
- **No position-feedback layer**: there is no lane-offset correction registered on `controls.curvature_correction` in the public build, and adding one based on `modelV2.laneLines` would not fix κ_des wobble on straights (laneLines share the same vision noise source as `desiredCurvature`). The controller must handle vision noise downstream without position feedback.
- **lagd never converges for BMW**: `liveDelay.lateralDelay` is permanently pinned at `SAD + 0.2` because lagd correlates on latcontrol_torque telemetry our controller doesn't produce. `SAD` is the effective knob — not a hint to be overridden.
- **BMW hydraulic rack**: stiction holds δ at zero torque, no self-centering. The plant-inversion controller is designed around this. **Don't add a FF term on κ_des** (correct for EPS, wrong here).

---

## 11. Failed experiments (do not re-litigate)

These were tried, deployed, and reverted. Each appears here so future maintainers know what's been ruled out and why.

| experiment | result | file:line | reverted by |
|---|---|---|---|
| Constant-angle deadzone (`TOL_DEG_CONST = 0.35°`) | Allowed 1.3–1.7 m drift at 85 kph with no position-feedback layer running; controller silent 98% of samples (route 31b seg 8/15). Wide angular deadzone is unsafe without a position-feedback layer. | commit 715114d | d43fb19 |
| Stiction-gated steering-angle FF (linear in \|steer\|, 20°/0.10 Nm·deg⁻¹) | Over-pushed in route 31d seg 8 right turn: 50% over-rotation, 6.22 Nm torque, sustained cancel_jerk oscillation. SAT physics aren't linear in steering angle alone — needs multi-regime (v×κ grid) calibration, not single-route fit. | commit 2c67a97 | e81f1c0 |
| Hysteresis-on-κ_des (`KD_HYST_GAP = 0.003`, `KD_GATE = 0.006`) | 95% sign-flip reduction in offline check, but route 322 had 32% time `\|offset\|>0.5 m` and 44-second frozen-κ_des stretches — controller faithfully tracked a held biased reference → drift. Structural failure of holding the reference without a position-feedback layer. | commit daef207 | 96945fc (replaced with box-on-delta_err) |
| Suggesting external sensor fusion (HD map, radar, lidar, IMU) for vision noise | Out of project scope by design — see [vision-only constraint](§10). | (recurring; do not propose) |

---

## 12. Retuning recipes (current operating point)

Current configuration is field-verified stable (2026-05-24, routes 32a/32d, user: "solid in straight, agile and accurate in turns"). Only retune if a new failure mode appears.

### To change the controller's overall timing (cadence, ramp, jerk horizon, modeld lookahead all together):
- Change `ret.steerActuatorDelay` in `bmw/interface.py:87`
- Range `[0.3, 0.5]` keeps cadence in `[5, 7]` ticks (250-350 ms). Don't go below 0.3.

### To suppress more wobble at near-straight (filter):
- Widen `KD_GATE` (e.g. 0.002 → 0.003) — filter operates over a wider κ range. Trades crisp response at very-low-κ for more smoothing.
- Larger `kd_filter_window` would add more group delay; currently coupled to `action_cadence_ticks` so it'll grow with SAD bumps automatically.

### If lane-change overshoot reappears on the post-LC phase:
- Look at `bmw_lat_control` log for that LC: is `de_at_end_raw > 0.3°`? Is post-LC `|torque|` > 3 N?
- The mechanism is one of: spurious cancel_jerk suppressing controller during LC (small κ, |κ_meas| < 0.001 — see seg 10 of route 326 for pattern) OR rack stiction → catch-up overshoot (see seg 42 ev2 for pattern).
- Don't apply a global LC torque cap — most LCs are fine. Targeted options exist (magnitude-gated cancel, soft post-LC re-engagement); see commit history around routes 326/32a/32d for the analysis.

### If straights feel less smooth than today:
- First, check that the κ_des wobble character hasn't changed (newer model versions may have different noise).
- The current 60% sign-flip reduction at gate 0.002 is the design point. Going tighter (0.001) makes the filter operate only at zero-crossings — less aggregate reduction. Going looser (0.005) starts compromising real small-amplitude corrections.

### If real-curve tracking feels under-aggressive:
- Increase `T_CAP_SCALE_BP[1]` or `T_CAP_SCALE_BP[2]` (currently 2.5, 3.0 at `|κ| = 0.01, 0.02`).
- **Don't** increase `T_CAP_SCALE_BP[0]` past 1.0 — that's where route 326's over-correction lived (was 1.5, now back to 1.0).

### If cancel events feel too frequent on moderate curves:
- Loosen `LATERAL_ACCEL_BP` / `LATERAL_JERK_BP` at the moderate-κ breakpoints (currently 2.5 / 3.0 at `|κ|=0.01`).
- Don't loosen the small-κ values (1.5) — they're the brake against small-amplitude overshoot.

### If unwind pulses are felt too often (controller pushes back too easily):
- Loosen `BMW_LATERAL_JERK_BP[0]` / `BMW_LATERAL_ACCEL_BP[0]` (currently 1.5). Cancel guards will fire less often at small κ. But verify no spurious overshoot crept in.

---

## 13. Code map (`register.py`)

| section | line range (approx) | content |
|---|---:|---|
| Constants block | 210-360 | All tuning constants with rationale comments |
| State init dict | 320-360 | Per-controller persistent state |
| `update()` function | 360+ | Per-CAN-tick body, with livePose-gated heavy logic |
| Plant horizon block | inside update | `model_action_t`, cadence/spread computed from `lat_delay` |
| κ-filter block | inside update | δ_err box filter with κ-gate + LC-disable |
| Tolerance | inside update | Kinematic 1/v² deadzone |
| ISO guards | inside update | Overshoot-gated cancel_jerk / cancel_accel |
| Cadence decision | inside update | hold_zero / brake_zero / breakaway / ramp / target_nm formula |
| Ramp application | inside update | Per-CAN-tick torque ramp |
| Telemetry publish | inside update | `bmw_lat_control` topic via plugin_bus |

---

## 14. Quick verification recipe

To check that a deployed change behaves as expected:

```bash
# On c3:
cd /data/openpilot && source /usr/local/venv/bin/activate

# Load a route's bmw_lat_control + modelV2:
python -c "
import zstandard, json
from cereal import log
ev = list(log.Event.read_multiple_bytes(zstandard.ZstdDecompressor().decompress(
    open('/data/media/0/realdata/<ROUTE>--<SEG>/rlog.zst','rb').read(),
    max_output_size=600*1024*1024)))
# ... iterate events, e.which()=='pluginBusLog' → e.pluginBusLog.entries
#                 where en.topic=='bmw_lat_control' → json.loads(en.json)
"
```

Key metrics to compute per route:
- **Sign-flip reduction** on near-straight (filter regime): `delta_err_raw` vs `delta_err` sign-flips per second
- **Lane offset**: from `modelV2.laneLines[1].y[0]` and `modelV2.laneLines[2].y[0]` averaged. RMS should be 0.30-0.45 m on healthy drives.
- **cancel_jerk / cancel_accel counts**: per lane change (should be < ~10 / LC each)
- **`de_at_end_raw`**: δ_err at the moment laneChangeState flips back to `off`. Should be < 0.3° on healthy LCs.
- **`post_tq_max`**: peak `|output·STEER_MAX|` in the 1.5 s after each LC end. Should be < 2-3 Nm.

Reference baselines (field-verified, 2026-05):
- Route 31c: lane offset rms 0.33 m, filter reduction 59% — the "nearly perfect" baseline
- Routes 32a / 32d: 0 flagged LC events out of 39 LCs, filter reduction 65-66%, cancel_accel essentially eliminated — current stable operating point
