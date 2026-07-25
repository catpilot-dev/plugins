# Calibration Trim — Perception-Side DC Position Compensation

**Date:** 2026-07-25
**Status:** SPEC — Phase 4 of the lane-keeping arc
**Depends on:** AC Stabilizer (2026-07-23-ac-stabilizer-design.md, deployed), catpilot hook framework
**Repos touched:** catpilot (`selfdrive/modeld/modeld.py` — two hook call sites) AND plugins (`lane_keeping`)

## 1. Motivation

Route 3c1 established the fundamental finding: the driving model closes its
position loop through the camera with unbounded curvature authority, so any
bounded output-side bias (κ correction) loses every sustained disagreement.
The AC Stabilizer resolved the fast band by damping wander and conceding the
DC set-point to the model. Routes 3c2/3c3 then showed the cost of that
concession: on leftmost lanes with a narrow (~0.8 m) shoulder, the model
references the ROAD EDGE rather than the painted line (route 3c3 seg 16:
34 s with the left wheel 0.10 m from a strongly-detected line, wheel-to-edge
0.875 m — squarely in the model's normal wheel-to-boundary band). The hard
floor's bounded push cannot correct this; it is the same lost fight.

Calibration trim moves the fight to the model's INPUT. A small yaw bias δ
added to the calibration euler used for the camera warp shifts the model's
perceived world laterally (≈ δ·x at distance x). The model repositions the
real car and then HOLDS the new position with its full authority, believing
it is centered. The model cannot fight what it cannot perceive.

Division of labor after this feature:
- AC (fast wander): stabilizer, output-side, bounded, zero-mean. Unchanged.
- DC (sustained offset): calibration trim, input-side, slow, model-enforced.

## 2. Verified plumbing facts (catpilot @ dev, 0.11 base)

- `modeld.py:348` — modeld reads `sm["liveCalibration"].rpyCalib` into
  `device_from_calib_euler` and builds both warp matrices from it
  (`get_warp_matrix`, lines 350–351). Injection point A.
- `modeld.py:389–410` — `cameraOdometry` is filled from the model's pose
  head (`fill_model_msg.py:186–187`: `trans = pose[0,:3]`,
  `rot = pose[0,3:]`) and published. The pose head runs on the WARPED
  frames, so a warp bias leaks into the pose. Injection point B (de-bias).
- `calibrationd.handle_cam_odom` consumes trans, rot, wideFromDeviceEuler,
  transStd, roadTransformTrans(+Std). Its yaw observation is
  `arctan2(trans[1], trans[0])`; pitch is `-arctan2(trans[2], trans[0])`;
  height is `roadTransformTrans[2]`. Under a pure yaw rotation about
  device-z, only `trans[1]` changes at first order. rot[2] is used only as
  a gate (`abs(rot[2]) < MAX_YAW_RATE_FILTER`).
- The hooks framework (`selfdrive/plugins/hooks.py`) is generic by name:
  `hooks.run('modeld.calib_bias', default, ...)` needs no registry change,
  loads plugins lazily, and returns `default` on ANY plugin exception
  (fail-safe). modeld does not import hooks today; the import is part of
  this change. controlsd (`controlsd.py:22`) shows the import pattern.

## 3. Architecture

```
lane_keeping plugin (controlsd process, 100 Hz hook)
  anchor.py ──gap_dc──▶ calib_trim.py (pure law: modes, integrator,
                          slew, cap, decay)
                          │  δ (deg), written 1 Hz, atomic tmp+rename
                          ▼
  /data/plugins-runtime/lane_keeping/data/CalibTrimYawDeg
                          │  read every 100 model frames (~5 s), clamped
                          ▼
lane_keeping plugin (modeld process, via hook)
  register.py: on_calib_bias() ──δ──▶ modeld.py call site A (warp euler)
                                └───▶ modeld.py call site B (pose de-bias)
```

- The trim LAW runs where the data is (controlsd process, has gap_dc and
  all anchor gating). modeld only APPLIES a number.
- Transport is the plugin's own data dir (atomic rename), consistent with
  the framework's param convention. File persists across reboots — δ is
  quasi-static compensation and SHOULD survive a restart; the in-band
  decay rule (§5) retires stale compensation within ~1 min of driving.
- Both call sites live in `modeld.py` only (the de-bias is applied to
  `posenet_send` after `fill_pose_msg`, before `pm.send`). No change to
  `fill_model_msg.py` or calibrationd.

## 4. catpilot call sites (exact contract)

Call site A — after line 348, before the warp matrices are built:

```python
device_from_calib_euler = np.array(sm["liveCalibration"].rpyCalib, dtype=np.float32)
yaw_bias_deg = float(hooks.run('modeld.calib_bias', 0.0))
yaw_bias_deg = max(-1.0, min(1.0, yaw_bias_deg))   # hard clamp, defense in depth
device_from_calib_euler[2] += np.float32(np.radians(yaw_bias_deg))
```

Call site B — after `fill_pose_msg(...)`, before `pm.send('cameraOdometry', ...)`:

```python
if yaw_bias_deg != 0.0:
  b = np.radians(yaw_bias_deg)
  c, s = np.cos(b), np.sin(b)
  for vec in (posenet_send.cameraOdometry.trans, posenet_send.cameraOdometry.rot):
    x, y = vec[0], vec[1]
    vec[0] = c * x + s * y      # R_z(-b): undo the frame rotation the
    vec[1] = -s * x + c * y     # biased warp induced in the pose head
```

De-bias sign contract: the warp was biased by +b about device z; the pose
head then reports vectors in a frame rotated by +b relative to true device;
rotating the reported vectors by R_z(−b) restores the true frame. The
IMPLEMENTATION must carry this derivation as a comment, and the on-car
blindness gate (§8) is the empirical check: during a fixed-δ drive,
`liveCalibration.rpyCalib` must not drift by more than 0.05° beyond its
δ=0 drift. If the analytical sign is wrong the drift will be ~δ-sized and
immediately visible — the gate catches it before mode 2 exists.

`yaw_bias_deg` is read ONCE per frame iteration (call site A) and reused at
call site B — the two sites must never see different values in one frame.
Cache-latency note: modeld's hook re-reads the file every 100 frames; the
value can be up to ~5 s stale. The writer's slew cap (§5) bounds any step
to ≤ 0.1°, i.e. ≤ 5 cm of perceived shift at 30 m — imperceptible.

Failure containment: `hooks.run` returns 0.0 on any plugin exception; a
0.0 bias makes both call sites exact no-ops. modeld's frame time budget
must not regress: the hook body is a cached file read amortized to
~1/100 frames; the probe (§8) checks `modelExecutionTime` before/after.

## 5. Trim law (`plugins/lane_keeping/calib_trim.py` — pure, no I/O)

Config (dataclass `TrimConfig`), with plugin params (all live in
`data/`, read at construction like AnchorConfig):

| param | default | meaning |
|---|---|---|
| `CalibTrimMode` | 0 | 0=off, 1=fixed (identification), 2=closed-loop |
| `CalibTrimFixedDeg` | 0.0 | mode-1 target δ (clamped to ±max) |
| `CalibTrimMaxDeg` | 0.8 | authority cap for the law (< modeld's 1.0 hard clamp) |
| `CalibTrimSlewDegS` | 0.02 | max |dδ/dt| — "slowly" is a design requirement |
| `CalibTrimYawSign` | 0 | +1/−1 mapping δ→gap, MEASURED in identification; 0 = mode 2 inert |
| `CalibTrimKi` | 0.04 | integrator gain, deg/s per meter of band error |
| `CalibTrimGapLo` / `Hi` | 0.6 / 1.0 | target band for gap_dc (the deleted [0.6,1.0] band returns — enforced BY the model, not against it) |

State: `delta_deg` only. Update at 100 Hz (called from the existing
curvature hook alongside the anchor), all transitions slew-limited by
`slew`; `delta` always clamped to ±`max_deg`.

- mode 0 (or plugin toggle `LaneKeepEnable=0`): slew `delta` → 0.
- mode 1: slew `delta` → clip(`fixed_deg`, ±max). No gating — the
  identification drive needs the bias held through all conditions.
- mode 2: requires `yaw_sign` ∈ {+1,−1}, else behaves as mode 0.
  - Gate for INTEGRATION (not for holding): line trusted
    (`authority > 0`), not lane-changing, `v_ego ≥ 5` m/s. gap_dc may be
    frozen (hard floor) — the frozen value is the last trusted reading and
    remains valid input; the floor does NOT gate the trim.
  - `err = gap_dc − gap_lo` if `gap_dc < gap_lo`, `gap_dc − gap_hi` if
    `gap_dc > gap_hi`, else 0. (err < 0 ⇒ too close to the line.)
  - `err ≠ 0`: `delta += clip(−ki · err · yaw_sign, ±slew) · DT` … i.e.
    integrate toward reducing |err|, rate-capped. (The −sign here is a
    convention anchor: with yaw_sign as measured, positive err must
    produce dδ that moves gap_dc DOWN. Tests pin this with both signs.)
  - `err = 0` continuously for > 5 s: slew `delta` → 0 at `slew/2`
    (retire compensation the moment the model behaves — this is what
    prevents a leftmost-lane δ from displacing the car after moving to a
    central lane).
  - Gate fails (untrusted / LC / slow): HOLD `delta` (no integrate, no
    decay). Dropouts must not dump quasi-static compensation.
- Return value each tick: `delta_deg` (for the writer) + telemetry dict
  (`delta_deg`, `err`, `mode`, `integrating` bool).

Response-time honesty: at 0.02°/s and an expected need of 0.3–0.5°, the
trim takes ~15–25 s to take hold. It is a fix for SUSTAINED stretches
(3c3 seg 16's 34 s episode, long leftmost-lane cruises), not for brief
hugs — those remain the stabilizer/floor's job. This is by design; do not
raise the slew to chase episodes.

## 6. Plugin wiring (`register.py`)

- Controls side: in `on_curvature_correction`, after the anchor update,
  step the trim law with (gap_dc, authority, lane_changing, v_ego) from
  the anchor's telemetry; every 100 ticks (1 Hz) — same cadence pattern as
  the live-toggle re-read — write `delta_deg` to `data/CalibTrimYawDeg`
  via tmp file + `os.replace` (atomic). Skip the write when the value is
  unchanged at 0.001° resolution.
- modeld side: register `modeld.calib_bias` → `on_calib_bias(default)`:
  cached read of `data/CalibTrimYawDeg` refreshed every 100 calls;
  missing/unparseable file → 0.0; result clamped to ±`CalibTrimMaxDeg`
  default (the modeld call site clamps again at ±1.0). NO imports of
  anchor/trim modules in this path — it is a float file read, nothing
  else, because it runs inside modeld's frame loop.
- Both registrations follow the existing lazy-import discipline; the
  modeld hook must be registerable without the controls-side state
  existing (separate processes, separate plugin instances).

## 7. Safety analysis

- **Bounded perception shift.** |δ| ≤ 0.8° = 0.014 rad ⇒ ≤ 0.42 m
  perceived lateral shift at 30 m — inside the mounting-yaw spread the
  fleet-trained model demonstrably tolerates (calibrationd accepts and
  the fleet contains comparable real mounting errors). Everything the
  model perceives shifts coherently (lanes, leads, edges); there is no
  internal inconsistency, it is exactly equivalent to a slightly
  yaw-rotated camera mount.
- **No arms race.** Call site B keeps calibrationd's observations
  δ-invariant (§2 math; §8 empirical gate). calibrationd continues to
  estimate the TRUE mounting; stored calibration is never polluted. This
  is mandatory — without it calibrationd cancels δ at its own filter
  timescale and the integrator escalates against it.
- **Our feedback measurement stays valid.** The anchor's gap is measured
  at x=0, where a yaw rotation shifts nothing (δ·0). The slow loop closes
  on an (approximately) unbiased measurement.
- **Fail-safe chain.** Plugin exception → hooks.run returns 0.0 → both
  call sites no-op. File missing → 0.0. Law crash → hook exception →
  0.0. modeld never sees a step: writer slews, reader clamps.
- **`.disabled` rollback.** With the plugin removed, the hook is
  unregistered → 0.0 bias → stock modeld behavior. The existing coupled
  BMW-controller rollback story is unchanged.
- **Double application is impossible** by construction: δ is applied to
  the warp only in modeld call site A; the de-bias (B) touches only
  cameraOdometry, which nothing in the lateral path consumes (only
  calibrationd does).

## 8. Testing & gates

Unit (plugins repo, `tests/test_calib_trim.py`): slew and cap honored in
all mode transitions; mode 2 inert with yaw_sign=0; integrator direction
correct for both yaw_sign values and both band violations; in-band decay
after 5 s dwell and its rate; HOLD on untrusted/LC/slow; toggle-off decay;
frozen-gap_dc input accepted; file-value round-trip at 0.001° resolution.

Unit (catpilot repo, `selfdrive/modeld/tests/test_calib_bias.py`): the
R_z(−b) de-bias composed with an R_z(+b)-rotated vector is identity to
1e-6; clamp at ±1.0°; zero-bias is bit-exact no-op on the message.

On-device probe additions (`on_device_probe.py`): CalibTrimYawDeg file
write/read round-trip through both register paths; modeld hook returns
0.0 with file absent; `modelExecutionTime` p95 within 5% of pre-change
baseline over 1000 frames (log replay or onroad idle).

Identification drive (mode 1) — AFTER the lane-keeping-OFF baseline drive:
1. Same road as 3c3 if possible. Set `CalibTrimMode=1`,
   `CalibTrimFixedDeg=+0.3`. Drive.
2. Analysis: Δ(gap median) vs the OFF baseline on matched stretches →
   yields `CalibTrimYawSign` and gain (m/deg). Blindness gate:
   `liveCalibration.rpyCalib` yaw drift ≤ 0.05° beyond baseline drift.
   κ_des/steering churn unchanged (the model should not "notice").
3. Repeat with −0.3° if sign ambiguous.
Mode 2 is GATED on: blindness gate passed, sign measured, gain in a sane
range (0.1–1.0 m/deg — outside that, the mechanism doesn't work as
modeled and we stop).

## 9. Deployment sequence (each step needs explicit user go)

1. Implement everything; suite green in both repos. **No deploy.**
2. USER: lane-keeping-OFF baseline drive (already armed) → hug analysis.
3. Deploy catpilot + plugins with `CalibTrimMode=0` (pure plumbing,
   inert). Verify probes; normal drive to confirm no regression.
4. Identification drive(s), mode 1 → sign/gain/blindness analysis.
5. Mode 2 enable with measured `CalibTrimYawSign` — the closed-loop trim.
6. Success metric (same roads as 3c2/3c3): floor-episode total duration
   and % time gap < 0.3 m reduced by ≥ 50% on leftmost-lane stretches,
   with κ_des AC and churn unchanged (the model must not be fighting).

## 10. Non-goals

- No pitch/roll biasing (lateral only).
- No per-lane / per-road memory of δ (in-band decay + re-integration is
  the v1 answer; a learned road-context prior is future work).
- No episode-speed response (see §5 response-time honesty).
- No changes to the AC stabilizer, floors, or BMW tracker.
