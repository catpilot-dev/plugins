# DCC Response Study & Data-Driven Command Mapping — Design

**Date:** 2026-07-29
**Status:** Approved design, Phase 1 committed, Phase 2 gated on Phase 1 findings
**Code under study:** `plugins/bmw_e9x_e8x/bmw/carcontroller.py`

## Problem

The BMW E9x DCC emulation selects cruise stalk commands (plus1/plus5/minus1/minus5)
and cadence (single 20 Hz / hold 40 Hz) by thresholding the planner's commanded
acceleration (`actuators.accel`) against four hand-picked constants
(`ACCEL_HOLD_THRESHOLD` 0.2, `ACCEL_STEP5_THRESHOLD` 0.6, `DECEL_HOLD_THRESHOLD` 0.4,
`DECEL_STEP5_THRESHOLD` 0.9 m/s²), anchored to a rough hand calibration
(PLUS1+HOLD ≈ +0.4, PLUS5+HOLD ≈ +1.2, MINUS1+HOLD ≈ −0.6, MINUS5+HOLD ≈ −1.2 m/s²).

Observed symptoms: sometimes sluggish, sometimes too harsh. The achieved
acceleration is not a constant per command — the stalk moves DCC's *setpoint* and
DCC's internal controller chases it, so response depends on command type, cadence,
current speed (engine torque/gearing), setpoint−vEgo gap, and road grade. Fixed
thresholds are right at one operating point and wrong at others.

## Decision

Two phases, gated:

- **Phase 1 (committed):** offline tooling on the dev machine that measures actual
  acceleration response per command×cadence from existing C3 route rlogs.
- **Phase 2 (gated):** replace the fixed thresholds with an **inverse response map**
  (approach B): expected-accel interp tables per command×cadence keyed on vEgo;
  selection picks the candidate closest to aTarget, with hysteresis. Fallback to
  **recalibrated constants** (approach A) if speed dependence is weak or coverage thin.
- **Rejected:** closed-loop escalation on accel error (approach C) — feedback wrapped
  around a black box that infers intent from stalk cadence and has its own internal
  dynamics; hunting risk, murky mid-burst cadence semantics. Out of scope permanently
  unless B demonstrably fails.

## Why this is feasible from existing logs

- rlogs record `sendcan`: every injected `CruiseControlStalk` frame (command bit,
  counter, timestamp) → exact command+cadence reconstruction per moment.
- rlogs record full-rate `carState`: `vEgo`, `aEgo`, `cruiseState.speed` (DCC's
  actual setpoint decoded from the car's own `DynamicCruiseControlStatus` message),
  `gasPressed`, `brakePressed`, stalk signals.
- rlogs record `carControl.actuators.accel` (aTarget) and `livePose` (pitch → grade proxy).
- `cruiseState.speed` stepping additionally gives per-tick **command acceptance**,
  field-validating the counter-overwrite mechanism.

## Phase 1 — Study tooling

**Location:** `plugins/bmw_e9x_e8x/tools/dcc_study/` (mirrors `speedlimitd/tools`
convention). Route data cached in a gitignored dir inside it.

### Data acquisition — COD API

Use the Connect-on-Device API (base `http://<device-ip>:8082`, no real auth locally;
resolve device IP from the `c3` ssh alias via `ssh -G c3`, never hardcode):

1. `GET /v1/me/devices/` — discover the dongle ID; then
   `GET /v1/devices/{dongleId}/routes` — enumerate routes.
2. `GET /v1/route/{routeName}/` — metadata; **pre-filter to `engagement_pct > 0`**
   (only engaged routes contain injected DCC commands).
3. `GET /v1/route/{routeName}/download?files=rlog` — stream tar.gz of rlogs into the
   local cache (skip segments already cached).

`fetch_routes.py` implements this; also usable with an explicit route list.

### Extraction — `extract.py`

Per segment rlog.zst → parquet of time-aligned channels:
`carState` (vEgo, aEgo, cruiseState.speed/enabled, gasPressed, brakePressed, human
stalk signals), `carControl.actuators.accel`, `sendcan` CruiseControlStalk frames,
`livePose` pitch. Decompress with zstandard, parse with
`log.Event.read_multiple_bytes`, filter on `evt.which()`.

### Burst segmentation & response measurement — `bursts.py`

- A **burst** = contiguous frames of the same command with inter-frame gap
  < 0.5 s (`BURST_LIVE_WINDOW`). Cadence classified from inter-frame intervals
  (20 vs 40 Hz). Neutral `act=0` trailing frames (counter-overwrite only) excluded.
- Dropped if contaminated: gas/brake pressed, or human stalk input during burst.
- Per burst record: command, cadence, duration, tick count, vEgo at start,
  setpoint gap (cruiseState.speed − vEgo) at start, mean pitch, baseline aEgo
  (mean over 0.5 s pre-burst), response profile (aEgo during burst + 1.5 s tail),
  steady-state Δaccel (response minus baseline), and setpoint-step acceptance
  count from `cruiseState.speed`.

### Report — `report.py`

- Achieved Δaccel vs vEgo per command×cadence: scatter + binned medians.
- Residuals vs setpoint gap (checks saturation when setpoint runs ahead of vEgo)
  and vs pitch (validates the grade filter).
- Coverage table: burst counts per command×cadence×10 km/h speed bin.
- Acceptance-rate table (ticks accepted / ticks sent) per cadence.

### Confounder handling

- Primary fits filter |pitch| < ~1° (≈2% grade); pitch residual plot verifies
  sufficiency rather than assuming it.
- Setpoint gap recorded to detect response saturation on long bursts.
- No GPS/map inputs (vision-only constraint; grade proxy is livePose only).

## Phase gate

Jointly review the report:

- **Coverage:** enough bursts per command×cadence across ≥3 speed bins.
- **Signal:** four command×cadence response curves distinguishable and monotone
  in speed.

Clear speed dependence → approach B. Flat curves → approach A. Thin coverage →
targeted calibration drives first, then re-decide.

## Phase 2 — Sketch (numbers fixed at the gate)

- Expected-response tables `interp(vEgo, V_BPS, A_VALS)` per command×cadence,
  values from the study.
- Selection: candidate minimizing |expected − aTarget|; hysteresis — switch only
  when the new candidate is better by a margin (~20%) for N consecutive frames,
  so selection cannot hunt at crossovers.
- Unchanged: burst/counter-overwrite mechanics, `V_ERROR_DEADZONE`, setpoint
  guards, min-cruise-speed headroom logic, cancel/gas handling.
- On-car verification required before Phase 2 is considered done.

## Testing & error handling

- `bursts.py` segmentation and response measurement: pytest with synthetic frame
  sequences (known bursts in → known measurements out), plugin test-helper style.
  Note: the pre-push hook glob (`plugins/*/tests/`) does not collect nested test
  dirs — run these explicitly or place accordingly.
- Corrupt/truncated rlogs: skip with a warning, never fatal.
- Missing livePose: pitch = NaN; excluded from grade-filtered fits only.
- COD API unreachable: clear error pointing at `ssh c3` / device power state.

## Success criteria

- Phase 1: report generated from existing routes with coverage + signal verdict,
  reviewed at the gate.
- Phase 2: on-road behavior tracks aTarget with less sluggish/harsh mismatch than
  the threshold version, verified on-car across low/mid/high speed.
