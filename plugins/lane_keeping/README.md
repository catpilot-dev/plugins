# Driver-Side Lane Keeping

Standalone plugin on `controls.curvature_correction`. AC STABILIZER: damps
the wander of the driver-side wheel-to-line gap around the model's own chosen
line (the line itself is conceded — a slow DC tracker follows it); hard floors
at 0.3/1.5 m remain absolute. Also conditions the model curvature reference
(low-pass). When no confident driver-side line is present the position
correction is off (smoothing always runs).

Design spec: `docs/superpowers/specs/2026-07-23-ac-stabilizer-design.md`
(supersedes the 2026-07-22 positioner spec and the predictive-deadband §7 trim).
Control core: `anchor.py` (pure, unit-tested). Hook + telemetry: `register.py`.
Params: files in `data/` (see `_load_config` in `register.py`).

## Calibration trim

A slow perception-side DC bias (`calib_trim.py`), separate from the
position-anchor above: it nudges modeld's `calib_bias` yaw by a few tenths of
a degree to correct sustained wheel-to-line gap error at the source, instead
of steering against it every frame. `CalibTrimMode=0` (off) by default —
zero effect until explicitly armed.

- **Mode 0 (off, default):** `delta` slews to 0. No-op.
- **Mode 1 (fixed):** slews `delta` toward `CalibTrimFixedDeg` (clamped to
  ±`CalibTrimMaxDeg`), ungated — used for the sign/gain identification drive.
- **Mode 2 (closed-loop):** integrates `delta` to hold the driver-side gap in
  `[CalibTrimGapLo, CalibTrimGapHi]`; requires `CalibTrimYawSign` to be
  measured (±1) or it behaves as mode 0. Gated on a trusted line, no lane
  change, and `v_ego ≥ 5 m/s`; decays after 5 s continuously in-band.

All transitions are rate-limited by `CalibTrimSlewDegS` (0.02°/s default —
~15-25 s to take hold; this is deliberately not a fast fix). Params live in
`data/` (see `_load_trim_config` in `register.py`); `delta` is written to
`data/CalibTrimYawDeg` at 1 Hz and read back by modeld via the
`modeld.calib_bias` hook — that reader path is intentionally import-free
(pure float file read, clamped to a hand-kept default independent of
`calib_trim.py`).

Design spec: `docs/superpowers/specs/2026-07-25-calibration-trim-design.md`.
Deployment (identification drive, mode 2 enable) is gated on explicit user
go per spec §9 — implementing and testing this does not deploy it.
