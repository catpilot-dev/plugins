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
