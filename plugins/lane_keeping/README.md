# Driver-Side Lane Keeping

Standalone plugin on `controls.curvature_correction`. Anchors the car's
driver-side wheel-to-line gap in a `[GAP_MIN, GAP_MAX]` deadband via a bounded
pure-pursuit curvature bias. When no confident driver-side line is present it
is a literal passthrough (the existing controller runs unchanged).

Design spec: `docs/superpowers/specs/2026-07-22-driver-side-lane-keeping-design.md`.
Control core: `anchor.py` (pure, unit-tested). Hook + telemetry: `register.py`.
Params: files in `data/` (see `_load_config` in `register.py`).
