# Driver-Side Lane Keeping (AC Wander Damper)

Standalone plugin on `controls.curvature_correction`. Damps the sub-Hz
lane wander of the e2e driving model — one of openpilot e2e's three known
issues (curve cutting, sub-Hz oscillation, curve blindness; this plugin
addresses the second) — without ever fighting the model for position.

How it works, in one paragraph: the wheel-to-line gap on the driver side
is measured (plan-based prediction at `v · 1.5 · lat_delay`, bias-cancelling
by construction) and split into DC and AC. The DC — where the model chooses
to place the car — is **conceded** via a slow tracker (`LaneKeepDcTau`,
field-proven 5 s): the model closes its own position loop through the
camera with unbounded authority, so every sustained output-side push loses
(field result, routes 3c1/3c3/3c5). The AC — the wander around that line —
is damped through a bounded, rate-limited pure-pursuit curvature bias.
Near the line an **asymmetric gate** (`LaneKeepAsymGap`, 0.6 m) suppresses
corrections *toward* the line so recoveries are never opposed. The damper
core is width-independent (the constant half-width cancels in the DC/AC
split); only the asym threshold consumes `LaneKeepHalfWidth`, making the
plugin vehicle-agnostic in one parameter.

The model curvature reference is also lightly conditioned (0.15 s low-pass,
frame-jitter only — sub-Hz content deliberately passes; filtering it would
add group delay to curve entries). With no confident driver-side line the
position correction is off; the smoothing always runs.

## Configuration

Params are files in `data/` (full list: `_load_config` in `register.py`),
read at process start except the live toggle:

| param | field-proven value | note |
|---|---|---|
| `LaneKeepEnable` | 1 | **live** (~1 s): the Driving-panel toggle; gates only the position correction, never the smoothing |
| `LaneKeepDriverSide` | left | China |
| `LaneKeepDcTau` | 5 | concession time constant (s); code default 20 |
| `LaneKeepAsymGap` | 0.6 (default) | never-oppose-recovery threshold (m); 0 = symmetric |
| `LaneKeepLpMax` | 25 (default) | pure-pursuit aim-point cap (m), reached at 60 km/h; keeps damper authority from collapsing as 1/v² at highway speed (route 3e7 segs 41–48: only ~30% of the model's sub-Hz wander was cancelled at 82 km/h) |
| `LaneKeepGapHardLo` / `Hi` | **−99 / 99 (floors disabled)** | code defaults 0.3/1.5; field testing showed the floors' sustained push turns brief line touches into pinned stalemates (3c3: 34 s holds vs model-alone 2.8 s) — leave disabled |

UX: green ring on the emblem button while anchored; the Driving-panel
"Lane Keeping" toggle is the single source of control. Enforced plugin
(`.enforced`): the BMW lateral controller's simplified tracker depends on
the smoothed reference, so full removal (`.disabled`) is coupled to
reverting that controller.

## Design history

Specs in `docs/superpowers/specs/`, newest governs:
`2026-07-23-ac-stabilizer-design.md` (+ its 2026-07-27 asymmetric-damping
addendum) supersedes the predictive-deadband and 2026-07-22 positioner
specs. The arc — absolute band → predictive deadband → integral trim →
AC/DC stabilizer → floors removed → asymmetric gate — is traceable through
the supersession banners; the one-line summary is that every mechanism
which held an *opinion about position* was removed after losing to the
model in the field, and what remains is a pure damper.

## Calibration trim (retired)

`calib_trim.py` and its `modeld.calib_bias` reader remain in the tree but
are inert: `CalibTrimMode=0` by default, and the modeld-side call sites
were never deployed (archived on the catpilot `calib-trim-parked` branch,
2026-07-29). It was a perception-side DC lever designed to move the
model's chosen line by biasing the calibration yaw — built and reviewed,
then retired when the hard-floor removal dissolved the problem it
targeted. Design record: `2026-07-25-calibration-trim-design.md`.
