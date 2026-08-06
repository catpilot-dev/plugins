# Lane Keeping

Keeps the car from slowly swaying left and right inside its lane.

## What it does

openpilot's end-to-end driving model is good at choosing *where* to drive,
but its steering plan carries a slow side-to-side wander — over roughly ten
seconds the car drifts a little toward one lane line, then back toward the
other. It stays inside the lane, but you can feel the sway, especially on
the highway.

This plugin watches the distance between the driver-side wheels and the
driver-side lane line and gently damps that wander: when the car starts
drifting away from where the model itself has been holding the lane, it adds
a small, bounded steering correction back. The result is the same lane
position, held more steadily.

Two things it deliberately does **not** do:

- It does not pick the car's position in the lane. The model decides that;
  the plugin only calms the motion *around* the model's own choice. If the
  model moves over — for a wide truck, a merge, a curve — the plugin follows
  within seconds.
- It is not a lane-departure or emergency system. Corrections are small and
  rate-limited (about a tenth of the steering authority of a normal turn).

## How to turn it on and off

**Settings → Driving → "Lane Keeping"** — a single toggle, on by default.
It takes effect within a second, even while driving, so you can A/B it on
the move: toggle it off on a straight road and the sway usually becomes
noticeable; toggle it back on and the car settles.

While the plugin is actively anchored to a lane line, a **green ring** is
drawn around the emblem button on the driving screen. The ring goes dark
when the line is lost (worn paint, occlusion) or during a lane change — the
plugin pauses itself automatically in both cases and resumes on its own.

## When it works

- The driver-side lane line must be visible and confidently detected. On
  well-marked roads the plugin is anchored well over 90% of the time; on
  worn markings it simply does less, never something wrong.
- During lane changes it fully disengages (zero correction) and re-anchors
  in the new lane with no memory of the old one.
- It is vehicle-agnostic: the only car-specific settings are the car's
  half-width and which side the driver sits on.

## Turning it off completely

The toggle above is the supported off switch. Fully removing the plugin
(`.disabled` marker) is **not** the same thing: the BMW lateral controller
relies on this plugin's smoothing of the model's steering reference, so full
removal must be paired with a controller revert. See [DESIGN.md](DESIGN.md)
for the details — for everyday use, the toggle is all you need.

## More

Implementation details, tuning parameters, telemetry, and the design
history are in [DESIGN.md](DESIGN.md).
