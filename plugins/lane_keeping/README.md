# Lateral Reference Conditioner (`lane_keeping`)

Smooths the steering target the driving model hands to the car.

## What it does

openpilot's driving model outputs a steering curvature at every frame. Taken
one frame at a time it wobbles: over roughly ten seconds the car drifts a
little toward one lane line, then back toward the other. This plugin gives the
car's lateral controller a steadier target instead:

- **An arc through the planned path.** Rather than the model's single
  curvature number, the target is the arc from the car through the point the
  model plans to be at 1.5–2.5 s ahead (further ahead at speed on straights,
  shorter when a bend is coming). Averaging over the path removes most of the
  frame-to-frame wobble without adding delay.
- **A light low-pass** (0.15 s) on top.
- **Lane changes pass straight through** — the model is deliberately moving
  the car, and the plugin gets out of the way.

It does not pick the car's position in the lane and it adds no correction of
its own; the model still decides where the car goes.

## On BMW

The BMW lateral controller is built for this reference and requires the
plugin: with it disabled the stock openpilot steering controller runs
instead. There is no toggle for it in the Driving panel.

## More

Design, constants and telemetry are in [DESIGN.md](DESIGN.md).
