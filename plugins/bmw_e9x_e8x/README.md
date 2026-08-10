# BMW E9x / E8x

Unofficial support of openpilot on BMW E8x and E9x platforms.

Built on the community port by **dzid** and the
[BMW-E8x-E9x openpilot project](https://github.com/BMW-E8x-E9x/openpilot/wiki),
which worked out the StepperServoCAN steering actuator and the cruise-stalk
emulation this plugin drives.

## Before anything else: this is a hardware conversion

These cars have no factory driver assistance for openpilot to build on, so
nothing here works on a stock car — the car needs the external steering
actuator and cruise wiring fitted first. That build is documented in the
[project wiki](https://github.com/BMW-E8x-E9x/openpilot/wiki); it is the real
project, and this plugin is the software half.

## What this adds to the original port

Three things are meaningfully different here.

**Curvature-based lateral control — the big one.** The original port steered
with a torque controller, the pattern openpilot uses for the electric racks in
supported cars. This car isn't one: it has a stepper servo on a *hydraulic*
rack, which holds its angle on its own at zero torque and ignores small
commands outright. A torque controller assumes the opposite — that the rack
needs sustained effort to stay turned, and self-centers when you stop pushing.
This plugin replaces it with a controller that works in curvature, computing
the torque needed to move the wheels by the remaining curvature error and then
letting stiction hold the angle. The result is far steadier tracking on this
rack. [LATERAL_CONTROLLER.md](LATERAL_CONTROLLER.md) is the full design.

**Fine-tuned DCC cruise control.** Longitudinal control on a DCC car works by
emulating cruise-stalk presses, which is fussy: the stalk message is shared
with the steering-column module, so naive sending collides with the car's own
traffic and drops commands. The plugin owns that message properly and shapes
set-speed changes to openpilot's target, so slowing for lead cars and curves
is smooth rather than steppy.

**VIN-based fingerprinting.** Instead of guessing the car from CAN traffic —
unreliable here, since E82 and E90 look alike on the bus — the plugin reads
the model code straight from the VIN. Detection is deterministic, and the live
CAN fingerprint is then used only for what it's good at: spotting which cruise
ECU the car has.

## Supported cars

Detection is **VIN-based**: the plugin reads the model code from the VIN
rather than probing for CAN fingerprints.

| Platform | Cars it covers |
|---|---|
| BMW E82 | 1-Series coupe / convertible (E82 / E88), 2004–13 |
| BMW E90 | 3-Series sedan / wagon / coupe / convertible (E90 / E91 / E92 / E93), 2005–11 |

## What it does

With the plugin enabled and the car recognized, openpilot can:

- **Steer**, through the external actuator, using the curvature-based
  controller described above.
- **Work the cruise control** by emulating presses of the cruise stalk. On a
  car with Dynamic Cruise Control (DCC) it nudges the set speed up and down to
  follow openpilot's target and to slow for lead cars and curves.
- **Change lanes** normally — the plugin reports the turn-signal stalk, so
  openpilot's usual nudge lane change works.
- **Show the car** — the Driving panel carries the BMW emblem and the detected
  model, and coolant and oil temperature can be drawn on the driving screen.

Cruise control needs at least **30 km/h**; that's the car's own limit, not the
plugin's.

## Settings

**Settings → Plugins → "BMW E9x/E8x Car Interface"** is the master switch.
With it on, BMW is a recognized car and openpilot fingerprints the vehicle at
startup; with it off, BMW isn't in the system at all.

When a BMW is detected, the vehicle section of **Settings → Driving** adds:

- **Temperature Overlay** — coolant and oil temperature at the bottom right of
  the driving screen, coloured blue through red from cold to critical. On by
  default.
- **Resume Button Repurposed** — a note rather than a toggle. On this car the
  cruise stalk's resume button does double duty: a short press resumes cruise
  when disengaged, or confirms/cancels a detected speed limit when engaged; a
  long press cycles the follow distance.

**Cruise-ceiling memory** is on by default: re-engaging cruise within a drive
restores the set-speed ceiling you last dialled in instead of resetting.

## Status and limits

This is a community port of an unsupported car. Treat it accordingly.

- **DCC is the tested cruise path.** Normal cruise control (NCC) and the
  ACC-module variants are recognized but much less exercised.
- **Tuned on an E90.** The E82 shares the software but has seen far less road
  time.
- **Panda safety is written but not yet active.** The plugin ships a Panda
  safety model (`safety/bmw.h`) defining a TX allow-list and torque and rate
  limits, and it is what *should* enforce those limits independently of the
  software above it. It is not currently wired into the Panda's dispatch
  registry, so today those limits are enforced only by the plugin's own
  controller, in software. See [DESIGN.md](DESIGN.md).
- **No cruise below 30 km/h**, per the car.

## More

Architecture, the DCC cruise-stalk machinery, fingerprinting, hooks, params
and telemetry are in [DESIGN.md](DESIGN.md). The lateral controller has its
own deep-dive in [LATERAL_CONTROLLER.md](LATERAL_CONTROLLER.md).
