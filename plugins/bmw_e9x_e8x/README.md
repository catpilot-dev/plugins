# BMW E9x / E8x

Unofficial support of openpilot on BMW E8x and E9x platforms.

Built on the community port by **dzid** and the
[BMW-E8x-E9x openpilot project](https://github.com/BMW-E8x-E9x/openpilot/wiki),
which worked out the CAN messages, the cruise-stalk control and the steering
conversion this plugin relies on. Their wiki is still the reference for the
hardware side.

## Before anything else: this is a hardware conversion

These cars have no factory driver assistance for openpilot to build on, so
nothing here works on a stock car. You need:

- an aftermarket **Ocelot stepper servo** on the steering rack, for steering;
- **DIY cruise-control wiring**, for engaging and setting speed;
- a comma device, wired to PT-CAN and F-CAN.

Fitting all that is the real project — the plugin is the software half.
Start with the [project wiki](https://github.com/BMW-E8x-E9x/openpilot/wiki).

## Supported cars

Detection is **VIN-based**: the plugin reads the model code from the VIN
rather than probing for CAN fingerprints.

| Platform | Cars it covers |
|---|---|
| BMW E82 | 1-Series coupe / convertible (E82 / E88), 2004–13 |
| BMW E90 | 3-Series sedan / wagon / coupe / convertible (E90 / E91 / E92 / E93), 2005–11 |

## What it does

With the plugin enabled and the car recognized, openpilot can:

- **Steer.** The hydraulic rack on these cars holds its angle when the motor
  goes quiet, unlike the electric racks openpilot was built around, so the
  plugin uses a lateral controller written specifically for it. How it behaves
  and how it's tuned is a topic of its own —
  see [LATERAL_CONTROLLER.md](LATERAL_CONTROLLER.md).
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
