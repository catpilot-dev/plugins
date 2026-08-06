# BMW E9x / E8x

Teaches openpilot how to drive a BMW E8x or E9x — the car interface that lets
the system read the car's sensors, steer it, and work its cruise control.

## What it does

openpilot ships with support for a fixed set of cars. This plugin adds the
BMW E8x/E9x family to that list. With the plugin enabled and the car
fingerprinted, openpilot can:

- **Steer** the car through the aftermarket Ocelot stepper servo on the
  hydraulic rack, using a lateral controller written specifically for this
  rack (it holds an angle with no motor torque, unlike the electric racks
  most cars have). The steering feel and tuning have their own deep-dive in
  [LATERAL_CONTROLLER.md](LATERAL_CONTROLLER.md).
- **Control cruise** by emulating presses of the cruise stalk. On a car with
  Dynamic Cruise Control (DCC) it nudges the set-speed up and down to hit
  openpilot's target speed and to slow for lead cars and curves.
- **Follow lane changes** — because the plugin reports the turn-signal stalk
  to openpilot, the stock nudge lane-change behaviour works normally.
- **Show the car** — the Driving settings panel shows the BMW emblem and the
  detected model, and (optionally) coolant and oil temperature are drawn on
  the driving screen.

Steering and cruise are gated by the Panda safety model (compiled C running
on the Panda itself), which enforces torque and rate limits independently of
the software above it.

## Supported cars

Detection is **VIN-based** — the plugin reads the model code from the VIN
rather than probing for CAN fingerprints.

| Platform | Cars it covers |
|---|---|
| BMW E82 | 1-Series coupe / convertible (E82 / E88), 2004–13 |
| BMW E90 | 3-Series sedan / wagon / coupe / convertible (E90 / E91 / E92 / E93), 2005–11 |

Both need the DIY cruise-control wiring and, for steering, the Ocelot stepper
servo installed — this is a hardware conversion, not a plug-and-play harness.
Cruise control needs a minimum speed of 30 km/h.

## How to turn it on and off

**Settings → Plugins → "BMW E9x/E8x Car Interface"** — the master toggle.
When it is on, BMW is registered as a supported car and openpilot will
fingerprint the vehicle at startup. When it is off, BMW is not in the system
at all.

Two BMW-specific options appear in **Settings → Driving** (in the vehicle
section that shows only when a BMW is detected):

- **Temperature Overlay** — coolant and oil temperature at the bottom-right
  of the driving screen, colour-coded blue/green/yellow/red from cold to
  critical. On by default.
- **Resume Button Repurposed** — a note, not a toggle: on this car the
  cruise-stalk resume button does double duty. A short press resumes cruise
  when disengaged, or confirms/cancels a detected speed limit when engaged; a
  long press cycles the follow distance.

There is also a **cruise-ceiling memory** (on by default): when you re-engage
cruise within the same drive, it restores the set-speed ceiling you last
dialled in instead of resetting to the default.

## Status and limits

- **This is a hardware-mod car**, not an officially supported openpilot
  vehicle. It depends on an added stepper servo and cruise wiring.
- **DCC is the tested cruise path.** Normal cruise control (NCC) and the
  ACC-module variants are recognised but are less exercised.
- **Cruise below 30 km/h is not available** — the car's own cruise minimum.
- The lateral controller is custom and has been tuned on the E90; see
  [LATERAL_CONTROLLER.md](LATERAL_CONTROLLER.md) for what is and isn't
  settled.

## More

Architecture, the DCC cruise-stalk machinery, fingerprinting, hooks, params,
and telemetry are in [DESIGN.md](DESIGN.md). The lateral controller is
documented separately in [LATERAL_CONTROLLER.md](LATERAL_CONTROLLER.md).
