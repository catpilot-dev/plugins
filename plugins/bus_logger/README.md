# Bus Logger

Records what other plugins are saying to each other, so you can see it later.

## What it does

Several plugins (speedlimitd, lane_keeping, and others) publish their internal
state onto a lightweight local "plugin bus" — small JSON messages used for
inter-plugin communication and on-screen overlays. That bus normally isn't
visible outside the device.

This plugin watches the bus, buffers whatever passes through it, and every
200 ms writes a `pluginBusLog` cereal message containing all the entries seen
(topic name, JSON payload, and timestamp). Because it's a real cereal
message, it gets picked up by loggerd and ends up in the rlog for every
drive — the same place carState, modelV2, and everything else live.

The point is post-drive debugging: if you're digging into why speedlimitd or
lane_keeping did something on a route, the bus_logger entries in that route's
rlog show you what those plugins were internally publishing at the time,
without needing to have been watching the device live.

## Is this a driver feature?

No. There's nothing to look at or interact with while driving — no toggle,
no HUD element, no setting. It's background tooling for whoever's inspecting
logs afterward. It runs automatically while onroad and needs no attention
from the driver.

## How it works

- Hooked into `device.health_check`, which reports whether the logger
  process is alive and how many bus topics it currently sees.
- Runs as its own background process (only while onroad), scanning for
  active plugin-bus topics, subscribing to all of them, and flushing the
  buffered messages to cereal 5 times a second.
- New publishers on the bus are picked up automatically within one cycle —
  nothing needs to register with bus_logger directly.

## Limits

- It only captures what plugins choose to publish on the plugin bus — it
  has no visibility into plugins that don't use that bus.
- It adds a small, constant background load (polling + a 5 Hz cereal
  publish) whenever the car is onroad.
