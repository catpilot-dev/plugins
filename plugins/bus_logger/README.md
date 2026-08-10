# Bus Logger

Records logs and messages from plugins to rlog.

## What it does

Several plugins (speedlimitd, lane_keeping, and others) publish their internal
state onto a lightweight local "plugin bus" — small JSON messages used for
inter-plugin communication and on-screen overlays.

This plugin watches the bus, buffers whatever passes through it, and every
200 ms writes a `pluginBusLog` cereal message containing all the entries seen
(topic name, JSON payload, and timestamp). Because it's a real cereal
message, loggerd picks it up and it lands in every drive's rlog, ready for
post-drive debugging.

## How it works

- Hooked into `device.health_check`, which reports whether the logger
  process is alive and how many bus topics it currently sees.
- Runs as its own background process, scanning for active plugin-bus topics,
  subscribing to all of them, and flushing the buffered messages to cereal
  5 times a second.
- New publishers on the bus are picked up automatically within one cycle —
  nothing needs to register with bus_logger directly.

## Limits

- It only captures what plugins choose to publish on the plugin bus — it
  has no visibility into plugins that don't use that bus.
- It adds a small, constant background load (polling + a 5 Hz cereal
  publish). The manifest asks for it to run only onroad, but the plugin
  framework does not yet act on that request, so it runs offroad too —
  harmless, since only onroad messages reach a drive's rlog.
