# bus_logger — Plugin Bus Logger

**Type**: daemon | **Process**: 5 Hz, only_onroad

## What it does

Captures all inter-plugin bus messages to cereal for rlog recording and debugging. Scans `/tmp/plugin_bus/` for active ZMQ topics, subscribes to all, buffers messages, and publishes a `pluginBusLog` cereal message every 200 ms.

Topics are auto-discovered — new publishers are picked up within one cycle.

## Cereal Messages

| Message | Direction | Frequency |
|---------|-----------|-----------|
| pluginBusLog | publish | 5 Hz |

Each message contains a list of entries with topic name, JSON payload, and monotonic timestamp.

## Key Files

```
bus_logger/
  plugin.json      # Plugin manifest
  bus_logger.py     # Main daemon loop
  cereal/slot1.capnp  # PluginBusLog schema
```
