#!/usr/bin/env python3
"""Plugin bus logger — captures all plugin bus messages to cereal at 5 Hz.

Scans /tmp/plugin_bus/ for active topics, subscribes to all, buffers
messages, and publishes a PluginBusLog cereal message every 200 ms.
"""
import json
import os
import time

import cereal.messaging as messaging
from openpilot.common.realtime import Ratekeeper

BUS_DIR = "/tmp/plugin_bus"


def _discover_topics() -> list[str]:
  """List active plugin bus topics from IPC socket files."""
  try:
    return [f for f in os.listdir(BUS_DIR)
            if not f.startswith('.') and os.path.exists(os.path.join(BUS_DIR, f))]
  except FileNotFoundError:
    return []


def main():
  pm = messaging.PubMaster(['pluginBusLog'])
  rk = Ratekeeper(5, print_delay_threshold=None)

  # Lazy import to avoid circular deps at module level
  from openpilot.selfdrive.plugins.plugin_bus import PluginSub

  sub = None
  known_topics: set[str] = set()
  buffer: list[tuple[str, dict, int]] = []  # (topic, data, mono_ns)

  while True:
    # Re-discover topics periodically (new publishers may appear)
    current_topics = set(_discover_topics())
    if current_topics != known_topics:
      known_topics = current_topics
      if sub is not None:
        sub.close()
      if known_topics:
        sub = PluginSub(list(known_topics))
      else:
        sub = None

    # Drain all pending messages
    if sub is not None:
      while True:
        msg = sub.recv()
        if msg is None:
          break
        topic, data = msg
        mono_ns = int(time.monotonic() * 1e9)
        buffer.append((topic, data, mono_ns))

    # Publish buffered entries at 1 Hz
    if buffer:
      msg = messaging.new_message('pluginBusLog')
      log = msg.pluginBusLog
      log.init('entries', len(buffer))
      for i, (topic, data, mono_ns) in enumerate(buffer):
        entry = log.entries[i]
        entry.topic = topic
        entry.json = json.dumps(data)
        entry.monoTime = mono_ns
      pm.send('pluginBusLog', msg)
      buffer.clear()
    else:
      # Send empty message so service stays alive in SubMaster
      msg = messaging.new_message('pluginBusLog')
      log = msg.pluginBusLog
      log.init('entries', 0)
      pm.send('pluginBusLog', msg)

    rk.keep_time()


if __name__ == "__main__":
  main()
