#!/usr/bin/env python3
"""
Mapd process entry point for plugin system.
Ensures the mapd binary exists and execs it.

Settings are NOT written here. mapd loads its built-in defaults, then
/data/openpilot/mapd_defaults.json (placed by install.sh), then the
MapdSettings param. Keeping our configuration in the defaults file means it
survives openpilot wiping /data/params/d/ on boot without this process
rewriting it on every start — and leaves exactly one place that decides
whether mapd controls anything. It does not: see mapd_defaults.json.
"""
import os
import sys
import time

# plugind respawns any dead process every POLL_INTERVAL (5 s). Exiting
# immediately on a failed download would therefore hammer the GitHub releases
# API 12x/minute on a device with no network. Back off in-process instead, then
# exit and let plugind schedule the next round.
RETRY_DELAYS = (5, 15, 60, 180)


def main():
  from mapd_manager import ensure_binary, MAPD_PATH
  for attempt, delay in enumerate(RETRY_DELAYS, start=1):
    if ensure_binary():
      os.execv(str(MAPD_PATH), [str(MAPD_PATH)])
    print(f"mapd binary unavailable (attempt {attempt}/{len(RETRY_DELAYS)}), "
          f"retrying in {delay}s", file=sys.stderr)
    time.sleep(delay)
  print("ERROR: Failed to ensure mapd binary after retries, exiting", file=sys.stderr)
  sys.exit(1)


if __name__ == "__main__":
  main()
