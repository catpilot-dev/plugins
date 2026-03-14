#!/usr/bin/env python3
"""
Mapd process entry point for plugin system.
Ensures the mapd binary exists and execs it.
"""
import os
import sys

def _ensure_mapd_settings():
  """Write default MapdSettings to /data/params/d/ if missing.

  The Go binary reads this param on startup and warns every second if absent.
  An empty JSON object triggers the binary's built-in defaults.
  """
  params_file = "/data/params/d/MapdSettings"
  if not os.path.exists(params_file):
    os.makedirs("/data/params/d", exist_ok=True)
    with open(params_file, "w") as f:
      f.write("{}")


def main():
  from mapd_manager import ensure_binary, MAPD_PATH
  _ensure_mapd_settings()
  if ensure_binary():
    os.execv(str(MAPD_PATH), [str(MAPD_PATH)])
  else:
    print("ERROR: Failed to ensure mapd binary, exiting", file=sys.stderr)
    sys.exit(1)

if __name__ == "__main__":
  main()
