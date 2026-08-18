#!/usr/bin/env python3
"""
Mapd process entry point for plugin system.
Writes our declarative settings to the MapdSettings param, ensures the mapd
binary exists, and execs it.

Settings delivery (2026-08-18): mapd_defaults.json (sibling file, the single
declarative source of what mapd is allowed to do — nothing) is written to the
MapdSettings PARAM here on every start, NOT copied to
/data/openpilot/mapd_defaults.json. The custom-defaults file path is UNUSABLE
on mapd v2.3.0: settings.go's Default() reads the file with gabs (JSON numbers
become float64), compares the version against a uint64 (always "mismatched"),
then either panics on `settingsVersion.(uint64)` (version present) or panics on
a nil-interface assertion in Migrate() (version absent). The param path in
Load() compares float64 to float64 and is the one mapd itself round-trips, so
it is safe. Writing on every start also survives openpilot wiping
/data/params/d/ on boot. install.sh actively REMOVES any stray
/data/openpilot/mapd_defaults.json for the same reason.
"""
import json
import os
import sys
import time

# plugind respawns any dead process every POLL_INTERVAL (5 s). Exiting
# immediately on a failed download would therefore hammer the GitHub releases
# API 12x/minute on a device with no network. Back off in-process instead, then
# exit and let plugind schedule the next round.
RETRY_DELAYS = (5, 15, 60, 180)

DEFAULTS_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mapd_defaults.json")


def write_settings_param():
  """Copy mapd_defaults.json into the MapdSettings param.

  Best-effort: a failure must not block the mapd launch — mapd then runs on
  its internal defaults, which is degraded (its control features default on)
  but still harmless in Phase 1, where nothing consumes mapd's control
  outputs. Failures are logged so a drive with wrong settings is explicable.
  """
  try:
    from config import PARAMS_DIR
    with open(DEFAULTS_PATH) as f:
      settings = json.load(f)          # validate before writing
    os.makedirs(PARAMS_DIR, exist_ok=True)
    tmp = os.path.join(PARAMS_DIR, ".MapdSettings.tmp")
    with open(tmp, "w") as f:
      f.write(json.dumps(settings))
    os.replace(tmp, os.path.join(PARAMS_DIR, "MapdSettings"))
    return True
  except Exception as e:
    print(f"WARNING: could not write MapdSettings param: {e}", file=sys.stderr)
    return False


def main():
  from mapd_manager import ensure_binary, MAPD_PATH
  write_settings_param()
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
