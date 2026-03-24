#!/usr/bin/env python3
"""
Mapd process entry point for plugin system.
Ensures the mapd binary exists and execs it.
"""
import json
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import PARAMS_DIR, plugin_data_dir

LAT_ACCEL_VALUES = [1.5, 2.0, 2.5, 3.0]  # indexed by MapdCurveTargetLatAccel param


def _read_speedlimitd_param(key: str) -> str:
  """Read a persisted speedlimitd plugin param (survives openpilot Params::clearAll)."""
  try:
    return (plugin_data_dir('speedlimitd') / key).read_text().strip()
  except OSError:
    return ''


def _ensure_mapd_settings():
  """Write MapdSettings to PARAMS_DIR from persisted speedlimitd plugin params.

  Called on every mapd startup so the correct curve comfort and speed limit
  settings are applied even after openpilot wipes /data/params/d/ on boot.
  """
  enabled = _read_speedlimitd_param('MapdSpeedLimitControlEnabled') == '1'

  try:
    lat_idx = int(_read_speedlimitd_param('MapdCurveTargetLatAccel') or '0')
  except ValueError:
    lat_idx = 0
  lat_accel = LAT_ACCEL_VALUES[lat_idx] if 0 <= lat_idx < len(LAT_ACCEL_VALUES) else 1.5

  settings = {
    'speed_limit_control_enabled': enabled,
    'map_curve_speed_control_enabled': True,   # always on — curve control is independent of speed limit toggle
    'vision_curve_speed_control_enabled': True, # always on — toggle removed by design
    'speed_limit_offset': 0.0,   # no offset — planner_hook applies tiered offset
    'map_curve_target_lat_a': lat_accel,
    'vision_curve_target_lat_a': lat_accel,
  }

  os.makedirs(PARAMS_DIR, exist_ok=True)
  with open(os.path.join(PARAMS_DIR, 'MapdSettings'), 'w') as f:
    f.write(json.dumps(settings))


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
