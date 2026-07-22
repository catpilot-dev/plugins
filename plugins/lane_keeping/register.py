"""Driver-side lane keeping — hook entry, config loading, telemetry.

Registers on controls.curvature_correction. Phase 1: coexists with the
existing DRIFT_M controller; MODEL state is a literal passthrough.
"""
import os
import sys

_PLUGIN_DIR = os.path.dirname(os.path.abspath(__file__))
if _PLUGIN_DIR not in sys.path:
  sys.path.insert(0, _PLUGIN_DIR)


def on_curvature_correction(curvature, model_v2, v_ego, lane_changing, lat_delay=None):
  # Task 7 replaces this passthrough with the anchor + telemetry.
  return curvature
