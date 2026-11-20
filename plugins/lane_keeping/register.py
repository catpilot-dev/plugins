"""Lateral reference conditioner — hook entry, config loading, telemetry.

Registers on controls.curvature_correction (controls process). See anchor.py
for the law and DESIGN.md for the design.
"""
import importlib.util
import os
import sys

_PLUGIN_DIR = os.path.dirname(os.path.abspath(__file__))

# Load the sibling `anchor` module by explicit path rather than a bare
# `from anchor import` off sys.path. A sys.path insert here would leak the
# plugin dir into the global path and shadow other plugins' same-named modules
# (e.g. bmw's `register`) when the test suite runs everything together. The
# runtime registry loads plugins under canonical names for the same reason.
_anchor_mod = None


def _anchor_module():
  global _anchor_mod
  if _anchor_mod is None:
    spec = importlib.util.spec_from_file_location(
      'lane_keeping_anchor', os.path.join(_PLUGIN_DIR, 'anchor.py'))
    _anchor_mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(_anchor_mod)
  return _anchor_mod


def _read_param(key, default='', strip=True):
  try:
    with open(os.path.join(_PLUGIN_DIR, 'data', key)) as f:
      v = f.read()
      return v.strip() if strip else v
  except (FileNotFoundError, OSError):
    return default


def _arc_smooth_enabled():
  """Arc Smooth toggle (kept 2026-11-20 for evaluating new models). Only the exact string '0'
  means OFF; missing, unreadable or any other value is ON. Shared by the
  controls hook and the UI toggle so both parse identically."""
  return _read_param('LaneKeepArcSmoothTest', strip=False) != '0'


def _load_config():
  AnchorConfig = _anchor_module().AnchorConfig
  d = AnchorConfig()

  def fget(key, dflt):
    v = _read_param(key)
    return float(v) if v else dflt

  return AnchorConfig(
    kappa_filter_tau=fget('LaneKeepKappaFilterTau', d.kappa_filter_tau),
    arc_horizon=fget('LaneKeepArcHorizon', d.arc_horizon),
  )


_anchor = None
_pub = None
_arc_smooth = True   # read once, at the same lazy init as _anchor


def _publish(telem):
  global _pub
  if _pub is None:
    from openpilot.selfdrive.plugins.plugin_bus import PluginPub
    _pub = PluginPub('lane_keeping')
  _pub.send(telem)


def on_curvature_correction(curvature, model_v2, v_ego, lane_changing, lat_delay=None):
  global _anchor, _arc_smooth
  if _anchor is None:
    LaneAnchor = _anchor_module().LaneAnchor
    _arc_smooth = _arc_smooth_enabled()
    _anchor = LaneAnchor(_load_config())
  arc_curvature, telem = _anchor.update(curvature, model_v2, v_ego, lane_changing)   # lat_delay: hook signature only
  # Arc Smooth OFF: the Arc keeps running (state, telemetry) but the raw input
  # is returned. The plugin stays loaded so HOOK_READY and the BMW Direct
  # controller are unaffected.
  new_curvature = arc_curvature if _arc_smooth else curvature
  try:
    telem['kappa_arc_ref'] = float(arc_curvature)
    telem['arc_bypass'] = not _arc_smooth
    telem['kappa_ref'] = float(new_curvature)
  except Exception:
    pass
  try:
    _publish(telem)
  except Exception:
    pass  # telemetry must never break the control path
  return new_curvature


def _write_param(key, value):
  data_dir = os.path.join(_PLUGIN_DIR, 'data')
  os.makedirs(data_dir, exist_ok=True)
  with open(os.path.join(data_dir, key), 'w') as f:
    f.write(value)


def on_vehicle_settings(items, CP):
  """ui.vehicle_settings: BMW-only "Arc Smooth" toggle (default ON)."""
  if CP.brand != 'bmw':
    return items

  from openpilot.system.ui.widgets.list_view import toggle_item

  items.append(toggle_item(
    "Arc Smooth",
    "Off = steer on the raw model curvature (supervised test only). Takes effect on the next drive. "
    "To restore mid-drive: disengage, stop, turn ON, cycle the ignition.",
    _arc_smooth_enabled(),
    callback=lambda state: _write_param('LaneKeepArcSmoothTest', '1' if state else '0'),
  ))
  return items


# Last statement: the BMW controller patches its lateral law only when this is
# set (a module the registry left half-executed in sys.modules lacks it).
HOOK_READY = True
