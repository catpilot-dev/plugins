"""Driver-side lane keeping — hook entry, config loading, telemetry.

Registers on controls.curvature_correction. Phase 1: coexists with the
existing DRIFT_M controller; MODEL state is a literal passthrough.
"""
import importlib.util
import os

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


def _read_param(key, default=''):
  try:
    with open(os.path.join(_PLUGIN_DIR, 'data', key)) as f:
      return f.read().strip()
  except (FileNotFoundError, OSError):
    return default


def _load_config():
  AnchorConfig = _anchor_module().AnchorConfig
  d = AnchorConfig()

  def fget(key, dflt):
    v = _read_param(key)
    return float(v) if v else dflt

  def sget(key, dflt):
    v = _read_param(key)
    return v if v else dflt

  def bget(key, dflt):
    v = _read_param(key)
    return dflt if v == '' else v not in ('0', 'false', 'False')

  return AnchorConfig(
    enable=bget('LaneKeepEnable', d.enable),
    driver_side=sget('LaneKeepDriverSide', d.driver_side),
    half_width=fget('LaneKeepHalfWidth', d.half_width),
    gap_min=fget('LaneKeepGapMin', d.gap_min),
    gap_max=fget('LaneKeepGapMax', d.gap_max),
    t_preview=fget('LaneKeepTPreview', d.t_preview),
    excess_max=fget('LaneKeepExcessMax', d.excess_max),
    kappa_bias_max=fget('LaneKeepKappaBiasMax', d.kappa_bias_max),
    kappa_rate_max=fget('LaneKeepKappaRateMax', d.kappa_rate_max),
    filter_tau=fget('LaneKeepFilterTau', d.filter_tau),
    prob_on=fget('LaneKeepProbOn', d.prob_on),
    prob_fade=fget('LaneKeepProbFade', d.prob_fade),
  )


_anchor = None
_pub = None


def _publish(telem):
  global _pub
  if _pub is None:
    from openpilot.selfdrive.plugins.plugin_bus import PluginPub
    _pub = PluginPub('lane_keeping')
  _pub.send(telem)


def on_curvature_correction(curvature, model_v2, v_ego, lane_changing, lat_delay=None):
  global _anchor
  if _anchor is None:
    LaneAnchor = _anchor_module().LaneAnchor
    _anchor = LaneAnchor(_load_config())
  if not _anchor.cfg.enable:
    return curvature
  new_curvature, telem = _anchor.update(curvature, model_v2, v_ego, lane_changing)
  try:
    _publish(telem)
  except Exception:
    pass  # telemetry must never break the control path
  return new_curvature
