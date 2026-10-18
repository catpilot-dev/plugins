"""Driver-side lane keeping — hook entry, config loading, telemetry.

Registers on controls.curvature_correction (controls process).
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


def _read_param(key, default=''):
  try:
    with open(os.path.join(_PLUGIN_DIR, 'data', key)) as f:
      return f.read().strip()
  except (FileNotFoundError, OSError):
    return default


# Plugin-level switch defaults (2026-10-12, routes 4d9/4da/4dc): Arc Smoothing
# ON, the lane-line position correction OFF. Arc alone was the calmest; the
# position correction stacked on the model's own lane-centering loop and
# amplified swings. AnchorConfig's own defaults are the control core's and stay
# as they are; these decide what an absent param file means.
LANE_KEEP_DEFAULT = False
ARC_SMOOTH_DEFAULT = True


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
    enable=bget('LaneKeepEnable', LANE_KEEP_DEFAULT),
    driver_side=sget('LaneKeepDriverSide', d.driver_side),
    half_width=fget('LaneKeepHalfWidth', d.half_width),
    gap_min=fget('LaneKeepGapMin', d.gap_min),
    gap_max=fget('LaneKeepGapMax', d.gap_max),
    t_preview=fget('LaneKeepTPreview', d.t_preview),
    lp_max=fget('LaneKeepLpMax', d.lp_max),
    excess_max=fget('LaneKeepExcessMax', d.excess_max),
    kappa_bias_max=fget('LaneKeepKappaBiasMax', d.kappa_bias_max),
    kappa_rate_max=fget('LaneKeepKappaRateMax', d.kappa_rate_max),
    filter_tau=fget('LaneKeepFilterTau', d.filter_tau),
    kappa_filter_tau=fget('LaneKeepKappaFilterTau', d.kappa_filter_tau),
    arc_smooth=bget('LaneKeepArcSmooth', ARC_SMOOTH_DEFAULT),
    arc_horizon=fget('LaneKeepArcHorizon', d.arc_horizon),
    prob_on=fget('LaneKeepProbOn', d.prob_on),
    prob_fade=fget('LaneKeepProbFade', d.prob_fade),
    dc_tau=fget('LaneKeepDcTau', d.dc_tau),
    ac_deadband=fget('LaneKeepAcDeadband', d.ac_deadband),
    ac_deadband_hi=fget('LaneKeepAcDeadbandHi', d.ac_deadband_hi),
    pred_delay_mult=fget('LaneKeepPredDelayMult', d.pred_delay_mult),
    gap_hard_lo=fget('LaneKeepGapHardLo', d.gap_hard_lo),
    gap_hard_hi=fget('LaneKeepGapHardHi', d.gap_hard_hi),
    asym_gap=fget('LaneKeepAsymGap', d.asym_gap),
  )


_anchor = None
_pub = None
_tick = 0
_direct_switch = False   # cached BMW DirectTracking switch (refreshed every 100 ticks)
_user_enable = False     # user's lane-keep switches as of the last refresh
_user_arc = ARC_SMOOTH_DEFAULT

_BMW_LAT_MODULE = 'plugins.bmw_e9x_e8x.bmw.latcontroller'


def _read_direct_switch():
  try:
    from config import read_plugin_param
    return read_plugin_param('bmw_e9x_e8x', 'DirectTracking', '') == '1'
  except Exception:
    return False


def _bmw_lat_flag(name):
  # Looked up at call time by canonical registry name; never imported.
  # Missing module/accessor or any failure reads False.
  try:
    fn = getattr(sys.modules.get(_BMW_LAT_MODULE), name, None)
    return bool(fn()) if fn is not None else False
  except Exception:
    return False


def _direct_latched():
  return _bmw_lat_flag('direct_latched')


def _lateral_engaged():
  return _bmw_lat_flag('lateral_engaged')


def _direct_tracking_on():
  # While lateral is engaged only the controller's latch decides (ruling
  # R11): a switch flip mid-engagement must not change the reference under a
  # controller still running the other law. Disengaged, the cached switch
  # pre-applies so the first engaged tick already sees the Direct reference.
  if _lateral_engaged():
    return _direct_latched()
  return _direct_latched() or _direct_switch


def _publish(telem):
  global _pub
  if _pub is None:
    from openpilot.selfdrive.plugins.plugin_bus import PluginPub
    _pub = PluginPub('lane_keeping')
  _pub.send(telem)


def on_curvature_correction(curvature, model_v2, v_ego, lane_changing, lat_delay=None):
  global _anchor, _tick, _direct_switch, _user_enable, _user_arc
  if _anchor is None:
    LaneAnchor = _anchor_module().LaneAnchor
    _anchor = LaneAnchor(_load_config())
    _user_enable = _anchor.cfg.enable
    _user_arc = _anchor.cfg.arc_smooth
    _direct_switch = _read_direct_switch()
  # Live enable toggle (~1 s latency): the Driving-panel switch writes
  # data/LaneKeepEnable; re-read it cheaply every 100 ticks. NOTE: disabling
  # must NOT short-circuit this hook — since Phase 2 the downstream tracker
  # needs the smoothed reference unconditionally (raw kappa_des against a
  # deadzone-free tracker is the documented-unsafe rollback). cfg.enable
  # only gates the POSITION correction inside update(): the bias
  # rate-releases, the stabilizer releases smoothly, smoothing stays on.
  _tick += 1
  if _tick % 100 == 0:
    # Same defaults as _load_config when a switch file is absent.
    _user_enable = _read_param('LaneKeepEnable') in ('1', 'true', 'True')
    _user_arc = _read_param('LaneKeepArcSmooth') not in ('0', 'false', 'False')
    _direct_switch = _read_direct_switch()
  # Direct Tracking owns the lateral reference: Arc on, position correction
  # off. Applied every call so a switch-off while the controller is still
  # latched keeps Arc forced; the user's switches return once it releases.
  if _direct_tracking_on():
    _anchor.cfg.enable = False
    _anchor.cfg.arc_smooth = True
  else:
    _anchor.cfg.enable = _user_enable
    _anchor.cfg.arc_smooth = _user_arc
  new_curvature, telem = _anchor.update(curvature, model_v2, v_ego, lane_changing, lat_delay)
  try:
    _publish(telem)
  except Exception:
    pass  # telemetry must never break the control path
  return new_curvature
