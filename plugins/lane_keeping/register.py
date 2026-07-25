"""Driver-side lane keeping — hook entry, config loading, telemetry.

Registers on controls.curvature_correction (controls process) and
modeld.calib_bias (modeld process). Phase 4 adds the calibration-trim
wiring: the trim LAW runs alongside the anchor here (where gap_dc lives),
writes δ to data/CalibTrimYawDeg at 1 Hz, and the modeld-side hook reads
that file back as a cached float.
"""
import importlib.util
import math
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


# The trim law (calib_trim.py) is loaded the same way as the anchor — by
# explicit path under a unique module name, never off sys.path. It is a pure,
# dependency-free dataclass module (no cereal/zmq), so it is cheap to import in
# either process; the modeld-side reader only touches TrimConfig()'s default.
_trim_mod = None


def _trim_module():
  global _trim_mod
  if _trim_mod is None:
    spec = importlib.util.spec_from_file_location(
      'lane_keeping_calib_trim', os.path.join(_PLUGIN_DIR, 'calib_trim.py'))
    _trim_mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(_trim_mod)
  return _trim_mod


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
    kappa_filter_tau=fget('LaneKeepKappaFilterTau', d.kappa_filter_tau),
    prob_on=fget('LaneKeepProbOn', d.prob_on),
    prob_fade=fget('LaneKeepProbFade', d.prob_fade),
    dc_tau=fget('LaneKeepDcTau', d.dc_tau),
    ac_deadband=fget('LaneKeepAcDeadband', d.ac_deadband),
    pred_delay_mult=fget('LaneKeepPredDelayMult', d.pred_delay_mult),
    gap_hard_lo=fget('LaneKeepGapHardLo', d.gap_hard_lo),
    gap_hard_hi=fget('LaneKeepGapHardHi', d.gap_hard_hi),
  )


def _load_trim_config():
  TrimConfig = _trim_module().TrimConfig
  d = TrimConfig()

  def fget(key, dflt):
    v = _read_param(key)
    return float(v) if v else dflt

  def iget(key, dflt):
    v = _read_param(key)
    return int(v) if v else dflt

  return TrimConfig(
    mode=iget('CalibTrimMode', d.mode),
    fixed_deg=fget('CalibTrimFixedDeg', d.fixed_deg),
    max_deg=fget('CalibTrimMaxDeg', d.max_deg),
    slew_deg_s=fget('CalibTrimSlewDegS', d.slew_deg_s),
    yaw_sign=iget('CalibTrimYawSign', d.yaw_sign),
    ki=fget('CalibTrimKi', d.ki),
    gap_lo=fget('CalibTrimGapLo', d.gap_lo),
    gap_hi=fget('CalibTrimGapHi', d.gap_hi),
  )


_anchor = None
_trim = None
_pub = None
_tick = 0
_last_yaw_written = None
# modeld-side reader cache: a plain float refreshed off disk every 100 calls.
# Lives at module scope so it survives across frames without any anchor/trim
# state (separate process from the controls-side singletons above).
_calib_bias_cache = {'val': 0.0, 'calls': 0}
# Mirrors TrimConfig.max_deg's default (0.8). The modeld-side reader must not
# import calib_trim.py at all (spec §6: "float file read, nothing else") — this
# constant is the deliberate, independent stand-in. modeld's own ±1.0 call-site
# clamp is a separate, independent line of defense on top of this one.
_CLAMP_DEG_DEFAULT = 0.8


def _write_yaw_file(delta):
  # Atomic 1 Hz publish of δ (deg) to the plugin data dir. Skip when the value
  # is unchanged at 0.001° resolution so modeld's reader doesn't churn on
  # identical bytes and the file's mtime reflects real motion.
  global _last_yaw_written
  r = round(delta, 3)
  if r == _last_yaw_written:
    return
  path = os.path.join(_PLUGIN_DIR, 'data', 'CalibTrimYawDeg')
  tmp = path + '.tmp'
  with open(tmp, 'w') as f:
    f.write(f"{delta:.3f}")
  os.replace(tmp, path)   # atomic rename: readers never see a partial write
  _last_yaw_written = r


def _clamp_bound():
  # ±max_deg *default* — NOT the configured param, and NOT read from calib_trim.py
  # (spec §6: the modeld-side reader path must not import the trim module — pure
  # float file read, nothing else). See _CLAMP_DEG_DEFAULT above.
  return _CLAMP_DEG_DEFAULT


def _read_yaw_deg():
  # Pure float file read for the modeld process: missing/unparseable/nonfinite
  # → 0.0; result clamped to ±max_deg default. No anchor/trim state, no config
  # load, no trim-module import — this runs inside modeld's frame loop.
  try:
    with open(os.path.join(_PLUGIN_DIR, 'data', 'CalibTrimYawDeg')) as f:
      v = float(f.read().strip())
    if not math.isfinite(v):
      return 0.0
    m = _clamp_bound()
    return max(-m, min(m, v))
  except Exception:
    return 0.0


def on_calib_bias(default):
  # modeld.calib_bias hook. Refresh the cached δ off disk every 100 calls
  # (~5 s at model rate); return the cached float otherwise. hooks.run already
  # substitutes `default` on any exception, so a raise here fails safe to 0.0.
  c = _calib_bias_cache
  if c['calls'] % 100 == 0:
    c['val'] = _read_yaw_deg()
  c['calls'] += 1
  return c['val']


def _publish(telem):
  global _pub
  if _pub is None:
    from openpilot.selfdrive.plugins.plugin_bus import PluginPub
    _pub = PluginPub('lane_keeping')
  _pub.send(telem)


def on_curvature_correction(curvature, model_v2, v_ego, lane_changing, lat_delay=None):
  global _anchor, _trim, _tick
  if _anchor is None:
    LaneAnchor = _anchor_module().LaneAnchor
    _anchor = LaneAnchor(_load_config())
  # Live enable toggle (~1 s latency): the Driving-panel switch writes
  # data/LaneKeepEnable; re-read it cheaply every 100 ticks. NOTE: disabling
  # must NOT short-circuit this hook — since Phase 2 the BMW tracker needs
  # the smoothed reference unconditionally (raw kappa_des against a
  # deadzone-free tracker is the documented-unsafe rollback). cfg.enable
  # only gates the POSITION correction inside update(): the bias
  # rate-releases, the stabilizer releases smoothly, smoothing stays on.
  _tick += 1
  if _tick % 100 == 0:
    _anchor.cfg.enable = _read_param('LaneKeepEnable') not in ('0', 'false', 'False')
  new_curvature, telem = _anchor.update(curvature, model_v2, v_ego, lane_changing, lat_delay)
  # Calibration trim (Phase 4): step the pure law with the anchor's own gating
  # signals and merge its telemetry into the SINGLE published message. Wrapped
  # so a config-load or law fault can never break the control path — δ simply
  # stops updating and the last file value ages out under modeld's clamp.
  try:
    if _trim is None:
      CalibTrim = _trim_module().CalibTrim
      _trim = CalibTrim(_load_trim_config())
    delta_deg, ttelem = _trim.update(
      _anchor.gap_dc, telem.get('authority', 0.0),
      lane_changing, v_ego, _anchor.cfg.enable)
    telem['trim_delta_deg'] = ttelem.get('delta_deg')
    telem['trim_err'] = ttelem.get('err')
    telem['trim_mode'] = ttelem.get('mode')
    telem['trim_integrating'] = ttelem.get('integrating')
    if _tick % 100 == 0:
      _write_yaw_file(delta_deg)
  except Exception:
    pass  # trim must never break the control path
  try:
    _publish(telem)
  except Exception:
    pass  # telemetry must never break the control path
  return new_curvature
