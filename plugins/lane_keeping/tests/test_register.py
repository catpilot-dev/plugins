import importlib.util, os, sys
from types import SimpleNamespace
import pytest

# Load register (and the anchor it depends on) by explicit path under unique
# module names — do NOT insert the plugin dir on sys.path. Two plugins both have
# a top-level `register.py`; a bare `import register` after a sys.path insert
# shadows the other plugin's module when the whole suite runs together (breaks
# bmw's tests). The runtime avoids this via canonical registry module names.
_PLUGIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _load(name, unique):
  spec = importlib.util.spec_from_file_location(unique, os.path.join(_PLUGIN_DIR, name + '.py'))
  m = importlib.util.module_from_spec(spec)
  sys.modules[unique] = m
  spec.loader.exec_module(m)
  return m


# register.py loads its sibling anchor by explicit path (no sys.path use), so we
# only need to load register itself under a unique name.
register = _load('register', 'lk_register')


@pytest.fixture(autouse=True)
def _reset_register():
  # Module-level lazy singletons persist across tests; reset before each.
  register._anchor = None
  register._pub = None
  yield


@pytest.fixture
def data_dir(tmp_path, monkeypatch):
  d = tmp_path / 'data'
  d.mkdir()
  monkeypatch.setattr(register, '_PLUGIN_DIR', str(tmp_path))
  return d


def test_passthrough_returns_input_curvature():
  mv = SimpleNamespace(laneLines=[], laneLineProbs=[])
  out = register.on_curvature_correction(0.0123, mv, 25.0, False, lat_delay=0.45)
  assert out == 0.0123


def test_load_config_defaults(data_dir):
  """Defaults since 2026-10-12 (routes 4d9/4da/4dc): Arc Smoothing ON, the
  lane-line position correction OFF - Arc alone was the calmest, and the
  position correction amplified swings on top of the model's own loop."""
  cfg = register._load_config()
  assert cfg.enable is False
  assert cfg.arc_smooth is True
  assert cfg.driver_side == 'left'
  assert cfg.gap_min == 0.6 and cfg.gap_max == 1.0
  assert cfg.t_preview == 1.5
  assert cfg.lp_max == 9999.0     # inert by default (post-3ea rollback)


def test_load_config_overrides(data_dir):
  (data_dir / 'LaneKeepDriverSide').write_text('right')
  (data_dir / 'LaneKeepGapMin').write_text('0.5')
  (data_dir / 'LaneKeepEnable').write_text('0')
  (data_dir / 'LaneKeepLpMax').write_text('30')
  cfg = register._load_config()
  assert cfg.driver_side == 'right'
  assert cfg.gap_min == 0.5
  assert cfg.enable is False
  assert cfg.lp_max == 30.0


def test_hook_applies_bias_and_survives_pub_failure(data_dir, monkeypatch):
  monkeypatch.setattr(register, '_PLUGIN_DIR', str(data_dir.parent))
  (data_dir / 'LaneKeepEnable').write_text('1')   # off by default since 2026-10-12
  # force telemetry publish to raise — control path must still return a value
  monkeypatch.setattr(register, '_publish', lambda telem: (_ for _ in ()).throw(RuntimeError('no bus')))
  # A constant out-of-band gap concedes under the AC stabilizer (zero-mean by
  # construction), so seed the DC tracker then drift away from it -> the AC
  # term builds a bias (left ego line drifting away, +y=right frame).
  def mv_at(gap):
    y = -(gap + 0.91)
    xs = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0]
    return SimpleNamespace(
      laneLines=[SimpleNamespace(x=[], y=[0.0]), SimpleNamespace(x=xs, y=[y]*6),
                 SimpleNamespace(x=xs, y=[1.75]*6), SimpleNamespace(x=[], y=[0.0])],
      laneLineProbs=[0.0, 1.0, 1.0, 0.0],
      position=SimpleNamespace(x=xs, y=[0.0]*6))
  for _ in range(1000):                          # seed DC at 0.84
    register.on_curvature_correction(0.01, mv_at(0.84), 25.0, False, lat_delay=0.45)
  out = None
  for i in range(600):                           # drift -> bias builds, pub error swallowed
    out = register.on_curvature_correction(0.01, mv_at(0.84 + 0.05*(i/100.0)), 25.0, False, lat_delay=0.45)
  assert out < 0.01           # biased left (too far from left line) = negative, pub error swallowed


def test_hook_passthrough_when_disabled(data_dir, monkeypatch):
  monkeypatch.setattr(register, '_PLUGIN_DIR', str(data_dir.parent))
  (data_dir / 'LaneKeepEnable').write_text('0')
  mv = SimpleNamespace(laneLines=[], laneLineProbs=[])
  assert register.on_curvature_correction(0.0123, mv, 25.0, False) == 0.0123


def test_load_config_kappa_filter_tau(data_dir):
  cfg = register._load_config()
  assert cfg.kappa_filter_tau == 0.15         # default
  (data_dir / 'LaneKeepKappaFilterTau').write_text('0.45')
  cfg2 = register._load_config()
  assert cfg2.kappa_filter_tau == 0.45


def test_load_config_predictive_params(data_dir):
  cfg = register._load_config()
  assert cfg.pred_delay_mult == 1.5
  assert cfg.gap_hard_lo == 0.3 and cfg.gap_hard_hi == 1.5
  (data_dir / 'LaneKeepPredDelayMult').write_text('3.0')
  (data_dir / 'LaneKeepGapHardLo').write_text('0.4')
  cfg2 = register._load_config()
  assert cfg2.pred_delay_mult == 3.0
  assert cfg2.gap_hard_lo == 0.4


def test_hook_passes_lat_delay_through(data_dir, monkeypatch):
  monkeypatch.setattr(register, '_PLUGIN_DIR', str(data_dir.parent))
  seen = {}
  class FakeAnchor:
    cfg = type('C', (), {'enable': True})()
    def update(self, curvature, model_v2, v_ego, lane_changing, lat_delay=None):
      seen['lat_delay'] = lat_delay
      return curvature, {}
  register._anchor = FakeAnchor()
  monkeypatch.setattr(register, '_publish', lambda telem: None)
  mv = SimpleNamespace(laneLines=[], laneLineProbs=[])
  register.on_curvature_correction(0.0, mv, 25.0, False, lat_delay=0.55)
  assert seen['lat_delay'] == 0.55


def test_load_config_ac_params(data_dir):
  cfg = register._load_config()
  assert cfg.dc_tau == 20.0 and cfg.ac_deadband == 0.10
  assert cfg.ac_deadband_hi == 0.10   # flat taper by default (post-3ea rollback)
  (data_dir / 'LaneKeepDcTau').write_text('30')
  (data_dir / 'LaneKeepAcDeadbandHi').write_text('0.07')
  cfg = register._load_config()
  assert cfg.dc_tau == 30.0
  assert cfg.ac_deadband_hi == 0.07


def test_live_toggle_disables_and_releases(data_dir, monkeypatch):
  monkeypatch.setattr(register, '_PLUGIN_DIR', str(data_dir.parent))
  (data_dir / 'LaneKeepEnable').write_text('1')   # off by default since 2026-10-12
  monkeypatch.setattr(register, '_publish', lambda telem: None)
  def mv_at(gap):
    y = -(gap + 0.91)
    xs = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0]
    return SimpleNamespace(
      laneLines=[SimpleNamespace(x=[], y=[0.0]), SimpleNamespace(x=xs, y=[y]*6),
                 SimpleNamespace(x=xs, y=[1.75]*6), SimpleNamespace(x=[], y=[0.0])],
      laneLineProbs=[0.0, 1.0, 1.0, 0.0],
      position=SimpleNamespace(x=xs, y=[0.0]*6))
  for _ in range(1000):                          # seed DC at 0.84
    register.on_curvature_correction(0.0, mv_at(0.84), 17.0, False, lat_delay=0.45)
  out = None
  for i in range(600):                           # drift -> stabilizer damps
    out = register.on_curvature_correction(0.0, mv_at(0.84 + 0.05*(i/100.0)), 17.0, False, lat_delay=0.45)
  assert out < -1e-5                            # damping leftward = negative
  (data_dir / 'LaneKeepEnable').write_text('0')
  for i in range(500):                           # toggle off mid-drift: releases
    out = register.on_curvature_correction(0.0, mv_at(1.14 + 0.05*(i/100.0)), 17.0, False, lat_delay=0.45)
  assert abs(out) < 1e-6


def test_load_config_arc_params(data_dir):
  cfg = register._load_config()
  assert cfg.arc_smooth is True and cfg.arc_horizon == 0.0    # defaults: on, auto horizon
  (data_dir / 'LaneKeepArcSmooth').write_text('0')
  assert register._load_config().arc_smooth is False
  (data_dir / 'LaneKeepArcSmooth').write_text('1')
  (data_dir / 'LaneKeepArcHorizon').write_text('2.0')
  cfg = register._load_config()
  assert cfg.arc_smooth is True and cfg.arc_horizon == 2.0


def test_live_arc_toggle(data_dir, monkeypatch):
  """The Driving-panel Arc Smoothing switch takes effect within ~1 s, like
  LaneKeepEnable."""
  monkeypatch.setattr(register, '_publish', lambda telem: None)
  register.on_curvature_correction(0.0, SimpleNamespace(laneLines=[], laneLineProbs=[]), 20.0, False, 0.9)
  assert register._anchor.cfg.arc_smooth is True              # default on
  (data_dir / 'LaneKeepArcSmooth').write_text('0')
  for _ in range(100):
    register.on_curvature_correction(0.0, SimpleNamespace(laneLines=[], laneLineProbs=[]), 20.0, False, 0.9)
  assert register._anchor.cfg.arc_smooth is False


def test_live_defaults_without_param_files(data_dir, monkeypatch):
  """No stored switch files (fresh install): the live re-read must land on the
  same defaults as _load_config - arc on, position correction off."""
  monkeypatch.setattr(register, '_publish', lambda telem: None)
  for _ in range(100):
    register.on_curvature_correction(0.0, SimpleNamespace(laneLines=[], laneLineProbs=[]), 20.0, False, 0.9)
  assert register._anchor.cfg.arc_smooth is True
  assert register._anchor.cfg.enable is False


# ---- BMW Direct Tracking: lane keeping follows Direct mode ----

_BMW_LAT = 'plugins.bmw_e9x_e8x.bmw.latcontroller'


def _direct_env(monkeypatch, data_dir, latched, direct_param, enable='1', arc='0',
                engaged=None):
  monkeypatch.syspath_prepend(os.path.dirname(_PLUGIN_DIR))
  import config
  (data_dir / 'LaneKeepEnable').write_text(enable)
  (data_dir / 'LaneKeepArcSmooth').write_text(arc)
  if latched is None:
    monkeypatch.delitem(sys.modules, _BMW_LAT, raising=False)
  else:
    bmw = SimpleNamespace(direct_latched=lambda: latched)
    if engaged is not None:
      bmw.lateral_engaged = lambda: engaged
    monkeypatch.setitem(sys.modules, _BMW_LAT, bmw)
  monkeypatch.setattr(config, 'read_plugin_param',
                      lambda pid, key, default='': direct_param if key == 'DirectTracking' else default)
  monkeypatch.setattr(register, '_publish', lambda t: None)
  monkeypatch.setattr(register, '_tick', 0)
  monkeypatch.setattr(register, '_direct_switch', False, raising=False)


def _call():
  mv = SimpleNamespace(laneLines=[], laneLineProbs=[])
  return register.on_curvature_correction(0.01, mv, 25.0, False, lat_delay=0.45)


def test_direct_forces_arc_on_and_lane_keep_off(monkeypatch, data_dir):
  _direct_env(monkeypatch, data_dir, True, '1')
  _call()
  assert register._anchor.cfg.arc_smooth is True
  assert register._anchor.cfg.enable is False


def test_switch_off_while_latched_keeps_arc(monkeypatch, data_dir):
  _direct_env(monkeypatch, data_dir, True, '0')
  _call()
  assert register._anchor.cfg.arc_smooth is True
  assert register._anchor.cfg.enable is False


def test_user_switches_respected_when_direct_off(monkeypatch, data_dir):
  _direct_env(monkeypatch, data_dir, None, '')
  _call()
  assert register._anchor.cfg.enable is True
  assert register._anchor.cfg.arc_smooth is False


def test_user_switches_restored_after_latch_releases(monkeypatch, data_dir):
  state = {'on': True}
  _direct_env(monkeypatch, data_dir, True, '0')
  monkeypatch.setitem(sys.modules, _BMW_LAT, SimpleNamespace(direct_latched=lambda: state['on']))
  _call()
  assert register._anchor.cfg.arc_smooth is True
  state['on'] = False
  _call()
  assert register._anchor.cfg.enable is True
  assert register._anchor.cfg.arc_smooth is False


def test_switch_on_without_latch_applies_after_refresh(monkeypatch, data_dir):
  _direct_env(monkeypatch, data_dir, False, '1')
  for _ in range(100):
    _call()
  assert register._anchor.cfg.arc_smooth is True
  assert register._anchor.cfg.enable is False


def test_broken_bmw_module_means_not_latched(monkeypatch, data_dir):
  _direct_env(monkeypatch, data_dir, None, '')
  monkeypatch.setitem(sys.modules, _BMW_LAT, SimpleNamespace(direct_latched=lambda: 1 / 0))
  _call()
  assert register._anchor.cfg.enable is True


# Ruling R11: no mid-engagement change — while lateral is engaged only the
# controller's latch decides; the cached switch applies only while disengaged.

def test_switch_on_mid_engagement_keeps_user_switches(monkeypatch, data_dir):
  _direct_env(monkeypatch, data_dir, False, '1', engaged=True)
  for _ in range(100):
    _call()
  assert register._direct_switch is True                  # cached, not applied
  assert register._anchor.cfg.enable is True
  assert register._anchor.cfg.arc_smooth is False


def test_latched_while_engaged_forces_direct(monkeypatch, data_dir):
  _direct_env(monkeypatch, data_dir, True, '0', engaged=True)
  _call()
  assert register._anchor.cfg.arc_smooth is True
  assert register._anchor.cfg.enable is False


def test_switch_on_while_disengaged_applies(monkeypatch, data_dir):
  _direct_env(monkeypatch, data_dir, False, '1', engaged=False)
  for _ in range(100):
    _call()
  assert register._anchor.cfg.arc_smooth is True
  assert register._anchor.cfg.enable is False


def test_engaged_lookup_failure_means_not_engaged(monkeypatch, data_dir):
  _direct_env(monkeypatch, data_dir, False, '1')
  monkeypatch.setitem(sys.modules, _BMW_LAT, SimpleNamespace(
    direct_latched=lambda: False, lateral_engaged=lambda: 1 / 0))
  for _ in range(100):
    _call()
  assert register._anchor.cfg.arc_smooth is True
  assert register._anchor.cfg.enable is False
