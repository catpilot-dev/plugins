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
  cfg = register._load_config()
  assert cfg.enable is True
  assert cfg.driver_side == 'left'
  assert cfg.gap_min == 0.6 and cfg.gap_max == 1.0
  assert cfg.t_preview == 1.5
  assert cfg.lp_max == 25.0


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
  assert out > 0.01           # biased left (too far from left line), pub error swallowed


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
  assert cfg.ac_deadband_hi == 0.05
  (data_dir / 'LaneKeepDcTau').write_text('30')
  (data_dir / 'LaneKeepAcDeadbandHi').write_text('0.07')
  cfg = register._load_config()
  assert cfg.dc_tau == 30.0
  assert cfg.ac_deadband_hi == 0.07


def test_live_toggle_disables_and_releases(data_dir, monkeypatch):
  monkeypatch.setattr(register, '_PLUGIN_DIR', str(data_dir.parent))
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
  assert out > 1e-5
  (data_dir / 'LaneKeepEnable').write_text('0')
  for i in range(500):                           # toggle off mid-drift: releases
    out = register.on_curvature_correction(0.0, mv_at(1.14 + 0.05*(i/100.0)), 17.0, False, lat_delay=0.45)
  assert abs(out) < 1e-6
