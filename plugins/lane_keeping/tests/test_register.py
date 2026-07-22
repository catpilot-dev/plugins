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


def test_load_config_overrides(data_dir):
  (data_dir / 'LaneKeepDriverSide').write_text('right')
  (data_dir / 'LaneKeepGapMin').write_text('0.5')
  (data_dir / 'LaneKeepEnable').write_text('0')
  cfg = register._load_config()
  assert cfg.driver_side == 'right'
  assert cfg.gap_min == 0.5
  assert cfg.enable is False


def test_hook_applies_bias_and_survives_pub_failure(data_dir, monkeypatch):
  monkeypatch.setattr(register, '_PLUGIN_DIR', str(data_dir.parent))
  # force telemetry publish to raise — control path must still return a value
  monkeypatch.setattr(register, '_publish', lambda telem: (_ for _ in ()).throw(RuntimeError('no bus')))
  # left ego line (laneLines[1]) at y=-2.3 (far left, +y=right frame) -> gap 1.39
  # above the band -> steer left (positive, left-positive curvature)
  mv = SimpleNamespace(
    laneLines=[SimpleNamespace(y=[0.0]), SimpleNamespace(y=[-2.3]),
               SimpleNamespace(y=[1.2]), SimpleNamespace(y=[0.0])],
    laneLineProbs=[0.0, 1.0, 1.0, 0.0])
  out = None
  for _ in range(2000):
    out = register.on_curvature_correction(0.01, mv, 25.0, False, lat_delay=0.45)
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
