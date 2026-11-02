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


def test_hook_ready_is_set_after_a_full_load():
  """The BMW controller patches its lateral law only on this sentinel."""
  assert register.HOOK_READY is True


def test_passthrough_returns_input_curvature():
  mv = SimpleNamespace(laneLines=[], laneLineProbs=[])
  out = register.on_curvature_correction(0.0123, mv, 25.0, False, lat_delay=0.45)
  assert out == 0.0123


def test_hook_survives_pub_failure(data_dir, monkeypatch):
  monkeypatch.setattr(register, '_PLUGIN_DIR', str(data_dir.parent))
  # force telemetry publish to raise — the control path must still return the reference
  monkeypatch.setattr(register, '_publish', lambda telem: (_ for _ in ()).throw(RuntimeError('no bus')))
  mv = SimpleNamespace(laneLines=[], laneLineProbs=[])
  out = None
  for _ in range(200):
    out = register.on_curvature_correction(0.01, mv, 25.0, False, lat_delay=0.45)
  assert out == pytest.approx(0.01)   # no plan: the low-passed model curvature


def test_hook_accepts_lat_delay(data_dir, monkeypatch):
  """controlsd passes lat_delay; the hook keeps the parameter and ignores it."""
  monkeypatch.setattr(register, '_publish', lambda telem: None)
  mv = SimpleNamespace(laneLines=[], laneLineProbs=[])
  assert register.on_curvature_correction(0.0, mv, 25.0, False, lat_delay=0.55) == 0.0


def test_load_config_kappa_filter_tau(data_dir):
  cfg = register._load_config()
  assert cfg.kappa_filter_tau == 0.15         # default
  (data_dir / 'LaneKeepKappaFilterTau').write_text('0.45')
  cfg2 = register._load_config()
  assert cfg2.kappa_filter_tau == 0.45


def test_load_config_arc_params(data_dir):
  cfg = register._load_config()
  assert cfg.arc_horizon == 0.0    # default: auto horizon
  (data_dir / 'LaneKeepArcHorizon').write_text('2.0')
  cfg = register._load_config()
  assert cfg.arc_horizon == 2.0


def test_reference_is_always_arc_and_never_position_corrected(data_dir, monkeypatch):
  """The BMW controller is the Direct law unconditionally, so this plugin
  always serves the Arc reference and never the position correction: the old
  switch files are ignored and the Direct cross-plugin machinery is gone."""
  monkeypatch.setattr(register, '_publish', lambda telem: None)
  (data_dir / 'LaneKeepEnable').write_text('1')
  (data_dir / 'LaneKeepArcSmooth').write_text('0')
  for _ in range(150):
    register.on_curvature_correction(0.0, SimpleNamespace(laneLines=[], laneLineProbs=[]), 20.0, False, 0.9)
  assert register._anchor.cfg.arc_horizon == 0.0
  assert not hasattr(register, '_direct_tracking_on')
  assert not hasattr(register._anchor.cfg, 'enable')
