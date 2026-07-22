import os, sys
from types import SimpleNamespace
_PLUGIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PLUGIN_DIR not in sys.path:
  sys.path.insert(0, _PLUGIN_DIR)


def test_passthrough_returns_input_curvature():
  import register
  mv = SimpleNamespace(laneLines=[], laneLineProbs=[])
  out = register.on_curvature_correction(0.0123, mv, 25.0, False, lat_delay=0.45)
  assert out == 0.0123


import pytest


@pytest.fixture
def data_dir(tmp_path, monkeypatch):
  import register
  d = tmp_path / 'data'
  d.mkdir()
  monkeypatch.setattr(register, '_PLUGIN_DIR', str(tmp_path))
  return d


def test_load_config_defaults(data_dir):
  import register
  cfg = register._load_config()
  assert cfg.enable is True
  assert cfg.driver_side == 'left'
  assert cfg.gap_min == 0.6 and cfg.gap_max == 1.0
  assert cfg.t_preview == 1.5


def test_load_config_overrides(data_dir):
  import register
  (data_dir / 'LaneKeepDriverSide').write_text('right')
  (data_dir / 'LaneKeepGapMin').write_text('0.5')
  (data_dir / 'LaneKeepEnable').write_text('0')
  cfg = register._load_config()
  assert cfg.driver_side == 'right'
  assert cfg.gap_min == 0.5
  assert cfg.enable is False


def test_hook_applies_bias_and_survives_pub_failure(data_dir, monkeypatch):
  import importlib, register
  importlib.reload(register)
  monkeypatch.setattr(register, '_PLUGIN_DIR', str(data_dir.parent))
  # force telemetry publish to raise — control path must still return a value
  monkeypatch.setattr(register, '_publish', lambda telem: (_ for _ in ()).throw(RuntimeError('no bus')))
  register._anchor = None
  mv = SimpleNamespace(
    laneLines=[SimpleNamespace(y=[0.0]), SimpleNamespace(y=[2.3]),
               SimpleNamespace(y=[-1.2]), SimpleNamespace(y=[0.0])],
    laneLineProbs=[0.0, 1.0, 1.0, 0.0])
  out = None
  for _ in range(2000):
    out = register.on_curvature_correction(0.01, mv, 25.0, False, lat_delay=0.45)
  assert out > 0.01           # biased left (too far from left line), pub error swallowed


def test_hook_passthrough_when_disabled(data_dir, monkeypatch):
  import importlib, register
  importlib.reload(register)
  monkeypatch.setattr(register, '_PLUGIN_DIR', str(data_dir.parent))
  (data_dir / 'LaneKeepEnable').write_text('0')
  register._anchor = None
  mv = SimpleNamespace(laneLines=[], laneLineProbs=[])
  assert register.on_curvature_correction(0.0123, mv, 25.0, False) == 0.0123
