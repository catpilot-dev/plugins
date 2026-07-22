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
