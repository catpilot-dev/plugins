"""Tests for offline_basemap tile-path math and coverage/parse logic."""
import importlib.util
import os
import sys
import types

import pytest

UI_MOD_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


@pytest.fixture
def obm():
  # config lives at the plugins root; a stub keeps the test hermetic.
  # tile_dir is always passed explicitly, so MEDIA_DIR's value is irrelevant.
  sys.modules.setdefault('config', types.SimpleNamespace(MEDIA_DIR='/data/media'))
  if UI_MOD_DIR not in sys.path:
    sys.path.insert(0, UI_MOD_DIR)
  spec = importlib.util.spec_from_file_location(
    'offline_basemap', os.path.join(UI_MOD_DIR, 'offline_basemap.py'))
  mod = importlib.util.module_from_spec(spec)
  sys.modules['offline_basemap'] = mod
  spec.loader.exec_module(mod)
  return mod


class TestTilePaths:
  def test_tile_relpath_matches_device(self, obm):
    rel = obm._tile_relpath(31.5, 117.25)
    assert rel == os.path.join('30', '116', '31.500000_117.250000_31.750000_117.500000')

  def test_tiles_covering_single(self, obm):
    rels = obm._tiles_covering_bbox(31.6, 117.30, 31.6, 117.30)
    assert rels == [os.path.join('30', '116', '31.500000_117.250000_31.750000_117.500000')]

  def test_tiles_covering_span_2x2(self, obm):
    # lat 31.4..31.6 -> tiles 31.25 & 31.5; lon 117.2..117.4 -> tiles 117.0 & 117.25
    rels = obm._tiles_covering_bbox(31.4, 117.2, 31.6, 117.4)
    assert len(rels) == 4


class TestCoverage:
  def test_coverage_complete_true(self, obm, tmp_path):
    for rel in obm._tiles_covering_bbox(31.6, 117.3, 31.6, 117.3):
      p = tmp_path / rel
      p.parent.mkdir(parents=True, exist_ok=True)
      p.write_bytes(b'x')
    assert obm.coverage_complete(31.6, 117.3, 31.6, 117.3, tile_dir=str(tmp_path)) is True

  def test_coverage_incomplete(self, obm, tmp_path):
    assert obm.coverage_complete(31.6, 117.3, 31.6, 117.3, tile_dir=str(tmp_path)) is False
