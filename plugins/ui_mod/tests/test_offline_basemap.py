"""Tests for offline_basemap tile-path math and coverage/parse logic."""
import importlib.util
import os
import sys

import pytest

UI_MOD_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


@pytest.fixture
def obm():
  # config lives at the plugins root and is importable via PYTHONPATH=plugins;
  # tile_dir is always passed explicitly, so MEDIA_DIR's value is irrelevant.
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
    expected = {
      os.path.join('30', '116', '31.250000_117.000000_31.500000_117.250000'),
      os.path.join('30', '116', '31.250000_117.250000_31.500000_117.500000'),
      os.path.join('30', '116', '31.500000_117.000000_31.750000_117.250000'),
      os.path.join('30', '116', '31.500000_117.250000_31.750000_117.500000'),
    }
    assert set(rels) == expected


class TestCoverage:
  def test_coverage_complete_true(self, obm, tmp_path):
    for rel in obm._tiles_covering_bbox(31.6, 117.3, 31.6, 117.3):
      p = tmp_path / rel
      p.parent.mkdir(parents=True, exist_ok=True)
      p.write_bytes(b'x')
    assert obm.coverage_complete(31.6, 117.3, 31.6, 117.3, tile_dir=str(tmp_path)) is True

  def test_coverage_incomplete(self, obm, tmp_path):
    assert obm.coverage_complete(31.6, 117.3, 31.6, 117.3, tile_dir=str(tmp_path)) is False


class TestLoadPolylines:
  def test_no_capnp_returns_empty(self, obm, monkeypatch):
    monkeypatch.setattr(obm, 'HAVE_CAPNP', False)
    assert obm.load_polylines(31.6, 117.3, 31.6, 117.3, tile_dir='/nonexistent') == []

  def test_parses_one_way(self, obm, tmp_path):
    capnp = pytest.importorskip('capnp')
    schema = capnp.load(obm.SCHEMA_PATH)
    msg = schema.Offline.new_message()
    msg.minLat, msg.minLon, msg.maxLat, msg.maxLon = 31.5, 117.25, 31.75, 117.5
    ways = msg.init('ways', 1)
    nodes = ways[0].init('nodes', 2)
    nodes[0].latitude, nodes[0].longitude = 31.60, 117.30
    nodes[1].latitude, nodes[1].longitude = 31.61, 117.31
    rel = obm._tile_relpath(31.5, 117.25)
    path = tmp_path / rel
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(msg.to_bytes_packed())

    polys = obm.load_polylines(31.6, 117.3, 31.6, 117.3, tile_dir=str(tmp_path))
    assert len(polys) == 1
    assert len(polys[0]) == 2
    assert polys[0][0] == pytest.approx((31.60, 117.30))
    assert polys[0][1] == pytest.approx((31.61, 117.31))

  def test_skips_short_and_bad_tiles(self, obm, tmp_path):
    pytest.importorskip('capnp')
    # A tile of garbage bytes must be skipped, not raise.
    rel = obm._tile_relpath(31.5, 117.25)
    path = tmp_path / rel
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(b'not a capnp message')
    assert obm.load_polylines(31.6, 117.3, 31.6, 117.3, tile_dir=str(tmp_path)) == []
