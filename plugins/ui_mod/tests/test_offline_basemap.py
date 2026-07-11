"""Tests for offline_basemap tile-path math and coverage/parse logic."""
import importlib.util
import os
import sys
from types import SimpleNamespace

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


class TestRoadImportance:
  def test_tier_ref_is_top(self, obm):
    assert obm._way_tier(SimpleNamespace(ref="G42", lanes=0, maxSpeed=0.0)) == 0

  def test_tier_multilane(self, obm):
    assert obm._way_tier(SimpleNamespace(ref="", lanes=3, maxSpeed=0.0)) == 1

  def test_tier_speed_or_single_lane(self, obm):
    assert obm._way_tier(SimpleNamespace(ref="", lanes=0, maxSpeed=13.9)) == 2
    assert obm._way_tier(SimpleNamespace(ref="", lanes=1, maxSpeed=0.0)) == 2

  def test_tier_residential(self, obm):
    assert obm._way_tier(SimpleNamespace(ref="", lanes=0, maxSpeed=0.0)) == 3

  def test_max_tier_for_zoom(self, obm):
    assert obm._max_tier_for_zoom(11) == 0
    assert obm._max_tier_for_zoom(12) == 0
    assert obm._max_tier_for_zoom(13) == 1
    assert obm._max_tier_for_zoom(14) == 1
    assert obm._max_tier_for_zoom(15) == 2
    assert obm._max_tier_for_zoom(16) == 3


class TestBboxIntersects:
  def test_inside(self, obm):
    way = SimpleNamespace(minLat=31.30, maxLat=31.31, minLon=121.60, maxLon=121.61)
    assert obm._bbox_intersects(way, (31.2, 121.5, 31.4, 121.7)) is True

  def test_fully_outside(self, obm):
    way = SimpleNamespace(minLat=32.00, maxLat=32.01, minLon=121.60, maxLon=121.61)
    assert obm._bbox_intersects(way, (31.2, 121.5, 31.4, 121.7)) is False


class TestLoadPolylinesFiltering:
  def _write_two_road_tile(self, obm, tmp_path):
    """Tile with one major (ref) road and one residential (no ref, 0 lanes),
    both geometrically at ~31.60,117.30."""
    capnp = pytest.importorskip('capnp')
    schema = capnp.load(obm.SCHEMA_PATH)
    msg = schema.Offline.new_message()
    msg.minLat, msg.minLon, msg.maxLat, msg.maxLon = 31.5, 117.25, 31.75, 117.5
    ways = msg.init('ways', 2)
    ways[0].ref = "G42"
    ways[0].minLat, ways[0].maxLat, ways[0].minLon, ways[0].maxLon = 31.60, 31.61, 117.30, 117.31
    n0 = ways[0].init('nodes', 2)
    n0[0].latitude, n0[0].longitude = 31.60, 117.30
    n0[1].latitude, n0[1].longitude = 31.61, 117.31
    ways[1].ref = ""
    ways[1].lanes = 0
    ways[1].minLat, ways[1].maxLat, ways[1].minLon, ways[1].maxLon = 31.60, 31.61, 117.30, 117.31
    n1 = ways[1].init('nodes', 2)
    n1[0].latitude, n1[0].longitude = 31.605, 117.305
    n1[1].latitude, n1[1].longitude = 31.606, 117.306
    rel = obm._tile_relpath(31.5, 117.25)
    path = tmp_path / rel
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(msg.to_bytes_packed())

  def test_zoom_drops_minor_roads(self, obm, tmp_path):
    self._write_two_road_tile(obm, tmp_path)
    # zoom 12 -> only tier 0 (the ref road) kept
    polys = obm.load_polylines(31.6, 117.3, 31.6, 117.3, zoom=12, tile_dir=str(tmp_path))
    assert len(polys) == 1
    assert polys[0][0] == pytest.approx((31.60, 117.30))

  def test_high_zoom_keeps_all(self, obm, tmp_path):
    self._write_two_road_tile(obm, tmp_path)
    polys = obm.load_polylines(31.6, 117.3, 31.6, 117.3, zoom=16, tile_dir=str(tmp_path))
    assert len(polys) == 2

  def test_no_zoom_keeps_all(self, obm, tmp_path):
    # Backward compatibility: without zoom/view, no filtering.
    self._write_two_road_tile(obm, tmp_path)
    polys = obm.load_polylines(31.6, 117.3, 31.6, 117.3, tile_dir=str(tmp_path))
    assert len(polys) == 2

  def test_view_culls_offscreen(self, obm, tmp_path):
    self._write_two_road_tile(obm, tmp_path)
    # View window far from the roads -> everything culled.
    culled = obm.load_polylines(
      31.6, 117.3, 31.6, 117.3, view=(31.70, 117.40, 31.72, 117.42), tile_dir=str(tmp_path))
    assert culled == []
    # View window over the roads -> both kept.
    kept = obm.load_polylines(
      31.6, 117.3, 31.6, 117.3, view=(31.59, 117.29, 31.62, 117.32), tile_dir=str(tmp_path))
    assert len(kept) == 2
