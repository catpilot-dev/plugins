"""Tests for the offline_hw tile generator — binning, filtering, capnp output.

generate_hw_tiles.py only imports argparse/math/os/re/sys at module level —
capnp is imported lazily inside write_tiles() — so the pure-function tests
(highway filter, binning, maxspeed/maxspeed:lanes parsing) run with no capnp
present. Only TestWriteTiles actually builds/reads a capnp tile, so the
capnp gate is scoped to that class alone via an autouse fixture, instead of
skipping the whole module.
"""
import os
import sys

import pytest

PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
TOOLS_DIR = os.path.join(PKG_DIR, 'tools')
for p in (TOOLS_DIR, PKG_DIR, os.path.abspath(os.path.join(PKG_DIR, '..'))):
  if p not in sys.path:
    sys.path.insert(0, p)


@pytest.fixture
def gen():
  import generate_hw_tiles as mod
  return mod


def _way(name='', highway='tertiary', nodes=None, **kw):
  return {'name': name, 'ref': kw.get('ref', ''), 'highway': highway,
          'maxspeed': kw.get('maxspeed', 0.0), 'lanes': kw.get('lanes', 0),
          'oneway': kw.get('oneway', False),
          'nodes': nodes or [(31.31, 121.53), (31.32, 121.54)]}


class TestHighwayFilter:
  def test_drivable_types_kept(self, gen):
    for hw in ('motorway', 'trunk', 'primary', 'secondary', 'tertiary',
               'residential', 'unclassified', 'living_street', 'service',
               'motorway_link', 'trunk_link', 'primary_link'):
      assert gen.is_drivable_highway(hw), hw

  def test_non_drivable_types_dropped(self, gen):
    for hw in ('footway', 'cycleway', 'path', 'steps', 'pedestrian',
               'track', 'construction', 'proposed', 'corridor', ''):
      assert not gen.is_drivable_highway(hw), hw


class TestBinning:
  def test_way_lands_in_its_tile(self, gen):
    ways = [_way(name='白城路', nodes=[(31.3136, 121.5394), (31.3202, 121.5394)])]
    tiles = gen.bin_ways_into_tiles(ways)
    assert (31.25, 121.5) in tiles
    assert tiles[(31.25, 121.5)][0]['name'] == '白城路'

  def test_spanning_way_lands_in_all_touched_tiles(self, gen):
    # Way crossing the 31.25 lat boundary appears in both tiles
    ways = [_way(nodes=[(31.24, 121.53), (31.26, 121.53)])]
    tiles = gen.bin_ways_into_tiles(ways)
    assert (31.0, 121.5) in tiles
    assert (31.25, 121.5) in tiles

  def test_bbox_filter_excludes_outside_ways(self, gen):
    ways = [_way(nodes=[(31.31, 121.53), (31.32, 121.54)]),
            _way(name='far', nodes=[(39.9, 116.4), (39.91, 116.41)])]
    tiles = gen.bin_ways_into_tiles(ways, bbox=(31.0, 121.0, 32.0, 122.0))
    all_names = [w['name'] for tws in tiles.values() for w in tws]
    assert 'far' not in all_names


class TestWriteTiles:
  @pytest.fixture(autouse=True)
  def _require_capnp(self):
    pytest.importorskip('capnp')

  def test_written_tile_readable_by_osm_query(self, gen, tmp_path, monkeypatch):
    # Full round trip: generate → write → query via OsmTileReader
    monkeypatch.setenv('MEDIA_DIR', str(tmp_path))
    for mod in ('config', 'osm_query'):
      sys.modules.pop(mod, None)
    import importlib
    import osm_query
    importlib.reload(osm_query)

    ways = [_way(name='白城路', highway='tertiary',
                 nodes=[(31.3136, 121.5394), (31.3202, 121.5394)])]
    tiles = gen.bin_ways_into_tiles(ways)
    out_dir = os.path.join(str(tmp_path), '0/osm/offline_hw')
    gen.write_tiles(tiles, out_dir)

    reader = osm_query.OsmTileReader()
    import time
    result = None
    for _ in range(100):
      result = reader.query(31.3137, 121.5395)
      if result is not None:
        break
      time.sleep(0.02)
    assert result is not None
    assert result['roadName'] == '白城路'
    assert result['highwayType'] == 'tertiary'

  def test_negative_and_oversize_lanes_clamped(self, gen, tmp_path):
    # OSM has junk like lanes=-1; UInt8 write must not blow up.
    ways = [_way(lanes=-1, nodes=[(31.31, 121.53), (31.32, 121.54)]),
            _way(lanes=999, nodes=[(31.31, 121.53), (31.32, 121.54)])]
    tiles = gen.bin_ways_into_tiles(ways)
    assert gen.write_tiles(tiles, str(tmp_path / 'offline_hw')) == 1

  def test_tile_file_layout_matches_reader(self, gen, tmp_path):
    ways = [_way(nodes=[(31.3136, 121.5394), (31.3202, 121.5394)])]
    tiles = gen.bin_ways_into_tiles(ways)
    out_dir = str(tmp_path / 'offline_hw')
    gen.write_tiles(tiles, out_dir)
    expected = os.path.join(
      out_dir, '30', '120',
      '31.250000_121.500000_31.500000_121.750000')
    assert os.path.isfile(expected)


class TestMaxspeedParse:
  def test_plain_kph(self, gen):
    assert gen.parse_maxspeed('40') == pytest.approx(40 / 3.6)

  def test_kmh_suffix(self, gen):
    assert gen.parse_maxspeed('60 km/h') == pytest.approx(60 / 3.6)

  def test_unparseable(self, gen):
    assert gen.parse_maxspeed('walk') == 0.0
    assert gen.parse_maxspeed('') == 0.0
    assert gen.parse_maxspeed(None) == 0.0


class TestMaxspeedLanesParse:
  def test_multi_lane_collapses_to_minimum(self, gen):
    # 100|80|80|80 -> the slowest posted lane (80), never the fastest: a
    # single commanded speed must not over-command a car sitting in a
    # slower-limited lane.
    assert gen.parse_maxspeed_lanes('100|80|80|80') == pytest.approx(80 / 3.6)

  def test_unspecified_middle_lane_skipped(self, gen):
    assert gen.parse_maxspeed_lanes('100||80') == pytest.approx(80 / 3.6)

  def test_single_lane(self, gen):
    assert gen.parse_maxspeed_lanes('80') == pytest.approx(80 / 3.6)

  def test_unparseable(self, gen):
    assert gen.parse_maxspeed_lanes('') == 0.0
    assert gen.parse_maxspeed_lanes(None) == 0.0
    assert gen.parse_maxspeed_lanes('|') == 0.0

  def test_mph_lane_value_converts(self, gen):
    assert gen.parse_maxspeed_lanes('65 mph|55 mph') == pytest.approx(55 * 1.609344 / 3.6)


class TestResolveWayMaxspeed:
  def test_scalar_only(self, gen):
    assert gen.resolve_way_maxspeed({'maxspeed': '100'}) == pytest.approx(100 / 3.6)

  def test_lanes_fallback_when_scalar_absent(self, gen):
    assert gen.resolve_way_maxspeed({'maxspeed:lanes': '100|80|80|80'}) == pytest.approx(80 / 3.6)

  def test_scalar_wins_over_lanes(self, gen):
    tags = {'maxspeed': '100', 'maxspeed:lanes': '60|60|60|60'}
    assert gen.resolve_way_maxspeed(tags) == pytest.approx(100 / 3.6)

  def test_lanes_forward_used_when_lanes_absent(self, gen):
    assert gen.resolve_way_maxspeed(
        {'maxspeed:lanes:forward': '100|80|80|80'}) == pytest.approx(80 / 3.6)

  def test_nothing_tagged_returns_zero(self, gen):
    assert gen.resolve_way_maxspeed({}) == 0.0
