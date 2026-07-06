"""Tests for osm_query — tile reading, way matching, highwayType support."""
import os
import sys
import time
import importlib

import pytest

capnp = pytest.importorskip('capnp')

PKG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
SCHEMA_PATH = os.path.join(PKG_DIR, 'osm_reader.capnp')


@pytest.fixture
def osm_query(monkeypatch, tmp_path):
  """Import osm_query with MEDIA_DIR pointed at tmp_path."""
  monkeypatch.setenv('MEDIA_DIR', str(tmp_path))
  for mod in ('config', 'osm_query'):
    sys.modules.pop(mod, None)
  for p in (PKG_DIR, os.path.join(PKG_DIR, '..')):
    if p not in sys.path:
      sys.path.insert(0, p)
  import osm_query as mod
  importlib.reload(mod)
  return mod


def _write_tile(schema, path, ways):
  """Write a packed Offline tile file with the given way dicts."""
  offline = schema.Offline.new_message()
  way_list = offline.init('ways', len(ways))
  for i, w in enumerate(ways):
    way = way_list[i]
    way.name = w.get('name', '')
    way.ref = w.get('ref', '')
    way.maxSpeed = w.get('maxSpeed', 0.0)
    nodes = w['nodes']
    lats = [n[0] for n in nodes]
    lons = [n[1] for n in nodes]
    way.minLat, way.maxLat = min(lats), max(lats)
    way.minLon, way.maxLon = min(lons), max(lons)
    node_list = way.init('nodes', len(nodes))
    for j, (lat, lon) in enumerate(nodes):
      node_list[j].latitude = lat
      node_list[j].longitude = lon
    if 'highwayType' in w:
      way.highwayType = w['highwayType']
  os.makedirs(os.path.dirname(path), exist_ok=True)
  with open(path, 'wb') as f:
    f.write(offline.to_bytes_packed())


class TestHighwayType:
  LAT, LON = 31.3137, 121.5395

  def _make_reader_with_tile(self, osm_query, ways):
    reader = osm_query.OsmTileReader()
    path = osm_query._tile_path(self.LAT, self.LON)
    _write_tile(reader.schema, path, ways)
    return reader

  def _query_when_loaded(self, reader, lat, lon):
    # First query kicks off the background tile load; poll until loaded
    for _ in range(100):
      result = reader.query(lat, lon)
      if result is not None:
        return result
      time.sleep(0.02)
    return None

  def test_query_returns_highway_type(self, osm_query):
    reader = self._make_reader_with_tile(osm_query, [{
      'name': '白城路',
      'highwayType': 'tertiary',
      'nodes': [(31.3136, 121.5394), (31.3202, 121.5394)],
    }])
    result = self._query_when_loaded(reader, self.LAT, self.LON)
    assert result is not None
    assert result['roadName'] == '白城路'
    assert result['highwayType'] == 'tertiary'

  def test_old_tile_without_field_reads_empty(self, osm_query):
    # Simulates a pfeifer tile written before highwayType existed
    reader = self._make_reader_with_tile(osm_query, [{
      'name': '白城路',
      'nodes': [(31.3136, 121.5394), (31.3202, 121.5394)],
    }])
    result = self._query_when_loaded(reader, self.LAT, self.LON)
    assert result is not None
    assert result['highwayType'] == ''


class TestHwTileDirPreference:
  LAT, LON = 31.3137, 121.5395

  def test_hw_tile_preferred_over_pfeifer_tile(self, osm_query):
    reader = osm_query.OsmTileReader()
    ways_kwargs = {'nodes': [(31.3136, 121.5394), (31.3202, 121.5394)]}
    # pfeifer tile: no highwayType
    _write_tile(reader.schema, osm_query._tile_path(self.LAT, self.LON),
                [{'name': 'old', **ways_kwargs}])
    # hw tile: same tile coords in the offline_hw dir
    _write_tile(reader.schema, osm_query._hw_tile_path(self.LAT, self.LON),
                [{'name': 'new', 'highwayType': 'tertiary', **ways_kwargs}])
    for _ in range(100):
      result = reader.query(self.LAT, self.LON)
      if result is not None:
        break
      time.sleep(0.02)
    assert result is not None
    assert result['roadName'] == 'new'
    assert result['highwayType'] == 'tertiary'

  def test_falls_back_to_pfeifer_tile_when_no_hw_tile(self, osm_query):
    reader = osm_query.OsmTileReader()
    _write_tile(reader.schema, osm_query._tile_path(self.LAT, self.LON),
                [{'name': 'old', 'nodes': [(31.3136, 121.5394), (31.3202, 121.5394)]}])
    for _ in range(100):
      result = reader.query(self.LAT, self.LON)
      if result is not None:
        break
      time.sleep(0.02)
    assert result is not None
    assert result['roadName'] == 'old'
    assert result['highwayType'] == ''
