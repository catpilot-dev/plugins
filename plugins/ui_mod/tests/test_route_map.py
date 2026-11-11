"""Tests for route_map Web Mercator math and tile logic."""
import importlib.util
import os
import pytest


@pytest.fixture
def route_map():
  """Import route_map module without raylib dependency."""
  mod_path = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'route_map.py')

  # Stub pyray and openpilot so module loads without GPU/openpilot
  import sys
  from unittest.mock import MagicMock
  pyray_mock = MagicMock()
  pyray_mock.Color = lambda r, g, b, a: (r, g, b, a)
  sys.modules.setdefault('pyray', pyray_mock)
  plugins_dir = os.path.dirname(os.path.dirname(mod_path))
  if plugins_dir not in sys.path:  # plugins/config.py, on the path at runtime
    sys.path.insert(0, plugins_dir)
  for mod_name in [
    'openpilot', 'openpilot.system', 'openpilot.system.ui',
    'openpilot.system.ui.lib', 'openpilot.system.ui.lib.application',
    'openpilot.system.ui.lib.text_measure',
  ]:
    sys.modules.setdefault(mod_name, MagicMock())

  spec = importlib.util.spec_from_file_location('route_map', mod_path)
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  return mod


# ============================================================
# _lat_lng_to_tile_xy
# ============================================================

class TestLatLngToTileXY:
  def test_origin(self, route_map):
    """(0, 0) at zoom 0 should be at tile center."""
    x, y = route_map._lat_lng_to_tile_xy(0, 0, 0)
    assert x == pytest.approx(0.5)
    assert y == pytest.approx(0.5)

  def test_known_location_zoom_10(self, route_map):
    """Berlin (52.52, 13.405) at zoom 10 — known tile coords."""
    x, y = route_map._lat_lng_to_tile_xy(52.52, 13.405, 10)
    assert int(x) == 550
    assert int(y) == 335

  def test_zoom_increases_coords(self, route_map):
    """Higher zoom should give proportionally larger coordinates."""
    x1, y1 = route_map._lat_lng_to_tile_xy(40.0, -74.0, 10)
    x2, y2 = route_map._lat_lng_to_tile_xy(40.0, -74.0, 11)
    assert x2 == pytest.approx(x1 * 2, abs=0.01)
    assert y2 == pytest.approx(y1 * 2, abs=0.01)


# ============================================================
# _tiles_for_rect
# ============================================================

class TestTilesForRect:
  def test_center_point_inside_range(self, route_map):
    """Center tile coords should be within the returned tile range."""
    tx0, tx1, ty0, ty1, cx, cy = route_map._tiles_for_rect(52.52, 13.405, 14, 750, 300)
    assert tx0 <= int(cx) <= tx1
    assert ty0 <= int(cy) <= ty1

  def test_covers_rect(self, route_map):
    """Tile range should cover at least the requested pixel dimensions."""
    w, h = 750, 300
    tx0, tx1, ty0, ty1, _, _ = route_map._tiles_for_rect(40.0, -74.0, 14, w, h)
    tile_px = route_map.TILE_PX
    assert (tx1 - tx0 + 1) * tile_px >= w
    assert (ty1 - ty0 + 1) * tile_px >= h

  def test_reasonable_tile_count(self, route_map):
    """Should not produce an excessive number of tiles."""
    tx0, tx1, ty0, ty1, _, _ = route_map._tiles_for_rect(39.9, 116.4, 14, 750, 300)
    assert (tx1 - tx0 + 1) <= 4
    assert (ty1 - ty0 + 1) <= 3

  def test_different_zoom_levels(self, route_map):
    """Higher zoom should produce same tile count for same rect size."""
    _, tx1a, _, ty1a, _, _ = route_map._tiles_for_rect(52.52, 13.405, 12, 750, 300)
    _, tx1b, _, ty1b, _, _ = route_map._tiles_for_rect(52.52, 13.405, 14, 750, 300)
    # Tile count depends on rect size / TILE_PX, not zoom — should be similar
    tx0a, _, ty0a, _, _, _ = route_map._tiles_for_rect(52.52, 13.405, 12, 750, 300)
    tx0b, _, ty0b, _, _, _ = route_map._tiles_for_rect(52.52, 13.405, 14, 750, 300)
    count_a = (tx1a - tx0a + 1) * (ty1a - ty0a + 1)
    count_b = (tx1b - tx0b + 1) * (ty1b - ty0b + 1)
    assert abs(count_a - count_b) <= 2


# ============================================================
# RouteMapRenderer tile paths
# ============================================================

class TestTilePaths:
  def test_tile_path_structure(self, route_map):
    r = route_map.RouteMapRenderer()
    path = r._tile_path(12, 550, 335)
    assert path.endswith('/osm-dark/12/550/335.png')

  def test_tile_path_cache_dir(self, route_map):
    r = route_map.RouteMapRenderer()
    path = r._tile_path(14, 100, 200)
    assert route_map.TILE_CACHE_DIR in path


# ============================================================
# Tile source: OpenStreetMap, darkened like COD's map
# ============================================================

class TestTileSource:
  def test_osm_url_no_carto(self, route_map):
    # CARTO basemaps now answer every keyless request with an "API KEY REQUIRED" tile
    assert route_map.TILE_URL.format(z=12, x=3430, y=1673) == 'https://tile.openstreetmap.org/12/3430/1673.png'
    assert 'carto' not in route_map.TILE_URL

  def test_attribution_is_drawable_ascii(self, route_map):
    # the UI font holds printable ASCII, not "©"
    assert 'OpenStreetMap' in route_map.ATTRIBUTION
    assert all(32 <= ord(c) < 127 for c in route_map.ATTRIBUTION)


class TestDarken:
  def _px(self, route_map, rgb):
    import numpy as np
    return tuple(int(v) for v in route_map.darken(np.array([[rgb]], dtype=np.uint8))[0, 0])

  def test_white_becomes_black(self, route_map):
    assert self._px(route_map, (255, 255, 255)) == (0, 0, 0)

  def test_black_becomes_light(self, route_map):
    # invert → 1, brightness 0.95, contrast 1.1 → 0.995
    assert self._px(route_map, (0, 0, 0)) == (254, 254, 254)

  def test_osm_land_becomes_dark_and_muted(self, route_map):
    r, g, b = self._px(route_map, (0xf2, 0xef, 0xe9))  # OSM land background
    assert max(r, g, b) < 40
    assert max(r, g, b) - min(r, g, b) < 10

  def test_png_roundtrip(self, route_map):
    import io
    from PIL import Image
    buf = io.BytesIO()
    Image.new('P', (256, 256), 0).save(buf, 'PNG')  # OSM tiles are palette PNGs
    out = Image.open(io.BytesIO(route_map._darken_png(buf.getvalue())))
    assert out.size == (256, 256)
    assert out.mode == 'RGB'


class TestTileCache:
  def _renderer(self, route_map, tmp_path, monkeypatch, keys):
    monkeypatch.setattr(route_map, 'TILE_CACHE_DIR', str(tmp_path))
    r = route_map.RouteMapRenderer()
    r._tile_keys = keys
    return r

  def _fake_urlopen(self, route_map, monkeypatch, calls):
    import io
    from PIL import Image
    buf = io.BytesIO()
    Image.new('RGB', (256, 256), (255, 255, 255)).save(buf, 'PNG')

    class Resp(io.BytesIO):
      def __enter__(self):
        return self

      def __exit__(self, *a):
        return False

    def fake(req, timeout=None):
      calls.append(req.full_url)
      return Resp(buf.getvalue())
    monkeypatch.setattr(route_map.urllib.request, 'urlopen', fake)

  def test_downloads_missing_tile_darkened(self, route_map, tmp_path, monkeypatch):
    from PIL import Image
    calls = []
    self._fake_urlopen(route_map, monkeypatch, calls)
    r = self._renderer(route_map, tmp_path, monkeypatch, [(12, 1, 2)])
    r._download_tiles()
    assert calls == ['https://tile.openstreetmap.org/12/1/2.png']
    assert Image.open(r._tile_path(12, 1, 2)).getpixel((0, 0)) == (0, 0, 0)

  def test_fresh_cached_tile_not_refetched(self, route_map, tmp_path, monkeypatch):
    calls = []
    self._fake_urlopen(route_map, monkeypatch, calls)
    r = self._renderer(route_map, tmp_path, monkeypatch, [(12, 1, 2)])
    os.makedirs(os.path.dirname(r._tile_path(12, 1, 2)))
    with open(r._tile_path(12, 1, 2), 'wb') as f:
      f.write(b'cached')
    r._download_tiles()
    assert calls == []

  def test_old_tiles_and_retired_sources_pruned(self, route_map, tmp_path, monkeypatch):
    import time
    calls = []
    self._fake_urlopen(route_map, monkeypatch, calls)
    r = self._renderer(route_map, tmp_path, monkeypatch, [])
    stale = r._tile_path(12, 1, 2)
    os.makedirs(os.path.dirname(stale))
    open(stale, 'wb').close()
    old = time.time() - (route_map.TILE_MAX_AGE_DAYS + 1) * 86400
    os.utime(stale, (old, old))
    os.makedirs(tmp_path / 'cartodb' / '12' / '1')  # left by the retired CARTO source
    r._download_tiles()
    assert not os.path.exists(stale)
    assert not (tmp_path / 'cartodb').exists()

  def _flaky_urlopen(self, route_map, monkeypatch, fails):
    """urlopen that raises for the first `fails` calls (no network yet at boot), then serves a white tile."""
    import io
    from PIL import Image
    buf = io.BytesIO()
    Image.new('RGB', (256, 256), (255, 255, 255)).save(buf, 'PNG')
    calls = []

    class Resp(io.BytesIO):
      def __enter__(self):
        return self

      def __exit__(self, *a):
        return False

    def fake(req, timeout=None):
      calls.append(req.full_url)
      if len(calls) <= fails:
        raise OSError('network unreachable')
      return Resp(buf.getvalue())
    monkeypatch.setattr(route_map.urllib.request, 'urlopen', fake)
    monkeypatch.setattr(route_map.time, 'sleep', lambda s: None)
    return calls

  def test_failed_download_retried(self, route_map, tmp_path, monkeypatch):
    # at boot the network is down (and the clock unsynced): the tiles must still arrive later
    calls = self._flaky_urlopen(route_map, monkeypatch, fails=3)
    r = self._renderer(route_map, tmp_path, monkeypatch, [(12, 1, 2), (12, 1, 3)])
    r._download_tiles()
    assert os.path.exists(r._tile_path(12, 1, 2))
    assert os.path.exists(r._tile_path(12, 1, 3))
    assert len(calls) == 5

  def test_retry_stops_when_trace_replaced(self, route_map, tmp_path, monkeypatch):
    calls = self._flaky_urlopen(route_map, monkeypatch, fails=10**6)
    r = self._renderer(route_map, tmp_path, monkeypatch, [(12, 1, 2)])
    monkeypatch.setattr(route_map.time, 'sleep', lambda s: r.cleanup())  # new trace / map closed while waiting
    r._download_tiles()
    assert len(calls) == 1

  def test_cached_tile_not_touched_back_in_time(self, route_map, tmp_path, monkeypatch):
    # at boot the clock may sit months in the past: touching a tile then would age it into the next prune
    import time
    calls = []
    self._fake_urlopen(route_map, monkeypatch, calls)
    r = self._renderer(route_map, tmp_path, monkeypatch, [(12, 1, 2)])
    path = r._tile_path(12, 1, 2)
    os.makedirs(os.path.dirname(path))
    open(path, 'wb').close()
    later = time.time() + 200 * 86400
    os.utime(path, (later, later))
    r._download_tiles()
    assert os.path.getmtime(path) == later
    assert calls == []
