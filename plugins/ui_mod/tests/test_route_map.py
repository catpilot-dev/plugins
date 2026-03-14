"""Tests for route_map Web Mercator math and tile logic."""
import importlib.util
import math
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
    assert path.endswith('/cartodb/12/550/335.png')

  def test_tile_path_cache_dir(self, route_map):
    r = route_map.RouteMapRenderer()
    path = r._tile_path(14, 100, 200)
    assert route_map.TILE_CACHE_DIR in path
