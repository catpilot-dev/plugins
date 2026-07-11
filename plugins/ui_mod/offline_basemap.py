"""Offline OSM vector-tile reader for the route map basemap.

Self-contained within ui_mod: vendors the tile-path math and the
osm_reader.capnp schema so the route map never depends on the speedlimitd
plugin being installed. Tiles are packed Cap'n Proto `Offline` structs
(0.25 deg geographic chunks) under MEDIA_DIR/0/osm/offline.
"""
import math
import os

from config import MEDIA_DIR

OFFLINE_DIR = os.path.join(MEDIA_DIR, "0", "osm", "offline")
SCHEMA_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "osm_reader.capnp")
TILE_SIZE = 0.25  # degrees per tile

try:
  import capnp
  _SCHEMA = capnp.load(SCHEMA_PATH)
  HAVE_CAPNP = True
except (ImportError, OSError):
  _SCHEMA = None
  HAVE_CAPNP = False


def _tile_relpath(min_lat, min_lon):
  """Relative path (lat_dir/lon_dir/fname) for the tile whose min corner is
  (min_lat, min_lon). Matches speedlimitd's on-device tile naming."""
  lat_dir = str(int(math.floor(min_lat / 2) * 2))
  lon_dir = str(int(math.floor(min_lon / 2) * 2))
  fname = f"{min_lat:.6f}_{min_lon:.6f}_{min_lat + TILE_SIZE:.6f}_{min_lon + TILE_SIZE:.6f}"
  return os.path.join(lat_dir, lon_dir, fname)


def _tiles_covering_bbox(min_lat, min_lng, max_lat, max_lng):
  """Relative paths of every 0.25 deg tile spanning the bbox (inclusive)."""
  lat_i0 = int(math.floor(min_lat / TILE_SIZE))
  lat_i1 = int(math.floor(max_lat / TILE_SIZE))
  lon_i0 = int(math.floor(min_lng / TILE_SIZE))
  lon_i1 = int(math.floor(max_lng / TILE_SIZE))
  paths = []
  for lat_i in range(lat_i0, lat_i1 + 1):
    for lon_i in range(lon_i0, lon_i1 + 1):
      paths.append(_tile_relpath(lat_i * TILE_SIZE, lon_i * TILE_SIZE))
  return paths


def coverage_complete(min_lat, min_lng, max_lat, max_lng, tile_dir=OFFLINE_DIR):
  """True only if every tile spanning the bbox exists on disk."""
  rels = _tiles_covering_bbox(min_lat, min_lng, max_lat, max_lng)
  return bool(rels) and all(os.path.exists(os.path.join(tile_dir, r)) for r in rels)


def _way_tier(way):
  """Road-importance tier, 0 = most important. `highwayType` is empty on the
  pfeifer offline tiles, so importance is derived from the fields that ARE
  populated: a road reference (ref), mapped lane count, and speed limit."""
  if way.ref:
    return 0                       # numbered road: expressway / 国道 / 省道 / arterial
  if way.lanes >= 2:
    return 1                       # mapped multi-lane road
  if way.maxSpeed > 0 or way.lanes == 1:
    return 2                       # has a speed limit or a single mapped lane
  return 3                         # residential / service / unclassified


def _max_tier_for_zoom(zoom):
  """Highest tier drawn at a given fit-zoom. Zoomed out (whole route spans a
  large area) shows only major roads; zoomed in adds progressively smaller
  roads. Keeps the per-frame segment count bounded in dense urban tiles."""
  if zoom <= 12:
    return 0
  if zoom <= 14:
    return 1
  if zoom <= 15:
    return 2
  return 3


def _bbox_intersects(way, view):
  """True if the way's own bbox overlaps view=(min_lat, min_lng, max_lat,
  max_lng). Uses the Way's stored corners, so no node iteration is needed."""
  return not (way.maxLat < view[0] or way.minLat > view[2] or
              way.maxLon < view[1] or way.minLon > view[3])


def load_polylines(min_lat, min_lng, max_lat, max_lng, zoom=None, view=None, tile_dir=OFFLINE_DIR):
  """Parse every covering tile into a list of road polylines.

  Each polyline is a list of (lat, lng) tuples (>= 2 points), matching
  RouteMapRenderer._to_screen's point convention. Returns [] if capnp is
  unavailable. Tiles that fail to parse are skipped, not fatal.

  To keep the per-frame draw count bounded, roads are filtered:
  - `zoom` (fit-zoom): drops roads whose importance tier exceeds what that
    zoom shows (see _max_tier_for_zoom). None disables importance filtering.
  - `view` (min_lat, min_lng, max_lat, max_lng): drops roads whose bbox lies
    entirely outside the visible window. None disables culling.
  """
  if not HAVE_CAPNP:
    return []
  max_tier = _max_tier_for_zoom(zoom) if zoom is not None else None
  polylines = []
  for rel in _tiles_covering_bbox(min_lat, min_lng, max_lat, max_lng):
    path = os.path.join(tile_dir, rel)
    try:
      with open(path, "rb") as f:
        data = f.read()
      offline = _SCHEMA.Offline.from_bytes_packed(
        data, traversal_limit_in_words=len(data) * 8,
      )
      for way in offline.ways:
        if max_tier is not None and _way_tier(way) > max_tier:
          continue
        if view is not None and not _bbox_intersects(way, view):
          continue
        pts = [(n.latitude, n.longitude) for n in way.nodes]
        if len(pts) >= 2:
          polylines.append(pts)
    except Exception:
      continue
  return polylines
