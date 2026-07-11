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
