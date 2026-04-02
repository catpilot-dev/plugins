#!/usr/bin/env python3
"""Lightweight offline OSM tile reader — replaces mapd Go binary for real-time queries.

Reads pre-downloaded Cap'n Proto tile files, finds the nearest way to the
current GPS position, and returns road context (wayRef, roadName, speedLimit,
lanes, roadContext).

No msgq subscription, no Go binary, no crashes.
"""
import os
import math
import time
import logging

from config import MEDIA_DIR

log = logging.getLogger("osm_query")

TILE_DIR = os.path.join(MEDIA_DIR, "0/osm/offline")
SCHEMA_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "osm_reader.capnp")
TILE_SIZE = 0.25  # degrees per tile
MAX_WAY_DISTANCE = 50.0  # meters — ignore ways farther than this

# Approximate meters per degree at mid-latitudes
LAT_DEG_TO_M = 111_320.0
LON_DEG_TO_M = 111_320.0 * 0.85  # cos(31°) ≈ 0.85 for Shanghai


def _tile_path(lat: float, lon: float) -> str:
  """Get tile file path for a GPS coordinate."""
  min_lat = math.floor(lat / TILE_SIZE) * TILE_SIZE
  min_lon = math.floor(lon / TILE_SIZE) * TILE_SIZE
  max_lat = min_lat + TILE_SIZE
  max_lon = min_lon + TILE_SIZE

  lat_dir = str(int(math.floor(lat / 2) * 2))
  lon_dir = str(int(math.floor(lon / 2) * 2))

  fname = f"{min_lat:.6f}_{min_lon:.6f}_{max_lat:.6f}_{max_lon:.6f}"
  return os.path.join(TILE_DIR, lat_dir, lon_dir, fname)


def _point_to_segment_distance(px, py, ax, ay, bx, by):
  """Distance from point (px,py) to line segment (ax,ay)-(bx,by) in meters."""
  dx, dy = bx - ax, by - ay
  if dx == 0 and dy == 0:
    return math.sqrt(((px - ax) * LON_DEG_TO_M) ** 2 + ((py - ay) * LAT_DEG_TO_M) ** 2)
  t = max(0, min(1, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
  cx, cy = ax + t * dx, ay + t * dy
  return math.sqrt(((px - cx) * LON_DEG_TO_M) ** 2 + ((py - cy) * LAT_DEG_TO_M) ** 2)


class OsmTileReader:
  def __init__(self):
    try:
      import capnp
      self.schema = capnp.load(SCHEMA_PATH)
    except (ImportError, OSError):
      self.schema = None
      log.warning("capnp not available — OSM tile queries disabled")
    self._tile_cache = {}  # path -> (offline_msg, timestamp)
    self._cache_ttl = 300  # seconds

  def _load_tile(self, path: str):
    """Load and cache a tile file."""
    now = time.monotonic()
    if path in self._tile_cache:
      cached, ts = self._tile_cache[path]
      if now - ts < self._cache_ttl:
        return cached

    if not os.path.exists(path):
      return None

    try:
      with open(path, 'rb') as f:
        data = f.read()
      offline = self.schema.Offline.from_bytes_packed(data)
      self._tile_cache[path] = (offline, now)

      # Evict old tiles
      if len(self._tile_cache) > 10:
        oldest = min(self._tile_cache, key=lambda k: self._tile_cache[k][1])
        del self._tile_cache[oldest]

      return offline
    except Exception as e:
      log.warning("Failed to load tile %s: %s", path, e)
      return None

  def query(self, lat: float, lon: float) -> dict | None:
    """Find the nearest way to the given GPS position.

    Returns dict with wayRef, wayName, speedLimit, lanes, roadContext, distance,
    or None if no tile or no nearby way.
    """
    if self.schema is None:
      return None

    path = _tile_path(lat, lon)
    offline = self._load_tile(path)
    if offline is None:
      return None

    best_dist = MAX_WAY_DISTANCE
    best_way = None

    for way in offline.ways:
      # Quick bounding box check — skip ways whose bbox doesn't overlap
      if lat < way.minLat - 0.001 or lat > way.maxLat + 0.001:
        continue
      if lon < way.minLon - 0.001 or lon > way.maxLon + 0.001:
        continue

      # Skip ways with no geometry
      nodes = way.nodes
      if len(nodes) < 2:
        continue

      min_seg_dist = float('inf')
      for i in range(len(nodes) - 1):
        d = _point_to_segment_distance(
          lon, lat,
          nodes[i].longitude, nodes[i].latitude,
          nodes[i + 1].longitude, nodes[i + 1].latitude)
        if d < min_seg_dist:
          min_seg_dist = d

      if min_seg_dist < best_dist:
        best_dist = min_seg_dist
        best_way = way

    if best_way is None:
      return None

    ref = best_way.ref
    name = best_way.name
    speed = best_way.maxSpeed  # m/s
    if speed <= 0:
      speed = best_way.maxSpeedForward
    lanes = best_way.lanes

    # Road context: only G/S wayRef = freeway (expressway with controlled access).
    # Urban elevated roads (高架路) and fast roads (快速路) are NOT freeways —
    # they have intersections, on/off ramps at grade, and 80 km/h limits.
    is_freeway = bool(ref and (ref.startswith('G') or ref.startswith('S')))

    return {
      'wayRef': ref or '',
      'wayName': name or '',
      'speedLimit': speed,  # m/s
      'lanes': lanes,
      'roadContext': 0 if is_freeway else 1,  # 0=freeway, 1=city
      'roadName': name or '',
      'distance': best_dist,
    }
