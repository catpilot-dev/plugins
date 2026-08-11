#!/usr/bin/env python3
"""Generate offline_hw OSM tiles with highway classification.

Builds 0.25° tiles in the same capnp format and directory layout as pfeiferj's
mapd offline tiles (see osm_reader.capnp), plus the highwayType field that the
pfeifer tile server does not provide. speedlimitd prefers these tiles
(osm/offline_hw) over the downloaded pfeifer ones (osm/offline).

Usage (on a PC):
  # Get an extract, e.g. https://download.geofabrik.de/asia/china-latest.osm.pbf
  python generate_hw_tiles.py --pbf china-latest.osm.pbf \
      --bbox 30.6,120.8,32.0,122.2 --out ./offline_hw
  rsync -r ./offline_hw/ device:/data/media/0/osm/offline_hw/

Requires: pip install osmium capnp
"""
import argparse
import math
import os
import re
import sys

TILE_SIZE = 0.25

SCHEMA_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'osm_reader.capnp')

# highway=* values a car can drive on. Everything else (footway, cycleway,
# track, construction, ...) is dropped to keep tiles small.
DRIVABLE_HIGHWAYS = {
  'motorway', 'trunk', 'primary', 'secondary', 'tertiary',
  'unclassified', 'residential', 'living_street', 'service',
  'motorway_link', 'trunk_link', 'primary_link', 'secondary_link', 'tertiary_link',
}


def is_drivable_highway(highway: str) -> bool:
  return highway in DRIVABLE_HIGHWAYS


def parse_maxspeed(value) -> float:
  """Parse an OSM maxspeed tag to m/s. Returns 0.0 if unparseable.

  speedlimitd consumes this value as its base inference when the OSM Data
  Integration toggle is on.
  """
  if not value:
    return 0.0
  m = re.match(r'^\s*(\d+)', str(value))
  if not m:
    return 0.0
  kph = int(m.group(1))
  if 'mph' in str(value):
    kph = kph * 1.609344
  return kph / 3.6


def parse_maxspeed_lanes(value) -> float:
  """Collapse an OSM maxspeed:lanes (or maxspeed:lanes:forward) tag to m/s.

  These tags carry one entry per lane, pipe-separated and left-to-right
  across the carriageway, e.g. '100|80|80|80'; an entry may be empty when a
  lane's limit isn't individually specified, e.g. '100||80'. Each non-empty
  entry is parsed with parse_maxspeed (so unit handling/formats stay
  identical to the scalar tag) and the result collapses to the MAXIMUM
  parsed value, deliberately not the minimum: the rest of speedlimitd's
  OSM path already targets the mainline limit (the G/S expressway table
  returns 100-120 for a motorway), so collapsing to the slowest lane would
  make the OSM path read MORE conservative than the vision fallback it
  replaces — the displayed limit would visibly drop the moment OSM data
  appeared, which reads as a regression, not a safety feature. A
  '100|80|80|80' gantry posts a road limit of 100 with lower limits on the
  slower lanes; that headline mainline number is what the sign should show,
  and holding the car at 80 in the fast lane while traffic flows at 100 is
  itself a hazard, not a neutral conservative choice.

  Accepted tradeoff: the returned value can exceed the limit of the specific
  lane the car actually occupies (speedlimitd has no notion of which lane
  that is). This is bounded downstream by the driver confirm tap, the
  curvature/lateral-accel safety caps, and gas override — it is not left
  unchecked. Returns 0.0 if no entry is parseable.
  """
  if not value:
    return 0.0
  speeds = [s for s in (parse_maxspeed(part.strip()) for part in str(value).split('|')) if s]
  return max(speeds) if speeds else 0.0


def resolve_way_maxspeed(tags) -> float:
  """Resolve a way's posted speed limit from its OSM tags, in m/s.

  Prefers the scalar `maxspeed` tag. Falls back to the per-lane
  `maxspeed:lanes` tag when the scalar is absent or unparseable (0.0), and to
  `maxspeed:lanes:forward` when `maxspeed:lanes` itself is absent — one-way
  carriageways commonly carry only the forward variant. See
  parse_maxspeed_lanes for why the per-lane fallback collapses to the
  maximum (mainline) lane value. Returns 0.0 when nothing is parseable.
  """
  scalar = parse_maxspeed(tags.get('maxspeed'))
  if scalar:
    return scalar
  lanes_value = tags.get('maxspeed:lanes') or tags.get('maxspeed:lanes:forward')
  return parse_maxspeed_lanes(lanes_value)


def _tile_key(lat: float, lon: float) -> tuple[float, float]:
  return (math.floor(lat / TILE_SIZE) * TILE_SIZE,
          math.floor(lon / TILE_SIZE) * TILE_SIZE)


def bin_ways_into_tiles(ways: list[dict], bbox: tuple | None = None) -> dict:
  """Assign each way to every 0.25° tile its bounding box touches.

  ways: [{name, ref, highway, maxspeed (m/s), lanes, oneway, nodes: [(lat, lon)]}]
  bbox: optional (min_lat, min_lon, max_lat, max_lon) filter — ways entirely
        outside are dropped.
  Returns {(tile_min_lat, tile_min_lon): [way, ...]}.
  """
  tiles: dict[tuple[float, float], list[dict]] = {}
  for way in ways:
    nodes = way['nodes']
    if len(nodes) < 2:
      continue
    min_lat = min(n[0] for n in nodes)
    max_lat = max(n[0] for n in nodes)
    min_lon = min(n[1] for n in nodes)
    max_lon = max(n[1] for n in nodes)

    if bbox is not None:
      b_min_lat, b_min_lon, b_max_lat, b_max_lon = bbox
      if max_lat < b_min_lat or min_lat > b_max_lat or max_lon < b_min_lon or min_lon > b_max_lon:
        continue

    lat0, _ = _tile_key(min_lat, min_lon)
    lon0 = _tile_key(0, min_lon)[1]
    lat = lat0
    while lat <= max_lat:
      lon = lon0
      while lon <= max_lon:
        tiles.setdefault((round(lat, 6), round(lon, 6)), []).append(way)
        lon += TILE_SIZE
      lat += TILE_SIZE
  return tiles


def write_tiles(tiles: dict, out_dir: str) -> int:
  """Write binned ways as packed capnp Offline files in mapd's dir layout.

  Layout: {out_dir}/{even_lat}/{even_lon}/{min_lat}_{min_lon}_{max_lat}_{max_lon}
  Returns number of tile files written.
  """
  import capnp
  capnp.remove_import_hook()
  schema = capnp.load(os.path.abspath(SCHEMA_PATH))

  written = 0
  for (tile_lat, tile_lon), ways in tiles.items():
    offline = schema.Offline.new_message()
    offline.minLat = tile_lat
    offline.minLon = tile_lon
    offline.maxLat = tile_lat + TILE_SIZE
    offline.maxLon = tile_lon + TILE_SIZE
    way_list = offline.init('ways', len(ways))
    for i, w in enumerate(ways):
      way = way_list[i]
      way.name = w.get('name', '')
      way.ref = w.get('ref', '')
      way.maxSpeed = w.get('maxspeed', 0.0)
      way.lanes = max(0, min(w.get('lanes', 0), 255))  # OSM has junk like lanes=-1
      way.oneWay = bool(w.get('oneway', False))
      way.highwayType = w.get('highway', '')
      nodes = w['nodes']
      way.minLat = min(n[0] for n in nodes)
      way.maxLat = max(n[0] for n in nodes)
      way.minLon = min(n[1] for n in nodes)
      way.maxLon = max(n[1] for n in nodes)
      node_list = way.init('nodes', len(nodes))
      for j, (lat, lon) in enumerate(nodes):
        node_list[j].latitude = lat
        node_list[j].longitude = lon

    lat_dir = str(int(math.floor(tile_lat / 2) * 2))
    lon_dir = str(int(math.floor(tile_lon / 2) * 2))
    fname = f"{tile_lat:.6f}_{tile_lon:.6f}_{tile_lat + TILE_SIZE:.6f}_{tile_lon + TILE_SIZE:.6f}"
    path = os.path.join(out_dir, lat_dir, lon_dir, fname)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'wb') as f:
      f.write(offline.to_bytes_packed())
    written += 1
  return written


def extract_ways_from_pbf(pbf_path: str, bbox: tuple | None = None) -> list[dict]:
  """Read drivable highway ways (with node locations) from an OSM .pbf."""
  import osmium

  ways: list[dict] = []

  class Handler(osmium.SimpleHandler):
    def way(self, w):
      highway = w.tags.get('highway', '')
      if not is_drivable_highway(highway):
        return
      nodes = []
      for n in w.nodes:
        if not n.location.valid():
          continue
        nodes.append((n.location.lat, n.location.lon))
      if len(nodes) < 2:
        return
      if bbox is not None:
        b_min_lat, b_min_lon, b_max_lat, b_max_lon = bbox
        if (max(n[0] for n in nodes) < b_min_lat or min(n[0] for n in nodes) > b_max_lat or
            max(n[1] for n in nodes) < b_min_lon or min(n[1] for n in nodes) > b_max_lon):
          return
      try:
        lanes = int(w.tags.get('lanes', '0'))
      except ValueError:
        lanes = 0
      ways.append({
        'name': w.tags.get('name', ''),
        'ref': w.tags.get('ref', ''),
        'highway': highway,
        'maxspeed': resolve_way_maxspeed(w.tags),
        'lanes': lanes,
        'oneway': w.tags.get('oneway', '') in ('yes', '1', 'true'),
        'nodes': nodes,
      })

  Handler().apply_file(pbf_path, locations=True)
  return ways


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument('--pbf', required=True, help='OSM extract (.osm.pbf), e.g. from Geofabrik')
  ap.add_argument('--out', required=True, help='output tile dir (rsync to device osm/offline_hw)')
  ap.add_argument('--bbox', help='min_lat,min_lon,max_lat,max_lon filter (default: whole extract)')
  args = ap.parse_args()

  bbox = None
  if args.bbox:
    parts = [float(x) for x in args.bbox.split(',')]
    if len(parts) != 4:
      ap.error('--bbox needs min_lat,min_lon,max_lat,max_lon')
    bbox = tuple(parts)

  print(f'reading {args.pbf}...', file=sys.stderr)
  ways = extract_ways_from_pbf(args.pbf, bbox=bbox)
  print(f'{len(ways)} drivable ways', file=sys.stderr)

  tiles = bin_ways_into_tiles(ways, bbox=bbox)
  n = write_tiles(tiles, args.out)
  print(f'wrote {n} tiles to {args.out}', file=sys.stderr)


if __name__ == '__main__':
  main()
