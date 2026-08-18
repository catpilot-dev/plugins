"""Adapter — mapdOut road context for speedlimitd.

Pure and stateless: no I/O, no clock, no globals, no imports from speedlimitd.
That keeps it unit-testable with plain stubs and keeps speedlimitd.py (already
~1700 lines) from absorbing another responsibility.

PHASE 1 SCOPE: observation only. This module reports what mapd sees so a drive
can be compared against the offline tile reader that still drives control.
Phase 2 adds result_from_mapd(), which rebuilds the osm_query-shaped dict for
_ingest_osm_result — deliberately not written yet, because nothing calls it and
the Phase 1 drive may change its shape. See
docs/superpowers/specs/2026-08-18-mapd-integration-design.md.
"""

# MapdOut.highwayClass -> the OSM highway=* string speedlimitd already uses for
# last_osm_hwtype. Keys are capnp enumerant names (pycapnp returns the camelCase
# name when reading an enum field).
#
# Values are UNDERSCORE OSM form, not camelCase: they are tested against
# URBAN_ONLY_TYPES and used as speed-table keys in infer_speed_from_road_type
# (speedlimitd.py:556 and :585), both of which expect raw OSM values.
#
# 'unknown' -> '' because '' is what speedlimitd already treats as "no OSM
# classification available".
#
# Known gap: OSM highway=service has no HighwayClass member upstream, so service
# roads arrive as 'unknown' -> ''. The retired tile generator carried them
# verbatim, and 'service' is in URBAN_ONLY_TYPES, so such roads lose their
# urban demotion. Phase 1 telemetry measures how often this occurs.
_CLASS_TO_OSM = {
  'unknown': '',
  'motorway': 'motorway',
  'motorwayLink': 'motorway_link',
  'trunk': 'trunk',
  'trunkLink': 'trunk_link',
  'primary': 'primary',
  'primaryLink': 'primary_link',
  'secondary': 'secondary',
  'secondaryLink': 'secondary_link',
  'tertiary': 'tertiary',
  'tertiaryLink': 'tertiary_link',
  'unclassified': 'unclassified',
  'residential': 'residential',
  'livingStreet': 'living_street',
}

MS_TO_KPH = 3.6


def highway_class_name(value) -> str:
  """capnp HighwayClass enumerant name -> OSM highway=* string.

  pycapnp reads an enum field back as its camelCase enumerant name, so that is
  the only input form this needs to handle.

  Unrecognised values return '' rather than raising: a mapd release that adds an
  enum member must not crash the daemon, it must degrade to "no classification".
  """
  return _CLASS_TO_OSM.get(value, '')


def telemetry_from_mapd(mapd_out, valid: bool, our_way_ref: str) -> dict:
  """Phase 1 observation fields for pluginBusLog.

  Always returns the same keys whether or not mapd is up — a drive with a dead
  mapd must still produce an analysable rlog.

  mapdRefAgree is the headline number: it compares mapd's bearing-aware match
  against the tile reader's nearest-polyline match, and is the evidence for both
  the Phase 2 cutover and retiring the G/S margin-release rule.
  """
  if not valid or mapd_out is None:
    return {
      'mapdAlive': False,
      'mapdWayRef': '',
      'mapdWayId': 0,
      'mapdSpeedLimit': 0.0,
      'mapdHwClass': '',
      'mapdLanes': 0,
      'mapdSelType': '',
      'mapdTileLoaded': False,
      'mapdDistance': 0.0,
      'mapdRefAgree': False,
    }

  ref = mapd_out.wayRef or ''
  return {
    'mapdAlive': True,
    'mapdWayRef': ref,
    'mapdWayId': int(mapd_out.wayId or 0),
    'mapdSpeedLimit': round(float(mapd_out.speedLimit or 0.0) * MS_TO_KPH, 1),
    'mapdHwClass': highway_class_name(mapd_out.highwayClass),
    'mapdLanes': int(mapd_out.lanes or 0),
    'mapdSelType': str(mapd_out.waySelectionType),
    'mapdTileLoaded': bool(mapd_out.tileLoaded),
    'mapdDistance': round(float(mapd_out.distanceFromWayCenter or 0.0), 2),
    'mapdRefAgree': ref == (our_way_ref or ''),
  }
