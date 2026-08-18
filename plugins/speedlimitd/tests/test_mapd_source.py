"""Adapter tests — mapdOut becomes the road-context dict speedlimitd consumes."""
import os
import re
import sys
from dataclasses import dataclass

import pytest

_SLD_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SLD_DIR not in sys.path:
  sys.path.insert(0, _SLD_DIR)

import mapd_source  # noqa: E402


@dataclass
class FakeMapdOut:
  """Stands in for a capnp MapdOut reader. Field names match the schema."""
  wayRef: str = ''
  wayName: str = ''
  roadName: str = ''
  speedLimit: float = 0.0
  lanes: int = 0
  highwayClass: str = 'unknown'
  wayId: int = 0
  tileLoaded: bool = True
  distanceFromWayCenter: float = 0.0
  waySelectionType: str = 'current'
  roadContext: str = 'city'


class TestHighwayClassName:
  def test_maps_to_underscore_osm_strings(self):
    # These feed URBAN_ONLY_TYPES membership and speed-table lookups in
    # infer_speed_from_road_type, which expect raw OSM values.
    assert mapd_source.highway_class_name('motorwayLink') == 'motorway_link'
    assert mapd_source.highway_class_name('livingStreet') == 'living_street'
    assert mapd_source.highway_class_name('trunkLink') == 'trunk_link'

  def test_simple_names_pass_through(self):
    assert mapd_source.highway_class_name('motorway') == 'motorway'
    assert mapd_source.highway_class_name('residential') == 'residential'

  def test_unknown_becomes_empty_string(self):
    # '' is what speedlimitd already treats as "no OSM classification".
    assert mapd_source.highway_class_name('unknown') == ''

  def test_unrecognised_value_is_empty_not_an_error(self):
    # A future mapd release adding an enum member must degrade to "no
    # classification", not crash the daemon.
    assert mapd_source.highway_class_name('someFutureClass') == ''


class TestTelemetry:
  def test_reports_fields_for_the_phase1_drive(self):
    out = FakeMapdOut(wayRef='S20', roadName='外环高速', speedLimit=27.8,
                      lanes=4, highwayClass='motorway', wayId=42,
                      tileLoaded=True, distanceFromWayCenter=1.5,
                      waySelectionType='current', roadContext='freeway')
    t = mapd_source.telemetry_from_mapd(out, valid=True, our_way_ref='S20')
    assert t['mapdAlive'] is True
    assert t['mapdWayRef'] == 'S20'
    assert t['mapdWayId'] == 42
    assert t['mapdSpeedLimit'] == pytest.approx(100.1, abs=0.1)   # 27.8 m/s → km/h
    assert t['mapdHwClass'] == 'motorway'
    assert t['mapdLanes'] == 4
    assert t['mapdSelType'] == 'current'
    assert t['mapdTileLoaded'] is True
    assert t['mapdDistance'] == pytest.approx(1.5)
    assert t['mapdRefAgree'] is True
    # mapd's OWN urban/rural verdict — the S100+ reclassification concern in the
    # design spec cannot be checked from the rlog without it.
    assert t['mapdRoadContext'] == 'freeway'

  def test_road_context_is_stringified_not_the_enum_object(self):
    # The publisher writes into a capnp Text field; a _DynamicEnum would not
    # serialise. str() is the same treatment mapdSelType already gets.
    t = mapd_source.telemetry_from_mapd(FakeMapdOut(roadContext='city'), True, '')
    assert t['mapdRoadContext'] == 'city'
    assert isinstance(t['mapdRoadContext'], str)

  def test_ref_disagreement_is_the_headline_number(self):
    out = FakeMapdOut(wayRef='S20', roadName='x')
    t = mapd_source.telemetry_from_mapd(out, valid=True, our_way_ref='G1503')
    assert t['mapdRefAgree'] is False

  def test_dead_mapd_reports_not_alive_with_neutral_values(self):
    t = mapd_source.telemetry_from_mapd(None, valid=False, our_way_ref='S20')
    assert t['mapdAlive'] is False
    assert t['mapdWayRef'] == ''
    assert t['mapdSpeedLimit'] == 0.0
    assert t['mapdRefAgree'] is False
    assert t['mapdRoadContext'] == ''

  def test_always_returns_the_same_keys(self):
    # The rlog schema must not depend on mapd being up, or a drive with a dead
    # mapd would be unanalysable.
    live = mapd_source.telemetry_from_mapd(FakeMapdOut(wayRef='S20'), True, 'S20')
    dead = mapd_source.telemetry_from_mapd(None, False, 'S20')
    assert set(live) == set(dead)


# plugins/mapd/cereal holds the real generated schema. slot19.capnp is a
# struct-body FRAGMENT (no wrapper) that install.sh's custom_capnp.py injects
# into openpilot's cereal/custom.capnp in place of CustomReserved19; to load it
# standalone here it is reassembled the same way
# plugins/mapd/tests/test_slot_schemas.py does: the fragment wrapped in its own
# `struct MapdOut { ... }` declaration, plus the shared enum defs it needs from
# standalone.capnp.
#
# Only the ENUM blocks are pulled from standalone.capnp, not the whole file:
# standalone.capnp's structs (MapdPosition etc.) carry explicit, hand-assigned
# @0x IDs, and plugins/mapd/tests/test_slot_schemas.py already loads a merged
# file containing the full standalone.capnp text in this same pytest process.
# capnp's schema loader aborts the whole interpreter (uncaught kj::Exception,
# "Duplicate ID @0x...") if those explicit IDs are registered a second time by
# a second capnp.load() call — reproduced directly while building this fixture.
# Enums have no explicit ID here, so their (file-id, name)-derived ID differs
# safely between the two independently-loaded files.
_MAPD_CEREAL_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                 '..', 'mapd', 'cereal')

# capnp requires a file ID. Arbitrary but fixed; never persisted anywhere.
_FILE_ID = '@0xd4a1f0b3e2c9a716;'

# Enums referenced by MapdOut's fields (waySelectionType, roadContext,
# highwayClass) — the only standalone.capnp types this fixture needs.
_REQUIRED_ENUMS = ('WaySelectionType', 'RoadContext', 'HighwayClass')


def _extract_enum_blocks(standalone_text, names):
  """Pull the named top-level `enum X { ... }` blocks out of standalone.capnp.

  Reads the real, current file rather than hand-copying enum bodies, so this
  stays in sync with upstream drift the same way test_slot_schemas.py's own
  EXPECTED_HIGHWAY_CLASS check does. Enums in this file have no nested braces,
  so a non-greedy match to the first `^}` closes each block correctly.
  """
  found = {}
  for m in re.finditer(r'^enum (\w+) \{.*?^\}\n', standalone_text, re.DOTALL | re.MULTILINE):
    found[m.group(1)] = m.group(0)
  missing = [n for n in names if n not in found]
  assert not missing, f'enum(s) not found in standalone.capnp: {missing}'
  return '\n'.join(found[n] for n in names)


@pytest.fixture(scope='module')
def real_mapdout_schema(tmp_path_factory):
  """Reassemble the needed standalone.capnp enums + the MapdOut slot fragment
  into one loadable file.

  The capnp gate lives here, not at module scope (see
  plugins/speedlimitd/tests/test_generate_hw_tiles.py / plugins/mapd/tests/
  test_slot_schemas.py for the established pattern): a module-scope
  pytest.importorskip('capnp') would skip collection of this whole file,
  whereas gating inside the fixture only skips the tests that request it —
  the other 8 tests above keep running with no capnp present.
  """
  capnp = pytest.importorskip('capnp')
  with open(os.path.join(_MAPD_CEREAL_DIR, 'standalone.capnp')) as f:
    standalone_text = f.read()
  with open(os.path.join(_MAPD_CEREAL_DIR, 'slot19.capnp')) as f:
    body = f.read()
  parts = [
    _FILE_ID,
    _extract_enum_blocks(standalone_text, _REQUIRED_ENUMS),
    f'struct MapdOut {{\n{body}}}\n',
  ]
  merged = tmp_path_factory.mktemp('capnp') / 'merged.capnp'
  merged.write_text('\n'.join(parts))
  return capnp.load(str(merged))


class TestRealCapnpEnum:
  """Pins the adapter against an actual generated MapdOut, not FakeMapdOut.

  On a real message, mapd_out.highwayClass is a capnp._DynamicEnum, not a str
  -- isinstance(hc, str) is False. mapd_source's _CLASS_TO_OSM.get(value, '')
  still returns the right answer because _DynamicEnum hashes/compares equal to
  the matching enumerant name string, but FakeMapdOut above only ever hands
  the adapter plain strings, so nothing else in the suite would notice if that
  equivalence broke. It matters because the car runs pycapnp 2.1.0 (pinned for
  an unrelated memory leak) while this dev machine runs 2.2.4, and the
  _DynamicEnum hash/eq behaviour has only ever been observed on 2.2.4: if it
  differs on-device, every road silently classifies as '' with no error.
  """

  def test_compound_enum_value_is_not_a_str(self, real_mapdout_schema):
    msg = real_mapdout_schema.MapdOut.new_message()
    msg.highwayClass = 'motorwayLink'
    assert not isinstance(msg.highwayClass, str)

  def test_compound_name_from_real_enum(self, real_mapdout_schema):
    msg = real_mapdout_schema.MapdOut.new_message()
    msg.highwayClass = 'motorwayLink'
    assert mapd_source.highway_class_name(msg.highwayClass) == 'motorway_link'

  def test_simple_name_from_real_enum(self, real_mapdout_schema):
    msg = real_mapdout_schema.MapdOut.new_message()
    msg.highwayClass = 'motorway'
    assert mapd_source.highway_class_name(msg.highwayClass) == 'motorway'

  def test_default_unset_enum_is_empty(self, real_mapdout_schema):
    msg = real_mapdout_schema.MapdOut.new_message()   # highwayClass left unset
    assert mapd_source.highway_class_name(msg.highwayClass) == ''

  def test_telemetry_end_to_end_from_real_message(self, real_mapdout_schema):
    msg = real_mapdout_schema.MapdOut.new_message()
    msg.wayRef = 'S20'
    msg.speedLimit = 27.8
    msg.lanes = 4
    msg.highwayClass = 'motorway'
    msg.wayId = 42
    msg.tileLoaded = True
    msg.distanceFromWayCenter = 1.5
    msg.waySelectionType = 'current'
    msg.roadContext = 'freeway'
    with real_mapdout_schema.MapdOut.from_bytes(msg.to_bytes()) as out:
      t = mapd_source.telemetry_from_mapd(out, valid=True, our_way_ref='S20')
    assert t['mapdWayRef'] == 'S20'
    assert t['mapdWayId'] == 42
    assert t['mapdSpeedLimit'] == pytest.approx(100.1, abs=0.1)   # 27.8 m/s → km/h
    assert t['mapdHwClass'] == 'motorway'
    assert t['mapdLanes'] == 4
    assert t['mapdSelType'] == 'current'
    assert t['mapdTileLoaded'] is True
    assert t['mapdDistance'] == pytest.approx(1.5)
    assert t['mapdRefAgree'] is True
    assert t['mapdRoadContext'] == 'freeway'

  def test_class_map_covers_every_enumerant_exactly(self, real_mapdout_schema):
    """_CLASS_TO_OSM keys == the real HighwayClass enumerants, no drift.

    Without this, renaming an enumerant in standalone.capnp and in
    test_slot_schemas.py's EXPECTED_HIGHWAY_CLASS together passes the whole
    suite while the adapter silently returns '' for that class — every road of
    that type loses its OSM classification with no error anywhere (mutation
    verified). The schema is the source of truth in BOTH directions: a missing
    key means a silent '' and an extra key is dead code hiding a rename.
    """
    enumerants = set(real_mapdout_schema.HighwayClass.schema.enumerants.keys())
    assert set(mapd_source._CLASS_TO_OSM.keys()) == enumerants
