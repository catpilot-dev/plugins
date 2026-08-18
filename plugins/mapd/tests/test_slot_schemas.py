"""Bus schema tests — mapd slots 17-19 must match mapd v2.3.0's custom.capnp.

The slotN.capnp files are FRAGMENTS (struct bodies with no wrapper):
install.sh's custom_capnp.py injects them into openpilot's cereal/custom.capnp
in place of the CustomReservedN stubs. To parse them standalone this module
reassembles a valid schema file — standalone.capnp plus each fragment wrapped
in its struct declaration — and loads that.

The capnp gate lives inside the `schema` fixture below, not at module scope
(see plugins/speedlimitd/tests/test_generate_hw_tiles.py for the established
pattern): a module-scope `pytest.importorskip('capnp')` skips collection of
the whole file as a single opaque unit, whereas gating inside the fixture
lets pytest attribute the skip to each individual test that requests it.
"""
import os

import pytest

CEREAL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'cereal')

# capnp requires a file ID. Arbitrary but fixed; never persisted anywhere.
FILE_ID = '@0xbdc0e1e5a4c9f8d2;'

SLOTS = ((17, 'MapdExtendedOut'), (18, 'MapdIn'), (19, 'MapdOut'))

# HighwayClass, copied verbatim from mapd v2.3.0 cereal/custom/custom.capnp.
# Upstream requires this to stay name- and value-identical to
# cereal/offline/offline.capnp because state.go casts between the generated
# enum types. Any drift here is a silent mis-classification on device.
EXPECTED_HIGHWAY_CLASS = {
  'unknown': 0,
  'motorway': 1,
  'motorwayLink': 2,
  'trunk': 3,
  'trunkLink': 4,
  'primary': 5,
  'primaryLink': 6,
  'secondary': 7,
  'secondaryLink': 8,
  'tertiary': 9,
  'tertiaryLink': 10,
  'unclassified': 11,
  'residential': 12,
  'livingStreet': 13,
}


@pytest.fixture(scope='module')
def schema(tmp_path_factory):
  """Reassemble standalone.capnp + wrapped slot fragments into one loadable file."""
  capnp = pytest.importorskip('capnp')
  parts = [FILE_ID]
  with open(os.path.join(CEREAL_DIR, 'standalone.capnp')) as f:
    parts.append(f.read())
  for num, struct_name in SLOTS:
    with open(os.path.join(CEREAL_DIR, f'slot{num}.capnp')) as f:
      body = f.read()
    parts.append(f'struct {struct_name} {{\n{body}}}\n')
  merged = tmp_path_factory.mktemp('capnp') / 'merged.capnp'
  merged.write_text('\n'.join(parts))
  return capnp.load(str(merged))


class TestMapdOut:
  def test_has_v230_fields(self, schema):
    fields = schema.MapdOut.schema.fieldnames
    assert 'highwayClass' in fields
    assert 'wayId' in fields
    assert 'conditionalSpeedLimit' in fields

  def test_preexisting_fields_unmoved(self, schema):
    # Additions only — the v2.0.5 fields keep their ordinals, so an older
    # binary still round-trips against this schema.
    fields = schema.MapdOut.schema.fieldnames
    assert fields[0] == 'wayName'
    assert fields[23] == 'speedLimitAccepted'

  def test_new_fields_are_appended_in_order(self, schema):
    fields = schema.MapdOut.schema.fieldnames
    assert fields[24:27] == ('highwayClass', 'wayId', 'conditionalSpeedLimit')

  def test_roundtrip_new_fields(self, schema):
    msg = schema.MapdOut.new_message()
    msg.wayRef = 'S20'
    msg.highwayClass = 'motorway'
    msg.wayId = 123456789
    msg.conditionalSpeedLimit = '100 @ (Mo-Fr 06:00-20:00)'
    with schema.MapdOut.from_bytes(msg.to_bytes()) as out:
      assert out.wayRef == 'S20'
      assert out.highwayClass == 'motorway'
      assert out.wayId == 123456789
      assert out.conditionalSpeedLimit == '100 @ (Mo-Fr 06:00-20:00)'


class TestHighwayClass:
  def test_members_match_upstream_exactly(self, schema):
    assert dict(schema.HighwayClass.schema.enumerants) == EXPECTED_HIGHWAY_CLASS


class TestMapdIn:
  def test_has_json_path(self, schema):
    assert 'jsonPath' in schema.MapdIn.schema.fieldnames

  def test_json_path_roundtrip(self, schema):
    msg = schema.MapdIn.new_message()
    msg.jsonPath = 'speed_limit.offset'
    with schema.MapdIn.from_bytes(msg.to_bytes()) as out:
      assert out.jsonPath == 'speed_limit.offset'


class TestMapdExtendedOut:
  def test_has_position(self, schema):
    assert 'position' in schema.MapdExtendedOut.schema.fieldnames

  def test_position_roundtrip(self, schema):
    msg = schema.MapdExtendedOut.new_message()
    msg.position.latitude = 31.3137
    msg.position.longitude = 121.5395
    with schema.MapdExtendedOut.from_bytes(msg.to_bytes()) as out:
      assert out.position.latitude == pytest.approx(31.3137)
      assert out.position.longitude == pytest.approx(121.5395)


class TestMapdInputType:
  def test_has_v230_setters(self, schema):
    members = dict(schema.MapdInputType.schema.enumerants)
    assert members['setConditionalSpeedLimitControl'] == 39
    assert members['setShadowCarState'] == 40
    assert members['setShadowModelV2'] == 41
    assert members['setShadowGpsLocation'] == 42
    assert members['setJsonPathFloat'] == 43
    assert members['setJsonPathText'] == 44
    assert members['setJsonPathBool'] == 45
    assert members['setShadowGpsLocationExternal'] == 46
