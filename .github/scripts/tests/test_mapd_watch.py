"""Tests for the mapd release watch gate, schema diff and reconcile logic."""
import importlib.util
import pathlib
import sys

import pytest

_SCRIPT = pathlib.Path(__file__).resolve().parents[1] / 'mapd_watch.py'
_spec = importlib.util.spec_from_file_location('mapd_watch', _SCRIPT)
mw = importlib.util.module_from_spec(_spec)
sys.modules['mapd_watch'] = mw
_spec.loader.exec_module(mw)


# mapd v2.3.0's cereal/custom/custom.capnp, verbatim but for the sixteen empty
# `CustomReserved` structs. All THREE structs we consume are here — MapdIn,
# MapdExtendedOut and MapdOut — plus the enums they reference and the plain
# structs that must not leak into anyone's field list. Verbatim upstream text:
# the clean case needs a real fixture, or "identical" only ever proves the
# parser agrees with itself.
#
# This is now the OLDER of the two real fixtures: our slots have caught up to
# v2.3.1, so v2.3.0 is behind us and is kept only to prove the differ reports
# that direction as `removed` rather than as a clean diff.
_V230_CUSTOM_CAPNP = """using Go = import "/go.capnp";
@0xb526ba661d550a59;
$Go.package("custom");
$Go.import("pfeifer.dev/mapd/cereal/custom");

# custom.capnp: a home for empty structs reserved for custom forks
# These structs are guaranteed to remain reserved and empty in mainline
# cereal, so use these if you want custom events in your fork.

# DO rename the structs
# DON'T change the identifier (e.g. @0x81c2f05a394cf4af)

struct MapdDownloadLocationDetails @0xff889853e7b0987f {
  location @0 :Text;
  totalFiles @1 :UInt32;
  downloadedFiles @2 :UInt32;
}

struct MapdDownloadProgress @0xfaa35dcac85073a2 {
  active @0 :Bool;
  cancelled @1 :Bool;
  totalFiles @2 :UInt32;
  downloadedFiles @3 :UInt32;
  locations @4 :List(Text);
  locationDetails @5 :List(MapdDownloadLocationDetails);
}

struct MapdPathPoint @0xd6f78acca1bc3939 {
  latitude @0 :Float64;
  longitude @1 :Float64;
  curvature @2 :Float32;
  targetVelocity @3 :Float32;
}

struct MapdPosition @0xde9705979aca8339 {
  latitude @0 :Float64;
  longitude @1 :Float64;
}

struct MapdExtendedOut @0xa30662f84033036c {
  downloadProgress @0 :MapdDownloadProgress;
  settings @1 :Text;
  path @2 :List(MapdPathPoint);
  position @3 :MapdPosition;
}

enum MapdInputType {
  download @0;
  reloadSettings @9;
  saveSettings @10;
  loadDefaultSettings @21;
  loadRecommendedSettings @22;
  loadPersistentSettings @26;
  cancelDownload @27;
  setJsonPathFloat @43;
  setJsonPathText @44;
  setJsonPathBool @45;
  acceptSpeedLimit @34;

  # DEPRECATED settings inputs
  setLogLevel @6;
  setLogSource @29;
  setLogJson @28;
  setTargetLateralAccel @1;
  setSpeedLimitOffset @2;
  setSpeedLimitControl @3;
  setMapCurveSpeedControl @4;
  setVisionCurveSpeedControl @5;
  setVisionCurveTargetLatA @7;
  setVisionCurveMinTargetV @8;
  setEnableSpeed @11;
  setVisionCurveUseEnableSpeed @12;
  setMapCurveUseEnableSpeed @13;
  setSpeedLimitUseEnableSpeed @14;
  setHoldLastSeenSpeedLimit @15;
  setTargetSpeedJerk @16;
  setTargetSpeedAccel @17;
  setTargetSpeedTimeOffset @18;
  setDefaultLaneWidth @19;
  setMapCurveTargetLatA @20;
  setSlowDownForNextSpeedLimit @23;
  setSpeedUpForNextSpeedLimit @24;
  setHoldSpeedLimitWhileChangingSetSpeed @25;
  setExternalSpeedLimitControl @30;
  setExternalSpeedLimit @31;
  setSpeedLimitPriority @32;
  setSpeedLimitChangeRequiresAccept @33;
  setPressGasToAcceptSpeedLimit @35;
  setAdjustSetSpeedToAcceptSpeedLimit @36;
  setAcceptSpeedLimitTimeout @37;
  setPressGasToOverrideSpeedLimit @38;
  setConditionalSpeedLimitControl @39;
  setShadowCarState @40;
  setShadowModelV2 @41;
  setShadowGpsLocation @42;
  setShadowGpsLocationExternal @46;
}

enum WaySelectionType {
  current @0;
  predicted @1;
  possible @2;
  extended @3;
  fail @4;
}

enum SpeedLimitOffsetType {
  static @0;
  percent @1;
}

struct MapdIn @0xc86a3d38d13eb3ef {
  type @0 :MapdInputType;
  float @1 :Float32;
  str @2 :Text;
  bool @3 :Bool;
  jsonPath @4 :Text;
}

enum RoadContext {
  freeway @0;
  city @1;
  unknown @2;
}

# WARNING: must be kept in perfect sync (names and values) with the
# HighwayClass enum in cereal/offline/offline.capnp — state.go casts directly
# between the two generated enum types.
# unknown either means the way's highway tag was not one of the listed values
# or the loaded map tiles predate this field.
enum HighwayClass {
  unknown @0;
  motorway @1;
  motorwayLink @2;
  trunk @3;
  trunkLink @4;
  primary @5;
  primaryLink @6;
  secondary @7;
  secondaryLink @8;
  tertiary @9;
  tertiaryLink @10;
  unclassified @11;
  residential @12;
  livingStreet @13;
}

struct MapdOut @0xa4f1eb3323f5f582 {
  wayName @0 :Text;
  wayRef @1 :Text;
  roadName @2 :Text;
  speedLimit @3 :Float32;
  nextSpeedLimit @4 :Float32;
  nextSpeedLimitDistance @5 :Float32;
  hazard @6 :Text;
  nextHazard @7 :Text;
  nextHazardDistance @8 :Float32;
  advisorySpeed @9 :Float32;
  nextAdvisorySpeed @10 :Float32;
  nextAdvisorySpeedDistance @11 :Float32;
  oneWay @12 :Bool;
  lanes @13 :UInt8;
  tileLoaded @14 :Bool;
  speedLimitSuggestedSpeed @15 :Float32;
  suggestedSpeed @16 :Float32;
  estimatedRoadWidth @17 :Float32;
  roadContext @18 :RoadContext;
  distanceFromWayCenter @19 :Float32;
  visionCurveSpeed @20 :Float32;
  mapCurveSpeed @21 :Float32;
  waySelectionType @22 :WaySelectionType;
  speedLimitAccepted @23 :Bool;
  highwayClass @24 :HighwayClass;
  wayId @25 :Int64;
  conditionalSpeedLimit @26 :Text;
}
"""

# v2.3.1 changed exactly this and nothing else: two new MapdExtendedOut fields,
# MapdOut untouched — which is why the single-struct differ called it
# "schema-safe. No new fields." (issue #30). We have since adopted both fields
# and pinned v2.3.1, so THIS is the clean baseline: every struct here is
# field-identical to the slot files on disk, and every synthetic mutation below
# is exactly one deliberate step away from clean.
_V231_CUSTOM_CAPNP = _V230_CUSTOM_CAPNP.replace(
    '  position @3 :MapdPosition;\n',
    '  position @3 :MapdPosition;\n'
    '  loopRateAverage @4 :Float32;\n  loopRateMin @5 :Float32;\n')

# The multi-struct regression guard (issue #30), rebuilt so it cannot expire.
# A HYPOTHETICAL field added to MapdExtendedOut with MapdOut left alone — the
# same shape as the v2.3.1 bug, but synthetic, because pinning the guard to a
# real release only works while our slots lag that release. They no longer lag
# v2.3.1; they will not lag whatever comes next either, once it is adopted.
# Synthetic keeps "a change confined to a non-MapdOut struct is still caught"
# true forever.
_EXTENDED_ONLY_CHANGE_CAPNP = _V231_CUSTOM_CAPNP.replace(
    '  loopRateMin @5 :Float32;\n',
    '  loopRateMin @5 :Float32;\n  someFutureField @6 :Float32;\n')
assert 'someFutureField' in _EXTENDED_ONLY_CHANGE_CAPNP  # fixture, not a no-op

# The real slot files, read from disk rather than fixtured: that makes every
# "identical" assertion below a live guard against OUR schema drifting too.
_OURS = mw.read_local_slots()
_OURS_FIELDS = _OURS['MapdOut']
_OURS_ENUMS = mw.STANDALONE_CAPNP.read_text()


def _release(tag='v2.4.0', **over):
  r = {'tag_name': tag, 'draft': False, 'prerelease': False,
       'published_at': '2026-09-01T00:00:00Z',
       'html_url': f'https://github.com/pfeiferj/mapd/releases/tag/{tag}',
       'assets': [{'name': 'mapd'}]}
  r.update(over)
  return r


def _issue(tag, state='open'):
  return {'number': 1, 'state': state, 'body': f'{mw.MARKER} {tag}\n\nrest'}


class _FakeResponse:
  def __init__(self, payload, status_code=200):
    self._payload = payload
    self.status_code = status_code

  def raise_for_status(self):
    if self.status_code >= 400:
      raise mw.requests.HTTPError(f'status {self.status_code}')

  def json(self):
    return self._payload


class TestWatchedStructs:
  def test_every_slot_file_we_ship_is_watched(self):
    """A slot file nobody diffs is a field drop waiting to happen — the exact
    shape of the bug that let v2.3.1 through."""
    shipped = {p.name for p in mw._CEREAL_DIR.glob('slot*.capnp')}
    assert {p.name for p in mw.WATCHED_STRUCTS.values()} == shipped

  def test_slot_names_are_repo_relative_for_the_issue_body(self):
    assert mw.slot_name('MapdExtendedOut') == 'plugins/mapd/cereal/slot17.capnp'


class TestParseFields:
  """Two shapes, one parser: mapd wraps each struct in a struct block, our slot
  files are bare fragments of field lines that custom_capnp.py splices into the
  real struct at install time."""

  def test_parses_the_wrapped_upstream_struct(self):
    fields = mw.parse_fields(_V231_CUSTOM_CAPNP, 'MapdOut')
    assert len(fields) == 27
    assert sorted(f['ordinal'] for f in fields.values()) == list(range(27))
    assert fields['highwayClass'] == {'ordinal': 24, 'type': 'HighwayClass'}

  def test_parses_our_bare_fragment(self):
    fields = mw.parse_fields(_OURS_FIELDS, 'MapdOut')
    assert len(fields) == 27
    assert sorted(f['ordinal'] for f in fields.values()) == list(range(27))
    assert fields['conditionalSpeedLimit'] == {'ordinal': 26, 'type': 'Text'}

  def test_parses_the_other_two_watched_structs(self):
    """MapdExtendedOut and MapdIn are consumed exactly like MapdOut."""
    assert mw.parse_fields(_V231_CUSTOM_CAPNP, 'MapdExtendedOut') == {
        'downloadProgress': {'ordinal': 0, 'type': 'MapdDownloadProgress'},
        'settings': {'ordinal': 1, 'type': 'Text'},
        'path': {'ordinal': 2, 'type': 'List(MapdPathPoint)'},
        'position': {'ordinal': 3, 'type': 'MapdPosition'},
        'loopRateAverage': {'ordinal': 4, 'type': 'Float32'},
        'loopRateMin': {'ordinal': 5, 'type': 'Float32'},
    }
    assert set(mw.parse_fields(_V231_CUSTOM_CAPNP, 'MapdIn')) == {
        'type', 'float', 'str', 'bool', 'jsonPath'}

  def test_does_not_absorb_fields_from_neighbouring_structs(self):
    """MapdPosition's latitude/longitude sit above MapdOut in the same file."""
    assert 'latitude' not in mw.parse_fields(_V231_CUSTOM_CAPNP, 'MapdOut')

  def test_missing_struct_in_a_file_with_blocks_is_an_error_not_a_scrape(self):
    """A rename or deletion must fail loudly. Falling back to bare-fragment
    mode here would scrape every field line in the file and report a
    plausible-looking diff against a struct that no longer exists."""
    with pytest.raises(mw.SchemaError):
      mw.parse_fields(_V231_CUSTOM_CAPNP.replace('struct MapdOut',
                                                 'struct MapdOutV2'), 'MapdOut')

  def test_ignores_commented_out_field_lines(self):
    text = _OURS_FIELDS + "\n  # futureField @27 :Text;\n"
    assert 'futureField' not in mw.parse_fields(text, 'MapdOut')


class TestParseEnum:
  def test_reads_members_and_ordinals(self):
    assert mw.parse_enum(_OURS_ENUMS, 'RoadContext') == {
        'freeway': 0, 'city': 1, 'unknown': 2}

  def test_returns_none_for_an_undeclared_enum(self):
    assert mw.parse_enum(_OURS_ENUMS, 'NoSuchEnum') is None

  def test_referenced_enums_are_derived_from_the_field_types(self):
    fields = mw.parse_fields(_V231_CUSTOM_CAPNP, 'MapdOut')
    assert set(mw.referenced_enums(fields, _V231_CUSTOM_CAPNP)) == {
        'RoadContext', 'WaySelectionType', 'HighwayClass'}

  def test_struct_typed_fields_are_not_mistaken_for_enums(self):
    """MapdExtendedOut's fields are structs, lists and floats — no enums."""
    fields = mw.parse_fields(_V231_CUSTOM_CAPNP, 'MapdExtendedOut')
    assert mw.referenced_enums(fields, _V231_CUSTOM_CAPNP) == []


class TestDiffSchema:
  def test_v231_is_identical_to_our_slots(self):
    """The real baseline: all three structs at v2.3.1 — the version we pin and
    run — are field-identical to slot17/18/19.capnp. The counts are asserted
    too, and per struct: a differ that parsed nothing would otherwise report
    'identical' vacuously."""
    diff = mw.diff_schema(_V231_CUSTOM_CAPNP, _OURS, _OURS_ENUMS)
    assert diff['identical']
    assert set(diff['structs']) == set(mw.WATCHED_STRUCTS)
    counts = {n: (s['theirs_field_count'], s['ours_field_count'])
              for n, s in diff['structs'].items()}
    assert counts == {'MapdExtendedOut': (6, 6), 'MapdIn': (5, 5),
                      'MapdOut': (27, 27)}
    for s in diff['structs'].values():
      assert s['identical']
      assert s['added'] == s['removed'] == s['changed'] == []

  def test_v230_is_now_behind_our_slots(self):
    """Our slots caught up to v2.3.1, so the older release is the side that is
    missing fields. That must surface as `removed` (fields WE declare that
    upstream lacks) and never as 'identical': downgrading the pin is safe to
    read but is not a pure add, and the issue body has to say so."""
    diff = mw.diff_schema(_V230_CUSTOM_CAPNP, _OURS, _OURS_ENUMS)
    assert not diff['identical']
    assert diff['structs']['MapdOut']['identical']
    assert diff['structs']['MapdIn']['identical']
    extended = diff['structs']['MapdExtendedOut']
    assert not extended['identical']
    assert extended['added'] == []
    assert extended['removed'] == [('loopRateAverage', 4, 'Float32'),
                                   ('loopRateMin', 5, 'Float32')]
    assert extended['theirs_field_count'] == 4
    assert extended['ours_field_count'] == 6

  def test_a_change_outside_mapd_out_is_still_caught(self):
    """REGRESSION (issue #30). The differ once read MapdOut and nothing else,
    so v2.3.1 — two new MapdExtendedOut fields, MapdOut untouched — was filed
    as "schema-safe. No new fields." and both fields would have dropped
    silently into slot17.

    We have since adopted those two fields, so the guard is rebuilt from a
    SYNTHETIC field instead: it must not depend on our slots lagging any
    particular release, because that lag is exactly what gets fixed."""
    diff = mw.diff_schema(_EXTENDED_ONLY_CHANGE_CAPNP, _OURS, _OURS_ENUMS)
    assert not diff['identical']
    assert diff['structs']['MapdOut']['identical']
    assert diff['structs']['MapdIn']['identical']
    extended = diff['structs']['MapdExtendedOut']
    assert not extended['identical']
    assert extended['added'] == [('someFutureField', 6, 'Float32')]
    assert extended['slot'] == 'plugins/mapd/cereal/slot17.capnp'

  def test_reports_an_upstream_added_field(self):
    theirs = _V231_CUSTOM_CAPNP.replace(
        '  conditionalSpeedLimit @26 :Text;',
        '  conditionalSpeedLimit @26 :Text;\n  tollRoad @27 :Bool;')
    diff = mw.diff_schema(theirs, _OURS, _OURS_ENUMS)
    assert not diff['identical']
    assert diff['structs']['MapdOut']['added'] == [('tollRoad', 27, 'Bool')]
    assert diff['structs']['MapdOut']['theirs_field_count'] == 28

  def test_reports_a_field_added_to_mapd_in(self):
    """MapdIn is what WE publish, so an upstream addition there is a command we
    cannot send — silent in exactly the same way."""
    theirs = _V231_CUSTOM_CAPNP.replace(
        '  jsonPath @4 :Text;\n}', '  jsonPath @4 :Text;\n  int @5 :Int64;\n}')
    diff = mw.diff_schema(theirs, _OURS, _OURS_ENUMS)
    assert not diff['identical']
    assert diff['structs']['MapdIn']['added'] == [('int', 5, 'Int64')]
    assert diff['structs']['MapdOut']['identical']

  def test_reports_an_upstream_added_enumerant(self):
    theirs = _V231_CUSTOM_CAPNP.replace(
        '  livingStreet @13;', '  livingStreet @13;\n  service @14;')
    diff = mw.diff_schema(theirs, _OURS, _OURS_ENUMS)
    assert not diff['identical']
    assert diff['structs']['MapdOut']['added'] == []
    highway = next(e for e in diff['enums'] if e['name'] == 'HighwayClass')
    assert highway['added'] == [('service', 14)]

  def test_diffs_an_enum_reached_only_through_mapd_in(self):
    """MapdInputType is referenced by no MapdOut field at all. A new input type
    we cannot name is a command we cannot send."""
    theirs = _V231_CUSTOM_CAPNP.replace(
        '  setShadowGpsLocationExternal @46;',
        '  setShadowGpsLocationExternal @46;\n  setShadowLiveDelay @47;')
    diff = mw.diff_schema(theirs, _OURS, _OURS_ENUMS)
    assert not diff['identical']
    inputs = next(e for e in diff['enums'] if e['name'] == 'MapdInputType')
    assert inputs['added'] == [('setShadowLiveDelay', 47)]

  def test_reports_a_renumbered_field_as_changed(self):
    theirs = _V231_CUSTOM_CAPNP.replace('  wayId @25 :Int64;',
                                        '  wayId @25 :UInt64;')
    diff = mw.diff_schema(theirs, _OURS, _OURS_ENUMS)
    assert [n for n, _, _ in diff['structs']['MapdOut']['changed']] == ['wayId']

  def test_reports_an_enum_we_do_not_declare_at_all(self):
    theirs = _V231_CUSTOM_CAPNP.replace(
        '  conditionalSpeedLimit @26 :Text;',
        '  conditionalSpeedLimit @26 :Text;\n  surface @27 :SurfaceType;'
    ).replace('struct MapdOut',
              'enum SurfaceType {\n  paved @0;\n  gravel @1;\n}\n\nstruct MapdOut')
    diff = mw.diff_schema(theirs, _OURS, _OURS_ENUMS)
    surface = next(e for e in diff['enums'] if e['name'] == 'SurfaceType')
    assert surface['missing']

  def test_a_side_that_parsed_to_nothing_is_an_error_not_a_clean_diff(self):
    with pytest.raises(mw.SchemaError):
      mw.diff_schema('', _OURS, _OURS_ENUMS)

  def test_a_struct_upstream_dropped_is_an_error_not_a_clean_diff(self):
    """Only MapdOut renamed — the other two still parse. The verdict must not
    be 'two of three are fine'."""
    with pytest.raises(mw.SchemaError):
      mw.diff_schema(_V231_CUSTOM_CAPNP.replace('struct MapdExtendedOut',
                                                'struct MapdExtendedOutV2'),
                     _OURS, _OURS_ENUMS)


class TestSchemaSection:
  def test_clean_diff_says_schema_safe(self):
    section = mw.schema_section('v2.4.0', fetch=lambda tag: _V231_CUSTOM_CAPNP)
    assert 'schema-safe' in section
    # Per-struct counts, so a vacuous "identical" stays impossible.
    assert '`MapdOut` upstream: 27 fields' in section
    assert '`MapdExtendedOut` upstream: 6 fields' in section
    assert '`MapdIn` upstream: 5 fields' in section

  def test_added_field_is_listed_by_name_and_ordinal(self):
    theirs = _V231_CUSTOM_CAPNP.replace(
        '  conditionalSpeedLimit @26 :Text;',
        '  conditionalSpeedLimit @26 :Text;\n  tollRoad @27 :Bool;')
    section = mw.schema_section('v2.4.0', fetch=lambda tag: theirs)
    assert '`tollRoad @27 :Bool;`' in section
    assert 'add to `plugins/mapd/cereal/slot19.capnp`' in section
    assert 'schema-safe' not in section

  def test_change_outside_mapd_out_names_the_struct_and_the_slot_file(self):
    """REGRESSION (issue #30): the filed issue said 'schema-safe. No new
    fields.' A change confined to a struct other than MapdOut must now name
    that struct and point at its slot file. Synthetic (see
    _EXTENDED_ONLY_CHANGE_CAPNP) so the guard outlives every pin bump."""
    section = mw.schema_section(
        'v2.4.0', fetch=lambda tag: _EXTENDED_ONLY_CHANGE_CAPNP)
    assert 'schema-safe' not in section
    assert '`MapdExtendedOut`' in section
    assert 'slot17' in section
    assert 'add to `plugins/mapd/cereal/slot17.capnp`' in section
    assert '`someFutureField @6 :Float32;`' in section
    # The clean structs are still named, with their counts, so "we checked all
    # three" is visible rather than assumed.
    assert '`MapdOut` → `plugins/mapd/cereal/slot19.capnp`: identical.' in section
    assert '`MapdOut` upstream: 27 fields' in section

  def test_unfetchable_file_still_yields_a_section_stating_the_gap(self):
    """The one exception to fail-loudly: a release notification is worth more
    than a perfect issue body, and the missing check is stated, not implied."""
    def _boom(tag):
      raise mw.requests.HTTPError('404')

    section = mw.schema_section('v2.4.0', fetch=_boom)
    assert 'Could not be checked' in section
    assert 'by hand' in section
    # Every slot the reader has to diff by hand is named.
    for slot in ('slot17.capnp', 'slot18.capnp', 'slot19.capnp'):
      assert slot in section


class TestCommitSection:
  """The gate is RETIRED (`REQUIRED_COMMIT = \'\'`) now that we are pinned to a
  release carrying the gomsgq fix, so these tests arm it explicitly. They stay
  because the mechanism is re-armable: the next upstream fix that blocks us the
  same way sets the constant again, and this is the coverage proving it works.
  """

  @pytest.fixture(autouse=True)
  def _armed(self, monkeypatch):
    monkeypatch.setattr(mw, 'REQUIRED_COMMIT', 'fe45d10')

  @pytest.mark.parametrize('status', ['ahead', 'identical'])
  def test_ahead_or_identical_means_the_fix_is_in(self, status):
    section = mw.commit_section('v2.4.0', status)
    assert 'Contains' in section
    assert 'Does NOT contain' not in section

  @pytest.mark.parametrize('status', ['behind', 'diverged'])
  def test_behind_or_diverged_means_the_fix_is_missing(self, status):
    """v2.3.0 answers `behind` — the release that made this watch necessary."""
    section = mw.commit_section('v2.3.0', status)
    assert 'Does NOT contain' in section

  def test_unrecognised_status_asks_for_a_hand_check(self):
    assert 'by hand' in mw.commit_section('v2.4.0', 'wat')

  def test_retired_gate_is_the_shipped_default(self):
    """The autouse fixture arms the gate; the shipped constant must be empty,
    or every issue carries a stale blocker section for a fix we already have."""
    import importlib.util
    spec = importlib.util.spec_from_file_location('mapd_watch_fresh', mw.__file__)
    fresh = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fresh)
    assert fresh.REQUIRED_COMMIT == ''

  def test_empty_required_commit_drops_the_section(self, monkeypatch):
    monkeypatch.setattr(mw, 'REQUIRED_COMMIT', '')
    assert mw.commit_section('v2.4.0', 'behind') == ''

  def test_build_issue_skips_the_compare_call_once_the_gate_is_retired(self, monkeypatch):
    monkeypatch.setattr(mw, 'REQUIRED_COMMIT', '')
    monkeypatch.setattr(mw, 'fetch_compare_status', lambda tag: pytest.fail(
        'compare must not be called with REQUIRED_COMMIT retired'))
    monkeypatch.setattr(mw, 'fetch_upstream_capnp', lambda tag: _V231_CUSTOM_CAPNP)
    body = mw.build_issue(_release())['body']
    assert 'gomsgq' not in body


class TestPlanReleases:
  def test_newer_than_the_pin_is_reported(self):
    planned = mw.plan_releases([_release('v2.4.0')], set())
    assert [r['tag_name'] for r in planned] == ['v2.4.0']

  def test_the_pinned_release_itself_is_not_news(self):
    assert mw.plan_releases([_release(mw.MAX_ALLOWED_VERSION)], set()) == []

  def test_older_releases_are_not_news(self):
    assert mw.plan_releases([_release('v2.2.0')], set()) == []

  def test_double_digit_minor_sorts_above_the_pin(self):
    """The reason _version_tuple is imported rather than string-compared:
    'v2.10.0' > 'v2.3.0' is False lexicographically."""
    planned = mw.plan_releases([_release('v2.10.0')], set())
    assert [r['tag_name'] for r in planned] == ['v2.10.0']

  def test_unparseable_tag_degrades_to_not_newer(self):
    assert mw.plan_releases([_release('nightly')], set()) == []

  def test_drafts_and_prereleases_are_skipped(self):
    releases = [_release('v2.4.0', draft=True), _release('v2.5.0', prerelease=True)]
    assert mw.plan_releases(releases, set()) == []

  def test_already_reported_tag_is_not_refiled(self):
    assert mw.plan_releases([_release('v2.4.0')], {'v2.4.0'}) == []

  def test_reports_oldest_first(self):
    releases = [_release('v3.0.0'), _release('v2.4.0'), _release('v2.10.0')]
    assert [r['tag_name'] for r in mw.plan_releases(releases, set())] == [
        'v2.4.0', 'v2.10.0', 'v3.0.0']


class TestReportedTags:
  def test_extracts_marker_from_bodies(self):
    assert mw.reported_tags([_issue('v2.4.0'), _issue('v2.5.0')]) == {'v2.4.0', 'v2.5.0'}

  def test_closed_issues_still_count_as_reported(self):
    """Closed means handled — bumped or dismissed. Never re-report it."""
    assert mw.reported_tags([_issue('v2.4.0', state='closed')]) == {'v2.4.0'}

  def test_ignores_bodies_without_the_marker(self):
    assert mw.reported_tags([{'number': 2, 'state': 'open', 'body': 'unrelated'}]) == set()

  def test_tolerates_missing_body(self):
    assert mw.reported_tags([{'number': 3, 'state': 'open', 'body': None}]) == set()

  def test_round_trips_the_marker_it_writes(self, monkeypatch):
    """Day 1's body must feed day 2's dedup set, or every release is refiled
    daily."""
    monkeypatch.setattr(mw, 'fetch_upstream_capnp', lambda tag: _V231_CUSTOM_CAPNP)
    monkeypatch.setattr(mw, 'fetch_compare_status', lambda tag: 'ahead')
    body = mw.build_issue(_release('v2.4.0'))['body']
    reported = mw.reported_tags([{'number': 1, 'state': 'open', 'body': body}])
    assert reported == {'v2.4.0'}
    assert mw.plan_releases([_release('v2.4.0')], reported) == []


class TestFetchIssues:
  def test_queries_all_states_for_the_label_and_terminates(self, monkeypatch):
    pages_seen = []

    def fake_get(url, params=None, headers=None, timeout=None):
      pages_seen.append(dict(params))
      if params['page'] == 1:
        return _FakeResponse([{'number': 1, 'state': 'closed', 'body': ''}])
      return _FakeResponse([])

    monkeypatch.setattr(mw.requests, 'get', fake_get)
    issues = mw.fetch_issues('owner/repo')

    assert [i['number'] for i in issues] == [1]
    assert all(p['state'] == 'all' and p['labels'] == mw.LABEL for p in pages_seen)
    assert [p['page'] for p in pages_seen] == [1, 2]


class TestEnsureLabel:
  def test_already_exists_is_success(self, monkeypatch):
    monkeypatch.setattr(mw.requests, 'post',
                        lambda *a, **k: _FakeResponse({}, status_code=422))
    mw.ensure_label('owner/repo')  # must not raise

  def test_other_failures_are_fatal(self, monkeypatch):
    monkeypatch.setattr(mw.requests, 'post',
                        lambda *a, **k: _FakeResponse({}, status_code=403))
    with pytest.raises(mw.requests.HTTPError):
      mw.ensure_label('owner/repo')


class TestMain:
  def test_dry_run_plans_against_the_real_reported_set_and_creates_nothing(
      self, monkeypatch, capsys):
    monkeypatch.setattr(mw, 'fetch_releases', lambda: [_release('v2.4.0')])
    monkeypatch.setattr(mw, 'fetch_issues', lambda repo: [])
    monkeypatch.setattr(mw, 'fetch_upstream_capnp', lambda tag: _V231_CUSTOM_CAPNP)
    monkeypatch.setattr(mw, 'fetch_compare_status', lambda tag: 'ahead')

    def _must_not_be_called(*a, **k):
      raise AssertionError('a dry run must not write to GitHub')

    monkeypatch.setattr(mw, 'create_issue', _must_not_be_called)
    monkeypatch.setattr(mw, 'ensure_label', _must_not_be_called)

    rc = mw.main(['--repo', 'owner/repo', '--dry-run'])

    assert rc == 0
    assert 'mapd release: v2.4.0' in capsys.readouterr().out

  def test_missing_repo_is_a_usage_error(self, monkeypatch):
    monkeypatch.delenv('GITHUB_REPOSITORY', raising=False)
    monkeypatch.setattr(mw, 'fetch_releases', lambda: pytest.fail(
        'main() must return before hitting the API'))
    assert mw.main([]) == 2

  def test_files_one_issue_per_release_after_ensuring_the_label(self, monkeypatch):
    calls = []
    monkeypatch.setattr(mw, 'fetch_releases',
                        lambda: [_release('v2.4.0'), _release('v2.5.0')])
    monkeypatch.setattr(mw, 'fetch_issues', lambda repo: [])
    monkeypatch.setattr(mw, 'fetch_upstream_capnp', lambda tag: _V231_CUSTOM_CAPNP)
    monkeypatch.setattr(mw, 'fetch_compare_status', lambda tag: 'ahead')
    monkeypatch.setattr(mw, 'ensure_label', lambda repo: calls.append('label'))
    monkeypatch.setattr(mw, 'create_issue',
                        lambda repo, item: calls.append(item['title']) or 7)

    assert mw.main(['--repo', 'owner/repo']) == 0
    assert calls == ['label', 'mapd release: v2.4.0', 'mapd release: v2.5.0']
