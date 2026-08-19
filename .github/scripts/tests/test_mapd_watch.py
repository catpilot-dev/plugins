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


# mapd v2.3.0's cereal/custom/custom.capnp, trimmed to the parts this watch
# reads: MapdOut, the enums it references, and one unrelated struct that must
# not leak into MapdOut's field list. Verbatim upstream text — the clean case
# needs a real fixture, or "identical" only ever proves the parser agrees with
# itself.
_V230_CUSTOM_CAPNP = """using Go = import "/go.capnp";
@0xb526ba661d550a59;

struct MapdPosition @0xde9705979aca8339 {
  latitude @0 :Float64;
  longitude @1 :Float64;
}

enum WaySelectionType {
  current @0;
  predicted @1;
  possible @2;
  extended @3;
  fail @4;
}

enum RoadContext {
  freeway @0;
  city @1;
  unknown @2;
}

# WARNING: must be kept in perfect sync (names and values) with the
# HighwayClass enum in cereal/offline/offline.capnp
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

_OURS_FIELDS = mw.SLOT19_CAPNP.read_text()
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


class TestParseFields:
  """Two shapes, one parser: mapd wraps MapdOut in a struct block, our
  slot19.capnp is a bare fragment of field lines that custom_capnp.py splices
  into the real struct at install time."""

  def test_parses_the_wrapped_upstream_struct(self):
    fields = mw.parse_fields(_V230_CUSTOM_CAPNP)
    assert len(fields) == 27
    assert sorted(f['ordinal'] for f in fields.values()) == list(range(27))
    assert fields['highwayClass'] == {'ordinal': 24, 'type': 'HighwayClass'}

  def test_parses_our_bare_fragment(self):
    fields = mw.parse_fields(_OURS_FIELDS)
    assert len(fields) == 27
    assert sorted(f['ordinal'] for f in fields.values()) == list(range(27))
    assert fields['conditionalSpeedLimit'] == {'ordinal': 26, 'type': 'Text'}

  def test_does_not_absorb_fields_from_neighbouring_structs(self):
    """MapdPosition's latitude/longitude sit above MapdOut in the same file."""
    assert 'latitude' not in mw.parse_fields(_V230_CUSTOM_CAPNP)

  def test_missing_struct_in_a_file_with_blocks_is_an_error_not_a_scrape(self):
    """A rename or deletion must fail loudly. Falling back to bare-fragment
    mode here would scrape every field line in the file and report a
    plausible-looking diff against a struct that no longer exists."""
    with pytest.raises(mw.SchemaError):
      mw.parse_fields(_V230_CUSTOM_CAPNP.replace('struct MapdOut', 'struct MapdOutV2'))

  def test_ignores_commented_out_field_lines(self):
    text = _OURS_FIELDS + "\n  # futureField @27 :Text;\n"
    assert 'futureField' not in mw.parse_fields(text)


class TestParseEnum:
  def test_reads_members_and_ordinals(self):
    assert mw.parse_enum(_OURS_ENUMS, 'RoadContext') == {
        'freeway': 0, 'city': 1, 'unknown': 2}

  def test_returns_none_for_an_undeclared_enum(self):
    assert mw.parse_enum(_OURS_ENUMS, 'NoSuchEnum') is None

  def test_referenced_enums_are_derived_from_the_field_types(self):
    fields = mw.parse_fields(_V230_CUSTOM_CAPNP)
    assert set(mw.referenced_enums(fields, _V230_CUSTOM_CAPNP)) == {
        'RoadContext', 'WaySelectionType', 'HighwayClass'}


class TestDiffSchema:
  def test_v230_is_identical_to_our_slots(self):
    """The real baseline: MapdOut at v2.3.0 is field-identical to slot19.capnp,
    ordinals 0-26. The counts are asserted too — a differ that parsed nothing
    would otherwise report 'identical' vacuously."""
    diff = mw.diff_schema(_V230_CUSTOM_CAPNP, _OURS_FIELDS, _OURS_ENUMS)
    assert diff['identical']
    assert diff['theirs_field_count'] == 27
    assert diff['ours_field_count'] == 27
    assert diff['added'] == diff['removed'] == diff['changed'] == []

  def test_reports_an_upstream_added_field(self):
    theirs = _V230_CUSTOM_CAPNP.replace(
        '  conditionalSpeedLimit @26 :Text;',
        '  conditionalSpeedLimit @26 :Text;\n  tollRoad @27 :Bool;')
    diff = mw.diff_schema(theirs, _OURS_FIELDS, _OURS_ENUMS)
    assert not diff['identical']
    assert diff['added'] == [('tollRoad', 27, 'Bool')]
    assert diff['theirs_field_count'] == 28

  def test_reports_an_upstream_added_enumerant(self):
    theirs = _V230_CUSTOM_CAPNP.replace(
        '  livingStreet @13;', '  livingStreet @13;\n  service @14;')
    diff = mw.diff_schema(theirs, _OURS_FIELDS, _OURS_ENUMS)
    assert not diff['identical']
    assert diff['added'] == []
    highway = next(e for e in diff['enums'] if e['name'] == 'HighwayClass')
    assert highway['added'] == [('service', 14)]

  def test_reports_a_renumbered_field_as_changed(self):
    theirs = _V230_CUSTOM_CAPNP.replace('  wayId @25 :Int64;',
                                        '  wayId @25 :UInt64;')
    diff = mw.diff_schema(theirs, _OURS_FIELDS, _OURS_ENUMS)
    assert [n for n, _, _ in diff['changed']] == ['wayId']

  def test_reports_an_enum_we_do_not_declare_at_all(self):
    theirs = _V230_CUSTOM_CAPNP.replace(
        '  conditionalSpeedLimit @26 :Text;',
        '  conditionalSpeedLimit @26 :Text;\n  surface @27 :SurfaceType;'
    ).replace('struct MapdOut',
              'enum SurfaceType {\n  paved @0;\n  gravel @1;\n}\n\nstruct MapdOut')
    diff = mw.diff_schema(theirs, _OURS_FIELDS, _OURS_ENUMS)
    surface = next(e for e in diff['enums'] if e['name'] == 'SurfaceType')
    assert surface['missing']

  def test_a_side_that_parsed_to_nothing_is_an_error_not_a_clean_diff(self):
    with pytest.raises(mw.SchemaError):
      mw.diff_schema('', _OURS_FIELDS, _OURS_ENUMS)


class TestSchemaSection:
  def test_clean_diff_says_schema_safe(self):
    section = mw.schema_section('v2.4.0', fetch=lambda tag: _V230_CUSTOM_CAPNP)
    assert 'schema-safe' in section
    assert '27 fields' in section

  def test_added_field_is_listed_by_name_and_ordinal(self):
    theirs = _V230_CUSTOM_CAPNP.replace(
        '  conditionalSpeedLimit @26 :Text;',
        '  conditionalSpeedLimit @26 :Text;\n  tollRoad @27 :Bool;')
    section = mw.schema_section('v2.4.0', fetch=lambda tag: theirs)
    assert '`tollRoad @27 :Bool;`' in section
    assert 'schema-safe' not in section

  def test_unfetchable_file_still_yields_a_section_stating_the_gap(self):
    """The one exception to fail-loudly: a release notification is worth more
    than a perfect issue body, and the missing check is stated, not implied."""
    def _boom(tag):
      raise mw.requests.HTTPError('404')

    section = mw.schema_section('v2.4.0', fetch=_boom)
    assert 'Could not be checked' in section
    assert 'by hand' in section


class TestCommitSection:
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

  def test_empty_required_commit_drops_the_section(self, monkeypatch):
    monkeypatch.setattr(mw, 'REQUIRED_COMMIT', '')
    assert mw.commit_section('v2.4.0', 'behind') == ''

  def test_build_issue_skips_the_compare_call_once_the_gate_is_retired(self, monkeypatch):
    monkeypatch.setattr(mw, 'REQUIRED_COMMIT', '')
    monkeypatch.setattr(mw, 'fetch_compare_status', lambda tag: pytest.fail(
        'compare must not be called with REQUIRED_COMMIT retired'))
    monkeypatch.setattr(mw, 'fetch_upstream_capnp', lambda tag: _V230_CUSTOM_CAPNP)
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
    monkeypatch.setattr(mw, 'fetch_upstream_capnp', lambda tag: _V230_CUSTOM_CAPNP)
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
    monkeypatch.setattr(mw, 'fetch_upstream_capnp', lambda tag: _V230_CUSTOM_CAPNP)
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
    monkeypatch.setattr(mw, 'fetch_upstream_capnp', lambda tag: _V230_CUSTOM_CAPNP)
    monkeypatch.setattr(mw, 'fetch_compare_status', lambda tag: 'ahead')
    monkeypatch.setattr(mw, 'ensure_label', lambda repo: calls.append('label'))
    monkeypatch.setattr(mw, 'create_issue',
                        lambda repo, item: calls.append(item['title']) or 7)

    assert mw.main(['--repo', 'owner/repo']) == 0
    assert calls == ['label', 'mapd release: v2.4.0', 'mapd release: v2.5.0']
