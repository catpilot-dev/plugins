"""Tests for the upstream model watch reconcile logic."""
import importlib.util
import pathlib
import sys

import pytest

_SCRIPT = pathlib.Path(__file__).resolve().parents[1] / 'model_watch.py'
_spec = importlib.util.spec_from_file_location('model_watch', _SCRIPT)
mw = importlib.util.module_from_spec(_spec)
sys.modules['model_watch'] = mw
_spec.loader.exec_module(mw)


def _cand(**over):
  c = {'id': 'nice_model_37727', 'name': 'Nice Model', 'commit': 'a' * 40,
       'date': '2025-12-01', 'pr': '(#37727)', 'type': 'driving',
       'files': ['driving_vision.onnx', 'driving_policy.onnx'],
       'upstream_reverted': None}
  c.update(over)
  return c


def _issue(sha, state='open'):
  return {'number': 1, 'state': state, 'body': f'{mw.MARKER} {sha}\n\nrest'}


class _FakeResponse:
  def __init__(self, payload):
    self._payload = payload

  def raise_for_status(self):
    pass

  def json(self):
    return self._payload


class TestFetchIssues:
  """Regression coverage for the labels-param AND-semantics bug.

  GitHub's issues-list `labels` filter is AND: an issue must carry every
  listed label to match. Our issues each carry exactly one of
  model-candidate / model-revert, never both, so a single query for
  'model-candidate,model-revert' matches nothing. fetch_issues must query
  per label and merge, or every already-reported model gets re-filed daily.
  """

  def test_returns_issues_for_either_label_not_only_both(self, monkeypatch):
    candidate_issue = {'number': 1, 'state': 'open', 'body': 'candidate body'}
    revert_issue = {'number': 2, 'state': 'closed', 'body': 'revert body'}
    calls = []

    def fake_get(url, params=None, headers=None, timeout=None):
      calls.append(dict(params))
      if params['page'] > 1:
        return _FakeResponse([])
      if params['labels'] == 'model-candidate':
        return _FakeResponse([candidate_issue])
      if params['labels'] == 'model-revert':
        return _FakeResponse([revert_issue])
      # Simulates real GitHub AND semantics: a combined-label query matches
      # nothing here, since no issue in this fixture carries both labels.
      return _FakeResponse([])

    monkeypatch.setattr(mw.requests, 'get', fake_get)
    issues = mw.fetch_issues('owner/repo')

    assert {i['number'] for i in issues} == {1, 2}
    # Queried per label, not one combined AND query.
    assert any(c['labels'] == 'model-candidate' for c in calls)
    assert any(c['labels'] == 'model-revert' for c in calls)
    assert not any(c['labels'] == 'model-candidate,model-revert' for c in calls)

  def test_paginates_per_label_and_terminates_without_dropping_a_page(self, monkeypatch):
    pages_seen = []

    def fake_get(url, params=None, headers=None, timeout=None):
      pages_seen.append((params['labels'], params['page']))
      if params['labels'] == 'model-candidate' and params['page'] == 1:
        return _FakeResponse([{'number': 10, 'state': 'open', 'body': ''}])
      if params['labels'] == 'model-revert' and params['page'] == 1:
        return _FakeResponse([{'number': 20, 'state': 'open', 'body': ''}])
      return _FakeResponse([])

    monkeypatch.setattr(mw.requests, 'get', fake_get)
    issues = mw.fetch_issues('owner/repo')

    assert {i['number'] for i in issues} == {10, 20}
    # Each label's pagination fetched page 1, then terminated on the empty
    # page 2 — no infinite loop, and page 1 was not skipped.
    assert pages_seen.count(('model-candidate', 1)) == 1
    assert pages_seen.count(('model-candidate', 2)) == 1
    assert pages_seen.count(('model-revert', 1)) == 1
    assert pages_seen.count(('model-revert', 2)) == 1

  def test_dedups_an_issue_that_somehow_carries_both_labels(self, monkeypatch):
    both_labels_issue = {'number': 5, 'state': 'open', 'body': 'both'}

    def fake_get(url, params=None, headers=None, timeout=None):
      if params['page'] > 1:
        return _FakeResponse([])
      return _FakeResponse([both_labels_issue])

    monkeypatch.setattr(mw.requests, 'get', fake_get)
    issues = mw.fetch_issues('owner/repo')

    assert [i['number'] for i in issues] == [5]


class TestReportedShas:
  def test_extracts_marker_from_bodies(self):
    assert mw.reported_shas([_issue('a' * 40), _issue('b' * 40)]) == {'a' * 40, 'b' * 40}

  def test_closed_issues_still_count_as_reported(self):
    """Closed means handled — catalogued or rejected. Never re-report it."""
    assert mw.reported_shas([_issue('a' * 40, state='closed')]) == {'a' * 40}

  def test_ignores_bodies_without_the_marker(self):
    assert mw.reported_shas([{'number': 2, 'state': 'open', 'body': 'unrelated'}]) == set()

  def test_tolerates_missing_body(self):
    assert mw.reported_shas([{'number': 3, 'state': 'open', 'body': None}]) == set()


class TestPlanIssues:
  def _catalog(self, ids=()):
    return {'driving': [{'id': i, 'commit': 'c' * 40} for i in ids], 'dm': []}

  def test_files_a_new_candidate(self):
    scan = {'candidates': [_cand()], 'reverted': {}}
    planned = mw.plan_issues(scan, self._catalog(), set())
    assert [p['kind'] for p in planned] == ['candidate']
    assert planned[0]['sha'] == 'a' * 40
    assert 'model-candidate' in planned[0]['labels']
    assert f"{mw.MARKER} {'a' * 40}" in planned[0]['body']

  def test_skips_already_reported(self):
    scan = {'candidates': [_cand()], 'reverted': {}}
    assert mw.plan_issues(scan, self._catalog(), {'a' * 40}) == []

  def test_skips_models_already_catalogued(self):
    scan = {'candidates': [_cand()], 'reverted': {}}
    assert mw.plan_issues(scan, self._catalog(ids=['nice_model_37727']), set()) == []

  def test_never_reported_and_reverted_yields_one_candidate_issue_only(self):
    """Published and reverted between two runs: one issue, not two."""
    scan = {'candidates': [_cand(upstream_reverted='d' * 40)], 'reverted': {'a' * 40: 'd' * 40}}
    planned = mw.plan_issues(scan, self._catalog(), set())
    assert [p['kind'] for p in planned] == ['candidate']
    assert 'reverted' in planned[0]['body'].lower()

  def test_revert_of_an_already_reported_model_files_a_revert_issue(self):
    scan = {'candidates': [_cand(upstream_reverted='d' * 40)], 'reverted': {'a' * 40: 'd' * 40}}
    planned = mw.plan_issues(scan, self._catalog(), {'a' * 40})
    assert [p['kind'] for p in planned] == ['revert']
    assert 'model-revert' in planned[0]['labels']

  def test_revert_issue_is_deduped_by_its_own_marker(self):
    scan = {'candidates': [_cand(upstream_reverted='d' * 40)], 'reverted': {'a' * 40: 'd' * 40}}
    planned = mw.plan_issues(scan, self._catalog(), {'a' * 40, 'd' * 40})
    assert planned == []

  def test_revert_of_a_catalogued_model_is_flagged_in_the_title(self):
    cat = {'driving': [{'id': 'nice_model_37727', 'commit': 'a' * 40}], 'dm': []}
    scan = {'candidates': [_cand(upstream_reverted='d' * 40)], 'reverted': {'a' * 40: 'd' * 40}}
    planned = mw.plan_issues(scan, cat, {'a' * 40})
    assert planned[0]['kind'] == 'revert'
    assert 'catalog' in planned[0]['title'].lower()

  def test_candidate_body_carries_a_paste_ready_entry_without_verified_on(self):
    planned = mw.plan_issues({'candidates': [_cand()], 'reverted': {}}, self._catalog(), set())
    body = planned[0]['body']
    assert '"id": "nice_model_37727"' in body
    assert '"commit": "' + 'a' * 40 + '"' in body
    assert 'verified_on' not in body.split('```')[1]

  def test_reverted_candidate_is_never_reported_twice_across_two_runs(self):
    """Finding 1 regression: a model published AND reverted between two daily
    runs must get exactly one issue, ever — not a candidate issue on day 1
    followed by a spec-forbidden model-revert issue on day 2.

    Day 1: never-reported reverted candidate -> one candidate issue, whose
    body must carry the revert sha as a second marker line (this is the ONLY
    issue this revert will ever get). Day 2: feed that body's markers back in
    as `reported` against the identical scan -> nothing left to file.
    """
    scan = {'candidates': [_cand(upstream_reverted='d' * 40)], 'reverted': {'a' * 40: 'd' * 40}}

    day1 = mw.plan_issues(scan, self._catalog(), set())
    assert [p['kind'] for p in day1] == ['candidate']

    day1_issue = {'number': 1, 'state': 'open', 'body': day1[0]['body']}
    reported_after_day1 = mw.reported_shas([day1_issue])
    assert reported_after_day1 == {'a' * 40, 'd' * 40}

    day2 = mw.plan_issues(scan, self._catalog(), reported_after_day1)
    assert day2 == []


class TestMainCatalogFailClosed:
  """Finding 2: catalog.load_catalog() fails OPEN by design ({} on any read
  or parse error) because that's correct for the car's UI thread. In CI that
  same {} makes every catalogued model look brand new to _catalog_ids and
  spam a fresh candidate issue — main() must treat an empty catalog as a
  hard error instead of planning against it."""

  def test_empty_catalog_returns_1_and_creates_nothing(self, monkeypatch, capsys):
    monkeypatch.setattr(mw, 'fetch_commits', lambda: [])
    monkeypatch.setattr(mw.catalog, 'load_catalog', lambda: {})

    def _must_not_be_called(*a, **k):
      raise AssertionError('main() must return before touching issues/creating anything')

    monkeypatch.setattr(mw, 'fetch_issues', _must_not_be_called)
    monkeypatch.setattr(mw, 'create_issue', _must_not_be_called)

    rc = mw.main(['--repo', 'owner/repo'])

    assert rc == 1
    assert 'catalog' in capsys.readouterr().err.lower()


class TestWatchFiles:
  """The watcher must follow the ONNX files THIS fork actually loads.

  Upstream has since replaced driving_vision/driving_policy with a single
  driving_supercombo.onnx. Watching the directory reported every refactor and
  every supercombo-era model as a candidate; watching the files does not.
  """

  def test_watch_files_match_the_forks_expected_onnx(self):
    """Drift guard: if the fork's model file set changes, this must fail loudly
    rather than leave the watcher silently following the wrong paths."""
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / 'plugins' / 'model_selector'))
    from model_swapper import ModelSwapper, ModelType
    assert mw.WATCH_FILES['driving'] == ModelSwapper.MODEL_CONFIGS[ModelType.DRIVING]['onnx_files']
    assert mw.WATCH_FILES['dm'] == ModelSwapper.MODEL_CONFIGS[ModelType.DM]['onnx_files']


class TestFetchCommitsPerFile:
  def _capture(self, monkeypatch, payload_for):
    seen = []

    def fake_get(url, params=None, headers=None, timeout=None):
      seen.append(params['path'])
      return _FakeResponse(payload_for(params['path']))

    monkeypatch.setattr(mw.requests, 'get', fake_get)
    return seen

  def test_queries_each_watched_onnx_file_not_the_directory(self, monkeypatch):
    seen = self._capture(monkeypatch, lambda path: [])
    mw.fetch_commits()
    assert seen == [
      'selfdrive/modeld/models/driving_vision.onnx',
      'selfdrive/modeld/models/driving_policy.onnx',
      'selfdrive/modeld/models/dmonitoring_model.onnx',
    ]

  def test_dedups_a_commit_touching_two_watched_files(self, monkeypatch):
    commit = {'sha': 'a' * 40, 'commit': {'message': 'Nice Model (#1)',
                                          'committer': {'date': '2025-12-01T00:00:00Z'}}}
    self._capture(monkeypatch, lambda path: [commit] if 'driving_' in path else [])
    commits = mw.fetch_commits()
    assert [c['sha'] for c in commits] == ['a' * 40]

  def test_annotates_driving_type_from_the_file_touched(self, monkeypatch):
    commit = {'sha': 'a' * 40, 'commit': {'message': 'Nice Model (#1)',
                                          'committer': {'date': '2025-12-01T00:00:00Z'}}}
    self._capture(monkeypatch, lambda path: [commit] if 'driving_vision' in path else [])
    assert mw.fetch_commits()[0]['_watch_type'] == 'driving'

  def test_annotates_dm_type_from_the_file_touched(self, monkeypatch):
    commit = {'sha': 'b' * 40, 'commit': {'message': 'Sharp Eyes (#2)',
                                          'committer': {'date': '2025-12-01T00:00:00Z'}}}
    self._capture(monkeypatch, lambda path: [commit] if 'dmonitoring' in path else [])
    assert mw.fetch_commits()[0]['_watch_type'] == 'dm'


class TestApplyWatchTypes:
  """Type comes from the file that changed, not from guessing at the message."""

  def test_file_evidence_overrides_the_message_heuristic(self):
    scan = {'candidates': [_cand(type='dm', files=['dmonitoring_model.onnx'])], 'reverted': {}}
    mw.apply_watch_types(scan, {'a' * 40: 'driving'})
    cand = scan['candidates'][0]
    assert cand['type'] == 'driving'
    assert cand['files'] == ['driving_vision.onnx', 'driving_policy.onnx']

  def test_leaves_a_candidate_alone_when_no_evidence(self):
    scan = {'candidates': [_cand()], 'reverted': {}}
    mw.apply_watch_types(scan, {})
    assert scan['candidates'][0]['type'] == 'driving'


class TestCatalogMatchingBySha:
  """Catalog entry ids may be keyed on a PR number OR a short sha; the commit
  sha is the only unambiguous key. Matching by id alone re-filed a model that
  was already catalogued."""

  def test_catalogued_commit_is_not_refiled_even_when_the_id_differs(self):
    cat = {'driving': [{'id': 'le_mans_gt3_model_04dcdf4', 'commit': 'c' * 40}], 'dm': []}
    scan = {'candidates': [_cand(id='le_mans_gt3_model_37425', commit='c' * 40)], 'reverted': {}}
    assert mw.plan_issues(scan, cat, set()) == []

  def test_id_match_still_suppresses_when_the_entry_has_no_commit(self):
    cat = {'driving': [{'id': 'stock_0.11.1', 'source': 'shipped'}], 'dm': []}
    scan = {'candidates': [_cand(id='stock_0.11.1')], 'reverted': {}}
    assert mw.plan_issues(scan, cat, set()) == []


class TestFileStatusFilter:
  """Touching a path is not changing a model.

  A commit that REMOVES driving_vision.onnx (upstream's move to a single
  supercombo model) or RENAMES files under the models dir still shows up in a
  path-filtered commit query. Only an add or a modify of a file this fork loads
  is a model this fork could actually install.
  """

  def _files(self, monkeypatch, by_sha):
    def fake_get(url, params=None, headers=None, timeout=None):
      sha = url.rstrip('/').split('/')[-1]
      return _FakeResponse({'files': [
        {'filename': f'selfdrive/modeld/models/{name}', 'status': status}
        for name, status in by_sha.get(sha, [])
      ]})
    monkeypatch.setattr(mw.requests, 'get', fake_get)

  def test_keeps_a_commit_that_modifies_a_watched_file(self, monkeypatch):
    self._files(monkeypatch, {'a' * 40: [('driving_vision.onnx', 'modified'),
                                         ('driving_policy.onnx', 'modified')]})
    assert [c['id'] for c in mw.filter_by_file_status([_cand()])] == ['nice_model_37727']

  def test_drops_a_commit_that_only_removes_a_watched_file(self, monkeypatch):
    self._files(monkeypatch, {'a' * 40: [('driving_vision.onnx', 'removed'),
                                         ('driving_supercombo.onnx', 'added')]})
    assert mw.filter_by_file_status([_cand()]) == []

  def test_drops_a_commit_that_only_renames(self, monkeypatch):
    self._files(monkeypatch, {'a' * 40: [('driving_vision.onnx', 'renamed')]})
    assert mw.filter_by_file_status([_cand()]) == []

  def test_drops_a_commit_with_no_watched_file_change(self, monkeypatch):
    self._files(monkeypatch, {'a' * 40: [('driving_supercombo.onnx', 'modified')]})
    assert mw.filter_by_file_status([_cand()]) == []

  def test_judges_a_dm_candidate_on_the_dm_file(self, monkeypatch):
    self._files(monkeypatch, {'b' * 40: [('dmonitoring_model.onnx', 'modified')]})
    dm = _cand(commit='b' * 40, type='dm', files=['dmonitoring_model.onnx'])
    assert [c['id'] for c in mw.filter_by_file_status([dm])] == ['nice_model_37727']

  def test_a_dm_candidate_is_not_kept_by_a_driving_file_change(self, monkeypatch):
    self._files(monkeypatch, {'b' * 40: [('driving_vision.onnx', 'modified')]})
    dm = _cand(commit='b' * 40, type='dm', files=['dmonitoring_model.onnx'])
    assert mw.filter_by_file_status([dm]) == []


class TestDropNonModelCandidates:
  """The status check costs one API call per commit, so it must run on the
  handful of issues about to be FILED — not on every candidate in the window."""

  def _files(self, monkeypatch, by_sha, calls):
    def fake_get(url, params=None, headers=None, timeout=None):
      sha = url.rstrip('/').split('/')[-1]
      calls.append(sha)
      return _FakeResponse({'files': [
        {'filename': f'selfdrive/modeld/models/{name}', 'status': status}
        for name, status in by_sha.get(sha, [])
      ]})
    monkeypatch.setattr(mw.requests, 'get', fake_get)

  def test_checks_only_the_candidates_about_to_be_filed(self, monkeypatch):
    calls = []
    self._files(monkeypatch, {'a' * 40: [('driving_vision.onnx', 'modified')]}, calls)
    planned = [{'kind': 'candidate', 'sha': 'a' * 40}]
    candidates = [_cand(), _cand(commit='z' * 40, id='other_1')]
    mw.drop_non_model_candidates(planned, candidates)
    assert calls == ['a' * 40], "must not spend a call on the unplanned candidate"

  def test_drops_a_planned_candidate_whose_file_was_only_removed(self, monkeypatch):
    self._files(monkeypatch, {'a' * 40: [('driving_vision.onnx', 'removed')]}, [])
    planned = [{'kind': 'candidate', 'sha': 'a' * 40}]
    assert mw.drop_non_model_candidates(planned, [_cand()]) == []

  def test_keeps_revert_issues_without_checking_them(self, monkeypatch):
    calls = []
    self._files(monkeypatch, {}, calls)
    planned = [{'kind': 'revert', 'sha': 'd' * 40}]
    assert mw.drop_non_model_candidates(planned, [_cand()]) == planned
    assert calls == []
