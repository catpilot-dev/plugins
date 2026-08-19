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
