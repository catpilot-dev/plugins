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
