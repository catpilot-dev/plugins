"""Regression tests for model_download.py PR ingestion.

Covers add_model_from_pr's dedup path, which referenced an undefined
`registry` name (a latent NameError) until it was loaded explicitly.
"""
import importlib
from unittest.mock import patch, MagicMock

import pytest

# model_download imports `requests` at module scope; auto-skip where the
# dependency isn't installed (same convention as the openpilot/opendbc-gated
# tests in this repo).
try:
  md = importlib.import_module('plugins.model_selector.model_download')
except ImportError:
  pytest.skip('requests not available', allow_module_level=True)


def _fake_pr(**over):
  pr = {
    'title': 'New driving model',
    'merge_commit_sha': 'a' * 40,
    'merged_at': '2025-12-01T00:00:00Z',
    'head': {'sha': 'b' * 40},
  }
  pr.update(over)
  return pr


def test_add_from_pr_reaches_registry_without_nameerror():
  # The dedup loop over the registry must not raise NameError when the
  # registry has no matching PR entry.
  resp = MagicMock()
  resp.json.return_value = _fake_pr()
  resp.raise_for_status.return_value = None
  with patch.object(md.requests, 'get', return_value=resp), \
       patch.object(md, 'load_registry', return_value=({}, {})) as load_reg, \
       patch.object(md, 'add_model_to_registry', return_value=0) as add_reg:
    rc = md.add_model_from_pr(36849, model_type='driving')
  assert rc == 0
  load_reg.assert_called_once()
  # brand-new PR -> id derived from title + PR number
  assert add_reg.call_args.kwargs['model_id'] == 'new_driving_model_36849'
  assert add_reg.call_args.kwargs['commit'] == 'b' * 40  # head sha, not merge


def test_add_from_pr_dedups_to_existing_entry():
  # An existing registry entry from the same PR is reused as the model_id.
  resp = MagicMock()
  resp.json.return_value = _fake_pr()
  resp.raise_for_status.return_value = None
  existing = {'old_name_36849': {'pr': '#36849'}}
  with patch.object(md.requests, 'get', return_value=resp), \
       patch.object(md, 'load_registry', return_value=(existing, {})), \
       patch.object(md, 'add_model_to_registry', return_value=0) as add_reg:
    md.add_model_from_pr(36849, model_type='driving')
  assert add_reg.call_args.kwargs['model_id'] == 'old_name_36849'


def test_add_from_pr_dm_type_checks_dm_registry():
  # For a dm model the dedup must scan the dm sub-registry, not driving.
  resp = MagicMock()
  resp.json.return_value = _fake_pr()
  resp.raise_for_status.return_value = None
  driving = {'driving_36849': {'pr': '#36849'}}   # must be ignored for dm
  dm = {'dm_36849': {'pr': '#36849'}}
  with patch.object(md.requests, 'get', return_value=resp), \
       patch.object(md, 'load_registry', return_value=(driving, dm)), \
       patch.object(md, 'add_model_to_registry', return_value=0) as add_reg:
    md.add_model_from_pr(36849, model_type='dm')
  assert add_reg.call_args.kwargs['model_id'] == 'dm_36849'


def test_add_from_pr_unmerged_returns_error():
  resp = MagicMock()
  resp.json.return_value = _fake_pr(merge_commit_sha=None)
  resp.raise_for_status.return_value = None
  with patch.object(md.requests, 'get', return_value=resp), \
       patch.object(md, 'add_model_to_registry') as add_reg:
    rc = md.add_model_from_pr(36849)
  assert rc == 1
  add_reg.assert_not_called()


import json


@pytest.fixture
def dl_env(tmp_path, monkeypatch):
  """Point catalog + install dirs at tmp_path for the download/check paths."""
  import plugins.model_selector.catalog as cat
  version_h = tmp_path / 'version.h'
  version_h.write_text('#define COMMA_VERSION "0.11.1"\n')
  monkeypatch.setattr(cat, 'VERSION_H', version_h)
  monkeypatch.setattr(cat, 'CATALOG_FILE', tmp_path / 'catalog.json')
  monkeypatch.setattr(cat, 'UNLOCK_MARKER', tmp_path / '.unlocked')
  (tmp_path / 'catalog.json').write_text(json.dumps({
    'driving': [{'id': 'good_model', 'name': 'Good', 'date': '2025-10-20',
                 'commit': 'c' * 40,
                 'files': ['driving_vision.onnx', 'driving_policy.onnx'],
                 'verified_on': ['0.11.1']},
                {'id': 'stock_0.11.1', 'name': 'Release default', 'date': '2026-05-18',
                 'source': 'shipped', 'verified_on': ['0.11.1'], 'baseline_for': ['0.11.1']}],
    'dm': [],
  }))
  (tmp_path / 'models' / 'driving').mkdir(parents=True)
  (tmp_path / 'models' / 'dm').mkdir(parents=True)
  monkeypatch.setattr(md, 'BASE_DATA_DIR', tmp_path)
  return tmp_path, cat


class TestCheckUpdates:
  def test_offers_verified_uninstalled(self, dl_env, capsys):
    md.check_updates()
    out = json.loads(capsys.readouterr().out)
    # 'stock_0.11.1' is a shipped entry: download_model always refuses it (no
    # commit/files, it's imported from disk by import_stock, not downloaded),
    # so it must never be offered even though it is verified and uninstalled.
    assert [m['id'] for m in out['driving']] == ['good_model']
    assert out['total'] == 1
    assert out['version'] == '0.11.1'
    # verified_total still counts the shipped entry: it answers "does this
    # openpilot version have any tested models at all".
    assert out['verified_total'] == 2

  def test_shipped_entry_never_offered_even_when_never_installed(self, dl_env, capsys):
    # A device that has swapped models away from stock never has the shipped
    # entry's directory under models/driving/ (import_stock only runs while
    # no swap has ever happened) — it must not show up as "available" forever.
    md.check_updates()
    out = json.loads(capsys.readouterr().out)
    assert 'stock_0.11.1' not in {m['id'] for m in out['driving']}

  def test_skips_installed(self, dl_env, capsys):
    tmp_path, _ = dl_env
    (tmp_path / 'models' / 'driving' / 'good_model').mkdir()
    md.check_updates()
    out = json.loads(capsys.readouterr().out)
    # good_model is now installed and stock_0.11.1 is shipped (never offered):
    # nothing left to offer, but verified_total still counts both.
    assert out['driving'] == []
    assert out['verified_total'] == 2

  def test_entries_carry_type(self, dl_env, capsys):
    md.check_updates()
    out = json.loads(capsys.readouterr().out)
    assert all(m['type'] == 'driving' for m in out['driving'])

  def test_unknown_version_offers_nothing(self, dl_env, capsys):
    tmp_path, cat = dl_env
    cat.VERSION_H.write_text('#define COMMA_VERSION "9.9.9"\n')
    md.check_updates()
    out = json.loads(capsys.readouterr().out)
    assert out['total'] == 0
    assert out['verified_total'] == 0


class TestDownloadGate:
  def test_refuses_uncatalogued(self, dl_env, capsys):
    assert md.download_model(md.ModelType.DRIVING, 'mystery_model') == 1
    assert 'not a tested model' in capsys.readouterr().out

  def test_refuses_shipped_entry(self, dl_env, capsys):
    assert md.download_model(md.ModelType.DRIVING, 'stock_0.11.1') == 1
    assert 'imported from disk' in capsys.readouterr().out

  def test_allow_untested_bypasses_the_gate(self, dl_env, capsys):
    # Gate passes, then the registry lookup fails — proving the gate was not
    # what stopped it.
    assert md.download_model(md.ModelType.DRIVING, 'mystery_model', allow_untested=True) == 1
    assert 'not a tested model' not in capsys.readouterr().out

  def test_unlock_marker_bypasses_the_gate(self, dl_env, capsys):
    _, cat = dl_env
    cat.UNLOCK_MARKER.write_text('')
    assert md.download_model(md.ModelType.DRIVING, 'mystery_model') == 1
    assert 'not a tested model' not in capsys.readouterr().out

  def test_verified_model_downloads_from_catalog_metadata(self, dl_env, monkeypatch, capsys):
    calls = []
    monkeypatch.setattr(md, 'download_file', lambda url, dest, desc=None: calls.append(url))
    assert md.download_model(md.ModelType.DRIVING, 'good_model') == 0
    assert len(calls) == 2
    assert all(('c' * 40) in url for url in calls)


class TestCompatibilityHeuristicSuperseded:
  """Curation strictly supersedes the pre-desire_pulse date heuristic: a
  catalogued entry is maintainer-test-driven and must not be vetoed by the
  date rule. The registry (non-catalogued, maintainer-add-a-model) path keeps
  running the heuristic since nothing has verified those entries."""

  def test_catalogued_entry_predating_the_cutoff_still_downloads(self, dl_env, monkeypatch, capsys):
    tmp_path, cat = dl_env
    data = json.loads(cat.CATALOG_FILE.read_text())
    data['driving'].append({
      'id': 'old_but_catalogued', 'name': 'Old But Catalogued', 'date': '2025-01-01',
      'commit': 'd' * 40, 'files': ['driving_vision.onnx', 'driving_policy.onnx'],
      'verified_on': ['0.11.1'],
    })
    cat.CATALOG_FILE.write_text(json.dumps(data))

    calls = []
    monkeypatch.setattr(md, 'download_file', lambda url, dest, desc=None: calls.append(url))
    assert md.download_model(md.ModelType.DRIVING, 'old_but_catalogued') == 0
    assert len(calls) == 2
    assert all(('d' * 40) in url for url in calls)

  def test_registry_path_still_blocks_pre_desire_pulse_noninteractive(self, dl_env, monkeypatch, capsys):
    import types
    _, cat = dl_env
    # Bypass the catalog gate (this id is not in the catalog at all) so the
    # registry lookup is reached — the existing maintainer escape hatch.
    cat.UNLOCK_MARKER.write_text('')
    monkeypatch.setattr(md, 'load_registry', lambda: (
      {'old_registry_model': {
        'name': 'Old Registry Model', 'commit': 'e' * 40, 'date': '2025-01-01',
        'files': ['driving_vision.onnx', 'driving_policy.onnx'],
      }}, {}))
    monkeypatch.setattr(md.sys, 'stdin', types.SimpleNamespace(isatty=lambda: False))
    assert md.download_model(md.ModelType.DRIVING, 'old_registry_model') == 1
    assert 'Skipping incompatible model' in capsys.readouterr().out

  def test_registry_path_compatible_model_still_downloads(self, dl_env, monkeypatch, capsys):
    _, cat = dl_env
    cat.UNLOCK_MARKER.write_text('')
    monkeypatch.setattr(md, 'load_registry', lambda: (
      {'new_registry_model': {
        'name': 'New Registry Model', 'commit': 'f' * 40, 'date': '2025-10-20',
        'files': ['driving_vision.onnx', 'driving_policy.onnx'],
      }}, {}))
    calls = []
    monkeypatch.setattr(md, 'download_file', lambda url, dest, desc=None: calls.append(url))
    assert md.download_model(md.ModelType.DRIVING, 'new_registry_model') == 0
    assert len(calls) == 2


def _commit(sha, message, date='2025-12-01T00:00:00Z'):
  return {'sha': sha, 'commit': {'message': message, 'committer': {'date': date}}}


class TestScanUpstreamModels:
  def test_parses_a_standard_model_commit(self):
    out = md.scan_upstream_models([_commit('a' * 40, 'Nice Model (#37727)')])
    assert len(out['candidates']) == 1
    c = out['candidates'][0]
    assert c['id'] == 'nice_model_37727'
    assert c['name'] == 'Nice Model'
    assert c['commit'] == 'a' * 40
    assert c['pr'] == '(#37727)'
    assert c['type'] == 'driving'
    assert c['files'] == ['driving_vision.onnx', 'driving_policy.onnx']
    assert c['upstream_reverted'] is None

  def test_detects_dm_models(self):
    out = md.scan_upstream_models([_commit('b' * 40, 'DM: Sharp Eyes (#37800)')])
    c = out['candidates'][0]
    assert c['type'] == 'dm'
    assert c['files'] == ['dmonitoring_model.onnx']
    assert c['name'] == 'Sharp Eyes'

  def test_skips_commits_before_the_firehose_floor(self):
    out = md.scan_upstream_models([_commit('c' * 40, 'Old Model (#100)', '2025-01-01T00:00:00Z')])
    assert out['candidates'] == []

  def test_revert_commit_itself_is_never_a_candidate(self):
    revert = _commit('d' * 40, f"Revert \"Nice Model (#37727)\"\n\nThis reverts commit {'a' * 40}.")
    out = md.scan_upstream_models([revert])
    assert out['candidates'] == []
    assert out['reverted'] == {'a' * 40: 'd' * 40}

  def test_reverted_model_stays_a_candidate_and_is_marked(self):
    """A revert is not a road test — the model must remain drivable and catalogable."""
    revert = _commit('d' * 40, f"Revert \"Nice Model (#37727)\"\n\nThis reverts commit {'a' * 40}.")
    model = _commit('a' * 40, 'Nice Model (#37727)')
    out = md.scan_upstream_models([revert, model])
    ids = [c['id'] for c in out['candidates']]
    assert ids == ['nice_model_37727']
    assert out['candidates'][0]['upstream_reverted'] == 'd' * 40

  def test_pr_fallback_when_message_has_no_pr_number(self, monkeypatch):
    class _Resp:
      def raise_for_status(self): pass
      def json(self): return [{'number': 12345, 'title': 'Fallback Model'}]
    monkeypatch.setattr(md.requests, 'get', lambda *a, **k: _Resp())
    out = md.scan_upstream_models([_commit('e' * 40, 'some model bump')])
    c = out['candidates'][0]
    assert c['pr'] == '#12345'
    assert c['name'] == 'Fallback Model'
    assert c['id'] == 'fallback_model_12345'

  def test_pr_fallback_failure_drops_the_commit(self, monkeypatch):
    def _boom(*a, **k):
      raise RuntimeError('network down')
    monkeypatch.setattr(md.requests, 'get', _boom)
    assert md.scan_upstream_models([_commit('f' * 40, 'unlabelled bump')])['candidates'] == []

  def test_id_matches_the_catalog_format(self):
    """Catalog ids like pop_model_37727 come from this function — they must agree."""
    out = md.scan_upstream_models([_commit('0' * 40, 'POP model (#37727)')])
    assert out['candidates'][0]['id'] == 'pop_model_37727'

  def test_skip_commits_drops_the_candidate_without_a_pr_fallback_call(self, monkeypatch):
    """Finding 3 regression: a commit already in the registry would be
    discarded by update_registry_from_github anyway, so it must never reach
    the unauthenticated, 60/hr-shared-limit PR-fallback lookup. skip_commits
    must short-circuit BEFORE that network call, not just filter results
    after the fact."""
    def _boom(*a, **k):
      raise AssertionError('PR-fallback network call must not happen for a skipped commit')
    monkeypatch.setattr(md.requests, 'get', _boom)

    out = md.scan_upstream_models(
      [_commit('e' * 40, 'unlabelled bump')],  # no '(#' -> would hit PR fallback
      skip_commits={'e' * 40},
    )
    assert out['candidates'] == []

  def test_skip_commits_default_none_preserves_existing_behavior(self, monkeypatch):
    """Default (no skip_commits) must behave exactly as before: the
    PR-fallback call still happens for a commit without a `(#NNNNN)`."""
    calls = []

    class _Resp:
      def raise_for_status(self): pass
      def json(self): return [{'number': 999, 'title': 'Still Called'}]

    def _get(*a, **k):
      calls.append(1)
      return _Resp()

    monkeypatch.setattr(md.requests, 'get', _get)
    out = md.scan_upstream_models([_commit('e' * 40, 'some model bump')])
    assert len(calls) == 1
    assert out['candidates'][0]['name'] == 'Still Called'


class TestUpdateRegistryFromGithub:
  def test_skips_pr_fallback_for_a_commit_already_in_the_registry(self, tmp_path, monkeypatch):
    """Finding 3 wiring: update_registry_from_github must compute
    existing_commits BEFORE calling scan_upstream_models and pass it as
    skip_commits, so a commit already in the registry never reaches the
    PR-fallback network call — it would be discarded on the way out anyway."""
    registry_file = tmp_path / 'model_registry.json'
    registry_file.write_text(json.dumps({
      'driving_models': {'old_model_1': {'commit': 'e' * 40, 'pr': '#1',
                                          'files': [], 'name': 'Old', 'date': '2025-12-01'}},
      'dm_models': {},
    }))
    monkeypatch.setattr(md, 'REGISTRY_FILE', registry_file)

    commits_resp = MagicMock()
    commits_resp.raise_for_status.return_value = None
    # 'e' * 40 has no '(#NNNNN)' -> would hit the PR-fallback call unless
    # skip_commits filters it out first.
    commits_resp.json.return_value = [_commit('e' * 40, 'unlabelled bump')]

    calls = []

    def _get(url, **kwargs):
      calls.append(url)
      if 'commits' in url and 'pulls' not in url:
        return commits_resp
      raise AssertionError(f'unexpected/PR-fallback request: {url}')

    monkeypatch.setattr(md.requests, 'get', _get)
    rc = md.update_registry_from_github()

    assert rc in (0, None)
    # Only the initial commits-list request happened — no PR-fallback lookup.
    assert len(calls) == 1
