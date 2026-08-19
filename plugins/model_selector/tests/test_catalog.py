"""Tests for the curated model catalog — the gate on install and activation."""
import importlib
import json

import pytest


@pytest.fixture
def cat():
  import plugins.model_selector.catalog as mod
  importlib.reload(mod)
  return mod


@pytest.fixture
def catalog_env(cat, tmp_path, monkeypatch):
  """Point the module at a temp catalog, version.h and unlock marker."""
  version_h = tmp_path / 'version.h'
  version_h.write_text('#define COMMA_VERSION "0.11.1"\n')
  monkeypatch.setattr(cat, 'VERSION_H', version_h)
  monkeypatch.setattr(cat, 'CATALOG_FILE', tmp_path / 'compatible_models.json')
  monkeypatch.setattr(cat, 'UNLOCK_MARKER', tmp_path / '.unlocked')
  return cat


def _write(cat, data):
  cat.CATALOG_FILE.write_text(json.dumps(data))


def _entry(**over):
  e = {
    'id': 'cool_people_3c957c6',
    'name': 'Cool People',
    'date': '2025-10-20',
    'commit': 'c' * 40,
    'files': ['driving_vision.onnx', 'driving_policy.onnx'],
    'verified_on': ['0.11.1'],
  }
  e.update(over)
  return e


def _stock(**over):
  e = {
    'id': 'stock_0.11.1',
    'name': 'Release default',
    'date': '2026-05-18',
    'source': 'shipped',
    'verified_on': ['0.11.1'],
    'baseline_for': ['0.11.1'],
  }
  e.update(over)
  return e


class TestOpenpilotVersion:
  def test_parses_version_h(self, catalog_env):
    assert catalog_env.openpilot_version() == '0.11.1'

  def test_missing_version_h_falls_back_to_empty(self, catalog_env, tmp_path, monkeypatch):
    monkeypatch.setattr(catalog_env, 'VERSION_H', tmp_path / 'nope.h')
    # The openpilot package is not importable from the plugins repo, so the
    # manifest fallback fails and the chain ends at ''.
    assert catalog_env.openpilot_version() == ''

  def test_malformed_version_h_does_not_raise(self, catalog_env):
    catalog_env.VERSION_H.write_text('garbage with no quotes\n')
    assert catalog_env.openpilot_version() == ''


class TestLoadCatalog:
  def test_missing_file_fails_closed(self, catalog_env):
    assert catalog_env.load_catalog() == {}
    assert catalog_env.verified_entries('driving') == []

  def test_corrupt_json_fails_closed(self, catalog_env):
    catalog_env.CATALOG_FILE.write_text('{ this is not json')
    assert catalog_env.load_catalog() == {}
    assert catalog_env.verified_entries('driving') == []

  def test_entries_without_id_are_dropped(self, catalog_env):
    _write(catalog_env, {'driving': [_entry(), {'name': 'no id'}], 'dm': []})
    assert len(catalog_env.load_catalog()['driving']) == 1


class TestVerifiedEntries:
  def test_matching_version_is_returned(self, catalog_env):
    _write(catalog_env, {'driving': [_entry()], 'dm': []})
    assert [e['id'] for e in catalog_env.verified_entries('driving')] == ['cool_people_3c957c6']

  def test_other_version_is_excluded(self, catalog_env):
    _write(catalog_env, {'driving': [_entry(verified_on=['0.11.2'])], 'dm': []})
    assert catalog_env.verified_entries('driving') == []

  def test_multi_version_entry_matches_both(self, catalog_env):
    _write(catalog_env, {'driving': [_entry(verified_on=['0.11.1', '0.11.2'])], 'dm': []})
    assert len(catalog_env.verified_entries('driving')) == 1

  def test_accepts_model_type_enum(self, catalog_env):
    from plugins.model_selector.model_swapper import ModelType
    _write(catalog_env, {'driving': [_entry()], 'dm': []})
    assert len(catalog_env.verified_entries(ModelType.DRIVING)) == 1

  def test_unknown_version_returns_nothing(self, catalog_env):
    catalog_env.VERSION_H.write_text('#define COMMA_VERSION "9.9.9"\n')
    _write(catalog_env, {'driving': [_entry()], 'dm': []})
    assert catalog_env.verified_entries('driving') == []


class TestIsVerified:
  def test_true_for_listed(self, catalog_env):
    _write(catalog_env, {'driving': [_entry()], 'dm': []})
    assert catalog_env.is_verified('driving', 'cool_people_3c957c6')

  def test_false_for_unlisted(self, catalog_env):
    _write(catalog_env, {'driving': [_entry()], 'dm': []})
    assert not catalog_env.is_verified('driving', 'mystery_model')

  def test_false_for_wrong_type(self, catalog_env):
    _write(catalog_env, {'driving': [_entry()], 'dm': []})
    assert not catalog_env.is_verified('dm', 'cool_people_3c957c6')


class TestBaselineEntry:
  def test_returns_the_baseline(self, catalog_env):
    _write(catalog_env, {'driving': [_entry(), _stock()], 'dm': []})
    assert catalog_env.baseline_entry('driving')['id'] == 'stock_0.11.1'

  def test_none_when_no_baseline(self, catalog_env):
    _write(catalog_env, {'driving': [_entry()], 'dm': []})
    assert catalog_env.baseline_entry('driving') is None


class TestUnlocked:
  def test_false_without_marker(self, catalog_env):
    assert not catalog_env.unlocked()

  def test_true_with_marker(self, catalog_env):
    catalog_env.UNLOCK_MARKER.write_text('')
    assert catalog_env.unlocked()


class TestValidateCatalog:
  def _valid(self):
    return {'driving': [_entry(), _stock()],
            'dm': [_stock(id='stock_dm_0.11.1', name='Release default DM')]}

  def test_valid_catalog_has_no_problems(self, catalog_env):
    assert catalog_env.validate_catalog(self._valid()) == []

  def test_missing_baseline_is_reported(self, catalog_env):
    c = self._valid()
    c['driving'] = [_entry()]
    problems = catalog_env.validate_catalog(c)
    assert any('0.11.1' in p and 'baseline' in p for p in problems)

  def test_two_baselines_is_reported(self, catalog_env):
    c = self._valid()
    c['driving'] = [_stock(), _stock(id='other_stock')]
    problems = catalog_env.validate_catalog(c)
    assert any('baseline' in p for p in problems)

  def test_duplicate_id_is_reported(self, catalog_env):
    c = self._valid()
    c['driving'] = [_entry(), _entry(), _stock()]
    assert any('duplicate' in p for p in catalog_env.validate_catalog(c))

  def test_downloadable_entry_needs_commit_and_files(self, catalog_env):
    c = self._valid()
    bad = _entry()
    del bad['commit']
    del bad['files']
    c['driving'] = [bad, _stock()]
    problems = catalog_env.validate_catalog(c)
    assert any('commit' in p for p in problems)
    assert any('files' in p for p in problems)

  def test_shipped_entry_needs_neither(self, catalog_env):
    assert catalog_env.validate_catalog(self._valid()) == []

  def test_baseline_must_be_verified_on_that_version(self, catalog_env):
    c = self._valid()
    c['driving'] = [_stock(verified_on=['0.11.2'], baseline_for=['0.11.1'])]
    assert any('baseline_for' in p for p in catalog_env.validate_catalog(c))
