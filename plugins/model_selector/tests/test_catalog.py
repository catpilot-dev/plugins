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
  version_h.write_text('#define COMMA_VERSION "0.11.2"\n')
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
    'files': ['driving_supercombo.onnx'],
    'verified_on': ['0.11.2'],
  }
  e.update(over)
  return e


def _stock(**over):
  e = {
    'id': 'stock_0.11.2',
    'name': 'Release default',
    'date': '2026-10-14',
    'source': 'shipped',
    'sha256': '65a08adc31d5c456219687d99b7bf5e44d61dae2d49ea67850e76105c7248cce',
    'size': 60918562,
    'verified_on': ['0.11.2'],
    'baseline_for': ['0.11.2'],
  }
  e.update(over)
  return e


def _converted(**over):
  e = {
    'id': 'pop_supercombo_37727',
    'name': 'POP model',
    'date': '2026-03-24',
    'source': 'converted',
    'converted_from': {'commit': '1' * 40, 'vision': 'v/200', 'policy': 'p/100'},
    'files': [{'name': 'driving_supercombo.onnx', 'url': 'https://example.invalid/pop.onnx',
               'sha256': 'a' * 64, 'size': 60918575}],
    'verified_on': [],
  }
  e.update(over)
  return e


class TestOpenpilotVersion:
  def test_parses_version_h(self, catalog_env):
    assert catalog_env.openpilot_version() == '0.11.2'

  def test_missing_version_h_falls_back_to_empty(self, catalog_env, tmp_path, monkeypatch):
    monkeypatch.setattr(catalog_env, 'VERSION_H', tmp_path / 'nope.h')
    # The openpilot package is not importable from the plugins repo, so the
    # manifest fallback fails and the chain ends at ''.
    assert catalog_env.openpilot_version() == ''

  def test_malformed_version_h_does_not_raise(self, catalog_env):
    catalog_env.VERSION_H.write_text('garbage with no quotes\n')
    assert catalog_env.openpilot_version() == ''

  def test_default_version_h_is_in_the_nested_package(self):
    import plugins.model_selector.catalog as mod
    assert mod.VERSION_H.parts[-3:] == ('openpilot', 'common', 'version.h')


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

  def test_non_utf8_file_fails_closed(self, catalog_env):
    # A binary-garbled catalog (e.g. truncated write, wrong encoding) must
    # not raise UnicodeDecodeError out of the gate.
    catalog_env.CATALOG_FILE.write_bytes(b'\xff\xfe\x00\x01garbage')
    assert catalog_env.load_catalog() == {}
    assert catalog_env.verified_entries('driving') == []

  def test_wrong_shaped_type_value_fails_closed(self, catalog_env):
    # Structurally valid JSON of the wrong shape: passes isinstance(data,
    # dict) but a per-type value that isn't a list must not raise TypeError
    # when iterated.
    _write(catalog_env, {'driving': 5, 'dm': []})
    result = catalog_env.load_catalog()
    assert result['driving'] == []
    assert catalog_env.verified_entries('driving') == []

  def test_one_bad_type_does_not_poison_the_other(self, catalog_env):
    _write(catalog_env, {'driving': [_entry()], 'dm': 'nope'})
    result = catalog_env.load_catalog()
    assert result['dm'] == []
    assert len(result['driving']) == 1
    assert catalog_env.verified_entries('driving')[0]['id'] == 'cool_people_3c957c6'
    assert catalog_env.verified_entries('dm') == []


class TestVerifiedEntries:
  def test_matching_version_is_returned(self, catalog_env):
    _write(catalog_env, {'driving': [_entry()], 'dm': []})
    assert [e['id'] for e in catalog_env.verified_entries('driving')] == ['cool_people_3c957c6']

  def test_other_version_is_excluded(self, catalog_env):
    _write(catalog_env, {'driving': [_entry(verified_on=['0.11.1'])], 'dm': []})
    assert catalog_env.verified_entries('driving') == []

  def test_multi_version_entry_matches_both(self, catalog_env):
    _write(catalog_env, {'driving': [_entry(verified_on=['0.11.2', '0.11.3'])], 'dm': []})
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
    assert catalog_env.baseline_entry('driving')['id'] == 'stock_0.11.2'

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
            'dm': [_stock(id='stock_dm_0.11.2', name='Release default DM')]}

  def test_valid_catalog_has_no_problems(self, catalog_env):
    assert catalog_env.validate_catalog(self._valid()) == []

  def test_missing_baseline_is_reported(self, catalog_env):
    c = self._valid()
    c['driving'] = [_entry()]
    problems = catalog_env.validate_catalog(c)
    assert any('0.11.2' in p and 'baseline' in p for p in problems)

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
    c['driving'] = [_stock(verified_on=['0.11.1'], baseline_for=['0.11.2'])]
    assert any('baseline_for' in p for p in catalog_env.validate_catalog(c))

  def test_upstream_reverted_field_is_allowed(self, catalog_env):
    """A model comma withdrew can still be catalogued — the field is provenance."""
    c = {'driving': [_entry(upstream_reverted='d' * 40), _stock()],
         'dm': [_stock(id='stock_dm_0.11.2', name='Release default DM')]}
    assert catalog_env.validate_catalog(c) == []

  def test_upstream_reverted_does_not_affect_verification(self, catalog_env):
    _write(catalog_env, {'driving': [_entry(upstream_reverted='d' * 40)], 'dm': []})
    assert catalog_env.is_verified('driving', 'cool_people_3c957c6')


class TestValidateKinds:
  def test_shipped_entry_needs_sha256_and_size(self, catalog_env):
    problems = catalog_env.validate_catalog({'driving': [_stock(sha256=None, size=0)], 'dm': []})
    assert any('sha256' in p for p in problems) and any('size' in p for p in problems)

  def test_shipped_sha256_must_be_64_hex(self, catalog_env):
    assert catalog_env.validate_catalog({'driving': [_stock(sha256='xyz')], 'dm': []})

  def test_converted_entry_is_valid(self, catalog_env):
    dm = [_stock(id='stock_dm_0.11.2', name='Release default DM')]
    assert catalog_env.validate_catalog({'driving': [_stock(), _converted()], 'dm': dm}) == []

  def test_converted_entry_needs_file_objects(self, catalog_env):
    bad = _converted(files=['driving_supercombo.onnx'])
    assert any('files' in p for p in catalog_env.validate_catalog({'driving': [_stock(), bad], 'dm': []}))

  def test_converted_file_needs_url_sha256_size(self, catalog_env):
    bad = _converted(files=[{'name': 'driving_supercombo.onnx'}])
    problems = catalog_env.validate_catalog({'driving': [_stock(), bad], 'dm': []})
    assert any('url' in p for p in problems) and any('sha256' in p for p in problems) and any('size' in p for p in problems)

  def test_converted_entry_needs_converted_from(self, catalog_env):
    bad = _converted(converted_from=None)
    assert any('converted_from' in p for p in catalog_env.validate_catalog({'driving': [_stock(), bad], 'dm': []}))


class TestEntry:
  def test_entry_is_found_regardless_of_version(self, catalog_env):
    _write(catalog_env, {'driving': [_stock(), _converted()], 'dm': []})
    assert catalog_env.entry('driving', 'pop_supercombo_37727')['source'] == 'converted'
    assert catalog_env.is_verified('driving', 'pop_supercombo_37727') is False

  def test_entry_none_for_unknown(self, catalog_env):
    _write(catalog_env, {'driving': [_stock()], 'dm': []})
    assert catalog_env.entry('driving', 'nope') is None


class TestShippedCatalog:
  """The catalog that actually ships must be valid and cover this version."""

  def test_shipped_catalog_is_valid(self, cat):
    assert cat.validate_catalog() == []

  def test_shipped_catalog_has_baselines_for_0_11_2_only(self, cat):
    c = cat.load_catalog()
    for t in ('driving', 'dm'):
      assert [e['id'] for e in c[t] if '0.11.2' in e.get('baseline_for', [])] == [f"stock_{'' if t == 'driving' else 'dm_'}0.11.2"]
      assert all('0.11.1' not in e.get('verified_on', []) for e in c[t])

  def test_shipped_entries_carry_the_tag_hashes(self, cat):
    c = cat.load_catalog()
    d = next(e for e in c['driving'] if e['id'] == 'stock_0.11.2')
    assert (d['sha256'], d['size']) == ('65a08adc31d5c456219687d99b7bf5e44d61dae2d49ea67850e76105c7248cce', 60918562)
    m = next(e for e in c['dm'] if e['id'] == 'stock_dm_0.11.2')
    assert (m['sha256'], m['size']) == ('dee5a294e8afaacc9295ac5d100e00733ecac278e79264b96331e40a3ede1b04', 7844499)

  def test_shipped_catalog_lists_pop_as_converted_and_untested(self, cat):
    c = cat.load_catalog()
    pop = next(e for e in c['driving'] if e['id'] == 'pop_supercombo_37727')
    assert pop['source'] == 'converted' and pop['verified_on'] == []
    f = pop['files'][0]
    assert f['name'] == 'driving_supercombo.onnx'
    assert (f['sha256'], f['size']) == ('98be77d66621ce26a064d71af6f81b36b04389b5106cee8dfdbd2a973d40fee5', 60926942)
    assert f['url'] == 'https://github.com/catpilot-dev/plugins/releases/download/models-0.11.2/pop_supercombo_37727.onnx'
