"""Tests for model_selector plugin — ModelType, configs, listing, ONNX validation, PKL compat."""
import hashlib
import json
import shutil
import pytest
from unittest.mock import patch, MagicMock
from pathlib import Path
import importlib
import sys


@pytest.fixture
def swapper_mod():
  import plugins.model_selector.model_swapper as mod
  importlib.reload(mod)
  return mod


@pytest.fixture
def ModelSwapper(swapper_mod):
  return swapper_mod.ModelSwapper


@pytest.fixture
def ModelType(swapper_mod):
  return swapper_mod.ModelType


class TestModelType:
  def test_driving_value(self, ModelType):
    assert ModelType.DRIVING.value == "driving"

  def test_dm_value(self, ModelType):
    assert ModelType.DM.value == "dm"


class TestModelConfigs:
  def test_driving_config(self, ModelSwapper, ModelType):
    cfg = ModelSwapper.MODEL_CONFIGS[ModelType.DRIVING]
    assert cfg['onnx_files'] == ['driving_supercombo.onnx']
    assert cfg['compiled_file'] == 'driving_tinygrad.pkl'
    assert 'pkl_patterns' not in cfg and 'required_pkl_stems' not in cfg
    assert cfg['active_file'] == 'active_driving_model'

  def test_dm_config(self, ModelSwapper, ModelType):
    cfg = ModelSwapper.MODEL_CONFIGS[ModelType.DM]
    assert cfg['onnx_files'] == ['dmonitoring_model.onnx']
    assert cfg['compiled_file'] == 'dmonitoring_model_tinygrad.pkl'
    assert cfg['active_file'] == 'active_dm_model'

  def test_active_dir_is_the_nested_openpilot_package(self, ModelSwapper):
    assert ModelSwapper.ACTIVE_DIR.parts[-4:] == ('openpilot', 'selfdrive', 'modeld', 'models')


class TestListModels:
  def test_empty_dir(self, ModelSwapper, ModelType, tmp_path):
    with patch.object(ModelSwapper, 'BASE_DATA_DIR', tmp_path):
      swapper = ModelSwapper.__new__(ModelSwapper)
      swapper.model_type = ModelType.DRIVING
      swapper.config = ModelSwapper.MODEL_CONFIGS[ModelType.DRIVING]
      swapper.models_dir = tmp_path / 'models' / 'driving'
      swapper.onnx_files = swapper.config['onnx_files']
      swapper.compiled_file = swapper.config['compiled_file']
      models = swapper.list_models()
    assert models == []

  def test_lists_models_with_info(self, ModelSwapper, ModelType, tmp_path):
    models_dir = tmp_path / 'models' / 'driving'
    model_dir = models_dir / 'test_model_abc123'
    model_dir.mkdir(parents=True)

    # Create model_info.json
    info = {'id': 'test_model_abc123', 'name': 'Test Model', 'date': '2025-12-15'}
    (model_dir / 'model_info.json').write_text(json.dumps(info))
    # Create ONNX files
    (model_dir / 'driving_supercombo.onnx').write_bytes(b'\x00')

    swapper = ModelSwapper.__new__(ModelSwapper)
    swapper.model_type = ModelType.DRIVING
    swapper.config = ModelSwapper.MODEL_CONFIGS[ModelType.DRIVING]
    swapper.models_dir = models_dir
    swapper.onnx_files = swapper.config['onnx_files']
    swapper.compiled_file = swapper.config['compiled_file']

    models = swapper.list_models()
    assert len(models) == 1
    assert models[0]['id'] == 'test_model_abc123'
    assert models[0]['name'] == 'Test Model'
    assert models[0]['has_onnx'] is True
    assert models[0]['cached_pkl_count'] == 0

  def test_skips_hidden_dirs(self, ModelSwapper, ModelType, tmp_path):
    models_dir = tmp_path / 'models' / 'driving'
    hidden = models_dir / '_backup_replaced'
    hidden.mkdir(parents=True)
    (hidden / 'model_info.json').write_text('{}')

    swapper = ModelSwapper.__new__(ModelSwapper)
    swapper.model_type = ModelType.DRIVING
    swapper.config = ModelSwapper.MODEL_CONFIGS[ModelType.DRIVING]
    swapper.models_dir = models_dir
    swapper.onnx_files = swapper.config['onnx_files']
    swapper.compiled_file = swapper.config['compiled_file']

    models = swapper.list_models()
    assert len(models) == 0

  def test_sorted_newest_first(self, ModelSwapper, ModelType, tmp_path):
    models_dir = tmp_path / 'models' / 'driving'

    for name, date in [('old', '2025-10-15'), ('new', '2025-12-01'), ('mid', '2025-11-15')]:
      d = models_dir / name
      d.mkdir(parents=True)
      (d / 'model_info.json').write_text(json.dumps({'id': name, 'name': name, 'date': date}))

    swapper = ModelSwapper.__new__(ModelSwapper)
    swapper.model_type = ModelType.DRIVING
    swapper.config = ModelSwapper.MODEL_CONFIGS[ModelType.DRIVING]
    swapper.models_dir = models_dir
    swapper.onnx_files = swapper.config['onnx_files']
    swapper.compiled_file = swapper.config['compiled_file']

    models = swapper.list_models()
    dates = [m['date'] for m in models]
    assert dates == ['2025-12-01', '2025-11-15', '2025-10-15']


class TestResolveModelId:
  def test_existing_directory(self, ModelSwapper, ModelType, tmp_path):
    models_dir = tmp_path / 'models' / 'driving'
    (models_dir / 'my_model').mkdir(parents=True)

    swapper = ModelSwapper.__new__(ModelSwapper)
    swapper.model_type = ModelType.DRIVING
    swapper.config = ModelSwapper.MODEL_CONFIGS[ModelType.DRIVING]
    swapper.models_dir = models_dir
    swapper.onnx_files = swapper.config['onnx_files']
    swapper.compiled_file = swapper.config['compiled_file']

    assert swapper.resolve_model_id('my_model') == 'my_model'

  def test_unknown_returns_original(self, ModelSwapper, ModelType, tmp_path):
    models_dir = tmp_path / 'models' / 'driving'
    models_dir.mkdir(parents=True)

    swapper = ModelSwapper.__new__(ModelSwapper)
    swapper.model_type = ModelType.DRIVING
    swapper.config = ModelSwapper.MODEL_CONFIGS[ModelType.DRIVING]
    swapper.models_dir = models_dir
    swapper.onnx_files = swapper.config['onnx_files']
    swapper.compiled_file = swapper.config['compiled_file']

    assert swapper.resolve_model_id('nonexistent') == 'nonexistent'


class TestSwapModelValidation:
  def test_missing_model_raises(self, swapper_mod, ModelSwapper, ModelType, tmp_path, monkeypatch):
    # These validations sit behind the catalog gate (STEP 0); unlock so the
    # test reaches the storage-validation code under test rather than the gate.
    monkeypatch.setattr(swapper_mod.catalog, 'unlocked', lambda: True)

    models_dir = tmp_path / 'models' / 'driving'
    models_dir.mkdir(parents=True)

    swapper = ModelSwapper.__new__(ModelSwapper)
    swapper.model_type = ModelType.DRIVING
    swapper.config = ModelSwapper.MODEL_CONFIGS[ModelType.DRIVING]
    swapper.models_dir = models_dir
    swapper.active_model_file = models_dir.parent / 'active_driving_model'
    swapper.onnx_files = swapper.config['onnx_files']
    swapper.compiled_file = swapper.config['compiled_file']

    with pytest.raises(ValueError, match="not found"):
      swapper.swap_model('nonexistent')

  def test_missing_onnx_raises(self, swapper_mod, ModelSwapper, ModelType, tmp_path, monkeypatch):
    # Same as above: unlock the catalog gate so this exercises ONNX validation.
    monkeypatch.setattr(swapper_mod.catalog, 'unlocked', lambda: True)

    models_dir = tmp_path / 'models' / 'driving'
    model_dir = models_dir / 'incomplete_model'
    model_dir.mkdir(parents=True)
    # Only a legacy split-model file, no supercombo
    (model_dir / 'driving_vision.onnx').write_bytes(b'\x00')

    swapper = ModelSwapper.__new__(ModelSwapper)
    swapper.model_type = ModelType.DRIVING
    swapper.config = ModelSwapper.MODEL_CONFIGS[ModelType.DRIVING]
    swapper.models_dir = models_dir
    swapper.active_model_file = models_dir.parent / 'active_driving_model'
    swapper.onnx_files = swapper.config['onnx_files']
    swapper.compiled_file = swapper.config['compiled_file']

    with patch.object(swapper, 'get_active_model', return_value='unknown'), \
         pytest.raises(ValueError, match="missing required ONNX"):
      swapper.swap_model('incomplete_model')


CATALOG_FIXTURE = {
  'driving': [{'id': 'good_model', 'name': 'Good', 'date': '2025-10-20',
               'commit': 'c' * 40, 'files': ['driving_supercombo.onnx'],
               'verified_on': ['0.11.2']},
              {'id': 'stock_0.11.2', 'name': 'Release default', 'date': '2026-10-14',
               'source': 'shipped', 'sha256': hashlib.sha256(b'shipped-onnx').hexdigest(), 'size': 12,
               'verified_on': ['0.11.2'], 'baseline_for': ['0.11.2']}],
  'dm': [],
}


def _build_swapper(swapper_mod, tmp_path, monkeypatch):
  """A DRIVING swapper rooted in tmp_path.

  Built with __new__ like the other tests in this file: ModelSwapper.__init__
  mkdirs under the real BASE_DATA_DIR, which a test must not touch.
  """
  models_dir = tmp_path / 'models' / 'driving'
  models_dir.mkdir(parents=True)
  active = tmp_path / 'active'
  active.mkdir()

  sw = swapper_mod.ModelSwapper.__new__(swapper_mod.ModelSwapper)
  sw.model_type = swapper_mod.ModelType.DRIVING
  sw.config = swapper_mod.ModelSwapper.MODEL_CONFIGS[swapper_mod.ModelType.DRIVING]
  sw.models_dir = models_dir
  sw.active_model_file = models_dir.parent / 'active_driving_model'
  sw.onnx_files = sw.config['onnx_files']
  sw.compiled_file = sw.config['compiled_file']
  sw.display_name = sw.config['display_name']
  monkeypatch.setattr(swapper_mod.ModelSwapper, 'ACTIVE_DIR', active)
  return sw


def _point_catalog_at(tmp_path, monkeypatch, data):
  """Redirect the catalog module at a temp catalog, version.h and marker."""
  import plugins.model_selector.catalog as cat
  version_h = tmp_path / 'version.h'
  version_h.write_text('#define COMMA_VERSION "0.11.2"\n')
  monkeypatch.setattr(cat, 'VERSION_H', version_h)
  monkeypatch.setattr(cat, 'CATALOG_FILE', tmp_path / 'catalog.json')
  monkeypatch.setattr(cat, 'UNLOCK_MARKER', tmp_path / '.unlocked')
  (tmp_path / 'catalog.json').write_text(json.dumps(data))
  return cat


class TestCatalogGate:
  @pytest.fixture
  def gated(self, swapper_mod, tmp_path, monkeypatch):
    cat = _point_catalog_at(tmp_path, monkeypatch, CATALOG_FIXTURE)
    return _build_swapper(swapper_mod, tmp_path, monkeypatch), cat, tmp_path

  def _install(self, sw, model_id, date='2025-10-20'):
    d = sw.models_dir / model_id
    d.mkdir(parents=True)
    for f in sw.onnx_files:
      (d / f).write_bytes(b'onnx')
    (d / 'model_info.json').write_text(json.dumps({'name': model_id, 'date': date}))
    return d

  def test_list_models_flags_verified(self, gated):
    sw, _, _ = gated
    self._install(sw, 'good_model')
    self._install(sw, 'mystery_model')
    by_id = {m['id']: m for m in sw.list_models()}
    assert by_id['good_model']['verified'] is True
    assert by_id['mystery_model']['verified'] is False

  def test_list_models_no_longer_hides_old_models(self, gated):
    sw, _, _ = gated
    self._install(sw, 'ancient_model', date='2024-01-01')
    assert 'ancient_model' in {m['id'] for m in sw.list_models()}

  def test_swap_refuses_unverified(self, gated):
    sw, _, _ = gated
    self._install(sw, 'mystery_model')
    with pytest.raises(ValueError, match='not verified'):
      sw.swap_model('mystery_model')

  def test_swap_allows_unverified_when_unlocked(self, gated):
    sw, cat, tmp_path = gated
    self._install(sw, 'mystery_model')
    cat.UNLOCK_MARKER.write_text('')
    sw.swap_model('mystery_model')
    assert sw.get_active_model() == 'mystery_model'

  def test_swap_allows_verified(self, gated):
    sw, _, _ = gated
    self._install(sw, 'good_model')
    sw.swap_model('good_model')
    assert sw.get_active_model() == 'good_model'

  def test_swap_leaves_other_compiled_files_alone(self, gated):
    sw, cat, _ = gated
    cat.UNLOCK_MARKER.write_text('')
    self._install(sw, 'good_model')
    others = ['big_driving_tinygrad.pkl', 'big_driving_warp_1928x1208_tinygrad.pkl',
              'driving_warp_1928x1208_tinygrad.pkl', 'dm_warp_1928x1208_tinygrad.pkl',
              'dmonitoring_model_tinygrad.pkl', 'dmonitoring_model.onnx']
    for f in others:
      (sw.ACTIVE_DIR / f).write_bytes(b'x')
    (sw.ACTIVE_DIR / 'driving_tinygrad.pkl').write_bytes(b'stale')
    sw.swap_model('good_model')  # driving_tinygrad.pkl absent or stale: both must work
    assert not (sw.ACTIVE_DIR / 'driving_tinygrad.pkl').exists()
    assert all((sw.ACTIVE_DIR / f).exists() for f in others)
    assert (sw.ACTIVE_DIR / 'driving_supercombo.onnx').read_bytes() == b'onnx'

  def test_swap_succeeds_when_nothing_was_compiled_yet(self, gated):
    sw, cat, _ = gated
    cat.UNLOCK_MARKER.write_text('')
    self._install(sw, 'good_model')
    assert sw.swap_model('good_model')['requires_reboot'] is True

  def test_cache_copies_only_the_models_own_artifact(self, gated):
    sw, _, _ = gated
    d = self._install(sw, 'good_model')
    for f in ['driving_tinygrad.pkl', 'big_driving_tinygrad.pkl', 'driving_warp_1928x1208_tinygrad.pkl']:
      (sw.ACTIVE_DIR / f).write_bytes(b'c')
    assert sw.cache_compiled_pkl('good_model') == 1
    assert (d / 'driving_tinygrad.pkl').exists()
    assert not (d / 'big_driving_tinygrad.pkl').exists()

  def test_legacy_split_dir_cannot_be_activated_even_unlocked(self, gated):
    sw, cat, _ = gated
    cat.UNLOCK_MARKER.write_text('')
    d = sw.models_dir / 'pop_model_37727'
    d.mkdir()
    (d / 'driving_vision.onnx').write_bytes(b'v'); (d / 'driving_policy.onnx').write_bytes(b'p')
    (d / 'model_info.json').write_text(json.dumps({'name': 'POP', 'date': '2026-03-24'}))
    with pytest.raises(ValueError, match='missing required ONNX'):
      sw.swap_model('pop_model_37727')
    assert all(not m['has_onnx'] for m in sw.list_models() if m['id'] == 'pop_model_37727')


class TestImportStock:
  @pytest.fixture
  def stocked(self, swapper_mod, tmp_path, monkeypatch):
    _point_catalog_at(tmp_path, monkeypatch, CATALOG_FIXTURE)
    sw = _build_swapper(swapper_mod, tmp_path, monkeypatch)
    for f in sw.onnx_files:
      (sw.ACTIVE_DIR / f).write_bytes(b'shipped-onnx')
    return sw

  def test_imports_when_active_matches_shipped_hash(self, stocked):
    assert stocked.import_stock() is True
    dest = stocked.models_dir / 'stock_0.11.2'
    assert (dest / 'driving_supercombo.onnx').read_bytes() == b'shipped-onnx'
    assert json.loads((dest / 'model_info.json').read_text())['source'] == 'shipped'

  def test_imports_even_when_a_tracker_exists(self, stocked):
    stocked.active_model_file.write_text(json.dumps({'id': 'something_else'}))
    assert stocked.import_stock() is True

  def test_skips_when_active_bytes_differ(self, stocked):
    (stocked.ACTIVE_DIR / 'driving_supercombo.onnx').write_bytes(b'swapped-onnx')  # same length, other bytes
    assert stocked.import_stock() is False
    assert not (stocked.models_dir / 'stock_0.11.2').exists()

  def test_pointer_sized_file_is_not_imported(self, stocked):
    (stocked.ACTIVE_DIR / 'driving_supercombo.onnx').write_text('version https://git-lfs.github.com/spec/v1\noid sha256:65a0\nsize 60918562\n')
    assert stocked.import_stock() is False

  def test_wrong_size_is_rejected_without_hashing(self, stocked, monkeypatch):
    (stocked.ACTIVE_DIR / 'driving_supercombo.onnx').write_bytes(b'x' * 99)
    monkeypatch.setattr(stocked, '_file_sha256', MagicMock(side_effect=AssertionError('hashed')))
    assert stocked.import_stock() is False

  def test_stale_legacy_tracker_is_rewritten_to_stock(self, stocked):
    legacy = stocked.models_dir / 'pop_model_37727'; legacy.mkdir()
    (legacy / 'driving_vision.onnx').write_bytes(b'v')
    stocked.active_model_file.write_text(json.dumps({'id': 'pop_model_37727', 'name': 'POP'}))
    stocked.import_stock()
    assert json.loads(stocked.active_model_file.read_text())['id'] == 'stock_0.11.2'

  def test_missing_tracker_is_written_to_stock(self, stocked):
    stocked.import_stock()
    assert json.loads(stocked.active_model_file.read_text())['id'] == 'stock_0.11.2'

  def test_tracker_naming_a_loadable_model_is_kept(self, stocked):
    good = stocked.models_dir / 'good_model'; good.mkdir()
    (good / 'driving_supercombo.onnx').write_bytes(b'onnx')
    stocked.active_model_file.write_text(json.dumps({'id': 'good_model'}))
    stocked.import_stock()
    assert json.loads(stocked.active_model_file.read_text())['id'] == 'good_model'

  def test_existing_stock_dir_still_repairs_stale_tracker(self, stocked):
    assert stocked.import_stock() is True
    stocked.active_model_file.unlink()
    assert stocked.import_stock() is False
    assert json.loads(stocked.active_model_file.read_text())['id'] == 'stock_0.11.2'

  def test_no_baseline_for_version_imports_nothing(self, stocked, tmp_path):
    (tmp_path / 'version.h').write_text('#define COMMA_VERSION "0.11.3"\n')
    assert stocked.import_stock() is False
    assert not stocked.active_model_file.exists()

  def test_is_idempotent(self, stocked):
    assert stocked.import_stock() is True
    assert stocked.import_stock() is False

  def test_imported_stock_is_listed_and_verified(self, stocked):
    stocked.import_stock()
    by_id = {m['id']: m for m in stocked.list_models()}
    assert by_id['stock_0.11.2']['verified'] is True

  def test_failed_copy_does_not_leave_an_orphaned_temp_dir(self, stocked, monkeypatch):
    # Simulate a failure partway through the build (e.g. disk full). The temp
    # dir must be cleaned up rather than surviving as orphaned junk, and the
    # exception must still propagate (callers, e.g. on_health_check, already
    # wrap this in try/except).
    monkeypatch.setattr(shutil, 'copy2', MagicMock(side_effect=OSError('disk full')))
    with pytest.raises(OSError, match='disk full'):
      stocked.import_stock()

    leftovers = [p for p in stocked.models_dir.iterdir() if p.name.startswith('_')]
    assert leftovers == [], f"orphaned temp dir(s) left behind: {leftovers}"
    assert not (stocked.models_dir / 'stock_0.11.2').exists()

  def test_successful_import_still_lands_atomically(self, stocked):
    # The build happens under a _-prefixed temp name and only becomes the
    # real model dir via an atomic rename — no _-prefixed leftovers after a
    # successful import either.
    assert stocked.import_stock() is True
    dest = stocked.models_dir / 'stock_0.11.2'
    assert dest.is_dir()
    leftovers = [p for p in stocked.models_dir.iterdir() if p.name.startswith('_')]
    assert leftovers == []

  def test_concurrent_callers_use_distinct_temp_dirs(self, stocked):
    # UI's show() and plugind's on_health_check (every ~5s) both call
    # import_stock; the temp dir name must not be a fixed path they could
    # race on. We can't easily run two real processes in a unit test, but we
    # can assert the mechanism (pid-qualified name) that prevents the race.
    import os
    tmp_name = f"_stock_0.11.2.{os.getpid()}.tmp"
    assert not (stocked.models_dir / tmp_name).exists()
    assert stocked.import_stock() is True
    # The pid-qualified temp path must never survive as itself post-rename.
    assert not (stocked.models_dir / tmp_name).exists()


class TestDrift:
  @pytest.fixture
  def drifty(self, swapper_mod, tmp_path, monkeypatch):
    cat = _point_catalog_at(tmp_path, monkeypatch, CATALOG_FIXTURE)
    sw = _build_swapper(swapper_mod, tmp_path, monkeypatch)
    d = sw.models_dir / 'pop_supercombo_37727'; d.mkdir()
    (d / 'driving_supercombo.onnx').write_bytes(b'pop-bytes')
    (d / 'model_info.json').write_text(json.dumps({'name': 'POP', 'sha256': hashlib.sha256(b'pop-bytes').hexdigest()}))
    return sw, cat

  def test_matches_when_active_is_the_model(self, drifty):
    sw, _ = drifty
    (sw.ACTIVE_DIR / 'driving_supercombo.onnx').write_bytes(b'pop-bytes')
    assert sw.active_file_matches('pop_supercombo_37727') is True

  def test_false_when_deploy_put_stock_back(self, drifty):
    sw, _ = drifty
    (sw.ACTIVE_DIR / 'driving_supercombo.onnx').write_bytes(b'shipped-onnx')
    assert sw.active_file_matches('pop_supercombo_37727') is False

  def test_active_file_matches_false_for_pointer(self, drifty):
    sw, _ = drifty
    (sw.ACTIVE_DIR / 'driving_supercombo.onnx').write_text('version https://git-lfs.github.com/spec/v1\n')
    assert sw.active_file_matches('pop_supercombo_37727') is False

  def test_missing_active_file_is_false(self, drifty):
    sw, _ = drifty
    (sw.ACTIVE_DIR / 'driving_supercombo.onnx').unlink(missing_ok=True)
    assert sw.active_file_matches('pop_supercombo_37727') is False

  def test_shipped_expectation_comes_from_catalog(self, drifty):
    sw, _ = drifty
    assert sw.expected_sha256('stock_0.11.2') == hashlib.sha256(b'shipped-onnx').hexdigest()

  def test_none_without_expectation(self, drifty):
    sw, _ = drifty
    d = sw.models_dir / 'old'; d.mkdir(); (d / 'model_info.json').write_text('{}')
    assert sw.active_file_matches('old') is None

  def test_cache_refuses_a_pkl_compiled_from_another_file(self, drifty):
    # A deploy put stock back under the POP tracker; scons compiled stock.
    sw, _ = drifty
    (sw.ACTIVE_DIR / 'driving_supercombo.onnx').write_bytes(b'shipped-onnx')
    (sw.ACTIVE_DIR / 'driving_tinygrad.pkl').write_bytes(b'compiled-from-stock')
    assert sw.cache_compiled_pkl('pop_supercombo_37727') == 0
    d = sw.models_dir / 'pop_supercombo_37727'
    assert not (d / 'driving_tinygrad.pkl').exists() and not (d / '.tinygrad_commit').exists()

  def test_cache_copies_a_pkl_compiled_from_the_models_file(self, drifty):
    sw, _ = drifty
    (sw.ACTIVE_DIR / 'driving_supercombo.onnx').write_bytes(b'pop-bytes')
    (sw.ACTIVE_DIR / 'driving_tinygrad.pkl').write_bytes(b'compiled-from-pop')
    assert sw.cache_compiled_pkl('pop_supercombo_37727') == 1
    assert (sw.models_dir / 'pop_supercombo_37727' / 'driving_tinygrad.pkl').read_bytes() == b'compiled-from-pop'

  def test_swap_away_from_a_drifted_model_caches_nothing_under_it(self, drifty):
    sw, _ = drifty
    good = sw.models_dir / 'good_model'; good.mkdir()
    (good / 'driving_supercombo.onnx').write_bytes(b'onnx')
    (good / 'model_info.json').write_text(json.dumps({'name': 'Good'}))
    sw.active_model_file.write_text(json.dumps({'id': 'pop_supercombo_37727'}))
    (sw.ACTIVE_DIR / 'driving_supercombo.onnx').write_bytes(b'shipped-onnx')
    (sw.ACTIVE_DIR / 'driving_tinygrad.pkl').write_bytes(b'compiled-from-stock')
    assert sw.swap_model('good_model')['previous_model_cached'] == 0
    assert not (sw.models_dir / 'pop_supercombo_37727' / 'driving_tinygrad.pkl').exists()

  def test_hash_is_cached_by_size_and_mtime(self, drifty, monkeypatch):
    sw, _ = drifty
    (sw.ACTIVE_DIR / 'driving_supercombo.onnx').write_bytes(b'pop-bytes')
    calls = []
    real = sw._file_sha256
    monkeypatch.setattr(sw, '_file_sha256', lambda p: calls.append(p) or real(p))
    sw.active_file_matches('pop_supercombo_37727'); sw.active_file_matches('pop_supercombo_37727')
    assert len(calls) == 1


class TestDriftFile:
  @pytest.fixture
  def mod(self, swapper_mod, tmp_path, monkeypatch):
    cfgs = swapper_mod.ModelSwapper.MODEL_CONFIGS
    t = swapper_mod.ModelType.DRIVING
    (tmp_path / 'models').mkdir()
    monkeypatch.setitem(cfgs, t, {**cfgs[t], 'models_dir': tmp_path / 'models' / 'driving'})
    return swapper_mod

  def test_round_trip(self, mod):
    mod.write_drift({'driving': True, 'dm': False})
    assert mod.read_drift() == {'driving': True, 'dm': False}

  def test_missing_file_reads_empty(self, mod):
    assert mod.read_drift() == {}

  def test_malformed_file_reads_empty(self, mod):
    mod.drift_file().write_text('not json')
    assert mod.read_drift() == {}

  def test_no_tmp_left_behind(self, mod):
    mod.write_drift({'driving': False})
    assert not mod.drift_file().with_name('.drift.tmp').exists()


class TestActivationAllowed:
  """The panel's Activate button: tested models only; the active one only when its file drifted."""

  def test_inactive_tested_model_can_be_activated(self, swapper_mod):
    assert swapper_mod.activation_allowed({'verified': True}, is_active=False, drifted=False) is True

  def test_active_model_without_drift_is_greyed_out(self, swapper_mod):
    assert swapper_mod.activation_allowed({'verified': True}, is_active=True, drifted=False) is False

  def test_active_drifted_model_can_be_reactivated(self, swapper_mod):
    assert swapper_mod.activation_allowed({'verified': True}, is_active=True, drifted=True) is True

  @pytest.mark.parametrize('model', [{'verified': False}, {}])
  @pytest.mark.parametrize('is_active,drifted', [(False, False), (True, False), (True, True)])
  def test_untested_model_never_activates(self, swapper_mod, model, is_active, drifted):
    assert swapper_mod.activation_allowed(model, is_active=is_active, drifted=drifted) is False
