"""Tests for model_selector plugin — ModelType, configs, listing, ONNX validation, PKL compat."""
import json
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
    assert 'driving_vision.onnx' in cfg['onnx_files']
    assert 'driving_policy.onnx' in cfg['onnx_files']
    assert len(cfg['onnx_files']) == 2
    assert cfg['active_file'] == 'active_driving_model'

  def test_dm_config(self, ModelSwapper, ModelType):
    cfg = ModelSwapper.MODEL_CONFIGS[ModelType.DM]
    assert 'dmonitoring_model.onnx' in cfg['onnx_files']
    assert len(cfg['onnx_files']) == 1
    assert cfg['active_file'] == 'active_dm_model'

  def test_pkl_patterns_match_onnx(self, ModelSwapper, ModelType):
    """Each ONNX file should have a corresponding tinygrad PKL pattern."""
    for mt in ModelType:
      cfg = ModelSwapper.MODEL_CONFIGS[mt]
      for onnx in cfg['onnx_files']:
        base = onnx.replace('.onnx', '')
        stem = f"{base}_tinygrad.pkl"
        assert stem in cfg['required_pkl_stems'], f"Missing required PKL stem for {onnx}"
        assert cfg['pkl_patterns'] == ['*pkl*'], "pkl_patterns should be ['*pkl*']"


class TestListModels:
  def test_empty_dir(self, ModelSwapper, ModelType, tmp_path):
    with patch.object(ModelSwapper, 'BASE_DATA_DIR', tmp_path):
      swapper = ModelSwapper.__new__(ModelSwapper)
      swapper.model_type = ModelType.DRIVING
      swapper.config = ModelSwapper.MODEL_CONFIGS[ModelType.DRIVING]
      swapper.models_dir = tmp_path / 'models' / 'driving'
      swapper.onnx_files = swapper.config['onnx_files']
      swapper.pkl_patterns = swapper.config['pkl_patterns']
      swapper.required_pkl_stems = swapper.config['required_pkl_stems']
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
    (model_dir / 'driving_vision.onnx').write_bytes(b'\x00')
    (model_dir / 'driving_policy.onnx').write_bytes(b'\x00')

    swapper = ModelSwapper.__new__(ModelSwapper)
    swapper.model_type = ModelType.DRIVING
    swapper.config = ModelSwapper.MODEL_CONFIGS[ModelType.DRIVING]
    swapper.models_dir = models_dir
    swapper.onnx_files = swapper.config['onnx_files']
    swapper.pkl_patterns = swapper.config['pkl_patterns']
    swapper.required_pkl_stems = swapper.config['required_pkl_stems']

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
    swapper.pkl_patterns = swapper.config['pkl_patterns']
    swapper.required_pkl_stems = swapper.config['required_pkl_stems']

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
    swapper.pkl_patterns = swapper.config['pkl_patterns']
    swapper.required_pkl_stems = swapper.config['required_pkl_stems']

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
    swapper.pkl_patterns = swapper.config['pkl_patterns']
    swapper.required_pkl_stems = swapper.config['required_pkl_stems']

    assert swapper.resolve_model_id('my_model') == 'my_model'

  def test_unknown_returns_original(self, ModelSwapper, ModelType, tmp_path):
    models_dir = tmp_path / 'models' / 'driving'
    models_dir.mkdir(parents=True)

    swapper = ModelSwapper.__new__(ModelSwapper)
    swapper.model_type = ModelType.DRIVING
    swapper.config = ModelSwapper.MODEL_CONFIGS[ModelType.DRIVING]
    swapper.models_dir = models_dir
    swapper.onnx_files = swapper.config['onnx_files']
    swapper.pkl_patterns = swapper.config['pkl_patterns']
    swapper.required_pkl_stems = swapper.config['required_pkl_stems']

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
    swapper.pkl_patterns = swapper.config['pkl_patterns']
    swapper.required_pkl_stems = swapper.config['required_pkl_stems']

    with pytest.raises(ValueError, match="not found"):
      swapper.swap_model('nonexistent')

  def test_missing_onnx_raises(self, swapper_mod, ModelSwapper, ModelType, tmp_path, monkeypatch):
    # Same as above: unlock the catalog gate so this exercises ONNX validation.
    monkeypatch.setattr(swapper_mod.catalog, 'unlocked', lambda: True)

    models_dir = tmp_path / 'models' / 'driving'
    model_dir = models_dir / 'incomplete_model'
    model_dir.mkdir(parents=True)
    # Only create one ONNX file
    (model_dir / 'driving_vision.onnx').write_bytes(b'\x00')

    swapper = ModelSwapper.__new__(ModelSwapper)
    swapper.model_type = ModelType.DRIVING
    swapper.config = ModelSwapper.MODEL_CONFIGS[ModelType.DRIVING]
    swapper.models_dir = models_dir
    swapper.active_model_file = models_dir.parent / 'active_driving_model'
    swapper.onnx_files = swapper.config['onnx_files']
    swapper.pkl_patterns = swapper.config['pkl_patterns']
    swapper.required_pkl_stems = swapper.config['required_pkl_stems']

    with patch.object(swapper, 'get_active_model', return_value='unknown'), \
         pytest.raises(ValueError, match="missing required ONNX"):
      swapper.swap_model('incomplete_model')


CATALOG_FIXTURE = {
  'driving': [{'id': 'good_model', 'name': 'Good', 'date': '2025-10-20',
               'commit': 'c' * 40, 'files': ['driving_vision.onnx', 'driving_policy.onnx'],
               'verified_on': ['0.11.1']},
              {'id': 'stock_0.11.1', 'name': 'Release default', 'date': '2026-05-18',
               'source': 'shipped', 'verified_on': ['0.11.1'], 'baseline_for': ['0.11.1']}],
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
  sw.pkl_patterns = sw.config['pkl_patterns']
  sw.required_pkl_stems = sw.config['required_pkl_stems']
  sw.display_name = sw.config['display_name']
  monkeypatch.setattr(swapper_mod.ModelSwapper, 'ACTIVE_DIR', active)
  return sw


def _point_catalog_at(tmp_path, monkeypatch, data):
  """Redirect the catalog module at a temp catalog, version.h and marker."""
  import plugins.model_selector.catalog as cat
  version_h = tmp_path / 'version.h'
  version_h.write_text('#define COMMA_VERSION "0.11.1"\n')
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


class TestImportStock:
  @pytest.fixture
  def stocked(self, swapper_mod, tmp_path, monkeypatch):
    _point_catalog_at(tmp_path, monkeypatch, CATALOG_FIXTURE)
    sw = _build_swapper(swapper_mod, tmp_path, monkeypatch)
    for f in sw.onnx_files:
      (sw.ACTIVE_DIR / f).write_bytes(b'shipped-onnx')
    return sw

  def test_imports_when_no_tracker(self, stocked):
    assert stocked.import_stock() is True
    dest = stocked.models_dir / 'stock_0.11.1'
    assert (dest / 'driving_vision.onnx').read_bytes() == b'shipped-onnx'
    assert json.loads((dest / 'model_info.json').read_text())['source'] == 'shipped'

  def test_is_idempotent(self, stocked):
    assert stocked.import_stock() is True
    assert stocked.import_stock() is False

  def test_skips_when_tracker_exists(self, stocked):
    stocked.active_model_file.write_text(json.dumps({'id': 'something_else'}))
    assert stocked.import_stock() is False
    assert not (stocked.models_dir / 'stock_0.11.1').exists()

  def test_skips_when_active_dir_incomplete(self, stocked):
    (stocked.ACTIVE_DIR / 'driving_policy.onnx').unlink()
    assert stocked.import_stock() is False

  def test_imported_stock_is_listed_and_verified(self, stocked):
    stocked.import_stock()
    by_id = {m['id']: m for m in stocked.list_models()}
    assert by_id['stock_0.11.1']['verified'] is True
