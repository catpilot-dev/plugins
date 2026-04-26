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
  def test_missing_model_raises(self, ModelSwapper, ModelType, tmp_path):
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

  def test_missing_onnx_raises(self, ModelSwapper, ModelType, tmp_path):
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
