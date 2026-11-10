"""Tests for c3_compat plugin — AGNOS version, device type, version description, health check."""
import pytest
from unittest.mock import patch, mock_open, MagicMock
import sys
import types
import importlib


@pytest.fixture(autouse=True)
def mock_openpilot(monkeypatch):
  """Mock openpilot imports."""
  for mod in ['openpilot', 'openpilot.common', 'openpilot.common.swaglog',
              'openpilot.cereal', 'openpilot.cereal.messaging']:
    monkeypatch.setitem(sys.modules, mod, MagicMock())


@pytest.fixture
def compat():
  import plugins.c3_compat.compat as mod
  importlib.reload(mod)
  return mod


class TestGetAgnosVersion:
  def test_reads_version_file(self, compat):
    with patch('builtins.open', mock_open(read_data='12.8.1\n')):
      assert compat.get_agnos_version() == '12.8.1'

  def test_missing_version_file(self, compat):
    with patch('builtins.open', side_effect=FileNotFoundError):
      assert compat.get_agnos_version() == 'unknown'


class TestGetDeviceType:
  def test_tici_detected(self, compat):
    with patch('builtins.open', mock_open(read_data='Qualcomm Technologies, Inc. tici\x00')):
      assert compat.get_device_type() == 'tici'

  def test_tizi_detected(self, compat):
    with patch('builtins.open', mock_open(read_data='Qualcomm Technologies, Inc. tizi\x00')):
      assert compat.get_device_type() == 'tizi'

  def test_mici_detected(self, compat):
    with patch('builtins.open', mock_open(read_data='Qualcomm Technologies, Inc. mici\x00')):
      assert compat.get_device_type() == 'mici'

  def test_unknown_device(self, compat):
    with patch('builtins.open', side_effect=FileNotFoundError):
      assert compat.get_device_type() == 'unknown'


class TestBuildVersionDescription:
  def _run(self, compat, monkeypatch, basedir):
    monkeypatch.setitem(sys.modules, 'openpilot.common.basedir',
                        types.SimpleNamespace(BASEDIR=str(basedir)))
    outputs = {
      ('git', 'rev-parse', '--abbrev-ref', 'HEAD'): 'master\n',
      ('git', 'rev-parse', 'HEAD'): '0123456789abcdef\n',
      ('git', 'show', '-s', '--format=%ct', 'HEAD'): '1700000000\n',
    }
    def fake_run(cmd, **kwargs):
      return MagicMock(stdout=outputs[tuple(cmd)])
    with patch('subprocess.run', side_effect=fake_run):
      return compat._build_version_description()

  def _write_version_h(self, path):
    path.mkdir(parents=True)
    (path / 'version.h').write_text('#define COMMA_VERSION "0.11.2"\n')

  def test_reads_version_h_under_openpilot(self, compat, monkeypatch, tmp_path):
    """BASEDIR/openpilot/common/version.h"""
    self._write_version_h(tmp_path / 'openpilot' / 'common')
    desc = self._run(compat, monkeypatch, tmp_path)
    assert desc.startswith('0.11.2 / master / 0123456 / ')

  def test_missing_version_h_returns_empty(self, compat, monkeypatch, tmp_path):
    assert self._run(compat, monkeypatch, tmp_path) == ''


class TestHealthCheck:
  def _call(self, compat, acc=None):
    """Helper: call on_health_check with empty accumulator (SubMaster mocked to fail)."""
    if acc is None:
      acc = {}
    # SubMaster will raise on update — health check falls back to warnings
    return compat.on_health_check(acc)

  def test_result_nested_under_plugin_key(self, compat):
    with patch.object(compat, 'get_device_type', return_value='tici'), \
         patch.object(compat, 'get_agnos_version', return_value='12.8.1'):
      result = self._call(compat)

    assert 'c3-compat' in result
    entry = result['c3-compat']
    assert entry['agnos_version'] == '12.8.1'
    assert entry['device_type'] == 'tici'
    assert entry['status'] in ('ok', 'warning')  # warning acceptable if pandaStates unavailable

  def test_accumulator_preserved(self, compat):
    """Other plugins' prior results must be passed through unchanged."""
    with patch.object(compat, 'get_device_type', return_value='tici'), \
         patch.object(compat, 'get_agnos_version', return_value='12.8.1'):
      result = self._call(compat, acc={'other-plugin': {'status': 'ok'}})

    assert 'other-plugin' in result
    assert result['other-plugin'] == {'status': 'ok'}
    assert 'c3-compat' in result

  def test_unknown_device_still_ok(self, compat):
    with patch.object(compat, 'get_device_type', return_value='unknown'), \
         patch.object(compat, 'get_agnos_version', return_value='unknown'):
      result = self._call(compat)
    assert result['c3-compat']['status'] in ('ok', 'warning')


class TestStartupInfo:
  """Every process that runs hooks imports the module; the startup work belongs to plugind only."""

  def test_import_does_no_startup_work(self):
    import plugins.c3_compat.compat as mod
    with patch('builtins.open', side_effect=AssertionError("file read at import")), \
         patch('subprocess.run', side_effect=AssertionError("subprocess at import")):
      importlib.reload(mod)
    assert mod._startup_logged is False

  def test_first_health_check_logs_once(self, compat):
    with patch.object(compat, 'get_device_type', return_value='tici'), \
         patch.object(compat, 'get_agnos_version', return_value='12.8.1'), \
         patch.object(compat, 'log_startup_info') as startup:
      compat.on_health_check({})
      compat.on_health_check({})
    startup.assert_called_once_with()

  def test_startup_info_goes_to_cloudlog(self, compat):
    with patch.object(compat, 'get_device_type', return_value='tici'), \
         patch.object(compat, 'get_agnos_version', return_value='12.8.1'):
      compat.log_startup_info()
    compat.cloudlog.info.assert_any_call("C3 compat: AGNOS 12.8.1, device tici")
