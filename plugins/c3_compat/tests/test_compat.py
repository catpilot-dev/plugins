"""Tests for c3_compat plugin — AGNOS version, device type, MCU expectations, health check."""
import pytest
from unittest.mock import patch, mock_open, MagicMock
import sys
import importlib


@pytest.fixture(autouse=True)
def mock_openpilot(monkeypatch):
  """Mock openpilot imports."""
  for mod in ['openpilot', 'openpilot.common', 'openpilot.common.swaglog',
              'cereal', 'cereal.messaging']:
    monkeypatch.setitem(sys.modules, mod, MagicMock())


@pytest.fixture
def compat():
  # Suppress log_startup_info() that runs on import
  with patch('builtins.open', side_effect=FileNotFoundError):
    import plugins.c3_compat.compat as mod
    importlib.reload(mod)
  return mod


class TestDeviceMCUExpectations:
  def test_c3_expects_f4(self, compat):
    assert compat.DEVICE_MCU_EXPECTATIONS['tici'] == 'f4'

  def test_c3x_expects_h7(self, compat):
    assert compat.DEVICE_MCU_EXPECTATIONS['tizi'] == 'h7'

  def test_c4_expects_h7(self, compat):
    assert compat.DEVICE_MCU_EXPECTATIONS['mici'] == 'h7'


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


class TestHealthCheck:
  def test_basic_health_check_no_params(self, compat):
    with patch.object(compat, 'get_device_type', return_value='tici'), \
         patch.object(compat, 'get_agnos_version', return_value='12.8.1'):
      result = compat.on_health_check(params=None)

    assert result['plugin'] == 'c3-compat'
    assert result['agnos_version'] == '12.8.1'
    assert result['device_type'] == 'tici'
    assert result['status'] == 'ok'
    assert result['warnings'] == []

  def test_unknown_device_still_ok(self, compat):
    with patch.object(compat, 'get_device_type', return_value='unknown'), \
         patch.object(compat, 'get_agnos_version', return_value='unknown'):
      result = compat.on_health_check(params=None)
    assert result['status'] == 'ok'
