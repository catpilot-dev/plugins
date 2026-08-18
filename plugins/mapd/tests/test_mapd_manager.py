"""Tests for mapd plugin — version parsing, path construction, update logic."""
import pytest
from unittest.mock import patch, mock_open, MagicMock
from pathlib import Path
import importlib


@pytest.fixture
def manager():
  import plugins.mapd.mapd_manager as mod
  importlib.reload(mod)
  return mod


class TestConstants:
  def test_paths(self, manager):
    assert manager.MAPD_PATH == Path("/data/media/0/osm/mapd")
    assert manager.BACKUP_DIR == Path("/data/media/0/osm/mapd_backups")
    assert manager.VERSION_PATH == Path("/data/media/0/osm/mapd_version")
    assert manager.PLUGIN_DATA_DIR == Path("/data/plugins-runtime/mapd/data")

  def test_github_url(self, manager):
    assert "pfeiferj/mapd" in manager.GITHUB_API_URL
    assert manager.GITHUB_API_URL.endswith("/latest")


class TestGetCurrentVersion:
  def test_reads_from_params(self, manager):
    with patch.object(Path, 'read_text', return_value='v2.1.0\n'):
      result = manager.get_current_version()
    assert result == 'v2.1.0'

  def test_default_when_missing(self, manager):
    with patch.object(Path, 'read_text', side_effect=FileNotFoundError):
      result = manager.get_current_version()
    assert result == 'v2.0.2'


class TestGetLatestVersion:
  def test_parses_github_response(self, manager):
    mock_response = MagicMock()
    mock_response.read.return_value = b'{"tag_name": "v2.1.5", "published_at": "2026-01-31T03:28:20Z"}'
    mock_response.__enter__ = MagicMock(return_value=mock_response)
    mock_response.__exit__ = MagicMock(return_value=False)

    with patch('urllib.request.urlopen', return_value=mock_response):
      version, date = manager.get_latest_version()

    assert version == 'v2.1.5'
    assert date == '2026-01-31'

  def test_handles_network_error(self, manager):
    with patch('urllib.request.urlopen', side_effect=Exception("timeout")):
      version, date = manager.get_latest_version()
    assert version == ''
    assert date == ''

  def test_handles_missing_date(self, manager):
    mock_response = MagicMock()
    mock_response.read.return_value = b'{"tag_name": "v2.1.5"}'
    mock_response.__enter__ = MagicMock(return_value=mock_response)
    mock_response.__exit__ = MagicMock(return_value=False)

    with patch('urllib.request.urlopen', return_value=mock_response):
      version, date = manager.get_latest_version()
    assert version == 'v2.1.5'
    assert date == ''


class TestBackupCurrentBinary:
  def test_no_binary_to_backup(self, manager):
    with patch.object(Path, 'exists', return_value=False):
      result = manager.backup_current_binary()
    assert result is True

  def test_backup_path_includes_version(self, manager):
    with patch.object(Path, 'exists', return_value=True), \
         patch.object(Path, 'mkdir'), \
         patch('shutil.copy2') as mock_copy, \
         patch.object(manager, 'get_current_version', return_value='v2.1.0'):
      manager.backup_current_binary()

    dst = mock_copy.call_args[0][1]
    assert 'v2.1.0' in str(dst)


class TestCheckForUpdates:
  def test_up_to_date(self, manager):
    with patch.object(manager, 'get_current_version', return_value='v2.1.0'), \
         patch.object(manager, 'get_latest_version', return_value=('v2.1.0', '2026-01-31')):
      result = manager.check_for_updates()
    assert result is True

  def test_update_available(self, manager):
    with patch.object(manager, 'get_current_version', return_value='v2.0.2'), \
         patch.object(manager, 'get_latest_version', return_value=('v2.1.0', '2026-01-31')):
      result = manager.check_for_updates()
    assert result is False

  def test_network_error(self, manager):
    with patch.object(manager, 'get_current_version', return_value='v2.0.2'), \
         patch.object(manager, 'get_latest_version', return_value=('', '')):
      result = manager.check_for_updates()
    assert result is False


class TestUpdateVersionParam:
  def test_writes_version(self, manager, tmp_path):
    # Override paths to use temp dir
    manager.PLUGIN_DATA_DIR = tmp_path / "data"
    manager.VERSION_PATH = tmp_path / "version"

    result = manager.update_version_param('v2.1.5')
    assert result is True
    assert (manager.PLUGIN_DATA_DIR / "MapdVersion").read_text() == 'v2.1.5'
    assert manager.VERSION_PATH.read_text() == 'v2.1.5'


class TestPerformUpdate:
  def test_skip_if_already_current(self, manager):
    with patch.object(manager, 'get_current_version', return_value='v2.0.5'), \
         patch.object(manager, 'get_latest_version', return_value=('v2.0.5', '')):
      result = manager.perform_update()
    assert result is True

  def test_abort_on_download_failure(self, manager):
    with patch.object(manager, 'get_current_version', return_value='v2.0.2'), \
         patch.object(manager, 'get_latest_version', return_value=('v2.1.0', '')), \
         patch.object(manager, 'backup_current_binary', return_value=True), \
         patch.object(manager, 'download_binary', return_value=None):
      result = manager.perform_update()
    assert result is False

  def test_double_digit_minor_is_clamped_to_the_pin(self, manager):
    # Lexicographically "v2.10.0" < "v2.3.0", so a string clamp would install
    # an unreviewed schema. Assert the DOWNLOAD target, not just the result.
    with patch.object(manager, 'get_current_version', return_value='v2.0.5'), \
         patch.object(manager, 'get_latest_version', return_value=('v2.10.0', '')), \
         patch.object(manager, 'backup_current_binary', return_value=True), \
         patch.object(manager, 'download_binary', return_value=Path('/tmp/mapd_temp')) as mock_dl, \
         patch.object(manager, 'stop_mapd', return_value=True), \
         patch.object(manager, 'replace_binary', return_value=True), \
         patch.object(manager, 'update_version_param', return_value=True), \
         patch.object(manager, 'start_mapd', return_value=True):
      result = manager.perform_update()

    assert result is True
    assert mock_dl.call_args[0][0] == manager.MAX_ALLOWED_VERSION

  def test_abort_on_no_latest(self, manager):
    with patch.object(manager, 'get_current_version', return_value='v2.0.2'), \
         patch.object(manager, 'get_latest_version', return_value=('', '')):
      result = manager.perform_update()
    assert result is False


class TestVersionTuple:
  def test_double_digit_minor_sorts_numerically(self, manager):
    # The bug this exists for: "v2.10.0" > "v2.3.0" is False as a string, so a
    # lexicographic clamp would let the first double-digit minor past the pin.
    assert manager._version_tuple('v2.10.0') > manager._version_tuple('v2.3.0')

  def test_strips_leading_v(self, manager):
    assert manager._version_tuple('v2.3.0') == (2, 3, 0)

  def test_non_numeric_component_does_not_raise(self, manager):
    assert manager._version_tuple('v2.3.0-rc1') == (2, 3, 0)


class TestEnsureBinary:
  def test_exists_and_current_returns_true_without_download(self, manager):
    with patch.object(Path, 'exists', return_value=True), \
         patch.object(manager, 'get_current_version', return_value=manager.MAX_ALLOWED_VERSION), \
         patch.object(manager, 'download_binary') as mock_dl:
      assert manager.ensure_binary() is True
    mock_dl.assert_not_called()

  def test_stale_binary_is_upgraded_to_the_pin(self, manager):
    # A device on v2.0.5 would otherwise run the OLD binary against the new
    # slot19 schema: highwayClass always empty, mapd_defaults.json unread.
    with patch.object(Path, 'exists', return_value=True), \
         patch.object(manager, 'get_current_version', return_value='v2.0.5'), \
         patch.object(manager, 'backup_current_binary', return_value=True) as mock_backup, \
         patch.object(manager, 'download_binary', return_value=Path('/tmp/mapd_temp')) as mock_dl, \
         patch.object(manager, 'replace_binary', return_value=True) as mock_replace, \
         patch.object(manager, 'update_version_param', return_value=True) as mock_param:
      assert manager.ensure_binary() is True

    mock_backup.assert_called_once()
    assert mock_dl.call_args[0][0] == manager.MAX_ALLOWED_VERSION
    mock_replace.assert_called_once_with(Path('/tmp/mapd_temp'))
    mock_param.assert_called_once_with(manager.MAX_ALLOWED_VERSION)

  def test_failed_upgrade_keeps_the_old_binary(self, manager):
    # GitHub unreachable at boot is the common case. An old mapd is better than
    # no mapd — mapd_runner's retry loop must not brick startup.
    with patch.object(Path, 'exists', return_value=True), \
         patch.object(manager, 'get_current_version', return_value='v2.0.5'), \
         patch.object(manager, 'backup_current_binary', return_value=True), \
         patch.object(manager, 'download_binary', return_value=None), \
         patch.object(manager, 'replace_binary') as mock_replace, \
         patch.object(manager, 'update_version_param') as mock_param:
      assert manager.ensure_binary() is True

    mock_replace.assert_not_called()
    mock_param.assert_not_called()   # the param must keep naming what is on disk

  def test_missing_downloads(self, manager):
    call_count = [0]

    def mock_exists(self_path=None):
      call_count[0] += 1
      return call_count[0] > 1  # First call False (check), rest True

    with patch.object(Path, 'exists', mock_exists), \
         patch.object(Path, 'mkdir'), \
         patch.object(manager, 'get_latest_version', return_value=('v2.1.0', '')), \
         patch.object(manager, 'download_binary', return_value=Path('/tmp/mapd_temp')), \
         patch('os.rename'), \
         patch.object(manager, 'update_version_param', return_value=True):
      result = manager.ensure_binary()
    assert result is True
