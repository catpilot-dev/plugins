"""Tests for network_settings plugin — params, proxy env vars, github pinger, static IP."""
import os
import time
import pytest
from unittest.mock import MagicMock, patch
import sys
import importlib


@pytest.fixture(autouse=True)
def mock_openpilot(monkeypatch):
  """Mock openpilot + UI imports that aren't available in test env."""
  for mod in ['openpilot', 'openpilot.common', 'openpilot.common.params',
              'openpilot.common.swaglog',
              'openpilot.system', 'openpilot.system.ui', 'openpilot.system.ui.lib',
              'openpilot.system.ui.lib.application', 'openpilot.system.ui.lib.multilang',
              'openpilot.system.ui.lib.wifi_manager',
              'openpilot.system.ui.lib.networkmanager',
              'openpilot.system.ui.widgets',
              'openpilot.system.ui.widgets.keyboard', 'openpilot.system.ui.widgets.list_view',
              'openpilot.system.ui.widgets.network', 'openpilot.system.ui.widgets.button',
              'pyray', 'jeepney', 'jeepney.low_level', 'jeepney.wrappers']:
    monkeypatch.setitem(sys.modules, mod, MagicMock())


# ============================================================
# params_helper
# ============================================================

class TestParamsHelper:
  @pytest.fixture
  def params_dir(self, tmp_path):
    return tmp_path / "params" / "d"

  @pytest.fixture
  def ph(self, params_dir, monkeypatch):
    import plugins.network_settings.params_helper as mod
    importlib.reload(mod)
    monkeypatch.setattr(mod, 'PARAMS_DIR', params_dir)
    monkeypatch.setattr(mod, 'PERSIST_DIR', params_dir)
    params_dir.mkdir(parents=True)
    return mod

  def test_get_nonexistent(self, ph):
    assert ph.get("NoSuchKey") is None

  def test_put_and_get(self, ph):
    ph.put("TestKey", "hello")
    assert ph.get("TestKey") == "hello"

  def test_get_bool_true(self, ph):
    ph.put("Flag", "1")
    assert ph.get_bool("Flag") is True

  def test_get_bool_false(self, ph):
    ph.put("Flag", "0")
    assert ph.get_bool("Flag") is False

  def test_get_bool_missing(self, ph):
    assert ph.get_bool("Missing") is False

  def test_put_bool(self, ph):
    ph.put_bool("B", True)
    assert ph.get("B") == "1"
    ph.put_bool("B", False)
    assert ph.get("B") == "0"

  def test_remove(self, ph):
    ph.put("Gone", "value")
    assert ph.get("Gone") == "value"
    ph.remove("Gone")
    assert ph.get("Gone") is None

  def test_remove_nonexistent(self, ph):
    ph.remove("NeverExisted")

  def test_put_creates_dir(self, ph, params_dir, monkeypatch):
    import shutil
    shutil.rmtree(params_dir)
    assert not params_dir.exists()
    ph.put("AutoCreate", "works")
    assert ph.get("AutoCreate") == "works"


# ============================================================
# proxy env var management
# ============================================================

class TestProxyEnvVars:
  @pytest.fixture
  def proxy(self):
    import plugins.network_settings.proxy as mod
    importlib.reload(mod)
    return mod

  def test_apply_proxy_env(self, proxy):
    proxy.apply_proxy_env("socks5://1.2.3.4:1080")
    assert os.environ["ALL_PROXY"] == "socks5://1.2.3.4:1080"
    assert os.environ["HTTP_PROXY"] == "socks5://1.2.3.4:1080"
    assert os.environ["HTTPS_PROXY"] == "socks5://1.2.3.4:1080"
    assert "localhost" in os.environ["NO_PROXY"]

  def test_clear_proxy_env(self, proxy):
    proxy.apply_proxy_env("socks5://1.2.3.4:1080")
    proxy.clear_proxy_env()
    assert "ALL_PROXY" not in os.environ
    assert "HTTP_PROXY" not in os.environ
    assert "HTTPS_PROXY" not in os.environ
    assert "NO_PROXY" not in os.environ

  def test_clear_when_not_set(self, proxy):
    for key in ("ALL_PROXY", "HTTP_PROXY", "HTTPS_PROXY", "NO_PROXY"):
      os.environ.pop(key, None)
    proxy.clear_proxy_env()

  def test_on_startup_enabled(self, proxy, tmp_path, monkeypatch):
    import plugins.network_settings.params_helper as ph
    importlib.reload(ph)
    params_dir = tmp_path / "params" / "d"
    params_dir.mkdir(parents=True)
    monkeypatch.setattr(ph, 'PARAMS_DIR', params_dir)
    monkeypatch.setattr(ph, 'PERSIST_DIR', params_dir)
    monkeypatch.setattr(proxy, 'params_helper', ph)

    ph.put_bool("ProxyEnabled", True)
    ph.put("ProxyAddress", "socks5://10.0.0.1:9999")

    result = proxy.on_startup("default_val")
    assert result == "default_val"
    # on_startup is now a no-op — proxy setup deferred to github_pinger
    assert "ALL_PROXY" not in os.environ

  def test_on_startup_disabled(self, proxy, tmp_path, monkeypatch):
    import plugins.network_settings.params_helper as ph
    importlib.reload(ph)
    params_dir = tmp_path / "params" / "d"
    params_dir.mkdir(parents=True)
    monkeypatch.setattr(ph, 'PARAMS_DIR', params_dir)
    monkeypatch.setattr(ph, 'PERSIST_DIR', params_dir)
    monkeypatch.setattr(proxy, 'params_helper', ph)

    ph.put_bool("ProxyEnabled", False)
    os.environ.pop("ALL_PROXY", None)

    result = proxy.on_startup("default_val")
    assert result == "default_val"
    assert "ALL_PROXY" not in os.environ

  def test_on_startup_default_address(self, proxy, tmp_path, monkeypatch):
    import plugins.network_settings.params_helper as ph
    importlib.reload(ph)
    params_dir = tmp_path / "params" / "d"
    params_dir.mkdir(parents=True)
    monkeypatch.setattr(ph, 'PARAMS_DIR', params_dir)
    monkeypatch.setattr(ph, 'PERSIST_DIR', params_dir)
    monkeypatch.setattr(proxy, 'params_helper', ph)

    ph.put_bool("ProxyEnabled", True)
    proxy.on_startup("x")
    assert "ALL_PROXY" not in os.environ


# ============================================================
# github_pinger
# ============================================================

class TestGithubPinger:
  @pytest.fixture
  def pinger(self):
    import plugins.network_settings.github_pinger as mod
    importlib.reload(mod)
    return mod

  def test_check_github_success(self, pinger):
    with patch('plugins.network_settings.github_pinger.subprocess.run') as mock_run:
      mock_run.return_value = MagicMock(stdout="301")
      assert pinger.check_github() is True

  def test_check_github_200(self, pinger):
    with patch('plugins.network_settings.github_pinger.subprocess.run') as mock_run:
      mock_run.return_value = MagicMock(stdout="200")
      assert pinger.check_github() is True

  def test_check_github_failure(self, pinger):
    with patch('plugins.network_settings.github_pinger.subprocess.run') as mock_run:
      mock_run.return_value = MagicMock(stdout="000")
      assert pinger.check_github() is False

  def test_check_github_timeout(self, pinger):
    with patch('plugins.network_settings.github_pinger.subprocess.run') as mock_run:
      mock_run.side_effect = TimeoutError
      assert pinger.check_github() is False

  def test_check_github_curl_missing(self, pinger):
    with patch('plugins.network_settings.github_pinger.subprocess.run') as mock_run:
      mock_run.side_effect = FileNotFoundError
      assert pinger.check_github() is False

  def test_main_writes_param_on_success(self, pinger, tmp_path, monkeypatch):
    import plugins.network_settings.params_helper as ph
    importlib.reload(ph)
    params_dir = tmp_path / "params" / "d"
    params_dir.mkdir(parents=True)
    monkeypatch.setattr(ph, 'PARAMS_DIR', params_dir)
    monkeypatch.setattr(ph, 'PERSIST_DIR', params_dir)
    monkeypatch.setattr(pinger, 'params_helper', ph)

    call_count = 0
    def fake_sleep(t):
      nonlocal call_count
      call_count += 1
      if call_count >= 1:
        raise KeyboardInterrupt

    monkeypatch.setattr(time, 'sleep', fake_sleep)
    with patch.object(pinger, 'check_github', return_value=True):
      with pytest.raises(KeyboardInterrupt):
        pinger.main()

    val = ph.get("LastGithubPingTime")
    assert val is not None
    age_ns = time.monotonic_ns() - int(val)
    assert age_ns < 5_000_000_000  # < 5 seconds

  def test_main_removes_param_on_failure(self, pinger, tmp_path, monkeypatch):
    import plugins.network_settings.params_helper as ph
    importlib.reload(ph)
    params_dir = tmp_path / "params" / "d"
    params_dir.mkdir(parents=True)
    monkeypatch.setattr(ph, 'PARAMS_DIR', params_dir)
    monkeypatch.setattr(ph, 'PERSIST_DIR', params_dir)
    monkeypatch.setattr(pinger, 'params_helper', ph)

    ph.put("LastGithubPingTime", "123")

    call_count = 0
    def fake_sleep(t):
      nonlocal call_count
      call_count += 1
      if call_count >= 1:
        raise KeyboardInterrupt

    monkeypatch.setattr(time, 'sleep', fake_sleep)
    with patch.object(pinger, 'check_github', return_value=False):
      with pytest.raises(KeyboardInterrupt):
        pinger.main()

    assert ph.get("LastGithubPingTime") is None


# ============================================================
# is_github_connected (from ui.py, tested in isolation)
# ============================================================

class TestIsGithubConnected:
  def test_connected_recent(self, tmp_path, monkeypatch):
    import plugins.network_settings.params_helper as ph
    importlib.reload(ph)
    params_dir = tmp_path / "params" / "d"
    params_dir.mkdir(parents=True)
    monkeypatch.setattr(ph, 'PARAMS_DIR', params_dir)
    monkeypatch.setattr(ph, 'PERSIST_DIR', params_dir)

    ph.put("LastGithubPingTime", str(time.monotonic_ns()))

    GITHUB_TIMEOUT_NS = 160_000_000_000
    val = ph.get("LastGithubPingTime")
    assert val is not None
    assert (time.monotonic_ns() - int(val)) < GITHUB_TIMEOUT_NS

  def test_disconnected_stale(self, tmp_path, monkeypatch):
    import plugins.network_settings.params_helper as ph
    importlib.reload(ph)
    params_dir = tmp_path / "params" / "d"
    params_dir.mkdir(parents=True)
    monkeypatch.setattr(ph, 'PARAMS_DIR', params_dir)
    monkeypatch.setattr(ph, 'PERSIST_DIR', params_dir)

    ph.put("LastGithubPingTime", str(time.monotonic_ns() - 200_000_000_000))

    GITHUB_TIMEOUT_NS = 160_000_000_000
    val = ph.get("LastGithubPingTime")
    assert (time.monotonic_ns() - int(val)) >= GITHUB_TIMEOUT_NS

  def test_disconnected_missing(self, tmp_path, monkeypatch):
    import plugins.network_settings.params_helper as ph
    importlib.reload(ph)
    params_dir = tmp_path / "params" / "d"
    params_dir.mkdir(parents=True)
    monkeypatch.setattr(ph, 'PARAMS_DIR', params_dir)
    monkeypatch.setattr(ph, 'PERSIST_DIR', params_dir)

    assert ph.get("LastGithubPingTime") is None


# ============================================================
# plugin.json validation
# ============================================================

class TestPluginManifest:
  def test_valid_json(self):
    import json
    manifest_path = os.path.join(os.path.dirname(__file__), '..', 'plugin.json')
    with open(manifest_path) as f:
      manifest = json.load(f)

    assert manifest['id'] == 'network-settings'
    assert manifest['type'] == 'hybrid'
    assert 'manager.startup' in manifest['hooks']
    assert manifest['hooks']['manager.startup']['module'] == 'proxy'
    assert manifest['hooks']['manager.startup']['function'] == 'on_startup'

  def test_has_network_settings_extend_hook(self):
    import json
    manifest_path = os.path.join(os.path.dirname(__file__), '..', 'plugin.json')
    with open(manifest_path) as f:
      manifest = json.load(f)

    assert 'ui.network_settings_extend' in manifest['hooks']
    hook = manifest['hooks']['ui.network_settings_extend']
    assert hook['module'] == 'ui'
    assert hook['function'] == 'on_network_settings_extend'

  def test_no_connectivity_check_hook(self):
    """ui.connectivity_check is handled by sidebar, not this plugin."""
    import json
    manifest_path = os.path.join(os.path.dirname(__file__), '..', 'plugin.json')
    with open(manifest_path) as f:
      manifest = json.load(f)

    assert 'ui.connectivity_check' not in manifest['hooks']

  def test_has_process(self):
    import json
    manifest_path = os.path.join(os.path.dirname(__file__), '..', 'plugin.json')
    with open(manifest_path) as f:
      manifest = json.load(f)

    processes = manifest.get('processes', [])
    assert len(processes) == 1
    assert processes[0]['name'] == 'github_pinger'
    assert processes[0]['module'] == 'github_pinger'
    assert processes[0]['condition'] == 'always_run'

  def test_no_params_in_manifest(self):
    """Params stored in plugin data dir, not in manifest (openpilot clearAll wipes /data/params/d/)."""
    import json
    manifest_path = os.path.join(os.path.dirname(__file__), '..', 'plugin.json')
    with open(manifest_path) as f:
      manifest = json.load(f)

    assert 'params' not in manifest


# ============================================================
# Sidebar github connectivity integration
# ============================================================

class TestSidebarGithubConnectivity:
  SIDEBAR_TIMEOUT_NS = 80_000_000_000

  def test_recent_ping_is_connected(self):
    last_ping = time.monotonic_ns()
    assert (time.monotonic_ns() - last_ping) < self.SIDEBAR_TIMEOUT_NS

  def test_stale_ping_is_disconnected(self):
    last_ping = time.monotonic_ns() - 100_000_000_000
    assert (time.monotonic_ns() - last_ping) >= self.SIDEBAR_TIMEOUT_NS

  def test_file_roundtrip(self, tmp_path):
    param_file = tmp_path / "LastGithubPingTime"
    ts = str(time.monotonic_ns())
    param_file.write_text(ts)
    val = param_file.read_text()
    assert (time.monotonic_ns() - int(val)) < self.SIDEBAR_TIMEOUT_NS

  def test_missing_file_raises(self, tmp_path):
    param_file = tmp_path / "LastGithubPingTime"
    with pytest.raises(FileNotFoundError):
      param_file.read_text()


# ============================================================
# netmask_to_prefix (from static_ip.py)
# ============================================================

class TestNetmaskToPrefix:
  @pytest.fixture
  def static_ip(self):
    import plugins.network_settings.static_ip as mod
    importlib.reload(mod)
    return mod

  def test_class_c(self, static_ip):
    assert static_ip.netmask_to_prefix("255.255.255.0") == 24

  def test_class_b(self, static_ip):
    assert static_ip.netmask_to_prefix("255.255.0.0") == 16

  def test_class_a(self, static_ip):
    assert static_ip.netmask_to_prefix("255.0.0.0") == 8

  def test_host_mask(self, static_ip):
    assert static_ip.netmask_to_prefix("255.255.255.255") == 32

  def test_slash_25(self, static_ip):
    assert static_ip.netmask_to_prefix("255.255.255.128") == 25

  def test_slash_20(self, static_ip):
    assert static_ip.netmask_to_prefix("255.255.240.0") == 20

  def test_invalid_format_raises(self, static_ip):
    with pytest.raises(ValueError):
      static_ip.netmask_to_prefix("255.255")


# ============================================================
# Static IP params persistence
# ============================================================

class TestStaticIPPerSSID:
  @pytest.fixture
  def ph(self, tmp_path, monkeypatch):
    import plugins.network_settings.params_helper as mod
    importlib.reload(mod)
    params_dir = tmp_path / "params" / "d"
    params_dir.mkdir(parents=True)
    monkeypatch.setattr(mod, 'PARAMS_DIR', params_dir)
    monkeypatch.setattr(mod, 'PERSIST_DIR', params_dir)
    return mod

  def test_ssid_in_networks_means_enabled(self, ph):
    import json
    networks = {"BluesHome_AX": {"ip": "10.0.0.161", "gw": "10.0.0.244"}}
    ph.put("StaticIPNetworks", json.dumps(networks))
    loaded = json.loads(ph.get("StaticIPNetworks"))
    assert "BluesHome_AX" in loaded
    assert "13mini" not in loaded

  def test_networks_json_roundtrip(self, ph):
    import json
    networks = {"BluesHome_AX": {"ip": "10.0.0.161", "gw": "10.0.0.244"},
                "13mini": {"ip": "172.20.10.8", "gw": "172.20.10.1"}}
    ph.put("StaticIPNetworks", json.dumps(networks))
    loaded = json.loads(ph.get("StaticIPNetworks"))
    assert loaded["BluesHome_AX"]["ip"] == "10.0.0.161"
    assert loaded["BluesHome_AX"]["gw"] == "10.0.0.244"
    assert loaded["13mini"]["ip"] == "172.20.10.8"

  def test_empty_networks_default(self, ph):
    assert ph.get("StaticIPNetworks") is None

  def test_ssid_lookup_with_defaults(self, ph):
    networks = {"BluesHome_AX": {"ip": "10.0.0.161", "gw": "10.0.0.244"}}
    cfg = networks.get("BluesHome_AX", {})
    assert cfg.get("ip", "172.20.10.8") == "10.0.0.161"
    cfg = networks.get("13mini", {})
    assert cfg.get("ip", "172.20.10.8") == "172.20.10.8"
    assert cfg.get("gw", "172.20.10.1") == "172.20.10.1"

  def test_save_updates_existing(self, ph):
    import json
    networks = {"BluesHome_AX": {"ip": "10.0.0.161", "gw": "10.0.0.244"}}
    networks["BluesHome_AX"]["ip"] = "10.0.0.162"
    ph.put("StaticIPNetworks", json.dumps(networks))
    loaded = json.loads(ph.get("StaticIPNetworks"))
    assert loaded["BluesHome_AX"]["ip"] == "10.0.0.162"
    assert loaded["BluesHome_AX"]["gw"] == "10.0.0.244"

  def test_add_new_ssid(self, ph):
    import json
    networks = {"BluesHome_AX": {"ip": "10.0.0.161", "gw": "10.0.0.244"}}
    networks["13mini"] = {"ip": "172.20.10.5", "gw": "172.20.10.1"}
    ph.put("StaticIPNetworks", json.dumps(networks))
    loaded = json.loads(ph.get("StaticIPNetworks"))
    assert len(loaded) == 2
    assert loaded["13mini"]["ip"] == "172.20.10.5"


# ============================================================
# plugin.json — static IP params validation
# ============================================================

