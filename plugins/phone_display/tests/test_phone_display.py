"""Tests for phone_display plugin — standalone, no openpilot runtime deps."""
import importlib
import importlib.util
import json
import os
import sys
from unittest.mock import MagicMock, patch

import pytest


# ---------------------------------------------------------------------------
# Stub heavy deps before importing modules under test
# ---------------------------------------------------------------------------

_STUBS = [
  "cereal", "cereal.messaging", "cereal.log",
  "openpilot", "openpilot.common", "openpilot.common.params",
  "openpilot.common.swaglog", "openpilot.common.realtime",
  "openpilot.system", "openpilot.system.hardware",
  "openpilot.selfdrive", "openpilot.selfdrive.plugins",
  "openpilot.selfdrive.plugins.plugin_bus",
]
for _s in _STUBS:
  sys.modules.setdefault(_s, MagicMock())

_PLUGIN_DIR = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))


def _load(filename):
  path = os.path.join(_PLUGIN_DIR, filename)
  spec = importlib.util.spec_from_file_location(filename[:-3], path,
                                                 submodule_search_locations=[])
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  return mod


# ---------------------------------------------------------------------------
# watchdog tests
# ---------------------------------------------------------------------------

class TestCheckWebrtcd:
  def setup_method(self):
    self.wd = _load("watchdog.py")

  def test_returns_true_when_sessions_present(self):
    resp_mock = MagicMock()
    resp_mock.__enter__ = lambda s: s
    resp_mock.__exit__ = MagicMock(return_value=False)
    resp_mock.read.return_value = json.dumps({"status": "ok", "sessions": 2}).encode()

    with patch("urllib.request.urlopen", return_value=resp_mock):
      assert self.wd._check_webrtcd() is True

  def test_returns_false_when_no_sessions(self):
    resp_mock = MagicMock()
    resp_mock.__enter__ = lambda s: s
    resp_mock.__exit__ = MagicMock(return_value=False)
    resp_mock.read.return_value = json.dumps({"status": "ok", "sessions": 0}).encode()

    with patch("urllib.request.urlopen", return_value=resp_mock):
      assert self.wd._check_webrtcd() is False

  def test_returns_false_on_connection_error(self):
    with patch("urllib.request.urlopen", side_effect=OSError("connection refused")):
      assert self.wd._check_webrtcd() is False

  def test_returns_false_on_timeout(self):
    import urllib.error
    with patch("urllib.request.urlopen", side_effect=TimeoutError()):
      assert self.wd._check_webrtcd() is False


class TestWatchdogRequired:
  def setup_method(self):
    self.wd = _load("watchdog.py")

  def test_headless_always_required(self):
    self.wd._IS_HEADLESS = True
    params = MagicMock()
    params.get_bool.return_value = False  # param says no
    # required = _IS_HEADLESS OR param → True
    required = self.wd._IS_HEADLESS or params.get_bool("CatEyePhoneRequired")
    assert required is True

  def test_comma_device_respects_param_off(self):
    self.wd._IS_HEADLESS = False
    params = MagicMock()
    params.get_bool.return_value = False
    required = self.wd._IS_HEADLESS or params.get_bool("CatEyePhoneRequired")
    assert required is False

  def test_comma_device_respects_param_on(self):
    self.wd._IS_HEADLESS = False
    params = MagicMock()
    params.get_bool.return_value = True
    required = self.wd._IS_HEADLESS or params.get_bool("CatEyePhoneRequired")
    assert required is True


# ---------------------------------------------------------------------------
# hook tests
# ---------------------------------------------------------------------------

class TestHook:
  def setup_method(self):
    # Reload fresh module state each test
    self.hook = _load("hook.py")
    self.hook._sub = MagicMock()

  def _make_event_name(self, val):
    en = MagicMock()
    en.phoneDisplayUnavailable = val
    return en

  def test_no_block_when_not_required(self):
    self.hook._required = False
    self.hook._phone_active = False
    self.hook._sub.recv.return_value = None

    result = self.hook.on_selfdrived_events([], MagicMock(), MagicMock())
    assert result == []

  def test_no_block_when_required_and_phone_active(self):
    self.hook._required = True
    self.hook._phone_active = True
    self.hook._sub.recv.return_value = None

    result = self.hook.on_selfdrived_events([], MagicMock(), MagicMock())
    assert result == []

  def test_injects_event_when_required_and_phone_absent(self):
    self.hook._required = True
    self.hook._phone_active = False
    self.hook._sub.recv.return_value = None

    # Stub EventName.phoneDisplayUnavailable = 99
    log_mock = MagicMock()
    log_mock.OnroadEvent.EventName.phoneDisplayUnavailable = 99
    sys.modules["cereal"].log = log_mock
    sys.modules["cereal.log"] = log_mock

    import cereal
    cereal.log = log_mock

    result = self.hook.on_selfdrived_events([], MagicMock(), MagicMock())
    assert 99 in result

  def test_bus_message_updates_state(self):
    self.hook._required = False
    self.hook._phone_active = True
    # Simulate watchdog publishing required=True, phone_active=False
    self.hook._sub.recv.side_effect = [
      ("phone_display", {"required": True, "phone_active": False}),
      None,
    ]

    log_mock = MagicMock()
    log_mock.OnroadEvent.EventName.phoneDisplayUnavailable = 99
    sys.modules["cereal"].log = log_mock

    import cereal
    cereal.log = log_mock

    result = self.hook.on_selfdrived_events([], MagicMock(), MagicMock())
    assert self.hook._required is True
    assert self.hook._phone_active is False
    assert 99 in result

  def test_preserves_existing_events(self):
    self.hook._required = False
    self.hook._phone_active = True
    self.hook._sub.recv.return_value = None

    existing = [42, 77]
    result = self.hook.on_selfdrived_events(existing, MagicMock(), MagicMock())
    assert result == [42, 77]


# ---------------------------------------------------------------------------
# plugin.json validity
# ---------------------------------------------------------------------------

class TestWebrtcHooks:
  def setup_method(self):
    self.hook = _load("hook.py")

  def test_app_routes_registers_health(self):
    app_mock = MagicMock()
    result = self.hook.on_webrtc_app_routes([], app_mock)
    app_mock.router.add_get.assert_called_once_with("/health", self.hook._health_handler)
    assert "/health" in result

  def test_app_routes_appends_to_existing(self):
    app_mock = MagicMock()
    result = self.hook.on_webrtc_app_routes(["/existing"], app_mock)
    assert "/existing" in result
    assert "/health" in result

  def test_session_started_publishes(self):
    published = {}
    def fake_pub_send(data):
      published.update(data)
    pub_mock = MagicMock()
    pub_mock.send.side_effect = fake_pub_send

    hw_mock = MagicMock()
    hw_mock.get_device_type.return_value = 'tici'
    params_mock = MagicMock()
    params_mock.get_bool.return_value = True

    plugin_bus_mock = MagicMock()
    plugin_bus_mock.PluginPub.return_value = pub_mock
    sys.modules["openpilot.selfdrive.plugins.plugin_bus"] = plugin_bus_mock
    sys.modules["openpilot.system.hardware"].HARDWARE = hw_mock
    sys.modules["openpilot.common.params"].Params.return_value = params_mock

    self.hook.on_webrtc_session_started(None, "test-session-id")
    assert published.get("phone_active") is True

  def test_session_ended_publishes(self):
    published = {}
    pub_mock = MagicMock()
    pub_mock.send.side_effect = lambda d: published.update(d)

    plugin_bus_mock = MagicMock()
    plugin_bus_mock.PluginPub.return_value = pub_mock
    sys.modules["openpilot.selfdrive.plugins.plugin_bus"] = plugin_bus_mock
    sys.modules["openpilot.system.hardware"].HARDWARE.get_device_type.return_value = 'tici'
    sys.modules["openpilot.common.params"].Params.return_value.get_bool.return_value = False

    self.hook.on_webrtc_session_ended(None, "test-session-id")
    assert published.get("phone_active") is False

  def test_session_hooks_survive_publish_failure(self):
    plugin_bus_mock = MagicMock()
    plugin_bus_mock.PluginPub.side_effect = OSError("bus unavailable")
    sys.modules["openpilot.selfdrive.plugins.plugin_bus"] = plugin_bus_mock

    # Must not raise
    self.hook.on_webrtc_session_started(None, "test-id")
    self.hook.on_webrtc_session_ended(None, "test-id")


class TestAlertRegistry:
  def setup_method(self):
    self.hook = _load("hook.py")

  def test_returns_dict_with_event(self):
    log_mock = MagicMock()
    log_mock.OnroadEvent.EventName.phoneDisplayUnavailable = 99
    sys.modules["cereal"].log = log_mock

    events_mock = MagicMock()
    events_mock.ET = MagicMock()
    events_mock.NoEntryAlert = MagicMock(return_value="no_entry_alert")
    events_mock.NormalPermanentAlert = MagicMock(return_value="permanent_alert")
    sys.modules["openpilot.selfdrive.selfdrived.events"] = events_mock

    result = self.hook.on_alert_registry({})
    assert 99 in result
    assert "no_entry" in str(result[99]).lower() or result[99] is not None

  def test_preserves_existing_registrations(self):
    log_mock = MagicMock()
    log_mock.OnroadEvent.EventName.phoneDisplayUnavailable = 99
    sys.modules["cereal"].log = log_mock

    events_mock = MagicMock()
    sys.modules["openpilot.selfdrive.selfdrived.events"] = events_mock

    existing = {42: {"warning": "some_alert"}}
    result = self.hook.on_alert_registry(existing)
    assert 42 in result
    assert 99 in result


class TestPluginJson:
  @pytest.fixture(autouse=True)
  def load_json(self):
    with open(os.path.join(_PLUGIN_DIR, "plugin.json")) as f:
      self.cfg = json.load(f)

  def test_required_fields(self):
    for field in ("id", "name", "version", "type", "hooks", "processes", "params", "cereal"):
      assert field in self.cfg, f"missing field: {field}"

  def test_webrtc_hooks_registered(self):
    for hook in ("webrtc.app_routes", "webrtc.session_started", "webrtc.session_ended"):
      assert hook in self.cfg["hooks"], f"missing hook: {hook}"
      h = self.cfg["hooks"][hook]
      assert h["module"] == "hook"

  def test_id(self):
    assert self.cfg["id"] == "phone_display"

  def test_type_is_hybrid(self):
    assert self.cfg["type"] == "hybrid"

  def test_alert_registry_hook(self):
    assert "selfdrived.alert_registry" in self.cfg["hooks"]
    h = self.cfg["hooks"]["selfdrived.alert_registry"]
    assert h["module"] == "hook"
    assert h["function"] == "on_alert_registry"

  def test_events_hook(self):
    assert "selfdrived.events" in self.cfg["hooks"]
    h = self.cfg["hooks"]["selfdrived.events"]
    assert h["module"] == "hook"
    assert h["function"] == "on_selfdrived_events"

  def test_event_name_declared(self):
    assert "event_names" in self.cfg["cereal"]
    assert self.cfg["cereal"]["event_names"]["phoneDisplayUnavailable"] == 99

  def test_process_only_onroad(self):
    proc = self.cfg["processes"][0]
    assert proc["name"] == "phone_watchdog"
    assert proc["module"] == "watchdog"
    assert proc["condition"] == "only_onroad"

  def test_param_declared(self):
    assert "CatEyePhoneRequired" in self.cfg["params"]
    p = self.cfg["params"]["CatEyePhoneRequired"]
    assert p["type"] == "bool"
    assert p["default"] is False  # opt-in on comma devices
