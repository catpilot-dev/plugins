"""Tests for phone_gps plugin — standalone, no openpilot runtime deps."""
import asyncio
import importlib
import importlib.util
import json
import os
import sys
from types import SimpleNamespace
from unittest.mock import AsyncMock, MagicMock, patch

import pytest


# ---------------------------------------------------------------------------
# Stub heavy deps before importing modules under test
# ---------------------------------------------------------------------------

_STUBS = [
  "cereal", "cereal.messaging",
  "openpilot", "openpilot.common", "openpilot.common.swaglog",
]
for _s in _STUBS:
  sys.modules.setdefault(_s, MagicMock())

# Stub aiohttp so tests run in environments that don't have it installed
# (e.g. the pre-push hook venv).  WSMsgType values must be distinct so the
# type comparisons in the handler work correctly.
try:
  import aiohttp as _aiohttp_real  # noqa: F401 — use real package if present
except ModuleNotFoundError:
  import enum

  class _WSMsgType(enum.IntEnum):
    TEXT   = 1
    BINARY = 2
    PING   = 9
    PONG   = 10
    CLOSE  = 8
    ERROR  = 258  # aiohttp sentinel value

  _aiohttp_web_mod = MagicMock()
  _aiohttp_web_mod.WebSocketResponse = MagicMock
  _aiohttp_web_mod.WSMsgType = _WSMsgType

  _aiohttp_mod = MagicMock()
  _aiohttp_mod.WSMsgType = _WSMsgType
  _aiohttp_mod.web = _aiohttp_web_mod

  sys.modules.setdefault("aiohttp", _aiohttp_mod)
  sys.modules.setdefault("aiohttp.web", _aiohttp_web_mod)

_PLUGIN_DIR = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))


def _load():
  """Load hook.py fresh (isolates module-level state between tests)."""
  path = os.path.join(_PLUGIN_DIR, "hook.py")
  spec = importlib.util.spec_from_file_location("hook", path,
                                                 submodule_search_locations=[])
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  return mod


def _reset_cereal_messaging():
  """Reset the shared cereal.messaging mock and return it.

  ``import cereal.messaging as messaging`` compiles to IMPORT_NAME + IMPORT_FROM.
  IMPORT_FROM does ``getattr(cereal_module, 'messaging')``, NOT a sys.modules
  lookup.  Python only syncs that attribute when it actually loads the submodule;
  since both cereal and cereal.messaging are MagicMock stubs from the start,
  the attribute on the parent is an auto-created child disconnected from
  sys.modules["cereal.messaging"].

  We fix this by explicitly writing the sys.modules entry onto the parent mock
  so that every import path — sys.modules dict AND parent-attribute traversal —
  resolves to the same object.
  """
  m = sys.modules["cereal.messaging"]
  m.reset_mock()
  # Sync parent attribute so IMPORT_FROM resolves to the same mock.
  sys.modules["cereal"].messaging = m
  return m


class _AsyncIter:
  """Wraps a plain list as an async iterator for mocking aiohttp WS iteration."""
  def __init__(self, items):
    self._iter = iter(items)

  def __aiter__(self):
    return self

  async def __anext__(self):
    try:
      return next(self._iter)
    except StopIteration:
      raise StopAsyncIteration


def _make_ws_message(text="", msg_type=None):
  from aiohttp import WSMsgType
  m = MagicMock()
  m.type = msg_type if msg_type is not None else WSMsgType.TEXT
  m.data = text
  return m


def _make_request(messages):
  """Build a mock aiohttp Request + WebSocketResponse for handler tests."""
  # Do NOT use spec= here: when aiohttp is stubbed, web.WebSocketResponse is
  # itself a MagicMock, and spec=MagicMock blocks magic attribute assignment.
  ws_mock = MagicMock()
  ws_mock.prepare = AsyncMock()
  ws_mock.__aiter__ = MagicMock(return_value=_AsyncIter(messages))
  request_mock = MagicMock()

  def ws_factory(*a, **kw):
    return ws_mock

  return request_mock, ws_mock, ws_factory


# ---------------------------------------------------------------------------
# _publish_gps — field mapping
# ---------------------------------------------------------------------------

class TestPublishGps:
  def setup_method(self):
    self.hook = _load()
    self.cereal_msg = _reset_cereal_messaging()

    # Use SimpleNamespace so attribute assignments produce real Python values.
    self.fix = SimpleNamespace()
    msg = MagicMock()
    msg.gpsLocationExternal = self.fix
    self.cereal_msg.new_message.return_value = msg

    self.pm = MagicMock()

  def _publish(self, data):
    self.hook._publish_gps(self.pm, data)

  def test_complete_fix_all_fields(self):
    self._publish({
      "latitude": 37.7749, "longitude": -122.4194, "altitude": 15.5,
      "speed": 13.8, "heading": 270.0,
      "accuracy": 4.2, "altitudeAccuracy": 8.1,
      "timestamp": 1700000000000,
    })
    assert self.fix.latitude == 37.7749
    assert self.fix.longitude == -122.4194
    assert self.fix.altitude == 15.5
    assert self.fix.speed == 13.8
    assert self.fix.bearingDeg == 270.0
    assert self.fix.horizontalAccuracy == 4.2
    assert self.fix.verticalAccuracy == 8.1
    assert self.fix.unixTimestampMillis == 1700000000000

  def test_nullable_fields_default_to_zero(self):
    """altitude, speed, heading, altitudeAccuracy are None on some devices."""
    self._publish({
      "latitude": 51.5, "longitude": -0.12,
      "altitude": None, "speed": None, "heading": None, "altitudeAccuracy": None,
      "accuracy": 10.0, "timestamp": 0,
    })
    assert self.fix.altitude == 0.0
    assert self.fix.speed == 0.0
    assert self.fix.bearingDeg == 0.0
    assert self.fix.verticalAccuracy == 0.0

  def test_missing_accuracy_defaults_to_100(self):
    self._publish({"latitude": 0, "longitude": 0, "timestamp": 0})
    assert self.fix.horizontalAccuracy == 100.0

  def test_has_fix_always_true(self):
    """Browser only fires watchPosition callback when it has a position."""
    self._publish({"latitude": 0, "longitude": 0, "timestamp": 0})
    assert self.fix.hasFix is True

  def test_source_is_external(self):
    """SensorSource.external == 5 in log.capnp GpsLocationData enum."""
    self._publish({"latitude": 0, "longitude": 0, "timestamp": 0})
    assert self.fix.source == 5

  def test_pm_send_called_with_correct_topic(self):
    self._publish({"latitude": 0, "longitude": 0, "timestamp": 0})
    self.pm.send.assert_called_once()
    topic, _ = self.pm.send.call_args.args
    assert topic == "gpsLocationExternal"

  def test_new_message_called_with_correct_topic(self):
    self._publish({"latitude": 0, "longitude": 0, "timestamp": 0})
    self.cereal_msg.new_message.assert_called_with("gpsLocationExternal")

  def test_negative_coordinates(self):
    self._publish({"latitude": -33.87, "longitude": -70.65, "timestamp": 0})
    assert self.fix.latitude == -33.87
    assert self.fix.longitude == -70.65

  def test_timestamp_int_conversion(self):
    """Float timestamps from some browsers must be stored as int."""
    self._publish({"latitude": 0, "longitude": 0, "timestamp": 1700000000123.7})
    assert isinstance(self.fix.unixTimestampMillis, int)
    assert self.fix.unixTimestampMillis == 1700000000123

  def test_missing_all_optional_fields(self):
    """Minimal payload — only required latitude/longitude/timestamp."""
    self._publish({"latitude": 48.85, "longitude": 2.35, "timestamp": 42})
    assert self.fix.latitude == 48.85
    assert self.fix.longitude == 2.35
    assert self.fix.unixTimestampMillis == 42
    assert self.fix.altitude == 0.0
    assert self.fix.hasFix is True


# ---------------------------------------------------------------------------
# on_webrtc_app_routes
# ---------------------------------------------------------------------------

class TestHook:
  def setup_method(self):
    self.hook = _load()

  def test_registers_ws_gps_route(self):
    app = MagicMock()
    self.hook.on_webrtc_app_routes([], app)
    app.router.add_get.assert_called_once_with("/ws/gps", self.hook._gps_ws_handler)

  def test_returns_ws_gps_in_routes(self):
    app = MagicMock()
    result = self.hook.on_webrtc_app_routes([], app)
    assert "/ws/gps" in result

  def test_appends_to_existing_routes(self):
    app = MagicMock()
    result = self.hook.on_webrtc_app_routes(["/health", "/hud"], app)
    assert "/health" in result
    assert "/hud" in result
    assert "/ws/gps" in result

  def test_returns_list(self):
    app = MagicMock()
    result = self.hook.on_webrtc_app_routes([], app)
    assert isinstance(result, list)

  def test_original_routes_unchanged(self):
    """Hook must not mutate the input list."""
    app = MagicMock()
    original = ["/health"]
    self.hook.on_webrtc_app_routes(original, app)
    assert original == ["/health"]


# ---------------------------------------------------------------------------
# _gps_ws_handler — async WebSocket handler
# ---------------------------------------------------------------------------

class TestGpsWsHandler:
  def setup_method(self):
    self.hook = _load()
    self.cereal_msg = _reset_cereal_messaging()
    self.pm = MagicMock()
    self.cereal_msg.PubMaster.return_value = self.pm

    fix = SimpleNamespace()
    msg = MagicMock()
    msg.gpsLocationExternal = fix
    self.cereal_msg.new_message.return_value = msg

  def _run(self, coro):
    return asyncio.run(coro)

  def test_valid_gps_message_publishes(self):
    from aiohttp import WSMsgType
    payload = json.dumps({
      "latitude": 37.77, "longitude": -122.41,
      "accuracy": 5.0, "timestamp": 1700000000000,
    })
    messages = [_make_ws_message(payload, WSMsgType.TEXT)]
    request, ws, ws_factory = _make_request(messages)

    with patch("aiohttp.web.WebSocketResponse", ws_factory):
      self._run(self.hook._gps_ws_handler(request))

    self.pm.send.assert_called_once()
    topic, _ = self.pm.send.call_args.args
    assert topic == "gpsLocationExternal"

  def test_malformed_json_does_not_crash(self):
    from aiohttp import WSMsgType
    messages = [_make_ws_message("not valid json", WSMsgType.TEXT)]
    request, ws, ws_factory = _make_request(messages)

    with patch("aiohttp.web.WebSocketResponse", ws_factory):
      self._run(self.hook._gps_ws_handler(request))  # must not raise

    self.pm.send.assert_not_called()

  def test_multiple_fixes_published_in_order(self):
    from aiohttp import WSMsgType
    payloads = [
      json.dumps({"latitude": 37.0 + i, "longitude": -122.0, "accuracy": 5.0, "timestamp": i})
      for i in range(3)
    ]
    messages = [_make_ws_message(p, WSMsgType.TEXT) for p in payloads]
    request, ws, ws_factory = _make_request(messages)

    with patch("aiohttp.web.WebSocketResponse", ws_factory):
      self._run(self.hook._gps_ws_handler(request))

    assert self.pm.send.call_count == 3

  def test_pm_closed_on_normal_exit(self):
    from aiohttp import WSMsgType
    messages = [_make_ws_message("{}", WSMsgType.TEXT)]
    request, ws, ws_factory = _make_request(messages)

    with patch("aiohttp.web.WebSocketResponse", ws_factory):
      self._run(self.hook._gps_ws_handler(request))

    self.pm.close.assert_called_once()

  def test_pm_closed_on_ws_error(self):
    from aiohttp import WSMsgType
    messages = [_make_ws_message("", WSMsgType.ERROR)]
    request, ws, ws_factory = _make_request(messages)

    with patch("aiohttp.web.WebSocketResponse", ws_factory):
      self._run(self.hook._gps_ws_handler(request))

    self.pm.close.assert_called_once()

  def test_pm_closed_on_ws_close(self):
    from aiohttp import WSMsgType
    messages = [_make_ws_message("", WSMsgType.CLOSE)]
    request, ws, ws_factory = _make_request(messages)

    with patch("aiohttp.web.WebSocketResponse", ws_factory):
      self._run(self.hook._gps_ws_handler(request))

    self.pm.close.assert_called_once()

  def test_pm_closed_after_mixed_valid_and_malformed(self):
    from aiohttp import WSMsgType
    messages = [
      _make_ws_message('{"latitude":1,"longitude":2,"accuracy":5,"timestamp":0}', WSMsgType.TEXT),
      _make_ws_message("bad json", WSMsgType.TEXT),
      _make_ws_message('{"latitude":3,"longitude":4,"accuracy":5,"timestamp":1}', WSMsgType.TEXT),
    ]
    request, ws, ws_factory = _make_request(messages)

    with patch("aiohttp.web.WebSocketResponse", ws_factory):
      self._run(self.hook._gps_ws_handler(request))

    assert self.pm.send.call_count == 2
    self.pm.close.assert_called_once()

  def test_survives_pubmaster_creation_failure(self):
    from aiohttp import WSMsgType
    self.cereal_msg.PubMaster.side_effect = OSError("zmq unavailable")

    messages = [_make_ws_message("{}", WSMsgType.TEXT)]
    request, ws, ws_factory = _make_request(messages)

    with patch("aiohttp.web.WebSocketResponse", ws_factory):
      self._run(self.hook._gps_ws_handler(request))  # must not raise

  def test_returns_ws_response(self):
    messages = []
    request, ws, ws_factory = _make_request(messages)

    with patch("aiohttp.web.WebSocketResponse", ws_factory):
      result = self._run(self.hook._gps_ws_handler(request))

    assert result is ws


# ---------------------------------------------------------------------------
# plugin.json validity
# ---------------------------------------------------------------------------

class TestPluginJson:
  @pytest.fixture(autouse=True)
  def load_json(self):
    with open(os.path.join(_PLUGIN_DIR, "plugin.json")) as f:
      self.cfg = json.load(f)

  def test_required_fields(self):
    for field in ("id", "name", "version", "type", "hooks"):
      assert field in self.cfg, f"missing field: {field}"

  def test_id(self):
    assert self.cfg["id"] == "phone_gps"

  def test_type_is_hook(self):
    assert self.cfg["type"] == "hook"

  def test_webrtc_app_routes_hook_declared(self):
    assert "webrtc.app_routes" in self.cfg["hooks"]
    h = self.cfg["hooks"]["webrtc.app_routes"]
    assert h["module"] == "hook"
    assert h["function"] == "on_webrtc_app_routes"

  def test_no_processes(self):
    """phone_gps is hook-only — no background processes."""
    assert "processes" not in self.cfg or self.cfg["processes"] == []

  def test_no_cereal_events(self):
    """phone_gps uses the existing gpsLocationExternal topic — no custom events."""
    assert "cereal" not in self.cfg or "event_names" not in self.cfg.get("cereal", {})
