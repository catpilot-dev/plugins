"""Tests for bus_logger plugin — standalone, no openpilot deps."""
import importlib
import json
import os
import sys

import pytest


# ---------------------------------------------------------------------------
# Import bus_logger module directly (avoid package path ambiguity)
# ---------------------------------------------------------------------------

_mod_path = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "bus_logger.py"))
_spec = importlib.util.spec_from_file_location("_bus_logger_mod", _mod_path,
                                                 submodule_search_locations=[])
_bl = importlib.util.module_from_spec(_spec)
# Stub out cereal and openpilot imports that aren't available locally
from unittest.mock import MagicMock
for mod_name in ("cereal", "cereal.messaging",
                 "openpilot", "openpilot.common", "openpilot.common.realtime",
                 "openpilot.selfdrive", "openpilot.selfdrive.plugins",
                 "openpilot.selfdrive.plugins.plugin_bus"):
  sys.modules.setdefault(mod_name, MagicMock())
_spec.loader.exec_module(_bl)


# ---------------------------------------------------------------------------
# Test _discover_topics
# ---------------------------------------------------------------------------

class TestDiscoverTopics:
  def test_discovers_socket_files(self, tmp_path, monkeypatch):
    for name in ("bmw_temps", "speedlimit", "lane_centering_state"):
      (tmp_path / name).touch()
    monkeypatch.setattr(_bl, "BUS_DIR", str(tmp_path))
    topics = _bl._discover_topics()
    assert set(topics) == {"bmw_temps", "speedlimit", "lane_centering_state"}

  def test_ignores_dotfiles(self, tmp_path, monkeypatch):
    (tmp_path / ".hidden").touch()
    (tmp_path / "real_topic").touch()
    monkeypatch.setattr(_bl, "BUS_DIR", str(tmp_path))
    topics = _bl._discover_topics()
    assert topics == ["real_topic"]

  def test_returns_empty_when_dir_missing(self, monkeypatch):
    monkeypatch.setattr(_bl, "BUS_DIR", "/tmp/nonexistent_bus_dir_xyz")
    assert _bl._discover_topics() == []


# ---------------------------------------------------------------------------
# Test plugin.json validity
# ---------------------------------------------------------------------------

_PLUGIN_DIR = os.path.join(os.path.dirname(__file__), "..")


class TestPluginJson:
  @pytest.fixture(autouse=True)
  def load_json(self):
    with open(os.path.join(_PLUGIN_DIR, "plugin.json")) as f:
      self.cfg = json.load(f)

  def test_required_fields(self):
    for field in ("id", "name", "version", "type", "services", "processes", "cereal"):
      assert field in self.cfg, f"missing {field}"

  def test_id(self):
    assert self.cfg["id"] == "bus_logger"

  def test_type_is_hybrid(self):
    assert self.cfg["type"] == "hybrid"

  def test_service_definition(self):
    svc = self.cfg["services"]["pluginBusLog"]
    assert svc[0] is True   # should_log
    assert svc[1] == 5.0    # frequency
    assert svc[2] == 1      # decimation

  def test_cereal_slot(self):
    slot = self.cfg["cereal"]["slots"]["1"]
    assert slot["struct_name"] == "PluginBusLog"
    assert slot["event_field"] == "pluginBusLog"

  def test_process_only_onroad(self):
    proc = self.cfg["processes"][0]
    assert proc["condition"] == "only_onroad"
    assert proc["module"] == "bus_logger"


# ---------------------------------------------------------------------------
# Test capnp schema file exists and has expected structure
# ---------------------------------------------------------------------------

class TestCapnpSchema:
  def test_schema_file_exists(self):
    path = os.path.join(_PLUGIN_DIR, "cereal", "slot1.capnp")
    assert os.path.isfile(path)

  def test_schema_has_entries(self):
    path = os.path.join(_PLUGIN_DIR, "cereal", "slot1.capnp")
    with open(path) as f:
      content = f.read()
    assert "entries" in content
    assert "Entry" in content
    assert "topic" in content
    assert "json" in content
    assert "monoTime" in content
