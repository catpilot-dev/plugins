"""Tests for screen_capture — standalone, no openpilot deps.

Guards the onroad bookmark path. The plugin runs inside the UI process, which
already owns the only bookmarkButton publisher; opening a second one is
accepted silently by msgq and then makes the first publisher's next send()
raise MultiplePublishersError, killing the UI mid-drive (observed 2026-08-11:
black screen + TAKE CONTROL IMMEDIATELY on route 000003ef seg 24).
"""
import importlib.util
import os
import sys
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

_mod_path = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "capture.py"))

# capture.py needs pyray/config at import time. Stub them, then put sys.modules
# back exactly as it was — leaving stubs in place breaks every later test
# module in the session that imports the real ones (speedlimitd does).
_STUBBED = ("pyray", "config",
            "openpilot", "openpilot.system", "openpilot.system.ui",
            "openpilot.system.ui.lib", "openpilot.system.ui.lib.application")
_saved = {name: sys.modules.get(name) for name in _STUBBED}
for name in _STUBBED:
  sys.modules[name] = MagicMock()
sys.modules["config"].MEDIA_DIR = "/tmp"

_spec = importlib.util.spec_from_file_location("_screen_capture_mod", _mod_path,
                                               submodule_search_locations=[])
_cap = importlib.util.module_from_spec(_spec)
try:
  _spec.loader.exec_module(_cap)
finally:
  for name, original in _saved.items():
    if original is None:
      sys.modules.pop(name, None)
    else:
      sys.modules[name] = original


@pytest.fixture
def gui_app(monkeypatch):
  """Stub gui_app whose nav stack the plugin searches."""
  app = SimpleNamespace(_nav_stack=[])
  monkeypatch.setitem(sys.modules, "openpilot.system.ui.lib.application",
                      SimpleNamespace(gui_app=app))
  return app


def test_uses_the_uis_own_bookmark_action(gui_app):
  """The tap must drive the stock handler, never open its own publisher."""
  calls = []
  gui_app._nav_stack = [
    SimpleNamespace(),                                        # unrelated widget
    SimpleNamespace(_on_bookmark_clicked=lambda: calls.append(1)),  # MainLayout
  ]
  assert _cap._send_bookmark() is True
  assert calls == [1]


def test_never_creates_a_publisher():
  """No PubMaster anywhere in the module — that is the whole point."""
  src = open(_mod_path).read()
  assert "PubMaster" not in src
  assert "bookmarkButton'" not in src and 'bookmarkButton"' not in src


def test_reports_failure_when_no_handler_found(gui_app):
  gui_app._nav_stack = [SimpleNamespace(), SimpleNamespace()]
  assert _cap._send_bookmark() is False


def test_survives_a_broken_handler(gui_app):
  """A raising handler must never propagate into the render loop."""
  def boom():
    raise RuntimeError("messaging failure")
  gui_app._nav_stack = [SimpleNamespace(_on_bookmark_clicked=boom)]
  assert _cap._send_bookmark() is False


def test_offroad_tap_still_saves_a_png(monkeypatch):
  """Offroad behaviour is unchanged: capture locally, no bookmark."""
  saved, booked = [], []
  monkeypatch.setattr(_cap, "_is_onroad", lambda: False)
  monkeypatch.setattr(_cap, "_save_png", lambda: saved.append(1))
  monkeypatch.setattr(_cap, "_send_bookmark", lambda: booked.append(1))
  _cap._capture_pending = True
  _cap.on_post_end_drawing(None)
  assert saved == [1] and booked == []


def test_onroad_tap_bookmarks_without_touching_the_gpu(monkeypatch):
  saved, booked = [], []
  monkeypatch.setattr(_cap, "_is_onroad", lambda: True)
  monkeypatch.setattr(_cap, "_save_png", lambda: saved.append(1))
  monkeypatch.setattr(_cap, "_send_bookmark", lambda: booked.append(1))
  _cap._capture_pending = True
  _cap.on_post_end_drawing(None)
  assert booked == [1] and saved == []
