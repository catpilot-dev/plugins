"""Tests for LKA mode — two-stage disengagement event filter.

FULL (lat+long, DCC on) → LKA (lat only) on cancel/brake/DCC drop;
LKA → OFF on a cancel press whose rising edge occurs while already in LKA.
Brief: .superpowers/sdd/2026-08-14-bmw-lka-mode/lka-mode-brief.md
"""
import os
import sys
from enum import IntEnum
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

_PLUGIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PLUGIN_DIR not in sys.path:
  sys.path.insert(0, _PLUGIN_DIR)

from test_helpers import install_all_mocks


class EventName(IntEnum):
  """Subset of log.OnroadEvent.EventName used by lka_mode. Values are arbitrary
  (the filter only passes identities through); real cereal values differ."""
  pedalPressed = 1
  buttonCancel = 2
  doorOpen = 3
  steerSaturated = 4


class ButtonType(IntEnum):
  """Subset of car.CarState.ButtonEvent.Type."""
  cancel = 0
  resumeCruise = 1
  accelCruise = 2


@pytest.fixture(autouse=True)
def mock_deps(monkeypatch):
  install_all_mocks(monkeypatch)
  # lka_mode does `from cereal import car, log` — wire the enums it reads
  # onto the installed cereal mock so tests and filter share identities.
  cereal_mock = sys.modules['cereal']
  cereal_mock.log.OnroadEvent.EventName = EventName
  car_mock = MagicMock()
  car_mock.CarState.ButtonEvent.Type = ButtonType
  cereal_mock.car = car_mock
  monkeypatch.setitem(sys.modules, 'cereal.car', car_mock)


class FakeEvents:
  """Mirror of the openpilot Events attribute the filter mutates."""
  def __init__(self, names):
    self.events = list(names)


def make_cs(dcc_on=False, cancel_press=None):
  """cancel_press: True = rising edge event, False = release edge, None = no event."""
  btns = []
  if cancel_press is not None:
    btns.append(SimpleNamespace(type=ButtonType.cancel, pressed=cancel_press))
  return SimpleNamespace(
    cruiseState=SimpleNamespace(enabled=dcc_on),
    buttonEvents=btns,
  )


@pytest.fixture
def filt():
  import lka_mode
  return lka_mode.LkaModeFilter()


class TestNotEngaged:
  def test_no_filtering_when_disengaged(self, filt):
    """Stock NO_ENTRY behavior preserved: cancel press while off is untouched."""
    events = FakeEvents([EventName.buttonCancel, EventName.pedalPressed])
    filt.filter(events, make_cs(dcc_on=False, cancel_press=True), make_cs(), op_enabled=False)
    assert events.events == [EventName.buttonCancel, EventName.pedalPressed]


class TestFullMode:
  def test_brake_stripped(self, filt):
    """Brake while FULL must not disengage openpilot (DCC drops on its own → LKA)."""
    events = FakeEvents([EventName.pedalPressed])
    filt.filter(events, make_cs(dcc_on=True), make_cs(), op_enabled=True)
    assert events.events == []

  def test_cancel_press_stripped(self, filt):
    """Stage 1: cancel pressed while DCC on drops only DCC, openpilot stays."""
    events = FakeEvents([EventName.buttonCancel])
    filt.filter(events, make_cs(dcc_on=True, cancel_press=True), make_cs(), op_enabled=True)
    assert events.events == []

  def test_unrelated_events_kept(self, filt):
    events = FakeEvents([EventName.doorOpen, EventName.pedalPressed, EventName.steerSaturated])
    filt.filter(events, make_cs(dcc_on=True), make_cs(), op_enabled=True)
    assert events.events == [EventName.doorOpen, EventName.steerSaturated]


class TestLkaMode:
  def test_stage1_release_edge_does_not_cascade(self, filt):
    """buttonCancel fires again on release, after DCC already dropped — the press
    began in FULL, so the release edge must NOT fully disengage."""
    # press while FULL (stripped)
    filt.filter(FakeEvents([EventName.buttonCancel]),
                make_cs(dcc_on=True, cancel_press=True), make_cs(), op_enabled=True)
    # release arrives after DCC dropped
    events = FakeEvents([EventName.buttonCancel])
    filt.filter(events, make_cs(dcc_on=False, cancel_press=False), make_cs(), op_enabled=True)
    assert events.events == []

  def test_brake_stripped_in_lka(self, filt):
    events = FakeEvents([EventName.pedalPressed])
    filt.filter(events, make_cs(dcc_on=False), make_cs(), op_enabled=True)
    assert events.events == []

  def test_stage2_cancel_disengages(self, filt):
    """A new cancel press starting in LKA keeps buttonCancel → USER_DISABLE."""
    # enter LKA via stage-1 press + release
    filt.filter(FakeEvents([EventName.buttonCancel]),
                make_cs(dcc_on=True, cancel_press=True), make_cs(), op_enabled=True)
    filt.filter(FakeEvents([EventName.buttonCancel]),
                make_cs(dcc_on=False, cancel_press=False), make_cs(), op_enabled=True)
    # second, fresh press while in LKA
    events = FakeEvents([EventName.buttonCancel])
    filt.filter(events, make_cs(dcc_on=False, cancel_press=True), make_cs(), op_enabled=True)
    assert events.events == [EventName.buttonCancel]

  def test_reengage_resets_to_stage1(self, filt):
    """LKA → FULL via resume, then cancel is stage-1 again (stripped)."""
    # in LKA, no press
    filt.filter(FakeEvents([]), make_cs(dcc_on=False), make_cs(), op_enabled=True)
    # DCC re-engaged, cancel pressed → stage 1
    events = FakeEvents([EventName.buttonCancel])
    filt.filter(events, make_cs(dcc_on=True, cancel_press=True), make_cs(), op_enabled=True)
    assert events.events == []


class TestLkaBadgePredicate:
  @pytest.fixture(autouse=True)
  def bmw_ui_overlay(self, monkeypatch):
    """Load THIS plugin's ui_overlay by path — the flat module name collides
    with other plugins' ui_overlay in sys.modules when the whole suite runs."""
    for mod in ('pyray', 'fonts', 'openpilot.system.ui.lib.application',
                'openpilot.selfdrive.ui.ui_state'):
      monkeypatch.setitem(sys.modules, mod, MagicMock())
    import importlib.util
    spec = importlib.util.spec_from_file_location(
      'bmw_lka_test_ui_overlay', os.path.join(_PLUGIN_DIR, 'ui_overlay.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

  @staticmethod
  def make_ui_state(engaged, dcc_on):
    return SimpleNamespace(
      engaged=engaged,
      sm={'carState': SimpleNamespace(cruiseState=SimpleNamespace(enabled=dcc_on))},
    )

  def test_active_when_engaged_without_dcc(self, bmw_ui_overlay):
    assert bmw_ui_overlay.lka_active(self.make_ui_state(True, False)) is True

  def test_inactive_when_full_mode(self, bmw_ui_overlay):
    assert bmw_ui_overlay.lka_active(self.make_ui_state(True, True)) is False

  def test_inactive_when_disengaged(self, bmw_ui_overlay):
    assert bmw_ui_overlay.lka_active(self.make_ui_state(False, False)) is False

  def test_inactive_on_error(self, bmw_ui_overlay):
    assert bmw_ui_overlay.lka_active(SimpleNamespace(engaged=True, sm={})) is False


class TestHookCallback:
  def test_on_events_filter_returns_default(self):
    import lka_mode
    events = FakeEvents([EventName.pedalPressed])
    result = lka_mode.on_events_filter(None, events, make_cs(dcc_on=True), make_cs(), True)
    assert result is None
    assert events.events == []
