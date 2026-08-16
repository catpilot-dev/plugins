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
  wrongGear = 5
  pcmDisable = 6


class ButtonType(IntEnum):
  """Subset of car.CarState.ButtonEvent.Type."""
  cancel = 0
  resumeCruise = 1
  accelCruise = 2


class GearShifter(IntEnum):
  """Subset of car.CarState.GearShifter."""
  unknown = 0
  park = 1
  drive = 2
  neutral = 3
  reverse = 4


@pytest.fixture(autouse=True)
def mock_deps(monkeypatch):
  install_all_mocks(monkeypatch)
  # lka_mode does `from cereal import car, log` — wire the enums it reads
  # onto the installed cereal mock so tests and filter share identities.
  cereal_mock = sys.modules['cereal']
  cereal_mock.log.OnroadEvent.EventName = EventName
  car_mock = MagicMock()
  car_mock.CarState.ButtonEvent.Type = ButtonType
  car_mock.CarState.GearShifter = GearShifter
  cereal_mock.car = car_mock
  monkeypatch.setitem(sys.modules, 'cereal.car', car_mock)


class FakeEvents:
  """Mirror of the openpilot Events API the filter uses (events list + add)."""
  def __init__(self, names):
    self.events = list(names)

  def add(self, name):
    self.events.append(name)


def make_cs(dcc_on=False, cancel_press=None, gear=GearShifter.drive):
  """cancel_press: True = rising edge event, False = release edge, None = no event."""
  btns = []
  if cancel_press is not None:
    btns.append(SimpleNamespace(type=ButtonType.cancel, pressed=cancel_press))
  return SimpleNamespace(
    cruiseState=SimpleNamespace(enabled=dcc_on),
    buttonEvents=btns,
    gearShifter=gear,
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


class TestGearDisengage:
  """Route 3fb seg 2: Neutral in LKA triggered wrongGear soft-disable (loud
  'Gear not D' countdown). User ruling 2026-08-16: only Drive permits
  engagement — any other definite gear disengages directly, FULL or LKA.
  `unknown` (signal glitch) falls back to stock soft-disable."""

  @pytest.mark.parametrize("gear", [GearShifter.neutral, GearShifter.reverse,
                                    GearShifter.park])
  @pytest.mark.parametrize("dcc_on", [False, True])
  def test_non_drive_gear_disengages_quietly(self, filt, gear, dcc_on):
    events = FakeEvents([EventName.wrongGear])
    filt.filter(events, make_cs(dcc_on=dcc_on, gear=gear),
                make_cs(), op_enabled=True)
    assert EventName.pcmDisable in events.events
    assert EventName.wrongGear not in events.events

  def test_drive_gear_no_disengage(self, filt):
    events = FakeEvents([])
    filt.filter(events, make_cs(dcc_on=False, gear=GearShifter.drive),
                make_cs(), op_enabled=True)
    assert EventName.pcmDisable not in events.events

  def test_unknown_gear_falls_back_to_stock(self, filt):
    """A transient CAN glitch must not instantly disengage — stock wrongGear
    soft-disable handles it with its countdown."""
    events = FakeEvents([EventName.wrongGear])
    filt.filter(events, make_cs(dcc_on=False, gear=GearShifter.unknown),
                make_cs(), op_enabled=True)
    assert EventName.pcmDisable not in events.events
    assert EventName.wrongGear in events.events

  def test_neutral_while_disengaged_untouched(self, filt):
    events = FakeEvents([EventName.wrongGear])
    filt.filter(events, make_cs(dcc_on=False, gear=GearShifter.neutral),
                make_cs(), op_enabled=False)
    assert events.events == [EventName.wrongGear]


class TestLkaUiStatus:
  """LKA border shows the override grey: ui.state_tick sets UIStatus.OVERRIDE."""

  @pytest.fixture(autouse=True)
  def mock_ui_deps(self, monkeypatch):
    for mod in ('pyray', 'fonts', 'openpilot.system.ui.lib.application'):
      monkeypatch.setitem(sys.modules, mod, MagicMock())
    self.ui_status = SimpleNamespace(ENGAGED='engaged', OVERRIDE='override',
                                     DISENGAGED='disengaged')
    self.ui_state = SimpleNamespace(
      engaged=True,
      status=self.ui_status.ENGAGED,
      sm={'carState': SimpleNamespace(cruiseState=SimpleNamespace(enabled=False))},
    )
    mod = MagicMock()
    mod.ui_state = self.ui_state
    mod.UIStatus = self.ui_status
    monkeypatch.setitem(sys.modules, 'openpilot.selfdrive.ui.ui_state', mod)
    import importlib.util
    spec = importlib.util.spec_from_file_location(
      'bmw_lka_test_ui_overlay2', os.path.join(_PLUGIN_DIR, 'ui_overlay.py'))
    self.overlay = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(self.overlay)

  def test_lka_sets_override_status(self):
    self.overlay.on_ui_state_tick(None, self.ui_state.sm)
    assert self.ui_state.status == self.ui_status.OVERRIDE

  def test_full_mode_keeps_engaged_status(self):
    self.ui_state.sm['carState'].cruiseState.enabled = True
    self.overlay.on_ui_state_tick(None, self.ui_state.sm)
    assert self.ui_state.status == self.ui_status.ENGAGED

  def test_disengaged_untouched(self):
    self.ui_state.engaged = False
    self.ui_state.status = self.ui_status.DISENGAGED
    self.overlay.on_ui_state_tick(None, self.ui_state.sm)
    assert self.ui_state.status == self.ui_status.DISENGAGED


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
