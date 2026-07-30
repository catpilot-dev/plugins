import os
import sys

import pytest

_PLUGIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PLUGIN_DIR not in sys.path:
  sys.path.insert(0, _PLUGIN_DIR)

from test_helpers import make_opendbc_mocks


@pytest.fixture(autouse=True)
def mock_opendbc(monkeypatch):
  for mod_name, mod_mock in make_opendbc_mocks().items():
    monkeypatch.setitem(sys.modules, mod_name, mod_mock)


def _cc(mock_opendbc):
  import importlib
  import bmw.carcontroller as mod
  importlib.reload(mod)
  return mod


DEAD = ("ACCEL_HOLD_THRESHOLD", "DECEL_HOLD_THRESHOLD",
        "ACCEL_STEP5_THRESHOLD", "DECEL_STEP5_THRESHOLD")


@pytest.mark.parametrize("name", DEAD)
def test_dead_thresholds_are_gone(mock_opendbc, name):
  assert not hasattr(_cc(mock_opendbc), name), \
      f"{name} is superseded by the setpoint-error law and must be deleted"


def test_new_constants_present(mock_opendbc):
  mod = _cc(mock_opendbc)
  # CMD_INTERVAL is gone -- interval is now chosen per-command by _tx_interval.
  assert not hasattr(mod, "CMD_INTERVAL")
  assert mod.HOLD_INTERVAL == 0.025
  assert mod.SINGLE_INTERVAL == 0.050
  assert mod.SETPOINT_DEADBAND_KPH == 1.0


def test_tx_interval_selects_by_command_magnitude(mock_opendbc):
  """+-5 commands must transmit at HOLD_INTERVAL (40 Hz; measured 2.1x more
  setpoint ticks/sec than 20 Hz), +-1 commands at SINGLE_INTERVAL (20 Hz;
  measured cadence-insensitive)."""
  mod = _cc(mock_opendbc)
  assert mod._tx_interval("plus5") == mod.HOLD_INTERVAL
  assert mod._tx_interval("minus5") == mod.HOLD_INTERVAL
  assert mod._tx_interval("plus1") == mod.SINGLE_INTERVAL
  assert mod._tx_interval("minus1") == mod.SINGLE_INTERVAL


def test_decision_function_is_wired_in(mock_opendbc):
  mod = _cc(mock_opendbc)
  from bmw.dcc_map import select_cruise_command
  assert mod.select_cruise_command is select_cruise_command


def test_safety_machinery_survives(mock_opendbc):
  """The burst/counter-overwrite and entry-deadzone machinery must not be
  collateral damage of the rewrite."""
  mod = _cc(mock_opendbc)
  for name in ("V_ERROR_DEADZONE", "HOLD_INTERVAL", "SINGLE_INTERVAL",
               "PRE_TICK_LEAD", "BURST_LIVE_WINDOW", "CRUISE_STALK_IDLE_TICK_STOCK"):
    assert hasattr(mod, name), f"{name} was removed but is still required"
