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
  """minus5 (braking) transmits at HOLD_INTERVAL (40 Hz; measured 2.1x more
  setpoint ticks/sec than 20 Hz) for faster slew rate. plus5 (acceleration)
  is deliberately gentler and stays at SINGLE_INTERVAL (20 Hz), like plus1/
  minus1 (measured cadence-insensitive)."""
  mod = _cc(mock_opendbc)
  assert mod._tx_interval("plus5") == mod.SINGLE_INTERVAL
  assert mod._tx_interval("minus5") == mod.HOLD_INTERVAL
  assert mod._tx_interval("plus1") == mod.SINGLE_INTERVAL
  assert mod._tx_interval("minus1") == mod.SINGLE_INTERVAL


def test_decision_function_is_wired_in(mock_opendbc):
  mod = _cc(mock_opendbc)
  from bmw.dcc_map import select_cruise_command
  assert mod.select_cruise_command is select_cruise_command


def test_safety_machinery_survives(mock_opendbc):
  """The burst/counter-overwrite machinery must not be collateral damage of
  the vTarget-tracking rewrite. V_ERROR_DEADZONE is gone -- it belonged to the
  deleted direction gate, not to the burst machinery -- so it is intentionally
  excluded here (see test_v_error_deadzone_is_gone)."""
  mod = _cc(mock_opendbc)
  for name in ("HOLD_INTERVAL", "SINGLE_INTERVAL",
               "PRE_TICK_LEAD", "BURST_LIVE_WINDOW", "CRUISE_STALK_IDLE_TICK_STOCK"):
    assert hasattr(mod, name), f"{name} was removed but is still required"


def test_v_error_deadzone_is_gone(mock_opendbc):
  """V_ERROR_DEADZONE was the direction gate's threshold; the gate (and the
  gap-map inversion it supported) is gone from the control path, so the
  symbol must no longer be importable from either module."""
  mod = _cc(mock_opendbc)
  assert not hasattr(mod, "V_ERROR_DEADZONE")
  from bmw import dcc_map
  assert not hasattr(dcc_map, "V_ERROR_DEADZONE")
  assert not hasattr(dcc_map, "ACCEL_TRIGGER_KPH")
