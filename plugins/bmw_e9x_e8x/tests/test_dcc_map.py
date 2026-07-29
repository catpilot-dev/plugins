import pytest
import sys
import os

# Add plugin dir to path so bmw package is importable
_PLUGIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PLUGIN_DIR not in sys.path:
  sys.path.insert(0, _PLUGIN_DIR)

from bmw.dcc_map import (expected_accel, gap_for_accel, accel_envelope,
                         select_cruise_command)
from bmw.dcc_map_table import GAP_BPS, V_BPS, A_TABLE


def test_table_columns_are_monotone():
  for j in range(len(V_BPS)):
    col = [A_TABLE[i][j] for i in range(len(GAP_BPS))]
    assert all(b >= a for a, b in zip(col, col[1:])), f"column {j} not monotone"


def test_expected_accel_hits_table_nodes():
  for i, g in enumerate(GAP_BPS):
    for j, v in enumerate(V_BPS):
      assert expected_accel(g, v) == pytest.approx(A_TABLE[i][j], abs=1e-9)


def test_expected_accel_clamps_outside_table():
  v = V_BPS[2]
  assert expected_accel(GAP_BPS[0] - 5.0, v) == pytest.approx(expected_accel(GAP_BPS[0], v))
  assert expected_accel(GAP_BPS[-1] + 5.0, v) == pytest.approx(expected_accel(GAP_BPS[-1], v))


def test_negative_gap_gives_negative_accel():
  for v in V_BPS:
    assert expected_accel(-2.0, v) < 0.0
    assert expected_accel(+1.5, v) > expected_accel(-1.0, v)


def test_inversion_round_trips_inside_the_envelope():
  for v in V_BPS:
    lo, hi = accel_envelope(v)
    for frac in (0.2, 0.5, 0.8):
      a = lo + (hi - lo) * frac
      g = gap_for_accel(a, v)
      assert expected_accel(g, v) == pytest.approx(a, abs=0.02)


def test_inversion_clamps_beyond_authority():
  v = V_BPS[2]
  lo, hi = accel_envelope(v)
  assert gap_for_accel(hi + 5.0, v) == pytest.approx(GAP_BPS[-1])
  assert gap_for_accel(lo - 5.0, v) == pytest.approx(GAP_BPS[0])


def test_gap_never_escapes_table_bounds():
  for v in (1.0, V_BPS[0], V_BPS[-1], 60.0):
    for a in (-9.0, -1.0, 0.0, 1.0, 9.0):
      assert GAP_BPS[0] <= gap_for_accel(a, v) <= GAP_BPS[-1]


def test_envelope_is_ordered():
  for v in V_BPS:
    lo, hi = accel_envelope(v)
    assert lo < hi


# ---- command selection ----

def cmd(a_target, v_ego, setpoint, v_target, min_setpoint=5.0):
  return select_cruise_command(a_target, v_ego, setpoint, v_target, min_setpoint)


def test_deadband_emits_nothing():
  v, a = 20.0, 0.0
  sp = v + gap_for_accel(a, v)
  assert cmd(a, v, sp, v_target=v + 5.0) is None


def test_accel_request_below_target_emits_plus():
  v = 20.0
  assert cmd(+0.3, v, setpoint=v, v_target=v + 5.0) in ("plus1", "plus5")


def test_large_setpoint_error_uses_step5():
  v = 20.0
  _, a_max = accel_envelope(v)
  assert cmd(a_max, v, setpoint=v, v_target=v + 10.0) == "plus5"


def test_small_setpoint_error_uses_step1():
  v = 20.0
  sp = v + gap_for_accel(0.0, v) - (2.0 / 3.6)   # 2 km/h short -> one tick
  assert cmd(0.0, v, setpoint=sp, v_target=v + 10.0) == "plus1"


def test_setpoint_never_commanded_above_v_target():
  v, v_target = 20.0, 20.5
  assert cmd(+0.5, v, setpoint=v_target, v_target=v_target) is None


def test_decel_request_emits_minus():
  v = 25.0
  assert cmd(-0.5, v, setpoint=v, v_target=v - 5.0, min_setpoint=5.0) in ("minus1", "minus5")


def test_beyond_authority_saturates_not_escalates():
  v = 20.0
  _, a_max = accel_envelope(v)
  assert cmd(a_max + 5.0, v, v, v + 20.0) == cmd(a_max, v, v, v + 20.0)


def test_commanded_setpoint_never_goes_below_min():
  """The desired-setpoint clamp is what protects min cruise speed.

  A minus tick moves 1 or 5 km/h and is only emitted when at least that much
  error exists above the floor, so it can never carry the setpoint under it.
  """
  v, min_sp = 10.0, 8.0
  for excess_kph in (0.2, 0.9, 1.5, 4.0, 6.0, 12.0):
    sp = min_sp + excess_kph / 3.6
    out = cmd(-1.0, v, setpoint=sp, v_target=0.0, min_setpoint=min_sp)
    if out is None:
      continue
    step_kph = 5.0 if out == "minus5" else 1.0
    assert sp - step_kph / 3.6 >= min_sp - 1e-9, \
        f"{out} from setpoint {sp:.3f} would cross the {min_sp} m/s floor"
