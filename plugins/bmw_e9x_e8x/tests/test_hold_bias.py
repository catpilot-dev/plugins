"""Unit tests for the self-calibrating hold-bias pure functions (register.py).

These functions are plain-Python and stateless; importing `register` still needs
the opendbc/cereal mocks (module runs _register_interfaces at load).
"""
import os
import sys
import pytest

_PLUGIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PLUGIN_DIR not in sys.path:
  sys.path.insert(0, _PLUGIN_DIR)

from test_helpers import install_all_mocks


@pytest.fixture(autouse=True)
def mock_deps(monkeypatch):
  install_all_mocks(monkeypatch)


# Starting constants (mirror register.py HOLD_* defaults)
A_ON, A_FULL, K_ON, K_FULL = 20.0, 35.0, 0.006, 0.012
KI, B_MAX, LEAK = 0.8, 0.20, 0.10


class TestHoldGate:
  def test_below_both_thresholds_zero(self):
    import register
    assert register.hold_gate(10.0, 0.003, A_ON, A_FULL, K_ON, K_FULL) == 0.0

  def test_both_full_is_one(self):
    import register
    assert register.hold_gate(35.0, 0.012, A_ON, A_FULL, K_ON, K_FULL) == pytest.approx(1.0)

  def test_above_full_clamps_to_one(self):
    import register
    assert register.hold_gate(60.0, 0.02, A_ON, A_FULL, K_ON, K_FULL) == pytest.approx(1.0)

  def test_angle_midpoint_kappa_full(self):
    import register
    # ga = (27.5-20)/15 = 0.5 ; gk = 1.0 -> 0.5
    assert register.hold_gate(27.5, 0.012, A_ON, A_FULL, K_ON, K_FULL) == pytest.approx(0.5)

  def test_and_gate_low_curvature_closes(self):
    import register
    # high angle but curvature below threshold -> gate is 0 (AND of both)
    assert register.hold_gate(50.0, 0.003, A_ON, A_FULL, K_ON, K_FULL) == 0.0

  def test_uses_absolute_value(self):
    import register
    assert register.hold_gate(-50.0, -0.02, A_ON, A_FULL, K_ON, K_FULL) == pytest.approx(1.0)

  def test_equal_or_inverted_thresholds_return_zero(self):
    import register
    assert register.hold_gate(50.0, 0.02, 20.0, 20.0, 0.006, 0.012) == 0.0   # equal angle span
    assert register.hold_gate(50.0, 0.02, 20.0, 35.0, 0.012, 0.012) == 0.0   # equal kappa span


class TestHoldLearnFlags:
  def test_holding_on_target_learns(self):
    import register
    assert register.hold_learn_flags(0.5, 'hold_zero', False, False, False) == (True, False)

  def test_ramp_learns(self):
    import register
    assert register.hold_learn_flags(0.8, 'ramp', False, False, False) == (True, False)

  def test_out_of_regime_releases(self):
    import register
    assert register.hold_learn_flags(0.0, 'hold_zero', False, False, False) == (False, True)

  def test_driver_pressed_releases(self):
    import register
    assert register.hold_learn_flags(0.5, 'ramp', True, False, False) == (False, True)

  def test_overshoot_freezes_no_release(self):
    import register
    assert register.hold_learn_flags(0.5, 'ramp', False, True, False) == (False, False)

  def test_saturated_freezes(self):
    import register
    assert register.hold_learn_flags(0.5, 'ramp', False, False, True) == (False, False)

  def test_cancel_action_freezes(self):
    import register
    assert register.hold_learn_flags(0.5, 'cancel_jerk', False, False, False) == (False, False)


class TestHoldBiasStep:
  def test_integrates_up(self):
    import register
    # 0 + 0.8*1.0*0.01 = 0.008
    assert register.hold_bias_step(0.0, 1.0, 0.01, True, False, KI, B_MAX, LEAK) == pytest.approx(0.008)

  def test_integrates_signed(self):
    import register
    assert register.hold_bias_step(0.0, 1.0, -0.01, True, False, KI, B_MAX, LEAK) == pytest.approx(-0.008)

  def test_clamps_positive(self):
    import register
    assert register.hold_bias_step(0.19, 1.0, 0.05, True, False, KI, B_MAX, LEAK) == pytest.approx(0.20)

  def test_clamps_negative(self):
    import register
    assert register.hold_bias_step(-0.19, 1.0, -0.05, True, False, KI, B_MAX, LEAK) == pytest.approx(-0.20)

  def test_freeze_holds_value(self):
    import register
    assert register.hold_bias_step(0.1, 0.5, 0.01, False, False, KI, B_MAX, LEAK) == pytest.approx(0.1)

  def test_release_leaks_toward_zero(self):
    import register
    # 0.1 + 0.1*(0-0.1) = 0.09
    assert register.hold_bias_step(0.1, 0.0, 0.0, False, True, KI, B_MAX, LEAK) == pytest.approx(0.09)

  def test_converges_and_clamps_over_iteration(self):
    import register
    b = 0.0
    for _ in range(200):
      b = register.hold_bias_step(b, 1.0, 0.005, True, False, KI, B_MAX, LEAK)
    assert b == pytest.approx(0.20)  # ramps to the clamp under sustained error

  def test_leak_decays_to_near_zero(self):
    import register
    b = 0.20
    for _ in range(100):
      b = register.hold_bias_step(b, 0.0, 0.0, False, True, KI, B_MAX, LEAK)
    assert abs(b) < 0.01


class TestHoldApplied:
  def test_scales_by_gate(self):
    import register
    assert register.hold_applied(0.0, 0.5, 0.15) == pytest.approx(0.075)

  def test_adds_to_base(self):
    import register
    assert register.hold_applied(0.1, 1.0, 0.15) == pytest.approx(0.25)

  def test_gate_zero_is_passthrough(self):
    import register
    assert register.hold_applied(0.1, 0.0, 0.15) == pytest.approx(0.1)

  def test_signed(self):
    import register
    assert register.hold_applied(-0.1, 1.0, -0.15) == pytest.approx(-0.25)
