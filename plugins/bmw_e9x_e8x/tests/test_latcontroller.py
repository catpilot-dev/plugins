"""Tests for the BMW lateral controller's ISO cancel-guard call site
(on_lat_controller_init / update()) — as opposed to the pure module-scope
helpers (hold_factor, accel_guard_threshold) already covered in
test_hooks.py.

2026-07-28 (route 3ce S-exit census): the shared `overshooting` predicate
gates both cancel_accel and cancel_jerk, whose only action is draining
torque to zero. That is remedial only when the current torque still feeds
the measured turn — draining a torque that already opposes the measured
curvature (i.e. an active unwind already in flight) resets the ramp toward
0 every tick and locks out that unwind for the guard's drain window
(1.6-1.9s observed on sharp leg exits with a_y_meas > 2). This module
builds a minimal in-process harness to drive update() and prove the
torque-direction gate at the call-site level, not just via the predicate
formula in isolation.

Harness notes:
  - cereal.messaging is a MagicMock (test_helpers.install_all_mocks); we
    replace its SubMaster with a small controllable FakeSubMaster so
    livePose content (yaw rate -> measured kappa, v_ego) is test-driven
    instead of MagicMock auto-attributes.
  - `state` (the controller's per-tick dict) is a closure variable of the
    nested `update` function returned by on_lat_controller_init — there is
    no public accessor. We reach it via update.__closure__ (the standard,
    if unusual, way to introspect a closure cell), matching the "capture
    via the state dict" fallback the plan anticipated. This lets tests
    seed state['torque'] directly (bypassing the ramp) to control the
    torque/measured sign relationship precisely, and read back
    state['action'] / state['target_frac'] afterward.
  - accel_guard_threshold is a module-level free function; update()'s
    closure resolves it via the module globals, so monkeypatching the
    attribute on the loaded `bmw.latcontroller` module intercepts the
    real call site (proves what update() actually calls it with).
"""
import math
import os
import sys
from types import SimpleNamespace

import pytest

_PLUGIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PLUGIN_DIR not in sys.path:
  sys.path.insert(0, _PLUGIN_DIR)

from test_helpers import install_all_mocks


@pytest.fixture(autouse=True)
def mock_deps(monkeypatch):
  install_all_mocks(monkeypatch)


class FakeSubMaster:
  """Controllable stand-in for cereal.messaging.SubMaster(['livePose']).

  Real SubMaster exposes livePose via __getitem__, plus .updated/.seen
  dicts keyed by service name. Tests drive lp.angularVelocityDevice.z
  (yaw rate) and lp.velocityDevice.x (v_ego) directly.
  """
  def __init__(self, services):
    self.lp = SimpleNamespace(
      angularVelocityDevice=SimpleNamespace(z=0.0),
      velocityDevice=SimpleNamespace(x=20.0),
    )
    self.updated = {'livePose': True}
    self.seen = {'livePose': True}

  def update(self, timeout):
    pass

  def __getitem__(self, key):
    return self.lp


def _make_controller(monkeypatch, wheelbase=2.66):
  """Load bmw.latcontroller fresh-ish and construct a controller instance
  wired to a FakeSubMaster. Returns (lac, fake_sm, mod, state)."""
  import cereal.messaging as messaging
  fake_sm = FakeSubMaster([])
  monkeypatch.setattr(messaging, 'SubMaster', lambda services: fake_sm)

  import bmw.latcontroller as mod

  CP = SimpleNamespace(wheelbase=wheelbase, steerActuatorDelay=0.4)
  lac = SimpleNamespace()
  result = SimpleNamespace()
  mod.on_lat_controller_init(result, lac, CP)

  state = _closure_state(lac.update)
  return lac, fake_sm, mod, state


def _closure_state(update_fn):
  """Pull the `state` dict out of update()'s closure cells."""
  names = update_fn.__code__.co_freevars
  idx = names.index('state')
  return update_fn.__closure__[idx].cell_contents


def _set_measured(fake_sm, v, kappa_meas):
  """Arrange livePose so update() computes state['measured'] == kappa_meas
  at ego speed v (measured = angularVelocityDevice.z / v)."""
  fake_sm.lp.velocityDevice.x = v
  fake_sm.lp.angularVelocityDevice.z = kappa_meas * v


def _call_update(lac, desired_curvature, lat_delay=0.2, v_ego=20.0, active=True):
  CS = SimpleNamespace(vEgo=v_ego)
  return lac.update(active, CS, None, None, False, desired_curvature, False, lat_delay)


# ============================================================
# (a) call-site wiring: accel_guard_threshold must be called with the
# COMMANDED lateral accel v^2 * |kappa_des| -- not kappa_des alone, and
# not the measured a_y (v^2 * kappa_meas).
# ============================================================

class TestAccelGuardThresholdCallSite:
  def test_called_with_commanded_a_y_not_kappa_or_measured_a_y(self, monkeypatch):
    lac, fake_sm, mod, state = _make_controller(monkeypatch)

    v = 20.0
    desired = 0.006
    measured = 0.001  # deliberately different from desired so the three
                       # candidate call-args (kappa, v^2*kappa, v^2*measured)
                       # are all numerically distinct
    _set_measured(fake_sm, v, measured)

    calls = []
    real_fn = mod.accel_guard_threshold

    def spy(a_y_des_abs):
      calls.append(a_y_des_abs)
      return real_fn(a_y_des_abs)

    monkeypatch.setattr(mod, 'accel_guard_threshold', spy)

    _call_update(lac, desired, v_ego=v)

    assert len(calls) == 1
    expected = v * v * abs(desired)
    assert calls[0] == pytest.approx(expected)
    # Not called with kappa_des alone...
    assert calls[0] != pytest.approx(abs(desired))
    # ...and not with the measured a_y.
    assert calls[0] != pytest.approx(v * v * abs(measured))


# ============================================================
# (b)/(c) torque-direction gate on the shared `overshooting` predicate
# (2026-07-28, route 3ce S-exit census). Scenario mirrors a sharp leg
# exit: measured kappa still deep in the turn (a_y_meas > 2) while
# desired kappa has collapsed toward straight -- exactly the shape of
# overshoot the guard is meant to see.
# ============================================================

class TestTorqueDirectionGate:
  V = 15.0            # m/s
  DESIRED = 0.001      # kappa_des collapsed near straight (leg exit)
  MEASURED = 0.012     # kappa_meas still deep in the turn -> a_y_meas = 2.7

  def _prime(self, monkeypatch, torque, tick_count):
    lac, fake_sm, mod, state = _make_controller(monkeypatch)
    _set_measured(fake_sm, self.V, self.MEASURED)
    state['torque'] = torque
    # target_frac defaults to 0.0 at construction, which coincidentally
    # equals the drain target -- prime it away from 0 (as a completed
    # prior ramp would leave it) so the cancel path's re-arm guard
    # (`if state['target_frac'] != unwind_target`) actually arms the
    # drain ramp instead of seeing "already there".
    state['target_frac'] = torque
    state['ramp_frames'] = 0
    state['action'] = 'ramp'
    state['tick_count'] = tick_count
    return lac, state

  def test_precondition_overshooting_and_above_threshold(self, monkeypatch):
    """Sanity-check the scenario itself before testing the gate: plain
    overshoot math and a_y_meas both land where expected."""
    assert (self.DESIRED - self.MEASURED) * self.MEASURED < 0
    a_y_meas = self.V * self.V * self.MEASURED
    assert abs(a_y_meas) > 2.0

  def test_into_turn_torque_cancels(self, monkeypatch):
    """torque same sign as measured (still pushing into the turn) ->
    cancel_accel fires and drains toward 0."""
    lac, state = self._prime(monkeypatch, torque=0.5, tick_count=0)
    _call_update(lac, self.DESIRED, v_ego=self.V)
    assert state['action'] == 'cancel_accel'
    assert state['target_frac'] == pytest.approx(0.0)
    assert state['ramp_frames'] > 0

  def test_counter_turn_torque_does_not_cancel(self, monkeypatch):
    """torque opposite sign to measured (already actively unwinding) ->
    guard must stay silent; the normal off-target decision path runs
    instead (deep in a measured curve, this scenario also qualifies for
    relax_dwell -- still the ordinary decision cadence, not an ISO
    cancel). This is the route 3ce fix: previously this cancelled too,
    locking out the active unwind for 1.6-1.9s."""
    lac, state = self._prime(monkeypatch, torque=-0.3, tick_count=999)
    _call_update(lac, self.DESIRED, v_ego=self.V)
    assert state['action'] not in ('cancel_accel', 'cancel_jerk')
    assert state['action'] in ('ramp', 'relax_dwell')

  def test_zero_torque_does_not_cancel(self, monkeypatch):
    """No torque applied -> nothing feeds the turn -> guard stays silent."""
    lac, state = self._prime(monkeypatch, torque=0.0, tick_count=999)
    _call_update(lac, self.DESIRED, v_ego=self.V)
    assert state['action'] not in ('cancel_accel', 'cancel_jerk')

  # ---- mirror for negative-kappa (right-hand / opposite-sign) turns ----

  def test_into_turn_torque_cancels_negative_kappa(self, monkeypatch):
    lac, fake_sm, mod, state = _make_controller(monkeypatch)
    _set_measured(fake_sm, self.V, -self.MEASURED)
    state['torque'] = -0.5  # same sign as measured (-) -> into-turn
    state['target_frac'] = -0.5
    state['ramp_frames'] = 0
    state['action'] = 'ramp'
    state['tick_count'] = 0
    _call_update(lac, -self.DESIRED, v_ego=self.V)
    assert state['action'] == 'cancel_accel'
    assert state['target_frac'] == pytest.approx(0.0)
    assert state['ramp_frames'] > 0

  def test_counter_turn_torque_does_not_cancel_negative_kappa(self, monkeypatch):
    lac, fake_sm, mod, state = _make_controller(monkeypatch)
    _set_measured(fake_sm, self.V, -self.MEASURED)
    state['torque'] = 0.3  # opposite sign to measured (-)
    state['ramp_frames'] = 0
    state['action'] = 'ramp'
    state['tick_count'] = 999
    _call_update(lac, -self.DESIRED, v_ego=self.V)
    assert state['action'] not in ('cancel_accel', 'cancel_jerk')
    assert state['action'] in ('ramp', 'relax_dwell')

  def test_zero_torque_does_not_cancel_negative_kappa(self, monkeypatch):
    lac, fake_sm, mod, state = _make_controller(monkeypatch)
    _set_measured(fake_sm, self.V, -self.MEASURED)
    state['torque'] = 0.0
    state['ramp_frames'] = 0
    state['action'] = 'ramp'
    state['tick_count'] = 999
    _call_update(lac, -self.DESIRED, v_ego=self.V)
    assert state['action'] not in ('cancel_accel', 'cancel_jerk')
