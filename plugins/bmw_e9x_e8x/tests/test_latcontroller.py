"""Tests for the BMW lateral controller's update() call site
(on_lat_controller_init / update()).

2026-07-28 — SAFETY ARCHITECTURE change (lateral never gives up in a turn).
The ISO accel/jerk cancel machinery was removed: keeping lateral acceleration
within comfort/ISO limits is speedlimitd's job (it caps vEgo for curves), and
the lateral controller's contract is to track the commanded curvature, always.
Draining torque mid-turn converted a comfort exceedance into a trajectory
failure (car runs wide) — the incident record (routes 326, 385 seg27, 2ba
1.29 m off-lane, 3ce hunting, 3cf seg15 firing 75% of a sharp curve) shows
every cancel caused harm and none prevented any.

These tests pin, at the update() call-site level:
  - Formerly-cancelling conditions (overshoot + high measured a_y + into-turn
    torque) now produce normal P-tracking: torque is commanded toward κ_des,
    NOT drained to zero, and the action is never a cancel. (This test is RED
    against the pre-removal controller — it cancel_accel'd instead of ramping.)
  - The NON-cancel decision paths (ramp, hold_zero, hold_curve, relax_dwell)
    still work unchanged.
  - cancel_tol — which is NOT ISO machinery (it is HOLD_BAND boundary hygiene:
    stop an in-flight push ramp once the error falls into the on-target band,
    draining to the sign-guarded, capped hold, 0 on straights) — still fires
    identically in its boundary case.

Harness notes:
  - cereal.messaging is a MagicMock (test_helpers.install_all_mocks); we
    replace its SubMaster with a small controllable FakeSubMaster so
    livePose content (yaw rate -> measured kappa, v_ego) is test-driven
    instead of MagicMock auto-attributes.
  - `state` (the controller's per-tick dict) is a closure variable of the
    nested `update` function returned by on_lat_controller_init — there is
    no public accessor. We reach it via update.__closure__. This lets tests
    seed state['torque'] directly (bypassing the ramp) to control the
    torque/measured sign relationship precisely, and read back
    state['action'] / state['target_frac'] afterward.
"""
import math
import os
import sys
from types import SimpleNamespace

import pytest

_PLUGIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PLUGIN_DIR not in sys.path:
  sys.path.insert(0, _PLUGIN_DIR)

# bmw.latcontroller does `from config import read_plugin_param` at module scope
# (push-budget param helper) — on device the shared plugins/ dir (config.py,
# services.py, ...) is already on sys.path; replicate that here or the import
# raises ModuleNotFoundError at collection time and takes every test in this
# file (and test_hooks.py, which also loads bmw.latcontroller) down with it.
# Same fix, same reason, as speedlimitd/tests/test_speedlimitd.py's
# _PLUGINS_DIR insert.
_PLUGINS_DIR = os.path.dirname(_PLUGIN_DIR)
if _PLUGINS_DIR not in sys.path:
  sys.path.insert(0, _PLUGINS_DIR)

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
  """Load bmw.latcontroller and construct a controller instance wired to a
  FakeSubMaster. Returns (lac, fake_sm, mod, state)."""
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


def _call_update(lac, desired_curvature, lat_delay=0.2, v_ego=20.0, active=True,
                 steering_angle_deg=0.0):
  CS = SimpleNamespace(vEgo=v_ego, steeringAngleDeg=steering_angle_deg)
  return lac.update(active, CS, None, None, False, desired_curvature, False, lat_delay)


_CANCELS = ('cancel_accel', 'cancel_jerk')


# ============================================================
# (1) NEW — the SAFETY ARCHITECTURE change. Formerly-cancelling conditions now
# produce normal P-tracking. Scenario: overshoot ((κ_des - κ_meas)·κ_meas < 0)
# with high measured lateral accel (a_y_meas = v²·κ_meas ≈ 5.6 m/s², well past
# any old ISO threshold) and into-turn torque (torque·κ_meas > 0). Kept just
# below the deep-curve threshold (|κ_meas| < RELAX_DWELL_KAPPA) so the decision
# is a plain ramp, not a relax_dwell bridge.
#
# Pre-removal controller: this cancel_accel'd and drained target_frac -> 0.
# Post-removal: it tracks — torque is commanded toward κ_des (reduced from the
# into-turn value, i.e. unwinding), NOT to zero, and action is never a cancel.
# ============================================================

class TestFormerlyCancellingNowTracks:
  V = 25.0
  DESIRED = 0.001      # κ_des collapsed near straight
  MEASURED = 0.009     # κ_meas still turning -> a_y_meas ≈ 5.6, |κ| < deep-curve

  def _prime(self, monkeypatch, desired, measured, torque):
    lac, fake_sm, mod, state = _make_controller(monkeypatch)
    _set_measured(fake_sm, self.V, measured)
    state['torque'] = torque
    state['target_frac'] = torque      # as a completed prior ramp would leave it
    state['ramp_frames'] = 0
    state['action'] = 'ramp'
    state['tick_count'] = 999           # force the cadence decision to run
    return lac, state

  def test_precondition_is_overshoot_high_ay_into_turn(self, monkeypatch):
    """Sanity-check the scenario is exactly the shape the old guard fired on."""
    assert (self.DESIRED - self.MEASURED) * self.MEASURED < 0        # overshoot
    assert self.V * self.V * self.MEASURED > 3.0                     # a_y past ISO
    assert abs(self.MEASURED) < 0.010                                # not a deep curve

  def test_into_turn_torque_tracks_not_cancels(self, monkeypatch):
    """torque same sign as measured (into-turn) + overshoot + high a_y:
    used to cancel_accel and drain to 0. Now: normal ramp toward κ_des."""
    torque = 0.5
    lac, state = self._prime(monkeypatch, self.DESIRED, self.MEASURED, torque)
    _call_update(lac, self.DESIRED, v_ego=self.V)
    assert state['action'] not in _CANCELS
    assert state['action'] == 'ramp'
    # Commanded toward κ_des: reduced from the into-turn torque (unwinding)...
    assert state['target_frac'] < torque
    # ...but NOT drained to zero (the removed cancel behaviour).
    assert state['target_frac'] > 0.0
    assert state['ramp_frames'] > 0

  def test_into_turn_torque_tracks_not_cancels_negative_kappa(self, monkeypatch):
    """Mirror for a right-hand turn (all signs flipped)."""
    torque = -0.5
    lac, state = self._prime(monkeypatch, -self.DESIRED, -self.MEASURED, torque)
    _call_update(lac, -self.DESIRED, v_ego=self.V)
    assert state['action'] not in _CANCELS
    assert state['action'] == 'ramp'
    assert state['target_frac'] > torque      # reduced magnitude (unwinding)
    assert state['target_frac'] < 0.0         # not drained to zero
    assert state['ramp_frames'] > 0

  def test_no_a_y_or_jerk_field_triggers_a_cancel_action(self, monkeypatch):
    """a_y_meas / jerk_pred are still computed (telemetry), but nothing reads
    them to make a control decision anymore. Even with an extreme measured a_y
    the action never lands on a cancel."""
    lac, state = self._prime(monkeypatch, self.DESIRED, 0.02, 0.6)  # a_y ≈ 12.5
    _call_update(lac, self.DESIRED, v_ego=self.V)
    assert state['action'] not in _CANCELS
    # telemetry fields still populated for log analysis
    assert state['a_y_meas'] == pytest.approx(self.V * self.V * 0.02)


# ============================================================
# (2) cancel_tol still fires, unchanged. NOT ISO machinery — it is HOLD_BAND
# boundary hygiene: an in-flight PUSH ramp (action=='ramp', ramp_frames>0,
# |target_frac| > FRICTION) whose error has fallen into the on-target band
# (|δ_err| ≤ 1.2·HOLD_BAND) is stopped and drained to the sign-guarded, capped
# hold (hold_f·torque) instead of continuing toward a stale push target.
# ============================================================

class TestCancelTolStillFires:
  def test_boundary_case_drains_to_held(self, monkeypatch):
    """κ_des ≈ κ_meas (δ_err ≈ 0, inside the band) mid push-ramp: cancel_tol
    fires and re-arms the ramp toward the held value (hold_f·torque, capped),
    NOT toward the stale push target and NOT to zero."""
    lac, fake_sm, mod, state = _make_controller(monkeypatch)
    v = 20.0
    kappa = 0.006          # δ_des = δ_meas -> δ_err ≈ 0, inside 1.2·HOLD_BAND
    _set_measured(fake_sm, v, kappa)
    state['torque'] = 0.4          # -> held = hold_f(=1.0)·0.4 = 0.4, under hold_cap
    state['target_frac'] = 0.6     # stale in-flight push target (!= held, arms re-drain)
    state['ramp_frames'] = 5       # a ramp still in flight
    state['action'] = 'ramp'
    state['tick_count'] = 0

    _call_update(lac, kappa, v_ego=v)

    assert state['action'] == 'cancel_tol'
    assert state['target_frac'] == pytest.approx(0.4)   # drained to the hold, not 0.6, not 0
    assert state['ramp_frames'] > 0

  def test_cancel_tol_does_not_fire_off_target(self, monkeypatch):
    """Well outside the on-target band, cancel_tol must NOT fire (it is not a
    general drain — it only stops a ramp that already arrived)."""
    lac, fake_sm, mod, state = _make_controller(monkeypatch)
    v = 20.0
    _set_measured(fake_sm, v, 0.001)     # δ_err large vs κ_des below
    state['torque'] = 0.2
    state['target_frac'] = 0.3
    state['ramp_frames'] = 5
    state['action'] = 'ramp'
    state['tick_count'] = 0
    _call_update(lac, 0.010, v_ego=v)    # κ_des far from κ_meas
    assert state['action'] != 'cancel_tol'


# ============================================================
# (3) NON-cancel decision paths still work (ramp / hold_zero / hold_curve /
# relax_dwell). These pass on both the pre- and post-removal controller — they
# exercise the paths the removal must leave untouched.
# ============================================================

class TestNonCancelPaths:
  def test_off_target_ramps(self, monkeypatch):
    """Under-tracking into a curve (no overshoot): normal push ramp."""
    lac, fake_sm, mod, state = _make_controller(monkeypatch)
    v = 20.0
    _set_measured(fake_sm, v, 0.001)     # measured lagging
    state['torque'] = 0.0
    state['target_frac'] = 0.0
    state['ramp_frames'] = 0
    state['action'] = 'idle'
    state['tick_count'] = 999
    _call_update(lac, 0.008, v_ego=v)    # κ_des well ahead of κ_meas
    assert state['action'] == 'ramp'
    assert state['target_frac'] > 0.0    # pushing into the turn
    assert state['ramp_frames'] > 0

  def test_on_target_straight_holds_zero(self, monkeypatch):
    """On-target with low commanded a_y (straight): drain to zero, stiction
    holds."""
    lac, fake_sm, mod, state = _make_controller(monkeypatch)
    v = 20.0
    _set_measured(fake_sm, v, 0.0)
    state['torque'] = 0.0
    state['target_frac'] = 0.0
    state['ramp_frames'] = 0
    state['action'] = 'idle'
    state['tick_count'] = 999
    _call_update(lac, 0.0, v_ego=v)
    assert state['action'] == 'hold_zero'

  def test_on_target_curve_holds_curve(self, monkeypatch):
    """On-target in a curve with high commanded a_y: keep the standing torque
    (hold_curve), not drain."""
    lac, fake_sm, mod, state = _make_controller(monkeypatch)
    v = 20.0
    kappa = 0.006
    _set_measured(fake_sm, v, kappa)
    state['torque'] = 0.3
    state['target_frac'] = 0.3
    state['ramp_frames'] = 0
    state['action'] = 'hold_curve'   # NOT 'ramp' -> cancel_tol gate stays shut
    state['tick_count'] = 999
    _call_update(lac, kappa, v_ego=v)
    assert state['action'] == 'hold_curve'
    assert state['target_frac'] == pytest.approx(0.3)

  def test_counter_turn_torque_runs_normal_decision(self, monkeypatch):
    """Deep in a measured curve with torque already opposing the turn (an
    active unwind in flight): never a cancel. Deep + same-side κ_des qualifies
    for the relax_dwell bridge — the ordinary decision cadence, not a drain.
    (Pre-removal this scenario stayed silent too, because the overshoot gate
    required into-turn torque; the removal does not change it.)"""
    lac, fake_sm, mod, state = _make_controller(monkeypatch)
    v = 15.0
    _set_measured(fake_sm, v, 0.012)     # deep curve
    state['torque'] = -0.3               # opposite sign to measured -> unwinding
    state['target_frac'] = -0.3
    state['ramp_frames'] = 0
    state['action'] = 'ramp'
    state['tick_count'] = 999
    _call_update(lac, 0.001, v_ego=v)    # κ_des collapsed (leg exit)
    assert state['action'] not in _CANCELS
    assert state['action'] in ('ramp', 'relax_dwell')

  def test_zero_torque_runs_normal_decision(self, monkeypatch):
    """No torque applied: never a cancel; normal off-target ramp."""
    lac, fake_sm, mod, state = _make_controller(monkeypatch)
    v = 15.0
    _set_measured(fake_sm, v, 0.012)
    state['torque'] = 0.0
    state['target_frac'] = 0.0
    state['ramp_frames'] = 0
    state['action'] = 'ramp'
    state['tick_count'] = 999
    _call_update(lac, 0.001, v_ego=v)
    assert state['action'] not in _CANCELS


# ============================================================
# (4) Push budget — task-1-brief Step 2. Human-style: push harder until the
# wheel actually moves, then stop pushing harder and ease off. `_drive` pins
# state['torque']/state['action'] each tick (as the existing tests above do)
# while feeding a steering-angle trajectory through the new steeringAngleDeg
# plumbing (Step 1).
# ============================================================

def _drive(lac, sm, state, angles, torque=-0.30, desired=0.002):
    """Hold a commanded torque while feeding a steering-angle trajectory."""
    for a in angles:
        state['torque'] = torque
        state['action'] = 'ramp'
        _set_measured(sm, 20.0, 0.001)
        _call_update(lac, desired, steering_angle_deg=a)


def test_budget_unspent_while_the_wheel_barely_moves(monkeypatch):
    lac, sm, mod, state = _make_controller(monkeypatch)
    monkeypatch.setattr(mod, 'read_plugin_param', lambda *a, **k: '1')
    _drive(lac, sm, state, [0.0, 0.4, 0.8, 1.2, 1.5, 1.8])
    assert state['budget_spent'] is False


def test_budget_spent_after_two_degrees_in_the_commanded_direction(monkeypatch):
    lac, sm, mod, state = _make_controller(monkeypatch)
    monkeypatch.setattr(mod, 'read_plugin_param', lambda *a, **k: '1')
    # Negative torque commands LEFT; LEFT is POSITIVE steering angle.
    _drive(lac, sm, state, [0.0, 0.5, 1.0, 1.5, 2.0, 2.5])
    assert state['budget_spent'] is True


def test_movement_opposing_the_command_does_not_spend_it(monkeypatch):
    """Camber or a bump moving the wheel the wrong way is not our doing."""
    lac, sm, mod, state = _make_controller(monkeypatch)
    monkeypatch.setattr(mod, 'read_plugin_param', lambda *a, **k: '1')
    _drive(lac, sm, state, [0.0, -0.5, -1.0, -1.5, -2.0, -2.5])
    assert state['budget_spent'] is False


def test_reference_resets_when_the_push_ends(monkeypatch):
    lac, sm, mod, state = _make_controller(monkeypatch)
    monkeypatch.setattr(mod, 'read_plugin_param', lambda *a, **k: '1')
    _drive(lac, sm, state, [0.0, 1.0, 2.0, 3.0])
    assert state['budget_spent'] is True
    state['action'] = 'hold_zero'
    _set_measured(sm, 20.0, 0.001)
    _call_update(lac, 0.002, steering_angle_deg=3.0)
    assert state['budget_spent'] is False
    _drive(lac, sm, state, [3.0, 3.5])          # new push from 3.0
    assert state['budget_spent'] is False


def test_offset_cancels(monkeypatch):
    """A constant alignment offset must not change when the budget is spent."""
    for base in (0.0, -1.58, +7.3):
        lac, sm, mod, state = _make_controller(monkeypatch)
        monkeypatch.setattr(mod, 'read_plugin_param', lambda *a, **k: '1')
        _drive(lac, sm, state, [base + 0.5 * i for i in range(5)])
        assert state['budget_spent'] is True


def test_toggle_off_never_spends_the_budget(monkeypatch):
    lac, sm, mod, state = _make_controller(monkeypatch)
    monkeypatch.setattr(mod, 'read_plugin_param', lambda *a, **k: '0')
    _drive(lac, sm, state, [0.0 + 0.5 * i for i in range(10)])
    assert state['budget_spent'] is False


def test_missing_steering_angle_degrades_safely(monkeypatch):
    lac, sm, mod, state = _make_controller(monkeypatch)
    CS = SimpleNamespace(vEgo=20.0)          # no steeringAngleDeg
    for _ in range(20):
        _set_measured(sm, 20.0, 0.001)
        out = lac.update(True, CS, None, None, False, 0.002, False, 0.2)
    assert out is not None
    assert state['budget_spent'] is False
