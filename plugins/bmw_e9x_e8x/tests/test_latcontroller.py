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
from unittest.mock import MagicMock

import pytest

_PLUGIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PLUGIN_DIR not in sys.path:
  sys.path.insert(0, _PLUGIN_DIR)

# The shared-plugins-dir sys.path fix (for bmw.latcontroller's module-scope
# `from config import read_plugin_param`) now lives in test_helpers.py, which
# every file in this directory that loads bmw.latcontroller already imports
# — see the comment there (review fix, Minor 10: a fix scoped to only this
# file left test_hooks.py broken when run in isolation).
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


def _make_controller(monkeypatch, wheelbase=2.66, angle_budget=None, hold_hyst=None):
  """Load bmw.latcontroller and construct a controller instance wired to a
  FakeSubMaster. Returns (lac, fake_sm, mod, state).

  angle_budget: if not None, patches config.read_plugin_param so AngleBudget
  reads as '1' (True) or '0' (False) — must happen BEFORE
  on_lat_controller_init runs, because the param is read exactly once at
  construction (review fix, Important 4: no more per-tick/cached re-read),
  so patching it *after* construction has no effect. Patches the `config`
  module's attribute, not `mod`'s (review fix, Important 2:
  on_lat_controller_init now does `from config import read_plugin_param` at
  function scope, mirroring bmw/carstate.py and speedlimitd/speedlimitd.py,
  so `bmw.latcontroller` no longer has a `read_plugin_param` module
  attribute to patch — same pattern as
  speedlimitd/tests/test_speedlimitd.py's `monkeypatch.setattr(config,
  'read_plugin_param', ...)`).

  hold_hyst: if not None, patches the same config.read_plugin_param so
  HoldHysteresis reads as '1' (True) or '0' (False) — also read exactly once
  at construction. angle_budget and hold_hyst go through the same
  read_plugin_param call, so the stub dispatches on the param key rather than
  returning one value for every key (2026-08-13 — the original angle_budget-
  only stub ignored the key entirely, which would have made HoldHysteresis
  silently inherit whatever angle_budget was set to). Either kwarg left None
  falls through to the `default` argument production passes (''), i.e. the
  real unpatched-param behaviour for that key.
  """
  import cereal.messaging as messaging
  fake_sm = FakeSubMaster([])
  monkeypatch.setattr(messaging, 'SubMaster', lambda services: fake_sm)

  import bmw.latcontroller as mod
  if angle_budget is not None or hold_hyst is not None:
    import config
    def _param_stub(plugin_id, key, default=''):
      if key == 'AngleBudget' and angle_budget is not None:
        return '1' if angle_budget else '0'
      if key == 'HoldHysteresis' and hold_hyst is not None:
        return '1' if hold_hyst else '0'
      return default
    monkeypatch.setattr(config, 'read_plugin_param', _param_stub)

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
# (4) Push budget — task-1-brief Step 2, updated after code review (2026-08-12
# fix round). Human-style: push harder until the wheel actually moves, then
# stop pushing harder and ease off. `_drive` pins state['torque']/
# state['action'] each tick (as the existing tests above do) while feeding a
# steering-angle trajectory through the steeringAngleDeg plumbing (Step 1).
#
# angle_budget=True/False is now passed to _make_controller() at construction
# time rather than monkeypatched afterward — the param is read exactly once
# at construction (Important 4: no more per-tick cache), so a
# post-construction monkeypatch.setattr(mod, 'read_plugin_param', ...) (the
# original task-1 pattern, and also stale on the attribute path: Important 2
# moved the import to function scope, so `mod` no longer has a
# read_plugin_param attribute at all — config.read_plugin_param is now the
# patch target) has no effect. See _make_controller's docstring.
# ============================================================

def _drive(lac, sm, state, angles, torque=-0.30, desired=0.002):
    """Hold a commanded torque while feeding a steering-angle trajectory."""
    for a in angles:
        state['torque'] = torque
        state['action'] = 'ramp'
        _set_measured(sm, 20.0, 0.001)
        _call_update(lac, desired, steering_angle_deg=a)


def test_budget_unspent_while_the_wheel_barely_moves(monkeypatch):
    lac, sm, mod, state = _make_controller(monkeypatch, angle_budget=True)
    _drive(lac, sm, state, [0.0, 0.4, 0.8, 1.2, 1.5, 1.8])
    assert state['budget_spent'] is False


def test_budget_spent_after_two_degrees_in_the_commanded_direction(monkeypatch):
    lac, sm, mod, state = _make_controller(monkeypatch, angle_budget=True)
    # Negative torque commands LEFT; LEFT is POSITIVE steering angle.
    # Two calls, not a slow multi-step ramp (Critical 1 review fix): push_ref
    # now re-arms every decision, and this harness's default lat_delay=0.2
    # gives a 2-tick decision cadence, so a slow 0.5 deg/call crawl spread
    # across many decisions never accumulates 2 deg within any single
    # decision window anymore (by design — see
    # test_budget_unspent_while_the_wheel_barely_moves and the C1-recovery
    # test below). The must-spend case is 2+ degrees inside ONE decision.
    _drive(lac, sm, state, [0.0, 2.5])
    assert state['budget_spent'] is True


def test_movement_opposing_the_command_does_not_spend_it(monkeypatch):
    """Camber or a bump moving the wheel the wrong way is not our doing."""
    lac, sm, mod, state = _make_controller(monkeypatch, angle_budget=True)
    _drive(lac, sm, state, [0.0, -0.5, -1.0, -1.5, -2.0, -2.5])
    assert state['budget_spent'] is False


def test_reference_resets_when_the_push_ends(monkeypatch):
    lac, sm, mod, state = _make_controller(monkeypatch, angle_budget=True)
    # Jump straight to +3 deg within one decision (Critical 1 review fix: see
    # test_budget_spent_after_two_degrees_in_the_commanded_direction for why
    # a slow multi-step climb no longer spends it) -- ends at the same 3.0
    # deg the rest of this test already assumes the wheel is sitting at.
    _drive(lac, sm, state, [0.0, 3.0])
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
        lac, sm, mod, state = _make_controller(monkeypatch, angle_budget=True)
        # 2.5 deg within one decision (Critical 1 review fix -- see
        # test_budget_spent_after_two_degrees_in_the_commanded_direction).
        _drive(lac, sm, state, [base + 2.5 * i for i in range(2)])
        assert state['budget_spent'] is True


def test_toggle_off_never_spends_the_budget(monkeypatch):
    lac, sm, mod, state = _make_controller(monkeypatch, angle_budget=False)
    _drive(lac, sm, state, [0.0 + 0.5 * i for i in range(10)])
    assert state['budget_spent'] is False


def test_missing_steering_angle_degrades_safely(monkeypatch):
    """Minor 9 (review): angle_budget=True so the getattr guard is actually
    exercised — with the param left unpatched (real read_plugin_param sees no
    file, defaults off), this test passed for the wrong reason: budget_spent
    would be False regardless of whether the guard existed, since the AND
    chain short-circuits on the toggle before ever reaching the angle."""
    lac, sm, mod, state = _make_controller(monkeypatch, angle_budget=True)
    CS = SimpleNamespace(vEgo=20.0)          # no steeringAngleDeg
    for _ in range(20):
        _set_measured(sm, 20.0, 0.001)
        out = lac.update(True, CS, None, None, False, 0.002, False, 0.2)
    assert out is not None
    assert state['budget_spent'] is False


def test_budget_does_not_accrue_while_disengaged(monkeypatch):
    """Important 3 (review): update() runs regardless of engagement, and the
    decision state machine that sets state['action'] is not itself gated on
    active — so without gating the capture on active, driver steering while
    disengaged would accrue into push_moved and a push could begin already
    spent. CS.steeringPressed is not a substitute gate on this car (it is a
    voice-control button ORed with gasPressed)."""
    lac, sm, mod, state = _make_controller(monkeypatch, angle_budget=True)
    for a in [0.0, 1.0, 2.0, 3.0, 4.0]:
        state['torque'] = -0.30
        state['action'] = 'ramp'
        _set_measured(sm, 20.0, 0.001)
        _call_update(lac, 0.002, steering_angle_deg=a, active=False)
    assert state['push_ref'] is None
    assert state['push_moved'] == 0.0
    assert state['budget_spent'] is False


# ============================================================
# (5) Push-budget CLAMP — review fix (2026-08-12), Important 6. The 7 tests
# above only ever assert on state['budget_spent']; none of them exercised the
# clamp itself, and _drive's per-tick re-pin of state['torque'] would mask
# any ramp effect even if they tried to. These four drive a REAL decision
# (not _drive) through the sign-aware clamp and assert on state['target_frac']
# and the resulting state['torque'] after one ramp tick — exactly the tests
# that would have caught Criticals 1 and 2 in the original abs()-only
# comparison: it either froze torque at its wrong-direction value on a
# reversal (case C shape), or, when the counter-target had smaller magnitude,
# applied no cap at all (case D shape — a Delta 0.547 frac swing here, the
# same shape as the reported Delta 0.578 frac / 6.9 Nm route example).
#
# All four use v=20.0, wheelbase=2.66 (default), measured=0.0 (so
# delta_err == delta_des and hold_f/held_target stay out of the way — hold_f
# is 0 in cases A/B (v^2*|desired| < HOLD_AY_BP[0] = 0.5) and in C/D where
# hold_f == 1, held_target sign-matches torque and is smaller in magnitude
# than the P-computed target, so the pre-existing hold-floor is a no-op).
# Expected numbers were derived by mirroring the production formula
# (kappa_scale, target_nm, t_cap, hold-floor, then the budget clamp) in a
# standalone script, cross-checked against the old (pre-fix) formula to
# confirm each case actually discriminates buggy from fixed behaviour — see
# task-1-report.md's fix-round section for the full derivation and the
# old-vs-new numbers.
# ============================================================

def _prime_and_decide(lac, sm, state, *, torque, v, desired, measured, push_angle):
    """Prime state for a fresh decision to fire this tick, with the push
    budget already spent in the commanded (torque) direction, then call
    update() once. Bypasses _drive's per-tick re-pin of torque so the ramp's
    effect on state['torque'] is observable."""
    state['torque'] = torque
    state['target_frac'] = torque
    state['ramp_frames'] = 0
    state['action'] = 'ramp'
    state['push_ref'] = 0.0
    state['tick_count'] = 999
    _set_measured(sm, v, measured)
    _call_update(lac, desired, v_ego=v, steering_angle_deg=push_angle)


def test_budget_clamp_same_direction_harder_freezes(monkeypatch):
    """Case A: pushing harder in the same direction the budget was spent on
    must freeze at the current torque, not follow the P-law further out."""
    lac, sm, mod, state = _make_controller(monkeypatch, angle_budget=True)
    _prime_and_decide(lac, sm, state, torque=-0.30, v=20.0, desired=-0.003,
                      measured=0.0, push_angle=2.5)
    assert state['budget_spent'] is True
    assert state['target_frac'] == pytest.approx(-0.30, abs=1e-6)
    assert state['torque'] == pytest.approx(-0.30, abs=1e-6)   # unmoved


def test_budget_clamp_same_direction_easing_is_unthrottled(monkeypatch):
    """Case B: easing off in the same direction goes straight to the P-law's
    (smaller-magnitude) target in one decision, not stepped at STEP_MAX like
    a normal ramp.

    desired moved from -0.0005 to -0.0008 (2026-08-13, entry/settle
    hysteresis): with hysteresis default ON, -0.0005 produces
    |delta_err| ≈ 0.00133 rad, which is > HOLD_BAND (0.001, the legacy single
    threshold this value used to clear) but < HOLD_BAND_ENTER (0.0015) — from
    this test's fresh controller (state['at_rest'] starts True), that no
    longer leaves rest, so the decision lands in the hold branch instead of
    the ramp/easing branch this test exists to exercise. The specific error
    magnitude here was always incidental to the test's intent (a P-target
    smaller in magnitude than the -0.30 held torque, with hold_f == 0 so
    held_target stays out of the way — v²·|desired| = 400·0.0008 = 0.32 <
    HOLD_AY_BP[0] = 0.5); -0.0008 preserves both properties while clearing
    HOLD_BAND_ENTER (|delta_err| ≈ 0.00213 rad).

    Independent derivation of the expected numbers (review fix, Important 2
    -- these were originally transcribed from a controller run rather than
    derived, which made them circular as a pin; re-derived by hand here from
    the production formula so they stand on their own):

        δ_err   = atan(0.0008·2.66)          = 0.00212800
        κ_scale = 1.0 (below first breakpoint)
        target  = 1.0·1.0·400·(-0.002128)/12 = -0.0709333  (T_CAP_SLOPE_BASE=1.0, STEER_MAX=12)
        hold_f  = 0 (400·0.0008 = 0.32 < HOLD_AY_BP[0]=0.5) -> held_target = 0
        t_cap   = (2.0+0.8512)/12 = 0.2376 -> no clip; budget lo/hi -0.3808/+0.0808 -> no clip
        torque  = -0.30 + 0.2290667/10       = -0.27709333  (spread_frames=10)
    """
    lac, sm, mod, state = _make_controller(monkeypatch, angle_budget=True)
    _prime_and_decide(lac, sm, state, torque=-0.30, v=20.0, desired=-0.0008,
                      measured=0.0, push_angle=2.5)
    assert state['budget_spent'] is True
    assert state['target_frac'] == pytest.approx(-0.070933, abs=1e-5)
    assert state['torque'] == pytest.approx(-0.277093, abs=1e-5)
    # A STEP_MAX(20 m/s)=0.080769-clamped ramp would have moved torque by
    # only step_max/spread_frames = 0.0080769 this tick; the actual movement
    # is well over 2x that, proving the step was not throttled.
    step_max_over_spread_frames = 0.080769 / 10
    assert abs(state['torque'] - (-0.30)) > step_max_over_spread_frames


def test_budget_clamp_reversal_reaches_zero_and_one_step_past_not_frozen(monkeypatch):
    """Case C: the P-law reversing (target flips sign vs standing torque),
    counter-target BIGGER in magnitude than torque. The old sign-blind
    |target|>|torque| check froze here — the controller giving up mid-turn,
    the invariant the module's SAFETY ARCHITECTURE note forbids. Must instead
    shed to zero and continue exactly one step_max past."""
    lac, sm, mod, state = _make_controller(monkeypatch, angle_budget=True)
    _prime_and_decide(lac, sm, state, torque=-0.15, v=20.0, desired=0.003,
                      measured=0.0, push_angle=2.5)
    assert state['budget_spent'] is True
    step_max = 0.080769   # interp(20, [15,28], [0.10,0.05])
    assert state['target_frac'] == pytest.approx(step_max, abs=1e-5)
    assert state['target_frac'] != pytest.approx(-0.15)   # did NOT freeze
    assert state['torque'] == pytest.approx(-0.126923, abs=1e-5)


def test_budget_clamp_reversal_smaller_counter_target_is_still_bounded(monkeypatch):
    """Case D: the P-law reversing with a counter-target SMALLER in magnitude
    than torque — the old code's other failure mode. Since |target|>|torque|
    was false, it skipped the freeze branch entirely and applied the raw,
    unclamped step (a Delta 0.547 frac swing here — precisely what STEP_MAX
    exists to prevent). Must bound to zero plus one step_max, same as case C."""
    lac, sm, mod, state = _make_controller(monkeypatch, angle_budget=True)
    _prime_and_decide(lac, sm, state, torque=-0.313, v=20.0, desired=0.0022,
                      measured=0.0, push_angle=2.5)
    assert state['budget_spent'] is True
    step_max = 0.080769
    assert state['target_frac'] == pytest.approx(step_max, abs=1e-5)
    assert state['torque'] == pytest.approx(-0.273623, abs=1e-5)


def test_budget_clamp_same_direction_harder_freezes_right_hand(monkeypatch):
    """Minor (review): mirror of Case A with all signs flipped — every clamp
    test above drives a LEFT (negative-torque) push; this pins the identical
    freeze on a RIGHT (positive-torque, negative-angle) push, so a sign bug
    that only shows up on one side of center can't hide behind an
    all-one-direction suite."""
    lac, sm, mod, state = _make_controller(monkeypatch, angle_budget=True)
    _prime_and_decide(lac, sm, state, torque=0.30, v=20.0, desired=0.003,
                      measured=0.0, push_angle=-2.5)
    assert state['budget_spent'] is True
    assert state['target_frac'] == pytest.approx(0.30, abs=1e-6)
    assert state['torque'] == pytest.approx(0.30, abs=1e-6)    # unmoved


def test_budget_recovers_after_wheel_goes_static(monkeypatch):
    """Critical 1 (review fix): a spent push must not lock torque authority
    out for the rest of the turn. Before this fix, push_ref cleared only
    when action left 'ramp' — and action stays 'ramp' for as long as
    |delta_err| > HOLD_BAND, so once budget_spent latched, the same-side
    freeze pinned torque for the rest of the push (reproduced on the shipped
    controller: budget spends at 1.74 Nm, the wheel re-sticks, and over 3 s
    the controller adds 0.00 Nm — pinned below the 2.0-2.75 Nm rack
    breakaway, unable to ever break the rack free again; with the toggle off
    the same scenario ramps to 12 Nm). Two real decisions chained by hand
    (not _prime_and_decide, which would re-prime push_ref=0.0 for both):
    decision 1 spends the budget and freezes torque (same shape as Case A
    above); decision 2 forces the next decision (tick_count>=cadence)
    WITHOUT touching torque/action/push_ref, and keeps push_angle at the
    same 2.5 deg — the wheel hasn't moved since decision 1 re-armed the
    reference. That must both un-spend the budget and let the P-law ask for
    (and get) more torque than decision 1's frozen value; before the fix,
    push_ref would still read the original 0.0 here, push_moved would still
    be 2.5, and budget_spent would still be True."""
    lac, sm, mod, state = _make_controller(monkeypatch, angle_budget=True)

    _prime_and_decide(lac, sm, state, torque=-0.30, v=20.0, desired=-0.003,
                      measured=0.0, push_angle=2.5)
    assert state['budget_spent'] is True
    assert state['torque'] == pytest.approx(-0.30, abs=1e-6)      # frozen, as Case A
    torque_after_spend = state['torque']

    state['ramp_frames'] = 0
    state['tick_count'] = 999
    _set_measured(sm, 20.0, 0.0)
    _call_update(lac, -0.003, v_ego=20.0, steering_angle_deg=2.5)

    assert state['push_moved'] == pytest.approx(0.0, abs=1e-9)     # re-armed at 2.5, wheel static since
    assert state['budget_spent'] is False                          # recovered, not still spent
    assert abs(state['target_frac']) > abs(torque_after_spend)     # no longer frozen at -0.30
    assert abs(state['torque']) > abs(torque_after_spend)          # torque actually climbed again


def test_budget_unspent_large_target_still_step_max_limited(monkeypatch):
    """Important 3 (review): every clamp test above primes budget_spent=True.
    Changing the production check from `if state['budget_spent']:` to
    `if _angle_budget_on:` would pass the entire rest of this suite while
    silently removing STEP_MAX from every ramp decision whenever the toggle
    is on, regardless of whether any push has actually spent its budget. Pin
    the un-spent path explicitly: toggle on, wheel has barely moved (well
    under BUDGET_DEG), a deep-curve target that would saturate all the way to
    STEER_MAX if unclamped — the step this decision takes must still be
    bounded to STEP_MAX, exactly like the toggle-off path.

    torque=-0.20 (not 0.0) is load-bearing: with torque==0.0 the mutated
    branch's `lo = min(0, torque) - step_max` / `hi = max(0, torque) +
    step_max` collapses to exactly [-step_max, +step_max] around zero, the
    same range the correct un-spent path produces — a zero-torque prime
    can't tell the two apart. A nonzero same-direction torque makes the
    mutated range asymmetric ([-0.28077, +0.08077] here) and wrong."""
    lac, sm, mod, state = _make_controller(monkeypatch, angle_budget=True)
    _prime_and_decide(lac, sm, state, torque=-0.20, v=20.0, desired=-0.02,
                      measured=0.0, push_angle=0.1)
    assert state['budget_spent'] is False
    assert state['target_frac'] == pytest.approx(-0.280769, abs=1e-5)
    assert state['torque'] == pytest.approx(-0.208077, abs=1e-5)


# ============================================================
# Hot toggle via the 'angle_budget' plugin-bus topic (2026-08-13)
# ============================================================

@pytest.fixture(autouse=True)
def _isolate_budget_socket(monkeypatch, tmp_path):
  """Keep every test off the real /tmp/plugin_bus path: a live UI on the C3
  (or a stray socket on a dev box) must never make the suite attach a real
  subscriber. Tests that need the socket to exist re-patch it themselves."""
  import bmw.latcontroller as _mod
  monkeypatch.setattr(_mod, '_BUDGET_BUS_SOCKET', str(tmp_path / 'no_socket'))


def _touch_budget_socket():
  """Create the (test-patched) socket path so an injected fake sub survives
  the close-on-missing-socket branch."""
  import bmw.latcontroller as _mod
  open(_mod._BUDGET_BUS_SOCKET, 'w').close()


class _FakeBudgetSub:
  def __init__(self, msgs):
    self._msgs = list(msgs)

  def recv(self):
    return self._msgs.pop(0) if self._msgs else None

  def close(self):
    pass


class TestBudgetHotToggle:
  def test_bus_enable_applies_mid_drive(self, monkeypatch):
    # Init with the toggle OFF; a bus message flips it on without restart.
    lac, sm, mod, state = _make_controller(monkeypatch)
    assert state['budget_on'] is False
    _touch_budget_socket()
    state['budget_sub'] = _FakeBudgetSub([('angle_budget', {'enabled': True})])
    _set_measured(sm, 20.0, 0.001)
    _call_update(lac, 0.002, steering_angle_deg=0.0)
    assert state['budget_on'] is True

  def test_bus_disable_stops_budget_spending(self, monkeypatch):
    # Init ON, spend the budget with a 2.5-deg jump inside one decision
    # window (per-decision re-arm makes slow multi-decision crawls unspendable
    # by design), then disable over the bus: the SAME jump pattern must stop
    # spending.
    lac, sm, mod, state = _make_controller(monkeypatch, angle_budget=True)
    _drive(lac, sm, state, [0.0, 2.5])
    assert state['budget_spent'] is True
    _touch_budget_socket()
    state['budget_sub'] = _FakeBudgetSub([('angle_budget', {'enabled': False})])
    _drive(lac, sm, state, [5.0, 7.5, 10.0, 12.5])
    assert state['budget_on'] is False
    assert state['budget_spent'] is False

  def test_no_publisher_keeps_init_state(self, monkeypatch, tmp_path):
    import bmw.latcontroller as mod_direct
    monkeypatch.setattr(mod_direct, '_BUDGET_BUS_SOCKET',
                        str(tmp_path / 'nonexistent_socket'))
    lac, sm, mod, state = _make_controller(monkeypatch, angle_budget=True)
    for _ in range(10):
      _set_measured(sm, 20.0, 0.001)
      _call_update(lac, 0.002, steering_angle_deg=0.0)
    assert state['budget_sub'] is None
    assert state['budget_on'] is True

  def test_malformed_bus_message_is_ignored(self, monkeypatch):
    lac, sm, mod, state = _make_controller(monkeypatch, angle_budget=True)
    _touch_budget_socket()
    state['budget_sub'] = _FakeBudgetSub([('angle_budget', {})])  # no 'enabled' key
    _set_measured(sm, 20.0, 0.001)
    _call_update(lac, 0.002, steering_angle_deg=0.0)
    assert state['budget_on'] is True   # default falls back to current state

  def test_lazy_subscriber_creation_pins_import_path_and_topic(self, monkeypatch, tmp_path):
    """Mutation-proof the creation branch: a typo in the plugin_bus import
    path, the PluginSub topic list, or the message filter must fail HERE
    (the except-pass guard otherwise ships a permanently dead feature)."""
    import bmw.latcontroller as mod_direct
    sock = tmp_path / 'angle_budget'
    sock.write_text('')                       # socket "exists" -> creation runs
    monkeypatch.setattr(mod_direct, '_BUDGET_BUS_SOCKET', str(sock))

    constructed = []

    class FakeSub:
      def __init__(self, topics):
        constructed.append(topics)
        self._q = [('angle_budget', {'enabled': True})]

      def recv(self):
        return self._q.pop(0) if self._q else None

      def close(self):
        pass

    fake_mod = MagicMock()
    fake_mod.PluginSub = FakeSub
    monkeypatch.setitem(sys.modules, 'openpilot.selfdrive.plugins.plugin_bus', fake_mod)

    lac, sm, mod, state = _make_controller(monkeypatch)
    _set_measured(sm, 20.0, 0.001)
    _call_update(lac, 0.002, steering_angle_deg=0.0)
    assert constructed == [['angle_budget']]
    assert state['budget_on'] is True

  def test_subscriber_closed_when_socket_disappears(self, monkeypatch, tmp_path):
    lac, sm, mod, state = _make_controller(monkeypatch)
    closed = []
    state['budget_sub'] = SimpleNamespace(recv=lambda: None,
                                          close=lambda: closed.append(True))
    # autouse fixture points _BUDGET_BUS_SOCKET at a nonexistent path
    _set_measured(sm, 20.0, 0.001)
    _call_update(lac, 0.002, steering_angle_deg=0.0)
    assert closed == [True]
    assert state['budget_sub'] is None

  def test_telemetry_carries_budget_on(self, monkeypatch):
    lac, sm, mod, state = _make_controller(monkeypatch)
    _touch_budget_socket()
    payloads = []
    state['lat_pub'] = SimpleNamespace(send=payloads.append)
    _set_measured(sm, 20.0, 0.001)
    _call_update(lac, 0.002, steering_angle_deg=0.0)
    assert payloads and payloads[-1]['budget_on'] is False
    state['budget_sub'] = _FakeBudgetSub([('angle_budget', {'enabled': True})])
    _set_measured(sm, 20.0, 0.001)
    _call_update(lac, 0.002, steering_angle_deg=0.0)
    assert payloads[-1]['budget_on'] is True


# ============================================================
# Entry/settle hysteresis on the HOLD_BAND rest band (2026-08-13, route 3f4
# data). One shared threshold made the controller flicker across the
# boundary ~89x/min on straights (error noise sigma = 0.00081 rad); leaving
# rest now requires clearing HOLD_BAND_ENTER (0.0015), settling back still
# only needs HOLD_BAND (0.001) -- the settle point is unchanged.
#
# _drive_to(lac, sm, state, E) places delta_err at exactly E rad (measured
# pinned to 0, so delta_err == delta_des == atan(desired*L); desired is
# chosen as tan(E)/L so atan(tan(E)) round-trips to E) and forces the cadence
# decision to run this tick (tick_count = 999). Every scenario below was
# verified against the real controller before being pinned here -- see
# hysteresis-report.md for the derivation, including the one case
# (E=0.0012 continuing a correction) where the decision that tick is
# cancel_tol rather than a fresh hysteresis-branch evaluation: at_rest is
# only ever written inside the cadence decision block, so when cancel_tol
# preempts that block (it fires first, resets tick_count to 0, and the
# cadence condition then reads false), at_rest simply keeps whatever value
# the prior decision left it at -- which is exactly the "still correcting"
# assertion this test makes, just reached by a different code path than the
# plain else-branch case exercises on its own.
# ============================================================

def _drive_to(lac, sm, state, err_rad, v=20.0, wheelbase=2.66):
  """Force delta_err to err_rad (measured=0) and run one cadence decision."""
  desired = math.tan(err_rad) / wheelbase
  _set_measured(sm, v, 0.0)
  state['tick_count'] = 999
  _call_update(lac, desired, v_ego=v)


def test_between_settle_and_enter_from_rest_stays_holding(monkeypatch):
  """err = 0.0012 sustained from a resting controller: legacy single
  threshold (0.0012 > HOLD_BAND=0.001) would ramp; hysteresis's leave-rest
  gate (HOLD_BAND_ENTER=0.0015) is not cleared, so it must stay holding."""
  lac, sm, mod, state = _make_controller(monkeypatch)
  _drive_to(lac, sm, state, 0.0012)
  assert state['at_rest'] is True
  assert state['action'] in ('hold_zero', 'hold_curve')


def test_beyond_enter_leaves_rest(monkeypatch):
  """err = 0.0018 sustained clears HOLD_BAND_ENTER (0.0015): leaves rest."""
  lac, sm, mod, state = _make_controller(monkeypatch)
  _drive_to(lac, sm, state, 0.0018)
  assert state['at_rest'] is False
  assert state['action'] == 'ramp'


def test_correcting_continues_below_enter_until_settle(monkeypatch):
  """Leave rest at 0.0018, then hold err below HOLD_BAND_ENTER but above the
  HOLD_BAND settle point: must NOT snap back to holding.

  Two sub-cases, in separate controllers so neither's decision path can mask
  the other (review fix, Important 1):

  Case 1 -- err=0.0012, the cancel_tol boundary (1.2*HOLD_BAND). cancel_tol
  fires first (it's HOLD_BAND boundary hygiene on an in-flight push ramp,
  <=, unrelated to this state machine) and resets tick_count to 0, which
  makes the cadence condition read false THIS tick -- so the hysteresis
  decision block (where at_rest is written) never runs, and at_rest merely
  carries forward its prior value. That is a real, worth-pinning interaction
  (a regression that made cancel_tol stop preempting here would matter), but
  by itself it does NOT exercise the settle branch's own comparison -- a
  mutant that widens the settle branch from `<= HOLD_BAND` to `<= _enter`
  still passes this case, because the branch is never reached.

  Case 2 -- err=0.0014: strictly above the cancel_tol gate (0.0012, so
  cancel_tol does NOT preempt) and strictly below HOLD_BAND_ENTER (0.0015).
  This is the case that actually reaches and evaluates the settle
  comparison: unmutated, 0.0014 > HOLD_BAND (0.001) so it stays correcting
  (ramp); under the rejected single-threshold-widening mutation (settle
  branch changed to `<= _enter` = 0.0015), 0.0014 would incorrectly settle.
  """
  # Case 1: cancel_tol boundary.
  lac, sm, mod, state = _make_controller(monkeypatch)
  _drive_to(lac, sm, state, 0.0018)
  assert state['at_rest'] is False
  _drive_to(lac, sm, state, 0.0012)
  assert state['action'] == 'cancel_tol'
  assert state['at_rest'] is False

  # Case 2: the settle branch itself, isolated from cancel_tol.
  lac2, sm2, mod2, state2 = _make_controller(monkeypatch)
  _drive_to(lac2, sm2, state2, 0.0018)
  assert state2['at_rest'] is False
  _drive_to(lac2, sm2, state2, 0.0014)
  assert state2['at_rest'] is False
  assert state2['action'] == 'ramp'


def test_settle_returns_to_rest(monkeypatch):
  """Continuing from the correcting state, dropping err to 0.0008 (at/below
  the unchanged HOLD_BAND settle point) returns to rest."""
  lac, sm, mod, state = _make_controller(monkeypatch)
  _drive_to(lac, sm, state, 0.0018)
  _drive_to(lac, sm, state, 0.0012)
  assert state['at_rest'] is False
  _drive_to(lac, sm, state, 0.0008)
  assert state['at_rest'] is True
  assert state['action'] in ('hold_zero', 'hold_curve')


def test_killswitch_reproduces_legacy_single_threshold(monkeypatch):
  """HoldHysteresis='0': both thresholds collapse to HOLD_BAND, reproducing
  the legacy single-comparison decision-for-decision. This is the
  legacy-identity pin: err 0.0012 (> HOLD_BAND) must RAMP, err 0.0008
  (<= HOLD_BAND) must hold, and err exactly HOLD_BAND (the boundary itself)
  must hold too -- legacy was `abs(delta_err) <= HOLD_BAND`, a <=, so
  equality is on-target. (Review fix, Minor 4: the doc claims this test pins
  "including the boundary case"; it didn't until this third construction was
  added -- 0.0012/0.0008 both sit strictly off the boundary.)

  The boundary value itself: `desired = tan(HOLD_BAND)/L` round-trips through
  `atan(desired*L)` to bit-exact 0.001 at this L (verified: no float-precision
  hair to chase here), so this asserts the exact `<=` behaviour directly
  rather than approximately."""
  lac, sm, mod, state = _make_controller(monkeypatch, hold_hyst=False)
  _drive_to(lac, sm, state, 0.0012)
  assert state['at_rest'] is False
  assert state['action'] == 'ramp'

  lac2, sm2, mod2, state2 = _make_controller(monkeypatch, hold_hyst=False)
  _drive_to(lac2, sm2, state2, 0.0008)
  assert state2['at_rest'] is True
  assert state2['action'] in ('hold_zero', 'hold_curve')

  lac3, sm3, mod3, state3 = _make_controller(monkeypatch, hold_hyst=False)
  boundary_err = 0.001   # == HOLD_BAND in latcontroller.py
  _drive_to(lac3, sm3, state3, boundary_err)
  assert state3['delta_err'] == boundary_err   # bit-exact, see docstring
  assert state3['at_rest'] is True
  assert state3['action'] in ('hold_zero', 'hold_curve')


def test_default_is_enabled(monkeypatch):
  """No param patched (angle_budget/hold_hyst both left None -> real
  read_plugin_param, which sees no param file and returns the default ''):
  hysteresis is active by default. err 0.0012 stays holding, the same
  scenario that pins hysteresis-on behaviour in
  test_between_settle_and_enter_from_rest_stays_holding above, here without
  any monkeypatch at all."""
  lac, sm, mod, state = _make_controller(monkeypatch)
  _drive_to(lac, sm, state, 0.0012)
  assert state['at_rest'] is True
  assert state['action'] in ('hold_zero', 'hold_curve')


def test_hb_enter_telemetry_reflects_killswitch(monkeypatch):
  """Review fix, Minor 3: `hb_enter` is documented as each drive's A/B
  self-label (LATERAL_CONTROLLER.md: 0.0015 = hysteresis active, 0.001 =
  kill-switched). Hardcoding `float(HOLD_BAND_ENTER)` in the payload would
  pass every other test in this file (none of them read `hb_enter`) while
  silently mislabeling every kill-switched drive. Pin both legs directly
  from the published telemetry payload, not from the internal `_enter`
  local."""
  lac, sm, mod, state = _make_controller(monkeypatch, hold_hyst=False)
  payloads = []
  state['lat_pub'] = SimpleNamespace(send=payloads.append)
  _drive_to(lac, sm, state, 0.0008)
  assert payloads and payloads[-1]['hb_enter'] == pytest.approx(0.001)

  lac2, sm2, mod2, state2 = _make_controller(monkeypatch, hold_hyst=True)
  payloads2 = []
  state2['lat_pub'] = SimpleNamespace(send=payloads2.append)
  _drive_to(lac2, sm2, state2, 0.0008)
  assert payloads2 and payloads2[-1]['hb_enter'] == pytest.approx(0.0015)


# ============================================================
# FRICTION retirement (2026-08-13): the two epsilon-gates are DELETED.
# These tests pin the deletions — restoring either conjunct fails here.
# ============================================================

class TestFrictionGatesDeleted:
  def test_cancel_tol_fires_even_for_tiny_inflight_targets(self, monkeypatch):
    """Pre-deletion, |target_frac| <= 0.05 blocked cancel_tol and tiny stale
    ramps ran to completion. Now arrival hygiene applies to every push ramp."""
    lac, sm, mod, state = _make_controller(monkeypatch)
    _set_measured(sm, 20.0, 0.0005)          # measured ~ desired: err in band
    state['torque'] = 0.02
    state['target_frac'] = 0.03              # tiny — the old gate blocked this
    state['ramp_frames'] = 5
    state['action'] = 'ramp'
    state['tick_count'] = 0                  # not a cadence decision tick
    _call_update(lac, 0.0005, steering_angle_deg=0.0)
    assert state['action'] == 'cancel_tol'

  def test_deep_relax_arms_even_with_tiny_held_torque(self, monkeypatch):
    """Pre-deletion, |torque| <= 0.05 blocked the dwell from arming."""
    lac, sm, mod, state = _make_controller(monkeypatch)
    _set_measured(sm, 9.0, 0.012)            # deep curve, |κ_meas| > 0.010
    state['torque'] = 0.02                   # tiny — the old gate blocked this
    state['action'] = 'hold_curve'
    state['tick_count'] = 0
    before = state['relax_ticks']
    # overshoot-side error: desired same side but smaller than measured
    _call_update(lac, 0.008, steering_angle_deg=0.0, v_ego=9.0)
    assert state['relax_ticks'] == before + 1
