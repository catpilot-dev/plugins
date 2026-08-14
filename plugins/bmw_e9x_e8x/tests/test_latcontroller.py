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


def _make_controller(monkeypatch, wheelbase=2.66, hold_hyst=None, stall_v2=None):
  """Load bmw.latcontroller and construct a controller instance wired to a
  FakeSubMaster. Returns (lac, fake_sm, mod, state).

  hold_hyst / stall_v2: if not None, patches config.read_plugin_param so
  HoldHysteresis / StallBreakaway reads as '1' (True) or '0' (False) — must
  happen BEFORE
  on_lat_controller_init runs, because the param is read exactly once at
  construction (review fix, Important 4: no more per-tick/cached re-read),
  so patching it *after* construction has no effect. Patches the `config`
  module's attribute, not `mod`'s (review fix, Important 2:
  on_lat_controller_init does `from config import read_plugin_param` at
  function scope, mirroring bmw/carstate.py and speedlimitd/speedlimitd.py,
  so `bmw.latcontroller` has no `read_plugin_param` module attribute to
  patch — same pattern as speedlimitd/tests/test_speedlimitd.py's
  `monkeypatch.setattr(config, 'read_plugin_param', ...)`).

  Every param this controller reads goes through that one call, so the stub
  dispatches on the param KEY rather than returning one value for every key
  (2026-08-13 — a key-blind stub would make one param silently inherit
  another's test value). A kwarg left None falls through to the `default`
  argument production passes (''), i.e. the real unpatched-param behaviour
  for that key.
  """
  import cereal.messaging as messaging
  fake_sm = FakeSubMaster([])
  monkeypatch.setattr(messaging, 'SubMaster', lambda services: fake_sm)

  import bmw.latcontroller as mod
  if hold_hyst is not None or stall_v2 is not None:
    import config
    def _param_stub(plugin_id, key, default=''):
      if key == 'HoldHysteresis' and hold_hyst is not None:
        return '1' if hold_hyst else '0'
      if key == 'StallBreakaway' and stall_v2 is not None:
        return '1' if stall_v2 else '0'
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
  """No param patched (hold_hyst left None -> real
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
# Persistent-lean escape (2026-08-14, route 3f8 on-car verdict). The entry
# gap (0.001, 0.0015) rejects symmetric noise but also hid a constant
# road-crown pull (left-hug, slow correction). A second leave-rest condition
# on the SLOW error bounds the lean latency: escape when the 2 s EMA of
# delta_err exceeds HOLD_EMA_ESCAPE = 0.0012 (strict >).
#
# One _call_update == one livePose tick, so the EMA alpha per call is
# DT_LIVEPOSE / HOLD_EMA_TAU = 0.05 / 2.0 = 0.025. A constant bias E reaches
# 0.0012 after n ticks where E*(1 - 0.975^n) > 0.0012 -- E=0.0014 needs ~78
# ticks (~3.9 s), E=0.0012 never (converges from below). The pre-existing
# hysteresis tests above drive <= 4 ticks, far from any of this.
# ============================================================

_EMA_ESCAPE = 0.0012   # == HOLD_EMA_ESCAPE in latcontroller.py


class TestPersistentLeanEscape:
  def test_sustained_lean_escapes_rest(self, monkeypatch):
    """delta_err = 0.0014 sits inside the entry gap: the plain hysteresis
    would rest forever. The 2 s EMA climbs past 0.0012 (~78 ticks) and forces
    the correction."""
    lac, sm, mod, state = _make_controller(monkeypatch, hold_hyst=True)
    for _ in range(150):
      _drive_to(lac, sm, state, 0.0014)
    assert state['at_rest'] is False
    assert state['action'] == 'ramp'
    assert abs(state['derr_ema']) > _EMA_ESCAPE

  def test_lean_at_threshold_never_escapes(self, monkeypatch):
    """delta_err = 0.0012 exactly: the EMA converges to 0.0012 from below and
    the comparison is strict >, so the escape never fires. This is also what
    keeps test_between_settle_and_enter_from_rest_stays_holding valid.

    Convergence alone does NOT pin the strictness: after 200 ticks the EMA is
    only ~0.0011924, where `>` and `>=` are indistinguishable (mutation
    survived review). So the second phase forces the EMA to the threshold
    EXACTLY and keeps it there: with derr_ema == delta_err == 0.0012 the EMA
    update is bit-identity (x + a*(x - x) == x), so every subsequent tick
    evaluates the comparison precisely AT 0.0012 -- `>` holds rest, `>=`
    escapes."""
    lac, sm, mod, state = _make_controller(monkeypatch, hold_hyst=True)
    for _ in range(200):
      _drive_to(lac, sm, state, _EMA_ESCAPE)
    assert state['at_rest'] is True
    assert state['action'] in ('hold_zero', 'hold_curve')
    assert abs(state['derr_ema']) <= _EMA_ESCAPE

    # Exact-threshold pin for the strict `>`.
    state['derr_ema'] = _EMA_ESCAPE
    for _ in range(5):
      _drive_to(lac, sm, state, _EMA_ESCAPE)
      assert state['derr_ema'] == _EMA_ESCAPE   # bit-identity, no drift
      assert state['at_rest'] is True
    assert state['action'] in ('hold_zero', 'hold_curve')

  def test_symmetric_flicker_does_not_escape(self, monkeypatch):
    """The calm-preservation pin: +/-0.0013 alternating every tick (both legs
    inside the entry gap) averages to ~0 over 2 s, so the escape stays quiet
    and the controller keeps resting."""
    lac, sm, mod, state = _make_controller(monkeypatch, hold_hyst=True)
    for i in range(200):
      _drive_to(lac, sm, state, 0.0013 if i % 2 == 0 else -0.0013)
      assert state['at_rest'] is True
    assert abs(state['derr_ema']) < _EMA_ESCAPE

  def test_ema_survives_settle_and_escape_fires(self, monkeypatch):
    """Mutation pin for the settle re-prime's REMOVAL (route 3f9, 2026-08-15).
    The re-prime reset derr_ema to the settle-point error, which starved the
    escape to zero on-car fires. A still-high lean average must survive the
    settle and immediately re-exit rest -- that is the escape working."""
    lac, sm, mod, state = _make_controller(monkeypatch, hold_hyst=True)
    _drive_to(lac, sm, state, 0.0018)
    assert state['at_rest'] is False
    # Same shape as test_settle_returns_to_rest: the 0.0012 tick is consumed
    # by cancel_tol (which preempts the cadence decision), so the settle
    # branch is only reached on the following tick.
    _drive_to(lac, sm, state, 0.0012)
    assert state['at_rest'] is False

    state['derr_ema'] = 0.005              # standing lean, far past the escape point
    _drive_to(lac, sm, state, 0.0008)      # settles (<= HOLD_BAND)
    assert state['at_rest'] is True
    # NOT re-primed to the small current error: only normal EMA decay applies.
    assert state['derr_ema'] != pytest.approx(state['delta_err'])
    assert abs(state['derr_ema']) > 0.003

    _drive_to(lac, sm, state, 0.0008)      # escape fires right back out of rest
    assert state['at_rest'] is False
    assert abs(state['derr_ema']) > _EMA_ESCAPE

  def test_killswitch_disables_escape(self, monkeypatch):
    """Mutation pin for the `_hold_hyst_on and ...` gate: with the
    kill-switch off, an EMA well past the escape threshold must not move
    at_rest -- HoldHysteresis='0' is exact legacy behaviour."""
    lac, sm, mod, state = _make_controller(monkeypatch, hold_hyst=False)
    state['derr_ema'] = 0.005
    for _ in range(3):
      _drive_to(lac, sm, state, 0.0009)    # below HOLD_BAND: legacy holds
      assert state['at_rest'] is True
    # The escape condition WOULD have been true throughout (decays slowly).
    assert abs(state['derr_ema']) > _EMA_ESCAPE
    assert state['action'] in ('hold_zero', 'hold_curve')

  def test_telemetry_has_derr_ema(self, monkeypatch):
    lac, sm, mod, state = _make_controller(monkeypatch, hold_hyst=True)
    payloads = []
    state['lat_pub'] = SimpleNamespace(send=payloads.append)
    _drive_to(lac, sm, state, 0.0008)
    assert payloads
    assert isinstance(payloads[-1]['derr_ema'], float)
    assert payloads[-1]['derr_ema'] == pytest.approx(state['derr_ema'])


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


# ============================================================
# STEP_MAX speed schedule (route 39b seg 18, 2026-07-09 — user safety call)
# and the steeringAngleDeg getattr guard.
#
# Both properties used to be pinned only incidentally, by assertions inside
# the steering-push tests deleted with that feature (2026-08-14). They are
# live behaviour with no owner otherwise, so they are re-pinned here directly,
# with no dependence on the retired mechanism.
# ============================================================

def _decide_from_zero(lac, sm, state, *, v, desired):
  """Run ONE cadence decision from zero standing torque at speed v.

  Zero torque is load-bearing: held = hold_f * torque = 0, so held_target is 0
  and the hold-floor cannot touch target_frac (it needs target_frac *
  held_target > 0). measured = 0 keeps the relax-dwell disarmed (it needs
  |κ_meas| > RELAX_DWELL_KAPPA) and makes delta_err == delta_des.
  """
  state['torque'] = 0.0
  state['target_frac'] = 0.0
  state['ramp_frames'] = 0
  state['action'] = 'idle'
  state['tick_count'] = 999          # force the cadence decision this tick
  _set_measured(sm, v, 0.0)
  _call_update(lac, desired, v_ego=v)


class TestStepMaxSpeedSchedule:
  """`step_max = interp(vEgo, STEP_MAX_V=[15, 28], STEP_MAX_BP=[0.10, 0.05])`.

  Route 39b seg 18 (2026-07-09): a slight highway left produced sudden
  back-and-forth wheel motion. The user's ruling was that aggressive
  per-decision steps are riskier at highway speed (lane margin is consumed
  faster, less time to react), so the cap halves from 0.10 to 0.05 between
  15 and 28 m/s while curves below 15 m/s keep full entry authority.
  A constant `step_max = 0.10` — i.e. the schedule dropped — is exactly the
  regression these three speeds catch.

  Derivation of the expected numbers (from the constants, not from a
  controller run). With measured = 0 and desired = -0.02:

      delta_err = delta_des = atan(-0.02 * 2.66)     = -0.0531489 rad
      kappa_scale = interp(0.02, [0.001, 0.01, 0.02], [1.0, 2.5, 3.0]) = 3.0
      target_nm  = 1.0 * 3.0 * v^2 * delta_err        (T_CAP_SLOPE_BASE = 1.0)
      t_cap_nm   = min(12, 2.0 + 3.0 * v^2 * 0.0531489)

  At every speed here t_cap_nm saturates at STEER_MAX = 12 (t_cap_frac = 1.0)
  and |target_nm / 12| >= 1.0, so the P target clips to -1.0 — far past any
  step cap. From torque = 0 the decision is therefore `step = clip(-1.0, 0,
  -step_max, +step_max) = -step_max` and `target_frac = -step_max`: the
  commanded target IS the schedule value, so these assertions read the
  schedule directly.

      v = 15  ->  step_max = 0.10       (first breakpoint, flat below)
      v = 20  ->  step_max = 0.10 + (20-15)/(28-15) * (0.05-0.10) = 0.0807692
      v = 28  ->  step_max = 0.05       (second breakpoint, flat above)

  The intermediate speed is what pins the INTERPOLATION rather than just the
  two endpoints: a schedule replaced by `0.10 if v < 28 else 0.05` would pass
  on 15 and 28 alone.

  One CAN tick of ramp is then applied before update() returns:
  spread_frames = action_cadence_ticks * 5, and with the harness's default
  lat_delay = 0.2 -> model_action_t = 0.25 -> half = 0.125 s ->
  action_cadence_ticks = round(2.5) = 2 (banker's rounding) -> 10 frames.
  So state['torque'] lands on -step_max / 10.
  """

  DESIRED = -0.02      # deep target: P saturates, so the step cap is what binds

  def test_full_step_at_and_below_the_first_breakpoint(self, monkeypatch):
    lac, sm, mod, state = _make_controller(monkeypatch)
    _decide_from_zero(lac, sm, state, v=15.0, desired=self.DESIRED)
    assert state['action'] == 'ramp'
    assert state['target_frac'] == pytest.approx(-0.10, abs=1e-9)
    assert state['torque'] == pytest.approx(-0.010, abs=1e-9)

  def test_half_step_at_and_above_the_second_breakpoint(self, monkeypatch):
    lac, sm, mod, state = _make_controller(monkeypatch)
    _decide_from_zero(lac, sm, state, v=28.0, desired=self.DESIRED)
    assert state['action'] == 'ramp'
    assert state['target_frac'] == pytest.approx(-0.05, abs=1e-9)
    assert state['torque'] == pytest.approx(-0.005, abs=1e-9)

  def test_interpolated_step_between_the_breakpoints(self, monkeypatch):
    """20 m/s sits strictly inside the ramp: 0.10 - 5/13*0.05 = 0.0807692.
    This is the case a two-level step function would get wrong."""
    lac, sm, mod, state = _make_controller(monkeypatch)
    _decide_from_zero(lac, sm, state, v=20.0, desired=self.DESIRED)
    assert state['action'] == 'ramp'
    assert state['target_frac'] == pytest.approx(-0.0807692307, abs=1e-9)
    assert state['torque'] == pytest.approx(-0.00807692307, abs=1e-9)

  def test_step_cap_shrinks_monotonically_with_speed(self, monkeypatch):
    """The safety property in one assertion, independent of exact values: a
    faster decision may never take a LARGER torque step than a slower one."""
    steps = []
    for v in (15.0, 20.0, 28.0):
      lac, sm, mod, state = _make_controller(monkeypatch)
      _decide_from_zero(lac, sm, state, v=v, desired=self.DESIRED)
      steps.append(abs(state['target_frac']))
    assert steps[0] > steps[1] > steps[2]


class TestSteeringAngleGuard:
  def test_missing_steering_angle_degrades_safely(self, monkeypatch):
    """`_angle = float(getattr(CS, 'steeringAngleDeg', 0.0))` runs every CAN
    tick before anything else in update(). It is not inside the telemetry
    try/except, so a bare attribute access would propagate an AttributeError
    straight out of the control loop on any CS lacking the field. Pin the
    default: a CS stub without steeringAngleDeg must complete update()
    normally, decision path and all."""
    lac, sm, mod, state = _make_controller(monkeypatch)
    CS = SimpleNamespace(vEgo=20.0)          # no steeringAngleDeg
    _set_measured(sm, 20.0, 0.001)
    state['tick_count'] = 999                # force a full cadence decision
    out = lac.update(True, CS, None, None, False, 0.008, False, 0.2)
    assert out is not None
    assert state['action'] == 'ramp'         # the decision really did run


# ============================================================
# Stall/breakaway v2 — displacement trip (2026-08-15, routes 3f2 + 3f4
# replay-validated). A stalled rack that releases sweeps the wheel far past
# what the model is asking for while the controller still pushes on a stale
# kappa-space error (every kappa-derived signal lags the rack at release).
#
#   ARM       action=='ramp' and the 0.4 s angle ring spans < 2 quanta
#   BREAKAWAY >= 3 quanta of advance inside 0.2 s; latches sb_brk_angle and
#             sb_dir (the OBSERVED motion direction over that window)
#   TRIP      all four: not relax_dwell; |kappa_des| <= SB_TRIP_KAPPA_MAX;
#             the wheel has travelled >= SB_TRIP_DISP_DEG past the breakaway
#             point IN THE BREAKAWAY DIRECTION; and it is sweeping at
#             >= SB_TRIP_RATE_DPS -> shed to 0 over SB_SHED_FRAMES + block
#             same-side pushes
#
# Everything is measured against the wheel's own breakaway state, so there is
# no absolute angle or curvature convention in the machine at all — which is
# why there is no target derivation, no offset estimator and no readiness
# gate to test here. The three trip discriminators each have a pin:
#   - deep-curve gate      -> test_deep_curve_no_trip
#   - displacement direction -> test_wrong_direction_no_trip
#   - rate gate            -> test_slow_crossing_no_trip
#
# Harness: one _call_update is one CAN tick AND one livePose tick, so tick
# counts map straight onto the SB_* constants.
# ============================================================

_SB_Q = 0.04395         # == ANGLE_QUANTUM_DEG
_SB_V = 20.0
# Mild curve, like the route 3f2 seg 10 lurch (kappa_des -0.0035): inside
# SB_TRIP_KAPPA_MAX, so the deep-curve gate is open.
_SB_KAPPA_DES = -0.004
# Under-tracking on the same side, and shallow enough that relax_dwell stays
# disarmed (it needs |kappa_meas| > RELAX_DWELL_KAPPA).
_SB_KAPPA_MEAS = -0.002
_SB_A0 = 5.0            # arbitrary start angle — only deltas from it matter


def _sb_make(monkeypatch, *, enabled=True, kappa_meas=_SB_KAPPA_MEAS):
  """Controller with the stall/breakaway feature on (or at its default)."""
  lac, sm, mod, state = _make_controller(
    monkeypatch, stall_v2=True if enabled else None)
  _set_measured(sm, _SB_V, kappa_meas)
  return lac, sm, mod, state


def _sb_tick(lac, angle, action='ramp', kappa_des=_SB_KAPPA_DES):
  """One CAN+livePose tick at `angle`, with the decision action forced.

  tick_count is pinned to 0 every tick so no cadence decision ever runs
  during the choreography: the state machine reads state['action'] BEFORE
  the livePose branch, so forcing it here is what makes the arm/episode
  conditions deterministic, and suppressing the decision keeps the trip's
  own target_frac / ramp_step / ramp_frames writes observable.
  """
  state = _closure_state(lac.update)
  state['action'] = action
  state['tick_count'] = 0
  _call_update(lac, kappa_des, v_ego=_SB_V, steering_angle_deg=angle)


def _sb_arm(lac, a0=_SB_A0, ticks=45, kappa_des=_SB_KAPPA_DES):
  """Hold the angle frozen at a0 while ramping -> ARM (sb_state 1).

  The ring is SB_FROZEN_TICKS + 1 long, so it takes 41 ticks to fill; 45
  gives margin and exercises "stays armed while still frozen".
  """
  for _ in range(ticks):
    _sb_tick(lac, a0, kappa_des=kappa_des)


class TestStallBreakawayV2:
  """Displacement + rate trip.

  Choreography arithmetic, once, for every test below. After arming, the
  angle advances a fixed number of quanta per tick from _SB_A0:

    step(n quanta) = n * 0.04395 deg/tick  ->  n * 4.395 deg/s

  Breakaway lands on the first moving tick k = 1 (one step already clears
  SB_MOVE_QUANTA = 3 quanta for n >= 3), latching
  sb_brk_angle = _SB_A0 + step and sb_dir = +1.0.

  The two trip quantities then evolve as (k counted from the arm end):
    displacement = (k - 1) * step                    >= SB_TRIP_DISP_DEG (2.0)
    rate         = min(k, 20) * step / 0.2 s         >= SB_TRIP_RATE_DPS (30)
  The rate is measured over a FIXED 0.2 s window, so while the window still
  contains frozen samples (k < 20) it reads the average over that window,
  not the instantaneous step — which is why the rate conjunct is the one
  that binds in the fast case even though displacement clears much earlier.
  """
  # 7 quanta/tick = 0.30765 deg/tick = 30.765 deg/s: past SB_TRIP_RATE_DPS.
  FAST_STEP = 7 * _SB_Q
  # 5 quanta/tick = 0.21975 deg/tick = 21.975 deg/s: short of it (and short
  # of the 25 the rate sweep also rejected).
  SLOW_STEP = 5 * _SB_Q
  # Fast case: displacement clears 2.0 deg at k = 8 ((8-1)*0.30765 = 2.15),
  # the rate window fills at k = 20 (20*0.30765/0.2 = 30.765 >= 30) while
  # k = 19 reads only 29.23 -> the trip lands exactly on k = 20.
  TRIP_K = 20

  def test_default_off_inert(self, monkeypatch):
    """No StallBreakaway param (production default): the machine never runs,
    even with the full trip choreography played out."""
    lac, sm, mod, state = _sb_make(monkeypatch, enabled=False)
    _sb_arm(lac)
    assert state['sb_state'] == 0
    for k in range(1, self.TRIP_K + 5):
      _sb_tick(lac, _SB_A0 + k * self.FAST_STEP)
    assert state['sb_state'] == 0
    assert state['sb_trips'] == 0
    assert state['sb_block'] == 0

  def test_arm_requires_frozen_ramp(self, monkeypatch):
    """A frozen angle arms only while the controller is actually pushing
    (action == 'ramp'). Frozen while holding is just a car going straight."""
    lac, sm, mod, state = _sb_make(monkeypatch)
    _sb_arm(lac)
    assert state['sb_state'] == 1

    lac2, sm2, mod2, state2 = _sb_make(monkeypatch)
    for _ in range(45):
      _sb_tick(lac2, _SB_A0, action='hold_zero')
    assert state2['sb_state'] == 0

  def test_breakaway_and_trip(self, monkeypatch):
    """The motivating event, in miniature: the stalled rack releases and
    sweeps 2 deg past where it broke free, fast, in a mild curve.

    See the class docstring for the arithmetic: displacement clears at
    k = 8, the 0.2 s rate window reaches 30.765 deg/s at k = 20 (k = 19 is
    29.23), so the trip lands on k = 20 — which pins the rate boundary to a
    single tick.

    Torque is NEGATIVE: sb_dir = +1.0 is leftward wheel motion, and torque
    is opposite in sign to angle, so the push driving it is < 0.
    """
    lac, sm, mod, state = _sb_make(monkeypatch)
    _sb_arm(lac)
    assert state['sb_state'] == 1

    _sb_tick(lac, _SB_A0 + self.FAST_STEP)
    assert state['sb_state'] == 2                      # breakaway
    assert state['sb_dir'] == 1.0                      # moved in +angle
    assert state['sb_brk_angle'] == pytest.approx(_SB_A0 + self.FAST_STEP)

    state['torque'] = -0.3                             # standing left push at release
    for k in range(2, self.TRIP_K):
      _sb_tick(lac, _SB_A0 + k * self.FAST_STEP)
      assert state['sb_trips'] == 0
    # Displacement was satisfied long before the rate window filled.
    assert (self.TRIP_K - 2) * self.FAST_STEP > mod.SB_TRIP_DISP_DEG
    assert (self.TRIP_K - 1) * self.FAST_STEP / 0.2 < mod.SB_TRIP_RATE_DPS

    _sb_tick(lac, _SB_A0 + self.TRIP_K * self.FAST_STEP)
    assert mod.SB_MOVE_TICKS * self.FAST_STEP / 0.2 >= mod.SB_TRIP_RATE_DPS
    assert state['sb_trips'] == 1
    assert state['sb_state'] == 0                      # one trip per episode
    assert state['target_frac'] == 0.0
    assert state['ramp_step'] == pytest.approx(0.3 / mod.SB_SHED_FRAMES)
    # One shed frame is consumed by the same tick's ramp application at the
    # bottom of update(), so SB_SHED_FRAMES - 1 remain when we look.
    assert state['ramp_frames'] == mod.SB_SHED_FRAMES - 1
    assert state['torque'] == pytest.approx(-0.3 + 0.3 / mod.SB_SHED_FRAMES)
    assert state['sb_block'] == mod.SB_BLOCK_TICKS

  def test_slow_crossing_no_trip(self, monkeypatch):
    """THE RATE-GATE PIN. Same release shape at 5 quanta/tick = 21.975 deg/s:
    displacement clears comfortably, but ordinary post-stick corrections live
    down here (the 3f4 sweep put >= 25 at 0.31 trips/min and >= 30 at 0.153),
    so it must not trip. Note 21.975 would have passed a 20 deg/s gate."""
    lac, sm, mod, state = _sb_make(monkeypatch)
    _sb_arm(lac)
    for k in range(1, 61):
      _sb_tick(lac, _SB_A0 + k * self.SLOW_STEP)
    # Displacement is satisfied many times over...
    assert 59 * self.SLOW_STEP > mod.SB_TRIP_DISP_DEG
    assert state['sb_state'] == 2                      # episode still live
    # ...and the rate never reaches the gate, even with a full window.
    assert mod.SB_MOVE_TICKS * self.SLOW_STEP / 0.2 < mod.SB_TRIP_RATE_DPS
    assert state['sb_trips'] == 0

  def test_deep_curve_no_trip(self, monkeypatch):
    """THE DEEP-CURVE PIN. Identical fast release, but |kappa_des| = 0.012 is
    past SB_TRIP_KAPPA_MAX: self-aligning torque at that loading arrests the
    release on its own (hairpins tracked fine and dominated the false trips
    before this gate), and shedding there would be the give-up-mid-turn
    failure mode this controller has re-earned three times."""
    deep = 0.012
    lac, sm, mod, state = _sb_make(monkeypatch)
    assert deep > mod.SB_TRIP_KAPPA_MAX
    _sb_arm(lac, kappa_des=deep)
    assert state['sb_state'] == 1
    for k in range(1, self.TRIP_K + 6):
      _sb_tick(lac, _SB_A0 + k * self.FAST_STEP, kappa_des=deep)
    assert state['sb_state'] == 2                      # episode live, just no trip
    assert state['sb_trips'] == 0

  def test_wrong_direction_no_trip(self, monkeypatch):
    """THE DIRECTION PIN. The rack breaks away one way, then travels the
    OTHER way. Displacement is signed by sb_dir — the direction the wheel
    actually broke free in, observed rather than assumed — so travel back
    the other way never accumulates toward the trip, however fast it is."""
    lac, sm, mod, state = _sb_make(monkeypatch)
    _sb_arm(lac)
    _sb_tick(lac, _SB_A0 + self.FAST_STEP)
    assert state['sb_state'] == 2
    assert state['sb_dir'] == 1.0
    brk = state['sb_brk_angle']

    for k in range(1, 41):
      _sb_tick(lac, brk - k * self.FAST_STEP)
    last = brk - 40 * self.FAST_STEP
    # Plenty of travel, plenty fast — but the wrong way.
    assert abs(last - brk) > mod.SB_TRIP_DISP_DEG
    assert mod.SB_MOVE_TICKS * self.FAST_STEP / 0.2 >= mod.SB_TRIP_RATE_DPS
    assert (last - brk) * state['sb_dir'] < 0.0
    assert state['sb_state'] == 2
    assert state['sb_trips'] == 0

  def test_relax_dwell_no_trip(self, monkeypatch):
    """The dwell gate: while relax_dwell is bridging a kappa_des dip the trip
    is suppressed — certified doctrine, and it costs nothing. The follow-up
    tick proves the setup was otherwise fully trip-ready."""
    lac, sm, mod, state = _sb_make(monkeypatch)
    _sb_arm(lac)
    _sb_tick(lac, _SB_A0 + self.FAST_STEP)
    assert state['sb_state'] == 2
    for k in range(2, self.TRIP_K):
      _sb_tick(lac, _SB_A0 + k * self.FAST_STEP)

    _sb_tick(lac, _SB_A0 + self.TRIP_K * self.FAST_STEP, action='relax_dwell')
    assert state['sb_trips'] == 0
    assert state['sb_state'] == 2          # episode still live, not consumed

    _sb_tick(lac, _SB_A0 + (self.TRIP_K + 1) * self.FAST_STEP)
    assert state['sb_trips'] == 1

  def test_block_suppresses_same_side_only(self, monkeypatch):
    """THE SIGN PIN for the post-trip block. sb_dir = +1.0 means the wheel
    broke away moving LEFT in angle space; torque is opposite in sign to
    angle, so the push driving that motion is target_frac < 0. That push must
    be zeroed while sb_block runs; a RIGHT push (opposite-side correction)
    must pass through untouched — the controller may stop pushing, never give
    up correcting.

    Expected magnitudes come from the STEP_MAX schedule at v=20 (see
    TestStepMaxSpeedSchedule): |target_frac| = 0.0807692.
    """
    lac, sm, mod, state = _make_controller(monkeypatch, stall_v2=True)
    state['sb_block'] = 10
    state['sb_dir'] = 1.0
    _decide_from_zero(lac, sm, state, v=20.0, desired=-0.02)   # LEFT push
    assert state['action'] == 'ramp'
    assert state['sb_block'] > 0
    assert state['target_frac'] == 0.0

    lac2, sm2, mod2, state2 = _make_controller(monkeypatch, stall_v2=True)
    state2['sb_block'] = 10
    state2['sb_dir'] = 1.0
    _decide_from_zero(lac2, sm2, state2, v=20.0, desired=0.02)  # RIGHT push
    assert state2['action'] == 'ramp'
    assert state2['sb_block'] > 0
    assert state2['target_frac'] == pytest.approx(0.0807692307, abs=1e-9)

  def test_block_does_not_zero_a_relax_dwell_hold(self, monkeypatch):
    """Review Minor: the block zeroes PUSHES. By the time it runs, the
    relax-dwell bridge may have replaced target_frac with a HOLD of the
    current torque — zeroing that is the give-up-mid-turn failure mode the
    dwell exists to prevent, so the block is gated on action == 'ramp'."""
    lac, sm, mod, state = _make_controller(monkeypatch, stall_v2=True)
    v = 9.0
    _set_measured(sm, v, 0.012)          # deep curve -> dwell can arm
    state['torque'] = -0.3               # standing left push
    state['target_frac'] = -0.3
    state['ramp_frames'] = 0
    state['action'] = 'hold_curve'
    state['sb_block'] = 10
    state['sb_dir'] = 1.0                # a left push is the "same side"
    state['relax_ticks'] = 1             # already dwelling
    state['tick_count'] = 999
    # Overshoot-side kappa_des dip: same side as kappa_meas (the dwell aborts
    # on S-curve sign flips) but smaller, so delta_err opposes delta_des.
    _call_update(lac, 0.001, v_ego=v)
    assert state['action'] == 'relax_dwell'
    assert state['target_frac'] != 0.0   # the curve hold survived the block

  def test_shed_rate_enforced_through_decisions(self, monkeypatch):
    """Review Important 1: a cadence decision landing inside the shed window
    recomputes ramp_step over spread_frames, stretching the 100 ms drain to
    ~400 ms — 6x slower, on the one action the trip exists to perform (the
    3f2 event peaked 0.30 s AFTER the trip point). While the block is up, a
    too-slow same-side drain must be re-asserted at the shed rate."""
    lac, sm, mod, state = _sb_make(monkeypatch)
    _sb_arm(lac)
    _sb_tick(lac, _SB_A0 + self.FAST_STEP)
    state['torque'] = -0.3
    for k in range(2, self.TRIP_K + 1):
      _sb_tick(lac, _SB_A0 + k * self.FAST_STEP)
    assert state['sb_trips'] == 1
    assert state['sb_block'] > 0
    torque = state['torque']
    assert torque < 0.0

    # A cadence decision lands: same destination (0), 4x the frames.
    state['target_frac'] = 0.0
    state['ramp_step'] = -torque / 40.0
    state['ramp_frames'] = 40

    _sb_tick(lac, _SB_A0 + (self.TRIP_K + 1) * self.FAST_STEP)
    assert state['ramp_step'] == pytest.approx(-torque / mod.SB_SHED_FRAMES)
    assert state['ramp_frames'] == mod.SB_SHED_FRAMES - 1   # one frame applied
    assert state['target_frac'] == 0.0

    # An opposite-side (counter-steer) ramp is NOT re-asserted: it already
    # drains the same-side torque at least as fast, and the block must never
    # interfere with correcting the other way.
    state['ramp_step'] = -torque          # big, opposite to torque
    state['ramp_frames'] = 5
    _sb_tick(lac, _SB_A0 + (self.TRIP_K + 2) * self.FAST_STEP)
    assert state['ramp_step'] == pytest.approx(-torque)
    assert state['ramp_frames'] == 4

  def test_episode_timeout_rearms_clean(self, monkeypatch):
    """No trip within SB_EPISODE_TICKS: the episode closes and a fresh
    freeze arms again. The wheel moves once (breakaway) then stalls again —
    displacement stops at one step and after 20 ticks its rate is 0."""
    lac, sm, mod, state = _sb_make(monkeypatch)
    _sb_arm(lac)
    stalled = _SB_A0 + self.FAST_STEP
    _sb_tick(lac, stalled)
    assert state['sb_state'] == 2

    for _ in range(mod.SB_EPISODE_TICKS - 1):
      _sb_tick(lac, stalled)
      assert state['sb_state'] == 2
    _sb_tick(lac, stalled)                     # the SB_EPISODE_TICKS'th decrement
    assert state['sb_state'] == 0
    assert state['sb_trips'] == 0

    _sb_tick(lac, stalled)                     # ring is uniform -> re-arms
    assert state['sb_state'] == 1

  def test_telemetry_keys(self, monkeypatch):
    """Four keys, and the absolute-target machinery's keys must be GONE —
    a stale sb_off/sb_ready in the payload would advertise an estimator this
    build does not have."""
    lac, sm, mod, state = _sb_make(monkeypatch)
    payloads = []
    state['lat_pub'] = SimpleNamespace(send=payloads.append)
    _sb_tick(lac, _SB_A0)
    assert payloads
    p = payloads[-1]
    assert isinstance(p['sb_state'], int)
    assert isinstance(p['sb_trips'], int)
    assert isinstance(p['sb_block'], int)
    assert isinstance(p['sb_on'], bool)
    assert p['sb_on'] is True
    assert 'sb_off' not in p
    assert 'sb_ready' not in p

  def test_disengage_clears(self, monkeypatch):
    """active=False mid-episode drops the whole machine: ring, state and the
    post-trip block. Nothing survives a disengagement into the next drive
    segment, where the angle history would be meaningless."""
    lac, sm, mod, state = _sb_make(monkeypatch)
    _sb_arm(lac)
    _sb_tick(lac, _SB_A0 + self.FAST_STEP)
    assert state['sb_state'] == 2
    assert len(state['sb_ring']) > 0
    state['sb_block'] = 20

    _call_update(lac, _SB_KAPPA_DES, v_ego=_SB_V, active=False,
                 steering_angle_deg=_SB_A0 + self.FAST_STEP)
    assert state['sb_state'] == 0
    assert state['sb_block'] == 0
    assert len(state['sb_ring']) == 0

  # ---- Action-half safety (review findings (b) and (c), 2026-08-15) ----
  # The trip discrimination above decides WHETHER a release happened; these
  # three pin what the machine is allowed to DO about it. The invariant they
  # share is the standing SAFETY ARCHITECTURE law: the controller may stop
  # pushing, it must never give up correcting.

  def test_rebound_snap_back_no_trip(self, monkeypatch):
    """FINDING (b) PIN — the rate conjunct must be SIGNED.

    With an unsigned rate the trip doubled as a REBOUND detector: a slow
    drift out (below the gate, so it never trips on the way out) followed by
    a fast snap BACK toward the breakaway point reads as a fast crossing
    while displacement is still decaying through the 2 deg mark. The wheel is
    travelling the WRONG way — the opposite of a release — so it must not
    trip.

    Reviewer's repro, reproduced tick-for-tick:
      39 outbound ticks at 5 quanta   -> 21.975 deg/s (sub-gate), displacement
                                         reaches 8.3505 deg, no trip
      then 20 return ticks at 7 quanta -> the 0.2 s window now holds pure
                                         return motion: |rate| = 30.765 >= 30,
                                         and displacement has decayed to
                                         exactly 2.1975 deg, still >= 2.0
    Both pre-fix conjuncts are satisfied on that single tick (j = 20 is the
    only one where they overlap — j = 19 reads 28.1 deg/s, j = 21 is at
    displacement 1.890), which is why the assertions below check it exactly.
    """
    lac, sm, mod, state = _sb_make(monkeypatch)
    _sb_arm(lac)
    for k in range(1, 40):
      _sb_tick(lac, _SB_A0 + k * self.SLOW_STEP)
    assert state['sb_state'] == 2
    assert state['sb_dir'] == 1.0
    assert state['sb_trips'] == 0                 # sub-gate on the way out
    brk = state['sb_brk_angle']
    out = _SB_A0 + 39 * self.SLOW_STEP
    assert (out - brk) > mod.SB_TRIP_DISP_DEG

    for j in range(1, 26):
      _sb_tick(lac, out - j * self.FAST_STEP)
      assert state['sb_trips'] == 0

    # The pre-fix trip tick, spelled out: displacement still past the
    # threshold, unsigned window rate past the gate — and the motion pointing
    # back toward the breakaway point, which is what now rejects it.
    at20 = out - 20 * self.FAST_STEP
    win20 = at20 - out                            # 20 ticks earlier
    assert (at20 - brk) >= mod.SB_TRIP_DISP_DEG
    assert abs(win20) / (mod.SB_MOVE_TICKS * 0.01) >= mod.SB_TRIP_RATE_DPS
    assert win20 * state['sb_dir'] < 0.0          # travelling the wrong way
    assert state['sb_state'] == 2                 # episode was live throughout

  def test_trip_with_standing_counter_steer_sheds_nothing(self, monkeypatch):
    """FINDING (c) PIN — the shed must not drain a counter-steer.

    If the standing torque at the trip tick is already OPPOSITE-side, the
    controller is counter-steering the release: exactly what it should be
    doing, and there is no same-side surplus to dump. Draining it would
    contradict the invariant the block itself asserts. The event is still
    recorded — it happened, and telemetry must show it — but nothing is
    drained and no block is armed.
    """
    lac, sm, mod, state = _sb_make(monkeypatch)
    _sb_arm(lac)
    _sb_tick(lac, _SB_A0 + self.FAST_STEP)
    assert state['sb_dir'] == 1.0
    for k in range(2, self.TRIP_K):
      _sb_tick(lac, _SB_A0 + k * self.FAST_STEP)
    assert state['sb_trips'] == 0

    # Standing torque is counter-steer (positive against sb_dir = +1), with a
    # distinctive in-flight ramp so any tampering is visible.
    state['torque'] = 0.3
    state['target_frac'] = 0.5
    state['ramp_step'] = 0.02
    state['ramp_frames'] = 3
    assert state['torque'] * (-state['sb_dir']) < 0.0

    _sb_tick(lac, _SB_A0 + self.TRIP_K * self.FAST_STEP)

    # The event is recorded...
    assert state['sb_trips'] == 1
    assert state['sb_state'] == 0
    # ...but nothing was shed and no block armed.
    assert state['target_frac'] == pytest.approx(0.5)
    assert state['ramp_step'] == pytest.approx(0.02)
    assert state['ramp_frames'] == 2               # only the normal ramp tick
    assert state['torque'] == pytest.approx(0.32)  # still counter-steering
    assert state['sb_block'] == 0

  def test_enforcement_never_drains_counter_steer(self, monkeypatch):
    """FINDING (c) PIN, second half — the shed-rate enforcement must yield.

    The enforcement's `_draining` test has a DIRECTION conjunct and a
    MAGNITUDE conjunct. A counter-steer ramp passes the first but fails the
    second, because a correction spread over the normal ~40-frame cadence
    ramp steps far slower than |torque|/SB_SHED_FRAMES. Pre-fix it was
    therefore misclassified as "not draining" and overwritten — and the
    overwrite set target_frac = 0, destroying the correction's DESTINATION,
    not just its rate.

    Reviewer's numbers: standing torque -0.27 with a fresh +0.20 counter-steer
    gives ramp_step +0.01175 against a 0.027 drain bar.
    """
    lac, sm, mod, state = _sb_make(monkeypatch)
    _sb_arm(lac)
    _sb_tick(lac, _SB_A0 + self.FAST_STEP)
    state['torque'] = -0.3
    for k in range(2, self.TRIP_K + 1):
      _sb_tick(lac, _SB_A0 + k * self.FAST_STEP)
    assert state['sb_trips'] == 1
    assert state['sb_block'] > 0
    torque = state['torque']
    assert torque < 0.0                            # same-side shed in progress

    # A counter-steer decision lands mid-shed.
    counter = 0.20
    step = (counter - torque) / 40.0
    state['target_frac'] = counter
    state['ramp_step'] = step
    state['ramp_frames'] = 40
    # It is gentler than the shed bar — the exact misclassification.
    assert abs(step) < abs(torque) / mod.SB_SHED_FRAMES

    for i in range(3):
      _sb_tick(lac, _SB_A0 + (self.TRIP_K + 1 + i) * self.FAST_STEP)
      # The torque gate is still open (torque has not yet crossed zero), so
      # the ONLY thing standing the enforcement down is the counter-steer
      # test — which makes this a clean pin.
      assert state['torque'] * (-state['sb_dir']) > 0.0
      assert state['sb_block'] > 0
      assert state['target_frac'] == pytest.approx(counter)
      assert state['ramp_step'] == pytest.approx(step)


# ============================================================
# LKA low-speed reference (2026-08-16). Below 30 km/h (lateral now runs to
# standstill in LKA mode) the torque parameters keep 8.5 m/s as their
# reference — but the MEASUREMENT must not: kappa_meas = yawRate / v_true,
# else the loop settles at kappa_des * (8.5 / v_true) and cuts inside
# low-speed turns (~2x over-curvature at 15 km/h).
# ============================================================

class TestLowSpeedLkaReference:
  def test_kappa_meas_uses_true_speed_below_30kph(self, monkeypatch):
    """kappa_meas divides yawRate by TRUE speed, not the 8.5 m/s gain floor."""
    lac, fake_sm, mod, state = _make_controller(monkeypatch)
    _set_measured(fake_sm, 5.0, 0.02)   # 18 km/h intersection turn
    _call_update(lac, 0.02, v_ego=5.0)
    assert state['measured'] == pytest.approx(0.02, rel=1e-6)

  def test_torque_gains_keep_30kph_floor(self, monkeypatch):
    """Identical kappa state at 5 and 8.5 m/s commands identical torque:
    gains/caps reference max(v, 8.5) even though the measurement does not."""
    torques = []
    for v in (5.0, 8.5):
      lac, fake_sm, mod, state = _make_controller(monkeypatch)
      _set_measured(fake_sm, v, 0.005)
      # kappa_des small enough that the P target stays below the per-decision
      # step cap — otherwise both runs saturate at step_max and can't differ
      _call_update(lac, 0.006, v_ego=v)
      torques.append(state['torque'])
    assert torques[0] == pytest.approx(torques[1], rel=1e-9)
