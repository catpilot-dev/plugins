"""Tests for the BMW lateral controller's update() call site
(on_lat_controller_init / update()).

The controller tracks the commanded curvature with Direct Tracking as its only
law: no rest state or hold band, the P target on the full error every decision,
sign-guarded reversals with free-play take-up, the LKA-regime unwind boost,
Slope Compensation. It never drains torque to respect an a_y ceiling (speedlimitd
owns a_y through v); the P-law reversing on overshoot IS the correction.

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
  - The controller patches lac.update only when plugins.lane_keeping.register
    is loaded with HOOK_READY (ruling R10); mock_deps installs that stub.
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


_LK_REGISTER = 'plugins.lane_keeping.register'


@pytest.fixture(autouse=True)
def mock_deps(monkeypatch):
  install_all_mocks(monkeypatch)
  # The controller patches lac.update only with lane_keeping loaded (ruling R10).
  monkeypatch.setitem(sys.modules, _LK_REGISTER, SimpleNamespace(HOOK_READY=True))


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


def _make_controller(monkeypatch, wheelbase=2.66, slope=None):
  """Load bmw.latcontroller and construct a controller instance wired to a
  FakeSubMaster. Returns (lac, fake_sm, mod, state).

  slope: if not None, SlopeCompensation reads as '1' (True) or '0' (False).
  Patched on the `config` module BEFORE on_lat_controller_init runs (the
  controller does `from config import read_plugin_param` at function scope and
  reads SlopeCompensation at each engagement edge). The stub dispatches on the
  param KEY; a kwarg left None falls through to the production default ('').
  """
  import cereal.messaging as messaging
  fake_sm = FakeSubMaster([])
  monkeypatch.setattr(messaging, 'SubMaster', lambda services: fake_sm)

  import bmw.latcontroller as mod
  if slope is not None:
    import config
    def _param_stub(plugin_id, key, default=''):
      if key == 'SlopeCompensation' and slope is not None:
        return '1' if slope else '0'
      return default
    monkeypatch.setattr(config, 'read_plugin_param', _param_stub)

  CP = SimpleNamespace(wheelbase=wheelbase, steerActuatorDelay=0.4)
  lac = SimpleNamespace()
  result = SimpleNamespace()
  mod.on_lat_controller_init(result, lac, CP)

  state = _closure_state(lac.update)
  return lac, fake_sm, mod, state


def _seed_learner(state, c_right, c_left):
  """Give the controller's contact learner its two contacts, as a drive's
  learning would (no persistence: every drive starts unknown)."""
  state['learner']._val = {0: c_right, 1: c_left}


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
                 steering_angle_deg=0.0, gap_deg=0.0, pressed=False, steer_rate=0.0):
  CS = SimpleNamespace(vEgo=v_ego, steeringAngleDeg=steering_angle_deg,
                       steeringAngleOffsetDeg=gap_deg, steeringPressed=pressed,
                       steeringRateDeg=steer_rate)
  return lac.update(active, CS, None, None, False, desired_curvature, False, lat_delay)


_CANCELS = ('cancel_accel', 'cancel_jerk')


# ============================================================
# (1) NEW — the SAFETY ARCHITECTURE change. Formerly-cancelling conditions now
# produce normal P-tracking. Scenario: overshoot ((κ_des - κ_meas)·κ_meas < 0)
# with high measured lateral accel (v²·κ_meas ≈ 5.6 m/s², well past
# any old ISO threshold) and into-turn torque (torque·κ_meas > 0). Kept just
# below the deep-curve threshold (|κ_meas| < RELAX_DWELL_KAPPA) so the decision
# is a plain ramp.
#
# Pre-removal controller: this cancel_accel'd and drained target_frac -> 0.
# Post-removal: it tracks — torque is commanded toward κ_des (reduced from the
# into-turn value, i.e. unwinding), NOT to zero, and action is never a cancel.
# ============================================================

class TestFormerlyCancellingNowTracks:
  V = 25.0
  DESIRED = 0.001      # κ_des collapsed near straight
  MEASURED = 0.009     # κ_meas still turning -> a_y ≈ 5.6, |κ| < deep-curve

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


# ============================================================
# (3) The decision path (ramp): torque is commanded toward the
# reference and never drained to respect a lateral-accel ceiling.
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



  def test_counter_turn_torque_runs_normal_decision(self, monkeypatch):
    """Deep in a measured curve with torque already opposing the turn (an
    active unwind in flight): never a cancel. Deep + same-side κ_des qualifies
    — the ordinary decision cadence, not a drain.
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

  measured = 0 makes delta_err == delta_des.
  """
  state['torque'] = 0.0
  state['target_frac'] = 0.0
  state['ramp_frames'] = 0
  state['action'] = 'idle'
  state['tick_count'] = 999          # force the cadence decision this tick
  _set_measured(sm, v, 0.0)
  _call_update(lac, desired, v_ego=v)


def _decide_from_torque(lac, sm, state, *, v, desired, torque):
  """One cadence decision with STANDING torque, so the step DIRECTION matters.

  measured = 0 makes delta_err == delta_des.
  """
  state['torque'] = torque
  state['target_frac'] = torque
  state['ramp_frames'] = 0
  state['action'] = 'idle'
  state['tick_count'] = 999          # force the cadence decision this tick
  _set_measured(sm, v, 0.0)
  _call_update(lac, desired, v_ego=v)


class TestRelaxDwellLowSpeedGate:
  """Route 417 (2026-08-21): at LKA intersection EXITS the dwell froze torque
  at 3.9 Nm for its full 1.0 s while the driver hauled the wheel back from
  -160 to -75 deg and delta_err grew to -0.097. The wheel's return was late
  and had to be fought. Seg 2 09:43:53.5-54.5 is the reference case; seg 16
  is the milder variant where kappa_des crossed zero and aborted the dwell
  early (its only escape is a SIGN flip -- a merely decaying reference is
  bridged for the whole second no matter how large the overshoot grows).

  The dwell guards ONE failure: surrender torque mid-turn -> SAT flings the
  freed wheel ~20 deg out of the curve -> slow step-capped rebuild. SAT scales
  with v^2, so that hazard fades at low speed -- and on this rack stiction
  HOLDS the angle at zero torque (the stall/breakaway story), the opposite of
  being flung. Below 30 km/h the dwell insures against a hazard that is not
  present, and route 417 shows it actively fighting the driver instead.

  Scope is SPEED ONLY (user ruling 2026-08-21: must not affect normal HOLD).
  The 'intersection' half is already inherent -- deep_relax needs
  |kappa_meas| > RELAX_DWELL_KAPPA -- so this cannot leak into straight-line
  driving, and gating on curvature as well would repeat the gain-floor
  mistake of putting a cliff on the kappa axis. Below 30 km/h is LKA by
  construction: DCC cannot engage there.
  """

  def _arm_attempt(self, monkeypatch, v):
    """One tick that WOULD arm the dwell; returns the relax_ticks increment."""
    lac, sm, mod, state = _make_controller(monkeypatch)
    _set_measured(sm, v, 0.012)          # deep curve, |κ_meas| > RELAX_DWELL_KAPPA
    state['torque'] = 0.30
    state['action'] = 'idle'
    state['tick_count'] = 0
    before = state['relax_ticks']
    # overshoot-side error: desired same side but smaller than measured
    _call_update(lac, 0.008, steering_angle_deg=0.0, v_ego=v)
    return state['relax_ticks'] - before

  def test_dwell_disarmed_below_30kph(self, monkeypatch):
    """The route 417 regime: 18 km/h intersection exit."""
    assert self._arm_attempt(monkeypatch, 5.0) == 0

  def test_dwell_disarmed_at_walking_pace(self, monkeypatch):
    assert self._arm_attempt(monkeypatch, 2.0) == 0

  def test_dwell_still_arms_at_30kph(self, monkeypatch):
    """Boundary: the dwell's own hairpin validation was at ~9 m/s."""
    assert self._arm_attempt(monkeypatch, 8.5) == 1

  def test_dwell_still_arms_at_speed(self, monkeypatch):
    """Normal HOLD behaviour is untouched — the whole point of the ruling."""
    for v in (12.0, 20.0, 30.0):
      assert self._arm_attempt(monkeypatch, v) == 1, v


class TestUnwindStepBoost:
  """Route 417 (2026-08-21), the second half of the intersection-exit fight.

  ~0.8 s of resistance remains at an intersection exit: STEP_MAX bounds the decay at ~0.33 frac/s (~4 Nm/s), so shedding
  the 3.9 Nm held through an intersection takes about a second and the wheel's
  return is still fought.

  STEP_MAX exists to stop abrupt APPLICATION — route 39b/seg 27, where single
  decisions swinging 0.69 frac drove 150 deg/s wheel bursts. Releasing torque
  carries no such hazard in the LKA regime: below 30 km/h SAT cannot fling the
  freed wheel and this rack's stiction holds the angle at zero torque. The
  controller's own comment puts it plainly — "unwinding is instant and free,
  rebuilding is slow and fought".

  So below LKA_MAX_V, and ONLY while the step moves torque toward zero, the
  cap is multiplied by UNWIND_STEP_GAIN. Building torque is untouched at every
  speed, and so is everything at or above 30 km/h.

  Deliberately NOT gated on curvature: kappa_meas collapses through
  RELAX_DWELL_KAPPA partway through the exit (route 417 seg 2: 0.037 -> 0.003
  during the return), so a curvature conjunct would switch the fast decay off
  at exactly the moment it is needed.
  """

  DESIRED = -0.02      # deep target: P saturates, so the step cap is what binds
  TORQUE = 0.60        # standing push, opposite sign to the target

  def _target(self, monkeypatch, v, torque=TORQUE):
    lac, sm, mod, state = _make_controller(monkeypatch)
    _decide_from_torque(lac, sm, state, v=v, desired=self.DESIRED, torque=torque)
    return state['target_frac'], mod

  def test_unwind_boosted_below_30kph(self, monkeypatch):
    """18 km/h intersection exit: the step toward zero gets the boost."""
    got, mod = self._target(monkeypatch, 5.0)
    expected = self.TORQUE - 0.10 * mod.UNWIND_STEP_GAIN
    assert got == pytest.approx(expected, abs=1e-6), got

  def test_unwind_fully_boosted_at_30kph(self, monkeypatch):
    """Band floor. This assertion FLIPPED on 2026-08-24: it used to pin the
    cliff (no boost at exactly LKA_MAX_V). LKA_MAX_V is now the lower edge of
    a blend band, so full boost holds AT 30 km/h and fades to 1.0 by
    LKA_BLEND_V — see TestLkaSpeedBlend. Everything strictly below 30 and
    everything at/above 43 km/h is unchanged."""
    got, mod = self._target(monkeypatch, 8.5)
    expected = self.TORQUE - 0.10 * mod.UNWIND_STEP_GAIN
    assert got == pytest.approx(expected, abs=1e-6), got

  def test_unwind_not_boosted_at_speed(self, monkeypatch):
    got, mod = self._target(monkeypatch, 20.0)
    step = 0.10 + (20.0 - 15.0) / (28.0 - 15.0) * (0.05 - 0.10)
    assert got == pytest.approx(self.TORQUE - step, abs=1e-6), got

  def test_building_torque_is_not_boosted_below_30kph(self, monkeypatch):
    """From rest the step BUILDS torque — normal cap, no boost. This is the
    conjunct that keeps the seg-27 abrupt-application hazard covered."""
    got, mod = self._target(monkeypatch, 5.0, torque=0.0)
    assert got == pytest.approx(-0.10, abs=1e-6), got

  def test_boost_stops_once_torque_reverses(self, monkeypatch):
    """Past zero the step is building in the NEW direction — normal cap."""
    got, mod = self._target(monkeypatch, 5.0, torque=-0.05)
    assert got == pytest.approx(-0.05 - 0.10, abs=1e-6), got


def _dwell_tick(monkeypatch, *, v, relax_ticks):
  """One decision that WOULD dwell: deep measured curve, standing torque, and
  an overshoot-side kappa_des dip on the same side. relax_ticks is the value
  BEFORE the tick — deep_relax increments it before `dwelling` is evaluated,
  so the dwell is tested against relax_ticks + 1."""
  lac, sm, mod, state = _make_controller(monkeypatch)
  _set_measured(sm, v, 0.012)          # |κ_meas| > RELAX_DWELL_KAPPA
  state['torque'] = -0.3
  state['target_frac'] = -0.3
  state['ramp_frames'] = 0
  state['action'] = 'idle'
  state['relax_ticks'] = relax_ticks
  state['tick_count'] = 999            # force the cadence decision
  _call_update(lac, 0.001, v_ego=v)
  return state['action'], mod


class TestLkaSpeedBlend:
  """Route 41b (2026-08-24): LKA_MAX_V was a cliff on the v axis. Three
  behaviours flipped together at 30 km/h and all in the same direction, so a
  turn one side of the line got a wholly different controller from a turn on
  the other. Seg 33 12:01:37 is the reference case — a driver turn entered at
  31.8 km/h, 1.8 km/h over the line, got no unwind boost, then decelerated THROUGH 30
  mid-turn and switched regime halfway. 13-19% of that drive sat within
  +-1.8 km/h of the line.

  Crossings were rare (3-4 per segment), so this is not chatter and hysteresis
  would buy nothing. The mismatch is that LKA_MAX_V is a good MODE detector
  (DCC cannot engage below 30, so it identifies the LKA regime for free) being
  used as a PLANT threshold — and the plant does not step at 30 km/h. SAT at
  29.9 and 30.1 km/h is the same number.

  Fix: keep v as the axis, replace the boolean with a ramp over
  [LKA_MAX_V, LKA_BLEND_V]. Both endpoints reproduce the previously verified
  behaviour exactly (np.interp clamps outside the range); only the band
  between them changes. The one exception is v == LKA_MAX_V itself, where the
  old strict `<` gave no boost — an arbitrary tie-break with no physical
  meaning, now folded into the full-boost edge.

  Deliberately still SPEED ONLY: no curvature conjunct (that would repeat the
  gain-floor cliff on the kappa axis).
  """
  DESIRED = -0.02
  TORQUE = 0.60

  def _target(self, monkeypatch, v):
    lac, sm, mod, state = _make_controller(monkeypatch)
    _decide_from_torque(lac, sm, state, v=v, desired=self.DESIRED,
                        torque=self.TORQUE)
    return state['target_frac'], mod

  def _gain(self, monkeypatch, v):
    """Recover the applied unwind gain from the step actually taken. Below
    15 m/s the base STEP_MAX schedule clamps to 0.10, so the step is
    0.10 * gain for every speed this class exercises."""
    got, mod = self._target(monkeypatch, v)
    return (self.TORQUE - got) / 0.10, mod

  # --- band definition ---
  def test_band_ceiling_is_12_ms(self, monkeypatch):
    import bmw.latcontroller as mod
    assert mod.LKA_BLEND_V == 12.0
    assert mod.LKA_BLEND_V > mod.LKA_MAX_V

  # --- unwind step gain across the band ---
  def test_gain_is_full_below_the_band(self, monkeypatch):
    g, mod = self._gain(monkeypatch, 5.0)
    assert g == pytest.approx(mod.UNWIND_STEP_GAIN, abs=1e-6), g

  def test_gain_is_full_at_the_band_floor(self, monkeypatch):
    g, mod = self._gain(monkeypatch, 8.5)
    assert g == pytest.approx(mod.UNWIND_STEP_GAIN, abs=1e-6), g

  def test_gain_is_half_way_down_at_the_band_midpoint(self, monkeypatch):
    g, mod = self._gain(monkeypatch, 10.25)
    assert g == pytest.approx(1.0 + (mod.UNWIND_STEP_GAIN - 1.0) / 2.0,
                              abs=1e-6), g

  def test_gain_is_gone_at_the_band_ceiling(self, monkeypatch):
    g, mod = self._gain(monkeypatch, 12.0)
    assert g == pytest.approx(1.0, abs=1e-6), g

  def test_gain_is_monotonic_through_the_band(self, monkeypatch):
    gains = [self._gain(monkeypatch, v)[0]
             for v in (8.5, 9.4, 10.25, 11.1, 12.0)]
    assert all(a > b for a, b in zip(gains, gains[1:])), gains

  def test_gain_stays_gone_above_the_band(self, monkeypatch):
    """Nothing at highway speed may inherit the boost — STEP_MAX exists to
    move LESS per decision as v rises."""
    got, mod = self._target(monkeypatch, 20.0)
    step = 0.10 + (20.0 - 15.0) / (28.0 - 15.0) * (0.05 - 0.10)
    assert got == pytest.approx(self.TORQUE - step, abs=1e-6), got


def _kappa_for_delta_err(kappa_des, delta_err, L=2.66):
  """κ_meas that makes δ_err come out exactly `delta_err`.

  δ_err = atan(κ_des·L) − atan(κ_meas·L), so invert the second atan. Solving
  it rather than approximating κ_des − δ_err/L matters here: the band is
  0.0012 rad wide and at the intersection operating point
  (κ ≈ 0.1) the atan is well off its linear region.
  """
  return math.tan(math.atan(kappa_des * L) - delta_err) / L


  # --- relax dwell length across the band ---
  def _max_dwell(self, monkeypatch, v, cap=40):
    """Longest dwell the bridge will sustain at speed v, in ticks (0 = it can
    never form). Probed rather than read off RELAX_DWELL_TICKS, which is a
    local inside on_lat_controller_init and not reachable from the module."""
    n = 0
    for t in range(cap):
      action, _ = _dwell_tick(monkeypatch, v=v, relax_ticks=t)
      if action != 'relax_dwell':
        break
      n = t + 1
    return n

  def test_dwell_never_engages_at_the_band_floor(self, monkeypatch):
    """30 km/h: dwell length interpolates to 0, so the bridge cannot form —
    the previously verified e748434 behaviour. deep_relax still counts its
    ticks, so the rlx telemetry column keeps its meaning."""
    assert self._max_dwell(monkeypatch, 8.5) == 0

  def test_dwell_never_engages_below_the_band(self, monkeypatch):
    assert self._max_dwell(monkeypatch, 5.0) == 0

  def test_dwell_is_full_length_at_the_band_ceiling(self, monkeypatch):
    """43 km/h and above is untouched — normal HOLD is the thing this must
    not disturb."""
    full = self._max_dwell(monkeypatch, 12.0)
    assert full > 0
    assert self._max_dwell(monkeypatch, 20.0) == full
    assert self._max_dwell(monkeypatch, 30.0) == full

  def test_dwell_is_half_length_at_the_band_midpoint(self, monkeypatch):
    full = self._max_dwell(monkeypatch, 12.0)
    assert self._max_dwell(monkeypatch, 10.25) == full // 2

  def test_dwell_grows_monotonically_through_the_band(self, monkeypatch):
    lens = [self._max_dwell(monkeypatch, v)
            for v in (8.5, 9.4, 10.25, 11.1, 12.0)]
    assert all(a <= b for a, b in zip(lens, lens[1:])), lens
    assert lens[0] == 0 and lens[-1] > 0, lens


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
  spread_frames = action_cadence_ticks * 5, from the fixed CONTROL_HORIZON_S
  = 0.65 s (independent of lat_delay since 2026-10-12) -> half = 0.325 s ->
  action_cadence_ticks = round(6.5) = 6 (banker's rounding) -> 30 frames.
  So state['torque'] lands on -step_max / 30.
  """

  DESIRED = -0.02      # deep target: P saturates, so the step cap is what binds

  def test_full_step_at_and_below_the_first_breakpoint(self, monkeypatch):
    lac, sm, mod, state = _make_controller(monkeypatch)
    _decide_from_zero(lac, sm, state, v=15.0, desired=self.DESIRED)
    assert state['action'] == 'ramp'
    assert state['target_frac'] == pytest.approx(-0.10, abs=1e-9)
    assert state['torque'] == pytest.approx(-0.10 / 30, abs=1e-9)

  def test_half_step_at_and_above_the_second_breakpoint(self, monkeypatch):
    lac, sm, mod, state = _make_controller(monkeypatch)
    _decide_from_zero(lac, sm, state, v=28.0, desired=self.DESIRED)
    assert state['action'] == 'ramp'
    assert state['target_frac'] == pytest.approx(-0.05, abs=1e-9)
    assert state['torque'] == pytest.approx(-0.05 / 30, abs=1e-9)

  def test_interpolated_step_between_the_breakpoints(self, monkeypatch):
    """20 m/s sits strictly inside the ramp: 0.10 - 5/13*0.05 = 0.0807692.
    This is the case a two-level step function would get wrong."""
    lac, sm, mod, state = _make_controller(monkeypatch)
    _decide_from_zero(lac, sm, state, v=20.0, desired=self.DESIRED)
    assert state['action'] == 'ramp'
    assert state['target_frac'] == pytest.approx(-0.0807692307, abs=1e-9)
    assert state['torque'] == pytest.approx(-0.0807692307 / 30, abs=1e-9)

  def test_step_cap_shrinks_monotonically_with_speed(self, monkeypatch):
    """The safety property in one assertion, independent of exact values: a
    faster decision may never take a LARGER torque step than a slower one."""
    steps = []
    for v in (15.0, 20.0, 28.0):
      lac, sm, mod, state = _make_controller(monkeypatch)
      _decide_from_zero(lac, sm, state, v=v, desired=self.DESIRED)
      steps.append(abs(state['target_frac']))
    assert steps[0] > steps[1] > steps[2]


class TestControllerHorizonDecoupled:
  """2026-10-12: CP.steerActuatorDelay 0.4 -> 0.7 so modeld plans for the
  measured straight-line loop delay (target -> yaw 0.90-1.0 s on routes
  4d4/4d5 vs the 0.65 s action point). The controller's own timing must NOT
  follow: its cadence and ramp spread stay on the field-tuned 0.65 s horizon.
  A 0.95 s horizon would stretch the cadence to ~500 ms (more lag - the 4d8
  lesson)."""

  def _decision(self, monkeypatch, lat_delay):
    lac, sm, mod, state = _make_controller(monkeypatch)
    state['torque'] = 0.0
    state['target_frac'] = 0.0
    state['ramp_frames'] = 0
    state['action'] = 'idle'
    state['tick_count'] = 999
    _set_measured(sm, 20.0, 0.0)
    _call_update(lac, -0.004, v_ego=20.0, lat_delay=lat_delay)
    return state, mod

  def test_ramp_spread_ignores_lat_delay(self, monkeypatch):
    a, _ = self._decision(monkeypatch, 0.6)
    b, _ = self._decision(monkeypatch, 0.9)
    assert a['action'] == b['action'] == 'ramp'
    assert a['ramp_frames'] == b['ramp_frames']
    assert a['torque'] == pytest.approx(b['torque'], abs=1e-12)

  def test_spread_is_the_field_tuned_300ms(self, monkeypatch):
    """0.65 s horizon -> half 0.325 s -> 6 livePose ticks -> 30 CAN frames;
    one frame is applied inside the decision tick."""
    a, _ = self._decision(monkeypatch, 0.9)
    assert a['ramp_frames'] == 30 - 1

  def test_horizon_constant_is_the_old_production_value(self, monkeypatch):
    """0.4 + 0.2 (lagd initial_lag) + DT_MDL: the horizon every controller
    constant was tuned at."""
    _, mod = self._decision(monkeypatch, 0.6)
    assert mod.CONTROL_HORIZON_S == pytest.approx(0.65)


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
# Retired mechanisms. The driver-override observers and stall/breakaway v2
# (StallBreakaway, removed 2026-10-23: its shadow trips on 4ee-4f7 were
# ordinary turn-ins) must not leave state, telemetry or constants behind.
# ============================================================

class TestRetiredMechanisms:
  def test_no_ov_keys_in_state_or_payload(self, monkeypatch):
    """The driver-override observers are gone: nothing in the state or the
    telemetry starts with ov_."""
    lac, sm, mod, state = _make_controller(monkeypatch)
    pay = []
    state['lat_pub'] = SimpleNamespace(send=pay.append)
    _call_update(lac, 0.001, v_ego=20.0)
    assert not [k for k in state if k.startswith('ov_')]
    assert not [k for k in pay[-1] if k.startswith('ov_')]
    assert not hasattr(mod, 'OV_EOD_RATE_DPS')

  def test_no_sb_keys_in_state_or_payload(self, monkeypatch):
    """Stall/breakaway v2 is gone: no sb_ state or telemetry, no SB_ constants."""
    lac, sm, mod, state = _make_controller(monkeypatch)
    pay = []
    state['lat_pub'] = SimpleNamespace(send=pay.append)
    _call_update(lac, 0.001, v_ego=20.0)
    assert not [k for k in state if k.startswith('sb_')]
    assert not [k for k in pay[-1] if k.startswith('sb_')]
    assert not [n for n in dir(mod) if n.startswith('SB_')]


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

  def test_torque_gains_track_true_speed_above_the_floor(self, monkeypatch):
    """SUPERSEDES test_torque_gains_keep_30kph_floor (route 413 seg 2): the
    gains referenced max(v, 8.5), so 5 m/s commanded exactly what 8.5 m/s did.
    They now reference max(v, 2.778), so below 30 km/h torque follows actual
    speed. Same near-straight kappa the old test used — the point is that this
    case is no longer exempt."""
    torques = []
    for v in (5.0, 8.5):
      lac, fake_sm, mod, state = _make_controller(monkeypatch)
      _set_measured(fake_sm, v, 0.005)
      # kappa_des small enough that the P target stays below the per-decision
      # step cap — otherwise both runs saturate at step_max and can't differ
      _call_update(lac, 0.006, v_ego=v)
      torques.append(abs(state['torque']))
    assert torques[0] < torques[1] * 0.6, torques


class TestLowSpeedGainFloor:
  """Route 411 seg 2 -> 413 seg 2. The torque law referenced max(v_true, 8.5):
  below 30 km/h the gain stopped extrapolating v^2 toward zero. That floor is a
  PROXY for constant rack friction, and a fake v^2 is the wrong shape for it —
  it over-delivers by 8.5^2/v_true^2 (2.9x at 18 km/h, 37x at 5 km/h) and
  saturates the P law, so commanded torque carries no information about how
  wrong the wheel is.

  Floor moved to 2.778 m/s (10 km/h). A curvature-banded version of this
  shipped first (033df35) and was WRONG: route 413 seg 2 fought at 5 km/h with
  |kappa_des| BELOW the band, where the old 8.5 floor still applied and the
  target was ~10 Nm; the band edge also swung the gain 7x as kappa_des drifted
  across it. Speed is the axis that matters — and since DCC cannot engage below
  30 km/h, a sub-30 change is LKA-scoped by construction."""

  def test_floor_is_flat_10kph(self, monkeypatch):
    """No curvature dependence — that was the 033df35 mistake."""
    _, _, mod, _ = _make_controller(monkeypatch)
    for k in (0.0, 0.0003, 0.005, 0.03, 0.11):
      assert mod.gain_reference_speed(1.0) == pytest.approx(2.778)

  def test_true_speed_governs_above_the_floor(self, monkeypatch):
    _, _, mod, _ = _make_controller(monkeypatch)
    assert mod.gain_reference_speed(5.0) == pytest.approx(5.0)
    assert mod.gain_reference_speed(3.0) == pytest.approx(3.0)

  def test_floor_binds_at_walking_pace(self, monkeypatch):
    """5 km/h: a pure v^2 law commands 0.6-1.6 Nm, all below the ~2.75 Nm
    breakaway knee — i.e. no steering at all. The floor is the friction
    headroom v^2 cannot supply, so it is lowered, never removed."""
    _, _, mod, _ = _make_controller(monkeypatch)
    assert mod.gain_reference_speed(1.389) == pytest.approx(2.778)

  def test_unchanged_at_and_above_30kph(self, monkeypatch):
    """The property that leaves the at-speed field tuning alone: at or above
    the OLD floor, old and new laws are the identity."""
    _, _, mod, _ = _make_controller(monkeypatch)
    for v in (8.5, 10.0, 25.0):
      assert mod.gain_reference_speed(v) == pytest.approx(max(v, 8.5))

  def test_near_straight_low_speed_is_no_longer_exempt(self, monkeypatch):
    """The 413 seg 2 case: 5 km/h with kappa_des ~0 used to target ~10 Nm."""
    torques = []
    for v in (1.4, 8.5):
      lac, fake_sm, mod, state = _make_controller(monkeypatch)
      # near-straight, but a real delta_err: kappa_des ~0 with kappa_meas ~0
      # commands ~nothing on BOTH sides, which would pass the assertion
      # vacuously (0 vs 0 tests nothing)
      _set_measured(fake_sm, v, 0.004)
      _call_update(lac, 0.005, v_ego=v)
      torques.append(abs(state['torque']))
    assert torques[0] > 0.0, "fixture commands no torque - test proves nothing"
    assert torques[0] < torques[1] * 0.2, torques


# ============================================================
# Direct Tracking decision path (spec §2/§4): the only law. No rest state; P
# sign reversals pass the noise guard; the hold-floor in curves is kept.
# ============================================================



# ============================================================
# Curve feed-forward: ff_nm = clip(CF_GAIN * sign(a_ref) * max(|a_ref| - CF_AY0, 0), +/-CF_MAX_NM)
# with a_ref = v_true^2 * kappa_ref (docs/superpowers/specs/2026-10-20-bmw-curve-
# feedforward-and-audit-design.md, Part A).
# ============================================================

def test_curve_ff_zero_inside_deadband():
  from bmw.latcontroller import curve_ff_nm
  assert curve_ff_nm(0.29) == 0.0 == curve_ff_nm(-0.29) == curve_ff_nm(0.0)


def test_curve_ff_sign_and_gain():
  from bmw.latcontroller import curve_ff_nm
  assert curve_ff_nm(1.3) == pytest.approx(1.0)
  assert curve_ff_nm(-1.3) == pytest.approx(-1.0)


def test_curve_ff_cap():
  from bmw.latcontroller import curve_ff_nm
  assert curve_ff_nm(5.0) == 2.0 and curve_ff_nm(-5.0) == -2.0


class TestDirectTracking:

  def _engaged(self, monkeypatch, v=25.0):
    lac, fake_sm, mod, state = _make_controller(monkeypatch)
    _set_measured(fake_sm, v, 0.0)
    _call_update(lac, 0.0, v_ego=v, active=False)
    _call_update(lac, 0.0, v_ego=v, active=True)
    return lac, fake_sm, mod, state

  def test_no_lane_keeping_means_stock_controller(self, monkeypatch):
    """lane_keeping not loaded (disabled or failed): the hook does not patch
    lac.update, so the stock lateral controller runs."""
    monkeypatch.delitem(sys.modules, _LK_REGISTER)
    import bmw.latcontroller as mod
    sentinel = object()
    lac = SimpleNamespace(update=sentinel); result = SimpleNamespace()
    out = mod.on_lat_controller_init(result, lac, SimpleNamespace(wheelbase=2.66, steerActuatorDelay=0.4))
    assert lac.update is sentinel and out is result

  def test_half_loaded_lane_keeping_means_stock_controller(self, monkeypatch):
    """A module the registry left half-executed has no HOOK_READY: same."""
    monkeypatch.setitem(sys.modules, _LK_REGISTER, SimpleNamespace())
    import bmw.latcontroller as mod
    sentinel = object()
    lac = SimpleNamespace(update=sentinel); result = SimpleNamespace()
    out = mod.on_lat_controller_init(result, lac, SimpleNamespace(wheelbase=2.66, steerActuatorDelay=0.4))
    assert lac.update is sentinel and out is result

  def test_slope_latches_at_engagement(self, monkeypatch):
    import config
    params = {'SlopeCompensation': '1'}
    monkeypatch.setattr(config, 'read_plugin_param',
                        lambda plugin_id, key, default='': params.get(key, default))
    lac, fake_sm, mod, state = self._engaged(monkeypatch)
    assert state['slope']
    _call_update(lac, 0.0, v_ego=25.0, active=False)
    assert not state['slope']
    params['SlopeCompensation'] = '0'
    _call_update(lac, 0.0, v_ego=25.0, active=True)
    assert not state['slope']

  def test_disengage_zeroes_the_ramp_state(self, monkeypatch):
    """While lateral is inactive the law does not run on the state: disengaging
    zeroes torque/target/ramp, and the next engagement ramps from 0 on its
    first livePose tick instead of jumping to a torque built while inactive."""
    lac, fake_sm, mod, state = self._engaged(monkeypatch)
    state.update(torque=0.3, target_frac=0.4, ramp_step=0.002, ramp_frames=10, action='ramp')
    _call_update(lac, 0.004, v_ego=25.0, active=False)
    assert state['torque'] == 0.0 and state['target_frac'] == 0.0 and state['ramp_frames'] == 0
    for _ in range(20):
      _call_update(lac, 0.004, v_ego=25.0, active=False)
    assert state['torque'] == 0.0 and state['target_frac'] == 0.0   # inactive: no decision runs
    out = _call_update(lac, 0.004, v_ego=25.0, active=True)        # first engaged tick decides, from 0
    assert state['action'] == 'ramp' and state['target_frac'] > 0.0
    assert abs(out[0]) < 0.004                                      # one ramp frame of one STEP_MAX

  def test_straight_gain_is_1_25(self, monkeypatch):
    """Near straight (|kappa_ref| < 1e-3) the P gain is T_CAP_SLOPE_BASE * 1.25 (route 4ed:
    the 0.45 Nm near-centre stiction band let gentle-bend errors sit at ~4e-4 before the rack
    moved; the error needed to break free scales with 1/gain)."""
    lac, fake_sm, mod, state = self._engaged(monkeypatch)
    _set_measured(fake_sm, 25.0, 0.0)
    state.update(torque=0.0, target_frac=0.0, ramp_frames=0, action='idle', tick_count=999)
    _call_update(lac, 0.0001, v_ego=25.0)             # delta_err = atan(0.0001 * 2.66), straight
    p_nm = 1.25 * 625.0 * math.atan(0.0001 * 2.66)    # 0.208 Nm, inside STEP_MAX and t_cap
    assert state['target_frac'] * 12.0 == pytest.approx(p_nm, rel=1e-6)

  def test_sub_band_error_is_tracked(self, monkeypatch):   # the 4e5 arc-only case
    lac, fake_sm, mod, state = self._engaged(monkeypatch)
    state['tick_count'] = 999
    _call_update(lac, 0.0003, v_ego=25.0)       # δ_err ≈ 0.0008 rad: no band, tracked
    assert state['action'] == 'ramp' and state['target_frac'] > 0.0


  def test_short_reversal_sheds_to_zero_not_across(self, monkeypatch):  # Review Focus 4
    lac, fake_sm, mod, state = self._engaged(monkeypatch)
    state['tick_count'] = 999; _call_update(lac, +0.002, v_ego=25.0)   # commit right
    assert state['target_frac'] > 0.0 and state['guard'].side == +1
    state['tick_count'] = 999; _call_update(lac, -0.002, v_ego=25.0)   # one tick opposite
    # Exactly 0, not merely non-negative: only one CAN frame of the right ramp
    # has been applied, so shedding it is far inside step_max (torque + (-torque)
    # is exact in floats).
    assert state['target_frac'] == pytest.approx(0.0)
    assert state['guard'].suppressed == 1

  def test_persisted_reversal_crosses(self, monkeypatch):
    """Once the opposite error sign has persisted the guard window (3 livePose
    ticks), the reversal passes — the guard delays, it never blocks."""
    lac, fake_sm, mod, state = self._engaged(monkeypatch)
    state['tick_count'] = 999; _call_update(lac, +0.002, v_ego=25.0)
    state['torque'] = 0.0; state['ramp_frames'] = 0       # old side already shed
    for _ in range(3):
      _call_update(lac, -0.002, v_ego=25.0)             # persist (no decision)
    state['tick_count'] = 999; _call_update(lac, -0.002, v_ego=25.0)
    assert state['target_frac'] < 0.0 and state['guard'].side == -1

  def test_direct_applies_in_lka_regime(self, monkeypatch):        # user ruling: all speeds
    lac, fake_sm, mod, state = self._engaged(monkeypatch, v=5.0)
    state['tick_count'] = 999
    _call_update(lac, 0.0003, v_ego=5.0)        # sub-band δ_err at 18 km/h
    assert state['action'] == 'ramp' and state['target_frac'] > 0.0

  def test_no_hold_floor_in_curves(self, monkeypatch):
    """v=20, kappa_des=0.006 (a_ref 2.4 -> ff 2.0 Nm), standing torque 0.15,
    kappa_meas lagging by delta_err 0.002: P = 1.833*400*0.002 = 1.47 Nm. The
    decision targets ff + P = 3.47 Nm = 0.289 frac, step-limited from 0.15 by
    STEP_MAX(20) = 0.0808 — the standing torque is neither held nor floored."""
    v, kappa = 20.0, 0.006
    lac, fake_sm, mod, state = _make_controller(monkeypatch)
    kappa_meas = math.tan(math.atan(kappa * 2.66) - 0.002) / 2.66
    _set_measured(fake_sm, v, kappa_meas)
    _call_update(lac, kappa, v_ego=v, active=False)
    _call_update(lac, kappa, v_ego=v, active=True)
    state.update(torque=0.15, target_frac=0.15, ramp_frames=0, action='idle',
                 tick_count=999)
    _call_update(lac, kappa, v_ego=v)
    assert state['delta_err'] == pytest.approx(0.002)
    assert state['action'] == 'ramp'
    assert state['ff_nm'] == pytest.approx(2.0)
    assert state['target_frac'] == pytest.approx(0.15 + 0.0807692307, abs=1e-6)
    assert not hasattr(mod, 'hold_factor')
    assert 'hold_f' not in state and 'hold_cap' not in state

  def test_hairpin_reference_dip_is_bridged(self, monkeypatch):
    """Route 3a0 / 4e9 seg 9: in a deep measured curve (kappa_meas 0.012 at
    12 m/s) the reference dips 40 % for under a second while the wheel holds.
    The FF is at its cap there and cannot bridge the dip, so relax-dwell
    keeps the standing torque for the dip: the decision targets the current
    torque, not the dropped demand."""
    v = 12.0
    lac, fake_sm, mod, state = _make_controller(monkeypatch)
    _set_measured(fake_sm, v, 0.012)
    _call_update(lac, 0.012, v_ego=v, active=False)
    _call_update(lac, 0.012, v_ego=v, active=True)
    state.update(torque=0.3, target_frac=0.3, ramp_frames=0, action='idle', tick_count=999)
    _call_update(lac, 0.0072, v_ego=v)                     # the dip: 40 % below the measured curve
    assert state['action'] == 'relax_dwell'
    assert state['target_frac'] == pytest.approx(0.3)
    assert state['relax_ticks'] == 1

  def _curve_decision(self, monkeypatch, *, v, kappa, delta_err, torque):
    """Engage in a curve with delta_err of lag, standing torque `torque`, the
    guard committed to that push's side; run one decision."""
    lac, fake_sm, mod, state = _make_controller(monkeypatch)
    self._last_lac = lac
    _set_measured(fake_sm, v, math.tan(math.atan(kappa * 2.66) - delta_err) / 2.66)
    _call_update(lac, kappa, v_ego=v, active=False)
    _call_update(lac, kappa, v_ego=v, active=True)
    state['guard'].reset(); state['guard'].side = +1     # pushing right
    state.update(torque=torque, target_frac=torque, ramp_frames=0, action='idle',
                 tick_count=999)
    _call_update(lac, kappa, v_ego=v)
    assert state['delta_err'] == pytest.approx(delta_err)
    return mod, state

  # v=25, kappa_ref 0.002: kappa_scale 1.3889, gain G = 868.06 Nm/rad, a_ref 1.25 -> ff 0.95 Nm,
  # step_max 0.0615 frac, t_cap ~0.49 frac.
  _G = 1.0 * (1.25 + (0.001 / 0.009) * 1.25) * 625.0

  def test_decision_is_ff_plus_p(self, monkeypatch):
    mod, state = self._curve_decision(monkeypatch, v=25.0, kappa=0.002,
                                      delta_err=0.60 / self._G, torque=0.125)
    assert state['ff_nm'] == pytest.approx(0.95)
    assert state['target_frac'] * 12.0 == pytest.approx(1.55, abs=0.05)

  def test_overshoot_trims_the_sum(self, monkeypatch):
    mod, state = self._curve_decision(monkeypatch, v=25.0, kappa=0.002,
                                      delta_err=-0.40 / self._G, torque=0.05)
    assert state['target_frac'] * 12.0 == pytest.approx(0.55, abs=0.05)
    assert state['target_frac'] > 0.0 and state['action'] == 'ramp'

  def test_large_overshoot_crosses_zero_under_guard(self, monkeypatch):
    """ff +0.95 with P -1.50: the demand is -0.55 Nm. The guard (committed
    right) holds the first decision at 0; once the demand's sign has persisted
    GUARD_PERSIST_S the next decision goes negative."""
    mod, state = self._curve_decision(monkeypatch, v=25.0, kappa=0.002,
                                      delta_err=-1.50 / self._G, torque=0.0)
    assert state['target_frac'] == 0.0 and state['guard'].suppressed == 1
    lac = self._last_lac
    for _ in range(3):                                     # the demand's sign persists
      _call_update(lac, 0.002, v_ego=25.0)
    state['tick_count'] = 999
    _call_update(lac, 0.002, v_ego=25.0)
    assert state['ff_nm'] == pytest.approx(0.95)
    assert state['target_frac'] * 12.0 == pytest.approx(-0.55, abs=0.05)   # the sum, not P alone

  def test_ff_uses_true_speed_not_gain_floor(self, monkeypatch):
    """At 1 m/s (gain reference floored to 2.78) a 0.05 1/m turn is a_ref 0.05,
    not 0.39: the feed-forward reads the true speed and stays 0."""
    lac, fake_sm, mod, state = _make_controller(monkeypatch)
    _set_measured(fake_sm, 1.0, 0.05)
    _call_update(lac, 0.05, v_ego=1.0, active=False)
    _call_update(lac, 0.05, v_ego=1.0, active=True)
    assert state['a_ref'] == pytest.approx(0.05)
    assert state['ff_nm'] == 0.0

  def test_ff_low_speed_turn_bounded(self, monkeypatch):
    lac, fake_sm, mod, state = _make_controller(monkeypatch)
    _set_measured(fake_sm, 5.0, 0.05)
    _call_update(lac, 0.05, v_ego=5.0, active=False)
    _call_update(lac, 0.05, v_ego=5.0, active=True)
    assert state['a_ref'] == pytest.approx(1.25)
    assert state['ff_nm'] == pytest.approx(0.95)

  def test_ff_capped_and_step_limited_on_kappa_spike(self, monkeypatch):
    """A raw-model spike (lane change) of 0.003 at 30 m/s is a_ref 2.7: the FF
    saturates at CF_MAX_NM and the decision still moves at most STEP_MAX."""
    lac, fake_sm, mod, state = _make_controller(monkeypatch)
    _set_measured(fake_sm, 30.0, 0.003)
    _call_update(lac, 0.003, v_ego=30.0, active=False)
    _call_update(lac, 0.003, v_ego=30.0, active=True)
    state.update(torque=0.0, target_frac=0.0, ramp_frames=0, action='idle', tick_count=999)
    _call_update(lac, 0.003, v_ego=30.0)
    assert state['ff_nm'] == 2.0
    assert abs(state['target_frac']) <= 0.05 + 1e-9        # STEP_MAX at/above 28 m/s

  def test_s_curve_ff_reversal_passes_guard_then_takes_up(self, monkeypatch):
    """Steady right curve carried by the FF, then the mirror-image left curve:
    the demand flips sign, the guard holds 0 until the new sign has persisted,
    the ramp crosses zero and the take-up fires once."""
    lac, fake_sm, mod, state = self._engaged(monkeypatch)
    _seed_learner(state, 77.6, 65.0)
    _set_measured(fake_sm, 25.0, 0.002)
    for _ in range(124):          # settle on the right curve; the flip lands 2 livePose ticks before a decision
      _call_update(lac, 0.002, v_ego=25.0, gap_deg=77.6)
    assert state['torque'] > 0.05 and state['takeup'].fires == 0
    _set_measured(fake_sm, 25.0, -0.002)
    gap = 77.6
    for _ in range(300):
      _call_update(lac, -0.002, v_ego=25.0, gap_deg=gap)
      if state['takeup'].state == 'taking_up':
        gap = 65.0                                         # the servo reaches the left contact
    assert state['guard'].suppressed >= 1
    assert state['takeup'].fires == 1 and state['takeup'].timeouts == 0
    assert state['torque'] < -0.05

  def test_guard_judges_the_full_demand_in_a_curve(self, monkeypatch):
    """ff +0.95 with P -0.40 is a positive demand: the guard stays committed
    right although the error alone is negative."""
    mod, state = self._curve_decision(monkeypatch, v=25.0, kappa=0.002,
                                      delta_err=-0.40 / self._G, torque=0.05)
    assert state['guard'].side == +1 and state['guard'].suppressed == 0


  def test_relax_dwell_exempt_from_guard(self, monkeypatch):
    """Ruling R3: relax-dwell never gives up mid-turn. With the guard already
    committed to the NEW (overshoot-correction) side, the dwell's old-side
    hold of the current torque must pass untouched — it holds the current
    torque's sign, so it cannot cross the gap. v=15, κ_des 0.012 < κ_meas
    0.015 (deep, same side, overshoot) -> dwelling; t_cap > 0.15."""
    v = 15.0
    lac, fake_sm, mod, state = self._engaged(monkeypatch, v=v)
    _set_measured(fake_sm, v, 0.015)
    state['guard'].side = -1          # reversal already confirmed (left)
    state.update(torque=0.15, target_frac=0.15, ramp_frames=0, action='idle',
                 tick_count=999, relax_ticks=0)
    _call_update(lac, 0.012, v_ego=v)
    assert state['action'] == 'relax_dwell'
    assert state['target_frac'] == pytest.approx(0.15)   # not zeroed / shed
    assert state['guard'].suppressed == 0

  def test_guard_uncommitted_after_reengage(self, monkeypatch):
    """Disengage -> re-engage starts the guard with no committed side."""
    lac, fake_sm, mod, state = self._engaged(monkeypatch)
    state['tick_count'] = 999; _call_update(lac, +0.002, v_ego=25.0)   # commit right
    assert state['guard'].side == +1
    _call_update(lac, 0.0, v_ego=25.0, active=False)
    _call_update(lac, 0.0, v_ego=25.0, active=True)
    assert state['guard'].side == 0

  # ---- free-play take-up, contact learning, telemetry ----
  # Learned contacts (route 4e5): gap 77.6 deg under right torque, 65.0 under left.

  def _start_takeup(self, monkeypatch):
    """Seated left (gap 65), torque at 0, a right ramp in flight: this tick
    crosses zero toward the unseated right flank."""
    lac, fake_sm, mod, state = self._engaged(monkeypatch)
    _seed_learner(state, 77.6, 65.0)
    state.update(torque=0.0, target_frac=0.04, ramp_step=+0.002, ramp_frames=20,
                 action='ramp', tick_count=0)
    out = _call_update(lac, 0.002, v_ego=25.0, gap_deg=65.0)
    return lac, fake_sm, mod, state, out

  def test_takeup_waits_for_learned_contacts(self, monkeypatch):
    """No persistence: a drive starts with unknown contacts and the take-up
    stays inert until the learner has both sides; then the same reversal
    fires. The seed/parse/format/fresh API is gone with the persistence."""
    import inspect, bmw.takeup as takeup
    assert 'seed' not in inspect.signature(takeup.ContactLearner.__init__).parameters
    assert not hasattr(takeup.ContactLearner, 'fresh')
    assert not hasattr(takeup, 'parse_contacts') and not hasattr(takeup, 'format_contacts')
    lac, fake_sm, mod, state = self._engaged(monkeypatch)
    assert state['learner'].contacts() is None
    state.update(torque=0.0, target_frac=0.04, ramp_step=+0.002, ramp_frames=20, action='ramp', tick_count=0)
    _call_update(lac, 0.002, v_ego=25.0, gap_deg=65.0)
    assert state['takeup'].fires == 0 and state['action'] != 'take_up'
    _seed_learner(state, 77.6, 65.0)
    state.update(torque=0.0, target_frac=0.04, ramp_step=+0.002, ramp_frames=20, action='ramp', tick_count=0)
    _call_update(lac, 0.002, v_ego=25.0, gap_deg=65.0)
    assert state['takeup'].fires == 1 and state['action'] == 'take_up'

  def test_takeup_overrides_torque_on_reversal(self, monkeypatch):
    from bmw.takeup import TAKEUP_NM
    lac, fake_sm, mod, state, out = self._start_takeup(monkeypatch)
    assert state['takeup'].state == 'taking_up' and state['takeup'].fires == 1
    assert state['torque'] == pytest.approx(TAKEUP_NM / 12.0)   # CCP.STEER_MAX = 12 Nm
    assert out[0] == pytest.approx(-TAKEUP_NM / 12.0)
    assert state['ramp_frames'] == 20          # ramp held, not applied or decremented
    assert state['action'] == 'take_up'


  def test_takeup_handover_ramps_from_takeup_level(self, monkeypatch):
    """Gap reaches the right contact -> hand over: the remaining ramp is
    re-aimed at target_frac from the take-up level and applies this tick."""
    from bmw.takeup import TAKEUP_NM
    lac, fake_sm, mod, state, _ = self._start_takeup(monkeypatch)
    tu = TAKEUP_NM / 12.0
    _call_update(lac, 0.002, v_ego=25.0, gap_deg=77.6)
    assert state['takeup'].state == 'right'
    assert state['takeup'].last_ms == pytest.approx(10.0)
    assert state['ramp_step'] == pytest.approx((0.04 - tu) / 20)
    assert state['torque'] == pytest.approx(tu + (0.04 - tu) / 20)
    assert state['ramp_frames'] == 19
    assert state['action'] == 'ramp'           # the decision label resumes

  def test_disengage_mid_takeup_resets(self, monkeypatch):        # Review Focus 3
    lac, fake_sm, mod, state, _ = self._start_takeup(monkeypatch)
    out = _call_update(lac, 0.002, v_ego=25.0, active=False, gap_deg=65.0)
    assert out[0] == 0.0
    assert state['takeup'].state != 'taking_up'
    assert state['action'] != 'take_up'

  def test_shed_to_zero_does_not_fire(self, monkeypatch):
    """Shedding to a zero target (e.g. a guard-suppressed reversal) can land a
    float epsilon past zero; that is not a push toward the new side."""
    lac, fake_sm, mod, state = self._engaged(monkeypatch)
    _seed_learner(state, 77.6, 65.0)
    state.update(torque=0.01, target_frac=0.0, ramp_step=-(0.01 + 1e-12), ramp_frames=1,
                 tick_count=0)
    _call_update(lac, 0.0, v_ego=25.0, gap_deg=77.6)       # seated right
    assert state['takeup'].fires == 0
    assert state['torque'] == pytest.approx(0.0, abs=1e-9)

  def test_learner_observes_every_tick_in_both_modes(self, monkeypatch):
    lac, fake_sm, mod, state = self._engaged(monkeypatch)
    calls = []
    monkeypatch.setattr(state['learner'], 'observe',
                        lambda gap, tq, rate, ok: calls.append((gap, tq, rate, ok)))
    state.update(torque=0.05, ramp_frames=0, tick_count=0)
    _call_update(lac, 0.0, v_ego=25.0, gap_deg=77.6, steer_rate=0.3)
    assert calls[-1] == (77.6, pytest.approx(0.6), 0.3, True)
    _call_update(lac, 0.0, v_ego=25.0, gap_deg=77.6, pressed=True)
    assert calls[-1][3] is False                # hands on: no sample
    _call_update(lac, 0.0, v_ego=25.0, active=False, gap_deg=77.6)
    assert calls[-1][3] is False                # inactive: no sample

  def test_learner_exception_never_breaks_lateral(self, monkeypatch):
    """Ruling R12: observe runs every tick; an exception in it
    must never break lateral — the tick completes and the ramp still applies."""
    lac, fake_sm, mod, state = self._engaged(monkeypatch)
    def _boom(*a, **k):
      raise ValueError('bad sample')
    monkeypatch.setattr(state['learner'], 'observe', _boom)
    state.update(torque=0.0, target_frac=0.04, ramp_step=+0.002, ramp_frames=20,
                 action='ramp', tick_count=0)
    out = _call_update(lac, 0.002, v_ego=25.0, gap_deg=77.6)
    assert state['torque'] == pytest.approx(0.002)
    assert out[0] == pytest.approx(-0.002)

  def test_telemetry_has_direct_fields(self, monkeypatch):
    from bmw.takeup import TAKEUP_NM
    lac, fake_sm, mod, state, _ = self._start_takeup(monkeypatch)
    pay = []
    state['lat_pub'] = SimpleNamespace(send=pay.append)
    state['guard'].suppressed = 2
    _call_update(lac, 0.002, v_ego=25.0, gap_deg=77.6)     # handover tick
    p = pay[-1]
    assert p['tu_state'] == 'right'
    assert p['c_right'] == pytest.approx(77.6) and p['c_left'] == pytest.approx(65.0)
    assert p['tu_fires'] == 1 and p['tu_timeouts'] == 0
    assert p['tu_ms'] == pytest.approx(10.0)
    assert p['tu_guarded'] == 2
    _call_update(lac, 0.002, v_ego=25.0, gap_deg=77.6)
    assert pay[-1]['action'] != 'take_up'

  def test_payload_exact_key_set(self, monkeypatch):
    """The telemetry carries exactly the fields the current law produces."""
    lac, fake_sm, mod, state = self._engaged(monkeypatch)
    pay = []
    state['lat_pub'] = SimpleNamespace(send=pay.append)
    _call_update(lac, 0.001, v_ego=25.0)
    assert set(pay[-1]) == {
      'desired', 'measured', 'err', 'delta_err', 'target_frac', 'ramp_step', 'ramp_frames', 'action',
      'torque', 'output', 'vEgo', 'active', 'ff_nm', 'a_ref', 'relax_ticks', 'angle',
      'servo_gap', 'steer_rate', 'pressed', 'lat_delay', 'tu_state', 'c_right', 'c_left', 'tu_fires', 'tu_ms', 'tu_timeouts', 'tu_guarded', 'sl_bias', 'sl_learn'}

  def test_telemetry_has_steering_signals(self, monkeypatch):
    """Steering-side signals ride the plugin bus with the controller state:
    wheel angle, servo-to-column gap (slack/windup), steering rate, driver
    pressed, and the lat_delay the controller was given."""
    lac, sm, mod, state = _make_controller(monkeypatch)
    payloads = []
    state['lat_pub'] = SimpleNamespace(send=payloads.append)
    _set_measured(sm, 20.0, 0.0)
    CS = SimpleNamespace(vEgo=20.0, steeringAngleDeg=-1.5, steeringAngleOffsetDeg=-14.2,
                         steeringRateDeg=3.0, steeringPressed=True)
    lac.update(True, CS, None, None, False, 0.001, False, 0.6)
    p = payloads[-1]
    assert p['angle'] == pytest.approx(-1.5)
    assert p['servo_gap'] == pytest.approx(-14.2)
    assert p['steer_rate'] == pytest.approx(3.0)
    assert p['pressed'] is True
    assert p['lat_delay'] == pytest.approx(0.6)

  def test_telemetry_tolerates_a_minimal_car_state(self, monkeypatch):
    lac, sm, mod, state = _make_controller(monkeypatch)
    payloads = []
    state['lat_pub'] = SimpleNamespace(send=payloads.append)
    _set_measured(sm, 20.0, 0.0)
    _call_update(lac, 0.001, v_ego=20.0)
    assert payloads and payloads[-1]['servo_gap'] == 0.0 and payloads[-1]['pressed'] is False

  def test_telemetry_action_take_up(self, monkeypatch):
    lac, fake_sm, mod, state, _ = self._start_takeup(monkeypatch)
    pay = []
    state['lat_pub'] = SimpleNamespace(send=pay.append)
    _call_update(lac, 0.002, v_ego=25.0, gap_deg=70.0)     # still taking up
    assert pay[-1]['action'] == 'take_up' and pay[-1]['tu_state'] == 'taking_up'

  def test_telemetry_unknown_contacts_read_zero(self, monkeypatch):
    lac, fake_sm, mod, state = self._engaged(monkeypatch)
    pay = []
    state['lat_pub'] = SimpleNamespace(send=pay.append)
    _call_update(lac, 0.0, v_ego=25.0)
    p = pay[-1]
    assert p['c_right'] == 0.0 and p['c_left'] == 0.0
    assert p['tu_state'] == 'unknown'

  # ---- Task 4 review fixes ----

  def _shed_left_to_zero(self, lac, state, n=3):
    """Shed -0.01 to a 0 target over n frames; n=3 lands at +8.7e-19 (new side)
    in plain float arithmetic. Seated left (gap 65)."""
    state.update(torque=-0.01, target_frac=0.0, ramp_step=(0.0 - -0.01) / n,
                 ramp_frames=n, action='ramp')
    for _ in range(n):
      state['tick_count'] = 0
      _call_update(lac, 0.0, v_ego=25.0, gap_deg=65.0)

  def test_shed_to_zero_snaps_exactly_then_reversal_fires(self, monkeypatch):
    """Review fix 1: a shed to a 0 target ends at exactly 0.0, so the genuine
    reversal that follows still crosses zero and fires the take-up."""
    lac, fake_sm, mod, state = self._engaged(monkeypatch)
    _seed_learner(state, 77.6, 65.0)
    self._shed_left_to_zero(lac, state)
    assert state['ramp_frames'] == 0 and state['torque'] == 0.0
    assert state['takeup'].fires == 0
    state.update(target_frac=0.04, ramp_step=+0.002, ramp_frames=20, action='ramp',
                 tick_count=0)
    _call_update(lac, 0.002, v_ego=25.0, gap_deg=65.0)
    assert state['takeup'].fires == 1 and state['takeup'].state == 'taking_up'


  def test_driver_override_mid_takeup_resets_and_resumes_ramp(self, monkeypatch):
    """Review fix 2 (Focus 3, override half): steeringPressed while still
    active ends the take-up (driver-driven SZL motion must not hand over
    against the driver); the held ramp resumes from the take-up level."""
    from bmw.takeup import TAKEUP_NM
    lac, fake_sm, mod, state, _ = self._start_takeup(monkeypatch)
    tu = TAKEUP_NM / 12.0
    state['tick_count'] = 0
    _call_update(lac, 0.002, v_ego=25.0, gap_deg=70.0, pressed=True)
    assert state['takeup'].state != 'taking_up' and state['takeup'].fires == 1
    assert state['ramp_step'] == pytest.approx((0.04 - tu) / 20)
    assert state['torque'] == pytest.approx(tu + (0.04 - tu) / 20)
    assert state['ramp_frames'] == 19
    assert state['action'] == 'ramp'

  def test_small_reversal_target_does_not_fire(self, monkeypatch):
    """Ruling R9: a 0.012 Nm reversal target is not a real reversal — plain
    ramp, no take-up."""
    lac, fake_sm, mod, state = self._engaged(monkeypatch)
    _seed_learner(state, 77.6, 65.0)
    state.update(torque=0.0, target_frac=0.001, ramp_step=+0.0001, ramp_frames=10,
                 action='ramp', tick_count=0)
    _call_update(lac, 0.0, v_ego=25.0, gap_deg=65.0)
    assert state['takeup'].fires == 0 and state['takeup'].state != 'taking_up'
    assert state['torque'] == pytest.approx(0.0001)
    assert state['action'] == 'ramp'

  def test_reversal_mid_takeup_ends_it(self, monkeypatch):
    """Ruling R9: a new decision target off the take-up side (here 0, a
    guard-held reversal) ends the take-up; the ramp re-aims from its level."""
    from bmw.takeup import TAKEUP_NM
    lac, fake_sm, mod, state, _ = self._start_takeup(monkeypatch)
    tu = TAKEUP_NM / 12.0
    state.update(target_frac=0.0, tick_count=0)
    _call_update(lac, 0.0, v_ego=25.0, gap_deg=70.0)       # mid-gap, contact not reached
    assert state['takeup'].state != 'taking_up' and state['takeup'].timeouts == 0
    assert state['ramp_step'] == pytest.approx((0.0 - tu) / 20)
    assert state['torque'] == pytest.approx(tu - tu / 20)
    assert state['ramp_frames'] == 19
    assert state['action'] == 'ramp'

  def test_step_limited_handover_holds_the_takeup_level(self, monkeypatch):
    """A take-up fired on the demand for a capped target under TAKEUP_NM
    (0.078 Nm from -0.055) hands over at the take-up level, not ramping down
    off the closed flank; the next decision steps up from it."""
    from bmw.takeup import TAKEUP_NM
    lac, fake_sm, mod, state = self._engaged(monkeypatch)
    _seed_learner(state, 77.6, 65.0)
    g = state['guard']
    g.side, g._opp = -1, 10
    state.update(torque=-0.055, target_frac=-0.055, ramp_frames=0, action='idle',
                 tick_count=999)
    gap, fired_low, floor = 65.0, None, None
    for i in range(80):
      fake_sm.updated = {'livePose': i % 5 == 0}
      if state['takeup'].state == 'taking_up':
        gap = 77.6                               # contact one tick after the fire
        fired_low = state['target_frac'] * 12.0 < TAKEUP_NM
      _call_update(lac, 0.0015, v_ego=25.0, gap_deg=gap)
      if state['takeup'].fires and state['action'] != 'take_up':
        floor = state['torque'] if floor is None else min(floor, state['torque'])
    assert state['takeup'].fires == 1 and fired_low
    assert floor * 12.0 >= TAKEUP_NM - 1e-9

  @pytest.mark.parametrize('torque0', [-0.045, -0.055])
  def test_step_limited_crossing_still_fires(self, monkeypatch, torque0):
    """Final-review residual: from 0.24-0.66 Nm of old-side torque at 25 m/s
    the STEP_MAX-capped crossing lands under TAKEUP_NM (0.0065 frac at
    -0.055) and the next decision is already past zero. The gate sizes the
    reversal on the pre-cap demand, so the take-up still fires once."""
    lac, fake_sm, mod, state = self._engaged(monkeypatch)
    _seed_learner(state, 77.6, 65.0)
    g = state['guard']
    g.side, g._opp = -1, 10                    # opposite sign already persisted
    state.update(torque=torque0, target_frac=torque0, ramp_frames=0, action='idle',
                 tick_count=999)
    for i in range(60):
      fake_sm.updated = {'livePose': i % 5 == 0}
      _call_update(lac, 0.0015, v_ego=25.0, gap_deg=65.0)    # P demand >> TAKEUP_NM, right
    assert state['tu_demand'] * 12.0 > 1.0
    assert state['takeup'].fires == 1

  def test_takeup_fault_falls_back_to_the_ramp(self, monkeypatch):
    """A take-up exception on the 100 Hz path drops to the plain ramp."""
    lac, fake_sm, mod, state = self._engaged(monkeypatch)
    _seed_learner(state, 77.6, 65.0)
    def _boom(*a, **k):
      raise RuntimeError('takeup fault')
    monkeypatch.setattr(state['takeup'], 'step', _boom)
    state.update(torque=0.0, target_frac=0.04, ramp_step=+0.002, ramp_frames=20,
                 action='ramp', tick_count=0)
    _call_update(lac, 0.002, v_ego=25.0, gap_deg=65.0)
    assert state['torque'] == pytest.approx(0.002) and state['action'] == 'ramp'

  def test_no_takeup_fires_while_pressed(self, monkeypatch):
    lac, fake_sm, mod, state = self._engaged(monkeypatch)
    _seed_learner(state, 77.6, 65.0)
    state.update(torque=0.0, target_frac=0.04, ramp_step=+0.002, ramp_frames=20,
                 action='ramp', tick_count=0)
    _call_update(lac, 0.002, v_ego=25.0, gap_deg=65.0, pressed=True)
    assert state['takeup'].fires == 0
    assert state['torque'] == pytest.approx(0.002)         # plain ramp


# ============================================================
# Slope bias (docs/superpowers/specs/2026-10-19-bmw-slope-bias-design.md):
# a slow, bounded torque bias learned from the P term on straights while
# Direct Tracking is latched, added to Direct's P target.
# ============================================================

class TestSlopeBias:
  V = 25.0
  L = 2.66


  def _engaged(self, monkeypatch, slope=True, v=V):
    lac, fake_sm, mod, state = _make_controller(monkeypatch, slope=slope)
    _set_measured(fake_sm, v, 0.0)
    _call_update(lac, 0.0, v_ego=v, active=False)
    _call_update(lac, 0.0, v_ego=v, active=True)
    return lac, fake_sm, mod, state

  def _p_nm(self, kappa_meas, v=V):
    # T_CAP_SLOPE_BASE 1.0 x kappa_scale 1.25 (straight) x v^2 x delta_err
    return 1.25 * v * v * (0.0 - math.atan(kappa_meas * self.L))

  def _run(self, lac, fake_sm, n, desired=0.0, kappa_meas=0.0, v=V, **kw):
    _set_measured(fake_sm, v, kappa_meas)
    for _ in range(n):
      fake_sm.updated = {'livePose': True}
      _call_update(lac, desired, v_ego=v, **kw)

  @pytest.mark.parametrize('kappa_meas', [2e-4, -2e-4])
  def test_learns_toward_standing_p_both_signs(self, monkeypatch, kappa_meas):
    lac, fake_sm, mod, state = self._engaged(monkeypatch)
    b0 = state['sl_bias']
    self._run(lac, fake_sm, 10, kappa_meas=kappa_meas)
    expect = 10 * self._p_nm(kappa_meas) * 0.05 / mod.SL_TAU_S
    assert state['sl_bias'] - b0 == pytest.approx(expect, rel=0.15)
    assert math.copysign(1.0, state['sl_bias']) == -math.copysign(1.0, kappa_meas)
    self._run(lac, fake_sm, 2000, kappa_meas=kappa_meas)
    assert state['sl_bias'] == pytest.approx(-math.copysign(mod.SL_BIAS_MAX_NM, kappa_meas))

  def test_clamp_covers_steep_road_need(self, monkeypatch):
    """Route 4ea: roads with roll >= 2.3 deg need -0.20..-0.25 Nm; the 0.15
    clamp pinned the bias and left the car +0.3 m right on them."""
    lac, fake_sm, mod, state = self._engaged(monkeypatch)
    self._run(lac, fake_sm, 4000, kappa_meas=2e-4)
    assert state['sl_bias'] == pytest.approx(-0.25)
    assert mod.SL_BIAS_MAX_NM < 0.5   # still well below rack breakaway

  def test_bias_added_to_p_target(self, monkeypatch):
    lac, fake_sm, mod, state = self._engaged(monkeypatch)
    state.update(sl_bias=-0.1, torque=-0.1 / 12.0, tick_count=999)
    _set_measured(fake_sm, self.V, 0.0)
    fake_sm.updated = {'livePose': True}
    _call_update(lac, 0.0, v_ego=self.V)
    assert state['target_frac'] * 12.0 == pytest.approx(-0.1, abs=1e-6)

  def test_curve_holds_bias_without_learning(self, monkeypatch):
    lac, fake_sm, mod, state = self._engaged(monkeypatch)
    state['sl_bias'] = -0.05
    self._run(lac, fake_sm, 200, desired=3e-3, kappa_meas=2e-3)
    assert state['sl_bias'] == -0.05 and state['sl_learn'] is False

  def test_low_speed_no_learning(self, monkeypatch):
    lac, fake_sm, mod, state = self._engaged(monkeypatch, v=10.0)
    self._run(lac, fake_sm, 100, kappa_meas=2e-4, v=10.0)
    assert state['sl_bias'] == 0.0

  @pytest.mark.parametrize('kappa_meas', [2e-4, -2e-4])
  def test_press_freezes_bias(self, monkeypatch, kappa_meas):
    lac, fake_sm, mod, state = self._engaged(monkeypatch)
    state['sl_bias'] = 0.03
    self._run(lac, fake_sm, 100, kappa_meas=kappa_meas, pressed=True)
    assert state['sl_bias'] == 0.03

  def test_takeup_closes_gate(self, monkeypatch):
    """A take-up in flight closes the gate."""
    lac, fake_sm, mod, state = self._engaged(monkeypatch)
    self._run(lac, fake_sm, 1, kappa_meas=1.2e-3)   # straight, a clear error
    state['sl_bias'] = 0.03
    for _ in range(20):
      state['takeup'].state = 'taking_up'
      fake_sm.updated = {'livePose': True}
      _call_update(lac, 0.0, v_ego=self.V)
      assert state['sl_bias'] == 0.03 and state['sl_learn'] is False

  def test_bias_survives_reengage(self, monkeypatch):
    lac, fake_sm, mod, state = self._engaged(monkeypatch)
    self._run(lac, fake_sm, 50, kappa_meas=2e-4)
    b = state['sl_bias']
    assert b < 0.0
    _set_measured(fake_sm, self.V, 0.0)          # zero error: no learning on the re-engage tick
    _call_update(lac, 0.0, v_ego=self.V, active=False)
    assert not state['slope']
    _call_update(lac, 0.0, v_ego=self.V, active=True)
    assert state['slope'] and state['sl_bias'] == b

  def test_bias_frozen_above_a_ref_gate(self, monkeypatch):
    """A gentle bend above SL_AY_MAX (kappa 4e-4 at 25 m/s = 0.25 m/s^2) does
    not charge the bias although |kappa| < SL_KAPPA_DES; below the gate
    (2e-4 = 0.125 m/s^2) learning runs."""
    lac, fake_sm, mod, state = self._engaged(monkeypatch)
    self._run(lac, fake_sm, 1, desired=4e-4, kappa_meas=4e-4)
    assert state['sl_learn'] is False
    self._run(lac, fake_sm, 1, desired=2e-4, kappa_meas=2e-4)
    assert state['sl_learn'] is True

  def test_guard_judges_the_full_demand(self, monkeypatch):
    """Final review, Important 1: the noise guard must observe the same
    demand it judges. With a left bias (-0.1 Nm) and the guard committed right
    after a correction, a steady small positive error (P +0.05 Nm) asks for
    P + b = -0.05 Nm. The error itself never turns negative, so a guard that
    observes delta_err alone would hold the target at 0 and drop the bias."""
    lac, fake_sm, mod, state = self._engaged(monkeypatch)
    state['sl_bias'] = -0.10
    state['guard'].side = +1
    kappa = -math.tan(0.05 / (self.V * self.V)) / self.L     # P = +0.05 Nm
    targets = []
    _set_measured(fake_sm, self.V, kappa)
    for _ in range(40):
      fake_sm.updated = {'livePose': True}
      _call_update(lac, 0.0, v_ego=self.V)
      if state['tick_count'] == 0:
        targets.append(state['target_frac'] * 12.0)
    assert state['guard'].side == -1
    assert min(targets) < -0.01

  def _mixed(self, lac, fake_sm):
    out = []
    for i in range(300):
      v = 25.0 if i < 200 else 9.0
      desired = 3e-3 if 60 <= i < 120 else 0.0
      _set_measured(fake_sm, v, 2e-4 if i % 40 < 20 else -1e-4)
      fake_sm.updated = {'livePose': i % 5 == 0}
      _call_update(lac, desired, v_ego=v, pressed=150 <= i < 170)
      out.append(lac.update.__closure__ and _closure_state(lac.update)['torque'])
    return out


  def test_slope_off_bit_identical(self, monkeypatch):
    lac, fake_sm, mod, state = self._engaged(monkeypatch, slope=False)
    a = self._mixed(lac, fake_sm)
    lac2, fake_sm2, _, _ = self._engaged(monkeypatch, slope=None)
    assert a == self._mixed(lac2, fake_sm2)
