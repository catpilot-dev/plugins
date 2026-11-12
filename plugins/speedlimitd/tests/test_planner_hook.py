"""Tests for planner_hook — the jerk-limited allowed-speed ceiling.

Spec: docs/superpowers/specs/2026-09-10-speedlimitd-jerk-limited-ceiling-design.md
"""
import os
import sys
import importlib
import pytest
from unittest.mock import MagicMock
from types import SimpleNamespace

_PLUGINS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
if _PLUGINS_DIR not in sys.path:
  sys.path.insert(0, _PLUGINS_DIR)
_REPO_ROOT = os.path.dirname(_PLUGINS_DIR)
if _REPO_ROOT not in sys.path:
  sys.path.insert(0, _REPO_ROOT)


@pytest.fixture
def ph():
  """planner_hook with openpilot's CV stubbed, module state cleared."""
  mock_cv = MagicMock()
  mock_cv.KPH_TO_MS = 1.0 / 3.6
  mock_cv.MS_TO_KPH = 3.6
  sys.modules['openpilot.common'] = MagicMock()
  sys.modules['openpilot.common.constants'] = MagicMock(CV=mock_cv)

  import plugins.speedlimitd.planner_hook as mod
  importlib.reload(mod)
  mod._sl_sub = None
  mod._sl_data = None
  mod._baseline_ms = None
  mod._gas_floor_ms = None
  mod._road_id = ''
  mod._ceiling_ms = None
  mod._ceiling_rate = 0.0
  mod._last_t = None
  return mod


DT = 0.05  # plannerd runs on modelV2 — DT_MDL


def _trace(ph, start_kph, target_kph, dt=DT, max_s=120.0):
  """Advance the ceiling to the target, returning [(t, ceiling_ms, rate_ms2)]."""
  ceiling = start_kph / 3.6
  target = target_kph / 3.6
  rate = 0.0
  out = [(0.0, ceiling, rate)]
  t = 0.0
  while t < max_s:
    ceiling, rate = ph._advance_ceiling(ceiling, rate, target, dt)
    t += dt
    out.append((t, ceiling, rate))
    if ceiling == target and rate == 0.0:
      break
  return out


class TestAdvanceCeiling:
  def test_jerk_bound_on_descent(self, ph):
    """No tick may change the rate by more than J_DOWN * dt. This is the point
    of the whole design: DCC reacts to setpoint gap, so the slope must be
    continuous, not just the value."""
    tr = _trace(ph, 88.0, 46.0)
    for (_, _, r0), (_, _, r1) in zip(tr, tr[1:]):
      assert abs(r1 - r0) <= ph.CEIL_J_DOWN * DT + 1e-9

  def test_value_continuity_on_descent(self, ph):
    """No tick may change the ceiling by more than A_DOWN * dt."""
    tr = _trace(ph, 88.0, 46.0)
    for (_, c0, _), (_, c1, _) in zip(tr, tr[1:]):
      assert abs(c1 - c0) <= ph.CEIL_A_DOWN * DT + 1e-9

  def test_descent_is_monotonic_and_does_not_overshoot(self, ph):
    tr = _trace(ph, 88.0, 46.0)
    target = 46.0 / 3.6
    for (_, c0, _), (_, c1, _) in zip(tr, tr[1:]):
      assert c1 <= c0 + 1e-9, 'ceiling rose during a descent'
    for _, c, _ in tr:
      assert c >= target - 1e-9, 'ceiling undershot the target'

  def test_arrives_with_zero_slope(self, ph):
    tr = _trace(ph, 88.0, 46.0)
    _, c_end, r_end = tr[-1]
    assert c_end == pytest.approx(46.0 / 3.6, abs=1e-9)
    assert r_end == 0.0

  def test_peak_descent_rate_respects_a_down(self, ph):
    tr = _trace(ph, 88.0, 46.0)
    assert min(r for _, _, r in tr) >= -ph.CEIL_A_DOWN - 1e-9

  def test_80_to_40_lands_in_the_expected_window(self, ph):
    """Spec: ~24 s. Ramp-in 1.0 s + 11.1/0.5 hold + ramp-out 1.0 s."""
    tr = _trace(ph, 88.0, 46.0)
    assert 22.0 <= tr[-1][0] <= 27.0

  def test_rising_limit_is_applied_immediately(self, ph):
    """A rising limit is a release, not a manoeuvre. The hook only ever lowers
    v_cruise, so raising the ceiling merely stops capping — DCC's own
    acceleration envelope shapes what follows. Ramping it only delayed it."""
    c, r = ph._advance_ceiling(46.0 / 3.6, 0.0, 88.0 / 3.6, DT)
    assert c == pytest.approx(88.0 / 3.6, abs=1e-9)
    assert r == 0.0

  def test_rising_limit_is_immediate_even_from_a_standstill_ceiling(self, ph):
    c, r = ph._advance_ceiling(30.0 / 3.6, 0.0, 120.0 / 3.6, DT)
    assert c == pytest.approx(120.0 / 3.6, abs=1e-9)
    assert r == 0.0

  def test_retarget_mid_ramp_stays_continuous(self, ph):
    """80 -> 60, then 40 arrives while still ramping. No step in value or slope."""
    target_a = 69.0 / 3.6
    target_b = 46.0 / 3.6
    ceiling, rate = 88.0 / 3.6, 0.0
    prev = (ceiling, rate)
    for i in range(1200):
      target = target_a if i < 40 else target_b
      ceiling, rate = ph._advance_ceiling(ceiling, rate, target, DT)
      assert abs(rate - prev[1]) <= ph.CEIL_J_DOWN * DT + 1e-9
      assert abs(ceiling - prev[0]) <= ph.CEIL_A_DOWN * DT + 1e-9
      prev = (ceiling, rate)
    assert ceiling == pytest.approx(target_b, abs=1e-9)
    assert rate == 0.0

  def test_reversal_mid_descent_releases_at_once(self, ph):
    """The limit rises while the ceiling is still falling. The release wins
    immediately and the descent slope is dropped with it."""
    ceiling, rate = 88.0 / 3.6, 0.0
    for _ in range(40):
      ceiling, rate = ph._advance_ceiling(ceiling, rate, 46.0 / 3.6, DT)
    assert rate < 0.0, 'precondition: still descending'
    ceiling, rate = ph._advance_ceiling(ceiling, rate, 100.0 / 3.6, DT)
    assert ceiling == pytest.approx(100.0 / 3.6, abs=1e-9)
    assert rate == 0.0

  def test_already_at_target_is_a_no_op(self, ph):
    target = 46.0 / 3.6
    assert ph._advance_ceiling(target, 0.0, target, DT) == (target, 0.0)

  def test_zero_dt_does_not_move_the_ceiling(self, ph):
    c, r = ph._advance_ceiling(88.0 / 3.6, 0.0, 46.0 / 3.6, 0.0)
    assert c == pytest.approx(88.0 / 3.6)
    assert r == 0.0


class TestCeilingInOnVCruise:
  """The ceiling as on_v_cruise actually drives it, with a fake clock."""

  def _sm(self, gas=False, lead_present=False, lead_vLead=0.0):
    cs = MagicMock()
    cs.gasPressed = gas
    # Shaped like 0.11.2's RadarState.LeadData: `present` (renamed from
    # `status`, upstream #38339). A strict object, so reading a field the
    # schema does not have raises as capnp does on the car.
    lead = SimpleNamespace(present=lead_present, vLead=lead_vLead)
    radar = MagicMock()
    radar.leadOne = lead
    sm = MagicMock()

    def getitem(key):
      if key == 'carState':
        return cs
      if key == 'radarState':
        return radar
      return MagicMock()

    sm.__getitem__ = MagicMock(side_effect=getitem)
    return sm

  def _sl(self, ph, speed_limit, source=1, safety=False, confirmed=True, road='A'):
    ph._sl_data = {'confirmed': confirmed, 'speedLimit': speed_limit,
                   'safetyCapped': safety, 'source': source,
                   'roadName': road, 'wayRef': '', 'safetyCap': 0}

  def _clock(self, ph, monkeypatch, t0=1000.0):
    """Fake monotonic clock. `clk['t'] += DT` advances one planner tick."""
    clk = {'t': t0}
    monkeypatch.setattr(ph.time, 'monotonic', lambda: clk['t'])
    return clk

  # --- first reading is immediate, not ramped -------------------------

  def test_first_reading_applies_immediately(self, ph, monkeypatch):
    """No ramp from a null ceiling — there is nothing to ramp from."""
    self._clock(ph, monkeypatch)
    self._sl(ph, 80, source=1)
    assert ph.on_v_cruise(100 / 3.6, 25.0, self._sm()) == pytest.approx(88 / 3.6, abs=0.01)

  # --- a drop ramps ---------------------------------------------------

  def test_drop_does_not_jump(self, ph, monkeypatch):
    """80 -> 40 must not land on 46 km/h the very next tick."""
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 80, source=1)
    ph.on_v_cruise(100 / 3.6, 25.0, self._sm())
    self._sl(ph, 40, source=1)
    clk['t'] += DT
    out = ph.on_v_cruise(100 / 3.6, 25.0, self._sm())
    assert out > 80 / 3.6, 'ceiling teleported to the new limit'

  def test_drop_reaches_the_target(self, ph, monkeypatch):
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 80, source=1)
    ph.on_v_cruise(100 / 3.6, 25.0, self._sm())
    self._sl(ph, 40, source=1)
    out = None
    for _ in range(600):
      clk['t'] += DT
      out = ph.on_v_cruise(100 / 3.6, 25.0, self._sm())
    assert out == pytest.approx(46 / 3.6, abs=0.01)

  def test_drop_is_monotonic_and_never_speeds_up(self, ph, monkeypatch):
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 80, source=1)
    prev = ph.on_v_cruise(100 / 3.6, 25.0, self._sm())
    self._sl(ph, 40, source=1)
    for _ in range(600):
      clk['t'] += DT
      out = ph.on_v_cruise(100 / 3.6, 25.0, self._sm())
      assert out <= prev + 1e-9
      assert out <= 100 / 3.6 + 1e-9, 'returned above the incoming v_cruise'
      prev = out

  # --- safety caps bypass the ramp ------------------------------------

  def test_safety_cap_bypasses_the_ramp(self, ph, monkeypatch):
    """A tightening curve must bite now, not in 16 s."""
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 80, source=1)
    ph.on_v_cruise(100 / 3.6, 25.0, self._sm())
    self._sl(ph, 40, source=4, safety=True)
    clk['t'] += DT
    # safetyCapped => no offset, exact limit, immediately
    assert ph.on_v_cruise(100 / 3.6, 25.0, self._sm()) == pytest.approx(40 / 3.6, abs=0.01)

  def test_safety_cap_release_is_immediate(self, ph, monkeypatch):
    """Curve ends, limit goes back up: the cap lifts on the next tick rather
    than holding the car down while a ramp catches up."""
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 40, source=4, safety=True)
    ph.on_v_cruise(100 / 3.6, 25.0, self._sm())
    self._sl(ph, 80, source=1)
    clk['t'] += DT
    out = ph.on_v_cruise(100 / 3.6, 25.0, self._sm())
    assert out == pytest.approx(88 / 3.6, abs=0.01)

  # --- gas -------------------------------------------------------------

  def test_ceiling_keeps_ramping_while_gas_is_pressed(self, ph, monkeypatch):
    """The ceiling is a function of the limit, not of the driver. On release
    the limit must already be where it belongs, not 16 s behind."""
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 80, source=1)
    ph.on_v_cruise(100 / 3.6, 25.0, self._sm())
    self._sl(ph, 40, source=1)
    for _ in range(600):
      clk['t'] += DT
      ph.on_v_cruise(100 / 3.6, 25.0, self._sm(gas=True))
    assert ph._ceiling_ms == pytest.approx(46 / 3.6, abs=0.01)

  # --- reset -----------------------------------------------------------

  def test_invalid_limit_clears_the_ceiling(self, ph, monkeypatch):
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 80, source=1)
    ph.on_v_cruise(100 / 3.6, 25.0, self._sm())
    self._sl(ph, 80, source=1, confirmed=False)
    clk['t'] += DT
    ph.on_v_cruise(100 / 3.6, 25.0, self._sm())
    assert ph._ceiling_ms is None
    # …and the next valid limit is applied immediately, not ramped from stale state
    self._sl(ph, 40, source=1)
    clk['t'] += DT
    assert ph.on_v_cruise(100 / 3.6, 25.0, self._sm()) == pytest.approx(46 / 3.6, abs=0.01)

  def test_road_change_does_not_reset_the_ceiling(self, ph, monkeypatch):
    """Resetting on a road change would reintroduce exactly the jump we removed."""
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 80, source=1, road='A')
    ph.on_v_cruise(100 / 3.6, 25.0, self._sm())
    self._sl(ph, 40, source=1, road='B')
    clk['t'] += DT
    out = ph.on_v_cruise(100 / 3.6, 25.0, self._sm())
    assert out > 80 / 3.6, 'ceiling was reset by the road change'

  # --- dt hygiene -------------------------------------------------------

  def test_long_stall_does_not_lurch(self, ph, monkeypatch):
    """A 10 s gap between ticks (process stall) must be clamped to CEIL_DT_MAX,
    not integrated as a 10 s step."""
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 80, source=1)
    ph.on_v_cruise(100 / 3.6, 25.0, self._sm())
    self._sl(ph, 40, source=1)
    clk['t'] += 10.0
    out = ph.on_v_cruise(100 / 3.6, 25.0, self._sm())
    assert out > 87 / 3.6, 'a stalled tick integrated the whole gap'


class TestDescentAnchor:
  """A fresh drop anchors the ceiling to current speed.

  The ceiling used to begin every descent at the OLD limit + offset, even when
  the car was already far below it — on 80 -> 40 at 55 km/h that is ~20 s
  (~300 m) of dead air before the cap reaches the car and anything happens.
  Anchoring is a step in the CAP, not in the setpoint GAP: clamping 88 -> 55
  while the car is doing 55 leaves zero error, so there is nothing for DCC to
  react to. modelV2/MPC decides vTarget; the ceiling only bounds it.
  """
  _sm = TestCeilingInOnVCruise._sm
  _sl = TestCeilingInOnVCruise._sl
  _clock = TestCeilingInOnVCruise._clock

  def test_drop_anchors_ceiling_to_current_speed(self, ph, monkeypatch):
    clk = self._clock(ph, monkeypatch)
    v = 55 / 3.6
    self._sl(ph, 80, source=1)
    assert ph.on_v_cruise(120 / 3.6, v, self._sm()) == pytest.approx(88 / 3.6, abs=0.01)
    self._sl(ph, 40, source=1)
    clk['t'] += DT
    out = ph.on_v_cruise(120 / 3.6, v, self._sm())
    assert out == pytest.approx(v, abs=0.05), 'ceiling did not anchor to v_ego'

  def test_anchor_makes_the_cap_bite_at_once(self, ph, monkeypatch):
    clk = self._clock(ph, monkeypatch)
    v = 55 / 3.6
    self._sl(ph, 80, source=1)
    ph.on_v_cruise(120 / 3.6, v, self._sm())
    self._sl(ph, 40, source=1)
    clk['t'] += DT
    assert ph.on_v_cruise(120 / 3.6, v, self._sm()) <= v + 1e-9

  def test_anchor_never_goes_below_the_new_target(self, ph, monkeypatch):
    """Driver already slower than the new limit: the cap must not be dragged
    down to their speed and hold them there."""
    clk = self._clock(ph, monkeypatch)
    v = 30 / 3.6
    self._sl(ph, 80, source=1)
    ph.on_v_cruise(120 / 3.6, v, self._sm())
    self._sl(ph, 40, source=1)
    clk['t'] += DT
    ph.on_v_cruise(120 / 3.6, v, self._sm())
    assert ph._ceiling_ms == pytest.approx(46 / 3.6, abs=0.01)

  def test_anchor_never_raises_the_ceiling(self, ph, monkeypatch):
    """Driver above the old limit: anchoring must not hand them a higher cap."""
    clk = self._clock(ph, monkeypatch)
    v = 95 / 3.6
    self._sl(ph, 80, source=1)
    ph.on_v_cruise(120 / 3.6, v, self._sm())
    self._sl(ph, 40, source=1)
    clk['t'] += DT
    assert ph._ceiling_ms == pytest.approx(88 / 3.6, abs=0.05)

  def test_ramp_still_governs_after_the_anchor(self, ph, monkeypatch):
    """Anchoring sets where the descent starts; CEIL_A_DOWN still sets how fast."""
    clk = self._clock(ph, monkeypatch)
    v = 55 / 3.6
    self._sl(ph, 80, source=1)
    ph.on_v_cruise(120 / 3.6, v, self._sm())
    self._sl(ph, 40, source=1)
    prev = None
    for _ in range(900):
      clk['t'] += DT
      out = ph.on_v_cruise(120 / 3.6, v, self._sm())
      if prev is not None:
        assert out <= prev + 1e-9
        assert abs(out - prev) <= ph.CEIL_A_DOWN * DT + 1e-9, 'anchored ramp broke the rate limit'
      prev = out
    assert prev == pytest.approx(46 / 3.6, abs=0.01)

  def test_anchor_only_fires_on_a_fresh_drop(self, ph, monkeypatch):
    """Mid-ramp ticks must not keep re-anchoring to a decelerating v_ego —
    that would ratchet the cap down with the car."""
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 80, source=1)
    ph.on_v_cruise(120 / 3.6, 55 / 3.6, self._sm())
    self._sl(ph, 40, source=1)
    clk['t'] += DT
    ph.on_v_cruise(120 / 3.6, 55 / 3.6, self._sm())
    for _ in range(20):
      clk['t'] += DT
      ph.on_v_cruise(120 / 3.6, 20 / 3.6, self._sm())   # car brakes hard on its own
    assert ph._ceiling_ms > 46 / 3.6, 'ceiling ratcheted down with v_ego'


class TestEnforcementTelemetry:
  """The payload published to /tmp/plugin_bus/speedLimitEnforce.

  bus_logger auto-discovers topics, so these dicts land in pluginBusLog in the
  rlog. Without them a drive can only be analysed by inferring the cap from
  longitudinalPlan.aTarget — the MPC output, with lead-following mixed in.
  """
  _sm = TestCeilingInOnVCruise._sm
  _sl = TestCeilingInOnVCruise._sl
  _clock = TestCeilingInOnVCruise._clock

  @pytest.fixture
  def bus(self, ph, monkeypatch):
    """Capture published telemetry instead of binding a real socket."""
    sent = []
    ph._pub = type("P", (), {"send": staticmethod(lambda d: sent.append(dict(d)))})()
    ph._pub_ok = True
    return sent

  def test_publishes_every_tick_on_every_path(self, ph, bus, monkeypatch):
    clk = self._clock(ph, monkeypatch)
    ph._sl_data = None
    ph.on_v_cruise(100 / 3.6, 25.0, self._sm())          # no_data
    self._sl(ph, 80, source=1, confirmed=False)
    clk['t'] += DT; ph.on_v_cruise(100 / 3.6, 25.0, self._sm())   # unconfirmed
    self._sl(ph, 80, source=1)
    clk['t'] += DT; ph.on_v_cruise(100 / 3.6, 25.0, self._sm())   # capped
    clk['t'] += DT; ph.on_v_cruise(100 / 3.6, 25.0, self._sm(gas=True))  # gas
    states = [m['state'] for m in bus]
    assert states == ['no_data', 'unconfirmed', 'capped', 'gas']
    assert len(bus) == 4, 'one sample per tick, no gaps in the series'

  def test_capped_sample_carries_the_ceiling_and_the_cap(self, ph, bus, monkeypatch):
    self._clock(ph, monkeypatch)
    self._sl(ph, 80, source=1)
    out = ph.on_v_cruise(100 / 3.6, 20.0, self._sm())
    m = bus[-1]
    assert m['state'] == 'capped'
    assert m['capActive'] is True
    assert m['limit'] == 80
    assert m['target'] == pytest.approx(88.0, abs=0.05)
    assert m['ceiling'] == pytest.approx(88.0, abs=0.05)
    assert m['vCruiseOut'] == pytest.approx(out * 3.6, abs=0.05)
    assert m['vCruiseIn'] == pytest.approx(100.0, abs=0.05)
    assert m['vEgo'] == pytest.approx(72.0, abs=0.05)

  def test_not_binding_is_distinguishable_from_capped(self, ph, bus, monkeypatch):
    self._clock(ph, monkeypatch)
    self._sl(ph, 120, source=1)
    ph.on_v_cruise(40 / 3.6, 11.0, self._sm())
    assert bus[-1]['state'] == 'not_binding'
    assert bus[-1]['capActive'] is False

  def test_anchor_event_is_recorded_with_from_and_to(self, ph, bus, monkeypatch):
    """The anchor is a one-tick event — if it is not logged it is invisible."""
    clk = self._clock(ph, monkeypatch)
    v = 55 / 3.6
    self._sl(ph, 80, source=1)
    ph.on_v_cruise(120 / 3.6, v, self._sm())
    assert bus[-1]['anchored'] is False
    self._sl(ph, 40, source=1)
    clk['t'] += DT
    ph.on_v_cruise(120 / 3.6, v, self._sm())
    m = bus[-1]
    assert m['anchored'] is True
    assert m['anchorFrom'] == pytest.approx(88.0, abs=0.1)
    assert m['anchorTo'] == pytest.approx(55.0, abs=0.1)
    clk['t'] += DT
    ph.on_v_cruise(120 / 3.6, v, self._sm())
    assert bus[-1]['anchored'] is False, 'anchor flag must not stick across ticks'

  def test_ceiling_rate_is_published_so_jerk_is_measurable(self, ph, bus, monkeypatch):
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 80, source=1)
    ph.on_v_cruise(120 / 3.6, 25.0, self._sm())
    self._sl(ph, 40, source=1)
    for _ in range(60):
      clk['t'] += DT
      ph.on_v_cruise(120 / 3.6, 25.0, self._sm())
    rates = [m['ceilingRate'] for m in bus if m['state'] == 'capped']
    assert min(rates) < -0.3, 'descent slope never showed up in telemetry'
    for a, b in zip(rates, rates[1:]):
      assert abs(b - a) <= ph.CEIL_J_DOWN * DT + 1e-6

  def test_floors_are_visible(self, ph, bus, monkeypatch):
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 60, source=2, road='A')
    ph.on_v_cruise(120 / 3.6, 25.0, self._sm())
    assert bus[-1]['baselineFloor'] > 0.0
    clk['t'] += DT
    ph.on_v_cruise(120 / 3.6, 25.0, self._sm(gas=True))
    assert bus[-1]['gasFloor'] == pytest.approx(90.0, abs=0.1)

  def test_lead_override_is_visible(self, ph, bus, monkeypatch):
    """The override engages after the 0.5 s entry debounce, not on the first tick."""
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 80, source=1)
    sm = self._sm(lead_present=True, lead_vLead=100 / 3.6)
    ph.on_v_cruise(100 / 3.6, 25.0, sm)
    assert bus[-1]['state'] == 'capped'
    for _ in range(10):
      clk['t'] += DT
      ph.on_v_cruise(100 / 3.6, 25.0, sm)
    assert bus[-1]['state'] == 'lead_override'
    assert bus[-1]['capActive'] is False

  def test_a_broken_bus_never_reaches_the_planner(self, ph, monkeypatch):
    """Telemetry is best-effort. A publish failure must not change the cap."""
    self._clock(ph, monkeypatch)
    def boom(_): raise RuntimeError('bus gone')
    ph._pub = type("P", (), {"send": staticmethod(boom)})()
    ph._pub_ok = True
    self._sl(ph, 80, source=1)
    assert ph.on_v_cruise(100 / 3.6, 20.0, self._sm()) == pytest.approx(88 / 3.6, abs=0.01)


class TestFloorReleaseReanchor:
  """When a floor that was holding the output releases, the ceiling re-anchors
  at the car's speed instead of letting the output fall to a ceiling that kept
  descending in the background (route 536, 2026-11-11: 68 -> 54 km/h in one
  update on a road change, -1.08 m/s2 at a request near 0).
  """
  _sm = TestCeilingInOnVCruise._sm
  _sl = TestCeilingInOnVCruise._sl
  _clock = TestCeilingInOnVCruise._clock

  def _tick(self, ph, clk, v_cruise_kph, v_kph, sm=None, n=1):
    out = None
    for _ in range(n):
      clk['t'] += DT
      out = ph.on_v_cruise(v_cruise_kph / 3.6, v_kph / 3.6, sm or self._sm())
    return out

  def _baseline_hold(self, ph, clk, hold_s=7.0):
    """Inferred limit dropped 80 -> 40 while the car was at 56; the car then
    runs at 68 with the baseline floor (92) holding the output at v_ego while
    the ceiling descends underneath."""
    self._sl(ph, 80, source=2, road='A')
    self._tick(ph, clk, 100, 56)
    self._sl(ph, 40, source=2, road='A')
    self._tick(ph, clk, 100, 56)
    out = self._tick(ph, clk, 100, 68, n=int(hold_s / DT))
    assert out == pytest.approx(68 / 3.6, abs=1e-6), 'baseline floor should hold at v_ego'
    assert ph._ceiling_ms < 62 / 3.6, 'ceiling should have descended under the hold'

  def test_baseline_floor_release_does_not_step_the_output_down(self, ph, monkeypatch):
    clk = self._clock(ph, monkeypatch)
    self._baseline_hold(ph, clk)
    v = 68 / 3.6
    self._sl(ph, 40, source=2, road='B')          # road change drops the baseline
    out = self._tick(ph, clk, 100, 68)
    assert v - out <= ph.CEIL_A_DOWN * DT + 1e-9, 'output stepped down on floor release'

  def test_baseline_floor_release_then_descends_at_the_ceiling_rate_from_v_ego(self, ph, monkeypatch):
    clk = self._clock(ph, monkeypatch)
    self._baseline_hold(ph, clk)
    v = 68 / 3.6
    self._sl(ph, 40, source=2, road='B')
    prev = self._tick(ph, clk, 100, 68)
    for _ in range(100):
      out = self._tick(ph, clk, 100, 68)
      assert out <= prev + 1e-9
      assert prev - out <= ph.CEIL_A_DOWN * DT + 1e-9
      prev = out
    # 5 s after the release it is still on the ramp from v_ego, nowhere near the old ceiling
    assert prev < v - 0.5
    assert prev > 58 / 3.6

  def test_empty_road_id_release_does_not_step_down_either(self, ph, monkeypatch):
    """Route 536 seg 1: road name goes '' -> baseline cleared -> floor gone."""
    clk = self._clock(ph, monkeypatch)
    self._baseline_hold(ph, clk)
    self._sl(ph, 40, source=2, road='')
    out = self._tick(ph, clk, 100, 68)
    assert 68 / 3.6 - out <= ph.CEIL_A_DOWN * DT + 1e-9

  def test_release_is_recorded_in_the_anchor_telemetry(self, ph, monkeypatch):
    clk = self._clock(ph, monkeypatch)
    ph._pub = type("P", (), {"send": staticmethod(lambda d: sent.append(dict(d)))})()
    ph._pub_ok = True
    sent = []
    self._baseline_hold(ph, clk)
    ceil_before = ph._ceiling_ms * 3.6
    self._sl(ph, 40, source=2, road='B')
    self._tick(ph, clk, 100, 68)
    m = sent[-1]
    assert m['anchored'] is True
    assert m['anchorFrom'] == pytest.approx(ceil_before, abs=0.2)
    assert m['anchorTo'] == pytest.approx(68.0, abs=0.1)
    self._tick(ph, clk, 100, 68)
    assert sent[-1]['anchored'] is False

  def test_gas_floor_release_does_not_step_the_output_down(self, ph, monkeypatch):
    """Non-safety limit; the driver's gas floor holds at 68 while the ceiling
    descends; a road change clears it."""
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 80, source=1, road='A')
    self._tick(ph, clk, 100, 68)
    self._sl(ph, 40, source=1, road='A')
    self._tick(ph, clk, 100, 68, sm=self._sm(gas=True), n=int(7.0 / DT))
    out = self._tick(ph, clk, 100, 68, n=2)
    assert out == pytest.approx(68 / 3.6, abs=1e-6), 'gas floor should hold'
    assert ph._ceiling_ms < 66 / 3.6
    self._sl(ph, 40, source=1, road='B')
    out = self._tick(ph, clk, 100, 68)
    assert 68 / 3.6 - out <= ph.CEIL_A_DOWN * DT + 1e-9

  def test_safety_cap_still_acts_at_once_when_a_floor_releases(self, ph, monkeypatch):
    """A curve cap (source 4) arriving on the release tick is applied exactly,
    on that tick — the re-anchor never delays or raises it."""
    clk = self._clock(ph, monkeypatch)
    self._baseline_hold(ph, clk)
    self._sl(ph, 50, source=4, safety=True, road='A')
    out = self._tick(ph, clk, 100, 68)
    assert out == pytest.approx(50 / 3.6, abs=1e-6)
    assert ph._ceiling_ms == pytest.approx(50 / 3.6, abs=1e-6)

  def test_gas_floor_release_under_a_safety_cap_is_not_reanchored(self, ph, monkeypatch):
    """Route 536 seg 0 (src 4, limit 50): the gas floor clears on a road change
    and the output goes straight to the safety cap. Unchanged."""
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 50, source=4, safety=True, road='')
    self._tick(ph, clk, 100, 62.8, sm=self._sm(gas=True))
    out = self._tick(ph, clk, 100, 62.8, n=5)
    assert out == pytest.approx(62.8 / 3.6, abs=1e-6)
    self._sl(ph, 50, source=4, safety=True, road='X')
    out = self._tick(ph, clk, 100, 62.8)
    assert out == pytest.approx(50 / 3.6, abs=1e-6)

  def test_rising_limit_right_after_a_release_is_immediate(self, ph, monkeypatch):
    """Route 536 seg 4 +7.2: limit 40 -> 60 (69 km/h) 0.2 s after the release."""
    clk = self._clock(ph, monkeypatch)
    self._baseline_hold(ph, clk)
    self._sl(ph, 40, source=2, road='B')
    self._tick(ph, clk, 100, 68, n=4)
    self._sl(ph, 60, source=2, road='B')
    out = self._tick(ph, clk, 100, 68)
    assert out == pytest.approx(69 / 3.6, abs=1e-6)
    assert ph._ceiling_ms == pytest.approx(69 / 3.6, abs=1e-6)

  def test_never_exceeds_v_cruise_in(self, ph, monkeypatch):
    clk = self._clock(ph, monkeypatch)
    self._baseline_hold(ph, clk)
    self._sl(ph, 40, source=2, road='B')
    out = self._tick(ph, clk, 60, 68)      # cruise set below the car's speed
    assert out <= 60 / 3.6 + 1e-9

  def test_no_reanchor_when_the_car_slows_through_the_ceiling(self, ph, monkeypatch):
    """A floor that simply ends because the car has caught the descending
    ceiling must not lift the ceiling."""
    clk = self._clock(ph, monkeypatch)
    self._baseline_hold(ph, clk)
    c = ph._ceiling_ms
    self._sl(ph, 40, source=2, road='B')
    self._tick(ph, clk, 100, 45)           # car already below the ceiling
    assert ph._ceiling_ms <= c + 1e-9


class TestLeadOverrideDebounced:
  """A faster lead overrides ANY non-safety confirmed limit (guessed ones too),
  with a 0.5 s entry debounce and a 1.0 s release hold. Route 538, 2026-11-12:
  a guessed 40 pulled the car 83 -> 50 km/h behind a lead that ran 50-63.
  Spec: scratchpad plan_sl_inferred_lead.md (rev 3).
  """
  _sm = TestCeilingInOnVCruise._sm
  _clock = TestCeilingInOnVCruise._clock

  @pytest.fixture
  def bus(self, ph):
    sent = []
    ph._pub = type("P", (), {"send": staticmethod(lambda d: sent.append(dict(d)))})()
    ph._pub_ok = True
    return sent

  def _sl(self, ph, limit, source=2, safety=False, confirmed=True, road='A', cap=0):
    """cap=None omits the field (an older daemon)."""
    d = {'confirmed': confirmed, 'speedLimit': limit, 'safetyCapped': safety,
         'source': source, 'roadName': road, 'wayRef': ''}
    if cap is not None:
      d['safetyCap'] = cap
    ph._sl_data = d

  def _lead(self, kph):
    return self._sm(lead_present=True, lead_vLead=kph / 3.6)

  def _run(self, ph, clk, n, v_ego_kph, sm, v_cruise_kph=101.0):
    outs = []
    for _ in range(n):
      clk['t'] += DT
      outs.append(ph.on_v_cruise(v_cruise_kph / 3.6, v_ego_kph / 3.6, sm))
    return outs

  def _states(self, bus, n):
    return [m['state'] for m in bus[-n:]]

  # (a) / (b)
  @pytest.mark.parametrize('road', ['A', ''])
  def test_inferred_override_after_half_a_second_not_before(self, ph, bus, monkeypatch, road):
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 40, source=2, road=road)
    sm = self._lead(63)
    self._run(ph, clk, 1, 70.0, sm)
    outs = self._run(ph, clk, 8, 70.0, sm)           # t = 0.05 .. 0.40 s
    assert set(self._states(bus, 9)) == {'capped'}
    assert all(o < 101 / 3.6 for o in outs)
    out = self._run(ph, clk, 3, 70.0, sm)            # crosses 0.5 s
    assert self._states(bus, 3)[-1] == 'lead_override'
    assert out[-1] == pytest.approx(101 / 3.6)

  # (c)
  def test_inferred_slow_lead_stays_capped(self, ph, bus, monkeypatch):
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 40, source=2, road='')
    outs = self._run(ph, clk, 60, 70.0, self._lead(42))   # target 46, entry needs lead > 46*1.05 = 48.3; 42 never qualifies
    assert set(self._states(bus, 60)) == {'capped'}
    assert outs[-1] == pytest.approx(46 / 3.6, abs=0.01)

  # (d)
  def test_one_tick_spike_never_overrides(self, ph, bus, monkeypatch):
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 40, source=2, road='')
    self._run(ph, clk, 20, 70.0, self._sm())
    self._run(ph, clk, 1, 70.0, self._lead(63))
    self._run(ph, clk, 20, 70.0, self._sm())
    assert 'lead_override' not in self._states(bus, 41)
    assert 'lead_hold' not in self._states(bus, 41)

  def test_flicker_never_accumulates(self, ph, bus, monkeypatch):
    """0.4 s on, one tick off, 0.4 s on: the debounce restarts, never engages."""
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 40, source=2, road='')
    for _ in range(3):
      self._run(ph, clk, 8, 70.0, self._lead(63))
      self._run(ph, clk, 1, 70.0, self._sm())
    assert 'lead_override' not in [m['state'] for m in bus]

  # (e) / (f)
  @pytest.mark.parametrize('source,road', [(2, 'A'), (2, ''), (1, 'A')])
  def test_release_is_anchored_at_v_ego(self, ph, bus, monkeypatch, source, road):
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 60, source=source, road=road)
    self._run(ph, clk, 10, 80.0, self._sm())
    self._sl(ph, 40, source=source, road=road)        # 60 -> 40 under the override
    self._run(ph, clk, 60, 80.0, self._lead(70))
    assert bus[-1]['state'] == 'lead_override'
    # lead gone; hold 1.0 s then release
    outs = self._run(ph, clk, 40, 80.0, self._sm())
    states = self._states(bus, 40)
    rel = next(i for i, st in enumerate(states) if st not in ('lead_hold', 'lead_override'))
    assert all(st == 'lead_hold' for st in states[:rel])
    assert outs[rel] * 3.6 >= 80.0 - 0.1, 'release tick must not step below v_ego'
    flags = [m['anchored'] for m in bus[-40:]]
    assert flags[rel] is True and flags.count(True) == 1
    m = bus[-40 + rel]
    assert m['anchorTo'] == pytest.approx(80.0, abs=0.1)
    for a, b in zip(outs[rel:], outs[rel + 1:]):
      assert b - a >= -ph.CEIL_A_DOWN * DT - 1e-6, 'descent faster than CEIL_A_DOWN'

  # (g)
  def test_release_below_target_leaves_the_ceiling(self, ph, bus, monkeypatch):
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 40, source=2, road='')
    self._run(ph, clk, 60, 30.0, self._lead(60))
    assert bus[-1]['state'] == 'lead_override'
    self._run(ph, clk, 40, 30.0, self._sm())
    states = self._states(bus, 40)
    rel = next(i for i, st in enumerate(states) if st != 'lead_hold')
    m = bus[-40 + rel]
    assert m['state'] == 'capped' and m['anchored'] is False
    assert m['ceiling'] == pytest.approx(46.0, abs=0.05)

  # (h)
  def test_dropout_inside_the_hold_keeps_the_override(self, ph, bus, monkeypatch):
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 40, source=2, road='')
    self._run(ph, clk, 30, 70.0, self._lead(63))
    outs = self._run(ph, clk, 10, 70.0, self._sm())         # 0.5 s gap
    assert set(self._states(bus, 10)) == {'lead_hold'}
    assert all(o == pytest.approx(101 / 3.6) for o in outs)
    self._run(ph, clk, 1, 70.0, self._lead(63))             # lead back: still ON
    assert bus[-1]['state'] == 'lead_override'

  def test_dropout_past_the_hold_releases(self, ph, bus, monkeypatch):
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 40, source=2, road='')
    self._run(ph, clk, 30, 70.0, self._lead(63))
    self._run(ph, clk, 30, 70.0, self._sm())                # 1.5 s gap
    states = self._states(bus, 30)
    assert states[0] == 'lead_hold' and states[-1] == 'capped'
    assert states.count('lead_hold') == pytest.approx(19, abs=1)
    self._run(ph, clk, 1, 70.0, self._lead(63))             # a lone tick: debounce again
    assert bus[-1]['state'] == 'capped'

  # (i)
  def test_safety_cap_mid_override_applies_exactly(self, ph, bus, monkeypatch):
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 40, source=2, road='')
    self._run(ph, clk, 30, 70.0, self._lead(63))
    self._sl(ph, 50, source=4, safety=True, cap=50)
    out = self._run(ph, clk, 1, 70.0, self._lead(63))[0]
    assert bus[-1]['state'] == 'capped'
    assert out == pytest.approx(50 / 3.6, abs=1e-6)
    assert ph._lead_on is False

  def test_safety_cap_inside_the_hold_applies_exactly(self, ph, bus, monkeypatch):
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 40, source=2, road='')
    self._run(ph, clk, 30, 70.0, self._lead(63))
    self._run(ph, clk, 6, 70.0, self._sm())
    assert bus[-1]['state'] == 'lead_hold'
    self._sl(ph, 50, source=4, safety=True, cap=50)
    out = self._run(ph, clk, 1, 70.0, self._sm())[0]
    assert out == pytest.approx(50 / 3.6, abs=1e-6) and bus[-1]['state'] == 'capped'

  def test_no_override_resumes_after_the_cap_clears(self, ph, bus, monkeypatch):
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 40, source=2, road='')
    self._run(ph, clk, 30, 70.0, self._lead(63))
    self._sl(ph, 50, source=4, safety=True, cap=50)
    self._run(ph, clk, 3, 70.0, self._sm())
    self._sl(ph, 40, source=2, road='')                     # cap cleared, lead absent
    self._run(ph, clk, 30, 70.0, self._sm())
    assert not {'lead_override', 'lead_hold'} & set(self._states(bus, 30))
    self._run(ph, clk, 8, 70.0, self._lead(63))             # re-qualifies: debounce again
    assert 'lead_override' not in self._states(bus, 8)
    self._run(ph, clk, 4, 70.0, self._lead(63))
    assert bus[-1]['state'] == 'lead_override'

  # (j)
  def test_gas_inside_the_hold_clears_it(self, ph, bus, monkeypatch):
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 40, source=2, road='')
    self._run(ph, clk, 30, 70.0, self._lead(63))
    self._run(ph, clk, 6, 70.0, self._sm())
    out = self._run(ph, clk, 1, 70.0, self._sm(gas=True))[0]
    assert bus[-1]['state'] == 'gas' and out == pytest.approx(101 / 3.6)
    assert ph._lead_on is False
    self._run(ph, clk, 30, 70.0, self._sm())                # gas released, lead gone
    assert not {'lead_override', 'lead_hold'} & set(self._states(bus, 30))

  # (k)
  @pytest.mark.parametrize('kw', [{'confirmed': False}, {'limit': 0}])
  def test_unconfirmed_or_no_limit_clears_it(self, ph, bus, monkeypatch, kw):
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 40, source=2, road='')
    self._run(ph, clk, 30, 70.0, self._lead(63))
    assert ph._lead_on is True
    self._sl(ph, kw.get('limit', 40), source=2, road='', confirmed=kw.get('confirmed', True))
    self._run(ph, clk, 1, 70.0, self._lead(63))
    assert ph._lead_on is False and ph._lead_q_s == 0.0
    self._sl(ph, 40, source=2, road='')
    self._run(ph, clk, 5, 70.0, self._sm())
    assert 'lead_hold' not in self._states(bus, 5)

  # (l)
  def test_during_on_the_ceiling_follows_the_limit(self, ph, bus, monkeypatch):
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 40, source=2, road='')
    self._run(ph, clk, 80, 70.0, self._lead(63))
    on = [m for m in bus if m['state'] == 'lead_override']
    assert len(on) > 60
    assert all(m['anchored'] is False for m in bus)
    assert all(m['ceiling'] == pytest.approx(46.0, abs=0.05) for m in bus)

  # (m)
  def test_lead_speed_is_published(self, ph, bus, monkeypatch):
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 80, source=1)
    self._run(ph, clk, 1, 70.0, self._lead(63))
    assert bus[-1]['leadKph'] == pytest.approx(63.0, abs=0.05)
    self._run(ph, clk, 1, 70.0, self._sm())
    assert bus[-1]['leadKph'] == 0.0
    self._run(ph, clk, 1, 70.0, self._sm(lead_present=False, lead_vLead=50 / 3.6))
    assert bus[-1]['leadKph'] == 0.0

  # (n)
  @pytest.mark.parametrize('cap', [60, 55])
  def test_masked_safety_cap_bounds_the_override(self, ph, bus, monkeypatch, cap):
    """Limit 40 (guess), a curve/reactive cap ABOVE it that the daemon does not
    surface as safetyCapped: the override must not lift it."""
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 40, source=2, road='', cap=cap)
    outs = self._run(ph, clk, 30, 70.0, self._lead(63), v_cruise_kph=101.0)
    assert bus[-1]['state'] == 'lead_override'
    assert all(o <= cap / 3.6 + 1e-9 for o in outs[10:])
    assert outs[-1] == pytest.approx(cap / 3.6, abs=1e-6)
    outs = self._run(ph, clk, 10, 70.0, self._sm(), v_cruise_kph=101.0)   # hold
    assert bus[-1]['state'] == 'lead_hold'
    assert all(o == pytest.approx(cap / 3.6, abs=1e-6) for o in outs)

  def test_safety_cap_above_v_cruise_is_not_binding(self, ph, bus, monkeypatch):
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 40, source=2, road='', cap=120)
    outs = self._run(ph, clk, 30, 70.0, self._lead(63), v_cruise_kph=101.0)
    assert outs[-1] == pytest.approx(101 / 3.6)

  # --- 2026-11-12 replay fixes: hysteresis on the enforced target, gas floor ---

  def test_hysteresis_lead_between_the_margins_never_turns_on(self, ph, bus, monkeypatch):
    """Route 538 seg 25-26: guessed 80 (target 88), car at 88, lead hovering
    86-92. Entry needs lead > target x 1.05; a lead oscillating between
    target x 1.02 and x 1.04 never qualifies."""
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 80, source=2, road='')
    for i in range(80):
      self._run(ph, clk, 1, 88.0, self._lead(88 * (1.02 if i % 2 else 1.04)))
    assert not {'lead_override', 'lead_hold'} & {m['state'] for m in bus}

  def test_hysteresis_once_on_a_lead_at_the_target_keeps_it(self, ph, bus, monkeypatch):
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 80, source=2, road='')
    self._run(ph, clk, 15, 88.0, self._lead(88 * 1.10))
    assert bus[-1]['state'] == 'lead_override'
    self._run(ph, clk, 60, 88.0, self._lead(88 * 1.01))     # x1.01 > x1.00: stays
    assert set(self._states(bus, 60)) == {'lead_override'}
    self._run(ph, clk, 5, 88.0, self._lead(88 * 0.99))      # below target: hold
    assert set(self._states(bus, 5)) == {'lead_hold'}

  def test_confirmed_limit_uses_the_target_trigger_too(self, ph, bus, monkeypatch):
    """Limit 80 source 1: target 88. A lead at 90 (> limit x 1.10 = 88, but
    < target x 1.05 = 92.4) no longer enters."""
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 80, source=1)
    self._run(ph, clk, 30, 80.0, self._lead(90))
    assert 'lead_override' not in {m['state'] for m in bus}
    self._run(ph, clk, 15, 80.0, self._lead(93))
    assert bus[-1]['state'] == 'lead_override'

  def test_gas_floor_held_after_release_does_not_block_the_override(self, ph, bus, monkeypatch):
    """Route 538 seg 19 +24.5-31: after a gas press the gas floor held for 6 s
    and kept the override off while the lead ran 63-67 against a target of 46."""
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 40, source=2, road='')
    self._run(ph, clk, 5, 61.0, self._sm(gas=True))
    assert ph._gas_floor_ms is not None
    self._run(ph, clk, 8, 61.0, self._lead(65))
    assert 'lead_override' not in self._states(bus, 8)
    self._run(ph, clk, 4, 61.0, self._lead(65))
    assert ph._gas_floor_ms is not None
    assert bus[-1]['state'] == 'lead_override'

  def test_gas_press_still_clears_the_override(self, ph, bus, monkeypatch):
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 40, source=2, road='')
    self._run(ph, clk, 15, 61.0, self._lead(65))
    assert ph._lead_on
    self._run(ph, clk, 1, 61.0, self._sm(gas=True, lead_present=True, lead_vLead=65 / 3.6))
    assert bus[-1]['state'] == 'gas' and ph._lead_on is False

  def test_release_with_the_gas_floor_still_set(self, ph, bus, monkeypatch):
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 40, source=2, road='')
    self._run(ph, clk, 5, 61.0, self._sm(gas=True))
    self._run(ph, clk, 40, 61.0, self._lead(65))
    assert bus[-1]['state'] == 'lead_override' and ph._gas_floor_ms is not None
    outs = self._run(ph, clk, 40, 61.0, self._sm())
    states = self._states(bus, 40)
    rel = next(i for i, st in enumerate(states) if st != 'lead_hold')
    assert outs[rel] * 3.6 >= 61.0 - 0.1
    flags = [m['anchored'] for m in bus[-40:]]
    assert flags[rel] is True and flags.count(True) == 1


  # --- gate 2 fixes: floor bound, release anchor bound, fail closed ---

  def test_override_never_outputs_below_the_held_floor(self, ph, bus, monkeypatch):
    """Guess 40, safetyCap 50 (masked), driver gassed to 60 then released: the
    gas floor holds 60. The override must not drop the output to 50."""
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 40, source=2, road='', cap=50)
    self._run(ph, clk, 3, 60.0, self._sm(gas=True))
    assert ph._gas_floor_ms == pytest.approx(60 / 3.6)
    outs = self._run(ph, clk, 40, 60.0, self._lead(63))
    assert 'lead_override' in self._states(bus, 40)
    assert all(o * 3.6 >= 60.0 - 0.1 for o in outs)
    outs = self._run(ph, clk, 10, 60.0, self._sm())          # hold
    assert 'lead_hold' in self._states(bus, 10)
    assert all(o * 3.6 >= 60.0 - 0.1 for o in outs)

  def test_release_anchor_is_bounded_by_the_override_output(self, ph, bus, monkeypatch):
    """Override bounded at 50, car still at 58, lead gone: on hold expiry the
    output must not jump up to 58."""
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 40, source=2, road='', cap=50)
    outs_on = self._run(ph, clk, 40, 58.0, self._lead(63))
    assert outs_on[-1] == pytest.approx(50 / 3.6, abs=1e-6)
    outs = self._run(ph, clk, 40, 58.0, self._sm())
    prev = outs_on[-1]
    for o in outs:
      assert o <= prev + 1e-6, 'output rose'
      assert o * 3.6 <= 50.0 + 0.1
      prev = o

  def test_missing_safety_cap_field_fails_closed(self, ph, bus, monkeypatch):
    """An older daemon never publishes safetyCap: no override, no hold."""
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 40, source=2, road='', cap=None)            # no 'safetyCap' key
    outs = self._run(ph, clk, 40, 70.0, self._lead(63))
    assert not {'lead_override', 'lead_hold'} & set(self._states(bus, 40))
    assert all(o < 101 / 3.6 for o in outs)
    # also clears an ON override if the field disappears
    self._sl(ph, 40, source=2, road='', cap=0)
    self._run(ph, clk, 15, 70.0, self._lead(63))
    assert ph._lead_on
    self._sl(ph, 40, source=2, road='', cap=None)
    self._run(ph, clk, 1, 70.0, self._lead(63))
    assert ph._lead_on is False and bus[-1]['state'] == 'capped'

  def test_safety_cap_is_published_in_telemetry(self, ph, bus, monkeypatch):
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 40, source=2, road='', cap=60)
    self._run(ph, clk, 1, 50.0, self._sm())
    assert bus[-1]['safetyCap'] == 60
    self._sl(ph, 40, source=2, road='', cap=None)
    self._run(ph, clk, 1, 50.0, self._sm())
    assert bus[-1]['safetyCap'] == 0

  def test_baseline_floor_does_not_carry_the_lead_through_a_tightening_cap(self, ph, bus, monkeypatch):
    """Director's probe: the baseline floor is min(baseline, v_ego) and rises with
    the car, so only the HELD GAS floor may bound the override from below."""
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 80, source=2, road='A', cap=80)
    self._run(ph, clk, 5, 60.0, self._sm())
    self._sl(ph, 40, source=2, road='A', cap=80)           # baseline stays 88
    self._run(ph, clk, 15, 60.0, self._lead(70))
    assert bus[-1]['state'] == 'lead_override'
    self._run(ph, clk, 20, 70.0, self._lead(70))           # car follows the lead to 70
    assert bus[-1]['baselineFloor'] == pytest.approx(88.0, abs=0.1)   # running max; the floor is min(88, v_ego)
    self._sl(ph, 40, source=2, road='A', cap=60)           # cap tightens, still masked
    out = self._run(ph, clk, 1, 70.0, self._lead(70))[0]
    assert bus[-1]['state'] == 'lead_override'
    assert out == pytest.approx(60 / 3.6, abs=1e-6)
    outs = self._run(ph, clk, 40, 70.0, self._lead(70))
    assert all(o * 3.6 <= 60.1 for o in outs)
