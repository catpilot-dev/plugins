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
                   'roadName': road, 'wayRef': ''}

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
    self._clock(ph, monkeypatch)
    self._sl(ph, 80, source=1)
    ph.on_v_cruise(100 / 3.6, 25.0, self._sm(lead_present=True, lead_vLead=100 / 3.6))
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
