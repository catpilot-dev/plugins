"""Tests for planner_hook — the jerk-limited allowed-speed ceiling.

Spec: docs/superpowers/specs/2026-09-10-speedlimitd-jerk-limited-ceiling-design.md
"""
import os
import sys
import importlib
import pytest
from unittest.mock import MagicMock

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

  def test_ascent_is_brisker_than_descent(self, ph):
    """Braking jerk is what annoys; acceleration is fine. A_UP > A_DOWN."""
    down = _trace(ph, 88.0, 46.0)[-1][0]
    up = _trace(ph, 46.0, 88.0)[-1][0]
    assert up < down

  def test_ascent_respects_its_own_limits(self, ph):
    tr = _trace(ph, 46.0, 88.0)
    for (_, c0, r0), (_, c1, r1) in zip(tr, tr[1:]):
      assert abs(r1 - r0) <= ph.CEIL_J_UP * DT + 1e-9
      assert c1 >= c0 - 1e-9
    assert max(r for _, _, r in tr) <= ph.CEIL_A_UP + 1e-9
    assert tr[-1][1] == pytest.approx(88.0 / 3.6, abs=1e-9)

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

  def test_reversal_mid_descent_is_continuous(self, ph):
    """The limit rises while the ceiling is still falling. The rate must pass
    through zero under the jerk limit, not flip sign."""
    ceiling, rate = 88.0 / 3.6, 0.0
    for _ in range(40):
      ceiling, rate = ph._advance_ceiling(ceiling, rate, 46.0 / 3.6, DT)
    assert rate < 0.0, 'precondition: still descending'
    prev_rate = rate
    for _ in range(400):
      ceiling, rate = ph._advance_ceiling(ceiling, rate, 100.0 / 3.6, DT)
      assert abs(rate - prev_rate) <= ph.CEIL_J_UP * DT + 1e-9
      prev_rate = rate
    assert ceiling == pytest.approx(100.0 / 3.6, abs=1e-9)

  def test_already_at_target_is_a_no_op(self, ph):
    target = 46.0 / 3.6
    assert ph._advance_ceiling(target, 0.0, target, DT) == (target, 0.0)

  def test_zero_dt_does_not_move_the_ceiling(self, ph):
    c, r = ph._advance_ceiling(88.0 / 3.6, 0.0, 46.0 / 3.6, 0.0)
    assert c == pytest.approx(88.0 / 3.6)
    assert r == 0.0


class TestCeilingInOnVCruise:
  """The ceiling as on_v_cruise actually drives it, with a fake clock."""

  def _sm(self, gas=False, lead_status=False, lead_vLead=0.0):
    cs = MagicMock()
    cs.gasPressed = gas
    lead = MagicMock()
    lead.status = lead_status
    lead.vLead = lead_vLead
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

  def test_ramp_resumes_cleanly_after_a_safety_cap_releases(self, ph, monkeypatch):
    clk = self._clock(ph, monkeypatch)
    self._sl(ph, 40, source=4, safety=True)
    ph.on_v_cruise(100 / 3.6, 25.0, self._sm())
    self._sl(ph, 80, source=1)
    clk['t'] += DT
    out = ph.on_v_cruise(100 / 3.6, 25.0, self._sm())
    assert out < 88 / 3.6, 'release teleported instead of ramping up'
    assert out > 40 / 3.6

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
