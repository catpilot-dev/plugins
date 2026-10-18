import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from bmw.takeup import ContactLearner, parse_contacts, format_contacts, LEARN_MIN_SAMPLES


def feed(L, gap, tq, n, rate=0.0, ok=True):
  for _ in range(n):
    L.observe(gap, tq, rate, ok)


def test_learns_both_sides_by_median():
  L = ContactLearner()
  feed(L, 77.6, +0.8, LEARN_MIN_SAMPLES)
  assert L.contacts() is None            # one side only
  feed(L, 65.0, -0.8, LEARN_MIN_SAMPLES)
  assert L.contacts() == pytest.approx((77.6, 65.0))
  assert L.fresh()


def test_ignores_small_torque_moving_wheel_and_not_ok():
  L = ContactLearner()
  feed(L, 10.0, +0.3, 500)               # |T| < LEARN_TQ_NM
  feed(L, 10.0, +0.8, 500, rate=5.0)     # wheel moving
  feed(L, 10.0, +0.8, 500, ok=False)
  feed(L, 10.0, -0.8, 500, ok=False)
  assert L.contacts() is None


def test_seed_used_until_fresh_and_then_replaced():
  L = ContactLearner(seed=(77.0, 64.0))
  assert L.contacts() == (77.0, 64.0) and not L.fresh()
  feed(L, 78.0, +0.8, LEARN_MIN_SAMPLES); feed(L, 65.0, -0.8, LEARN_MIN_SAMPLES)
  assert L.contacts() == pytest.approx((78.0, 65.0))


def test_dead_gap_signal_gives_no_contacts():      # Review Focus 2
  L = ContactLearner()
  feed(L, 0.0, +0.8, 500); feed(L, 0.0, -0.8, 500)
  assert L.contacts() is None                       # width 0 < CONTACT_WIDTH_RANGE[0]


def test_parse_format_roundtrip_and_rejects():
  assert parse_contacts(format_contacts((77.6, 65.0))) == pytest.approx((77.6, 65.0))
  assert parse_contacts('') is None and parse_contacts('x,y') is None
  assert parse_contacts('70.0,69.0') is None        # width 1 deg


from bmw.takeup import SignGuard, FreePlayTakeUp, TAKEUP_NM, TAKEUP_TIMEOUT_S


def test_guard_first_push_commits_side():
  g = SignGuard(); g.observe(+0.002)
  assert g.allow(+0.1) == 0.1 and g.side == +1


def test_guard_suppresses_short_reversal_then_allows_persisted():
  g = SignGuard(); g.observe(+0.002); g.allow(+0.1)
  g.observe(-0.002); g.observe(-0.002)                  # 0.10 s < 0.15 s
  assert g.allow(-0.1) == 0.0 and g.suppressed == 1 and g.side == +1
  g.observe(-0.002)                                     # 3 ticks = 0.15 s
  assert g.allow(-0.1) == -0.1 and g.side == -1


def test_guard_flicker_never_reverses():
  g = SignGuard(); g.observe(+0.002); g.allow(+0.1)
  for k in range(20):
    g.observe(-0.002 if k % 2 else +0.002)
    assert g.allow(-0.1) == 0.0
  assert g.side == +1


def test_guard_value_equality():
  """Value semantics: controller state snapshots compare guards by content
  (TestOverrideObservers.test_no_behavior_change diffs every state key)."""
  a, b = SignGuard(), SignGuard()
  assert a == b
  a.observe(+0.002); a.allow(+0.1)
  assert a != b
  b.observe(+0.002); b.allow(+0.1)
  assert a == b
  assert a != object()
  with pytest.raises(TypeError):
    hash(a)


def _tu():
  L = ContactLearner(seed=(77.6, 65.0))
  return FreePlayTakeUp(L, steer_max_nm=12.0)


def test_takeup_fires_at_zero_crossing_toward_unseated_side():
  t = _tu()
  out = t.step(torque_frac=0.0, ramp_step=+0.002, target_frac=0.04, gap_deg=65.0,
               angle_deg=0.0, blocked=False)
  assert out == pytest.approx(TAKEUP_NM / 12.0) and t.state == 'taking_up' and t.fires == 1


def test_no_takeup_when_already_seated_or_still_shedding():
  t = _tu()
  assert t.step(0.0, +0.002, 0.04, 77.6, 0.0, False) is None          # seated right already
  assert t.step(+0.05, -0.002, -0.04, 77.6, 0.0, False) is None        # shedding old side, not at zero yet


def test_small_reversal_target_does_not_fire():
  """Ruling R9: take-up only for a real reversal — a target below TAKEUP_NM
  (here 0.012 Nm) would be overshot by the take-up step itself."""
  t = _tu()
  assert t.step(0.0, +0.0001, 0.012 / 12.0, 65.0, 0.0, False) is None
  assert t.fires == 0 and t.state == 'left'
  assert t.step(0.0, +0.002, TAKEUP_NM / 12.0, 65.0, 0.0, False) is not None   # at the bound fires
  assert t.fires == 1


def test_target_off_the_takeup_side_never_fires():
  t = _tu()
  assert t.step(0.0, +0.002, 0.0, 65.0, 0.0, False) is None
  assert t.step(0.0, +0.002, -0.04, 65.0, 0.0, False) is None
  assert t.fires == 0


@pytest.mark.parametrize('target', [-0.04, 0.0])
def test_takeup_ends_when_target_leaves_its_side(target):
  """Ruling R9: a reversal to the other side (or to 0) mid-take-up ends it
  — hand over as on contact, no timeout counted."""
  t = _tu(); t.step(0.0, +0.002, 0.04, 65.0, 0.0, False)
  assert t.state == 'taking_up'
  assert t.step(TAKEUP_NM / 12, +0.002, target, 70.0, 0.0, False) is None
  assert t.state == 'unknown' and t.timeouts == 0
  assert t.last_ms == pytest.approx(10.0)


def test_handover_on_gap_contact():
  t = _tu(); t.step(0.0, +0.002, 0.04, 65.0, 0.0, False)
  assert t.step(TAKEUP_NM / 12, +0.002, 0.04, 70.0, 0.0, False) == pytest.approx(TAKEUP_NM / 12)
  assert t.step(TAKEUP_NM / 12, +0.002, 0.04, 77.0, 0.0, False) is None and t.state == 'right'


def test_handover_on_szl_confirmation():
  t = _tu(); t.step(0.0, +0.002, 0.04, 65.0, 10.0, False)
  assert t.step(TAKEUP_NM / 12, +0.002, 0.04, 70.0, 10.0 - 0.14, False) is None   # right torque -> SZL negative, 3 quanta


def test_timeout_hands_over():
  t = _tu(); t.step(0.0, +0.002, 0.04, 65.0, 0.0, False)
  for _ in range(round(TAKEUP_TIMEOUT_S / 0.01) - 1):
    assert t.step(TAKEUP_NM / 12, +0.002, 0.04, 66.0, 0.0, False) is not None
  assert t.step(TAKEUP_NM / 12, +0.002, 0.04, 66.0, 0.0, False) is None and t.timeouts == 1


def test_blocked_or_unknown_contacts_never_fire():
  assert _tu().step(0.0, +0.002, 0.04, 65.0, 0.0, blocked=True) is None
  t = FreePlayTakeUp(ContactLearner(), steer_max_nm=12.0)
  assert t.step(0.0, +0.002, 0.04, 65.0, 0.0, False) is None


def test_learner_and_takeup_value_equality():
  """Value semantics, as SignGuard: the controller's twin-lockstep test
  (TestOverrideObservers.test_no_behavior_change) diffs every state key."""
  a, b = ContactLearner(seed=(77.6, 65.0)), ContactLearner(seed=(77.6, 65.0))
  assert a == b
  a.observe(77.6, +0.8, 0.0, True)
  assert a != b
  b.observe(77.6, +0.8, 0.0, True)
  assert a == b and a != object()
  ta, tb = FreePlayTakeUp(a, steer_max_nm=12.0), FreePlayTakeUp(b, steer_max_nm=12.0)
  assert ta == tb
  ta.step(0.0, +0.002, 0.04, 65.0, 0.0, False)
  assert ta != tb
  tb.step(0.0, +0.002, 0.04, 65.0, 0.0, False)
  assert ta == tb and ta != object()
  for x in (a, ta):
    with pytest.raises(TypeError):
      hash(x)
