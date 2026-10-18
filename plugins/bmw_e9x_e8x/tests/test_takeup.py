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
