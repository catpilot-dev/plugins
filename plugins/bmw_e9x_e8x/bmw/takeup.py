"""Direct Tracking support: servo-column contact learning.

Design: docs/superpowers/specs/2026-10-18-bmw-direct-tracking-design.md.

The servo drives the column through ~12-13 deg of lash (route 4e5, re-printed
gears: servo-column gap ~ +77.6..77.8 deg under right torque, ~65 deg under
left torque; contacts stable within +/-1 deg inside a drive).  ContactLearner
estimates the two contact gaps online from samples taken while a firm torque
presses the gear against one flank and the wheel is stationary.

Sign convention: controller torque + = right.  c_right is the gap under right
torque, c_left the gap under left torque.
"""
import statistics

TAKEUP_NM = 0.35
CONTACT_TOL_DEG = 1.5
GUARD_PERSIST_S = 0.15
ANGLE_QUANTUM_DEG = 0.04395
SZL_CONFIRM_QUANTA = 3
TAKEUP_TIMEOUT_S = 0.4
LEARN_MIN_SAMPLES = 200
LEARN_TAU_S = 60.0
LEARN_TQ_NM = 0.45
LEARN_RATE_DPS = 1.0
CONTACT_WIDTH_RANGE = (3.0, 25.0)  # deg


def _width_ok(c_right, c_left):
  return CONTACT_WIDTH_RANGE[0] <= abs(c_right - c_left) <= CONTACT_WIDTH_RANGE[1]


def parse_contacts(text):
  """Parse "c_right,c_left"; None if malformed or width out of range."""
  try:
    a, b = text.strip().split(',')
    c = (float(a), float(b))
  except (ValueError, AttributeError):
    return None
  if not all(x == x and abs(x) != float('inf') for x in c) or not _width_ok(*c):
    return None
  return c


def format_contacts(c):
  return f"{c[0]:.2f},{c[1]:.2f}"


class ContactLearner:
  """Median of the first LEARN_MIN_SAMPLES per side, then EMA (tau LEARN_TAU_S)."""

  def __init__(self, seed=None, dt=0.01):
    self._seed = seed
    self._alpha = dt / LEARN_TAU_S
    self._buf = {0: [], 1: []}   # 0 = right, 1 = left
    self._val = {0: None, 1: None}

  def observe(self, gap_deg, torque_nm, steer_rate_dps, ok):
    if not ok or abs(torque_nm) <= LEARN_TQ_NM or abs(steer_rate_dps) >= LEARN_RATE_DPS:
      return
    s = 0 if torque_nm > 0 else 1
    if self._val[s] is None:
      self._buf[s].append(gap_deg)
      if len(self._buf[s]) >= LEARN_MIN_SAMPLES:
        self._val[s] = statistics.median(self._buf[s])
        self._buf[s] = []
    else:
      self._val[s] += self._alpha * (gap_deg - self._val[s])

  def contacts(self):
    vals = []
    for s in (0, 1):
      v = self._val[s]
      if v is None and self._seed is not None:
        v = self._seed[s]
      if v is None:
        return None
      vals.append(v)
    return tuple(vals) if _width_ok(*vals) else None

  def fresh(self):
    return self._val[0] is not None and self._val[1] is not None
