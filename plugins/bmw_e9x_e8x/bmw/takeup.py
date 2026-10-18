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

  # Value equality, as SignGuard (twin-lockstep test diffs every state key).
  __hash__ = None

  def __eq__(self, other):
    if not isinstance(other, ContactLearner):
      return NotImplemented
    return vars(self) == vars(other)


def _sign(x):
  return (x > 0) - (x < 0)


class SignGuard:
  """Commits to a push side; an opposite-side target passes only once the
  desired-error sign has been opposite for persist_s (call observe at 20 Hz)."""

  def __init__(self, persist_s=GUARD_PERSIST_S, dt=0.05):
    self._need = round(persist_s / dt)
    self.side = 0
    self.suppressed = 0
    self._opp = 0

  def observe(self, delta_err):
    s = _sign(delta_err)
    if self.side != 0 and s == -self.side:
      self._opp += 1
    else:
      self._opp = 0

  def allow(self, target_frac):
    s = _sign(target_frac)
    if s == 0:
      return target_frac
    if self.side == 0:
      self.side = s
      return target_frac
    if s == self.side:
      return target_frac
    if self._opp >= self._need:
      self.side = s
      self._opp = 0
      return target_frac
    self.suppressed += 1
    return 0.0

  def reset(self):
    self.side = 0
    self._opp = 0

  # Value equality: controller state snapshots compare guards by content
  # (latcontroller's twin-lockstep test diffs every state key).
  __hash__ = None

  def __eq__(self, other):
    if not isinstance(other, SignGuard):
      return NotImplemented
    return vars(self) == vars(other)


class FreePlayTakeUp:
  """Fires a small torque step across the servo-column lash when the commanded
  torque crosses zero toward a side whose flank is not seated."""

  def __init__(self, learner, steer_max_nm, dt=0.01):
    self._learner = learner
    self._steer_max = steer_max_nm
    self._timeout_ticks = round(TAKEUP_TIMEOUT_S / dt)
    self.state = 'unknown'
    self.fires = 0
    self.timeouts = 0
    self.last_ms = 0.0
    self._dt = dt
    self._s = 0
    self._angle0 = 0.0
    self._ticks = 0

  def seat(self, gap_deg):
    c = self._learner.contacts()
    if c is None:
      return 0
    if abs(gap_deg - c[0]) <= CONTACT_TOL_DEG:
      return +1
    if abs(gap_deg - c[1]) <= CONTACT_TOL_DEG:
      return -1
    return 0

  def _seat_state(self, gap_deg):
    return {+1: 'right', -1: 'left', 0: 'unknown'}[self.seat(gap_deg)]

  def step(self, torque_frac, ramp_step, gap_deg, angle_deg, blocked):
    c = self._learner.contacts()
    if self.state == 'taking_up':
      s = self._s
      self._ticks += 1
      target = None if c is None else (c[0] if s > 0 else c[1])
      done = False
      if target is not None and abs(gap_deg - target) < CONTACT_TOL_DEG:
        done = True
      elif (angle_deg - self._angle0) * -s >= SZL_CONFIRM_QUANTA * ANGLE_QUANTUM_DEG:
        done = True
      elif self._ticks >= self._timeout_ticks:
        self.timeouts += 1
        done = True
      if not done:
        return s * TAKEUP_NM / self._steer_max
      self.last_ms = self._ticks * self._dt * 1000.0
      self.state = self._seat_state(gap_deg)
      return None

    self.state = self._seat_state(gap_deg)
    s = _sign(ramp_step)
    if (c is not None and not blocked and s != 0
        and torque_frac * s <= 0 and (torque_frac + ramp_step) * s > 0
        and self.seat(gap_deg) != s):
      self._s = s
      self._angle0 = angle_deg
      self._ticks = 0
      self.fires += 1
      self.state = 'taking_up'
      return s * TAKEUP_NM / self._steer_max
    return None

  def reset(self):
    self.state = 'unknown'
    self._s = 0
    self._ticks = 0

  # Value equality, as SignGuard; compares the learner by value too.
  __hash__ = None

  def __eq__(self, other):
    if not isinstance(other, FreePlayTakeUp):
      return NotImplemented
    return vars(self) == vars(other)
