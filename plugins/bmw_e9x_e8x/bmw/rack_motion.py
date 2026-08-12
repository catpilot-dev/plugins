"""Rack motion observation for the BMW lateral controller.

Replaces the job FRICTION used to do — PREDICTING whether a commanded torque
can move the steering rack — with OBSERVING whether it actually is moving.

Why: breakaway torque is not a constant. Measured on route 3f2 (51 segments,
`action=='ramp'` ticks only) the knee moves from under 0.25 Nm to beyond
3.9 Nm with lateral acceleration and speed, plus unobservables (tyre pressure,
road surface, rack lubrication). FRICTION = 0.05 frac (0.6 Nm) understates the
measured 2.0-2.75 Nm knee by roughly 4x, and HOLD_BAND is derived from it.

steeringAngleDeg is ground truth that the rack moved, independent of every one
of those unobservables. It carries a constant ~-1.58 deg physical alignment
offset which cancels exactly under differencing, so this module works only in
deltas and never reads absolute angle.

Loaded as plugins.bmw_e9x_e8x.bmw.rack_motion. Pure computation, no I/O.
"""
from collections import deque

# steeringAngleDeg quantisation, measured (169 distinct values over a segment).
ANGLE_LSB_DEG = 0.0879

# Rate window. Resolution = ANGLE_LSB_DEG / WINDOW_S = 0.55 deg/s.
# Shorter windows are noisier: a single-sample difference at 100 Hz is
# 8.8 deg/s of pure quantisation noise.
WINDOW_S = 0.16

# "Stuck" criterion used throughout the plant characterisation.
MOTION_THRESHOLD_DEG_S = 2.0

# Controller torque fraction is NEGATIVE for LEFT; steeringAngleDeg is
# POSITIVE for LEFT. Verified against GPS bearing, DSC yaw rate and lateral
# acceleration on route 3f2. This is the single place that conversion lives.
TORQUE_TO_ANGLE_SIGN = -1.0


class RackMotion:
  """Windowed least-squares slope of steering angle. Offset-immune."""

  def __init__(self, window_s=WINDOW_S):
    self.window_s = float(window_s)
    self._t = deque()
    self._a = deque()

  def reset(self):
    self._t.clear()
    self._a.clear()

  def update(self, t, angle_deg):
    t = float(t)
    # Non-monotonic time (log replay seek, engagement restart) invalidates the
    # window rather than producing a bogus slope.
    if self._t and t <= self._t[-1]:
      self.reset()
    self._t.append(t)
    self._a.append(float(angle_deg))
    while len(self._t) > 1 and (self._t[-1] - self._t[0]) > self.window_s:
      self._t.popleft()
      self._a.popleft()

  @property
  def rate_deg_s(self):
    n = len(self._t)
    if n < 3:
      return float('nan')
    span = self._t[-1] - self._t[0]
    if span < 0.5 * self.window_s:
      return float('nan')
    t_bar = sum(self._t) / n
    a_bar = sum(self._a) / n
    num = 0.0
    den = 0.0
    for ti, ai in zip(self._t, self._a):
      dt = ti - t_bar
      num += dt * (ai - a_bar)
      den += dt * dt
    if den <= 0.0:
      return float('nan')
    return num / den

  def is_moving(self, threshold_deg_s=MOTION_THRESHOLD_DEG_S):
    r = self.rate_deg_s
    return r == r and abs(r) >= threshold_deg_s

  def is_moving_with_torque(self, torque_frac, threshold_deg_s=MOTION_THRESHOLD_DEG_S):
    """True when the rack is moving in the direction the torque commands.

    Signed on purpose: on rough pavement or camber the wheel jiggles without
    the rack having broken free the way we asked.
    """
    r = self.rate_deg_s
    if r != r or torque_frac == 0.0:
      return False
    expected_sign = TORQUE_TO_ANGLE_SIGN * (1.0 if torque_frac > 0.0 else -1.0)
    return abs(r) >= threshold_deg_s and (r * expected_sign) > 0.0


# Seed = measured knee midpoint. Route 3f2 ramp ticks: stuck fraction first
# falls below 50% at 2.50-2.75 Nm (strict action-freshness gate) or
# 2.00-2.25 Nm (permissive gate). 2.4 Nm / STEER_MAX 12 = 0.20 frac.
# The old FRICTION = 0.05 (0.6 Nm) was roughly 4x too low.
BREAKAWAY_SEED_FRAC = 0.20

# Clamps. MIN keeps a pathological low observation from disabling the gates
# that consume this; MAX keeps a stuck-rack episode from ratcheting the
# estimate into the authority cap.
BREAKAWAY_MIN_FRAC = 0.05
BREAKAWAY_MAX_FRAC = 0.40

# EMA weight per observed breakaway. At 0.10 the estimate reaches ~90% of a
# step change in ~22 observations; route 3f2 offers roughly 40 qualifying
# transitions per hour of driving, so this tracks conditions across a drive
# without chasing a single anomalous release.
BREAKAWAY_ALPHA = 0.10

# Static friction exceeds kinetic: at the instant of release the applied
# torque already exceeds what sustains motion. Measured on route 3f2 —
# breakaway 2.5-2.9 Nm, sustained unwinding motion at 1.25-1.5 Nm.
SUSTAIN_RATIO = 0.5


class BreakawayEstimator:
  """Online estimate of the rack's breakaway torque, in torque fraction.

  Records the applied torque at each observed stationary -> moving transition
  and low-passes it. Never needs to know tyre pressure, surface or temperature:
  it re-measures the threshold on every push under whatever conditions apply.
  """

  def __init__(self, seed_frac=BREAKAWAY_SEED_FRAC):
    self._seed = float(seed_frac)
    self.breakaway_frac = float(seed_frac)
    self.observations = 0
    self._was_moving = False

  def reset(self):
    self.breakaway_frac = self._seed
    self.observations = 0
    self._was_moving = False

  def update(self, torque_frac, moving_with_torque):
    moving = bool(moving_with_torque)
    if moving and not self._was_moving and torque_frac != 0.0:
      obs = min(max(abs(float(torque_frac)), BREAKAWAY_MIN_FRAC), BREAKAWAY_MAX_FRAC)
      self.breakaway_frac += BREAKAWAY_ALPHA * (obs - self.breakaway_frac)
      self.observations += 1
    self._was_moving = moving

  @property
  def sustain_frac(self):
    return SUSTAIN_RATIO * self.breakaway_frac
