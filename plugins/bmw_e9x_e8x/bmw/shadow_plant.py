"""Shadow plant-gain + friction estimator for BMW E9x/E8x.

Collects (torque, v, measured_curvature) samples in speed-binned buckets and
periodically fits the 2-parameter model:
    measured_curvature = -K · torque / v²  −  b · torque

Friction is estimated from the perpendicular spread of fit residuals
converted to torque space:
    friction = std(residual / plant_gain(v)) × 1.5

On validation (stability + R² quality + bucket coverage), promotes the fit
to the live controller, then continues adapting with decay.

Speed bins: 30 km/h to 120 km/h in 5 km/h steps = 18 bins.
"""
from collections import deque
import numpy as np


MIN_VEGO_MS = 30.0 / 3.6           # 8.33 m/s (30 km/h minimum engage speed)
MAX_VEGO_MS = 120.0 / 3.6          # 33.33 m/s (120 km/h upper bin edge)
BIN_WIDTH_MS = 5.0 / 3.6           # 1.389 m/s (5 km/h per bin)
N_BINS = int(round((MAX_VEGO_MS - MIN_VEGO_MS) / BIN_WIDTH_MS))  # 18

BUCKET_MAX = 100                   # FIFO samples per bin
BUCKET_MIN = 30                    # min samples per bin to count as "covered"
COVERAGE_MIN_BINS = 6              # must have this many covered bins to fit
REFIT_EVERY = 100                  # new samples between refits
MIN_R2 = 0.40                      # fit quality threshold
STABLE_REL_DELTA = 0.10            # |K_new − K_last| / |K_last| < this → stable
STABLE_CYCLES = 3                  # consecutive stable refits before first promotion
DECAY = 0.3                        # post-promotion: new_live = 0.7·old + 0.3·fit
MIN_ABS_TORQUE = 0.02              # samples require meaningful torque
FRICTION_FACTOR = 1.5              # std × this = friction estimate (matches stock torqued)
# Physical breakaway for BMW hydraulic rack is ~0.5-1.5 Nm (0.04-0.13 torque
# fraction). Stock's nominal 0.15 is likely inflated by the LAF cancellation
# bug in get_friction; true physical friction is probably 0.06-0.13.
FRICTION_MIN = 0.02                # 0.24 Nm lower bound
FRICTION_MAX = 0.15                # 1.8 Nm upper bound (3.6 Nm was unphysical)


class ShadowPlantEstimator:
  def __init__(self, k_init, b_init, friction_init):
    self.buckets = [deque(maxlen=BUCKET_MAX) for _ in range(N_BINS)]
    # live values (used by controller)
    self.live_k = float(k_init)
    self.live_b = float(b_init)
    self.live_friction = float(friction_init)
    # last shadow fit (for stability checks)
    self.shadow_k = float(k_init)
    self.shadow_b = float(b_init)
    self.shadow_friction = float(friction_init)
    self.shadow_r2 = 0.0
    self.stable_count = 0
    self.validated = False
    self.sample_count = 0

  def add_sample(self, v, torque, measured_curv):
    """Return True if sample accepted."""
    if v < MIN_VEGO_MS or v >= MAX_VEGO_MS: return False
    if abs(torque) < MIN_ABS_TORQUE: return False
    bin_idx = int((v - MIN_VEGO_MS) / BIN_WIDTH_MS)
    if bin_idx < 0 or bin_idx >= N_BINS: return False
    # Store v too so friction can be estimated in torque space per-sample.
    # features: x1 = -torque/v², x2 = -torque,  y = measured_curv,  v kept separately
    self.buckets[bin_idx].append((-torque / (v * v), -torque, measured_curv, v))
    self.sample_count += 1
    if self.sample_count % REFIT_EVERY == 0:
      self._maybe_refit()
    return True

  def _maybe_refit(self):
    covered = sum(1 for b in self.buckets if len(b) >= BUCKET_MIN)
    if covered < COVERAGE_MIN_BINS:
      return

    samples = [s for b in self.buckets for s in b]
    arr = np.array(samples)
    X = arr[:, :2]
    y = arr[:, 2]
    v_per = arr[:, 3]
    try:
      coef, _, _, _ = np.linalg.lstsq(X, y, rcond=None)
    except np.linalg.LinAlgError:
      return
    k_new, b_new = float(coef[0]), float(coef[1])
    resid = y - X @ coef
    y_var = float(np.sum((y - y.mean()) ** 2))
    self.shadow_r2 = 1.0 - float(np.sum(resid ** 2)) / y_var if y_var > 0 else 0.0

    # Friction: std of residuals converted to torque space, × 1.5 to cover ~85% of data
    plant_gain_per = k_new / (v_per * v_per) + b_new
    plant_gain_per = np.where(np.abs(plant_gain_per) < 1e-4, 1e-4, plant_gain_per)
    resid_torque = resid / plant_gain_per
    friction_new = float(np.std(resid_torque) * FRICTION_FACTOR)
    friction_new = max(FRICTION_MIN, min(FRICTION_MAX, friction_new))  # sanity clamp

    # Stability: K change small vs previous shadow fit
    k_ref = max(abs(self.shadow_k), 0.1)
    stable = (abs(k_new - self.shadow_k) / k_ref < STABLE_REL_DELTA and
              self.shadow_r2 > MIN_R2)
    self.stable_count = self.stable_count + 1 if stable else 0
    self.shadow_k = k_new
    self.shadow_b = b_new
    self.shadow_friction = friction_new

    # Promotion
    if self.stable_count >= STABLE_CYCLES:
      if not self.validated:
        self.live_k = k_new
        self.live_b = b_new
        self.live_friction = friction_new
        self.validated = True
      else:
        self.live_k = (1 - DECAY) * self.live_k + DECAY * k_new
        self.live_b = (1 - DECAY) * self.live_b + DECAY * b_new
        self.live_friction = (1 - DECAY) * self.live_friction + DECAY * friction_new

  def plant_gain(self, v):
    return self.live_k / (v * v) + self.live_b

  def friction(self):
    return self.live_friction

  def debug_state(self):
    return {
      'shadow_k': self.shadow_k, 'shadow_b': self.shadow_b,
      'shadow_friction': self.shadow_friction, 'shadow_r2': self.shadow_r2,
      'live_k': self.live_k, 'live_b': self.live_b, 'live_friction': self.live_friction,
      'stable_count': self.stable_count,
      'validated': int(self.validated),
      'sample_count': self.sample_count,
      'covered_bins': sum(1 for b in self.buckets if len(b) >= BUCKET_MIN),
    }
