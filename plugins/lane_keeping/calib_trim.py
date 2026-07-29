"""Calibration trim law — slow yaw-bias integrator on gap_dc.

Pure math; no I/O. Spec: docs/superpowers/specs/
2026-07-25-calibration-trim-design.md section 5. delta_deg is the ONLY
state; every transition is slew-limited; the value is written to
data/CalibTrimYawDeg by register.py and applied inside modeld via the
modeld.calib_bias hook.
"""
import math
from dataclasses import dataclass

DT_CTRL = 0.01
IN_BAND_DWELL_S = 5.0
HOLD_MIN_SPEED = 5.0


def _clip(v, lo, hi):
  return max(lo, min(hi, v))


@dataclass
class TrimConfig:
  mode: int = 0
  fixed_deg: float = 0.0
  max_deg: float = 0.8
  slew_deg_s: float = 0.02
  yaw_sign: int = 0
  ki: float = 0.04
  gap_lo: float = 0.6
  gap_hi: float = 1.0


class CalibTrim:
  def __init__(self, cfg: TrimConfig, initial_deg: float = 0.0):
    # Seed delta_deg from the (already clamped-on-disk) persisted file value
    # so the controls law, the file, and modeld agree at startup with no
    # step (spec section 3). A nonfinite seed (shouldn't happen — the file
    # reader clamps — but defend anyway) collapses to 0.0 rather than
    # propagating nan/inf into the slew-limited state.
    if not math.isfinite(initial_deg):
      initial_deg = 0.0
    self.cfg = cfg
    self.delta_deg = _clip(initial_deg, -cfg.max_deg, cfg.max_deg)
    self._in_band_ticks = 0

  def _slew_toward(self, target, rate):
    step = rate * DT_CTRL
    self.delta_deg += _clip(target - self.delta_deg, -step, step)

  def update(self, gap_dc, authority, lane_changing, v_ego, enabled):
    cfg = self.cfg
    integrating = False
    err = 0.0
    if not enabled or cfg.mode == 0 or (cfg.mode == 2 and cfg.yaw_sign not in (1, -1)):
      self._slew_toward(0.0, cfg.slew_deg_s)
      self._in_band_ticks = 0
    elif cfg.mode == 1:
      self._slew_toward(_clip(cfg.fixed_deg, -cfg.max_deg, cfg.max_deg), cfg.slew_deg_s)
      self._in_band_ticks = 0
    else:  # mode 2, sign valid
      gate = (gap_dc is not None and authority > 0.0
              and not lane_changing and v_ego >= HOLD_MIN_SPEED)
      if gate:
        if gap_dc < cfg.gap_lo:
          err = gap_dc - cfg.gap_lo
        elif gap_dc > cfg.gap_hi:
          err = gap_dc - cfg.gap_hi
        else:
          err = 0.0
        if err != 0.0:
          self._in_band_ticks = 0
          integrating = True
          self.delta_deg += _clip(-cfg.ki * err * cfg.yaw_sign,
                                  -cfg.slew_deg_s, cfg.slew_deg_s) * DT_CTRL
        else:
          self._in_band_ticks += 1
          if self._in_band_ticks * DT_CTRL > IN_BAND_DWELL_S:
            self._slew_toward(0.0, cfg.slew_deg_s / 2.0)
      else:
        # gate failed: a blind period is not observed continuous in-band
        # dwell — a fresh IN_BAND_DWELL_S of *observed* in-band is required
        # after any dropout before decay may resume.
        self._in_band_ticks = 0
      # gate False: hold (no integrate, no decay)
    self.delta_deg = _clip(self.delta_deg, -cfg.max_deg, cfg.max_deg)
    telem = {'delta_deg': self.delta_deg, 'err': err, 'mode': cfg.mode, 'integrating': integrating}
    return self.delta_deg, telem
