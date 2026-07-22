"""Driver-side lane keeping — pure control core (no cereal/zmq imports).

Anchors the driver-side wheel-to-line gap in a [gap_min, gap_max] deadband
via a bounded pure-pursuit curvature bias. See design spec
docs/superpowers/specs/2026-07-22-driver-side-lane-keeping-design.md.
"""
import math
from dataclasses import dataclass

DT_CTRL = 0.01  # openpilot control-loop period (s); the hook runs at 100 Hz


def _clip(x, lo, hi):
  return lo if x < lo else hi if x > hi else x


def _interp(x, xp, fp):
  # two-point clamped linear interpolation (xp ascending, len 2)
  if x <= xp[0]:
    return fp[0]
  if x >= xp[1]:
    return fp[1]
  t = (x - xp[0]) / (xp[1] - xp[0])
  return fp[0] + t * (fp[1] - fp[0])


@dataclass
class AnchorConfig:
  enable: bool = True
  driver_side: str = 'left'      # 'left' or 'right'
  half_width: float = 0.91       # car half-width (m); E90 ~1.817 m
  gap_min: float = 0.6           # driver-wheel-to-line comfort band (m)
  gap_max: float = 1.0
  t_preview: float = 1.5         # pure-pursuit look-ahead time (s)
  excess_max: float = 0.5        # max deadband excess acted on (m)
  kappa_bias_max: float = 0.002  # hard cap on curvature bias (1/m)
  kappa_rate_max: float = 0.002  # bias slew (1/m per second)
  filter_tau: float = 0.7        # gap low-pass time constant (s)
  kappa_filter_tau: float = 0.3  # low-pass on the model's kappa_des (s)
  prob_on: float = 0.6           # driver-side line confidence to engage
  prob_fade: float = 0.1         # fade width above prob_on


class LaneAnchor:
  def __init__(self, config: AnchorConfig):
    self.cfg = config
    self.driver_idx = 1 if config.driver_side == 'left' else 2
    # Two different sign conventions are in play:
    #  - modelV2 laneLines are in the device frame, +y = RIGHT (camera.py), so
    #    laneLines[1] (left ego line) sits at NEGATIVE y and laneLines[2]
    #    (right) at POSITIVE y. Distance from car center to the driver-side
    #    line is therefore -y (left) or +y (right): line_sign converts to it.
    #  - desiredCurvature (what the bias is added to, consumed by the BMW
    #    controller) is LEFT-positive. To reduce a positive excess (car too far
    #    from the driver line) steer TOWARD the driver side: left -> +curvature
    #    (left turn), right -> -curvature: curv_sign gives that direction.
    self.line_sign = -1.0 if config.driver_side == 'left' else 1.0
    self.curv_sign = 1.0 if config.driver_side == 'left' else -1.0
    self.gap_filt = None
    self.kappa_filt = None
    self.kappa_bias = 0.0
    self.state = 'model'

  def _gap(self, model_v2):
    line_y = float(model_v2.laneLines[self.driver_idx].y[0])
    return self.line_sign * line_y - self.cfg.half_width

  def _excess(self, gap_filt):
    cfg = self.cfg
    excess = gap_filt - _clip(gap_filt, cfg.gap_min, cfg.gap_max)
    return _clip(excess, -cfg.excess_max, cfg.excess_max)

  def _pursuit(self, excess, v_ego):
    cfg = self.cfg
    lp = max(v_ego * cfg.t_preview, 1.0)   # look-ahead floor avoids div0 at standstill
    kappa = self.curv_sign * 2.0 * excess / (lp * lp)
    return _clip(kappa, -cfg.kappa_bias_max, cfg.kappa_bias_max)

  def _telem(self, prob, line_y, gap, excess, authority, v_ego, kappa_in=0.0, kappa_ref=0.0):
    return {
      'prob': float(prob), 'line_y': float(line_y), 'gap': float(gap),
      'gap_filt': float(self.gap_filt) if self.gap_filt is not None else 0.0,
      'excess': float(excess), 'kappa_bias': float(self.kappa_bias),
      'authority': float(authority), 'state': self.state, 'v_ego': float(v_ego),
      'kappa_in': float(kappa_in), 'kappa_ref': float(kappa_ref),
    }

  def update(self, curvature, model_v2, v_ego, lane_changing):
    cfg = self.cfg
    # Reference conditioning (Phase 2): low-pass the model's curvature to kill
    # the fast chatter, so the BMW controller can track it faithfully with no
    # deadzone. Safe because any lag this introduces shows up as position
    # drift, which the position correction below closes. A lane change bypasses
    # it — the model is deliberately reframing the trajectory.
    if lane_changing or self.kappa_filt is None:
      self.kappa_filt = curvature
      kappa_ref = curvature
    else:
      a_k = 1.0 - math.exp(-DT_CTRL / cfg.kappa_filter_tau)
      self.kappa_filt += a_k * (curvature - self.kappa_filt)
      kappa_ref = self.kappa_filt
    prob = line_y = gap = excess = authority = 0.0
    available = (cfg.enable and model_v2 is not None
                 and len(model_v2.laneLineProbs) > self.driver_idx
                 and len(model_v2.laneLines) > self.driver_idx
                 and len(model_v2.laneLines[self.driver_idx].y) > 0)
    if available:
      prob = float(model_v2.laneLineProbs[self.driver_idx])
      line_y = float(model_v2.laneLines[self.driver_idx].y[0])
      gap = self.line_sign * line_y - cfg.half_width
      if self.gap_filt is None:
        self.gap_filt = gap
      else:
        alpha = 1.0 - math.exp(-DT_CTRL / cfg.filter_tau)
        self.gap_filt += alpha * (gap - self.gap_filt)
      excess = self._excess(self.gap_filt)
      authority = _interp(prob, [cfg.prob_on, cfg.prob_on + cfg.prob_fade], [0.0, 1.0])
      if lane_changing:
        authority = 0.0
      kappa_target = authority * self._pursuit(excess, v_ego)
    else:
      self.gap_filt = None
      kappa_target = 0.0
    # single rate-limit path (also smoothly releases bias to 0 when the line is
    # lost or its confidence fades — no snap on anchor exit).
    max_step = cfg.kappa_rate_max * DT_CTRL
    self.kappa_bias = _clip(kappa_target, self.kappa_bias - max_step, self.kappa_bias + max_step)
    # Lane change is the exception: hard-zero the bias immediately (bypassing the
    # smooth release) so the anchor never fights the maneuver, per spec §3.4.
    if lane_changing:
      self.kappa_bias = 0.0
    self.state = 'anchor' if (available and authority > 0.0) else 'model'
    return kappa_ref + self.kappa_bias, self._telem(prob, line_y, gap, excess, authority, v_ego,
                                                    curvature, kappa_ref)
