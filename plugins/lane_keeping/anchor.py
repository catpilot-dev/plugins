"""Driver-side lane keeping — pure control core (no cereal/zmq imports).

Anchors the driver-side wheel-to-line gap in a [gap_min, gap_max] deadband
via a bounded pure-pursuit curvature bias. See design spec
docs/superpowers/specs/2026-07-22-driver-side-lane-keeping-design.md.
"""
import math
from dataclasses import dataclass

DT_CTRL = 0.01  # openpilot control-loop period (s); the hook runs at 100 Hz
LAT_DELAY_FALLBACK = 0.6  # s; used when the hook passes no lat_delay
X_PRED_MIN = 5.0          # m; keep a meaningful prediction at crawl
X_PRED_MAX = 50.0         # m; stay inside the model's reliable line region


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


def _interp_arr(x, xs, ys):
  # clamped-left linear interpolation over ascending xs; None past the end or
  # on malformed arrays (caller falls back to the current-gap deadband)
  n = len(xs)
  if n < 2 or n != len(ys):
    return None
  if x <= xs[0]:
    return float(ys[0])
  for i in range(1, n):
    if x <= xs[i]:
      t = (x - xs[i - 1]) / (xs[i] - xs[i - 1])
      return float(ys[i - 1]) + t * (float(ys[i]) - float(ys[i - 1]))
  return None


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
  kappa_filter_tau: float = 0.15  # low-pass on the model's kappa_des (s); group
  # delay tau=0.15s ~= field-verified 300ms box filter's 0.125s group delay.
  # tau=0.3s would exceed the reverted 600ms box's 0.275s group delay (route
  # 395: wobbling and lag) -- a first-order tau is NOT comparable to a box
  # window length, so 0.3s was laggier than the mechanism it was meant to match.
  prob_on: float = 0.5           # driver-side line confidence to engage.
                                 # 0.6->0.5 (2026-07-23, measured): gap noise in prob
                                 # [0.5,0.6) is 0.047-0.050 m — statistically the same as
                                 # the trusted [0.6,0.8) band (0.034-0.061); gains 4-7%
                                 # anchor time on worn-marking roads. Below 0.5 the
                                 # quality cliff is real and route-inconsistent (3c0's
                                 # [0.4,0.5): 0.242 m noise, 11% jumps >0.3 m) — do not.
  prob_fade: float = 0.1         # fade width above prob_on
  # Integral trim (2026-07-23, route 3c0): the pure-pursuit nudge is
  # proportional to excess, so it droops against a persistent disturbance
  # (the model shading left on narrow roads): measured 52% of ticks below
  # band, 90 s episodes with zero recovery, saturation only 13% — classic
  # P-only steady-state error. The trim is the DC authority P lacks: a slow
  # SIGNED integrator — accumulates while out-of-band, so an opposite-side
  # error unwinds it at the same rate it wound (the hold-bias lesson:
  # anti-windup may block only windup, never unwind; no ratchet possible),
  # leaks gently to exactly zero while in-band, hard-capped at half the
  # pursuit cap, authority-scaled, zeroed on lane change.
  trim_rate: float = 1e-4        # trim slew while out-of-band (1/m per s) — full cap in 10 s
  trim_max: float = 1e-3         # hard cap (1/m); half of kappa_bias_max
  trim_leak: float = 2e-5        # in-band decay toward 0 (1/m per s) — 5× slower than rate
  trim_accel_max: float = 0.3    # lateral-accel bound on the trim: |v²·κ_trim| ≤ this (m/s²);
                                 # crossover √(0.3/1e-3)≈17.3 m/s — the 3c0 operating point
                                 # (17.8 m/s) sits just past it, keeping ~95% of the flat cap
  pred_delay_mult: float = 1.5   # prediction horizon = mult × lateral delay
                                 # (sweep 2026-07-23: 1.5 beat trivial on all 3
                                 #  gate routes, +11..32%; 2.0 tie-to-+23; 3.0 worse
                                 #  — the plan is most trustworthy near the car)
  gap_hard_lo: float = 0.3       # current-gap floor: prediction may not defer below (m)
  gap_hard_hi: float = 1.5       # current-gap ceiling: prediction may not defer above (m)


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
    self.gap_pred_filt = None
    self.kappa_filt = None
    self.kappa_bias = 0.0
    self.kappa_trim = 0.0
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

  def _telem(self, prob, line_y, gap, excess, authority, v_ego, kappa_in=0.0, kappa_ref=0.0,
             x_pred=0.0):
    return {
      'prob': float(prob), 'line_y': float(line_y), 'gap': float(gap),
      'gap_filt': float(self.gap_filt) if self.gap_filt is not None else 0.0,
      'excess': float(excess), 'kappa_bias': float(self.kappa_bias),
      'authority': float(authority), 'state': self.state, 'v_ego': float(v_ego),
      'kappa_in': float(kappa_in), 'kappa_ref': float(kappa_ref),
      'kappa_trim': float(self.kappa_trim),
      'gap_pred': float(self.gap_pred_filt) if self.gap_pred_filt is not None else 0.0,
      'x_pred': float(x_pred),
    }

  def update(self, curvature, model_v2, v_ego, lane_changing, lat_delay=None):
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
    x_pred = 0.0
    available = (cfg.enable and model_v2 is not None
                 and len(model_v2.laneLineProbs) > self.driver_idx
                 and len(model_v2.laneLines) > self.driver_idx
                 and len(model_v2.laneLines[self.driver_idx].y) > 0)
    if available:
      prob = float(model_v2.laneLineProbs[self.driver_idx])
      line_y = float(model_v2.laneLines[self.driver_idx].y[0])
      gap = self.line_sign * line_y - cfg.half_width
      alpha = 1.0 - math.exp(-DT_CTRL / cfg.filter_tau)
      # During a lane change, track RAW (re-seed every tick — the kappa_filt
      # discipline): the driver-side line's identity changes under us mid-
      # maneuver, so smoothed history from the old lane is meaningless and
      # would otherwise leak a stale settle-nudge into the new lane. At the
      # moment the change completes the filter already holds the new lane's
      # value — a fresh start with zero convergence gap. Output stays inert
      # during the change regardless (authority 0 + bias hard-zero below).
      if lane_changing or self.gap_filt is None:
        self.gap_filt = gap
      else:
        self.gap_filt += alpha * (gap - self.gap_filt)
      # Predictive deadband (2026-07-23 spec): evaluate the driver-side line at
      # the point the car reaches after pred_delay_mult × lat_delay (~1.2 s for
      # BMW), subtract WHERE THE MODEL'S OWN PLAN puts the car at that point,
      # and decide on THAT gap. A correction commanded now takes ~one lat_delay
      # to act, so 2× = one delay to act + one to observe — the human rhythm.
      #
      # Why the plan and not a constant-curvature extrapolation (replay gate
      # 2026-07-23): a κ_ref·x²/2 path term multiplies κ_des sub-Hz noise by
      # x_pred²/2 (~450 m² at 30 m) — measured predictor RMSE 0.33–0.55 m,
      # 2–6× WORSE than assuming the gap doesn't change. The plan and the lane
      # line come from the same vision frame, so their coherent wander cancels
      # in the difference: measured 0.13–0.14 m, beating trivial (0.15–0.17)
      # on all three gate routes. It is also semantically exact — the Phase-2
      # tracker faithfully follows the plan, so line-minus-plan at x_pred IS
      # the predicted gap. Falls back to the current gap when the line or plan
      # geometry can't cover x_pred.
      pred_t = cfg.pred_delay_mult * (lat_delay if lat_delay else LAT_DELAY_FALLBACK)
      x_pred = _clip(v_ego * pred_t, X_PRED_MIN, X_PRED_MAX)
      line = model_v2.laneLines[self.driver_idx]
      xs = getattr(line, 'x', [])
      y_line = _interp_arr(x_pred, [float(p) for p in xs], [float(p) for p in line.y]) \
        if len(xs) == len(line.y) else None
      plan = getattr(model_v2, 'position', None)
      y_plan = None
      if plan is not None:
        pxs = getattr(plan, 'x', [])
        if len(pxs) == len(plan.y):
          y_plan = _interp_arr(x_pred, [float(p) for p in pxs], [float(p) for p in plan.y])
      if y_line is None or y_plan is None:
        gap_pred = gap
      else:
        gap_pred = self.line_sign * (y_line - y_plan) - cfg.half_width
      if lane_changing or self.gap_pred_filt is None:
        self.gap_pred_filt = gap_pred
      else:
        self.gap_pred_filt += alpha * (gap_pred - self.gap_pred_filt)
      # Hard floors: the prediction may DEFER a correction, never MASK one —
      # on the paint (or drifting far toward the opposite line), correct NOW.
      if self.gap_filt < cfg.gap_hard_lo or self.gap_filt > cfg.gap_hard_hi:
        excess = self._excess(self.gap_filt)
      else:
        excess = self._excess(self.gap_pred_filt)
      authority = _interp(prob, [cfg.prob_on, cfg.prob_on + cfg.prob_fade], [0.0, 1.0])
      if lane_changing:
        authority = 0.0
      kappa_target = authority * self._pursuit(excess, v_ego)
      # Integral trim (see trim_* constants): slow signed integration of the
      # SAME excess the nudge acts on — DC authority against persistent
      # disturbances the proportional law cannot cancel (route 3c0 droop).
      # Signed: opposite-side excess unwinds at the same rate (no ratchet).
      # Authority-scaled so a fading line also fades the accumulation.
      if excess != 0.0 and authority > 0.0:
        step = self.cfg.trim_rate * DT_CTRL * authority
        self.kappa_trim = _clip(self.kappa_trim + math.copysign(step, self.curv_sign * excess),
                                -self.cfg.trim_max, self.cfg.trim_max)
      elif self.kappa_trim != 0.0:
        # In-band, OR visible-but-untrusted line (authority 0 — review
        # finding): leak. Never freeze DC state without a trusted
        # measurement, and never snap a stale trim back at full value when
        # confidence returns.
        self.kappa_trim -= math.copysign(
          min(self.cfg.trim_leak * DT_CTRL, abs(self.kappa_trim)), self.kappa_trim)
    else:
      self.gap_filt = None
      self.gap_pred_filt = None
      kappa_target = 0.0
      # No measurement: never integrate blind — leak the trim gently instead
      # (brief line dropouts keep most of it; long loss decays to exactly 0).
      if self.kappa_trim != 0.0:
        self.kappa_trim -= math.copysign(
          min(self.cfg.trim_leak * DT_CTRL, abs(self.kappa_trim)), self.kappa_trim)
    # Speed-dependent trim clamp (review finding): the pursuit term's lateral
    # accel is speed-independent by construction (v² cancels in 2·excess/Lp²·v²)
    # but the trim's is NOT — a flat κ cap would grow as v²·trim_max (0.9 m/s²
    # at 108 km/h). Bound it explicitly: |v²·κ_trim| ≤ trim_accel_max.
    # Re-clamped every tick, so accelerating onto a highway sheds excess trim
    # gently as v rises (v changes slowly; no step).
    cap_eff = min(cfg.trim_max, cfg.trim_accel_max / max(v_ego * v_ego, 1.0))
    self.kappa_trim = _clip(self.kappa_trim, -cap_eff, cap_eff)
    # single rate-limit path (also smoothly releases bias to 0 when the line is
    # lost or its confidence fades — no snap on anchor exit).
    max_step = cfg.kappa_rate_max * DT_CTRL
    self.kappa_bias = _clip(kappa_target, self.kappa_bias - max_step, self.kappa_bias + max_step)
    # Lane change is the exception: hard-zero the bias immediately (bypassing the
    # smooth release) so the anchor never fights the maneuver, per spec §3.4.
    if lane_changing:
      self.kappa_bias = 0.0
      self.kappa_trim = 0.0
    self.state = 'anchor' if (available and authority > 0.0) else 'model'
    return kappa_ref + self.kappa_bias + self.kappa_trim, self._telem(prob, line_y, gap, excess, authority, v_ego,
                                                    curvature, kappa_ref, x_pred=x_pred)
