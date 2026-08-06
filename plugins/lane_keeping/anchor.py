"""Driver-side lane keeping — pure control core (no cereal/zmq imports).

AC STABILIZER (Phase 3): damps the wander of the driver-side wheel-to-line
gap around the model's own chosen line (a slow DC tracker concedes the line;
only deadbanded deviations from it are corrected via a bounded pure-pursuit
bias). Hard floors at the extremes remain absolute best-effort. See
docs/superpowers/specs/2026-07-23-ac-stabilizer-design.md (which supersedes
the positioner design in the 2026-07-22/predictive-deadband specs).
"""
import math
from dataclasses import dataclass

DT_CTRL = 0.01  # openpilot control-loop period (s); the hook runs at 100 Hz
LAT_DELAY_FALLBACK = 0.6  # s; used when the hook passes no lat_delay
X_PRED_MIN = 5.0          # m; keep a meaningful prediction at crawl
X_PRED_MAX = 50.0         # m; stay inside the model's reliable line region
AC_DEADBAND_V = [15.0, 25.0]  # m/s (54-90 km/h) — speed breakpoints for the
                              # ac_deadband -> ac_deadband_hi taper (2026-08-06,
                              # route 3e7: see the ac_deadband config comment)


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
  lp_max: float = 25.0           # look-ahead distance cap (m). Route 3e7 segs
                                 # 41-48 (2026-08-06, 82 km/h): uncapped lp =
                                 # v·t_preview collapses pursuit gain as 1/v²
                                 # (3.7× weaker than the urban speeds the damper
                                 # was field-tuned at), leaving 2/3 of the model's
                                 # sub-Hz wander uncancelled. Capping the aim
                                 # point at 25 m (reached at 60 km/h) keeps the
                                 # field-verified urban behavior bit-identical
                                 # and lets bias a_y authority grow with v²
                                 # beyond it, matching how the wander's a_y
                                 # grows. kappa_bias_max/rate still bound output.
  excess_max: float = 0.5        # max deadband excess acted on (m)
  kappa_bias_max: float = 0.002  # hard cap on curvature bias (1/m)
  kappa_rate_max: float = 0.002  # bias slew (1/m per second)
  filter_tau: float = 0.7        # gap low-pass time constant (s)
  kappa_filter_tau: float = 0.15  # low-pass on the model's kappa_des (s); group
  # delay tau=0.15s ~= field-verified 300ms box filter's 0.125s group delay.
  # tau=0.3s would exceed the reverted 600ms box's 0.275s group delay (route
  # 395: wobbling and lag) -- a first-order tau is NOT comparable to a box
  # window length, so 0.3s was laggier than the mechanism it was meant to match.
  dc_tau: float = 20.0           # DC-tracker time constant (s) — the forgetting time.
                                 # The stabilizer concedes any disagreement older than
                                 # ~this: zero-mean correction by construction, so the
                                 # 3c1 arm-wrestle (model counter-steers a sustained
                                 # bias to a stalemate) is structurally impossible.
  ac_deadband: float = 0.10      # AC excess ignored below this (m) at/below
                                 # AC_DEADBAND_V[0] — micro-noise guard where
                                 # pursuit gain is at its urban maximum
  ac_deadband_hi: float = 0.05   # deadband at/above AC_DEADBAND_V[1] (user rule
                                 # 2026-08-06: higher speed, tighter deadband —
                                 # route 3e7: the 0.10 band forgave 36% of the
                                 # wander amplitude at 82 km/h). Safe headroom:
                                 # above the taper the lp_max cap pins pursuit
                                 # gain, so a 0.05 m noise blip commands only
                                 # ~1.6e-4 kappa (~0.1 m/s^2 a_y at 90 km/h)
  prob_on: float = 0.5           # driver-side line confidence to engage.
                                 # 0.6->0.5 (2026-07-23, measured): gap noise in prob
                                 # [0.5,0.6) is 0.047-0.050 m — statistically the same as
                                 # the trusted [0.6,0.8) band (0.034-0.061); gains 4-7%
                                 # anchor time on worn-marking roads. Below 0.5 the
                                 # quality cliff is real and route-inconsistent (3c0's
                                 # [0.4,0.5): 0.242 m noise, 11% jumps >0.3 m) — do not.
  prob_fade: float = 0.1         # fade width above prob_on
  pred_delay_mult: float = 1.5   # prediction horizon = mult × lateral delay
                                 # (sweep 2026-07-23: 1.5 beat trivial on all 3
                                 #  gate routes, +11..32%; 2.0 tie-to-+23; 3.0 worse
                                 #  — the plan is most trustworthy near the car)
  gap_hard_lo: float = 0.3       # current-gap floor: prediction may not defer below (m)
  gap_hard_hi: float = 1.5       # current-gap ceiling: prediction may not defer above (m)
  asym_gap: float = 0.6          # suppress toward-line damping when gap_filt is below this (m);
                                 # 0 disables (exact prior symmetric behavior). The only
                                 # width-dependent term besides the hard floors above — see
                                 # Addendum 2026-07-27 in the AC stabilizer design doc.


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
    self.gap_dc = None
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
    # look-ahead floor avoids div0 at standstill; lp_max cap keeps highway
    # authority (see the lp_max config comment)
    lp = _clip(v_ego * cfg.t_preview, 1.0, cfg.lp_max)
    kappa = self.curv_sign * 2.0 * excess / (lp * lp)
    return _clip(kappa, -cfg.kappa_bias_max, cfg.kappa_bias_max)

  def _telem(self, prob, line_y, gap, excess, authority, v_ego, kappa_in=0.0, kappa_ref=0.0,
             x_pred=0.0, gap_dc=0.0, excess_ac=0.0):
    return {
      'prob': float(prob), 'line_y': float(line_y), 'gap': float(gap),
      'gap_filt': float(self.gap_filt) if self.gap_filt is not None else 0.0,
      'excess': float(excess), 'kappa_bias': float(self.kappa_bias),
      'authority': float(authority), 'state': self.state, 'v_ego': float(v_ego),
      'kappa_in': float(kappa_in), 'kappa_ref': float(kappa_ref),
      'gap_pred': float(self.gap_pred_filt) if self.gap_pred_filt is not None else 0.0,
      'x_pred': float(x_pred),
      'gap_dc': float(gap_dc), 'excess_ac': float(excess_ac),
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
    excess_ac = 0.0
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
      authority = _interp(prob, [cfg.prob_on, cfg.prob_on + cfg.prob_fade], [0.0, 1.0])
      if lane_changing:
        authority = 0.0
      # DC tracker (Phase 3): adiabatically follow the model's chosen line.
      # Seeds on the first anchor sample; adapts only while the measurement is
      # trusted; FROZEN on low authority (dropouts keep the reference); RESET
      # by a lane change (new lane, new line identity, new DC).
      in_floor = self.gap_filt < cfg.gap_hard_lo or self.gap_filt > cfg.gap_hard_hi
      if lane_changing:
        self.gap_dc = None
      elif not in_floor and authority > 0.0:
        # Seed AND adapt only from a TRUSTED measurement (final review: an
        # authority-0 seed pins a stale reference that snaps back on
        # confidence return — the same failure class fixed twice before).
        # Never while the hard-floor override is active either: the excursion
        # the floor is fighting must not become the reference (frozen, the
        # post-floor AC term mildly assists the recovery instead).
        if self.gap_dc is None:
          self.gap_dc = self.gap_pred_filt
        else:
          a_dc = 1.0 - math.exp(-DT_CTRL / cfg.dc_tau)
          self.gap_dc += a_dc * (self.gap_pred_filt - self.gap_dc)
      # Decision: hard floors are ABSOLUTE (best-effort at the extremes);
      # otherwise damp only the AC — the deviation from the tracked line.
      # Zero-mean by construction: a static scene, at ANY gap, concedes.
      excess_ac = (self.gap_pred_filt - self.gap_dc) if self.gap_dc is not None else 0.0
      if in_floor:
        excess = self._excess(self.gap_filt)
      else:
        db = _interp(v_ego, AC_DEADBAND_V, [cfg.ac_deadband, cfg.ac_deadband_hi])
        excess = excess_ac - _clip(excess_ac, -db, db)
        excess = _clip(excess, -cfg.excess_max, cfg.excess_max)
        # Addendum 2026-07-27: near the driver-side line, suppress only the
        # toward-line direction (excess > 0 -> pursuit steers toward the
        # driver line, restoring the DC) — that is the direction that opposes
        # the model's own escape from a too-close line. Away-pushes
        # (excess <= 0) are kept unconditionally.
        if cfg.asym_gap > 0.0 and self.gap_filt < cfg.asym_gap and excess > 0.0:
          excess = 0.0
      kappa_target = authority * self._pursuit(excess, v_ego)
    else:
      self.gap_filt = None
      self.gap_pred_filt = None
      kappa_target = 0.0
      if not cfg.enable:
        # Deliberate disable (Driving-panel toggle): forget the reference —
        # re-enabling starts fresh on the current line, a clean A/B. Line
        # DROPOUTS (enable True) keep the frozen DC through the gap.
        self.gap_dc = None
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
                                                    curvature, kappa_ref, x_pred=x_pred,
                                                    gap_dc=self.gap_dc if self.gap_dc is not None else 0.0,
                                                    excess_ac=excess_ac)
