"""Lateral reference conditioner — pure control core (no cereal/zmq imports).

Turns the model's curvature into the reference the vehicle's lateral
controller tracks: an arc through the model's planned path `arc_horizon`
seconds ahead (spatial averaging over one frame's path, so no time lag),
low-passed with `kappa_filter_tau`. A lane change bypasses both — the model is
deliberately reframing the trajectory. The ego lane-line geometry rides along
in the telemetry for drive analysis.

History: docs/superpowers/specs/ (2026-07-22 lateral-controller-simplification,
2026-10-18 bmw-direct-tracking, 2026-10-20 curve-feedforward-and-audit).
"""
import math
from dataclasses import dataclass

DT_CTRL = 0.01  # openpilot control-loop period (s); the hook runs at 100 Hz
ARC_X_MIN = 5.0           # m; arc aim point closer than this (crawl) is
                          # ill-conditioned -> use the model's curvature
# Arc look-ahead, speed- and curvature-dependent. Longer removes more
# straight-line wander at every speed (offline 4d4-4dc, 75-95 km/h: arc/raw
# 0.61 at 1.5 s -> 0.47 at 2.5 s) but softens S-curve legs and turns in
# earlier, worst at low speed and on curvy roads. Speed sets it on straights;
# the max plan curvature within ARC_K_AHEAD_T pulls it back to ARC_H_CURVE
# (straights: p99 1.38e-3; steady curves: p10 4.1e-3). Linear blends only -
# no gates. Speed edges 35/55 km/h (route 4eb): at 60 km/h a 2.1 s look-ahead
# passed 0.52 of the model's wobble vs 0.46 at 2.5 s; curves with
# |kappa_des| > 5e-3 replay identical because the pull-back sets H there.
ARC_H_SPEED_V = [35.0 / 3.6, 55.0 / 3.6]  # m/s
ARC_H_SPEED = [1.5, 2.5]                  # s
ARC_H_CURVE = 1.5                         # s, on a curvy road ahead
ARC_K_AHEAD_BP = [0.0015, 0.003]          # 1/m, straight -> curvy blend edges
ARC_K_AHEAD_T = 2.5                       # s of plan scanned for curvature


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


def plan_curvature_ahead(model_v2, t_max=ARC_K_AHEAD_T):
  """Max |orientationRate.z / velocity.x| over plan points with t <= t_max (the
  per-point curvature speedlimitd uses). None when the fields are missing."""
  if model_v2 is None:
    return None
  plan, rate, vel = (getattr(model_v2, n, None) for n in ('position', 'orientationRate', 'velocity'))
  if plan is None or rate is None or vel is None:
    return None
  ts = [float(p) for p in getattr(plan, 't', [])]
  zs = [float(p) for p in getattr(rate, 'z', [])]
  vs = [float(p) for p in getattr(vel, 'x', [])]
  if not ts or not (len(ts) == len(zs) == len(vs)):
    return None
  ks = [abs(z / vx) for t, z, vx in zip(ts, zs, vs) if t <= t_max and vx > 1.0]
  return max(ks) if ks else None


def arc_horizon(v_ego, k_ahead):
  """Look-ahead (s): ARC_H_SPEED by speed on straights, blended down to
  ARC_H_CURVE as the plan ahead gets curvy. No curvature info -> speed only."""
  h_speed = _interp(v_ego, ARC_H_SPEED_V, ARC_H_SPEED)
  if k_ahead is None:
    return h_speed
  return _interp(k_ahead, ARC_K_AHEAD_BP, [h_speed, ARC_H_CURVE])


def arc_curvature(model_v2, horizon):
  """Pure-pursuit arc through the model's plan point `horizon` seconds ahead:
  2*y/(x^2+y^2) — the curvature of the circle from the car (tangent to its
  heading) through that point. Spatial averaging over one frame's path, so no
  time lag (routes 4d4/4d5: 30-40% less 0.1-1 Hz wander than desiredCurvature
  at 1.5-2.0 s, same timing). position.y shares desiredCurvature's sign here,
  so no flip. None when the plan is missing/malformed or the aim point sits
  closer than ARC_X_MIN."""
  plan = getattr(model_v2, 'position', None) if model_v2 is not None else None
  if plan is None:
    return None
  ts = [float(p) for p in getattr(plan, 't', [])]
  xs = [float(p) for p in getattr(plan, 'x', [])]
  ys = [float(p) for p in getattr(plan, 'y', [])]
  if not (len(ts) == len(xs) == len(ys)):
    return None
  xa, ya = _interp_arr(horizon, ts, xs), _interp_arr(horizon, ts, ys)
  if xa is None or ya is None or xa < ARC_X_MIN:
    return None
  return 2.0 * ya / (xa * xa + ya * ya)


def _interp_arr(x, xs, ys):
  # clamped-left linear interpolation over ascending xs; None past the end or
  # on malformed arrays (the caller falls back to the model's curvature)
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
  kappa_filter_tau: float = 0.15  # low-pass on the reference (s). Group delay
                                  # ~= the field-verified 300 ms box filter's
                                  # 0.125 s; 0.3 s lagged and wobbled (route 395).
  arc_horizon: float = 0.0        # plan time (s) of the arc aim point; 0 = auto
                                  # (arc_horizon(): speed + curvature ahead)


class LaneAnchor:
  def __init__(self, config: AnchorConfig):
    self.cfg = config
    self.kappa_filt = None

  def update(self, curvature, model_v2, v_ego, lane_changing):
    """Returns (kappa_ref, telemetry). kappa_ref is the arc through the plan
    ahead, low-passed; the model's curvature when there is no usable plan; the
    raw input during a lane change."""
    cfg = self.cfg
    k_ahead = plan_curvature_ahead(model_v2)
    arc_h = cfg.arc_horizon if cfg.arc_horizon > 0.0 else arc_horizon(v_ego, k_ahead)
    kappa_arc = arc_curvature(model_v2, arc_h)
    arc_on = kappa_arc is not None and not lane_changing
    kappa_src = kappa_arc if arc_on else curvature
    if lane_changing or self.kappa_filt is None:
      self.kappa_filt = kappa_src
      kappa_ref = kappa_src
    else:
      a_k = 1.0 - math.exp(-DT_CTRL / cfg.kappa_filter_tau)
      self.kappa_filt += a_k * (kappa_src - self.kappa_filt)
      kappa_ref = self.kappa_filt
    # Ego lane lines at the car (m, +y = right) and their probabilities, logged
    # whatever the state: they are how sway and drift are measured offline.
    geo = (0.0, 0.0, 0.0, 0.0)
    if model_v2 is not None and len(getattr(model_v2, 'laneLines', [])) > 2 \
       and len(getattr(model_v2, 'laneLineProbs', [])) > 2 \
       and len(model_v2.laneLines[1].y) > 0 and len(model_v2.laneLines[2].y) > 0:
      geo = (float(model_v2.laneLines[1].y[0]), float(model_v2.laneLines[2].y[0]),
             float(model_v2.laneLineProbs[1]), float(model_v2.laneLineProbs[2]))
    telem = {
      'kappa_in': float(curvature), 'kappa_ref': float(kappa_ref),
      'kappa_arc': float(kappa_arc) if kappa_arc is not None else 0.0,
      'arc_on': bool(arc_on), 'arc_h': float(arc_h),
      'k_ahead': float(k_ahead) if k_ahead is not None else -1.0,
      'v_ego': float(v_ego),
      'line_l': geo[0], 'line_r': geo[1], 'prob_l': geo[2], 'prob_r': geo[3],
    }
    return kappa_ref, telem
