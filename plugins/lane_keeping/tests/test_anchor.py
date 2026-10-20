import importlib.util, os
import math as _math
from types import SimpleNamespace
import pytest
# Load the pure control core by explicit path under a unique module name — do
# NOT insert the plugin dir on sys.path, which would shadow other plugins'
# same-named modules (e.g. bmw's `register`) when the whole suite runs together.
_PLUGIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_spec = importlib.util.spec_from_file_location('lk_anchor', os.path.join(_PLUGIN_DIR, 'anchor.py'))
_anchor = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_anchor)
AnchorConfig, LaneAnchor = _anchor.AnchorConfig, _anchor.LaneAnchor


# modelV2 device frame: +y = RIGHT, so the LEFT ego line (laneLines[1]) sits at
# NEGATIVE y and the RIGHT ego line (laneLines[2]) at POSITIVE y.
def _mv(left_y, right_y, left_p=1.0, right_p=1.0):
  return SimpleNamespace(
    laneLines=[SimpleNamespace(y=[0.0]), SimpleNamespace(y=[left_y]),
               SimpleNamespace(y=[right_y]), SimpleNamespace(y=[0.0])],
    laneLineProbs=[0.0, left_p, right_p, 0.0])


def _circle_plan(k, v=20.0, n=41, t_max=10.0):
  """Plan on an exact circle of curvature k at speed v: arc length s = v*t,
  x = sin(s*k)/k, y = (1-cos(s*k))/k. 2y/(x^2+y^2) == k exactly at a grid
  point; the 0.25 s grid puts the 1.5 s aim point on one (between points the
  plan is interpolated linearly, so off-grid it is only ~1% exact)."""
  ts = [t_max * i / (n - 1) for i in range(n)]
  if k == 0.0:
    xs, ys = [v * t for t in ts], [0.0] * n
  else:
    xs = [_math.sin(v * t * k) / k for t in ts]
    ys = [(1.0 - _math.cos(v * t * k)) / k for t in ts]
  return SimpleNamespace(t=ts, x=xs, y=ys)


def _rates(k, v=20.0, n=41, t_max=10.0, spike_at=None, spike=0.0):
  """orientationRate / velocity for a constant-curvature plan (yaw rate v*k),
  optionally with a yaw-rate spike at one time."""
  ts = [t_max * i / (n - 1) for i in range(n)]
  z = [v * k + (spike if spike_at is not None and abs(t - spike_at) < 1e-9 else 0.0) for t in ts]
  return SimpleNamespace(z=z), SimpleNamespace(x=[v] * n), ts


def _mv_plan(plan):
  m = _mv(left_y=-1.75, right_y=1.75)
  m.position = plan
  return m


# --- Reference low-pass -------------------------------------------------------

def test_smoothing_lags_a_step_then_converges():
  # No plan -> the output is purely the smoothed model curvature.
  a = LaneAnchor(AnchorConfig(kappa_filter_tau=0.3))
  none_mv = SimpleNamespace(laneLines=[], laneLineProbs=[])
  out1, t1 = a.update(0.02, none_mv, 25.0, False)
  assert abs(out1 - 0.02) < 1e-9          # first sample seeds the filter
  assert abs(t1['kappa_in'] - 0.02) < 1e-9
  out2, _ = a.update(0.0, none_mv, 25.0, False)   # step down
  assert 0.0 < out2 < 0.02                # lags, does not jump
  for _ in range(500):                    # ~5s >> tau
    out, _t = a.update(0.0, none_mv, 25.0, False)
  assert abs(out) < 1e-4                  # converges


def test_smoothing_bypassed_during_lane_change():
  a = LaneAnchor(AnchorConfig(kappa_filter_tau=0.3))
  none_mv = SimpleNamespace(laneLines=[], laneLineProbs=[])
  for _ in range(200):
    a.update(0.02, none_mv, 25.0, False)  # settle filter at 0.02
  out, telem = a.update(0.0, none_mv, 25.0, True)   # lane change -> raw
  assert out == 0.0                       # bit-identical passthrough of raw
  assert abs(telem['kappa_ref']) < 1e-12


def test_smoothing_applies_with_lane_lines_present():
  a = LaneAnchor(AnchorConfig(kappa_filter_tau=0.3))
  mv = _mv(left_y=-1.75, right_y=1.75)    # lines but no plan: model curvature path
  a.update(0.02, mv, 25.0, False)
  out, telem = a.update(0.0, mv, 25.0, False)
  assert telem['arc_on'] is False
  assert 0.0 < out < 0.02


# --- Arc smoothing (2026-10-12) ----------------------------------------------
# Reference = pure-pursuit arc through the model's plan point at arc_horizon
# seconds, 2*y/(x^2+y^2), instead of the model's single desiredCurvature.
# Spatial averaging over one frame's path: no time lag. position.y shares
# desiredCurvature's sign in this stack (route 4d4: corr +0.95), so the arc
# needs no sign flip.

def test_arc_curvature_recovers_a_circle():
  for k in (0.004, -0.004, 0.0123):
    got = _anchor.arc_curvature(_mv_plan(_circle_plan(k)), 1.5)
    assert got == pytest.approx(k, rel=1e-6)


def test_arc_curvature_none_without_plan_or_at_crawl():
  assert _anchor.arc_curvature(_mv(left_y=-1.75, right_y=1.75), 1.5) is None
  # 1.5 s at 2 m/s = 3 m < ARC_X_MIN: ill-conditioned, fall back
  assert _anchor.arc_curvature(_mv_plan(_circle_plan(0.01, v=2.0)), 1.5) is None


def test_arc_reference_follows_the_plan_arc():
  a = LaneAnchor(AnchorConfig(arc_horizon=1.5))
  out, telem = a.update(0.002, _mv_plan(_circle_plan(0.006)), 20.0, False)
  assert out == pytest.approx(0.006, rel=1e-6)
  assert telem['arc_on'] is True
  assert telem['kappa_arc'] == pytest.approx(0.006, rel=1e-6)
  assert telem['kappa_in'] == pytest.approx(0.002)   # raw model value still logged


def test_arc_still_low_passed():
  a = LaneAnchor(AnchorConfig(arc_horizon=1.5))
  a.update(0.0, _mv_plan(_circle_plan(0.0)), 20.0, False)
  out, _ = a.update(0.0, _mv_plan(_circle_plan(0.006)), 20.0, False)
  assert 0.0 < out < 0.006


def test_arc_falls_back_to_model_curvature_without_plan():
  a = LaneAnchor(AnchorConfig())
  out, telem = a.update(0.003, _mv(left_y=-1.75, right_y=1.75), 20.0, False)
  assert out == pytest.approx(0.003)
  assert telem['arc_on'] is False


def test_arc_bypassed_during_lane_change():
  a = LaneAnchor(AnchorConfig())
  out, telem = a.update(0.003, _mv_plan(_circle_plan(0.006)), 20.0, True)
  assert out == pytest.approx(0.003)
  assert telem['arc_on'] is False


# --- Speed- and curvature-dependent arc look-ahead (2026-10-12) ---------------
# Longer look-ahead removes more wander on straights at every speed (offline
# 4d4-4dc: 75-95 km/h arc/raw 0.61 at 1.5 s -> 0.47 at 2.5 s) but softens
# S-curve legs and turns in earlier, worst at low speed and on curvy roads.
# Speed sets it on straights (1.5 s <= 35 km/h -> 2.5 s >= 55 km/h; was
# 45 -> 75 km/h until route 4eb: 60 km/h passed 0.52 of the model's wobble vs
# 0.43 at 2.5 s, curves unchanged by the pull-back); the max
# plan curvature within 2.5 s pulls it back to 1.5 s (blend 1.5e-3 -> 3e-3;
# straights p99 1.38e-3, steady curves p10 4.1e-3).

def test_horizon_speed_schedule_on_straights():
  h = _anchor.arc_horizon
  assert h(35 / 3.6, 0.0) == pytest.approx(1.5)        # 35 km/h
  assert h(45 / 3.6, 0.0) == pytest.approx(2.0)        # midway
  assert h(55 / 3.6, 0.0) == pytest.approx(2.5)        # 55 km/h
  assert h(60 / 3.6, 0.0) == pytest.approx(2.5)        # route 4eb swing speed
  assert h(25.0, 0.0) == pytest.approx(2.5)            # 90 km/h
  assert h(40.0, 0.0) == pytest.approx(2.5)            # clamped


def test_horizon_curvy_road_ahead_pulls_back_to_short():
  h = _anchor.arc_horizon
  assert h(25.0, 0.003) == pytest.approx(1.5)
  assert h(25.0, 0.02) == pytest.approx(1.5)
  assert h(25.0, 0.00225) == pytest.approx(2.0)        # linear blend midpoint
  assert h(25.0, 0.0015) == pytest.approx(2.5)         # straight edge


def test_horizon_without_curvature_info_uses_speed():
  assert _anchor.arc_horizon(25.0, None) == pytest.approx(2.5)


def test_plan_curvature_ahead():
  m = _mv_plan(_circle_plan(0.0))
  m.orientationRate, m.velocity, _ = _rates(0.005)
  assert _anchor.plan_curvature_ahead(m) == pytest.approx(0.005)
  # a spike beyond the 2.5 s window is ignored
  m.orientationRate, m.velocity, _ = _rates(0.0, spike_at=5.0, spike=1.0)
  assert _anchor.plan_curvature_ahead(m) == pytest.approx(0.0)
  # inside the window it counts, either sign
  m.orientationRate, m.velocity, _ = _rates(0.0, spike_at=1.0, spike=-0.2)
  assert _anchor.plan_curvature_ahead(m) == pytest.approx(0.2 / 20.0)


def test_plan_curvature_ahead_missing_is_none():
  assert _anchor.plan_curvature_ahead(_mv_plan(_circle_plan(0.0))) is None
  assert _anchor.plan_curvature_ahead(None) is None


def test_auto_horizon_logged_and_shortened_in_curves():
  a = LaneAnchor(AnchorConfig())     # auto (default)
  m = _mv_plan(_circle_plan(0.0, v=25.0)); m.orientationRate, m.velocity, _ = _rates(0.0, v=25.0)
  _, t = a.update(0.0, m, 25.0, False)
  assert t['arc_h'] == pytest.approx(2.5)
  m = _mv_plan(_circle_plan(0.006, v=25.0)); m.orientationRate, m.velocity, _ = _rates(0.006, v=25.0)
  _, t = a.update(0.006, m, 25.0, False)
  assert t['arc_h'] == pytest.approx(1.5)
  assert t['k_ahead'] == pytest.approx(0.006)


def test_fixed_horizon_override():
  a = LaneAnchor(AnchorConfig(arc_horizon=2.0))
  m = _mv_plan(_circle_plan(0.006, v=25.0)); m.orientationRate, m.velocity, _ = _rates(0.006, v=25.0)
  _, t = a.update(0.006, m, 25.0, False)
  assert t['arc_h'] == pytest.approx(2.0)


# --- Lane geometry telemetry (2026-10-17) --------------------------------------

def test_lane_geometry_logged():
  """The lane lines reach the log whatever the state: they are how sway and
  drift are measured offline."""
  a = LaneAnchor(AnchorConfig())
  _, t = a.update(0.0, _mv(left_y=-1.6, right_y=1.9, left_p=0.9, right_p=0.8), 20.0, False)
  assert t['line_l'] == pytest.approx(-1.6) and t['line_r'] == pytest.approx(1.9)
  assert t['prob_l'] == pytest.approx(0.9) and t['prob_r'] == pytest.approx(0.8)


def test_lane_geometry_absent_lines_logged_as_zero_prob():
  a = LaneAnchor(AnchorConfig())
  _, t = a.update(0.0, SimpleNamespace(laneLines=[], laneLineProbs=[]), 20.0, False)
  assert t['prob_l'] == 0.0 and t['prob_r'] == 0.0


# --- Reference conditioner only (2026-10-20 audit, D3) -------------------------

def test_telemetry_exact_key_set():
  a = LaneAnchor(AnchorConfig())
  _, telem = a.update(0.002, _mv_plan(_circle_plan(0.006)), 20.0, False)
  assert set(telem) == {'kappa_in', 'kappa_ref', 'kappa_arc', 'arc_on', 'arc_h', 'k_ahead', 'v_ego',
                        'line_l', 'line_r', 'prob_l', 'prob_r'}


def test_update_has_no_lat_delay_parameter():
  import inspect
  assert 'lat_delay' not in inspect.signature(LaneAnchor.update).parameters


def test_config_has_only_the_two_fields():
  import dataclasses
  assert [f.name for f in dataclasses.fields(AnchorConfig)] == ['kappa_filter_tau', 'arc_horizon']
