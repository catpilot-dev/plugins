"""Tests for map_curve — the G/S map curve cap from mapd's road-ahead path."""
import math
import os
import sys

import pytest

_SLD_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if _SLD_DIR not in sys.path:
  sys.path.insert(0, _SLD_DIR)

import map_curve  # noqa: E402

# speedlimitd's constants, passed in exactly as speedlimitd.py passes them.
K = dict(max_lat_accel=1.5, comfort_brake=0.8, tight_curve_factor=0.75, curve_gate=0.003)

ORIGIN = (31.27, 121.64)
R_EARTH = 6371000.0


def _to_ll(x, y):
  lat = ORIGIN[0] + math.degrees(y / R_EARTH)
  lon = ORIGIN[1] + math.degrees(x / (R_EARTH * math.cos(math.radians(ORIGIN[0]))))
  return lat, lon


def _walk(segments, step):
  """Lay out [('S', length) | ('A', length, radius)] heading north, turning
  right on arcs. Returns local (x, y) vertices every `step` m plus each
  segment end, and a dense (1 m) reference for placing the car."""
  x = y = 0.0
  hdg = 0.0                       # radians, 0 = north, clockwise positive
  pts = [(x, y)]
  dense = [(0.0, x, y, hdg)]
  s = 0.0
  for seg in segments:
    length = seg[1]
    radius = seg[2] if seg[0] == 'A' else None
    done = 0.0
    since = 0.0
    while done < length - 1e-9:
      d = min(1.0, length - done)
      if radius:
        hdg_mid = hdg + d / (2 * radius)
        x += d * math.sin(hdg_mid); y += d * math.cos(hdg_mid)
        hdg += d / radius
      else:
        x += d * math.sin(hdg); y += d * math.cos(hdg)
      done += d; since += d; s += d
      dense.append((s, x, y, hdg))
      if since >= step - 1e-9:
        pts.append((x, y)); since = 0.0
    if since > 0:
      pts.append((x, y))
  return pts, dense


def path_of(segments, step):
  pts, _ = _walk(segments, step)
  return [_to_ll(x, y) for x, y in pts]


def car_on(segments, s, lateral=0.0):
  _, dense = _walk(segments, 1.0)
  _, x, y, hdg = min(dense, key=lambda r: abs(r[0] - s))
  x -= lateral * math.cos(hdg); y += lateral * math.sin(hdg)   # offset to the left
  return _to_ll(x, y)


def straight(length):
  return [('S', length)]


def straight_then_arc(lead, radius, arc=300.0, tail=400.0):
  return [('S', lead), ('A', arc, radius), ('S', tail)]


def test_straight_road_no_cap():
  seg = straight(1000)
  r = map_curve.map_curve_cap(path_of(seg, 20), *car_on(seg, 0), **K)
  assert r.cap_kph == 0.0 and r.lat is None
  assert r.path_ahead_m == pytest.approx(800, abs=1)


def test_arc_ahead_matches_closed_form():
  # 400 m straight, then R=300 m arc; car at s=0. κ=1/300 < 0.02 → no derate.
  seg = straight_then_arc(400, radius=300)
  r = map_curve.map_curve_cap(path_of(seg, 10), *car_on(seg, 0), **K)
  v_curve = math.sqrt(1.5 * 300)
  expected = math.sqrt(v_curve ** 2 + 2 * 0.8 * r.dist_m) * 3.6
  assert r.cap_kph == pytest.approx(expected, rel=0.02)
  assert r.kappa == pytest.approx(1 / 300, rel=0.1)
  assert 375 <= r.dist_m <= 450


def test_cap_tightens_as_curve_nears():
  seg = straight_then_arc(400, radius=300)
  path = path_of(seg, 10)
  caps = [map_curve.map_curve_cap(path, *car_on(seg, s), **K).cap_kph for s in (0, 200, 380)]
  assert caps[0] > caps[1] > caps[2]


def test_far_sharp_beats_near_mild_when_tighter():
  # mild R=800 at 100 m vs sharp R=150 at 500 m → binding point is the sharp one
  seg = [('S', 100), ('A', 100, 800), ('S', 300), ('A', 150, 150), ('S', 200)]
  r = map_curve.map_curve_cap(path_of(seg, 10), *car_on(seg, 0), **K)
  assert r.kappa == pytest.approx(1 / 150, rel=0.15)


def test_works_above_100_kph_regime():
  # R=300 at 400 m: v_curve = 21.2 m/s, but the braking term over ~400 m puts
  # the cap NOW near 120 km/h — reported, not zeroed like the vision cap at ≥100.
  seg = straight_then_arc(400, radius=300)
  r = map_curve.map_curve_cap(path_of(seg, 10), *car_on(seg, 0), **K)
  assert r.cap_kph > 100.0


def test_points_behind_car_ignored():
  # sharp arc entirely behind the car, straight ahead → no cap
  seg = [('A', 150, 150), ('S', 700)]
  r = map_curve.map_curve_cap(path_of(seg, 10), *car_on(seg, 250), **K)
  assert r.cap_kph == 0.0


def test_car_past_path_end_is_none():
  seg = straight(200)
  car = _to_ll(0.0, 250.0)
  assert map_curve.map_curve_cap(path_of(seg, 20), *car, **K) is None


def test_coarse_node_arc_still_detected():
  # R=400 arc sampled every 60 m (sparse OSM nodes)
  seg = straight_then_arc(300, radius=400)
  r = map_curve.map_curve_cap(path_of(seg, 60), *car_on(seg, 0), **K)
  assert r.kappa == pytest.approx(1 / 400, rel=0.3) and r.cap_kph > 0


def test_single_node_kink_on_straight_does_not_bind():
  # one vertex displaced 3 m laterally on an otherwise straight 1 km road
  pts, _ = _walk(straight(1000), 20)
  pts = [(x + 3.0, y) if abs(y - 300) < 1e-6 else (x, y) for x, y in pts]
  path = [_to_ll(x, y) for x, y in pts]
  for s in (0, 200, 280, 295):
    r = map_curve.map_curve_cap(path, *_to_ll(0.0, s), **K)
    assert r.cap_kph == 0.0, f'car at {s} m'


def test_lateral_distance_reported():
  seg = straight(1000)
  r = map_curve.map_curve_cap(path_of(seg, 20), *car_on(seg, 0, lateral=16.0), **K)
  assert r.lateral_m == pytest.approx(16.0, abs=0.5)


@pytest.mark.parametrize('pts', [
  [],
  [ORIGIN],
  [ORIGIN] * 5,
  [ORIGIN, (float('nan'), ORIGIN[1]), (ORIGIN[0] + 0.001, ORIGIN[1])],
])
def test_degenerate_paths_do_not_raise(pts):
  r = map_curve.map_curve_cap(pts, *ORIGIN, **K)
  assert r is None or (math.isfinite(r.cap_kph) and math.isfinite(r.lateral_m))


# ── gate ──────────────────────────────────────────────────────────────────────

OK = dict(gs_mode=True, gps_valid=True, tile_ref='S20', tile_ref_age_s=0.0, mapd_ok=True,
          mapd_sel_type='current', refs_match=True, path_ok=True, path_age_s=0.0)


def test_gate_all_pass_is_ok():
  assert map_curve.map_curve_gate(**OK) == 'ok'


@pytest.mark.parametrize('override,reason', [
  ({'gs_mode': False}, 'off_gs'), ({'gps_valid': False}, 'no_gps'),
  ({'tile_ref': ''}, 'tile_lapsed'), ({'mapd_ok': False}, 'mapd_absent'),
  ({'mapd_sel_type': 'extended'}, 'sel_type'), ({'mapd_sel_type': 'fail'}, 'sel_type'),
  ({'refs_match': False}, 'ref_mismatch'), ({'path_ok': False}, 'path_stale'),
  ({'path_age_s': 2.6}, 'path_stale')])
def test_gate_each_condition_alone(override, reason):
  assert map_curve.map_curve_gate(**{**OK, **override}) == reason


def test_gate_predicted_selection_passes():
  assert map_curve.map_curve_gate(**{**OK, 'mapd_sel_type': 'predicted'}) == 'ok'


def test_tile_blip_tolerance_is_2s():
  assert map_curve.map_curve_gate(**{**OK, 'tile_ref_age_s': 1.9}) == 'ok'
  assert map_curve.map_curve_gate(**{**OK, 'tile_ref_age_s': 2.1}) == 'tile_lapsed'


def test_refs_share_gs_multi_ref():
  gs = lambda r: r in ('S20', 'G1503', 'G2')
  assert map_curve.refs_share_gs('G1503;S20', 'S20', gs)
  assert map_curve.refs_share_gs('S20', 'G1503; S20', gs)
  assert not map_curve.refs_share_gs('G312;S20', 'G312', gs)   # shared part is not G/S
  assert not map_curve.refs_share_gs('S20', 'G2', gs)
  assert not map_curve.refs_share_gs('', 'S20', gs)
