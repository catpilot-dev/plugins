"""Map curve cap — a G/S-only curve speed limit from mapd's road-ahead path.

Pure and stateless: no I/O, no clock, no globals, and no import of speedlimitd.
speedlimitd passes in its own a_y target, braking deceleration, tight-curve
derate and straight-road gate, so this cap and the vision curve cap
(`curvature_speed_cap`) always share one budget.

The input is `MapdExtendedOut.path`: mapd's current way plus its predicted next
ways as (lat, lon) points in travel order. Only speedlimitd's G/S mode ever
calls this (see docs/superpowers/specs/2026-11-09-mapd-gs-curve-lookahead-design.md):
OSM geometry is trustworthy on G/S expressways and not on Chinese city roads.

Curvature is recomputed from the geometry rather than taken from mapd's
per-point `curvature`. mapd derives that value from consecutive OSM nodes, so a
single misplaced node reads as a sharp curve. Here a vertex's curvature is the
NET heading change across a window of `window_m` metres of path centred on it:
a one-node zig-zag turns one way and straight back, so it largely cancels,
while a real arc accumulates. The cancellation is partial for windows whose
edge falls inside the zig-zag, and with raw OSM segments that leak scales with
1/node-spacing: dense nodes (5–10 m, common at bridges and junctions) turned a
1–2 m misplaced node into a 50–70 km/h phantom cap. So the path is first
resampled at a fixed MAP_CURVE_RESAMPLE_M, which bounds the leak by the
resample spacing instead: a single node misplaced by up to ~3 m stays under
the 0.003 straight-road gate at any node density. A wider window would hide
more but loses sparse-node arcs (60 m spacing, R=400 reads as straight at
80 m), so 50 m is the compromise.
"""
import math
from bisect import bisect_right
from dataclasses import dataclass
from typing import Callable, Sequence

MAP_CURVE_WINDOW_M = 50.0
MAP_CURVE_MAX_LOOKAHEAD_M = 800.0
MAP_CURVE_RESAMPLE_M = 20.0
TILE_BLIP_TOLERANCE_S = 2.0   # the tile reader may lose the G/S ref this long
PATH_FRESH_S = 2.5            # mapdExtendedOut.position must change this often (1 Hz)

_R_EARTH = 6371000.0
_MIN_SEGMENT_M = 0.1
_END_MARGIN_M = 0.5
_BEHIND_START_M = 5.0   # car this far behind the path's first point → no path ahead


@dataclass(frozen=True)
class MapCurveResult:
  cap_kph: float          # 0.0 = no curve above the gate within the look-ahead
  dist_m: float           # along-path distance to the binding point (0.0 if none)
  kappa: float            # curvature at the binding point, 1/m (0.0 if none)
  lat: float | None       # binding point position (None if none)
  lon: float | None
  path_ahead_m: float     # path length ahead of the car, capped at max_lookahead_m
  lateral_m: float        # car's perpendicular distance from the path


def _interp(x: float, xp: Sequence[float], fp: Sequence[float]) -> float:
  if x <= xp[0]:
    return fp[0]
  if x >= xp[-1]:
    return fp[-1]
  for i in range(len(xp) - 1):
    if xp[i] <= x <= xp[i + 1]:
      f = (x - xp[i]) / (xp[i + 1] - xp[i])
      return fp[i] + f * (fp[i + 1] - fp[i])
  return fp[-1]


def _wrap_pi(a: float) -> float:
  return (a + math.pi) % (2.0 * math.pi) - math.pi


def map_curve_cap(points: Sequence[tuple[float, float]], car_lat: float, car_lon: float, *,
                  max_lat_accel: float, comfort_brake: float, tight_curve_factor: float,
                  curve_gate: float, window_m: float = MAP_CURVE_WINDOW_M,
                  max_lookahead_m: float = MAP_CURVE_MAX_LOOKAHEAD_M) -> MapCurveResult | None:
  """The lowest speed allowed now so `comfort_brake` still makes every curve ahead.

  Per vertex ahead of the car with curvature κ ≥ curve_gate:
    target_ay = max_lat_accel · interp(κ, [0.02, 0.035], [1, tight_curve_factor])
    v_now     = √(target_ay/κ + 2·comfort_brake·d)
  The cap is the minimum v_now, in km/h, unrounded and with no 100 km/h cut-off.

  Returns None when the car cannot be placed on the path with path ahead of it.
  """
  if not (math.isfinite(car_lat) and math.isfinite(car_lon)):
    return None
  cos_lat = math.cos(math.radians(car_lat))

  # Local metres around the car; drop non-finite points and duplicates.
  xy: list[tuple[float, float]] = []
  ll: list[tuple[float, float]] = []
  for lat, lon in points:
    if not (math.isfinite(lat) and math.isfinite(lon)):
      continue
    x = math.radians(lon - car_lon) * _R_EARTH * cos_lat
    y = math.radians(lat - car_lat) * _R_EARTH
    if xy and math.hypot(x - xy[-1][0], y - xy[-1][1]) < _MIN_SEGMENT_M:
      continue
    xy.append((x, y))
    ll.append((lat, lon))
  if len(xy) < 2:
    return None

  # Cumulative arc length of the raw polyline.
  cum = [0.0]
  for (x0, y0), (x1, y1) in zip(xy, xy[1:]):
    cum.append(cum[-1] + math.hypot(x1 - x0, y1 - y0))
  total = cum[-1]

  # Project the car (at the origin) onto the polyline.
  best_d = math.inf
  s_car = 0.0
  behind_start = 0.0
  for k, ((x0, y0), (x1, y1)) in enumerate(zip(xy, xy[1:])):
    dx, dy = x1 - x0, y1 - y0
    seg_len2 = dx * dx + dy * dy
    t_raw = (-x0 * dx - y0 * dy) / seg_len2
    t = max(0.0, min(1.0, t_raw))
    d = math.hypot(x0 + t * dx, y0 + t * dy)
    if d < best_d:
      best_d = d
      s_car = cum[k] + t * (cum[k + 1] - cum[k])
      behind_start = -t_raw * math.sqrt(seg_len2) if k == 0 and t_raw < 0 else 0.0
  if s_car >= total - _END_MARGIN_M or behind_start > _BEHIND_START_M:
    return None

  # Resample at a fixed spacing (see module docstring) and take bearings there.
  def point_at(s: float) -> tuple[float, float]:
    k = min(max(bisect_right(cum, s) - 1, 0), len(xy) - 2)
    f = (s - cum[k]) / (cum[k + 1] - cum[k])
    return (xy[k][0] + f * (xy[k + 1][0] - xy[k][0]),
            xy[k][1] + f * (xy[k + 1][1] - xy[k][1]))

  n = max(int(total // MAP_CURVE_RESAMPLE_M), 1)
  rs = [i * MAP_CURVE_RESAMPLE_M for i in range(n + 1)]
  if total - rs[-1] > _MIN_SEGMENT_M:
    rs.append(total)
  rxy = [point_at(v) for v in rs]
  heading = [math.atan2(x1 - x0, y1 - y0) for (x0, y0), (x1, y1) in zip(rxy, rxy[1:])]

  def chord_at(s: float) -> int:
    return min(max(bisect_right(rs, s) - 1, 0), len(heading) - 1)

  def chord_mid(k: int) -> float:
    return (rs[k] + rs[k + 1]) / 2.0

  def to_ll(x: float, y: float) -> tuple[float, float]:
    return (car_lat + math.degrees(y / _R_EARTH),
            car_lon + math.degrees(x / (_R_EARTH * cos_lat)))

  half = window_m / 2.0
  horizon = s_car + max_lookahead_m
  cap_ms = None
  bind = (0.0, 0.0, None, None)
  for i in range(1, len(rxy) - 1):
    s_i = rs[i]
    if s_i <= s_car:
      continue
    if s_i > horizon:
      break
    h = min(half, s_i, total - s_i)
    if h < window_m / 4.0:
      continue
    # A chord's bearing is the tangent at its midpoint, so divide the heading
    # change by the midpoints' separation, not by the window width.
    k_lo, k_hi = chord_at(s_i - h), chord_at(s_i + h)
    span = chord_mid(k_hi) - chord_mid(k_lo)
    if span <= 0.0:
      continue
    kappa = abs(_wrap_pi(heading[k_hi] - heading[k_lo])) / span
    if kappa < curve_gate:
      continue
    target_ay = max_lat_accel * _interp(kappa, [0.02, 0.035], [1.0, tight_curve_factor])
    d_i = s_i - s_car
    v_now = math.sqrt(target_ay / kappa + 2.0 * comfort_brake * d_i)
    if cap_ms is None or v_now < cap_ms:
      cap_ms = v_now
      bind = (d_i, kappa, *to_ll(*rxy[i]))

  path_ahead = min(total - s_car, max_lookahead_m)
  if cap_ms is None:
    return MapCurveResult(0.0, 0.0, 0.0, None, None, path_ahead, best_d)
  return MapCurveResult(cap_ms * 3.6, bind[0], bind[1], bind[2], bind[3], path_ahead, best_d)


def refs_share_gs(a: str, b: str, is_gs: Callable[[str], bool]) -> bool:
  """True iff some ';'-separated part (whitespace stripped) is in both refs and is G/S.

  OSM joins concurrent designations with ';' (e.g. 'G1503;S20'), and mapd and
  the tile reader need not list them alike. A shared part that is NOT a G/S
  expressway (the guodao in 'G312;S20' vs a bare 'G312') does not count.
  """
  parts_a = {p.strip() for p in a.split(';') if p.strip()}
  parts_b = {p.strip() for p in b.split(';') if p.strip()}
  return any(is_gs(p) for p in parts_a & parts_b)


def map_curve_gate(*, gs_mode: bool, gps_valid: bool, tile_ref: str, tile_ref_age_s: float,
                   mapd_ok: bool, mapd_sel_type: str, refs_match: bool,
                   path_ok: bool, path_age_s: float) -> str:
  """'ok', or the first failing reason, in the order the checks are listed.

  The cap may only be computed while the tile reader puts the car on a G/S
  expressway (within TILE_BLIP_TOLERANCE_S) AND mapd is matched to that same
  road AND mapd's path is fresh: a path mapd built along a ramp or cross
  street must never yield a G/S cap.
  """
  if not gs_mode:
    return 'off_gs'
  if not gps_valid:
    return 'no_gps'
  if not tile_ref or tile_ref_age_s > TILE_BLIP_TOLERANCE_S:
    return 'tile_lapsed'
  if not mapd_ok:
    return 'mapd_absent'
  if mapd_sel_type not in ('current', 'predicted'):
    return 'sel_type'
  if not refs_match:
    return 'ref_mismatch'
  if not path_ok or path_age_s > PATH_FRESH_S:
    return 'path_stale'
  return 'ok'
