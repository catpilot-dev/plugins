# mapd G/S Curve Look-ahead Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** On G/S expressways, compute a map-based curve speed cap from mapd's road-ahead path, always publish it in `speedLimitState` telemetry, and let it enter the speed decision only when a new Driving-panel toggle (default off) is on.

**Architecture:** mapd's `mapdExtendedOut` (1 Hz, carries `path`) is registered as a service at mapd's 2 MB queue size. A new pure module `plugins/speedlimitd/map_curve.py` holds the path projection, curvature, cap and gate logic. `speedlimitd.py` subscribes conditionally, tracks freshness state, calls the module after `gs_mode` is decided, publishes `mapCurve*` telemetry, and appends the cap to `candidates` as source 4 only when the toggle is on. Road identity, G/S mode, and every existing cap are untouched.

**Tech Stack:** Python 3.12, pytest, openpilot SubMaster/msgq, pycapnp (device only), raylib UI (`plugins/ui_mod`).

**Spec:** `docs/superpowers/specs/2026-11-09-mapd-gs-curve-lookahead-design.md` (commit `87890c7`). Read it before Task 1.

Run every test with the catpilot venv from the repo root:
`cd /home/oxygen/catpilot-dev/plugins && ../catpilot/.venv/bin/python -m pytest -q -p no:cacheprovider <path>`

## Global Constraints

- `mapdExtendedOut` service tuple: `[false, 1.0, null, 2097152]` — `should_log` false, frequency 1.0, decimation null, queue size 2097152 (mapd's `QUEUE_SIZE_MEDIUM`).
- G/S only: the map cap is never computed when `gs_mode` is false.
- Tile-reader blip tolerance: 2.0 s (`TILE_BLIP_TOLERANCE_S`).
- Path freshness: `mapdExtendedOut.position` must have changed within 2.5 s (`PATH_FRESH_S`); judge by content, not receive time.
- a_y target is the existing `self.curve_target_lat_accel` (param `MapdCurveTargetLatAccel`, default 1.5); braking is the existing `COMFORT_BRAKE` (0.8); tight-curve derate is the existing `TIGHT_CURVE_FACTOR` (0.75) over κ ∈ [0.02, 0.035]; straight-gate κ < 0.003. Pass these in from `speedlimitd.py`; do not copy their values into `map_curve.py`.
- No 100 km/h cut-off on the map cap.
- Toggle param: `MapdCurveLookahead`, speedlimitd plugin param, bool, default off. With it off, actuation must be byte-identical to today.
- When on: append to `candidates` as `(float(max(cap, MIN_SPEED_LIMIT)), 4, 0.7)` — safety class, same as the vision curve cap.
- Telemetry keys (exact): `mapCurveCap`, `mapCurveActive`, `mapCurveDist`, `mapCurveKappa`, `mapCurveLatLon`, `mapCurvePathAhead`, `mapCurveLat`, `mapCurveGate`.
- Gate reasons (exact strings): `ok`, `off_gs`, `no_gps`, `tile_lapsed`, `mapd_absent`, `sel_type`, `ref_mismatch`, `path_stale`, `no_path_ahead`, `error`.
- Nothing from the map cap may raise into `update()`.
- `mapd_source.py` and `map_curve.py` stay pure: no I/O, no clock, no globals, no import of `speedlimitd`.

## Review Focus

1. **Multi-ref strings** — tile reader `G1503;S20` while mapd says `S20` (or the reverse) must count as the same road; `G312;S20` vs a bare `G312` must not. → Task 3, `test_refs_share_gs_*`.
2. **A resent stale path** — mapd keeps publishing but `position` is frozen (mapd wedged or GPS lost on its side): gate must go `path_stale` after 2.5 s even though messages keep arriving. → Task 4, `test_frozen_path_position_goes_stale`.
3. **Car before the path start or past its end** — a path that starts ahead of the car, or that the car has driven off the end of, must give `no_path_ahead`, not a cap from points behind the car. → Task 2, `test_car_past_path_end_is_none` and `test_points_behind_car_ignored`.
4. **Degenerate geometry** — duplicate consecutive points, a single point, NaN lat/lon: no exception, no NaN in telemetry. → Task 2, `test_degenerate_paths_do_not_raise`.
5. **Toggle flipped mid-drive** — turning it off must remove the cap from `candidates` on the next `update()` after the 5 s param read, with no residual hold. → Task 5, `test_toggle_off_after_on_removes_cap`.

---

### Task 1: Register `mapdExtendedOut` in the mapd plugin

**Files:**
- Modify: `plugins/mapd/plugin.json` (`services`)
- Modify: `plugins/mapd/tests/test_manifest.py` (`TestService`)
- Modify: `.github/scripts/mapd_watch.py` (`_issue_body` checklist item on `ServiceQueueSize`)
- Modify: `.github/scripts/tests/test_mapd_watch.py` (`TestChecklist.test_checklist_carries_the_queue_size_check`)

**Interfaces:**
- Produces: service name `mapdExtendedOut` in `SERVICE_LIST` on device (consumed by Task 4).

- [ ] **Step 1: Write the failing tests**

In `TestService`:

```python
def test_extended_out_registered_at_mapds_queue_size(self):
    """mapd creates mapdExtendedOut at QUEUE_SIZE_MEDIUM (2 MB), same as
    mapdOut. A smaller registration shrinks mapd's shm file and SIGBUSes its
    publisher (README root-cause section)."""
    entry = _manifest()['services']['mapdExtendedOut']
    assert entry == [False, 1.0, None, 2 * 1024 * 1024]
```

In `TestChecklist`, extend the existing test:

```python
assert 'ServiceQueueSize["mapdExtendedOut"]' in body
```

- [ ] **Step 2: Run to verify they fail**

Run: `... -m pytest plugins/mapd/tests/test_manifest.py .github/scripts/tests/test_mapd_watch.py -q`
Expected: 2 failures (KeyError `mapdExtendedOut`; missing checklist text).

- [ ] **Step 3: Implement**

Add `"mapdExtendedOut": [false, 1.0, null, 2097152]` to `services` in `plugins/mapd/plugin.json`. In `mapd_watch.py`'s checklist item, change the sentence so it reads: check that `ServiceQueueSize["mapdOut"]` and `ServiceQueueSize["mapdExtendedOut"]` in upstream `settings/const.go` still equal the 4th element of the matching entries in `plugins/mapd/plugin.json` (2 MB).

- [ ] **Step 4: Run to verify they pass**

Run: `... -m pytest plugins/mapd/tests plugins/tests .github/scripts/tests -q`
Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add plugins/mapd/plugin.json plugins/mapd/tests/test_manifest.py .github/scripts/mapd_watch.py .github/scripts/tests/test_mapd_watch.py
git commit -m "mapd: register mapdExtendedOut at mapd's 2 MB queue size, unlogged"
```

---

### Task 2: `map_curve.py` — project the car onto the path and compute the cap

**Files:**
- Create: `plugins/speedlimitd/map_curve.py`
- Create: `plugins/speedlimitd/tests/test_map_curve.py`

**Interfaces:**
- Produces:
  ```python
  MAP_CURVE_WINDOW_M = 50.0
  MAP_CURVE_MAX_LOOKAHEAD_M = 800.0

  @dataclass(frozen=True)
  class MapCurveResult:
      cap_kph: float          # 0.0 = no curve above the gate within the look-ahead
      dist_m: float           # along-path distance to the binding point (0.0 if none)
      kappa: float            # curvature at the binding point, 1/m (0.0 if none)
      lat: float | None       # binding point position (None if none)
      lon: float | None
      path_ahead_m: float     # path length ahead of the car, capped at max_lookahead_m
      lateral_m: float        # car's perpendicular distance from the path

  def map_curve_cap(points: Sequence[tuple[float, float]], car_lat: float, car_lon: float, *,
                    max_lat_accel: float, comfort_brake: float, tight_curve_factor: float,
                    curve_gate: float, window_m: float = MAP_CURVE_WINDOW_M,
                    max_lookahead_m: float = MAP_CURVE_MAX_LOOKAHEAD_M) -> MapCurveResult | None
  ```
  `points` are `(lat, lon)` in travel order. Returns `None` when the car cannot be placed on the path with path ahead of it (empty path, fewer than 2 distinct points, car at or past the end).

**Algorithm (decided here; not determined by the signature):**
- Work in local metres around the car: `x = radians(lon - car_lon)·R·cos(radians(car_lat))`, `y = radians(lat - car_lat)·R`, `R = 6371000`. Drop consecutive duplicate points (segment length < 0.1 m) and points with non-finite coordinates first.
- Car position: nearest point on the polyline (per-segment perpendicular projection, `t` clamped to [0, 1]); `s_car` = arc length at that projection; `lateral_m` = its distance. If `s_car >= total_length - 0.5` → `None`.
- Curvature: **recompute from geometry; do not use mapd's per-point `curvature`.** For each path vertex with arc length `s_i` in `(s_car, s_car + max_lookahead_m]`: `κ_i = |wrap_pi(heading(s_i + W/2) − heading(s_i − W/2))| / W`, where `heading(s)` is the bearing of the segment containing arc length `s` and `W = window_m`. If either end of the window falls outside the path, shrink the window symmetrically to fit; skip the vertex if the fitted window is shorter than `W/2`. Net heading change over the window cancels a single-node zig-zag, which is why it is used.
- Per vertex with `κ_i >= curve_gate`: `target_ay = max_lat_accel · interp(κ_i, [0.02, 0.035], [1.0, tight_curve_factor])` (clamped linear), `v_curve = √(target_ay/κ_i)`, `d_i = s_i − s_car`, `v_now = √(v_curve² + 2·comfort_brake·d_i)`. Binding point = minimum `v_now`; `cap_kph = v_now·3.6`, unrounded, no 100 km/h cut-off, no floor.
- `path_ahead_m = min(total_length − s_car, max_lookahead_m)`.

- [ ] **Step 1: Write the failing tests** (`tests/test_map_curve.py`; build paths with a helper that lays points along a straight line or a circular arc in local metres and converts back to lat/lon around `(31.27, 121.64)`)

```python
K = dict(max_lat_accel=1.5, comfort_brake=0.8, tight_curve_factor=0.75, curve_gate=0.003)

def test_straight_road_no_cap():
    r = map_curve.map_curve_cap(straight(length=1000, step=20), *car_at(0), **K)
    assert r.cap_kph == 0.0 and r.lat is None and r.path_ahead_m == pytest.approx(800, abs=1)

def test_arc_ahead_matches_closed_form():
    # 400 m straight, then R=300 m arc; car at s=0. κ=1/300 < 0.02 → no derate.
    r = map_curve.map_curve_cap(straight_then_arc(400, radius=300, step=10), *car_at(0), **K)
    v_curve = math.sqrt(1.5 * 300)
    expected = math.sqrt(v_curve**2 + 2 * 0.8 * r.dist_m) * 3.6
    assert r.cap_kph == pytest.approx(expected, rel=0.02)
    assert r.kappa == pytest.approx(1/300, rel=0.1)
    assert 375 <= r.dist_m <= 425

def test_cap_tightens_as_curve_nears():
    caps = [map_curve.map_curve_cap(path, *car_at(s), **K).cap_kph for s in (0, 200, 380)]
    assert caps[0] > caps[1] > caps[2]

def test_far_sharp_beats_near_mild_when_tighter():
    # mild R=800 at 100 m vs sharp R=150 at 500 m → binding point is the sharp one
    r = map_curve.map_curve_cap(mild_then_sharp(), *car_at(0), **K)
    assert r.kappa == pytest.approx(1/150, rel=0.15)

def test_works_above_100_kph_regime():
    # R=1000 at 300 m: v_curve = 38.7 m/s = 139 km/h → cap > 100, still reported
    r = map_curve.map_curve_cap(straight_then_arc(300, radius=1000, step=10), *car_at(0), **K)
    assert r.cap_kph > 100.0

def test_points_behind_car_ignored():
    # sharp arc entirely behind the car, straight ahead → no cap
    r = map_curve.map_curve_cap(arc_then_straight(radius=150), *car_at_after_arc(), **K)
    assert r.cap_kph == 0.0

def test_car_past_path_end_is_none():
    assert map_curve.map_curve_cap(straight(length=200, step=20), *car_at(250), **K) is None

def test_coarse_node_arc_still_detected():
    # R=400 arc sampled every 60 m (sparse OSM nodes)
    r = map_curve.map_curve_cap(straight_then_arc(300, radius=400, step=60), *car_at(0), **K)
    assert r.kappa == pytest.approx(1/400, rel=0.3) and r.cap_kph > 0

def test_single_node_kink_on_straight_does_not_bind():
    # one vertex displaced 3 m laterally on an otherwise straight 1 km road
    r = map_curve.map_curve_cap(straight_with_kink(at=300, offset=3.0), *car_at(0), **K)
    assert r.cap_kph == 0.0

def test_lateral_distance_reported():
    r = map_curve.map_curve_cap(straight(length=1000, step=20), *car_at(0, lateral=16.0), **K)
    assert r.lateral_m == pytest.approx(16.0, abs=0.5)

@pytest.mark.parametrize('pts', [[], [(31.27, 121.64)], [(31.27, 121.64)] * 5,
                                 [(31.27, 121.64), (float('nan'), 121.64), (31.271, 121.64)]])
def test_degenerate_paths_do_not_raise(pts):
    r = map_curve.map_curve_cap(pts, 31.27, 121.64, **K)
    assert r is None or (math.isfinite(r.cap_kph) and math.isfinite(r.lateral_m))
```

- [ ] **Step 2: Run to verify they fail**

Run: `... -m pytest plugins/speedlimitd/tests/test_map_curve.py -q`
Expected: collection error / `ModuleNotFoundError: map_curve`.

- [ ] **Step 3: Implement `map_curve_cap` and `MapCurveResult` in `plugins/speedlimitd/map_curve.py`** per the algorithm above. Module docstring: pure, G/S-only consumer of `MapdExtendedOut.path`, constants passed in from speedlimitd, and why mapd's own curvature is not used.

- [ ] **Step 4: Run to verify they pass**

Run: `... -m pytest plugins/speedlimitd/tests/test_map_curve.py -q`
Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add plugins/speedlimitd/map_curve.py plugins/speedlimitd/tests/test_map_curve.py
git commit -m "speedlimitd: pure map curve cap from mapd's road-ahead path"
```

---

### Task 3: `map_curve.py` — the gate

**Files:**
- Modify: `plugins/speedlimitd/map_curve.py`
- Modify: `plugins/speedlimitd/tests/test_map_curve.py`

**Interfaces:**
- Produces:
  ```python
  TILE_BLIP_TOLERANCE_S = 2.0
  PATH_FRESH_S = 2.5

  def refs_share_gs(a: str, b: str, is_gs: Callable[[str], bool]) -> bool
      # True iff some ';'-split, whitespace-stripped part is present in both and is_gs(part)

  def map_curve_gate(*, gs_mode: bool, gps_valid: bool, tile_ref: str, tile_ref_age_s: float,
                     mapd_ok: bool, mapd_sel_type: str, refs_match: bool,
                     path_ok: bool, path_age_s: float) -> str
  ```
  Checks in this order, returning the first failing reason: `off_gs` (not gs_mode), `no_gps`, `tile_lapsed` (tile_ref empty or age > `TILE_BLIP_TOLERANCE_S`), `mapd_absent` (not mapd_ok), `sel_type` (not in `('current', 'predicted')`), `ref_mismatch` (not refs_match), `path_stale` (not path_ok or age > `PATH_FRESH_S`); else `ok`.

- [ ] **Step 1: Write the failing tests**

```python
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
```

- [ ] **Step 2: Run to verify they fail**

Run: `... -m pytest plugins/speedlimitd/tests/test_map_curve.py -q`
Expected: `AttributeError: ... map_curve_gate`.

- [ ] **Step 3: Implement `refs_share_gs`, `map_curve_gate` and the two constants in `map_curve.py`.**

- [ ] **Step 4: Run to verify they pass** — same command, all pass.

- [ ] **Step 5: Commit**

```bash
git add plugins/speedlimitd/map_curve.py plugins/speedlimitd/tests/test_map_curve.py
git commit -m "speedlimitd: map curve gate — G/S, tile reader within 2 s, mapd on the same ref, fresh path"
```

---

### Task 4: Wire the map cap into speedlimitd — telemetry only

**Files:**
- Modify: `plugins/speedlimitd/speedlimitd.py` (`SpeedLimitMiddleware.__init__` subscription block ~:735; new state in `__init__`; new method; call site after `gs_mode = not gs_released` ~:1553; `self._sl_pub.send({...})` ~:1674)
- Modify: `plugins/speedlimitd/tests/test_speedlimitd.py` (`mock_openpilot` fixture `SERVICE_LIST`; new `class TestMapCurveTelemetry`)

**Interfaces:**
- Consumes: `map_curve.map_curve_cap`, `map_curve.map_curve_gate`, `map_curve.refs_share_gs`, `MapCurveResult` (Tasks 2–3); `is_gs_expressway_ref`; `COMFORT_BRAKE`, `TIGHT_CURVE_FACTOR`.
- Produces (used by Task 5):
  - `self._map_curve_service_available: bool`
  - `self._map_curve_gate: str` and `self._map_curve_result: MapCurveResult | None`, both refreshed every `update()`
  - `def _update_map_curve(self, now: float, gs_mode: bool) -> None`
  - module constant `MAP_CURVE_GATE_KAPPA = 0.003` (replace the local `CURVE_GATE = 0.003` inside `curvature_speed_cap` with it, so both caps share one value)

- [ ] **Step 1: Write the failing tests**

Add `'mapdExtendedOut': MagicMock()` to the fixture's `SERVICE_LIST`. New class (reuse the `_mw`/`_hold_s1` pattern from `TestGsDistanceGuardedRelease` — copy those two helpers into the class or factor them to module level):

```python
def test_extended_out_subscribed_when_service_present(self, sld):
    mw = sld.SpeedLimitMiddleware()
    assert 'mapdExtendedOut' in sld.messaging.SubMaster.call_args[0][0]

def test_builds_without_extended_service(self, sld):
    services = sys.modules['openpilot.cereal.services']
    services.SERVICE_LIST = {k: v for k, v in services.SERVICE_LIST.items() if k != 'mapdExtendedOut'}
    mw = sld.SpeedLimitMiddleware()
    assert mw._map_curve_service_available is False
    sent = {}; mw._sl_pub.send = lambda p: sent.update(p); mw.update()
    assert sent['mapCurveGate'] == 'mapd_absent' and sent['mapCurveCap'] == 0

def test_telemetry_keys_always_published(self, sld):
    # every key from Global Constraints, off G/S
    ...  # update() once; assert all 8 keys present; mapCurveGate == 'off_gs'

def test_cap_computed_on_gs_with_curved_path(self, sld, monkeypatch):
    # S20 hold via _hold_s1-style ingest with wayRef='S20'; mapdOut stub wayRef='S20',
    # waySelectionType='current', alive/valid True; mapdExtendedOut stub with a
    # straight_then_arc path (R=300 at 400 m) and a position that changes each call;
    # _gps_valid True at the path start.
    ...  # assert published mapCurveGate == 'ok' and mapCurveCap > 0 and mapCurveDist > 0

def test_toggle_off_actuation_identical(self, sld, monkeypatch):
    # Load-bearing. Same G/S scenario run twice from identical state: once with the
    # curved path, once with mapdExtendedOut absent from sm. Assert speedLimit, source,
    # confidence and every key of the payload EXCEPT the mapCurve* keys are equal.

def test_frozen_path_position_goes_stale(self, sld, monkeypatch):
    # path message keeps arriving (sm.updated True) but position unchanged:
    # at t+2.4 s gate 'ok', at t+2.6 s gate 'path_stale'.

def test_tile_lapse_closes_gate_after_2s(self, sld, monkeypatch):
    # after an S20 tile match, ingest a blank result (no wayRef): gate stays 'ok'
    # at +1.9 s and becomes 'tile_lapsed' at +2.1 s (gs_mode still held).

def test_cap_function_exception_is_contained(self, sld, monkeypatch):
    monkeypatch.setattr(sld.map_curve, 'map_curve_cap', MagicMock(side_effect=ValueError))
    # update() does not raise; mapCurveGate == 'error'; speedLimit unchanged vs baseline
```

- [ ] **Step 2: Run to verify they fail**

Run: `... -m pytest plugins/speedlimitd/tests/test_speedlimitd.py -q -k MapCurve`
Expected: failures (`KeyError 'mapCurveGate'`, missing attribute).

- [ ] **Step 3: Implement in `speedlimitd.py`**

- `import map_curve` beside `import mapd_source` (bare sibling import, same as today).
- Subscription: `self._map_curve_service_available = 'mapdExtendedOut' in SERVICE_LIST`; append to `_subs` when true.
- State in `__init__`: `self._mc_tile_ref = ''`, `self._mc_tile_ref_t = 0.0`, `self._mc_path_pos = None`, `self._mc_path_pos_t = 0.0`, `self._map_curve_gate = 'off_gs'`, `self._map_curve_result = None`, `self._map_curve_error_logged = False`.
- `_update_map_curve(now, gs_mode)`, called right after `gs_mode = not gs_released`, whole body in `try/except Exception` → gate `error`, result `None`, `cloudlog.warning` once:
  - if `is_gs_expressway_ref(self.last_way_ref)`: `_mc_tile_ref, _mc_tile_ref_t = self.last_way_ref, now`.
  - `mapd_ok` = service available and `sm.alive`/`sm.valid` for `mapdOut`; `mapd_ref`, `sel` from `self.sm['mapdOut']` (`str(waySelectionType)`).
  - path: if extended service available and alive/valid, read `self.sm['mapdExtendedOut']`; when `(position.latitude, position.longitude)` differs from `_mc_path_pos`, store it and `_mc_path_pos_t = now`.
  - `gate = map_curve.map_curve_gate(...)` with `tile_ref_age_s = now - _mc_tile_ref_t`, `refs_match = map_curve.refs_share_gs(mapd_ref, _mc_tile_ref, is_gs_expressway_ref)`, `path_age_s = now - _mc_path_pos_t`.
  - when `ok`: `map_curve.map_curve_cap([(p.latitude, p.longitude) for p in path], self._gps_lat, self._gps_lon, max_lat_accel=self.curve_target_lat_accel, comfort_brake=COMFORT_BRAKE, tight_curve_factor=TIGHT_CURVE_FACTOR, curve_gate=MAP_CURVE_GATE_KAPPA)`; `None` → gate `no_path_ahead`.
- Publish (inside the existing `send({...})` dict, after `**self._mapd_telemetry`): `mapCurveCap` (int, rounded; 0 unless gate ok and result has a cap), `mapCurveActive` (`self.map_curve_enabled` — add the attribute now as `False`; Task 5 wires the param), `mapCurveDist` (1 dp), `mapCurveKappa` (5 dp), `mapCurveLatLon` (`[lat, lon]` 6 dp, or `[]`), `mapCurvePathAhead` (int), `mapCurveLat` (1 dp), `mapCurveGate`.
- Do NOT touch `candidates`.

- [ ] **Step 4: Run to verify they pass**

Run: `... -m pytest plugins/speedlimitd -q`
Expected: all pass, including the existing `TestMapdPhase1Telemetry` containment test.

- [ ] **Step 5: Commit**

```bash
git add plugins/speedlimitd/speedlimitd.py plugins/speedlimitd/tests/test_speedlimitd.py
git commit -m "speedlimitd: compute and publish the G/S map curve cap, telemetry only"
```

---

### Task 5: Toggle, actuation, Driving panel, docs

**Files:**
- Modify: `plugins/speedlimitd/plugin.json` (`params`)
- Modify: `plugins/speedlimitd/speedlimitd.py` (`_read_params` ~:909; `candidates` block ~:1604)
- Modify: `plugins/ui_mod/driving_panel.py` (after the "Mapd/OSM Data Integration" item ~:184; new callback beside `_on_osm_integration` ~:256)
- Modify: `plugins/speedlimitd/tests/test_speedlimitd.py` (new `class TestMapCurveToggle`)
- Create: `plugins/ui_mod/tests/test_driving_panel_map_curve.py`
- Modify: `docs/superpowers/specs/2026-08-18-mapd-integration-design.md`, `plugins/mapd/README.md`, `plugins/speedlimitd/mapd_source.py` (docstring)

**Interfaces:**
- Consumes: `self._map_curve_gate`, `self._map_curve_result`, `self.map_curve_enabled` (Task 4).
- Produces: param `MapdCurveLookahead`.

- [ ] **Step 1: Write the failing tests**

```python
class TestMapCurveToggle:
  def test_param_declared_default_off(self):
      p = json.load(open(MANIFEST))['params']['MapdCurveLookahead']
      assert p['type'] == 'bool' and p['default'] is False

  def test_read_params_wires_toggle(self, sld, monkeypatch):
      # read_plugin_param → '1' for MapdCurveLookahead → mw.map_curve_enabled True; '' → False

  def test_toggle_on_lowers_limit_as_source_4(self, sld, monkeypatch):
      # Task 4's G/S curved-path scenario with map_curve_enabled True and a cap below
      # the G/S limit → published speedLimit == snap of the cap path, source == 4

  def test_cap_above_limit_changes_nothing(self, sld, monkeypatch):
      # R=1000 arc (cap > 100) with S20 at 100 → speedLimit, source equal to toggle-off run

  def test_toggle_has_no_effect_off_gs(self, sld, monkeypatch):
      # non-G/S road, toggle on, curved path present → identical to toggle off

  def test_toggle_off_after_on_removes_cap(self, sld, monkeypatch):
      # on → source 4; param read returns '' at the next 5 s read → next update() source != 4

  def test_toggle_on_without_extended_service_changes_nothing(self, sld, monkeypatch):
      # SERVICE_LIST without mapdExtendedOut, toggle on, G/S scenario →
      # identical speedLimit/source to toggle off; mapCurveGate == 'mapd_absent'
```

`plugins/ui_mod/tests/test_driving_panel_map_curve.py` (source inspection; the panel needs raylib):

```python
def test_toggle_writes_param():
    src = open(DRIVING_PANEL).read()
    assert "read_plugin_param('speedlimitd', 'MapdCurveLookahead')" in src
    assert "write_plugin_param('speedlimitd', 'MapdCurveLookahead'" in src
    assert '"Map Curve Look-ahead (G/S)"' in src
```

- [ ] **Step 2: Run to verify they fail**

Run: `... -m pytest plugins/speedlimitd plugins/ui_mod/tests -q -k "MapCurveToggle or map_curve"`
Expected: failures.

- [ ] **Step 3: Implement**

- `plugin.json` params: `"MapdCurveLookahead": {"type": "bool", "default": false, "label": "Map Curve Look-ahead (G/S)", "desc": "Slow down for curves ahead on G/S expressways using mapd's map path"}`.
- `_read_params`: `self.map_curve_enabled = read_plugin_param('speedlimitd', 'MapdCurveLookahead', '') == '1'`.
- `candidates`: after the reactive cap append, `if self.map_curve_enabled and self._map_curve_gate == 'ok' and self._map_curve_result is not None and self._map_curve_result.cap_kph > 0: candidates.append((float(max(round(self._map_curve_result.cap_kph), MIN_SPEED_LIMIT)), 4, 0.7))`.
- `driving_panel.py`: a `toggle_item` titled `"Map Curve Look-ahead (G/S)"`, description exactly: `"Slow down for curves ahead on G/S expressways using mapd's map path, beyond what the camera can see. Off: the map cap is computed and logged only."`, initial state `read_plugin_param('speedlimitd', 'MapdCurveLookahead') == '1'`, callback `_on_map_curve_lookahead(self, state)` writing `'1'`/`'0'`. Same conditional block as the OSM toggle.
- Docs: in `2026-08-18-mapd-integration-design.md` add one line under "### Phase 2 — cutover and deletion": superseded by `2026-11-09-mapd-gs-curve-lookahead-design.md`. In `plugins/mapd/README.md`, replace "Phase 2 … is NOT started" with Phase 2 = G/S curve look-ahead, toggle `MapdCurveLookahead`, tile stack kept. In `mapd_source.py`'s docstring, drop the `result_from_mapd()` paragraph.

- [ ] **Step 4: Run the full suite**

Run: `... -m pytest -q -p no:cacheprovider --ignore=plugins/bmw_e9x_e8x/safety/tests/test_bmw.py`
Expected: only the 17 pre-existing `plugins/bmw_e9x_e8x/tests/test_integration.py` failures; everything else passes.

- [ ] **Step 5: Commit**

```bash
git add plugins/speedlimitd plugins/ui_mod docs/superpowers/specs/2026-08-18-mapd-integration-design.md plugins/mapd/README.md
git commit -m "speedlimitd: Map Curve Look-ahead toggle lets the G/S map cap act as a source-4 cap"
```

---

### Task 6: On-device verification (deploy only with the user's go-ahead)

**Files:** none.

- [ ] **Step 1: Ask the user before pushing and rebooting the C3.** Deploy is: push `rebase/v0.11.2`; on the C3 `cd /data/plugins && git fetch origin && git reset --hard origin/rebase/v0.11.2`; confirm offroad (`/data/params/d/IsOnroad` empty); `sudo reboot`.
- [ ] **Step 2: Verify the service.** `grep mapdExtendedOut /data/openpilot/openpilot/cereal/services.py` shows two lines `(False, 1.0, None, 2097152)`; `grep mapdExtendedOut /data/openpilot/openpilot/cereal/services.h` shows `2097152`; `stat -c %s /dev/shm/msgq_mapdExtendedOut` = `2097536`.
- [ ] **Step 3: Verify stability.** `pidof mapd` unchanged over 4 minutes (`ps -o etimes=`), `/dev/shm/msgq_mapdOut` still `2097536`, no `panic|sigbus` in `/tmp/plugin_logs/plugin_mapd.log` or `plugin_speedlimitd.log`.
- [ ] **Step 4: Verify the toggle.** Driving panel shows "Map Curve Look-ahead (G/S)", off by default; flipping it writes `/data/plugins-runtime/speedlimitd/data/MapdCurveLookahead`.
- [ ] **Step 5: After the next G/S drive**, the route's `speedLimitState` rows carry the 8 `mapCurve*` keys, with `mapCurveGate == 'ok'` on S20 stretches and `off_gs` elsewhere.
