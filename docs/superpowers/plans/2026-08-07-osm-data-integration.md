# Mapd/OSM Data Integration Toggle — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a Driving-panel toggle "Mapd/OSM Data Integration" that lets speedlimitd use OSM maxspeed as its base speed-limit source (default OFF in China, ON elsewhere), with a yellow missing-tiles warning.

**Architecture:** speedlimitd already queries offline OSM capnp tiles (`osm_query.py`) and discards the returned `speedLimit`. This plan stores it, gates it behind a param, and selects it as the base inference ahead of the vision path. The mapd Go binary is NOT involved. ui_mod renders the toggle + warning; tile downloads are COD's job (out of scope).

**Tech Stack:** Python, pytest (repo harness in `plugins/speedlimitd/tests/`), openpilot list_view widgets (raylib) for UI.

**Spec:** `docs/superpowers/specs/2026-08-07-osm-data-integration-design.md`

## Global Constraints

- Vision inference is always the fallback; toggle OFF ⇒ behavior byte-identical to today.
- Safety caps (YOLO, proactive curvature cap, reactive a_y cap) always `min()` over the base — OSM must never lift them.
- Param `OsmDataIntegration` lives in speedlimitd's plugin data dir (`config.write_plugin_param('speedlimitd', ...)`) — NEVER `/data/params/d/`.
- Region default: first GPS fix, `country_from_gps` == `'cn'` → `'0'`, anything else (incl. None) → `'1'`; never overwrite an existing param file.
- OSM-sourced display values bypass `snap_to_standard_speed` (CN ladder `[30,40,50,60,80,100,120]` would round a US 45 mph / 72 km/h limit UP to 80): round to nearest 5 km/h, step ±10 km/h at existing intervals.
- All UI imports in ui_mod stay module-internal to `driving_panel.py` (module itself is lazily imported by hooks).
- Tests run with `PYTHONPATH=. .venv/bin/python3 -m pytest plugins/<plugin>/tests/ -q --tb=short` from the repo root (matches the pre-push hook).
- No `Co-Authored-By` lines in commits.
- Implementation deviation from spec §3a (approved direction, simpler mechanism): the Driving panel reads the precise missing-tiles signal from a persisted plugin param `OsmTilesMissing` written by speedlimitd on state change, instead of a live plugin-bus read. The `osmTilesMissing` bus field is still published for telemetry/rlog.

---

### Task 1: `tile_missing` flag in osm_query

**Files:**
- Modify: `plugins/speedlimitd/osm_query.py`
- Test: `plugins/speedlimitd/tests/test_osm_query.py`

**Interfaces:**
- Produces: `OsmTileReader.tile_missing: bool` — True iff the most recent `query()` found NEITHER the `offline_hw` nor the `offline` tile file on disk for the queried position. A tile that exists but is still loading in the background is NOT missing.

- [ ] **Step 1: Write the failing tests**

Append to `plugins/speedlimitd/tests/test_osm_query.py` (reuse the existing `osm_query` fixture and `_write_tile` helper already in that file):

```python
class TestTileMissing:
  """OsmTileReader.tile_missing — missing-tiles signal for the UI warning."""

  def test_no_tile_file_sets_flag(self, osm_query):
    reader = osm_query.OsmTileReader()
    assert reader.tile_missing is False  # init default
    assert reader.query(31.0, 121.0) is None
    assert reader.tile_missing is True

  def test_present_tile_clears_flag_even_while_loading(self, osm_query):
    reader = osm_query.OsmTileReader()
    path = osm_query._tile_path(31.0, 121.0)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    _write_tile(reader.schema, path, [
      {'name': 'Test Rd', 'maxSpeed': 22.2,
       'nodes': [(30.999, 120.999), (31.001, 121.001)]},
    ])
    # First query kicks off a background load — result may be None, but the
    # tile file EXISTS, so tile_missing must be False.
    reader.query(31.0, 121.0)
    assert reader.tile_missing is False

  def test_flag_recovers_after_tile_appears(self, osm_query):
    reader = osm_query.OsmTileReader()
    reader.query(31.0, 121.0)
    assert reader.tile_missing is True
    path = osm_query._tile_path(31.0, 121.0)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    _write_tile(reader.schema, path, [
      {'name': 'Test Rd', 'maxSpeed': 22.2,
       'nodes': [(30.999, 120.999), (31.001, 121.001)]},
    ])
    reader.query(31.0, 121.0)
    assert reader.tile_missing is False
```

If `_write_tile`'s signature in the file differs (check its definition), adapt the calls — the way dicts must match what it accepts.

- [ ] **Step 2: Run tests to verify they fail**

Run: `PYTHONPATH=. .venv/bin/python3 -m pytest plugins/speedlimitd/tests/test_osm_query.py::TestTileMissing -v`
Expected: FAIL — `AttributeError: 'OsmTileReader' object has no attribute 'tile_missing'`

- [ ] **Step 3: Implement**

In `plugins/speedlimitd/osm_query.py`, `OsmTileReader.__init__`, after `self._loading = set()`:

```python
    self.tile_missing = False  # last query found no tile file at all (UI warning)
```

In `query()`, replace the current path-selection block:

```python
    # Prefer the self-generated tile (has highwayType) when it exists
    path = _hw_tile_path(lat, lon)
    if not os.path.exists(path):
      path = _tile_path(lat, lon)
    ways = self._get_tile(path)
```

with:

```python
    # Prefer the self-generated tile (has highwayType) when it exists
    hw_path = _hw_tile_path(lat, lon)
    pf_path = _tile_path(lat, lon)
    if os.path.exists(hw_path):
      path = hw_path
      self.tile_missing = False
    elif os.path.exists(pf_path):
      path = pf_path
      self.tile_missing = False
    else:
      # Neither tile file exists — area not covered by downloaded tiles.
      # Distinct from "tile present but still loading" (not missing).
      self.tile_missing = True
      return None
    ways = self._get_tile(path)
```

(Behavior-preserving for the query result: the old code's `_get_tile` also returned None for a nonexistent path.)

- [ ] **Step 4: Run tests to verify they pass**

Run: `PYTHONPATH=. .venv/bin/python3 -m pytest plugins/speedlimitd/tests/test_osm_query.py -v`
Expected: all PASS (new tests + no regression in existing ones)

- [ ] **Step 5: Commit**

```bash
git add plugins/speedlimitd/osm_query.py plugins/speedlimitd/tests/test_osm_query.py
git commit -m "speedlimitd: track missing offline tiles in OsmTileReader"
```

---

### Task 2: `OsmDataIntegration` param wiring + region-resolved default

**Files:**
- Modify: `plugins/speedlimitd/speedlimitd.py`
- Test: `plugins/speedlimitd/tests/test_speedlimitd.py`

**Interfaces:**
- Consumes: `config.read_plugin_param` / `write_plugin_param` / `plugin_data_dir`; existing `country_from_gps` + first-fix detection block in `update()` (search for `self.country_detected`).
- Produces: `self.osm_integration_enabled: bool` (refreshed by `_read_params()`, 5 s cadence); `self._resolve_osm_default(country) -> bool`; param file `OsmDataIntegration` in speedlimitd's data dir.

- [ ] **Step 1: Write the failing tests**

Append to `plugins/speedlimitd/tests/test_speedlimitd.py`:

```python
# ============================================================
# OSM Data Integration — param wiring + region default
# ============================================================

class TestOsmIntegrationParam:
  def _make_middleware(self, sld):
    import plugins.speedlimitd.speedlimitd as mod
    with patch.object(mod.messaging, 'SubMaster'):
      mw = mod.SpeedLimitMiddleware()
    mw._sl_pub = MagicMock()
    return mw

  def test_read_params_wires_toggle_on(self, sld, monkeypatch):
    import config
    monkeypatch.setattr(config, 'read_plugin_param',
                        lambda pid, key, default='': '1' if key == 'OsmDataIntegration' else '')
    mw = self._make_middleware(sld)
    mw._read_params()
    assert mw.osm_integration_enabled is True

  def test_read_params_missing_means_off(self, sld, monkeypatch):
    import config
    monkeypatch.setattr(config, 'read_plugin_param', lambda pid, key, default='': '')
    mw = self._make_middleware(sld)
    mw._read_params()
    assert mw.osm_integration_enabled is False


class TestOsmRegionDefault:
  def _make_middleware(self, sld):
    import plugins.speedlimitd.speedlimitd as mod
    with patch.object(mod.messaging, 'SubMaster'):
      mw = mod.SpeedLimitMiddleware()
    mw._sl_pub = MagicMock()
    return mw

  def _param_path(self, tmp_path):
    return tmp_path / 'speedlimitd' / 'data' / 'OsmDataIntegration'

  @pytest.fixture
  def data_dir(self, monkeypatch, tmp_path):
    import config
    monkeypatch.setattr(config, 'PLUGINS_RUNTIME_DIR', str(tmp_path))
    return tmp_path

  def test_cn_defaults_off(self, sld, data_dir):
    mw = self._make_middleware(sld)
    assert mw._resolve_osm_default('cn') is True
    assert self._param_path(data_dir).read_text() == '0'
    assert mw.osm_integration_enabled is False

  def test_non_cn_defaults_on(self, sld, data_dir):
    mw = self._make_middleware(sld)
    assert mw._resolve_osm_default('de') is True
    assert self._param_path(data_dir).read_text() == '1'
    assert mw.osm_integration_enabled is True

  def test_unknown_country_defaults_on(self, sld, data_dir):
    # No bbox match (e.g. US without a us.toml) → not China → ON.
    mw = self._make_middleware(sld)
    assert mw._resolve_osm_default(None) is True
    assert self._param_path(data_dir).read_text() == '1'

  def test_existing_value_never_overwritten(self, sld, data_dir):
    p = self._param_path(data_dir)
    p.parent.mkdir(parents=True)
    p.write_text('0')
    mw = self._make_middleware(sld)
    assert mw._resolve_osm_default('de') is True
    assert p.read_text() == '0'
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `PYTHONPATH=. .venv/bin/python3 -m pytest plugins/speedlimitd/tests/test_speedlimitd.py::TestOsmIntegrationParam plugins/speedlimitd/tests/test_speedlimitd.py::TestOsmRegionDefault -v`
Expected: FAIL — no attribute `osm_integration_enabled` / `_resolve_osm_default`

- [ ] **Step 3: Implement**

In `SpeedLimitMiddleware.__init__`, immediately BEFORE the `self._read_params()` call (search for `self._params_last_read_t = 0.0`):

```python
    # OSM Data Integration (2026-08-07 spec): use OSM maxspeed as base
    # inference. Param resolved to a region default on first GPS fix
    # (cn → OFF, elsewhere → ON), user-controlled thereafter.
    self.osm_integration_enabled: bool = False
    self._osm_default_resolved: bool = False
    self._osm_default_country: str | None = None
```

At the END of `_read_params()` (after the `react_lat_accel_threshold` assignment):

```python
    self.osm_integration_enabled = read_plugin_param(
      'speedlimitd', 'OsmDataIntegration', '') == '1'
```

New method after `_read_params`:

```python
  def _resolve_osm_default(self, country: str | None) -> bool:
    """One-time region default for OsmDataIntegration (first GPS fix).

    China → OFF (OSM maxspeed unreliable there); anywhere else → ON.
    Never overwrites an existing param file (user-controlled once it exists).
    Returns True when resolved (file exists or write succeeded) so the caller
    can retry on the next fix after a transient write failure.
    """
    try:
      from config import plugin_data_dir, write_plugin_param
      if (plugin_data_dir('speedlimitd') / 'OsmDataIntegration').exists():
        return True
      default = '0' if country == 'cn' else '1'
      write_plugin_param('speedlimitd', 'OsmDataIntegration', default)
      self.osm_integration_enabled = default == '1'
      return True
    except Exception:
      return False
```

In `update()`, modify the country-detection block (search `if not self.country_detected:`). Current code sets `self.country_detected = True` after loading the speed table; extend it so the resolved country is remembered and the default is (re)tried:

```python
        if not self.country_detected:
          country = country_from_gps(gps.latitude, gps.longitude, self.country_bboxes)
          if country:
            try:
              SPEED_TABLE_URBAN, SPEED_TABLE_NONURBAN, DEFAULT_FALLBACK_SPEED, LANE_WIDTH_CLASS_TABLE = load_speed_table(country)
            except FileNotFoundError:
              pass
          self.country_detected = True
          self._osm_default_country = country
        if not self._osm_default_resolved:
          self._osm_default_resolved = self._resolve_osm_default(self._osm_default_country)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `PYTHONPATH=. .venv/bin/python3 -m pytest plugins/speedlimitd/tests/test_speedlimitd.py -q --tb=short`
Expected: all PASS

- [ ] **Step 5: Commit**

```bash
git add plugins/speedlimitd/speedlimitd.py plugins/speedlimitd/tests/test_speedlimitd.py
git commit -m "speedlimitd: OsmDataIntegration param with first-fix region default"
```

---

### Task 3: OSM speed storage + freshness gate

**Files:**
- Modify: `plugins/speedlimitd/speedlimitd.py`
- Test: `plugins/speedlimitd/tests/test_speedlimitd.py`

**Interfaces:**
- Consumes: `_ingest_osm_result(result)` (result dict has `speedLimit` in m/s), `self._osm_query_interval` (5.0 s).
- Produces: `self.last_osm_speed_kph: float` (0.0 = none), `self.last_osm_speed_t: float`; `self._osm_base_active(now) -> bool` — the single gate Task 4 consults.

- [ ] **Step 1: Write the failing tests**

Append to `test_speedlimitd.py` (inside a new class; `_result` mirrors the existing helper in the OSM-ingest test class):

```python
class TestOsmSpeedStorage:
  def _make_middleware(self, sld):
    import plugins.speedlimitd.speedlimitd as mod
    with patch.object(mod.messaging, 'SubMaster'):
      mw = mod.SpeedLimitMiddleware()
    mw._sl_pub = MagicMock()
    return mw

  def _result(self, **kw):
    base = {'wayRef': '', 'wayName': '', 'speedLimit': 0.0, 'lanes': 0,
            'roadContext': 1, 'roadName': '', 'highwayType': '', 'distance': 5.0}
    base.update(kw)
    return base

  def test_maxspeed_stored_in_kph(self, sld):
    mw = self._make_middleware(sld)
    mw._ingest_osm_result(self._result(roadName='A1', speedLimit=27.78))
    assert mw.last_osm_speed_kph == pytest.approx(100.0, abs=0.1)
    assert mw.last_osm_speed_t > 0

  def test_same_road_without_maxspeed_keeps_value(self, sld):
    # Sub-segments of the same road may lack the tag — hold the value
    # (the freshness gate expires it if it stays absent).
    mw = self._make_middleware(sld)
    mw._ingest_osm_result(self._result(roadName='A1', speedLimit=27.78))
    mw._ingest_osm_result(self._result(roadName='A1', speedLimit=0.0))
    assert mw.last_osm_speed_kph == pytest.approx(100.0, abs=0.1)

  def test_new_road_without_maxspeed_clears_value(self, sld):
    mw = self._make_middleware(sld)
    mw._ingest_osm_result(self._result(roadName='A1', speedLimit=27.78))
    mw._ingest_osm_result(self._result(roadName='B2', speedLimit=0.0))
    assert mw.last_osm_speed_kph == 0.0

  def test_no_match_keeps_value_for_staleness(self, sld):
    mw = self._make_middleware(sld)
    mw._ingest_osm_result(self._result(roadName='A1', speedLimit=27.78))
    mw._ingest_osm_result(None)
    assert mw.last_osm_speed_kph == pytest.approx(100.0, abs=0.1)


class TestOsmBaseActive:
  def _make_middleware(self, sld):
    import plugins.speedlimitd.speedlimitd as mod
    with patch.object(mod.messaging, 'SubMaster'):
      mw = mod.SpeedLimitMiddleware()
    mw._sl_pub = MagicMock()
    return mw

  def _arm(self, mw, kph=100.0, age=0.0):
    import time as _t
    mw.osm_integration_enabled = True
    mw.last_osm_speed_kph = kph
    mw.last_osm_speed_t = _t.monotonic() - age

  def test_active_when_fresh_and_enabled(self, sld):
    import time as _t
    mw = self._make_middleware(sld)
    self._arm(mw)
    assert mw._osm_base_active(_t.monotonic()) is True

  def test_inactive_when_toggle_off(self, sld):
    import time as _t
    mw = self._make_middleware(sld)
    self._arm(mw)
    mw.osm_integration_enabled = False
    assert mw._osm_base_active(_t.monotonic()) is False

  def test_inactive_when_stale(self, sld):
    import time as _t
    mw = self._make_middleware(sld)
    self._arm(mw, age=11.0)  # > 2 × 5 s query interval
    assert mw._osm_base_active(_t.monotonic()) is False

  def test_inactive_when_implausibly_low(self, sld):
    import time as _t
    mw = self._make_middleware(sld)
    self._arm(mw, kph=20.0)  # < 30 km/h plausibility floor
    assert mw._osm_base_active(_t.monotonic()) is False
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `PYTHONPATH=. .venv/bin/python3 -m pytest plugins/speedlimitd/tests/test_speedlimitd.py::TestOsmSpeedStorage plugins/speedlimitd/tests/test_speedlimitd.py::TestOsmBaseActive -v`
Expected: FAIL — missing attributes/method

- [ ] **Step 3: Implement**

In `__init__`, next to the Task 2 block:

```python
    self.last_osm_speed_kph: float = 0.0   # 0 = no usable OSM maxspeed held
    self.last_osm_speed_t: float = 0.0     # monotonic time it was stored
```

In `_ingest_osm_result`, first line of the method body (before the `if result and (...)` branch):

```python
    prev_road_id = self.last_road_id
```

Then inside the matched branch (`if result and (...)`), AFTER the existing road-identity update (the `if road_id != self.last_road_id:` block), add:

```python
      # OSM maxspeed (m/s → km/h). Held across same-road sub-segments that
      # lack the tag (freshness gate expires it); cleared on a road change
      # to a way without maxspeed — the held value belongs to the old road.
      speed_ms = result.get('speedLimit', 0.0) or 0.0
      if speed_ms > 0.0:
        self.last_osm_speed_kph = speed_ms * 3.6
        self.last_osm_speed_t = time.monotonic()
      elif self.last_road_id != prev_road_id:
        self.last_osm_speed_kph = 0.0
```

(The no-match `else` branch is untouched — a held value survives a missed query and dies by staleness.)

New method after `_resolve_osm_default`:

```python
  def _osm_base_active(self, now: float) -> bool:
    """Toggle ON + fresh, plausible OSM maxspeed → OSM replaces base inference.

    Freshness is 2 query intervals (~10 s): one missed/None query keeps the
    limit, a sustained loss falls back to vision inference. 30 km/h floor
    rejects implausible tags (same floor as MIN_SPEED_LIMIT in update()).
    """
    return (self.osm_integration_enabled
            and self.last_osm_speed_kph >= 30.0
            and now - self.last_osm_speed_t <= 2.0 * self._osm_query_interval)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `PYTHONPATH=. .venv/bin/python3 -m pytest plugins/speedlimitd/tests/test_speedlimitd.py -q --tb=short`
Expected: all PASS

- [ ] **Step 5: Commit**

```bash
git add plugins/speedlimitd/speedlimitd.py plugins/speedlimitd/tests/test_speedlimitd.py
git commit -m "speedlimitd: store OSM maxspeed with freshness gate"
```

---

### Task 4: Base selection, snap bypass, publish fields, tiles-missing param

**Files:**
- Modify: `plugins/speedlimitd/speedlimitd.py`
- Test: `plugins/speedlimitd/tests/test_speedlimitd.py`

**Interfaces:**
- Consumes: `_osm_base_active(now)` (Task 3), `OsmTileReader.tile_missing` (Task 1).
- Produces: `inference_mode == 'osm'`; publish fields `osmSpeedLimit` (float kph) and `osmTilesMissing` (bool); persisted param `OsmTilesMissing` (`'1'`/`'0'`).

- [ ] **Step 1: Write the failing tests**

Append to `test_speedlimitd.py`. These drive full `update()` cycles with the established controlled-SubMaster pattern (see `TestCurvatureCapEnforcementVsDisplay._mw_for_update`):

```python
class TestOsmBaseSelection:
  """OSM maxspeed as base inference — full update() cycles."""

  def _mw_for_update(self, sld):
    import plugins.speedlimitd.speedlimitd as mod
    with patch.object(mod.messaging, 'SubMaster'):
      mw = mod.SpeedLimitMiddleware()
    mw._sl_pub = MagicMock()
    sm = MagicMock()
    sm.updated = {'modelV2': False, 'gpsLocationExternal': False, 'livePose': False}
    sm.update = MagicMock()
    mw.sm = sm
    mw._cmd_sub = None
    mw._lc_sub = None
    return mw

  def _arm(self, mw, kph):
    import time as _t
    mw.osm_integration_enabled = True
    mw.last_osm_speed_kph = kph
    mw.last_osm_speed_t = _t.monotonic()
    # update()'s 5 s param refresh would re-read the (absent) param and wipe
    # the armed toggle — mark params as freshly read.
    mw._params_last_read_t = _t.monotonic()

  def test_osm_replaces_base_and_bypasses_cn_ladder(self, sld):
    mw = self._mw_for_update(sld)
    mw.lane_count_stable = 2          # vision table would give a low urban limit
    self._arm(mw, 104.6)              # 65 mph — NOT a CN-ladder value
    mw.update()
    pub = mw._sl_pub.send.call_args[0][0]
    assert pub['inferenceMode'] == 'osm'
    assert pub['speedLimit'] == 105   # round-to-5, NOT snapped to 100/120
    assert pub['speedLimit'] not in sld._STANDARD_SPEEDS

  def test_toggle_off_is_todays_behavior(self, sld):
    mw = self._mw_for_update(sld)
    mw.lane_count_stable = 2
    self._arm(mw, 104.6)
    mw.osm_integration_enabled = False
    mw.update()
    pub = mw._sl_pub.send.call_args[0][0]
    assert pub['inferenceMode'] == 'lane_count'
    assert pub['speedLimit'] in sld._STANDARD_SPEEDS

  def test_stale_osm_falls_back_to_vision(self, sld):
    import time as _t
    mw = self._mw_for_update(sld)
    mw.lane_count_stable = 2
    self._arm(mw, 104.6)
    mw.last_osm_speed_t = _t.monotonic() - 11.0
    mw.update()
    pub = mw._sl_pub.send.call_args[0][0]
    assert pub['inferenceMode'] == 'lane_count'

  def test_vision_narrow_cap_not_applied_over_osm(self, sld):
    # A rural 2-lane road with OSM 90 must NOT be capped by the ≤2-lane
    # narrow-road heuristic — OSM is authoritative for the base.
    mw = self._mw_for_update(sld)
    mw.lane_count_stable = 2
    mw.vision_cap_stable = 60
    self._arm(mw, 90.0)
    mw.update()
    pub = mw._sl_pub.send.call_args[0][0]
    assert pub['speedLimit'] == 90

  def test_safety_cap_still_wins_over_osm(self, sld):
    mw = self._mw_for_update(sld)
    self._arm(mw, 104.6)
    mw.curvature_cap = 50
    mw.update()
    pub = mw._sl_pub.send.call_args[0][0]
    assert pub['speedLimit'] <= sld.snap_to_standard_speed(50)
    assert pub['safetyCapped'] is True

  def test_osm_suppresses_gs_hold(self, sld):
    import time as _t
    mw = self._mw_for_update(sld)
    # Arm an active G/S sticky hold at 120…
    mw.lane_count_stable = 4
    mw.last_highway_type = 'motorway'
    mw.last_road_context = 'freeway'
    mw.last_way_ref = 'G2'
    mw._gs_last_seen_t = _t.monotonic()
    mw._gs_limit_kph = 120
    # …and a fresh OSM 100 → OSM wins the base.
    self._arm(mw, 100.0)
    mw.update()
    pub = mw._sl_pub.send.call_args[0][0]
    assert pub['inferenceMode'] == 'osm'
    assert pub['speedLimit'] == 100

  def test_osm_step_is_plus_minus_10(self, sld):
    mw = self._mw_for_update(sld)
    self._arm(mw, 65.0)
    mw._displayed_speed_limit = 105   # previously showing 105
    mw._last_step_time = 0.0          # step interval elapsed
    mw.update()
    pub = mw._sl_pub.send.call_args[0][0]
    assert pub['speedLimit'] == 95    # one −10 step toward 65, not a ladder jump

  def test_publish_carries_osm_fields(self, sld):
    mw = self._mw_for_update(sld)
    self._arm(mw, 88.5)
    mw._osm_tiles_missing = True
    mw.update()
    pub = mw._sl_pub.send.call_args[0][0]
    assert pub['osmSpeedLimit'] == pytest.approx(88.5, abs=0.1)
    assert pub['osmTilesMissing'] is True
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `PYTHONPATH=. .venv/bin/python3 -m pytest plugins/speedlimitd/tests/test_speedlimitd.py::TestOsmBaseSelection -v`
Expected: FAIL (inferenceMode never 'osm', missing publish fields)

- [ ] **Step 3: Implement**

All in `speedlimitd.py`.

**(a) `__init__`** — next to the Task 3 state:

```python
    self._osm_tiles_missing: bool = False
    self._osm_tiles_written: bool = False  # first status write flushes stale param
```

**(b) OSM query block in `update()`** (search `self._ingest_osm_result(result)`) — after that call, add:

```python
      # Missing-tiles status → persisted param for the Driving-panel warning.
      # Written on first query and on every change (not every cycle).
      tiles_missing = bool(getattr(self._osm, 'tile_missing', False))
      if not self._osm_tiles_written or tiles_missing != self._osm_tiles_missing:
        self._osm_tiles_missing = tiles_missing
        self._osm_tiles_written = True
        try:
          from config import write_plugin_param
          write_plugin_param('speedlimitd', 'OsmTilesMissing', '1' if tiles_missing else '0')
        except Exception:
          pass
```

**(c) Base-inference block** (search `if gs_mode:` … `self.inference_mode = 'lane_count'`). Replace:

```python
    if gs_mode:
      # Hold the last promote-derived limit through momentary non-G/S flips.
      inferred_speed = self._gs_limit_kph
      self.inference_mode = 'gs_osm'
    else:
      # Lane-count debounce is the EXISTING lane_count_stable directional
      # hysteresis (up 1.5 s, down 2 s curving / 5 s straight) — the limit only
      # moves after a lane-count change has held, so a momentary lane-prob dip
      # can't flicker it.
      inferred_speed = lane_count_limit(self.lane_count_stable)
      self.inference_mode = 'lane_count'
```

with:

```python
    # OSM Data Integration: a fresh posted limit replaces the base inference
    # entirely (vision stays the fallback; safety caps below still min() over
    # it). The G/S bookkeeping above keeps ticking — its hold is simply not
    # consulted while OSM carries the expressway's real limit.
    osm_base = self._osm_base_active(now)
    if osm_base:
      inferred_speed = int(round(self.last_osm_speed_kph))
      self.inference_mode = 'osm'
    elif gs_mode:
      # Hold the last promote-derived limit through momentary non-G/S flips.
      inferred_speed = self._gs_limit_kph
      self.inference_mode = 'gs_osm'
    else:
      # Lane-count debounce is the EXISTING lane_count_stable directional
      # hysteresis (up 1.5 s, down 2 s curving / 5 s straight) — the limit only
      # moves after a lane-count change has held, so a momentary lane-prob dip
      # can't flicker it.
      inferred_speed = lane_count_limit(self.lane_count_stable)
      self.inference_mode = 'lane_count'
```

**(d) Vision narrow cap** (immediately below). Replace the condition:

```python
    if self.vision_cap_stable > 0 and self.lane_count_stable < 3:
```

with:

```python
    if not osm_base and self.vision_cap_stable > 0 and self.lane_count_stable < 3:
```

and extend its comment: the ≤2-lane narrow-road heuristic must not defeat an authoritative posted limit (rural 2-lane roads are legitimately 90–100 in the US/EU).

**(e) Stale CN comment** (search `# OSM maxSpeed is unreliable in China`). Replace the two comment lines with:

```python
    # OSM maxspeed is consumed as the base when OsmDataIntegration is ON (see
    # osm_base above). With the toggle OFF (CN default) OSM contributes only
    # road context, G/S classification, and road name.
```

**(f) Gradual-transition block** (search `target = snap_to_standard_speed(int(speed_limit))`). Replace the block:

```python
    target = snap_to_standard_speed(int(speed_limit))
    if self._displayed_speed_limit == 0:
      # First reading — set immediately
      self._displayed_speed_limit = target
      self._last_step_time = now
    elif target != self._displayed_speed_limit:
      interval = _STEP_DOWN_INTERVAL if target < self._displayed_speed_limit else _STEP_UP_INTERVAL
      if now - self._last_step_time >= interval:
        self._displayed_speed_limit = _step_speed_limit(self._displayed_speed_limit, target)
        self._last_step_time = now
```

with:

```python
    # OSM-sourced base carries the exact posted value — the CN ladder would
    # round a US 45 mph (72 km/h) limit UP to 80. Round to 5 km/h and step
    # ±10 toward it; all other sources keep the CN-ladder snap/step.
    osm_display = osm_base and source == 2
    if osm_display:
      target = int(round(speed_limit / 5.0) * 5)
    else:
      target = snap_to_standard_speed(int(speed_limit))
    if self._displayed_speed_limit == 0:
      # First reading — set immediately
      self._displayed_speed_limit = target
      self._last_step_time = now
    elif target != self._displayed_speed_limit:
      interval = _STEP_DOWN_INTERVAL if target < self._displayed_speed_limit else _STEP_UP_INTERVAL
      if now - self._last_step_time >= interval:
        if osm_display:
          step = 10 if target > self._displayed_speed_limit else -10
          nxt = self._displayed_speed_limit + step
          self._displayed_speed_limit = min(nxt, target) if step > 0 else max(nxt, target)
        else:
          self._displayed_speed_limit = _step_speed_limit(self._displayed_speed_limit, target)
        self._last_step_time = now
```

**(g) Publish block** (search `'reactLatAccel':`) — add before the closing brace:

```python
      # OSM Data Integration telemetry
      'osmSpeedLimit': round(self.last_osm_speed_kph, 1),
      'osmTilesMissing': self._osm_tiles_missing,
```

- [ ] **Step 4: Run the full plugin suite**

Run: `PYTHONPATH=. .venv/bin/python3 -m pytest plugins/speedlimitd/tests/ -q --tb=short`
Expected: all PASS — pay attention to pre-existing `TestGs*` and display-snapping tests: with the toggle default OFF they must be untouched.

- [ ] **Step 5: Commit**

```bash
git add plugins/speedlimitd/speedlimitd.py plugins/speedlimitd/tests/test_speedlimitd.py
git commit -m "speedlimitd: OSM maxspeed as base inference behind OsmDataIntegration"
```

---

### Task 5: Driving-panel toggle + yellow missing-tiles warning

**Files:**
- Modify: `plugins/ui_mod/driving_panel.py`

No automated test (repo has no panel tests — raylib UI is verified on-device). Correctness bar: code review + import check.

- [ ] **Step 1: Implement the helper**

In `driving_panel.py`, module level (near `_plugin_enabled`). Extend the existing `from config import ...` line to also import `MEDIA_DIR` (check the current import first):

```python
def _osm_tiles_missing():
  """Missing-tiles signal for the OSM toggle warning.

  Precise: speedlimitd's persisted OsmTilesMissing status (written from live
  tile queries). Coarse fallback when the daemon has never written it: do the
  tile dirs contain any files at all?
  """
  v = read_plugin_param('speedlimitd', 'OsmTilesMissing', '')
  if v != '':
    return v == '1'
  for d in (os.path.join(MEDIA_DIR, '0/osm/offline_hw'),
            os.path.join(MEDIA_DIR, '0/osm/offline')):
    try:
      for _root, _dirs, files in os.walk(d):
        if files:
          return False
    except OSError:
      pass
  return True
```

- [ ] **Step 2: Add the toggle + warning rows**

In `_build_scroller`, inside the existing speedlimitd block (after the `self._road_info` append). Also extend the module's list_view import with `ListItem, TextAction`, and ensure `import pyray as rl` is present (add if not):

```python
      current_osm = read_plugin_param('speedlimitd', 'OsmDataIntegration') == '1'
      self._osm_integration = toggle_item(
        "Mapd/OSM Data Integration",
        "Use OpenStreetMap speed limits as the base speed limit when available. "
        "OSM data may be unreliable in some regions (e.g. China). Requires "
        "offline map tiles downloaded via Connect.",
        current_osm,
        callback=self._on_osm_integration,
      )
      items.append(self._osm_integration)

      if current_osm and _osm_tiles_missing():
        items.append(ListItem(
          title="⚠ Offline Map Tiles",
          description="No offline map tiles for your area — download them in Connect.",
          action_item=TextAction(text="Missing", color=rl.Color(255, 193, 7, 255)),
        ))
```

- [ ] **Step 3: Add the callback**

Next to `_on_road_info`:

```python
  def _on_osm_integration(self, state):
    write_plugin_param('speedlimitd', 'OsmDataIntegration', '1' if state else '0')
```

Note: the warning row appears/disappears on the next `show_event` rebuild (panel re-entry), which is acceptable.

- [ ] **Step 4: Verify imports & suite**

Run: `PYTHONPATH=. .venv/bin/python3 -c "import ast; ast.parse(open('plugins/ui_mod/driving_panel.py').read())"` (syntax) and the full suite `PYTHONPATH=. .venv/bin/python3 -m pytest plugins/*/tests/ -q --tb=short`.
Expected: syntax OK; suite green. Confirm `ListItem` and `TextAction` are exported by `openpilot.system.ui.widgets.list_view` (they are defined there; check spelling against the catpilot tree at `system/ui/widgets/list_view.py`).

- [ ] **Step 5: Commit**

```bash
git add plugins/ui_mod/driving_panel.py
git commit -m "ui_mod: Mapd/OSM Data Integration toggle with missing-tiles warning"
```

---

### Task 6: Documentation

**Files:**
- Modify: `plugins/speedlimitd/README.md` (user-facing: what the toggle does, region default, tiles via Connect/COD, yellow warning meaning)
- Modify: `plugins/speedlimitd/DESIGN.md` (new section: OSM Data Integration — param, first-fix default, freshness gate, base-priority order `osm > gs_osm > lane_count`, snap bypass, `OsmTilesMissing` mechanism, new publish fields)
- Modify: `plugins/ui_mod/DESIGN.md` (Driving panel row list + params table: add `OsmDataIntegration` and `OsmTilesMissing` rows, both owned by speedlimitd's data dir)

- [ ] **Step 1: Write the docs** — match each file's existing tone (README = ordinary users, no internals; DESIGN = technical). State explicitly in both speedlimitd docs: vision inference is always the fallback; toggle OFF ⇒ behavior identical to before; safety caps always win over OSM. No route IDs, no memory-file references, no device serials.

- [ ] **Step 2: Verify claims against code** — every param name, publish field, default, and threshold mentioned must match the implementation (grep before writing).

- [ ] **Step 3: Commit**

```bash
git add plugins/speedlimitd/README.md plugins/speedlimitd/DESIGN.md plugins/ui_mod/DESIGN.md
git commit -m "docs: OSM Data Integration toggle documentation"
```

---

## Final verification

- [ ] Full suite under the exact pre-push command: `PYTHONPATH=. .venv/bin/python3 -m pytest plugins/*/tests/ -q --tb=short` — all green.
- [ ] `git log --oneline` shows the six thematic commits above on `dev`.
- [ ] Grep the diff for leaked internals: route IDs, memory-file slugs, device serials — none.
