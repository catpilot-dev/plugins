# Offline Vector Basemap Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Render the last-drive route map from offline OSM vector tiles (roads-only, zero network) when they fully cover the route, else fall back to the existing CartoDB raster download.

**Architecture:** A new self-contained `plugins/ui_mod/offline_basemap.py` vendors the tile-path math and a copy of `osm_reader.capnp`; it exposes `coverage_complete()` and `load_polylines()`. `RouteMapRenderer` (in `route_map.py`) picks offline vs. online mode inside `load_trace()` and draws grey road polylines instead of raster tiles when offline.

**Tech Stack:** Python, pycapnp (packed Cap'n Proto), pyray/raylib, pytest.

## Global Constraints

- All-or-nothing coverage: offline mode only if **every** 0.25° tile spanning the trace bbox exists; any missing tile → full CartoDB fallback.
- Self-contained in `ui_mod`: **no** import of `speedlimitd`. Vendor the schema + tile-path math into `ui_mod`.
- Uniform grey roads: one weight/color, no road-class/lane styling, no labels/water/land fills.
- `capnp` import must be guarded — its absence must never crash the home screen; it forces online mode.
- Offline dir: `MEDIA_DIR/0/osm/offline`; tiles are **packed** Cap'n Proto `Offline` structs, 0.25° geographic chunks, path `<floor(lat/2)*2>/<floor(lon/2)*2>/<minLat>_<minLon>_<maxLat>_<maxLon>` with `%.6f` coords.
- Point convention: `RouteMapRenderer._to_screen(point, ...)` reads `point[0]=lat`, `point[1]=lng`. Polyline points MUST be `(lat, lng)`.
- Sibling imports in `ui_mod` are bare (e.g. `drive_stats.py` does `from route_map import RouteMapRenderer`); use `import offline_basemap`.
- Don't add `Co-Authored-By` trailers to commits.
- Run tests with `PYTHONPATH=. uv run pytest` from the `plugins/` repo root.

---

### Task 1: Vendored schema + tile-path math + coverage check

**Files:**
- Create: `plugins/ui_mod/osm_reader.capnp`
- Create: `plugins/ui_mod/offline_basemap.py`
- Test: `plugins/ui_mod/tests/test_offline_basemap.py`

**Interfaces:**
- Produces:
  - `_tile_relpath(min_lat: float, min_lon: float) -> str` — relative path of the tile whose min corner is `(min_lat, min_lon)`.
  - `_tiles_covering_bbox(min_lat: float, min_lng: float, max_lat: float, max_lng: float) -> list[str]` — relpaths of all 0.25° tiles spanning the bbox.
  - `coverage_complete(min_lat, min_lng, max_lat, max_lng, tile_dir=OFFLINE_DIR) -> bool`.
  - Constants `OFFLINE_DIR`, `SCHEMA_PATH`, `TILE_SIZE = 0.25`.

- [ ] **Step 1: Vendor the capnp schema**

Create `plugins/ui_mod/osm_reader.capnp` with exactly this content (copied from `plugins/speedlimitd/osm_reader.capnp`):

```capnp
@0xb5e5f44e3ff0ea5a;

struct Coordinates {
  latitude @0 :Float64;
  longitude @1 :Float64;
}

struct Way {
  name @0 :Text;
  ref @1 :Text;
  maxSpeed @2 :Float64;
  minLat @3 :Float64;
  minLon @4 :Float64;
  maxLat @5 :Float64;
  maxLon @6 :Float64;
  nodes @7 :List(Coordinates);
  lanes @8 :UInt8;
  advisorySpeed @9 :Float64;
  hazard @10 :Text;
  oneWay @11 :Bool;
  maxSpeedForward @12 :Float64;
  maxSpeedBackward @13 :Float64;
  # OSM highway=* classification (e.g. "tertiary"). Only present in
  # self-generated offline_hw tiles; reads as "" on pfeifer tiles.
  highwayType @14 :Text;
}

struct Offline {
  minLat @0 :Float64;
  minLon @1 :Float64;
  maxLat @2 :Float64;
  maxLon @3 :Float64;
  ways @4 :List(Way);
  overlap @5 :Float64;
}
```

- [ ] **Step 2: Write the failing test**

Create `plugins/ui_mod/tests/test_offline_basemap.py`:

```python
"""Tests for offline_basemap tile-path math and coverage/parse logic."""
import importlib.util
import os
import sys
import types

import pytest

UI_MOD_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


@pytest.fixture
def obm():
  # config lives at the plugins root; a stub keeps the test hermetic.
  # tile_dir is always passed explicitly, so MEDIA_DIR's value is irrelevant.
  sys.modules.setdefault('config', types.SimpleNamespace(MEDIA_DIR='/data/media'))
  if UI_MOD_DIR not in sys.path:
    sys.path.insert(0, UI_MOD_DIR)
  spec = importlib.util.spec_from_file_location(
    'offline_basemap', os.path.join(UI_MOD_DIR, 'offline_basemap.py'))
  mod = importlib.util.module_from_spec(spec)
  sys.modules['offline_basemap'] = mod
  spec.loader.exec_module(mod)
  return mod


class TestTilePaths:
  def test_tile_relpath_matches_device(self, obm):
    rel = obm._tile_relpath(31.5, 117.25)
    assert rel == os.path.join('30', '116', '31.500000_117.250000_31.750000_117.500000')

  def test_tiles_covering_single(self, obm):
    rels = obm._tiles_covering_bbox(31.6, 117.30, 31.6, 117.30)
    assert rels == [os.path.join('30', '116', '31.500000_117.250000_31.750000_117.500000')]

  def test_tiles_covering_span_2x2(self, obm):
    # lat 31.4..31.6 -> tiles 31.25 & 31.5; lon 117.2..117.4 -> tiles 117.0 & 117.25
    rels = obm._tiles_covering_bbox(31.4, 117.2, 31.6, 117.4)
    assert len(rels) == 4


class TestCoverage:
  def test_coverage_complete_true(self, obm, tmp_path):
    for rel in obm._tiles_covering_bbox(31.6, 117.3, 31.6, 117.3):
      p = tmp_path / rel
      p.parent.mkdir(parents=True, exist_ok=True)
      p.write_bytes(b'x')
    assert obm.coverage_complete(31.6, 117.3, 31.6, 117.3, tile_dir=str(tmp_path)) is True

  def test_coverage_incomplete(self, obm, tmp_path):
    assert obm.coverage_complete(31.6, 117.3, 31.6, 117.3, tile_dir=str(tmp_path)) is False
```

- [ ] **Step 3: Run test to verify it fails**

Run: `PYTHONPATH=. uv run pytest plugins/ui_mod/tests/test_offline_basemap.py -v`
Expected: FAIL — `offline_basemap.py` does not exist (import error / ModuleNotFoundError).

- [ ] **Step 4: Write minimal implementation**

Create `plugins/ui_mod/offline_basemap.py`:

```python
"""Offline OSM vector-tile reader for the route map basemap.

Self-contained within ui_mod: vendors the tile-path math and the
osm_reader.capnp schema so the route map never depends on the speedlimitd
plugin being installed. Tiles are packed Cap'n Proto `Offline` structs
(0.25 deg geographic chunks) under MEDIA_DIR/0/osm/offline.
"""
import math
import os

from config import MEDIA_DIR

OFFLINE_DIR = os.path.join(MEDIA_DIR, "0", "osm", "offline")
SCHEMA_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "osm_reader.capnp")
TILE_SIZE = 0.25  # degrees per tile


def _tile_relpath(min_lat, min_lon):
  """Relative path (lat_dir/lon_dir/fname) for the tile whose min corner is
  (min_lat, min_lon). Matches speedlimitd's on-device tile naming."""
  lat_dir = str(int(math.floor(min_lat / 2) * 2))
  lon_dir = str(int(math.floor(min_lon / 2) * 2))
  fname = f"{min_lat:.6f}_{min_lon:.6f}_{min_lat + TILE_SIZE:.6f}_{min_lon + TILE_SIZE:.6f}"
  return os.path.join(lat_dir, lon_dir, fname)


def _tiles_covering_bbox(min_lat, min_lng, max_lat, max_lng):
  """Relative paths of every 0.25 deg tile spanning the bbox (inclusive)."""
  lat_i0 = int(math.floor(min_lat / TILE_SIZE))
  lat_i1 = int(math.floor(max_lat / TILE_SIZE))
  lon_i0 = int(math.floor(min_lng / TILE_SIZE))
  lon_i1 = int(math.floor(max_lng / TILE_SIZE))
  paths = []
  for lat_i in range(lat_i0, lat_i1 + 1):
    for lon_i in range(lon_i0, lon_i1 + 1):
      paths.append(_tile_relpath(lat_i * TILE_SIZE, lon_i * TILE_SIZE))
  return paths


def coverage_complete(min_lat, min_lng, max_lat, max_lng, tile_dir=OFFLINE_DIR):
  """True only if every tile spanning the bbox exists on disk."""
  rels = _tiles_covering_bbox(min_lat, min_lng, max_lat, max_lng)
  return bool(rels) and all(os.path.exists(os.path.join(tile_dir, r)) for r in rels)
```

- [ ] **Step 5: Run test to verify it passes**

Run: `PYTHONPATH=. uv run pytest plugins/ui_mod/tests/test_offline_basemap.py -v`
Expected: PASS (5 tests).

- [ ] **Step 6: Commit**

```bash
git add plugins/ui_mod/osm_reader.capnp plugins/ui_mod/offline_basemap.py plugins/ui_mod/tests/test_offline_basemap.py
git commit -m "feat(ui_mod): offline tile coverage check + vendored osm schema"
```

---

### Task 2: Parse vector tiles into polylines

**Files:**
- Modify: `plugins/ui_mod/offline_basemap.py`
- Test: `plugins/ui_mod/tests/test_offline_basemap.py`

**Interfaces:**
- Consumes: `_tiles_covering_bbox`, `SCHEMA_PATH`, `OFFLINE_DIR` from Task 1.
- Produces:
  - Module flag `HAVE_CAPNP: bool` and `_SCHEMA` (loaded schema or `None`).
  - `load_polylines(min_lat, min_lng, max_lat, max_lng, tile_dir=OFFLINE_DIR) -> list[list[tuple[float, float]]]` — each inner list is a road's `(lat, lng)` points (>= 2). Returns `[]` if capnp unavailable; skips tiles that fail to parse.

- [ ] **Step 1: Write the failing tests**

Append to `plugins/ui_mod/tests/test_offline_basemap.py`:

```python
class TestLoadPolylines:
  def test_no_capnp_returns_empty(self, obm, monkeypatch):
    monkeypatch.setattr(obm, 'HAVE_CAPNP', False)
    assert obm.load_polylines(31.6, 117.3, 31.6, 117.3, tile_dir='/nonexistent') == []

  def test_parses_one_way(self, obm, tmp_path):
    capnp = pytest.importorskip('capnp')
    schema = capnp.load(obm.SCHEMA_PATH)
    msg = schema.Offline.new_message()
    msg.minLat, msg.minLon, msg.maxLat, msg.maxLon = 31.5, 117.25, 31.75, 117.5
    ways = msg.init('ways', 1)
    nodes = ways[0].init('nodes', 2)
    nodes[0].latitude, nodes[0].longitude = 31.60, 117.30
    nodes[1].latitude, nodes[1].longitude = 31.61, 117.31
    rel = obm._tile_relpath(31.5, 117.25)
    path = tmp_path / rel
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(msg.to_bytes_packed())

    polys = obm.load_polylines(31.6, 117.3, 31.6, 117.3, tile_dir=str(tmp_path))
    assert len(polys) == 1
    assert len(polys[0]) == 2
    assert polys[0][0] == pytest.approx((31.60, 117.30))
    assert polys[0][1] == pytest.approx((31.61, 117.31))

  def test_skips_short_and_bad_tiles(self, obm, tmp_path):
    pytest.importorskip('capnp')
    # A tile of garbage bytes must be skipped, not raise.
    rel = obm._tile_relpath(31.5, 117.25)
    path = tmp_path / rel
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(b'not a capnp message')
    assert obm.load_polylines(31.6, 117.3, 31.6, 117.3, tile_dir=str(tmp_path)) == []
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `PYTHONPATH=. uv run pytest plugins/ui_mod/tests/test_offline_basemap.py::TestLoadPolylines -v`
Expected: FAIL — `AttributeError: module 'offline_basemap' has no attribute 'HAVE_CAPNP'` / `load_polylines`.

- [ ] **Step 3: Write minimal implementation**

In `plugins/ui_mod/offline_basemap.py`, add the capnp load block immediately after the `TILE_SIZE = 0.25` line:

```python
try:
  import capnp
  _SCHEMA = capnp.load(SCHEMA_PATH)
  HAVE_CAPNP = True
except (ImportError, OSError):
  _SCHEMA = None
  HAVE_CAPNP = False
```

And append this function to the end of the file:

```python
def load_polylines(min_lat, min_lng, max_lat, max_lng, tile_dir=OFFLINE_DIR):
  """Parse every covering tile into a list of road polylines.

  Each polyline is a list of (lat, lng) tuples (>= 2 points), matching
  RouteMapRenderer._to_screen's point convention. Returns [] if capnp is
  unavailable. Tiles that fail to parse are skipped, not fatal.
  """
  if not HAVE_CAPNP:
    return []
  polylines = []
  for rel in _tiles_covering_bbox(min_lat, min_lng, max_lat, max_lng):
    path = os.path.join(tile_dir, rel)
    try:
      with open(path, "rb") as f:
        data = f.read()
      offline = _SCHEMA.Offline.from_bytes_packed(
        data, traversal_limit_in_words=len(data) * 8,
      )
      for way in offline.ways:
        pts = [(n.latitude, n.longitude) for n in way.nodes]
        if len(pts) >= 2:
          polylines.append(pts)
    except Exception:
      continue
  return polylines
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `PYTHONPATH=. uv run pytest plugins/ui_mod/tests/test_offline_basemap.py -v`
Expected: PASS. (If pycapnp is not installed locally, the two `importorskip` tests SKIP and the rest PASS — that is acceptable.)

- [ ] **Step 5: Commit**

```bash
git add plugins/ui_mod/offline_basemap.py plugins/ui_mod/tests/test_offline_basemap.py
git commit -m "feat(ui_mod): parse offline vector tiles into road polylines"
```

---

### Task 3: Wire offline mode into RouteMapRenderer

**Files:**
- Modify: `plugins/ui_mod/route_map.py`
- Test: `plugins/ui_mod/tests/test_route_map.py`

**Interfaces:**
- Consumes: `offline_basemap.HAVE_CAPNP`, `offline_basemap.coverage_complete`, `offline_basemap.load_polylines`.
- Produces (on `RouteMapRenderer`): instance attrs `self._offline: bool`, `self._polylines: list`; method `self._load_offline(bbox: tuple)`.

- [ ] **Step 1: Add the module import and constants**

In `plugins/ui_mod/route_map.py`, add the sibling import directly below `import pyray as rl` (line 17):

```python
import offline_basemap
```

Add these constants next to the trace constants (after `TRACE_WIDTH = 8.0`, line 29):

```python
ROAD_COLOR = rl.Color(90, 90, 90, 255)
ROAD_WIDTH = 2.0
```

- [ ] **Step 2: Initialize offline state in `__init__`**

In `RouteMapRenderer.__init__`, add after `self._rect_h = 0` (line 94):

```python
    self._offline = False
    self._polylines = []
```

- [ ] **Step 3: Branch mode selection in `load_trace`**

In `load_trace`, replace the current tile-keys + download block (from `self._tile_keys = [` through the `threading.Thread(target=self._download_tiles, daemon=True).start()` line, current lines 136-144) with:

```python
    if offline_basemap.HAVE_CAPNP and offline_basemap.coverage_complete(
        min_lat, min_lng, max_lat, max_lng):
      self._offline = True
      self._polylines = []
      threading.Thread(
        target=self._load_offline,
        args=((min_lat, min_lng, max_lat, max_lng),),
        daemon=True,
      ).start()
    else:
      self._offline = False
      self._tile_keys = [
        (self._zoom, x, y)
        for x in range(self._tx0, self._tx1 + 1)
        for y in range(self._ty0, self._ty1 + 1)
      ]
      self._downloading = True
      self._download_done = False
      threading.Thread(target=self._download_tiles, daemon=True).start()
```

- [ ] **Step 4: Add the `_load_offline` method**

Add this method immediately after `load_trace` (before `render`):

```python
  def _load_offline(self, bbox):
    """Background thread: parse offline vector tiles into road polylines."""
    self._polylines = offline_basemap.load_polylines(*bbox)
```

- [ ] **Step 5: Branch the tile-drawing in `render`**

In `render`, replace the `self._load_pending()` call (line 155) and the tile-blit loop (lines 166-174) so the basemap step is mode-aware. Specifically, delete the standalone `self._load_pending()` line under the "Load any newly downloaded tiles" comment, and replace the `# Draw tiles at 1:1 pixel scale` loop with:

```python
    # Draw basemap: offline vector roads, or downloaded raster tiles.
    if self._offline:
      for poly in self._polylines:
        for i in range(len(poly) - 1):
          p0 = self._to_screen(poly[i], ox, oy)
          p1 = self._to_screen(poly[i + 1], ox, oy)
          rl.draw_line_ex(p0, p1, ROAD_WIDTH, ROAD_COLOR)
    else:
      self._load_pending()
      for key in self._tile_keys:
        tex = self._textures.get(key)
        if tex and rl.is_texture_valid(tex):
          z, tx, ty = key
          dx = ox + tx * TILE_PX
          dy = oy + ty * TILE_PX
          src = rl.Rectangle(0, 0, tex.width, tex.height)
          dst = rl.Rectangle(dx, dy, TILE_PX, TILE_PX)
          rl.draw_texture_pro(tex, src, dst, rl.Vector2(0, 0), 0, rl.WHITE)
```

- [ ] **Step 6: Reset offline state in `cleanup`**

In `cleanup`, add after `self._tile_keys = []` (end of method):

```python
    self._offline = False
    self._polylines = []
```

- [ ] **Step 7: Add sys.path to the route_map test fixture**

In `plugins/ui_mod/tests/test_route_map.py`, the fixture must let `import offline_basemap` resolve. In the `route_map` fixture, add right before the `spec = importlib.util.spec_from_file_location(...)` line:

```python
  ui_mod_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
  if ui_mod_dir not in sys.path:
    sys.path.insert(0, ui_mod_dir)
```

(`sys` is already imported inside the fixture; `os` is imported at module top.)

- [ ] **Step 8: Write the failing mode-selection tests**

Append to `plugins/ui_mod/tests/test_route_map.py`:

```python
# ============================================================
# Offline vs online mode selection
# ============================================================

class _FakeThread:
  """Records the target instead of starting a real thread."""
  started = []

  def __init__(self, target=None, args=(), daemon=None):
    self.target = target

  def start(self):
    _FakeThread.started.append(self.target)


class TestModeSelection:
  def _setup(self, route_map, monkeypatch, have_capnp, covered):
    import offline_basemap
    monkeypatch.setattr(offline_basemap, 'HAVE_CAPNP', have_capnp)
    monkeypatch.setattr(offline_basemap, 'coverage_complete', lambda *a, **k: covered)
    _FakeThread.started = []
    monkeypatch.setattr(route_map.threading, 'Thread', _FakeThread)

  def test_offline_when_capnp_and_covered(self, route_map, monkeypatch):
    self._setup(route_map, monkeypatch, have_capnp=True, covered=True)
    r = route_map.RouteMapRenderer()
    r.load_trace([(31.60, 117.30), (31.61, 117.31)])
    assert r._offline is True
    assert _FakeThread.started == [r._load_offline]

  def test_online_when_not_covered(self, route_map, monkeypatch):
    self._setup(route_map, monkeypatch, have_capnp=True, covered=False)
    r = route_map.RouteMapRenderer()
    r.load_trace([(31.60, 117.30), (31.61, 117.31)])
    assert r._offline is False
    assert _FakeThread.started == [r._download_tiles]

  def test_online_when_no_capnp(self, route_map, monkeypatch):
    self._setup(route_map, monkeypatch, have_capnp=False, covered=True)
    r = route_map.RouteMapRenderer()
    r.load_trace([(31.60, 117.30), (31.61, 117.31)])
    assert r._offline is False
    assert _FakeThread.started == [r._download_tiles]
```

- [ ] **Step 9: Run tests to verify they fail, then pass**

Run: `PYTHONPATH=. uv run pytest plugins/ui_mod/tests/test_route_map.py -v`
Expected before Steps 1-7 applied: FAIL. After all edits: PASS (existing tests + 3 new mode-selection tests).

- [ ] **Step 10: Run the full ui_mod test suite**

Run: `PYTHONPATH=. uv run pytest plugins/ui_mod/tests/ -v`
Expected: PASS (all `test_route_map.py`, `test_offline_basemap.py`, `test_drive_tracker.py`).

- [ ] **Step 11: Commit**

```bash
git add plugins/ui_mod/route_map.py plugins/ui_mod/tests/test_route_map.py
git commit -m "feat(ui_mod): offline-first vector basemap in route map"
```

---

## On-device verification (after merge, not part of TDD loop)

The vector-render path (`render` offline branch) has no unit test — raylib draw calls need a GPU. Verify on the C3:

1. Deploy plugins to C3 (`ssh c3`, fetch/reset `origin/dev`, `bash install.sh`).
2. With a last drive whose route sits inside the offline-covered region (lat 28-32 / lng 114-120), open the home screen and confirm grey roads render with the blue trace on top and **no** network fetch (check `/tmp/plugin_logs` / no `map_tiles` dir writes).
3. Confirm a route outside the offline region still downloads CartoDB tiles as before.
4. Tune `ROAD_COLOR` / `ROAD_WIDTH` if the roads read too faint/heavy against the dark background.

---

## Self-Review

**Spec coverage:**
- All-or-nothing coverage → Task 1 `coverage_complete` (all tiles) + Task 3 branch. ✓
- Self-contained in ui_mod → Task 1 vendors schema + math; `import offline_basemap` is a sibling import; no speedlimitd reference. ✓
- Uniform grey → Task 3 Step 1 constants + Step 5 single-color loop. ✓
- capnp-guarded, never crashes → Task 2 try/except sets `HAVE_CAPNP`; Task 3 branch requires it; Task 2 per-tile parse guarded. ✓
- Offline dir / packed format / path scheme → Task 1 `OFFLINE_DIR`, `_tile_relpath`; Task 2 `from_bytes_packed` with `traversal_limit_in_words`. ✓
- `(lat, lng)` point convention → Task 2 `(n.latitude, n.longitude)`; Task 3 render uses `_to_screen`. ✓
- Fallback path unchanged → Task 3 online branch preserves existing `_tile_keys` + `_download_tiles`. ✓
- Testing bullets → Tasks 1-3 cover tile-path/coverage/parse/mode-selection; render path deferred to on-device (documented). ✓

**Placeholder scan:** No TBD/TODO; every code step shows complete code. ✓

**Type consistency:** `_tile_relpath`, `_tiles_covering_bbox`, `coverage_complete`, `load_polylines`, `HAVE_CAPNP`, `_load_offline`, `_offline`, `_polylines` named identically across all tasks and tests. Polyline type `list[list[(lat,lng)]]` consistent. ✓
