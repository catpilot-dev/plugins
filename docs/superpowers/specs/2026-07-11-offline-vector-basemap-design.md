# Offline-first vector basemap for the route map

**Date:** 2026-07-11
**Component:** `plugins/ui_mod/route_map.py` (+ new `plugins/ui_mod/offline_basemap.py`)
**Status:** Approved design, pending implementation plan

## Goal

When the last drive's route is fully covered by the device's offline OSM vector
tiles, render a roads-only basemap with **zero network access**. Otherwise fall
back to the existing CartoDB raster-tile download path, unchanged.

"Offline first, online second" — there is no explicit connectivity probe. Offline
coverage is checked first; if it is incomplete, the existing CartoDB download runs
(and already degrades gracefully to a bare dark map + trace when there is no
internet).

## Background / current state

`RouteMapRenderer` in `plugins/ui_mod/route_map.py` downloads CartoDB dark
`@2x.png` raster tiles in a background thread, loads them as raylib textures on the
main thread, and draws the GPS-trace polyline + start/end markers on top. Map zoom
is chosen to fit the whole trace in the render rect.

The device's offline map data at `MEDIA_DIR/0/osm/offline` is **not** raster tiles.
Each file is a **packed Cap'n Proto** `Offline` struct (schema:
`plugins/speedlimitd/osm_reader.capnp`) holding a `List(Way)`, where each `Way` has
`nodes : List(Coordinates)` (lat/lng float64). Files are chunked into 0.25°×0.25°
geographic tiles, path-addressed by `plugins/speedlimitd/osm_query.py::_tile_relpath`:

```
MEDIA_DIR/0/osm/offline/<floor(lat/2)*2>/<floor(lon/2)*2>/<minLat>_<minLon>_<maxLat>_<maxLon>
# e.g. lat 31.6, lon 117.3 -> 30/116/31.500000_117.250000_31.750000_117.500000
```

Because the offline data is vector geometry, not raster images, it cannot be used
as a drop-in tile source. It must be rendered as polylines instead.

## Design decisions (locked)

1. **Coverage rule: all-or-nothing.** Offline mode is used only if *every* 0.25°
   tile spanning the trace bbox exists. Any missing tile → full CartoDB fallback.
   No partial fill, no mixed styles.
2. **Code location: self-contained in `ui_mod`.** No cross-plugin import of
   `speedlimitd`. `ui_mod` vendors a copy of `osm_reader.capnp` and the tile-path
   math. The route map does not depend on `speedlimitd` being installed.
3. **Styling: uniform grey.** Every road drawn at one weight/color. No road-class
   differentiation, no lane weighting, no labels/water/land fills.

## Architecture

### New module: `plugins/ui_mod/offline_basemap.py`

Self-contained. Vendors `_tile_relpath` (0.25° tile granularity) and loads a
vendored `ui_mod/osm_reader.capnp`. Public surface:

- `coverage_complete(min_lat, min_lng, max_lat, max_lng) -> bool`
  Enumerate every 0.25° tile spanning the bbox and return `True` only if all files
  exist under `MEDIA_DIR/0/osm/offline`. Pure `os.path.exists`, no parsing — cheap
  enough to run synchronously in `load_trace()`.

- `load_polylines(min_lat, min_lng, max_lat, max_lng) -> list[list[tuple[float, float]]]`
  Parse each covering tile with `schema.Offline.from_bytes_packed`, mirroring
  `osm_query`'s one-pass conversion (capnp readers have a cumulative traversal
  limit; convert to plain Python tuples immediately). Return each `Way`'s `nodes`
  as a list of `(lat, lng)` tuples. Returns `[]` if `capnp` is unavailable or any
  tile fails to parse (caller then falls back to online).

`capnp` is imported lazily/guarded inside the module so an environment without it
never breaks the home screen (matches `osm_query` behavior).

### Changes to `RouteMapRenderer` (`route_map.py`)

New instance state: `self._offline = False`, `self._polylines = []`.

**`load_trace()`** — after the existing bbox + zoom computation, branch:

1. If `capnp` available **and** `coverage_complete(bbox)`:
   - `self._offline = True`
   - Spawn a daemon background thread that calls `load_polylines(bbox)` and assigns
     the result to `self._polylines` (atomic list assignment; no lock needed).
   - Do **not** call `_download_tiles()`; do **not** touch the `map_tiles` cache dir.
2. Else:
   - `self._offline = False`
   - Existing `_download_tiles()` background-thread path, untouched.

**`render()`**:

- If `self._offline`: skip the texture-loading loop and the tile-blit loop
  entirely. For each polyline in `self._polylines`, project points with the
  existing `_to_screen(pt, ox, oy)` and draw consecutive segments with
  `rl.draw_line_ex(p0, p1, ROAD_WIDTH, ROAD_COLOR)`. The existing scissor clip
  trims roads to the rect; roads fill the view edge-to-edge and the trace's chosen
  zoom still fits the whole route.
- Else: existing raster path (`_load_pending()` + texture blit).
- In both modes the blue GPS trace, start/end markers, and URL overlay draw on top
  exactly as today.

**`cleanup()`**: additionally reset `self._offline = False` and clear
`self._polylines = []`.

### Constants

```python
ROAD_COLOR = rl.Color(90, 90, 90, 255)
ROAD_WIDTH = 2.0
```
Tunable on-device.

## Data flow

```
load_trace(trace)
  compute bbox, zoom, center            (unchanged)
  if capnp and coverage_complete(bbox): # offline branch
      _offline = True
      thread -> _polylines = load_polylines(bbox)
  else:                                 # online branch (unchanged)
      _offline = False
      thread -> _download_tiles()

render(rect)
  draw bg
  if _offline: draw grey road polylines (projected via _to_screen)
  else:        load + blit raster textures
  draw blue trace + markers + URL bar   (unchanged)
```

## Error handling

- `capnp` import failure or any tile parse error → `load_polylines` returns `[]`
  and/or the coverage/import guard selects the online path. The home screen never
  crashes.
- Offline mode issues no network requests.
- Route with no offline coverage and no internet → identical to today's behavior
  (dark rounded rect + blue trace, no basemap).

## Testing (extend `plugins/ui_mod/tests/test_route_map.py`)

- Tile enumeration / `_tile_relpath` produces paths matching known on-device
  filenames for a sample bbox.
- `coverage_complete` → `True` when all covering files exist (tmp fixture);
  `False` when one is removed.
- `load_polylines` parses a synthetic packed `Offline` tile into the expected
  `list[list[(lat,lng)]]` structure.
- Mode selection: full coverage → `_offline True`, no CartoDB download thread and
  no writes under `map_tiles`; a missing tile → `_offline False`, online path taken.
- `capnp` unavailable → `load_polylines` returns `[]` and the renderer selects the
  online path.

## Out of scope (YAGNI)

- Partial/best-effort offline fill or offline+online patching.
- Road-class styling, lane weighting, labels, water/land fills.
- Live re-query or panning — this is the static last-drive map only.
- Any change to how the trace, markers, zoom-fit, or URL bar work.
