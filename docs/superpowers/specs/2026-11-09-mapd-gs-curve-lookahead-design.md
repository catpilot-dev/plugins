# mapd Phase 2 — G/S Curve Look-ahead — Design

**Status:** design, 2026-11-09. **Supersedes** the "Phase 2 — cutover and
deletion" section of `2026-08-18-mapd-integration-design.md`: road identity is
NOT moving to mapd, and nothing in the tile stack is deleted.

## Intent

On G/S expressways the vision curve cap only sees as far as the model's
confident path — at most 100 m (`curvature_speed_cap`, `MAX_VISION`), and it
switches itself off at 100 km/h and above. At 100–120 km/h that is 3–4 s of
warning. OSM geometry in China is trustworthy on G/S expressways (and only
there), and mapd already publishes the road ahead as a list of points with
curvature. Phase 2 uses that path to compute a **map curve cap** that sees
hundreds of metres ahead, **on G/S only**.

Whether the cap actually slows the car is a **Driving-panel toggle, default
off**. With the toggle off the cap is still computed and published in
`speedLimitState` telemetry, so every G/S drive records what it would have
done next to what the vision cap did and what the car measured in the curve.

## Why road identity stays with the tile reader (route 531 evidence)

Route 531 (`00000531--2aede86b73`, 2026-11-09, the first drive with mapd v2.4.0
running the whole drive) compared mapd's road match against the offline tile
reader:

- mapd's match holds on to the current way while the car stays inside a
  fixed radius `(max(hAcc,5) + lanes·3.7) × multiplier` (1.5 on freeway), and
  never compares other candidates. On the S20 off-ramp, which OSM draws
  parallel to S20 about 16 m away, mapd reported S20 for 14 s while the car was
  2 m from the ramp. On 金海路 (X581) the GPS ran 20–26 m off and mapd dropped
  the road for 36 s.
- Today's tile-only G/S logic handled every S20 moment on 531 correctly: entry
  when the tile reader saw S20, sticky hold through two 1 s tile blips, release
  via lane-drop 7 s before the tile reader even dropped S20.
- The user's rules: G/S mode starts only when the tile reader agrees, ends when
  the tile reader confirms a non-G/S road, and city roads use vision only.
  Under those rules mapd's road identity adds nothing, so it is not used.

mapd's real addition on G/S is look-ahead. Its own `mapCurveSpeed` fired five
times on 531, all on ramps and city roads, four of them false or too cautious
(OSM geometry kinks at city junctions) — consistent with "OSM only on G/S".

## Goals

1. Receive mapd's road-ahead path (`MapdExtendedOut.path`) in speedlimitd.
2. While on a G/S expressway, compute a map curve cap with speedlimitd's own
   lateral-acceleration budget and braking model, including at ≥100 km/h.
3. Publish it, with diagnostics, in `speedLimitState` telemetry on every G/S
   drive, toggle on or off.
4. A Driving-panel toggle that lets the cap enter the speed decision.

## Non-goals

- Any change to road identity, G/S entry/exit, the sticky hold, margin
  release, lane-drop release, the vision curve cap or the reactive cap.
- Deleting or changing `osm_query.py`, `osm_reader.capnp`,
  `generate_hw_tiles.py`, the offline_hw tiles or their tests.
- Using mapd on non-G/S roads for anything.
- Using mapd's own control outputs (`mapCurveSpeed`, `suggestedSpeed`,
  `visionCurveSpeed`, `speedLimitSuggestedSpeed`). mapd's control stays off.
- Logging `mapdExtendedOut` in the rlog (user: not needed).
- Changing the existing "Mapd/OSM Data Integration" toggle (`OsmDataIntegration`,
  OSM maxspeed as the base limit). It is unrelated to this feature despite its
  title.

## Anti-goals (a failure even if it technically works)

- **Any actuation change with the toggle off.** With the toggle off — and when
  the cap is absent for any reason — the published `speedLimit`, `source`,
  `confidence` and every pre-existing telemetry field must be identical to
  today.
- **The cap raising a limit.** It only ever enters the `min()`; it can make the
  target lower, never higher.
- **A cap from the wrong road, unseen.** A cap from a path mapd built along an
  exit ramp, a parallel collector or a cross street is the 531 off-ramp failure
  in a new place. The gate (§4) excludes it wherever the car's own road is
  wrong. Beyond mapd's current way the path cannot be checked on the device
  (§3); the binding point is logged so such cases are visible offline.
- **speedlimitd crashing or degrading when mapd is absent.** `mapdExtendedOut`
  is an injected service that can be missing; a missing or dead mapd must
  leave everything else exactly as today.
- **Reintroducing the queue-size bug.** `mapdExtendedOut` must be registered at
  mapd's own segment size (2 MB), or readers shrink mapd's shm file and SIGBUS
  its publisher (see `plugins/mapd/README.md`, root-cause section).

## Decisions

| Decision | Choice | Whose |
|---|---|---|
| Scope of Phase 2 | G/S curve look-ahead only; no identity cutover | user |
| Road identity / G/S entry & exit | tile reader, unchanged | user |
| Off G/S | mapd ignored entirely; vision only (as today) | user |
| G/S mode start | only once the tile reader agrees (as today) | user |
| Data source | `MapdExtendedOut.path`, own a_y budget; not `mapCurveSpeed` | user (agreed) |
| Cap at ≥100 km/h | yes, the map cap works there | user |
| Tile-reader blip tolerance in the gate | 2 s | user |
| Logging `mapdExtendedOut` in rlog | no | user |
| Actuation | Driving-panel toggle | user |
| Toggle default | off | Claude's call (unvalidated feature; matches the CurveIntegrator precedent) |
| Toggle off behaviour | still computed and published in telemetry | Claude's call |
| How it acts when on | enters the `candidates` `min()` as a safety-class source 4, like the vision curve cap | Claude's call |
| Anchoring | tile reader's current match (2 s tolerance), mapd's ref and the held G/S ref agree | Claude's call (follows "tile reader confirms") |
| Path beyond mapd's current way | not verifiable on device; binding point logged | Claude's call |

## Architecture

```
mapd ──mapdOut @20 Hz──────────────┐
     ──mapdExtendedOut @1 Hz (path)─┤
                                    ▼
                              speedlimitd
   tile reader ─► G/S mode (unchanged) ──┐
                                         ├─► map curve cap ─┬─► speedLimitState telemetry (always)
   mapdOut.wayRef + selType ─────────────┤                  └─► candidates min(), source 4
   mapdExtendedOut.path ─────────────────┘                       (only when the toggle is ON)
```

### 1. mapd plugin — register `mapdExtendedOut`

- `plugins/mapd/plugin.json` `services` gains `mapdExtendedOut` with
  `should_log` **false**, frequency `1.0`, and queue size **2097152**.
  - Rate verified in mapd's `main.go`: the extended publisher is started with
    `StartAutoPublish(time.Second)`, and `ExtendedState.Send()` rebuilds the
    message at most once a second.
  - Size: mapd's `ServiceQueueSize["mapdExtendedOut"]` is `QUEUE_SIZE_MEDIUM`.
  - The tuple layout is `(should_log, frequency, decimation, queue_size)`, so
    decimation must be given explicitly before the size (`null` → `None`).
- Both injectors must carry it: `plugins/services.py` (install time) and
  catpilot's boot-time `selfdrive/plugins/builder.py`, which regenerates its
  "plugin services (auto-generated)" block from the manifest. Verify on device
  that both `services.py` entries and `services.h` show the 4-element tuple,
  and that `/dev/shm/msgq_mapdExtendedOut` stays at 2097536 bytes.
- `test_manifest.py`: assert the new entry and its size, alongside the
  existing `mapdOut` queue-size test.
- The mapd-watch bump checklist (`.github/scripts/mapd_watch.py`) says to check
  `ServiceQueueSize["mapdOut"]`; extend it to `mapdExtendedOut`.

### 2. speedlimitd — subscribe

Add `'mapdExtendedOut'` to the SubMaster **conditionally**, exactly like
`mapdOut` (`'mapdExtendedOut' in SERVICE_LIST`), for the same reason: an
unknown service crash-loops the daemon. Absent service → the cap is never
computed and telemetry reports why.

### 3. The map curve cap — a pure function

New pure function (no I/O, no clock, no globals), unit-testable with plain
stubs. Placement is the builder's call; `mapd_source.py` is the natural home,
since its docstring already describes itself as the mapd adapter.

Inputs (minimum): the path points (lat, lon, curvature), the car's current
position (lat, lon), `v_ego`, the target lateral acceleration, and
`COMFORT_BRAKE`.

Behaviour:

- **Locate the car on the path** and use only points *ahead* of it. The path
  is rebuilt at most once a second and starts at the beginning of mapd's
  current way, so it contains points already behind the car. Project the
  car's *current* GPS position (speedlimitd's 5 Hz view, not the position mapd
  had at publish time) onto the polyline and measure distance along the path
  from there.
- **Per-point cap**, the same model as `curvature_speed_cap`:
  `target_ay_i = a_y · interp(κ_i, [0.02, 0.035], [1.0, TIGHT_CURVE_FACTOR])`,
  `v_curve_i = √(target_ay_i / κ_i)`,
  `v_now_i = √(v_curve_i² + 2·COMFORT_BRAKE·d_i)`; the cap is the minimum
  `v_now_i`. Points below `CURVE_GATE` (0.003) are straight and skipped. The
  a_y is the existing `curve_target_lat_accel` (param
  `MapdCurveTargetLatAccel`, default 1.5). Share the constants with the vision
  cap rather than copying them.
- **No 100 km/h cut-off.** The cap reports its raw value at any speed. When it
  is at or above the current limit it simply does not bind.
- **Curvature quality.** mapd's curvature comes from consecutive OSM nodes,
  which are sparse on long G/S curves and dense at junctions. Expect noise.
  Whether to use mapd's per-point `curvature` as-is, recompute it from the
  lat/lon geometry, and/or smooth it over a window is the builder's call. Unit
  tests must cover a coarse-node arc and a single-node kink, and a single-node
  kink on an otherwise straight G/S road must not produce a binding cap.
- **Look-ahead limit.** Use as much of the path as mapd provides, bounded by a
  sensible maximum distance (builder's call; long enough that a 120 km/h
  approach to the sharpest plausible G/S curve is covered by
  `COMFORT_BRAKE`).
- **Path beyond the current way.** mapd builds the path from its current way
  plus `NextWays`, and `Way.NextWay()` prefers the same name, then the same
  ref, then any valid connection. On a G/S mainline that normally continues
  the mainline, but where the G/S ends, splits or changes ref the path can run
  onto a different road. `MapdPathPoint` carries no way id or ref, so this is
  not checked on the device. The binding point's position is published (§5)
  so it can be checked offline against the offline_hw G/S polylines.

Output: the cap in km/h (0 for "no curve ahead"), plus diagnostics for the
binding point: distance ahead, curvature, its lat/lon, the length of path
available ahead of the car, and the car's lateral distance from the path where
it was projected.

### 4. Gate — when the cap is computed

All of:

1. **G/S mode is on** (`gs_mode` as speedlimitd already computes it; the tile
   reader decides). Off G/S the function is not called.
2. **Both sources put the car on the same G/S road**:
   - `mapdOut` is fresh (`sm.alive` and `sm.valid`), its `waySelectionType`
     is `current` or `predicted`, and its `wayRef` is the G/S ref speedlimitd
     is holding;
   - the tile reader's *current* match (`last_way_ref`, not only the sticky
     held ref) is that same G/S ref, **or was within the last 2 s** (user).
     The 2 s tolerance rides through short tile blips (on 531 both were about
     1 s: one blank, one cross-street snap); a longer lapse closes the gate. It
     also means that on a real exit the cap can keep running for up to 2 s
     after the tile reader leaves the G/S; that is accepted, because the cap
     only ever lowers the target.
   - A G/S mode held only via a ref-less OSM `motorway` produces no cap.
   - Ref comparison normalises multi-ref strings (e.g. `G1503;S20`) — builder
     detail.
3. **The path is fresh**: `mapdExtendedOut` alive/valid, and its `position`
   field has changed within a window that tolerates the 1 Hz rate (builder's
   call, ~2.5 s). Judge freshness by content, not by receive time: the
   auto-publisher can resend the last message with a fresh timestamp.

When any condition fails the cap is not computed (0), and telemetry records
which gate failed (e.g. `off_gs`, `mapd_absent`, `ref_mismatch`, `sel_type`,
`tile_lapsed`, `path_stale`, `no_path_ahead`).

Note: in today's `update()` the vision curve cap is computed before
`gs_mode`. Ordering the map cap after `gs_mode` is the builder's call.

### 5. Telemetry (always, toggle on or off)

New `speedLimitState` keys (names indicative; keep the `mapCurve` prefix):

- `mapCurveCap` — the raw map cap, km/h (0 when none).
- `mapCurveActive` — the toggle state.
- `mapCurveDist` — distance ahead to the binding point, m.
- `mapCurveKappa` — curvature at the binding point.
- `mapCurveLatLon` — the binding point's position (for the offline G/S check,
  since the path itself is not in the rlog).
- `mapCurvePathAhead` — length of path available ahead of the car, m.
- `mapCurveLat` — the car's lateral distance from the path, m (not a gate;
  makes a mis-anchored path visible).
- `mapCurveGate` — `ok` or the failing gate reason.

Already present and needed for evaluation: `curvatureCap` (vision),
`reactLatAccel`, `speedLimit`, `source`, `inferenceMode`, `wayRef`,
`mapdWayRef`, `mapdSelType`. The measured lateral acceleration in the curve can
be recomputed from `carState`/`deviceMotion` in the rlog.

### 6. Driving-panel toggle and actuation

- **Param**: a new speedlimitd plugin param (name indicative:
  `MapdCurveLookahead`), bool, **default off**. Declared in
  `plugins/speedlimitd/plugin.json` `params`; read in speedlimitd's existing
  periodic param read alongside `MapdCurveTargetLatAccel`.
- **UI**: a new `toggle_item` in `plugins/ui_mod/driving_panel.py`, next to the
  existing speedlimitd toggles, written with `write_plugin_param` like
  `_on_osm_integration`. Title indicative: "Map Curve Look-ahead (G/S)";
  description says it slows for curves ahead on G/S expressways using mapd's
  map path. Shown only when the speedlimitd plugin is enabled; whether it is
  also hidden when mapd is disabled is the builder's call.
- **When on**: if the gate passed and `mapCurveCap ≥ MIN_SPEED_LIMIT`, append
  `(mapCurveCap, 4, conf)` to `candidates` — the same safety-class source as
  the vision curve cap. That inherits today's behaviour for safety caps:
  planner_hook's gas-override suspends it, and the lead-override protection
  does not bypass it.
- **Smoothness**: the cap must not step the target down abruptly when a curve
  first comes into range or when the gate opens. The per-point braking term
  already makes it tighten gradually as a curve approaches; whether a hold or
  relax like the vision cap's (3 s hold, ladder relax) is needed on top is the
  builder's call, informed by the telemetry of toggle-off drives.
- **When off**: computed and published, never appended to `candidates`.

## Error handling

| Failure | Response |
|---|---|
| `mapdExtendedOut` not in `SERVICE_LIST` | not subscribed; `mapCurveGate=mapd_absent`; nothing else changes |
| mapd dead / restarting | `sm.alive` false → gate fails; nothing else changes |
| path empty, or car beyond its end | `no_path_ahead` |
| malformed point (NaN/inf curvature, zero-length segment) | skip the point; never raise into `update()` |
| exception inside the cap function | caught at the call site, logged once (like the Phase 1 mapd sampling guard), telemetry reports a gate reason; never affects the speed path |
| toggle param unreadable | treated as off |

## Testing

1. **Pure function** with stub paths: straight road → no cap; constant-radius
   arc at distance d → cap matches the closed form; nearer mild vs farther
   sharp curve → the achievable-profile minimum wins; points behind the car are
   ignored; coarse-node arc; single-node kink on a straight road does not bind;
   empty path; NaN curvature; works at 110–120 km/h.
2. **Gate**: each condition failing on its own produces its reason; the tile
   reader lapsing for 1.9 s keeps the gate open and for 2.1 s closes it;
   all passing computes the cap.
3. **Toggle off — no actuation change** (load-bearing): drive `update()`
   through a G/S scenario with and without a path implying a sharp curve;
   assert `speedLimit`, `source`, `confidence` and every pre-existing telemetry
   key are identical.
4. **Toggle on**: the same scenario lowers `speedLimit` to the cap with
   `source == 4`; a cap above the current limit changes nothing; off G/S the
   toggle has no effect.
5. **Absent service**: `SERVICE_LIST` without `mapdExtendedOut` → daemon
   constructs and runs; telemetry `mapd_absent`; toggle on changes nothing.
6. **Manifest/services**: `mapdExtendedOut` queue size 2 MB, `should_log` false;
   speedlimitd param declared with default off; Driving-panel toggle writes the
   param.
7. Full suite green (the current baseline excludes the pre-existing
   `bmw_e9x_e8x` failures).
8. **On device**: `msgq_mapdExtendedOut` size, mapd PID stable, the new
   telemetry keys present on a G/S drive, the toggle visible and persisting.

## Evaluating before switching it on (guidance, not a gate in code)

The toggle ships off. The telemetry of toggle-off G/S drives answers whether
to switch it on:

- for real G/S curves (measured |a_y| above ~1 m/s²), is the map cap binding
  earlier than the vision cap, and no tighter than the speed the car actually
  needed?
- on straight G/S stretches, does it ever bind below the current limit?
- how often does a binding point (`mapCurveLatLon`) lie off the G/S, checked
  against the offline_hw polylines?

## Documentation updates

- `2026-08-18-mapd-integration-design.md`: mark the Phase 2 section superseded
  by this document.
- `plugins/mapd/README.md`: Phase 2 status — look-ahead only, tile stack kept.
- `plugins/speedlimitd/mapd_source.py` module docstring: drop the
  `result_from_mapd()` plan.

## Left to the builder

Module placement; curvature source and smoothing; look-ahead distance bound;
path freshness window; exact telemetry key names (keeping the prefix); the
param name and toggle wording; ordering relative to `gs_mode` in `update()`;
any hold/relax on the cap; whether the toggle hides when mapd is disabled.
