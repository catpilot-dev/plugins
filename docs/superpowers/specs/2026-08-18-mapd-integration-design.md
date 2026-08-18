# mapd Integration — Design

**Date:** 2026-08-18
**Status:** approved, ready for planning
**Supersedes the OSM tile-reading architecture in** `2026-08-07-osm-data-integration-design.md`
**and the tile-generation half of** `2026-08-10-osm-gs-maxspeed-design.md`.

## Goal

Make `mapd` the sole provider of OpenStreetMap road context, and relieve
`speedlimitd` of every tile concern — reading, generating, and its own capnp
schema. `speedlimitd` keeps all speed-limit arbitration and becomes a pure
consumer of the `mapdOut` message.

## Why now

Three upstream changes landed that individually blocked this and are now
resolved:

1. **v2.1.0** added `highwayClass` and `wayId` to both the tile schema and
   `MapdOut`. `last_osm_hwtype` drives our roadContext demotion
   (`speedlimitd.py:555`) and the motorway hold (`:1146`); until v2.1.0 that
   data existed only in our self-generated tiles, so a pure-mapd consumer was
   impossible.
2. **v2.2.0** added conditional speed-limit support and `conditionalSpeedLimit`.
3. **v2.3.0** (2026-08-12) made shadow subscription a per-queue setting,
   closing [pfeiferj/mapd#88](https://github.com/pfeiferj/mapd/issues/88) —
   the slotless-`carState` torn read that pinned us at v2.0.5.

Separately, our `osm_reader.capnp` declares `highwayType @14 :Text` where
mapd declares `id @14 :Int64`. That collision is live and misreads any
v2.1.0+ tile. This design retires it by deletion rather than alignment.

## Decisions

| Decision | Choice | Rationale |
|---|---|---|
| mapd's role | **OSM data source only** | Preserves the layer contract: speedlimitd owns vEgo/a_y. mapd's own speed-limit and curve control stay off. |
| G/S margin-release | **Retired** | It compensates for a candidate-*set* input; mapd returns a *decision*. See "Margin-release retirement". |
| Tile ownership | **mapd owns `offline/` end to end** | Our generator is retired. COD already downloads into that directory. |
| Tile generation | **Retired** | Accepts the S20 regression; the fix is an upstream patch. |
| `carState` subscription | **Shadow (upstream default `true`)** | Consumes no reader slot. The panic risk is bounded by vision-only degradation plus plugind respawn. |
| mapd unavailable | **Degrade to vision-only** | Reuses the existing `stale`/`no_data` gate paths. No in-process fallback. |
| Download UX | **COD web UI (already built)** | `connect-on-device/tile_manager.py` downloads to the directory mapd reads. |
| Rollout | **Staged: telemetry-first, then cutover** | Measures matcher equivalence and shadow stability before they can affect actuation. |

## Architecture

```
COD web UI ──► map-data.pfeifer.dev ──► /data/media/0/osm/offline/
                                               │
                                         mapd v2.3.0  (plugind-managed binary)
                                               │  mapdOut @ 20 Hz over msgq
                                               ▼
                                         speedlimitd  ── keeps ALL arbitration:
                                                          CN G/S gate, lane table,
                                                          curvature + a_y caps, YOLO
```

Three units with one job each:

- **mapd plugin** — binary lifecycle: version pin, download/update, process
  management, settings. Exists and is complete; this switches it on.
- **Bus schema (slots 17–19)** — the interface contract. Aligning to v2.3.0's
  `custom.capnp` is mechanical and independently testable.
- **speedlimitd** — pure `mapdOut` consumer. Keeps all arbitration; loses all
  tile handling.

`mapd` and `speedlimitd` share only the `mapdOut` message. Neither reads the
other's files.

## Phasing

### Phase 1 — bring-up, telemetry only

mapd runs. speedlimitd subscribes to `mapdOut` and logs what it sees
**alongside** the tile-derived values. The tile reader still drives control, so
actuation is byte-identical to today.

One drive answers the three open questions:

- **Matcher equivalence** — does mapd's bearing-aware match agree with our
  nearest-polyline match? (`mapdRefAgree`)
- **Shadow stability** — does the shadow `carState` subscriber survive
  model-inference load? (mapd uptime and restart count via `plugin_health`)
- **S20 coverage loss** — how much of the corridor actually goes dark?
  (`mapdSpeedLimit` beside `osmSpeedLimit`)

### Phase 2 — cutover and deletion

`mapdOut` becomes the source. Deleted:

- `plugins/speedlimitd/osm_query.py`
- `plugins/speedlimitd/osm_reader.capnp`
- `plugins/speedlimitd/tools/generate_hw_tiles.py`
- `plugins/speedlimitd/tests/test_osm_query.py`
- `plugins/speedlimitd/tests/test_generate_hw_tiles.py`
- `_eval_gs_margin_release`, `_gs_held_ref`, `_gs_margin_count`,
  `_gs_force_release`, `GS_RELEASE_MARGIN_M`, `GS_RELEASE_MARGIN_QUERIES`
- speedlimitd params `MapdSpeedLimitControlEnabled`, `MapdCurveTargetLatAccel`
  (dead once mapd's control is off)

speedlimitd ends with no capnp schema of its own, permanently retiring the
`@14` collision.

### Change surface by phase

| | Phase 1 | Phase 2 |
|---|---|---|
| `plugins/mapd` | pin → v2.3.0, enable process + service + health hook, `.enforced` | — |
| slots 17–19 | align to v2.3.0 | — |
| `speedlimitd.py` | subscribe + telemetry fields | source swap, margin-release removed |
| tile stack | untouched | deleted |
| actuation | unchanged | mapd-driven |

## Data flow and field mapping

`_ingest_osm_result(result: dict | None)` (`speedlimitd.py:1066`) is the single
point where road identity enters, called once per query on-device and in tests.
Phase 2 keeps the function and its semantics unchanged — it receives its dict
from a `mapdOut` adapter instead of a tile query. Everything downstream
(`last_way_ref`, `_osm_gate_ref`, the G/S promotion grammar, the
maxspeed hold-across-sub-segments rule) is untouched.

| `result` key | today, from `osm_query` | Phase 2, from `MapdOut` |
|---|---|---|
| `wayRef` | `way.ref` | `wayRef` |
| `roadName` | `way.name` | `roadName` |
| `speedLimit` (m/s) | `maxSpeed`, else `maxSpeedForward` | `speedLimit` — mapd resolves direction by bearing |
| `lanes` | `way.lanes` | `lanes` |
| `highwayType` | `way.highwayType` (Text) | `highwayClass` (enum) → same lowercase strings |
| `roadContext` | derived from the ref | **still derived from the ref** |
| `distance` | nearest-polyline distance | `distanceFromWayCenter` |
| `refDistances` | computed per ref | removed — margin-release retired |
| `tile_missing` | tile file exists? | `not tileLoaded` |

### The adapter

The adapter is a new module, `plugins/speedlimitd/mapd_source.py`, exposing one
pure function:

```python
def result_from_mapd(mapd_out, valid: bool) -> dict | None
```

It takes the `mapdOut` struct and `sm.valid['mapdOut']`, and returns either the
`result` dict shaped exactly as `osm_query.query()` returned it, or `None`. It
holds no state and performs no I/O, so it is unit-testable with dataclass stubs
and carries the enum→string conversion and the ref-derived `roadContext`.
Keeping it out of `speedlimitd.py` matters: that file is already 1682 lines.

`speedlimitd` subscribes by adding `'mapdOut'` to the existing SubMaster at
`speedlimitd.py:735`. No new messaging object.

In Phase 1 the adapter's output feeds telemetry only. In Phase 2 the same call
feeds `_ingest_osm_result`, replacing `self._osm.query(...)` at `:1205`.

### `roadContext` stays ours

`osm_query` derives it from `_is_expressway_ref` — G-prefixed or S1–S99 →
freeway, else city. mapd's `roadContext` enum carries no CN semantics and would
reclassify S100+ provincial arterials. The adapter keeps computing it from the
ref with the existing grammar, so classification is bit-identical to today.
mapd's own value is logged in Phase 1 for comparison but never drives anything.
Its third state, `unknown`, needs no handling: `_ingest_osm_result` acts only on
`0` and `1` and otherwise leaves `last_road_context` alone.

### Ingest cadence stays 5 s

`mapdOut` publishes at 20 Hz, but every tuned constant in the hold/release
machinery is expressed against the 0.2 Hz query rhythm — most importantly
`_osm_gate`'s freshness window, `2.0 * self._osm_query_interval`. Sampling the
latest `mapdOut` on the existing 5 s tick preserves all of them. Raising the
rate is a separate change with its own verification.

### Phase 1 telemetry fields

Published in `pluginBusLog` alongside the existing ones, all read-only:

`mapdAlive`, `mapdWayRef`, `mapdWayId`, `mapdSpeedLimit` (km/h), `mapdHwClass`,
`mapdLanes`, `mapdSelType` (`current`/`predicted`/`possible`/`extended`/`fail`),
`mapdTileLoaded`, `mapdDistance`, `mapdRefAgree`.

`mapdRefAgree` — mapd's ref versus our tile-derived `last_way_ref` — is the
headline number: it is both the matcher-equivalence check and the evidence for
retiring the margin rule.

## Bus schema alignment

All additions, so ordinals stay stable.

- **`standalone.capnp`** — add the `MapdPosition` struct (`latitude`,
  `longitude`); the `HighwayClass` enum (14 values, `unknown @0` …
  `livingStreet @13`); and `MapdInputType` values `@39`–`@46`
  (`setConditionalSpeedLimitControl`, four `setShadow*` setters,
  `setJsonPathFloat`, `setJsonPathText`, `setJsonPathBool`).
- **`slot17.capnp`** `MapdExtendedOut` — `+ position @3 :MapdPosition`
- **`slot18.capnp`** `MapdIn` — `+ jsonPath @4 :Text`
- **`slot19.capnp`** `MapdOut` — `+ highwayClass @24 :HighwayClass`,
  `+ wayId @25 :Int64`, `+ conditionalSpeedLimit @26 :Text`

Copy `HighwayClass` verbatim: upstream requires it to stay name- and
value-identical to `offline.capnp` because `state.go` casts directly between the
generated enum types.

**Ordering constraint.** Because these are additions, an old binary still works
against the new schema. The converse is the hazard: a v2.3.0 binary publishing
into the current 24-field `MapdOut` silently drops `highwayClass`, the field
Phase 2 depends on. **Schema alignment lands before the version bump.**

## mapd plugin changes

- **Service** — `"services": {"mapdOut": [true, 20.0, 20]}`, the 3-element form
  `bus_logger` uses; `plugins/services.py` injects it into `cereal/services.py`.
  Rate and decimation match upstream's integration doc.
- **Process** — `"processes": [{"name": "mapd", "module": "mapd_runner",
  "condition": "always_run"}]`. plugind stores `condition` (`registry.py:257`)
  but never evaluates it: every declared process runs and is respawned by
  `proc_mgr.sync()` when it dies. That is the desired behaviour here — mapd runs
  offroad so COD downloads work, and a gomsgq panic self-heals.
  `plugind.py:60` already anticipates this case by name.
- **Health hook** — register `device.health_check` → `hook.py:on_health_check`.
  It is implemented but `"hooks": {}` means it never runs. This puts mapd
  liveness into `plugin_health` and therefore into rlogs via bus_logger, which
  is what makes Phase 1's uptime question answerable. Telemetry only; nothing
  is surfaced on-screen.
- **Enable** — uncomment the `.enforced` block at `install.sh:355`, replacing
  the v2.0.6 slotless-crash rationale.
- **Version** — `MAX_ALLOWED_VERSION` and the `MapdVersion` default:
  v2.0.5 → v2.3.0. Both sites carry a comment that the version is coupled to
  slot19's field count.

### Settings

Settings move from runner-written `MapdSettings` to
`/data/openpilot/mapd_defaults.json`, placed by `install.sh`. mapd loads
internal defaults → our custom defaults → the `MapdSettings` param, so config
survives openpilot's boot-time wipe of `/data/params/d/` without
`_ensure_mapd_settings()` rewriting it on every start. The file is untracked in
the catpilot repo, so `git reset --hard` on the device leaves it alone;
`git clean` would remove it and install.sh re-places it.

Contents pin mapd as a data source:

```json
{
  "settings_version": 2,
  "speed_limit_control_enabled": false,
  "map_curve_speed_control_enabled": false,
  "vision_curve_speed_control_enabled": false,
  "external_speed_limit_control_enabled": false,
  "conditional_speed_limit_control_enabled": false,
  "subscriber": { "shadow_car_state": true }
}
```

Two of these are behaviour changes: `mapd_runner.py` currently forces both curve
controls **on** ("always on — curve control is independent of speed limit
toggle"). Under data-source-only they go off, and the speedlimitd params feeding
them become dead and are removed in Phase 2.

## Error handling

| Failure | Detection | Response |
|---|---|---|
| mapd dead or restarting | `sm` reports `mapdOut` invalid | adapter returns `None` → gate `stale` → `no_data` → indicator VISION; plugind respawns |
| shadow `carState` torn read → gomsgq panic | mapd exits, PID gone | as above — bounded and self-healing, which is what makes shadow tolerable where it was not at v2.0.5 |
| no tiles for the area | `tileLoaded == false` | existing `OsmTilesMissing` param → Driving-panel warning, unchanged |
| mapd binary missing, no network | `ensure_binary()` fails, runner exits 1 | needs a bounded retry and a log line — currently absent |
| mapd gains fields past v2.3.0 | none | harmless: capnp is additive |
| version bumped without schema alignment | none | **silent field loss** — `highwayClass` reads `unknown` and Phase 2 mis-classifies |

The last row is the one quiet footgun; it gets a comment at both the version pin
and the slot19 schema.

## Accepted regression: S20

mapd's `generate_offline.go:158-169` parses `maxspeed`, `:forward`, `:backward`,
`:advisory` and the conditionals — **not `maxspeed:lanes`**. On the S20
(外环高速) corridor, 104 of 106 ways are tagged only per-lane. Once mapd owns
tiles, that corridor returns no OSM maxspeed: the gate reports `no_data`, the
indicator reads VISION, and speed comes from the vision lane-count table —
precisely the route-3f1 behaviour diagnosed on 2026-08-12.

This is accepted. Two consequences:

1. **Measure, do not assume.** Phase 1 logs `mapdSpeedLimit` beside
   `osmSpeedLimit` on the same corridor, so the real loss is known before the
   tile stack is deleted.
2. **The closing move is an upstream patch** to `generate_offline.go`:
   `maxspeed:lanes:forward` → `MaxSpeedForward`, `:backward` →
   `MaxSpeedBackward`, bare `maxspeed:lanes` → forward only when `OneWay`,
   always as a fallback, never overriding an explicit scalar, collapsing lanes
   with **MAX**, reusing `ParseMaxSpeed` for units. No schema change. Composes
   with FrogAi's open PR #105, which fixes the `oneway` parsing that `:163`
   currently does with a literal `"yes"` comparison.

The CN G/S gate is not wasted: it still gates corridors carrying a scalar
`maxspeed`, and it is what makes the upstream patch pay off the moment it ships.
The open observation about 16 corridor ways tagged `maxspeed=60` clearing the
`GS_OSM_MIN_KPH` floor by one unit carries over unchanged.

## Margin-release retirement

`_eval_gs_margin_release` exists because `osm_query` returns a candidate
*set*. At route 3de seg 19 the held S1 polyline was still 13.5 m away and still
present in `refDistances`, so `is_gs_now` kept re-arming and `_gs_absent_since`
never started — the 10 s absence timer could not fire, and the margin comparison
was the only way out.

mapd returns a *decision*, not a set. When mapd matches the ramp instead of S1,
the held ref stops appearing, `_gs_absent_since` starts, and the absence timer
releases. The fast path is not lost so much as made unnecessary by the shape of
the new input. mapd's bearing gate (`ACCEPTABLE_BEARING_DELTA_SIN`, sin 45°) and
`IsForward` one-way rejection further reduce the stacked-match class the rule
was built for, and `waySelectionType` exposes mapd's own match confidence.

This is verified in Phase 1 telemetry before anything is deleted.

## Testing

1. **Bus schema roundtrip** — write and read each new field. Gate
   `importorskip('capnp')` narrowly around capnp-dependent tests only, never at
   module scope (lesson from `a0d07ea`, where a file-level gate silently skipped
   18 tests in the pre-push venv).
2. **Adapter unit tests** — `MapdOut` dataclass stubs → result dict, no capnp.
   Covers each mapped field, enum→string conversion, `roadContext` derived from
   the ref rather than taken from mapd, and `unknown` leaving
   `last_road_context` untouched.
3. **Equivalence table** — the load-bearing suite. The same scenarios
   (G2 expressway, S20 with a scalar maxspeed, 3-digit S203, unnamed service
   road, no match, maxspeed absent on a sub-segment) fed through
   `_ingest_osm_result` from both a tile-shaped and a mapd-shaped dict,
   asserting identical `last_way_ref`, `_osm_gate_ref`, `last_osm_hwtype`,
   `last_road_context`, `last_highway_type`, `last_osm_speed_kph`.
4. **Release coverage after margin removal** — the seven existing margin tests
   are *rewritten*, not deleted: the same seg-19 geometry replayed through
   mapd-shaped input, asserting the hold still releases via the absence timer
   within its window, plus the stacked-mismatch case asserting it does not
   release early. Deleting release tests without replacing them is the one way
   this change could quietly remove a safety behaviour.
5. **Degradation** — `mapdOut` invalid → `None` → `stale` → `no_data` →
   `osmTrusted` false → indicator VISION.
6. **Phase 1 telemetry** — the new `mapd*` fields present and correctly typed.
7. **Full suite** with `PYTHONPATH=` (namespace-hijack guard) against the
   current 516-test gate.
8. **On-road** — Phase 1: matcher agreement from `mapdRefAgree`, mapd uptime and
   restart count from `plugin_health`, S20 coverage delta. Phase 2: release
   behaviour on seg-19-like geometry, indicator correctness.

## Rollback

- **Phase 1** — `touch .disabled` on the mapd plugin. The tile reader is still
  driving, so it is a no-op revert.
- **Phase 2** — no runtime fallback by design; rollback is a git revert.

That asymmetry is the reason for staging.

## Out of scope

- Wiring mapd's speed-limit or curve control into the speed target.
- An in-UI download panel — COD already provides one.
- A custom `mapd_download_menu.json` — COD downloads by tile coordinate, not by
  named region, so the menu is unused.
- Raising the ingest cadence above 0.2 Hz.
- The upstream `maxspeed:lanes` patch — tracked as follow-up work, not a
  prerequisite for either phase.
