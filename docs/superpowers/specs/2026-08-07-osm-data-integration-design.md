# "Mapd/OSM Data Integration" Toggle — Design

**Goal:** Let speedlimitd consume OSM maxspeed data as its base speed-limit
source, behind a Driving-panel toggle that defaults OFF in China and ON
elsewhere (US/EU, where OSM data is reliable).

**Scope note:** This feature does NOT involve the mapd Go binary (which stays
disabled — see pfeiferj/mapd issue #88). `osm_query.py` already reads offline
capnp tiles directly and already returns `speedLimit` on every query;
speedlimitd currently discards it. This feature stops discarding it, behind a
toggle. Tile acquisition is COD's responsibility (downloads tiles for a
user-selected range); speedlimitd only reads
`/data/media/0/osm/{offline_hw,offline}`.

## Core rule: vision is the floor, OSM is opportunistic

Vision inference (lane-count / G-S tables) is always available and is the
fallback in every degraded case. OSM maxspeed, when the toggle is ON and a
fresh valid value exists, replaces the *base* inference only. It is never a
dependency: no tiles on disk, no maxspeed on the matched way, invalid GPS,
match farther than 50 m, or a stale value (> 2 query intervals ≈ 10 s) all
degrade seamlessly to the existing vision path, which is byte-identical to
today's behavior.

Safety caps are unchanged and always win: YOLO sign detection, the proactive
curvature cap, and the reactive measured-a_y cap still `min()` over the base
source. OSM can never lift a safety cap (preserves the lead-override lesson).

## 1. Toggle & param

- Driving panel row **"Mapd/OSM Data Integration"**, in the speedlimitd group
  (next to "Speed Limit Sign"), rendered by ui_mod `driving_panel.py` using the
  same cross-plugin pattern as `ShowSpeedLimitSign`.
- Toggle description text notes: "Uses OpenStreetMap speed limits when
  available. OSM data may be unreliable in some regions (e.g. China)."
- Param `OsmDataIntegration` (`'1'` / `'0'`) in **speedlimitd's data dir**
  (`plugin_data_dir('speedlimitd')` — survives openpilot `Params::clearAll`).
- File absent = "not yet resolved" (see §2). UI renders absent as OFF.

## 2. Region-resolved default (first GPS fix)

In the speedlimitd daemon: while the param file is absent, on the first valid
GPS fix:

- inside China bounding box (lat 18–54, lon 73–135) → write `'0'` (OFF)
- otherwise → write `'1'` (ON)

One-time write; from then on the param is purely user-controlled (the daemon
never writes it again once the file exists). Before any GPS fix, behavior is
OFF (safe default). The bbox includes some neighboring countries; acceptable
for a default that users can override.

A CN user who manually flips it ON gets CN OSM maxspeeds (unreliable) as
base — an explicit, informed choice per the toggle description.

## 3. OSM consumption path (toggle ON)

- `_ingest_osm_result()` additionally stores:
  - `last_osm_speed_kph` = `result['speedLimit']` (m/s) × 3.6, only when > 0
  - `last_osm_speed_t` = query timestamp
  - cleared when the matched road identity changes to a way without maxspeed;
    a no-match (None) query keeps the held value — the freshness gate expires
    it, giving one-missed-query tolerance.
- **Base-source selection** in `update()`: if toggle ON and
  `last_osm_speed_kph ≥ 30` (MIN_SPEED_LIMIT) and fresh (≤ 10 s):
  - base = OSM speed, `inference_mode = 'osm'`
  - the `gs_osm` sticky-hold machinery is bypassed while OSM base is active
    (OSM already carries the expressway's real limit; G/S promote is a proxy
    for exactly this data)
  - otherwise: existing `gs_osm` / `lane_count` path runs unchanged.
- **Snapping bypass** (correctness-critical): `_STANDARD_SPEEDS`
  `[30,40,50,60,80,100,120]` is a China ladder — a US 45 mph (72 km/h) limit
  would snap UP to 80, 70 mph (113) up to 120. When the base source is OSM:
  - target = OSM speed rounded to nearest 5 km/h (not snapped to the ladder)
  - gradual transition steps ±10 km/h toward the exact target, keeping the
    existing intervals (2 s up / 3 s down), instead of walking the CN ladder.
  - Safety-cap immediate-clamp behavior unchanged.
- **Publish**: `inferenceMode: 'osm'` plus two new fields on
  `speedLimitState`: `osmSpeedLimit` (raw km/h, 0 when absent) and
  `osmTilesMissing` (bool, see §3a) for telemetry/rlog and UI.

## 3a. Missing-tiles warning (yellow, Driving panel)

When the toggle is ON but no offline tiles cover the driving area, the user
must learn to download them via COD (Connect). Two signals, one warning row:

- **Precise (onroad, GPS fix):** `osm_query.OsmTileReader` tracks
  `tile_missing` — set when a query finds *neither* the `offline_hw` nor the
  `offline` tile file on disk for the current position (distinct from
  "tile exists but still loading", which is not a missing tile). speedlimitd
  publishes it as `osmTilesMissing` on `speedLimitState` each cycle.
- **Coarse (offroad, no fix):** ui_mod's Driving panel checks whether the
  tile dirs (`/data/media/0/osm/offline_hw`, `/data/media/0/osm/offline`)
  contain any tile files at all.

The Driving panel renders a **yellow warning line under the toggle row**
(only when the toggle is ON):
"⚠ No offline map tiles for your area — download in Connect."
Shown when either signal indicates missing tiles (latest `osmTilesMissing`
from the plugin bus when available, else the coarse dir check). No warning
when the toggle is OFF.

## 4. Out of scope

- mapd binary / issue #88 (unchanged, still tracked separately).
- Tile downloading UI or logic (COD's job).
- mph display conversion (openpilot's existing IsMetric handling applies).
- Any change to CN default behavior: toggle OFF ⇒ byte-identical to today.

## 4a. Future enhancements (not in this feature)

The offline tiles are mapd-generated; the difference between our direct
reader and the mapd Go binary is the runtime engine on top. Capabilities the
binary has that `osm_query.py` lacks, in value order for US/EU users:

1. **Bearing-aware, sticky way matching** — stateful CurrentWay tracking +
   connected-way traversal + heading-based direction rejection. Biggest
   robustness win (divided highways / frontage roads — the US analogue of
   the CN stacked-ring mismatch). Implementable in `osm_query.py` (tiles
   carry node sequences; GPS heading is available).
2. **Upcoming-limit look-ahead** — walk the way graph ahead to announce
   speed-limit changes with distance, enabling smooth decel before a drop.
3. **Directional maxspeed** — forward/backward limit selected by heading
   (needs `maxSpeedBackward` in our capnp read + bearing from item 1).
4. **Conditional (time-based) limits; map-based curve speeds** — lowest
   priority; curve speeds stay vision-only per the vision-only constraint.

**Preferred path for these: adopt the mapd Go binary as the engine — gated
on pfeiferj shipping the configurable slotted-reader option (mapd issue
#88).** Until that ships, the binary's hardcoded shadow carState read
panic-flaps under load and there is no free msgq reader slot on 0.11.x for
a safe slotted build, so items 1–3 would have to be implemented in
`osm_query.py` if needed sooner. Once #88 lands (and a carState slot budget
check passes), wiring the binary in replaces per-item reimplementation.

## 5. Error handling

| Failure | Behavior |
|---|---|
| No tiles on disk | `query()` → None → vision inference (today's path) + yellow warning in Driving panel (§3a) |
| Way has no maxspeed | OSM speed not stored → vision inference |
| GPS invalid | No queries run → vision inference |
| OSM speed stale (>10 s) | Freshness gate fails → vision inference |
| OSM speed < 30 km/h | Rejected as implausible → vision inference |
| Param write fails (first-fix default) | Logged, retried on next fix; toggle behaves as OFF meanwhile |

## 6. Testing

Unit tests (existing speedlimitd test harness):

1. Default resolution: CN fix → writes `'0'`; EU/US fix → writes `'1'`;
   existing file never overwritten; no GPS → no write, OFF behavior.
2. Base selection: toggle ON + fresh OSM speed → base = OSM,
   `inference_mode == 'osm'`; toggle OFF → identical to current behavior with
   same inputs.
3. Staleness: OSM speed older than 10 s → falls back to lane-count.
4. Way-change clearing: identity change to maxspeed-less way clears stored
   speed.
5. Snap bypass: OSM 72 km/h → displayed 70 (not 80); stepping ±10 toward
   target at correct intervals.
6. Safety caps: curve cap / reactive cap below OSM speed still win;
   OSM never lifts them.
7. gs_osm interplay: OSM base active suppresses G/S sticky hold; OSM absent →
   G/S hold works as today.
8. Missing-tiles flag: query at position with no tile file on disk →
   `osmTilesMissing` published true; tile present (even mid-load) → false;
   toggle OFF → warning row not rendered regardless of flag.
