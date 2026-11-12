# Speed Limit Daemon — Design & Implementation

`speedlimitd` is a **hybrid plugin**: a background process (`speedlimitd.py`,
5 Hz, `only_onroad`) that fuses several speed sources into a single
`speedLimitState` message on the plugin bus, plus a set of UI/planner **hooks**
that read that message to draw the on-screen sign and cap cruise speed. It has
no required process dependencies (`dependencies: []`) — it reads pre-downloaded
offline OSM tiles directly (`osm_query.py`) for road identity. When the mapd
plugin runs, speedlimitd also observes `mapdOut` (telemetry only, see [mapd
observation](#mapd-observation-phase-1-2026-08-18)) and, on G/S expressways
only, reads `mapdExtendedOut`'s road-ahead path for the [G/S map curve
look-ahead](#gs-map-curve-look-ahead-map_curvepy-2026-11-09). Both subscriptions
are conditional on the services being injected, so speedlimitd runs unchanged
without mapd.

The governing constraint is **vision-only by default in China**: OSM
*maxspeed* and OSM *geometry* are treated as unreliable there (roads stack in
2D on elevated sections; posted limits are sparse/stale), so by default OSM
is used only for **road identity** — name, and whether the way is a numbered
G/S expressway — and everything about the actual limit on ordinary roads
comes from the camera. An opt-in `OsmDataIntegration` toggle (default ON
outside China, OFF inside — see [OSM Data
Integration](#osm-data-integration-opportunistic-base-source) below) lets
OSM's posted `maxspeed` become the base source directly; vision remains the
fallback whenever OSM isn't fresh, plausible, or enabled.

## Signal flow

```
gpsLocationExternal ─► country auto-detect (bbox) ─► per-country speed table
                    └► OSM tile query (1 Hz) ─► wayRef / name / G|S class

modelV2 ─► infer_lane_count ─► lane_count_stable ─► lane_count_limit ─┐
        ├► vision_speed_cap (≤2-lane narrow cap) ────────────────────┤
        └► curvature_speed_cap (distance-aware curve cap) ───────────┤
mapdExtendedOut.path ─► map_curve_cap (G/S only, toggle) ────────────┤
                                                                      ├─► min() ─► speedLimitState (5 Hz, plugin bus)
deviceMotion ─► measured a_y ─► reactive-a_y cap ─────────────────────────┤            │
        └► measured κ ─► curve-cap virtual apex point ────────────────┘            │
                                                                                   ▼
                                          planner.v_cruise hook ─► cap cruise speed
                                          ui.render_overlay hook ─► draw sign + tap
```

The daemon never *is* the speed source in the controller; it publishes
`speedLimitState`, and the planner hook (`planner_hook.on_v_cruise`) decides
whether and how much to cap. The sign overlay (`ui_overlay.py`) reads the same
message and handles the confirm/cancel tap.

## Source fusion — "lowest valid reading wins"

Despite the plugin.json description, which still mentions a three-tier
OSM/YOLO/inference *priority*, the live logic (`update()`) is a
**`min()` over candidate speeds**, not a priority cascade. Each candidate is
`(speed_kph, source_id, confidence)`; the lowest speed wins:

| Candidate | source id | confidence | Included when |
|---|---|---|---|
| YOLO sign reading | 1 | 0.80 | `yolo_speed ≥ 30` — **placeholder, currently always 0** |
| Curvature look-ahead cap | 4 | 0.70 | `curvature_cap ≥ 30` |
| Reactive measured-a_y cap | 4 | 0.70 | `react_cap ≥ 30` |
| G/S map curve cap | 4 | 0.70 | `MapdCurveLookahead` ON and a shaped map cap exists (gate `ok`, see [below](#gs-map-curve-look-ahead-map_curvepy-2026-11-09)) |
| Base inference (OSM / G-S promote / lane-count) | 2 | `lane_conf` | always (`max(inferred, 30)`) |

OSM *maxspeed* is **not** a candidate in its own right — it never gets its own
source id. When `OsmDataIntegration` is on and a fresh, plausible reading
exists, it instead supplies the *value* the base-inference candidate (source
id 2) carries, ahead of the G-S promote and lane-count paths — see [OSM Data
Integration](#osm-data-integration-opportunistic-base-source) below. Source id
4 is the safety class (any curve cap); it has no entry in the `slot0.capnp`
`Source` enum (which predates it) because the live message is a plain dict on
the plugin bus, not the capnp struct. `MIN_SPEED_LIMIT = 30` km/h floors every
source — no real road is lower.

## Lane-count-first inference and the ×20 rule

`infer_lane_count(model_msg)` counts lane lines from `modelV2.laneLineProbs`
(confidence > 0.3): 4 lines → base 4, 3 → 3, 2 → 2, else 1. Two corrections:

- **Bounded-road demote (Fix G, 2026-08-05):** exactly 3 visible lines *and*
  both road edges "bounded" (each `roadEdgeStd < 0.9` and hugging its outermost
  line within 1.5 m at ~10 m) → force **2 lanes**. A physically closed 3-line
  road holds two lanes; construction squeezes and barriers read here too, which
  is accepted (slowing for them is correct). Pinned by
  `test_demote_accepted_wide_road_characterization`.
- **Edge boost:** base ≥ 3 *and* the car sits next to a confident road edge
  (`_near_road_edge`) → `+1` lane (cap 4), compensating for an unseen far side.
  Deliberately gated at base ≥ 3 (not ≥ 2) so an honest 2-line ramp reading is
  never inflated.

`lane_count_limit(lane_count_stable)` then maps the debounced count to a limit.
The design law for **non-expressway** roads is roughly *lane_count × 20 km/h*,
with a 30 floor:

| Stable lane count | Limit |
|---|---|
| 1 | **30** km/h (floored, not 20) |
| 2 (or ≤2) | **40** km/h |
| 3 | **60** km/h (`LANE_COUNT_LIMIT_3`) |
| ≥4 | **80** km/h (`LANE_COUNT_LIMIT_4`) |

The ≤2 branch delegates verbatim to `infer_speed_from_road_type`'s narrow-road
shortcut (2 → 40, 1 → 30), because vision can't distinguish a through road from
a link/ramp at ≤2 lanes.

### Narrow-road vision cap

`vision_speed_cap` is a separate, stricter ≤2-lane cap for links/ramps: it
requires the **inner** lane pair (indices 1,2) confident at > 0.6, counts the
outer pair (0,3) only at > 0.5 (rejecting faint echoes of an adjacent main
road), and returns **30 km/h for 1 visible lane, 40 for 2**. It is applied
(as a further `min`) only when `lane_count_stable < 3`, so a genuinely
multi-lane road whose outer line probability momentarily dips isn't capped —
and never while OSM carries the base or G/S mode is held (see [Vision
narrow-cap skip](#vision-narrow-cap-skip-under-osm-base-and-in-gs-mode)).

## G/S expressway promotion + sticky hold

On stacked/elevated geometry the matched OSM way churns (field-measured at
~37 flips in 8 min) while the vision lane count stays stable, so OSM road-type is trusted
**only** for controlled-access expressways. A way is in **G/S mode** iff its ref
matches the expressway grammar `^[GS](\d{1,2}|\d{4})$` (`is_gs_expressway_ref`
— note 3-digit G312/S203 *surface* guodao/shengdao are excluded) **or** its OSM
highway class is `motorway`.

In G/S mode the limit comes from the existing `infer_speed_from_road_type`
promote (ref class × lane count, `lane_count_stable ≥ 3`). It is recomputed on
a G/S match only while `lane_count_stable ≥ 3`, or when a stretch starts
(`_gs_limit_kph == 0`, reset on every release): a camera ≤2-lane reading inside
a held stretch keeps the last wide promote, while a stretch entered on a road
already read as narrow promotes from that count (→ 40). See the *Narrow section*
release below.

- **G** ref → `motorway` → nonurban table → **120 km/h**
- **S** ref → `trunk` → nonurban table → **100 km/h**
- ref-less `motorway` OSM class (urban elevated expressway, 中环路-style) has its
  context demoted freeway→city → **80 km/h** (trunk-grade), never 120.

The G/S classification is **sticky** so a momentary flip to a stacked non-G/S
way can't drop the expressway limit out from under the car (the 100→80
transient). Release happens on **any** of:

| Release path | Constant | Meaning |
|---|---|---|
| Absolute ceiling | `GS_STICKY_S = 30 s` | time since last G/S match |
| Continuous absence | `GS_RELEASE_CONT_S = 10 s` | non-G/S matches unbroken this long (an alternating flicker keeps resetting it → stickiness preserved) |
| Margin rule (path 1) | `GS_RELEASE_MARGIN_M = 8 m`, `GS_RELEASE_MARGIN_S = 5 s` | matched way is decisively closer than the held G/S way (or held ref absent → +inf) continuously for 5 s (time-based since 2026-08-18; equals the old 2-consecutive-query window at the former 5 s cadence). Uses `refDistances` from `osm_query`; an absolute distance gate on the held way was proven insufficient (held S1 only 13.5 m off at ramp entry) |
| Lane-drop (path 2) | `GS_LANE_DROP_S = 1.5 s` | held ref absent **and** raw lane count ≤2 continuous — two independent exit signals, no OSM-distance dependence |
| Narrow section | — | `lane_count_stable ≤ 2` **and** the tile reader no longer matching the G/S way (absence run active) releases immediately. Camera-only narrowing on a still-matched G/S way neither releases the hold nor takes a wide promote down, and the ≤2-lane vision cap is skipped in G/S mode (2026-11-09, route 534: 214 mid-road 100/120→40 flaps over 138 routes; on real exits the tile reader left a median ~4 s later, and planner_hook's same-road floor already waited for that before braking). A stretch entered narrow still promotes from the narrow count (40); `_gs_limit_kph` resets on release |

`_eval_gs_margin_release` runs once per OSM query (in `_ingest_osm_result`); a
genuine G/S re-match clears the force-release latch and re-establishes the held
ref.

## OSM Data Integration (opportunistic base source)

An opt-in extension of the same OSM tile read that already runs for G/S
identity above: `osm_query.py` has always returned a `speedLimit` field on
every query; when this feature is on, that value is stored and, if fresh and
plausible, replaces the base-inference candidate (source id 2) outright,
ahead of the G-S promote and lane-count paths. It adds no new candidate/source
id — it changes what value source id 2 carries.

### Param + first-fix region default

`OsmDataIntegration` (`'1'`/`'0'`) lives in speedlimitd's own data dir
(`plugin_data_dir('speedlimitd')`, survives `Params::clearAll`) and is
re-read on the same 5 s cadence as the lateral-accel params (`_read_params`),
so a UI toggle takes effect without a restart.

While the param file is absent, `_resolve_osm_default` writes a **one-time**
region default on the first valid GPS fix (inside `update()`'s country
auto-detect block): `'0'` if the GPS-detected country is `cn`, `'1'`
otherwise. It **never overwrites an existing file** — once set (by the
daemon or by the user), the param is purely user-controlled from then on. A
write failure leaves `_osm_default_resolved` False, so the daemon retries the
default on the next GPS fix; until it resolves, `osm_integration_enabled`
stays at its initial `False` (safe/OFF).

### Storage (`_ingest_osm_result`)

Every OSM query result carrying `speedLimit > 0` (m/s) is converted to km/h
(`speed_ms * 3.6`) and stored as `last_osm_speed_kph` / `last_osm_speed_t`
(`time.monotonic()`) — **regardless of whether the toggle is on**, since the
underlying query always runs (it's also how G/S identity is read):

- **Holds across same-road sub-segments that lack the tag.** A query that
  matches the *same* `road_id` but returns no `speedLimit` leaves the prior
  value in place; the freshness gate below is what eventually expires it.
- **Clears on a road change to a way without `maxspeed`.** When the matched
  `road_id` changes and the new result has no `speedLimit`,
  `last_osm_speed_kph` resets to `0.0` — the held value belonged to the old
  road.

### Freshness gate + floor

Outside China, OSM replaces the base only when **all** of:

- the toggle (`osm_integration_enabled`) is on,
- `last_osm_speed_kph >= 30.0` — the same 30 km/h floor as `MIN_SPEED_LIMIT`,
  rejecting implausible tags,
- `now - last_osm_speed_t <= 2.0 * self._osm_query_interval` — **two query
  intervals**; the freshness window is an absolute `OSM_SPEED_FRESH_S = 10 s`
  (decoupled from the query cadence 2026-08-18 when queries moved 5 s → 1 s)
  window. One missed/`None` query keeps the limit; a sustained loss (two
  consecutive misses) falls back to vision.

In China — and until the first GPS fix has *resolved* a country — two further
conditions apply on top of these. All of it is now one predicate, `_osm_gate`;
see the next section.

### OSM maxspeed gate (CN, 2026-08-10)

`_osm_gate(now, gs_mode) -> (trusted, reason)` is the single predicate deciding
whether a posted OSM maxspeed replaces the base inference. It feeds arbitration,
the ≤2-lane vision-cap bypass, display rounding and telemetry, so those four
cannot drift apart.

Outside China the gate is toggle + 30 km/h floor + 10 s freshness, exactly as
shipped 2026-08-07. In China — and until the first GPS fix has resolved a
country (`country_detected` still `False`), which fails safe rather than
falling through to the permissive path — the posted value must additionally sit
on a confirmed G/S expressway (`gs_mode` true **and**
`is_gs_expressway_ref(self._osm_gate_ref)`) and be at least `GS_OSM_MIN_KPH`
(60).

**The test is `country_detected`, NOT `self.country` being non-empty** — do not
"simplify" it back. `country_from_gps` only matches the bboxes it has tables for
(`cn`, `de`, `au`), so a fix in the US, UK, France, Japan or Canada resolves to
`country == ''` **permanently**, with `country_detected` `True`. Keying the
permissive branch on the string being non-empty therefore applied the
China-only G/S restriction to most of the world forever — no US/EU way ref
matches `^[GS](\d{1,2}|\d{4})$`, so the gate never opened while the toggle still
read as ON. A resolved-but-unmatched country is *definitively not China* (the CN
bbox spans lat 18-54, lon 73-135) and takes the permissive path; only the
genuinely unresolved boot window is strict.

`_osm_gate_ref` is a gate-scoped mirror of the matched way ref, not
`last_way_ref` itself. It's written alongside `last_osm_speed_kph` in the same
branch of `_ingest_osm_result`, so the gate's two inputs can never diverge in
age: it follows `last_osm_speed_kph`'s HOLD semantics (written on a match,
deliberately **not** cleared on a no-match tile query) rather than
`last_way_ref`'s prompt-clear semantics. That's what stops a single missed
tile query from flipping the trust decision and wobbling the enforced limit,
without freezing the release paths below — `last_way_ref` itself keeps
clearing immediately on a no-match, because the `gs_mode` release machinery
depends on that prompt clear: the 30 s sticky ceiling, the absence timer, the
lane-drop path, and the margin rule all key off it, and holding it would stall
all four during a tile gap.

This targets the two systematic map-matching failures found in the 2026-08-07
audit, neither of which any safety cap can catch, since both are wrong numbers
on straight roads:

| Failure mode | Excluded by |
| --- | --- |
| Viaduct tag matched to the surface road beneath (华夏中路, 金海路, 申江路) | no G/S ref on the surface way |
| 40 km/h ramp sign matched to a 高速/高架 mainline | the 60 km/h floor |

Requiring `gs_mode` rather than the ref alone inherits every G/S release guard:
the narrow-section release, the margin rule, `gs_lane_drop`, and the continuous-absence
timer.

`osmTrusted` and `osmRejectReason` (`''｜disabled｜no_data｜stale｜not_gs｜low_value`)
are published every tick, alongside the unconditional `osmSpeedLimit`, so rlogs
record what OSM claimed even when it was rejected. The CN default for
`OsmDataIntegration` remains OFF; `low_value` and `not_gs` rates are the evidence
for ever revisiting that.

### Base-priority order

In `update()`, `gs_mode` (the G/S sticky-hold release state) is computed
first, then `osm_trusted, osm_reject_reason = self._osm_gate(now, gs_mode)`
decides whether OSM replaces the base. `osm_base = osm_trusted` is then
checked **before** `gs_mode` in the `if`/`elif` that picks `inferred_speed`:

```
osm_base  → inferred_speed = round(last_osm_speed_kph);  inference_mode = 'osm'
gs_mode   → inferred_speed = self._gs_limit_kph;          inference_mode = 'gs_osm'
otherwise → inferred_speed = lane_count_limit(...);       inference_mode = 'lane_count'
```

i.e. **`osm` > `gs_osm` > `lane_count`**. The G/S sticky-hold bookkeeping
(`_gs_limit_kph`, absence timers, margin release) keeps running unconditionally
underneath — it simply isn't consulted for the displayed value while OSM base
is active, so a toggle-off or a freshness drop falls straight back into
whatever G/S state it had been tracking.

### Vision narrow-cap skip under OSM base and in G/S mode

The ≤2-lane `vision_cap_stable` cap is applied only when `not osm_base and
not gs_mode and self.vision_cap_stable > 0 and self.lane_count_stable < 3`. An
authoritative posted OSM limit is not overridden by the narrow-road heuristic —
a legitimate 2-lane rural road is 90–100 km/h in the US/EU, well above the
40 km/h the heuristic would otherwise impose. In G/S mode (since 2026-11-09,
route 534) the held promote already carries 40 for a stretch entered narrow,
and a wide promote must not be capped by a camera ≤2-lane reading on a way the
tile reader still matches.

### CN-ladder snap bypass

`_STANDARD_SPEEDS = [30, 40, 50, 60, 80, 100, 120]` is a China-specific
display ladder. Snapping an OSM value to it is actively wrong outside China —
snapping *rounds toward the nearest rung*, and for a value like a US 45 mph
limit (72 km/h) the nearest rung is **up**, to 80: speedlimitd would display
and enforce a higher number than what's actually posted. When the active base
is OSM (`osm_display = osm_base and source == 2`):

- **Published value** is the OSM speed rounded to the nearest 5 km/h
  (`round(speed_limit / 5.0) * 5`), not snapped to the ladder.
- Every other source (lane-count, G-S, YOLO) keeps the CN-ladder snap.
- Neither path shapes the *transition* any more — the value is published at
  once and `planner_hook` ramps the enforced ceiling. See §Enforcement.

### Safety caps unchanged and always winning

The proactive curvature cap and reactive measured-a_y cap are computed
exactly as before and enter the same `candidates` `min()` (source 4) and the
same post-transition immediate-clamp override, **regardless of `osm_base`**.
OSM can lower the effective limit but can never raise a value past an active
safety cap — an OSM-posted 100 km/h limit still yields to a 60 km/h curve cap.

### `OsmTilesMissing` mechanism

`osm_query.OsmTileReader.tile_missing` is set True when a query finds
**neither** the `offline_hw` nor `offline` tile file on disk for the current
position (distinct from "tile exists but is still loading in the background",
which is *not* missing). On every OSM query cycle (1 Hz, gated on GPS
validity — independent of the toggle), the daemon reads that flag and writes
it to the `OsmTilesMissing` param (`'1'`/`'0'`) **only on the first query or
when the value changes** (not every cycle, to avoid needless param-file
churn). Nothing in speedlimitd itself reads this param back — it exists
purely for ui_mod's Driving-panel warning (see `plugins/ui_mod/DESIGN.md`).

### New publish fields

`speedLimitState` gains telemetry fields, published every cycle regardless of
toggle state:

| Field | Meaning |
|---|---|
| `osmSpeedLimit` | `round(last_osm_speed_kph, 1)` — held OSM maxspeed in km/h, `0` when none/expired |
| `osmTilesMissing` | last `OsmTileReader.tile_missing` reading |
| `osmTrusted` | `_osm_gate`'s verdict (2026-08-10) — whether `osmSpeedLimit` is actually driving the base inference |
| `osmRejectReason` | `''` when trusted, else the first failing gate condition: `disabled\|no_data\|stale\|not_gs\|low_value` |

`inferenceMode` gains a third value, `'osm'` (OSM base active), alongside the
existing `'gs_osm'` and `'lane_count'`.

### mapd observation (Phase 1, 2026-08-18)

speedlimitd subscribes to `mapdOut` and publishes eleven `mapd*` fields into
`speedLimitState` for comparison against the tile-derived values beside them.
This is telemetry only: `mapd_source.telemetry_from_mapd` output is published
and discarded, and the offline tile reader still drives every control path.

The subscription is conditional on `mapdOut` being present in the injected
`SERVICE_LIST`: the schema/service injection is not stock, so a catpilot
`git reset --hard` or a `.disabled` on mapd can remove it, and an unguarded
`SubMaster` would crash-loop speedlimitd — leaving the car with no speed-limit
data at all. When absent, the same eleven keys still publish in their
dead-mapd shape (`mapdAlive: False`).

`mapdRoadContext` is mapd's own urban/rural verdict, observed alongside
`mapdHwClass` so the S100+ reclassification concern can be checked from the
rlog.

`mapdRefAgree` — mapd's bearing-aware way match versus our nearest-polyline
match — was the metric for the planned Phase 2 identity cutover. Route 531
(2026-11-09) showed the two matchers fail in complementary ways (mapd holds the
old way after exits and drops out at ~25 m GPS offset; the tile reader snaps to
cross streets at junctions), so the cutover was dropped: the tile reader keeps
road identity, and Phase 2 became the G/S-only map curve look-ahead below. See
`docs/superpowers/specs/2026-08-18-mapd-integration-design.md` and
`docs/superpowers/specs/2026-11-09-mapd-gs-curve-lookahead-design.md`.

## Curve caps

### Proactive: `curvature_speed_cap` (distance-aware braking)

Reads the model's predicted `orientationRate.z` / `velocity.x` over the path,
bounded by the tighter of a time horizon (T_IDXS index ≤30, ≈8.8 s) and a
**confidence distance** (first point from the far end with `1/(1+yStd) > 0.6`,
capped 100 m). Beyond that, predictions are extrapolation noise.

Rather than mapping the single max curvature to a speed, it makes a **per-point
pass**. For each meaningfully-curved point *i* (`κ = |yaw|/v > CURVE_GATE
= 0.003`):

- `target_ay = max_lat_accel · interp(κ, [0.02,0.035], [1.0, 0.75])` — the
  comfort target is derated toward `TIGHT_CURVE_FACTOR = 0.75` on hairpin-class
  curvature.
- `v_curve = sqrt(target_ay / κ)` — comfortable speed *at* the curve.
- `v_now = sqrt(v_curve² + 2·COMFORT_BRAKE·d_i)` with `COMFORT_BRAKE = 0.8`
  m/s² and `d_i = position.x[i]` — the speed allowed **now** so a comfortable
  deceleration still bleeds to `v_curve` by the time the curve is reached.

The binding cap is `min(v_now)` over points, so a near-mild and a far-sharp
curve compete on an achievable-profile footing and the cap **tightens as the
curve nears** instead of a last-second one-shot drop. `max_lat_accel` is the
`MapdCurveTargetLatAccel` param (default 1.5, clamp [1.0,3.0]).

A **measured-curvature virtual apex point** is folded in at
`d=0` (no braking-headroom term) using the driver's own `κ_meas = |yaw|/v` from
`deviceMotion`, but only when that reading is fresh this tick — it delivers the
derated target exactly at the apex, where the model tends to cut the inner line.

The function returns a **raw, unsnapped** km/h value floored at 30, and 0 when
the raw value is ≥100 (curve far/mild enough to need no slowdown). It is
snapped to a standard speed only at the publish site.

### Reactive: measured-a_y backstop (`_update_reactive_cap`)

Proactive capping can't see late-appearing curves or plant amplification (a
recorded curve measured 3.6 m/s² where vision capped nothing), and the BMW
lateral controller no longer ISO-cancels in curves, so the **ISO 3.0 m/s²
defense lives here**. It low-passes measured `|a_y| = |v·yawRate|` from
`deviceMotion` (`REACT_TAU = 0.3 s`). When it exceeds the threshold
(`MapdReactLatAccel`, default 2.5, clamp [1.8,3.0], 0 disables) continuously for
`REACT_ENGAGE_S = 0.5 s` and `v_ego ≥ REACT_MIN_SPEED (~29 km/h)`, it engages
`v = v_ego·sqrt(threshold/|a_y|) − REACT_HYST_MS (1 m/s)`. While engaged the cap
only **moves down or holds** (never chases a_y up), floored at `REACT_MIN_SPEED`.
Release ramps back up at `REACT_RELEASE_RATE = 1 m/s²` after
`REACT_QUIET_S = 2 s` below `threshold − 0.3`, or immediately if `deviceMotion` goes
stale (`REACT_LIVEPOSE_STALE_S = 1 s`) — a stalled localizer must not latch a
cap forever.

Both curve caps enter the same `min()` as **safety-class** (source 4) sources:
they inherit the planner hook's gas-suspend and are in the lead-override-
protected class — a faster lead never lifts a curve cap (route 2fd). Since
2026-11-12 the daemon also publishes `safetyCap` (km/h, snapped): the lowest
safety-class candidate (curve cap ≥ 30, reactive cap when engaged, map-curve cap
when enabled and shaped), 0 when none, published **even when a guessed limit is
lower than it**. `speedLimit`/`safetyCapped` alone hide a cap above the guess,
and a lead override of that guess would lift it; under an override the hook
outputs `min(v_cruise, safetyCap)` (no offset, immediate) — see Lead override.

### G/S map curve look-ahead (`map_curve.py`, 2026-11-09)

On G/S expressways vision's confident path ends at ~100 m, short of what a
comfortable slowdown from 100–120 km/h needs. There, and only there (OSM
geometry is trustworthy on G/S expressways, not on Chinese city roads),
speedlimitd computes its own curve cap from mapd's road-ahead path
(`mapdExtendedOut.path`, 1 Hz, mapd's current way plus predicted next ways).

**Gate** (`map_curve_gate`, first failing reason wins, published as
`mapCurveGate`): `off_gs` (not in G/S mode) → `no_gps` (speedlimitd's fix older
than 1.5 s) → `tile_lapsed` (no G/S ref from the tile reader within
`TILE_BLIP_TOLERANCE_S = 2 s`) → `mapd_absent` → `sel_type` (mapd's way
selection not `current`/`predicted`) → `ref_mismatch` (mapd's way shares no G/S
ref part with any tile ref of the last 2 s) → `path_stale` (path position
unchanged for `PATH_FRESH_S = 2.5 s`) → `ok`. The caller adds `no_path_ahead`
(car past the path end or > 5 m before its start) and `error` (any exception,
logged once). The tile reader keeps road identity: mapd's path is used only
while it agrees.

**Cap** (`map_curve_cap`, pure): the path is projected into local metres,
resampled every `MAP_CURVE_RESAMPLE_M = 20 m`, and each vertex's curvature is
the net chord-heading change across a `MAP_CURVE_WINDOW_M = 50 m` window
divided by the chord-midpoint separation — recomputed from geometry rather than
mapd's per-node `curvature`, so a single misplaced OSM node cancels instead of
reading as a sharp curve. Vertices within `MAP_CURVE_MAX_LOOKAHEAD_M = 800 m`
with κ ≥ `MAP_CURVE_GATE_KAPPA = 0.003` (shared with the vision cap) get the
vision cap's law with its own budget: `target_ay` from `MapdCurveTargetLatAccel`
derated toward `TIGHT_CURVE_FACTOR`, `v_now = √(target_ay/κ + 2·COMFORT_BRAKE·d)`.
The minimum is the cap — unrounded and with **no ≥100 km/h cut-off**, unlike
the vision cap.

**Shaping** (`_shape_map_curve`): a cap that first appears starts from the
current speed and descends no faster than `COMFORT_BRAKE`; it rises at once and
holds `MAP_CURVE_HOLD_S = 3 s` across gate dropouts. Safety caps reach the
planner un-ramped, so this keeps a gate opening with a curve already near from
landing as a setpoint step.

**Toggle**: `MapdCurveLookahead` (Driving panel *Map Curve Look-ahead (G/S)*,
default OFF). Off, the cap is computed and published as telemetry only; on, the
shaped value enters `min()` as source 4. `mapdExtendedOut` is not logged; the
`mapCurve*` keys in `speedLimitState` are the record.

**Field notes.** Routes 532/534 (toggle ON, 148 s gate `ok` on S20): no cap
ever bound. S20 mainline curvature peaked at ~0.0016 (R ≈ 620 m), below the
0.003 gate; above it, an R ≈ 620 m curve's comfort speed is ~110 km/h anyway.
On gentle expressways like S20 the look-ahead therefore never binds; it matters
only on tighter G/S geometry. Known gap: there is no lateral-offset gate — on
534 the gate read `ok` for 4 s while the car drove an R ≈ 120 m interchange loop
17–33 m off mapd's S20 path (`mapCurveLat`); harmless there (straight mainline
ahead, cap 0), but the path did not describe the car's road.

## Temporal accumulators

Vision is noisy, so nearly every input is debounced before it moves the limit:

- **Lane-count debounce.** Widening (UP) uses a **threshold-hold window** (Fix
  I1): a window opens while raw ≥3, tracks the *minimum* raw seen, and commits
  that minimum after a sustained 1.5 s (conservative — commits 3 on mixed 3/4);
  any raw ≤2 frame closes it, then it re-arms. Narrowing among wide counts
  (staying ≥3) keeps a directional debounce: **2 s window while curving, 5 s
  straight**.
- **Leaky narrow-band (≤2) accumulator.** Demotion *into* the narrow band uses a
  leaky integrator, not a single timer: `+dt` while raw ≤2, bleed
  `NARROW_DECAY·dt (0.5)` while raw ≥3, clamped `[0, NARROW_ACCUM_CAP = 3]`.
  Reaching `NARROW_CONFIRM_S = 3 s` commits `lane_count_stable = 2` (a fixed 2,
  not the min raw — the commit fires during the noisiest lane-loss frame). A
  committed widening zeroes the accumulator so a re-narrow needs a fresh 3 s.
- **Vision narrow cap** holds 1 s of stability (`vision_cap_stable`).
- **Curvature cap hold + relax ladder.** A tighter/equal/first reading applies
  immediately and refreshes a 3 s hold; when the hold expires and raw is looser,
  the cap **step-relaxes up the standard ladder** `[30,40,50,60,80,100,120]` at
  2 s per rung (releasing to off above 80) rather than snapping to 0.
- **No display ladder.** The published limit is the fused value, applied at
  once (CN-ladder snapped, or rounded to 5 km/h under an OSM base). Transition
  shaping lives in `planner_hook`, which ramps the *enforced* ceiling — see
  below. The ladder that used to live here set the sign's step interval and the
  brake schedule with one number; splitting them is what removed the brake
  spikes. Safety caps still clamp the published limit down immediately.

## Enforcement & gas override (`planner_hook.on_v_cruise`)

The hook drains `speedLimitState` from the bus and returns a possibly-reduced
`v_cruise`. It **only ever lowers or holds** — `floored_target < v_cruise`
returns the target, otherwise `v_cruise` is unchanged. Key rules:

- **Jerk-limited ceiling.** The enforced target does not jump between limits.
  `_ceiling_ms` slides toward `limit + offset` under a trapezoidal acceleration
  profile (`_advance_ceiling`), so both its value and its slope stay continuous
  and it arrives with zero slope. Descent `CEIL_A_DOWN = 0.5` m/s² at
  `CEIL_J_DOWN = 0.5` m/s³ (~24 s for 80 → 40). Setpoint *gap* is what drives
  DCC deceleration on this car (corr +0.746), so a step in the target is a
  brake spike — the old 3 s ladder produced three of them per drop.

  **Only the descent is shaped.** A rising limit is applied at once. The hook
  only ever lowers `v_cruise`, so raising the ceiling merely stops capping —
  DCC's own ~+0.5 m/s² envelope shapes the acceleration from there, and
  ramping the release only delayed it. This also means a safety cap lifts on
  the tick the curve ends, instead of holding the car down while a ramp
  catches up.

  The stop budget is measured against the error remaining **after** the tick's
  travel (`|err| - |rate|·dt`). The textbook `sqrt(2·j·|err|)` is optimistic by
  one tick and the shortfall compounds: measured on 88 → 46, the ceiling landed
  still carrying −0.15 m/s², a 6× jerk violation on the final tick — the same
  spike, relocated to the end of the ramp. On overshoot the value is pinned to
  the target and the slope bleeds out over the following ticks rather than
  being zeroed outright, for the same reason.

  The ceiling is a **pure function of the limit history**: it never reads
  `v_ego` or `v_cruise`, so a given limit change always produces the same
  trajectory. It keeps advancing while the gas is pressed (the gas *floor* is
  what suspends enforcement), and it survives a `road_id` change — resetting it
  there would reintroduce the jump. `_reset_all()` clears it, so the next valid
  limit is applied immediately rather than ramped from stale state.

  **Safety caps bypass the ramp** — `safetyCapped` assigns the target directly.
  A tightening curve cannot wait out a 24 s ramp. `CEIL_A_DOWN` (0.5) is kept
  strictly below `COMFORT_BRAKE` (0.8) so a safety cap's own distance-aware
  profile is never gentler than the comfort ramp — a safety cap always wins by
  simply being steeper, and the two can never fight.

  The descent rate is the comfort knob. It started at 0.8 (= `COMFORT_BRAKE`)
  and was lowered to 0.5 after route 45f, where the ramp was confirmed binding
  on real drops. The cost is overspeed distance: 80 → 40 takes ~24 s / ~410 m,
  and a 120 → 40 drop takes ~49 s / ~1 km. Do not lower it further without
  weighing that.

  **Descent anchor.** On the tick the target *drops*, the ceiling is clamped to
  `min(ceiling, max(v_ego, target))`. Without it every descent began at the old
  limit + offset, so the ceiling spent seconds in dead air above the car before
  the cap reached it and anything happened — measured on route 45f, an 80 → 40
  at 55 km/h gave ~20 s and ~300 m inside the 40 zone with the car untouched,
  and 100 → 30 at 48 km/h gave ~35 s.

  This is a step in the **cap**, not in the setpoint **gap**: clamping 88 → 55
  while the car is doing 55 leaves zero error, so there is nothing for DCC to
  react to. `max(v_ego, target)` keeps it from dragging a slower driver down and
  holding them there; `min` keeps it from ever handing out a *higher* cap.

  It fires only on the tick the target drops. Re-anchoring every tick would
  ratchet the cap down with a decelerating car. So the ceiling's pure-function
  property is given up at exactly one instant, and the ramp is a pure function
  of the limit from there on. Bite delay across the real route-45f drops went
  from 3.7–22.2 s to one tick.

  **Floor-release re-anchor (2026-11-11).** The ceiling keeps descending while a
  floor (baseline or gas) holds the output at the car's speed, so when the floor
  released the output fell in one update to a ceiling that had never been in
  effect — route 536: a road change reset the baseline and `vCruiseOut` went
  68.1 → 54.4 km/h at a request near 0, braking −1.08 m/s². The rule: on the
  tick a floor that was holding the output above the ceiling lets go, the
  ceiling re-anchors to `min(held output, max(v_ego, target))` — only upward,
  never above what was being output, never below the target — and descends from
  there at its normal rate. A rising limit is still immediate, and **safety caps
  are excluded** (they assign the target directly and keep acting at once, so a
  gas floor cleared under a source-4 cap still drops to the cap). Logged in
  `anchored`/`anchorFrom`/`anchorTo` (here `anchorTo` > `anchorFrom`).
- **Comfort offset** (`_effective_offset_percent`): the enforced target is the
  limit **+15%** below 80 km/h, **+10%** at/above 80 km/h, and **+0%** (exact)
  when `safetyCapped`. *(This corrects the old README's +40/+30/+10 tiers,
  which no longer exist.)*
- **Gas override.** `carState.gasPressed` suspends enforcement entirely (all
  sources, incl. safety caps) and raises a **hold floor** to current speed; on
  release enforcement resumes from there, ratcheting the floor down with the
  driver until eased back to target.
- **Baseline (road-continuity) floor.** For inferred limits (`source == 2`, not
  safety-capped) *with an OSM road identity*, a running-max floor rejects
  spurious downward jitter when lane lines fade. Without a road_id (unnamed
  ramp/link) the hold is invalid — the inferred/vision cap is allowed to slow
  the car.
- **Lead override.** If `radarState.leadOne` travels above the ENFORCED target
  (limit + offset), a non-safety confirmed limit of
  ANY source (inferred included since 2026-11-12) is skipped (likely wrong data).
  Trigger with hysteresis (2026-11-12 replay): entry needs
  `lead_kph > target_kph x (1 + LEAD_ENTER_MARGIN)` (0.05); once ON a tick stays
  qualifying while `lead_kph > target_kph x (1 + LEAD_STAY_MARGIN)` (0.0). The
  old `limit x 1.10` (`LEAD_OVERRIDE_THRESHOLD`, retired) sat at or under the
  target, so on 538 seg 25-26 (guessed 80, target 88, car at 88, lead 86-92)
  the override flipped up to 6 times a minute; confirmed limits get the same
  rule. A gas floor still held after release no longer blocks the override
  (538 seg 19 +24.5-31: it stayed off for 6 s while the lead ran 63-67 against
  a target of 46 and the car coasted 61 -> 55); a gas PRESS still clears it.
  Route 538: a lane-count guess of 40 pulled the car 83 -> 50 km/h at -0.5 m/s²
  while the lead ran 50-63 km/h and pulled away; the baseline floor does not
  cover unnamed roads. Half of the measured qualifying episodes were one-to-five
  ticks, so it is debounced with its own state (`_lead_q_s`, `_lead_on`,
  `_lead_last_q_t`, `_lead_held_ms`; `_floor_held_ms` keeps its meaning):
  ON after `LEAD_OVERRIDE_ENTER_S = 0.5` s of continuous qualifying ticks, OFF
  `LEAD_OVERRIDE_HOLD_S = 1.0` s after the last one. A safety-capped tick, a gas
  press (the gas floor owns the release) or `_reset_all` clear it at once, with
  no hold and no re-anchor. During ON the ceiling stays a pure function of the
  limit. On the hold-expiry tick the ceiling re-anchors once,
  `min(v_ego at last ON tick, max(v_ego, target))`, BEFORE the floors are
  applied, so a baseline floor below v_ego is never the release output; if v_ego
  is below target the ceiling is untouched. While ON (override and hold) the
  output is `min(v_cruise, max(safetyCap, held gas floor))`
  whenever the daemon's `safetyCap` > 0 (no gas floor counts as 0), so a fast
  lead never lifts a curve/reactive/map-curve cap, including one above the
  guessed limit. The gas floor term keeps a masked cap below it from stepping the
  output down under the driver (the floor is <= v_ego, so it never accelerates).
  The baseline floor is deliberately excluded: it is `min(baseline, v_ego)` and
  rises with the car, so under an override it would let a lead's lift carry
  through a tightening cap. Each ON tick records `_lead_held_ms = min(v_ego, override output)`, so the
  release anchor can never be above what the override was outputting (override
  bounded at 50 with the car at 58 would otherwise jump to 58 on release). A
  payload WITHOUT `safetyCap` (older daemon) fails closed: nothing qualifies, an
  ON override is cleared, no hold. A safety-capped tick still gets the cap directly and
  immediately.
  The lead is read from `leadOne.present` (0.11.2 renamed `status`, upstream
  #38339). Until 2026-11-11 the hook still read `status`, which raises on
  0.11.2 and was swallowed, so the override never fired there (routes 517–536).
- **Confirmation** is sticky, starts **True** onroad, and toggles only on an
  explicit `toggle_confirm` command (UI sign tap → `speedlimit_cmd_ui`, or a
  steering-wheel command → `speedlimit_cmd_car`), 1 s debounced. It never
  auto-resets on limit change, disengage, or process restart.

**Safety-cap bound on the capped path (2026-11-12 follow-up).** The hook sees only
the daemon's minimum, so a safety-class cap ABOVE a guessed limit (not
`safetyCapped`) was enforced by nothing outside the lead override while the
ceiling descended toward the guess or the baseline floor held the car at v_ego
(route 537 seg 10: the cap tightened 80/60/50/40 under the floor and the hook
acted only when it reached the guess; 538 seg 28 52.9 -> 50). With `safetyCap` > 0
in the payload: (1) the ceiling is clamped to the cap once (`_ceiling_ms,
_ceiling_rate = cap, 0.0`), after the lead-release and floor-release anchors and
before the floors are applied, so nothing lifts it back and the descent resumes
from the cap; (2) the output is `min(max(ceiling, floor), max(cap, held gas
floor))` - only a held gas floor may stay above the cap (user decision; it is
most of the measured above-cap time), the baseline floor may not; (3)
`_floor_held_ms` records the bounded output, so a floor release with the cap
clearing on the same tick anchors at or below the previous output. A cap
appearing below the current output steps it to the cap at once (curve caps stay
immediate). Telemetry `safetyBound` marks those ticks. A missing field or
`safetyCap` 0 leaves the path unchanged (the override path still fails closed).
Side effect: a safety cap at or below the guess that the daemon does not flag
`safetyCapped` (a map-curve cap snapped to the guess, e.g. guess 40 / cap 40) now
binds without the comfort offset (46 -> 40), as a visible safety cap does.
Masked-to-visible hand-over: while the cap is masked, the clamped ceiling keeps
descending toward the guess at <= 0.5 m/s2, so the output can sit below the cap
(after ~12 s masked it reaches the guess target, e.g. 46). When the cap later
becomes the visible minimum (the guess rose above it) the output rises to the cap:
the normal release of a risen limit, present before this change too.

### Enforcement telemetry (`speedLimitEnforce` topic)

`speedLimitState` describes what the **limit** is. The enforcement side — the
ceiling, the anchor, the floors, whether the cap even bound — used to be
invisible, so a drive could only be analysed by inferring the cap from
`longitudinalPlan.aTarget`, which is the MPC output with lead-following mixed
in. `on_v_cruise` therefore publishes one sample per tick on **every** path
(no gaps in the series), best-effort — a publish failure can never disturb
enforcement.

| field | meaning |
|-------|---------|
| `state` | `no_data` / `unconfirmed` / `no_limit` / `gas` / `lead_override` (ON and qualifying) / `lead_hold` (ON, inside the 1 s hold) / `capped` / `not_binding` |
| `safetyBound` | True on a tick where the safety-cap bound moved the ceiling or lowered the output (False on every other tick, incl. gas / no_data / unconfirmed); also True on `lead_override` / `lead_hold` ticks whose output the cap lowered below `v_cruise` |
| `leadKph` | `leadOne.vLead` in km/h when `present`, else 0.0 |
| `vEgo`, `vCruiseIn`, `vCruiseOut` | km/h; `capActive` = did the hook actually lower it |
| `limit`, `source`, `safetyCapped`, `safetyCap` | echo of the driving `speedLimitState` sample, so the series is self-contained |
| `target` | limit + offset, km/h — the ramp's destination |
| `ceiling`, `ceilingRate` | the ramped cap (km/h) and its slope (m/s²) — jerk compliance is directly measurable |
| `anchored`, `anchorFrom`, `anchorTo` | one-tick anchor event; invisible otherwise |
| `baselineFloor`, `gasFloor` | km/h, 0 = inactive |

`bus_logger` discovers topics by scanning `/tmp/plugin_bus/` and re-scans
periodically, so this needs no registration — samples land in `pluginBusLog`
in the rlog.

## Hooks (from `plugin.json`)

| Hook | Module.function | Role |
|---|---|---|
| `planner.v_cruise` | `planner_hook.on_v_cruise` | Cap cruise speed to the confirmed limit (never raises) |
| `ui.render_overlay` | `ui_overlay.on_render_overlay` | Draw the speed-limit sign; handle tap-to-confirm |
| `ui.hud_set_speed_override` | `ui_overlay.on_hud_set_speed_override` | No-op — returns default; MAX block keeps showing the user's set cruise speed |
| `device.health_check` | `planner_hook.on_health_check` | Report whether the `speedlimitd` process is alive |

The process itself (`speedlimitd`, `only_onroad`) is declared under
`processes`. `ui_overlay.on_state_subscriptions` exists but is a no-op and is
**not** registered in `plugin.json` (speedLimitState moved to the plugin bus);
there is **no** `planner.subscriptions` / `ui.state_subscriptions` registration
— *(both corrections to the old README's hook table).*

## `speedLimitState` telemetry (plugin bus dict, 5 Hz)

Published by `_sl_pub.send({...})`. `slot0.capnp` reserves `CustomReserved0`
with an older subset; the live dict is the authoritative payload:

| Field | Meaning |
|---|---|
| `speedLimit` | displayed limit (km/h, snapped, post display-ladder) |
| `source` | winning source id: 1 = YOLO, 2 = base inference, 4 = safety cap |
| `confirmed` | user confirm state (sticky) |
| `confidence` | winning candidate's confidence (0–1) |
| `inferenceMode` | `'osm'` (OSM maxspeed base), `'gs_osm'` (expressway promote), or `'lane_count'` |
| `yoloSpeed` | YOLO reading (km/h; placeholder, 0) |
| `inferredSpeed` | base lane-count/G-S limit before caps (km/h) |
| `highwayType` | ref-derived class (`motorway`/`trunk`/`''`) |
| `osmHwType` | OSM `highway=*` class from offline_hw tiles |
| `wayRef` | matched OSM ref (e.g. `G2`) |
| `roadName` | matched OSM road name |
| `laneCount` | `lane_count_stable` (debounced) |
| `laneWidth` | smoothed lane width (m) from `lane_centering` plugin |
| `laneWidthClass` | road-class hint from lane width |
| `curvatureCap` | snapped proactive curve cap (km/h, 0 = none) |
| `safetyCapped` | a safety cap is the active/binding source |
| `safetyCap` | lowest safety-class candidate, snapped (km/h, 0 = none), published even when a guessed limit is lower; bounds the lead override |
| `reactCapEngaged` | reactive a_y cap engaged |
| `reactCap` | reactive cap value (km/h, 0 = off) |
| `reactLatAccel` | filtered measured `|a_y|` (m/s²) |
| `osmSpeedLimit` | held OSM maxspeed (km/h, 0 = none/expired) |
| `osmTilesMissing` | no offline tile file for the current area |
| `osmTrusted` | `_osm_gate` verdict — whether `osmSpeedLimit` is driving the base inference |
| `osmRejectReason` | `''｜disabled｜no_data｜stale｜not_gs｜low_value` — why the gate rejected it, when it did |
| `mapd*` (11 keys) | mapd observation, telemetry only — see [mapd observation](#mapd-observation-phase-1-2026-08-18) |
| `mapCurveGate` | map curve gate verdict: `ok` or the first failing reason (`off_gs`, `no_gps`, `tile_lapsed`, `mapd_absent`, `sel_type`, `ref_mismatch`, `path_stale`, `no_path_ahead`, `error`) |
| `mapCurveActive` | the `MapdCurveLookahead` toggle (the cap may act) |
| `mapCurveCap` | raw map curve cap (km/h, 0 = no curve above the gate in the look-ahead) |
| `mapCurveShaped` | shaped cap — the value that enters `min()` when active (km/h, 0 = none) |
| `mapCurveDist` / `mapCurveKappa` / `mapCurveLatLon` | binding point: along-path distance (m), curvature (1/m), `[lat, lon]` |
| `mapCurvePathAhead` | mapd path length ahead of the car, capped at 800 m |
| `mapCurveLat` | car's perpendicular distance from mapd's path (m; not gated) |

## Configuration

Params are files in the plugin's `data/` dir (runtime
`/data/plugins-runtime/speedlimitd/data/`), read via `config.read_plugin_param`;
`_read_params` (the two lateral-accel params, `OsmDataIntegration`,
`MapdCurveLookahead`) re-reads every 5 s so a UI change applies without a
restart.

| Param | Default | Note |
|---|---|---|
| `ShowSpeedLimitSign` | `1` (true) | Draw the on-screen sign (enforcement unaffected); re-checked ~every 2 s by the overlay |
| `MapdCurveTargetLatAccel` | `1.5` | Proactive curve-cap target a_y (m/s²), clamp [1.0, 3.0] |
| `MapdReactLatAccel` | `2.5` | Reactive measured-a_y threshold (m/s²), clamp [1.8, 3.0]; **0 disables** the reactive cap |
| `OsmDataIntegration` | `'0'` (China) / `'1'` (elsewhere), region-resolved once on first GPS fix | Use OSM `maxspeed` as the base speed source when fresh — see [OSM Data Integration](#osm-data-integration-opportunistic-base-source) |
| `MapdCurveLookahead` | `'0'` (off) | Let the G/S map curve cap act (source 4); off = telemetry only — see [G/S map curve look-ahead](#gs-map-curve-look-ahead-map_curvepy-2026-11-09) |
| `OsmTilesMissing` | daemon-written only, no default file | Last `OsmTileReader.tile_missing` reading; consumed by ui_mod's Driving-panel warning, not read back by speedlimitd |

The whole plugin is enabled/disabled from **Settings → Plugins** (framework
toggle), not a param. *(The old README's `MapdSpeedLimitControlEnabled` param
is dead — it is read nowhere in the code.)*

## Per-country speed tables

`speed_tables/*.toml` — `cn` (China, GB 5768), `de` (Germany, StVO), `au`
(Australia). Selected by GPS `bbox` (`country_from_gps`); default is China. Each
holds `urban`/`nonurban` road-class → `{multi, single}` limits, a
`default_fallback`, a `lane_width_class` table (lane width → road-class vote),
and the country `bbox`. Adding a country is a new TOML, no code change.

Only `cn` is road-validated (Shanghai). `de` and `au` ship as reasonable
table values but have never been driven — treat them as starting points, not
tuned configurations, and say so if anyone reports behavior there.

## Known modelV2 limitations

- **Ghost lane lines on parallel carriageways.** On roads running beside a
  parallel carriageway the model can hallucinate extra lane lines, over-counting
  lanes and briefly showing a too-high limit (false 80s). These are accepted
  modelV2 artifacts pending a better model — the post-narrow ceiling that once
  guarded against them (Fix F) was removed 2026-08-05 because its exit-release
  hold cost outweighed the protection.
- **Edge-lane vs middle-lane counts.** From an edge lane the far side of the
  road is hard to see, so the raw count under-reads; the edge boost compensates
  but a sustained edge-lane *misread* on a genuine expressway (base 2 for ≥3 s)
  can release a 100/120 G/S hold to 40 — an accepted trade for the counter
  reporting what vision actually sees on narrow roads, damped by the 3 s narrow
  accumulator and the display ladder.
- **Stacked/elevated OSM geometry.** Offline tiles carry no layer/bridge/
  altitude, so the matched way flickers among vertically-stacked roads; this is
  exactly why ordinary-road inference is lane-count-first and OSM road-type is
  trusted only for sticky G/S expressways.
