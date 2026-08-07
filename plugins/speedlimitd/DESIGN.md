# Speed Limit Daemon — Design & Implementation

`speedlimitd` is a **hybrid plugin**: a background process (`speedlimitd.py`,
5 Hz, `only_onroad`) that fuses several speed sources into a single
`speedLimitState` message on the plugin bus, plus a set of UI/planner **hooks**
that read that message to draw the on-screen sign and cap cruise speed. It has
no external process dependencies (`dependencies: []`) — it reads pre-downloaded
offline OSM tiles directly (`osm_query.py`), not the mapd Go binary.

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
                    └► OSM tile query (0.2 Hz) ─► wayRef / name / G|S class

modelV2 ─► infer_lane_count ─► lane_count_stable ─► lane_count_limit ─┐
        ├► vision_speed_cap (≤2-lane narrow cap) ────────────────────┤
        └► curvature_speed_cap (distance-aware curve cap) ───────────┤
                                                                      ├─► min() ─► speedLimitState (5 Hz, plugin bus)
livePose ─► measured a_y ─► reactive-a_y cap ─────────────────────────┤            │
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

Despite the plugin.json description and a stale module docstring that mention a
three-tier OSM/YOLO/inference *priority*, the live logic (`update()`) is a
**`min()` over candidate speeds**, not a priority cascade. Each candidate is
`(speed_kph, source_id, confidence)`; the lowest speed wins:

| Candidate | source id | confidence | Included when |
|---|---|---|---|
| YOLO sign reading | 1 | 0.80 | `yolo_speed ≥ 30` — **placeholder, currently always 0** |
| Curvature look-ahead cap | 4 | 0.70 | `curvature_cap ≥ 30` |
| Reactive measured-a_y cap | 4 | 0.70 | `react_cap ≥ 30` |
| Base inference (OSM / G-S promote / lane-count) | 2 | `lane_conf` | always (`max(inferred, 30)`) |

OSM *maxspeed* is **not** a candidate in its own right — it never gets its own
source id. When `OsmDataIntegration` is on and a fresh, plausible reading
exists, it instead supplies the *value* the base-inference candidate (source
id 2) carries, ahead of the G-S promote and lane-count paths — see [OSM Data
Integration](#osm-data-integration-opportunistic-base-source) below. Source id
4 is the safety class (either curve cap); it has no entry in the `slot0.capnp`
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
multi-lane road whose outer line probability momentarily dips isn't capped.

## G/S expressway promotion + sticky hold

On stacked/elevated geometry the matched OSM way churns (field-measured at
~37 flips in 8 min) while the vision lane count stays stable, so OSM road-type is trusted
**only** for controlled-access expressways. A way is in **G/S mode** iff its ref
matches the expressway grammar `^[GS](\d{1,2}|\d{4})$` (`is_gs_expressway_ref`
— note 3-digit G312/S203 *surface* guodao/shengdao are excluded) **or** its OSM
highway class is `motorway`.

In G/S mode the limit comes from the existing `infer_speed_from_road_type`
promote (ref class × lane count, `lane_count_stable ≥ 3`):

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
| Margin rule (path 1) | `GS_RELEASE_MARGIN_M = 8 m`, `GS_RELEASE_MARGIN_QUERIES = 2` | matched way is decisively closer than the held G/S way (or held ref absent → +inf) for 2 consecutive OSM queries. Uses `refDistances` from `osm_query`; an absolute distance gate on the held way was proven insufficient (held S1 only 13.5 m off at ramp entry) |
| Lane-drop (path 2) | `GS_LANE_DROP_S = 1.5 s` | held ref absent **and** raw lane count ≤2 continuous — two independent exit signals, no OSM-distance dependence |
| Narrow section | — | `lane_count_stable ≤ 2` releases immediately (a ramp must obey the narrow limit, never a stale 100/120) |

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

### Freshness gate + floor (`_osm_base_active`)

OSM replaces the base only when **all** of:

- the toggle (`osm_integration_enabled`) is on,
- `last_osm_speed_kph >= 30.0` — the same 30 km/h floor as `MIN_SPEED_LIMIT`,
  rejecting implausible tags,
- `now - last_osm_speed_t <= 2.0 * self._osm_query_interval` — **two query
  intervals**; queries run at 5 s cadence, so this is a 10 s freshness
  window. One missed/`None` query keeps the limit; a sustained loss (two
  consecutive misses) falls back to vision.

### Base-priority order

In `update()`, `osm_base = self._osm_base_active(now)` is checked **before**
`gs_mode`:

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

### Vision narrow-cap skip under OSM base

The ≤2-lane `vision_cap_stable` cap is applied only when `not osm_base and
self.vision_cap_stable > 0 and self.lane_count_stable < 3`. An authoritative
posted OSM limit is not overridden by the narrow-road heuristic — a
legitimate 2-lane rural road is 90–100 km/h in the US/EU, well above the
40 km/h the heuristic would otherwise impose.

### CN-ladder snap bypass

`_STANDARD_SPEEDS = [30, 40, 50, 60, 80, 100, 120]` is a China-specific
display ladder. Snapping an OSM value to it is actively wrong outside China —
snapping *rounds toward the nearest rung*, and for a value like a US 45 mph
limit (72 km/h) the nearest rung is **up**, to 80: speedlimitd would display
and enforce a higher number than what's actually posted. When the active base
is OSM (`osm_display = osm_base and source == 2`):

- **Target** is the OSM speed rounded to the nearest 5 km/h
  (`round(speed_limit / 5.0) * 5`), not snapped to the ladder.
- **Gradual transition** steps **±10 km/h** toward that target, clamped so it
  never overshoots, at the same cadence as every other source
  (`_STEP_DOWN_INTERVAL = 3 s`, `_STEP_UP_INTERVAL = 2 s`) — instead of
  walking `_step_speed_limit`'s ladder.
- Every other source (lane-count, G-S, YOLO) keeps the existing CN-ladder
  snap/step unchanged.

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
which is *not* missing). On every OSM query cycle (0.2 Hz, gated on GPS
validity — independent of the toggle), the daemon reads that flag and writes
it to the `OsmTilesMissing` param (`'1'`/`'0'`) **only on the first query or
when the value changes** (not every cycle, to avoid needless param-file
churn). Nothing in speedlimitd itself reads this param back — it exists
purely for ui_mod's Driving-panel warning (see `plugins/ui_mod/DESIGN.md`).

### New publish fields

`speedLimitState` gains two telemetry fields, published every cycle
regardless of toggle state:

| Field | Meaning |
|---|---|
| `osmSpeedLimit` | `round(last_osm_speed_kph, 1)` — held OSM maxspeed in km/h, `0` when none/expired |
| `osmTilesMissing` | last `OsmTileReader.tile_missing` reading |

`inferenceMode` gains a third value, `'osm'` (OSM base active), alongside the
existing `'gs_osm'` and `'lane_count'`.

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
`livePose`, but only when that reading is fresh this tick — it delivers the
derated target exactly at the apex, where the model tends to cut the inner line.

The function returns a **raw, unsnapped** km/h value floored at 30, and 0 when
the raw value is ≥100 (curve far/mild enough to need no slowdown). It is
snapped to a standard speed only at the publish site.

### Reactive: measured-a_y backstop (`_update_reactive_cap`)

Proactive capping can't see late-appearing curves or plant amplification (a
recorded curve measured 3.6 m/s² where vision capped nothing), and the BMW
lateral controller no longer ISO-cancels in curves, so the **ISO 3.0 m/s²
defense lives here**. It low-passes measured `|a_y| = |v·yawRate|` from
`livePose` (`REACT_TAU = 0.3 s`). When it exceeds the threshold
(`MapdReactLatAccel`, default 2.5, clamp [1.8,3.0], 0 disables) continuously for
`REACT_ENGAGE_S = 0.5 s` and `v_ego ≥ REACT_MIN_SPEED (~29 km/h)`, it engages
`v = v_ego·sqrt(threshold/|a_y|) − REACT_HYST_MS (1 m/s)`. While engaged the cap
only **moves down or holds** (never chases a_y up), floored at `REACT_MIN_SPEED`.
Release ramps back up at `REACT_RELEASE_RATE = 1 m/s²` after
`REACT_QUIET_S = 2 s` below `threshold − 0.3`, or immediately if `livePose` goes
stale (`REACT_LIVEPOSE_STALE_S = 1 s`) — a stalled localizer must not latch a
cap forever.

Both curve caps enter the same `min()` as **safety-class** (source 4) sources:
they inherit the planner hook's gas-suspend and are in the lead-override-
protected class — a faster lead never lifts a curve cap (route 2fd).

## Temporal accumulators & the display ladder

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
- **Display step ladder.** The published limit changes one standard step at a
  time — `_STEP_DOWN_INTERVAL = 3 s`, `_STEP_UP_INTERVAL = 2 s` per rung — via
  `_step_speed_limit`. **Safety caps bypass this**: a tightening curve or
  reactive cap clamps the displayed limit down immediately.

## Enforcement & gas override (`planner_hook.on_v_cruise`)

The hook drains `speedLimitState` from the bus and returns a possibly-reduced
`v_cruise`. It **only ever lowers or holds** — `floored_target < v_cruise`
returns the target, otherwise `v_cruise` is unchanged; DCC comfort-shapes the
deceleration (no artificial ramp). Key rules:

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
- **Lead override.** If `radarState.leadOne` travels >10%
  (`LEAD_OVERRIDE_THRESHOLD`) above the limit, a *non-safety, non-inferred*
  confirmed limit is skipped (likely wrong data). It does **not** bypass safety
  caps, inferred limits, or an active gas hold.
- **Confirmation** is sticky, starts **True** onroad, and toggles only on an
  explicit `toggle_confirm` command (UI sign tap → `speedlimit_cmd_ui`, or a
  steering-wheel command → `speedlimit_cmd_car`), 1 s debounced. It never
  auto-resets on limit change, disengage, or process restart.

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
| `reactCapEngaged` | reactive a_y cap engaged |
| `reactCap` | reactive cap value (km/h, 0 = off) |
| `reactLatAccel` | filtered measured `|a_y|` (m/s²) |
| `osmSpeedLimit` | held OSM maxspeed (km/h, 0 = none/expired) |
| `osmTilesMissing` | no offline tile file for the current area |

## Configuration

Params are files in the plugin's `data/` dir (runtime
`/data/plugins-runtime/speedlimitd/data/`), read via `config.read_plugin_param`;
the two lateral-accel params are refreshed every 5 s so a UI change applies
without a restart.

| Param | Default | Note |
|---|---|---|
| `ShowSpeedLimitSign` | `1` (true) | Draw the on-screen sign (enforcement unaffected); re-checked ~every 2 s by the overlay |
| `MapdCurveTargetLatAccel` | `1.5` | Proactive curve-cap target a_y (m/s²), clamp [1.0, 3.0] |
| `MapdReactLatAccel` | `2.5` | Reactive measured-a_y threshold (m/s²), clamp [1.8, 3.0]; **0 disables** the reactive cap |
| `OsmDataIntegration` | `'0'` (China) / `'1'` (elsewhere), region-resolved once on first GPS fix | Use OSM `maxspeed` as the base speed source when fresh — see [OSM Data Integration](#osm-data-integration-opportunistic-base-source) |
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
