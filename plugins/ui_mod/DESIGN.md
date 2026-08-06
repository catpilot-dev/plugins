# UI Customizations — Design & Implementation

A `hook`-type plugin that decorates the openpilot UI process. It owns no
control logic: every hook runs inside the UI's own render/update loop and only
draws widgets, injects Settings panels, and reads status off the plugin bus.
All UI imports are lazy (inside the hook functions) — the plugin framework
loads `hooks.py` mid-import of the UI, so module-level UI imports would crash.

## Hooks

From `plugin.json`, all handlers in `hooks.py` unless noted:

| Hook | Function | What it does |
|---|---|---|
| `ui.settings_extend` | `on_settings_extend` | Registers two Settings panels: **Driving** (`DrivingLayout`) and **Plugins** (`PluginsLayout`), via `settings.add_panel`. Keeps the returned panel keys. |
| `ui.home_extend` | `on_home_extend` | Sets the home screen's left widget (`RouteMapWidget`) and right widget (`DriveStatsWidget`), and registers the eco/update-badge counter. |
| `ui.main_extend` | `on_main_extend` | Makes **Driving** the default Settings panel; wires the home "plugins" affordance to open the Plugins panel. |
| `ui.state_tick` | `on_state_tick` | Ticks the `DriveTracker` each UI frame; throttles target FPS (20 offroad, 60 onroad) on `started` transitions. |
| `ui.state_subscriptions` | `on_state_subscriptions` | Appends `pluginBusLog` to the UI's cereal subscriptions (used for replay of plugin-bus topics). |
| `ui.onroad_exp_button` | `on_exp_button` | Returns the branded `ExpButton` to replace the stock experimental-mode button. |
| `ui.render_overlay` (prio 40) | `road_info_overlay.on_render_overlay` | Draws the current road name / highway ref at the bottom of the onroad HUD. |
| `device.health_check` (prio 50) | `on_health_check` | Reports `{"ui_mod": {"status": "ok"}}`. |

## Panel-injection architecture

openpilot's Settings screen exposes `settings.add_panel(title, widget)`.
ui_mod calls it twice in `on_settings_extend` and holds onto the returned keys
so `on_main_extend` can set the default panel and open the Plugins panel on
demand. Each panel is a `Widget` that lazily builds a `Scroller` of list-view
items (`toggle_item`, `multiple_button_item`, `text_item`, `button_item`).

### Driving panel (`driving_panel.py`)

`DrivingLayout` rebuilds its scroller on every `show_event` (so newly
installed/removed plugins appear). The rows, in order:

1. **Personality** — `multiple_button_item`, writes the stock
   `LongitudinalPersonality` param; disabled when the car has no longitudinal
   control.
2. **Lane Keeping** — only if `lane_keeping` is installed and not
   `.disabled`; toggles the plugin's `LaneKeepEnable` param.
3. **Lane Centering in Turns** — if `lane_centering` present; toggles
   `LaneCenteringEnabled`.
4. **Speed Limit Sign** + **Road Info** — if `speedlimitd` present; toggle
   `speedlimitd`'s `ShowSpeedLimitSign` and ui_mod's own `RoadInfoOverlay`.
5. **Look Ahead Steering** — if `look_ahead` present; toggles
   `LookAheadEnabled`.
6. **Cruise Speed Memory** and **Consecutive Lane Changes** — display-only
   rows (rendered disabled; describe catpilot core behavior, no param
   written).
7. **Live Torque** / **Lateral Delay** — status text rows, only for
   torque-tuned brands in `torqued.ALLOWED_CARS`; poll `liveTorqueParameters`
   / `liveDelay` (10 s cache).
8. **Vehicle-specific rows** — appended by car plugins, see below.

A heading strip (brand icon + `CP.carFingerprint`) is drawn above the scroller
when a car is fingerprinted.

### Plugins panel (`plugins_panel.py`)

`PluginsLayout` scans `PLUGINS_RUNTIME_DIR` for `plugin.json` manifests and
renders one toggle per plugin (skipping `panel: false` and device-filtered
plugins). Enable/disable is a `.disabled` marker file in the plugin's runtime
dir; toggling one prompts a reboot (the plugin builder re-runs on next boot).
Plugins in `ESSENTIAL_PLUGINS` or carrying a `.enforced` marker render as
locked-on. A top **Plugin Updates** button runs `git fetch` / `rev-list`
against the plugins repo on the catpilot-aligned branch (background thread),
and on "UPDATE" does `git reset --hard` + `install.sh`, then offers a reboot.

## The `ui.vehicle_settings` dispatch pattern

ui_mod **owns the panel; car plugins contribute the rows.** Inside
`DrivingLayout._build_scroller`, once a car is fingerprinted
(`ui_state.CP is not None`), ui_mod dispatches:

```python
items = hooks.run('ui.vehicle_settings', items, CP)
```

Each car plugin registers a `ui.vehicle_settings` handler that appends its own
`toggle_item` / `multiple_button_item` widgets to `items` and returns the
list. The BMW plugin (`bmw_e9x_e8x`) is the current producer — its
`on_vehicle_settings` adds BMW-specific rows. This keeps car-specific UI in the
car plugin while ui_mod controls placement and styling, so the Driving panel
gains a vehicle section automatically for whatever car is detected, with no
ui_mod change. `hooks.py` registers the Driving and Plugins panels.

## Home screen widgets

- **`DriveStatsWidget`** (right column, `drive_stats.py`) — draws the brand
  emblem + model name, the **Last Drive** row, and the **Past 7 Days** row.
  Owns the shared `RouteMapRenderer` instance. Last-drive data is read from
  `.last_drive.json`, reloaded only when that file's mtime changes (i.e. after
  an offroad transition). Weekly stats are fetched from connect-on-device
  (COD): `GET http://localhost/v1.1/devices/{DongleId}/stats` on a background
  thread, retried until it succeeds.
- **`RouteMapWidget`** (left column, `route_map_widget.py`) — thin wrapper that
  renders the shared `RouteMapRenderer` if a trace is loaded, else a dark
  placeholder.
- **Eco/update badge** — `on_home_extend` registers a counter that sums
  `update_checker.get_update_status()`; the home screen shows a dot when > 0.

## Drive tracking (`drive_tracker.py`)

`DriveTracker` accumulates the drive stats live so nothing has to parse qlogs.
It registers an offroad-transition callback: **onroad → reset**, **offroad →
save**. It ticks from `ui.state_tick`, sampling only on `deviceState` updates
(~2 Hz, matching qlog resolution). Per tick it integrates `carState.vEgo` into
distance, adds wall time to duration, and — when `selfdriveState.enabled` —
into engaged distance/time (`dt` clamped to guard against process pauses). GPS
(`gpsLocationExternal`, valid-fix flag) fixes the start point, continuously
updates the end point, and appends to a trace with a 50 m minimum spacing; the
first fix's `unixTimestampMillis` is captured as `gps_time`.

On save it writes `.last_drive.json` atomically **only if** the drive was
≥ 5 s **and** ≥ 100 m (so ignition cycles don't overwrite a real drive); a
drive with no GPS lock inherits the previous drive's trace. It then
best-effort POSTs a brief summary to COD:
`POST http://localhost/v1/route/{route_id}/drive_stats` (route id resolved
from the newest realdata dir), fully guarded so no failure can escape `_save`.

## The branded emblem button (`exp_button.py`)

`ExpButton` replaces the stock onroad button. It loads two textures for the
detected brand from the repo's `logos/`: a white icon (`icons/<brand>.png`,
normal mode) and a color emblem (`emblems/<brand>.png`, experimental mode),
resized to fit the button. Tapping toggles the stock `ExperimentalMode` param
(only when `ExperimentalModeConfirmed` and the car has longitudinal control),
with a 2 s visual hold on the new state.

**Green ring = lane keeping working.** Each `_update_state`, the button reads
the `lane_keeping` topic off the plugin bus (`PluginSub(['lane_keeping'])`,
live) and, as a replay fallback, from `pluginBusLog` entries. `state ==
'anchor'` means the driver-side line is trusted and the position loop is live;
the ring is drawn (green, `RING_WIDTH` 12 px) only when that is true **and**
openpilot is `enabled`.

## Road info overlay (`road_info_overlay.py`)

`on_render_overlay` draws the current road identifier (highway ref, e.g.
`S20`) bottom-center of the HUD. It reads `wayRef` / `roadName` from
speedlimitd's `speedLimitState` topic over the plugin bus, opening a
`PluginSub` only when the `/tmp/plugin_bus/speedLimitState` socket exists and
dropping the sub when it disappears. Gated by ui_mod's own `RoadInfoOverlay`
param (default off); returns early (draws nothing) when disabled or when there
is no `wayRef`.

## Route map rendering (`route_map.py`)

`RouteMapRenderer` draws a static map of the last drive from **CartoDB dark
raster tiles** — this is the deployed choice. (An offline vector basemap was
tried and reverted — polylines were impractical on the C3; `offline_basemap.py`
is gone, only stale `__pycache__`/test artifacts remain.)

Web-Mercator math (`_lat_lng_to_tile_xy`, `_tiles_for_rect`) picks the zoom
(clamped `MIN_ZOOM`..`MAX_ZOOM`) that fits the whole trace within
`FIT_PADDING` of the rect, centered on the trace bounding-box midpoint. Tiles
are `@2x` (512 px) from `{s}.basemaps.cartocdn.com/dark_all/...`, downloaded to
`PLUGINS_RUNTIME_DIR/map_tiles/cartodb/z/x/y.png` on a background thread (old
cache wiped first — only the last drive is kept) and loaded as raylib textures
on the main thread. On top it draws the trace polyline, green/red
start/end markers, and a small URL caption.

## Plugin-bus topics & external endpoints consumed

| Source | Where | Used for |
|---|---|---|
| `lane_keeping` (plugin bus) | `exp_button.py` | `state == 'anchor'` → green ring |
| `pluginBusLog` (cereal) | `exp_button.py` | replay fallback for `lane_keeping` |
| `speedLimitState` (plugin bus) | `road_info_overlay.py` | `wayRef` / `roadName` overlay |
| COD `GET /v1.1/devices/{id}/stats` | `drive_stats.py` | weekly stats |
| COD `POST /v1/route/{id}/drive_stats` | `drive_tracker.py` | push per-drive summary |

## Configuration / params

ui_mod's **own** param is a file in its data dir
(`PLUGINS_RUNTIME_DIR/ui_mod/data/`, read/written via `config.read_plugin_param`
/ `write_plugin_param`). The Driving panel also read/writes params belonging to
*other* plugins and a couple of stock openpilot params.

| Param | Owner / store | Meaning |
|---|---|---|
| `RoadInfoOverlay` | ui_mod data dir | Road-name overlay on/off (default off) |
| `LaneKeepEnable` | lane_keeping data dir | Lane Keeping toggle |
| `LaneCenteringEnabled` | lane_centering data dir | Lane Centering toggle |
| `ShowSpeedLimitSign` | speedlimitd data dir | Speed-limit sign toggle |
| `LookAheadEnabled` | look_ahead data dir | Look Ahead Steering toggle |
| `LongitudinalPersonality` | openpilot `Params` | Driving personality |
| `ExperimentalMode` / `ExperimentalModeConfirmed` | openpilot `Params` | Emblem-button toggle + gate |

Plugin enable/disable in the Plugins panel is a `.disabled` marker file in each
plugin's runtime dir, not a param.

## Key files

```
ui_mod/
  plugin.json          # manifest — the hook table above
  hooks.py             # hook handlers (panels, home, tick, exp button, health)
  driving_panel.py     # DrivingLayout — Driving Settings panel + ui.vehicle_settings dispatch
  plugins_panel.py     # PluginsLayout — plugin enable/disable + updates
  drive_stats.py       # DriveStatsWidget — home right column, COD stats
  drive_tracker.py     # DriveTracker — live drive-stats accumulator + COD POST
  route_map.py         # RouteMapRenderer — CartoDB dark tiles + GPS trace
  route_map_widget.py  # RouteMapWidget — home left column wrapper
  exp_button.py        # ExpButton — branded emblem, lane-keeping green ring
  road_info_overlay.py # ui.render_overlay — road name/ref from speedlimitd
  tests/               # test_drive_tracker.py, test_route_map.py
```
