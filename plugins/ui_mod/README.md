# ui_mod — UI Customizations

**Type**: hook

## What it does

Adds settings panels, home screen widgets, drive tracking, and the branded experimental mode button.

### Settings Panels

- **Driving** — personality selector, lane centering toggle, conditional speed control toggle, curve comfort setting
- **Vehicle** — brand emblem + car name, car-specific settings (populated by car plugins via `ui.vehicle_settings` hook)
- **Plugins** — enable/disable plugins with toggles, check for updates

### Home Screen

- **Left column**: Route map showing last drive GPS trace with auto-zoom, start/end markers, and offline tile support
- **Right column**: Brand emblem + car name, last drive stats (distance, duration, engaged %), past 7 days summary (drives, distance, hours)
- **Ecosystem update badge** — notification dot when plugin updates are available

### Drive Tracker

Real-time drive statistics accumulator that runs during every drive:
- Accumulates distance, duration, and engagement time from cereal messages at ~2 Hz (gated on deviceState)
- Captures GPS start/end coordinates and route trace (50m minimum between points)
- Writes summary JSON on offroad transition for instant display without parsing qlogs
- Minimum thresholds: 5s duration AND 100m distance (prevents stationary ignition cycles from overwriting real data)

### Experimental Mode Button

Branded replacement for the stock onroad experimental/chill mode button. Shows vehicle emblem with lane centering visual feedback ring.

## Hooks

| Hook | Function | Description |
|------|----------|-------------|
| `ui.settings_extend` | on_settings_extend | Add Driving, Vehicle, Plugins panels |
| `ui.home_extend` | on_home_extend | Add route map + drive stats widgets |
| `ui.main_extend` | on_main_extend | Set default panel, wire plugins callback |
| `ui.state_tick` | on_state_tick | Tick drive tracker with cereal messages |
| `ui.onroad_exp_button` | on_exp_button | Branded experimental mode button |

## Key Files

```
ui_mod/
  plugin.json          # Plugin manifest
  hooks.py             # Hook handlers
  driving_panel.py     # Driving settings panel
  vehicle_panel.py     # Vehicle settings panel
  plugins_panel.py     # Plugins settings panel
  drive_stats.py       # DriveStatsWidget (home screen right column)
  drive_tracker.py     # DriveTracker (real-time stats accumulator)
  route_map.py         # Route map rendering (GPS trace + tiles)
  route_map_widget.py  # RouteMapWidget (home screen left column)
  exp_button.py        # Branded experimental mode button
```
