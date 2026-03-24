# catpilot plugins

Plugin packages for [catpilot](https://github.com/catpilot-dev/catpilot). Integrated into catpilot releases starting from `v0.10.3` — plugins are automatically installed on first boot.

## Plugins

| Plugin | Type | Description |
|--------|------|-------------|
| `bmw_e9x_e8x` | car | BMW E82/E90 car interface — VIN detection, cruise, lane change, torque learning |
| `c3_compat` | hybrid | Comma 3 compatibility (AGNOS 12.8, STM32F4 panda, DRM display, MSGQ fix) |
| `lane_centering` | hook | Lane centering curvature correction with kP/kD tuning |
| `mapd` | process | OSM map daemon for speed limits, curves, and road context |
| `model_selector` | hook | Download and swap driving models from Software panel |
| `speedlimitd` | hybrid | Conditional speed control — road type inference, vision cap, per-country speed tables |
| `network_settings` | hook | Per-SSID proxy and static IPv4 configuration |
| `bus_logger` | process | Capture plugin bus messages to cereal for logging |
| `ui_mod` | hook | Settings panels (Driving, Vehicle, Plugins), home screen (drive stats, route map, emblem) |
| `screen_capture` | hook | Tap-to-capture screenshots with camera icon overlay |

## Directory Layout

```
plugins/
├── install.sh                     # Plugin installer (→ /data/plugins-runtime/)
├── logos/                         # Brand emblems and icons for all supported cars
│   ├── emblems/                   #   Color SVG+PNG (512px)
│   └── icons/                     #   White-on-transparent PNG (168px)
├── plugins/                       # Plugin packages
│   ├── bmw_e9x_e8x/
│   ├── bus_logger/
│   ├── c3_compat/
│   ├── lane_centering/
│   ├── mapd/
│   ├── model_selector/
│   ├── network_settings/
│   ├── screen_capture/
│   ├── speedlimitd/
│   └── ui_mod/
└── docs/                          # Architecture and technical docs
```

## Installation

Plugins are installed automatically with catpilot. To update manually:

```bash
ssh comma
cd /data/plugins
git pull origin dev && bash install.sh
```

install.sh copies plugins to `/data/plugins-runtime/`, injects cereal schemas and services, clears bytecode caches, and writes a restart marker. The plugin daemon (plugind) detects the marker when offroad and restarts managed processes and the UI.

## Managing Plugins

### From the device

Settings → Plugins panel lets you enable/disable plugins with toggles.

### From SSH

```bash
# Disable a plugin
touch /data/plugins-runtime/speedlimitd/.disabled

# Re-enable
rm /data/plugins-runtime/speedlimitd/.disabled
```

## Writing a Plugin

A plugin needs a directory in `plugins/` with a `plugin.json` manifest and one or more Python modules.

### Minimal example

```
my_plugin/
├── plugin.json
└── my_hook.py
```

**plugin.json:**
```json
{
  "id": "my_plugin",
  "name": "My Plugin",
  "version": "1.0.0",
  "type": "hook",
  "hooks": {
    "planner.v_cruise": {
      "module": "my_hook",
      "function": "on_v_cruise",
      "priority": 50
    }
  }
}
```

**my_hook.py:**
```python
def on_v_cruise(v_cruise, v_ego, sm):
    return min(v_cruise, 120.0)
```

### Plugin types

- **hook** — registers callbacks on catpilot hook points, runs in existing processes
- **process** — runs as a managed daemon via plugind (PID files in `/data/plugins-runtime/.pids/`)
- **car** — registers a car interface via monkey-patching (no opendbc fork needed)
- **hybrid** — combination of hook + process

### Key rules

- **ALL UI imports MUST be lazy** — import inside the hook function body, not at module level. Hooks load during `__init__` mid-import.
- **Plugin params** go in `/data/plugins/<id>/data/`, never `/data/params/d/` (openpilot wipes unknown keys on boot).
- **Fail-safe by default** — if your hook raises an exception, the default value is returned and other plugins continue.

### Available hooks

See the [catpilot README](https://github.com/catpilot-dev/catpilot#hook-call-sites) for the full list of 26 hook call sites.

## Testing

```bash
PYTHONPATH=. uv run pytest
```

A pre-push hook runs all tests automatically. Tests that require openpilot/opendbc auto-skip when those dependencies are unavailable.

## License

MIT
