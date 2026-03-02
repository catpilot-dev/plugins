# openpilot-plugins

A lightweight plugin framework for [openpilot](https://github.com/commaai/openpilot). Extend openpilot with custom car support, speed limit enforcement, map integration, and more — without maintaining a full fork.

## Directory Layout

```
openpilot-plugins/
├── install.sh                     # Overlay installer
├── plugins/                       # Plugin packages (→ /data/plugins/)
│   ├── bmw_e9x_e8x/              # BMW E8x/E9x car interface
│   ├── c3_compat/                 # Comma 3 compatibility + custom UI
│   ├── lane_centering/            # Lane centering correction
│   ├── mapd/                      # OSM map daemon
│   ├── model_selector/            # Driving model selector
│   ├── speedlimitd/               # Speed limit enforcement
├── overlays/                       # Files overlaid into openpilot tree
│   ├── selfdrive/plugins/         #   Plugin framework (hooks, registry, builder)
│   ├── selfdrive/ui/onroad/       #   UI overlay modules (HUD, model, alerts)
│   └── cereal/custom.capnp        #   20 reserved struct slots for plugins
└── docs/                          # Architecture docs
```

## Quick Start

### Install

```bash
git clone https://github.com/OxygenLiu/openpilot-plugins.git
cd openpilot-plugins

# Preview what will be installed
bash install.sh --dry-run

# Install (auto-detects openpilot at /data/openpilot or ~/openpilot)
bash install.sh

# Or specify openpilot location
bash install.sh --target /path/to/openpilot
```

### On Comma 3

```bash
ssh comma
cd /data
git clone https://github.com/OxygenLiu/openpilot-plugins.git
cd openpilot-plugins && bash install.sh
sudo reboot
```

### Enable / Disable Plugins

```bash
# Disable a plugin
touch /data/plugins/speedlimitd/.disabled

# Re-enable
rm /data/plugins/speedlimitd/.disabled

# Reboot or restart openpilot to apply
```

## How It Works

### Hook System

The framework injects ~10 lines into 4 upstream openpilot files (controlsd, plannerd, card, manager). Each line calls `hooks.run()` at a defined extension point:

```python
from openpilot.selfdrive.plugins.hooks import hooks

# In controlsd — curvature correction hook
curvature = hooks.run('controls.curvature_correction', curvature, model_v2, v_ego)
```

**Fail-safe guarantee**: If any plugin raises an exception, `hooks.run()` returns the unmodified default value. Zero impact on stock behavior.

### Plugin Types

| Type | Description | Example |
|------|-------------|---------|
| `hook` | Pure callback registration | lane_centering |
| `process` | Daemon process | mapd |
| `hybrid` | Hook + process | speedlimitd |
| `car` | Car interface (exclusive) | bmw_e9x_e8x |
| `tool` | Utility/panel | model_selector |

### Boot-time Schema Patching

Plugins that need custom cereal messages claim reserved slots in `custom.capnp`. The `builder.py` module patches schemas at boot — stock cereal files stay unmodified in git.

## Included Plugins

| Plugin | Type | Description |
|--------|------|-------------|
| `bmw_e9x_e8x` | car | BMW E82/E90 car interface with VIN-based detection |
| `c3_compat` | hybrid | Comma 3 compatibility layer + Raylib Python UI |
| `lane_centering` | hook | Single-lane centering curvature correction |
| `mapd` | process | OSM map daemon for speed limits and curves |
| `model_selector` | tool | Download and swap driving models |
| `speedlimitd` | hybrid | Speed limit enforcement (OSM + YOLO + inference) |

## Writing a Plugin

### Minimal plugin (hook type)

```
my_plugin/
├── plugin.json
└── my_hook.py
```

**plugin.json:**
```json
{
  "id": "my-plugin",
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
def on_v_cruise(v_cruise, sm, v_ego):
  # Modify cruise speed — return original on error (fail-safe)
  return min(v_cruise, 120.0)
```

Drop the directory into `/data/plugins/` and reboot.

### Available Hooks

| Hook | File | Arguments |
|------|------|-----------|
| `car.register_interfaces` | card | interfaces_dict |
| `controls.curvature_correction` | controlsd | curvature, model_v2, v_ego, lane_changing |
| `planner.v_cruise` | plannerd | v_cruise, sm, v_ego |
| `planner.accel_limits` | plannerd | accel_limits, v_ego, lead |
| `desire.post_update` | plannerd | desire, model_v2, lane_change_state |
| `device.health_check` | manager | health_status |

### Custom Cereal Messages

Claim a slot (0-19) in your manifest to get a custom cereal struct:

```json
{
  "cereal": {
    "slots": {
      "0": {
        "struct_name": "MyState",
        "event_field": "myState",
        "schema_file": "cereal/slot0.capnp"
      }
    }
  },
  "services": {
    "myState": [true, 1.0, 1]
  }
}
```

## Architecture Docs

- [VISION.md](docs/VISION.md) — Plugin architecture vision
- [INTEGRATION_ARCHITECTURE.md](docs/INTEGRATION_ARCHITECTURE.md) — Technical deep-dive
- [HOOK_INTEGRATION_POINTS.md](docs/HOOK_INTEGRATION_POINTS.md) — All hook call sites

## License

MIT
