# catpilot plugins

Plugin packages for [catpilot](https://github.com/catpilot-dev/catpilot). Integrated into catpilot releases starting from `v0.10.3` — plugins are automatically installed on first boot.

## Directory Layout

```
plugins/
├── install.sh                     # Overlay installer
├── plugins/                       # Plugin packages (→ /data/plugins/)
│   ├── bmw_e9x_e8x/              # BMW E8x/E9x car interface
│   ├── c3_compat/                 # Comma 3 compatibility (AGNOS 12.8)
│   ├── lane_centering/            # Lane centering correction
│   ├── mapd/                      # OSM map daemon
│   ├── model_selector/            # Driving model selector
│   └── speedlimitd/               # Speed limit enforcement
├── overlays/                      # Files overlaid into openpilot tree
│   ├── selfdrive/plugins/         #   Plugin framework (hooks, registry, builder)
│   ├── selfdrive/ui/onroad/       #   UI overlay modules (HUD, model, alerts)
│   └── cereal/custom.capnp        #   Reserved struct slots for plugins
└── docs/                          # Architecture and technical docs
```

## Plugins

| Plugin | Type | Description |
|--------|------|-------------|
| `bmw_e9x_e8x` | car | BMW E82/E90 car interface with VIN-based detection |
| `c3_compat` | hybrid | Comma 3 compatibility layer (AGNOS 12.8, STM32F4 panda, DRM display) |
| `lane_centering` | hook | Single-lane centering curvature correction |
| `mapd` | process | OSM map daemon for speed limits and curves |
| `model_selector` | tool | Download and swap driving models |
| `speedlimitd` | hybrid | Speed limit enforcement (OSM + YOLO + inference) |

## Managing Plugins

### For users: Connect on Device

Use [Connect on Device](https://github.com/catpilot-dev/connect) to enable or disable plugins from the web UI — no SSH required.

### For developers: Manual installation

```bash
ssh comma
cd /data
git clone https://github.com/catpilot-dev/plugins.git openpilot-plugins
cd openpilot-plugins && bash install.sh
sudo reboot
```

To update an existing installation:

```bash
ssh comma
cd /data/openpilot-plugins
git pull && bash install.sh
sudo reboot
```

To enable/disable a plugin manually:

```bash
# Disable
touch /data/plugins/speedlimitd/.disabled

# Re-enable
rm /data/plugins/speedlimitd/.disabled
```

## Writing a Plugin

A minimal plugin needs a directory in `plugins/` with a `plugin.json` and a Python module:

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
  return min(v_cruise, 120.0)
```

Drop the directory into `/data/plugins/` and reboot.

### Available Hooks

| Hook | Arguments |
|------|-----------|
| `car.register_interfaces` | interfaces_dict |
| `controls.curvature_correction` | curvature, model_v2, v_ego, lane_changing |
| `planner.v_cruise` | v_cruise, sm, v_ego |
| `planner.accel_limits` | accel_limits, v_ego, lead |
| `desire.post_update` | desire, model_v2, lane_change_state |
| `device.health_check` | health_status |

## License

MIT
