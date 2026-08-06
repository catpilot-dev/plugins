# Plugin Development

Technical reference for working on this repo. For what the plugins do,
see the top-level [README](../README.md) and each plugin's own README.

## Directory Layout

```
plugins/
├── install.sh                     # Plugin installer (→ /data/plugins-runtime/)
├── logos/                         # Brand emblems and icons for all supported cars
│   ├── emblems/                   #   Color SVG+PNG (512px)
│   └── icons/                     #   White-on-transparent PNG (168px)
├── plugins/                       # Plugin packages
└── docs/                          # Architecture and technical docs
```

## Installation mechanics

Plugins are installed automatically with catpilot. To update manually:

```bash
ssh comma
cd /data/plugins
git pull origin dev && bash install.sh
```

install.sh copies plugins to `/data/plugins-runtime/`, injects cereal
schemas and services, clears bytecode caches, and writes a restart marker.
The plugin daemon (plugind) detects the marker when offroad and restarts
managed processes and the UI.

### Enable/disable from SSH

```bash
# Disable a plugin
touch /data/plugins-runtime/speedlimitd/.disabled

# Re-enable
rm /data/plugins-runtime/speedlimitd/.disabled
```

Plugins with an `.enforced` marker are required by the platform (their
removal is coupled to other components) — install.sh clears stale
`.disabled` markers for those.

## Writing a Plugin

A plugin needs a directory in `plugins/` with a `plugin.json` manifest and
one or more Python modules.

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

- **ALL UI imports MUST be lazy** — import inside the hook function body,
  not at module level. Hooks load during `__init__` mid-import.
- **Plugin params** go in `/data/plugins/<id>/data/`, never
  `/data/params/d/` (openpilot wipes unknown keys on boot).
- **Fail-safe by default** — if your hook raises an exception, the default
  value is returned and other plugins continue.

### Available hooks

See the [catpilot README](https://github.com/catpilot-dev/catpilot#hook-call-sites)
for the full list of hook call sites.

## Testing

```bash
PYTHONPATH=. uv run pytest
```

A pre-push hook runs all tests automatically. Tests that require
openpilot/opendbc auto-skip when those dependencies are unavailable.
