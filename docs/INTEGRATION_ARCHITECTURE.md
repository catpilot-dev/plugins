# Plugin Integration Architecture

## Overview

**Status**: IMPLEMENTED AND VALIDATED
**Base**: catpilot (openpilot fork with plugin hook points)
**Devices**: Comma 3 (TICI/AGNOS), RK3588 (Orange Pi 5 Plus)

---

## Repository Layout

### catpilot (`~/catpilot-dev/catpilot/` → `catpilot-dev/catpilot`)

Fork of openpilot with the plugin framework built-in. Contains:

- `selfdrive/plugins/hooks.py` — HookRegistry: register, run, fail-safe
- `selfdrive/plugins/loader.py` — Plugin discovery + lazy-load per-process
- `selfdrive/plugins/registry.py` — PluginRegistry: manifest scanning, enable/disable
- `selfdrive/plugins/plugind.py` — Boot-time plugin manager process
- `selfdrive/plugins/builder.py` — Boot-time capnp/services/events injection
- `selfdrive/plugins/bus.py` — ZMQ IPC pub/sub (PluginPub/PluginSub)
- Hook call sites in: controlsd, plannerd, card, longitudinal_planner, desire_helper, car_helpers, selfdrived, torqued, webrtcd, ui layouts, ui_state, application

No `plugins/` directory — all plugin code lives in the plugins repo.

### plugins (`~/catpilot-dev/plugins/` → `catpilot-dev/plugins`, branches `dev`/`main`)

Hooks-only plugins. No framework code, no overlays. Works on both C3 and RK3588 via `device_filter`.

```
plugins/
  install.sh              # Deploy plugins to /data/plugins-runtime/; install cereal slots
  plugins/
    config.py             # Shared path config; deployed to /data/plugins-runtime/config.py
    bmw_e9x_e8x/          # BMW E8x/E9x car interface (hook-based registration)
    c3_compat/            # Comma 3 AGNOS compatibility (boot patches, venv_sync)
    lane_centering/       # Curvature correction hook
    mapd/                 # OSM map data process
    model_selector/       # Runtime driving model swapping
    network_settings/     # Proxy + static IP + github connectivity
    phone_display/        # Phone-as-display: WebRTC video + HUD, watchdog
    speedlimitd/          # Speed limit fusion (process + hook)
    trafficd/             # YOLO traffic sign detection (NPU)
    ui_mod/               # Custom UI panels: Driving, Vehicle, Plugins, drive stats
  docs/                   # This documentation
```

### connect-on-device (`~/catpilot-dev/connect-on-device/` → `catpilot-dev/connect-on-device`)

Web app for device management. Served on port 8082 (redirected from port 80 via iptables in `setup_service.sh`). Works on both C3 and RK3588.

---

## Runtime Layout on Device

```
/data/openpilot/            # catpilot (git clone, branch rk3588 or dev)
  selfdrive/plugins/        # Hook system + plugind (built-in to catpilot)

/data/plugins/              # Plugins source repo (git clone, branch dev)
  plugins/
    config.py               # Source; copied to /data/plugins-runtime/config.py

/data/plugins-runtime/      # Deployed plugin runtime (written by install.sh)
  config.py                 # Shared path config — importable by all plugins
  bmw_e9x_e8x/
  c3_compat/
  lane_centering/
  mapd/
  model_selector/
  network_settings/
  phone_display/
  speedlimitd/
  trafficd/
  ui_mod/
    data/                   # Plugin-specific param storage (plugin_data_dir())

/data/connect-on-device/    # COD repo (git clone)
```

---

## Shared Config (`plugins/config.py`)

All plugins import path constants from `config.py` via sys.path trick:

```python
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import PLUGINS_RUNTIME_DIR, PLUGINS_REPO_DIR, plugin_data_dir, read_plugin_param
```

This import works in both:
- Source repo: `__file__` = `plugins/plugins/<id>/something.py` → parent of parent = `plugins/plugins/`
- On device: `__file__` = `/data/plugins-runtime/<id>/something.py` → parent of parent = `/data/plugins-runtime/`

**Path constants** (all overridable via env var for testing):

| Constant | Default |
|----------|---------|
| `OPENPILOT_DIR` | `/data/openpilot` |
| `PLUGINS_RUNTIME_DIR` | `/data/plugins-runtime` |
| `PLUGINS_REPO_DIR` | `/data/plugins` |
| `PARAMS_DIR` | `/data/params/d` |

**Helpers**: `plugin_data_dir(plugin_id)` → `Path(PLUGINS_RUNTIME_DIR) / plugin_id / "data"`

---

## Plugin Params

Plugin params are **NOT** in `params_keys.h`. Using `Params().get()` for unknown keys throws `UnknownKeyName`. All plugin params use raw file I/O via `config.py` helpers:

```python
from config import read_plugin_param, write_plugin_param

# Read: returns '' if not set
value = read_plugin_param("phone_display", "CatEyePhoneRequired")

# Write
write_plugin_param("phone_display", "CatEyePhoneRequired", "1")
```

Files are stored at `/data/plugins-runtime/<id>/data/<key>`.

---

## Plugin Manifest Format

```json
{
  "id": "speedlimitd",
  "name": "Speed Limit Middleware",
  "version": "1.0.0",
  "type": "hybrid",
  "device_filter": ["tici", "tizi", "mici", "rk3588"],
  "hooks": {
    "planner.v_cruise": {
      "module": "planner_hook",
      "function": "on_v_cruise",
      "priority": 50
    }
  },
  "processes": [
    {"name": "speedlimitd", "module": "speedlimitd", "condition": "only_onroad"}
  ],
  "cereal": {
    "slots": [0],
    "services": {
      "speedLimitState": [true, 1.0, 1]
    },
    "event_names": ["speedLimitActive", "speedLimitConfirmRequired"]
  },
  "params": {
    "SpeedLimitConfirmed": {"type": "string", "default": "0", "desc": null}
  }
}
```

Key fields:
- **device_filter**: `["tici"]` for C3-only, `["rk3588"]` for RK3588-only, omit for all devices
- **cereal.slots**: Custom capnp struct slots 0-19 (must be unique per plugin)
- **cereal.event_names**: Injected into `log.capnp` EventName enum at boot by `builder.py`
- **params**: `desc` non-null = shown in Settings UI; `desc` null = internal only

---

## Boot Flow

```
continue.sh
  └─ /data/connect-on-device/setup_service.sh
       └─ iptables: 80 → 8082 (PREROUTING + OUTPUT, idempotent)
       └─ python server.py &   (COD web app)

openpilot manager
  └─ plugind (first process)
       └─ builder.py
            └─ patch custom.capnp with plugin cereal slots
            └─ inject EventName entries into log.capnp
            └─ register services in services.py
            └─ write param defaults to /data/params/d/
       └─ start plugin processes (mapd, speedlimitd, phone_watchdog, ...)
  └─ selfdrived, controlsd, plannerd, card, webrtcd, ...
       └─ hooks._ensure_loaded() on first hooks.run() call
            └─ PluginRegistry.discover() + load_enabled()
            └─ plugin hook callbacks registered in this process
```

---

## Plugin: phone_display

Enables phone-as-display for headless devices (RK3588, future C3 headless).

**Hooks registered**:
- `selfdrived.events` — publishes `EventName.catEyePhoneRequired` when phone not connected and required
- `webrtc.app_routes` — registers WebRTC signaling + HUD data endpoints in webrtcd
- `webrtc.session_started` / `webrtc.session_ended` — watchdog lifecycle management

**Param**: `CatEyePhoneRequired` (in plugin data dir, not params_keys.h)
- `0` = phone optional (no blocking alert)
- `1` = phone required (blocks engagement when disconnected)

**COD routing**: `_PLUGIN_TOGGLE_PARAMS` in `handlers/params.py` routes `CatEyePhoneRequired` reads/writes through `read_plugin_param`/`write_plugin_param` instead of openpilot Params.

---

## Plugin: c3_compat

Comma 3 hardware compatibility for AGNOS 12.8. Device filter: `tici`.

**boot_patch.sh** (runs before openpilot launch):
1. Cache symlinks: `~/.cache/pip` + `~/.cache/tinygrad` → `/data/cache/` (100MB /home overlay)
2. **venv_sync.py**: sync Python packages against local `uv.lock` (hash-cached, <100ms when unchanged)
3. kaitaistruct install (AGNOS system dep)
4. DRM raylib overlay (GPU-accelerated rendering)
5. Wayland/Weston socket permissions
6. SPI disable (USB-only F4 panda)
7. PATH + PYTHONPATH for scons build
8. Crash diagnostics

**venv_sync.py**:
- Parses `uv.lock` TOML, walks dependency graph, filters PEP 508 markers
- `--runtime-only` mode: 62 packages (boot default)
- Hash cache at `plugin_data_dir("c3_compat") / ".venv_synced_hash"`

---

## COD Integration

- **Port 80**: iptables PREROUTING + OUTPUT redirect from 80 → 8082, applied by `setup_service.sh` on every boot. Uses `iptables-legacy` on C3/AGNOS, falls back to `iptables` on other platforms.
- **Plugin API**: plugind serves REST on `:8083` — plugin list, enable/disable, install, status
- **Software panel**: CHECK → DOWNLOAD → INSTALL flow matching stock openpilot

---

## On-Device Validation

```bash
# Verify COD running
ssh comma@c3 "curl -s http://localhost:8082/api/health"

# Verify iptables redirect
ssh comma@c3 "sudo iptables-legacy -t nat -L PREROUTING | grep 8082"

# Verify plugins deployed
ssh comma@c3 "ls /data/plugins-runtime/"

# Verify config.py deployed
ssh comma@c3 "python3 -c 'import sys; sys.path.insert(0, \"/data/plugins-runtime\"); from config import PLUGINS_RUNTIME_DIR; print(PLUGINS_RUNTIME_DIR)'"

# Verify hook system
ssh comma@c3 "cd /data/openpilot && python3 -c \"
from openpilot.selfdrive.plugins.hooks import hooks
hooks._loaded = True  # skip plugin load
hooks.register('test', 'p1', lambda x: x * 2, 50)
assert hooks.run('test', 5) == 10
print('Hook system OK')
\""

# Check plugin logs
ssh comma@c3 "cat /tmp/plugin_logs/phone_display.log"
```
