# Plugin Architecture — Vision

> This document describes the long-term goal. The architecture is validated on
> a BMW E90 with Comma 3 running AGNOS 12.8 and v0.10.3 release. 

## The Problem

Every openpilot fork (FrogPilot, sunnypilot, DragonPilot, car-specific forks)
maintains thousands of lines of diff against upstream comma. Each upstream
release triggers a painful rebase across the main repo and forked submodules.
Features developed in one fork are inaccessible to users of another.

## The Idea

A plugin architecture that turns forks into composable, drop-in modules:

- **One upstream openpilot** with a small set of hook points
- **N independent plugins** that extend behavior without modifying core code
- **Users mix and match** — install only what they need

## What Plugins Can Replace

| Today (forked) | Tomorrow (plugin) |
|---|---|
| Fork opendbc + panda for unsupported car | Car interface plugin (BMW, Rivian, Porsche, ...) |
| Fork selfdrive for custom features | Hook plugins (lane centering, speed limits, ...) |
| Fork UI for custom HUD elements | UI overlay plugin |
| Fork manager for custom processes | Process plugin (mapd, dashcam manager, ...) |
| Maintain N diverging forks | One upstream + N plugins |

## Hook Points

Implemented:
- `controls.curvature_correction` — curvature adjustment
- `planner.v_cruise` — cruise speed override
- `planner.accel_limits` — acceleration limit adjustment
- `desire.post_update` — lane change extensions
- `car.register_interfaces` — register car platforms
- `car.panda_status` — monitor panda health

Planned:
- `ui.render_overlay` — custom HUD elements
- `manager.register_processes` — spawn plugin processes (currently wired via plugind)

## Who Benefits

- **Car porters**: Ship a car plugin instead of maintaining a full fork
- **Feature developers**: Write a self-contained plugin, works on any fork that has the hook points
- **End users**: Install plugins like apps — no git knowledge required
- **Upstream comma**: Could adopt the hook points (minimal, zero-overhead) and let the community extend via plugins instead of forks

## Upstream Surface Area

For true plug-and-play, a small set of changes would need to exist in upstream
openpilot. These fall into two categories.

### 1. Plugin framework (~30 lines, one-time)

These files enable plugin discovery and hook dispatch:

| File | Lines | Purpose |
|------|-------|---------|
| `selfdrive/plugins/hooks.py` | ~20 | HookRegistry: register callbacks, dispatch with fail-safe |
| `selfdrive/plugins/loader.py` | ~10 | Scan `plugins/`, parse plugin.json, lazy-load hook callbacks |

Zero overhead when no plugins installed — `hooks.run()` returns the default
value immediately if no callbacks are registered (~50ns).

### 2. Hook call sites (~2-3 lines each)

Each hook point is a single `hooks.run()` call inserted at the right place.
These are the minimal touch points in upstream code:

| File | Hook | Lines added |
|------|------|-------------|
| `selfdrive/controls/controlsd.py` | `controls.curvature_correction` | 2 |
| `selfdrive/controls/lib/longitudinal_planner.py` | `planner.v_cruise` | 2 |
| `selfdrive/controls/lib/longitudinal_planner.py` | `planner.accel_limits` | 2 |
| `selfdrive/controls/lib/desire_helper.py` | `desire.post_update` | 2 |
| `selfdrive/ui/layouts/main.py` | `ui.render_overlay` | 2 |
| `system/manager/process_config.py` | `manager.register_processes` | 10 |
| `opendbc_repo/opendbc/car/car_helpers.py` | `car.register_interfaces` | 5 |
| `cereal/custom.capnp` | (schema definitions) | ~50 |
| `cereal/services.py` | (service registration) | ~5 |
| `common/params_keys.h` | (param registration) | ~5 |

**Total upstream diff: ~85 lines** to enable the entire plugin ecosystem.

### What stays outside upstream

Everything else lives in `plugins/` and requires zero upstream changes:

- All plugin code (car interfaces, features, UI overlays, processes)
- Plugin manifests (plugin.json)
- Plugin documentation (README.md per plugin)
- Plugin-specific params defaults and configuration

### Boot-Time JIT Builder (plugind)

Instead of maintaining ~85 lines of upstream patches, the **plugin-aware build
step at boot** reduces the upstream diff to a single line — running
`plugind` before other processes.

#### How it works

```
Boot → plugind scans enabled plugins → patch + build → start openpilot
```

1. `plugind` reads `plugins/*/plugin.json`, checks which are enabled (Params toggle)
2. **capnp schemas**: Merges field definitions from enabled plugins into
   `custom.capnp` reserved structs → `scons cereal/` to recompile
3. **services.py**: Appends service registrations from plugin manifests
4. **params**: Writes default values directly to `/data/params/d/`
5. **hook call sites**: Generates wrapper modules from plugin manifest declarations
6. Normal openpilot processes start with everything in place

Enable/disable a plugin → toggle a param → reboot → plugind rebuilds →
next drive runs with the new configuration.

### Adoption path

1. **Today**: Maintain ~85 lines as a thin patch on top of upstream ✅
2. **After C3 validation**: Implement boot-time JIT builder to eliminate patches ✅ (plugind exists)
3. **Proven at scale**: Propose plugind + hook call sites (~25 lines) to comma
4. **If accepted**: Zero upstream diff — `git clone commaai/openpilot && cp -r plugins/ .` and drive

The key selling point to comma: these hook points cost nothing when unused,
but eliminate the need for hundreds of forks that each carry thousands of
lines of unmaintainable diff.

---

## Status

Proven with 6 plugins on a BMW E90 with Comma 3 (AGNOS 12.8):
- bmw_e9x_e8x (car interface + safety + DCC control)
- c3_compat (AGNOS 12.8 compat: Raylib UI, venv_sync, boot patches, watchdog)
- lane_centering (curvature correction)
- mapd (OSM data)
- speedlimitd (speed limit fusion)
- model_selector (runtime model swapping)

Validated:
- ✅ Overnight stability (10+ hours continuous operation)
- ✅ scons build on AGNOS 12.8
- ✅ venv_sync for automatic Python package management
- ✅ connect_on_device integration for OTA updates
- ✅ Fork diff against upstream v0.10.3 is minimal (~20 files, plugin framework only)
