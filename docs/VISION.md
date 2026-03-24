# Plugin Architecture — Vision

> This document describes the long-term goal. The architecture is validated on
> a BMW E90 with Comma 3 (AGNOS 12.8) and Orange Pi 5 Plus (RK3588).

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

28 hook points implemented across controls, car, planning, selfdrived, UI, and WebRTC.
See `docs/HOOK_INTEGRATION_POINTS.md` for the full list.

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

| File | Hook(s) | Lines added |
|------|---------|-------------|
| `selfdrive/controls/controlsd.py` | `controls.curvature_correction`, `controls.post_actuators` | 4 |
| `selfdrive/controls/lib/longitudinal_planner.py` | `planner.v_cruise`, `planner.accel_limits` | 4 |
| `selfdrive/controls/lib/desire_helper.py` | `desire.pre/post_lane_change`, `desire.post_update` | 6 |
| `selfdrive/controls/plannerd.py` | `planner.subscriptions` | 2 |
| `selfdrive/car/card.py` | `car.cruise_initialized` | 2 |
| `selfdrive/selfdrived/selfdrived.py` | `selfdrived.alert_registry`, `selfdrived.events` | 4 |
| `selfdrive/locationd/torqued.py` | `torqued.allowed_cars` | 2 |
| `selfdrive/ui/onroad/augmented_road_view.py` | `ui.render_overlay` | 2 |
| `selfdrive/ui/onroad/hud_renderer.py` | `ui.onroad_exp_button`, `ui.hud_set_speed_override`, `ui.hud_speed_color` | 6 |
| `selfdrive/ui/ui_state.py` | `ui.state_subscriptions`, `ui.state_tick` | 4 |
| `selfdrive/ui/layouts/sidebar.py` | `ui.connectivity_check` | 2 |
| `selfdrive/ui/layouts/settings/settings.py` | `ui.network_settings_extend`, `ui.settings_extend` | 4 |
| `selfdrive/ui/layouts/settings/software.py` | `ui.software_settings_extend` | 2 |
| `selfdrive/ui/layouts/main.py` | `ui.main_extend` | 2 |
| `selfdrive/ui/layouts/home.py` | `ui.home_extend` | 2 |
| `system/ui/lib/application.py` | `ui.pre_end_drawing`, `ui.post_end_drawing` | 4 |
| `system/webrtc/webrtcd.py` | `webrtc.app_routes`, `webrtc.session_started/ended` | 6 |
| `opendbc_repo/opendbc/car/car_helpers.py` | `car.register_interfaces`, `car.panda_status` | 6 |
| `cereal/custom.capnp` | (schema definitions, injected at boot by builder.py) | ~50 |
| `cereal/services.py` | (service registration, injected at boot) | ~5 |

**Total upstream diff: ~120 lines** across 20 files for the full 28-hook ecosystem. cereal changes are injected at boot by `builder.py` — no static upstream patch needed.

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

Proven with 10 plugins on Comma 3 (AGNOS 12.8) and Orange Pi 5 Plus (RK3588):
- bmw_e9x_e8x (car interface + safety + DCC control)
- c3_compat (AGNOS 12.8 compat: Raylib UI, venv_sync, boot patches, watchdog)
- lane_centering (curvature correction)
- mapd (OSM data)
- speedlimitd (speed limit fusion)
- model_selector (runtime model swapping)
- network_settings (proxy, static IP, github connectivity)
- phone_display (phone-as-display: WebRTC camera + HUD, engagement watchdog)
- trafficd (YOLO traffic sign detection via RKNN NPU)
- ui_mod (custom UI panels: Driving, Vehicle, Plugins, drive stats, route map)

Validated:
- ✅ Overnight stability (10+ hours continuous operation)
- ✅ scons build on AGNOS 12.8 and RK3588 (Ubuntu 22.04)
- ✅ venv_sync for automatic Python package management
- ✅ connect_on_device integration (port 80 via iptables, OTA updates, plugin management)
- ✅ catpilot diff is minimal (~20 files: plugin framework + hook call sites only)
