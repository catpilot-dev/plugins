# Plugin Architecture — Tasks & Progress

## Overview

Plugin system for openpilot-plugins standalone repo (`~/openpilot-plugins-standalone/` → `OxygenLiu/openpilot-plugins` on GitHub).
Plugins live in `plugins/` and hook into the control loop via the hook system, run as managed processes, or register car interfaces.

### Repository Structure

- **Standalone repo**: `~/openpilot-plugins-standalone/` — all plugin code + framework overlay files
- **Fork repo**: `~/openpilot-plugins/` (branch `plugins-release`) — minimal upstream v0.10.3 fork with only plugin framework hooks (~20 files diff)
- **Runtime**: `install.sh` overlays framework + cereal into openpilot tree, copies plugins to `/data/plugins/`

### Available Hook Points (wired in repo)
- `controls.curvature_correction` (controlsd.py) — curvature adjustment
- `planner.v_cruise` (longitudinal_planner.py) — cruise speed override
- `planner.accel_limits` (longitudinal_planner.py) — accel limit adjustment
- `desire.post_update` (desire_helper.py) — lane change extensions
- `car.register_interfaces` (car_helpers.py) — register car platforms
- `car.panda_status` (car_helpers.py) — monitor panda health

---

## Repo-Level Prerequisites ✅ COMPLETED

- ✅ `cereal/custom.capnp` — SpeedLimitState, MapdOut, MapdIn, MapdExtendedOut structs
- ✅ `cereal/log.capnp` — Event field renames (speedLimitState, mapdOut, mapdIn, mapdExtendedOut)
- ✅ `cereal/services.py` — mapdOut (20Hz), mapdExtendedOut (20Hz), mapdIn (no-log), speedLimitState (1Hz)
- ✅ `common/params_keys.h` — LaneCenteringCorrection, MapdVersion, SpeedLimitConfirmed, SpeedLimitValue
- ✅ `selfdrive/controls/plannerd.py` — speedLimitState + mapdOut added to SubMaster

---

## Plugin: bmw_e9x_e8x ✅ COMPLETED

**Type**: car (hook-based registration)
**Hooks**: `car.register_interfaces`, `car.panda_status`

Full BMW E8x/E9x car interface:
- VIN-based detection (empty CAN fingerprints, pure VIN model code matching)
- Car interface (CarState, CarController, CarParams)
- DCC control (speed-dependent Dynamic Cruise Control via tick-counted commands)
- Stepper servo steering (Ocelot stepper servo on F-CAN/AUX-CAN)
- Panda safety model (bmw.h, safety model ID 35)

Based on [dzid26's BMW E8x/E9x openpilot implementation](https://github.com/BMW-E8x-E9x/openpilot).

---

## Plugin: c3_compat ✅ COMPLETED

**Type**: hybrid (process + hook)
**Device filter**: tici (Comma 3)

Comma 3 hardware compatibility for AGNOS 12.8:
- **Raylib Python UI** — Full replacement UI (onroad HUD, settings, driver camera)
- **boot_patch.sh** — AGNOS 12.8 boot patches:
  - Cache symlinks (pip, tinygrad → /data/cache/) to avoid filling 100MB /home overlay
  - PATH + PYTHONPATH for scons build (cythonize, opendbc imports)
  - kaitaistruct install (AGNOS system dependency)
  - DRM raylib overlay for GPU-accelerated rendering
  - Wayland/Weston socket permissions
  - SPI disable for USB-only F4 panda
  - Crash diagnostics
- **venv_sync.py** — Python package sync against local uv.lock:
  - Dependency graph walk with PEP 508 marker filtering
  - Hash cache for <100ms skip when unchanged
  - AGNOS 12.8 pip workarounds (sudo -E, TMPDIR, --no-cache-dir)
  - Runtime-only mode (62 packages) for boot
- **watchdog.sh** — Process watchdog with crash recovery
- **Panda health monitoring** — STM32F4/Dos health check hook

---

## Plugin: lane_centering ✅ COMPLETED

**Type**: hook
**Hook**: `controls.curvature_correction`
**Param toggle**: `LaneCenteringCorrection` (bool, default off)

Features:
- Curvature-dependent K gain (sharper turns → stronger correction)
- Hysteresis activation (MIN_CURVATURE=0.002 on, EXIT_CURVATURE=0.001 off)
- Dynamic lane width estimation from both lane lines
- Jump rejection (MAX_JUMP=0.3m per frame)
- Smooth wind-down on deactivation (WINDDOWN_TAU=1.0s)
- Disabled during lane changes and at low speed (<9 m/s)

---

## Plugin: mapd ✅ COMPLETED

**Type**: process
**Condition**: always_run
**Device filter**: tici, tizi, mici

Binary lifecycle manager for pfeiferj/mapd (Go binary). Publishes mapdOut at 20Hz with OSM speed limits, road names, curve speeds, hazards.

---

## Plugin: speedlimitd ✅ COMPLETED

**Type**: hybrid (process + hook)
**Dependency**: mapd plugin
**Process**: speedlimitd at 1Hz (only_onroad)
**Hook**: `planner.v_cruise` — enforces confirmed speed limits

Three-tier priority:
1. OSM maxspeed (confidence 0.95)
2. YOLO speed sign detection (confidence 0.8)
3. Road type + lane count inference (confidence 0.5)

Speed-dependent offset (km/h above limit):
- 20-60 km/h zones: +20 km/h
- 70-120 km/h zones: +10 km/h

---

## Plugin: model_selector ✅ COMPLETED

**Type**: process
**Condition**: always_run

Runtime model swapping:
- Download alternative driving models from GitHub releases
- Hot-swap models without rebuild (modeld restart)
- Model compatibility filtering by device type

---

## Benefits Over Monolith Fork

### No more submodule forks
The monolith approach required maintaining forked `opendbc` and `panda` submodules
with BMW-specific code. Every upstream comma update meant rebasing three repos
(openpilot + opendbc fork + panda fork) and resolving merge conflicts in all three.

With plugins, `opendbc_repo` and `panda` use upstream submodules directly. All
BMW-specific code lives in `plugins/bmw_e9x_e8x/` and loads dynamically at runtime.

| Before (monolith) | After (plugins) |
|---|---|
| Fork opendbc → add car/bmw/ | plugins/bmw_e9x_e8x/bmw/ |
| Fork panda → add safety/bmw.h | plugins/bmw_e9x_e8x/safety/bmw.h |
| Rebase 3 repos on every upstream update | `git pull upstream` — plugins untouched |

### Upstream sync is trivial
Update workflow: `git pull upstream master`. Plugins are isolated — nothing to
rebase. The only breakage risk is if comma changes a hook call site signature
(~5 lines in the repo), which is easy to spot and fix.

### Clean separation of concerns
- Official openpilot code: repo root (`selfdrive/`, `cereal/`, etc.)
- All custom features: `plugins/` directory
- Easy to enable/disable individual features via Params toggles
- Each plugin is self-contained with its own README, manifest, and code

---

## C3 Deployment Status ✅ VERIFIED

- ✅ scons build working on AGNOS 12.8 (PATH, PYTHONPATH, cache symlinks)
- ✅ Overnight stability verified (10+ hours, no crashes/OOM/hangs)
- ✅ venv_sync working (runtime-only mode, hash caching)
- ✅ Boot patches applied cleanly every reboot
- ✅ connect_on_device SOFTWARE panel matches stock openpilot update flow

---

## TODO

- [ ] YOLO speed sign integration into speedlimitd (currently placeholder)
- [ ] Interactive speed limit HUD element (tap to confirm/dismiss)
- [ ] UI render hook for plugin overlays
