# Plugin Integration Architecture

## Overview

**Status**: IMPLEMENTED AND VALIDATED
**Target Openpilot**: v0.10.3 with msgq memory optimization
**Device**: Comma 3 (TICI) on AGNOS 12.8

---

## Repository Layout

### Standalone Repo (`~/openpilot-plugins-standalone/` → `OxygenLiu/openpilot-plugins`)

```
openpilot-plugins-standalone/
  install.sh              # Overlay framework + cereal into openpilot, copy plugins
  selfdrive/              # Framework overlay files
    plugins/
      hooks.py            # HookRegistry (register, run, fail-safe)
      loader.py           # Plugin discovery + lazy-load
      plugin_base.py      # Plugin base class
      plugind.py           # Boot-time plugin builder process
  cereal/                 # Custom capnp schemas + service registrations
  common/                 # params_keys.h additions
  plugins/                # All plugins
    bmw_e9x_e8x/          # BMW car interface
    c3_compat/            # Comma 3 AGNOS 12.8 compatibility
    lane_centering/       # Curvature correction hook
    mapd/                 # OSM map data process
    model_selector/       # Runtime model swapping
    speedlimitd/          # Speed limit fusion (process + hook)
    docs/                 # This documentation
```

### Fork Repo (`~/openpilot-plugins/` → `OxygenLiu/c3pilot` branch `plugins-release`)

Minimal diff against upstream v0.10.3 (~20 files). Contains only:
- `selfdrive/plugins/` — hook system + plugind
- Hook call sites in controlsd, planner, desire_helper, car_helpers

No `plugins/` directory — all plugin code lives in the standalone repo.

### Runtime Layout on C3

```
/data/openpilot/          # Openpilot with framework overlay applied
  selfdrive/plugins/      # Hook system (from install.sh overlay)
  cereal/custom.capnp     # Custom schemas (from install.sh overlay)
/data/plugins/            # All plugins (copied by install.sh)
  bmw_e9x_e8x/
  c3_compat/
  lane_centering/
  mapd/
  model_selector/
  speedlimitd/
```

---

## Car Interface Registration

BMW uses a hook-based registration system that injects platforms into openpilot's
existing dynamic interface loading:

```
Boot → plugind → car.register_interfaces hook
  → bmw_e9x_e8x/register.py injects BMW platforms into opendbc car registry
  → Fingerprinting finds BMW via VIN-based detection (empty CAN fingerprints)
  → CarInterface loaded from plugin's bmw/ directory
```

Key files:
- `plugins/bmw_e9x_e8x/register.py` — `car.register_interfaces` hook
- `plugins/bmw_e9x_e8x/bmw/values.py` — Platform config, VIN detection
- `plugins/bmw_e9x_e8x/bmw/interface.py` — CarInterface implementation
- `plugins/bmw_e9x_e8x/bmw/carstate.py` — CAN parsing (empty parser pattern)
- `plugins/bmw_e9x_e8x/bmw/carcontroller.py` — DCC commands, servo, turn signals

### VIN-Based Detection

```python
# Empty fingerprints — accept all CAN messages, detect by VIN
FINGERPRINTS = {CAR.BMW_E82: [{}], CAR.BMW_E90: [{}]}

# VIN positions 4-6 contain model code
def match_fw_to_car_fuzzy(live_fw_versions, vin, offline_fw_versions):
    model_code = vin[3:6]
    vin_to_model = {'PH1': 'BMW_E90', 'UF1': 'BMW_E82', ...}
    return {vin_to_model[model_code]} if model_code in vin_to_model else set()
```

### Panda Safety

Panda safety model (bmw.h, ID 35) is compiled into the panda firmware and
activated automatically via `CarParams.safetyConfigs`. The safety model binary
is included in the plugin's `safety/` directory and flashed at boot if needed.

---

## Hook System

### Architecture

```python
class HookRegistry:
    def run(self, hook_name, default, *args, **kwargs):
        callbacks = self._hooks.get(hook_name)
        if not callbacks:
            return default  # ~50ns, zero overhead
        for priority, plugin_name, callback in callbacks:
            try:
                result = callback(result, *args, **kwargs)
            except Exception:
                return default  # Fail-safe
        return result
```

### Implemented Hook Points

| Hook | File | Plugin | Description |
|------|------|--------|-------------|
| `controls.curvature_correction` | controlsd.py | lane_centering | Curvature adjustment for lane centering |
| `planner.v_cruise` | longitudinal_planner.py | speedlimitd | Speed limit enforcement |
| `planner.accel_limits` | longitudinal_planner.py | (available) | Acceleration limit adjustment |
| `desire.post_update` | desire_helper.py | (available) | Lane change extensions |
| `car.register_interfaces` | car_helpers.py | bmw_e9x_e8x | Register car platforms |
| `car.panda_status` | car_helpers.py | bmw_e9x_e8x | Monitor panda health |

---

## Process Management

Plugins can declare managed processes in their `plugin.json`:

```json
{
  "processes": [
    {"name": "mapd", "module": "mapd_runner", "condition": "always_run"},
    {"name": "speedlimitd", "module": "speedlimitd", "condition": "only_onroad"}
  ]
}
```

`plugind` discovers these declarations and registers them with the manager.

---

## AGNOS 12.8 Compatibility (c3_compat)

### boot_patch.sh

Runs before openpilot build/launch. Key sections:

1. **Overlay init removal** — prevents stale overlay swap
1b. **Cache symlinks** — `~/.cache/pip` and `~/.cache/tinygrad` → `/data/cache/` (100MB /home overlay protection)
2. **venv_sync** — sync Python packages against local uv.lock (dependency graph walk, PEP 508 markers)
2b. **kaitaistruct** — AGNOS system dependency (not in uv.lock)
3. **DRM raylib** — GPU-accelerated rendering overlay
4. **Wayland socket permissions** — Weston compositor access
5. **SPI disable** — USB-only F4 panda mode
6. **PATH + PYTHONPATH** — scons build support (cythonize, opendbc imports)
7. **Crash diagnostics** — dmesg + process state capture on crash

### venv_sync.py

Ensures C3 venv has all packages required by the current openpilot branch:

```
local uv.lock → SHA256 hash → cached? → skip (<100ms)
                             → changed? → parse TOML → walk dependency graph
                               → filter PEP 508 markers (platform, arch)
                               → diff against installed packages
                               → sudo -E pip install (read-only rootfs)
```

- `--runtime-only`: 62 packages (boot default, excludes dev/test/docs groups)
- Full mode: 135 packages
- AGNOS quirks: `TMPDIR=/data/tmp/pip`, `--no-cache-dir` for large wheels

### connect_on_device Integration

- `POST /v1/software/prepare-plugins` — copies staged plugins + runs venv_sync
- `POST /v1/software/venv-sync` — manual venv sync trigger
- SOFTWARE panel matches stock openpilot update flow (CHECK → DOWNLOAD → INSTALL)

---

## Plugin Manifest Format

```json
{
  "id": "speedlimitd",
  "name": "Speed Limit Middleware",
  "version": "1.0.0",
  "type": "hybrid",
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
  "params": {
    "SpeedLimitConfirmed": {"type": "string", "default": "0"},
    "SpeedLimitValue": {"type": "string", "default": "0"}
  },
  "services": {
    "speedLimitState": [true, 1.0, 1]
  }
}
```

---

## Testing

### Pre-deployment validation

```bash
# Test BMW imports
source ~/openpilot/.venv/bin/activate
python -c "from opendbc.car.bmw.carstate import CarState; print('OK')"

# Test VIN detection
python -c "
from opendbc.car.bmw.values import match_fw_to_car_fuzzy
print(match_fw_to_car_fuzzy({}, 'LBVPH18059SC20723', {}))  # {'BMW_E90'}
"

# Test safety model
python -m pytest opendbc_repo/opendbc/safety/tests/test_bmw.py -v

# Test hook system
python -c "
from openpilot.selfdrive.plugins.hooks import HookRegistry
h = HookRegistry()
h.register('test', 'p1', lambda x: x * 2, 50)
assert h.run('test', 5) == 10
print('Hook system OK')
"
```

### On-device validation

```bash
# Verify boot patches
ssh c3 "cat /tmp/c3_compat.log"

# Verify venv sync
ssh c3 "cat /data/plugins/c3_compat/.venv_synced_hash"

# Verify scons build
ssh c3 "cd /data/openpilot && scons -j8"
```
