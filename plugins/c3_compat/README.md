# Comma 3 Compatibility

Keeps the [comma three](https://github.com/commaai/hardware/tree/master/comma_three) (2021) running on openpilot v0.10.3+. Upstream dropped Comma 3 support in v0.10.3 — this plugin restores it without maintaining a separate branch or codebase.

## For Comma 3 Owners

Install a pre-patched [catpilot](https://github.com/catpilot-dev/catpilot) v0.10.3+ release for minimal risk — c3_compat is included and applied automatically on every boot. No manual setup required.

**Disclaimer**: c3_compat has been fully tested on a comma three device (serial dc8405e6). Please [report any issues](https://github.com/catpilot-dev/plugins/issues). Use at your own risk — we are not liable for any consequences.

## Why

The Comma 3 (code name "tici") uses a Snapdragon 845 SoC and STM32F4 panda MCU. When comma released v0.10.3, they removed C3-specific code paths: the tici amplifier config, STM32F4 panda support, SPI protocol compatibility, and the Wayland/Weston display stack. Without this plugin, a Comma 3 running v0.10.3 can't detect its panda, produce audio, or render a UI.

The other major gap is AGNOS — the comma device OS. Comma 3 is capped at AGNOS 12.8, while openpilot v0.10.3 targets AGNOS 16. c3_compat bridges the differences: missing system packages, read-only root filesystem constraints, Wayland→DRM display backend, and Python venv dependencies that changed between the two AGNOS versions.

## What It Does

**Boot patches** (`boot_patch.sh`) — applied before openpilot launches:

- Restores STM32F4 (Dos board) panda support: MCU type, USB detection, firmware version skip, SPI→USB fallback
- Restores tici amplifier EQ config (audio)
- Replaces Weston/Wayland display stack with DRM backend (eliminates 28s boot delay)
- Installs `venv_sync` — ensures venv matches `uv.lock` on every boot regardless of how code was deployed
- Patches `launch_chffrplus.sh` with correct PATH/PYTHONPATH for scons builds
- Symlinks cache directories to `/data/` (prevents 100MB `/home` overlay from filling up)
- Persistent crash diagnostics: rotated dmesg + system state snapshots in `/data/crash_diag/`
- Background watchdog for system health monitoring

**Panda health hook** (`compat.py`) — runtime STM32F4/Dos health check

## Plugin Details

**Type**: hybrid (process + hook) | **Device filter**: tici (Comma 3 only)

```
c3_compat/
  plugin.json          # Plugin manifest
  boot_patch.sh        # AGNOS 12.8 boot-time patches (14 sections)
  compat.py            # device.health_check hook
  venv_sync.py         # uv.lock → venv dependency synchronizer
  watchdog.sh          # Background system health monitor
  raylib_drm/          # DRM-backend raylib .so (Git LFS)
```
