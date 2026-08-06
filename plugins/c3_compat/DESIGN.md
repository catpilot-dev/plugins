# Comma 3 Compatibility — Design & Implementation

Infrastructure plugin that lets the comma three (SoC: Snapdragon 845; internal
panda: STM32F4 "Dos" board; OS: AGNOS 12.8) run the current catpilot, which is
built against newer hardware and a newer AGNOS. Upstream openpilot removed
comma-three support in v0.10.3; this plugin restores it as a set of **idempotent
boot-time patches** applied to the deployed openpilot tree plus one runtime
health hook — no maintained fork, no source overlays of the affected files.

`plugin.json`: `id = c3-compat`, `type = hook`, `device_filter = ["tici"]`,
`min_openpilot 0.10.0` / `max_openpilot 0.11.99`, `params = {}` (no user params).
The single registered hook is:

| hook | module.fn | priority |
|---|---|---|
| `device.health_check` | `compat.on_health_check` | 50 |

Everything else runs from `boot_patch.sh`, which is **not** invoked by plugind —
it is sourced by catpilot's `launch_chffrplus.sh` early in boot, after the
overlay swap and before the plugin install / scons build:

```
launch_chffrplus.sh
  └─ source /data/plugins-runtime/c3_compat/boot_patch.sh "$DIR"   # unless .disabled present
       ├─ patch the openpilot tree (amplifier, panda, pandad, spi, dfu, multilang, hardware)
       ├─ venv_sync.py --runtime-only --native-deps   (sync AGNOS 12.8 venv to uv.lock)
       ├─ install msgfmt stub + DRM raylib into the venv
       ├─ stop/mask Weston, install weston-ready stub, strip Wayland env
       └─ disable SPI, symlink caches, write commaai/dependencies stubs
  └─ install.sh, builder.py, manager …
```

The `.disabled` marker in the plugin's runtime dir is the only opt-out, and it
gates the boot patch at the launcher (`[ ! -f …/.disabled ]`). On a comma three
`install.sh` writes `.enforced` (`[[ -f /TICI ]]`), so the plugin is always-on
and greyed out in the Plugins panel — see **Enforcement** below.

## Enforcement

On TICI hardware `install.sh` touches `plugins-runtime/c3_compat/.enforced`. The
device does not run catpilot correctly without these patches (no panda, no audio,
no UI, or an OOM reset in ~60 s on-road), so the plugin is treated as
must-stay-on. There is no user toggle. The `.disabled` escape hatch exists for
recovery/bring-up only and is honored by the launcher even when enforced. On
non-tici hardware the `device_filter` keeps the whole plugin dormant.

## Compatibility shims

Grouped by subsystem. Section numbers refer to `boot_patch.sh`.

### Python environment / AGNOS 12.8 (`venv_sync.py`, boot §1c/2/2a/2b/5b/6)

catpilot's `uv.lock` is resolved for the AGNOS 16 venv; the comma three is capped
at AGNOS 12.8 with a different, root-owned, read-only Python 3.12 venv at
`/usr/local/venv`. `venv_sync.py` makes that venv match whatever code was
deployed, by any path (COD update, manual `git checkout`, AGNOS reflash):

- Parses `/data/openpilot/uv.lock` (stdlib `tomllib`, regex fallback), walks the
  dependency graph from the root package, and evaluates PEP 508 environment
  markers for the C3 target (`sys_platform = linux`, `platform_machine =
  aarch64`, `cp312`) so macOS/Windows/PyPy-only packages are skipped. Unknown
  markers default to "install" (safe). `--runtime-only` (the boot mode) drops the
  dev/test/docs optional groups.
- Selects only wheels compatible with `cp312` + `aarch64` + linux (or
  `py3-none-any`, or `abi3` ≤ cp312), batch-checks installed versions in one
  subprocess, then installs/upgrades the delta with
  `pip install --no-deps --no-cache-dir` **into** the venv.
- Wraps the install in `mount -o remount,rw /` … `remount,ro /` (root FS is
  sealed read-only — see Device notes), and sets `TMPDIR` to a root-FS path
  because AGNOS `/tmp` is a 150 MB tmpfs and pip can't `os.rename()` across
  filesystems.
- **Fast path**: the SHA-256 of the last successfully-synced `uv.lock` is cached
  at `plugins-runtime/c3_compat/.venv_synced_hash`; an unchanged lock skips in
  <100 ms.
- `--native-deps` (`ensure_native_deps`): installs the `commaai/dependencies`
  git packages (`bzip2 capnproto eigen ffmpeg libjpeg libyuv ncurses zeromq
  zstd`) that bundle prebuilt native libs for scons and are **not** in `uv.lock`;
  cached in `.native_deps_installed`. ffmpeg's ~100 MB static libs force
  `TMPDIR=/data/tmp`.

Supporting boot patches:

- **§1c cache symlinks** — `~/.cache/pip` and `~/.cache/tinygrad` → `/data/cache/`.
  `/home` is a 100 MB overlay that the tinygrad model-compile cache and pip cache
  would fill.
- **§2a msgfmt stub** — AGNOS 12.8 has no gettext; scons needs `msgfmt` to build
  `.po` → `.mo`. The stub writes a valid empty `.mo` (correct binary header).
- **§5b commaai/dependencies stubs** — v0.11.0's `SConstruct` `import`s modules
  (`bzip2`, `capnproto`, `eigen`, …) that only ship on AGNOS 16+. Stubs written to
  `/data/pip_packages/<name>.py` expose `INCLUDE_DIR` / `LIB_DIR` pointing at
  system libraries so scons can parse. `/data/pip_packages` is on PYTHONPATH.
- **§6 PATH/PYTHONPATH** — patches `launch_chffrplus.sh` so scons sees the venv:
  `/usr/local/venv/bin` on PATH (cythonize), venv site-packages + `opendbc_repo`
  on PYTHONPATH (system `/usr/bin/scons` uses `/usr/bin/python3` and the DBC
  generator imports `opendbc`).

### pycapnp / messaging-layer memory leak (boot §1b, `venv_sync` SKIP set)

pycapnp 2.2.2 leaks ~6 MB/s in `Event.new_message()` (≈666 k objects / 10 s). On
the comma three's 3.6 GB RAM this reaches OOM and triggers a panda SOM reset in
~60 s on-road. pycapnp 2.1.0 (the version openpilot 0.10.3 shipped) has no such
leak. `boot_patch.sh` downgrades to 2.1.0 (remount rw → `pip install
pycapnp==2.1.0` → remount ro), and `pycapnp` is in `venv_sync`'s `SKIP_PACKAGES`
so the sync never re-upgrades it back to the lockfile version. This is the
messaging-layer fix; there is no separate MSGQ patch in this plugin.

### Display — DRM instead of Wayland/Weston (boot §2c/5/7, `raylib_drm/`)

The comma three renders the raylib UI directly on `/dev/dri/card0` via a DRM
backend, not through the Wayland/Weston compositor the AGNOS 12.8 venv's stock
raylib expects.

- **§2c DRM raylib** — the plugin ships a DRM-built
  `_raylib_cffi.cpython-312-aarch64-linux-gnu.so` (plus the raylib Python package)
  under `raylib_drm/` (Git LFS). Boot copies it over the venv's Wayland build,
  detected by probing the `.so` for the `gbm_create_device` symbol.
- **§5 Weston** — `systemctl stop weston` + `mask weston` so raylib can take DRM
  master. The stock `weston-ready.service` is replaced with a `oneshot` /
  `RemainAfterExit=yes` `/bin/true` stub: AGNOS `comma.sh` polls
  `systemctl is-active weston-ready` up to ~200×; a masked service stays
  "inactive" (28 s timeout), the stub reports "active" immediately (~1 s boot).
- **§7 launch_env.sh** — the AGNOS < 16 Wayland env block is replaced with a
  plain `systemctl stop weston`, removing `WAYLAND_DISPLAY` etc.
- **§3 multilang** — `multilang.py`'s `.mo` loader has its `except
  FileNotFoundError` broadened to `except Exception` so the empty stub `.mo`
  files (from the msgfmt stub) don't crash the UI.

### Audio (boot §1)

v0.10.3 dropped the `"tici"` entry from `amplifier.py`'s `CONFIGS`. Boot
re-inserts the full tici `AmpConfig` / EQ block so the comma three speaker works.

### Panda — STM32F4 / Dos board (boot §8/8a/8b/9/10/11/12/13)

Upstream removed STM32F4 support (only STM32H7 red panda / tres / cuatro remain);
the comma three's internal panda is an F4 on `HW_TYPE_DOS` (0x06). Boot re-teaches
the panda library and pandad about F4:

- **§8 panda library** — adds `F4Config` + `McuType.F4` to `constants.py`; adds
  `HW_TYPE_DOS`, `F4_DEVICES`, extends `SUPPORTED_DEVICES` / `INTERNAL_DEVICES`,
  and restores `get_mcu_type()` in `__init__.py`.
- **§8a health struct padding** — F4 firmware v16 sends a shorter health packet
  than the v18 library `struct` expects; short reads are zero-padded so `unpack()`
  succeeds (new field defaults to 0).
- **§8b version check skip** — `ensure_version` short-circuits for F4 (firmware
  v16 vs library v18) instead of raising "Reflash panda".
- **§9 pandad** — skips firmware flashing for F4 (there is no `panda.bin.signed`;
  the BMW plugin owns firmware), caches `_mcu_type`, skips the `first_run`
  `reset(reconnect=True)` (F4 USB re-enumeration hangs the reconnect loop), adds a
  2 s USB-settle `sleep` so native `pandad` can claim the device, sets
  `BOARDD_SKIP_FW_CHECK=1`, and adds crash-backoff (5 s × crash count, capped 60 s)
  so a `pandad` crash-loop can't cascade into a SOM reset / red-LED hang.
- **§10–12 SPI off (USB-only)** — v0.10.3's native `pandad` SPI protocol is
  incompatible with the F4 and crash-loops on it. Boot `chmod 000
  /dev/spidev0.0`; `spi.py` maps the resulting `PermissionError` →
  `PandaSpiUnavailable`, and `dfu.py`'s `spi_list()` handler is broadened to
  `except Exception`, so the C++/Python panda code cleanly falls back to USB.
- **§13** — clears `panda/python/__pycache__` so the patched sources take effect.

### Modem / eSIM (boot §14)

The AGNOS 12.8 modem fails `AT+CCHO` (ISD-R open) with "Unknown error", which
crashes `configure_modem()` in `hardware.py` and blocks onroad/offroad
transitions. Boot wraps the `get_sim_lpa().is_comma_profile()` call in
`try/except` so modem init proceeds.

## Runtime hook — `compat.py`

`device.health_check` → `on_health_check(acc, **kwargs)` is called periodically by
plugind; its result dict is merged into the accumulator and published to the
plugin bus (captured into rlogs by bus_logger). It:

- reads AGNOS version (`/VERSION`) and device type
  (`/sys/firmware/devicetree/base/model` → tici / tizi / mici),
- reads `pandaStates[0].pandaType` via a cached `SubMaster`, and warns if the
  panda MCU doesn't match the device (expects Dos/F4 on tici, H7 on tizi/mici) —
  `DEVICE_MCU_EXPECTATIONS`.

On import (`log_startup_info`) it also logs AGNOS/device, warns if AGNOS major > 13
on a tici, and — because `updated` never runs under `DisableUpdates=True` —
populates `UpdaterCurrentDescription` from git so the Software panel still shows a
version string. This hook is diagnostic only; it does not gate anything.

## watchdog.sh (standalone diagnostic)

`watchdog.sh` snapshots vitals (uptime, `free -m`, top CPU/MEM, thermals, GPU
busy%, panda USB, filtered dmesg) to `/data/crash_diag/vitals.log` every 60 s and
a filtered `dmesg` to `dmesg_current.log` every 10 min (WiFi RCPI spam is grepped
out so it doesn't flood the ring buffer). **Note:** in the current tree nothing in
`boot_patch.sh` or the launch chain starts it — its header documents a manual
`setsid /data/plugins-runtime/c3_compat/watchdog.sh &`. Treat it as an on-demand
diagnostic tool, not an always-running service, unless it is wired up elsewhere.

## Files

```
c3_compat/
  plugin.json      # manifest: type=hook, device_filter=[tici], device.health_check
  boot_patch.sh    # all boot-time patches (sourced by launch_chffrplus.sh)
  venv_sync.py     # uv.lock → AGNOS 12.8 venv synchronizer (+ native deps)
  compat.py        # device.health_check hook + startup logging
  watchdog.sh      # on-demand vitals/dmesg snapshotter → /data/crash_diag/
  raylib_drm/      # DRM-backend raylib .so + package (Git LFS)
  __init__.py      # UI_BORDER_SIZE constant
  tests/test_compat.py
```

## Params / caches

No user params (`plugin.json params = {}`; nothing in a `data/` dir). State the
plugin keeps is internal cache files in the plugin's runtime dir
(`/data/plugins-runtime/c3_compat/`): `.venv_synced_hash` (last-synced uv.lock
hash) and `.native_deps_installed` (native-dep install marker). Path resolution
goes through `config.py` (`OPENPILOT_DIR`, `PLUGINS_RUNTIME_DIR`, `plugin_data_dir`).

## Device notes

- **Root FS is sealed read-only.** Every venv write (venv_sync, pycapnp
  downgrade, msgfmt stub, DRM raylib) does `mount -o remount,rw /` … work …
  `mount -o remount,ro /`, and re-seals in a `finally`/guard. The venv is
  root-owned, so installs go through `sudo`.
- **Persistence.** The systemd changes (Weston mask, `weston-ready` stub) and the
  on-disk stubs (`/data/pip_packages`, cache symlinks) persist across reboots and
  are re-checked idempotently each boot; the openpilot-tree source patches are
  re-applied after every overlay swap (guarded by `grep -q c3_compat` / marker
  checks so they no-op when already present).
- **AGNOS `/tmp` is a 150 MB tmpfs** — pip work is redirected to root-FS or
  `/data` TMPDIRs to avoid cross-device rename failures and space exhaustion.
