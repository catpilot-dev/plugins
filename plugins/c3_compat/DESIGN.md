# Comma 3 Compatibility — Design & Implementation

Infrastructure plugin that lets the comma three (SoC: Snapdragon 845; internal
panda: STM32F4 "Dos" board; OS: AGNOS 12.8) run the current catpilot, which is
built against newer hardware and a newer AGNOS. Upstream openpilot's last release
for the comma three was v0.10.0; this plugin restores its support as a set of **idempotent
boot-time patches** applied to the deployed openpilot tree plus one runtime
health hook — no maintained fork, no source overlays of the affected files.

`plugin.json`: `id = c3-compat`, `type = hook`, `device_filter = ["tici"]`,
`min_openpilot 0.11.2` / `max_openpilot 0.11.99`, `params = {}` (no user params).
The boot patcher is written against the openpilot 0.11.2 tree; it carries no
paths for older releases. The single registered hook is:

| hook | module.fn | priority |
|---|---|---|
| `device.health_check` | `compat.on_health_check` | 50 |

Everything else runs from `boot_patch.sh`, which is **not** invoked by plugind —
it is sourced by catpilot's `launch_chffrplus.sh` early in boot: after the
overlay swap and the plugin install (so the patcher that runs is the copy
`install.sh` just put in place), before the scons build:

```
launch_chffrplus.sh
  └─ install.sh (refreshes /data/plugins-runtime), first-boot setup
  └─ source /data/plugins-runtime/c3_compat/boot_patch.sh "$DIR"   # unless .disabled present
       ├─ patch the openpilot tree (panda, pandad, spi, dfu, multilang, hardware)
       ├─ copy src/ (AR0231 sensor, pandad USB transport) into the tree + hook them in
       ├─ install the comma three kernel UAPI headers under /usr/local/include
       ├─ venv_sync.py --runtime-only   (sync AGNOS 12.8 venv to uv.lock)
       ├─ install msgfmt stub + Adreno-linked raylib backend into the venv
       ├─ stop/mask Weston, install weston-ready stub, strip Wayland env
       └─ disable SPI, symlink caches
  └─ build.py (builder.py, scons), manager …
```

A catpilot launcher that still sources the patcher before running `install.sh`
runs the previously installed copy, so there a patcher update takes effect one
boot after it is deployed.

The `.disabled` marker in the plugin's runtime dir is the only opt-out, and it
gates the boot patch at the launcher (`[ ! -f …/.disabled ]`). On a comma three
`install.sh` writes `.enforced` (`[[ -f /TICI ]]`), so the plugin is always-on
and grayed out in the Plugins panel — see **Enforcement** below.

## Enforcement

On TICI hardware `install.sh` touches `plugins-runtime/c3_compat/.enforced`. The
device does not run catpilot correctly without these patches (no panda, no audio,
no UI, or an OOM reset in ~60 s on-road), so the plugin is treated as
must-stay-on. There is no user toggle. The `.disabled` escape hatch exists for
recovery/bring-up only and is honored by the launcher even when enforced. On
non-tici hardware the `device_filter` keeps the whole plugin dormant.

## Compatibility shims

Grouped by subsystem. Section numbers refer to `boot_patch.sh`.

### Python environment / AGNOS 12.8 (`venv_sync.py`, boot §1c/2/2a/2b/6)

catpilot's `uv.lock` is resolved for the AGNOS 16 venv; the comma three is capped
at AGNOS 12.8 with a different, root-owned, read-only Python 3.12 venv at
`/usr/local/venv`. `venv_sync.py` makes that venv match whatever code was
deployed, by any path (COD update, manual `git checkout`, AGNOS reflash):

- Parses `/data/openpilot/uv.lock` (stdlib `tomllib`), walks the
  dependency graph from the root package, and evaluates PEP 508 environment
  markers for the C3 target (`sys_platform = linux`, `platform_machine =
  aarch64`, `cp312`) so macOS/Windows/PyPy-only packages are skipped. Unknown
  markers default to "install" (safe). `--runtime-only` (the boot mode) drops the
  dev/test/docs optional groups.
- Selects only wheels compatible with `cp312` + `aarch64` + linux (or
  `py3-none-any`, or `abi3` ≤ cp312), batch-checks installed versions in one
  subprocess, then installs/upgrades the delta with
  `pip install --no-deps --no-cache-dir <wheel url>#sha256=<lock hash>` **into** the
  venv; pip verifies the `#sha256=` fragment and refuses a wheel that doesn't match
  `uv.lock` (no fragment when the lock entry has no hash).
- Wraps the install in `mount -o remount,rw /` … `remount,ro /` (root FS is
  sealed read-only — see Device notes), and sets `TMPDIR` to a root-FS path
  because AGNOS `/tmp` is a 150 MB tmpfs and pip can't `os.rename()` across
  filesystems.
- **Fast path**: after a successful sync, the SHA-256 of `uv.lock` and a venv
  fingerprint — the `st_mtime_ns` of the venv's `site-packages` directory, read
  after the installs — are cached in `plugins-runtime/c3_compat/data/.venv_synced_hash`
  (`data/` is the one part of the runtime dir `install.sh` keeps across its
  every-boot reinstall). The sync is skipped (<100 ms) only when both match. pip
  adding/removing a package or dist-info dir changes the mtime, and an AGNOS reflash
  (which resets the venv but not `/data`) puts back the image's value, so the
  persistent cache can't hide a changed venv. If `site-packages` can't be stat'ed
  the fast path is never taken; a cache file in the old hash-only format is a miss.
- The native libraries scons links (`comma-deps-*` wheels: capnproto, ffmpeg,
  zeromq, raylib, libusb, …) are ordinary `uv.lock` packages and are installed by
  the same sync; the ones upstream lists only under optional groups are kept even
  in `--runtime-only` mode.

Supporting boot patches:

- **§1c cache symlinks** — `~/.cache/pip` and `~/.cache/tinygrad` → `/data/cache/`.
  `/home` is a 100 MB overlay that the tinygrad model-compile cache and pip cache
  would fill.
- **§2a msgfmt stub** — AGNOS 12.8 has no gettext; scons needs `msgfmt` to build
  `.po` → `.mo`. The stub writes a valid empty `.mo` (correct binary header).
- **§6 PATH/PYTHONPATH** — patches `launch_chffrplus.sh` so scons sees the venv:
  `/usr/local/venv/bin` on PATH (cythonize), venv site-packages + `opendbc_repo`
  on PYTHONPATH (system `/usr/bin/scons` uses `/usr/bin/python3` and the DBC
  generator imports `opendbc`). The comma-deps-capnproto wheel's
  `capnproto/install/bin` goes first on PATH: `cereal/SConscript` runs a bare
  `capnpc`, and the headers it generates must match the wheel's
  libcapnp the build links, not the AGNOS 12.8 system capnpc. Keyed on the PATH
  line it writes (the launch script mentions c3_compat itself), so it is skipped
  on a catpilot launcher that sets all of this itself (its AGNOS < 16 block). The file edit takes
  effect from the **next** boot: `boot_patch.sh` is sourced from inside the
  launcher's `launch()`, which bash has already read. So the step also prepends
  `capnproto/install/bin` to the running shell's `PATH` on every pass, which covers the
  current boot's build. PYTHONPATH can't be fixed that way (the launcher assigns it
  after sourcing `boot_patch.sh`): on a tree whose launcher doesn't set them itself,
  the first boot after the tree is replaced runs without the venv site-packages /
  `opendbc_repo` entries.

### pycapnp / messaging-layer memory leak (boot §1b, `venv_sync` SKIP set)

pycapnp 2.2.2 leaks ~6 MB/s in `Event.new_message()` (≈666 k objects / 10 s). On
the comma three's 3.6 GB RAM this reaches OOM and triggers a panda SOM reset in
~60 s on-road. pycapnp 2.1.0 (the version openpilot 0.10.3 shipped) has no such
leak. `boot_patch.sh` downgrades to 2.1.0 (remount rw → `pip install
pycapnp==2.1.0` → remount ro), and `pycapnp` is in `venv_sync`'s `SKIP_PACKAGES`
so the sync never re-upgrades it back to the lockfile version. This is the
messaging-layer fix; there is no separate MSGQ patch in this plugin.

### Display — DRM instead of Wayland/Weston (boot §2d/5/7/7b, `raylib_drm/`)

The comma three renders the raylib UI directly on `/dev/dri/card0`, not through
the Wayland/Weston compositor AGNOS 12.8 starts. The UI's raylib is
`comma-deps-raylib` 6 (installed by venv_sync), whose "comma" backend
(GLES/EGL/gbm/drm, selected when `/AGNOS` exists) draws on DRM.

- **§2d Adreno-linked comma backend** — comma-deps-raylib 6's
  `_raylib_cffi_comma.abi3.so` links `libEGL.so.1`/`libGLESv2.so.2`/`libgbm.so.1`, which
  on AGNOS 12.8 are Mesa (GLVND). Mesa's msm driver cannot drive the 4.9 downstream
  kernel, so EGL lands on llvmpipe, which segfaults on the first frame (threads
  `llvmpipe-N`, `libgallium-24.2.8`). The comma three renders through Qualcomm's
  `libEGL_adreno.so` + `libGLESv2_adreno.so` on Qualcomm's `libgbm.so` (backend
  `msm_drm`), and that EGL offers
  the ABGR8888 (`AB24`) visual the backend asks for, so no format change is needed.
  `raylib_drm/comma-<wheel version>/_raylib_cffi_comma.abi3.so` is the wheel's own file
  with three `DT_NEEDED` entries rewritten:
  `patchelf --replace-needed libEGL.so.1 libEGL_adreno.so --replace-needed libGLESv2.so.2
  libGLESv2_adreno.so --replace-needed libgbm.so.1 libgbm.so <file>` (symbols are
  unversioned). Boot swaps it in only when the venv file's sha256 is the known wheel
  original (kept as `.orig`); an unknown hash (wheel bump) logs a WARNING and leaves the
  venv alone — re-run the recipe and update the hash. Dead ends worth remembering:
  `LD_PRELOAD` is ignored because the venv python has `cap_net_bind_service` (secure-exec
  mode); binary-patching the backend's `GBM_FORMAT_ABGR8888` immediates to ARGB8888 does
  make Mesa's config search pass but still ends on llvmpipe.
- **§7b `NO_MASTER=1`** (launch_env.sh) — the comma backend takes its DRM fd from a broker
  (`magic`, `/tmp/drmfd.sock`, AGNOS 19) and otherwise opens `/dev/dri/card0` itself only
  when `NO_MASTER` is set (`DRM_FD=<inherited fd>` is the other option). No broker on
  AGNOS 12.8, so the launch tree exports it. The launcher has already sourced
  `launch_env.sh` when `boot_patch.sh` runs, so the file edit (like §7's) takes effect
  from the next boot; `boot_patch.sh` also does `export NO_MASTER=1` in the running
  launch shell on every pass, which covers the current one. Touch is `/dev/input/event2` (Samsung
  Touchscreen here, matches), backlight `panel0-backlight`, `som_id` 8 → 90° rotation.
- **§5 Weston** — `systemctl stop weston` + `mask weston` so raylib can take DRM
  master. The stock `weston-ready.service` is replaced with a `oneshot` /
  `RemainAfterExit=yes` `/bin/true` stub: AGNOS `comma.sh` polls
  `systemctl is-active weston-ready` up to ~200×; a masked service stays
  "inactive" (28 s timeout), the stub reports "active" immediately (~1 s boot).
- **§7 launch_env.sh** — the AGNOS < 16 Wayland env block is replaced with a
  plain `systemctl stop weston`, removing `WAYLAND_DISPLAY` etc. Keyed on the
  `export WAYLAND_DISPLAY=` line (the file mentions c3_compat itself). catpilot's
  `launch_env.sh` no longer carries that block and exports `NO_MASTER` itself; §7
  and §7b's file edits skip there.
- **§3 multilang** — `multilang.py`'s `.mo` loader has its `except
  FileNotFoundError` broadened to `except Exception` so the empty stub `.mo`
  files (from the msgfmt stub) don't crash the UI.

### Audio — no patch

Upstream applies one amplifier `CONFIG` list on every device (no per-device
dict since openpilot 0.11.1), so the comma three needs no amplifier patch.

### Panda — STM32F4 / Dos board (boot §8/8b/9b/9c/10/11/12/13)

Upstream removed STM32F4 support (only STM32H7 red panda / tres / cuatro remain);
the comma three's internal panda is an F4 on `HW_TYPE_DOS` (0x06). Boot re-teaches
the panda library and pandad about F4:

- **§8 panda library** — adds `F4Config` + `McuType.F4` to `constants.py`; adds
  `HW_TYPE_DOS`, `F4_DEVICES`, extends `SUPPORTED_DEVICES` / `INTERNAL_DEVICES`,
  and restores `get_mcu_type()` in `__init__.py`.
- **§8b version check skip** — `ensure_version` short-circuits for F4 (its
  firmware's packet versions are older than the library's) instead of raising
  "Reflash panda". That does not make the health packet parse: see §9c.
- **§9b pandad.py** (`panda_ready()` / `flash_panda()` via DFU, bare `./pandad`) —
  on tici: no `HARDWARE.reset_internal_panda()` each pass (the USB
  re-enumeration outruns `panda_ready`'s 1 s wait), an F4 passes `panda_ready()`
  without the H7 `up_to_date()` check (there is no F4 image in the tree; the BMW
  plugin owns that firmware), and `flash_panda()` — BOOT0 → DFU → recover + flash
  the **H7** image — is never reached (retry instead). Before starting native
  `pandad`: a 2 s USB-settle `sleep` so it can claim the device, and
  `BOARDD_SKIP_FW_CHECK=1`; after it exits non-zero: a crash backoff (5 s × crash
  count, capped at 60 s) so a `pandad` crash-loop can't cascade into a SOM reset /
  red-LED hang. All four anchors or none.
- **§9c pandad.py lost-heartbeat check** — at start pandad.py reads
  `Panda.health()["heartbeat_lost"]` to set `PandaHeartbeatLost`. The F4 answers
  with the v16 health packet, which the Python library reads with its own layout
  (§8b only skips the version check), so that field would be an unrelated bit.
  The check is skipped for an F4. Applied on top of §9b, with its own marker.
- **§10–12 SPI off (USB-only)** — native `pandad`'s SPI protocol is
  incompatible with the F4 and crash-loops on it. Boot `chmod 000
  /dev/spidev0.0`; `spi.py` maps the resulting `PermissionError` from its
  `open(DEV_PATH)` → `PandaSpiUnavailable`, and `dfu.py`'s `spi_list()` handler is
  broadened to `except Exception`, so the Python panda code cleanly falls back to
  USB. Native `pandad` gets its USB transport from §16.
- **§13** — clears `panda/python/__pycache__` so the patched sources take effect.

### Native device type (boot §19)

openpilot 0.11.2 dropped `tici` from `HardwareComma::get_device_type()`'s map
(`common/hardware/comma/hardware.h`), so native pandad asserted right after connecting to
the panda (`Assertion it != device_map.end()`); loggerd/camerad would follow. §19 restores
the 0.11.1 entries: `{"tici", TICI}` in the map, IR LEDs left alone on tici (as on tizi), and
loggerd.h's livestream encoder size for tici = tizi's 1152x720 (same 1928x1208 road camera).

### Compiled comma-three support (boot §15–18, `src/`)

**F4 health packet (§16c, `src/pandad/health_f4.h`).** The BMW plugin's F4 firmware answers USB 0xd2 with
health v16 (58 bytes: uint32 voltage/current, one byte per boolean, float interrupt load). 0.11.2's
`health_t` is 48 bytes with uint16 voltage/current and a `flags_pkt` bitfield, so a straight read shifts
every field after `uptime`: ignition_line landed in safety_param, flags were read out of faults_pkt
(always 0) and the car never went onroad (first 0.11.2 drive attempt: VBAT 15.3 V, ignitionLine False,
safety 'silent', alternativeExperience 4864 = harness/safety bytes). `Panda::get_state()` now reads
`health_f4_t` on a dos panda and `health_from_f4()` assembles the 0.11.2 struct (flags from the booleans,
interrupt load scaled to uint8, sound level/temperature 0). Host-compiled tests: `tests/test_health_f4.py`.
The Python library (`Panda.health()`) is not taught the v16 layout: §8b only skips its version check, so
on the F4 its fields after `uptime` are wrong. Its one on-device caller, pandad.py's lost-heartbeat check,
is skipped for the F4 (§9c).

**AR0231 BPS IO-config blob (§15, `src/camerad/bps_cfg_ar0231.inc`).** 0.11.2's auto-generated
`cameras/bps_blobs.h` ships `bps_cfg[1]` (cereal `ImageSensor::ar0231`) as an empty placeholder.
The cabin camera runs the BPS/ICP pipeline, so `configICP()` handed the ICP firmware an all-zero
IO config: A5 watchdog 3 ms after the firmware download, `SFR: Assertion Failure ... hlssr.c`,
acquire timeout, `assert(h)` in camerad. §15 writes 0.11.1's 768-byte entry back (the OX03C10
entry is byte-identical across the two releases, so the blob format is unchanged).

Three pieces of comma-three support were never boot patches: upstream removed
them with the comma three and catpilot carried them as in-repo C++ (commit
`7a4e44e1f`) until the openpilot 0.11.2 rebase dropped that commit. Since
0.11.2 they live here: the plugin ships the sources under `src/` and boot copies
them into the tree before scons runs, with the hooks applied as text patches on
the stock files (guarded by `c3_compat` markers like every other section).

- **§15 camerad AR0231** — the comma three's image sensor; 0.11.2 ships only
  the 3X/four sensors (OS04C10, OX03C10). `src/camerad/ar0231.cc` +
  `ar0231_registers.h` are copied into `system/camerad/sensors/`; hooks: the
  `SConscript` source list, the class declaration in `sensor.h`, AR0231 first in
  `spectra.cc::openSensor`, the encoder-aligned NV12 buffer size in
  `camera_common.cc` (Venus needs more than `VENUS_BUFFER_SIZE` above 1344 px),
  and the AR0231 exposure target / wide-camera AE rectangle in `camera_qcom2.cc`.
  `SensorInfo` is unchanged since the driver last built upstream.
- **§16 pandad over USB** — 0.11.2's native `pandad` is SPI-only
  (`PandaSpiHandle`; `Panda::list()` is the SPI list) and §10 blocks SPI.
  `src/pandad/panda_usb.cc` is a libusb `PandaUsbHandle`; `panda_comms.h` gains
  a `PandaCommsHandle` base that both transports implement, `Panda` holds the
  base, connects USB first and falls back to SPI, and lists USB then SPI pandas.
  SConstruct links only libraries a vendored package provides, so `libusb` joins
  `pkg_names` (the comma-deps-libusb wheel from uv.lock supplies include + lib
  dirs) and the pandad `SConscript` adds `panda_usb.cc` and `usb-1.0`.
- **§17 loggerd v4l_encoder** — on AGNOS 12.8 the VIDC device node exists
  before the Venus firmware is loaded; the stock constructor asserts on the
  first failed `VIDIOC_QUERYCAP` and encoderd crash-loops at boot. The open is
  retried every 500 ms for ~15 s instead.
- **§18 kernel UAPI headers** — 0.11.2 takes the Qualcomm camera/video headers
  from the system (#37993): `media/cam_*.h`, `media/msm_camsensor_sdk.h`,
  `media/msm_media_info.h`, the VIDC control IDs in `linux/v4l2-controls.h`, and the ION
allocator headers `linux/ion.h` + `linux/msm_ion.h` that msgq's `visionbuf_ion.cc` needs (ION left
mainline Linux in 5.11, so Ubuntu's headers never had them).
  AGNOS 16+ has them in `/usr/include`; AGNOS 12.8 has none of `media/` and the
  stock Ubuntu `v4l2-controls.h`. `src/linux_include/` carries the comma three
  kernel's own set (the 0.11.1 `third_party/linux/include` files camerad built
  against there), installed under `/usr/local/include`, which the compiler
  searches before `/usr/include`, so no build-system change is needed.

Verified off-device: the patched `pandad` builds and links on x86 against the
0.11.2 tree; `spectra.cc`, `camera_qcom2.cc`, `camera_common.cc`, `ar0231.cc`
and `v4l_encoder.cc` pass `clang++ -fsyntax-only` with the §18 header set
(camerad itself only builds for `comma_arm64`). The device build is the gate.

Build-system items also handled: the comma-deps-* wheels (venv_sync installs
them from uv.lock), and the capnproto wheel's `capnpc` first on PATH (§6).

## Runtime hook — `compat.py`

`device.health_check` → `on_health_check(acc, **kwargs)` is called periodically by
plugind; its result dict is merged into the accumulator and published to the
plugin bus (captured into rlogs by bus_logger). It:

- reads AGNOS version (`/VERSION`) and device type
  (`/sys/firmware/devicetree/base/model` → tici / tizi / mici),
- reads `pandaStates[0].pandaType` via a cached `SubMaster`, and warns if a tici
  doesn't report a Dos (F4) panda (the plugin only loads on tici, see `device_filter`).

On its first call (`log_startup_info`, once per plugind run) it also logs
AGNOS/device through `cloudlog`, warns if AGNOS major > 13 on a tici, and — because
`updated` never runs under `DisableUpdates=True` — populates
`UpdaterCurrentDescription` from git so the Software panel still shows a version
string (version from `openpilot/common/version.h`). The module does nothing at
import time: every process that runs plugin hooks imports it, and only plugind
dispatches `device.health_check`. This hook is diagnostic only; it does not gate
anything.

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
  venv_sync.py     # uv.lock → AGNOS 12.8 venv synchronizer
  compat.py        # device.health_check hook + startup logging
  watchdog.sh      # on-demand vitals/dmesg snapshotter → /data/crash_diag/
  raylib_drm/      # comma-<wheel version>/: Adreno-relinked raylib 6 comma backend (Git LFS)
  src/camerad/     # AR0231 sensor driver (ar0231.cc, ar0231_registers.h) -> system/camerad/sensors/
  src/pandad/      # panda_usb.cc (libusb PandaUsbHandle), health_f4.h (F4 health v16) -> selfdrive/pandad/
  src/linux_include/  # comma three kernel UAPI headers (media/*.h, linux/{ion,msm_ion,v4l2-controls}.h) -> /usr/local/include
  __init__.py      # package marker (empty; the tests import plugins.c3_compat.*)
  tests/test_compat.py
  tests/test_venv_sync.py
  tests/test_health_f4.py   # host-compiles src/pandad/health_f4.h
```

## Params / caches

No user params (`plugin.json params = {}`). The only state the plugin keeps is
`data/.venv_synced_hash` in its runtime dir (`/data/plugins-runtime/c3_compat/`):
the last-synced uv.lock hash plus the venv fingerprint (see the venv_sync fast path).
It lives in `data/` because `install.sh` deletes and re-copies the rest of the
runtime dir on every boot. Path resolution goes through `config.py`
(`OPENPILOT_DIR`, `PLUGINS_RUNTIME_DIR`, `plugin_data_dir`).

## Device notes

- **Root FS is sealed read-only.** Every venv write (venv_sync, pycapnp
  downgrade, msgfmt stub, raylib backend swap) does `mount -o remount,rw /` … work …
  `mount -o remount,ro /`, and re-seals in a `finally`/guard. The venv is
  root-owned, so installs go through `sudo`.
- **Persistence.** The systemd changes (Weston mask, `weston-ready` stub) and the
  cache symlinks persist across reboots and are re-checked idempotently each
  boot; the openpilot-tree source patches are re-applied after every overlay swap
  (guarded by `grep -q c3_compat` / marker checks so they no-op when already
  present). A `/data/pip_packages` directory left by plugin versions for
  openpilot ≤ 0.11.1 (stub modules) is no longer written or read and can be deleted.
- **AGNOS `/tmp` is a 150 MB tmpfs** — pip work is redirected to a root-FS
  TMPDIR to avoid cross-device rename failures and space exhaustion.
- **Boot log.** Each tree-patching section reports what it actually did:
  `[c3_compat] Patched …` only when it wrote the change, otherwise
  `[c3_compat] WARNING …: anchor not found, left untouched`. Sections with several
  replacements name the missing anchors or report `n/m` (§8 applies its found
  pieces independently, §9b only all four or none). The pass ends with
  `Boot patch pass complete`, which claims nothing about the individual sections.
