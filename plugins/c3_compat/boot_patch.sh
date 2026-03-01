#!/usr/bin/env bash
# c3_compat boot patcher — runs before openpilot build/launch on AGNOS 12.8
# Called from /data/continue.sh to apply Comma 3 compatibility fixes
#
# Patches are idempotent — safe to run multiple times.

# NOTE: Do NOT use 'set -e' here — this script is sourced by continue.sh,
# and set -e would propagate to the parent shell, killing the launch chain
# on any non-zero exit from build.py, manager.py, etc.
OPENPILOT_DIR="${1:-/data/openpilot}"

echo "[c3_compat] Applying AGNOS 12.8 patches to $OPENPILOT_DIR"

# 0. Prevent overlay swap from wiping our patches
#    launch_chffrplus.sh checks .overlay_init + finalized/.overlay_consistent
#    and swaps /data/openpilot with the staging copy (losing all our patches).
#    Remove the marker so the swap is skipped — updates go through COD instead.
if [ -f "$OPENPILOT_DIR/.overlay_init" ]; then
  rm -f "$OPENPILOT_DIR/.overlay_init"
  echo "[c3_compat] Removed .overlay_init (prevents stale overlay swap)"
fi

# 1. Amplifier: add tici config (removed in v0.10.3 which dropped C3 support)
AMP_FILE="$OPENPILOT_DIR/openpilot/system/hardware/tici/amplifier.py"
if [ -f "$AMP_FILE" ] && ! grep -q '"tici"' "$AMP_FILE"; then
  python3 - "$AMP_FILE" << 'PYEOF'
import sys
path = sys.argv[1]
with open(path) as f:
    content = f.read()

tici_config = '''  "tici": [
    AmpConfig("Right speaker output from right DAC", 0b1, 0x2C, 0, 0b11111111),
    AmpConfig("Right Speaker Mixer Gain", 0b00, 0x2D, 2, 0b00001100),
    AmpConfig("Right speaker output volume", 0x1c, 0x3E, 0, 0b00011111),
    AmpConfig("DAI2 EQ enable", 0b1, 0x49, 1, 0b00000010),

    *configs_from_eq_params(0x84, EQParams(0x274F, 0xC0FF, 0x3BF9, 0x0B3C, 0x1656)),
    *configs_from_eq_params(0x8E, EQParams(0x1009, 0xC6BF, 0x2952, 0x1C97, 0x30DF)),
    *configs_from_eq_params(0x98, EQParams(0x0F75, 0xCBE5, 0x0ED2, 0x2528, 0x3E42)),
    *configs_from_eq_params(0xA2, EQParams(0x091F, 0x3D4C, 0xCE11, 0x1266, 0x2807)),
    *configs_from_eq_params(0xAC, EQParams(0x0A9E, 0x3F20, 0xE573, 0x0A8B, 0x3A3B)),
  ],
'''
content = content.replace('CONFIGS = {\n  "tizi"', 'CONFIGS = {\n' + tici_config + '  "tizi"')
with open(path, 'w') as f:
    f.write(content)
PYEOF
  echo "[c3_compat] Patched amplifier.py with tici config"
fi

# 1b. Cache directories: symlink ~/.cache/pip and ~/.cache/tinygrad to /data/
#     /home is a 100MB overlay that fills up fast. tinygrad cache (model compilation)
#     and pip cache (venv_sync) need to live on /data/ (16GB+ available).
CACHE_DIR="/home/comma/.cache"
for subdir in pip tinygrad; do
  if [ ! -L "$CACHE_DIR/$subdir" ]; then
    mkdir -p "/data/cache/$subdir"
    rm -rf "$CACHE_DIR/$subdir"
    ln -sfn "/data/cache/$subdir" "$CACHE_DIR/$subdir"
  fi
done

# 2. Venv patching: install missing tools and packages directly into the venv
#    AGNOS 12.8 root is read-only — remount rw, patch venv, remount ro.
#    This avoids fragile PYTHONPATH juggling with /data/pip_packages + c3_compat/site-packages.
#    Everything lives in /usr/local/venv/ — the single source of truth.
VENV_BIN="/usr/local/venv/bin"
VENV_SITE="/usr/local/venv/lib/python3.12/site-packages"
_venv_patched=0

# 2a. msgfmt stub: AGNOS 12.8 lacks gettext — scons needs it for .po → .mo
if [ ! -f "$VENV_BIN/msgfmt" ]; then
  sudo mount -o remount,rw / 2>/dev/null
  sudo tee "$VENV_BIN/msgfmt" > /dev/null << 'STUB'
#!/bin/sh
# Stub: c3_compat — produce valid empty .mo files (proper gettext binary format)
while [ $# -gt 0 ]; do
  case "$1" in -o) shift; OUTPUT="$1" ;; esac
  shift
done
[ -n "$OUTPUT" ] && printf '\xde\x12\x04\x95\x00\x00\x00\x00\x00\x00\x00\x00\x1c\x00\x00\x00\x1c\x00\x00\x00\x00\x00\x00\x00\x1c\x00\x00\x00' > "$OUTPUT"
STUB
  sudo chmod +x "$VENV_BIN/msgfmt"
  _venv_patched=1
  echo "[c3_compat] Installed msgfmt stub to venv"
fi

# 2b. AGNOS-specific packages: not in uv.lock, needed by AGNOS system services
#     kaitaistruct: used by AGNOS system daemons, not declared in openpilot's pyproject.toml
for pkg in kaitaistruct; do
  if ! /usr/local/venv/bin/python3 -c "import $pkg" 2>/dev/null; then
    [ $_venv_patched -eq 0 ] && sudo mount -o remount,rw / 2>/dev/null
    sudo -E /usr/local/venv/bin/pip install --no-cache-dir "$pkg" -q 2>/dev/null || true
    _venv_patched=1
    echo "[c3_compat] Installed AGNOS shim: $pkg"
  fi
done

# 2d. venv_sync: ensure venv matches deployed branch's uv.lock BEFORE launch.
#     Compares each package in uv.lock against what's installed in the venv.
#     Installs anything missing or at wrong version. Fast path: if uv.lock hash
#     matches cached .venv_synced_hash, skip entirely (<100ms).
#     This guarantees openpilot won't crash on import errors regardless of how
#     the code was deployed (COD update, manual git checkout, AGNOS reflash).
VENV_SYNC="/data/plugins/c3_compat/venv_sync.py"
if [ -f "$VENV_SYNC" ] && [ -f /data/openpilot/uv.lock ]; then
  /usr/local/venv/bin/python3 "$VENV_SYNC" --runtime-only 2>&1 | while IFS= read -r line; do
    echo "[c3_compat] $line"
  done
fi

# 2c. DRM raylib: AGNOS 12.8 venv has Wayland raylib, but C3 uses DRM backend
#     Copy DRM-built raylib from plugin's raylib_drm/ into venv (overwrites Wayland version)
RAYLIB_DRM="/data/plugins/c3_compat/raylib_drm"
if [ -d "$RAYLIB_DRM/raylib" ] && ! /usr/local/venv/bin/python3 -c "import raylib" 2>&1 | grep -q 'DRM\|STATIC'; then
  [ $_venv_patched -eq 0 ] && sudo mount -o remount,rw / 2>/dev/null
  sudo cp -rf "$RAYLIB_DRM/raylib/"* "$VENV_SITE/raylib/"
  _venv_patched=1
  echo "[c3_compat] Installed DRM raylib to venv"
fi

# Re-seal root filesystem
if [ $_venv_patched -eq 1 ]; then
  sudo mount -o remount,ro / 2>/dev/null || true
fi

# 3. Multilang: patch to handle invalid/empty .mo files gracefully
MULTILANG_FILE="$OPENPILOT_DIR/openpilot/system/ui/lib/multilang.py"
if [ -f "$MULTILANG_FILE" ] && grep -q 'except FileNotFoundError' "$MULTILANG_FILE"; then
  sed -i 's/except FileNotFoundError:/except Exception:/' "$MULTILANG_FILE"
  echo "[c3_compat] Patched multilang.py to handle invalid .mo files"
fi

# 5a. UI FPS: set tici (C3) to 20 FPS like tizi (C3X)
#     v0.10.3 dropped C3 from the FPS dict — defaults to 60 FPS, wasting ~15% CPU.
#     C3's Snapdragon 845 GPU is fine at 20 FPS for the offroad UI.
APP_FILE="$OPENPILOT_DIR/system/ui/lib/application.py"
if [ -f "$APP_FILE" ] && grep -q "{'tizi': 20}" "$APP_FILE"; then
  sed -i "s/{'tizi': 20}/{'tizi': 20, 'tici': 20}/" "$APP_FILE"
  echo "[c3_compat] Patched UI FPS: tici → 20 FPS (was defaulting to 60)"
fi

# 5. Display: DRM backend (no Weston compositor)
#    Must stop Weston so raylib can get DRM master on /dev/dri/card0
sudo systemctl stop weston 2>/dev/null || true

# 6. PATH + PYTHONPATH: make venv tools and packages visible to scons
#    scons at /usr/bin/scons uses #!/usr/bin/python3 which can't see venv packages.
#    scons also shells out to cythonize, which lives in /usr/local/venv/bin/.
#    DBC generator imports 'opendbc' — needs opendbc_repo on PYTHONPATH.
LAUNCH_FILE="$OPENPILOT_DIR/launch_chffrplus.sh"
if [ -f "$LAUNCH_FILE" ] && ! grep -q 'c3_compat' "$LAUNCH_FILE"; then
  python3 - "$LAUNCH_FILE" << 'PYEOF'
import sys
path = sys.argv[1]
with open(path) as f:
    content = f.read()
# Add venv/bin to PATH (for cythonize, scons tools)
content = content.replace(
    'export PYTHONPATH="$PWD"',
    '# c3_compat: venv bin for cythonize, venv site-packages + opendbc_repo for scons\n'
    'export PATH="/usr/local/venv/bin:$PATH"\n'
    'export PYTHONPATH="$PWD:$PWD/opendbc_repo:/usr/local/venv/lib/python3.12/site-packages"'
)
with open(path, 'w') as f:
    f.write(content)
PYEOF
  echo "[c3_compat] Patched launch_chffrplus.sh PATH + PYTHONPATH"
fi

# 7. launch_env.sh: remove Wayland env vars (DRM backend uses /dev/dri/card0 directly)
LAUNCH_ENV="$OPENPILOT_DIR/launch_env.sh"
if [ -f "$LAUNCH_ENV" ] && grep -q 'WAYLAND_DISPLAY' "$LAUNCH_ENV" && ! grep -q 'c3_compat' "$LAUNCH_ENV"; then
  python3 - "$LAUNCH_ENV" << 'PYEOF'
import sys, re
path = sys.argv[1]
with open(path) as f:
    content = f.read()
# Replace the AGNOS < 16 Wayland block with Weston stop
old_block = re.search(
    r'if \[ "\$AGNOS_MAJOR" -lt 16 \].*fi\n',
    content, re.DOTALL
)
if old_block:
    new_block = """if [ "$AGNOS_MAJOR" -lt 16 ] 2>/dev/null; then
  # c3_compat: DRM backend — stop Weston so raylib gets DRM master
  sudo systemctl stop weston 2>/dev/null || true
fi
"""
    content = content[:old_block.start()] + new_block + content[old_block.end():]
    with open(path, 'w') as f:
        f.write(content)
PYEOF
  echo "[c3_compat] Patched launch_env.sh for DRM backend (removed Wayland config)"
fi

# 8. Panda: restore STM32F4 (Dos board) support for C3 internal panda
#    v0.10.3 dropped F4 support — only H7 (red panda, tres, cuatro) remains
#    C3 has HW_TYPE_DOS (0x06) with STM32F4 MCU
PANDA_INIT="$OPENPILOT_DIR/panda/python/__init__.py"
PANDA_CONST="$OPENPILOT_DIR/panda/python/constants.py"
if [ -f "$PANDA_INIT" ] && ! grep -q 'HW_TYPE_DOS' "$PANDA_INIT"; then
  python3 - "$PANDA_INIT" "$PANDA_CONST" << 'PYEOF'
import sys
init_path, const_path = sys.argv[1], sys.argv[2]

# --- Patch constants.py: add F4Config and McuType.F4 ---
with open(const_path) as f:
    const = f.read()
if 'F4Config' not in const:
    f4_config = '''
F4Config = McuConfig(
  "STM32F4",
  0x463,
  [0x4000 for _ in range(4)] + [0x10000] + [0x20000 for _ in range(7)],
  12,
  0x1FFF7A10,
  0x800,
  0x1FFF79C0,
  0x8004000,
  "panda.bin.signed",
  0x8000000,
  "bootstub.panda.bin",
)

'''
    const = const.replace('H7Config = McuConfig(', f4_config + 'H7Config = McuConfig(')
    const = const.replace(
        'class McuType(enum.Enum):\n  H7 = H7Config',
        'class McuType(enum.Enum):\n  F4 = F4Config\n  H7 = H7Config'
    )
    with open(const_path, 'w') as f:
        f.write(const)

# --- Patch __init__.py: add HW_TYPE_DOS, update get_mcu_type, devices ---
with open(init_path) as f:
    init = f.read()

# Add HW_TYPE_DOS after HW_TYPE_BLACK
init = init.replace(
    "HW_TYPE_BLACK = b'\\x03'\n",
    "HW_TYPE_BLACK = b'\\x03'\n  HW_TYPE_DOS = b'\\x06'  # C3 internal panda (STM32F4)\n"
)

# Add F4 devices list and extend SUPPORTED_DEVICES
init = init.replace(
    'H7_DEVICES = [HW_TYPE_RED_PANDA, HW_TYPE_TRES, HW_TYPE_CUATRO, HW_TYPE_BODY]\n  SUPPORTED_DEVICES = H7_DEVICES',
    'H7_DEVICES = [HW_TYPE_RED_PANDA, HW_TYPE_TRES, HW_TYPE_CUATRO, HW_TYPE_BODY]\n  F4_DEVICES = [HW_TYPE_DOS]\n  SUPPORTED_DEVICES = H7_DEVICES + F4_DEVICES'
)

# Add DOS to INTERNAL_DEVICES
init = init.replace(
    'INTERNAL_DEVICES = (HW_TYPE_TRES, HW_TYPE_CUATRO)',
    'INTERNAL_DEVICES = (HW_TYPE_TRES, HW_TYPE_CUATRO, HW_TYPE_DOS)'
)

# Patch get_mcu_type to handle F4
init = init.replace(
    '  def get_mcu_type(self) -> McuType:\n    hw_type = self.get_type()\n    if hw_type in Panda.H7_DEVICES:\n      return McuType.H7\n    raise ValueError(f"unknown HW type: {hw_type}")',
    '  def get_mcu_type(self) -> McuType:\n    hw_type = self.get_type()\n    if hw_type in Panda.H7_DEVICES:\n      return McuType.H7\n    if hw_type in Panda.F4_DEVICES:\n      return McuType.F4\n    raise ValueError(f"unknown HW type: {hw_type}")'
)

with open(init_path, 'w') as f:
    f.write(init)
PYEOF
  echo "[c3_compat] Patched panda library with STM32F4 (Dos board) support"
fi

# 8b. Panda: skip packet version checks for F4 panda
#     F4 firmware is v16 but the v0.10.3 library expects v17 for health,
#     v5 for CAN health, etc. The struct layouts are compatible — only the
#     version counter changed. Without this patch pandad crash-loops calling
#     health() and eventually hangs the device.
if [ -f "$PANDA_INIT" ] && ! grep -q 'c3_compat.*version' "$PANDA_INIT"; then
  python3 - "$PANDA_INIT" << 'PYEOF'
import sys
path = sys.argv[1]
with open(path) as f:
    content = f.read()

# Patch ensure_version to skip check for F4 devices
old_ensure = '''def ensure_version(desc, lib_field, panda_field, fn):
  @wraps(fn)
  def wrapper(self, *args, **kwargs):
    lib_version = getattr(self, lib_field)
    panda_version = getattr(self, panda_field)
    if lib_version != panda_version:
      raise RuntimeError(f"{desc} packet version mismatch: panda\'s firmware v{panda_version}, library v{lib_version}. Reflash panda.")
    return fn(self, *args, **kwargs)
  return wrapper'''

new_ensure = '''def ensure_version(desc, lib_field, panda_field, fn):
  @wraps(fn)
  def wrapper(self, *args, **kwargs):
    # c3_compat: skip version check for F4 panda (firmware v16, library v17)
    from panda.python.constants import McuType
    if getattr(self, '_mcu_type', None) == McuType.F4:
      return fn(self, *args, **kwargs)
    lib_version = getattr(self, lib_field)
    panda_version = getattr(self, panda_field)
    if lib_version != panda_version:
      raise RuntimeError(f"{desc} packet version mismatch: panda\'s firmware v{panda_version}, library v{lib_version}. Reflash panda.")
    return fn(self, *args, **kwargs)
  return wrapper'''

if old_ensure in content:
    content = content.replace(old_ensure, new_ensure)
    with open(path, 'w') as f:
        f.write(content)
PYEOF
  echo "[c3_compat] Patched panda version check to skip for F4"
fi

# 9. Pandad: skip firmware flashing for F4 panda (BMW plugin handles firmware)
#    Without this, pandad would try to flash non-existent panda.bin.signed
PANDAD_FILE="$OPENPILOT_DIR/selfdrive/pandad/pandad.py"
if [ -f "$PANDAD_FILE" ] && ! grep -q 'c3_compat' "$PANDAD_FILE"; then
  python3 - "$PANDAD_FILE" << 'PYEOF'
import sys
path = sys.argv[1]
with open(path) as f:
    content = f.read()

# Replace flash_panda to skip firmware check for F4 devices
old = '''def flash_panda(panda_serial: str) -> Panda:
  try:
    panda = Panda(panda_serial)
  except PandaProtocolMismatch:'''

new = '''def flash_panda(panda_serial: str) -> Panda:
  try:
    panda = Panda(panda_serial)
  except PandaProtocolMismatch:'''

# Simpler approach: add F4 skip after firmware signature check
old_sig = '''  if panda.bootstub or panda_signature != fw_signature:
    cloudlog.info("Panda firmware out of date, update required")
    panda.flash()'''

new_sig = '''  # c3_compat: skip firmware flashing for F4 panda (BMW plugin handles firmware)
  from panda.python.constants import McuType
  if panda.get_mcu_type() == McuType.F4:
    cloudlog.warning("c3_compat: F4 panda detected, skipping firmware flash")
    return panda
  if panda.bootstub or panda_signature != fw_signature:
    cloudlog.info("Panda firmware out of date, update required")
    panda.flash()'''

if old_sig in content:
    content = content.replace(old_sig, new_sig)

# c3_compat: skip first_run reset for F4 pandas
# panda.reset(reconnect=True) disconnects USB then tries reconnect() which calls
# connect(wait=True) — this creates an infinite loop if F4 panda is slow to
# re-enumerate over USB after a soft reset (0xd8 control transfer).
old_reset = '''        if first_run:
          # reset panda to ensure we're in a good state
          cloudlog.info(f"Resetting panda {panda.get_usb_serial()}")
          panda.reset(reconnect=True)'''
new_reset = '''        if first_run:
          # c3_compat: skip reset for F4 panda (USB reconnect hangs with STM32F4)
          from panda.python.constants import McuType
          if panda.get_mcu_type() != McuType.F4:
            cloudlog.info(f"Resetting panda {panda.get_usb_serial()}")
            panda.reset(reconnect=True)
          else:
            cloudlog.info("c3_compat: skipping reset for F4 panda %s", panda.get_usb_serial())'''
if old_reset in content:
    content = content.replace(old_reset, new_reset)

# c3_compat: USB settle delay + BOARDD_SKIP_FW_CHECK + crash backoff
# - time.sleep(2): gives kernel time to release USB after Python closes handles
# - BOARDD_SKIP_FW_CHECK: F4 firmware files don't exist in v0.10.3
# - crash backoff: prevents crash loop → SOM reset → red LED hang
old_launch = '''    first_run = False

    # run pandad with all connected serials as arguments
    os.environ['MANAGER_DAEMON'] = 'pandad'
    process = subprocess.Popen(["./pandad", *panda_serials], cwd=os.path.join(BASEDIR, "selfdrive/pandad"))
    process.wait()'''
new_launch = '''    first_run = False

    # c3_compat: give kernel time to release USB device so native pandad can claim it
    time.sleep(2)

    # run pandad with all connected serials as arguments
    os.environ['MANAGER_DAEMON'] = 'pandad'
    # c3_compat: skip C++ firmware check for F4 panda
    os.environ["BOARDD_SKIP_FW_CHECK"] = "1"
    process = subprocess.Popen(["./pandad", *panda_serials], cwd=os.path.join(BASEDIR, "selfdrive/pandad"))
    process.wait()

    # c3_compat: crash backoff for F4 panda SPI incompatibility
    if process.returncode != 0:
      if not hasattr(main, '_crash_count'):
        main._crash_count = 0
      main._crash_count += 1
      backoff = min(60, 5 * main._crash_count)
      cloudlog.warning("c3_compat: native pandad crashed (code %d), backing off %ds (crash #%d)",
                       process.returncode, backoff, main._crash_count)
      time.sleep(backoff)
    else:
      main._crash_count = 0'''
if old_launch in content and 'BOARDD_SKIP_FW_CHECK' not in content:
    content = content.replace(old_launch, new_launch)

with open(path, 'w') as f:
    f.write(content)
PYEOF
  echo "[c3_compat] Patched pandad.py to skip F4 firmware flashing"
fi

# 10. SPI: disable for C3 (F4 panda) to force native pandad USB-only mode
#     v0.10.3's native pandad SPI protocol is incompatible with STM32F4.
#     Without this, native pandad crash-loops on SPI → SIGABRT → Python wrapper
#     restarts → panda heartbeat watchdog fires → SOM reset → red LED hang.
#     Blocking SPI makes the C++ Panda constructor's SPI fallback throw immediately,
#     so it connects via USB instead.
if [ -e /dev/spidev0.0 ]; then
  sudo chmod 000 /dev/spidev0.0
  echo "[c3_compat] Disabled SPI device (F4 panda USB-only mode)"
fi

# 11. SPI Python: handle PermissionError when SPI device is blocked
#     SpiDevice.__init__ opens /dev/spidev0.0 — blocked by chmod 000 above.
#     Without this patch, PandaSpiHandle() raises raw PermissionError instead
#     of PandaSpiException, bypassing the SPI error handlers in connect/list.
PANDA_SPI="$OPENPILOT_DIR/panda/python/spi.py"
if [ -f "$PANDA_SPI" ] && ! grep -q 'c3_compat' "$PANDA_SPI"; then
  python3 - "$PANDA_SPI" << 'PYEOF'
import sys
path = sys.argv[1]
with open(path) as f:
    content = f.read()

# Wrap SpiDev open in PermissionError handler
old = '''      if speed not in SPI_DEVICES:
        SPI_DEVICES[speed] = spidev.SpiDev()
        SPI_DEVICES[speed].open(0, 0)
        SPI_DEVICES[speed].max_speed_hz = speed'''

new = '''      if speed not in SPI_DEVICES:
        try:
          SPI_DEVICES[speed] = spidev.SpiDev()
          SPI_DEVICES[speed].open(0, 0)
          SPI_DEVICES[speed].max_speed_hz = speed
        except PermissionError:
          # c3_compat: SPI disabled for F4 panda USB-only mode
          raise PandaSpiUnavailable("SPI device permission denied (F4 USB-only mode)")'''

if old in content:
    content = content.replace(old, new)
    with open(path, 'w') as f:
        f.write(content)
PYEOF
  echo "[c3_compat] Patched spi.py PermissionError → PandaSpiUnavailable"
fi

# 12. DFU Python: broaden spi_list() exception handler
#     PandaDFU.spi_list() only catches PandaSpiException but PermissionError
#     can cascade as ValueError from SpiDev cleanup. Catch all exceptions.
PANDA_DFU="$OPENPILOT_DIR/panda/python/dfu.py"
if [ -f "$PANDA_DFU" ] && ! grep -q 'c3_compat' "$PANDA_DFU"; then
  python3 - "$PANDA_DFU" << 'PYEOF'
import sys
path = sys.argv[1]
with open(path) as f:
    content = f.read()

# Replace narrow exception handler with broad one
old = '''    except PandaSpiException:'''
new = '''    except Exception:  # c3_compat: catch PermissionError from disabled SPI'''

if old in content:
    content = content.replace(old, new, 1)  # only replace in spi_list, not elsewhere
    with open(path, 'w') as f:
        f.write(content)
PYEOF
  echo "[c3_compat] Patched dfu.py spi_list() exception handler"
fi

# 13. Clear Python bytecode cache for patched panda modules
#     Stale .pyc files cause patches to be ignored until cache expires.
find "$OPENPILOT_DIR/panda/python" -name '__pycache__' -type d -exec rm -rf {} + 2>/dev/null || true

# 14. Persistent crash diagnostics
#     AGNOS 12.8 uses volatile journal (/var/log is 128MB tmpfs) — all evidence
#     is lost on hang/reboot. Capture dmesg + system state to /data/ on each boot
#     so we can inspect the previous boot's final moments after a crash.
DIAG_DIR="/data/crash_diag"
mkdir -p "$DIAG_DIR"

# Rotate previous boot's dmesg before overwriting
# Keep last 3 boots: dmesg_boot1.log (prev), dmesg_boot2.log, dmesg_boot3.log
for i in 2 1; do
  [ -f "$DIAG_DIR/dmesg_boot${i}.log" ] && mv -f "$DIAG_DIR/dmesg_boot${i}.log" "$DIAG_DIR/dmesg_boot$((i+1)).log"
done
[ -f "$DIAG_DIR/dmesg_current.log" ] && mv -f "$DIAG_DIR/dmesg_current.log" "$DIAG_DIR/dmesg_boot1.log"

# Save THIS boot's dmesg ring buffer (kernel panics, OOM kills,
# USB disconnects, watchdog triggers, thermal shutdowns)
dmesg -T > "$DIAG_DIR/dmesg_current.log" 2>/dev/null || dmesg > "$DIAG_DIR/dmesg_current.log"

# Snapshot system state at boot time (baseline for comparison if hang occurs)
{
  echo "=== Boot $(date '+%Y-%m-%d %H:%M:%S') ==="
  echo "--- uptime ---"
  uptime
  echo "--- memory ---"
  free -m
  echo "--- panda USB ---"
  lsusb 2>/dev/null | grep -i 'panda\|1209\|bbaa\|3801' || echo "no panda USB device found"
  echo "--- thermal ---"
  cat /sys/class/thermal/thermal_zone*/temp 2>/dev/null | head -10
  echo "--- processes ---"
  ps aux --sort=-%mem | head -15
  echo "--- disk ---"
  df -h /data
} > "$DIAG_DIR/boot_state.log" 2>/dev/null

# Background watchdog: periodically save system vitals to catch state before hang
# Uses setsid to create a new session — survives the exec in continue.sh
WATCHDOG="/data/plugins/c3_compat/watchdog.sh"
if [ -f "$WATCHDOG" ]; then
  # Kill any stale watchdog from previous boot
  pkill -f "watchdog.sh" 2>/dev/null || true
  setsid "$WATCHDOG" &
  echo "[c3_compat] Crash diagnostics enabled (logs in $DIAG_DIR)"
fi

# On next boot after a hang, check these files:
#   /data/crash_diag/dmesg_boot1.log  — previous boot's kernel log
#   /data/crash_diag/vitals.log       — last system state before hang
#   /data/crash_diag/boot_state.log   — current boot baseline

echo "[c3_compat] Boot patches applied successfully"
