#!/usr/bin/env bash
# c3_compat boot patcher — runs before openpilot build/launch on AGNOS 12.8
# Called from /data/continue.sh to apply Comma 3 compatibility fixes
#
# Patches are idempotent — safe to run multiple times.

# NOTE: Do NOT use 'set -e' here — this script is sourced by continue.sh,
# and set -e would propagate to the parent shell, killing the launch chain
# on any non-zero exit from build.py, manager.py, etc.
OPENPILOT_DIR="${1:-/data/openpilot}"
# this plugin's runtime directory (artifacts under raylib_drm/ and src/ are addressed from here)
C3_COMPAT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
C3_SRC="$C3_COMPAT_DIR/src"

echo "[c3_compat] Applying AGNOS 12.8 patches to $OPENPILOT_DIR"

# 1. Amplifier: nothing to patch since openpilot 0.11.1 — upstream folded the
#    per-device CONFIGS dict into one CONFIG list that initialize_configuration()
#    applies on every device, and the tici-only entries left with it. The old
#    "tici" insertion keyed on CONFIGS = {"tizi" ...} has been a no-op since then.

# 1b. pycapnp: downgrade 2.2.2 → 2.1.0 to fix memory leak on C3
#     pycapnp 2.2.2 leaks ~6MB/s in Event.new_message() (666k objects/10s).
#     On C3 with 3.6GB RAM this causes OOM + panda SOM reset in ~60s on-road.
#     pycapnp 2.1.0 (used by openpilot 0.10.3) does not have this leak.
if python3 -c "import capnp; exit(0 if capnp.__version__ != '2.1.0' else 1)" 2>/dev/null; then
  echo "[c3_compat] Downgrading pycapnp to 2.1.0 (memory leak fix)"
  sudo mount -o remount,rw / 2>/dev/null
  sudo /usr/local/venv/bin/pip install --quiet pycapnp==2.1.0 2>/dev/null && echo "[c3_compat] pycapnp downgraded to 2.1.0" || echo "[c3_compat] WARNING: pycapnp downgrade failed"
  sudo mount -o remount,ro / 2>/dev/null
fi

# 1c. Cache directories: symlink ~/.cache/pip and ~/.cache/tinygrad to /data/
#     /home is a 100MB overlay that fills up fast. tinygrad cache (model compilation)
#     and pip cache (venv_sync) need to live on /data/ (16GB+ available).
CACHE_DIR="/home/comma/.cache"
mkdir -p "$CACHE_DIR"
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

# 2b. venv_sync: ensure venv matches deployed branch's uv.lock BEFORE launch.
#     Compares each package in uv.lock against what's installed in the venv.
#     Installs anything missing or at wrong version. Fast path: if uv.lock hash
#     matches cached .venv_synced_hash, skip entirely (<100ms).
#     This guarantees openpilot won't crash on import errors regardless of how
#     the code was deployed (COD update, manual git checkout, AGNOS reflash).
VENV_SYNC="/data/plugins-runtime/c3_compat/venv_sync.py"
if [ -f "$VENV_SYNC" ] && [ -f /data/openpilot/uv.lock ]; then
  PYTHONPATH="/data/plugins-runtime${PYTHONPATH:+:$PYTHONPATH}" /usr/local/venv/bin/python3 "$VENV_SYNC" --runtime-only --native-deps 2>&1 | while IFS= read -r line; do
    echo "[c3_compat] $line"
  done
fi

# 2c. DRM raylib (openpilot <= 0.11.1 only): the PyPI raylib 5.5 wheel in the
#     AGNOS 12.8 venv is a Wayland build; copy the DRM build from raylib_drm/
#     over it. openpilot 0.11.2 ships comma-deps-raylib 6 instead, which carries
#     its own DRM "comma" backend (auto-selected on /AGNOS) under different file
#     names — that package must never be touched (copying 5.5 files over it
#     breaks the UI). Keyed on the 5.5 .so being present and raylib/_backend.py absent.
RAYLIB_DRM="/data/plugins-runtime/c3_compat/raylib_drm"
_raylib_so="$VENV_SITE/raylib/_raylib_cffi.cpython-312-aarch64-linux-gnu.so"
if [ -d "$RAYLIB_DRM/raylib" ] && [ -f "$_raylib_so" ] && [ ! -f "$VENV_SITE/raylib/_backend.py" ] \
   && ! nm -D "$_raylib_so" 2>/dev/null | grep -q 'gbm_create_device'; then
  [ $_venv_patched -eq 0 ] && sudo mount -o remount,rw / 2>/dev/null
  sudo cp -rf "$RAYLIB_DRM/raylib/"* "$VENV_SITE/raylib/"
  _venv_patched=1
  echo "[c3_compat] Installed DRM raylib to venv"
fi

# Re-seal root filesystem
if [ $_venv_patched -eq 1 ]; then
  sudo mount -o remount,ro / 2>/dev/null || true
fi

# 2d. raylib 6 "comma" backend on the comma three (openpilot 0.11.2+). The
#     comma-deps-raylib wheel links its backend against libEGL.so.1 / libGLESv2.so.2 /
#     libgbm.so.1, which on AGNOS 12.8 resolve to Mesa (GLVND); Mesa's msm driver
#     cannot drive the 4.9 downstream kernel, falls back to llvmpipe and segfaults
#     on the first frame. The comma three renders through Qualcomm's libEGL_adreno /
#     libGLESv2_adreno on Qualcomm's libgbm (exactly what the 0.11.1 raylib_drm
#     build linked), whose GBM display offers the ABGR8888 visual the backend
#     asks for. The shipped module is the wheel's own file with three DT_NEEDED
#     entries rewritten (patchelf --replace-needed; recipe in DESIGN.md), keyed on
#     the wheel's sha256 so a wheel bump surfaces as a warning, never a swap.
#     LD_PRELOAD cannot do this: the venv python carries cap_net_bind_service, so
#     the loader runs in secure mode and ignores it.
_cb="$VENV_SITE/raylib/_raylib_cffi_comma.abi3.so"
_cb_src="$C3_COMPAT_DIR/raylib_drm/comma-6.0.0.1.post103/_raylib_cffi_comma.abi3.so"
_cb_orig_sha="ebb0b6c4112a39d700e06fd40947023485d3c14c0d0ac36abef8c6a9551d7502"
if [ -f "$_cb" ] && [ -f "$VENV_SITE/raylib/_backend.py" ] && [ -f "$_cb_src" ]; then
  _cb_sha=$(sha256sum "$_cb" | cut -d' ' -f1); _cb_src_sha=$(sha256sum "$_cb_src" | cut -d' ' -f1)
  if [ "$_cb_sha" = "$_cb_src_sha" ]; then
    :  # already the Adreno-linked module
  elif [ "$_cb_sha" = "$_cb_orig_sha" ]; then
    sudo mount -o remount,rw / 2>/dev/null
    sudo cp "$_cb" "$_cb.orig"
    sudo cp "$_cb_src" "$_cb"
    sudo mount -o remount,ro / 2>/dev/null || true
    echo "[c3_compat] Installed Adreno-linked raylib comma backend (original kept as .orig)"
  else
    echo "[c3_compat] WARNING: raylib comma backend has unknown sha256 ${_cb_sha:0:12} (new comma-deps-raylib wheel?) — not swapped; UI will fall back to llvmpipe. Re-run the patchelf recipe in DESIGN.md"
  fi
fi

# 3. Multilang: patch to handle invalid/empty .mo files gracefully
MULTILANG_FILE="$OPENPILOT_DIR/openpilot/system/ui/lib/multilang.py"
if [ -f "$MULTILANG_FILE" ] && grep -q 'except FileNotFoundError' "$MULTILANG_FILE"; then
  sed -i 's/except FileNotFoundError:/except Exception:/' "$MULTILANG_FILE"
  echo "[c3_compat] Patched multilang.py to handle invalid .mo files"
fi

# 5. Display: DRM backend (no Weston compositor)
#    Must stop Weston so raylib can get DRM master on /dev/dri/card0.
#    Also replace weston-ready with a no-op stub to eliminate the 28s boot
#    timeout in AGNOS comma.sh which polls `systemctl is-active weston-ready`.
#    A masked service stays "inactive" (comma.sh loops 200x), but a stub with
#    RemainAfterExit=yes exits instantly and stays "active" (1s boot).
#    Persists across reboots — only needs root rw once.
sudo systemctl stop weston 2>/dev/null || true
if ! grep -q 'c3_compat' /etc/systemd/system/weston-ready.service 2>/dev/null; then
  sudo mount -o remount,rw / 2>/dev/null
  sudo systemctl mask weston 2>/dev/null || true
  sudo tee /etc/systemd/system/weston-ready.service > /dev/null << 'WESTONEOF'
[Unit]
Description=Weston ready stub (c3_compat: DRM mode, no Weston needed)

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/bin/true

[Install]
WantedBy=multi-user.target
WESTONEOF
  sudo systemctl daemon-reload
  sudo systemctl enable weston-ready 2>/dev/null || true
  sudo mount -o remount,ro / 2>/dev/null || true
  echo "[c3_compat] Installed weston-ready stub (eliminates 28s boot delay)"
fi

# 5b. commaai/dependencies stubs: v0.11.0 SConstruct imports pkg modules
#     (bzip2, capnproto, eigen, ...) that are only pre-installed on AGNOS 16+.
#     On AGNOS 12.8 we stub them to return system library paths so scons can parse.
#     /data/pip_packages is LAST on PYTHONPATH (launch_chffrplus.sh), so the real
#     packages win where installed. openpilot 0.11.2 needs acados/json11 too and
#     ships every one of them as a comma-deps-* wheel that venv_sync (2b)
#     installs from uv.lock — there the stubs are inert and the wheels are required.
if [ ! -f /data/pip_packages/bzip2.py ]; then
  mkdir -p /data/pip_packages
  for pkg_spec in \
    "bzip2:/usr/include:/usr/lib/aarch64-linux-gnu" \
    "capnproto:/usr/local/include:/usr/local/lib" \
    "eigen:/usr/include/eigen3:/usr/local/lib" \
    "ffmpeg:/usr/local/include:/usr/local/lib" \
    "libjpeg:/usr/include:/usr/lib/aarch64-linux-gnu" \
    "libyuv:/usr/include:/usr/lib/aarch64-linux-gnu" \
    "ncurses:/usr/include:/usr/lib/aarch64-linux-gnu" \
    "zeromq:/usr/include:/usr/lib/aarch64-linux-gnu" \
    "zstd:/usr/include:/usr/lib/aarch64-linux-gnu"; do
    name="${pkg_spec%%:*}"
    rest="${pkg_spec#*:}"
    inc="${rest%%:*}"
    lib="${rest#*:}"
    cat > "/data/pip_packages/${name}.py" << STUBEOF
# c3_compat: commaai/dependencies stub for AGNOS 12.8
INCLUDE_DIR = '${inc}'
LIB_DIR = '${lib}'
STUBEOF
  done
  echo "[c3_compat] Created commaai/dependencies stubs in /data/pip_packages"
fi

# 6. PATH + PYTHONPATH: make venv tools and packages visible to scons
#    scons at /usr/bin/scons uses #!/usr/bin/python3 which can't see venv packages.
#    scons also shells out to cythonize, which lives in /usr/local/venv/bin/.
#    DBC generator imports 'opendbc' — needs opendbc_repo on PYTHONPATH.
#    capnpc: cereal/SConscript runs a bare `capnpc`; on 0.11.2 the C++ build links
#    the comma-deps-capnproto wheel, whose capnpc must generate the headers (the
#    AGNOS 12.8 system capnpc is another version -> "version mismatch"). Its
#    bin dir goes first; absent on 0.11.1, harmless there.
#    Guard: the launch script itself mentions c3_compat (it sources this file),
#    so key on the PATH line this step writes.
LAUNCH_FILE="$OPENPILOT_DIR/launch_chffrplus.sh"
if [ -f "$LAUNCH_FILE" ] && ! grep -q 'export PATH=".*/usr/local/venv/bin' "$LAUNCH_FILE"; then
  python3 - "$LAUNCH_FILE" "$VENV_SITE" << 'PYEOF'
import sys
path, venv_site = sys.argv[1], sys.argv[2]
with open(path) as f:
    content = f.read()
# Add venv/bin (cythonize, scons tools) and the capnproto wheel's bin (capnpc) to PATH
content = content.replace(
    'export PYTHONPATH="$PWD"',
    '# c3_compat: venv bin for cythonize, capnproto wheel bin for capnpc, venv site-packages + opendbc_repo for scons\n'
    f'export PATH="{venv_site}/capnproto/install/bin:/usr/local/venv/bin:$PATH"\n'
    f'export PYTHONPATH="$PWD:$PWD/opendbc_repo:{venv_site}"'
)
with open(path, 'w') as f:
    f.write(content)
PYEOF
  echo "[c3_compat] Patched launch_chffrplus.sh PATH + PYTHONPATH"
fi

# 7. launch_env.sh: remove Wayland env vars (DRM backend uses /dev/dri/card0 directly)
#    Guard: the file itself mentions c3_compat since 0.11.2 (AGNOS policy comment),
#    so key on the Wayland export this step removes.
LAUNCH_ENV="$OPENPILOT_DIR/launch_env.sh"
if [ -f "$LAUNCH_ENV" ] && grep -q 'export WAYLAND_DISPLAY=' "$LAUNCH_ENV" && ! grep -q 'c3_compat: DRM backend' "$LAUNCH_ENV"; then
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

# 7b. launch_env.sh: raylib 6's comma backend gets its DRM fd from a broker
#     ("magic", /tmp/drmfd.sock, AGNOS 19) and otherwise only opens /dev/dri/card0
#     itself when NO_MASTER is set. No broker on AGNOS 12.8 -> export it for the
#     whole launch tree (manager -> ui). Harmless on 0.11.1 (raylib 5.5 ignores it).
if [ -f "$LAUNCH_ENV" ] && ! grep -q 'export NO_MASTER=' "$LAUNCH_ENV"; then
  printf '\n# c3_compat: no DRM fd broker (magic) on AGNOS 12.8 — raylib comma backend opens /dev/dri/card0 itself\nexport NO_MASTER=1\n' >> "$LAUNCH_ENV"
  echo "[c3_compat] Patched launch_env.sh: NO_MASTER=1 for the raylib comma backend"
fi

# 8. Panda: restore STM32F4 (Dos board) support for C3 internal panda
#    v0.10.3 dropped F4 support — only H7 (red panda, tres, cuatro) remains
#    C3 has HW_TYPE_DOS (0x06) with STM32F4 MCU
PANDA_INIT="$OPENPILOT_DIR/panda/python/__init__.py"
PANDA_CONST="$OPENPILOT_DIR/panda/python/constants.py"
if [ -f "$PANDA_INIT" ] && ! grep -q "HW_TYPE_DOS = b'" "$PANDA_INIT"; then
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

# Add HW_TYPE_DOS after HW_TYPE_BODY (HW_TYPE_BLACK removed in panda v0.11.0)
if "HW_TYPE_DOS = b'" not in init:
    init = init.replace(
        "HW_TYPE_BODY = b'\\xb1'\n",
        "HW_TYPE_BODY = b'\\xb1'\n  HW_TYPE_DOS = b'\\x06'  # C3 internal panda (STM32F4)\n"
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

# Patch or add get_mcu_type to handle F4
# Upstream v0.11.0 removed get_mcu_type (dropped F4) — add it back if missing
if 'def get_mcu_type' in init:
    init = init.replace(
        '  def get_mcu_type(self) -> McuType:\n    hw_type = self.get_type()\n    if hw_type in Panda.H7_DEVICES:\n      return McuType.H7\n    raise ValueError(f"unknown HW type: {hw_type}")',
        '  def get_mcu_type(self) -> McuType:\n    hw_type = self.get_type()\n    if hw_type in Panda.H7_DEVICES:\n      return McuType.H7\n    if hw_type in Panda.F4_DEVICES:\n      return McuType.F4\n    raise ValueError(f"unknown HW type: {hw_type}")'
    )
else:
    # Method doesn't exist — inject after SUPPORTED_DEVICES
    init = init.replace(
        '  SUPPORTED_DEVICES = H7_DEVICES + F4_DEVICES',
        '  SUPPORTED_DEVICES = H7_DEVICES + F4_DEVICES\n\n'
        '  def get_mcu_type(self):\n'
        '    from panda.python.constants import McuType\n'
        '    hw_type = self.get_type()\n'
        '    if hw_type in Panda.H7_DEVICES:\n'
        '      return McuType.H7\n'
        '    if hw_type in Panda.F4_DEVICES:\n'
        '      return McuType.F4\n'
        '    raise ValueError(f"unknown HW type: {hw_type}")'
    )

with open(init_path, 'w') as f:
    f.write(init)
PYEOF
  echo "[c3_compat] Patched panda library with STM32F4 (Dos board) support"
fi

# 8a. Panda: health() struct padding for F4 panda
#     F4 firmware v16 sends 58-byte health packets; library v18 expects 59.
#     Pad with zero bytes so unpack() succeeds (new field defaults to 0).
if [ -f "$PANDA_INIT" ] && ! grep -q 'c3_compat: F4 firmware may send fewer bytes' "$PANDA_INIT"; then
  python3 - "$PANDA_INIT" << 'PYEOF'
import sys
path = sys.argv[1]
with open(path) as f:
    content = f.read()
old = ('    dat = self._handle.controlRead(Panda.REQUEST_IN, 0xd2, 0, 0, self.HEALTH_STRUCT.size)\n'
       '    a = self.HEALTH_STRUCT.unpack(dat)')
new = ('    dat = self._handle.controlRead(Panda.REQUEST_IN, 0xd2, 0, 0, self.HEALTH_STRUCT.size)\n'
       '    # c3_compat: F4 firmware may send fewer bytes (older struct layout), pad to match\n'
       '    if len(dat) < self.HEALTH_STRUCT.size:\n'
       '      dat = dat + bytes(self.HEALTH_STRUCT.size - len(dat))\n'
       '    a = self.HEALTH_STRUCT.unpack(dat)')
if old in content:
    content = content.replace(old, new, 1)
    with open(path, 'w') as f:
        f.write(content)
PYEOF
  echo "[c3_compat] Patched panda health() to pad F4 short packets"
fi

# 8b. Panda: skip packet version checks for F4 panda
#     F4 firmware is v16 but the v0.11.0 library expects v18 for health.
#     Directly inject the final version with get_mcu_type() call + try/except.
if [ -f "$PANDA_INIT" ] && ! grep -q 'c3_compat.*version' "$PANDA_INIT"; then
  python3 - "$PANDA_INIT" << 'PYEOF'
import sys
path = sys.argv[1]
with open(path) as f:
    content = f.read()

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
    # c3_compat: skip version check for F4 panda (firmware v16, library v18)
    from panda.python.constants import McuType
    try:
      if self.get_mcu_type() == McuType.F4:
        return fn(self, *args, **kwargs)
    except Exception:
      pass
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
#    Two layouts: <= 0.11.1 (flash_panda(serial), first_run reset, pandad <serial>)
#    below, and 0.11.2 (panda_ready()/flash_panda() via DFU, bare ./pandad) in 9b.
PANDAD_FILE="$OPENPILOT_DIR/openpilot/selfdrive/pandad/pandad.py"
if [ -f "$PANDAD_FILE" ] && ! grep -q 'c3_compat' "$PANDAD_FILE" && ! grep -q 'def panda_ready' "$PANDAD_FILE"; then
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
    panda._mcu_type = McuType.F4  # c3_compat: cache for ensure_version skip
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
old_reset = '''      if first_run:
        # reset panda to ensure we're in a good state
        cloudlog.info(f"Resetting panda {panda.get_usb_serial()}")
        panda.reset(reconnect=True)'''
new_reset = '''      if first_run:
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
    process = subprocess.Popen(["./pandad", panda_serial], cwd=os.path.join(BASEDIR, "openpilot/selfdrive/pandad"))
    process.wait()'''
new_launch = '''    first_run = False

    # c3_compat: give kernel time to release USB device so native pandad can claim it
    time.sleep(2)

    # run pandad with all connected serials as arguments
    os.environ['MANAGER_DAEMON'] = 'pandad'
    # c3_compat: skip C++ firmware check for F4 panda
    os.environ["BOARDD_SKIP_FW_CHECK"] = "1"
    process = subprocess.Popen(["./pandad", panda_serial], cwd=os.path.join(BASEDIR, "openpilot/selfdrive/pandad"))
    process.wait()

    # c3_compat: crash backoff for pandad crashes
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

# 9b. Pandad (openpilot 0.11.2 layout). Stock flow per pass: GPIO-reset the panda,
#     panda_ready() (1 s wait, bootstub check, up_to_date() against the H7 image),
#     and on failure flash_panda() = BOOT0 -> DFU -> recover + flash the H7 image.
#     On the comma three every step is wrong for the F4: the USB re-enumeration
#     after a reset outruns the 1 s wait, up_to_date() can never match, and the
#     DFU path would push H7 images at an STM32F4 (the BMW plugin owns that
#     firmware). On tici: no reset, F4 passes panda_ready() once connected,
#     never flash — just retry; then the USB settle delay, BOARDD_SKIP_FW_CHECK
#     for the native check and the crash backoff, as before.
if [ -f "$PANDAD_FILE" ] && grep -q 'def panda_ready' "$PANDAD_FILE" && ! grep -q 'c3_compat' "$PANDAD_FILE"; then
  python3 - "$PANDAD_FILE" << 'PYEOF'
import sys
path = sys.argv[1]
with open(path) as f:
    content = f.read()
n = 0

old = 'import os\nimport signal\nimport subprocess\n'
new = 'import os\nimport signal\nimport subprocess\nimport time  # c3_compat\n'
if old in content:
    content = content.replace(old, new, 1); n += 1

old_ready = """      if p.bootstub:
        cloudlog.warning(f"panda stuck in bootstub, version {p.get_version()}")
        return False
      if not p.up_to_date():"""
new_ready = """      if p.bootstub:
        cloudlog.warning(f"panda stuck in bootstub, version {p.get_version()}")
        return False
      # c3_compat: F4 (comma three) panda — the shipped image is H7-only, the BMW plugin owns the F4 firmware
      from panda.python.constants import McuType
      if p.get_mcu_type() == McuType.F4:
        cloudlog.warning("c3_compat: F4 panda detected, skipping firmware check")
        return True
      if not p.up_to_date():"""
if old_ready in content:
    content = content.replace(old_ready, new_ready, 1); n += 1

old_loop = """      HARDWARE.reset_internal_panda()
      if not panda_ready():
        flash_panda()
        assert panda_ready(), "panda not ready after flashing"
"""
new_loop = """      # c3_compat: comma three (F4 panda) — no GPIO reset each pass (USB re-enumeration
      # outruns panda_ready's wait) and never DFU-flash the H7 image onto it: retry instead
      if HARDWARE.get_device_type() == "tici":
        if not panda_ready():
          cloudlog.warning("c3_compat: panda not ready, retrying")
          time.sleep(1)
          continue
      else:
        HARDWARE.reset_internal_panda()
        if not panda_ready():
          flash_panda()
          assert panda_ready(), "panda not ready after flashing"
"""
if old_loop in content:
    content = content.replace(old_loop, new_loop, 1); n += 1

old_run = """      # run real pandad
      os.environ['MANAGER_DAEMON'] = 'pandad'
      process = subprocess.Popen(["./pandad"], cwd=os.path.join(BASEDIR, "openpilot/selfdrive/pandad"))
      process.wait()
"""
new_run = """      # run real pandad
      os.environ['MANAGER_DAEMON'] = 'pandad'
      # c3_compat: give the kernel time to release the USB device so native pandad can
      # claim it, and skip its firmware-signature check (the F4 image is the BMW plugin's)
      time.sleep(2)
      os.environ["BOARDD_SKIP_FW_CHECK"] = "1"
      process = subprocess.Popen(["./pandad"], cwd=os.path.join(BASEDIR, "openpilot/selfdrive/pandad"))
      process.wait()
      # c3_compat: crash backoff (a crash loop -> heartbeat watchdog -> SOM reset -> red LED hang)
      if process.returncode != 0:
        main._crash_count = getattr(main, '_crash_count', 0) + 1
        backoff = min(60, 5 * main._crash_count)
        cloudlog.warning("c3_compat: native pandad exited %d, backing off %ds (crash #%d)",
                         process.returncode, backoff, main._crash_count)
        time.sleep(backoff)
      else:
        main._crash_count = 0
"""
if old_run in content:
    content = content.replace(old_run, new_run, 1); n += 1

if n == 4:
    with open(path, 'w') as f:
        f.write(content)
    print("OK")
else:
    print(f"SKIP: only {n}/4 anchors found, pandad.py left untouched")
PYEOF
  echo "[c3_compat] Patched pandad.py (0.11.2 layout) for the F4 panda"
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

# 0.11.2 panda: the handle opens the device node directly
old2 = '''      if speed not in SPI_DEVICES:
        file = open(DEV_PATH, 'r+b', buffering=0)'''
new2 = '''      if speed not in SPI_DEVICES:
        try:
          file = open(DEV_PATH, 'r+b', buffering=0)
        except PermissionError as e:
          # c3_compat: SPI disabled for F4 panda USB-only mode
          raise PandaSpiUnavailable("SPI device permission denied (F4 USB-only mode)") from e'''

if old in content:
    content = content.replace(old, new)
elif old2 in content:
    content = content.replace(old2, new2)
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

# 14. Hardwared: guard eSIM prime block against AT command failure
#     AGNOS 12.8 modem fails AT+CCHO (ISD-R open) with Unknown error.
#     configure_modem() crashes hardwared which blocks onroad/offroad transitions.
#     Wrap the get_sim_lpa() call in a try/except so modem init succeeds.
#    0.11.2: file moved to common/hardware/comma/; the eSIM prime block is gone from it
#    (eSIM handling moved to the UI), so this section is expected to SKIP there.
HW_FILE="$OPENPILOT_DIR/openpilot/common/hardware/comma/hardware.py"
if [ -f "$HW_FILE" ] && ! grep -q 'c3_compat.*lpa' "$HW_FILE"; then
  python3 - "$HW_FILE" << 'PYEOF'
import sys
path = sys.argv[1]
with open(path) as f:
    content = f.read()

old = '''    # eSIM prime
    dest = "/etc/NetworkManager/system-connections/esim.nmconnection"
    if self.get_sim_lpa().is_comma_profile(sim_id) and not os.path.exists(dest):'''

new = '''    # eSIM prime
    dest = "/etc/NetworkManager/system-connections/esim.nmconnection"
    try:
      _esim_check = self.get_sim_lpa().is_comma_profile(sim_id)  # c3_compat: AT+CCHO fails on AGNOS 12.8
    except Exception:
      _esim_check = False
    if _esim_check and not os.path.exists(dest):'''

if old in content:
    content = content.replace(old, new, 1)
    with open(path, 'w') as f:
        f.write(content)
    print("[c3_compat] Patched hardware.py: guarded eSIM prime against AT command failure")
else:
    print("[c3_compat] hardware.py: no eSIM prime block (openpilot 0.11.2), nothing to patch")
PYEOF
fi

# ---------------------------------------------------------------------------
# 15-17. comma three hardware support that is compiled, not scripted. Upstream
#     removed it with the comma three; catpilot carried it in-repo until the
#     openpilot 0.11.2 rebase. The sources ship with this plugin under src/ and
#     are copied in before scons runs; the hooks below are text patches on the
#     stock files, guarded by their c3_compat markers like every other section.

# 15. camerad: AR0231 image sensor (the comma three's cameras). 0.11.2 ships only
#     the comma 3X / four sensors (OS04C10, OX03C10); without this camerad finds
#     no sensor. SensorInfo is unchanged since the driver last built upstream.
CAMERAD="$OPENPILOT_DIR/openpilot/system/camerad"
if [ -d "$CAMERAD/sensors" ] && [ -f "$C3_SRC/camerad/ar0231.cc" ]; then
  for f in ar0231.cc ar0231_registers.h; do
    if ! cmp -s "$C3_SRC/camerad/$f" "$CAMERAD/sensors/$f"; then
      cp "$C3_SRC/camerad/$f" "$CAMERAD/sensors/$f"
      echo "[c3_compat] Installed camerad/sensors/$f"
    fi
  done
  python3 - "$CAMERAD" "$C3_SRC" << 'PYEOF'
import sys, os
d = sys.argv[1]
src = sys.argv[2]

def patch(rel, pairs, marker='c3_compat'):
    path = os.path.join(d, rel)
    with open(path) as f:
        s = f.read()
    if marker in s:
        return
    n = 0
    for old, new in pairs:
        if old in s:
            s = s.replace(old, new, 1); n += 1
    if n == len(pairs):
        with open(path, 'w') as f:
            f.write(s)
        print(f"[c3_compat] Patched camerad/{rel} for AR0231")
    else:
        print(f"[c3_compat] WARNING camerad/{rel}: {n}/{len(pairs)} anchors found, left untouched")

patch('SConscript', [(
    "'cameras/cdm.cc', 'sensors/ox03c10.cc', 'sensors/os04c10.cc'])",
    "'cameras/cdm.cc', 'sensors/ar0231.cc',  # c3_compat: comma three sensor\n"
    "                           'sensors/ox03c10.cc', 'sensors/os04c10.cc'])",
)])
patch('sensors/sensor.h', [(
    '#include "system/camerad/sensors/os04c10_registers.h"\n',
    '#include "system/camerad/sensors/os04c10_registers.h"\n'
    '#include "system/camerad/sensors/ar0231_registers.h"  // c3_compat: comma three sensor\n',
), (
    'class OX03C10 : public SensorInfo {\n',
    'class AR0231 : public SensorInfo {  // c3_compat: comma three sensor\n'
    'public:\n'
    '  AR0231();\n'
    '  std::vector<i2c_random_wr_payload> getExposureRegisters(int exposure_time, int new_exp_g, bool dc_gain_enabled) const override;\n'
    '  float getExposureScore(float desired_ev, int exp_t, int exp_g_idx, float exp_gain, int gain_idx) const override;\n'
    '  int getSlaveAddress(int port) const override;\n'
    '};\n'
    '\n'
    'class OX03C10 : public SensorInfo {\n',
)])
# bps_blobs.h: 0.11.2 ships the AR0231 slot of the auto-generated BPS IO-config
# table (bps_cfg[1], indexed by cereal ImageSensor) as an empty placeholder, so the
# ICP firmware gets an all-zero IO config at the cabin camera's acquire and asserts
# (A5 watchdog, "SFR: Assertion Failure ... hlssr.c"). Restore 0.11.1's entry; the
# OX03C10 entry is byte-identical across the two releases, so the format is the same.
blobs = os.path.join(d, 'cameras', 'bps_blobs.h')
with open(blobs) as f:
    b = f.read()
if 'c3_compat: AR0231 BPS IO config' not in b:
    marker = '{ /* placeholder */ },\n{ /* placeholder */ },\n'
    if b.count(marker) == 1:
        with open(os.path.join(src, 'camerad', 'bps_cfg_ar0231.inc')) as f:
            entry = f.read()
        b = b.replace(marker, '{ /* placeholder */ },\n' + entry, 1)
        with open(blobs, 'w') as f:
            f.write(b)
        print("[c3_compat] Patched camerad/cameras/bps_blobs.h: AR0231 BPS IO config restored")
    else:
        print("[c3_compat] WARNING camerad/cameras/bps_blobs.h: placeholder anchor not found, left untouched")

patch('cameras/spectra.cc', [(
    '  if (!init_sensor_lambda(new OS04C10) &&\n      !init_sensor_lambda(new OX03C10)) {',
    '  if (!init_sensor_lambda(new AR0231) &&   // c3_compat: comma three sensor first\n'
    '      !init_sensor_lambda(new OS04C10) &&\n      !init_sensor_lambda(new OX03C10)) {',
)])
patch('cameras/camera_common.cc', [(
    '  vipc_server->create_buffers_with_sizes(stream_type, VIPC_BUFFER_COUNT, out_img_width, out_img_height, cam->yuv_size, cam->stride, cam->uv_offset);',
    '  // c3_compat: VENUS_BUFFER_SIZE is too small for the HW encoder on wide frames (>1344px); use the encoder-aligned size\n'
    '  size_t nv12_size = (out_img_width <= 1344 ? 2900 : 2346) * cam->stride;\n'
    '  vipc_server->create_buffers_with_sizes(stream_type, VIPC_BUFFER_COUNT, out_img_width, out_img_height, nv12_size, cam->stride, cam->uv_offset);',
)])
patch('cameras/camera_qcom2.cc', [(
    '  float target_grey_fraction = 0.125;',
    '  float target_grey_fraction = 0.3;  // c3_compat: AR0231 exposure target',
), (
    '    std::make_pair((Rect){96, 400, 1734, 524}, 567.0),  // wide',
    '    std::make_pair((Rect){96, 250, 1734, 524}, 567.0),  // wide (c3_compat: AR0231 framing)',
)])
PYEOF
fi

# 16. pandad: USB transport. 0.11.2's native pandad is SPI-only (PandaSpiHandle);
#     the comma three's F4 panda is USB-only (and §10 blocks SPI). Adds a
#     PandaCommsHandle base, a libusb PandaUsbHandle (src/pandad/panda_usb.cc),
#     USB-first connect with SPI fallback, USB+SPI discovery, and -lusb-1.0.
PANDAD_DIR="$OPENPILOT_DIR/openpilot/selfdrive/pandad"
if [ -f "$PANDAD_DIR/panda_comms.h" ] && [ -f "$C3_SRC/pandad/panda_usb.cc" ]; then
  if ! cmp -s "$C3_SRC/pandad/panda_usb.cc" "$PANDAD_DIR/panda_usb.cc"; then
    cp "$C3_SRC/pandad/panda_usb.cc" "$PANDAD_DIR/panda_usb.cc"
    echo "[c3_compat] Installed pandad/panda_usb.cc"
  fi
  python3 - "$PANDAD_DIR" << 'PYEOF'
import sys, os
d = sys.argv[1]

def patch(rel, pairs, marker='c3_compat'):
    path = os.path.join(d, rel)
    with open(path) as f:
        s = f.read()
    if marker in s:
        return
    n = 0
    for old, new in pairs:
        if old in s:
            s = s.replace(old, new, 1); n += 1
    if n == len(pairs):
        with open(path, 'w') as f:
            f.write(s)
        print(f"[c3_compat] Patched pandad/{rel} for USB")
    else:
        print(f"[c3_compat] WARNING pandad/{rel}: {n}/{len(pairs)} anchors found, left untouched")

patch('panda_comms.h', [(
    '#include <vector>\n',
    '#include <vector>\n\n#include <libusb-1.0/libusb.h>  // c3_compat\n',
), (
    'class PandaSpiHandle {\n'
    'public:\n'
    '  std::string hw_serial;\n'
    '  std::atomic<bool> connected = true;\n'
    '  std::atomic<bool> comms_healthy = true;\n'
    '\n'
    '  PandaSpiHandle(std::string serial);\n'
    '  ~PandaSpiHandle();\n'
    '\n'
    '  int control_write(uint8_t request, uint16_t param1, uint16_t param2, unsigned int timeout=TIMEOUT);\n'
    '  int control_read(uint8_t request, uint16_t param1, uint16_t param2, unsigned char *data, uint16_t length, unsigned int timeout=TIMEOUT);\n'
    '  int bulk_write(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout=TIMEOUT);\n'
    '  int bulk_read(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout=TIMEOUT);\n'
    '  void cleanup();\n'
    '\n'
    '  static std::vector<std::string> list();\n',
    '// c3_compat: transport base so Panda can hold USB (comma three F4) or SPI\n'
    'class PandaCommsHandle {\n'
    'public:\n'
    '  PandaCommsHandle() = default;\n'
    '  virtual ~PandaCommsHandle() = default;\n'
    '\n'
    '  std::string hw_serial;\n'
    '  std::atomic<bool> connected = true;\n'
    '  std::atomic<bool> comms_healthy = true;\n'
    '\n'
    '  virtual int control_write(uint8_t request, uint16_t param1, uint16_t param2, unsigned int timeout=TIMEOUT) = 0;\n'
    '  virtual int control_read(uint8_t request, uint16_t param1, uint16_t param2, unsigned char *data, uint16_t length, unsigned int timeout=TIMEOUT) = 0;\n'
    '  virtual int bulk_write(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout=TIMEOUT) = 0;\n'
    '  virtual int bulk_read(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout=TIMEOUT) = 0;\n'
    '  virtual void cleanup() = 0;\n'
    '};\n'
    '\n'
    '// c3_compat: libusb transport (src/pandad/panda_usb.cc)\n'
    'class PandaUsbHandle : public PandaCommsHandle {\n'
    'public:\n'
    '  PandaUsbHandle(std::string serial);\n'
    '  ~PandaUsbHandle();\n'
    '  int control_write(uint8_t request, uint16_t param1, uint16_t param2, unsigned int timeout=TIMEOUT) override;\n'
    '  int control_read(uint8_t request, uint16_t param1, uint16_t param2, unsigned char *data, uint16_t length, unsigned int timeout=TIMEOUT) override;\n'
    '  int bulk_write(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout=TIMEOUT) override;\n'
    '  int bulk_read(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout=TIMEOUT) override;\n'
    '  void cleanup() override;\n'
    '\n'
    '  static std::vector<std::string> list();\n'
    '\n'
    'private:\n'
    '  libusb_context *ctx = NULL;\n'
    '  libusb_device_handle *dev_handle = NULL;\n'
    '  std::recursive_mutex hw_lock;\n'
    '  void handle_usb_issue(int err, const char func[]);\n'
    '};\n'
    '\n'
    'class PandaSpiHandle : public PandaCommsHandle {\n'
    'public:\n'
    '  PandaSpiHandle(std::string serial);\n'
    '  ~PandaSpiHandle();\n'
    '\n'
    '  int control_write(uint8_t request, uint16_t param1, uint16_t param2, unsigned int timeout=TIMEOUT) override;\n'
    '  int control_read(uint8_t request, uint16_t param1, uint16_t param2, unsigned char *data, uint16_t length, unsigned int timeout=TIMEOUT) override;\n'
    '  int bulk_write(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout=TIMEOUT) override;\n'
    '  int bulk_read(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout=TIMEOUT) override;\n'
    '  void cleanup() override;\n'
    '\n'
    '  static std::vector<std::string> list();\n',
)])
patch('panda.h', [(
    '  std::unique_ptr<PandaSpiHandle> handle;',
    '  std::unique_ptr<PandaCommsHandle> handle;  // c3_compat: USB (comma three) or SPI',
)])
patch('panda.cc', [(
    '#include <vector>\n',
    '#include <vector>\n#include <algorithm>  // c3_compat\n',
), (
    '  handle = std::make_unique<PandaSpiHandle>(serial);\n'
    '  LOGW("connected to %s over SPI", serial.c_str());\n',
    '  // c3_compat: the comma three\'s F4 panda is USB-only — try USB first, then SPI\n'
    '  try {\n'
    '    handle = std::make_unique<PandaUsbHandle>(serial);\n'
    '    LOGW("connected to %s over USB", serial.c_str());\n'
    '  } catch (std::exception &e) {\n'
    '    handle = std::make_unique<PandaSpiHandle>(serial);\n'
    '    LOGW("connected to %s over SPI", serial.c_str());\n'
    '  }\n',
), (
    '  return PandaSpiHandle::list();\n',
    '  // c3_compat: USB pandas first, then SPI\n'
    '  std::vector<std::string> serials = PandaUsbHandle::list();\n'
    '  for (const auto &s : PandaSpiHandle::list()) {\n'
    '    if (std::find(serials.begin(), serials.end(), s) == serials.end()) {\n'
    '      serials.push_back(s);\n'
    '    }\n'
    '  }\n'
    '  return serials;\n',
)])
patch('SConscript', [(
    "  libs = [common, messaging, 'pthread']\n"
    "  panda = env.Library('panda', ['panda.cc', 'spi.cc'])",
    "  libs = ['usb-1.0', common, messaging, 'pthread']  # c3_compat: USB transport\n"
    "  panda = env.Library('panda', ['panda.cc', 'panda_usb.cc', 'spi.cc'])",
)])
PYEOF
  # SConstruct links only libraries that a vendored package provides: libusb is the
  # comma-deps-libusb wheel (uv.lock), so it joins the package list (include + lib dirs)
  SCONSTRUCT="$OPENPILOT_DIR/SConstruct"
  if [ -f "$SCONSTRUCT" ] && ! grep -q "c3_compat: libusb" "$SCONSTRUCT"; then
    python3 - "$SCONSTRUCT" << 'PYEOF'
import sys
path = sys.argv[1]
with open(path) as f:
    s = f.read()
old = "pkg_names = ['acados', 'capnproto', 'ffmpeg', 'json11', 'ncurses', 'zeromq', 'zstd']\n"
new = "pkg_names = ['acados', 'capnproto', 'ffmpeg', 'json11', 'ncurses', 'zeromq', 'zstd', 'libusb']  # c3_compat: libusb for pandad USB\n"
if old in s:
    with open(path, 'w') as f:
        f.write(s.replace(old, new, 1))
    print("[c3_compat] Patched SConstruct: libusb vendored package")
else:
    print("[c3_compat] WARNING SConstruct: pkg_names anchor not found, left untouched")
PYEOF
  fi
fi

# 17. loggerd: wait for the Venus hardware encoder. On AGNOS 12.8 the VIDC
#     device node exists before its firmware is loaded; the stock constructor
#     asserts on the first failed ioctl and encoderd crash-loops at boot.
V4L_ENC="$OPENPILOT_DIR/openpilot/system/loggerd/encoder/v4l_encoder.cc"
if [ -f "$V4L_ENC" ] && ! grep -q 'c3_compat' "$V4L_ENC"; then
  python3 - "$V4L_ENC" << 'PYEOF'
import sys
path = sys.argv[1]
with open(path) as f:
    s = f.read()
old = """  fd = HANDLE_EINTR(open("/dev/v4l/by-path/platform-aa00000.qcom_vidc-video-index1", O_RDWR|O_NONBLOCK));
  assert(fd >= 0);

  struct v4l2_capability cap;
  util::safe_ioctl(fd, VIDIOC_QUERYCAP, &cap, "VIDIOC_QUERYCAP failed");
  LOGD("opened encoder device %s %s = %d", cap.driver, cap.card, fd);
  assert(strcmp((const char *)cap.driver, "msm_vidc_driver") == 0);
  assert(strcmp((const char *)cap.card, "msm_vidc_venc") == 0);
"""
new = """  // c3_compat: wait for the Venus firmware — on AGNOS 12.8 the VIDC node exists before the
  // firmware is loaded and early ioctls fail; retry with backoff (~15 s) instead of asserting
  const char *dev_path = "/dev/v4l/by-path/platform-aa00000.qcom_vidc-video-index1";
  struct v4l2_capability cap;
  for (int attempt = 0;; ++attempt) {
    fd = HANDLE_EINTR(open(dev_path, O_RDWR|O_NONBLOCK));
    if (fd >= 0) {
      if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0 &&
          strcmp((const char *)cap.driver, "msm_vidc_driver") == 0 &&
          strcmp((const char *)cap.card, "msm_vidc_venc") == 0) {
        break;  // Venus ready
      }
      close(fd);
      fd = -1;
    }
    if (attempt >= 30) {
      LOGE("Venus encoder not ready after %d attempts, giving up", attempt);
      assert(false);
    }
    LOGW("Venus encoder not ready (attempt %d), retrying in 500ms...", attempt);
    usleep(500000);
  }
  LOGD("opened encoder device %s %s = %d", cap.driver, cap.card, fd);
"""
inc_old = '#include <cassert>\n'
inc_new = '#include <cassert>\n#include <unistd.h>  // c3_compat: close/usleep\n'
if old in s and inc_old in s:
    s = s.replace(old, new, 1).replace(inc_old, inc_new, 1)
    with open(path, 'w') as f:
        f.write(s)
    print("[c3_compat] Patched loggerd v4l_encoder.cc: wait for the Venus encoder")
else:
    print("[c3_compat] WARNING loggerd v4l_encoder.cc: anchors not found, left untouched")
PYEOF
fi

# 18. Kernel UAPI headers for camerad / loggerd. openpilot 0.11.2 includes the
#     Qualcomm camera and video headers from the system (#37993): media/cam_*.h,
#     media/msm_camsensor_sdk.h, media/msm_media_info.h, and the VIDC control
#     IDs in linux/v4l2-controls.h. AGNOS 16+ ships them in /usr/include; AGNOS
#     12.8 has only the stock Ubuntu v4l2-controls.h and no media/ at all. These
#     are the comma three kernel's own headers (the 0.11.1 third_party/linux/
#     include set camerad built against there), installed under
#     /usr/local/include, which the compiler searches before /usr/include, so
#     the qcom v4l2-controls.h shadows the stock one and no build-system change
#     is needed. Verified: the 0.11.2 camerad/loggerd sources pass clang
#     -fsyntax-only against this set.
HDR_SRC="$C3_SRC/linux_include"
HDR_DST="/usr/local/include"
if [ -d "$HDR_SRC" ]; then
  _hdr_changed=0
  for sub in media linux; do
    [ -d "$HDR_SRC/$sub" ] || continue
    for f in "$HDR_SRC/$sub"/*.h; do
      cmp -s "$f" "$HDR_DST/$sub/$(basename "$f")" || _hdr_changed=1
    done
  done
  if [ $_hdr_changed -eq 1 ]; then
    sudo mount -o remount,rw / 2>/dev/null
    for sub in media linux; do
      [ -d "$HDR_SRC/$sub" ] || continue
      sudo mkdir -p "$HDR_DST/$sub"
      sudo cp "$HDR_SRC/$sub"/*.h "$HDR_DST/$sub/"
    done
    sudo mount -o remount,ro / 2>/dev/null || true
    echo "[c3_compat] Installed kernel UAPI headers (media/*.h, linux/{ion,msm_ion,v4l2-controls}.h) to $HDR_DST"
  fi
fi

# 19. Native device type: openpilot 0.11.2 dropped "tici" from HardwareComma's
#     device_map (common/hardware/comma/hardware.h), so every C++ daemon that
#     asks for the device type (pandad, loggerd, camerad via set_ir_power) hits
#     assert(it != device_map.end()) on the comma three. Restore the 0.11.1
#     entries: tici in the map, IR LEDs untouched on tici (as on tizi), and the
#     livestream encoder dimensions loggerd.h keys on the device (tici shares
#     the 1928x1208 road camera with tizi -> 1152x720).
HW_H="$OPENPILOT_DIR/openpilot/common/hardware/comma/hardware.h"
LOGGERD_H="$OPENPILOT_DIR/openpilot/system/loggerd/loggerd.h"
if [ -f "$HW_H" ] && ! grep -q '"tici"' "$HW_H"; then
  python3 - "$HW_H" "$LOGGERD_H" << 'PYEOF'
import sys
hw_path, lg_path = sys.argv[1], sys.argv[2]
with open(hw_path) as f:
    s = f.read()
n = 0
old = '      {"tizi", cereal::InitData::DeviceType::TIZI},\n'
new = '      {"tici", cereal::InitData::DeviceType::TICI},  // c3_compat: comma three\n      {"tizi", cereal::InitData::DeviceType::TIZI},\n'
if old in s:
    s = s.replace(old, new, 1); n += 1
old = '    if (device == cereal::InitData::DeviceType::TIZI) {\n      return;\n'
new = '    if (device == cereal::InitData::DeviceType::TICI ||  // c3_compat\n        device == cereal::InitData::DeviceType::TIZI) {\n      return;\n'
if old in s:
    s = s.replace(old, new, 1); n += 1
if n == 2:
    with open(hw_path, 'w') as f:
        f.write(s)
    print("[c3_compat] Patched hardware.h: tici in device_map")
else:
    print(f"[c3_compat] WARNING hardware.h: {n}/2 anchors found, left untouched")
try:
    with open(lg_path) as f:
        s = f.read()
except FileNotFoundError:
    sys.exit(0)
m = 0
for old, new in (
    ('    case cereal::InitData::DeviceType::TIZI: return 1152;\n',
     '    case cereal::InitData::DeviceType::TICI:  // c3_compat\n    case cereal::InitData::DeviceType::TIZI: return 1152;\n'),
    ('    case cereal::InitData::DeviceType::TIZI:\n    case cereal::InitData::DeviceType::MICI: return 720;\n',
     '    case cereal::InitData::DeviceType::TICI:  // c3_compat\n    case cereal::InitData::DeviceType::TIZI:\n    case cereal::InitData::DeviceType::MICI: return 720;\n'),
):
    if old in s:
        s = s.replace(old, new, 1); m += 1
if m == 2:
    with open(lg_path, 'w') as f:
        f.write(s)
    print("[c3_compat] Patched loggerd.h: livestream size for tici")
else:
    print(f"[c3_compat] WARNING loggerd.h: {m}/2 anchors found, left untouched")
PYEOF
fi

echo "[c3_compat] Boot patches applied successfully"
