#!/usr/bin/env bash
# c3_compat boot patcher — runs before openpilot build/launch on AGNOS 12.8
# Sourced by launch_chffrplus.sh (inside launch()) to apply Comma 3 compatibility fixes
#
# Targets openpilot 0.11.2 and above (plugin.json min_openpilot); every anchor
# below is written against that tree.
# Patches are idempotent — safe to run multiple times.

# NOTE: Do NOT use 'set -e' here — this script is sourced by launch_chffrplus.sh,
# and set -e would propagate to the parent shell, killing the launch chain
# on any non-zero exit from build.py, manager.py, etc.
OPENPILOT_DIR="${1:-/data/openpilot}"
# this plugin's runtime directory (artifacts under raylib_drm/ and src/ are addressed from here)
C3_COMPAT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
C3_SRC="$C3_COMPAT_DIR/src"

echo "[c3_compat] Applying AGNOS 12.8 patches to $OPENPILOT_DIR"

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
#    Everything lives in /usr/local/venv/ — the single source of truth.
VENV_BIN="/usr/local/venv/bin"
VENV_SITE="/usr/local/venv/lib/python3.12/site-packages"
_venv_patched=0

# 2a. msgfmt stub: AGNOS 12.8 lacks gettext — scons needs it for .po → .mo
if [ ! -f "$VENV_BIN/msgfmt" ]; then
  sudo mount -o remount,rw / 2>/dev/null
  if sudo tee "$VENV_BIN/msgfmt" > /dev/null << 'STUB' && sudo chmod +x "$VENV_BIN/msgfmt"; then
#!/bin/sh
# Stub: c3_compat — produce valid empty .mo files (proper gettext binary format)
# Octal escapes: /bin/sh may be dash, whose printf has no \x and would write the text literally.
while [ $# -gt 0 ]; do
  case "$1" in -o) shift; OUTPUT="$1" ;; esac
  shift
done
[ -n "$OUTPUT" ] && printf '\336\022\004\225\000\000\000\000\000\000\000\000\034\000\000\000\034\000\000\000\000\000\000\000\034\000\000\000' > "$OUTPUT"
STUB
    echo "[c3_compat] Installed msgfmt stub to venv"
  else
    echo "[c3_compat] WARNING: msgfmt stub install failed"
  fi
  _venv_patched=1
fi

# 2b. venv_sync: ensure venv matches deployed branch's uv.lock BEFORE launch.
#     Compares each package in uv.lock against what's installed in the venv.
#     Installs anything missing or at wrong version. Fast path: if the uv.lock hash
#     and the venv fingerprint match data/.venv_synced_hash, skip entirely (<100ms).
#     This guarantees openpilot won't crash on import errors regardless of how
#     the code was deployed (COD update, manual git checkout, AGNOS reflash).
VENV_SYNC="/data/plugins-runtime/c3_compat/venv_sync.py"
if [ -f "$VENV_SYNC" ] && [ -f /data/openpilot/uv.lock ]; then
  PYTHONPATH="/data/plugins-runtime${PYTHONPATH:+:$PYTHONPATH}" /usr/local/venv/bin/python3 "$VENV_SYNC" --runtime-only 2>&1 | while IFS= read -r line; do
    echo "[c3_compat] $line"
  done
fi

# Re-seal root filesystem
if [ $_venv_patched -eq 1 ]; then
  sudo mount -o remount,ro / 2>/dev/null || true
fi

# 2d. raylib 6 "comma" backend on the comma three. The comma-deps-raylib wheel
#     (installed by 2b) links its backend against libEGL.so.1 / libGLESv2.so.2 /
#     libgbm.so.1, which on AGNOS 12.8 resolve to Mesa (GLVND); Mesa's msm driver
#     cannot drive the 4.9 downstream kernel, falls back to llvmpipe and segfaults
#     on the first frame. The comma three renders through Qualcomm's libEGL_adreno /
#     libGLESv2_adreno on Qualcomm's libgbm, whose GBM display offers the ABGR8888
#     visual the backend asks for. The shipped module is the wheel's own file with
#     three DT_NEEDED entries rewritten (patchelf --replace-needed; recipe in
#     DESIGN.md), keyed on the wheel's sha256 so a wheel bump surfaces as a
#     warning, never a swap.
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

# 6. PATH + PYTHONPATH: make venv tools and packages visible to scons
#    scons at /usr/bin/scons uses #!/usr/bin/python3 which can't see venv packages.
#    scons also shells out to cythonize, which lives in /usr/local/venv/bin/.
#    DBC generator imports 'opendbc' — needs opendbc_repo on PYTHONPATH.
#    capnpc: cereal/SConscript runs a bare `capnpc`; the C++ build links the
#    comma-deps-capnproto wheel, whose capnpc must generate the headers (the
#    AGNOS 12.8 system capnpc is another version -> "version mismatch"). Its
#    bin dir goes first.
#    A catpilot launcher that sets this up itself (its AGNOS < 16 block) already
#    has the PATH line, and this step skips; it is here for trees that lack it.
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
found = 'export PYTHONPATH="$PWD"' in content
content = content.replace(
    'export PYTHONPATH="$PWD"',
    '# c3_compat: venv bin for cythonize, capnproto wheel bin for capnpc, venv site-packages + opendbc_repo for scons\n'
    f'export PATH="{venv_site}/capnproto/install/bin:/usr/local/venv/bin:$PATH"\n'
    f'export PYTHONPATH="$PWD:$PWD/opendbc_repo:{venv_site}"'
)
with open(path, 'w') as f:
    f.write(content)
if found:
    print("[c3_compat] Patched launch_chffrplus.sh PATH + PYTHONPATH")
else:
    print("[c3_compat] WARNING launch_chffrplus.sh: 'export PYTHONPATH=\"$PWD\"' anchor not found, left untouched")
PYEOF
fi
# The edit above reaches the next boot only: this file is sourced from inside the
# launcher's launch(), which bash has already read. Put capnpc first on PATH for this
# boot's build too. PYTHONPATH can't be fixed here: the launcher assigns it after this file.
case ":$PATH:" in
  *":$VENV_SITE/capnproto/install/bin:"*) ;;
  *) export PATH="$VENV_SITE/capnproto/install/bin:$PATH" ;;
esac

# 7. launch_env.sh: remove Wayland env vars (DRM backend uses /dev/dri/card0 directly).
#    Only a launch_env.sh that still carries the Wayland block is touched.
#    Guard: the file itself mentions c3_compat (AGNOS policy comment), so key on
#    the Wayland export this step removes.
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
    print("[c3_compat] Patched launch_env.sh for DRM backend (removed Wayland config)")
else:
    print("[c3_compat] WARNING launch_env.sh: AGNOS < 16 Wayland block anchor not found, left untouched")
PYEOF
fi

# 7b. launch_env.sh: raylib 6's comma backend gets its DRM fd from a broker
#     ("magic", /tmp/drmfd.sock, AGNOS 19) and otherwise only opens /dev/dri/card0
#     itself when NO_MASTER is set. No broker on AGNOS 12.8 -> export it for the
#     whole launch tree (manager -> ui). Skipped when launch_env.sh exports it itself.
if [ -f "$LAUNCH_ENV" ] && ! grep -q 'export NO_MASTER=' "$LAUNCH_ENV"; then
  if printf '\n# c3_compat: no DRM fd broker (magic) on AGNOS 12.8 — raylib comma backend opens /dev/dri/card0 itself\nexport NO_MASTER=1\n' >> "$LAUNCH_ENV"; then
    echo "[c3_compat] Patched launch_env.sh: NO_MASTER=1 for the raylib comma backend"
  else
    echo "[c3_compat] WARNING launch_env.sh: could not append NO_MASTER=1"
  fi
fi
# The launcher sourced launch_env.sh before this file, so the edit above reaches the
# next boot only; export it into the running launch shell for this one.
export NO_MASTER=1

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
missing = []  # c3_compat: anchors not found (their pieces are left out)
total = 6
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
    if 'H7Config = McuConfig(' not in const:
        missing.append('constants.py F4Config')
    const = const.replace('H7Config = McuConfig(', f4_config + 'H7Config = McuConfig(')
    if 'class McuType(enum.Enum):\n  H7 = H7Config' not in const:
        missing.append('constants.py McuType.F4')
    const = const.replace(
        'class McuType(enum.Enum):\n  H7 = H7Config',
        'class McuType(enum.Enum):\n  F4 = F4Config\n  H7 = H7Config'
    )
    with open(const_path, 'w') as f:
        f.write(const)

# --- Patch __init__.py: add HW_TYPE_DOS, update get_mcu_type, devices ---
with open(init_path) as f:
    init = f.read()

# Add HW_TYPE_DOS after HW_TYPE_BODY
if "HW_TYPE_DOS = b'" not in init:
    if "HW_TYPE_BODY = b'\\xb1'\n" not in init:
        missing.append('HW_TYPE_DOS')
    init = init.replace(
        "HW_TYPE_BODY = b'\\xb1'\n",
        "HW_TYPE_BODY = b'\\xb1'\n  HW_TYPE_DOS = b'\\x06'  # C3 internal panda (STM32F4)\n"
    )

# Add F4 devices list and extend SUPPORTED_DEVICES
if 'H7_DEVICES = [HW_TYPE_RED_PANDA, HW_TYPE_TRES, HW_TYPE_CUATRO, HW_TYPE_BODY]\n  SUPPORTED_DEVICES = H7_DEVICES' not in init:
    missing.append('F4_DEVICES/SUPPORTED_DEVICES')
init = init.replace(
    'H7_DEVICES = [HW_TYPE_RED_PANDA, HW_TYPE_TRES, HW_TYPE_CUATRO, HW_TYPE_BODY]\n  SUPPORTED_DEVICES = H7_DEVICES',
    'H7_DEVICES = [HW_TYPE_RED_PANDA, HW_TYPE_TRES, HW_TYPE_CUATRO, HW_TYPE_BODY]\n  F4_DEVICES = [HW_TYPE_DOS]\n  SUPPORTED_DEVICES = H7_DEVICES + F4_DEVICES'
)

# Add DOS to INTERNAL_DEVICES
if 'INTERNAL_DEVICES = (HW_TYPE_TRES, HW_TYPE_CUATRO)' not in init:
    missing.append('INTERNAL_DEVICES')
init = init.replace(
    'INTERNAL_DEVICES = (HW_TYPE_TRES, HW_TYPE_CUATRO)',
    'INTERNAL_DEVICES = (HW_TYPE_TRES, HW_TYPE_CUATRO, HW_TYPE_DOS)'
)

# Add Panda.get_mcu_type() (upstream removed it together with the F4): inject after SUPPORTED_DEVICES
if '  SUPPORTED_DEVICES = H7_DEVICES + F4_DEVICES' not in init:
    missing.append('get_mcu_type')
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
if not missing:
    print("[c3_compat] Patched panda library with STM32F4 (Dos board) support")
else:
    print(f"[c3_compat] WARNING panda library F4 support: {total - len(missing)}/{total} anchors found;"
          f" not found, left untouched: {', '.join(missing)}")
PYEOF
fi

# 8b. Panda: skip packet version checks for F4 panda
#     The F4 firmware (health packet v16) is older than the library, so every
#     version-checked call would raise "packet version mismatch ... Reflash panda".
#     Skipping the check does not make the health packet parse: Panda.health() on
#     the F4 reads the v16 bytes with the library's layout, so the fields after
#     uptime are wrong and must not be relied on. Native pandad translates the
#     packet properly (16c); pandad.py's one use of health() is skipped in 9c.
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
    print("[c3_compat] Patched panda version check to skip for F4")
else:
    print("[c3_compat] WARNING panda ensure_version: anchor not found, left untouched")
PYEOF
fi

# 9b. pandad.py. Stock flow per pass: GPIO-reset the panda,
#     panda_ready() (1 s wait, bootstub check, up_to_date() against the H7 image),
#     and on failure flash_panda() = BOOT0 -> DFU -> recover + flash the H7 image.
#     On the comma three every step is wrong for the F4: the USB re-enumeration
#     after a reset outruns the 1 s wait, up_to_date() can never match, and the
#     DFU path would push H7 images at an STM32F4 (the BMW plugin owns that
#     firmware). On tici: no reset, F4 passes panda_ready() once connected,
#     never flash — just retry; then a USB settle delay (the kernel must release
#     the device before native pandad claims it), BOARDD_SKIP_FW_CHECK for the
#     native check, and a crash backoff.
PANDAD_FILE="$OPENPILOT_DIR/openpilot/selfdrive/pandad/pandad.py"
if [ -f "$PANDAD_FILE" ] && ! grep -q 'c3_compat' "$PANDAD_FILE"; then
  python3 - "$PANDAD_FILE" << 'PYEOF'
import sys
path = sys.argv[1]
with open(path) as f:
    content = f.read()
n = 0
missing = []

old = 'import os\nimport signal\nimport subprocess\n'
new = 'import os\nimport signal\nimport subprocess\nimport time  # c3_compat\n'
if old in content:
    content = content.replace(old, new, 1); n += 1
else:
    missing.append('imports')

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
else:
    missing.append('panda_ready up_to_date')

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
else:
    missing.append('reset/flash loop')

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
else:
    missing.append('run real pandad')

if n == 4:
    with open(path, 'w') as f:
        f.write(content)
    print("[c3_compat] Patched pandad.py for the F4 panda")
else:
    print(f"[c3_compat] WARNING pandad.py: {n}/4 anchors found"
          f" (missing: {', '.join(missing)}), left untouched")
PYEOF
fi

# 9c. pandad.py: no lost-heartbeat check on the F4. At start pandad.py reads
#     Panda.health()["heartbeat_lost"] to set PandaHeartbeatLost. The F4 answers
#     with the v16 health packet, which health() reads with the library's layout
#     (8b only skips the version check), so that field is an unrelated bit.
#     Applied only on top of 9b (it needs get_mcu_type from 8); own marker so a
#     tree patched before this section existed picks it up.
if [ -f "$PANDAD_FILE" ] && grep -q 'c3_compat' "$PANDAD_FILE" && ! grep -q 'c3_compat: F4 health v16' "$PANDAD_FILE"; then
  python3 - "$PANDAD_FILE" << 'PYEOF'
import sys
path = sys.argv[1]
with open(path) as f:
    content = f.read()
old = """      with Panda(s) as p:
        health = p.health()
        if p.is_internal() and health["heartbeat_lost"]:
"""
new = """      with Panda(s) as p:
        # c3_compat: F4 health v16 is not what health() parses — heartbeat_lost would be an unrelated bit
        from panda.python.constants import McuType
        if p.get_mcu_type() == McuType.F4:
          continue
        health = p.health()
        if p.is_internal() and health["heartbeat_lost"]:
"""
if old in content:
    with open(path, 'w') as f:
        f.write(content.replace(old, new, 1))
    print("[c3_compat] Patched pandad.py: no lost-heartbeat check on the F4 panda")
else:
    print("[c3_compat] WARNING pandad.py: lost-heartbeat check anchor not found, left untouched")
PYEOF
fi

# 10. SPI: disable for C3 (F4 panda) to force native pandad USB-only mode
#     Native pandad's SPI protocol is incompatible with the STM32F4.
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

# The handle opens the device node directly: wrap that open in a PermissionError handler
old = '''      if speed not in SPI_DEVICES:
        file = open(DEV_PATH, 'r+b', buffering=0)'''
new = '''      if speed not in SPI_DEVICES:
        try:
          file = open(DEV_PATH, 'r+b', buffering=0)
        except PermissionError as e:
          # c3_compat: SPI disabled for F4 panda USB-only mode
          raise PandaSpiUnavailable("SPI device permission denied (F4 USB-only mode)") from e'''

if old in content:
    with open(path, 'w') as f:
        f.write(content.replace(old, new))
    print("[c3_compat] Patched spi.py PermissionError → PandaSpiUnavailable")
else:
    print("[c3_compat] WARNING spi.py: SPI device open anchor (open(DEV_PATH)) not found, left untouched")
PYEOF
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
    print("[c3_compat] Patched dfu.py spi_list() exception handler")
else:
    print("[c3_compat] WARNING dfu.py: 'except PandaSpiException:' anchor not found, left untouched")
PYEOF
fi

# 13. Clear Python bytecode cache for patched panda modules
#     Stale .pyc files cause patches to be ignored until cache expires.
find "$OPENPILOT_DIR/panda/python" -name '__pycache__' -type d -exec rm -rf {} + 2>/dev/null || true

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
# bps_blobs.h: 0.11.2 ships the AR0231 slot (index 1, cereal ImageSensor) of all three
# auto-generated BPS tables -- bps_cfg (IO config), bps_striping_output, bps_settings --
# as empty placeholders. The cabin camera runs the BPS pipeline (the SDM845 ISP has only
# two full IFE pixel paths), so the ICP firmware got zeros: A5 watchdog at acquire
# ("SFR: Assertion Failure ... hlssr.c") and, once the IO config was back, a failed
# frame_process (error 4, "BPS sync failed") on every frame. Restore 0.11.1's entries
# (bps_cfg's OX03C10 entry is byte-identical across the releases; the striping output
# and settings tables changed for the other sensors, 0.11.1's AR0231 blobs are the
# ones that drove this hardware). Each table is handled on its own marker so a
# partially patched file completes.
blobs = os.path.join(d, 'cameras', 'bps_blobs.h')
with open(blobs) as f:
    b = f.read()
import re
changed = []
for tbl in ('bps_cfg', 'bps_striping_output', 'bps_settings'):
    inc = os.path.join(src, 'camerad', f'{tbl}_ar0231.inc')
    if not os.path.exists(inc):
        continue
    with open(inc) as f:
        entry = f.read()
    if entry.split('//')[-1].strip() in b:
        continue
    m = re.search(r'unsigned char ' + tbl + r'\[4\]\[[^\]]+\] = \{\n([ \t]*\{[ \t]*/\* placeholder \*/[ \t]*\},[ \t]*\n)([ \t]*\{[ \t]*/\* placeholder \*/[ \t]*\},[ \t]*\n)', b)
    if m:
        b = b[:m.end(1)] + entry + b[m.end(2):]
        changed.append(tbl)
    else:
        print(f"[c3_compat] WARNING camerad/cameras/bps_blobs.h: {tbl} placeholder anchor not found, left untouched")
if changed:
    with open(blobs, 'w') as f:
        f.write(b)
    print(f"[c3_compat] Patched camerad/cameras/bps_blobs.h: AR0231 entries restored ({', '.join(changed)})")

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

# 16c. pandad: health packet v16 of the F4 ("dos") panda. The BMW plugin's firmware
#      answers USB 0xd2 with the 58-byte v16 layout (uint32 voltage/current, one byte
#      per boolean, float interrupt load); 0.11.2's health_t is 48 bytes with uint16
#      voltage/current and a flags_pkt bitfield. Read into health_t, every field after
#      uptime is shifted: ignition_line landed in safety_param, flags were read out of
#      faults_pkt (always 0) -> the car never went onroad on 0.11.2 (first drive
#      attempt 2026-11-03: VBAT 15.3 V, ignitionLine False). Panda::get_state() now
#      reads health_f4_t on a dos panda and translates it (src/pandad/health_f4.h,
#      host-tested in tests/test_health_f4.py).
PANDAD_DIR="$OPENPILOT_DIR/openpilot/selfdrive/pandad"
if [ -d "$PANDAD_DIR" ] && [ -f "$C3_SRC/pandad/health_f4.h" ]; then
  if ! cmp -s "$C3_SRC/pandad/health_f4.h" "$PANDAD_DIR/health_f4.h"; then
    cp "$C3_SRC/pandad/health_f4.h" "$PANDAD_DIR/health_f4.h"
    echo "[c3_compat] Installed pandad/health_f4.h"
  fi
  python3 - "$PANDAD_DIR" << 'PYEOF'
import sys, os
d = sys.argv[1]
def patch(rel, pairs, marker):
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
        print(f"[c3_compat] Patched pandad/{rel} for the F4 health packet")
    else:
        print(f"[c3_compat] WARNING pandad/{rel}: {n}/{len(pairs)} anchors found, left untouched (F4 health)")

patch('panda.h', [(
    '#include "panda/board/health.h"\n',
    '#include "panda/board/health.h"\n#include "selfdrive/pandad/health_f4.h"  // c3_compat: F4 health v16\n',
)], 'health_f4.h')
patch('panda.cc', [(
    'std::optional<health_t> Panda::get_state() {\n'
    '  health_t health {0};\n'
    '  int err = handle->control_read(0xd2, 0, 0, (unsigned char*)&health, sizeof(health));\n',
    'std::optional<health_t> Panda::get_state() {\n'
    '  health_t health {0};\n'
    '  if (hw_type == cereal::PandaState::PandaType::DOS) {  // c3_compat: the F4 firmware speaks health v16\n'
    '    health_f4_t h4 {0};\n'
    '    int err4 = handle->control_read(0xd2, 0, 0, (unsigned char*)&h4, sizeof(h4));\n'
    '    if (err4 < 0) return std::nullopt;\n'
    '    health_from_f4(health, h4);\n'
    '    return health;\n'
    '  }\n'
    '  int err = handle->control_read(0xd2, 0, 0, (unsigned char*)&health, sizeof(health));\n',
)], 'health_f4_t')
PYEOF
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

echo "[c3_compat] Boot patch pass complete"
