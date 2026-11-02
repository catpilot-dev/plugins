#!/bin/bash
# BMW panda firmware build — plugin-owned safety, zero opendbc fork.
#
# The plugin's safety/bmw.h and safety/tests/test_bmw.py are the single source
# of record. This script injects them into a firmware workspace (dzid26-era
# openpilot fork with the F4/Dos-capable panda: default ~/openpilot), runs the
# safety suite there, builds the STM32F4 panda firmware, then restores the
# workspace tree so its opendbc_repo carries no local modifications.
#
# Output: $WORKSPACE/panda/board/obj/panda.bin.signed  (flash from the C3)
set -euo pipefail

WORKSPACE="${BMW_FW_WORKSPACE:-$HOME/openpilot}"
ODB="$WORKSPACE/opendbc_repo"
SAFETY_DIR="$(cd "$(dirname "$0")" && pwd)"

MODES_BMW="opendbc/safety/modes/bmw.h"
TEST_BMW="opendbc/safety/tests/test_bmw.py"
LIBSAFETY_C="opendbc/safety/tests/libsafety/safety.c"

[ -d "$ODB" ] || { echo "firmware workspace not found: $ODB"; exit 1; }
if ! git -C "$ODB" diff --quiet -- "$MODES_BMW" "$TEST_BMW" "$LIBSAFETY_C"; then
  echo "refusing: injection targets have local modifications in $ODB"
  exit 1
fi

restore() {
  git -C "$ODB" checkout -- "$MODES_BMW" "$TEST_BMW" "$LIBSAFETY_C"
}
trap restore EXIT

# 1) inject plugin sources
cp "$SAFETY_DIR/bmw.h" "$ODB/$MODES_BMW"
cp "$SAFETY_DIR/tests/test_bmw.py" "$ODB/$TEST_BMW"
# libsafety shim: ignition_can is a firmware-only global (panda can_common.h),
# referenced by the UDS gating in bmw.h — define it for the x86 test lib.
python3 - "$ODB/$LIBSAFETY_C" <<'PYEOF'
import sys
p = sys.argv[1]
src = open(p).read()
if 'bool ignition_can' not in src:
  marker = '#include "opendbc/safety/board/can.h"'
  src = src.replace(marker, marker +
    '\n\n// injected by bmw_e9x_e8x build_firmware.sh: firmware-only global\nbool ignition_can = false;')
  open(p, 'w').write(src)
PYEOF

source "$WORKSPACE/.venv/bin/activate"

# 2) safety suite against the injected sources
(cd "$ODB" && scons -j"$(nproc)" opendbc/safety/tests/libsafety \
  && python -m pytest "$TEST_BMW" -q)

# 3) F4 (Dos) panda firmware
# Built from a pinned panda ref in a detached worktree: the fork's HEAD
# (27bc6f2b "Redirect CAN header to opendbc/safety/can.h") targets a newer
# opendbc layout than this workspace has and does not build here. a0848226 is
# the provenance of the firmware running on the car (gitversion DEV-a0848226).
PANDA_REF="${BMW_PANDA_REF:-a0848226}"
FW_TREE="$WORKSPACE/panda-bmw-fw"
if [ ! -d "$FW_TREE" ]; then
  git -C "$WORKSPACE/panda" worktree add --detach "$FW_TREE" "$PANDA_REF"
elif [ "$(git -C "$FW_TREE" rev-parse HEAD)" != "$(git -C "$WORKSPACE/panda" rev-parse "$PANDA_REF")" ]; then
  git -C "$FW_TREE" checkout --detach "$PANDA_REF"
fi
(cd "$FW_TREE" && scons -j"$(nproc)")

echo
echo "OK: $(ls -la "$FW_TREE/board/obj/panda.bin.signed")"
