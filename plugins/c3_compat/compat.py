"""
Comma 3 compatibility module.

Reports AGNOS version, device type and panda type via the device.health_check
hook, and logs them once at startup. Warns when a tici (the only device this
plugin loads on, see plugin.json device_filter) does not report a Dos (STM32F4)
panda.

Every process that runs plugin hooks imports this module, so it does nothing
at import time: the startup work runs on the first health check, which only
plugind dispatches.
"""

import os

from openpilot.common.swaglog import cloudlog


def get_agnos_version() -> str:
  """Read AGNOS version from /VERSION."""
  try:
    with open("/VERSION") as f:
      return f.read().strip()
  except FileNotFoundError:
    return "unknown"


def get_device_type() -> str:
  """Detect device type from devicetree model string."""
  model_path = "/sys/firmware/devicetree/base/model"
  try:
    with open(model_path) as f:
      model = f.read().strip().rstrip("\x00")
    model_lower = model.lower()
    if "tici" in model_lower:
      return "tici"
    elif "tizi" in model_lower:
      return "tizi"
    elif "mici" in model_lower:
      return "mici"
    return model
  except FileNotFoundError:
    return "unknown"


def _build_version_description() -> str:
  """Build the UpdaterCurrentDescription string from git info (mirrors updated.py)."""
  import subprocess
  import datetime
  try:
    from openpilot.common.basedir import BASEDIR
    branch = subprocess.run(
      ["git", "rev-parse", "--abbrev-ref", "HEAD"],
      cwd=BASEDIR, capture_output=True, text=True, timeout=5,
    ).stdout.strip()
    commit = subprocess.run(
      ["git", "rev-parse", "HEAD"],
      cwd=BASEDIR, capture_output=True, text=True, timeout=5,
    ).stdout.strip()[:7]
    with open(os.path.join(BASEDIR, "openpilot", "common", "version.h")) as f:
      version = f.read().split('"')[1]
    ts = subprocess.run(
      ["git", "show", "-s", "--format=%ct", "HEAD"],
      cwd=BASEDIR, capture_output=True, text=True, timeout=5,
    ).stdout.strip()
    dt = datetime.datetime.fromtimestamp(int(ts)).strftime("%b %d")
    return f"{version} / {branch} / {commit} / {dt}"
  except Exception:
    return ""


def log_startup_info():
  """Log compatibility status and fill in the version shown in the Software panel."""
  agnos = get_agnos_version()
  device = get_device_type()
  cloudlog.info(f"C3 compat: AGNOS {agnos}, device {device}")

  # Warn if AGNOS version is unexpected for C3
  if device == "tici":
    try:
      major = int(agnos.split(".")[0])
      if major > 13:
        cloudlog.warning(f"AGNOS {agnos} may not be fully supported on Comma 3 (tici)")
    except (ValueError, IndexError):
      pass

  # UpdaterCurrentDescription is CLEAR_ON_MANAGER_START but updated never runs
  # when DisableUpdates=True — populate it here so Software panel shows the version.
  try:
    from openpilot.common.params import Params
    params = Params()
    if params.get_bool("DisableUpdates") and not params.get("UpdaterCurrentDescription"):
      desc = _build_version_description()
      if desc:
        params.put("UpdaterCurrentDescription", desc)
        cloudlog.info(f"C3 compat: set UpdaterCurrentDescription = {desc}")
  except Exception:
    pass


_health_sm = None
_startup_logged = False


def on_health_check(acc, **kwargs):
  """
  Hook: device.health_check

  Reports AGNOS version, device type and panda type; warns if a tici does
  not report a Dos (F4) panda.
  Called periodically by plugind; result is merged into the accumulator
  dict and published to plugin_bus for bus_logger to capture into rlogs.
  """
  global _health_sm, _startup_logged

  if not _startup_logged:
    _startup_logged = True
    log_startup_info()

  device = get_device_type()

  result = {
    "agnos_version": get_agnos_version(),
    "device_type": device,
    "status": "ok",
    "warnings": [],
  }

  # Check panda MCU type from pandaStates
  try:
    from openpilot.cereal import messaging
    if _health_sm is None:
      _health_sm = messaging.SubMaster(["pandaStates"])
    _health_sm.update(0)
    if _health_sm.valid["pandaStates"] and len(_health_sm["pandaStates"]) > 0:
      panda_type = str(_health_sm["pandaStates"][0].pandaType)
      result["panda_type"] = panda_type

      # The comma three (tici) has a Dos board panda (STM32F4)
      if device == "tici" and "dos" not in panda_type.lower():
        result["warnings"].append(
          f"Expected Dos (F4) panda on C3, got {panda_type}"
        )
  except Exception as e:
    result["warnings"].append(f"Could not read pandaStates: {e}")

  if result["warnings"]:
    result["status"] = "warning"
    for w in result["warnings"]:
      cloudlog.warning(f"C3 compat health: {w}")

  return {**acc, "c3-compat": result}
