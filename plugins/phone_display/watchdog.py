#!/usr/bin/env python3
"""
phone_watchdog — monitors WebRTC phone display session at 0.5 Hz.

Polls localhost:5001/health every 2s. Publishes {phone_active, required}
on the plugin bus topic "phone_display". The selfdrived.events hook
reads this to block engagement when phone is disconnected.

Logic:
  - RK3588 (headless): phone is always required regardless of param
  - Comma devices: respects CatEyePhoneRequired param (opt-in)

The bus_logger plugin captures these messages automatically for debugging.
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import time
import urllib.request
import urllib.error
import json
from config import read_plugin_param
from openpilot.common.swaglog import cloudlog
from openpilot.system.hardware import HARDWARE

POLL_INTERVAL = 2.0
WEBRTCD_HEALTH_URL = "http://127.0.0.1:5001/health"

# RK3588 is headless — phone display is always mandatory
_IS_HEADLESS = HARDWARE.get_device_type() == 'rk3588'


def _read_required_param() -> bool:
  """Read CatEyePhoneRequired from plugin data dir. Returns False on any error."""
  return read_plugin_param("phone_display", "CatEyePhoneRequired") == "1"


def _check_webrtcd() -> bool:
  """Poll webrtcd /health. Returns True if at least one phone session is active."""
  try:
    req = urllib.request.Request(WEBRTCD_HEALTH_URL)
    with urllib.request.urlopen(req, timeout=2) as resp:
      data = json.loads(resp.read())
    return data.get("sessions", 0) > 0
  except Exception:
    return False


def main():
  from openpilot.selfdrive.plugins.plugin_bus import PluginPub

  pub = PluginPub("phone_display")
  prev_state: tuple[bool, bool] | None = None  # (required, phone_active)

  cloudlog.info("phone_watchdog: starting (headless=%s)", _IS_HEADLESS)

  while True:
    required = _IS_HEADLESS or _read_required_param()
    phone_active = _check_webrtcd() if required else True

    state = (required, phone_active)
    if state != prev_state:
      if required and not phone_active:
        cloudlog.warning("phone_watchdog: phone display disconnected — engagement blocked")
      elif required and phone_active:
        cloudlog.info("phone_watchdog: phone display connected")
      prev_state = state

    pub.send({"required": required, "phone_active": phone_active})
    time.sleep(POLL_INTERVAL)


if __name__ == "__main__":
  main()
