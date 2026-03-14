"""Apply/remove static IPv4 via nmcli device modify.

AGNOS uses netplan as NM backend — 'nmcli connection modify' is rejected with
"netplan generate failed". Instead we use 'nmcli device modify' which changes
the runtime device configuration directly, bypassing connection profiles.
This is a temporary runtime change (reverts on reconnect/reboot), which is
fine since the plugin re-applies static IP on SSID change.
"""
import subprocess
import threading

from openpilot.common.swaglog import cloudlog


def netmask_to_prefix(mask: str) -> int:
  parts = mask.split('.')
  if len(parts) != 4:
    raise ValueError(f"Invalid netmask: {mask}")
  num = 0
  for p in parts:
    num = (num << 8) | int(p)
  prefix = 0
  for i in range(31, -1, -1):
    if num & (1 << i):
      prefix += 1
    else:
      break
  return prefix


def apply_static_ip_blocking(ip: str, mask: str, gateway: str, dns: str = ""):
  """Set static IPv4 on wlan0 via nmcli device modify (blocking)."""
  try:
    prefix = netmask_to_prefix(mask)
    dns_val = dns or gateway

    result = subprocess.run(
      ["sudo", "nmcli", "device", "modify", "wlan0",
       "ipv4.method", "manual",
       "ipv4.addresses", f"{ip}/{prefix}",
       "ipv4.gateway", gateway,
       "ipv4.dns", dns_val],
      capture_output=True, text=True, timeout=15,
    )

    if result.returncode != 0:
      cloudlog.warning(f"static_ip: nmcli device modify failed: {result.stderr.strip()}")
      return

    cloudlog.info(f"static_ip: applied {ip}/{prefix} gw {gateway} dns {dns_val}")

  except Exception:
    cloudlog.exception("static_ip: failed to apply")


def apply_static_ip(wifi_manager, ip: str, mask: str, gateway: str, dns: str = ""):
  """Set static IPv4 on wlan0 via nmcli device modify (async, for UI)."""
  threading.Thread(target=lambda: apply_static_ip_blocking(ip, mask, gateway, dns), daemon=True).start()


def remove_static_ip(wifi_manager):
  """Revert wlan0 to DHCP via nmcli device modify."""
  def worker():
    try:
      result = subprocess.run(
        ["sudo", "nmcli", "device", "modify", "wlan0",
         "ipv4.method", "auto",
         "ipv4.addresses", "",
         "ipv4.gateway", "",
         "ipv4.dns", ""],
        capture_output=True, text=True, timeout=15,
      )

      if result.returncode != 0:
        cloudlog.warning(f"static_ip: nmcli device modify (revert) failed: {result.stderr.strip()}")
        return

      cloudlog.info("static_ip: reverted to DHCP")

    except Exception:
      cloudlog.exception("static_ip: failed to revert")

  threading.Thread(target=worker, daemon=True).start()
