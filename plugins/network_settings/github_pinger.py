#!/usr/bin/env python3
"""Plugin process that pings github.com and manages per-SSID proxy.

Follows the same pattern as Athena's LastAthenaPingTime:
  - On success: writes monotonic_ns timestamp to LastGithubPingTime
  - On failure: removes LastGithubPingTime
  - UI reads from Params to determine green SSID

Per-SSID proxy: when the user enables proxy on a given SSID, that SSID is
saved to ProxySSID.  Each cycle we check the current connected SSID and
auto-enable/disable proxy accordingly.

curl automatically routes through SOCKS5 proxy when ALL_PROXY is set,
so this check proves proxy connectivity when behind GFW.
"""
import os
import subprocess
import time

import params_helper
from proxy import DEFAULT_PROXY, apply_proxy_env, clear_proxy_env

CHECK_INTERVAL = 80  # seconds — matches Athena timeout window
SSID_POLL_INTERVAL = 5  # seconds — fast poll for SSID changes
DEFAULT_MASK = "255.255.255.0"


def check_github() -> bool:
  """Check if github.com is reachable. Uses proxy env vars if set."""
  try:
    result = subprocess.run(
      ["curl", "-s", "--connect-timeout", "5",
       "-o", "/dev/null", "-w", "%{http_code}", "https://github.com"],
      capture_output=True, text=True, timeout=8,
    )
    code = result.stdout.strip()
    return code.startswith("2") or code == "301"
  except Exception:
    return False


def get_connected_ssid() -> str:
  """Return the SSID of the currently connected WiFi network, or ''."""
  try:
    result = subprocess.run(
      ["nmcli", "-t", "-f", "NAME,TYPE,DEVICE", "connection", "show", "--active"],
      capture_output=True, text=True, timeout=5,
    )
    for line in result.stdout.strip().split("\n"):
      parts = line.split(":")
      if len(parts) >= 3 and parts[1] == "802-11-wireless" and parts[2] == "wlan0":
        name = parts[0]
        # Strip "openpilot connection " prefix if present
        prefix = "openpilot connection "
        return name[len(prefix):] if name.startswith(prefix) else name
  except Exception:
    pass
  return ""


def sync_proxy_for_ssid(current_ssid: str):
  """Auto-enable/disable proxy based on current SSID vs saved ProxySSID."""
  proxy_ssid = (params_helper.get("ProxySSID") or "").strip()
  if not proxy_ssid:
    return

  enabled = params_helper.get_bool("ProxyEnabled")

  if current_ssid == proxy_ssid and not enabled:
    addr = (params_helper.get("ProxyAddress") or "").strip() or DEFAULT_PROXY
    params_helper.put_bool("ProxyEnabled", True)
    apply_proxy_env(addr)
  elif current_ssid != proxy_ssid and enabled:
    params_helper.put_bool("ProxyEnabled", False)
    clear_proxy_env()


def sync_static_ip_for_ssid(current_ssid: str):
  """Auto-apply/remove static IP based on current SSID vs saved StaticIPNetworks."""
  import json
  raw = params_helper.get("StaticIPNetworks")
  if not raw:
    return
  try:
    networks = json.loads(raw)
  except Exception:
    return

  if current_ssid in networks:
    cfg = networks[current_ssid]
    ip = cfg.get("ip", "")
    gw = cfg.get("gw", "")
    dns = cfg.get("dns", gw)
    if ip and gw:
      from static_ip import apply_static_ip_blocking
      apply_static_ip_blocking(ip, DEFAULT_MASK, gw, dns)


def main():
  last_ssid = ""

  # Initial setup
  current_ssid = get_connected_ssid()
  if current_ssid:
    sync_proxy_for_ssid(current_ssid)
    sync_static_ip_for_ssid(current_ssid)
    last_ssid = current_ssid
  if params_helper.get_bool("ProxyEnabled"):
    addr = (params_helper.get("ProxyAddress") or "").strip() or DEFAULT_PROXY
    apply_proxy_env(addr)

  last_github_check = 0
  while True:
    current_ssid = get_connected_ssid()

    # Fast SSID change detection
    if current_ssid and current_ssid != last_ssid:
      sync_proxy_for_ssid(current_ssid)
      sync_static_ip_for_ssid(current_ssid)
      last_ssid = current_ssid

    # Periodic github check
    now = time.monotonic()
    if now - last_github_check >= CHECK_INTERVAL:
      sync_proxy_for_ssid(current_ssid or "")
      if check_github():
        params_helper.put("LastGithubPingTime", str(time.monotonic_ns()))
      else:
        params_helper.remove("LastGithubPingTime")
      last_github_check = now

    time.sleep(SSID_POLL_INTERVAL)


if __name__ == "__main__":
  main()
