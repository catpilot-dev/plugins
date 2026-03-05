# network_settings

Network proxy and static IPv4 configuration for openpilot, with per-SSID auto-switching and github.com connectivity indicator.

## Features

**Proxy** — SOCKS5/HTTP proxy toggle with per-SSID auto-switch. Enable proxy on one network (e.g. hotspot behind GFW), and it auto-enables when connecting to that SSID, auto-disables on other networks. Persists across reboots and SSID switches.

**Static IPv4** — Per-SSID static IP configuration via `nmcli device modify`. Each SSID stores its own IP/GW/DNS. Unsaved SSIDs default to DHCP. Auto-reapplied on SSID reconnect.

**Connectivity** — Background `github_pinger` process checks github.com reachability every 80s. Connected SSID turns green in WiFi list. Sidebar shows online when github.com is reachable (proves internet works even behind GFW).

**Background SSID Monitoring** — `github_pinger` polls SSID every 5s and auto-applies proxy/static IP on network switch, without requiring the UI to be open.

## UI Layout

Settings > Network > Advanced:

```
Enable Proxy                                    [TOGGLE]
Proxy Address       socks5://172.20.10.1:7890     [EDIT]
Static IPv4                                     [TOGGLE]
IP                  172.20.10.8                    [EDIT]
GW                  172.20.10.1                    [EDIT]
DNS                 172.20.10.1                    [EDIT]
```

## Per-SSID Behavior

| Action | Effect |
|---|---|
| Enable static IP | Saves config for current SSID, applies via nmcli |
| Disable static IP | Removes SSID config, reverts to DHCP |
| Switch to configured SSID | Toggle ON, saved IP/GW/DNS applied (within 5s) |
| Switch to unconfigured SSID | Toggle OFF, DHCP used |
| Enable proxy | Saves current SSID as proxy SSID |
| Disable proxy on proxy SSID | Removes proxy SSID association |
| Disable proxy on other SSID | Proxy SSID preserved for auto-restore |
| Switch to proxy SSID | Proxy auto-enables (within 5s) |
| Switch away from proxy SSID | Proxy auto-disables |
| Edit IP address | GW/DNS auto-derive to x.x.x.1 if unchanged |
| Reboot | Proxy and static IP auto-applied on reconnect |

## Files

| File | Purpose |
|---|---|
| `ui.py` | UI extensions: proxy toggle, static IP fields, green SSID |
| `static_ip.py` | Apply/remove static IPv4 via nmcli device modify |
| `proxy.py` | Proxy environment variable management |
| `github_pinger.py` | Background process: github.com ping, SSID polling, auto-apply proxy/static IP |
| `params_helper.py` | Lightweight params read/write (file-based) |
| `plugin.json` | Plugin manifest: hooks, process, params |

## Hooks

- `manager.startup` — Proxy setup (deferred to github_pinger)
- `ui.network_settings_extend` — Inject proxy/static IP controls into Advanced panel
- `ui.connectivity_check` — Report github.com reachability to sidebar

## Params

| Param | Type | Description |
|---|---|---|
| `ProxyEnabled` | bool | Proxy toggle state |
| `ProxyAddress` | string | Proxy URL (default: `socks5://172.20.10.1:7890`) |
| `ProxySSID` | string | SSID where proxy auto-enables |
| `StaticIPNetworks` | JSON | Per-SSID static IP config: `{"ssid": {"ip", "gw", "dns"}}` |

## Implementation Notes

- Static IP uses `nmcli device modify` (runtime-only, not `connection modify`) because AGNOS netplan rejects connection-level changes. The pinger re-applies on every SSID change/reboot.
- ProxySSID is only cleared when toggling proxy off on the proxy SSID itself, not when disabling on a different network.
- SSID change detection runs in both `github_pinger` (background, every 5s) and `ui.py` `_update_state` (when Advanced panel is open).

## Tests

```bash
python -m pytest plugins/network_settings/tests/ -v
```
