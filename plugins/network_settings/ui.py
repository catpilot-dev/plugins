"""Network settings UI extensions.

Extends AdvancedNetworkSettings with proxy toggle, proxy address editor,
static IPv4 toggle, and IP/gateway/DNS editors. Per-SSID static IP config
stored as JSON in StaticIPNetworks param.

Connectivity check (green CONNECT indicator) is handled by the sidebar's
ui.connectivity_check hook — not in this module.
"""
import json
import math
import time

import params_helper
from proxy import DEFAULT_PROXY, apply_proxy_env, clear_proxy_env

GITHUB_TIMEOUT_NS = 160_000_000_000  # 160 seconds — 2x the 80s check interval

# Static IP defaults (iPhone 13 mini hotspot)
DEFAULT_IP = "172.20.10.8"
DEFAULT_MASK = "255.255.255.0"
DEFAULT_GATEWAY = "172.20.10.1"


def _subnet_gateway(ip: str) -> str:
  """Derive default gateway from IP: 10.0.8.100 -> 10.0.8.1."""
  parts = ip.rsplit('.', 1)
  return parts[0] + '.1' if len(parts) == 2 else ip


def is_github_connected() -> bool:
  """Check if github_pinger has succeeded recently."""
  try:
    last_ping = params_helper.get("LastGithubPingTime")
    if last_ping is None:
      return False
    return (time.monotonic_ns() - int(last_ping)) < GITHUB_TIMEOUT_NS
  except Exception:
    return False


def on_network_settings_extend(default, net_ui):
  """Hook: ui.network_settings_extend — inject proxy + static IP controls."""
  from openpilot.common.swaglog import cloudlog
  from openpilot.system.ui.lib.application import gui_app
  from openpilot.system.ui.lib.multilang import tr
  from openpilot.system.ui.widgets.keyboard import Keyboard
  from openpilot.system.ui.widgets.list_view import ListItem, ToggleAction, button_item
  from openpilot.system.ui.widgets.network import AdvancedNetworkSettings, MAX_PASSWORD_LENGTH

  class ProxyNetworkSettings(AdvancedNetworkSettings):
    """Extends Advanced panel with proxy toggle, address editor, and per-SSID static IPv4."""

    def __init__(self, wifi_manager):
      super().__init__(wifi_manager)
      self._wifi_manager = wifi_manager
      self._last_connected_ssid = ""

      # Proxy toggle
      proxy_enabled = params_helper.get_bool("ProxyEnabled")
      self._proxy_action = ToggleAction(initial_state=proxy_enabled)
      proxy_toggle = ListItem(
        lambda: tr("Enable Proxy"),
        action_item=self._proxy_action,
        callback=self._toggle_proxy,
      )

      # Proxy address editor
      self._proxy_setting_btn = button_item(
        lambda: tr("Proxy Address"),
        lambda: tr("EDIT"),
        callback=self._edit_proxy_address,
      )
      self._proxy_setting_btn.action_item.set_value(lambda: self._get_proxy_address())
      self._proxy_setting_btn.set_visible(lambda: self._proxy_action.get_state())

      # Static IPv4 toggle (per-SSID)
      self._static_ip_action = ToggleAction(initial_state=False)
      static_toggle = ListItem(
        lambda: tr("Static IPv4"),
        action_item=self._static_ip_action,
        callback=self._toggle_static_ip,
      )

      # Static IP fields
      self._ip_btn = button_item(
        lambda: tr("IP Address"), lambda: tr("EDIT"),
        callback=lambda: self._edit_static_field("ip", "IP Address", "e.g. 172.20.10.8"),
      )
      self._ip_btn.action_item.set_value(lambda: self._get_current_ip())
      self._ip_btn.set_visible(lambda: self._static_ip_action.get_state())

      self._gateway_btn = button_item(
        lambda: tr("Gateway"), lambda: tr("EDIT"),
        callback=lambda: self._edit_static_field("gw", "Gateway", "e.g. 172.20.10.1"),
      )
      self._gateway_btn.action_item.set_value(lambda: self._get_current_gateway())
      self._gateway_btn.set_visible(lambda: self._static_ip_action.get_state())

      self._dns_btn = button_item(
        lambda: tr("DNS"), lambda: tr("EDIT"),
        callback=lambda: self._edit_static_field("dns", "DNS", "e.g. 172.20.10.1"),
      )
      self._dns_btn.action_item.set_value(lambda: self._get_current_dns())
      self._dns_btn.set_visible(lambda: self._static_ip_action.get_state())

      # ButtonAction.get_width_hint() returns exact float width — float32 precision
      # loss through rl.Rectangle causes gui_label to truncate. ceil() fixes it.
      for btn in (self._ip_btn, self._gateway_btn, self._dns_btn, self._proxy_setting_btn):
        _orig = btn.action_item.get_width_hint
        btn.action_item.get_width_hint = lambda _o=_orig: math.ceil(_o())

      # Insert items at top (before Enable Tethering)
      for i, item in enumerate([proxy_toggle, self._proxy_setting_btn,
                                 static_toggle, self._ip_btn, self._gateway_btn, self._dns_btn]):
        self._scroller._items.insert(i, item)

      self._proxy_keyboard = Keyboard(max_text_size=MAX_PASSWORD_LENGTH, min_text_size=1)

    def _update_state(self):
      super()._update_state()
      ssid = self._get_connected_ssid()
      if ssid != self._last_connected_ssid:
        # Re-apply or clear proxy based on saved ProxySSID
        proxy_ssid = (params_helper.get("ProxySSID") or "").strip()
        if ssid and ssid == proxy_ssid:
          self._proxy_action.set_state(True)
          params_helper.put_bool("ProxyEnabled", True)
          apply_proxy_env(self._get_proxy_address())
        else:
          if self._proxy_action.get_state():
            self._proxy_action.set_state(False)
            params_helper.put_bool("ProxyEnabled", False)
            clear_proxy_env()
        # Re-apply or clear static IP based on saved config for new SSID
        networks = self._get_networks()
        if ssid and ssid in networks:
          self._static_ip_action.set_state(True)
          self._apply_for_ssid(ssid)
        else:
          if self._static_ip_action.get_state():
            self._static_ip_action.set_state(False)
            from static_ip import remove_static_ip
            remove_static_ip(self._wifi_manager)
        self._last_connected_ssid = ssid

    def _get_connected_ssid(self) -> str:
      for n in self._wifi_manager._networks:
        if n.is_connected:
          return n.ssid
      return ""

    def _get_proxy_address(self) -> str:
      return (params_helper.get("ProxyAddress") or "").strip() or DEFAULT_PROXY

    def _toggle_proxy(self):
      enabled = self._proxy_action.get_state()
      params_helper.put_bool("ProxyEnabled", enabled)
      ssid = self._get_connected_ssid()
      if enabled:
        if ssid:
          params_helper.put("ProxySSID", ssid)
        apply_proxy_env(self._get_proxy_address())
      else:
        # Only remove ProxySSID if disabling on the proxy SSID itself
        proxy_ssid = (params_helper.get("ProxySSID") or "").strip()
        if ssid == proxy_ssid:
          params_helper.remove("ProxySSID")
        clear_proxy_env()

    def _edit_proxy_address(self):
      def on_done(result):
        if result != 1:
          return
        addr = self._proxy_keyboard.text.strip()
        if addr:
          params_helper.put("ProxyAddress", addr)
          if self._proxy_action.get_state():
            apply_proxy_env(addr)

      self._proxy_keyboard.reset(min_text_size=1)
      self._proxy_keyboard.set_title(tr("Proxy Address"), tr("e.g. socks5://host:port"))
      self._proxy_keyboard.set_text(self._get_proxy_address())
      gui_app.set_modal_overlay(self._proxy_keyboard, on_done)

    # --- Static IP helpers ---

    def _get_networks(self) -> dict:
      raw = params_helper.get("StaticIPNetworks")
      if raw:
        try:
          return json.loads(raw)
        except Exception:
          pass
      return {}

    def _save_networks(self, networks: dict):
      params_helper.put("StaticIPNetworks", json.dumps(networks))

    def _get_ssid_config(self, ssid: str) -> tuple[str, str, str]:
      if not ssid:
        return DEFAULT_IP, DEFAULT_GATEWAY, DEFAULT_GATEWAY
      cfg = self._get_networks().get(ssid, {})
      gw = cfg.get("gw", DEFAULT_GATEWAY)
      return cfg.get("ip", DEFAULT_IP), gw, cfg.get("dns", gw)

    def _save_ssid_config(self, ssid: str, ip: str, gateway: str, dns: str):
      networks = self._get_networks()
      networks[ssid] = {"ip": ip, "gw": gateway, "dns": dns}
      self._save_networks(networks)

    def _get_current_ip(self) -> str:
      return self._get_ssid_config(self._get_connected_ssid())[0]

    def _get_current_gateway(self) -> str:
      return self._get_ssid_config(self._get_connected_ssid())[1]

    def _get_current_dns(self) -> str:
      return self._get_ssid_config(self._get_connected_ssid())[2]

    def _apply_for_ssid(self, ssid: str):
      from static_ip import apply_static_ip
      ip, gateway, dns = self._get_ssid_config(ssid)
      apply_static_ip(self._wifi_manager, ip, DEFAULT_MASK, gateway, dns)

    def _toggle_static_ip(self):
      enabled = self._static_ip_action.get_state()
      ssid = self._get_connected_ssid()
      if enabled:
        if ssid:
          networks = self._get_networks()
          if ssid not in networks:
            networks[ssid] = {"ip": DEFAULT_IP, "gw": DEFAULT_GATEWAY, "dns": DEFAULT_GATEWAY}
            self._save_networks(networks)
          self._apply_for_ssid(ssid)
      else:
        if ssid:
          networks = self._get_networks()
          if ssid in networks:
            del networks[ssid]
            self._save_networks(networks)
        from static_ip import remove_static_ip
        remove_static_ip(self._wifi_manager)

    def _edit_static_field(self, field: str, title: str, hint: str):
      ssid = self._get_connected_ssid()
      ip, gw, dns = self._get_ssid_config(ssid)
      current_val = ip if field == "ip" else (dns if field == "dns" else gw)

      def on_done(result):
        if result != 1:
          return
        value = self._proxy_keyboard.text.strip()
        if value and ssid:
          cur_ip, cur_gw, cur_dns = self._get_ssid_config(ssid)
          if field == "ip":
            old_default_gw = _subnet_gateway(cur_ip)
            new_gw = _subnet_gateway(value) if cur_gw == old_default_gw else cur_gw
            new_dns = new_gw if cur_dns == cur_gw else cur_dns
            self._save_ssid_config(ssid, value, new_gw, new_dns)
          elif field == "gw":
            new_dns = value if cur_dns == cur_gw else cur_dns
            self._save_ssid_config(ssid, cur_ip, value, new_dns)
          else:
            self._save_ssid_config(ssid, cur_ip, cur_gw, value)
          if self._static_ip_action.get_state():
            self._apply_for_ssid(ssid)

      self._proxy_keyboard.reset(min_text_size=1)
      self._proxy_keyboard.set_title(tr(title), tr(hint))
      self._proxy_keyboard.set_text(current_val)
      gui_app.set_modal_overlay(self._proxy_keyboard, on_done)

  # --- Inject into NetworkUI ---
  try:
    net_ui._advanced_panel = ProxyNetworkSettings(net_ui._wifi_manager)
    cloudlog.info("network_settings: injected ProxyNetworkSettings into advanced panel")
  except Exception:
    cloudlog.exception("network_settings: failed to inject ProxyNetworkSettings")

  return None
