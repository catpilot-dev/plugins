"""Network proxy env var management.

Manages HTTP/SOCKS5 proxy env vars (ALL_PROXY, HTTP_PROXY, HTTPS_PROXY).
"""
import os

import params_helper

DEFAULT_PROXY = "http://172.20.10.1:7890"


def on_startup(default, **kwargs):
  """Hook: manager.startup — no-op, proxy setup deferred to github_pinger.

  Previously set proxy env vars here before WiFi connected, which blocked
  all network traffic when proxy was unreachable. Now github_pinger handles
  proxy setup with SSID awareness after WiFi is up.
  """
  return default


def apply_proxy_env(addr: str):
  """Set proxy environment variables for the current process."""
  os.environ["ALL_PROXY"] = addr
  os.environ["HTTP_PROXY"] = addr
  os.environ["HTTPS_PROXY"] = addr
  os.environ["NO_PROXY"] = "localhost,127.0.0.1,10.0.0.0/24"


def clear_proxy_env():
  """Remove proxy environment variables."""
  for key in ("ALL_PROXY", "HTTP_PROXY", "HTTPS_PROXY", "NO_PROXY"):
    os.environ.pop(key, None)
