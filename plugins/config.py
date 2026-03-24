"""Centralized path configuration for catpilot plugins.

All device paths are defined here with env var overrides, mirroring the
pattern in connect-on-device/config.py.

Environment variables:
  OPENPILOT_DIR        — openpilot installation  (default: /data/openpilot)
  PLUGINS_RUNTIME_DIR  — installed plugins        (default: /data/plugins-runtime)
  PLUGINS_REPO_DIR     — plugins git repo         (default: /data/plugins)
  PARAMS_DIR           — openpilot params dir     (default: /data/params/d)
  MEDIA_DIR            — media storage root       (default: /data/media)
"""

import os
from pathlib import Path


# ── Core paths ────────────────────────────────────────────────────────────────
OPENPILOT_DIR       = os.getenv("OPENPILOT_DIR",       "/data/openpilot")
PLUGINS_RUNTIME_DIR = os.getenv("PLUGINS_RUNTIME_DIR", "/data/plugins-runtime")
PLUGINS_REPO_DIR    = os.getenv("PLUGINS_REPO_DIR",    "/data/plugins")
PARAMS_DIR          = os.getenv("PARAMS_DIR",          "/data/params/d")
MEDIA_DIR           = os.getenv("MEDIA_DIR",           "/data/media")


# ── Plugin data helpers ───────────────────────────────────────────────────────

def plugin_data_dir(plugin_id: str) -> Path:
  """Return the data directory for a plugin: PLUGINS_RUNTIME_DIR/<id>/data/"""
  return Path(PLUGINS_RUNTIME_DIR) / plugin_id / "data"


def read_plugin_param(plugin_id: str, key: str, default: str = "") -> str:
  """Read a plugin param from its data dir. Returns default if missing."""
  try:
    return (plugin_data_dir(plugin_id) / key).read_text().strip()
  except (FileNotFoundError, OSError):
    return default


def write_plugin_param(plugin_id: str, key: str, value) -> None:
  """Write a plugin param to its data dir. Creates the directory if needed."""
  d = plugin_data_dir(plugin_id)
  d.mkdir(parents=True, exist_ok=True)
  (d / key).write_text(str(value))


def read_param(key: str, default: str = "") -> str:
  """Read an openpilot param from PARAMS_DIR. Returns default if missing."""
  try:
    return (Path(PARAMS_DIR) / key).read_text().strip()
  except (FileNotFoundError, OSError):
    return default


def write_param(key: str, value) -> None:
  """Write an openpilot param to PARAMS_DIR."""
  os.makedirs(PARAMS_DIR, exist_ok=True)
  (Path(PARAMS_DIR) / key).write_text(str(value))
