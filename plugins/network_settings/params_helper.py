"""Plugin-safe params read/write via raw file I/O.

Two storage locations:
- /data/params/d/ — volatile params (cleared by openpilot on boot via clearAll)
- PERSIST_DIR — persistent params (survives reboot, not touched by openpilot)

Plugin params are NOT in params_keys.h, so clearAll() deletes any unknown
files from /data/params/d/ on every manager start. Persistent user config
(ProxySSID, ProxyAddress, StaticIPNetworks) must be stored outside of it.
"""
import os
from pathlib import Path

PARAMS_DIR = Path("/data/plugins-runtime/network_settings/data")
# Keep PERSIST_DIR as alias for tests that patch both
PERSIST_DIR = PARAMS_DIR


def _dir_for(key: str) -> Path:
  return PARAMS_DIR


def get(key: str) -> str | None:
  """Read a param value. Returns None if not set."""
  try:
    return (_dir_for(key) / key).read_text()
  except FileNotFoundError:
    return None


def get_bool(key: str) -> bool:
  """Read a boolean param. '1' = True, anything else = False."""
  return get(key) == "1"


def put(key: str, value: str) -> None:
  """Write a param value."""
  d = _dir_for(key)
  d.mkdir(parents=True, exist_ok=True)
  with open(d / key, "w") as f:
    f.write(value)
    os.fsync(f.fileno())


def put_bool(key: str, value: bool) -> None:
  """Write a boolean param."""
  put(key, "1" if value else "0")


def remove(key: str) -> None:
  """Remove a param."""
  try:
    (_dir_for(key) / key).unlink()
  except FileNotFoundError:
    pass
