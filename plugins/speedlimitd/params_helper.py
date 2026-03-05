"""Plugin-safe params read/write via raw file I/O.

Uses /data/plugins/speedlimitd/data/ instead of /data/params/d/ to avoid
openpilot's Params::clearAll() which rejects unknown keys.
"""
import os
from pathlib import Path

PARAMS_DIR = Path("/data/plugins/speedlimitd/data")


def get(key: str) -> str | None:
  """Read a param value. Returns None if not set."""
  try:
    return (PARAMS_DIR / key).read_text()
  except FileNotFoundError:
    return None


def put(key: str, value: str) -> None:
  """Write a param value."""
  PARAMS_DIR.mkdir(parents=True, exist_ok=True)
  with open(PARAMS_DIR / key, "w") as f:
    f.write(value)
    os.fsync(f.fileno())


def remove(key: str) -> None:
  """Remove a param."""
  try:
    (PARAMS_DIR / key).unlink()
  except FileNotFoundError:
    pass
