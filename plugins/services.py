#!/usr/bin/env python3
"""Scan plugin manifests and inject declared services into cereal/services.py."""
import glob
import json
import os
import re
import sys
from config import PLUGINS_RUNTIME_DIR


def collect_services(plugins_dir: str) -> dict:
  """Collect services from all enabled plugin.json files."""
  services: dict = {}
  for manifest in sorted(glob.glob(os.path.join(plugins_dir, "*/plugin.json"))):
    plugin_dir = os.path.dirname(manifest)
    if os.path.exists(os.path.join(plugin_dir, ".disabled")):
      continue
    try:
      with open(manifest) as f:
        data = json.load(f)
    except (json.JSONDecodeError, OSError):
      continue
    for name, entry in data.get("services", {}).items():
      services[name] = entry
  return services


def _entry_line(name: str, entry) -> str:
  return f'  "{name}": {tuple(entry)},'


def inject_services(services_py: str, services: dict) -> int:
  """Inject or update plugin services in cereal/services.py.

  Returns the count of added or rewritten entries.

  services.py is edited in place and never reset, so an entry injected by an
  older manifest stays there across reinstalls. Adding new entries alone would
  leave a changed entry stale forever. The case that forced this was mapdOut
  gaining a queue_size: the old 3-tuple kept it at SMALL (250 KB) while mapd
  creates the queue at MEDIUM (2 MB), and whichever side opens the queue last
  ftruncates the shared file to its own size. So an existing single-line entry
  whose tuple differs from the manifest is rewritten to match.
  """
  with open(services_py) as f:
    content = f.read()

  added = 0
  lines = content.rstrip("\n").split("\n")

  # Rewrite stale single-line entries in place. Plugin service names never
  # collide with stock openpilot ones, so this only ever touches lines that a
  # plugin injected. A name present in any other form is left alone.
  for name, entry in sorted(services.items()):
    want = _entry_line(name, entry)
    entry_re = re.compile(rf'^\s*"{re.escape(name)}"\s*:\s*\(.*\),\s*$')
    for i, line in enumerate(lines):
      if entry_re.match(line):
        if line != want:
          lines[i] = want
          added += 1
        break
  content_now = "\n".join(lines)

  # Find closing brace of _services dict (standalone "}" line)
  insert_idx = None
  for i in range(len(lines) - 1, -1, -1):
    if lines[i].strip() == "}":
      insert_idx = i
      break

  if insert_idx is None:
    print("[services] ERROR: could not find closing } in services.py", file=sys.stderr)
    return 0

  new_lines = []
  for name, entry in sorted(services.items()):
    if f'"{name}"' in content_now:
      continue
    new_lines.append(_entry_line(name, entry))
    added += 1

  if added:
    lines = lines[:insert_idx] + new_lines + lines[insert_idx:]
    with open(services_py, "w") as f:
      f.write("\n".join(lines) + "\n")

  return added


def main():
  if len(sys.argv) < 2:
    print(f"Usage: {sys.argv[0]} <path/to/cereal/services.py> [plugins_dir]", file=sys.stderr)
    sys.exit(1)

  services_py = sys.argv[1]
  plugins_dir = sys.argv[2] if len(sys.argv) > 2 else PLUGINS_RUNTIME_DIR

  services = collect_services(plugins_dir)
  if not services:
    print("[services] No plugin services found")
    return

  added = inject_services(services_py, services)
  if added:
    print(f"[services] Injected or updated {added} service(s) in {services_py}")
  else:
    print("[services] All plugin services already present and current")


if __name__ == "__main__":
  main()
