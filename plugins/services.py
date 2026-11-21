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
  #
  # EVERY matching line, not just the first: catpilot's boot-time
  # selfdrive/plugins/builder.py appends its own "# plugin services
  # (auto-generated)" block after ours, so a device carries each plugin
  # service twice, and in a dict literal the LAST duplicate wins.
  for name, entry in sorted(services.items()):
    want = _entry_line(name, entry)
    entry_re = re.compile(rf'^\s*"{re.escape(name)}"\s*:\s*\(.*\),\s*$')
    for i, line in enumerate(lines):
      if entry_re.match(line) and line != want:
        lines[i] = want
        added += 1
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


def remove_services(services_py: str, names) -> int:
  """Remove every single-line entry for `names` from cereal/services.py.

  Returns the count of removed lines. Matches entries the same way
  inject_services() does (`"name": (...),` on one line), so it removes both
  the line install.sh injected and the copy in builder.py's auto-generated
  block. A name present in any other form (multi-line, commented out) is left
  alone. The file is only rewritten when something was removed.

  Used by install.d/mapd_migration.sh: once the mapd plugin is gone nothing
  re-injects mapdOut / mapdExtendedOut, but services.py is never reset, so
  their old lines would stay forever.
  """
  with open(services_py) as f:
    content = f.read()

  patterns = [re.compile(rf'^\s*"{re.escape(name)}"\s*:\s*\(.*\),\s*$') for name in names]
  lines = content.split("\n")
  kept = [line for line in lines if not any(p.match(line) for p in patterns)]
  removed = len(lines) - len(kept)

  if removed:
    with open(services_py, "w") as f:
      f.write("\n".join(kept))

  return removed


def main():
  if len(sys.argv) >= 2 and sys.argv[1] == "--remove":
    if len(sys.argv) < 4:
      print(f"Usage: {sys.argv[0]} --remove <path/to/cereal/services.py> <name> [name ...]", file=sys.stderr)
      sys.exit(1)
    services_py, names = sys.argv[2], sys.argv[3:]
    removed = remove_services(services_py, names)
    if removed:
      print(f"[services] Removed {removed} stale entr{'y' if removed == 1 else 'ies'} ({', '.join(names)}) from {services_py}")
    else:
      print(f"[services] No stale entries for {', '.join(names)}")
    return

  if len(sys.argv) < 2:
    print(f"Usage: {sys.argv[0]} <path/to/cereal/services.py> [plugins_dir]", file=sys.stderr)
    print(f"       {sys.argv[0]} --remove <path/to/cereal/services.py> <name> [name ...]", file=sys.stderr)
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
