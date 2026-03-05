#!/usr/bin/env python3
"""Scan plugin manifests and inject declared services into cereal/services.py."""
import glob
import json
import os
import sys


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


def inject_services(services_py: str, services: dict) -> int:
  """Inject missing services into cereal/services.py. Returns count of added entries."""
  with open(services_py) as f:
    content = f.read()

  added = 0
  lines = content.rstrip("\n").split("\n")

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
    if f'"{name}"' in content:
      continue
    new_lines.append(f'  "{name}": {tuple(entry)},')
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
  plugins_dir = sys.argv[2] if len(sys.argv) > 2 else "/data/plugins"

  services = collect_services(plugins_dir)
  if not services:
    print("[services] No plugin services found")
    return

  added = inject_services(services_py, services)
  if added:
    print(f"[services] Injected {added} service(s) into {services_py}")
  else:
    print("[services] All plugin services already present")


if __name__ == "__main__":
  main()
