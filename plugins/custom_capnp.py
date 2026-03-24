#!/usr/bin/env python3
"""Scan plugin manifests and inject cereal schemas into custom.capnp, log.capnp, and car.capnp."""
import glob
import json
import os
import re
import sys
from config import PLUGINS_RUNTIME_DIR


def collect_cereal(plugins_dir: str) -> tuple[dict, list[tuple[str, str]], dict[str, int], dict[str, int]]:
  """Collect slot definitions, standalone schemas, safety models, and event names from enabled plugins.

  Returns (slots, standalone_files, safety_models, event_names) where:
    slots = {slot_num: (struct_name, event_field, slot_file_path)}
    standalone_files = [(plugin_id, path), ...]
    safety_models = {name: ordinal}  — entries to add to SafetyModel enum in car.capnp
    event_names = {name: ordinal}    — entries to add to EventName enum in log.capnp
  """
  slots: dict[int, tuple[str, str, str]] = {}
  standalone: list[tuple[str, str]] = []
  safety_models: dict[str, int] = {}
  event_names: dict[str, int] = {}

  for manifest_path in sorted(glob.glob(os.path.join(plugins_dir, "*/plugin.json"))):
    plugin_dir = os.path.dirname(manifest_path)
    if os.path.exists(os.path.join(plugin_dir, ".disabled")):
      continue
    try:
      with open(manifest_path) as f:
        data = json.load(f)
    except (json.JSONDecodeError, OSError):
      continue

    cereal = data.get("cereal", {})

    for slot_num, slot_info in cereal.get("slots", {}).items():
      schema_file = slot_info.get("schema_file", "")
      struct_name = slot_info.get("struct_name", "")
      event_field = slot_info.get("event_field", "")
      if schema_file and struct_name and event_field:
        slot_path = os.path.join(plugin_dir, schema_file)
        if os.path.isfile(slot_path):
          slots[int(slot_num)] = (struct_name, event_field, slot_path)

    standalone_schema = cereal.get("standalone_schema", "")
    if standalone_schema:
      standalone_path = os.path.join(plugin_dir, standalone_schema)
      if os.path.isfile(standalone_path):
        standalone.append((data.get("id", os.path.basename(plugin_dir)), standalone_path))

    for name, ordinal in cereal.get("safety_models", {}).items():
      safety_models[name] = int(ordinal)

    for name, ordinal in cereal.get("event_names", {}).items():
      event_names[name] = int(ordinal)

  return slots, standalone, safety_models, event_names


def inject_custom_capnp(custom_capnp: str, slots: dict, standalone_files: list[tuple[str, str]]) -> int:
  """Inject slot fields and standalone schemas into custom.capnp. Returns count of changes."""
  with open(custom_capnp) as f:
    content = f.read()

  changes = 0

  for slot_num, (struct_name, _, slot_path) in sorted(slots.items()):
    with open(slot_path) as f:
      body = f.read().rstrip("\n")

    if struct_name in content:
      # Already injected — check if body has changed and update if so
      pattern = rf'(struct {re.escape(struct_name)} @0x[0-9a-f]+ \{{)\n(.*?)\n\}}'
      match = re.search(pattern, content, re.DOTALL)
      if match and match.group(2).strip() == body.strip():
        continue
      # Body changed: replace the existing struct body
      new_content = re.sub(pattern, lambda m: m.group(1) + '\n' + body + '\n}', content, flags=re.DOTALL)
      if new_content != content:
        content = new_content
        changes += 1
      continue

    pattern = rf'(struct )CustomReserved{slot_num}( @0x[0-9a-f]+ \{{\n)\}}'
    replacement = rf'\g<1>{struct_name}\g<2>{body}\n}}'
    new_content = re.sub(pattern, replacement, content)
    if new_content != content:
      content = new_content
      changes += 1

  for plugin_id, path in standalone_files:
    with open(path) as f:
      standalone_content = f.read().strip()

    first_name = re.search(r'(?:struct|enum)\s+(\w+)', standalone_content)
    if first_name and first_name.group(1) in content:
      continue

    content = content.rstrip("\n") + "\n\n# " + plugin_id + " plugin\n\n" + standalone_content + "\n"
    changes += 1

  if changes:
    with open(custom_capnp, "w") as f:
      f.write(content)

  return changes


def inject_log_capnp(log_capnp: str, slots: dict) -> int:
  """Rename Event union fields in log.capnp to match plugin struct names. Returns count of changes."""
  with open(log_capnp) as f:
    content = f.read()

  changes = 0

  for slot_num, (struct_name, event_field, _) in sorted(slots.items()):
    if event_field in content:
      continue

    # customReservedN @ID :Custom.CustomReservedN;
    pattern = rf'customReserved{slot_num}( @\d+ :Custom\.)CustomReserved{slot_num};'
    replacement = rf'{event_field}\g<1>{struct_name};'
    new_content = re.sub(pattern, replacement, content)
    if new_content != content:
      content = new_content
      changes += 1

  if changes:
    with open(log_capnp, "w") as f:
      f.write(content)

  return changes


def inject_car_capnp(car_capnp: str, safety_models: dict[str, int]) -> int:
  """Inject safety model entries into SafetyModel enum in car.capnp. Returns count of changes."""
  if not safety_models:
    return 0

  with open(car_capnp) as f:
    content = f.read()

  changes = 0
  for name, ordinal in sorted(safety_models.items(), key=lambda x: x[1]):
    if re.search(rf'\b{re.escape(name)}\s+@', content):
      continue

    # Insert before the closing brace of SafetyModel enum
    pattern = r'(  enum SafetyModel \{.*?)(  \})'
    def _insert(m):
      return m.group(1) + f'    {name} @{ordinal};\n' + m.group(2)
    new_content = re.sub(pattern, _insert, content, count=1, flags=re.DOTALL)
    if new_content != content:
      content = new_content
      changes += 1

  if changes:
    with open(car_capnp, "w") as f:
      f.write(content)

  return changes


def inject_event_names(log_capnp: str, event_names: dict[str, int]) -> int:
  """Inject EventName enum entries into log.capnp. Returns count of changes."""
  if not event_names:
    return 0

  with open(log_capnp) as f:
    content = f.read()

  changes = 0
  for name, ordinal in sorted(event_names.items(), key=lambda x: x[1]):
    if re.search(rf'\b{re.escape(name)}\s+@', content):
      continue
    pattern = r'(    soundsUnavailableDEPRECATED @47;\n  \})'
    replacement = rf'    {name} @{ordinal};\n\1'
    new_content = re.sub(pattern, replacement, content)
    if new_content != content:
      content = new_content
      changes += 1

  if changes:
    with open(log_capnp, "w") as f:
      f.write(content)

  return changes


def main():
  if len(sys.argv) < 2:
    print(f"Usage: {sys.argv[0]} <cereal_dir> [plugins_dir]", file=sys.stderr)
    sys.exit(1)

  cereal_dir = sys.argv[1]
  plugins_dir = sys.argv[2] if len(sys.argv) > 2 else PLUGINS_RUNTIME_DIR

  slots, standalone_files, safety_models, event_names = collect_cereal(plugins_dir)
  if not slots and not standalone_files and not safety_models and not event_names:
    print("[custom_capnp] No plugin cereal schemas found")
    return

  custom_capnp = os.path.join(cereal_dir, "custom.capnp")
  log_capnp = os.path.join(cereal_dir, "log.capnp")
  # car.capnp lives in opendbc, derive path relative to cereal dir
  car_capnp = os.path.join(cereal_dir, "..", "opendbc_repo", "opendbc", "car", "car.capnp")

  changes = 0
  if os.path.isfile(custom_capnp):
    changes += inject_custom_capnp(custom_capnp, slots, standalone_files)
  if os.path.isfile(log_capnp):
    changes += inject_log_capnp(log_capnp, slots)
    changes += inject_event_names(log_capnp, event_names)
  if os.path.isfile(car_capnp):
    changes += inject_car_capnp(car_capnp, safety_models)

  if changes:
    print(f"[custom_capnp] Injected {changes} schema(s)")
  else:
    print("[custom_capnp] All plugin schemas already present")


if __name__ == "__main__":
  main()
