#!/usr/bin/env python3
"""Scan plugin manifests and inject cereal schemas into custom.capnp, log.capnp, and car.capnp."""
import glob
import json
import os
import re
import sys
from config import PLUGINS_RUNTIME_DIR


def _parse_cereal_from_manifest(manifest_path: str) -> tuple[str, dict]:
  """Parse cereal config from a plugin manifest. Returns (plugin_id, cereal_dict)."""
  plugin_dir = os.path.dirname(manifest_path)
  try:
    with open(manifest_path) as f:
      data = json.load(f)
  except (json.JSONDecodeError, OSError):
    return "", {}
  return data.get("id", os.path.basename(plugin_dir)), data.get("cereal", {})


def collect_cereal(plugins_dir: str) -> tuple[dict, list[tuple[str, str]], dict[str, int], dict[str, int], dict, list[tuple[str, str]]]:
  """Collect slot definitions, standalone schemas, safety models, and event names from plugins.

  Returns (slots, standalone_files, safety_models, event_names, disabled_slots, disabled_standalone) where:
    slots = {slot_num: (struct_name, event_field, slot_file_path)}  — enabled plugins
    standalone_files = [(plugin_id, path), ...]                     — enabled plugins
    safety_models = {name: ordinal}
    event_names = {name: ordinal}
    disabled_slots = {slot_num: struct_name}                        — disabled plugins (for cleanup)
    disabled_standalone = [(plugin_id, path), ...]                  — disabled plugins (for cleanup)
  """
  slots: dict[int, tuple[str, str, str]] = {}
  standalone: list[tuple[str, str]] = []
  safety_models: dict[str, int] = {}
  event_names: dict[str, int] = {}
  disabled_slots: dict[int, str] = {}
  disabled_standalone: list[tuple[str, str]] = []

  for manifest_path in sorted(glob.glob(os.path.join(plugins_dir, "*/plugin.json"))):
    plugin_dir = os.path.dirname(manifest_path)
    is_disabled = os.path.exists(os.path.join(plugin_dir, ".disabled"))
    plugin_id, cereal = _parse_cereal_from_manifest(manifest_path)
    if not cereal:
      continue

    for slot_num, slot_info in cereal.get("slots", {}).items():
      schema_file = slot_info.get("schema_file", "")
      struct_name = slot_info.get("struct_name", "")
      event_field = slot_info.get("event_field", "")
      if schema_file and struct_name and event_field:
        slot_path = os.path.join(plugin_dir, schema_file)
        if is_disabled:
          disabled_slots[int(slot_num)] = struct_name
        elif os.path.isfile(slot_path):
          slots[int(slot_num)] = (struct_name, event_field, slot_path)

    standalone_schema = cereal.get("standalone_schema", "")
    if standalone_schema:
      standalone_path = os.path.join(plugin_dir, standalone_schema)
      if is_disabled:
        disabled_standalone.append((plugin_id, standalone_path))
      elif os.path.isfile(standalone_path):
        standalone.append((plugin_id, standalone_path))

    if not is_disabled:
      for name, ordinal in cereal.get("safety_models", {}).items():
        safety_models[name] = int(ordinal)
      for name, ordinal in cereal.get("event_names", {}).items():
        event_names[name] = int(ordinal)

  return slots, standalone, safety_models, event_names, disabled_slots, disabled_standalone


def cleanup_disabled_capnp(custom_capnp: str, disabled_slots: dict, disabled_standalone: list[tuple[str, str]]) -> int:
  """Revert schemas from disabled plugins back to empty CustomReservedN stubs. Returns count of changes."""
  with open(custom_capnp) as f:
    content = f.read()

  changes = 0

  # Revert slot structs back to empty CustomReservedN
  for slot_num, struct_name in disabled_slots.items():
    if struct_name not in content:
      continue
    pattern = rf'struct {re.escape(struct_name)}( @0x[0-9a-f]+ \{{)\n.*?\n\}}'
    replacement = rf'struct CustomReserved{slot_num}\1\n}}'
    new_content = re.sub(pattern, replacement, content, flags=re.DOTALL)
    if new_content != content:
      content = new_content
      changes += 1

  # Remove standalone schema blocks from disabled plugins
  for plugin_id, path in disabled_standalone:
    if not os.path.isfile(path):
      continue
    # Find all top-level struct/enum names defined in the standalone file
    with open(path) as f:
      standalone_content = f.read()
    names = re.findall(r'(?:struct|enum)\s+(\w+)', standalone_content)
    for name in names:
      if name not in content:
        continue
      # Remove struct/enum block: "struct Name @0x... {\n...\n}" or "enum Name {\n...\n}"
      pattern = rf'\n*(?:struct|enum) {re.escape(name)}(?: @0x[0-9a-f]+)? \{{[^{{}}]*\}}'
      new_content = re.sub(pattern, '', content, flags=re.DOTALL)
      if new_content != content:
        content = new_content
        changes += 1
    # Also remove the "# plugin_id plugin" comment if present
    comment_pattern = rf'\n*# {re.escape(plugin_id)} plugin\n*'
    new_content = re.sub(comment_pattern, '\n', content)
    if new_content != content:
      content = new_content

  if changes:
    with open(custom_capnp, "w") as f:
      f.write(content)

  return changes


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


def cleanup_disabled_log_capnp(log_capnp: str, disabled_slots: dict) -> int:
  """Revert Event union fields from disabled plugins back to customReservedN. Returns count of changes."""
  with open(log_capnp) as f:
    content = f.read()

  changes = 0
  for slot_num, struct_name in disabled_slots.items():
    if struct_name not in content:
      continue
    # Find the plugin's event field and revert: eventField @ID :Custom.StructName → customReservedN @ID :Custom.CustomReservedN
    pattern = rf'(\w+)( @\d+ :Custom\.){re.escape(struct_name)};'
    match = re.search(pattern, content)
    if match and match.group(1) != f'customReserved{slot_num}':
      replacement = rf'customReserved{slot_num}\g<2>CustomReserved{slot_num};'
      new_content = re.sub(pattern, replacement, content)
      if new_content != content:
        content = new_content
        changes += 1

  if changes:
    with open(log_capnp, "w") as f:
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
    existing = re.search(rf'\b{re.escape(name)}\s+@(\d+);', content)
    if existing:
      if int(existing.group(1)) == ordinal:
        continue
      # Ordinal changed — update in place
      new_content = re.sub(rf'\b{re.escape(name)}\s+@\d+;', f'{name} @{ordinal};', content)
      if new_content != content:
        content = new_content
        changes += 1
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

  slots, standalone_files, safety_models, event_names, disabled_slots, disabled_standalone = collect_cereal(plugins_dir)

  custom_capnp = os.path.join(cereal_dir, "custom.capnp")
  log_capnp = os.path.join(cereal_dir, "log.capnp")
  # car.capnp lives in opendbc, derive path relative to cereal dir
  car_capnp = os.path.join(cereal_dir, "..", "opendbc_repo", "opendbc", "car", "car.capnp")

  changes = 0
  # Clean up schemas from disabled plugins first (custom.capnp + log.capnp)
  if disabled_slots or disabled_standalone:
    cleaned = 0
    if os.path.isfile(custom_capnp):
      cleaned += cleanup_disabled_capnp(custom_capnp, disabled_slots, disabled_standalone)
    if os.path.isfile(log_capnp) and disabled_slots:
      cleaned += cleanup_disabled_log_capnp(log_capnp, disabled_slots)
    if cleaned:
      print(f"[custom_capnp] Cleaned {cleaned} schema(s) from disabled plugins")
      changes += cleaned

  if not slots and not standalone_files and not safety_models and not event_names:
    if not changes:
      print("[custom_capnp] No plugin cereal schemas found")
    return

  # Inject schemas from enabled plugins
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
