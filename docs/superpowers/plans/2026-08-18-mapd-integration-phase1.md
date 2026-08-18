# mapd Integration Phase 1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Bring mapd v2.3.0 up on the device and have speedlimitd observe its road-context output as telemetry only, leaving actuation byte-identical to today.

**Architecture:** Align the mapd cereal bus schema (slots 17–19) to mapd v2.3.0, register the mapd plugin with the framework so plugind runs the binary, and add a pure stateless adapter in speedlimitd that turns a `mapdOut` message into observation fields. That output is published to `pluginBusLog` and nothing else — the offline tile reader still drives every control path, so actuation is unchanged.

**Tech Stack:** Python 3.12, Cap'n Proto (pycapnp), pytest, openpilot cereal/msgq, Go binary from pfeiferj/mapd.

**Spec:** `docs/superpowers/specs/2026-08-18-mapd-integration-design.md`

## Global Constraints

- **Actuation must remain byte-identical.** No value derived from `mapdOut` may reach any control path, param, or displayed value in Phase 1. It goes to `pluginBusLog` and nowhere else.
- **Schema alignment lands before the version bump.** Task 1 must be complete before Task 2. A v2.3.0 binary publishing into the current 24-field `MapdOut` silently drops `highwayClass`.
- `HighwayClass` enum members must be copied **verbatim** from mapd v2.3.0 — same names, same ordinals. Upstream `state.go` casts directly between the generated enum types.
- The `mapdOut` service frequency must be **20.0**, matching mapd's `LOOP_DELAY = 50 * time.Millisecond` in `settings/const.go`. A mismatch makes `sm.valid['mapdOut']` false permanently.
- `highwayClass` maps to **underscore OSM strings** (`motorway_link`, `living_street`), never capnp camelCase. These are compared against `URBAN_ONLY_TYPES` and used as speed-table keys at `speedlimitd.py:556` and `:585`.
- `pytest.importorskip('capnp')` may only be scoped to capnp-dependent tests, never at module scope. A file-level gate silently skipped 18 tests in the pre-push venv (fixed in `a0d07ea`).
- Run all suites with `PYTHONPATH=` — a foreign `PYTHONPATH` in the session env hijacks the namespace package and tests the wrong worktree.
- No `Co-Authored-By` lines in commit messages.
- Two-space indentation in Python, matching the existing plugin code.

**Test baseline:** `PYTHONPATH= python3 -m pytest plugins -q` → `3 failed, 740 passed, 21 skipped`. The 3 failures are pre-existing and unrelated: `plugins/bmw_e9x_e8x/tools/dcc_study/tests/test_extract.py` fails with `No module named 'cereal'` (dev-machine env gap; the pre-push hook's `plugins/*/tests/` glob does not collect them). Do not attempt to fix them.

---

## File Structure

**Created:**
- `plugins/mapd/tests/test_slot_schemas.py` — bus schema roundtrip and ordinal verification
- `plugins/mapd/tests/test_manifest.py` — manifest registration and version pin
- `plugins/mapd/mapd_defaults.json` — declarative mapd settings, data-source-only
- `plugins/speedlimitd/mapd_source.py` — pure `mapdOut` → telemetry adapter (Phase 1 scope)
- `plugins/speedlimitd/tests/test_mapd_source.py` — adapter unit tests

**Modified:**
- `plugins/mapd/cereal/standalone.capnp` — add `MapdPosition`, `HighwayClass`, 8 `MapdInputType` values
- `plugins/mapd/cereal/slot17.capnp` — add `position @3`
- `plugins/mapd/cereal/slot18.capnp` — add `jsonPath @4`
- `plugins/mapd/cereal/slot19.capnp` — add `highwayClass @24`, `wayId @25`, `conditionalSpeedLimit @26`
- `plugins/mapd/plugin.json` — cereal slots, service, process, health hook, version default
- `plugins/mapd/mapd_manager.py:26` — `MAX_ALLOWED_VERSION` → `v2.3.0`
- `plugins/mapd/mapd_runner.py` — drop control-related settings writing; add bounded binary retry
- `plugins/mapd/README.md` — status, settings, version rationale
- `install.sh` — uncomment mapd `.enforced`, place `mapd_defaults.json`
- `plugins/speedlimitd/speedlimitd.py:735` — subscribe `mapdOut`; publish `mapd*` telemetry
- `plugins/speedlimitd/tests/test_speedlimitd.py` — telemetry tests, sys.path insert
- `plugins/speedlimitd/DESIGN.md` — note the Phase 1 observation path

**Responsibility split:** `mapd_source.py` owns all mapd→speedlimitd translation (enum mapping, unit conversion, field shaping). `speedlimitd.py` owns only subscription and publication. Nothing else changes.

---

### Task 1: Align the mapd bus schema to v2.3.0

**Files:**
- Modify: `plugins/mapd/cereal/standalone.capnp`
- Modify: `plugins/mapd/cereal/slot17.capnp`
- Modify: `plugins/mapd/cereal/slot18.capnp`
- Modify: `plugins/mapd/cereal/slot19.capnp`
- Test: `plugins/mapd/tests/test_slot_schemas.py` (create)

**Interfaces:**
- Consumes: nothing from earlier tasks.
- Produces: the `MapdOut` struct fields `highwayClass` (enum `HighwayClass`), `wayId` (`Int64`), `conditionalSpeedLimit` (`Text`); the `HighwayClass` enum with members `unknown, motorway, motorwayLink, trunk, trunkLink, primary, primaryLink, secondary, secondaryLink, tertiary, tertiaryLink, unclassified, residential, livingStreet` at ordinals 0–13. Task 4 maps these to OSM strings.

**Background the implementer needs:** The `slotN.capnp` files are **fragments** — struct *bodies* with no `struct X { }` wrapper. `install.sh` runs `plugins/custom_capnp.py`, which injects them into openpilot's `cereal/custom.capnp` in place of the `CustomReservedN` stubs. To parse them standalone, the test reassembles a valid schema file: `standalone.capnp` plus each fragment wrapped in its struct declaration.

- [ ] **Step 1: Write the failing test**

Create `plugins/mapd/tests/test_slot_schemas.py`:

```python
"""Bus schema tests — mapd slots 17-19 must match mapd v2.3.0's custom.capnp.

The slotN.capnp files are FRAGMENTS (struct bodies with no wrapper):
install.sh's custom_capnp.py injects them into openpilot's cereal/custom.capnp
in place of the CustomReservedN stubs. To parse them standalone this module
reassembles a valid schema file — standalone.capnp plus each fragment wrapped
in its struct declaration — and loads that.
"""
import os

import pytest

capnp = pytest.importorskip('capnp')

CEREAL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'cereal')

# capnp requires a file ID. Arbitrary but fixed; never persisted anywhere.
FILE_ID = '@0xbdc0e1e5a4c9f8d2;'

SLOTS = ((17, 'MapdExtendedOut'), (18, 'MapdIn'), (19, 'MapdOut'))

# HighwayClass, copied verbatim from mapd v2.3.0 cereal/custom/custom.capnp.
# Upstream requires this to stay name- and value-identical to
# cereal/offline/offline.capnp because state.go casts between the generated
# enum types. Any drift here is a silent mis-classification on device.
EXPECTED_HIGHWAY_CLASS = {
  'unknown': 0,
  'motorway': 1,
  'motorwayLink': 2,
  'trunk': 3,
  'trunkLink': 4,
  'primary': 5,
  'primaryLink': 6,
  'secondary': 7,
  'secondaryLink': 8,
  'tertiary': 9,
  'tertiaryLink': 10,
  'unclassified': 11,
  'residential': 12,
  'livingStreet': 13,
}


@pytest.fixture(scope='module')
def schema(tmp_path_factory):
  """Reassemble standalone.capnp + wrapped slot fragments into one loadable file."""
  parts = [FILE_ID]
  with open(os.path.join(CEREAL_DIR, 'standalone.capnp')) as f:
    parts.append(f.read())
  for num, struct_name in SLOTS:
    with open(os.path.join(CEREAL_DIR, f'slot{num}.capnp')) as f:
      body = f.read()
    parts.append(f'struct {struct_name} {{\n{body}}}\n')
  merged = tmp_path_factory.mktemp('capnp') / 'merged.capnp'
  merged.write_text('\n'.join(parts))
  return capnp.load(str(merged))


class TestMapdOut:
  def test_has_v230_fields(self, schema):
    fields = schema.MapdOut.schema.fieldnames
    assert 'highwayClass' in fields
    assert 'wayId' in fields
    assert 'conditionalSpeedLimit' in fields

  def test_preexisting_fields_unmoved(self, schema):
    # Additions only — the v2.0.5 fields keep their ordinals, so an older
    # binary still round-trips against this schema.
    fields = schema.MapdOut.schema.fieldnames
    assert fields[0] == 'wayName'
    assert fields[23] == 'speedLimitAccepted'

  def test_new_fields_are_appended_in_order(self, schema):
    fields = schema.MapdOut.schema.fieldnames
    assert fields[24:27] == ('highwayClass', 'wayId', 'conditionalSpeedLimit')

  def test_roundtrip_new_fields(self, schema):
    msg = schema.MapdOut.new_message()
    msg.wayRef = 'S20'
    msg.highwayClass = 'motorway'
    msg.wayId = 123456789
    msg.conditionalSpeedLimit = '100 @ (Mo-Fr 06:00-20:00)'
    with schema.MapdOut.from_bytes(msg.to_bytes()) as out:
      assert out.wayRef == 'S20'
      assert out.highwayClass == 'motorway'
      assert out.wayId == 123456789
      assert out.conditionalSpeedLimit == '100 @ (Mo-Fr 06:00-20:00)'


class TestHighwayClass:
  def test_members_match_upstream_exactly(self, schema):
    assert dict(schema.HighwayClass.schema.enumerants) == EXPECTED_HIGHWAY_CLASS


class TestMapdIn:
  def test_has_json_path(self, schema):
    assert 'jsonPath' in schema.MapdIn.schema.fieldnames

  def test_json_path_roundtrip(self, schema):
    msg = schema.MapdIn.new_message()
    msg.jsonPath = 'speed_limit.offset'
    with schema.MapdIn.from_bytes(msg.to_bytes()) as out:
      assert out.jsonPath == 'speed_limit.offset'


class TestMapdExtendedOut:
  def test_has_position(self, schema):
    assert 'position' in schema.MapdExtendedOut.schema.fieldnames

  def test_position_roundtrip(self, schema):
    msg = schema.MapdExtendedOut.new_message()
    msg.position.latitude = 31.3137
    msg.position.longitude = 121.5395
    with schema.MapdExtendedOut.from_bytes(msg.to_bytes()) as out:
      assert out.position.latitude == pytest.approx(31.3137)
      assert out.position.longitude == pytest.approx(121.5395)


class TestMapdInputType:
  def test_has_v230_setters(self, schema):
    members = dict(schema.MapdInputType.schema.enumerants)
    assert members['setConditionalSpeedLimitControl'] == 39
    assert members['setShadowCarState'] == 40
    assert members['setShadowModelV2'] == 41
    assert members['setShadowGpsLocation'] == 42
    assert members['setJsonPathFloat'] == 43
    assert members['setJsonPathText'] == 44
    assert members['setJsonPathBool'] == 45
    assert members['setShadowGpsLocationExternal'] == 46
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `PYTHONPATH= python3 -m pytest plugins/mapd/tests/test_slot_schemas.py -q`

Expected: FAIL. `test_has_v230_fields` fails on `'highwayClass' in fields`; `TestHighwayClass` fails because the enum does not exist yet (`AttributeError` on `schema.HighwayClass`).

- [ ] **Step 3: Add `MapdPosition` and `HighwayClass` to `standalone.capnp`**

Insert the `MapdPosition` struct immediately after the existing `MapdPathPoint` struct. The type ID is copied from upstream:

```capnp
struct MapdPosition @0xde9705979aca8339 {
  latitude @0 :Float64;
  longitude @1 :Float64;
}
```

Insert the `HighwayClass` enum immediately after the existing `RoadContext` enum, at the end of the file:

```capnp
# WARNING: must be kept in perfect sync (names and values) with the
# HighwayClass enum in mapd's cereal/offline/offline.capnp — state.go casts
# directly between the two generated enum types.
# unknown either means the way's highway tag was not one of the listed values
# or the loaded map tiles predate this field.
enum HighwayClass {
  unknown @0;
  motorway @1;
  motorwayLink @2;
  trunk @3;
  trunkLink @4;
  primary @5;
  primaryLink @6;
  secondary @7;
  secondaryLink @8;
  tertiary @9;
  tertiaryLink @10;
  unclassified @11;
  residential @12;
  livingStreet @13;
}
```

- [ ] **Step 4: Add the new `MapdInputType` members**

In `standalone.capnp`, inside the existing `enum MapdInputType`, append after `setPressGasToOverrideSpeedLimit @38;`:

```capnp
  setConditionalSpeedLimitControl @39;
  setShadowCarState @40;
  setShadowModelV2 @41;
  setShadowGpsLocation @42;
  setJsonPathFloat @43;
  setJsonPathText @44;
  setJsonPathBool @45;
  setShadowGpsLocationExternal @46;
```

- [ ] **Step 5: Extend the three slot fragments**

`plugins/mapd/cereal/slot17.capnp` — append one line:

```capnp
  position @3 :MapdPosition;
```

`plugins/mapd/cereal/slot18.capnp` — append one line:

```capnp
  jsonPath @4 :Text;
```

`plugins/mapd/cereal/slot19.capnp` — append three lines:

```capnp
  highwayClass @24 :HighwayClass;
  wayId @25 :Int64;
  conditionalSpeedLimit @26 :Text;
```

- [ ] **Step 6: Run the test to verify it passes**

Run: `PYTHONPATH= python3 -m pytest plugins/mapd/tests/test_slot_schemas.py -q`

Expected: PASS, 10 passed.

- [ ] **Step 7: Run the full suite**

Run: `PYTHONPATH= python3 -m pytest plugins -q`

Expected: `3 failed, 750 passed, 21 skipped` — the same 3 pre-existing `dcc_study` failures, 10 new passes.

- [ ] **Step 8: Commit**

```bash
git add plugins/mapd/cereal/ plugins/mapd/tests/test_slot_schemas.py
git commit -m "feat(mapd): align bus schema to v2.3.0 — highwayClass, wayId, conditional limits"
```

---

### Task 2: Register mapd with the plugin framework

**Files:**
- Modify: `plugins/mapd/plugin.json`
- Modify: `plugins/mapd/mapd_manager.py:24-26`
- Modify: `install.sh:355-359`
- Modify: `plugins/mapd/README.md`
- Test: `plugins/mapd/tests/test_manifest.py` (create)

**Interfaces:**
- Consumes: the slot files from Task 1 (referenced by path from the manifest's `cereal.slots`).
- Produces: the `mapdOut` service registered at 20 Hz, so Task 5's `SubMaster(['mapdOut'])` resolves; the `mapd` standalone process, so plugind spawns the binary.

**Background the implementer needs:** `plugins/custom_capnp.py` reads each manifest's `cereal.slots` map to know which fragment goes in which `CustomReservedN` slot, and `cereal.standalone_schema` for shared structs/enums. `plugins/services.py` reads `services` and injects entries into `cereal/services.py` as `tuple(entry)`. Follow `plugins/bus_logger/plugin.json` as the reference for both shapes. plugind stores a process's `condition` (`registry.py:257`) but never evaluates it — every declared process runs and is respawned when it dies.

- [ ] **Step 1: Write the failing test**

Create `plugins/mapd/tests/test_manifest.py`:

```python
"""Manifest registration — mapd must declare its slots, service, process and hook."""
import json
import os
import re

PLUGIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _manifest():
  with open(os.path.join(PLUGIN_DIR, 'plugin.json')) as f:
    return json.load(f)


class TestCerealSlots:
  def test_declares_all_three_slots(self):
    slots = _manifest()['cereal']['slots']
    assert set(slots) == {'17', '18', '19'}

  def test_slot_struct_and_event_names(self):
    slots = _manifest()['cereal']['slots']
    assert slots['17']['struct_name'] == 'MapdExtendedOut'
    assert slots['17']['event_field'] == 'mapdExtendedOut'
    assert slots['18']['struct_name'] == 'MapdIn'
    assert slots['18']['event_field'] == 'mapdIn'
    assert slots['19']['struct_name'] == 'MapdOut'
    assert slots['19']['event_field'] == 'mapdOut'

  def test_slot_schema_files_exist(self):
    for num, info in _manifest()['cereal']['slots'].items():
      path = os.path.join(PLUGIN_DIR, info['schema_file'])
      assert os.path.isfile(path), f'slot {num} schema missing: {path}'

  def test_standalone_schema_declared_and_exists(self):
    schema = _manifest()['cereal']['standalone_schema']
    assert os.path.isfile(os.path.join(PLUGIN_DIR, schema))


class TestService:
  def test_mapd_out_registered(self):
    assert 'mapdOut' in _manifest()['services']

  def test_frequency_matches_mapd_loop_rate(self):
    # mapd's main.go sleeps settings.LOOP_DELAY = 50ms, i.e. 20 Hz. SubMaster
    # validity is checked against this number: a mismatch makes
    # sm.valid['mapdOut'] false forever and degrades us to VISION permanently.
    entry = _manifest()['services']['mapdOut']
    assert entry[0] is True          # logged
    assert entry[1] == 20.0          # frequency (Hz)
    assert entry[2] == 20            # decimation


class TestProcess:
  def test_mapd_process_declared(self):
    procs = {p['name']: p for p in _manifest()['processes']}
    assert 'mapd' in procs
    assert procs['mapd']['module'] == 'mapd_runner'


class TestHooks:
  def test_health_check_registered(self):
    hook = _manifest()['hooks']['device.health_check']
    assert hook['module'] == 'hook'
    assert hook['function'] == 'on_health_check'


class TestVersionPin:
  def test_manifest_default_is_v230(self):
    assert _manifest()['params']['MapdVersion']['default'] == 'v2.3.0'

  def test_manager_max_allowed_is_v230(self):
    with open(os.path.join(PLUGIN_DIR, 'mapd_manager.py')) as f:
      src = f.read()
    match = re.search(r'^MAX_ALLOWED_VERSION\s*=\s*"([^"]+)"', src, re.M)
    assert match is not None
    assert match.group(1) == 'v2.3.0'
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `PYTHONPATH= python3 -m pytest plugins/mapd/tests/test_manifest.py -q`

Expected: FAIL. `TestCerealSlots` raises `KeyError: 'slots'` (manifest has `"cereal": {}`); `TestService` raises `KeyError: 'mapdOut'`; `TestVersionPin` fails on `'v2.0.5' != 'v2.3.0'`.

- [ ] **Step 3: Update the manifest**

In `plugins/mapd/plugin.json`, replace the `"cereal": {}`, `"services": {}`, `"hooks": {}`, `"processes": []` entries and the `MapdVersion` default:

```json
  "cereal": {
    "slots": {
      "17": {
        "struct_name": "MapdExtendedOut",
        "event_field": "mapdExtendedOut",
        "schema_file": "cereal/slot17.capnp"
      },
      "18": {
        "struct_name": "MapdIn",
        "event_field": "mapdIn",
        "schema_file": "cereal/slot18.capnp"
      },
      "19": {
        "struct_name": "MapdOut",
        "event_field": "mapdOut",
        "schema_file": "cereal/slot19.capnp"
      }
    },
    "standalone_schema": "cereal/standalone.capnp"
  },
  "services": {
    "mapdOut": [true, 20.0, 20]
  },
  "panel": false,
  "hooks": {
    "device.health_check": {
      "module": "hook",
      "function": "on_health_check",
      "priority": 50
    }
  },
  "processes": [
    {
      "name": "mapd",
      "module": "mapd_runner",
      "condition": "always_run"
    }
  ],
  "params": {
    "MapdVersion": {
      "type": "string",
      "default": "v2.3.0",
      "label": "Mapd Version"
    }
  }
```

- [ ] **Step 4: Bump the version pin**

In `plugins/mapd/mapd_manager.py`, replace lines 24-26:

```python
# Pinned to the release whose MapdOut shape matches cereal/slot19.capnp.
# BUMPING THIS REQUIRES A SCHEMA REVIEW: capnp is additive, so a newer binary
# publishing into an older slot19 silently DROPS its new fields — highwayClass
# would read as 'unknown' and speedlimitd would mis-classify every road with no
# error anywhere. Check mapd's cereal/custom/custom.capnp MapdOut against
# cereal/slot19.capnp before changing this.
MAX_ALLOWED_VERSION = "v2.3.0"
```

- [ ] **Step 5: Enable the plugin in install.sh**

In `install.sh`, replace the commented block at lines 355-359:

```bash
# mapd is the sole OSM road-context provider (speedlimitd consumes mapdOut).
# Enforced so a stale .disabled cannot silently leave the car with no map data.
# Was pinned off until v2.3.0: v2.0.6-v2.2.0 hardcoded a slotless ("shadow")
# carState subscription whose torn reads panic gomsgq. v2.3.0 made shadow a
# per-queue setting — see mapd_defaults.json.
if [[ -d "$PLUGINS_DEST/mapd" ]]; then
  touch "$PLUGINS_DEST/mapd/.enforced"
  rm -f "$PLUGINS_DEST/mapd/.disabled"
fi
```

- [ ] **Step 6: Run the test to verify it passes**

Run: `PYTHONPATH= python3 -m pytest plugins/mapd/tests/test_manifest.py -q`

Expected: PASS, 10 passed.

- [ ] **Step 7: Update the README**

In `plugins/mapd/README.md`, replace the `**Status: disabled — not currently running.**` line with:

```markdown
**Status: enabled — mapd is the sole provider of OSM road context.**

speedlimitd consumes the `mapdOut` message; it no longer reads map tiles
itself. Tiles are downloaded by COD's web UI into `/data/media/0/osm/offline/`,
which is exactly where mapd reads them.
```

And replace the "The v2.0.5 pin is deliberate…" paragraph with:

```markdown
The v2.3.0 pin is deliberate, for two independent reasons.

**Schema coupling.** Cap'n Proto is additive, so a newer binary publishing into
our `cereal/slot19.capnp` silently drops any field we have not declared —
`highwayClass` would read as `unknown` and speedlimitd would mis-classify every
road, with no error anywhere. Bumping the pin requires diffing mapd's
`cereal/custom/custom.capnp` against our slot files first.

**Shadow subscribers.** v2.0.6 through v2.2.0 hardcoded a slotless ("shadow")
`carState` subscription: it reads the msgq ring buffer without claiming a
reader slot, so the writer can overwrite the region mid-read and gomsgq panics
on the torn size field (pfeiferj/mapd#88). v2.3.0 made shadow a per-queue
setting. We keep upstream's default of shadow-on for carState — it consumes no
reader slot — and rely on plugind respawning mapd, with speedlimitd degrading
to vision-only meanwhile.
```

- [ ] **Step 8: Run the full suite**

Run: `PYTHONPATH= python3 -m pytest plugins -q`

Expected: `3 failed, 760 passed, 21 skipped`.

- [ ] **Step 9: Commit**

```bash
git add plugins/mapd/plugin.json plugins/mapd/mapd_manager.py plugins/mapd/README.md \
        plugins/mapd/tests/test_manifest.py install.sh
git commit -m "feat(mapd): register slots, mapdOut service, process and health hook; pin v2.3.0"
```

---

### Task 3: Declarative mapd settings and a bounded binary retry

**Files:**
- Create: `plugins/mapd/mapd_defaults.json`
- Modify: `plugins/mapd/mapd_runner.py`
- Modify: `install.sh` (place the defaults file)
- Test: `plugins/mapd/tests/test_manifest.py` (extend — add `TestDefaults`, `TestRunnerSettings`, `TestRunnerRetry`)

**Interfaces:**
- Consumes: nothing from earlier tasks.
- Produces: `/data/openpilot/mapd_defaults.json` on device, pinning mapd to data-source-only behaviour. No Python interface.

**Why the retry belongs here:** plugind respawns any dead process every `POLL_INTERVAL = 5.0` seconds (`plugind.py:31`). `mapd_runner` currently exits 1 the moment `ensure_binary()` fails, so a device with no network would exec the runner 12×/minute forever, each attempt hitting the GitHub releases API and writing a log line. That was harmless while the plugin was disabled and goes live the moment Task 2 enables it. This task already rewrites `mapd_runner.py`, so the backoff lands with it.

**Background the implementer needs:** mapd loads settings in three layers — built-in defaults, then `/data/openpilot/mapd_defaults.json` if present, then the `MapdSettings` param. openpilot wipes `/data/params/d/` on boot, which is why `mapd_runner._ensure_mapd_settings()` currently rewrites `MapdSettings` on every start. Moving configuration to the defaults file makes that unnecessary: the file is untracked in the catpilot repo so `git reset --hard` leaves it alone, and install.sh re-places it if `git clean` removes it. v2.3.0 uses `settings_version: 2` with nested sub-objects.

- [ ] **Step 1: Write the failing test**

Append to `plugins/mapd/tests/test_manifest.py`:

```python
class TestDefaults:
  """mapd_defaults.json pins mapd to data-source-only behaviour.

  Every control feature must be off: speedlimitd owns the speed target and the
  lateral-accel budget (the layer contract). mapd supplies road context only.
  """

  @staticmethod
  def _defaults():
    with open(os.path.join(PLUGIN_DIR, 'mapd_defaults.json')) as f:
      return json.load(f)

  def test_settings_version_is_2(self):
    assert self._defaults()['settings_version'] == 2

  def test_all_control_features_disabled(self):
    d = self._defaults()
    assert d['speed_limit_control_enabled'] is False
    assert d['map_curve_speed_control_enabled'] is False
    assert d['vision_curve_speed_control_enabled'] is False
    assert d['external_speed_limit_control_enabled'] is False
    assert d['conditional_speed_limit_control_enabled'] is False

  def test_car_state_stays_shadow(self):
    # Upstream default. Shadow consumes no reader slot; the torn-read panic is
    # contained by plugind respawn plus speedlimitd degrading to vision-only.
    assert self._defaults()['subscriber']['shadow_car_state'] is True


class TestRunnerSettings:
  def test_runner_no_longer_writes_control_settings(self):
    """Control config lives in mapd_defaults.json, not in runner-written MapdSettings.

    Leaving both in place would let a stale MapdSettings silently re-enable
    mapd's control features, since the param layer is applied last.
    """
    with open(os.path.join(PLUGIN_DIR, 'mapd_runner.py')) as f:
      src = f.read()
    for key in ('speed_limit_control_enabled',
                'map_curve_speed_control_enabled',
                'vision_curve_speed_control_enabled',
                'map_curve_target_lat_a',
                'vision_curve_target_lat_a'):
      assert key not in src, f'{key} should now come from mapd_defaults.json'


class TestRunnerRetry:
  """plugind respawns dead processes every POLL_INTERVAL (5 s).

  Without an internal backoff, a device with no network would exec the runner
  12x/minute forever, each attempt hitting the GitHub releases API.
  """

  @staticmethod
  def _load_runner(monkeypatch, ensure_results):
    """Import mapd_runner with a stub mapd_manager (the real one needs config)."""
    import importlib
    import types
    calls = []

    fake_manager = types.ModuleType('mapd_manager')

    def ensure_binary():
      calls.append(1)
      idx = len(calls) - 1
      return ensure_results[idx] if idx < len(ensure_results) else False

    fake_manager.ensure_binary = ensure_binary
    fake_manager.MAPD_PATH = '/tmp/fake-mapd'
    monkeypatch.setitem(sys.modules, 'mapd_manager', fake_manager)
    if PLUGIN_DIR not in sys.path:
      sys.path.insert(0, PLUGIN_DIR)
    import mapd_runner
    importlib.reload(mapd_runner)
    return mapd_runner, calls

  def test_execs_immediately_when_binary_ready(self, monkeypatch):
    runner, calls = self._load_runner(monkeypatch, [True])
    execed = []
    monkeypatch.setattr(runner.os, 'execv', lambda p, a: execed.append(p))
    monkeypatch.setattr(runner.time, 'sleep', lambda s: pytest.fail('should not sleep'))
    runner.main()
    assert execed == ['/tmp/fake-mapd']
    assert len(calls) == 1

  def test_retries_with_backoff_then_succeeds(self, monkeypatch):
    runner, calls = self._load_runner(monkeypatch, [False, False, True])
    execed, slept = [], []
    monkeypatch.setattr(runner.os, 'execv', lambda p, a: execed.append(p))
    monkeypatch.setattr(runner.time, 'sleep', lambda s: slept.append(s))
    runner.main()
    assert execed == ['/tmp/fake-mapd']
    assert slept == list(runner.RETRY_DELAYS[:2])   # backoff grows between tries

  def test_gives_up_after_all_delays(self, monkeypatch):
    runner, calls = self._load_runner(monkeypatch, [])
    slept = []
    monkeypatch.setattr(runner.os, 'execv', lambda p, a: pytest.fail('should not exec'))
    monkeypatch.setattr(runner.time, 'sleep', lambda s: slept.append(s))
    with pytest.raises(SystemExit) as exc:
      runner.main()
    assert exc.value.code == 1
    assert slept == list(runner.RETRY_DELAYS)
    assert len(calls) == len(runner.RETRY_DELAYS)

  def test_backoff_is_monotonic_and_bounded(self, monkeypatch):
    runner, _ = self._load_runner(monkeypatch, [True])
    delays = runner.RETRY_DELAYS
    assert list(delays) == sorted(delays)
    # Total in-process backoff stays well under a plugind poll storm but long
    # enough that a booting device with slow DHCP is not thrashing the API.
    assert 60 <= sum(delays) <= 600
```

The test module needs `import sys` and `import pytest` at its top — add them if not already present.

- [ ] **Step 2: Run the test to verify it fails**

Run: `PYTHONPATH= python3 -m pytest plugins/mapd/tests/test_manifest.py -q`

Expected: FAIL. `TestDefaults` raises `FileNotFoundError` for `mapd_defaults.json`; `TestRunnerSettings` fails because `speed_limit_control_enabled` is still in `mapd_runner.py`; `TestRunnerRetry` fails with `AttributeError: module 'mapd_runner' has no attribute 'RETRY_DELAYS'`.

- [ ] **Step 3: Create the defaults file**

Create `plugins/mapd/mapd_defaults.json`:

```json
{
  "settings_version": 2,
  "speed_limit_control_enabled": false,
  "map_curve_speed_control_enabled": false,
  "vision_curve_speed_control_enabled": false,
  "external_speed_limit_control_enabled": false,
  "conditional_speed_limit_control_enabled": false,
  "subscriber": {
    "shadow_car_state": true,
    "shadow_model_v2": false,
    "shadow_gps_location": false,
    "shadow_gps_location_external": false,
    "shadow_selfdrive_state": false
  }
}
```

- [ ] **Step 4: Rewrite the runner**

Replace the entire contents of `plugins/mapd/mapd_runner.py` with:

```python
#!/usr/bin/env python3
"""
Mapd process entry point for plugin system.
Ensures the mapd binary exists and execs it.

Settings are NOT written here. mapd loads its built-in defaults, then
/data/openpilot/mapd_defaults.json (placed by install.sh), then the
MapdSettings param. Keeping our configuration in the defaults file means it
survives openpilot wiping /data/params/d/ on boot without this process
rewriting it on every start — and leaves exactly one place that decides
whether mapd controls anything. It does not: see mapd_defaults.json.
"""
import os
import sys
import time

# plugind respawns any dead process every POLL_INTERVAL (5 s). Exiting
# immediately on a failed download would therefore hammer the GitHub releases
# API 12x/minute on a device with no network. Back off in-process instead, then
# exit and let plugind schedule the next round.
RETRY_DELAYS = (5, 15, 60, 180)


def main():
  from mapd_manager import ensure_binary, MAPD_PATH
  for attempt, delay in enumerate(RETRY_DELAYS, start=1):
    if ensure_binary():
      os.execv(str(MAPD_PATH), [str(MAPD_PATH)])
    print(f"mapd binary unavailable (attempt {attempt}/{len(RETRY_DELAYS)}), "
          f"retrying in {delay}s", file=sys.stderr)
    time.sleep(delay)
  print("ERROR: Failed to ensure mapd binary after retries, exiting", file=sys.stderr)
  sys.exit(1)


if __name__ == "__main__":
  main()
```

This deletes `LAT_ACCEL_VALUES`, `_read_speedlimitd_param`, `_ensure_mapd_settings`, and the `import json` / `from config import PARAMS_DIR, plugin_data_dir` imports. Dropping the `config` import is also what lets the retry tests import this module without the plugin runtime on `sys.path`.

- [ ] **Step 5: Place the file from install.sh**

In `install.sh`, immediately after the mapd `.enforced` block added in Task 2, add:

```bash
  # mapd reads its custom defaults from a fixed path in the openpilot repo
  # root. Untracked there, so `git reset --hard` leaves it alone; re-placed
  # here on every install in case `git clean` removed it.
  if [[ -f "$PLUGINS_DEST/mapd/mapd_defaults.json" ]]; then
    cp "$PLUGINS_DEST/mapd/mapd_defaults.json" "$OPENPILOT_ROOT/mapd_defaults.json"
  fi
```

Place it inside the `if [[ -d "$PLUGINS_DEST/mapd" ]]; then` block from Task 2, before its `fi`.

- [ ] **Step 6: Run the test to verify it passes**

Run: `PYTHONPATH= python3 -m pytest plugins/mapd/tests/test_manifest.py -q`

Expected: PASS, 18 passed (10 from Task 2, 8 new).

- [ ] **Step 7: Run the full suite**

Run: `PYTHONPATH= python3 -m pytest plugins -q`

Expected: `3 failed, 768 passed, 21 skipped`.

Note: `plugins/mapd/tests/test_mapd_manager.py` may reference the removed runner helpers. If it fails, update it to match the new `mapd_runner.py` surface — do not re-add the deleted functions.

- [ ] **Step 8: Commit**

```bash
git add plugins/mapd/mapd_defaults.json plugins/mapd/mapd_runner.py \
        plugins/mapd/tests/test_manifest.py install.sh
git commit -m "feat(mapd): declarative defaults — data-source-only, shadow carState"
```

---

### Task 4: speedlimitd mapd_source adapter

**Files:**
- Create: `plugins/speedlimitd/mapd_source.py`
- Test: `plugins/speedlimitd/tests/test_mapd_source.py` (create)

**Interfaces:**
- Consumes: the `MapdOut` field names from Task 1 (`wayRef`, `speedLimit`, `lanes`, `highwayClass`, `wayId`, `tileLoaded`, `distanceFromWayCenter`, `waySelectionType`).
- Produces, for Task 5:
  - `highway_class_name(value) -> str`
  - `telemetry_from_mapd(mapd_out, valid: bool, our_way_ref: str) -> dict`

**Scope note — do not add more than this.** Phase 2 will need a `result_from_mapd` adapter that rebuilds the `osm_query`-shaped dict, plus a `tiles_missing` helper and a `roadContext` classifier. **None of those belong in Phase 1**, because nothing in Phase 1 calls them: Phase 1 observes and publishes, it does not feed `_ingest_osm_result`. Writing them now would ship untested-in-anger code and pre-commit to a shape the Phase 1 drive might invalidate. The design spec records their intended signatures; implement them in Phase 2 against real telemetry.

**Background the implementer needs:** pycapnp returns an enum field as its **camelCase enumerant name string**, so `highway_class_name` takes that form and no other — do not add an integer branch, nothing produces one. This module is pure — no imports from `speedlimitd`, no I/O, no clock — which is what makes it testable without the openpilot mocks the rest of the suite needs.

- [ ] **Step 1: Write the failing test**

Create `plugins/speedlimitd/tests/test_mapd_source.py`:

```python
"""Adapter tests — mapdOut becomes the road-context dict speedlimitd consumes."""
import os
import sys
from dataclasses import dataclass

import pytest

_SLD_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SLD_DIR not in sys.path:
  sys.path.insert(0, _SLD_DIR)

import mapd_source  # noqa: E402


@dataclass
class FakeMapdOut:
  """Stands in for a capnp MapdOut reader. Field names match the schema."""
  wayRef: str = ''
  wayName: str = ''
  roadName: str = ''
  speedLimit: float = 0.0
  lanes: int = 0
  highwayClass: str = 'unknown'
  wayId: int = 0
  tileLoaded: bool = True
  distanceFromWayCenter: float = 0.0
  waySelectionType: str = 'current'
  roadContext: str = 'city'


class TestHighwayClassName:
  def test_maps_to_underscore_osm_strings(self):
    # These feed URBAN_ONLY_TYPES membership and speed-table lookups in
    # infer_speed_from_road_type, which expect raw OSM values.
    assert mapd_source.highway_class_name('motorwayLink') == 'motorway_link'
    assert mapd_source.highway_class_name('livingStreet') == 'living_street'
    assert mapd_source.highway_class_name('trunkLink') == 'trunk_link'

  def test_simple_names_pass_through(self):
    assert mapd_source.highway_class_name('motorway') == 'motorway'
    assert mapd_source.highway_class_name('residential') == 'residential'

  def test_unknown_becomes_empty_string(self):
    # '' is what speedlimitd already treats as "no OSM classification".
    assert mapd_source.highway_class_name('unknown') == ''

  def test_unrecognised_value_is_empty_not_an_error(self):
    # A future mapd release adding an enum member must degrade to "no
    # classification", not crash the daemon.
    assert mapd_source.highway_class_name('someFutureClass') == ''


class TestTelemetry:
  def test_reports_fields_for_the_phase1_drive(self):
    out = FakeMapdOut(wayRef='S20', roadName='外环高速', speedLimit=27.8,
                      lanes=4, highwayClass='motorway', wayId=42,
                      tileLoaded=True, distanceFromWayCenter=1.5,
                      waySelectionType='current')
    t = mapd_source.telemetry_from_mapd(out, valid=True, our_way_ref='S20')
    assert t['mapdAlive'] is True
    assert t['mapdWayRef'] == 'S20'
    assert t['mapdWayId'] == 42
    assert t['mapdSpeedLimit'] == pytest.approx(100.1, abs=0.1)   # 27.8 m/s → km/h
    assert t['mapdHwClass'] == 'motorway'
    assert t['mapdLanes'] == 4
    assert t['mapdSelType'] == 'current'
    assert t['mapdTileLoaded'] is True
    assert t['mapdDistance'] == pytest.approx(1.5)
    assert t['mapdRefAgree'] is True

  def test_ref_disagreement_is_the_headline_number(self):
    out = FakeMapdOut(wayRef='S20', roadName='x')
    t = mapd_source.telemetry_from_mapd(out, valid=True, our_way_ref='G1503')
    assert t['mapdRefAgree'] is False

  def test_dead_mapd_reports_not_alive_with_neutral_values(self):
    t = mapd_source.telemetry_from_mapd(None, valid=False, our_way_ref='S20')
    assert t['mapdAlive'] is False
    assert t['mapdWayRef'] == ''
    assert t['mapdSpeedLimit'] == 0.0
    assert t['mapdRefAgree'] is False

  def test_always_returns_the_same_keys(self):
    # The rlog schema must not depend on mapd being up, or a drive with a dead
    # mapd would be unanalysable.
    live = mapd_source.telemetry_from_mapd(FakeMapdOut(wayRef='S20'), True, 'S20')
    dead = mapd_source.telemetry_from_mapd(None, False, 'S20')
    assert set(live) == set(dead)
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `PYTHONPATH= python3 -m pytest plugins/speedlimitd/tests/test_mapd_source.py -q`

Expected: FAIL at collection with `ModuleNotFoundError: No module named 'mapd_source'`.

- [ ] **Step 3: Write the adapter**

Create `plugins/speedlimitd/mapd_source.py`:

```python
"""Adapter — mapdOut road context for speedlimitd.

Pure and stateless: no I/O, no clock, no globals, no imports from speedlimitd.
That keeps it unit-testable with plain stubs and keeps speedlimitd.py (already
~1700 lines) from absorbing another responsibility.

PHASE 1 SCOPE: observation only. This module reports what mapd sees so a drive
can be compared against the offline tile reader that still drives control.
Phase 2 adds result_from_mapd(), which rebuilds the osm_query-shaped dict for
_ingest_osm_result — deliberately not written yet, because nothing calls it and
the Phase 1 drive may change its shape. See
docs/superpowers/specs/2026-08-18-mapd-integration-design.md.
"""

# MapdOut.highwayClass -> the OSM highway=* string speedlimitd already uses for
# last_osm_hwtype. Keys are capnp enumerant names (pycapnp returns the camelCase
# name when reading an enum field).
#
# Values are UNDERSCORE OSM form, not camelCase: they are tested against
# URBAN_ONLY_TYPES and used as speed-table keys in infer_speed_from_road_type
# (speedlimitd.py:556 and :585), both of which expect raw OSM values.
#
# 'unknown' -> '' because '' is what speedlimitd already treats as "no OSM
# classification available".
#
# Known gap: OSM highway=service has no HighwayClass member upstream, so service
# roads arrive as 'unknown' -> ''. The retired tile generator carried them
# verbatim, and 'service' is in URBAN_ONLY_TYPES, so such roads lose their
# urban demotion. Phase 1 telemetry measures how often this occurs.
_CLASS_TO_OSM = {
  'unknown': '',
  'motorway': 'motorway',
  'motorwayLink': 'motorway_link',
  'trunk': 'trunk',
  'trunkLink': 'trunk_link',
  'primary': 'primary',
  'primaryLink': 'primary_link',
  'secondary': 'secondary',
  'secondaryLink': 'secondary_link',
  'tertiary': 'tertiary',
  'tertiaryLink': 'tertiary_link',
  'unclassified': 'unclassified',
  'residential': 'residential',
  'livingStreet': 'living_street',
}

MS_TO_KPH = 3.6


def highway_class_name(value) -> str:
  """capnp HighwayClass enumerant name -> OSM highway=* string.

  pycapnp reads an enum field back as its camelCase enumerant name, so that is
  the only input form this needs to handle.

  Unrecognised values return '' rather than raising: a mapd release that adds an
  enum member must not crash the daemon, it must degrade to "no classification".
  """
  return _CLASS_TO_OSM.get(value, '')


def telemetry_from_mapd(mapd_out, valid: bool, our_way_ref: str) -> dict:
  """Phase 1 observation fields for pluginBusLog.

  Always returns the same keys whether or not mapd is up — a drive with a dead
  mapd must still produce an analysable rlog.

  mapdRefAgree is the headline number: it compares mapd's bearing-aware match
  against the tile reader's nearest-polyline match, and is the evidence for both
  the Phase 2 cutover and retiring the G/S margin-release rule.
  """
  if not valid or mapd_out is None:
    return {
      'mapdAlive': False,
      'mapdWayRef': '',
      'mapdWayId': 0,
      'mapdSpeedLimit': 0.0,
      'mapdHwClass': '',
      'mapdLanes': 0,
      'mapdSelType': '',
      'mapdTileLoaded': False,
      'mapdDistance': 0.0,
      'mapdRefAgree': False,
    }

  ref = mapd_out.wayRef or ''
  return {
    'mapdAlive': True,
    'mapdWayRef': ref,
    'mapdWayId': int(mapd_out.wayId or 0),
    'mapdSpeedLimit': round(float(mapd_out.speedLimit or 0.0) * MS_TO_KPH, 1),
    'mapdHwClass': highway_class_name(mapd_out.highwayClass),
    'mapdLanes': int(mapd_out.lanes or 0),
    'mapdSelType': str(mapd_out.waySelectionType),
    'mapdTileLoaded': bool(mapd_out.tileLoaded),
    'mapdDistance': round(float(mapd_out.distanceFromWayCenter or 0.0), 2),
    'mapdRefAgree': ref == (our_way_ref or ''),
  }
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `PYTHONPATH= python3 -m pytest plugins/speedlimitd/tests/test_mapd_source.py -q`

Expected: PASS, 8 passed.

- [ ] **Step 5: Run the full suite**

Run: `PYTHONPATH= python3 -m pytest plugins -q`

Expected: `3 failed, 776 passed, 21 skipped`.

- [ ] **Step 6: Commit**

```bash
git add plugins/speedlimitd/mapd_source.py plugins/speedlimitd/tests/test_mapd_source.py
git commit -m "feat(speedlimitd): mapdOut adapter — pure mapdOut to road-context dict"
```

---

### Task 5: Subscribe mapdOut and publish Phase 1 telemetry

**Files:**
- Modify: `plugins/speedlimitd/speedlimitd.py:735` (SubMaster), `:1202-1220` (query block), `:1635-1668` (publish)
- Modify: `plugins/speedlimitd/tests/test_speedlimitd.py`
- Modify: `plugins/speedlimitd/DESIGN.md`

**Interfaces:**
- Consumes from Task 4: `mapd_source.telemetry_from_mapd(mapd_out, valid, our_way_ref) -> dict`.
- Consumes from Task 2: the `mapdOut` service registration, without which `SubMaster(['mapdOut'])` raises.
- Produces: ten `mapd*` keys in the `speedLimitState` message. No new interface for later tasks.

**Background the implementer needs:** `speedlimitd` publishes one dict per 5 Hz tick via `self._sl_pub.send({...})` at `:1635`; bus_logger captures it into rlogs as JSON. The OSM tile query runs on a 5 s cadence inside `update()` at `:1202`. mapd telemetry must be sampled on that **same 5 s cadence**, not every tick, so the rlog rows line up with the tile-derived values they are being compared against.

`speedlimitd.py` imports its sibling `osm_query` inside `__init__` (`:743`) because the device runtime layout is flat. Import `mapd_source` the same way. The dev-machine test suite imports `plugins.speedlimitd.speedlimitd` as a package, so the sibling directory must be on `sys.path` — `test_speedlimitd.py` currently adds only `plugins/` and the repo root, and works by accident because `test_osm_query.py` inserts the speedlimitd directory first. Add the insert explicitly rather than relying on collection order.

**Critical constraint:** nothing computed here may reach a control path. The telemetry values are read, published, and discarded.

- [ ] **Step 1: Add the sys.path insert to the test module header**

In `plugins/speedlimitd/tests/test_speedlimitd.py`, after the existing `_REPO_ROOT` insert block, add:

```python
# speedlimitd.py imports its siblings (osm_query, mapd_source) by bare name,
# matching the device's flat runtime layout. Put that directory on sys.path
# explicitly — relying on test_osm_query.py having been collected first makes
# this suite depend on collection order.
_SLD_DIR = os.path.join(_PLUGINS_DIR, 'speedlimitd')
if _SLD_DIR not in sys.path:
  sys.path.insert(0, _SLD_DIR)
```

- [ ] **Step 2: Write the failing test**

Append to `plugins/speedlimitd/tests/test_speedlimitd.py`:

```python
class TestMapdPhase1Telemetry:
  """Phase 1 publishes mapd observations and changes NOTHING else.

  The point of the phase is to compare mapd's road context against the tile
  reader's on a real drive while the tile reader still drives every control
  path. If any mapd value reached actuation, the comparison would be measuring
  itself.
  """

  def test_mapd_out_is_subscribed(self, sld):
    import inspect
    src = inspect.getsource(sld.SpeedLimitMiddleware.__init__)
    assert "'mapdOut'" in src or '"mapdOut"' in src

  def test_telemetry_keys_published(self, sld, monkeypatch):
    mw = sld.SpeedLimitMiddleware()
    sent = {}
    mw._sl_pub.send = lambda payload: sent.update(payload)
    mw.update()
    for key in ('mapdAlive', 'mapdWayRef', 'mapdWayId', 'mapdSpeedLimit',
                'mapdHwClass', 'mapdLanes', 'mapdSelType', 'mapdTileLoaded',
                'mapdDistance', 'mapdRefAgree'):
      assert key in sent, f'{key} missing from published telemetry'

  def test_publishes_even_when_mapd_absent(self, sld):
    # A drive with a dead mapd must still yield an analysable rlog.
    mw = sld.SpeedLimitMiddleware()
    sent = {}
    mw._sl_pub.send = lambda payload: sent.update(payload)
    mw.update()
    assert sent['mapdAlive'] is False
    assert sent['mapdWayRef'] == ''

  @staticmethod
  def _armed(sld):
    """An instance whose 5 s OSM/mapd sampling block will actually run.

    A fresh instance has _gps_valid False, so the sampling block is skipped
    entirely — a containment test built on one would pass trivially without ever
    injecting anything.
    """
    mw = sld.SpeedLimitMiddleware()
    mw._gps_valid = True
    mw._gps_lat, mw._gps_lon = 31.3137, 121.5395
    mw._osm_last_query_t = 0.0
    return mw

  def test_sampling_block_actually_runs(self, sld):
    # Guards the containment test below: proves _armed() reaches the sampling
    # path, so a passing containment result means something.
    mw = self._armed(sld)
    mw.update()
    assert mw._osm_last_query_t > 0.0

  def test_mapd_values_do_not_reach_control_state(self, sld):
    """Containment: injecting mapd data must not move any control variable.

    Runs update() from identical armed state twice — once with no mapd data,
    once with a full mapd message reporting a DIFFERENT road at a DIFFERENT
    speed — and asserts every control-bearing attribute is unchanged. The tile
    reader sees no tiles in either run, so it contributes identically.
    """
    control_attrs = ('last_way_ref', 'last_road_name', 'last_osm_hwtype',
                     'last_osm_speed_kph', 'last_road_context',
                     'last_highway_type', 'last_road_id', 'curvature_cap',
                     'inference_mode', '_gs_limit_kph')

    baseline = self._armed(sld)
    baseline.update()
    before = {a: getattr(baseline, a) for a in control_attrs}

    fake = MagicMock()
    fake.wayRef = 'G1503'
    fake.wayName = 'injected'
    fake.roadName = 'injected'
    fake.speedLimit = 33.3
    fake.lanes = 5
    fake.highwayClass = 'motorway'
    fake.wayId = 999
    fake.tileLoaded = True
    fake.distanceFromWayCenter = 0.4
    fake.waySelectionType = 'current'

    injected = self._armed(sld)
    real_sm = injected.sm
    injected.sm = MagicMock()
    injected.sm.__getitem__ = lambda _s, k: fake if k == 'mapdOut' else real_sm[k]
    injected.sm.valid = {'mapdOut': True}
    injected.sm.updated = {}
    sent = {}
    injected._sl_pub.send = lambda payload: sent.update(payload)
    injected.update()

    after = {a: getattr(injected, a) for a in control_attrs}
    assert after == before, 'mapd data leaked into a control variable'
    # ...and the injected data really did arrive, so the assertion above is
    # testing containment rather than an absent message.
    assert sent['mapdWayRef'] == 'G1503'
    assert sent['mapdAlive'] is True
```

- [ ] **Step 3: Run the test to verify it fails**

Run: `PYTHONPATH= python3 -m pytest plugins/speedlimitd/tests/test_speedlimitd.py -k MapdPhase1 -q`

Expected: FAIL. `test_mapd_out_is_subscribed` fails because the SubMaster list has no `mapdOut`; the telemetry tests fail with `KeyError`/`assert key in sent`.

- [ ] **Step 4: Subscribe to mapdOut**

In `plugins/speedlimitd/speedlimitd.py`, change line 735:

```python
    self.sm = messaging.SubMaster(['modelV2', 'gpsLocationExternal', 'livePose', 'mapdOut'])
```

In `__init__`, next to the existing `from osm_query import OsmTileReader` (`:743`), add the sibling import and the telemetry holder:

```python
    import mapd_source
    self._mapd_source = mapd_source
    # Phase 1: mapd road context is OBSERVED ONLY. These values are published
    # to pluginBusLog for the cutover comparison and must never be read by any
    # control path — the tile reader above still drives everything.
    self._mapd_telemetry = mapd_source.telemetry_from_mapd(None, False, '')
```

- [ ] **Step 5: Sample mapd on the existing 5 s cadence**

In `update()`, inside the `if self._gps_valid and now - self._osm_last_query_t >= self._osm_query_interval:` block (`:1202`), immediately after the `write_plugin_param('speedlimitd', 'OsmTilesMissing', ...)` try/except and still inside the block, add:

```python
      # --- Phase 1 mapd observation (telemetry only) ---
      # Sampled on the SAME 5 s cadence as the tile query above so each rlog row
      # compares mapd and the tile reader at the same instant. Nothing below
      # reads these values.
      try:
        self._mapd_telemetry = self._mapd_source.telemetry_from_mapd(
          self.sm['mapdOut'], bool(self.sm.valid.get('mapdOut', False)),
          self.last_way_ref)
      except Exception:
        self._mapd_telemetry = self._mapd_source.telemetry_from_mapd(None, False, '')
```

- [ ] **Step 6: Publish the telemetry**

In the `self._sl_pub.send({...})` call at `:1635`, add as the final entry, after `'osmRejectReason': osm_reject_reason,`:

```python
      # --- Phase 1 mapd observation (see mapd_source.telemetry_from_mapd) ---
      # Read-only comparison against the tile-derived fields above.
      # mapdRefAgree is the cutover decision metric.
      **self._mapd_telemetry,
```

- [ ] **Step 7: Run the test to verify it passes**

Run: `PYTHONPATH= python3 -m pytest plugins/speedlimitd/tests/test_speedlimitd.py -k MapdPhase1 -q`

Expected: PASS, 6 passed.

- [ ] **Step 8: Run the full suite**

Run: `PYTHONPATH= python3 -m pytest plugins -q`

Expected: `3 failed, 782 passed, 21 skipped`. If any pre-existing speedlimitd test now fails, the mapd sampling has leaked into control state — fix the leak, do not adjust the test.

- [ ] **Step 9: Document the observation path**

In `plugins/speedlimitd/DESIGN.md`, under the section describing OSM data, add:

```markdown
### mapd observation (Phase 1, 2026-08-18)

speedlimitd subscribes to `mapdOut` and publishes ten `mapd*` fields into
`speedLimitState` for comparison against the tile-derived values beside them.
This is telemetry only: `mapd_source.telemetry_from_mapd` output is published
and discarded, and the offline tile reader still drives every control path.

`mapdRefAgree` — mapd's bearing-aware way match versus our nearest-polyline
match — is the metric that decides the Phase 2 cutover and whether the G/S
margin-release rule can be retired. See
`docs/superpowers/specs/2026-08-18-mapd-integration-design.md`.
```

- [ ] **Step 10: Commit**

```bash
git add plugins/speedlimitd/speedlimitd.py plugins/speedlimitd/tests/test_speedlimitd.py \
        plugins/speedlimitd/DESIGN.md
git commit -m "feat(speedlimitd): observe mapdOut — phase 1 telemetry, actuation unchanged"
```

---

## Post-implementation verification

Not part of the task loop — run after all five tasks land.

**1. Confirm the schema injects cleanly.** The slot fragments are only exercised end-to-end by `install.sh`. On the device, after deploying:

```bash
ssh c3 'grep -A3 "highwayClass" /data/openpilot/cereal/custom.capnp'
```

Expected: the three new `MapdOut` fields present in the injected schema.

**2. Confirm mapd starts and stays up.**

```bash
ssh c3 'cat /data/plugins-runtime/.pids/mapd.pid && tail -20 /tmp/plugin_logs/mapd.log'
```

Expected: a live PID and no gomsgq panic. Re-check after a drive — the shadow `carState` torn read appears under model-inference load, not at idle.

**3. Confirm the binary version.**

```bash
ssh c3 'cat /data/media/0/osm/mapd_version'
```

Expected: `v2.3.0`.

**4. Phase 1 drive.** One drive covering the S20 corridor. Then from the rlog, report:
- `mapdRefAgree` rate over samples where both refs are non-empty
- mapd uptime and restart count from `plugin_health`
- `mapdSpeedLimit` versus `osmSpeedLimit` on S20 — the expected regression is mapd reporting 0 where our tiles report 100
- `mapdSelType` distribution, especially `fail` rate

Those four numbers decide whether Phase 2 proceeds as specified.
