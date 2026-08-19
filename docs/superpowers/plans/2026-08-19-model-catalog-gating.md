# Model Catalog Gating Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Only models that have passed a test drive on the running openpilot version (plus the model the release itself ships) can be downloaded or activated from the model_selector UI.

**Architecture:** A new `catalog.py` in the model_selector plugin owns all compatibility policy and reads a curated `compatible_models.json` shipped at the plugin root. Three call sites consult it — `check_updates` (what may be offered), `download_model` (what may be fetched), `swap_model` (what may be activated) — so no path is left ungated. A maintainer bypasses the gate with an unlock marker in the plugin's runtime `data/` dir, which survives reinstalls.

**Tech Stack:** Python 3.12, pytest, uv. No new dependencies.

**Spec:** `docs/superpowers/specs/2026-08-19-model-catalog-gating-design.md`

## Global Constraints

- Repo: `/home/oxygen/catpilot-dev/plugins`, branch `dev`. Plugin dir: `plugins/model_selector/`.
- Run tests as `PYTHONPATH= uv run pytest plugins/model_selector -q` from the repo root. The empty `PYTHONPATH` is mandatory — a foreign `PYTHONPATH` in this shell shadows the repo's namespace package and silently tests a different worktree.
- The catalog file lives at the **plugin root** (`plugins/model_selector/compatible_models.json`), never in `data/`. `install.sh` preserves `data/` wholesale across reinstalls, so a catalog there would freeze at first install.
- The unlock marker lives at `<PLUGINS_RUNTIME_DIR>/model_selector/data/.unlocked` precisely because `data/` is preserved.
- Every module in this plugin must import siblings with the repo's dual-import pattern (`try: from plugins.model_selector.X import Y / except ImportError: from X import Y`) — the scripts run both as an installed package and as bare CLI scripts under `/usr/local/venv/bin/python`.
- `catalog.py` holds policy only. It must not learn file layout (ONNX filenames, storage dirs, tracker paths); that lives in `ModelSwapper.MODEL_CONFIGS`.
- Fail closed: any missing, unreadable, or corrupt catalog resolves to zero verified models.
- Openpilot version string comes from `/data/openpilot/common/version.h` (`"0.11.1"`), falling back to `manifest.OPENPILOT_VERSION`, then `''`.
- Do NOT commit unrelated working-tree changes. `plugins/c3_compat/boot_patch.sh` is modified and `overlays/` is untracked; both must stay out of every commit. Use explicit `git add <paths>`, never `git commit -a`.
- No `Co-Authored-By` lines in commit messages.
- Do not deploy to the C3 or restart anything on the device as part of this plan. Task 7 lists the deploy commands for the user to run when they choose.

## File Structure

| File | Responsibility |
|---|---|
| `plugins/model_selector/catalog.py` (create) | All compatibility policy: version detection, catalog loading, verified/baseline lookup, unlock detection, catalog validation. |
| `plugins/model_selector/compatible_models.json` (create) | The curated data. Ships with the plugin, overwritten by every deploy. |
| `plugins/model_selector/model_swapper.py` (modify) | Adds a `verified` flag to listed models, refuses unverified swaps, imports the release's stock ONNX. Loses `MIN_MODEL_DATE`. |
| `plugins/model_selector/model_download.py` (modify) | `check_updates` sources the catalog instead of the GitHub registry; `download_model` refuses uncatalogued ids; `--unlocked` CLI flag. |
| `plugins/model_selector/ui.py` (modify) | Untested markers, blocked activation, stock-recovery offer, CHECK becomes a local diff. |
| `plugins/model_selector/tests/test_catalog.py` (create) | Unit tests for the policy module. |
| `plugins/model_selector/tests/test_model_swapper.py` (modify) | Gate and `verified`-flag tests. |
| `plugins/model_selector/tests/test_model_download.py` (modify) | `check_updates` and download-gate tests. |
| `plugins/model_selector/DESIGN.md`, `README.md` (modify) | Documentation. |

---

### Task 1: `catalog.py` — the policy module

**Files:**
- Create: `plugins/model_selector/catalog.py`
- Test: `plugins/model_selector/tests/test_catalog.py`

**Interfaces:**
- Consumes: nothing from other tasks.
- Produces, all consumed by Tasks 3-5:
  - `openpilot_version() -> str`
  - `load_catalog() -> dict` with keys `'driving'`, `'dm'`, each a `list[dict]`
  - `verified_entries(model_type) -> list[dict]` — `model_type` is a `ModelType` enum **or** the string `'driving'` / `'dm'`
  - `is_verified(model_type, model_id: str) -> bool`
  - `baseline_entry(model_type) -> dict | None`
  - `unlocked() -> bool`
  - `validate_catalog(catalog: dict | None = None) -> list[str]` — returns problem strings, empty means valid
  - Module constants `CATALOG_FILE`, `VERSION_H`, `UNLOCK_MARKER` (tests monkeypatch these)

- [ ] **Step 1: Write the failing tests**

Create `plugins/model_selector/tests/test_catalog.py`:

```python
"""Tests for the curated model catalog — the gate on install and activation."""
import importlib
import json

import pytest


@pytest.fixture
def cat():
  import plugins.model_selector.catalog as mod
  importlib.reload(mod)
  return mod


@pytest.fixture
def catalog_env(cat, tmp_path, monkeypatch):
  """Point the module at a temp catalog, version.h and unlock marker."""
  version_h = tmp_path / 'version.h'
  version_h.write_text('#define COMMA_VERSION "0.11.1"\n')
  monkeypatch.setattr(cat, 'VERSION_H', version_h)
  monkeypatch.setattr(cat, 'CATALOG_FILE', tmp_path / 'compatible_models.json')
  monkeypatch.setattr(cat, 'UNLOCK_MARKER', tmp_path / '.unlocked')
  return cat


def _write(cat, data):
  cat.CATALOG_FILE.write_text(json.dumps(data))


def _entry(**over):
  e = {
    'id': 'cool_people_3c957c6',
    'name': 'Cool People',
    'date': '2025-10-20',
    'commit': 'c' * 40,
    'files': ['driving_vision.onnx', 'driving_policy.onnx'],
    'verified_on': ['0.11.1'],
  }
  e.update(over)
  return e


def _stock(**over):
  e = {
    'id': 'stock_0.11.1',
    'name': 'Release default',
    'date': '2026-05-18',
    'source': 'shipped',
    'verified_on': ['0.11.1'],
    'baseline_for': ['0.11.1'],
  }
  e.update(over)
  return e


class TestOpenpilotVersion:
  def test_parses_version_h(self, catalog_env):
    assert catalog_env.openpilot_version() == '0.11.1'

  def test_missing_version_h_falls_back_to_empty(self, catalog_env, tmp_path, monkeypatch):
    monkeypatch.setattr(catalog_env, 'VERSION_H', tmp_path / 'nope.h')
    # manifest import fails off-device, so the chain ends at ''
    assert catalog_env.openpilot_version() in ('', '0.11.1')

  def test_malformed_version_h_does_not_raise(self, catalog_env):
    catalog_env.VERSION_H.write_text('garbage with no quotes\n')
    assert catalog_env.openpilot_version() in ('', '0.11.1')


class TestLoadCatalog:
  def test_missing_file_fails_closed(self, catalog_env):
    assert catalog_env.load_catalog() == {}
    assert catalog_env.verified_entries('driving') == []

  def test_corrupt_json_fails_closed(self, catalog_env):
    catalog_env.CATALOG_FILE.write_text('{ this is not json')
    assert catalog_env.load_catalog() == {}
    assert catalog_env.verified_entries('driving') == []

  def test_entries_without_id_are_dropped(self, catalog_env):
    _write(catalog_env, {'driving': [_entry(), {'name': 'no id'}], 'dm': []})
    assert len(catalog_env.load_catalog()['driving']) == 1


class TestVerifiedEntries:
  def test_matching_version_is_returned(self, catalog_env):
    _write(catalog_env, {'driving': [_entry()], 'dm': []})
    assert [e['id'] for e in catalog_env.verified_entries('driving')] == ['cool_people_3c957c6']

  def test_other_version_is_excluded(self, catalog_env):
    _write(catalog_env, {'driving': [_entry(verified_on=['0.11.2'])], 'dm': []})
    assert catalog_env.verified_entries('driving') == []

  def test_multi_version_entry_matches_both(self, catalog_env):
    _write(catalog_env, {'driving': [_entry(verified_on=['0.11.1', '0.11.2'])], 'dm': []})
    assert len(catalog_env.verified_entries('driving')) == 1

  def test_accepts_model_type_enum(self, catalog_env):
    from plugins.model_selector.model_swapper import ModelType
    _write(catalog_env, {'driving': [_entry()], 'dm': []})
    assert len(catalog_env.verified_entries(ModelType.DRIVING)) == 1

  def test_unknown_version_returns_nothing(self, catalog_env):
    catalog_env.VERSION_H.write_text('#define COMMA_VERSION "9.9.9"\n')
    _write(catalog_env, {'driving': [_entry()], 'dm': []})
    assert catalog_env.verified_entries('driving') == []


class TestIsVerified:
  def test_true_for_listed(self, catalog_env):
    _write(catalog_env, {'driving': [_entry()], 'dm': []})
    assert catalog_env.is_verified('driving', 'cool_people_3c957c6')

  def test_false_for_unlisted(self, catalog_env):
    _write(catalog_env, {'driving': [_entry()], 'dm': []})
    assert not catalog_env.is_verified('driving', 'mystery_model')

  def test_false_for_wrong_type(self, catalog_env):
    _write(catalog_env, {'driving': [_entry()], 'dm': []})
    assert not catalog_env.is_verified('dm', 'cool_people_3c957c6')


class TestBaselineEntry:
  def test_returns_the_baseline(self, catalog_env):
    _write(catalog_env, {'driving': [_entry(), _stock()], 'dm': []})
    assert catalog_env.baseline_entry('driving')['id'] == 'stock_0.11.1'

  def test_none_when_no_baseline(self, catalog_env):
    _write(catalog_env, {'driving': [_entry()], 'dm': []})
    assert catalog_env.baseline_entry('driving') is None


class TestUnlocked:
  def test_false_without_marker(self, catalog_env):
    assert not catalog_env.unlocked()

  def test_true_with_marker(self, catalog_env):
    catalog_env.UNLOCK_MARKER.write_text('')
    assert catalog_env.unlocked()


class TestValidateCatalog:
  def _valid(self):
    return {'driving': [_entry(), _stock()],
            'dm': [_stock(id='stock_dm_0.11.1', name='Release default DM')]}

  def test_valid_catalog_has_no_problems(self, catalog_env):
    assert catalog_env.validate_catalog(self._valid()) == []

  def test_missing_baseline_is_reported(self, catalog_env):
    c = self._valid()
    c['driving'] = [_entry()]
    problems = catalog_env.validate_catalog(c)
    assert any('0.11.1' in p and 'baseline' in p for p in problems)

  def test_two_baselines_is_reported(self, catalog_env):
    c = self._valid()
    c['driving'] = [_stock(), _stock(id='other_stock')]
    problems = catalog_env.validate_catalog(c)
    assert any('baseline' in p for p in problems)

  def test_duplicate_id_is_reported(self, catalog_env):
    c = self._valid()
    c['driving'] = [_entry(), _entry(), _stock()]
    assert any('duplicate' in p for p in catalog_env.validate_catalog(c))

  def test_downloadable_entry_needs_commit_and_files(self, catalog_env):
    c = self._valid()
    bad = _entry()
    del bad['commit']
    del bad['files']
    c['driving'] = [bad, _stock()]
    problems = catalog_env.validate_catalog(c)
    assert any('commit' in p for p in problems)
    assert any('files' in p for p in problems)

  def test_shipped_entry_needs_neither(self, catalog_env):
    assert catalog_env.validate_catalog(self._valid()) == []

  def test_baseline_must_be_verified_on_that_version(self, catalog_env):
    c = self._valid()
    c['driving'] = [_stock(verified_on=['0.11.2'], baseline_for=['0.11.1'])]
    assert any('baseline_for' in p for p in catalog_env.validate_catalog(c))
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `PYTHONPATH= uv run pytest plugins/model_selector/tests/test_catalog.py -q`
Expected: collection error — `ModuleNotFoundError: No module named 'plugins.model_selector.catalog'`

- [ ] **Step 3: Write `catalog.py`**

```python
"""Curated model catalog — the gate on which models may be installed or activated.

Compatibility policy lives here and nowhere else: model_download and
model_swapper ask this module rather than re-deriving compatibility from model
dates. A model may be offered or activated only if the catalog records that it
passed a test drive on the openpilot version this device is running — or that it
is the model the release itself ships, which is verified by definition.

This module is pure policy. It deliberately knows nothing about ONNX filenames
or storage layout; that belongs to ModelSwapper.MODEL_CONFIGS.
"""
import json
import os
from pathlib import Path

CATALOG_FILE = Path(__file__).resolve().parent / 'compatible_models.json'
VERSION_H = Path(os.getenv('OPENPILOT_DIR', '/data/openpilot')) / 'common' / 'version.h'
UNLOCK_MARKER = (Path(os.getenv('PLUGINS_RUNTIME_DIR', '/data/plugins-runtime'))
                 / 'model_selector' / 'data' / '.unlocked')

MODEL_TYPES = ('driving', 'dm')


def _type_key(model_type) -> str:
  """Accept either a ModelType enum or a plain 'driving'/'dm' string."""
  return getattr(model_type, 'value', model_type)


def openpilot_version() -> str:
  """Version of the openpilot code actually running, e.g. '0.11.1'.

  version.h is the truth; manifest.OPENPILOT_VERSION is a hand-maintained
  mirror that drifts between rebases, so it is only a fallback. Parsing is
  import-free because this runs inside a bare-venv CLI subprocess too.
  """
  try:
    return VERSION_H.read_text().split('"')[1]
  except (OSError, IndexError):
    pass
  try:
    from openpilot.selfdrive.plugins.manifest import OPENPILOT_VERSION
    return OPENPILOT_VERSION
  except Exception:
    return ''


def load_catalog() -> dict:
  """Parse the catalog. Returns {} on any problem — the gate fails closed."""
  try:
    with open(CATALOG_FILE) as f:
      data = json.load(f)
  except (OSError, json.JSONDecodeError):
    return {}
  if not isinstance(data, dict):
    return {}
  return {t: [e for e in data.get(t, []) if isinstance(e, dict) and e.get('id')]
          for t in MODEL_TYPES}


def verified_entries(model_type) -> list:
  """Catalog entries verified for the running openpilot version."""
  version = openpilot_version()
  if not version:
    return []
  return [e for e in load_catalog().get(_type_key(model_type), [])
          if version in e.get('verified_on', [])]


def is_verified(model_type, model_id: str) -> bool:
  return any(e['id'] == model_id for e in verified_entries(model_type))


def baseline_entry(model_type):
  """The known-good fallback model for the running version, or None."""
  version = openpilot_version()
  for e in verified_entries(model_type):
    if version in e.get('baseline_for', []):
      return e
  return None


def unlocked() -> bool:
  """True when the maintainer has unlocked untested models on this device."""
  return UNLOCK_MARKER.exists()


def validate_catalog(catalog: dict | None = None) -> list:
  """Return a list of problems with the catalog. Empty list means valid.

  Catches malformed catalogs at push time rather than on a device.
  """
  catalog = load_catalog() if catalog is None else catalog
  problems = []

  versions = set()
  for t in MODEL_TYPES:
    for e in catalog.get(t, []):
      versions.update(e.get('verified_on', []))

  for t in MODEL_TYPES:
    entries = catalog.get(t, [])

    ids = [e.get('id') for e in entries]
    for dup in sorted({i for i in ids if ids.count(i) > 1}):
      problems.append(f"{t}: duplicate entry '{dup}'")

    for e in entries:
      eid = e.get('id', '?')
      required = ['id', 'name', 'date', 'verified_on']
      if e.get('source') != 'shipped':
        required += ['commit', 'files']
      for field in required:
        if not e.get(field):
          problems.append(f"{t}: entry '{eid}' missing {field}")
      for ver in e.get('baseline_for', []):
        if ver not in e.get('verified_on', []):
          problems.append(f"{t}: '{eid}' is baseline_for {ver} but not verified_on it")

    for ver in sorted(versions):
      n = sum(1 for e in entries if ver in e.get('baseline_for', []))
      if n != 1:
        problems.append(f"{t}: version {ver} has {n} baselines, expected exactly 1")

  return problems
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `PYTHONPATH= uv run pytest plugins/model_selector/tests/test_catalog.py -q`
Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add plugins/model_selector/catalog.py plugins/model_selector/tests/test_catalog.py
git commit -m "feat(model_selector): add catalog.py — curated compatibility policy"
```

---

### Task 2: Seed `compatible_models.json`

**Files:**
- Create: `plugins/model_selector/compatible_models.json`
- Test: `plugins/model_selector/tests/test_catalog.py` (append one class)

**Interfaces:**
- Consumes: `validate_catalog()` from Task 1.
- Produces: the shipped catalog. Stock ids `stock_0.11.1` (driving) and `stock_dm_0.11.1` (dm) are referenced by Task 3's `import_stock` tests and Task 5's recovery flow.

The catalog seeds with the release defaults only. catpilot v0.11.1 is based on stock openpilot v0.11.1 (2026-05-18, per `RELEASES.md:3`), and the ONNX it ships are LFS-tracked in the catpilot tree — so they are verified by definition and need no commit. Downloadable entries get added by the user later, after test drives.

- [ ] **Step 1: Write the failing test**

Append to `plugins/model_selector/tests/test_catalog.py`:

```python
class TestShippedCatalog:
  """The catalog that actually ships must be valid and cover this version."""

  def test_shipped_catalog_is_valid(self, cat):
    assert cat.validate_catalog(cat.load_catalog()) == []

  def test_shipped_catalog_has_a_baseline_for_0_11_1(self, cat):
    catalog = cat.load_catalog()
    for model_type in ('driving', 'dm'):
      baselines = [e for e in catalog[model_type] if '0.11.1' in e.get('baseline_for', [])]
      assert len(baselines) == 1, f"{model_type} needs exactly one 0.11.1 baseline"
      assert baselines[0]['source'] == 'shipped'
```

Note this uses the `cat` fixture, not `catalog_env` — it deliberately reads the real shipped file.

- [ ] **Step 2: Run the test to verify it fails**

Run: `PYTHONPATH= uv run pytest plugins/model_selector/tests/test_catalog.py::TestShippedCatalog -q`
Expected: FAIL — `load_catalog()` returns `{}` because the file does not exist, so `catalog['driving']` raises `KeyError`.

- [ ] **Step 3: Write the catalog**

Create `plugins/model_selector/compatible_models.json`:

```json
{
  "driving": [
    {
      "id": "stock_0.11.1",
      "name": "Release default",
      "date": "2026-05-18",
      "source": "shipped",
      "verified_on": ["0.11.1"],
      "baseline_for": ["0.11.1"],
      "notes": "driving_vision.onnx + driving_policy.onnx shipped in catpilot v0.11.1"
    }
  ],
  "dm": [
    {
      "id": "stock_dm_0.11.1",
      "name": "Release default DM",
      "date": "2026-05-18",
      "source": "shipped",
      "verified_on": ["0.11.1"],
      "baseline_for": ["0.11.1"],
      "notes": "dmonitoring_model.onnx shipped in catpilot v0.11.1"
    }
  ]
}
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `PYTHONPATH= uv run pytest plugins/model_selector/tests/test_catalog.py -q`
Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add plugins/model_selector/compatible_models.json plugins/model_selector/tests/test_catalog.py
git commit -m "feat(model_selector): seed catalog with the v0.11.1 release defaults"
```

---

### Task 3: Gate `model_swapper.py`

**Files:**
- Modify: `plugins/model_selector/model_swapper.py` (lines 17-21, 122-163, 183-213)
- Test: `plugins/model_selector/tests/test_model_swapper.py`

**Interfaces:**
- Consumes: `catalog.is_verified`, `catalog.unlocked`, `catalog.baseline_entry`, `catalog.openpilot_version` from Task 1; stock ids from Task 2.
- Produces, consumed by Tasks 4-5:
  - `ModelSwapper.list_models()` entries gain a `verified: bool` key
  - `ModelSwapper.swap_model(model_id)` raises `ValueError` whose message contains `not verified` when the model is unverified and the device is locked
  - `ModelSwapper.import_stock() -> bool`
  - `MIN_MODEL_DATE` is **removed** from this module

- [ ] **Step 1: Write the failing tests**

Append to `plugins/model_selector/tests/test_model_swapper.py`:

```python
class TestCatalogGate:
  @pytest.fixture
  def gated(self, swapper_mod, tmp_path, monkeypatch):
    """A DRIVING swapper whose storage is tmp_path and whose catalog is ours."""
    import plugins.model_selector.catalog as cat

    version_h = tmp_path / 'version.h'
    version_h.write_text('#define COMMA_VERSION "0.11.1"\n')
    monkeypatch.setattr(cat, 'VERSION_H', version_h)
    monkeypatch.setattr(cat, 'CATALOG_FILE', tmp_path / 'catalog.json')
    monkeypatch.setattr(cat, 'UNLOCK_MARKER', tmp_path / '.unlocked')
    (tmp_path / 'catalog.json').write_text(json.dumps({
      'driving': [{'id': 'good_model', 'name': 'Good', 'date': '2025-10-20',
                   'commit': 'c' * 40, 'files': ['driving_vision.onnx', 'driving_policy.onnx'],
                   'verified_on': ['0.11.1']},
                  {'id': 'stock_0.11.1', 'name': 'Release default', 'date': '2026-05-18',
                   'source': 'shipped', 'verified_on': ['0.11.1'], 'baseline_for': ['0.11.1']}],
      'dm': [],
    }))

    models_dir = tmp_path / 'models' / 'driving'
    models_dir.mkdir(parents=True)
    sw = swapper_mod.ModelSwapper(swapper_mod.ModelType.DRIVING)
    sw.models_dir = models_dir
    sw.active_model_file = models_dir.parent / 'active_driving_model'
    monkeypatch.setattr(type(sw), 'ACTIVE_DIR', tmp_path / 'active')
    (tmp_path / 'active').mkdir()
    return sw, cat, tmp_path

  def _install(self, sw, model_id, date='2025-10-20'):
    d = sw.models_dir / model_id
    d.mkdir(parents=True)
    for f in sw.onnx_files:
      (d / f).write_bytes(b'onnx')
    (d / 'model_info.json').write_text(json.dumps({'name': model_id, 'date': date}))
    return d

  def test_list_models_flags_verified(self, gated):
    sw, _, _ = gated
    self._install(sw, 'good_model')
    self._install(sw, 'mystery_model')
    by_id = {m['id']: m for m in sw.list_models()}
    assert by_id['good_model']['verified'] is True
    assert by_id['mystery_model']['verified'] is False

  def test_list_models_no_longer_hides_old_models(self, gated):
    sw, _, _ = gated
    self._install(sw, 'ancient_model', date='2024-01-01')
    assert 'ancient_model' in {m['id'] for m in sw.list_models()}

  def test_swap_refuses_unverified(self, gated):
    sw, _, _ = gated
    self._install(sw, 'mystery_model')
    with pytest.raises(ValueError, match='not verified'):
      sw.swap_model('mystery_model')

  def test_swap_allows_unverified_when_unlocked(self, gated):
    sw, cat, tmp_path = gated
    self._install(sw, 'mystery_model')
    cat.UNLOCK_MARKER.write_text('')
    sw.swap_model('mystery_model')
    assert sw.get_active_model() == 'mystery_model'

  def test_swap_allows_verified(self, gated):
    sw, _, _ = gated
    self._install(sw, 'good_model')
    sw.swap_model('good_model')
    assert sw.get_active_model() == 'good_model'


class TestImportStock:
  @pytest.fixture
  def stocked(self, swapper_mod, tmp_path, monkeypatch):
    import plugins.model_selector.catalog as cat
    version_h = tmp_path / 'version.h'
    version_h.write_text('#define COMMA_VERSION "0.11.1"\n')
    monkeypatch.setattr(cat, 'VERSION_H', version_h)
    monkeypatch.setattr(cat, 'CATALOG_FILE', tmp_path / 'catalog.json')
    monkeypatch.setattr(cat, 'UNLOCK_MARKER', tmp_path / '.unlocked')
    (tmp_path / 'catalog.json').write_text(json.dumps({
      'driving': [{'id': 'stock_0.11.1', 'name': 'Release default', 'date': '2026-05-18',
                   'source': 'shipped', 'verified_on': ['0.11.1'], 'baseline_for': ['0.11.1']}],
      'dm': [],
    }))
    models_dir = tmp_path / 'models' / 'driving'
    models_dir.mkdir(parents=True)
    sw = swapper_mod.ModelSwapper(swapper_mod.ModelType.DRIVING)
    sw.models_dir = models_dir
    sw.active_model_file = models_dir.parent / 'active_driving_model'
    active = tmp_path / 'active'
    active.mkdir()
    monkeypatch.setattr(type(sw), 'ACTIVE_DIR', active)
    for f in sw.onnx_files:
      (active / f).write_bytes(b'shipped-onnx')
    return sw

  def test_imports_when_no_tracker(self, stocked):
    assert stocked.import_stock() is True
    dest = stocked.models_dir / 'stock_0.11.1'
    assert (dest / 'driving_vision.onnx').read_bytes() == b'shipped-onnx'
    assert json.loads((dest / 'model_info.json').read_text())['source'] == 'shipped'

  def test_is_idempotent(self, stocked):
    assert stocked.import_stock() is True
    assert stocked.import_stock() is False

  def test_skips_when_tracker_exists(self, stocked):
    stocked.active_model_file.write_text(json.dumps({'id': 'something_else'}))
    assert stocked.import_stock() is False
    assert not (stocked.models_dir / 'stock_0.11.1').exists()

  def test_skips_when_active_dir_incomplete(self, stocked):
    (stocked.ACTIVE_DIR / 'driving_policy.onnx').unlink()
    assert stocked.import_stock() is False

  def test_imported_stock_is_listed_and_verified(self, stocked):
    stocked.import_stock()
    by_id = {m['id']: m for m in stocked.list_models()}
    assert by_id['stock_0.11.1']['verified'] is True
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `PYTHONPATH= uv run pytest plugins/model_selector/tests/test_model_swapper.py -q`
Expected: FAIL — `AttributeError: 'ModelSwapper' object has no attribute 'import_stock'` and `KeyError: 'verified'`.

- [ ] **Step 3: Modify `model_swapper.py`**

3a. Replace the `MIN_MODEL_DATE` block (lines 17-21) with the catalog import. Delete the constant entirely — the catalog supersedes it and nothing else reads it:

```python
try:
    from plugins.model_selector import catalog
except ImportError:
    import catalog
```

3b. In `list_models`, delete the date-filter block:

```python
                        # Filter out models incompatible with current openpilot version
                        min_date = MIN_MODEL_DATE.get(self.model_type.value, '0000-00-00')
                        if info.get('date', '0000-00-00') < min_date:
                            continue
```

and replace the `models.append({...})` call with:

```python
                        entry = {
                            'id': model_dir.name,
                            'has_onnx': has_onnx,
                            'cached_pkl_count': cached_pkl,
                            'total_pkl_count': len(self.required_pkl_stems),
                            **info
                        }
                        # after **info so a stale model_info.json cannot forge a verdict
                        entry['verified'] = catalog.is_verified(self.model_type, model_dir.name)
                        models.append(entry)
```

3c. In `swap_model`, resolve and gate **before** any work. Insert immediately after the docstring, above `# STEP 1`:

```python
        # STEP 0: Refuse models the catalog has not verified for this version.
        # Resolution happens first so a name (not just an id) can be gated.
        model_id = self.resolve_model_id(model_id)
        if not catalog.is_verified(self.model_type, model_id) and not catalog.unlocked():
            raise ValueError(
                f"Model '{model_id}' is not verified for openpilot "
                f"{catalog.openpilot_version() or 'unknown'}"
            )
```

and delete the now-duplicate resolution line in STEP 2 (`model_id = self.resolve_model_id(model_id)`, currently line 211), leaving `source_dir = self.models_dir / model_id`.

3d. Add `import_stock` as a new method after `list_models`:

```python
    def import_stock(self) -> bool:
        """Import the release's own ONNX as the stock model. Returns True if imported.

        Only runs while the active tracker is absent. An absent tracker proves no
        swap has ever happened, so ACTIVE_DIR still holds the files the release
        shipped; once a swap has run they could be any model, and labeling those
        as the release default would put an untested model behind a trusted name.
        """
        entry = catalog.baseline_entry(self.model_type)
        if not entry or entry.get('source') != 'shipped':
            return False
        if self.active_model_file.exists():
            return False

        dest = self.models_dir / entry['id']
        if dest.exists():
            return False
        if not all((self.ACTIVE_DIR / f).exists() for f in self.onnx_files):
            return False

        # Build in a _-prefixed temp dir (list_models skips those), then rename,
        # so an interrupted copy can never present as a complete model.
        tmp = dest.with_name(f"_{entry['id']}.tmp")
        shutil.rmtree(tmp, ignore_errors=True)
        tmp.mkdir(parents=True)
        for filename in self.onnx_files:
            shutil.copy2(self.ACTIVE_DIR / filename, tmp / filename)
        (tmp / 'model_info.json').write_text(json.dumps({
            'name': entry.get('name', entry['id']),
            'date': entry.get('date', ''),
            'description': entry.get('notes', ''),
            'source': 'shipped',
            'type': self.model_type.value,
        }, indent=2))
        tmp.rename(dest)
        return True
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `PYTHONPATH= uv run pytest plugins/model_selector -q`
Expected: all pass. If any pre-existing test asserted the date filter, update it to the catalog behavior rather than restoring `MIN_MODEL_DATE`.

- [ ] **Step 5: Commit**

```bash
git add plugins/model_selector/model_swapper.py plugins/model_selector/tests/test_model_swapper.py
git commit -m "feat(model_selector): gate swaps on the catalog, import release stock model"
```

---

### Task 4: Gate `model_download.py`

**Files:**
- Modify: `plugins/model_selector/model_download.py` (lines 18-22, 167-200, 226-232, 340-418, 726-735)
- Test: `plugins/model_selector/tests/test_model_download.py`

**Interfaces:**
- Consumes: `catalog.verified_entries`, `catalog.unlocked`, `catalog.openpilot_version` from Task 1.
- Produces, consumed by Task 5:
  - `check_updates()` prints JSON `{"version": str, "verified_total": int, "driving": [...], "dm": [...], "total": int}`; each entry carries `id`, `type`, `name`, `date`
  - `download_model(model_type, model_id, output_dir=None, allow_untested=False) -> int`
  - CLI flag `--unlocked` on the `download` action

- [ ] **Step 1: Write the failing tests**

Append to `plugins/model_selector/tests/test_model_download.py`:

```python
import json


@pytest.fixture
def dl_env(tmp_path, monkeypatch):
  """Point catalog + install dirs at tmp_path for the download/check paths."""
  import plugins.model_selector.catalog as cat
  version_h = tmp_path / 'version.h'
  version_h.write_text('#define COMMA_VERSION "0.11.1"\n')
  monkeypatch.setattr(cat, 'VERSION_H', version_h)
  monkeypatch.setattr(cat, 'CATALOG_FILE', tmp_path / 'catalog.json')
  monkeypatch.setattr(cat, 'UNLOCK_MARKER', tmp_path / '.unlocked')
  (tmp_path / 'catalog.json').write_text(json.dumps({
    'driving': [{'id': 'good_model', 'name': 'Good', 'date': '2025-10-20',
                 'commit': 'c' * 40,
                 'files': ['driving_vision.onnx', 'driving_policy.onnx'],
                 'verified_on': ['0.11.1']},
                {'id': 'stock_0.11.1', 'name': 'Release default', 'date': '2026-05-18',
                 'source': 'shipped', 'verified_on': ['0.11.1'], 'baseline_for': ['0.11.1']}],
    'dm': [],
  }))
  (tmp_path / 'models' / 'driving').mkdir(parents=True)
  (tmp_path / 'models' / 'dm').mkdir(parents=True)
  monkeypatch.setattr(md, 'BASE_DATA_DIR', tmp_path)
  return tmp_path, cat


class TestCheckUpdates:
  def test_offers_verified_uninstalled(self, dl_env, capsys):
    md.check_updates()
    out = json.loads(capsys.readouterr().out)
    assert [m['id'] for m in out['driving']] == ['good_model', 'stock_0.11.1']
    assert out['total'] == 2
    assert out['version'] == '0.11.1'
    assert out['verified_total'] == 2

  def test_skips_installed(self, dl_env, capsys):
    tmp_path, _ = dl_env
    (tmp_path / 'models' / 'driving' / 'good_model').mkdir()
    md.check_updates()
    out = json.loads(capsys.readouterr().out)
    assert [m['id'] for m in out['driving']] == ['stock_0.11.1']

  def test_entries_carry_type(self, dl_env, capsys):
    md.check_updates()
    out = json.loads(capsys.readouterr().out)
    assert all(m['type'] == 'driving' for m in out['driving'])

  def test_unknown_version_offers_nothing(self, dl_env, capsys):
    tmp_path, cat = dl_env
    cat.VERSION_H.write_text('#define COMMA_VERSION "9.9.9"\n')
    md.check_updates()
    out = json.loads(capsys.readouterr().out)
    assert out['total'] == 0
    assert out['verified_total'] == 0


class TestDownloadGate:
  def test_refuses_uncatalogued(self, dl_env, capsys):
    assert md.download_model(md.ModelType.DRIVING, 'mystery_model') == 1
    assert 'not a tested model' in capsys.readouterr().out

  def test_refuses_shipped_entry(self, dl_env, capsys):
    assert md.download_model(md.ModelType.DRIVING, 'stock_0.11.1') == 1
    assert 'imported from disk' in capsys.readouterr().out

  def test_allow_untested_bypasses_the_gate(self, dl_env, capsys):
    # Gate passes, then the registry lookup fails — proving the gate was not
    # what stopped it.
    assert md.download_model(md.ModelType.DRIVING, 'mystery_model', allow_untested=True) == 1
    assert 'not a tested model' not in capsys.readouterr().out

  def test_unlock_marker_bypasses_the_gate(self, dl_env, capsys):
    _, cat = dl_env
    cat.UNLOCK_MARKER.write_text('')
    assert md.download_model(md.ModelType.DRIVING, 'mystery_model') == 1
    assert 'not a tested model' not in capsys.readouterr().out

  def test_verified_model_downloads_from_catalog_metadata(self, dl_env, monkeypatch, capsys):
    calls = []
    monkeypatch.setattr(md, 'download_file', lambda url, dest, desc=None: calls.append(url))
    assert md.download_model(md.ModelType.DRIVING, 'good_model') == 0
    assert len(calls) == 2
    assert all(('c' * 40) in url for url in calls)
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `PYTHONPATH= uv run pytest plugins/model_selector/tests/test_model_download.py -q`
Expected: FAIL — `AttributeError: module 'plugins.model_selector.model_download' has no attribute 'BASE_DATA_DIR'`.

- [ ] **Step 3: Modify `model_download.py`**

3a. Replace the `MIN_MODEL_DATE` import (lines 18-22) with the catalog import and a module-level data dir the tests can point at:

```python
try:
    from plugins.model_selector import catalog
except ImportError:
    import catalog

# Root of model storage. Module-level so tests can redirect it.
BASE_DATA_DIR = Path('/data') if Path('/data').exists() else Path.home() / 'driving_data'
```

3b. In `download_model`, change the signature and replace the registry lookup (the `if model_id not in registry:` block through `model_info = registry[model_id]`) with the catalog-first gate:

```python
def download_model(model_type: ModelType, model_id: str, output_dir: Path = None,
                   allow_untested: bool = False):
    """Download a model from openpilot master at specific commit"""

    # Load registry (maintainer catalogue; the curated catalog takes priority)
    driving_models, dm_models = load_registry()

    if model_type == ModelType.DRIVING:
        registry = driving_models
        type_name = "Driving Model"
        default_dir_name = "models/driving"
    else:
        registry = dm_models
        type_name = "Driver Monitoring Model"
        default_dir_name = "models/dm"

    entry = next((e for e in catalog.verified_entries(model_type) if e['id'] == model_id), None)

    if entry is None and not (allow_untested or catalog.unlocked()):
        version = catalog.openpilot_version() or 'unknown'
        tested = [e['id'] for e in catalog.verified_entries(model_type)]
        print(f"❌ '{model_id}' is not a tested model for openpilot {version}")
        print(f"   Tested: {', '.join(tested) if tested else 'none'}")
        print("   Maintainers: re-run with --unlocked to install an untested model.")
        return 1

    if entry is not None and entry.get('source') == 'shipped':
        print(f"❌ '{model_id}' ships with the release — it is imported from disk, not downloaded")
        return 1

    model_info = entry if entry is not None else registry.get(model_id)

    if model_info is None:
        print(f"❌ {type_name} '{model_id}' not found in registry")
        print(f"\nAvailable {type_name.lower()}s:")
        for mid, info in registry.items():
            print(f"  {mid}: {info['name']} ({info['commit']})")
        return 1
```

3c. Make the two registry-only fields tolerant, since catalog entries use `notes` and have no `description`. Replace the `print(f"Description: {model_info['description']}")` line with:

```python
    description = model_info.get('description') or model_info.get('notes', '')
    print(f"Description: {description}")
```

and in the `info_data` dict replace `'description': model_info['description'],` with `'description': description,`.

3d. Replace `output_dir` derivation to use the module-level constant, so tests can redirect it:

```python
    if output_dir is None:
        output_dir = BASE_DATA_DIR / default_dir_name / model_id
```

3e. Replace the whole body of `check_updates` with the catalog-sourced version:

```python
def check_updates():
    """List tested models not yet installed.

    The catalog is the only source — GitHub is not consulted. Output is JSON for
    the UI to parse.
    """
    result = {'version': catalog.openpilot_version()}
    total = 0
    verified_total = 0

    for type_name in ('driving', 'dm'):
        models_dir = BASE_DATA_DIR / 'models' / type_name
        installed = set()
        if models_dir.exists():
            installed = {d.name for d in models_dir.iterdir()
                         if d.is_dir() and not d.name.startswith('_')}

        verified = catalog.verified_entries(type_name)
        verified_total += len(verified)
        entries = [dict(e, type=type_name) for e in verified if e['id'] not in installed]
        result[type_name] = entries
        total += len(entries)

    result['total'] = total
    result['verified_total'] = verified_total

    print(json.dumps(result))
    return 0
```

3f. In `main()`, add the flag and pass it through:

```python
    parser.add_argument('--unlocked', action='store_true',
                        help='Maintainer: allow installing a model the catalog has not verified')
```

and change the download dispatch to:

```python
        return download_model(model_type, args.model_id, args.output, allow_untested=args.unlocked)
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `PYTHONPATH= uv run pytest plugins/model_selector -q`
Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add plugins/model_selector/model_download.py plugins/model_selector/tests/test_model_download.py
git commit -m "feat(model_selector): source downloads from the catalog, gate untested installs"
```

---

### Task 5: UI — untested markers, blocked activation, stock recovery

**Files:**
- Modify: `plugins/model_selector/ui.py` (lines 12-35, 67-77, 113-121, 150-168, 185-191, 200-215, 265-300, 363-370)

**Interfaces:**
- Consumes: `verified` flag from Task 3's `list_models`, `ModelSwapper.import_stock` from Task 3, `check_updates` JSON from Task 4, `catalog.baseline_entry` / `catalog.openpilot_version` from Task 1.
- Produces: no programmatic interface; this is the last code task.

There are no automated tests for this file — it needs a live raylib surface. Verification is by inspection plus the device check in Task 7.

- [ ] **Step 1: Import catalog and carry the verified flag**

At the top of `ui.py`, beside the existing `from model_swapper import ModelSwapper, ModelType`:

```python
try:
  from plugins.model_selector import catalog
except ImportError:
  import catalog
```

Change `_list_models` to pass the flag through:

```python
def _list_models(model_type):
  models = _SWAPPERS[model_type].list_models()
  return [{'id': m['id'], 'name': m.get('name', m['id']), 'date': m.get('date', ''),
           'verified': m.get('verified', False)}
          for m in models if m.get('has_onnx')]
```

and mark untested models in the label:

```python
def _display_label(model):
  label = f"{model['name']} ({model['date']})" if model['date'] else model['name']
  if not model.get('verified', True):
    label += '  — untested'
  return label
```

- [ ] **Step 2: Block activation of untested models**

In `ModelActionDialog.__init__`, add a `can_activate` parameter:

```python
    def __init__(self, model_name, is_active=False, can_activate=True, callback=None):
```

and change the enable line to:

```python
      self._activate_btn.set_enabled(not is_active and can_activate)
```

At the `ModelActionDialog(...)` construction site inside `on_select`, pass the flag and explain the block:

```python
            action_dlg = ModelActionDialog(_display_label(m), is_active=is_active,
                                           can_activate=m.get('verified', False),
                                           callback=on_action)
            gui_app.push_widget(action_dlg)
```

Guard `on_action` too, so a future caller cannot bypass the disabled button:

```python
              elif r == ACTION_ACTIVATE:
                if not m.get('verified', False):
                  gui_app.push_widget(ConfirmDialog(
                    f'This model has not been tested with openpilot {catalog.openpilot_version()}.',
                    'OK', cancel_text='', callback=lambda _: None))
                  return
                try:
                  _swap_model(mt, mid)
```

- [ ] **Step 3: Warn when the active model is untested, and offer the release default**

Change `show()` to import stock, flag an untested active model, and remember the state:

```python
    def show(self):
      self._untested_active = {}
      for model_type in MODEL_TYPE_LABELS:
        _SWAPPERS[model_type].import_stock()
        self._model_cache[model_type] = _list_models(model_type)
        active_id, active_name = _read_active(model_type)
        if active_id and not catalog.is_verified(model_type, active_id):
          self._untested_active[model_type] = active_id
          active_name = f'{active_name} — untested'
        self._model_btns[model_type].action_item.set_value(active_name)
      self._set_status(f'up to date, last checked {self._time_ago()}')
```

Initialise `self._untested_active = {}` in `__init__` too, so `_on_model_select` is safe before the first `show()`.

At the top of `_on_model_select`, offer the one-tap recovery before the normal list:

```python
    def _on_model_select(self, model_type):
      models = self._model_cache.get(model_type, [])
      if not models:
        return

      baseline = catalog.baseline_entry(model_type)
      if model_type in self._untested_active and baseline:
        installed = {m['id'] for m in models}
        if baseline['id'] in installed:
          def on_recover(r, mt=model_type, bid=baseline['id']):
            if r == DialogResult.CONFIRM:
              self._activate(mt, bid)
            else:
              self._untested_active.pop(mt, None)
              self._on_model_select(mt)

          gui_app.push_widget(ConfirmDialog(
            f"The active model is not tested with openpilot {catalog.openpilot_version()}. "
            f"Switch to {baseline.get('name', bid_default(baseline))}?",
            'Switch', cancel_text='Keep', callback=on_recover))
          return
      ...
```

Replace `bid_default(baseline)` with `baseline['id']` — written out:

```python
            f"Switch to {baseline.get('name', baseline['id'])}?",
```

- [ ] **Step 4: Extract the activate-and-reboot flow so recovery can reuse it**

The activation body inside `on_action` and the recovery path need the same swap + reboot prompt. Add a method on `ModelSelectorUI`:

```python
    def _activate(self, model_type, model_id):
      """Swap to model_id and prompt for the reboot that makes it take effect."""
      prev = _read_active(model_type)[0]
      try:
        _swap_model(model_type, model_id)
      except Exception:
        gui_app.push_widget(ConfirmDialog('Model swap failed.', 'OK', cancel_text='',
                                          callback=lambda _: None))
        return

      _, new_name = _read_active(model_type)
      if model_type in self._model_btns:
        self._model_btns[model_type].action_item.set_value(new_name)

      def on_reboot(r2, mt2=model_type, prev2=prev):
        if r2 == DialogResult.CONFIRM:
          ui_state.params.put_bool_nonblocking("DoReboot", True)
          return
        # Canceling reverts, so a canceled activation is a no-op. Reverting to an
        # untested previous model is refused by the gate; the new model then
        # simply stays, which is the safer of the two outcomes.
        try:
          _swap_model(mt2, prev2)
          _, reverted_name = _read_active(mt2)
          if mt2 in self._model_btns:
            self._model_btns[mt2].action_item.set_value(reverted_name)
        except Exception:
          pass

      gui_app.push_widget(ConfirmDialog('Model swapped. Reboot to activate.', 'Reboot',
                                        cancel_text='Cancel', callback=on_reboot))
```

Then replace the activation branch in `on_action` with a call to it:

```python
              elif r == ACTION_ACTIVATE:
                if not m.get('verified', False):
                  gui_app.push_widget(ConfirmDialog(
                    f'This model has not been tested with openpilot {catalog.openpilot_version()}.',
                    'OK', cancel_text='', callback=lambda _: None))
                  return
                self._activate(mt, mid)
```

- [ ] **Step 5: CHECK becomes a local catalog diff**

Rename the row and drop the GitHub scrape. In `__init__`:

```python
      self._new_models_btn = button_item('Tested Models', 'CHECK', callback=self._on_check_new_models)
```

In `_on_check_new_models`, replace the two-command `bash -c` Popen with a direct call:

```python
      self._check_proc = subprocess.Popen(
        [PYTHON_BIN, str(script), 'check-updates'],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
      )
```

In `_on_check_complete`, distinguish "nothing new" from "nothing tested for this version":

```python
      total = updates.get('total', 0)
      self._last_check_time = time.monotonic()
      if total == 0:
        if updates.get('verified_total', 0) == 0:
          self._set_status(f"no tested models for {updates.get('version', 'this version')}")
        else:
          self._set_status('up to date, last checked now')
        return
```

and change the dialog title from `'New Models'` to `'Tested Models'`.

- [ ] **Step 6: Import stock from the health-check hook**

In `on_health_check`, alongside the existing `cache_compiled_pkl` calls, add the idempotent import so a fresh device gets its stock entry without opening the panel:

```python
    for model_type in ('driving', 'dm'):
      try:
        _SWAPPERS[model_type].import_stock()
      except Exception:
        pass
```

Place it before the existing PKL caching loop, and keep the existing return value unchanged.

- [ ] **Step 7: Verify the module still imports and the suite passes**

Run: `PYTHONPATH= uv run python -c "import ast,sys; ast.parse(open('plugins/model_selector/ui.py').read())" && PYTHONPATH= uv run pytest plugins/model_selector -q`
Expected: no syntax error; all tests pass. (`ui.py` cannot be imported off-device — it needs raylib — so an AST parse is the available check.)

- [ ] **Step 8: Commit**

```bash
git add plugins/model_selector/ui.py
git commit -m "feat(model_selector): mark untested models, block activation, offer release default"
```

---

### Task 6: Documentation

**Files:**
- Modify: `plugins/model_selector/DESIGN.md` (the "Model source", "Compatibility filtering", "Config / paths", "Key files" sections)
- Modify: `plugins/model_selector/README.md` ("How to use it", "Things to know")

**Interfaces:**
- Consumes: the finished behavior from Tasks 1-5.
- Produces: nothing programmatic.

- [ ] **Step 1: Rewrite DESIGN.md's "Compatibility filtering" section**

Replace the whole three-gate section with:

```markdown
## Compatibility gating

A model may be downloaded or activated only if `compatible_models.json` — the
curated catalog shipped at the plugin root — records that it passed a test drive
on the openpilot version this device runs. `catalog.py` owns that policy; the
date heuristics it replaced (`MIN_MODEL_DATE`, the revert-name filter) are gone.

- **Catalog** — `compatible_models.json`, at the plugin root so every deploy
  overwrites it. `data/` is preserved across reinstalls and would freeze a
  catalog placed there. Entries carry `verified_on: ["0.11.1", …]`; one entry per
  type per version also carries `baseline_for`, marking the known-good fallback.
- **Version** — read from `/data/openpilot/common/version.h`, falling back to
  `manifest.OPENPILOT_VERSION`.
- **Shipped models** — the ONNX a release ships is verified by definition. Its
  entry carries `source: "shipped"` and no `commit`; `ModelSwapper.import_stock`
  copies the files out of `ACTIVE_DIR` into storage, but only while the active
  tracker is absent, which proves no swap has happened yet.
- **Fail closed** — a missing or corrupt catalog yields zero verified models.
- **Unlock** — `<PLUGINS_RUNTIME_DIR>/model_selector/data/.unlocked` (or
  `download --unlocked`) lets a maintainer install and activate untested models.
  It has no UI surface and survives reinstalls, since `data/` is preserved.

Enforced at three call sites so no path is ungated: `check_updates` (what is
offered), `download_model` (what is fetched), `swap_model` (what is activated).
```

- [ ] **Step 2: Update DESIGN.md's surrounding sections**

In "Model source", change the registry paragraph to note that `update-registry` /
`add-from-pr` / `list` are now maintainer-only CLI tools that no longer feed the
UI; the UI reads the catalog. In "Software-panel injection", change the **New
Models** row to **Tested Models** and note that CHECK is a local diff needing no
network. In "Config / paths", drop the two `MIN_MODEL_DATE` rows and add
`CATALOG_FILE`, `VERSION_H`, and `UNLOCK_MARKER`. In "Key files", add
`catalog.py` and `compatible_models.json`, and add `tests/test_catalog.py`.

- [ ] **Step 3: Update README.md**

Replace the **New Models** bullet under "How to use it" with:

```markdown
- **Tested Models** — tap **CHECK**. It lists the models that have been test
  driven on your openpilot version and aren't installed yet, and lets you
  download one. The list ships with the plugin, so it updates when catpilot
  does.
```

Replace the "Old models are hidden" bullet under "Things to know" with:

```markdown
- **Only tested models can be selected.** Every model offered here has been
  test driven on your openpilot version, along with the model your release
  ships. Anything else on the device is shown as *untested* and can't be
  activated.
- **After a catpilot update, a model you were using may become untested.**
  It keeps running — nothing changes under you — but the panel flags it and
  offers to switch you back to the model your release ships.
```

- [ ] **Step 4: Commit**

```bash
git add plugins/model_selector/DESIGN.md plugins/model_selector/README.md
git commit -m "docs(model_selector): document catalog gating"
```

---

### Task 7: Full-suite check and deploy notes

**Files:** none modified.

- [ ] **Step 1: Run the whole plugin suite**

Run: `PYTHONPATH= uv run pytest plugins -q`
Expected: no regressions against the pre-change baseline of 13 passed, 1 skipped in `plugins/model_selector` (the wider `plugins` suite includes other plugins; compare failures, not totals).

- [ ] **Step 2: Confirm no unrelated files are staged or committed**

Run: `git status --short && git log --oneline origin/dev..HEAD`
Expected: `plugins/c3_compat/boot_patch.sh` still shows as modified-but-uncommitted, `overlays/` still untracked, and the commit list contains only this feature's commits.

- [ ] **Step 3: Hand the deploy to the user**

Do not run these. Report them as the next step for the user to run when they choose to deploy:

```bash
git push origin dev
ssh c3 'cd /data/plugins && GIT_SSL_NO_VERIFY=1 git fetch origin dev && git reset --hard origin/dev && bash install.sh'
```

On-device verification after the deploy, for the user:

```bash
# the catalog shipped, and the gate sees the right version
ssh c3 'source /usr/local/venv/bin/activate && cd /data/plugins-runtime/model_selector && python -c "import catalog; print(catalog.openpilot_version(), [e[\"id\"] for e in catalog.verified_entries(\"driving\")])"'
# CHECK's data path, without touching the UI
ssh c3 'source /usr/local/venv/bin/activate && cd /data/plugins-runtime/model_selector && python model_download.py check-updates'
```

Note for the user: this device has already swapped models, so its active tracker
exists and `import_stock` will correctly decline to import. Its currently active
model will show as **untested** until it is added to `compatible_models.json`
with `verified_on: ["0.11.1"]` — that entry is a judgement call about a drive
already done, so it is left to the user rather than guessed here.
