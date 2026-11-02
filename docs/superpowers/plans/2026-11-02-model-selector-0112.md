# Model Selector 0.11.2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the model_selector plugin work on openpilot 0.11.2's single-file supercombo format and carry the POP driving model over as a converted, downloadable file.

**Architecture:** The plugin keeps its three modules (`catalog.py` policy, `model_download.py` fetch, `model_swapper.py` storage/activation) and only changes what the format change forces: one driving ONNX, one compiled artefact, byte-proof stock import, a direct-URL download kind, and a drift check. A separate PC-only tool converts a legacy split model into a supercombo by swapping policy weights inside the stock file; its output is hosted as a GitHub release asset and listed in the catalog.

**Tech Stack:** Python 3.12, pytest, onnx + onnxruntime + numpy (tool and its tests only), requests (existing downloader).

**Spec:** `docs/superpowers/specs/2026-11-02-model-selector-0112-design.md` (plugins repo). Spike evidence: `/home/oxygen/catpilot-dev/spike_pop_supercombo/NOTES.md`.

## Global Constraints

- Repo: `/home/oxygen/catpilot-dev/plugins`, branch `rebase/v0.11.2`. Never push; the user pushes. No `Co-Authored-By` lines.
- Tree paths at 0.11.2: `ACTIVE_DIR = /data/openpilot/openpilot/selfdrive/modeld/models`, version at `/data/openpilot/openpilot/common/version.h`.
- Driving model is exactly `driving_supercombo.onnx`; its compiled artefact is `driving_tinygrad.pkl`. DM is `dmonitoring_model.onnx` / `dmonitoring_model_tinygrad.pkl`. Nothing else in `ACTIVE_DIR` may be deleted or overwritten by the plugin (`big_*`, `*_warp_*` files are shared or git-tracked).
- Shipped 0.11.2 hashes (from the tag's LFS pointers): driving `65a08adc31d5c456219687d99b7bf5e44d61dae2d49ea67850e76105c7248cce`, 60918562 bytes; dm `dee5a294e8afaacc9295ac5d100e00733ecac278e79264b96331e40a3ede1b04`, 7844499 bytes.
- POP converted file: produced by the tool in Task 6 (vision backbone from the template, POP's retrained hidden-state summarizer head transplanted, POP's policy swapped in); checkpoint `6a7d09ad-bcc9-43bc-916d-29287e60cee2/200/05a58a51-e0e3-4e9b-8e27-e644685f2c50/100`. Its sha256 and size are whatever the tool prints; Task 7 copies them from the Task 6 report. (Corrected 2026-11-02 during execution: the spike's file `147f8c4e…` ran POP's policy on CD210's features — POP's `summarizer.*` head differs from CD210's — and is superseded.)
- Catalog gate semantics unchanged: offered/activated only when `verified_on` contains the running version; exactly one `baseline_for` per type per version; the unlock marker opens the CLI only, never the UI.
- Plugin UI imports stay lazy inside hook functions. Two-space indentation in plugin code (existing style); the tool follows the same.
- Test command: `cd /home/oxygen/catpilot-dev/plugins && PYTHONPATH=. uv run pytest plugins/model_selector/tests .github/scripts/tests/test_model_watch.py -q` (baseline before Task 1: 86 + 35 passed). Do not use the catpilot venv's python: it can import `openpilot.selfdrive.plugins.manifest`, which makes two `TestOpenpilotVersion` fallback tests fail for reasons unrelated to this work.
- The device is not touched by any task. The one-off rollout (Appendix) runs only after the C3 boots catpilot 0.11.2.

## Review Focus

1. `ACTIVE_DIR` holds a 133-byte LFS pointer instead of a model after a deploy: it must never be imported as stock and must count as drift. Test in Task 3 (`test_pointer_sized_file_is_not_imported`) and Task 5 (`test_active_file_matches_false_for_pointer`).
2. A download interrupted midway must leave no directory that lists as a complete model. Test in Task 4 (`test_interrupted_download_leaves_only_part_file`).
3. A legacy split directory (vision + policy, no supercombo) must not be activatable even when unlocked. Test in Task 1 (`test_legacy_split_dir_cannot_be_activated_even_unlocked`).
4. A device whose running version has no baseline in the catalog must fail closed everywhere (no import, nothing offered). Test in Task 3 (`test_no_baseline_for_version_imports_nothing`) and existing `test_unknown_version_offers_nothing`.
5. A swap on a device that never compiled (`driving_tinygrad.pkl` absent) must succeed and leave every other file in `ACTIVE_DIR` untouched. Test in Task 1 (`test_swap_leaves_other_compiled_files_alone`).

---

### Task 1: Single-file layout in the swapper and the watcher

**Files:**
- Modify: `plugins/model_selector/model_swapper.py` (lines 44-45 paths; 60-90 `MODEL_CONFIGS`; 108-118 attributes and `_glob_compiled_files`; 292-316 swap step 3; 360-386 `cache_compiled_pkl`; 428-476 `verify_model`)
- Modify: `plugins/model_selector/catalog.py:17` (`VERSION_H`)
- Modify: `.github/scripts/model_watch.py:31,41-44` (`MODEL_PATH`, `WATCH_FILES`)
- Test: `plugins/model_selector/tests/test_model_swapper.py`, `plugins/model_selector/tests/test_catalog.py`, `.github/scripts/tests/test_model_watch.py`

**Interfaces:**
- Produces: `MODEL_CONFIGS[ModelType.DRIVING] = {..., 'onnx_files': ['driving_supercombo.onnx'], 'compiled_file': 'driving_tinygrad.pkl'}`, `MODEL_CONFIGS[ModelType.DM] = {..., 'onnx_files': ['dmonitoring_model.onnx'], 'compiled_file': 'dmonitoring_model_tinygrad.pkl'}`; the keys `pkl_patterns` and `required_pkl_stems` are removed. Instance attribute `self.compiled_file: str`. `ModelSwapper.ACTIVE_DIR = OPENPILOT_DIR / 'openpilot' / 'selfdrive' / 'modeld' / 'models'`. `_compiled_files(directory: Path) -> list[str]` returns `[compiled_file]` when present, else `[]`.
- Later tasks rely on `self.onnx_files[0]` being the single model file and on `self.compiled_file`.

- [ ] **Step 1: Replace the config tests and add the layout tests**

In `TestModelConfigs` replace `test_driving_config`, `test_dm_config`, `test_pkl_patterns_match_onnx` with:

```python
  def test_driving_config(self, ModelSwapper, ModelType):
    cfg = ModelSwapper.MODEL_CONFIGS[ModelType.DRIVING]
    assert cfg['onnx_files'] == ['driving_supercombo.onnx']
    assert cfg['compiled_file'] == 'driving_tinygrad.pkl'
    assert 'pkl_patterns' not in cfg and 'required_pkl_stems' not in cfg

  def test_dm_config(self, ModelSwapper, ModelType):
    cfg = ModelSwapper.MODEL_CONFIGS[ModelType.DM]
    assert cfg['onnx_files'] == ['dmonitoring_model.onnx']
    assert cfg['compiled_file'] == 'dmonitoring_model_tinygrad.pkl'

  def test_active_dir_is_the_nested_openpilot_package(self, ModelSwapper):
    assert ModelSwapper.ACTIVE_DIR.parts[-4:] == ('openpilot', 'selfdrive', 'modeld', 'models')
```

Add to `TestCatalogGate` (uses `gated`, `_install`):

```python
  def test_swap_leaves_other_compiled_files_alone(self, gated):
    sw, cat, _ = gated
    cat.UNLOCK_MARKER.write_text('')
    self._install(sw, 'good_model')
    others = ['big_driving_tinygrad.pkl', 'big_driving_warp_1928x1208_tinygrad.pkl',
              'driving_warp_1928x1208_tinygrad.pkl', 'dm_warp_1928x1208_tinygrad.pkl',
              'dmonitoring_model_tinygrad.pkl', 'dmonitoring_model.onnx']
    for f in others:
      (sw.ACTIVE_DIR / f).write_bytes(b'x')
    (sw.ACTIVE_DIR / 'driving_tinygrad.pkl').write_bytes(b'stale')
    sw.swap_model('good_model')  # driving_tinygrad.pkl absent or stale: both must work
    assert not (sw.ACTIVE_DIR / 'driving_tinygrad.pkl').exists()
    assert all((sw.ACTIVE_DIR / f).exists() for f in others)
    assert (sw.ACTIVE_DIR / 'driving_supercombo.onnx').read_bytes() == b'onnx'

  def test_swap_succeeds_when_nothing_was_compiled_yet(self, gated):
    sw, cat, _ = gated
    cat.UNLOCK_MARKER.write_text('')
    self._install(sw, 'good_model')
    assert sw.swap_model('good_model')['requires_reboot'] is True

  def test_cache_copies_only_the_models_own_artifact(self, gated):
    sw, _, _ = gated
    d = self._install(sw, 'good_model')
    for f in ['driving_tinygrad.pkl', 'big_driving_tinygrad.pkl', 'driving_warp_1928x1208_tinygrad.pkl']:
      (sw.ACTIVE_DIR / f).write_bytes(b'c')
    assert sw.cache_compiled_pkl('good_model') == 1
    assert (d / 'driving_tinygrad.pkl').exists()
    assert not (d / 'big_driving_tinygrad.pkl').exists()

  def test_legacy_split_dir_cannot_be_activated_even_unlocked(self, gated):
    sw, cat, _ = gated
    cat.UNLOCK_MARKER.write_text('')
    d = sw.models_dir / 'pop_model_37727'
    d.mkdir()
    (d / 'driving_vision.onnx').write_bytes(b'v'); (d / 'driving_policy.onnx').write_bytes(b'p')
    (d / 'model_info.json').write_text(json.dumps({'name': 'POP', 'date': '2026-03-24'}))
    with pytest.raises(ValueError, match='missing required ONNX'):
      sw.swap_model('pop_model_37727')
    assert all(not m['has_onnx'] for m in sw.list_models() if m['id'] == 'pop_model_37727')
```

In `test_catalog.py` add to `TestOpenpilotVersion`:

```python
  def test_default_version_h_is_in_the_nested_package(self):
    import plugins.model_selector.catalog as mod
    assert mod.VERSION_H.parts[-3:] == ('openpilot', 'common', 'version.h')
```

Move the shared fixtures to 0.11.2 in the same step, so every later task builds on them:
- `test_model_swapper.py`: `CATALOG_FIXTURE` becomes `{'driving': [{'id': 'good_model', 'name': 'Good', 'date': '2025-10-20', 'commit': 'c'*40, 'files': ['driving_supercombo.onnx'], 'verified_on': ['0.11.2']}, {'id': 'stock_0.11.2', 'name': 'Release default', 'date': '2026-10-14', 'source': 'shipped', 'sha256': hashlib.sha256(b'shipped-onnx').hexdigest(), 'size': 12, 'verified_on': ['0.11.2'], 'baseline_for': ['0.11.2']}], 'dm': []}` (add `import hashlib`); `_point_catalog_at` writes `COMMA_VERSION "0.11.2"`; `_build_swapper` sets `sw.compiled_file = sw.config['compiled_file']` instead of the two removed attributes; `TestImportStock` assertions name `driving_supercombo.onnx` and `stock_0.11.2` (Task 3 rewrites that class anyway).
- `test_model_download.py`: `dl_env` writes version `0.11.2`, `good_model.files = ['driving_supercombo.onnx']`, and the shipped entry as `stock_0.11.2` with the same `sha256`/`size` as above and `verified_on`/`baseline_for` `['0.11.2']`; every `'stock_0.11.1'`/`'0.11.1'` literal in that file follows.

- [ ] **Step 2: Run the three test files, verify the new tests fail**

Run: `PYTHONPATH=. uv run pytest plugins/model_selector/tests/test_model_swapper.py plugins/model_selector/tests/test_catalog.py -q`
Expected: FAIL on `compiled_file` KeyError / `AttributeError` and the `ACTIVE_DIR` parts assertion; watcher test untouched for now.

- [ ] **Step 3: Implement the layout change in `model_swapper.py`**

`ACTIVE_DIR` as in Interfaces. `MODEL_CONFIGS` as in Interfaces (keep `models_dir`, `active_file`, `display_name`). `__init__` sets `self.compiled_file`. Replace `_glob_compiled_files` with `_compiled_files(self, directory: Path) -> list[str]`. In `swap_model` step 3 delete only `self.ACTIVE_DIR / self.compiled_file` (if it exists), and copy a cached artefact only when `pkl_compatible` and `source_dir / self.compiled_file` exists. `cache_compiled_pkl` copies only `self.ACTIVE_DIR / self.compiled_file`. `list_models`, `verify_model` and the `swap_model` result use `_compiled_files` and `total_pkl_count = 1`. Update the module docstring (lines 5-6) and `swap_model`'s docstring to name the single file.

- [ ] **Step 4: Point `catalog.VERSION_H` at the nested package**

`VERSION_H = Path(os.getenv('OPENPILOT_DIR', '/data/openpilot')) / 'openpilot' / 'common' / 'version.h'`.

- [ ] **Step 5: Run the two plugin test files, verify they pass**

Run: `PYTHONPATH=. uv run pytest plugins/model_selector/tests/test_model_swapper.py plugins/model_selector/tests/test_catalog.py -q`
Expected: PASS.

- [ ] **Step 6: Update the watcher's file list and path, run its test**

In `model_watch.py`: `MODEL_PATH = 'openpilot/selfdrive/modeld/models'`, `WATCH_FILES = {'driving': ['driving_supercombo.onnx'], 'dm': ['dmonitoring_model.onnx']}`; rewrite the comment above it to say the single supercombo is what this fork loads since 0.11.2. Run: `PYTHONPATH=. uv run pytest .github/scripts/tests/test_model_watch.py -q`. Expected: PASS (the drift guard compares against `MODEL_CONFIGS`).

- [ ] **Step 7: Run the whole plugin suite and commit**

Run: `PYTHONPATH=. uv run pytest plugins/model_selector/tests .github/scripts/tests/test_model_watch.py -q`. Expected: PASS.

```bash
git add plugins/model_selector/model_swapper.py plugins/model_selector/catalog.py .github/scripts/model_watch.py plugins/model_selector/tests .github/scripts/tests/test_model_watch.py
git commit -m "model_selector: single-file supercombo layout for openpilot 0.11.2"
```

---

### Task 2: Catalog schema for 0.11.2: shipped hashes and the converted kind

**Files:**
- Modify: `plugins/model_selector/catalog.py:97-134` (`validate_catalog`), add `entry()` helper
- Modify: `plugins/model_selector/compatible_models.json` (rewrite)
- Test: `plugins/model_selector/tests/test_catalog.py`

**Interfaces:**
- Produces: `catalog.entry(model_type, model_id: str) -> dict | None` returning the catalog entry regardless of version. `validate_catalog` rules: every entry needs `id, name, date, verified_on`; `source == 'shipped'` additionally needs `sha256` (64 lowercase hex) and `size` (int > 0); `source == 'converted'` needs `converted_from` (dict with `commit`) and `files` as a non-empty list of dicts each with `name`, `url` (starts with `https://`), `sha256` (64 hex), `size` (int > 0); any other entry needs `commit` and `files` as a list of filenames.
- Catalog contents for this branch: driving `stock_0.11.2` (shipped, driving hash and size from Global Constraints, `verified_on` and `baseline_for` `["0.11.2"]`, `notes` "CD210 as driving_supercombo.onnx shipped in catpilot v0.11.2", `date` "2026-10-14"), dm `stock_dm_0.11.2` (shipped, dm hash and size, same versions, `notes` "Super Leicht dmonitoring_model.onnx shipped in catpilot v0.11.2", `date` "2026-09-18"). No 0.11.1 entries. The POP entry is added in Task 7.

- [ ] **Step 1: Write the failing tests**

Replace `TestShippedCatalog` and extend `TestValidateCatalog` and `_stock`/`_entry` helpers:

```python
def _stock(**over):
  e = {'id': 'stock_0.11.2', 'name': 'Release default', 'date': '2026-10-14', 'source': 'shipped',
       'sha256': '65a08adc31d5c456219687d99b7bf5e44d61dae2d49ea67850e76105c7248cce', 'size': 60918562,
       'verified_on': ['0.11.2'], 'baseline_for': ['0.11.2']}
  e.update(over); return e

def _converted(**over):
  e = {'id': 'pop_supercombo_37727', 'name': 'POP model', 'date': '2026-03-24', 'source': 'converted',
       'converted_from': {'commit': '1' * 40, 'vision': 'v/200', 'policy': 'p/100'},
       'files': [{'name': 'driving_supercombo.onnx', 'url': 'https://example.invalid/pop.onnx',
                  'sha256': 'a' * 64, 'size': 60918575}],
       'verified_on': []}
  e.update(over); return e

class TestValidateCatalog:
  ...existing tests, with _entry() still a commit-kind entry...
  def test_shipped_entry_needs_sha256_and_size(self, catalog_env):
    problems = catalog_env.validate_catalog({'driving': [_stock(sha256=None, size=0)], 'dm': []})
    assert any('sha256' in p for p in problems) and any('size' in p for p in problems)

  def test_shipped_sha256_must_be_64_hex(self, catalog_env):
    assert catalog_env.validate_catalog({'driving': [_stock(sha256='xyz')], 'dm': []})

  def test_converted_entry_is_valid(self, catalog_env):
    assert catalog_env.validate_catalog({'driving': [_stock(), _converted()], 'dm': []}) == []

  def test_converted_entry_needs_file_objects(self, catalog_env):
    bad = _converted(files=['driving_supercombo.onnx'])
    assert any('files' in p for p in catalog_env.validate_catalog({'driving': [_stock(), bad], 'dm': []}))

  def test_converted_file_needs_url_sha256_size(self, catalog_env):
    bad = _converted(files=[{'name': 'driving_supercombo.onnx'}])
    problems = catalog_env.validate_catalog({'driving': [_stock(), bad], 'dm': []})
    assert any('url' in p for p in problems) and any('sha256' in p for p in problems) and any('size' in p for p in problems)

  def test_converted_entry_needs_converted_from(self, catalog_env):
    bad = _converted(converted_from=None)
    assert any('converted_from' in p for p in catalog_env.validate_catalog({'driving': [_stock(), bad], 'dm': []}))

class TestEntry:
  def test_entry_is_found_regardless_of_version(self, catalog_env):
    _write(catalog_env, {'driving': [_stock(), _converted()], 'dm': []})
    assert catalog_env.entry('driving', 'pop_supercombo_37727')['source'] == 'converted'
    assert catalog_env.is_verified('driving', 'pop_supercombo_37727') is False

  def test_entry_none_for_unknown(self, catalog_env):
    _write(catalog_env, {'driving': [_stock()], 'dm': []})
    assert catalog_env.entry('driving', 'nope') is None

class TestShippedCatalog:
  def test_shipped_catalog_is_valid(self, cat):
    assert cat.validate_catalog() == []

  def test_shipped_catalog_has_baselines_for_0_11_2_only(self, cat):
    c = cat.load_catalog()
    for t in ('driving', 'dm'):
      assert [e['id'] for e in c[t] if '0.11.2' in e.get('baseline_for', [])] == [f"stock_{'' if t == 'driving' else 'dm_'}0.11.2"]
      assert all('0.11.1' not in e.get('verified_on', []) for e in c[t])

  def test_shipped_entries_carry_the_tag_hashes(self, cat):
    c = cat.load_catalog()
    d = next(e for e in c['driving'] if e['id'] == 'stock_0.11.2')
    assert (d['sha256'], d['size']) == ('65a08adc31d5c456219687d99b7bf5e44d61dae2d49ea67850e76105c7248cce', 60918562)
    m = next(e for e in c['dm'] if e['id'] == 'stock_dm_0.11.2')
    assert (m['sha256'], m['size']) == ('dee5a294e8afaacc9295ac5d100e00733ecac278e79264b96331e40a3ede1b04', 7844499)
```

- [ ] **Step 2: Run, verify failure**

Run: `PYTHONPATH=. uv run pytest plugins/model_selector/tests/test_catalog.py -q`
Expected: FAIL (`entry` missing; validation does not report sha256/size/files problems; shipped catalog still 0.11.1).

- [ ] **Step 3: Implement `entry()` and the validation rules in `catalog.py`; rewrite `compatible_models.json`**

`entry()` reads `load_catalog()`. Validation per Interfaces; messages contain the field name (`"driving: entry 'x' missing sha256"`, `"... bad sha256"`, `"... files must be objects with name/url/sha256/size"`). The catalog JSON holds exactly the two shipped entries described in Interfaces.

- [ ] **Step 4: Run the catalog tests and the full suite, verify pass**

Run: `PYTHONPATH=. uv run pytest plugins/model_selector/tests -q`. Expected: PASS (fixtures from Task 1 already use 0.11.2-style entries where they construct catalogs; if any fixture still carries `stock_0.11.1` without a hash, update it to `_stock()`-shaped data).

- [ ] **Step 5: Commit**

```bash
git add plugins/model_selector/catalog.py plugins/model_selector/compatible_models.json plugins/model_selector/tests/test_catalog.py plugins/model_selector/tests
git commit -m "model_selector: 0.11.2 catalog — shipped hashes, converted kind, entry()"
```

---

### Task 3: Stock import by byte proof, stale tracker migration

**Files:**
- Modify: `plugins/model_selector/model_swapper.py:161-205` (`import_stock`), add `_file_sha256`, `_tracker_is_stale`
- Test: `plugins/model_selector/tests/test_model_swapper.py` (`TestImportStock`)

**Interfaces:**
- Consumes: `catalog.baseline_entry(model_type)` with `sha256`/`size` (Task 2).
- Produces: `ModelSwapper._file_sha256(path: Path) -> str` (chunked, 1 MiB reads). `ModelSwapper.import_stock() -> bool`: returns True only when it created the stock directory this call. Rule: baseline entry must be `shipped` with `sha256` and `size`; the active file `ACTIVE_DIR / onnx_files[0]` must exist, have `st_size == size`, and hash to `sha256`; the tracker is irrelevant to the decision. After a successful import, or when the stock dir already exists, `_tracker_is_stale()` is checked: True when the tracker is missing or names a model dir that lacks any file in `onnx_files`; a stale tracker is rewritten to `{'id': <stock id>, 'name': <entry name>, 'tinygrad': get_tinygrad_commit()}`. Returns False when the stock dir already existed (tracker repair may still happen).

- [ ] **Step 1: Rewrite `TestImportStock`**

Fixture `stocked`: `_point_catalog_at(tmp_path, monkeypatch, CATALOG_FIXTURE)` (already carries `stock_0.11.2` with the sha256 of `b'shipped-onnx'`, size 12, since Task 1), `_build_swapper(...)`, and `ACTIVE_DIR / 'driving_supercombo.onnx'` written as `b'shipped-onnx'`.

```python
  def test_imports_when_active_matches_shipped_hash(self, stocked):
    assert stocked.import_stock() is True
    dest = stocked.models_dir / 'stock_0.11.2'
    assert (dest / 'driving_supercombo.onnx').read_bytes() == b'shipped-onnx'
    assert json.loads((dest / 'model_info.json').read_text())['source'] == 'shipped'

  def test_imports_even_when_a_tracker_exists(self, stocked):
    stocked.active_model_file.write_text(json.dumps({'id': 'something_else'}))
    assert stocked.import_stock() is True

  def test_skips_when_active_bytes_differ(self, stocked):
    (stocked.ACTIVE_DIR / 'driving_supercombo.onnx').write_bytes(b'swapped-onnx')  # same length, other bytes
    assert stocked.import_stock() is False
    assert not (stocked.models_dir / 'stock_0.11.2').exists()

  def test_pointer_sized_file_is_not_imported(self, stocked):
    (stocked.ACTIVE_DIR / 'driving_supercombo.onnx').write_text('version https://git-lfs.github.com/spec/v1\noid sha256:65a0\nsize 60918562\n')
    assert stocked.import_stock() is False

  def test_stale_legacy_tracker_is_rewritten_to_stock(self, stocked):
    legacy = stocked.models_dir / 'pop_model_37727'; legacy.mkdir()
    (legacy / 'driving_vision.onnx').write_bytes(b'v')
    stocked.active_model_file.write_text(json.dumps({'id': 'pop_model_37727', 'name': 'POP'}))
    stocked.import_stock()
    assert json.loads(stocked.active_model_file.read_text())['id'] == 'stock_0.11.2'

  def test_missing_tracker_is_written_to_stock(self, stocked):
    stocked.import_stock()
    assert json.loads(stocked.active_model_file.read_text())['id'] == 'stock_0.11.2'

  def test_tracker_naming_a_loadable_model_is_kept(self, stocked):
    good = stocked.models_dir / 'good_model'; good.mkdir()
    (good / 'driving_supercombo.onnx').write_bytes(b'onnx')
    stocked.active_model_file.write_text(json.dumps({'id': 'good_model'}))
    stocked.import_stock()
    assert json.loads(stocked.active_model_file.read_text())['id'] == 'good_model'

  def test_no_baseline_for_version_imports_nothing(self, stocked, tmp_path):
    (tmp_path / 'version.h').write_text('#define COMMA_VERSION "0.11.3"\n')
    assert stocked.import_stock() is False
    assert not stocked.active_model_file.exists()
```

Keep `test_is_idempotent`, `test_imported_stock_is_listed_and_verified`, `test_failed_copy_does_not_leave_an_orphaned_temp_dir`, `test_successful_import_still_lands_atomically`, `test_concurrent_callers_use_distinct_temp_dirs` (rename ids to `stock_0.11.2`). Delete `test_skips_when_tracker_exists` and `test_skips_when_active_dir_incomplete` (the hash rule covers both).

- [ ] **Step 2: Run, verify failure**

Run: `PYTHONPATH=. uv run pytest plugins/model_selector/tests/test_model_swapper.py -k ImportStock -q`
Expected: FAIL (`test_imports_even_when_a_tracker_exists`, the tracker tests, pointer test).

- [ ] **Step 3: Implement per Interfaces**

Rewrite the `import_stock` docstring: the proof is now the shipped hash, not tracker absence. Check `st_size` before hashing so a pointer or foreign file is rejected without reading 60 MB.

- [ ] **Step 4: Run the swapper tests, then the suite, verify pass; commit**

```bash
git add plugins/model_selector/model_swapper.py plugins/model_selector/tests/test_model_swapper.py
git commit -m "model_selector: import stock by shipped hash, repair stale trackers"
```

---

### Task 4: Direct-URL downloads with hash verification; unlocked CLI reaches untested catalog entries

**Files:**
- Modify: `plugins/model_selector/model_download.py:84-130` (add `download_direct`), `172-310` (`download_model`), `357-392` (`check_updates`)
- Test: `plugins/model_selector/tests/test_model_download.py`

**Interfaces:**
- Consumes: `catalog.entry()` (Task 2).
- Produces: `download_direct(url: str, dest: Path, sha256: str, size: int) -> None`: streams with `requests.get(url, stream=True, timeout=600)` to `dest.with_name(dest.name + '.part')`, verifies size and sha256, renames onto `dest`; on mismatch deletes the `.part` file and raises `ValueError('sha256 mismatch for <name>')`. `download_model(...)`: entry lookup is `catalog.entry(model_type, model_id)` when `allow_untested or catalog.unlocked()`, else the verified lookup as today; for each item of `entry['files']`, a `dict` goes through `download_direct`, a `str` through the existing `download_file`; `model_info.json` gains `'source': entry.get('source', 'comma.ai')`, `'converted_from': entry.get('converted_from')`, `'sha256': <sha256 of files[0] when it is a dict>`; `'commit'`/`'version'` use `entry.get('commit') or entry.get('converted_from', {}).get('commit', '')`. All `model_info['commit']` prints become `.get`.

- [ ] **Step 1: Write the failing tests**

Extend `dl_env`'s catalog with a converted entry (`verified_on: []`, url `https://example.invalid/pop.onnx`, `sha256` of `b'pop-bytes'`, size 9) and a second converted entry `pop_verified` identical but `verified_on: ['0.11.1']`; a fake `requests.get` fixture `fake_http(monkeypatch, body: bytes)` returning an object with `raise_for_status()`, `iter_content(chunk_size)` yielding `body`, `headers={}`.

```python
class TestDirectDownload:
  def test_unlocked_download_finds_untested_catalog_entry(self, dl_env, monkeypatch, capsys):
    _, cat = dl_env; cat.UNLOCK_MARKER.write_text('')
    fake_http(monkeypatch, b'pop-bytes')
    assert md.download_model(md.ModelType.DRIVING, 'pop_supercombo_37727') == 0
    info = json.loads((dl_env[0] / 'models' / 'driving' / 'pop_supercombo_37727' / 'model_info.json').read_text())
    assert info['source'] == 'converted' and info['sha256'] == hashlib.sha256(b'pop-bytes').hexdigest()
    assert info['converted_from']['commit']

  def test_locked_download_still_refuses_untested_entry(self, dl_env, capsys):
    assert md.download_model(md.ModelType.DRIVING, 'pop_supercombo_37727') == 1
    assert 'not a tested model' in capsys.readouterr().out

  def test_verified_converted_entry_downloads_without_unlock(self, dl_env, monkeypatch):
    fake_http(monkeypatch, b'pop-bytes')
    assert md.download_model(md.ModelType.DRIVING, 'pop_verified') == 0

  def test_sha256_mismatch_refuses_and_leaves_no_file(self, dl_env, monkeypatch, capsys):
    fake_http(monkeypatch, b'tampered!')
    assert md.download_model(md.ModelType.DRIVING, 'pop_verified') == 1
    d = dl_env[0] / 'models' / 'driving' / 'pop_verified'
    assert not (d / 'driving_supercombo.onnx').exists() and not list(d.glob('*.part'))

  def test_interrupted_download_leaves_only_part_file(self, dl_env, monkeypatch):
    def boom(*a, **k): raise ConnectionError('cut')
    monkeypatch.setattr(md.requests, 'get', boom)
    assert md.download_model(md.ModelType.DRIVING, 'pop_verified') == 1
    d = dl_env[0] / 'models' / 'driving' / 'pop_verified'
    assert not (d / 'driving_supercombo.onnx').exists()

  def test_check_updates_offers_verified_converted_entry(self, dl_env, capsys):
    md.check_updates()
    out = json.loads(capsys.readouterr().out)
    ids = {m['id'] for m in out['driving']}
    assert 'pop_verified' in ids and 'pop_supercombo_37727' not in ids
```

Keep `TestDownloadGate` as is, except `test_allow_untested_bypasses_the_gate` and `test_unlock_marker_bypasses_the_gate` still expect return 1 for `mystery_model` (not in catalog nor registry) with the message now `not found`.

- [ ] **Step 2: Run, verify failure**

Run: `PYTHONPATH=. uv run pytest plugins/model_selector/tests/test_model_download.py -q`
Expected: FAIL (`download_direct` missing; untested entry not found while unlocked).

- [ ] **Step 3: Implement per Interfaces**

Wrap the per-file download in `try/except Exception` that prints the error, removes a leftover `.part`, appends to `failed_files`; the function already returns 1 when `failed_files` is non-empty. Print `Source: converted from <commit> (<vision>/<policy>)` instead of `Commit:` for converted entries.

- [ ] **Step 4: Run download tests and the suite, verify pass; commit**

```bash
git add plugins/model_selector/model_download.py plugins/model_selector/tests/test_model_download.py
git commit -m "model_selector: direct-URL downloads with sha256 check; unlocked CLI sees untested catalog entries"
```

---

### Task 5: Active-file drift detection

**Files:**
- Modify: `plugins/model_selector/model_swapper.py` (add `expected_sha256`, `active_file_matches`)
- Modify: `plugins/model_selector/ui.py:193-216` (`show`), `424-448` (`on_health_check`), module-level `_DRIFT`
- Test: `plugins/model_selector/tests/test_model_swapper.py` (new `TestDrift`)

**Interfaces:**
- Consumes: `_file_sha256` (Task 3); `model_info.json['sha256']` written by Task 4; catalog shipped `sha256`.
- Produces: `ModelSwapper.expected_sha256(model_id: str) -> str | None`: `model_info.json['sha256']` of the model dir if present, else the catalog entry's `sha256` when `catalog.entry(...)` is shipped, else None. `ModelSwapper.active_file_matches(model_id: str) -> bool | None`: None when no expectation; otherwise compares the active file's hash, cached in `self._hash_cache: dict[Path, tuple[int, float, str]]` keyed by path with `(st_size, st_mtime, sha256)` so an unchanged file is hashed once; a missing active file is False. `ui._DRIFT: dict[str, bool]` updated by `on_health_check`; `show()` appends `' — active file differs, re-activate'` to the row value when `_DRIFT.get(model_type)` is True.

- [ ] **Step 1: Write the failing tests**

```python
class TestDrift:
  @pytest.fixture
  def drifty(self, swapper_mod, tmp_path, monkeypatch):
    cat = _point_catalog_at(tmp_path, monkeypatch, CATALOG_FIXTURE)  # fixture includes stock_0.11.2 with sha256 of b'shipped-onnx'
    sw = _build_swapper(swapper_mod, tmp_path, monkeypatch)
    d = sw.models_dir / 'pop_supercombo_37727'; d.mkdir()
    (d / 'driving_supercombo.onnx').write_bytes(b'pop-bytes')
    (d / 'model_info.json').write_text(json.dumps({'name': 'POP', 'sha256': hashlib.sha256(b'pop-bytes').hexdigest()}))
    return sw, cat

  def test_matches_when_active_is_the_model(self, drifty):
    sw, _ = drifty
    (sw.ACTIVE_DIR / 'driving_supercombo.onnx').write_bytes(b'pop-bytes')
    assert sw.active_file_matches('pop_supercombo_37727') is True

  def test_false_when_deploy_put_stock_back(self, drifty):
    sw, _ = drifty
    (sw.ACTIVE_DIR / 'driving_supercombo.onnx').write_bytes(b'shipped-onnx')
    assert sw.active_file_matches('pop_supercombo_37727') is False

  def test_active_file_matches_false_for_pointer(self, drifty):
    sw, _ = drifty
    (sw.ACTIVE_DIR / 'driving_supercombo.onnx').write_text('version https://git-lfs.github.com/spec/v1\n')
    assert sw.active_file_matches('pop_supercombo_37727') is False

  def test_shipped_expectation_comes_from_catalog(self, drifty):
    sw, _ = drifty
    assert sw.expected_sha256('stock_0.11.2') == hashlib.sha256(b'shipped-onnx').hexdigest()

  def test_none_without_expectation(self, drifty):
    sw, _ = drifty
    d = sw.models_dir / 'old'; d.mkdir(); (d / 'model_info.json').write_text('{}')
    assert sw.active_file_matches('old') is None

  def test_hash_is_cached_by_size_and_mtime(self, drifty, monkeypatch):
    sw, _ = drifty
    (sw.ACTIVE_DIR / 'driving_supercombo.onnx').write_bytes(b'pop-bytes')
    calls = []
    real = sw._file_sha256
    monkeypatch.setattr(sw, '_file_sha256', lambda p: calls.append(p) or real(p))
    sw.active_file_matches('pop_supercombo_37727'); sw.active_file_matches('pop_supercombo_37727')
    assert len(calls) == 1
```

- [ ] **Step 2: Run, verify failure**

Run: `PYTHONPATH=. uv run pytest plugins/model_selector/tests/test_model_swapper.py -k Drift -q`. Expected: FAIL (`active_file_matches` missing).

- [ ] **Step 3: Implement in `model_swapper.py`, then wire `ui.py`**

In `on_health_check`, after the pkl caching loop: `_DRIFT[model_type_key] = swapper.active_file_matches(active_id) is False` inside the same try. In `show()`, after computing `active_name`, append the suffix when drifted. Health check runs in plugind, so hashing never happens on the UI thread; `show()` only reads `_DRIFT`.

- [ ] **Step 4: Run the suite, verify pass; commit**

```bash
git add plugins/model_selector/model_swapper.py plugins/model_selector/ui.py plugins/model_selector/tests/test_model_swapper.py
git commit -m "model_selector: flag when the active model file no longer matches the selected model"
```

---

### Task 6: Converter tool `tools/legacy_policy_swap.py`

**Files:**
- Create: `tools/legacy_policy_swap.py`, `tools/requirements-legacy-policy-swap.txt` (`onnx>=1.19`, `onnxruntime>=1.20`, `numpy`)
- Test: `tools/tests/test_legacy_policy_swap.py` (uses `pytest.importorskip('onnx')` and `importorskip('onnxruntime')`)

**Interfaces:**
- Produces (module `legacy_policy_swap`): `class ConversionError(Exception)`; `weight_sequence(model: onnx.ModelProto, node_filter=None) -> list[tuple[str, str, tuple[int, ...]]]` returning `(op_type, initializer_name, dims)` for float16 initializers in consuming-node order, optionally only for nodes where `node_filter(node)` is True; `is_policy_node(node) -> bool` is `any(o.startswith('model/p_') for o in node.output)`; `check_vision_compatible(template, legacy_vision) -> dict[str, str]` (legacy → template names for the hidden-state head arrays) raising `ConversionError` with both `model_checkpoint` values when any legacy vision float16 array outside the top-level `summarizer.*` head has no byte-identical counterpart in the template, or when the head does not align 1:1 by (op, shape) with the template's `model/summarizer.*` Gemms; `align_policy(template, legacy_policy) -> dict[str, str]` mapping template initializer name → legacy initializer name, raising `ConversionError` on length or (op, dims) mismatch at position k; `swap_weights(template, legacy_vision, legacy_policy) -> onnx.ModelProto` (copies the template, replaces policy `raw_data`, transplants the head: weights in place, biases as new dedicated initializers with the six head Gemms rewired when the template shares them, sets `model_checkpoint` to `<template vision part>/<legacy policy checkpoint>`, sets `producer_name = 'catpilot-policy-swap'`, runs `onnx.checker.check_model`); `verify(template, converted, legacy_vision, legacy_policy, seed=0) -> None`: the policy slices are the keys of the legacy policy's own `output_slices` (`plan`, `desire_state`, `pad`) looked up in the converted model's `output_slices`; every other slice is vision. It raises `ConversionError` when any vision slice or any `next_state_*` output differs from the template's on the same random inputs, when the policy slice differs from the standalone legacy policy by more than 0.0 on the glue's policy-input tensors, or when a 10-step recurrence is not finite; `main(argv) -> int` with `template legacy_vision legacy_policy output [--no-verify]`, printing `sha256`, `size`, and the catalog `files` object as JSON.
- The glue tensors feeding the policy are the non-initializer inputs of policy nodes that no policy node produces (spike: `model/dp_out`, `model/fb_full`, `model/tc_out`); match them to the legacy policy's inputs by shape.

- [ ] **Step 1: Write the failing tests with synthetic graphs**

Build helpers in the test file: `make_policy(seed, prefix='')` → a tiny ONNX with inputs `features_buffer [1,25,512]`, `desire_pulse [1,25,8]`, `traffic_convention [1,2]` (float16), two `Gemm` weights, output `outputs [1,1000]`, `metadata_props output_slices` (base64 pickle of `{'plan': slice(0, 990), 'desire_state': slice(990, 998), 'pad': slice(-2, None)}`) and `model_checkpoint 'p<seed>/100'`; `make_vision(seed)` → one `Conv` weight, uint8 inputs, fp16 output `[1,1576]`, checkpoint `'v<seed>/200'`; `make_template(vision, policy)` → a graph whose vision nodes are plain, whose policy nodes are renamed with output prefix `model/p_`, with a minimal glue (`Cast` of three float32 inputs into the policy inputs, a `state_feat_q` passthrough as `next_state_feat_q`), one `outputs` concat and `model_checkpoint 'v0/200/p0/100'`.

```python
def test_weight_sequence_lists_fp16_weights_in_node_order(): ...  # len == number of Gemm weights, op 'Gemm'
def test_vision_identical_passes_for_same_weights(): ...
def test_vision_mismatch_names_both_checkpoints():  # raises ConversionError mentioning 'v0/200' and 'v1/200'
def test_align_policy_maps_by_position(): ...  # dict len == 2 for the synthetic policy
def test_align_policy_refuses_shape_mismatch(): ...  # legacy policy with a (600,512) Gemm → ConversionError
def test_swap_replaces_only_policy_arrays():  # vision initializer bytes unchanged, policy bytes == legacy
def test_swap_sets_checkpoint_and_producer(): ...  # 'v0/200/p1/100', 'catpilot-policy-swap'
def test_verify_passes_for_a_correct_swap(): ...
def test_verify_fails_when_a_policy_array_is_wrong(): ...  # corrupt one raw_data after swap → ConversionError
def test_main_prints_catalog_files_object(tmp_path, capsys): ...  # json with name/url placeholder/sha256/size
```

- [ ] **Step 2: Run, verify failure**

Run: `cd /home/oxygen/catpilot-dev/plugins && /home/oxygen/catpilot-dev/spike_pop_supercombo/.venv/bin/python -m pytest tools/tests -q`
Expected: FAIL with `ModuleNotFoundError: legacy_policy_swap`.

- [ ] **Step 3: Implement the tool per Interfaces**

The spike scripts `build_pop_supercombo.py` and `validate_swap.py` in `/home/oxygen/catpilot-dev/spike_pop_supercombo/scripts/` are the reference for the algorithm; the one change is identifying the template's policy weights by `is_policy_node` instead of via the CD210 split file. Two-space indentation. No imports of openpilot or plugin code.

- [ ] **Step 4: Run the tool tests, verify pass**

Same command as Step 2. Expected: PASS.

- [ ] **Step 5: Run the real conversion and prove it reproduces the spike byte for byte**

```bash
cd /home/oxygen/catpilot-dev/plugins && S=/home/oxygen/catpilot-dev/spike_pop_supercombo
$S/.venv/bin/python tools/legacy_policy_swap.py $S/models/sc0112/driving_supercombo.onnx $S/models/pop/driving_vision.onnx $S/models/pop/driving_policy.onnx $S/out/pop_supercombo_tool.onnx
sha256sum $S/out/pop_supercombo_tool.onnx $S/out/pop_supercombo.onnx
```
Expected (corrected during execution): verify passes with the summarizer transplanted; the tool prints the file's sha256 and size — record them for Task 7. Against the old spike file every vision slice except `hidden_state` is identical and `hidden_state` differs.

- [ ] **Step 6: Commit**

```bash
git add tools/legacy_policy_swap.py tools/requirements-legacy-policy-swap.txt tools/tests/test_legacy_policy_swap.py
git commit -m "tools: legacy_policy_swap — convert a split driving model into a 0.11.2 supercombo by policy-weight swap"
```

---

### Task 7: Release asset, POP catalog entry, docs

**Files:**
- Modify: `plugins/model_selector/compatible_models.json` (add POP), `plugins/model_selector/README.md`, `plugins/model_selector/DESIGN.md`, `plugins/model_selector/plugin.json`
- Test: `plugins/model_selector/tests/test_catalog.py` (`TestShippedCatalog`)

**Interfaces:**
- Consumes: Task 6 output file and its sha256/size; Task 2 catalog schema.
- Produces: catalog driving entry `pop_supercombo_37727` exactly as the design describes, `verified_on: []`, `url` = the release asset's permanent download URL.

- [ ] **Step 1: Compile smoke on the PC with the stock pipeline**

```bash
cd /home/oxygen/catpilot-dev/catpilot && S=/home/oxygen/catpilot-dev/spike_pop_supercombo
PYTHONPATH=$PWD/tinygrad_repo DEV=CPU:LLVM $S/.venv/bin/python tinygrad_repo/examples/openpilot/compile_onnx.py $S/out/pop_supercombo_tool.onnx $S/out/tg_tool/driving_tinygrad.pkl --out-of-band --benchmark-runs 1
```
Expected: exit 0, a ~70 MB pkl.

- [ ] **Step 2: Publish the release asset (outward-facing: confirm with the user first)**

```bash
cd /home/oxygen/catpilot-dev/plugins && cp /home/oxygen/catpilot-dev/spike_pop_supercombo/out/pop_supercombo_tool.onnx /tmp/pop_supercombo_37727.onnx
gh release create models-0.11.2 /tmp/pop_supercombo_37727.onnx --repo catpilot-dev/plugins --title "Models for catpilot 0.11.2" --notes "pop_supercombo_37727.onnx: POP driving model (#37727) converted to the 0.11.2 supercombo format by tools/legacy_policy_swap.py. sha256 <from the Task 6 tool run>, <size> bytes. Checkpoint 6a7d09ad-bcc9-43bc-916d-29287e60cee2/200/05a58a51-e0e3-4e9b-8e27-e644685f2c50/100."
gh release view models-0.11.2 --repo catpilot-dev/plugins --json assets --jq '.assets[].url'
```
Expected: one asset URL of the form `https://github.com/catpilot-dev/plugins/releases/download/models-0.11.2/pop_supercombo_37727.onnx`. `gh release create` publishes; stop and ask the user before running it if they have not confirmed in this session.

- [ ] **Step 3: Add the catalog entry and a shipped-catalog test**

```python
  def test_shipped_catalog_lists_pop_as_converted_and_untested(self, cat):
    c = cat.load_catalog()
    pop = next(e for e in c['driving'] if e['id'] == 'pop_supercombo_37727')
    assert pop['source'] == 'converted' and pop['verified_on'] == []
    f = pop['files'][0]
    assert f['name'] == 'driving_supercombo.onnx'
    assert (f['sha256'], f['size']) == (<sha256 printed by the Task 6 tool run>, <size printed by the Task 6 tool run>)
    assert f['url'].startswith('https://github.com/catpilot-dev/plugins/releases/download/models-0.11.2/')
```
Run: `PYTHONPATH=. uv run pytest plugins/model_selector/tests/test_catalog.py -q`. Expected: FAIL, then add the entry (`name` "POP model", `date` "2026-03-24", `pr` "#37727", `converted_from` `{commit: "12f1be19cc", vision: "6a7d09ad-bcc9-43bc-916d-29287e60cee2/200", policy: "05a58a51-e0e3-4e9b-8e27-e644685f2c50/100"}` with the full 40-char commit from `git -C /home/oxygen/catpilot-dev/catpilot rev-parse 12f1be19cc`, `notes` "POP driving model converted to the 0.11.2 supercombo by tools/legacy_policy_swap.py; test drive on 0.11.2 pending"), rerun → PASS.

- [ ] **Step 4: Docs and manifest**

`plugin.json`: `"version": "0.11.2"`, add `"min_openpilot": "0.11.2"` (key read by `manifest.check_compatibility`). `DESIGN.md`: table "Two model types" → driving `driving_supercombo.onnx`, compiled `driving_tinygrad.pkl`; storage layout tree; "Swap flow" step 3 deletes only the model's own artefact; new sections "Stock import by shipped hash", "Converted models and the release asset", "Active-file drift", "Unlock is CLI-only"; watcher paragraph with the new filename and go-live note; `ACTIVE_DIR` row. `README.md`: say the plugin targets catpilot 0.11.2 onwards, explain "active file differs" and that legacy split models from 0.11.1 are not loadable. Keep the hook list and `min_openpilot` consistent with the manifest.

- [ ] **Step 5: Run the full suite and commit**

Run: `PYTHONPATH=. uv run pytest plugins/model_selector/tests .github/scripts/tests/test_model_watch.py -q`. Expected: PASS.

```bash
git add plugins/model_selector/compatible_models.json plugins/model_selector/README.md plugins/model_selector/DESIGN.md plugins/model_selector/plugin.json plugins/model_selector/tests/test_catalog.py
git commit -m "model_selector: POP supercombo catalog entry (release asset), 0.11.2 docs and manifest"
```

---

## Appendix: developer's C3 rollout (not part of plan execution; blocked on the 0.11.2 boot gate)

1. Deploy plugins `rebase/v0.11.2` to the C3 after catpilot 0.11.2 boots there and compiles the stock supercombo.
2. `ssh c3 'touch /data/plugins-runtime/model_selector/data/.unlocked'`.
3. `ssh c3 'cd /data/plugins-runtime/model_selector && /usr/local/venv/bin/python model_download.py download --type driving pop_supercombo_37727 --unlocked && /usr/local/venv/bin/python model_swapper.py --type driving swap pop_supercombo_37727'`, then reboot; first boot compiles (~1 min).
4. In Settings → Software confirm "POP model (2026-03-24) — untested" with no "active file differs" suffix, then test drive.
5. If POP is kept: set `verified_on: ["0.11.2"]` in the catalog, deploy, remove the unlock file. Delete `/data/models/driving/<legacy dirs>` and `/data/models/dm/<non-stock dirs>` by hand.
