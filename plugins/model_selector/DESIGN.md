# Model Selector — Design & Implementation

A hook plugin that manages multiple openpilot **driving** and **driver-monitoring
(DM)** models on the device: it discovers them from the commaai/openpilot GitHub
history, downloads the portable ONNX weights, stores them under `/data/models/`,
and swaps which one openpilot loads. It also caches the device-specific compiled
model so re-selecting a model is instant. It has no runtime control loop — it is
tooling plus a Software-panel UI.

## Two model types

Handled independently throughout, keyed by a `ModelType` enum (`driving` / `dm`):

| Type | ONNX files | Storage dir | Active tracker |
|---|---|---|---|
| Driving | `driving_supercombo.onnx` (compiled by openpilot to `driving_tinygrad.pkl`) | `/data/models/driving/` | `/data/models/active_driving_model` |
| DM | `dmonitoring_model.onnx` | `/data/models/dm/` | `/data/models/active_dm_model` |

## Model source

Models are openpilot's own model files, pulled at a specific **GitHub commit**:

- **Registry** (`/data/models/model_registry.json`) is the catalog of available
  models — `{name, commit, date, description, pr, files}` per entry, split into
  `driving_models` / `dm_models`. It is refreshed from the GitHub commits API
  (`/repos/commaai/openpilot/commits?path=selfdrive/modeld/models`,
  `model_download.py update-registry`), which parses commit messages / linked
  PRs to derive each model's name and type, and also **marks reverted models**
  (it scans recent commits for `Revert` messages and tags the reverted commit's
  registry entry with `upstream_reverted: "<sha>"`; it is never dropped — see
  Upstream reverts below). A single model can also be added directly from a merged PR
  (`add-from-pr <n>`) — note it records the PR **head** commit, because the LFS
  objects live there, not on the merge commit. `update-registry`, `add-from-pr`,
  and `list` are **maintainer-only CLI** now: they build the pool of candidates a
  maintainer can review, but nothing about the UI reads the registry directly —
  the UI, and every gate, read `compatible_models.json` through `catalog.py`
  instead (see Compatibility gating below).
- **Download** fetches each ONNX from
  `raw.githubusercontent.com/commaai/openpilot/<commit>/selfdrive/modeld/models/<file>`.
  These files are Git LFS pointers; `download_file` detects the pointer, parses
  `oid`/`size`, and resolves the real object through the LFS batch API, trying
  **GitHub first, then GitLab** (`_LFS_BATCH_URLS`). The resolved blob is written
  to the model dir, and a `model_info.json` is written alongside.
- Catalog entries with `source: converted` skip GitHub commits entirely: each
  `files` item carries a direct `url`, `sha256` and `size`. Bytes are streamed to
  `<name>.part` and renamed onto the final name only after the size and sha256
  match, so a failed or interrupted download never leaves a half-written model
  (the `.part` file is removed). `model_info.json` is written last, and
  `check_updates` counts a directory as installed only once it exists, so a
  download killed by a reboot or power cut (which leaves the `.part` file
  behind) is offered again.

## Storage layout on device

```
/data/models/
├── model_registry.json           # catalog of available models
├── active_driving_model          # JSON: {id, name, tinygrad} — active driving model
├── active_dm_model               # JSON: {id, name, tinygrad} — active DM model
├── .drift                        # JSON: {driving: bool, dm: bool} — written by the health check
├── driving/
│   └── <model_id>/
│       ├── model_info.json               # name, commit, date, pr, description, type
│       ├── driving_supercombo.onnx       # portable weights (GitHub, or a catalog URL)
│       ├── driving_tinygrad.pkl          # device-compiled cache (optional)
│       └── .tinygrad_commit              # tinygrad rev that built the PKL
└── dm/
    └── <model_id>/
        ├── model_info.json
        ├── dmonitoring_model.onnx
        └── dmonitoring_model_tinygrad.pkl
```

The **live/active** copies openpilot actually loads live in
`/data/openpilot/openpilot/selfdrive/modeld/models/` (`ACTIVE_DIR`). A swap
writes *real files* (not symlinks) there. The `big_*` and `*_warp_*` pickles in
that directory belong to the build and are never touched.

## Swap flow — reboot, not live

`ModelSwapper.swap_model(model_id)` (`model_swapper.py`):

1. **Cache the outgoing model's PKL.** Copy this type's compiled artefact from
   `ACTIVE_DIR` back into the current model's storage dir (idempotent, skipped
   under drift — see PKL caching).
2. **Validate the incoming model.** Its required ONNX files must exist in
   `/data/models/<type>/<id>/`, else `ValueError`.
3. **Copy ONNX into `ACTIVE_DIR`** (overwriting), then delete only this model
   type's own compiled artefact (`driving_tinygrad.pkl` / `dmonitoring_model_tinygrad.pkl`)
   from `ACTIVE_DIR`. If the model has cached PKLs **and** they were built by the
   *current* tinygrad commit (`.tinygrad_commit` == `git rev-parse` of
   `tinygrad_repo`), copy those PKLs in too; otherwise leave none so openpilot
   recompiles.
4. **Update the active tracker** (`active_<type>_model`) with `{id, name,
   tinygrad}`.

The swap **returns `requires_reboot: True`** and does not restart anything.
Nothing changes until openpilot reloads modeld — i.e. **a reboot**. In the UI,
activating a model raises a "Model swapped. Reboot to activate." dialog whose
**Reboot** sets the `DoReboot` param; **Cancel** swaps the previous model back
in immediately (so a canceled activation is a no-op). `swap_model` itself
refuses any model the catalog has not verified for the running openpilot
version (unless unlocked) — see Compatibility gating below.

## ONNX + PKL caching

Weights are stored as **portable ONNX**; on first boot after a swap, openpilot
compiles ONNX → device-specific **tinygrad PKL** (~1 min on a comma three). To
avoid recompiling every time you switch back:

- The `device.health_check` hook (`on_health_check`, runs ~every 5 s) calls
  `cache_compiled_pkl(active_id)` for both types. As soon as openpilot has
  written fresh PKLs into `ACTIVE_DIR`, they're copied back into the active
  model's storage dir and the building tinygrad commit is recorded in
  `.tinygrad_commit`. The copy only takes files not already cached, so repeated
  calls are safe. Nothing is cached while the active file has drifted
  (`active_file_matches` is False): that pkl was compiled from another model's
  file and would otherwise be copied back under this model's name later.
- On a later swap **to** that model, step 3 above reuses the cached PKLs only if
  `.tinygrad_commit` matches the current tinygrad — instant, no compile.
  A tinygrad upgrade changes the commit, so stale PKLs are skipped and the model
  recompiles once.

## Software-panel injection

`on_software_settings_extend(default, layout)` (hook
`ui.software_settings_extend`) builds a `ModelSelectorUI` and appends to the
Software layout's plugin extension points:

- `layout._plugin_items` — three `button_item` rows: **Driving Model** and
  **Driver Monitoring** (each `SELECT`, showing the active model's name+date),
  and **Tested Models** (`CHECK`, showing check status / "last checked …").
- `layout._plugin_updaters` — `manager.update`, polled to detect completion of
  the background check/download subprocess.
- `layout._plugin_show_cbs` — `manager.show`, refreshes the installed-model
  lists and active names when the panel opens.

All openpilot UI imports are **lazy inside the hook function** (the module is
imported during `SoftwareLayout.__init__`, so top-level UI imports would
circular-import).

Interaction flow:

- **SELECT** → `MultiOptionDialog` of installed models → `ModelActionDialog`
  (3 buttons: **Delete** / **Cancel** / **Activate**). Activate runs the swap +
  reboot dialog above; it is enabled for tested models other than the active
  one, and for the active one while its file has drifted (`activation_allowed`).
  Delete removes the model dir (blocked for the active model).
- **CHECK** → spawns `model_download.py check-updates` as a subprocess (via
  `/usr/local/venv/bin/python`). This is a local diff against
  `compatible_models.json` — no GitHub call, no network needed — that parses
  the JSON list of verified, uninstalled models and offers them in a dialog;
  selecting one spawns `model_download.py download <id> --type <type>`.

Scripts are located under `PLUGINS_RUNTIME_DIR/model_selector/` at runtime
(`/data/plugins-runtime/model_selector/`).

## Compatibility gating

A model may be downloaded or activated only if `compatible_models.json` — the
curated catalog shipped at the plugin root — records that it passed a test drive
on the openpilot version this device runs. `catalog.py` owns that policy; the
date heuristics it replaced (`MIN_MODEL_DATE`, the revert-name filter) are gone.

- **Catalog** — `compatible_models.json`, at the plugin root so every deploy
  overwrites it. `data/` is preserved across reinstalls and would freeze a
  catalog placed there. Entries carry `verified_on: ["0.11.2", …]`; one entry per
  type per version also carries `baseline_for`, marking the known-good fallback.
- **Version** — read from `/data/openpilot/openpilot/common/version.h`, falling back to
  `manifest.OPENPILOT_VERSION`.
- **Shipped models** — the ONNX a release ships is verified by definition. Its
  entry carries `source: "shipped"`, `sha256` and `size`, and no `commit`; see
  Stock import by shipped hash below.
- **Fail closed** — a missing or corrupt catalog yields zero verified models.
- **Unlock** — `<PLUGINS_RUNTIME_DIR>/model_selector/data/.unlocked` lets a
  maintainer install and activate untested models from the CLI; the `download
  --unlocked` flag covers the install only. It has no UI surface and survives
  reinstalls, since `data/` is preserved. See Unlock is CLI-only below.

Enforced at three call sites so no path is ungated: `check_updates` (what is
offered), `download_model` (what is fetched), `swap_model` (what is activated).

### Stock import by shipped hash

`ModelSwapper.import_stock` copies the release's own ONNX out of `ACTIVE_DIR`
into storage as the stock model. The proof is byte identity, not tracker state:
the active file must match the shipped entry's `size` and `sha256` (size is
checked first so a foreign file is rejected without reading it whole). A device
that swapped on an earlier release therefore still gets its baseline, and a
swapped model or an LFS pointer is never labelled as the release default. The
dir is built under a `_`-prefixed temp name and renamed, so an interrupted copy
never presents as a complete model. Whenever the stock dir exists, a **stale
tracker** — one that is missing or names a directory lacking the model's ONNX,
such as a legacy split `driving_vision`/`driving_policy` model from 0.11.1 — is
rewritten to name it. So is a tracker naming a model with no recorded sha256
(model_info.json or catalog) while the active file is provably stock: a 0.11.1
DM directory has the same `dmonitoring_model.onnx` name, cannot be checked
against the running file, and stock is what runs. A tracker naming a model with
a recorded hash is left alone; a mismatch there is drift (below). Legacy split
models are not loadable on 0.11.2; they are hidden from SELECT, so they stay on
disk until deleted over ssh.

### Converted models and the release asset

A model published for an older architecture can be carried to the current one
by a maintainer and shipped as a catalog entry with `source: "converted"`. The
POP driving model (`pop_supercombo_37727`, PR #37727) is the first. It was built
by `tools/legacy_policy_swap.py`, which transplants POP's policy **and its
retrained hidden-state summarizer** into the stock 0.11.2 supercombo — a policy
swap alone is not enough, because the policy reads the summarizer's hidden state.
The stock vision backbone and everything else stay as shipped. The entry records
`converted_from` (POP's commit and the vision/policy checkpoints) as provenance.

The converted ONNX is a GitHub release asset (`models-0.11.2` on
catpilot-dev/plugins), and the entry's `url`, `sha256` and `size` pin exactly
those bytes: the catalog, not the release page, is the trust root. The asset
name is stable, so the URL is known before publishing. Entries ship with
`verified_on: []` until a test drive on that version. POP passed its 0.11.2 test drive
(routes 517–534, 2026-11-04..09) and is `verified_on: ["0.11.2"]`; the
release asset must be published before the catalog that lists it ships, or a
download on a fresh install fails.

### Active-file drift

Something outside the plugin (a git reset, the updater) can put a different
file at `ACTIVE_DIR` than the one the tracker names. `device.health_check`
(`on_health_check`) compares the active file's sha256 with the selected model's
expected hash (`active_file_matches`) for each type and writes the result to
`/data/models/.drift` (atomically, and only when it changes, `write_drift`). The
Software panel runs in another process and only reads that file (`read_drift`);
a missing or malformed file means no drift. When a type has drifted, the row
appends ` — active file differs, re-activate`; Activate stays enabled for the
active model while it has drifted, and activating it again restores it.

### Unlock is CLI-only

The UI never offers or activates an untested model. `check_updates` lists only
verified models, `swap_model` refuses anything else, and the panel has no unlock
toggle. A maintainer touches `data/.unlocked` and drives
`model_download.py download --type driving <id>` and
`model_swapper.py --type driving swap <id>` from a shell, then reboots.
`--unlocked` is a `download` flag only (it installs without the marker); `swap`
has no such flag and honours the marker file alone. Once the
test drive passes, the entry's `verified_on` is updated, the catalog deployed and
the marker removed.

### Upstream reverts

A revert upstream is **not** a verdict on whether a model drives. comma reverts
for many reasons — a metric regression, infrastructure, a competing model
winning — and none of them is a road test on this fork. Reverted models stay
eligible: `update-registry` marks them `upstream_reverted: "<sha>"` and keeps
them, and a catalog entry may carry the same field as provenance. Only the
revert commit itself is never treated as a model.

### Upstream watch (CI)

`.github/workflows/model-watch.yml` runs `.github/scripts/model_watch.py` daily.
It reads the commits API once per ONNX file this fork loads
(`driving_supercombo.onnx`, `dmonitoring_model.onnx`) rather
than for the whole models directory, then keeps only commits that **added or
modified** one of them. Watching the directory reported every refactor, path
move and foreign-interface model as an installable candidate; a removal or
rename touches the path without changing a model this fork can load. Upstream
replaced `driving_vision`/`driving_policy` with the single
`driving_supercombo.onnx` in 0.11.2, so only commits from that point count. The
watcher goes live for driving models with this file: until upstream commits a new
supercombo after the release default, it reports nothing, which is the truthful
answer rather than a fault. The file that changed also decides the
model type, which is evidence; the commit message was a guess. It reconciles against
`compatible_models.json` and against existing GitHub issues, and files a
`model-candidate` issue per new model (and `model-revert` for a revert of
something already reported or catalogued). Dedup state is the issues themselves,
keyed by an `upstream-commit: <sha>` line in each body; closing an issue means
handled, so it is never re-reported. The job reports only — it cannot make a
model installable.

## Hooks

| Hook | Function (`ui.py`) | Priority | Role |
|---|---|---|---|
| `ui.software_settings_extend` | `on_software_settings_extend` | 50 | Injects the model-selector rows into Settings → Software |
| `device.health_check` | `on_health_check` | 50 | Accumulator hook; opportunistically caches freshly compiled PKLs for both active models, imports the stock model, publishes `/data/models/.drift`, returns `{…, "model_selector": {"status": "ok"}}` |

The manifest declares `"min_openpilot": "0.11.2"` (read by catpilot's
`check_compatibility`), so the plugin does not load on a release with the split
model layout.

## Config / paths

No plugin params in a `data/` dir — enable/disable is the framework-level
Settings → Plugins toggle. The moving parts are file locations:

| Name | Value | Where | Meaning |
|---|---|---|---|
| `REGISTRY_FILE` | `/data/models/model_registry.json` | `model_download.py` | Maintainer's available-models catalogue (candidate pool, not the gate) |
| `BASE_DATA_DIR` | `/data` (else `~/driving_data`) | `model_swapper.py` | Root of model storage |
| `ACTIVE_DIR` | `/data/openpilot/openpilot/selfdrive/modeld/models` | `model_swapper.py` | Where openpilot loads the live model |
| `.drift` | `/data/models/.drift` | `model_swapper.py` | Per-type active-file drift, written by the health check, read by the panel |
| `active_<type>_model` | JSON `{id, name, tinygrad}` | `/data/models/` | Which model is active per type |
| `CATALOG_FILE` | `<plugin root>/compatible_models.json` | `catalog.py` | The curated compatibility gate |
| `VERSION_H` | `/data/openpilot/openpilot/common/version.h` | `catalog.py` | Running openpilot version (source of truth) |
| `UNLOCK_MARKER` | `<PLUGINS_RUNTIME_DIR>/model_selector/data/.unlocked` | `catalog.py` | Maintainer unlock for untested models |
| `_LFS_BATCH_URLS` | GitHub, then GitLab | `model_download.py` | LFS resolve order |
| `PYTHON_BIN` | `/usr/local/venv/bin/python` | `ui.py` | Interpreter for the download subprocess |

## Key files

- `catalog.py` — the compatibility gate: `load_catalog`, `verified_entries`,
  `is_verified`, `baseline_entry`, `unlocked`, `validate_catalog`. Pure policy,
  no ONNX/storage knowledge; `model_download.py` and `model_swapper.py` both
  call into it rather than re-deriving compatibility themselves.
- `compatible_models.json` — the curated catalog data `catalog.py` reads, at
  the plugin root (not `data/`) so every deploy overwrites it.
- `model_download.py` — registry maintenance and downloading. CLI:
  `list`, `download` (`--unlocked`), `check-updates`, `add-model`, `add-from-pr`,
  `update-registry`. LFS pointer resolution lives here.
- `model_swapper.py` — the `ModelSwapper` class: `list_models`, `swap_model`,
  `cache_compiled_pkl`, `get_active_model`, `verify_model`, `delete_model`,
  `import_stock`, plus a `--type {driving,dm}` CLI.
- `ui.py` — the Software-panel hook, the dialogs, and the `device.health_check`
  hook.
- `tests/test_model_swapper.py` — covers `ModelType`, `MODEL_CONFIGS`,
  `list_models` (empty / with-info / hidden-dir skip / date sort),
  `resolve_model_id`, and `swap_model` validation (missing model, missing ONNX).
- `tests/test_catalog.py` — covers version parsing/fallback, fail-closed
  behavior on a missing or corrupt catalog, `verified_entries`/`is_verified`
  version matching, `baseline_entry`, the unlock marker, and
  `validate_catalog`'s checks (duplicate ids, missing baseline, required
  fields per entry type).

## Notes / rough edges

- `add_model_from_pr` references a `registry` name that isn't loaded in that
  function's scope before use — the dedup loop over `registry.items()` looks
  like it would raise `NameError`. The primary path (`update-registry`) does not
  hit this. Documented as-is; not verified on device.
- The CLI "Next steps" hints in `download_model` point at an old
  `selfdrive/modeld/model_swapper.py` path; the actual swapper is this plugin's
  `model_swapper.py`.
