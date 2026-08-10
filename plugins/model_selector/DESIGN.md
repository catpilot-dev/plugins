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
| Driving | `driving_vision.onnx` + `driving_policy.onnx` | `/data/models/driving/` | `/data/models/active_driving_model` |
| DM | `dmonitoring_model.onnx` | `/data/models/dm/` | `/data/models/active_dm_model` |

## Model source

Models are openpilot's own model files, pulled at a specific **GitHub commit**:

- **Registry** (`/data/models/model_registry.json`) is the catalog of available
  models — `{name, commit, date, description, pr, files}` per entry, split into
  `driving_models` / `dm_models`. It is refreshed from the GitHub commits API
  (`/repos/commaai/openpilot/commits?path=selfdrive/modeld/models`,
  `model_download.py update-registry`), which parses commit messages / linked
  PRs to derive each model's name and type, and also **removes reverted models**
  (it scans recent commits for `Revert` messages and drops the reverted commit
  hashes). A single model can also be added directly from a merged PR
  (`add-from-pr <n>`) — note it records the PR **head** commit, because the LFS
  objects live there, not on the merge commit.
- **Download** fetches each ONNX from
  `raw.githubusercontent.com/commaai/openpilot/<commit>/selfdrive/modeld/models/<file>`.
  These files are Git LFS pointers; `download_file` detects the pointer, parses
  `oid`/`size`, and resolves the real object through the LFS batch API, trying
  **GitHub first, then GitLab** (`_LFS_BATCH_URLS`). The resolved blob is written
  to the model dir, and a `model_info.json` is written alongside.

## Storage layout on device

```
/data/models/
├── model_registry.json           # catalog of available models
├── active_driving_model          # JSON: {id, name, tinygrad} — active driving model
├── active_dm_model               # JSON: {id, name, tinygrad} — active DM model
├── driving/
│   └── <model_id>/
│       ├── model_info.json               # name, commit, date, pr, description, type
│       ├── driving_vision.onnx           # portable weights (from GitHub)
│       ├── driving_policy.onnx
│       ├── driving_vision_tinygrad.pkl   # device-compiled cache (optional)
│       ├── driving_policy_tinygrad.pkl
│       └── .tinygrad_commit              # tinygrad rev that built the PKLs
└── dm/
    └── <model_id>/
        ├── model_info.json
        ├── dmonitoring_model.onnx
        └── dmonitoring_model_tinygrad.pkl
```

The **live/active** copies openpilot actually loads live in
`/data/openpilot/selfdrive/modeld/models/` (`ACTIVE_DIR`). A swap writes *real
files* (not symlinks) there.

## Swap flow — reboot, not live

`ModelSwapper.swap_model(model_id)` (`model_swapper.py`):

1. **Cache the outgoing model's PKLs.** Copy any compiled `*pkl*` from
   `ACTIVE_DIR` back into the current model's storage dir (idempotent — see PKL
   caching).
2. **Validate the incoming model.** Its required ONNX files must exist in
   `/data/models/<type>/<id>/`, else `ValueError`.
3. **Copy ONNX into `ACTIVE_DIR`** (overwriting), then delete stale `*pkl*` from
   `ACTIVE_DIR`. If the model has cached PKLs **and** they were built by the
   *current* tinygrad commit (`.tinygrad_commit` == `git rev-parse` of
   `tinygrad_repo`), copy those PKLs in too; otherwise leave none so openpilot
   recompiles.
4. **Update the active tracker** (`active_<type>_model`) with `{id, name,
   tinygrad}`.

The swap **returns `requires_reboot: True`** and does not restart anything.
Nothing changes until openpilot reloads modeld — i.e. **a reboot**. In the UI,
activating a model raises a "Model swapped. Reboot to activate." dialog whose
**Reboot** sets the `DoReboot` param; **Cancel** swaps the previous model back
in immediately (so a canceled activation is a no-op). Models older than
`MIN_MODEL_DATE` are filtered out of every list, so they can't be selected.

## ONNX + PKL caching

Weights are stored as **portable ONNX**; on first boot after a swap, openpilot
compiles ONNX → device-specific **tinygrad PKL** (~1 min on a comma three). To
avoid recompiling every time you switch back:

- The `device.health_check` hook (`on_health_check`, runs ~every 5 s) calls
  `cache_compiled_pkl(active_id)` for both types. As soon as openpilot has
  written fresh PKLs into `ACTIVE_DIR`, they're copied back into the active
  model's storage dir and the building tinygrad commit is recorded in
  `.tinygrad_commit`. The copy only takes files not already cached, so repeated
  calls are safe.
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
  and **New Models** (`CHECK`, showing check status / "last checked …").
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
  reboot dialog above; Delete removes the model dir (blocked for the active
  model).
- **CHECK** → spawns `model_download.py update-registry` then `check-updates`
  as a subprocess (via `/usr/local/venv/bin/python`), parses the JSON list of
  new, compatible, uninstalled models, and offers them in a dialog; selecting
  one spawns `model_download.py download <id> --type <type>`.

Scripts are located under `PLUGINS_RUNTIME_DIR/model_selector/` at runtime
(`/data/plugins-runtime/model_selector/`).

## Compatibility filtering

Three independent gates keep incompatible models out:

1. **`MIN_MODEL_DATE`** (`model_swapper.py`): `driving` ≥ `2025-10-01`,
   `dm` ≥ `2025-11-01`. Applied when *listing* installed models and available
   downloads — older ones are hidden (DM models before 2025-11-01 lack
   `output_slices` metadata; the driving floor is the v0.10.3 shipping model).
2. **`desire_pulse` transition** (`check_model_compatibility`, driving only):
   driving models dated before **2025-08-27** expect `desire` instead of
   `desire_pulse` and are flagged/blocked at download.
3. **Registry ingestion filters** (`update_registry_from_github`): only commits
   from **2025-09-05** (Firehose) onward are ingested; revert commits and the
   models they revert are excluded.

## Hooks

| Hook | Function (`ui.py`) | Priority | Role |
|---|---|---|---|
| `ui.software_settings_extend` | `on_software_settings_extend` | 50 | Injects the model-selector rows into Settings → Software |
| `device.health_check` | `on_health_check` | 50 | Accumulator hook; opportunistically caches freshly compiled PKLs for both active models, returns `{…, "model_selector": {"status": "ok"}}` |

## Config / paths

No plugin params in a `data/` dir — enable/disable is the framework-level
Settings → Plugins toggle. The moving parts are file locations and the
date constants:

| Name | Value | Where | Meaning |
|---|---|---|---|
| `REGISTRY_FILE` | `/data/models/model_registry.json` | `model_download.py` | Available-models catalog |
| `BASE_DATA_DIR` | `/data` (else `~/driving_data`) | `model_swapper.py` | Root of model storage |
| `ACTIVE_DIR` | `/data/openpilot/selfdrive/modeld/models` | `model_swapper.py` | Where openpilot loads the live model |
| `active_<type>_model` | JSON `{id, name, tinygrad}` | `/data/models/` | Which model is active per type |
| `MIN_MODEL_DATE['driving']` | `2025-10-01` | `model_swapper.py` | List/download floor for driving |
| `MIN_MODEL_DATE['dm']` | `2025-11-01` | `model_swapper.py` | List/download floor for DM |
| `_LFS_BATCH_URLS` | GitHub, then GitLab | `model_download.py` | LFS resolve order |
| `PYTHON_BIN` | `/usr/local/venv/bin/python` | `ui.py` | Interpreter for the download subprocess |

## Key files

- `model_download.py` — registry maintenance and downloading. CLI:
  `list`, `download`, `check-updates`, `add-model`, `add-from-pr`,
  `update-registry`. LFS pointer resolution lives here.
- `model_swapper.py` — the `ModelSwapper` class: `list_models`, `swap_model`,
  `cache_compiled_pkl`, `get_active_model`, `verify_model`, `delete_model`, plus
  a `--type {driving,dm}` CLI.
- `ui.py` — the Software-panel hook, the dialogs, and the `device.health_check`
  hook.
- `tests/test_model_swapper.py` — covers `ModelType`, `MODEL_CONFIGS`,
  `list_models` (empty / with-info / hidden-dir skip / date sort),
  `resolve_model_id`, and `swap_model` validation (missing model, missing ONNX).

## Notes / rough edges

- `add_model_from_pr` references a `registry` name that isn't loaded in that
  function's scope before use — the dedup loop over `registry.items()` looks
  like it would raise `NameError`. The primary path (`update-registry`) does not
  hit this. Documented as-is; not verified on device.
- The CLI "Next steps" hints in `download_model` point at an old
  `selfdrive/modeld/model_swapper.py` path; the actual swapper is this plugin's
  `model_swapper.py`.
