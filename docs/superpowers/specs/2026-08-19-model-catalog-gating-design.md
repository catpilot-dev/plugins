# Model Selector — curated compatibility gating

**Date:** 2026-08-19
**Status:** approved, not yet implemented
**Scope:** `plugins/model_selector/`

## Problem

`model_selector` discovers driving and driver-monitoring models by scraping the
commaai/openpilot commits API (`model_download.py update-registry`), then filters
the result with heuristics: date floors (`MIN_MODEL_DATE`), a `desire_pulse`
cutoff date, and a substring match on `revert`. Anything comma merges after those
dates is presented to the user as installable.

A driving model determines how the car steers and paces itself. Heuristics
cannot tell whether a given model actually works with a given catpilot version —
only a test drive can. Today a non-technical user can tap CHECK, install a model
nobody has ever driven on this fork, reboot, and drive it.

## Goal

Only models that have passed a test drive on the running catpilot version are
installable or activatable through the UI. Untested models remain reachable to
the maintainer over CLI, behind an explicit unlock.

## Design

### Catalog file

`plugins/model_selector/compatible_models.json`, at the **plugin root** — not in
`data/`. `install.sh` (lines 219-233) preserves a plugin's `data/` wholesale
across reinstalls: it moves the directory aside, `rm -rf`s the destination, and
restores the preserved copy over the repo skeleton. A catalog shipped in `data/`
would freeze at first install and never receive an update. At the plugin root it
is overwritten by every deploy, which is the intended update path.

```json
{
  "driving": [
    {
      "id": "cool_people_3c957c6",
      "name": "Cool People",
      "date": "2025-10-14",
      "commit": "3c957c6...",
      "files": ["driving_vision.onnx", "driving_policy.onnx"],
      "verified_on": ["0.11.1"],
      "baseline_for": ["0.11.1"],
      "notes": "test drive 2026-08-02, BMW E90"
    }
  ],
  "dm": []
}
```

The model an upstream release ships is **verified by definition** — it is what
catpilot itself runs, so it needs no test drive to enter the catalog. It is
represented by a `source: "shipped"` entry, which carries no `commit` because it
is never downloaded:

```json
{
  "id": "stock_0.11.1",
  "name": "Release default",
  "date": "2026-08-11",
  "source": "shipped",
  "verified_on": ["0.11.1"],
  "baseline_for": ["0.11.1"],
  "notes": "ONNX shipped in catpilot 0.11.1"
}
```

`verified_on` holds openpilot versions as reported by `common/version.h`. A model
that survives a rebase gains one string rather than a duplicated entry. `files`
is required on every downloadable entry and is consumed by the downloader, not by
the gate; `source: "shipped"` entries omit it along with `commit`, since they are
copied from disk rather than fetched.

The shipped default is normally the baseline. **`baseline_for` is mandatory:** for each version present in the catalog, exactly
one driving entry and one DM entry must claim `baseline_for` — the model that
ships with that catpilot version. Without it a user who switches away from stock
has no catalogued route back, and the recovery path below has nothing to point
at. Entries with `source: "shipped"` are exempt from the `commit` requirement;
every other entry must carry one, since `commit` is what the downloader fetches
by. The invariant is enforced by a `validate_catalog()` check in `catalog.py`,
covered by a test that fails on a version with zero or multiple baselines, so a
malformed catalog is caught before it ships rather than on a device.

### Stock import

A `source: "shipped"` entry has no download URL — its weights are the ONNX
already sitting in `ACTIVE_DIR` on a fresh install. `ModelSwapper.import_stock()`
copies them into `/data/models/<type>/<stock_id>/` and writes a `model_info.json`,
but **only when the active-model tracker is absent**. An absent tracker is proof
that no swap has ever run, so `ACTIVE_DIR` still holds the release's own files;
once a swap has happened those files may be any model, and mislabeling them as
stock would put an untested model behind a "release default" label. It is
idempotent and runs from the existing `device.health_check` hook.

Devices that swapped before this feature ships therefore get no stock entry.
Their baseline is whatever verified model they are running; nothing breaks, they
simply have no offline route back to the release default. Re-importing one would
mean fetching LFS blobs by oid, which is out of scope.

### `catalog.py`

New module in `plugins/model_selector/`, the single source of truth for policy:

| Function | Behavior |
|---|---|
| `openpilot_version()` | Parses `/data/openpilot/common/version.h` (`split('"')[1]`, mirroring `system/version.py:22`). Falls back to `manifest.OPENPILOT_VERSION`, then `''`. Deliberately import-free so it works inside the bare-venv download subprocess. |
| `load_catalog()` | Reads the JSON. Returns `{}` on missing or corrupt file. |
| `verified_entries(model_type)` | Catalog entries whose `verified_on` contains `openpilot_version()`. |
| `is_verified(model_type, model_id)` | Membership test over the above. |
| `baseline_entry(model_type)` | The entry claiming `baseline_for` for the running version, else `None`. |

`import_stock()` is the one piece of this feature that lives on `ModelSwapper`
rather than in `catalog.py`: it needs `MODEL_CONFIGS` (ONNX filenames, storage
dirs, tracker paths), and putting it in `catalog.py` would either duplicate that
table or import `model_swapper`, which imports `catalog` — a cycle. `catalog.py`
stays pure policy with no file-layout knowledge.
| `unlocked()` | True when the marker file exists in the plugin's runtime `data/` dir. |

`version.h` is preferred over `manifest.OPENPILOT_VERSION` because it is the
version of the code actually running; the manifest constant is hand-maintained
and drifts between rebases.

**Fail-closed.** A missing, unreadable, or corrupt catalog resolves to zero
verified models: nothing is offered for download and nothing can be activated.
The active model keeps running, so the user is never worse off than not touching
models at all. A missing catalog means a broken install, not a state to paper
over.

### Unlock marker

`data/.unlocked` inside the plugin's runtime directory
(`/data/plugins-runtime/model_selector/data/.unlocked`). Created by the
maintainer over ssh; `install.sh`'s `data/` preservation means it survives
deploys. It has no UI surface, so a user browsing Settings cannot discover it.
The download CLI also accepts `--unlocked` for a one-shot bypass.

### Enforcement points

Three, so no path is left ungated:

1. **`model_download.check_updates()`** — the candidate set becomes
   `catalog.verified_entries(type)` minus installed models, replacing the
   registry scan. The `MIN_MODEL_DATE` and revert-substring filters leave the
   user path; curation strictly supersedes them.
2. **`model_download.download_model()`** — refuses an id absent from the catalog
   for the running version unless unlocked. Without this the CLI is an
   accidental hole; with it, it stays deliberately usable.
3. **`model_swapper.swap_model()`** — refuses an unverified id unless unlocked,
   returning an error the UI renders. `list_models()` attaches a `verified`
   boolean to each entry.

`model_registry.json` and the `update-registry` / `add-from-pr` / `list`
commands survive unchanged as **maintainer** tooling. They no longer feed the UI.

### UI behavior

- **CHECK** stops spawning `update-registry`. It becomes a local diff of the
  shipped catalog against installed models — instant, no network. (Downloading
  a model still needs the network.) The row is renamed **Tested Models**, and
  status strings gain `no tested models for <version>`.
- **SELECT** lists unverified installed models with an `untested` marker.
  Activate is disabled for them and explains why when tapped.
- When the **active** model is unverified for the running version — the normal
  consequence of a rebase — its row shows a warning and offers one tap to switch
  to the baseline model. Never a silent auto-swap: that would change how the car
  drives without the driver's consent.

### Maintainer workflow

0. For a new catpilot version, add its `stock_<version>` entry with
   `source: "shipped"` and `baseline_for: ["<version>"]`. No test drive: the
   release's own model is verified by definition.
1. `model_download.py update-registry` (or `add-from-pr <n>`) — unchanged
   GitHub scrape, maintainer-only.
2. `touch /data/plugins-runtime/model_selector/data/.unlocked`, then
   `model_download.py download <id> --type driving`.
3. `model_swapper.py --type driving swap <id>`, reboot, test drive.
4. On a pass: add the entry to `compatible_models.json` with
   `verified_on: ["0.11.1"]` plus notes, commit, push to plugins `dev`. Users
   receive it with their next plugins update — the branch remains the update
   channel.
5. On a rebase to 0.11.2: re-drive, append `"0.11.2"` to entries that pass, and
   set that version's `baseline_for`. Anything not re-driven silently stops being
   offered, which is the desired default.

## Testing

`plugins/model_selector/tests/`:

- **`test_catalog.py`** (new) — version parse from a fake `version.h`, fallback
  chain, `verified_on` filtering, `baseline_entry` resolution, corrupt/missing
  file fails closed, unlock-marker detection, `import_stock` copying only when
  the tracker is absent and being idempotent, and `validate_catalog()` rejecting
  a version with zero or multiple baselines. One test runs `validate_catalog()`
  against the real shipped `compatible_models.json`.
- **`test_model_download.py`** — `check_updates` returns only verified,
  uninstalled entries; returns empty for a version absent from the catalog;
  `download_model` refuses an uncatalogued id and proceeds when unlocked.
- **`test_model_swapper.py`** — `swap_model` refuses unverified, permits when
  unlocked, permits verified; `list_models` sets `verified` correctly. Existing
  date-floor assertions updated.

Run with `PYTHONPATH= uv run pytest plugins/model_selector` — the empty
`PYTHONPATH` avoids the namespace hijack that silently tests a foreign worktree.
The pre-push hook globs `plugins/*/tests/`, so these are collected.

## Documentation

- `DESIGN.md` — the "Compatibility filtering" section is replaced by the catalog
  design; the source-of-models section notes that GitHub discovery is now
  maintainer-only.
- `README.md` — "Things to know" replaces "Old models are hidden" with the
  tested-models rule and the post-rebase warning behavior.

## Out of scope

- Signing or checksumming the catalog. It ships inside the plugin over the same
  trusted git channel as the code that reads it.
- Per-car verification. `verified_on` is keyed by openpilot version only; the
  car a model was driven on goes in `notes`.
- Migrating existing devices' installed models. Anything already on a device
  simply shows as untested until it appears in the catalog.
