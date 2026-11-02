# Model Selector for openpilot 0.11.2 onwards — design

Status: builder-checked 2026-11-02 (four findings folded in, marked "my call"); awaiting user approval. Branch `rebase/v0.11.2` of the plugins repo.
Companion evidence: `/home/oxygen/catpilot-dev/spike_pop_supercombo/NOTES.md` (PC spike, throwaway).

## Intent and why

catpilot is rebasing onto openpilot 0.11.2. The 0.11.2 small driving model (the one a comma three runs) is CD210, repackaged
as a single `driving_supercombo.onnx` whose history queues live inside the graph. The user's E90 runs the POP model
(#37727) on 0.11.1 and rejected CD210: POP steers more smoothly and, in Experimental mode, its gas and brake respond better.
0.11.2 cannot load any split vision+policy model, comma has published no other small model in the new format, and the
comma-three small model appears frozen. Without this work the rebase silently replaces POP with CD210.

The spike proved the conversion is trivial for POP specifically: POP shares CD210's vision checkpoint byte for byte and
differs only in a same-architecture policy, so a POP supercombo is the stock 0.11.2 file with 50 policy weight arrays
replaced. Verified exact (policy half equals POP's own policy; vision half and queue states bit-identical to stock) and
compiles through the stock tinygrad path unchanged.

## Goals

1. The model selector works on the 0.11.2 tree and model format: `openpilot/` paths, one driving ONNX file, one compiled
   driving artefact, without disturbing anything else in the active model directory.
2. POP is available on 0.11.2 as a finished supercombo file, downloadable through the existing "Tested Models" flow on any
   comma three running catpilot 0.11.2+, and installable by hand on the developer's C3.
3. A maintainer tool converts a legacy split model into a supercombo by policy-weight swap, verifies the result, and
   prints what the catalog entry needs (sha256, size). It runs on the PC only.
4. The compatibility gate keeps its meaning: a model is offered or activated only when the catalog says it was test
   driven on the running openpilot version.

## Non-goals

- On-device conversion. The device never sees the split format again (user decision: convert once on the PC, ship the file).
- A general legacy converter. Only models whose vision weights are identical to the template's and whose policy aligns
  one to one are supported; anything else is rejected with a reason. The other legacy models on the developer's device
  (dark_souls_2, macrostiff, sc_driving, cool_people, …) are out of scope (user decision: 0.11.2 keeps stock CD210 and POP).
- Driver-monitoring models other than stock (user decision: stock DM is fine).
- Chestnut / big-model artefacts (`big_*`), and any support for openpilot versions before 0.11.2 (user decision: the
  plugin on this branch targets 0.11.2 onwards). The `dev` branch keeps serving 0.11.1 as it is.
- Keeping the split format as a second runtime format. Single-file supercombo is the native format from now on.

## Anti-goals (a failure even if it "works")

- Activating a model modeld cannot load. Fail closed: no required ONNX, no catalog verification → not activatable.
- Deleting or modifying git-tracked files in the active model directory (`big_driving_*.pkl` are tracked LFS pointers;
  warp pkls are scons products shared by every model). A swap must touch only its own model's files.
- A silent model change for the driver. Every swap still requires a reboot and shows which model is active.
- Serving a converted file whose weights were not verified against the source policy.

## Constraints

- Repo layout at the 0.11.2 tag: tree root `/data/openpilot`, package `openpilot/`, active models in
  `openpilot/selfdrive/modeld/models/`, version in `openpilot/common/version.h`.
- 0.11.2 compile products (built by scons at boot, `openpilot/selfdrive/modeld/SConscript`): `driving_tinygrad.pkl` from
  `driving_supercombo.onnx`; `dmonitoring_model_tinygrad.pkl` from `dmonitoring_model.onnx`; `driving_warp_*` and
  `dm_warp_*` pkls are model-independent. scons rebuilds a pkl when its ONNX changes, so a swap only has to replace the
  ONNX and remove the model's own stale pkl.
- The compiled pkl carries `metadata.metadata.output_slices` and `model_checkpoint` from the ONNX `metadata_props`; the
  converter must preserve `output_slices` and should set `model_checkpoint` to `<vision ckpt>/<policy ckpt>`.
- catpilot's LFS is read-only (no new LFS objects). The converted file (~61 MB) is hosted elsewhere, see below.
- Catalog policy stays as in `catalog.py`: `verified_on` requires a real test drive on that version; exactly one
  `baseline_for` per type per version; `data/.unlocked` lets a maintainer install untested models.
- Plugin rules: all UI imports lazy inside hook functions; plugin state under `/data/plugins-runtime/model_selector/data/`;
  `compatible_models.json` at the plugin root so deploys overwrite it.
- Device venv has `onnx 1.19`; nothing in this design needs it on the device.
- Prerequisite outside this design: catpilot 0.11.2 must boot and compile the stock supercombo on the C3 (AGNOS 12.8,
  c3_compat gate). Until then POP on 0.11.2 cannot be test driven.

## Decisions already made

### Format and layout (builder implements as stated)

- `ModelSwapper.OPENPILOT_DIR` stays `/data/openpilot`; `ACTIVE_DIR = OPENPILOT_DIR / 'openpilot' / 'selfdrive' / 'modeld' / 'models'`.
- Driving config: `onnx_files = ['driving_supercombo.onnx']`, compiled artefact `driving_tinygrad.pkl`. DM config:
  `onnx_files = ['dmonitoring_model.onnx']`, compiled artefact `dmonitoring_model_tinygrad.pkl`.
- The "remove stale compiled files" step deletes exactly the model type's own compiled artefact in `ACTIVE_DIR`, nothing
  matched by a `*pkl*` glob. PKL caching back into storage likewise copies only that artefact (plus nothing else).
  The `.tinygrad_commit` compatibility rule is unchanged.
- `catalog.VERSION_H = <OPENPILOT_DIR>/openpilot/common/version.h`.
- Legacy model directories (a `driving_vision.onnx` but no `driving_supercombo.onnx`) are not activatable and do not
  appear in the SELECT list (the existing `has_onnx` filter already hides them). No automatic deletion; the developer's
  device is cleaned by hand once. (My call: simplest behaviour that cannot activate an unloadable model.)
- `import_stock` copies `driving_supercombo.onnx` into `/data/models/driving/stock_0.11.2/` when the file in `ACTIVE_DIR`
  is provably the shipped one: shipped catalog entries carry `sha256` and `size` (taken from the tag's LFS pointer, e.g.
  driving `65a08adc31d5…`, 60,918,562 B), and the import runs whenever the active file matches them, whether or not an
  active tracker exists. This replaces the 0.11.1 rule "import only while no tracker exists" (a history heuristic) with
  a byte check, which is strictly stronger. Reason: every device that ever swapped on 0.11.1 carries a tracker, so the
  old rule would leave 0.11.2 devices with no stock baseline in storage and no way back from a bad POP drive.
  (Builder-check finding, my call.)
- Stale tracker migration: when the active tracker names a model directory that lacks the required ONNX for this format
  (a 0.11.1 legacy dir), the tracker is stale — the file actually running is whatever the git tree holds. After the
  stock import above succeeds, the tracker is rewritten to the stock id so the panel shows the truth (CD210 running),
  and the legacy id is never offered for the cancel-reboot swap-back. (My call.)
- Active-file drift detection (in scope, detect only): a catpilot deploy or the updater daemon rewrites the git-tracked
  `driving_supercombo.onnx` in `ACTIVE_DIR`, putting stock CD210 back under a POP tracker, and scons then quietly
  compiles CD210 while the panel says POP. The `device.health_check` hook compares the active file's sha256 (cached by
  size and mtime, computed off the UI thread) with the active model's recorded sha256 (`model_info.json` for converted
  models, the catalog for shipped ones) and sets the Driving Model row's status to "active file differs — re-activate"
  when they differ. No automatic re-apply (that would fight the deploy; out of scope). This is why `model_info.json`
  records the sha256. (Builder-check finding, my call.)

### Catalog for this branch

`compatible_models.json` on `rebase/v0.11.2` lists only 0.11.2+ entries:

- driving `stock_0.11.2`: `source: shipped`, `verified_on: ["0.11.2"]`, `baseline_for: ["0.11.2"]`, notes "CD210 as
  `driving_supercombo.onnx` shipped in catpilot v0.11.2".
- driving `pop_supercombo_37727`: `name` "POP model", `date` 2026-03-24, `pr` "#37727", `source: "converted"`, `files`
  = one object `{name: "driving_supercombo.onnx", url: <release asset URL>, sha256: <hex>, size: <bytes>}`,
  `converted_from: {commit: "12f1be19cc…", vision: "6a7d09ad-…/200", policy: "05a58a51-…/100"}`,
  `verified_on: []` until the developer's test drive on 0.11.2, then `["0.11.2"]`.
  Until then it is installable only through the maintainer CLI with the unlock file present (see below); the UI never
  offers or activates it.
- dm `stock_dm_0.11.2`: shipped, verified and baseline for 0.11.2 (Super Leicht).
- `catalog.validate_catalog` accepts the new `source: converted` kind (requires `files` objects with url/sha256/size) and
  still requires one baseline per type per version.

### Unlock semantics and first install (builder-check finding, my call)

The unlock file keeps "no UI surface": CHECK lists only verified entries and SELECT never enables Activate for an
untested model, unlocked or not. This also means the 0.11.1-era DM directories still on the developer's device cannot be
activated by accident while unlocked. What unlock opens is the CLI only, and the CLI must actually work for an untested
catalog entry: `model_download.py download <id> --type driving --unlocked` (or with `data/.unlocked` present) looks the
id up among all catalog entries, not only verified ones, before falling back to the maintainer registry; `model_swapper.py
swap <id>` likewise accepts an untested catalog model when unlocked. "Installable by hand" in goal 2 means exactly this
CLI path over ssh; there is no install-from-local-file command.

### Download path

`model_download.download_model` gains a direct-URL kind: for `files` entries that are objects with `url`, download to a
temp name, verify `sha256` and `size`, then move into place; `model_info.json` records `source: converted`,
`converted_from`, and the sha256. The existing GitHub-commit + LFS-pointer kind stays for plain filename entries.
`check_updates` needs no change beyond treating the new kind as downloadable.

Hosting (my call, user can override): a GitHub Release on `catpilot-dev/plugins` named `models-0.11.2`, asset
`pop_supercombo_37727.onnx`. Release assets are plain HTTPS, need no auth, and are independent of catpilot's read-only
LFS. The catalog URL is the asset's permanent download URL.

### Converter (maintainer tool, PC only)

`tools/legacy_policy_swap.py` in the plugins repo (not inside the plugin's runtime files). Inputs: template supercombo
(the tag's `driving_supercombo.onnx`), legacy `driving_vision.onnx` + `driving_policy.onnx`, output path. Behaviour,
taken from the spike script `build_pop_supercombo.py` and `validate_swap.py`:

1. Refuse unless every weight array of the legacy vision is byte-identical to one in the template (same vision
   checkpoint) — otherwise print the two checkpoint ids and stop.
2. Build the fp16 weight sequence of the template's embedded policy and of the legacy policy in consuming-node order;
   refuse unless both have the same length and the same (op type, shape) at every position.
3. Replace, in the template, every initializer byte-identical to a template-policy array with the aligned legacy array;
   set `model_checkpoint` to `<vision>/<legacy policy>`; keep `output_slices`; set `producer_name` to identify the tool;
   `onnx.checker.check_model`.
4. `--verify` (default on): with onnxruntime, (a) vision slice, hidden slice and all `next_state_*` equal the template's
   outputs bit for bit on random inputs; (b) the policy slice equals the legacy policy run standalone on the glue's
   policy-input tensors (found as non-initializer tensors consumed by `model/p_*` nodes but produced outside them),
   max abs diff 0; (c) ten recurrent steps stay finite.
5. Print sha256, size, and a ready-to-paste catalog `files` object.

A tinygrad compile is not part of the tool (it is slow and needs the openpilot tree); the maintainer runs the stock
`compile_onnx.py` once by hand, as in the spike, before uploading.

### Upstream watcher

`.github/scripts/model_watch.py` watches `openpilot/selfdrive/modeld/models/driving_supercombo.onnx` and
`dmonitoring_model.onnx` (path and names as of 0.11.2). It still only reports; a report never makes a model installable.
Go-live: the workflow runs from the default branch and checks out `dev`, so the new script takes effect when this branch
becomes the 0.11.2 release/default branch; nothing is edited on other branches now. Until then `dev`'s watcher stays
quiet, which is the truthful state for split files upstream no longer changes. (My call, low stakes.)

### Developer's C3, one-off (after the boot gate)

1. Convert POP on the PC with the tool, compile once with `compile_onnx.py` as a smoke test, upload the release asset,
   add the catalog entry with `verified_on: []`.
2. Over ssh: create `/data/plugins-runtime/model_selector/data/.unlocked`, run the download CLI for
   `pop_supercombo_37727`, then the swap CLI; reboot; first boot compiles (~1 min). Confirm the Driving Model row shows
   POP with no drift warning before driving.
3. Test drive. If POP is kept, set `verified_on: ["0.11.2"]` and remove the unlock file.
4. Delete the legacy directories under `/data/models/driving/` (all nine) and `/data/models/dm/` extras by hand.

## Left to the builder

- Exact UI wording for a model whose directory lacks the supercombo file, if any is shown at all.
- Test fixtures: synthetic tiny ONNX graphs for the converter's alignment/replace/verify logic (the real 60 MB files
  are not committed); updated `test_model_swapper`, `test_catalog`, `test_model_download` for the new config, catalog
  kind and download path.
- How `download_file` reports progress for a 61 MB direct download in the existing CHECK/download subprocess UI.
- Documentation updates in `plugins/model_selector/README.md` and `DESIGN.md` (format change, converted kind, 0.11.2+).
