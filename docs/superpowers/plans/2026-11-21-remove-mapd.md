# Remove the mapd binary integration — spec + plan (2026-11-21)

Branch: plugins `rebase/v0.11.2`. COD half handled by the `cod-team` session.

## Goal
OSM data comes only from speedlimitd's local tile reader (`osm_query.OsmTileReader`).
Remove every use of the pfeiferj/mapd binary: speedlimitd's mapd consumers and the
whole `mapd` plugin.

## User decisions (2026-11-21)
- Remove the whole `mapd` plugin, not just speedlimitd's use of it.
- Drop the G/S map-curve look-ahead (it needs mapd's road-ahead path; default OFF;
  route 534 showed it could never bind on S20). A tile-geometry look-ahead would be
  a separate project.

## Scope
1. speedlimitd: remove Phase 1 mapd telemetry (`mapd_source.py`, `mapdOut` sub,
   11 `mapd*` published keys) and the map-curve look-ahead (`map_curve.py`,
   `mapdExtendedOut` sub, `MapdCurveLookahead` param, source-4 candidate, 9
   `mapCurve*` published keys, `_update_map_curve` / `_shape_map_curve` /
   `_map_curve_telemetry`).
2. ui_mod: remove the Map Curve Look-ahead toggle row and its callback/test; drop
   "Mapd/" from the OSM toggle title (param `OsmDataIntegration` unchanged); drop
   `'mapd'` from plugins_panel order.
3. mapd plugin and infra: delete `plugins/mapd/`, `.github/workflows/mapd-watch.yml`,
   `.github/scripts/mapd_watch.py`, `.github/scripts/tests/test_mapd_watch.py`.
   Replace the install.sh "ENFORCE mapd" block with the R5 migration (disable → revert
   slots → verify → delete dir; binary stays).
4. Docs that describe current behaviour: speedlimitd DESIGN.md/README.md, ui_mod
   DESIGN.md, top-level README.md table row, docs/INTEGRATION_ARCHITECTURE.md,
   docs/VISION.md.

## Out of scope
- Historical specs/plans under docs/superpowers/ (left as history).
- `cereal/slot0.capnp`: untouched. `mapdSuggested @10` and enum `mapdSuggested @3`
  stay: capnp ordinals cannot be removed without breaking old rlogs; already dead.
- Param names `MapdCurveTargetLatAccel` / `MapdReactLatAccel`: these are VISION-curve
  params with a legacy name; renaming would orphan stored device values. Keep;
  fix only comments that imply they relate to mapd.
- Constant `MAP_CURVE_GATE_KAPPA` is also the vision curve gate (`CURVE_GATE`,
  speedlimitd.py:423) — keep the value; renaming to `CURVE_GATE_KAPPA` allowed only
  if every reference moves with it.
- osm_query / generate_hw_tiles comments that name the tile FORMAT as mapd's (it is
  pfeiferj/mapd's capnp format; that stays true).
- catpilot repo (test fixtures and plugind/registry comments use "mapd" as an
  example name; RELEASES.md history). No functional dependency found.
- connect-on-device: owned by `cod-team` (tile panel moves from plugin id `mapd` to
  `speedlimitd`, `/v1/mapd/tiles*` routes, order map, docs), on COD `dev`.
- Merging this into other plugins branches (dev, main, cheddar-deploy, model-eval,
  lane-keeping-*). They carry the same mapd code and will need the change merged later.
- Device deploy (needs user go-ahead AND cod-team's commit first).

## Requirements
R1 speedlimitd builds and runs with no mapd service in SERVICE_LIST and subscribes to
   neither `mapdOut` nor `mapdExtendedOut` (even if they are present).
R2 Published speedLimitState payload has no key starting with `mapd` or `mapCurve`;
   the speed decision is unchanged for non-map-curve inputs. Key-set test: published
   keys before minus after == exactly the 20 removed keys (11 `mapd*` + 9 `mapCurve*`).
R3 No import of `mapd_source` / `map_curve` anywhere; files deleted.
R4 `plugin.json` has no `MapdCurveLookahead`; Driving panel has no look-ahead row.
R5 install.sh migration of a device that still has the old mapd install (REVISED after
   Gate 1: deleting the dir outright breaks cereal at the next boot. catpilot's
   builder.py strips mapd's standalone types while slots 17-19 still reference them:
   `custom.capnp:73: Not defined: MapdDownloadProgress`. Reproduced by the Director and
   re-run by the PM):
   a. BEFORE `overlay_cereal`: if `$PLUGINS_DEST/mapd` exists, `touch .disabled` and
      `rm -f .enforced`. The existing custom_capnp.py cleanup then reverts slots 17-19
      to `CustomReservedN` stubs (custom.capnp + log.capnp) and drops mapd's standalone
      types. services.py stops re-injecting mapd services.
   b. AFTER `overlay_cereal`: verify the revert. custom.capnp has no `struct MapdOut`
      and log.capnp has `customReserved17 @143`, `customReserved18 @144` and
      `customReserved19 @145`.
      - Verified: `rm -rf "$PLUGINS_DEST/mapd"`, using an exact path with a
        `${PLUGINS_DEST:?}` guard.
      - NOT verified: undo step (a) (rm .disabled, restore .enforced) so the device
        boots exactly as before with the stale mapd copy, and `warn`. A .disabled
        mapd with unreverted slots crashes the same way, so it is never left in place.
   c. Remove the two stale install.sh-injected lines `"mapdOut": (...)` and
      `"mapdExtendedOut": (...)` from cereal/services.py. Add a small tested
      `remove_services(services_py, names)` to plugins/services.py, called from the
      install.sh migration block. builder.py's own auto block is regenerated from
      enabled plugins at boot.
   d. Every step is warn-and-continue. install.sh runs under `set -euo pipefail`, and
      launch_chffrplus.sh swallows its exit (`|| true`), so an abort here would silently
      skip overlay_cereal, the shared config.py/fonts.py copy and the overlay restore.
      Guard each command (`|| warn ...`).
   e. Honours `--dry-run` (echo only). Idempotent: a second run on a migrated device
      does nothing.
   f. Does NOT touch /data/media at all: the mapd binary, mapd_backups/, mapd_version
      and the tiles all stay. Binary deletion is deferred, a PM decision on the
      Director's open question: cheddar-deploy is still driven and still enforces mapd,
      so deleting would force a GitHub re-download on every branch switch. Follow-up
      once no branch the device runs carries mapd. This removes the need for a
      media-dir variable.
   g. The old block's `rm $OPENPILOT_ROOT/mapd_defaults.json` is dropped: only mapd
      read that file, and the old block already removed it every boot.
R5h (Gate 1 round 2) No exit path may leave step (a) applied without (b) having run,
   and that includes a failing cereal step. So the migration owns the cereal step:
   `mapd_migrate <cereal_cmd>` does (a), runs `<cereal_cmd>`, then does (b)/(c).
   install.sh calls `mapd_migrate overlay_cereal` in place of the bare
   `overlay_cereal` call.
   - Bash gotcha: errexit is IGNORED inside anything run as an `if`/`||`/`&&`
     condition, subshells included. So `if ! overlay_cereal` would carry on past a
     failing python and report success. Capture the status with
     `set +e; ( set -e; "$cereal_cmd" ); rc=$?; set -e`, so failures inside still
     stop the subshell and rc is real.
   - Backstop: an EXIT trap that undoes (a) if the migration is still pending.
   - On rc != 0: undo (a), re-run the cereal step best-effort (guarded), warn,
     return 0.
   - The guarantee must live in the file the R6 harness executes.
R5i The migration lives in its OWN file, `install.d/mapd_migration.sh`: only
   function definitions, no top-level side effects. install.sh sources it via
   "$SCRIPT_DIR/install.d/mapd_migration.sh". The harness sources ONLY that file.
   NEVER run or source install.sh itself on this machine: it auto-detects ~/openpilot
   and defaults to the real /data/plugins-runtime, and align_branch can
   `git reset --hard` the plugins checkout. Sandbox targets have no .git.
R5j Verify (b) also requires `struct CustomReserved17`, `18` and `19` stubs in
   custom.capnp. That is the last check before an irreversible rm -rf.
   Step (c) (services line removal) runs whenever no mapd dir remains, including when
   it was already gone, and never on the undo branch.
R6 Required end-to-end test (T3 builds it, PM reruns it): a sandbox harness modelled on
   the Director's sim (scratchpad/director/sim.py). Scratch copies of catpilot cereal
   plus a runtime with the OLD mapd install. Run the NEW install.sh's migration and
   cereal step against the copies, then builder.py's three patches, then `capnp.load`
   log.capnp. Must compile; no `mapd` service left in SERVICE_LIST; second run is a
   no-op. Failure-path cases, each ending with mapd enabled + .enforced and the schema
   compiling after the builder: (i) the verify fails; (ii) the cereal step exits
   non-zero (stub cereal_cmd that exits 1, and one that fails midway under set -e).
   Committed as plugins/tests/test_mapd_migration.py. Old-mapd fixture is a minimal
   copy of baseline 73b20ac:plugins/mapd (plugin.json + cereal/) under
   plugins/tests/fixtures/old_mapd/. Uses scratch copies of the sibling
   ../catpilot cereal + builder.py and skips (pytest.skip) when that repo or pycapnp
   is absent.
   The harness sources only install.d/mapd_migration.sh (R5i) and passes sandbox
   paths; the engineer audits that nothing touches real /data.
R7 Test suite green. Exact command, which only works combined, because the ui_mod tests
   rely on plugins/ being on sys.path (alone, without PYTHONPATH=plugins, they give 44
   import errors):
   `catpilot/.venv/bin/python -m pytest plugins/speedlimitd/tests plugins/ui_mod/tests plugins/tests .github/scripts/tests -q -p no:cacheprovider`
   run from /home/oxygen/catpilot-dev/plugins with `../catpilot/.venv/bin/python`.
   Baseline (with plugins/mapd/tests): 771 passed, 1 failed (mapd test_paths, leaves
   with the plugin).

## Tasks
T1 engineer — speedlimitd removal (Scope 1, docs in speedlimitd/).
   Delete mapd_source.py, map_curve.py, tests/test_mapd_source.py, tests/test_map_curve.py.
   In tests/test_speedlimitd.py: delete TestMapdPhase1Telemetry and the
   TestMapCurve*/look-ahead classes; drop mapd services from the fake SERVICE_LIST; drop
   the tautological "mapd suggestedSpeed" arithmetic tests. ADD tests first (TDD):
   (a) subs contain no mapd service even when SERVICE_LIST has them;
   (b) R2 key-set test;
   (c) plugin.json has no MapdCurveLookahead.
   Verify: R7 command; `grep -rn "mapd_source\|map_curve\|mapdOut\|mapdExtendedOut\|MapdCurveLookahead\|mapCurve" plugins/speedlimitd --exclude-dir=__pycache__` is empty.
T2 engineer — ui_mod (Scope 2, plus plugins/ui_mod/DESIGN.md). Delete
   tests/test_driving_panel_map_curve.py; add a test that driving_panel never
   reads/writes MapdCurveLookahead. Verify: R7 command; grep clean.
T3 engineer — mapd plugin + CI + install.sh migration (R5) + services.remove_services
   + R6 harness + top-level docs (README.md, docs/INTEGRATION_ARCHITECTURE.md,
   docs/VISION.md). Verify: `bash -n install.sh`; R6 harness (both cases); R7 command;
   model-watch.yml still has .github/scripts/tests to run.
T1-T3 touch disjoint files → run in parallel, in one shared tree. To avoid seeing each
   other's half-done edits, each engineer uses its own isolated test command:
   T1 `pytest plugins/speedlimitd/tests` (506 passed at baseline);
   T2 `PYTHONPATH=plugins pytest plugins/ui_mod/tests` (99 passed at baseline);
   T3 `PYTHONPATH=plugins pytest plugins/tests .github/scripts/tests`.
   The combined R7 command is for T4.
T4 PM — rerun R6 harness and R7 command; residual sweep:
   `git grep -n -i mapd -- . ':!docs/superpowers'` reviewed line by line. Allowed
   survivors: slot0.capnp, legacy param names, tile-format comments, the install.sh
   migration block, services.py docstring/test fixture names.
T5 assistant — local commits (one per task), no push.
T6 PM — Gate 2 Director review, then report. Deploy only with the user's go-ahead,
   AFTER cod-team's COD deploy (`./deploy_dev.sh c3`, COD `dev` 20ebcc2, also the
   user's go-ahead, car offroad).

## Risks
- COD tile panel disappears if plugins deploy before the COD dev build is on the
  device → hard ordering gate.
- Rollback: reset /data/plugins to the previous commit and reboot. The old install.sh
  re-copies mapd, re-enforces it and re-injects slots 17-19 (works because they were
  reverted to stubs, not left half-stripped). The binary is still on disk (R5f), so no
  re-download.
- Branch switching on the device: cheddar-deploy and other branches still enforce
  mapd. Switching to one re-installs mapd; switching back runs the migration again.
  Both directions are covered by R5/R6.
- mapd-watch CI: the workflow on `main` checks out `dev` and runs
  .github/scripts/mapd_watch.py. When this change reaches `dev`, delete main's
  mapd-watch.yml in the same step, or the daily job fails. This branch alone does not
  trigger it.
- Other plugins branches diverge further (merge conflicts later).
- Cost: Sonnet engineers ×3, Fable gate reviews; no paid APIs; nothing outward.
