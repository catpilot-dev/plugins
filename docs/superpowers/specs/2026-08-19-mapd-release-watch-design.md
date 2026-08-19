# mapd release watch — daily checker with GitHub Issue notification

**Date:** 2026-08-19
**Status:** approved, not yet implemented
**Scope:** `.github/scripts/mapd_watch.py`, `.github/scripts/tests/`, `.github/workflows/mapd-watch.yml` (all new)

## Problem

mapd is dormant. `plugin.json` declares no process because mapd v2.3.0 ships
gomsgq v0.1.10, whose ungated `panic("Invalid Msgq message size")` kills the
binary on a shadow reader's expected torn read — and the resulting respawn loop
leaked reader slots until it took loggerd down. Upstream fixed it in commit
`fe45d10` (PR #133, gomsgq v0.1.11), which is on `main` but in no release.

Re-activation is therefore gated on an upstream release that nothing tells us
about. The interim signal was a session-scoped cron job: it dies with the
session and expires after seven days, so it cannot cover a wait of unknown
length.

## Goal

A daily job that watches `pfeiferj/mapd` releases and opens a GitHub Issue for
each release newer than our pin, so the maintainer receives an email and the
issue already answers the two questions that decide whether the pin can move.

**This is the watcher only. Integration is separate work.** Nothing here bumps
a pin, edits a schema, restores a process entry, or touches a device. It
reports; a human integrates.

## Mechanism

Identical to `model-watch` (`.github/scripts/model_watch.py`), deliberately:
one mechanism to understand, one to maintain.

- Daily `schedule` plus `workflow_dispatch` with `--dry-run`.
- GitHub is the state store. No state file, no bot commits.
- Dedup key is a marker line in the issue body, matched across `state=all`, so
  closing an issue is how the maintainer says "handled" — permanently.
- Authenticated with the built-in `GITHUB_TOKEN`. Unauthenticated API calls
  share a 60/hr per-IP budget on shared runners and will flake.

Marker: `mapd-release: <tag>`. Label: `mapd-release`.

## What it watches

Releases, not tags and not commits. A release is what
`mapd_manager.download_binary()` actually pulls
(`releases/download/<tag>/mapd`; asset name `mapd`, confirmed on v2.3.0). A tag
without a release is not installable, so it is not news.

Draft and prerelease entries are skipped: neither is a thing we would pin to.

## Trigger

A release is reported when `_version_tuple(tag) > _version_tuple(MAX_ALLOWED_VERSION)`,
both imported from `plugins/mapd/mapd_manager.py`.

Importing rather than re-implementing is the same rule that made `model_watch`
share `_github_headers`: one place decides how versions order. It also means
the watch silences itself the moment the pin is bumped — the newly pinned
release stops being "newer than the pin" without anyone editing the watcher.

`mapd_manager` imports only `config`, which is pure and env-driven, and its
module scope merely builds `Path` objects. It imports cleanly in CI with
`plugins/` and `plugins/mapd/` on `sys.path`.

## What the issue body carries

The mapd-specific value. A bare "there is a new release" would leave the
maintainer to do by hand exactly the two checks that decide the outcome.

### 1. Schema diff — does the pin bump need schema work first?

Fetch `cereal/custom/custom.capnp` at the release tag, extract the `MapdOut`
struct, and diff its fields against `plugins/mapd/cereal/slot19.capnp`.

This is the whole reason the pin is load-bearing: Cap'n Proto is additive, so a
newer binary publishing into an older `slot19` **silently drops** every field we
have not declared. `highwayClass` would read back as `unknown` and speedlimitd
would mis-classify every road, with no error anywhere.

Also diff the member lists of the enums `MapdOut` references (`HighwayClass`,
`RoadContext`, `WaySelectionType`) against `plugins/mapd/cereal/standalone.capnp`.
A new enumerant is the same silent failure in a different shape: an out-of-range
value arriving in a field we believe we understand.

Both files are parsed with a line regex (`name @N :Type;` inside a named block),
not a capnp compiler — CI must not need `capnp` installed, and the question is
"which field names and ordinals exist," which the text answers directly.

Verdict in the body: either *schema identical, pin bump is schema-safe*, or
*these N fields / M enumerants must land in our slots before the pin moves*,
listed by name and ordinal.

Baseline for tests: `MapdOut` at v2.3.0 is field-identical to our `slot19.capnp`
(ordinals 0–26), so the clean case has a real fixture.

### 2. Required-commit gate — does this release actually fix the crash?

`REQUIRED_COMMIT = 'fe45d10'`, checked with
`compare/fe45d10...<tag>` → `.status`. `ahead` or `identical` means the release
contains it; `behind` or `diverged` means it does not. Verified against v2.3.0,
which correctly answers `behind`.

Set `REQUIRED_COMMIT = ''` once mapd is re-activated, and the check drops out.
This is a temporary condition wearing a constant's clothes; the comment says so.

A release that is newer but does **not** contain the fix still gets an issue,
carrying that verdict plainly. Suppressing it would leave the maintainer unable
to distinguish "no relevant release yet" from "the watcher stopped running."

### 3. The re-activation checklist

Reproduced from `plugins/mapd/README.md`: restore the `processes` entry, bump
`MAX_ALLOWED_VERSION`, keep every `subscriber` entry on `shadow: true`
(v0.1.11 still has no deregistration, so slotted readers still leak a slot per
death; `selfdriveState` was measured 15/15 full), and the success criterion —
one long-lived mapd PID across a whole drive.

The body `@`-mentions the maintainer so the email arrives regardless of how
watch settings drift.

## Label creation

The script ensures the `mapd-release` label exists before filing, creating it if
absent and treating an "already exists" response as success. `model-watch`
relies on implicit creation at file time; making it explicit here removes a
failure mode that would surface only on the one run that matters.

## Location constraint

`install.sh` copies every `plugins/*/` directory to the device, so this script
must not live under `plugins/mapd/` — it would ship to the car as dead weight.
`.github/scripts/` is the established home, for the same reason.

## Error handling

- Any GitHub API failure is fatal: the job fails loudly and the next day's run
  retries. A watcher that swallows errors and exits 0 is indistinguishable from
  a watcher with nothing to report — the exact failure this design exists to
  prevent.
- The schema fetch is the one exception. If `custom.capnp` cannot be fetched or
  parsed at the tag, the issue is still filed, with the schema section reading
  "could not be checked — diff by hand before bumping." A release notification
  is worth more than a perfect issue body, and the missing check is stated
  rather than implied.
- `--dry-run` fetches the real reported set, so its plan is honest.

## Testing

- Version gate: newer / equal / older / unparseable tags against the pin;
  draft and prerelease skipped.
- Dedup: a tag already carrying a marker in an open **or closed** issue is not
  re-filed.
- Schema diff: identical (the real v2.3.0 fixture), upstream-added field,
  upstream-added enumerant, and the unfetchable-file path.
- Required-commit gate: each of the four `compare` statuses maps to the right
  verdict; empty `REQUIRED_COMMIT` drops the section.
- The workflow itself: one manual `workflow_dispatch --dry-run` run.

Run with `PYTHONPATH= uv run python -m pytest .github/scripts/tests -q`.

## Caveats

- GitHub cron is UTC and best-effort; peak-hour delay and occasional skipped
  runs are normal. Accepted: the dedup key is the release tag, not a time
  window, so a skipped run loses nothing and the next run reports the same
  release. Do not add catch-up or retry logic.
- GitHub disables scheduled workflows after 60 days of repository inactivity.
  `model-watch.yml` already carries this exposure; a second workflow does not
  add to it, but the watch is not truly unattended if the repo goes quiet.

## Out of scope

- Bumping `MAX_ALLOWED_VERSION`, editing slot schemas, restoring the `processes`
  entry, downloading a binary, or touching the C3. The watcher reports; a human
  integrates.
- Watching anything other than releases — no commit or `main`-branch watch.
- Any change to speedlimitd, mapd plugin code, or the dormancy state.
