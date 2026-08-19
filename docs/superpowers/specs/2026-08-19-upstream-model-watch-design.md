# Upstream model watch — daily checker with GitHub Issue notification

**Date:** 2026-08-19
**Status:** approved, not yet implemented
**Scope:** `.github/` (new), `plugins/model_selector/model_download.py`, `plugins/model_selector/catalog.py`

## Problem

Curated compatibility gating removed GitHub scraping from the user path: the UI
now offers only models recorded in `compatible_models.json` as test driven on
the running openpilot version. That is the intended safety property, but it
leaves the maintainer with no signal — nothing tells them comma published a new
model worth driving. The catalog goes stale silently.

## Goal

A daily job that watches `commaai/openpilot` for model changes and opens a
GitHub Issue describing each candidate, so the maintainer receives an email and
has a queue of models to test drive. Nothing it does can put a model in front of
a driver; only a human adding `verified_on` after a drive does that.

## Revert policy

**A revert upstream is not a verdict on whether a model drives well.** comma
reverts for many reasons — a metric regression, infrastructure, a competing
model winning — and none of them are a road test on a BMW E90. A reverted model
that drives well on this fork may be catalogued.

This inverts two existing behaviors in `update_registry_from_github()`:

- **FILTER 2b** currently skips any commit that was later reverted, so a
  reverted model can never become a candidate. Removed.
- **PHASE 2** currently *deletes* registry entries whose commit was reverted.
  Replaced with marking: the entry stays and gains `upstream_reverted: "<sha>"`.

FILTER 2a stays: a revert commit itself is not a model and must not be offered.

Feasibility confirmed: a revert is a new commit, so the reverted commit remains
reachable in master's history and its Git LFS objects stay fetchable.
`download_model` resolving by SHA continues to work for reverted models.

The CLI and the CI checker must apply the same rule. Two revert policies inside
one plugin is how the next confusing bug gets written.

## Components

### 1. `scan_upstream_models()` — extracted, testable parsing

`update_registry_from_github()` currently interleaves fetching, revert
detection, PR-fallback lookup, id generation, and registry writing in a single
function, and is untested. Extract the middle as a pure function:

```python
scan_upstream_models(commits_data: list) -> dict
# {
#   "candidates": [
#     {"id", "name", "commit", "date", "pr", "files", "type",
#      "upstream_reverted": "<revert sha>" | None},
#   ],
#   "reverted": {"<reverted sha>": "<revert commit sha>"},
# }
```

`update_registry_from_github()` calls it and keeps its device-side
responsibilities (loading, merging, writing `model_registry.json`). No behavior
change beyond the revert policy above.

The id it generates (`<clean_name>_<pr_number>`, e.g. `pop_model_37727`) already
matches the catalog's id format, so candidates map onto catalog entries directly.

### 2. `.github/scripts/model_watch.py`

**Location matters:** `install.sh` copies every `plugins/*/` directory to the
device, so this script must not live under `plugins/model_selector/` — it would
ship to the car as dead weight.

Flow:

1. Fetch commits for `selfdrive/modeld/models` from the GitHub API,
   **authenticated with `GITHUB_TOKEN`**. The existing scraper sends no auth
   header; on a shared GitHub runner that shares a 60/hr per-IP budget and will
   flake. The token works for public-repo reads and lifts the limit to 1,000/hr.
2. Call `scan_upstream_models`.
3. Load `compatible_models.json` through `catalog.load_catalog()`.
4. List repository issues `--state all` labelled `model-candidate` or
   `model-revert`, and extract every `upstream-commit:` marker to build the set
   of already-reported SHAs.
5. File what is new:
   - **New candidate** (SHA not reported, not in catalog) → `model-candidate`
     issue. If the model is already reverted upstream at discovery time, the body
     says so and explains it is not disqualifying.
   - **New revert of a SHA already reported or catalogued** → `model-revert`
     issue, informational. It does **not** close the candidate issue and does not
     ask for the catalog entry to be removed — under the revert policy the
     candidate remains valid.

   Exactly one issue per model per event. A model published and reverted between
   two daily runs has never been reported, so it produces a single
   `model-candidate` issue carrying the revert status in its body — not a
   candidate issue plus a revert issue. A `model-revert` issue is only for news
   about something a previous run already told the maintainer about, or that is
   already in the catalog.
6. `--dry-run` prints what it would file without touching issues.

Dedup key is the `upstream-commit: <40-char sha>` line in the issue body.
GitHub is the state store: no state file, no bot commits, and closing an issue
is how the maintainer says "handled", whether the model was catalogued or
rejected after a bad drive.

### 3. `.github/workflows/model-watch.yml`

```yaml
on:
  schedule:
    - cron: '0 6 * * *'
  workflow_dispatch:
permissions:
  issues: write
  contents: read
```

Checks out the repo, installs `requests`, runs the script with `GH_TOKEN` set to
the built-in `GITHUB_TOKEN`. No configured secrets. `workflow_dispatch` exists so
the first run can be triggered by hand.

### 4. Issue format

Title: `Model candidate: <name> (#<pr>)` or `Upstream revert: <name> (#<pr>)`.

Body carries, in order: the `upstream-commit: <sha>` marker line, a metadata
table (name, type, date, PR link, commit link, revert status), a ready-to-paste
catalog JSON block with every field filled except `verified_on` — which only a
drive can earn — and the maintainer checklist:

1. `download <id> --type <type> --unlocked`
2. swap and reboot
3. test drive
4. add `verified_on: ["<version>"]` to `compatible_models.json`, push, deploy

The body `@`-mentions the maintainer. Notifications should already arrive
because they own and watch the repo and the actor is `github-actions[bot]`
rather than themselves, but the mention guarantees the email regardless of how
watch settings drift.

### 5. Catalog schema addition

Entries gain an optional `upstream_reverted: "<revert sha>"`. `validate_catalog`
accepts it and requires nothing of it. It records provenance: if a catalogued
model was later withdrawn upstream, that fact is visible rather than lost.

## Testing

- `scan_upstream_models` against fixture commit payloads: revert detection, the
  PR-fallback path for non-standard commit messages, DM-vs-driving
  classification, id generation matching the catalog format, and — the case this
  design turns on — a reverted commit still appearing as a candidate, marked.
- `model_watch.py` reconcile logic against a fake issue list: already-reported
  SHAs skipped, new candidates filed, reverts filed without closing candidates.
- The workflow itself: one manual `workflow_dispatch` run.

Run with `PYTHONPATH= uv run python -m pytest plugins -q`. The bare
`uv run pytest` form omits the repo root from `sys.path` and silently skips
`test_model_download.py` entirely.

## Caveats

- GitHub cron is UTC and best-effort; peak-hour delays of 10–30 minutes are
  normal and individual runs are occasionally skipped. **Accepted by the
  maintainer as a non-issue** — this is a daily digest feeding a test-drive
  queue, not a latency-sensitive alert. Do not add catch-up, retry, or
  higher-frequency polling to compensate; a skipped run is picked up by the next
  one, because the dedup key is the commit SHA rather than a time window.
- GitHub disables scheduled workflows after 60 days of repository inactivity.
- This is the repository's first workflow; there is no existing CI to follow.

## Out of scope

- Downloading or installing anything in CI. The job reports; the maintainer
  drives.
- Any change to what the UI offers. The gate is unchanged: a model becomes
  installable only when a human adds `verified_on` after a test drive.
- Watching anything other than `selfdrive/modeld/models`.
