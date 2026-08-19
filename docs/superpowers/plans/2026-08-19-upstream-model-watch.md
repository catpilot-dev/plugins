# Upstream Model Watch Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A daily GitHub Actions job that watches commaai/openpilot for driving and driver-monitoring model changes and opens a GitHub Issue per candidate, giving the maintainer an emailed test-drive queue.

**Architecture:** The commit-parsing logic buried inside `update_registry_from_github()` is extracted as a pure, testable `scan_upstream_models()` that both the device CLI and a new CI script call, so one revert policy governs both. The CI script reconciles upstream candidates against the curated catalog and against existing GitHub issues — issues themselves are the dedup store, keyed by an `upstream-commit:` marker — and files what is new.

**Tech Stack:** Python 3.12, `requests`, pytest, GitHub Actions, `GITHUB_TOKEN`. No new dependencies and no configured secrets.

**Spec:** `docs/superpowers/specs/2026-08-19-upstream-model-watch-design.md`

## Global Constraints

- Repo: `/home/oxygen/catpilot-dev/plugins`, branch `dev`.
- Run tests as `PYTHONPATH= uv run python -m pytest plugins -q` from the repo root. **Never bare `uv run pytest`** — it omits the repo root from `sys.path`, so `test_model_download.py` fails to import and skips its entire module, showing a false green.
- **A revert upstream is not a verdict on whether a model drives.** Reverted models stay eligible as candidates and may be catalogued. A revert commit itself is never a candidate.
- `install.sh` copies every `plugins/*/` directory to the device. CI-only code must NOT live under `plugins/model_selector/` — it would ship to the car as dead weight.
- All GitHub API calls from CI must send `Authorization: Bearer $GITHUB_TOKEN`. Unauthenticated calls share a 60/hr per-IP budget on shared runners and will flake.
- Nothing in this feature may make a model installable. Only a human adding `verified_on` after a test drive does that.
- Do NOT commit unrelated working-tree changes. `plugins/c3_compat/boot_patch.sh` is modified and `overlays/` is untracked; both stay out of every commit. Use explicit `git add <paths>`, never `git commit -a`.
- No `Co-Authored-By` lines in commit messages.
- Do not ssh to the C3, deploy, run `install.sh`, or push to any remote. Local commits only.
- Do not add catch-up, retry, or higher-frequency polling for missed cron runs. Dedup is keyed on commit SHA, not a time window, so a skipped run is picked up by the next one. The maintainer has accepted schedule drift.

## File Structure

| File | Responsibility |
|---|---|
| `plugins/model_selector/model_download.py` (modify) | Gains pure `scan_upstream_models()`; `update_registry_from_github()` becomes fetch + merge + write around it, and marks reverted models instead of deleting them. |
| `plugins/model_selector/catalog.py` (modify) | `validate_catalog` tolerates the optional `upstream_reverted` field. |
| `.github/scripts/model_watch.py` (create) | The CI checker: fetch → scan → reconcile against catalog and issues → file issues. |
| `.github/workflows/model-watch.yml` (create) | Daily schedule + manual dispatch; wires `GITHUB_TOKEN`. |
| `plugins/model_selector/tests/test_model_download.py` (modify) | Tests for `scan_upstream_models`. |
| `.github/scripts/tests/test_model_watch.py` (create) | Tests for reconcile logic. |
| `plugins/model_selector/DESIGN.md` (modify) | Documents the revert policy and the watch job. |

---

### Task 1: Extract `scan_upstream_models()` and invert the revert policy

**Files:**
- Modify: `plugins/model_selector/model_download.py:506-694`
- Test: `plugins/model_selector/tests/test_model_download.py`

**Interfaces:**
- Consumes: nothing from other tasks.
- Produces, consumed by Task 3:
  ```python
  scan_upstream_models(commits_data: list) -> dict
  # {
  #   "candidates": [
  #     {"id": str, "name": str, "commit": str, "date": str, "pr": str,
  #      "files": list[str], "type": "driving"|"dm",
  #      "upstream_reverted": str | None},   # sha of the revert commit
  #   ],
  #   "reverted": {reverted_sha: revert_commit_sha},
  # }
  ```
  Candidates are ordered newest-first, as the API returns them.

- [ ] **Step 1: Write the failing tests**

Append to `plugins/model_selector/tests/test_model_download.py`:

```python
def _commit(sha, message, date='2025-12-01T00:00:00Z'):
  return {'sha': sha, 'commit': {'message': message, 'committer': {'date': date}}}


class TestScanUpstreamModels:
  def test_parses_a_standard_model_commit(self):
    out = md.scan_upstream_models([_commit('a' * 40, 'Nice Model (#37727)')])
    assert len(out['candidates']) == 1
    c = out['candidates'][0]
    assert c['id'] == 'nice_model_37727'
    assert c['name'] == 'Nice Model'
    assert c['commit'] == 'a' * 40
    assert c['pr'] == '(#37727)'
    assert c['type'] == 'driving'
    assert c['files'] == ['driving_vision.onnx', 'driving_policy.onnx']
    assert c['upstream_reverted'] is None

  def test_detects_dm_models(self):
    out = md.scan_upstream_models([_commit('b' * 40, 'DM: Sharp Eyes (#37800)')])
    c = out['candidates'][0]
    assert c['type'] == 'dm'
    assert c['files'] == ['dmonitoring_model.onnx']
    assert c['name'] == 'Sharp Eyes'

  def test_skips_commits_before_the_firehose_floor(self):
    out = md.scan_upstream_models([_commit('c' * 40, 'Old Model (#100)', '2025-01-01T00:00:00Z')])
    assert out['candidates'] == []

  def test_revert_commit_itself_is_never_a_candidate(self):
    revert = _commit('d' * 40, f"Revert \"Nice Model (#37727)\"\n\nThis reverts commit {'a' * 40}.")
    out = md.scan_upstream_models([revert])
    assert out['candidates'] == []
    assert out['reverted'] == {'a' * 40: 'd' * 40}

  def test_reverted_model_stays_a_candidate_and_is_marked(self):
    """A revert is not a road test — the model must remain drivable and catalogable."""
    revert = _commit('d' * 40, f"Revert \"Nice Model (#37727)\"\n\nThis reverts commit {'a' * 40}.")
    model = _commit('a' * 40, 'Nice Model (#37727)')
    out = md.scan_upstream_models([revert, model])
    ids = [c['id'] for c in out['candidates']]
    assert ids == ['nice_model_37727']
    assert out['candidates'][0]['upstream_reverted'] == 'd' * 40

  def test_pr_fallback_when_message_has_no_pr_number(self, monkeypatch):
    class _Resp:
      def raise_for_status(self): pass
      def json(self): return [{'number': 12345, 'title': 'Fallback Model'}]
    monkeypatch.setattr(md.requests, 'get', lambda *a, **k: _Resp())
    out = md.scan_upstream_models([_commit('e' * 40, 'some model bump')])
    c = out['candidates'][0]
    assert c['pr'] == '#12345'
    assert c['name'] == 'Fallback Model'
    assert c['id'] == 'fallback_model_12345'

  def test_pr_fallback_failure_drops_the_commit(self, monkeypatch):
    def _boom(*a, **k):
      raise RuntimeError('network down')
    monkeypatch.setattr(md.requests, 'get', _boom)
    assert md.scan_upstream_models([_commit('f' * 40, 'unlabelled bump')])['candidates'] == []

  def test_id_matches_the_catalog_format(self):
    """Catalog ids like pop_model_37727 come from this function — they must agree."""
    out = md.scan_upstream_models([_commit('0' * 40, 'POP model (#37727)')])
    assert out['candidates'][0]['id'] == 'pop_model_37727'
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `PYTHONPATH= uv run python -m pytest plugins/model_selector/tests/test_model_download.py -q`
Expected: FAIL — `AttributeError: module ... has no attribute 'scan_upstream_models'`

- [ ] **Step 3: Add `scan_upstream_models` to `model_download.py`**

Insert immediately above `def update_registry_from_github():` (line 506):

```python
# Firehose model onward — earlier commits predate the current model interface
_MODEL_COMMIT_FLOOR = "2025-09-05"


def _parse_reverts(commits_data: list) -> dict:
    """Map reverted commit sha -> the sha of the commit that reverted it."""
    reverted = {}
    for commit_data in commits_data:
        message = commit_data['commit']['message']
        if not message.split('\n')[0].lower().startswith('revert'):
            continue
        match = re.search(r'reverts commit ([0-9a-f]{40})', message, re.IGNORECASE)
        if match:
            reverted[match.group(1)] = commit_data['sha']
    return reverted


def scan_upstream_models(commits_data: list) -> dict:
    """Parse GitHub commits touching the model dir into model candidates.

    Pure: no network except the PR-title fallback, no disk, no registry. Shared
    by the device CLI (update-registry) and the CI watch job so that one revert
    policy governs both.

    A model whose commit was later reverted upstream stays a candidate, marked
    with `upstream_reverted`. comma reverts for many reasons — a metric
    regression, infrastructure, a competing model winning — and none of them is
    a road test on this fork. Only the revert commit itself is excluded, because
    it is not a model.
    """
    reverted = _parse_reverts(commits_data)
    candidates = []

    for commit_data in commits_data:
        commit_hash = commit_data['sha']
        commit_message = commit_data['commit']['message']
        commit_date = commit_data['commit']['committer']['date'][:10]

        if commit_date < _MODEL_COMMIT_FLOOR:
            continue

        # The revert commit itself is not a model
        if commit_message.split('\n')[0].lower().startswith('revert'):
            continue

        if '(#' not in commit_message:
            try:
                pr_resp = requests.get(
                    f"https://api.github.com/repos/commaai/openpilot/commits/{commit_hash}/pulls",
                    headers=_github_headers(),
                    timeout=10,
                )
                pr_resp.raise_for_status()
                prs = pr_resp.json()
                if not prs:
                    continue
                pr_number = f"#{prs[0]['number']}"
                model_name = prs[0]['title'].strip()
            except Exception:
                continue
        else:
            pr_match = commit_message.find('(#')
            pr_end = commit_message.find(')', pr_match)
            pr_number = commit_message[pr_match:pr_end + 1]
            model_name = commit_message[:pr_match].strip()

        if 'DM:' in model_name or 'dmonitoring' in commit_message.lower():
            model_type = 'dm'
            model_name = model_name.replace('DM:', '').strip()
            files = ['dmonitoring_model.onnx']
        else:
            model_type = 'driving'
            files = ['driving_vision.onnx', 'driving_policy.onnx']

        clean_name = re.sub(r'[^a-z0-9_]', '', model_name.lower().replace(' ', '_'))
        pr_id = pr_number.strip('#()') if pr_number else commit_hash[:7]

        candidates.append({
            'id': f"{clean_name}_{pr_id}",
            'name': model_name,
            'commit': commit_hash,
            'date': commit_date,
            'pr': pr_number,
            'files': files,
            'type': model_type,
            'upstream_reverted': reverted.get(commit_hash),
        })

    return {'candidates': candidates, 'reverted': reverted}
```

Add `import re` to the module's top-level imports if it is not already there (the old code imported it twice inside the function body — remove both local `import re` statements while you are in there).

Add the shared header helper near the other module-level constants:

```python
def _github_headers() -> dict:
    """Auth GitHub API calls when a token is available.

    CI must send this: unauthenticated calls share a 60/hr per-IP budget on
    shared runners and will flake. On device the env var is absent and the call
    falls back to unauthenticated, which is fine at one run per tap.
    """
    headers = {"Accept": "application/vnd.github.v3+json"}
    token = os.environ.get("GH_TOKEN") or os.environ.get("GITHUB_TOKEN")
    if token:
        headers["Authorization"] = f"Bearer {token}"
    return headers
```

Add `import os` at module level if absent.

- [ ] **Step 4: Rewrite `update_registry_from_github` to use it**

Replace the body from `# Load existing registry` (line 538) through the end of PHASE 3 (line 676) with:

```python
    # Load existing registry
    with open(REGISTRY_FILE) as f:
        registry = json.load(f)

    scan = scan_upstream_models(commits_data)

    existing_commits = set()
    for models_dict in [registry['driving_models'], registry['dm_models']]:
        for model_info in models_dict.values():
            existing_commits.add(model_info['commit'])

    # Mark — never delete — registry entries whose commit was reverted upstream.
    # A revert is not a verdict on whether the model drives; see DESIGN.md.
    models_marked = 0
    for registry_key in ['driving_models', 'dm_models']:
        for model_id, model_info in registry[registry_key].items():
            revert_sha = scan['reverted'].get(model_info['commit'])
            if revert_sha and model_info.get('upstream_reverted') != revert_sha:
                model_info['upstream_reverted'] = revert_sha
                models_marked += 1
                print(f"  ⚠️  Upstream reverted (kept): {model_id}")

    new_models_added = 0
    for cand in scan['candidates']:
        if cand['commit'] in existing_commits:
            continue

        registry_key = 'dm_models' if cand['type'] == 'dm' else 'driving_models'

        # Deduplicate by PR: a re-pushed model reuses its existing entry id
        pr_str = cand['pr'].strip('#()')
        existing_id = None
        for mid, minfo in registry[registry_key].items():
            if minfo.get('pr', '').strip('#()') == pr_str:
                existing_id = mid
                break
        model_id = existing_id or cand['id']

        registry[registry_key][model_id] = {
            'name': cand['name'],
            'commit': cand['commit'],
            'date': cand['date'],
            'description': f"Model from {cand['date']}",
            'pr': cand['pr'],
            'files': cand['files'],
            'upstream_reverted': cand['upstream_reverted'],
        }
        new_models_added += 1

        print(f"✅ Found new {cand['type']} model: {cand['name']}")
        print(f"   ID: {model_id}")
        print(f"   Commit: {cand['commit'][:7]}")
        print(f"   Date: {cand['date']}")
        print(f"   PR: {cand['pr']}")
        if cand['upstream_reverted']:
            print("   NOTE: reverted upstream — still eligible, test drive decides")
        print()
```

Then update the save block (line 678) so it triggers on `models_marked` rather than the deleted `models_removed`:

```python
    if new_models_added > 0 or models_marked > 0:
        registry['last_updated'] = datetime.now().strftime('%Y-%m-%d')

        with open(REGISTRY_FILE, 'w') as f:
            json.dump(registry, f, indent=2)

        if new_models_added > 0:
            print(f"✅ Added {new_models_added} new model(s) to registry")
        if models_marked > 0:
            print(f"⚠️  Marked {models_marked} model(s) reverted upstream (kept in registry)")
        print(f"📄 Registry updated: {REGISTRY_FILE}")
    else:
        print("✅ Registry is up to date")
```

Also add the auth headers to the commits fetch at line 530:

```python
        response = requests.get(github_api_url, params=params, headers=_github_headers())
```

- [ ] **Step 5: Update the function's docstring**

The docstring at lines 506-520 describes the old policy ("Removes reverted models from registry"). Replace that bullet with:

```
       - Detects "Revert" commits and MARKS the reverted model with
         `upstream_reverted`; it is never removed. A revert is not a verdict on
         whether the model drives — only a test drive is.
```

- [ ] **Step 6: Run the tests**

Run: `PYTHONPATH= uv run python -m pytest plugins/model_selector -q`
Expected: all pass, skip count 0.

- [ ] **Step 7: Commit**

```bash
git add plugins/model_selector/model_download.py plugins/model_selector/tests/test_model_download.py
git commit -m "refactor(model_selector): extract scan_upstream_models; reverts mark, not delete"
```

---

### Task 2: Catalog tolerates `upstream_reverted`

**Files:**
- Modify: `plugins/model_selector/catalog.py` (`validate_catalog`)
- Test: `plugins/model_selector/tests/test_catalog.py`

**Interfaces:**
- Consumes: nothing.
- Produces: catalog entries may carry `upstream_reverted: "<sha>"`. `validate_catalog` accepts it and requires nothing of it.

- [ ] **Step 1: Write the failing test**

Append to `plugins/model_selector/tests/test_catalog.py`, inside `TestValidateCatalog`:

```python
  def test_upstream_reverted_field_is_allowed(self, catalog_env):
    """A model comma withdrew can still be catalogued — the field is provenance."""
    c = {'driving': [_entry(upstream_reverted='d' * 40), _stock()],
         'dm': [_stock(id='stock_dm_0.11.1', name='Release default DM')]}
    assert catalog_env.validate_catalog(c) == []

  def test_upstream_reverted_does_not_affect_verification(self, catalog_env):
    _write(catalog_env, {'driving': [_entry(upstream_reverted='d' * 40)], 'dm': []})
    assert catalog_env.is_verified('driving', 'cool_people_3c957c6')
```

- [ ] **Step 2: Run to verify**

Run: `PYTHONPATH= uv run python -m pytest plugins/model_selector/tests/test_catalog.py -q`
Expected: both PASS already if `validate_catalog` ignores unknown fields, or FAIL if it rejects them.

If they already pass, `validate_catalog` needs no change — record that in the report, skip Step 3, and go to Step 4. Do not add code that is not needed.

- [ ] **Step 3: Only if a test failed — allow the field**

`validate_catalog` checks required fields; it must not reject additional ones. If it does, remove that rejection rather than adding `upstream_reverted` to an allowlist — the catalog is maintainer-authored data and an unknown-field whitelist is churn nobody asked for.

- [ ] **Step 4: Commit**

```bash
git add plugins/model_selector/catalog.py plugins/model_selector/tests/test_catalog.py
git commit -m "test(model_selector): catalog accepts upstream_reverted provenance"
```

(If `catalog.py` was unchanged, commit only the test file.)

---

### Task 3: The CI checker script

**Files:**
- Create: `.github/scripts/model_watch.py`
- Create: `.github/scripts/tests/test_model_watch.py`

**Interfaces:**
- Consumes: `scan_upstream_models(commits_data) -> {"candidates": [...], "reverted": {...}}` from Task 1; `catalog.load_catalog()` returning `{"driving": [...], "dm": [...]}`.
- Produces: `main(argv) -> int`, plus the pure functions below that Task 4's workflow depends on only through `main`.

  ```python
  MARKER = 'upstream-commit:'
  reported_shas(issues: list[dict]) -> set[str]
  plan_issues(scan: dict, catalog: dict, reported: set[str]) -> list[dict]
  # each planned issue: {"kind": "candidate"|"revert", "sha": str, "title": str,
  #                      "labels": list[str], "body": str}
  ```

**This script must not live under `plugins/model_selector/`** — `install.sh` copies every `plugins/*/` directory to the device.

- [ ] **Step 1: Write the failing tests**

Create `.github/scripts/tests/test_model_watch.py`:

```python
"""Tests for the upstream model watch reconcile logic."""
import importlib.util
import pathlib
import sys

import pytest

_SCRIPT = pathlib.Path(__file__).resolve().parents[1] / 'model_watch.py'
_spec = importlib.util.spec_from_file_location('model_watch', _SCRIPT)
mw = importlib.util.module_from_spec(_spec)
sys.modules['model_watch'] = mw
_spec.loader.exec_module(mw)


def _cand(**over):
  c = {'id': 'nice_model_37727', 'name': 'Nice Model', 'commit': 'a' * 40,
       'date': '2025-12-01', 'pr': '(#37727)', 'type': 'driving',
       'files': ['driving_vision.onnx', 'driving_policy.onnx'],
       'upstream_reverted': None}
  c.update(over)
  return c


def _issue(sha, state='open'):
  return {'number': 1, 'state': state, 'body': f'{mw.MARKER} {sha}\n\nrest'}


class TestReportedShas:
  def test_extracts_marker_from_bodies(self):
    assert mw.reported_shas([_issue('a' * 40), _issue('b' * 40)]) == {'a' * 40, 'b' * 40}

  def test_closed_issues_still_count_as_reported(self):
    """Closed means handled — catalogued or rejected. Never re-report it."""
    assert mw.reported_shas([_issue('a' * 40, state='closed')]) == {'a' * 40}

  def test_ignores_bodies_without_the_marker(self):
    assert mw.reported_shas([{'number': 2, 'state': 'open', 'body': 'unrelated'}]) == set()

  def test_tolerates_missing_body(self):
    assert mw.reported_shas([{'number': 3, 'state': 'open', 'body': None}]) == set()


class TestPlanIssues:
  def _catalog(self, ids=()):
    return {'driving': [{'id': i, 'commit': 'c' * 40} for i in ids], 'dm': []}

  def test_files_a_new_candidate(self):
    scan = {'candidates': [_cand()], 'reverted': {}}
    planned = mw.plan_issues(scan, self._catalog(), set())
    assert [p['kind'] for p in planned] == ['candidate']
    assert planned[0]['sha'] == 'a' * 40
    assert 'model-candidate' in planned[0]['labels']
    assert f"{mw.MARKER} {'a' * 40}" in planned[0]['body']

  def test_skips_already_reported(self):
    scan = {'candidates': [_cand()], 'reverted': {}}
    assert mw.plan_issues(scan, self._catalog(), {'a' * 40}) == []

  def test_skips_models_already_catalogued(self):
    scan = {'candidates': [_cand()], 'reverted': {}}
    assert mw.plan_issues(scan, self._catalog(ids=['nice_model_37727']), set()) == []

  def test_never_reported_and_reverted_yields_one_candidate_issue_only(self):
    """Published and reverted between two runs: one issue, not two."""
    scan = {'candidates': [_cand(upstream_reverted='d' * 40)], 'reverted': {'a' * 40: 'd' * 40}}
    planned = mw.plan_issues(scan, self._catalog(), set())
    assert [p['kind'] for p in planned] == ['candidate']
    assert 'reverted' in planned[0]['body'].lower()

  def test_revert_of_an_already_reported_model_files_a_revert_issue(self):
    scan = {'candidates': [_cand(upstream_reverted='d' * 40)], 'reverted': {'a' * 40: 'd' * 40}}
    planned = mw.plan_issues(scan, self._catalog(), {'a' * 40})
    assert [p['kind'] for p in planned] == ['revert']
    assert 'model-revert' in planned[0]['labels']

  def test_revert_issue_is_deduped_by_its_own_marker(self):
    scan = {'candidates': [_cand(upstream_reverted='d' * 40)], 'reverted': {'a' * 40: 'd' * 40}}
    planned = mw.plan_issues(scan, self._catalog(), {'a' * 40, 'd' * 40})
    assert planned == []

  def test_revert_of_a_catalogued_model_is_flagged_in_the_title(self):
    cat = {'driving': [{'id': 'nice_model_37727', 'commit': 'a' * 40}], 'dm': []}
    scan = {'candidates': [_cand(upstream_reverted='d' * 40)], 'reverted': {'a' * 40: 'd' * 40}}
    planned = mw.plan_issues(scan, cat, {'a' * 40})
    assert planned[0]['kind'] == 'revert'
    assert 'catalog' in planned[0]['title'].lower()

  def test_candidate_body_carries_a_paste_ready_entry_without_verified_on(self):
    planned = mw.plan_issues({'candidates': [_cand()], 'reverted': {}}, self._catalog(), set())
    body = planned[0]['body']
    assert '"id": "nice_model_37727"' in body
    assert '"commit": "' + 'a' * 40 + '"' in body
    assert 'verified_on' not in body.split('```')[1]
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `PYTHONPATH= uv run python -m pytest .github/scripts/tests -q`
Expected: collection error — `model_watch.py` does not exist.

- [ ] **Step 3: Write `.github/scripts/model_watch.py`**

```python
#!/usr/bin/env python3
"""Daily watch for new upstream openpilot models — files a GitHub Issue per candidate.

Reports only. Nothing here can put a model in front of a driver: a model becomes
installable when a human adds `verified_on` to compatible_models.json after a
test drive, and never otherwise.

Dedup state lives in the issues themselves. Each issue body carries an
`upstream-commit: <sha>` marker; any sha found in an open OR closed issue is
never reported again, because closing an issue is how the maintainer says
"handled" — whether the model was catalogued or rejected after a bad drive.
"""
import argparse
import json
import os
import sys
from pathlib import Path

import requests

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / 'plugins' / 'model_selector'))

import catalog  # noqa: E402
from model_download import scan_upstream_models  # noqa: E402

MARKER = 'upstream-commit:'
WATCH_REPO = 'commaai/openpilot'
MODEL_PATH = 'selfdrive/modeld/models'
MAINTAINER = '@OxygenLiu'


def _headers() -> dict:
    headers = {"Accept": "application/vnd.github.v3+json"}
    token = os.environ.get("GH_TOKEN") or os.environ.get("GITHUB_TOKEN")
    if token:
        headers["Authorization"] = f"Bearer {token}"
    return headers


def fetch_commits(per_page: int = 30) -> list:
    resp = requests.get(
        f"https://api.github.com/repos/{WATCH_REPO}/commits",
        params={'path': MODEL_PATH, 'per_page': per_page},
        headers=_headers(), timeout=30,
    )
    resp.raise_for_status()
    return resp.json()


def fetch_issues(repo: str) -> list:
    """All watch issues, open and closed, across pages."""
    issues, page = [], 1
    while True:
        resp = requests.get(
            f"https://api.github.com/repos/{repo}/issues",
            params={'state': 'all', 'labels': 'model-candidate,model-revert',
                    'per_page': 100, 'page': page},
            headers=_headers(), timeout=30,
        )
        resp.raise_for_status()
        batch = resp.json()
        if not batch:
            return issues
        issues.extend(batch)
        page += 1


def reported_shas(issues: list) -> set:
    shas = set()
    for issue in issues:
        for line in (issue.get('body') or '').splitlines():
            line = line.strip()
            if line.startswith(MARKER):
                shas.add(line[len(MARKER):].strip())
    return shas


def _catalog_ids(cat: dict) -> set:
    return {e['id'] for entries in cat.values() for e in entries}


def _candidate_body(cand: dict) -> str:
    entry = {
        'id': cand['id'], 'name': cand['name'], 'date': cand['date'],
        'commit': cand['commit'], 'pr': cand['pr'].strip('()'),
        'files': cand['files'],
    }
    revert_note = ''
    if cand['upstream_reverted']:
        revert_note = (
            f"\n> **Reverted upstream** by `{cand['upstream_reverted'][:12]}`. "
            "That is not a verdict on whether it drives — comma reverts for many "
            "reasons, none of them a road test on this car. Still worth a drive.\n"
        )
    return f"""{MARKER} {cand['commit']}

{MAINTAINER} — new upstream model candidate.
{revert_note}
| | |
|---|---|
| Name | {cand['name']} |
| Type | {cand['type']} |
| Date | {cand['date']} |
| PR | https://github.com/{WATCH_REPO}/pull/{cand['pr'].strip('#()')} |
| Commit | https://github.com/{WATCH_REPO}/commit/{cand['commit']} |

Paste into `plugins/model_selector/compatible_models.json` **after** a test
drive, adding `"verified_on": ["<version>"]`:

```json
{json.dumps(entry, indent=2)}
```

- [ ] `model_download.py download {cand['id']} --type {cand['type']} --unlocked`
- [ ] `model_swapper.py --type {cand['type']} swap {cand['id']}`, reboot
- [ ] test drive
- [ ] add `verified_on`, commit, push, deploy

Close this issue once catalogued — or once rejected. Either way it will not be
reported again.
"""


def _revert_body(cand: dict, catalogued: bool) -> str:
    state = ("**This model is in your catalog.** Deployed devices can still "
             "activate it. Nothing is required — your test drive stands — but "
             "you may want to know why comma pulled it."
             if catalogued else
             "Already reported as a candidate; recorded here for the trail.")
    return f"""{MARKER} {cand['upstream_reverted']}

{MAINTAINER} — upstream reverted a model you have already been told about.

| | |
|---|---|
| Model | {cand['name']} (`{cand['id']}`) |
| Model commit | https://github.com/{WATCH_REPO}/commit/{cand['commit']} |
| Revert commit | https://github.com/{WATCH_REPO}/commit/{cand['upstream_reverted']} |

{state}

A revert is not a verdict on whether the model drives well.
"""


def plan_issues(scan: dict, cat: dict, reported: set) -> list:
    planned = []
    catalogued = _catalog_ids(cat)

    for cand in scan['candidates']:
        sha = cand['commit']
        revert_sha = cand.get('upstream_reverted')

        if sha not in reported and cand['id'] not in catalogued:
            # Never reported: one candidate issue, carrying revert status in the
            # body. Not a candidate issue plus a revert issue.
            planned.append({
                'kind': 'candidate', 'sha': sha,
                'title': f"Model candidate: {cand['name']} ({cand['pr'].strip('()')})",
                'labels': ['model-candidate'],
                'body': _candidate_body(cand),
            })
            continue

        # Already known. Only a revert is news, and only once.
        if revert_sha and revert_sha not in reported:
            in_catalog = cand['id'] in catalogued
            prefix = "Upstream revert (in catalog)" if in_catalog else "Upstream revert"
            planned.append({
                'kind': 'revert', 'sha': revert_sha,
                'title': f"{prefix}: {cand['name']} ({cand['pr'].strip('()')})",
                'labels': ['model-revert'],
                'body': _revert_body(cand, in_catalog),
            })

    return planned


def create_issue(repo: str, planned: dict) -> int:
    resp = requests.post(
        f"https://api.github.com/repos/{repo}/issues",
        headers=_headers(), timeout=30,
        json={'title': planned['title'], 'body': planned['body'],
              'labels': planned['labels']},
    )
    resp.raise_for_status()
    return resp.json()['number']


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', default=os.environ.get('GITHUB_REPOSITORY'),
                        help='owner/name to file issues in')
    parser.add_argument('--dry-run', action='store_true',
                        help='print what would be filed, touch nothing')
    args = parser.parse_args(argv)

    if not args.repo:
        print("error: --repo or GITHUB_REPOSITORY required", file=sys.stderr)
        return 2

    scan = scan_upstream_models(fetch_commits())
    cat = catalog.load_catalog()
    reported = set() if args.dry_run else reported_shas(fetch_issues(args.repo))
    if args.dry_run:
        reported = reported_shas(fetch_issues(args.repo))

    planned = plan_issues(scan, cat, reported)
    print(f"{len(scan['candidates'])} candidates upstream, "
          f"{len(reported)} already reported, {len(planned)} to file")

    for item in planned:
        if args.dry_run:
            print(f"  [dry-run] {item['kind']}: {item['title']}")
            continue
        number = create_issue(args.repo, item)
        print(f"  filed #{number}: {item['title']}")

    return 0


if __name__ == '__main__':
    sys.exit(main())
```

Note the `--dry-run` block above is written twice by mistake in the draft — write it once, as:

```python
    reported = reported_shas(fetch_issues(args.repo))
```

with no conditional; a dry run still needs the real reported set to print an honest plan.

- [ ] **Step 4: Run the tests**

Run: `PYTHONPATH= uv run python -m pytest .github/scripts/tests -q`
Expected: all pass.

- [ ] **Step 5: Confirm the whole suite is unaffected**

Run: `PYTHONPATH= uv run python -m pytest plugins -q`
Expected: all pass, no new skips.

- [ ] **Step 6: Commit**

```bash
git add .github/scripts/model_watch.py .github/scripts/tests/test_model_watch.py
git commit -m "feat(ci): upstream model watch script"
```

---

### Task 4: Workflow, labels, and documentation

**Files:**
- Create: `.github/workflows/model-watch.yml`
- Modify: `plugins/model_selector/DESIGN.md`

**Interfaces:**
- Consumes: `.github/scripts/model_watch.py` `main()` from Task 3.
- Produces: nothing programmatic.

- [ ] **Step 1: Write the workflow**

Create `.github/workflows/model-watch.yml`:

```yaml
name: Upstream model watch

# Daily digest feeding the maintainer's test-drive queue. Schedule drift is
# accepted: dedup is keyed on commit SHA, not a time window, so a skipped run
# loses nothing — the next run reports the same candidates.
on:
  schedule:
    - cron: '0 6 * * *'
  workflow_dispatch:
    inputs:
      dry_run:
        description: 'Print what would be filed without creating issues'
        type: boolean
        default: false

permissions:
  issues: write
  contents: read

jobs:
  watch:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - uses: actions/setup-python@v5
        with:
          python-version: '3.12'

      - run: pip install requests

      - name: Check upstream for new models
        env:
          GH_TOKEN: ${{ secrets.GITHUB_TOKEN }}
          GITHUB_REPOSITORY: ${{ github.repository }}
        run: |
          python .github/scripts/model_watch.py \
            ${{ inputs.dry_run && '--dry-run' || '' }}
```

- [ ] **Step 2: Verify the YAML parses**

Run: `PYTHONPATH= uv run python -c "import yaml,sys; yaml.safe_load(open('.github/workflows/model-watch.yml')); print('workflow YAML OK')"`
Expected: `workflow YAML OK`. If PyYAML is unavailable, run `pip install pyyaml` inside `uv run` or use `python3 -c` with the system interpreter.

- [ ] **Step 3: Document the revert policy and the watch job**

In `plugins/model_selector/DESIGN.md`, add to the "Compatibility gating" section:

```markdown
### Upstream reverts

A revert upstream is **not** a verdict on whether a model drives. comma reverts
for many reasons — a metric regression, infrastructure, a competing model
winning — and none of them is a road test on this fork. Reverted models stay
eligible: `update-registry` marks them `upstream_reverted: "<sha>"` and keeps
them, and a catalog entry may carry the same field as provenance. Only the
revert commit itself is never treated as a model.

### Upstream watch (CI)

`.github/workflows/model-watch.yml` runs `.github/scripts/model_watch.py` daily.
It reads the commits API for `selfdrive/modeld/models`, reconciles against
`compatible_models.json` and against existing GitHub issues, and files a
`model-candidate` issue per new model (and `model-revert` for a revert of
something already reported or catalogued). Dedup state is the issues themselves,
keyed by an `upstream-commit: <sha>` line in each body; closing an issue means
handled, so it is never re-reported. The job reports only — it cannot make a
model installable.
```

- [ ] **Step 4: Commit**

```bash
git add .github/workflows/model-watch.yml plugins/model_selector/DESIGN.md
git commit -m "feat(ci): daily upstream model watch workflow"
```

- [ ] **Step 5: Hand the first run to the user**

Do NOT push or trigger the workflow. Report these as the user's next steps:

```bash
git push origin dev
gh label create model-candidate --description "Upstream model awaiting a test drive" --color 0e8a16
gh label create model-revert --description "Upstream withdrew a model" --color d93f0b
gh workflow run "Upstream model watch" -f dry_run=true    # dry run first
gh run watch
```

The dry run proves the parsing and dedup without filing anything. A second run
with `dry_run=false` files the real issues and is what confirms the notification
email actually arrives — the one claim in the spec that cannot be verified any
other way.
