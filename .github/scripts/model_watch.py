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
# _github_headers is shared, not re-implemented: one place decides how these
# calls authenticate, and CI must authenticate or it hits the 60/hr per-IP cap.
from model_download import _github_headers, scan_upstream_models  # noqa: E402

MARKER = 'upstream-commit:'
WATCH_REPO = 'commaai/openpilot'
MODEL_PATH = 'selfdrive/modeld/models'
MAINTAINER = '@OxygenLiu'

# The ONNX files THIS fork actually loads, per model type. The watcher follows
# these files rather than the directory: a commit that touches the directory
# without touching one of them is a refactor, a path move, or a model built for
# an interface this fork cannot load — upstream has since replaced
# driving_vision/driving_policy with a single driving_supercombo.onnx. Watching
# the directory reported all of those as installable candidates.
# Kept in step with ModelSwapper.MODEL_CONFIGS by a test.
WATCH_FILES = {
    'driving': ['driving_vision.onnx', 'driving_policy.onnx'],
    'dm': ['dmonitoring_model.onnx'],
}


def fetch_commits(per_page: int = 100) -> list:
    """Commits touching an ONNX file this fork loads, deduped by sha.

    Each returned commit carries `_watch_type` — the model type inferred from
    the file that changed, which is structural evidence and beats guessing the
    type from the commit message.

    100 is GitHub's max per_page — same single request per file as a smaller
    page, no extra cost — and it widens the window this job looks back through.
    That matters for reverts specifically: plan_issues only reconciles commits
    inside this window, so a revert of a model old enough to have scrolled out
    of it is never reported at all, not even late.
    """
    seen = {}
    for model_type, filenames in WATCH_FILES.items():
        for filename in filenames:
            resp = requests.get(
                f"https://api.github.com/repos/{WATCH_REPO}/commits",
                params={'path': f"{MODEL_PATH}/{filename}", 'per_page': per_page},
                headers=_github_headers(), timeout=30,
            )
            resp.raise_for_status()
            for commit in resp.json():
                # First file to claim a sha wins; the two driving files agree.
                seen.setdefault(commit['sha'], dict(commit, _watch_type=model_type))
    return list(seen.values())


# A model this fork can install is one whose ONNX was added or modified.
# A removal (upstream's move to a single supercombo model) or a rename (a
# directory move) still shows up in a path-filtered commit query but changes
# no model this fork could load.
_MODEL_FILE_STATUSES = ('added', 'modified')


def fetch_commit_files(sha: str) -> list:
    """(filename, status) for every file in a commit. One API call per commit."""
    resp = requests.get(
        f"https://api.github.com/repos/{WATCH_REPO}/commits/{sha}",
        headers=_github_headers(), timeout=30,
    )
    resp.raise_for_status()
    return [(f['filename'].split('/')[-1], f['status'])
            for f in resp.json().get('files', [])]


def filter_by_file_status(candidates: list) -> list:
    """Keep only candidates that add or modify an ONNX file of their own type.

    Costs one API call per candidate, which is why it runs last — after the
    path query and the catalog/issue reconciliation have already narrowed the
    set.
    """
    kept = []
    for cand in candidates:
        watched = set(WATCH_FILES.get(cand['type'], ()))
        changed = fetch_commit_files(cand['commit'])
        if any(name in watched and status in _MODEL_FILE_STATUSES
               for name, status in changed):
            kept.append(cand)
    return kept


def drop_non_model_candidates(planned: list, candidates: list) -> list:
    """Drop planned candidate issues whose commit changed no model this fork loads.

    Runs on the planned list, not the whole window: the status check costs an
    API call per commit, and on a normal day nothing is planned at all. Revert
    issues pass through untouched — a revert is news about a model already
    reported, and its own model change was vetted when it was first filed.
    """
    by_sha = {c['commit']: c for c in candidates}
    keep = []
    for item in planned:
        if item['kind'] != 'candidate':
            keep.append(item)
            continue
        cand = by_sha.get(item['sha'])
        if cand and filter_by_file_status([cand]):
            keep.append(item)
    return keep


def apply_watch_types(scan: dict, type_by_sha: dict) -> None:
    """Set each candidate's type/files from the file that actually changed.

    scan_upstream_models infers type from the commit message ("DM:" in the
    title, "dmonitoring" in the body), which misfiled commits like
    "dmonitoringmodeld: clean up data structures". The file that changed is
    evidence; the message is a guess.
    """
    for cand in scan['candidates']:
        model_type = type_by_sha.get(cand['commit'])
        if model_type:
            cand['type'] = model_type
            cand['files'] = list(WATCH_FILES[model_type])


WATCH_LABELS = ('model-candidate', 'model-revert')


def _fetch_issues_for_label(repo: str, label: str) -> list:
    """All issues carrying `label`, open and closed, across pages.

    GitHub's issues-list `labels` param is AND semantics (an issue must carry
    every listed label to match) — passing both watch labels at once would
    only ever match an issue carrying BOTH, which none of ours ever do. So we
    query once per label and the caller merges by issue number.
    """
    issues, page = [], 1
    while True:
        resp = requests.get(
            f"https://api.github.com/repos/{repo}/issues",
            params={'state': 'all', 'labels': label,
                    'per_page': 100, 'page': page},
            headers=_github_headers(), timeout=30,
        )
        resp.raise_for_status()
        batch = resp.json()
        if not batch:
            return issues
        issues.extend(batch)
        page += 1


def fetch_issues(repo: str) -> list:
    """All watch issues (either label), open and closed, deduped by number."""
    by_number = {}
    for label in WATCH_LABELS:
        for issue in _fetch_issues_for_label(repo, label):
            by_number[issue['number']] = issue
    return list(by_number.values())


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


def _catalog_commits(cat: dict) -> set:
    """Commit shas already catalogued.

    Entry ids are keyed on a PR number for some models and a short sha for
    others, so id matching alone re-filed a model that was already catalogued.
    The commit sha is the only unambiguous key. Shipped entries carry no commit
    and are matched by id instead.
    """
    return {e['commit'] for entries in cat.values() for e in entries if e.get('commit')}


def _candidate_body(cand: dict) -> str:
    entry = {
        'id': cand['id'], 'name': cand['name'], 'date': cand['date'],
        'commit': cand['commit'], 'pr': cand['pr'].strip('()'),
        'files': cand['files'],
    }
    revert_note = ''
    revert_marker = ''
    if cand['upstream_reverted']:
        revert_note = (
            f"\n> **Reverted upstream** by `{cand['upstream_reverted'][:12]}`. "
            "That is not a verdict on whether it drives — comma reverts for many "
            "reasons, none of them a road test on this car. Still worth a drive.\n"
        )
        # A second marker line for the revert sha itself: this candidate issue
        # is the ONLY issue this revert will ever get (a model reported and
        # reverted between two runs is one issue, not a candidate issue plus a
        # revert issue — see module docstring). reported_shas() collects every
        # marker line in a body, so recording the revert sha here means the
        # next run's dedup set already contains it and plan_issues' second
        # branch (news-worthy revert of an already-known model) never fires
        # for news that was already delivered in this very body.
        revert_marker = f"{MARKER} {cand['upstream_reverted']}\n"
    return f"""{MARKER} {cand['commit']}
{revert_marker}
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
    catalogued_commits = _catalog_commits(cat)

    for cand in scan['candidates']:
        sha = cand['commit']
        revert_sha = cand.get('upstream_reverted')

        if (sha not in reported and cand['id'] not in catalogued
                and sha not in catalogued_commits):
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
            in_catalog = cand['id'] in catalogued or sha in catalogued_commits
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
        headers=_github_headers(), timeout=30,
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

    commits = fetch_commits()
    scan = scan_upstream_models(commits)
    # File evidence beats the message heuristic for model type.
    apply_watch_types(scan, {c['sha']: c['_watch_type'] for c in commits})
    cat = catalog.load_catalog()
    if not cat:
        # load_catalog() fails CLOSED (returns {}) on any read/parse problem —
        # correct for the UI thread (offer nothing), wrong here. An empty
        # catalog is not a real state for this repo: compatible_models.json
        # always ships with at least the release-default entries. {} here
        # means the file is missing or corrupt, and every catalogued model
        # would look brand-new to _catalog_ids — silently spamming a fresh
        # candidate issue for each one. Fail loudly instead of filing.
        print("error: catalog.load_catalog() returned empty — file missing or "
              "corrupt; refusing to file issues against a blank catalog",
              file=sys.stderr)
        return 1
    # A dry run needs the real reported set too, or it prints a dishonest plan.
    reported = reported_shas(fetch_issues(args.repo))

    planned = plan_issues(scan, cat, reported)
    planned = drop_non_model_candidates(planned, scan['candidates'])
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
