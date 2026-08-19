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


def fetch_commits(per_page: int = 30) -> list:
    resp = requests.get(
        f"https://api.github.com/repos/{WATCH_REPO}/commits",
        params={'path': MODEL_PATH, 'per_page': per_page},
        headers=_github_headers(), timeout=30,
    )
    resp.raise_for_status()
    return resp.json()


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

    scan = scan_upstream_models(fetch_commits())
    cat = catalog.load_catalog()
    # A dry run needs the real reported set too, or it prints a dishonest plan.
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
