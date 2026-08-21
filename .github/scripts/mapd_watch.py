#!/usr/bin/env python3
"""Daily watch for new pfeiferj/mapd releases — files a GitHub Issue per release.

Reports only. Nothing here bumps a pin, edits a slot schema, restores mapd's
`processes` entry or touches a device: a release becomes usable when a human
diffs the schema, bumps `MAX_ALLOWED_VERSION` and drives the car, and never
otherwise.

Releases, not tags and not commits: a release is what
`mapd_manager.download_binary()` actually pulls, so a tag without one is not
installable and is not news. Drafts and prereleases are skipped for the same
reason — neither is something we would pin to.

Dedup state lives in the issues themselves. Each issue body carries a
`mapd-release: <tag>` marker; any tag found in an open OR closed issue is never
reported again, because closing an issue is how the maintainer says "handled" —
whether the pin moved or the release turned out to be irrelevant.
"""
import argparse
import os
import re
import sys
from base64 import b64decode
from pathlib import Path

import requests

REPO_ROOT = Path(__file__).resolve().parents[2]
# mapd_manager does `from config import ...`, so plugins/ has to be importable
# alongside plugins/mapd/ itself.
sys.path.insert(0, str(REPO_ROOT / 'plugins'))
sys.path.insert(0, str(REPO_ROOT / 'plugins' / 'mapd'))
sys.path.insert(0, str(REPO_ROOT / 'plugins' / 'model_selector'))

# The pin and the ordering rule are imported, never re-implemented: one place
# decides how mapd versions compare, and importing means this watch silences
# itself the moment the pin is bumped — the newly pinned release simply stops
# being newer than the pin, with no edit here.
from mapd_manager import MAX_ALLOWED_VERSION, _version_tuple  # noqa: E402
# _github_headers is shared with model_watch for the same reason: one place
# decides how these calls authenticate, and CI must authenticate or it hits the
# 60/hr per-IP cap.
from model_download import _github_headers  # noqa: E402

MARKER = 'mapd-release:'
WATCH_REPO = 'pfeiferj/mapd'
LABEL = 'mapd-release'
MAINTAINER = '@OxygenLiu'

# TEMPORARY, despite the constant's clothes. mapd v2.3.0 ships gomsgq v0.1.10,
# whose ungated `panic("Invalid Msgq message size")` kills the binary on a
# shadow reader's expected torn read; upstream fixed it in this commit
# (PR #133, gomsgq v0.1.11), which is on main but in no release. Set this to ''
# once mapd is re-activated and the whole section drops out of the issue body.
REQUIRED_COMMIT = 'fe45d10'

# mapd's copy of the schema, and ours. We consume THREE upstream structs, one
# per cereal slot, and a field added to any of them drops just as silently as
# one added to MapdOut — so all three are watched. Checking only MapdOut is
# worse than checking nothing: it launders two unchecked structs as a clean
# verdict, which is exactly how v2.3.1's two new `MapdExtendedOut` fields got
# reported as "schema-safe. No new fields."
UPSTREAM_CAPNP_PATH = 'cereal/custom/custom.capnp'
_CEREAL_DIR = REPO_ROOT / 'plugins' / 'mapd' / 'cereal'
WATCHED_STRUCTS = {
    'MapdExtendedOut': _CEREAL_DIR / 'slot17.capnp',
    'MapdIn': _CEREAL_DIR / 'slot18.capnp',
    'MapdOut': _CEREAL_DIR / 'slot19.capnp',
}
# The enums every watched struct's fields resolve against, ours in one file.
STANDALONE_CAPNP = _CEREAL_DIR / 'standalone.capnp'


class SchemaError(Exception):
    """The schema could not be fetched or parsed — issue is filed regardless."""


def fetch_releases(per_page: int = 100) -> list:
    # 100 is GitHub's max per_page — same single request as the default — and it
    # means a repo that cut several releases while this job was disabled is
    # still reported in full on the next run.
    resp = requests.get(
        f"https://api.github.com/repos/{WATCH_REPO}/releases",
        params={'per_page': per_page},
        headers=_github_headers(), timeout=30,
    )
    resp.raise_for_status()
    return resp.json()


def fetch_issues(repo: str) -> list:
    """All mapd-release issues, open and closed, across pages."""
    issues, page = [], 1
    while True:
        resp = requests.get(
            f"https://api.github.com/repos/{repo}/issues",
            params={'state': 'all', 'labels': LABEL,
                    'per_page': 100, 'page': page},
            headers=_github_headers(), timeout=30,
        )
        resp.raise_for_status()
        batch = resp.json()
        if not batch:
            return issues
        issues.extend(batch)
        page += 1


def reported_tags(issues: list) -> set:
    tags = set()
    for issue in issues:
        for line in (issue.get('body') or '').splitlines():
            line = line.strip()
            if line.startswith(MARKER):
                tags.add(line[len(MARKER):].strip())
    return tags


# ── capnp text parsing ────────────────────────────────────────────────────────
# Text, not the capnp compiler: CI must not need `capnp` installed, and the
# question here is only "which field names and ordinals exist", which the source
# answers directly.

_COMMENT_RE = re.compile(r'#.*$', re.M)
_BLOCK_HEAD_RE = re.compile(r'^\s*(struct|enum)\s+(\w+)', re.M)
_FIELD_RE = re.compile(r'^\s*([A-Za-z]\w*)\s*@(\d+)\s*:\s*([^;]+);', re.M)
_ENUMERANT_RE = re.compile(r'^\s*([A-Za-z]\w*)\s*@(\d+)\s*;', re.M)

_BUILTIN_TYPES = {
    'Void', 'Bool', 'Text', 'Data', 'AnyPointer',
    'Int8', 'Int16', 'Int32', 'Int64',
    'UInt8', 'UInt16', 'UInt32', 'UInt64',
    'Float32', 'Float64',
}


def _strip_comments(text: str) -> str:
    return _COMMENT_RE.sub('', text)


def _extract_block(text: str, kind: str, name: str):
    """Body of `<kind> <name> [@0x...] { ... }`, or None if absent.

    Brace-matched rather than regexed to the first `}` so a nested block (a
    group or a union) does not truncate the body.
    """
    head = re.search(rf'^\s*{kind}\s+{re.escape(name)}\b[^{{]*{{', text, re.M)
    if not head:
        return None
    depth, start = 1, head.end()
    for i in range(start, len(text)):
        if text[i] == '{':
            depth += 1
        elif text[i] == '}':
            depth -= 1
            if depth == 0:
                return text[start:i]
    raise SchemaError(f"unterminated {kind} {name}")


def parse_fields(text: str, struct: str) -> dict:
    """`{name: {'ordinal': int, 'type': str}}` for a struct's declared fields.

    Handles both shapes this watch has to read: mapd's `custom.capnp` wraps the
    fields in `struct MapdOut @0x... { ... }`, while each of our slot files is a
    bare fragment of field lines that `custom_capnp.py` splices into the real
    struct at install time — there is no enclosing block to find.
    """
    text = _strip_comments(text)
    block = _extract_block(text, 'struct', struct)
    if block is None:
        # Only a file with no blocks at all can be the bare fragment. A file
        # that declares blocks but not this one is a rename or a deletion, and
        # scraping every field line out of it would fake a plausible answer.
        if _BLOCK_HEAD_RE.search(text):
            raise SchemaError(f"struct {struct} not found")
        block = text
    return {m.group(1): {'ordinal': int(m.group(2)), 'type': m.group(3).strip()}
            for m in _FIELD_RE.finditer(block)}


def parse_enum(text: str, name: str):
    """`{member: ordinal}` for one enum, or None if the enum is not declared."""
    block = _extract_block(_strip_comments(text), 'enum', name)
    if block is None:
        return None
    return {m.group(1): int(m.group(2)) for m in _ENUMERANT_RE.finditer(block)}


def referenced_enums(fields: dict, text: str) -> list:
    """Enum type names used by `fields` and declared in `text`, in field order.

    Derived rather than hardcoded so an upstream field carrying a brand-new
    enum type gets its members diffed too, not just its own field line.
    """
    names, seen = [], set()
    for meta in fields.values():
        type_name = meta['type'].strip()
        if type_name in _BUILTIN_TYPES or type_name in seen:
            continue
        seen.add(type_name)
        if parse_enum(text, type_name) is not None:
            names.append(type_name)
    return names


def fetch_upstream_capnp(tag: str) -> str:
    resp = requests.get(
        f"https://api.github.com/repos/{WATCH_REPO}/contents/{UPSTREAM_CAPNP_PATH}",
        params={'ref': tag}, headers=_github_headers(), timeout=30,
    )
    resp.raise_for_status()
    return b64decode(resp.json()['content']).decode('utf-8')


def read_local_slots() -> dict:
    """`{struct: slot file text}` for every watched struct."""
    return {struct: path.read_text() for struct, path in WATCHED_STRUCTS.items()}


def slot_name(struct: str) -> str:
    """Repo-relative path of the slot file a struct's fields must land in."""
    return str(WATCHED_STRUCTS[struct].relative_to(REPO_ROOT))


def diff_schema(theirs: str, ours_by_struct: dict, ours_enums_text: str) -> dict:
    """Compare every watched upstream struct against its slot file, plus enums.

    capnp is additive, so a newer binary publishing into an older slot silently
    DROPS every field we have not declared — `highwayClass` would read back as
    `unknown` and speedlimitd would mis-classify every road, with no error
    anywhere. A new enumerant is that same silent failure in a different shape:
    an out-of-range value arriving in a field we believe we understand.

    Every watched struct is diffed and reported separately, and the single
    `identical` verdict is true only when every struct AND every enum is clean —
    a per-struct check that rolled up optimistically would be the bug it exists
    to prevent.
    """
    structs, theirs_by_struct = {}, {}
    for struct in WATCHED_STRUCTS:
        theirs_fields = parse_fields(theirs, struct)
        ours_fields = parse_fields(ours_by_struct[struct], struct)
        if not theirs_fields or not ours_fields:
            raise SchemaError(
                f"{struct}: parsed {len(theirs_fields)} upstream / "
                f"{len(ours_fields)} local fields — a side with none means the "
                f"parse failed, not that the schema is empty")
        theirs_by_struct[struct] = theirs_fields

        added = [(n, m['ordinal'], m['type']) for n, m in theirs_fields.items()
                 if n not in ours_fields]
        removed = [(n, m['ordinal'], m['type']) for n, m in ours_fields.items()
                   if n not in theirs_fields]
        changed = [(n, ours_fields[n], m) for n, m in theirs_fields.items()
                   if n in ours_fields and (
                       ours_fields[n]['ordinal'] != m['ordinal'] or
                       ours_fields[n]['type'] != m['type'])]
        structs[struct] = {
            'name': struct, 'slot': slot_name(struct),
            'theirs_field_count': len(theirs_fields),
            'ours_field_count': len(ours_fields),
            'added': added, 'removed': removed, 'changed': changed,
            'identical': not (added or removed or changed),
        }

    # The union across every watched struct, not just MapdOut's: an enum that
    # reaches us through `MapdIn` gains enumerants the same way and matters the
    # same amount.
    enum_names = []
    for fields in theirs_by_struct.values():
        for name in referenced_enums(fields, theirs):
            if name not in enum_names:
                enum_names.append(name)

    enums = []
    for enum_name in enum_names:
        theirs_members = parse_enum(theirs, enum_name)
        ours_members = parse_enum(ours_enums_text, enum_name)
        if ours_members is None:
            enums.append({'name': enum_name, 'missing': True, 'added': [],
                          'changed': []})
            continue
        enums.append({
            'name': enum_name, 'missing': False,
            'added': [(n, o) for n, o in theirs_members.items()
                      if n not in ours_members],
            'changed': [(n, ours_members[n], o) for n, o in theirs_members.items()
                        if n in ours_members and ours_members[n] != o],
        })

    dirty_enums = [e for e in enums if e['missing'] or e['added'] or e['changed']]
    return {
        'structs': structs,
        'enums': enums,
        'identical': all(s['identical'] for s in structs.values()) and not dirty_enums,
    }


def schema_section(tag: str, fetch=fetch_upstream_capnp) -> str:
    """The issue's schema verdict, or a stated non-answer.

    A failed schema check must not cost the notification: a release
    notification is worth more than a perfect issue body, and the missing check
    is stated rather than implied.
    """
    try:
        diff = diff_schema(fetch(tag), read_local_slots(),
                           STANDALONE_CAPNP.read_text())
    except Exception as exc:  # noqa: BLE001 — any failure degrades the same way
        slots = ', '.join(f'`{slot_name(s)}`' for s in WATCHED_STRUCTS)
        return ("### Schema\n\n"
                f"**Could not be checked** (`{exc}`) — diff "
                f"`{UPSTREAM_CAPNP_PATH}` at `{tag}` against "
                f"{slots} by hand before bumping.\n")

    # Per-struct counts, always rendered: a differ that parsed nothing would
    # otherwise be free to report "identical" vacuously.
    counts = "\n".join(
        f"- `{s['name']}` upstream: {s['theirs_field_count']} fields; "
        f"`{s['slot']}`: {s['ours_field_count']} fields"
        for s in diff['structs'].values())
    if diff['identical']:
        return ("### Schema\n\n"
                "**Identical — the pin bump is schema-safe.** No new fields in "
                "any watched struct and no new enumerants.\n\n"
                f"{counts}\n")

    lines = ["### Schema\n",
             "**Must land in our slots before the pin moves.** capnp is "
             "additive: a newer binary publishing into an older slot silently "
             "drops every field we have not declared.\n"]
    for s in diff['structs'].values():
        if s['identical']:
            lines.append(f"- `{s['name']}` → `{s['slot']}`: identical.")
            continue
        lines.append(f"\n**`{s['name']}` → `{s['slot']}` differs:**\n")
        if s['added']:
            lines.append(f"New fields in `{s['name']}` "
                         f"(add to `{s['slot']}`):\n")
            lines += [f"- `{n} @{o} :{t};`" for n, o, t in s['added']]
            lines.append("")
        if s['removed']:
            lines.append("Fields we declare that upstream no longer has "
                         "(harmless to read, but the pin bump is not a pure "
                         "add):\n")
            lines += [f"- `{n} @{o} :{t};`" for n, o, t in s['removed']]
            lines.append("")
        if s['changed']:
            lines.append("**Fields whose ordinal or type changed — a wire "
                         "break, not an addition:**\n")
            lines += [f"- `{n}`: ours `@{ours['ordinal']} :{ours['type']}` vs "
                      f"upstream `@{theirs['ordinal']} :{theirs['type']}`"
                      for n, ours, theirs in s['changed']]
            lines.append("")
    lines.append("")
    for enum in diff['enums']:
        if enum['missing']:
            lines.append(f"- enum `{enum['name']}` is not declared in "
                         "`plugins/mapd/cereal/standalone.capnp` at all\n")
            continue
        for name, ordinal in enum['added']:
            lines.append(f"- new enumerant `{enum['name']}.{name} @{ordinal}` "
                         "(an out-of-range value arriving in a field we believe "
                         "we understand)")
        for name, ours, theirs in enum['changed']:
            lines.append(f"- enumerant `{enum['name']}.{name}` renumbered: "
                         f"ours `@{ours}` vs upstream `@{theirs}`")
    lines.append(f"\nField counts:\n\n{counts}\n")
    return "\n".join(lines)


def fetch_compare_status(tag: str) -> str:
    """`ahead` / `identical` / `behind` / `diverged` for REQUIRED_COMMIT vs tag."""
    resp = requests.get(
        f"https://api.github.com/repos/{WATCH_REPO}/compare/{REQUIRED_COMMIT}...{tag}",
        headers=_github_headers(), timeout=30,
    )
    resp.raise_for_status()
    return resp.json()['status']


def commit_section(tag: str, status: str) -> str:
    """The crash-fix verdict. Empty string once REQUIRED_COMMIT is cleared."""
    if not REQUIRED_COMMIT:
        return ''
    if status in ('ahead', 'identical'):
        verdict = (f"**Contains `{REQUIRED_COMMIT}` (`{status}`) — this release "
                   "carries the gomsgq v0.1.11 torn-read fix.**")
    elif status in ('behind', 'diverged'):
        verdict = (f"**Does NOT contain `{REQUIRED_COMMIT}` (`{status}`) — mapd "
                   "would still flap on the shadow-reader panic. Re-activation "
                   "stays blocked; the pin can still move if the schema section "
                   "says so.**")
    else:
        verdict = (f"Unrecognised compare status `{status}` — check "
                   f"https://github.com/{WATCH_REPO}/compare/{REQUIRED_COMMIT}...{tag} "
                   "by hand.")
    return f"""### Crash fix (`{REQUIRED_COMMIT}`, PR #133, gomsgq v0.1.11)

{verdict}

https://github.com/{WATCH_REPO}/compare/{REQUIRED_COMMIT}...{tag}
"""


def _issue_body(release: dict, schema: str, commit: str) -> str:
    tag = release['tag_name']
    has_asset = any(a.get('name') == 'mapd' for a in release.get('assets') or [])
    asset = '`mapd` present' if has_asset else '**no `mapd` asset** — not installable'
    return f"""{MARKER} {tag}

{MAINTAINER} — new mapd release upstream. We are pinned to `{MAX_ALLOWED_VERSION}`.

| | |
|---|---|
| Tag | {tag} |
| Published | {release.get('published_at') or 'unknown'} |
| Release | {release.get('html_url') or f'https://github.com/{WATCH_REPO}/releases/tag/{tag}'} |
| Asset | {asset} |

{commit}
{schema}
### Re-activation checklist

Only if both sections above say yes. This issue reports; it changes nothing.

- [ ] restore the process entry in `plugins/mapd/plugin.json`:
      `"processes": [{{"name": "mapd", "module": "mapd_runner", "condition": "always_run"}}]`
- [ ] bump `MAX_ALLOWED_VERSION` in `plugins/mapd/mapd_manager.py` to `{tag}`
- [ ] keep **every** `subscriber` entry in `mapd_defaults.json` on `shadow: true`
      — v0.1.11 still has no deregistration, so slotted readers still leak a slot
      per death (`selfdriveState` was measured 15/15 full)
- [ ] success criterion: one long-lived mapd PID across a whole drive, versus
      the 1–2 min flap baseline

Close this issue once handled — bumped or dismissed. Either way this tag will
not be reported again.
"""


def plan_releases(releases: list, reported: set) -> list:
    """Releases newer than the pin that no issue already carries, oldest first.

    Drafts and prereleases are not things we would pin to, so they are not news.
    An unparseable tag degrades to "not newer than the pin" inside
    `_version_tuple` and is silently skipped — the same behaviour the daemon
    has, decided in the same place.
    """
    pin = _version_tuple(MAX_ALLOWED_VERSION)
    newer = [r for r in releases
             if not r.get('draft') and not r.get('prerelease')
             and _version_tuple(r['tag_name']) > pin]
    newer.sort(key=lambda r: _version_tuple(r['tag_name']))
    return [r for r in newer if r['tag_name'] not in reported]


def build_issue(release: dict) -> dict:
    tag = release['tag_name']
    commit = commit_section(tag, fetch_compare_status(tag)) if REQUIRED_COMMIT else ''
    return {
        'tag': tag,
        'title': f"mapd release: {tag}",
        'labels': [LABEL],
        'body': _issue_body(release, schema_section(tag), commit),
    }


def ensure_label(repo: str) -> None:
    """Create `mapd-release` if absent; an existing label is success.

    model-watch relies on implicit creation at file time. Doing it explicitly
    removes a failure mode that would otherwise surface only on the one run
    that matters.
    """
    resp = requests.post(
        f"https://api.github.com/repos/{repo}/labels",
        headers=_github_headers(), timeout=30,
        json={'name': LABEL, 'color': 'c2e0c6',
              'description': 'New upstream pfeiferj/mapd release'},
    )
    if resp.status_code == 422:  # already_exists
        return
    resp.raise_for_status()


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

    releases = fetch_releases()
    # A dry run needs the real reported set too, or it prints a dishonest plan.
    reported = reported_tags(fetch_issues(args.repo))
    to_file = plan_releases(releases, reported)

    print(f"{len(releases)} releases upstream, pinned to {MAX_ALLOWED_VERSION}, "
          f"{len(reported)} already reported, {len(to_file)} to file")

    if to_file and not args.dry_run:
        ensure_label(args.repo)

    for release in to_file:
        item = build_issue(release)
        if args.dry_run:
            print(f"  [dry-run] would file: {item['title']}")
            continue
        number = create_issue(args.repo, item)
        print(f"  filed #{number}: {item['title']}")

    return 0


if __name__ == '__main__':
    sys.exit(main())
