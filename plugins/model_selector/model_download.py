#!/usr/bin/env python3
"""
Download openpilot models from GitHub at specific commits
Handles two separate model types:
- Driving Models: driving_vision.onnx + driving_policy.onnx → /data/models/driving/
- Driver Monitoring (DM) Models: dmonitoring_model.onnx → /data/models/dm/

Model registry: /data/models/model_registry.json
"""
import argparse
import json
import os
import re
import sys
import requests
from pathlib import Path
from datetime import datetime
from enum import Enum


try:
    from plugins.model_selector import catalog
except ImportError:
    import catalog

# Root of model storage. Module-level so tests can redirect it.
BASE_DATA_DIR = Path('/data') if Path('/data').exists() else Path.home() / 'driving_data'


class ModelType(Enum):
    """Model type enumeration"""
    DRIVING = "driving"
    DM = "dm"


# Model registry location (persists across reboots on C3)
REGISTRY_FILE = Path('/data/models/model_registry.json')


def load_registry():
    """Load model registry from JSON file"""
    if not REGISTRY_FILE.exists():
        print(f"⚠️  Model registry not found: {REGISTRY_FILE}")
        return {}, {}

    with open(REGISTRY_FILE) as f:
        registry = json.load(f)

    return registry.get('driving_models', {}), registry.get('dm_models', {})


# openpilot LFS servers — try GitHub first (older models), fall back to GitLab
_LFS_BATCH_URLS = [
    "https://github.com/commaai/openpilot.git/info/lfs/objects/batch",
    "https://gitlab.com/commaai/openpilot-lfs.git/info/lfs/objects/batch",
]


def _resolve_lfs_url(oid: str, size: int) -> str:
    """Resolve a Git LFS object to its actual download URL, trying each known LFS server."""
    payload = {
        "operation": "download",
        "objects": [{"oid": oid, "size": size}],
    }
    headers = {
        "Content-Type": "application/vnd.git-lfs+json",
        "Accept": "application/vnd.git-lfs+json",
    }
    last_error = None
    for url in _LFS_BATCH_URLS:
        try:
            resp = requests.post(url, json=payload, headers=headers, timeout=30)
            resp.raise_for_status()
            obj = resp.json()["objects"][0]
            if "error" in obj:
                last_error = f"LFS error: {obj['error'].get('message', obj['error'])}"
                continue
            return obj["actions"]["download"]["href"]
        except Exception as e:
            last_error = str(e)
    raise Exception(last_error or "LFS object not found on any server")


def download_file(url: str, dest: Path, desc: str = None):
    """Download file from URL, resolving Git LFS pointers via the batch API."""
    print(f"  Downloading {desc or dest.name}...")

    # Download the file (may be LFS pointer or regular file)
    response = requests.get(url, timeout=30)
    response.raise_for_status()
    content = response.content

    # Check if this is a Git LFS pointer file
    if len(content) < 300:
        try:
            lfs_pointer = content.decode('utf-8')
            if lfs_pointer.startswith('version https://git-lfs.github.com'):
                # Parse LFS pointer for oid and size
                lfs_oid = None
                lfs_size = None
                for line in lfs_pointer.strip().split('\n'):
                    if line.startswith('oid sha256:'):
                        lfs_oid = line.split(':', 1)[1].strip()
                    elif line.startswith('size '):
                        lfs_size = int(line.split(' ', 1)[1].strip())

                if not lfs_oid or not lfs_size:
                    raise Exception("Failed to parse LFS pointer")

                print(f"    LFS file detected ({lfs_size / 1024 / 1024:.1f}MB), resolving...")
                real_url = _resolve_lfs_url(lfs_oid, lfs_size)

                # Download the actual file
                real_resp = requests.get(real_url, timeout=600)
                real_resp.raise_for_status()
                content = real_resp.content

                if len(content) < 300 or content.startswith(b'version https://git-lfs.github.com'):
                    raise Exception("LFS batch API returned pointer instead of file")

        except UnicodeDecodeError:
            pass

    with open(dest, 'wb') as f:
        f.write(content)

    file_size_mb = dest.stat().st_size / 1024 / 1024
    print(f"    Done: {dest.name} ({file_size_mb:.1f}MB)")


def check_model_compatibility(model_info: dict, model_type: ModelType) -> tuple[bool, str]:
    """Check if model is compatible with current openpilot version

    Returns:
        (is_compatible, warning_message)
    """
    # Only check driving models (DM models don't use desire_pulse)
    if model_type != ModelType.DRIVING:
        return True, ""

    # Parse model date
    try:
        from datetime import datetime
        model_date = datetime.strptime(model_info['date'], '%Y-%m-%d')
        # desire_pulse transition date: August 27, 2025
        transition_date = datetime(2025, 8, 27)

        if model_date < transition_date:
            warning = (
                f"\n⚠️  COMPATIBILITY WARNING ⚠️\n"
                f"This model was released BEFORE the desire_pulse transition (Aug 27, 2025).\n"
                f"Model date: {model_info['date']}\n"
                f"\n"
                f"Your current openpilot code uses 'desire_pulse' (commit 88e7c48bf).\n"
                f"This model expects 'desire' and will NOT work with your code.\n"
                f"\n"
                f"Compatible models (released after Aug 27, 2025):\n"
                f"  - modeld_desiredesire_pulse_f8ff156\n"
                f"  - firehose_model_f0f04d4\n"
                f"  - nevada_3ca9f35\n"
                f"  - north_nevada_4d08542\n"
                f"  - cool_people_3c957c6 (recommended)\n"
            )
            return False, warning
    except (ValueError, KeyError):
        # If date parsing fails, assume compatible (benefit of doubt)
        pass

    return True, ""


def download_model(model_type: ModelType, model_id: str, output_dir: Path = None,
                   allow_untested: bool = False):
    """Download a model from openpilot master at specific commit"""

    # Load registry (maintainer catalogue; the curated catalog takes priority)
    driving_models, dm_models = load_registry()

    if model_type == ModelType.DRIVING:
        registry = driving_models
        type_name = "Driving Model"
        default_dir_name = "models/driving"
    else:
        registry = dm_models
        type_name = "Driver Monitoring Model"
        default_dir_name = "models/dm"

    entry = next((e for e in catalog.verified_entries(model_type) if e['id'] == model_id), None)

    if entry is None and not (allow_untested or catalog.unlocked()):
        version = catalog.openpilot_version() or 'unknown'
        tested = [e['id'] for e in catalog.verified_entries(model_type)]
        print(f"❌ '{model_id}' is not a tested model for openpilot {version}")
        print(f"   Tested: {', '.join(tested) if tested else 'none'}")
        print("   Maintainers: re-run with --unlocked to install an untested model.")
        return 1

    if entry is not None and entry.get('source') == 'shipped':
        print(f"❌ '{model_id}' ships with the release — it is imported from disk, not downloaded")
        return 1

    model_info = entry if entry is not None else registry.get(model_id)

    if model_info is None:
        print(f"❌ {type_name} '{model_id}' not found in registry")
        print(f"\nAvailable {type_name.lower()}s:")
        for mid, info in registry.items():
            print(f"  {mid}: {info['name']} ({info['commit']})")
        return 1

    # Check compatibility — but only for the registry (maintainer) path.
    # Curation strictly supersedes the date heuristic: an entry that came
    # from the catalog was maintainer-test-driven and put there deliberately,
    # so the date rule (pre-desire_pulse) must not veto it. The registry path
    # has no such verification, so it keeps the heuristic.
    if entry is None:
        is_compatible, warning = check_model_compatibility(model_info, model_type)
        if not is_compatible:
            print(warning)
            if sys.stdin.isatty():
                print("=" * 70)
                response = input("Download anyway? (yes/no): ")
                if response.lower() not in ['yes', 'y']:
                    print("Download cancelled")
                    return 1
                print()
            else:
                print("Skipping incompatible model (non-interactive)")
                return 1

    # Determine output directory
    if output_dir is None:
        output_dir = BASE_DATA_DIR / default_dir_name / model_id

    output_dir.mkdir(parents=True, exist_ok=True)

    description = model_info.get('description') or model_info.get('notes', '')

    print("=" * 70)
    print(f"Downloading: {model_info['name']} ({type_name})")
    print("=" * 70)
    print(f"Commit: {model_info['commit']}")
    print(f"Date: {model_info['date']}")
    print(f"PR: {model_info.get('pr', 'N/A')}")
    print(f"Description: {description}")
    print(f"Output: {output_dir}")
    print()

    # Download ONNX files from GitHub
    base_url = f"https://raw.githubusercontent.com/commaai/openpilot/{model_info['commit']}/selfdrive/modeld/models"

    all_files = model_info['files']

    print(f"ONNX files to download: {len(all_files)}")
    print(f"Type: {type_name}")
    print("(PKL files will be compiled on C3 device)")
    print()

    failed_files = []
    for filename in all_files:
        url = f"{base_url}/{filename}"
        dest = output_dir / filename

        try:
            download_file(url, dest, filename)
        except Exception as e:
            print(f"    ❌ Failed: {e}")
            failed_files.append(filename)

    # Create model_info.json with type information
    info_file = output_dir / 'model_info.json'
    info_data = {
        'name': model_info['name'],
        'version': model_info['commit'],
        'commit': model_info['commit'],
        'date': model_info['date'],
        'pr': model_info.get('pr', ''),
        'description': description,
        'source': 'comma.ai',
        'type': model_type.value,
        'downloaded_date': datetime.now().isoformat(),
    }

    with open(info_file, 'w') as f:
        json.dump(info_data, f, indent=2)

    print()
    print("=" * 70)

    if failed_files:
        print(f"⚠️  Download completed with {len(failed_files)} failures:")
        for f in failed_files:
            print(f"  - {f}")
    else:
        print(f"✅ Download complete!")

    print(f"📍 Location: {output_dir}")
    print(f"📄 Metadata: {info_file}")

    # Calculate total size
    total_size = sum(f.stat().st_size for f in output_dir.iterdir() if f.is_file())
    print(f"💾 Total size: {total_size / 1024 / 1024:.1f}MB")

    print()
    print("Next steps:")
    print(f"  1. Verify: python model_swapper.py --type {model_type.value} verify {model_id}")
    print(f"  2. Swap: python model_swapper.py --type {model_type.value} swap {model_id}")
    print("=" * 70)

    return 0 if not failed_files else 1


def list_available(model_type: ModelType = None):
    """List all available models in registry"""
    # Load registry
    driving_models, dm_models = load_registry()

    print("=" * 70)
    print("Available Models for Download")
    print("=" * 70)
    print()

    if model_type is None or model_type == ModelType.DRIVING:
        print("[DRIVING MODELS]")
        print("For lateral/longitudinal control (driving_vision.onnx + driving_policy.onnx)")
        print()
        for model_id, info in driving_models.items():
            # Check compatibility
            is_compatible, _ = check_model_compatibility(info, ModelType.DRIVING)
            compat_icon = "✅" if is_compatible else "⚠️"
            compat_text = "Compatible" if is_compatible else "INCOMPATIBLE (pre-desire_pulse)"

            print(f"📦 {model_id}  {compat_icon} {compat_text}")
            print(f"   Name: {info['name']}")
            print(f"   Commit: {info['commit']}")
            print(f"   Date: {info['date']}")
            print(f"   PR: {info.get('pr', 'N/A')}")
            print(f"   Description: {info['description']}")
            print(f"   Files: {len(info['files'])}")
            print()

    if model_type is None or model_type == ModelType.DM:
        print("[DRIVER MONITORING MODELS]")
        print("For driver attention detection (dmonitoring_model.onnx)")
        print()
        for model_id, info in dm_models.items():
            print(f"📦 {model_id}")
            print(f"   Name: {info['name']}")
            print(f"   Commit: {info['commit']}")
            print(f"   Date: {info['date']}")
            print(f"   PR: {info.get('pr', 'N/A')}")
            print(f"   Description: {info['description']}")
            print(f"   Files: {len(info['files'])}")
            print()


def check_updates():
    """List tested models not yet installed.

    The catalog is the only source — GitHub is not consulted. Output is JSON for
    the UI to parse.
    """
    result = {'version': catalog.openpilot_version()}
    total = 0
    verified_total = 0

    for type_name in ('driving', 'dm'):
        models_dir = BASE_DATA_DIR / 'models' / type_name
        installed = set()
        if models_dir.exists():
            installed = {d.name for d in models_dir.iterdir()
                         if d.is_dir() and not d.name.startswith('_')}

        verified = catalog.verified_entries(type_name)
        verified_total += len(verified)
        # Shipped entries have no commit/files — download_model always refuses
        # them (they're imported from disk by ModelSwapper.import_stock, not
        # downloaded), so they must never appear in the offer list. They still
        # count toward verified_total: that answers "does this openpilot
        # version have any tested models at all", which shipped entries do
        # satisfy.
        entries = [dict(e, type=type_name) for e in verified
                   if e['id'] not in installed and e.get('source') != 'shipped']
        result[type_name] = entries
        total += len(entries)

    result['total'] = total
    result['verified_total'] = verified_total

    print(json.dumps(result))
    return 0


def add_model_to_registry(model_type: str, model_id: str, name: str, commit: str,
                          date: str, description: str, pr: str = None):
    """Add a new model to the registry"""

    # Load existing registry
    with open(REGISTRY_FILE) as f:
        registry = json.load(f)

    # Determine model type key and files
    if model_type == 'driving':
        registry_key = 'driving_models'
        files = ['driving_vision.onnx', 'driving_policy.onnx']
    else:
        registry_key = 'dm_models'
        files = ['dmonitoring_model.onnx']

    # Create model entry
    model_entry = {
        'name': name,
        'commit': commit,
        'date': date,
        'description': description,
        'files': files
    }

    if pr:
        model_entry['pr'] = pr

    # Add to registry
    registry[registry_key][model_id] = model_entry
    registry['last_updated'] = datetime.now().strftime('%Y-%m-%d')

    # Save registry
    with open(REGISTRY_FILE, 'w') as f:
        json.dump(registry, f, indent=2)

    print(f"✅ Added {model_type} model '{model_id}' to registry")
    print(f"   Name: {name}")
    print(f"   Commit: {commit}")
    print(f"   Date: {date}")
    if pr:
        print(f"   PR: {pr}")
    print()
    print(f"Registry updated: {REGISTRY_FILE}")

    return 0


def add_model_from_pr(pr_number: int, model_type: str = 'driving'):
    """Add a model to registry by extracting info from GitHub PR

    Args:
        pr_number: GitHub PR number (e.g., 36849)
        model_type: 'driving' or 'dm'
    """
    import re

    print(f"🔍 Fetching PR #{pr_number} from GitHub...")

    api_url = f"https://api.github.com/repos/commaai/openpilot/pulls/{pr_number}"
    try:
        response = requests.get(api_url)
        response.raise_for_status()
        pr = response.json()
    except Exception as e:
        print(f"❌ Failed to fetch PR: {e}")
        return 1

    # Extract info
    title = pr['title']
    merge_commit = pr.get('merge_commit_sha')
    merged_at = pr.get('merged_at')
    # LFS objects are stored against the PR head commit, not the merge commit
    head_commit = pr.get('head', {}).get('sha') or merge_commit

    if not merge_commit:
        print(f"❌ PR #{pr_number} has not been merged yet")
        return 1

    merged_date = merged_at[:10] if merged_at else datetime.now().strftime('%Y-%m-%d')

    # Generate model_id: use PR number for dedup (same PR = same model, different commits)
    clean_name = title.lower().replace(' ', '_').replace('-', '_')
    clean_name = re.sub(r'[^a-z0-9_]', '', clean_name)

    # Use PR number as model_id for dedup (same PR = same model)
    # Check if the registry already has an entry from this PR
    driving_models, dm_models = load_registry()
    registry = driving_models if model_type == 'driving' else dm_models
    existing_id = None
    for mid, minfo in registry.items():
        if minfo.get('pr', '').strip('#()') == str(pr_number):
            existing_id = mid
            break
    model_id = existing_id or f"{clean_name}_{pr_number}"

    print(f"✅ Found: {title}")
    print(f"   Head commit: {head_commit[:12]}")
    print(f"   Merge commit: {merge_commit[:12]}")
    print(f"   Merged: {merged_date}")
    print()

    # Add to registry using head commit (where LFS objects are stored)
    return add_model_to_registry(
        model_type=model_type,
        model_id=model_id,
        name=title,
        commit=head_commit,
        date=merged_date,
        description=f"Driving model from PR #{pr_number}",
        pr=f"#{pr_number}"
    )


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


def update_registry_from_github():
    """Fetch latest model commits from GitHub and update registry

    Three-Layer Filtering System:
    1. Date Filter: Exclude models older than Firehose (2025-09-05) from registry ingestion
    2. Revert Filter: Excludes revert commits themselves from candidacy
       - Detects "Revert" commits and MARKS the reverted model with
         `upstream_reverted`; it is never removed. A revert is not a verdict on
         whether the model drives — only a test drive is.
    3. Already Downloaded Filter: Applied in check_updates() to show only uninstalled models

    Note: Filter #3 is intentionally in check_updates(), not here, because the registry
    should contain ALL available models. The check_updates() function filters what to
    show users based on what's already installed.
    """

    print("🔍 Checking GitHub for new openpilot models...")

    # Fetch commits from GitHub API
    github_api_url = "https://api.github.com/repos/commaai/openpilot/commits"
    params = {
        'path': 'selfdrive/modeld/models',
        'per_page': 30  # Check last 30 commits to catch reverts
    }

    try:
        response = requests.get(github_api_url, params=params, headers=_github_headers())
        response.raise_for_status()
        commits_data = response.json()
    except Exception as e:
        print(f"❌ Failed to fetch commits from GitHub: {e}")
        return 1

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

    return 0


def main():
    parser = argparse.ArgumentParser(
        description='Download openpilot models from GitHub (separated driving/DM models)'
    )
    parser.add_argument('action', choices=['list', 'download', 'check-updates', 'add-model', 'add-from-pr', 'update-registry'],
                       help='Action to perform')
    parser.add_argument('--type', choices=['driving', 'dm'],
                       help='Model type: driving or dm (driver monitoring)')
    parser.add_argument('model_id', nargs='?',
                       help='Model ID to download/add, or PR number for add-from-pr')
    parser.add_argument('--output', '-o', type=Path,
                       help='Output directory (default: /data/models/ or /data/dm-models/)')
    parser.add_argument('--unlocked', action='store_true',
                        help='Maintainer: allow installing a model the catalog has not verified')

    # Arguments for add-model command
    parser.add_argument('--name', help='Model display name (for add-model)')
    parser.add_argument('--commit', help='Full GitHub commit hash (for add-model)')
    parser.add_argument('--date', help='Release date YYYY-MM-DD (for add-model)')
    parser.add_argument('--description', help='Model description (for add-model)')
    parser.add_argument('--pr', help='PR number like #36249 (for add-model)')

    args = parser.parse_args()

    if args.action == 'list':
        model_type = ModelType.DRIVING if args.type == 'driving' else (ModelType.DM if args.type == 'dm' else None)
        list_available(model_type)
        return 0

    elif args.action == 'check-updates':
        return check_updates()

    elif args.action == 'download':
        if not args.model_id:
            print("❌ model_id required for download")
            return 1

        if not args.type:
            print("❌ --type required for download (driving or dm)")
            print()
            print("Examples:")
            print("  python download_openpilot_models.py download --type driving cool_people_3c957c6")
            print("  python download_openpilot_models.py download --type dm medium_fanta_cc8f6ea")
            return 1

        model_type = ModelType.DRIVING if args.type == 'driving' else ModelType.DM

        return download_model(model_type, args.model_id, args.output, allow_untested=args.unlocked)

    elif args.action == 'add-model':
        if not all([args.model_id, args.type, args.name, args.commit, args.date, args.description]):
            print("❌ add-model requires: model_id, --type, --name, --commit, --date, --description")
            print()
            print("Example:")
            print("  python download_openpilot_models.py add-model cool_people_3c957c6 \\")
            print("    --type driving \\")
            print("    --name \"The Cool People's Model 😎\" \\")
            print("    --commit 3c957c6e9d8f05138b8a80523d50db5b5ca2cb73 \\")
            print("    --date 2025-10-20 \\")
            print("    --description \"Latest driving model with improved vision\" \\")
            print("    --pr \"#36249\"")
            return 1

        return add_model_to_registry(args.type, args.model_id, args.name, args.commit,
                                    args.date, args.description, args.pr)

    elif args.action == 'add-from-pr':
        if not args.model_id:
            print("❌ PR number required for add-from-pr")
            print()
            print("Example:")
            print("  python download_openpilot_models.py add-from-pr 36849")
            print("  python download_openpilot_models.py add-from-pr 36849 --type dm")
            return 1

        # Extract PR number (handle URLs or plain numbers)
        pr_input = args.model_id
        if 'github.com' in pr_input:
            # Extract from URL like https://github.com/commaai/openpilot/pull/36849
            import re
            match = re.search(r'/pull/(\d+)', pr_input)
            if match:
                pr_number = int(match.group(1))
            else:
                print(f"❌ Could not extract PR number from URL: {pr_input}")
                return 1
        else:
            try:
                pr_number = int(pr_input.replace('#', ''))
            except ValueError:
                print(f"❌ Invalid PR number: {pr_input}")
                return 1

        model_type = args.type or 'driving'
        return add_model_from_pr(pr_number, model_type)

    elif args.action == 'update-registry':
        return update_registry_from_github()


if __name__ == '__main__':
    import sys
    sys.exit(main())
