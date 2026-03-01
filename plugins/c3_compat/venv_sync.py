#!/usr/bin/env python3
"""venv_sync: sync C3 venv packages against target branch's uv.lock.

Compares /data/openpilot/uv.lock against the target branch's uv.lock on GitHub.
When they differ, determines missing/outdated packages and installs them into the
AGNOS 12.8 system venv at /usr/local/venv/.

Usage:
  python venv_sync.py --branch bmw-master --repo OxygenLiu/c3pilot
  python venv_sync.py --local-lock /path/to/uv.lock  (compare against local file)
  python venv_sync.py --check-only  (exit 0 if synced, 1 if not)

Designed to run on C3 with Python 3.12 (AGNOS 12.8 venv).
Uses only stdlib — no extra dependencies required.
"""
import argparse
import hashlib
import json
import logging
import os
import re
import subprocess
import sys
import urllib.request
import urllib.error

try:
    import tomllib
except ImportError:
    # Python 3.10 fallback (dev machine)
    try:
        import tomli as tomllib
    except ImportError:
        tomllib = None

log = logging.getLogger("venv_sync")

# --- Paths ---
LOCAL_LOCK = "/data/openpilot/uv.lock"
VENV_SITE = "/usr/local/venv/lib/python3.12/site-packages"
VENV_PIP = "/usr/local/venv/bin/pip"
GITHUB_RAW = "https://raw.githubusercontent.com"

# --- C3 target platform ---
# AGNOS 12.8: Python 3.12, aarch64, Linux
TARGET_PYTHON = "cp312"
TARGET_ARCH = "aarch64"


def sha256_of(text: str) -> str:
    return hashlib.sha256(text.encode()).hexdigest()


def read_local_lock() -> str | None:
    """Read the local uv.lock file. Returns None if not found."""
    try:
        with open(LOCAL_LOCK) as f:
            return f.read()
    except FileNotFoundError:
        log.warning("Local uv.lock not found at %s", LOCAL_LOCK)
        return None


def fetch_remote_lock(repo: str, branch: str) -> str | None:
    """Fetch uv.lock content from GitHub raw URL."""
    url = f"{GITHUB_RAW}/{repo}/{branch}/uv.lock"
    log.info("Fetching %s", url)
    try:
        req = urllib.request.Request(url, headers={"User-Agent": "venv_sync/1.0"})
        with urllib.request.urlopen(req, timeout=30) as resp:
            return resp.read().decode()
    except urllib.error.HTTPError as e:
        log.error("Failed to fetch remote uv.lock: HTTP %d", e.code)
        return None
    except Exception as e:
        log.error("Failed to fetch remote uv.lock: %s", e)
        return None


def read_lock_file(path: str) -> str | None:
    """Read a uv.lock from a local path."""
    try:
        with open(path) as f:
            return f.read()
    except FileNotFoundError:
        log.error("Lock file not found: %s", path)
        return None


class PackageInfo:
    __slots__ = ("name", "version", "wheel_url", "wheel_hash")

    def __init__(self, name: str, version: str, wheel_url: str = "", wheel_hash: str = ""):
        self.name = name
        self.version = version
        self.wheel_url = wheel_url
        self.wheel_hash = wheel_hash

    def __repr__(self):
        return f"PackageInfo({self.name}=={self.version})"


def _wheel_matches_target(url: str) -> bool:
    """Check if a wheel URL is compatible with C3 (cp312 + aarch64 + linux, or py3-none-any)."""
    filename = url.rsplit("/", 1)[-1]

    # Pure-python wheels work everywhere
    if "py3-none-any" in filename or "py2.py3-none-any" in filename:
        return True

    # ABI-compatible wheels (e.g., cp39-abi3-manylinux*aarch64)
    if TARGET_ARCH in filename and "linux" in filename:
        # Must be cp312 or abi3 (stable ABI compatible with 3.12)
        if TARGET_PYTHON in filename:
            return True
        if "abi3" in filename:
            # abi3 wheels: check that the minimum version is <= 3.12
            m = re.search(r"cp3(\d+)-abi3", filename)
            if m and int(m.group(1)) <= 12:
                return True

    return False


def parse_lock_packages(lock_text: str) -> dict[str, PackageInfo]:
    """Parse uv.lock TOML → {name: PackageInfo}.

    Filters wheels for cp312 + aarch64 + linux compatibility.
    Falls back to regex parsing if tomllib unavailable.
    """
    if tomllib is not None:
        return _parse_with_tomllib(lock_text)
    return _parse_with_regex(lock_text)


def _parse_with_tomllib(lock_text: str) -> dict[str, PackageInfo]:
    """Parse uv.lock using tomllib (Python 3.11+ stdlib)."""
    data = tomllib.loads(lock_text)
    packages = {}
    for pkg in data.get("package", []):
        name = pkg.get("name", "")
        version = pkg.get("version", "")
        if not name or not version:
            continue

        # Find best matching wheel
        wheel_url = ""
        wheel_hash = ""
        for w in pkg.get("wheels", []):
            url = w.get("url", "")
            if _wheel_matches_target(url):
                wheel_url = url
                raw_hash = w.get("hash", "")
                if raw_hash.startswith("sha256:"):
                    wheel_hash = raw_hash[7:]
                break

        packages[name] = PackageInfo(name, version, wheel_url, wheel_hash)
    return packages


def _parse_with_regex(lock_text: str) -> dict[str, PackageInfo]:
    """Fallback regex parser for systems without tomllib."""
    packages = {}
    # Split on [[package]] boundaries
    blocks = re.split(r'\n(?=\[\[package\]\])', lock_text)
    for block in blocks:
        name_m = re.search(r'^name\s*=\s*"([^"]+)"', block, re.MULTILINE)
        ver_m = re.search(r'^version\s*=\s*"([^"]+)"', block, re.MULTILINE)
        if not name_m or not ver_m:
            continue
        name = name_m.group(1)
        version = ver_m.group(1)

        # Extract wheel URLs
        wheel_url = ""
        wheel_hash = ""
        for url_m in re.finditer(r'url\s*=\s*"([^"]+\.whl)"', block):
            url = url_m.group(1)
            if _wheel_matches_target(url):
                wheel_url = url
                hash_m = re.search(
                    r'hash\s*=\s*"sha256:([^"]+)"',
                    block[url_m.start() - 200:url_m.end() + 200]
                )
                if hash_m:
                    wheel_hash = hash_m.group(1)
                break

        packages[name] = PackageInfo(name, version, wheel_url, wheel_hash)
    return packages


class PackageAction:
    __slots__ = ("name", "old_version", "new_version", "wheel_url", "wheel_hash", "action")

    def __init__(self, name, old_version, new_version, wheel_url, wheel_hash, action):
        self.name = name
        self.old_version = old_version
        self.new_version = new_version
        self.wheel_url = wheel_url
        self.wheel_hash = wheel_hash
        self.action = action  # "install" or "upgrade"

    def __repr__(self):
        if self.action == "upgrade":
            return f"{self.name}: {self.old_version} → {self.new_version}"
        return f"{self.name}: (new) {self.new_version}"


def diff_packages(local: dict[str, PackageInfo], remote: dict[str, PackageInfo]) -> list[PackageAction]:
    """Compare local vs remote packages, return list of actions needed."""
    actions = []
    for name, rpkg in remote.items():
        lpkg = local.get(name)
        if lpkg is None:
            # New package in remote
            if rpkg.wheel_url:
                actions.append(PackageAction(
                    name, "", rpkg.version, rpkg.wheel_url, rpkg.wheel_hash, "install"))
        elif lpkg.version != rpkg.version:
            # Version changed
            if rpkg.wheel_url:
                actions.append(PackageAction(
                    name, lpkg.version, rpkg.version, rpkg.wheel_url, rpkg.wheel_hash, "upgrade"))
    return actions


def _is_installed(name: str) -> bool:
    """Check if a package is importable."""
    # Normalize package name for import (e.g., google-crc32c → google_crc32c)
    import_name = name.replace("-", "_")
    try:
        result = subprocess.run(
            ["/usr/local/venv/bin/python3", "-c", f"import {import_name}"],
            capture_output=True, timeout=10)
        return result.returncode == 0
    except Exception:
        return False


def _get_installed_version(name: str) -> str | None:
    """Get installed version of a package via importlib.metadata."""
    try:
        result = subprocess.run(
            ["/usr/local/venv/bin/python3", "-c",
             f"from importlib.metadata import version; print(version('{name}'))"],
            capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            return result.stdout.strip()
    except Exception:
        pass
    return None


def install_packages(actions: list[PackageAction], dry_run: bool = False) -> dict:
    """Install packages into the AGNOS 12.8 venv.

    Remounts root rw, pip installs wheels, remounts ro.
    Returns summary dict.
    """
    if not actions:
        return {"installed": [], "failed": [], "skipped": []}

    installed = []
    failed = []
    skipped = []

    # Check which actions are actually needed (package might already be at correct version)
    needed = []
    for act in actions:
        current = _get_installed_version(act.name)
        if current == act.new_version:
            skipped.append({"name": act.name, "version": act.new_version, "reason": "already installed"})
            log.info("  %s==%s already installed, skipping", act.name, act.new_version)
        else:
            needed.append(act)

    if not needed:
        return {"installed": installed, "failed": failed, "skipped": skipped}

    if dry_run:
        for act in needed:
            log.info("  [dry-run] Would %s %s==%s from %s",
                     act.action, act.name, act.new_version, act.wheel_url)
            installed.append({"name": act.name, "version": act.new_version, "action": act.action})
        return {"installed": installed, "failed": failed, "skipped": skipped, "dry_run": True}

    # Remount root read-write
    log.info("Remounting / read-write for venv patching")
    subprocess.run(["sudo", "mount", "-o", "remount,rw", "/"],
                   capture_output=True, timeout=10)

    for act in needed:
        log.info("  Installing %s==%s (%s)", act.name, act.new_version, act.action)
        try:
            cmd = [VENV_PIP, "install", "--target", VENV_SITE,
                   "--no-deps", "-q", act.wheel_url]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
            if result.returncode == 0:
                installed.append({"name": act.name, "version": act.new_version, "action": act.action})
                log.info("    OK")
            else:
                failed.append({"name": act.name, "version": act.new_version,
                               "error": result.stderr.strip()[:200]})
                log.error("    FAILED: %s", result.stderr.strip()[:200])
        except subprocess.TimeoutExpired:
            failed.append({"name": act.name, "version": act.new_version, "error": "timeout"})
            log.error("    FAILED: timeout")
        except Exception as e:
            failed.append({"name": act.name, "version": act.new_version, "error": str(e)})
            log.error("    FAILED: %s", e)

    # Re-seal root filesystem
    log.info("Remounting / read-only")
    subprocess.run(["sudo", "mount", "-o", "remount,ro", "/"],
                   capture_output=True, timeout=10)

    return {"installed": installed, "failed": failed, "skipped": skipped}


def sync(repo: str = "OxygenLiu/c3pilot", branch: str = "bmw-master",
         local_lock_path: str | None = None,
         check_only: bool = False, dry_run: bool = False) -> dict:
    """Main entry point. Returns sync result dict.

    Args:
        repo: GitHub owner/repo for remote uv.lock
        branch: Branch name to fetch uv.lock from
        local_lock_path: Optional path to a local uv.lock to compare against (instead of GitHub)
        check_only: If True, only report status without installing
        dry_run: If True, show what would be installed without doing it

    Returns:
        {synced: bool, hash_match: bool, actions: [...], installed: [...], ...}
    """
    # Read local lock
    local_text = read_local_lock()
    if local_text is None:
        return {"synced": False, "error": "local uv.lock not found"}

    # Get remote/target lock
    if local_lock_path:
        remote_text = read_lock_file(local_lock_path)
    else:
        remote_text = fetch_remote_lock(repo, branch)
    if remote_text is None:
        return {"synced": False, "error": "failed to fetch remote uv.lock"}

    # Fast path: hash comparison
    local_hash = sha256_of(local_text)
    remote_hash = sha256_of(remote_text)
    if local_hash == remote_hash:
        log.info("uv.lock hashes match — venv is in sync")
        return {"synced": True, "hash_match": True, "local_hash": local_hash[:12]}

    log.info("uv.lock hashes differ (local=%s, remote=%s) — analyzing packages",
             local_hash[:12], remote_hash[:12])

    # Parse and diff
    local_pkgs = parse_lock_packages(local_text)
    remote_pkgs = parse_lock_packages(remote_text)
    actions = diff_packages(local_pkgs, remote_pkgs)

    if not actions:
        log.info("Hashes differ but no actionable package changes (wheels not available for aarch64)")
        return {
            "synced": True, "hash_match": False,
            "local_hash": local_hash[:12], "remote_hash": remote_hash[:12],
            "note": "lock files differ but no aarch64 wheel changes",
        }

    log.info("Found %d package(s) to %s:", len(actions),
             "check" if check_only else "install")
    for act in actions:
        log.info("  %s", act)

    if check_only:
        return {
            "synced": False, "hash_match": False,
            "local_hash": local_hash[:12], "remote_hash": remote_hash[:12],
            "actions": [{"name": a.name, "old": a.old_version,
                         "new": a.new_version, "action": a.action} for a in actions],
        }

    # Install
    result = install_packages(actions, dry_run=dry_run)
    return {
        "synced": len(result["failed"]) == 0,
        "hash_match": False,
        "local_hash": local_hash[:12],
        "remote_hash": remote_hash[:12],
        **result,
    }


def main():
    parser = argparse.ArgumentParser(description="Sync C3 venv packages against target branch uv.lock")
    parser.add_argument("--repo", default="OxygenLiu/c3pilot",
                        help="GitHub owner/repo (default: OxygenLiu/c3pilot)")
    parser.add_argument("--branch", default="bmw-master",
                        help="Branch name (default: bmw-master)")
    parser.add_argument("--local-lock", default=None,
                        help="Compare against a local uv.lock file instead of GitHub")
    parser.add_argument("--check-only", action="store_true",
                        help="Only check sync status, don't install (exit 0=synced, 1=not)")
    parser.add_argument("--dry-run", action="store_true",
                        help="Show what would be installed without doing it")
    parser.add_argument("--json", action="store_true",
                        help="Output result as JSON")
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="Verbose logging")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="[venv_sync] %(message)s",
    )

    result = sync(
        repo=args.repo,
        branch=args.branch,
        local_lock_path=args.local_lock,
        check_only=args.check_only,
        dry_run=args.dry_run,
    )

    if args.json:
        print(json.dumps(result, indent=2))
    else:
        if result.get("error"):
            print(f"ERROR: {result['error']}")
        elif result.get("synced"):
            if result.get("hash_match"):
                print("Venv is in sync (lock hashes match)")
            else:
                print(f"Venv is in sync ({result.get('note', 'all packages installed')})")
                for pkg in result.get("installed", []):
                    print(f"  {pkg['action']}: {pkg['name']}=={pkg['version']}")
                for pkg in result.get("skipped", []):
                    print(f"  skipped: {pkg['name']}=={pkg['version']} ({pkg['reason']})")
        else:
            print("Venv is OUT OF SYNC")
            for act in result.get("actions", []):
                label = "upgrade" if act["old"] else "install"
                print(f"  {label}: {act['name']} {act['old']} → {act['new']}")
            for pkg in result.get("installed", []):
                print(f"  installed: {pkg['name']}=={pkg['version']}")
            for pkg in result.get("failed", []):
                print(f"  FAILED: {pkg['name']}=={pkg['version']}: {pkg['error']}")

    # Exit code for --check-only mode
    if args.check_only:
        sys.exit(0 if result.get("synced") else 1)


if __name__ == "__main__":
    main()
