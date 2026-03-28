#!/usr/bin/env python3
"""venv_sync: ensure C3 venv matches the deployed branch's uv.lock.

Primary mode (--ensure, default):
  Parse /data/openpilot/uv.lock, check each package against what's installed
  in the AGNOS 12.8 venv, install anything missing or at the wrong version.
  Runs at boot in boot_patch.sh BEFORE openpilot launches — guarantees the
  venv is correct regardless of how the code got deployed.

Fast path:
  Caches the hash of the last successfully synced uv.lock. If the current
  uv.lock matches, skip entirely (<100ms on normal boots).

Usage:
  python venv_sync.py                     # ensure venv matches local uv.lock
  python venv_sync.py --check-only        # report status, don't install
  python venv_sync.py --dry-run           # show what would be installed
  python venv_sync.py --json              # JSON output for programmatic use

Designed for C3 with Python 3.12 (AGNOS 12.8 venv).
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

try:
    import tomllib
except ImportError:
    try:
        import tomli as tomllib
    except ImportError:
        tomllib = None

log = logging.getLogger("venv_sync")

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import OPENPILOT_DIR, plugin_data_dir

# --- Paths ---
LOCAL_LOCK = os.path.join(OPENPILOT_DIR, "uv.lock")
VENV_SITE = "/usr/local/venv/lib/python3.12/site-packages"
VENV_PIP = "/usr/local/venv/bin/pip"
VENV_PYTHON = "/usr/local/venv/bin/python3"
HASH_CACHE = str(plugin_data_dir("c3_compat").parent / ".venv_synced_hash")

# --- C3 target platform ---
TARGET_PYTHON = "cp312"
TARGET_ARCH = "aarch64"

# --- commaai/dependencies packages (not in uv.lock — git-sourced native libs) ---
# These wrap pre-built native libraries and provide INCLUDE_DIR/LIB_DIR for scons.
COMMAAI_DEPS_REPO = "https://github.com/commaai/dependencies.git@releases"
COMMAAI_DEPS = [
    "bzip2", "capnproto", "eigen", "ffmpeg",
    "libjpeg", "libyuv", "ncurses", "zeromq", "zstd",
]
NATIVE_DEPS_CACHE = str(plugin_data_dir("c3_compat").parent / ".native_deps_installed")


def sha256_of(text: str) -> str:
    return hashlib.sha256(text.encode()).hexdigest()


def _read_cached_hash() -> str:
    """Read the hash of the last successfully synced uv.lock."""
    try:
        with open(HASH_CACHE) as f:
            return f.read().strip()
    except FileNotFoundError:
        return ""


def _write_cached_hash(h: str):
    """Write the hash after a successful sync."""
    try:
        os.makedirs(os.path.dirname(HASH_CACHE), exist_ok=True)
        with open(HASH_CACHE, "w") as f:
            f.write(h)
    except OSError as e:
        log.warning("Could not write hash cache: %s", e)


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
        if TARGET_PYTHON in filename:
            return True
        if "abi3" in filename:
            m = re.search(r"cp3(\d+)-abi3", filename)
            if m and int(m.group(1)) <= 12:
                return True

    return False


def _eval_single_marker(clause: str) -> bool | None:
    """Evaluate a single marker clause for C3 (Python 3.12, aarch64, linux).

    Returns True/False if deterministic, None if unknown.
    """
    clause = clause.strip()

    # C3 environment values:
    #   sys_platform = 'linux'
    #   platform_machine = 'aarch64'
    #   os_name = 'posix'
    #   platform_python_implementation = 'CPython'
    #   implementation_name = 'cpython'
    #   python_full_version = '3.12.x'

    # == checks (equality)
    if "sys_platform == 'darwin'" in clause:
        return False
    if "sys_platform == 'win32'" in clause:
        return False
    if "os_name == 'nt'" in clause:
        return False
    if "implementation_name == 'pypy'" in clause:
        return False

    if "sys_platform == 'linux'" in clause:
        return True
    if "platform_machine == 'aarch64'" in clause:
        return True
    if "os_name == 'posix'" in clause:
        return True
    if "platform_python_implementation == 'CPython'" in clause:
        return True
    if "platform_python_implementation != 'PyPy'" in clause:
        return True

    # != checks (inequality)
    if "sys_platform != 'linux'" in clause:
        return False
    if "sys_platform != 'darwin'" in clause:
        return True
    if "sys_platform != 'win32'" in clause:
        return True
    if "platform_machine != 'aarch64'" in clause:
        return False

    # Python version checks (we're on 3.12)
    if "python_full_version < '3.12'" in clause:
        return False
    if "python_version < '3'" in clause or "python_version < '2'" in clause:
        return False

    return None  # unknown


def _marker_applies_to_c3(marker: str) -> bool:
    """Evaluate a PEP 508 environment marker for C3 (Python 3.12, aarch64, linux).

    Returns True if the marker condition is satisfied on C3.
    Handles compound `and`/`or` expressions from uv.lock.
    Defaults to True for unknown markers (safe: install rather than miss).
    """
    if not marker:
        return True

    marker = marker.strip()

    # Handle compound OR: all clauses must be False for the whole thing to be False
    if " or " in marker:
        clauses = marker.split(" or ")
        results = [_eval_single_marker(c) for c in clauses]
        # If any clause is True, the OR is True
        if any(r is True for r in results):
            return True
        # If all clauses are deterministically False, the OR is False
        if all(r is False for r in results):
            return False
        # Some unknown — default True (safe)
        return True

    # Handle compound AND: any clause being False makes the whole thing False
    if " and " in marker:
        clauses = marker.split(" and ")
        results = [_eval_single_marker(c) for c in clauses]
        # If any clause is False, the AND is False
        if any(r is False for r in results):
            return False
        # If all clauses are deterministically True, the AND is True
        if all(r is True for r in results):
            return True
        # Some unknown — default True (safe)
        return True

    # Single clause
    result = _eval_single_marker(marker)
    return result if result is not None else True


def parse_lock_packages(lock_text: str, runtime_only: bool = False) -> dict[str, PackageInfo]:
    """Parse uv.lock TOML → {name: PackageInfo}.

    When tomllib is available, also resolves the dependency graph to determine
    which packages are actually needed on C3 (linux/aarch64), filtering out
    macOS-only, Windows-only, and other platform-specific packages.

    If runtime_only=True, skip optional-dependencies (dev/testing/docs/tools)
    and only include packages needed to run openpilot.

    Falls back to regex parsing if tomllib unavailable (no dep graph filtering).
    """
    if tomllib is not None:
        return _parse_with_tomllib(lock_text, runtime_only=runtime_only)
    return _parse_with_regex(lock_text)


def _parse_with_tomllib(lock_text: str, runtime_only: bool = False) -> dict[str, PackageInfo]:
    data = tomllib.loads(lock_text)

    # Build full package index
    all_packages = {}
    for pkg in data.get("package", []):
        name = pkg.get("name", "")
        version = pkg.get("version", "")
        if not name or not version:
            continue

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

        # uv.lock uses "optional-dependencies" (not "dev-dependencies") for dev/test/tools groups
        opt_deps = pkg.get("optional-dependencies", {})

        all_packages[name] = {
            "info": PackageInfo(name, version, wheel_url, wheel_hash),
            "deps": pkg.get("dependencies", []),
            "opt_deps": opt_deps,
        }

    # Walk dependency graph from root, filtering by platform markers
    needed = set()
    _walk_deps(all_packages, needed, include_optional=not runtime_only)

    mode = "runtime-only" if runtime_only else "full"
    log.debug("Dependency walk (%s): %d/%d packages needed on C3", mode, len(needed), len(all_packages))
    return {name: all_packages[name]["info"] for name in needed if name in all_packages}


def _walk_deps(packages: dict, needed: set, root: str | None = None,
               include_optional: bool = True):
    """Walk the dependency graph, collecting packages reachable via C3-compatible markers.

    If include_optional=False, skip optional-dependencies (dev/testing/docs/tools)
    and only walk runtime dependencies.
    """
    # Find root package (the one with optional-dependencies — usually 'openpilot')
    if root is None:
        for candidate in packages:
            if packages[candidate].get("opt_deps"):
                root = candidate
                break
        if root is None:
            # No clear root — include everything
            needed.update(packages.keys())
            return

    queue = [root]
    while queue:
        name = queue.pop()
        if name in needed or name not in packages:
            continue
        needed.add(name)

        pkg = packages[name]
        # Regular dependencies (with marker filtering at every edge)
        for dep in pkg["deps"]:
            dep_name = dep["name"]
            marker = dep.get("marker", "")
            if _marker_applies_to_c3(marker) and dep_name not in needed:
                queue.append(dep_name)

        # Optional dependencies (dev/testing/tools groups — only from root package)
        if name == root and include_optional:
            for group_deps in pkg["opt_deps"].values():
                for dep in group_deps:
                    dep_name = dep["name"]
                    marker = dep.get("marker", "")
                    if _marker_applies_to_c3(marker) and dep_name not in needed:
                        queue.append(dep_name)


def _parse_with_regex(lock_text: str) -> dict[str, PackageInfo]:
    """Fallback regex parser — no dependency graph filtering (includes all packages)."""
    packages = {}
    blocks = re.split(r'\n(?=\[\[package\]\])', lock_text)
    for block in blocks:
        name_m = re.search(r'^name\s*=\s*"([^"]+)"', block, re.MULTILINE)
        ver_m = re.search(r'^version\s*=\s*"([^"]+)"', block, re.MULTILINE)
        if not name_m or not ver_m:
            continue
        name = name_m.group(1)
        version = ver_m.group(1)

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
    __slots__ = ("name", "installed_version", "needed_version", "wheel_url", "wheel_hash", "action")

    def __init__(self, name, installed_version, needed_version, wheel_url, wheel_hash, action):
        self.name = name
        self.installed_version = installed_version
        self.needed_version = needed_version
        self.wheel_url = wheel_url
        self.wheel_hash = wheel_hash
        self.action = action  # "install" or "upgrade"

    def __repr__(self):
        if self.action == "upgrade":
            return f"{self.name}: {self.installed_version} → {self.needed_version}"
        return f"{self.name}: (new) {self.needed_version}"


def _batch_get_installed_versions(names: list[str]) -> dict[str, str | None]:
    """Get installed versions of all packages in a single subprocess.

    Returns {name: version_string} for installed packages,
    {name: None} for missing packages.
    """
    if not names:
        return {}

    # Build a Python script that checks all packages at once
    check_script = "from importlib.metadata import version, PackageNotFoundError\n"
    check_script += "names = %r\n" % names
    check_script += """for n in names:
    try:
        print(f"{n}={version(n)}")
    except (PackageNotFoundError, Exception):
        print(f"{n}=__MISSING__")
"""

    python_bin = VENV_PYTHON if os.path.exists(VENV_PYTHON) else sys.executable
    try:
        result = subprocess.run(
            [python_bin, "-c", check_script],
            capture_output=True, text=True, timeout=30)
        versions = {}
        for line in result.stdout.strip().split("\n"):
            if "=" in line:
                name, ver = line.split("=", 1)
                versions[name] = None if ver == "__MISSING__" else ver
        return versions
    except Exception as e:
        log.warning("Batch version check failed: %s", e)
        return {n: None for n in names}


def find_actions(packages: dict[str, PackageInfo]) -> list[PackageAction]:
    """Compare packages from uv.lock against what's installed in the venv.

    Returns list of packages that need to be installed or upgraded.
    Only considers packages that have compatible wheels for C3.
    """
    # Filter to packages with installable wheels
    installable = {n: p for n, p in packages.items() if p.wheel_url}
    if not installable:
        return []

    log.info("Checking %d packages with aarch64 wheels against installed venv...", len(installable))

    # Batch check all installed versions (single subprocess, ~1 second)
    installed = _batch_get_installed_versions(list(installable.keys()))

    actions = []
    for name, pkg in installable.items():
        current = installed.get(name)
        if current is None:
            actions.append(PackageAction(
                name, "", pkg.version, pkg.wheel_url, pkg.wheel_hash, "install"))
        elif current != pkg.version:
            actions.append(PackageAction(
                name, current, pkg.version, pkg.wheel_url, pkg.wheel_hash, "upgrade"))

    return actions


def install_packages(actions: list[PackageAction], dry_run: bool = False) -> dict:
    """Install packages into the AGNOS 12.8 venv.

    Remounts root rw, pip installs wheels, remounts ro.
    Returns summary dict.
    """
    if not actions:
        return {"installed": [], "failed": [], "skipped": []}

    installed = []
    failed = []

    if dry_run:
        for act in actions:
            log.info("  [dry-run] Would %s %s==%s from %s",
                     act.action, act.name, act.needed_version, act.wheel_url)
            installed.append({"name": act.name, "version": act.needed_version, "action": act.action})
        return {"installed": installed, "failed": failed, "skipped": [], "dry_run": True}

    # Remount root read-write
    log.info("Remounting / read-write for venv patching")
    subprocess.run(["sudo", "mount", "-o", "remount,rw", "/"],
                   capture_output=True, timeout=10)

    # Use a tmpdir on the same filesystem as the venv to avoid cross-device link errors
    # (AGNOS /tmp is a 150MB tmpfs — pip can't os.rename() from tmpfs to root ext4)
    pip_tmpdir = "/usr/local/tmp/pip"
    subprocess.run(["sudo", "mkdir", "-p", pip_tmpdir], capture_output=True, timeout=5)

    # pip_env: set TMPDIR on the root filesystem, keep PATH for pip to find itself
    pip_env = {**os.environ, "TMPDIR": pip_tmpdir}

    for act in actions:
        log.info("  Installing %s==%s (%s)", act.name, act.needed_version, act.action)
        try:
            # sudo required: venv is root-owned on read-only root filesystem
            # No --target: install into venv normally so dist-info gets updated correctly
            cmd = ["sudo", "-E", VENV_PIP, "install",
                   "--no-deps", "--no-cache-dir", "-q", act.wheel_url]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=120,
                                    env=pip_env)
            if result.returncode == 0:
                installed.append({"name": act.name, "version": act.needed_version, "action": act.action})
                log.info("    OK")
            else:
                failed.append({"name": act.name, "version": act.needed_version,
                               "error": result.stderr.strip()[:200]})
                log.error("    FAILED: %s", result.stderr.strip()[:200])
        except subprocess.TimeoutExpired:
            failed.append({"name": act.name, "version": act.needed_version, "error": "timeout"})
            log.error("    FAILED: timeout")
        except Exception as e:
            failed.append({"name": act.name, "version": act.needed_version, "error": str(e)})
            log.error("    FAILED: %s", e)

    # Clean up pip tmpdir
    subprocess.run(["sudo", "rm", "-rf", pip_tmpdir], capture_output=True, timeout=10)

    # Re-seal root filesystem
    log.info("Remounting / read-only")
    subprocess.run(["sudo", "mount", "-o", "remount,ro", "/"],
                   capture_output=True, timeout=10)

    return {"installed": installed, "failed": failed, "skipped": []}


def ensure_venv(check_only: bool = False, dry_run: bool = False,
                lock_path: str | None = None, runtime_only: bool = False) -> dict:
    """Ensure the venv matches the local uv.lock. Primary entry point.

    1. Read /data/openpilot/uv.lock (or override with lock_path)
    2. Fast path: if hash matches cached .venv_synced_hash, skip
    3. Parse packages, batch-check installed versions
    4. Install missing/wrong-version packages
    5. Cache hash on success

    Returns: {synced: bool, actions: [...], installed: [...], ...}
    """
    # Read local lock
    path = lock_path or LOCAL_LOCK
    try:
        with open(path) as f:
            lock_text = f.read()
    except FileNotFoundError:
        log.warning("uv.lock not found at %s", path)
        return {"synced": False, "error": f"uv.lock not found at {path}"}

    lock_hash = sha256_of(lock_text)

    # Fast path: already synced for this exact uv.lock
    cached = _read_cached_hash()
    if cached == lock_hash and not check_only and not dry_run:
        log.info("venv already synced for uv.lock %s (cached)", lock_hash[:12])
        return {"synced": True, "cached": True, "hash": lock_hash[:12]}

    # Parse packages
    packages = parse_lock_packages(lock_text, runtime_only=runtime_only)
    log.info("Parsed %d packages from uv.lock (%s)", len(packages), lock_hash[:12])

    # Compare against installed venv
    actions = find_actions(packages)

    if not actions:
        log.info("venv is in sync — all %d installable packages match", len(packages))
        if not check_only and not dry_run:
            _write_cached_hash(lock_hash)
        return {"synced": True, "cached": False, "hash": lock_hash[:12],
                "checked": len(packages)}

    log.info("Found %d package(s) to %s:", len(actions),
             "check" if check_only else ("install" if not dry_run else "dry-run"))
    for act in actions:
        log.info("  %s", act)

    if check_only:
        return {
            "synced": False, "hash": lock_hash[:12],
            "actions": [{"name": a.name, "installed": a.installed_version,
                         "needed": a.needed_version, "action": a.action} for a in actions],
        }

    # Install
    result = install_packages(actions, dry_run=dry_run)

    synced = len(result["failed"]) == 0
    if synced and not dry_run:
        _write_cached_hash(lock_hash)

    return {"synced": synced, "hash": lock_hash[:12], **result}


def ensure_native_deps(dry_run: bool = False) -> dict:
    """Install commaai/dependencies packages if missing from the venv.

    These are not in uv.lock (they're git-sourced and bundle pre-built native
    libraries needed by scons). Checks a cache file to skip on normal boots.

    Returns: {installed: [...], failed: [...], skipped: bool}
    """
    # Fast path: if all were installed last time, check the cache
    try:
        with open(NATIVE_DEPS_CACHE) as f:
            cached = f.read().strip().split()
        if set(cached) >= set(COMMAAI_DEPS):
            log.debug("native deps already installed (cached)")
            return {"installed": [], "failed": [], "skipped": True}
    except FileNotFoundError:
        pass

    # Check which packages are actually missing (fast importlib check)
    import importlib
    missing = []
    for name in COMMAAI_DEPS:
        try:
            importlib.import_module(name)
        except ImportError:
            missing.append(name)

    if not missing:
        log.info("native deps: all %d packages present", len(COMMAAI_DEPS))
        _write_native_deps_cache(COMMAAI_DEPS)
        return {"installed": [], "failed": [], "skipped": True}

    log.info("native deps: %d missing: %s", len(missing), missing)
    if dry_run:
        return {"installed": missing, "failed": [], "skipped": False, "dry_run": True}

    installed = []
    failed = []

    # Remount rw — venv is root-owned on read-only root filesystem
    subprocess.run(["sudo", "mount", "-o", "remount,rw", "/"],
                   capture_output=True, timeout=10)

    # Use /data/tmp as TMPDIR — ffmpeg static libs are ~100MB,
    # far exceeding the 150MB /tmp tmpfs. Other packages are small.
    pip_tmpdir = "/data/tmp/pip_native"
    os.makedirs(pip_tmpdir, exist_ok=True)
    pip_env = {**os.environ, "TMPDIR": pip_tmpdir, "GIT_SSL_NO_VERIFY": "1"}

    # Install in one pip call (pip clones the repo once, builds all subdirs)
    specs = [f"git+{COMMAAI_DEPS_REPO}#subdirectory={name}" for name in missing]
    log.info("Installing: %s", ", ".join(missing))
    try:
        result = subprocess.run(
            ["sudo", "-E", VENV_PIP, "install", "--no-build-isolation"] + specs,
            capture_output=True, text=True, timeout=600, env=pip_env,
        )
        if result.returncode == 0:
            installed.extend(missing)
            log.info("native deps: installed %s", installed)
        else:
            # Batch failed — try each individually so one bad package doesn't block others
            log.warning("Batch install failed, retrying individually")
            for name in missing:
                spec = f"git+{COMMAAI_DEPS_REPO}#subdirectory={name}"
                r = subprocess.run(
                    ["sudo", "-E", VENV_PIP, "install", "--no-build-isolation", spec],
                    capture_output=True, text=True, timeout=300, env=pip_env,
                )
                if r.returncode == 0:
                    installed.append(name)
                else:
                    err = (r.stderr or r.stdout).strip()[-200:]
                    failed.append({"name": name, "error": err})
                    log.error("native deps: failed to install %s: %s", name, err)
    except subprocess.TimeoutExpired:
        failed.append({"name": "all", "error": "timeout"})
        log.error("native deps: install timed out")
    except Exception as e:
        failed.append({"name": "all", "error": str(e)})
        log.error("native deps: install error: %s", e)
    finally:
        subprocess.run(["sudo", "rm", "-rf", pip_tmpdir], capture_output=True, timeout=10)
        subprocess.run(["sudo", "mount", "-o", "remount,ro", "/"],
                       capture_output=True, timeout=10)

    if installed:
        _write_native_deps_cache(installed)

    return {"installed": installed, "failed": failed, "skipped": False}


def _write_native_deps_cache(names: list):
    try:
        os.makedirs(os.path.dirname(NATIVE_DEPS_CACHE), exist_ok=True)
        with open(NATIVE_DEPS_CACHE, "w") as f:
            f.write("\n".join(names))
    except OSError as e:
        log.warning("Could not write native deps cache: %s", e)


def main():
    parser = argparse.ArgumentParser(
        description="Ensure C3 venv matches deployed branch's uv.lock")
    parser.add_argument("--lock", default=None,
                        help="Path to uv.lock (default: /data/openpilot/uv.lock)")
    parser.add_argument("--check-only", action="store_true",
                        help="Report status without installing (exit 0=synced, 1=not)")
    parser.add_argument("--dry-run", action="store_true",
                        help="Show what would be installed without doing it")
    parser.add_argument("--json", action="store_true",
                        help="Output result as JSON")
    parser.add_argument("--runtime-only", action="store_true",
                        help="Only sync runtime deps (skip dev/testing/docs)")
    parser.add_argument("--native-deps", action="store_true",
                        help="Also install commaai/dependencies packages (not in uv.lock)")
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="Verbose logging")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="[venv_sync] %(message)s",
    )

    # Install commaai/dependencies packages (scons native libs) if requested
    if args.native_deps:
        nd = ensure_native_deps(dry_run=args.dry_run)
        if not nd["skipped"]:
            if nd.get("dry_run"):
                print(f"[dry-run] Would install native deps: {nd['installed']}")
            else:
                for name in nd.get("installed", []):
                    print(f"  installed native dep: {name}")
                for pkg in nd.get("failed", []):
                    print(f"  FAILED native dep: {pkg['name']}: {pkg['error']}")

    result = ensure_venv(
        check_only=args.check_only,
        dry_run=args.dry_run,
        lock_path=args.lock,
        runtime_only=args.runtime_only,
    )

    if args.json:
        print(json.dumps(result, indent=2))
    else:
        if result.get("error"):
            print(f"ERROR: {result['error']}")
        elif result.get("synced"):
            if result.get("cached"):
                print(f"venv in sync (cached, hash {result['hash']})")
            else:
                checked = result.get("checked", 0)
                msg = f"venv in sync (checked {checked} packages)" if checked else "venv in sync"
                print(msg)
                for pkg in result.get("installed", []):
                    print(f"  {pkg['action']}: {pkg['name']}=={pkg['version']}")
        else:
            print("venv OUT OF SYNC")
            for act in result.get("actions", []):
                label = "upgrade" if act.get("installed") else "install"
                old = act.get("installed", "")
                print(f"  {label}: {act['name']} {old + ' → ' if old else ''}{act['needed']}")
            for pkg in result.get("installed", []):
                print(f"  installed: {pkg['name']}=={pkg['version']}")
            for pkg in result.get("failed", []):
                print(f"  FAILED: {pkg['name']}=={pkg['version']}: {pkg['error']}")

    if args.check_only:
        sys.exit(0 if result.get("synced") else 1)


if __name__ == "__main__":
    main()
