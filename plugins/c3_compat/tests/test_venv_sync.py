"""venv_sync: lock parsing for the C3, the uv.lock fast path and hash-pinned installs."""
import importlib
import os
import subprocess
import sys

import pytest

_PLUGINS_DIR = __import__('os').path.dirname(__import__('os').path.dirname(__import__('os').path.dirname(__import__('os').path.abspath(__file__))))
if _PLUGINS_DIR not in sys.path:
  sys.path.insert(0, _PLUGINS_DIR)


@pytest.fixture
def vs(monkeypatch, tmp_path):
  monkeypatch.setenv("OPENPILOT_DIR", str(tmp_path))
  monkeypatch.setenv("PLUGINS_RUNTIME_DIR", str(tmp_path / "rt"))
  import config
  importlib.reload(config)
  import plugins.c3_compat.venv_sync as mod
  importlib.reload(mod)
  yield mod
  # the shared `config` module was reloaded under the patched env: put it back
  # for the other plugins' suites once monkeypatch has restored the env
  monkeypatch.undo()
  importlib.reload(config)


# openpilot 0.11.2's comma-deps-* wheels are tagged py3-none-manylinux_2_28_aarch64:
# no cp312 / abi3 in the name, but built for the C3 (glibc 2.39 >= 2.28).
@pytest.mark.parametrize("fn, ok", [
  ("comma_deps_raylib-6.0.0.1.post103-py3-none-manylinux_2_28_aarch64.whl", True),
  ("comma_deps_acados-0.2.2.post103-py3-none-manylinux_2_28_x86_64.whl", False),
  ("comma_deps_acados-0.2.2.post103-py3-none-macosx_11_0_arm64.whl", False),
  ("numpy-2.5.3-cp312-cp312-manylinux_2_28_aarch64.whl", True),
  ("some-1.0-py3-none-any.whl", True),
])
def test_wheel_matches_target_accepts_py3_none_manylinux_aarch64(vs, fn, ok):
  assert vs._wheel_matches_target("https://x/" + fn) is ok


def test_runs_as_a_script_without_pythonpath(tmp_path):
  """boot_patch.sh runs venv_sync.py as a bare script before launch_chffrplus.sh exports PYTHONPATH:
  the shared `config` module (runtime root, one directory up) must be importable from the script itself."""
  import os, subprocess
  script = os.path.join(_PLUGINS_DIR, 'c3_compat', 'venv_sync.py')
  env = {k: v for k, v in os.environ.items() if k != 'PYTHONPATH'}
  r = subprocess.run([sys.executable, script, '--help'], cwd=tmp_path, env=env, capture_output=True, text=True, timeout=60)
  assert r.returncode == 0, r.stderr
  assert 'usage' in r.stdout.lower()


LOCK_OPTIONAL_NATIVE = '''
[[package]]
name = "openpilot"
version = "0.1.0"
source = { editable = "." }
dependencies = [{ name = "numpy" }]

[package.optional-dependencies]
testing = [{ name = "comma-deps-libusb" }, { name = "pytest" }]

[[package]]
name = "numpy"
version = "2.5.3"
source = { registry = "https://pypi.org/simple" }
wheels = [{ url = "https://files.example/numpy-2.5.3-cp312-cp312-manylinux_2_28_aarch64.whl", hash = "sha256:aa" }]

[[package]]
name = "comma-deps-libusb"
version = "1.0.29.post103"
source = { registry = "https://pypi.org/simple" }
wheels = [{ url = "https://files.example/comma_deps_libusb-1.0.29.post103-py3-none-manylinux_2_28_aarch64.whl", hash = "sha256:bb" }]

[[package]]
name = "pytest"
version = "8.0.0"
source = { registry = "https://pypi.org/simple" }
wheels = [{ url = "https://files.example/pytest-8.0.0-py3-none-any.whl", hash = "sha256:cc" }]
'''


def test_runtime_only_keeps_optional_comma_deps_wheels(vs):
  """SConstruct imports every vendored comma-deps-* package it links (c3_compat adds libusb for pandad USB);
  upstream lists some of them only under optional-dependencies, so --runtime-only must still install them."""
  pkgs = vs.parse_lock_packages(LOCK_OPTIONAL_NATIVE, runtime_only=True)
  assert 'numpy' in pkgs
  assert 'comma-deps-libusb' in pkgs
  assert 'pytest' not in pkgs


# --- uv.lock fast path: lock hash + venv fingerprint, kept in the preserved data/ dir ---

LOCK_TEXT = '''
[[package]]
name = "numpy"
version = "2.5.3"
source = { registry = "https://pypi.org/simple" }

[[package]]
name = "comma-deps-raylib"
version = "6.0.0.1.post103"
source = { registry = "https://pypi.org/simple" }
'''


@pytest.fixture
def synced(vs, tmp_path, monkeypatch):
  """A venv at tmp_path/site, a uv.lock, and a find_actions that reports the venv in sync."""
  site = tmp_path / "site"
  site.mkdir()
  vs.VENV_SITE = str(site)
  (tmp_path / "uv.lock").write_text(LOCK_TEXT)
  vs.LOCAL_LOCK = str(tmp_path / "uv.lock")
  calls = []
  def _find_actions(packages):
    calls.append(packages)
    return []
  monkeypatch.setattr(vs, "find_actions", _find_actions)
  return vs, site, calls


def test_hash_cache_lives_in_preserved_data_dir(vs, tmp_path):
  """install.sh rm -rf's the plugin runtime dir on every boot, keeping only data/ (+ markers)."""
  import config
  assert vs.HASH_CACHE == str(config.plugin_data_dir("c3_compat") / ".venv_synced_hash")
  assert vs.HASH_CACHE == str(tmp_path / "rt" / "c3_compat" / "data" / ".venv_synced_hash")


def test_sync_writes_hash_and_fingerprint(synced):
  vs, site, calls = synced
  r = vs.ensure_venv()
  assert r["synced"] is True and r["cached"] is False and len(calls) == 1
  with open(vs.HASH_CACHE) as f:
    fields = f.read().split()
  assert fields == [vs.sha256_of(LOCK_TEXT), str(os.stat(site).st_mtime_ns)]


def test_fast_path_when_hash_and_fingerprint_match(synced):
  vs, site, calls = synced
  vs.ensure_venv()
  r = vs.ensure_venv()
  assert r["cached"] is True
  assert len(calls) == 1   # second run did not re-check the venv


def test_no_fast_path_when_site_packages_changed(synced):
  vs, site, calls = synced
  vs.ensure_venv()
  st = os.stat(site)
  os.utime(site, ns=(st.st_atime_ns, st.st_mtime_ns + 1_000_000_000))   # pip added/removed a package, or a reflash
  r = vs.ensure_venv()
  assert r["cached"] is False and len(calls) == 2


def test_no_fast_path_when_lock_changed(synced, tmp_path):
  vs, site, calls = synced
  vs.ensure_venv()
  (tmp_path / "uv.lock").write_text(LOCK_TEXT.replace("2.5.3", "2.5.4"))
  r = vs.ensure_venv()
  assert r["cached"] is False and len(calls) == 2


def test_old_one_field_cache_is_a_miss(synced):
  vs, site, calls = synced
  os.makedirs(os.path.dirname(vs.HASH_CACHE), exist_ok=True)
  with open(vs.HASH_CACHE, "w") as f:
    f.write(vs.sha256_of(LOCK_TEXT))   # pre-fingerprint format
  r = vs.ensure_venv()
  assert r["cached"] is False and len(calls) == 1


def test_no_fast_path_without_venv_site(synced, tmp_path):
  vs, site, calls = synced
  vs.ensure_venv()
  vs.VENV_SITE = str(tmp_path / "missing")
  r = vs.ensure_venv()
  assert r["cached"] is False and len(calls) == 2
  vs.ensure_venv()
  assert len(calls) == 3   # never cached while VENV_SITE can't be stat'ed


def test_fingerprint_read_after_installs(synced, monkeypatch):
  """The cached fingerprint is the venv state after pip ran, not before."""
  vs, site, calls = synced
  act = vs.PackageAction("numpy", "", "2.4.2", "https://x/numpy.whl", "", "install")
  monkeypatch.setattr(vs, "find_actions", lambda packages: [act])
  def _install(actions, dry_run=False):
    (site / "numpy-2.4.2.dist-info").mkdir()   # what pip does to site-packages
    st = os.stat(site)
    os.utime(site, ns=(st.st_atime_ns, st.st_mtime_ns + 1_000_000_000))
    return {"installed": [{"name": "numpy"}], "failed": [], "skipped": []}
  monkeypatch.setattr(vs, "install_packages", _install)
  vs.ensure_venv()
  monkeypatch.setattr(vs, "find_actions", lambda packages: calls.append(packages) or [])
  r = vs.ensure_venv()
  assert r["cached"] is True and calls == []


# --- uv.lock wheel hashes are enforced by pip via a #sha256= URL fragment ---

def test_pip_spec_appends_lock_hash(vs):
  act = vs.PackageAction("numpy", "", "2.5.3", "https://files.example/numpy-2.5.3.whl", "ab12", "install")
  assert vs._pip_spec(act) == "https://files.example/numpy-2.5.3.whl#sha256=ab12"


def test_pip_spec_without_hash_is_plain_url(vs):
  act = vs.PackageAction("numpy", "", "2.5.3", "https://files.example/numpy-2.5.3.whl", "", "install")
  assert vs._pip_spec(act) == "https://files.example/numpy-2.5.3.whl"


def test_lock_hash_reaches_pip_command(vs, monkeypatch):
  pkgs = vs.parse_lock_packages(LOCK_OPTIONAL_NATIVE)
  assert pkgs["numpy"].wheel_hash == "aa"   # "sha256:" prefix stripped
  act = vs.PackageAction("numpy", "", "2.5.3", pkgs["numpy"].wheel_url, pkgs["numpy"].wheel_hash, "install")
  cmds = []
  def _run(cmd, **kwargs):
    cmds.append(cmd)
    return subprocess.CompletedProcess(cmd, 0, "", "")
  monkeypatch.setattr(vs.subprocess, "run", _run)
  r = vs.install_packages([act])
  assert r["failed"] == []
  pip = [c for c in cmds if vs.VENV_PIP in c]
  assert len(pip) == 1
  assert pip[0][-1] == "https://files.example/numpy-2.5.3-cp312-cp312-manylinux_2_28_aarch64.whl#sha256=aa"
