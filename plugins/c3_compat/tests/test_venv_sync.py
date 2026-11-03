"""venv_sync: the git-sourced commaai/dependencies install must stand down when
the deployed uv.lock already ships the comma-deps-* wheels (openpilot 0.11.2).
Both provide the same module names (capnproto, ffmpeg, zstd, ...); installing
the git packages over the wheels would replace the libraries scons links."""
import importlib
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


LOCK_0112 = '''
[[package]]
name = "comma-deps-capnproto"
version = "1.0.1.post103"
source = { registry = "https://pypi.org/simple" }

[[package]]
name = "numpy"
version = "2.5.3"
source = { registry = "https://pypi.org/simple" }
'''

LOCK_0111 = '''
[[package]]
name = "numpy"
version = "2.4.2"
source = { registry = "https://pypi.org/simple" }

[[package]]
name = "raylib"
version = "5.5.0.4"
source = { registry = "https://pypi.org/simple" }
'''


def test_lock_ships_comma_deps(vs):
  assert vs.lock_ships_comma_deps(LOCK_0112) is True
  assert vs.lock_ships_comma_deps(LOCK_0111) is False


def test_native_deps_skipped_when_lock_ships_wheels(vs, tmp_path):
  (tmp_path / "uv.lock").write_text(LOCK_0112)
  vs.LOCAL_LOCK = str(tmp_path / "uv.lock")
  r = vs.ensure_native_deps(dry_run=True)
  assert r["skipped"] is True
  assert r.get("reason") == "comma-deps wheels in uv.lock"
  assert r["installed"] == [] and r["failed"] == []


def test_native_deps_still_run_on_old_lock(vs, tmp_path, monkeypatch):
  (tmp_path / "uv.lock").write_text(LOCK_0111)
  vs.LOCAL_LOCK = str(tmp_path / "uv.lock")
  vs.NATIVE_DEPS_CACHE = str(tmp_path / ".native_deps_installed")   # no cache -> real check
  def _missing(name):
    raise ImportError(name)
  monkeypatch.setattr(importlib, "import_module", _missing)   # ensure_native_deps imports importlib locally
  r = vs.ensure_native_deps(dry_run=True)
  assert r["skipped"] is False
  assert set(r["installed"]) == set(vs.COMMAAI_DEPS)


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
