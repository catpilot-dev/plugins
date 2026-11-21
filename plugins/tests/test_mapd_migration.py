"""R6 end-to-end test for install.d/mapd_migration.sh (plan 2026-11-21-remove-mapd.md).

The mapd plugin was removed from the repo, but devices still carry the old
install: /data/plugins-runtime/mapd (enforced) and cereal slots 17-19 patched to
mapd's structs. Deleting that dir outright breaks the schema at the next boot:
catpilot's builder.py strips every appended standalone type and re-appends only
those of enabled plugins, while slots 17-19 still reference mapd's types
(`custom.capnp:73: Not defined: MapdDownloadProgress`).

Each case builds that old install state in a sandbox (scratch copies of the
sibling catpilot cereal, a runtime dir with the real repo plugins plus the
baseline mapd manifest + schemas from fixtures/old_mapd), runs ONLY
install.d/mapd_migration.sh in bash against the sandbox, then builder.py's three
boot patches, then loads log.capnp with pycapnp in a subprocess.

SAFETY: install.sh itself is never run or sourced (it auto-detects ~/openpilot and
defaults to the real /data/plugins-runtime). Every path the test writes is under
pytest's tmp_path; _sandbox() asserts that, and that no .git dir is involved.
"""
import os
import shutil
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
CAT = REPO.parent / 'catpilot'
CAT_CEREAL = CAT / 'openpilot' / 'cereal'
BUILDER = CAT / 'openpilot' / 'selfdrive' / 'plugins' / 'builder.py'
OPENDBC_CAR = CAT / 'opendbc_repo' / 'opendbc' / 'car'
MIGRATION = REPO / 'install.d' / 'mapd_migration.sh'
FIXTURE = Path(__file__).resolve().parent / 'fixtures' / 'old_mapd'
PY = sys.executable

if not (CAT_CEREAL / 'log.capnp').is_file() or not BUILDER.is_file() or not (OPENDBC_CAR / 'car.capnp').is_file():
  pytest.skip(f'sibling catpilot checkout not found at {CAT}', allow_module_level=True)
pytest.importorskip('capnp')
if shutil.which('bash') is None:
  pytest.skip('bash not available', allow_module_level=True)

# builder.py, with its openpilot imports stubbed and its paths pointed at the
# sandbox. Runs in a subprocess so the stubs never leak into this test process.
_BUILDER_RUNNER = textwrap.dedent('''
  import importlib.util, os, sys, types
  builder_py, cereal, runtime = sys.argv[1:4]
  for m in ('openpilot', 'openpilot.common', 'openpilot.common.basedir', 'openpilot.common.swaglog'):
    sys.modules[m] = types.ModuleType(m)
  sys.modules['openpilot.common.basedir'].BASEDIR = os.path.dirname(cereal)
  class _Log:
    def __getattr__(self, k):
      return lambda *a, **kw: print('[builder]', k, *a)
  sys.modules['openpilot.common.swaglog'].cloudlog = _Log()
  spec = importlib.util.spec_from_file_location('builder', builder_py)
  b = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(b)
  b.CEREAL_DIR = cereal
  b.CUSTOM_CAPNP = os.path.join(cereal, 'custom.capnp')
  b.LOG_CAPNP = os.path.join(cereal, 'log.capnp')
  b.SERVICES_PY = os.path.join(cereal, 'services.py')
  b.PLUGINS_DIR = runtime
  b.BUILD_HASH_FILE = os.path.join(os.path.dirname(cereal), 'plugin_build_hash')
  dirs = b._get_enabled_plugins()
  b._patch_custom_capnp(dirs)
  b._patch_log_capnp(dirs)
  b._patch_services(dirs)
''')

_COMPILE_CHECK = textwrap.dedent('''
  import importlib.util, sys
  import capnp
  log_capnp, car_dir, cereal, services_py = sys.argv[1:5]
  capnp.remove_import_hook()
  capnp.load(log_capnp, imports=[car_dir, cereal])
  s = importlib.util.spec_from_file_location('svc', services_py)
  m = importlib.util.module_from_spec(s)
  s.loader.exec_module(m)
  print('MAPD_SERVICES=' + ','.join(sorted(k for k in m.SERVICE_LIST if 'mapd' in k.lower())))
''')

# Mirrors install.sh's overlay_cereal (DRY_RUN early return, python | while-read
# logging pipelines, pipefail from the caller), pointed at the sandbox.
_HARNESS_HEAD = textwrap.dedent('''
  set -euo pipefail
  {helpers}
  source "$MIGRATION"
  cereal_step() {
    $DRY_RUN && return
    "$PY" -B "$SCRIPT_DIR/plugins/custom_capnp.py" "$OP_PKG/cereal" "$PLUGINS_DEST" 2>&1 | while IFS= read -r line; do
      echo "[install] $line"
    done
    "$PY" -B "$SCRIPT_DIR/plugins/services.py" "$OP_PKG/cereal/services.py" "$PLUGINS_DEST" 2>&1 | while IFS= read -r line; do
      echo "[install] $line"
    done
  }
  # Verify fails: a cereal step that "succeeds" without reverting anything.
  cereal_skips_revert() { echo "[install] (cereal step that skips the revert)"; }
  # Cereal step exits non-zero immediately.
  cereal_exit1() { exit 1; }
  # Cereal step fails midway: custom_capnp.py has run, services.py has not.
  cereal_midway() {
    "$PY" -B "$SCRIPT_DIR/plugins/custom_capnp.py" "$OP_PKG/cereal" "$PLUGINS_DEST"
    false
    "$PY" -B "$SCRIPT_DIR/plugins/services.py" "$OP_PKG/cereal/services.py" "$PLUGINS_DEST"
  }
  # The main shell is killed while the migration is pending (EXIT-trap backstop).
  cereal_killed() { kill -TERM $$; sleep 1; }
''')

_INSTALL_HELPERS = textwrap.dedent('''
  log()  { echo "[install] $*"; }
  warn() { echo "[install] WARNING: $*" >&2; }
''')


def _sandbox(tmp_path: Path) -> dict:
  """Old mapd install: scratch cereal + runtime, cereal step + boot builder already run."""
  sb = tmp_path / 'sb'
  cereal = sb / 'cereal'
  runtime = sb / 'runtime'
  home = tmp_path / 'home'
  for p in (sb, cereal, runtime, home):
    assert str(p.resolve()).startswith(str(tmp_path.resolve()) + os.sep)
  assert not any((p / '.git').exists() for p in (sb, *sb.parents))

  cereal.mkdir(parents=True)
  home.mkdir()
  for f in ('custom.capnp', 'log.capnp', 'deprecated.capnp', 'services.py'):
    shutil.copy(CAT_CEREAL / f, cereal / f)
  shutil.copytree(CAT_CEREAL / 'include', cereal / 'include')

  # The real repo plugins (manifest + schemas: all the cereal step and builder
  # read), plus the baseline mapd plugin as the device still carries it.
  runtime.mkdir()
  for src in sorted((REPO / 'plugins').iterdir()):
    if src.name == 'mapd' or not (src / 'plugin.json').is_file():
      continue
    (runtime / src.name).mkdir()
    shutil.copy(src / 'plugin.json', runtime / src.name / 'plugin.json')
    if (src / 'cereal').is_dir():
      shutil.copytree(src / 'cereal', runtime / src.name / 'cereal')
  shutil.copytree(FIXTURE, runtime / 'mapd')
  (runtime / 'mapd' / '.enforced').touch()

  env = {
    'PATH': os.environ.get('PATH', '/usr/bin:/bin'),
    'HOME': str(home),
    'LANG': 'C.UTF-8',
    'PY': PY,
    'MIGRATION': str(MIGRATION),
    'SCRIPT_DIR': str(REPO),
    'OPENPILOT_ROOT': str(sb),
    'OP_PKG': str(sb),
    'PLUGINS_DEST': str(runtime),
    'PLUGINS_RUNTIME_DIR': str(runtime),
    'PYTHONDONTWRITEBYTECODE': '1',
    'DRY_RUN': 'false',
  }
  s = {'tmp': tmp_path, 'sb': sb, 'cereal': cereal, 'runtime': runtime, 'mapd': runtime / 'mapd', 'env': env}

  # Old install state: the old install.sh cereal step, then the boot builder.
  r = _run_bash(s, 'cereal_step')
  assert r.returncode == 0, r.stdout + r.stderr
  _builder(s)
  ok, out = _compile(s)
  assert ok, out
  assert 'MAPD_SERVICES=mapdExtendedOut,mapdOut' in out, out
  assert 'struct MapdOut @' in (cereal / 'custom.capnp').read_text()
  assert 'mapdOut @145 :Custom.MapdOut;' in (cereal / 'log.capnp').read_text()
  return s


def _run_bash(s, body: str, dry_run=False, helpers=True) -> subprocess.CompletedProcess:
  script = _HARNESS_HEAD.replace('{helpers}', _INSTALL_HELPERS if helpers else '') + body + '\n'
  env = dict(s['env'], DRY_RUN='true' if dry_run else 'false')
  return subprocess.run(['bash', '-c', script], env=env, cwd=s['sb'], capture_output=True, text=True, timeout=120)


def _migrate(s, cereal_cmd='cereal_step', **kw) -> subprocess.CompletedProcess:
  r = _run_bash(s, f'mapd_migrate {cereal_cmd}\necho MIGRATE-RETURNED', **kw)
  assert r.returncode == 0, r.stdout + r.stderr
  assert 'MIGRATE-RETURNED' in r.stdout, r.stdout + r.stderr
  return r


def _builder(s):
  r = subprocess.run([PY, '-B', '-c', _BUILDER_RUNNER, str(BUILDER), str(s['cereal']), str(s['runtime'])],
                     capture_output=True, text=True, cwd=s['sb'], timeout=60)
  assert r.returncode == 0, r.stdout + r.stderr


def _compile(s):
  c = s['cereal']
  r = subprocess.run([PY, '-B', '-c', _COMPILE_CHECK, str(c / 'log.capnp'), str(OPENDBC_CAR), str(c), str(c / 'services.py')],
                     capture_output=True, text=True, cwd=s['sb'], timeout=120)
  return r.returncode == 0, r.stdout + r.stderr[-2000:]


def _snapshot(root: Path) -> dict:
  return {str(p.relative_to(root)): (p.read_bytes() if p.is_file() else None)
          for p in sorted(root.rglob('*'))}


def _assert_mapd_restored(s):
  """Failure paths: the device boots exactly as before, with the stale mapd copy."""
  mapd = s['mapd']
  assert (mapd / 'plugin.json').is_file()
  assert not (mapd / '.disabled').exists()
  assert (mapd / '.enforced').is_file()
  _builder(s)
  ok, out = _compile(s)
  assert ok, out
  assert 'MAPD_SERVICES=mapdExtendedOut,mapdOut' in out, out


class TestNaiveDeleteBreaksTheSchema:
  def test_deleting_the_dir_outright_does_not_compile(self, tmp_path):
    """Why the migration exists: the reproduced failure, kept as a guard on the harness."""
    s = _sandbox(tmp_path)
    shutil.rmtree(s['mapd'])
    assert _run_bash(s, 'cereal_step').returncode == 0
    _builder(s)
    ok, out = _compile(s)
    assert not ok
    assert 'Not defined: Mapd' in out


class TestMigrationSuccess:
  def test_migrates_and_compiles(self, tmp_path):
    s = _sandbox(tmp_path)
    r = _migrate(s)
    assert not s['mapd'].exists()
    svc = (s['cereal'] / 'services.py').read_text()
    assert '"mapdOut"' not in svc and '"mapdExtendedOut"' not in svc
    _builder(s)
    ok, out = _compile(s)
    assert ok, out
    assert 'MAPD_SERVICES=\n' in out, out
    custom = (s['cereal'] / 'custom.capnp').read_text()
    assert 'Mapd' not in custom
    log = (s['cereal'] / 'log.capnp').read_text()
    for n, ev in ((17, 143), (18, 144), (19, 145)):
      assert f'struct CustomReserved{n} @' in custom
      assert f'customReserved{n} @{ev} :Custom.CustomReserved{n};' in log
    # bus_logger's slot 1 is untouched by the migration
    assert 'pluginBusLog' in svc
    assert 'WARNING' not in r.stderr, r.stderr

  def test_second_run_is_a_noop(self, tmp_path):
    s = _sandbox(tmp_path)
    _migrate(s)
    _builder(s)
    before = _snapshot(s['sb'])
    r = _migrate(s)
    assert _snapshot(s['sb']) == before
    assert 'mapd' not in (r.stdout + r.stderr).lower()
    _builder(s)
    ok, out = _compile(s)
    assert ok, out

  def test_works_without_install_sh_helpers(self, tmp_path):
    """Sourced alone (no log/warn defined), the file falls back to its own."""
    s = _sandbox(tmp_path)
    _migrate(s, helpers=False)
    assert not s['mapd'].exists()
    _builder(s)
    assert _compile(s)[0]

  def test_dir_already_gone_still_removes_stale_services(self, tmp_path):
    """An earlier run deleted the dir but stopped before step (c)."""
    s = _sandbox(tmp_path)
    (s['mapd'] / '.disabled').touch()
    assert _run_bash(s, 'cereal_step').returncode == 0
    shutil.rmtree(s['mapd'])
    assert '"mapdOut"' in (s['cereal'] / 'services.py').read_text()
    _migrate(s)
    svc = (s['cereal'] / 'services.py').read_text()
    assert '"mapdOut"' not in svc and '"mapdExtendedOut"' not in svc
    _builder(s)
    ok, out = _compile(s)
    assert ok, out
    assert 'MAPD_SERVICES=\n' in out

  def test_binary_and_tiles_are_not_touched(self):
    """R5f: the migration never references the media dir (binary, backups, tiles)."""
    text = MIGRATION.read_text()
    assert '/data' not in text
    assert 'media' not in text.lower()
    assert 'mapd_defaults' not in text


class TestDryRun:
  def test_dry_run_changes_nothing(self, tmp_path):
    s = _sandbox(tmp_path)
    before = _snapshot(s['sb'])
    r = _migrate(s, dry_run=True)
    assert _snapshot(s['sb']) == before
    assert 'mapd' in r.stdout


class TestFailurePaths:
  """Each ends with mapd enabled + .enforced and the schema compiling after the builder."""

  @pytest.mark.parametrize('cereal_cmd', ['cereal_skips_revert', 'cereal_exit1', 'cereal_midway'])
  def test_failure_restores_mapd(self, tmp_path, cereal_cmd):
    s = _sandbox(tmp_path)
    r = _migrate(s, cereal_cmd)
    assert 'WARNING' in r.stderr, r.stdout + r.stderr
    # never on the undo branch: the services lines stay with the restored plugin
    assert '"mapdOut"' in (s['cereal'] / 'services.py').read_text()
    _assert_mapd_restored(s)

  def test_exit_trap_backstop_restores_mapd_and_chains_the_previous_trap(self, tmp_path):
    s = _sandbox(tmp_path)
    r = _run_bash(s, 'trap "echo PREVIOUS-EXIT-TRAP" EXIT\nmapd_migrate cereal_killed\necho NOT-REACHED')
    assert r.returncode != 0
    assert 'NOT-REACHED' not in r.stdout
    assert 'PREVIOUS-EXIT-TRAP' in r.stdout, r.stdout + r.stderr
    _assert_mapd_restored(s)

  def test_exit_trap_is_restored_after_the_migration(self, tmp_path):
    s = _sandbox(tmp_path)
    r = _run_bash(s, 'trap "echo PREVIOUS-EXIT-TRAP" EXIT\nmapd_migrate cereal_step\n'
                     'echo "TRAP-AFTER=[$(trap -p EXIT)]"')
    assert r.returncode == 0, r.stdout + r.stderr
    assert "TRAP-AFTER=[trap -- 'echo PREVIOUS-EXIT-TRAP' EXIT]" in r.stdout, r.stdout
    assert r.stdout.rstrip().endswith('PREVIOUS-EXIT-TRAP')
    s2 = _sandbox(tmp_path / 'b')
    r = _run_bash(s2, 'mapd_migrate cereal_step\necho "TRAP-AFTER=[$(trap -p EXIT)]"')
    assert 'TRAP-AFTER=[]' in r.stdout, r.stdout
