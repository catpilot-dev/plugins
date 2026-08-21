"""Manifest registration — mapd must declare its slots, service, process and hook."""
import json
import os
import re
import sys

import pytest

PLUGIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _manifest():
  with open(os.path.join(PLUGIN_DIR, 'plugin.json')) as f:
    return json.load(f)


class TestCerealSlots:
  def test_declares_all_three_slots(self):
    slots = _manifest()['cereal']['slots']
    assert set(slots) == {'17', '18', '19'}

  def test_slot_struct_and_event_names(self):
    slots = _manifest()['cereal']['slots']
    assert slots['17']['struct_name'] == 'MapdExtendedOut'
    assert slots['17']['event_field'] == 'mapdExtendedOut'
    assert slots['18']['struct_name'] == 'MapdIn'
    assert slots['18']['event_field'] == 'mapdIn'
    assert slots['19']['struct_name'] == 'MapdOut'
    assert slots['19']['event_field'] == 'mapdOut'

  def test_slot_schema_files_exist(self):
    for num, info in _manifest()['cereal']['slots'].items():
      path = os.path.join(PLUGIN_DIR, info['schema_file'])
      assert os.path.isfile(path), f'slot {num} schema missing: {path}'

  def test_standalone_schema_declared_and_exists(self):
    schema = _manifest()['cereal']['standalone_schema']
    assert os.path.isfile(os.path.join(PLUGIN_DIR, schema))


class TestService:
  def test_mapd_out_registered(self):
    assert 'mapdOut' in _manifest()['services']

  def test_frequency_matches_mapd_loop_rate(self):
    # mapd's main.go sleeps settings.LOOP_DELAY = 50ms, i.e. 20 Hz. SubMaster
    # validity is checked against this number: a mismatch makes
    # sm.valid['mapdOut'] false forever and degrades us to VISION permanently.
    entry = _manifest()['services']['mapdOut']
    assert entry[0] is True          # logged
    assert entry[1] == 20.0          # frequency (Hz)
    assert entry[2] == 20            # decimation


class TestProcess:
  def test_exactly_one_process_declared_binary_is_active(self):
    """mapd is ACTIVE again: it declares its process, so the binary runs.

    This is the same tripwire as before, inverted. It read `processes == []`
    while mapd was dormant on gomsgq v0.1.10's shadow-reader panic; v2.3.1
    carries the fix (mapd fe45d10), so the entry is back. Deactivating mapd
    must stay a deliberate edit to this assertion, never a silent drift — and
    a SECOND process entry is just as much a surprise as zero.
    """
    procs = _manifest()['processes']
    assert len(procs) == 1
    assert procs[0]['name'] == 'mapd'
    assert procs[0]['module'] == 'mapd_runner'
    assert procs[0]['condition'] == 'always_run'

  def test_interface_survives_the_dormant_binary(self):
    """The cereal slots and mapdOut service MUST stay declared while dormant.

    This is why mapd is enforced rather than .disabled: .disabled makes
    custom_capnp.py revert the slots to CustomReservedN and services.py drop
    mapdOut, tearing down the interface we are deliberately keeping warm.
    """
    m = _manifest()
    assert set(m['cereal']['slots']) == {'17', '18', '19'}
    assert m['services']['mapdOut'][0] is True


class TestHooks:
  def test_health_check_registered(self):
    hook = _manifest()['hooks']['device.health_check']
    assert hook['module'] == 'hook'
    assert hook['function'] == 'on_health_check'


class TestVersionPin:
  """The pin is v2.3.1, and it is stated in two files that must never drift.

  MapdVersion is what mapd_manager downloads; MAX_ALLOWED_VERSION is the
  ceiling it refuses to exceed. A default ABOVE the ceiling installs nothing at
  all; a default BELOW it silently runs an older binary than the slot schemas
  were diffed against — which is the exact class of failure the slot files are
  there to prevent. Both are asserted literally AND against each other, so
  bumping one alone fails here rather than on the car.
  """

  @staticmethod
  def _max_allowed_version():
    with open(os.path.join(PLUGIN_DIR, 'mapd_manager.py')) as f:
      src = f.read()
    match = re.search(r'^MAX_ALLOWED_VERSION\s*=\s*"([^"]+)"', src, re.M)
    assert match is not None
    return match.group(1)

  def test_manifest_default_is_the_active_pin(self):
    assert _manifest()['params']['MapdVersion']['default'] == 'v2.3.1'

  def test_manager_max_allowed_is_the_active_pin(self):
    assert self._max_allowed_version() == 'v2.3.1'

  def test_pin_and_manifest_default_never_drift(self):
    assert self._max_allowed_version() == \
        _manifest()['params']['MapdVersion']['default']


class TestDefaults:
  """mapd_defaults.json pins mapd to data-source-only behaviour.

  Every control feature must be off: speedlimitd owns the speed target and the
  lateral-accel budget (the layer contract). mapd supplies road context only.
  """

  @staticmethod
  def _defaults():
    with open(os.path.join(PLUGIN_DIR, 'mapd_defaults.json')) as f:
      return json.load(f)

  def test_settings_version_is_2(self):
    assert self._defaults()['settings_version'] == 2

  def test_all_control_features_disabled(self):
    d = self._defaults()
    assert d['speed_limit_control_enabled'] is False
    assert d['map_curve_speed_control_enabled'] is False
    assert d['vision_curve_speed_control_enabled'] is False
    assert d['external_speed_limit_control_enabled'] is False
    assert d['conditional_speed_limit_control_enabled'] is False

  def test_car_state_stays_shadow(self):
    # Upstream default. Shadow consumes no reader slot; the torn-read panic is
    # contained by plugind respawn plus speedlimitd degrading to vision-only.
    assert self._defaults()['subscriber']['shadow_car_state'] is True


class TestRunnerSettings:
  def test_runner_no_longer_writes_control_settings(self):
    """Control config lives in mapd_defaults.json, not in runner-written MapdSettings.

    Leaving both in place would let a stale MapdSettings silently re-enable
    mapd's control features, since the param layer is applied last.
    """
    with open(os.path.join(PLUGIN_DIR, 'mapd_runner.py')) as f:
      src = f.read()
    for key in ('speed_limit_control_enabled',
                'map_curve_speed_control_enabled',
                'vision_curve_speed_control_enabled',
                'map_curve_target_lat_a',
                'vision_curve_target_lat_a'):
      assert key not in src, f'{key} should now come from mapd_defaults.json'


class _ExecReplacedProcess(Exception):
  """Sentinel raised by the fake execv to model its real contract: on success,
  os.execv replaces the process image and never returns to the caller. Not
  SystemExit — test_gives_up_after_all_delays already asserts on SystemExit
  and reusing it here would blur the two distinct outcomes.
  """


class TestRunnerRetry:
  """plugind respawns dead processes every POLL_INTERVAL (5 s).

  Without an internal backoff, a device with no network would exec the runner
  12x/minute forever, each attempt hitting the GitHub releases API.
  """

  @staticmethod
  def _load_runner(monkeypatch, ensure_results):
    """Import mapd_runner with a stub mapd_manager (the real one needs config)."""
    import importlib
    import types
    calls = []

    fake_manager = types.ModuleType('mapd_manager')

    def ensure_binary():
      calls.append(1)
      idx = len(calls) - 1
      return ensure_results[idx] if idx < len(ensure_results) else False

    fake_manager.ensure_binary = ensure_binary
    fake_manager.MAPD_PATH = '/tmp/fake-mapd'
    monkeypatch.setitem(sys.modules, 'mapd_manager', fake_manager)
    monkeypatch.delitem(sys.modules, 'config', raising=False)  # force ImportError path
    if PLUGIN_DIR not in sys.path:
      sys.path.insert(0, PLUGIN_DIR)
    import mapd_runner
    importlib.reload(mapd_runner)
    monkeypatch.setattr(mapd_runner, 'write_settings_param', lambda: True)
    return mapd_runner, calls

  @staticmethod
  def _fake_execv(execed):
    def execv(p, a):
      execed.append(p)
      raise _ExecReplacedProcess  # os.execv never returns on success
    return execv

  def test_execs_immediately_when_binary_ready(self, monkeypatch):
    runner, calls = self._load_runner(monkeypatch, [True])
    execed = []
    monkeypatch.setattr(runner.os, 'execv', self._fake_execv(execed))
    monkeypatch.setattr(runner.time, 'sleep', lambda s: pytest.fail('should not sleep'))
    with pytest.raises(_ExecReplacedProcess):
      runner.main()
    assert execed == ['/tmp/fake-mapd']
    assert len(calls) == 1

  def test_retries_with_backoff_then_succeeds(self, monkeypatch):
    runner, calls = self._load_runner(monkeypatch, [False, False, True])
    execed, slept = [], []
    monkeypatch.setattr(runner.os, 'execv', self._fake_execv(execed))
    monkeypatch.setattr(runner.time, 'sleep', lambda s: slept.append(s))
    with pytest.raises(_ExecReplacedProcess):
      runner.main()
    assert execed == ['/tmp/fake-mapd']
    assert slept == list(runner.RETRY_DELAYS[:2])   # backoff grows between tries

  def test_gives_up_after_all_delays(self, monkeypatch):
    runner, calls = self._load_runner(monkeypatch, [])
    slept = []
    monkeypatch.setattr(runner.os, 'execv', lambda p, a: pytest.fail('should not exec'))
    monkeypatch.setattr(runner.time, 'sleep', lambda s: slept.append(s))
    with pytest.raises(SystemExit) as exc:
      runner.main()
    assert exc.value.code == 1
    assert slept == list(runner.RETRY_DELAYS)
    assert len(calls) == len(runner.RETRY_DELAYS)

  def test_backoff_is_monotonic_and_bounded(self, monkeypatch):
    runner, _ = self._load_runner(monkeypatch, [True])
    delays = runner.RETRY_DELAYS
    assert list(delays) == sorted(delays)
    # Total in-process backoff stays well under a plugind poll storm but long
    # enough that a booting device with slow DHCP is not thrashing the API.
    assert 60 <= sum(delays) <= 600


class TestSettingsParamWrite:
  """mapd_runner.write_settings_param — mapd_defaults.json → MapdSettings param.

  The custom-defaults FILE path (/data/openpilot/mapd_defaults.json) is fatal
  on mapd v2.3.0: settings.go Default() parses it with gabs (numbers become
  float64) but version-checks against uint64, so any such file panics mapd at
  startup regardless of content. The param path (Load()) compares float64 to
  float64 and is what mapd itself round-trips — so the runner ships our
  declarative defaults through the param, rewritten on every start (which also
  survives openpilot wiping /data/params/d/ on boot).
  """

  @staticmethod
  def _runner_with_config(monkeypatch, tmp_path):
    import importlib
    import types
    fake_config = types.ModuleType('config')
    fake_config.PARAMS_DIR = str(tmp_path / 'params')
    monkeypatch.setitem(sys.modules, 'config', fake_config)
    if PLUGIN_DIR not in sys.path:
      sys.path.insert(0, PLUGIN_DIR)
    import mapd_runner
    importlib.reload(mapd_runner)
    return mapd_runner, fake_config

  def test_writes_defaults_verbatim(self, monkeypatch, tmp_path):
    runner, cfg = self._runner_with_config(monkeypatch, tmp_path)
    assert runner.write_settings_param() is True
    written = json.loads(open(os.path.join(cfg.PARAMS_DIR, 'MapdSettings')).read())
    with open(os.path.join(PLUGIN_DIR, 'mapd_defaults.json')) as f:
      assert written == json.load(f)

  def test_keeps_settings_version_numeric(self, monkeypatch, tmp_path):
    # Load() casts the version via float64 — a string here would fail its
    # comparison and reroute through migrations.
    runner, cfg = self._runner_with_config(monkeypatch, tmp_path)
    runner.write_settings_param()
    written = json.loads(open(os.path.join(cfg.PARAMS_DIR, 'MapdSettings')).read())
    assert written['settings_version'] == 2
    assert isinstance(written['settings_version'], int)

  def test_failure_is_nonfatal(self, monkeypatch, tmp_path):
    # An unwritable PARAMS_DIR (parent is a file) -> returns False, never
    # raises: a settings failure must not block the mapd launch. Deterministic
    # regardless of whether a real `config` module is importable in this run.
    blocker = tmp_path / 'blocker'
    blocker.write_text('not a directory')
    runner, cfg = self._runner_with_config(monkeypatch, tmp_path)
    cfg.PARAMS_DIR = str(blocker / 'params')
    assert runner.write_settings_param() is False


class TestHealthHookDormancy:
  """hook.on_health_check must not cry wolf while mapd is dormant by design.

  mapd is kept installed with its cereal interface warm but declares no
  process, so the binary never launches. A permanent expected warning is worse
  than none: it desensitises the reader, and a real failure after re-activation
  would not stand out. The manifest is therefore the source of truth for what
  "healthy" means, so restoring the process entry re-arms the warning with no
  edit to hook.py.
  """

  @staticmethod
  def _hook(monkeypatch, *, alive, processes):
    import importlib
    if PLUGIN_DIR not in sys.path:
      sys.path.insert(0, PLUGIN_DIR)
    import hook
    importlib.reload(hook)
    monkeypatch.setattr(hook, '_pid_alive', lambda name: alive)
    monkeypatch.setattr(hook, '_declares_process', lambda: bool(processes))
    return hook

  def test_dormant_and_stopped_is_ok_not_a_warning(self, monkeypatch):
    hook = self._hook(monkeypatch, alive=False, processes=False)
    r = hook.on_health_check({})['mapd']
    assert r['status'] == 'ok'
    assert r['dormant'] is True
    assert 'warnings' not in r

  def test_dormant_still_reports_process_alive_honestly(self, monkeypatch):
    # The field keeps its literal meaning so an rlog reader can distinguish
    # "switched off" from "crashed" without consulting the manifest.
    hook = self._hook(monkeypatch, alive=False, processes=False)
    assert hook.on_health_check({})['mapd']['process_alive'] is False

  def test_active_and_stopped_still_warns(self, monkeypatch):
    hook = self._hook(monkeypatch, alive=False, processes=True)
    r = hook.on_health_check({})['mapd']
    assert r['status'] == 'warning'
    assert r['dormant'] is False
    assert r['warnings'] == ['mapd process not running']

  def test_active_and_running_is_ok(self, monkeypatch):
    hook = self._hook(monkeypatch, alive=True, processes=True)
    r = hook.on_health_check({})['mapd']
    assert r['status'] == 'ok'
    assert 'warnings' not in r

  def test_accumulator_is_preserved(self, monkeypatch):
    hook = self._hook(monkeypatch, alive=False, processes=False)
    assert hook.on_health_check({'other': 1})['other'] == 1


class TestHealthHookManifestReading:
  """_declares_process reads the REAL manifest — this is what makes the hook
  self-maintaining across activation/deactivation."""

  @staticmethod
  def _fresh_hook():
    import importlib
    if PLUGIN_DIR not in sys.path:
      sys.path.insert(0, PLUGIN_DIR)
    import hook
    importlib.reload(hook)
    return hook

  def test_tracks_the_shipped_manifest(self):
    hook = self._fresh_hook()
    declared = any(p.get('name') == 'mapd' for p in _manifest().get('processes', []))
    assert hook._declares_process() is declared

  def test_unreadable_manifest_assumes_active(self, monkeypatch):
    # Fail LOUD, not silent: if we cannot tell, the state where a missing
    # binary is a real problem is the one worth defaulting to.
    hook = self._fresh_hook()
    monkeypatch.setattr(hook, '_PLUGIN_DIR', '/nonexistent-plugin-dir')
    assert hook._declares_process() is True
