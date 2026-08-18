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
  def test_mapd_process_declared(self):
    procs = {p['name']: p for p in _manifest()['processes']}
    assert 'mapd' in procs
    assert procs['mapd']['module'] == 'mapd_runner'


class TestHooks:
  def test_health_check_registered(self):
    hook = _manifest()['hooks']['device.health_check']
    assert hook['module'] == 'hook'
    assert hook['function'] == 'on_health_check'


class TestVersionPin:
  def test_manifest_default_is_v230(self):
    assert _manifest()['params']['MapdVersion']['default'] == 'v2.3.0'

  def test_manager_max_allowed_is_v230(self):
    with open(os.path.join(PLUGIN_DIR, 'mapd_manager.py')) as f:
      src = f.read()
    match = re.search(r'^MAX_ALLOWED_VERSION\s*=\s*"([^"]+)"', src, re.M)
    assert match is not None
    assert match.group(1) == 'v2.3.0'


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
    if PLUGIN_DIR not in sys.path:
      sys.path.insert(0, PLUGIN_DIR)
    import mapd_runner
    importlib.reload(mapd_runner)
    return mapd_runner, calls

  def test_execs_immediately_when_binary_ready(self, monkeypatch):
    runner, calls = self._load_runner(monkeypatch, [True])
    execed = []
    monkeypatch.setattr(runner.os, 'execv', lambda p, a: execed.append(p))
    monkeypatch.setattr(runner.time, 'sleep', lambda s: pytest.fail('should not sleep'))
    runner.main()
    assert execed == ['/tmp/fake-mapd']
    assert len(calls) == 1

  def test_retries_with_backoff_then_succeeds(self, monkeypatch):
    runner, calls = self._load_runner(monkeypatch, [False, False, True])
    execed, slept = [], []
    monkeypatch.setattr(runner.os, 'execv', lambda p, a: execed.append(p))
    monkeypatch.setattr(runner.time, 'sleep', lambda s: slept.append(s))
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
