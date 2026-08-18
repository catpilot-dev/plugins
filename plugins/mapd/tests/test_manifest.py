"""Manifest registration — mapd must declare its slots, service, process and hook."""
import json
import os
import re

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
