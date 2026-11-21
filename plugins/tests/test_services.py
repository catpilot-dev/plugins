"""plugins/services.py — injecting plugin services into cereal/services.py."""
import os
import sys

PLUGINS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PLUGINS_DIR not in sys.path:
  sys.path.insert(0, PLUGINS_DIR)

import services  # noqa: E402

_STOCK = '''_services: dict[str, tuple] = {
  "carState": (True, 100., 10),
  "modelV2": (True, 20., None, QueueSize.BIG),
}
'''


def _write(tmp_path, text):
  p = tmp_path / 'services.py'
  p.write_text(text)
  return p


def _sl(path):
  """Evaluate the written dict so the test checks Python's reading, not ours."""
  class QueueSize:
    BIG = 10 * 1024 * 1024
  ns = {'QueueSize': QueueSize}
  exec(path.read_text(), ns)
  return ns['_services']


class TestInject:
  def test_adds_a_missing_service(self, tmp_path):
    p = _write(tmp_path, _STOCK)
    assert services.inject_services(str(p), {'mapdOut': [True, 20.0, 20, 2097152]}) == 1
    assert _sl(p)['mapdOut'] == (True, 20.0, 20, 2097152)

  def test_current_entry_is_a_noop(self, tmp_path):
    p = _write(tmp_path, _STOCK)
    services.inject_services(str(p), {'mapdOut': [True, 20.0, 20, 2097152]})
    before = p.read_text()
    assert services.inject_services(str(p), {'mapdOut': [True, 20.0, 20, 2097152]}) == 0
    assert p.read_text() == before

  def test_rewrites_a_stale_entry_from_an_older_manifest(self, tmp_path):
    """REGRESSION: services.py is never reset, so the pre-v2.4.0 3-tuple mapdOut
    (default SMALL queue, 250 KB) would survive every reinstall. mapd creates the
    queue at 2 MB and both sides ftruncate the shared file, so the stale size is
    the suspected cause of the v2.3.1 publisher SIGBUS."""
    p = _write(tmp_path, _STOCK.replace(
        '}\n', '  "mapdOut": (True, 20.0, 20),\n}\n'))
    assert services.inject_services(str(p), {'mapdOut': [True, 20.0, 20, 2097152]}) == 1
    assert _sl(p)['mapdOut'] == (True, 20.0, 20, 2097152)
    assert p.read_text().count('"mapdOut"') == 1

  def test_stock_services_are_untouched(self, tmp_path):
    p = _write(tmp_path, _STOCK)
    services.inject_services(str(p), {'mapdOut': [True, 20.0, 20, 2097152]})
    sl = _sl(p)
    assert sl['carState'] == (True, 100., 10)
    assert sl['modelV2'] == (True, 20., None, 10 * 1024 * 1024)

  def test_rewrites_every_duplicate_not_just_the_first(self, tmp_path):
    """The C3 carries mapdOut twice: install.sh's line, then the
    "# plugin services (auto-generated)" block from catpilot's boot-time
    builder.py. A dict literal keeps the LAST duplicate, so rewriting only
    the first would leave the effective entry at 250 KB."""
    p = _write(tmp_path, _STOCK.replace('}\n',
        '  "mapdOut": (True, 20.0, 20),\n'
        '  # plugin services (auto-generated)\n'
        '  "mapdOut": (True, 20.0, 20),\n}\n'))
    assert services.inject_services(str(p), {'mapdOut': [True, 20.0, 20, 2097152]}) == 2
    assert _sl(p)['mapdOut'] == (True, 20.0, 20, 2097152)
    assert '(True, 20.0, 20),' not in p.read_text()


class TestRemove:
  """remove_services() drops stale single-line entries (used by the mapd migration)."""

  _STALE = _STOCK.replace('}\n',
      '  "mapdOut": (True, 20.0, 20, 2097152),\n'
      '  "mapdExtendedOut": (False, 1.0, None, 2097152),\n'
      '  "pluginBusLog": (True, 100.0, None),\n'
      '  # plugin services (auto-generated)\n'
      '  "mapdOut": (True, 20.0, 20, 2097152),\n'
      '  "mapdExtendedOut": (False, 1.0, None, 2097152),\n'
      '  "pluginBusLog": (True, 100.0, None),\n'
      '}\n')

  def test_removes_every_entry_for_the_names(self, tmp_path):
    p = _write(tmp_path, self._STALE)
    assert services.remove_services(str(p), ['mapdOut', 'mapdExtendedOut']) == 4
    sl = _sl(p)
    assert 'mapdOut' not in sl and 'mapdExtendedOut' not in sl
    assert '"mapd' not in p.read_text()

  def test_other_entries_and_layout_are_kept(self, tmp_path):
    p = _write(tmp_path, self._STALE)
    services.remove_services(str(p), ['mapdOut', 'mapdExtendedOut'])
    want = _STOCK.replace('}\n',
        '  "pluginBusLog": (True, 100.0, None),\n'
        '  # plugin services (auto-generated)\n'
        '  "pluginBusLog": (True, 100.0, None),\n'
        '}\n')
    assert p.read_text() == want
    sl = _sl(p)
    assert sl['carState'] == (True, 100., 10)
    assert sl['pluginBusLog'] == (True, 100.0, None)

  def test_absent_names_are_a_noop_and_do_not_rewrite(self, tmp_path):
    p = _write(tmp_path, _STOCK)
    p.chmod(0o444)  # any write attempt would raise PermissionError
    try:
      assert services.remove_services(str(p), ['mapdOut', 'mapdExtendedOut']) == 0
    finally:
      p.chmod(0o644)
    assert p.read_text() == _STOCK

  def test_prefix_names_and_other_forms_are_untouched(self, tmp_path):
    text = _STOCK.replace('}\n',
        '  "mapdOutX": (True, 20.0, 20),\n'
        '  # "mapdOut": (True, 20.0, 20),\n'
        '  "mapdOut": (True,\n'
        '              20.0, 20),\n'
        '}\n')
    p = _write(tmp_path, text)
    assert services.remove_services(str(p), ['mapdOut']) == 0
    assert p.read_text() == text

  def test_cli_remove(self, tmp_path):
    import subprocess
    p = _write(tmp_path, self._STALE)
    script = os.path.join(PLUGINS_DIR, 'services.py')
    r = subprocess.run([sys.executable, '-B', script, '--remove', str(p), 'mapdOut', 'mapdExtendedOut'],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    assert 'Removed 4' in r.stdout
    assert '"mapd' not in p.read_text()
    r = subprocess.run([sys.executable, '-B', script, '--remove', str(p), 'mapdOut', 'mapdExtendedOut'],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    assert 'Removed' not in r.stdout

  def test_cli_remove_needs_a_name(self, tmp_path):
    import subprocess
    p = _write(tmp_path, self._STALE)
    r = subprocess.run([sys.executable, '-B', os.path.join(PLUGINS_DIR, 'services.py'), '--remove', str(p)],
                       capture_output=True, text=True)
    assert r.returncode != 0
    assert p.read_text() == self._STALE

  def test_cli_inject_still_works(self, tmp_path):
    """The positional `services.py <services_py> <plugins_dir>` form is unchanged."""
    import json
    import subprocess
    p = _write(tmp_path, _STOCK)
    plug = tmp_path / 'runtime' / 'foo'
    plug.mkdir(parents=True)
    (plug / 'plugin.json').write_text(json.dumps({'id': 'foo', 'services': {'fooOut': [True, 20.0, None]}}))
    r = subprocess.run([sys.executable, '-B', os.path.join(PLUGINS_DIR, 'services.py'), str(p), str(tmp_path / 'runtime')],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    assert _sl(p)['fooOut'] == (True, 20.0, None)
