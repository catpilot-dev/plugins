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
