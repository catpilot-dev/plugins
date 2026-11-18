"""Driving panel: the Pace Lift toggle (spec R11).

test_source_* inspect the source (as test_driving_panel_map_curve does) and run
anywhere. The behavioural tests build the real item list through the panel's
_build_scroller; they need openpilot importable (raylib UI widgets) and skip
without it. Every param read/write is pointed at a pytest tmp dir.
"""
import importlib.util
import os
import sys
from types import SimpleNamespace

import pytest

_UI_MOD = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DRIVING_PANEL = os.path.join(_UI_MOD, 'driving_panel.py')
_PLUGINS_DIR = os.path.dirname(_UI_MOD)

DESCRIPTION = ("In experimental mode, speeds up toward the speed target when the driving model is slower than it. "
               "Strength follows Driving Personality: strongest on Aggressive, off on Relaxed.")


def test_source_toggle_reads_and_writes_param():
  with open(DRIVING_PANEL) as f:
    src = f.read()
  assert "read_plugin_param('pace_lift', 'PaceLiftEnabled') != '0'" in src
  assert "write_plugin_param('pace_lift', 'PaceLiftEnabled'" in src
  assert "_plugin_enabled('pace_lift')" in src
  assert 'callback=self._on_pace_lift' in src
  assert '"Pace Lift"' in src
  assert 'Strength follows Driving Personality: strongest on Aggressive, off on Relaxed.' in src


def test_source_toggle_after_personality():
  with open(DRIVING_PANEL) as f:
    src = f.read()
  assert src.index('items.append(self._personality)') < src.index("_plugin_enabled('pace_lift')") \
    < src.index("_plugin_enabled('lane_centering')")


@pytest.fixture
def panel(tmp_path, monkeypatch):
  pytest.importorskip('openpilot.system.ui.widgets.list_view')
  runtime = tmp_path / 'runtime'
  runtime.mkdir()
  monkeypatch.setenv('PLUGINS_RUNTIME_DIR', str(runtime))
  monkeypatch.setenv('PLUGINS_REPO_DIR', str(tmp_path / 'repo'))
  monkeypatch.setenv('MEDIA_DIR', str(tmp_path / 'media'))
  monkeypatch.setenv('PARAMS_DIR', str(tmp_path / 'params'))
  monkeypatch.syspath_prepend(_PLUGINS_DIR)
  import config
  for name, sub in (('PLUGINS_RUNTIME_DIR', 'runtime'), ('PLUGINS_REPO_DIR', 'repo'),
                    ('MEDIA_DIR', 'media'), ('PARAMS_DIR', 'params')):
    monkeypatch.setattr(config, name, str(tmp_path / sub))

  spec = importlib.util.spec_from_file_location('driving_panel_pace_lift_test', DRIVING_PANEL)
  mod = importlib.util.module_from_spec(spec)
  sys.modules[spec.name] = mod
  try:
    spec.loader.exec_module(mod)
  except ImportError as e:
    sys.modules.pop(spec.name, None)
    pytest.skip(f'openpilot UI not importable: {e}')
  monkeypatch.setattr(mod, 'PLUGINS_DIR', str(runtime))
  monkeypatch.setattr(mod, 'ui_state', SimpleNamespace(CP=None))

  built = {}

  class _Recorder:
    def __init__(self, items, **kw):
      built['items'] = items

  monkeypatch.setattr(mod, 'Scroller', _Recorder)
  # No window in a test: widgets ask the app singleton for fonts at build time.
  monkeypatch.setattr(mod.gui_app, 'font', lambda *a, **k: None)

  class _Params:
    def get(self, *a, **k):
      return 1

    def put(self, *a, **k):
      pass

  def build():
    layout = mod.DrivingLayout.__new__(mod.DrivingLayout)
    layout._params = _Params()
    layout._brand_texture = None
    layout._fingerprint = ''
    layout._build_scroller()
    return layout, built['items']

  yield SimpleNamespace(mod=mod, runtime=runtime, build=build, config=config)
  sys.modules.pop(spec.name, None)


def _install(runtime, plugin='pace_lift', disabled=False, param=None):
  d = runtime / plugin
  (d / 'data').mkdir(parents=True)
  if disabled:
    (d / '.disabled').write_text('')
  if param is not None:
    (d / 'data' / 'PaceLiftEnabled').write_text(param)


def _titles(items):
  return [i.title for i in items]


def test_toggle_shown_when_plugin_enabled(panel):
  _install(panel.runtime)
  layout, items = panel.build()
  assert 'Pace Lift' in _titles(items)
  item = items[_titles(items).index('Pace Lift')]
  assert item.description == DESCRIPTION
  assert layout._pace_lift is item


def test_toggle_hidden_when_plugin_missing_or_disabled(panel):
  _, items = panel.build()
  assert 'Pace Lift' not in _titles(items)
  _install(panel.runtime, disabled=True)
  _, items = panel.build()
  assert 'Pace Lift' not in _titles(items)


def test_toggle_reflects_param(panel):
  _install(panel.runtime, param='0')
  _, items = panel.build()
  assert items[_titles(items).index('Pace Lift')].action_item.get_state() is False
  (panel.runtime / 'pace_lift' / 'data' / 'PaceLiftEnabled').write_text('1')
  _, items = panel.build()
  assert items[_titles(items).index('Pace Lift')].action_item.get_state() is True


def test_toggle_missing_param_is_on(panel):
  _install(panel.runtime)
  _, items = panel.build()
  assert items[_titles(items).index('Pace Lift')].action_item.get_state() is True


def test_toggle_writes_param(panel):
  _install(panel.runtime)
  layout, _ = panel.build()
  path = panel.runtime / 'pace_lift' / 'data' / 'PaceLiftEnabled'
  layout._on_pace_lift(False)
  assert path.read_text() == '0'
  layout._on_pace_lift(True)
  assert path.read_text() == '1'


def test_toggle_after_personality(panel):
  _install(panel.runtime)
  _install(panel.runtime, plugin='lane_centering')
  layout, items = panel.build()
  assert items[0] is layout._personality
  assert items[1] is layout._pace_lift
  assert 'Lane Centering in Turns' in _titles(items)[2:]
