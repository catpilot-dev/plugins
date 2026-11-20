import importlib.util, math, os, sys
from types import SimpleNamespace
import pytest

# Load register (and the anchor it depends on) by explicit path under unique
# module names — do NOT insert the plugin dir on sys.path. Two plugins both have
# a top-level `register.py`; a bare `import register` after a sys.path insert
# shadows the other plugin's module when the whole suite runs together (breaks
# bmw's tests). The runtime avoids this via canonical registry module names.
_PLUGIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _load(name, unique):
  spec = importlib.util.spec_from_file_location(unique, os.path.join(_PLUGIN_DIR, name + '.py'))
  m = importlib.util.module_from_spec(spec)
  sys.modules[unique] = m
  spec.loader.exec_module(m)
  return m


# register.py loads its sibling anchor by explicit path (no sys.path use), so we
# only need to load register itself under a unique name.
register = _load('register', 'lk_register')


@pytest.fixture(autouse=True)
def _reset_register():
  # Module-level lazy singletons persist across tests; reset before each.
  register._anchor = None
  register._pub = None
  yield


@pytest.fixture
def data_dir(tmp_path, monkeypatch):
  d = tmp_path / 'data'
  d.mkdir()
  register._anchor_module()   # cache the sibling module before _PLUGIN_DIR is patched
  monkeypatch.setattr(register, '_PLUGIN_DIR', str(tmp_path))
  return d


def test_hook_ready_is_set_after_a_full_load():
  """The BMW controller patches its lateral law only on this sentinel."""
  assert register.HOOK_READY is True


def test_passthrough_returns_input_curvature():
  mv = SimpleNamespace(laneLines=[], laneLineProbs=[])
  out = register.on_curvature_correction(0.0123, mv, 25.0, False, lat_delay=0.45)
  assert out == 0.0123


def test_hook_survives_pub_failure(data_dir, monkeypatch):
  monkeypatch.setattr(register, '_PLUGIN_DIR', str(data_dir.parent))
  # force telemetry publish to raise — the control path must still return the reference
  monkeypatch.setattr(register, '_publish', lambda telem: (_ for _ in ()).throw(RuntimeError('no bus')))
  mv = SimpleNamespace(laneLines=[], laneLineProbs=[])
  out = None
  for _ in range(200):
    out = register.on_curvature_correction(0.01, mv, 25.0, False, lat_delay=0.45)
  assert out == pytest.approx(0.01)   # no plan: the low-passed model curvature


def test_hook_accepts_lat_delay(data_dir, monkeypatch):
  """controlsd passes lat_delay; the hook keeps the parameter and ignores it."""
  monkeypatch.setattr(register, '_publish', lambda telem: None)
  mv = SimpleNamespace(laneLines=[], laneLineProbs=[])
  assert register.on_curvature_correction(0.0, mv, 25.0, False, lat_delay=0.55) == 0.0


def test_load_config_kappa_filter_tau(data_dir):
  cfg = register._load_config()
  assert cfg.kappa_filter_tau == 0.15         # default
  (data_dir / 'LaneKeepKappaFilterTau').write_text('0.45')
  cfg2 = register._load_config()
  assert cfg2.kappa_filter_tau == 0.45


def test_load_config_arc_params(data_dir):
  cfg = register._load_config()
  assert cfg.arc_horizon == 0.0    # default: auto horizon
  (data_dir / 'LaneKeepArcHorizon').write_text('2.0')
  cfg = register._load_config()
  assert cfg.arc_horizon == 2.0


def test_reference_is_always_arc_and_never_position_corrected(data_dir, monkeypatch):
  """The BMW controller is the Direct law unconditionally, so this plugin
  always serves the Arc reference and never the position correction: the old
  switch files are ignored and the Direct cross-plugin machinery is gone."""
  monkeypatch.setattr(register, '_publish', lambda telem: None)
  (data_dir / 'LaneKeepEnable').write_text('1')
  (data_dir / 'LaneKeepArcSmooth').write_text('0')
  for _ in range(150):
    register.on_curvature_correction(0.0, SimpleNamespace(laneLines=[], laneLineProbs=[]), 20.0, False, 0.9)
  assert register._anchor.cfg.arc_horizon == 0.0
  assert not hasattr(register, '_direct_tracking_on')
  assert not hasattr(register._anchor.cfg, 'enable')


# ---- Arc Smooth toggle (temporary, 2026-11-20) -----------------------------

def _mv():
  return SimpleNamespace(laneLines=[], laneLineProbs=[])


def _capture(monkeypatch):
  out = []
  monkeypatch.setattr(register, '_publish', lambda telem: out.append(dict(telem)))
  return out


def _plan_mv(k_arc, v=25.0, state='off', direction='none', n=41, t_max=10.0):
  """Realistic modelV2 stub (as test_anchor's _lc_mv): ego lane lines, a plan on
  a circle of curvature k_arc, and lane-change meta."""
  ts = [t_max * i / (n - 1) for i in range(n)]
  if k_arc == 0.0:
    xs, ys = [v * t for t in ts], [0.0] * n
  else:
    xs = [math.sin(v * t * k_arc) / k_arc for t in ts]
    ys = [(1.0 - math.cos(v * t * k_arc)) / k_arc for t in ts]
  return SimpleNamespace(
    laneLines=[SimpleNamespace(y=[0.0]), SimpleNamespace(y=[-1.75]),
               SimpleNamespace(y=[1.75]), SimpleNamespace(y=[0.0])],
    laneLineProbs=[0.0, 1.0, 1.0, 0.0],
    position=SimpleNamespace(t=ts, x=xs, y=ys),
    meta=SimpleNamespace(laneChangeState=state, laneChangeDirection=direction))


def _calls():
  """(curvature, model_v2, v_ego, lane_changing): settled lane keeping on a
  0.003 plan with a wobbling raw curvature, a right lane change (raw points
  away first -> onset hold, then toward -> hand-over to raw), lane keeping
  again, and a crawl."""
  seq = []
  for i in range(60):
    seq.append((0.002 * ((-1) ** (i // 10)), _plan_mv(0.003), 25.0, False))
  for _ in range(3):
    seq.append((-0.0006, _plan_mv(0.003, state='laneChangeStarting', direction='right'), 25.0, True))
  for _ in range(5):
    seq.append((0.001, _plan_mv(0.003, state='laneChangeStarting', direction='right'), 25.0, True))
  for _ in range(10):
    seq.append((-0.0005, _plan_mv(0.003, state='laneChangeFinishing', direction='right'), 25.0, True))
  for _ in range(40):
    seq.append((-0.003, _plan_mv(-0.002), 25.0, False))
  for _ in range(30):
    seq.append((0.01, _plan_mv(0.01, v=0.5), 0.5, False))   # crawl
  return seq


def _run(calls):
  return [register.on_curvature_correction(k, mv, v, lc, 0.45) for k, mv, v, lc in calls]


def _assert_arc_exercised(tel):
  assert any(t['arc_on'] is True for t in tel)
  assert any(t['lc_hold'] is True for t in tel)
  assert any(t['arc_on'] is False and t['lc_hold'] is False for t in tel)


def test_arc_smooth_defaults_on(data_dir, monkeypatch):
  tel = _capture(monkeypatch)
  register.on_curvature_correction(0.01, _mv(), 25.0, False, 0.45)
  assert tel[-1]['arc_bypass'] is False


@pytest.mark.parametrize('bad', ['1', 'false', '', 'off', ' 0 ', '00', '0\n'])
def test_arc_smooth_non_zero_values_are_on(data_dir, monkeypatch, bad):
  tel = _capture(monkeypatch)
  (data_dir / 'LaneKeepArcSmoothTest').write_text(bad)
  register.on_curvature_correction(0.01, _mv(), 25.0, False, 0.45)
  assert tel[-1]['arc_bypass'] is False


def test_bypass_returns_input_every_call(data_dir, monkeypatch):
  tel = _capture(monkeypatch)
  (data_dir / 'LaneKeepArcSmoothTest').write_text('0')
  calls = _calls()
  outs = _run(calls)
  assert outs == [k for k, _, _, _ in calls]
  assert all(t['arc_bypass'] is True for t in tel)
  _assert_arc_exercised(tel)           # the Arc really ran underneath
  assert register.HOOK_READY is True

def test_bypass_still_runs_arc_and_publishes_arc_output(data_dir, monkeypatch):
  tel = _capture(monkeypatch)
  (data_dir / 'LaneKeepArcSmoothTest').write_text('0')
  _run(_calls())
  # reference actually returned = raw input; Arc output kept separately
  assert all(t['kappa_ref'] == t['kappa_in'] for t in tel)
  assert any(t['kappa_arc_ref'] != t['kappa_in'] for t in tel)
  assert any(t['arc_on'] is True and t['kappa_arc_ref'] != t['kappa_in'] for t in tel)

def test_default_mode_equals_lane_anchor_update(data_dir, monkeypatch):
  tel = _capture(monkeypatch)
  ref = register._anchor_module().LaneAnchor(register._load_config())
  for k, mv, v, lc in _calls():
    out = register.on_curvature_correction(k, mv, v, lc, 0.45)
    exp, exp_t = ref.update(k, mv, v, lc)
    assert out == exp
    assert tel[-1]['kappa_arc_ref'] == exp
    assert tel[-1]['arc_bypass'] is False
    for key, val in exp_t.items():
      assert tel[-1][key] == val
  _assert_arc_exercised(tel)

def test_bypass_mode_arc_state_matches_default_mode(data_dir, monkeypatch):
  """The Arc runs identically in bypass: kappa_arc_ref equals the default-mode output."""
  tel = _capture(monkeypatch)
  default_out = _run(_calls())
  register._anchor = None
  tel.clear()
  (data_dir / 'LaneKeepArcSmoothTest').write_text('0')
  _run(_calls())
  assert [t['kappa_arc_ref'] for t in tel] == default_out
  _assert_arc_exercised(tel)

def test_arc_smooth_read_once_per_process(data_dir, monkeypatch):
  tel = _capture(monkeypatch)
  register.on_curvature_correction(0.01, _mv(), 25.0, False, 0.45)
  (data_dir / 'LaneKeepArcSmoothTest').write_text('0')
  register.on_curvature_correction(0.01, _mv(), 25.0, False, 0.45)
  assert tel[-1]['arc_bypass'] is False
  register._anchor = None                      # new process
  assert register.on_curvature_correction(0.012, _mv(), 25.0, False, 0.45) == 0.012
  assert tel[-1]['arc_bypass'] is True
  (data_dir / 'LaneKeepArcSmoothTest').write_text('1')
  assert register.on_curvature_correction(0.013, _mv(), 25.0, False, 0.45) == 0.013   # still bypass
  assert tel[-1]['arc_bypass'] is True


def test_telemetry_failure_does_not_break_bypass(data_dir, monkeypatch):
  monkeypatch.setattr(register, '_publish', lambda telem: (_ for _ in ()).throw(RuntimeError('no bus')))
  (data_dir / 'LaneKeepArcSmoothTest').write_text('0')
  assert register.on_curvature_correction(0.02, _mv(), 25.0, False, 0.45) == 0.02


def test_arc_smooth_helper_parse(data_dir):
  f = data_dir / 'LaneKeepArcSmoothTest'
  assert register._arc_smooth_enabled() is True            # missing
  f.write_text('0')
  assert register._arc_smooth_enabled() is False
  for v in ('1', '', 'false', 'off', ' 0 '):
    f.write_text(v)
    assert register._arc_smooth_enabled() is True


# ---- UI hook ----------------------------------------------------------------

@pytest.fixture
def fake_toggle(monkeypatch):
  rows = []

  def toggle_item(title, description, initial_state=False, callback=None, enabled=True):
    row = SimpleNamespace(title=title, description=description, state=initial_state, callback=callback)
    rows.append(row)
    return row

  mods = {}
  for name in ('openpilot', 'openpilot.system', 'openpilot.system.ui', 'openpilot.system.ui.widgets'):
    mods[name] = sys.modules.get(name) or type(sys)(name)
  lv = type(sys)('openpilot.system.ui.widgets.list_view')
  lv.toggle_item = toggle_item
  mods['openpilot.system.ui.widgets.list_view'] = lv
  for k, m in mods.items():
    monkeypatch.setitem(sys.modules, k, m)
  return rows


def test_ui_hook_bmw_adds_one_row_default_on(data_dir, fake_toggle):
  items = register.on_vehicle_settings([], SimpleNamespace(brand='bmw'))
  assert len(items) == 1 and items[0].title == 'Arc Smooth'
  assert items[0].state is True
  d = items[0].description.lower()
  assert 'raw model curvature' in d and 'next drive' in d and 'ignition' in d and 'supervised' in d


def test_ui_hook_initial_state_from_file(data_dir, fake_toggle):
  (data_dir / 'LaneKeepArcSmoothTest').write_text('0')
  items = register.on_vehicle_settings([], SimpleNamespace(brand='bmw'))
  assert items[0].state is False


def test_ui_hook_callback_writes_file(tmp_path, monkeypatch, fake_toggle):
  monkeypatch.setattr(register, '_PLUGIN_DIR', str(tmp_path))   # no data/ dir yet
  items = register.on_vehicle_settings([], SimpleNamespace(brand='bmw'))
  items[0].callback(False)
  assert (tmp_path / 'data' / 'LaneKeepArcSmoothTest').read_text() == '0'
  items[0].callback(True)
  assert (tmp_path / 'data' / 'LaneKeepArcSmoothTest').read_text() == '1'


def test_ui_hook_non_bmw_adds_no_row(data_dir, fake_toggle):
  prior = ['x']
  assert register.on_vehicle_settings(prior, SimpleNamespace(brand='toyota')) == ['x']
  assert fake_toggle == []


def test_stale_old_arc_smooth_key_is_ignored(data_dir, monkeypatch):
  """A leftover data/LaneKeepArcSmooth='0' from the old switch must not bypass."""
  tel = _capture(monkeypatch)
  (data_dir / 'LaneKeepArcSmooth').write_text('0')
  ref = register._anchor_module().LaneAnchor(register._load_config())
  for k, mv, v, lc in _calls():
    out = register.on_curvature_correction(k, mv, v, lc, 0.45)
    assert out == ref.update(k, mv, v, lc)[0]
    assert tel[-1]['arc_bypass'] is False
  _assert_arc_exercised(tel)


def test_manifest_declares_no_params_and_both_hooks():
  """No manifest params: connect-on-device would list any param with `desc` as an
  ungated web toggle on every brand. Missing file already means ON."""
  import json
  with open(os.path.join(_PLUGIN_DIR, 'plugin.json')) as f:
    m = json.load(f)
  assert m.get('params', {}) == {}
  assert set(m['hooks']) == {'controls.curvature_correction', 'ui.vehicle_settings'}
