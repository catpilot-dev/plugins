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


def _plan_mv(k_arc, v=25.0, state='off', direction='none', n=41, t_max=10.0, raw=0.0):
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
    meta=SimpleNamespace(laneChangeState=state, laneChangeDirection=direction),
    action=SimpleNamespace(desiredCurvature=raw))


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
  assert [i.title for i in items] == ['Arc Smooth', 'Model Centring']
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


# ---- Model Centring (2026-11-20) ---------------------------------------------


def _anchor_mod():
  return register._anchor_module()


def _centring_files(data_dir, on='1', tau=None):
  if on is not None:
    (data_dir / 'LaneKeepModelCentring').write_text(on)
  if tau is not None:
    (data_dir / 'LaneKeepCentringTau').write_text(tau)


def _hook(k_raw, k_arc_plan=0.0, v=25.0, state='off', direction='none', lc=False, curvature=None):
  """One engaged hook call (curvature == action.desiredCurvature unless given)."""
  mv = _plan_mv(k_arc_plan, v=v, state=state, direction=direction, raw=k_raw)
  return register.on_curvature_correction(k_raw if curvature is None else curvature, mv, v, lc, 0.45)


def _lp_ref(xs, tc=4.0, dt=0.01):
  """Reference 1st-order filter for a hook run that engages on the first call:
  that tick outputs exactly 0 (engage), the filter charges from the second."""
  a = 1.0 - math.exp(-dt / tc)
  s, out = 0.0, [0.0]
  for x in xs[1:]:
    s += a * (x - s)
    out.append(s)
  return out


def test_centring_off_is_bit_identical_in_both_arc_modes(data_dir, monkeypatch):
  tel = _capture(monkeypatch)
  base = _run(_calls())
  for off in ('0', '', 'true', ' 1', '1\n', '2'):
    register._anchor = None
    _centring_files(data_dir, on=off)
    assert _run(_calls()) == base
  # Arc Smooth OFF: raw as today, centring ignored even when ON
  register._anchor = None
  (data_dir / 'LaneKeepArcSmoothTest').write_text('0')
  _centring_files(data_dir, on='1')
  calls = _calls()
  assert _run(calls) == [k for k, _, _, _ in calls]
  assert all(t['centring_on'] is False and t['kappa_centring'] == 0.0 for t in tel[-len(calls):])


def test_centring_on_with_raw_equal_arc_is_arc(data_dir, monkeypatch):
  tel = _capture(monkeypatch)
  base = _run(_calls())
  register._anchor = None
  _centring_files(data_dir)
  tel.clear()
  assert _run(_calls()) == base          # _calls never engages (curvature != action)
  assert all(t['kappa_centring'] == 0.0 and t['centring_on'] is True for t in tel)
  # engaged with raw == arc (plan arc 0.003 vs raw 0.003 once the Arc has settled)
  register._anchor = None
  tel.clear()
  outs = [_hook(0.003, 0.003) for _ in range(800)]
  assert max(abs(t['kappa_centring']) for t in tel) < 1e-9
  assert outs[-1] == pytest.approx(0.003, abs=1e-9)


def test_centring_lp_step_matches_reference_filter(data_dir, monkeypatch):
  tel = _capture(monkeypatch)
  _centring_files(data_dir)
  n = 1500
  for _ in range(n):
    _hook(3e-4)
  ref = _lp_ref([3e-4] * n)
  assert [t['kappa_centring'] for t in tel] == pytest.approx(ref, rel=0, abs=1e-9)
  assert tel[-1]['kappa_centring'] == pytest.approx(3e-4 * (1 - math.exp(-3.7475)), abs=1e-9)


def test_centring_lp_sine_matches_reference_filter(data_dir, monkeypatch):
  tel = _capture(monkeypatch)
  _centring_files(data_dir)
  xs = [2e-4 * math.sin(2 * math.pi * 0.1 * i * 0.01) for i in range(2000)]
  for x in xs:
    _hook(x)
  assert [t['kappa_centring'] for t in tel] == pytest.approx(_lp_ref(xs), rel=0, abs=1e-9)
  assert any(abs(t['kappa_centring']) > 1e-5 for t in tel)


def test_centring_tau_file(data_dir, monkeypatch):
  tel = _capture(monkeypatch)
  _centring_files(data_dir, tau='2.0')
  for _ in range(300):
    _hook(3e-4)
  assert [t['kappa_centring'] for t in tel] == pytest.approx(_lp_ref([3e-4] * 300, tc=2.0), rel=0, abs=1e-9)


@pytest.mark.parametrize('val,expect', [('junk', 4.0), ('nan', 4.0), ('inf', 4.0), ('', 4.0),
                                        ('0.2', 1.0), ('-3', 1.0), ('100', 20.0), ('7.5', 7.5)])
def test_centring_tau_parse(data_dir, val, expect):
  (data_dir / 'LaneKeepCentringTau').write_text(val)
  assert register._centring_tau() == expect
  assert register._centring_tau.__name__          # file-only: no param declared (manifest test)


def test_centring_missing_tau_file_is_default(data_dir):
  assert register._centring_tau() == 4.0


def test_centring_state_clamp_and_speed_gate_units():
  A = _anchor_mod()
  t = A.CentringTerm(1.0)
  v = 25.0
  for _ in range(5000):
    out = t.step(1e-2, 0.0, v, True, False)
  assert t.s == pytest.approx(0.3 / v ** 2)
  assert out == pytest.approx(0.3 / v ** 2)
  t = A.CentringTerm(1.0)
  for _ in range(5000):
    t.step(-1e-2, 0.0, 20.0, True, False)
  assert t.s == pytest.approx(-0.3 / 20.0 ** 2)


def test_centring_speed_gate():
  A = _anchor_mod()
  assert A.centring_speed_gate(25 / 3.6) == 0.0
  assert A.centring_speed_gate(35 / 3.6) == pytest.approx(0.5)
  assert A.centring_speed_gate(45 / 3.6) == 1.0
  t = A.CentringTerm(1.0)
  outs = []
  for v_kmh in (25, 35, 45):
    t = A.CentringTerm(1.0)
    for _ in range(3000):
      out = t.step(4e-4, 0.0, v_kmh / 3.6, True, False)
    outs.append(out / t.s)
  assert outs == pytest.approx([0.0, 0.5, 1.0])
  # the state itself still charges below the gate (only the output is gated)
  t = A.CentringTerm(1.0)
  for _ in range(500):
    t.step(4e-4, 0.0, 6.0, True, False)
  assert t.s != 0.0


def test_centring_bend_gate_decays_and_straight_charges():
  A = _anchor_mod()
  t = A.CentringTerm(1.0)
  v = 20.0
  for _ in range(500):
    t.step(1e-4, 0.0, v, True, False)
  s0 = t.s
  assert s0 > 0
  bend = 0.31 / v ** 2                   # |arc| v^2 just above 0.3 m/s^2
  for _ in range(50):
    t.step(1e-4, bend, v, True, False)   # 0.5 s of bend
  assert t.s == pytest.approx(s0 * math.exp(-1.0), rel=1e-9)
  s1 = t.s
  t.step(0.29 / v ** 2 + 1e-4, 0.29 / v ** 2, v, True, False)   # below the gate: charges again
  assert t.s > s1 * math.exp(-0.01 / 0.5)
  # never charges while in a bend
  t = A.CentringTerm(1.0)
  for _ in range(500):
    t.step(5e-4, bend, v, True, False)
  assert t.s == 0.0


def test_centring_engage_transient(data_dir, monkeypatch):
  tel = _capture(monkeypatch)
  _centring_files(data_dir)
  # not engaged: handed the measured curvature (!= action): output == Arc, term 0
  base = []
  for _ in range(300):
    out = _hook(3e-4, curvature=1e-3)
    base.append(out)
  assert all(t['kappa_centring'] == 0.0 for t in tel)
  assert register._centring.s == 0.0
  # first engaged tick == Arc, then the term builds from 0
  out = _hook(3e-4)
  assert tel[-1]['kappa_centring'] == 0.0 and out == tel[-1]['kappa_arc_ref']
  _hook(3e-4)
  assert tel[-1]['kappa_centring'] == pytest.approx(3e-4 * (1 - math.exp(-0.0025)), abs=1e-12)
  for _ in range(400):
    _hook(3e-4)
  assert tel[-1]['kappa_centring'] > 1e-5
  # latActive drops: state reset at once
  _hook(3e-4, curvature=1e-3)
  assert tel[-1]['kappa_centring'] == 0.0 and register._centring.s == 0.0


def test_centring_lane_change_onset_decays_and_resets(data_dir, monkeypatch):
  tel = _capture(monkeypatch)
  _centring_files(data_dir)
  for _ in range(800):
    _hook(3e-4)
  s0 = register._centring.s
  assert s0 > 1e-5
  # preLaneChange (lane_changing True) alone does not touch it
  for _ in range(50):
    _hook(3e-4, state='preLaneChange', direction='right', lc=True)
  assert register._centring.s > s0
  s1 = register._centring.s
  for _ in range(50):                         # 0.5 s in laneChangeStarting: one tau
    _hook(3e-4, state='laneChangeStarting', direction='right', lc=True)
  assert register._centring.s == pytest.approx(s1 * math.exp(-1.0), rel=1e-9)
  for _ in range(20):
    _hook(3e-4, state='laneChangeFinishing', direction='right', lc=True)
  assert register._centring.s > 0
  _hook(3e-4)                                  # change over: reset, then 1 tick of charging
  a = 1 - math.exp(-0.0025)
  assert register._centring.s == pytest.approx(a * (3e-4 - tel[-1]['kappa_arc_ref']), abs=1e-12)


@pytest.mark.parametrize('bad', [float('nan'), float('inf')])
def test_centring_non_finite_raw_falls_back_to_arc(data_dir, monkeypatch, bad):
  tel = _capture(monkeypatch)
  _centring_files(data_dir)
  for _ in range(300):
    _hook(3e-4)
  mv = _plan_mv(0.0, raw=bad)
  out = register.on_curvature_correction(bad, mv, 25.0, False, 0.45)
  assert tel[-1]['kappa_centring'] == 0.0 and register._centring.s == 0.0
  assert out == tel[-1]['kappa_arc_ref']
  # missing action: not engaged, no crash
  mv2 = _plan_mv(0.0)
  del mv2.action
  out = register.on_curvature_correction(3e-4, mv2, 25.0, False, 0.45)
  assert out == tel[-1]['kappa_arc_ref']


def test_centring_telemetry_all_modes(data_dir, monkeypatch):
  tel = _capture(monkeypatch)
  _hook(3e-4)
  assert tel[-1]['centring_on'] is False and tel[-1]['kappa_centring'] == 0.0
  register._anchor = None
  _centring_files(data_dir)
  _hook(3e-4)
  assert tel[-1]['centring_on'] is True and 'kappa_centring' in tel[-1]
  register._anchor = None
  (data_dir / 'LaneKeepArcSmoothTest').write_text('0')
  _hook(3e-4)
  assert tel[-1]['centring_on'] is False and tel[-1]['arc_bypass'] is True
  assert tel[-1]['kappa_ref'] == 3e-4
  assert register.HOOK_READY is True


def test_centring_applied_term_is_in_kappa_ref(data_dir, monkeypatch):
  tel = _capture(monkeypatch)
  _centring_files(data_dir)
  for _ in range(800):
    out = _hook(3e-4)
  assert out == tel[-1]['kappa_arc_ref'] + tel[-1]['kappa_centring']
  assert tel[-1]['kappa_ref'] == out
  assert tel[-1]['kappa_centring'] > 1e-5


def test_centring_ui_rows(data_dir, fake_toggle):
  items = register.on_vehicle_settings([], SimpleNamespace(brand='bmw'))
  assert [i.title for i in items] == ['Arc Smooth', 'Model Centring']
  assert items[1].state is False
  d = items[1].description.lower()
  assert 'lane-centring' in d and 'arc smooth' in d and 'next drive' in d and 'supervised' in d
  (data_dir / 'LaneKeepModelCentring').write_text('1')
  fake_toggle.clear()
  items = register.on_vehicle_settings([], SimpleNamespace(brand='bmw'))
  assert items[1].state is True
  items[1].callback(False)
  assert (data_dir / 'LaneKeepModelCentring').read_text() == '0'
  items[1].callback(True)
  assert (data_dir / 'LaneKeepModelCentring').read_text() == '1'
  fake_toggle.clear()
  assert register.on_vehicle_settings([], SimpleNamespace(brand='toyota')) == []
  assert fake_toggle == []


def test_replay_helper_uses_the_hook_law(data_dir, monkeypatch):
  import importlib.util as ilu
  spec = ilu.spec_from_file_location('lk_replay', os.path.join(_PLUGIN_DIR, 'tools', 'replay_centring.py'))
  m = ilu.module_from_spec(spec)
  spec.loader.exec_module(m)
  tel = _capture(monkeypatch)
  _centring_files(data_dir)
  raws = [3e-4 + 1e-4 * math.sin(i * 0.01) for i in range(600)]
  for r in raws:
    _hook(r)
  series = m.centring_series(raws, [0.0] * 600, [25.0] * 600, [True] * 600, [False] * 600)
  assert series == [t['kappa_centring'] for t in tel]
