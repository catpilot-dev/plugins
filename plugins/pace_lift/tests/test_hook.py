import importlib.util
import json
import math
import os
import sys
from types import SimpleNamespace

import numpy as np
import pytest

_PLUGIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_PLUGINS_DIR = os.path.dirname(_PLUGIN_DIR)

T = [i * 0.5 for i in range(13)]


@pytest.fixture
def cfg(tmp_path, monkeypatch):
  """Point every param read at a pytest tmp dir (the default is the real /data)."""
  runtime = tmp_path / 'runtime'
  monkeypatch.setenv('PLUGINS_RUNTIME_DIR', str(runtime))
  monkeypatch.syspath_prepend(_PLUGINS_DIR)
  import config
  assert hasattr(config, 'read_plugin_param')
  monkeypatch.setattr(config, 'PLUGINS_RUNTIME_DIR', str(runtime))
  data = runtime / 'pace_lift' / 'data'
  data.mkdir(parents=True)
  return data


@pytest.fixture
def hook(cfg, monkeypatch):
  spec = importlib.util.spec_from_file_location('pace_lift_hook_test', os.path.join(_PLUGIN_DIR, 'hook.py'))
  m = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(m)
  clock = SimpleNamespace(t=1000.0)
  monkeypatch.setattr(m, '_clock', lambda: clock.t)
  m._clock_state = clock
  m._reset_state()
  m._pub = FakePub()
  m._pub_ok = True
  return m


class FakePub:
  def __init__(self, fail=False):
    self.sent = []
    self.fail = fail

  def send(self, d):
    if self.fail:
      raise RuntimeError('bus down')
    self.sent.append(d)


def make_sm(enabled=True, exp=True, personality='aggressive', gas=False, brake=False,
            standstill=False, should_stop=False, lead=None, plan_v=None, yaw=None):
  lead = lead or {}
  return {
    'selfdriveState': SimpleNamespace(enabled=enabled, experimentalMode=exp, personality=personality),
    'carState': SimpleNamespace(gasPressed=gas, brakePressed=brake, standstill=standstill),
    'modelV2': SimpleNamespace(
      action=SimpleNamespace(shouldStop=should_stop),
      velocity=SimpleNamespace(t=T, x=plan_v if plan_v is not None else [22.0] * len(T)),
      orientationRate=SimpleNamespace(z=yaw if yaw is not None else [0.0] * len(T))),
    'radarState': SimpleNamespace(leadOne=SimpleNamespace(
      present=lead.get('present', False), dRel=lead.get('d', 0.0), vRel=lead.get('vrel', 0.0))),
  }


def tick(hook, dt=0.05, a=0.1, v=22.0, vc=27.0, ac=1.0, sm=None):
  hook._clock_state.t += dt
  return hook.on_e2e_accel(a, v, vc, ac, sm if sm is not None else make_sm())


def build_lift(hook, n=40, **kw):
  for _ in range(n):
    out = tick(hook, **kw)
  return out


def last_gate(hook):
  return hook._pub.sent[-1]['gate']


def test_rise_rate_limited(hook):
  for _ in range(10):
    tick(hook)
  assert hook._lift <= 0.125 + 1e-9
  assert hook._lift > 0.05


def test_rise_reaches_full(hook):
  build_lift(hook, n=60)
  assert math.isclose(hook._lift, 0.40)
  assert last_gate(hook) == 'active'


def test_fall_rate_limited(hook):
  build_lift(hook)
  assert math.isclose(hook._lift, 0.40)
  curve = make_sm(yaw=[0.07] * len(T))
  tick(hook, sm=curve)
  assert math.isclose(hook._lift, 0.35)
  assert last_gate(hook) == 'curve'
  for _ in range(2):
    tick(hook, sm=curve)
  assert math.isclose(hook._lift, 0.25)


@pytest.mark.parametrize('case', ['disabled', 'off_disengaged', 'off_chill', 'gas', 'brake', 'standstill', 'stop'])
def test_hard_resets(hook, cfg, case):
  build_lift(hook)
  assert hook._lift > 0.3
  sm = make_sm()
  expect = case
  if case == 'disabled':
    (cfg / 'PaceLiftEnabled').write_text('0')
    hook._clock_state.t += 1.5   # past the param cache age
  elif case == 'off_disengaged':
    sm, expect = make_sm(enabled=False), 'off'
  elif case == 'off_chill':
    sm, expect = make_sm(exp=False), 'off'
  elif case == 'gas':
    sm = make_sm(gas=True)
  elif case == 'brake':
    sm = make_sm(brake=True)
  elif case == 'standstill':
    sm = make_sm(standstill=True)
  elif case == 'stop':
    sm = make_sm(should_stop=True)
  out = tick(hook, sm=sm)
  assert out == 0.1
  assert hook._lift == 0.0
  assert last_gate(hook) == expect


def test_nonfinite_a_cruise_is_no_model(hook):
  build_lift(hook)
  out = tick(hook, ac=float('nan'))
  assert out == 0.1 and hook._lift == 0.0
  assert last_gate(hook) == 'no_model'


def test_ask_decel_same_tick(hook):
  build_lift(hook)
  assert math.isclose(hook._lift, 0.40)
  out = tick(hook, a=-0.5)
  assert out == -0.5
  assert hook._lift == 0.0
  assert last_gate(hook) == 'ask_decel'


def test_ask_cap_applied(hook):
  build_lift(hook)
  out = tick(hook, a=-0.05)
  assert out <= -0.05 + 0.20 + 1e-9
  assert hook._lift <= 0.20 + 1e-9
  # recovery to 0.40 goes through the rise rate: at least 0.8 s
  t = 0.0
  while hook._lift < 0.40 - 1e-9:
    tick(hook, a=0.1)
    t += 0.05
    assert t < 5.0
  assert t >= 0.8 - 1e-9


def test_lead_cut_in_and_hold(hook):
  build_lift(hook)
  assert math.isclose(hook._lift, 0.40)
  cut_in = make_sm(lead={'present': True, 'd': 30.0, 'vrel': 0.0})   # threshold 44.5
  prev = hook._lift
  tick(hook, sm=cut_in)
  assert last_gate(hook) == 'lead_gap'
  assert prev - hook._lift <= 0.05 + 1e-9
  # lead leaves: gate is held for 2.0 s, lift keeps falling to 0 and stays there
  clear = make_sm()
  for _ in range(38):
    tick(hook, sm=clear)
    assert last_gate(hook) == 'lead_gap'
    assert hook._lift <= prev
  assert hook._lift == 0.0
  # after the 2.0 s hold the lift rebuilds at the rise rate
  for _ in range(4):
    tick(hook, sm=clear)
  assert last_gate(hook) == 'active'
  assert 0.0 < hook._lift <= 0.10


def test_lead_closing_hold(hook):
  build_lift(hook)
  tick(hook, sm=make_sm(lead={'present': True, 'd': 90.0, 'vrel': -1.0}))
  assert last_gate(hook) == 'lead_closing'
  tick(hook, sm=make_sm())
  assert last_gate(hook) == 'lead_closing'


def _soft_sm(gate):
  return {
    'plan_decel': make_sm(plan_v=[22.0, 22.0, 22.0, 19.0] + [22.0] * (len(T) - 4)),
    'curve': make_sm(yaw=[0.07] * len(T)),
    'lead_gap': make_sm(lead={'present': True, 'd': 30.0, 'vrel': 0.0}),
    'lead_closing': make_sm(lead={'present': True, 'd': 90.0, 'vrel': -1.0}),
  }[gate]


@pytest.mark.parametrize('gate', ['plan_decel', 'curve', 'lead_gap', 'lead_closing'])
def test_soft_gate_hold_two_seconds(hook, gate):
  assert hook.SOFT_HOLD_S == 2.0
  build_lift(hook)
  tick(hook, sm=_soft_sm(gate))
  assert last_gate(hook) == gate
  clear = make_sm()
  for _ in range(38):   # 1.9 s after the trip: still held, target 0
    tick(hook, sm=clear)
    assert last_gate(hook) == gate
    assert hook._pub.sent[-1]['target'] == 0.0
  for _ in range(4):    # past 2.0 s: released
    tick(hook, sm=clear)
  assert last_gate(hook) == 'active'
  assert hook._pub.sent[-1]['target'] > 0.0


@pytest.mark.parametrize('case', ['gas', 'brake', 'standstill', 'stop', 'ask_decel', 'disabled'])
def test_hard_reset_during_hold_is_instant(hook, cfg, case):
  build_lift(hook)
  tick(hook, sm=_soft_sm('curve'))
  assert last_gate(hook) == 'curve'
  a, sm = 0.1, make_sm()
  if case == 'gas':
    sm = make_sm(gas=True)
  elif case == 'brake':
    sm = make_sm(brake=True)
  elif case == 'standstill':
    sm = make_sm(standstill=True)
  elif case == 'stop':
    sm = make_sm(should_stop=True)
  elif case == 'ask_decel':
    a = -0.5
  elif case == 'disabled':
    (cfg / 'PaceLiftEnabled').write_text('0')
    hook._clock_state.t += 1.5
  out = tick(hook, a=a, sm=sm)
  assert out == a
  assert hook._lift == 0.0
  assert last_gate(hook) == case


def _no_hold(hook, monkeypatch):
  monkeypatch.setattr(hook, 'SOFT_HOLD_S', 0.0)


def test_lead_gap_hysteresis(hook, monkeypatch):
  _no_hold(hook, monkeypatch)
  lead = lambda d: make_sm(lead={'present': True, 'd': d, 'vrel': 0.0})   # aggressive, v 22
  tick(hook, sm=lead(50.0))
  assert last_gate(hook) == 'active'          # not tripped: 50 > 44.5
  tick(hook, sm=lead(44.0))
  assert last_gate(hook) == 'lead_gap'        # trips below 44.5
  tick(hook, sm=lead(50.0))
  assert last_gate(hook) == 'lead_gap'        # latched: clears only at 27.5 + 6 + 0.8*22 = 51.1
  tick(hook, sm=lead(51.0))
  assert last_gate(hook) == 'lead_gap'
  tick(hook, sm=lead(51.2))
  assert last_gate(hook) == 'active'
  tick(hook, sm=lead(50.0))
  assert last_gate(hook) == 'active'          # latch is gone


def test_lead_closing_hysteresis(hook, monkeypatch):
  _no_hold(hook, monkeypatch)
  lead = lambda vr: make_sm(lead={'present': True, 'd': 90.0, 'vrel': vr})
  tick(hook, sm=lead(-0.3))
  assert last_gate(hook) == 'active'
  tick(hook, sm=lead(-0.6))
  assert last_gate(hook) == 'lead_closing'
  tick(hook, sm=lead(-0.3))
  assert last_gate(hook) == 'lead_closing'    # latched until vRel >= -0.2
  tick(hook, sm=lead(-0.1))
  assert last_gate(hook) == 'active'
  tick(hook, sm=lead(-0.3))
  assert last_gate(hook) == 'active'


def test_latch_reset_when_lead_disappears(hook, monkeypatch):
  _no_hold(hook, monkeypatch)
  lead = lambda d, vr=0.0: make_sm(lead={'present': True, 'd': d, 'vrel': vr})
  tick(hook, sm=lead(44.0))
  assert last_gate(hook) == 'lead_gap'
  tick(hook, sm=make_sm())                    # lead gone: latch resets
  tick(hook, sm=lead(50.0))
  assert last_gate(hook) == 'active'
  tick(hook, sm=lead(90.0, -0.6))
  assert last_gate(hook) == 'lead_closing'
  tick(hook, sm=make_sm())
  tick(hook, sm=lead(90.0, -0.3))
  assert last_gate(hook) == 'active'


def test_output_bounded_by_a_cruise(hook):
  out = build_lift(hook, n=60, a=0.1, ac=0.3)
  assert math.isclose(hook._lift, 0.40)
  assert math.isclose(out, 0.3)


def test_cruise_below_ask_passthrough(hook):
  out = build_lift(hook, n=60, a=0.1, ac=0.0)
  assert out == 0.1
  out = build_lift(hook, n=60, a=0.1, ac=-0.5)
  assert out == 0.1
  # speed target below v_ego: no lift at all
  hook._reset_state()
  out = build_lift(hook, n=60, a=0.1, v=22.0, vc=20.0)
  assert out == 0.1 and hook._lift == 0.0
  assert last_gate(hook) == 'none_needed'


def test_personality_mapping(hook):
  tick(hook, sm=make_sm(personality='standard'))
  assert hook._pub.sent[-1]['strength'] == pytest.approx(0.20)
  tick(hook, sm=make_sm(personality='aggressive'))
  assert hook._pub.sent[-1]['strength'] == pytest.approx(0.40)
  tick(hook, sm=make_sm(personality='relaxed'))
  assert hook._pub.sent[-1]['strength'] == 0.0
  tick(hook, sm=make_sm(personality='bogus'))
  assert hook._pub.sent[-1]['strength'] == 0.0
  # capnp-style enum text
  tick(hook, sm=make_sm(personality='LongitudinalPersonality.standard'))
  assert hook._pub.sent[-1]['strength'] == pytest.approx(0.20)
  # relaxed / unknown: output is the ask
  assert build_lift(hook, n=60, sm=make_sm(personality='relaxed')) == 0.1


def test_strength_params_override(hook, cfg):
  (cfg / 'PaceLiftStandard').write_text('0.3')
  tick(hook, sm=make_sm(personality='standard'))
  assert hook._pub.sent[-1]['strength'] == pytest.approx(0.30)


def test_disabled_param_passthrough(hook, cfg):
  (cfg / 'PaceLiftEnabled').write_text('0')
  out = build_lift(hook, n=60)
  assert out == 0.1 and hook._lift == 0.0
  assert last_gate(hook) == 'disabled'


@pytest.mark.parametrize('raw,expected', [
  ('abc', 0.40), ('', 0.40), ('nan', 0.40), ('inf', 0.40), ('9', 0.6), ('-1', 0.0), ('0.25', 0.25)])
def test_bad_params_fall_back(hook, cfg, raw, expected):
  (cfg / 'PaceLiftAggressive').write_text(raw)
  tick(hook)
  assert hook._pub.sent[-1]['strength'] == pytest.approx(expected)


def test_enabled_param_semantics(hook, cfg):
  (cfg / 'PaceLiftEnabled').write_text('1')
  tick(hook)
  assert last_gate(hook) != 'disabled'


def test_param_cache_refresh(hook, cfg):
  tick(hook)
  assert hook._pub.sent[-1]['strength'] == pytest.approx(0.40)
  (cfg / 'PaceLiftAggressive').write_text('0.1')
  tick(hook, dt=0.5)   # cache is 0.55 s old: not re-read
  assert hook._pub.sent[-1]['strength'] == pytest.approx(0.40)
  tick(hook, dt=0.5)   # 1.05 s old: re-read
  assert hook._pub.sent[-1]['strength'] == pytest.approx(0.10)


def test_param_reader_error_disables(hook, monkeypatch):
  def boom(*a, **k):
    raise RuntimeError('reader broke')
  monkeypatch.setattr(hook, 'read_plugin_param', boom)
  hook._params_t = None
  out = build_lift(hook, n=10)
  assert out == 0.1 and hook._lift == 0.0
  assert last_gate(hook) == 'disabled'


def test_missing_param_files_use_defaults(hook):
  tick(hook)
  assert last_gate(hook) != 'disabled'
  assert hook._pub.sent[-1]['strength'] == pytest.approx(0.40)


def test_dt_clamped(hook):
  tick(hook)
  tick(hook, dt=10.0)           # clamped to 0.2 s: rise <= 0.05
  assert hook._lift <= 0.05 + 1e-9
  hook._clock_state.t -= 100.0  # a clock going backwards: dt 0
  before = hook._lift
  hook.on_e2e_accel(0.1, 22.0, 27.0, 1.0, make_sm())
  assert hook._lift == before


def test_publish_failure_never_raises(hook):
  hook._pub = FakePub(fail=True)
  out = tick(hook)
  assert isinstance(out, float)


def test_publish_constructor_failure_never_raises(hook):
  hook._pub = None
  hook._pub_ok = False
  assert isinstance(tick(hook), float)


def test_telemetry_fields(hook):
  build_lift(hook, n=30, a=0.1, v=22.0, vc=25.0)
  d = hook._pub.sent[-1]
  assert set(d) >= {'aIn', 'aOut', 'lift', 'target', 'gap', 'strength', 'gate'}
  assert d['aIn'] == pytest.approx(0.1)
  assert d['gap'] == pytest.approx(3.0)
  assert d['target'] == pytest.approx(0.4 * 2 / 3)
  assert d['aOut'] == pytest.approx(0.1 + d['lift'])
  assert d['gate'] == 'active'
  json.dumps(d)   # plain JSON types only


def test_returns_python_float(hook):
  out = hook.on_e2e_accel(np.float32(0.1), np.float32(22.0), np.float32(27.0), np.float32(1.0), make_sm(
    plan_v=np.full(len(T), 22.0, dtype=np.float32), yaw=np.zeros(len(T), dtype=np.float32)))
  assert type(out) is float


def test_internal_error_passes_through(hook):
  out = hook.on_e2e_accel(0.1, 22.0, 27.0, 1.0, {})   # sm without any needed key
  assert out == 0.1
  assert hook._lift == 0.0


def test_on_health_check(hook):
  acc = hook.on_health_check({'x': {'status': 'ok'}})
  assert acc['x'] == {'status': 'ok'}
  assert 'pace_lift' in acc and acc['pace_lift']['status'] == 'ok'


def test_manifest():
  with open(os.path.join(_PLUGIN_DIR, 'plugin.json')) as f:
    m = json.load(f)
  assert m['id'] == 'pace_lift' and m['type'] == 'hook' and m['min_openpilot'] == '0.11.2'
  assert m['hooks']['planner.e2e_accel']['priority'] == 50
  assert m['hooks']['device.health_check']['priority'] == 50
  p = m['params']
  assert p['PaceLiftEnabled']['type'] == 'bool' and p['PaceLiftEnabled']['default'] is True
  assert p['PaceLiftAggressive']['type'] == 'string' and p['PaceLiftAggressive']['default'] == '0.40'
  assert p['PaceLiftStandard']['type'] == 'string' and p['PaceLiftStandard']['default'] == '0.20'


def test_registry_style_load(tmp_path, monkeypatch):
  """Load hook.py the way selfdrive/plugins/registry.py does: sys.path gets the
  plugins dir and the plugin dir, the module goes in under plugins.<id>.<module>,
  and the bare `from config import ...` must resolve."""
  runtime = tmp_path / 'runtime'
  monkeypatch.setenv('PLUGINS_RUNTIME_DIR', str(runtime))
  monkeypatch.setattr(sys, 'path', list(sys.path))
  saved = {k: sys.modules.get(k) for k in ('config', 'plugins.pace_lift.hook')}
  sys.modules.pop('config', None)
  sys.modules.pop('plugins.pace_lift.hook', None)
  try:
    with open(os.path.join(_PLUGIN_DIR, 'plugin.json')) as f:
      manifest = json.load(f)
    for p in (_PLUGINS_DIR, _PLUGIN_DIR):
      if p not in sys.path:
        sys.path.insert(0, p)
    registered = {}
    loaded_modules = {}
    for hook_name, hook_def in manifest['hooks'].items():
      module_name = hook_def['module']
      canonical_name = f"plugins.{manifest['id']}.{module_name}"
      if canonical_name not in loaded_modules:
        module_file = os.path.join(_PLUGIN_DIR, *module_name.split('.'))
        if os.path.isdir(module_file):
          module_file = os.path.join(module_file, '__init__.py')
        else:
          module_file += '.py'
        spec = importlib.util.spec_from_file_location(canonical_name, module_file)
        module = importlib.util.module_from_spec(spec)
        sys.modules[canonical_name] = module
        spec.loader.exec_module(module)
        loaded_modules[canonical_name] = module
      registered[hook_name] = getattr(loaded_modules[canonical_name], hook_def['function'])
    assert callable(registered['planner.e2e_accel'])
    assert callable(registered['device.health_check'])
    hook = loaded_modules['plugins.pace_lift.hook']
    hook._pub_ok = False   # never open a real bus socket from a test
    out = registered['planner.e2e_accel'](0.1, 22.0, 27.0, 1.0, make_sm())
    assert type(out) is float and out == 0.1   # first tick: dt 0, no lift yet
  finally:
    for k, v in saved.items():
      if v is None:
        sys.modules.pop(k, None)
      else:
        sys.modules[k] = v
