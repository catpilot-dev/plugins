"""Pace lift: planner.e2e_accel hook entry, params, state, rates, telemetry.

Lifts the e2e accel ask toward the planner's speed target (law in lift.py).
Design: docs/superpowers/specs/2026-11-19-pace-lift-design.md.
"""
import importlib.util
import math
import os
import time

from config import read_plugin_param

_PLUGIN_DIR = os.path.dirname(os.path.abspath(__file__))

# Load the sibling `lift` module by explicit path, not off sys.path: a plugin-dir
# sys.path entry would shadow other plugins' same-named modules (see
# lane_keeping/register.py).
_lift_mod = None


def _law():
  global _lift_mod
  if _lift_mod is None:
    spec = importlib.util.spec_from_file_location(
      'pace_lift_law', os.path.join(_PLUGIN_DIR, 'lift.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    _lift_mod = mod
  return _lift_mod


RISE_RATE = 0.25      # m/s^3
FALL_RATE = 1.0       # m/s^3
DT_MAX = 0.2          # s
LEAD_HOLD_S = 1.0     # s, lead gates stay tripped this long after the condition clears
PARAM_MAX_AGE_S = 1.0
STRENGTH_MAX = 0.6
LEAD_GATES = ('lead_gap', 'lead_closing')

_clock = time.monotonic

# State.
_lift = 0.0
_last_t = None
_lead_hold_until = 0.0
_lead_hold_gate = ''
_params = None
_params_t = None

# Telemetry: lands in pluginBusLog (bus_logger auto-discovers topics).
_pub = None
_pub_ok = True


def _reset_state():
  global _lift, _last_t, _lead_hold_until, _lead_hold_gate, _params, _params_t
  _lift = 0.0
  _last_t = None
  _lead_hold_until = 0.0
  _lead_hold_gate = ''
  _params = None
  _params_t = None


def _parse_strength(raw, default):
  try:
    v = float(raw)
  except (TypeError, ValueError):
    return default
  if not math.isfinite(v):
    return default
  return min(STRENGTH_MAX, max(0.0, v))


def _read_params():
  d = _law().STRENGTH_DEFAULT
  try:
    enabled = read_plugin_param('pace_lift', 'PaceLiftEnabled') != '0'
    aggressive = _parse_strength(read_plugin_param('pace_lift', 'PaceLiftAggressive'), d['aggressive'])
    standard = _parse_strength(read_plugin_param('pace_lift', 'PaceLiftStandard'), d['standard'])
  except Exception:
    # Not a plain missing file (the reader already maps those to defaults): fail
    # to disabled so an off switch can never silently stop working.
    return {'enabled': False, 'aggressive': d['aggressive'], 'standard': d['standard']}
  return {'enabled': enabled, 'aggressive': aggressive, 'standard': standard}


def _get_params(now):
  global _params, _params_t
  if _params is None or _params_t is None or not (0.0 <= now - _params_t < PARAM_MAX_AGE_S):
    _params = _read_params()
    _params_t = now
  return _params


def _publish(telem):
  """Best-effort telemetry. Never raises into the planner's hot path."""
  global _pub, _pub_ok
  if not _pub_ok:
    return
  if _pub is None:
    try:
      from openpilot.selfdrive.plugins.plugin_bus import PluginPub
      _pub = PluginPub('paceLift')
    except Exception:
      _pub_ok = False   # no bus here (tests, bench): stop trying
      return
  try:
    _pub.send(telem)
  except Exception:
    pass


def _strength_for(personality, params):
  name = str(personality).split('.')[-1].lower()
  if name == 'aggressive':
    return params['aggressive'], _law().T_FOLLOW['aggressive']
  if name == 'standard':
    return params['standard'], _law().T_FOLLOW['standard']
  if name == 'relaxed':
    return 0.0, _law().T_FOLLOW['relaxed']
  return 0.0, _law().T_FOLLOW['relaxed']


def _f(x):
  """JSON-safe float (NaN/inf become None)."""
  try:
    x = float(x)
  except (TypeError, ValueError):
    return None
  return x if math.isfinite(x) else None


def on_e2e_accel(a_e2e, v_ego, v_cruise, a_cruise, sm):
  """planner.e2e_accel: (a_e2e, v_ego, v_cruise, a_cruise, sm) -> a_e2e."""
  global _lift
  try:
    return _on_e2e_accel(a_e2e, v_ego, v_cruise, a_cruise, sm)
  except Exception:
    _lift = 0.0   # fail safe: no lift, planner keeps the ask
    return float(a_e2e)


def _on_e2e_accel(a_e2e, v_ego, v_cruise, a_cruise, sm):
  global _lift, _last_t, _lead_hold_until, _lead_hold_gate
  law = _law()
  now = _clock()
  dt = 0.0 if _last_t is None else min(DT_MAX, max(0.0, now - _last_t))
  _last_t = now

  params = _get_params(now)
  ss = sm['selfdriveState']
  strength, t_follow = _strength_for(ss.personality, params)

  a_in = float(a_e2e)
  target = 0.0
  gate = None

  cs = sm['carState']
  if not params['enabled']:
    gate = 'disabled'
  elif not (ss.enabled and ss.experimentalMode):
    gate = 'off'
  elif cs.gasPressed:
    gate = 'gas'
  elif cs.brakePressed:
    gate = 'brake'
  elif cs.standstill:
    gate = 'standstill'
  elif not math.isfinite(float(a_cruise)):
    gate = 'no_model'

  if gate is None:
    mv = sm['modelV2']
    lead = sm['radarState'].leadOne
    target, gate = law.lift_target(
      a_in, float(v_ego), float(v_cruise), strength, t_follow,
      mv.velocity.t, mv.velocity.x, mv.orientationRate.z,
      bool(mv.action.shouldStop), bool(lead.present), float(lead.dRel), float(lead.vRel))

  hard = gate in ('disabled', 'off', 'gas', 'brake', 'standstill', 'stop',
                  'ask_decel', 'no_model')
  if hard:
    _lift = 0.0
    target = 0.0
    out = a_in if math.isfinite(a_in) else float(a_e2e)
  else:
    # Lead gates hold for LEAD_HOLD_S after the condition clears, so a flapping
    # vision lead does not pump the lift.
    if gate in LEAD_GATES:
      _lead_hold_until = now + LEAD_HOLD_S
      _lead_hold_gate = gate
    elif gate in ('active', 'none_needed') and now < _lead_hold_until:
      gate = _lead_hold_gate
      target = 0.0

    if target > _lift:
      _lift = min(target, _lift + RISE_RATE * dt)
    else:
      _lift = max(target, _lift - FALL_RATE * dt)
    # The ask cap clamps the STATE: a model easing off cuts the lift at once and
    # recovery goes through the rise rate.
    _lift = max(0.0, min(_lift, law.ask_cap(a_in, strength)))
    out = max(a_in, min(a_in + _lift, float(a_cruise)))

  out = float(out)
  _publish({
    'aIn': _f(a_in), 'aOut': _f(out), 'lift': _f(_lift), 'target': _f(target),
    'gap': _f(float(v_cruise) - float(v_ego)), 'strength': _f(strength), 'gate': gate,
  })
  return out


def on_health_check(acc, **kwargs):
  return {**acc, 'pace_lift': {'status': 'ok'}}
