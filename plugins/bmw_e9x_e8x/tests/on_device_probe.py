#!/usr/bin/env python3
"""On-device probe for the SIMPLIFIED bmw latcontroller (run offroad on C3).

Phase 2: the controller is a faithful tracker — P acts on the FULL delta_err
(no tolerance subtraction), the stiction hold retriggers on a small fixed
HOLD_BAND, and all modelV2 noise machinery (box filter, sigma-observer,
persist gate, DRIFT_M tolerance) is gone.

Usage (on C3, offroad), LK-style overridable plugin dir:
  source /usr/local/venv/bin/activate
  BMW_PLUGIN_DIR=/data/plugins-runtime/bmw_e9x_e8x python <this file>
"""
import importlib.util, math, os, sys, time
from types import SimpleNamespace

sys.path.insert(0, '/data/openpilot')
PLUGIN_DIR = os.environ.get('BMW_PLUGIN_DIR', '/data/plugins-runtime/bmw_e9x_e8x')
sys.path.insert(0, PLUGIN_DIR)
# bmw.latcontroller does `from config import read_plugin_param` — config.py
# lives at the plugins-runtime root (install.sh copies it there as a shared
# module), not inside PLUGIN_DIR, so it needs its own sys.path entry or
# exec_module raises ModuleNotFoundError. Mirrors tests/test_helpers.py's
# _PLUGINS_DIR insert (review fix, Important 1).
_PLUGINS_DIR = os.path.dirname(PLUGIN_DIR)
if _PLUGINS_DIR not in sys.path:
  sys.path.insert(0, _PLUGINS_DIR)

import numpy as np
import cereal.messaging as messaging
from openpilot.common.params import Params

if Params().get_bool('IsOnroad'):
  print('ABORT: device is onroad'); sys.exit(1)

spec = importlib.util.spec_from_file_location(
  'plugins.bmw_e9x_e8x.bmw.latcontroller', os.path.join(PLUGIN_DIR, 'bmw', 'latcontroller.py'))
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

pm = messaging.PubMaster(['livePose'])
time.sleep(0.3)

PASS = FAIL = 0
def check(name, cond, detail=''):
  global PASS, FAIL
  print(f'  [{"PASS" if cond else "FAIL"}] {name} {detail}')
  PASS, FAIL = PASS + cond, FAIL + (not cond)

def fresh():
  lac = SimpleNamespace()
  mod.on_lat_controller_init({}, lac, SimpleNamespace(wheelbase=2.76, steerActuatorDelay=0.4))
  st = None
  for c in (lac.update.__closure__ or []):
    try: v = c.cell_contents
    except ValueError: continue
    if isinstance(v, dict) and 'torque' in v and 'action' in v: st = v
  assert st is not None, 'state dict not found in closure'
  return lac, st

def tick(lac, kappa_des, v=25.0, yaw=0.0, n_can=1, active=True):
  msg = messaging.new_message('livePose')
  msg.livePose.velocityDevice.x = v
  msg.livePose.angularVelocityDevice.z = yaw
  pm.send('livePose', msg)
  time.sleep(0.004)
  out = None
  cs = SimpleNamespace(vEgo=v)
  for _ in range(n_can):
    out = lac.update(active, cs, None, None, False, kappa_des, False, 0.4)
  return out

L = 2.76
HOLD_BAND = 0.001
STEER_MAX = 12.0

print('probe 1: loads, and the noise machinery is GONE')
lac, st = fresh()
check('module loads, update patched', callable(lac.update))
gone = [k for k in ('k_sigma', 'de_dc', 'persist_w', 'de_w', 'de_buffer',
                    'kn_ema_f', 'kn_ema_s', 'kn_var', 'delta_err_raw', 'tolerance')
        if k in st]
check('no noise-machinery state keys remain', not gone, f'found={gone}')

print('probe 2: P acts on the FULL delta_err (no tolerance subtraction)')
# v=15: STEP_MAX=0.10 so the step cap does NOT bind; kappa_scale=1.0 at small kappa.
# kappa_des chosen so delta_err = atan(k*L) = 0.002 rad = 2*HOLD_BAND.
k_des = math.tan(0.002) / L
lac2, st2 = fresh()
tick(lac2, k_des, v=15.0, yaw=0.0)
expected = (1.0 * 1.0 * 15.0 * 15.0 * 0.002) / STEER_MAX   # full-error P = 0.0375
check('ramp fired (old code would HOLD: 0.002 < old tol 0.0024)', st2['action'] == 'ramp',
      f"action={st2['action']}")
check('target == full-error P', abs(st2['target_frac'] - expected) < 1e-4,
      f"target={st2['target_frac']:.5f} expected={expected:.5f}")

print('probe 3: HOLD_BAND (0.001 rad) is the hold trigger — bracketed')
# Bracket the band from both sides so the probe actually pins its value.
# Lower: 0.0008 < HOLD_BAND -> must hold.
# Upper: 0.0012 > HOLD_BAND -> must ramp. This is the discriminating half:
# the OLD controller's tolerance (noise-floored to ~0.00145 at small kappa)
# still HOLDS at 0.0012, so this fails until HOLD_BAND lands.
lac3, st3 = fresh()
tick(lac3, math.tan(0.0008) / L, v=25.0, yaw=0.0)
check('0.0008 rad (inside HOLD_BAND) -> hold_zero', st3['action'] == 'hold_zero',
      f"action={st3['action']}")
lac3b, st3b = fresh()
tick(lac3b, math.tan(0.0012) / L, v=25.0, yaw=0.0)
check('0.0012 rad (outside HOLD_BAND) -> ramp', st3b['action'] == 'ramp',
      f"action={st3b['action']}")

print('probe 4: delta_err is RAW (box filter deleted)')
lac4, st4 = fresh()
for _ in range(10):
  tick(lac4, math.tan(0.004) / L, v=15.0, yaw=0.0)   # settle a would-be filter high
tick(lac4, k_des, v=15.0, yaw=0.0)                    # step down to 0.002
check('delta_err equals the instantaneous value (no filter lag)',
      abs(st4['delta_err'] - 0.002) < 1e-4, f"delta_err={st4['delta_err']:.5f}")

print('probe 5: STEP_MAX still caps (regression)')
lac5, st5 = fresh()
tick(lac5, math.tan(0.02) / L, v=25.0, yaw=0.0)       # big error at 25 m/s
check('first step <= STEP_MAX(25 m/s) ~0.0615', abs(st5['target_frac']) <= 0.0625,
      f"target={st5['target_frac']:.4f}")

print('probe 6: hold_curve still holds torque in a curve (regression)')
lac6, st6 = fresh()
for _ in range(30):
  tick(lac6, 0.015, v=10.0, yaw=10.0 * 0.010, n_can=10)
built = st6['torque']
for _ in range(8):
  tick(lac6, 0.015, v=10.0, yaw=10.0 * 0.0149, n_can=10)
check('torque built in curve', built > 0.05, f'tq={built:.3f}')
check('hold engaged on-target', st6['action'] in ('hold_curve', 'cancel_tol'), f"action={st6['action']}")
check('held torque not drained', st6['torque'] > 0.03, f"tq={st6['torque']:.3f}")

print(f'\n{PASS} passed, {FAIL} failed')
sys.exit(1 if FAIL else 0)
