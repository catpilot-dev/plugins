#!/usr/bin/env python3
"""On-device probe harness for bmw/latcontroller.py @ runtime tree (rebuilt 2026-07-19).

Loads /data/plugins-runtime/bmw_e9x_e8x/bmw/latcontroller.py the way the local
test does (file-level, canonical name), drives it with real livePose messages,
and asserts the noise-observer + tolerance-floor mechanics plus regressions.
Run OFFROAD only (needs exclusive livePose pub).
"""
import importlib.util, math, os, sys, time
from types import SimpleNamespace

sys.path.insert(0, '/data/openpilot')
PLUGIN_DIR = '/data/plugins-runtime/bmw_e9x_e8x'
sys.path.insert(0, PLUGIN_DIR)

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
time.sleep(0.3)  # let SubMaster sockets connect

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
    if isinstance(v, dict) and 'k_sigma' in v: st = v
  assert st is not None, 'state dict not found in closure'
  return lac, st

def tick(lac, kappa_des, v=25.0, yaw=0.0, n_can=1, active=True):
  """Publish one livePose then run update() n_can times (1 livePose tick + can frames)."""
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
def tol_kin(v, t=0.4 + 0.05):  # model_action_t = lat_delay + DT_MDL
  return 2.0 * 0.02 * L / ((v * t) ** 2)

print('probe 1: load + patch')
lac, st = fresh()
check('module loads, update patched, state reachable', callable(lac.update))
check('sigma initialized at prior', abs(st['k_sigma'] - 0.00035) < 1e-9, f"={st['k_sigma']:.6f}")

print('probe 2: sigma trains on near-straight ticks')
rng = np.random.default_rng(7)
for i in range(400):  # 20 s of 0.1 Hz wander, amp 0.001 + jitter
  k = 0.001 * math.sin(2 * math.pi * 0.1 * i / 20.0) + rng.normal(0, 0.0002)
  tick(lac, float(np.clip(k, -0.0019, 0.0019)))
s_trained = st['k_sigma']
check('sigma moved off prior', abs(s_trained - 0.00035) > 2e-5, f"={s_trained:.6f}")
check('sigma in sane range', 1e-4 < s_trained < 2e-3)

print('probe 3: sigma freezes in curves and when inactive')
tick(lac, 0.02, v=12.0, yaw=12.0 * 0.02)
tick(lac, 0.02, v=12.0, yaw=12.0 * 0.02)
check('frozen at |k|=0.02', st['k_sigma'] == s_trained)
tick(lac, 0.0005, active=False)
check('frozen when inactive', st['k_sigma'] == s_trained)

print('probe 4: floor widens tolerance on straight')
lac2, st2 = fresh()  # sigma at prior 0.00035
tick(lac2, 0.0005, v=25.0)
exp = max(tol_kin(25.0), min(1.5 * st2['k_sigma'] * L, 4.0 * tol_kin(25.0)))
check('tolerance == floored value', abs(st2['tolerance'] - exp) < 1e-6,
      f"tol={st2['tolerance']:.6f} exp={exp:.6f} kin={tol_kin(25.0):.6f}")
check('floor actually above kinematic', st2['tolerance'] > tol_kin(25.0) * 1.5)

print('probe 5: fade zeroes floor in curves')
tick(lac2, 0.008, v=12.0, yaw=12.0 * 0.008)
check('curve tolerance == pure kinematic', abs(st2['tolerance'] - tol_kin(12.0)) < 1e-6,
      f"tol={st2['tolerance']:.6f} kin={tol_kin(12.0):.6f}")

print('probe 6: drift cap binds when sigma inflates')
st2['kn_var'] = 0.005 ** 2  # inject at the source; k_sigma is derived on the next trained tick
tick(lac2, 0.0005, v=25.0)
check('tolerance capped at 4x kinematic', abs(st2['tolerance'] - 4.0 * tol_kin(25.0)) < 1e-6,
      f"tol={st2['tolerance']:.6f} cap={4.0 * tol_kin(25.0):.6f}")

print('probe 7: ramp + step cap regression, then idle label')
lac3, st3 = fresh()
tick(lac3, 0.0019, v=25.0, yaw=0.0)  # tick_count is primed: first tick fires a decision from torque=0
check('decision fired a push ramp', st3['action'] == 'ramp', f"action={st3['action']}")
step_seen = abs(st3['target_frac'])  # torque was 0, so first target IS the step
check('step-capped build (<= STEP_MAX interp at 25 m/s ~0.062)', 0 < step_seen <= 0.0625,
      f"target={st3['target_frac']:.4f}")
for _ in range(6):  # further decisions each move <= step_max from current torque
  tq_pre = st3['torque']
  tick(lac3, 0.0019, v=25.0, yaw=0.0)
check('subsequent steps also capped', abs(st3['target_frac'] - tq_pre) <= 0.0625 + 1e-6,
      f"target={st3['target_frac']:.4f} tq_pre={tq_pre:.4f}")
# Each n_can=300 tick fully drains any in-flight ramp; within one cadence
# cycle at least one livePose tick is a non-decision tick that then sees
# ramp_frames == 0 and must expire the label.
got_idle = False
for _ in range(6):
  tick(lac3, 0.0019, v=25.0, yaw=0.0, n_can=300)
  if st3['action'] == 'idle':
    got_idle = True; break
check('transient label expired to idle', got_idle, f"action={st3['action']}")

print('probe 8: hold_curve keeps torque in-band (regression)')
lac4, st4 = fresh()
for _ in range(30):  # build torque in a deep curve, under-turning
  tick(lac4, 0.015, v=10.0, yaw=10.0 * 0.010, n_can=10)
tq_built = st4['torque']
for _ in range(8):   # now on-target: in-band, expect hold not drain
  tick(lac4, 0.015, v=10.0, yaw=10.0 * 0.0149, n_can=10)
check('torque built in curve', tq_built > 0.05, f"tq={tq_built:.3f}")
check('hold_curve engaged on-target', st4['action'] in ('hold_curve', 'cancel_tol'),
      f"action={st4['action']}")
check('held torque not drained to 0', st4['torque'] > 0.03, f"tq={st4['torque']:.3f}")

print(f'\n{PASS} passed, {FAIL} failed')
sys.exit(1 if FAIL else 0)
