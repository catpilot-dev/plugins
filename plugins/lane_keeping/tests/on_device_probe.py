#!/usr/bin/env python3
"""On-device probe for the lane_keeping runtime (run offroad on C3).

Loads the plugin's anchor.py directly and asserts every control branch. No
cereal pub needed — anchor.py is pure. modelV2 device frame is +y=RIGHT, so
the LEFT ego line (laneLines[1]) is at NEGATIVE y and the RIGHT (laneLines[2])
at POSITIVE y; curvature is left-positive.

Usage (on C3, offroad), pointing PLUGIN_DIR at wherever anchor.py lives:
  source /usr/local/venv/bin/activate
  python <this file>            # defaults PLUGIN_DIR to the file's own plugin dir
"""
import importlib.util, os, sys
from types import SimpleNamespace

PLUGIN_DIR = os.environ.get('LK_PLUGIN_DIR',
                            os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
spec = importlib.util.spec_from_file_location('lk_anchor', os.path.join(PLUGIN_DIR, 'anchor.py'))
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
AnchorConfig, LaneAnchor = mod.AnchorConfig, mod.LaneAnchor

PASS = FAIL = 0
def check(name, cond, detail=''):
  global PASS, FAIL
  print(f'  [{"PASS" if cond else "FAIL"}] {name} {detail}')
  PASS, FAIL = PASS + cond, FAIL + (not cond)

def mv(left_y, right_y, lp=1.0, rp=1.0):
  return SimpleNamespace(
    laneLines=[SimpleNamespace(y=[0.0]), SimpleNamespace(y=[left_y]),
               SimpleNamespace(y=[right_y]), SimpleNamespace(y=[0.0])],
    laneLineProbs=[0.0, lp, rp, 0.0])

def settle(a, m, v=25.0, lc=False, n=3000):
  out = None
  for _ in range(n):
    out, _t = a.update(0.01, m, v, lc)
  return out

print('probe 1: passthrough')
a = LaneAnchor(AnchorConfig())
o, t = a.update(0.0123, SimpleNamespace(laneLines=[], laneLineProbs=[]), 25.0, False)
check('no line -> bit-identical passthrough', o == 0.0123 and t['state'] == 'model')
o, t = a.update(0.0123, mv(-1.75, 1.75, lp=0.4), 25.0, False)
check('low prob -> passthrough', o == 0.0123 and t['state'] == 'model')

print('probe 2: in-band no bias')
# left line at y=-1.75 -> gap 0.84 in [0.6,1.0]
check('gap 0.84 in [0.6,1.0] -> no bias', abs(settle(LaneAnchor(AnchorConfig()), mv(-1.75, 1.75)) - 0.01) < 1e-6)

print('probe 3: out-of-band bias, left driver')
# left line at y=-2.3 -> gap 1.39 above band -> steer left (positive)
check('too far from left line -> steer left', settle(LaneAnchor(AnchorConfig()), mv(-2.3, 1.2)) > 0.01 + 1e-5)
# left line at y=-1.3 -> gap 0.39 below band -> steer right (negative)
check('too close to left line -> steer right', settle(LaneAnchor(AnchorConfig()), mv(-1.3, 2.2)) < 0.01 - 1e-5)

print('probe 4: out-of-band bias, right driver')
# right driver: right line (laneLines[2]) at y=+2.3 -> gap 1.39 above band -> steer right (negative)
check('right driver too far from right line -> steer right',
      settle(LaneAnchor(AnchorConfig(driver_side='right')), mv(-1.2, 2.3)) < 0.01 - 1e-5)

print('probe 5: lane-change disable')
check('lane change -> passthrough', abs(settle(LaneAnchor(AnchorConfig()), mv(-2.3, 1.2), lc=True) - 0.01) < 1e-6)

print('probe 6: glitch clip')
a = LaneAnchor(AnchorConfig())
a.gap_filt = 9.0
_o, t = a.update(0.01, mv(-9.9, 1.2), 25.0, False)
check('huge gap -> excess clipped to excess_max', abs(t['excess']) == 0.5)

print('probe 7: rate limit')
a = LaneAnchor(AnchorConfig(kappa_rate_max=0.002))
m = mv(-2.3, 1.2); a.gap_filt = a._gap(m)
_o, t = a.update(0.01, m, 25.0, False)
check('first tick bias <= rate*DT', abs(t['kappa_bias']) <= 0.002 * 0.01 + 1e-12)

print(f'\n{PASS} passed, {FAIL} failed')
sys.exit(1 if FAIL else 0)
