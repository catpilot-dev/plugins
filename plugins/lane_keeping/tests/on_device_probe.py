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

print('probe 8: kappa_des smoothing (previously uncovered)')
a = LaneAnchor(AnchorConfig())
none_mv = SimpleNamespace(laneLines=[], laneLineProbs=[])
a.update(0.02, none_mv, 25.0, False)
o, t = a.update(0.0, none_mv, 25.0, False)
check('reference lags a step (smoothing active)', 0.0 < o < 0.02, f'o={o:.4f}')
o, t = a.update(0.0, none_mv, 25.0, True)
check('lane change bypasses the filter', o == 0.0)

print('probe 9: predictive deadband')
XS = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0]
def mv_geo(left_ys, right_ys, plan_ys=None):
  # position = the model's planned path; prediction is line-minus-plan
  return SimpleNamespace(
    laneLines=[SimpleNamespace(x=[], y=[0.0]),
               SimpleNamespace(x=list(XS), y=list(left_ys)),
               SimpleNamespace(x=list(XS), y=list(right_ys)),
               SimpleNamespace(x=[], y=[0.0])],
    laneLineProbs=[0.0, 1.0, 1.0, 0.0],
    position=SimpleNamespace(x=list(XS),
                             y=list(plan_ys if plan_ys is not None else [0.0] * len(XS))))
flat = lambda y: [y] * len(XS)

a = LaneAnchor(AnchorConfig())
out = None
for _ in range(500):
  out, t = a.update(0.0, mv_geo(flat(-1.75), flat(1.75)), 25.0, False, lat_delay=0.6)
check('parallel line: gap_pred==gap, x_pred=22.5 (mult 1.5), no bias',
      abs(t['gap_pred'] - 0.84) < 1e-3 and abs(t['x_pred'] - 22.5) < 1e-6 and abs(out) < 1e-5,
      f"gp={t['gap_pred']:.3f} xp={t['x_pred']:.1f} out={out:.5f}")

a = LaneAnchor(AnchorConfig())
conv = [-1.75 + 0.5 * (x / 30.0) for x in XS]
for _ in range(2000):
  out, t = a.update(0.0, mv_geo(conv, flat(1.75)), 25.0, False, lat_delay=0.6)
check('converging line: nudges early (right/negative)', t['gap_pred'] < 0.6 and out < -1e-5,
      f"gp={t['gap_pred']:.3f} out={out:.5f}")

a = LaneAnchor(AnchorConfig())
recov = [-1.41 - 0.4 * (x / 30.0) for x in XS]
for _ in range(2000):
  out, t = a.update(0.0, mv_geo(recov, flat(1.75)), 25.0, False, lat_delay=0.6)
check('recovering: current out-of-band but predictor holds',
      t['gap_filt'] < 0.6 and t['gap_pred'] > 0.6 and abs(out) < 1e-5,
      f"gf={t['gap_filt']:.3f} gp={t['gap_pred']:.3f} out={out:.5f}")

a = LaneAnchor(AnchorConfig())
crit = [-1.11 - 0.7 * (x / 30.0) for x in XS]
for _ in range(2000):
  out, t = a.update(0.0, mv_geo(crit, flat(1.75)), 25.0, False, lat_delay=0.6)
check('hard floor: 0.2m gap corrects despite recovering prediction',
      t['gap_filt'] < 0.3 and out < -1e-5, f"gf={t['gap_filt']:.3f} out={out:.5f}")

a = LaneAnchor(AnchorConfig())
drift_plan = [-0.5 * (x / 30.0) for x in XS]      # plan drifts toward the left line
for _ in range(2000):
  out, t = a.update(0.0, mv_geo(flat(-1.75), flat(1.75), plan_ys=drift_plan), 25.0, False, lat_delay=0.6)
check('plan drifting toward line: predicted out-of-band, nudges right',
      t['gap_pred'] < 0.6 and out < -1e-5, f"gp={t['gap_pred']:.3f} out={out:.5f}")

a = LaneAnchor(AnchorConfig())
for _ in range(3000):
  out, _t = a.update(0.01, mv(-2.3, 1.2), 25.0, False, lat_delay=0.6)
check('single-point lines fall back to current-gap deadband', out > 0.01 + 1e-5,
      f'out={out:.5f}')


print('probe 10: lane-change filter re-seed (no stale settle-nudge)')
a = LaneAnchor(AnchorConfig())
for _ in range(2000):
  a.update(0.0, mv_geo(flat(-1.31), flat(2.19)), 25.0, False, lat_delay=0.6)   # old lane: gap 0.40
out, t = a.update(0.0, mv_geo(flat(-1.75), flat(1.75)), 25.0, True, lat_delay=0.6)   # LC, new lane
ok_lc = abs(t['gap_filt'] - 0.84) < 1e-9 and out == 0.0
out, t = a.update(0.0, mv_geo(flat(-1.75), flat(1.75)), 25.0, False, lat_delay=0.6)  # first tick after
check('filters re-seed during LC; first post-LC tick clean',
      ok_lc and abs(t['gap_filt'] - 0.84) < 1e-6 and abs(out) < 1e-6,
      f"gf={t['gap_filt']:.3f} out={out:.6f}")


print('probe 11: integral trim (DC authority; hold-bias no-ratchet lesson)')
a = LaneAnchor(AnchorConfig())
below = mv_geo(flat(-1.41), flat(2.09))            # gap 0.50, below band
for _ in range(1000):
  a.update(0.0, below, 25.0, False, lat_delay=0.6)
ok_dir = a.kappa_trim < -0.5e-4                    # builds rightward (negative)
above = mv_geo(flat(-2.11), flat(1.39))            # gap 1.20, above band
for _ in range(1500):
  a.update(0.0, above, 25.0, False, lat_delay=0.6)
ok_unwind = a.kappa_trim > 0.0                     # unwound through zero: no ratchet
a.update(0.0, above, 25.0, True, lat_delay=0.6)
check('trim builds, unwinds on opposite error, zeroed on lane change',
      ok_dir and ok_unwind and a.kappa_trim == 0.0,
      f"trim_after_lc={a.kappa_trim}")

print(f'\n{PASS} passed, {FAIL} failed')
sys.exit(1 if FAIL else 0)
