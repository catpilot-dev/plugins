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
import importlib.util, os, shutil, sys, tempfile
from types import SimpleNamespace

PLUGIN_DIR = os.environ.get('LK_PLUGIN_DIR',
                            os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
spec = importlib.util.spec_from_file_location('lk_anchor', os.path.join(PLUGIN_DIR, 'anchor.py'))
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
AnchorConfig, LaneAnchor = mod.AnchorConfig, mod.LaneAnchor

# register.py and calib_trim.py, loaded the same explicit-path way (no
# sys.path insert — see register.py's own module-loading comment). Loaded
# once here since _anchor_module()/_trim_module() are lazy and never touched
# by the trim probes below (they only exercise the writer/reader/law).
reg_spec = importlib.util.spec_from_file_location('lk_register_probe', os.path.join(PLUGIN_DIR, 'register.py'))
register = importlib.util.module_from_spec(reg_spec)
reg_spec.loader.exec_module(register)

trim_spec = importlib.util.spec_from_file_location('lk_calib_trim_probe', os.path.join(PLUGIN_DIR, 'calib_trim.py'))
calib_trim = importlib.util.module_from_spec(trim_spec)
trim_spec.loader.exec_module(calib_trim)
TrimConfig, CalibTrim = calib_trim.TrimConfig, calib_trim.CalibTrim

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

print('probe 3: constant offsets are CONCEDED, left driver (AC stabilizer)')
# static gap 1.39 (above old band) and 0.39 (below): the line is the model's
check('constant far gap conceded', abs(settle(LaneAnchor(AnchorConfig()), mv(-2.3, 1.2)) - 0.01) < 1e-6)
check('constant near gap conceded', abs(settle(LaneAnchor(AnchorConfig()), mv(-1.3, 2.2)) - 0.01) < 1e-6)

print('probe 4: constant offset conceded, right driver')
check('right driver constant gap conceded',
      abs(settle(LaneAnchor(AnchorConfig(driver_side='right')), mv(-1.2, 2.3)) - 0.01) < 1e-6)

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
check('converging line: prediction sees it, static scene conceded',
      t['gap_pred'] < 0.6 and abs(out) < 1e-6, f"gp={t['gap_pred']:.3f} out={out:.5f}")

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
drift_plan = [-0.5 * (x / 30.0) for x in XS]      # plan angles toward the left line
for _ in range(2000):
  out, t = a.update(0.0, mv_geo(flat(-1.75), flat(1.75), plan_ys=drift_plan), 25.0, False, lat_delay=0.6)
check('plan-toward-line: prediction sees it, static scene conceded',
      t['gap_pred'] < 0.6 and abs(out) < 1e-6, f"gp={t['gap_pred']:.3f} out={out:.5f}")

a = LaneAnchor(AnchorConfig())
for _ in range(3000):
  out, t = a.update(0.01, mv(-2.3, 1.2), 25.0, False, lat_delay=0.6)
check('single-point lines: fallback wired (gap_pred==gap_filt), conceded',
      abs(t['gap_pred'] - t['gap_filt']) < 1e-9 and abs(out - 0.01) < 1e-6,
      f"gp={t['gap_pred']:.3f} gf={t['gap_filt']:.3f} out={out:.5f}")


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


print('probe 11: AC stabilizer (concede the line, damp the wander)')
def mv_at(gap):
  y = -(gap + 0.91)
  return mv_geo(flat(y), flat(1.75))
a = LaneAnchor(AnchorConfig())
out = None
for _ in range(3000):
  out, t = a.update(0.0, mv_at(1.39), 17.0, False, lat_delay=0.6)
check('constant out-of-band gap is CONCEDED (anti-3c1)', abs(out) < 1e-6 and abs(t['gap_dc'] - 1.39) < 0.05,
      f"out={out:.6f} dc={t['gap_dc']:.2f}")
for i in range(600):
  out, t = a.update(0.0, mv_at(1.39 + 0.05 * (i / 100.0)), 17.0, False, lat_delay=0.6)
check('drift is damped (positive/leftward vs rightward drift)', out > 1e-5, f'out={out:.6f}')
a2 = LaneAnchor(AnchorConfig())
out, _t = None, None
for _ in range(3000):
  out, _t = a2.update(0.0, mv_at(0.2), 17.0, False, lat_delay=0.6)
check('hard floor still absolute at 0.2 m', out < -1e-5, f'out={out:.6f}')


print('probe 22: trim file round-trip through _write_yaw_file + fresh-cache read')
# Point register at a throwaway data dir for the duration of this probe so a
# real device's live CalibTrimYawDeg / cache state is never touched, then
# restore + delete it — mirrors the tmp_path/monkeypatch pattern the pytest
# suite uses (test_register_trim.py's data_dir fixture) since this script has
# no fixtures of its own.
_orig_plugin_dir = register._PLUGIN_DIR
_tmp22 = tempfile.mkdtemp(prefix='lk_probe22_')
os.makedirs(os.path.join(_tmp22, 'data'), exist_ok=True)
try:
  register._PLUGIN_DIR = _tmp22
  register._last_yaw_written = None
  register._write_yaw_file(0.234)
  path22 = os.path.join(_tmp22, 'data', 'CalibTrimYawDeg')
  check('_write_yaw_file created the param file', os.path.exists(path22))
  with open(path22) as f:
    on_disk = f.read().strip()
  check('file round-trip value matches what was written', on_disk == '0.234', f'on_disk={on_disk}')
  register._calib_bias_cache = {'val': 0.0, 'calls': 0}   # force a fresh-cache read
  val22 = register.on_calib_bias(0.0)
  check('on_calib_bias fresh-cache read matches the file', abs(val22 - 0.234) < 1e-6, f'val={val22}')
finally:
  register._PLUGIN_DIR = _orig_plugin_dir
  register._last_yaw_written = None
  register._calib_bias_cache = {'val': 0.0, 'calls': 0}
  shutil.rmtree(_tmp22, ignore_errors=True)


print('probe 23: on_calib_bias / _read_yaw_deg return 0.0 with file deleted')
_tmp23 = tempfile.mkdtemp(prefix='lk_probe23_')
os.makedirs(os.path.join(_tmp23, 'data'), exist_ok=True)
try:
  register._PLUGIN_DIR = _tmp23
  register._last_yaw_written = None
  register._calib_bias_cache = {'val': 0.0, 'calls': 0}
  register._write_yaw_file(0.5)
  path23 = os.path.join(_tmp23, 'data', 'CalibTrimYawDeg')
  check('setup: file exists before deletion', os.path.exists(path23))
  os.remove(path23)
  check('_read_yaw_deg returns 0.0 once the file is gone', register._read_yaw_deg() == 0.0)
  register._calib_bias_cache = {'val': 0.0, 'calls': 0}   # fresh cache, forces the miss to re-read
  check('on_calib_bias returns 0.0 once the file is gone', register.on_calib_bias(0.0) == 0.0)
finally:
  register._PLUGIN_DIR = _orig_plugin_dir
  register._last_yaw_written = None
  register._calib_bias_cache = {'val': 0.0, 'calls': 0}
  shutil.rmtree(_tmp23, ignore_errors=True)


print('probe 24: trim law mode-1 (0.3 deg, 2000 ticks) — monotone, slew-capped, converges')
trim24 = CalibTrim(TrimConfig(mode=1, fixed_deg=0.3))
prev24, monotone24, step_ok24, d24 = 0.0, True, True, 0.0
step_cap = trim24.cfg.slew_deg_s * calib_trim.DT_CTRL + 1e-9   # default slew_deg_s * DT_CTRL, with float slack
for _ in range(2000):
  d24, _t24 = trim24.update(0.8, 1.0, False, 15.0, True)
  if d24 < prev24 - 1e-12:
    monotone24 = False
  if abs(d24 - prev24) > step_cap:
    step_ok24 = False
  prev24 = d24
check('mode-1 delta_deg is monotone non-decreasing toward fixed_deg', monotone24)
check('mode-1 per-tick step never exceeds slew_deg_s * DT_CTRL', step_ok24)
check('mode-1 converges to fixed_deg (cap not hit at 0.3 deg)', abs(d24 - 0.3) < 1e-6, f'd={d24:.4f}')


print('probe 25: trim law mode-2 with yaw_sign=0 stays inert at 0.0')
trim25 = CalibTrim(TrimConfig(mode=2, yaw_sign=0))
d25 = 0.0
for _ in range(2000):
  d25, t25 = trim25.update(0.2, 1.0, False, 15.0, True)   # gap far below band -> would integrate if armed
check('mode-2 with yaw_sign=0 never integrates (behaves as mode 0)', d25 == 0.0, f'd25={d25}')
check('mode-2 with yaw_sign=0 reports not integrating', t25['integrating'] is False)

# TODO(device): modelExecutionTime p95 regression check (spec Sec 8) is
# deferred to the C3 deploy step (spec Sec 9 step 3) — it needs a live model
# process and cannot be probed from a standalone script.

print(f'\n{PASS} passed, {FAIL} failed')
sys.exit(1 if FAIL else 0)
