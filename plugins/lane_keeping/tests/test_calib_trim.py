import importlib.util
import os

import pytest

_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
spec = importlib.util.spec_from_file_location('lk_calib_trim', os.path.join(_DIR, 'calib_trim.py'))
ct = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ct)


def _run(trim, n, **kw):
  args = dict(gap_dc=0.8, authority=1.0, lane_changing=False, v_ego=15.0, enabled=True)
  args.update(kw)
  out = None
  for _ in range(n):
    out = trim.update(**args)
  return out


def test_mode0_stays_zero():
  trim = ct.CalibTrim(ct.TrimConfig(mode=0))
  d, t = _run(trim, 1000)
  assert d == 0.0
  assert t['err'] == 0.0


def test_mode1_slews_to_fixed_and_respects_cap():
  trim = ct.CalibTrim(ct.TrimConfig(mode=1, fixed_deg=0.3))
  d1, _ = trim.update(0.8, 1.0, False, 15.0, True)
  assert 0 < d1 <= 0.02 * ct.DT_CTRL + 1e-12          # first tick slew-limited
  d, _ = _run(trim, 100 * 60)                          # 60 s
  assert abs(d - 0.3) < 1e-6
  trim2 = ct.CalibTrim(ct.TrimConfig(mode=1, fixed_deg=5.0, max_deg=0.8))
  d, _ = _run(trim2, 100 * 120)
  assert abs(d - 0.8) < 1e-6                           # capped


def test_mode1_rate_never_exceeds_slew():
  trim = ct.CalibTrim(ct.TrimConfig(mode=1, fixed_deg=0.8))
  prev = 0.0
  for _ in range(2000):
    d, _ = trim.update(0.8, 1.0, False, 15.0, True)
    assert abs(d - prev) <= 0.02 * ct.DT_CTRL + 1e-12
    prev = d


def test_mode2_inert_without_sign():
  trim = ct.CalibTrim(ct.TrimConfig(mode=2, yaw_sign=0))
  d, _ = _run(trim, 100 * 30, gap_dc=0.2)              # far out of band
  assert d == 0.0


def test_mode2_integrates_toward_band_both_signs():
  for sign in (1, -1):
    trim = ct.CalibTrim(ct.TrimConfig(mode=2, yaw_sign=sign))
    d, t = _run(trim, 100 * 10, gap_dc=0.3)            # err = -0.3 (too close)
    assert t['integrating']
    assert d != 0.0
    # convention: positive err moves gap down; err<0 must push delta the
    # direction that (per yaw_sign) raises the gap: d has sign +yaw_sign*? —
    # pin the ALGEBRA, not intuition: dδ = clip(-ki*err*yaw_sign, ±slew)*DT
    exp_sign = 1.0 if (-(-0.3) * sign) > 0 else -1.0
    assert (d > 0) == (exp_sign > 0)


def test_mode2_in_band_decays_after_dwell():
  trim = ct.CalibTrim(ct.TrimConfig(mode=2, yaw_sign=1))
  _run(trim, 100 * 20, gap_dc=0.3)                     # build up some delta
  d_built, _ = trim.update(0.3, 1.0, False, 15.0, True)
  assert d_built != 0.0
  d_after_dwell, t = _run(trim, 100 * 5, gap_dc=0.8)   # in band, dwell running
  assert abs(d_after_dwell - d_built) <= 0.02 * 0.05 + 1e-9  # no decay yet (first 5 s)
  d_decayed, _ = _run(trim, 100 * 30, gap_dc=0.8)
  assert abs(d_decayed) < abs(d_built)                 # decaying at slew/2
  d_final, _ = _run(trim, 100 * 120, gap_dc=0.8)
  assert d_final == 0.0


def test_hold_on_untrusted_lc_slow():
  trim = ct.CalibTrim(ct.TrimConfig(mode=2, yaw_sign=1))
  _run(trim, 100 * 20, gap_dc=0.3)
  d0, _ = trim.update(0.3, 1.0, False, 15.0, True)
  for kw in (dict(authority=0.0), dict(lane_changing=True), dict(v_ego=3.0)):
    d, t = _run(trim, 100 * 30, gap_dc=0.3, **kw)
    assert d == d0 and not t['integrating']            # held exactly


def test_disabled_decays_to_zero():
  trim = ct.CalibTrim(ct.TrimConfig(mode=2, yaw_sign=1))
  _run(trim, 100 * 20, gap_dc=0.3)
  d, _ = _run(trim, 100 * 200, enabled=False)
  assert d == 0.0


def test_frozen_gap_dc_accepted():
  trim = ct.CalibTrim(ct.TrimConfig(mode=2, yaw_sign=1))
  d, t = _run(trim, 100 * 10, gap_dc=0.43)             # frozen below band
  assert t['integrating'] and d != 0.0


def test_mode2_dwell_resets_after_hold_dropout():
  trim = ct.CalibTrim(ct.TrimConfig(mode=2, yaw_sign=1))
  _run(trim, 100 * 20, gap_dc=0.3)                       # build up delta
  d_built, _ = trim.update(0.3, 1.0, False, 15.0, True)
  assert d_built != 0.0
  # partial dwell in-band: 4 s — not enough to trigger decay on its own
  d_partial, _ = _run(trim, 100 * 4, gap_dc=0.8)
  assert abs(d_partial - d_built) <= 0.02 * 0.05 + 1e-9  # unchanged (no decay yet)
  # dropout: gate fails (authority=0) for 30 s — a blind period, NOT
  # observed dwell; must not carry forward toward the 5 s requirement
  d_held, t_held = _run(trim, 100 * 30, gap_dc=0.8, authority=0.0)
  assert d_held == d_partial
  assert not t_held['integrating']
  # back in-band: dwell restarts from zero — the first fresh 5 s must NOT decay
  d_first5, _ = _run(trim, 100 * 5, gap_dc=0.8)
  assert d_first5 == d_held                              # unchanged through fresh 5 s
  # beyond the fresh 5 s, decay finally begins
  d_decayed, _ = _run(trim, 100 * 5, gap_dc=0.8)
  assert abs(d_decayed) < abs(d_first5)


def test_mode2_upper_band_violation_opposite_sign():
  trim_lo = ct.CalibTrim(ct.TrimConfig(mode=2, yaw_sign=1))
  d_lo, _ = _run(trim_lo, 100 * 10, gap_dc=0.3)          # below gap_lo: err=-0.3

  trim_hi = ct.CalibTrim(ct.TrimConfig(mode=2, yaw_sign=1))
  d_hi, t_hi = _run(trim_hi, 100 * 10, gap_dc=1.4)       # above gap_hi=1.0: err=+0.4

  assert (d_lo > 0) != (d_hi > 0)                        # opposite integration direction
  assert t_hi['integrating']
  assert t_hi['err'] == pytest.approx(0.4)


def test_mode2_none_gap_dc_holds():
  trim = ct.CalibTrim(ct.TrimConfig(mode=2, yaw_sign=1))
  _run(trim, 100 * 20, gap_dc=0.3)
  d0, _ = trim.update(0.3, 1.0, False, 15.0, True)
  d, t = _run(trim, 100 * 10, gap_dc=None)
  assert d == d0                                          # held exactly, no crash
  assert not t['integrating']


# --------------------------------------------------------------------------
# Construction-time seeding (spec section 3): delta_deg starts from the
# clamped persisted file value, not always 0 — controls law, file, and
# modeld must agree at startup with no step.
# --------------------------------------------------------------------------

def test_seed_sets_delta_immediately_and_mode0_slews_down_from_it():
  trim = ct.CalibTrim(ct.TrimConfig(mode=0), initial_deg=0.3)
  assert trim.delta_deg == pytest.approx(0.3)             # seeded immediately, no step
  d1, _ = trim.update(0.8, 1.0, False, 15.0, True)         # mode 0: slews toward 0
  assert d1 < 0.3                                          # slewing DOWN from 0.3, not a reset
  assert d1 == pytest.approx(0.3 - 0.02 * ct.DT_CTRL, abs=1e-9)


def test_seed_clamps_to_max_deg():
  trim = ct.CalibTrim(ct.TrimConfig(mode=0, max_deg=0.8), initial_deg=5.0)
  assert trim.delta_deg == pytest.approx(0.8)


def test_seed_nonfinite_becomes_zero():
  for bad in (float('nan'), float('inf'), float('-inf')):
    trim = ct.CalibTrim(ct.TrimConfig(mode=0), initial_deg=bad)
    assert trim.delta_deg == 0.0
