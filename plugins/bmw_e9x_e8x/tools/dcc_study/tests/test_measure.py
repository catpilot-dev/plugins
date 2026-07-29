import numpy as np

from extract import CMDS
from bursts import Burst, measure

PLUS5 = CMDS.index("plus5")


def step_seg(step_a=0.6, t0=10.0, dur=3.0, tau=0.4, sp_step_kph=15.0):
  """vEgo 20 m/s; aEgo first-order step of step_a at t0; setpoint ramps up."""
  cs_t = np.arange(0.0, 60.0, 0.01)
  aEgo = np.where(cs_t < t0, 0.0, step_a * (1 - np.exp(-(cs_t - t0) / tau)))
  setpoint = np.where(cs_t < t0, 22.0, 22.0 + sp_step_kph / 3.6)
  tx_t = t0 + np.arange(int(dur / 0.025)) * 0.025
  n = len(cs_t)
  return {
    "cs_t": cs_t, "vEgo": np.full(n, 20.0), "aEgo": aEgo,
    "setpoint": setpoint, "cruiseEnabled": np.ones(n),
    "gas": np.zeros(n), "brake": np.zeros(n),
    "tx_t": tx_t, "tx_cmd": np.full(len(tx_t), PLUS5, dtype=np.int8),
    "rx_t": np.array([]), "rx_cmd": np.array([], dtype=np.int8),
    "pose_t": cs_t[::10], "pitch": np.full(len(cs_t[::10]), 0.01),
  }


def _burst(s):
  from bursts import find_bursts
  return find_bursts(s)[0]


def test_steady_state_and_baseline():
  s = step_seg(step_a=0.6)
  b = measure(_burst(s), s)
  assert abs(b.a_baseline) < 0.01
  assert 0.5 < b.steady_delta_a < 0.65
  assert 0.55 < b.peak_delta_a < 0.65


def test_rise_time_near_tau():
  s = step_seg(step_a=0.6, tau=0.4)
  b = measure(_burst(s), s)
  assert 0.2 < b.rise_time < 0.6


def test_short_burst_has_no_steady_state():
  s = step_seg(dur=0.5)
  b = measure(_burst(s), s)
  assert np.isnan(b.steady_delta_a)
  assert b.peak_delta_a > 0.3          # peak still measured


def test_setpoint_gap_and_acceptance():
  s = step_seg(sp_step_kph=15.0)
  b = measure(_burst(s), s)
  assert abs(b.setpoint_gap - 2.0) < 0.1     # 22 - 20 m/s at burst start
  assert b.ticks_accepted == 3               # 15 kph / 5 kph per plus5 tick


def test_decel_peak_is_signed():
  s = step_seg(step_a=-0.8)
  s["tx_cmd"][:] = CMDS.index("minus5")
  b = measure(_burst(s), s)
  assert b.peak_delta_a < -0.6


def test_pitch_mean_recorded():
  s = step_seg()
  b = measure(_burst(s), s)
  assert abs(b.pitch_mean - 0.01) < 1e-6


def test_no_pose_gives_nan_pitch():
  s = step_seg()
  s["pose_t"] = np.array([]); s["pitch"] = np.array([])
  assert np.isnan(measure(_burst(s), s).pitch_mean)
