import numpy as np
import pytest

from bursts import Burst
from common import CMD_STEP
import eval_route as ev


def _burst(t_start, dur, cmd="plus1", cadence="hold", n_frames=None):
  if n_frames is None:
    n_frames = max(2, int(round(dur / 0.02)) + 1)
  return Burst(t_start=t_start, t_end=t_start + dur, cmd=cmd, cadence=cadence,
               n_frames=n_frames)


# ------------------------------------------------------------------- isolation

def test_isolation_rejects_close_neighbour():
  """The failure this exists for: an opposing command 0.05 s away."""
  a = _burst(10.0, 0.2, "plus1")
  b = _burst(10.25, 0.15, "minus5")      # 0.05 s after a ends
  far = _burst(20.0, 0.1, "plus1")       # 9.6 s clear, no next burst
  assert ev.isolated_flags([a, b, far]) == [False, False, True]


def test_isolation_needs_clearance_on_both_sides():
  a = _burst(0.0, 0.1)                   # no prev, 2.9 s to next -> isolated
  mid = _burst(3.0, 0.1)                 # 2.9 s before but 0.5 s after -> not
  c = _burst(3.6, 0.1)                   # 0.5 s before, no next -> not
  assert ev.isolated_flags([a, mid, c]) == [True, False, False]

  a = _burst(0.0, 0.1)
  mid = _burst(3.0, 0.1)
  c = _burst(6.0, 0.1)                   # now clear on both sides
  assert ev.isolated_flags([a, mid, c]) == [True, True, True]


def test_isolation_is_order_independent():
  a, b = _burst(10.0, 0.2), _burst(10.25, 0.15)
  far = _burst(20.0, 0.1)
  assert ev.isolated_flags([far, b, a]) == [True, False, False]


# ------------------------------------------------------- end-of-segment settle

def _setpoint_segment():
  """10 s of carState at 100 Hz; setpoint steps +1 kph at t=5, garbage at end.

  The trailing 70 m/s is the disabled-cruise sentinel that produced a 252 km/h
  reading once np.interp clamped to it.
  """
  cs_t = np.arange(0.0, 10.0, 0.01)
  sp = np.full(cs_t.shape, 60.0 / 3.6)
  sp[cs_t >= 5.0] = 61.0 / 3.6
  sp[-5:] = 70.0
  return cs_t, sp


def test_settle_clamp_drops_burst_at_segment_end():
  cs_t, sp = _setpoint_segment()
  b = _burst(9.0, 0.85)                  # ends 9.85, only 0.14 s of data left
  assert ev.burst_ticks(b, cs_t, sp) is None


def test_settle_reads_normal_burst():
  cs_t, sp = _setpoint_segment()
  b = _burst(4.8, 0.2)                   # setpoint steps +1 kph during it
  assert ev.burst_ticks(b, cs_t, sp) == 1


def test_settle_clamp_prevents_sentinel_reading():
  """Without the clamp this burst reads the 70 m/s tail as tens of ticks."""
  cs_t, sp = _setpoint_segment()
  b = _burst(9.5, 0.4)
  assert ev.burst_ticks(b, cs_t, sp) is None
  # the unguarded computation is what we are refusing to print
  bogus = (float(np.interp(b.t_end + ev.SETTLE_S, cs_t, sp)) -
           float(np.interp(b.t_start - ev.PRE_S, cs_t, sp))) * 3.6
  assert bogus > 100.0


def test_missing_pre_burst_baseline_is_dropped():
  cs_t, sp = _setpoint_segment()
  assert ev.burst_ticks(_burst(0.05, 0.2), cs_t, sp) is None


def test_ticks_are_signed_by_command_direction():
  cs_t = np.arange(0.0, 10.0, 0.01)
  sp = np.full(cs_t.shape, 60.0 / 3.6)
  sp[cs_t >= 5.0] = 55.0 / 3.6           # setpoint fell 5 kph
  b = _burst(4.8, 0.2, cmd="minus5")
  assert CMD_STEP["minus5"] == -5
  assert ev.burst_ticks(b, cs_t, sp) == 1      # one accepted minus5 tick
  assert ev.burst_ticks(_burst(4.8, 0.2, cmd="plus5"), cs_t, sp) == -1  # wrong way


# --------------------------------------------------------- direction agreement

def test_direction_agreement_hand_built():
  # deadzones: |v_error| > 0.1389 m/s, |aTarget| > 0.05
  v_error = np.array([1.0, -1.0, 0.05, 1.0, -1.0])
  a_target = np.array([0.5, 0.5, 0.5, -0.5, 0.01])
  #              agree  BAD  v in dz  BAD   a in dz
  assert ev.direction_agreement(v_error, a_target) == (3, 2)


def test_direction_agreement_empty():
  assert ev.direction_agreement(np.zeros(5), np.zeros(5)) == (0, 0)


def test_sign_change_count_ignores_deadzone_dwell():
  # + + (dz) + -> no reversal; the deadzone samples are not sign flips
  assert ev.sign_change_count([1.0, 1.0, 0.0, 1.0], 0.05) == 0
  assert ev.sign_change_count([1.0, -1.0, 1.0], 0.05) == 2
  assert ev.sign_change_count([1.0], 0.05) == 0


def test_sign_changes_masked_ignores_run_boundaries():
  x = np.array([1.0, 1.0, -1.0, -1.0])
  mask = np.array([True, False, False, True])   # two runs, no flip inside either
  assert ev.sign_changes_masked(x, mask, 0.05) == 0
  assert ev.sign_change_count(x[mask], 0.05) == 1  # naive version would see one


# ------------------------------------------------------------------ setpoint churn

def test_setpoint_churn_counts_travel_not_net():
  sp = np.array([16.0, 17.0, 16.0, 17.0, 16.0]) / 1.0
  mask = np.ones(5, dtype=bool)
  travel, net, rev = ev.setpoint_churn(sp, mask)
  assert travel == pytest.approx(4.0)
  assert net == pytest.approx(0.0)
  assert rev == 3


def test_setpoint_churn_ignores_float_noise():
  sp = np.array([16.0, 16.0 + 1e-4, 16.0 - 1e-4, 16.0])
  travel, net, rev = ev.setpoint_churn(sp, np.ones(4, dtype=bool))
  assert travel == 0.0 and net == 0.0 and rev == 0


def test_clean_runs():
  mask = np.array([False, True, True, False, True])
  assert ev.clean_runs(mask) == [(1, 3), (4, 5)]
  assert ev.clean_runs(np.zeros(3, dtype=bool)) == []


# --------------------------------------------------------------- segment gating

def _synth_seg(n=1200, enabled=True, v=25.0, a_bias=0.1, sp=25.0):
  t = np.arange(n) * 0.01
  seg = {"cs_t": t,
         "vEgo": np.full(n, v),
         "aEgo": np.full(n, a_bias),
         "setpoint": np.full(n, sp),
         "cruiseEnabled": np.full(n, 1.0 if enabled else 0.0),
         "gas": np.zeros(n),
         "brake": np.zeros(n),
         "ctrl_t": t,
         "aTarget": np.zeros(n),
         "vTarget": np.full(n, sp),
         "ctrlEnabled": np.full(n, 1.0),
         "tx_t": np.array([]), "tx_cmd": np.array([], dtype=np.int8),
         "rx_t": np.array([]), "rx_cmd": np.array([], dtype=np.int8),
         "pose_t": np.array([]), "pitch": np.array([])}
  return seg


def test_segment_gating():
  assert ev.segment_usable(_synth_seg())
  assert not ev.segment_usable(_synth_seg(n=400))            # < MIN_TOTAL
  assert not ev.segment_usable(_synth_seg(enabled=False))    # no clean samples
  assert not ev.segment_usable(_synth_seg(v=1.0))            # below V_MIN


def test_evaluate_end_to_end():
  seg = _synth_seg(a_bias=0.1)
  m = ev.evaluate([("a", seg), ("short", _synth_seg(n=400))])
  assert m["n_used"] == 1 and m["n_skipped"] == 1
  assert m["n_clean"] == 1200
  assert m["minutes"] == pytest.approx(0.2)
  assert m["trk_med"] == pytest.approx(0.1)      # aTarget 0, aEgo 0.1
  assert m["trk_signed"] == pytest.approx(0.1)
  assert m["ls_n"] == 0                          # vEgo 25 -> not low speed
  assert m["n_bursts"] == 0
  assert np.isnan(m["churn_ratio"])              # setpoint never moves
  assert m["have_vtarget"]
  assert m["da_n"] == 0                          # vTarget == vEgo, in deadzone


def test_evaluate_flags_missing_vtarget():
  seg = _synth_seg()
  del seg["vTarget"]
  m = ev.evaluate([("a", seg)])
  assert not m["have_vtarget"]


def test_evaluate_returns_none_without_usable_segments():
  assert ev.evaluate([("short", _synth_seg(n=100))]) is None


def test_low_speed_braking_subset():
  seg = _synth_seg(n=1200, v=8.0, a_bias=-0.2)
  seg["aTarget"] = np.full(1200, -0.6)           # asked for more braking
  m = ev.evaluate([("a", seg)])
  assert m["ls_n"] == 1200
  assert m["ls_med"] == pytest.approx(0.4)
  assert ev.label_bias(m["ls_med"]) == "under-braking"
  assert ev.label_bias(-0.4) == "over-braking"


def test_burst_is_clean():
  seg = _synth_seg()
  clean = ev.clean_mask(seg)
  assert ev.burst_is_clean(_burst(5.0, 0.1), seg["cs_t"], clean)
  assert not ev.burst_is_clean(_burst(500.0, 0.1), seg["cs_t"], clean)
  assert not ev.burst_is_clean(_burst(5.0, 0.1), seg["cs_t"],
                               np.zeros(len(clean), dtype=bool))
