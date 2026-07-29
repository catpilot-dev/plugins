import numpy as np

from extract import CMDS
from bursts import find_bursts, is_contaminated, GAP_S

PLUS1 = CMDS.index("plus1")
PLUS5 = CMDS.index("plus5")


def seg(**over):
  base = {
    "cs_t": np.arange(0.0, 60.0, 0.01),
    "tx_t": np.array([]), "tx_cmd": np.array([], dtype=np.int8),
    "rx_t": np.array([]), "rx_cmd": np.array([], dtype=np.int8),
    "pose_t": np.array([]), "pitch": np.array([]),
  }
  n = len(base["cs_t"])
  base.update({"vEgo": np.full(n, 20.0), "aEgo": np.zeros(n),
               "setpoint": np.full(n, 22.0), "cruiseEnabled": np.ones(n),
               "gas": np.zeros(n), "brake": np.zeros(n)})
  base.update(over)
  return base


def tx(t0, n, interval, code):
  t = t0 + np.arange(n) * interval
  return t, np.full(n, code, dtype=np.int8)


def test_single_burst_hold_cadence():
  t, c = tx(10.0, 20, 0.025, PLUS5)          # 40 Hz
  bursts = find_bursts(seg(tx_t=t, tx_cmd=c))
  assert len(bursts) == 1
  b = bursts[0]
  assert (b.cmd, b.cadence, b.n_frames) == ("plus5", "hold", 20)
  assert b.t_start == 10.0


def test_single_cadence_classified():
  t, c = tx(10.0, 10, 0.05, PLUS1)           # 20 Hz
  assert find_bursts(seg(tx_t=t, tx_cmd=c))[0].cadence == "single"


def test_gap_splits_burst():
  t1, c1 = tx(10.0, 10, 0.05, PLUS1)
  t2, c2 = tx(t1[-1] + GAP_S + 0.1, 10, 0.05, PLUS1)
  bursts = find_bursts(seg(tx_t=np.concatenate([t1, t2]),
                           tx_cmd=np.concatenate([c1, c2])))
  assert len(bursts) == 2


def test_command_change_splits_burst():
  t1, c1 = tx(10.0, 10, 0.05, PLUS1)
  t2, c2 = tx(t1[-1] + 0.05, 10, 0.05, PLUS5)
  bursts = find_bursts(seg(tx_t=np.concatenate([t1, t2]),
                           tx_cmd=np.concatenate([c1, c2])))
  assert [b.cmd for b in bursts] == ["plus1", "plus5"]


def test_neutral_and_cancel_frames_ignored():
  t, c = tx(10.0, 10, 0.05, PLUS1)
  t_n = np.concatenate([t, t[-1] + np.arange(1, 6) * 0.05])
  c_n = np.concatenate([c, np.full(5, -1, dtype=np.int8)])   # trailing neutral
  bursts = find_bursts(seg(tx_t=t_n, tx_cmd=c_n))
  assert len(bursts) == 1 and bursts[0].n_frames == 10
  cancel = np.full(3, CMDS.index("cancel"), dtype=np.int8)
  assert find_bursts(seg(tx_t=t[:3], tx_cmd=cancel)) == []


def test_gas_contaminates():
  t, c = tx(10.0, 20, 0.025, PLUS1)
  s = seg(tx_t=t, tx_cmd=c)
  b = find_bursts(s)[0]
  assert not is_contaminated(b, s)
  s["gas"][(s["cs_t"] > 10.2) & (s["cs_t"] < 10.3)] = 1.0
  assert is_contaminated(b, s)


def test_brake_in_post_window_contaminates():
  t, c = tx(10.0, 20, 0.025, PLUS1)
  s = seg(tx_t=t, tx_cmd=c)
  b = find_bursts(s)[0]
  s["brake"][(s["cs_t"] > b.t_end + 1.0) & (s["cs_t"] < b.t_end + 1.2)] = 1.0
  assert is_contaminated(b, s)


def test_human_stalk_press_contaminates():
  t, c = tx(10.0, 20, 0.025, PLUS1)
  s = seg(tx_t=t, tx_cmd=c,
          rx_t=np.array([11.0]), rx_cmd=np.array([PLUS1], dtype=np.int8))
  # rx action frame 11.0 is >HUMAN_MATCH_S from every tx (last tx ~10.475)
  assert is_contaminated(find_bursts(s)[0], s)


def test_tx_echo_on_rx_is_not_human():
  t, c = tx(10.0, 20, 0.025, PLUS1)
  s = seg(tx_t=t, tx_cmd=c,
          rx_t=np.array([t[5] + 0.01]), rx_cmd=np.array([PLUS1], dtype=np.int8))
  assert not is_contaminated(find_bursts(s)[0], s)
