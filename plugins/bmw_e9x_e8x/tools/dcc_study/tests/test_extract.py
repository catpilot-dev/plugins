import numpy as np
import zstandard

from common import STALK_ADDR


def _stalk_frame(counter, byte2):
  return bytes([0x00, 0xF0 | counter, byte2, 0xFC])


def make_rlog(path):
  from cereal import log

  msgs = []

  def evt(t):
    e = log.Event.new_message()
    e.logMonoTime = int(t * 1e9)
    return e

  e = evt(1.0)
  e.init("carState")
  e.carState.vEgo = 20.0
  e.carState.aEgo = 0.125   # exactly representable in float32 (cereal stores f32)
  e.carState.cruiseState.speed = 22.0
  e.carState.cruiseState.enabled = True
  e.carState.gasPressed = False
  e.carState.brakePressed = True
  msgs.append(e)

  e = evt(1.01)
  e.init("carControl")
  e.carControl.enabled = True
  e.carControl.actuators.accel = 0.5
  msgs.append(e)

  e = evt(1.02)
  cans = e.init("sendcan", 2)
  cans[0].address = STALK_ADDR
  cans[0].dat = _stalk_frame(3, 0x01)   # plus1
  cans[1].address = 0x22E               # unrelated address -> ignored
  cans[1].dat = b"\x00" * 4
  msgs.append(e)

  e = evt(1.03)
  cans = e.init("can", 2)
  cans[0].address = STALK_ADDR
  cans[0].dat = _stalk_frame(4, 0x08)   # minus5 action frame -> recorded
  cans[1].address = STALK_ADDR
  cans[1].dat = _stalk_frame(5, 0x00)   # neutral idle frame -> NOT recorded
  msgs.append(e)

  e = evt(1.04)
  e.init("livePose")
  e.livePose.orientationNED.y = 0.02
  msgs.append(e)

  raw = b"".join(m.to_bytes() for m in msgs)
  path.write_bytes(zstandard.ZstdCompressor().compress(raw))


def test_extract_segment(tmp_path):
  from extract import extract_segment, CMDS

  rlog = tmp_path / "rlog.zst"
  make_rlog(rlog)
  seg = extract_segment(rlog)

  assert seg["cs_t"].shape == (1,)
  assert seg["vEgo"][0] == 20.0 and seg["aEgo"][0] == 0.125
  assert seg["setpoint"][0] == 22.0
  assert seg["brake"][0] == 1.0 and seg["gas"][0] == 0.0
  assert seg["aTarget"][0] == 0.5 and seg["ctrlEnabled"][0] == 1.0
  assert list(seg["tx_cmd"]) == [CMDS.index("plus1")]
  assert list(seg["rx_cmd"]) == [CMDS.index("minus5")]  # neutral rx dropped
  assert abs(seg["pitch"][0] - 0.02) < 1e-6  # tolerate float32 round-trip
  assert np.all(np.diff(seg["cs_t"]) >= 0)


def test_corrupt_rlog_returns_none(tmp_path):
  from extract import extract_segment

  bad = tmp_path / "rlog.zst"
  bad.write_bytes(b"not zstd at all")
  assert extract_segment(bad) is None
