import pytest

from common import decode_stalk, CMD_STEP


def frame(counter=0, byte2=0x00):
  return bytes([0x00, 0xF0 | (counter & 0x0F), byte2, 0xFC])


@pytest.mark.parametrize("byte2,cmd", [
  (0x01, "plus1"), (0x02, "plus5"), (0x04, "minus1"), (0x08, "minus5"),
  (0x10, "cancel"), (0x40, "resume"), (0x80, "cancel_lever_up"),
])
def test_decodes_each_command(byte2, cmd):
  counter, decoded = decode_stalk(frame(counter=7, byte2=byte2))
  assert counter == 7
  assert decoded == cmd


def test_neutral_frame_decodes_to_none():
  counter, decoded = decode_stalk(frame(counter=14))
  assert counter == 14
  assert decoded is None


def test_cmd_step_signs():
  assert CMD_STEP == {"plus1": 1, "plus5": 5, "minus1": -1, "minus5": -5}
