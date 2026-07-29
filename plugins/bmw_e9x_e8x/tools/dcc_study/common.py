"""Shared constants and CruiseControlStalk (0x194) frame decoding.

Wire format (DBC BO_ 404, 4 bytes):
  dat[0] checksum, dat[1] = (requests << 4) | counter,
  dat[2] action bits (see _CMD_BITS), dat[3] = 0xFC.
"""
from pathlib import Path

STALK_ADDR = 404  # 0x194

_CMD_BITS = (
  (0x01, "plus1"),
  (0x02, "plus5"),
  (0x04, "minus1"),
  (0x08, "minus5"),
  (0x10, "cancel"),
  (0x40, "resume"),
  (0x80, "cancel_lever_up"),
)

# kph moved per accepted tick, signed. Only these four are speed commands.
CMD_STEP = {"plus1": 1, "plus5": 5, "minus1": -1, "minus5": -5}

DATA_DIR = Path(__file__).resolve().parent / "data"
ROUTES_DIR = DATA_DIR / "routes"
EXTRACTED_DIR = DATA_DIR / "extracted"
PROFILES_DIR = DATA_DIR / "profiles"
REPORT_DIR = DATA_DIR / "report"


def decode_stalk(dat: bytes) -> tuple[int, str | None]:
  """Return (counter, command name or None for a neutral frame)."""
  counter = dat[1] & 0x0F
  for bit, name in _CMD_BITS:
    if dat[2] & bit:
      return counter, name
  return counter, None
