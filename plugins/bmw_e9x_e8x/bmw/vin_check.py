"""Deferred VIN check for a cached fingerprint — DESIGN.md: "Fingerprint cache".

One OBD mode-09 VIN request to the DME on PT-CAN (0x7DF, answered on 0x7E8 as
a 3-frame ISO-TP reply), sent only after the startup window. Both IDs are on
the panda's BMW TX allow-list. Anything but a matching VIN — another VIN, a
negative response, or silence after every attempt — calls on_fail once, which
drops the cache so the next boot fingerprints live.
"""
REQUEST_ADDR = 0x7DF
FLOW_CONTROL_ADDR = 0x7E0
RESPONSE_ADDR = 0x7E8
REQUEST = bytes([0x02, 0x09, 0x02, 0, 0, 0, 0, 0])      # mode 09, PID 02: VIN
FLOW_CONTROL = bytes([0x30, 0x00, 0x0A, 0, 0, 0, 0, 0])  # clear to send, 10 ms


class VinCheck:
  def __init__(self, expected_vin, bus, on_fail, delay=30.0, timeout=1.0, attempts=3, retry=10.0):
    self.expected_vin = expected_vin
    self.bus = bus
    self.on_fail = on_fail
    self.delay_ns = int(delay * 1e9)
    self.timeout_ns = int(timeout * 1e9)
    self.retry_ns = int(retry * 1e9)
    self.attempts_left = attempts
    self.state = 'pending'
    self.start_ns = None
    self.next_request_ns = None
    self.deadline_ns = None
    self.flow_control_due = False
    self.payload = b''
    self.expected_len = 0

  def tx(self, now_nanos):
    """Frames to send this cycle, as (addr, dat, bus)."""
    if self.state != 'pending':
      return []
    if self.start_ns is None:
      self.start_ns = now_nanos
      self.next_request_ns = now_nanos + self.delay_ns
    if self.flow_control_due:
      self.flow_control_due = False
      return [(FLOW_CONTROL_ADDR, FLOW_CONTROL, self.bus)]
    if self.deadline_ns is not None and now_nanos >= self.deadline_ns:
      self.deadline_ns = None
      self.payload = b''
      if self.attempts_left == 0:
        self._finish('failed')
        return []
      self.next_request_ns = now_nanos + self.retry_ns
    if self.deadline_ns is None and now_nanos >= self.next_request_ns and self.attempts_left > 0:
      self.attempts_left -= 1
      self.deadline_ns = now_nanos + self.timeout_ns
      return [(REQUEST_ADDR, REQUEST, self.bus)]
    return []

  def rx(self, can_packets):
    """Feed raw CAN packets, [(nanos, [(address, dat, src), ...])].

    Frames are unpacked by position: card passes plain tuples
    (can_capnp_to_list), other callers opendbc's CanData namedtuple.
    """
    if self.state != 'pending' or self.deadline_ns is None:
      return
    for _, frames in can_packets:
      for address, dat, src in frames:
        if address == RESPONSE_ADDR and src == self.bus:
          self._frame(bytes(dat))

  def _frame(self, dat):
    kind = dat[0] >> 4
    if kind == 1:                                 # first frame
      self.expected_len = ((dat[0] & 0x0F) << 8) | dat[1]
      self.payload = dat[2:8]
      self.flow_control_due = True
    elif kind == 2 and self.payload:              # consecutive frame
      self.payload += dat[1:8]
      if len(self.payload) >= self.expected_len:
        self._complete(self.payload[:self.expected_len])
    # single frames (e.g. a 0x7F negative response) are left to time out

  def _complete(self, payload):
    self.deadline_ns = None
    if payload[:3] != bytes([0x49, 0x02, 0x01]):
      return
    vin = payload[3:].decode('ascii', errors='replace')
    self._finish('confirmed' if vin == self.expected_vin else 'failed')

  def _finish(self, state):
    self.state = state
    print(f"[bmw] cached fingerprint VIN check: {state}")
    if state == 'failed':
      self.on_fail()
