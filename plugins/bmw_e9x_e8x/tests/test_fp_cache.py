"""Persistent fingerprint cache and the deferred VIN check.

openpilot's own fingerprint cache never applies to this car: BMW defines no FW
queries, so CarParams.carFw is always empty and car_helpers' cache condition
(`len(cached_params.carFw) > 0`) is never met — and CarParamsCache is cleared on
every manager start anyway. So every boot, including every morning cold start,
ran the full VIN/FW sweep on PT-CAN and F-CAN with OBD multiplexing toggled.
The cache skips all of that; one OBD VIN request on PT-CAN, sent well after
startup, confirms the cache still belongs to this car. See DESIGN.md:
"Fingerprint cache".
"""
import importlib
import sys
from unittest.mock import MagicMock

import pytest

from test_helpers import make_carcontroller_mocks, make_cereal_mocks

VIN = 'LBVPH18059SC20723'
OTHER_VIN = 'WBAPH71090A123456'
PT = 0


@pytest.fixture(autouse=True)
def _env(monkeypatch, tmp_path):
  for mods in (make_carcontroller_mocks(), make_cereal_mocks()):
    for name, mod in mods.items():
      monkeypatch.setitem(sys.modules, name, mod)
  import config
  monkeypatch.setattr(config, 'PLUGINS_RUNTIME_DIR', str(tmp_path))
  monkeypatch.delenv('FINGERPRINT', raising=False)
  monkeypatch.delenv('SKIP_FW_QUERY', raising=False)
  import bmw.fp_cache as fp_cache
  importlib.reload(fp_cache)
  yield


def _fp_cache():
  import bmw.fp_cache as fp_cache
  return fp_cache


# ---- the cache file ---------------------------------------------------------

class TestCacheFile:
  def test_roundtrip(self):
    c = _fp_cache()
    c.save(VIN, 'BMW_E90')
    assert c.load() == {'vin': VIN, 'fingerprint': 'BMW_E90'}

  def test_missing_is_none(self):
    assert _fp_cache().load() is None

  @pytest.mark.parametrize('raw', ['', 'not json', '{"vin": "SHORT", "fingerprint": "BMW_E90"}',
                                   '{"vin": "LBVPH18059SC20723"}', '[1, 2]'])
  def test_garbage_is_none(self, raw):
    c = _fp_cache()
    import config
    config.write_plugin_param(c.PLUGIN_ID, c.KEY, raw)
    assert c.load() is None

  def test_clear_removes_it(self):
    c = _fp_cache()
    c.save(VIN, 'BMW_E90')
    c.clear()
    assert c.load() is None

  def test_clear_without_a_cache_is_fine(self):
    _fp_cache().clear()


# ---- the fingerprint wrapper ------------------------------------------------

FW_SOURCE = 'fw'
FIXED_SOURCE = 'fixed'
MODELS = {'BMW_E82', 'BMW_E90'}


class _Car:
  """Stands in for card's side of fingerprint(): records every bus action."""
  def __init__(self):
    self.sent = []
    self.obd = []
    self.recv_calls = 0

  def can_recv(self, *a, **kw):
    self.recv_calls += 1
    return []

  def can_send(self, msgs):
    self.sent.append(msgs)

  def set_obd(self, on):
    self.obd.append(on)


def _wrap(live_result=None):
  c = _fp_cache()
  orig = MagicMock(return_value=live_result)
  can_fp = MagicMock(return_value=(None, {0: {0x1A0: 8}}))
  wrapped = c.wrap_fingerprint(orig, can_fp, lambda vin: len(vin) == 17, FW_SOURCE, MODELS)
  return wrapped, orig, can_fp


class TestFingerprintWrapper:
  def test_cache_hit_matches_a_live_bmw_fingerprint(self):
    """Same tuple a live VIN fingerprint returns today: VIN-derived model, no FW,
    source fw, fuzzy (exact_match False)."""
    c = _fp_cache()
    c.save(VIN, 'BMW_E90')
    wrapped, orig, _ = _wrap()
    car = _Car()
    result = wrapped(car.can_recv, car.can_send, car.set_obd, 1, None)
    assert result == ('BMW_E90', {0: {0x1A0: 8}}, VIN, [], FW_SOURCE, False)
    orig.assert_not_called()

  def test_cache_hit_transmits_nothing(self):
    """The point of the cache: no VIN/FW sweep and no OBD multiplexing, which
    would detach the panda from F-CAN. set_obd(False) is a no-op unless an
    earlier session left multiplexing on."""
    c = _fp_cache()
    c.save(VIN, 'BMW_E90')
    wrapped, _, can_fp = _wrap()
    car = _Car()
    wrapped(car.can_recv, car.can_send, car.set_obd, 1, None)
    assert car.sent == []
    assert True not in car.obd
    can_fp.assert_called_once()        # passive CAN fingerprint still runs

  def test_cache_hit_arms_the_vin_check(self):
    c = _fp_cache()
    c.save(VIN, 'BMW_E90')
    wrapped, _, _ = _wrap()
    car = _Car()
    wrapped(car.can_recv, car.can_send, car.set_obd, 1, None)
    assert c.cached_vin == VIN

  def test_live_bmw_fingerprint_is_saved(self):
    wrapped, orig, _ = _wrap(('BMW_E90', {}, VIN, [], FW_SOURCE, False))
    car = _Car()
    result = wrapped(car.can_recv, car.can_send, car.set_obd, 1, None)
    orig.assert_called_once()
    assert result[0] == 'BMW_E90'
    c = _fp_cache()
    assert c.load() == {'vin': VIN, 'fingerprint': 'BMW_E90'}
    assert c.cached_vin is None        # live result: nothing to confirm

  @pytest.mark.parametrize('result', [
    ('TOYOTA_COROLLA', {}, VIN, [], FW_SOURCE, True),     # not ours
    ('BMW_E90', {}, 'SHORTVIN', [], FW_SOURCE, False),     # no usable VIN
    ('BMW_E90', {}, VIN, [], FIXED_SOURCE, True),          # forced, not identified
    (None, {}, VIN, [], FW_SOURCE, True),
  ])
  def test_only_identified_bmw_results_are_saved(self, result):
    wrapped, _, _ = _wrap(result)
    car = _Car()
    wrapped(car.can_recv, car.can_send, car.set_obd, 1, None)
    assert _fp_cache().load() is None

  @pytest.mark.parametrize('env', ['FINGERPRINT', 'SKIP_FW_QUERY'])
  def test_env_overrides_bypass_the_cache(self, env, monkeypatch):
    c = _fp_cache()
    c.save(VIN, 'BMW_E90')
    monkeypatch.setenv(env, '1')
    wrapped, orig, _ = _wrap(('BMW_E90', {}, VIN, [], FIXED_SOURCE, True))
    car = _Car()
    wrapped(car.can_recv, car.can_send, car.set_obd, 1, None)
    orig.assert_called_once()

  def test_cached_model_that_no_longer_exists_is_ignored(self):
    c = _fp_cache()
    c.save(VIN, 'BMW_E46')
    wrapped, orig, _ = _wrap(('BMW_E90', {}, VIN, [], FW_SOURCE, False))
    car = _Car()
    wrapped(car.can_recv, car.can_send, car.set_obd, 1, None)
    orig.assert_called_once()


# ---- the deferred VIN check -------------------------------------------------

S = 1_000_000_000


def _check(**kw):
  from bmw.vin_check import VinCheck
  fails = []
  chk = VinCheck(VIN, PT, on_fail=lambda: fails.append(1), **kw)
  return chk, fails


def _vin_frames(vin, src=PT):
  payload = bytes([0x49, 0x02, 0x01]) + vin.encode()
  ff = bytes([0x10, len(payload)]) + payload[:6]
  cf1 = bytes([0x21]) + payload[6:13]
  cf2 = bytes([0x22]) + payload[13:20]
  return [(0x7E8, ff, src), (0x7E8, cf1, src), (0x7E8, cf2, src)]


def _pkt(frames, t=0):
  from collections import namedtuple
  CanData = namedtuple('CanData', 'address dat src')
  return [(t, [CanData(*f) for f in frames])]


class TestVinCheck:
  def test_silent_through_the_startup_window(self):
    chk, _ = _check(delay=30.0)
    chk.tx(0)
    assert all(chk.tx(int(t * S)) == [] for t in (1, 10, 29.9))

  def test_one_obd_vin_request_on_pt_can_after_the_window(self):
    chk, _ = _check(delay=30.0)
    chk.tx(0)
    assert chk.tx(30 * S) == [(0x7DF, bytes([0x02, 0x09, 0x02, 0, 0, 0, 0, 0]), PT)]
    assert chk.tx(30 * S + S // 100) == []

  def test_first_frame_gets_a_flow_control(self):
    chk, _ = _check(delay=0.0)
    chk.tx(0)
    chk.tx(1)
    chk.rx(_pkt(_vin_frames(VIN)[:1]))
    assert chk.tx(2) == [(0x7E0, bytes([0x30, 0x00, 0x0A, 0, 0, 0, 0, 0]), PT)]
    assert chk.tx(3) == []

  def test_matching_vin_confirms(self):
    chk, fails = _check(delay=0.0)
    chk.tx(0)
    chk.tx(1)
    chk.rx(_pkt(_vin_frames(VIN)))
    assert chk.state == 'confirmed'
    assert fails == []
    assert chk.tx(100 * S) == []

  def test_other_vin_fails_once(self):
    chk, fails = _check(delay=0.0)
    chk.tx(0)
    chk.tx(1)
    chk.rx(_pkt(_vin_frames(OTHER_VIN)))
    assert chk.state == 'failed'
    chk.rx(_pkt(_vin_frames(OTHER_VIN)))
    assert fails == [1]

  def test_no_answer_retries_then_fails(self):
    chk, fails = _check(delay=0.0, timeout=1.0, attempts=3, retry=10.0)
    requests = [t for t in range(0, 40) if chk.tx(t * S)]
    assert requests == [0, 11, 22]
    assert chk.state == 'failed'
    assert fails == [1]

  def test_ignores_our_own_echo_and_other_buses(self):
    chk, fails = _check(delay=0.0)
    chk.tx(0)
    chk.tx(1)
    chk.rx(_pkt(_vin_frames(VIN, src=128)))   # TX echo of bus 0
    chk.rx(_pkt(_vin_frames(VIN, src=2)))     # gateway copy on K-CAN
    assert chk.state == 'pending'

  def test_negative_response_counts_as_no_answer(self):
    chk, fails = _check(delay=0.0, attempts=1)
    chk.tx(0)
    chk.tx(1)
    chk.rx(_pkt([(0x7E8, bytes([0x03, 0x7F, 0x09, 0x11, 0, 0, 0, 0]), PT)]))
    chk.tx(2 * S)
    assert chk.state == 'failed'


# ---- wiring -----------------------------------------------------------------

def _controller():
  import bmw.carcontroller as mod
  importlib.reload(mod)
  from bmw.values import BmwFlags
  CP = MagicMock()
  CP.flags = BmwFlags.DYNAMIC_CRUISE_CONTROL
  CP.minEnableSpeed = 30 / 3.6
  return mod.CarController({0: 'bmw_e9x_e8x'}, CP)


def _drive(cc, seconds):
  from test_helpers import make_stalk_carstate, make_stalk_carcontrol
  out = []
  for i in range(int(seconds * 100)):
    t = i * 10_000_000
    _, sends = cc.update(make_stalk_carcontrol(0.0, 24.0, enabled=False),
                         make_stalk_carstate(i // 20 % 15), t)
    out += [s for s in sends if s[0] == 0x7DF]
  return out


class TestWiring:
  def test_controller_sends_the_request_when_the_cache_was_used(self):
    _fp_cache().cached_vin = VIN
    cc = _controller()
    assert len(_drive(cc, 31)) == 1

  def test_controller_sends_nothing_after_a_live_fingerprint(self):
    _fp_cache().cached_vin = None
    cc = _controller()
    assert _drive(cc, 31) == []

  def test_interface_feeds_raw_frames_to_the_check(self, monkeypatch):
    # sys.modules, not `import opendbc.car.interfaces`: under the mocked parent
    # package that import binds an auto-attribute of the parent mock instead.
    class Base:
      def __init__(self, *a, **kw): pass
      def update(self, can_packets): return 'carstate'
    monkeypatch.setattr(sys.modules['opendbc.car.interfaces'], 'CarInterfaceBase', Base)
    import bmw.interface as mod
    importlib.reload(mod)
    ci = mod.CarInterface.__new__(mod.CarInterface)
    ci.CC = MagicMock()
    packets = _pkt(_vin_frames(VIN))
    assert ci.update(packets) == 'carstate'
    ci.CC.vin_check.rx.assert_called_once_with(packets)
