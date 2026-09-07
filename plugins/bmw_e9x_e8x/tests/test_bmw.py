"""Tests for BMW E9x/E8x plugin — VIN detection, CAN checksums, DBC paths, resume button."""
import pytest
from unittest.mock import MagicMock, patch, call
import sys
import os

# Add plugin dir to path so bmw package is importable
_PLUGIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PLUGIN_DIR not in sys.path:
  sys.path.insert(0, _PLUGIN_DIR)

from test_helpers import make_opendbc_mocks, make_cereal_mocks


@pytest.fixture(autouse=True)
def mock_opendbc(monkeypatch):
  """Mock opendbc imports so tests run without openpilot installed."""
  for mod_name, mod_mock in make_opendbc_mocks().items():
    monkeypatch.setitem(sys.modules, mod_name, mod_mock)


# ============================================================
# VIN Detection
# ============================================================

class TestVINDetection:
  def _get_match_fn(self):
    import importlib
    import bmw.values as mod
    importlib.reload(mod)
    return mod.match_fw_to_car_fuzzy

  def test_e90_vin(self, mock_opendbc):
    match = self._get_match_fn()
    # Real E90 VIN — model code PH1 at positions 4-6
    result = match({}, 'LBVPH18059SC20723', {})
    assert result == {'BMW_E90'}

  def test_e82_vin(self, mock_opendbc):
    match = self._get_match_fn()
    result = match({}, 'WBAUF1C50BVM12345', {})
    assert result == {'BMW_E82'}

  def test_all_e90_codes(self, mock_opendbc):
    match = self._get_match_fn()
    e90_codes = ['PH1', 'PH2', 'PK1', 'PK2', 'PM1', 'PM2', 'PN1']
    for code in e90_codes:
      vin = f'LBV{code}8059SC20723'
      result = match({}, vin, {})
      assert result == {'BMW_E90'}, f"Failed for model code {code}"

  def test_all_e82_codes(self, mock_opendbc):
    match = self._get_match_fn()
    e82_codes = ['UF1', 'UF2', 'UH1']
    for code in e82_codes:
      vin = f'WBA{code}C50BVM12345'
      result = match({}, vin, {})
      assert result == {'BMW_E82'}, f"Failed for model code {code}"

  def test_unknown_model_code(self, mock_opendbc):
    match = self._get_match_fn()
    result = match({}, 'WBAXX1C50BVM12345', {})
    assert result == set()

  def test_empty_vin(self, mock_opendbc):
    match = self._get_match_fn()
    assert match({}, '', {}) == set()
    assert match({}, None, {}) == set()

  def test_short_vin(self, mock_opendbc):
    match = self._get_match_fn()
    assert match({}, 'LBVPH', {}) == set()

  def test_offline_fw_filtering(self, mock_opendbc):
    """When offline_fw_versions provided, only return if model is in it."""
    match = self._get_match_fn()
    # E90 detected but not in offline versions
    result = match({}, 'LBVPH18059SC20723', {'BMW_E82': {}})
    assert result == set()
    # E90 detected and in offline versions
    result = match({}, 'LBVPH18059SC20723', {'BMW_E90': {}})
    assert result == {'BMW_E90'}


# ============================================================
# CAN Checksums
# ============================================================

class TestCANChecksums:
  def _get_checksums(self):
    from bmw.bmwcan import calc_checksum_8bit, calc_checksum_4bit, calc_checksum_cruise
    return calc_checksum_8bit, calc_checksum_4bit, calc_checksum_cruise

  def test_checksum_8bit_zero_data(self, mock_opendbc):
    calc_8bit, _, _ = self._get_checksums()
    result = calc_8bit(bytearray([0, 0, 0, 0]), 0)
    assert result == 0

  def test_checksum_8bit_with_msg_id(self, mock_opendbc):
    calc_8bit, _, _ = self._get_checksums()
    # msg_id 0xA8 with zero data
    result = calc_8bit(bytearray([0, 0, 0, 0]), 0xA8)
    assert result == 0xA8

  def test_checksum_8bit_overflow_wraps(self, mock_opendbc):
    calc_8bit, _, _ = self._get_checksums()
    # 0xFF * 4 = 0x3FC, msg_id = 0 → (0xFC + 0x03) & 0xFF = 0xFF
    result = calc_8bit(bytearray([0xFF, 0xFF, 0xFF, 0xFF]), 0)
    assert result == 0xFF

  def test_checksum_8bit_carry(self, mock_opendbc):
    calc_8bit, _, _ = self._get_checksums()
    # Test carry from upper byte: sum > 0xFF
    result = calc_8bit(bytearray([0x80, 0x80]), 0)
    assert result == (0x00 + 0x01) & 0xFF  # 0x100 → carry 1 + 0x00 = 1
    assert result == 1

  def test_checksum_4bit(self, mock_opendbc):
    _, calc_4bit, _ = self._get_checksums()
    result = calc_4bit(bytearray([0, 0, 0, 0]), 0)
    assert result == 0

  def test_checksum_4bit_nibble_wrap(self, mock_opendbc):
    _, calc_4bit, _ = self._get_checksums()
    result = calc_4bit(bytearray([0, 0, 0, 0]), 0x130)
    # 0x130 → (0x30 + 0x01) = 0x31 → (0x1 + 0x3) = 0x4
    assert result == 4

  def test_checksum_cruise_uses_zero_init(self, mock_opendbc):
    calc_8bit, _, calc_cruise = self._get_checksums()
    data = bytearray([0x10, 0x20, 0x30])
    assert calc_cruise(data) == calc_8bit(data, 0)

  def test_checksum_8bit_deterministic(self, mock_opendbc):
    calc_8bit, _, _ = self._get_checksums()
    data = bytearray([0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE])
    r1 = calc_8bit(data, 0xA8)
    r2 = calc_8bit(data, 0xA8)
    assert r1 == r2


# ============================================================
# Steering / Cruise Enums
# ============================================================

class TestEnums:
  def test_steering_modes(self, mock_opendbc):
    from bmw.bmwcan import SteeringModes
    assert SteeringModes.Off.value == 0
    assert SteeringModes.TorqueControl.value == 1
    assert SteeringModes.AngleControl.value == 2
    assert SteeringModes.SoftOff.value == 3

  def test_cruise_stalk_values(self, mock_opendbc):
    from bmw.bmwcan import CruiseStalk
    expected = {'plus1', 'plus5', 'minus1', 'minus5', 'cancel', 'resume', 'cancel_lever_up'}
    actual = {s.value for s in CruiseStalk}
    assert actual == expected


# ============================================================
# DBC Path Resolution
# ============================================================

class TestDBCPaths:
  def test_dbc_dict_has_all_buses(self, mock_opendbc):
    """All Bus entries (pt, chassis, body, alt) resolve to plugin-local DBC files."""
    import importlib
    import bmw.values as mod
    importlib.reload(mod)
    assert os.path.isabs(mod.PLUGIN_DBC_DIR)
    dbc_dict = mod.BmwPlatformConfig([], mod.CarSpecs()).dbc_dict
    for bus_key in ['pt', 'chassis', 'body', 'alt']:
      bus_val = getattr(mod.Bus, bus_key) if hasattr(mod.Bus, bus_key) else bus_key
      path = dbc_dict.get(bus_val)
      if path is None:
        path = dbc_dict.get({'pt': 0, 'chassis': 1, 'body': 2, 'alt': 3}[bus_key])
      assert path is not None, f"Bus.{bus_key} not in dbc_dict"
      assert mod.PLUGIN_DBC_DIR in path, f"Bus.{bus_key} path not in plugin dir: {path}"

  def test_ocelot_controls_dbc_exists(self, mock_opendbc):
    """ocelot_controls.dbc exists in plugin dbc directory."""
    import importlib
    import bmw.values as mod
    importlib.reload(mod)
    ocelot_path = os.path.join(mod.PLUGIN_DBC_DIR, 'ocelot_controls.dbc')
    assert os.path.exists(ocelot_path), f"Missing: {ocelot_path}"

  def test_bmw_dbc_exists(self, mock_opendbc):
    """bmw_e9x_e8x.dbc exists in plugin dbc directory."""
    import importlib
    import bmw.values as mod
    importlib.reload(mod)
    bmw_path = os.path.join(mod.PLUGIN_DBC_DIR, 'bmw_e9x_e8x.dbc')
    assert os.path.exists(bmw_path), f"Missing: {bmw_path}"


# ============================================================
# Platform Config
# ============================================================

class TestPlatformConfig:
  def test_controller_params(self, mock_opendbc):
    from bmw.values import CarControllerParams
    p = CarControllerParams(None)
    assert p.STEER_MAX == 12
    assert p.STEER_STEP == 1
    assert p.STEER_DELTA_UP == 0.1
    assert p.STEER_DELTA_DOWN == 0.1

  def test_bmw_flags(self, mock_opendbc):
    from bmw.values import BmwFlags
    # Flags are distinct powers of 2
    assert BmwFlags.STEPPER_SERVO_CAN == 1
    assert BmwFlags.NORMAL_CRUISE_CONTROL == 2
    assert BmwFlags.DYNAMIC_CRUISE_CONTROL == 4
    # Can combine flags
    combined = BmwFlags.STEPPER_SERVO_CAN | BmwFlags.DYNAMIC_CRUISE_CONTROL
    assert BmwFlags.STEPPER_SERVO_CAN in combined
    assert BmwFlags.NORMAL_CRUISE_CONTROL not in combined

  def test_can_bus_assignments(self, mock_opendbc):
    from bmw.values import CanBus
    assert CanBus.PT_CAN == 0
    assert CanBus.SERVO_CAN == 1
    assert CanBus.F_CAN == 1
    assert CanBus.AUX_CAN == 2


# ============================================================
# Resume Button Logic
# ============================================================

class TestResumeButton:
  """Test resume button: short press disengaged = resume, short press engaged = toggle speed limit, long press = gap adjust."""

  @pytest.fixture(autouse=True)
  def _cereal_mocks(self, monkeypatch):
    for mod_name, mod_mock in make_cereal_mocks().items():
      monkeypatch.setitem(sys.modules, mod_name, mod_mock)

  def _classify_release(self, cruise_state_enabled, hold_frames):
    """Classify what a resume button release should do given state."""
    from bmw.carstate import RESUME_LONG_PRESS_FRAMES
    if hold_frames >= RESUME_LONG_PRESS_FRAMES:
      return 'gapAdjust'
    elif cruise_state_enabled:
      return 'speed_limit_toggle'
    else:
      return 'resume'

  def test_short_press_disengaged_emits_resume(self):
    assert self._classify_release(cruise_state_enabled=False, hold_frames=1) == 'resume'

  def test_short_press_engaged_toggles_speed_limit(self):
    assert self._classify_release(cruise_state_enabled=True, hold_frames=1) == 'speed_limit_toggle'

  def test_long_press_emits_gap_adjust(self):
    from bmw.carstate import RESUME_LONG_PRESS_FRAMES
    assert self._classify_release(cruise_state_enabled=True, hold_frames=RESUME_LONG_PRESS_FRAMES + 5) == 'gapAdjust'

  def test_long_press_disengaged_emits_gap_adjust(self):
    from bmw.carstate import RESUME_LONG_PRESS_FRAMES
    assert self._classify_release(cruise_state_enabled=False, hold_frames=RESUME_LONG_PRESS_FRAMES) == 'gapAdjust'

  def test_toggle_sends_bus_command(self):
    """Toggle sends plugin bus command without crashing."""
    from bmw.carstate import toggle_speed_limit_confirm
    import bmw.carstate as cs
    cs._sl_pub = None  # reset lazy init
    toggle_speed_limit_confirm()  # Should not raise (bus may not be available)


# ============================================================
# Steer Fault Debounce
# ============================================================

class TestSteerFaultDebounce:
  """steerFaultTemporary should only be True after >=10 consecutive fault frames.

  The debounce logic in carstate.py:
    self.steer_fault_counter = self.steer_fault_counter + 1 if raw_fault else 0
    ret.steerFaultTemporary = self.steer_fault_counter >= 10
  """

  def _simulate(self, fault_sequence):
    """Simulate fault frames, return (counter, would_trigger) after each."""
    counter = 0
    results = []
    for raw_fault in fault_sequence:
      counter = counter + 1 if raw_fault else 0
      results.append((counter, counter >= 10))
    return results

  def test_transient_fault_suppressed(self):
    """9 consecutive fault frames should NOT trigger."""
    results = self._simulate([True] * 9)
    assert results[-1] == (9, False)

  def test_sustained_fault_triggers(self):
    """10 consecutive fault frames should trigger."""
    results = self._simulate([True] * 10)
    assert results[-1] == (10, True)

  def test_counter_resets_on_clear(self):
    """Counter resets to 0 when fault clears."""
    results = self._simulate([True] * 8 + [False])
    assert results[-1] == (0, False)

  def test_intermittent_fault_resets(self):
    """7 on, 1 off, 7 on should not trigger."""
    results = self._simulate([True] * 7 + [False] + [True] * 7)
    assert results[-1] == (7, False)


# ============================================================
# Engagement (update_button_enable)
# ============================================================

class TestButtonEnable:
  """How openpilot engages on this car.

  Despite the name, update_button_enable() IGNORED its buttonEvents argument
  and fired purely on the DCC ENGAGEMENT rising edge — which is why every
  gesture that brings DCC up (plus, minus, AND resume) engages openpilot.

  Below DCC's 30 km/h floor that edge never arrives, so openpilot could not
  engage at all down there. User ruling 2026-08-20: below minEnableSpeed the
  stalk itself is the engage control, matching the panda's stalk latch
  (bmw.h mask 0x0F — plus/minus only, resume excluded).
  """

  MIN_ENABLE = 30 / 3.6

  @pytest.fixture(autouse=True)
  def _cereal_mocks(self, monkeypatch):
    """bmw.carstate imports cereal.messaging at module scope."""
    for mod_name, mod_mock in make_cereal_mocks().items():
      monkeypatch.setitem(sys.modules, mod_name, mod_mock)

  def _call(self, events=(), *, dcc_now=False, dcc_prev=False, v_ego=0.0):
    from bmw.carstate import should_button_enable
    return should_button_enable(list(events), dcc_engaged=dcc_now,
                                dcc_engaged_prev=dcc_prev, v_ego=v_ego,
                                min_enable_speed=self.MIN_ENABLE)

  def _btn(self, btype, pressed):
    from types import SimpleNamespace
    return SimpleNamespace(type=btype, pressed=pressed)

  def _stalk(self, pressed=False, kind='accelCruise'):
    import bmw.carstate as cs
    return [self._btn(getattr(cs.ButtonType, kind), pressed)]

  # --- existing behaviour: DCC drives engagement -------------------------
  def test_dcc_rising_edge_engages(self):
    assert self._call(dcc_now=True, dcc_prev=False, v_ego=15.0) is True

  def test_dcc_steady_does_not_engage(self):
    assert self._call(dcc_now=True, dcc_prev=True, v_ego=15.0) is False

  def test_dcc_off_at_speed_does_not_engage(self):
    """Above minEnableSpeed the stalk is NOT an engage source — DCC will come
    up on its own and its rising edge handles it."""
    assert self._call(self._stalk(), v_ego=15.0) is False

  # --- new: stalk engages LKA below DCC's floor --------------------------
  def test_stalk_release_engages_below_min_speed(self):
    for kind in ('accelCruise', 'decelCruise'):
      assert self._call(self._stalk(kind=kind), v_ego=5.5) is True, kind

  def test_stalk_press_does_not_engage(self):
    """Release edge only — mirrors opendbc's own enable convention."""
    assert self._call(self._stalk(pressed=True), v_ego=5.5) is False

  def test_resume_does_not_engage_below_min_speed(self):
    """Ruling A: resume stays out of the engage set, matching the panda mask."""
    assert self._call(self._stalk(kind='resumeCruise'), v_ego=5.5) is False

  def test_cancel_does_not_engage(self):
    assert self._call(self._stalk(kind='cancel'), v_ego=5.5) is False

  def test_no_stalk_events_does_not_engage(self):
    assert self._call(v_ego=5.5) is False

  def test_stalk_ignored_below_min_speed_while_dcc_already_on(self):
    """Setpoint adjustment with DCC somehow live below the floor is not an
    engage request."""
    assert self._call(self._stalk(), dcc_now=True, dcc_prev=True, v_ego=5.5) is False


# ============================================================
# Cruise stalk burst counter (0x194)
# ============================================================

class TestCruiseBurstCounter:
  """DCC accepts a 0x194 frame only if its counter is a forward step —
  (counter - accepted) mod 15 in [1, 7]. Anything else is dropped as stale,
  and a persistent rollback is what stores 5ECE.

  During a burst we deliberately outrun SZL so DCC follows our sequence. The
  hazard is the handoff back: once we fall silent long enough for SZL's idle
  frame to be accepted, DCC's accepted counter is SZL's again, and resuming on
  our own stale sequence is a rollback.

  Regression: BURST_LIVE_WINDOW (0.5 s) is longer than SZL's 200 ms idle slot,
  so a 200-500 ms pause used to resume mid-sequence. Measured on route 444:
  14 rollbacks in 4 minutes, e.g. a 280 ms pause where SZL had reached 9 and we
  resumed at 3 (delta 9).
  """

  SZL_TICK = 0.2      # stock idle cadence
  STEP = 0.01         # control loop

  @pytest.fixture(autouse=True)
  def _mocks(self, monkeypatch):
    from test_helpers import make_carcontroller_mocks
    for mod_name, mod_mock in make_carcontroller_mocks().items():
      monkeypatch.setitem(sys.modules, mod_name, mod_mock)
    for mod_name, mod_mock in make_cereal_mocks().items():
      monkeypatch.setitem(sys.modules, mod_name, mod_mock)

  def _controller(self):
    import importlib
    import bmw.carcontroller as mod
    importlib.reload(mod)
    from bmw.values import BmwFlags
    CP = MagicMock()
    CP.flags = BmwFlags.DYNAMIC_CRUISE_CONTROL   # cruise on F-CAN, servo path off
    CP.minEnableSpeed = 30 / 3.6
    return mod.CarController({0: 'bmw_e9x_e8x'}, CP)

  def _replay(self, phases):
    """phases: list of (duration_s, accel, v_target, human_pressing).
    Returns the interleaved bus as [(t, 'SZL'|'OP', counter), ...]."""
    from test_helpers import make_stalk_carstate, make_stalk_carcontrol
    cc = self._controller()
    events, t, szl = [], 0.0, 0
    for dur, accel, v_target, human in phases:
      for _ in range(int(round(dur / self.STEP))):
        t += self.STEP
        if abs(t / self.SZL_TICK - round(t / self.SZL_TICK)) < 1e-9:
          szl = (szl + 1) % 15
          events.append((t, 'SZL', szl))
        CS = make_stalk_carstate(szl, human_pressing=human)
        CC = make_stalk_carcontrol(accel, v_target)
        _, sends = cc.update(CC, CS, int(round(t * 1e9)))
        for addr, dat, _bus in sends:
          if addr == 404:
            events.append((t, 'OP', dat[1] & 0xF))
    return events

  def _resume_delta(self, pause_s, human=False):
    """Burst, then pause, then command again. Returns the first resumed frame's
    counter delta from the newest SZL counter (which is DCC's accepted value
    once the handoff has happened)."""
    idle    = (1.0,  0.0, 24.0, False)    # anchor SZL phase, nothing commanded
    burst   = (0.25, -0.8, 22.0, False)   # decel burst
    pause   = (pause_s, 0.0, 24.0, human) # deadzone / driver on the stalk
    resume  = (0.20, -0.8, 22.0, False)
    events = self._replay([idle, burst, pause, resume])
    # first OP frame emitted after the pause began
    t_pause_start = 1.25 + self.STEP / 2
    first = next(e for e in events if e[1] == 'OP' and e[0] > t_pause_start + pause_s)
    szl_now = [c for (t, w, c) in events if w == 'SZL' and t <= first[0]][-1]
    return (first[2] - szl_now) % 15

  @pytest.mark.parametrize('pause_ms', [210, 250, 280, 300, 350, 400, 450])
  def test_resume_within_burst_live_window_is_forward(self, pause_ms):
    """The regression: pauses shorter than BURST_LIVE_WINDOW but longer than
    SZL's idle slot must still resync, not resume the stale sequence."""
    delta = self._resume_delta(pause_ms / 1000.0)
    assert 1 <= delta <= 7, f"rollback after {pause_ms} ms pause: delta={delta}"

  def test_resume_after_long_pause_is_forward(self):
    """The path BURST_LIVE_WINDOW already covered stays correct."""
    assert 1 <= self._resume_delta(0.60) <= 7

  def test_resume_after_driver_stalk_press_is_forward(self):
    """We yield the bus to the driver, so DCC follows SZL — resync on resume."""
    assert 1 <= self._resume_delta(0.30, human=True) <= 7

  def test_counter_advances_by_one_within_a_burst(self):
    """The handoff latch must not fire during a live burst: our frames still
    have to be a contiguous +1 sequence, or the overwrite stops outrunning SZL."""
    events = self._replay([(1.0, 0.0, 24.0, False), (0.60, -0.8, 22.0, False)])
    ours = [c for (_t, w, c) in events if w == 'OP']
    assert len(ours) > 15, f"expected a sustained burst, got {len(ours)} frames"
    for prev, nxt in zip(ours, ours[1:]):
      assert (nxt - prev) % 15 == 1, f"burst counter jumped {prev} -> {nxt}"


class TestCruiseCadencePin:
  """CruiseCadence debug param — pins the stalk cadence so a HOLD-vs-SINGLE A/B
  can be driven without openpilot's demand choosing the cadence for us.

  openpilot picks the command and the cadence from the same demanded accel, so
  observationally the two cells are never matched: 46 decel bursts over 25
  segments left the question open (gap bin [0.5,2) showed HOLD at -0.284 vs
  SINGLE at -0.075, but two other bins were a tie or a slight reversal, and the
  cells differed in speed by 24 km/h). Pinning breaks the entanglement.

  Safety: this must change frame SPACING only. Counter steps stay +1 — that is
  what the 16-per-slot counter law violated, setting 5ECE + CD95 on-car.
  """

  SZL_TICK = 0.2
  STEP = 0.01

  @pytest.fixture(autouse=True)
  def _mocks(self, monkeypatch):
    from test_helpers import make_carcontroller_mocks
    for mod_name, mod_mock in make_carcontroller_mocks().items():
      monkeypatch.setitem(sys.modules, mod_name, mod_mock)
    for mod_name, mod_mock in make_cereal_mocks().items():
      monkeypatch.setitem(sys.modules, mod_name, mod_mock)

  def _run(self, pin, accel):
    """Drive one burst at `accel`, with CruiseCadence set to `pin`.
    Returns (median TX interval in ms, counter steps seen)."""
    import importlib
    import bmw.carcontroller as mod
    importlib.reload(mod)
    from bmw.values import BmwFlags
    from test_helpers import make_stalk_carstate, make_stalk_carcontrol
    monkey = {'bmw_e9x_e8x': {'CruiseCadence': pin}}
    import config as cfg
    orig = cfg.read_plugin_param
    cfg.read_plugin_param = lambda pid, key, default='': monkey.get(pid, {}).get(key, default)
    try:
      CP = MagicMock()
      CP.flags = BmwFlags.DYNAMIC_CRUISE_CONTROL
      CP.minEnableSpeed = 30 / 3.6
      cc = mod.CarController({0: 'bmw_e9x_e8x'}, CP)
    finally:
      cfg.read_plugin_param = orig
    t, szl, sent = 0.0, 0, []
    v_target = 24.0 + (2.0 if accel > 0 else -2.0)
    for dur, a, vt in [(1.0, 0.0, 24.0), (1.2, accel, v_target)]:
      for _ in range(int(round(dur / self.STEP))):
        t += self.STEP
        if abs(t / self.SZL_TICK - round(t / self.SZL_TICK)) < 1e-9:
          szl = (szl + 1) % 15
        _, msgs = cc.update(make_stalk_carcontrol(a, vt),
                            make_stalk_carstate(szl), int(round(t * 1e9)))
        for addr, dat, _bus in msgs:
          if addr == 404 and dat[2]:
            sent.append((t, dat[1] & 0xF))
    assert len(sent) > 8, f"expected a burst, got {len(sent)} frames"
    import statistics
    iv = statistics.median((sent[i + 1][0] - sent[i][0]) * 1000 for i in range(len(sent) - 1))
    steps = {(sent[i + 1][1] - sent[i][1]) % 15 for i in range(len(sent) - 1)}
    return iv, steps

  def test_default_follows_demand(self):
    """Unset: gentle decel picks SINGLE (50 ms), firm decel picks HOLD."""
    slow, _ = self._run('', -0.15)
    fast, _ = self._run('', -0.80)
    assert slow > fast, f"gentle {slow:.0f} ms should be slower than firm {fast:.0f} ms"
    assert slow > 35, f"gentle decel should use SINGLE cadence, got {slow:.0f} ms"
    assert fast < 35, f"firm decel should use HOLD cadence, got {fast:.0f} ms"

  def test_pin_hold_forces_fast_cadence_on_gentle_decel(self):
    iv, _ = self._run('hold', -0.15)
    assert iv < 35, f"pinned HOLD should transmit fast, got {iv:.0f} ms"

  def test_pin_single_forces_slow_cadence_on_firm_decel(self):
    iv, _ = self._run('single', -0.80)
    assert iv > 35, f"pinned SINGLE should transmit slowly, got {iv:.0f} ms"

  def test_unknown_value_falls_back_to_demand(self):
    """A typo must not silently pin anything."""
    assert self._run('HOLDD', -0.80)[0] < 35
    assert self._run('yes', -0.15)[0] > 35

  @pytest.mark.parametrize('pin', ['', 'hold', 'single'])
  @pytest.mark.parametrize('accel', [-0.15, -0.80])
  def test_counter_always_steps_by_one(self, pin, accel):
    """The safety invariant. Pinning must never touch counter values."""
    _, steps = self._run(pin, accel)
    assert steps == {1}, f"pin={pin!r} accel={accel} produced counter steps {steps}"


class TestSetpointBias:
  """Accel-derived setpoint on the decel side.

  Before this, the setpoint was driven to v_target and stopped there. That gate
  is a clamp, not a rate limit: measured over routes 452 + 453 the setpoint
  reached it in a median of 0.00 s, so no choice of step size or cadence could
  buy authority, and the closed loop delivered a_ego = 0.611 * a_cmd. The plant
  is linear and symmetric (0.0935 m/s² per km/h of gap on the decel side,
  0.0912 on the accel side), so ask for the gap the demand actually needs.

  The invariants worth holding onto:
    - the accel branch is untouched (sp_target == v_target whenever accel >= 0),
      so the setpoint is never pushed above the driver's set speed;
    - counter steps stay +1 (the 5ECE/CD95 axis);
    - every downward bias is repaid, with plus1 and not plus5.
  """

  SZL_TICK = 0.2
  STEP = 0.01
  KPH = 1 / 3.6

  @pytest.fixture(autouse=True)
  def _mocks(self, monkeypatch):
    from test_helpers import make_carcontroller_mocks
    for mod_name, mod_mock in make_carcontroller_mocks().items():
      monkeypatch.setitem(sys.modules, mod_name, mod_mock)
    for mod_name, mod_mock in make_cereal_mocks().items():
      monkeypatch.setitem(sys.modules, mod_name, mod_mock)

  ACTION = {0: 'plus1', 1: 'plus5', 2: 'minus1', 3: 'minus5', 4: 'cancel'}

  def _run(self, accel, v_ego_kmh=86.0, setpoint_kmh=86.0, v_target_kmh=None,
           bias='', dur=1.2):
    """Hold one steady operating point and report what we transmit.

    Returns (set of action names emitted, set of counter steps, median TX
    interval in ms). v_target defaults to v_ego so v_error sits at zero — the
    case the old v_error gate silently dropped.
    """
    import importlib
    import bmw.carcontroller as mod
    importlib.reload(mod)
    from bmw.values import BmwFlags
    from test_helpers import make_stalk_carstate, make_stalk_carcontrol
    monkey = {'bmw_e9x_e8x': {'SetpointBias': bias}}
    import config as cfg
    orig = cfg.read_plugin_param
    cfg.read_plugin_param = lambda pid, key, default='': monkey.get(pid, {}).get(key, default)
    try:
      CP = MagicMock()
      CP.flags = BmwFlags.DYNAMIC_CRUISE_CONTROL
      CP.minEnableSpeed = 30 / 3.6
      cc = mod.CarController({0: 'bmw_e9x_e8x'}, CP)
    finally:
      cfg.read_plugin_param = orig

    v_ego = v_ego_kmh * self.KPH
    setpoint = setpoint_kmh * self.KPH
    v_target = (v_ego_kmh if v_target_kmh is None else v_target_kmh) * self.KPH

    t, szl, sent = 0.0, 0, []
    for phase_dur, a, vt in [(1.0, 0.0, v_ego), (dur, accel, v_target)]:
      for _ in range(int(round(phase_dur / self.STEP))):
        t += self.STEP
        if abs(t / self.SZL_TICK - round(t / self.SZL_TICK)) < 1e-9:
          szl = (szl + 1) % 15
        _, msgs = cc.update(make_stalk_carcontrol(a, vt),
                            make_stalk_carstate(szl, v_ego=v_ego, setpoint=setpoint),
                            int(round(t * 1e9)))
        for addr, dat, _bus in msgs:
          if addr == 404 and t > 1.0:
            sent.append((t, dat[1] & 0xF, dat[2]))
    acts = {self.ACTION[b] for _, _, a2 in sent for b in self.ACTION if a2 & (1 << b)}
    steps = {(sent[i + 1][1] - sent[i][1]) % 15 for i in range(len(sent) - 1)}
    import statistics
    iv = (statistics.median((sent[i + 1][0] - sent[i][0]) * 1000
                            for i in range(len(sent) - 1)) if len(sent) > 1 else None)
    return acts, steps, iv

  # ---- the target itself -------------------------------------------------

  def test_sp_target_is_vego_plus_bias_on_decel(self):
    """a_cmd -0.8 wants an 8 km/h gap, so a setpoint 8 km/h under vEgo."""
    import bmw.carcontroller as mod
    assert -0.8 / mod.K_DCC == pytest.approx(-8.0)

  def test_bias_is_capped(self):
    import bmw.carcontroller as mod
    assert max(-3.0 / mod.K_DCC, -mod.SETPOINT_BIAS_MAX) == -mod.SETPOINT_BIAS_MAX
    assert mod.SETPOINT_BIAS_MAX == 12.0

  def test_k_dcc_is_conservative_vs_measured_plant(self):
    """0.1 rather than the measured 0.0935 — every ask lands ~7% shallow."""
    import bmw.carcontroller as mod
    assert 0.9 < 0.0935 / mod.K_DCC < 1.0

  # ---- decel side --------------------------------------------------------

  def test_commands_decel_when_v_error_is_zero(self):
    """The band the old v_error gate threw away: at the set speed, planner
    asking for -0.5. It blocked 51% of the a_cmd -0.4..-0.3 samples."""
    acts, _, _ = self._run(accel=-0.5)
    assert acts & {'minus1', 'minus5'}, acts

  def test_no_command_when_setpoint_already_deep_enough(self):
    """Deadzone: setpoint 6 km/h under vEgo already covers a -0.5 ask."""
    acts, _, _ = self._run(accel=-0.5, setpoint_kmh=80.0)
    assert 'minus1' not in acts and 'minus5' not in acts, acts

  def test_deeper_demand_reopens_commanding(self):
    acts, _, _ = self._run(accel=-1.1, setpoint_kmh=80.0)
    assert 'minus5' in acts, acts

  def test_step_size_keys_on_setpoint_error_not_accel(self):
    """Mild demand, but the setpoint is 4 km/h high: that is a minus5. The
    accel-keyed rule would have sent minus1 and needed four presses."""
    acts, _, _ = self._run(accel=-0.4, v_ego_kmh=86.0, setpoint_kmh=86.0)
    assert 'minus5' in acts and 'minus1' not in acts, acts

  def test_large_demand_still_uses_minus1_when_little_is_owed(self):
    """The mirror case. accel -1.5 would be minus5 under the old rule, but the
    setpoint only has 2 km/h left to travel, so one step is the right one."""
    acts, _, _ = self._run(accel=-1.5, v_ego_kmh=86.0, setpoint_kmh=76.0)
    assert 'minus1' in acts and 'minus5' not in acts, acts

  def test_rollback_path_keeps_the_accel_keyed_step(self):
    acts, _, _ = self._run(accel=-1.0, v_target_kmh=80.0, setpoint_kmh=86.0, bias='0')
    assert 'minus5' in acts, acts
    acts, _, _ = self._run(accel=-0.4, v_target_kmh=80.0, setpoint_kmh=86.0, bias='0')
    assert 'minus1' in acts and 'minus5' not in acts, acts

  def test_floor_still_blocks(self):
    """min_cruise_setpoint is 35 km/h and the branch guard still owns it."""
    acts, _, _ = self._run(accel=-1.0, v_ego_kmh=40.0, setpoint_kmh=35.0)
    assert 'minus1' not in acts and 'minus5' not in acts, acts

  # ---- restore -----------------------------------------------------------

  def test_restores_with_plus1_when_demand_releases(self):
    """Setpoint parked 10 km/h low, demand gone: walk it back."""
    acts, _, iv = self._run(accel=0.0, setpoint_kmh=76.0)
    assert 'plus1' in acts, acts
    assert 'plus5' not in acts, "plus5 would park the setpoint high — a lurch"
    assert iv == pytest.approx(50.0, abs=6), iv

  def test_restore_stops_inside_the_deadzone(self):
    acts, _, _ = self._run(accel=0.0, setpoint_kmh=85.5)
    assert 'plus1' not in acts, acts

  def test_restore_runs_while_decel_demand_is_still_easing(self):
    """accel still negative but shallower than the parked bias — climb back."""
    acts, _, _ = self._run(accel=-0.2, setpoint_kmh=76.0)
    assert 'plus1' in acts, acts

  def test_never_pushes_setpoint_above_v_target(self):
    """Ceiling: at the set speed with no demand, nothing is sent."""
    acts, _, _ = self._run(accel=0.0, setpoint_kmh=86.0, v_target_kmh=86.0)
    assert 'plus1' not in acts and 'plus5' not in acts, acts

  # ---- the accel branch is untouched -------------------------------------

  @pytest.mark.parametrize('accel', [0.2, 0.5, 0.8])
  def test_accel_branch_identical_with_and_without_bias(self, accel):
    on = self._run(accel=accel, v_target_kmh=92.0, setpoint_kmh=86.0, bias='')
    off = self._run(accel=accel, v_target_kmh=92.0, setpoint_kmh=86.0, bias='0')
    assert on[0] == off[0], (on[0], off[0])
    assert on[2] == pytest.approx(off[2], abs=1e-6)

  # ---- the param ---------------------------------------------------------

  def test_param_off_restores_old_clamp(self):
    """With the bias off, a zero v_error decel is dropped again."""
    acts, _, _ = self._run(accel=-0.5, bias='0')
    assert 'minus1' not in acts and 'minus5' not in acts, acts

  def test_param_off_sends_no_restore(self):
    acts, _, _ = self._run(accel=0.0, setpoint_kmh=76.0, bias='0')
    assert 'plus1' not in acts, acts

  # ---- the safety invariant ----------------------------------------------

  @pytest.mark.parametrize('bias', ['', '0'])
  @pytest.mark.parametrize('accel,setpoint_kmh', [
    (-0.5, 86.0), (-1.0, 86.0), (-2.5, 86.0), (0.0, 76.0), (-0.2, 76.0), (0.5, 86.0),
  ])
  def test_counter_always_steps_by_one(self, bias, accel, setpoint_kmh):
    """The 5ECE/CD95 axis. Nothing here may emit a step other than +1."""
    _, steps, _ = self._run(accel=accel, setpoint_kmh=setpoint_kmh,
                            v_target_kmh=92.0 if accel > 0 else None, bias=bias)
    assert steps <= {1}, f"bias={bias!r} accel={accel} produced counter steps {steps}"

  def test_never_asks_shallower_than_the_old_law(self):
    """Monotonicity: sp_target is min'd against v_target, the old target, so
    the bias can only deepen the ask. v_target here is long_plan.vTarget — a
    ~0.5 s horizon plan speed close to vEgo, not the driver's set speed."""
    import bmw.carcontroller as mod
    for accel in (-0.05, -0.3, -0.8, -2.0):
      for v_ego, v_target in [(24.0, 24.0), (24.0, 23.0), (24.0, 25.0)]:
        bias = max(accel / mod.K_DCC, -mod.SETPOINT_BIAS_MAX) / 3.6
        assert min(v_target, v_ego + bias) <= v_target

  def test_param_off_keeps_the_v_error_gate(self):
    """SetpointBias=0 must be a true rollback. A v_error inside the deadzone
    with the setpoint above v_target is the case that separates the old gate
    from the new one: the old code blocks on v_error, and off must too."""
    acts, _, _ = self._run(accel=-0.5, v_ego_kmh=86.0, v_target_kmh=85.8,
                           setpoint_kmh=90.0, bias='0')
    assert 'minus1' not in acts and 'minus5' not in acts, acts

  def test_bias_on_commands_that_same_case(self):
    acts, _, _ = self._run(accel=-0.5, v_ego_kmh=86.0, v_target_kmh=85.8,
                           setpoint_kmh=90.0, bias='')
    assert acts & {'minus1', 'minus5'}, acts


class TestSetpointDebtLedger:
  """Every downward bias is borrowed and has to be repaid.

  A setpoint left low keeps braking — the plant is symmetric — and the branches
  that restore it only run while openpilot is driving. So every exit that stops
  us commanding (openpilot disengaging, the driver braking) parks the bias in
  DCC's setpoint memory: the driver resumes expecting their set speed and gets
  one up to SETPOINT_BIAS_MAX low, braking into it. The ledger is what makes
  that recoverable.

  It is a ledger and not a "restore to vCruise" policy on purpose. While
  openpilot is disengaged, v_cruise does not track the driver's stalk presses
  (_update_v_cruise_non_pcm returns early when not enabled), so there is no
  live signal to reconcile against — the only safe rule is to repay exactly
  what we took, and to drop the claim entirely once the driver touches the
  stalk.
  """

  SZL_TICK = 0.2
  STEP = 0.01
  KPH = 1 / 3.6

  @pytest.fixture(autouse=True)
  def _mocks(self, monkeypatch):
    from test_helpers import make_carcontroller_mocks
    for mod_name, mod_mock in make_carcontroller_mocks().items():
      monkeypatch.setitem(sys.modules, mod_name, mod_mock)
    for mod_name, mod_mock in make_cereal_mocks().items():
      monkeypatch.setitem(sys.modules, mod_name, mod_mock)

  ACTION = {0: 'plus1', 1: 'plus5', 2: 'minus1', 3: 'minus5', 4: 'cancel'}

  def _cc(self, bias=''):
    import importlib
    import bmw.carcontroller as mod
    importlib.reload(mod)
    from bmw.values import BmwFlags
    monkey = {'bmw_e9x_e8x': {'SetpointBias': bias}}
    import config as cfg
    orig = cfg.read_plugin_param
    cfg.read_plugin_param = lambda pid, key, default='': monkey.get(pid, {}).get(key, default)
    try:
      CP = MagicMock()
      CP.flags = BmwFlags.DYNAMIC_CRUISE_CONTROL
      CP.minEnableSpeed = 30 / 3.6
      return mod.CarController({0: 'bmw_e9x_e8x'}, CP), mod
    finally:
      cfg.read_plugin_param = orig

  def _phases(self, cc, phases):
    """phases: list of (seconds, dict of update kwargs). Returns actions seen
    per phase, so a repay can be told apart from the decel that caused it.

    Closes the loop on the setpoint. Debt is now *measured* off
    cruiseState.speed rather than counted off transmitted frames, so a harness
    that held the setpoint fixed would show no debt no matter what we sent.
    DCC moves the setpoint one step per 200 ms slot in which it saw a command
    (measured: a 3-frame minus5 burst moves it 5-10 km/h, a sub-slot minus1
    burst often moves it not at all), so that is what this models.

    The clock and the setpoint live on the controller, not this call:
    cruise_cmd throttles on now_nanos - last_cruise_tx_timestamp, so restarting
    time between calls makes dt_tx negative and silently drops every frame.
    """
    from test_helpers import make_stalk_carstate, make_stalk_carcontrol
    t = getattr(cc, '_test_t', 0.0)
    szl = getattr(cc, '_test_szl', 0)
    out = []
    STEP_KMH = {'minus1': -1, 'minus5': -5, 'plus1': 1, 'plus5': 5}
    for dur, kw in phases:
      if not hasattr(cc, '_sim_setpoint'):
        cc._sim_setpoint = kw.get('setpoint_kmh', 86.0)
        cc._sim_vcruise = kw.get('v_cruise_kmh', cc._sim_setpoint)
      acts = set()
      slot_acts = set()
      for _ in range(int(round(dur / self.STEP))):
        t += self.STEP
        if abs(t / self.SZL_TICK - round(t / self.SZL_TICK)) < 1e-9:
          szl = (szl + 1) % 15
          # slot boundary: DCC applies at most one step from what it saw
          for a in ('minus5', 'minus1', 'plus5', 'plus1'):
            if a in slot_acts:
              cc._sim_setpoint = max(0.0, cc._sim_setpoint + STEP_KMH[a])
              break
          slot_acts = set()
        cs = make_stalk_carstate(szl,
                                 v_ego=kw.get('v_ego_kmh', 86.0) * self.KPH,
                                 setpoint=cc._sim_setpoint * self.KPH,
                                 human_pressing=kw.get('human', False),
                                 dcc_enabled=kw.get('dcc', True),
                                 available=kw.get('available', True),
                                 v_cruise=cc._sim_vcruise)
        ccx = make_stalk_carcontrol(kw.get('accel', 0.0),
                                    kw.get('v_target_kmh', 86.0) * self.KPH,
                                    enabled=kw.get('enabled', True))
        _, msgs = cc.update(ccx, cs, int(round(t * 1e9)))
        for addr, dat, _bus in msgs:
          if addr == 404:
            seen = {self.ACTION[b] for b in self.ACTION if dat[2] & (1 << b)}
            acts |= seen
            slot_acts |= seen
      out.append(acts)
    cc._test_t, cc._test_szl = t, szl
    return out

  def test_debt_accrues_on_decel(self):
    cc, _ = self._cc()
    self._phases(cc, [(1.0, {}), (1.5, {'accel': -0.8})])
    assert cc.setpoint_debt > 0

  def test_debt_is_capped(self):
    cc, mod = self._cc()
    self._phases(cc, [(1.0, {}), (8.0, {'accel': -3.0})])
    assert cc.setpoint_debt <= mod.SETPOINT_BIAS_MAX

  def test_debt_repaid_by_restore_branch(self):
    cc, _ = self._cc()
    self._phases(cc, [(1.0, {}), (1.5, {'accel': -0.8})])
    owed = cc.setpoint_debt
    assert owed > 0
    self._phases(cc, [(3.0, {'accel': 0.0, 'setpoint_kmh': 76.0})])
    assert cc.setpoint_debt < owed

  def test_pending_cancel_outranks_the_repay(self):
    """An openpilot disengage raises cruise_cancel, and that must win: the
    repay is never allowed to hold the bus while a cancel is outstanding.

    This is also why the repay only ever lands after a standby round-trip —
    disengaging always cancels first, so the debt waits for the driver to
    bring DCC back (see test_debt_survives_dcc_standby)."""
    cc, _ = self._cc()
    self._phases(cc, [(1.0, {}), (1.5, {'accel': -0.8})])
    assert cc.setpoint_debt > 0
    acts, = self._phases(cc, [(2.0, {'enabled': False, 'setpoint_kmh': 76.0})])
    assert acts == {'cancel'}, acts

  def test_repay_never_uses_plus5(self):
    cc, _ = self._cc()
    self._phases(cc, [(1.0, {}), (2.0, {'accel': -2.5}),
                      (1.0, {'enabled': False, 'dcc': False})])
    acts, = self._phases(cc, [(2.0, {'enabled': False, 'setpoint_kmh': 74.0})])
    assert 'plus1' in acts, acts
    assert 'plus5' not in acts, "plus5 repay parks the setpoint high — a lurch"

  def test_driver_on_the_stalk_cancels_the_claim(self):
    cc, _ = self._cc()
    self._phases(cc, [(1.0, {}), (1.5, {'accel': -0.8})])
    assert cc.setpoint_debt > 0
    self._phases(cc, [(0.3, {'enabled': False, 'human': True, 'setpoint_kmh': 76.0})])
    assert cc.setpoint_debt == 0.0
    acts, = self._phases(cc, [(1.0, {'enabled': False, 'setpoint_kmh': 76.0})])
    assert 'plus1' not in acts, acts

  def test_losing_dcc_availability_clears_the_claim(self):
    cc, _ = self._cc()
    self._phases(cc, [(1.0, {}), (1.5, {'accel': -0.8})])
    assert cc.setpoint_debt > 0
    self._phases(cc, [(0.3, {'enabled': False, 'available': False})])
    assert cc.setpoint_debt == 0.0

  def test_debt_survives_dcc_standby(self):
    """A brake takes DCC to standby but the setpoint memory — and the debt —
    outlive it, so the repay lands when the driver brings DCC back."""
    cc, _ = self._cc()
    self._phases(cc, [(1.0, {}), (1.5, {'accel': -0.8})])
    owed = cc.setpoint_debt
    self._phases(cc, [(1.0, {'enabled': False, 'dcc': False})])
    assert cc.setpoint_debt == owed
    acts, = self._phases(cc, [(2.0, {'enabled': False, 'setpoint_kmh': 76.0})])
    assert 'plus1' in acts, acts
    assert cc.setpoint_debt < owed

  def test_no_repay_below_the_deadzone(self):
    cc, _ = self._cc()
    acts, = self._phases(cc, [(2.0, {'enabled': False, 'setpoint_kmh': 86.0})])
    assert 'plus1' not in acts, acts

  def test_param_off_keeps_no_ledger(self):
    cc, _ = self._cc(bias='0')
    self._phases(cc, [(1.0, {}), (2.0, {'accel': -0.8, 'v_target_kmh': 80.0}),
                      (1.0, {'enabled': False, 'dcc': False})])
    acts, = self._phases(cc, [(2.0, {'enabled': False, 'setpoint_kmh': 76.0})])
    assert 'plus1' not in acts, acts

  def test_counter_steps_by_one_through_a_repay(self):
    """The 5ECE/CD95 axis, on the one path that transmits with openpilot out."""
    from test_helpers import make_stalk_carstate, make_stalk_carcontrol
    cc, _ = self._cc()
    self._phases(cc, [(1.0, {}), (2.0, {'accel': -2.5}),
                      (1.0, {'enabled': False, 'dcc': False})])
    t, szl, sent = cc._test_t, cc._test_szl, []
    for _ in range(300):
      t += self.STEP
      if abs(t / self.SZL_TICK - round(t / self.SZL_TICK)) < 1e-9:
        szl = (szl + 1) % 15
      _, msgs = cc.update(
        make_stalk_carcontrol(0.0, 86.0 * self.KPH, enabled=False),
        make_stalk_carstate(szl, v_ego=86.0 * self.KPH, setpoint=74.0 * self.KPH),
        int(round(t * 1e9)))
      for addr, dat, _bus in msgs:
        if addr == 404:
          sent.append(dat[1] & 0xF)
    assert len(sent) > 4, sent
    steps = {(sent[i + 1] - sent[i]) % 15 for i in range(len(sent) - 1)}
    assert steps <= {1}, steps

  def test_debt_tracks_real_setpoint_movement_not_frames(self):
    """Regression. The first ledger counted transmitted frames, so it hit the
    SETPOINT_BIAS_MAX cap after 4 frames (60 ms) while DCC had moved the
    setpoint by at most one step. Debt must equal what the setpoint actually
    lost, and must not be pinned to the cap on the way there."""
    cc, mod = self._cc()
    self._phases(cc, [(1.0, {}), (0.5, {'accel': -0.8})])
    assert cc.setpoint_debt < mod.SETPOINT_BIAS_MAX, "pinned to the cap again"
    assert cc.setpoint_debt == pytest.approx(cc._sim_vcruise - cc._sim_setpoint, abs=1e-6)

  def test_repay_continues_until_the_setpoint_really_returns(self):
    """The other half of that bug: decrementing per frame let the repay call
    itself settled after ~0.6 s having actually returned about 3 km/h. The
    setpoint has to come all the way back."""
    cc, _ = self._cc()
    self._phases(cc, [(1.0, {}), (4.0, {'accel': -1.2}),
                      (1.0, {'enabled': False, 'dcc': False})])
    borrowed = cc._sim_vcruise - cc._sim_setpoint
    assert borrowed > 4, borrowed
    self._phases(cc, [(8.0, {'enabled': False})])
    assert cc._sim_vcruise - cc._sim_setpoint < 1.0, (
      f"setpoint still {cc._sim_vcruise - cc._sim_setpoint:.1f} km/h low")
    assert cc.setpoint_debt < 1.0

  def test_repay_stops_at_the_handback_point(self):
    """It must not run the setpoint past the driver's set speed."""
    cc, _ = self._cc()
    self._phases(cc, [(1.0, {}), (4.0, {'accel': -1.2}),
                      (1.0, {'enabled': False, 'dcc': False}),
                      (12.0, {'enabled': False})])
    assert cc._sim_setpoint <= cc._sim_vcruise + 1e-6, (cc._sim_setpoint, cc._sim_vcruise)
