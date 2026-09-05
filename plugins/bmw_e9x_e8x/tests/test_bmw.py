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
