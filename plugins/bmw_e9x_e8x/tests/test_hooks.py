"""Tests for BMW plugin hook handlers — interface registration, cruise ceiling, consecutive lane changes."""
import os
import sys
import pytest
from unittest.mock import MagicMock, patch
from types import SimpleNamespace

# Add plugin dir to path so register module is importable
_PLUGIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PLUGIN_DIR not in sys.path:
  sys.path.insert(0, _PLUGIN_DIR)

from test_helpers import install_all_mocks


@pytest.fixture(autouse=True)
def mock_deps(monkeypatch):
  """Mock opendbc/cereal imports for standalone testing."""
  install_all_mocks(monkeypatch)


@pytest.fixture
def param_dir(tmp_path, monkeypatch):
  """Set up a temp data dir for plugin params."""
  import register
  data_dir = tmp_path / 'data'
  data_dir.mkdir()
  monkeypatch.setattr(register, '_PLUGIN_DIR', str(tmp_path))
  return data_dir


@pytest.fixture
def reset_clc():
  """Reset consecutive lane change state between tests."""
  import register
  register._clc.prev_steering_button = False
  register._clc.consecutive_requested = False
  register._clc.desire_gap = 0
  yield
  register._clc.prev_steering_button = False
  register._clc.consecutive_requested = False
  register._clc.desire_gap = 0


# ============================================================
# Interface Registration (opendbc patching)
# ============================================================

class TestRegisterInterfaces:
  def test_patches_interfaces_at_load(self, mock_deps):
    """_register_interfaces runs at module load and patches car_helpers.interfaces."""
    from opendbc.car.car_helpers import interfaces
    import importlib
    import register
    importlib.reload(register)
    assert any('E82' in str(k) for k in interfaces)
    assert any('E90' in str(k) for k in interfaces)

  def test_preserves_existing_interfaces(self, mock_deps):
    from opendbc.car.car_helpers import interfaces
    mock_iface = MagicMock()
    interfaces['HONDA_CIVIC'] = mock_iface
    import importlib
    import register
    importlib.reload(register)
    assert interfaces['HONDA_CIVIC'] is mock_iface

  def test_patches_torque_params(self, mock_deps):
    import importlib
    import opendbc.car.interfaces as _intf
    original_params = {'HONDA_CIVIC': {'LAT_ACCEL_FACTOR': 1.0}}
    _intf.get_torque_params = lambda: dict(original_params)

    import register
    importlib.reload(register)

    patched_params = _intf.get_torque_params()
    assert 'HONDA_CIVIC' in patched_params
    bmw_keys = [k for k in patched_params if 'BMW' in k.upper()]
    assert len(bmw_keys) >= 1

  def test_torque_params_toml_exists(self):
    """torque_params.toml exists and is parseable."""
    import tomllib
    toml_path = os.path.join(_PLUGIN_DIR, 'torque_params.toml')
    assert os.path.exists(toml_path)
    with open(toml_path, 'rb') as f:
      data = tomllib.load(f)
    assert 'legend' in data
    assert any('BMW' in k.upper() for k in data if k != 'legend')


# ============================================================
# Cruise Ceiling Memory
# ============================================================

class TestCruiseCeilingMemory:
  def test_restores_last_cruise(self, param_dir):
    import register
    helper = SimpleNamespace(v_cruise_kph=105, v_cruise_kph_last=80, v_cruise_cluster_kph=105)
    register.on_cruise_initialized(None, helper, None)
    assert helper.v_cruise_kph == 80
    assert helper.v_cruise_cluster_kph == 80

  def test_no_restore_on_first_engage(self, param_dir):
    import register
    helper = SimpleNamespace(v_cruise_kph=105, v_cruise_kph_last=0, v_cruise_cluster_kph=105)
    register.on_cruise_initialized(None, helper, None)
    assert helper.v_cruise_kph == 105

  def test_disabled_by_param(self, param_dir):
    (param_dir / 'CruiseCeilingMemory').write_text('0')
    import register
    helper = SimpleNamespace(v_cruise_kph=105, v_cruise_kph_last=80, v_cruise_cluster_kph=105)
    register.on_cruise_initialized(None, helper, None)
    assert helper.v_cruise_kph == 105

  def test_enabled_by_param(self, param_dir):
    (param_dir / 'CruiseCeilingMemory').write_text('1')
    import register
    helper = SimpleNamespace(v_cruise_kph=105, v_cruise_kph_last=80, v_cruise_cluster_kph=105)
    register.on_cruise_initialized(None, helper, None)
    assert helper.v_cruise_kph == 80

  def test_enabled_by_default_no_file(self, param_dir):
    """Default enabled when param file doesn't exist."""
    import register
    helper = SimpleNamespace(v_cruise_kph=105, v_cruise_kph_last=80, v_cruise_cluster_kph=105)
    register.on_cruise_initialized(None, helper, None)
    assert helper.v_cruise_kph == 80


# ============================================================
# Consecutive Lane Change — Pre Lane Change Hook
# ============================================================

class TestPreLaneChange:
  def test_gap_countdown_resets_state(self, param_dir, reset_clc):
    from cereal import log
    import register
    (param_dir / 'ConsecutiveLaneChange').write_text('1')

    dh = SimpleNamespace(
      lane_change_state=log.LaneChangeState.laneChangeFinishing,
      lane_change_ll_prob=0.5,
      lane_change_timer=3.0,
    )
    register._clc.desire_gap = 1

    register.on_pre_lane_change(None, dh, None)

    assert register._clc.desire_gap == 0
    assert dh.lane_change_state == log.LaneChangeState.laneChangeStarting
    assert dh.lane_change_ll_prob == 1.0  # reset so state machine doesn't immediately jump to finishing
    assert dh.lane_change_timer == 0.0
    assert register._clc.consecutive_requested is False

  def test_gap_no_action_when_zero(self, param_dir, reset_clc):
    from cereal import log
    import register
    (param_dir / 'ConsecutiveLaneChange').write_text('1')

    dh = SimpleNamespace(
      lane_change_state=log.LaneChangeState.laneChangeStarting,
      lane_change_ll_prob=0.8,
      lane_change_timer=1.0,
    )

    register.on_pre_lane_change(None, dh, None)
    assert dh.lane_change_state == log.LaneChangeState.laneChangeStarting
    assert dh.lane_change_ll_prob == 0.8

  def test_disabled_skips_gap(self, param_dir, reset_clc):
    from cereal import log
    import register
    (param_dir / 'ConsecutiveLaneChange').write_text('0')

    dh = SimpleNamespace(
      lane_change_state=log.LaneChangeState.laneChangeFinishing,
      lane_change_ll_prob=0.5,
      lane_change_timer=3.0,
    )
    register._clc.desire_gap = 1

    register.on_pre_lane_change(None, dh, None)
    # Gap should NOT be processed when disabled
    assert register._clc.desire_gap == 1
    assert dh.lane_change_state == log.LaneChangeState.laneChangeFinishing


# ============================================================
# Consecutive Lane Change — Post Lane Change Hook
# ============================================================

class TestPostLaneChange:
  def test_button_press_during_starting_sets_requested(self, param_dir, reset_clc):
    from cereal import log
    import register
    (param_dir / 'ConsecutiveLaneChange').write_text('1')

    cs = SimpleNamespace(steeringPressed=True, gasPressed=False)
    # ll_prob=0.4 < 0.5 threshold: press counts as consecutive (not the initiating press)
    dh = SimpleNamespace(lane_change_state=log.LaneChangeState.laneChangeStarting, lane_change_ll_prob=0.4)

    register.on_post_lane_change(None, dh, cs, one_blinker=True, below_lane_change_speed=False, lane_change_prob=0.5)
    assert register._clc.consecutive_requested is True
    assert register._clc.desire_gap == 0  # ll_prob not faded yet

  def test_consecutive_triggers_when_committed(self, param_dir, reset_clc):
    """consecutive_requested + ll_prob < 0.01 triggers desire_gap in on_pre_lane_change."""
    from cereal import log
    import register
    (param_dir / 'ConsecutiveLaneChange').write_text('1')
    register._clc.consecutive_requested = True

    cs = SimpleNamespace(steeringPressed=False, gasPressed=False, leftBlinker=True, rightBlinker=False)
    dh = SimpleNamespace(lane_change_state=log.LaneChangeState.laneChangeStarting, lane_change_ll_prob=0.005, lane_change_timer=3.0)

    # on_pre_lane_change intercepts before state machine
    register.on_pre_lane_change(None, dh, cs)
    assert register._clc.desire_gap == 1
    assert register._clc.consecutive_requested is False

  def test_no_trigger_when_ll_prob_not_faded(self, param_dir, reset_clc):
    from cereal import log
    import register
    (param_dir / 'ConsecutiveLaneChange').write_text('1')
    register._clc.consecutive_requested = True

    cs = SimpleNamespace(steeringPressed=True, gasPressed=False)
    dh = SimpleNamespace(lane_change_state=log.LaneChangeState.laneChangeStarting, lane_change_ll_prob=0.5)

    register.on_post_lane_change(None, dh, cs, one_blinker=True, below_lane_change_speed=False, lane_change_prob=0.5)
    assert register._clc.desire_gap == 0

  def test_button_during_finishing_triggers_gap(self, param_dir, reset_clc):
    from cereal import log
    import register
    (param_dir / 'ConsecutiveLaneChange').write_text('1')

    cs = SimpleNamespace(steeringPressed=True, gasPressed=False)
    dh = SimpleNamespace(lane_change_state=log.LaneChangeState.laneChangeFinishing, lane_change_ll_prob=0.8)

    register.on_post_lane_change(None, dh, cs, one_blinker=True, below_lane_change_speed=False, lane_change_prob=0.01)
    assert register._clc.desire_gap == 1

  def test_gas_pedal_does_not_trigger(self, param_dir, reset_clc):
    from cereal import log
    import register
    (param_dir / 'ConsecutiveLaneChange').write_text('1')

    cs = SimpleNamespace(steeringPressed=True, gasPressed=True)
    dh = SimpleNamespace(lane_change_state=log.LaneChangeState.laneChangeStarting, lane_change_ll_prob=0.005)

    register.on_post_lane_change(None, dh, cs, one_blinker=True, below_lane_change_speed=False, lane_change_prob=0.01)
    assert register._clc.consecutive_requested is False

  def test_resets_on_off_state(self, param_dir, reset_clc):
    from cereal import log
    import register
    (param_dir / 'ConsecutiveLaneChange').write_text('1')
    register._clc.consecutive_requested = True
    register._clc.desire_gap = 1

    cs = SimpleNamespace(steeringPressed=False, gasPressed=False)
    dh = SimpleNamespace(lane_change_state=log.LaneChangeState.off, lane_change_ll_prob=1.0)

    register.on_post_lane_change(None, dh, cs, one_blinker=False, below_lane_change_speed=False, lane_change_prob=0.0)
    assert register._clc.consecutive_requested is False
    assert register._clc.desire_gap == 0

  def test_below_speed_blocks_finishing_trigger(self, param_dir, reset_clc):
    from cereal import log
    import register
    (param_dir / 'ConsecutiveLaneChange').write_text('1')

    cs = SimpleNamespace(steeringPressed=True, gasPressed=False)
    dh = SimpleNamespace(lane_change_state=log.LaneChangeState.laneChangeFinishing, lane_change_ll_prob=0.8)

    register.on_post_lane_change(None, dh, cs, one_blinker=True, below_lane_change_speed=True, lane_change_prob=0.01)
    assert register._clc.desire_gap == 0

  def test_disabled_resets_button_state(self, param_dir, reset_clc):
    from cereal import log
    import register
    (param_dir / 'ConsecutiveLaneChange').write_text('0')
    register._clc.prev_steering_button = True

    cs = SimpleNamespace(steeringPressed=False, gasPressed=False)
    dh = SimpleNamespace(lane_change_state=log.LaneChangeState.laneChangeStarting, lane_change_ll_prob=0.5)

    register.on_post_lane_change(None, dh, cs, one_blinker=True, below_lane_change_speed=False, lane_change_prob=0.5)
    assert register._clc.prev_steering_button is False


# ============================================================
# Consecutive Lane Change — Desire Override
# ============================================================

class TestDesirePostUpdate:
  def test_overrides_during_gap(self, reset_clc):
    from cereal import log
    import register
    register._clc.desire_gap = 1

    result = register.on_desire_post_update(log.Desire.laneChangeLeft, None, None, None)
    assert result == log.Desire.none

  def test_passes_through_normally(self, reset_clc):
    from cereal import log
    import register

    result = register.on_desire_post_update(log.Desire.laneChangeRight, None, None, None)
    assert result == log.Desire.laneChangeRight


# ============================================================
# Full Consecutive Lane Change Sequence
# ============================================================

class TestConsecutiveSequence:
  def test_full_double_lane_change(self, param_dir, reset_clc):
    """Simulate: first LC active → button press → ll_prob fades → immediate re-trigger."""
    from cereal import log
    import register
    (param_dir / 'ConsecutiveLaneChange').write_text('1')

    dh = SimpleNamespace(
      lane_change_state=log.LaneChangeState.laneChangeStarting,
      lane_change_ll_prob=0.3,
      lane_change_timer=0.2,
    )

    # Step 1: Button press during laneChangeStarting (rising edge)
    cs = SimpleNamespace(steeringPressed=True, gasPressed=False)
    register.on_post_lane_change(None, dh, cs, one_blinker=True, below_lane_change_speed=False, lane_change_prob=0.5)
    assert register._clc.consecutive_requested is True

    # Step 2: ll_prob fades — on_pre_lane_change intercepts before state machine
    dh.lane_change_ll_prob = 0.005
    dh.lane_change_timer = 3.0
    cs_blinker = SimpleNamespace(steeringPressed=True, gasPressed=False, leftBlinker=False, rightBlinker=True)
    register.on_pre_lane_change(None, dh, cs_blinker)
    assert register._clc.desire_gap == 1

    # Step 3: Desire overridden to none during gap frame
    desire = register.on_desire_post_update(log.Desire.laneChangeRight, None, None, None)
    assert desire == log.Desire.none

    # Step 4: next frame, desire_gap counts down, resets state
    register.on_pre_lane_change(None, dh, cs_blinker)
    assert dh.lane_change_state == log.LaneChangeState.laneChangeStarting
    assert dh.lane_change_ll_prob == 1.0
    assert dh.lane_change_timer == 0.0

  def test_frame_by_frame_consecutive_lc(self, param_dir, reset_clc):
    """Simulate full desire_helper state machine frame-by-frame with hooks.

    Verifies the consecutive LC skips laneChangeFinishing entirely:
      laneChangeStarting → (button press) → (ll_prob fades) → desire gap →
      laneChangeStarting (fresh) — no finishing/preLaneChange in between.
    """
    from cereal import log
    import register
    (param_dir / 'ConsecutiveLaneChange').write_text('1')

    DT = 0.05  # 20Hz model rate
    state = log.LaneChangeState.off
    ll_prob = 1.0
    timer = 0.0
    direction = log.LaneChangeDirection.none

    states_seen = []

    def run_frame(steer_pressed, blinker_on, lane_change_prob=0.0):
      nonlocal state, ll_prob, timer, direction
      cs = SimpleNamespace(
        steeringPressed=steer_pressed, gasPressed=False,
        leftBlinker=False, rightBlinker=blinker_on,
        steeringTorque=-1.0 if steer_pressed and blinker_on else 0.0,
        vEgo=20.0, leftBlindspot=False, rightBlindspot=False,
      )
      dh = SimpleNamespace(
        lane_change_state=state, lane_change_ll_prob=ll_prob,
        lane_change_timer=timer, lane_change_direction=direction,
        prev_one_blinker=blinker_on,
      )

      # Pre-hook (runs BEFORE state machine)
      register.on_pre_lane_change(None, dh, cs)

      # Simplified state machine (mirrors desire_helper.py)
      one_blinker = cs.rightBlinker
      if dh.lane_change_state == log.LaneChangeState.laneChangeStarting:
        dh.lane_change_ll_prob = max(dh.lane_change_ll_prob - 2 * DT, 0.0)
        if lane_change_prob < 0.02 and dh.lane_change_ll_prob < 0.01:
          dh.lane_change_state = log.LaneChangeState.laneChangeFinishing
      elif dh.lane_change_state == log.LaneChangeState.laneChangeFinishing:
        dh.lane_change_ll_prob = min(dh.lane_change_ll_prob + DT, 1.0)
        if dh.lane_change_ll_prob > 0.99:
          if one_blinker:
            dh.lane_change_state = log.LaneChangeState.preLaneChange
          else:
            dh.lane_change_state = log.LaneChangeState.off
          dh.lane_change_direction = log.LaneChangeDirection.none

      if dh.lane_change_state in (log.LaneChangeState.off, log.LaneChangeState.preLaneChange):
        dh.lane_change_timer = 0.0
      else:
        dh.lane_change_timer += DT

      # Post-hook
      register.on_post_lane_change(None, dh, cs, one_blinker=one_blinker,
                                    below_lane_change_speed=False, lane_change_prob=lane_change_prob)

      state = dh.lane_change_state
      ll_prob = dh.lane_change_ll_prob
      timer = dh.lane_change_timer
      direction = getattr(dh, 'lane_change_direction', direction)

      sname = str(state).split('.')[-1]
      states_seen.append(sname)
      return sname

    # --- Sequence: blinker on, first press initiates LC ---
    state = log.LaneChangeState.laneChangeStarting
    direction = log.LaneChangeDirection.right
    ll_prob = 1.0
    timer = 0.0

    # Frames 1-6: ll_prob fading from 1.0 (no button yet)
    for _ in range(6):
      run_frame(False, True)
    assert ll_prob < 0.5  # ll_prob = 1.0 - 6*0.1 = 0.4

    # Frame 7: Button press during starting (ll_prob < 0.5)
    run_frame(True, True)
    assert register._clc.consecutive_requested is True

    # Frames 8-12: ll_prob continues fading toward 0
    for _ in range(5):
      s = run_frame(False, True)

    # At some point, on_pre_lane_change should have intercepted and set desire_gap
    # The state should NOT have gone through laneChangeFinishing
    finishing_count = sum(1 for s in states_seen if s == 'laneChangeFinishing')
    starting_count = sum(1 for s in states_seen if s == 'laneChangeStarting')

    # Key assertion: no finishing states — consecutive LC skipped it entirely
    assert finishing_count == 0, f'Expected 0 finishing states, got {finishing_count}. States: {states_seen}'
    # All frames stayed in laneChangeStarting (enum value '2')
    assert all(s == '2' for s in states_seen), f'Expected all laneChangeStarting. States: {states_seen}'

  def test_race_condition_starting_to_finishing(self, param_dir, reset_clc):
    """State machine transitions starting→finishing in same frame as post hook.

    consecutive_requested was set during laneChangeStarting, but by the time
    on_post_lane_change runs the state is already laneChangeFinishing. The
    finishing branch must honor the pending consecutive_requested flag.
    """
    from cereal import log
    import register
    (param_dir / 'ConsecutiveLaneChange').write_text('1')

    # Button was pressed earlier during starting, consecutive_requested is set
    register._clc.consecutive_requested = True
    register._clc.prev_steering_button = False  # button released

    dh = SimpleNamespace(
      lane_change_state=log.LaneChangeState.laneChangeFinishing,
      lane_change_ll_prob=0.0,
      lane_change_timer=5.0,
    )

    # No rising_edge (button not pressed this frame), but consecutive_requested is pending
    cs = SimpleNamespace(steeringPressed=False, gasPressed=False)
    register.on_post_lane_change(None, dh, cs, one_blinker=True, below_lane_change_speed=False, lane_change_prob=0.01)

    # Should trigger gap via the pending consecutive_requested
    assert register._clc.desire_gap == 1
    assert register._clc.consecutive_requested is False
