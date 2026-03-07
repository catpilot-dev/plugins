"""Tests for BMW plugin hook handlers — cruise ceiling memory and consecutive lane changes."""
import os
import sys
import pytest
from unittest.mock import MagicMock
from types import SimpleNamespace

# Add plugin dir to path so register module is importable
_PLUGIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PLUGIN_DIR not in sys.path:
  sys.path.insert(0, _PLUGIN_DIR)


@pytest.fixture(autouse=True)
def mock_deps(monkeypatch):
  """Mock opendbc/cereal imports for standalone testing."""
  from enum import IntEnum

  class LaneChangeState(IntEnum):
    off = 0
    preLaneChange = 1
    laneChangeStarting = 2
    laneChangeFinishing = 3

  class LaneChangeDirection(IntEnum):
    none = 0
    left = 1
    right = 2

  class Desire(IntEnum):
    none = 0
    laneChangeLeft = 1
    laneChangeRight = 2

  log_mock = MagicMock()
  log_mock.LaneChangeState = LaneChangeState
  log_mock.LaneChangeDirection = LaneChangeDirection
  log_mock.Desire = Desire

  cereal_mock = MagicMock()
  cereal_mock.log = log_mock

  mods = {
    'cereal': cereal_mock,
    'cereal.log': log_mock,
    'opendbc': MagicMock(),
    'opendbc.car': MagicMock(),
    'opendbc.car.structs': MagicMock(),
    'opendbc.car.docs_definitions': MagicMock(),
    'opendbc.car.common': MagicMock(),
    'opendbc.car.common.conversions': MagicMock(),
    'opendbc.car.fw_query_definitions': MagicMock(),
    'opendbc.can': MagicMock(),
  }
  for mod_name, mod_mock in mods.items():
    monkeypatch.setitem(sys.modules, mod_name, mod_mock)


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
    assert dh.lane_change_ll_prob == 1.0
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
    dh = SimpleNamespace(lane_change_state=log.LaneChangeState.laneChangeStarting, lane_change_ll_prob=0.5)

    register.on_post_lane_change(None, dh, cs, one_blinker=True, below_lane_change_speed=False, lane_change_prob=0.5)
    assert register._clc.consecutive_requested is True
    assert register._clc.desire_gap == 0  # ll_prob not faded yet

  def test_consecutive_triggers_when_committed(self, param_dir, reset_clc):
    from cereal import log
    import register
    (param_dir / 'ConsecutiveLaneChange').write_text('1')
    register._clc.consecutive_requested = True
    register._clc.prev_steering_button = True  # button already held

    cs = SimpleNamespace(steeringPressed=True, gasPressed=False)
    dh = SimpleNamespace(lane_change_state=log.LaneChangeState.laneChangeStarting, lane_change_ll_prob=0.005)

    register.on_post_lane_change(None, dh, cs, one_blinker=True, below_lane_change_speed=False, lane_change_prob=0.01)
    assert register._clc.desire_gap == 1

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
    """Simulate: first LC active → button press → ll_prob fades → gap → re-trigger."""
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
    assert register._clc.desire_gap == 0  # ll_prob still > 0.01

    # Step 2: ll_prob fades below threshold
    dh.lane_change_ll_prob = 0.005
    register.on_post_lane_change(None, dh, cs, one_blinker=True, below_lane_change_speed=False, lane_change_prob=0.01)
    assert register._clc.desire_gap == 1

    # Step 3: Desire override during gap frame
    desire = register.on_desire_post_update(log.Desire.laneChangeLeft, None, None, None)
    assert desire == log.Desire.none

    # Step 4: Pre-hook on next frame — gap countdown resets state
    register.on_pre_lane_change(None, dh, cs)
    assert dh.lane_change_state == log.LaneChangeState.laneChangeStarting
    assert dh.lane_change_ll_prob == 1.0
    assert dh.lane_change_timer == 0.0
    assert register._clc.desire_gap == 0

    # Step 5: Desire no longer overridden
    desire = register.on_desire_post_update(log.Desire.laneChangeLeft, None, None, None)
    assert desire == log.Desire.laneChangeLeft
