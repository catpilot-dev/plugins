"""Tests for the drive tracker."""
import json
import os
import pytest
import sys
import time
from types import SimpleNamespace
from unittest.mock import MagicMock, patch


@pytest.fixture(autouse=True)
def mock_openpilot(monkeypatch):
  """Mock openpilot imports."""
  for mod in ['openpilot', 'openpilot.selfdrive', 'openpilot.selfdrive.ui',
              'openpilot.selfdrive.ui.ui_state', 'openpilot.common',
              'openpilot.common.swaglog', 'cereal', 'cereal.messaging']:
    monkeypatch.setitem(sys.modules, mod, MagicMock())


@pytest.fixture
def tracker_module(tmp_path, monkeypatch, mock_openpilot):
  """Import drive_tracker with mocked deps and temp file path."""
  import importlib

  # Mock ui_state with callback registration
  ui_state_mock = MagicMock()
  ui_state_mock.started = False
  callbacks = []
  ui_state_mock.add_offroad_transition_callback = lambda cb: callbacks.append(cb)
  sys.modules['openpilot.selfdrive.ui.ui_state'].ui_state = ui_state_mock

  mod_path = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'drive_tracker.py')

  spec = importlib.util.spec_from_file_location('drive_tracker', mod_path)
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)

  # Patch output file path after exec
  last_drive_file = str(tmp_path / '.last_drive.json')
  mod.LAST_DRIVE_FILE = last_drive_file

  mod._callbacks = callbacks
  mod._ui_state_mock = ui_state_mock
  mod._last_drive_file = last_drive_file
  return mod


class MockSM(dict):
  def __init__(self, v_ego=0.0, enabled=False, gps_updated=False, gps_lat=0.0, gps_lng=0.0, gps_flags=0,
               device_state_updated=True):
    super().__init__({
      'carState': SimpleNamespace(vEgo=v_ego),
      'selfdriveState': SimpleNamespace(enabled=enabled),
      'gpsLocationExternal': SimpleNamespace(flags=gps_flags, latitude=gps_lat, longitude=gps_lng),
    })
    self.updated = {'gpsLocationExternal': gps_updated, 'deviceState': device_state_updated}


def make_sm(**kw):
  return MockSM(**kw)


def ready_tick(t, elapsed=0.5):
  """Set tracker timestamp so next tick() measures correct dt."""
  t._last_tick = time.monotonic() - elapsed


# ============================================================
# DriveTracker
# ============================================================

class TestDriveTracker:
  def test_inactive_by_default(self, tracker_module):
    t = tracker_module.DriveTracker()
    assert t._active is False

  def test_tick_noop_when_inactive(self, tracker_module):
    t = tracker_module.DriveTracker()
    sm = make_sm(v_ego=30.0)
    t.tick(sm)
    assert t._distance_m == 0.0

  def test_reset_activates(self, tracker_module):
    t = tracker_module.DriveTracker()
    t._reset()
    assert t._active is True
    assert t._distance_m == 0.0

  def test_tick_accumulates_distance(self, tracker_module):
    t = tracker_module.DriveTracker()
    t._reset()

    sm = make_sm(v_ego=10.0)  # 10 m/s
    # Simulate 0.05s frame
    ready_tick(t)
    t.tick(sm)

    assert t._distance_m == pytest.approx(5.0, abs=0.5)  # 10 m/s * 0.5s

  def test_tick_accumulates_engaged(self, tracker_module):
    t = tracker_module.DriveTracker()
    t._reset()

    sm = make_sm(v_ego=10.0, enabled=True)
    ready_tick(t)
    t.tick(sm)

    assert t._engaged_s > 0

  def test_tick_not_engaged(self, tracker_module):
    t = tracker_module.DriveTracker()
    t._reset()

    sm = make_sm(v_ego=10.0, enabled=False)
    ready_tick(t)
    t.tick(sm)

    assert t._engaged_s == 0.0

  def test_tick_captures_gps(self, tracker_module):
    t = tracker_module.DriveTracker()
    t._reset()

    sm = make_sm(v_ego=10.0, gps_updated=True, gps_lat=39.9, gps_lng=116.4, gps_flags=1)
    ready_tick(t)
    t.tick(sm)

    assert t._has_gps is True
    assert t._start_lat == 39.9
    assert t._end_lat == 39.9

  def test_gps_start_fixed_end_updates(self, tracker_module):
    t = tracker_module.DriveTracker()
    t._reset()

    # First GPS fix
    sm = make_sm(v_ego=10.0, gps_updated=True, gps_lat=39.9, gps_lng=116.4, gps_flags=1)
    ready_tick(t)
    t.tick(sm)

    # Second GPS fix at different location
    sm = make_sm(v_ego=10.0, gps_updated=True, gps_lat=40.0, gps_lng=116.5, gps_flags=1)
    ready_tick(t)
    t.tick(sm)

    assert t._start_lat == 39.9  # unchanged
    assert t._end_lat == 40.0    # updated

  def test_gps_no_fix_ignored(self, tracker_module):
    t = tracker_module.DriveTracker()
    t._reset()

    sm = make_sm(v_ego=10.0, gps_updated=True, gps_lat=39.9, gps_lng=116.4, gps_flags=0)
    ready_tick(t)
    t.tick(sm)

    assert t._has_gps is False

  def test_tick_skips_without_device_state(self, tracker_module):
    t = tracker_module.DriveTracker()
    t._reset()

    sm = make_sm(v_ego=10.0, device_state_updated=False)
    ready_tick(t)
    t.tick(sm)

    assert t._distance_m == 0.0  # skipped — no deviceState update

  def test_dt_clamped(self, tracker_module):
    t = tracker_module.DriveTracker()
    t._reset()

    sm = make_sm(v_ego=10.0)
    # Simulate 5s pause (should be clamped to 0.5s)
    t._last_tick = time.monotonic() - 5.0
    t.tick(sm)

    assert t._distance_m == pytest.approx(5.0, abs=0.5)  # 10 * 0.5

  def test_save_writes_json(self, tracker_module):
    t = tracker_module.DriveTracker()
    t._reset()
    t._duration_s = 120.0
    t._distance_m = 5000.0
    t._engaged_s = 100.0
    t._has_gps = True
    t._start_lat = 39.9
    t._start_lng = 116.4
    t._end_lat = 40.0
    t._end_lng = 116.5

    t._save()

    data = json.loads(open(tracker_module._last_drive_file).read())
    assert data['version'] == 1
    assert data['duration_s'] == 120.0
    assert data['distance_m'] == 5000.0
    assert data['engaged_s'] == 100.0
    assert data['has_gps'] is True
    assert data['start_lat'] == 39.9
    assert data['end_lat'] == 40.0

  def test_save_skips_short_drives(self, tracker_module):
    t = tracker_module.DriveTracker()
    t._reset()
    t._duration_s = 3.0
    t._save()

    assert not os.path.exists(tracker_module._last_drive_file)

  def test_save_deactivates(self, tracker_module):
    t = tracker_module.DriveTracker()
    t._reset()
    t._duration_s = 60.0
    t._save()
    assert t._active is False

  def test_transition_onroad_resets(self, tracker_module):
    t = tracker_module.DriveTracker()
    tracker_module._ui_state_mock.started = True
    t._on_transition()
    assert t._active is True

  def test_transition_offroad_saves(self, tracker_module):
    t = tracker_module.DriveTracker()
    t._reset()
    t._duration_s = 60.0
    t._distance_m = 1000.0
    tracker_module._ui_state_mock.started = False
    t._on_transition()
    assert t._active is False
    assert os.path.exists(tracker_module._last_drive_file)

  def test_summary_returns_stats_when_active(self, tracker_module):
    t = tracker_module.DriveTracker()
    t._reset()
    t._distance_m = 500.0
    t._duration_s = 30.0
    t._engaged_s = 20.0

    s = t.summary
    assert s['distance_m'] == 500.0
    assert s['duration_s'] == 30.0
    assert s['engaged_s'] == 20.0

  def test_summary_returns_none_when_inactive(self, tracker_module):
    t = tracker_module.DriveTracker()
    assert t.summary is None

  def test_trace_first_gps_point(self, tracker_module):
    t = tracker_module.DriveTracker()
    t._reset()

    sm = make_sm(v_ego=10.0, gps_updated=True, gps_lat=39.9, gps_lng=116.4, gps_flags=1)
    ready_tick(t)
    t.tick(sm)

    assert len(t._trace) == 1
    assert t._trace[0] == [39.9, 116.4]

  def test_trace_skips_nearby_points(self, tracker_module):
    t = tracker_module.DriveTracker()
    t._reset()

    # First point
    sm = make_sm(v_ego=10.0, gps_updated=True, gps_lat=39.9, gps_lng=116.4, gps_flags=1)
    ready_tick(t)
    t.tick(sm)

    # Second point very close (< 50m)
    sm = make_sm(v_ego=10.0, gps_updated=True, gps_lat=39.9001, gps_lng=116.4001, gps_flags=1)
    ready_tick(t)
    t.tick(sm)

    assert len(t._trace) == 1  # too close, not added

  def test_trace_adds_distant_points(self, tracker_module):
    t = tracker_module.DriveTracker()
    t._reset()

    # First point
    sm = make_sm(v_ego=10.0, gps_updated=True, gps_lat=39.9, gps_lng=116.4, gps_flags=1)
    ready_tick(t)
    t.tick(sm)

    # Second point far away (> 50m)
    sm = make_sm(v_ego=10.0, gps_updated=True, gps_lat=39.91, gps_lng=116.41, gps_flags=1)
    ready_tick(t)
    t.tick(sm)

    assert len(t._trace) == 2

  def test_trace_saved_in_json(self, tracker_module):
    t = tracker_module.DriveTracker()
    t._reset()
    t._duration_s = 60.0
    t._distance_m = 500.0
    t._has_gps = True
    t._trace = [[39.9, 116.4], [39.91, 116.41], [39.92, 116.42]]
    t._save()

    data = json.loads(open(tracker_module._last_drive_file).read())
    assert len(data['trace']) == 3
    assert data['trace'][0] == [39.9, 116.4]

  def test_save_preserves_previous_trace_when_no_gps(self, tracker_module):
    """Drive with no GPS lock keeps previous drive's trace in the saved file."""
    # Write a previous drive with a GPS trace
    prev = {
      'version': 1, 'has_gps': True,
      'trace': [[39.9, 116.4], [39.91, 116.41]],
      'start_lat': 39.9, 'start_lng': 116.4,
      'end_lat': 39.91, 'end_lng': 116.41,
      'duration_s': 60.0, 'distance_m': 500.0, 'engaged_s': 30.0,
    }
    with open(tracker_module._last_drive_file, 'w') as f:
      json.dump(prev, f)

    # New drive: sufficient length but no GPS
    t = tracker_module.DriveTracker()
    t._reset()
    t._duration_s = 50.0
    t._distance_m = 800.0
    t._has_gps = False
    t._trace = []
    t._save()

    data = json.loads(open(tracker_module._last_drive_file).read())
    # New drive stats should be saved
    assert data['duration_s'] == 50.0
    assert data['distance_m'] == 800.0
    # GPS trace carried over from previous drive
    assert data['has_gps'] is True
    assert len(data['trace']) == 2
    assert data['start_lat'] == 39.9
    assert data['end_lat'] == 39.91

  def test_save_no_gps_no_previous_trace(self, tracker_module):
    """Drive with no GPS and no previous drive: trace stays empty."""
    t = tracker_module.DriveTracker()
    t._reset()
    t._duration_s = 50.0
    t._distance_m = 800.0
    t._has_gps = False
    t._trace = []
    t._save()

    data = json.loads(open(tracker_module._last_drive_file).read())
    assert data['has_gps'] is False
    assert data['trace'] == []


# ============================================================
# get_last_drive
# ============================================================

class TestGetLastDrive:
  def test_reads_valid_json(self, tracker_module):
    with open(tracker_module._last_drive_file, 'w') as f:
      json.dump({'version': 1, 'distance_m': 1000}, f)
    result = tracker_module.get_last_drive()
    assert result['distance_m'] == 1000

  def test_missing_file_returns_none(self, tracker_module):
    assert tracker_module.get_last_drive() is None

  def test_corrupted_file_returns_none(self, tracker_module):
    with open(tracker_module._last_drive_file, 'w') as f:
      f.write('garbage{{{')
    assert tracker_module.get_last_drive() is None
