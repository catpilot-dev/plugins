"""Tests for speedlimitd daemon — lane inference, speed tables, priority cascade, confirmation."""
import pytest
from unittest.mock import MagicMock, patch
import sys
import importlib


@pytest.fixture(autouse=True)
def mock_openpilot(monkeypatch):
  """Mock openpilot + cereal imports."""
  for mod in ['openpilot', 'openpilot.common',
              'openpilot.common.realtime', 'cereal', 'cereal.messaging']:
    monkeypatch.setitem(sys.modules, mod, MagicMock())
  sys.modules['openpilot.common.realtime'].Ratekeeper = MagicMock

  # Mock params_helper (file-based params used by speedlimitd)
  mock_params_helper = MagicMock()
  mock_params_helper.get = MagicMock(return_value=None)
  monkeypatch.setitem(sys.modules, 'params_helper', mock_params_helper)


@pytest.fixture
def sld():
  import plugins.speedlimitd.speedlimitd as mod
  importlib.reload(mod)
  return mod


# ============================================================
# Lane Count Inference
# ============================================================

class TestInferLaneCount:
  def _make_model(self, probs):
    m = MagicMock()
    m.laneLineProbs = probs
    return m

  def test_multi_lane(self, sld):
    # left lane 0.6, right lane 0.7, left edge 0.4
    model = self._make_model([0.4, 0.6, 0.7, 0.2])
    assert sld.infer_lane_count(model) == 2

  def test_single_lane_no_edge(self, sld):
    # Both lanes visible but no road edge
    model = self._make_model([0.1, 0.6, 0.7, 0.1])
    assert sld.infer_lane_count(model) == 1

  def test_single_lane_low_probs(self, sld):
    model = self._make_model([0.1, 0.3, 0.3, 0.1])
    assert sld.infer_lane_count(model) == 1

  def test_missing_probs(self, sld):
    model = MagicMock()
    model.laneLineProbs = [0.5, 0.5]  # < 4 elements
    assert sld.infer_lane_count(model) == 1

  def test_no_attribute(self, sld):
    model = MagicMock(spec=[])  # no laneLineProbs
    assert sld.infer_lane_count(model) == 1


# ============================================================
# Speed Table Lookup
# ============================================================

class TestInferSpeedFromRoadType:
  def test_motorway_freeway_multi(self, sld):
    assert sld.infer_speed_from_road_type('motorway', 2, 'freeway') == 120

  def test_motorway_urban_multi(self, sld):
    assert sld.infer_speed_from_road_type('motorway', 2, 'city') == 100

  def test_trunk_single_urban(self, sld):
    assert sld.infer_speed_from_road_type('trunk', 1, 'city') == 60

  def test_trunk_single_freeway(self, sld):
    assert sld.infer_speed_from_road_type('trunk', 1, 'freeway') == 70

  def test_residential(self, sld):
    assert sld.infer_speed_from_road_type('residential', 1, 'city') == 30

  def test_unknown_road_type(self, sld):
    assert sld.infer_speed_from_road_type('footpath', 1, 'city') == 40  # DEFAULT_FALLBACK

  def test_unknown_context_defaults_urban(self, sld):
    # 'unknown' context uses urban table (more conservative)
    assert sld.infer_speed_from_road_type('trunk', 2, 'unknown') == 80  # urban multi

  def test_living_street(self, sld):
    assert sld.infer_speed_from_road_type('living_street', 1, 'city') == 20

  def test_service_road(self, sld):
    assert sld.infer_speed_from_road_type('service', 2, 'city') == 20


# ============================================================
# Speed Table Completeness
# ============================================================

class TestSpeedTables:
  def test_urban_table_has_all_types(self, sld):
    expected = {'motorway', 'trunk', 'primary', 'secondary', 'tertiary',
                'residential', 'unclassified', 'living_street', 'service'}
    assert set(sld.SPEED_TABLE_URBAN.keys()) == expected

  def test_nonurban_table_types(self, sld):
    for key in sld.SPEED_TABLE_NONURBAN:
      assert key in sld.SPEED_TABLE_URBAN, f"{key} in nonurban but not urban"

  def test_all_entries_have_both_lane_types(self, sld):
    for table in [sld.SPEED_TABLE_URBAN, sld.SPEED_TABLE_NONURBAN]:
      for road_type, entry in table.items():
        assert 'multi' in entry, f"{road_type} missing 'multi'"
        assert 'single' in entry, f"{road_type} missing 'single'"

  def test_nonurban_ge_urban_major_roads(self, sld):
    """Non-urban speed limits should be >= urban for major road types."""
    for road_type in ['motorway', 'trunk', 'primary', 'secondary']:
      if road_type not in sld.SPEED_TABLE_NONURBAN:
        continue
      for lane in ['multi', 'single']:
        nonurban = sld.SPEED_TABLE_NONURBAN[road_type][lane]
        urban = sld.SPEED_TABLE_URBAN[road_type][lane]
        assert nonurban >= urban, f"{road_type}/{lane}: nonurban {nonurban} < urban {urban}"


# ============================================================
# Planner Hook
# ============================================================

class TestPlannerHook:
  @pytest.fixture
  def hook(self):
    # Need to mock CV
    mock_cv = MagicMock()
    mock_cv.KPH_TO_MS = 1.0 / 3.6
    sys.modules['openpilot.common.constants'] = MagicMock(CV=mock_cv)
    sys.modules['openpilot.common'] = MagicMock()

    import plugins.speedlimitd.planner_hook as mod
    importlib.reload(mod)
    return mod

  def test_no_speed_limit_state(self, hook):
    sm = MagicMock()
    sm.valid = {}
    sm.recv_frame = {}
    result = hook.on_v_cruise(30.0, 20.0, sm)
    assert result == 30.0

  def test_unconfirmed_returns_original(self, hook):
    sm = MagicMock()
    sm.valid = {'speedLimitState': True}
    sm.recv_frame = {'speedLimitState': 1}
    sls = MagicMock()
    sls.confirmed = False
    sls.speedLimit = 60
    sm.__getitem__ = MagicMock(return_value=sls)
    result = hook.on_v_cruise(30.0, 20.0, sm)
    assert result == 30.0

  def test_confirmed_limits_v_cruise(self, hook):
    sm = MagicMock()
    sm.valid = {'speedLimitState': True}
    sm.recv_frame = {'speedLimitState': 1}
    sls = MagicMock()
    sls.confirmed = True
    sls.speedLimit = 60  # kph
    sm.__getitem__ = MagicMock(return_value=sls)

    # v_cruise = 100 m/s (way above limit)
    result = hook.on_v_cruise(100.0, 20.0, sm)
    # Should be (60 + 20) / 3.6 ≈ 22.2 m/s (60 kph limit + 20 kph offset for <=60)
    assert result < 100.0
    assert result == pytest.approx((60 + 20) / 3.6, abs=0.1)

  def test_confirmed_no_limit_if_already_below(self, hook):
    sm = MagicMock()
    sm.valid = {'speedLimitState': True}
    sm.recv_frame = {'speedLimitState': 1}
    sls = MagicMock()
    sls.confirmed = True
    sls.speedLimit = 120  # kph
    sm.__getitem__ = MagicMock(return_value=sls)

    # v_cruise = 10 m/s (already well below 120+10 kph limit)
    result = hook.on_v_cruise(10.0, 8.0, sm)
    assert result == 10.0

  def test_offset_table(self, hook):
    # Lower speeds get +20, higher speeds get +10
    assert hook.SPEED_LIMIT_OFFSET[30] == 20
    assert hook.SPEED_LIMIT_OFFSET[60] == 20
    assert hook.SPEED_LIMIT_OFFSET[70] == 10
    assert hook.SPEED_LIMIT_OFFSET[120] == 10


# ============================================================
# plugin.json validation
# ============================================================

class TestPluginManifest:
  def test_valid_json(self):
    import json, os
    manifest_path = os.path.join(os.path.dirname(__file__), '..', 'plugin.json')
    with open(manifest_path) as f:
      manifest = json.load(f)

    assert manifest['id'] == 'speedlimitd'
    assert manifest['type'] == 'hybrid'
    assert 'planner.v_cruise' in manifest['hooks']
    assert 'ui.render_overlay' in manifest['hooks']

  def test_has_state_subscriptions_hook(self):
    import json, os
    manifest_path = os.path.join(os.path.dirname(__file__), '..', 'plugin.json')
    with open(manifest_path) as f:
      manifest = json.load(f)

    assert 'ui.state_subscriptions' in manifest['hooks']
    hook = manifest['hooks']['ui.state_subscriptions']
    assert hook['module'] == 'ui_overlay'
    assert hook['function'] == 'on_state_subscriptions'
