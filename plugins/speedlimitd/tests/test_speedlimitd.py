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

  def test_four_lines_visible(self, sld):
    # All 4 lines visible (>0.3) → 4 lanes
    model = self._make_model([0.4, 0.6, 0.7, 0.5])
    assert sld.infer_lane_count(model) == 4

  def test_three_lines_visible(self, sld):
    # 3 lines visible → 3 lanes
    model = self._make_model([0.4, 0.6, 0.7, 0.2])
    assert sld.infer_lane_count(model) == 3

  def test_two_lines_visible(self, sld):
    # Inner pair visible → 2 lanes
    model = self._make_model([0.1, 0.6, 0.7, 0.1])
    assert sld.infer_lane_count(model) == 2

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
# Vision Speed Cap
# ============================================================

class TestVisionSpeedCap:
  def _make_model(self, probs):
    m = MagicMock()
    m.laneLineProbs = probs
    return m

  def test_two_lanes_high_confidence(self, sld):
    # Inner pair confident, only 2 lines visible → 40 km/h cap
    model = self._make_model([0.1, 0.8, 0.9, 0.1])
    assert sld.vision_speed_cap(model) == 40

  def test_one_lane_high_confidence(self, sld):
    # One inner line confident, only 1 line visible → 30 km/h cap
    model = self._make_model([0.1, 0.8, 0.1, 0.1])
    assert sld.vision_speed_cap(model) == 30

  def test_wide_road_no_cap(self, sld):
    # 4 lines visible → no cap
    model = self._make_model([0.5, 0.8, 0.9, 0.5])
    assert sld.vision_speed_cap(model) == 0

  def test_low_confidence_no_cap(self, sld):
    # Inner pair not confident → no cap even with few lines
    model = self._make_model([0.1, 0.4, 0.5, 0.1])
    assert sld.vision_speed_cap(model) == 0

  def test_three_lanes_no_cap(self, sld):
    # 3 lines visible → no cap
    model = self._make_model([0.5, 0.8, 0.9, 0.1])
    assert sld.vision_speed_cap(model) == 0

  def test_missing_probs(self, sld):
    model = MagicMock()
    model.laneLineProbs = [0.5, 0.5]
    assert sld.vision_speed_cap(model) == 0


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
    assert sld.infer_speed_from_road_type('trunk', 1, 'freeway') == 80

  def test_residential(self, sld):
    assert sld.infer_speed_from_road_type('residential', 1, 'city') == 30

  def test_unknown_road_type(self, sld):
    assert sld.infer_speed_from_road_type('footpath', 1, 'city') == 40  # DEFAULT_FALLBACK

  def test_unknown_context_defaults_urban(self, sld):
    # 'unknown' context uses urban table (more conservative)
    assert sld.infer_speed_from_road_type('trunk', 2, 'unknown') == 80  # urban multi

  def test_living_street(self, sld):
    assert sld.infer_speed_from_road_type('living_street', 1, 'city') == 30

  def test_service_road(self, sld):
    assert sld.infer_speed_from_road_type('service', 1, 'city') == 30

  def test_secondary_freeway_overridden_to_urban(self, sld):
    # secondary roads should never use nonurban table, even if mapd says freeway
    # 4-lane secondary → promoted to trunk, urban trunk multi = 80
    assert sld.infer_speed_from_road_type('secondary', 4, 'freeway') == 80

  def test_tertiary_freeway_overridden_to_urban(self, sld):
    assert sld.infer_speed_from_road_type('tertiary', 2, 'freeway') == 60  # urban primary multi


# ============================================================
# Speed Table Loading & Completeness
# ============================================================

class TestSpeedTables:
  def test_load_cn(self, sld):
    urban, nonurban, fallback = sld.load_speed_table('cn')
    assert fallback == 40
    assert urban['motorway']['multi'] == 100
    assert nonurban['motorway']['multi'] == 120

  def test_load_de(self, sld):
    urban, nonurban, fallback = sld.load_speed_table('de')
    assert fallback == 50
    assert nonurban['motorway']['multi'] == 130

  def test_load_au(self, sld):
    urban, nonurban, fallback = sld.load_speed_table('au')
    assert fallback == 50
    assert nonurban['motorway']['multi'] == 110

  def test_load_missing_country(self, sld):
    with pytest.raises(FileNotFoundError):
      sld.load_speed_table('xx')

  def test_country_bboxes_loaded(self, sld):
    bboxes = sld.load_country_bboxes()
    codes = [c for c, _ in bboxes]
    assert 'cn' in codes
    assert 'de' in codes
    assert 'au' in codes

  def test_country_from_gps_china(self, sld):
    bboxes = sld.load_country_bboxes()
    assert sld.country_from_gps(31.2, 121.5, bboxes) == 'cn'  # Shanghai

  def test_country_from_gps_germany(self, sld):
    bboxes = sld.load_country_bboxes()
    assert sld.country_from_gps(52.5, 13.4, bboxes) == 'de'  # Berlin

  def test_country_from_gps_australia(self, sld):
    bboxes = sld.load_country_bboxes()
    assert sld.country_from_gps(-33.9, 151.2, bboxes) == 'au'  # Sydney

  def test_country_from_gps_unknown(self, sld):
    bboxes = sld.load_country_bboxes()
    assert sld.country_from_gps(0, 0, bboxes) is None  # middle of ocean

  def test_all_tables_have_both_lane_types(self, sld):
    import os
    for fname in os.listdir(sld.SPEED_TABLES_DIR):
      if not fname.endswith('.toml'):
        continue
      country = fname[:-5]
      urban, nonurban, _ = sld.load_speed_table(country)
      for table_name, table in [('urban', urban), ('nonurban', nonurban)]:
        for road_type, entry in table.items():
          assert 'multi' in entry, f"{country}/{table_name}/{road_type} missing 'multi'"
          assert 'single' in entry, f"{country}/{table_name}/{road_type} missing 'single'"

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
    mock_cv.MS_TO_KPH = 3.6
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

  def test_confirmed_limits_v_cruise_highway(self, hook):
    """Highway limit (>60 kph) uses 10% offset."""
    sm = MagicMock()
    sm.valid = {'speedLimitState': True}
    sm.recv_frame = {'speedLimitState': 1}
    sls = MagicMock()
    sls.confirmed = True
    sls.speedLimit = 80  # kph — highway, 10% offset
    sm.__getitem__ = MagicMock(return_value=sls)

    result = hook.on_v_cruise(100.0, 20.0, sm)
    assert result < 100.0
    assert result == pytest.approx(80 * 1.10 / 3.6, abs=0.1)

  def test_confirmed_limits_v_cruise_low_speed(self, hook):
    """Low limit (≤50 kph) uses 40% comfort offset."""
    sm = MagicMock()
    sm.valid = {'speedLimitState': True}
    sm.recv_frame = {'speedLimitState': 1}
    sls = MagicMock()
    sls.confirmed = True
    sls.speedLimit = 40  # kph — low speed, 40% offset
    sm.__getitem__ = MagicMock(return_value=sls)

    result = hook.on_v_cruise(100.0, 20.0, sm)
    assert result == pytest.approx(40 * 1.40 / 3.6, abs=0.1)

  def test_confirmed_limits_v_cruise_mid_speed(self, hook):
    """Mid limit (51-60 kph) uses 30% comfort offset."""
    sm = MagicMock()
    sm.valid = {'speedLimitState': True}
    sm.recv_frame = {'speedLimitState': 1}
    sls = MagicMock()
    sls.confirmed = True
    sls.speedLimit = 60  # kph — mid speed, 30% offset
    sm.__getitem__ = MagicMock(return_value=sls)

    result = hook.on_v_cruise(100.0, 20.0, sm)
    assert result == pytest.approx(60 * 1.30 / 3.6, abs=0.1)

  def test_confirmed_no_limit_if_already_below(self, hook):
    sm = MagicMock()
    sm.valid = {'speedLimitState': True}
    sm.recv_frame = {'speedLimitState': 1}
    sls = MagicMock()
    sls.confirmed = True
    sls.speedLimit = 120  # kph
    sm.__getitem__ = MagicMock(return_value=sls)

    # v_cruise = 10 m/s (already well below 120 * 1.10 kph limit)
    result = hook.on_v_cruise(10.0, 8.0, sm)
    assert result == 10.0

  def _make_sm(self, speed_limit, confirmed=True, lead_status=False, lead_vLead=0.0):
    """Helper: build SubMaster mock with speedLimitState and radarState."""
    sm = MagicMock()
    sm.valid = {'speedLimitState': True}
    sm.recv_frame = {'speedLimitState': 1}

    sls = MagicMock()
    sls.confirmed = confirmed
    sls.speedLimit = speed_limit

    lead = MagicMock()
    lead.status = lead_status
    lead.vLead = lead_vLead

    radar = MagicMock()
    radar.leadOne = lead

    def getitem(key):
      if key == 'speedLimitState':
        return sls
      if key == 'radarState':
        return radar
      return MagicMock()

    sm.__getitem__ = MagicMock(side_effect=getitem)
    return sm

  def test_lead_override_fast_lead_skips_limit(self, hook):
    """Lead >10% above speed limit → skip capping."""
    # Speed limit 80 kph (highway, 10% offset), lead at 95 kph (19% above → override)
    sm = self._make_sm(80, confirmed=True, lead_status=True, lead_vLead=95 / 3.6)
    result = hook.on_v_cruise(100.0, 20.0, sm)
    assert result == 100.0  # original v_cruise, not capped

  def test_lead_override_slow_lead_keeps_limit(self, hook):
    """Lead only 5% above speed limit → still cap."""
    # Speed limit 80 kph, lead at 84 kph (5% above → no override)
    sm = self._make_sm(80, confirmed=True, lead_status=True, lead_vLead=84 / 3.6)
    result = hook.on_v_cruise(100.0, 20.0, sm)
    assert result == pytest.approx(80 * 1.10 / 3.6, abs=0.1)

  def test_lead_override_no_lead_keeps_limit(self, hook):
    """No tracked lead → normal capping."""
    sm = self._make_sm(80, confirmed=True, lead_status=False, lead_vLead=0)
    result = hook.on_v_cruise(100.0, 20.0, sm)
    assert result == pytest.approx(80 * 1.10 / 3.6, abs=0.1)

  def test_lead_override_exactly_at_threshold(self, hook):
    """Lead exactly at 10% threshold → no override (must be strictly above)."""
    # Speed limit 80 kph, lead at exactly 88 kph (10% above → boundary)
    sm = self._make_sm(80, confirmed=True, lead_status=True, lead_vLead=88 / 3.6)
    result = hook.on_v_cruise(100.0, 20.0, sm)
    assert result == pytest.approx(80 * 1.10 / 3.6, abs=0.1)


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
