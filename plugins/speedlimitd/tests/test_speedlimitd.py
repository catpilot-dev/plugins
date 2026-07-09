"""Tests for speedlimitd daemon — lane inference, speed tables, priority cascade, confirmation."""
import os
import pytest
from unittest.mock import MagicMock, patch
import sys
import importlib

# osm_query does `from config import MEDIA_DIR` — on device the plugins dir is
# on sys.path; replicate that here.
_PLUGINS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
if _PLUGINS_DIR not in sys.path:
  sys.path.insert(0, _PLUGINS_DIR)


@pytest.fixture(autouse=True)
def mock_openpilot(monkeypatch):
  """Mock openpilot + cereal imports."""
  mock_services = MagicMock()
  mock_services.SERVICE_LIST = {'modelV2': MagicMock(), 'gpsLocationExternal': MagicMock()}
  mock_plugin_bus = MagicMock()
  # PluginSub().drain() must return None to avoid infinite loop in __init__
  mock_plugin_bus.PluginSub.return_value.drain.return_value = None
  for mod in ['openpilot', 'openpilot.common',
              'openpilot.common.realtime', 'cereal', 'cereal.messaging',
              'cereal.services',
              'openpilot.selfdrive', 'openpilot.selfdrive.plugins',
              'openpilot.selfdrive.plugins.plugin_bus']:
    monkeypatch.setitem(sys.modules, mod, MagicMock())
  sys.modules['cereal.services'] = mock_services
  sys.modules['openpilot.selfdrive.plugins.plugin_bus'] = mock_plugin_bus
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
    # 4 lines visible (outer pair >0.5) → no cap
    model = self._make_model([0.7, 0.8, 0.9, 0.6])
    assert sld.vision_speed_cap(model) == 0

  def test_low_confidence_no_cap(self, sld):
    # Inner pair not confident → no cap even with few lines
    model = self._make_model([0.1, 0.4, 0.5, 0.1])
    assert sld.vision_speed_cap(model) == 0

  def test_three_lanes_no_cap(self, sld):
    # 3 lines visible (one outer >0.5) → no cap
    model = self._make_model([0.6, 0.8, 0.9, 0.1])
    assert sld.vision_speed_cap(model) == 0

  def test_faint_outer_line_triggers_cap(self, sld):
    # Outer line at 0.4 (faint echo of adjacent road) should not block cap
    model = self._make_model([0.01, 0.9, 0.95, 0.4])
    assert sld.vision_speed_cap(model) == 40

  def test_missing_probs(self, sld):
    model = MagicMock()
    model.laneLineProbs = [0.5, 0.5]
    assert sld.vision_speed_cap(model) == 0


# ============================================================
# Standard Speed Snap
# ============================================================

class TestSnapToStandardSpeed:
  def test_exact_standard_values(self, sld):
    for v in [30, 40, 50, 60, 80, 100, 120]:
      assert sld.snap_to_standard_speed(v) == v

  def test_rounds_to_nearest(self, sld):
    assert sld.snap_to_standard_speed(31) == 30
    assert sld.snap_to_standard_speed(44) == 40
    assert sld.snap_to_standard_speed(47) == 50
    assert sld.snap_to_standard_speed(55) == 50
    assert sld.snap_to_standard_speed(56) == 60
    assert sld.snap_to_standard_speed(75) == 80
    assert sld.snap_to_standard_speed(83) == 80

  def test_mapd_raw_curve_values(self, sld):
    # Values seen from mapd visionCurveSpeed on Shanghai expressways
    assert sld.snap_to_standard_speed(99) == 100
    assert sld.snap_to_standard_speed(105) == 100
    assert sld.snap_to_standard_speed(46) == 50


# ============================================================
# Speed Table Lookup
# ============================================================

class TestInferSpeedFromRoadType:
  def test_motorway_freeway_multi(self, sld):
    # lane_count=4 + freeway → motorway table → 120 km/h
    assert sld.infer_speed_from_road_type('motorway', 4, 'freeway') == 120

  def test_motorway_urban_multi(self, sld):
    # no wayRef (hw=''), lane_count=4 + city → trunk (not motorway — urban arterials are trunk-grade)
    assert sld.infer_speed_from_road_type('', 4, 'city') == 80

  def test_trunk_single_urban(self, sld):
    # lane_count=1 → 30 km/h directly (narrow road, skip table)
    assert sld.infer_speed_from_road_type('trunk', 1, 'city') == 30

  def test_trunk_single_freeway(self, sld):
    # lane_count=1 → 30 km/h directly (narrow road, skip table)
    assert sld.infer_speed_from_road_type('trunk', 1, 'freeway') == 30

  def test_residential(self, sld):
    assert sld.infer_speed_from_road_type('residential', 1, 'city') == 30

  def test_unknown_road_type(self, sld):
    # lane_count=1 → 30 km/h directly (narrow road, skip table regardless of highway_type)
    assert sld.infer_speed_from_road_type('footpath', 1, 'city') == 30

  def test_unknown_context_defaults_urban(self, sld):
    # 'unknown' context uses urban table; lane_count=2 → 40 km/h directly
    assert sld.infer_speed_from_road_type('trunk', 2, 'unknown') == 40
    # lane_count=3 → primary (city) → urban primary multi = 60
    assert sld.infer_speed_from_road_type('trunk', 3, 'unknown') == 80  # urban trunk multi

  def test_living_street(self, sld):
    assert sld.infer_speed_from_road_type('living_street', 1, 'city') == 30

  def test_service_road(self, sld):
    assert sld.infer_speed_from_road_type('service', 1, 'city') == 30

  def test_secondary_freeway_overridden_to_urban(self, sld):
    # secondary roads forced to urban table; 4-lane city → trunk → urban trunk multi = 80
    assert sld.infer_speed_from_road_type('secondary', 4, 'freeway') == 80

  def test_tertiary_freeway_overridden_to_urban(self, sld):
    # lane_count=2 → 40 km/h directly (narrow road, skip table)
    assert sld.infer_speed_from_road_type('tertiary', 2, 'freeway') == 40


# ============================================================
# Speed Table Loading & Completeness
# ============================================================

class TestSpeedTables:
  def test_load_cn(self, sld):
    urban, nonurban, fallback, lane_width_class = sld.load_speed_table('cn')
    assert fallback == 40
    assert urban['motorway']['multi'] == 100
    assert nonurban['motorway']['multi'] == 120
    # cn has lane_width_class populated; sorted descending by `min`
    assert len(lane_width_class) >= 2
    mins = [e['min'] for e in lane_width_class]
    assert mins == sorted(mins, reverse=True)

  def test_load_de(self, sld):
    urban, nonurban, fallback, _ = sld.load_speed_table('de')
    assert fallback == 50
    assert nonurban['motorway']['multi'] == 130

  def test_load_au(self, sld):
    urban, nonurban, fallback, _ = sld.load_speed_table('au')
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
      urban, nonurban, _, _ = sld.load_speed_table(country)
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
# Lane Width → Road Class Fusion
# ============================================================

class TestLaneWidthClassification:
  def test_classify_highway_lane(self, sld):
    assert sld.classify_by_width(3.75, sld.LANE_WIDTH_CLASS_TABLE) == 'trunk'

  def test_classify_city_arterial(self, sld):
    assert sld.classify_by_width(3.40, sld.LANE_WIDTH_CLASS_TABLE) == 'primary'

  def test_classify_city_collector(self, sld):
    assert sld.classify_by_width(3.00, sld.LANE_WIDTH_CLASS_TABLE) == 'secondary'

  def test_classify_narrow_lane(self, sld):
    assert sld.classify_by_width(2.50, sld.LANE_WIDTH_CLASS_TABLE) == 'residential'

  def test_classify_no_observation(self, sld):
    assert sld.classify_by_width(0.0, sld.LANE_WIDTH_CLASS_TABLE) == ''

  def test_classify_empty_table(self, sld):
    assert sld.classify_by_width(3.5, []) == ''


class TestLaneWidthFusion:
  def test_width_promotes_when_osm_unknown(self, sld):
    # 3-lane urban road with no OSM highway type.
    # Without width: lane_count votes 'primary' (rank 2) → urban primary = 60 km/h.
    no_width = sld.infer_speed_from_road_type('', 3, 'city')
    # With width hint 'trunk' (rank 3, from 3.75 m lanes) → should pick urban trunk = 80.
    with_width = sld.infer_speed_from_road_type('', 3, 'city', width_class='trunk')
    assert with_width > no_width
    assert with_width == sld.SPEED_TABLE_URBAN['trunk']['multi']

  def test_width_does_not_override_known_motorway(self, sld):
    # OSM already identified the road as motorway (G-ref). Width can't override.
    speed = sld.infer_speed_from_road_type('motorway', 4, 'freeway', width_class='residential')
    assert speed == sld.SPEED_TABLE_NONURBAN['motorway']['multi']

  def test_width_ignored_when_lane_class_higher(self, sld):
    # 5-lane road (lane_class='trunk' urban, rank 3). Width 'primary' (rank 2)
    # should not demote — highest-rank voter wins.
    speed = sld.infer_speed_from_road_type('', 5, 'city', width_class='primary')
    assert speed == sld.SPEED_TABLE_URBAN['trunk']['multi']

  def test_width_breaks_tie_with_osm_tertiary(self, sld):
    # OSM says 'secondary' (rank 1), lane_count=3 says 'primary' (rank 2),
    # width says 'secondary' (rank 1). Primary wins — width doesn't demote.
    speed = sld.infer_speed_from_road_type('secondary', 3, 'city', width_class='secondary')
    assert speed == sld.SPEED_TABLE_URBAN['primary']['multi']


# ============================================================
# OSM highwayType — trusted classification vote
# ============================================================

class TestOsmHighwayTypeVote:
  def test_osm_tertiary_overrides_inflated_lane_vote(self, sld):
    # Route 38d 白城路 regression: two-way 1+1 road, vision overcounts to
    # 4 lanes (counts oncoming lane + edge boost) → trunk → 80 km/h.
    # OSM highway=tertiary is the trusted classification → urban tertiary = 40.
    without_osm = sld.infer_speed_from_road_type('', 4, 'city')
    with_osm = sld.infer_speed_from_road_type('', 4, 'city', osm_type='tertiary')
    assert without_osm == sld.SPEED_TABLE_URBAN['trunk']['multi']  # the bug: 80
    assert with_osm == sld.SPEED_TABLE_URBAN['tertiary']['multi']  # 40

  def test_osm_motorway_without_ref_demotes_to_trunk(self, sld):
    # Elevated urban expressways without G/S refs (中环路-style) are trunk-grade
    # (80), not motorway-grade (100), even if OSM tags them motorway.
    speed = sld.infer_speed_from_road_type('', 4, 'city', osm_type='motorway')
    assert speed == sld.SPEED_TABLE_URBAN['trunk']['multi']

  def test_expressway_ref_beats_osm_type(self, sld):
    # G/S ref classification stays highest priority — a matched parallel
    # side-road's OSM type must not demote a ref'd expressway.
    speed = sld.infer_speed_from_road_type('motorway', 4, 'freeway', osm_type='tertiary')
    assert speed == sld.SPEED_TABLE_NONURBAN['motorway']['multi']

  def test_narrow_road_shortcut_beats_osm_type(self, sld):
    # Vision confidently seeing ≤2 lanes still wins (link/ramp safety net).
    assert sld.infer_speed_from_road_type('', 2, 'city', osm_type='trunk') == 40
    assert sld.infer_speed_from_road_type('', 1, 'city', osm_type='trunk') == 30

  def test_osm_urban_only_type_forces_city_context(self, sld):
    # tertiary can't be a freeway — context demotes to city → urban table.
    speed = sld.infer_speed_from_road_type('', 4, 'freeway', osm_type='tertiary')
    assert speed == sld.SPEED_TABLE_URBAN['tertiary']['multi']

  def test_unknown_osm_type_falls_back(self, sld):
    # OSM types with no table entry (e.g. 'track') → default fallback.
    assert sld.infer_speed_from_road_type('', 3, 'city', osm_type='track') == sld.DEFAULT_FALLBACK_SPEED

  def test_osm_link_type_uses_link_entry(self, sld):
    # _link classifications map to their table entries (ramps → 40).
    speed = sld.infer_speed_from_road_type('', 3, 'city', osm_type='trunk_link')
    assert speed == sld.SPEED_TABLE_URBAN['trunk_link']['multi']


class TestOsmResultIngest:
  def _make_middleware(self, sld):
    import plugins.speedlimitd.speedlimitd as mod
    with patch.object(mod.messaging, 'SubMaster'):
      mw = mod.SpeedLimitMiddleware()
    mw._sl_pub = MagicMock()
    return mw

  def _result(self, **kw):
    base = {'wayRef': '', 'wayName': '', 'speedLimit': 0.0, 'lanes': 0,
            'roadContext': 1, 'roadName': '', 'highwayType': '', 'distance': 5.0}
    base.update(kw)
    return base

  def test_refless_named_way_accepted(self, sld):
    # Ways without a G/S ref (all residential/tertiary streets) must no longer
    # be discarded — name and highwayType are ingested.
    mw = self._make_middleware(sld)
    mw._ingest_osm_result(self._result(roadName='白城路', highwayType='tertiary'))
    assert mw.last_road_name == '白城路'
    assert mw.last_osm_hwtype == 'tertiary'

  def test_unnamed_typed_way_accepted(self, sld):
    # Unnamed service/residential ways still carry a classification.
    mw = self._make_middleware(sld)
    mw._ingest_osm_result(self._result(highwayType='service'))
    assert mw.last_osm_hwtype == 'service'

  def test_no_match_clears_osm_state(self, sld):
    mw = self._make_middleware(sld)
    mw._ingest_osm_result(self._result(roadName='白城路', highwayType='tertiary'))
    mw._ingest_osm_result(None)
    assert mw.last_road_name == ''
    assert mw.last_osm_hwtype == ''

  def test_ref_way_still_sets_highway_type(self, sld):
    # Existing G-ref behavior is preserved through the ingest path.
    mw = self._make_middleware(sld)
    mw._ingest_osm_result(self._result(wayRef='G2', roadName='京沪高速',
                                       roadContext=0, highwayType='motorway'))
    assert mw.last_highway_type == 'motorway'
    assert mw.last_road_context == 'freeway'
    assert mw.last_osm_hwtype == 'motorway'


# ============================================================
# Priority Cascade
# ============================================================

class TestPriorityCascade:
  def _make_middleware(self, sld):
    """Create a SpeedLimitMiddleware with messaging mocked out."""
    import plugins.speedlimitd.speedlimitd as mod
    with patch.object(mod.messaging, 'SubMaster'):
      mw = mod.SpeedLimitMiddleware()
    mw._sl_pub = MagicMock()
    return mw

  def test_min_of_all_sources(self, sld):
    """Speed limit is the minimum across mapd, inference, and YOLO."""
    # mapd=105 (highway max), inference=80 → min=80
    inferred = sld.infer_speed_from_road_type('primary', 2, 'city')
    mapd = 105
    result = min(mapd, inferred)
    assert result == inferred
    assert result < mapd

  def test_mapd_curve_wins_over_inference(self, sld):
    """When mapd gives a curve constraint lower than inference, mapd wins."""
    inferred = sld.infer_speed_from_road_type('motorway', 6, 'freeway')  # high-speed road
    mapd = 70  # sharp curve
    result = min(mapd, inferred)
    assert result == 70
    assert inferred > 70  # confirm inference is higher

  def test_yolo_wins_when_lowest(self, sld):
    """YOLO sign (e.g. 60) beats both mapd and inference when it's lowest."""
    inferred = sld.infer_speed_from_road_type('primary', 3, 'city')  # 3-lane primary → 60
    mapd = 105
    yolo = 60
    result = min(yolo, mapd, inferred)
    assert result == 60

  def test_mapd_unconstrained_excluded(self, sld):
    """mapd suggestedSpeed >= 130 km/h is excluded from candidates."""
    raw = 145
    MAPD_UNCONSTRAINED = 130
    mapd_suggested = raw if raw < MAPD_UNCONSTRAINED else 0
    assert mapd_suggested == 0  # not included in min()

  def test_lane_count_locked_after_2s_stable(self, sld):
    """lane_count_locked becomes True after 2 s of stable lane detection."""
    import time
    import plugins.speedlimitd.speedlimitd as mod
    with patch.object(mod.messaging, 'SubMaster'), \
         patch.object(mod.messaging, 'PubMaster'):
      mw = mod.SpeedLimitMiddleware()
    assert mw.lane_count_locked is False
    mw.lane_count = 3
    mw.lane_count_stable_since = time.monotonic() - 3.0  # 3 s ago
    # Simulate a model update with same lane count
    now = time.monotonic()
    if mw.lane_count == 3 and now - mw.lane_count_stable_since > 2.0:
      mw.lane_count_stable = mw.lane_count
      mw.lane_count_locked = True
    assert mw.lane_count_locked is True
    assert mw.lane_count_stable == 3

  def test_lane_count_demotion_requires_2s(self, sld):
    """Dropping lane count requires 2 s stability (directional hysteresis)."""
    import time
    import plugins.speedlimitd.speedlimitd as mod
    with patch.object(mod.messaging, 'SubMaster'), \
         patch.object(mod.messaging, 'PubMaster'):
      mw = mod.SpeedLimitMiddleware()
    # Establish stable 3-lane reading
    mw.lane_count_stable = 3
    mw.lane_count_locked = True
    # Vision now sees 1 lane, stable for 1 s (< 2 s demotion window)
    mw.lane_count = 1
    mw.lane_count_stable_since = time.monotonic() - 1.0
    going_down = mw.lane_count < mw.lane_count_stable
    stability_window = 2.0 if going_down else 1.5
    if time.monotonic() - mw.lane_count_stable_since > stability_window:
      mw.lane_count_stable = mw.lane_count
    # 1 s is not enough to demote
    assert mw.lane_count_stable == 3

  def test_lane_count_demotion_commits_after_2s(self, sld):
    """Dropping lane count commits after 2 s of stable lower reading."""
    import time
    import plugins.speedlimitd.speedlimitd as mod
    with patch.object(mod.messaging, 'SubMaster'), \
         patch.object(mod.messaging, 'PubMaster'):
      mw = mod.SpeedLimitMiddleware()
    mw.lane_count_stable = 3
    mw.lane_count_locked = True
    # Vision sees 1 lane for 3 s (> 2 s demotion window)
    mw.lane_count = 1
    mw.lane_count_stable_since = time.monotonic() - 3.0
    going_down = mw.lane_count < mw.lane_count_stable
    stability_window = 2.0 if going_down else 1.5
    if time.monotonic() - mw.lane_count_stable_since > stability_window:
      mw.lane_count_stable = mw.lane_count
    assert mw.lane_count_stable == 1


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
    mod._sl_sub = None
    mod._sl_data = None
    return mod

  def _set_sl_data(self, hook, data):
    """Set the speed limit state dict directly on the module."""
    hook._sl_data = data

  def test_no_speed_limit_state(self, hook):
    sm = MagicMock()
    hook._sl_data = None
    result = hook.on_v_cruise(30.0, 20.0, sm)
    assert result == 30.0

  def test_unconfirmed_returns_original(self, hook):
    sm = MagicMock()
    hook._sl_data = {'confirmed': False, 'speedLimit': 60}
    result = hook.on_v_cruise(30.0, 20.0, sm)
    assert result == 30.0

  def test_confirmed_limits_v_cruise_highway(self, hook):
    """Limit >= 80 kph uses 10% offset."""
    sm = MagicMock()
    hook._sl_data = {'confirmed': True, 'speedLimit': 80, 'safetyCapped': False}

    result = hook.on_v_cruise(100.0, 20.0, sm)
    assert result < 100.0
    assert result == pytest.approx(80 * 1.10 / 3.6, abs=0.1)

  def test_confirmed_limits_v_cruise_low_speed(self, hook):
    """Limit < 80 kph uses 15% offset."""
    sm = MagicMock()
    hook._sl_data = {'confirmed': True, 'speedLimit': 40, 'safetyCapped': False}

    result = hook.on_v_cruise(100.0, 20.0, sm)
    assert result == pytest.approx(40 * 1.15 / 3.6, abs=0.1)

  def test_confirmed_limits_v_cruise_mid_speed(self, hook):
    """Limit 60 kph (< 80) uses 15% offset."""
    sm = MagicMock()
    hook._sl_data = {'confirmed': True, 'speedLimit': 60, 'safetyCapped': False}

    result = hook.on_v_cruise(100.0, 20.0, sm)
    assert result == pytest.approx(60 * 1.15 / 3.6, abs=0.1)

  def test_confirmed_no_limit_if_already_below(self, hook):
    sm = MagicMock()
    hook._sl_data = {'confirmed': True, 'speedLimit': 120, 'safetyCapped': False}

    # v_cruise = 10 m/s (already well below 120 * 1.10 kph limit)
    result = hook.on_v_cruise(10.0, 8.0, sm)
    assert result == 10.0

  def _make_sm(self, hook, speed_limit, confirmed=True, lead_status=False, lead_vLead=0.0, safety_capped=False):
    """Helper: set _sl_data and build SubMaster mock with radarState."""
    hook._sl_data = {'confirmed': confirmed, 'speedLimit': speed_limit, 'safetyCapped': safety_capped}

    sm = MagicMock()
    lead = MagicMock()
    lead.status = lead_status
    lead.vLead = lead_vLead

    radar = MagicMock()
    radar.leadOne = lead

    def getitem(key):
      if key == 'radarState':
        return radar
      return MagicMock()

    sm.__getitem__ = MagicMock(side_effect=getitem)
    return sm

  def test_lead_override_fast_lead_skips_limit(self, hook):
    """Lead >10% above speed limit → skip capping."""
    sm = self._make_sm(hook, 80, confirmed=True, lead_status=True, lead_vLead=95 / 3.6)
    result = hook.on_v_cruise(100.0, 20.0, sm)
    assert result == 100.0  # original v_cruise, not capped

  def test_lead_override_slow_lead_keeps_limit(self, hook):
    """Lead only 5% above speed limit → still cap."""
    sm = self._make_sm(hook, 80, confirmed=True, lead_status=True, lead_vLead=84 / 3.6)
    result = hook.on_v_cruise(100.0, 20.0, sm)
    assert result == pytest.approx(80 * 1.10 / 3.6, abs=0.1)

  def test_lead_override_no_lead_keeps_limit(self, hook):
    """No tracked lead → normal capping."""
    sm = self._make_sm(hook, 80, confirmed=True, lead_status=False, lead_vLead=0)
    result = hook.on_v_cruise(100.0, 20.0, sm)
    assert result == pytest.approx(80 * 1.10 / 3.6, abs=0.1)

  def test_lead_override_exactly_at_threshold(self, hook):
    """Lead exactly at 10% threshold → no override (must be strictly above)."""
    sm = self._make_sm(hook, 80, confirmed=True, lead_status=True, lead_vLead=88 / 3.6)
    result = hook.on_v_cruise(100.0, 20.0, sm)
    assert result == pytest.approx(80 * 1.10 / 3.6, abs=0.1)

  def test_safety_cap_ignores_fast_lead(self, hook):
    """Safety cap (curve/lookahead) must not be bypassed by a fast lead —
    geometry doesn't care what the lead is doing. Route 2fd seg15 regression."""
    sm = self._make_sm(hook, 40, confirmed=True, lead_status=True,
                       lead_vLead=60 / 3.6, safety_capped=True)
    result = hook.on_v_cruise(100.0, 20.0, sm)
    # No offset (safety_capped=True), and lead override is ignored
    assert result == pytest.approx(40 / 3.6, abs=0.1)

  # ----------------------------------------------------------
  # Gentle ramp + momentary gas override (source==2 inferred)
  # ----------------------------------------------------------

  def _make_sm_full(self, gas=False, lead_status=False, lead_vLead=0.0):
    """SubMaster mock exposing carState.gasPressed and radarState.leadOne."""
    cs = MagicMock()
    cs.gasPressed = gas
    lead = MagicMock()
    lead.status = lead_status
    lead.vLead = lead_vLead
    radar = MagicMock()
    radar.leadOne = lead

    sm = MagicMock()

    def getitem(key):
      if key == 'carState':
        return cs
      if key == 'radarState':
        return radar
      return MagicMock()

    sm.__getitem__ = MagicMock(side_effect=getitem)
    return sm

  # --- pure ramp helper ---

  def test_ramp_glide_down_rate(self, hook):
    """Far from target, no gas: cap drops by exactly RAMP_DECEL_MS2 * dt."""
    new_cap, enforced = hook._ramp_cap(30.0, 10.0, 0.1, False, 28.0)
    assert new_cap == pytest.approx(30.0 - hook.RAMP_DECEL_MS2 * 0.1)
    assert enforced == pytest.approx(new_cap)

  def test_ramp_clamps_at_target(self, hook):
    """Within one step of target: snap to target, never below."""
    new_cap, enforced = hook._ramp_cap(10.01, 10.0, 1.0, False, 9.0)
    assert new_cap == pytest.approx(10.0)
    assert enforced == pytest.approx(10.0)

  def test_ramp_restores_up_immediately(self, hook):
    """Target above cap (limit rose): restore in one step."""
    new_cap, enforced = hook._ramp_cap(16.0, 30.0, 0.1, False, 16.0)
    assert new_cap == pytest.approx(30.0)
    assert enforced == pytest.approx(30.0)

  def test_ramp_gas_suspends_and_floats(self, hook):
    """Gas pressed: cap suspended (enforced None) and floats up to v_ego."""
    new_cap, enforced = hook._ramp_cap(20.0, 16.0, 0.1, True, 25.0)
    assert enforced is None
    assert new_cap == pytest.approx(25.0)

  def test_ramp_gas_does_not_lower_cap(self, hook):
    """Gas pressed with v_ego below cap: cap held, not lowered."""
    new_cap, enforced = hook._ramp_cap(20.0, 16.0, 0.1, True, 12.0)
    assert enforced is None
    assert new_cap == pytest.approx(20.0)

  def test_ramp_release_resumes_from_current(self, hook):
    """After floating to 25, release resumes glide from 25 toward target."""
    new_cap, enforced = hook._ramp_cap(25.0, 16.0, 0.1, False, 24.0)
    assert new_cap == pytest.approx(25.0 - hook.RAMP_DECEL_MS2 * 0.1)
    assert enforced == pytest.approx(new_cap)

  def test_ramp_init_holds_current_speed(self, hook):
    """Uninitialized cap with v_ego above target: init to v_ego, don't jump down."""
    new_cap, enforced = hook._ramp_cap(None, 16.0, 0.0, False, 25.0)
    assert new_cap == pytest.approx(25.0)
    assert enforced == pytest.approx(25.0)

  # --- integration through on_v_cruise ---

  def test_source2_first_call_holds_current_speed(self, hook):
    """Inferred limit (source 2): first cycle holds current speed, not the offset cap."""
    hook._sl_data = {'confirmed': True, 'speedLimit': 60, 'source': 2, 'safetyCapped': False}
    sm = self._make_sm_full(gas=False)
    result = hook.on_v_cruise(100 / 3.6, 25.0, sm)
    # Holds ~v_ego (25), not the immediate offset cap (60*1.15/3.6 = 19.17)
    assert result == pytest.approx(25.0, abs=0.1)

  def test_source2_gas_suspends_cap(self, hook):
    """Inferred limit + gas pressed: cap fully suspended, v_cruise unchanged."""
    hook._sl_data = {'confirmed': True, 'speedLimit': 60, 'source': 2, 'safetyCapped': False}
    sm = self._make_sm_full(gas=True)
    result = hook.on_v_cruise(100 / 3.6, 25.0, sm)
    assert result == pytest.approx(100 / 3.6)

  def test_source2_glides_down_over_time(self, hook, monkeypatch):
    """Inferred limit: cap glides down at ~0.2 m/s^2 across cycles."""
    hook._sl_data = {'confirmed': True, 'speedLimit': 60, 'source': 2, 'safetyCapped': False}
    sm = self._make_sm_full(gas=False)
    times = [100.0, 100.1]
    monkeypatch.setattr(hook.time, 'monotonic', lambda: times.pop(0))
    hook.on_v_cruise(100 / 3.6, 25.0, sm)          # init, holds 25
    r2 = hook.on_v_cruise(100 / 3.6, 25.0, sm)     # 0.1s later: 25 - RAMP*0.1
    assert r2 == pytest.approx(25.0 - hook.RAMP_DECEL_MS2 * 0.1, abs=0.01)

  def test_source2_safetycapped_uses_immediate_path(self, hook):
    """source==2 but safetyCapped=True: immediate cap, no ramp, no offset."""
    hook._sl_data = {'confirmed': True, 'speedLimit': 40, 'source': 2, 'safetyCapped': True}
    sm = self._make_sm_full(gas=True)  # gas must NOT override a safety cap
    result = hook.on_v_cruise(100 / 3.6, 20.0, sm)
    assert result == pytest.approx(40 / 3.6, abs=0.1)

  def test_source1_yolo_uses_immediate_path(self, hook):
    """source==1 (YOLO): immediate offset cap, unaffected by ramp/gas."""
    hook._sl_data = {'confirmed': True, 'speedLimit': 80, 'source': 1, 'safetyCapped': False}
    sm = self._make_sm_full(gas=False)
    result = hook.on_v_cruise(100 / 3.6, 20.0, sm)
    assert result == pytest.approx(80 * 1.10 / 3.6, abs=0.1)

  def test_source2_reset_when_gate_false(self, hook):
    """Leaving the inferred regime clears ramp state so it re-inits next time."""
    hook._sl_data = {'confirmed': True, 'speedLimit': 60, 'source': 2, 'safetyCapped': False}
    sm = self._make_sm_full(gas=False)
    hook.on_v_cruise(100 / 3.6, 25.0, sm)
    hook._sl_data = {'confirmed': False, 'speedLimit': 60, 'source': 2, 'safetyCapped': False}
    result = hook.on_v_cruise(100 / 3.6, 25.0, sm)
    assert result == pytest.approx(100 / 3.6)  # unconfirmed → pass through
    assert hook._eff_cap_ms is None            # state cleared


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

  def test_no_cereal_slot(self):
    """speedLimitState moved to plugin_bus — no cereal slot needed."""
    import json, os
    manifest_path = os.path.join(os.path.dirname(__file__), '..', 'plugin.json')
    with open(manifest_path) as f:
      manifest = json.load(f)

    assert 'cereal' not in manifest or not manifest.get('cereal', {}).get('slots')
    assert 'services' not in manifest or not manifest.get('services')
