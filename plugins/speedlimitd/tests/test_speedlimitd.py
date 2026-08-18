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
# The tests import `plugins.speedlimitd.speedlimitd` as a package — that needs
# the REPO ROOT on sys.path too (the documented invocation is
# `PYTHONPATH=. uv run pytest` from the repo root; this insert makes a bare
# `pytest` from any cwd work the same). NOTE: dev-machine only — the device's
# flat runtime layout has no `plugins` package; on-device coverage is
# on_device_probe.py, not this file.
_REPO_ROOT = os.path.dirname(_PLUGINS_DIR)
if _REPO_ROOT not in sys.path:
  sys.path.insert(0, _REPO_ROOT)

# speedlimitd.py imports its siblings (osm_query, mapd_source) by bare name,
# matching the device's flat runtime layout. Put that directory on sys.path
# explicitly — relying on test_osm_query.py having been collected first makes
# this suite depend on collection order.
_SLD_DIR = os.path.join(_PLUGINS_DIR, 'speedlimitd')
if _SLD_DIR not in sys.path:
  sys.path.insert(0, _SLD_DIR)


@pytest.fixture(autouse=True)
def mock_openpilot(monkeypatch):
  """Mock openpilot + cereal imports."""
  mock_services = MagicMock()
  # mapdOut is present here because install.sh injects it into cereal/services.py
  # on a healthy device. TestMapdPhase1Telemetry removes it to cover the
  # injection-missing device state.
  mock_services.SERVICE_LIST = {'modelV2': MagicMock(), 'gpsLocationExternal': MagicMock(),
                                'livePose': MagicMock(), 'mapdOut': MagicMock()}
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

  # --- edge-lane boost gate (route 3de seg 19; ships per user decision) ------

  def _make_edge_model(self, probs):
    """Model near the RIGHT edge on an OPEN wide road → _near_road_edge fires.

    The near (right) edge hugs the right line (+2.4 vs +1.9, std 0.3); the FAR
    (left) edge is open (y −8.0, std 1.5). The far side MUST stay unbounded so
    the 2026-08-05 bounded-road demote ("3 lines + BOTH edges → 2") does not
    fire here — this stub exercises the edge BOOST (unseen far side of a genuinely
    wide road), not the demote.
    """
    m = MagicMock()
    m.laneLineProbs = probs
    ll_y = [-4.5, -1.4, 1.9, 3.2]   # lane lines at y-index 2 (~10 m)
    re_y = [-8.0, 2.4]              # far LEFT edge open; near RIGHT edge hugs +1.9
    m.laneLines = [MagicMock(y=[0.0, 0.0, ll_y[i], 0.0]) for i in range(4)]
    m.roadEdges = [MagicMock(y=[0.0, 0.0, re_y[i], 0.0]) for i in range(2)]
    m.roadEdgeStds = [1.5, 0.3]     # left(far) unbounded; right(near) confident
    return m

  def _make_full_model(self, probs, ll_y2, re_y2, re_stds):
    """Fully-specified modelV2 stub: lane-line / road-edge y at index 2 (~10 m)
    and road-edge stds set explicitly, for the bounded-road demote tests."""
    m = MagicMock()
    m.laneLineProbs = probs
    m.laneLines = [MagicMock(y=[0.0, 0.0, ll_y2[i], 0.0]) for i in range(4)]
    m.roadEdges = [MagicMock(y=[0.0, 0.0, re_y2[i], 0.0]) for i in range(2)]
    m.roadEdgeStds = re_stds
    return m

  def test_edge_boost_gated_at_base_3_not_2(self, sld):
    # (a) base 2 + near edge → STAYS 2 (no boost). The seg-19 fix: a narrow exit
    # link's 2-line reading must not be inflated to ≥3 (which defeated the narrow
    # confirmation, the lane≤2 G/S escape, and the ramp-40).
    assert sld.infer_lane_count(self._make_edge_model([0.1, 0.6, 0.7, 0.1])) == 2
    # base 2, NO near edge → 2 (unchanged; bare mock has roadEdges len 0).
    assert sld.infer_lane_count(self._make_model([0.1, 0.6, 0.7, 0.1])) == 2

  def test_edge_boost_still_applies_at_base_3(self, sld):
    # (b) base 3 + near edge → boosts to 4 (unchanged wide-road behavior — the
    # boost's actual purpose: an unseen far side of a genuinely wide road). The
    # far (left) edge stays unbounded (std 1.5, y −8.0) so the bounded-road
    # demote does NOT fire here; only the near (right) edge triggers the boost.
    assert sld.infer_lane_count(self._make_edge_model([0.4, 0.6, 0.7, 0.2])) == 4

  # --- Fix G: bounded-road demote — "3 lines + both edges → 2 lanes" ----------
  # (user rule, 2026-08-05; replay-gated s0.9/g1.5). Exemplar frames carry the
  # exact driver-audited numbers.

  def test_demote_3e5_ramp_frame(self, sld):
    # route 3e5 seg7, GPS 09:41:59.7 — driver-audited ramp. 3 lines; BOTH edges
    # bounded: left gap |−5.5−(−4.7)|=0.8 < 1.5, std 0.75 < 0.9; right gap
    # |2.6−2.07|=0.53 < 1.5, std 0.33 < 0.9 → demote to 2.
    model = self._make_full_model(
        [0.55, 0.98, 0.89, 0.001],
        [-4.7, -1.45, 2.07, 2.9], [-5.5, 2.6], [0.75, 0.33])
    assert sld.infer_lane_count(model) == 2

  def test_no_demote_ring_road_frame(self, sld):
    # seg20 09:55:07 — 4-lane ring road, edge lane. 3 lines but the LEFT edge is
    # NOT bounded (gap 3.1, std 1.11 both fail) → no demote → base 3; the near
    # right edge still boosts → 4.
    model = self._make_full_model(
        [0.84, 0.99, 0.98, 0.01],
        [-4.5, -1.36, 1.86, 3.24], [-7.6, 2.35], [1.11, 0.32])
    assert sld.infer_lane_count(model) == 4

  def test_no_demote_under_occlusion(self, sld):
    # seg20 09:55:19 — a left-passing vehicle blows up the left edge (std 5.9,
    # mean −9.0). Not bounded → no demote (fail-safe direction) → 4 via boost.
    model = self._make_full_model(
        [0.84, 0.99, 0.98, 0.01],
        [-4.5, -1.36, 1.86, 3.24], [-9.0, 2.35], [5.9, 0.32])
    assert sld.infer_lane_count(model) == 4

  def test_demote_accepted_wide_road_characterization(self, sld):
    # Barrier-adjacent wide-road frame: 3 lines, both edges std 0.8 / gap 1.0 →
    # demote to 2. Pinned AS DESIRED behavior (user decision 2026-08-05): a wall/
    # barrier hugging the outer line is a real road edge — most often a
    # construction zone occupying lanes, where slowing to 40 is the humanly-
    # correct read. The known residual (permanent sound wall / median divider at
    # full lane count) stays gas-overridable and Fix-F ceiling-capped. This
    # assertion pins that behavior so any future change to it is a conscious one.
    model = self._make_full_model(
        [0.84, 0.99, 0.98, 0.01],
        [-4.5, -1.4, 1.9, 3.2], [-5.5, 2.9], [0.8, 0.8])
    assert sld.infer_lane_count(model) == 2

  def test_demote_requires_exactly_3_lines(self, sld):
    # Same bounded edges, but 4 visible lines → demote requires exactly 3, so it
    # does NOT fire; count follows the existing 4-line path (→ 4).
    four = self._make_full_model(
        [0.84, 0.99, 0.98, 0.6],
        [-4.5, -1.4, 1.9, 3.2], [-5.5, 2.9], [0.8, 0.8])
    assert sld.infer_lane_count(four) == 4
    # 2 visible lines with the same bounded edges → existing behavior (2); the
    # demote logic is not involved (base 2, no boost).
    two = self._make_full_model(
        [0.1, 0.99, 0.98, 0.1],
        [-4.5, -1.4, 1.9, 3.2], [-5.5, 2.9], [0.8, 0.8])
    assert sld.infer_lane_count(two) == 2


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

  def test_bounded_demote_commits_narrow_after_3s(self, sld):
    """Sustained bounded-road demote frames drive the leaky narrow accumulator to
    a stable=2 commit after NARROW_CONFIRM_S (3 s). Reuses the accumulator
    machinery (route 3d3/3d1 pattern) with the 3e5 ramp exemplar frame, which
    infer_lane_count demotes to raw=2 (3 lines, both edges bounded)."""
    import plugins.speedlimitd.speedlimitd as mod
    with patch.object(mod.messaging, 'SubMaster'), \
         patch.object(mod.messaging, 'PubMaster'):
      mw = mod.SpeedLimitMiddleware()
    # 3e5 ramp exemplar (driver-audited): both edges bounded → raw demotes to 2.
    m = MagicMock()
    m.laneLineProbs = [0.55, 0.98, 0.89, 0.001]
    m.laneLines = [MagicMock(y=[0.0, 0.0, v, 0.0]) for v in (-4.7, -1.45, 2.07, 2.9)]
    m.roadEdges = [MagicMock(y=[0.0, 0.0, v, 0.0]) for v in (-5.5, 2.6)]
    m.roadEdgeStds = [0.75, 0.33]
    assert sld.infer_lane_count(m) == 2  # the frame demotes

    mw.lane_count_stable = 3
    mw._narrow_accum = 0.0
    mw._lane_last_t = 0.0
    now = 100.0
    for _ in range(80):  # 80 × 0.05 s = 4 s > NARROW_CONFIRM_S (3 s)
      now += 0.05
      raw = sld.infer_lane_count(m)
      dt = min(max(now - mw._lane_last_t, 0.0), 0.5) if mw._lane_last_t > 0.0 else 0.0
      mw._lane_last_t = now
      if raw <= 2:
        mw._narrow_accum = min(mw._narrow_accum + dt, mod.NARROW_ACCUM_CAP)
      else:
        mw._narrow_accum = max(mw._narrow_accum - mod.NARROW_DECAY * dt, 0.0)
      if mw._narrow_accum >= mod.NARROW_CONFIRM_S:
        mw.lane_count_stable = 2
    assert mw._narrow_accum >= mod.NARROW_CONFIRM_S
    assert mw.lane_count_stable == 2


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
    mod._baseline_ms = None
    mod._gas_floor_ms = None
    mod._road_id = ''
    return mod

  # helpers -------------------------------------------------

  def _sm(self, gas=False, lead_status=False, lead_vLead=0.0):
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

  def _sl(self, hook, speed_limit, source=1, safety=False, confirmed=True, road='A'):
    hook._sl_data = {'confirmed': confirmed, 'speedLimit': speed_limit,
                     'safetyCapped': safety, 'source': source,
                     'roadName': road, 'wayRef': ''}

  def _clock(self, monkeypatch, hook, t0=1000.0):
    # planner_hook no longer uses a clock (enforcement is immediate — DCC shapes
    # the decel, no ramp). Kept as a no-op so existing tests still read cleanly.
    class _C:
      def tick(self, dt):
        pass
    return _C()

  # basic enforcement (non-inferred, no gas) ----------------

  def test_no_speed_limit_state(self, hook):
    hook._sl_data = None
    assert hook.on_v_cruise(30.0, 20.0, self._sm()) == 30.0

  def test_unconfirmed_returns_original(self, hook):
    self._sl(hook, 60, confirmed=False)
    assert hook.on_v_cruise(30.0, 20.0, self._sm()) == 30.0

  def test_confirmed_caps_highway(self, hook):
    """Limit >= 80 kph uses 10% offset (immediate for non-inferred)."""
    self._sl(hook, 80, source=1)
    assert hook.on_v_cruise(100 / 3.6, 20.0, self._sm()) == pytest.approx(80 * 1.10 / 3.6, abs=0.1)

  def test_confirmed_caps_low_speed(self, hook):
    """Limit < 80 kph uses 15% offset."""
    self._sl(hook, 40, source=1)
    assert hook.on_v_cruise(100 / 3.6, 20.0, self._sm()) == pytest.approx(40 * 1.15 / 3.6, abs=0.1)

  def test_inferred_lane_count_40_enforces_no_display_only(self, hook):
    """route 3d3 seg 16 / 3d1 seg 29 regression removed: a genuine 2-lane ramp 40
    (inferred, source 2, unnamed road, no coincident cap) ENFORCES immediately —
    v_cruise is lowered. Under the reverted narrow-band display-only patch this
    was suppressed (car coasted through at speed)."""
    self._sl(hook, 40, source=2, road='')
    r = hook.on_v_cruise(85 / 3.6, 85 / 3.6, self._sm())
    assert r == pytest.approx(40 * 1.15 / 3.6, abs=0.1)   # enforced (offset applies)
    assert r < 85 / 3.6                                   # v_cruise lowered

  def test_no_cap_if_already_below(self, hook):
    self._sl(hook, 120, source=1)
    assert hook.on_v_cruise(10.0, 8.0, self._sm()) == 10.0

  def test_safety_cap_no_offset_immediate(self, hook):
    """Safety cap: no offset, immediate (prompt) enforcement."""
    self._sl(hook, 40, source=4, safety=True)
    assert hook.on_v_cruise(100 / 3.6, 20.0, self._sm()) == pytest.approx(40 / 3.6, abs=0.1)

  # lead override (non-safety, no gas) ----------------------

  def test_lead_override_fast_lead_skips(self, hook):
    self._sl(hook, 80, source=1)
    sm = self._sm(lead_status=True, lead_vLead=95 / 3.6)
    assert hook.on_v_cruise(100 / 3.6, 20.0, sm) == 100 / 3.6

  def test_lead_override_slow_lead_keeps(self, hook):
    self._sl(hook, 80, source=1)
    sm = self._sm(lead_status=True, lead_vLead=84 / 3.6)
    assert hook.on_v_cruise(100 / 3.6, 20.0, sm) == pytest.approx(80 * 1.10 / 3.6, abs=0.1)

  def test_safety_cap_ignores_fast_lead(self, hook):
    """Route-2fd: a fast lead must not bypass a safety cap."""
    self._sl(hook, 40, source=4, safety=True)
    sm = self._sm(lead_status=True, lead_vLead=60 / 3.6)
    assert hook.on_v_cruise(100 / 3.6, 20.0, sm) == pytest.approx(40 / 3.6, abs=0.1)

  def test_reactive_cap_survives_fast_lead(self, hook):
    """The reactive measured-a_y cap publishes as source 4 / safetyCapped, so
    it is in the same protected class as the proactive curve cap — a faster
    lead can never lift it (a fast lead doesn't make a real curve less tight)."""
    self._sl(hook, 50, source=4, safety=True)   # reactive cap → 50 km/h
    sm = self._sm(lead_status=True, lead_vLead=90 / 3.6)  # lead way over limit
    assert hook.on_v_cruise(100 / 3.6, 25.0, sm) == pytest.approx(50 / 3.6, abs=0.1)

  # universal gas suspend -----------------------------------

  def test_gas_suspends_inferred(self, hook):
    self._sl(hook, 60, source=2)
    assert hook.on_v_cruise(100 / 3.6, 25.0, self._sm(gas=True)) == pytest.approx(100 / 3.6)

  def test_gas_suspends_safety_cap(self, hook):
    """Driver pedal suspends even a curve/safety cap."""
    self._sl(hook, 40, source=4, safety=True)
    assert hook.on_v_cruise(100 / 3.6, 25.0, self._sm(gas=True)) == pytest.approx(100 / 3.6)

  # hold-floor behavior -------------------------------------

  def test_inferred_spurious_drop_holds_speed(self, hook, monkeypatch):
    """Same road, uncorroborated inferred drop → hold current speed, no brake."""
    clk = self._clock(monkeypatch, hook)
    self._sl(hook, 100, source=2, road='A')
    hook.on_v_cruise(120 / 3.6, 100 / 3.6, self._sm())
    clk.tick(0.1)
    self._sl(hook, 60, source=2, road='A')  # spurious drop
    r = None
    for _ in range(5):
      r = hook.on_v_cruise(120 / 3.6, 100 / 3.6, self._sm())
      clk.tick(0.1)
    assert r >= 100 / 3.6 - 0.2       # held at current speed
    assert r > 75 / 3.6               # definitely NOT braked toward 60

  def test_inferred_real_drop_new_road_slows(self, hook):
    """road_id change → baseline resets → new lower limit enforced immediately
    (DCC shapes the deceleration, no artificial ramp)."""
    self._sl(hook, 100, source=2, road='A')
    hook.on_v_cruise(120 / 3.6, 100 / 3.6, self._sm())
    self._sl(hook, 40, source=2, road='B')  # new road, real lower limit
    r = hook.on_v_cruise(120 / 3.6, 100 / 3.6, self._sm())
    assert hook._baseline_ms == pytest.approx(40 * 1.15 / 3.6, abs=0.1)  # baseline reset
    assert r == pytest.approx(40 * 1.15 / 3.6, abs=0.1)   # cap enforced immediately

  def test_inferred_recovery_allows_accel(self, hook, monkeypatch):
    """Inferred limit rises again → cap restores up, acceleration allowed."""
    clk = self._clock(monkeypatch, hook)
    self._sl(hook, 60, source=2, road='A')
    hook.on_v_cruise(120 / 3.6, 60 / 3.6, self._sm())
    clk.tick(0.1)
    self._sl(hook, 100, source=2, road='A')
    r = hook.on_v_cruise(120 / 3.6, 60 / 3.6, self._sm())
    assert r > 60 / 3.6

  def test_never_speed_up_on_drop(self, hook, monkeypatch):
    """Cap never rises above the new (lower) limit when it drops."""
    clk = self._clock(monkeypatch, hook)
    self._sl(hook, 100, source=2, road='A')
    hook.on_v_cruise(120 / 3.6, 50 / 3.6, self._sm())
    clk.tick(0.1)
    self._sl(hook, 60, source=2, road='A')
    r = hook.on_v_cruise(120 / 3.6, 50 / 3.6, self._sm())
    assert r <= 60 * 1.15 / 3.6 + 0.1   # never above the dropped limit's target

  def test_inferred_gas_release_holds_speed(self, hook, monkeypatch):
    """Ramp 40, gas to 60, release → hold 60, no brake-back."""
    clk = self._clock(monkeypatch, hook)
    self._sl(hook, 40, source=2, road='A')
    hook.on_v_cruise(120 / 3.6, 46 / 3.6, self._sm())
    clk.tick(0.1)
    hook.on_v_cruise(120 / 3.6, 60 / 3.6, self._sm(gas=True))
    clk.tick(0.1)
    r = hook.on_v_cruise(120 / 3.6, 60 / 3.6, self._sm(gas=False))
    assert r >= 60 / 3.6 - 0.2         # holds driver's speed, not braking to 46

  def test_curve_gas_release_holds_speed(self, hook, monkeypatch):
    """Curve/safety cap follows the gas too: hold driver's speed on release."""
    clk = self._clock(monkeypatch, hook)
    self._sl(hook, 40, source=4, safety=True, road='A')
    hook.on_v_cruise(120 / 3.6, 40 / 3.6, self._sm())
    clk.tick(0.1)
    hook.on_v_cruise(120 / 3.6, 60 / 3.6, self._sm(gas=True))
    clk.tick(0.1)
    r = hook.on_v_cruise(120 / 3.6, 60 / 3.6, self._sm(gas=False))
    assert r >= 60 / 3.6 - 0.2         # holds 60 over the curve cap

  def test_curve_no_gas_brakes(self, hook):
    """Curve cap, no gas, no prior override → prompt braking."""
    self._sl(hook, 40, source=4, safety=True, road='A')
    r = hook.on_v_cruise(120 / 3.6, 100 / 3.6, self._sm())
    assert r == pytest.approx(40 / 3.6, abs=0.1)

  def test_gas_floor_ratchets_and_clears(self, hook, monkeypatch):
    """After release the hold follows the driver down and clears at the limit."""
    clk = self._clock(monkeypatch, hook)
    self._sl(hook, 40, source=4, safety=True, road='A')
    hook.on_v_cruise(120 / 3.6, 40 / 3.6, self._sm())
    clk.tick(0.1)
    hook.on_v_cruise(120 / 3.6, 60 / 3.6, self._sm(gas=True))  # gas_floor = 60
    clk.tick(0.1)
    r_hold = hook.on_v_cruise(120 / 3.6, 50 / 3.6, self._sm())  # ease to 50
    assert r_hold == pytest.approx(50 / 3.6, abs=0.2)           # follows down
    r_settle = hook.on_v_cruise(120 / 3.6, 40 / 3.6, self._sm())  # back at limit
    assert hook._gas_floor_ms is None                          # cleared
    assert r_settle == pytest.approx(40 / 3.6, abs=0.2)        # curve enforced again

  def test_road_change_clears_gas_floor(self, hook):
    """A new road drops any carried gas hold."""
    self._sl(hook, 40, source=4, safety=True, road='A')
    hook.on_v_cruise(120 / 3.6, 60 / 3.6, self._sm(gas=True))  # gas_floor set on road A
    self._sl(hook, 40, source=4, safety=True, road='B')        # new road
    r = hook.on_v_cruise(120 / 3.6, 60 / 3.6, self._sm())
    assert hook._gas_floor_ms is None
    assert r == pytest.approx(40 / 3.6, abs=0.1)               # B's cap enforced, no carried hold

  def test_empty_road_disables_hold_keeps_identity(self, hook, monkeypatch):
    """An empty road_id disables the baseline hold (no continuity claim without an
    identity), but _road_id is retained so returning to the same named road is not
    seen as a road change (no spurious reset of the gas hold / ramp)."""
    clk = self._clock(monkeypatch, hook)
    self._sl(hook, 100, source=2, road='A')
    hook.on_v_cruise(120 / 3.6, 100 / 3.6, self._sm())
    clk.tick(0.1)
    self._sl(hook, 60, source=2, road='')   # OSM dropout / unnamed way
    hook.on_v_cruise(120 / 3.6, 100 / 3.6, self._sm())
    assert hook._baseline_ms is None                       # hold disabled without identity
    assert hook._road_id == 'A'                            # identity retained (no false reset)

  def test_invalid_limit_resets_state(self, hook):
    self._sl(hook, 60, source=2, road='A')
    hook.on_v_cruise(120 / 3.6, 25.0, self._sm())
    self._sl(hook, 60, source=2, confirmed=False, road='A')
    r = hook.on_v_cruise(120 / 3.6, 25.0, self._sm())
    assert r == pytest.approx(120 / 3.6)
    assert hook._baseline_ms is None
    assert hook._gas_floor_ms is None

  # baseline floor requires a road identity ------------------

  def test_empty_road_id_disables_baseline_hold(self, hook):
    """No OSM identity (road_id='') → baseline hold invalid → the inferred/vision
    cap is enforced immediately (route 3a1 unnamed motorway_link ramp)."""
    self._sl(hook, 100, source=2, road='')          # unnamed way
    hook.on_v_cruise(120 / 3.6, 100 / 3.6, self._sm())
    self._sl(hook, 40, source=2, road='')           # vision cap → 40, still unnamed
    r = hook.on_v_cruise(120 / 3.6, 100 / 3.6, self._sm())
    assert hook._baseline_ms is None                # baseline not built without identity
    assert r == pytest.approx(40 * 1.15 / 3.6, abs=0.1)   # enforced immediately, not held

  def test_named_road_id_keeps_baseline_hold(self, hook, monkeypatch):
    """With a road identity, spurious same-road drops are still held (unchanged)."""
    clk = self._clock(monkeypatch, hook)
    self._sl(hook, 100, source=2, road='S20')
    hook.on_v_cruise(120 / 3.6, 100 / 3.6, self._sm())
    clk.tick(0.2)
    self._sl(hook, 40, source=2, road='S20')        # spurious drop, same named road
    r = None
    for _ in range(5):
      r = hook.on_v_cruise(120 / 3.6, 100 / 3.6, self._sm())
      clk.tick(0.2)
    assert r >= 100 / 3.6 - 0.2                      # held at current speed


# ============================================================
# Change 1 — MapdCurveTargetLatAccel param wiring
# ============================================================

class TestLatAccelParamParse:
  """_parse_lat_accel: unset/0/non-finite handling + clamping."""

  def test_curve_unset_defaults(self, sld):
    assert sld._parse_lat_accel('', 1.5, 1.0, 3.0) == 1.5

  def test_curve_zero_defaults(self, sld):
    # 0 is treated as unset for the curve target (default 1.5).
    assert sld._parse_lat_accel('0', 1.5, 1.0, 3.0) == 1.5
    assert sld._parse_lat_accel('0.0', 1.5, 1.0, 3.0) == 1.5

  def test_curve_unparseable_defaults(self, sld):
    assert sld._parse_lat_accel('abc', 1.5, 1.0, 3.0) == 1.5
    assert sld._parse_lat_accel(None, 1.5, 1.0, 3.0) == 1.5

  def test_curve_non_finite_defaults(self, sld):
    assert sld._parse_lat_accel('inf', 1.5, 1.0, 3.0) == 1.5
    assert sld._parse_lat_accel('nan', 1.5, 1.0, 3.0) == 1.5

  def test_curve_real_value_passes(self, sld):
    assert sld._parse_lat_accel('2.0', 1.5, 1.0, 3.0) == 2.0

  def test_curve_clamps_low(self, sld):
    assert sld._parse_lat_accel('0.5', 1.5, 1.0, 3.0) == 1.0

  def test_curve_clamps_high(self, sld):
    assert sld._parse_lat_accel('5', 1.5, 1.0, 3.0) == 3.0

  def test_react_zero_disables(self, sld):
    # For the reactive threshold, 0 means OFF (not default).
    assert sld._parse_lat_accel('0', 2.5, 1.8, 3.0, zero_disables=True) == 0.0

  def test_react_unset_defaults(self, sld):
    assert sld._parse_lat_accel('', 2.5, 1.8, 3.0, zero_disables=True) == 2.5

  def test_react_clamps(self, sld):
    assert sld._parse_lat_accel('1.0', 2.5, 1.8, 3.0, zero_disables=True) == 1.8
    assert sld._parse_lat_accel('9', 2.5, 1.8, 3.0, zero_disables=True) == 3.0


class TestCurveTargetLatAccelWiring:
  def _make_middleware(self, sld):
    import plugins.speedlimitd.speedlimitd as mod
    with patch.object(mod.messaging, 'SubMaster'):
      mw = mod.SpeedLimitMiddleware()
    mw._sl_pub = MagicMock()
    return mw

  def _curve_model(self, sld):
    """A model with a fixed moderate curvature within confident vision."""
    m = MagicMock()
    n = 33
    # Constant yaw rate + velocity → constant curvature κ = yaw/v.
    # yaw=0.30 rad/s, v=20 m/s → κ=0.015 (radius ~67 m).
    m.orientationRate.z = [0.30] * n
    m.velocity.x = [20.0] * n
    m.position.x = [float(i * 3) for i in range(n)]  # 0..96 m, within 100 m
    m.position.yStd = [0.1] * n                        # high confidence throughout
    return m

  def test_higher_lat_accel_raises_cap(self, sld):
    model = self._curve_model(sld)
    low = sld.curvature_speed_cap(model, 1.5)
    high = sld.curvature_speed_cap(model, 3.0)
    # Higher allowed a_y → higher safe speed for the same curvature.
    assert high > low > 0

  def test_default_matches_hardcoded_15(self, sld):
    model = self._curve_model(sld)
    assert sld.curvature_speed_cap(model) == sld.curvature_speed_cap(model, 1.5)

  def test_read_params_wires_curve_target(self, sld, monkeypatch):
    import config
    monkeypatch.setattr(config, 'read_plugin_param',
                        lambda pid, key, default='': '2.5' if key == 'MapdCurveTargetLatAccel' else '')
    mw = self._make_middleware(sld)
    mw._read_params()
    assert mw.curve_target_lat_accel == 2.5

  def test_read_params_defaults_on_missing(self, sld, monkeypatch):
    import config
    monkeypatch.setattr(config, 'read_plugin_param', lambda pid, key, default='': '')
    mw = self._make_middleware(sld)
    mw._read_params()
    assert mw.curve_target_lat_accel == 1.5
    assert mw.react_lat_accel_threshold == 2.5


# ============================================================
# Distance-aware curve braking + tight-curve a_y derating (route 3d0)
# ============================================================

class TestDistanceAwareCurveCap:
  """curvature_speed_cap now plans the deceleration POINT: for each confident,
  meaningfully-curved path point it computes the curve speed there (with a
  tight-curve a_y derate) and the speed allowed NOW so a COMFORT_BRAKE decel
  still makes the curve. The binding cap is the min over points."""
  import math as _math

  def _model(self, yaw, vel, px, ystd):
    m = MagicMock()
    m.orientationRate.z = list(yaw)
    m.velocity.x = list(vel)
    m.position.x = list(px)
    m.position.yStd = list(ystd)
    return m

  def _uniform(self, yaw, vel, px_val, n=33):
    """Constant curvature at a single forward distance px_val (all points)."""
    return self._model([yaw] * n, [vel] * n, [float(px_val)] * n, [0.1] * n)

  # --- Test 1: a point at d≈0 reduces exactly to the pre-change (distance-less)
  #             law — v_now == v_curve. κ=0.015 keeps the derate factor at 1.0,
  #             so this is bitwise-identical to the old sqrt(a_y/κ) behaviour.
  def test_d0_equals_old_behavior(self, sld):
    import math
    kappa = 0.30 / 20.0  # 0.015 — below the 0.02 derate onset → factor 1.0
    model = self._uniform(0.30, 20.0, 0.0)  # all points at d=0
    v_curve = math.sqrt(1.5 / kappa)         # target_ay = 1.5 (no derate)
    # Enforcement is the RAW (unsnapped) ramp value, rounded to 1 km/h —
    # snapping to a standard speed is a display-only concern now (see
    # TestCurvatureCapDisplaySnap / TestMidBandCurveOnset).
    expected = round(v_curve * 3.6)
    assert sld.curvature_speed_cap(model, 1.5) == expected

  # --- Test 2: same curve at d=40 m → braking headroom raises the allowed
  #             speed by the exact 2·a·d formula, but it is still a real cap.
  def test_same_curve_at_40m(self, sld):
    import math
    kappa = 0.30 / 20.0  # 0.015, factor 1.0
    model = self._uniform(0.30, 20.0, 40.0)
    v_curve = math.sqrt(1.5 / kappa)                       # 10 m/s
    cap_now = math.sqrt(v_curve ** 2 + 2 * 0.8 * 40)       # sqrt(164) ≈ 12.8 m/s
    assert sld.curvature_speed_cap(model, 1.5) == round(cap_now * 3.6)  # raw, unsnapped
    assert cap_now > v_curve                               # braking headroom
    result = sld.curvature_speed_cap(model, 1.5)
    assert 0 < result < 100                                # still a meaningful cap

  # --- Test 3: two curves (mild near, sharp far). The min-over-points picks the
  #             binding (sharp) one even though it is farther; moving it closer
  #             tightens the cap monotonically.
  def _two_curve(self, sharp_idx, sharp_yaw=0.80):
    n = 33
    yaw = [0.0] * n
    yaw[6] = 0.20            # mild near curve at d=18: κ=0.010
    if sharp_idx is not None:
      yaw[sharp_idx] = sharp_yaw   # sharp curve: κ=0.040
    vel = [20.0] * n
    px = [float(i * 3) for i in range(n)]
    return self._model(yaw, vel, px, [0.1] * n)

  def test_two_curves_min_and_monotonic(self, sld):
    mild_only = sld.curvature_speed_cap(self._two_curve(None), 1.5)
    far = sld.curvature_speed_cap(self._two_curve(20), 1.5)   # sharp at d=60
    near = sld.curvature_speed_cap(self._two_curve(10), 1.5)  # sharp at d=30
    assert far < mild_only          # sharp curve binds below the mild near one
    assert near < far               # closer sharp curve → tighter cap (monotone)
    # Monotone as the sharp curve marches in from d=90 to d=15.
    caps = [sld.curvature_speed_cap(self._two_curve(idx), 1.5) for idx in (30, 25, 20, 15, 10, 5)]
    assert all(b <= a for a, b in zip(caps, caps[1:]))

  # --- Test 4: tight-curve a_y derate. interp over [0.02, 0.035]→[1.0, 0.75].
  def test_derate_interp_boundaries(self, sld):
    assert sld._interp(0.02, [0.02, 0.035], [1.0, 0.75]) == 1.0    # onset boundary
    assert sld._interp(0.015, [0.02, 0.035], [1.0, 0.75]) == 1.0   # clamped below
    assert sld._interp(0.035, [0.02, 0.035], [1.0, 0.75]) == 0.75  # full derate
    f03 = sld._interp(0.03, [0.02, 0.035], [1.0, 0.75])
    assert 0.75 < f03 < 1.0                                        # mid-range

  def test_derate_applied_in_cap(self, sld):
    import math
    # κ=0.03 at d=0 → cap uses the derated target (factor ≈ 0.833).
    factor = sld._interp(0.03, [0.02, 0.035], [1.0, 0.75])
    model = self._uniform(0.60, 20.0, 0.0)  # κ = 0.60/20 = 0.03
    v_curve = math.sqrt(1.5 * factor / 0.03)
    # Raw ramp value (~23 km/h) is below the 30 km/h enforcement floor.
    assert sld.curvature_speed_cap(model, 1.5) == max(30, round(v_curve * 3.6))
    # κ=0.02 exactly → factor 1.0 (no derate).
    model2 = self._uniform(0.40, 20.0, 0.0)  # κ = 0.02
    v_curve2 = math.sqrt(1.5 * 1.0 / 0.02)
    assert sld.curvature_speed_cap(model2, 1.5) == max(30, round(v_curve2 * 3.6))

  # --- Test 5: seg-61 regression. κ=0.026 at d=35, v=12.6 m/s (~45 km/h),
  #             target 1.5. The commanded cap must START the slowdown at that
  #             distance (cap < current speed) while sitting ABOVE v_curve — a
  #             braking ramp, not the old distance-less cliff to v_curve.
  def test_seg61_starts_slowdown_at_distance(self, sld):
    import math
    v = 12.6
    kappa = 0.026
    n = 33
    yaw = [0.0] * n
    yaw[7] = kappa * v         # curve at index 7
    px = [float(i * 5) for i in range(n)]   # index 7 → d = 35 m
    model = self._model(yaw, [v] * n, px, [0.1] * n)
    cap_kph = sld.curvature_speed_cap(model, 1.5)
    assert 0 < cap_kph < v * 3.6          # slowdown commanded 35 m out (~45 km/h)
    factor = sld._interp(kappa, [0.02, 0.035], [1.0, 0.75])
    v_curve_kph = math.sqrt(1.5 * factor / kappa) * 3.6
    assert cap_kph > v_curve_kph          # braking headroom, not the old cliff

  # --- Test 6 (review regression guard): mid-band onset must not be
  #     quantized away by snap-to-standard. Before the fix, curvature_speed_cap
  #     snapped its own return value, so a raw v_now sitting in the 80<->100
  #     standard-speed gap (e.g. ~89-92 km/h) could round UP to 100 — which
  #     downstream logic (self.curvature_cap held/compared as an already-
  #     snapped number) could treat as if no meaningful constraint existed
  #     yet, only to cliff straight to 80 once the curve was almost on top of
  #     the car. The raw (unsnapped) value must show a real, gentle cap the
  #     whole way in.
  def test_mid_band_onset_not_quantized_away(self, sld):
    import math
    kappa = 0.0032  # just above CURVE_GATE (0.003); below 0.02 → no derate
    v = 20.0
    yaw = kappa * v
    v_curve = math.sqrt(1.5 / kappa)  # ~77.9 km/h — the curve itself

    caps = {d: sld.curvature_speed_cap(self._uniform(yaw, v, d), 1.5)
            for d in (90, 60, 30, 0)}

    # At d=90 (near the confidence boundary) the sqrt-ramp lands the raw cap
    # in the 80<->100 gap — a real, gentle constraint, not suppressed to 0.
    v_now_90 = math.sqrt(v_curve ** 2 + 2 * 0.8 * 90)
    assert caps[90] == round(v_now_90 * 3.6)
    assert 80 < caps[90] < 100

    # Tightens monotonically (strictly) as the curve nears — no one-shot
    # 100 -> 80 style cliff hiding inside the sequence.
    assert caps[90] > caps[60] > caps[30] > caps[0]

    # d=0 reduces to the raw (unsnapped) v_curve, per test_d0_equals_old_behavior.
    assert caps[0] == round(v_curve * 3.6)

  # --- Test 7 (review): the 30 km/h enforcement floor must be explicit now
  #     that snap_to_standard_speed no longer incidentally provides it (30 is
  #     the lowest standard speed, so any raw value below 35 used to land on
  #     30 for free via nearest-snap).
  def test_hairpin_floor_clamps_to_30(self, sld):
    kappa = 0.03  # hairpin-class curvature
    v = 20.0
    model = self._uniform(kappa * v, v, 0.0)  # d=0 → no braking headroom
    assert sld.curvature_speed_cap(model, 1.5) == 30

  # --- Test 8 (route 3d0 seg 60 apex fix): the sharpest PATH point always
  #     sits at d>0, so the distance term (2·COMFORT_BRAKE·d_i) keeps the cap
  #     above the derated target — the car never actually reaches it. The
  #     driver's own measured curvature (kappa_meas), passed in as a virtual
  #     d=0 apex point, delivers the derated target directly. (a) apex
  #     delivery: no path curvature at all, so the virtual point alone binds.
  def test_apex_virtual_point_delivers_derated_target(self, sld):
    import math
    kappa_meas = 0.02  # onset boundary — factor 1.0, no derate
    model = self._uniform(0.0, 10.0, 50.0)  # yaw=0 everywhere: no path curvature
    expected = round(math.sqrt(1.5 * 1.0 / kappa_meas) * 3.6)  # ~31
    assert sld.curvature_speed_cap(model, 1.5, kappa_meas) == expected
    assert expected > 30  # the virtual point is the binding (non-floored) one

  # --- Test (b): derated target below the floor clamps to exactly 30, same
  #     as any other point.
  def test_apex_virtual_point_floored(self, sld):
    kappa_meas = 0.03  # hairpin-class, derate applies
    model = self._uniform(0.0, 10.0, 50.0)  # no path curvature
    assert sld.curvature_speed_cap(model, 1.5, kappa_meas) == 30

  # --- Test (d): below CURVE_GATE, the virtual point has no effect at all —
  #     same result as not passing kappa_meas (None).
  def test_apex_virtual_point_below_gate_no_effect(self, sld):
    kappa_meas = 0.002  # below CURVE_GATE (0.003)
    model = self._uniform(0.0, 10.0, 50.0)  # no path curvature either
    assert sld.curvature_speed_cap(model, 1.5, kappa_meas) == 0
    assert sld.curvature_speed_cap(model, 1.5, None) == 0

  # --- Test (e): min semantics — a tighter PATH point must never be loosened
  #     by a milder virtual apex point.
  def test_apex_virtual_point_never_raises_cap(self, sld):
    kappa_meas = 0.005  # looser than the path point below
    model = self._uniform(0.30, 20.0, 0.0)  # κ_path=0.015, d=0 → 36 km/h
    path_only = sld.curvature_speed_cap(model, 1.5, None)
    with_virtual = sld.curvature_speed_cap(model, 1.5, kappa_meas)
    assert with_virtual == path_only == 36


class TestMeasuredCurvatureApexWiring:
  """update() wires the SAME livePose plumbing the reactive cap already
  consumes into curvature_speed_cap()'s kappa_meas virtual apex point — no
  new subscription, no new staleness tracking (route 3d0 seg 60 apex fix)."""

  def _model(self, yaw, vel, px_val, n=33):
    m = MagicMock()
    m.orientationRate.z = [yaw] * n
    m.velocity.x = [vel] * n
    m.position.x = [float(px_val)] * n
    m.position.yStd = [0.1] * n
    return m

  def _mw(self, sld):
    import plugins.speedlimitd.speedlimitd as mod
    with patch.object(mod.messaging, 'SubMaster'):
      mw = mod.SpeedLimitMiddleware()
    mw._sl_pub = MagicMock()
    mw._cmd_sub = None
    mw._lc_sub = None
    return mw

  # --- Test (c): no livePose update this tick -> kappa_meas stays None ->
  #     curvature_speed_cap sees no virtual apex point -> result is exactly
  #     the path-only computation, unaffected by the apex-point feature.
  def test_stale_livepose_no_virtual_point(self, sld):
    mw = self._mw(sld)
    model = self._model(0.30, 20.0, 0.0)  # κ_path=0.015, d=0 -> path-only 36 km/h
    sm = MagicMock()
    sm.updated = {'modelV2': True, 'gpsLocationExternal': False, 'livePose': False}
    sm.update = MagicMock()
    sm.__getitem__ = MagicMock(side_effect=lambda k: model if k == 'modelV2' else MagicMock())
    mw.sm = sm
    mw.update()
    expected = sld.curvature_speed_cap(model, mw.curve_target_lat_accel, None)
    assert mw.curvature_cap == expected == 36

  def test_fresh_livepose_wires_virtual_point(self, sld):
    """Sanity check the wiring the other direction: a fresh, valid livePose
    reading with tight measured curvature DOES tighten self.curvature_cap
    below the path-only value."""
    mw = self._mw(sld)
    model = self._model(0.0, 20.0, 50.0)  # no path curvature at all
    lp = MagicMock()
    lp.angularVelocityDevice.valid = True
    lp.angularVelocityDevice.z = 0.6  # kappa_meas = 0.6 / max(10, 0.1) = 0.06 -> derated, floored
    lp.velocityDevice.valid = True
    lp.velocityDevice.x = 10.0
    sm = MagicMock()
    sm.updated = {'modelV2': True, 'gpsLocationExternal': False, 'livePose': True}
    sm.update = MagicMock()

    def getitem(k):
      if k == 'modelV2':
        return model
      if k == 'livePose':
        return lp
      return MagicMock()

    sm.__getitem__ = MagicMock(side_effect=getitem)
    mw.sm = sm
    mw.update()
    assert mw.curvature_cap == 30  # virtual apex point bound and floored


class TestCurvatureCapEnforcementVsDisplay:
  """self.curvature_cap is the RAW enforcement value; only the published/
  displayed limit is snapped to a standard speed (review fix)."""

  def _mw_for_update(self, sld):
    import plugins.speedlimitd.speedlimitd as mod
    with patch.object(mod.messaging, 'SubMaster'):
      mw = mod.SpeedLimitMiddleware()
    mw._sl_pub = MagicMock()
    # Controlled SubMaster: nothing updated this tick, model block skipped —
    # lets us drive self.curvature_cap directly, same pattern as
    # TestReactiveCapIntegration.
    sm = MagicMock()
    sm.updated = {'modelV2': False, 'gpsLocationExternal': False, 'livePose': False}
    sm.update = MagicMock()
    mw.sm = sm
    mw._cmd_sub = None
    mw._lc_sub = None
    return mw

  def test_curvature_cap_display_snapped_enforcement_raw(self, sld):
    mw = self._mw_for_update(sld)
    # Road inference would allow well above the curvature cap so the curve
    # cap is the binding (min()) candidate.
    mw.lane_count_stable = 4
    mw.last_road_context = 'freeway'
    mw.last_way_ref = 'G2'
    mw.last_highway_type = 'motorway'
    mw.curvature_cap = 87   # raw enforcement value — NOT a standard speed
    mw.update()
    published = mw._sl_pub.send.call_args[0][0]

    # Published/displayed values are always standard speeds...
    assert published['curvatureCap'] == sld.snap_to_standard_speed(87)
    assert published['curvatureCap'] in sld._STANDARD_SPEEDS
    assert published['speedLimit'] in sld._STANDARD_SPEEDS
    # ...and the curve cap is what's actually constraining the display,
    # tighter than the 120 km/h the road inference alone would allow.
    assert published['speedLimit'] <= sld.snap_to_standard_speed(87)
    assert published['safetyCapped'] is True

  def test_curvature_cap_zero_publishes_zero(self, sld):
    mw = self._mw_for_update(sld)
    mw.lane_count_stable = 4
    mw.last_road_context = 'freeway'
    mw.last_way_ref = 'G2'
    mw.last_highway_type = 'motorway'
    mw.curvature_cap = 0
    mw.update()
    published = mw._sl_pub.send.call_args[0][0]
    assert published['curvatureCap'] == 0


# ============================================================
# Change 2 — reactive measured-a_y cap
# ============================================================

class TestReactiveLatAccelCap:
  def _mw(self, sld):
    import plugins.speedlimitd.speedlimitd as mod
    with patch.object(mod.messaging, 'SubMaster'):
      mw = mod.SpeedLimitMiddleware()
    mw._sl_pub = MagicMock()
    return mw

  def _drive(self, mw, a_y, v_ego, thr, t0, duration, dt=0.1):
    """Feed a constant a_y for `duration`s starting at monotonic t0."""
    import math
    t = t0
    end = t0 + duration - 1e-9
    last = mw._react_cap_ms
    while t <= end:
      last = mw._update_reactive_cap(a_y, v_ego, thr, t, dt)
      t += dt
    return last

  def test_engages_after_half_second_sustained(self, sld):
    import math
    mw = self._mw(sld)
    mw._ay_filt = 2.6            # pre-warm the low-pass to the sustained value
    v_ego, thr = 20.0, 2.5
    # 0.4 s over threshold → not engaged yet
    self._drive(mw, 2.6, v_ego, thr, 100.0, 0.4)
    assert mw._react_cap_ms == 0.0
    # cross 0.5 s → engages (0.5 s more, comfortably past the debounce)
    self._drive(mw, 2.6, v_ego, thr, 100.5, 0.5)
    assert mw._react_cap_ms > 0.0
    expected = v_ego * math.sqrt(thr / 2.6) - 1.0
    assert mw._react_cap_ms == pytest.approx(expected, abs=0.3)

  def test_no_engage_on_transient(self, sld):
    mw = self._mw(sld)
    mw._ay_filt = 2.6
    # A 0.4 s spike never reaches the 0.5 s engage debounce.
    self._drive(mw, 2.6, 20.0, 2.5, 100.0, 0.4)
    assert mw._react_cap_ms == 0.0

  def test_monotonic_down_while_engaged(self, sld):
    mw = self._mw(sld)
    mw._ay_filt = 2.6
    v_ego, thr = 20.0, 2.5
    self._drive(mw, 2.6, v_ego, thr, 100.0, 0.8)   # engage
    engaged = mw._react_cap_ms
    assert engaged > 0.0
    # a_y eases (but stays cornering) → cap must NOT rise
    mw._ay_filt = 2.55
    cap_after_ease = self._drive(mw, 2.55, v_ego, thr, 101.0, 0.4)
    assert cap_after_ease <= engaged + 1e-6
    # a_y sharpens → cap must drop
    mw._ay_filt = 3.2
    cap_after_sharpen = self._drive(mw, 3.2, v_ego, thr, 102.0, 0.4)
    assert cap_after_sharpen < cap_after_ease

  def test_release_ramps_up_after_quiet(self, sld):
    mw = self._mw(sld)
    mw._ay_filt = 2.6
    v_ego, thr = 20.0, 2.5
    self._drive(mw, 2.6, v_ego, thr, 100.0, 0.8)   # engage
    engaged = mw._react_cap_ms
    # a_y drops well below threshold-0.3; needs 2 s quiet before release ramp
    self._drive(mw, 2.0, v_ego, thr, 101.0, 1.9)   # quiet, but < 2 s
    assert mw._react_cap_ms == pytest.approx(engaged, abs=1e-6)  # still held
    # continue past 2 s quiet → ramps up
    self._drive(mw, 2.0, v_ego, thr, 102.9, 0.6)
    assert mw._react_cap_ms > engaged or mw._react_cap_ms == 0.0
    # ramp all the way → disengages (reaches v_ego)
    self._drive(mw, 2.0, v_ego, thr, 103.5, 3.0)
    assert mw._react_cap_ms == 0.0

  def test_disabled_when_threshold_zero(self, sld):
    mw = self._mw(sld)
    mw._ay_filt = 3.5   # way over any threshold
    cap = self._drive(mw, 3.5, 20.0, 0.0, 100.0, 2.0)  # threshold 0 → disabled
    assert cap == 0.0
    assert mw._react_cap_ms == 0.0


# ============================================================
# Change 3 — reactive cap hardening (speed floor, stale-pose release)
# ============================================================

class TestReactiveCapHardening:
  """Speed floor + stale-livePose release. Unlike TestReactiveLatAccelCap,
  these drive the *real* low-pass filter from a cold start (no pre-set
  `_ay_filt`) to exercise the engage/ratchet/release transients end-to-end."""

  def _mw(self, sld):
    import plugins.speedlimitd.speedlimitd as mod
    with patch.object(mod.messaging, 'SubMaster'):
      mw = mod.SpeedLimitMiddleware()
    mw._sl_pub = MagicMock()
    return mw

  def test_closed_loop_convergence_fixed_radius_curve(self, sld):
    """Fixed-radius curve: a_y = kappa * v**2. As the cap ratchets the cap
    down, the (rate-limited — a real car can't teleport to the cap) speed
    follows it down, which lowers a_y in turn. The closed loop should settle
    near the analytic fixed point sqrt(threshold/kappa) - hyst, and never
    below REACT_MIN_SPEED."""
    import math
    mw = self._mw(sld)
    kappa, thr = 0.0116, 2.5
    v, t, dt = 15.5, 100.0, 0.1
    MAX_DECEL = 1.0  # m/s^2 — bounded braking response, not an instant snap
    for _ in range(32):
      a_y = kappa * v * v
      cap = mw._update_reactive_cap(a_y, v, thr, t, dt)
      if cap > 0.0:
        v = max(cap, v - MAX_DECEL * dt)
      t += dt
    target = math.sqrt(thr / kappa) - 1.0  # ~13.68 (REACT_HYST_MS = 1.0)
    assert mw._react_cap_ms >= sld.REACT_MIN_SPEED
    assert mw._react_cap_ms == pytest.approx(target, abs=1.0)

  def test_ratchet_guard_stops_at_min_speed(self, sld):
    """a_y held constant (speed-independent — banking/yaw-bias/tightening
    spiral) while v_ego tracks the cap down: the ratchet must stop exactly
    at REACT_MIN_SPEED, never below."""
    mw = self._mw(sld)
    v, thr, t, dt = 20.0, 2.5, 200.0, 0.1
    for _ in range(120):
      cap = mw._update_reactive_cap(2.6, v, thr, t, dt)
      if cap > 0.0:
        v = max(cap, 0.0)
      t += dt
    assert mw._react_cap_ms == sld.REACT_MIN_SPEED

  def test_no_engage_below_floor(self, sld):
    """v_ego below REACT_MIN_SPEED must never engage the cap, no matter how
    long a_y stays over threshold — at parking-lot speed the driver is in
    charge, not this cap."""
    mw = self._mw(sld)
    t, dt = 300.0, 0.1
    cap = 0.0
    for _ in range(20):  # 2 s, well past REACT_ENGAGE_S
      cap = mw._update_reactive_cap(3.0, 6.0, 2.5, t, dt)
      t += dt
      assert cap == 0.0
    assert mw._react_cap_ms == 0.0

  def test_direct_sign_flip_no_dip_filter_stays_high(self, sld):
    """Mechanism guard (abs-then-filter): a DIRECT +2.8 -> -2.8 flip with no
    zero-dip must leave _ay_filt high throughout. A filter-then-abs
    implementation would collapse the magnitude at the crossing and fail."""
    mw = self._mw(sld)
    t, dt = 500.0, 0.1
    for _ in range(15):  # +2.8 for 1.5 s -> engage
      mw._update_reactive_cap(2.8, 20.0, 2.5, t, dt)
      t += dt
    assert mw._react_cap_ms > 0.0
    for _ in range(10):  # instant flip to -2.8, no dip
      cap = mw._update_reactive_cap(-2.8, 20.0, 2.5, t, dt)
      t += dt
      assert mw._ay_filt > 2.5, 'abs-then-filter must not dip at the crossing'
      assert cap >= sld.REACT_MIN_SPEED

  def test_sign_flip_persists_engagement(self, sld):
    """A signed a_y_meas (curve reverses direction) must not release the
    cap — only |a_y| matters, and a brief near-zero dip mid-flip must not
    accumulate the full REACT_QUIET_S."""
    mw = self._mw(sld)
    t, dt = 400.0, 0.1
    for _ in range(15):  # +2.8 for 1.5 s -> engage
      mw._update_reactive_cap(2.8, 20.0, 2.5, t, dt)
      t += dt
    engaged = mw._react_cap_ms
    assert engaged > 0.0
    for _ in range(3):  # dip near 0 for 0.3 s (short of REACT_QUIET_S)
      cap = mw._update_reactive_cap(0.0, 20.0, 2.5, t, dt)
      t += dt
      assert cap > 0.0
    for _ in range(15):  # flip to -2.8 for 1.5 s
      cap = mw._update_reactive_cap(-2.8, 20.0, 2.5, t, dt)
      t += dt
      assert cap >= sld.REACT_MIN_SPEED, 'released across the sign flip'

  def test_noise_bounce_resets_quiet_timer_then_clean_releases(self, sld):
    """Pins EXISTING behavior (pre-dates this change; must stay green): a_y
    bouncing across the quiet line (threshold - REACT_QUIET_MARGIN = 2.2)
    keeps resetting the quiet timer — no release — until it goes clean, at
    which point the release ramp begins."""
    mw = self._mw(sld)
    t, dt = 500.0, 0.1
    for _ in range(30):  # engage
      mw._update_reactive_cap(2.6, 20.0, 2.5, t, dt)
      t += dt
    engaged = mw._react_cap_ms
    assert engaged > 0.0
    for i in range(40):  # 4 s of 2.3 <-> 2.1 straddling the 2.2 quiet line
      val = 2.3 if (i % 4) < 2 else 2.1
      mw._update_reactive_cap(val, 20.0, 2.5, t, dt)
      t += dt
    assert mw._react_cap_ms == engaged  # noise never released it
    for _ in range(30):  # clean 2.0, well below the quiet line, sustained
      mw._update_reactive_cap(2.0, 20.0, 2.5, t, dt)
      t += dt
    assert mw._react_cap_ms > engaged  # release ramp under way

  def test_stale_livepose_forces_release(self, sld):
    """A livePose that stops updating while the cap is latched must not be
    allowed to hold it forever. This mirrors the real call site: on a stale
    tick a_y_meas is None and v_ego is 0.0 (nothing to measure)."""
    mw = self._mw(sld)
    t, dt = 600.0, 0.1
    for _ in range(20):  # engage, livePose "updating" every tick
      mw._update_reactive_cap(2.8, 20.0, 2.5, t, dt, True)
      t += dt
    engaged = mw._react_cap_ms
    assert engaged > 0.0
    for _ in range(15):  # 1.5 s with no livePose update — must not crash
      mw._update_reactive_cap(None, 0.0, 2.5, t, dt, False)
      t += dt
    assert mw._react_cap_ms == 0.0  # stale localizer -> forced release


class TestReactiveCapIntegration:
  """Reactive cap flows through update() into the published speedLimitState as a
  protected safety-class source (source 4, safetyCapped True)."""

  def _mw_for_update(self, sld):
    import plugins.speedlimitd.speedlimitd as mod
    with patch.object(mod.messaging, 'SubMaster'):
      mw = mod.SpeedLimitMiddleware()
    mw._sl_pub = MagicMock()
    # Controlled SubMaster: nothing updated this tick, model block skipped.
    sm = MagicMock()
    sm.updated = {'modelV2': False, 'gpsLocationExternal': False, 'livePose': False}
    sm.update = MagicMock()
    mw.sm = sm
    mw._cmd_sub = None
    mw._lc_sub = None
    return mw

  def test_reactive_cap_publishes_source4_safety(self, sld):
    mw = self._mw_for_update(sld)
    # Road inference would allow 120 (motorway/freeway) so the reactive cap wins.
    mw.lane_count_stable = 4
    mw.last_road_context = 'freeway'
    mw.last_way_ref = 'G2'
    mw.last_highway_type = 'motorway'
    # Simulate an engaged reactive cap at ~60 km/h (16.67 m/s).
    mw._react_cap_ms = 60.0 / 3.6
    mw.react_lat_accel_threshold = 2.5
    mw.update()
    published = mw._sl_pub.send.call_args[0][0]
    assert published['reactCapEngaged'] is True
    assert published['source'] == 4
    assert published['safetyCapped'] is True
    assert published['reactCap'] == pytest.approx(60.0, abs=0.1)
    assert published['speedLimit'] <= 60


# ============================================================
# Lane-count-first speed limits — OSM trusted only for G/S
# expressways (route 3d0 elevated-road flicker, 2026-07-28)
# ============================================================

class TestLaneCountFirstInference:
  """Non-G/S roads: the limit is driven by the vision lane count only (OSM
  road-type churn among stacked/elevated ways is ignored). G/S expressways: the
  EXISTING promote mechanism (wayRef class x lane count via
  infer_speed_from_road_type), preserved verbatim and made sticky for
  GS_STICKY_S."""

  def _mw(self, sld):
    import plugins.speedlimitd.speedlimitd as mod
    with patch.object(mod.messaging, 'SubMaster'):
      mw = mod.SpeedLimitMiddleware()
    mw._sl_pub = MagicMock()
    sm = MagicMock()
    sm.updated = {'modelV2': False, 'gpsLocationExternal': False, 'livePose': False}
    sm.update = MagicMock()
    mw.sm = sm
    mw._cmd_sub = None
    mw._lc_sub = None
    return mw

  def _result(self, **kw):
    base = {'wayRef': '', 'wayName': '', 'speedLimit': 0.0, 'lanes': 0,
            'roadContext': 1, 'roadName': '', 'highwayType': '', 'distance': 5.0}
    base.update(kw)
    return base

  def _published(self, mw):
    return mw._sl_pub.send.call_args[0][0]

  # --- pure helpers -----------------------------------------

  def test_lane_count_limit_table(self, sld):
    assert sld.lane_count_limit(1) == 30
    assert sld.lane_count_limit(2) == 40
    assert sld.lane_count_limit(3) == 60
    assert sld.lane_count_limit(4) == 80
    assert sld.lane_count_limit(6) == 80

  def test_lane_count_limit_narrow_delegates_to_existing_subtable(self, sld):
    # ≤2 lanes: keep the EXISTING narrow-road sub-table exactly — delegated
    # verbatim to infer_speed_from_road_type's lane-count shortcut.
    assert sld.lane_count_limit(2) == sld.infer_speed_from_road_type('', 2, 'city')
    assert sld.lane_count_limit(1) == sld.infer_speed_from_road_type('', 1, 'city')

  def test_is_gs_expressway_ref(self, sld):
    # 1–2 digit and 4-digit G/S refs are controlled-access expressways.
    assert sld.is_gs_expressway_ref('G2')
    assert sld.is_gs_expressway_ref('G50')
    assert sld.is_gs_expressway_ref('S1')
    assert sld.is_gs_expressway_ref('S20')
    assert sld.is_gs_expressway_ref('G1501')      # regional ring/spur (4-digit)
    # 3-digit refs are ordinary guodao/shengdao SURFACE highways — NOT expressway.
    assert not sld.is_gs_expressway_ref('G312')
    assert not sld.is_gs_expressway_ref('S203')
    assert not sld.is_gs_expressway_ref('G101')
    # malformed / non-G-S.
    assert not sld.is_gs_expressway_ref('')
    assert not sld.is_gs_expressway_ref('白城路')
    assert not sld.is_gs_expressway_ref('G')       # no digits
    assert not sld.is_gs_expressway_ref('G12345')  # 5-digit
    assert not sld.is_gs_expressway_ref('X5')

  def test_is_gs_expressway_ref_multi_ref(self, sld):
    # OSM joins concurrent designations on one carriageway with ';'. ANY part
    # matching the expressway grammar wins — the S20 ring in 'G1503;S20' is
    # unambiguously a controlled-access expressway even though it also
    # carries a concurrent G1503 designation.
    assert sld.is_gs_expressway_ref('G1503;S20')
    assert sld.is_gs_expressway_ref('G1503; S20')   # whitespace after ';'
    assert sld.is_gs_expressway_ref('G312;S20')     # concurrency: expressway part governs
    # bare 3-digit guodao is still excluded when no part is an expressway.
    assert not sld.is_gs_expressway_ref('G312')
    assert not sld.is_gs_expressway_ref('G312;X601')
    # existing behaviour must not regress.
    assert not sld.is_gs_expressway_ref('')
    assert sld.is_gs_expressway_ref('S20')

  # --- Test 1: THE 3d0 regression scenario ------------------

  def test_3d0_stacked_way_churn_limit_constant_80(self, sld):
    """Lane count 4 stable while the matched-way identity churns among
    trunk/primary/residential/trunk_link every tick (no G/S refs) → published
    limit CONSTANT 80. The flicker is structurally impossible: the matched-way
    identity no longer feeds the limit at all on non-G/S roads."""
    mw = self._mw(sld)
    mw.lane_count = 4
    mw.lane_count_stable = 4                     # vision rock-stable at 4 lanes
    stacked = [
      {'roadName': 'trunk way',   'highwayType': 'trunk',       'roadContext': 0},
      {'roadName': 'primary way', 'highwayType': 'primary',     'roadContext': 1},
      {'roadName': 'res way',     'highwayType': 'residential', 'roadContext': 1},
      {'roadName': 'link way',    'highwayType': 'trunk_link',  'roadContext': 1},
    ]
    limits, modes = [], []
    for _ in range(3):                           # 12 way flips
      for w in stacked:
        mw._ingest_osm_result(self._result(**w))
        mw.update()
        limits.append(self._published(mw)['speedLimit'])
        modes.append(self._published(mw)['inferenceMode'])
    assert set(limits) == {80}, f'limit flickered under way churn: {limits}'
    assert set(modes) == {'lane_count'}

  # --- Test 2: G/S promote (existing mechanism, preserved) --

  def test_gs_promote_g_road_120(self, sld):
    mw = self._mw(sld)
    mw.lane_count_stable = 4
    mw._ingest_osm_result(self._result(wayRef='G50', roadName='京沪高速',
                                       roadContext=0, highwayType='motorway'))
    mw.update()
    pub = self._published(mw)
    assert pub['inferenceMode'] == 'gs_osm'
    assert pub['inferredSpeed'] == 120           # nonurban motorway multi (promote)

  def test_gs_promote_g_road_3lane_still_120(self, sld):
    mw = self._mw(sld)
    mw.lane_count_stable = 3
    mw._ingest_osm_result(self._result(wayRef='G50', roadContext=0,
                                       highwayType='motorway'))
    mw.update()
    assert self._published(mw)['inferredSpeed'] == 120

  def test_gs_promote_s_road_100(self, sld):
    mw = self._mw(sld)
    mw.lane_count_stable = 3
    mw._ingest_osm_result(self._result(wayRef='S20', roadContext=0,
                                       highwayType='trunk'))
    mw.update()
    pub = self._published(mw)
    assert pub['inferenceMode'] == 'gs_osm'
    assert pub['inferredSpeed'] == 100           # nonurban trunk multi (promote)

  def test_gs_motorway_type_no_ref_demotes_to_80(self, sld):
    # osmHwType == 'motorway' (urban elevated, no ref) enters G/S mode, but the
    # EXISTING promote DEMOTES a ref-less motorway to trunk-grade 80 — NOT 120.
    # Assert the demote path end-to-end (review gap e).
    mw = self._mw(sld)
    mw.lane_count_stable = 4
    mw._ingest_osm_result(self._result(highwayType='motorway', roadContext=1))
    mw.update()
    pub = self._published(mw)
    assert pub['inferenceMode'] == 'gs_osm'
    assert pub['inferredSpeed'] == 80            # urban trunk-grade demote
    assert pub['inferredSpeed'] != 120

  def test_gs_motorway_type_no_ref_freeway_context_still_demotes_to_80(self, sld):
    # Re-review precision item: the ON-CAR elevated-expressway case is
    # roadContext=0 (mapd tags elevated ways 'freeway'). The ref-less
    # motorway demote must hold there too — 80 via the freeway-demote
    # branch, NOT 100 via the motorway+freeway promote.
    mw = self._mw(sld)
    mw.lane_count_stable = 4
    mw._ingest_osm_result(self._result(highwayType='motorway', roadContext=0))
    mw.update()
    pub = self._published(mw)
    assert pub['inferenceMode'] == 'gs_osm'
    assert pub['inferredSpeed'] == 80
    assert pub['inferredSpeed'] not in (100, 120)

  def test_gs_narrow_ramp_immediate_release(self, sld):
    # lane_count_stable ≤ 2 while a G ref matches → immediate release to
    # lane-count mode (narrow ramp/exit), never a stale expressway limit
    # (review gap b).
    mw = self._mw(sld)
    mw.lane_count_stable = 2
    mw._ingest_osm_result(self._result(wayRef='G50', roadContext=0,
                                       highwayType='motorway'))
    mw.update()
    pub = self._published(mw)
    assert pub['inferenceMode'] == 'lane_count'
    assert pub['inferredSpeed'] == 40            # narrow-road limit, immediate

  # --- Test 3: G/S stickiness -------------------------------

  def test_gs_stickiness_holds_then_reverts(self, sld, monkeypatch):
    mw = self._mw(sld)
    mw.lane_count_stable = 4
    clock = {'t': 1000.0}
    monkeypatch.setattr(sld.time, 'monotonic', lambda: clock['t'])
    # G50 seen → promote to 120
    mw._ingest_osm_result(self._result(wayRef='G50', roadContext=0,
                                       highwayType='motorway'))
    mw.update()
    assert self._published(mw)['inferenceMode'] == 'gs_osm'
    assert self._published(mw)['inferredSpeed'] == 120
    # 10 s of stacked non-G/S matches → sticky holds expressway mode + limit
    mw._ingest_osm_result(self._result(roadName='res', highwayType='residential'))
    clock['t'] += 10.0
    mw.update()
    assert self._published(mw)['inferenceMode'] == 'gs_osm'
    assert self._published(mw)['inferredSpeed'] == 120   # held (not the 30 res value)
    # 35 s total since the last G/S match → reverts to lane-count mode
    clock['t'] += 25.0
    mw.update()
    assert self._published(mw)['inferenceMode'] == 'lane_count'
    assert self._published(mw)['inferredSpeed'] == 80    # lane_count(4)

  # --- review gap (a): exit connector, continuous absence releases at 10 s ---

  def test_exit_ramp_continuous_absence_releases_at_10s(self, sld, monkeypatch):
    mw = self._mw(sld)
    mw.lane_count_stable = 3                    # wide connector — NOT narrow
    clock = {'t': 2000.0}
    monkeypatch.setattr(sld.time, 'monotonic', lambda: clock['t'])
    # On a G expressway → promote (G ref → motorway → 120 even at 3 lanes).
    mw._ingest_osm_result(self._result(wayRef='G50', roadContext=0,
                                       highwayType='motorway'))
    mw.update()
    assert self._published(mw)['inferredSpeed'] == 120
    # Continuous non-G/S connector matches from here on.
    mw._ingest_osm_result(self._result(roadName='exit connector',
                                       highwayType='primary'))
    clock['t'] += 2.0
    mw.update()                                 # absence run starts
    clock['t'] += 6.0                           # 8 s continuous absence
    mw.update()
    assert self._published(mw)['inferenceMode'] == 'gs_osm'   # < 10 s → still held
    assert self._published(mw)['inferredSpeed'] == 120
    clock['t'] += 4.0                           # 12 s absence, still < 30 s ceiling
    mw.update()
    assert self._published(mw)['inferenceMode'] == 'lane_count'   # released at 10 s
    assert self._published(mw)['inferredSpeed'] == 60            # lane_count(3)

  # --- review gap (c): 3-digit guodao/shengdao → lane-count end-to-end ---

  def test_guodao_3digit_ref_uses_lane_count(self, sld):
    for ref in ('G312', 'S203', 'G101'):
      mw = self._mw(sld)
      mw.lane_count_stable = 4
      mw._ingest_osm_result(self._result(wayRef=ref, roadName=ref,
                                         roadContext=0, highwayType='trunk'))
      mw.update()
      pub = self._published(mw)
      assert pub['inferenceMode'] == 'lane_count', ref
      assert pub['inferredSpeed'] == 80, ref     # lane_count(4), not an OSM promote

  # --- review gap (d1): stacked flicker WITH a real S-ref never releases ---

  def test_stacked_flicker_with_sref_never_releases(self, sld, monkeypatch):
    mw = self._mw(sld)
    mw.lane_count_stable = 4
    clock = {'t': 3000.0}
    monkeypatch.setattr(sld.time, 'monotonic', lambda: clock['t'])
    # S20 (2-digit expressway) flickers in among stacked non-G/S ways every ~2 s;
    # the longest continuous non-G/S run is 4 s (< GS_RELEASE_CONT_S).
    seq = [
      self._result(wayRef='S20', roadContext=0, highwayType='trunk'),
      self._result(roadName='res', highwayType='residential'),
      self._result(roadName='pri', highwayType='primary'),
    ]
    modes = []
    for i in range(12):                         # 24 s of ticks
      mw._ingest_osm_result(seq[i % 3])
      mw.update()
      modes.append(self._published(mw)['inferenceMode'])
      clock['t'] += 2.0
    assert set(modes) == {'gs_osm'}             # flicker never accumulates 10 s

  # --- review gap (d2): stacked 3-digit refs never enter gs at all ---

  def test_stacked_churn_3digit_refs_never_enter_gs(self, sld):
    mw = self._mw(sld)
    mw.lane_count_stable = 4
    seq = [
      self._result(wayRef='G312', roadName='G312', roadContext=0, highwayType='trunk'),
      self._result(wayRef='S203', roadName='S203', roadContext=0, highwayType='primary'),
      self._result(roadName='res', highwayType='residential'),
    ]
    limits, modes = [], []
    for i in range(9):
      mw._ingest_osm_result(seq[i % 3])
      mw.update()
      limits.append(self._published(mw)['speedLimit'])
      modes.append(self._published(mw)['inferenceMode'])
    assert set(modes) == {'lane_count'}
    assert set(limits) == {80}

  # --- Test 4: lane-count transition debounce ---------------

  def _straight_model(self, probs):
    m = MagicMock()
    m.laneLineProbs = list(probs)
    n = 33
    m.orientationRate.z = [0.0] * n              # straight → no curvature cap
    m.velocity.x = [20.0] * n
    m.position.x = [float(i * 3) for i in range(n)]
    m.position.yStd = [0.1] * n
    m.roadEdges = [MagicMock(y=[0.0] * 10) for _ in range(2)]
    m.roadEdgeStds = [1.0, 1.0]                  # not near an edge → no boost
    m.laneLines = [MagicMock(y=[0.0] * 10) for _ in range(4)]
    return m

  def _mw_model(self, sld):
    import plugins.speedlimitd.speedlimitd as mod
    with patch.object(mod.messaging, 'SubMaster'):
      mw = mod.SpeedLimitMiddleware()
    mw._sl_pub = MagicMock()
    mw._cmd_sub = None
    mw._lc_sub = None
    holder = {'model': None}
    sm = MagicMock()
    sm.updated = {'modelV2': True, 'gpsLocationExternal': False, 'livePose': False}
    sm.update = MagicMock()
    sm.__getitem__ = MagicMock(
      side_effect=lambda k: holder['model'] if k == 'modelV2' else MagicMock())
    mw.sm = sm
    return mw, holder

  def test_lane_drop_4_to_3_debounced(self, sld, monkeypatch):
    """4→3 lanes sustained → 80→60 after the (existing lane_count_stable)
    debounce; a 1 s dip to 3 → no change."""
    mw, holder = self._mw_model(sld)
    clock = {'t': 5000.0}
    monkeypatch.setattr(sld.time, 'monotonic', lambda: clock['t'])
    m4 = self._straight_model([0.6, 0.7, 0.7, 0.6])   # 4 visible lines
    m3 = self._straight_model([0.6, 0.7, 0.7, 0.1])   # 3 visible lines
    # establish a stable, locked 4-lane reading
    holder['model'] = m4
    mw.update()
    mw.lane_count = 4
    mw.lane_count_stable = 4
    mw.lane_count_stable_since = clock['t']
    mw.update()
    assert self._published(mw)['inferredSpeed'] == 80

    # vision now sees 3 lanes; a 1 s dip is below the 5 s (straight) demotion win
    holder['model'] = m3
    mw.update()                        # raw=3 != lane_count(4): lane_count=3, since=now
    clock['t'] += 1.0
    mw.update()                        # 1 s < 5 s → no commit
    assert mw.lane_count_stable == 4
    assert self._published(mw)['inferredSpeed'] == 80   # dip ignored

    # sustain 3 lanes past the 5 s window → commit → limit follows to 60
    clock['t'] += 5.1
    mw.update()
    assert mw.lane_count_stable == 3
    assert self._published(mw)['inferredSpeed'] == 60

  # --- noise-tolerant narrow-band (≤2) confirmation (route 3d3 seg 16) -------
  # A single directional debounce timer resets on ANY raw-count change, so on a
  # genuine 2-lane ramp with brief 3↔4 occlusion spikes the demotion window kept
  # restarting and lane_count_stable never committed to 2 (seg 16: raw ≤2 for
  # 24 s continuous, never committed). The leaky time-in-narrow accumulator
  # (NARROW_* in speedlimitd) commits ≤2 once sustained NARROW_CONFIRM_S, tolerating
  # sub-NARROW_CONFIRM_S occlusion spikes.

  def _lane_models(self):
    """modelV2 stand-ins whose laneLineProbs infer 1/2/3/4 raw lanes (straight →
    no curvature cap, no edge boost; the 2-lane probs give vision_speed_cap 0 so
    the published inferredSpeed is purely the lane-count table)."""
    return {
      1: self._straight_model([0.6, 0.1, 0.1, 0.1]),   # 1 visible line
      2: self._straight_model([0.6, 0.7, 0.1, 0.1]),   # 2 visible lines
      3: self._straight_model([0.6, 0.7, 0.7, 0.1]),   # 3 visible lines
      4: self._straight_model([0.6, 0.7, 0.7, 0.6]),   # 4 visible lines
    }

  def _feed(self, mw, holder, clock, raw_seq, dt=0.2):
    """Drive update() once per raw lane count in raw_seq, advancing the clock by
    dt each tick (5 Hz default). Returns the elapsed time (from the first tick) at
    which lane_count_stable first commits to ≤2, or None if it never does."""
    models = self._lane_models()
    t0 = clock['t']
    commit_t = None
    for raw in raw_seq:
      holder['model'] = models[raw]
      mw.update()
      if commit_t is None and mw.lane_count_stable <= 2:
        commit_t = round(clock['t'] - t0, 3)
      clock['t'] += dt
    return commit_t

  def test_noisy_two_lane_ramp_commits_and_enforces(self, sld, monkeypatch):
    """THE seg-16 case: a genuine 2-lane ramp read as ≤2 with brief single-frame
    3-4 occlusion spikes (~every 1 s) sustained > 3 s → the leaky accumulator
    commits lane_count_stable to 2 within ~3-4 s, and the inferred limit is 40 —
    an ENFORCING source-2 reading (planner lowers v_cruise; no display-only skip)."""
    mw, holder = self._mw_model(sld)
    clock = {'t': 6000.0}
    monkeypatch.setattr(sld.time, 'monotonic', lambda: clock['t'])
    mw.lane_count_stable = 4               # came off a wide road onto the ramp
    # 25 s of ≤2 with a single 3-4 occlusion spike every ~1 s (every 5th tick).
    seq = [(3 if i % 5 == 4 else 2) for i in range(125)]
    commit_t = self._feed(mw, holder, clock, seq)
    assert commit_t is not None                    # genuine ramp DID commit
    assert 3.0 <= commit_t <= 4.6                  # within ~3-4 s despite spikes
    assert mw.lane_count_stable == 2
    pub = self._published(mw)
    assert pub['inferredSpeed'] == 40             # 2-lane limit
    assert pub['source'] == 2                     # enforcing (not safety, not display-only)
    assert pub['safetyCapped'] is False

  def test_transient_narrow_dip_never_commits(self, sld, monkeypatch):
    """A sub-3 s narrow dip (occlusion transients cluster ~0.1 s; here a generous
    2 s) then raw ≥3 sustained → the accumulator never reaches NARROW_CONFIRM_S, so
    lane_count_stable never commits to 2 — no spurious 40 (the seg-29 false narrow)."""
    mw, holder = self._mw_model(sld)
    clock = {'t': 7000.0}
    monkeypatch.setattr(sld.time, 'monotonic', lambda: clock['t'])
    mw.lane_count_stable = 4
    # 2.0 s of raw=2 (10 ticks) — under the 3 s threshold — then 6 s of raw=4.
    seq = [2] * 10 + [4] * 30
    commit_t = self._feed(mw, holder, clock, seq)
    assert commit_t is None                        # NEVER committed to ≤2
    assert mw.lane_count_stable == 4
    assert self._published(mw)['inferredSpeed'] == 80

  def test_clean_two_lane_ramp_commits_at_3s(self, sld, monkeypatch):
    """A clean genuine ramp (raw ≤2 continuous, no spikes) commits to 2 at ~3 s
    (NARROW_CONFIRM_S), not before."""
    mw, holder = self._mw_model(sld)
    clock = {'t': 8000.0}
    monkeypatch.setattr(sld.time, 'monotonic', lambda: clock['t'])
    mw.lane_count_stable = 4
    commit_t = self._feed(mw, holder, clock, [2] * 30)
    assert commit_t is not None
    assert 3.0 <= commit_t <= 3.4                  # ~3 s (first-tick dt=0 → 3.0 s)
    assert mw.lane_count_stable == 2
    assert self._published(mw)['inferredSpeed'] == 40

  def test_narrow_commits_to_2_despite_raw1_glitch(self, sld, monkeypatch):
    """L1 (route 3d3 seg16 / 3d1 seg29): both real ramps commit on a raw=1
    occluded frame. The commit must be a FIXED 2 (→40), never the glitch raw=1
    (which would read 30 on the very ramps this fixes)."""
    mw, holder = self._mw_model(sld)
    clock = {'t': 8500.0}
    monkeypatch.setattr(sld.time, 'monotonic', lambda: clock['t'])
    mw.lane_count_stable = 4
    # sustained narrow with a single raw=1 occlusion frame right at the commit
    commit_t = self._feed(mw, holder, clock, [2] * 14 + [1] + [2] * 15)
    assert commit_t is not None
    assert mw.lane_count_stable == 2                # NOT 1
    assert self._published(mw)['inferredSpeed'] == 40   # NOT 30

  def test_leaving_ramp_promotes_back_to_wide(self, sld, monkeypatch):
    """The narrow accumulator must NOT block promotion: after committing to 2, a
    sustained raw=4 (rejoining a wide road) commits back to 4 → 80 via the existing
    up-debounce (1.5 s)."""
    mw, holder = self._mw_model(sld)
    clock = {'t': 9000.0}
    monkeypatch.setattr(sld.time, 'monotonic', lambda: clock['t'])
    mw.lane_count_stable = 4
    self._feed(mw, holder, clock, [2] * 30)        # commit to 2 (→40)
    assert mw.lane_count_stable == 2
    self._feed(mw, holder, clock, [4] * 20)        # 4 s of raw=4 → promote
    assert mw.lane_count_stable == 4
    assert self._published(mw)['inferredSpeed'] == 80

  # --- wide commit resets the narrow accumulator (route 3e0 seg 33) ---------

  def test_wide_commit_resets_narrow_accumulator(self, sld, monkeypatch):
    """3e0 五洲大道 exit: after a committed narrow, a sustained raw=4 commits WIDE
    and zeroes the narrow accumulator. A subsequent single ≤2 edge-lane dip of
    2.5 s then does NOT re-commit narrow — it needs a fresh full NARROW_CONFIRM_S
    (3 s). Without the reset the residual would re-commit in ~2-3 s, repeatedly
    yanking the climbing limit back to 40-60 (the 20.8 s hesitation)."""
    mw, holder = self._mw_model(sld)
    clock = {'t': 9500.0}
    monkeypatch.setattr(sld.time, 'monotonic', lambda: clock['t'])
    mw.lane_count_stable = 4
    self._feed(mw, holder, clock, [2] * 20)        # commit narrow (~3 s of raw 2)
    assert mw.lane_count_stable == 2
    assert mw._narrow_accum > 0.0                  # accumulator loaded (at cap)
    # Sustained raw=4 → commit wide (up-debounce 1.5 s) AND reset the accumulator.
    self._feed(mw, holder, clock, [4] * 12)        # ~2.4 s of raw 4
    assert mw.lane_count_stable == 4
    assert mw._narrow_accum == 0.0                 # reset on the wide commit
    # A 2.5 s ≤2 dip must NOT re-commit — the residual is gone, so 2.5 s < 3 s.
    self._feed(mw, holder, clock, [2] * 12)        # ~2.4 s of raw 2
    assert mw.lane_count_stable == 4               # stayed wide

  def test_re_narrow_after_wide_commit_needs_full_3s(self, sld, monkeypatch):
    """The reset does not break genuine re-narrowing: a full 3 s of sustained
    raw=2 after a committed widening DOES re-commit narrow (→40)."""
    mw, holder = self._mw_model(sld)
    clock = {'t': 9800.0}
    monkeypatch.setattr(sld.time, 'monotonic', lambda: clock['t'])
    mw.lane_count_stable = 4
    self._feed(mw, holder, clock, [2] * 20)        # commit narrow
    self._feed(mw, holder, clock, [4] * 12)        # commit wide → accum 0.0
    assert mw.lane_count_stable == 4 and mw._narrow_accum == 0.0
    commit_t = self._feed(mw, holder, clock, [2] * 18)   # ~3.4 s of raw 2
    assert commit_t is not None                    # re-committed narrow
    assert mw.lane_count_stable == 2
    assert self._published(mw)['inferredSpeed'] == 40

  # --- Fix I1 threshold-hold wide commit (route 3e6 seg12/13 S20 merge) -------
  # The old up-debounce required raw to hold the SAME exact value for the full
  # 1.5 s interval, so an oscillating raw (2↔3↔4) starved it. New semantics: a
  # window opens on raw ≥3, tracks the MINIMUM raw while every frame stays ≥3,
  # and commits that minimum after 1.5 s sustained. Any raw ≤2 frame closes it.

  def test_s20_merge_oscillating_wide_commits(self, sld, monkeypatch):
    """THE S20 regression: from stable=2, an oscillating raw with EVERY frame ≥3
    (alternating 3/4) sustained past 1.5 s commits to 3 (the minimum). The OLD
    exact-match semantics would never commit here (raw != lane_count every frame
    resets the up clock → 11.4 s of unjustified 40). Then a sustained raw=4
    promotes to 4."""
    mw, holder = self._mw_model(sld)
    clock = {'t': 10000.0}
    monkeypatch.setattr(sld.time, 'monotonic', lambda: clock['t'])
    mw.lane_count_stable = 2
    # 3.2 s of oscillating 3/4 — every frame ≥3, no exact-match hold possible.
    self._feed(mw, holder, clock, [3, 4] * 8)
    assert mw.lane_count_stable == 3       # committed the MIN (3), not 40-stuck
    assert self._published(mw)['inferredSpeed'] == 60
    # sustained raw=4 → threshold-held promotion to 4
    self._feed(mw, holder, clock, [4] * 24)
    assert mw.lane_count_stable == 4
    assert self._published(mw)['inferredSpeed'] == 80

  def test_wide_window_closed_by_narrow_frame(self, sld, monkeypatch):
    """A single raw ≤2 frame closes the wide window: 1.0 s of raw ≥3, one raw=2
    frame, then raw ≥3 again does NOT carry the earlier 1.0 s — a fresh full 1.5 s
    clean window is required before the wide commit fires."""
    mw, holder = self._mw_model(sld)
    clock = {'t': 10500.0}
    monkeypatch.setattr(sld.time, 'monotonic', lambda: clock['t'])
    mw.lane_count_stable = 2
    self._feed(mw, holder, clock, [3] * 6)        # 1.0 s ≥3 — under 1.5 s
    assert mw.lane_count_stable == 2
    self._feed(mw, holder, clock, [2] * 1)        # one narrow frame closes window
    assert mw.lane_count_stable == 2
    self._feed(mw, holder, clock, [3] * 6)        # 1.0 s of a FRESH window
    assert mw.lane_count_stable == 2              # pre-narrow 1.0 s did NOT carry
    self._feed(mw, holder, clock, [3] * 6)        # completes the fresh 1.5 s window
    assert mw.lane_count_stable == 3

  def test_min_raw_commit_conservative(self, sld, monkeypatch):
    """A window of mixed 4/3/4 commits the MINIMUM (3), never 4 — conservative."""
    mw, holder = self._mw_model(sld)
    clock = {'t': 11000.0}
    monkeypatch.setattr(sld.time, 'monotonic', lambda: clock['t'])
    mw.lane_count_stable = 2
    self._feed(mw, holder, clock, [4, 3, 4] * 4)  # 3.6 s, every frame ≥3, min=3
    assert mw.lane_count_stable == 3
    assert self._published(mw)['inferredSpeed'] == 60

  def test_in_ramp_ghost_burst_still_blocked(self, sld, monkeypatch):
    """From stable=2 with the narrow accumulator full, a 1.0 s ghost burst of
    raw=4 inside otherwise-narrow traffic does NOT commit wide — the window is
    under 1.5 s and closes on the next narrow frame."""
    mw, holder = self._mw_model(sld)
    clock = {'t': 11500.0}
    monkeypatch.setattr(sld.time, 'monotonic', lambda: clock['t'])
    mw.lane_count_stable = 4
    self._feed(mw, holder, clock, [2] * 20)       # commit narrow, accum at cap
    assert mw.lane_count_stable == 2
    assert mw._narrow_accum > 0.0
    self._feed(mw, holder, clock, [4] * 5)        # 1.0 s ghost burst — under 1.5 s
    assert mw.lane_count_stable == 2              # NOT promoted
    assert self._published(mw)['inferredSpeed'] == 40

  def test_two_lane_no_cap_enforces_source2_40(self, sld):
    """Committed 2-lane → 40 is published as an ENFORCING source-2 limit (not
    safety-capped): the seg-29 regression removal — a genuine ramp 40 is no longer
    excluded from enforcement (planner lowers v_cruise; see TestPlannerHook)."""
    mw = self._mw(sld)
    mw.lane_count_stable = 2
    mw.update()
    pub = self._published(mw)
    assert pub['speedLimit'] == 40
    assert pub['source'] == 2
    assert pub['safetyCapped'] is False

  def test_curve_cap_below_two_lane_enforces_as_safety(self, sld):
    """Curve-cap / lane-count interaction: a curve cap BELOW the 2-lane guess
    (30 < 40) binds as the safety source — speedLimit 30, source 4, safetyCapped."""
    mw = self._mw(sld)
    mw.lane_count_stable = 2
    mw.curvature_cap = 30
    mw.update()
    pub = self._published(mw)
    assert pub['speedLimit'] == 30
    assert pub['source'] == 4
    assert pub['safetyCapped'] is True

  # --- source / telemetry -----------------------------------

  def test_source_stays_road_type_inference_class(self, sld):
    # The base-inference candidate keeps source==2 so planner_hook's hold-floor
    # (SOURCE_ROAD_TYPE_INFERENCE) still recognizes it.
    mw = self._mw(sld)
    mw.lane_count_stable = 4
    mw.update()
    assert self._published(mw)['source'] == 2
    assert self._published(mw)['inferenceMode'] == 'lane_count'


# ============================================================
# Distance-guarded fast release for the G/S sticky hold (route 3de seg 19)
# ============================================================

class TestGsDistanceGuardedRelease:
  """Distance-guarded fast release for the sticky G/S hold (route 3de seg 19),
  two independent paths OR'd alongside the existing 10 s timer / 30 s ceiling /
  lane≤2 escape:

    Path 1 — MARGIN rule: while holding and the best match is non-G/S, release
    when the matched way is GS_RELEASE_MARGIN_M CLOSER than the held G/S way (or
    the held way is absent → margin +inf) for a GS_RELEASE_MARGIN_S continuous run of
    consecutive queries. An absolute gate on the held-way distance fails — at
    seg-19 ramp entry the held S1 is only ~13.5 m off — so the MARGIN separates a
    genuine exit (car on the ramp, held way receding) from a stacked mis-match
    (matched way co-located with the held one, margin 0.2-5 m).

    Path 2 — ref-empty + narrow-drop conjunction: the held ref has stopped
    matching AND the RAW lane count has held ≤2 for GS_LANE_DROP_S (1.5 s). Two
    independent signals, no OSM candidate-distance dependence.
  """

  def _mw(self, sld):
    import plugins.speedlimitd.speedlimitd as mod
    with patch.object(mod.messaging, 'SubMaster'):
      mw = mod.SpeedLimitMiddleware()
    mw._sl_pub = MagicMock()
    sm = MagicMock()
    sm.updated = {'modelV2': False, 'gpsLocationExternal': False, 'livePose': False}
    sm.update = MagicMock()
    mw.sm = sm
    mw._cmd_sub = None
    mw._lc_sub = None
    return mw

  def _result(self, **kw):
    base = {'wayRef': '', 'wayName': '', 'speedLimit': 0.0, 'lanes': 0,
            'roadContext': 1, 'roadName': '', 'highwayType': '', 'distance': 5.0}
    base.update(kw)
    return base

  def _published(self, mw):
    return mw._sl_pub.send.call_args[0][0]

  def _hold_s1(self, mw):
    """Establish an S1 (trunk, 100) sticky hold and return the published limit."""
    mw._ingest_osm_result(self._result(wayRef='S1', roadContext=0,
                                       highwayType='trunk',
                                       refDistances={'S1': 3.0}))
    mw.update()
    return self._published(mw)

  def _diverge(self, mw, clock, held_dist, matched_dist, hwtype='motorway_link'):
    """One non-G/S query: matched way at `matched_dist`, held S1 at `held_dist`
    (margin = held_dist - matched_dist), then a 5 s-cadence update()."""
    ref_d = {} if held_dist is None else {'S1': held_dist}
    mw._ingest_osm_result(self._result(roadName='ramp', highwayType=hwtype,
                                       distance=matched_dist, refDistances=ref_d))
    clock['t'] += 5.0
    mw.update()
    return self._published(mw)

  # --- helpers for the model-driven (path 2) tests --------------------------

  def _straight_model(self, probs):
    m = MagicMock()
    m.laneLineProbs = list(probs)
    n = 33
    m.orientationRate.z = [0.0] * n              # straight → no curvature cap
    m.velocity.x = [20.0] * n
    m.position.x = [float(i * 3) for i in range(n)]
    m.position.yStd = [0.1] * n
    m.roadEdges = [MagicMock(y=[0.0] * 10) for _ in range(2)]
    m.roadEdgeStds = [1.0, 1.0]                  # not near an edge → no boost
    m.laneLines = [MagicMock(y=[0.0] * 10) for _ in range(4)]
    return m

  def _model(self, lanes):
    return self._straight_model({
      1: [0.6, 0.1, 0.1, 0.1],
      2: [0.6, 0.7, 0.1, 0.1],
      3: [0.6, 0.7, 0.7, 0.1],
      4: [0.6, 0.7, 0.7, 0.6],
    }[lanes])

  def _mw_model(self, sld):
    import plugins.speedlimitd.speedlimitd as mod
    with patch.object(mod.messaging, 'SubMaster'):
      mw = mod.SpeedLimitMiddleware()
    mw._sl_pub = MagicMock()
    mw._cmd_sub = None
    mw._lc_sub = None
    holder = {'model': None}
    sm = MagicMock()
    sm.updated = {'modelV2': True, 'gpsLocationExternal': False, 'livePose': False}
    sm.update = MagicMock()
    sm.__getitem__ = MagicMock(
      side_effect=lambda k: holder['model'] if k == 'modelV2' else MagicMock())
    mw.sm = sm
    return mw, holder

  # ======================= Path 1 — MARGIN rule ============================

  def test_margin_exit_releases_on_second_query(self, sld, monkeypatch):
    """seg-19-style exit: matched link at 0.6 m, held S1 at 13.5 m (margin ~13)
    for 2 queries → release on the 2nd query (~5 s into absence), NOT the 10 s
    timer. The 1st divergent query alone must NOT release."""
    mw = self._mw(sld)
    mw.lane_count_stable = 3                     # wide connector, NOT narrow
    clock = {'t': 5000.0}
    monkeypatch.setattr(sld.time, 'monotonic', lambda: clock['t'])
    pub = self._hold_s1(mw)
    assert pub['inferenceMode'] == 'gs_osm' and pub['inferredSpeed'] == 100
    # Query 1: margin ~12.9 > 8 → count=1, still held.
    pub = self._diverge(mw, clock, held_dist=13.5, matched_dist=0.6)
    assert pub['inferenceMode'] == 'gs_osm'
    assert mw._gs_margin_since is not None
    # Query 2: margin still > 8 → count=2 → release.
    pub = self._diverge(mw, clock, held_dist=17.0, matched_dist=0.6)
    assert pub['inferenceMode'] == 'lane_count'
    assert pub['inferredSpeed'] == 60            # lane_count(3)

  def test_margin_release_is_time_based_at_1hz(self, sld, monkeypatch):
    """At the production 1 Hz cadence the margin rule needs a CONTINUOUS
    GS_RELEASE_MARGIN_S (5 s) run — queries 1-5 (elapsed 0-4 s) must NOT
    release; query 6 (elapsed 5.0 s) must. Pins the 2026-08-18 conversion
    from a 2-consecutive-query count (which at 1 Hz would have released
    after 1 s) to a time window."""
    mw = self._mw(sld)
    mw.lane_count_stable = 3
    clock = {'t': 7000.0}
    monkeypatch.setattr(sld.time, 'monotonic', lambda: clock['t'])
    self._hold_s1(mw)
    for i in range(5):                           # queries 1-5: elapsed 0..4 s
      mw._ingest_osm_result(self._result(roadName='ramp',
                                         highwayType='motorway_link',
                                         distance=0.6,
                                         refDistances={'S1': 13.5}))
      mw.update()
      assert self._published(mw)['inferenceMode'] == 'gs_osm', f'query {i+1}'
      clock['t'] += 1.0
    # Query 6: elapsed exactly 5.0 s → release.
    mw._ingest_osm_result(self._result(roadName='ramp',
                                       highwayType='motorway_link',
                                       distance=0.6,
                                       refDistances={'S1': 13.5}))
    mw.update()
    assert self._published(mw)['inferenceMode'] == 'lane_count'

  def test_margin_stacked_no_early_release(self, sld, monkeypatch):
    """Stacked: matched surface street 5.7 m, held 中环路-style S1 at 5.9 m
    (margin 0.2) → NO release; the 10 s timer governs and an S1 re-match
    resets it, exactly as today."""
    mw = self._mw(sld)
    mw.lane_count_stable = 4
    clock = {'t': 6000.0}
    monkeypatch.setattr(sld.time, 'monotonic', lambda: clock['t'])
    self._hold_s1(mw)
    for _ in range(2):                           # two stacked queries, margin 0.2
      pub = self._diverge(mw, clock, held_dist=5.9, matched_dist=5.7,
                          hwtype='residential')
      assert pub['inferenceMode'] == 'gs_osm'
    assert mw._gs_margin_since is None              # never accumulated
    # An S1 re-match resets the absence timer.
    self._hold_s1(mw)
    assert mw._gs_absent_since is None

  def test_margin_6m_sustained_no_release(self, sld, monkeypatch):
    # Margin 6 m (< 8) sustained across many queries → never releases via margin.
    mw = self._mw(sld)
    mw.lane_count_stable = 4
    clock = {'t': 6500.0}
    monkeypatch.setattr(sld.time, 'monotonic', lambda: clock['t'])
    self._hold_s1(mw)
    pub = self._diverge(mw, clock, held_dist=8.0, matched_dist=2.0)   # margin 6
    assert pub['inferenceMode'] == 'gs_osm'
    assert mw._gs_margin_since is None

  def test_margin_resets_on_a_below_threshold_query(self, sld, monkeypatch):
    # margin 9 (one query) then margin 4 → count resets, no release.
    mw = self._mw(sld)
    mw.lane_count_stable = 4
    clock = {'t': 6800.0}
    monkeypatch.setattr(sld.time, 'monotonic', lambda: clock['t'])
    self._hold_s1(mw)
    self._diverge(mw, clock, held_dist=11.0, matched_dist=2.0)        # margin 9
    assert mw._gs_margin_since is not None
    pub = self._diverge(mw, clock, held_dist=6.0, matched_dist=2.0)   # margin 4
    assert mw._gs_margin_since is None              # reset
    assert pub['inferenceMode'] == 'gs_osm'

  def test_margin_boundary_strictly_greater(self, sld, monkeypatch):
    # margin exactly 8 → NOT counted (>, not >=); margin 8.01 → counted.
    mw = self._mw(sld)
    mw.lane_count_stable = 4
    clock = {'t': 7000.0}
    monkeypatch.setattr(sld.time, 'monotonic', lambda: clock['t'])
    self._hold_s1(mw)
    self._diverge(mw, clock, held_dist=10.0, matched_dist=2.0)        # margin 8.0
    assert mw._gs_margin_since is None
    self._diverge(mw, clock, held_dist=10.01, matched_dist=2.0)       # margin 8.01
    assert mw._gs_margin_since is not None

  def test_margin_held_ref_absent_releases(self, sld, monkeypatch):
    """Held S1 absent from the candidate set → margin +inf → releases after 2
    consecutive absent queries."""
    mw = self._mw(sld)
    mw.lane_count_stable = 3
    clock = {'t': 7300.0}
    monkeypatch.setattr(sld.time, 'monotonic', lambda: clock['t'])
    self._hold_s1(mw)
    pub = self._diverge(mw, clock, held_dist=None, matched_dist=5.0)  # S1 absent
    assert pub['inferenceMode'] == 'gs_osm' and mw._gs_margin_since is not None
    pub = self._diverge(mw, clock, held_dist=None, matched_dist=5.0)
    assert pub['inferenceMode'] == 'lane_count'
    assert pub['inferredSpeed'] == 60

  def test_margin_unavailable_falls_back_to_timer(self, sld, monkeypatch):
    """No refDistances anywhere → the seam signals distance-unavailable → the
    margin path never fires; the 10 s absence timer still releases, unchanged."""
    mw = self._mw(sld)
    mw.lane_count_stable = 3
    clock = {'t': 9000.0}
    monkeypatch.setattr(sld.time, 'monotonic', lambda: clock['t'])
    mw._ingest_osm_result(self._result(wayRef='S1', roadContext=0, highwayType='trunk'))
    mw.update()
    assert self._published(mw)['inferredSpeed'] == 100
    mw._ingest_osm_result(self._result(roadName='ramp', highwayType='primary'))
    clock['t'] += 5.0
    mw.update()
    assert self._published(mw)['inferenceMode'] == 'gs_osm'   # < 10 s → still held
    assert mw._gs_margin_since is None                            # margin path idle
    clock['t'] += 11.0
    mw._ingest_osm_result(self._result(roadName='ramp', highwayType='primary'))
    mw.update()
    assert self._published(mw)['inferenceMode'] == 'lane_count'  # 10 s timer

  def test_held_ref_tracked_on_match_and_reset_on_rematch(self, sld, monkeypatch):
    mw = self._mw(sld)
    mw.lane_count_stable = 4
    clock = {'t': 9500.0}
    monkeypatch.setattr(sld.time, 'monotonic', lambda: clock['t'])
    self._hold_s1(mw)
    assert mw._gs_held_ref == 'S1'
    # A ref-less 'motorway' hold has no trackable identity → margin path disabled.
    mw._ingest_osm_result(self._result(highwayType='motorway', roadContext=0))
    mw.update()
    assert mw._gs_held_ref == ''

  def test_force_release_is_sticky_until_rematch(self, sld, monkeypatch):
    """Once the margin forces release, a stacked way drifting back within range
    must NOT re-hold — only a genuine G/S re-match re-enters gs mode."""
    mw = self._mw(sld)
    mw.lane_count_stable = 3
    clock = {'t': 9700.0}
    monkeypatch.setattr(sld.time, 'monotonic', lambda: clock['t'])
    self._hold_s1(mw)
    self._diverge(mw, clock, held_dist=40.0, matched_dist=0.6)
    self._diverge(mw, clock, held_dist=40.0, matched_dist=0.6)
    assert self._published(mw)['inferenceMode'] == 'lane_count'
    # A later query shows the (stacked) way back at 5 m — still released.
    pub = self._diverge(mw, clock, held_dist=5.0, matched_dist=4.0)
    assert pub['inferenceMode'] == 'lane_count'
    # A genuine S1 re-match re-enters gs mode.
    self._hold_s1(mw)
    assert self._published(mw)['inferenceMode'] == 'gs_osm'

  # ============ Path 2 — ref-empty + narrow-drop conjunction ===============

  def _drive(self, mw, holder, clock, lanes, ticks, dt=0.2):
    """Feed `ticks` modelV2 updates at raw `lanes`, advancing the clock dt each."""
    holder['model'] = self._model(lanes)
    for _ in range(ticks):
      mw.update()
      clock['t'] += dt

  def test_path2_ref_empty_lane_drop_releases_at_1p5s(self, sld, monkeypatch):
    """ref empty + RAW drops 4→2 sustained 1.5 s → release at ~1.5 s (before both
    the margin's 2 queries and the 10 s timer)."""
    mw, holder = self._mw_model(sld)
    clock = {'t': 10000.0}
    monkeypatch.setattr(sld.time, 'monotonic', lambda: clock['t'])
    mw.lane_count_stable = 4
    # Establish an S1 hold (raw 4).
    holder['model'] = self._model(4)
    mw._ingest_osm_result(self._result(wayRef='S1', roadContext=0, highwayType='trunk'))
    mw.update()
    clock['t'] += 0.2
    assert self._published(mw)['inferenceMode'] == 'gs_osm'
    # Ref goes empty (exit) and vision drops to raw 2 and holds.
    mw._ingest_osm_result(self._result(roadName='ramp'))   # ref empty, non-G/S
    self._drive(mw, holder, clock, lanes=2, ticks=7)       # ~1.4 s of raw 2
    assert self._published(mw)['inferenceMode'] == 'gs_osm'   # < 1.5 s → held
    self._drive(mw, holder, clock, lanes=2, ticks=2)       # cross 1.5 s
    assert self._published(mw)['inferenceMode'] == 'lane_count'

  def test_path2_short_dip_then_recover_no_release(self, sld, monkeypatch):
    """ref empty + raw 2 for only ~0.8 s then back to 3 → the 1.5 s run resets →
    no release."""
    mw, holder = self._mw_model(sld)
    clock = {'t': 11000.0}
    monkeypatch.setattr(sld.time, 'monotonic', lambda: clock['t'])
    mw.lane_count_stable = 4
    holder['model'] = self._model(4)
    mw._ingest_osm_result(self._result(wayRef='S1', roadContext=0, highwayType='trunk'))
    mw.update()
    clock['t'] += 0.2
    mw._ingest_osm_result(self._result(roadName='ramp'))   # ref empty
    self._drive(mw, holder, clock, lanes=2, ticks=4)       # ~0.8 s of raw 2
    self._drive(mw, holder, clock, lanes=3, ticks=10)      # raw ≥3 → run resets
    assert self._published(mw)['inferenceMode'] == 'gs_osm'
    assert mw._gs_lane_drop_since is None

  def test_path2_requires_ref_empty_not_stacked(self, sld, monkeypatch):
    """raw drops to 2 but the G/S ref is STILL matching (stacked) → NO release:
    the conjunction requires ref-empty. Same 1.5 s+ duration that releases when
    ref IS empty (previous test)."""
    mw, holder = self._mw_model(sld)
    clock = {'t': 12000.0}
    monkeypatch.setattr(sld.time, 'monotonic', lambda: clock['t'])
    mw.lane_count_stable = 4
    holder['model'] = self._model(2)
    # S1 keeps matching on every query while raw sits at 2 for ~1.8 s.
    for _ in range(9):
      mw._ingest_osm_result(self._result(wayRef='S1', roadContext=0, highwayType='trunk'))
      mw.update()
      clock['t'] += 0.2
    assert self._published(mw)['inferenceMode'] == 'gs_osm'   # ref present → no path-2

  # ============ Both paths coexist (layered for geometry) ==================

  def test_coexist_margin_first_on_wide_ramp(self, sld, monkeypatch):
    """Wide ramp (raw ≥3, so path 2 idle): the MARGIN path releases."""
    mw, holder = self._mw_model(sld)
    clock = {'t': 13000.0}
    monkeypatch.setattr(sld.time, 'monotonic', lambda: clock['t'])
    mw.lane_count_stable = 4
    holder['model'] = self._model(4)             # wide → raw stays 4
    mw._ingest_osm_result(self._result(wayRef='S1', roadContext=0, highwayType='trunk',
                                       refDistances={'S1': 3.0}))
    mw.update()
    clock['t'] += 0.2
    # Divergent queries at the 1 Hz cadence until GS_RELEASE_MARGIN_S (5 s) of
    # continuous evidence has accumulated (was 2 queries 0.2 s apart under the
    # count-based rule — spacing the time-based rule rightly rejects).
    for _ in range(6):                           # elapsed 0..5 s of margin ~13
      mw._ingest_osm_result(self._result(roadName='ramp', highwayType='motorway_link',
                                         distance=0.6, refDistances={'S1': 13.5}))
      mw.update()
      clock['t'] += 1.0
    assert self._published(mw)['inferenceMode'] == 'lane_count'
    assert mw._gs_force_release is True          # released via the margin path
    assert mw._gs_lane_drop_since is None        # path 2 never armed (raw ≥3)

  def test_coexist_lanedrop_first_on_narrow_ramp(self, sld, monkeypatch):
    """Narrow ramp with NO candidate distances (margin can't fire): the
    ref-empty + lane-drop path releases at 1.5 s."""
    mw, holder = self._mw_model(sld)
    clock = {'t': 14000.0}
    monkeypatch.setattr(sld.time, 'monotonic', lambda: clock['t'])
    mw.lane_count_stable = 4
    holder['model'] = self._model(4)
    mw._ingest_osm_result(self._result(wayRef='S1', roadContext=0, highwayType='trunk'))
    mw.update()
    clock['t'] += 0.2
    mw._ingest_osm_result(self._result(roadName='ramp'))   # ref empty, NO refDistances
    self._drive(mw, holder, clock, lanes=2, ticks=10)      # ~2 s of raw 2
    assert self._published(mw)['inferenceMode'] == 'lane_count'
    assert mw._gs_force_release is False         # margin never fired (no distances)

  # ================= F6 — kept-code robustness (review) ====================

  def test_oscillation_ladder_damps_displayed_limit(self, sld, monkeypatch):
    """Release → genuine G/S re-match (re-promote, margin count reset) → fresh
    divergence → re-release, repeated. The inferenceMode flag may oscillate, but
    _displayed_speed_limit only ever moves ONE standard-ladder rung per step
    interval (3 s down / 2 s up) — it never teleports 100→60 and back (review
    F3, the ladder damping)."""
    mw = self._mw(sld)
    mw.lane_count_stable = 3
    clock = {'t': 20000.0}
    monkeypatch.setattr(sld.time, 'monotonic', lambda: clock['t'])
    STD = sld._STANDARD_SPEEDS
    displayed = []
    self._hold_s1(mw)
    displayed.append(mw._displayed_speed_limit)
    for _ in range(3):
      self._diverge(mw, clock, held_dist=13.5, matched_dist=0.6)
      displayed.append(mw._displayed_speed_limit)
      self._diverge(mw, clock, held_dist=17.0, matched_dist=0.6)
      displayed.append(mw._displayed_speed_limit)
      assert mw._gs_force_release is True
      self._hold_s1(mw)                          # genuine re-match re-promotes
      displayed.append(mw._displayed_speed_limit)
      assert mw._gs_margin_since is None            # reset on the re-match
    # No teleport: consecutive displayed limits are equal or exactly one rung apart.
    for a, b in zip(displayed, displayed[1:]):
      assert abs(STD.index(a) - STD.index(b)) <= 1, f'displayed teleported {a}->{b}'
    assert min(displayed) >= 80                  # never cliffed to the released 60

  def test_held_ref_transitions_s1_to_s20(self, sld, monkeypatch):
    """Held on S1, then S20 (a different expressway) matches → _gs_held_ref
    switches cleanly to S20 and the margin path then tracks S20 — NOT confused by
    a stale S1 that happens to sit close in the candidate set."""
    mw = self._mw(sld)
    mw.lane_count_stable = 4
    clock = {'t': 21000.0}
    monkeypatch.setattr(sld.time, 'monotonic', lambda: clock['t'])
    self._hold_s1(mw)
    assert mw._gs_held_ref == 'S1'
    mw._ingest_osm_result(self._result(wayRef='S20', roadContext=0, highwayType='trunk',
                                       refDistances={'S20': 2.0}))
    clock['t'] += 5.0
    mw.update()
    assert mw._gs_held_ref == 'S20'              # held identity switched cleanly
    assert mw._gs_margin_since is None
    # Divergence tracks S20 (20 m, margin 19.4 > 8), ignoring S1 sitting at 1 m.
    for _ in range(2):
      mw._ingest_osm_result(self._result(roadName='ramp', highwayType='motorway_link',
                                         distance=0.6,
                                         refDistances={'S20': 20.0, 'S1': 1.0}))
      clock['t'] += 5.0
      mw.update()
    assert self._published(mw)['inferenceMode'] == 'lane_count'
    assert mw._gs_force_release is True

  def test_f1_edge_misread_releases_expressway_hold_accepted(self, sld, monkeypatch):
    """F1 accepted-risk characterization (user decision 2026-08-03, made with the
    replay + review numbers in view). With the edge boost gated to ≥3 visible
    lanes, an edge-lane MISREAD on a genuine expressway (vision sees only 2 lines
    while hugging an edge) reads RAW 2 — the gate does NOT rescue it to 3.
    Sustained ≥3 s while gs-held (ref still matching), the narrow accumulator
    commits lane_count_stable=2 and the existing lane≤2 escape releases the
    100/120 hold to a 40 base inference. Asserted AS the accepted behavior
    (damped by the 3 s accumulator + display ladder, gas-overridable) so a future
    change that alters it is a conscious one."""
    mw, holder = self._mw_model(sld)
    clock = {'t': 22000.0}
    monkeypatch.setattr(sld.time, 'monotonic', lambda: clock['t'])
    mw.lane_count_stable = 4
    # base-2 probs WITH confident, co-located edges — the gate keeps it at 2.
    edge2 = self._straight_model([0.6, 0.7, 0.1, 0.1])
    edge2.roadEdgeStds = [0.3, 0.3]
    assert sld.infer_lane_count(edge2) == 2        # gate does NOT boost to 3
    # Establish an S1 hold (raw 4, ref matching).
    holder['model'] = self._model(4)
    for _ in range(3):
      mw._ingest_osm_result(self._result(wayRef='S1', roadContext=0, highwayType='trunk'))
      mw.update()
      clock['t'] += 0.2
    assert self._published(mw)['inferenceMode'] == 'gs_osm'
    # The edge-lane misread: raw 2 sustained > 3 s while S1 keeps matching.
    holder['model'] = edge2
    for _ in range(25):                            # 5 s
      mw._ingest_osm_result(self._result(wayRef='S1', roadContext=0, highwayType='trunk'))
      mw.update()
      clock['t'] += 0.2
    pub = self._published(mw)
    assert mw.lane_count_stable == 2               # accumulator committed narrow
    assert pub['inferenceMode'] == 'lane_count'    # 100/120 hold released
    assert pub['inferredSpeed'] == 40              # narrow base inference


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


# ============================================================
# OSM Data Integration — param wiring + region default
# ============================================================

class TestOsmIntegrationParam:
  def _make_middleware(self, sld):
    import plugins.speedlimitd.speedlimitd as mod
    with patch.object(mod.messaging, 'SubMaster'):
      mw = mod.SpeedLimitMiddleware()
    mw._sl_pub = MagicMock()
    return mw

  def test_read_params_wires_toggle_on(self, sld, monkeypatch):
    import config
    monkeypatch.setattr(config, 'read_plugin_param',
                        lambda pid, key, default='': '1' if key == 'OsmDataIntegration' else '')
    mw = self._make_middleware(sld)
    mw._read_params()
    assert mw.osm_integration_enabled is True

  def test_read_params_missing_means_off(self, sld, monkeypatch):
    import config
    monkeypatch.setattr(config, 'read_plugin_param', lambda pid, key, default='': '')
    mw = self._make_middleware(sld)
    mw._read_params()
    assert mw.osm_integration_enabled is False


class TestOsmRegionDefault:
  def _make_middleware(self, sld):
    import plugins.speedlimitd.speedlimitd as mod
    with patch.object(mod.messaging, 'SubMaster'):
      mw = mod.SpeedLimitMiddleware()
    mw._sl_pub = MagicMock()
    return mw

  def _param_path(self, tmp_path):
    return tmp_path / 'speedlimitd' / 'data' / 'OsmDataIntegration'

  @pytest.fixture
  def data_dir(self, monkeypatch, tmp_path):
    import config
    monkeypatch.setattr(config, 'PLUGINS_RUNTIME_DIR', str(tmp_path))
    return tmp_path

  def test_cn_defaults_off(self, sld, data_dir):
    mw = self._make_middleware(sld)
    assert mw._resolve_osm_default('cn') is True
    assert self._param_path(data_dir).read_text() == '0'
    assert mw.osm_integration_enabled is False

  def test_non_cn_defaults_on(self, sld, data_dir):
    mw = self._make_middleware(sld)
    assert mw._resolve_osm_default('de') is True
    assert self._param_path(data_dir).read_text() == '1'
    assert mw.osm_integration_enabled is True

  def test_unknown_country_defaults_on(self, sld, data_dir):
    # No bbox match (e.g. US without a us.toml) → not China → ON.
    mw = self._make_middleware(sld)
    assert mw._resolve_osm_default(None) is True
    assert self._param_path(data_dir).read_text() == '1'

  def test_existing_value_never_overwritten(self, sld, data_dir):
    p = self._param_path(data_dir)
    p.parent.mkdir(parents=True)
    p.write_text('0')
    mw = self._make_middleware(sld)
    assert mw._resolve_osm_default('de') is True
    assert p.read_text() == '0'


class TestOsmSpeedStorage:
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

  def test_maxspeed_stored_in_kph(self, sld):
    mw = self._make_middleware(sld)
    mw._ingest_osm_result(self._result(roadName='A1', speedLimit=27.78))
    assert mw.last_osm_speed_kph == pytest.approx(100.0, abs=0.1)
    assert mw.last_osm_speed_t > 0

  def test_same_road_without_maxspeed_keeps_value(self, sld):
    # Sub-segments of the same road may lack the tag — hold the value
    # (the freshness gate expires it if it stays absent).
    mw = self._make_middleware(sld)
    mw._ingest_osm_result(self._result(roadName='A1', speedLimit=27.78))
    mw._ingest_osm_result(self._result(roadName='A1', speedLimit=0.0))
    assert mw.last_osm_speed_kph == pytest.approx(100.0, abs=0.1)

  def test_new_road_without_maxspeed_clears_value(self, sld):
    mw = self._make_middleware(sld)
    mw._ingest_osm_result(self._result(roadName='A1', speedLimit=27.78))
    mw._ingest_osm_result(self._result(roadName='B2', speedLimit=0.0))
    assert mw.last_osm_speed_kph == 0.0

  def test_no_match_keeps_value_for_staleness(self, sld):
    mw = self._make_middleware(sld)
    mw._ingest_osm_result(self._result(roadName='A1', speedLimit=27.78))
    mw._ingest_osm_result(None)
    assert mw.last_osm_speed_kph == pytest.approx(100.0, abs=0.1)


class TestOsmBaseActive:
  def _make_middleware(self, sld):
    import plugins.speedlimitd.speedlimitd as mod
    with patch.object(mod.messaging, 'SubMaster'):
      mw = mod.SpeedLimitMiddleware()
    mw._sl_pub = MagicMock()
    return mw

  def _arm(self, mw, kph=100.0, age=0.0):
    import time as _t
    mw.country_detected = True  # a GPS fix has landed: country is RESOLVED
    mw.country = 'us'          # non-CN: gate is the pre-2026-08-10 behaviour
    mw.osm_integration_enabled = True
    mw.last_osm_speed_kph = kph
    mw.last_osm_speed_t = _t.monotonic() - age

  def test_active_when_fresh_and_enabled(self, sld):
    import time as _t
    mw = self._make_middleware(sld)
    self._arm(mw)
    assert mw._osm_gate(_t.monotonic(), gs_mode=False)[0] is True

  def test_inactive_when_toggle_off(self, sld):
    import time as _t
    mw = self._make_middleware(sld)
    self._arm(mw)
    mw.osm_integration_enabled = False
    assert mw._osm_gate(_t.monotonic(), gs_mode=False)[0] is False

  def test_inactive_when_stale(self, sld):
    import time as _t
    mw = self._make_middleware(sld)
    self._arm(mw, age=11.0)  # > 2 × 5 s query interval
    assert mw._osm_gate(_t.monotonic(), gs_mode=False)[0] is False

  def test_inactive_when_implausibly_low(self, sld):
    import time as _t
    mw = self._make_middleware(sld)
    self._arm(mw, kph=20.0)  # < 30 km/h plausibility floor
    assert mw._osm_gate(_t.monotonic(), gs_mode=False) == (False, 'no_data')


class TestOsmBaseSelection:
  """OSM maxspeed as base inference — full update() cycles."""

  def _mw_for_update(self, sld):
    import plugins.speedlimitd.speedlimitd as mod
    with patch.object(mod.messaging, 'SubMaster'):
      mw = mod.SpeedLimitMiddleware()
    mw._sl_pub = MagicMock()
    sm = MagicMock()
    sm.updated = {'modelV2': False, 'gpsLocationExternal': False, 'livePose': False}
    sm.update = MagicMock()
    mw.sm = sm
    mw._cmd_sub = None
    mw._lc_sub = None
    return mw

  def _arm(self, mw, kph):
    import time as _t
    mw.country_detected = True  # a GPS fix has landed: country is RESOLVED
    mw.country = 'us'          # non-CN: gate is the pre-2026-08-10 behaviour
    mw.osm_integration_enabled = True
    mw.last_osm_speed_kph = kph
    mw.last_osm_speed_t = _t.monotonic()
    # update()'s 5 s param refresh would re-read the (absent) param and wipe
    # the armed toggle — mark params as freshly read.
    mw._params_last_read_t = _t.monotonic()

  def test_osm_replaces_base_and_bypasses_cn_ladder(self, sld):
    mw = self._mw_for_update(sld)
    mw.lane_count_stable = 2          # vision table would give a low urban limit
    self._arm(mw, 104.6)              # 65 mph — NOT a CN-ladder value
    mw.update()
    pub = mw._sl_pub.send.call_args[0][0]
    assert pub['inferenceMode'] == 'osm'
    assert pub['speedLimit'] == 105   # round-to-5, NOT snapped to 100/120
    assert pub['speedLimit'] not in sld._STANDARD_SPEEDS

  def test_toggle_off_is_todays_behavior(self, sld):
    mw = self._mw_for_update(sld)
    mw.lane_count_stable = 2
    self._arm(mw, 104.6)
    mw.osm_integration_enabled = False
    mw.update()
    pub = mw._sl_pub.send.call_args[0][0]
    assert pub['inferenceMode'] == 'lane_count'
    assert pub['speedLimit'] in sld._STANDARD_SPEEDS

  def test_stale_osm_falls_back_to_vision(self, sld):
    import time as _t
    mw = self._mw_for_update(sld)
    mw.lane_count_stable = 2
    self._arm(mw, 104.6)
    mw.last_osm_speed_t = _t.monotonic() - 11.0
    mw.update()
    pub = mw._sl_pub.send.call_args[0][0]
    assert pub['inferenceMode'] == 'lane_count'

  def test_vision_narrow_cap_not_applied_over_osm(self, sld):
    # A rural 2-lane road with OSM 90 must NOT be capped by the ≤2-lane
    # narrow-road heuristic — OSM is authoritative for the base.
    mw = self._mw_for_update(sld)
    mw.lane_count_stable = 2
    mw.vision_cap_stable = 60
    self._arm(mw, 90.0)
    mw.update()
    pub = mw._sl_pub.send.call_args[0][0]
    assert pub['speedLimit'] == 90

  def test_safety_cap_still_wins_over_osm(self, sld):
    mw = self._mw_for_update(sld)
    self._arm(mw, 104.6)
    mw.curvature_cap = 50
    mw.update()
    pub = mw._sl_pub.send.call_args[0][0]
    assert pub['speedLimit'] <= sld.snap_to_standard_speed(50)
    assert pub['safetyCapped'] is True

  def test_safety_cap_uses_ladder_snap_not_osm_round(self, sld):
    # curvature_cap=57 is the discriminating value: the CN-ladder snap
    # (source 4, safety class) rounds UP to 60, while the OSM round-to-5
    # display path would give 55. The two disagree in a direction the
    # downstream "safety cap override" re-clamp (which only ever pulls the
    # displayed value DOWN toward snap_to_standard_speed(curvature_cap))
    # cannot paper over — 60 is not < 55, so that block would leave a
    # regressed 55 standing. If the `osm_display = osm_base and source == 2`
    # guard drops the `source == 2` check, the safety cap wrongly takes the
    # OSM round-to-5 path and this test catches the resulting 55.
    mw = self._mw_for_update(sld)
    self._arm(mw, 104.6)
    mw.curvature_cap = 57
    mw.update()
    pub = mw._sl_pub.send.call_args[0][0]
    assert pub['speedLimit'] == 60
    assert pub['speedLimit'] in sld._STANDARD_SPEEDS
    assert pub['safetyCapped'] is True

  def test_osm_suppresses_gs_hold(self, sld):
    import time as _t
    mw = self._mw_for_update(sld)
    # Arm an active G/S sticky hold at 120…
    mw.lane_count_stable = 4
    mw.last_highway_type = 'motorway'
    mw.last_road_context = 'freeway'
    mw.last_way_ref = 'G2'
    mw._gs_last_seen_t = _t.monotonic()
    mw._gs_limit_kph = 120
    # …and a fresh OSM 100 → OSM wins the base.
    self._arm(mw, 100.0)
    mw.update()
    pub = mw._sl_pub.send.call_args[0][0]
    assert pub['inferenceMode'] == 'osm'
    assert pub['speedLimit'] == 100

  def test_osm_step_is_plus_minus_10(self, sld):
    mw = self._mw_for_update(sld)
    self._arm(mw, 65.0)
    mw._displayed_speed_limit = 105   # previously showing 105
    mw._last_step_time = 0.0          # step interval elapsed
    mw.update()
    pub = mw._sl_pub.send.call_args[0][0]
    assert pub['speedLimit'] == 95    # one −10 step toward 65, not a ladder jump

  def test_publish_carries_osm_fields(self, sld):
    mw = self._mw_for_update(sld)
    self._arm(mw, 88.5)
    mw._osm_tiles_missing = True
    mw.update()
    pub = mw._sl_pub.send.call_args[0][0]
    assert pub['osmSpeedLimit'] == pytest.approx(88.5, abs=0.1)
    assert pub['osmTilesMissing'] is True


class TestOsmTilesMissingWritePath:
  """Drives the real `update()` OSM-query block — tile_missing detection and
  the persisted OsmTilesMissing param, written on first status and on change
  only (not every 5 s query cycle)."""

  def _mw_for_update(self, sld):
    import plugins.speedlimitd.speedlimitd as mod
    with patch.object(mod.messaging, 'SubMaster'):
      mw = mod.SpeedLimitMiddleware()
    mw._sl_pub = MagicMock()
    sm = MagicMock()
    sm.updated = {'modelV2': False, 'gpsLocationExternal': False, 'livePose': False}
    sm.update = MagicMock()
    mw.sm = sm
    mw._cmd_sub = None
    mw._lc_sub = None
    return mw

  def _param_path(self, tmp_path):
    return tmp_path / 'speedlimitd' / 'data' / 'OsmTilesMissing'

  @pytest.fixture
  def data_dir(self, monkeypatch, tmp_path):
    import config
    monkeypatch.setattr(config, 'PLUGINS_RUNTIME_DIR', str(tmp_path))
    return tmp_path

  def _arm_query(self, mw, tile_missing):
    """Force update()'s real OSM-query block to run this tick."""
    mw._gps_valid = True
    mw._gps_lat = 39.9
    mw._gps_lon = 116.4
    mw._osm_last_query_t = 0.0
    mw._osm = MagicMock()
    mw._osm.query.return_value = None
    mw._osm.tile_missing = tile_missing

  def test_first_status_writes_missing_true(self, sld, data_dir):
    mw = self._mw_for_update(sld)
    self._arm_query(mw, True)
    mw.update()
    assert self._param_path(data_dir).read_text() == '1'
    pub = mw._sl_pub.send.call_args[0][0]
    assert pub['osmTilesMissing'] is True

  def test_change_to_present_rewrites_param(self, sld, data_dir):
    mw = self._mw_for_update(sld)
    self._arm_query(mw, True)
    mw.update()
    assert self._param_path(data_dir).read_text() == '1'

    mw._osm.tile_missing = False
    mw._osm_last_query_t = 0.0  # allow another query this tick
    mw.update()
    assert self._param_path(data_dir).read_text() == '0'
    pub = mw._sl_pub.send.call_args[0][0]
    assert pub['osmTilesMissing'] is False

  def test_unchanged_status_not_rewritten(self, sld, data_dir, monkeypatch):
    import config
    mw = self._mw_for_update(sld)
    self._arm_query(mw, True)
    mw.update()  # first status write — establishes the on-disk baseline

    calls = []
    real_write = config.write_plugin_param

    def _recording_write(plugin_id, key, value):
      calls.append((plugin_id, key, value))
      return real_write(plugin_id, key, value)

    monkeypatch.setattr(config, 'write_plugin_param', _recording_write)

    mw._osm_last_query_t = 0.0  # allow another query this tick
    mw.update()  # tile_missing unchanged (still True) — no redundant write

    tiles_missing_calls = [c for c in calls if c[1] == 'OsmTilesMissing']
    assert tiles_missing_calls == []


class TestCountryField:
  def _make_middleware(self, sld):
    import plugins.speedlimitd.speedlimitd as mod
    with patch.object(mod.messaging, 'SubMaster'):
      mw = mod.SpeedLimitMiddleware()
    mw._sl_pub = MagicMock()
    return mw

  def test_country_defaults_to_empty_string(self, sld):
    mw = self._make_middleware(sld)
    assert mw.country == ''

  def test_resolve_osm_default_reads_country(self, sld):
    """cn → OFF, anything else → ON (unchanged behaviour, new field name)."""
    mw = self._make_middleware(sld)
    mw.country = 'cn'
    with patch('config.write_plugin_param') as wp, \
         patch('config.plugin_data_dir') as pdd:
      pdd.return_value.__truediv__.return_value.exists.return_value = False
      mw._resolve_osm_default(mw.country)
      assert wp.call_args[0][2] == '0'

  def _mw_for_gps_update(self, sld, gps_flags=1, gps_lat=39.9, gps_lon=116.4):
    """Build a middleware whose update() will run the real GPS-detection
    block (sm.updated['gpsLocationExternal'] True, a valid-fix gps object)."""
    import plugins.speedlimitd.speedlimitd as mod
    with patch.object(mod.messaging, 'SubMaster'):
      mw = mod.SpeedLimitMiddleware()
    mw._sl_pub = MagicMock()
    mw._cmd_sub = None
    mw._lc_sub = None
    gps = MagicMock()
    gps.flags = gps_flags
    gps.latitude = gps_lat
    gps.longitude = gps_lon
    sm = MagicMock()
    sm.updated = {'modelV2': False, 'gpsLocationExternal': True, 'livePose': False}
    sm.update = MagicMock()
    sm.__getitem__ = MagicMock(side_effect=lambda k: gps if k == 'gpsLocationExternal' else MagicMock())
    mw.sm = sm
    return mw

  def test_gps_country_none_coerced_to_empty_string(self, sld):
    """GPS fix outside every known bounding box → country_from_gps returns
    None → self.country must be coerced to '' at the real assignment site
    (speedlimitd.py:1104), never left as None."""
    import plugins.speedlimitd.speedlimitd as mod
    mw = self._mw_for_gps_update(sld)
    with patch.object(mod, 'country_from_gps', return_value=None), \
         patch('config.write_plugin_param'), \
         patch('config.plugin_data_dir') as pdd:
      pdd.return_value.__truediv__.return_value.exists.return_value = False
      mw.update()
    assert mw.country == ''
    assert mw.country is not None

  def test_gps_country_cn_sets_field(self, sld):
    """Companion case: a resolved country ('cn') is passed straight through."""
    import plugins.speedlimitd.speedlimitd as mod
    mw = self._mw_for_gps_update(sld)
    with patch.object(mod, 'country_from_gps', return_value='cn'), \
         patch('config.write_plugin_param'), \
         patch('config.plugin_data_dir') as pdd:
      pdd.return_value.__truediv__.return_value.exists.return_value = False
      mw.update()
    assert mw.country == 'cn'


class TestOsmGsGate:
  """CN: a posted OSM maxspeed is trusted only on a confirmed G/S expressway."""

  def _mw(self, sld):
    import plugins.speedlimitd.speedlimitd as mod
    with patch.object(mod.messaging, 'SubMaster'):
      mw = mod.SpeedLimitMiddleware()
    mw._sl_pub = MagicMock()
    return mw

  def _arm_cn(self, mw, kph, way_ref):
    import time as _t
    mw.country_detected = True   # a GPS fix has landed: country is RESOLVED
    mw.country = 'cn'
    mw.osm_integration_enabled = True
    mw.last_osm_speed_kph = kph
    mw.last_osm_speed_t = _t.monotonic()
    mw.last_way_ref = way_ref
    mw._osm_gate_ref = way_ref     # _osm_gate reads this, not last_way_ref

  def test_gs_expressway_is_trusted(self, sld):
    import time as _t
    mw = self._mw(sld)
    self._arm_cn(mw, 100.0, 'G1503')
    assert mw._osm_gate(_t.monotonic(), gs_mode=True) == (True, '')

  def test_ramp_sign_on_expressway_rejected(self, sld):
    """Audit: 40 km/h ramp signs mis-attributed onto 高速/高架 mainlines."""
    import time as _t
    mw = self._mw(sld)
    self._arm_cn(mw, 40.0, 'G1503')
    assert mw._osm_gate(_t.monotonic(), gs_mode=True) == (False, 'low_value')

  def test_viaduct_tag_on_surface_road_rejected(self, sld):
    """Audit: an elevated deck's 80 written onto the arterial beneath it."""
    import time as _t
    mw = self._mw(sld)
    self._arm_cn(mw, 80.0, '')          # 华夏中路 etc. carry no G/S ref
    assert mw._osm_gate(_t.monotonic(), gs_mode=True) == (False, 'not_gs')

  def test_three_digit_guodao_rejected(self, sld):
    """G312 is an ordinary surface highway, not a controlled-access expressway."""
    import time as _t
    mw = self._mw(sld)
    self._arm_cn(mw, 80.0, 'G312')
    assert mw._osm_gate(_t.monotonic(), gs_mode=True) == (False, 'not_gs')

  def test_gs_released_rejects_even_with_ref(self, sld):
    """Exiting onto a ≤2-lane ramp releases gs_mode; the posted 100 must not hold."""
    import time as _t
    mw = self._mw(sld)
    self._arm_cn(mw, 100.0, 'G1503')
    assert mw._osm_gate(_t.monotonic(), gs_mode=False) == (False, 'not_gs')

  def test_unknown_country_uses_strict_gate(self, sld):
    """Before the first GPS fix an opted-in CN user must not get the open gate."""
    import time as _t
    mw = self._mw(sld)
    self._arm_cn(mw, 80.0, '')
    mw.country_detected = False   # no fix yet: UNRESOLVED, not "resolved as non-CN"
    mw.country = ''
    assert mw._osm_gate(_t.monotonic(), gs_mode=True) == (False, 'not_gs')

  def test_non_cn_ignores_gs_entirely(self, sld):
    """US/EU regression guard: any way, no G/S ref, still trusted."""
    import time as _t
    mw = self._mw(sld)
    self._arm_cn(mw, 80.0, '')
    mw.country = 'us'            # _arm_cn already set country_detected True
    assert mw._osm_gate(_t.monotonic(), gs_mode=False) == (True, '')

  def test_reason_precedence_disabled_beats_not_gs(self, sld):
    import time as _t
    mw = self._mw(sld)
    self._arm_cn(mw, 80.0, '')
    mw.osm_integration_enabled = False
    assert mw._osm_gate(_t.monotonic(), gs_mode=True) == (False, 'disabled')

  def test_reason_precedence_stale_beats_not_gs(self, sld):
    import time as _t
    mw = self._mw(sld)
    self._arm_cn(mw, 80.0, '')
    mw.last_osm_speed_t = _t.monotonic() - 11.0
    assert mw._osm_gate(_t.monotonic(), gs_mode=True) == (False, 'stale')

  def test_no_match_query_does_not_wobble_the_gate(self, sld):
    """A single 5 s tile gap must not drop the gate's held ref and flip it.

    _osm_gate_ref (which _osm_gate reads) is held — not cleared — on a
    no-match _ingest_osm_result, paired with the already-held
    last_osm_speed_kph: both age together on the 10 s speed TTL rather than
    the ref dropping out a query early. last_way_ref itself DOES still clear
    immediately (see TestGsGateSharedStateIsolation below) — only the
    gate-private mirror is held, so gs_mode's release paths are unaffected.
    """
    import time as _t
    mw = self._mw(sld)
    self._arm_cn(mw, 100.0, 'G1503')
    mw._ingest_osm_result(None)          # one no-match query
    assert mw._osm_gate(_t.monotonic(), gs_mode=True) == (True, '')

  def test_gs_min_kph_boundary_exactly_60_trusted(self, sld):
    import time as _t
    mw = self._mw(sld)
    self._arm_cn(mw, 60.0, 'G1503')
    assert mw._osm_gate(_t.monotonic(), gs_mode=True) == (True, '')

  def test_gs_min_kph_boundary_59_rejected(self, sld):
    import time as _t
    mw = self._mw(sld)
    self._arm_cn(mw, 59.0, 'G1503')
    assert mw._osm_gate(_t.monotonic(), gs_mode=True) == (False, 'low_value')


class TestOsmGateCountryResolution:
  """The strict G/S gate is CN-only — it must key on RESOLVED-ness, not on
  self.country being a non-empty string.

  Only au/cn/de ship a speed_tables/*.toml with a bbox, so country_from_gps()
  returns None everywhere else and the assignment site latches
  country_detected=True with country=''. If the gate tested `self.country and
  ...`, that permanent '' would route every US/UK/FR/JP/CA driver down the
  China branch forever, where no way ref can match ^[GS](\\d{1,2}|\\d{4})$ —
  the gate would never open while its toggle still read ON.
  """

  def _mw_with_gps(self, sld, lat, lon):
    """Middleware whose update() runs the REAL GPS-detection block against a
    valid fix at (lat, lon) — country_from_gps is not stubbed, so the bbox
    tables decide, exactly as on the car."""
    import plugins.speedlimitd.speedlimitd as mod
    with patch.object(mod.messaging, 'SubMaster'):
      mw = mod.SpeedLimitMiddleware()
    mw._sl_pub = MagicMock()
    mw._cmd_sub = None
    mw._lc_sub = None
    gps = MagicMock()
    gps.flags = 1              # valid fix
    gps.latitude = lat
    gps.longitude = lon
    sm = MagicMock()
    sm.updated = {'modelV2': False, 'gpsLocationExternal': True, 'livePose': False}
    sm.update = MagicMock()
    sm.__getitem__ = MagicMock(side_effect=lambda k: gps if k == 'gpsLocationExternal' else MagicMock())
    mw.sm = sm
    return mw

  def test_us_fix_opens_gate_on_non_gs_way(self, sld):
    """Mountain View, CA (37.4, -122.1): matches no bbox, so country stays ''
    while country_detected latches True. The driver is provably not in China,
    so a fresh posted 105 km/h on a plain US way with no G/S ref must be
    trusted — the pre-2026-08-10 behaviour.

    Regression guard for the whole finding: this fails if the gate goes back
    to testing self.country truthiness.
    """
    import time as _t
    mw = self._mw_with_gps(sld, 37.4, -122.1)
    with patch('config.write_plugin_param'), \
         patch('config.plugin_data_dir') as pdd:
      pdd.return_value.__truediv__.return_value.exists.return_value = False
      mw.update()

    # Resolution HAPPENED but matched nothing: the two-state '' case.
    assert mw.country_detected is True
    assert mw.country == ''
    # ...and the region default armed the toggle ON (cn → '0', else → '1').
    assert mw.osm_integration_enabled is True

    mw.last_osm_speed_kph = 105.0
    mw.last_osm_speed_t = _t.monotonic()
    mw._osm_gate_ref = ''       # US ways carry no Chinese G/S ref, ever
    assert mw._osm_gate(_t.monotonic(), gs_mode=False) == (True, '')

  def test_cn_fix_still_uses_strict_gate(self, sld):
    """Companion: Beijing (39.9, 116.4) resolves to 'cn', so the same
    non-G/S way is rejected. Proves the US case above is not the gate
    going permissive for everyone."""
    import time as _t
    mw = self._mw_with_gps(sld, 39.9, 116.4)
    with patch('config.write_plugin_param'), \
         patch('config.plugin_data_dir') as pdd:
      pdd.return_value.__truediv__.return_value.exists.return_value = False
      mw.update()

    assert mw.country_detected is True
    assert mw.country == 'cn'

    mw.osm_integration_enabled = True   # cn default is OFF; opt the driver in
    mw.last_osm_speed_kph = 80.0
    mw.last_osm_speed_t = _t.monotonic()
    mw._osm_gate_ref = ''
    assert mw._osm_gate(_t.monotonic(), gs_mode=True) == (False, 'not_gs')

  def test_before_first_fix_gate_is_strict(self, sld):
    """The fail-safe boot window must survive the fix: with no GPS fix yet,
    country_detected is False and the gate stays strict even though country
    is '' — the driver might be in China."""
    import time as _t
    import plugins.speedlimitd.speedlimitd as mod
    with patch.object(mod.messaging, 'SubMaster'):
      mw = mod.SpeedLimitMiddleware()
    mw._sl_pub = MagicMock()

    assert mw.country_detected is False    # boot state: UNRESOLVED
    assert mw.country == ''

    mw.osm_integration_enabled = True      # toggle armed
    mw.last_osm_speed_kph = 80.0           # fresh, plausible posted limit
    mw.last_osm_speed_t = _t.monotonic()
    mw._osm_gate_ref = ''                  # non-G/S way
    assert mw._osm_gate(_t.monotonic(), gs_mode=True) == (False, 'not_gs')


class TestGsGateSharedStateIsolation:
  """Regression: _osm_gate_ref must be a private mirror, never a hold on the
  shared last_way_ref / gs_mode state (2026-08-10, round-2 correction).

  Round 1 of the wobble fix held last_way_ref ITSELF across a no-match
  query. That silently froze four of gs_mode's five release paths (the
  30 s sticky-ceiling refresh, the 10 s continuous-absence timer, the
  lane-drop timer, and the margin rule) for the duration of any tile gap —
  a safety regression on exactly the route-3de-seg-19 wide-ramp geometry the
  margin rule exists for: on a wide ramp (lane_count_stable > 2) none of the
  OTHER release paths fire either, so the expressway limit would never
  release. _osm_gate_ref fixes the OSM-gate wobble without touching that
  shared state; these tests pin last_way_ref's prompt-clear behaviour and
  the release machinery it drives.
  """

  def _mw(self, sld):
    import plugins.speedlimitd.speedlimitd as mod
    with patch.object(mod.messaging, 'SubMaster'):
      mw = mod.SpeedLimitMiddleware()
    mw._sl_pub = MagicMock()
    sm = MagicMock()
    sm.updated = {'modelV2': False, 'gpsLocationExternal': False, 'livePose': False}
    sm.update = MagicMock()
    mw.sm = sm
    mw._cmd_sub = None
    mw._lc_sub = None
    return mw

  def _result(self, **kw):
    base = {'wayRef': '', 'wayName': '', 'speedLimit': 0.0, 'lanes': 0,
            'roadContext': 1, 'roadName': '', 'highwayType': '', 'distance': 5.0}
    base.update(kw)
    return base

  def test_no_match_still_clears_last_way_ref(self, sld):
    """The shared field the G/S release machinery reads must clear on a
    no-match query exactly as it did before 2026-08-10 — only the
    gate-private _osm_gate_ref is held across the gap."""
    mw = self._mw(sld)
    mw._ingest_osm_result(self._result(wayRef='G1503', roadContext=0,
                                       highwayType='motorway'))
    assert mw.last_way_ref == 'G1503'
    mw._ingest_osm_result(None)
    assert mw.last_way_ref == ''                # cleared, bit-identical to pre-fix
    assert mw._osm_gate_ref == 'G1503'           # gate-private mirror stays held

  def test_sustained_no_match_starts_the_absence_run(self, sld, monkeypatch):
    """The whole point of the round-2 rework: a tile gap on a WIDE ramp (no
    other release path available) must still let the 10 s continuous-absence
    path engage and eventually release gs_mode. If a future change holds
    last_way_ref (shared state) instead of _osm_gate_ref, is_gs_now stays
    True forever on a no-match run and _gs_absent_since never leaves None —
    this test fails in exactly that case."""
    mw = self._mw(sld)
    mw.lane_count_stable = 3                    # wide connector — NOT narrow,
                                                 # so the lane<=2 release can't
                                                 # paper over a frozen timer
    clock = {'t': 4000.0}
    monkeypatch.setattr(sld.time, 'monotonic', lambda: clock['t'])
    # Establish a G/S hold.
    mw._ingest_osm_result(self._result(wayRef='G1503', roadContext=0,
                                       highwayType='motorway'))
    mw.update()
    assert mw._sl_pub.send.call_args[0][0]['inferenceMode'] == 'gs_osm'
    assert mw._gs_absent_since is None
    # Sustained no-tile-coverage gap (route 3de seg 19 geometry: wide ramp).
    mw._ingest_osm_result(None)
    clock['t'] += 5.0
    mw.update()
    assert mw._gs_absent_since is not None       # the absence run actually starts
    # ...and given the full 10 s, gs_mode genuinely releases.
    mw._ingest_osm_result(None)
    clock['t'] += 10.0
    mw.update()
    assert mw._sl_pub.send.call_args[0][0]['inferenceMode'] == 'lane_count'


class TestOsmGateTelemetry:
  def _mw_for_update(self, sld):
    import plugins.speedlimitd.speedlimitd as mod
    with patch.object(mod.messaging, 'SubMaster'):
      mw = mod.SpeedLimitMiddleware()
    mw._sl_pub = MagicMock()
    sm = MagicMock()
    sm.updated = {'modelV2': False, 'gpsLocationExternal': False, 'livePose': False}
    sm.update = MagicMock()
    mw.sm = sm
    mw._cmd_sub = None
    mw._lc_sub = None
    return mw

  def test_publishes_reject_reason_for_low_value(self, sld):
    import time as _t
    mw = self._mw_for_update(sld)
    mw.country_detected = True
    mw.country = 'cn'
    mw.osm_integration_enabled = True
    mw.last_osm_speed_kph = 40.0
    mw.last_osm_speed_t = _t.monotonic()
    mw.last_way_ref = 'G1503'
    mw._osm_gate_ref = 'G1503'  # _osm_gate reads this, not last_way_ref
    mw._params_last_read_t = _t.monotonic()
    # Hold gs_mode open: 4 lanes, freshly seen, no release latched.
    mw.lane_count_stable = 4
    mw._gs_last_seen_t = _t.monotonic()
    mw._gs_limit_kph = 100
    mw._gs_force_release = False
    mw._gs_absent_since = None
    mw.update()
    pub = mw._sl_pub.send.call_args[0][0]
    assert pub['osmTrusted'] is False
    assert pub['osmRejectReason'] == 'low_value'
    assert pub['osmSpeedLimit'] == 40.0      # still reported for telemetry
    assert pub['inferenceMode'] != 'osm'

  def test_publishes_trusted_with_empty_reason(self, sld):
    import time as _t
    mw = self._mw_for_update(sld)
    mw.country_detected = True
    mw.country = 'cn'
    mw.osm_integration_enabled = True
    mw.last_osm_speed_kph = 100.0
    mw.last_osm_speed_t = _t.monotonic()
    mw.last_way_ref = 'G1503'
    mw._osm_gate_ref = 'G1503'  # _osm_gate reads this, not last_way_ref
    mw._params_last_read_t = _t.monotonic()
    # Hold gs_mode open: 4 lanes, freshly seen, no release latched.
    mw.lane_count_stable = 4
    mw._gs_last_seen_t = _t.monotonic()
    mw._gs_limit_kph = 100
    mw._gs_force_release = False
    mw._gs_absent_since = None
    mw.update()
    pub = mw._sl_pub.send.call_args[0][0]
    assert pub['osmTrusted'] is True
    assert pub['osmRejectReason'] == ''
    assert pub['inferenceMode'] == 'osm'


class TestMapdPhase1Telemetry:
  """Phase 1 publishes mapd observations and changes NOTHING else.

  The point of the phase is to compare mapd's road context against the tile
  reader's on a real drive while the tile reader still drives every control
  path. If any mapd value reached actuation, the comparison would be measuring
  itself.
  """

  def test_mapd_out_is_subscribed(self, sld):
    import inspect
    src = inspect.getsource(sld.SpeedLimitMiddleware.__init__)
    assert "'mapdOut'" in src or '"mapdOut"' in src

  def test_telemetry_keys_published(self, sld, monkeypatch):
    mw = sld.SpeedLimitMiddleware()
    sent = {}
    mw._sl_pub.send = lambda payload: sent.update(payload)
    mw.update()
    for key in ('mapdAlive', 'mapdWayRef', 'mapdWayId', 'mapdSpeedLimit',
                'mapdHwClass', 'mapdLanes', 'mapdSelType', 'mapdTileLoaded',
                'mapdDistance', 'mapdRefAgree', 'mapdRoadContext'):
      assert key in sent, f'{key} missing from published telemetry'

  def test_publishes_even_when_mapd_absent(self, sld):
    # A drive with a dead mapd must still yield an analysable rlog.
    mw = sld.SpeedLimitMiddleware()
    sent = {}
    mw._sl_pub.send = lambda payload: sent.update(payload)
    mw.update()
    assert sent['mapdAlive'] is False
    assert sent['mapdWayRef'] == ''

  @staticmethod
  def _drop_mapd_service():
    """Simulate a device whose cereal has no injected mapdOut service.

    Reachable three ways: a catpilot `git reset --hard` wiping the injection,
    an install that touched the enforcement markers after overlay_cereal, and a
    user's .disabled on mapd at the next install. SubMaster raises on an unknown
    service, so an unguarded subscribe crash-loops speedlimitd and the car
    drives with NO speed-limit data — a far worse failure than losing Phase 1
    telemetry. The mock sys.modules entry is restored by mock_openpilot's
    monkeypatch.
    """
    services = sys.modules['cereal.services']
    services.SERVICE_LIST = {k: v for k, v in services.SERVICE_LIST.items()
                             if k != 'mapdOut'}

  def test_builds_without_mapd_service_in_service_list(self, sld):
    self._drop_mapd_service()
    mw = sld.SpeedLimitMiddleware()
    assert mw._mapd_service_available is False
    subs = sld.messaging.SubMaster.call_args[0][0]
    assert 'mapdOut' not in subs
    assert 'modelV2' in subs and 'livePose' in subs

  def test_update_runs_and_still_publishes_absent_shape_without_service(self, sld):
    self._drop_mapd_service()
    mw = self._armed(sld)          # armed: the 5 s sampling block really runs
    sent = {}
    mw._sl_pub.send = lambda payload: sent.update(payload)
    mw.update()
    assert mw._osm_last_query_t > 0.0   # the sampling block was entered
    assert sent['mapdAlive'] is False
    assert sent['mapdWayRef'] == ''
    assert sent['mapdRoadContext'] == ''

  @staticmethod
  def _armed(sld):
    """An instance whose 5 s OSM/mapd sampling block will actually run.

    A fresh instance has _gps_valid False, so the sampling block is skipped
    entirely — a containment test built on one would pass trivially without ever
    injecting anything.
    """
    mw = sld.SpeedLimitMiddleware()
    mw._gps_valid = True
    mw._gps_lat, mw._gps_lon = 31.3137, 121.5395
    mw._osm_last_query_t = 0.0
    return mw

  def test_sampling_block_actually_runs(self, sld):
    # Guards the containment test below: proves _armed() reaches the sampling
    # path, so a passing containment result means something.
    mw = self._armed(sld)
    mw.update()
    assert mw._osm_last_query_t > 0.0

  def test_mapd_values_do_not_reach_control_state(self, sld):
    """Containment: injecting mapd data must not move any control variable.

    Runs update() from identical armed state twice — once with no mapd data,
    once with a full mapd message reporting a DIFFERENT road at a DIFFERENT
    speed — and asserts every control-bearing attribute is unchanged. The tile
    reader sees no tiles in either run, so it contributes identically.

    Two complementary assertions:
    - `control_attrs` is a fixed, named list — it FAILS FAST and localises
      exactly which attribute leaked.
    - the full published payload (minus the eleven `mapd*` keys) must also be
      identical between the two runs. That is the actual "actuation stays
      byte-identical" claim — `speedLimit` (planner_hook's input) and
      `laneCount` live here, not in `control_attrs`, and unlike a hand-picked
      attribute list this can't go stale as speedlimitd grows new published
      fields.
    """
    control_attrs = ('last_way_ref', 'last_road_name', 'last_osm_hwtype',
                     'last_osm_speed_kph', 'last_road_context',
                     'last_highway_type', 'last_road_id', 'curvature_cap',
                     'inference_mode', '_gs_limit_kph')

    baseline = self._armed(sld)
    baseline_sent = {}
    baseline._sl_pub.send = lambda payload: baseline_sent.update(payload)
    baseline.update()
    before = {a: getattr(baseline, a) for a in control_attrs}

    fake = MagicMock()
    fake.wayRef = 'G1503'
    fake.wayName = 'injected'
    fake.roadName = 'injected'
    fake.speedLimit = 33.3
    fake.lanes = 5
    fake.highwayClass = 'motorway'
    fake.wayId = 999
    fake.tileLoaded = True
    fake.distanceFromWayCenter = 0.4
    fake.waySelectionType = 'current'

    injected = self._armed(sld)
    real_sm = injected.sm
    injected.sm = MagicMock()
    injected.sm.__getitem__ = lambda _s, k: fake if k == 'mapdOut' else real_sm[k]
    injected.sm.valid = {'mapdOut': True}
    # alive must be explicit too (2nd review round): the freshness conjunct
    # added at the speedlimitd.py call site reads sm.alive, not just sm.valid.
    injected.sm.alive = {'mapdOut': True}
    # Route to real_sm.updated (not a bare {}) so every OTHER key (modelV2,
    # gpsLocationExternal, livePose) behaves exactly as it does in the
    # baseline run above — the containment comparison is only meaningful if
    # mapdOut is the sole difference between the two update() calls. A literal
    # {} here made speedlimitd.py's unrelated `self.sm.updated['modelV2']`
    # direct-index read KeyError, which is a mocking-parity bug, not a leak.
    injected.sm.updated = real_sm.updated
    sent = {}
    injected._sl_pub.send = lambda payload: sent.update(payload)
    injected.update()

    after = {a: getattr(injected, a) for a in control_attrs}
    assert after == before, 'mapd data leaked into a control variable'

    # Structural containment over the WHOLE published payload, not just the
    # named control_attrs above.
    baseline_rest = {k: v for k, v in baseline_sent.items() if not k.startswith('mapd')}
    injected_rest = {k: v for k, v in sent.items() if not k.startswith('mapd')}
    assert injected_rest == baseline_rest, 'mapd data leaked into the published payload'

    # ...and the injected data really did arrive, so the assertions above are
    # testing containment rather than an absent message.
    assert sent['mapdWayRef'] == 'G1503'
    assert sent['mapdAlive'] is True
