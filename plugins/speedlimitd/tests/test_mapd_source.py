"""Adapter tests — mapdOut becomes the road-context dict speedlimitd consumes."""
import os
import sys
from dataclasses import dataclass

import pytest

_SLD_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SLD_DIR not in sys.path:
  sys.path.insert(0, _SLD_DIR)

import mapd_source  # noqa: E402


@dataclass
class FakeMapdOut:
  """Stands in for a capnp MapdOut reader. Field names match the schema."""
  wayRef: str = ''
  wayName: str = ''
  roadName: str = ''
  speedLimit: float = 0.0
  lanes: int = 0
  highwayClass: str = 'unknown'
  wayId: int = 0
  tileLoaded: bool = True
  distanceFromWayCenter: float = 0.0
  waySelectionType: str = 'current'
  roadContext: str = 'city'


class TestHighwayClassName:
  def test_maps_to_underscore_osm_strings(self):
    # These feed URBAN_ONLY_TYPES membership and speed-table lookups in
    # infer_speed_from_road_type, which expect raw OSM values.
    assert mapd_source.highway_class_name('motorwayLink') == 'motorway_link'
    assert mapd_source.highway_class_name('livingStreet') == 'living_street'
    assert mapd_source.highway_class_name('trunkLink') == 'trunk_link'

  def test_simple_names_pass_through(self):
    assert mapd_source.highway_class_name('motorway') == 'motorway'
    assert mapd_source.highway_class_name('residential') == 'residential'

  def test_unknown_becomes_empty_string(self):
    # '' is what speedlimitd already treats as "no OSM classification".
    assert mapd_source.highway_class_name('unknown') == ''

  def test_unrecognised_value_is_empty_not_an_error(self):
    # A future mapd release adding an enum member must degrade to "no
    # classification", not crash the daemon.
    assert mapd_source.highway_class_name('someFutureClass') == ''


class TestTelemetry:
  def test_reports_fields_for_the_phase1_drive(self):
    out = FakeMapdOut(wayRef='S20', roadName='外环高速', speedLimit=27.8,
                      lanes=4, highwayClass='motorway', wayId=42,
                      tileLoaded=True, distanceFromWayCenter=1.5,
                      waySelectionType='current')
    t = mapd_source.telemetry_from_mapd(out, valid=True, our_way_ref='S20')
    assert t['mapdAlive'] is True
    assert t['mapdWayRef'] == 'S20'
    assert t['mapdWayId'] == 42
    assert t['mapdSpeedLimit'] == pytest.approx(100.1, abs=0.1)   # 27.8 m/s → km/h
    assert t['mapdHwClass'] == 'motorway'
    assert t['mapdLanes'] == 4
    assert t['mapdSelType'] == 'current'
    assert t['mapdTileLoaded'] is True
    assert t['mapdDistance'] == pytest.approx(1.5)
    assert t['mapdRefAgree'] is True

  def test_ref_disagreement_is_the_headline_number(self):
    out = FakeMapdOut(wayRef='S20', roadName='x')
    t = mapd_source.telemetry_from_mapd(out, valid=True, our_way_ref='G1503')
    assert t['mapdRefAgree'] is False

  def test_dead_mapd_reports_not_alive_with_neutral_values(self):
    t = mapd_source.telemetry_from_mapd(None, valid=False, our_way_ref='S20')
    assert t['mapdAlive'] is False
    assert t['mapdWayRef'] == ''
    assert t['mapdSpeedLimit'] == 0.0
    assert t['mapdRefAgree'] is False

  def test_always_returns_the_same_keys(self):
    # The rlog schema must not depend on mapd being up, or a drive with a dead
    # mapd would be unanalysable.
    live = mapd_source.telemetry_from_mapd(FakeMapdOut(wayRef='S20'), True, 'S20')
    dead = mapd_source.telemetry_from_mapd(None, False, 'S20')
    assert set(live) == set(dead)
