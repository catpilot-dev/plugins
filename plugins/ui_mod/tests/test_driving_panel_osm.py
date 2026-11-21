"""Driving panel — the OSM Data Integration toggle, with mapd gone.

Source inspection: the panel needs raylib and a UI state, so it is checked by
what it reads and writes rather than rendered.
"""
import os

DRIVING_PANEL = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'driving_panel.py')


def _src():
  with open(DRIVING_PANEL) as f:
    return f.read()


def test_no_map_curve_lookahead_param():
  src = _src()
  assert 'MapdCurveLookahead' not in src
  assert 'Map Curve Look-ahead' not in src


def test_osm_toggle_title_and_param():
  src = _src()
  assert '"OSM Data Integration"' in src
  assert 'Mapd/OSM' not in src
  assert "read_plugin_param('speedlimitd', 'OsmDataIntegration')" in src
  assert "write_plugin_param('speedlimitd', 'OsmDataIntegration'" in src
