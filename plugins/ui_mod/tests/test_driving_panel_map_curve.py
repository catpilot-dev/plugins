"""Driving panel — the Map Curve Look-ahead (G/S) toggle.

Source inspection: the panel needs raylib and a UI state, so it is checked by
what it reads and writes rather than rendered.
"""
import os

DRIVING_PANEL = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'driving_panel.py')


def test_toggle_writes_param():
  with open(DRIVING_PANEL) as f:
    src = f.read()
  assert "read_plugin_param('speedlimitd', 'MapdCurveLookahead')" in src
  assert "write_plugin_param('speedlimitd', 'MapdCurveLookahead'" in src
  assert '"Map Curve Look-ahead (G/S)"' in src
