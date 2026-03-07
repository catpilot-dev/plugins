"""Vehicle panel for Settings UI — vehicle-specific parameters.

Only visible when a car is fingerprinted (ui_state.CP is not None).
Items are populated by car plugins via the 'ui.vehicle_settings' hook.

Hook signature:
  callback(items: list, CP: CarParams) -> items
  Plugins append toggle_item/multiple_button_item widgets to the list.
"""
import os

from openpilot.selfdrive.plugins.hooks import hooks
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.scroller_tici import Scroller


class VehicleLayout(Widget):
  def __init__(self):
    super().__init__()
    self._scroller = None
    self._needs_rebuild = True

  def _build_scroller(self):
    items = []

    CP = ui_state.CP
    if CP is not None:
      items = hooks.run('ui.vehicle_settings', items, CP)

    self._scroller = Scroller(items, line_separator=True, spacing=0) if items else None

  def show_event(self):
    super().show_event()
    self._needs_rebuild = True

  def _render(self, rect):
    if self._needs_rebuild:
      self._build_scroller()
      self._needs_rebuild = False

    if self._scroller:
      self._scroller.render(rect)
