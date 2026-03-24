"""Hook handlers for UI customizations.

Registers Driving, Vehicle, and Plugins settings panels, replaces PrimeWidget
with DriveStatsWidget, wires up ecosystem update badge, and provides the
branded ExpButton for onroad HUD.
"""

_drive_tracker = None
_driving_key = None
_plugins_key = None


def on_settings_extend(default, settings):
  from driving_panel import DrivingLayout
  from vehicle_panel import VehicleLayout
  from plugins_panel import PluginsLayout

  global _driving_key, _plugins_key
  _driving_key = settings.add_panel("Driving", DrivingLayout())
  settings.add_panel(
    "Vehicle", VehicleLayout(),
    enabled_fn=lambda: __import__(
      'openpilot.selfdrive.ui.ui_state', fromlist=['ui_state']
    ).ui_state.CP is not None,
  )
  _plugins_key = settings.add_panel("Plugins", PluginsLayout())


def on_home_extend(default, home):
  from drive_stats import DriveStatsWidget
  from route_map_widget import RouteMapWidget
  stats = DriveStatsWidget()
  home.set_left_widget(RouteMapWidget(stats))
  home.set_right_widget(stats)

  def _eco_count():
    try:
      from openpilot.selfdrive.plugins.update_checker import get_update_status
      return sum(1 for v in get_update_status().values() if v)
    except Exception:
      return 0

  home.set_eco_update_checker(_eco_count)


def on_main_extend(default, main):
  if _driving_key is not None:
    main.set_default_settings_panel(_driving_key)
  if _plugins_key is not None:
    main.get_home_layout().set_plugins_callback(
      lambda: main.open_settings(_plugins_key))


def on_state_tick(default, sm):
  global _drive_tracker
  if _drive_tracker is None:
    try:
      from drive_tracker import DriveTracker
      _drive_tracker = DriveTracker()
    except Exception:
      _drive_tracker = False
  if _drive_tracker:
    _drive_tracker.tick(sm)


def on_state_subscriptions(services):
  if 'pluginBusLog' not in services:
    services.append('pluginBusLog')
  return services


def on_exp_button(default, size, icon_size):
  from exp_button import ExpButton
  return ExpButton(size, icon_size)
