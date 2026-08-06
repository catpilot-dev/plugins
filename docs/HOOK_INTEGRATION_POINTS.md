# Hook Integration Points

## Quick Reference

- **24 hook call sites** in catpilot (selfdrive + system), plus plugin-dispatched hooks (see below)
- **Zero overhead** when no plugins registered (~50ns per call)
- **Fail-safe**: plugin exceptions revert to default value, log error, skip remaining plugins
- **Lazy loading**: each process loads plugins on first `hooks.run()` call

The "Plugin" column names the in-repo consumer(s); "(available)" means the
call site exists but no current plugin registers on it.

---

## Hook Points by Category

### Controls & Car

| Hook | File | Plugin | Signature |
|------|------|--------|-----------|
| `controls.lat_controller_init` | `selfdrive/controls/controlsd.py` | bmw_e9x_e8x | `(None, LaC, CP) → None` (void, one-shot at init) |
| `controls.curvature_correction` | `selfdrive/controls/controlsd.py` | lane_keeping | `(curvature, model_v2, v_ego, lane_changing, lat_delay=) → curvature` |
| `controls.post_actuators` | `selfdrive/controls/controlsd.py` | bmw_e9x_e8x | `(None, actuators, CS, long_plan) → None` (void) |
| `car.cruise_initialized` | `selfdrive/car/card.py` | bmw_e9x_e8x | `(None, v_cruise_helper, CS_prev) → None` (void) |
| `torqued.allowed_cars` | `selfdrive/locationd/torqued.py` | (available) | `(allowed_cars) → allowed_cars` (one-shot at import) |
| `desire.post_update` | `selfdrive/controls/lib/desire_helper.py` | (available) | `(desire, lane_change_state, lane_change_direction, carstate) → desire` |

### Planning

| Hook | File | Plugin | Signature |
|------|------|--------|-----------|
| `planner.subscriptions` | `selfdrive/controls/plannerd.py` | (available) | `(services_list) → services_list` (one-shot at init) |
| `planner.v_cruise` | `selfdrive/controls/lib/longitudinal_planner.py` | speedlimitd | `(v_cruise, v_ego, sm) → v_cruise` |
| `planner.accel_limits` | `selfdrive/controls/lib/longitudinal_planner.py` | (available) | `(accel_clip, v_ego, v_cruise, sm) → accel_clip` |

### Device Health

| Hook | File | Plugin | Signature |
|------|------|--------|-----------|
| `device.health_check` | `selfdrive/plugins/plugind.py` | all plugins | `(acc, sm=) → acc` (accumulator dict `{plugin: status}`) |

Every plugin registers here; plugind aggregates the returned dict into the
plugin health report.

### UI — Onroad

| Hook | File | Plugin | Signature |
|------|------|--------|-----------|
| `ui.render_overlay` | `selfdrive/ui/onroad/augmented_road_view.py` | speedlimitd, ui_mod, bmw_e9x_e8x, screen_capture | `(None, content_rect) → None` (void) |
| `ui.onroad_exp_button` | `selfdrive/ui/onroad/hud_renderer.py` | ui_mod | `(exp_button, button_size, wheel_icon_size) → exp_button` |
| `ui.hud_set_speed_override` | `selfdrive/ui/onroad/hud_renderer.py` | speedlimitd | `(None, max_color, set_speed_color, set_speed, is_metric) → override` |
| `ui.hud_speed_color` | `selfdrive/ui/onroad/hud_renderer.py` | (available) | `(speed_color) → speed_color` |

`ui.render_overlay` is called each frame inside scissor mode, after HUD
render and before the alert renderer. Render pipeline order:

1. Camera view (base)
2. model_renderer (path, lane lines, lead)
3. hud_renderer (MAX box, speed, exp button)
4. **ui.render_overlay** (plugin overlays)
5. alert_renderer (critical alerts, always topmost)
6. driver_state_renderer (driver monitoring)

### UI — State & Frame

| Hook | File | Plugin | Signature |
|------|------|--------|-----------|
| `ui.state_subscriptions` | `selfdrive/ui/ui_state.py` | ui_mod | `(services_list) → services_list` (one-shot at init) |
| `ui.state_tick` | `selfdrive/ui/ui_state.py` | ui_mod | `(None, sm) → None` (void, every UI frame) |
| `ui.pre_end_drawing` | `system/ui/lib/application.py` | screen_capture | `(None) → None` (void, before EndDrawing) |
| `ui.post_end_drawing` | `system/ui/lib/application.py` | screen_capture, ui_recorder | `(None) → None` (void, after EndDrawing) |

### UI — Layout Extension

| Hook | File | Plugin | Signature |
|------|------|--------|-----------|
| `ui.main_extend` | `selfdrive/ui/layouts/main.py` | ui_mod | `(None, main_layout) → None` (void) |
| `ui.home_extend` | `selfdrive/ui/layouts/home.py` | ui_mod | `(None, home_layout) → None` (void) |

### UI — Settings Extension

| Hook | File | Plugin | Signature |
|------|------|--------|-----------|
| `ui.connectivity_check` | `selfdrive/ui/layouts/sidebar.py` | (available) | `(False) → bool` |
| `ui.network_settings_extend` | `selfdrive/ui/layouts/settings/settings.py` | (available) | `(None, net_ui) → None` (void) |
| `ui.settings_extend` | `selfdrive/ui/layouts/settings/settings.py` | ui_mod | `(None, settings_layout) → None` (void) |
| `ui.software_settings_extend` | `selfdrive/ui/layouts/settings/software.py` | model_selector | `(None, software_layout) → None` (void) |

`ui.settings_extend` is called during `SettingsLayout.__init__`. The
`ui_mod` plugin uses it to inject custom panels (Driving, Vehicle, Plugins)
into the settings sidebar.

### Plugin-dispatched hooks

Hooks can also be dispatched *by plugins* through the same registry —
catpilot itself has no call site:

| Hook | Dispatched by | Provider | Signature |
|------|--------------|----------|-----------|
| `ui.vehicle_settings` | ui_mod (Vehicle panel) | bmw_e9x_e8x | `(items, CP) → items` |

This is the pattern for car-specific settings: ui_mod owns the panel, car
plugins contribute rows.

---

## Removed hooks (0.11 rebase)

Documented here so old manifests and examples aren't mistaken for current
API: `desire.pre_lane_change`, `desire.post_lane_change`,
`selfdrived.alert_registry`, `selfdrived.events`, `webrtc.session_factory`,
`webrtc.app_routes`, `webrtc.session_started`, `webrtc.session_ended`,
`car.register_interfaces`, `car.panda_status`. Car interfaces now register
by monkey-patching at plugin load (see `bmw_e9x_e8x`), not through a
car_helpers hook.

---

## Plugin Manifest Hook Declaration

```json
{
  "hooks": {
    "planner.v_cruise": {
      "module": "planner_hook",
      "function": "on_v_cruise",
      "priority": 50
    }
  }
}
```

- **module**: Python module within the plugin directory (relative to plugin root)
- **function**: Callable name within that module; receives `(current_value, *args)`
- **priority**: Lower number runs first (default 50); hooks chain in priority order

---

## Hook Performance

| Scenario | Latency |
|----------|---------|
| No plugins registered | ~50ns |
| 1 plugin callback | ~200ns |
| 3 plugin callbacks | ~500ns |
| 100Hz control loop budget | 10,000,000ns |

All scenarios are negligible vs the 10ms control loop cycle.

---

## Fail-Safe Behavior

```python
def run(self, hook_name: str, default, *args, **kwargs):
    self._ensure_loaded()  # Lazy per-process plugin discovery

    callbacks = self._hooks.get(hook_name)
    if not callbacks:
      return default       # No plugins → immediate return (~50ns)

    result = default
    for priority, plugin_name, callback in callbacks:
      try:
        result = callback(result, *args, **kwargs)
      except Exception:
        cloudlog.exception(f"Plugin '{plugin_name}' hook '{hook_name}' failed, returning default")
        return default     # Any error → revert to default, skip rest of chain
    return result
```

If a plugin throws an exception:
1. Error is logged to cloudlog with full traceback
2. Default value is returned (as if no plugins were registered)
3. Remaining plugins in the chain are skipped for that call
4. Openpilot continues operating normally — no crash, no control interruption
