# Hook Integration Points

## Quick Reference

- **28 hook points** implemented across catpilot (selfdrive + system)
- **Zero overhead** when no plugins registered (~50ns per call)
- **Fail-safe**: plugin exceptions revert to default value, log error, skip remaining plugins
- **Lazy loading**: each process loads plugins on first `hooks.run()` call

---

## Hook Points by Category

### Controls

| Hook | File | Plugin | Signature |
|------|------|--------|-----------|
| `controls.curvature_correction` | `selfdrive/controls/controlsd.py` | lane_centering | `(curvature, model_v2, v_ego, lane_changing) → curvature` |
| `controls.post_actuators` | `selfdrive/controls/controlsd.py` | (available) | `(None, actuators, CS, long_plan) → None` (void) |
| `car.cruise_initialized` | `selfdrive/car/card.py` | (available) | `(None, v_cruise_helper, CS_prev) → None` (void) |
| `car.register_interfaces` | `opendbc_repo/opendbc/car/car_helpers.py` | bmw_e9x_e8x | `(interfaces, platforms) → (interfaces, platforms)` |
| `car.panda_status` | `opendbc_repo/opendbc/car/car_helpers.py` | bmw_e9x_e8x | `(panda_states) → None` (void) |
| `torqued.allowed_cars` | `selfdrive/locationd/torqued.py` | (available) | `(allowed_cars) → allowed_cars` (one-shot at init) |

### Planning

| Hook | File | Plugin | Signature |
|------|------|--------|-----------|
| `planner.subscriptions` | `selfdrive/controls/plannerd.py` | speedlimitd | `(services_list) → services_list` (one-shot at init) |
| `planner.v_cruise` | `selfdrive/controls/lib/longitudinal_planner.py` | speedlimitd | `(v_cruise, v_ego, sm) → v_cruise` |
| `planner.accel_limits` | `selfdrive/controls/lib/longitudinal_planner.py` | (available) | `(accel_clip, v_ego, v_cruise, sm) → accel_clip` |

### Desire / Lane Change

| Hook | File | Plugin | Signature |
|------|------|--------|-----------|
| `desire.pre_lane_change` | `selfdrive/controls/lib/desire_helper.py` | (available) | `(None, desire_helper, carstate) → None` (void) |
| `desire.post_lane_change` | `selfdrive/controls/lib/desire_helper.py` | (available) | `(None, desire_helper, carstate, ...) → None` (void) |
| `desire.post_update` | `selfdrive/controls/lib/desire_helper.py` | (available) | `(desire, lane_change_state, sm) → desire` |

### Selfdrived

| Hook | File | Plugin | Signature |
|------|------|--------|-----------|
| `selfdrived.alert_registry` | `selfdrive/selfdrived/selfdrived.py` | (available) | `({}) → {EventName: Alert}` (one-shot at init) |
| `selfdrived.events` | `selfdrive/selfdrived/selfdrived.py` | phone_display | `([], CS, sm) → [EventName]` (100Hz) |

`selfdrived.alert_registry` is called once at selfdrived startup. Plugins return a dict of `{EventName: Alert}` entries that are merged into the alert registry.

`selfdrived.events` is called every 100Hz control tick. Plugins return a list of extra `EventName` values to add to the current event set.

### UI — Onroad

| Hook | File | Plugin | Signature |
|------|------|--------|-----------|
| `ui.render_overlay` | `selfdrive/ui/onroad/augmented_road_view.py` | speedlimitd | `(None, content_rect) → None` (void) |
| `ui.onroad_exp_button` | `selfdrive/ui/onroad/hud_renderer.py` | ui_mod | `(exp_button, ...) → exp_button` |
| `ui.hud_set_speed_override` | `selfdrive/ui/onroad/hud_renderer.py` | speedlimitd | `(None, max_color, set_speed_color, set_speed, is_metric) → None` (void) |
| `ui.hud_speed_color` | `selfdrive/ui/onroad/hud_renderer.py` | speedlimitd | `(speed_color) → speed_color` |

`ui.render_overlay` is called each frame inside scissor mode, after HUD render and before alert renderer. Render pipeline order:

1. Camera view (base)
2. model_renderer (path, lane lines, lead)
3. hud_renderer (MAX box, speed, exp button)
4. **ui.render_overlay** (plugin overlays)
5. alert_renderer (critical alerts, always topmost)
6. driver_state_renderer (driver monitoring)

### UI — State

| Hook | File | Plugin | Signature |
|------|------|--------|-----------|
| `ui.state_subscriptions` | `selfdrive/ui/ui_state.py` | speedlimitd | `(services_list) → services_list` (one-shot at init) |
| `ui.state_tick` | `selfdrive/ui/ui_state.py` | (available) | `(None, sm) → None` (void, every UI frame) |
| `ui.pre_end_drawing` | `system/ui/lib/application.py` | (available) | `(None) → None` (void, before EndDrawing) |
| `ui.post_end_drawing` | `system/ui/lib/application.py` | (available) | `(None) → None` (void, after EndDrawing) |

### UI — Layout Extension

| Hook | File | Plugin | Signature |
|------|------|--------|-----------|
| `ui.main_extend` | `selfdrive/ui/layouts/main.py` | (available) | `(None, main_layout) → None` (void) |
| `ui.home_extend` | `selfdrive/ui/layouts/home.py` | (available) | `(None, home_layout) → None` (void) |

### UI — Settings Extension

| Hook | File | Plugin | Signature |
|------|------|--------|-----------|
| `ui.connectivity_check` | `selfdrive/ui/layouts/sidebar.py` | network_settings | `(False) → bool` |
| `ui.network_settings_extend` | `selfdrive/ui/layouts/settings/settings.py` | network_settings | `(None, net_ui) → None` (void) |
| `ui.settings_extend` | `selfdrive/ui/layouts/settings/settings.py` | ui_mod | `(None, settings_layout) → None` (void) |
| `ui.software_settings_extend` | `selfdrive/ui/layouts/settings/software.py` | model_selector | `(None, software_layout) → None` (void) |

`ui.settings_extend` is called during `SettingsLayout.__init__`. The `ui_mod` plugin uses it to inject custom panels (Driving, Vehicle) into the settings sidebar.

### WebRTC

| Hook | File | Plugin | Signature |
|------|------|--------|-----------|
| `webrtc.session_factory` | `system/webrtc/webrtcd.py` | webrtc_stack | `(StreamSession) → SessionClass` (one-shot at init) |
| `webrtc.app_routes` | `system/webrtc/webrtcd.py` | phone_display | `([], aiohttp_app) → [RouteTableDef]` |
| `webrtc.session_started` | `system/webrtc/webrtcd.py` | phone_display | `(None, identifier) → None` (void) |
| `webrtc.session_ended` | `system/webrtc/webrtcd.py` | phone_display | `(None, identifier) → None` (void) |

`webrtc.session_factory` is called once at webrtcd startup. It receives the default `StreamSession` class and must return a class with the same constructor signature `(sdp, cameras, incoming_services, outgoing_services, debug_mode)` and public interface (`get_answer()`, `get_messaging_channel()`, `start()`, `stop()`). The `webrtc_stack` plugin uses this to substitute a portable aiortc-native session implementation that carries no teleoprtc dependency.

`webrtc.app_routes` is called once at webrtcd startup to register additional aiohttp routes. The phone_display plugin uses it to add WebRTC signaling and HUD data endpoints.

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
        return default      # No plugins → immediate return (~50ns)

    result = default
    for i, (priority, plugin_name, callback) in enumerate(callbacks):
        try:
            result = callback(result, *args, **kwargs)
        except Exception:
            skipped = [name for _, name, _ in callbacks[i + 1:]]
            msg = f"Plugin '{plugin_name}' hook '{hook_name}' failed, returning default"
            if skipped:
                msg += f" (skipping remaining plugins: {skipped})"
            cloudlog.exception(msg)
            return default  # Any error → revert to default, skip rest of chain
    return result
```

If a plugin throws an exception:
1. Error is logged to cloudlog with full traceback
2. Names of any skipped downstream plugins are included in the log
3. Default value is returned (as if no plugins were registered)
4. Openpilot continues operating normally — no crash, no control interruption
