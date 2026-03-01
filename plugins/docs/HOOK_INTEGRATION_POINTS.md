# Hook Integration Points

## Quick Reference

- **6 hook points** implemented across 4 files
- **Zero overhead** when no plugins registered (~50ns per call)
- **Fail-safe**: plugin exceptions return default value, never crash openpilot

---

## Implemented Hooks

### 1. `controls.curvature_correction`

**File**: `selfdrive/controls/controlsd.py`
**Plugin**: lane_centering
**Signature**: `callback(curvature, model_v2, v_ego, lane_changing) -> curvature`

Adjusts steering curvature for lane centering correction. Applied after model curvature calculation, before actuator command.

### 2. `planner.v_cruise`

**File**: `selfdrive/controls/lib/longitudinal_planner.py`
**Plugin**: speedlimitd
**Signature**: `callback(v_cruise, sm, v_ego) -> v_cruise`

Overrides cruise speed target. Used by speed limit middleware to enforce confirmed speed limits with speed-dependent offsets.

### 3. `planner.accel_limits`

**File**: `selfdrive/controls/lib/longitudinal_planner.py`
**Plugin**: (available, not currently used)
**Signature**: `callback(a_min, a_max, sm, v_ego) -> (a_min, a_max)`

Adjusts acceleration limits for custom braking/acceleration profiles.

### 4. `desire.post_update`

**File**: `selfdrive/controls/lib/desire_helper.py`
**Plugin**: (available, not currently used)
**Signature**: `callback(desire, lane_change_state, sm) -> desire`

Post-processes lane change desire for extensions like consecutive lane changes.

### 5. `car.register_interfaces`

**File**: `opendbc_repo/opendbc/car/car_helpers.py`
**Plugin**: bmw_e9x_e8x
**Signature**: `callback(interfaces, platforms) -> (interfaces, platforms)`

Registers car platforms into openpilot's dynamic interface loading system. BMW plugin injects E82/E90 platforms and their CarInterface implementations.

### 6. `car.panda_status`

**File**: `opendbc_repo/opendbc/car/car_helpers.py`
**Plugin**: bmw_e9x_e8x
**Signature**: `callback(panda_states) -> None`

Monitors panda safety model status. BMW plugin detects ELM327 fallback and logs warnings.

---

## Planned Hooks

### `ui.render_overlay`

**File**: `selfdrive/ui/layouts/main.py`
**Signature**: `callback(None, screen, ui_state) -> None`

Custom HUD overlays (speed limit sign, DCC mode display, debug info).

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

- **module**: Python module within the plugin directory
- **function**: Callable that receives `(current_value, *args)`
- **priority**: Lower number runs first (default 50)

---

## Fail-Safe Behavior

```python
def run(self, hook_name, default, *args, **kwargs):
    callbacks = self._hooks.get(hook_name)
    if not callbacks:
        return default          # No plugins → immediate return
    result = default
    for priority, plugin_name, callback in callbacks:
        try:
            result = callback(result, *args, **kwargs)
        except Exception:
            cloudlog.exception(f"Plugin '{plugin_name}' hook '{hook_name}' failed")
            return default      # Any error → revert to default
    return result
```

If a plugin throws an exception:
1. Error is logged to cloudlog
2. Default value is returned (as if no plugins were registered)
3. Openpilot continues operating normally
4. No crash, no control interruption
