# Plan: Plugins Panel in Settings UI

## Context

COD and the Settings Plugins panel are complementary:
- **COD** (Connect on Device) handles offline/parked tasks: plugin repo install, model downloads, route review, SSH keys. Updated in v0.10.2 with process status badges, author info, dynamic dependency labels.
- **Settings Plugins panel** enables on-road parameter tuning directly on the C3 touchscreen — no need to pull out a phone or open a browser.

Both read/write the same `/data/params/d/` files and `.disabled` markers, so changes from either side are immediately visible to the other.

## Architecture

The Plugins panel follows the exact same pattern as stock `TogglesLayout`:
- A `Scroller` containing `ListItem` widgets built from `toggle_item()`, `multiple_button_item()`, and `button_item()` helpers
- Plugin discovery by scanning `/data/plugins/*/plugin.json` (same as COD's `_scan_plugins()`)
- Param I/O via raw file read/write to `/data/params/d/` (same as `params_helper.py`)
- Rebuild widget list on `show_event()` to reflect changes from COD or adb

## Stock UI Reference

### Settings Layout (`selfdrive/ui/layouts/settings/settings.py`)
- `PanelType` IntEnum: DEVICE=0, NETWORK=1, TOGGLES=2, SOFTWARE=3, FIREHOSE=4, DEVELOPER=5
- `_panels` dict maps `PanelType` → `PanelInfo(name, Widget instance)`
- Sidebar: 7 × 110px nav buttons starting at y=300, total 1070px — fits within 1080px screen
- Each panel is a `Widget` subclass with `_render(rect)`, `show_event()`, `hide_event()`

### Toggles Layout (`selfdrive/ui/layouts/settings/toggles.py`)
Key patterns to follow:
- Uses `Params()` for reading/writing — we CANNOT use this (plugin params not in params_keys.h)
- Creates `toggle_item()` and `multiple_button_item()` widgets in `__init__`
- Stores widgets in a dict, feeds them to `Scroller(list(widgets.values()))`
- `show_event()` refreshes toggle states from params
- `_render(rect)` just calls `self._scroller.render(rect)`

### Widget Primitives (`system/ui/widgets/list_view.py`)
Factory functions we'll use:
- `toggle_item(title, description, initial_state, callback, icon, enabled)` → `ListItem` with `ToggleAction`
- `multiple_button_item(title, description, buttons, selected_index, button_width, callback, icon)` → `ListItem` with `MultipleButtonAction`
- `button_item(title, button_text, description, callback, enabled)` → `ListItem` with `ButtonAction`
- `text_item(title, value, description, callback, enabled)` → `ListItem` with `TextAction`

Title/description/enabled accept callables for live updates.

### Confirm Dialog (`system/ui/widgets/confirm_dialog.py`)
```python
ConfirmDialog(text, confirm_text, cancel_text=None, rich=False)
gui_app.set_modal_overlay(dlg, callback=confirm_callback)
# callback receives DialogResult.CONFIRM or DialogResult.CANCEL
```

## Files

### New: `overlays/selfdrive/ui/layouts/settings/plugins.py` (~250 lines)

```python
class PluginsLayout(Widget):
    # Plugin discovery
    _scan_plugins()        # scan /data/plugins/*/plugin.json, filter device_filter, detect .disabled
    _scan_and_build()      # build Scroller from plugin entries

    # Widget building
    _build_plugin_items()  # per-plugin: header toggle + param items
    _build_param_item()    # per-param: bool→toggle_item, pills→multiple_button_item

    # Actions
    _toggle_plugin()       # create/remove .disabled, clear build hash, show reboot dialog
    _make_visibility_fn()  # dependsOn + requiresPlugin conditionals

    # Param I/O (raw file, NOT openpilot Params)
    _read_param(key)       # read /data/params/d/{key}
    _write_param(key, val) # write /data/params/d/{key}

    # MapdSettings sync
    _sync_mapd_settings()  # regenerate MapdSettings JSON after mapd param changes
```

**Plugin list rendering:**
- Each plugin: section header as a `toggle_item(name, description, enabled, callback=_toggle_plugin)`
  - Locked plugins (c3_compat on C3) have toggle disabled via `enabled=False`
  - Plugins without visible params (no `desc` field) show as toggle-only, no expand
- Below header: params with `desc` field rendered by type:
  - `bool` → `toggle_item(label, desc, state, callback=_on_param_toggle)`
  - `pills` → `multiple_button_item(label, desc, buttons=[opt+suffix], selected_index, callback=_on_param_pills)`
  - `string` → `button_item(label, "EDIT", callback)` (stretch goal — needs Keyboard widget)
- Param items only visible when plugin is enabled: `set_visible(lambda: entry.enabled)`
- `dependsOn` conditionals: param toggle/pills disabled when parent param is False
  - Use `enabled=lambda: _read_param(depends_on) == "1"`
- `requiresPlugin` conditionals: param disabled when dependency plugin is not enabled
  - Use `enabled=lambda: not os.path.exists(f'/data/plugins/{req_plugin}/.disabled')`

**Sort order** (matching COD):
```python
ORDER = {'model_selector': -3, 'lane_centering': -2, 'speedlimitd': -1, 'mapd': 0, 'bmw_e9x_e8x': 1, 'c3_compat': 2}
```

**Enable/disable flow:**
1. Toggle creates/removes `/data/plugins/{id}/.disabled`
2. Removes `/tmp/plugin_build_hash` to force rebuild
3. Shows `ConfirmDialog("Reboot required to apply changes.", "OK", cancel_text="")`
4. Sets `_needs_rebuild = True` → `show_event()` rebuilds Scroller

**MapdSettings sync** (~25 lines, ported from `connect/handlers/params.py:96-125`):
- After writing `MapdSpeedLimitControlEnabled`, `MapdSpeedLimitOffsetPercent`, or `MapdCurveTargetLatAccel`, regenerate the `MapdSettings` JSON param
- Same logic as COD: read individual params, convert to mapd JSON format, write to `/data/params/d/MapdSettings`

**Param I/O** (raw file, NOT openpilot `Params` class):
```python
def _read_param(self, key: str) -> str:
    try:
        with open(f'/data/params/d/{key}') as f:
            return f.read().strip()
    except (FileNotFoundError, OSError):
        return ''

def _write_param(self, key: str, value: str):
    with open(f'/data/params/d/{key}', 'w') as f:
        f.write(value)
```

### Modify: `overlays/selfdrive/ui/layouts/settings/settings.py` (3 lines)

1. Add import: `from openpilot.selfdrive.ui.layouts.settings.plugins import PluginsLayout`
2. Add enum: `PLUGINS = 6` to `PanelType`
3. Add panel: `PanelType.PLUGINS: PanelInfo(tr_noop("Plugins"), PluginsLayout())`

## Current Plugin Params (reference)

**speedlimitd** (5 visible params):
| Key | Type | Default | dependsOn | requiresPlugin |
|-----|------|---------|-----------|----------------|
| ShowSpeedLimitSign | bool | true | — | — |
| ShowCurvatureSpeed | bool | true | — | — |
| MapdSpeedLimitControlEnabled | bool | false | — | mapd |
| MapdSpeedLimitOffsetPercent | pills [0,5,10,15]% | 10 | MapdSpeedLimitControlEnabled | — |
| MapdCurveTargetLatAccel | pills [1.5,2.0,2.5,3.0] | 1 | MapdSpeedLimitControlEnabled | — |

**Other plugins**: no visible params (no `desc` field in their param definitions).

## Implementation Order

1. Create `plugins.py` with `_scan_plugins()`, `_read_param()`, `_write_param()`
2. Build plugin toggle headers (enable/disable with reboot dialog)
3. Build bool param items with `toggle_item()`
4. Build pills param items with `multiple_button_item()`
5. Add `dependsOn` / `requiresPlugin` conditional logic
6. Add `_sync_mapd_settings()` for mapd param writes
7. Modify `settings.py` to add Plugins panel to sidebar
8. Test on C3

## Verification

1. `PYTHONPATH=. uv run pytest plugins/*/tests/ -q` — existing tests still pass
2. Deploy to C3, reboot
3. Settings sidebar shows "Plugins" as 7th panel
4. Each plugin listed with enable/disable toggle
5. Expand speedlimitd — bool toggles and pills buttons work
6. Toggle a plugin off → reboot dialog shown → `.disabled` marker created
7. Edit a mapd param → `MapdSettings` JSON updated
8. COD reflects changes made from Settings panel (and vice versa)
