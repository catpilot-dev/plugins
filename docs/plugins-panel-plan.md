# Plan: Plugins Panel in Settings UI

## Context
COD and the Settings Plugins panel are complementary:
- **COD** (Connect on Device) handles offline/parked tasks: plugin repo install, model downloads, route review, SSH keys
- **Settings Plugins panel** enables on-road parameter tuning directly on the C3 touchscreen — no need to pull out a phone

Both read/write the same `/data/params/d/` files and `.disabled` markers, so changes from either side are immediately visible to the other.

## Architecture

The Plugins panel follows the exact same pattern as stock `TogglesLayout`:
- A `Scroller` containing `ListItem` widgets built from `toggle_item()`, `multiple_button_item()`, and `button_item()` helpers
- Plugin discovery by scanning `/data/plugins/*/plugin.json` (same as COD's `_scan_plugins()`)
- Param I/O via raw file read/write to `/data/params/d/` (same as `params_helper.py`)
- Rebuild widget list on `show_event()` to reflect changes from COD or adb

## Files

### New: `overlays/selfdrive/ui/layouts/settings/plugins.py` (~250 lines)

```
class PluginsLayout(Widget):
    _scan_plugins()        # scan /data/plugins/*/plugin.json, filter device, detect .disabled
    _scan_and_build()      # build Scroller from plugin entries
    _build_plugin_items()  # per-plugin: header toggle + param items
    _build_param_item()    # per-param: bool→toggle_item, pills→multiple_button_item, string→button_item
    _toggle_plugin()       # create/remove .disabled, clear build hash, show reboot dialog
    _make_visibility_fn()  # dependsOn + requiresPlugin conditionals
    _sync_mapd_settings()  # regenerate MapdSettings JSON (ported from connect/handlers/params.py)
    _read_param()/_write_param()  # raw /data/params/d/ file I/O
```

**Plugin list rendering:**
- Each plugin: section header as a `toggle_item(name, description, enabled, callback=_toggle_plugin)` — locked plugins (c3_compat) have toggle disabled
- Below header: params with `desc` field rendered by type:
  - `bool` → `toggle_item(label, desc, state, callback)`
  - `pills` → `multiple_button_item(label, desc, buttons=[opt+suffix], selected_index, callback)`
  - `string` → `button_item(label, "EDIT", callback)` + `Keyboard` modal
- Param items only visible when plugin is enabled (`set_visible(lambda: entry.enabled)`)
- `dependsOn` / `requiresPlugin` conditionals dim + hide items

**Enable/disable flow:**
1. Toggle creates/removes `/data/plugins/{id}/.disabled`
2. Removes `/tmp/plugin_build_hash` to force rebuild
3. Shows `ConfirmDialog("Reboot required to apply changes.", "OK", cancel_text="")`
4. Sets `_needs_rebuild = True` → next frame rebuilds Scroller

**MapdSettings sync** (~25 lines, ported from `connect/handlers/params.py:96-125`):
- After writing `MapdSpeedLimitControlEnabled`, `MapdSpeedLimitOffsetPercent`, or `MapdCurveTargetLatAccel`, regenerate the `MapdSettings` JSON param

### Modify: `overlays/selfdrive/ui/layouts/settings/settings.py` (3 lines)

1. Add import: `from openpilot.selfdrive.ui.layouts.settings.plugins import PluginsLayout`
2. Add enum: `PLUGINS = 6` to `PanelType`
3. Add panel: `PanelType.PLUGINS: PanelInfo(tr_noop("Plugins"), PluginsLayout())`

Sidebar fits 7 panels: 7 × 110px = 770px, starting at y=300, total 1070px — within 1080px screen height.

## Key References
- Stock pattern: `/data/openpilot/selfdrive/ui/layouts/settings/toggles.py` — Scroller + toggle_item + multiple_button_item
- UI primitives: `/data/openpilot/system/ui/widgets/list_view.py` — toggle_item, button_item, multiple_button_item, ListItem, ToggleAction
- COD scanner: `connect/handlers/plugins.py:_scan_plugins()` — plugin discovery logic
- COD mapd sync: `connect/handlers/params.py:update_mapd_settings()` — JSON regeneration
- Confirm dialog: `/data/openpilot/system/ui/widgets/confirm_dialog.py` — ConfirmDialog, alert_dialog

## Verification
1. `.venv/bin/python3 -m pytest plugins/*/tests/ -q` — existing tests still pass
2. Deploy to C3, reboot
3. Settings sidebar shows "Plugins" as 7th panel
4. Each plugin listed with enable/disable toggle
5. Expand settings for enabled plugins — bool toggles, pills buttons, string editors work
6. Toggle a plugin off → reboot dialog shown → `.disabled` marker created
7. Edit a mapd param → `MapdSettings` JSON updated
