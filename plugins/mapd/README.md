# Mapd — OpenStreetMap Data

**Status: disabled — not currently running.**

## What it's for

`mapd` is a small Go binary from [pfeiferj/mapd](https://github.com/pfeiferj/mapd)
that reads GPS and publishes OpenStreetMap data — speed limits, road names,
map-based curve speeds, and road classification (freeway vs. city). This
plugin's job is to manage that binary: download it, keep it pinned to a
known-good version, and start/stop it.

Device filter: `tici`, `tizi`, `mici` — the comma three, 3X and four, so the
filter excludes nothing in practice.

## Binary management (when running)

The binary lives at `/data/media/0/osm/mapd`, outside the plugin/openpilot
repos, so it survives updates. `mapd_manager.py` can be run by hand:

```bash
python mapd_manager.py check    # is a newer version available?
python mapd_manager.py update   # backup, download, swap, restart
python mapd_manager.py ensure   # download only if missing
```

Update flow: backup current binary → download new one → stop the daemon →
atomically replace the binary → update the version param → restart. Old
binaries are kept in `/data/media/0/osm/mapd_backups/`.

## Params

| Param | Type | Default | Description |
|-------|------|---------|-------------|
| MapdVersion | string | v2.0.5 | Version tracked/pinned by `mapd_manager.py` |

The v2.0.5 pin is deliberate: v2.0.6 subscribes to `carState` in a "shadow"
mode that reads the shared-memory ring buffer without claiming a reader slot,
so the writer can overwrite unread data and crash the process. It is
hardcoded upstream and can't be configured away — see `MAX_ALLOWED_VERSION`
in `mapd_manager.py` before changing this.

## Key files

```
mapd/
  plugin.json      # Plugin manifest — currently empty hooks/processes (disabled)
  mapd_manager.py  # Binary download, update, version management (manual use)
  mapd_runner.py   # Process entry point (ensure + execv) — not currently invoked
  hook.py          # device.health_check reporting — not currently invoked
```
