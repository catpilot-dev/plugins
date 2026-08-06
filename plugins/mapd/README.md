# Mapd — OpenStreetMap Data

**Status: disabled — not currently running.**

## What it's for

`mapd` is a small Go binary from [pfeiferj/mapd](https://github.com/pfeiferj/mapd)
that reads GPS and publishes OpenStreetMap data — speed limits, road names,
map-based curve speeds, and road classification (freeway vs. city). This
plugin's job is to manage that binary: download it, keep it pinned to a
known-good version, and start/stop it.

Device filter: `tici`, `tizi`, `mici` — even if re-enabled, this only runs
on those comma-three-family hardware variants.

## Why it's off

As of the last change to this plugin (`plugin.json` has no `processes` and
no `hooks` registered, and `panel: false`), mapd doesn't run at all and
doesn't appear in the Plugins panel — this isn't a Settings toggle you can
flip, the plugin manifest itself has been emptied out. The code
(`mapd_manager.py`, `mapd_runner.py`, `hook.py`) is still in the repo, but
nothing in the plugin framework currently calls it.

Two reasons this was turned off:

- **A crash bug in mapd itself.** mapd v2.0.6 subscribes to `carState` using
  a "shadow" mode that reads the shared-memory ring buffer without claiming
  a reader slot. With no backpressure, the writer can overwrite data mapd
  hasn't read yet, which trips an assert in `msgq.cc` and crashes the
  process (and fragments the route). This is hardcoded in the upstream
  binary and can't be configured away. The fix here was pinning to v2.0.5
  (the last version without shadow mode) — see `MAX_ALLOWED_VERSION` in
  `mapd_manager.py` — but the plugin was disabled outright rather than
  relying on the pin alone.
- **Limited value on the roads this is actually driven on.** OSM coverage
  in China (speed limits, road classification, curve geometry) is sparse
  and often wrong. `speedlimitd` gets equivalent or better results there
  from vision-based road-type inference (lane count + urban/highway
  tables), so it doesn't depend on mapd to do its job.

`speedlimitd` does **not** consume mapd's output — it reads the offline OSM
tiles directly (`osm_query.py`) and infers speed from vision. Older
`mapdOut` / `suggestedSpeed` references survive only in speedlimitd's
docstrings; no live code path uses them. So mapd being off costs
speedlimitd nothing today. (mapd and speedlimitd do share a plugin-param
store for tile configuration, but not mapd's speed suggestions.)

## If you want to re-enable it

This requires editing `plugin.json` to restore a `processes` entry pointing
at `mapd_runner.py` (and re-adding the cereal slots for `mapdOut` /
`mapdExtendedOut` / `mapdIn` if you want the messages logged), then
reinstalling. Expect the crash risk described above unless you also keep
the binary pinned to v2.0.5 or older.

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

## Key files

```
mapd/
  plugin.json        # Plugin manifest — currently empty hooks/processes (disabled)
  mapd_manager.py     # Binary download, update, version management (manual use)
  mapd_runner.py       # Process entry point (ensure + execv) — not currently invoked
  hook.py               # device.health_check reporting — not currently invoked
```
