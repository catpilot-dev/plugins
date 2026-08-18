# Mapd — OpenStreetMap Data

**Status: enabled — mapd is the sole provider of OSM road context.**

speedlimitd consumes the `mapdOut` message; it no longer reads map tiles
itself. Tiles are downloaded by COD's web UI into `/data/media/0/osm/offline/`,
which is exactly where mapd reads them.

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
| MapdVersion | string | v2.3.0 | Version tracked/pinned by `mapd_manager.py` |

The v2.3.0 pin is deliberate, for two independent reasons.

**Schema coupling.** Cap'n Proto is additive, so a newer binary publishing into
our `cereal/slot19.capnp` silently drops any field we have not declared —
`highwayClass` would read as `unknown` and speedlimitd would mis-classify every
road, with no error anywhere. Bumping the pin requires diffing mapd's
`cereal/custom/custom.capnp` against our slot files first.

**Shadow subscribers.** v2.0.6 through v2.2.0 hardcoded a slotless ("shadow")
`carState` subscription: it reads the msgq ring buffer without claiming a
reader slot, so the writer can overwrite the region mid-read and gomsgq panics
on the torn size field (pfeiferj/mapd#88). v2.3.0 made shadow a per-queue
setting. We keep upstream's default of shadow-on for carState — it consumes no
reader slot — and rely on plugind respawning mapd, with speedlimitd degrading
to vision-only meanwhile.

## Key files

```
mapd/
  plugin.json      # Plugin manifest — slots 17-19, mapdOut service, mapd process, health hook
  mapd_manager.py  # Binary download, update, version management (manual use)
  mapd_runner.py   # Process entry point (ensure + execv) — spawned by plugind
  hook.py          # device.health_check reporting — invoked via the manifest hook
```
