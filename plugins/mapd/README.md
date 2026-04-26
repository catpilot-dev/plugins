# mapd — OpenStreetMap Data Plugin

**Type**: process
**Condition**: always_run
**Device filter**: tici, tizi, mici
**Upstream**: [pfeiferj/mapd](https://github.com/pfeiferj/mapd)
**Status**: ⚠️ **Disabled by default** — see Known Issues below

## What it does

Manages the mapd Go binary that provides OpenStreetMap data to openpilot:

- Speed limits (maxspeed tags)
- Road names and reference codes
- Curve speeds (map-based and vision-based)
- Hazards and advisory speeds
- Road context (freeway/city)
- Offline map tile management

## Known Issues

### gomsgq shadow subscription (v2.0.6)

mapd v2.0.6 uses a "shadow" subscription for `carState` via gomsgq — it reads
the msgq ring buffer **without claiming a reader slot**. This means the writer
has no backpressure from mapd's reader, and can overwrite data mapd hasn't
consumed yet. This triggers `assert()` failures in `msgq.cc` → SIGABRT →
process crashes and fragmented routes.

This is **hardcoded** in the Go binary (`NewSubscriber("carState", ..., shadow=true)`)
and cannot be changed via settings or `MAPD_SETTINGS` param. The shadow mode was
introduced because stock openpilot uses 14/15 carState reader slots, leaving no
room for mapd. catpilot only uses 11/15 slots, so a regular subscription would work.

**Workaround**: mapd is disabled by default. Users can enable it in the Plugins
panel, but may experience crashes on C3 devices. speedlimitd works independently
using vision lane-count inference — no mapd dependency.

**Fix**: Awaiting upstream mapd release with configurable shadow mode, or build
from source with `shadow=false`.

### Limited value in China

OSM coverage in China is sparse — speed limits, road classifications, and curve
geometry are largely missing or inaccurate. speedlimitd's vision-based road-type
inference (lane count + urban speed tables) provides equivalent or better results
for Chinese roads.

## How it works

1. `mapd_runner.py` is spawned by the plugin process manager
2. It calls `ensure_binary()` to download the mapd binary if missing
3. Then `os.execv()` replaces the Python process with the native Go binary
4. The binary reads GPS from cereal, queries OSM tiles, publishes `mapdOut` at 20Hz

## Binary management

The binary lives at `/data/media/0/osm/mapd` (outside the repo, persists across updates).

```bash
# Check for updates
python mapd_manager.py check

# Download/update to latest
python mapd_manager.py update

# Ensure binary exists (download if missing)
python mapd_manager.py ensure
```

Update flow: backup → download to temp → stop daemon → atomic replace → update version → restart.

## Offline maps

Downloaded to `/data/media/0/osm/offline/`. Use `mapd interactive` TUI to download regions.

## Cereal messages

| Message | Direction | Frequency | Description |
|---------|-----------|-----------|-------------|
| mapdOut | publish | 20 Hz | Speed limits, road info, curve speeds |
| mapdExtendedOut | publish | 20 Hz | Download progress, settings, path points |
| mapdIn | subscribe | event | Download triggers, settings changes |

## Key files

```
mapd/
  plugin.json        # Plugin manifest
  mapd_manager.py    # Binary download, update, version management
  mapd_runner.py     # Process entry point (ensure + execv)
```

## Params

| Param | Type | Default | Description |
|-------|------|---------|-------------|
| MapdVersion | string | v2.0.6 | Currently installed mapd version |
