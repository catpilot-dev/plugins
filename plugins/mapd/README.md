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

**Shadow subscribers — ALL queues, not just carState (2026-08-18).** v2.0.6
through v2.2.0 hardcoded a slotless ("shadow") `carState` subscription: it
reads the msgq ring buffer without claiming a reader slot, so the writer can
overwrite the region mid-read and gomsgq panics on the torn size field
(pfeiferj/mapd#88). v2.3.0 made shadow a per-queue setting, and upstream's
defaults leave the OTHER queues (modelV2, gpsLocation, selfdriveState)
SLOTTED. That combination crashed loggerd on route 410 seg 4: mapd still
flaps on the carState torn-read panic, a Go panic exits without
deregistering its slotted readers, each respawn leaks a slot, and when a
queue's registration exceeds NUM_READERS=15 gomsgq ZEROES THE WHOLE READER
TABLE (`gomsgq/subscriber.go:31-34`) — every reader of that socket gets its
state wiped mid-read, and loggerd (which subscribes to everything) dies on
the C++ msgq assert with a truncated rlog and a red "logger error" alert.
So every `subscriber` entry in `mapd_defaults.json` is `shadow: true`:
mapd consumes ZERO reader slots anywhere, its panics stay contained to
itself, and plugind respawns it with speedlimitd degrading to vision-only
meanwhile. Do not set any of them slotted without re-auditing the slot
budget of that queue AND fixing the slot leak upstream.

## DORMANT — binary inactive, interface warm (2026-08-19)

`plugin.json` declares **no process**, so the Go binary never launches. Nothing
in the control path notices: speedlimitd has always driven control from the
offline tile reader (`osm_query.OsmTileReader`, "replaces mapd Go binary for
real-time queries"), and `mapdOut` fed only Phase-1 observation telemetry
(`mapd_source.telemetry_from_mapd`), which already has a `(None, False)` path
for an absent service (`speedlimitd.py:1285`). OSM road context therefore comes
straight from the tiles in `/data/media/0/osm/offline{,_hw}`.

The plugin stays **installed and enforced**, NOT `.disabled` — `.disabled` makes
`custom_capnp.py` revert slots 17–19 to the `CustomReservedN` stub and
`services.py` drop `mapdOut`, which would tear down the very interface we are
keeping warm. Slots, service, schemas, `mapd_source.py` and the binary on disk
all stay exactly as they are.

**Why dormant:** mapd v2.3.0 ships gomsgq v0.1.10, whose
`panic("Invalid Msgq message size")` is ungated — a shadow reader's *expected*
torn read kills the process (measured 2026-08-18: a restart every 1–2 min while
parked). Upstream fixed it in gomsgq v0.1.11 (mapd commit `fe45d10`, PR #133):
the panic is now gated to non-shadow readers and shadow readers get
`ShadowValid()`, turning a torn read into a re-sync. That commit is on mapd
`main` but **not in any release** (latest is v2.3.0, Aug 12).

**Re-activation, when pfeiferj cuts the release:**
1. Restore the process entry in `plugin.json`:
   `"processes": [{"name": "mapd", "module": "mapd_runner", "condition": "always_run"}]`
2. Bump `MAX_ALLOWED_VERSION` in `mapd_manager.py` to the new tag.
3. Keep every `subscriber` entry in `mapd_defaults.json` on `shadow: true` —
   v0.1.11 still has **no deregistration** (readers only `Reset()`), so slotted
   readers still leak a slot per death. Slotted mode additionally needs a
   per-queue budget audit: measured 2026-08-18 (offroad) `selfdriveState` was
   already **15/15 full**, modelV2 13/15, gpsLocation 9/15, carState 4/15.
4. Success criterion: one long-lived mapd PID across a whole drive (versus the
   1–2 min flap baseline above).

Nothing to do for the health hook: `hook.py` reads this manifest, so it reports
`status: ok, dormant: true` while no process is declared and re-arms the
"mapd process not running" warning the moment step 1 restores the entry. That
indirection exists so a permanently-expected warning never desensitises the
reader to a real post-re-activation failure.

## Settings

`mapd_defaults.json` (in this plugin dir) is the single declarative source of
what mapd is allowed to do — nothing: every control feature off, shadow
carState on. `mapd_runner.py` writes it to the **MapdSettings param** on every
start, which survives openpilot wiping `/data/params/d/` on boot.

Do NOT place a `/data/openpilot/mapd_defaults.json` — on mapd v2.3.0 the
custom-defaults file path panics at startup regardless of content
(`settings.go` `Default()` parses with gabs, so JSON numbers arrive as
float64, but the version check and migration cast expect uint64; version
absent hits a nil-interface assert in `Migrate()` instead). The param path
(`Load()`) compares float64 to float64 and is safe. `install.sh` removes any
stray copy of that file. Worth an upstream bug report.

## Key files

```
mapd/
  plugin.json        # Plugin manifest — slots 17-19, mapdOut service, mapd process, health hook
  mapd_defaults.json # Declarative settings — data-source-only, shadow carState
  mapd_manager.py    # Binary download, update, version management (auto-upgrade to the pin)
  mapd_runner.py     # Process entry point (settings param + ensure + execv) — spawned by plugind
  hook.py            # device.health_check reporting — invoked via the manifest hook
```
