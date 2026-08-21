# Mapd — OpenStreetMap Data

**Status: DORMANT again since 2026-08-21 — v2.3.1 crashes too, in a new place.**

The binary does not run: `plugin.json` declares no process. The cereal interface
is deliberately kept warm — slots 17-19 and the `mapdOut` service stay injected,
so the plugin remains installed and `.enforced` and is NOT `.disabled`.

Nothing in the control path notices. speedlimitd has always derived every control
decision from the offline tile reader (`osm_query.OsmTileReader`); it consumes
`mapdOut` through `mapd_source.telemetry_from_mapd` purely as Phase-1 observation,
logged into `speedLimitState` for comparison. Actuation is byte-identical with mapd
running or absent, and an absent service degrades to vision-only rather than failing.

Tiles are downloaded by COD's web UI into `/data/media/0/osm/offline/`, which is
exactly where mapd reads them.

**Phase 2 — cutting control over to mapd (`mapd_source.result_from_mapd`, then
deleting `osm_query.py`, `osm_reader.capnp`, `generate_hw_tiles.py` and the
margin-release machinery) — is NOT started.** It is gated on a Phase 1 drive that
has never happened: `mapdRefAgree` rate, mapd uptime, S20 coverage delta and
selType distribution are all still unmeasured.

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
| MapdVersion | string | v2.3.1 | Version tracked/pinned by `mapd_manager.py` |

The v2.3.1 pin is deliberate, for two independent reasons.

**Schema coupling.** Cap'n Proto is additive, so a newer binary publishing into
our `cereal/slot17-19.capnp` silently drops any field we have not declared —
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

## DORMANT — v2.3.1 activation attempt, 2026-08-21

Activated and reverted the same day. **v2.3.1 fixes the bug we waited for and
introduces a different one.**

Observed on the C3: mapd `SIGBUS`es reliably ~80 s after start (≈1600 messages at
20 Hz — consistent with a ring-buffer wrap), and plugind respawns it every ~85 s.

```
[signal SIGBUS: bus error code=0x2 addr=0x7f46e20e20]
gomsgq@v0.1.11/publisher.go:65   MsgqPublisher.Send
mapd/cereal/publisher.go:119     autoPublishLoop
                                 StartAutoPublish
```

That is the **publisher**, not a subscriber — it is v2.3.1's new "publishing on its
own thread at a constant 20 Hz" feature faulting while writing into the msgq shared
memory. The v0.1.10 bug we went dormant over (`panic("Invalid Msgq message size")`
in a *shadow reader*) is genuinely fixed by v0.1.11; this is unrelated and new.

**What did NOT recur: the loggerd kill chain.** That needed leaked reader *slots*,
and every subscriber is `shadow: true`, so mapd consumes zero slots and its deaths
leak nothing. loggerd held steady across the whole flap. Keep all five shadow — the
mitigation is load-bearing independently of either crash.

Confirmed good in the attempt, worth keeping: the pin, the binary
(sha256 matches the v2.3.1 asset), the adopted `slot17` fields, and the publish rate
— measured **20.02 Hz**, up from 19.7 on v2.3.0, so the strict-rate work does what it
claims for the ~80 s it survives.

**Debugging note:** plugind opens the process log with `'w'`, truncating it on every
respawn, so a crash trace is erased ~85 s later. To catch one, poll
`/tmp/plugin_logs/plugin_mapd.log` every second and copy it the moment it is
non-empty.

### Re-activation, when a release fixes the publisher fault
1. Restore the process entry in `plugins/mapd/plugin.json`:
   `"processes": [{"name": "mapd", "module": "mapd_runner", "condition": "always_run"}]`
2. Bump `MAX_ALLOWED_VERSION` in `mapd_manager.py`, paired with a slot schema diff.
3. Keep every `subscriber` entry on `shadow: true` (see above).
4. Deploy, then **reboot** — plugind freezes manifests at discovery, so a
   `processes` change needs plugind restarted, not just `.needs_restart`.
5. Success criterion: one long-lived mapd PID across a whole drive. Watch the
   spawn cadence in swaglog (`plugin process 'mapd' spawned`), which is how the
   ~85 s flap was measured.

### History: why it was dormant before this attempt (2026-08-19)


Dormant from 2026-08-19 to 2026-08-21. mapd v2.3.0 shipped gomsgq **v0.1.10**, whose
ungated `panic("Invalid Msgq message size")` killed the process on a shadow reader's
*expected* torn read — measured as a restart every 1-2 min while parked. Worse, a Go
panic exits without deregistering, so each respawn leaked a reader slot, and once a
queue passed `NUM_READERS=15` gomsgq zeroed the whole reader table and **killed
loggerd** (route 410 seg 4, truncated rlog). The plugin stayed installed and
`.enforced` throughout, declaring no process, because `.disabled` would have reverted
slots 17-19 to the `CustomReservedN` stub and dropped the `mapdOut` service — tearing
down the very interface we were keeping warm.

**v2.3.1 (2026-08-21) pins gomsgq v0.1.11** (mapd `497a4f4`, merge `fe45d10`,
PR #133): the panic is gated to non-shadow readers and shadow readers get
`ShadowValid()`, which turns a torn read into a re-sync. That fix is real and
verified — it is simply not sufficient, because v2.3.1 faults in the publisher
instead (see above).

**Every `subscriber` entry stays `shadow: true`.** v0.1.11 fixed the panic, not the
leak — readers still only `Reset()`, with no deregistration, so a slotted reader
still leaks a slot per death. Upstream's own defaults leave four of the five queues
slotted, which is exactly why we override all five. Going slotted would additionally
need a per-queue budget audit: measured 2026-08-18 offroad, `selfdriveState` was
already **15/15 full**, modelV2 13/15, gpsLocation 9/15, carState 4/15.

**Schema delta adopted with the pin.** v2.3.1 added `loopRateAverage @4` and
`loopRateMin @5` to `MapdExtendedOut`; both are declared in `cereal/slot17.capnp`.
`MapdIn` and `MapdOut` were unchanged. v2.3.1 also moved publishing to its own
thread for a constant 20 Hz rate (we measured 19.7 Hz on v2.3.0) and made large
main-loop performance improvements — `loopRateMin` is the field that shows whether
those hold up on a C3.

**Success criterion, still unverified on the road:** one long-lived mapd PID across a
whole drive, versus the 1-2 min flap baseline. The Phase 1 telemetry drive
(`mapdRefAgree`, mapd uptime, S20 coverage delta, selType distribution) has never
happened — mapd went dormant before it could.

`hook.py` needs no edit either way: it reads this manifest, so it reported
`status: ok, dormant: true` while no process was declared and re-armed the
"mapd process not running" warning the moment the entry came back. That indirection
exists so a permanently-expected warning never desensitises the reader to a real
failure.

An upstream release is watched automatically — see `.github/scripts/mapd_watch.py`,
which files a GitHub Issue with the schema and crash-fix verdicts precomputed.

## Settings

`mapd_defaults.json` (in this plugin dir) is the single declarative source of
what mapd is allowed to do — nothing: every control feature off, shadow
carState on. `mapd_runner.py` writes it to the **MapdSettings param** on every
start, which survives openpilot wiping `/data/params/d/` on boot.

Do NOT place a `/data/openpilot/mapd_defaults.json` — on mapd v2.3.0/v2.3.1 the
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
