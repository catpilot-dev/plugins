# Mapd — OpenStreetMap Data

**Status: RE-ACTIVATED on v2.4.0 (2026-11-09); first drives clean.** `plugin.json`
declares the process again, and `mapdOut` is now registered at mapd's own 2 MB
queue size. That size mismatch is the suspected cause of the v2.3.1 SIGBUS (see
[the root cause](#root-cause-mapdout-queue-size-mismatch)). Routes 532/534
(2026-11-09, 39 min): `/dev/shm/msgq_mapdOut` is 2 MB, `mapdOut` arrived at a
steady 20 Hz in every segment — 1200 messages per minute, no gap a respawn would
leave — swaglog shows one `mapd` spawn before each drive and none during (the
success criterion below), and no mapd error. Way-match quality against the tile
reader is not assessed yet.

Nothing in the control path notices. speedlimitd has always derived every control
decision from the offline tile reader (`osm_query.OsmTileReader`); it consumes
`mapdOut` through `mapd_source.telemetry_from_mapd` purely as Phase-1 observation,
logged into `speedLimitState` for comparison. Actuation is byte-identical with mapd
running or absent, and an absent service degrades to vision-only rather than failing.

Tiles are downloaded by COD's web UI into `/data/media/0/osm/offline/`, which is
exactly where mapd reads them.

**Phase 2 — G/S curve look-ahead, not a cutover (2026-11-09).** Route 531 showed
mapd's way match and the tile reader fail in complementary ways, and the tile-only
G/S logic already handled every S20 moment, so road identity stays with the tile
reader and nothing in the tile stack is deleted. Phase 2 is instead a map curve cap
on G/S expressways only: speedlimitd reads `mapdExtendedOut.path` (registered at
mapd's 2 MB queue size, not logged) and computes its own cap with its own a_y
budget, beyond the camera's ~100 m. It is always published as `mapCurve*` in
`speedLimitState`, and slows the car only when the Driving-panel toggle
"Map Curve Look-ahead (G/S)" (`MapdCurveLookahead`, default off) is on. See
`docs/superpowers/specs/2026-11-09-mapd-gs-curve-lookahead-design.md`.

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
| MapdVersion | string | v2.4.0 | Version tracked/pinned by `mapd_manager.py` |

The v2.4.0 pin is deliberate, for three independent reasons.

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

**Queue size coupling.** mapd creates `mapdOut` at its own hardcoded size
(`ServiceQueueSize` in upstream `settings/const.go`, MEDIUM = 2 MB), and the
4th element of our `services.mapdOut` entry must equal it. See below.

## Root cause: mapdOut queue size mismatch

Found 2026-11-09 while reviewing v2.4.0 (issue #31). It is a hypothesis that
fits every observation, but no drive has confirmed it.

- mapd creates `mapdOut` at 2 MB. We registered it as `[true, 20.0, 20]`, so
  openpilot's `Service` defaulted it to `QueueSize.SMALL`, 250 KB. speedlimitd's
  SubMaster opens it at that size.
- Every msgq opener, C++ and gomsgq alike, `ftruncate`s the shared
  `/dev/shm/msgq_mapdOut` to **its own** size. A reader opening after mapd
  shrinks the file under mapd's 2 MB mmap.
- mapd's first write past 250 KB is then a write beyond EOF, which raises
  `SIGBUS code=0x2` (BUS_ADRERR), exactly the trace below. A parked `mapdOut` is
  about 170 B, so 250 KB is about 1,500 messages, or about 75 s at 20 Hz. The
  observed figure was "~80 s, ≈1600 msgs".
- Not yet confirmed: what shrinks the file again after each respawn. The likely
  candidate is speedlimitd, whose 250 KB mapping cannot reach mapd's offsets, so
  it would crash and respawn on the same cycle. Check speedlimitd's spawn cadence
  if this recurs.
- Nobody has reported a SIGBUS upstream, which suggests forks that register
  `mapdOut` at MEDIUM never hit this.

The fix is `"mapdOut": [true, 20.0, 20, 2097152]`. `plugins/services.py` now
**rewrites** a stale plugin entry instead of skipping any name already present.
Before this, a device that already carried the 3-tuple would have kept 250 KB
across every reinstall, because `services.py` is edited in place and never reset.

v2.4.0's msgq change, reader capacity auto-detection, is unrelated: our
`NUM_READERS` is 15, the same as gomsgq's fallback. `mapd_defaults.json` sets
`msgq_queue_size: 0` (auto-detect) rather than hardcoding 15, so mapd follows
the msgq.h actually on the device.

**Schema delta adopted with the pin.** v2.4.0 added `isForward @27 :Bool` to
`MapdOut` ("travel follows the way's stored node order"). It is declared in
`cereal/slot19.capnp`. `MapdIn` and `MapdExtendedOut` are unchanged.

## History: DORMANT after the v2.3.1 activation attempt, 2026-08-21

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

### Verifying the v2.4.0 re-activation
1. Deploy, then **reboot**. plugind freezes manifests at discovery, so a
   `processes` change needs plugind restarted, not just `.needs_restart`.
2. Check that the `mapdOut` line in the installed `cereal/services.py`
   (`/data/openpilot/cereal/` or `/data/openpilot/openpilot/cereal/`) reads
   `(True, 20.0, 20, 2097152)`, and that
   `ls -l /dev/shm/msgq_mapdOut` is 2 MB plus the header, not about 250 KB.
3. Success criterion: one long-lived mapd PID across a whole drive. Watch the
   spawn cadence in swaglog (`plugin process 'mapd' spawned`), which is how the
   ~85 s flap was measured. Watch speedlimitd's cadence too.
4. If it still flaps, revert to dormant (empty `processes`) and use the
   debugging note above to capture the trace.

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
which files a GitHub Issue with the schema verdict and a bump checklist precomputed.

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
