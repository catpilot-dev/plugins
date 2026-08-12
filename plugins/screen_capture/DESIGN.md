# Screen Capture — Design & Implementation

`screen_capture` is a pure **hook plugin** (no background process): a dim
camera icon at the bottom-center of every screen that saves a screenshot when
tapped. Its defining design decision is that it has **two capture paths with
very different costs**, chosen by driving state:

| State | On tap | Cost on the UI thread |
|---|---|---|
| Offroad | GPU readback → PNG → `/data/media/screenshots/` | one readback stall + format convert (harmless parked) |
| Onroad | trigger the UI's own Bookmark action — nothing else | one msgq publish (~µs) |

Onroad, the plugin deliberately does **not** produce the image. It only marks
the moment; Connect-on-Device's background screenshot worker reconstructs the
exact HUD frame later, offline, with full overlay fidelity. The plugin and COD
form a cross-repo contract described below.

## Why the onroad path must not touch GPU or disk

The naive implementation (used through v0.11.1) captured onroad the same way
as offroad. The blocking cost is not the PNG write — that was already on a
background thread — it is the readback itself: `rl.load_image_from_texture`
issues a synchronous `glReadPixels`, which forces a full GPU pipeline flush
and copies ~9 MB (2160×1080 RGBA) back to the CPU **on the render thread**,
even when called after `end_drawing()`. That stalls the driving UI's frame
loop every tap.

A truly non-blocking in-process capture was considered and rejected: it would
need asynchronous PBO double-buffered readback, which neither raylib's
`rlReadTexturePixels` nor pyray exposes (both are synchronous by design), so
it means raw GL calls on the render thread or another raylib patch — and even
done perfectly it still spends GPU memory bandwidth shared with the driving
UI, a map-time memcpy, CPU PNG encoding, and a disk write while driving.
Bookmark-only reduces the onroad cost to a single message publish and moves
all heavy work to a parked device. The reconstruction is also *better* than a
live grab: frame-exact, never mid-composite, rendered at leisure.

The offroad path keeps the simple synchronous readback: with the car parked
there is no control loop to disturb, and a one-frame hiccup on the settings
screen is invisible. Only the PNG encode + disk write are offloaded to a
daemon thread (`_write_png_bg`).

## Tap handling — hooks

| Hook | Role |
|---|---|
| `ui.pre_end_drawing` | Draws the icon and flash into the **screen buffer only** (after the render-texture composite, so neither ever appears in a capture). Polls `gui_app.mouse_events` for a tap inside the 200×80 zone, applies the 1 s cooldown, and sets `_capture_pending`. |
| `ui.post_end_drawing` | Consumes `_capture_pending` after the frame completes: onroad (`UIState().started`) → `_send_bookmark()`; offroad → `_save_png()`. |
| `ui.render_overlay` | Registers the tap zone with `overlay_zones`, so an onroad tap doesn't also toggle the sidebar underneath. |
| `device.health_check` | Always `"ok"` — the plugin holds no state that can go unhealthy. |

Tap detection and capture are split across the two hooks so the readback (or
publish) happens after `end_drawing()` returns — the offroad readback then at
least doesn't delay the *current* frame's completion.

`_send_bookmark()` calls the stock UI's `_on_bookmark_clicked()`, found by
walking `gui_app._nav_stack` for the widget that defines it (both `MainLayout`
and `MiciMainLayout` do). It deliberately does **not** open its own socket:
the UI process already owns the only `bookmarkButton` publisher, msgq accepts
a second publisher in the same process silently, and the *first* publisher's
next `send()` then raises `MultiplePublishersError` — which killed the UI on
the next press of the stock Bookmark button. One publisher, one code path.

From there the stock path takes over: feedbackd turns `bookmarkButton` into a
`userBookmark` event in the drive's log. `userBookmark` has qlog decimation 1
(`services.py`), so every tap is present in the small qlog — this is what
makes cheap offline discovery possible.

## The COD handshake — onroad extraction

Everything below lives in the connect-on-device repo (`screenshot_worker.py`,
`render_clip_headless.py --screenshot-at`, `handlers/screenshots.py`), but it
is half of this plugin's design, so it is documented here.

### Discovery

A worker in the COD server wakes every 5 minutes and runs **only while the
device is offroad**, re-checking `IsOnroad` before every job and yielding to
user-triggered HUD prerenders. It scans the **latest route only** — captures
belong to the current drive; there is no retroactive backfill of old routes.
Scanned segments and per-tap status live in route metadata under
`hud_capture_state` (`scanned_segs`, per-offset `status`/`attempts`, 3-attempt
cap), and once nothing is pending, cycles self-skip on a cheap work signature
(newest route id + segment count) until a new drive appears.

### Exact tap time and naming

The tap's absolute time is computed exactly, not estimated:
`userBookmark.logMonoTime` is converted mono→wall against the `unixTimestampMillis`
of the *closest* fixed `gpsLocationExternal` in the same segment. GPS is the only
absolute clock in the log that can be trusted — the device RTC boots months stale
(`initData.wallTimeNanos` records that stale value), and the two clocks agree to
within 0.2 s once NTP has run. Measured on a real drive, GPS-vs-mono holds to
±20 ms within a segment and drifts ~25 ms over 16 minutes.

A segment with no fix of its own falls back to a route-wide anchor — any fix from
anywhere in the drive, since `logMonoTime` is boot-relative and continuous across
segments. That still dates the tap to ~40 ms.

A drive with no fix at all produces **no capture**. There is no estimate-from-
`create_time` path: `create_time` dates a route from its *first GPS fix*, which
lags the drive's true start by 21-45 s on real cold starts, so a PNG named from it
is wrong to the user and outside the 2 s window that matches it back to its own
bookmark row. Those taps are marked `no_gps_time` and skipped.

The PNG is named `capture_YYYYMMDD_HHMMSS.png` in the **drive location's
local time** (route GPS longitude → `round(lng/15)`, the same convention as
route dates), because the C3 system clock runs UTC and the name exists for a
human scanning the folder. The name is cosmetic: the worker records
`file` + `epoch` per capture in `hud_capture_state`, and COD's screenshot
handlers use that map for `capture_time` and ±2 s bookmark-row matching.
Filename parsing (device clock) remains only as the fallback for
plugin-saved offroad captures.

### Rendering

One frame per tap via the stock clip pipeline: replay a ~3 s window ending
just past the target (so SubMaster state settles), render headlessly at
2160×1080, and PNG-export the render texture at the target frame index —
no ffmpeg, no burned-in metadata or clip-time overlays. Plugin UI overlays
(speed-limit sign, temperatures, …) are loaded from their allow-listed hooks,
so the extracted frame shows what the driver saw. `screen_capture` itself is
explicitly skipped during replay (`_SKIP_PLUGINS`) — the icon and flash never
appear in an extraction, matching the live behavior. A render takes roughly
40 s on-device, serialized one job at a time.

### Durability

The reconstruction needs the route's rlog + fcamera to still exist when the
worker runs. COD's storage reclaim sorts bookmarked routes **last** among
normal deletion candidates, so tap footage survives longest — but a route
deleted before extraction loses its pending captures, by design.

## Failure modes

- **No GPS fix in the tap's segment** → dated from the route-wide anchor
  (~40 ms). A drive that never got a fix yields no capture at all
  (`no_gps_time`) rather than a wrongly-dated one.
- **Render failure** (missing fcamera, corrupt log) → retried up to 3 times,
  then marked `failed` in `hud_capture_state`; never retried forever.
- **Route not yet enriched** (no `gps_time`) → skipped until the route list
  has been viewed once, which fills it; picked up on a later cycle.
- **Two taps in the same second** → same filename; second render overwrites
  the first (the 1 s cooldown makes this rare; the live path had the same
  property).
- **Stock bookmark handler missing** (UI internals changed) → the onroad tap
  is a silent no-op rather than a crash; `_send_bookmark()` returns False.
  Regression-tested in `tests/test_capture.py`.

## History

Until 2026-08-11 the onroad path published `bookmarkButton` from a PubMaster
of its own. Because the UI already owns that publisher, the plugin's socket
silently hijacked the queue and the stock Bookmark button's next send raised
`MultiplePublishersError`, taking the UI process down: black screen for the
few seconds the manager needed to restart it, and a TAKE CONTROL IMMEDIATELY
/ System Lagging alert on a moving car (route `000003ef` seg 24, 09:58:41).
The plugin now drives the stock action instead of publishing.

Through v0.11.1 the plugin did the full GPU readback + PNG save onroad as
well, sending the bookmark *in addition to* the file. The onroad readback
stall motivated the split design above; the bookmark, which had been a
convenience marker, became the entire onroad mechanism.
