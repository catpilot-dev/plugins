# ui_recorder — UI Video Capture

**Type**: hook (`ui.post_end_drawing`, `device.health_check`)

Captures the rendered driving UI (HUD overlay included) and encodes it to
video. This is how Connect-on-Device (COD) produces its HUD-rendered replay
videos — it is tooling, not a driver-facing feature: with no configuration
set, every hook call is a no-op and the plugin adds zero overhead.

## How it works

COD sets environment variables before spawning the UI process; the plugin
reads them at import and lazily starts ffmpeg plus a writer thread on the
first rendered frame. The render thread drops frames rather than blocking
when the encoder falls behind, so recording can never stall the UI.

## Configuration (environment variables)

| var | meaning |
|---|---|
| `RECORD=1` | record to a file (`RECORD_OUTPUT`, default `output.mp4`) |
| `RECORD_HLS=1` / `RECORD_FRAG_MP4=1` / `RECORD_RAW=1` | alternative container/stream formats |
| `RECORD_CODEC` | encoder (default `libx264`) |
| `RECORD_SKIP` | frame decimation |
| `RECORD_VF` | extra ffmpeg video filter |
| `STREAM_UI=1` | stream raw frames to a FIFO (`STREAM_UI_FIFO`), with optional `STREAM_UI_SKIP` / `STREAM_UI_RESIZE` |

`device.health_check` registers the plugin with the health system (a static
"ok" — it does not yet probe encoder liveness).
