# Screen Capture

Tap a button on screen to save a screenshot.

## What it does

A dim white camera icon sits at the bottom-center of every screen — home,
settings, and onroad. Tap it and the plugin saves a PNG of what's currently
on screen, with a brief white flash to confirm the capture happened.

Onroad, it also sends a bookmark event alongside the screenshot, so the
capture shows up as a bookmarked moment in that drive's rlog (visible in
Connect-on-Device) in addition to the saved image.

## How to use it

Tap the camera icon at the bottom-center of the screen. There's a 1-second
cooldown between captures so a stray double-tap doesn't save two images.

No setting to turn this on or off — the tap zone is always present. The
capture itself is read from the app's off-screen render texture, so the
camera icon and the flash effect never appear in the saved image, and
onroad, tapping it doesn't also trigger the sidebar toggle underneath it.

## Where captures go

```
/data/media/screenshots/capture_YYYYMMDD_HHMMSS.png
```

Pull them off the device with:

```bash
scp c3:/data/media/screenshots/*.png .
```

## Hooks

| Hook | What it's for |
|------|----------------|
| `ui.pre_end_drawing` | Draws the icon, watches for a tap, shows the flash |
| `ui.post_end_drawing` | Does the actual screenshot capture + bookmark, after the frame finishes (so it doesn't block drawing) |
| `ui.render_overlay` | Registers the icon's tap zone so it doesn't double as the onroad sidebar-toggle area |
| `device.health_check` | Reports plugin status (always "ok" — no state to be unhealthy about) |

## Limits

- No history or gallery in the app itself — screenshots just accumulate in
  the folder above until you clear them or pull them off.
- Onroad, saving still writes to local storage; there's no upload step.
