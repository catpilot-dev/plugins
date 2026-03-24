# screen_capture — Screen Capture

**Type**: hook

## What it does

Tap-to-capture screenshots on any screen (home, settings, onroad). A dim camera icon at bottom center acts as the tap zone. Screenshots are saved as PNG files with timestamps.

### How it works

1. A dim white camera icon (alpha 50) is drawn at bottom center of every screen
2. Tap the icon to capture — a brief white flash confirms the capture
3. The screenshot is read from the render texture (offscreen FBO), which contains the complete frame without the camera icon overlay
4. Saved to `/data/media/screenshots/capture_YYYYMMDD_HHMMSS.png`

### Design decisions

- **Render texture capture** — uses `load_image_from_texture()` instead of `load_image_from_screen()` to avoid DRM framebuffer orientation issues on C3
- **Clean captures** — camera icon and flash draw to the screen buffer only, not the render texture, so they never appear in screenshots
- **1-second cooldown** prevents accidental double-captures
- **Onroad tap suppression** — registers the tap zone via `ui.render_overlay` to prevent triggering the sidebar toggle

## Hooks

| Hook | Function | Description |
|------|----------|-------------|
| `ui.pre_end_drawing` | on_pre_end_drawing | Draw icon, detect taps, capture |
| `ui.render_overlay` | on_render_overlay | Register tap zone for onroad suppression |

## Output

```
/data/media/screenshots/
  capture_20260314_101051.png   # 2160x1080 PNG
  capture_20260314_101157.png
  ...
```

Retrieve via: `scp c3:/data/media/screenshots/*.png .`

## Key Files

```
screen_capture/
  plugin.json    # Plugin manifest
  capture.py     # Tap detection, render texture capture, camera icon
```
