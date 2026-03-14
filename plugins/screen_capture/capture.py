"""Screen capture plugin — dim camera icon tap zone at bottom center."""
import os
import time
import pyray as rl

SCREENSHOT_DIR = '/data/media/screenshots'
TAP_W = 200
TAP_H = 80
FLASH_FRAMES = 2
COOLDOWN = 1.0  # seconds between captures

# Camera icon dimensions (centered within tap zone)
ICON_W = 84
ICON_H = 60
ICON_ALPHA = 50  # subtle but visible

_initialized = False
_tap_rect = None  # rl.Rectangle
_icon_x = 0
_icon_y = 0
_flash_remaining = 0
_last_capture = 0.0


def _ensure_init():
    global _initialized, _tap_rect, _icon_x, _icon_y
    if _initialized:
        return
    _initialized = True
    from openpilot.system.ui.lib.application import gui_app
    w, h = gui_app.width, gui_app.height
    _tap_rect = rl.Rectangle(
        (w - TAP_W) / 2,
        h - TAP_H,
        TAP_W,
        TAP_H,
    )
    _icon_x = int(w / 2)
    _icon_y = int(h - TAP_H / 2)
    os.makedirs(SCREENSHOT_DIR, exist_ok=True)


def _draw_camera_icon(cx, cy, alpha):
    """Draw a simple camera icon centered at (cx, cy)."""
    color = rl.Color(255, 255, 255, alpha)
    hw, hh = ICON_W // 2, ICON_H // 2

    # Camera body — rounded rectangle
    body_top = cy - hh + 5
    rl.draw_rectangle_rounded(
        rl.Rectangle(cx - hw, body_top, ICON_W, ICON_H - 5),
        0.3, 4, color,
    )
    # Viewfinder bump on top
    rl.draw_rectangle(cx - 8, cy - hh, 16, 7, color)
    # Lens circle (dark cutout)
    lens_r = int(ICON_H * 0.28)
    rl.draw_circle(cx, cy + 3, lens_r + 2, rl.Color(0, 0, 0, alpha))
    # Lens ring
    rl.draw_ring(rl.Vector2(cx, cy + 3), lens_r, lens_r + 3, 0, 360, 36, color)


def _capture_from_texture():
    """Capture screenshot from the render texture (complete frame, no UI overlay)."""
    from openpilot.system.ui.lib.application import gui_app
    rt = gui_app._render_texture
    if rt is None:
        return
    ts = time.strftime('%Y%m%d_%H%M%S')
    path = os.path.join(SCREENSHOT_DIR, f'capture_{ts}.png')
    image = rl.load_image_from_texture(rt.texture)
    rl.image_flip_vertical(image)
    rl.export_image(image, path)
    rl.unload_image(image)


def on_pre_end_drawing(default):
    """Draw camera icon, detect taps, capture from render texture."""
    global _flash_remaining, _last_capture
    _ensure_init()

    from openpilot.system.ui.lib.application import gui_app

    # Draw dim camera icon (on screen buffer, won't appear in captures)
    _draw_camera_icon(_icon_x, _icon_y, ICON_ALPHA)

    # Check for tap in zone
    now = time.monotonic()
    if now - _last_capture > COOLDOWN:
        for ev in gui_app.mouse_events:
            if ev.left_released and rl.check_collision_point_rec(
                rl.Vector2(ev.pos.x, ev.pos.y), _tap_rect
            ):
                _capture_from_texture()
                _flash_remaining = FLASH_FRAMES
                _last_capture = now
                break

    # Visual feedback — brief white flash
    if _flash_remaining > 0:
        rl.draw_rectangle(0, 0, gui_app.width, gui_app.height,
                          rl.Color(255, 255, 255, 60))
        _flash_remaining -= 1


def on_render_overlay(default, content_rect):
    """Register tap zone so onroad sidebar toggle is suppressed."""
    _ensure_init()
    from openpilot.selfdrive.ui.onroad.overlay_zones import register_rect_zone
    register_rect_zone(_tap_rect.x, _tap_rect.y, _tap_rect.width, _tap_rect.height)
