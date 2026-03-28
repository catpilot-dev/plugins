"""Screen capture plugin — dim camera icon tap zone at bottom center.

Offroad: saves PNG screenshot to /data/media/screenshots/ (GUI documentation).
Onroad:  saves HUD PNG + sends bookmarkButton (→ userBookmark in rlog).
         The PNG captures the full HUD overlay from the render texture.
         COD shows it on the bookmark row for instant HUD frame export.
"""
import os
import sys
import threading
import time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import MEDIA_DIR
import pyray as rl

SCREENSHOT_DIR = os.path.join(MEDIA_DIR, 'screenshots')
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
_capture_pending = False   # set by pre_end_drawing, consumed by post_end_drawing
_pm = None  # cereal PubMaster for bookmarkButton


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


def _is_onroad():
    """Check if device is onroad via ui_state."""
    try:
        from openpilot.selfdrive.ui.ui_state import UIState
        return UIState().started
    except Exception:
        return False


def _write_png_bg(image, path):
    """Background thread: encode and write PNG, then unload image."""
    try:
        rl.export_image(image, path)
    finally:
        rl.unload_image(image)


def _save_png():
    """Save PNG screenshot from the render texture. Returns filename or None.

    Called from on_post_end_drawing (after end_drawing()), so GPU readback
    does not block the current frame's completion.  PNG encoding and disk
    write are handed off to a background thread.
    """
    from openpilot.system.ui.lib.application import gui_app
    rt = gui_app._render_texture
    if rt is None:
        return None
    ts = time.strftime('%Y%m%d_%H%M%S')
    filename = f'capture_{ts}.png'
    path = os.path.join(SCREENSHOT_DIR, filename)
    # GPU readback — safe here because end_drawing() has already returned
    image = rl.load_image_from_texture(rt.texture)
    rl.image_flip_vertical(image)
    rl.image_format(image, rl.PIXELFORMAT_UNCOMPRESSED_R8G8B8)
    # Encode + write off the render thread
    threading.Thread(target=_write_png_bg, args=(image, path), daemon=True).start()
    return filename


def _send_bookmark():
    """Send bookmarkButton event → feedbackd → userBookmark in rlog."""
    global _pm
    try:
        if _pm is None:
            from cereal import messaging
            _pm = messaging.PubMaster(['bookmarkButton'])
        from cereal import messaging
        msg = messaging.new_message('bookmarkButton')
        msg.valid = True
        _pm.send('bookmarkButton', msg)
    except Exception:
        pass


def on_pre_end_drawing(default):
    """Draw camera icon + flash overlay; record tap intent for post_end_drawing."""
    global _flash_remaining, _last_capture, _capture_pending
    _ensure_init()

    from openpilot.system.ui.lib.application import gui_app

    # Draw dim camera icon (screen buffer only — won't appear in render texture)
    _draw_camera_icon(_icon_x, _icon_y, ICON_ALPHA)

    # Detect tap — set flag for post_end_drawing to act on
    now = time.monotonic()
    if now - _last_capture > COOLDOWN:
        for ev in gui_app.mouse_events:
            if ev.left_released and rl.check_collision_point_rec(
                rl.Vector2(ev.pos.x, ev.pos.y), _tap_rect
            ):
                _capture_pending = True
                _flash_remaining = FLASH_FRAMES
                _last_capture = now
                break

    # Visual feedback — brief white flash
    if _flash_remaining > 0:
        rl.draw_rectangle(0, 0, gui_app.width, gui_app.height,
                          rl.Color(255, 255, 255, 60))
        _flash_remaining -= 1


def on_post_end_drawing(default):
    """GPU readback + bookmark — runs after end_drawing() to avoid blocking the frame."""
    global _capture_pending
    if not _capture_pending:
        return
    _capture_pending = False

    _save_png()

    if _is_onroad():
        _send_bookmark()


def on_render_overlay(default, content_rect):
    """Register tap zone so onroad sidebar toggle is suppressed."""
    _ensure_init()
    from openpilot.selfdrive.ui.onroad.overlay_zones import register_rect_zone
    register_rect_zone(_tap_rect.x, _tap_rect.y, _tap_rect.width, _tap_rect.height)


def on_health_check(acc, **kwargs):
    return {**acc, "screen_capture": {"status": "ok"}}
