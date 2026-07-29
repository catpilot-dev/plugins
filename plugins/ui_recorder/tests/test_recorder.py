"""Tests for ui_recorder plugin."""
import importlib
import os
import sys
import queue
import threading
import pytest
from unittest.mock import MagicMock, patch, call

_PLUGIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PLUGIN_DIR not in sys.path:
  sys.path.insert(0, _PLUGIN_DIR)


def _load_recorder(env: dict):
  """Import recorder with specific env vars, isolated from previous imports."""
  for k in list(sys.modules):
    if 'ui_recorder' in k or k == 'recorder':
      del sys.modules[k]
  with patch.dict(os.environ, env, clear=False):
    import recorder as mod
  return mod


def _make_gui_app(width=1920, height=1080, fps=20, frame=0):
  app = MagicMock()
  app.width = width
  app.height = height
  app.target_fps = fps
  app.frame = frame
  tex = MagicMock()
  tex.texture = MagicMock()
  app._render_texture = tex
  return app


# ── No-op when RECORD and STREAM_UI are both off ─────────────────────────────────

class TestNoOp:
  def test_hook_returns_immediately(self):
    mod = _load_recorder({})
    assert not mod.RECORD
    assert not mod.STREAM_UI
    # Should return without touching anything
    mod.on_post_end_drawing(None)
    assert not mod._initialized

  def test_build_ffmpeg_args_not_called_when_disabled(self):
    mod = _load_recorder({})
    with patch.object(mod, '_start') as mock_start:
      mod.on_post_end_drawing(None)
      mock_start.assert_not_called()


# ── ffmpeg arg construction ───────────────────────────────────────────────────────

class TestBuildFfmpegArgs:
  def test_plain_mp4(self):
    with patch.dict(os.environ, {'RECORD': '1', 'RECORD_OUTPUT': '/tmp/out.mp4'}, clear=False):
      mod = _load_recorder({'RECORD': '1', 'RECORD_OUTPUT': '/tmp/out.mp4'})
      args = mod._build_ffmpeg_args(1920, 1080, 20)
    assert '-f' in args
    assert 'rawvideo' in args
    assert '1920x1080' in args
    assert 'libx264' in args
    assert 'ultrafast' in args
    assert '/tmp/out.mp4' in args
    assert 'vflip' in ' '.join(args)

  def test_raw_mode_skips_codec(self):
    mod = _load_recorder({'RECORD': '1', 'RECORD_RAW': '1', 'RECORD_OUTPUT': '/tmp/out.raw'})
    args = mod._build_ffmpeg_args(1920, 1080, 20)
    assert 'libx264' not in args
    assert 'yuv420p' in args
    assert '/tmp/out.raw' in args

  def test_hls_mode(self):
    mod = _load_recorder({'RECORD': '1', 'RECORD_HLS': '1', 'RECORD_OUTPUT': '/tmp/hls/index.m3u8'})
    args = mod._build_ffmpeg_args(1920, 1080, 20)
    assert 'hls' in args
    assert '-hls_time' in args
    assert '/tmp/hls/index.m3u8' in args

  def test_frag_mp4_mode(self):
    mod = _load_recorder({'RECORD': '1', 'RECORD_FRAG_MP4': '1', 'RECORD_OUTPUT': '/tmp/out.mp4'})
    args = mod._build_ffmpeg_args(1920, 1080, 20)
    assert 'frag_keyframe' in ' '.join(args)

  def test_record_vf_inserted(self):
    mod = _load_recorder({'RECORD': '1', 'RECORD_VF': 'scale=960:540', 'RECORD_OUTPUT': '/tmp/out.mp4'})
    args = mod._build_ffmpeg_args(1920, 1080, 20)
    vf_arg = args[args.index('-vf') + 1]
    assert 'scale=960:540' in vf_arg

  def test_record_skip_affects_fps(self):
    mod = _load_recorder({'RECORD': '1', 'RECORD_SKIP': '1', 'RECORD_OUTPUT': '/tmp/out.mp4'})
    args = mod._build_ffmpeg_args(1920, 1080, 20)
    # RECORD_SKIP=1 → capture_fps = 20 / 2 = 10.0
    r_idx = args.index('-r')
    assert float(args[r_idx + 1]) == pytest.approx(10.0)

  def test_output_mp4_suffix_added(self):
    mod = _load_recorder({'RECORD': '1', 'RECORD_OUTPUT': '/tmp/out'})
    # No HLS/FRAG/RAW → suffix forced to .mp4
    assert mod.RECORD_OUTPUT.endswith('.mp4')

  def test_custom_codec(self):
    mod = _load_recorder({'RECORD': '1', 'RECORD_CODEC': 'h264_v4l2m2m', 'RECORD_OUTPUT': '/tmp/out.mp4'})
    args = mod._build_ffmpeg_args(1920, 1080, 20)
    assert 'h264_v4l2m2m' in args
    assert 'ultrafast' not in args  # preset only for libx264


# ── Lazy init ─────────────────────────────────────────────────────────────────────

class TestLazyInit:
  def test_init_on_first_frame(self):
    mod = _load_recorder({'RECORD': '1', 'RECORD_OUTPUT': '/tmp/out.mp4'})
    app = _make_gui_app()

    mock_proc = MagicMock()
    mock_proc.stdin = MagicMock()

    with patch('subprocess.Popen', return_value=mock_proc) as mock_popen, \
         patch.dict('sys.modules', {'openpilot.system.ui.lib.application': MagicMock(gui_app=app)}), \
         patch.dict('sys.modules', {'pyray': MagicMock()}):
      mod.on_post_end_drawing(None)

    assert mod._initialized
    mock_popen.assert_called_once()

  def test_no_init_when_render_texture_none(self):
    mod = _load_recorder({'RECORD': '1', 'RECORD_OUTPUT': '/tmp/out.mp4'})
    app = _make_gui_app()
    app._render_texture = None

    with patch.dict('sys.modules', {'openpilot.system.ui.lib.application': MagicMock(gui_app=app)}), \
         patch.dict('sys.modules', {'pyray': MagicMock()}):
      mod.on_post_end_drawing(None)

    assert not mod._initialized


# ── Frame skip ────────────────────────────────────────────────────────────────────

class TestFrameSkip:
  def _run_frames(self, mod, app, n_frames):
    """Run n frames through on_post_end_drawing, returning captured frame indices."""
    captured = []
    rl_mock = MagicMock()

    def fake_load_image(tex):
      img = MagicMock()
      img.width = app.width
      img.height = app.height
      img.data = MagicMock()
      return img

    rl_mock.load_image_from_texture.side_effect = fake_load_image
    rl_mock.ffi.buffer.return_value = b'\x00' * (app.width * app.height * 4)

    app_mod = MagicMock()
    app_mod.gui_app = app

    with patch.dict('sys.modules', {
      'openpilot.system.ui.lib.application': app_mod,
      'pyray': rl_mock,
    }), patch('subprocess.Popen', return_value=MagicMock(stdin=MagicMock())):
      for i in range(n_frames):
        app.frame = i
        mod.on_post_end_drawing(None)
        if rl_mock.load_image_from_texture.called:
          captured.append(i)
          rl_mock.load_image_from_texture.reset_mock()

    return captured

  def test_no_skip_captures_every_frame(self):
    mod = _load_recorder({'RECORD': '1', 'RECORD_SKIP': '0', 'RECORD_OUTPUT': '/tmp/out.mp4'})
    app = _make_gui_app()
    captured = self._run_frames(mod, app, 5)
    assert captured == [0, 1, 2, 3, 4]

  def test_skip_1_captures_every_other_frame(self):
    mod = _load_recorder({'RECORD': '1', 'RECORD_SKIP': '1', 'RECORD_OUTPUT': '/tmp/out.mp4'})
    app = _make_gui_app()
    captured = self._run_frames(mod, app, 6)
    assert captured == [0, 2, 4]

  def test_skip_2_captures_every_third_frame(self):
    mod = _load_recorder({'RECORD': '1', 'RECORD_SKIP': '2', 'RECORD_OUTPUT': '/tmp/out.mp4'})
    app = _make_gui_app()
    captured = self._run_frames(mod, app, 9)
    assert captured == [0, 3, 6]


# ── Writer thread queue drop ──────────────────────────────────────────────────────

class TestWriterQueueDrop:
  def test_full_queue_drops_frame_no_block(self):
    mod = _load_recorder({'RECORD': '1', 'RECORD_OUTPUT': '/tmp/out.mp4'})
    # Pre-fill the queue
    mod._writer_queue = queue.Queue(maxsize=2)
    mod._writer_queue.put(b'frame1')
    mod._writer_queue.put(b'frame2')
    mod._initialized = True

    app = _make_gui_app(frame=0)
    rl_mock = MagicMock()
    img = MagicMock()
    img.width = 1920
    img.height = 1080
    img.data = MagicMock()
    rl_mock.load_image_from_texture.return_value = img
    rl_mock.ffi.buffer.return_value = b'\x00' * (1920 * 1080 * 4)

    import time
    start = time.monotonic()
    with patch.dict('sys.modules', {
      'openpilot.system.ui.lib.application': MagicMock(gui_app=app),
      'pyray': rl_mock,
    }):
      mod.on_post_end_drawing(None)
    elapsed = time.monotonic() - start

    # Must return quickly — no blocking on full queue
    assert elapsed < 0.1
    assert mod._writer_queue.qsize() == 2  # queue unchanged (frame dropped)


# ── Cleanup ───────────────────────────────────────────────────────────────────────

class TestCleanup:
  def test_cleanup_signals_writer_thread(self):
    mod = _load_recorder({'RECORD': '1', 'RECORD_OUTPUT': '/tmp/out.mp4'})

    received = []
    q = queue.Queue(maxsize=4)
    mod._writer_queue = q

    stop_event = threading.Event()

    def fake_writer():
      while True:
        item = q.get()
        received.append(item)
        if item is None:
          stop_event.set()
          break

    t = threading.Thread(target=fake_writer, daemon=True)
    t.start()
    mod._writer_thread = t

    mock_proc = MagicMock()
    mock_proc.stdin = MagicMock()
    mock_proc.wait.return_value = 0
    mod._ffmpeg_proc = mock_proc

    mod._cleanup()
    stop_event.wait(timeout=2)

    assert None in received
    mock_proc.stdin.close.assert_called()
