"""UI recorder plugin — moves all RECORD/STREAM_UI ffmpeg logic out of application.py.

Reads the same env vars that COD sets before spawning the UI process.
No-op when neither RECORD nor STREAM_UI is set (zero overhead in normal sessions).

Lazy init: ffmpeg and writer thread start on the first post_end_drawing call that
finds a render texture available — no new init hook required.

Writer thread: render thread puts raw RGBA frames into a queue (maxsize=2).
Background thread drains the queue and writes to ffmpeg stdin. Frames are
dropped (not queued) when the encoder is behind — same semantics as upstream
v0.11.0's _ffmpeg_writer_thread.
"""
import atexit
import os
import queue
import subprocess
import threading
from pathlib import Path

# ── Env var interface (identical to current application.py; COD sets these) ─────

RECORD = os.getenv("RECORD") == "1"
STREAM_UI = os.getenv("STREAM_UI") == "1"

if RECORD:
  RECORD_HLS = os.getenv("RECORD_HLS") == "1"
  RECORD_OUTPUT = os.getenv("RECORD_OUTPUT", "output.mp4")
  RECORD_SKIP = int(os.getenv("RECORD_SKIP", "0"))
  RECORD_CODEC = os.getenv("RECORD_CODEC", "libx264")
  RECORD_FRAG_MP4 = os.getenv("RECORD_FRAG_MP4") == "1"
  RECORD_RAW = os.getenv("RECORD_RAW") == "1"
  RECORD_VF = os.getenv("RECORD_VF", "")
  if not RECORD_HLS and not RECORD_FRAG_MP4 and not RECORD_RAW:
    RECORD_OUTPUT = str(Path(RECORD_OUTPUT).with_suffix(".mp4"))

if STREAM_UI:
  STREAM_UI_FIFO = os.getenv("STREAM_UI_FIFO", "/tmp/ui_stream.fifo")
  STREAM_UI_SKIP = int(os.getenv("STREAM_UI_SKIP", "1"))
  _resize_str = os.getenv("STREAM_UI_RESIZE", "")
  if _resize_str and "x" in _resize_str:
    _w, _h = _resize_str.split("x", 1)
    STREAM_UI_RESIZE: tuple[int, int] | None = (int(_w), int(_h))
  else:
    STREAM_UI_RESIZE = None

# ── Module state ─────────────────────────────────────────────────────────────────

_initialized = False
_ffmpeg_proc: subprocess.Popen | None = None
_writer_queue: queue.Queue | None = None
_writer_thread: threading.Thread | None = None
_stream_queue: queue.Queue | None = None
_stream_thread: threading.Thread | None = None


# ── Internal helpers ─────────────────────────────────────────────────────────────

def _build_ffmpeg_args(width: int, height: int, fps: float) -> list[str]:
  """Build the ffmpeg command line. Pure function — easy to test."""
  capture_fps = fps / (RECORD_SKIP + 1) if RECORD_SKIP > 0 else fps
  args = [
    'ffmpeg',
    '-v', 'warning',
    '-stats',
    '-f', 'rawvideo',
    '-pix_fmt', 'rgba',
    '-s', f'{width}x{height}',
    '-r', str(capture_fps),
    '-i', 'pipe:0',
    '-vf', f'vflip,{RECORD_VF + "," if RECORD_VF else ""}format=yuv420p',
  ]
  if not RECORD_RAW:
    args += ['-c:v', RECORD_CODEC]
    if RECORD_CODEC == 'libx264':
      args += ['-preset', 'ultrafast']
  args += ['-y']

  if RECORD_RAW:
    args += ['-f', 'rawvideo', '-pix_fmt', 'yuv420p', RECORD_OUTPUT]
  elif RECORD_HLS:
    hls_time = os.getenv("RECORD_HLS_TIME", "2")
    hls_list_size = os.getenv("RECORD_HLS_LIST_SIZE", "10")
    args += [
      '-g', str(max(1, int(capture_fps))),
      '-f', 'hls',
      '-hls_time', hls_time,
      '-hls_list_size', hls_list_size,
      '-hls_flags', 'delete_segments+append_list',
      RECORD_OUTPUT,
    ]
  elif RECORD_FRAG_MP4:
    args += [
      '-g', str(max(1, int(capture_fps))),
      '-f', 'mp4',
      '-movflags', 'frag_keyframe+empty_moov+default_base_moof',
      RECORD_OUTPUT,
    ]
  else:
    args += ['-f', 'mp4', RECORD_OUTPUT]

  return args


def _start(width: int, height: int, fps: float) -> None:
  global _initialized, _ffmpeg_proc, _writer_queue, _writer_thread
  global _stream_queue, _stream_thread

  if RECORD:
    _writer_queue = queue.Queue(maxsize=2)
    _ffmpeg_proc = subprocess.Popen(_build_ffmpeg_args(width, height, fps), stdin=subprocess.PIPE)

    def _writer():
      while True:
        data = _writer_queue.get()
        if data is None:
          break
        try:
          _ffmpeg_proc.stdin.write(data)
          _ffmpeg_proc.stdin.flush()
        except (BrokenPipeError, OSError):
          break

    _writer_thread = threading.Thread(target=_writer, daemon=True, name="ui_recorder_writer")
    _writer_thread.start()

  if STREAM_UI:
    if not os.path.exists(STREAM_UI_FIFO):
      os.mkfifo(STREAM_UI_FIFO)
    _stream_queue = queue.Queue(maxsize=2)

    def _stream_worker():
      import struct
      try:
        with open(STREAM_UI_FIFO, 'wb') as fifo:
          while True:
            data = _stream_queue.get()
            if data is None:
              break
            fifo.write(struct.pack('<I', len(data)))
            fifo.write(data)
            fifo.flush()
      except (BrokenPipeError, OSError):
        pass

    _stream_thread = threading.Thread(target=_stream_worker, daemon=True, name="ui_recorder_stream")
    _stream_thread.start()

  atexit.register(_cleanup)
  _initialized = True


def _cleanup() -> None:
  global _ffmpeg_proc, _writer_queue, _writer_thread, _stream_queue, _stream_thread

  if _writer_queue is not None:
    try:
      _writer_queue.put_nowait(None)
    except queue.Full:
      pass
    if _writer_thread is not None:
      _writer_thread.join(timeout=2)

  if _ffmpeg_proc is not None:
    try:
      _ffmpeg_proc.stdin.flush()
      _ffmpeg_proc.stdin.close()
    except (BrokenPipeError, OSError):
      pass
    try:
      _ffmpeg_proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
      _ffmpeg_proc.terminate()
      _ffmpeg_proc.wait()

  if _stream_queue is not None:
    try:
      _stream_queue.put_nowait(None)
    except queue.Full:
      pass
    if _stream_thread is not None:
      _stream_thread.join(timeout=2)


# ── Hook ─────────────────────────────────────────────────────────────────────────

def on_post_end_drawing(default):
  """Capture render texture and feed ffmpeg / stream FIFO. Called every frame."""
  global _initialized

  if not RECORD and not STREAM_UI:
    return

  from openpilot.system.ui.lib.application import gui_app
  import pyray as rl

  rt = gui_app._render_texture
  if rt is None:
    return

  if not _initialized:
    _start(gui_app.width, gui_app.height, gui_app.target_fps)

  frame = gui_app.frame

  if RECORD:
    if RECORD_SKIP <= 0 or frame % (RECORD_SKIP + 1) == 0:
      image = rl.load_image_from_texture(rt.texture)
      data_size = image.width * image.height * 4
      data = bytes(rl.ffi.buffer(image.data, data_size))
      rl.unload_image(image)
      try:
        _writer_queue.put_nowait(data)
      except queue.Full:
        pass  # drop frame — encoder is behind

  if STREAM_UI:
    if STREAM_UI_SKIP <= 1 or frame % STREAM_UI_SKIP == 0:
      image = rl.load_image_from_texture(rt.texture)
      if STREAM_UI_RESIZE:
        rl.image_resize(image, STREAM_UI_RESIZE[0], STREAM_UI_RESIZE[1])
      data_size = image.width * image.height * 4
      data = bytes(rl.ffi.buffer(image.data, data_size))
      rl.unload_image(image)
      try:
        _stream_queue.put_nowait(data)
      except queue.Full:
        pass  # drop frame


def on_health_check(acc, **kwargs):
  return {**acc, "ui_recorder": {"status": "ok"}}
