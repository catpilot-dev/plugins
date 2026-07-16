"""Tests for the offline route-replay harness (hevc -> pipeline -> publishes/imgs/timing)."""
import json
from pathlib import Path

import pytest

np = pytest.importorskip("numpy")
av = pytest.importorskip("av")
cv2 = pytest.importorskip("cv2")

from plugins.speedlimitd.tools.sign_vision.harness import ROI_PRESETS, iter_route_frames, run
from plugins.speedlimitd.tools.sign_vision.pipeline import PipelineConfig

WIDTH, HEIGHT, FPS = 320, 240, 20


def _encode_hevc_segment(path: Path, n_frames: int, width=WIDTH, height=HEIGHT, fps=FPS):
  """Writes an n_frames raw-hevc elementary stream to path via PyAV (libx265, falls back to
  libx264 if x265 is unavailable in this environment; skips the test if neither works)."""
  encoder_names = ["libx265", "libx264"]
  container = None
  last_err = None
  for name in encoder_names:
    try:
      container = av.open(str(path), mode="w", format="hevc" if name == "libx265" else "h264")
      stream = container.add_stream(name, rate=fps)
      break
    except Exception as e:  # pragma: no cover - environment dependent
      last_err = e
      container = None
  if container is None:
    pytest.skip(f"no usable hevc/h264 encoder in this environment: {last_err}")

  stream.width = width
  stream.height = height
  stream.pix_fmt = "yuv420p"
  if hasattr(stream, "options"):
    stream.options = {"x265-params": "log-level=none"} if "265" in stream.codec_context.name else {}

  for i in range(n_frames):
    val = (i * 5) % 255
    arr = np.full((height, width, 3), val, dtype=np.uint8)
    frame = av.VideoFrame.from_ndarray(arr, format="bgr24")
    for packet in stream.encode(frame):
      container.mux(packet)
  for packet in stream.encode():
    container.mux(packet)
  container.close()


def _make_route(tmp_path, route="000003a1--5f9bdd218a", n_segments=2, n_frames=20):
  route_dir = tmp_path / "route"
  route_dir.mkdir()
  for seg in range(n_segments):
    seg_dir = route_dir / f"{route}--{seg}"
    seg_dir.mkdir()
    _encode_hevc_segment(seg_dir / "fcamera.hevc", n_frames)
  return route_dir


def test_roi_presets_exact():
  assert ROI_PRESETS == {"full": (0.0, 0.0, 1.0, 1.0), "right": (0.45, 0.05, 1.0, 0.80)}


def test_iter_route_frames_sampling_and_segments(tmp_path):
  route_dir = _make_route(tmp_path, n_segments=2, n_frames=20)
  frames = list(iter_route_frames(route_dir, sample_hz=4.0, fps=20.0))

  assert len(frames) == 8
  assert [f[3] for f in frames] == [0, 0, 0, 0, 1, 1, 1, 1]
  assert [f[2] for f in frames] == [0, 5, 10, 15, 20, 25, 30, 35]
  assert [f[1] for f in frames] == pytest.approx([0.0, 0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 1.75])
  for frame_bgr, t, frame_idx, seg in frames:
    assert frame_bgr.shape == (HEIGHT, WIDTH, 3)


def test_iter_route_frames_missing_hevc_skipped(tmp_path, capsys):
  route_dir = tmp_path / "route"
  route_dir.mkdir()
  seg0 = route_dir / "r--0"
  seg0.mkdir()
  _encode_hevc_segment(seg0 / "fcamera.hevc", 20)
  seg1 = route_dir / "r--1"
  seg1.mkdir()  # no fcamera.hevc -> skipped with a warning

  frames = list(iter_route_frames(route_dir, sample_hz=4.0, fps=20.0))

  assert len(frames) == 4
  assert all(f[3] == 0 for f in frames)
  captured = capsys.readouterr()
  assert "warning" in captured.out.lower()
  assert "fcamera.hevc" in captured.out


def test_iter_route_frames_corrupt_segment_skipped(tmp_path, capsys):
  route_dir = tmp_path / "route"
  route_dir.mkdir()

  seg0 = route_dir / "r--0"
  seg0.mkdir()
  _encode_hevc_segment(seg0 / "fcamera.hevc", 20)

  seg1 = route_dir / "r--1"
  seg1.mkdir()
  # Present-but-corrupt file (garbage bytes): av.open()/decode() raises InvalidDataError,
  # mirroring a C3 realdata segment truncated/corrupted at ignition-off.
  (seg1 / "fcamera.hevc").write_bytes(b"\x00\xff" * 1000)

  seg2 = route_dir / "r--2"
  seg2.mkdir()
  _encode_hevc_segment(seg2 / "fcamera.hevc", 20)

  frames = list(iter_route_frames(route_dir, sample_hz=4.0, fps=20.0))

  segments_seen = {f[3] for f in frames}
  assert 0 in segments_seen
  assert 2 in segments_seen

  captured = capsys.readouterr()
  assert "warning" in captured.out.lower()
  assert "decode error" in captured.out.lower()


def test_run_end_to_end_publishes_once_and_writes_files(tmp_path, capsys):
  route_dir = _make_route(tmp_path, route="00000abc--deadbeef01", n_segments=1, n_frames=20)
  out_dir = tmp_path / "out"

  box = (100.0, 80.0, 160.0, 140.0)  # steady 60x60 box, well inside the 320x240 frame

  def detect_fn(frame_bgr):
    return [(box, 0.9)]

  def classify_fn(crop_bgr):
    return ("60", 0.95)

  config = PipelineConfig(confirm_votes=3, confirm_window_s=5.0)
  summary = run(route_dir, out_dir, detect_fn, classify_fn, sample_hz=4.0, config=config)

  pub_lines = (out_dir / "publishes.jsonl").read_text().strip().splitlines()
  assert len(pub_lines) == 1
  rec = json.loads(pub_lines[0])
  assert set(rec) == {"t", "frame_idx", "value", "conf", "box", "crop_img", "ctx_img", "route", "segment"}
  assert rec["value"] == 60
  assert rec["route"] == "00000abc--deadbeef01"
  assert rec["segment"] == 0
  assert rec["crop_img"] == f"crops/{rec['frame_idx']:06d}_60.jpg"
  assert rec["ctx_img"] == f"ctx/{rec['frame_idx']:06d}_60.jpg"

  assert (out_dir / rec["crop_img"]).exists()
  assert (out_dir / rec["ctx_img"]).exists()

  timing_lines = (out_dir / "timing.jsonl").read_text().strip().splitlines()
  assert len(timing_lines) == 4  # 4 sampled frames at sample_hz=4 over a 20-frame/20fps segment
  trec = json.loads(timing_lines[0])
  assert set(trec) == {"frame_idx", "det_ms", "cls_ms", "n_crops"}
  assert trec["n_crops"] >= 1

  assert summary["n_frames"] == 4
  assert summary["n_publishes"] == 1

  captured = capsys.readouterr()
  assert "publishes" in captured.out.lower()
