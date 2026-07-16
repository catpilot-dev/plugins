"""Offline route-replay harness for the sign_vision PoC.

Decodes recorded C3 fcamera.hevc footage frame-by-frame (via PyAV), samples
frames at a target rate, and runs each through the detect/classify pipeline
core (pipeline.py). On every publish it writes a review crop + a full-frame
context image and appends a record to publishes.jsonl; per-frame wall-clock
timing is appended to timing.jsonl.

CLI:
  uv run python harness.py --route-dir ~/catpilot-dev/datasets/routes/<route> \\
    --det-onnx models/det.onnx --cls-onnx models/cls.onnx \\
    --roi right --sample-hz 2 --out ~/catpilot-dev/datasets/sign_vision/runs/<name>
"""
from __future__ import annotations

import argparse
import json
import re
import time
from pathlib import Path
from typing import Callable, Iterator

import av
import cv2
import numpy as np

from plugins.speedlimitd.tools.sign_vision.dataset_prep import expand_box
from plugins.speedlimitd.tools.sign_vision.pipeline import (
  Box,
  ClassifyFn,
  DetectFn,
  PipelineConfig,
  SignPipeline,
)

ROI_PRESETS: dict[str, tuple[float, float, float, float]] = {
  "full": (0.0, 0.0, 1.0, 1.0),
  "right": (0.45, 0.05, 1.0, 0.80),
}

CROP_MARGIN = 1.2       # +20% margin on the crop saved for review
CTX_WIDTH = 960          # downscale ctx images to this width
CTX_BOX_THICKNESS = 3

_SEGMENT_RE = re.compile(r"^(.*)--(\d+)$")


def _segment_dirs(route_dir: Path) -> list[tuple[int, Path, str]]:
  """(segment_int, dir_path, route_name) for children of route_dir whose name ends in
  `--<int>`, sorted by that int. route_name is the child dir name with the `--<int>`
  suffix stripped."""
  entries = []
  for child in Path(route_dir).iterdir():
    if not child.is_dir():
      continue
    m = _SEGMENT_RE.match(child.name)
    if not m:
      continue
    route_name, seg_str = m.group(1), m.group(2)
    entries.append((int(seg_str), child, route_name))
  entries.sort(key=lambda e: e[0])
  return entries


def iter_route_frames(
  route_dir: Path, sample_hz: float, fps: float = 20.0
) -> Iterator[tuple[np.ndarray, float, int, int]]:
  """Yields (frame_bgr, t_seconds, frame_idx, segment) for sampled frames across every
  segment dir under route_dir, in segment order. frame_idx/t are cumulative across
  segments (a global decoded-frame counter, t = frame_idx / fps). Segments missing
  fcamera.hevc are skipped with a printed warning. Segments whose fcamera.hevc is present
  but fails to open/decode (e.g. truncated at ignition-off, as C3 realdata's final segment
  routinely is) also log a warning and are skipped, without aborting the rest of the route;
  frames already yielded before the error are kept, and the cumulative frame counter
  continues into the next segment."""
  stride = max(1, round(fps / sample_hz))
  global_idx = 0
  for segment, seg_dir, _route_name in _segment_dirs(route_dir):
    hevc_path = seg_dir / "fcamera.hevc"
    if not hevc_path.exists():
      print(f"warning: {hevc_path} missing, skipping segment {segment}")
      continue
    try:
      with av.open(str(hevc_path)) as container:
        for frame in container.decode(video=0):
          if global_idx % stride == 0:
            frame_bgr = frame.to_ndarray(format="bgr24")
            t = global_idx / fps
            yield frame_bgr, t, global_idx, segment
          global_idx += 1
    except (av.error.FFmpegError, OSError) as e:
      print(f"warning: decode error in {hevc_path}: {e}; skipping rest of segment")
      continue


def _timed(fn: Callable) -> tuple[Callable, list[float]]:
  """Wraps fn so every call appends its wall-clock duration (ms) to the returned list."""
  times: list[float] = []

  def wrapped(*args, **kwargs):
    t0 = time.perf_counter()
    result = fn(*args, **kwargs)
    times.append((time.perf_counter() - t0) * 1000.0)
    return result

  return wrapped, times


def _save_crop(frame_bgr: np.ndarray, box: Box, out_path: Path) -> None:
  h, w = frame_bgr.shape[:2]
  x1, y1, x2, y2 = expand_box(box, CROP_MARGIN, w, h)
  out_path.parent.mkdir(parents=True, exist_ok=True)
  cv2.imwrite(str(out_path), frame_bgr[y1:y2, x1:x2])


def _save_ctx(frame_bgr: np.ndarray, box: Box, out_path: Path) -> None:
  h, w = frame_bgr.shape[:2]
  x1, y1, x2, y2 = (int(round(v)) for v in box)
  ctx = frame_bgr.copy()
  cv2.rectangle(ctx, (x1, y1), (x2, y2), (0, 0, 255), CTX_BOX_THICKNESS)
  if w > CTX_WIDTH:
    scale = CTX_WIDTH / w
    ctx = cv2.resize(ctx, (CTX_WIDTH, max(1, int(round(h * scale)))))
  out_path.parent.mkdir(parents=True, exist_ok=True)
  cv2.imwrite(str(out_path), ctx)


def run(
  route_dir: Path,
  out_dir: Path,
  detect_fn: DetectFn,
  classify_fn: ClassifyFn,
  sample_hz: float,
  route_name: str | None = None,
  config: PipelineConfig | None = None,
) -> dict:
  """Testable core: replays route_dir through the pipeline, writing publishes.jsonl,
  timing.jsonl, and review images under out_dir. Returns a summary dict."""
  route_dir = Path(route_dir)
  out_dir = Path(out_dir)
  crops_dir = out_dir / "crops"
  ctx_dir = out_dir / "ctx"
  crops_dir.mkdir(parents=True, exist_ok=True)
  ctx_dir.mkdir(parents=True, exist_ok=True)

  if route_name is None:
    segments = _segment_dirs(route_dir)
    route_name = segments[0][2] if segments else route_dir.name

  timed_detect, det_times = _timed(detect_fn)
  timed_classify, cls_times = _timed(classify_fn)
  pipeline = SignPipeline(timed_detect, timed_classify, config or PipelineConfig())

  n_frames = 0
  n_publishes = 0
  all_det_ms: list[float] = []
  all_cls_ms: list[float] = []

  publishes_path = out_dir / "publishes.jsonl"
  timing_path = out_dir / "timing.jsonl"

  with open(publishes_path, "w") as pub_f, open(timing_path, "w") as time_f:
    for frame_bgr, t, frame_idx, segment in iter_route_frames(route_dir, sample_hz):
      n_frames += 1
      det_times.clear()
      cls_times.clear()

      publishes = pipeline.process_frame(frame_bgr, t, frame_idx)

      det_ms = sum(det_times)
      cls_ms = sum(cls_times)
      all_det_ms.append(det_ms)
      all_cls_ms.append(cls_ms)
      time_f.write(json.dumps({
        "frame_idx": frame_idx,
        "det_ms": det_ms,
        "cls_ms": cls_ms,
        "n_crops": len(cls_times),
      }) + "\n")

      for pub in publishes:
        crop_name = f"{frame_idx:06d}_{pub['value']}.jpg"
        _save_crop(frame_bgr, pub["box"], crops_dir / crop_name)
        _save_ctx(frame_bgr, pub["box"], ctx_dir / crop_name)
        record = dict(pub)
        record["crop_img"] = f"crops/{crop_name}"
        record["ctx_img"] = f"ctx/{crop_name}"
        record["route"] = route_name
        record["segment"] = segment
        pub_f.write(json.dumps(record) + "\n")
        n_publishes += 1

  mean_det_ms = sum(all_det_ms) / len(all_det_ms) if all_det_ms else 0.0
  mean_cls_ms = sum(all_cls_ms) / len(all_cls_ms) if all_cls_ms else 0.0
  print(f"frames processed: {n_frames}")
  print(f"publishes: {n_publishes}")
  print(f"mean det_ms: {mean_det_ms:.2f}  mean cls_ms: {mean_cls_ms:.2f}")

  return {
    "n_frames": n_frames,
    "n_publishes": n_publishes,
    "mean_det_ms": mean_det_ms,
    "mean_cls_ms": mean_cls_ms,
  }


def make_onnx_runners(
  det_onnx: Path, cls_onnx: Path, roi: tuple[float, float, float, float], det_conf: float
) -> tuple[DetectFn, ClassifyFn]:
  """Builds (DetectFn, ClassifyFn) backed by ultralytics YOLO ONNX predictors. DetectFn
  runs detection on the ROI crop (normalized rect -> px) at imgsz=256 and maps boxes back
  to full-frame coords; ClassifyFn runs classification at imgsz=128."""
  from ultralytics import YOLO

  # ONNX files carry no task metadata — without an explicit task, ultralytics
  # guesses (and guessed "detect" for the classifier in the first real run).
  det_model = YOLO(str(det_onnx), task="detect")
  cls_model = YOLO(str(cls_onnx), task="classify")
  rx1, ry1, rx2, ry2 = roi

  def detect_fn(frame_bgr: np.ndarray) -> list[tuple[Box, float]]:
    h, w = frame_bgr.shape[:2]
    ox1, oy1, ox2, oy2 = int(rx1 * w), int(ry1 * h), int(rx2 * w), int(ry2 * h)
    roi_crop = frame_bgr[oy1:oy2, ox1:ox2]
    if roi_crop.size == 0:
      return []
    results = det_model.predict(roi_crop, imgsz=256, conf=det_conf, verbose=False)
    boxes: list[tuple[Box, float]] = []
    for res in results:
      if res.boxes is None:
        continue
      for b in res.boxes:
        bx1, by1, bx2, by2 = (float(v) for v in b.xyxy[0].tolist())
        conf = float(b.conf[0])
        boxes.append(((bx1 + ox1, by1 + oy1, bx2 + ox1, by2 + oy1), conf))
    return boxes

  def classify_fn(crop_bgr: np.ndarray) -> tuple[str, float]:
    results = cls_model.predict(crop_bgr, imgsz=128, verbose=False)
    probs = results[0].probs
    top1 = int(probs.top1)
    return results[0].names[top1], float(probs.top1conf)

  return detect_fn, classify_fn


def main(argv: list[str] | None = None) -> None:
  parser = argparse.ArgumentParser(description="Offline route-replay harness for sign_vision")
  parser.add_argument("--route-dir", required=True, help="route dir containing --<int> segment subdirs")
  parser.add_argument("--det-onnx", required=True, help="path to detection ONNX model")
  parser.add_argument("--cls-onnx", required=True, help="path to classification ONNX model")
  parser.add_argument("--roi", choices=sorted(ROI_PRESETS), default="full",
                       help="ROI preset the detector runs on (default: full)")
  parser.add_argument("--sample-hz", type=float, default=2.0, help="frame sampling rate")
  parser.add_argument("--out", required=True, help="output run dir")
  parser.add_argument("--det-conf", type=float, default=PipelineConfig().det_conf)
  parser.add_argument("--route-name", default=None, help="override the route name in publishes.jsonl")
  args = parser.parse_args(argv)

  roi = ROI_PRESETS[args.roi]
  detect_fn, classify_fn = make_onnx_runners(
    Path(args.det_onnx).expanduser(), Path(args.cls_onnx).expanduser(), roi, args.det_conf)
  config = PipelineConfig(det_conf=args.det_conf)
  run(Path(args.route_dir).expanduser(), Path(args.out).expanduser(),
      detect_fn, classify_fn, args.sample_hz, route_name=args.route_name, config=config)


if __name__ == "__main__":
  main()
