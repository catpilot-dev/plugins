"""2-stage speed-limit-sign pipeline core: detect -> multi-crop classify -> multi-frame confirm.

Model-agnostic: detector and classifier are injected callables (real ONNX
models in the replay harness, plain stubs in tests). Imports only numpy +
stdlib so it stays importable in light environments.
"""
from collections import Counter, deque
from dataclasses import dataclass
from typing import Callable

import numpy as np

Box = tuple[float, float, float, float]                        # x1,y1,x2,y2 full-frame px
DetectFn = Callable[[np.ndarray], list[tuple[Box, float]]]     # BGR frame -> [(box, conf)]
ClassifyFn = Callable[[np.ndarray], tuple[str, float]]         # BGR crop -> (label, conf); label "30".."120" or "reject"

REJECT = "reject"


@dataclass
class PipelineConfig:
  det_conf: float = 0.35
  expansions: tuple[float, ...] = (1.0, 1.15, 1.3)   # box scale factors for classifier crops
  cls_conf: float = 0.55
  min_box_px: int = 14
  aspect_range: tuple[float, float] = (0.65, 1.5)     # w/h
  confirm_votes: int = 3
  confirm_window_s: float = 2.5
  publish_cooldown_s: float = 20.0


class SignPipeline:
  def __init__(self, detect_fn: DetectFn, classify_fn: ClassifyFn,
               config: PipelineConfig = PipelineConfig()):
    self.detect_fn = detect_fn
    self.classify_fn = classify_fn
    self.config = config
    self._votes: dict[int, deque[float]] = {}     # value -> timestamps of recent votes
    self._cooldown_until: dict[int, float] = {}   # value -> suppress publishes before this t

  def _box_ok(self, box: Box, frame_w: int, frame_h: int, conf: float) -> bool:
    cfg = self.config
    if conf < cfg.det_conf:
      return False
    x1, y1, x2, y2 = box
    w, h = x2 - x1, y2 - y1
    if w < cfg.min_box_px or h < cfg.min_box_px:
      return False
    aspect = w / h
    if not (cfg.aspect_range[0] <= aspect <= cfg.aspect_range[1]):
      return False
    if x1 < 0 or y1 < 0 or x2 > frame_w or y2 > frame_h:
      return False
    return True

  def _crops(self, frame_bgr: np.ndarray, box: Box) -> list[np.ndarray]:
    """Center-preserving scaled crops, clamped to frame; skip if clamped area < 50% of requested."""
    frame_h, frame_w = frame_bgr.shape[:2]
    x1, y1, x2, y2 = box
    cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    w, h = x2 - x1, y2 - y1
    crops = []
    for e in self.config.expansions:
      ew, eh = w * e, h * e
      rx1, ry1 = cx - ew / 2.0, cy - eh / 2.0
      rx2, ry2 = cx + ew / 2.0, cy + eh / 2.0
      qx1, qy1 = max(0.0, rx1), max(0.0, ry1)
      qx2, qy2 = min(float(frame_w), rx2), min(float(frame_h), ry2)
      if (qx2 - qx1) * (qy2 - qy1) < 0.5 * ew * eh:
        continue
      ix1, iy1 = int(np.floor(qx1)), int(np.floor(qy1))
      ix2, iy2 = int(np.ceil(qx2)), int(np.ceil(qy2))
      if ix2 <= ix1 or iy2 <= iy1:
        continue
      crops.append(frame_bgr[iy1:iy2, ix1:ix2])
    return crops

  def _box_vote(self, frame_bgr: np.ndarray, box: Box) -> tuple[int, float] | None:
    """Classify all expansion crops; return (value, mean_conf) if a strict majority agrees."""
    crops = self._crops(frame_bgr, box)
    if not crops:
      return None
    results = [self.classify_fn(c) for c in crops]
    counts = Counter(label for label, _ in results if label != REJECT)
    if not counts:
      return None
    label, n = counts.most_common(1)[0]
    if n * 2 <= len(crops):  # strict majority of total crops
      return None
    confs = [conf for lab, conf in results if lab == label]
    mean_conf = float(np.mean(confs))
    if mean_conf < self.config.cls_conf:
      return None
    return int(label), mean_conf

  def process_frame(self, frame_bgr: np.ndarray, t: float, frame_idx: int) -> list[dict]:
    """Returns publish dicts: {"t","frame_idx","value","conf","box"} (imgs/route added by harness)."""
    cfg = self.config
    frame_h, frame_w = frame_bgr.shape[:2]
    publishes = []
    for box, det_conf in self.detect_fn(frame_bgr):
      if not self._box_ok(box, frame_w, frame_h, det_conf):
        continue
      vote = self._box_vote(frame_bgr, box)
      if vote is None:
        continue
      value, conf = vote
      if t < self._cooldown_until.get(value, -np.inf):
        continue
      dq = self._votes.setdefault(value, deque())
      dq.append(t)
      while dq and dq[0] < t - cfg.confirm_window_s:
        dq.popleft()
      if len(dq) >= cfg.confirm_votes:
        publishes.append({
          "t": t,
          "frame_idx": frame_idx,
          "value": value,
          "conf": float(conf),
          "box": [float(v) for v in box],
        })
        self._cooldown_until[value] = t + cfg.publish_cooldown_s
        dq.clear()
    return publishes
