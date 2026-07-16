"""TT100K 2021 -> YOLO det/cls dataset prep for the sign_vision PoC.

Standalone CLI + pure functions. Parses TT100K annotations, partitions
categories into `pl*` "value" classes (kept if they clear --min-crops) vs a
reject bucket (every other category present: sub-threshold `pl*`, plus
`il*`/`pr*`/`pm*`/`ph*`/`pw*`/`pn*`), and writes:
  - a YOLO detection dataset (single class: "sign", positives = pl* boxes only)
  - a YOLO classification dataset (one folder per speed value, plus "reject",
    which also gets 2 random background crops per train image)
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import random
from pathlib import Path

import cv2

Box = tuple[float, float, float, float]   # xmin, ymin, xmax, ymax, pixel coords

ANNOTATION_FILENAMES = ("annotations_all.json", "annotations.json")
CLS_PREFIXES = ("pl", "il", "pr", "pm", "ph", "pw", "pn")
BG_CROPS_PER_TRAIN_IMAGE = 2
BG_SIZE_RANGE = (64, 160)
BG_MAX_ATTEMPTS = 10
CLS_MARGIN = 1.2
PROGRESS_EVERY = 500


def load_tt100k_annotations(root: Path) -> dict:
  """Parses annotations_all.json (2021) or annotations.json (fallback) under root."""
  root = Path(root)
  for name in ANNOTATION_FILENAMES:
    path = root / name
    if path.exists():
      with open(path) as f:
        return json.load(f)
  raise FileNotFoundError(f"no {' or '.join(ANNOTATION_FILENAMES)} under {root}")


def _bbox(obj: dict) -> Box:
  b = obj["bbox"]
  return (float(b["xmin"]), float(b["ymin"]), float(b["xmax"]), float(b["ymax"]))


def category_counts(annotations: dict) -> dict[str, int]:
  """Counts object instances per category, restricted to speed-limit-adjacent prefixes."""
  counts: dict[str, int] = {}
  for meta in annotations["imgs"].values():
    for obj in meta.get("objects", []):
      cat = obj["category"]
      if cat.startswith(CLS_PREFIXES):
        counts[cat] = counts.get(cat, 0) + 1
  return counts


def partition_categories(counts: dict[str, int], min_crops: int) -> tuple[set[str], set[str]]:
  """(value_classes, reject_source_classes) partitioning every category in counts:
  pl* categories with count >= min_crops are values; everything else (sub-threshold
  pl*, il*/pr*/pm*/ph*/pw*/pn*) is a reject source."""
  value_classes = {cat for cat, n in counts.items() if cat.startswith("pl") and n >= min_crops}
  reject_source_classes = set(counts) - value_classes
  return value_classes, reject_source_classes


def yolo_det_label(img_w: float, img_h: float, boxes: list[Box]) -> str:
  """'0 cx cy w h' lines (normalized to [0,1]), one per box, newline-joined. Empty -> ""."""
  lines = []
  for xmin, ymin, xmax, ymax in boxes:
    cx = (xmin + xmax) / 2.0 / img_w
    cy = (ymin + ymax) / 2.0 / img_h
    w = (xmax - xmin) / img_w
    h = (ymax - ymin) / img_h
    lines.append(f"0 {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")
  return "\n".join(lines)


def is_val_split(img_id: str, val_frac: float) -> bool:
  """Deterministic split by image-id hash."""
  h = int(hashlib.md5(img_id.encode()).hexdigest(), 16)
  return (h % 1000) < val_frac * 1000


def expand_box(box: Box, margin: float, img_w: int, img_h: int) -> tuple[int, int, int, int]:
  """Scales box by margin around its center, clamped to [0,img_w]x[0,img_h] int px."""
  xmin, ymin, xmax, ymax = box
  cx, cy = (xmin + xmax) / 2.0, (ymin + ymax) / 2.0
  w, h = (xmax - xmin) * margin, (ymax - ymin) * margin
  x1, y1 = max(0.0, cx - w / 2.0), max(0.0, cy - h / 2.0)
  x2, y2 = min(float(img_w), cx + w / 2.0), min(float(img_h), cy + h / 2.0)
  ix1, iy1 = int(math.floor(x1)), int(math.floor(y1))
  ix2, iy2 = int(math.ceil(x2)), int(math.ceil(y2))
  ix2 = max(ix2, ix1 + 1)
  iy2 = max(iy2, iy1 + 1)
  return ix1, iy1, min(ix2, img_w), min(iy2, img_h)


def _overlaps(a: Box, b: Box) -> bool:
  ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
  ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
  return ix2 > ix1 and iy2 > iy1


def sample_background_crop(rng: random.Random, img_w: int, img_h: int, boxes: list[Box],
                            size_range: tuple[int, int] = BG_SIZE_RANGE,
                            max_attempts: int = BG_MAX_ATTEMPTS) -> tuple[int, int, int, int] | None:
  """Random square crop (side in size_range px, clamped to image) that overlaps none of
  boxes (any IoU>0 -> resample); None if no attempt found a free region."""
  lo, hi = size_range
  for _ in range(max_attempts):
    side = min(rng.randint(lo, hi), img_w, img_h)
    if side < 1:
      return None
    x1 = rng.randint(0, img_w - side)
    y1 = rng.randint(0, img_h - side)
    candidate = (float(x1), float(y1), float(x1 + side), float(y1 + side))
    if not any(_overlaps(candidate, b) for b in boxes):
      return x1, y1, x1 + side, y1 + side
  return None


def run(tt100k_root: Path, out_root: Path, min_crops: int, val_frac: float) -> dict:
  """Full TT100K -> YOLO det/cls conversion. Returns the classes.json dict."""
  tt100k_root = Path(tt100k_root)
  out_root = Path(out_root)
  annotations = load_tt100k_annotations(tt100k_root)
  imgs = annotations["imgs"]
  counts = category_counts(annotations)
  value_classes, reject_source_classes = partition_categories(counts, min_crops)

  for split in ("train", "val"):
    (out_root / "det" / "images" / split).mkdir(parents=True, exist_ok=True)
    (out_root / "det" / "labels" / split).mkdir(parents=True, exist_ok=True)
    (out_root / "cls" / split / "reject").mkdir(parents=True, exist_ok=True)
    for cat in value_classes:
      (out_root / "cls" / split / cat[2:]).mkdir(parents=True, exist_ok=True)

  img_ids = sorted(imgs)
  total = len(img_ids)
  for i, img_id in enumerate(img_ids):
    if i and i % PROGRESS_EVERY == 0:
      print(f"[{i}/{total}] images processed")
    meta = imgs[img_id]
    split = "val" if is_val_split(img_id, val_frac) else "train"
    img_path = tt100k_root / meta["path"]
    img = cv2.imread(str(img_path))
    if img is None:
      print(f"warning: could not read {img_path}, skipping")
      continue
    img_h, img_w = img.shape[:2]
    objects = meta.get("objects", [])
    all_boxes = [_bbox(o) for o in objects]
    pl_boxes = [_bbox(o) for o in objects if o["category"].startswith("pl")]

    cv2.imwrite(str(out_root / "det" / "images" / split / f"{img_id}.jpg"), img)
    label = yolo_det_label(img_w, img_h, pl_boxes)
    (out_root / "det" / "labels" / split / f"{img_id}.txt").write_text(
      label + "\n" if label else "")

    for obj_idx, obj in enumerate(objects):
      cat = obj["category"]
      if not cat.startswith(CLS_PREFIXES):
        continue
      x1, y1, x2, y2 = expand_box(_bbox(obj), CLS_MARGIN, img_w, img_h)
      crop = img[y1:y2, x1:x2]
      folder = cat[2:] if cat in value_classes else "reject"
      cv2.imwrite(str(out_root / "cls" / split / folder / f"{img_id}_{obj_idx}.jpg"), crop)

    if split == "train":
      rng = random.Random(img_id)
      for k in range(BG_CROPS_PER_TRAIN_IMAGE):
        bg = sample_background_crop(rng, img_w, img_h, all_boxes)
        if bg is None:
          continue
        x1, y1, x2, y2 = bg
        crop = img[y1:y2, x1:x2]
        cv2.imwrite(str(out_root / "cls" / "train" / "reject" / f"{img_id}_bg{k}.jpg"), crop)

  det_yaml = (
    f"path: {out_root / 'det'}\n"
    f"train: images/train\n"
    f"val: images/val\n"
    f"names: {{0: sign}}\n"
  )
  (out_root / "det.yaml").write_text(det_yaml)

  classes = {
    "values": sorted(int(c[2:]) for c in value_classes),
    "counts": {c[2:]: counts[c] for c in value_classes},
    "min_crops": min_crops,
  }
  (out_root / "classes.json").write_text(json.dumps(classes, indent=2))

  print("\nClass inventory:")
  print(f"{'class':>10} {'count':>8}")
  for cat in sorted(value_classes, key=lambda c: int(c[2:])):
    print(f"{cat[2:]:>10} {counts[cat]:>8}")
  reject_total = sum(counts.get(c, 0) for c in reject_source_classes)
  print(f"{'reject':>10} {reject_total:>8}")

  return classes


def main(argv: list[str] | None = None) -> None:
  parser = argparse.ArgumentParser(description="TT100K -> YOLO det/cls dataset prep")
  parser.add_argument("--tt100k", required=True, help="TT100K 2021 root (has annotations_all.json)")
  parser.add_argument("--out", required=True, help="output dataset root")
  parser.add_argument("--min-crops", type=int, default=150,
                       help="min pl* instances to keep as its own value class")
  parser.add_argument("--val-frac", type=float, default=0.1)
  args = parser.parse_args(argv)
  run(Path(args.tt100k).expanduser(), Path(args.out).expanduser(), args.min_crops, args.val_frac)


if __name__ == "__main__":
  main()
