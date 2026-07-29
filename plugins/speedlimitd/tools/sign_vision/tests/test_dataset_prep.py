"""Tests for TT100K -> YOLO det/cls dataset prep."""
import hashlib
import json
import random

import pytest

np = pytest.importorskip("numpy")
cv2 = pytest.importorskip("cv2")

from plugins.speedlimitd.tools.sign_vision.dataset_prep import (
  is_val_split,
  load_tt100k_annotations,
  main,
  partition_categories,
  sample_background_crop,
  yolo_det_label,
)


def _write_img(path, seed):
  rng = np.random.RandomState(seed)
  img = rng.randint(0, 256, (64, 64, 3)).astype(np.uint8)
  path.parent.mkdir(parents=True, exist_ok=True)
  cv2.imwrite(str(path), img)


def _make_fixture(root):
  """4 images: pl60+pl60, pl40, il80-only, no-signs."""
  imgs = {
    "1": {
      "id": "1", "path": "train/1.jpg",
      "objects": [
        {"category": "pl60", "bbox": {"xmin": 4, "ymin": 4, "xmax": 24, "ymax": 24}},
        {"category": "pl60", "bbox": {"xmin": 34, "ymin": 34, "xmax": 54, "ymax": 54}},
      ],
    },
    "2": {
      "id": "2", "path": "train/2.jpg",
      "objects": [{"category": "pl40", "bbox": {"xmin": 10, "ymin": 10, "xmax": 30, "ymax": 30}}],
    },
    "3": {
      "id": "3", "path": "train/3.jpg",
      "objects": [{"category": "il80", "bbox": {"xmin": 10, "ymin": 10, "xmax": 30, "ymax": 30}}],
    },
    "4": {"id": "4", "path": "train/4.jpg", "objects": []},
  }
  ann = {"imgs": imgs, "types": ["pl60", "pl40", "il80"]}
  root.mkdir(parents=True, exist_ok=True)
  (root / "annotations_all.json").write_text(json.dumps(ann))
  for img_id, meta in imgs.items():
    _write_img(root / meta["path"], seed=int(img_id))
  return ann


def test_load_tt100k_annotations(tmp_path):
  ann = _make_fixture(tmp_path)
  loaded = load_tt100k_annotations(tmp_path)
  assert loaded == ann
  assert set(loaded["imgs"]) == {"1", "2", "3", "4"}
  assert loaded["imgs"]["1"]["objects"][0]["category"] == "pl60"


def test_load_tt100k_annotations_fallback_name(tmp_path):
  ann = {"imgs": {}, "types": []}
  tmp_path.mkdir(exist_ok=True)
  (tmp_path / "annotations.json").write_text(json.dumps(ann))
  assert load_tt100k_annotations(tmp_path) == ann


def test_load_tt100k_annotations_missing_raises(tmp_path):
  with pytest.raises(FileNotFoundError):
    load_tt100k_annotations(tmp_path)


def test_partition_categories_min_crops_2():
  counts = {"pl60": 2, "pl40": 1, "il80": 1}
  value_classes, reject_source_classes = partition_categories(counts, min_crops=2)
  assert value_classes == {"pl60"}
  assert reject_source_classes == {"pl40", "il80"}


def test_partition_categories_partitions_every_category():
  counts = {"pl60": 5, "pl30": 200, "pr40": 9, "pm55": 3}
  value_classes, reject_source_classes = partition_categories(counts, min_crops=150)
  assert value_classes == {"pl30"}
  assert reject_source_classes == {"pl60", "pr40", "pm55"}
  assert value_classes | reject_source_classes == set(counts)
  assert value_classes & reject_source_classes == set()


def test_yolo_det_label_hand_computed():
  # img 100x200, box (10,20,30,60) -> cx=0.2, cy=0.2, w=0.2, h=0.2
  label = yolo_det_label(100, 200, [(10, 20, 30, 60)])
  assert label == "0 0.200000 0.200000 0.200000 0.200000"


def test_yolo_det_label_multi_box_multi_line():
  label = yolo_det_label(100, 100, [(0, 0, 10, 10), (50, 50, 60, 60)])
  lines = label.split("\n")
  assert len(lines) == 2
  assert lines[0] == "0 0.050000 0.050000 0.100000 0.100000"
  assert lines[1] == "0 0.550000 0.550000 0.100000 0.100000"


def test_yolo_det_label_empty_boxes():
  assert yolo_det_label(100, 100, []) == ""


def test_is_val_split_matches_hash_formula():
  for img_id in ["1", "2", "3", "4", "abc"]:
    h = int(hashlib.md5(img_id.encode()).hexdigest(), 16)
    expected = (h % 1000) < 0.5 * 1000
    assert is_val_split(img_id, 0.5) == expected


def test_sample_background_crop_avoids_gt_box_covering_whole_image():
  rng = random.Random(0)
  boxes = [(0.0, 0.0, 300.0, 300.0)]  # covers everything -> no valid region, ever
  result = sample_background_crop(rng, 300, 300, boxes, size_range=(64, 160), max_attempts=10)
  assert result is None


def test_sample_background_crop_finds_free_region():
  rng = random.Random(0)
  boxes = [(0.0, 0.0, 50.0, 50.0)]  # small corner box, plenty of free space in a 300x300 image
  result = sample_background_crop(rng, 300, 300, boxes, size_range=(64, 160), max_attempts=10)
  assert result is not None
  x1, y1, x2, y2 = result
  assert 64 <= (x2 - x1) <= 160
  assert 64 <= (y2 - y1) <= 160
  gt = boxes[0]
  overlap = not (x2 <= gt[0] or x1 >= gt[2] or y2 <= gt[1] or y1 >= gt[3])
  assert not overlap


def test_end_to_end_main_layout(tmp_path):
  tt100k = tmp_path / "tt100k"
  out = tmp_path / "out"
  ann = _make_fixture(tt100k)

  main(["--tt100k", str(tt100k), "--out", str(out), "--min-crops", "2", "--val-frac", "0.5"])

  splits = {img_id: ("val" if is_val_split(img_id, 0.5) else "train") for img_id in ann["imgs"]}

  # det layout: every image copied + labeled under its split
  for img_id, split in splits.items():
    assert (out / "det" / "images" / split / f"{img_id}.jpg").exists()
    assert (out / "det" / "labels" / split / f"{img_id}.txt").exists()

  label1 = (out / "det" / "labels" / splits["1"] / "1.txt").read_text()
  assert len(label1.strip().splitlines()) == 2
  label2 = (out / "det" / "labels" / splits["2"] / "2.txt").read_text()
  assert len(label2.strip().splitlines()) == 1
  # il80-only and signless images: no pl* positives -> empty label file
  assert (out / "det" / "labels" / splits["3"] / "3.txt").read_text() == ""
  assert (out / "det" / "labels" / splits["4"] / "4.txt").read_text() == ""

  # cls layout: pl60 (count 2 >= min_crops 2) -> value folder "60", 2 crops from image 1
  value_crops = sorted((out / "cls" / splits["1"] / "60").glob("1_*.jpg"))
  assert len(value_crops) == 2

  # pl40 (under min_crops) and il80 -> reject
  assert (out / "cls" / splits["2"] / "reject" / "2_0.jpg").exists()
  assert (out / "cls" / splits["3"] / "reject" / "3_0.jpg").exists()

  # image 4 (no signs): 2 background reject crops if train, none if val
  bg_crops = list((out / "cls" / "train" / "reject").glob("4_bg*.jpg"))
  if splits["4"] == "train":
    assert len(bg_crops) == 2
  else:
    assert len(bg_crops) == 0

  det_yaml = (out / "det.yaml").read_text()
  assert f"path: {out / 'det'}" in det_yaml
  assert "train: images/train" in det_yaml
  assert "val: images/val" in det_yaml
  assert "names: {0: sign}" in det_yaml

  classes = json.loads((out / "classes.json").read_text())
  assert classes == {"values": [60], "counts": {"60": 2}, "min_crops": 2}


def test_main_prints_progress_and_inventory(tmp_path, capsys):
  tt100k = tmp_path / "tt100k"
  out = tmp_path / "out"
  _make_fixture(tt100k)
  main(["--tt100k", str(tt100k), "--out", str(out), "--min-crops", "2", "--val-frac", "0.5"])
  captured = capsys.readouterr()
  assert "Class inventory" in captured.out
  assert "60" in captured.out
  assert "reject" in captured.out
