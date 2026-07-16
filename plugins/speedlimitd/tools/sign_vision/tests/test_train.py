"""Tests for sign_vision train/export wrapper."""
import pytest

from plugins.speedlimitd.tools.sign_vision.train import build_train_kwargs


def test_build_train_kwargs_det():
  """Detection: yolo11n.pt @ 256, batch -1."""
  kwargs = build_train_kwargs("det", "/path/to/det.yaml", 60, "0")
  assert kwargs == {
    "model": "yolo11n.pt",
    "data": "/path/to/det.yaml",
    "imgsz": 256,
    "epochs": 60,
    "device": "0",
    "batch": -1,
  }


def test_build_train_kwargs_cls():
  """Classification: yolo11n-cls.pt @ 128, batch 64."""
  kwargs = build_train_kwargs("cls", "/path/to/cls", 40, "cpu")
  assert kwargs == {
    "model": "yolo11n-cls.pt",
    "data": "/path/to/cls",
    "imgsz": 128,
    "epochs": 40,
    "device": "cpu",
    "batch": 64,
  }


def test_build_train_kwargs_unknown_kind():
  """Unknown kind raises ValueError."""
  with pytest.raises(ValueError, match="Unknown kind"):
    build_train_kwargs("unknown", "/path", 60, "0")
