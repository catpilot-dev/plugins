"""Tests for sign_vision train/export wrapper."""
import pytest
from unittest.mock import patch, MagicMock

from plugins.speedlimitd.tools.sign_vision.train import build_train_kwargs, main


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


def test_cli_det_top_level():
  """CLI: det is a top-level subcommand, not nested under train."""
  with patch("plugins.speedlimitd.tools.sign_vision.train._train_handler") as mock_handler:
    main(["det", "--data", "x.yaml", "--epochs", "2"])
    mock_handler.assert_called_once()
    args = mock_handler.call_args[0][0]
    assert args.kind == "det"
    assert args.data == "x.yaml"
    assert args.epochs == 2


def test_cli_cls_top_level():
  """CLI: cls is a top-level subcommand."""
  with patch("plugins.speedlimitd.tools.sign_vision.train._train_handler") as mock_handler:
    main(["cls", "--data", "y.yaml", "--epochs", "3"])
    mock_handler.assert_called_once()
    args = mock_handler.call_args[0][0]
    assert args.kind == "cls"
    assert args.data == "y.yaml"
    assert args.epochs == 3


def test_cli_export_top_level():
  """CLI: export is a top-level subcommand."""
  with patch("plugins.speedlimitd.tools.sign_vision.train._export_handler") as mock_handler:
    main(["export", "--weights", "best.pt", "--kind", "det"])
    mock_handler.assert_called_once()
    args = mock_handler.call_args[0][0]
    assert args.weights == "best.pt"
    assert args.kind == "det"
