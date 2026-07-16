"""Train/export wrapper for YOLO11 speed-limit-sign detection/classification.

CLI:
  uv run python train.py det --data ~/catpilot-dev/datasets/sign_vision/det.yaml --epochs 60
  uv run python train.py cls --data ~/catpilot-dev/datasets/sign_vision/cls --epochs 40
  uv run python train.py export --weights <best.pt> --kind det|cls

Imports ultralytics only inside main() so build_train_kwargs can be tested
without the library.
"""
import argparse
import shutil
from pathlib import Path

# Image sizes for each model kind (single source of truth)
IMGSZ = {"det": 256, "cls": 128}


def build_train_kwargs(kind: str, data: str, epochs: int, device: str) -> dict:
  """Build YOLO training kwargs for the given kind.

  Args:
    kind: "det" (detection) or "cls" (classification)
    data: path to data.yaml (det) or data folder (cls)
    epochs: number of training epochs
    device: device spec, e.g. "0" or "cpu"

  Returns:
    dict with keys: model, data, imgsz, epochs, device, batch

  Raises:
    ValueError: if kind is unknown
  """
  if kind == "det":
    return {
      "model": "yolo11n.pt",
      "data": data,
      "imgsz": IMGSZ["det"],
      "epochs": epochs,
      "device": device,
      "batch": -1,
    }
  elif kind == "cls":
    return {
      "model": "yolo11n-cls.pt",
      "data": data,
      "imgsz": IMGSZ["cls"],
      "epochs": epochs,
      "device": device,
      "batch": 64,
    }
  else:
    raise ValueError(f"Unknown kind: {kind}")


def _train_handler(args) -> None:
  """Handle train subcommand."""
  from ultralytics import YOLO

  kwargs = build_train_kwargs(args.kind, args.data, args.epochs, args.device)
  model = kwargs.pop("model")
  YOLO(model).train(**kwargs)


def _export_handler(args) -> None:
  """Handle export subcommand."""
  from ultralytics import YOLO

  imgsz = IMGSZ[args.kind]
  yolo = YOLO(args.weights)
  yolo.export(format="onnx", imgsz=imgsz)

  # YOLO.export() produces model_name.onnx in the same directory as weights
  weights_path = Path(args.weights)
  onnx_src = weights_path.parent / f"{weights_path.stem}.onnx"

  # Copy to standard location
  models_dir = Path.home() / "catpilot-dev" / "datasets" / "sign_vision" / "models"
  models_dir.mkdir(parents=True, exist_ok=True)
  onnx_dst = models_dir / f"{args.kind}.onnx"
  shutil.copy(onnx_src, onnx_dst)
  print(f"Exported {args.kind} ONNX to {onnx_dst}")


def main(argv=None) -> None:
  """CLI entry point."""
  parser = argparse.ArgumentParser(
    description="Train/export YOLO11 sign-vision models (det@256, cls@128)"
  )
  subparsers = parser.add_subparsers(dest="cmd", required=True)

  # det subcommand
  det_p = subparsers.add_parser("det", help="Train detection model")
  det_p.add_argument("--data", required=True, help="data.yaml for detection")
  det_p.add_argument("--epochs", type=int, required=True, help="Training epochs")
  det_p.add_argument("--device", default="0", help="Device spec (default: 0)")
  det_p.set_defaults(kind="det", handler=_train_handler)

  # cls subcommand
  cls_p = subparsers.add_parser("cls", help="Train classification model")
  cls_p.add_argument("--data", required=True, help="data/ folder for classification")
  cls_p.add_argument("--epochs", type=int, required=True, help="Training epochs")
  cls_p.add_argument("--device", default="0", help="Device spec (default: 0)")
  cls_p.set_defaults(kind="cls", handler=_train_handler)

  # export subcommand
  export_p = subparsers.add_parser("export", help="Export trained model to ONNX")
  export_p.add_argument("--weights", required=True, help="Path to best.pt")
  export_p.add_argument("--kind", choices=["det", "cls"], required=True, help="Model kind")
  export_p.set_defaults(handler=_export_handler)

  args = parser.parse_args(argv)
  args.handler(args)


if __name__ == "__main__":
  main()
