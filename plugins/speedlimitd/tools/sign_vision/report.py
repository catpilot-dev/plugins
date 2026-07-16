"""Precision/timing report for a sign_vision harness run.

Aggregates publishes.jsonl (what the pipeline published), review.jsonl (the
human-reviewed verdicts, produced by review.py) and timing.jsonl (per-frame
wall-clock timing from the harness) into a single report: overall precision,
a per-published-value breakdown, and detect/classify timing stats.

`compute_report` is pure stdlib (no numpy/cv2) so it's importable and
unit-testable in the bare repo env.

CLI:
  uv run python report.py --run ~/catpilot-dev/datasets/sign_vision/runs/<name>
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

SKIP = "__skip__"
VERDICT_KEYS = ("correct", "wrong_value", "false_positive")


def _percentile(values: list[float], pct: float) -> float:
  """Linear-interpolated percentile (numpy's default 'linear' method), pure stdlib."""
  s = sorted(values)
  n = len(s)
  if n == 1:
    return s[0]
  idx = (pct / 100.0) * (n - 1)
  lo = int(idx)
  hi = min(lo + 1, n - 1)
  frac = idx - lo
  return s[lo] + (s[hi] - s[lo]) * frac


def compute_report(publishes: list[dict], reviews: list[dict], timings: list[dict]) -> dict:
  """Pure aggregation over the three run jsonl files (already parsed into lists of
  dicts). Returns a dict with EXACTLY these keys:
    n_publishes, n_reviewed, precision (correct / reviewed-non-skip, None if 0
    reviewed), per_value (dict keyed by published value ->
    {"correct","wrong_value","false_positive"} counts), mean_det_ms, mean_cls_ms,
    p95_det_ms (mean_det_ms/mean_cls_ms/p95_det_ms are all None when timings is empty).
  """
  n_publishes = len(publishes)
  value_by_crop = {p["crop_img"]: p["value"] for p in publishes}

  non_skip = [r for r in reviews if r.get("verdict") != SKIP]
  n_reviewed = len(non_skip)

  correct = sum(1 for r in non_skip if r["verdict"] == "correct")
  precision = (correct / n_reviewed) if n_reviewed else None

  per_value: dict[int, dict[str, int]] = {}
  for r in non_skip:
    value = value_by_crop.get(r["crop_img"])
    if value is None:
      continue
    bucket = per_value.setdefault(value, {k: 0 for k in VERDICT_KEYS})
    if r["verdict"] in bucket:
      bucket[r["verdict"]] += 1

  det_ms = [t["det_ms"] for t in timings]
  cls_ms = [t["cls_ms"] for t in timings]
  mean_det_ms = (sum(det_ms) / len(det_ms)) if det_ms else None
  mean_cls_ms = (sum(cls_ms) / len(cls_ms)) if cls_ms else None
  p95_det_ms = _percentile(det_ms, 95) if det_ms else None

  return {
    "n_publishes": n_publishes,
    "n_reviewed": n_reviewed,
    "precision": precision,
    "per_value": per_value,
    "mean_det_ms": mean_det_ms,
    "mean_cls_ms": mean_cls_ms,
    "p95_det_ms": p95_det_ms,
  }


def _load_jsonl(path: Path) -> list[dict]:
  if not path.exists():
    return []
  lines = path.read_text().strip().splitlines()
  return [json.loads(line) for line in lines if line.strip()]


def _fmt(value, suffix: str = "") -> str:
  return f"{value:.2f}{suffix}" if value is not None else "n/a"


def _print_table(report: dict) -> None:
  print(f"n_publishes:  {report['n_publishes']}")
  print(f"n_reviewed:   {report['n_reviewed']}")
  precision = report["precision"]
  print(f"precision:    {precision:.3f}" if precision is not None else "precision:    n/a")
  print("per_value:")
  for value in sorted(report["per_value"]):
    counts = report["per_value"][value]
    print(f"  {value:>4}: correct={counts['correct']:<3} "
          f"wrong_value={counts['wrong_value']:<3} false_positive={counts['false_positive']:<3}")
  print(f"mean_det_ms:  {_fmt(report['mean_det_ms'])}")
  print(f"mean_cls_ms:  {_fmt(report['mean_cls_ms'])}")
  print(f"p95_det_ms:   {_fmt(report['p95_det_ms'])}")


def main(argv: list[str] | None = None) -> None:
  parser = argparse.ArgumentParser(description="Precision/timing report for a sign_vision run")
  parser.add_argument("--run", required=True,
                       help="run dir containing publishes.jsonl/review.jsonl/timing.jsonl")
  args = parser.parse_args(argv)
  run_dir = Path(args.run).expanduser()

  publishes = _load_jsonl(run_dir / "publishes.jsonl")
  reviews = _load_jsonl(run_dir / "review.jsonl")
  timings = _load_jsonl(run_dir / "timing.jsonl")

  report = compute_report(publishes, reviews, timings)
  _print_table(report)
  (run_dir / "report.json").write_text(json.dumps(report, indent=2))


if __name__ == "__main__":
  main()
