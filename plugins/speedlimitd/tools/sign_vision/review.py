"""Single-key review tool for sign_vision publishes.

Walks publishes.jsonl one record at a time, shows the context frame with a
crop inset (cv2.imshow), and reads a single key per record:
  y        -> correct
  n        -> false_positive
  0-9 ENTER -> wrong_value(true_value=<typed digits>)
  s        -> skip this record (no verdict written; will re-appear next run)
  q        -> quit the review session

Verdicts are appended to review.jsonl incrementally as they are made, and a
resumed run skips any crop_img already present in review.jsonl.

`next_state` is the pure transition core (no cv2) -- unit-tested directly.
cv2 is only imported inside `_show_and_get_key`, so this module stays
importable without a display/GUI environment.

CLI:
  uv run python review.py --run ~/catpilot-dev/datasets/sign_vision/runs/<name>
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

QUIT = {"verdict": "__quit__"}


def next_state(pending: list[dict], key: str, typed: str) -> tuple[dict | None, str]:
  """Pure key-transition core. `pending[0]` (if any) is the record currently on
  screen. Returns (verdict_record_or_None, new_typed).

  y -> correct, n -> false_positive, digit chars accumulate into `typed`,
  ENTER('\\r') with non-empty typed -> wrong_value(true_value=int(typed)),
  ENTER with empty typed -> (None, "") no-op, s -> skip sentinel
  ({"verdict": "__skip__"}), q -> quit sentinel ({"verdict": "__quit__"}, no
  crop_img), any other key -> (None, typed) unchanged. Every emitted verdict
  (except quit) carries the current record's crop_img; typed resets to ""
  whenever a verdict/skip is emitted.
  """
  if key == "q":
    return dict(QUIT), ""

  current = pending[0] if pending else None

  if key == "y" and current is not None:
    return {"crop_img": current["crop_img"], "verdict": "correct"}, ""

  if key == "n" and current is not None:
    return {"crop_img": current["crop_img"], "verdict": "false_positive"}, ""

  if key == "s" and current is not None:
    return {"crop_img": current["crop_img"], "verdict": "__skip__"}, ""

  if key == "\r":
    if typed and current is not None:
      return {
        "crop_img": current["crop_img"],
        "verdict": "wrong_value",
        "true_value": int(typed),
      }, ""
    return None, ""

  if len(key) == 1 and key.isdigit():
    return None, typed + key

  return None, typed


def _load_jsonl(path: Path) -> list[dict]:
  if not path.exists():
    return []
  lines = path.read_text().strip().splitlines()
  return [json.loads(line) for line in lines if line.strip()]


def _append_jsonl(path: Path, record: dict) -> None:
  with open(path, "a") as f:
    f.write(json.dumps(record) + "\n")


def _pending_records(publishes: list[dict], reviews: list[dict]) -> list[dict]:
  """Publish records whose crop_img has not already been reviewed (resumable)."""
  reviewed = {r["crop_img"] for r in reviews}
  return [p for p in publishes if p["crop_img"] not in reviewed]


def _key_to_char(code: int) -> str:
  """Maps a cv2.waitKey() return code to the single char next_state expects."""
  if code in (13, 10):
    return "\r"
  masked = code & 0xFF
  if masked in (13, 10):
    return "\r"
  if 0 <= masked < 256:
    return chr(masked)
  return ""


def _show_and_get_key(run_dir: Path, record: dict, wait_ms: int = 0) -> str:
  """Shows the ctx image with the crop pasted in as a top-left inset via
  cv2.imshow, blocks for a keypress, and returns the char per _key_to_char.
  cv2 is imported here only, so the rest of this module stays cv2-free."""
  import cv2

  ctx = cv2.imread(str(run_dir / record["ctx_img"]))
  crop = cv2.imread(str(run_dir / record["crop_img"]))
  display = ctx if ctx is not None else crop
  if ctx is not None and crop is not None and crop.size:
    inset_w = max(1, min(200, ctx.shape[1] // 3))
    scale = inset_w / crop.shape[1]
    inset_h = max(1, int(round(crop.shape[0] * scale)))
    inset = cv2.resize(crop, (inset_w, inset_h))
    ctx[0:inset_h, 0:inset_w] = inset
  cv2.imshow("sign_vision review", display)
  code = cv2.waitKey(wait_ms)
  return _key_to_char(code)


def run_review(run_dir: Path) -> dict:
  """Interactive review loop over run_dir/publishes.jsonl. Writes verdicts to
  run_dir/review.jsonl incrementally, resuming past any crop_img already
  reviewed. Returns {"n_reviewed", "n_remaining"}."""
  run_dir = Path(run_dir)
  publishes = _load_jsonl(run_dir / "publishes.jsonl")
  review_path = run_dir / "review.jsonl"
  reviews = _load_jsonl(review_path)
  pending = _pending_records(publishes, reviews)

  typed = ""
  n_reviewed = 0
  opened_window = False
  try:
    while pending:
      opened_window = True
      key = _show_and_get_key(run_dir, pending[0])
      verdict, typed = next_state(pending, key, typed)
      if verdict is None:
        continue
      if verdict["verdict"] == "__quit__":
        break
      if verdict["verdict"] == "__skip__":
        pending.pop(0)
        continue
      record = {
        "crop_img": verdict["crop_img"],
        "verdict": verdict["verdict"],
        "true_value": verdict.get("true_value"),
      }
      _append_jsonl(review_path, record)
      n_reviewed += 1
      pending.pop(0)
  finally:
    if opened_window:
      import cv2
      cv2.destroyAllWindows()

  summary = {"n_reviewed": n_reviewed, "n_remaining": len(pending)}
  print(f"reviewed: {n_reviewed}  remaining: {len(pending)}")
  return summary


def main(argv: list[str] | None = None) -> None:
  parser = argparse.ArgumentParser(description="Single-key review tool for sign_vision publishes")
  parser.add_argument("--run", required=True, help="run dir containing publishes.jsonl")
  args = parser.parse_args(argv)
  run_review(Path(args.run).expanduser())


if __name__ == "__main__":
  main()
