# Sign Vision PoC Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Offline 2-stage (YOLO11n detect → YOLO11n-cls read) Chinese speed-limit-sign pipeline, trained on TT100K, evaluated by replaying recorded C3 routes with a human review loop.

**Architecture:** A pure, dependency-injected pipeline core (`pipeline.py`) does expansion/voting/multi-frame confirmation and is fully unit-tested with stub models. Around it: dataset prep (TT100K → YOLO formats), a thin ultralytics train/export wrapper, an offline route-replay harness (PyAV + ultralytics-ONNX runners), and a single-key review tool + report.

**Tech Stack:** Python 3.11+, ultralytics (YOLO11n), onnxruntime, PyAV, OpenCV, numpy. Isolated env — NOT the plugins repo env.

## Global Constraints

- All code under `plugins/speedlimitd/tools/sign_vision/` in the plugins repo (branch `dev`).
- Heavy deps isolated: `tools/sign_vision/pyproject.toml`; repo-suite tests MUST auto-skip when deps are absent (`pytest.importorskip`) — the pre-push hook runs the full suite without them.
- Datasets/models/routes live under `~/catpilot-dev/datasets/` — never committed.
- Sign scope: static red-ring regulatory limits only. `il*` (min-speed), `pr*` (end-of-limit), other `p*` circulars are negatives/reject — a wrong value must never publish.
- Commit style: existing repo style, no Co-Authored-By lines.

## Shared Data Schemas (all tasks)

`publishes.jsonl` (one per confirmed publish):
```json
{"t": 12.35, "frame_idx": 247, "value": 60, "conf": 0.81,
 "box": [1421.0, 322.0, 1462.0, 363.0],
 "crop_img": "crops/000247_60.jpg", "ctx_img": "ctx/000247_60.jpg",
 "route": "000003a1--5f9bdd218a", "segment": 18}
```
`review.jsonl`: `{"crop_img": "crops/000247_60.jpg", "verdict": "correct"|"wrong_value"|"false_positive", "true_value": 40}` (`true_value` null unless wrong_value).
`timing.jsonl`: `{"frame_idx": 247, "det_ms": 41.2, "cls_ms": 12.3, "n_crops": 3}`.

---

### Task 1: Package scaffold + pipeline core (the heart)

**Files:**
- Create: `plugins/speedlimitd/tools/sign_vision/pyproject.toml`
- Create: `plugins/speedlimitd/tools/sign_vision/README.md`
- Create: `plugins/speedlimitd/tools/sign_vision/__init__.py` (empty)
- Create: `plugins/speedlimitd/tools/sign_vision/pipeline.py`
- Test: `plugins/speedlimitd/tools/sign_vision/tests/test_pipeline.py`

**Interfaces (Produces — later tasks import these exactly):**
```python
# pipeline.py
Box = tuple[float, float, float, float]          # x1,y1,x2,y2 full-frame px
DetectFn   = Callable[[np.ndarray], list[tuple[Box, float]]]   # BGR frame -> [(box, conf)]
ClassifyFn = Callable[[np.ndarray], tuple[str, float]]         # BGR crop -> (label, conf); label "30".."120" or "reject"

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
               config: PipelineConfig = PipelineConfig()): ...
  def process_frame(self, frame_bgr: np.ndarray, t: float, frame_idx: int) -> list[dict]:
    """Returns publish dicts: {"t","frame_idx","value","conf","box"} (imgs/route added by harness)."""
```
Semantics: per frame — detector boxes filtered by `det_conf` + geometry (min size,
aspect, inside frame); for each surviving box crop the `expansions` (clamped to
frame), classify each; a box votes value v only if a strict majority of crops
agree on the same non-reject label with mean conf ≥ `cls_conf`. Votes feed a
per-value deque of timestamps; when ≥ `confirm_votes` land within
`confirm_window_s` → publish once, then suppress that value until
`t + publish_cooldown_s`.

- [ ] **Step 1: scaffold** — `pyproject.toml` (name `sign-vision`, requires-python `>=3.11`, deps: `ultralytics>=8.3`, `onnxruntime`, `opencv-python`, `av`, `numpy`; `[tool.pytest.ini_options] testpaths=["tests"]`), README (one-paragraph purpose + commands), empty `__init__.py`, `tests/__init__.py`.
- [ ] **Step 2: failing tests** — write `tests/test_pipeline.py` with stub detect/classify fns (plain closures over scripted returns). Cover, at minimum:
```python
import numpy as np
import pytest
np.random.seed(0)
cv2 = pytest.importorskip("cv2")  # heavy-dep guard pattern; pipeline itself needs only numpy
from plugins.speedlimitd.tools.sign_vision.pipeline import SignPipeline, PipelineConfig

FRAME = np.zeros((1208, 1928, 3), dtype=np.uint8)
BOX = (1400.0, 300.0, 1460.0, 360.0)  # 60x60 px, aspect 1.0

def mk(detect_seq, label="60", conf=0.9, cfg=None):
    it = iter(detect_seq)
    detect = lambda f: next(it)
    classify = lambda c: (label, conf)
    return SignPipeline(detect, classify, cfg or PipelineConfig())

def test_three_votes_publishes_once():
    p = mk([[(BOX, 0.8)]] * 4)
    pubs = [p.process_frame(FRAME, t, i) for i, t in enumerate([0.0, 0.5, 1.0, 1.5])]
    flat = [x for fr in pubs for x in fr]
    assert len(flat) == 1 and flat[0]["value"] == 60

def test_reject_never_publishes(): ...      # classify returns ("reject", 0.99) -> zero publishes
def test_majority_of_expansions_required(): ...  # 1-of-3 crops says "60", others reject -> no vote
def test_cooldown_suppresses_republish(): ...    # votes continue after publish -> no 2nd publish inside cooldown_s
def test_votes_outside_window_dont_confirm(): ...# 3 votes at t=0, 3, 6 with window 2.5 -> no publish
def test_tiny_box_filtered(): ...                # 8x8 box -> no classify calls (assert via counting stub)
def test_bad_aspect_filtered(): ...              # 60x20 box -> filtered
```
(Write all seven as real tests, not comments.)
- [ ] **Step 3: run, verify FAIL** (`cd plugins/speedlimitd/tools/sign_vision && uv run pytest tests/ -q`) — ImportError/AttributeError expected.
- [ ] **Step 4: implement `pipeline.py`** exactly to the interface above. Crop expansion: center-preserving scale, clamp to frame bounds, skip if clamped area < 50% of requested. Majority: `Counter` over non-reject labels; strict majority of total crops.
- [ ] **Step 5: run, verify PASS.**
- [ ] **Step 6: also run the REPO suite from repo root** (`PYTHONPATH=. uv run pytest -q` in plugins repo) — must stay green (skips fine).
- [ ] **Step 7: commit** — `sign_vision: pipeline core (2-stage detect/classify with multi-frame confirmation)`.

---

### Task 2: TT100K dataset prep

**Files:**
- Create: `plugins/speedlimitd/tools/sign_vision/dataset_prep.py`
- Test: `plugins/speedlimitd/tools/sign_vision/tests/test_dataset_prep.py`

**Interfaces (Produces):**
```python
# dataset_prep.py — CLI:
#   uv run python dataset_prep.py --tt100k ~/catpilot-dev/datasets/tt100k_2021 \
#       --out ~/catpilot-dev/datasets/sign_vision --min-crops 150 --val-frac 0.1
# Pure functions (unit-tested):
def load_tt100k_annotations(root: Path) -> dict            # parses annotations_all.json (2021) or annotations.json
def partition_categories(counts: dict[str, int], min_crops: int) -> tuple[set[str], set[str]]
    # -> (value_classes e.g. {"pl30",...}, reject_source_classes e.g. {"il60","pr40","pm55",...})
def yolo_det_label(img_w, img_h, boxes: list[Box]) -> str   # "0 cx cy w h" lines, normalized
```
Output layout under `--out`:
```
det/images/{train,val}/*.jpg   det/labels/{train,val}/*.txt   det.yaml (single class: sign)
cls/{train,val}/{30,40,...,reject}/*.jpg                      classes.json
```
Rules: det positives = `pl*` boxes only (other categories = implicit background). Images containing ONLY non-pl signs are still included (hard-negative backgrounds, no label file content). Cls crops: GT `pl*` boxes with 1.2× margin → `<value>/`; `il*`,`pr*`,`pm*`,`ph*`,`pw*`,`pn*` crops AND 2 random 64–160 px background crops per train image → `reject/`. `pl*` under `--min-crops` → `reject/`. `classes.json` = `{"values": [30,...], "counts": {...}, "min_crops": 150}`. Deterministic split by image-id hash (`--val-frac`).

- [ ] **Step 1: failing tests** — build a tiny synthetic TT100K fixture in `tmp_path` (annotations JSON with 4 images: pl60+pl60, pl40, il80 only, no-signs; 64×64 generated JPGs). Test: annotation parsing; partition with `min_crops=2` (pl60 kept, pl40→reject); label-line math against hand-computed normals; end-to-end `main()` produces the exact layout incl. `det.yaml` and `classes.json`.
- [ ] **Step 2: run, verify FAIL.**
- [ ] **Step 3: implement.** No global state; `main(argv)` entry.
- [ ] **Step 4: run tests + repo suite, verify PASS/green.**
- [ ] **Step 5: commit** — `sign_vision: TT100K -> YOLO det/cls dataset prep`.

---

### Task 3: Train + export wrapper

**Files:**
- Create: `plugins/speedlimitd/tools/sign_vision/train.py`
- Test: `plugins/speedlimitd/tools/sign_vision/tests/test_train.py`

**Interfaces (Produces):**
```python
# CLI: uv run python train.py det --data ~/catpilot-dev/datasets/sign_vision/det.yaml --epochs 60
#      uv run python train.py cls --data ~/catpilot-dev/datasets/sign_vision/cls --epochs 40
#      uv run python train.py export --weights <best.pt> --kind det|cls
def build_train_kwargs(kind: str, data: str, epochs: int, device: str) -> dict
    # det -> {"model":"yolo11n.pt","imgsz":256,...}; cls -> {"model":"yolo11n-cls.pt","imgsz":128,...}
```
Export writes ONNX next to weights AND copies to `~/catpilot-dev/datasets/sign_vision/models/{det,cls}.onnx`. `--device` defaults `0`, accepts `cpu`.

- [ ] **Step 1: failing test** — `build_train_kwargs` returns exact imgsz/model per kind; unknown kind raises `ValueError`. (No actual training in tests — ultralytics import stays inside `main()`.)
- [ ] **Step 2: FAIL → implement → PASS → repo suite green.**
- [ ] **Step 3: commit** — `sign_vision: train/export wrapper (yolo11n det@256, cls@128 -> onnx)`.

---

### Task 4: Offline route-replay harness

**Files:**
- Create: `plugins/speedlimitd/tools/sign_vision/harness.py`
- Test: `plugins/speedlimitd/tools/sign_vision/tests/test_harness.py`

**Interfaces:**
- Consumes: `SignPipeline`, `PipelineConfig`, `DetectFn`, `ClassifyFn` from `pipeline.py` (Task 1 signatures verbatim).
- Produces:
```python
ROI_PRESETS = {"full": (0.0, 0.0, 1.0, 1.0), "right": (0.45, 0.05, 1.0, 0.80)}
def iter_route_frames(route_dir: Path, sample_hz: float, fps: float = 20.0)
    # yields (frame_bgr, t_seconds, frame_idx, segment:int); segments = numeric-suffix dirs
    # sorted by int suffix, each containing fcamera.hevc; PyAV decode, take every
    # round(fps/sample_hz)-th frame; t is cumulative across segments (idx/fps).
def make_onnx_runners(det_onnx: Path, cls_onnx: Path, roi: tuple, det_conf: float)
    # -> (DetectFn, ClassifyFn). Uses ultralytics.YOLO(onnx) predictors (correct pre/post
    # for free). DetectFn crops ROI (normalized rect -> px), runs det at imgsz=256, maps
    # boxes back to FULL-frame coords. ClassifyFn runs cls at imgsz=128 -> (top1_label, top1_conf).
# CLI: uv run python harness.py --route-dir ~/catpilot-dev/datasets/routes/<route> \
#        --det-onnx models/det.onnx --cls-onnx models/cls.onnx \
#        --roi right --sample-hz 2 --out ~/catpilot-dev/datasets/sign_vision/runs/<name>
```
Harness loop: for each sampled frame run pipeline; on publish, write crop (tight box +20% margin) to `crops/{frame_idx:06d}_{value}.jpg` and full frame with box drawn to `ctx/...jpg`; append to `publishes.jsonl` (schema in Global) and per-frame `timing.jsonl` (wall-clock ms via `time.perf_counter()`). Print end summary (frames, publishes, mean det/cls ms).

- [ ] **Step 1: failing tests** — (a) `iter_route_frames`: fixture builds 2 fake segments (`--0`,`--1`) each with a 20-frame 320×240 hevc written via PyAV in the test; assert frame count at `sample_hz=4`, cumulative timestamps, segment ids. (b) end-to-end `run()` with injected stub DetectFn/ClassifyFn (bypass `make_onnx_runners`): steady fake sign → exactly one publish; assert jsonl schema keys, crop/ctx files exist, timing lines present. `pytest.importorskip("av")`.
- [ ] **Step 2: FAIL → implement → PASS → repo suite green.**
- [ ] **Step 3: commit** — `sign_vision: offline route-replay harness (hevc -> pipeline -> publishes/review imgs/timing)`.

---

### Task 5: Review tool + report

**Files:**
- Create: `plugins/speedlimitd/tools/sign_vision/review.py`
- Create: `plugins/speedlimitd/tools/sign_vision/report.py`
- Test: `plugins/speedlimitd/tools/sign_vision/tests/test_review_report.py`

**Interfaces:**
- Consumes: `publishes.jsonl` / `review.jsonl` / `timing.jsonl` schemas (Global section, verbatim).
- Produces:
```python
# review.py — pure core, unit-tested WITHOUT a window:
def next_state(pending: list[dict], key: str, typed: str) -> tuple[dict | None, str]
    # keys: 'y'->correct, 'n'->false_positive, digits accumulate typed value,
    # ENTER('\r') with typed digits -> wrong_value(true_value=int(typed)),
    # 's' skip, 'q' quit sentinel ({"verdict":"__quit__"}, "")
# CLI shows ctx image + crop inset via cv2.imshow, appends verdicts to review.jsonl, resumable
# (skips crop_imgs already in review.jsonl).
# report.py:
def compute_report(publishes, reviews, timings) -> dict
    # {"n_publishes","n_reviewed","precision","per_value":{60:{"correct":..,"wrong_value":..,"false_positive":..}},
    #  "mean_det_ms","mean_cls_ms","p95_det_ms"}
# CLI: uv run python report.py --run <runs/name>  -> prints table + writes report.json
```

- [ ] **Step 1: failing tests** — `next_state` transitions (y/n/digits+enter/skip/quit); `compute_report` on 6 hand-built records → exact precision fraction, per-value counts, timing means. No cv2 window in tests.
- [ ] **Step 2: FAIL → implement → PASS → repo suite green.**
- [ ] **Step 3: commit** — `sign_vision: single-key review tool + precision/timing report`.

---

### Operations (supervisor, not subagents)

- [ ] O1: Download TT100K 2021 to `~/catpilot-dev/datasets/tt100k_2021/` (resumable, background; find current link on cg.cs.tsinghua.edu.cn/traffic-sign).
- [ ] O2: Fix NVIDIA driver/library mismatch (likely reboot) — required before `train.py` on GPU; CPU smoke run OK meanwhile.
- [ ] O3: When C3 reachable — verify `fcamera.hevc` exists in realdata; copy 3-5 routes (incl. 000003a1 segs with known signs) to `~/catpilot-dev/datasets/routes/`.
- [ ] O4: Run prep → train det → train cls → export.
- [ ] O5: Harness over copied routes (roi=right and full), user reviews, generate report; compare against phase gate (≥90% precision, 70-80% known-sign recall, C3-plausible timing).
