# Sign Vision PoC: 2-Stage Chinese Speed-Limit Sign Reading (Offline Feasibility)

**Date:** 2026-07-16
**Component:** `plugins/speedlimitd/tools/sign_vision/` (new, offline tooling only)
**Status:** Design approved (proof-of-concept)

## Goal

Prove that a 2-stage pipeline (YOLO11n detector + YOLO11n classifier, per
StarPilot's speed-limit-vision writeup) trained on public Chinese datasets can
read **static regulatory speed-limit signs** from recorded C3 camera footage,
and produce the numbers needed to green-light an on-device phase:
precision, recall on known routes, and per-stage compute cost.

speedlimitd already has the integration point — the `source==1` YOLO fusion
slot (conf 0.8, 120 s timeout) is a placeholder nothing feeds. This PoC does
NOT touch speedlimitd or the device; it is offline tooling only.

## Scope

- **In:** static red-ring circular regulatory limit signs (值 from TT100K `pl*`
  class inventory, expected ≈ 30/40/50/60/70/80/100/120 km/h).
- **Negatives (never publish):** blue minimum-speed circles, slash/end-of-limit
  signs, other circular signs, non-signs — trained as detector hard negatives
  AND a learned classifier **reject** class (defense in depth).
- **Out (later phases):** electronic/LED gantry signs, end-of-limit as an
  active "clear" signal, on-device runtime, speedlimitd ingestion.

## Architecture

1. **Detector:** YOLO11n, 256×256 input, single class
   ("speed-limit-like circular sign").
2. **Classifier:** YOLO11n-cls, 128×128 input, value classes + reject.
3. **Runtime logic (offline harness):** per frame — configurable ROI →
   detector → several crop expansions per box → classifier on each →
   combined support + geometry/size checks → multi-frame confirmation
   before a detection counts as a "publish".

Both models export to ONNX; the harness runs onnxruntime (CPU is fine
offline). Models stay separate (StarPilot: merging saves nothing).

## Data

- **TT100K** (primary; per-value `pl*` labels) + **CCTSDB** (variety/negatives).
- Datasets and trained artifacts live under `~/catpilot-dev/datasets/`
  (NOT in git; models not committed in this phase).
- Classifier value classes chosen by TT100K sample count; thin classes fold
  into reject for the PoC.

## Evaluation (phase gate)

- Harness replays recorded routes from C3 `realdata` (`fcamera.hevc`,
  1928×1208@20 — availability on the user's device **must be verified**;
  qcamera 526×330 is too soft for distant signs).
- Every publish saves crop + context image → **single-key review tool**
  (OpenCV window: y/n/value-correction) → exact precision.
- Recall estimated on commute routes with known posted signs.
- Timing per stage recorded → extrapolate C3 (SD845) feasibility at ~1–2 Hz.
- **Success ≈** ≥90 % precision on reviewed publishes; ≥70–80 % of known
  signs caught; compute plausibly fits C3. Miss badly → phase 1b =
  fine-tune on own footage (bookmark-and-review loop) before device work.

## Constraints

- Heavy deps (ultralytics, onnxruntime, av, opencv) are **isolated** in
  `tools/sign_vision/` (own pyproject/requirements). Any tests added to the
  plugins suite must auto-skip when those deps are absent — the pre-push hook
  runs the full suite on machines without them.
- Training on the local NVIDIA GPU; **prerequisite:** fix the current
  driver/library version mismatch (likely a reboot). Fallback: cloud notebook.

## Roadmap after PoC (each its own spec)

fine-tune on own footage → on-device `yolod` plugin (VisionIPC + onnxruntime,
publish `yoloSpeedLimit` on plugin_bus) → speedlimitd ingestion (replace
hardcoded 0.8 confidence) → LED gantry signs.
