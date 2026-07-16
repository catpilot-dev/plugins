"""Tests for the single-key review tool core (review.py) and the precision/timing
report (report.py). Both `next_state` and `compute_report` are pure stdlib (no numpy,
no cv2) so these tests must run in the bare repo env -- no importorskip here."""
import json

from plugins.speedlimitd.tools.sign_vision.review import next_state, _pending_records
from plugins.speedlimitd.tools.sign_vision.report import compute_report


REC = {"crop_img": "crops/000010_60.jpg", "value": 60}


# ---------------------------------------------------------------------------
# next_state
# ---------------------------------------------------------------------------

def test_next_state_y_is_correct():
  verdict, typed = next_state([REC], "y", "")
  assert verdict == {"crop_img": "crops/000010_60.jpg", "verdict": "correct"}
  assert typed == ""


def test_next_state_n_is_false_positive():
  verdict, typed = next_state([REC], "n", "")
  assert verdict == {"crop_img": "crops/000010_60.jpg", "verdict": "false_positive"}
  assert typed == ""


def test_next_state_digits_accumulate():
  verdict, typed = next_state([REC], "6", "")
  assert verdict is None
  assert typed == "6"

  verdict, typed = next_state([REC], "0", typed)
  assert verdict is None
  assert typed == "60"


def test_next_state_enter_with_typed_is_wrong_value():
  verdict, typed = next_state([REC], "\r", "60")
  assert verdict == {
    "crop_img": "crops/000010_60.jpg",
    "verdict": "wrong_value",
    "true_value": 60,
  }
  assert typed == ""


def test_next_state_enter_with_empty_typed_is_noop():
  verdict, typed = next_state([REC], "\r", "")
  assert verdict is None
  assert typed == ""


def test_next_state_s_is_skip_sentinel():
  verdict, typed = next_state([REC], "s", "")
  assert verdict == {"crop_img": "crops/000010_60.jpg", "verdict": "__skip__"}
  assert typed == ""


def test_next_state_q_is_quit_sentinel_no_crop_img():
  verdict, typed = next_state([REC], "q", "")
  assert verdict == {"verdict": "__quit__"}
  assert typed == ""


def test_next_state_q_resets_typed_buffer():
  verdict, typed = next_state([REC], "q", "1")
  assert verdict == {"verdict": "__quit__"}
  assert typed == ""


def test_next_state_other_key_unchanged():
  verdict, typed = next_state([REC], "x", "1")
  assert verdict is None
  assert typed == "1"


def test_next_state_typed_resets_after_verdict():
  verdict, typed = next_state([REC], "y", "5")
  assert verdict is not None
  assert typed == ""


# ---------------------------------------------------------------------------
# _pending_records (resume filtering)
# ---------------------------------------------------------------------------

def test_pending_records_drops_already_reviewed():
  publishes = [
    {"crop_img": "crops/a.jpg", "value": 60},
    {"crop_img": "crops/b.jpg", "value": 40},
    {"crop_img": "crops/c.jpg", "value": 80},
  ]
  reviews = [{"crop_img": "crops/b.jpg", "verdict": "correct", "true_value": None}]
  pending = _pending_records(publishes, reviews)
  assert [p["crop_img"] for p in pending] == ["crops/a.jpg", "crops/c.jpg"]


def test_pending_records_no_reviews_keeps_all():
  publishes = [{"crop_img": "crops/a.jpg", "value": 60}]
  assert _pending_records(publishes, []) == publishes


# ---------------------------------------------------------------------------
# compute_report -- 6 hand-built publish records
# ---------------------------------------------------------------------------

PUBLISHES = [
  {"crop_img": "crops/p1.jpg", "value": 60},
  {"crop_img": "crops/p2.jpg", "value": 60},
  {"crop_img": "crops/p3.jpg", "value": 40},
  {"crop_img": "crops/p4.jpg", "value": 80},
  {"crop_img": "crops/p5.jpg", "value": 60},
  {"crop_img": "crops/p6.jpg", "value": 40},
]

REVIEWS = [
  {"crop_img": "crops/p1.jpg", "verdict": "correct", "true_value": None},
  {"crop_img": "crops/p2.jpg", "verdict": "wrong_value", "true_value": 50},
  {"crop_img": "crops/p3.jpg", "verdict": "correct", "true_value": None},
  {"crop_img": "crops/p4.jpg", "verdict": "false_positive", "true_value": None},
  {"crop_img": "crops/p5.jpg", "verdict": "__skip__"},
  # p6 never reviewed
]

TIMINGS = [
  {"frame_idx": 0, "det_ms": 10.0, "cls_ms": 5.0, "n_crops": 1},
  {"frame_idx": 1, "det_ms": 20.0, "cls_ms": 10.0, "n_crops": 1},
  {"frame_idx": 2, "det_ms": 30.0, "cls_ms": 15.0, "n_crops": 1},
  {"frame_idx": 3, "det_ms": 40.0, "cls_ms": 20.0, "n_crops": 1},
]


def test_compute_report_counts_and_precision():
  report = compute_report(PUBLISHES, REVIEWS, TIMINGS)
  assert report["n_publishes"] == 6
  # p5 (__skip__) excluded, p6 (unreviewed) excluded -> 4 reviewed
  assert report["n_reviewed"] == 4
  # correct: p1, p3 -> 2 of 4
  assert report["precision"] == 0.5


def test_compute_report_per_value():
  report = compute_report(PUBLISHES, REVIEWS, TIMINGS)
  assert report["per_value"] == {
    60: {"correct": 1, "wrong_value": 1, "false_positive": 0},
    40: {"correct": 1, "wrong_value": 0, "false_positive": 0},
    80: {"correct": 0, "wrong_value": 0, "false_positive": 1},
  }


def test_compute_report_timing_means_and_p95():
  report = compute_report(PUBLISHES, REVIEWS, TIMINGS)
  assert report["mean_det_ms"] == 25.0
  assert report["mean_cls_ms"] == 12.5
  # sorted [10,20,30,40], n=4, idx=0.95*3=2.85 -> 30 + (40-30)*0.85 = 38.5
  assert report["p95_det_ms"] == 38.5


def test_compute_report_exact_keys():
  report = compute_report(PUBLISHES, REVIEWS, TIMINGS)
  assert set(report) == {
    "n_publishes", "n_reviewed", "precision", "per_value",
    "mean_det_ms", "mean_cls_ms", "p95_det_ms",
  }


def test_compute_report_zero_reviewed_precision_none():
  report = compute_report(PUBLISHES, [], TIMINGS)
  assert report["n_reviewed"] == 0
  assert report["precision"] is None


def test_compute_report_no_timings_means_none():
  report = compute_report(PUBLISHES, REVIEWS, [])
  assert report["mean_det_ms"] is None
  assert report["mean_cls_ms"] is None
  assert report["p95_det_ms"] is None


def test_compute_report_all_skipped_precision_none():
  reviews = [{"crop_img": "crops/p1.jpg", "verdict": "__skip__"}]
  report = compute_report(PUBLISHES, reviews, TIMINGS)
  assert report["n_reviewed"] == 0
  assert report["precision"] is None
  assert report["per_value"] == {}


# ---------------------------------------------------------------------------
# report.json round-trips through json.dumps (report.py has no cv2/numpy dep)
# ---------------------------------------------------------------------------

def test_compute_report_json_serializable():
  report = compute_report(PUBLISHES, REVIEWS, TIMINGS)
  reparsed = json.loads(json.dumps(report))
  assert reparsed["n_publishes"] == 6
