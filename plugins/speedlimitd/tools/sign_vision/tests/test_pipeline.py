"""Tests for the sign_vision 2-stage detect/classify pipeline core."""
import pytest

np = pytest.importorskip("numpy")
np.random.seed(0)

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


def test_reject_never_publishes():
  # classify returns ("reject", 0.99) -> zero publishes, no matter how many frames
  p = mk([[(BOX, 0.8)]] * 6, label="reject", conf=0.99)
  pubs = [p.process_frame(FRAME, t, i) for i, t in enumerate([0.0, 0.5, 1.0, 1.5, 2.0, 2.5])]
  flat = [x for fr in pubs for x in fr]
  assert flat == []


def test_majority_of_expansions_required():
  # 1-of-3 crop expansions says "60", the other two say reject -> no vote for that box
  it_count = {"n": 0}

  def classify(crop):
    it_count["n"] += 1
    if it_count["n"] % 3 == 1:
      return ("60", 0.9)
    return ("reject", 0.9)

  detect = lambda f: [(BOX, 0.8)]
  cfg = PipelineConfig()
  p = SignPipeline(detect, classify, cfg)
  pubs = [p.process_frame(FRAME, t, i) for i, t in enumerate([0.0, 0.5, 1.0, 1.5, 2.0])]
  flat = [x for fr in pubs for x in fr]
  assert flat == []


def test_cooldown_suppresses_republish():
  # votes continue after the first publish -> no 2nd publish inside cooldown_s
  p = mk([[(BOX, 0.8)]] * 10)
  ts = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5]
  pubs = [p.process_frame(FRAME, t, i) for i, t in enumerate(ts)]
  flat = [x for fr in pubs for x in fr]
  assert len(flat) == 1
  assert flat[0]["value"] == 60


def test_votes_outside_window_dont_confirm():
  # 3 votes at t=0, 3, 6 with window 2.5 -> no publish (votes age out of the deque)
  p = mk([[(BOX, 0.8)]] * 3)
  pubs = [p.process_frame(FRAME, t, i) for i, t in enumerate([0.0, 3.0, 6.0])]
  flat = [x for fr in pubs for x in fr]
  assert flat == []


def test_tiny_box_filtered():
  calls = {"n": 0}

  def classify(crop):
    calls["n"] += 1
    return ("60", 0.9)

  tiny_box = (1400.0, 300.0, 1408.0, 308.0)  # 8x8 px < min_box_px=14
  detect = lambda f: [(tiny_box, 0.8)]
  p = SignPipeline(detect, classify, PipelineConfig())
  p.process_frame(FRAME, 0.0, 0)
  assert calls["n"] == 0


def test_bad_aspect_filtered():
  calls = {"n": 0}

  def classify(crop):
    calls["n"] += 1
    return ("60", 0.9)

  bad_box = (1400.0, 300.0, 1460.0, 320.0)  # 60x20 -> aspect 3.0, out of (0.65, 1.5)
  detect = lambda f: [(bad_box, 0.8)]
  p = SignPipeline(detect, classify, PipelineConfig())
  p.process_frame(FRAME, 0.0, 0)
  assert calls["n"] == 0
