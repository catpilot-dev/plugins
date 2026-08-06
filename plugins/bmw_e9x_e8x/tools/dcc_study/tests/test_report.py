import csv
import math
import random

from bursts import CSV_FIELDS


def fake_rows(n=120):
  random.seed(0)
  rows = []
  for i in range(n):
    cmd = random.choice(["plus1", "plus5", "minus1", "minus5"])
    cadence = random.choice(["hold", "single"])
    v = random.uniform(8, 35)
    a = (0.4 if "plus" in cmd else -0.6) * (1.5 if "5" in cmd else 1.0)
    rows.append({"route": "r", "segment": i % 8, "t_start": i * 10.0,
                 "duration_s": 2.0, "cmd": cmd, "cadence": cadence,
                 "n_frames": 40, "v_start_mps": round(v, 2),
                 "setpoint_gap_mps": 1.0, "pitch_rad": 0.001,
                 "a_baseline": 0.0, "peak_delta_a": round(a * 1.1, 3),
                 "steady_delta_a": round(a, 3) if i % 3 else "nan",
                 "rise_time_s": 0.5, "ticks_accepted": 4})
  return rows


def test_report_generates_outputs(tmp_path):
  import report

  csv_path = tmp_path / "bursts.csv"
  with open(csv_path, "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
    w.writeheader()
    w.writerows(fake_rows())

  out = tmp_path / "report"
  report.generate(csv_path, out)

  assert (out / "response_vs_speed.png").exists()
  assert (out / "residual_vs_setpoint_gap.png").exists()
  assert (out / "residual_vs_pitch.png").exists()
  text = (out / "summary.txt").read_text()
  assert "coverage" in text.lower()
  assert "plus5" in text


def test_load_bursts_types(tmp_path):
  import report

  csv_path = tmp_path / "bursts.csv"
  with open(csv_path, "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
    w.writeheader()
    w.writerows(fake_rows(5))
  rows = report.load_bursts(csv_path)
  assert isinstance(rows[0]["v_start_mps"], float)
  assert isinstance(rows[0]["n_frames"], int)
  nan_rows = [r for r in rows if isinstance(r["steady_delta_a"], float)
              and math.isnan(r["steady_delta_a"])]
  assert nan_rows  # "nan" strings parsed to float nan


def test_pitch_bias_recovers_injected_offset_and_is_flat_filters():
  import report

  random.seed(1)
  offset = -0.113
  rows = [{"pitch_rad": offset + random.uniform(-0.003, 0.003)} for _ in range(50)]

  bias = report.pitch_bias(rows)
  assert math.isclose(bias, offset, abs_tol=0.01)

  near_row = {"pitch_rad": offset + 0.005}   # within PITCH_FLAT of the offset
  far_row = {"pitch_rad": offset + 0.05}     # well outside PITCH_FLAT
  assert report.is_flat(near_row, bias)
  assert not report.is_flat(far_row, bias)


def test_bin_medians_drops_nan_delta_a_but_coverage_still_counts_it(tmp_path):
  import report

  rows = fake_rows(30)
  # Segment-edge burst: no pre-burst baseline window on either side, so both
  # delta_a sources are NaN (see bursts.measure()/delta_a()).
  edge = dict(rows[0])
  edge.update({"cmd": "plus1", "cadence": "hold", "v_start_mps": 20.0,
               "pitch_rad": 0.001, "steady_delta_a": "nan", "peak_delta_a": "nan"})
  rows.append(edge)

  csv_path = tmp_path / "bursts.csv"
  with open(csv_path, "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
    w.writeheader()
    w.writerows(rows)

  loaded = report.load_bursts(csv_path)
  bias = report.pitch_bias(loaded)
  medians = report._bin_medians(loaded, bias)
  assert medians
  assert all(not math.isnan(m) for m in medians.values())  # not NaN-poisoned

  out = tmp_path / "report"
  report.generate(csv_path, out)
  text = (out / "summary.txt").read_text()

  # The coverage table counts every burst per bin, regardless of whether its
  # delta_a could be computed, so the edge burst must still show up there.
  bins = sorted({report.speed_bin(r["v_start_mps"]) for r in loaded})
  col = bins.index(report.speed_bin(20.0))
  expected = sum(1 for r in loaded if r["cmd"] == "plus1" and r["cadence"] == "hold"
                 and report.speed_bin(r["v_start_mps"]) == report.speed_bin(20.0))
  line = next(l for l in text.splitlines() if l.split()[:2] == ["plus1", "hold"])
  counts = [int(x) for x in line.split()[2:]]
  assert counts[col] == expected
