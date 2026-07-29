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
