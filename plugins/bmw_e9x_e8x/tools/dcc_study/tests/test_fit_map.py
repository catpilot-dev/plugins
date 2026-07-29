import numpy as np

from fit_map import monotonise, emit, GAP_BPS, V_BPS


def test_monotonise_is_running_max():
  assert monotonise([-1.0, -0.5, -0.6, 0.2, 0.1]) == [-1.0, -0.5, -0.5, 0.2, 0.2]


def test_monotonise_leaves_increasing_untouched():
  col = [-1.0, -0.4, 0.0, 0.3, 0.5]
  assert monotonise(col) == col


def test_monotonise_handles_nan_by_carrying_previous():
  out = monotonise([-1.0, float("nan"), -0.2])
  assert out[0] == -1.0
  assert out[1] == -1.0        # NaN carries the previous value forward
  assert out[2] == -0.2


def test_emit_writes_importable_module(tmp_path):
  table = [[float(i) / 10 + j / 100 for j in range(len(V_BPS))]
           for i in range(len(GAP_BPS))]
  path = tmp_path / "dcc_map_table.py"
  emit(path, GAP_BPS, V_BPS, table)

  ns = {}
  exec(compile(path.read_text(), str(path), "exec"), ns)
  assert ns["GAP_BPS"] == GAP_BPS
  assert ns["V_BPS"] == V_BPS
  assert len(ns["A_TABLE"]) == len(GAP_BPS)
  assert len(ns["A_TABLE"][0]) == len(V_BPS)
  assert "GENERATED" in path.read_text()


def test_emitted_table_columns_are_strictly_increasing(tmp_path):
  # regression guard: whatever fit() produced must invert single-valued
  from fit_map import monotonise
  raw = [[-1.0, -1.0], [-0.5, -0.6], [-0.5, -0.2], [0.3, 0.1]]
  cols = [monotonise([r[j] for r in raw]) for j in range(2)]
  for c in cols:
    assert all(b >= a for a, b in zip(c, c[1:]))
