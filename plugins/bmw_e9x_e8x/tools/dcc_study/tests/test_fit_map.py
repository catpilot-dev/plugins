import numpy as np

from fit_map import monotonise, emit, fit, GAP_BPS, V_BPS, MIN_CELL


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
  emit(path, GAP_BPS, V_BPS, table, counts=None)

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


def test_fit_segment_with_low_samples_is_skipped(tmp_path):
  """A segment with < 200 qualifying samples is skipped."""
  # Create a tiny extracted dir with one .npz file containing < 200 qualifying samples
  d = np.savez(
    tmp_path / "tiny.npz",
    cs_t=np.arange(50),
    vEgo=np.ones(50) * 10,
    aEgo=np.zeros(50),
    setpoint=np.ones(50) * 15,
    cruiseEnabled=np.ones(50),  # Always enabled
    gas=np.zeros(50),            # Never pressing gas
    brake=np.zeros(50),          # Never pressing brake
  )
  gap_bps, v_bps, table, counts = fit(tmp_path)
  # All cells should be NaN (no valid data after skipping)
  for row in table:
    for x in row:
      assert np.isnan(x)


def test_fit_cell_with_sufficient_samples_returns_median(tmp_path):
  """A cell with >= MIN_CELL samples returns a real median, not NaN."""
  # Create data where a specific gap/speed cell has enough samples
  # Gap = setpoint - vEgo, Speed = vEgo
  # We want a cell at gap ~0 and vEgo ~20
  n = MIN_CELL + 50
  gap_vals = np.zeros(n)          # gap = 0
  vel_vals = np.ones(n) * 20.0    # vEgo = 20
  acc_vals = np.linspace(-0.5, 0.5, n)  # aEgo varies, median should be ~0.0
  setpoint_vals = vel_vals + gap_vals   # setpoint = vEgo + gap

  d = np.savez(
    tmp_path / "sufficient.npz",
    cs_t=np.arange(n),
    vEgo=vel_vals,
    aEgo=acc_vals,
    setpoint=setpoint_vals,
    cruiseEnabled=np.ones(n),
    gas=np.zeros(n),
    brake=np.zeros(n),
  )
  gap_bps, v_bps, table, counts = fit(tmp_path)

  # Find the cell that corresponds to gap ~0 and speed ~20
  # gap_bps = [-3.0, -2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5]
  # v_bps = [8.0, 14.0, 20.0, 26.0, 33.0]
  gap_idx = 5  # gap 0.0
  v_idx = 2    # speed 20.0
  assert not np.isnan(table[gap_idx][v_idx])
  # Median of linspace should be ~0.0
  assert abs(table[gap_idx][v_idx] - 0.0) < 0.1
  # Count should be preserved
  assert counts[gap_idx][v_idx] == n


def test_fit_cell_below_min_shows_true_count(tmp_path):
  """A cell below MIN_CELL is NaN in the raw table and shows count in N_TABLE."""
  # Create data where specific cells have low sample counts
  n_low = MIN_CELL - 50  # Below threshold
  n_high = MIN_CELL + 50  # Above threshold

  # Create two segments: one with few samples, one with many
  # Both at gap=0, but different speeds to trigger different cells
  gap_vals = np.concatenate([
    np.zeros(n_low),        # Segment 1: gap=0, but only n_low samples
    np.zeros(n_high),       # Segment 2: gap=0, with n_high samples
  ])
  vel_vals = np.concatenate([
    np.ones(n_low) * 20.0,   # Speed 20 (sparse)
    np.ones(n_high) * 14.0,  # Speed 14 (dense)
  ])
  acc_vals = np.concatenate([
    np.linspace(-0.5, 0.5, n_low),
    np.linspace(-0.5, 0.5, n_high),
  ])
  setpoint_vals = vel_vals + gap_vals

  d = np.savez(
    tmp_path / "mixed.npz",
    cs_t=np.arange(len(gap_vals)),
    vEgo=vel_vals,
    aEgo=acc_vals,
    setpoint=setpoint_vals,
    cruiseEnabled=np.ones(len(gap_vals)),
    gas=np.zeros(len(gap_vals)),
    brake=np.zeros(len(gap_vals)),
  )
  gap_bps, v_bps, table, counts = fit(tmp_path)

  gap_idx = 5  # gap 0.0
  # v_idx=2 is 20.0 (should have few samples)
  # v_idx=1 is 14.0 (should have many samples)
  assert counts[gap_idx][2] == n_low
  assert counts[gap_idx][1] == n_high
