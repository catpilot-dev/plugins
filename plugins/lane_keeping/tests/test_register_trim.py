import importlib.util, os, sys
from types import SimpleNamespace
import pytest

# Load register/anchor by explicit path under unique module names — do NOT
# insert the plugin dir on sys.path (two plugins both ship a top-level
# register.py; a bare import would shadow the other under the full suite).
_PLUGIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _load(name, unique):
  spec = importlib.util.spec_from_file_location(unique, os.path.join(_PLUGIN_DIR, name + '.py'))
  m = importlib.util.module_from_spec(spec)
  sys.modules[unique] = m
  spec.loader.exec_module(m)
  return m


register = _load('register', 'lk_register_trim')
# Warm the sibling-module caches against the REAL plugin dir now, before any
# test monkeypatches _PLUGIN_DIR to a tmp data dir (the loaders build their
# path from _PLUGIN_DIR; param reads legitimately follow the override, module
# loads must not). Mirrors the implicit warming test_register.py relies on.
register._anchor_module()
register._trim_module()


@pytest.fixture(autouse=True)
def _reset_register():
  register._anchor = None
  register._trim = None
  register._pub = None
  register._tick = 0
  register._last_yaw_written = None
  register._calib_bias_cache = {'val': 0.0, 'calls': 0}
  yield


@pytest.fixture
def data_dir(tmp_path, monkeypatch):
  d = tmp_path / 'data'
  d.mkdir()
  monkeypatch.setattr(register, '_PLUGIN_DIR', str(tmp_path))
  return d


def _mv_at(gap):
  # left-ego-line model_v2 at driver-wheel-to-line gap (m); +y = right frame.
  y = -(gap + 0.91)
  xs = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0]
  return SimpleNamespace(
    laneLines=[SimpleNamespace(x=[], y=[0.0]), SimpleNamespace(x=xs, y=[y] * 6),
               SimpleNamespace(x=xs, y=[1.75] * 6), SimpleNamespace(x=[], y=[0.0])],
    laneLineProbs=[0.0, 1.0, 1.0, 0.0],
    position=SimpleNamespace(x=xs, y=[0.0] * 6))


# --------------------------------------------------------------------------
# Config load
# --------------------------------------------------------------------------

def test_load_trim_config_defaults(data_dir):
  cfg = register._load_trim_config()
  assert cfg.mode == 0
  assert cfg.fixed_deg == 0.0
  assert cfg.max_deg == 0.8
  assert cfg.slew_deg_s == 0.02
  assert cfg.yaw_sign == 0
  assert cfg.ki == 0.04
  assert cfg.gap_lo == 0.6 and cfg.gap_hi == 1.0


def test_load_trim_config_overrides(data_dir):
  (data_dir / 'CalibTrimMode').write_text('2')
  (data_dir / 'CalibTrimFixedDeg').write_text('0.3')
  (data_dir / 'CalibTrimMaxDeg').write_text('0.6')
  (data_dir / 'CalibTrimSlewDegS').write_text('0.05')
  (data_dir / 'CalibTrimYawSign').write_text('-1')
  (data_dir / 'CalibTrimKi').write_text('0.08')
  (data_dir / 'CalibTrimGapLo').write_text('0.5')
  (data_dir / 'CalibTrimGapHi').write_text('1.2')
  cfg = register._load_trim_config()
  assert cfg.mode == 2 and isinstance(cfg.mode, int)
  assert cfg.fixed_deg == 0.3
  assert cfg.max_deg == 0.6
  assert cfg.slew_deg_s == 0.05
  assert cfg.yaw_sign == -1 and isinstance(cfg.yaw_sign, int)
  assert cfg.ki == 0.08
  assert cfg.gap_lo == 0.5 and cfg.gap_hi == 1.2


# --------------------------------------------------------------------------
# Writer
# --------------------------------------------------------------------------

def test_write_yaw_file_atomic_no_tmp_left(data_dir):
  register._write_yaw_file(0.123)
  path = data_dir / 'CalibTrimYawDeg'
  assert path.exists()
  assert not (data_dir / 'CalibTrimYawDeg.tmp').exists()
  # no stray .tmp files anywhere in data/
  assert not any(p.suffix == '.tmp' for p in data_dir.iterdir())


def test_write_yaw_file_roundtrip_millidegree(data_dir):
  register._write_yaw_file(0.001)
  path = data_dir / 'CalibTrimYawDeg'
  assert float(path.read_text()) == pytest.approx(0.001, abs=1e-9)


def test_write_yaw_file_skips_when_unchanged(data_dir):
  path = data_dir / 'CalibTrimYawDeg'
  register._write_yaw_file(0.123)
  assert path.exists()
  path.unlink()                         # remove; a skipped write won't recreate it
  register._write_yaw_file(0.1234)      # rounds to same 0.123 -> skipped
  assert not path.exists()
  register._write_yaw_file(0.200)       # different -> written again
  assert path.exists()
  assert float(path.read_text()) == pytest.approx(0.200, abs=1e-9)


# --------------------------------------------------------------------------
# Reader (modeld-side hook)
# --------------------------------------------------------------------------

def test_read_yaw_missing_returns_zero(data_dir):
  assert register._read_yaw_deg() == 0.0
  assert register.on_calib_bias(0.0) == 0.0


def test_read_yaw_garbage_returns_zero(data_dir):
  (data_dir / 'CalibTrimYawDeg').write_text('not-a-number')
  assert register._read_yaw_deg() == 0.0


def test_read_yaw_nonfinite_returns_zero(data_dir):
  (data_dir / 'CalibTrimYawDeg').write_text('inf')
  assert register._read_yaw_deg() == 0.0
  (data_dir / 'CalibTrimYawDeg').write_text('nan')
  assert register._read_yaw_deg() == 0.0


def test_read_yaw_clamps_to_max_deg_default(data_dir):
  (data_dir / 'CalibTrimYawDeg').write_text('5.0')
  assert register._read_yaw_deg() == pytest.approx(0.8)
  (data_dir / 'CalibTrimYawDeg').write_text('-5.0')
  assert register._read_yaw_deg() == pytest.approx(-0.8)


def test_calib_bias_clamps_via_hook(data_dir):
  (data_dir / 'CalibTrimYawDeg').write_text('5.0')
  assert register.on_calib_bias(0.0) == pytest.approx(0.8)


def test_calib_bias_cache_refreshes_every_100_calls(data_dir):
  path = data_dir / 'CalibTrimYawDeg'
  path.write_text('0.100')
  assert register.on_calib_bias(0.0) == pytest.approx(0.100)   # call #1: reads
  path.write_text('0.200')                                     # change on disk
  for _ in range(99):                                          # calls #2..#100: stale
    assert register.on_calib_bias(0.0) == pytest.approx(0.100)
  assert register.on_calib_bias(0.0) == pytest.approx(0.200)   # call #101: refresh


# --------------------------------------------------------------------------
# Anchor telemetry exposes authority
# --------------------------------------------------------------------------

def test_anchor_telem_contains_authority():
  anchor = _load('anchor', 'lk_anchor_trimtest')
  a = anchor.LaneAnchor(anchor.AnchorConfig())
  _out, telem = a.update(0.01, _mv_at(0.84), 25.0, False, lat_delay=0.45)
  assert 'authority' in telem
  assert isinstance(telem['authority'], float)


# --------------------------------------------------------------------------
# Hook wires trim telemetry into the single published message
# --------------------------------------------------------------------------

def test_hook_publishes_trim_keys(data_dir, monkeypatch):
  captured = {}
  monkeypatch.setattr(register, '_publish', lambda telem: captured.update(telem))
  register.on_curvature_correction(0.01, _mv_at(0.84), 25.0, False, lat_delay=0.45)
  for key in ('trim_delta_deg', 'trim_err', 'trim_mode', 'trim_integrating'):
    assert key in captured
  # the anchor telemetry is still present in the same message (single publish)
  assert 'authority' in captured and 'gap_dc' in captured


def test_hook_trim_never_breaks_control_path(data_dir, monkeypatch):
  # a broken trim config load must not stop the hook returning a curvature
  monkeypatch.setattr(register, '_load_trim_config',
                      lambda: (_ for _ in ()).throw(RuntimeError('boom')))
  monkeypatch.setattr(register, '_publish', lambda telem: None)
  out = register.on_curvature_correction(0.0177, _mv_at(0.84), 25.0, False, lat_delay=0.45)
  assert isinstance(out, float)


def test_hook_writes_yaw_file_on_cadence(data_dir, monkeypatch):
  monkeypatch.setattr(register, '_publish', lambda telem: None)
  path = data_dir / 'CalibTrimYawDeg'
  for _ in range(99):
    register.on_curvature_correction(0.0, _mv_at(0.84), 25.0, False, lat_delay=0.45)
  assert not path.exists()          # nothing written before the 100th tick
  register.on_curvature_correction(0.0, _mv_at(0.84), 25.0, False, lat_delay=0.45)
  assert path.exists()              # 100th tick writes


# --------------------------------------------------------------------------
# Mode-2 sentinel guard: gap_dc must be the anchor's own None-preserving
# attribute, NOT the lossy 0.0 telemetry float (anchor.py:262 maps its
# internal None -> 0.0 for the published message only). A hard-floor state
# with a TRUSTED line but an unseeded DC must never be mistaken for a
# trusted gap_dc == 0.0, which would spuriously satisfy calib_trim's
# `gap_dc < cfg.gap_lo` branch and integrate against a floor excursion.
# --------------------------------------------------------------------------

def _configure_mode2(data_dir):
  (data_dir / 'CalibTrimMode').write_text('2')
  (data_dir / 'CalibTrimYawSign').write_text('1')


@pytest.mark.parametrize('gap', [0.05, 2.0])  # below gap_hard_lo=0.3 / above gap_hard_hi=1.5
def test_mode2_hard_floor_unseeded_dc_never_integrates(data_dir, monkeypatch, gap):
  _configure_mode2(data_dir)
  captured = []
  monkeypatch.setattr(register, '_publish', lambda telem: captured.append(dict(telem)))
  for _ in range(200):
    register.on_curvature_correction(0.0, _mv_at(gap), 25.0, False, lat_delay=0.45)
  assert len(captured) == 200
  # sanity: the scenario really is a trusted-line hard floor with no DC seed
  assert captured[-1]['authority'] == pytest.approx(1.0)
  assert register._anchor.gap_dc is None
  for telem in captured:
    assert telem['trim_integrating'] is False
    assert telem['trim_delta_deg'] == 0.0


# --------------------------------------------------------------------------
# modeld reader path must not import the trim module (spec §6: "float file
# read, nothing else"). Break _trim_module() and confirm the reader still
# clamps correctly — if the reader secretly depended on it, the exception
# would be swallowed by _read_yaw_deg's broad except and silently mask the
# clamp as 0.0 instead of the expected 0.8.
# --------------------------------------------------------------------------

def test_calib_bias_reader_path_does_not_import_trim_module(data_dir, monkeypatch):
  def _boom():
    raise AssertionError('modeld reader path must not touch the trim module')
  monkeypatch.setattr(register, '_trim_module', _boom)
  (data_dir / 'CalibTrimYawDeg').write_text('5.0')
  assert register.on_calib_bias(0.0) == pytest.approx(0.8)
