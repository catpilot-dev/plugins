"""Offline replay of the Model Centring term (not part of the hook path).

centring_series() runs the SAME CentringTerm code the hook uses (anchor.py),
so a replay over logged raw / Arc / speed series reproduces the live term.
"""
import importlib.util
import os

_ANCHOR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'anchor.py')
_mod = None


def _anchor():
  global _mod
  if _mod is None:
    spec = importlib.util.spec_from_file_location('lk_anchor_replay', _ANCHOR)
    _mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(_mod)
  return _mod


def centring_series(raw, arc, v, lat_active, lc_active, tc=4.0):
  """Per-tick applied term g(v)*s (100 Hz ticks, like the hook).

  raw: model desiredCurvature; arc: the Arc output (kappa_arc_ref); v: m/s;
  lat_active: engaged flag (True = the hook was handed the model's curvature);
  lc_active: laneChangeStarting/Finishing flag; tc: time constant (s).
  """
  term = _anchor().CentringTerm(tc)
  return [term.step(float(r), float(a), float(vv), bool(e), bool(lc))
          for r, a, vv, e, lc in zip(raw, arc, v, lat_active, lc_active)]
