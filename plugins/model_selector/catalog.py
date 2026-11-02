"""Curated model catalog — the gate on which models may be installed or activated.

Compatibility policy lives here and nowhere else: model_download and
model_swapper ask this module rather than re-deriving compatibility from model
dates. A model may be offered or activated only if the catalog records that it
passed a test drive on the openpilot version this device is running — or that it
is the model the release itself ships, which is verified by definition.

This module is pure policy. It deliberately knows nothing about ONNX filenames
or storage layout; that belongs to ModelSwapper.MODEL_CONFIGS.
"""
import json
import os
import re
from pathlib import Path

CATALOG_FILE = Path(__file__).resolve().parent / 'compatible_models.json'
VERSION_H = Path(os.getenv('OPENPILOT_DIR', '/data/openpilot')) / 'openpilot' / 'common' / 'version.h'
UNLOCK_MARKER = (Path(os.getenv('PLUGINS_RUNTIME_DIR', '/data/plugins-runtime'))
                 / 'model_selector' / 'data' / '.unlocked')

MODEL_TYPES = ('driving', 'dm')
_HEX64 = re.compile(r'[0-9a-f]{64}')


def _type_key(model_type) -> str:
  """Accept either a ModelType enum or a plain 'driving'/'dm' string."""
  return getattr(model_type, 'value', model_type)


def openpilot_version() -> str:
  """Version of the openpilot code actually running, e.g. '0.11.1'.

  version.h is the truth; manifest.OPENPILOT_VERSION is a hand-maintained
  mirror that drifts between rebases, so it is only a fallback. Parsing is
  import-free because this runs inside a bare-venv CLI subprocess too.
  """
  try:
    return VERSION_H.read_text().split('"')[1]
  except (OSError, IndexError):
    pass
  try:
    from openpilot.selfdrive.plugins.manifest import OPENPILOT_VERSION
    return OPENPILOT_VERSION
  except Exception:
    return ''


def load_catalog() -> dict:
  """Parse the catalog. Returns {} on any problem — the gate fails closed.

  Any failure mode (missing file, malformed JSON, non-UTF-8 bytes, or valid
  JSON of the wrong shape) must degrade to an empty catalog rather than raise
  — this is called from the UI thread with no caller-side guard.
  """
  try:
    with open(CATALOG_FILE) as f:
      data = json.load(f)
  except Exception:
    return {}
  if not isinstance(data, dict):
    return {}
  result = {}
  for t in MODEL_TYPES:
    entries = data.get(t, [])
    if not isinstance(entries, list):
      entries = []
    result[t] = [e for e in entries if isinstance(e, dict) and e.get('id')]
  return result


def verified_entries(model_type) -> list:
  """Catalog entries verified for the running openpilot version."""
  version = openpilot_version()
  if not version:
    return []
  return [e for e in load_catalog().get(_type_key(model_type), [])
          if version in e.get('verified_on', [])]


def is_verified(model_type, model_id: str) -> bool:
  return any(e['id'] == model_id for e in verified_entries(model_type))


def entry(model_type, model_id: str):
  """The catalog entry with this id, regardless of version, or None."""
  for e in load_catalog().get(_type_key(model_type), []):
    if e['id'] == model_id:
      return e
  return None


def baseline_entry(model_type):
  """The known-good fallback model for the running version, or None."""
  version = openpilot_version()
  for e in verified_entries(model_type):
    if version in e.get('baseline_for', []):
      return e
  return None


def unlocked() -> bool:
  """True when the maintainer has unlocked untested models on this device."""
  return UNLOCK_MARKER.exists()


def _hash_problems(t, eid, obj) -> list:
  problems = []
  sha = obj.get('sha256')
  if not sha:
    problems.append(f"{t}: entry '{eid}' missing sha256")
  elif not (isinstance(sha, str) and _HEX64.fullmatch(sha)):
    problems.append(f"{t}: entry '{eid}' bad sha256")
  size = obj.get('size')
  if not isinstance(size, int) or isinstance(size, bool) or size <= 0:
    problems.append(f"{t}: entry '{eid}' size must be a positive int")
  return problems


def validate_catalog(catalog: dict | None = None) -> list:
  """Return a list of problems with the catalog. Empty list means valid.

  Catches malformed catalogs at push time rather than on a device.
  """
  catalog = load_catalog() if catalog is None else catalog
  problems = []

  versions = set()
  for t in MODEL_TYPES:
    for e in catalog.get(t, []):
      versions.update(e.get('verified_on', []))

  for t in MODEL_TYPES:
    entries = catalog.get(t, [])

    ids = [e.get('id') for e in entries]
    for dup in sorted({i for i in ids if ids.count(i) > 1}):
      problems.append(f"{t}: duplicate entry '{dup}'")

    for e in entries:
      eid = e.get('id', '?')
      source = e.get('source')
      required = ['id', 'name', 'date']
      if not isinstance(e.get('verified_on'), list):
        problems.append(f"{t}: entry '{eid}' missing verified_on")
      if source == 'converted':
        required += ['converted_from', 'files']
      elif source != 'shipped':
        required += ['commit', 'files']
      for field in required:
        if not e.get(field):
          problems.append(f"{t}: entry '{eid}' missing {field}")
      if source == 'shipped':
        problems += _hash_problems(t, eid, e)
      if source == 'converted':
        cf = e.get('converted_from')
        if cf and not (isinstance(cf, dict) and cf.get('commit')):
          problems.append(f"{t}: entry '{eid}' converted_from needs a commit")
        files = e.get('files')
        if files:
          if not isinstance(files, list) or not all(isinstance(f, dict) for f in files):
            problems.append(f"{t}: entry '{eid}' files must be objects with name/url/sha256/size")
          else:
            for f in files:
              if not f.get('name'):
                problems.append(f"{t}: entry '{eid}' file missing name")
              if not str(f.get('url') or '').startswith('https://'):
                problems.append(f"{t}: entry '{eid}' file '{f.get('name', '?')}' needs an https url")
              problems += _hash_problems(t, f"{eid}/{f.get('name', '?')}", f)
      for ver in e.get('baseline_for', []):
        if ver not in e.get('verified_on', []):
          problems.append(f"{t}: '{eid}' is baseline_for {ver} but not verified_on it")

    for ver in sorted(versions):
      n = sum(1 for e in entries if ver in e.get('baseline_for', []))
      if n != 1:
        problems.append(f"{t}: version {ver} has {n} baselines, expected exactly 1")

  return problems
