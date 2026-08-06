"""Regression tests for model_download.py PR ingestion.

Covers add_model_from_pr's dedup path, which referenced an undefined
`registry` name (a latent NameError) until it was loaded explicitly.
"""
import importlib
from unittest.mock import patch, MagicMock

import pytest

# model_download imports `requests` at module scope; auto-skip where the
# dependency isn't installed (same convention as the openpilot/opendbc-gated
# tests in this repo).
try:
  md = importlib.import_module('plugins.model_selector.model_download')
except ImportError:
  pytest.skip('requests not available', allow_module_level=True)


def _fake_pr(**over):
  pr = {
    'title': 'New driving model',
    'merge_commit_sha': 'a' * 40,
    'merged_at': '2025-12-01T00:00:00Z',
    'head': {'sha': 'b' * 40},
  }
  pr.update(over)
  return pr


def test_add_from_pr_reaches_registry_without_nameerror():
  # The dedup loop over the registry must not raise NameError when the
  # registry has no matching PR entry.
  resp = MagicMock()
  resp.json.return_value = _fake_pr()
  resp.raise_for_status.return_value = None
  with patch.object(md.requests, 'get', return_value=resp), \
       patch.object(md, 'load_registry', return_value=({}, {})) as load_reg, \
       patch.object(md, 'add_model_to_registry', return_value=0) as add_reg:
    rc = md.add_model_from_pr(36849, model_type='driving')
  assert rc == 0
  load_reg.assert_called_once()
  # brand-new PR -> id derived from title + PR number
  assert add_reg.call_args.kwargs['model_id'] == 'new_driving_model_36849'
  assert add_reg.call_args.kwargs['commit'] == 'b' * 40  # head sha, not merge


def test_add_from_pr_dedups_to_existing_entry():
  # An existing registry entry from the same PR is reused as the model_id.
  resp = MagicMock()
  resp.json.return_value = _fake_pr()
  resp.raise_for_status.return_value = None
  existing = {'old_name_36849': {'pr': '#36849'}}
  with patch.object(md.requests, 'get', return_value=resp), \
       patch.object(md, 'load_registry', return_value=(existing, {})), \
       patch.object(md, 'add_model_to_registry', return_value=0) as add_reg:
    md.add_model_from_pr(36849, model_type='driving')
  assert add_reg.call_args.kwargs['model_id'] == 'old_name_36849'


def test_add_from_pr_dm_type_checks_dm_registry():
  # For a dm model the dedup must scan the dm sub-registry, not driving.
  resp = MagicMock()
  resp.json.return_value = _fake_pr()
  resp.raise_for_status.return_value = None
  driving = {'driving_36849': {'pr': '#36849'}}   # must be ignored for dm
  dm = {'dm_36849': {'pr': '#36849'}}
  with patch.object(md.requests, 'get', return_value=resp), \
       patch.object(md, 'load_registry', return_value=(driving, dm)), \
       patch.object(md, 'add_model_to_registry', return_value=0) as add_reg:
    md.add_model_from_pr(36849, model_type='dm')
  assert add_reg.call_args.kwargs['model_id'] == 'dm_36849'


def test_add_from_pr_unmerged_returns_error():
  resp = MagicMock()
  resp.json.return_value = _fake_pr(merge_commit_sha=None)
  resp.raise_for_status.return_value = None
  with patch.object(md.requests, 'get', return_value=resp), \
       patch.object(md, 'add_model_to_registry') as add_reg:
    rc = md.add_model_from_pr(36849)
  assert rc == 1
  add_reg.assert_not_called()
