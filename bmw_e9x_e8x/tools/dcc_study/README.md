# DCC Response Study

Measures actual vehicle acceleration per DCC stalk command×cadence from C3 rlogs.
Spec: docs/superpowers/specs/2026-07-29-dcc-response-mapping-design.md

## Run (dev machine, from this directory)

    PY="PYTHONPATH=/home/oxygen/catpilot-dev/catpilot:. /home/oxygen/catpilot-dev/catpilot/.venv/bin/python"
    # 1. pull rlogs of engaged routes from the C3 via COD API
    $PY fetch_routes.py
    # 2. rlog.zst -> per-segment npz
    $PY extract.py
    # 3. npz -> data/bursts.csv (+ data/profiles/*.npz)
    $PY bursts.py
    # 4. bursts.csv -> data/report/*.png + summary.txt
    $PY report.py

Tests: `$PY -m pytest tests/ -v` (NOT collected by the repo pre-push hook).
