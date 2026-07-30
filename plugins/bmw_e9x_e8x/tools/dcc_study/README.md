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

Step 2 is self-healing: a .npz missing any key in `extract._KEYS` is re-extracted,
so adding a channel only costs a re-run (no need to wipe data/extracted).

## Evaluate one route against a baseline

    $PY eval_route.py --route 000003d4 [--baseline 2026-07-2]

Prints, for the route and for the baseline set: tracking error vs `aTarget`,
low-speed braking bias, churn (bursts/min, direction reversals, setpoint churn
ratio), per-command acceptance, planner direction agreement — then a headline
side-by-side with percent deltas.

Two numbers to read carefully:
* acceptance is measured on the ISOLATED subset (>2 s clear of any other burst
  on both sides) and prints `UNMEASURABLE` below 10 isolated bursts. A raw rate
  over contaminated bursts once read as a 45 % rejection that was not real.
* low-speed braking says `insufficient` below 50 samples rather than reporting a
  median of noise.

Tests: `$PY -m pytest tests/ -v` (NOT collected by the repo pre-push hook).
