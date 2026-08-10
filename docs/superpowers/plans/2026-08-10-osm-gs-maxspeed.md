# OSM G/S maxspeed gate + source indicator — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** In China, trust a posted OSM maxspeed only on a confirmed G/S expressway at ≥ 60 km/h; everywhere else keep today's behaviour — and show `OSM` / `YOLO` / `VISION` below the speed limit sign so the active source is visible on road.

**Architecture:** A single gate function `_osm_gate()` returns `(trusted, reason)` and becomes the one source of truth feeding all four existing consumers (arbitration, ≤2-lane cap bypass, display rounding, telemetry). It replaces `_osm_base_active()` outright rather than wrapping it, so no uncalled method is left behind. The CN restriction reuses the already-tested `is_gs_expressway_ref()` predicate and the existing `gs_mode` state machine, so it inherits every G/S release guard for free.

**Tech Stack:** Python 3, pytest, pyray (mocked in tests), openpilot plugin framework.

**Spec:** `docs/superpowers/specs/2026-08-10-osm-gs-maxspeed-design.md`

## Global Constraints

- Repo: `/home/oxygen/catpilot-dev/plugins`, branch `dev`. All paths below are relative to it.
- Run tests with `.venv/bin/python3 -m pytest plugins/*/tests/ -q` (the exact command the pre-push hook uses). Running `plugins/speedlimitd/tests/` alone is fine here — unlike ui_mod, it has no cross-plugin import ordering dependency.
- **Before the first test run**, check `echo $PYTHONPATH`. If it contains a path like `~/catpilot-dev/plugins-sign_vision`, unset it — a foreign `plugins/__init__.py` hijacks the namespace package and silently tests the wrong worktree.
- No `Co-Authored-By` lines in commit messages.
- Do NOT deploy to the C3, do NOT `ssh c3`. This plan is dev-machine only. Deployment is a separate decision.
- No behaviour change outside China: every task must keep `country not in ('', 'cn')` on exactly today's code path.
- The CN default for `OsmDataIntegration` stays `'0'`. Do not change `_resolve_osm_default()`'s default logic.
- All UI imports in `ui_overlay.py` stay lazy / module-level-guarded as they already are; do not add a top-level openpilot import.

## File Structure

| File | Responsibility | Change |
| --- | --- | --- |
| `plugins/speedlimitd/speedlimitd.py` | Gate, arbitration, telemetry, docstring | Modify |
| `plugins/speedlimitd/ui_overlay.py` | Sign rendering + source label | Modify |
| `plugins/speedlimitd/tests/test_speedlimitd.py` | Gate + arbitration tests | Modify |
| `plugins/speedlimitd/tests/test_ui_overlay.py` | Label mapping tests | Modify |
| `plugins/speedlimitd/DESIGN.md`, `README.md` | Docs | Modify |

---

### Task 1: Rename `_osm_default_country` → `country`

Pure refactor, no behaviour change. Task 2 needs a clearly-named, always-present country field; doing the rename first keeps Task 2's diff about the gate only.

**Files:**
- Modify: `plugins/speedlimitd/speedlimitd.py:785`, `:867`, `:879`, `:1102`, `:1104`
- Test: `plugins/speedlimitd/tests/test_speedlimitd.py`

**Interfaces:**
- Consumes: nothing.
- Produces: `self.country: str` — GPS-detected ISO country code, `''` until the first valid GPS fix resolves it. Task 2 reads it.

- [ ] **Step 1: Write the failing test**

Add to `tests/test_speedlimitd.py`, at the end of the file:

```python
class TestCountryField:
  def _make_middleware(self, sld):
    import plugins.speedlimitd.speedlimitd as mod
    with patch.object(mod.messaging, 'SubMaster'):
      mw = mod.SpeedLimitMiddleware()
    mw._sl_pub = MagicMock()
    return mw

  def test_country_defaults_to_empty_string(self, sld):
    mw = self._make_middleware(sld)
    assert mw.country == ''

  def test_resolve_osm_default_reads_country(self, sld):
    """cn → OFF, anything else → ON (unchanged behaviour, new field name)."""
    mw = self._make_middleware(sld)
    mw.country = 'cn'
    with patch('config.write_plugin_param') as wp, \
         patch('config.plugin_data_dir') as pdd:
      pdd.return_value.__truediv__.return_value.exists.return_value = False
      mw._resolve_osm_default(mw.country)
      assert wp.call_args[0][2] == '0'
```

- [ ] **Step 2: Run test to verify it fails**

Run: `.venv/bin/python3 -m pytest plugins/speedlimitd/tests/test_speedlimitd.py::TestCountryField -v`
Expected: FAIL — `AttributeError: 'SpeedLimitMiddleware' object has no attribute 'country'`

- [ ] **Step 3: Apply the rename**

At `speedlimitd.py:785`, replace:

```python
    self._osm_default_country: str | None = None
```

with:

```python
    # GPS-detected country code, '' until the first valid fix. Read by the OSM
    # gate, which treats '' as strict-CN (fail safe before the fix lands).
    self.country: str = ''
```

At `:867`, change the signature and docstring first line:

```python
  def _resolve_osm_default(self, country: str) -> bool:
```

At `:1102`, replace `self._osm_default_country = country` with:

```python
          self.country = country or ''
```

At `:1104`, replace `self._resolve_osm_default(self._osm_default_country)` with:

```python
          self._osm_default_resolved = self._resolve_osm_default(self.country)
```

- [ ] **Step 4: Verify no stragglers**

Run: `grep -n "_osm_default_country" plugins/speedlimitd/`
Expected: no output.

- [ ] **Step 5: Run the full plugin suite**

Run: `.venv/bin/python3 -m pytest plugins/*/tests/ -q`
Expected: PASS — same count as before the change plus 2 (was 468 passed, 22 skipped).

- [ ] **Step 6: Commit**

```bash
git add plugins/speedlimitd/speedlimitd.py plugins/speedlimitd/tests/test_speedlimitd.py
git commit -m "speedlimitd: rename _osm_default_country to country

It already holds the GPS-detected country but was named as if it only
existed for OSM default resolution. The OSM gate is about to read it."
```

---

### Task 2: CN G/S-only gate

**Files:**
- Modify: `plugins/speedlimitd/speedlimitd.py:886-895` (replace `_osm_base_active`), `:1412` (call site), constants near `:646`
- Test: `plugins/speedlimitd/tests/test_speedlimitd.py` — new `TestOsmGsGate`, plus migration of `TestOsmBaseActive` and `TestOsmBaseSelection`

**Interfaces:**
- Consumes: `self.country` from Task 1; existing `is_gs_expressway_ref(way_ref) -> bool` (`:678`); existing `gs_mode` local computed at `:1406`.
- Produces:
  - `GS_OSM_MIN_KPH = 60` (module constant)
  - `_osm_gate(self, now: float, gs_mode: bool) -> tuple[bool, str]` — `(trusted, reason)`; reason `''` when trusted, else one of `'disabled'|'no_data'|'stale'|'not_gs'|'low_value'`. Task 3 publishes both halves.
  - `_osm_base_active` is **removed**; `_osm_gate` is the single predicate feeding arbitration, the ≤2-lane cap bypass, display rounding and telemetry.

- [ ] **Step 1: Migrate the existing tests to state their country explicitly**

The existing OSM tests arm a posted limit with `lane_count_stable = 2`. Under the new gate an unset country means strict-CN, and 2 lanes releases `gs_mode`, so they would fail for the *right* reason. Their intent is "non-CN, as shipped", so make that explicit.

In `TestOsmBaseActive._arm` (`:2774`) add `mw.country = 'us'`:

```python
  def _arm(self, mw, kph=100.0, age=0.0):
    import time as _t
    mw.country = 'us'          # non-CN: gate is the pre-2026-08-10 behaviour
    mw.osm_integration_enabled = True
    mw.last_osm_speed_kph = kph
    mw.last_osm_speed_t = _t.monotonic() - age
```

In the same class, `_osm_base_active` is being replaced outright by `_osm_gate` (Step 5), so update all four call sites to unpack the tuple. `gs_mode=False` is deliberate: on the non-CN path it must be irrelevant, and passing `False` proves it.

```python
    assert mw._osm_gate(_t.monotonic(), gs_mode=False)[0] is True    # test_active_when_fresh_and_enabled
    assert mw._osm_gate(_t.monotonic(), gs_mode=False)[0] is False   # the three inactive_* tests
```

In `TestOsmBaseSelection._arm` (`:2822`) add the same line:

```python
  def _arm(self, mw, kph):
    import time as _t
    mw.country = 'us'          # non-CN: gate is the pre-2026-08-10 behaviour
    mw.osm_integration_enabled = True
    mw.last_osm_speed_kph = kph
    mw.last_osm_speed_t = _t.monotonic()
    mw._params_last_read_t = _t.monotonic()
```

- [ ] **Step 2: Write the failing tests**

Append to `tests/test_speedlimitd.py`:

```python
class TestOsmGsGate:
  """CN: a posted OSM maxspeed is trusted only on a confirmed G/S expressway."""

  def _mw(self, sld):
    import plugins.speedlimitd.speedlimitd as mod
    with patch.object(mod.messaging, 'SubMaster'):
      mw = mod.SpeedLimitMiddleware()
    mw._sl_pub = MagicMock()
    return mw

  def _arm_cn(self, mw, kph, way_ref):
    import time as _t
    mw.country = 'cn'
    mw.osm_integration_enabled = True
    mw.last_osm_speed_kph = kph
    mw.last_osm_speed_t = _t.monotonic()
    mw.last_way_ref = way_ref

  def test_gs_expressway_is_trusted(self, sld):
    import time as _t
    mw = self._mw(sld)
    self._arm_cn(mw, 100.0, 'G1503')
    assert mw._osm_gate(_t.monotonic(), gs_mode=True) == (True, '')

  def test_ramp_sign_on_expressway_rejected(self, sld):
    """Audit: 40 km/h ramp signs mis-attributed onto 高速/高架 mainlines."""
    import time as _t
    mw = self._mw(sld)
    self._arm_cn(mw, 40.0, 'G1503')
    assert mw._osm_gate(_t.monotonic(), gs_mode=True) == (False, 'low_value')

  def test_viaduct_tag_on_surface_road_rejected(self, sld):
    """Audit: an elevated deck's 80 written onto the arterial beneath it."""
    import time as _t
    mw = self._mw(sld)
    self._arm_cn(mw, 80.0, '')          # 华夏中路 etc. carry no G/S ref
    assert mw._osm_gate(_t.monotonic(), gs_mode=True) == (False, 'not_gs')

  def test_three_digit_guodao_rejected(self, sld):
    """G312 is an ordinary surface highway, not a controlled-access expressway."""
    import time as _t
    mw = self._mw(sld)
    self._arm_cn(mw, 80.0, 'G312')
    assert mw._osm_gate(_t.monotonic(), gs_mode=True) == (False, 'not_gs')

  def test_gs_released_rejects_even_with_ref(self, sld):
    """Exiting onto a ≤2-lane ramp releases gs_mode; the posted 100 must not hold."""
    import time as _t
    mw = self._mw(sld)
    self._arm_cn(mw, 100.0, 'G1503')
    assert mw._osm_gate(_t.monotonic(), gs_mode=False) == (False, 'not_gs')

  def test_unknown_country_uses_strict_gate(self, sld):
    """Before the first GPS fix an opted-in CN user must not get the open gate."""
    import time as _t
    mw = self._mw(sld)
    self._arm_cn(mw, 80.0, '')
    mw.country = ''
    assert mw._osm_gate(_t.monotonic(), gs_mode=True) == (False, 'not_gs')

  def test_non_cn_ignores_gs_entirely(self, sld):
    """US/EU regression guard: any way, no G/S ref, still trusted."""
    import time as _t
    mw = self._mw(sld)
    self._arm_cn(mw, 80.0, '')
    mw.country = 'us'
    assert mw._osm_gate(_t.monotonic(), gs_mode=False) == (True, '')

  def test_reason_precedence_disabled_beats_not_gs(self, sld):
    import time as _t
    mw = self._mw(sld)
    self._arm_cn(mw, 80.0, '')
    mw.osm_integration_enabled = False
    assert mw._osm_gate(_t.monotonic(), gs_mode=True) == (False, 'disabled')

  def test_reason_precedence_stale_beats_not_gs(self, sld):
    import time as _t
    mw = self._mw(sld)
    self._arm_cn(mw, 80.0, '')
    mw.last_osm_speed_t = _t.monotonic() - 11.0
    assert mw._osm_gate(_t.monotonic(), gs_mode=True) == (False, 'stale')
```

- [ ] **Step 3: Run tests to verify they fail**

Run: `.venv/bin/python3 -m pytest plugins/speedlimitd/tests/test_speedlimitd.py::TestOsmGsGate -v`
Expected: FAIL — `AttributeError: 'SpeedLimitMiddleware' object has no attribute '_osm_gate'`

- [ ] **Step 4: Add the constant**

In `speedlimitd.py`, immediately after `LANE_COUNT_LIMIT_4 = 80` (`:647`):

```python
# CN ramp-sign guard (2026-08-10). The OSM audit found 40 km/h ramp signs
# map-matched onto 高速/高架 mainlines; no safety cap catches a wrong-and-low
# tag on a straight road. No real G/S expressway is posted below this.
GS_OSM_MIN_KPH = 60
```

- [ ] **Step 5: Replace `_osm_base_active` with the gate**

Replace the whole of `speedlimitd.py:886-895` with:

```python
  def _osm_gate(self, now: float, gs_mode: bool) -> tuple[bool, str]:
    """Decide whether the posted OSM maxspeed may serve as the base inference.

    Returns (trusted, reason). reason is '' when trusted, otherwise the FIRST
    failing condition in the fixed order below, so telemetry is deterministic.

    Freshness is 2 query intervals (~10 s): one missed/None query keeps the
    limit, a sustained loss falls back to vision inference. 30 km/h floor
    rejects implausible tags (same floor as MIN_SPEED_LIMIT in update()).

    In China — and whenever the country is not yet known, which must fail safe
    rather than fall through to the permissive path — the posted value is
    trusted only on a confirmed G/S expressway at >= GS_OSM_MIN_KPH. That
    excludes both audited map-matching failure modes: viaduct tags landing on
    the surface road beneath (no G/S ref) and ramp signs landing on the
    mainline (below the floor). Requiring gs_mode inherits every G/S release
    guard — the <=2-lane release, the margin rule, gs_lane_drop and the
    continuous-absence timer — at no extra cost.
    See docs/superpowers/specs/2026-08-10-osm-gs-maxspeed-design.md
    """
    if not self.osm_integration_enabled:
      return False, 'disabled'
    if self.last_osm_speed_kph < 30.0:
      return False, 'no_data'
    if now - self.last_osm_speed_t > 2.0 * self._osm_query_interval:
      return False, 'stale'
    if self.country and self.country != 'cn':
      return True, ''                       # non-CN: unchanged, as shipped
    if not (gs_mode and is_gs_expressway_ref(self.last_way_ref)):
      return False, 'not_gs'
    if self.last_osm_speed_kph < GS_OSM_MIN_KPH:
      return False, 'low_value'
    return True, ''
```

`_osm_gate` fully replaces `_osm_base_active` — do not keep the old method as a
wrapper. After Step 6 the only production call site is the arbitration block, so a
wrapper would be uncalled code that still looks like API.

- [ ] **Step 6: Update the call site**

At `speedlimitd.py:1412`, replace `osm_base = self._osm_base_active(now)` with:

```python
    osm_trusted, osm_reject_reason = self._osm_gate(now, gs_mode)
    osm_base = osm_trusted
```

(Task 3 publishes `osm_reject_reason`; binding it here keeps that a one-line addition.)

- [ ] **Step 7: Run the gate tests**

Run: `.venv/bin/python3 -m pytest plugins/speedlimitd/tests/test_speedlimitd.py::TestOsmGsGate -v`
Expected: PASS, 9 tests.

- [ ] **Step 8: Run the full plugin suite**

Run: `.venv/bin/python3 -m pytest plugins/*/tests/ -q`
Expected: PASS. If `TestOsmBaseSelection` fails, Step 1's `mw.country = 'us'` was missed — those tests encode non-CN intent and must state it.

- [ ] **Step 9: Commit**

```bash
git add plugins/speedlimitd/speedlimitd.py plugins/speedlimitd/tests/test_speedlimitd.py
git commit -m "speedlimitd: trust OSM maxspeed only on G/S expressways in CN

The audit found two systematic map-matching failures in our region: viaduct
tags on the surface road beneath, and 40 km/h ramp signs on 高速 mainlines.
Neither is caught by a safety cap. Requiring a G/S ref excludes the first,
a 60 km/h floor the second. Unknown country is treated as CN so an opted-in
driver is not exposed between boot and the first GPS fix. Non-CN unchanged."
```

---

### Task 3: Publish gate telemetry

**Files:**
- Modify: `plugins/speedlimitd/speedlimitd.py:1566-1568` (publish block)
- Test: `plugins/speedlimitd/tests/test_speedlimitd.py`

**Interfaces:**
- Consumes: `osm_trusted`, `osm_reject_reason` locals bound in Task 2 Step 6.
- Produces: published keys `osmTrusted: bool`, `osmRejectReason: str`.

- [ ] **Step 1: Write the failing test**

Append to `tests/test_speedlimitd.py`:

```python
class TestOsmGateTelemetry:
  def _mw_for_update(self, sld):
    import plugins.speedlimitd.speedlimitd as mod
    with patch.object(mod.messaging, 'SubMaster'):
      mw = mod.SpeedLimitMiddleware()
    mw._sl_pub = MagicMock()
    sm = MagicMock()
    sm.updated = {'modelV2': False, 'gpsLocationExternal': False, 'livePose': False}
    sm.update = MagicMock()
    mw.sm = sm
    mw._cmd_sub = None
    mw._lc_sub = None
    return mw

  def test_publishes_reject_reason_for_low_value(self, sld):
    import time as _t
    mw = self._mw_for_update(sld)
    mw.country = 'cn'
    mw.osm_integration_enabled = True
    mw.last_osm_speed_kph = 40.0
    mw.last_osm_speed_t = _t.monotonic()
    mw.last_way_ref = 'G1503'
    mw._params_last_read_t = _t.monotonic()
    # Hold gs_mode open: 4 lanes, freshly seen, no release latched.
    mw.lane_count_stable = 4
    mw._gs_last_seen_t = _t.monotonic()
    mw._gs_limit_kph = 100
    mw._gs_force_release = False
    mw._gs_absent_since = None
    mw.update()
    pub = mw._sl_pub.send.call_args[0][0]
    assert pub['osmTrusted'] is False
    assert pub['osmRejectReason'] == 'low_value'
    assert pub['osmSpeedLimit'] == 40.0      # still reported for telemetry
    assert pub['inferenceMode'] != 'osm'

  def test_publishes_trusted_with_empty_reason(self, sld):
    import time as _t
    mw = self._mw_for_update(sld)
    mw.country = 'cn'
    mw.osm_integration_enabled = True
    mw.last_osm_speed_kph = 100.0
    mw.last_osm_speed_t = _t.monotonic()
    mw.last_way_ref = 'G1503'
    mw._params_last_read_t = _t.monotonic()
    # Hold gs_mode open: 4 lanes, freshly seen, no release latched.
    mw.lane_count_stable = 4
    mw._gs_last_seen_t = _t.monotonic()
    mw._gs_limit_kph = 100
    mw._gs_force_release = False
    mw._gs_absent_since = None
    mw.update()
    pub = mw._sl_pub.send.call_args[0][0]
    assert pub['osmTrusted'] is True
    assert pub['osmRejectReason'] == ''
    assert pub['inferenceMode'] == 'osm'
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `.venv/bin/python3 -m pytest plugins/speedlimitd/tests/test_speedlimitd.py::TestOsmGateTelemetry -v`
Expected: FAIL with `KeyError: 'osmTrusted'`

- [ ] **Step 3: Add the fields**

In the publish block, replace:

```python
      'osmSpeedLimit': round(self.last_osm_speed_kph, 1),
      'osmTilesMissing': self._osm_tiles_missing,
```

with:

```python
      'osmSpeedLimit': round(self.last_osm_speed_kph, 1),
      'osmTilesMissing': self._osm_tiles_missing,
      # Gate outcome (2026-08-10). osmSpeedLimit above stays unconditional, so
      # an rlog records what OSM claimed even when the gate rejected it —
      # 'low_value' counts measure how live the ramp-sign mis-attribution is.
      'osmTrusted': osm_trusted,
      'osmRejectReason': osm_reject_reason,
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `.venv/bin/python3 -m pytest plugins/speedlimitd/tests/test_speedlimitd.py::TestOsmGateTelemetry -v`
Expected: PASS, 2 tests.

- [ ] **Step 5: Run the full plugin suite**

Run: `.venv/bin/python3 -m pytest plugins/*/tests/ -q`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add plugins/speedlimitd/speedlimitd.py plugins/speedlimitd/tests/test_speedlimitd.py
git commit -m "speedlimitd: publish osmTrusted and osmRejectReason

Makes the CN gate measurable from rlogs, which is the evidence the CN
default flip was gated on."
```

---

### Task 4: `OSM` / `YOLO` / `VISION` indicator

**Files:**
- Modify: `plugins/speedlimitd/ui_overlay.py` — constants (after `:18`), `_update_state` (`:58-93`), `_draw_speed_limit_sign` (`:109-143`)
- Test: `plugins/speedlimitd/tests/test_ui_overlay.py`

**Interfaces:**
- Consumes: published `source` and `inferenceMode` (already on the bus; `inferenceMode` is newly *read* by the UI).
- Produces: `_source_label() -> str` returning exactly `'OSM'`, `'YOLO'` or `'VISION'`.

- [ ] **Step 1: Write the failing test**

Append to `tests/test_ui_overlay.py`:

```python
class TestSourceLabel:
  """OSM / YOLO / VISION indicator — posted vs inferred is the distinction."""

  def test_yolo_source(self, overlay):
    overlay._speed_limit_source = 1
    overlay._inference_mode = ''
    assert overlay._source_label() == 'YOLO'

  def test_osm_posted_maxspeed(self, overlay):
    overlay._speed_limit_source = 2
    overlay._inference_mode = 'osm'
    assert overlay._source_label() == 'OSM'

  def test_gs_osm_is_vision_not_osm(self, overlay):
    """gs_osm takes only the road CLASS from OSM; the number is table-inferred."""
    overlay._speed_limit_source = 2
    overlay._inference_mode = 'gs_osm'
    assert overlay._source_label() == 'VISION'

  def test_lane_count_is_vision(self, overlay):
    overlay._speed_limit_source = 2
    overlay._inference_mode = 'lane_count'
    assert overlay._source_label() == 'VISION'

  def test_safety_cap_is_vision(self, overlay):
    """source 4 = curvature / reactive a_y cap, both vision-derived."""
    overlay._speed_limit_source = 4
    overlay._inference_mode = 'osm'
    assert overlay._source_label() == 'VISION'

  def test_update_state_reads_inference_mode(self, overlay):
    overlay._sl_data = {'speedLimit': 100, 'source': 2, 'inferenceMode': 'osm'}
    overlay._sl_sub = None
    overlay._update_state()
    assert overlay._inference_mode == 'osm'
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `.venv/bin/python3 -m pytest plugins/speedlimitd/tests/test_ui_overlay.py::TestSourceLabel -v`
Expected: FAIL — `AttributeError: module ... has no attribute '_source_label'`

- [ ] **Step 3: Add layout constants**

In `ui_overlay.py`, after `SPEED_SIGN_FONT_SIZE = 84` (`:18`):

```python
SOURCE_LABEL_FONT_SIZE = 36       # source indicator below the sign
SOURCE_LABEL_GAP = 12             # gap between sign bottom and label top
```

- [ ] **Step 4: Track the inference mode**

At `:35`, after `_speed_limit_source = 2`, add:

```python
_inference_mode = ''       # 'osm' | 'gs_osm' | 'lane_count'
```

In `_update_state`, extend the globals declaration at `:60`:

```python
  global _speed_limit, _speed_limit_source, _speed_limit_confirmed, _inference_mode
```

and inside the `if _sl_data is not None:` block after `:91`:

```python
    _inference_mode = _sl_data.get('inferenceMode', '')
```

- [ ] **Step 5: Add the label function**

Immediately above `_draw_speed_limit_sign` (`:109`):

```python
def _source_label() -> str:
  """Which source produced the displayed limit: posted, sign-read, or inferred.

  'gs_osm' maps to VISION deliberately — in that mode OSM supplies only the
  road class while the number comes from the lane-count/road-type table. The
  distinction drawn here is posted vs inferred, which is what makes the label
  a useful on-road check: on a G/S expressway, OSM means the posted tag passed
  the gate, VISION means it was rejected or absent.

  YOLO cannot appear yet — speedlimitd's yolo_speed is a permanent 0 until
  sign vision is wired. The branch is here so that work needs no UI change.
  """
  if _speed_limit_source == 1:
    return 'YOLO'
  if _speed_limit_source == 2 and _inference_mode == 'osm':
    return 'OSM'
  return 'VISION'
```

- [ ] **Step 6: Draw it**

At the end of `_draw_speed_limit_sign`, after the `rl.draw_text_ex` that renders the speed number (`:136-143`):

```python
  # Source indicator, centred below the sign. Full-opacity white regardless of
  # the sign's confirmed alpha — it is a diagnostic readout, and legibility
  # over a bright camera feed matters more than matching the sign's
  # suggestion/active semantics.
  label = _source_label()
  label_size = measure(_font_medium, label, SOURCE_LABEL_FONT_SIZE)
  rl.draw_text_ex(
    _font_medium,
    label,
    rl.Vector2(cx - label_size.x / 2, cy + r + SOURCE_LABEL_GAP),
    SOURCE_LABEL_FONT_SIZE,
    0,
    rl.Color(255, 255, 255, 255),
  )
```

- [ ] **Step 7: Update the stale docstring promise**

`_draw_speed_limit_sign`'s docstring at `:113` currently reads `Small source indicator below: "OSM" / "SIGN" / "~"`. Replace that line with:

```python
  Source indicator below the sign: "OSM" / "YOLO" / "VISION".
```

- [ ] **Step 8: Run tests to verify they pass**

Run: `.venv/bin/python3 -m pytest plugins/speedlimitd/tests/test_ui_overlay.py -v`
Expected: PASS — the 6 new tests plus the existing ones.

- [ ] **Step 9: Run the full plugin suite**

Run: `.venv/bin/python3 -m pytest plugins/*/tests/ -q`
Expected: PASS.

- [ ] **Step 10: Commit**

```bash
git add plugins/speedlimitd/ui_overlay.py plugins/speedlimitd/tests/test_ui_overlay.py
git commit -m "speedlimitd: show OSM/YOLO/VISION below the speed limit sign

Implements the indicator ui_overlay.py has documented but never drawn, so
the active source is verifiable on road. gs_osm reads as VISION: the label
distinguishes posted from inferred, not which data touched the decision."
```

---

### Task 5: Documentation

**Files:**
- Modify: `plugins/speedlimitd/speedlimitd.py:1-10` (module docstring), `plugins/speedlimitd/DESIGN.md`, `plugins/speedlimitd/README.md`

**Interfaces:**
- Consumes: everything from Tasks 2–4. No code changes; docs only.

- [ ] **Step 1: Replace the misleading module docstring**

The current docstring advertises a live three-tier cascade whose top two tiers have no runtime path. With the YOLO team about to integrate against this module, replace `speedlimitd.py:1-10` entirely with:

```python
#!/usr/bin/env python3
"""
Speed Limit Middleware — publishes one speedLimitState message at 5 Hz.

Base inference picks ONE source, in this order:
  1. OSM posted maxspeed — only when OsmDataIntegration is ON and _osm_gate()
     opens. In China, and before the country is known, that gate additionally
     requires a confirmed G/S expressway ref and >= GS_OSM_MIN_KPH. See
     docs/superpowers/specs/2026-08-10-osm-gs-maxspeed-design.md
  2. G/S expressway table — sticky promote from the OSM road class ('gs_osm')
  3. Vision lane-count table ('lane_count')

Safety caps (predicted-curvature and reactive measured-a_y) then min() over
the base and can only ever lower it.

NOT WIRED — do not read these as live inputs:
  - YOLO sign reading: self.yolo_speed is never assigned (permanently 0), so
    the tier-1 branch in update() is unreachable. Sign vision is in
    development; assigning yolo_speed is the only hook it needs.
  - mapd suggestedSpeed: no runtime path. 'mapdSuggested' survives only in
    cereal/slot0.capnp, which is itself out of sync with what is published.
"""
```

- [ ] **Step 2: Locate the DESIGN.md source-priority section**

Run: `grep -n "OSM\|priority\|Priority\|placeholder" plugins/speedlimitd/DESIGN.md | head -30`

Note the line number of the source-priority table (the one whose YOLO row is marked "placeholder, currently always 0", around `:46`).

- [ ] **Step 3: Document the gate in DESIGN.md**

Immediately after that priority table, insert:

```markdown
### OSM maxspeed gate (CN, 2026-08-10)

`_osm_gate(now, gs_mode) -> (trusted, reason)` is the single predicate deciding
whether a posted OSM maxspeed replaces the base inference. It feeds arbitration,
the ≤2-lane vision-cap bypass, display rounding and telemetry, so those four
cannot drift apart.

Outside China the gate is toggle + 30 km/h floor + 10 s freshness, exactly as
shipped 2026-08-07. In China — and whenever `self.country` is still `''`, which
fails safe rather than falling through to the permissive path — the posted value
must additionally sit on a confirmed G/S expressway (`gs_mode` true **and**
`is_gs_expressway_ref(last_way_ref)`) and be at least `GS_OSM_MIN_KPH` (60).

This targets the two systematic map-matching failures found in the 2026-08-07
audit, neither of which any safety cap can catch, since both are wrong numbers
on straight roads:

| Failure mode | Excluded by |
| --- | --- |
| Viaduct tag matched to the surface road beneath (华夏中路, 金海路, 申江路) | no G/S ref on the surface way |
| 40 km/h ramp sign matched to a 高速/高架 mainline | the 60 km/h floor |

Requiring `gs_mode` rather than the ref alone inherits every G/S release guard:
the ≤2-lane release, the margin rule, `gs_lane_drop`, and the continuous-absence
timer. A cleared `last_way_ref` (no tile match) therefore closes the gate at once
rather than coasting on the 10 s freshness window.

`osmTrusted` and `osmRejectReason` (`''｜disabled｜no_data｜stale｜not_gs｜low_value`)
are published every tick, alongside the unconditional `osmSpeedLimit`, so rlogs
record what OSM claimed even when it was rejected. The CN default for
`OsmDataIntegration` remains OFF; `low_value` and `not_gs` rates are the evidence
for ever revisiting that.
```

- [ ] **Step 4: Document the indicator in README.md**

`README.md` is user-facing. Add, after the section describing the speed limit sign:

```markdown
### Where the limit came from

A small white label under the sign names the source:

- **OSM** — a posted speed limit read from OpenStreetMap map data. In China this
  appears only on G/S expressways, where the map data has been verified.
- **VISION** — inferred from what the camera sees (lane count, road type) or
  lowered by a curve/lateral-acceleration safety cap. This is the normal label
  on ordinary roads.
- **YOLO** — read directly off a road sign. Not active yet; sign reading is
  still in development.
```

- [ ] **Step 5: Verify nothing broke**

Run: `.venv/bin/python3 -m pytest plugins/*/tests/ -q`
Expected: PASS — docs-only task, counts unchanged from Task 4.

- [ ] **Step 6: Commit**

```bash
git add plugins/speedlimitd/speedlimitd.py plugins/speedlimitd/DESIGN.md plugins/speedlimitd/README.md
git commit -m "speedlimitd: document the CN G/S gate; fix the three-tier docstring

The module docstring presented YOLO and mapd as live tiers when neither has
a runtime path — an active hazard with the YOLO team about to integrate."
```

---

## Verification

After Task 5, confirm the whole change before any deploy decision:

- [ ] `.venv/bin/python3 -m pytest plugins/*/tests/ -q` — expect 468 + ~19 new passing, 22 skipped, 0 failures.
- [ ] `grep -rn "_osm_default_country\|_osm_base_active" plugins/speedlimitd/` — expect no output (rename complete, old predicate fully replaced).
- [ ] `git log --oneline dev ^origin/dev` — expect 5 commits, one per task.
- [ ] Confirm non-CN behaviour is untouched by reading the diff of `_osm_gate`: the `self.country and self.country != 'cn'` early return must precede every G/S condition.

Deployment to the C3 is a separate, explicit decision — this plan does not include it. Once deployed, the on-road check is: drive a G/S expressway with `OsmDataIntegration` toggled ON and confirm the label reads `OSM`; leave onto a ramp or a surface arterial and confirm it falls back to `VISION`.
