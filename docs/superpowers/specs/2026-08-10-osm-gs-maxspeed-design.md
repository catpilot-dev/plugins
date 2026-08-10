# OSM maxspeed restricted to G/S expressways (CN) + on-road source indicator

**Date:** 2026-08-10
**Status:** Approved, pending implementation
**Supersedes (partially):** `2026-08-07-osm-data-integration-design.md` — narrows the CN
behaviour of the toggle shipped there; leaves non-CN behaviour untouched.

## Problem

The OSM Data Integration feature shipped 2026-08-07 already implements "use OSM
maxspeed when available, fall back to inference when not": `_osm_base_active()`
(`speedlimitd.py:886`) gates the arbitration at `speedlimitd.py:1412`, and a stale or
absent maxspeed falls through to the G/S table or lane-count inference. That part
needs no new work.

What it lacks is *selectivity*. In China the risk is not absent maxspeed tags — those
degrade gracefully to vision — but tags that are **present and wrong**. The
2026-08-07 OSM audit found two systematic map-matching failure modes in exactly the
region we drive:

1. **Viaduct → ground road.** An elevated deck's 80 written onto the surface arterial
   beneath it (华夏中路, 华夏西路, 金海路, 申江路). Reading it back commands 80 on a
   surface street.
2. **Ramp sign → main road.** A 40 km/h ramp sign written onto a 高速/高架 mainline;
   8 certain-wrong cases. Reading it back commands 40 in a fast lane.

No safety cap catches either: both are wrong numbers on straight roads. This is why
the CN region default was set OFF and gated on telemetry agreement.

The vision team has since confirmed OSM *does* carry usable maxspeed on G/S
expressways. So the fix is to trust the posted number precisely where it has been
confirmed good, and nowhere else.

## Scope decisions

| Decision | Choice |
| --- | --- |
| Region scope | CN only. Non-CN behaviour is unchanged — OSM maxspeed stays the base on any way, default ON. |
| Ramp-sign guard | Reject a posted maxspeed below 60 km/h on a G/S way. |
| CN default | Stays OFF (`OsmDataIntegration = '0'`). Opt-in via the Driving panel. |

The G/S ref grammar (`^[GS](\d{1,2}|\d{4})$`) is China-specific and would not match
`A 9`, `M1`, or `I-95`. Applying it globally would disable the feature exactly where
OSM is reliable, so the gate is conditioned on the detected country.

## Terminology

"Vision inferred" in this document means the **lane-count / road-type table**
(`lane_count_limit()` at `speedlimitd.py:687`, `infer_speed_from_road_type()` at
`:515`). It is *not* YOLO sign reading: `self.yolo_speed` is initialised to 0 at
`speedlimitd.py:840` and never assigned, so the tier-1 branch at `:1454` is
unreachable (already noted in `DESIGN.md:46`). Sign vision remains a separate future
input.

## Design

### Gate placement

The gate is added as a condition of the existing `_osm_base_active()` predicate rather
than by restructuring the `if/elif` at `:1412`. That keeps a single predicate feeding
all four consumers — arbitration (`:1413`), the ≤2-lane vision-cap bypass (`:1435`),
display rounding (`:1470`), and telemetry — so they cannot drift apart.

`gs_mode` is included as a condition, which makes the OSM maxspeed path a strict
refinement of the G/S state machine in CN: it inherits every existing release guard
(`lane_count_stable <= 2`, the margin rule `_eval_gs_margin_release`, `gs_lane_drop`,
`GS_RELEASE_CONT_S`, the 30 s sticky) at no extra cost.

```python
GS_OSM_MIN_KPH = 60   # a G/S mainline is never posted below this; lower = ramp sign

def _osm_maxspeed_trusted(self, gs_mode: bool) -> bool:
    """Trust a posted OSM maxspeed only on a confirmed G/S expressway.

    Applies in CN and whenever the country is not yet known — an unknown
    country must fail safe, not fall through to the permissive path.
    """
    if self.country and self.country != 'cn':
        return True                              # non-CN: unchanged, as shipped
    return (gs_mode
            and is_gs_expressway_ref(self.last_way_ref)
            and self.last_osm_speed_kph >= GS_OSM_MIN_KPH)
```

**The unknown-country case is load-bearing, not defensive boilerplate.** `self.country`
is `''` until the first GPS fix resolves it (`:1102`). A user who has opted in inside
China starts the next drive with `osm_integration_enabled` already `True` (read from
the param in `__init__` at `:864`) but no country yet — so a plain `country != 'cn'`
test would open the gate on *any* way, including the mis-attributed surface arterials,
for the seconds until the fix lands. Treating unknown as strict closes that window.
`self.country` is therefore typed `str` and defaults to `''`, not `None`.

`_osm_base_active(now, gs_mode)` gains `and self._osm_maxspeed_trusted(gs_mode)`.
`gs_mode` is already computed at `:1406`, before the call site at `:1412`.

### Why the G/S *ref*, not `hwtype == 'motorway'`

The existing `is_gs_now` test at `:1355` accepts either a G/S ref or
`last_osm_hwtype == 'motorway'`. This gate deliberately requires the **ref**. Urban
elevated roads (高架路) are motorway-tagged and are precisely the viaduct→ground-road
case; none of the audited viaduct errors (华夏中路, 华夏西路, 金海路, 申江路,
龙东高架路, 罗山高架路) carry a G/S ref. The grammar's existing 3-digit exclusion
additionally keeps ordinary surface guodao/shengdao (G312, S203) out.

### Fallback timing

When a tile query returns no match, `:1021` clears `last_way_ref` but *holds*
`last_osm_speed_kph` for the 10 s TTL. Under this gate the cleared ref closes the door
immediately, so the limit drops to the G/S table value (itself 30 s sticky) instead of
coasting on a posted number belonging to a way we are no longer matched to. This is a
deliberate behaviour change and is more conservative than today.

### Supporting rename

`self._osm_default_country` (`:785`, assigned `:1102`) already holds the
GPS-detected country code but is named as if it existed only for default resolution.
Rename to `self.country` and have `_resolve_osm_default()` read it. Four call sites.

## Telemetry

The revisit gate for the CN default is "does `osmSpeedLimit` agree with vision on real
routes". Two fields make the new gate's behaviour directly measurable rather than
inferred:

- `osmTrusted: bool` — did the gate open this tick.
- `osmRejectReason: str` — first failing condition, evaluated in this fixed order so
  the value is deterministic:

  | Order | Reason | Condition |
  | --- | --- | --- |
  | 1 | `'disabled'` | `osm_integration_enabled` is False |
  | 2 | `'no_data'` | `last_osm_speed_kph` below the existing 30 km/h sanity floor (includes "never seen") |
  | 3 | `'stale'` | outside the 10 s freshness window |
  | 4 | `'not_gs'` | CN/unknown country, and not a confirmed G/S expressway — covers both a non-G/S `last_way_ref` and a released `gs_mode` |
  | 5 | `'low_value'` | CN/unknown country, on a G/S way, posted value below `GS_OSM_MIN_KPH` |

  `''` when the gate opened.

`osmSpeedLimit` continues to publish unconditionally and toggle-independently, so
rlogs still record what OSM claimed even when the gate rejected it. `'low_value'`
counts are the direct measure of how live the ramp-sign mis-attribution still is.

## On-road source indicator

`ui_overlay.py:113` already documents a source indicator (`"OSM" / "SIGN" / "~"`) that
was never implemented. Implement it with three labels: **`OSM`**, **`YOLO`**,
**`VISION`**.

### Label mapping

Read from the published `source` (`:1462`) and `inferenceMode` (`:1546`). `source`
alone is insufficient — OSM, `gs_osm`, and `lane_count` all publish `source == 2`.

| Condition | Label |
| --- | --- |
| `source == 1` (YOLO) | `YOLO` |
| `source == 2` and `inferenceMode == 'osm'` | `OSM` |
| `source == 2` and `inferenceMode` in `('gs_osm', 'lane_count')` | `VISION` |
| `source == 4` (curvature / reactive a_y safety cap) | `VISION` |

`gs_osm` maps to `VISION`, not `OSM`: in that mode OSM supplies only the road *class*,
while the number itself comes from the lane-count/road-type table. The distinction the
indicator draws is **posted vs inferred**, which is what makes it a useful on-road
check of this feature — on a G/S expressway, `OSM` means the posted tag passed the
gate, `VISION` means it was rejected or absent.

`YOLO` will not appear until sign vision is wired (`yolo_speed` is permanently 0); it
is included for forward compatibility.

### Rendering

Drawn inside `_draw_speed_limit_sign()`, so it inherits the existing
`ShowSpeedLimitSign` param gate and the `_speed_limit > 0` condition.

- Position: centred on the sign's `cx`, below the circle at `cy + r + SOURCE_LABEL_GAP`.
- Colour: white, full opacity — **not** faded with the sign's confirmed/unconfirmed
  alpha. The label is a diagnostic readout, and legibility over a bright camera feed
  matters more than matching the sign's suggestion/active semantics.
- Font: `_font_medium`, `SOURCE_LABEL_FONT_SIZE = 36`.
- New constants `SOURCE_LABEL_GAP` and `SOURCE_LABEL_FONT_SIZE` beside the existing
  layout constants.
- `_update_state()` gains `_inference_mode = _sl_data.get('inferenceMode', '')`.

If the label proves hard to read against bright road surfaces, add a 1 px dark outline
or a semi-transparent rounded backing — deferred until seen on road.

## Testing

Extend `TestOsmBaseSelection` (`tests/test_speedlimitd.py:2805`), one case per audited
real-world failure:

| Case | Expected |
| --- | --- |
| CN, G/S ref, fresh 100 | used — `inferenceMode == 'osm'` |
| CN, G/S ref, 40 (ramp-sign mis-attribution) | rejected → G/S table, `'low_value'` |
| CN, surface arterial (no ref), 80 (viaduct mis-attribution) | rejected → lane_count, `'not_gs'` |
| CN, G/S ref, `gs_mode` released on ≤2 lanes | rejected → lane_count, `'not_gs'` |
| CN, `G312` (3-digit surface guodao), 80 | rejected → lane_count, `'not_gs'` |
| country not yet resolved (`''`), non-G/S way, 80 | rejected → lane_count, `'not_gs'` |
| non-CN, any way, 80 | **used** — regression guard for US/EU |

`TestOsmBaseActive` (`:2765`) calls `_osm_base_active` directly and needs its call
sites updated for the new `gs_mode` argument.

Add UI tests to `tests/test_ui_overlay.py` for the label mapping table above,
including the `gs_osm → VISION` case and the `source == 4 → VISION` case.

## Out of scope

- Flipping the CN default (gated on telemetry from this change).
- Wiring YOLO / sign vision as a live input.
- Fixing the stale `cereal/slot0.capnp`, which declares an unused `osmMaxspeed @4`
  source id and a `Source` enum with no id 4 while the code emits 4 for safety caps.
  Pre-existing drift, unrelated to this change.
- Bearing-aware way matching and upcoming-limit look-ahead, which wait on mapd
  issue #88.
