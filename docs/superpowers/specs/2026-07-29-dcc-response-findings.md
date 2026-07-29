# DCC Response Study — Phase 1 Findings

**Date:** 2026-07-29
**Status:** Phase 1 complete; Phase 2 approach pending the gate decision
**Design:** `docs/superpowers/specs/2026-07-29-dcc-response-mapping-design.md`
**Tooling:** `plugins/bmw_e9x_e8x/tools/dcc_study/`
**Data:** 16 engaged routes / 715 segments pulled from the C3 over the COD API
(newest first, 2026-07-29 back to 2026-07-05), 6.6 GB of rlogs.
**Result:** 7680 clean bursts kept, 890 dropped as contaminated (10.4 %).

---

## TL;DR

The study succeeded, but it invalidates the model the Phase 2 sketch was built on.

1. **Cadence (hold 40 Hz vs single 20 Hz) has no measurable effect** — not on
   achieved acceleration, and not on tick acceptance rate once burst duration is
   controlled for. Half of the current threshold logic buys nothing.
2. **The per-burst `peak_delta_a` numbers in `summary.txt` for `plus1`/`minus1`
   are indistinguishable from noise.** A no-command null test returns the same
   ±0.28 m/s² that those commands "produce". Only the ±5 commands clear the floor.
3. **DCC's achieved acceleration is a clean monotone function of the setpoint gap
   (`cruiseState.speed − vEgo`), not of the command that produced it.**
   corr = **+0.746** over 2.46 M engaged samples; +0.70…+0.80 within every speed
   band. Command and cadence only matter through how fast they slew the setpoint.

Recommendation: **do not build approach B as specified.** Build **B′** — an
inverse map on *setpoint gap* (× vEgo), which the existing data already supports —
or fall back to A. Details in [Recommendation](#recommendation).

---

## 1. Coverage — verdict: PASS for `plus1`/`minus1`, FAIL for `plus5`/`minus5`

Gate criterion: *enough bursts per command×cadence across ≥3 speed bins.*

| command × cadence | bursts | speed bins with n ≥ 20 | verdict |
|---|---:|---:|---|
| `plus1` hold    | 1861 | 9 (30–120 km/h) | pass |
| `plus1` single  | 2748 | 9 (30–120 km/h) | pass |
| `minus1` hold   | 1174 | 9 (40–130 km/h) | pass |
| `minus1` single | 1535 | 9 (40–130 km/h) | pass |
| `minus5` hold   |  302 | 7 (40–120 km/h) | marginal |
| `plus5` hold    |   60 | 2 (30–50 km/h)  | **fail** |
| `plus5` single  |    0 | — | **structurally unreachable** |
| `minus5` single |    0 | — | **structurally unreachable** |

**The two empty rows are not a data gap — they are impossible by construction.**
In `carcontroller.py` the step-5 threshold is strictly above the hold threshold
(`ACCEL_STEP5_THRESHOLD` 0.6 > `ACCEL_HOLD_THRESHOLD` 0.2; `DECEL_STEP5_THRESHOLD`
0.9 > `DECEL_HOLD_THRESHOLD` 0.4), so any accel large enough to select ±5 has
already selected the hold cadence. Only **6 of 8** command×cadence combinations
can ever be emitted by the current controller, and observational data can never
cover the other two. Characterising `plus5`/`minus5` at single cadence requires
either targeted drives with a modified controller or accepting that they stay
unused.

`plus5` hold is also thin (60 bursts, none above 100 km/h) because the planner
rarely demands ≥ +0.6 m/s² in this driving.

## 2. Speed dependence — verdict: real but second-order

The `summary.txt` binned medians look **flat** for `plus1`/`minus1` (e.g. `plus1`
hold ranges +0.258…+0.414 with no trend across 20–130 km/h). That flatness is an
artefact — see §3; those numbers are noise, and noise does not depend on speed.

Measured properly (median `aEgo` at a *fixed setpoint gap*, flat pitch only,
2.46 M samples), speed dependence is **monotone and modest**:

| setpoint gap | 18–36 | 36–50 | 50–65 | 65–79 | 79–94 | 94–115 | 115–144 km/h |
|---|---:|---:|---:|---:|---:|---:|---:|
| +0.3…+1.0 m/s | +0.133 | +0.114 | +0.116 | +0.071 | +0.041 | +0.023 | +0.004 |
| −0.3…+0.3 m/s | −0.001 | −0.009 | −0.071 | −0.067 | −0.054 | −0.179 | −0.217 |
| +1.0…+2.0 m/s |   —    | +0.370 | +0.365 | +0.328 | +0.304 | +0.263 | +0.208 |

Consistent **−0.13 to −0.22 m/s² offset from ~25 → ~130 km/h** (drag / reduced
torque reserve), monotone in every row. But compare the span: speed moves the
answer by ~0.2 m/s², the setpoint gap moves it by ~1.5 m/s² (§4). **Speed is a
correction term; setpoint gap is the independent variable.**

## 3. The per-burst measurement is at the noise floor (why check 1 failed)

The brief's first sanity check — *hold-cadence bursts should show larger |Δaccel|
than single at the same speed* — **fails**, and investigating it is the most
important result of this study.

**Burst isolation is poor.** The controller re-arms almost continuously: median
gap to the neighbouring burst is **0.89 s**, 57.9 % of bursts have the *next*
burst starting inside their own 1.5 s response tail, and only **8.1 %** are
isolated by ≥3 s on both sides. The 0.5 s "baseline" window is usually mid-response
to the previous burst. Bursts are also short: median duration **0.13 s**, median
**4–6 frames**, and only 9.2 % reach the 1.0 s needed for a steady-state estimate —
so 91 % of rows fall back to `peak_delta_a`.

**`peak_delta_a` is a peak-pick over a ~1.6 s window of noisy `aEgo`, which
manufactures a response from nothing.** Null test — same estimator applied to
1.6 s windows with cruise engaged, no gas/brake, and **no stalk command within
4 s** (454 windows, 150 segments):

| | median | IQR |
|---|---:|---|
| NULL peak-up (mimics `plus*`)   | **+0.272** | +0.183 … +0.387 |
| NULL peak-down (mimics `minus*`)| **−0.278** | −0.421 … −0.189 |
| measured `plus1` (either cadence) | +0.287…+0.289 | +0.203 … +0.395 |
| measured `minus1` (either cadence)| −0.295…−0.312 | −0.466 … −0.207 |

The ±1 "responses" **are the null**. Only ±5 clears it: `plus5` +0.759,
`minus5` −0.814, ~2.8× the floor. Within-window `aEgo` σ is 0.119 m/s².

**Re-measured with an unbiased estimator** — mean over [t₀+0.3, t₀+1.6] minus the
0.5 s pre-burst mean, restricted to isolated bursts (≥2.5 s clear before, ≥2.0 s
after). Null for the same estimator: median −0.001, IQR ±0.035, σ 0.073 — unbiased.

| command × cadence | n | median Δaccel | IQR | z vs null |
|---|---:|---:|---|---:|
| `plus1` hold    | 136 | **+0.039** | −0.005 … +0.086 | +6.3 |
| `plus1` single  | 450 | **+0.058** | +0.020 … +0.107 | +17.3 |
| `minus1` hold   |  44 | **−0.145** | −0.240 … −0.004 | −13.0 |
| `minus1` single | 198 | **−0.147** | −0.202 … −0.083 | −28.1 |
| `minus5` hold   |  44 | **−0.427** | −0.528 … −0.324 | −38.7 |
| `plus5` hold    |   7 | **+0.202** | +0.190 … +0.303 | +7.4 |

Two conclusions:

- **No cadence effect.** `plus1` single ≥ `plus1` hold; `minus1` hold ≈ single.
  Same on the all-flat-pitch set and on the ≥3 s isolated set (`plus1` +0.331
  single vs +0.268 hold; `minus1` −0.465 single vs −0.364 hold). The sanity check
  did not fail because of a bug — **the effect it assumed does not exist.**
- **The hand-calibration anchors in the design are far too large for a single
  burst.** Anchors were +0.4 / +1.2 / −0.6 / −1.2; a real isolated burst (median
  **1** accepted tick) delivers +0.04 / +0.20 / −0.15 / −0.43. The anchors are only
  reachable by *sustained* multi-tick trains. Caveat: isolated bursts are
  selection-biased toward small demands (the planner asks for one tick when it
  wants little), so treat these as lower bounds, not as the map.

## 4. Setpoint gap is the actual control variable — and it saturates asymmetrically

Median `aEgo` vs `cruiseState.speed − vEgo`, all engaged samples, no gas/brake,
vEgo > 3 m/s (n = 2 463 548):

| gap (m/s) | −6…−3 | −3…−2 | −2…−1.5 | −1.5…−1 | −1…−0.5 | −0.5…−0.2 | −0.2…+0.2 |
|---|---:|---:|---:|---:|---:|---:|---:|
| median aEgo | **−1.026** | −0.957 | −0.765 | −0.522 | −0.337 | −0.226 | −0.082 |

| gap (m/s) | +0.2…+0.5 | +0.5…+1 | +1…+1.5 | +1.5…+2 | +2…+3 | +3…+6 |
|---|---:|---:|---:|---:|---:|---:|
| median aEgo | +0.004 | +0.090 | +0.280 | +0.440 | **+0.519** | **+0.504** |

**corr(gap, aEgo) = +0.746** overall; +0.443 (11–36 km/h), +0.796 (36–61),
+0.751 (61–90), +0.705 (90–144).

**Saturation (the brief's third observation):** the acceleration side plateaus at
**≈ +0.50 m/s²** — gaps of +2…+3 and +3…+6 m/s return the same value. The
deceleration side does not plateau in the same range, reaching **−1.03 m/s²** at
gaps beyond −3 m/s. DCC's authority is asymmetric and capped: **you cannot request
more than about +0.5 m/s² from it, no matter what you send.** Any Phase 2 map must
encode that ceiling rather than escalating into it.

Caveat: only 1.2 % of samples sit at |gap| > 2 m/s, so the plateau is suggestive,
not conclusive. Confirming the exact accel ceiling is the one thing a targeted
calibration drive would genuinely add.

**Operating range.** The gap distribution is narrow — p5…p95 = **−0.82 … +1.35 m/s**,
median +0.47. The controller keeps the setpoint glued to vEgo. So the map is
well-populated only over roughly [−1.5, +2.0] m/s of gap; outside that it is
extrapolation.

## 5. Acceptance — verdict: PASS, counter-overwrite validated in the field

Fraction of bursts that moved `cruiseState.speed` by ≥1 tick in the commanded
direction, and never in the wrong direction:

| command × cadence | n | frac > 0 ticks | frac < 0 | mean ticks | median |
|---|---:|---:|---:|---:|---:|
| `plus1` hold    | 1861 | 0.989 | 0.000 | 1.56 | 1 |
| `plus1` single  | 2748 | 0.992 | 0.000 | 1.51 | 1 |
| `minus1` hold   | 1174 | 0.993 | 0.000 | 3.45 | 1 |
| `minus1` single | 1535 | 0.991 | 0.000 | 1.62 | 1 |
| `minus5` hold   |  302 | 0.977 | 0.000 | 1.77 | 2 |
| `plus5` hold    |   60 | 0.900 | 0.000 | 1.55 | 2 |

**97.7–99.3 % of bursts land** (90 % for the 60-sample `plus5` row), and no burst
ever moved the setpoint backwards. The counter-overwrite mechanism works.

The apparent `minus1` hold advantage (3.45 vs 1.62 mean ticks) is a **duration-mix
artefact**, not a cadence effect. Controlled for duration the two are the same:

| duration | `minus1` hold ticks/s | `minus1` single ticks/s | `plus1` hold | `plus1` single |
|---|---:|---:|---:|---:|
| 0.5–1.0 s | 4.2 | 4.3 | 2.9 | 3.0 |
| 1.0–2.0 s | 3.5 | 3.4 | 2.9 | 2.9 |
| ≥2.0 s    | 3.1 | 2.8 | 2.4 | 2.3 |

DCC's internal auto-repeat runs at **~2.5–4.5 ticks/s regardless of whether we
transmit at 20 Hz or 40 Hz.** Setpoint slew is therefore ≈ 3 ticks/s × step, i.e.
**~3 km/h/s** for ±1 and **~15 km/h/s** for ±5 — which is exactly why ±5 shows a
larger response: bigger setpoint jump → bigger gap → bigger accel. Command choice
is a *slew-rate* knob; cadence is not a knob at all.

## 6. Pitch filter — verdict: adequate, arguably over-strict

Two things to record.

**The channel needed a fix.** `livePose.orientationNED.y` is the **device** pitch
in NED — it carries the windshield-mount tilt and calibration offset, **−0.113 rad
(−6.47°)** on this car. The spec's absolute `|pitch| < 0.017 rad` test therefore
rejected **100 %** of bursts and the first run produced an empty medians table and
empty residual plots. `report.py` now estimates the offset as the sample median and
filters on the deviation from it (commit `786ec6f`); 5137 of 7680 bursts (66.9 %)
survive. `carControl.orientationNED[1]` (calibrated road pitch) still carries a
~−2.8° residual bias, so it would not have avoided the problem either.

**Grade is barely a confounder here.** Median `aEgo` at a fixed setpoint gap, by
de-biased pitch bin:

| de-biased pitch (rad) | −0.08…−0.04 | −0.04…−0.017 | −0.017…+0.017 | +0.017…+0.04 | +0.04…+0.08 |
|---|---:|---:|---:|---:|---:|
| gap +0.3…+1.0 | +0.167 | +0.040 | +0.052…+0.060 | +0.049 | +0.051 |
| gap −1.0…−0.3 | −0.191 | −0.268 | −0.294 | −0.306 | −0.326 |

Flat within ±0.04 rad (±2.3°); only the steep-downhill extreme bin shifts
materially. This is expected — **DCC is itself a closed-loop speed controller, so
it absorbs grade**; we are measuring its output, not an open-loop plant. The
±0.017 rad filter is sufficient and could be relaxed to ±0.03–0.04 rad to recover
~18 % more samples if coverage ever becomes the binding constraint.

---

## Recommendation

**Approach B as specified in the design should not be built.** Its premise —
"expected-accel interp tables per command×cadence keyed on vEgo" — is
contradicted on two of three axes by the data:

- *cadence* is not a factor at all (§3, §5) → the tables would have 6 reachable
  entries of which half are duplicates;
- *vEgo* is a −0.2 m/s² correction, not the primary key (§2);
- what actually determines accel is the **setpoint gap** (§4, corr +0.746).

**Recommended: B′ — inverse map on setpoint gap, with a vEgo correction.**

- Model `a_expected = f(setpoint_gap, vEgo)`, fitted from the tables in §2 and §4.
  The data to fit it is already on disk (2.46 M samples, no new drives needed).
- Selection inverts it: given `aTarget`, solve for the required gap, compare to the
  current `cruiseState.speed − vEgo`, and emit ticks to close the difference —
  `plus5`/`minus5` when the required setpoint move is large, `plus1`/`minus1` when
  it is small. This is a *setpoint* controller, which is what DCC actually exposes,
  and it subsumes the ±1 vs ±5 choice naturally.
- **Drop the cadence branch.** Pick one cadence (single 20 Hz is sufficient — it
  accepts ticks at the same rate and halves bus load) and delete
  `ACCEL_HOLD_THRESHOLD` / `DECEL_HOLD_THRESHOLD`. This alone removes two hand-tuned
  constants that provably do nothing.
- **Encode the ceiling:** clamp requested accel to ~+0.5 m/s² (and note decel
  reaches ~−1.0). Above the clamp, escalating gap further does nothing except
  overshoot on the way back down.
- Keep hysteresis as sketched — the crossovers move to gap thresholds but the
  hunting risk is unchanged.

### The gap is already known — and that exposes what B′ is really for

`v_target` (`actuators.speed`) was not in the Phase-1 extraction, so this was
re-measured directly from 14 segments of route `2026-07-29--09-00-20`
(46 347 engaged clean samples):

| quantity | corr with `aEgo` |
|---|---:|
| actual gap (`cruiseState.speed − vEgo`) | **+0.826** |
| commanded gap (`v_target − vEgo`) | +0.699 |
| `aTarget` (`actuators.accel`) | +0.618 |

Both gaps are already available in `carcontroller.py` today with zero new
plumbing — `v_target` as `actuators.speed`, the actual setpoint as
`CS.out.cruiseState.speed`. The actual gap is the better predictor and is the
causal one, so the map should be keyed on it; the commanded gap is the useful
*bound* (see below).

**The two gaps nearly coincide, and that is the finding.**
median(`v_target − cruiseState.speed`) = **−0.048 m/s**, |difference| < 0.2 m/s
in 61 % of samples and < 0.5 m/s in 83 %; even while actively demanding
acceleration (`gap_cmd > 1.0`) the median offset is only +0.084 m/s. The
existing controller already slews the setpoint onto `v_target` and keeps it
there.

The consequence is uncomfortable but clarifying: **today the achieved
acceleration is essentially determined by `v_target` alone.** Because the
setpoint converges on `v_target` within ~0.05 m/s, the gap — and therefore the
accel — is fixed by the speed request, and the `aTarget` threshold ladder only
changes how fast that convergence happens. That is why `aTarget` correlates
worst of the three with what the car actually does. openpilot currently has
very little authority over the longitudinal *profile*, only over the
destination speed.

So B′ is not "compute a setpoint" — the setpoint already goes to the right
place. **B′ is deliberately holding the setpoint _short_ of `v_target` when the
full gap would produce more acceleration than `aTarget` wants:**

    gap_required = f⁻¹(aTarget, vEgo)
    setpoint_cmd = min(vEgo + gap_required, v_target)     # never above v_target

The `min` is the whole safety story. B′ only ever commands a setpoint at or
below what today's code would reach, so it is strictly more conservative than
the current behaviour, and it degrades to exactly today's behaviour whenever
`aTarget` is large. It also inherits the existing `setpoint_error > 0` guard
rather than fighting it.

### Can the stalk actually track `aTarget`? Yes — and the rate limit picks the command

The stalk moves the setpoint in 1 or 5 km/h ticks at DCC's own ~2.5–4.5 ticks/s,
so there is a hard ceiling on how fast `aEgo` can be steered. Measured plant
gain over 666 k samples in the responsive band (gap 0.2…2.0 m/s):
**d(aEgo)/d(gap) ≈ 0.313 m/s² per m/s of gap.** Converting tick rate to
acceleration rate, against how fast the planner's `aTarget` actually moves
(157 segments, 49 k samples):

| | d(aEgo)/dt ceiling |
|---|---:|
| `±1` @ 2.5–4.5 ticks/s | **0.22 – 0.39 m/s³** |
| `±5` @ 2.5–4.5 ticks/s | **1.09 – 1.96 m/s³** |

| `aTarget` rate of change | p50 | p90 | p95 | p99 |
|---|---:|---:|---:|---:|
| \|d(aTarget)/dt\| (m/s³) | 0.075 | 0.256 | 0.353 | 0.778 |

**`±1` alone covers the planner's demand up to about the 90th percentile; `±5`
covers beyond the 99th.** Tracking is feasible, and this is the honest,
non-arbitrary reason to keep both commands: they are *slew-rate* selections, not
acceleration selections.

### The resulting control law

Everything above collapses into one quantity — the setpoint error in km/h, which
is literally the number of ticks still owed:

    gap_required   = f⁻¹(aTarget, vEgo)                      # invert the steady-state map
    setpoint_des   = min(vEgo + gap_required, v_target)      # never above v_target
    err_kph        = (setpoint_des − cruiseState.speed) * 3.6

    |err_kph| ≥ 5  → plus5 / minus5      (need to move ≥ 5 km/h)
    |err_kph| ≥ 1  → plus1 / minus1
    otherwise      → no command (deadband)

This replaces **both** hand-tuned ladders. `ACCEL_STEP5_THRESHOLD` /
`DECEL_STEP5_THRESHOLD` become "is at least one 5 km/h tick owed", which is not a
tuned constant but an arithmetic fact. `ACCEL_HOLD_THRESHOLD` /
`DECEL_HOLD_THRESHOLD` disappear entirely, since cadence is inert (§3, §5).

**Known remaining unknown:** `f` is a *steady-state* map. DCC's transient lag
between a gap change and the resulting acceleration was not measured, and it adds
phase that this open-loop inversion does not compensate. The margin is
comfortable — `aTarget` moves at 0.075 m/s³ median against a `±1` ceiling of
0.22–0.39 — but a first on-car run should watch for lag-driven overshoot before
the deadband is tightened. Note also that measured `aEgo` is far noisier than
`aTarget` (p50 rate 0.72 vs 0.075 m/s³), so `aEgo` must not be used as a
tracking error signal; that would be approach C, which the data does not support.

**Two constraints B′ must respect** (added at controller review, not from the
run itself):

- *B′ does not stack a second controller.* The rejected approach C wrapped
  feedback on the acceleration error around DCC. B′ instead computes a desired
  *setpoint* from a static inverse map — a reference, which is the interface DCC
  actually exposes. The only closed loop remains DCC's own. Keep it that way:
  the moment the map is adapted online from measured `aEgo`, B′ becomes C.
- *The setpoint is the disengagement fallback, and today's guard caps it.*
  `carcontroller.py` only raises the setpoint while `setpoint_error =
  v_target − cruiseState.speed > 0`, so the setpoint never exceeds `v_target`.
  Under B′ a meaningful accel needs a gap of ~1.5–2 m/s, which means the
  achievable acceleration is structurally bounded by how far `v_target` sits
  above `vEgo` — you cannot request hard acceleration once you are near target
  speed. That is desirable behaviour and the guard must stay: raising the
  setpoint above `v_target` to force accel would leave the car targeting an
  unsafe speed if openpilot disengaged mid-manoeuvre.

**Approach A (recalibrated constants) is the fallback**, and is *not* a bad
outcome: since cadence is inert and the map is dominated by one variable, A reduces
to two constants (±1 vs ±5 crossover, up and down). The §3 isolated-burst numbers
give the honest per-burst magnitudes to recalibrate against, but note they are
lower bounds (selection bias toward small demands).

**Targeted calibration drives — recommended but narrow.** Only two questions need
new data, and neither blocks starting B′:

1. The **acceleration ceiling** (§4): sustained large positive gaps to confirm the
   +0.5 m/s² plateau. 1.2 % of existing samples is thin.
2. **`plus5`/`minus5` at single cadence** — only if the gate decides to keep a
   cadence branch at all. If cadence is dropped as recommended, this question
   disappears.

A third, cheaper improvement needs no drives: make the controller **hold each burst
long enough to be measurable** (median 0.13 s / 1 tick is below the observability
floor). Longer, less frequent bursts would improve both control authority and every
future measurement.

---

## Tooling changes made during this run

Two commits, both required to get the pipeline to run against the real device:

- `f2c35d6` **dcc_study: adapt fetch to actual COD response shape** — COD binds
  **port 80** on the device, not the 8082 in `API.md`. The
  `/v1/route/{name}/download` endpoint builds the entire route tar.gz in device RAM
  (~460 MB for a 48-segment route) and names members by opaque `local_id`, so the
  cache check never matched; replaced with per-segment `/connectdata/.../rlog.zst`
  GETs listed from `/v1/route/{name}/files`. Dropped the per-route metadata GET
  (the listing already carries `engagement_pct`, and the metadata endpoint can
  trigger enrichment work on the device). Added `--max-routes`.
- `786ec6f` **dcc_study: de-bias the pitch channel and drop NaN bursts from bin
  medians** — see §6; also, a burst at a segment edge has no baseline window and
  its NaN `delta_a` was poisoning whole bin medians via `np.median`.

Test suite: 32 passed.

---

## Appendix — `data/report/summary.txt`, verbatim

```
DCC response study summary
========================================

7680 clean bursts; pitch-channel bias -0.1129 rad (-6.47 deg), 5137 bursts within +/-0.017 rad of it (flat-pitch set used for medians).

Coverage (bursts per cmd x cadence x 10 km/h speed bin):
cmd     cadence      20     30     40     50     60     70     80     90    100    110    120
plus1   hold          3    138    216    412    516    286    127     67     49     41      6
plus1   single        0     50     87    219    569    693    353    304    261    192     20
plus5   hold          0     10     21     17      9      1      1      1      0      0      0
plus5   single        0      0      0      0      0      0      0      0      0      0      0
minus1  hold          0     19    100    166    267    262    181     65     62     32     20
minus1  single        0     18     96    179    377    382    193     87    108     81     14
minus5  hold          0      0     20     37     57     76     57     20     15     16      4
minus5  single        0      0      0      0      0      0      0      0      0      0      0

Median response, flat pitch (m/s²):
  minus1  hold      30-40 km/h: -0.331
  minus1  hold      40-50 km/h: -0.356
  minus1  hold      50-60 km/h: -0.268
  minus1  hold      60-70 km/h: -0.295
  minus1  hold      70-80 km/h: -0.281
  minus1  hold      80-90 km/h: -0.368
  minus1  hold      90-100 km/h: -0.244
  minus1  hold     100-110 km/h: -0.199
  minus1  hold     110-120 km/h: -0.205
  minus1  hold     120-130 km/h: -0.380
  minus1  single    30-40 km/h: -0.307
  minus1  single    40-50 km/h: -0.390
  minus1  single    50-60 km/h: -0.324
  minus1  single    60-70 km/h: -0.305
  minus1  single    70-80 km/h: -0.303
  minus1  single    80-90 km/h: -0.306
  minus1  single    90-100 km/h: -0.268
  minus1  single   100-110 km/h: -0.339
  minus1  single   110-120 km/h: -0.402
  minus1  single   120-130 km/h: -0.190
  minus5  hold      40-50 km/h: -0.654
  minus5  hold      50-60 km/h: -0.781
  minus5  hold      60-70 km/h: -0.816
  minus5  hold      70-80 km/h: -0.746
  minus5  hold      80-90 km/h: -0.852
  minus5  hold      90-100 km/h: -0.911
  minus5  hold     100-110 km/h: -0.928
  minus5  hold     110-120 km/h: -0.683
  minus5  hold     120-130 km/h: -0.847
  plus1   hold      20-30 km/h: +0.414
  plus1   hold      30-40 km/h: +0.293
  plus1   hold      40-50 km/h: +0.307
  plus1   hold      50-60 km/h: +0.292
  plus1   hold      60-70 km/h: +0.291
  plus1   hold      70-80 km/h: +0.258
  plus1   hold      80-90 km/h: +0.274
  plus1   hold      90-100 km/h: +0.292
  plus1   hold     100-110 km/h: +0.338
  plus1   hold     110-120 km/h: +0.284
  plus1   hold     120-130 km/h: +0.369
  plus1   single    30-40 km/h: +0.287
  plus1   single    40-50 km/h: +0.267
  plus1   single    50-60 km/h: +0.294
  plus1   single    60-70 km/h: +0.292
  plus1   single    70-80 km/h: +0.272
  plus1   single    80-90 km/h: +0.293
  plus1   single    90-100 km/h: +0.266
  plus1   single   100-110 km/h: +0.306
  plus1   single   110-120 km/h: +0.301
  plus1   single   120-130 km/h: +0.433
  plus5   hold      30-40 km/h: +0.899
  plus5   hold      40-50 km/h: +0.841
  plus5   hold      50-60 km/h: +0.705
  plus5   hold      60-70 km/h: +0.597
  plus5   hold      70-80 km/h: +0.586
  plus5   hold      90-100 km/h: +1.216

Acceptance (mean accepted ticks per burst):
  minus1  hold    :   3.4 ticks over 1174 bursts
  minus1  single  :   1.6 ticks over 1535 bursts
  minus5  hold    :   1.8 ticks over 302 bursts
  plus1   hold    :   1.6 ticks over 1861 bursts
  plus1   single  :   1.5 ticks over 2748 bursts
  plus5   hold    :   1.6 ticks over 60 bursts
```

**Read the `Median response` block above with §3 in mind:** the `plus1`/`minus1`
entries are at the noise floor and should not be used as calibration values.

Plots: `plugins/bmw_e9x_e8x/tools/dcc_study/data/report/{response_vs_speed,
residual_vs_setpoint_gap, residual_vs_pitch}.png`

---

## Routes analysed

| route | bursts | | route | bursts |
|---|---:|---|---|---:|
| 2026-07-29--09-00-20 | 487 | | 2026-07-23--08-41-25 | 509 |
| 2026-07-28--09-33-47 | 1026 | | 2026-07-22--16-59-00 | 274 |
| 2026-07-27--17-03-31 | 432 | | 2026-07-21--08-36-00 | 102 |
| 2026-07-27--09-23-35 | 536 | | 2026-07-09--13-39-57 | 967 |
| 2026-07-26--14-02-30 | 607 | | 2026-07-09--09-23-43 | 1257 |
| 2026-07-26--11-01-07 | 662 | | 2026-07-05--10-45-09 | 227 |
| 2026-07-25--13-09-34 | 21 | | | |
| 2026-07-25--10-35-14 | 225 | | | |
| 2026-07-25--09-04-44 | 191 | | | |
| 2026-07-24--18-20-04 | 157 | | | |

Every route on the device with `engagement_pct ≥ 1` was used; older routes report
no engagement metric and were skipped. **Caveat on that skip:** COD computes
`engagement_pct` only opportunistically (when a route's events are already
cached from being opened in the Connect UI), and nothing backfills it, so a
missing metric means "never computed" — which is *not* the same as "not
engaged". Some skipped older routes may have contained usable bursts. This does
not threaten the conclusions (the newest-13-only re-analysis in §Routes gives
the same curve), but the sample is "routes with a cached engagement metric",
not "all engaged routes". `fetch_routes.py` currently logs both cases
identically; distinguishing them is a known follow-up.

The three pre-2026-07-21 routes (2026-07-09 ×2 and 2026-07-05, 2451 bursts / 34 %
of the sample) predate three weeks of controller changes. Re-running the §4
gap→accel analysis on the 538 newer segments only (1.63 M samples) gives
**corr +0.754** (vs +0.746) and the same curve within ≤0.04 m/s² per bin:

| gap (m/s) | −3…−2 | −1…−0.5 | −0.2…+0.2 | +0.5…+1 | +1…+1.5 | +2…+3 | +3…+6 |
|---|---:|---:|---:|---:|---:|---:|---:|
| all 16 routes  | −0.957 | −0.337 | −0.082 | +0.090 | +0.280 | +0.519 | +0.504 |
| newest 13 only | −0.944 | −0.328 | −0.055 | +0.127 | +0.297 | +0.468 | +0.492 |

Same shape, same plateau, same conclusions. The newer-only fit runs ~0.03 m/s²
higher on the acceleration side — use the newest-13 column if Phase 2 numbers are
fixed from this data.

---

## Independent verification

The three load-bearing claims were recomputed from the extracted channels by a
second script that does not import the study's own modules
(`corr(gap, aEgo)`, the null-floor test, and the cadence comparison):

| claim | study | independent recompute |
|---|---|---|
| corr(gap, aEgo), 2.46 M samples | +0.746 | **+0.746** |
| accel plateau, gap +2…3 / +3…6 | +0.519 / +0.504 | **+0.519 / +0.504** |
| decel at gap −6…−3 | −1.026 | **−1.026** |
| null peak-up / peak-down | +0.272 / −0.278 | **+0.247 / −0.257** |
| null unbiased median / σ | −0.001 / 0.073 | **+0.001 / 0.073** |
| plus1 single vs hold | +0.058 / +0.039 | **+0.058 / +0.040** |
| minus1 single vs hold | −0.147 / −0.145 | **−0.151 / −0.142** |

The null-floor medians differ by ~0.025 m/s² because the two scripts sample
command-free windows on different grids; the conclusion is unchanged — the ±1
peak-pick "responses" (+0.29 / −0.30) sit inside the null distribution.

Burst statistics independently confirmed: median duration 0.130 s, median 5
frames, median 1 accepted tick, only 9.2 % of bursts reach the 1.0 s needed for
a steady-state estimate, and 61.6 % have their successor begin within 1.5 s.
