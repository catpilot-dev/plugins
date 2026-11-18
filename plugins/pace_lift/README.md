# Pace Lift

In experimental mode, speeds the car up toward the planner's speed target when
the driving model is slower than it. Strength follows Driving Personality:
strongest on Aggressive, off on Relaxed.

Design: `docs/superpowers/specs/2026-11-19-pace-lift-design.md`.

## What it does

The planner hook `planner.e2e_accel` hands this plugin the e2e accel ask. The
plugin adds a small, rate-limited lift to it and returns the sum. Nothing else
changes: the MPC, the cruise candidate, speedlimitd's target and the BMW DCC
law are untouched, and the planner still takes the minimum of its candidates.

### The law (`lift.py`)

    gap    = v_cruise - v_ego                        (m/s)
    target = strength * ramp(gap, 1..4 m/s) * ramp(v_ego, 13.89..16.67 m/s)

`v_cruise` is the planner's speed target after the `planner.v_cruise` hook
(speedlimitd's cap, or the set speed). The speed ramp fades the lift in over
50-60 km/h.

Strength by personality: Aggressive 0.40, Standard 0.20, Relaxed 0.0 m/s2
(unknown personality 0.0).

### Gates

Hard resets (output = the ask, lift state = 0, same tick): `disabled`, `off`
(not engaged or not experimental), `gas`, `brake`, `standstill`, `stop` (model
shouldStop), `ask_decel` (ask at or below -0.10 m/s2), `no_model` (short or
non-finite inputs, including a_cruise).

Soft gates (target 0, the lift falls at up to 1.0 m/s3), first match reported:
`plan_decel` (model plan drops more than 1 m/s below v_ego within 6 s), `curve`
(planned lateral accel 1.5 m/s2 or more within 6 s), `lead_gap` (lead closer
than T_FOLLOW * v + 6 m + 0.5 s * v; T_FOLLOW 1.25 / 1.45 / 1.75 s), `lead_closing`
(lead closing faster than 0.5 m/s), `none_needed` (the ramps give 0). The lead
gates hold for 1.0 s after their condition clears.

### Bounds and rates

- The lift state is clamped to strength * ramp(ask, -0.10..0 m/s2) on every
  tick, so a model easing off cuts the lift at once.
- Rise at most 0.25 m/s3, fall at most 1.0 m/s3; dt comes from a monotonic
  clock, clamped 0 to 0.2 s.
- output = max(a_e2e, min(a_e2e + lift, a_cruise)): never lowers the ask,
  never pushes above the cruise candidate.

## Params

Plugin data dir (`PLUGINS_RUNTIME_DIR/pace_lift/data/`), re-read at most once a
second, so a change applies within 1 s, also while driving.

| Param | Type | Default | Meaning |
|---|---|---|---|
| `PaceLiftEnabled` | bool | true | `0` turns the lift off (Driving panel toggle "Pace Lift") |
| `PaceLiftAggressive` | string | `0.40` | strength on Aggressive, parsed as float, clamped 0 to 0.6 |
| `PaceLiftStandard` | string | `0.20` | strength on Standard, same parsing |

Bad or missing strength values fall back to the defaults. A param read that
fails for any reason other than a missing file counts as disabled.

## Telemetry

Every call publishes `paceLift` on the plugin bus (lands in `pluginBusLog`):
`aIn`, `aOut`, `lift`, `target`, `gap`, `strength`, `gate`. Publishing never
raises.

## Way back

- From the car: the Driving panel's Pace Lift toggle, or Relaxed personality.
- On the device: a `.disabled` marker in the plugin dir.
