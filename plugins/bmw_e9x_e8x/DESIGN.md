# BMW E9x/E8x — Design & Implementation

A **car-type plugin**: it supplies a full openpilot car interface (CarState,
CarController, CarParams), a custom lateral controller, a Panda safety model,
and the UI touches (temperature overlay, Driving-panel vehicle items) for the BMW
E8x/E9x family. It carries its own DBC files and does not fork opendbc.

The lateral controller has its own canonical reference —
[LATERAL_CONTROLLER.md](LATERAL_CONTROLLER.md). This document does **not**
repeat it; it covers everything around it.

## How the car interface registers

There is **no `car_helpers` registration hook** (that hook was removed in the
0.11 rebase). Instead `register.py` **monkey-patches opendbc at plugin load
time**: `_register_interfaces()` runs at module-exec time (during
`registry.load_plugin()`, before `card.py` starts fingerprinting) and mutates
opendbc's global dicts in place:

- `opendbc.car.car_helpers.interfaces[BMW_E82 / BMW_E90] = CarInterface`
- `opendbc.car.fingerprints._FINGERPRINTS` / `FW_VERSIONS`
- `opendbc.car.fw_versions` globals (`FW_QUERY_CONFIGS`, `VERSIONS`,
  `MODEL_TO_BRAND`, `REQUESTS`)
- `opendbc.car.values.PLATFORMS`
- `opendbc.car.interfaces.get_torque_params` is **wrapped** to fold in the
  BMW rows from `torque_params.toml`

Because `card.py` holds references to the same dict objects, BMW becomes a
visible car. Disable the plugin and none of this runs, so BMW is simply not
in the system. `device.health_check` (`on_health_check`) reports `ok` only if
`BMW_E90` is present in the `interfaces` dict.

## Fingerprinting — VIN only

Fingerprints are **empty** (`fingerprints.py`: `{}` for both platforms) with
dummy FW entries, which forces opendbc's fuzzy path. `values.py::match_fw_to_car_fuzzy`
reads VIN positions 4–6 (`vin[3:6]`) as the BMW model code and maps it:

- `UF1 / UF2 / UH1` → `BMW_E82`
- `PH1 / PH2 / PK1 / PK2 / PM1 / PM2 / PN1` → `BMW_E90` (covers E90/E91/E92/E93)

Within a platform, `interface.py::_get_params` further reads the live CAN
fingerprint to detect **which cruise ECU** is present and set feature flags
(`BmwFlags`):

| Flag | Meaning | Detected from |
|---|---|---|
| `NORMAL_CRUISE_CONTROL` | NCC ($540) | `0x200` on PT-CAN |
| `DYNAMIC_CRUISE_CONTROL` | DCC ($544) | `0x193` on PT-CAN, or `0x194` stalk on F-CAN, and no LDM |
| `ACTIVE_CRUISE_CONTROL_NO_ACC` / `_NO_LDM` | ACC-module variants | LDM (`0x0D5`) presence |
| `STEPPER_SERVO_CAN` | Ocelot servo present | `0x22F` on SERVO/AUX-CAN |

Transmission type (auto vs manual) and a couple of steer-ratio tweaks are also
inferred from message presence. DCC and NCC both set `minEnableSpeed = 30 km/h`.

## Data flow

```
Panda ──CAN──► carstate.py (CarState.update)
                 parses PT-CAN / F-CAN / AUX-CAN → structs.CarState
                 publishes bmw_temps (0.2 Hz) on the plugin bus
                 resume-button state machine → ButtonEvents / speedlimit toggle
                        │
                        ▼
        controlsd  ──► LatControl (custom, controls.lat_controller_init)
                       ──► actuators.torque
                   ──► Longitudinal planner ──► actuators.speed
                       (overwritten to long_plan.vTarget by post_actuators)
                        │
                        ▼
              carcontroller.py (CarController.update)
                 torque → Ocelot STEERING_COMMAND (rate-limited)
                 v_target/accel → DCC cruise-stalk 0x194 bursts
                        │
                        ▼
                 Panda safety (bmw.h) — torque & rate limits, TX allow-list
```

### CAN bus layout

| Bus | Names in `values.py::CanBus` | Traffic |
|---|---|---|
| 0 | `PT_CAN` | engine, brakes, speed, yaw, transmission, cruise status/stalk, temps |
| 1 | `SERVO_CAN` / `F_CAN` | Ocelot stepper servo (steering); DCC cruise stalk when DCC is present |
| 2 | `AUX_CAN` / `K_CAN` | alternative servo bus; logging |

`get_can_parsers` deliberately subscribes to **both** DCC and NCC cruise
messages (with a `nan` timeout) so a slow-to-wake ECU can't cause `canValid`
failures from an unsubscribed message accessed later.

### CarState notes (`carstate.py`)

- Speed/yaw/temps/cruise are parsed from the plugin's own `bmw_e9x_e8x.dbc`.
- **Temperatures** aren't in the stock CarState schema, so coolant/oil are
  published on plugin-bus topic `bmw_temps` at 0.2 Hz for the UI overlay.
- **Resume-button repurposing** is implemented here (not in `ui_overlay.py`):
  a rising/falling-edge state machine times the hold. Long press
  (≥ `RESUME_LONG_PRESS_FRAMES` = 49 frames ≈ 500 ms) emits a
  `gapAdjustCruise` button event; short press while engaged sends
  `{'action': 'toggle_confirm'}` on plugin-bus topic `speedlimit_cmd_car`
  (and drains `speedLimitState`); short press while disengaged emits
  `resumeCruise`.
- A **steering-angle offset** can be supplied live on plugin-bus topic
  `steer_angle_offset`; it is persisted to the plugin data dir as
  `SteerAngleOffset` and subtracted from the reported steering angle.
- `is_metric` is auto-detected from the ratio of set-speed to vEgo the first
  time cruise is engaged above 5 m/s.

## DCC cruise-stalk control (`carcontroller.py`)

openpilot has no direct set-speed channel on this car, so longitudinal
control is done by **emulating cruise-stalk (0x194) presses** — `plus1`,
`plus5`, `minus1`, `minus5`, `cancel`. The controller compares
`actuators.speed` (= planner `vTarget`, injected by `post_actuators`) against
current speed and the DCC set-speed, and issues stalk pulses:

- **Command selection** — direction gated by a `V_ERROR_DEADZONE` (~0.5 km/h)
  plus accel sign and set-speed headroom; `plus5/minus5` vs `plus1/minus1`
  chosen by accel magnitude thresholds; decel is blocked below the cruise
  minimum + buffer.
- **Cadence encodes magnitude** — DCC infers accel magnitude from press
  *rate*: `HOLD_INTERVAL` for large accel, `SINGLE_INTERVAL` otherwise. Note
  the constants are 25 ms / 50 ms but 25 ms is not representable on the 10 ms
  control grid: `dt_tx < interval - DT_CTRL / 2` puts HOLD's threshold exactly
  on a tick, so HOLD actually transmits at ~20 ms — measured **48 Hz on-car,
  not the 40 Hz the constant implies**. SINGLE is unaffected and measures
  20.1 Hz. The calibration table (PLUS1+HOLD ≈ +0.4 m/s², PLUS5+HOLD ≈ +1.2,
  MINUS1 ≈ −0.6, MINUS5 ≈ −1.2 m/s²) was measured against the real 48 Hz
  behaviour, so do not "correct" the cadence without re-measuring it.

### `CruiseCadence` — debug A/B param (default off)

Pins the stalk cadence regardless of demanded accel, so a HOLD-vs-SINGLE
comparison can be driven. Read once at `CarController.__init__`, so it applies
from the next drive start.

```sh
ssh c3 'echo hold   > /data/plugins-runtime/bmw_e9x_e8x/data/CruiseCadence'  # pin 48 Hz
ssh c3 'echo single > /data/plugins-runtime/bmw_e9x_e8x/data/CruiseCadence'  # pin 20 Hz
ssh c3 'rm -f        /data/plugins-runtime/bmw_e9x_e8x/data/CruiseCadence'   # back to normal
```

It exists because openpilot picks the command **and** the cadence from the same
demanded accel, so the two cells can never be matched observationally — 46 decel
bursts over 25 segments left the question open (gap bin [0.5, 2) showed HOLD at
−0.284 m/s² vs SINGLE at −0.075, but two other bins were a tie or a slight
reversal, and the cells differed in speed by 24 km/h). Drive the same road once
pinned `hold` and once pinned `single`, then compare decel at matched setpoint
gaps.

Changes frame **spacing only** — counter steps stay +1, so it carries none of
the 5ECE exposure described below. Any unrecognised value falls back to normal
demand-driven behaviour.

### How DCC accepts 0x194 — read this before touching the counter

> **The rule.** DSC/DCC ignores the stock SZL frame when an emulated frame
> arrives a few milliseconds ahead of it. **Suppression is by TIMING, not by
> comparing counters.** The counter's only job is to increment by **exactly
> +1** on the stream that survives that timing filter.
>
> **Never emit a step other than +1 on 0x194.**

The stock SZL module emits its own 0x194 counter open-loop at 5 Hz (+1 per
200 ms slot). The controller injects a frame inside a `PRE_TICK_LEAD` (15 ms)
window at the end of each stock idle slot so ours lands first and stock's
arrives into the suppression shadow. `bmwcan.create_accel_command` builds the
frame with the special zero-initialised 0x194 checksum.

Because SZL's frames are only *sometimes* overwritten, our counter has to stay
+1 relative to whatever DCC last processed — which may be an SZL frame that got
through. That is what `cruise_burst_released` handles: after a pause long
enough for SZL to slip through, the next command resyncs from RX rather than
resuming our own sequence (see `carcontroller.py`).

**Correction, 2026-09-06.** This section previously stated that DCC accepts a
frame iff `(1 + M − K) mod 15 ∈ [1, 7]`, and that the counter-overwrite works by
keeping our counter inside a forward window. **That is not the mechanism**, and
building on it put DTC 5ECE + CD95 on the car twice (routes 44c seg 24, 450 seg
18): a change that advanced the counter 16 per slot instead of +1 per frame
dropped the merged stream's +1 rate from ~94% to ~55%, and DCC matured the fault
after ~95 s of engaged driving and latched cruise off. Reverted in `12a3f57`.

`cruise_burst_release_safe()` still carries the old `(1 + M − K) mod 15` test.
It is retained because it is what currently gates when the trailing `act=0`
overwrite stops, and that behaviour is field-proven — but its stated rationale
is wrong, and it should be revisited against the timing rule rather than
extended.

Evidence — merged 0x194 stream after timing suppression, fraction of +1 steps:

| route | build | +1 rate | outcome |
|-------|-------|---------|---------|
| 444 | pre resume-fix | 94.6% | no 5ECE |
| 44b | resume fix | 93.9% | no DTC, 11 segments |
| 44c | 16-per-slot counter | 55.9% | **5ECE + CD95** |
| 450 | 16-per-slot counter | 54.8% | **5ECE + CD95** |

## Steering command (`carcontroller.py` + `bmwcan.py`)

When `STEPPER_SERVO_CAN` is set and lateral is active, the torque fraction
from the lateral controller is scaled to Nm (`STEER_MAX = 12`), rate-limited
by `apply_dist_to_meas_limits` against the measured EPS torque
(`STEER_DELTA_UP/DOWN = 0.1 Nm/10 ms`), and sent as an Ocelot
`STEERING_COMMAND` in `TorqueControl` mode with an 8-bit checksum. On
disengage it issues `SoftOff` then `Off` frames. `bmwcan.SteeringModes` also
defines an `AngleControl` mode, but only torque control is used.

## Panda safety (`safety/bmw.h`)

Compiled C, safety model id **35** (`bmw`), declared in `plugin.json`'s
`cereal.safety_models`. Enforced independently on the Panda:

- **TX allow-list**: only `CruiseControlStalk` (0x194, PT-CAN and F-CAN) and
  `STEPPER_STEERING_COMMAND` (F-CAN / AUX-CAN).
- **Torque limits**: `TorqueMotorLimited`, max 12 Nm, **speed-scaled** down to
  8 Nm at 80 km/h and 4 Nm at 100 km/h; rate up ≤ 0.125 Nm/10 ms, rate down
  ≤ 1.0 Nm/10 ms (matches the flashed firmware; tightening to the controller's
  0.2 needs on-car validation of the disengage/SoftOff decay path first);
  RT delta 25 Nm/250 ms.
- **Firmware build/flash** (2026-08-14): `safety/build_firmware.sh` injects the
  plugin-owned `bmw.h` + `tests/test_bmw.py` into the firmware workspace
  (`~/openpilot`, dzid26 fork with the F4-capable `OxygenLiu/panda`; panda
  pinned at `a0848226`), runs the safety suite, builds
  `panda-bmw-fw/board/obj/panda.bin.signed`, and restores the workspace tree —
  no opendbc fork is maintained. Flash from the C3 with the stack stopped;
  note the device lib's `Panda.flash()` hangs on F4 re-enumeration and
  hardcodes H7 sector layout — call `Panda.flash_static(handle, code,
  mcu_type=McuType.F4)` with the panda already in bootstub instead.
- **RX checks** on brake, gas, speed and either cruise-status message, plus
  the stepper status.
- `disable_forwarding = true`; cruise-engaged state is taken from the DCC/NCC
  status messages.
- **LKA mode** (2026-08-14): DCC engaging latches `controls_allowed`; DCC
  dropping does NOT clear it (lateral continues while the driver owns
  gas/brake), and `brake_pressed` is deliberately not reported — brake means
  "drop to LKA", not "kill steering". Openpilot owns ALL disengagement
  semantics (two-stage cancel etc.); panda follows it down via the stock
  firmware heartbeat (`controls_allowed && !heartbeat_engaged` for 3 s clears
  `controls_allowed`; lost heartbeat → SILENT) and enforces the torque limits.
  See `.superpowers/sdd/2026-08-14-bmw-lka-mode/lka-mode-brief.md`.

## Hooks

From `plugin.json` — **eight** hooks:

| Hook | Function (module) | Purpose |
|---|---|---|
| `controls.lat_controller_init` | `on_lat_controller_init` (`bmw.latcontroller`) | installs the custom BMW lateral controller — see [LATERAL_CONTROLLER.md](LATERAL_CONTROLLER.md) |
| `controls.post_actuators` | `on_post_actuators` (`register`) | overwrites `actuators.speed` with `long_plan.vTarget` (time-aligned with aTarget) for the DCC v_error gate |
| `car.cruise_initialized` | `on_cruise_initialized` (`register`) | cruise-ceiling memory: restore `v_cruise_kph_last` (30–145 km/h) on re-engage |
| `ui.vehicle_settings` | `on_vehicle_settings` (`register`) | append the Temperature-Overlay toggle + Resume-Button note to the Driving panel's vehicle section (only when `CP.brand == 'bmw'`) |
| `ui.render_overlay` | `on_render_overlay` (`ui_overlay`) | draw coolant/oil temperature on the driving HUD |
| `device.health_check` | `on_health_check` (`register`) | report whether the BMW interface registered into opendbc |
| `selfdrived.events_filter` | `on_events_filter` (`lka_mode`) | LKA two-stage disengagement: strip brake/first-cancel disengage events so lateral survives DCC dropping; a cancel press that starts in LKA fully disengages; any definite gear other than Drive disengages directly (stock soft-disable only for `unknown` glitches) |
| `ui.state_tick` | `on_ui_state_tick` (`ui_overlay`) | LKA border shows the override grey (UIStatus.OVERRIDE, same as gasPressed while engaged) |

The `ui.vehicle_settings` hook is **dispatched by** the `ui_mod` plugin from
inside its Driving panel: when a car is detected, ui_mod draws a vehicle
heading (brand emblem from `logos/icons/bmw.png` via `CP.brand`, plus the
fingerprint) and runs `ui.vehicle_settings` to collect car-specific rows —
this plugin's `on_vehicle_settings` is the producer. These rows render within
the Driving panel.

## Configuration / params

Params are **files in the plugin's `data/` dir** (runtime:
`/data/plugins-runtime/bmw_e9x_e8x/data/`), read/written via `register.py`'s
`_read_param`/`_write_param`. Never `/data/params/d/`.

| param | default | live? | note |
|---|---|---|---|
| `TemperatureOverlay` | on | yes (read each frame) | coolant/oil temps on the HUD; Driving-panel toggle |
| `CruiseCeilingMemory` | on | yes (read on engage) | restore last set-speed ceiling on re-engage within a drive |
| `SteerAngleOffset` | 0.0 | yes (1 Hz) | persisted steering-angle zero offset; updated from the `steer_angle_offset` plugin-bus topic, **not** a user-facing toggle |

`torque_params.toml` (LAT_ACCEL_FACTOR / MAX_LAT_ACCEL_MEASURED / FRICTION per
platform) is folded into opendbc's torque params at load time. Lateral-timing
knobs (chiefly `steerActuatorDelay = 0.4`) live in `interface.py`; see
[LATERAL_CONTROLLER.md](LATERAL_CONTROLLER.md).

> Note: the old README listed a `ConsecutiveLaneChange` param — it does not
> exist in the code or manifest and has been dropped.

## Telemetry

Plugin-bus topics (recorded into rlogs because `install.sh` injects the plugin
cereal schemas):

| topic | rate | source | payload |
|---|---|---|---|
| `bmw_lat_control` | 20 Hz | `bmw/latcontroller.py` | full lateral-controller state — see LATERAL_CONTROLLER.md §9 |
| `bmw_temps` | 0.2 Hz | `carstate.py` | `coolant`, `oil` (°C) for the HUD overlay |
| `speedlimit_cmd_car` | on press | `carstate.py` | `{'action': 'toggle_confirm'}` to speedlimitd |
| `steer_angle_offset` | in (1 Hz) | consumed by `carstate.py` | live steering-angle offset from any publisher |
| `speedLimitState` | in | consumed by `carstate.py` | drained on resume-press to sync speed-limit state |

## Key files

```
bmw_e9x_e8x/
  plugin.json           # manifest: 6 hooks, 2 declared params, safety model bmw=35
  register.py           # monkey-patch registration + 4 hook callbacks + param IO
  ui_overlay.py         # temperature HUD overlay (ui.render_overlay)
  torque_params.toml    # per-platform lateral torque params
  bmw/
    values.py           # platforms, VIN detection, flags, CAN bus map, DBC map
    fingerprints.py     # empty fingerprints + dummy FW (forces VIN fuzzy match)
    interface.py        # CarInterface._get_params — flags, cruise type, delays
    carstate.py         # CAN parsing, resume-button SM, temps, offset
    carcontroller.py    # DCC 0x194 stalk emulation + Ocelot steering
    bmwcan.py           # CAN message builders + checksums
    latcontroller.py    # custom lateral controller (see LATERAL_CONTROLLER.md)
  dbc/
    bmw_e9x_e8x.dbc     # PT-CAN / F-CAN definitions
    ocelot_controls.dbc # stepper-servo definitions
  safety/bmw.h          # Panda safety model (C, safety id 35)
  tests/                # unit + replay/analysis scripts
```

## Known issues / notes

- **`lagd` never converges** for this car (it correlates on
  `latcontrol_torque` telemetry the custom controller doesn't produce), so
  `liveDelay.lateralDelay` is permanently pinned at `steerActuatorDelay + 0.2`.
  `steerActuatorDelay` is the effective single timing knob — details in
  LATERAL_CONTROLLER.md §2.
- **DCC is the exercised cruise path.** NCC and the ACC-module variants are
  recognized and wired but far less tested; the `Footnote` strings in
  `values.py` flag this.
- **Hardware-dependent.** Steering needs the Ocelot stepper servo
  (`STEPPER_SERVO_CAN` flag); without it, only longitudinal/DCC is available.
