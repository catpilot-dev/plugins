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
  *rate*: `HOLD_INTERVAL` (40 Hz) for large accel, `SINGLE_INTERVAL` (20 Hz)
  otherwise. The calibration table (PLUS1+HOLD ≈ +0.4 m/s², PLUS5+HOLD ≈
  +1.2, MINUS1 ≈ −0.6, MINUS5 ≈ −1.2 m/s²) is noted inline.
- **Counter-overwrite machinery (the 0x194 / DTC-5ECE fix)** — the stock SZL
  module emits its own 0x194 counter open-loop at 5 Hz (+1 per 200 ms slot).
  To keep DCC accepting openpilot's frames without raising DTC 5ECE, the
  controller injects a frame inside a `PRE_TICK_LEAD` (15 ms) window at the
  end of each stock idle slot so **our** counter lands first on PT-CAN and
  stock's later same-or-earlier-counter idle frame is dropped as stale. Every
  in-burst slot must be overwritten (SZL drifts +7/slot on HOLD, +3 on
  SINGLE). Handoff back to stock is only released when
  `(1 + M − K) mod 15 ∈ [1, 7]` (M = slots overwritten, K = frames sent);
  until then it keeps transmitting neutral `act=0` frames.
  `bmwcan.create_accel_command` builds the frame with the special
  zero-initialised 0x194 checksum. This counter-collision handling is what
  keeps DCC from raising the 5ECE rollback fault.

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
  ≤ 0.2 Nm/10 ms; RT delta 25 Nm/250 ms.
- **RX checks** on brake, gas, speed and either cruise-status message, plus
  the stepper status.
- `disable_forwarding = true`; cruise-engaged state is taken from the DCC/NCC
  status messages via `pcm_cruise_check`.

## Hooks

From `plugin.json` — **six** hooks:

| Hook | Function (module) | Purpose |
|---|---|---|
| `controls.lat_controller_init` | `on_lat_controller_init` (`bmw.latcontroller`) | installs the custom BMW lateral controller — see [LATERAL_CONTROLLER.md](LATERAL_CONTROLLER.md) |
| `controls.post_actuators` | `on_post_actuators` (`register`) | overwrites `actuators.speed` with `long_plan.vTarget` (time-aligned with aTarget) for the DCC v_error gate |
| `car.cruise_initialized` | `on_cruise_initialized` (`register`) | cruise-ceiling memory: restore `v_cruise_kph_last` (30–145 km/h) on re-engage |
| `ui.vehicle_settings` | `on_vehicle_settings` (`register`) | append the Temperature-Overlay toggle + Resume-Button note to the Driving panel's vehicle section (only when `CP.brand == 'bmw'`) |
| `ui.render_overlay` | `on_render_overlay` (`ui_overlay`) | draw coolant/oil temperature on the driving HUD |
| `device.health_check` | `on_health_check` (`register`) | report whether the BMW interface registered into opendbc |

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
  tools/dcc_study/      # DCC response-study tooling
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
