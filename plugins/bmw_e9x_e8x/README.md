# bmw_e9x_e8x — BMW Car Interface Plugin

**Type**: car (monkey-patch registration, no opendbc fork)

## What it does

Full BMW E8x/E9x car interface for openpilot:

- **VIN-based detection** — empty CAN fingerprints, pure VIN model code matching
- **Car interface** — CarState, CarController, CarParams via monkey-patching into openpilot's car registry
- **DCC control** — speed-dependent Dynamic Cruise Control via tick-counted commands
- **Stepper servo steering** — Ocelot stepper servo on F-CAN/AUX-CAN
- **Panda safety model** — BMW safety rules (bmw.h, safety model ID 35)
- **Cruise ceiling memory** — remembers cruise speed across disengage/re-engage within a drive
- **Temperature overlay** — coolant and oil temps on the onroad HUD
- **Vehicle settings** — brand-specific settings panel with car name and emblem
- **Speed limit toggle** — short press resume button while engaged to confirm/cancel speed limit

## Supported Platforms

| Platform | Description |
|----------|-------------|
| BMW_E82 | 1-Series Coupe/Convertible (2004-13) |
| BMW_E90 | 3-Series Sedan/Wagon/Coupe/Convertible (2005-11) |

## Hooks

| Hook | Function | Description |
|------|----------|-------------|
| `controls.post_actuators` | on_post_actuators | DCC vTarget override |
| `car.cruise_initialized` | on_cruise_initialized | Cruise ceiling memory on engage |
| `ui.vehicle_settings` | on_vehicle_settings | Vehicle panel items |
| `ui.state_subscriptions` | on_state_subscriptions | Subscribe to carState for temps |
| `ui.render_overlay` | on_render_overlay | Temperature overlay + resume button handler |

## CAN Bus Layout

| Bus | Name | Messages |
|-----|------|----------|
| 0 | PT-CAN | Engine, brakes, speed, transmission, cruise |
| 1 | F-CAN | Stepper servo (if equipped) |
| 2 | AUX-CAN | Alternative servo bus, message routing |

## Key Files

```
bmw_e9x_e8x/
  plugin.json           # Plugin manifest (9 hooks, 3 params, safety model)
  register.py           # Hook handlers + monkey-patch registration
  ui_overlay.py         # Temperature overlay + resume button handler
  bmw/                  # Car interface (mirrors opendbc/car/bmw/)
    values.py           # Platform config, VIN detection, DBC mapping
    fingerprints.py     # Empty fingerprints + dummy FW versions
    interface.py        # CarInterface (get_params, _get_params)
    carstate.py         # CAN message parsing
    carcontroller.py    # DCC commands, stepper servo, turn signals
    bmwcan.py           # CAN message construction helpers
  dbc/                  # DBC files
    bmw_e9x_e8x.dbc    # BMW PT-CAN + F-CAN definitions
    ocelot_controls.dbc # Stepper servo CAN definitions
  safety/
    bmw.h               # Panda safety model (C, compiled on STM32)
  torque_params.toml    # Lateral torque tuning parameters
```

## Params

| Param | Default | Description |
|-------|---------|-------------|
| CruiseCeilingMemory | true | Remember cruise speed ceiling across disengage/re-engage |
| ConsecutiveLaneChange | true | Chain lane changes via steering button |
| TemperatureOverlay | true | Show coolant/oil temps on onroad HUD |

## Credits

Based on [dzid26's BMW E8x/E9x openpilot implementation](https://github.com/BMW-E8x-E9x/openpilot).
