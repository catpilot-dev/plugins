# catpilot plugins

Plugins for [catpilot](https://github.com/catpilot-dev/catpilot) — the
features that ride on top of stock openpilot. They come pre-installed with
every catpilot release and are versioned with it (release `0.11.1` ships
plugins `0.11.1`): flash catpilot, and everything below is already on the
device.

## What you get

### On the road

| Plugin | What it does for you |
|--------|----------------------|
| `bmw_e9x_e8x` | Makes BMW E9x/E8x cars (E82/E90 family) drivable with catpilot: cruise control, steering, lane changes — no factory driver-assist needed |
| `speedlimitd` | Watches the speed limit for you — from offline OpenStreetMap data (optional toggle) and what the camera sees (lane count, road type) — and slows the car for sharp curves and highway ramps |
| `lane_keeping` | Calms the slow left-right sway ("ping-pong") inside the lane, so the car holds its line more steadily |

### On the screen

| Plugin | What it does for you |
|--------|----------------------|
| `ui_mod` | The catpilot look: home screen with drive stats and a route map, your car's emblem on the driving screen, and extra settings panels (Driving, Plugins) |
| `model_selector` | Lets you download and switch between driving models from the Software panel |
| `screen_capture` | Tap the camera icon to save a screenshot of the driving screen |

### Under the hood

| Plugin | What it does for you |
|--------|----------------------|
| `c3_compat` | Keeps the comma three fully supported on current catpilot (display, panda, OS compatibility) |
| `mapd` | Manages an external OpenStreetMap data binary. Currently inactive (unwired in its manifest) — speed limits come through `speedlimitd` instead |
| `bus_logger` | Saves the plugins' status messages into the drive logs, so problems can be diagnosed after a drive |

## Turning things on and off

- **Settings → Plugins** — one toggle per plugin.
- Feature-level switches live in their own panels: for example, the
  **Driving** panel has the "Lane Keeping" toggle, and the **Software**
  panel hosts the model selector.
- A few plugins are marked *enforced* because the car depends on them
  (for example `lane_keeping` on BMW) — for those, use their feature
  toggle instead of disabling the whole plugin.

## Updating

Plugins update together with catpilot. Nothing to do — a catpilot update
brings the matching plugins with it.

## Learn more

Every plugin has its own `README.md` inside
[`plugins/`](plugins/) explaining what it does in more detail; the more
technical ones also carry design docs (for example
[`lane_keeping/DESIGN.md`](plugins/lane_keeping/DESIGN.md)).

Want to build a plugin or understand the framework? Start with
[`docs/DEVELOPMENT.md`](docs/DEVELOPMENT.md).

## License

MIT
