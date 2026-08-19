# Model Selector

Browse, download, and switch between openpilot driving and driver-monitoring
models.

## What it does

openpilot ships with one driving model and one driver-monitoring (DM) model.
This plugin lets you keep several of each on the device and pick which one is
active:

- **Driving model** — the neural net that actually steers and paces the car
  (lateral and longitudinal). Swapping it is the one that changes how the car
  drives.
- **Driver-monitoring model** — swapping it changes how driver monitoring behaves.

It can also download models that comma has published and that have been test
driven on your openpilot version, ready to switch to whenever you want.

## How to use it

<img src="../../docs/software_panel.png" width="66%" alt="Settings → Software panel with the model selector rows" />

Everything lives in **Settings → Software**. The plugin adds three rows:

- **Driving Model** — shows the active driving model; tap **SELECT** to see
  every driving model installed on the device and switch, or delete one you no
  longer want.
- **Driver Monitoring** — same, for the DM model.
- **Tested Models** — tap **CHECK**. It lists the models that have been test
  driven on your openpilot version and aren't installed yet, and lets you
  download one. The list ships with the plugin, so it updates when catpilot
  does. The row shows status (`checking…`, `N available`, `downloading`,
  `download complete`, `up to date`).

When you activate a model, the plugin stages it and asks you to **Reboot** —
the swap only takes effect after the reboot. If you cancel the reboot prompt,
it puts the previous model back, so nothing changes until you actually reboot.

## Things to know

- **A driving-model swap changes how the car drives.** Treat a new driving
  model like any other tuning change: try it somewhere safe first.
- **It takes effect after a reboot**, not live while driving.
- **First drive after a swap is slow to start.** The first time a given model
  runs on this device, openpilot has to compile it for the hardware (roughly a
  minute on a comma three). After that it's cached — switching back to a model
  you've used before is instant.
- **You can't delete the model that's currently active.** Switch to another
  one first.
- **Only tested models can be selected.** Every model offered here has been
  test driven on your openpilot version, along with the model your release
  ships. Anything else on the device is shown as *untested* and can't be
  activated.
- **After a catpilot update, a model you were using may become untested.**
  It keeps running — nothing changes under you — but the panel flags it and
  offers to switch you back to the model your release ships.

## Turning it on and off

It's a standard plugin: enable or disable it under **Settings → Plugins**.
With it off, the extra rows in the Software panel simply don't appear and the
active model is whatever was last selected.

## More

Where models are stored, how downloads and the ONNX→PKL cache work, and the
hooks it uses are in [DESIGN.md](DESIGN.md).
