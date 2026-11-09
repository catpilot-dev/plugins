# Comma 3 Compatibility

Keeps the [comma three](https://github.com/commaai/hardware/tree/master/comma_three)
(2021, code name "tici") fully supported on current catpilot, whereas upstream
ended comma three support after v0.10.0, the last stock release that runs on it.

On a comma three device this plugin is **enforced**: it is always on and grayed out in
the Plugins panel, because the device will not run catpilot correctly without it.

## Prerequisite — AGNOS 12.8

The comma three must already be running AGNOS 12.8 before installing catpilot;
the installer checks this and refuses with instructions otherwise. To get there,
either:

+ flash AGNOS 12.8 via https://flash.comma.ai/, or
+ install the latest supported stock openpilot (v0.10.0) via https://openpilot.comma.ai

**WARNING: flashing wipes all data on the device (drives, settings, pairing).**

Then install catpilot by entering `install.catpilot.dev` under Custom
Software — c3_compat is bundled and applied automatically on every boot, with
no manual setup.

## What it restores

Everything below is applied automatically at every boot:

+ the AGNOS 12.8 Python environment, kept in sync with whichever catpilot
  version is installed
+ the display, drawn directly on the screen without the Wayland compositor
+ the comma three's panda (STM32F4) over USB, including its health reporting
+ the road and cabin cameras (AR0231 sensor) and video recording
+ modem start-up on AGNOS 12.8

## Updating to openpilot 0.11.2

openpilot 0.11.2 removed the comma three's camera sensor and panda USB code, so
on a comma three catpilot 0.11.2 builds that support on the device. The first
boot after installing or updating takes noticeably longer than usual while
it builds and syncs the Python environment. Keep the device powered until the
home screen appears; later boots are normal.

Please
[report any issues](https://github.com/catpilot-dev/plugins/issues). Use at your
own risk — we are not liable for any consequences.

## More

Every compatibility shim, why it exists, and how it hooks in at boot are in
[DESIGN.md](DESIGN.md).
