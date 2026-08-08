# Comma 3 Compatibility

Keeps the [comma three](https://github.com/commaai/hardware/tree/master/comma_three)
(2021, code name "tici") fully supported on current catpilot. Upstream openpilot
dropped the comma three in v0.10.3 — it removed the STM32F4 panda code, the tici
audio config, and the old display stack, and it now targets a newer device OS
(AGNOS 16) than the comma three can run (AGNOS 12.8). This plugin bridges every
one of those gaps at boot, so a comma three runs the same catpilot as newer
hardware without a separate branch. It is infrastructure, not a driving feature —
there is nothing to tune and nothing to interact with.

On a comma three device this plugin is **enforced**: it is always on and greyed out in
the Plugins panel, because the device will not run catpilot correctly without it.

## Prerequisite — AGNOS 12.8

The comma three must already be running AGNOS 12.8 before installing catpilot;
the installer checks this and refuses with instructions otherwise. To get there,
either:

+ flash AGNOS 12.8 via https://flash.comma.ai/, or
+ install the latest supported stock openpilot (v0.10.0) — just pick the
  standard openpilot option during device setup; a comma three is automatically
  given the last release that supports it.

**WARNING: flashing wipes all data on the device (drives, settings, pairing).**

Then install catpilot by entering `installer.catpilot.dev` under Custom
Software — c3_compat is bundled and applied automatically on every boot, with
no manual setup.

Please
[report any issues](https://github.com/catpilot-dev/plugins/issues). Use at your
own risk — we are not liable for any consequences.

## More

Every compatibility shim, why it exists, and how it hooks in at boot are in
[DESIGN.md](DESIGN.md).
