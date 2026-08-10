# Comma 3 Compatibility

Keeps the [comma three](https://github.com/commaai/hardware/tree/master/comma_three)
(2021, code name "tici") fully supported on current catpilot, whereas upstream
moved the comma three to LTS at the v0.10.0 release.

On a comma three device this plugin is **enforced**: it is always on and greyed out in
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

Please
[report any issues](https://github.com/catpilot-dev/plugins/issues). Use at your
own risk — we are not liable for any consequences.

## More

Every compatibility shim, why it exists, and how it hooks in at boot are in
[DESIGN.md](DESIGN.md).
