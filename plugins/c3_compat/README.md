# Comma 3 Compatibility

Keeps the [comma three](https://github.com/commaai/hardware/tree/master/comma_three)
(2021, code name "tici") fully supported on current catpilot. Upstream openpilot
dropped the comma three in v0.10.3 — it removed the STM32F4 panda code, the tici
audio config, and the old display stack, and it now targets a newer device OS
(AGNOS 16) than the comma three can run (AGNOS 12.8). This plugin bridges every
one of those gaps at boot, so a comma three runs the same catpilot as newer
hardware without a separate branch. It is infrastructure, not a driving feature —
there is nothing to tune and nothing to interact with.

## What it keeps working

- **The panda** — the comma three's internal STM32F4 (Dos) board is detected,
  talks over USB, and skips the firmware reflash it doesn't need.
- **The screen** — the UI renders through the DRM display backend instead of the
  Wayland/Weston stack the comma three doesn't use, and boots ~28 s faster for it.
- **Audio** — the tici speaker/EQ configuration is restored.
- **The software build** — the Python environment is kept in sync with whatever
  catpilot version is deployed, so the code always imports and builds on the
  older AGNOS 12.8 system.
- **Memory stability** — a known memory leak in a core library is pinned out, so
  the device doesn't run out of RAM and reset mid-drive.
- **The modem / cell connection** — a boot-time crash from the older modem
  firmware is guarded so the device still goes on- and off-road normally.

## Do not disable this on a comma three

On a comma three this plugin is **enforced**: it is always on and greyed out in
the Plugins panel, because the device will not run catpilot correctly without it.
Turn it off and the comma three may fail to see its panda, produce no audio, show
no UI, or run out of memory and reset. There is no reason to disable it and no
supported way to run without it. On other hardware (comma 3X / 4) it does nothing
and stays out of the way.

## For comma three owners

Install a pre-patched [catpilot](https://github.com/catpilot-dev/catpilot)
release — c3_compat is bundled and applied automatically on every boot, with no
manual setup. Please
[report any issues](https://github.com/catpilot-dev/plugins/issues). Use at your
own risk — we are not liable for any consequences.

## More

Every compatibility shim, why it exists, and how it hooks in at boot are in
[DESIGN.md](DESIGN.md).
