# Workingpins

This repo provides a simple virtual input pin implementation for Klipper.
Eight global pins named `pin1` through `pin8` are available via the
`ams_pin:` prefix and may be referenced anywhere an endstop-style input pin
is expected.  No dedicated configuration section is strictly required -- the
pins are created automatically the first time the prefix is used.

Klipper processes configuration sections in order. If a `[ams_pin]`
section is present it may appear anywhere in the file. The pins are
also created automatically the first time the `ams_pin:` prefix is
encountered, so the section is optional unless a module specifically
requires it. An example configuration referencing `pin1` via the
provided virtual filament sensor module is:

```ini
[ams_pin]        # optional
[virtual_filament_sensor my_sensor]
pin: ams_pin:pin1
```

Individual pin sections are optional – the pins exist globally once the
`ams_pin` module is imported.

If Klipper reports `Unknown pin chip name 'ams_pin'` during startup it
usually means the module was not copied into Klipper's
`klippy/extras/` directory.  Verify that `ams_pin.py` is located in that
directory and restart Klipper.

If you have existing scripts that import `virtual_input_pin.py` you can copy
that file as well; it simply re-exports everything from `ams_pin.py`.  Only the
`ams_pin.py` file is required for the prefix to be recognized.
