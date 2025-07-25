# Workingpins

This repo provides a simple virtual input pin implementation for Klipper.
Eight global pins named `pin1` through `pin8` are available via the
`ams_pin:` prefix and may be referenced anywhere an endstop-style input pin
is expected.

To enable the virtual pins the configuration must contain a single
`[ams_pin]` section **before** any other section that references an
`ams_pin:` pin.  The minimal configuration is:

```ini
[ams_pin]
```

Individual pin sections are optional – the pins exist globally once the
`[ams_pin]` section is processed.

Example usage with the provided virtual filament sensor module:

```ini
[ams_pin]

[virtual_filament_sensor my_sensor]
pin: ams_pin:pin1
```
