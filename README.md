

This backup is provided by [Klipper-Backup](https://github.com/Staubgeborener/klipper-backup).

## AMS virtual pins

This repository includes a small Klipper module that creates eight
software input pins on a fake MCU named `ams`.  After adding the section

```
[ams_virtual_pins]
```

to your configuration, pins `pin1` through `pin8` become available under
the chip name `ams` (or the aliases `ams_pin` and `virtual_pin`).  They may be referenced
like normal endstop pins,
for example:

```
[filament_switch_sensor my_sensor]
    switch_pin: ams:pin1
```

Change a pin state at runtime with:

```
SET_AMS_PIN PIN=pin1 VALUE=1
QUERY_AMS_PIN PIN=pin1
# The `PIN` parameter may use either `pin1`, `ams_pin:pin1`, or `virtual_pin:pin1`.
```

These pins behave like real endstop inputs, so they can be used anywhere
an input pin is expected.

## Virtual input pins

A generic module allows defining arbitrary software pins. Add sections like:

```
[virtual_input_pin my_pin]
initial_value: 0
```

Use these pins anywhere an endstop pin is accepted by referencing
`virtual_pin:my_pin`. Change a pin at runtime using:

```
SET_VIRTUAL_PIN PIN=my_pin VALUE=1
QUERY_VIRTUAL_PIN PIN=my_pin
```

