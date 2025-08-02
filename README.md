

This backup is provided by [Klipper-Backup](https://github.com/Staubgeborener/klipper-backup).

## AMS virtual pins

Create virtual pins with Klipper's `virtual_input_pin` module and use
`auto_ams_update` to automatically mirror AMS lane status. Define pin
sections named `pin1` through `pin16` and configure `auto_ams_update`
to update them:

```
[virtual_input_pin pin1]
[virtual_input_pin pin2]
...
[virtual_input_pin pin16]

[auto_ams_update]
oams1: oams1
oams2: oams2
pins: pin1, pin2, pin3, pin4, pin5, pin6, pin7, pin8, \
      pin9, pin10, pin11, pin12, pin13, pin14, pin15, pin16
interval: 1
```

Use these pins like normal endstop pins:

```
[filament_switch_sensor my_sensor]
    switch_pin: virtual_pin:pin1
```

Change a pin state at runtime with `SET_VIRTUAL_PIN` and query it with
`QUERY_VIRTUAL_PIN`. These pins behave like real endstop inputs, so they
can be used anywhere an input pin is expected.

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

