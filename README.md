

This backup is provided by [Klipper-Backup](https://github.com/Staubgeborener/klipper-backup).

## AMS virtual pins

Create virtual pins with Klipper's `virtual_input_pin` module and use
`auto_ams_update` to automatically mirror AMS lane status. Define
virtual pins for each AMS lane and hub input and configure
`auto_ams_update` to update them:

```
[virtual_input_pin ams1lane0pl]
[virtual_input_pin ams1lane1pl]
[virtual_input_pin ams1lane2pl]
[virtual_input_pin ams1lane3pl]
[virtual_input_pin ams2lane0pl]
[virtual_input_pin ams2lane1pl]
[virtual_input_pin ams2lane2pl]
[virtual_input_pin ams2lane3pl]
[virtual_input_pin ams1hub0]
[virtual_input_pin ams1hub1]
[virtual_input_pin ams1hub2]
[virtual_input_pin ams1hub3]
[virtual_input_pin ams2hub0]
[virtual_input_pin ams2hub1]
[virtual_input_pin ams2hub2]
[virtual_input_pin ams2hub3]

[auto_ams_update]
oams1: oams1
oams2: oams2
pins: ams1lane0pl, ams1lane1pl, ams1lane2pl, ams1lane3pl, \
      ams2lane0pl, ams2lane1pl, ams2lane2pl, ams2lane3pl, \
      ams1hub0, ams1hub1, ams1hub2, ams1hub3, \
      ams2hub0, ams2hub1, ams2hub2, ams2hub3
interval: 1
```

Add more `oams#` options (for example, `oams3: oams3`) and extend the
`pins` list with that AMS's pin names. List the lane pins for all AMS
units first, followed by the hub pins for all AMS units.

Use these pins like normal endstop pins:

```
[filament_switch_sensor my_sensor]
    switch_pin: virtual_pin:ams1lane0pl
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

