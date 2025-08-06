

This backup is provided by [Klipper-Backup](https://github.com/Staubgeborener/klipper-backup).

## AMS virtual pins


Use Klipper's `virtual_input_pin` module with `auto_ams_update` to
mirror AMS lane status. `auto_ams_update` will automatically create
virtual pins for each configured AMS using the naming pattern
`ams#lane{0-3}pl` and `ams#hub{0-3}`. List AMS objects under
`oams#:` options:

```
[auto_ams_update]
oams1: oams1
oams2: oams2
interval: 1
#pins: ams1lane0pl, ams1lane1pl, ams1lane2pl, ams1lane3pl, ams2lane0pl, ams2lane1pl, ams2lane2pl, ams2lane3pl, ams1hub0, ams1hub1, ams1hub2, ams1hub3, ams2hub0, ams2hub1, ams2hub2, ams2hub3
```

Pins `ams1lane0pl` through `ams1hub3` and `ams2lane0pl` through
`ams2hub3` are created automatically. Add more `oams#` options (for
example, `oams3: oams3`) to manage additional AMS units. To override the
default pin names, supply a `pins` option listing lane pins for all AMS
units followed by the hub pins. Additional virtual pins may still be
defined manually using `[virtual_input_pin my_pin]` if needed.





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
