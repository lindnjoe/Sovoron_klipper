# Virtual Pins and Filament Sensors

This repository includes custom Klipper extras that add software defined input pins and a filament sensor emulation. These sections are configured in your `printer.cfg` and can be controlled from G-code.

## Defining a `[virtual_pin]`

```
[virtual_pin <name>]
initial_value: <0 or 1>
```

`initial_value` is optional and defaults to `0`.  A virtual pin behaves like an endstop input that can be toggled via G-code.

## Linking a `[virtual_filament_sensor]`

```
[virtual_filament_sensor <sensor_name>]
pin: <virtual_pin name>
# pause_on_runout: True
# runout_gcode: ...
# insert_gcode: ...
# pause_delay: 0.5
# event_delay: 3.0
```

The `pin` option must reference an existing `[virtual_pin]`.  Additional parameters mirror those of Klipper's standard `filament_switch_sensor` module.

## G-code commands

The modules register several commands to read or modify the virtual pins and sensors:

- `SET_VIRTUAL_PIN PIN=<name> [VALUE=0|1]`
- `QUERY_VIRTUAL_PIN PIN=<name>`
- `SET_VIRTUAL_FILAMENT_PIN SENSOR=<name> [VALUE=0|1]`
- `QUERY_VIRTUAL_FILAMENT_PIN SENSOR=<name>`
- `SET_FILAMENT_SENSOR SENSOR=<name> [ENABLE=0|1]`
- `QUERY_FILAMENT_SENSOR SENSOR=<name>`

Example usage:

```
SET_VIRTUAL_PIN PIN=runout_button VALUE=0
QUERY_VIRTUAL_PIN PIN=runout_button
```
