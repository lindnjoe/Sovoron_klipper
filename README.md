# Klipper-Backup 💾 
Klipper backup script for manual or automated GitHub backups 

This backup is provided by [Klipper-Backup](https://github.com/Staubgeborener/klipper-backup).

## Virtual input pins

This repository includes a small Klipper module that provides software
**input** pins. Each pin behaves as an endstop-style input so other
modules treat it like any physical input pin.  Define a pin with an
`[virtual_input_pin]` section and reference it elsewhere as `virtual_pin:<name>`.

Use the `SET_VIRTUAL_PIN` and `QUERY_VIRTUAL_PIN` gcode commands to
manually update or read the pin state.

If other modules parse pin names before any `[virtual_input_pin]` section is
encountered, add an empty `[virtual_input_pin]` section near the start of the
config file to register the virtual chip before those modules load.

Example:

```
[virtual_input_pin runout_button]
initial_value: 1
```

Use `SET_VIRTUAL_PIN PIN=runout_button VALUE=0` to toggle the pin at runtime
and `QUERY_VIRTUAL_PIN PIN=runout_button` to report its current state.

Pin names are matched case-insensitively, so `RUNOUT_BUTTON` and
`runout_button` refer to the same pin.  Pins also register with an
`virtual_pin:` prefix for modules that expect the chip name to be present.

Each pin is stored internally under a single normalized name.  This
ensures that multiple `[virtual_input_pin]` sections act independently even when
names differ only by case.

As of this version every pin allocates its own unique OID so subsystems
like the button handler can treat each virtual pin as a separate MCU
input.  Button events now include this OID so handlers can distinguish
between multiple virtual pins.  This fixes issues where only the first
configured pin responded to changes.

