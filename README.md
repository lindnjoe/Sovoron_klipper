# Klipper-Backup 💾 
Klipper backup script for manual or automated GitHub backups 

This backup is provided by [Klipper-Backup](https://github.com/Staubgeborener/klipper-backup).

## Virtual input pins

This repository includes a small Klipper module that provides software
**input** pins. Each pin behaves as an endstop-style input so other
modules treat it like any physical input pin.  Define a pin with an
`[ams_pin]` (or `[virtual_pin]`) section and reference it elsewhere as
`ams_pin:<name>`.

Use the `SET_VIRTUAL_PIN` and `QUERY_VIRTUAL_PIN` gcode commands to
manually update or read the pin state.

Example:

```
[ams_pin runout_button]
initial_value: 1
```

