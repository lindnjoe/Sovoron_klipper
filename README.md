
# Klipper-Backup 💾 
Klipper backup script for manual or automated GitHub backups 

This backup is provided by [Klipper-Backup](https://github.com/Staubgeborener/klipper-backup).

## Virtual input pins

This repository includes a small Klipper module that provides software
**input** pins. Each pin behaves as an endstop-style input so other
modules treat it like any physical input pin.  Pins register under the
chip name `ams`, so they are referenced as `ams:<name>` just like a real
MCU pin.

Use the `SET_AMS_PIN` and `QUERY_AMS_PIN` gcode commands to
manually update or read the pin state.

If other modules parse pin names before any `[ams_pins]` or `[ams_pin]`
