# Klipper-Backup 💾 
Klipper backup script for manual or automated GitHub backups 

This backup is provided by [Klipper-Backup](https://github.com/Staubgeborener/klipper-backup).

## Virtual filament sensor configuration

The printer configuration can emulate a filament runout switch using
Klipper's `virtual_input` and `virtual_filament_sensor` objects.  Add the
following to `printer.cfg`:

```
[virtual_input spool_pin]
initial_value: 1

[virtual_filament_sensor spool_sensor]
pin: spool_pin
```

The `virtual_input` defines a named pin (`spool_pin` in this example) that
defaults to a high logic level.  The `virtual_filament_sensor` references
this pin and behaves like a standard filament sensor.

Macros or console commands can toggle or query the pin with:

```
SET_VIRTUAL_FILAMENT_PIN PIN=spool_pin VALUE=<0|1>
QUERY_VIRTUAL_FILAMENT_PIN PIN=spool_pin
```

These commands allow testing filament runout logic or implementing custom
automation without physical hardware.
