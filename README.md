
# Klipper-Backup 💾
Klipper backup script for manual or automated GitHub backups

This backup is provided by [Klipper-Backup](https://github.com/Staubgeborener/klipper-backup).

## AMS virtual pins

This repository includes a small Klipper module that creates eight
software input pins on a fake MCU named `ams`.  After adding the section

```
[ams_virtual_pins]
```

to your configuration, pins `pin1` through `pin8` become available under
the chip name `ams`.  They may be referenced like normal endstop pins,
for example:

```
[filament_switch_sensor my_sensor]
    switch_pin: ams:pin1
```

Change a pin state at runtime with:

```
SET_AMS_PIN PIN=pin1 VALUE=1
QUERY_AMS_PIN PIN=pin1
```

These pins behave like real endstop inputs, so they can be used anywhere
an input pin is expected.
=======


