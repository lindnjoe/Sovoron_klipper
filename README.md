
# Klipper-Backup 💾
Klipper backup script for manual or automated GitHub backups

This backup is provided by [Klipper-Backup](https://github.com/Staubgeborener/klipper-backup).

## Virtual input pins

This repository includes a small Klipper module that lets you define
software input pins.  Each pin is addressed through the chip name
`virtual_pin` so it can be referenced like `virtual_pin:runout_button` in
other configuration sections.

Add a section such as:

```
[virtual_input_pin runout_button]
initial_value: 1
```

Change the pin state with `SET_VIRTUAL_PIN PIN=runout_button VALUE=0`
and query it with `QUERY_VIRTUAL_PIN PIN=runout_button`.
=======

# Klipper-Backup 💾 
Klipper backup script for manual or automated GitHub backups 

This backup is provided by [Klipper-Backup](https://github.com/Staubgeborener/klipper-backup).


## Virtual AMS pins

This repo includes an extra Klipper module that provides simple software
input pins.  Each pin registers under the chip name `ams` so it can be
referenced like `ams:pin1`.  Add a section like the following to create a
single pin:

```
[ams_pin runout_button]
initial_value: 1
```

Use `SET_AMS_PIN PIN=runout_button VALUE=0` to change its state at
runtime and `QUERY_AMS_PIN PIN=runout_button` to read it.

Alternatively `[ams_pins]` creates eight pins named `pin1` through
`pin8`, each paired with a basic filament switch sensor so the rest of
Klipper can read their status.

If other modules need to reference these pins before the `[ams_pin]` or
`[ams_pins]` sections load, add an empty `[ams_chip]` section near the
top of your configuration file.  This registers the virtual `ams` chip
early so pin lookups succeed.
=======


