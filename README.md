# Klipper-Backup 💾 
Klipper backup script for manual or automated GitHub backups 

This backup is provided by [Klipper-Backup](https://github.com/Staubgeborener/klipper-backup).

## Virtual input pins

This repository includes a small Klipper module that provides software
input pins. Define a pin with an `[ams_pin]` (or `[virtual_pin]`)
section and reference it elsewhere as `ams_pin:<name>`.

Example:

```
[ams_pin runout_button]
initial_value: 1
```

