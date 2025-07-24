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
