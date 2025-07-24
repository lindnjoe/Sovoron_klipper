# Klipper-Backup 💾
Klipper backup script for manual or automated GitHub backups

This backup is provided by [Klipper-Backup](https://github.com/Staubgeborener/klipper-backup).

## Virtual input pins

This repository includes a small Klipper module that lets you define
software input pins.  Pins register under the chip name `virtual_pin` but
aliases of `ams_pin` and `ams` are also provided.  Thus a pin can be
referenced as `virtual_pin:name`, `ams_pin:name`, or simply `ams:name`
in other configuration sections.

If another module references an `ams:` pin before any `[virtual_input_pin]`
section is parsed, add an empty `[ams_chip]` section near the start of the
configuration to register the chip early.

Legacy configurations using `[ams_pin]` sections continue to work via a
compatibility wrapper that forwards to the same implementation.

Add a section such as:

```
[virtual_input_pin runout_button]
initial_value: 1
```

Change the pin state with `SET_VIRTUAL_PIN PIN=runout_button VALUE=0`
and query it with `QUERY_VIRTUAL_PIN PIN=runout_button`.
