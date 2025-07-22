# Klipper-Backup 💾 
Klipper backup script for manual or automated GitHub backups 

This backup is provided by [Klipper-Backup](https://github.com/Staubgeborener/klipper-backup).

## Virtual input pins

This repository includes a small Klipper module that provides software
**input** pins. Each pin behaves as an endstop-style input so other
modules treat it like any physical input pin.  Define a pin with an
`[ams_pin]` section and reference it elsewhere as `ams_pin:<name>`.

Use the `SET_AMS_PIN` and `QUERY_AMS_PIN` gcode commands to
manually update or read the pin state.

If other modules parse pin names before any `[ams_pin]` section is
encountered (for example `duplicate_pin_override`), add an empty
`[ams_pins]` section to register the `ams_pin` chip early.

Example:

```
[ams_pin runout_button]
initial_value: 1
```

Use `SET_AMS_PIN PIN=runout_button VALUE=0` to toggle the pin at runtime
and `QUERY_AMS_PIN PIN=runout_button` to report its current state.

### Automatic OAMS lane pins

The `oams_virtual_pins` module can mirror each `[oams <name>]` section in
`oamsc.cfg` to eight virtual pins named
`<name>_lane0_prep`, `<name>_lane0_load`, ...,
`<name>_lane3_prep`, `<name>_lane3_load`.
These pins update automatically from the `f1s_hes_value` list exported by
the corresponding OAMS module and may be referenced as
`ams_pin:<name>_lane#_prep` or `ams_pin:<name>_lane#_load` elsewhere in the configuration. Enable the feature by
adding a simple section:

```
[oams_virtual_pins]
oams_config: oamsc.cfg
poll_interval: 1.0
```

