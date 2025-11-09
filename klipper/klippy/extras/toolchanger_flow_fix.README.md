# Toolchanger Flow Fix

This module fixes the volumetric flow rate tracking issue with toolchanger configurations.

## Problem Fixed

### Volumetric Flow Rate Always Shows 0.0 mm³/s
**Symptom**: In Mainsail/Fluidd, the "Flow" display shows "0.0 mm³/s" and never updates, especially after the first toolchange.

**Cause**: The `motion_report` module had a hardcoded check for extruder axis index 4, which doesn't work correctly with toolchangers where different extruders activate dynamically.

**Fix**: The monkey patch queries the currently active extruder and tracks its velocity correctly.

## Installation

Add this to your `printer.cfg`:

```ini
[toolchanger_flow_fix]
```

Then restart Klipper:

```bash
sudo systemctl restart klipper
```

## Testing

1. **Test Volumetric Flow Display**:
   - Watch the "Flow" indicator in Mainsail during printing
   - Should show actual mm³/s values (not 0.0)
   - Should continue updating after toolchanges

2. **Test Filament Usage Tracking**:
   - Verify "Filament used" continues incrementing after toolchanges
   - Verify print time estimates remain visible

## How It Works

The module uses runtime monkey-patching to override:

- **`motion_report.get_status()`**: Enhanced to query the currently active extruder's velocity instead of using a hardcoded axis index

The patch is applied at `klippy:connect` time, after all modules are loaded.

## Compatibility

- Works with standard Klipper toolchanger configurations
- Compatible with AFC (Armored Turtle Filament Changer)
- Safe to use - can be disabled by simply commenting out `[toolchanger_flow_fix]`

## Removal

To remove the fix:

1. Comment out or delete `[toolchanger_flow_fix]` from `printer.cfg`
2. Restart Klipper

## Future

This is a temporary fix for testing. If proven stable, these changes should be merged into core Klipper's `gcode_move.py` and `motion_report.py`.
