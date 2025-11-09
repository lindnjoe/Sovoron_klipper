# Toolchanger Flow Fix

This module fixes flow rate tracking issues with toolchanger configurations.

## Problems Fixed

### 1. Flow Rate Percentage Reset (M221)
**Symptom**: When you set flow rate to something other than 100% (e.g., `M221 S95`), it resets to 100% after the first toolchange.

**Cause**: The `ACTIVATE_EXTRUDER` command (called during toolchanges) was resetting `extrude_factor` to 1.0.

**Fix**: The monkey patch preserves the user's flow rate setting across toolchanges.

### 2. Volumetric Flow Rate Always Shows 0.0 mm³/s
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

1. **Test Flow Rate Percentage Preservation**:
   - Start a print with multiple tools
   - Set flow rate: `M221 S95`
   - Perform a toolchange
   - Verify flow rate is still 95% in Mainsail (not reset to 100%)

2. **Test Volumetric Flow Display**:
   - Watch the "Flow" indicator in Mainsail during printing
   - Should show actual mm³/s values (not 0.0)
   - Should continue updating after toolchanges

3. **Test Filament Usage Tracking**:
   - Verify "Filament used" continues incrementing after toolchanges
   - Verify print time estimates remain visible

## How It Works

The module uses runtime monkey-patching to override two methods:

1. **`gcode_move._handle_activate_extruder()`**: Modified to skip resetting `extrude_factor`
2. **`motion_report.get_status()`**: Enhanced to query active extruder velocity

These patches are applied at `klippy:connect` time, after all modules are loaded.

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
