# Toolchanger Flow Fix

This module fixes flow rate and filament tracking issues with toolchanger configurations.

## Problems Fixed

### 1. Volumetric Flow Rate Always Shows 0.0 mm³/s
**Symptom**: In Mainsail/Fluidd, the "Flow" display shows "0.0 mm³/s" and never updates, especially after the first toolchange.

**Cause**: The `motion_report` module had a hardcoded check for extruder axis index 4, which doesn't work correctly with toolchangers where different extruders activate dynamically.

**Fix**: The monkey patch queries the currently active extruder and tracks its velocity correctly.

### 2. AFC Lane Changes Cause Negative Filament Tracking
**Symptom**: With AFC (Armored Turtle Filament Changer), filament usage tracking goes negative after OpenAMS lane changes. For example, tracking shows 241mm used, then after a lane change drops to -9mm.

**Cause**: AFC's `save_pos()` and `restore_pos()` methods perform large retracts and loads during filament swaps (e.g., 250mm retract to unload), but `print_stats` continues tracking these toolchange moves as if they were print moves, causing the filament_used counter to go wildly negative.

**Fix**: The monkey patch updates `print_stats.last_epos` (the E position tracker) before and after AFC operations, effectively "skipping over" the toolchange moves without affecting the print state. This prevents toolchange retracts/loads from being counted as filament usage.

## Installation

Add this to your `printer.cfg`:

```ini
[toolchanger_flow_fix]
debug: False  # Set to True for detailed logging during troubleshooting
```

Then restart Klipper:

```bash
sudo systemctl restart klipper
```

### Debug Mode

When `debug: True` is enabled, the module logs detailed information about:
- Extruder velocity tracking updates (every 50th call)
- Filament usage changes with delta values
- AFC save_pos/restore_pos extrude_factor values
- Trapq position lookups and any issues

Use the `FLOW_FIX_STATUS` G-code command to check current status at any time.

## Testing

1. **Test Volumetric Flow Display**:
   - Watch the "Flow" indicator in Mainsail during printing
   - Should show actual mm³/s values (not 0.0)
   - Should continue updating after toolchanges

2. **Test Filament Usage Tracking**:
   - Verify "Filament used" continues incrementing after toolchanges
   - Verify print time estimates remain visible
   - **For AFC users**: Verify filament usage stays positive after OpenAMS lane changes
   - Use `FLOW_FIX_STATUS` command before/after lane changes to monitor

3. **Verify Logs** (with debug enabled):
   - Look for "AFC save_pos() - updated last_epos" during lane changes
   - Look for "AFC restore_pos() - updated last_epos" after lane changes
   - Confirm no negative filament_used values in logs
   - Verify print state stays "printing" (not "paused") during lane changes

## How It Works

The module uses runtime monkey-patching to override several methods:

1. **`motion_report.get_status()`**: Enhanced to query the currently active extruder's velocity instead of using a hardcoded axis index (fixes timing bug where check happened after updating next_status_time)
2. **`print_stats._update_filament_usage()`**: Adds debugging to track filament usage changes
3. **`print_stats._handle_activate_extruder()`**: Adds debugging to track E position resets
4. **`AFC.save_pos()`**: Updates `print_stats.last_epos` to current E position before toolchange moves
5. **`AFC.restore_pos()`**: Updates `print_stats.last_epos` to current E position after toolchange moves, effectively skipping toolchange retracts/loads in filament tracking

These patches are applied at `klippy:connect` time, after all modules are loaded. The AFC patches are only applied if AFC is detected.

## Compatibility

- Works with standard Klipper toolchanger configurations
- Compatible with AFC (Armored Turtle Filament Changer)
- Safe to use - can be disabled by simply commenting out `[toolchanger_flow_fix]`

## Removal

To remove the fix:

1. Comment out or delete `[toolchanger_flow_fix]` from `printer.cfg`
2. Restart Klipper

## Future

This is a temporary fix for testing. If proven stable:
- The `motion_report.py` patch should be merged into core Klipper
- The AFC patches may remain as AFC-specific enhancements
- Consider submitting pull requests to both Klipper and AFC projects

## Troubleshooting

If issues persist:
1. Enable debug mode: `debug: True` in config
2. Run `FLOW_FIX_STATUS` command during printing
3. Check klippy.log for detailed tracking information
4. Verify all patches applied successfully (look for "All patches applied successfully" in log)

Common issues:
- **Flow still shows 0.0**: Check that motion_report has trapq for active extruder (use FLOW_FIX_STATUS)
- **Negative filament**: Ensure AFC patches applied (check log for "AFC patches applied")
- **Module not loading**: Check printer.cfg syntax and restart Klipper
