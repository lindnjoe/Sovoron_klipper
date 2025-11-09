# Testing Guide for Toolchanger Flow Fix

This guide provides detailed testing procedures and example macros to verify all three fixes are working correctly.

## Prerequisites

1. Add to `printer.cfg`:
   ```ini
   [toolchanger_flow_fix]
   debug: True  # Enable for testing
   ```

2. Restart Klipper: `sudo systemctl restart klipper`

3. Verify module loaded by checking klippy.log for:
   ```
   toolchanger_flow_fix: All patches applied successfully
   ```

## Test 1: Volumetric Flow Display (mm³/s)

### Expected Behavior
The "Flow" indicator in Mainsail/Fluidd should show actual mm³/s values during extrusion and continue updating after toolchanges.

### Test Procedure

1. Start a print with extrusion moves
2. Watch the "Flow" display in Mainsail - should show non-zero values (e.g., 5.2 mm³/s)
3. Perform a toolchange (T0 → T1)
4. Resume printing - Flow should continue showing correct values

### Verification Commands

```gcode
FLOW_FIX_STATUS
```

Look for:
- `Live Extruder Velocity: X.XXX mm/s` (should be non-zero during extrusion)
- `Trapq for extruderN: Found` (confirms velocity tracking is working)

### Expected Log Output (debug enabled)

```
toolchanger_flow_fix: Active extruder=extruder1, velocity=12.345 mm/s
```

## Test 2: AFC Lane Changes (OpenAMS)

### Expected Behavior
With AFC, OpenAMS lane changes should NOT cause filament usage to go negative. The large retracts/loads during filament swaps should be ignored.

### Test Procedure

**IMPORTANT**: This test requires AFC and OpenAMS to be configured and functional.

1. Start a print
2. Note the filament usage: Run `FLOW_FIX_STATUS` and record the value
3. Trigger an OpenAMS lane change (either manually or via G-code color change)
4. After lane change completes, run `FLOW_FIX_STATUS` again
5. Verify filament usage has NOT gone negative

### Example

```
Before lane change:
  Filament used: 241.109 mm

During lane change (check klippy.log):
  toolchanger_flow_fix: AFC save_pos() - updated last_epos
  [AFC performs 250mm retract and reload - NOT tracked]
  toolchanger_flow_fix: AFC restore_pos() - updated last_epos

After lane change:
  Filament used: 241.109 mm  ← Should be same or slightly higher, NEVER negative!
```

### Expected Log Output

```
toolchanger_flow_fix: AFC save_pos() - updated last_epos from 50.123 to 50.123
toolchanger_flow_fix: AFC save_pos() called, extrude_factor=1.000 saved
[... AFC toolchange moves occur but are NOT tracked ...]
toolchanger_flow_fix: AFC restore_pos() - updated last_epos from -199.877 to -199.877
toolchanger_flow_fix: AFC restore_pos() called, extrude_factor restored to 1.000
```

Note: The E positions may be negative after toolchange, but this is normal - the key is that `last_epos` is updated to match, so the next filament tracking calculation will have a delta of zero (skipping the toolchange moves).

### AFC Lane Change Test Macro

```ini
[gcode_macro TEST_AFC_LANE_CHANGE]
gcode:
    {% set current_lane = printer["AFC"].current %}

    # Print starting status
    FLOW_FIX_STATUS
    RESPOND MSG="Starting filament: {current_lane}"

    # Extrude some filament
    G92 E0
    G1 E100 F300
    M400

    # Check filament usage before lane change
    FLOW_FIX_STATUS

    # Trigger lane change (adjust lane IDs for your setup)
    # This assumes you have multiple lanes configured
    AFC_CHANGE_LANE LANE=leg1_lane2

    # Check filament usage after lane change
    # Should be same as before (or slightly higher if any print moves occurred)
    # Should NEVER be negative!
    FLOW_FIX_STATUS
```

## Monitoring During Real Prints

### Mainsail/Fluidd Dashboard

Watch these indicators during a multi-tool print:

1. **Flow** (mm³/s): Should show values like 5.2, 8.7, etc. during extrusion
   - Should update continuously
   - Should NOT show 0.0 during active extrusion
   - Should continue working after every toolchange

2. **Filament used** (mm): Should increment continuously
   - Should NEVER reset to 0 during toolchanges
   - Should NEVER go negative (especially with AFC lane changes)

3. **Print time estimates**: Should remain visible
   - Should NOT blank out after toolchanges

### Real-time Log Monitoring

Open a terminal and tail the Klipper log:

```bash
tail -f ~/printer_data/logs/klippy.log | grep toolchanger_flow_fix
```

During a print, you should see:
- Filament tracking updates (every change > 0.001mm)
- AFC last_epos updates during lane changes
- Periodic velocity updates (every 50th status call)

## Troubleshooting Failed Tests

### Flow shows 0.0 during extrusion

1. Run `FLOW_FIX_STATUS` and check:
   - `Trapq for extruderN: Found` should show "Found"
   - `Available trapqs:` should list all your extruders

2. Check klippy.log for:
   ```
   toolchanger_flow_fix: motion_report patch applied
   toolchanger_flow_fix: Active extruder=extruderN, velocity=X.XXX mm/s
   ```

3. If trapq is "NOT FOUND", there may be a timing issue - report this

### Filament usage goes negative with AFC

1. Check klippy.log for AFC patch messages:
   ```
   toolchanger_flow_fix: AFC patches applied
   ```

2. During lane change, verify you see:
   ```
   toolchanger_flow_fix: AFC save_pos() - updated last_epos
   toolchanger_flow_fix: AFC restore_pos() - updated last_epos
   ```

3. If AFC patches not applied, verify AFC module loaded before this module

4. Verify print state stays "printing" during lane changes (not "paused")

## Performance Impact

The monkey patches have minimal performance impact:
- Motion report: Only updates when status refresh is due (every 250ms)
- Filament tracking: Only logs when changes > 0.001mm
- Velocity tracking: Logged every 50th call (reduces log spam)

During normal printing, expect < 1% CPU overhead from this module.

## Reporting Issues

If any test fails, collect:
1. Output of `FLOW_FIX_STATUS` command
2. Relevant section of klippy.log (with debug: True)
3. Your printer.cfg `[toolchanger_flow_fix]` section
4. AFC version (if applicable)
5. Klipper version

Include this information when reporting issues.
