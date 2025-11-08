# AFC M109 Deadband Override Module

## Overview

This module provides a toggleable Python-level override for AFC's M109 command to automatically use the deadband settings configured for each AFC extruder in `AFC_Hardware.cfg`.

## Problem Solved

AFC overrides the standard M109 temperature command at the Python level. While AFC's M109 supports a `D` parameter for deadband, you had to manually specify it every time. This module eliminates that need by automatically using the value configured for each extruder.

## How It Works (Technical)

This is a **Python module**, not a macro, because AFC itself overrides M109 at the Python level. The module:

1. Loads after AFC initializes
2. Renames AFC's M109 command to `M109_AFC_ORIGINAL`
3. Registers its own M109 command that:
   - Looks up the AFC extruder's deadband value
   - Adds the `D` parameter automatically
   - Calls `M109_AFC_ORIGINAL` with the deadband included

## Installation

The module is already installed:

1. **Python Module**: `/home/user/Sovoron_klipper/klipper/klippy/extras/AFC_M109_deadband.py`
2. **Configuration**: `AFC_M109_deadband_override.cfg`
3. **Include**: Already added to `printer.cfg`

After making changes, restart Klipper with:
```
FIRMWARE_RESTART
```

## Features

- **Toggleable**: Can be enabled or disabled on the fly without restarting
- **Automatic Deadband**: Uses the deadband value from each AFC extruder's configuration
- **Per-Extruder Settings**: Each extruder can have its own deadband value
- **Manual Override**: You can still manually specify `D` parameter if needed
- **Non-Invasive**: Doesn't modify AFC files, works as a separate module

## Usage Commands

```gcode
AFC_M109_DEADBAND_ENABLE   # Enable automatic deadband usage (default)
AFC_M109_DEADBAND_DISABLE  # Disable automatic deadband usage
AFC_M109_DEADBAND_STATUS   # Show current status
```

## How It Works (User Perspective)

### When ENABLED (default):
```gcode
M109 S240 T0  # Automatically uses the deadband from AFC config
```
Behind the scenes: `M109_AFC_ORIGINAL S240 T0 D=10`

### When DISABLED:
```gcode
M109 S240 T0  # Uses AFC's default behavior
```
Behind the scenes: `M109_AFC_ORIGINAL S240 T0`

### Manual Override (works either way):
```gcode
M109 S240 T0 D=5  # Uses 5°C deadband instead of config value
```

## Setting Deadband Values

Edit your deadband values in `AFC/AFC_Hardware.cfg`:

```ini
[AFC_extruder extruder]
deadband: 10  # Temperature tolerance in degrees Celsius
tool: tool T0

[AFC_extruder extruder1]
deadband: 15  # Different extruders can have different values
tool: tool T1
```

**Your Current Settings**: All extruders are set to 10°C deadband.

## How Deadband Works

A deadband creates a temperature tolerance range:
- **Target**: 240°C
- **Deadband**: 10°C
- **Acceptable range**: 235°C to 245°C (target ± deadband/2)

The printer will consider the target temperature reached when it enters this range, significantly reducing wait time.

## Configuration Options

In `AFC_M109_deadband_override.cfg`:

```ini
[AFC_M109_deadband]
enabled: 1  # 1 = start enabled (default), 0 = start disabled
```

## Architecture

### Command Flow

```
User: M109 S240 T0
  ↓
[AFC_M109_deadband.py intercepts]
  ↓
[Check if enabled]
  ↓
[Lookup AFC extruder config for T0]
  ↓
[Get deadband value: 10]
  ↓
[Call M109_AFC_ORIGINAL S240 T0 D=10]
  ↓
[AFC's original M109 handler executes]
```

### File Structure

```
Sovoron_klipper/
├── klipper/klippy/extras/
│   └── AFC_M109_deadband.py          # Python module
├── printer_data/config/
│   ├── AFC_M109_deadband_override.cfg # Configuration
│   ├── AFC_M109_DEADBAND_README.md    # This file
│   └── printer.cfg                    # Includes the config
```

## Troubleshooting

### Error: "AFC_M109_deadband requires AFC to be loaded"
**Cause**: The module tried to load before AFC
**Solution**: Ensure AFC config is included before this module in `printer.cfg`

### M109 not using deadband
**Check**: Run `AFC_M109_DEADBAND_STATUS` to verify it's enabled
**Solution**: Run `AFC_M109_DEADBAND_ENABLE`

### Deadband value not found
**Check**: Verify AFC extruder section in `AFC/AFC_Hardware.cfg`
**Solution**: Add `deadband: 10` to each `[AFC_extruder ...]` section

### Wrong extruder deadband used
**Check**: Tool number mapping (T0 → extruder, T1 → extruder1, etc.)
**Solution**: Verify tool mapping in AFC configuration

## Examples

```gcode
# Check current status
AFC_M109_DEADBAND_STATUS

# Enable the override (if not already enabled)
AFC_M109_DEADBAND_ENABLE

# Heat extruder 0 to 240°C - automatically uses 10°C deadband
M109 S240 T0
# Console shows: "M109: Using AFC deadband 10.0°C for extruder"

# Heat current extruder to 210°C with automatic deadband
M109 S210

# Temporarily disable for testing
AFC_M109_DEADBAND_DISABLE
M109 S240 T0  # Uses AFC default (no automatic deadband)

# Re-enable for normal operation
AFC_M109_DEADBAND_ENABLE

# Manual deadband override (works regardless of enable/disable status)
M109 S240 T0 D=5  # Uses 5°C instead of configured 10°C
```

## Default State

The module is **ENABLED by default**. To change the default, edit `AFC_M109_deadband_override.cfg`:

```ini
[AFC_M109_deadband]
enabled: 0  # Start disabled
```

Then run `FIRMWARE_RESTART` to apply.

## Benefits

1. **Faster Heating**: No more waiting for exact temperatures
2. **Consistent Behavior**: All M109 commands use your configured deadband
3. **Less Typing**: No need to add `D=10` to every M109 command
4. **Flexible**: Can still override deadband manually when needed
5. **Safe**: Original AFC functionality is preserved

## Notes

- The module works seamlessly with tool changes and multi-extruder setups
- If a deadband is not found in the AFC config, it falls back to AFC's default behavior
- Informational messages are logged to help track which deadband is being used
- The module loads after AFC, so AFC must be properly configured first
- You can disable the module without removing it by running `AFC_M109_DEADBAND_DISABLE`

## Version

- **Created**: 2025-11-08
- **Compatible with**: AFC Klipper Add-On (Python-level override)
- **Type**: Klipper Python module + configuration

## Upgrading Notes

If you update Klipper, the Python module at `/home/user/Sovoron_klipper/klipper/klippy/extras/AFC_M109_deadband.py` will be preserved. However, if you ever need to reinstall Klipper from scratch, you'll need to:

1. Restore `AFC_M109_deadband.py` to `klipper/klippy/extras/`
2. The config file `AFC_M109_deadband_override.cfg` is in your config directory and should be backed up normally

## Support

For issues or questions:
1. Check the Klipper console for error messages
2. Run `AFC_M109_DEADBAND_STATUS` to verify the module loaded
3. Verify AFC is working correctly first: `M109_AFC_ORIGINAL S240`
4. Check that deadband values are set in `AFC_Hardware.cfg`
