# AFC M109 Deadband Override Module

## Overview

This module provides a toggleable override for AFC's M109 macro to automatically use the deadband settings configured for each AFC extruder in `AFC_Hardware.cfg`.

## Problem Solved

AFC overrides the standard M109 temperature command at the Python level. While AFC's M109 supports a `D` parameter for deadband, this module eliminates the need to manually specify the deadband by automatically using the value configured for each extruder.

## Features

- **Toggleable**: Can be enabled or disabled on the fly
- **Automatic Deadband**: Uses the deadband value from each AFC extruder's configuration
- **Per-Extruder Settings**: Each extruder can have its own deadband value
- **Non-Invasive**: Doesn't modify AFC Python code, works as a macro layer

## Installation

The module is already installed and included in your `printer.cfg`:

```ini
[include AFC_M109_deadband_override.cfg]
```

## Usage

### Enable/Disable Commands

```gcode
AFC_M109_DEADBAND_ENABLE   # Enable automatic deadband usage (default)
AFC_M109_DEADBAND_DISABLE  # Disable automatic deadband usage
AFC_M109_DEADBAND_STATUS   # Show current status
```

### How It Works

**When ENABLED (default):**
```gcode
M109 S240 T0  # Automatically uses the deadband from AFC extruder config
# Equivalent to: AFC_M109 S240 T0 D=10
```

**When DISABLED:**
```gcode
M109 S240 T0  # Uses AFC's default behavior (no deadband unless specified)
# Equivalent to: AFC_M109 S240 T0
```

### Setting Deadband Values

Edit your deadband values in `AFC_Hardware.cfg`:

```ini
[AFC_extruder extruder]
deadband: 10  # Temperature tolerance in degrees Celsius
tool: tool T0

[AFC_extruder extruder1]
deadband: 15  # Different extruders can have different values
tool: tool T1
```

## How Deadband Works

A deadband creates a temperature tolerance range:
- Target: 240°C
- Deadband: 10°C
- Acceptable range: 235°C to 245°C (target ± deadband/2)

The printer will consider the target temperature reached when it enters this range, reducing wait time.

## Configuration Files

- **Main Module**: `AFC_M109_deadband_override.cfg`
- **AFC Hardware Config**: `AFC/AFC_Hardware.cfg`
- **Documentation**: `AFC_M109_DEADBAND_README.md`

## Technical Details

### Macro Override Chain

1. **Original M109** → Renamed to `M109.AFC_BASE`
2. **AFC M109** → Available as `AFC_M109` command
3. **This Module** → Intercepts M109, adds deadband, calls `AFC_M109`

### Deadband Parameter Flow

```
M109 S240 T0
    ↓
[Check if override enabled]
    ↓
[Lookup AFC extruder config for T0]
    ↓
[Get deadband value (e.g., 10)]
    ↓
AFC_M109 S240 T0 D=10
```

## Default State

The module is **ENABLED** by default. To change the default, edit:

```ini
[gcode_macro _AFC_M109_DEADBAND_OVERRIDE]
variable_enabled: 1  # Change to 0 to default to disabled
```

## Troubleshooting

### Issue: M109 not using deadband
**Solution**: Check that the override is enabled with `AFC_M109_DEADBAND_STATUS`

### Issue: Deadband value not found
**Solution**: Verify that the AFC extruder section exists in `AFC_Hardware.cfg` with a `deadband:` parameter

### Issue: Wrong extruder deadband used
**Solution**: Check that the tool number matches the extruder name (T0 → extruder, T1 → extruder1, etc.)

## Examples

```gcode
# Enable the override
AFC_M109_DEADBAND_ENABLE

# Heat extruder 0 to 240°C with automatic deadband
M109 S240 T0

# Heat current extruder to 210°C with automatic deadband
M109 S210

# Disable the override if you want AFC's default behavior
AFC_M109_DEADBAND_DISABLE

# Heat without deadband (AFC default)
M109 S240 T0

# Re-enable for future commands
AFC_M109_DEADBAND_ENABLE
```

## Notes

- The module respects all standard M109 parameters (S, T)
- If a deadband is not found in the AFC config, it falls back to AFC's default behavior
- Informational messages are logged to help track which deadband is being used
- The module works seamlessly with tool changes and multi-extruder setups

## Version

Created: 2025-11-08
Compatible with: AFC Klipper Add-On
