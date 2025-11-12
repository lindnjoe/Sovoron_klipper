# AFC Toolchanger Bridge - Setup Guide

## Overview

The **AFC Toolchanger Bridge** seamlessly integrates AFC (Armored Filament Changer) with klipper-toolchanger-easy, eliminating the need for manual tool change coordination in your macros.

### What It Does

- **Auto-discovers** lane → extruder → tool relationships
- **Smart tool changes** - only changes tools when extruder differs
- **Dock/pickup support** for same-tool lane swaps (OpenAMS safety)
- **Tool verification** via detection pins
- **Configurable purge timing** - before or after tool pickup

---

## Architecture

```
┌─────────────────────────────────────┐
│  AFC (Filament Management)          │
│  - Lane routing                     │
│  - OpenAMS control                  │
│  - Spoolman integration             │
└──────────────┬──────────────────────┘
               │
               │ calls bridge
               ▼
┌─────────────────────────────────────┐
│  AFC_toolchanger.py (Bridge)        │
│  - Auto-discovery                   │
│  - Tool change detection            │
│  - Dock/pickup coordination         │
│  - Offset management                │
└──────────────┬──────────────────────┘
               │
               │ calls toolchanger
               ▼
┌─────────────────────────────────────┐
│  Toolchanger (Physical Tools)       │
│  - Tool detection                   │
│  - Dock/undock paths                │
│  - Offset management                │
│  - Verification                     │
└─────────────────────────────────────┘
```

---

## Installation

### Step 1: Copy Files

```bash
# Copy bridge module to Klipper extras
cp /path/to/AFC_toolchanger.py ~/klipper/klippy/extras/

# Copy example configs to your AFC config directory
cp AFC_toolchanger_bridge.cfg ~/printer_data/config/AFC/
cp AFC_toolchanger_macros_updated.cfg ~/printer_data/config/AFC/
```

### Step 2: Include in printer.cfg

Add to your `printer.cfg` or main config file:

```ini
[include AFC/AFC_toolchanger_bridge.cfg]
```

### Step 3: Configure Bridge Settings

Edit `AFC/AFC_toolchanger_bridge.cfg`:

**For Standard Toolchanger:**
```ini
[AFC_toolchanger_bridge]
enable: True
auto_tool_change: True
dock_when_swap: False           # No need to dock for lane swaps
verify_tool_mounted: True
verify_timeout: 1.0
```

**For OpenAMS on Toolheads:**
```ini
[AFC_toolchanger_bridge]
enable: True
auto_tool_change: True
dock_when_swap: True            # Dock tool before loading lanes
purge_before_pickup: True       # Purge while tool is docked
auto_purge: True                # Let bridge handle purge
purge_command: LOAD_NOZZLE      # Your purge macro
verify_tool_mounted: True
verify_timeout: 1.0
```

### Step 4: Update Your Macros

Update your `_TX1`, `_TX2`, etc. macros to use bridge helper commands:

**Before:**
```gcode
[gcode_macro _TX1]
gcode:
    {% set GROUP = params.GROUP|default("T0")|string %}

    # Manual tool selection
    SELECT_TOOL T=4

    # Temperature & load logic
    M109 S240
    LOAD_NOZZLE
```

**After:**
```gcode
[gcode_macro _TX1]
gcode:
    {% set GROUP = params.GROUP|default("T0")|string %}
    {% set LANE = "lane" ~ GROUP[1:] %}

    # Bridge handles tool preparation
    AFC_BRIDGE_PREPARE_LANE LANE={LANE}

    # Temperature & load logic (unchanged)
    M109 S240
    LOAD_NOZZLE

    # Bridge handles finalization
    AFC_BRIDGE_LANE_LOADED LANE={LANE}
```

### Step 5: Restart Klipper

```gcode
FIRMWARE_RESTART
```

---

## Configuration Examples

### Example 1: Single Extruder, Multiple Lanes

```ini
# Your existing configs
[tool T0]
extruder: extruder
gcode_x_offset: 0
gcode_y_offset: 0
gcode_z_offset: 0

[AFC_lane lane0]
extruder: extruder
# ... other AFC settings

[AFC_lane lane1]
extruder: extruder
# ... other AFC settings
```

**Bridge automatically discovers:**
- `lane0` → `extruder` → `T0`
- `lane1` → `extruder` → `T0`
- Swapping `lane0↔lane1` = NO tool change (same extruder)

---

### Example 2: Multiple Tools, One Lane Each

```ini
[tool T0]
extruder: extruder
# ... offsets

[tool T4]
extruder: extruder4
# ... offsets

[AFC_lane lane0]
extruder: extruder

[AFC_lane lane4]
extruder: extruder4
```

**Bridge automatically discovers:**
- `lane0` → `extruder` → `T0`
- `lane4` → `extruder4` → `T4`
- Swapping `lane0→lane4` = NEEDS tool change (different extruders)

---

### Example 3: Your Setup (Multiple Lanes per Tool)

```ini
[tool T4]
extruder: extruder4
gcode_x_offset: 1.071
gcode_y_offset: -1.353
gcode_z_offset: -0.371

[AFC_lane lane4]
extruder: extruder4

[AFC_lane lane6]
extruder: extruder4

[AFC_lane lane7]
extruder: extruder4

[AFC_lane lane8]
extruder: extruder4
```

**Bridge automatically discovers:**
- All 4 lanes → `extruder4` → `T4`
- Swapping between any of these lanes = NO tool change
- But if `dock_when_swap=True`, tool docks before load (OpenAMS safety)

---

## Usage

### Diagnostic Commands

```gcode
# Show bridge status and all mappings
AFC_BRIDGE_STATUS

# Check which tool a lane uses
AFC_BRIDGE_GET_TOOL LANE=lane4

# Output:
# Lane lane4: Tool=T4 (T4), Extruder=extruder4
```

### Manual Lane Change (Future Option)

```gcode
# Complete lane change handled by bridge
AFC_CHANGE_LANE FROM=lane4 TO=lane6
```

---

## Behavior Examples

### Scenario 1: Same-Tool Lane Swap (dock_when_swap=False)

```gcode
User: Load lane4 → lane6 (both extruder4/T4)

Bridge:
✓ Check: Same extruder? YES
✓ Check: dock_when_swap? NO
→ Action: Skip tool change, just swap filament

Result: Direct lane swap, tool stays mounted
```

---

### Scenario 2: Same-Tool Lane Swap (dock_when_swap=True)

```gcode
User: Load lane4 → lane6 (both extruder4/T4)

Bridge:
✓ Check: Same extruder? YES
✓ Check: dock_when_swap? YES
→ Action: Dock tool before load

Sequence (without cutting):
1. Unload lane4
2. UNSELECT_TOOL (dock T4)
3. Load lane6 into extruder4
4. Purge (if purge_before_pickup=True)
5. SELECT_TOOL T=4 (pickup T4)
6. Purge (if purge_before_pickup=False)

Sequence (with tool_cut=True):
1. Unload lane4
2. AFC_CUT (severs filament at toolhead)
3. UNSELECT_TOOL (dock T4)
4. Continue unloading from hub/buffer
5. Load lane6 into hub/buffer and extruder4
6. Purge (if purge_before_pickup=True)
7. SELECT_TOOL T=4 (pickup T4)
8. Purge (if purge_before_pickup=False)

Result: Safe lane swap with tool out of the way (cleaner with cutting)
```

---

### Scenario 3: Different Tool Change

```gcode
User: Load lane4 (extruder4/T4) → lane0 (extruder/T0)

Bridge:
✓ Check: Same extruder? NO
✓ Check: Tool change needed? YES
→ Action: SELECT_TOOL T=0 (dock T4, pickup T0)

Sequence:
1. Unload lane4 (via your macro - optional)
2. AFC_BRIDGE_PREPARE_LANE:
   - Dock T4 (filament STAYS in extruder4)
   - Pickup T0
   - Verify T0 mounted
3. Load lane0 into extruder
4. Apply tool offsets (automatic via toolchanger)

IMPORTANT: Filament remains in each tool's extruder when docked!
- T4 docked with lane4 still in extruder4
- T0 picked up ready to load lane0 into extruder

Result: Complete tool change without unloading previous tool
```

---

## Troubleshooting

### Bridge not initializing

**Check klippy.log for:**
```
AFC_toolchanger_bridge: AFC not found
AFC_toolchanger_bridge: Toolchanger not found
```

**Solution:** Ensure both `[AFC]` and `[toolchanger]` sections exist

---

### Lanes not mapping

**Run:**
```gcode
AFC_BRIDGE_STATUS
```

**Check output:**
```
Lane Mappings (0 lanes):
```

**Solution:**
1. Verify `[AFC_lane]` sections have `extruder:` defined
2. Verify `[tool TX]` sections have `extruder:` defined
3. Ensure extruder names match exactly

---

### Tool verification fails

**Error:**
```
AFC_toolchanger_bridge: Tool verification failed! Expected T4, detected T0
```

**Solutions:**
1. Check detection_pin wiring
2. Increase `verify_timeout: 2.0`
3. Check `[tool T4]` has `detection_pin:` defined
4. Verify toolchanger initialization fix is applied

---

### Tool doesn't dock when expected

**Check:**
```ini
[AFC_toolchanger_bridge]
dock_when_swap: True  # Must be True for same-tool docking
```

**Verify:**
```gcode
AFC_BRIDGE_STATUS

# Should show:
# Dock When Swap: True
```

---

## Migration Path

### Phase 1: Helper Commands (Recommended Start)

Use `AFC_BRIDGE_PREPARE_LANE` and `AFC_BRIDGE_LANE_LOADED` in your existing macros:

```gcode
[gcode_macro _TX1]
gcode:
    AFC_BRIDGE_PREPARE_LANE LANE={LANE}
    # Your existing logic
    AFC_BRIDGE_LANE_LOADED LANE={LANE}
```

**Benefits:**
- Minimal changes to existing macros
- Easy to revert if issues
- Can keep custom temperature logic

---

### Phase 2: Full Automation (Future)

Let bridge handle everything:

```gcode
AFC_CHANGE_LANE FROM=lane4 TO=lane6
```

**Benefits:**
- Simplest possible macro
- Bridge handles all coordination
- Consistent behavior across all tools

---

## Advanced Configuration

### Custom Purge Commands

```ini
[AFC_toolchanger_bridge]
auto_purge: True
purge_command: MY_CUSTOM_PURGE_MACRO
purge_before_pickup: True
```

### Speed Overrides

```ini
[AFC_toolchanger_bridge]
tool_change_speed: 60000  # Faster tool changes
path_speed: 3000          # Faster dock/pickup
```

### Disable Verification for Testing

```ini
[AFC_toolchanger_bridge]
verify_tool_mounted: False  # Skip detection verification
```

---

## Testing Checklist

### ✅ Basic Tests

- [ ] `FIRMWARE_RESTART` completes without errors
- [ ] `AFC_BRIDGE_STATUS` shows all lanes mapped
- [ ] `AFC_BRIDGE_GET_TOOL LANE=lane4` shows correct tool

### ✅ Same-Tool Swap (dock_when_swap=False)

- [ ] Load lane4 → lane6 (same extruder)
- [ ] Tool does NOT change
- [ ] Filament swaps directly

### ✅ Same-Tool Swap (dock_when_swap=True)

- [ ] Load lane4 → lane6 (same extruder)
- [ ] Tool docks before load
- [ ] Tool picks up after load
- [ ] Purge timing matches config

### ✅ Different-Tool Change

- [ ] Load lane4 (extruder4) → lane0 (extruder)
- [ ] Tool change occurs automatically
- [ ] Tool offsets applied
- [ ] Detection verification passes

### ✅ Startup Detection

- [ ] Reboot with tool mounted
- [ ] Check klippy.log: "Detected tool at startup: T4"
- [ ] `AFC_BRIDGE_STATUS` shows correct active tool

---

## Support & Debugging

### Enable Debug Logging

Check `~/printer_data/logs/klippy.log` for bridge messages:

```
AFC_toolchanger_bridge: Mapped extruder4 → Tool T4 (T4)
AFC_toolchanger_bridge: Lane lane4 → T4 (T4)
AFC_toolchanger_bridge: Same-tool swap, docking T4
AFC_toolchanger_bridge: Purging before pickup (tool docked)
AFC_toolchanger_bridge: Picking up T4 after purge
```

### Get Status

```gcode
AFC_BRIDGE_STATUS
```

Shows:
- Current configuration
- Active/detected tools
- All lane mappings
- Current state

---

## Benefits Summary

### Before Bridge

❌ Manual tool selection in macros
❌ Duplicate extruder configuration
❌ Complex macro chains
❌ State synchronization issues
❌ No automatic verification

### After Bridge

✅ Automatic tool change detection
✅ Single source of configuration
✅ Simplified macros
✅ Automatic state management
✅ Built-in tool verification
✅ OpenAMS-specific docking support
✅ Configurable purge timing

---

## Next Steps

1. **Install** the bridge module and config
2. **Configure** your specific setup (OpenAMS vs standard)
3. **Update** your macros to use helper commands
4. **Test** with each scenario
5. **Monitor** klippy.log for any issues
6. **Optimize** speeds and timeouts as needed

---

## Contributing

Found a bug or have a feature request?
- Check klippy.log for error messages
- Run `AFC_BRIDGE_STATUS` to capture state
- Report issues with your config and log output

---

## License

This module integrates:
- AFC (Armored Turtle) - GPL-3.0
- klipper-toolchanger-easy (Viesturs Zarins) - GPL-3.0

Bridge module follows GPL-3.0 license.
