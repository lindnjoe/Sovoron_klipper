# Toolchanger Flow Fix - Quick Start Guide

## What This Fixes

If you have a Klipper toolchanger and experience any of these issues:

❌ Flow display shows "0.0 mm³/s" and never updates
❌ Filament usage tracking stops after toolchanges
❌ Print time estimates blank out after toolchanges
❌ **(AFC/OpenAMS only)** Filament usage goes negative after lane changes

✅ **This module fixes all of them!**

---

## Installation (30 seconds)

1. **Add to your `printer.cfg`:**
   ```ini
   [toolchanger_flow_fix]
   ```

2. **Restart Klipper:**
   ```bash
   sudo systemctl restart klipper
   ```

3. **Verify it loaded:**
   Check `klippy.log` for:
   ```
   toolchanger_flow_fix: All patches applied successfully
   ```

**Done!** The fix is now active.

---

## Quick Test

Run this command from the console:
```gcode
FLOW_FIX_STATUS
```

You should see:
- Active extruder name
- Live extruder velocity (should be non-zero during extrusion)
- Filament used (should increment continuously)
- Print stats and timing info

---

## Does It Work?

### During Your Next Print

Watch these in Mainsail/Fluidd:

1. **Flow (mm³/s)**: Should show values like 5.2, 8.7, etc.
   - ✅ Updates during extrusion
   - ✅ Continues working after toolchanges
   - ❌ Should NOT show 0.0 during active printing

2. **Filament used (mm)**: Should count up continuously
   - ✅ Keeps incrementing after toolchanges
   - ✅ Never resets to 0
   - ✅ Never goes negative (even with AFC lane changes!)

3. **Time estimates**: Should stay visible
   - ✅ Don't blank out after toolchanges

---

## Troubleshooting

### If Flow Still Shows 0.0

Add debug mode:
```ini
[toolchanger_flow_fix]
debug: True
```

Restart Klipper, then check `klippy.log` for detailed output.

### If Filament Tracking Still Breaks

Run `FLOW_FIX_STATUS` before and after a toolchange. Check if:
- Extrude factor is preserved (should stay at current value)
- Print stats state is correct
- Trapq is found for the active extruder

---

## Testing Macros (Optional)

For comprehensive testing, copy macros from `toolchanger_flow_fix.MACROS.cfg` to your `printer.cfg`, then run:

```gcode
TEST_FLOW_VELOCITY        # Test flow display
TEST_FILAMENT_TRACKING    # Test usage tracking
TEST_AFC_LANE_CHANGE      # Test AFC lane changes (if you have AFC)
```

---

## Documentation

- **README.md** - Detailed documentation and how it works
- **TESTING.md** - Comprehensive testing procedures
- **MACROS.cfg** - Ready-to-use test macros
- **SUMMARY.md** - Technical implementation details

---

## Need Help?

1. Enable debug: `debug: True`
2. Run `FLOW_FIX_STATUS`
3. Check klippy.log
4. See TESTING.md for detailed troubleshooting

---

## What's Happening Behind the Scenes?

The module patches three Klipper components:

1. **Motion Report** - Tracks active extruder velocity dynamically
2. **GCode Move** - Preserves extrude_factor across toolchanges
3. **AFC (if present)** - Pauses filament tracking during lane changes

All done via runtime patching - no Klipper files are modified!

---

## That's It!

The fix is transparent and automatic. Just add the config line, restart, and enjoy working flow displays and filament tracking on your toolchanger.

**Happy printing!** 🎉

---

*For detailed information, see the other documentation files in this directory.*
