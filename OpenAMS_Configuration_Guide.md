# OpenAMS Configuration Guide

This guide provides a comprehensive overview of all OpenAMS configuration settings for AFC (Automatic Filament Changer) integration.

---

## Table of Contents

1. [Hardware Configuration (AFC_oams.cfg)](#hardware-configuration)
2. [Macro Configuration (AFC_oams_macros.cfg)](#macro-configuration)
3. [Quick Start Checklist](#quick-start-checklist)
4. [Troubleshooting](#troubleshooting)

---

## Hardware Configuration

**File**: `AFC_oams.cfg`

This file configures the physical OpenAMS hardware including MCUs, sensors, and mechanical parameters.

### MCU Configuration

#### FPS MCU
```ini
[mcu fps]
canbus_uuid: a839e785afe7
```
- **canbus_uuid**: CAN bus identifier for the FPS (Filament Path Sensor) board
- **How to find**: Run `~/klippy-env/bin/python ~/klipper/scripts/canbus_query.py can0`

#### OAMS MCU
```ini
[mcu oams_mcu1]
canbus_uuid: 7564b5192d2f
```
- **canbus_uuid**: CAN bus identifier for the OAMS (OpenAMS) control board
- Each OAMS unit requires its own MCU section

---

### OAMS Unit Configuration

**Section**: `[oams oams1]`

Each OpenAMS unit requires a configuration section. Multiple units can be configured (oams1, oams2, etc.).

#### Retry Behavior
```ini
load_retry_max: 3              # Maximum load attempts
unload_retry_max: 2            # Maximum unload attempts
retry_backoff_base: 1.0        # Base delay in seconds (1s, 2s, 4s, ...)
retry_backoff_max: 5.0         # Maximum delay between retries
auto_unload_on_failed_load: True   # Automatically unload before retry
```

**What these do**:
- `load_retry_max`: If a load fails, retry up to this many times before giving up
- `unload_retry_max`: If an unload fails, retry up to this many times
- `retry_backoff_base`: Starting delay between retries (doubles each retry: 1s, 2s, 4s...)
- `retry_backoff_max`: Maximum wait time between retries (caps exponential backoff)
- `auto_unload_on_failed_load`: When True, automatically unloads filament before retrying a failed load

**Recommended values**:
- Leave at defaults unless experiencing frequent load/unload failures
- Increase `load_retry_max` if you have longer PTFE tubes or more friction

#### FPS Thresholds

```ini
fps_upper_threshold: 0.7  # Hub motor runs full speed above this
fps_lower_threshold: 0.3  # Hub motor stops below this
```

**What these do**:
- The FPS (Filament Path Sensor) slide has a Hall Effect Sensor (HES) that ranges from 0.0 to 1.0
- The average of these thresholds creates a **setpoint** for the PID loop
- In this example: (0.7 + 0.3) / 2 = 0.5
- The hub motor speed is continuously adjusted to maintain filament pressure at this setpoint

**How to tune**:
- **Wider range (e.g., 0.8/0.2)**: More aggressive response, faster but less stable
- **Narrower range (e.g., 0.6/0.4)**: Smoother operation, more stable
- Default values work for most setups

#### FPS Reversal

```ini
fps_is_reversed: true
```

**CRITICAL SETTING** - Must be set correctly for proper operation!

**How to verify**:
1. With printer on and filament **unloaded**
2. Check `klippy.log`: `tail -f ~/printer_data/logs/klippy.log`
3. Look for FPS value in the log
4. **Correct**: FPS value should be close to **0.0** when unloaded
5. **Incorrect**: If FPS value is close to **1.0**, set `fps_is_reversed: false`

#### First Stage Feeder (F1S) Detection

```ini
f1s_hes_on: 0.1, 0.1, 0.1, 0.1
f1s_hes_is_above: false
```

**What these do**:
- `f1s_hes_on`: HES threshold values for each of the 4 spool bays (0.0 to 1.0)
- `f1s_hes_is_above`: Direction of threshold detection
  - `false`: Filament detected when value is **below** threshold
  - `true`: Filament detected when value is **above** threshold

**How to tune**:
- Default values (0.1) work for most setups
- If spool detection is unreliable, adjust these values
- All 4 bays can have different values if needed

#### Hub Motor Detection

```ini
hub_hes_on: 0.824217, 0.809127, 0.837821, 0.813065
hub_hes_is_above: true
```

**IMPORTANT**: These values must be calibrated!

**What these do**:
- Threshold values for detecting filament at the hub motor for each bay
- Used to determine when filament has been successfully fed from spool to hub

**How to calibrate**:
1. Run `OAMS_CALIBRATE_HUB_HES` command
2. Follow on-screen instructions to load/unload each bay
3. Copy the calibrated values from the output
4. Paste into your config
5. Restart Klipper

#### PTFE Length

```ini
ptfe_length: 2001  # CRITICAL - Measure your actual PTFE length!
```

**CRITICAL SETTING** - Must match your machine!

**What this does**:
- Total PTFE tube length from OAMS hub to toolhead (in mm)
- When OAMS has fed `ptfe_length - 100mm`, it slows down and checks FPS
- If FPS slide is above `fps_upper_threshold`, loading stops

**How to measure**:
1. Measure the actual PTFE tube length on your machine
2. Include any curved sections (measure along the path, not straight line)
3. Add ~10-20mm margin for accuracy
4. Update this value

**Symptoms of incorrect value**:
- **Too short**: Filament stops before reaching toolhead
- **Too long**: Filament grinds against extruder gears

#### Rewind PID Control

```ini
current_target: 0.30
current_kp: 3.0
current_ki: 0.0
current_kd: 0.0
```

**What these do**:
- Control spool rewind behavior during unload
- `current_target`: Target DC motor current (0.0 to 1.0) during rewind
- `current_kp/ki/kd`: PID tuning parameters for rewind control

**When to change**:
- **Leave at defaults** unless you have issues with:
  - Filament tangling during rewind
  - Spool overwinding
  - Inconsistent rewind tension

#### OAMS Index

```ini
oams_idx: 1
```

**What this does**:
- Unique identifier for this OAMS unit (use 1, 2, 3, etc. for multiple units)
- Used in logging and debugging to distinguish between units
- Used when creating filament groups

---

### FPS Configuration

**Section**: `[fps fps1]`

Each FPS (Filament Path Sensor) requires a configuration section.

```ini
[fps fps1]
pin: fps:PA2
reversed: false
oams: oams1
extruder: extruder4
reload_before_toolhead_distance: 6
```

**Settings**:
- `pin`: MCU pin for the FPS sensor
- `reversed`: Reverses sensor logic if needed (usually `false`)
- `oams`: Which OAMS unit this FPS connects to (must match `[oams ...]` section)
- `extruder`: Which extruder this FPS feeds (e.g., extruder4, extruder5)
- `reload_before_toolhead_distance`: Distance (in mm) before toolhead to reload filament during runout
  - **Lower value (3-5mm)**: Less waste, but requires precise tuning
  - **Higher value (8-10mm)**: More forgiving, but wastes more filament
  - **Default (6mm)**: Good balance for most setups

---

### Temperature Sensors

**Section**: `[temperature_sensor oams1]`

Optional but recommended - monitors temperature and humidity inside OAMS units.

```ini
[temperature_sensor oams1]
sensor_type: HDC1080
i2c_address: 64
i2c_mcu: oams_mcu1
i2c_bus: i2c0
i2c_speed: 200000
temp_offset: 0.0
humidity_offset: 0.0
temp_resolution: 14
humidity_resolution: 14
```

**Settings**:
- `temp_offset`: Calibration offset for temperature readings (in °C)
- `humidity_offset`: Calibration offset for humidity readings (in %)
- `temp_resolution`: Bits of resolution for temperature (14 = 0.01°C precision)
- `humidity_resolution`: Bits of resolution for humidity (14 = 0.01% precision)

**Note**: HDC1080 driver modifies Klipper's temperature_sensors.cfg, causing "dirty" branch status in Mainsail. This is cosmetic and doesn't affect operation.

---

### OAMS Manager

**Section**: `[oams_manager]`

```ini
[oams_manager]
```

This section enables the OAMS manager module. No additional parameters needed - it auto-detects all configured OAMS units.

---

## Macro Configuration

**File**: `AFC_oams_macros.cfg`

This file configures macros for loading, unloading, purging, and temperature management.

---

### Spoolman LED Sync

**Section**: `[spoolman_led_sync]`

```ini
[spoolman_led_sync]
enable: True
```

**What this does**:
- Automatically sets the active tool LED to match the filament color from Spoolman
- Falls back to default colors if Spoolman data not available
- Requires Klipper restart to enable/disable

**Optional LED Color Overrides**:
```ini
#default_color: 0000FF          # Active tool when no Spoolman data (blue)
#ready_color: 00FF00            # Lane ready (filament at load sensor)
#not_ready_color: FF0000        # Lane without filament
#loading_color: FFFF00          # Lane loading filament
#prep_loaded_color: 00FFFF     # Lane with filament at prep sensor
#unloading_color: FFA500        # Lane unloading
#fault_color: FF0000            # Lane fault state
#tool_loaded_idle_color: 0000AA # Tool loaded but idle
```

**Color format**: RRGGBB (6-digit hex without # prefix)

---

### Smart Temperature Management

**Section**: `[gcode_macro _oams_smart_temp_settings]`

```ini
[gcode_macro _oams_smart_temp_settings]
variable_enable_smart_temp: False
```

**IMPORTANT**: Controls how temperatures are managed during filament changes

#### When `enable_smart_temp: True` (RECOMMENDED)
- **Unload**: Uses temperature from `AFC.var.unit` for the loaded lane (from Spoolman or manual setting)
- **Load**: Uses temperature from `AFC.var.unit` for the new lane
- **Purge**: Uses `max(old_temp, new_temp)` to ensure proper melting of both filaments
- **Fallback**: Uses AFC material defaults or 240°C if no lane temp set

**Example**: Changing from PLA (220°C) to PETG (250°C)
1. Unloads at 220°C (PLA temp)
2. Heats to 250°C (max of 220 and 250)
3. Loads PETG at 250°C
4. Purges at 250°C

#### When `enable_smart_temp: False`
- Uses current extruder temperature without changes
- Allows manual temperature control
- Good for testing or if you prefer manual control

**Changing this setting**:
1. Edit the config file
2. Set `variable_enable_smart_temp: True` or `False`
3. Run `FIRMWARE_RESTART`

---

### Temperature Cache

**Sections**: `[gcode_macro _oams_temp_cache_fps1]`, `[gcode_macro _oams_temp_cache_fps2]`

```ini
[gcode_macro _oams_temp_cache_fps1]
variable_old_temp: 240
```

**What this does**:
- Stores the unload temperature for each FPS
- Used during load to calculate purge temperature
- Automatically updated during filament changes
- Isolated per FPS so each toolhead maintains its own state

**Don't change manually** - these are automatically managed by the macros

---

### Debug Settings

**Section**: `[gcode_macro _oams_debug_settings]`

```ini
[gcode_macro _oams_debug_settings]
variable_debug_enabled: 0  # Set to 1 to enable debug messages
```

**What this does**:
- `0` (default): Normal operation, minimal logging
- `1`: Enables detailed debug messages showing temperature calculations and decisions

**Performance**: Debug mode adds 5-10% overhead due to extra logging

---

### Purge Settings

**Section**: `[gcode_macro _oams_purge_settings]`

```ini
[gcode_macro _oams_purge_settings]
variable_purge_amount: 70  # Extrusion amount during purge (mm)
variable_purge_speed: 350  # Extrusion speed during purge (mm/min)
```

**What these do**:
- `purge_amount`: How much filament to extrude during purge phase
  - **Lower (50mm)**: Less waste, but may not fully clear old color
  - **Higher (100mm)**: More waste, but ensures complete color change
  - **Default (70mm)**: Good balance for most filament changes
- `purge_speed`: How fast to extrude during purge (mm/min)
  - **Lower (250mm/min)**: Slower, more controlled
  - **Higher (500mm/min)**: Faster, but may cause issues with some filaments

---

### Macro Variables (Legacy)

**Section**: `[gcode_macro _oams_macro_variables]`

```ini
[gcode_macro _oams_macro_variables]
variable_hotend_meltzone_compensation: 0
variable_retract_length: 10
variable_extrusion_reload_length: 55
variable_extrusion_unload_length: 45
variable_reload_speed: 400
```

**NOTE**: These variables are **deprecated** and no longer used by the macros. The macros now read values directly from `[AFC_extruder ...]` sections for per-toolhead configuration.

**Ignore these** - they're kept for backward compatibility only.

---

## Quick Start Checklist

Follow this checklist when setting up OpenAMS for the first time:

### 1. Hardware Configuration
- [ ] Set correct `canbus_uuid` for all MCUs
- [ ] Set `ptfe_length` to match your machine
- [ ] Verify `fps_is_reversed` setting (should show ~0.0 when unloaded)
- [ ] Run `OAMS_CALIBRATE_HUB_HES` and update `hub_hes_on` values

### 2. Macro Configuration
- [ ] Enable smart temperature: `variable_enable_smart_temp: True`
- [ ] Enable Spoolman LED sync if desired: `enable: True`
- [ ] Adjust purge settings if needed (defaults are good for most)

### 3. Testing
- [ ] Test single bay load/unload with `OAMSM_LOAD_FILAMENT LANE=lane1`
- [ ] Test filament change between different temperatures
- [ ] Verify LED colors match Spoolman data
- [ ] Check temperature logging shows correct values

### 4. Tuning
- [ ] Adjust `reload_before_toolhead_distance` if runout recovery fails
- [ ] Adjust `fps_upper/lower_threshold` if hub motor behavior is unstable
- [ ] Adjust `purge_amount` based on your color change needs

---

## Troubleshooting

### Filament loads but stops short of extruder
**Cause**: `ptfe_length` is too short
**Fix**: Increase `ptfe_length` by 50-100mm increments until filament reaches extruder

### Filament grinds against extruder gears
**Cause**: `ptfe_length` is too long
**Fix**: Decrease `ptfe_length` by 50mm increments

### FPS shows wrong values (1.0 when unloaded)
**Cause**: `fps_is_reversed` is incorrect
**Fix**: Toggle `fps_is_reversed` to the opposite value and restart

### Hub motor doesn't maintain pressure
**Cause**: FPS thresholds too narrow or PID needs tuning
**Fix**:
1. Try wider thresholds: `fps_upper_threshold: 0.8`, `fps_lower_threshold: 0.2`
2. If still unstable, adjust `current_kp` (start with 5.0)

### Wrong temperature during load
**Cause**: Smart temp disabled or lane doesn't have `extruder_temp` set
**Fix**:
1. Enable smart temp: `variable_enable_smart_temp: True`
2. Ensure Spoolman has temperatures set for all filaments
3. Or manually set temperature in AFC for each lane

### Spool detection unreliable
**Cause**: `f1s_hes_on` or `hub_hes_on` thresholds incorrect
**Fix**:
1. Run `OAMS_CALIBRATE_HUB_HES` for hub sensors
2. Check `klippy.log` for actual HES values when filament is loaded
3. Adjust thresholds to be between loaded and unloaded values

### Load retries failing
**Cause**: Physical obstruction or mechanical issue
**Fix**:
1. Check for PTFE tube kinks or sharp bends
2. Ensure filament path is clear
3. Verify spool can unwind freely
4. Increase `load_retry_max` if issue is intermittent

---

## Advanced: Temperature Resolution Flow

When smart temperature is enabled, here's how temperatures are resolved:

### 1. Unload Temperature
```
1. Check lane's extruder_temp from AFC.var.unit (from Spoolman)
2. If not set, check AFC material type and match to default_material_temps
3. If not found, use AFC default_material_type temp
4. If still not found, use fallback (min_extrude_temp + 5°C)
```

### 2. Load Temperature
```
1. Same resolution as unload, but for the NEW lane
2. Spoolman data is already loaded into lane before load
```

### 3. Purge Temperature
```
purge_temp = max(old_temp, new_temp)
```
Ensures both filaments melt properly during transition

### 4. Logging
When temperature is resolved, you'll see:
```
[TEMP] Lane lane5: extruder_temp=220C (from lane object)
[UNLOAD] Resolved temp: 220C for lane5
[PURGE] Cached old temp: 220C
[PURGE] New lane lane9 temp: 250C
[PURGE] Using purge temp: 250C (max of 220C and 250C)
```

This helps diagnose any temperature issues.

---

## Additional Resources

- **OpenAMS Documentation**: https://openams.si-forge.com/en/docs/klipper-config
- **Startup Checks**: https://openams.si-forge.com/en/docs/startup-checks
- **AFC Documentation**: (link to AFC docs)
- **Spoolman Integration**: (link to Spoolman docs)

---

## Version Information

**Document Version**: 1.0
**Last Updated**: 2026-01-02
**Compatible with**:
- AFC-Klipper-Add-On (latest)
- klipper_openams (latest)

---

*This guide was created to help new users understand and configure OpenAMS integration with AFC. If you find errors or have suggestions, please contribute to the documentation!*
