# OpenAMS Gcode Commands Reference

This guide provides a comprehensive reference for all gcode commands available in the OpenAMS system, organized by use case and module.

---

## Table of Contents

1. [Command Overview](#command-overview)
2. [Daily Operations](#daily-operations)
3. [Calibration Commands](#calibration-commands)
4. [Advanced Tuning](#advanced-tuning)
5. [Diagnostics & Troubleshooting](#diagnostics--troubleshooting)
6. [Developer Reference](#developer-reference)

---

## Command Overview

OpenAMS provides 25 gcode commands across three Python modules:

- **oams.py**: Low-level hardware control (10 commands)
- **oams_manager.py**: High-level coordinator (7 commands)
- **AFC_OpenAMS.py**: AFC integration (7 commands)
- **openams_integration.py**: No commands (backend utilities only)

### Command Naming Convention

- **OAMS_XXX**: Low-level hardware commands (require OAMS=<id> parameter)
- **OAMSM_XXX**: Manager-level commands (work across all OAMS units)
- **AFC_OAMS_XXX**: AFC-integrated commands (require UNIT=<name> parameter)

---

## Daily Operations

Commands you'll use regularly during normal printing operations.

### Loading Filament

#### OAMSM_LOAD_FILAMENT
**Module**: oams_manager.py
**Usage**: `OAMSM_LOAD_FILAMENT LANE=<lane_name>`
**When to use**: Load filament from a specific AFC lane

**Example**:
```gcode
OAMSM_LOAD_FILAMENT LANE=lane4
```

**What it does**:
- Loads filament from the specified AFC lane configuration
- Automatically determines which OAMS unit and spool bay to use
- Reports success or failure with detailed messages
- Integrates with AFC's lane management system

**Typical output**:
```
Loading lane4 from oams1 spool 0
Load complete
```

**Troubleshooting**:
- "Lane not found": Check your AFC lane configuration
- "OAMS not ready": Check MCU connection with OAMSM_STATUS
- "Already loaded": Run OAMSM_UNLOAD_FILAMENT first

---

### Unloading Filament

#### OAMSM_UNLOAD_FILAMENT
**Module**: oams_manager.py
**Usage**: `OAMSM_UNLOAD_FILAMENT FPS=<fps_name>`
**When to use**: Unload currently loaded filament from an FPS

**Example**:
```gcode
OAMSM_UNLOAD_FILAMENT FPS=fps1
```

**What it does**:
- Unloads whichever spool is currently loaded on the specified FPS
- Works regardless of which lane or bay the filament came from
- Automatically rewinds spool with PID-controlled tension
- Updates AFC and OAMS state tracking

**Parameters**:
- `FPS=<name>`: FPS sensor name (e.g., fps1, fps2)
  - Must match your `[fps ...]` config section name

**Typical output**:
```
Unloading from fps1
Unload complete
```

**Troubleshooting**:
- "FPS not found": Check your [fps ...] config sections
- "Already unloading": Wait for current operation to complete
- "Nothing loaded": FPS shows no filament loaded

---

### Follower Motor Control

The follower motor is the BLDC motor that feeds filament from the OAMS hub to the toolhead.

#### OAMSM_FOLLOWER
**Module**: oams_manager.py
**Usage**: `OAMSM_FOLLOWER FPS=<fps_name> ENABLE=<0|1> [DIRECTION=<0|1>]`
**When to use**: Manually control follower motor during maintenance or troubleshooting

**Example**:
```gcode
# Enable follower in forward direction
OAMSM_FOLLOWER FPS=fps1 ENABLE=1 DIRECTION=1

# Disable follower
OAMSM_FOLLOWER FPS=fps1 ENABLE=0
```

**Parameters**:
- `FPS=<name>`: FPS sensor name
- `ENABLE=<0|1>`: 0=disable, 1=enable
- `DIRECTION=<0|1>`: 0=reverse (retract), 1=forward (feed)
  - Required when ENABLE=1
  - Ignored when ENABLE=0

**What it does**:
- **Enable**: Starts follower motor feeding filament
- **Disable**: Stops follower motor (filament hangs freely)
- Sets "manual override" flag to prevent automatic control

**Safety features**:
- Blocks enable during LOADING/UNLOADING states
- Blocks disable if clog or stuck spool error is active
- Must use OAMSM_FOLLOWER_RESET to return to automatic control

**Use cases**:
- **Manual filament feeding**: Enable follower to push filament during maintenance
- **Clearing jams**: Reverse direction to pull filament back
- **Testing**: Verify follower motor operation

**IMPORTANT**: After manual control, run `OAMSM_FOLLOWER_RESET` to return to automatic operation!

---

#### OAMSM_FOLLOWER_RESET
**Module**: oams_manager.py
**Usage**: `OAMSM_FOLLOWER_RESET FPS=<fps_name>`
**When to use**: Return follower to automatic control after manual operation

**Example**:
```gcode
OAMSM_FOLLOWER_RESET FPS=fps1
```

**What it does**:
- Clears manual override flag
- Returns follower to automatic hub sensor-based control
- Immediately updates follower state based on current hub sensors
- Essential after using OAMSM_FOLLOWER

**Typical output**:
```
Follower on fps1 returned to automatic control (hub sensor based)
Follower state updated based on current hub sensors
```

---

### Error Recovery

#### OAMSM_CLEAR_ERRORS
**Module**: oams_manager.py
**Usage**: `OAMSM_CLEAR_ERRORS`
**When to use**: Clear all error states after fixing a problem

**Example**:
```gcode
OAMSM_CLEAR_ERRORS
```

**What it does** (comprehensive reset):
1. Resets all runout monitors (clears COASTING and error states)
2. Clears FPS state error flags and tracking
3. Resets stuck spool and clog trackers
4. Clears OAMS hardware errors and LED errors
5. Re-detects state from hardware sensors
6. Refreshes state from AFC snapshot
7. **NEW**: Syncs both loaded AND unloaded states bidirectionally
   - If AFC shows lane loaded, syncs OAMS to match
   - If AFC shows lane NOT loaded, clears OAMS FPS state
8. Clears manual follower overrides
9. Returns followers to automatic control
10. Syncs virtual tool sensors

**When to use**:
- After fixing a clog or jam
- After manually clearing filament
- When OAMS state and AFC state are out of sync
- After runout recovery
- When LED shows errors that are no longer relevant

**Preserves**:
- Lane mappings (use OAMSM_CLEAR_LANE_MAPPINGS to clear those)
- Configuration settings
- Calibration data

**Typical output**:
```
OAMS errors cleared and system re-initialized
Force-synced from AFC: 1 loaded, 0 unloaded
```

---

#### OAMSM_CLEAR_LANE_MAPPINGS
**Module**: oams_manager.py
**Usage**: `OAMSM_CLEAR_LANE_MAPPINGS`
**When to use**: Clear cross-extruder lane redirects after print ends

**Example**:
```gcode
OAMSM_CLEAR_LANE_MAPPINGS
```

**What it does**:
- Removes cross-extruder lane mappings created during runout recovery
- Example: If lane4 → lane8 redirect was created, this removes it
- Iterates through all AFC lanes and clears any lane.map redirects
- Reports count of cleared mappings

**Background**:
When a same-FPS runout occurs but target lane is on a different extruder, OAMS creates a lane mapping to redirect the load. These mappings should be cleared after print completion.

**Best practice**:
Add to your PRINT_END or CANCEL_PRINT macros:
```gcode
[gcode_macro PRINT_END]
gcode:
    # ... your existing print end code ...
    OAMSM_CLEAR_LANE_MAPPINGS
```

**Typical output**:
```
Cleared 2 lane mapping(s)
```

---

## Calibration Commands

These commands must be run during initial setup and after hardware changes.

### PTFE Length Calibration

#### AFC_OAMS_CALIBRATE_PTFE
**Module**: AFC_OpenAMS.py
**Usage**: `AFC_OAMS_CALIBRATE_PTFE UNIT=<unit_name> SPOOL=<0-3>`
**When to use**: Calibrate PTFE tube length for a specific lane

**Example**:
```gcode
AFC_OAMS_CALIBRATE_PTFE UNIT=AMS_1 SPOOL=0
```

**Prerequisites**:
- No lanes loaded to toolhead
- Filament loaded in the bay you're calibrating
- Extruder at operating temperature

**What it does**:
1. Finds the AFC lane corresponding to the spool index
2. Calls low-level OAMS_CALIBRATE_PTFE_LENGTH
3. Feeds filament until FPS slide maxes out
4. Measures total clicks (encoder pulses)
5. Calculates PTFE length
6. Updates configuration file automatically

**Parameters**:
- `UNIT=<name>`: AFC unit name (e.g., AMS_1, AMS_2)
- `SPOOL=<0-3>`: Spool bay index (0-3)

**Typical output**:
```
Calibrating PTFE length for lane4 (spool 0)
Calibration complete: 2001 clicks
Updated configuration with ptfe_length: 2001
```

**After calibration**:
- Restart Klipper to load new ptfe_length value
- Test load/unload to verify filament reaches toolhead

---

#### UNIT_PTFE_CALIBRATION
**Module**: AFC_OpenAMS.py
**Usage**: `UNIT_PTFE_CALIBRATION UNIT=<unit_name>`
**When to use**: Show interactive calibration menu for all lanes

**Example**:
```gcode
UNIT_PTFE_CALIBRATION UNIT=AMS_1
```

**What it does**:
- Creates interactive menu with buttons for each loaded lane
- Clicking a button runs calibration for that lane
- Convenient alternative to typing commands for each bay
- Only shows lanes that currently have filament loaded

**Prerequisites**:
- No lanes loaded to toolhead
- At least one lane loaded in OAMS bays

**Typical output**:
```
[Interactive menu appears with buttons]
Calibrate Lane 4 (Bay 0)
Calibrate Lane 5 (Bay 1)
...
```

---

### Hub HES Calibration

#### AFC_OAMS_CALIBRATE_HUB_HES
**Module**: AFC_OpenAMS.py
**Usage**: `AFC_OAMS_CALIBRATE_HUB_HES UNIT=<unit_name> SPOOL=<0-3>`
**When to use**: Calibrate Hall Effect Sensor threshold for hub filament detection

**Example**:
```gcode
AFC_OAMS_CALIBRATE_HUB_HES UNIT=AMS_1 SPOOL=0
```

**CRITICAL**: This must be run during initial setup!

**Prerequisites**:
- No lanes loaded to toolhead
- Filament loaded in the bay you're calibrating

**What it does**:
1. Finds the AFC lane corresponding to the spool index
2. Calls low-level OAMS_CALIBRATE_HUB_HES
3. Measures HES value when filament is present at hub motor
4. Updates hub_hes_on threshold in configuration
5. Automatically saves to config file

**Parameters**:
- `UNIT=<name>`: AFC unit name
- `SPOOL=<0-3>`: Spool bay index

**Typical output**:
```
Calibrating HUB HES for lane4 (spool 0)
Calibration complete: threshold = 0.824217
Updated hub_hes_on[0] = 0.824217
```

**After calibration**:
- Restart Klipper to load new threshold
- Test loading to verify hub detection works correctly

---

#### AFC_OAMS_CALIBRATE_HUB_HES_ALL
**Module**: AFC_OpenAMS.py
**Usage**: `AFC_OAMS_CALIBRATE_HUB_HES_ALL UNIT=<unit_name>`
**When to use**: Bulk calibration of all loaded lanes

**Example**:
```gcode
AFC_OAMS_CALIBRATE_HUB_HES_ALL UNIT=AMS_1
```

**What it does**:
- Automatically calibrates hub HES for ALL lanes with filament loaded
- Iterates through all 4 bays
- Skips bays without filament
- Updates configuration for all calibrated bays
- Reports total number of successful calibrations

**Prerequisites**:
- No lanes loaded to toolhead
- At least one bay loaded with filament
- More filament loaded = more bays calibrated

**Typical output**:
```
Calibrating all loaded lanes on AMS_1...
Calibrated bay 0: lane4 = 0.824217
Calibrated bay 1: lane5 = 0.809127
Calibrated bay 2: lane6 = 0.837821
Skipped bay 3: no filament
Successfully calibrated 3 bays
```

**Best practice**: Run this during initial setup with all 4 bays loaded to calibrate everything at once.

---

## Advanced Tuning

Commands for fine-tuning OAMS behavior. Most users won't need these.

### PID Tuning

#### OAMS_PID_SET
**Module**: oams.py
**Usage**: `OAMS_PID_SET OAMS=<id> P=<float> I=<float> D=<float> [TARGET=<float>]`
**When to use**: Manually set PID values for FPS pressure control

**Example**:
```gcode
# Set PID for OAMS unit 1
OAMS_PID_SET OAMS=1 P=3.0 I=0.5 D=0.1 TARGET=0.5
```

**What it does**:
- Updates PID control parameters for filament pressure sensor
- Controls how aggressively hub motor maintains setpoint pressure
- TARGET sets desired FPS slide position (0.0 to 1.0)

**Parameters**:
- `OAMS=<id>`: OAMS unit ID (1, 2, etc.)
- `P=<float>`: Proportional gain (how much to react to error)
- `I=<float>`: Integral gain (how much to correct accumulated error)
- `D=<float>`: Derivative gain (how much to dampen oscillations)
- `TARGET=<float>`: Desired FPS position (optional, defaults to config)

**When to tune**:
- Hub motor oscillates (increase D, decrease P)
- Hub motor too slow to respond (increase P)
- Persistent offset from target (increase I)

**Default values from config** (usually work well):
```ini
fps_upper_threshold: 0.7
fps_lower_threshold: 0.3
# TARGET = average = 0.5
```

---

#### OAMS_CURRENT_PID_SET
**Module**: oams.py
**Usage**: `OAMS_CURRENT_PID_SET OAMS=<id> P=<float> I=<float> D=<float> [TARGET=<float>]`
**When to use**: Manually set PID values for motor current control during rewind

**Example**:
```gcode
OAMS_CURRENT_PID_SET OAMS=1 P=3.0 I=0.0 D=0.0 TARGET=0.30
```

**What it does**:
- Controls spool rewind behavior during unload
- Adjusts hub motor speed to maintain target DC motor current
- Prevents over-tension or under-tension during rewind

**Parameters**:
- `OAMS=<id>`: OAMS unit ID
- `P=<float>`: Proportional gain
- `I=<float>`: Integral gain (usually 0)
- `D=<float>`: Derivative gain (usually 0)
- `TARGET=<float>`: Target DC motor current (0.0 to 1.0)

**Default values** (from config, rarely need changing):
```ini
current_target: 0.30
current_kp: 3.0
current_ki: 0.0
current_kd: 0.0
```

**When to tune**:
- Filament tangling during rewind (increase TARGET)
- Spool overwinding (decrease TARGET)

---

#### OAMS_PID_AUTOTUNE
**Module**: oams.py
**Usage**: `OAMS_PID_AUTOTUNE OAMS=<id> TARGET_FLOW=<float> TARGET_TEMP=<float>`
**When to use**: Automatically tune PID values for pressure control

**Example**:
```gcode
# Autotune for PLA at 220°C with 5mm³/s flow
OAMS_PID_AUTOTUNE OAMS=1 TARGET_FLOW=5.0 TARGET_TEMP=220
```

**What it does**:
1. Heats nozzle to TARGET_TEMP
2. Extrudes for 30 seconds at TARGET_FLOW rate
3. Monitors FPS response and calculates optimal PID values
4. Reports calculated P, I, D values

**Parameters**:
- `OAMS=<id>`: OAMS unit ID
- `TARGET_FLOW=<float>`: Flow rate in mm³/s
  - Typical values: 3-10 mm³/s
  - Higher flow = more aggressive tuning
- `TARGET_TEMP=<float>`: Nozzle temperature in °C
  - Match your typical printing temperature

**After autotune**:
1. Note the calculated P, I, D values
2. Run `OAMS_PID_SET` with those values
3. Save to configuration file
4. Restart Klipper

**When to run**:
- After changing PTFE tube length
- After changing nozzle or hotend
- If pressure control is unstable

---

### Retry Configuration

#### OAMS_RETRY_STATUS
**Module**: oams.py
**Usage**: `OAMS_RETRY_STATUS OAMS=<id>`
**When to use**: Check retry configuration and current retry state

**Example**:
```gcode
OAMS_RETRY_STATUS OAMS=1
```

**Typical output**:
```
OAMS[1] Retry Status:
  Load retry max: 3
  Unload retry max: 2
  Backoff: 1.0s (max 5.0s)
  Auto-unload on failed load: True
  Current unload retry count: 0
  Load retry counts:
    Spool 0: 1/3
    Spool 2: 2/3
  No active load retries
```

**What it shows**:
- Maximum retry attempts configured
- Backoff timing (exponential: 1s, 2s, 4s, ...)
- Auto-unload setting
- Current retry counts for each spool
- Active retry states

**Configuration** (in AFC_oams.cfg):
```ini
load_retry_max: 3
unload_retry_max: 2
retry_backoff_base: 1.0
retry_backoff_max: 5.0
auto_unload_on_failed_load: True
```

---

#### OAMS_RESET_RETRY_COUNTS
**Module**: oams.py
**Usage**: `OAMS_RESET_RETRY_COUNTS OAMS=<id>`
**When to use**: Clear retry counters after fixing a problem

**Example**:
```gcode
OAMS_RESET_RETRY_COUNTS OAMS=1
```

**What it does**:
- Clears all load and unload retry counters
- Resets attempt timestamps for all spools
- Clears retry status flags
- Allows fresh retry attempts

**When to use**:
- After fixing a mechanical issue
- After load failures that exhausted retries
- To manually reset after troubleshooting

**Note**: OAMSM_CLEAR_ERRORS also resets retry counts as part of its comprehensive reset.

---

## Diagnostics & Troubleshooting

Commands for diagnosing issues and checking system state.

### System Status

#### OAMSM_STATUS
**Module**: oams_manager.py
**Usage**: `OAMSM_STATUS`
**When to use**: Comprehensive diagnostic check of OAMS system

**Example**:
```gcode
OAMSM_STATUS
```

**What it shows** (comprehensive report):
1. AFC and tool configuration
2. All FPS sensors and their states
3. Current OAMS assignments for each FPS
4. Current lane assignments
5. Load states (LOADED, UNLOADED, LOADING, UNLOADING)
6. Spool indices for loaded bays
7. State detection diagnostics
8. Virtual tool sensor sync status

**Typical output**:
```
OAMSM STATUS:

AFC Configuration:
  Found AFC object: AFC
  Available tools: extruder4, extruder5

FPS Sensors:
  fps1 (extruder4):
    State: LOADED
    Current OAMS: oams1
    Current lane: lane4
    Spool index: 0

  fps2 (extruder5):
    State: UNLOADED
    Current OAMS: None
    Current lane: None

State Detection Results:
  Running determine_state()...
  Detection complete

Virtual Tool Sensors:
  Synced 2 sensor(s)
```

**When to use**:
- Troubleshooting load/unload issues
- Verifying state after OAMSM_CLEAR_ERRORS
- Checking if OAMS MCUs are ready
- Diagnosing state detection problems
- Before contacting support (include this output!)

---

### Filament Sensor Queries

#### QUERY_FILAMENT_SENSOR
**Module**: AFC_OpenAMS.py
**Usage**: `QUERY_FILAMENT_SENSOR SENSOR=<sensor_name>`
**When to use**: Check if FPS detects filament

**Example**:
```gcode
QUERY_FILAMENT_SENSOR SENSOR=fila_fps1
```

**What it does**:
- Queries current filament detection status
- Uses runout helper to check sensor state

**Typical output**:
```
Filament Sensor fila_fps1: filament detected
```
or
```
Filament Sensor fila_fps1: filament not detected
```

**Sensor names**:
- Usually `fila_fps1`, `fila_fps2`, etc.
- Defined in your AFC configuration
- Related to FPS sensor definitions

---

#### SET_FILAMENT_SENSOR
**Module**: AFC_OpenAMS.py
**Usage**: `SET_FILAMENT_SENSOR SENSOR=<sensor_name> ENABLE=<0|1>`
**When to use**: Enable or disable filament sensor monitoring

**Example**:
```gcode
# Disable sensor
SET_FILAMENT_SENSOR SENSOR=fila_fps1 ENABLE=0

# Enable sensor
SET_FILAMENT_SENSOR SENSOR=fila_fps1 ENABLE=1
```

**What it does**:
- Enables or disables runout detection for specific FPS
- When disabled, no runout events will be triggered
- When enabled, runout detection is active

**Parameters**:
- `SENSOR=<name>`: Filament sensor name
- `ENABLE=<0|1>`: 0=disable, 1=enable (default=1)

**Use cases**:
- **Disable during maintenance**: Prevent false runout triggers
- **Disable for manual loading**: Avoid runout errors during setup
- **Enable after maintenance**: Resume normal operation

---

### Virtual Sensor Sync

#### SYNC_TOOL_SENSOR
**Module**: AFC_OpenAMS.py
**Usage**: `SYNC_TOOL_SENSOR UNIT=<unit_name>`
**When to use**: Synchronize virtual tool sensor with actual lane state

**Example**:
```gcode
SYNC_TOOL_SENSOR UNIT=AMS_1
```

**What it does**:
- Synchronizes virtual tool-start sensor state with actual filament state
- Corrects stale sensor states after reboot or errors
- Ensures sensor accurately reflects current lane loading status

**When to use**:
- After Klipper restart (sensor may show wrong state)
- After OAMSM_CLEAR_ERRORS
- When AFC shows lane loaded but sensor shows empty (or vice versa)
- Before starting a print if sensor state looks wrong

**Background**:
Virtual tool sensors (like `fila_tool_start`) track whether filament is loaded to the toolhead. These can get out of sync after power cycles or errors. This command forces a resync.

---

## Developer Reference

### Low-Level Hardware Commands

These commands directly control OAMS hardware. Most users should use the OAMSM_XXX equivalents instead.

#### OAMS_LOAD_SPOOL
**Module**: oams.py
**Usage**: `OAMS_LOAD_SPOOL OAMS=<id> SPOOL=<0-3>`
**Developer use**: Direct spool load without AFC integration

**Example**:
```gcode
OAMS_LOAD_SPOOL OAMS=1 SPOOL=0
```

**What it does**:
- Low-level load command
- Bypasses AFC lane management
- Directly loads from hardware spool bay
- Includes automatic retry with exponential backoff

**Parameters**:
- `OAMS=<id>`: OAMS unit ID (1, 2, etc.)
- `SPOOL=<0-3>`: Physical spool bay index

**Safety**:
- Blocks if another spool already loaded
- Times out after 30 seconds
- Updates internal spool state on success

**Use case**: Hardware testing, debugging, development

---

#### OAMS_UNLOAD_SPOOL
**Module**: oams.py
**Usage**: `OAMS_UNLOAD_SPOOL OAMS=<id>`
**Developer use**: Direct spool unload without AFC integration

**Example**:
```gcode
OAMS_UNLOAD_SPOOL OAMS=1
```

**What it does**:
- Low-level unload command
- Bypasses AFC lane management
- Directly unloads currently loaded spool
- Includes automatic retry with exponential backoff

**Parameters**:
- `OAMS=<id>`: OAMS unit ID

**Use case**: Hardware testing, debugging, development

---

#### OAMS_FOLLOWER
**Module**: oams.py
**Usage**: `OAMS_FOLLOWER OAMS=<id> ENABLE=<0|1> DIRECTION=<0|1>`
**Developer use**: Direct follower control without state management

**Example**:
```gcode
OAMS_FOLLOWER OAMS=1 ENABLE=1 DIRECTION=1
```

**What it does**:
- Low-level follower motor control
- No state validation or safety checks
- Direct MCU command execution

**Parameters**:
- `OAMS=<id>`: OAMS unit ID
- `ENABLE=<0|1>`: Motor enable
- `DIRECTION=<0|1>`: Rotation direction

**WARNING**: Use OAMSM_FOLLOWER instead for normal operations! This bypasses safety checks.

**Use case**: Hardware testing, MCU debugging

---

## Quick Reference Tables

### Command by Use Case

| Use Case | Command | Frequency |
|----------|---------|-----------|
| Load filament | OAMSM_LOAD_FILAMENT | Daily |
| Unload filament | OAMSM_UNLOAD_FILAMENT | Daily |
| Clear errors | OAMSM_CLEAR_ERRORS | As needed |
| System status | OAMSM_STATUS | Troubleshooting |
| PTFE calibration | AFC_OAMS_CALIBRATE_PTFE | Initial setup |
| HUB HES calibration | AFC_OAMS_CALIBRATE_HUB_HES_ALL | Initial setup |
| Check sensor | QUERY_FILAMENT_SENSOR | Troubleshooting |
| Sync sensors | SYNC_TOOL_SENSOR | After restart |
| Manual follower | OAMSM_FOLLOWER | Maintenance |
| Reset follower | OAMSM_FOLLOWER_RESET | After manual |

---

### Command by Module

| Module | Commands | Purpose |
|--------|----------|---------|
| oams.py | 10 commands | Low-level hardware control |
| oams_manager.py | 7 commands | High-level coordination |
| AFC_OpenAMS.py | 7 commands | AFC integration |
| openams_integration.py | 0 commands | Backend utilities |

---

### Mux Commands

Mux commands require a parameter to identify which instance:

| Command Pattern | Parameter | Example |
|----------------|-----------|---------|
| OAMS_XXX | OAMS=<id> | OAMS_LOAD_SPOOL OAMS=1 |
| AFC_OAMS_XXX | UNIT=<name> | AFC_OAMS_CALIBRATE_HUB_HES UNIT=AMS_1 |
| QUERY_FILAMENT_SENSOR | SENSOR=<name> | QUERY_FILAMENT_SENSOR SENSOR=fila_fps1 |
| SET_FILAMENT_SENSOR | SENSOR=<name> | SET_FILAMENT_SENSOR SENSOR=fila_fps1 |

**Direct commands** (no mux parameter):
- OAMSM_XXX commands work across all OAMS units
- SYNC_TOOL_SENSOR requires UNIT=<name>

---

## Common Workflows

### Initial Setup Workflow

1. **Calibrate HUB HES** (required):
   ```gcode
   AFC_OAMS_CALIBRATE_HUB_HES_ALL UNIT=AMS_1
   FIRMWARE_RESTART
   ```

2. **Calibrate PTFE Length** (required):
   ```gcode
   # Heat nozzle first
   M109 S220

   # Calibrate each bay with filament
   AFC_OAMS_CALIBRATE_PTFE UNIT=AMS_1 SPOOL=0
   AFC_OAMS_CALIBRATE_PTFE UNIT=AMS_1 SPOOL=1
   # etc.

   FIRMWARE_RESTART
   ```

3. **Test Load/Unload**:
   ```gcode
   OAMSM_LOAD_FILAMENT LANE=lane4
   # Verify filament reaches toolhead

   OAMSM_UNLOAD_FILAMENT FPS=fps1
   # Verify filament rewinds cleanly
   ```

4. **Check Status**:
   ```gcode
   OAMSM_STATUS
   # Verify all systems show ready
   ```

---

### Error Recovery Workflow

1. **Fix physical problem** (clear jam, reload filament, etc.)

2. **Clear all errors**:
   ```gcode
   OAMSM_CLEAR_ERRORS
   ```

3. **Verify state**:
   ```gcode
   OAMSM_STATUS
   # Check FPS states, lane assignments
   ```

4. **Sync virtual sensors** (if needed):
   ```gcode
   SYNC_TOOL_SENSOR UNIT=AMS_1
   ```

5. **Resume operation**

---

### Print End Workflow

Add to your PRINT_END macro:
```gcode
[gcode_macro PRINT_END]
gcode:
    # ... your existing print end code ...

    # Clear cross-extruder lane mappings
    OAMSM_CLEAR_LANE_MAPPINGS

    # Optionally unload filament
    # OAMSM_UNLOAD_FILAMENT FPS=fps1
```

---

### Manual Maintenance Workflow

1. **Disable filament sensor**:
   ```gcode
   SET_FILAMENT_SENSOR SENSOR=fila_fps1 ENABLE=0
   ```

2. **Enable follower manually**:
   ```gcode
   OAMSM_FOLLOWER FPS=fps1 ENABLE=1 DIRECTION=1
   ```

3. **Perform maintenance** (feed filament, clear jam, etc.)

4. **Reset follower to automatic**:
   ```gcode
   OAMSM_FOLLOWER_RESET FPS=fps1
   ```

5. **Re-enable filament sensor**:
   ```gcode
   SET_FILAMENT_SENSOR SENSOR=fila_fps1 ENABLE=1
   ```

6. **Clear any errors**:
   ```gcode
   OAMSM_CLEAR_ERRORS
   ```

---

## Troubleshooting Guide

### Issue: Filament loads but stops short of extruder

**Check**:
1. Run `OAMSM_STATUS` - verify OAMS is ready
2. Check `ptfe_length` in config
3. Run `AFC_OAMS_CALIBRATE_PTFE` to recalibrate

---

### Issue: "OAMS not ready" error

**Check**:
1. Run `OAMSM_STATUS` - check MCU status
2. Verify CAN bus connection: `~/klippy-env/bin/python ~/klipper/scripts/canbus_query.py can0`
3. Check MCU power and wiring
4. Restart Klipper: `FIRMWARE_RESTART`

---

### Issue: Load keeps failing/retrying

**Check**:
1. Run `OAMS_RETRY_STATUS OAMS=1` - check retry count
2. Check for physical obstruction
3. Verify spool can unwind freely
4. Check PTFE tube for kinks
5. Run `AFC_OAMS_CALIBRATE_HUB_HES` if hub detection unreliable
6. Run `OAMS_RESET_RETRY_COUNTS OAMS=1` to clear retries

---

### Issue: Follower motor won't enable

**Check**:
1. Check if clog or stuck spool error active: `OAMSM_STATUS`
2. Clear errors: `OAMSM_CLEAR_ERRORS`
3. Verify not in LOADING/UNLOADING state
4. Try manual enable: `OAMSM_FOLLOWER FPS=fps1 ENABLE=1 DIRECTION=1`

---

### Issue: Virtual sensor shows wrong state

**Fix**:
1. Sync sensor: `SYNC_TOOL_SENSOR UNIT=AMS_1`
2. If still wrong: `OAMSM_CLEAR_ERRORS`
3. Verify with: `QUERY_FILAMENT_SENSOR SENSOR=fila_fps1`

---

### Issue: AFC and OAMS states out of sync

**Fix**:
1. Run `OAMSM_CLEAR_ERRORS` - this now does bidirectional sync
2. Verify with `OAMSM_STATUS`
3. Check logs for sync messages:
   - "Force-synced from AFC: X loaded, Y unloaded"

---

## Version Information

**Document Version**: 1.0
**Last Updated**: 2026-01-02
**Compatible with**:
- oams.py (klipper_openams)
- oams_manager.py (klipper_openams)
- AFC_OpenAMS.py (AFC-Klipper-Add-On)
- openams_integration.py (AFC-Klipper-Add-On)

---

## Additional Resources

- **Configuration Guide**: See OpenAMS_Configuration_Guide.md
- **OpenAMS Documentation**: https://openams.si-forge.com/
- **AFC Documentation**: (link to AFC docs)

---

*This reference was created to document all available gcode commands in the OpenAMS system. If you find errors or have suggestions, please contribute!*
