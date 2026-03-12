# AFCACE Configuration Reference

Direct ACE PRO hardware integration for AFC without ACEPRO/DuckACE dependency.
Supports two operational modes: combined (shared toolhead) and direct (multi-extruder).

---

## Unit Configuration

```ini
[AFC_AFCACE <unit_name>]
```

### Required Settings

| Setting | Type | Description |
|---------|------|-------------|
| `serial_port` | string | USB serial device path (e.g., `/dev/ttyACM0`) |

### Operational Mode

| Setting | Type | Default | Description |
|---------|------|---------|-------------|
| `mode` | string | `combined` | `combined` = multiple slots share one toolhead (retracts current before feeding new). `direct` = each slot feeds its own extruder independently (no auto-retract). |

### Feed / Retract Parameters

| Setting | Type | Default | Unit | Description |
|---------|------|---------|------|-------------|
| `feed_speed` | float | `800.0` | mm/min | Speed to feed filament from ACE to toolhead |
| `retract_speed` | float | `800.0` | mm/min | Speed to retract filament back to ACE |
| `feed_length` | float | `500.0` | mm | Distance from ACE unit to toolhead sensor. Use `AFCACE_CALIBRATE` to measure this. |
| `retract_length` | float | `500.0` | mm | Distance to retract back to ACE. Should be >= feed_length. |

### Feed Assist

| Setting | Type | Default | Description |
|---------|------|---------|-------------|
| `use_feed_assist` | bool | `True` | Enable motorized feed assist during loading (default for all slots). Can be overridden per-slot at runtime via `AFCACE_FEED_ASSIST`. |
| `extruder_assist_length` | float | `50.0` | mm to advance with extruder motor during feed assist (pulls filament into hotend) |
| `extruder_assist_speed` | float | `300.0` | mm/min speed for extruder assist movement |

### Sensor-Based Feeding

During loading, AFCACE feeds the bulk of the distance in one shot, then switches to
incremental feeding near the toolhead sensor. This allows it to stop precisely when the
sensor triggers instead of blindly overshooting.

| Setting | Type | Default | Unit | Description |
|---------|------|---------|------|-------------|
| `sensor_approach_margin` | float | `60.0` | mm | How far before the expected sensor position to switch from bulk feed to incremental feed. Larger = more sensor checks but slower loading. |
| `sensor_step` | float | `20.0` | mm | Feed increment size during sensor approach (normal loading) |
| `calibration_step` | float | `10.0` | mm | Feed increment size during calibration (smaller = more precise but slower) |
| `max_feed_overshoot` | float | `100.0` | mm | Extra distance to try past feed_length before giving up on sensor detection |

### Serial / Hardware

| Setting | Type | Default | Description |
|---------|------|---------|-------------|
| `baud_rate` | int | `115200` | USB serial baud rate (ACE PRO default is 115200) |
| `poll_interval` | float | `1.0` | Seconds between slot status polls for runout detection |

### Inherited from AFC Unit (afcUnit)

These settings are inherited from the base AFC unit class and can be set in the `[AFC_AFCACE]`
section to override AFC-level defaults:

| Setting | Type | Default | Description |
|---------|------|---------|-------------|
| `hub` | string | None | Hub name (`[AFC_hub]` section) for this unit |
| `extruder` | string | None | Extruder name (`[AFC_extruder]` section) for this unit |
| `buffer` | string | None | Buffer name (`[AFC_buffer]` section). Use with Turtle Neck buffer for ramming mode |
| `td1_device_id` | string | None | TD-1 device serial number for filament detection calibration |
| `capture_td1_when_loaded` | bool | AFC default | Capture TD-1 spool data when filament is loaded to toolhead |
| `debug` | bool | `False` | Enable debug logging |
| `led_fault` | string | AFC default | LED color for fault state (R,G,B,W format) |
| `led_ready` | string | AFC default | LED color for ready state |
| `led_not_ready` | string | AFC default | LED color for not-ready state |
| `led_loading` | string | AFC default | LED color during loading |
| `led_unloading` | string | AFC default | LED color during unloading |
| `led_tool_loaded` | string | AFC default | LED color when loaded into tool |
| `unload_on_runout` | bool | AFC default | Unload lane on runout when no infinite spool target |

---

## Lane Configuration

Each ACE slot maps to an AFC lane:

```ini
[AFC_lane <lane_name>]
unit: <unit_name>:<slot_number>    # Slot is 1-based (1-4)
hub: <hub_name>
extruder: <extruder_name>
```

Slot numbers are **1-based** in the config (1, 2, 3, 4) but converted to 0-based internally
for the ACE hardware protocol.

### Optional Per-Lane Settings

| Setting | Type | Default | Description |
|---------|------|---------|-------------|
| `buffer` | string | unit default | Buffer name for this specific lane (overrides unit setting) |
| `td1_device_id` | string | unit default | TD-1 device serial number for this lane (overrides unit setting) |
| `td1_bowden_length` | float | None | Calibrated distance from hub to TD-1 device (mm). Use `AFC_UNIT_TD_ONE_CALIBRATION` to measure. |
| `capture_td1_when_loaded` | bool | unit default | Capture TD-1 spool data when loaded |

---

## GCode Commands

### AFCACE_STATUS
Query hardware status (temperatures, sensors, firmware info).
```
AFCACE_STATUS UNIT=<unit_name>
```

### AFCACE_CALIBRATE
Measure bowden length by feeding until the toolhead sensor triggers.
Feeds in small increments (`calibration_step` mm each), reports the measured distance.
Retracts filament after measurement.
```
AFCACE_CALIBRATE UNIT=<unit_name> LANE=<lane_name> [MAX=<max_distance_mm>]
```

### AFCACE_FEED_ASSIST
Query or set feed assist state, globally or per-slot.
```
AFCACE_FEED_ASSIST UNIT=<unit_name>                           # Query all slots
AFCACE_FEED_ASSIST UNIT=<unit_name> ENABLE=<0|1>              # Set default for all slots
AFCACE_FEED_ASSIST UNIT=<unit_name> SLOT=<1-4> ENABLE=<0|1>   # Override specific slot
AFCACE_FEED_ASSIST UNIT=<unit_name> SLOT=<1-4> ENABLE=default # Clear slot override
```

### AFCACE_SYNC_INVENTORY
Re-enable RFID, pull fresh spool data from all slots, and sync to AFC lanes.
```
AFCACE_SYNC_INVENTORY UNIT=<unit_name>
```

### AFCACE_DRY
Start the ACE filament dryer.
```
AFCACE_DRY UNIT=<unit_name> TEMP=<celsius> DURATION=<minutes> [FAN=<rpm>]
```
Default: TEMP=50, DURATION=240, FAN=800

### AFCACE_DRY_STOP
Stop the ACE filament dryer.
```
AFCACE_DRY_STOP UNIT=<unit_name>
```

---

## Example Configurations

### Combined Mode (Single Toolhead with Combiner)

```ini
[AFC_hub ace_hub]
switch_pin: virtual

[AFC_extruder extruder]
pin_tool_start: PF4          # Toolhead filament sensor pin
tool_unload_speed: 25

[AFC_AFCACE ace1]
serial_port: /dev/ttyACM0
hub: ace_hub
extruder: extruder
mode: combined
feed_speed: 800
retract_speed: 800
feed_length: 450              # Calibrate with AFCACE_CALIBRATE
retract_length: 470           # Slightly longer than feed_length
use_feed_assist: True
sensor_approach_margin: 60
sensor_step: 20

[AFC_lane lane1]
unit: ace1:1
hub: ace_hub
extruder: extruder

[AFC_lane lane2]
unit: ace1:2
hub: ace_hub
extruder: extruder

[AFC_lane lane3]
unit: ace1:3
hub: ace_hub
extruder: extruder

[AFC_lane lane4]
unit: ace1:4
hub: ace_hub
extruder: extruder
```

### Direct Mode (Multi-Extruder, No Combiner)

In direct mode each lane feeds its own extruder through its own physical path.
Each lane needs its own hub since there is no shared merge point.

```ini
# Each lane has its own hub (separate physical filament paths)
[AFC_hub ace_hub1]
switch_pin: virtual

[AFC_hub ace_hub2]
switch_pin: virtual

[AFC_extruder extruder]
pin_tool_start: PF4
tool_unload_speed: 25

[AFC_extruder extruder1]
pin_tool_start: PF5
tool_unload_speed: 25

[AFC_AFCACE ace1]
serial_port: /dev/ttyACM0
mode: direct
feed_speed: 800
retract_speed: 800
feed_length: 400
retract_length: 420
use_feed_assist: True
# No hub at unit level - each lane defines its own

# Each lane feeds its own extruder through its own hub
[AFC_lane lane1]
unit: ace1:1
hub: ace_hub1
extruder: extruder

[AFC_lane lane2]
unit: ace1:2
hub: ace_hub2
extruder: extruder1
```

### Multi-ACE Setup (8 Slots)

```ini
[AFC_AFCACE ace1]
serial_port: /dev/ttyACM0
hub: ace_hub
extruder: extruder
mode: combined
feed_length: 450
retract_length: 470

[AFC_AFCACE ace2]
serial_port: /dev/ttyACM1
hub: ace_hub
extruder: extruder
mode: combined
feed_length: 500              # Different length - further from toolhead
retract_length: 520

[AFC_lane lane1]
unit: ace1:1
hub: ace_hub
extruder: extruder

# ... lanes 2-4 on ace1 ...

[AFC_lane lane5]
unit: ace2:1
hub: ace_hub
extruder: extruder

# ... lanes 6-8 on ace2 ...
```

### With Turtle Neck Buffer (Ramming Mode)

```ini
[AFC_buffer TN]
# Turtle Neck buffer configuration (see AFC_buffer docs)

[AFC_extruder extruder]
pin_tool_start: buffer          # Use buffer as tool start sensor (enables ramming)
tool_unload_speed: 25
buffer: TN

[AFC_AFCACE ace1]
serial_port: /dev/ttyACM0
hub: ace_hub
extruder: extruder
mode: combined
feed_length: 450
retract_length: 470
buffer: TN                      # Optional: set buffer at unit level

[AFC_lane lane1]
unit: ace1:1
hub: ace_hub
extruder: extruder
# buffer inherited from unit, or override per-lane:
# buffer: TN
```

### With TD-1 Device

```ini
[AFC_AFCACE ace1]
serial_port: /dev/ttyACM0
hub: ace_hub
extruder: extruder
mode: combined
feed_length: 450
retract_length: 470
td1_device_id: ABC123           # TD-1 serial number (applies to all lanes)

[AFC_lane lane1]
unit: ace1:1
hub: ace_hub
extruder: extruder
# td1_device_id inherited from unit, or override per-lane:
# td1_device_id: DEF456
# td1_bowden_length: 380        # Set after running TD-1 calibration
```

---

## Calibration Workflow

1. Load filament into an ACE slot physically
2. Run PREP to validate the slot: `AFC_PREP`
3. Make sure the toolhead is heated to printing temperature
4. Run calibration: `AFCACE_CALIBRATE UNIT=ace1 LANE=lane1`
5. Note the measured distance in the output
6. Update your config with the measured value:
   ```ini
   feed_length: <measured_distance>
   retract_length: <measured_distance + 20>   # Add some margin
   ```
7. Restart Klipper to apply

---

## How It Works

### Loading (combined mode)
1. If another slot is loaded, retract it first
2. Bulk feed `feed_length - sensor_approach_margin` mm in one shot (fast)
3. Feed in `sensor_step` mm increments, checking toolhead sensor between each step
4. When sensor triggers (or max distance reached), enable feed assist
5. Run extruder motor for `extruder_assist_length` mm to pull filament into hotend
6. Verify toolhead sensor is triggered (or buffer compression for ramming mode)
7. If buffer: retract off buffer sensor to confirm load, then enable buffer

### Loading (direct mode)
Same as combined, but skips step 1 (no auto-retract of other slots).

### Loading with Buffer (Ramming Mode)
When `pin_tool_start: buffer` is set on the extruder, the buffer's advance sensor
replaces the toolhead sensor. After feeding reaches the buffer:
1. Filament compresses the buffer (advance_state becomes True)
2. Lane retracts in small steps until buffer decompresses (advance_state False)
3. This confirms filament is properly seated in the toolhead
4. Buffer is enabled for active filament management during printing

### Unloading
1. Disable buffer (if configured)
2. Quick pull (-2mm) to prevent oozing
3. Cut filament (if configured in AFC)
4. Park nozzle (if configured)
5. Form tip (if configured)
6. Retract `retract_length` mm back to ACE unit

### TD-1 Calibration
When a TD-1 device is configured (`td1_device_id`), you can calibrate the distance
from the ACE unit to the TD-1 sensor:
1. Run `AFC_UNIT_TD_ONE_CALIBRATION` from the unit prompt
2. Filament feeds in small increments, polling TD-1 via Moonraker
3. When TD-1 detects filament, the distance is reported
4. Set `td1_bowden_length` in your lane config with the measured value

### Runout Detection
Periodic polling (every `poll_interval` seconds during printing) checks slot status
from ACE hardware. When a slot transitions from "ready" to empty on a TOOLED lane,
triggers AFC's infinite spool system or pauses the print.
