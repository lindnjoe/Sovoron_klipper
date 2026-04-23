# AFC Configuration Reference

Complete reference for all AFC-Klipper-Add-On configuration options with defaults and example configs.

---

## Table of Contents

1. [Core AFC Configuration](#core-afc-configuration)
2. [Hub Configuration](#hub-configuration)
3. [Extruder Configuration](#extruder-configuration)
4. [Lane Configuration](#lane-configuration)
5. [Buffer Configuration](#buffer-configuration)
6. [FPS Buffer Configuration](#fps-buffer-configuration)
7. [Toolchanger Configuration](#toolchanger-configuration)
8. [Unit Types](#unit-types)
   - [BoxTurtle](#boxturtle)
   - [NightOwl](#nightowl)
   - [QuattroBox](#quattrobox)
   - [ACE](#ace)
   - [OpenAMS](#openams)
9. [Example Configs](#example-configs)
   - [BoxTurtle with TurtleNeck Buffer](#example-boxturtle-with-turtleneck-buffer)
   - [BoxTurtle with FPS Buffer](#example-boxturtle-with-fps-buffer)
   - [ACE Combined Mode](#example-ace-combined-mode)
   - [ACE Direct Mode with FPS Buffer](#example-ace-direct-mode-with-fps-buffer)
   - [OpenAMS](#example-openams)
   - [Toolchanger with BoxTurtle](#example-toolchanger-with-boxturtle)

---

## Core AFC Configuration

Section: `[AFC]`

### Temperature & Heating

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `default_material_temps` | list | `PLA:210, PETG:235, ABS:235, ASA:235` | Default temperatures per material type |
| `default_material_type` | string | None | Default material type for new spools |
| `temp_wait_tolerance` | float | `5.0` | Temperature tolerance (C) for M109 wait |
| `deadband_auto` | bool | `True` | Auto-use extruder deadband for M109 when D= not specified |
| `restore_extruder_temp_on_load_or_unload` | bool | `False` | Restore extruder target temp after load/unload when not printing |
| `lower_extruder_temp_on_change` | bool | `True` | Lower extruder temp during filament change if above target |
| `toolchange_temp_drop` | float | `0` | Degrees to drop old extruder temp after toolchange |
| `ignore_spoolman_material_temps` | bool | `False` | Ignore Spoolman temps, use default_material_temps instead |

### Movement

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `long_moves_speed` | float | `100` | Speed (mm/s) for long filament moves |
| `long_moves_accel` | float | `400` | Acceleration (mm/s^2) for long moves |
| `short_moves_speed` | float | `25` | Speed (mm/s) for short filament moves |
| `short_moves_accel` | float | `400` | Acceleration (mm/s^2) for short moves |
| `short_move_dis` | float | `10` | Distance (mm) for failsafe moves |
| `max_move_dis` | float | `999999` | Max distance per move before splitting |
| `z_hop` | float | `0` | Z-hop height (mm) during tool change |
| `resume_speed` | float | `25` | Resume move speed (mm/s), 0 = use gcode speed |
| `resume_z_speed` | float | `25` | Resume Z move speed (mm/s) |
| `rev_long_moves_speed_factor` | float | `1.0` | Speed factor when reversing filament |
| `quiet_moves_speed` | float | `50` | Max speed (mm/s) in quiet mode |

### Homing

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `homing_enabled` | bool | `True` | Enable homing |
| `home_to_hub` | bool | `True` | Auto-home to hub during moves |
| `home_to_tool` | bool | `True` | Auto-home to tool during moves |
| `load_then_home` | bool | `True` | Load then home |
| `load_undershoot` | float | `20` | Load undershoot distance (mm) |
| `tool_homing_distance` | float | `200` | Distance (mm) over which toolhead homing is attempted |
| `disable_homing_check` | bool | `False` | Disable homing check during toolchanges (toolchanger only) |

### Toolhead Operations

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `tool_cut` | bool | `False` | Enable toolhead cutting |
| `tool_cut_cmd` | string | None | Macro for toolhead cutting |
| `park` | bool | `False` | Enable parking during unload |
| `park_cmd` | string | None | Macro for parking |
| `kick` | bool | `False` | Enable poop kicking after load |
| `kick_cmd` | string | None | Macro for kicking |
| `wipe` | bool | `False` | Enable nozzle wiping after load |
| `wipe_cmd` | string | None | Macro for wiping |
| `poop` | bool | `False` | Enable purging after load |
| `poop_cmd` | string | None | Macro for purging |
| `form_tip` | bool | `False` | Enable tip forming on unload |
| `form_tip_cmd` | string | None | Macro for tip forming |
| `post_load_macro` | string | None | Macro after lane load |
| `post_unload_macro` | string | None | Macro after lane unload |

### Retry Limits

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `tool_max_unload_attempts` | int | `4` | Max unload attempts with buffer as ram sensor |
| `tool_max_load_checks` | int | `4` | Max load checks with buffer as ram sensor |
| `max_move_tries` | int | `20` | Max move retries |
| `error_timeout` | float | `36000` | Seconds to pause before erroring out |

### LEDs

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `led_fault` | string | `1,0,0,0` | Color (R,G,B,W) for fault state |
| `led_ready` | string | `0,0.8,0,0` | Color for ready state |
| `led_not_ready` | string | `1,0,0,0` | Color for not ready state |
| `led_loading` | string | `1,1,1,0` | Color during loading |
| `led_unloading` | string | `1,1,.5,0` | Color during unloading |
| `led_tool_loaded` | string | `0,0,1,0` | Color when loaded in tool |
| `led_tool_loaded_idle` | string | `0.4,0.4,0,0` | Color when loaded in tool, idle |
| `led_tool_unloaded` | string | `1,0,0,0` | Color when extruder unloaded |
| `led_buffer_advancing` | string | `0,0,1,0` | Color when buffer advancing |
| `led_buffer_trailing` | string | `0,1,0,0` | Color when buffer trailing |
| `led_buffer_disable` | string | `0,0,0,0.25` | Color when buffer disabled |
| `led_spool_illuminate` | string | `1,1,1,1` | Color for spool illumination |

### Moonraker / Spoolman

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `moonraker_host` | string | `http://localhost` | Moonraker host |
| `moonraker_port` | string | `7125` | Moonraker port |
| `moonraker_timeout` | string | `30` | Moonraker timeout (seconds) |
| `disable_weight_check` | bool | `False` | Disable weight check on filament load |
| `disable_ooze_check` | bool | `False` | Disable ooze check in M104/M109 |

### Miscellaneous

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `VarFile` | string | `../printer_data/config/AFC/AFC.var` | Path to AFC variables file |
| `auto_home` | bool | `False` | Auto home if printer not homed |
| `auto_level_macro` | string | None | Bed leveling macro to run with auto_home |
| `show_macros` | bool | `True` | Show AFC_ macros in web GUI |
| `show_quiet_mode` | bool | `True` | Show quiet mode option |
| `enable_sensors_in_gui` | bool | `False` | Show sensor switches in GUI |
| `enable_assist` | bool | `True` | Enable assist functionality |
| `enable_assist_weight` | float | `500` | Spool weight threshold for assist |
| `enable_hub_runout` | bool | `True` | Enable hub runout detection |
| `enable_tool_runout` | bool | `True` | Enable tool runout detection |
| `debounce_delay` | float | `0.0` | Sensor debounce delay |
| `load_to_hub` | bool | `True` | Auto-load filament to hub on insert |
| `assisted_unload` | bool | `True` | Assist unload to prevent loose windings |
| `global_print_current` | float | None | Stepper current during printing |
| `full_weight` | float | `1000` | Default full filament weight (g) |
| `spool_ratio` | float | `2` | Gear ratio for N20 spooler |
| `n20_break_delay_time` | float | `0.200` | N20 motor brake delay (seconds) |
| `debug` | bool | `False` | Enable debug logging |

---

## Hub Configuration

Section: `[AFC_hub hub_name]`

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `switch_pin` | string | `virtual` | Hub sensor pin (`virtual` for units without physical hub) |
| `afc_bowden_length` | float | `900` | Bowden length hub-to-toolhead (mm) |
| `afc_unload_bowden_length` | float | `afc_bowden_length` | Unload retract distance (mm) |
| `hub_clear_move_dis` | float | `65` | Move distance to clear hub exit (mm) |
| `move_dis` | float | `75` | Filament move distance within hub (mm) |
| `assisted_retract` | bool | `False` | Assist retracts to prevent loose windings |
| `cut` | bool | `False` | Hub cutter installed (e.g. Snappy) |
| `cut_cmd` | string | None | Macro for hub cut |
| `cut_dist` | float | `50` | Filament to cut off (mm) |
| `cut_clear` | float | `120` | Retract distance after cut (mm) |
| `cut_min_length` | float | `200` | Min filament length to cut |
| `cut_servo_name` | string | `cut` | Servo name for cutter |
| `cut_servo_pass_angle` | float | `0` | Servo angle for Bowden pass-through |
| `cut_servo_clip_angle` | float | `160` | Servo angle for cutting |
| `cut_servo_prep_angle` | float | `75` | Servo angle for cut prep |
| `cut_confirm` | bool | `False` | Cut filament twice |

---

## Extruder Configuration

Section: `[AFC_extruder extruder_name]`

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `pin_tool_start` | string | None | Sensor pin before extruder gears |
| `pin_tool_end` | string | None | Sensor pin after extruder gears (optional) |
| `tool_stn` | float | `72` | Sensor-to-nozzle distance (mm) |
| `tool_stn_unload` | float | `100` | Unload move distance (mm) |
| `tool_sensor_after_extruder` | float | `0` | Extra distance once sensors clear (mm) |
| `tool_unload_speed` | float | `25` | Unload speed (mm/s) |
| `tool_load_speed` | float | `25` | Load speed (mm/s) |
| `buffer` | string | None | Buffer name for this extruder |
| `deadband` | float | `2` | Heater deadband (C) |
| `toolchange_temp_drop` | float | AFC global | Temp drop after toolchange (C) |

### Toolchanger-Specific Extruder Options

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `toolchanger_unit` | string | None | Toolchanger unit name |
| `tool_number` | int | `-1` | Physical tool number |
| `gcode_x_offset` | float | `0.0` | X gcode offset |
| `gcode_y_offset` | float | `0.0` | Y gcode offset |
| `gcode_z_offset` | float | `0.0` | Z gcode offset |
| `t_command_restore_axis` | string | `XYZ` | Axes to restore after tool change |
| `tool_fan` | string | None | Fan name for this tool |
| `detection_pin` | string | None | Tool detection pin |
| `resonance_chip` | string | None | Input shaper chip |
| `custom_tool_swap` | string | None | Custom tool swap gcode |
| `custom_unselect` | string | None | Custom unselect gcode |
| `pickup_gcode` | string | `''` | Tool pickup gcode template |
| `dropoff_gcode` | string | `''` | Tool dropoff gcode template |
| `before_change_gcode` | string | `''` | Before tool change gcode template |
| `after_change_gcode` | string | `''` | After tool change gcode template |

---

## Lane Configuration

Section: `[AFC_lane lane_name]`

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `unit` | string | **Required** | Unit name (e.g. `Turtle_1` or `ace1:1` for ACE slot) |
| `hub` | string | None | Hub override (default: unit's hub) |
| `extruder` | string | None | Extruder name |
| `buffer` | string | None | Buffer override |
| `map` | string | None | T-command mapping (e.g. `T0`, `T5`) |
| `prep` | string | None | Prep sensor pin |
| `load` | string | None | Load sensor pin |
| `dist_hub` | float | `60` | Distance to hub (mm) |
| `load_to_hub` | bool | AFC global | Auto-load to hub on insert |
| `led_index` | string | None | LED index in chain |

### Per-Lane Overrides (inherit from unit/AFC global)

All movement, homing, and LED options from `[AFC]` can be overridden per-lane.

---

## Buffer Configuration

Section: `[AFC_buffer buffer_name]` (TurtleNeck physical buffer)

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `advance_pin` | string | **Required** | Advance endstop pin |
| `trailing_pin` | string | **Required** | Trailing endstop pin |
| `distance` | float | **Required** | Buffer distance (mm) |
| `multiplier_high` | float | `1.1` | Speed multiplier when advancing (min 1.0) |
| `multiplier_low` | float | `0.9` | Speed multiplier when trailing (0.0-1.0) |
| `filament_error_sensitivity` | float | `0` | Error sensitivity (0-10, 0=disabled) |
| `led_index` | string | None | LED index for buffer status |
| `debug` | bool | `False` | Enable debug output |

---

## FPS Buffer Configuration

Section: `[AFC_FPS buffer_name]` (Filament Pressure Sensor ADC buffer)

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `adc_pin` | string | **Required** | ADC pin for pressure sensor |
| `sample_count` | int | `5` | ADC samples per reading |
| `sample_time` | float | `0.005` | Time between ADC samples (seconds) |
| `report_time` | float | `0.100` | Time between state reports (seconds) |
| `reversed` | bool | `False` | Reverse ADC polarity |
| `set_point` | float | `0.5` | ADC setpoint for buffer control (0.1-0.9) |
| `low_point` | float | `0.1` | Low ADC threshold (0.0-0.5) |
| `high_point` | float | `0.9` | High ADC threshold (0.5-1.0) |
| `multiplier_high` | float | `1.1` | Speed multiplier when advancing (min 1.0) |
| `multiplier_low` | float | `0.9` | Speed multiplier when trailing (0.0-1.0) |
| `deadband` | float | `0.30` | ADC deadband (0.0-0.6) |
| `smoothing` | float | `0.3` | ADC value smoothing factor (0.0-0.95) |
| `update_interval` | float | `0.25` | Buffer state update interval (seconds, min 0.05) |
| `filament_error_sensitivity` | float | `0` | Error sensitivity (0-10, 0=disabled) |
| `debug` | bool | `False` | Enable debug output |

---

## Toolchanger Configuration

Section: `[AFC_Toolchanger unit_name]`

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `uses_axis` | string | `xyz` | Axes used by toolchanger (must be homed) |
| `require_tool_present` | bool | `True` | Require tool on shuttle during init |
| `verify_tool_pickup` | bool | `True` | Verify tool detection after pickup |
| `transfer_fan_speed` | bool | `True` | Transfer fan speed during tool change |
| `initialize_gcode` | string | `''` | Initialization gcode template |
| `error_gcode` | string | None | Error recovery gcode template |
| `before_change_gcode` | string | `''` | Default before-change gcode (per-tool overrides) |
| `after_change_gcode` | string | `''` | Default after-change gcode (per-tool overrides) |
| `pickup_gcode` | string | `''` | Default pickup gcode (per-tool overrides) |
| `dropoff_gcode` | string | `''` | Default dropoff gcode (per-tool overrides) |
| `params_*` | varies | varies | Default params inherited by all tools |

---

## Unit Types

### BoxTurtle

Section: `[AFC_BoxTurtle unit_name]`

Stepper-driven filament changer with physical sensors. No unit-specific config options beyond what's inherited from `afcUnit`.

### NightOwl

Section: `[AFC_NightOwl unit_name]`

Inherits BoxTurtle. No additional config options.

### QuattroBox

Section: `[AFC_QuattroBox unit_name]`

Inherits NightOwl/BoxTurtle. No additional config options.

### ACE

Section: `[AFC_ACE unit_name]`

Serial-connected filament changer with RFID support.  Supports two modes:
`combined` (multiple slots share one extruder via combiner) or `direct`
(each slot feeds its own extruder).

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `serial_port` | string | **Required** | Serial port for ACE hardware |
| `mode` | string | `combined` | `combined` (multi-slot, one extruder) or `direct` (each slot has own extruder) |
| `feed_speed` | float | `60` | Feed speed in mm/s (ACE firmware native unit) |
| `retract_speed` | float | `50` | Retract speed in mm/s (ACE firmware native unit) |
| `feed_length` | float | `500` | ACE-to-toolhead distance (mm) |
| `retract_length` | float | `500` | Retract distance back to ACE (mm) |
| `unload_preretract` | float | `50` | ACE rewind before cut to tighten spool (mm) |
| `dist_hub` | float | `200` | Slot-to-hub/combiner distance (mm). Calibrate with ACE_CALIBRATE_HUB |
| `load_to_hub` | bool | None | Override AFC global load_to_hub |
| `use_feed_assist` | bool | `True` | Enable feed assist for all slots |
| `extruder_assist_length` | float | `50` | Extruder motor assist distance after filament reaches sensor (mm) |
| `extruder_assist_speed` | float | `300` | Extruder assist speed (mm/min) |
| `sensor_approach_margin` | float | `30` | Distance before sensor for incremental feed (mm) |
| `sensor_step` | float | `40` | Per-check distance during sensor approach (mm) |
| `calibration_step` | float | `50` | Per-check distance during calibration (mm) |
| `max_feed_overshoot` | float | `100` | Extra distance past feed_length before giving up (mm) |
| `dock_purge` | bool | `False` | Enable dock purge during load |
| `dock_purge_length` | float | `50` | Extrude length while docked (mm) |
| `dock_purge_speed` | float | `5` | Extrude speed while docked (mm/s) |
| `auto_spoolman_create` | bool | `False` | Auto-create filaments/spools in Spoolman from RFID |
| `fps_threshold` | float | `0.9` | FPS trigger threshold (0-1) for toolhead sensor |
| `fps_load_threshold` | float | `0.65` | FPS threshold during active operations (loading/calibration) |
| `fps_delta_threshold` | float | `0.15` | Min FPS jump between readings for engagement detection |
| `fps_confirm_count` | int | `3` | Consecutive ADC reads above threshold to confirm trigger |
| `poll_interval` | float | `1.0` | Sensor/status polling interval (seconds) |
| `baud_rate` | int | `115200` | Serial baud rate |

**ACE Lane-Level Overrides** (in `[AFC_lane]` sections):

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `feed_length` | float | unit default | Per-lane feed distance |
| `retract_length` | float | unit default | Per-lane retract distance |
| `dist_hub` | float | unit default | Per-lane hub distance |
| `use_feed_assist` | bool | unit default | Per-lane feed assist |
| `auto_spoolman_create` | bool | unit default | Per-lane auto Spoolman create |

### OpenAMS

Section: `[AFC_OpenAMS unit_name]`

OpenAMS units communicate with OAMS firmware over Klipper's CAN/serial bus.
The OAMS hub motor and firmware PID loop handle filament feeding and tension.
Only AFC_FPS buffers are supported (TurtleNeck mechanical buffers are ignored).

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `oams` | string | `oams1` | OpenAMS hardware instance name (must match `[AFC_oams]` section) |
| `stuck_spool_auto_recovery` | bool | `False` | Auto unload + reload + resume on stuck spool (vs pause for manual intervention) |
| `stuck_spool_load_grace` | float | `8.0` | Grace distance/time threshold for stuck spool detection during loading |
| `stuck_spool_pressure_threshold` | float | `0.08` | FPS pressure value above which the spool is considered stuck |
| `engagement_pressure_threshold` | float | `0.6` | Max FPS pressure for filament engagement detection |
| `engagement_min_pressure` | float | `0.25` | Min FPS pressure before engagement is considered |
| `clog_sensitivity` | string | `medium` | Clog detection sensitivity level for OAMS monitor |
| `dock_purge` | bool | `False` | Enable dock purge during load (requires toolchanger) |
| `dock_purge_length` | float | `105` | Extrude length while docked for purging (mm) |
| `dock_purge_speed` | float | `7` | Extrude speed while docked (mm/s) |

---

## Example Configs

### Example: BoxTurtle with TurtleNeck Buffer

```ini
[AFC]
long_moves_speed: 100
short_moves_speed: 25
z_hop: 0.5
poop: True
poop_cmd: AFC_POOP
kick: True
kick_cmd: AFC_KICK
wipe: True
wipe_cmd: AFC_BRUSH
form_tip: True
form_tip_cmd: AFC_FORM_TIP

[AFC_BoxTurtle Turtle_1]

[AFC_hub hub1]
switch_pin: nhk:gpio13
afc_bowden_length: 900
move_dis: 75

[AFC_buffer TN1]
advance_pin: nhk:gpio6
trailing_pin: nhk:gpio9
distance: 80
multiplier_high: 1.05
multiplier_low: 0.95

[AFC_extruder extruder]
pin_tool_start: nhk:gpio2
tool_stn: 72
tool_stn_unload: 100
buffer: TN1

[AFC_lane lane0]
unit: Turtle_1:1
hub: hub1
extruder: extruder
map: T0
prep: mcu:gpio1
load: mcu:gpio2
dist_hub: 60
led_index: AFC_led:1

[AFC_lane lane1]
unit: Turtle_1:2
hub: hub1
extruder: extruder
map: T1
prep: mcu:gpio3
load: mcu:gpio4
dist_hub: 60
led_index: AFC_led:2

[AFC_lane lane2]
unit: Turtle_1:3
hub: hub1
extruder: extruder
map: T2
prep: mcu:gpio5
load: mcu:gpio6
dist_hub: 60
led_index: AFC_led:3

[AFC_lane lane3]
unit: Turtle_1:4
hub: hub1
extruder: extruder
map: T3
prep: mcu:gpio7
load: mcu:gpio8
dist_hub: 60
led_index: AFC_led:4
```

### Example: BoxTurtle with FPS Buffer

```ini
[AFC]
long_moves_speed: 100
short_moves_speed: 25
z_hop: 0.5
poop: True
poop_cmd: AFC_POOP

[AFC_BoxTurtle Turtle_1]

[AFC_hub hub1]
switch_pin: nhk:gpio13
afc_bowden_length: 900

[AFC_FPS FPS_buffer1]
adc_pin: nhk:gpio29
set_point: 0.5
low_point: 0.1
high_point: 0.9
multiplier_high: 1.1
multiplier_low: 0.9
deadband: 0.30
smoothing: 0.3

[AFC_extruder extruder]
pin_tool_start: FPS_buffer1
tool_stn: 72
buffer: FPS_buffer1

[AFC_lane lane0]
unit: Turtle_1:1
hub: hub1
extruder: extruder
map: T0
prep: mcu:gpio1
load: mcu:gpio2
dist_hub: 60
```

### Example: ACE Combined Mode

```ini
[AFC]
long_moves_speed: 100
short_moves_speed: 25
z_hop: 0.5
poop: True
poop_cmd: AFC_POOP
wipe: True
wipe_cmd: AFC_BRUSH

[AFC_ACE ace1]
serial_port: /dev/ttyACM0
mode: combined
feed_speed: 2500
retract_speed: 2500
feed_length: 1800
retract_length: 1800
dist_hub: 800
dock_purge: True
dock_purge_length: 105
dock_purge_speed: 7
auto_spoolman_create: True

[AFC_hub hub1]
switch_pin: virtual
afc_bowden_length: 1800

[AFC_buffer TN1]
advance_pin: nhk:gpio6
trailing_pin: nhk:gpio9
distance: 80

[AFC_extruder extruder]
pin_tool_start: nhk:gpio2
tool_stn: 72
buffer: TN1

[AFC_lane lane0]
unit: ace1:1
hub: hub1
extruder: extruder
map: T0

[AFC_lane lane1]
unit: ace1:2
hub: hub1
extruder: extruder
map: T1

[AFC_lane lane2]
unit: ace1:3
hub: hub1
extruder: extruder
map: T2

[AFC_lane lane3]
unit: ace1:4
hub: hub1
extruder: extruder
map: T3
```

### Example: ACE Direct Mode with FPS Buffer

```ini
[AFC]
long_moves_speed: 100
short_moves_speed: 25
z_hop: 0.5

[AFC_ACE ace1]
serial_port: /dev/ttyACM0
mode: direct
feed_speed: 2500
retract_speed: 2500
feed_length: 2600
retract_length: 2600
dist_hub: 800
dock_purge: True
dock_purge_length: 105
dock_purge_speed: 7
auto_spoolman_create: True

[AFC_Toolchanger Tools]
uses_axis: xyz
require_tool_present: False
verify_tool_pickup: True

[AFC_hub hub1]
switch_pin: virtual
afc_bowden_length: 2600

[AFC_FPS FPS_buffer1]
adc_pin: nhk:gpio29

[AFC_extruder extruder5]
pin_tool_start: FPS_buffer1
tool_stn: 72
buffer: FPS_buffer1
toolchanger_unit: AFC_Toolchanger Tools
tool_number: 5

# Each slot maps to its own extruder
[AFC_lane lane12]
unit: ace1:1
hub: hub1
extruder: extruder5
map: T12
feed_length: 2600

[AFC_lane lane13]
unit: ace1:2
hub: hub1
extruder: extruder5
map: T13
feed_length: 2800

[AFC_lane lane14]
unit: ace1:3
hub: hub1
extruder: extruder5
map: T14
feed_length: 2400

[AFC_lane lane15]
unit: ace1:4
hub: hub1
extruder: extruder5
map: T15
feed_length: 2400
```

### Example: OpenAMS

Each OpenAMS lane has its own hub sensor (Hall Effect Sensor per bay on the OAMS hardware), so each lane needs its own `[AFC_hub]` with a unique pin.

```ini
[AFC]
long_moves_speed: 100
short_moves_speed: 25
z_hop: 0.5

[AFC_OpenAMS oams_unit1]
oams: oams1

# Each lane gets its own hub — OAMS has per-bay Hall Effect sensors
[AFC_hub hub_oams1_bay1]
switch_pin: oams1:HES_0
afc_bowden_length: 3077

[AFC_hub hub_oams1_bay2]
switch_pin: oams1:HES_1
afc_bowden_length: 3077

[AFC_hub hub_oams1_bay3]
switch_pin: oams1:HES_2
afc_bowden_length: 3077

[AFC_hub hub_oams1_bay4]
switch_pin: oams1:HES_3
afc_bowden_length: 3077

[AFC_FPS FPS_buffer1]
adc_pin: nhk:gpio29

[AFC_extruder extruder4]
pin_tool_start: FPS_buffer1
tool_stn: 72
buffer: FPS_buffer1
toolchanger_unit: AFC_Toolchanger Tools
tool_number: 4

[AFC_lane lane4]
unit: oams_unit1:1
hub: hub_oams1_bay1
extruder: extruder4
map: T4

[AFC_lane lane5]
unit: oams_unit1:2
hub: hub_oams1_bay2
extruder: extruder4
map: T5

[AFC_lane lane6]
unit: oams_unit1:3
hub: hub_oams1_bay3
extruder: extruder4
map: T6

[AFC_lane lane7]
unit: oams_unit1:4
hub: hub_oams1_bay4
extruder: extruder4
map: T7
```

### Example: Toolchanger with BoxTurtle

```ini
[AFC]
z_hop: 5
poop: True
poop_cmd: AFC_POOP
kick: True
kick_cmd: AFC_KICK
wipe: True
wipe_cmd: AFC_BRUSH
disable_homing_check: True

[AFC_Toolchanger Tools]
uses_axis: xyz
require_tool_present: False
verify_tool_pickup: True
transfer_fan_speed: True

initialize_gcode:

error_gcode:
    M112

before_change_gcode:
after_change_gcode:

pickup_gcode:
    {% if tool.extruder and printer[tool.extruder].target|int > 0 %}
      M109 S{printer[tool.extruder].target|int} EXTRUDER={tool.tool_number}
    {% endif %}

dropoff_gcode:
    STOP_TOOL_CRASH_DETECTION

[AFC_BoxTurtle Turtle_1]
[AFC_BoxTurtle Turtle_2]

[AFC_hub hub1]
switch_pin: nhk1:gpio13
afc_bowden_length: 900

[AFC_hub hub2]
switch_pin: nhk2:gpio13
afc_bowden_length: 900

[AFC_buffer TN1]
advance_pin: nhk1:gpio6
trailing_pin: nhk1:gpio9
distance: 80

[AFC_buffer TN2]
advance_pin: nhk2:gpio6
trailing_pin: nhk2:gpio9
distance: 80

[AFC_extruder extruder]
pin_tool_start: nhk1:gpio2
tool_stn: 72
buffer: TN1
toolchanger_unit: AFC_Toolchanger Tools
tool_number: 0

[AFC_extruder extruder1]
pin_tool_start: nhk2:gpio2
tool_stn: 72
buffer: TN2
toolchanger_unit: AFC_Toolchanger Tools
tool_number: 1

# Turtle_1 lanes -> extruder (tool 0)
[AFC_lane lane0]
unit: Turtle_1:1
hub: hub1
extruder: extruder
map: T0

[AFC_lane lane1]
unit: Turtle_1:2
hub: hub1
extruder: extruder
map: T1

[AFC_lane lane2]
unit: Turtle_1:3
hub: hub1
extruder: extruder
map: T2

[AFC_lane lane3]
unit: Turtle_1:4
hub: hub1
extruder: extruder
map: T3

# Turtle_2 lanes -> extruder1 (tool 1)
[AFC_lane lane4]
unit: Turtle_2:1
hub: hub2
extruder: extruder1
map: T4

[AFC_lane lane5]
unit: Turtle_2:2
hub: hub2
extruder: extruder1
map: T5

[AFC_lane lane6]
unit: Turtle_2:3
hub: hub2
extruder: extruder1
map: T6

[AFC_lane lane7]
unit: Turtle_2:4
hub: hub2
extruder: extruder1
map: T7
```
