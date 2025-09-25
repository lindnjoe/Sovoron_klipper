
# Sovoron Klipper AFC & OpenAMS Integration Guide

This repository packages the firmware modules, configuration, and macros required to run Sovoron printers with both the Automatic Filament Changer (AFC) and the OpenAMS multi-spool management system. It combines the upstream `klipper_openams` Python extension with a complete Klipper configuration tailored for dual OpenAMS units feeding an AFC-based toolchanger.


## Table of Contents
1. [System Architecture](#system-architecture)
   1. [Software Components](#software-components)
   2. [Hardware & MCU Topology](#hardware--mcu-topology)
   3. [Filament Path & Monitoring](#filament-path--monitoring)
2. [Integration Workflow](#integration-workflow)
   1. [Spool Loading & Switchover](#spool-loading--switchover)

   2. [Runout and Stuck Spool Handling](#runout-and-stuck-spool-handling)

   3. [Macros Bridging AFC and OpenAMS](#macros-bridging-afc-and-openams)
3. [Configuration Reference](#configuration-reference)
   1. [OpenAMS Core (`printer_data/config/oamsc.cfg`)](#openams-core-printer_dataconfigoamsccfg)
   2. [OpenAMS Macros (`printer_data/config/oams_macros.cfg`)](#openams-macros-printer_dataconfigoams_macroscfg)
   3. [AFC Core Settings (`printer_data/config/AFC/AFC.cfg`)](#afc-core-settings-printer_dataconfigafcafccfg)
   4. [AFC Macro Variables (`printer_data/config/AFC/AFC_Macro_Vars.cfg`)](#afc-macro-variables-printer_dataconfigafcafc_macro_varscfg)
   5. [AFC Hardware Definition (`printer_data/config/AFC/AFC_Hardware.cfg`)](#afc-hardware-definition-printer_dataconfigafcafc_hardwarecfg)
   6. [AFC AMS/Lane Mapping (`printer_data/config/AFC/AFC_AMS*.cfg`)](#afc-amslane-mapping-printer_dataconfigafcafc_amscfg)
   7. [Box Turtle & Buffers (`printer_data/config/AFC/AFC_Turtle_1.cfg`)](#box-turtle--buffers-printer_dataconfigafcafc_turtle_1cfg)
   8. [Pico Coprocessor (`printer_data/config/AFC/AFC_Pico.cfg`)](#pico-coprocessor-printer_dataconfigafcafc_picocfg)
   9. [AFC Convenience Macros (`printer_data/config/AFC/macros/*.cfg`)](#afc-convenience-macros-printer_dataconfigafcmacroscfg)
4. [Extending the Setup](#extending-the-setup)
   1. [Adding Additional OpenAMS Units](#adding-additional-openams-units)
   2. [Customising AFC Behaviour](#customising-afc-behaviour)
   3. [Debugging & Monitoring Tips](#debugging--monitoring-tips)

---

## System Architecture

### Software Components

- **`klipper_openams` extension** – Implements the runtime logic that coordinates spool loading, runout recovery, and stuck-spool monitoring across all filament pressure sensors (FPS) and AMS hubs. The `oams_manager.py` module provides configurable thresholds, timers, and LED control hooks that interact with the hardware macros defined in the Klipper configuration.

- **Klipper configuration** – Located under `printer_data/config`, it declares all MCUs, stepper lanes, hubs, macros, and gcode variables that tie the OpenAMS hardware to the AFC system. The configuration is separated into logical files so each subsystem can be tuned independently.

### Hardware & MCU Topology
- **OpenAMS MCUs** – `oamsc.cfg` declares the CAN-connected MCUs for each FPS and hub pair (`[mcu fps]`, `[mcu oams_mcu1]`, and matching entries for the second AMS). Multiple filament pressure sensors can be defined to support dual extruders.
- **AFC controllers** – `AFC_Turtle_1.cfg` and `AFC_Pico.cfg` register the Box Turtle CAN board and RP2040-based buffer expander that drive lane steppers, hub motors, and buffer sensors. Each lane maps to an AFC stepper with its own TMC2209 driver configuration and optional buffer logic.

### Filament Path & Monitoring
- **Filament groups & FPS definitions** – `oamsc.cfg` maps toolchanger tools (`T4`–`T11`) into OpenAMS filament groups and binds each group to an FPS (`fps1`/`fps2`) and extruder. The `reload_before_toolhead_distance` value per FPS determines how early the manager coasts followers before a swap.

- **Pressure and encoder tracking** – `klipper_openams/src/oams_manager.py` tracks encoder counts and FPS pressure, automatically retrying failed loads, pausing when loading speeds stall, and latching LEDs when a stuck spool is detected. The current baseline no longer performs automatic clog detection inside the manager.


## Integration Workflow

### Spool Loading & Switchover
1. **Lane selection** – AFC lanes are mapped to OpenAMS units (`[AFC_lane]` sections) so that commands like `CHANGE_TOOL` or `_TX1` select the correct group before requesting a load.
2. **Load execution** – `OAMSM_LOAD_FILAMENT` gcode is emitted by the `_TX1`/`_TX2` macros. The OpenAMS manager advances the designated hub until the FPS reaches the configured midpoint between `fps_upper_threshold` and `fps_lower_threshold`.
3. **Toolhead priming** – After a successful load, macros pull filament into the toolhead using the `_oams_macro_variables` lengths and optionally trigger purge, wipe, or brush cycles defined in AFC macro variables.


### Runout and Stuck Spool Handling
- **Runout** – When an FPS detects hub HES sensors dropping out, `OAMSRunoutMonitor` transitions through pause, coast, and reload states before pulling the next spool in the group.
- **Stuck spool detection** – While printing, the stuck spool monitor watches FPS pressure for sustained readings below `STUCK_SPOOL_PRESSURE_THRESHOLD` for the configured dwell window. If the pressure stays low long enough, the system pauses, asserts the spool LED, and waits for the user to resume after clearing the jam. Resuming or switching spools automatically clears the latch, resets the follower, and turns the LED off.
- **Clog detection** – The baseline manager no longer performs extruder/encoder-based clog checks. Any future clog monitoring should be implemented in macros or external modules without relying on the previous in-manager logic.


### Macros Bridging AFC and OpenAMS
- **Safe unload routines** – `SAFE_UNLOAD_FILAMENT1/2` macros coordinate nozzle heating, cutter actuation, follower reversal, and hub unload commands for each AMS.
- **Transfer macros** – `_TX1` and `_TX2` manage changeovers by unloading the currently loaded group (if any), loading the requested OpenAMS group, and priming the toolhead before returning to print moves.
- **Convenience wrappers** – `BT_CHANGE_TOOL`, `BT_LANE_EJECT`, and related macros provide high-level commands for tool changes and manual lane operations while respecting the underlying AFC lane naming conventions.

## Configuration Reference

### OpenAMS Core (`printer_data/config/oamsc.cfg`)

#### MCU & Sensor Blocks
| Section | Settings | Purpose |
| --- | --- | --- |
| `[mcu fps]` | `canbus_uuid` | CAN address for the first filament pressure sensor MCU. |
| `[mcu oams_mcu1]` | `canbus_uuid` | CAN address for the first OpenAMS hub MCU. |
| `[mcu fps2]` | `canbus_uuid` | Optional second FPS MCU for dual extruders. |
| `[mcu oams_mcu2]` | `canbus_uuid` | Optional second OpenAMS hub MCU. |
| `[filament_switch_sensor extruder_in1]` | `switch_pin`, `pause_on_runout` | Optional inline filament sensor for the first toolhead. |
| `[filament_switch_sensor extruder_in2]` | `switch_pin`, `pause_on_runout` | Optional inline filament sensor for the second toolhead. |

#### Macro Variable Block
| Section | Variable | Description |
| --- | --- | --- |
| `[gcode_macro _oams_macro_variables]` | `variable_pre_cut_x`, `variable_pre_cut_y`, `variable_pre_cut_speed`, `variable_cut_x`, `variable_cut_y`, `variable_cut_speed` | Cutter entry positions and speeds. |
|  | `variable_hotend_meltzone_compensation`, `variable_retract_length`, `variable_extrusion_reload_length`, `variable_extrusion_unload_length`, `variable_reload_speed` | Toolhead extrusion lengths and speeds for load/unload cycles. |
|  | `variable_fs_extruder_in1`, `variable_fs_extruder_out1`, `variable_fs_extruder_in2`, `variable_fs_extruder_out2` | Enable flags for auxiliary filament sensors per extruder. |

#### OpenAMS Unit Blocks
| Section | Key Settings | Description |
| --- | --- | --- |
| `[oams oams1]` & `[oams oams2]` | `fps_upper_threshold`, `fps_lower_threshold`, `fps_is_reversed`, `f1s_hes_on`, `f1s_hes_is_above`, `hub_hes_on`, `hub_hes_is_above`, `ptfe_length`, `current_target`, `current_kp`, `current_ki`, `current_kd`, `oams_idx` | Tune how each AMS responds to FPS pressure, HES sensors, PTFE length, and rewind PID. |

#### Filament Groups & FPS Definitions
| Section | Settings | Description |
| --- | --- | --- |
| `[filament_group T4]` … `[filament_group T11]` | `group` | Map toolchanger tools to OpenAMS spool groups. |
| `[fps fps1]` & `[fps fps2]` | `pin`, `reversed`, `oams`, `extruder`, `reload_before_toolhead_distance` | Bind an FPS sensor to its AMS, extruder, and follower safety margin. |

#### Manager & Includes
| Section | Settings | Description |
| --- | --- | --- |
| `[oams_manager]` | (commented) `UNLOAD_RETRY_NUDGE_TIME`, `reload_before_toolhead_distance` | Optional overrides for retry timing and follower safety margin. |
| `[include oams_macros.cfg]` | — | Pulls in all macros that cooperate with the manager. |

### OpenAMS Macros (`printer_data/config/oams_macros.cfg`)

| Macro | Role |
| --- | --- |
| `SAFE_UNLOAD_FILAMENT1`, `SAFE_UNLOAD_FILAMENT2` | Heat the nozzle, actuate the cutter, command follower reversal, and unload the active spool for AMS 1 and 2 respectively. |
| `_TX1`, `_TX2` | High-level transfer macros that unload the current group, request an OpenAMS load (`OAMSM_LOAD_FILAMENT`), prime the extruder, and restore print state for each FPS. |
| `_UNLOAD_FS_OUT1`, `_UNLOAD_FS_OUT2`, `_LOAD_FS_IN1`, `_LOAD_FS_OUT1` (and similar helpers) | Internal helper macros that synchronise auxiliary filament sensors with hub and toolhead movement. |

### AFC Core Settings (`printer_data/config/AFC/AFC.cfg`)

#### Speed & Motion
| Setting | Description |
| --- | --- |
| `long_moves_speed`, `long_moves_accel` | Feed rate/acceleration for long lane moves. |
| `short_moves_speed`, `short_moves_accel`, `short_move_dis` | Parameters for short failsafe moves used during retries. |
| `load_to_hub` | Global toggle for automatically fast-loading filament to the hub when a spool is inserted. |
| `assisted_unload` | Whether the hub assists retracts during unload to prevent loose windings. |
| `unload_on_runout` | Enables automatic unload on runout if no fallback spool is configured. |
| `print_short_stats`, `debug`, `auto_home`, `debounce_delay` | Runtime logging and safety toggles. |

#### Material & Sensor Defaults
| Setting | Description |
| --- | --- |
| `enable_sensors_in_gui` | Expose sensors in the UI for monitoring. |
| `default_material_temps`, `common_density_values`, `default_material_type` | Default extrusion parameters when Spoolman data is unavailable. |

#### Pause/Resume Behaviour
| Setting | Description |
| --- | --- |
| `z_hop`, `resume_speed`, `resume_z_speed`, `error_timeout` | Head movement and timeout behaviour around tool changes and errors. |

#### LED Colours
| Setting | Description |
| --- | --- |
| `led_name`, `led_fault`, `led_ready`, `led_not_ready`, `led_loading`, `led_tool_loaded`, `led_buffer_advancing`, `led_buffer_trailing`, `led_buffer_disable`, `led_spool_illuminate` | Neopixel colours used to indicate AFC state transitions. |

#### Macro Enablement
| Setting | Description |
| --- | --- |
| `tool_cut`, `tool_cut_threshold`, `tool_cut_cmd` | Enable and configure cutter usage during unload. |
| `park`, `park_cmd` | Enable toolhead park routine. |
| `poop`, `poop_cmd`, `kick`, `kick_cmd`, `wipe`, `wipe_cmd`, `form_tip`, `form_tip_cmd` | Toggle purge, kick, brush, and tip-form routines. |

#### Startup & Tip Forming
| Section | Key Settings |
| --- | --- |
| `[AFC_prep]` | `enable` toggles the PREP routine executed by `delayed_gcode welcome`. |
| `[AFC_form_tip]` | `ramming_volume`, `toolchange_temp`, `unloading_speed_start`, `unloading_speed`, `cooling_tube_position`, `cooling_tube_length`, `initial_cooling_speed`, `final_cooling_speed`, `cooling_moves`, `use_skinnydip`, `skinnydip_distance`, `dip_insertion_speed`, `dip_extraction_speed`, `melt_zone_pause`, `cooling_zone_pause`. |

### AFC Macro Variables (`printer_data/config/AFC/AFC_Macro_Vars.cfg`)

#### `_AFC_GLOBAL_VARS`
| Variable | Description |
| --- | --- |
| `variable_stepper_name` | Prefix for AFC lane steppers (e.g., `lane`). |
| `variable_travel_speed`, `variable_z_travel_speed`, `variable_accel` | Default travel speeds and acceleration for macro moves. |
| `variable_verbose` | Console verbosity level (0–2). |

#### `_AFC_CUT_TIP_VARS`
| Variable | Description |
| --- | --- |
| `variable_pin_loc_xy`, `variable_pin_park_dist`, `variable_cut_move_dist` | Cutter positioning parameters. |
| `variable_cut_accel`, `variable_cut_direction`, `variable_cut_fast_move_speed`, `variable_cut_slow_move_speed`, `variable_evacuate_speed`, `variable_cut_dwell_time`, `variable_cut_fast_move_fraction`, `variable_extruder_move_speed` | Motion profile for cutting. |
| `variable_restore_position`, `variable_retract_length`, `variable_quick_tip_forming`, `variable_cut_count`, `variable_rip_length`, `variable_rip_speed`, `variable_pushback_length`, `variable_pushback_dwell_time` | Behavioural tweaks for retracting and tip shaping. |
| `variable_safe_margin_xy` | Safety envelope when approaching the cutter. |
| `variable_cut_current_stepper_x`, `variable_cut_current_stepper_y`, `variable_cut_current_stepper_z`, `variable_conf_name_stepper_x`, `variable_conf_name_stepper_y`, `variable_conf_name_stepper_z` | Optional current overrides and matching TMC section names. |
| `variable_tool_servo_enable`, `variable_tool_servo_name`, `variable_tool_servo_angle_out`, `variable_tool_servo_angle_in` | Servo configuration for blade actuation. |

#### `_AFC_POOP_VARS`
| Variable | Description |
| --- | --- |
| `variable_purge_loc_xy`, `variable_purge_spd`, `variable_z_purge_move`, `variable_fast_z`, `variable_z_lift` | Purge path geometry and speeds. |
| `variable_restore_position`, `variable_purge_start` | Toolhead positioning before/after purging. |
| `variable_part_cooling_fan`, `variable_part_cooling_fan_speed`, `variable_purge_cool_time` | Cooling fan usage during purge. |
| `variable_purge_length`, `variable_purge_length_minimum` (+ optional modifiers) | Filament length heuristics for purging. |

#### `_AFC_KICK_VARS`
| Variable | Description |
| --- | --- |
| `variable_kick_start_loc`, `variable_kick_z`, `variable_kick_speed`, `variable_kick_accel`, `variable_kick_direction`, `variable_kick_move_dist`, `variable_z_after_kick` | Parameters for the kick routine that knocks purge blobs off the deck. |

#### `_AFC_BRUSH_VARS`
| Variable | Description |
| --- | --- |
| `variable_brush_loc`, `variable_brush_clean_speed`, `variable_brush_clean_accel`, `variable_brush_width`, `variable_brush_depth`, `variable_y_brush`, `variable_brush_count`, `variable_z_move` | Define brush location, speeds, passes, and optional Z hop. |

#### `_AFC_PARK_VARS`
| Variable | Description |
| --- | --- |
| `variable_park_loc_xy`, `variable_z_hop`, `variable_park_z` | Position and Z management during park moves. |

### AFC Hardware Definition (`printer_data/config/AFC/AFC_Hardware.cfg`)

| Section | Key Settings |
| --- | --- |
| `[force_move]` | `enable_force_move` enables manual jogs with unloaded steppers. |
| `[AFC_extruder extruder*]` | `pin_tool_start`, optional `pin_tool_end`, `tool_stn`, `tool_stn_unload`, `tool_sensor_after_extruder`, `tool_unload_speed`, `tool_load_speed`, `deadband` define toolhead sensor geometry and speeds for each extruder channel. |
| `[#filament_switch_sensor bypass]` | Template for an additional bypass sensor if needed. |

### AFC AMS/Lane Mapping (`printer_data/config/AFC/AFC_AMS*.cfg`)

| Section | Purpose |
| --- | --- |
| `[gcode_button T4_unload_button]`, `[gcode_button T5_unload_button]` | Hardware buttons that trigger `SAFE_UNLOAD_FILAMENT` routines when the printer is idle. |
| `[AFC_AMS AMS_1]`, `[AFC_AMS AMS_2]` | Bind an OpenAMS unit (`oams1`/`oams2`) to the AFC extruder channel that services it. |
| `[AFC_lane lane4]` … `[AFC_lane lane11]` | Assign individual lanes to AMS slots, define LED indices, hub IDs, tool mappings, and custom load/unload macros per lane. |
| `[AFC_hub Hub_1]` … `[AFC_hub Hub_8]` | Configure bowden lengths, movement distances, and cutter availability for each hub. |

### Box Turtle & Buffers (`printer_data/config/AFC/AFC_Turtle_1.cfg`)

| Section | Highlights |
| --- | --- |
| `[AFC_BoxTurtle Turtle_1]` | Enables assist and kick-start behaviour for the Box Turtle hub controller and optionally links buffers. |
| `[AFC_stepper lane0]` … `[AFC_stepper lane3]` | Define CAN pins, rotation distances, hub associations, buffers, and tool mappings for the four primary AFC lanes serviced by the Turtle board. |
| `[tmc2209 AFC_stepper lane*]` | UART pins and current settings for each lane’s stepper driver. |
| `[AFC_buffer TN*]` | Buffer advance/trailing sensors and gain multipliers (including those hosted on the Pico coprocessor). |
| `[AFC_led AFC_Indicator]`, `[AFC_led AFC_Tndicator]`, `[AFC_led AFC_Sndicator]` | LED strips that visualise AFC state per AMS. |

### Pico Coprocessor (`printer_data/config/AFC/AFC_Pico.cfg`)

| Section | Highlights |
| --- | --- |
| `[mcu Pico]` | Registers the USB-connected RP2040 board that supplies additional GPIO for buffer sensors. |

### AFC Convenience Macros (`printer_data/config/AFC/macros/*.cfg`)

| Macro | Description |
| --- | --- |
| `BT_TOOL_UNLOAD` | Wrapper that issues `TOOL_UNLOAD` to drop the current filament. |
| `BT_CHANGE_TOOL` | Calculates the lane name from `_AFC_GLOBAL_VARS.variable_stepper_name` and calls `CHANGE_TOOL`. |
| `BT_LANE_EJECT` | Fully ejects filament from a specified lane. |
| `BT_LANE_MOVE` | Manually advances or retracts a lane by a user-specified distance. |
| `BT_RESUME` | Ensures AFC resume logic runs before resuming a paused print. |
| `BT_PREP` | Runs the AFC PREP sequence via the `PREP` macro. |

## Extending the Setup

### Adding Additional OpenAMS Units
- Duplicate the `[mcu fps]`, `[mcu oams_mcu]`, `[oams]`, `[filament_group]`, and `[fps]` sections with new UUIDs and group names. Ensure each new FPS is mapped to an extruder and lane macros reference the expanded group list.

### Customising AFC Behaviour
- Adjust macro variables in `AFC_Macro_Vars.cfg` to tune cutter timing, purge lengths, and brush routines for your specific toolhead and waste handling setup.
- Modify lane definitions in `AFC_Turtle_1.cfg` to change hub assignments, add buffers, or alter LED indices when you relocate hardware.

### Debugging & Monitoring Tips
- Follow the comments in `oamsc.cfg` to calibrate FPS reversal, HES thresholds, and PTFE lengths. Use `tail -f ~/printer_data/logs/klippy.log` during calibration.
- Inspect the AFC LEDs defined in `AFC.cfg` and `AFC_Turtle_1.cfg` to confirm status changes during loads, unloads, and error states.
- Use the `SAFE_UNLOAD_FILAMENT` macros when clearing jams—resuming the print will clear stuck spool latches and LEDs automatically.

---


With these references, you can tune every aspect of the Sovoron AFC + OpenAMS integration, extend it to additional lanes or AMS units, and understand how the runtime code and macros cooperate to keep filament flowing reliably.
