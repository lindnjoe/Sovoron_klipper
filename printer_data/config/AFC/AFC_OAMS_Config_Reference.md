# AFC + OpenAMS Configuration Reference

Quick reference for all config sections used in the Sovoron OpenAMS setup.
Config keys marked **bold** are ones we've explicitly set or tuned.

---

## `[oams <name>]` — AFC_OpenAMS.py

One section per OAMS unit (`oams1`, `oams2`).

| Config Key | Default | oams1 | oams2 | Description |
|---|---|---|---|---|
| `mcu` | *(required)* | `oams_mcu1` | `oams_mcu2` | MCU assignment |
| `oams_idx` | *(required)* | `1` | `2` | OAMS index (used in logging and filament groups) |
| `load_retry_max` | — | `3` | `3` | Max load attempts before error |
| `unload_retry_max` | — | `2` | `2` | Max unload attempts before error |
| `retry_delay` | — | `6.0` | `3.0` | Delay between retries (s) |
| `auto_unload_on_failed_load` | — | `True` | `True` | Auto unload before retrying a failed load |
| `dock_load` | — | `True` | `True` | Enable dock-based loading |
| **`post_load_purge`** | — | `75` | `75` | Post-load purge amount (mm) |
| `extra_retract` | `10.0` | `-10` | `-10` | Extra retract distance on unload (mm) |
| `ptfe_length` | — | `2060` | `2057` | PTFE tube length — critical, must match physical tube (mm) |
| `fps_upper_threshold` | — | `0.7` | `0.7` | FPS HES value above which hub motor runs at full speed |
| `fps_lower_threshold` | — | `0.3` | `0.3` | FPS HES value below which hub motor stops |
| `fps_is_reversed` | — | `true` | `true` | Invert FPS polarity (set true if unloaded FPS reads ~1.0) |
| `f1s_hes_on` | — | `0.1, 0.1, 0.1, 0.1` | `0.1, 0.1, 0.1, 0.1` | Per-lane F1S HES threshold for "spool loaded" |
| `f1s_hes_is_above` | — | `false` | `false` | If true, HES must be *above* threshold to trigger; if false, *below* |
| **`hub_hes_on`** | — | `0.825845, 0.809982, 0.837821, 0.813116` | `0.791565, 0.797029, 0.800661, 0.786823` | Per-lane hub HES thresholds — calibrated with `OAMS_CALIBRATE_HUB_HES` |
| `hub_hes_is_above` | — | `true` | `true` | Hub HES comparison direction |
| `current_target` | — | `0.30` | `0.30` | Rewind motor current target (0.0–1.0) |
| `current_kp` | — | `3.0` | `3.0` | Rewind PID — proportional gain |
| `current_ki` | — | `0.0` | `0.0` | Rewind PID — integral gain |
| `current_kd` | — | `0.0` | `0.0` | Rewind PID — derivative gain |
| **`stuck_spool_auto_recovery`** | `False` | *(not set)* | *(not set)* | Auto unload+reload+resume on stuck spool detection *(our addition)* |

---

## `[fps <name>]` — oams_manager.py / AFC_OpenAMS.py

One section per Filament Pressure Sensor (`fps1`, `fps2`).

| Config Key | Default | fps1 | fps2 | Description |
|---|---|---|---|---|
| `pin` | *(required)* | `fps:PA2` | `fps2:PA2` | FPS sensor ADC pin |
| `reversed` | `false` | `false` | `false` | Invert FPS polarity |
| `oams` | *(required)* | `oams1` | `oams2` | Parent OAMS unit |
| `extruder` | *(required)* | `extruder4` | `extruder5` | Toolhead extruder this FPS feeds |
| **`reload_before_toolhead_distance`** | `0.0` | `60` | `60` | mm remaining in PTFE before toolhead at which to trigger a reload |

---

## `[oams_manager]` — oams_manager.py

Single global section.

| Config Key | Default | Our Value | Min | Max | Description |
|---|---|---|---|---|---|
| **`stuck_spool_load_grace`** | `8.0` | `8.0` | 0.0 | 60.0 | Seconds after load completes before stuck detection activates |
| `stuck_spool_max_attempts` | *(constant)* | — | 1 | 5 | Max auto-recovery attempts before giving up |
| **`stuck_spool_pressure_threshold`** | *(constant)* | `0.06` | 0.0 | 1.0 | FPS pressure below which stuck detection fires |
| `stuck_spool_pressure_clear_threshold` | *(constant)* | — | 0.0 | 1.0 | FPS pressure above which stuck state clears |
| **`engagement_pressure_threshold`** | `0.6` | `0.6` | 0.0 | 1.0 | Minimum FPS pressure to consider filament engaged |
| `clog_pressure_target` | *(constant)* | — | 0.0 | 1.0 | FPS pressure target for clog detection |
| `post_load_pressure_dwell` | *(constant)* | — | 0.0 | 60.0 | Dwell time after load for pressure to settle (s) |
| `load_fps_stuck_threshold` | *(constant)* | — | 0.0 | 1.0 | FPS threshold for detecting stuck-during-load |
| **`crash_detection`** | `"0"` | `tool` | — | — | Crash detection mode (`tool` = detect via toolchanger, `0` = disabled) |
| `enable_clog_detection` | `True` | — | — | — | Enable clog detection |
| `enable_stuck_spool_detection` | `True` | — | — | — | Enable stuck spool detection |
| `clog_sensitivity` | `DEFAULT` | — | — | — | Clog sensitivity preset |
| ~~`moonraker_host`~~ | `http://localhost` | — | — | — | *Deprecated — set in `[AFC]` instead* |
| ~~`moonraker_port`~~ | `7125` | — | 1 | 65535 | *Deprecated — set in `[AFC]` instead* |

---

## `[AFC]` — AFC.py

Global AFC settings. Many can be overridden per-unit (`[AFC_OpenAMS]`) or per-lane (`[AFC_lane]`).

### Moonraker / General

| Config Key | Default | Description |
|---|---|---|
| `moonraker_host` | `http://localhost` | Moonraker host URL |
| `moonraker_port` | `7125` | Moonraker port |
| `moonraker_timeout` | `30` | Moonraker connection timeout (s) |
| `VarFile` | `../printer_data/config/AFC/AFC.var` | Path to AFC variables file |
| `debug` | `False` | Enable verbose debug logging |
| `testing` | `False` | Testing mode — logs moves without executing them |

### Motion

| Config Key | Default | Description |
|---|---|---|
| `long_moves_speed` | `100` | Fast move speed (mm/s) |
| `long_moves_accel` | `400` | Fast move acceleration (mm/s²) |
| `short_moves_speed` | `25` | Short move speed (mm/s) |
| `short_moves_accel` | `400` | Short move acceleration (mm/s²) |
| `short_move_dis` | `10` | Short move distance (mm) |
| `tool_homing_distance` | `200` | Max homing travel distance (mm) |
| `z_hop` | `0` | Z hop height on tool change (mm) |
| `xy_resume` | `False` | Return to saved XY position after resume |
| `resume_speed` | *(from speed)* | XY resume move speed (mm/s) |
| `resume_z_speed` | *(from speed)* | Z resume move speed (mm/s) |
| `rev_long_moves_speed_factor` | `1.0` | Speed multiplier for reverse long moves |
| `quiet_moves_speed` | `50` | Speed in quiet mode (mm/s) |

### Homing & Loading

| Config Key | Default | Description |
|---|---|---|
| `home_to_hub` | `True` | Home filament to hub sensor |
| `home_to_tool` | `True` | Home filament to toolhead sensor |
| `homing_enabled` | `True` | Enable filament homing |
| `load_to_hub` | `True` | Load only to hub (not all the way to toolhead) |
| `assisted_unload` | `True` | Use stepper to assist during unload |
| `tool_max_unload_attempts` | `4` | Max unload retry attempts |
| `tool_max_load_checks` | `4` | Max load sensor checks before error |
| `max_move_tries` | `20` | Max move retries |
| `disable_homing_check` | `False` | Skip homing check before moves |

### Runout & Sensors

| Config Key | Default | Description |
|---|---|---|
| `enable_hub_runout` | `True` | Detect runout at hub sensor |
| `enable_tool_runout` | `True` | Detect runout at toolhead sensor |
| `unload_on_runout` | `False` | Auto-unload on runout detection |
| `debounce_delay` | `0.0` | Sensor debounce delay (s) |
| `enable_sensors_in_gui` | `False` | Show sensors in Mainsail/Fluidd GUI |
| `pause_when_bypass_active` | `False` | Pause print when bypass is active |
| `error_timeout` | `36000` | Seconds before a pause becomes an error (s) |

### Tool Change Macros

| Config Key | Default | Description |
|---|---|---|
| `park` | `False` | Enable toolhead parking on change |
| `park_cmd` | `None` | Macro name for park move |
| `poop` | `False` | Enable purge bucket |
| `poop_cmd` | `None` | Macro name for purge |
| `kick` | `False` | Enable filament kick |
| `kick_cmd` | `None` | Macro name for kick |
| `wipe` | `False` | Enable nozzle wipe |
| `wipe_cmd` | `None` | Macro name for wipe |
| `form_tip` | `False` | Enable tip forming on unload |
| `form_tip_cmd` | `None` | Macro name for tip form |
| `tool_cut` | `False` | Enable filament cutter |
| `tool_cut_cmd` | `None` | Macro name for cut |
| `post_load_macro` | `None` | Macro to run after filament loads |
| `post_unload_macro` | `None` | Macro to run after filament unloads |
| `auto_home` | `False` | Auto-home printer before tool change |
| `auto_level_macro` | `None` | Macro to call for auto-levelling |

### Spoolman / Weight

| Config Key | Default | Description |
|---|---|---|
| `global_print_current` | `None` | Override stepper current during print (A) |
| `spool_ratio` | `2` | Spool diameter ratio for weight estimation |
| `full_weight` | `1000` | Full spool weight for estimation (g) |
| `disable_weight_check` | `False` | Disable spool weight check |
| `ignore_spoolman_material_temps` | `False` | Ignore Spoolman material temperatures |

---

## `[AFC_OpenAMS <name>]` — AFC_OpenAMS.py

Per-unit AFC integration settings (sits alongside the `[oams]` section).

| Config Key | Default | Our Value | Description |
|---|---|---|---|
| `oams` | `oams1` | `oams1` / `oams2` | Which `[oams]` unit this AFC unit maps to |
| `extruder` | *(required)* | `extruder4` / `extruder5` | Toolhead extruder |
| `capture_td1_when_loaded` | `False` | `True` | Capture TD-1 filament data when lane is loaded |
| `td1_device_id` | `None` | *(device id)* | TD-1 sensor device ID |
| `stuck_spool_auto_recovery` | `False` | *(not set)* | Enable auto unload+reload+resume on stuck spool *(our addition)* |

---

## Notes

- Keys marked *(constant)* have defaults defined as named constants inside `oams_manager.py` — check the source to find exact values.
- `stuck_spool_auto_recovery` is a feature added in this branch; it is not yet in upstream AFC or OpenAMS.
- `hub_hes_on` values must be calibrated per-unit using `OAMS_CALIBRATE_HUB_HES`. Do not copy between units.
- `ptfe_length` is critical — wrong values cause load failures or over-feeding. Calibrate with `UNIT_PTFE_CALIBRATION`.
- Several `[AFC]` motion keys (`long_moves_speed`, `short_move_dis`, etc.) can be overridden in `[AFC_lane]` for per-lane tuning.
