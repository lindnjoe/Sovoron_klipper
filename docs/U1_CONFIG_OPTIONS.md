# U1 Configuration Options

This document covers all configuration options for the ACE, OpenAMS, FPS buffer,
U1 RFID, and temperature sensor modules, as well as new options added to existing
AFC files (lanes, hubs) for U1/OpenAMS/ACE support.

---

## ACE Unit (`[AFC_ACE name]`)

The ACE unit driver communicates directly with Anycubic ACE PRO hardware over USB
serial. It supports two operational modes: `combined` (multiple slots share one
toolhead path) and `direct` (each slot feeds its own extruder).

Inherits all options from `afcUnit` (see `AFC_unit.py`): hub, extruder, buffer,
LED colors, move speeds, etc.

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `serial_port` | string | *(required)* | Serial port path for ACE PRO hardware (e.g. `/dev/ttyACM0`) |
| `mode` | string | `combined` | Operational mode: `combined` (shared toolhead, retract before feed) or `direct` (per-extruder, independent operation) |
| `feed_speed` | float | `60.0` | Feed speed in mm/s (ACE firmware native unit) |
| `retract_speed` | float | `50.0` | Retract speed in mm/s (ACE firmware native unit) |
| `feed_length` | float | `500.0` | Distance in mm from ACE to toolhead for feeding |
| `retract_length` | float | `500.0` | Distance in mm to retract filament back to ACE |
| `unload_preretract` | float | `50.0` | Distance in mm for ACE rewind before cut to tighten spool |
| `dist_hub` | float | `200.0` | Slot-to-hub/combiner distance in mm, used for two-phase loading |
| `load_to_hub` | bool | *(None -- inherits from AFC global)* | Unit-level override for load-to-hub behavior; can also be overridden per-lane |
| `use_feed_assist` | bool | `True` | Default enable/disable feed assist for all slots |
| `extruder_assist_length` | float | `50.0` | Distance in mm to advance with extruder motor during feed assist after filament reaches toolhead sensor area |
| `extruder_assist_speed` | float | `300.0` | Extruder assist speed in mm/min |
| `sensor_approach_margin` | float | `30.0` | Distance in mm before expected sensor position to switch to incremental feeding |
| `sensor_step` | float | `40.0` | Distance in mm per check during sensor approach |
| `calibration_step` | float | `50.0` | Distance in mm per check during calibration |
| `max_feed_overshoot` | float | `100.0` | Extra distance in mm to try past feed_length before giving up |
| `dock_purge` | bool | `False` | Enable dock purge during load (drop tool at dock, load filament, purge, then pick up) |
| `dock_purge_length` | float | `50.0` | Distance in mm of filament to extrude while docked for purging |
| `dock_purge_speed` | float | `5.0` | Extrude speed in mm/s during dock purge |
| `auto_spoolman_create` | bool | `False` | When True, automatically create filaments and spools in Spoolman from RFID data. When False, RFID data still matches existing entries but won't create new ones |
| `fps_threshold` | float | `0.9` | FPS ADC value threshold for toolhead sensor detection (0.0-1.0) |
| `fps_load_threshold` | float | `0.65` | Lower FPS threshold used during active operations (loading/calibration); the latch prevents false clears |
| `fps_delta_threshold` | float | `0.15` | Minimum jump between consecutive FPS readings to count as filament engagement |
| `fps_confirm_count` | int | `3` | Number of consecutive ADC readings above threshold required before sensor is considered triggered (min: 1) |
| `poll_interval` | float | `1.0` | Sensor polling interval in seconds for status/runout monitoring |
| `baud_rate` | int | `115200` | Serial baud rate for ACE communication |

### ACE Per-Lane Overrides (in `[AFC_lane]` sections)

When a lane's `unit` references an ACE unit (e.g. `unit: Ace_1:1`), these
additional options can be set in the `[AFC_lane]` section to override the
unit-level defaults:

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `feed_length` | float | *(unit default)* | Per-lane override for feed distance in mm |
| `retract_length` | float | *(unit default)* | Per-lane override for retract distance in mm |
| `dist_hub` | float | *(unit default)* | Per-lane override for slot-to-hub distance in mm |
| `use_feed_assist` | bool | *(unit default)* | Per-lane override for feed assist enable/disable |
| `auto_spoolman_create` | bool | *(unit default)* | Per-lane override for automatic Spoolman creation from RFID |

---

## ACE Serial (`AFC_ACE_serial`)

The ACE serial module (`AFC_ACE_serial.py`) handles low-level binary-framed
JSON-RPC communication with ACE PRO hardware. It does **not** read any Klipper
config options directly. All serial parameters (port, baud rate) are passed
programmatically from the `AFC_ACE` unit.

Protocol constants (not configurable via config, defined in code):

| Constant | Value | Description |
|----------|-------|-------------|
| `DEFAULT_BAUD` | `115200` | Default baud rate |
| `REQUEST_TIMEOUT` | `5.0` | Default command timeout in seconds |
| `HEARTBEAT_INTERVAL` | `2.0` | Seconds between heartbeats (must be < 3s to prevent USB autosuspend) |
| `RECONNECT_BACKOFF_MIN` | `5.0` | Initial reconnection delay in seconds |
| `RECONNECT_BACKOFF_MAX` | `30.0` | Maximum reconnection delay in seconds |

---

## OpenAMS Unit (`[AFC_OpenAMS name]`)

The OpenAMS unit driver synchronizes state with OpenAMS hardware (MCU-based
filament changer with encoder, hall-effect sensors, and follower motor).

Inherits all options from `afcUnit` (see `AFC_unit.py`): hub, extruder, buffer,
LED colors, move speeds, etc.

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `oams` | string | `oams1` | Name of the OpenAMS hardware object to bind to |
| `stuck_spool_auto_recovery` | bool | `False` | When True, a stuck spool during printing triggers automatic unload + reload + resume instead of just pausing |
| `stuck_spool_load_grace` | float | `8.0` | Grace period in seconds after load completes before stuck spool detection activates (min: 0.0) |
| `stuck_spool_pressure_threshold` | float | `0.08` | FPS pressure below this value (combined with encoder stopped) indicates a stuck spool (min: 0.0) |
| `engagement_pressure_threshold` | float | `0.6` | Maximum FPS pressure during engagement verification (min: 0.0) |
| `engagement_min_pressure` | float | `0.25` | Minimum FPS pressure required to confirm filament engagement (min: 0.0) |
| `clog_sensitivity` | string | `medium` | Clog detection sensitivity: `off`, `low`, `medium`, or `high` |
| `dock_purge` | bool | `False` | Enable dock purge during load (drop tool at dock, load, purge, pick up) |
| `dock_purge_length` | float | `105.0` | Distance in mm of filament to extrude while docked for purging (min: 0.0) |
| `dock_purge_speed` | float | `7.0` | Extrude speed in mm/s during dock purge (min: 0.0) |

---

## OpenAMS Follower (`AFC_OpenAMS_follower`)

The follower controller (`AFC_OpenAMS_follower.py`) manages follower motor
control, LED error state, and rate-limited MCU command queuing for OpenAMS
hardware units. It does **not** read any Klipper config options directly. It is
instantiated programmatically by `AFC_OpenAMS` and receives OAMS hardware
references at runtime.

---

## OpenAMS Monitor (`AFC_OpenAMS_monitor`)

The OpenAMS monitor (`AFC_OpenAMS_monitor.py`) provides real-time stuck spool
and clog detection during printing. It does **not** read any Klipper config
options directly. It is instantiated by `AFC_OpenAMS` with parameters derived
from the unit's config (e.g. `clog_sensitivity`, `stuck_spool_load_grace`).

Detection thresholds are defined as module-level constants:

| Constant | Value | Description |
|----------|-------|-------------|
| `MONITOR_INTERVAL` | `2.0` | Seconds between monitoring checks during printing |
| `MONITOR_INTERVAL_IDLE` | `4.0` | Slower interval when not printing |
| `STUCK_PRESSURE_LOW` | `0.08` | FPS below this + encoder stopped = stuck |
| `STUCK_PRESSURE_CLEAR` | `0.12` | FPS must rise above this to clear stuck condition |
| `STUCK_DWELL` | `2.0` | Seconds condition must persist before firing |
| `STUCK_LOAD_GRACE` | `8.0` | Grace period after load completes |
| `STUCK_MIN_ENCODER` | `3` | Minimum encoder clicks per check to be considered moving |
| `CLOG_PRESSURE_TARGET` | `0.50` | Expected FPS pressure during normal printing |
| `CLOG_PRESSURE_BAND` | `0.06` | Deadband around target |
| `CLOG_EXTRUSION_WINDOW` | `24.0` | mm of extrusion before checking for clog |
| `CLOG_DWELL` | `10.0` | Seconds clog condition must persist |
| `CLOG_POST_LOAD_GRACE` | `12.0` | Grace period after load before clog detection starts |
| `CLOG_ENCODER_SLACK` | `8` | Encoder clicks of slack allowed |

Clog sensitivity presets (set via `clog_sensitivity` on the OpenAMS unit):

| Preset | Multiplier | Description |
|--------|-----------|-------------|
| `off` | *(disabled)* | No clog detection |
| `low` | `1.5` | Longer dwell before clog triggers |
| `medium` | `1.0` | Default sensitivity |
| `high` | `0.7` | Shorter dwell, more aggressive detection |

---

## FPS Buffer (`[AFC_FPS name]`)

The FPS (Filament Position Sensor) buffer driver reads an analog sensor via ADC
and provides proportional buffer control. For stepper-based units it adjusts
rotation distance; for non-stepper units (OpenAMS, ACE) it provides ADC readings
and sensor state only.

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `adc_pin` | string | *(required)* | MCU pin for the analog FPS sensor |
| `debug` | bool | `False` | Enable debug logging for this buffer |
| `sample_count` | int | `5` | Number of ADC samples to average per reading |
| `sample_time` | float | `0.005` | Time in seconds between individual ADC samples |
| `report_time` | float | `0.100` | Interval in seconds between ADC callback reports |
| `reversed` | bool | `False` | Invert the FPS reading (1.0 becomes 0.0 and vice versa) |
| `set_point` | float | `0.5` | Target FPS value for buffer equilibrium (0.1-0.9) |
| `low_point` | float | `0.1` | FPS value indicating buffer is stretched / not feeding enough (0.0-0.5) |
| `high_point` | float | `0.9` | FPS value indicating buffer is compressed / pushing too much (0.5-1.0) |
| `multiplier_high` | float | `1.15` | Maximum rotation distance multiplier when buffer is trailing (min: 1.0) |
| `multiplier_low` | float | `0.85` | Minimum rotation distance multiplier when buffer is advancing (0.0-1.0) |
| `trailing_min_multiplier` | float | `1.05` | Floor multiplier applied when buffer is in trailing state (min: 1.0) |
| `deadband` | float | `0.30` | Total width of the neutral window centered on set_point; no correction within this range (0.0-0.6) |
| `smoothing` | float | `0.3` | Exponential moving average factor (0.0 = no smoothing, 0.95 = max smoothing) |
| `update_interval` | float | `0.25` | Timer interval in seconds for applying corrections (min: 0.05) |
| `flip_cooldown` | float | `180.0` | Cooldown in seconds to suppress rapid direction changes (min: 0.0) |
| `filament_error_sensitivity` | float | `0` | Fault detection sensitivity (0 = disabled, 1-10 = increasing sensitivity) |
| `led_index` | string | `None` | LED index for buffer state indication |
| `led_buffer_advancing` | string | `0,0,1,0` | LED RGBW color when buffer is advancing (compressed) |
| `led_buffer_trailing` | string | `0,1,0,0` | LED RGBW color when buffer is trailing (stretched) |
| `led_buffer_neutral` | string | `0,0.5,0.5,0` | LED RGBW color when buffer is neutral |
| `led_buffer_disable` | string | `0,0,0,0.25` | LED RGBW color when buffer is disabled |
| `enable_sensors_in_gui` | bool | *(AFC global)* | Show virtual filament sensors in Mainsail/Fluidd GUI |

---

## U1 RFID (`AFC_U1_rfid`)

The U1 RFID module (`AFC_U1_rfid.py`) polls the Snapmaker U1 `filament_detect`
Klipper module for RFID tag data and applies it to AFC lanes. It does **not**
read Klipper config options directly. Instead, it is instantiated
programmatically by AFC and lanes register themselves with their RFID channel
number (set via `u1_rfid_channel` in `[AFC_lane]` sections).

The RFID channel mapping is configured per-lane (see "New Options in Existing
Files" below).

---

## Temperature Sensors

### temperature_ace (`sensor_type: temperature_ace`)

Reads ACE device temperature from an `AFC_ACE` unit's cached hardware status and
exposes it as a standard Klipper temperature sensor.

Configuration is placed inside a `[temperature_sensor]` section:

```ini
[temperature_sensor ace_temp]
sensor_type: temperature_ace
ace_unit: Ace1
min_temp: 0
max_temp: 70
```

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `ace_unit` | string | `Ace1` | Name of the AFC_ACE unit to read temperature from |
| `min_temp` | float | `0` | Minimum allowed temperature before shutdown (set via Klipper's `setup_minmax`) |
| `max_temp` | float | `70` | Maximum allowed temperature before shutdown (set via Klipper's `setup_minmax`) |

### temperature_oams (`sensor_type: temperature_oams`)

HDC1080 I2C temperature and humidity sensor for OpenAMS units. Reads temperature
and humidity via I2C and exposes them as a standard Klipper temperature sensor.

Configuration is placed inside a `[temperature_sensor]` section:

```ini
[temperature_sensor oams1]
sensor_type: temperature_oams
i2c_mcu: oams_mcu1
i2c_bus: i2c0
i2c_speed: 200000
min_temp: 0
max_temp: 100
```

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `i2c_mcu` | string | *(required)* | MCU name for I2C communication (standard Klipper I2C config) |
| `i2c_bus` | string | *(required)* | I2C bus name (standard Klipper I2C config) |
| `i2c_speed` | int | `100000` | I2C bus speed in Hz (standard Klipper I2C config, default via `MCU_I2C_from_config`) |
| `report_time` | int | `5` | Seconds between temperature/humidity samples (min: 5) |
| `temp_resolution` | int | `14` | Temperature measurement resolution in bits: `11` or `14` |
| `humidity_resolution` | int | `14` | Humidity measurement resolution in bits: `8`, `11`, or `14` |
| `temp_offset` | float | `0.0` | Offset added to temperature readings for calibration |
| `humidity_offset` | float | `0.0` | Offset added to humidity readings for calibration |
| `heater_enabled` | bool | `False` | Enable the HDC1080's built-in heater element (used to drive off condensation) |
| `min_temp` | float | `0` | Minimum allowed temperature before shutdown (set via Klipper's `setup_minmax`) |
| `max_temp` | float | `100` | Maximum allowed temperature before shutdown (set via Klipper's `setup_minmax`) |

---

## New Options in Existing Files

These are new config options added to existing AFC files to support U1/OpenAMS/ACE
integration.

### AFC_lane (`[AFC_lane name]`)

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `u1_rfid_channel` | int | `-1` | U1 `filament_detect` RFID channel index for this lane. Set to `0`, `1`, `2`, etc. to enable RFID polling for this lane. `-1` disables RFID for the lane |

### AFC_hub (`[AFC_hub name]`)

| Option | Type | Default Change | Description |
|--------|------|----------------|-------------|
| `switch_pin` | string | Changed from `None` to `virtual` | Hub sensor pin. Now defaults to `virtual` to support units without physical hub sensors (ACE, OpenAMS use virtual hub sensors) |
