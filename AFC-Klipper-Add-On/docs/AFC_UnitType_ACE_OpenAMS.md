# AFC ACE & OpenAMS Unit Type Configuration

These sections document the `[AFC_ACE]` and `[AFC_OpenAMS]` unit types for the
AFC-Klipper-Add-On.  Both inherit base options from `afcUnit` (LED colors, move
speeds, homing, espooler, etc.) — only unit-specific options are listed here.

---

## [AFC_ACE unit_name] Section

The following options are available in the `[AFC_ACE unit_name]` section.  These
options control the configuration of the AFC system when interfacing with
Anycubic ACE PRO hardware via direct serial communication.

AFC_ACE supports two operational modes:

- **combined** — Multiple ACE slots share one toolhead path through a combiner/splitter.
  The unit retracts the current slot before feeding a new one.
- **direct** — Each ACE slot feeds its own extruder independently.

AFC_ACE inherits many LED, speed/acceleration, and misc options from the base
`afcUnit` class (same as `[AFC_BoxTurtle]`), but does not use stepper- or
espooler-specific options (ACE has no per-lane stepper).  Below are the
configuration values specific to an ACE unit.

!!! note
    ACE feed/retract speeds are in **mm/s** (matching the ACE firmware's native unit).
    This differs from stepper-based units which use mm/s configured via Klipper.

``` cfg
[AFC_ACE Ace_1]
serial_port:
#    Required.  Serial port path for the ACE hardware
#    (e.g. /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0).

mode: combined
#    Default: combined
#    Operational mode.
#    combined: multiple ACE slots share one toolhead (retract before feed).
#    direct:   each ACE slot feeds its own extruder independently.

feed_speed: 60
#    Default: 60
#    Feed speed in mm/s.  ACE firmware expects mm/s natively.

retract_speed: 50
#    Default: 50
#    Retract/unwind speed in mm/s.  ACE firmware expects mm/s natively.

feed_length: 500
#    Default: 500
#    Distance in mm from the ACE slot to the toolhead.

retract_length: 500
#    Default: 500
#    Distance in mm to retract filament back from the toolhead to the ACE slot.

unload_preretract: 50
#    Default: 50
#    Distance in mm the ACE rewinds before a filament cut to tighten the
#    spool loop so filament rewinds cleanly onto the spool.

dist_hub: 200
#    Default: 200
#    Slot-to-hub/combiner distance in mm.  Used for two-phase loading:
#    prep_post_load feeds dist_hub mm to the hub, then load_sequence feeds
#    the remaining distance (feed_length - dist_hub) to the toolhead.
#    Default 200mm is a safe starting point — calibrate with ACE_CALIBRATE_HUB.

load_to_hub:
#    Default: <none> (inherits from AFC global)
#    Unit-level override for load-to-hub behavior.  When True, filament is
#    automatically staged at the hub after spool detection.  Can also be
#    overridden per-lane in [AFC_lane] sections.

use_feed_assist: True
#    Default: True
#    Enable the ACE feed assist motor for all slots on this unit.
#    Feed assist keeps the ACE spool motor pushing filament while the
#    extruder is printing, maintaining tension in the bowden path.

extruder_assist_length: 50
#    Default: 50
#    Distance in mm to advance with the extruder motor during the final
#    phase of filament loading, after the ACE has fed filament to the
#    toolhead sensor area.

extruder_assist_speed: 300
#    Default: 300
#    Speed in mm/min for the extruder assist advancement during loading.

sensor_approach_margin: 30
#    Default: 30
#    Distance in mm before the expected toolhead sensor position at which
#    feeding switches from bulk mode to incremental sensor-based feeding.
#    Enables accurate calibration measurements.

sensor_step: 40
#    Default: 40
#    Distance in mm per feed step during the sensor approach phase.
#    Smaller values give more precise sensor detection at the cost of
#    slower loading.

calibration_step: 50
#    Default: 50
#    Distance in mm per feed step during ACE_CALIBRATE_HUB calibration.

max_feed_overshoot: 100
#    Default: 100
#    Extra distance in mm to attempt past feed_length before declaring
#    a load failure.  Accounts for spool slip and path variance.

dock_purge: False
#    Default: False
#    When True, the load sequence drops the tool at the dock, feeds
#    filament, purges in the dock, then picks the tool back up.
#    Requires a toolchanger with dock purge gcode configured.

dock_purge_length: 50
#    Default: 50
#    Distance in mm of filament to extrude while the tool is docked
#    for purging.  Only used when dock_purge is True.

dock_purge_speed: 5
#    Default: 5
#    Extrude speed in mm/s during dock purge.

auto_spoolman_create: False
#    Default: False
#    When True, automatically creates filaments and spools in Spoolman
#    from ACE RFID tag data if no matching spool is found.  When False,
#    RFID data still matches against existing Spoolman entries but will
#    not create new ones.

fps_threshold: 0.9
#    Default: 0.9
#    FPS (Filament Pressure Sensor) ADC threshold (0.0-1.0).
#    When the extruder uses an AFC_FPS buffer as pin_tool_start, this
#    value determines when the toolhead sensor is considered triggered.
#    1.0 = filament fully compressed against extruder gears.

fps_load_threshold: 0.65
#    Default: 0.65
#    Lower FPS threshold used during active operations (loading,
#    calibration).  The latch prevents false clears, so this can
#    trigger earlier to catch brief pressure spikes when filament
#    first engages the extruder gears.

fps_delta_threshold: 0.15
#    Default: 0.15
#    Minimum jump between consecutive FPS ADC readings to count as
#    filament engagement during active operations.  Triggers even if
#    the absolute value stays below fps_load_threshold.

fps_confirm_count: 3
#    Default: 3  (minimum: 1)
#    Number of consecutive ADC readings above fps_threshold required
#    before the sensor is considered triggered.  Filters out brief
#    spikes from ACE pulsed feeding.  At ~100ms per ADC callback,
#    3 readings adds ~200ms of confirmation delay.

poll_interval: 1.0
#    Default: 1.0
#    Sensor/status polling interval in seconds for slot monitoring
#    and filament runout detection.

baud_rate: 115200
#    Default: 115200
#    Serial baud rate for ACE communication.
```

### ACE Lane-Level Overrides

The following options can be set in individual `[AFC_lane lane_name]` sections
to override the unit-level defaults for specific lanes.  This is useful when
lanes have different bowden lengths or hub distances.

``` cfg
[AFC_lane lane0]
unit: Ace_1:1
#    Required.  Unit:Slot assignment.  The slot number is 1-based in config
#    (converted to 0-based internally).

feed_length:
#    Default: (inherits from unit)
#    Per-lane override for feed distance in mm.

retract_length:
#    Default: (inherits from unit)
#    Per-lane override for retract distance in mm.

dist_hub:
#    Default: (inherits from unit)
#    Per-lane override for slot-to-hub distance in mm.

use_feed_assist:
#    Default: (inherits from unit)
#    Per-lane override for feed assist enable/disable.

auto_spoolman_create:
#    Default: (inherits from unit)
#    Per-lane override for automatic Spoolman spool creation from RFID data.
```

---

## [AFC_OpenAMS unit_name] Section

The following options are available in the `[AFC_OpenAMS unit_name]` section.
These options control the configuration of the AFC system when interfacing
with OpenAMS hardware.

OpenAMS units communicate with the OAMS firmware over Klipper's CAN/serial bus
and do not have physical stepper-based lane control — the OAMS hub motor and
firmware PID loop handle filament feeding and tension.

AFC_OpenAMS inherits many LED, speed/acceleration, and misc options from the
base `afcUnit` class (same as `[AFC_BoxTurtle]`), but does not use stepper- or
espooler-specific options (OpenAMS has no per-lane stepper).  Below are the
configuration values specific to an OpenAMS unit.

!!! note
    OpenAMS units only support AFC_FPS buffers (not TurtleNeck mechanical
    buffers).  If a non-FPS buffer is configured, it is silently ignored.

``` cfg
[AFC_OpenAMS AMS_1]
oams: oams1
#    Default: oams1
#    Name of the OpenAMS hardware instance this unit communicates with.
#    Must match a configured [AFC_oams oams_name] section.

stuck_spool_auto_recovery: False
#    Default: False
#    When True, a stuck spool detected during printing triggers an
#    automatic unload + reload + resume cycle instead of pausing for
#    manual intervention.  Defaults to False (pause-only) so the
#    behavior is opt-in.

stuck_spool_load_grace: 8.0
#    Default: 8.0
#    Grace distance/time threshold for stuck spool detection during
#    loading.  Higher values make detection less sensitive.

stuck_spool_pressure_threshold: 0.08
#    Default: 0.08
#    FPS pressure value above which the spool is considered stuck
#    during loading.  Lower values trigger earlier.

engagement_pressure_threshold: 0.6
#    Default: 0.6
#    Maximum FPS pressure value for filament engagement detection.
#    Readings above this threshold indicate the filament has
#    engaged the extruder gears.

engagement_min_pressure: 0.25
#    Default: 0.25
#    Minimum FPS pressure value for filament engagement detection.
#    Readings must exceed this threshold before engagement is
#    considered.

clog_sensitivity: medium
#    Default: medium
#    Sensitivity level for clog detection.  Passed to the OAMS
#    monitor for runtime clog detection tuning.

dock_purge: False
#    Default: False
#    When True, the load sequence drops the tool at the dock, feeds
#    filament, purges in the dock, then picks the tool back up.
#    Requires a toolchanger with dock purge gcode configured.

dock_purge_length: 105
#    Default: 105
#    Distance in mm of filament to extrude while the tool is docked
#    for purging.  Only used when dock_purge is True.

dock_purge_speed: 7
#    Default: 7
#    Extrude speed in mm/s during dock purge.
```
