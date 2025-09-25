
# Sovoron Klipper AFC + OpenAMS Reference

This repository bundles the Klipper configuration, helper macros, and Python
extensions needed to run Sovoron printers with dual OpenAMS units feeding an
AFC-based toolchanger.  It tracks a customised copy of the upstream
`klipper_openams` manager and the configuration layout that stitches the
hardware together.

The goal of this document is to give you an up-to-date view of how the system
is wired, the runtime protections it provides, and where each piece of
configuration lives so that you can maintain or extend your own installation.

---

## 1. System overview

### 1.1 Software components
- **`klipper_openams/src/oams_manager.py`** – Coordinates filament loading,
  monitors for runout and jam conditions, and orchestrates infinite-runout
  hand-offs to AFC when a lane change crosses FPS boundaries.
- **`printer_data/config`** – Modular Klipper configuration that defines every
  MCU, stepper, lane, sensor, and macro required for the AFC/OpenAMS pairing.
- **`printer_data/config/AFC`** – AFC specific configuration, including lane
  definitions, Box Turtle and Pico coprocessors, resume logic, and user-facing
  macros.

### 1.2 Hardware topology
- **Filament Pressure Sensors (`[fps]`)** – One per extruder path (e.g. `fps1`
  for tools mapped to T4–T7, `fps2` for T8–T11).  Each FPS object references an
  OpenAMS unit and the Klipper extruder it services.
- **OpenAMS hubs (`[oams]`)** – CAN-connected controllers that advance the
  spool follower motors, read hub sensors, and drive LEDs for four lanes each.
- **AFC controllers** – A Box Turtle CAN board provides the primary lane
  steppers, while an RP2040 “Pico” board extends the buffer sensor inputs.  The
  AFC configuration glues the OpenAMS hardware to Klipper tool changes.

### 1.3 Filament path at a glance
```
Spool → OpenAMS hub → Follower → FPS → Toolhead sensor → Extruder
```
The manager keeps the follower engaged while the lane is loaded, watches the
FPS pressure, and uses the extruder encoder to decide when to unload or swap
lanes.

---

## 2. Runtime behaviour

### 2.1 Loading and unloading
1. **Lane selection** – Macros such as `_TX1`/`_TX2` translate a requested tool
   into an OpenAMS filament group and call `OAMSM_LOAD_FILAMENT`.
2. **Load execution** – The manager commands the follower forward until the FPS
   reaches the midpoint between each hub’s `fps_lower_threshold` and
   `fps_upper_threshold`.  If the load stalls, an automatic unload retry is
   attempted before trying the next ready bay.
3. **Priming** – Once the toolhead sensor detects filament, the macro takes
   over to purge, wipe, or brush according to the AFC macro variables.
4. **Unloading** – Manual or automatic unloads call `OAMSM_UNLOAD_FILAMENT`
   which reverses the follower, clears LEDs, and updates FPS state so that the
   next load starts from a clean slate.

### 2.2 Runout management
- **Follower coasting** – When the hub’s HES sensor reports empty filament the
  `OAMSRunoutMonitor` waits `PAUSE_DISTANCE` millimetres before stopping the
  follower, giving the extruder time to coast while the next spool is prepared.
- **Reload threshold** – After coasting, the monitor continues tracking
  extruder travel until the estimated PTFE length (minus
  `reload_before_toolhead_distance`) is consumed, then triggers the reload
  callback.
- **Infinite runout** – If AFC has a mapped fall-back lane on the same FPS, the
  manager unloads the empty spool and loads the replacement.  When the mapping
  crosses FPS boundaries or another unit must take over, the manager delegates
  the swap back to AFC to avoid conflicting motion.


### 2.3 Clog detection
- **Preconditions** – Monitoring only runs when the printer is actively
  printing, the FPS is `LOADED`, all axes are homed, and no stuck-spool
  recovery is pending.  The manager samples the extruder position, OpenAMS
  encoder, and FPS pressure each cycle.【F:klipper_openams/src/oams_manager.py†L1326-L1370】
- **Trigger logic** – Once the extruder advances beyond the configured
  extrusion window (12–40 mm depending on sensitivity), the manager checks that
  the encoder has not moved more than the slack allowance, the FPS pressure has
  remained inside a tight band around 0.50, and the readings have persisted for
  the dwell period (6–12 s).  Any retract or pressure swing outside the band
  resets the tracker.【F:klipper_openams/src/oams_manager.py†L1386-L1450】
- **Sensitivity control** – `clog_sensitivity` can be set to `low`, `medium`
  (default), or `high` under `[oams_manager]`.  Lower sensitivity requires more
  extrusion and tolerates larger encoder slack, while higher sensitivity reacts
  sooner with tighter pressure bands.【F:klipper_openams/src/oams_manager.py†L24-L43】【F:klipper_openams/src/oams_manager.py†L366-L394】
- **Response** – When a clog is confirmed the matching hub LED is latched, the
  printer is paused, and the console logs the extruded distance, encoder delta,
  and observed pressure window so the operator knows what was detected.【F:klipper_openams/src/oams_manager.py†L1447-L1474】

### 2.4 Stuck-spool protection
- **Detection** – While the printer is actively printing and the lane is
  `LOADED`, the stuck-spool monitor samples the FPS value.  If the pressure
  stays below `STUCK_SPOOL_PRESSURE_THRESHOLD` (default `0.08`) for longer than
  `STUCK_SPOOL_DWELL` seconds, the spool is treated as jammed.【F:klipper_openams/src/oams_manager.py†L119-L132】【F:klipper_openams/src/oams_manager.py†L1194-L1267】
- **Response** – The affected hub LED is latched red, the follower is stopped,
  and the printer is paused with a descriptive console message.【F:klipper_openams/src/oams_manager.py†L1256-L1267】
- **Recovery** – Clearing the jam and resuming the print runs the recovery
  helper: LEDs are cleared, the hub error state is reset, and the follower is
  re-enabled in its stored direction before monitoring resumes.【F:klipper_openams/src/oams_manager.py†L1194-L1247】【F:klipper_openams/src/oams_manager.py†L1505-L1514】

### 2.5 Load/unload speed guards

Separate timers watch the encoder ticks during manual or automatic loads and
unloads.  If the encoder fails to advance by at least `MIN_ENCODER_DIFF`
counts within the `MONITOR_ENCODER_*_AFTER` window the printer is paused and
the offending lane is highlighted so the user can inspect the hardware.

---

## 3. Commands and macros

| Command | Parameters | Description |
| --- | --- | --- |
| `OAMSM_LOAD_FILAMENT` | `GROUP=<name>` | Load the named filament group (e.g. `T4`).  Tries each ready bay once, performing an unload retry after a failed attempt. |
| `OAMSM_UNLOAD_FILAMENT` | `FPS=<id>` | Unload the currently loaded spool on the specified FPS (`fps1`, `fps2`, …). |
| `OAMSM_FOLLOWER` | `ENABLE=0|1`, `DIRECTION=0|1`, `FPS=<id>` | Manually control the follower motor for debugging. |
| `OAMSM_CLEAR_ERRORS` | — | Stops all monitors, clears hub LEDs, resets FPS encoder samples, and restarts monitoring. |

Common higher-level macros are defined in
`printer_data/config/oams_macros.cfg` and the AFC macro folders:

- `_TX1` / `_TX2` – Primary toolchange helpers that unload the current group,
  invoke `OAMSM_LOAD_FILAMENT`, prime the extruder, and hand control back to the
  print.
- `SAFE_UNLOAD_FILAMENT1` / `SAFE_UNLOAD_FILAMENT2` – Heat the nozzle (if
  required), actuate the cutter, reverse the follower, and park the lane.
- `BT_CHANGE_TOOL`, `BT_LANE_EJECT`, `BT_RESUME`, etc. – AFC utility macros for
  orchestrating tool changes and manual lane manipulation from the console or
  UI buttons.

---

## 4. Configuration reference

The configuration files are grouped by subsystem so you can customise specific
behaviour without touching the entire stack.

### 4.1 OpenAMS core – `printer_data/config/oamsc.cfg`
- **MCUs & sensors** – `[mcu fps*]`, `[mcu oams_mcu*]`, and optional
  `[filament_switch_sensor extruder_*]` sections declare the CAN devices and
  inline runout sensors.
- **Macro variables** – `_oams_macro_variables` stores cutter coordinates,
  retract amounts, and feed lengths used by the load/unload macros.
- **OpenAMS units** – `[oams oams1]`, `[oams oams2]` tune PID values,
  FPS thresholds, PTFE lengths, and LED channel indices for each hub.
- **Filament groups & FPS definitions** – `[filament_group T4]` …
  `[filament_group T11]` map tools to lanes; `[fps fps1]`, `[fps fps2]` bind
  groups to an extruder and set lane-specific `reload_before_toolhead_distance`
  overrides if needed.
- **Manager include** – `[include oams_macros.cfg]` pulls in the macros that
  partner with the Python manager.

- **Manager options** – `[oams_manager]` accepts overrides like
  `reload_before_toolhead_distance` and `clog_sensitivity` to tune reload
  timing and the clog detection thresholds for your installation.【F:printer_data/config/oamsc.cfg†L293-L298】【F:klipper_openams/src/oams_manager.py†L32-L43】【F:klipper_openams/src/oams_manager.py†L366-L394】


### 4.2 OpenAMS macros – `printer_data/config/oams_macros.cfg`
Defines the sequence of operations around loading, unloading, filament sensor
sync, and cutter usage.  Adjust these macros to change toolhead preparation or
purge routines without modifying the manager itself.

### 4.3 AFC configuration – `printer_data/config/AFC`
- **`AFC.cfg`** – Global AFC behaviour (speeds, purge/kick/wipe toggles, LED
  colours, resume behaviour).
- **`AFC_Hardware.cfg`** – Lane steppers, hub geometry, and optional bypass
  sensors.
- **`AFC_AMS*.cfg`** – Maps OpenAMS units and lanes to AFC tool numbers and
  defines UI buttons for manual unloads.
- **`AFC_Turtle_1.cfg` & `AFC_Pico.cfg`** – Hardware definitions for the Box
  Turtle CAN board and Pico GPIO expander.
- **`AFC_Macro_Vars.cfg` & `macros/`** – Parameter blocks and helper macros
  used by purge, cut, brush, and resume routines.

---

## 5. Troubleshooting tips

1. **Identify the lane** – Hub LEDs and console messages from the manager
   always include the AMS index and spool position (0–3).  Use `BT_LANE_EJECT`
   or the UI buttons to investigate the physical lane.
2. **Clearing errors** – Run `OAMSM_CLEAR_ERRORS` after fixing hardware issues
   to reset LEDs, followers, and encoder history.
3. **Follower status** – If a follower fails to restart after maintenance, run
   `OAMSM_FOLLOWER ENABLE=1 DIRECTION=1 FPS=fps1` (or the appropriate FPS) to
   verify wiring and direction before reloading a spool.
4. **Monitoring logs** – `tail -f printer_data/logs/klippy.log` while loading or
   swapping lanes to watch the manager’s detailed logging for runout, retries,
  and jam detection decisions.
5. **Extending the system** – When adding another OpenAMS unit, duplicate the
   relevant `[mcu]`, `[oams]`, `[fps]`, and `[filament_group]` sections, update
   the UUIDs, and create matching AFC lane definitions.

---

With the pieces above you should have a clear map of how Sovoron’s AFC and
OpenAMS integration works, what safeguards are active at runtime, and where to
adjust parameters for your own hardware.

