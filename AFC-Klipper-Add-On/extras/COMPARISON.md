# AFC Extras Comparison

This document highlights the most significant differences between the Sovoron
`work` branch copy of `AFC-Klipper-Add-On/extras` and Armored Turtle's upstream
`multi_extruder` branch. Citations reference the Sovoron sources, while short
upstream excerpts at the end capture the prior behaviour for context.

## High-level themes

- Global runout handling adds shared enable/disable switches and debounce
  defaults that downstream components inherit. 【F:AFC-Klipper-Add-On/extras/AFC.py†L200-L208】【F:AFC-Klipper-Add-On/extras/AFC_lane.py†L128-L189】
- All virtual filament switches are registered through an expanded helper that
  optionally hides sensors from the GUI, wires delayed runout callbacks, and
  returns a `DebounceButton` shim. 【F:AFC-Klipper-Add-On/extras/AFC.py†L327-L334】【F:AFC-Klipper-Add-On/extras/AFC_utils.py†L25-L137】
- Temperature control for `AFC_M109` resolves the active toolhead extruder,
  honours per-extruder deadbands, and performs manual tolerance waits. 【F:AFC-Klipper-Add-On/extras/AFC.py†L1862-L1999】
- Moonraker and Spoolman helpers list spools while leaving Spoolman extras
  untouched during lane updates. 【F:AFC-Klipper-Add-On/extras/AFC_utils.py†L320-L345】【F:AFC-Klipper-Add-On/extras/AFC_spool.py†L276-L304】
- Stepper control detects Klipper's newer `motion_queuing` interface before
  allocating trap queues, improving compatibility. 【F:AFC-Klipper-Add-On/extras/AFC_stepper.py†L20-L103】

## File-by-file notes

### AFC.py

- Introduces `enable_hub_runout`, `enable_tool_runout`, and `debounce_delay`
  configuration that downstream objects consume. 【F:AFC-Klipper-Add-On/extras/AFC.py†L200-L208】
- Registers virtual bypass/quiet sensors with simplified names via the enhanced
  helper. 【F:AFC-Klipper-Add-On/extras/AFC.py†L327-L334】
- Adds `_get_afc_extruder_for_tool()` and rewrites `_cmd_AFC_M109` to reuse the
  resolved AFC extruder, pull its deadband, and decide how to wait for
  temperature stability. 【F:AFC-Klipper-Add-On/extras/AFC.py†L1862-L1946】
- Updates `_wait_for_temp_within_tolerance()` to use an absolute ± tolerance and
  exit when the provided tolerance is non-positive. 【F:AFC-Klipper-Add-On/extras/AFC.py†L1983-L1999】
- **Upstream contrast:** the baseline lacked the global runout toggles and kept
  the original virtual-sensor registration form. 【F:AFC-Klipper-Add-On/extras/COMPARISON.md†L149-L165】

### AFC_HTLF.py

- Always creates the home sensor through `add_filament_switch`, allowing GUI
  visibility to be toggled while still benefiting from the helper's debounce
  logic. 【F:AFC-Klipper-Add-On/extras/AFC_HTLF.py†L45-L52】
- **Upstream contrast:** the multi_extruder branch only registered the sensor
  when GUI exposure was enabled. 【F:AFC-Klipper-Add-On/extras/COMPARISON.md†L167-L173】

### AFC_buffer.py

- Registers advance and trailing switches regardless of GUI visibility, letting
  the helper manage whether they appear in the UI. 【F:AFC-Klipper-Add-On/extras/AFC_buffer.py†L60-L64】
- **Upstream contrast:** the baseline guarded both registrations behind
  `enable_sensors_in_gui`. 【F:AFC-Klipper-Add-On/extras/COMPARISON.md†L175-L182】

### AFC_extruder.py

- Adds per-extruder runout toggles, debounce delay, and the printer reactor
  reference. 【F:AFC-Klipper-Add-On/extras/AFC_extruder.py†L22-L41】
- Tool start/end sensors use the new helper to receive delayed runout callbacks
  that update `min_event_systime`, preventing missed events. 【F:AFC-Klipper-Add-On/extras/AFC_extruder.py†L51-L149】
- **Upstream contrast:** sensors were only registered for GUI visibility without
  callback overrides. 【F:AFC-Klipper-Add-On/extras/COMPARISON.md†L184-L197】

### AFC_hub.py

- Accepts a missing hardware pin by generating a virtual pin name, so hubs can
  participate purely through software triggers. 【F:AFC-Klipper-Add-On/extras/AFC_hub.py†L32-L67】
- Uses the helper with a hub-specific runout callback that debounces events and
  forwards them to the active lane before rescheduling. 【F:AFC-Klipper-Add-On/extras/AFC_hub.py†L65-L91】
- **Upstream contrast:** runout logic lived inside the raw button callback and
  only supported GUI-exposed sensors. 【F:AFC-Klipper-Add-On/extras/COMPARISON.md†L199-L215】

### AFC_lane.py

- Lanes adopt global debounce/runout defaults and wire prep/load sensors through
  the helper with delayed callbacks, enabling consistent handling. 【F:AFC-Klipper-Add-On/extras/AFC_lane.py†L128-L189】【F:AFC-Klipper-Add-On/extras/AFC_lane.py†L228-L246】
- Adds `handle_load_runout()` and `handle_prep_runout()` helpers that respect
  GUI-disabled sensors, trigger infinite/pause runout logic, and persist AFC
  state. 【F:AFC-Klipper-Add-On/extras/AFC_lane.py†L546-L704】
- **Upstream contrast:** sensors were only registered when GUI exposure was
  enabled and runout checks happened directly in the button callbacks without
  debounce awareness. 【F:AFC-Klipper-Add-On/extras/COMPARISON.md†L217-L234】

### AFC_logger.py

- Drops the Kalico-specific `APP_NAME` import, wraps queue listener construction
  in a `try/except`, and disables automatic log rollover to keep diagnostic
  history. 【F:AFC-Klipper-Add-On/extras/AFC_logger.py†L15-L29】
- Adds a `warning()` helper that mirrors other severities by logging and pushing
  messages to the UI queue. 【F:AFC-Klipper-Add-On/extras/AFC_logger.py†L76-L83】
- **Upstream contrast:** the baseline relied on the `APP_NAME` guard and always
  rolled the log immediately. 【F:AFC-Klipper-Add-On/extras/COMPARISON.md†L236-L248】

### AFC_spool.py

- Assigning a Spoolman ID now only fetches metadata for the lane—no extras are
  parsed or PATCHed—so spool updates stay local to Klipper. 【F:AFC-Klipper-Add-On/extras/AFC_spool.py†L276-L304】
- Duplicate spool IDs are rejected to prevent multiple lanes from referencing
  the same Spoolman entry. 【F:AFC-Klipper-Add-On/extras/AFC_spool.py†L228-L237】
- **Upstream contrast:** the baseline updated the `loaded_lane` extra when
  swapping spools. 【F:AFC-Klipper-Add-On/extras/COMPARISON.md†L250-L255】

### AFC_stepper.py

- Defines `LARGE_TIME_OFFSET` and checks for the `motion_queuing` object to use
  its trap queue when available. 【F:AFC-Klipper-Add-On/extras/AFC_stepper.py†L20-L103】
- Resets/wipes trap queues when motion queuing is present to prevent stale moves.
  【F:AFC-Klipper-Add-On/extras/AFC_stepper.py†L78-L103】
- **Upstream contrast:** the baseline always allocated and finalised its own trap
  queue via `chelper`. 【F:AFC-Klipper-Add-On/extras/COMPARISON.md†L257-L266】

### AFC_utils.py

- Extends `add_filament_switch()` with GUI visibility, debounce, and runout
  callback parameters, returning the helper and a `DebounceButton` wrapper.
  【F:AFC-Klipper-Add-On/extras/AFC_utils.py†L25-L73】
- Adds `DebounceButton` plus `list_spools()` to support delayed callbacks and
  Spoolman discovery. 【F:AFC-Klipper-Add-On/extras/AFC_utils.py†L90-L345】
- **Upstream contrast:** the helper only created GUI-visible sensors with
  runout paused. 【F:AFC-Klipper-Add-On/extras/COMPARISON.md†L268-L276】

## Selected upstream excerpts (multi_extruder)

These snippets are copied from the upstream branch to illustrate the previous
behaviour referenced above.

```python
# AFC.py excerpt
self.enable_assist          = config.getboolean("enable_assist",        True)
self.enable_assist_weight   = config.getfloat("enable_assist_weight",   500.0)

self.debug                  = config.getboolean('debug', False)

# Virtual sensor registration
self.bypass = add_filament_switch("filament_switch_sensor virtual_bypass", "afc_virtual_bypass:virtual_bypass", self.printer ).runout_helper
self.quiet_switch = add_filament_switch("filament_switch_sensor quiet_mode", "afc_quiet_mode:afc_quiet_mode", self.printer ).runout_helper

# _cmd_AFC_M109 core
pheaters = self.printer.lookup_object('heaters')
heater = extruder.get_heater()
pheaters.set_temperature(heater, temp, False)
if wait and deadband is not None and temp > 0:
    self._wait_for_temp_within_tolerance(heater, temp, deadband)
    return
current_temp = heater.get_temp(self.reactor.monotonic())[0]
should_wait = wait and abs(current_temp - temp) > self.temp_wait_tolerance
pheaters.set_temperature(heater, temp, should_wait)
```
【F:AFC-Klipper-Add-On/extras/COMPARISON.md†L149-L193】

```python
# AFC_HTLF.py excerpt
def __init__(...):
    if self.enable_sensors_in_gui:
        if self.home_pin is not None:
            self.home_filament_switch_name = "filament_switch_sensor {}_home_pin".format(self.name)
            self.home_sensor = add_filament_switch(self.home_filament_switch_name, self.home_pin, self.printer )
```
【F:AFC-Klipper-Add-On/extras/COMPARISON.md†L195-L203】

```python
# AFC_buffer.py excerpt
if self.enable_sensors_in_gui:
    self.adv_filament_switch_name = "filament_switch_sensor {}_{}".format(self.name, "expanded")
    self.fila_avd = add_filament_switch(self.adv_filament_switch_name, self.advance_pin, self.printer )

    self.trail_filament_switch_name = "filament_switch_sensor {}_{}".format(self.name, "compressed")
    self.fila_trail = add_filament_switch(self.trail_filament_switch_name, self.trailing_pin, self.printer )
```
【F:AFC-Klipper-Add-On/extras/COMPARISON.md†L205-L212】

```python
# AFC_extruder.py excerpt
if self.tool_start is not None:
    if self.tool_start == "buffer":
        self.logger.info("Setting up as buffer")
    else:
        self.tool_start_state = False
        buttons.register_buttons([self.tool_start], self.tool_start_callback)
        if self.enable_sensors_in_gui:
            self.tool_start_filament_switch_name = "filament_switch_sensor {}".format("tool_start")
            self.fila_tool_start = add_filament_switch(self.tool_start_filament_switch_name, self.tool_start, self.printer )
```
【F:AFC-Klipper-Add-On/extras/COMPARISON.md†L214-L223】

```python
# AFC_hub.py excerpt
if self.switch_pin is not None:
    self.state = False
    buttons.register_buttons([self.switch_pin], self.switch_pin_callback)

if self.enable_sensors_in_gui:
    self.filament_switch_name = "filament_switch_sensor {}_Hub".format(self.name)
    self.fila = add_filament_switch(self.filament_switch_name, self.switch_pin, self.printer )
```
【F:AFC-Klipper-Add-On/extras/COMPARISON.md†L225-L232】

```python
# AFC_lane.py excerpt
if self.enable_sensors_in_gui:
    if self.prep is not None and (self.sensor_to_show is None or self.sensor_to_show == 'prep'):
        self.prep_filament_switch_name = "filament_switch_sensor {}_prep".format(self.name)
        self.fila_prep = add_filament_switch(self.prep_filament_switch_name, self.prep, self.printer )

    if self.load is not None and (self.sensor_to_show is None or self.sensor_to_show == 'load'):
        self.load_filament_switch_name = "filament_switch_sensor {}_load".format(self.name)
        self.fila_load = add_filament_switch(self.load_filament_switch_name, self.load, self.printer )
```
【F:AFC-Klipper-Add-On/extras/COMPARISON.md†L234-L242】

```python
# AFC_logger.py excerpt
if APP_NAME == "Kalico":
    super().__init__(filename, False)
else:
    super().__init__(filename)

logging.handlers.TimedRotatingFileHandler.doRollover(self)
```
【F:AFC-Klipper-Add-On/extras/COMPARISON.md†L244-L248】

```python
# AFC_spool.py excerpt
self.afc.tool_cmds[map_switch]=lane_switch
sw_lane.map=map_switch
self.afc.save_vars()
```
【F:AFC-Klipper-Add-On/extras/COMPARISON.md†L250-L255】

```python
# AFC_stepper.py excerpt
self.motion_queue = None
self.trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
self.trapq_append = ffi_lib.trapq_append
self.trapq_finalize_moves = ffi_lib.trapq_finalize_moves
```
【F:AFC-Klipper-Add-On/extras/COMPARISON.md†L257-L266】

```python
# AFC_utils.py excerpt
def add_filament_switch( switch_name, switch_pin, printer ):
    ...
    fila = printer.load_object(cfg_wrap, switch_name)
    fila.runout_helper.sensor_enabled = False
    fila.runout_helper.runout_pause = False
    return fila
```
【F:AFC-Klipper-Add-On/extras/COMPARISON.md†L268-L276】
