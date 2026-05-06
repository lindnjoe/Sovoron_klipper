# U1 Integration Changes

## Overview

This branch adds support for the Snapmaker U1 toolchanger, Anycubic ACE PRO filament management hardware, and OpenAMS filament monitoring units to the Armored Turtle AFC (Automated Filament Changer) system. The changes introduce new unit drivers, hardware communication layers, sensor integrations, and compatibility shims for the Snapmaker Klipper fork, while refactoring the core AFC code to support unit-level overrides for load/unload/move sequences.

---

## New Files

### `extras/AFC_ACE.py`
**ACE Unit Support** — Full AFC unit driver for the Anycubic ACE PRO hardware. Supports two operational modes: `combined` (multiple ACE slots share one toolhead path via a combiner/splitter) and `direct` (each ACE slot feeds its own extruder independently). Manages filament feed/retract, slot selection, drying, and status via serial commands. (~4500 lines)

### `extras/AFC_ACE_serial.py`
**ACE Unit Support** — Serial communication layer for the ACE PRO. Implements a binary-framed JSON-RPC protocol over USB serial with CRC-16 checksums, reconnection logic, and health monitoring. (~760 lines)

### `extras/AFC_FPS.py`
**OpenAMS Support** — FPS (Filament Position Sensor) buffer driver. Reads a continuous 0.0-1.0 analog value from an ADC pin and applies proportional rotation-distance adjustment to maintain buffer tension at a configurable set point. Unit-agnostic; registers as an AFC buffer so any lane can reference it. Also provides software endstops for homing. (~1100 lines)

### `extras/AFC_OpenAMS_follower.py`
**OpenAMS Support** — Follower motor controller for OpenAMS hardware. Manages the OAMS follower motor that maintains filament tension in the buffer tube, including LED error state and rate-limited MCU command queuing. (~200 lines)

### `extras/AFC_OpenAMS_monitor.py`
**OpenAMS Support** — Real-time filament monitoring for OpenAMS units. Monitors encoder movement and FPS pressure during printing to detect stuck spools, clogs, and runout conditions. Reports problems to AFC_OpenAMS via callbacks rather than making policy decisions directly. (~370 lines)

### `extras/AFC_U1_rfid.py`
**U1/Toolchanger Support** — RFID integration for the Snapmaker U1. Polls the U1's `filament_detect` Klipper module for RFID tag data and syncs material type, color, and spool information to AFC lanes and Spoolman. (~405 lines)

### `extras/temperature_ace.py`
**ACE Unit Support** — Klipper temperature sensor plugin that reads ACE device temperature from the AFC_ACE unit's cached hardware status and exposes it as a standard `temperature_sensor`. (~200 lines)

### `extras/temperature_oams.py`
**OpenAMS Support** — HDC1080 I2C temperature and humidity sensor driver for OpenAMS hardware. Exposes readings as a standard Klipper temperature sensor. (~210 lines)

---

## Removed Files

### `extras/openams_integration.py`
**OpenAMS Support** — Removed. The shared OpenAMS integration helpers (`AMSHardwareService`, `AMSRunoutCoordinator`) have been refactored and absorbed directly into the rewritten `AFC_OpenAMS.py`. (~395 lines removed)

---

## Modified Files

### `extras/AFC.py` (+578 modified lines)

- **U1/Toolchanger Support** — Added `u1_rfid` attribute and `AFC_RFID_READ` GCode command to force RFID re-reads for U1 lanes.
- **U1/Toolchanger Support** — `LANE_MOVE` now delegates to `cur_lane.unit_obj.lane_move()` instead of calling `move_advanced` directly, allowing ACE and OpenAMS units to use their own movement logic.
- **U1/Toolchanger Support** — `load_sequence()` now calls `cur_lane.unit_obj.load_sequence()` first, allowing units like ACE to provide custom load sequences. If the unit returns non-None, the default load path is skipped.
- **U1/Toolchanger Support** — `unload_sequence()` similarly calls `cur_lane.unit_obj.unload_sequence()` for unit-level override. Added a standalone toolchanger unload path that does a quick pull and prompts manual filament removal.
- **U1/Toolchanger Support** — `LANE_UNLOAD` calls `cur_lane.unit_obj.lane_unload()` for unit-level override.
- **U1/Toolchanger Support** — `TOOL_CHANGE` now checks `on_shuttle()` to detect when a lane is logically loaded but the tool is not physically on the shuttle (e.g. after restart). Added logic to re-activate the correct extruder without a full tool swap when the lane is already physically loaded. Added handling for unloading a lane from the target extruder when the shuttle is empty but lane_loaded is set.
- **U1/Toolchanger Support** — Added `_dock_purge_dropoff()` and `_dock_purge_pickup()` methods for units that support purging filament while the tool is parked in its dock. Wraps the existing load_sequence in try/finally to ensure dock pickup happens even on failure.
- **Bug Fix** — `HUB_LOAD` loops now have a maximum retry count (30 attempts) instead of unbounded `while` loops with TODO comments.
- **Bug Fix** — `TOOL_LOAD` now allows re-loading a lane that is already marked as loaded on its extruder (e.g. after restart), instead of blocking with an error.
- **Bug Fix** — Z-hop in `TOOL_UNLOAD` no longer mutates the `last_position` list in place, preventing side effects.
- **Bug Fix** — Buffer retract loops during unload now reference `self.tool_max_unload_attempts` (AFC-level) instead of the incorrect `cur_lane.tool_max_unload_attempts`, and pause between retract attempts reduced from 0.5s to 0.1s.
- **Bug Fix** — Toolhead unload skip logic simplified: only skips when next lane is NOT on the current extruder.
- **Bug Fix** — `AFC_form_tip` object now looked up once at connect time instead of on every unload call.
- **Bug Fix** — Added `abort_load()` calls before error returns in load sequence to allow units to cancel in-progress hardware operations.
- **Bug Fix** — Added `afcDeltaTime` logging for `post_load_macro` and `post_unload_macro` (previously had TODO comments).

### `extras/AFC_BoxTurtle.py` (+1 line)

- **U1/Toolchanger Support** — Added `cur_lane.extruder_obj.on_shuttle()` check in the toolhead load detection logic, so BoxTurtle correctly detects when a toolchanger tool is on the shuttle even if the pre-sensor and tool_end states are false.

### `extras/AFC_buffer.py` (+51 lines)

- **U1/Toolchanger Support** — Added per-extruder state memory (`_saved_states` dict) that saves/restores the last known buffer state (advancing/trailing) across tool changes. Only restores state when the same lane returns to the same extruder; a different lane gets a fresh start.
- **U1/Toolchanger Support** — `enable_buffer()` restores saved state and sets the initial multiplier accordingly. Unknown/no-prior-state starts at 1.0x instead of defaulting to the high multiplier.
- **U1/Toolchanger Support** — `disable_buffer()` saves the current state for the extruder/lane pair before resetting.
- **Bug Fix** — `check_runout_encoder_cb` now skips fault detection for lanes without an extruder stepper (e.g. OpenAMS/ACE lanes that use unit firmware for movement).
- **Bug Fix** — Fault detection is skipped when the active extruder is not this buffer's extruder, preventing false faults during tool changes when the other extruder is moving.
- **Bug Fix** — Status reporting now guards against `None` extruder_stepper before accessing `stepper.get_rotation_distance()`.

### `extras/AFC_error.py` (+9 lines)

- **U1/Toolchanger Support** — `handle_lane_failure()` now calls `cur_lane.unit_obj.abort_load()` before disabling the stepper, allowing ACE/OpenAMS units to cancel any in-progress hardware operation.
- **Bug Fix** — Replaced bare `logging.warning()` calls with `self.logger.debug()` / `self.logger.warning()` for consistent log routing.

### `extras/AFC_extruder.py` (+124 lines)

- **U1/Toolchanger Support** — Added `"internal"` as a valid `tool_start` mode. This mode relies on unit firmware (e.g. ACE) for filament engagement verification instead of requiring an AFC-visible sensor or buffer.
- **U1/Toolchanger Support** — Added `_u1_runout_event_handler()` that intercepts Klipper's native `filament_motion_sensor` runout event and routes it through AFC's infinite spool system instead of triggering a native Klipper pause. Patches the `filament_sensor_obj.runout_helper._runout_event_handler` and ensures `runout_gcode` is set so the callback path is always reachable.
- **Snapmaker Compatibility** — Added `_detect_trapq_extra_arg()` that introspects the `trapq_append` C function signature to detect if the Snapmaker Klipper fork's extra trailing argument is present. The `trapq_append()` wrapper method automatically appends the extra `0` argument when needed, making the code work on both stock Klipper and the Snapmaker fork.
- **Snapmaker Compatibility** — Removed the hardcoded trailing `0` from `trapq_append` calls (was a TODO workaround).
- **Bug Fix** — `_handle_klippy_ready` now wraps `printer.lookup_object` in try/except for the extruder and park_detector lookups, converting missing-object crashes into config errors or graceful degradation with a debug log.
- **Bug Fix** — `handle_start_runout()` now guards against missing `fila_tool_start` attribute (buffer/internal setups don't have it).
- **Bug Fix** — `note_tool_start_callback()` wrapped in mutex for thread safety. Added check to only unload when not printing to prevent accidental unloads during active prints.
- **Bug Fix** — `set_status_color()` now handles color as list/tuple in addition to comma-separated string.

### `extras/AFC_functions.py` (+119 lines)

- **U1/Toolchanger Support** — `get_current_lane()` now has a fallback path that asks the lane's `unit_obj.get_current_lane_fallback()` when `on_shuttle()` is False (e.g. ACE after power cycle with shuttle parked).
- **U1/Toolchanger Support** — `_handle_activate_extruder()` accepts an optional `lane` parameter and calls `lane.activate_toolhead_extruder()` before syncing state. Non-active lanes now only have their buffer disabled if they don't share a buffer object with the active lane (prevents disabling a shared buffer during tool change).
- **U1/Toolchanger Support** — `manual_unset_lane_loaded()` now calls `cur_lane_loaded.unit_obj.on_lane_unset_loaded()` after unsetting, allowing units to perform cleanup (e.g. ACE retracting filament).
- **U1/Toolchanger Support** — Added `_lane_reset_command()` helper that checks `lane.unit_obj.get_lane_reset_command()` for a custom reset command before falling back to the default `AFC_LANE_RESET`. Used in calibration failure prompts and the reset lane UI.
- **U1/Toolchanger Support** — `_afc_lane_reset()` now checks for a custom reset command from the unit and runs it instead of the default reset logic when provided.
- **U1/Toolchanger Support** — TD-1 calibration now supports `TD1=all` to calibrate all lanes with a `td1_device_id`, skipping lanes that are loaded to the toolhead or have a triggered hub.
- **Bug Fix** — TD-1 calibration hub check now handles virtual hubs (`switch_pin == "virtual"`) by skipping the hub state check instead of erroring.
- **Bug Fix** — Calibration flow no longer falls through to "No lanes selected" message when `afc_bl` or `td1` parameters are provided.

### `extras/AFC_hub.py` (+10 lines)

- **U1/Toolchanger Support** — Default `switch_pin` changed from `None` to `"virtual"`, making virtual hubs the default for units that don't have physical hub sensors.
- **Bug Fix** — Virtual hub lane validation now skips the load sensor check for lanes where `prep` is not None (lanes without a prep sensor, like ACE lanes, don't need a load sensor for virtual hub operation).
- **Bug Fix** — Virtual hub `state` property now includes `self._state` in the OR condition and only checks `raw_load_state` for lanes that actually have a load sensor, preventing false positives from lanes without load sensors.

### `extras/AFC_lane.py` (+163 lines)

- **U1/Toolchanger Support** — Added `u1_rfid_channel` config parameter for mapping lanes to U1 RFID channels.
- **U1/Toolchanger Support** — Homing endstop setup now skips hub endstop registration for virtual hub pins, skips buffer endstop registration when no advance_pin exists, and registers FPS software endstops (both advance and trailing) when the buffer is an FPS type.
- **U1/Toolchanger Support** — `_get_buffer_object()` now tries both `AFC_buffer` and `AFC_FPS` prefixes when looking up buffer objects.
- **U1/Toolchanger Support** — Added validation that `pin_tool_start=internal` is only allowed for ACE and Toolchanger unit types.
- **U1/Toolchanger Support** — Buffer resolution now falls back to `unit_obj.buffer_name` when extruder-level buffer is not set. Removed the hard requirement for a buffer on shared extruders (valid for ACE units that don't need one).
- **U1/Toolchanger Support** — `_perform_infinite_runout()` now has a standalone toolchanger path: for U1-style setups, it docks the current tool and picks up the next one (with temperature management) instead of doing a full filament unload/reload cycle.
- **U1/Toolchanger Support** — `capture_td1_when_loaded()` and `get_td1_data()` now call `unit_obj.prep_capture_td1()` and `unit_obj.capture_td1_data()` respectively, allowing units to override TD-1 capture behavior.
- **U1/Toolchanger Support** — `get_toolhead_pre_sensor_state()` and `get_toolhead_endstop()` handle `"internal"` tool_start mode by returning False/None (unit firmware handles verification).
- **Snapmaker Compatibility** — Toolhead endstop setup skips registration for `"internal"` pin_tool_start.
- **Bug Fix** — Weight tracking timer now uses `getattr` for `toolhead_extruder` to avoid AttributeError on extruder objects without that attribute.

### `extras/AFC_led.py` (+1 line)

- **Bug Fix** — Replaced bare `logging.info()` call with `self.afc.logger.info()` for consistent log routing through the AFC logger.

### `extras/AFC_prep.py` (+60 lines)

- **U1/Toolchanger Support** — Startup extruder activation split into two passes: first pass restores `lane_loaded` from saved state, second pass activates the correct extruder. Only the first `on_shuttle()` extruder is activated (prevents multiple activations in multi-tool setups).
- **U1/Toolchanger Support** — Added `_start_u1_rfid()` method called after prep completes. Scans lanes for `u1_rfid_channel` configuration, creates an `AFC_U1_RFID` instance, registers lanes, and starts RFID polling.
- **Bug Fix** — Saved vars numeric field conversion (`bed_temp`, `extruder_temp`, `weight`) simplified and made more robust: handles None, empty string, "NONE" consistently; `weight` defaults to 0 instead of None.

### `extras/AFC_stepper.py` (+86 lines)

- **Snapmaker Compatibility** — Added the same `_detect_trapq_extra_arg()` / `trapq_append()` wrapper as in AFC_extruder.py, detecting and adapting to the Snapmaker fork's extra `trapq_append` argument at runtime.
- **Snapmaker Compatibility** — Removed the hardcoded trailing `0` from `_submit_move()`'s `trapq_append` call.
- **U1/Toolchanger Support** — `_setup_endstops()` now handles `tool_start_pin` values of `None` gracefully (skips registration) and registers FPS software endstops when no hardware advance pin exists.
- **U1/Toolchanger Support** — Added `_add_fps_endstop()` method that looks up an FPS buffer object and registers its software advance and trailing endstops on the stepper, enabling homing moves against FPS pressure thresholds.

### `extras/AFC_unit.py` (+67 lines)

- **U1/Toolchanger Support** — `_get_lane_led_color()` now calls `lane.get_color()` instead of accessing `lane.color` directly, ensuring proper color resolution through the lane's getter method.
- **U1/Toolchanger Support** — Added base class stubs for unit-level overrides. These methods return `None` or `pass` by default, letting existing unit types (BoxTurtle, NightOwl) work unchanged while new units (ACE, OpenAMS) can override them:
  - `abort_load()` — Cancel in-progress load operations on hardware.
  - `lane_move()` — Move filament (default delegates to `move_advanced`).
  - `load_sequence()` — Custom load logic.
  - `unload_sequence()` — Custom unload logic.
  - `lane_unload()` — Custom lane unload.
  - `on_lane_unset_loaded()` — Post-unset cleanup.
  - `prep_capture_td1()` — Custom TD-1 prep capture.
  - `capture_td1_data()` — Custom TD-1 data capture.
  - `get_lane_reset_command()` — Custom lane reset GCode.
  - `get_current_lane_fallback()` — Fallback lane name when `on_shuttle()` is False.

### `extras/AFC_utils.py` (+148 lines)

- **ACE Unit Support / OpenAMS Support** — Added a full Spoolman proxy API to `AFC_moonraker`: `_spoolman_proxy()` for raw Spoolman API calls through Moonraker, plus convenience methods `search_filaments()`, `create_vendor()`, `search_vendors()`, `get_or_create_vendor()`, `create_filament()`, `create_spool()`, and `search_spools()`. These enable ACE and RFID integrations to automatically create and match spools/filaments in Spoolman.
- **Bug Fix** — `add_filament_switch()` now sets `extruder` to `'extruder'` instead of an empty string, fixing a Klipper config validation issue.

### `extras/AFC_OpenAMS.py` (complete rewrite, +2359/-656 lines)

- **OpenAMS Support** — Complete rewrite of the OpenAMS unit driver. The previous implementation (~656 lines) with virtual sensor classes and monkeypatched lane methods has been replaced with a comprehensive event-driven architecture (~2359 lines) featuring:
  - `AMSEventBus` — Pub/sub event system with priority-based subscriber dispatch and event history.
  - `LaneRegistry` — Thread-safe lane registration with multi-key lookup (by name, spool index, extruder).
  - `AMSHardwareService` — Hardware polling service that monitors encoder, FPS, F1S, and hub Hall-effect sensors, publishing state change events.
  - Full integration with the new `AFC_OpenAMS_follower`, `AFC_OpenAMS_monitor`, and `AFC_FPS` modules.
  - Replaces the removed `openams_integration.py` by absorbing its functionality.
