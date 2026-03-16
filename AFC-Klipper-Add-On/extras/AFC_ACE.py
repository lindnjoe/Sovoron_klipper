# Armored Turtle Automated Filament Changer
#
# ACE Unit Type - Direct ACE PRO hardware integration without ACEPRO/DuckACE
#
# Supports two operational modes:
#   combined: Multiple ACE slots share one toolhead path (combiner/splitter).
#             Must retract current slot before feeding new one.
#   direct:   Each ACE slot feeds its own extruder independently.
#             No global retract-before-feed constraint.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import traceback

from configfile import error as config_error
from datetime import datetime
from typing import TYPE_CHECKING, Any, Dict, Optional

if TYPE_CHECKING:
    from extras.AFC_lane import AFCLane
    from configfile import ConfigWrapper

try: from extras.AFC_utils import ERROR_STR
except: raise config_error("Error when trying to import AFC_utils.ERROR_STR\n{trace}".format(trace=traceback.format_exc()))

try: from extras.AFC_unit import afcUnit
except: raise config_error(ERROR_STR.format(import_lib="AFC_unit", trace=traceback.format_exc()))

try: from extras.AFC_lane import AFCLane, AFCLaneState
except: raise config_error(ERROR_STR.format(import_lib="AFC_lane", trace=traceback.format_exc()))

try: from extras.AFC_respond import AFCprompt
except: raise config_error(ERROR_STR.format(import_lib="AFC_respond", trace=traceback.format_exc()))

try: from extras.AFC_ACE_serial import ACEConnection, ACESerialError, ACETimeoutError
except: raise config_error(ERROR_STR.format(import_lib="AFC_ACE_serial", trace=traceback.format_exc()))

# Operational modes
MODE_COMBINED = "combined"  # Multiple slots -> one toolhead (retract before feed)
MODE_DIRECT = "direct"      # Each slot -> its own extruder (independent operation)



class afcACE(afcUnit):
    """AFC unit that talks directly to Anycubic ACE PRO hardware.

    Owns the serial communication to ACE PRO hardware and implements both
    combined (shared toolhead) and direct (per-extruder) operational modes.

    Hub config (required - virtual sensor since ACE has no physical hub sensor):
        [AFC_hub ace_hub1]
        switch_pin: virtual

    Config example:
        [AFC_ACE Ace_1]
        serial_port: /dev/ttyACM0
        hub: ace_hub1
        extruder: extruder
        mode: combined          # or "direct" for multi-extruder
        feed_speed: 800         # mm/min
        retract_speed: 800      # mm/min
        feed_length: 500        # mm - distance from ACE to toolhead
        retract_length: 500     # mm - distance to retract back to ACE

    Lane config:
        [AFC_lane lane1]
        unit: Ace_1:1            # Unit:Slot (1-based in config, 0-based internal)
        hub: ace_hub1
        extruder: extruder
        feed_length: 2800        # (optional) per-lane override
        retract_length: 2500     # (optional) per-lane override
        use_feed_assist: True    # (optional) per-lane override
    """

    SLOTS_PER_UNIT = 4

    def __init__(self, config: ConfigWrapper):
        super().__init__(config)
        self.type = "ACE"

        # Serial port configuration
        self.serial_port = config.get("serial_port")

        # Operational mode
        mode = config.get("mode", MODE_COMBINED).lower().strip()
        if mode not in (MODE_COMBINED, MODE_DIRECT):
            raise config_error(
                f"[{config.get_name()}] invalid mode '{mode}'. "
                f"Must be '{MODE_COMBINED}' or '{MODE_DIRECT}'"
            )
        self.mode = mode

        # Feed/retract parameters
        self.feed_speed = config.getfloat("feed_speed", 800.0)          # mm/min
        self.retract_speed = config.getfloat("retract_speed", 800.0)    # mm/min
        self.feed_length = config.getfloat("feed_length", 500.0)        # mm
        self.retract_length = config.getfloat("retract_length", 500.0)  # mm

        # Feed assist: default enable/disable for all slots
        self._default_feed_assist = config.getboolean("use_feed_assist", True)

        # Per-slot feed assist overrides (populated at runtime)
        # None = use default, True/False = explicit override
        self._slot_feed_assist: Dict[int, Optional[bool]] = {}

        # Per-lane overrides for feed/retract parameters.
        # Keyed by lane name (e.g. "lane12").  None values omitted.
        self._lane_feed_length: Dict[str, float] = {}
        self._lane_retract_length: Dict[str, float] = {}
        self._lane_feed_assist: Dict[str, bool] = {}
        self._parse_lane_overrides(config)

        # Extruder assist length: how far to advance with extruder motor
        # during feed assist (after filament reaches toolhead sensor area)
        self.extruder_assist_length = config.getfloat("extruder_assist_length", 50.0)  # mm
        self.extruder_assist_speed = config.getfloat("extruder_assist_speed", 300.0)   # mm/min

        # Sensor-based feeding: feed in increments near the toolhead sensor
        # instead of blindly feeding a fixed distance. This enables calibration.
        self.sensor_approach_margin = config.getfloat("sensor_approach_margin", 60.0)  # mm before expected sensor to switch to incremental
        self.sensor_step = config.getfloat("sensor_step", 20.0)                       # mm per check during sensor approach
        self.calibration_step = config.getfloat("calibration_step", 50.0)              # mm per check during calibration
        self.max_feed_overshoot = config.getfloat("max_feed_overshoot", 100.0)         # mm extra to try past feed_length before giving up

        # Dock purge: drop tool at dock, load filament, purge in dock, then pick up
        self.dock_purge = config.getboolean("dock_purge", False)                       # Set to True to enable dock purge during load
        self.dock_purge_length = config.getfloat("dock_purge_length", 50.0)            # mm of filament to extrude while docked for purging
        self.dock_purge_speed = config.getfloat("dock_purge_speed", 5.0)               # mm/s extrude speed during dock purge

        # Tool crash detection: stop/start detection around dock operations
        # Options: "disabled" (default), "tool", "probe"
        crash_detection_raw = config.get("crash_detection", "disabled")
        crash_detection_mode = str(crash_detection_raw).strip().lower()
        if crash_detection_mode in {"0", "off", "false", "disabled", "disable"}:
            crash_detection_mode = "disabled"
        elif crash_detection_mode in {"tool", "probe"}:
            pass
        else:
            self.logger.warning(
                f"Unknown crash_detection '{crash_detection_raw}', defaulting to disabled."
            )
            crash_detection_mode = "disabled"
        self.crash_detection_mode = crash_detection_mode

        # FPS (Filament Pressure Sensor) integration: when the extruder uses
        # an AMS_extruder# virtual pin (shared FPS with OpenAMS), the FPS
        # value is used as the toolhead sensor.  fps_value 0-1, where 1 means
        # filament is fully compressed against extruder gears.
        self.fps_threshold = config.getfloat("fps_threshold", 0.9)
        self._fps_obj = None       # resolved FPS object (fps.py)
        self._fps_extruder = None  # the extruder object associated with FPS
        self._fps_runout_helper = None  # cached runout helper for virtual sensor
        # Latch: during active operations (calibration/feed), once the FPS
        # crosses the threshold we latch tool_start_state=True so that
        # pulsed ACE feeding doesn't clear it between motor pulses.
        self._fps_latched = False

        # Sensor polling interval for status/runout monitoring
        self.poll_interval = config.getfloat("poll_interval", 1.0)

        # Baud rate (ACE default is 115200)
        self.baud_rate = config.getint("baud_rate", 115200)

        # Serial connection (created on ready)
        self._ace: Optional[ACEConnection] = None

        # Slot inventory cache: [{status, material, color}, ...]
        self._slot_inventory = [{} for _ in range(self.SLOTS_PER_UNIT)]

        # Cached hardware status (refreshed by _poll_slot_status, never by get_status)
        self._cached_hw_status = {}

        # For combined mode: track which slot is currently in the toolhead
        # -1 means nothing loaded
        self._current_loaded_slot = -1

        # Previous slot states for runout detection
        self._prev_slot_states: Dict[str, bool] = {}

        # Set True during active operations (load, unload, calibration) to
        # prevent heartbeat/poll callbacks from modifying lane state
        self._operation_active = False

        # Set True while ACE dryer is running so slot status polling
        # skips runout detection (drying can cause transient slot changes)
        self._drying_active = False

        # Track which slots have feed assist actively running on hardware.
        # Used to restore feed assist after ACE reconnection.
        self._feed_assist_active: set = set()

        # Periodic feed assist refresh: re-send start_feed_assist every
        # ~30 seconds to guard against the ACE firmware silently dropping
        # the assist motor (internal timeout, brief busy state, etc.).
        self._feed_assist_refresh_counter = 0
        self._FEED_ASSIST_REFRESH_INTERVAL = 7  # heartbeats (~15s at 2s interval)

        # When True, the next successful heartbeat response will restore
        # feed assist for all tracked slots before clearing the flag.
        self._pending_feed_assist_restore = False

        self.printer.register_event_handler("klippy:ready", self._handle_ready)

        # Register temperature_ace sensor factory early enough for
        # [temperature_sensor] sections to resolve sensor_type: temperature_ace.
        # Must happen during config parsing (before sensors are instantiated).
        try:
            from extras.temperature_ace import TemperatureACE
            pheaters = self.printer.load_object(config, "heaters")
            pheaters.add_sensor_factory("temperature_ace", TemperatureACE)
        except Exception:
            pass

        # Apply the OpenAMS virtual pin patch so that AMS_extruder# values
        # in pin_tool_start are handled correctly.  AFC_ACE configs load
        # alphabetically before AFC_OpenAMS, so without this the patch would
        # not be in place when [AFC_extruder] sections are parsed.
        try:
            from extras.AFC_OpenAMS import _patch_extruder_for_virtual_ams
            _patch_extruder_for_virtual_ams()
        except Exception:
            pass

    def _parse_lane_overrides(self, config: ConfigWrapper):
        """Scan [AFC_lane] sections for per-lane feed/retract overrides."""
        unit_name = self.name
        for section in config.fileconfig.sections():
            if not section.startswith("AFC_lane "):
                continue
            lane_cfg = config.getsection(section)
            unit_val = lane_cfg.get("unit", "")
            if unit_val.split(":")[0] != unit_name:
                continue
            lane_name = section.split()[-1]
            feed_len = lane_cfg.getfloat("feed_length", None)
            retract_len = lane_cfg.getfloat("retract_length", None)
            feed_assist = lane_cfg.getboolean("use_feed_assist", None)
            if feed_len is not None:
                self._lane_feed_length[lane_name] = feed_len
            if retract_len is not None:
                self._lane_retract_length[lane_name] = retract_len
            if feed_assist is not None:
                self._lane_feed_assist[lane_name] = feed_assist

    def _handle_ready(self):
        """Schedule deferred init - reactor pause is disabled during klippy:ready."""
        self.afc.reactor.register_callback(self._deferred_init)

    # USB devices may not be enumerated yet at Klipper startup.
    # Retry with backoff so a full reboot doesn't require a manual
    # FIRMWARE_RESTART just to bring the ACE online.
    _CONNECT_MAX_RETRIES = 5
    _CONNECT_RETRY_DELAYS = [2.0, 3.0, 5.0, 8.0, 10.0]

    def _deferred_init(self, eventtime):
        """Connect to ACE hardware after reactor is fully running."""
        last_err = None
        for attempt in range(self._CONNECT_MAX_RETRIES):
            try:
                self._ace = ACEConnection(
                    reactor=self.afc.reactor,
                    serial_port=self.serial_port,
                    logger=self.logger,
                    baud_rate=self.baud_rate,
                )
                self._ace.connect()
                if attempt > 0:
                    self.logger.info(
                        f"ACE {self.name}: connected on attempt "
                        f"{attempt + 1}"
                    )
                break
            except Exception as e:
                last_err = e
                self._ace = None
                delay = self._CONNECT_RETRY_DELAYS[attempt]
                if attempt < self._CONNECT_MAX_RETRIES - 1:
                    self.logger.info(
                        f"ACE {self.name}: serial port not available, "
                        f"retrying in {delay:.0f}s "
                        f"(attempt {attempt + 1}/{self._CONNECT_MAX_RETRIES})"
                    )
                    self.afc.reactor.pause(
                        self.afc.reactor.monotonic() + delay
                    )
        else:
            self.logger.error(
                f"ACE {self.name}: failed to connect to ACE at "
                f"{self.serial_port} after {self._CONNECT_MAX_RETRIES} "
                f"attempts: {last_err}\n{traceback.format_exc()}"
            )
            return

        # Enable RFID reader so get_filament_info returns spool data
        try:
            self._ace.enable_rfid()
            self.logger.debug(f"ACE {self.name}: RFID enabled")
        except Exception as e:
            self.logger.warning(
                f"ACE {self.name}: enable_rfid failed (non-fatal): {e}"
            )

        # Seed slot status from get_status (needed before _sync_slot_loaded_state)
        try:
            hw_status = self._ace.get_status(timeout=2.0)
            if isinstance(hw_status, dict):
                self._cached_hw_status = hw_status
                for i, slot_data in enumerate(hw_status.get("slots", [])):
                    if i < self.SLOTS_PER_UNIT and isinstance(slot_data, dict):
                        self._slot_inventory[i]["status"] = slot_data.get("status", "")
        except Exception as e:
            self.logger.debug(f"ACE {self.name}: initial get_status failed: {e}")

        # Sync RFID data and lane loaded states
        self._sync_inventory()
        self._sync_slot_loaded_state()

        # Register callbacks for heartbeat status updates and reconnection
        self._ace.status_callback = self._on_hw_status_callback
        self._ace.reconnect_callback = self._on_ace_reconnected

        # Resolve FPS sensor for this unit's extruder (if one exists)
        self._resolve_fps_sensor()

        # Hydrate the virtual tool sensor for lanes that were tool-loaded
        # before restart.  OpenAMS's _hydrate_from_saved_state skips non-
        # OpenAMS lanes, so ACE must set the sensor itself.
        self._hydrate_virtual_tool_sensor(eventtime)

        # Start runout detection polling
        self._start_slot_status_monitor()

        self.logger.info(
            f"ACE {self.name}: connected, mode={self.mode}, "
            f"port={self.serial_port}, slots={self.SLOTS_PER_UNIT}"
        )

    def _get_feed_assist_for_slot(self, slot_index) -> bool:
        """Check if feed assist is enabled for a specific slot."""
        override = self._slot_feed_assist.get(slot_index)
        if override is not None:
            return override
        return self._default_feed_assist

    def _get_feed_length(self, lane=None) -> float:
        """Get effective feed_length, checking lane override first."""
        if lane is not None:
            name = getattr(lane, 'name', None)
            if name and name in self._lane_feed_length:
                return self._lane_feed_length[name]
        return self.feed_length

    def _get_retract_length(self, lane=None) -> float:
        """Get effective retract_length, checking lane override first."""
        if lane is not None:
            name = getattr(lane, 'name', None)
            if name and name in self._lane_retract_length:
                return self._lane_retract_length[name]
        return self.retract_length

    def _get_feed_assist(self, slot_index, lane=None) -> bool:
        """Get effective feed assist: lane override > slot override > unit default."""
        if lane is not None:
            name = getattr(lane, 'name', None)
            if name and name in self._lane_feed_assist:
                return self._lane_feed_assist[name]
        return self._get_feed_assist_for_slot(slot_index)

    # ---- FPS (Filament Pressure Sensor) Integration ----

    def _resolve_fps_sensor(self):
        """Find the FPS object that feeds into this unit's extruder.

        Only activates when the extruder's ``pin_tool_start`` is set to an
        ``AMS_extruder#`` value -this indicates the extruder uses a shared
        FPS as its toolhead sensor.

        Iterates over all ``fps <name>`` printer objects and picks the one
        whose ``extruder_name`` matches this ACE unit's extruder.  When
        found, registers an ADC callback so ``extruder.tool_start_state`` is
        updated in real-time as the FPS value crosses the threshold.
        """
        extruder_name = getattr(self, "extruder", None)
        if not extruder_name:
            return

        # Resolve the extruder object first to check pin_tool_start
        try:
            extruder_obj = self.printer.lookup_object(
                f"AFC_extruder {extruder_name}", None
            )
        except Exception:
            extruder_obj = None

        if extruder_obj is None:
            return

        # Only activate FPS integration when pin_tool_start is AMS_extruder#
        tool_start = getattr(extruder_obj, "tool_start", None)
        if not tool_start or not isinstance(tool_start, str):
            return
        cleaned = tool_start.strip()
        for ch in "#;":
            idx = cleaned.find(ch)
            if idx != -1:
                cleaned = cleaned[:idx].strip()
        if not cleaned.upper().startswith("AMS_"):
            return

        self._fps_extruder = extruder_obj

        # Cache the virtual filament sensor's runout helper so the FPS
        # callback can update it (drives Klipper's filament sensor state
        # and runout detection).
        fila = getattr(extruder_obj, "fila_tool_start", None)
        helper = getattr(fila, "runout_helper", None) if fila else None
        self._fps_runout_helper = helper

        # Scan printer objects for fps instances matching our extruder.
        # Try known fps names (fps1, fps2, ...) via lookup_object, then
        # fall back to iterating printer.objects if available.
        found = False
        for i in range(1, 9):
            fps_name = f"fps fps{i}"
            fps_obj = self.printer.lookup_object(fps_name, None)
            if fps_obj is None:
                continue
            fps_ext = getattr(fps_obj, "extruder_name", None)
            if fps_ext and fps_ext == extruder_name:
                self._fps_obj = fps_obj
                self.logger.info(
                    f"ACE {self.name}: linked to FPS '{fps_name}' "
                    f"(extruder={extruder_name}, pin_tool_start={tool_start}, "
                    f"threshold={self.fps_threshold})"
                )
                fps_obj.add_callback(self._fps_adc_callback)
                found = True
                break

        if not found:
            # Fallback: iterate printer.objects dict if accessible
            objects = getattr(self.printer, "objects", {})
            for obj_name, obj in objects.items():
                if not obj_name.startswith("fps "):
                    continue
                fps_ext = getattr(obj, "extruder_name", None)
                if fps_ext and fps_ext == extruder_name:
                    self._fps_obj = obj
                    self.logger.info(
                        f"ACE {self.name}: linked to FPS '{obj_name}' "
                        f"(extruder={extruder_name}, pin_tool_start={tool_start}, "
                        f"threshold={self.fps_threshold})"
                    )
                    obj.add_callback(self._fps_adc_callback)
                    found = True
                    break

        if not found:
            self.logger.warning(
                f"ACE {self.name}: extruder '{extruder_name}' uses "
                f"pin_tool_start={tool_start} but no matching FPS found"
            )

    def _hydrate_virtual_tool_sensor(self, eventtime):
        """Set the virtual tool sensor if an ACE lane was loaded at shutdown.

        At startup the FPS pressure is zero (ACE motor off), so the ADC
        callback would never set the sensor.  Check saved state and set
        it explicitly so the extruder knows filament is present.
        """
        extruder = self._fps_extruder
        if extruder is None:
            return

        for lane in self.lanes.values():
            lane_name = getattr(lane, "name", None)
            if not getattr(lane, "tool_loaded", False):
                continue
            # Verify extruder agrees this lane is loaded
            if getattr(extruder, "lane_loaded", None) != lane_name:
                continue
            extruder.tool_start_state = True
            self._update_virtual_sensor(eventtime, True)
            self.logger.info(
                f"ACE {self.name}: hydrated virtual tool sensor "
                f"for {lane_name} (tool_loaded at startup)"
            )
            return

    def _fps_adc_callback(self, read_time, fps_value):
        """ADC callback from the FPS sensor -update the virtual tool sensor.

        Called every ~100 ms with the current pressure reading (0.0-1.0).
        When the value crosses the threshold, updates the extruder's
        tool_start_state so that ``lane.get_toolhead_pre_sensor_state()``
        reflects filament presence at the extruder gears.  Also updates
        the virtual filament sensor (fila_tool_start) so Klipper's
        filament sensor system tracks the state and can trigger runout.

        During active operations (calibration/feeding), the state is
        latched: once triggered it stays True until the operation clears
        the latch.  This prevents pulsed ACE feeding from briefly
        dropping the FPS value between motor pulses and clearing the
        triggered state before the calibration loop can see it.
        """
        extruder = self._fps_extruder
        if extruder is None:
            return

        triggered = fps_value >= self.fps_threshold

        if self._operation_active:
            # Latch mode: once triggered, stay triggered
            if triggered and not self._fps_latched:
                self._fps_latched = True
                extruder.tool_start_state = True
                self._update_virtual_sensor(read_time, True)
                self.logger.debug(
                    f"ACE FPS: tool_start_state LATCHED True "
                    f"(fps_value={fps_value:.3f}, threshold={self.fps_threshold})"
                )
            # Don't clear tool_start_state while latched
            return

        # Normal mode: track the sensor state directly.
        # When a lane is loaded to the toolhead, do NOT clear the sensor
        # on pressure drop.  The FPS is pressure-based: it only reads high
        # while the ACE motor is actively pushing.  After feeding stops the
        # pressure falls below threshold even though filament is still
        # present, so clearing the sensor would incorrectly signal a runout.
        if not triggered and getattr(extruder, "lane_loaded", None) is not None:
            return

        if extruder.tool_start_state != triggered:
            extruder.tool_start_state = triggered
            self._update_virtual_sensor(read_time, triggered)
            self.logger.debug(
                f"ACE FPS: tool_start_state -> {triggered} "
                f"(fps_value={fps_value:.3f}, threshold={self.fps_threshold})"
            )

    def _update_virtual_sensor(self, eventtime, filament_present):
        """Push filament state into the virtual filament sensor.

        Updates the runout helper on the extruder's fila_tool_start so
        that Klipper's filament sensor system (QUERY_FILAMENT_SENSOR,
        SET_FILAMENT_SENSOR, runout detection) reflects the FPS state.
        """
        helper = self._fps_runout_helper
        if helper is None:
            return
        try:
            helper.note_filament_present(eventtime, filament_present)
        except TypeError:
            helper.note_filament_present(filament_present)

    def get_fps_value(self):
        """Return the current FPS pressure value, or None if no FPS linked."""
        if self._fps_obj is not None:
            return self._fps_obj.fps_value
        return None

    # ---- Slot / Lane Mapping ----

    def _get_local_slot_for_lane(self, lane) -> int:
        """Map an AFC lane to a local ACE slot index (0-3).

        AFC_lane stores the slot index as lane.index (1-based from config
        'unit: ace1:1'). We convert to 0-based for ACE hardware.
        """
        index = getattr(lane, "index", 0)
        if isinstance(index, int) and 1 <= index <= self.SLOTS_PER_UNIT:
            return index - 1  # Config is 1-based, ACE is 0-based
        return -1

    def _disable_extruder_buffers(self, cur_extruder, cur_lane):
        """Disable any active buffers on the shared extruder from other units.

        When an ACE lane shares an extruder with a stepper-based unit (e.g. Turtle),
        the other unit's buffer may still be active and its fault detection timer
        can trigger 'AFC NOT FEEDING' errors during ACE operations. This disables
        those buffers to prevent cross-lane fault detection.
        """
        for lane_name, lane in cur_extruder.lanes.items():
            if lane_name == cur_lane.name:
                continue
            buf = getattr(lane, "buffer_obj", None)
            if buf is not None and getattr(buf, "enable", False):
                self.logger.info(
                    f"ACE: disabling buffer '{buf.name}' on shared extruder "
                    f"lane {lane_name} to prevent cross-lane fault detection"
                )
                buf.disable_buffer()

    def _dock_purge_dropoff(self):
        """Drop off current tool at dock for dock purging.

        Enters docking mode and runs the toolchanger's dropoff gcode so the
        nozzle rests on the dock pad while filament is loaded and purged.
        """
        tc = self.printer.lookup_object('toolchanger')
        tool = tc.active_tool
        if not tool:
            self.logger.warning("ACE dock purge: no active tool, skipping dropoff")
            return

        self.afc.gcode.run_script_from_command("ENTER_DOCKING_MODE")

        gcode_pos = list(tc.gcode_move.get_status()['gcode_position'])
        start_pos = tc._position_with_tool_offset(gcode_pos, None)
        self._dock_purge_context = {
            'dropoff_tool': tool.name,
            'pickup_tool': tool.name,
            'start_position': tc._position_to_xyz(start_pos, 'xyz'),
            'restore_position': tc._position_to_xyz(start_pos, 'XYZ'),
        }

        tc.run_gcode('tool.dropoff_gcode', tool.dropoff_gcode, self._dock_purge_context)

    def _dock_purge_pickup(self):
        """Pick up tool from dock after purging.

        Runs the toolchanger's pickup gcode (nozzle wipes on pad during pickup)
        and exits docking mode to restore normal operation.
        """
        tc = self.printer.lookup_object('toolchanger')
        tool = tc.active_tool
        if not tool or not hasattr(self, '_dock_purge_context'):
            self.logger.warning("ACE dock purge: no context for pickup, skipping")
            return

        tc.run_gcode('tool.pickup_gcode', tool.pickup_gcode, self._dock_purge_context)
        self.afc.gcode.run_script_from_command("EXIT_DOCKING_MODE")
        self._dock_purge_context = None

    def _run_tool_crash_detection(self, enable):
        """Start or stop tool crash detection around dock operations.

        Mirrors OpenAMS behaviour: supports 'tool' and 'probe' modes.
        Returns True on success or if detection is disabled (no-op).
        """
        if self.crash_detection_mode == "disabled":
            return True
        gcode = self.afc.gcode
        if enable:
            if self.crash_detection_mode == "probe":
                commands = ("START_TOOL_PROBE_CRASH_DETECTION",)
            else:
                commands = ("START_TOOL_CRASH_DETECTION",)
        else:
            if self.crash_detection_mode == "probe":
                commands = ("STOP_TOOL_PROBE_CRASH_DETECTION",)
            else:
                commands = ("STOP_TOOL_CRASH_DETECTION",)
        last_exc = None
        for command in commands:
            for candidate in (command, command.lower()):
                try:
                    self.logger.debug(f"Running tool crash detection command: {candidate}")
                    gcode.run_script_from_command(candidate)
                    self.logger.debug(f"Tool crash detection command completed: {candidate}")
                    return True
                except Exception as exc:
                    self.logger.debug(
                        f"Tool crash detection command failed: {candidate} ({exc})"
                    )
                    last_exc = exc
                    continue
        if last_exc is not None:
            self.logger.debug(f"Skipping tool crash detection; failed {commands}: {last_exc}")
        else:
            self.logger.debug("Skipping tool crash detection command; none available")
        return False

    # ---- Inventory / State Sync ----

    def _sync_inventory(self):
        """Pull RFID/NFC filament data from ACE hardware into local cache."""
        if self._ace is None or not self._ace.connected:
            return

        for slot in range(self.SLOTS_PER_UNIT):
            try:
                info = self._ace.get_filament_info(slot)
                if isinstance(info, dict):
                    # Update material/color from RFID data, preserve status
                    # (status comes from get_status, not get_filament_info)
                    self._slot_inventory[slot]["material"] = info.get("material", info.get("type", ""))
                    self._slot_inventory[slot]["color"] = info.get("color", [0, 0, 0])
            except Exception as e:
                self.logger.debug(
                    f"ACE {self.name}: slot {slot} inventory query failed: {e}"
                )

    def _clear_slot_inventory(self, slot):
        """Clear cached RFID data for a slot so stale info isn't reapplied."""
        if 0 <= slot < self.SLOTS_PER_UNIT:
            self._slot_inventory[slot]["material"] = ""
            self._slot_inventory[slot]["color"] = [0, 0, 0]

    def _refresh_slot_inventory(self, slot):
        """Fetch fresh RFID data for a single slot from ACE hardware."""
        if self._ace is None or not self._ace.connected:
            return
        if slot < 0 or slot >= self.SLOTS_PER_UNIT:
            return
        try:
            info = self._ace.get_filament_info(slot)
            if isinstance(info, dict):
                self._slot_inventory[slot]["material"] = info.get("material", info.get("type", ""))
                self._slot_inventory[slot]["color"] = info.get("color", [0, 0, 0])
        except Exception as e:
            self.logger.debug(
                f"ACE {self.name}: slot {slot} RFID refresh failed: {e}"
            )

    def _sync_slot_loaded_state(self):
        """Sync ACE slot status to lane load/prep state."""
        if self._ace is None or not self._ace.connected:
            return

        for lane in self.lanes.values():
            local_slot = self._get_local_slot_for_lane(lane)
            if 0 <= local_slot < self.SLOTS_PER_UNIT:
                slot_info = self._slot_inventory[local_slot]
                slot_loaded = bool(
                    slot_info and slot_info.get("status", "") == "ready"
                )
                lane._load_state = slot_loaded
                lane.prep_state = slot_loaded

                # Only set material/color if the lane doesn't already have
                # values (from Spoolman, manual set, or previous RFID read).
                # Never overwrite existing assignments.
                self._apply_slot_filament_defaults(lane, slot_info)

    def _apply_slot_filament_defaults(self, lane, slot_info):
        """Apply filament material/color to a lane if not already set.

        Priority: existing lane values > Spoolman next_spool_id > ACE RFID > AFC defaults.
        Never overwrites values that are already set.
        """
        # If lane already has material and color set, don't touch them
        has_material = getattr(lane, "material", None) not in (None, "")
        has_color = getattr(lane, "color", None) not in (None, "", "#000000")

        if has_material and has_color:
            return

        # Try ACE RFID data from slot inventory
        rfid_material = slot_info.get("material", "") if slot_info else ""
        rfid_color = slot_info.get("color", [0, 0, 0]) if slot_info else [0, 0, 0]

        # Treat "unknown" material from 3rd-party spools as empty
        if rfid_material and rfid_material.lower() == "unknown":
            rfid_material = ""
        rfid_has_data = bool(rfid_material) or rfid_color != [0, 0, 0]

        if rfid_has_data:
            if not has_material and rfid_material:
                lane.material = rfid_material
            if not has_color and rfid_color != [0, 0, 0]:
                lane.color = self._ace_color_to_hex(rfid_color)

        # Apply AFC defaults for anything still missing
        if not has_material and not getattr(lane, "material", None):
            default_mat = getattr(self.afc, "default_material_type", None)
            if default_mat:
                lane.material = default_mat
        if not has_color and not getattr(lane, "color", None):
            default_color = getattr(self.afc, "default_color", None)
            if default_color:
                lane.color = default_color
        if not getattr(lane, "weight", 0):
            lane.weight = 1000

    def _restore_tool_loaded_state(self, lane):
        """Restore a lane to TOOLED state after klipper restart.

        Called when the heartbeat/poll detects a slot is ready and the lane's
        saved state indicates it was previously loaded to the toolhead.  This
        covers the timing gap where _deferred_init completes after PREP has
        already restored saved vars but system_Test couldn't reach the ACE.
        """
        lane.loaded_to_hub = True
        lane.tool_loaded = True
        lane.status = AFCLaneState.TOOLED
        lane.extruder_obj.lane_loaded = lane.name
        lane.sync_to_extruder()
        self.afc.spool.set_active_spool(lane.spool_id)

        # Hydrate virtual sensor so extruder knows filament is present
        extruder = lane.extruder_obj
        extruder.tool_start_state = True
        fila = getattr(extruder, "fila_tool_start", None)
        helper = getattr(fila, "runout_helper", None) if fila else None
        if helper is not None:
            eventtime = self.afc.reactor.monotonic()
            try:
                helper.note_filament_present(eventtime, True)
            except TypeError:
                helper.note_filament_present(True)

        self.lane_tool_loaded(lane)

        # Start feed assist immediately
        local_slot = self._get_local_slot_for_lane(lane)
        if (local_slot >= 0
                and self._get_feed_assist(local_slot, lane)
                and self._ace is not None):
            try:
                self._ace.start_feed_assist(local_slot)
                self._feed_assist_active.add(local_slot)
            except Exception:
                pass

        # Restore combined mode slot tracking
        if self.mode == MODE_COMBINED and local_slot >= 0:
            self._current_loaded_slot = local_slot

        self.afc.save_vars()
        self.logger.info(
            f"ACE: restored TOOLED state for {lane.name}"
        )

    def _on_hw_status_callback(self, response):
        """Process slot status from any ACE response (including heartbeat).

        Called from the serial layer's reactor thread for every response that
        doesn't match a pending synchronous request - primarily the heartbeat
        get_status responses that fire every 2 seconds.
        """
        if not isinstance(response, dict):
            return

        # Restore feed assist after reconnection - the first successful
        # heartbeat response confirms the connection is stable.
        if self._pending_feed_assist_restore:
            self._restore_feed_assist()
            self._feed_assist_refresh_counter = 0

        # Periodic feed assist enforcement: any lane that is tool_loaded on
        # its active extruder should have feed assist running if configured.
        # This catches cases where _feed_assist_active lost track of a slot
        # (e.g. startup recovery, manual SET_LANE_LOADED, firmware drop).
        # Also re-sends the command every ~15s to guard against the ACE
        # firmware silently disabling the assist motor.
        if not self._operation_active and self._ace is not None:
            self._feed_assist_refresh_counter += 1
            if self._feed_assist_refresh_counter >= self._FEED_ASSIST_REFRESH_INTERVAL:
                self._feed_assist_refresh_counter = 0
                for lane in self.lanes.values():
                    if not getattr(lane, "tool_loaded", False):
                        continue
                    local_slot = self._get_local_slot_for_lane(lane)
                    if local_slot < 0:
                        continue
                    if not self._get_feed_assist(local_slot, lane):
                        continue
                    try:
                        self._ace.start_feed_assist(local_slot)
                        self._feed_assist_active.add(local_slot)
                    except Exception:
                        pass

        # Skip lane state updates during active operations (load/unload/calibration)
        if self._operation_active:
            return

        # Extract result payload (JSON-RPC wraps it)
        result = response.get("result", response)
        if not isinstance(result, dict):
            return

        slots = result.get("slots")
        if not slots or not isinstance(slots, list):
            return

        # Update cached hardware status and slot inventory
        self._cached_hw_status = result
        for i, slot_data in enumerate(slots):
            if i < self.SLOTS_PER_UNIT and isinstance(slot_data, dict):
                self._slot_inventory[i]["status"] = slot_data.get("status", "")

        # Sync slot states and ensure lane consistency
        for lane in self.lanes.values():
            local_slot = self._get_local_slot_for_lane(lane)
            if local_slot < 0 or local_slot >= self.SLOTS_PER_UNIT:
                continue

            slot_info = self._slot_inventory[local_slot]
            slot_status = slot_info.get("status", "") if slot_info else ""
            slot_ready = slot_status == "ready"
            slot_shifting = slot_status == "shifting"

            # Keep load/prep state in sync with slot status
            lane._load_state = slot_ready
            lane.prep_state = slot_ready

            prep_done = getattr(lane, '_afc_prep_done', False)

            # State consistency: if hardware says ready but lane is stuck
            # in NONE, fix it regardless of transition detection
            if slot_ready and prep_done and lane.status == AFCLaneState.NONE:
                # Check if this lane was tool-loaded before restart -
                # restore TOOLED state instead of just LOADED.
                if (lane.tool_loaded
                        and hasattr(lane, 'extruder_obj')
                        and lane.extruder_obj.lane_loaded == lane.name):
                    self.logger.info(
                        f"ACE callback: {lane.name} slot {local_slot} ready, "
                        f"restoring TOOLED state from saved vars"
                    )
                    self._restore_tool_loaded_state(lane)
                else:
                    self.logger.info(
                        f"ACE callback: {lane.name} slot {local_slot} ready, "
                        f"setting loaded"
                    )
                    # New filament insertion: clear old data, apply fresh.
                    self.afc.spool.clear_values(lane)
                    lane.set_loaded()
                    self._refresh_slot_inventory(local_slot)
                    slot_info = self._slot_inventory[local_slot]
                    self._apply_slot_filament_defaults(lane, slot_info)
                    self.lane_illuminate_spool(lane)
                    self.afc.save_vars()

            # When a slot goes empty, transition the lane to unloaded
            # so new filament insertion is properly detected (NONE gate).
            if not slot_ready and not slot_shifting:
                prev_ready_cb = self._prev_slot_states.get(lane.name)
                if prev_ready_cb:
                    self._clear_slot_inventory(local_slot)
                    if lane.status == AFCLaneState.LOADED:
                        lane.set_unloaded()
                        self.lane_not_ready(lane)
                        self.afc.save_vars()

            # Don't update prev state during transient "shifting" to
            # avoid false runout triggers in _poll_slot_status.
            if not slot_shifting:
                self._prev_slot_states[lane.name] = slot_ready

    def _on_ace_reconnected(self):
        """Called after ACE reconnects (power cycle or USB disconnect).

        Defers feed assist restoration until the first successful heartbeat
        to ensure the connection is stable before sending commands.
        """
        if self._feed_assist_active:
            self.logger.info(
                f"ACE reconnect: deferring feed assist restore for "
                f"slots {sorted(self._feed_assist_active)} until first heartbeat"
            )
            self._pending_feed_assist_restore = True
        else:
            self.logger.debug("ACE reconnect: no feed assist to restore")

    def _restore_feed_assist(self):
        """Restore feed assist for all tracked active slots after reconnect."""
        self._pending_feed_assist_restore = False
        if not self._feed_assist_active or self._ace is None:
            return

        for slot_index in sorted(self._feed_assist_active):
            try:
                self._ace.start_feed_assist(slot_index)
                self.logger.info(
                    f"ACE reconnect: feed assist restored for slot {slot_index}"
                )
            except Exception as e:
                self.logger.warning(
                    f"ACE reconnect: failed to restore feed assist "
                    f"for slot {slot_index}: {e}"
                )

    def get_slot_info(self, local_slot):
        """Return cached slot info dict for a slot index."""
        if 0 <= local_slot < self.SLOTS_PER_UNIT:
            return self._slot_inventory[local_slot]
        return {}

    # ---- AFC Unit Interface Implementation ----

    def handle_connect(self):
        super().handle_connect()
        self._register_gcode_commands()
        self._wrap_get_current_lane()

        self.logo = '<span class=success--text>R  _____ _____ _____\n'
        self.logo += 'E | AFC | ACE |     |\n'
        self.logo += 'A |  P  |  R  |  O  |\n'
        self.logo += 'D |_____|_____|_____|\n'
        self.logo += 'Y |_1_|_2_|_3_|_4_|\n'
        self.logo += '  ' + self.name + ' (' + self.mode + ')\n'

        self.logo_error = '<span class=error--text>E  _____ _____ _____\n'
        self.logo_error += 'R | AFC | ACE |     |\n'
        self.logo_error += 'R | ERR | ERR | ERR |\n'
        self.logo_error += 'O |_____|_____|_____|\n'
        self.logo_error += 'R |_X_|_X_|_X_|_X_|\n'
        self.logo_error += '  ' + self.name + '</span>\n'

    def get_status(self, eventtime=None):
        """Return status dict including cached ACE hardware state.

        IMPORTANT: This must NOT send serial commands - Klipper calls this
        multiple times per second for UI updates. Use cached data only.
        The cache is refreshed by _poll_slot_status at poll_interval.
        """
        response = super().get_status(eventtime)

        response["ace_mode"] = self.mode
        response["ace_connected"] = (
            self._ace is not None and self._ace.connected
        )
        response["ace_serial_port"] = self.serial_port
        response["ace_status"] = self._cached_hw_status
        response["ace_drying"] = self._drying_active

        # Hardware temps/humidity from cached get_status response
        hw = self._cached_hw_status
        response["ace_temp"] = hw.get("temp")
        response["ace_humidity"] = hw.get("humidity")

        # Feed assist state: list of slot indices with active feed assist
        response["ace_feed_assist_slots"] = sorted(self._feed_assist_active)

        return response

    def lane_tool_loaded(self, lane):
        """Set the virtual tool sensor when an ACE lane loads into the toolhead.

        The base class only updates LEDs.  For ACE with a shared AMS virtual
        pin, the FPS pressure is zero when the ACE motor isn't running, so the
        virtual sensor must be set explicitly whenever a lane becomes tool-loaded
        (normal load sequence, SET_LANE_LOADED recovery macro, etc.).
        """
        super().lane_tool_loaded(lane)
        extruder = self._fps_extruder
        if extruder is None:
            return
        eventtime = self.afc.reactor.monotonic()
        extruder.tool_start_state = True
        self._update_virtual_sensor(eventtime, True)

    def lane_tool_unloaded(self, lane):
        """Clear the virtual tool sensor when an ACE lane unloads.

        Mirrors lane_tool_loaded: explicitly clears the sensor so the extruder
        knows the toolhead is empty.
        """
        super().lane_tool_unloaded(lane)
        extruder = self._fps_extruder
        if extruder is None:
            return
        eventtime = self.afc.reactor.monotonic()
        extruder.tool_start_state = False
        self._update_virtual_sensor(eventtime, False)

    def load_sequence(self, cur_lane, cur_hub, cur_extruder):
        """Load filament from ACE slot into the toolhead.

        In combined mode: retracts the currently loaded slot first.
        In direct mode: feeds independently (each lane has its own extruder).
        """
        afc = self.afc
        self._operation_active = True
        self._fps_latched = False
        try:
            return self._load_sequence_inner(cur_lane, cur_hub, cur_extruder)
        finally:
            self._operation_active = False
            self._fps_latched = False

    def _load_sequence_inner(self, cur_lane, cur_hub, cur_extruder):
        afc = self.afc

        if self._ace is None or not self._ace.connected:
            afc.error.handle_lane_failure(
                cur_lane,
                f"ACE load failed: ACE not connected ({self.serial_port})",
            )
            return False

        # Disable any active buffers on the shared extruder from other units.
        # This prevents cross-lane buffer fault detection (e.g. a Turtle buffer
        # on the same extruder) from triggering "AFC NOT FEEDING" during ACE ops.
        self._disable_extruder_buffers(cur_extruder, cur_lane)

        # Check if this lane is already loaded
        if (cur_lane.get_toolhead_pre_sensor_state()
                and hasattr(cur_lane, "tool_loaded") and cur_lane.tool_loaded):
            self.logger.debug(
                f"Lane {cur_lane.name} already loaded to toolhead, skipping"
            )
            cur_lane.set_tool_loaded()
            self.afc.save_vars()
            return True

        # Check if a different lane is loaded on this extruder and unload it
        # first.  This handles the case where CHANGE_TOOL's normal unload was
        # skipped (e.g. shuttle was empty at startup so self.current was None).
        loaded_lane_name = cur_extruder.lane_loaded
        if loaded_lane_name is not None and loaded_lane_name != cur_lane.name:
            loaded_lane = afc.lanes.get(loaded_lane_name)
            if loaded_lane is not None:
                self.logger.info(
                    f"ACE load: {loaded_lane_name} still loaded on "
                    f"{cur_extruder.name}, unloading first"
                )
                if not afc.TOOL_UNLOAD(loaded_lane, set_start_time=False):
                    afc.error.handle_lane_failure(
                        cur_lane,
                        f"ACE load failed: could not unload {loaded_lane_name} "
                        f"from {cur_extruder.name}",
                    )
                    return False

        if afc._check_extruder_temp(cur_lane):
            afc.afcDeltaTime.log_with_time("Done heating toolhead")

        # Dock purge phase 1: drop off tool at dock before feeding filament
        dock_dropped_off = False
        if self.dock_purge:
            if not self._run_tool_crash_detection(False):
                self.logger.warning("Failed to stop tool crash detection before dock dropoff")
            self.logger.info("ACE dock purge: dropping tool off at dock before feed")
            self._dock_purge_dropoff()
            dock_dropped_off = True
            afc.afcDeltaTime.log_with_time("ACE: After dock purge dropoff")

        # Wrap the rest of the load in try/finally so the tool is always
        # picked back up from the dock even if the load fails.
        load_result = False
        try:
            load_result = self._load_sequence_feed_and_verify(
                cur_lane, cur_hub, cur_extruder, afc
            )
        finally:
            if dock_dropped_off:
                # Always pick up the tool -even on failure
                if load_result and self.dock_purge:
                    # Success path: purge in dock, then pick up
                    self.logger.info(
                        f"ACE dock purge: extruding {self.dock_purge_length}mm "
                        f"@ {self.dock_purge_speed}mm/s in dock, then picking up"
                    )
                    afc.move_e_pos(
                        self.dock_purge_length, self.dock_purge_speed,
                        "dock purge extrude"
                    )
                else:
                    self.logger.info(
                        "ACE dock purge: picking up tool after load failure"
                    )
                self._dock_purge_pickup()
                afc.afcDeltaTime.log_with_time("ACE: After dock purge pickup")
                if not self._run_tool_crash_detection(True):
                    self.logger.warning(
                        "Failed to start tool crash detection after dock pickup"
                    )

        return load_result

    def _load_sequence_feed_and_verify(self, cur_lane, cur_hub,
                                       cur_extruder, afc):
        """Feed filament and verify it reached the toolhead.

        Separated from _load_sequence_inner so dock purge pickup always
        runs via the try/finally wrapper even on failure.
        """
        local_slot = self._get_local_slot_for_lane(cur_lane)
        if local_slot < 0:
            afc.error.handle_lane_failure(
                cur_lane,
                f"ACE load failed: cannot determine slot for {cur_lane.name}",
            )
            return False

        try:
            # Combined mode: retract current slot first
            if self.mode == MODE_COMBINED and self._current_loaded_slot >= 0:
                if self._current_loaded_slot != local_slot:
                    self.logger.info(
                        f"ACE combined mode: retracting slot "
                        f"{self._current_loaded_slot} before loading slot {local_slot}"
                    )
                    # Stop feed assist on old slot before retracting
                    try:
                        self._wait_for_ace_ready()
                        self._ace.stop_feed_assist(self._current_loaded_slot)
                        self._feed_assist_active.discard(self._current_loaded_slot)
                    except Exception:
                        pass
                    self._retract_slot(self._current_loaded_slot)
                    self._current_loaded_slot = -1

            # Feed the target slot (pass lane for sensor-based stopping)
            self.logger.info(
                f"ACE load: feeding slot {local_slot} for lane {cur_lane.name} "
                f"(mode={self.mode})"
            )
            self._feed_slot(local_slot, lane=cur_lane)

            if self.mode == MODE_COMBINED:
                self._current_loaded_slot = local_slot

            # Filament has been fed through the hub into the bowden
            cur_lane.loaded_to_hub = True

        except Exception as e:
            message = f"ACE load failed for {cur_lane.name}: {e}"
            self.logger.error(f"{message}\n{traceback.format_exc()}")
            afc.error.handle_lane_failure(cur_lane, message)
            return False

        # Verify toolhead sensor triggered
        if cur_extruder.tool_start == "buffer" and cur_lane.buffer_obj is not None:
            # Buffer/ramming mode: buffer's advance_state is the sensor.
            # Retract off the buffer sensor to confirm load and reset buffer.
            # ACE lanes have no lane stepper, so use move_e_pos (extruder motor)
            # instead of move_advanced for the retract moves.
            try:
                load_checks = 0
                while cur_lane.get_toolhead_pre_sensor_state():
                    afc.move_e_pos(
                        cur_lane.short_move_dis * -1,
                        cur_extruder.tool_unload_speed,
                        "Buffer decompress", wait_tool=True
                    )
                    load_checks += 1
                    afc.reactor.pause(afc.reactor.monotonic() + 0.1)
                    if load_checks > afc.tool_max_load_checks:
                        message = (
                            f"Buffer did not decompress after {afc.tool_max_load_checks} "
                            f"retract moves. Check filament path and buffer.\n"
                            f"To resolve, set lane loaded with "
                            f"`SET_LANE_LOADED LANE={cur_lane.name}` macro."
                        )
                        afc.error.handle_lane_failure(cur_lane, message)
                        return False
            except Exception as e:
                message = f"ACE buffer load check failed for {cur_lane.name}: {e}"
                self.logger.error(f"{message}\n{traceback.format_exc()}")
                afc.error.handle_lane_failure(cur_lane, message)
                return False
        elif cur_extruder.tool_start:
            # Standard toolhead sensor verification with retry.
            # If the sensor hasn't triggered after the full feed, do up to 5
            # additional 20mm feeds checking the sensor after each one.  This
            # handles slight bowden length variations without immediately
            # failing the load.
            if not cur_lane.get_toolhead_pre_sensor_state():
                smart_load_step = 20.0   # mm per retry
                smart_load_max  = 5      # max retries
                for attempt in range(1, smart_load_max + 1):
                    self.logger.info(
                        f"ACE smart load: sensor not triggered, "
                        f"feeding {smart_load_step}mm (attempt {attempt}/{smart_load_max})"
                    )
                    # Ensure feed assist is running so ACE pushes filament
                    if (local_slot >= 0
                            and self._get_feed_assist(local_slot, cur_lane)
                            and local_slot not in self._feed_assist_active):
                        try:
                            self._wait_for_ace_ready()
                            self._ace.start_feed_assist(local_slot)
                            self._feed_assist_active.add(local_slot)
                        except Exception:
                            pass
                    self._wait_for_ace_ready()
                    self._ace.feed_filament(local_slot, smart_load_step, self.feed_speed)
                    self._wait_for_feed_complete(
                        local_slot, smart_load_step, self.feed_speed,
                        lane=cur_lane, poll_interval=0.3,
                    )
                    afc.reactor.pause(afc.reactor.monotonic() + 0.2)
                    if cur_lane.get_toolhead_pre_sensor_state():
                        self.logger.info(
                            f"ACE smart load: sensor triggered after "
                            f"{attempt * smart_load_step:.0f}mm extra feed"
                        )
                        break
                else:
                    # All retries exhausted -error out
                    message = (
                        f"ACE load did not trigger toolhead sensor after "
                        f"{smart_load_max * smart_load_step:.0f}mm of extra feeding. "
                        f"CHECK FILAMENT PATH\n"
                        "To resolve, set lane loaded with "
                        f"`SET_LANE_LOADED LANE={cur_lane.name}` macro."
                    )
                    if afc.function.in_print():
                        message += (
                            "\nOnce filament is fully loaded click resume to continue printing"
                        )
                    afc.error.handle_lane_failure(cur_lane, message)
                    return False

        # Sync to extruder and load filament into the nozzle using tool_stn
        cur_lane.status = AFCLaneState.TOOL_LOADED
        self.afc.save_vars()
        cur_lane.sync_to_extruder()

        # If tool_end sensor exists, feed until it triggers
        if cur_extruder.tool_end:
            tool_attempts = 0
            while not cur_extruder.tool_end_state:
                tool_attempts += 1
                afc.move_e_pos(
                    cur_lane.short_move_dis, cur_extruder.tool_load_speed,
                    "Tool end", wait_tool=True
                )
                if tool_attempts > 20:
                    message = (
                        "ACE load: filament failed to trigger post-extruder sensor.\n"
                        "To resolve, set lane loaded with "
                        f"`SET_LANE_LOADED LANE={cur_lane.name}` macro."
                    )
                    afc.error.handle_lane_failure(cur_lane, message)
                    return False

        # Push filament into the nozzle using tool_stn distance
        if cur_extruder.tool_stn:
            self.logger.info(
                f"ACE load: advancing {cur_extruder.tool_stn}mm into nozzle "
                f"@ {cur_extruder.tool_load_speed}mm/s"
            )
            afc.move_e_pos(
                cur_extruder.tool_stn, cur_extruder.tool_load_speed, "tool stn"
            )

        cur_lane.set_tool_loaded()
        cur_lane.enable_buffer(disable_fault=True)

        # Ensure feed assist is running while filament is loaded.
        # _feed_slot starts it during phase 3, but if the sensor triggered
        # early (during bulk feed) phase 3 may not have run.
        local_slot = self._get_local_slot_for_lane(cur_lane)
        if local_slot >= 0 and self._get_feed_assist(local_slot, cur_lane):
            try:
                self._wait_for_ace_ready()
                self._ace.start_feed_assist(local_slot)
                self._feed_assist_active.add(local_slot)
                self.logger.debug(
                    f"ACE load: feed assist enabled for slot {local_slot}"
                )
            except Exception as e:
                self.logger.warning(
                    f"ACE load: failed to start feed assist for slot {local_slot}: {e}"
                )

        self.afc.save_vars()
        return True

    def unload_sequence(self, cur_lane, cur_hub, cur_extruder):
        """Unload filament: shared toolhead steps then ACE retraction."""
        afc = self.afc
        self._operation_active = True
        self._fps_latched = False
        try:
            return self._unload_sequence_inner(cur_lane, cur_hub, cur_extruder)
        finally:
            self._operation_active = False
            self._fps_latched = False

    def _unload_sequence_inner(self, cur_lane, cur_hub, cur_extruder):
        afc = self.afc

        if self._ace is None or not self._ace.connected:
            afc.error.handle_lane_failure(
                cur_lane,
                f"ACE unload failed: ACE not connected ({self.serial_port})",
            )
            return False

        # Disable any active buffers on the shared extruder from other units
        self._disable_extruder_buffers(cur_extruder, cur_lane)

        cur_lane.status = AFCLaneState.TOOL_UNLOADING

        # Stop feed assist before unloading - it was kept running while loaded
        local_slot = self._get_local_slot_for_lane(cur_lane)
        if local_slot >= 0 and self._ace is not None:
            try:
                self._wait_for_ace_ready()
                self._ace.stop_feed_assist(local_slot)
                self._feed_assist_active.discard(local_slot)
                self.logger.debug(
                    f"ACE unload: feed assist stopped for slot {local_slot}"
                )
            except Exception as e:
                self.logger.warning(
                    f"ACE unload: failed to stop feed assist for slot {local_slot}: {e}"
                )

        # Disable buffer before unloading (safe no-op if no buffer)
        cur_lane.disable_buffer()

        if afc._check_extruder_temp(cur_lane):
            afc.afcDeltaTime.log_with_time("Done heating toolhead")

        # Quick pull to prevent oozing
        afc.move_e_pos(
            -2, cur_extruder.tool_unload_speed, "Quick Pull", wait_tool=False
        )
        self.lane_unloading(cur_lane)
        cur_lane.sync_to_extruder()
        cur_lane.do_enable(True)
        cur_lane.select_lane()

        # Shared toolhead steps: cut, park, form tip
        if afc.tool_cut:
            cur_lane.extruder_obj.estats.increase_cut_total()
            afc.gcode.run_script_from_command(
                f"{afc.tool_cut_cmd} EXTRUDER={cur_extruder.name}"
            )

            if afc.park:
                afc.gcode.run_script_from_command(
                    f"{afc.park_cmd} EXTRUDER={cur_extruder.name}"
                )

        if afc.form_tip:
            if afc.park:
                afc.gcode.run_script_from_command(
                    f"{afc.park_cmd} EXTRUDER={cur_extruder.name}"
                )

            if afc.form_tip_cmd == "AFC":
                afc.tip = self.printer.lookup_object("AFC_form_tip")
                afc.tip.tip_form()
            else:
                afc.gcode.run_script_from_command(afc.form_tip_cmd)

        # Retract filament out of the nozzle/extruder gears.
        # ACE lanes have no lane stepper, so all retract moves use move_e_pos
        # (extruder motor) instead of move_advanced. After clearing the toolhead,
        # the ACE hardware handles the full bowden retraction via _retract_slot.
        local_slot = self._get_local_slot_for_lane(cur_lane)

        # Start ACE unwind before extruder retract so the ACE begins pulling
        # tension on the spool while the extruder simultaneously retracts.
        # The command returns immediately � the motor runs asynchronously.
        self.logger.info(
            f"ACE unload: starting ACE unwind slot {local_slot} "
            f"before extruder retract for lane {cur_lane.name}"
        )
        retract_length = self._get_retract_length(cur_lane)
        try:
            self._wait_for_ace_ready()
            self._ace.unwind_filament(
                local_slot, retract_length, self.retract_speed
            )
        except Exception as e:
            message = f"ACE unload: failed to start ACE unwind for {cur_lane.name}: {e}"
            self.logger.error(f"{message}\n{traceback.format_exc()}")
            afc.error.handle_lane_failure(cur_lane, message)
            return False

        if cur_extruder.tool_start == "buffer" and cur_lane.buffer_obj is not None:
            # Buffer mode: retract until buffer decompresses using extruder motor
            num_tries = 0
            while not cur_lane.get_trailing() and afc.tool_max_unload_attempts > 0:
                num_tries += 1
                afc.move_e_pos(
                    cur_lane.short_move_dis * -1,
                    cur_extruder.tool_unload_speed,
                    "Buffer retract", wait_tool=True
                )
                afc.reactor.pause(afc.reactor.monotonic() + 0.1)
                if num_tries > afc.tool_max_unload_attempts:
                    message = (
                        f"Buffer did not decompress after {afc.tool_max_unload_attempts} "
                        f"retract moves during unload.\n"
                        "Please check filament is free from toolhead extruder."
                    )
                    afc.error.handle_lane_failure(cur_lane, message)
                    return False
            # Retract tool_stn_unload distance to clear extruder gears
            if cur_extruder.tool_stn_unload > 0:
                self.logger.info(
                    f"ACE unload: buffer retract {cur_extruder.tool_stn_unload}mm "
                    f"@ {cur_extruder.tool_unload_speed}mm/s"
                )
                afc.move_e_pos(
                    cur_extruder.tool_stn_unload * -1,
                    cur_extruder.tool_unload_speed, "Buffer Move",
                    wait_tool=True
                )
        else:
            # Standard mode (no buffer): retract with extruder motor while
            # ACE is already pulling from the spool side.
            retract_distance = cur_extruder.tool_stn_unload
            if retract_distance > 0:
                self.logger.info(
                    f"ACE unload: extruder retract {retract_distance}mm "
                    f"@ {cur_extruder.tool_unload_speed}mm/s to clear nozzle/gears"
                )
                afc.move_e_pos(
                    retract_distance * -1,
                    cur_extruder.tool_unload_speed,
                    "ACE nozzle retract", wait_tool=True
                )

        # Move past the sensor-after-extruder if configured
        if cur_extruder.tool_sensor_after_extruder > 0:
            afc.move_e_pos(
                cur_extruder.tool_sensor_after_extruder * -1,
                cur_extruder.tool_unload_speed, "After extruder"
            )

        try:
            # Unsync from extruder before waiting for ACE retraction
            cur_lane.unsync_to_extruder()

            self.logger.info(
                f"ACE unload: waiting for ACE unwind to complete "
                f"slot {local_slot} for lane {cur_lane.name}"
            )
            self._wait_for_feed_complete(
                local_slot, retract_length, self.retract_speed
            )

            if self.mode == MODE_COMBINED:
                self._current_loaded_slot = -1

            cur_lane.loaded_to_hub = False
            cur_lane.set_tool_unloaded()
            cur_lane.status = AFCLaneState.LOADED
            self.lane_tool_unloaded(cur_lane)
            self.afc.save_vars()

        except Exception as e:
            message = f"ACE unload failed for {cur_lane.name}: {e}"
            self.logger.error(f"{message}\n{traceback.format_exc()}")
            afc.error.handle_lane_failure(cur_lane, message)
            return False

        return True

    # ---- Low-Level Slot Operations ----

    def _wait_for_ace_ready(self, timeout=10.0):
        """Wait for the overall ACE status to be 'ready' before sending commands.

        After a retract/feed completes, the slot may report 'ready'/'empty'
        before the ACE controller finishes its internal housekeeping.  Sending
        a new feed_filament while the ACE is still 'busy' returns FORBIDDEN.
        """
        ace = self._ace
        if ace is None or not ace.connected:
            return
        poll_interval = 0.5
        elapsed = 0.0
        while elapsed < timeout:
            try:
                hw_status = ace.get_status(timeout=2.0)
                if isinstance(hw_status, dict):
                    if hw_status.get("status", "") == "ready":
                        return
                    self.logger.debug(
                        f"ACE: waiting for ACE ready "
                        f"(status={hw_status.get('status', '?')}, "
                        f"{elapsed:.1f}s/{timeout:.0f}s)"
                    )
            except Exception:
                pass
            self.afc.reactor.pause(
                self.afc.reactor.monotonic() + poll_interval
            )
            elapsed += poll_interval
        self.logger.warning(
            f"ACE: ACE did not become ready within {timeout:.0f}s, proceeding anyway"
        )

    def _feed_slot(self, slot_index, lane=None):
        """Feed filament from an ACE slot through to the toolhead.

        Uses a two-phase approach:
        1. Bulk feed: advance most of the bowden distance in one shot (fast)
        2. Sensor approach: feed in small increments, checking the toolhead
           sensor between each step (precise, stops when sensor triggers)
        3. Feed assist + extruder assist for the final stretch into the hotend

        If no lane is provided (no sensor to check), falls back to fixed-distance feed.
        """
        ace = self._ace
        feed_length = self._get_feed_length(lane)

        # Ensure ACE is not still busy from a previous operation
        self._wait_for_ace_ready()

        self.logger.debug(
            f"ACE feed: slot {slot_index}, "
            f"length={feed_length}mm @ {self.feed_speed}mm/min"
        )

        # Phase 1: Bulk feed (skip the last sensor_approach_margin mm)
        bulk_distance = max(0, feed_length - self.sensor_approach_margin)
        if bulk_distance > 0:
            ace.feed_filament(slot_index, bulk_distance, self.feed_speed)
            # Wait for ACE to physically complete the bulk feed movement
            # (ACE ACKs the command before the motor finishes)
            sensor_triggered_early = self._wait_for_feed_complete(
                slot_index, bulk_distance, self.feed_speed, lane=lane
            )
        else:
            sensor_triggered_early = False

        # Phase 2: Sensor approach - feed in increments, checking sensor
        total_fed = bulk_distance
        sensor_triggered = sensor_triggered_early

        if lane is not None:
            if not sensor_triggered:
                # Check if sensor triggered after bulk feed settled
                self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.2)
                if lane.get_toolhead_pre_sensor_state():
                    sensor_triggered = True
                    self.logger.info(
                        f"ACE feed: toolhead sensor triggered during bulk feed "
                        f"at ~{total_fed:.0f}mm"
                    )

            # Incremental feed until sensor triggers or max distance reached
            max_total = feed_length + self.max_feed_overshoot
            while not sensor_triggered and total_fed < max_total:
                step = min(self.sensor_step, max_total - total_fed)
                ace.feed_filament(slot_index, step, self.feed_speed)
                # Wait for this small step to physically complete
                sensor_hit = self._wait_for_feed_complete(
                    slot_index, step, self.feed_speed, lane=lane,
                    poll_interval=0.3
                )
                total_fed += step

                if sensor_hit or lane.get_toolhead_pre_sensor_state():
                    sensor_triggered = True
                    self.logger.info(
                        f"ACE feed: toolhead sensor triggered at {total_fed:.0f}mm"
                    )
        else:
            # No lane/sensor - just feed the remaining fixed distance
            remaining = feed_length - bulk_distance
            if remaining > 0:
                ace.feed_filament(slot_index, remaining, self.feed_speed)
                self._wait_for_feed_complete(
                    slot_index, remaining, self.feed_speed
                )
                total_fed = feed_length

        # Phase 3: Feed assist + extruder assist for the last stretch
        # Feed assist stays enabled after loading to maintain filament tension
        # during printing. It is stopped in _unload_sequence_inner before retraction.
        if self._get_feed_assist(slot_index, lane):
            try:
                self._wait_for_ace_ready()
                ace.start_feed_assist(slot_index)
                self._feed_assist_active.add(slot_index)
            except Exception as e:
                self.logger.warning(
                    f"ACE feed: start_feed_assist failed for slot "
                    f"{slot_index}: {e}"
                )

            # Use extruder motor to pull filament into hotend
            if self.extruder_assist_length > 0:
                self.afc.gcode.run_script_from_command(
                    f"G92 E0\n"
                    f"G1 E{self.extruder_assist_length} F{self.extruder_assist_speed}"
                )

        return total_fed, sensor_triggered

    def _wait_for_feed_complete(self, slot_index, length_mm, speed_mm_min,
                                 lane=None, poll_interval=0.5):
        """Wait for the ACE hardware to finish a feed/unwind movement.

        The ACE acknowledges feed_filament/unwind_filament commands immediately
        via JSON-RPC, but the physical motor movement continues asynchronously.
        This method polls get_status until the slot is no longer busy, or until
        the calculated maximum movement time expires.

        If a lane with a toolhead sensor is provided, also checks the sensor
        each poll iteration and returns early if triggered.

        Returns True if the sensor triggered during the wait, False otherwise.
        """
        ace = self._ace
        if ace is None or not ace.connected:
            return False

        # Calculate max wait: movement time + generous buffer
        max_wait = (length_mm / max(speed_mm_min, 1)) * 60 + 5.0
        deadline = self.afc.reactor.monotonic() + max_wait
        sensor_triggered = False

        # Initial delay: give the ACE time to start the motor and update
        # its slot status before we begin polling. Without this, the first
        # poll can see the slot still in "ready" state (or missing status)
        # and exit immediately before the motor has begun moving.
        self.afc.reactor.pause(self.afc.reactor.monotonic() + 1.0)

        while self.afc.reactor.monotonic() < deadline:
            self.afc.reactor.pause(
                self.afc.reactor.monotonic() + poll_interval
            )

            # Check toolhead sensor if available
            if lane is not None and lane.get_toolhead_pre_sensor_state():
                sensor_triggered = True
                self.logger.debug(
                    f"ACE wait: toolhead sensor triggered for slot {slot_index}"
                )
                # Stop the ACE feed -the motor is still running for the
                # full requested distance but we don't need any more filament.
                # Without this, the ACE stays "busy/feeding" for minutes,
                # blocking feed_assist and causing RFID reads to return empty.
                try:
                    ace.stop_feed_filament(slot_index)
                    self.logger.debug(
                        f"ACE wait: stopped feed for slot {slot_index} "
                        "(sensor triggered early)"
                    )
                    # Brief pause for ACE to transition out of "feeding" state
                    self.afc.reactor.pause(
                        self.afc.reactor.monotonic() + 0.5
                    )
                except Exception:
                    pass
                return True

            # Poll ACE status to check if movement is done
            try:
                hw_status = ace.get_status(timeout=2.0)
                if isinstance(hw_status, dict):
                    slots = hw_status.get("slots", [])
                    if slot_index < len(slots):
                        slot_data = slots[slot_index]
                        if isinstance(slot_data, dict):
                            status = slot_data.get("status", "")
                            # If slot is back to "ready" or "empty",
                            # movement is done. Empty/missing status is
                            # NOT treated as complete -it likely means the
                            # ACE hasn't updated yet.
                            if status in ("ready", "empty"):
                                self.logger.debug(
                                    f"ACE wait: slot {slot_index} movement "
                                    f"complete (status={status})"
                                )
                                return sensor_triggered
            except Exception:
                # Status poll failed, just keep waiting
                pass

        self.logger.debug(
            f"ACE wait: timeout waiting for slot {slot_index} "
            f"movement ({max_wait:.1f}s)"
        )
        return sensor_triggered

    def _feed_until_sensor(self, slot_index, lane, max_length, step_size=None):
        """Feed filament incrementally until the toolhead sensor triggers.

        Used for calibration. Feeds in small steps, checking the sensor
        between each step. Returns the total distance fed.

        Returns (distance_fed, sensor_triggered).
        """
        if step_size is None:
            step_size = self.calibration_step

        ace = self._ace
        total_fed = 0.0

        while total_fed < max_length:
            step = min(step_size, max_length - total_fed)
            # Ensure ACE is ready before each step � after the previous
            # step completes, the ACE overall status may briefly stay
            # "busy" for internal housekeeping.  Sending feed_filament
            # while busy returns FORBIDDEN and kills the calibration.
            self._wait_for_ace_ready()
            try:
                ace.feed_filament(slot_index, step, self.feed_speed)
            except Exception as e:
                self.logger.warning(
                    f"ACE calibration: feed_filament failed at "
                    f"{total_fed:.0f}mm, retrying: {e}"
                )
                self._wait_for_ace_ready(timeout=15.0)
                ace.feed_filament(slot_index, step, self.feed_speed)
            # Wait for ACE to physically complete this step
            sensor_hit = self._wait_for_feed_complete(
                slot_index, step, self.feed_speed, lane=lane,
                poll_interval=0.3
            )
            total_fed += step

            if sensor_hit or lane.get_toolhead_pre_sensor_state():
                self.logger.info(
                    f"ACE calibration: sensor triggered at {total_fed:.1f}mm"
                )
                return total_fed, True

        return total_fed, False

    def _retract_slot(self, slot_index, lane=None):
        """Retract filament from the toolhead back into the ACE slot."""
        ace = self._ace
        retract_length = self._get_retract_length(lane)

        # Ensure ACE is not still busy from a previous operation
        self._wait_for_ace_ready()

        self.logger.debug(
            f"ACE retract: slot {slot_index}, "
            f"length={retract_length}mm @ {self.retract_speed}mm/min"
        )
        ace.unwind_filament(slot_index, retract_length, self.retract_speed)
        # Wait for ACE to physically complete the retraction
        self._wait_for_feed_complete(
            slot_index, retract_length, self.retract_speed
        )

    # ---- No-Op / Unsupported Operations ----

    def prep_load(self, lane):
        """No-op: ACE hardware manages filament to sensors directly."""
        pass

    def prep_post_load(self, lane):
        """No-op: ACE hardware handles loading internally."""
        pass

    def eject_lane(self, lane):
        """ACE units don't support stepper-based lane ejection."""
        lane_name = getattr(lane, "name", "unknown")
        message = (
            f"LANE_UNLOAD is not supported for ACE lane {lane_name}. "
            "ACE units handle filament automatically - just remove the spool physically. "
            "Use TOOL_UNLOAD if you need to unload from the toolhead."
        )
        self.logger.info(message)
        try:
            self.gcode.respond_info(message)
        except Exception:
            pass

    def lane_unload(self, cur_lane):
        """Block manual LANE_UNLOAD for ACE lanes."""
        self.eject_lane(cur_lane)
        return None

    def get_lane_reset_command(self, lane, dis) -> str:
        """ACE lanes retract via ACE hardware, bypassing TOOL_UNLOAD."""
        return f"ACE_LANE_RESET UNIT={self.name} LANE={lane.name}"

    # ---- System Test / PREP ----

    def system_Test(self, cur_lane, delay, assignTcmd, enable_movement):
        """Validate ACE lane state during PREP without attempting motion."""
        msg = ""
        succeeded = True

        cur_lane.unsync_to_extruder(False)
        self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.7)

        # Query slot status directly from hardware (cache may not be populated yet)
        if self._ace is not None and self._ace.connected:
            local_slot = self._get_local_slot_for_lane(cur_lane)
            if 0 <= local_slot < self.SLOTS_PER_UNIT:
                try:
                    hw_status = self._ace.get_status(timeout=2.0)
                    if isinstance(hw_status, dict):
                        slots = hw_status.get("slots", [])
                        for i, slot_data in enumerate(slots):
                            if i < self.SLOTS_PER_UNIT and isinstance(slot_data, dict):
                                self._slot_inventory[i]["status"] = slot_data.get("status", "")
                except Exception as e:
                    self.logger.debug(f"ACE {self.name}: get_status failed during PREP: {e}")

                slot_info = self._slot_inventory[local_slot]
                slot_ready = slot_info.get("status", "") == "ready"
                cur_lane._load_state = slot_ready
                cur_lane.prep_state = slot_ready

                # ACE filament is only "at the hub" when actively fed to
                # toolhead.  Clear any stale persisted value — the
                # tool_loaded check below will restore it if needed.
                cur_lane.loaded_to_hub = False

                # Apply filament defaults if lane doesn't have values set
                self._apply_slot_filament_defaults(cur_lane, slot_info)

        if not cur_lane.prep_state:
            if not cur_lane.load_state:
                self.lane_not_ready(cur_lane)
                msg += '<span class=success--text>EMPTY READY FOR SPOOL</span>'
            else:
                self.lane_fault(cur_lane)
                msg += '<span class=error--text>CHECK FILAMENT Prep: False - Load: True</span>'
                cur_lane.do_enable(False)
                succeeded = False
        else:
            self.lane_loaded(cur_lane)
            msg += '<span class=success--text>LOCKED</span>'
            if not cur_lane.load_state:
                msg += '<span class=error--text> NOT LOADED</span>'
                self.lane_not_ready(cur_lane)
                succeeded = False
            else:
                cur_lane.status = AFCLaneState.LOADED
                msg += '<span class=success--text> AND LOADED</span>'
                self.lane_illuminate_spool(cur_lane)

                if cur_lane.tool_loaded:
                    # Filament is in the toolhead, so it's also in the hub path
                    cur_lane.loaded_to_hub = True
                    # For ACE with AMS virtual pin, the FPS reads zero
                    # at startup (motor off) so tool_start_state is False.
                    # If saved state confirms this lane is loaded, set the
                    # virtual sensor directly (like OpenAMS does for its
                    # lanes) so the toolhead-pre-sensor check passes the
                    # same way a physical switch would on BoxTurtle.
                    extruder_obj = cur_lane.extruder_obj
                    if extruder_obj.lane_loaded == cur_lane.name:
                        extruder_obj.tool_start_state = True
                        fila = getattr(extruder_obj, "fila_tool_start", None)
                        helper = getattr(fila, "runout_helper", None) if fila else None
                        if helper is not None:
                            eventtime = self.afc.reactor.monotonic()
                            try:
                                helper.note_filament_present(eventtime, True)
                            except TypeError:
                                helper.note_filament_present(True)

                    tool_ready = (
                        cur_lane.get_toolhead_pre_sensor_state()
                        or extruder_obj.tool_start == "buffer"
                        or extruder_obj.tool_end_state
                    )
                    if tool_ready and extruder_obj.lane_loaded == cur_lane.name:
                        cur_lane.sync_to_extruder()
                        on_shuttle = ""
                        if extruder_obj.tool_obj and extruder_obj.tc_unit_name:
                            on_shuttle = " and toolhead on shuttle" if extruder_obj.on_shuttle() else ""
                        msg += f'<span class=primary--text> in ToolHead{on_shuttle}</span>'
                        if cur_lane.extruder_obj.tool_start == "buffer":
                            msg += '<span class=warning--text> Ram sensor enabled, confirm tool is loaded</span>'

                        # Restore combined mode tracking regardless of shuttle
                        # state so ACE knows which slot is loaded for the next
                        # tool change even when the shuttle starts empty.
                        if self.mode == MODE_COMBINED:
                            local_slot = self._get_local_slot_for_lane(cur_lane)
                            if local_slot >= 0:
                                self._current_loaded_slot = local_slot

                        if self.afc.function.get_current_lane() == cur_lane.name:
                            self.afc.spool.set_active_spool(cur_lane.spool_id)
                            cur_lane.unit_obj.lane_tool_loaded(cur_lane)
                            cur_lane.status = AFCLaneState.TOOLED

                            # Start feed assist so ACE pushes filament
                            # immediately — don't wait for the periodic
                            # refresh (~15 s) which could starve the
                            # extruder if a print resumes right away.
                            fa_slot = self._get_local_slot_for_lane(cur_lane)
                            if (fa_slot >= 0
                                    and self._get_feed_assist_for_slot(fa_slot)
                                    and self._ace is not None):
                                try:
                                    self._ace.start_feed_assist(fa_slot)
                                    self._feed_assist_active.add(fa_slot)
                                    self.logger.info(
                                        f"PREP: feed assist started for "
                                        f"slot {fa_slot} ({cur_lane.name})"
                                    )
                                except Exception as e:
                                    self.logger.debug(
                                        f"PREP: feed assist start failed "
                                        f"for slot {fa_slot}: {e}"
                                    )
                        else:
                            cur_lane.unit_obj.lane_tool_loaded_idle(cur_lane)
                    elif tool_ready:
                        msg += (
                            '<span class=error--text> error in ToolHead. '
                            'Lane identified as loaded but not identified as loaded in extruder</span>'
                        )
                        succeeded = False

        if assignTcmd:
            self.afc.function.TcmdAssign(cur_lane)
        cur_lane.do_enable(False)
        self.logger.info(
            '{lane_name} tool cmd: {tcmd:3} {msg}'.format(
                lane_name=cur_lane.name, tcmd=cur_lane.map, msg=msg
            )
        )
        cur_lane.set_afc_prep_done()
        return succeeded

    # ---- Calibration ----

    def calibrate_bowden(self, cur_lane, dis, tol):
        """Calibrate bowden length by feeding until toolhead sensor triggers."""
        self._operation_active = True
        self._fps_latched = False
        try:
            return self._calibrate_bowden_inner(cur_lane, dis, tol)
        finally:
            self._operation_active = False
            self._fps_latched = False

    def _calibrate_bowden_inner(self, cur_lane, dis, tol):
        if self._ace is None or not self._ace.connected:
            return False, "ACE not connected", 0

        local_slot = self._get_local_slot_for_lane(cur_lane)
        if local_slot < 0:
            return False, f"Cannot determine slot for {cur_lane.name}", 0

        # Don't calibrate if sensor is already triggered
        if cur_lane.get_toolhead_pre_sensor_state():
            return False, "Toolhead sensor already triggered - unload first", 0

        # ACE always needs the full range -the 'dis' parameter from
        # CALIBRATE_AFC DISTANCE=N is designed for stepper-based units and
        # is far too small (default 25mm) for bowden-length calibration.
        max_distance = 6000

        self.logger.info(
            f"ACE calibrate_bowden: feeding slot {local_slot} "
            f"in {self.calibration_step}mm steps, max {max_distance}mm"
        )

        distance, triggered = self._feed_until_sensor(
            local_slot, cur_lane, max_distance, step_size=self.calibration_step
        )

        # Retract after calibration - retract 200mm less than fed distance
        # to avoid pulling filament out of the ACE unit during rewind
        # (matches how retract_length is stored as feed_length - 200)
        retract_dist = distance - 200 if triggered else distance
        self.logger.info(
            f"ACE calibrate_bowden: retracting {retract_dist:.0f}mm "
            f"@ {self.retract_speed}mm/min"
        )
        try:
            self._wait_for_ace_ready()
            self._ace.unwind_filament(local_slot, retract_dist, self.retract_speed)
            self._wait_for_feed_complete(
                local_slot, retract_dist, self.retract_speed
            )
            self.logger.info("ACE calibrate_bowden: retract complete")
        except Exception as e:
            self.logger.error(f"ACE calibrate_bowden: retract failed: {e}")

        if not triggered:
            msg = (
                f"Toolhead sensor did not trigger after {distance:.0f}mm. "
                "Check filament path and sensor wiring."
            )
            return False, msg, distance

        # Round to nearest integer for clean config values
        new_feed_length = round(distance, 0)
        new_retract_length = round(distance - 200, 0)

        # Update in-memory values
        old_feed = self.feed_length
        old_retract = self.retract_length
        self.feed_length = new_feed_length
        self.retract_length = new_retract_length

        # Write calibrated values back to config file
        unit_section = " ".join(self.full_name)
        cal_msg = f"\n feed_length: New: {new_feed_length} Old: {old_feed}"
        self.afc.function.ConfigRewrite(
            unit_section, "feed_length", new_feed_length, cal_msg
        )
        cal_msg = f"\n retract_length: New: {new_retract_length} Old: {old_retract}"
        self.afc.function.ConfigRewrite(
            unit_section, "retract_length", new_retract_length, cal_msg
        )
        self.afc.save_vars()

        msg = (
            f"ACE bowden calibration: toolhead sensor triggered at {distance:.1f}mm.\n"
            f"feed_length: {new_feed_length:.0f} (was {old_feed:.0f})\n"
            f"retract_length: {new_retract_length:.0f} (was {old_retract:.0f})\n"
            f"Values saved to config."
        )
        return True, msg, distance

    def calibrate_hub(self, cur_lane, tol):
        """Hub calibration not applicable - ACE manages hub state internally."""
        return False, "ACE hub is managed by ACE hardware. No calibration needed.", 0

    def calibrate_lane(self, cur_lane, tol):
        """Lane calibration: alias for bowden calibration on ACE units.

        On stepper units, calibrate_lane measures extruder-to-hub distance.
        On ACE, there's only one distance that matters: ACE slot to toolhead.
        This delegates to calibrate_bowden for the same measurement.
        """
        return self.calibrate_bowden(cur_lane, 0, tol)

    def calibrate_td1(self, cur_lane, dis, tol):
        """Calibrate TD-1 bowden length by feeding until TD-1 device detects filament."""
        self._operation_active = True
        self._fps_latched = False
        try:
            return self._calibrate_td1_inner(cur_lane, dis, tol)
        finally:
            self._operation_active = False
            self._fps_latched = False

    def _calibrate_td1_inner(self, cur_lane, dis, tol):
        if self._ace is None or not self._ace.connected:
            return False, "ACE not connected", 0

        # Validate TD-1 device ID
        if cur_lane.td1_device_id is None:
            msg = (
                f"Cannot calibrate TD-1 for {cur_lane.name}, td1_device_id is a required "
                "field in AFC_hub or per AFC_lane"
            )
            return False, msg, 0

        valid, msg = self.afc.function.check_for_td1_id(cur_lane.td1_device_id)
        if not valid:
            msg = (
                f"TD-1 device(SN: {cur_lane.td1_device_id}) not detected anymore, "
                "please check before continuing to calibrate TD-1 bowden length"
            )
            return False, msg, 0

        local_slot = self._get_local_slot_for_lane(cur_lane)
        if local_slot < 0:
            return False, f"Cannot determine slot for {cur_lane.name}", 0

        # Use calibration_step if dis is not specified
        step_size = dis if dis > 0 else self.calibration_step
        max_bowden_length = 6000

        self.logger.info(
            f"ACE calibrate_td1: feeding slot {local_slot} in {step_size}mm steps, "
            f"max {max_bowden_length}mm, TD-1 device={cur_lane.td1_device_id}"
        )

        # Ensure ACE is not still busy from a previous operation
        self._wait_for_ace_ready()

        # Feed incrementally until TD-1 detects filament
        bow_pos = 0.0
        compare_time = datetime.now()
        while not self.get_td1_data(cur_lane, compare_time):
            if bow_pos > max_bowden_length:
                # Retract what we fed
                self._retract_slot(local_slot, lane=cur_lane)
                msg = f"TD-1 failed to detect filament after moving {bow_pos:.0f}mm"
                return False, msg, bow_pos

            compare_time = datetime.now()
            bow_pos += step_size
            self._ace.feed_filament(local_slot, step_size, self.feed_speed)

            # Brief pause for TD-1 to register
            self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.2)

        self.logger.info(
            f"ACE calibrate_td1: TD-1 detected filament at {bow_pos:.1f}mm"
        )

        # Retract back to ACE unit using measured distance
        retract_dist = bow_pos + 50
        self.logger.info(
            f"ACE calibrate_td1: retracting {retract_dist:.0f}mm "
            f"@ {self.retract_speed}mm/min"
        )
        try:
            self._wait_for_ace_ready()
            self._ace.unwind_filament(local_slot, retract_dist, self.retract_speed)
            self._wait_for_feed_complete(
                local_slot, retract_dist, self.retract_speed
            )
            self.logger.info("ACE calibrate_td1: retract complete")
        except Exception as e:
            self.logger.error(f"ACE calibrate_td1: retract failed: {e}")

        # Save td1_bowden_length to the lane's config section
        old_td1 = getattr(cur_lane, "td1_bowden_length", None)
        cur_lane.td1_bowden_length = bow_pos
        fullname = cur_lane.fullname
        cal_msg = f"\n td1_bowden_length: New: {bow_pos} Old: {old_td1}"
        self.afc.function.ConfigRewrite(
            fullname, "td1_bowden_length", bow_pos, cal_msg
        )
        self.afc.save_vars()

        msg = (
            f"ACE TD-1 calibration: filament detected at {bow_pos:.1f}mm.\n"
            f"td1_bowden_length: {bow_pos:.0f} (was {old_td1})\n"
            f"Value saved to config."
        )
        return True, msg, bow_pos

    # ---- Runout Detection ----

    def check_runout(self, cur_lane):
        """ACE supports runout detection during printing."""
        try:
            return self.afc.function.is_printing()
        except Exception:
            return False

    def _start_slot_status_monitor(self):
        """Start periodic polling for slot status changes (runout detection)."""
        self._prev_slot_states = {}
        # Track when we last did a full inventory sync (RFID data)
        self._last_inventory_sync = 0.0
        self._inventory_sync_interval = 30.0  # Full RFID sync every 30s
        self._slot_monitor_timer = self.afc.reactor.register_timer(
            self._poll_slot_status,
            self.afc.reactor.monotonic() + 5.0,
        )
        self.logger.info(
            f"ACE {self.name}: started slot status monitor "
            f"(interval={self.poll_interval}s)"
        )

    def _poll_slot_status(self, eventtime):
        """Periodic callback to detect slot status changes during printing.

        Uses a single get_status call per poll to check slot states.
        Full inventory sync (4x get_filament_info) only runs infrequently.
        """
        if self._ace is None or not self._ace.connected:
            return eventtime + self.poll_interval * 4

        # Skip during active operations (load/unload/calibration)
        if self._operation_active:
            return eventtime + self.poll_interval

        is_printing = False
        try:
            is_printing = self.afc.function.is_printing()
        except Exception:
            pass

        # Single get_status call - refreshes cached hw status AND slot states
        try:
            hw_status = self._ace.get_status(timeout=2.0)
            if isinstance(hw_status, dict):
                self._cached_hw_status = hw_status
                # Parse slot states from get_status response
                slots = hw_status.get("slots", [])
                for i, slot_data in enumerate(slots):
                    if i < self.SLOTS_PER_UNIT and isinstance(slot_data, dict):
                        status = slot_data.get("status", "")
                        # Update inventory cache with status from get_status
                        self._slot_inventory[i]["status"] = status
        except Exception:
            pass

        # Full RFID inventory sync only infrequently (material, color data)
        if eventtime - self._last_inventory_sync > self._inventory_sync_interval:
            self._last_inventory_sync = eventtime
            try:
                self._sync_inventory()
            except Exception:
                pass

        # Sync loaded states and detect runout
        for lane in self.lanes.values():
            local_slot = self._get_local_slot_for_lane(lane)
            if local_slot < 0 or local_slot >= self.SLOTS_PER_UNIT:
                continue

            slot_info = self._slot_inventory[local_slot]
            slot_status = slot_info.get("status", "") if slot_info else ""
            slot_ready = slot_status == "ready"

            # "shifting" is a transient state during feed_assist start -
            # don't treat it as not-ready or it will false-trigger runout.
            slot_shifting = slot_status == "shifting"

            # Always sync load/prep state so status is accurate
            lane._load_state = slot_ready
            lane.prep_state = slot_ready

            prev_ready = self._prev_slot_states.get(lane.name)
            # Don't update prev state during transient "shifting" - wait
            # for a definitive ready/empty to avoid false runout triggers.
            if not slot_shifting:
                self._prev_slot_states[lane.name] = slot_ready

            # State consistency: if hardware says ready but lane is stuck
            # in NONE, fix it (covers first poll, missed transitions, etc.)
            if slot_ready and lane._afc_prep_done and lane.status == AFCLaneState.NONE:
                # Check if this lane was tool-loaded before restart -
                # restore TOOLED state instead of just LOADED.
                if (lane.tool_loaded
                        and hasattr(lane, 'extruder_obj')
                        and lane.extruder_obj.lane_loaded == lane.name):
                    self.logger.info(
                        f"ACE poll: {lane.name} slot {local_slot} ready, "
                        f"restoring TOOLED state from saved vars"
                    )
                    self._restore_tool_loaded_state(lane)
                else:
                    self.logger.info(
                        f"ACE poll: {lane.name} slot {local_slot} ready, "
                        f"setting loaded"
                    )
                    # New filament insertion: clear old data, apply fresh.
                    self.afc.spool.clear_values(lane)
                    lane.set_loaded()
                    self._refresh_slot_inventory(local_slot)
                    slot_info = self._slot_inventory[local_slot]
                    self._apply_slot_filament_defaults(lane, slot_info)
                    self.lane_illuminate_spool(lane)
                    self.afc.save_vars()

            # Detect ready -> not-ready transition (filament runout)
            # Skip runout detection while dryer is running - drying can
            # cause transient slot state changes that aren't real runouts.
            elif not self._drying_active and prev_ready and not slot_ready and not slot_shifting:
                if is_printing and lane.status == AFCLaneState.TOOLED:
                    self.logger.info(
                        f"ACE runout detected on {lane.name} (slot {local_slot})"
                    )
                    lane.loaded_to_hub = False

                    if lane.runout_lane:
                        try:
                            lane._perform_infinite_runout()
                        except Exception as e:
                            self.logger.error(
                                f"ACE infinite spool failed for {lane.name}: "
                                f"{e}\n{traceback.format_exc()}"
                            )
                            lane._perform_pause_runout()
                        finally:
                            lane.loaded_to_hub = False
                    else:
                        lane._perform_pause_runout()
                elif lane.status == AFCLaneState.LOADED:
                    # Slot went empty - transition to unloaded so new
                    # filament insertion is properly detected.
                    self._clear_slot_inventory(local_slot)
                    lane.set_unloaded()
                    self.lane_not_ready(lane)
                    self.afc.save_vars()

        # Polling rates: 2s when printing (runout detection), 5s when idle
        if is_printing:
            return eventtime + max(self.poll_interval, 2.0)
        return eventtime + 5.0

    # ---- Inventory Sync ----

    def sync_ace_inventory(self):
        """Public method to force-sync RFID/spool data from ACE hardware."""
        self._sync_inventory()
        self._sync_slot_loaded_state()

    # ---- AFC Function Wrappers ----

    def _wrap_get_current_lane(self):
        """Wrap AFC's get_current_lane to return the loaded lane even when
        the toolchanger shuttle is empty.

        Upstream's get_current_lane() gates on on_shuttle(), which returns
        False when no tool is detected (e.g. after a power cycle with the
        shuttle parked).  This causes CHANGE_TOOL to skip the unload step
        because self.current returns None, leaving stale filament in the
        hub/bowden path.

        The wrapper falls back to extruder.lane_loaded for ACE lanes so
        that CHANGE_TOOL properly triggers TOOL_UNLOAD (which does its own
        tool_swap to pick up the shuttle before unloading).
        """
        afc_function = getattr(self.afc, "function", None)
        if afc_function is None:
            return

        # Only wrap once across all ACE instances
        if hasattr(afc_function, "_ace_get_current_lane_original"):
            return

        afc_function._ace_get_current_lane_original = afc_function.get_current_lane

        def get_current_lane_wrapper():
            result = afc_function._ace_get_current_lane_original()
            if result is not None:
                return result

            # Fallback: check if the active Klipper extruder has an ACE
            # lane loaded according to saved state.
            if self.printer.state_message != 'Printer is ready':
                return None
            current_extruder_name = self.afc.toolhead.get_extruder().name
            tool_obj = self.afc.tools.get(current_extruder_name)
            if tool_obj is None or tool_obj.lane_loaded is None:
                return None
            lane_obj = self.afc.lanes.get(tool_obj.lane_loaded)
            if lane_obj is not None and isinstance(lane_obj.unit_obj, afcACE):
                return tool_obj.lane_loaded
            return None

        afc_function.get_current_lane = get_current_lane_wrapper
        self.logger.debug("Wrapped AFC.function.get_current_lane for ACE shuttle-empty fallback")

    # ---- GCode Commands ----

    def _register_gcode_commands(self):
        """Register ACE-specific GCode commands."""
        self.gcode.register_mux_command(
            "ACE_STATUS", "UNIT", self.name,
            self.cmd_ACE_STATUS,
            desc="Query ACE hardware status",
        )
        self.gcode.register_mux_command(
            "ACE_DRY", "UNIT", self.name,
            self.cmd_ACE_DRY,
            desc="Start ACE filament dryer",
        )
        self.gcode.register_mux_command(
            "ACE_DRY_STOP", "UNIT", self.name,
            self.cmd_ACE_DRY_STOP,
            desc="Stop ACE filament dryer",
        )
        self.gcode.register_mux_command(
            "ACE_FEED_ASSIST", "UNIT", self.name,
            self.cmd_ACE_FEED_ASSIST,
            desc="Enable/disable feed assist for ACE unit",
        )
        self.gcode.register_mux_command(
            "ACE_SYNC_INVENTORY", "UNIT", self.name,
            self.cmd_ACE_SYNC_INVENTORY,
            desc="Refresh RFID/spool inventory from ACE hardware",
        )
        self.gcode.register_mux_command(
            "ACE_CALIBRATE", "UNIT", self.name,
            self.cmd_ACE_CALIBRATE,
            desc="Calibrate ACE bowden length by feeding until toolhead sensor triggers",
        )
        self.gcode.register_mux_command(
            "ACE_LANE_RESET", "UNIT", self.name,
            self.cmd_ACE_LANE_RESET,
            desc="Retract ACE lane filament back into the unit",
        )

    def cmd_ACE_STATUS(self, gcmd):
        """Query and display ACE hardware status.

        Usage: ACE_STATUS UNIT=<name>
        """
        if self._ace is None or not self._ace.connected:
            gcmd.respond_info(f"ACE {self.name}: not connected")
            return

        try:
            status = self._ace.get_status()
            gcmd.respond_info(f"ACE {self.name} status: {status}")
        except Exception as e:
            gcmd.respond_info(f"ACE {self.name} status query failed: {e}")

    def cmd_ACE_DRY(self, gcmd):
        """Start ACE filament dryer.

        Usage: ACE_DRY UNIT=<name> TEMP=<celsius> DURATION=<minutes> [FAN=<rpm>]
        """
        if self._ace is None or not self._ace.connected:
            gcmd.respond_info(f"ACE {self.name}: not connected")
            return

        temp = gcmd.get_int("TEMP", 50)
        duration = gcmd.get_int("DURATION", 240)
        fan = gcmd.get_int("FAN", 800)

        try:
            self._ace.start_drying(temp, fan, duration)
            self._drying_active = True
            gcmd.respond_info(
                f"ACE {self.name}: drying started "
                f"({temp}C, {duration}min, fan={fan}rpm)"
            )
        except Exception as e:
            gcmd.respond_info(f"ACE {self.name}: drying failed: {e}")

    def cmd_ACE_DRY_STOP(self, gcmd):
        """Stop ACE filament dryer.

        Usage: ACE_DRY_STOP UNIT=<name>
        """
        if self._ace is None or not self._ace.connected:
            gcmd.respond_info(f"ACE {self.name}: not connected")
            return

        try:
            self._ace.stop_drying()
            self._drying_active = False
            gcmd.respond_info(f"ACE {self.name}: drying stopped")
        except Exception as e:
            self._drying_active = False
            gcmd.respond_info(f"ACE {self.name}: stop drying failed: {e}")

    def cmd_ACE_FEED_ASSIST(self, gcmd):
        """Enable or disable feed assist, globally or per-slot.

        Usage:
            ACE_FEED_ASSIST UNIT=<name>                    # query all slots
            ACE_FEED_ASSIST UNIT=<name> ENABLE=<0|1>       # set default for all
            ACE_FEED_ASSIST UNIT=<name> SLOT=<1-4> ENABLE=<0|1>  # set per-slot
            ACE_FEED_ASSIST UNIT=<name> SLOT=<1-4> ENABLE=default # clear per-slot override
        """
        slot_num = gcmd.get_int("SLOT", default=None)
        enable_str = gcmd.get("ENABLE", default=None)

        # Query mode: no ENABLE param
        if enable_str is None:
            lines = [f"ACE {self.name} feed assist:"]
            default_str = "enabled" if self._default_feed_assist else "disabled"
            lines.append(f"  Default: {default_str}")
            for s in range(self.SLOTS_PER_UNIT):
                override = self._slot_feed_assist.get(s)
                effective = self._get_feed_assist_for_slot(s)
                eff_str = "enabled" if effective else "disabled"
                if override is not None:
                    lines.append(f"  Slot {s + 1}: {eff_str} (override)")
                else:
                    lines.append(f"  Slot {s + 1}: {eff_str} (default)")
            gcmd.respond_info("\n".join(lines))
            return

        # Per-slot override
        if slot_num is not None:
            slot_index = slot_num - 1  # Config is 1-based
            if slot_index < 0 or slot_index >= self.SLOTS_PER_UNIT:
                gcmd.respond_info(
                    f"ACE {self.name}: invalid slot {slot_num} (must be 1-{self.SLOTS_PER_UNIT})"
                )
                return

            if enable_str.lower() == "default":
                # Clear per-slot override
                self._slot_feed_assist.pop(slot_index, None)
                effective = self._get_feed_assist_for_slot(slot_index)
                state = "enabled" if effective else "disabled"
                gcmd.respond_info(
                    f"ACE {self.name}: slot {slot_num} feed assist reset to default ({state})"
                )
            else:
                enable = bool(int(enable_str))
                self._slot_feed_assist[slot_index] = enable
                state = "enabled" if enable else "disabled"
                gcmd.respond_info(
                    f"ACE {self.name}: slot {slot_num} feed assist {state}"
                )
            return

        # Global default
        enable = bool(int(enable_str))
        self._default_feed_assist = enable
        state = "enabled" if enable else "disabled"
        gcmd.respond_info(f"ACE {self.name}: feed assist default {state}")

    def cmd_ACE_SYNC_INVENTORY(self, gcmd):
        """Refresh RFID/spool inventory from ACE hardware and sync to lanes.

        Usage: ACE_SYNC_INVENTORY UNIT=<name>
        """
        if self._ace is None or not self._ace.connected:
            gcmd.respond_info(f"ACE {self.name}: not connected")
            return

        try:
            # Re-enable RFID in case it was disabled
            self._ace.enable_rfid()
            # Pull fresh inventory
            self._sync_inventory()
            self._sync_slot_loaded_state()

            # Report what we found
            lines = [f"ACE {self.name} inventory:"]
            for slot in range(self.SLOTS_PER_UNIT):
                info = self._slot_inventory[slot]
                status = info.get("status", "unknown")
                material = info.get("material", "")
                color = info.get("color", [0, 0, 0])
                hex_color = self._ace_color_to_hex(color)
                lines.append(
                    f"  Slot {slot + 1}: {status} | {material} | {hex_color}"
                )
            gcmd.respond_info("\n".join(lines))
        except Exception as e:
            gcmd.respond_info(
                f"ACE {self.name}: inventory sync failed: {e}"
            )

    def cmd_ACE_LANE_RESET(self, gcmd):
        """Retract ACE lane filament back into the unit.

        Unlike TOOL_UNLOAD this skips homing, z-hop, tip forming and other
        toolhead operations.  It simply tells the ACE hardware to retract
        the slot and updates lane state.

        Usage: ACE_LANE_RESET UNIT=<unit> LANE=<lane>
        """
        lane_name = gcmd.get("LANE", None)
        if lane_name is None or lane_name not in self.afc.lanes:
            gcmd.respond_info(f"ACE_LANE_RESET: invalid lane '{lane_name}'")
            return

        cur_lane = self.afc.lanes[lane_name]
        if cur_lane.unit_obj is not self:
            gcmd.respond_info(
                f"ACE_LANE_RESET: {lane_name} does not belong to {self.name}"
            )
            return

        local_slot = self._get_local_slot_for_lane(cur_lane)
        if local_slot < 0 or local_slot >= self.SLOTS_PER_UNIT:
            gcmd.respond_info(
                f"ACE_LANE_RESET: {lane_name} has no valid slot mapping"
            )
            return

        if self._ace is None or not self._ace.connected:
            gcmd.respond_info(
                f"ACE_LANE_RESET: ACE {self.name} not connected"
            )
            return

        self.logger.info(f"ACE lane reset: retracting {lane_name} slot {local_slot}")

        # Stop feed assist if running
        try:
            self._wait_for_ace_ready()
            self._ace.stop_feed_assist(local_slot)
            self._feed_assist_active.discard(local_slot)
        except Exception as e:
            self.logger.warning(
                f"ACE lane reset: failed to stop feed assist: {e}"
            )

        # Retract filament back into the ACE
        try:
            self._retract_slot(local_slot, lane=cur_lane)
        except Exception as e:
            self.logger.error(f"ACE lane reset failed for {lane_name}: {e}")
            gcmd.respond_info(f"ACE_LANE_RESET: retract failed for {lane_name}: {e}")
            return

        # Update lane state
        if self.mode == MODE_COMBINED:
            self._current_loaded_slot = -1
        cur_lane.loaded_to_hub = False
        cur_lane.set_tool_unloaded()
        cur_lane.status = AFCLaneState.LOADED
        self.lane_tool_unloaded(cur_lane)

        # Clear extruder's lane_loaded if this lane was loaded
        cur_extruder = cur_lane.extruder_obj
        if cur_extruder is not None and cur_extruder.lane_loaded == lane_name:
            cur_extruder.lane_loaded = None

        self.afc.save_vars()
        self.afc.save_vars()
        gcmd.respond_info(f"ACE lane reset: {lane_name} retracted successfully")

    def cmd_ACE_CALIBRATE(self, gcmd):
        """Calibrate bowden length by feeding until toolhead sensor triggers.

        Feeds filament from the specified lane's ACE slot in small increments,
        checking the toolhead sensor between each step. Reports the measured
        distance when the sensor triggers.

        Usage: ACE_CALIBRATE UNIT=<name> LANE=<lane_name> [MAX=<mm>]
        """
        if self._ace is None or not self._ace.connected:
            gcmd.respond_info(f"ACE {self.name}: not connected")
            return

        lane_name = gcmd.get("LANE", default=None)
        if lane_name is None:
            gcmd.respond_info("ACE_CALIBRATE requires LANE=<lane_name>")
            return

        cur_lane = self.lanes.get(lane_name)
        if cur_lane is None:
            # Try looking up from AFC global lanes
            cur_lane = self.afc.lanes.get(lane_name)
        if cur_lane is None:
            gcmd.respond_info(f"Lane '{lane_name}' not found")
            return

        max_distance = gcmd.get_float("MAX", 6000)

        gcmd.respond_info(
            f"ACE {self.name}: starting bowden calibration for {lane_name}...\n"
            f"Feeding in {self.calibration_step}mm steps, max {max_distance:.0f}mm"
        )

        success, msg, distance = self.calibrate_bowden(cur_lane, max_distance, 0)
        gcmd.respond_info(msg)

    # ---- Utilities ----

    @staticmethod
    def _ace_color_to_hex(color_array):
        """Convert ACE RGB array [r, g, b] to hex color string."""
        if isinstance(color_array, (list, tuple)) and len(color_array) >= 3:
            r, g, b = int(color_array[0]), int(color_array[1]), int(color_array[2])
            return f"#{r:02x}{g:02x}{b:02x}"
        return "#000000"


def load_config_prefix(config):
    return afcACE(config)