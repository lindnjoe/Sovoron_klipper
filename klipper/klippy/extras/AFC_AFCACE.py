# Armored Turtle Automated Filament Changer
#
# AFCACE Unit Type - Direct ACE PRO hardware integration without ACEPRO/DuckACE
#
# Supports two operational modes:
#   combined: Multiple ACE slots share one toolhead path (combiner/splitter).
#             Must retract current slot before feeding new one.
#   direct:   Each ACE slot feeds its own extruder independently.
#             No global retract-before-feed constraint.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import logging
import traceback

from configparser import Error as ConfigError
from datetime import datetime
from typing import TYPE_CHECKING, Any, Dict, Optional

if TYPE_CHECKING:
    from extras.AFC_lane import AFCLane
    from configfile import ConfigWrapper

try:
    from extras.AFC_utils import ERROR_STR
except Exception:
    trace = traceback.format_exc()
    raise ConfigError(f"Error when trying to import AFC_utils.ERROR_STR\n{trace}")

try:
    from extras.AFC_unit import afcUnit
except Exception:
    raise ConfigError(ERROR_STR.format(import_lib="AFC_unit", trace=traceback.format_exc()))

try:
    from extras.AFC_lane import AFCLane, AFCLaneState
except Exception:
    raise ConfigError(ERROR_STR.format(import_lib="AFC_lane", trace=traceback.format_exc()))

try:
    from extras.AFC_respond import AFCprompt
except Exception:
    raise ConfigError(ERROR_STR.format(import_lib="AFC_respond", trace=traceback.format_exc()))

try:
    from extras.AFC_AFCACE_serial import ACEConnection, ACESerialError, ACETimeoutError
except Exception:
    raise ConfigError(ERROR_STR.format(import_lib="AFC_AFCACE_serial", trace=traceback.format_exc()))


_module_logger = logging.getLogger(__name__)

# Operational modes
MODE_COMBINED = "combined"  # Multiple slots -> one toolhead (retract before feed)
MODE_DIRECT = "direct"      # Each slot -> its own extruder (independent operation)


class afcAFCACE(afcUnit):
    """AFC unit that talks directly to Anycubic ACE PRO hardware.

    Unlike AFC_ACE.py which wraps ACEPRO/DuckACE backends, this unit owns the
    serial communication and implements both combined (shared toolhead) and
    direct (per-extruder) operational modes.

    Config example:
        [AFC_AFCACE ace1]
        serial_port: /dev/ttyACM0
        hub: ace_hub
        extruder: extruder
        mode: combined          # or "direct" for multi-extruder
        feed_speed: 800         # mm/min
        retract_speed: 800      # mm/min
        feed_length: 500        # mm - distance from ACE to toolhead
        retract_length: 500     # mm - distance to retract back to ACE

    Lane config:
        [AFC_lane lane1]
        unit: ace1:1            # Unit:Slot (1-based in config, 0-based internal)
        hub: ace_hub
        extruder: extruder
    """

    SLOTS_PER_UNIT = 4

    def __init__(self, config: ConfigWrapper):
        super().__init__(config)
        self.type = "AFCACE"
        self.logger = self.afc.logger

        # Serial port configuration
        self.serial_port = config.get("serial_port")

        # Operational mode
        mode = config.get("mode", MODE_COMBINED).lower().strip()
        if mode not in (MODE_COMBINED, MODE_DIRECT):
            raise ConfigError(
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

        # Extruder assist length: how far to advance with extruder motor
        # during feed assist (after filament reaches toolhead sensor area)
        self.extruder_assist_length = config.getfloat("extruder_assist_length", 50.0)  # mm
        self.extruder_assist_speed = config.getfloat("extruder_assist_speed", 300.0)   # mm/min

        # Sensor-based feeding: feed in increments near the toolhead sensor
        # instead of blindly feeding a fixed distance. This enables calibration.
        self.sensor_approach_margin = config.getfloat("sensor_approach_margin", 60.0)  # mm before expected sensor to switch to incremental
        self.sensor_step = config.getfloat("sensor_step", 20.0)                       # mm per check during sensor approach
        self.calibration_step = config.getfloat("calibration_step", 10.0)              # mm per check during calibration (more precise)
        self.max_feed_overshoot = config.getfloat("max_feed_overshoot", 100.0)         # mm extra to try past feed_length before giving up

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

        self.printer.register_event_handler("klippy:ready", self._handle_ready)

    def _handle_ready(self):
        """Schedule deferred init — reactor pause is disabled during klippy:ready."""
        self.afc.reactor.register_callback(self._deferred_init)

    def _deferred_init(self, eventtime):
        """Connect to ACE hardware after reactor is fully running."""
        try:
            self._ace = ACEConnection(
                reactor=self.afc.reactor,
                serial_port=self.serial_port,
                logger=self.logger,
                baud_rate=self.baud_rate,
            )
            self._ace.connect()
        except Exception as e:
            self.logger.error(
                f"AFCACE {self.name}: failed to connect to ACE at "
                f"{self.serial_port}: {e}\n{traceback.format_exc()}"
            )
            self._ace = None
            return

        # Enable RFID reader so get_filament_info returns spool data
        try:
            self._ace.enable_rfid()
            self.logger.debug(f"AFCACE {self.name}: RFID enabled")
        except Exception as e:
            self.logger.warning(
                f"AFCACE {self.name}: enable_rfid failed (non-fatal): {e}"
            )

        # Sync slot inventory and loaded states
        self._sync_inventory()
        self._sync_slot_loaded_state()

        # Start runout detection polling
        self._start_slot_status_monitor()

        self.logger.info(
            f"AFCACE {self.name}: connected, mode={self.mode}, "
            f"port={self.serial_port}, slots={self.SLOTS_PER_UNIT}"
        )

    def _get_feed_assist_for_slot(self, slot_index) -> bool:
        """Check if feed assist is enabled for a specific slot."""
        override = self._slot_feed_assist.get(slot_index)
        if override is not None:
            return override
        return self._default_feed_assist

    # ---- Slot / Lane Mapping ----

    def _get_local_slot_for_lane(self, lane) -> int:
        """Map an AFC lane to a local ACE slot index (0-3).

        Lane's unit field is 'UnitName:SlotIndex' where SlotIndex is 1-based
        in the config. We convert to 0-based for ACE hardware.
        """
        unit_field = getattr(lane, "unit", "")
        if ":" in str(unit_field):
            try:
                slot_str = str(unit_field).split(":")[-1]
                return int(slot_str) - 1  # Config is 1-based, ACE is 0-based
            except (ValueError, IndexError):
                pass
        return -1

    # ---- Inventory / State Sync ----

    def _sync_inventory(self):
        """Pull RFID/NFC filament data from ACE hardware into local cache."""
        if self._ace is None or not self._ace.connected:
            return

        for slot in range(self.SLOTS_PER_UNIT):
            try:
                info = self._ace.get_filament_info(slot)
                if isinstance(info, dict):
                    self._slot_inventory[slot] = {
                        "status": info.get("status", "unknown"),
                        "material": info.get("material", info.get("type", "")),
                        "color": info.get("color", [0, 0, 0]),
                    }
            except Exception as e:
                self.logger.debug(
                    f"AFCACE {self.name}: slot {slot} inventory query failed: {e}"
                )

    def _sync_slot_loaded_state(self):
        """Sync ACE slot status to lane loaded_to_hub for virtual sensors."""
        if self._ace is None or not self._ace.connected:
            return

        for lane in self.lanes.values():
            local_slot = self._get_local_slot_for_lane(lane)
            if 0 <= local_slot < self.SLOTS_PER_UNIT:
                slot_info = self._slot_inventory[local_slot]
                slot_loaded = bool(
                    slot_info and slot_info.get("status", "") == "ready"
                )
                lane.loaded_to_hub = slot_loaded

                # Sync material/color from RFID data
                material = slot_info.get("material", "")
                color = slot_info.get("color", [0, 0, 0])
                if material and hasattr(lane, "material"):
                    lane.material = material
                if color and hasattr(lane, "color"):
                    lane.color = self._ace_color_to_hex(color)

    def get_slot_info(self, local_slot):
        """Return cached slot info dict for a slot index."""
        if 0 <= local_slot < self.SLOTS_PER_UNIT:
            return self._slot_inventory[local_slot]
        return {}

    # ---- AFC Unit Interface Implementation ----

    def handle_connect(self):
        super().handle_connect()
        self._register_gcode_commands()

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

        IMPORTANT: This must NOT send serial commands — Klipper calls this
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

        return response

    def load_sequence(self, cur_lane, cur_hub, cur_extruder):
        """Load filament from ACE slot into the toolhead.

        In combined mode: retracts the currently loaded slot first.
        In direct mode: feeds independently (each lane has its own extruder).
        """
        afc = self.afc

        if self._ace is None or not self._ace.connected:
            afc.error.handle_lane_failure(
                cur_lane,
                f"AFCACE load failed: ACE not connected ({self.serial_port})",
            )
            return False

        # Check if already loaded
        if (cur_lane.get_toolhead_pre_sensor_state()
                and hasattr(cur_lane, "tool_loaded") and cur_lane.tool_loaded):
            self.logger.debug(
                f"Lane {cur_lane.name} already loaded to toolhead, skipping"
            )
            cur_lane.set_tool_loaded()
            afc.save_vars()
            return True

        if afc._check_extruder_temp(cur_lane):
            afc.afcDeltaTime.log_with_time("Done heating toolhead")

        local_slot = self._get_local_slot_for_lane(cur_lane)
        if local_slot < 0:
            afc.error.handle_lane_failure(
                cur_lane,
                f"AFCACE load failed: cannot determine slot for {cur_lane.name}",
            )
            return False

        try:
            # Combined mode: retract current slot first
            if self.mode == MODE_COMBINED and self._current_loaded_slot >= 0:
                if self._current_loaded_slot != local_slot:
                    self.logger.info(
                        f"AFCACE combined mode: retracting slot "
                        f"{self._current_loaded_slot} before loading slot {local_slot}"
                    )
                    self._retract_slot(self._current_loaded_slot)
                    self._current_loaded_slot = -1

            # Feed the target slot (pass lane for sensor-based stopping)
            self.logger.info(
                f"AFCACE load: feeding slot {local_slot} for lane {cur_lane.name} "
                f"(mode={self.mode})"
            )
            self._feed_slot(local_slot, lane=cur_lane)

            if self.mode == MODE_COMBINED:
                self._current_loaded_slot = local_slot

        except Exception as e:
            message = f"AFCACE load failed for {cur_lane.name}: {e}"
            self.logger.error(f"{message}\n{traceback.format_exc()}")
            afc.error.handle_lane_failure(cur_lane, message)
            return False

        # Buffer/ramming mode: if tool_start == "buffer", the buffer's advance_state
        # is used as the toolhead sensor. After loading, the buffer should be compressed
        # (advance_state True). We need to back off until it decompresses to confirm load.
        if cur_extruder.tool_start == "buffer" and cur_lane.buffer_obj is not None:
            try:
                cur_lane.unsync_to_extruder()
                load_checks = 0
                while cur_lane.get_toolhead_pre_sensor_state():
                    cur_lane.move_advanced(cur_lane.short_move_dis * -1, 1)  # 1 = SpeedMode.SHORT
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
                cur_lane.sync_to_extruder()
            except Exception as e:
                message = f"AFCACE buffer load check failed for {cur_lane.name}: {e}"
                self.logger.error(f"{message}\n{traceback.format_exc()}")
                afc.error.handle_lane_failure(cur_lane, message)
                return False
        else:
            # Standard toolhead sensor verification
            if not cur_lane.get_toolhead_pre_sensor_state():
                message = (
                    "AFCACE load did not trigger toolhead sensor. CHECK FILAMENT PATH\n"
                    "To resolve, set lane loaded with "
                    f"`SET_LANE_LOADED LANE={cur_lane.name}` macro."
                )
                if afc.function.in_print():
                    message += (
                        "\nOnce filament is fully loaded click resume to continue printing"
                    )
                afc.error.handle_lane_failure(cur_lane, message)
                return False

        cur_lane.set_tool_loaded()
        cur_lane.enable_buffer(disable_fault=True)
        afc.save_vars()
        return True

    def unload_sequence(self, cur_lane, cur_hub, cur_extruder):
        """Unload filament: shared toolhead steps then ACE retraction."""
        afc = self.afc

        if self._ace is None or not self._ace.connected:
            afc.error.handle_lane_failure(
                cur_lane,
                f"AFCACE unload failed: ACE not connected ({self.serial_port})",
            )
            return False

        cur_lane.status = AFCLaneState.TOOL_UNLOADING

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
            afc.gcode.run_script_from_command(afc.tool_cut_cmd)

            if afc.park:
                afc.gcode.run_script_from_command(afc.park_cmd)

        if afc.form_tip:
            if afc.park:
                afc.gcode.run_script_from_command(afc.park_cmd)

            if afc.form_tip_cmd == "AFC":
                afc.tip = self.printer.lookup_object("AFC_form_tip")
                afc.tip.tip_form()
            else:
                afc.gcode.run_script_from_command(afc.form_tip_cmd)

        local_slot = self._get_local_slot_for_lane(cur_lane)

        try:
            # Unsync from extruder before ACE retraction
            cur_lane.unsync_to_extruder()

            self.logger.info(
                f"AFCACE unload: retracting slot {local_slot} "
                f"for lane {cur_lane.name} (mode={self.mode})"
            )

            self._retract_slot(local_slot)

            if self.mode == MODE_COMBINED:
                self._current_loaded_slot = -1

            cur_lane.loaded_to_hub = True
            cur_lane.set_tool_unloaded()
            cur_lane.status = AFCLaneState.LOADED
            self.lane_tool_unloaded(cur_lane)
            afc.save_vars()

        except Exception as e:
            message = f"AFCACE unload failed for {cur_lane.name}: {e}"
            self.logger.error(f"{message}\n{traceback.format_exc()}")
            afc.error.handle_lane_failure(cur_lane, message)
            return False

        return True

    # ---- Low-Level Slot Operations ----

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

        self.logger.debug(
            f"AFCACE feed: slot {slot_index}, "
            f"length={self.feed_length}mm @ {self.feed_speed}mm/min"
        )

        # Phase 1: Bulk feed (skip the last sensor_approach_margin mm)
        bulk_distance = max(0, self.feed_length - self.sensor_approach_margin)
        if bulk_distance > 0:
            ace.feed_filament(slot_index, bulk_distance, self.feed_speed)

        # Phase 2: Sensor approach - feed in increments, checking sensor
        total_fed = bulk_distance
        sensor_triggered = False

        if lane is not None:
            # Check if sensor already triggered during bulk feed
            self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.1)
            if lane.get_toolhead_pre_sensor_state():
                sensor_triggered = True
                self.logger.info(
                    f"AFCACE feed: toolhead sensor triggered during bulk feed "
                    f"at ~{total_fed:.0f}mm"
                )

            # Incremental feed until sensor triggers or max distance reached
            max_total = self.feed_length + self.max_feed_overshoot
            while not sensor_triggered and total_fed < max_total:
                step = min(self.sensor_step, max_total - total_fed)
                ace.feed_filament(slot_index, step, self.feed_speed)
                total_fed += step

                # Brief pause for sensor state to settle
                self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.1)

                if lane.get_toolhead_pre_sensor_state():
                    sensor_triggered = True
                    self.logger.info(
                        f"AFCACE feed: toolhead sensor triggered at {total_fed:.0f}mm"
                    )
        else:
            # No lane/sensor - just feed the remaining fixed distance
            remaining = self.feed_length - bulk_distance
            if remaining > 0:
                ace.feed_filament(slot_index, remaining, self.feed_speed)
                total_fed = self.feed_length

        # Phase 3: Feed assist + extruder assist for the last stretch
        if self._get_feed_assist_for_slot(slot_index):
            try:
                ace.start_feed_assist(slot_index)

                # Use extruder motor to pull filament into hotend
                if self.extruder_assist_length > 0:
                    self.afc.gcode.run_script_from_command(
                        f"G92 E0\n"
                        f"G1 E{self.extruder_assist_length} F{self.extruder_assist_speed}"
                    )
            finally:
                ace.stop_feed_assist(slot_index)

        return total_fed, sensor_triggered

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
            ace.feed_filament(slot_index, step, self.feed_speed)
            total_fed += step

            # Pause for sensor state to settle
            self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.1)

            if lane.get_toolhead_pre_sensor_state():
                self.logger.info(
                    f"AFCACE calibration: sensor triggered at {total_fed:.1f}mm"
                )
                return total_fed, True

        return total_fed, False

    def _retract_slot(self, slot_index):
        """Retract filament from the toolhead back into the ACE slot."""
        ace = self._ace

        self.logger.debug(
            f"AFCACE retract: slot {slot_index}, "
            f"length={self.retract_length}mm @ {self.retract_speed}mm/min"
        )
        ace.unwind_filament(slot_index, self.retract_length, self.retract_speed)

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
            f"LANE_UNLOAD is not supported for AFCACE lane {lane_name}. "
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
        """ACE units use TOOL_UNLOAD for lane reset."""
        return f"TOOL_UNLOAD LANE={lane.name}"

    # ---- System Test / PREP ----

    def system_Test(self, cur_lane, delay, assignTcmd, enable_movement):
        """Validate ACE lane state during PREP without attempting motion."""
        msg = ""
        succeeded = True

        cur_lane.unsync_to_extruder(False)
        self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.7)

        # Query slot status from hardware
        if self._ace is not None and self._ace.connected:
            local_slot = self._get_local_slot_for_lane(cur_lane)
            if 0 <= local_slot < self.SLOTS_PER_UNIT:
                slot_info = self._slot_inventory[local_slot]
                if slot_info:
                    slot_ready = slot_info.get("status", "") == "ready"
                    cur_lane.loaded_to_hub = slot_ready

                    # Sync RFID data to lane
                    material = slot_info.get("material", "")
                    color = slot_info.get("color", [0, 0, 0])
                    if material and hasattr(cur_lane, "material"):
                        cur_lane.material = material
                    if color and hasattr(cur_lane, "color"):
                        cur_lane.color = self._ace_color_to_hex(color)

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
                    tool_ready = (
                        cur_lane.get_toolhead_pre_sensor_state()
                        or cur_lane.extruder_obj.tool_start == "buffer"
                        or cur_lane.extruder_obj.tool_end_state
                    )
                    if tool_ready and cur_lane.extruder_obj.lane_loaded == cur_lane.name:
                        cur_lane.sync_to_extruder()
                        msg += '<span class=primary--text> in ToolHead</span>'
                        if cur_lane.extruder_obj.tool_start == "buffer":
                            msg += '<span class=warning--text> Ram sensor enabled, confirm tool is loaded</span>'
                        if self.afc.function.get_current_lane() == cur_lane.name:
                            self.afc.spool.set_active_spool(cur_lane.spool_id)
                            cur_lane.unit_obj.lane_tool_loaded(cur_lane)
                            cur_lane.status = AFCLaneState.TOOLED

                            # Restore combined mode tracking
                            if self.mode == MODE_COMBINED:
                                local_slot = self._get_local_slot_for_lane(cur_lane)
                                if local_slot >= 0:
                                    self._current_loaded_slot = local_slot
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
        """Calibrate bowden length by feeding until toolhead sensor triggers.

        Feeds filament from the ACE slot in small increments, checking the
        toolhead sensor between each step. The total distance when the sensor
        triggers becomes the measured bowden/feed_length.

        This replaces the "not supported" stub - since we control the ACE
        hardware directly, we can measure the distance ourselves.
        """
        if self._ace is None or not self._ace.connected:
            return False, "AFCACE not connected", 0

        local_slot = self._get_local_slot_for_lane(cur_lane)
        if local_slot < 0:
            return False, f"Cannot determine slot for {cur_lane.name}", 0

        # Don't calibrate if sensor is already triggered
        if cur_lane.get_toolhead_pre_sensor_state():
            return False, "Toolhead sensor already triggered - unload first", 0

        max_distance = dis if dis > 0 else self.feed_length + self.max_feed_overshoot + 200

        self.logger.info(
            f"AFCACE calibrate_bowden: feeding slot {local_slot} "
            f"in {self.calibration_step}mm steps, max {max_distance}mm"
        )

        distance, triggered = self._feed_until_sensor(
            local_slot, cur_lane, max_distance, step_size=self.calibration_step
        )

        if not triggered:
            # Retract what we fed so filament doesn't jam
            self._retract_slot(local_slot)
            msg = (
                f"Toolhead sensor did not trigger after {distance:.0f}mm. "
                "Check filament path and sensor wiring."
            )
            return False, msg, distance

        # Retract back to the ACE unit
        self._retract_slot(local_slot)

        # Round to nearest integer for clean config values
        new_feed_length = round(distance, 0)
        new_retract_length = round(distance + 20, 0)

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
            f"AFCACE bowden calibration: toolhead sensor triggered at {distance:.1f}mm.\n"
            f"feed_length: {new_feed_length:.0f} (was {old_feed:.0f})\n"
            f"retract_length: {new_retract_length:.0f} (was {old_retract:.0f})\n"
            f"Values saved to config."
        )
        return True, msg, distance

    def calibrate_hub(self, cur_lane, tol):
        """Hub calibration not applicable - ACE manages hub state internally."""
        return False, "AFCACE hub is managed by ACE hardware. No calibration needed.", 0

    def calibrate_lane(self, cur_lane, tol):
        """Lane calibration: alias for bowden calibration on AFCACE units.

        On stepper units, calibrate_lane measures extruder-to-hub distance.
        On AFCACE, there's only one distance that matters: ACE slot to toolhead.
        This delegates to calibrate_bowden for the same measurement.
        """
        return self.calibrate_bowden(cur_lane, 0, tol)

    def calibrate_td1(self, cur_lane, dis, tol):
        """Calibrate TD-1 bowden length by feeding until TD-1 device detects filament.

        Feeds filament from the ACE slot in small increments, polling the TD-1
        sensor via Moonraker between each step. The total distance when TD-1
        detects filament becomes the measured td1_bowden_length.

        :param cur_lane: Lane to use for calibration
        :param dis: Distance step for incremental feeding (mm)
        :param tol: Tolerance (unused for AFCACE but kept for interface compatibility)
        :return: (success, message, length) tuple
        """
        if self._ace is None or not self._ace.connected:
            return False, "AFCACE not connected", 0

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
        max_bowden_length = self.feed_length + self.max_feed_overshoot + 500

        self.logger.info(
            f"AFCACE calibrate_td1: feeding slot {local_slot} in {step_size}mm steps, "
            f"max {max_bowden_length}mm, TD-1 device={cur_lane.td1_device_id}"
        )

        # Feed incrementally until TD-1 detects filament
        bow_pos = 0.0
        compare_time = datetime.now()
        while not self.get_td1_data(cur_lane, compare_time):
            if bow_pos > max_bowden_length:
                # Retract what we fed
                self._retract_slot(local_slot)
                msg = f"TD-1 failed to detect filament after moving {bow_pos:.0f}mm"
                return False, msg, bow_pos

            compare_time = datetime.now()
            bow_pos += step_size
            self._ace.feed_filament(local_slot, step_size, self.feed_speed)

            # Brief pause for TD-1 to register
            self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.2)

        self.logger.info(
            f"AFCACE calibrate_td1: TD-1 detected filament at {bow_pos:.1f}mm"
        )

        # Retract back to ACE unit
        self._retract_slot(local_slot)

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
            f"AFCACE TD-1 calibration: filament detected at {bow_pos:.1f}mm.\n"
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
            f"AFCACE {self.name}: started slot status monitor "
            f"(interval={self.poll_interval}s)"
        )

    def _poll_slot_status(self, eventtime):
        """Periodic callback to detect slot status changes during printing.

        Uses a single get_status call per poll to check slot states.
        Full inventory sync (4x get_filament_info) only runs infrequently.
        """
        if self._ace is None or not self._ace.connected:
            return eventtime + self.poll_interval * 4

        is_printing = False
        try:
            is_printing = self.afc.function.is_printing()
        except Exception:
            pass

        # Single get_status call — refreshes cached hw status AND slot states
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
            slot_ready = bool(
                slot_info and slot_info.get("status", "") == "ready"
            )

            # Always sync loaded_to_hub so prep/status is accurate
            lane.loaded_to_hub = slot_ready

            prev_ready = self._prev_slot_states.get(lane.name, slot_ready)
            self._prev_slot_states[lane.name] = slot_ready

            # Detect ready -> not-ready transition (filament runout)
            if prev_ready and not slot_ready:
                if lane.status == AFCLaneState.TOOLED:
                    self.logger.info(
                        f"AFCACE runout detected on {lane.name} (slot {local_slot})"
                    )
                    lane.loaded_to_hub = False

                    if lane.runout_lane:
                        try:
                            lane._perform_infinite_runout()
                        except Exception as e:
                            self.logger.error(
                                f"AFCACE infinite spool failed for {lane.name}: "
                                f"{e}\n{traceback.format_exc()}"
                            )
                            lane._perform_pause_runout()
                        finally:
                            lane.loaded_to_hub = False
                    else:
                        lane._perform_pause_runout()
                elif lane.status == AFCLaneState.LOADED:
                    # Slot went empty on a non-active lane - just update sensor state
                    lane.loaded_to_hub = False

        # Polling rates: 2s when printing (runout detection), 5s when idle
        if is_printing:
            return eventtime + max(self.poll_interval, 2.0)
        return eventtime + 5.0

    # ---- Inventory Sync ----

    def sync_ace_inventory(self):
        """Public method to force-sync RFID/spool data from ACE hardware."""
        self._sync_inventory()
        self._sync_slot_loaded_state()

    # ---- GCode Commands ----

    def _register_gcode_commands(self):
        """Register AFCACE-specific GCode commands."""
        self.gcode.register_mux_command(
            "AFCACE_STATUS", "UNIT", self.name,
            self.cmd_AFCACE_STATUS,
            desc="Query AFCACE hardware status",
        )
        self.gcode.register_mux_command(
            "AFCACE_DRY", "UNIT", self.name,
            self.cmd_AFCACE_DRY,
            desc="Start ACE filament dryer",
        )
        self.gcode.register_mux_command(
            "AFCACE_DRY_STOP", "UNIT", self.name,
            self.cmd_AFCACE_DRY_STOP,
            desc="Stop ACE filament dryer",
        )
        self.gcode.register_mux_command(
            "AFCACE_FEED_ASSIST", "UNIT", self.name,
            self.cmd_AFCACE_FEED_ASSIST,
            desc="Enable/disable feed assist for AFCACE unit",
        )
        self.gcode.register_mux_command(
            "AFCACE_SYNC_INVENTORY", "UNIT", self.name,
            self.cmd_AFCACE_SYNC_INVENTORY,
            desc="Refresh RFID/spool inventory from ACE hardware",
        )
        self.gcode.register_mux_command(
            "AFCACE_CALIBRATE", "UNIT", self.name,
            self.cmd_AFCACE_CALIBRATE,
            desc="Calibrate AFCACE bowden length by feeding until toolhead sensor triggers",
        )

    def cmd_AFCACE_STATUS(self, gcmd):
        """Query and display AFCACE hardware status.

        Usage: AFCACE_STATUS UNIT=<name>
        """
        if self._ace is None or not self._ace.connected:
            gcmd.respond_info(f"AFCACE {self.name}: not connected")
            return

        try:
            status = self._ace.get_status()
            gcmd.respond_info(f"AFCACE {self.name} status: {status}")
        except Exception as e:
            gcmd.respond_info(f"AFCACE {self.name} status query failed: {e}")

    def cmd_AFCACE_DRY(self, gcmd):
        """Start ACE filament dryer.

        Usage: AFCACE_DRY UNIT=<name> TEMP=<celsius> DURATION=<minutes> [FAN=<rpm>]
        """
        if self._ace is None or not self._ace.connected:
            gcmd.respond_info(f"AFCACE {self.name}: not connected")
            return

        temp = gcmd.get_int("TEMP", 50)
        duration = gcmd.get_int("DURATION", 240)
        fan = gcmd.get_int("FAN", 800)

        try:
            self._ace.start_drying(temp, fan, duration)
            gcmd.respond_info(
                f"AFCACE {self.name}: drying started "
                f"({temp}C, {duration}min, fan={fan}rpm)"
            )
        except Exception as e:
            gcmd.respond_info(f"AFCACE {self.name}: drying failed: {e}")

    def cmd_AFCACE_DRY_STOP(self, gcmd):
        """Stop ACE filament dryer.

        Usage: AFCACE_DRY_STOP UNIT=<name>
        """
        if self._ace is None or not self._ace.connected:
            gcmd.respond_info(f"AFCACE {self.name}: not connected")
            return

        try:
            self._ace.stop_drying()
            gcmd.respond_info(f"AFCACE {self.name}: drying stopped")
        except Exception as e:
            gcmd.respond_info(f"AFCACE {self.name}: stop drying failed: {e}")

    def cmd_AFCACE_FEED_ASSIST(self, gcmd):
        """Enable or disable feed assist, globally or per-slot.

        Usage:
            AFCACE_FEED_ASSIST UNIT=<name>                    # query all slots
            AFCACE_FEED_ASSIST UNIT=<name> ENABLE=<0|1>       # set default for all
            AFCACE_FEED_ASSIST UNIT=<name> SLOT=<1-4> ENABLE=<0|1>  # set per-slot
            AFCACE_FEED_ASSIST UNIT=<name> SLOT=<1-4> ENABLE=default # clear per-slot override
        """
        slot_num = gcmd.get_int("SLOT", default=None)
        enable_str = gcmd.get("ENABLE", default=None)

        # Query mode: no ENABLE param
        if enable_str is None:
            lines = [f"AFCACE {self.name} feed assist:"]
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
                    f"AFCACE {self.name}: invalid slot {slot_num} (must be 1-{self.SLOTS_PER_UNIT})"
                )
                return

            if enable_str.lower() == "default":
                # Clear per-slot override
                self._slot_feed_assist.pop(slot_index, None)
                effective = self._get_feed_assist_for_slot(slot_index)
                state = "enabled" if effective else "disabled"
                gcmd.respond_info(
                    f"AFCACE {self.name}: slot {slot_num} feed assist reset to default ({state})"
                )
            else:
                enable = bool(int(enable_str))
                self._slot_feed_assist[slot_index] = enable
                state = "enabled" if enable else "disabled"
                gcmd.respond_info(
                    f"AFCACE {self.name}: slot {slot_num} feed assist {state}"
                )
            return

        # Global default
        enable = bool(int(enable_str))
        self._default_feed_assist = enable
        state = "enabled" if enable else "disabled"
        gcmd.respond_info(f"AFCACE {self.name}: feed assist default {state}")

    def cmd_AFCACE_SYNC_INVENTORY(self, gcmd):
        """Refresh RFID/spool inventory from ACE hardware and sync to lanes.

        Usage: AFCACE_SYNC_INVENTORY UNIT=<name>
        """
        if self._ace is None or not self._ace.connected:
            gcmd.respond_info(f"AFCACE {self.name}: not connected")
            return

        try:
            # Re-enable RFID in case it was disabled
            self._ace.enable_rfid()
            # Pull fresh inventory
            self._sync_inventory()
            self._sync_slot_loaded_state()

            # Report what we found
            lines = [f"AFCACE {self.name} inventory:"]
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
                f"AFCACE {self.name}: inventory sync failed: {e}"
            )

    def cmd_AFCACE_CALIBRATE(self, gcmd):
        """Calibrate bowden length by feeding until toolhead sensor triggers.

        Feeds filament from the specified lane's ACE slot in small increments,
        checking the toolhead sensor between each step. Reports the measured
        distance when the sensor triggers.

        Usage: AFCACE_CALIBRATE UNIT=<name> LANE=<lane_name> [MAX=<mm>]
        """
        if self._ace is None or not self._ace.connected:
            gcmd.respond_info(f"AFCACE {self.name}: not connected")
            return

        lane_name = gcmd.get("LANE", default=None)
        if lane_name is None:
            gcmd.respond_info("AFCACE_CALIBRATE requires LANE=<lane_name>")
            return

        cur_lane = self.lanes.get(lane_name)
        if cur_lane is None:
            # Try looking up from AFC global lanes
            cur_lane = self.afc.lanes.get(lane_name)
        if cur_lane is None:
            gcmd.respond_info(f"Lane '{lane_name}' not found")
            return

        max_distance = gcmd.get_float("MAX", self.feed_length + self.max_feed_overshoot + 200)

        gcmd.respond_info(
            f"AFCACE {self.name}: starting bowden calibration for {lane_name}...\n"
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
    return afcAFCACE(config)