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

        # ACE units don't have physical buffers or stepper-based mechanisms
        self.buffer_obj = None

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

        # Feed assist: enable motorized backing during load
        self.use_feed_assist = config.getboolean("use_feed_assist", True)

        # Extruder assist length: how far to advance with extruder motor
        # during feed assist (after filament reaches toolhead sensor area)
        self.extruder_assist_length = config.getfloat("extruder_assist_length", 50.0)  # mm
        self.extruder_assist_speed = config.getfloat("extruder_assist_speed", 300.0)   # mm/min

        # Sensor polling interval for status/runout monitoring
        self.poll_interval = config.getfloat("poll_interval", 1.0)

        # Baud rate (ACE default is 115200)
        self.baud_rate = config.getint("baud_rate", 115200)

        # Serial connection (created on ready)
        self._ace: Optional[ACEConnection] = None

        # Slot inventory cache: [{status, material, color}, ...]
        self._slot_inventory = [{} for _ in range(self.SLOTS_PER_UNIT)]

        # For combined mode: track which slot is currently in the toolhead
        # -1 means nothing loaded
        self._current_loaded_slot = -1

        # Previous slot states for runout detection
        self._prev_slot_states: Dict[str, bool] = {}

        self.printer.register_event_handler("klippy:ready", self._handle_ready)

    def _handle_ready(self):
        """Connect to ACE hardware once Klipper is fully ready."""
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

        try:
            status = self._ace.get_status()
        except Exception:
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
        """Return status dict including ACE hardware state."""
        response = super().get_status(eventtime)

        response["ace_mode"] = self.mode
        response["ace_connected"] = (
            self._ace is not None and self._ace.connected
        )
        response["ace_serial_port"] = self.serial_port

        if self._ace is not None and self._ace.connected:
            try:
                hw_status = self._ace.get_status(timeout=2.0)
                response["ace_status"] = hw_status
            except Exception:
                response["ace_status"] = {}
        else:
            response["ace_status"] = {}

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

            # Feed the target slot
            self.logger.info(
                f"AFCACE load: feeding slot {local_slot} for lane {cur_lane.name} "
                f"(mode={self.mode})"
            )
            self._feed_slot(local_slot)

            if self.mode == MODE_COMBINED:
                self._current_loaded_slot = local_slot

        except Exception as e:
            message = f"AFCACE load failed for {cur_lane.name}: {e}"
            self.logger.error(f"{message}\n{traceback.format_exc()}")
            afc.error.handle_lane_failure(cur_lane, message)
            return False

        # Verify toolhead sensor
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

    def _feed_slot(self, slot_index):
        """Feed filament from an ACE slot through to the toolhead.

        1. Feed filament through the bowden path
        2. Optionally enable feed assist for extruder loading
        3. Use extruder motor to pull filament into the hotend
        """
        ace = self._ace

        # Step 1: Feed filament through bowden
        self.logger.debug(
            f"AFCACE feed: slot {slot_index}, "
            f"length={self.feed_length}mm @ {self.feed_speed}mm/min"
        )
        ace.feed_filament(slot_index, self.feed_length, self.feed_speed)

        # Step 2: Feed assist + extruder assist for the last stretch
        if self.use_feed_assist:
            try:
                ace.start_feed_assist(slot_index)

                # Use extruder motor to pull filament into hotend
                if self.extruder_assist_length > 0:
                    speed_mm_s = self.extruder_assist_speed / 60.0
                    self.afc.gcode.run_script_from_command(
                        f"G92 E0\n"
                        f"G1 E{self.extruder_assist_length} F{self.extruder_assist_speed}"
                    )
            finally:
                ace.stop_feed_assist(slot_index)

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

    # ---- Calibration (not applicable for ACE hardware) ----

    def calibrate_bowden(self, cur_lane, dis, tol):
        msg = (
            "AFCACE units do not support standard AFC bowden calibration. "
            "Configure feed_length and retract_length in [AFC_AFCACE] section."
        )
        return False, msg, 0

    def calibrate_hub(self, cur_lane, tol):
        return False, "AFCACE units do not support hub calibration.", 0

    def calibrate_lane(self, cur_lane, tol):
        return False, "AFCACE units do not support lane calibration.", 0

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
        self._slot_monitor_timer = self.afc.reactor.register_timer(
            self._poll_slot_status,
            self.afc.reactor.monotonic() + 5.0,
        )
        self.logger.info(
            f"AFCACE {self.name}: started slot status monitor "
            f"(interval={self.poll_interval}s)"
        )

    def _poll_slot_status(self, eventtime):
        """Periodic callback to detect slot status changes during printing."""
        try:
            if not self.afc.function.is_printing():
                return eventtime + self.poll_interval * 2
        except Exception:
            return eventtime + self.poll_interval * 2

        if self._ace is None or not self._ace.connected:
            return eventtime + self.poll_interval * 2

        # Refresh inventory
        try:
            self._sync_inventory()
        except Exception:
            pass

        for lane in self.lanes.values():
            local_slot = self._get_local_slot_for_lane(lane)
            if local_slot < 0 or local_slot >= self.SLOTS_PER_UNIT:
                continue

            slot_info = self._slot_inventory[local_slot]
            slot_ready = bool(
                slot_info and slot_info.get("status", "") == "ready"
            )
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

        return eventtime + self.poll_interval

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
        """Enable or disable feed assist at runtime.

        Usage: AFCACE_FEED_ASSIST UNIT=<name> ENABLE=<0|1>
        """
        enable = gcmd.get_int("ENABLE", default=None)
        if enable is None:
            state = "enabled" if self.use_feed_assist else "disabled"
            gcmd.respond_info(
                f"AFCACE {self.name}: feed assist is {state}"
            )
            return

        self.use_feed_assist = bool(enable)
        state = "enabled" if self.use_feed_assist else "disabled"
        gcmd.respond_info(f"AFCACE {self.name}: feed assist {state}")

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
