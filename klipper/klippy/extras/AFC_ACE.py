# Armored Turtle Automated Filament Control
#
# Copyright (C) 2024-2026 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.

"""AFC unit driver for Anycubic ACE PRO filament changers."""

from __future__ import annotations

import logging
import traceback
from configparser import Error as error
from pathlib import Path
from typing import Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from extras.AFC_lane import AFCLane

try: from extras.AFC_utils import ERROR_STR
except: raise error("Error when trying to import AFC_utils.ERROR_STR\n{trace}".format(trace=traceback.format_exc()))

try: from extras.AFC_lane import AFCLaneState, SpeedMode, AssistActive
except: raise error(ERROR_STR.format(import_lib="AFC_lane", trace=traceback.format_exc()))

try: from extras.AFC_unit import afcUnit
except: raise error(ERROR_STR.format(import_lib="AFC_unit", trace=traceback.format_exc()))

try: from extras.AFC_ACE_serial import ACEConnection, ACESerialError, ACETimeoutError
except: raise error(ERROR_STR.format(import_lib="AFC_ACE_serial", trace=traceback.format_exc()))

try:
    from extras.AFC_RFID import (
        rgb_array_to_hex, color_name, apply_filament_defaults,
        sync_rfid_to_spoolman, get_auto_spoolman_create,
        log_new_filament, log_new_spool,
    )
except:
    rgb_array_to_hex = None

try: from extras.AFC_logger import AFC_QueueListener
except: AFC_QueueListener = None

try: from queuelogger import QueueHandler
except: QueueHandler = None


MODE_COMBINED = "combined"
MODE_DIRECT = "direct"


class afcACE(afcUnit):
    SLOTS_PER_UNIT = 4

    def __init__(self, config):
        super().__init__(config)
        self.type = config.get('type', 'ACE')

        self.serial_port = config.get("serial_port")
        mode = config.get("mode", MODE_COMBINED).lower().strip()
        if mode not in (MODE_COMBINED, MODE_DIRECT):
            raise error(
                f"[{config.get_name()}] invalid mode '{mode}'. "
                f"Use '{MODE_COMBINED}' or '{MODE_DIRECT}'.")
        self.mode = mode

        self.feed_speed = config.getfloat("feed_speed", 60.0)
        self.retract_speed = config.getfloat("retract_speed", 50.0)
        self.unload_preretract = config.getfloat("unload_preretract", 50.0)
        self._unit_load_to_hub = config.getboolean("load_to_hub", None)
        self._default_feed_assist = config.getboolean("use_feed_assist", True)
        self.extruder_assist_length = config.getfloat("extruder_assist_length", 50.0)
        self.extruder_assist_speed = config.getfloat("extruder_assist_speed", 300.0)
        self.sensor_approach_margin = config.getfloat("sensor_approach_margin", 30.0)
        self.sensor_step = config.getfloat("sensor_step", 40.0)
        self.calibration_step = config.getfloat("calibration_step", 50.0)
        self.max_feed_overshoot = config.getfloat("max_feed_overshoot", 100.0)
        self.fps_threshold = config.getfloat("fps_threshold", 0.9)
        self.fps_load_threshold = config.getfloat("fps_load_threshold", 0.65)
        self.fps_delta_threshold = config.getfloat("fps_delta_threshold", 0.15)
        self.fps_confirm_count = config.getint("fps_confirm_count", 3, minval=1)
        self.baud_rate = config.getint("baud_rate", 115200)

        self.auto_spoolman_create = config.getboolean("auto_spoolman_create", False)

        self._ace: Optional[ACEConnection] = None
        self._slot_map: dict[str, int] = {}
        self._feed_assist_active: set[int] = set()
        self._slot_inventory: list[dict] = [{} for _ in range(self.SLOTS_PER_UNIT)]
        self._operation_active = False
        self._prev_states_stale = False
        self._prev_slot_states: dict[str, bool] = {}
        self._hub_load_suppressed: set[str] = set()

        self.gcode = self.printer.lookup_object('gcode')
        unit_suffix = self.name.upper().replace(" ", "_")
        self._custom_load_cmd_name = f'_ACE_CUSTOM_LOAD_{unit_suffix}'
        self._custom_unload_cmd_name = f'_ACE_CUSTOM_UNLOAD_{unit_suffix}'
        self.gcode.register_command(
            self._custom_load_cmd_name, self._cmd_ace_custom_load,
            desc=f"ACE internal load command ({self.name})")
        self.gcode.register_command(
            self._custom_unload_cmd_name, self._cmd_ace_custom_unload,
            desc=f"ACE internal unload command ({self.name})")
        self.gcode.register_command(
            f'ACE_CALIBRATE_{unit_suffix}', self.cmd_ACE_CALIBRATE,
            desc=f"Calibrate ACE feed distance to toolhead ({self.name})")
        self.gcode.register_command(
            f'ACE_CALIBRATE_HUB_{unit_suffix}', self.cmd_ACE_CALIBRATE_HUB,
            desc=f"Calibrate ACE feed distance to hub ({self.name})")
        self.gcode.register_command(
            f'ACE_STATUS_{unit_suffix}', self.cmd_ACE_STATUS,
            desc=f"Query ACE hardware status ({self.name})")
        self.gcode.register_command(
            f'ACE_DRY_{unit_suffix}', self.cmd_ACE_DRY,
            desc=f"Start ACE filament dryer ({self.name})")
        self.gcode.register_command(
            f'ACE_LANE_RESET_{unit_suffix}', self.cmd_ACE_LANE_RESET,
            desc=f"Retract ACE lane filament back into unit ({self.name})")

        # Register temperature_ace sensor factory during config parsing
        # so [temperature_sensor] sections can resolve sensor_type: temperature_ace
        try:
            from extras.temperature_ace import TemperatureACE
            pheaters = self.printer.load_object(config, "heaters")
            pheaters.add_sensor_factory("temperature_ace", TemperatureACE)
        except Exception:
            pass

    def handle_connect(self):
        super().handle_connect()

        self.logo = '<span class=success--text>R  ACE PRO\n'
        self.logo += 'E  ========\n'
        self.logo += 'A  |      |\n'
        self.logo += 'D  | {slots} |\n'.format(slots=''.join(['[{}]'.format(i) for i in range(len(self.lanes))]))
        self.logo += 'Y  ========</span>\n'
        self.logo += '  ' + self.name + '\n'

        self.logo_error = '<span class=error--text>E  ACE PRO\n'
        self.logo_error += 'R  ========\n'
        self.logo_error += 'R  | ERR  |\n'
        self.logo_error += 'O  |  <span class=secondary--text>X</span>   |\n'
        self.logo_error += 'R  ========</span>\n'
        self.logo_error += '  ' + self.name + '\n'

        # Build slot map (1-based config index → 0-based ACE slot)
        # and set custom load/unload commands
        for lane_name, lane in self.lanes.items():
            idx = getattr(lane, 'index', 0)
            slot = max(0, idx - 1)
            self._slot_map[lane_name] = slot
            lane.custom_load_cmd = f"{self._custom_load_cmd_name} UNIT={self.name} LANE={lane_name}"
            lane.custom_unload_cmd = f"{self._custom_unload_cmd_name} UNIT={self.name} LANE={lane_name}"

        # Defer serial connection until reactor is running (klippy:ready)
        self.printer.register_event_handler("klippy:ready", self._handle_ready)

    _CONNECT_MAX_RETRIES = 5
    _CONNECT_RETRY_DELAYS = [2.0, 3.0, 5.0, 8.0, 10.0]
    _SERIAL_LOG_MAX_BYTES = 5 * 1024 * 1024

    def _create_serial_logger(self):
        """Create a dedicated logger for ACE serial comms (writes to AFC_ACE_serial.log)."""
        if AFC_QueueListener is None or QueueHandler is None:
            return None
        logger = logging.getLogger("AFC_ACE_serial_file")
        if logger.handlers:
            return logger
        logger.setLevel(logging.DEBUG)
        logger.propagate = False
        try:
            log_path = self.printer.start_args.get("log_file", None)
            if log_path:
                log_file = Path(log_path).parent / "AFC_ACE_serial.log"
                # Rotate oversized log on startup
                try:
                    lp = Path(log_file)
                    if lp.exists() and lp.stat().st_size > self._SERIAL_LOG_MAX_BYTES:
                        backup = lp.with_suffix('.log.1')
                        if backup.exists():
                            backup.unlink()
                        lp.rename(backup)
                except Exception:
                    pass
                ql = AFC_QueueListener(log_file)
                ql.setFormatter(logging.Formatter(
                    "%(asctime)s %(message)s", datefmt="%H:%M:%S"))
                handler = QueueHandler(ql.bg_queue)
                logger.addHandler(handler)
                self._serial_ql = ql
                return logger
        except Exception:
            pass
        return None

    def _handle_ready(self):
        self.afc.reactor.register_callback(self._deferred_ace_connect)

    def _deferred_ace_connect(self, eventtime):
        """Connect to ACE hardware after reactor is fully running."""
        serial_logger = self._create_serial_logger() or self.logger
        last_err = None
        for attempt in range(self._CONNECT_MAX_RETRIES):
            try:
                self._ace = ACEConnection(
                    reactor=self.afc.reactor,
                    serial_port=self.serial_port,
                    logger=serial_logger,
                    baud_rate=self.baud_rate,
                )
                self._ace.connect()
                if attempt > 0:
                    self.logger.info(
                        f"ACE {self.name}: connected on attempt {attempt + 1}")
                break
            except Exception as e:
                last_err = e
                self._ace = None
                if attempt < self._CONNECT_MAX_RETRIES - 1:
                    delay = self._CONNECT_RETRY_DELAYS[attempt]
                    self.logger.info(
                        f"ACE {self.name}: serial port not available, "
                        f"retrying in {delay:.0f}s "
                        f"(attempt {attempt + 1}/{self._CONNECT_MAX_RETRIES})")
                    self.afc.reactor.pause(
                        self.afc.reactor.monotonic() + delay)
        else:
            self.logger.error(
                f"ACE {self.name}: failed to connect at "
                f"{self.serial_port} after {self._CONNECT_MAX_RETRIES} "
                f"attempts: {last_err}")
            return

        self.logger.info(
            f"ACE {self.name}: connected, mode={self.mode}, "
            f"port={self.serial_port}, slots={self.SLOTS_PER_UNIT}")

        # Enable RFID reader so get_filament_info returns spool data
        try:
            self._ace.enable_rfid()
            self.logger.debug(f"ACE {self.name}: RFID enabled")
        except Exception as e:
            self.logger.warning(
                f"ACE {self.name}: enable_rfid failed (non-fatal): {e}")

        # Seed slot status from get_status
        try:
            hw_status = self._ace.get_status(timeout=2.0)
            if isinstance(hw_status, dict):
                for i, slot_data in enumerate(hw_status.get("slots", [])):
                    if i < self.SLOTS_PER_UNIT and isinstance(slot_data, dict):
                        self._slot_inventory[i]["status"] = slot_data.get("status", "")
        except Exception as e:
            self.logger.debug(f"ACE initial get_status failed: {e}")

        # Sync RFID data and lane loaded states
        self._sync_inventory()
        self._sync_slot_loaded_state()

        # Register callback for heartbeat status updates
        self._ace.status_callback = self._on_hw_status_callback

    def _on_hw_status_callback(self, response):
        """Process heartbeat status from ACE — keep lane states in sync."""
        if self._operation_active:
            return
        if not isinstance(response, dict):
            return
        result = response.get("result", response)
        if not isinstance(result, dict):
            return
        self._sync_slot_states(result)

    def _is_virtual_hub(self, lane) -> bool:
        hub = lane.hub_obj
        return (hub is not None
                and hasattr(hub, 'is_virtual_pin')
                and hub.is_virtual_pin())

    def _set_hub_state(self, lane, state: bool):
        """Set the virtual hub sensor state for an ACE lane."""
        hub = lane.hub_obj
        if hub and hasattr(hub, 'is_virtual_pin') and hub.is_virtual_pin():
            try:
                eventtime = self.afc.reactor.monotonic()
                hub.switch_pin_callback(eventtime, state)
            except Exception:
                pass

    def _sync_slot_states(self, hw_status):
        """Sync lane prep/load state from ACE hardware slot status.

        Uses _prev_slot_states (per-lane dict) and _prev_states_stale to
        avoid false insert/remove detection after load/unload operations.
        """
        slots = hw_status.get("slots", [])

        resync_prev = self._prev_states_stale
        if resync_prev:
            self._prev_states_stale = False

        for i, slot_data in enumerate(slots):
            if i < self.SLOTS_PER_UNIT and isinstance(slot_data, dict):
                self._slot_inventory[i]["status"] = slot_data.get("status", "")

        for lane in self.lanes.values():
            slot = self._get_slot(lane.name)
            if slot >= len(slots) or not isinstance(slots[slot], dict):
                continue

            slot_status = slots[slot].get("status", "")
            slot_ready = slot_status == "ready"
            slot_transient = slot_status in ("shifting", "feeding", "unwinding")

            if not slot_transient:
                lane._load_state = slot_ready
                lane.prep_state = slot_ready

            prep_done = getattr(lane, '_afc_prep_done', False)

            # Slot ready but lane stuck in NONE — restore loaded state.
            # Uses _hub_load_suppressed to avoid treating a recently-
            # unloaded (but still present) spool as new filament.
            if slot_ready and not slot_transient and prep_done and lane.status == AFCLaneState.NONE:
                if (lane.tool_loaded
                        and hasattr(lane, 'extruder_obj')
                        and lane.extruder_obj.lane_loaded == lane.name):
                    self.logger.info(
                        f"ACE: {lane.name} restoring TOOLED state from saved vars")
                    lane.loaded_to_hub = True
                    lane.sync_to_extruder()
                    if self.afc.current == lane.name:
                        self.afc.spool.set_active_spool(lane.spool_id)
                        self.lane_tool_loaded(lane)
                        lane.status = AFCLaneState.TOOLED
                        if self._use_feed_assist(lane) and self._ace is not None:
                            try:
                                self._ace.start_feed_assist(slot)
                                self._feed_assist_active.add(slot)
                            except Exception:
                                pass
                    else:
                        self.lane_tool_loaded_idle(lane)
                    lane.enable_buffer()
                elif lane.name in self._hub_load_suppressed:
                    lane.set_loaded()
                    self.afc.save_vars()
                else:
                    self.logger.info(f"ACE: {lane.name} filament inserted")
                    self.afc.spool.clear_values(lane)
                    lane.set_loaded()
                    self._hub_load_suppressed.discard(lane.name)
                    self._refresh_slot_inventory(slot)
                    slot_info = self._slot_inventory[slot]
                    if apply_filament_defaults is not None:
                        apply_filament_defaults(
                            lane, slot_info,
                            color_converter=rgb_array_to_hex,
                            afc_defaults={
                                "default_material_type": getattr(self.afc, "default_material_type", None),
                                "default_color": getattr(self.afc, "default_color", None),
                            })
                    if sync_rfid_to_spoolman is not None:
                        self._sync_rfid_to_spoolman(lane, slot_info)
                    self.lane_illuminate_spool(lane)
                    self.afc.save_vars()
                    try:
                        self.prep_post_load(lane)
                    except Exception as e:
                        self.logger.error(f"ACE prep_post_load error for {lane.name}: {e}")

            # Filament removed — skip on first callback after operation
            # to avoid false triggers from stale _prev_slot_states.
            if not slot_ready and not slot_transient and not resync_prev:
                # Spool physically removed — clear suppression so the next
                # new spool insertion gets a fresh prep_post_load cycle.
                self._hub_load_suppressed.discard(lane.name)
                prev_ready = self._prev_slot_states.get(lane.name)
                if prev_ready:
                    if getattr(lane, 'tool_loaded', False):
                        try:
                            is_printing = self.afc.function.is_printing()
                        except Exception:
                            is_printing = False
                        if is_printing:
                            self.afc.error.AFC_error(
                                f"ACE runout on {lane.name}", pause=True)
                    elif lane.status != AFCLaneState.NONE:
                        self.logger.info(f"ACE: {lane.name} filament removed")
                        if slot < self.SLOTS_PER_UNIT:
                            self._clear_slot_inventory(slot)
                        lane.loaded_to_hub = False
                        self._set_hub_state(lane, False)
                        lane.set_unloaded()
                        self.lane_not_ready(lane)
                        self.afc.save_vars()

            if not slot_transient:
                self._prev_slot_states[lane.name] = slot_ready

    # ── Lane parameter helpers ──────────────────────────────────────

    def _get_bowden_length(self, cur_lane) -> float:
        """Total feed distance: dist_hub (lane→hub) + afc_bowden_length (hub→toolhead)."""
        dist = cur_lane.dist_hub
        hub = cur_lane.hub_obj
        if hub is not None:
            dist += getattr(hub, 'afc_bowden_length', 0)
        return dist

    def _get_unload_length(self, cur_lane) -> float:
        """Total retract distance: dist_hub + afc_unload_bowden_length."""
        dist = cur_lane.dist_hub
        hub = cur_lane.hub_obj
        if hub is not None:
            dist += getattr(hub, 'afc_unload_bowden_length', getattr(hub, 'afc_bowden_length', 0))
        return dist

    def _use_feed_assist(self, cur_lane) -> bool:
        fa = getattr(cur_lane, 'use_feed_assist', None)
        if fa is not None:
            return fa
        return self._default_feed_assist

    def _get_slot(self, lane_name: str) -> int:
        return self._slot_map.get(lane_name, 0)

    # ── Unit interface overrides ────────────────────────────────────

    def prep_load(self, lane: AFCLane):
        pass

    def prep_post_load(self, lane: AFCLane):
        """Stage filament to hub via ACE serial feed."""
        if self._unit_load_to_hub is not None:
            load_to_hub = self._unit_load_to_hub
        else:
            load_to_hub = getattr(lane, 'load_to_hub',
                                  getattr(self.afc, 'load_to_hub', False))
        if not load_to_hub or lane.loaded_to_hub:
            return
        if not lane.prep_state:
            return
        slot = self._get_slot(lane.name)
        if not self._ace or not self._ace.connected:
            return

        dist_hub = lane.dist_hub
        if dist_hub <= 0:
            return

        max_attempts = 3
        for attempt in range(max_attempts):
            try:
                self._wait_for_ace_ready(timeout=30.0)
                self._ace.feed_filament(slot, dist_hub, self.feed_speed)
                self._wait_for_feed_complete(slot, dist_hub, self.feed_speed)
                lane.loaded_to_hub = True
                self.afc.save_vars()
                self.logger.info(
                    f"ACE prep_post_load: {lane.name} staged at hub "
                    f"(dist_hub={dist_hub:.0f}mm)")
                return
            except Exception as e:
                if attempt < max_attempts - 1:
                    wait = 5.0 * (attempt + 1)
                    self.logger.warning(
                        f"ACE prep_post_load: attempt {attempt + 1}/{max_attempts} "
                        f"failed, retrying in {wait:.0f}s: {e}")
                    self.afc.reactor.pause(
                        self.afc.reactor.monotonic() + wait)
                    continue
                self.logger.error(
                    f"ACE prep_post_load failed for {lane.name} after "
                    f"{max_attempts} attempts: {e}")

    def eject_lane(self, lane: AFCLane):
        """Retract filament back into ACE unit."""
        slot = self._get_slot(lane.name)
        if self._ace and self._ace.connected:
            try:
                self._stop_feed_assist(slot)
                self._wait_for_ace_ready()
                dist = self._get_unload_length(lane)
                self._ace.unwind_filament(slot, dist, self.retract_speed)
                self._wait_for_feed_complete(slot, dist, self.retract_speed)
            except Exception as e:
                self.logger.error(f"ACE eject failed for {lane.name}: {e}")

    def lane_move(self, cur_lane, distance, speed_mode):
        """Move filament via ACE serial instead of stepper."""
        slot = self._get_slot(cur_lane.name)
        if not self._ace or not self._ace.connected:
            self.logger.error("ACE not connected for lane_move")
            return
        try:
            self._wait_for_ace_ready()
            if distance > 0:
                self._ace.feed_filament(slot, abs(distance), self.feed_speed)
            else:
                self._ace.unwind_filament(slot, abs(distance), self.retract_speed)
            self._wait_for_feed_complete(slot, abs(distance),
                                         self.feed_speed if distance > 0 else self.retract_speed)
        except Exception as e:
            self.logger.error(f"ACE lane_move failed: {e}")

    def lane_unload(self, cur_lane):
        """Custom lane unload — retract via ACE serial."""
        slot = self._get_slot(cur_lane.name)
        if self._ace and self._ace.connected:
            try:
                self._stop_feed_assist(slot)
                self._wait_for_ace_ready()
                dist = self._get_unload_length(cur_lane)
                self._ace.unwind_filament(slot, dist, self.retract_speed)
                self._wait_for_feed_complete(slot, dist, self.retract_speed)
            except Exception as e:
                self.logger.error(f"ACE lane_unload failed for {cur_lane.name}: {e}")
        return True

    def abort_load(self, cur_lane):
        """Cancel in-progress ACE feed."""
        if self._ace and self._ace.connected:
            try:
                self._ace.abort_current_action()
            except Exception:
                pass

    def get_lane_reset_command(self, lane, dis):
        return f"ACE_LANE_RESET UNIT={self.name} LANE={lane.name}"

    def system_Test(self, cur_lane, delay, assignTcmd, enable_movement):
        """ACE system test — query hardware slot status instead of physical switches."""
        msg = ''
        succeeded = True

        cur_lane.unsync_to_extruder(False)
        self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.7)

        # ACE has no physical prep/load switches — determine state from hardware
        if self._ace is not None and self._ace.connected:
            slot = self._get_slot(cur_lane.name)
            try:
                hw_status = self._ace.get_status(timeout=2.0)
                if isinstance(hw_status, dict):
                    slots = hw_status.get("slots", [])
                    if slot < len(slots) and isinstance(slots[slot], dict):
                        slot_ready = slots[slot].get("status", "") == "ready"
                        cur_lane._load_state = slot_ready
                        cur_lane.prep_state = slot_ready
                        if not slot_ready:
                            cur_lane.loaded_to_hub = False
                        elif self._is_virtual_hub(cur_lane):
                            # For virtual hub pins, load_state returns loaded_to_hub,
                            # so we must set it for load_state to report correctly
                            cur_lane.loaded_to_hub = True
            except Exception as e:
                self.logger.debug(f"ACE get_status failed during PREP: {e}")
        else:
            msg = '<span class=error--text>ACE NOT CONNECTED</span>'
            succeeded = False

        if succeeded:
            if not cur_lane.prep_state:
                if not cur_lane.load_state:
                    self.lane_not_ready(cur_lane)
                    msg += '<span class=success--text>EMPTY READY FOR SPOOL</span>'
                else:
                    self.lane_fault(cur_lane)
                    msg += '<span class=error--text>CHECK FILAMENT Prep: False - Load: True</span>'
                    succeeded = False
            else:
                self.lane_loaded(cur_lane)
                msg += "<span class=success--text>LOCKED</span>"
                if not cur_lane.load_state:
                    msg += "<span class=error--text> NOT LOADED</span>"
                    self.lane_not_ready(cur_lane)
                    succeeded = False
                else:
                    cur_lane.status = AFCLaneState.LOADED
                    msg += "<span class=success--text> AND LOADED</span>"
                    self.lane_illuminate_spool(cur_lane)

                    # Apply RFID data if available and not already set
                    if apply_filament_defaults is not None:
                        slot_info = self._slot_inventory[slot] if slot < self.SLOTS_PER_UNIT else {}
                        apply_filament_defaults(
                            cur_lane, slot_info,
                            color_converter=rgb_array_to_hex,
                            afc_defaults={
                                "default_material_type": getattr(self.afc, "default_material_type", None),
                                "default_color": getattr(self.afc, "default_color", None),
                            })

                    # Assume filament staged at hub on startup
                    if not cur_lane.tool_loaded and not cur_lane.loaded_to_hub:
                        load_to_hub = getattr(cur_lane, 'load_to_hub',
                                              getattr(self.afc, 'load_to_hub', False))
                        if load_to_hub:
                            cur_lane.loaded_to_hub = True

                    if (cur_lane.tool_loaded
                        and cur_lane.extruder_obj.lane_loaded == cur_lane.name):
                        cur_lane.loaded_to_hub = True
                        cur_lane.sync_to_extruder()
                        msg += "<span class=primary--text> in ToolHead</span>"

                        if self.afc.current == cur_lane.name:
                            self.afc.spool.set_active_spool(cur_lane.spool_id)
                            self.lane_tool_loaded(cur_lane)
                            cur_lane.status = AFCLaneState.TOOLED

                            # Start feed assist immediately for loaded tool
                            if self._use_feed_assist(cur_lane) and self._ace is not None:
                                try:
                                    self._ace.start_feed_assist(slot)
                                    self._feed_assist_active.add(slot)
                                except Exception:
                                    pass
                        else:
                            self.lane_tool_loaded_idle(cur_lane)
                        cur_lane.enable_buffer()

        if assignTcmd:
            self.afc.function.TcmdAssign(cur_lane)
        cur_lane.send_lane_data()
        cur_lane.do_enable(False)
        self.logger.info('{lane_name} tool cmd: {tcmd:3} {msg}'.format(
            lane_name=cur_lane.name, tcmd=cur_lane.map, msg=msg))
        cur_lane.set_afc_prep_done()
        return succeeded

    def _clear_stale_sensor_state(self, cur_lane):
        """Clear stale tool_start_state / filament_present / FPS latch so
        calibration reads real-time sensor values."""
        sensor_obj = getattr(cur_lane.extruder_obj, 'filament_sensor_obj', None)
        if sensor_obj is not None:
            sensor_obj.runout_helper.filament_present = False
        if hasattr(cur_lane.extruder_obj, 'tool_start_state'):
            cur_lane.extruder_obj.tool_start_state = False
        buffer_obj = getattr(cur_lane, 'buffer_obj', None)
        if buffer_obj is not None and hasattr(buffer_obj, 'clear_advance_latch'):
            buffer_obj.clear_advance_latch()
        self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.5)

    def _toolhead_sensor_triggered(self, cur_lane):
        """Check if the toolhead sensor is triggered, using the raw hardware
        button state for U1 motion sensors (which need encoder rotation for
        filament_present but have a physical switch for static detection)."""
        sensor_obj = getattr(cur_lane.extruder_obj, 'filament_sensor_obj', None)
        if sensor_obj is not None and hasattr(sensor_obj, 'runout_buttun_state'):
            return bool(sensor_obj.runout_buttun_state)
        return cur_lane.get_toolhead_pre_sensor_state()

    def _feed_until_sensor(self, slot, cur_lane, max_distance, step_size=None):
        """Feed filament in steps until toolhead sensor triggers.

        Returns (distance_fed, sensor_triggered).
        """
        if step_size is None:
            step_size = self.calibration_step
        total_fed = 0.0

        while total_fed < max_distance:
            step = min(step_size, max_distance - total_fed)
            self._wait_for_ace_ready()
            try:
                self._ace.feed_filament(slot, step, self.feed_speed)
            except Exception as e:
                self.logger.warning(
                    f"ACE calibration: feed failed at {total_fed:.0f}mm, retrying: {e}")
                self._wait_for_ace_ready(timeout=15.0)
                self._ace.feed_filament(slot, step, self.feed_speed)
            self._wait_for_feed_complete(slot, step, self.feed_speed)
            total_fed += step

            self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.3)
            if self._toolhead_sensor_triggered(cur_lane):
                self.logger.info(f"ACE calibration: sensor triggered at {total_fed:.1f}mm")
                return total_fed, True

        return total_fed, False

    def calibrate_bowden(self, cur_lane, dis, tol):
        """Calibrate afc_bowden_length (hub→toolhead) by feeding until
        the toolhead sensor triggers."""
        slot = self._get_slot(cur_lane.name)
        if not self._ace or not self._ace.connected:
            return False, "ACE not connected", 0

        cur_hub = cur_lane.hub_obj
        if cur_hub is None:
            return False, "Lane has no hub configured", 0

        self._operation_active = True
        try:
            return self._calibrate_bowden_inner(cur_lane, cur_hub, slot)
        finally:
            self._operation_active = False
            self._prev_states_stale = True

    def _calibrate_bowden_inner(self, cur_lane, cur_hub, slot):
        self._clear_stale_sensor_state(cur_lane)

        if self._toolhead_sensor_triggered(cur_lane):
            return False, "Toolhead sensor already triggered — unload first", 0

        old_bowden = getattr(cur_hub, 'afc_bowden_length', 0)
        max_distance = 6000

        self.logger.raw(
            f'Calibrating afc_bowden_length for {cur_lane.name} '
            f'(max {max_distance}mm in {self.calibration_step}mm steps)')

        distance, triggered = self._feed_until_sensor(
            slot, cur_lane, max_distance)

        # Retract
        retract_dist = distance + 5 if triggered else distance
        self.logger.info(f"ACE calibrate: retracting {retract_dist:.0f}mm")
        try:
            self._wait_for_ace_ready()
            self._ace.unwind_filament(slot, retract_dist, self.retract_speed)
            self._wait_for_feed_complete(slot, retract_dist, self.retract_speed)
            self._wait_for_ace_ready(timeout=15.0)
        except Exception as e:
            self.logger.error(f"ACE calibrate: retract failed: {e}")

        # Clear sensor state after retract
        self._clear_stale_sensor_state(cur_lane)
        self.afc.reactor.pause(self.afc.reactor.monotonic() + 2.0)

        if not triggered:
            return False, (
                f"Toolhead sensor did not trigger after {distance:.0f}mm. "
                "Check filament path and sensor wiring."), distance

        # Account for filament already at hub
        if cur_lane.loaded_to_hub:
            dist_hub = cur_lane.dist_hub
            if dist_hub > 0:
                self.logger.info(
                    f"ACE calibrate: filament at hub, adding dist_hub="
                    f"{dist_hub:.0f}mm to measured {distance:.1f}mm")
                distance += dist_hub

        bowden_dist = round(distance, 2)
        cal_msg = f'\n afc_bowden_length: New: {bowden_dist} Old: {old_bowden}'
        cur_hub.afc_bowden_length = bowden_dist
        cur_hub.afc_unload_bowden_length = bowden_dist
        self.afc.function.ConfigRewrite(
            cur_hub.fullname, "afc_bowden_length", bowden_dist, cal_msg)
        self.afc.function.ConfigRewrite(
            cur_hub.fullname, "afc_unload_bowden_length", bowden_dist,
            f'\n afc_unload_bowden_length: {bowden_dist}')

        return True, f"afc_bowden_length calibration: {bowden_dist}mm (was {old_bowden}mm)", bowden_dist

    def calibrate_lane(self, cur_lane, tol):
        """Calibrate dist_hub from ACE slot to hub sensor."""
        self._operation_active = True
        try:
            return self._calibrate_hub_inner(cur_lane)
        finally:
            self._operation_active = False
            self._prev_states_stale = True

    # ── Custom load/unload gcode handlers ───────────────────────────

    def _cmd_ace_custom_load(self, gcmd):
        """Handle _ACE_CUSTOM_LOAD — filament transport to toolhead area."""
        lane_name = gcmd.get('LANE')
        cur_lane = self.afc.lanes.get(lane_name)
        if cur_lane is None:
            raise gcmd.error(f"Unknown lane: {lane_name}")
        cur_extruder = cur_lane.extruder_obj
        result = self._ace_load_sequence(cur_lane, cur_extruder)
        if not result:
            raise gcmd.error(f"ACE load failed for {lane_name}")

    def _cmd_ace_custom_unload(self, gcmd):
        """Handle _ACE_CUSTOM_UNLOAD — filament transport from toolhead."""
        lane_name = gcmd.get('LANE')
        cur_lane = self.afc.lanes.get(lane_name)
        if cur_lane is None:
            raise gcmd.error(f"Unknown lane: {lane_name}")
        cur_extruder = cur_lane.extruder_obj
        result = self._ace_unload_sequence(cur_lane, cur_extruder)
        if not result:
            raise gcmd.error(f"ACE unload failed for {lane_name}")

    def _ace_load_sequence(self, cur_lane, cur_extruder) -> bool:
        """ACE load transport: serial feed to toolhead area + feed assist."""
        self._operation_active = True
        try:
            return self._ace_load_inner(cur_lane, cur_extruder)
        finally:
            self._operation_active = False
            self._prev_states_stale = True

    def _ace_load_inner(self, cur_lane, cur_extruder) -> bool:
        """ACE custom load — filament transport only.

        AFC's load_sequence handles the shared toolhead engagement
        (sync_to_extruder, tool_end, tool_stn, sensor verification,
        buffer ram) after this returns via custom_load_cmd.
        """
        afc = self.afc
        slot = self._get_slot(cur_lane.name)

        if not self._ace or not self._ace.connected:
            afc.error.handle_lane_failure(
                cur_lane, f"ACE not connected ({self.serial_port})")
            return False

        # In combined mode, stop feed assist on other slots
        if self.mode == MODE_COMBINED:
            for other_name, other_slot in self._slot_map.items():
                if other_slot != slot and other_slot in self._feed_assist_active:
                    self._stop_feed_assist(other_slot)

        # Calculate feed distance
        if cur_lane.loaded_to_hub:
            hub = cur_lane.hub_obj
            feed_dist = getattr(hub, 'afc_bowden_length', 0) if hub else 0
            feed_dist = max(feed_dist, 50.0)
        else:
            feed_dist = self._get_bowden_length(cur_lane)

        self._set_hub_state(cur_lane, True)
        self._hub_load_suppressed.discard(cur_lane.name)

        # Enable FPS advance latch so the toolhead sensor stays triggered
        # even when pressure drops briefly after ACE feed completes.
        buffer_obj = getattr(cur_lane, 'buffer_obj', None)
        if buffer_obj is not None and hasattr(buffer_obj, 'enable_advance_latch'):
            buffer_obj.enable_advance_latch()

        # ACE serial feed to toolhead area
        try:
            self._wait_for_ace_ready()
            self._ace.feed_filament(slot, feed_dist, self.feed_speed)
            success = self._wait_for_feed_complete(slot, feed_dist, self.feed_speed, cur_lane)

            if not success:
                if cur_lane.get_toolhead_pre_sensor_state():
                    self.logger.info("Feed error but sensor triggered — continuing")
                else:
                    success = self._smart_load_retry(cur_lane, slot)
                    if not success:
                        afc.error.handle_lane_failure(
                            cur_lane, f"ACE feed failed for {cur_lane.name}")
                        return False
        except Exception as e:
            afc.error.handle_lane_failure(
                cur_lane, f"ACE load feed error: {e}")
            return False

        # Set loaded_to_hub AFTER successful feed
        cur_lane.loaded_to_hub = True

        # Enable feed assist
        if self._use_feed_assist(cur_lane):
            self._start_feed_assist(slot)

        afc.afcDeltaTime.log_with_time("ACE load transport complete")
        return True

    def prepare_unload(self, cur_lane, cur_hub, cur_extruder):
        """Stop ACE feed assist before AFC runs cut/park/tip."""
        slot = self._get_slot(cur_lane.name)
        if slot >= 0:
            self._stop_feed_assist(slot)

    def _ace_unload_sequence(self, cur_lane, cur_extruder) -> bool:
        """ACE unload transport — ACE serial unwind back to hub."""
        self._operation_active = True
        try:
            return self._ace_unload_inner(cur_lane, cur_extruder)
        finally:
            self._operation_active = False
            self._prev_states_stale = True

    def _ace_unload_inner(self, cur_lane, cur_extruder) -> bool:
        """ACE custom unload — filament transport only.

        AFC's unload_sequence handles the shared toolhead operations
        (LED, heat, quick pull, buffer disable, sync, cut/park/tip)
        and unsync_to_extruder before calling this via custom_unload_cmd.
        """
        afc = self.afc
        slot = self._get_slot(cur_lane.name)

        if not self._ace or not self._ace.connected:
            afc.error.handle_lane_failure(
                cur_lane, f"ACE not connected ({self.serial_port})")
            return False

        # Feed assist already stopped by prepare_unload() before cut/park/tip.
        # Extruder retract and unsync handled by AFC.py before custom_unload_cmd.

        # ACE serial unwind — retract to hub staging point
        hub = cur_lane.hub_obj
        bowden = getattr(hub, 'afc_unload_bowden_length', getattr(hub, 'afc_bowden_length', 0)) if hub else 0
        retract_dist = bowden - cur_lane.dist_hub
        try:
            self._wait_for_ace_ready()
            self._ace.unwind_filament(slot, retract_dist, self.retract_speed)
            self._wait_for_feed_complete(slot, retract_dist, self.retract_speed)
        except Exception as e:
            afc.error.handle_lane_failure(
                cur_lane, f"ACE unwind failed for {cur_lane.name}: {e}")
            return False

        afc.afcDeltaTime.log_with_time("ACE unwind complete")

        # Filament staged near hub, ready for fast reload.
        self._set_hub_state(cur_lane, False)
        cur_lane.loaded_to_hub = True
        self.lane_tool_unloaded(cur_lane)
        self._hub_load_suppressed.add(cur_lane.name)

        return True

    # ── Calibration commands ────────────────────────────────────────

    cmd_ACE_CALIBRATE_options = {
        "UNIT": {"type": "string", "default": ""},
        "LANE": {"type": "string", "default": ""},
    }

    def cmd_ACE_CALIBRATE(self, gcmd):
        """Calibrate ACE feed distance to toolhead for a lane."""
        lane_name = gcmd.get('LANE', '')
        if not lane_name or lane_name not in self.afc.lanes:
            gcmd.respond_info("Usage: ACE_CALIBRATE LANE=<lane_name>")
            return
        cur_lane = self.afc.lanes[lane_name]
        success, msg, dist = self.calibrate_bowden(cur_lane, self.calibration_step, 1.0)
        gcmd.respond_info(msg)

    cmd_ACE_CALIBRATE_HUB_options = {
        "UNIT": {"type": "string", "default": ""},
        "LANE": {"type": "string", "default": ""},
    }

    def cmd_ACE_CALIBRATE_HUB(self, gcmd):
        """Calibrate ACE feed distance to hub sensor."""
        lane_name = gcmd.get('LANE', '')
        if not lane_name or lane_name not in self.afc.lanes:
            gcmd.respond_info("Usage: ACE_CALIBRATE_HUB LANE=<lane_name>")
            return
        cur_lane = self.afc.lanes[lane_name]
        if cur_lane.hub_obj and cur_lane.hub_obj.is_virtual_pin():
            gcmd.respond_info("Hub calibration requires a physical hub sensor, not virtual")
            return
        success, msg, dist = self._calibrate_hub_inner(cur_lane)
        gcmd.respond_info(msg)

    def cmd_ACE_STATUS(self, gcmd):
        """Query and display ACE hardware status."""
        if not self._ace or not self._ace.connected:
            gcmd.respond_info("ACE not connected")
            return
        try:
            status = self._ace.get_status()
            gcmd.respond_info(f"ACE Status: {status}")
        except Exception as e:
            gcmd.respond_info(f"Error querying ACE: {e}")

    cmd_ACE_DRY_options = {
        "UNIT": {"type": "string", "default": ""},
        "TEMP": {"type": "float", "default": 50.0},
        "DURATION": {"type": "float", "default": 240.0},
        "FAN": {"type": "int", "default": 5000},
    }

    def cmd_ACE_DRY(self, gcmd):
        """Start ACE filament dryer."""
        temp = gcmd.get_float('TEMP', 50.0)
        duration = gcmd.get_float('DURATION', 240.0)
        fan = gcmd.get_int('FAN', 5000)
        if not self._ace or not self._ace.connected:
            gcmd.respond_info("ACE not connected")
            return
        try:
            self._ace.start_drying(temp, fan, duration)
            gcmd.respond_info(f"ACE dryer started: {temp}°C for {duration} min")
        except Exception as e:
            gcmd.respond_info(f"Error starting dryer: {e}")

    def cmd_ACE_LANE_RESET(self, gcmd):
        """Retract lane filament back into ACE unit."""
        lane_name = gcmd.get('LANE', '')
        if not lane_name or lane_name not in self.afc.lanes:
            gcmd.respond_info("Usage: ACE_LANE_RESET LANE=<lane_name>")
            return
        cur_lane = self.afc.lanes[lane_name]
        self.eject_lane(cur_lane)
        gcmd.respond_info(f"Lane {lane_name} reset")

    # ── Hardware interaction helpers ────────────────────────────────

    def _start_feed_assist(self, slot: int):
        if self._ace and self._ace.connected:
            try:
                self._ace.start_feed_assist(slot)
                self._feed_assist_active.add(slot)
            except Exception as e:
                self.logger.error(f"Failed to start feed assist slot {slot}: {e}")

    def _stop_feed_assist(self, slot: int):
        if self._ace and self._ace.connected:
            try:
                self._ace.stop_feed_assist(slot)
                self._feed_assist_active.discard(slot)
            except Exception as e:
                self.logger.error(f"Failed to stop feed assist slot {slot}: {e}")

    def _wait_for_ace_ready(self, timeout=30.0):
        """Wait for the overall ACE status to be 'ready' before sending commands.

        After a retract/feed completes, the slot may report 'ready'/'empty'
        before the ACE controller finishes internal housekeeping. Sending
        a new feed_filament while the ACE is still busy returns FORBIDDEN.
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
                        f"ACE: waiting for ready "
                        f"(status={hw_status.get('status', '?')}, "
                        f"{elapsed:.1f}s/{timeout:.0f}s)")
            except Exception:
                pass
            self.afc.reactor.pause(
                self.afc.reactor.monotonic() + poll_interval)
            elapsed += poll_interval
        self.logger.warning(
            f"ACE: did not become ready within {timeout:.0f}s, proceeding anyway")

    def _wait_for_feed_complete(self, slot_index, length_mm, speed_mm_s,
                                 lane=None, poll_interval=0.5) -> bool:
        """Wait for ACE feed/unwind movement to complete by polling slot status."""
        ace = self._ace
        if ace is None or not ace.connected:
            return False

        max_wait = (length_mm / max(speed_mm_s, 1)) + 10.0
        deadline = self.afc.reactor.monotonic() + max_wait

        # Phase 0: Wait for slot to leave ready/empty (motor starting)
        departure_deadline = self.afc.reactor.monotonic() + 3.0
        motor_started = False
        while self.afc.reactor.monotonic() < departure_deadline:
            self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.2)
            try:
                hw_status = ace.get_status(timeout=2.0)
                if isinstance(hw_status, dict):
                    slots = hw_status.get("slots", [])
                    if slot_index < len(slots):
                        slot_data = slots[slot_index]
                        if isinstance(slot_data, dict):
                            status = slot_data.get("status", "")
                            if status not in ("ready", "empty", ""):
                                motor_started = True
                                break
            except Exception:
                pass
        if not motor_started:
            self.logger.debug(
                f"ACE wait: slot {slot_index} never left ready state "
                f"after feed command — motor may not have started")
            return False

        # Phase 1: Wait for slot to return to ready/empty (motor done)
        while self.afc.reactor.monotonic() < deadline:
            self.afc.reactor.pause(
                self.afc.reactor.monotonic() + poll_interval)

            if lane is not None and lane.get_toolhead_pre_sensor_state():
                self.logger.debug(
                    f"ACE wait: toolhead sensor triggered for slot {slot_index}")
                try:
                    ace.stop_feed_filament(slot_index)
                except Exception:
                    pass
                return True

            try:
                hw_status = ace.get_status(timeout=2.0)
                if isinstance(hw_status, dict):
                    slots = hw_status.get("slots", [])
                    if slot_index < len(slots):
                        slot_data = slots[slot_index]
                        if isinstance(slot_data, dict):
                            status = slot_data.get("status", "")
                            if status in ("ready", "empty"):
                                return True
            except Exception:
                pass

        self.logger.debug(
            f"ACE wait: timeout waiting for slot {slot_index} "
            f"movement ({max_wait:.1f}s)")
        return False

    def _smart_load_retry(self, cur_lane, slot, max_retries: int = 3) -> bool:
        """Retry loading with incremental feeds when initial feed fails."""
        for attempt in range(max_retries):
            self.logger.info(f"Smart load retry {attempt+1}/{max_retries} for {cur_lane.name}")
            try:
                self._wait_for_ace_ready()
                self._ace.feed_filament(slot, self.sensor_step, self.feed_speed)
                self._wait_for_feed_complete(slot, self.sensor_step, self.feed_speed)
                if cur_lane.get_toolhead_pre_sensor_state():
                    return True
            except Exception:
                pass
            self.afc.reactor.pause(self.afc.reactor.monotonic() + 1.0)
        return False

    def _calibrate_hub_inner(self, cur_lane) -> tuple:
        """Calibrate dist_hub by incrementally feeding until hub sensor triggers."""
        slot = self._get_slot(cur_lane.name)
        if not self._ace or not self._ace.connected:
            return False, "ACE not connected", 0

        hub = cur_lane.hub_obj
        if hub is None or hub.is_virtual_pin():
            return False, "Physical hub sensor required for calibration", 0

        if hub.state:
            return False, "Hub sensor already triggered — clear the hub first", 0

        max_distance = 4000
        step = self.calibration_step
        total_fed = 0.0

        self.logger.raw(
            f'Calibrating dist_hub for {cur_lane.name} '
            f'(max {max_distance}mm in {step}mm steps)')

        while total_fed < max_distance:
            feed_step = min(step, max_distance - total_fed)
            try:
                self._wait_for_ace_ready()
                self._ace.feed_filament(slot, feed_step, self.feed_speed)
                self._wait_for_feed_complete(slot, feed_step, self.feed_speed)
            except Exception as e:
                self.logger.error(f"ACE hub calibrate: feed failed: {e}")
                break
            total_fed += feed_step

            self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.2)
            if hub.state:
                self.logger.info(
                    f"ACE hub calibrate: hub sensor triggered at {total_fed:.1f}mm")
                break

        # Retract
        retract_dist = total_fed - 50 if hub.state else total_fed
        if retract_dist > 0:
            try:
                self._wait_for_ace_ready()
                self._ace.unwind_filament(slot, retract_dist, self.retract_speed)
                self._wait_for_feed_complete(slot, retract_dist, self.retract_speed)
            except Exception as e:
                self.logger.error(f"ACE hub calibrate: retract failed: {e}")

        if not hub.state and total_fed >= max_distance:
            return False, (
                f"Hub sensor did not trigger after {total_fed:.0f}mm. "
                "Check filament path and hub sensor wiring."), total_fed

        old_dist = cur_lane.dist_hub
        new_dist = round(total_fed, 2)
        cal_msg = f'\n dist_hub: New: {new_dist} Old: {old_dist}'
        cur_lane.dist_hub = new_dist
        self.afc.function.ConfigRewrite(cur_lane.fullname, "dist_hub", new_dist, cal_msg)

        return True, f"dist_hub calibration: {new_dist}mm (was {old_dist}mm)", new_dist


    # ── RFID / Spoolman helpers ─────────────────────────────────────

    def _sync_inventory(self):
        """Pull RFID/NFC filament data from ACE hardware into slot cache."""
        if self._ace is None or not self._ace.connected:
            return
        for slot in range(self.SLOTS_PER_UNIT):
            try:
                info = self._ace.get_filament_info(slot)
                if isinstance(info, dict):
                    self._store_slot_rfid(slot, info)
            except Exception as e:
                self.logger.debug(
                    f"ACE {self.name}: slot {slot} inventory query failed: {e}")

    def _store_slot_rfid(self, slot, info):
        """Store RFID fields from a get_filament_info response into slot cache."""
        inv = self._slot_inventory[slot]
        inv["material"] = info.get("material", info.get("type", ""))
        inv["color"] = info.get("color", [0, 0, 0])
        inv["sku"] = info.get("sku", "")
        inv["brand"] = info.get("brand", "")
        inv["diameter"] = info.get("diameter", 1.75)
        inv["total_weight"] = info.get("total", 0)
        inv["current_weight"] = info.get("current", 0)
        ext_temp = info.get("extruder_temp", {})
        bed_temp = info.get("hotbed_temp", {})
        if isinstance(ext_temp, dict) and ext_temp.get("min") and ext_temp.get("max"):
            inv["extruder_temp"] = (ext_temp["min"] + ext_temp["max"]) // 2
        elif isinstance(ext_temp, dict) and ext_temp.get("max"):
            inv["extruder_temp"] = ext_temp["max"]
        else:
            inv["extruder_temp"] = None
        inv["bed_temp"] = bed_temp.get("max") if isinstance(bed_temp, dict) else None

    def _refresh_slot_inventory(self, slot):
        """Fetch fresh RFID data for a single slot from ACE hardware."""
        if self._ace is None or not self._ace.connected:
            return
        if slot < 0 or slot >= self.SLOTS_PER_UNIT:
            return
        try:
            info = self._ace.get_filament_info(slot)
            if isinstance(info, dict):
                self._store_slot_rfid(slot, info)
        except Exception as e:
            self.logger.debug(
                f"ACE {self.name}: slot {slot} RFID refresh failed: {e}")

    def _clear_slot_inventory(self, slot):
        """Clear cached RFID data for a slot."""
        if 0 <= slot < self.SLOTS_PER_UNIT:
            self._slot_inventory[slot]["material"] = ""
            self._slot_inventory[slot]["color"] = [0, 0, 0]

    def _sync_slot_loaded_state(self):
        """Sync ACE slot status to lane load/prep state at startup."""
        if self._ace is None or not self._ace.connected:
            return
        for lane in self.lanes.values():
            slot = self._get_slot(lane.name)
            if 0 <= slot < self.SLOTS_PER_UNIT:
                slot_info = self._slot_inventory[slot]
                slot_loaded = bool(
                    slot_info and slot_info.get("status", "") == "ready")
                lane._load_state = slot_loaded
                lane.prep_state = slot_loaded

                if apply_filament_defaults is not None:
                    apply_filament_defaults(
                        lane, slot_info,
                        color_converter=rgb_array_to_hex,
                        afc_defaults={
                            "default_material_type": getattr(self.afc, "default_material_type", None),
                            "default_color": getattr(self.afc, "default_color", None),
                        })

                if slot_loaded and sync_rfid_to_spoolman is not None:
                    self._sync_rfid_to_spoolman(lane, slot_info)

    def _sync_rfid_to_spoolman(self, lane, slot_info):
        """Sync ACE RFID tag data to Spoolman."""
        if sync_rfid_to_spoolman is None:
            return
        if self.afc.spoolman is None or self.afc.moonraker is None:
            return
        if getattr(lane, "spool_id", None) not in (None, "", 0):
            return
        sku = slot_info.get("sku", "")
        if not sku:
            return

        color_rgb = slot_info.get("color", [0, 0, 0])
        color_hex = ""
        if rgb_array_to_hex is not None and color_rgb != [0, 0, 0]:
            color_hex = rgb_array_to_hex(color_rgb).lstrip("#")

        normalized = dict(slot_info)
        normalized["color_hex"] = color_hex

        spool_wt = slot_info.get("total_weight", 0)
        allow_create = self._get_auto_spoolman_create(lane)
        sync_rfid_to_spoolman(
            self.afc, lane, normalized, self.logger, "ACE RFID",
            allow_create=allow_create,
            spool_weight=spool_wt if spool_wt > 0 else None)

    def _get_auto_spoolman_create(self, lane):
        """Check if auto Spoolman spool creation is enabled for a lane."""
        if get_auto_spoolman_create is not None:
            return get_auto_spoolman_create(lane, self.auto_spoolman_create)
        return self.auto_spoolman_create


def load_config_prefix(config):
    return afcACE(config)
