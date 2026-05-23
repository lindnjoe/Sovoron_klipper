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

        self.gcode = self.printer.lookup_object('gcode')
        cmd_prefix = self.name.upper().replace(" ", "_")
        self.gcode.register_command(
            f'_ACE_CUSTOM_LOAD', self._cmd_ace_custom_load,
            desc="ACE internal load command")
        self.gcode.register_command(
            f'_ACE_CUSTOM_UNLOAD', self._cmd_ace_custom_unload,
            desc="ACE internal unload command")
        self.gcode.register_command(
            'ACE_CALIBRATE', self.cmd_ACE_CALIBRATE,
            desc="Calibrate ACE feed distance to toolhead")
        self.gcode.register_command(
            'ACE_CALIBRATE_HUB', self.cmd_ACE_CALIBRATE_HUB,
            desc="Calibrate ACE feed distance to hub")
        self.gcode.register_command(
            'ACE_STATUS', self.cmd_ACE_STATUS,
            desc="Query ACE hardware status")
        self.gcode.register_command(
            'ACE_DRY', self.cmd_ACE_DRY,
            desc="Start ACE filament dryer")
        self.gcode.register_command(
            'ACE_LANE_RESET', self.cmd_ACE_LANE_RESET,
            desc="Retract ACE lane filament back into unit")

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
            lane.custom_load_cmd = f"_ACE_CUSTOM_LOAD UNIT={self.name} LANE={lane_name}"
            lane.custom_unload_cmd = f"_ACE_CUSTOM_UNLOAD UNIT={self.name} LANE={lane_name}"

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

    def _sync_slot_states(self, hw_status):
        """Sync lane prep/load state from ACE hardware slot status."""
        slots = hw_status.get("slots", [])
        for lane in self.lanes.values():
            slot = self._get_slot(lane.name)
            if slot >= len(slots) or not isinstance(slots[slot], dict):
                continue

            slot_status = slots[slot].get("status", "")
            slot_ready = slot_status == "ready"
            slot_transient = slot_status in ("shifting", "feeding", "unwinding")

            # Update inventory status cache
            if slot < self.SLOTS_PER_UNIT:
                self._slot_inventory[slot]["status"] = slot_status

            if slot_transient:
                continue

            prev_ready = getattr(lane, '_prev_slot_ready', None)
            lane._load_state = slot_ready
            lane.prep_state = slot_ready

            # For virtual hub pins, load_state returns loaded_to_hub.
            # Keep it in sync with the hardware slot state.
            if self._is_virtual_hub(lane):
                if slot_ready and lane.status in (AFCLaneState.LOADED, AFCLaneState.TOOLED):
                    lane.loaded_to_hub = True
                elif not slot_ready:
                    lane.loaded_to_hub = False

            prep_done = getattr(lane, '_afc_prep_done', False)

            # Filament inserted: slot went from not-ready to ready
            if slot_ready and prev_ready is False and prep_done:
                if lane.status == AFCLaneState.NONE:
                    self.logger.info(f"ACE: {lane.name} filament inserted")
                    self.afc.spool.clear_values(lane)
                    lane.set_loaded()
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

            # Filament removed: slot went from ready to not-ready
            if not slot_ready and prev_ready is True and prep_done:
                if lane.status not in (AFCLaneState.NONE,):
                    self.logger.info(f"ACE: {lane.name} filament removed")
                    if slot < self.SLOTS_PER_UNIT:
                        self._clear_slot_inventory(slot)
                    if getattr(lane, 'tool_loaded', False):
                        try:
                            is_printing = self.afc.function.is_printing()
                        except Exception:
                            is_printing = False
                        if is_printing:
                            self.afc.error.AFC_error(
                                f"ACE runout on {lane.name}", pause=True)
                    else:
                        lane.loaded_to_hub = False
                        lane.set_unloaded()
                        self.lane_not_ready(lane)
                        self.afc.save_vars()

            lane._prev_slot_ready = slot_ready

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

    def calibrate_bowden(self, cur_lane, dis, tol):
        """Calibrate afc_bowden_length (hub→toolhead) by feeding from hub until
        the toolhead sensor triggers, matching the upstream BoxTurtle pattern."""
        slot = self._get_slot(cur_lane.name)
        if not self._ace or not self._ace.connected:
            return False, "ACE not connected", 0

        cur_hub = cur_lane.hub_obj
        if cur_hub is None:
            return False, "Lane has no hub configured", 0

        self.logger.raw(f'Calibrating afc_bowden_length for {cur_lane.name}')
        total_fed = 0.0
        step = self.calibration_step
        old_bowden = getattr(cur_hub, 'afc_bowden_length', 0)
        max_distance = old_bowden + self.max_feed_overshoot

        while not cur_lane.get_toolhead_pre_sensor_state():
            if total_fed >= max_distance:
                msg = f'Filament did not reach toolhead sensor after {total_fed}mm'
                return False, msg, total_fed
            self._wait_for_ace_ready()
            self._ace.feed_filament(slot, step, self.feed_speed)
            self._wait_for_feed_complete(slot, step, self.feed_speed)
            total_fed += step
            self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.2)

        bowden_dist = round(total_fed, 2)
        cal_msg = f'\n afc_bowden_length: New: {bowden_dist} Old: {old_bowden}'
        cur_hub.afc_bowden_length = bowden_dist
        cur_hub.afc_unload_bowden_length = bowden_dist
        self.afc.function.ConfigRewrite(cur_hub.fullname, "afc_bowden_length", bowden_dist, cal_msg)
        self.afc.function.ConfigRewrite(cur_hub.fullname, "afc_unload_bowden_length", bowden_dist,
                                         f'\n afc_unload_bowden_length: {bowden_dist}')

        # Retract back
        self._wait_for_ace_ready()
        self._ace.unwind_filament(slot, total_fed, self.retract_speed)
        self._wait_for_feed_complete(slot, total_fed, self.retract_speed)

        return True, "afc_bowden_length calibration successful", bowden_dist

    def calibrate_lane(self, cur_lane, tol):
        """Calibrate dist_hub from ACE slot to hub sensor."""
        return self._calibrate_hub_inner(cur_lane)

    # ── Custom load/unload gcode handlers ───────────────────────────

    def _cmd_ace_custom_load(self, gcmd):
        """Handle _ACE_CUSTOM_LOAD — full load sequence with toolhead ops."""
        lane_name = gcmd.get('LANE')
        cur_lane = self.afc.lanes.get(lane_name)
        if cur_lane is None:
            raise gcmd.error(f"Unknown lane: {lane_name}")
        cur_extruder = cur_lane.extruder_obj
        result = self._ace_load_sequence(cur_lane, cur_extruder)
        if not result:
            raise gcmd.error(f"ACE load failed for {lane_name}")

    def _cmd_ace_custom_unload(self, gcmd):
        """Handle _ACE_CUSTOM_UNLOAD — full unload sequence with toolhead ops."""
        lane_name = gcmd.get('LANE')
        cur_lane = self.afc.lanes.get(lane_name)
        if cur_lane is None:
            raise gcmd.error(f"Unknown lane: {lane_name}")
        cur_extruder = cur_lane.extruder_obj
        result = self._ace_unload_sequence(cur_lane, cur_extruder)
        if not result:
            raise gcmd.error(f"ACE unload failed for {lane_name}")

    def _ace_load_sequence(self, cur_lane, cur_extruder) -> bool:
        """Full ACE load: heat → ACE serial feed → extruder load → feed assist."""
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

        # Heat extruder
        if afc._check_extruder_temp(cur_lane):
            afc.afcDeltaTime.log_with_time("Done heating toolhead")

        # Calculate feed distance
        if cur_lane.loaded_to_hub:
            hub = cur_lane.hub_obj
            feed_dist = getattr(hub, 'afc_bowden_length', 0) if hub else 0
            feed_dist = max(feed_dist, 50.0)
        else:
            feed_dist = self._get_bowden_length(cur_lane)

        if self._is_virtual_hub(cur_lane):
            cur_lane.loaded_to_hub = True

        # ACE serial feed to toolhead area
        try:
            self._wait_for_ace_ready()
            self._ace.feed_filament(slot, feed_dist, self.feed_speed)
            success = self._wait_for_feed_complete(slot, feed_dist, self.feed_speed)

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

        # Sync to extruder and load into toolhead
        cur_lane.status = AFCLaneState.TOOL_LOADED
        afc.save_vars()
        cur_lane.sync_to_extruder()

        # Extruder load (tool_stn)
        if cur_extruder.tool_stn > 0:
            afc.move_e_pos(cur_extruder.tool_stn, cur_extruder.tool_load_speed, "tool stn")
            afc.toolhead.wait_moves()

        # Enable buffer
        cur_lane.enable_buffer()

        # Enable feed assist
        if self._use_feed_assist(cur_lane):
            self._start_feed_assist(slot)

        afc.afcDeltaTime.log_with_time("ACE load complete")
        return True

    def _ace_unload_sequence(self, cur_lane, cur_extruder) -> bool:
        """Full ACE unload: heat → cut → park → tip → retract extruder → ACE serial unwind."""
        afc = self.afc
        slot = self._get_slot(cur_lane.name)

        if not self._ace or not self._ace.connected:
            afc.error.handle_lane_failure(
                cur_lane, f"ACE not connected ({self.serial_port})")
            return False

        # Stop feed assist
        self._stop_feed_assist(slot)

        # Disable buffer
        cur_lane.disable_buffer()

        # LED animation
        self.lane_unloading(cur_lane)

        # Heat extruder
        if afc._check_extruder_temp(cur_lane):
            afc.afcDeltaTime.log_with_time("Done heating toolhead")

        # Quick pull to prevent oozing
        afc.move_e_pos(-2, cur_extruder.tool_unload_speed, "Quick Pull", wait_tool=False)

        # Sync to extruder for cut/park/tip operations
        cur_lane.sync_to_extruder()
        cur_lane.do_enable(True)

        # Cut
        if afc.tool_cut:
            cur_lane.extruder_obj.estats.increase_cut_total()
            afc.gcode.run_script_from_command(
                "{} EXTRUDER={}".format(afc.tool_cut_cmd, cur_extruder.name))
            if afc.park:
                afc.gcode.run_script_from_command(
                    "{} EXTRUDER={}".format(afc.park_cmd, cur_extruder.name))

        # Form tip
        if afc.form_tip:
            if afc.park:
                afc.gcode.run_script_from_command(
                    "{} EXTRUDER={}".format(afc.park_cmd, cur_extruder.name))
            if afc.form_tip_cmd == "AFC":
                tip = afc.printer.lookup_object('AFC_form_tip')
                tip.tip_form()
            else:
                afc.gcode.run_script_from_command(afc.form_tip_cmd)

        # Retract from extruder (tool_stn_unload via extruder motor)
        if cur_extruder.tool_stn_unload > 0:
            afc.move_e_pos(
                cur_extruder.tool_stn_unload * -1,
                cur_extruder.tool_unload_speed,
                "Retract from extruder", wait_tool=True)

        # Unsync from extruder
        cur_lane.unsync_to_extruder()

        afc.afcDeltaTime.log_with_time("Toolhead operations complete")

        # ACE serial unwind — retract bowden length
        retract_dist = self._get_unload_length(cur_lane)
        try:
            self._wait_for_ace_ready()
            self._ace.unwind_filament(slot, retract_dist, self.retract_speed)
            self._wait_for_feed_complete(slot, retract_dist, self.retract_speed)
        except Exception as e:
            afc.error.handle_lane_failure(
                cur_lane, f"ACE unwind failed for {cur_lane.name}: {e}")
            return False

        afc.afcDeltaTime.log_with_time("ACE unwind complete")

        # Finalize state — filament staged near hub
        cur_lane.set_tool_unloaded()
        cur_lane.loaded_to_hub = True
        self.lane_tool_unloaded(cur_lane)
        afc.save_vars()

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

        self.logger.raw(f'Calibrating dist_hub for {cur_lane.name}')
        total_fed = 0.0
        step = self.sensor_step
        max_distance = cur_lane.dist_hub + 200.0

        while not hub.state:
            if total_fed >= max_distance:
                msg = f'Filament did not reach hub sensor after {total_fed}mm'
                return False, msg, total_fed
            self._wait_for_ace_ready()
            self._ace.feed_filament(slot, step, self.feed_speed)
            self._wait_for_feed_complete(slot, step, self.feed_speed)
            total_fed += step
            self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.2)

        new_dist = round(total_fed, 2)
        cal_msg = f'\n dist_hub: New: {new_dist} Old: {cur_lane.dist_hub}'
        cur_lane.dist_hub = new_dist
        self.afc.function.ConfigRewrite(cur_lane.fullname, "dist_hub", new_dist, cal_msg)

        # Retract to clear hub
        self._wait_for_ace_ready()
        backoff = getattr(hub, 'hub_clear_move_dis', 50.0)
        self._ace.unwind_filament(slot, backoff, self.retract_speed)
        self._wait_for_feed_complete(slot, backoff, self.retract_speed)

        return True, "dist_hub calibration successful", new_dist


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
