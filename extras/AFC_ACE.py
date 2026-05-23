# Armored Turtle Automated Filament Control
#
# Copyright (C) 2024-2026 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.

"""AFC unit driver for Anycubic ACE PRO filament changers."""

from __future__ import annotations

import traceback
from configparser import Error as error
from typing import Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from extras.AFC_lane import AFCLane

try: from extras.AFC_utils import ERROR_STR
except: raise error("Error when trying to import AFC_utils.ERROR_STR\n{trace}".format(trace=traceback.format_exc()))

try: from extras.AFC_lane import AFCLaneState, SpeedMode, AssistActive
except: raise error(ERROR_STR.format(import_lib="AFC_lane", trace=traceback.format_exc()))

try: from extras.AFC_unit import afcUnit
except: raise error(ERROR_STR.format(import_lib="AFC_unit", trace=traceback.format_exc()))

try: from extras.AFC_ACE_serial import ACEConnection
except: raise error(ERROR_STR.format(import_lib="AFC_ACE_serial", trace=traceback.format_exc()))


MODE_COMBINED = "combined"
MODE_DIRECT = "direct"


class afcACE(afcUnit):
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

        self._ace: Optional[ACEConnection] = None
        self._slot_map: dict[str, int] = {}
        self._feed_assist_active: set[int] = set()

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

        # Connect to ACE hardware
        self._ace = ACEConnection(
            self.afc.reactor, self.serial_port,
            baud_rate=self.baud_rate,
            logger=self.logger)

        # Build slot map and set custom load/unload commands
        for lane_name, lane in self.lanes.items():
            slot = getattr(lane, 'index', 0)
            self._slot_map[lane_name] = slot
            lane.custom_load_cmd = f"_ACE_CUSTOM_LOAD UNIT={self.name} LANE={lane_name}"
            lane.custom_unload_cmd = f"_ACE_CUSTOM_UNLOAD UNIT={self.name} LANE={lane_name}"

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
        if not lane.load_to_hub or lane.loaded_to_hub:
            return
        slot = self._get_slot(lane.name)
        if not self._ace or not self._ace.connected:
            return
        if not self._ace.is_bay_ready(slot):
            return
        try:
            self._ace.feed_filament(slot, lane.dist_hub, self.feed_speed)
            self._wait_for_feed_complete()
            lane.loaded_to_hub = True
        except Exception as e:
            self.logger.error(f"ACE prep_post_load failed for {lane.name}: {e}")

    def eject_lane(self, lane: AFCLane):
        """Retract filament back into ACE unit."""
        slot = self._get_slot(lane.name)
        if self._ace and self._ace.connected:
            try:
                self._stop_feed_assist(slot)
                dist = self._get_unload_length(lane)
                self._ace.unwind_filament(slot, dist, self.retract_speed)
                self._wait_for_retract_complete()
            except Exception as e:
                self.logger.error(f"ACE eject failed for {lane.name}: {e}")

    def lane_move(self, cur_lane, distance, speed_mode):
        """Move filament via ACE serial instead of stepper."""
        slot = self._get_slot(cur_lane.name)
        if not self._ace or not self._ace.connected:
            self.logger.error("ACE not connected for lane_move")
            return
        try:
            if distance > 0:
                self._ace.feed_filament(slot, abs(distance), self.feed_speed)
            else:
                self._ace.unwind_filament(slot, abs(distance), self.retract_speed)
            self._wait_for_idle()
        except Exception as e:
            self.logger.error(f"ACE lane_move failed: {e}")

    def lane_unload(self, cur_lane):
        """Custom lane unload — retract via ACE serial."""
        slot = self._get_slot(cur_lane.name)
        if self._ace and self._ace.connected:
            try:
                self._stop_feed_assist(slot)
                dist = self._get_unload_length(cur_lane)
                self._ace.unwind_filament(slot, dist, self.retract_speed)
                self._wait_for_retract_complete()
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
        """ACE system test — verify hardware connection and slot status."""
        msg = ''
        succeeded = True

        if not self._ace or not self._ace.connected:
            msg = '<span class=error--text>ACE NOT CONNECTED</span>'
            succeeded = False
        else:
            slot = self._get_slot(cur_lane.name)
            if cur_lane.prep_state:
                self.lane_loaded(cur_lane)
                msg += "<span class=success--text>LOCKED</span>"
                if cur_lane.load_state:
                    cur_lane.status = AFCLaneState.LOADED
                    msg += "<span class=success--text> AND LOADED</span>"
                    self.lane_illuminate_spool(cur_lane)

                    if (cur_lane.tool_loaded
                        and cur_lane.extruder_obj.lane_loaded == cur_lane.name):
                        if cur_lane.get_toolhead_pre_sensor_state():
                            cur_lane.sync_to_extruder()
                            msg += "<span class=primary--text> in ToolHead</span>"
                            if self.afc.current == cur_lane.name:
                                self.afc.spool.set_active_spool(cur_lane.spool_id)
                                self.lane_tool_loaded(cur_lane)
                                cur_lane.status = AFCLaneState.TOOLED
                            else:
                                self.lane_tool_loaded_idle(cur_lane)
                            cur_lane.enable_buffer()
                else:
                    msg += "<span class=error--text> NOT LOADED</span>"
                    self.lane_not_ready(cur_lane)
                    succeeded = False
            else:
                if not cur_lane.load_state:
                    self.afc.function.afc_led(cur_lane.led_not_ready, cur_lane.led_index)
                    msg += 'EMPTY READY FOR SPOOL'
                else:
                    self.lane_fault(cur_lane)
                    msg += "<span class=error--text> NOT READY</span>"
                    msg += '<span class=error--text>CHECK FILAMENT Prep: False - Load: True</span>'
                    succeeded = False

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
            self._ace.feed_filament(slot, step, self.feed_speed)
            self._wait_for_feed_complete()
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
        self._ace.unwind_filament(slot, total_fed, self.retract_speed)
        self._wait_for_retract_complete()

        return True, "afc_bowden_length calibration successful", bowden_dist

    def calibrate_lane(self, cur_lane, tol):
        """Calibrate dist_hub from ACE slot to hub sensor."""
        return self._calibrate_hub_inner(cur_lane)

    # ── Custom load/unload gcode handlers ───────────────────────────

    def _cmd_ace_custom_load(self, gcmd):
        """Handle _ACE_CUSTOM_LOAD gcode command."""
        lane_name = gcmd.get('LANE')
        if lane_name not in self.afc.lanes:
            self.logger.error(f"Unknown lane: {lane_name}")
            return
        cur_lane = self.afc.lanes[lane_name]
        slot = self._get_slot(lane_name)

        if not self._ace or not self._ace.connected:
            self.logger.error("ACE not connected")
            return

        # In combined mode, retract any other loaded slot first
        if self.mode == MODE_COMBINED:
            for other_name, other_slot in self._slot_map.items():
                if other_slot != slot and other_slot in self._feed_assist_active:
                    self._stop_feed_assist(other_slot)

        # Calculate effective feed distance using upstream variables:
        # dist_hub (lane→hub) + afc_bowden_length (hub→toolhead)
        if cur_lane.loaded_to_hub:
            hub = cur_lane.hub_obj
            feed_dist = getattr(hub, 'afc_bowden_length', 0) if hub else 0
            feed_dist = max(feed_dist, 50.0)
        else:
            feed_dist = self._get_bowden_length(cur_lane)

        # Set virtual hub sensor state
        if cur_lane.hub_obj and cur_lane.hub_obj.is_virtual_pin():
            cur_lane.loaded_to_hub = True

        # Feed filament
        try:
            self._ace.feed_filament(slot, feed_dist, self.feed_speed)
            success = self._wait_for_feed_complete()

            if not success:
                # Check if filament reached despite error
                if cur_lane.get_toolhead_pre_sensor_state():
                    self.logger.info("Feed reported error but sensor triggered — continuing")
                else:
                    # Retry with incremental feeds
                    success = self._smart_load_retry(cur_lane, slot)
                    if not success:
                        self.logger.error(f"ACE load failed for {lane_name}")
                        return

            # Enable feed assist
            if self._use_feed_assist(cur_lane):
                self._start_feed_assist(slot)

        except Exception as e:
            self.logger.error(f"ACE load error for {lane_name}: {e}")

    def _cmd_ace_custom_unload(self, gcmd):
        """Handle _ACE_CUSTOM_UNLOAD gcode command."""
        lane_name = gcmd.get('LANE')
        if lane_name not in self.afc.lanes:
            self.logger.error(f"Unknown lane: {lane_name}")
            return
        cur_lane = self.afc.lanes[lane_name]
        slot = self._get_slot(lane_name)

        if not self._ace or not self._ace.connected:
            self.logger.error("ACE not connected")
            return

        try:
            # Stop feed assist
            self._stop_feed_assist(slot)

            # Disable buffer
            cur_lane.disable_buffer()

            # ACE rewind — full retract using upstream distance variables
            retract_dist = self._get_unload_length(cur_lane)
            self._ace.unwind_filament(slot, retract_dist, self.retract_speed)
            self._wait_for_retract_complete()

            # Clear hub state but keep filament staged near hub
            if cur_lane.hub_obj:
                hub = cur_lane.hub_obj
                if not hub.is_virtual_pin() and hasattr(hub, 'state') and not hub.state:
                    cur_lane.loaded_to_hub = False
                else:
                    cur_lane.loaded_to_hub = True

        except Exception as e:
            self.logger.error(f"ACE unload error for {lane_name}: {e}")

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

    def _wait_for_feed_complete(self, timeout: float = 60.0) -> bool:
        """Wait for ACE feed operation to complete."""
        if not self._ace:
            return False
        deadline = self.afc.reactor.monotonic() + timeout
        while self.afc.reactor.monotonic() < deadline:
            if not self._ace.is_busy():
                return True
            self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.5)
        self.logger.error("ACE feed timed out")
        return False

    def _wait_for_retract_complete(self, timeout: float = 60.0) -> bool:
        """Wait for ACE retract operation to complete."""
        return self._wait_for_feed_complete(timeout)

    def _wait_for_idle(self, timeout: float = 30.0) -> bool:
        """Wait for ACE to become idle."""
        return self._wait_for_feed_complete(timeout)

    def _smart_load_retry(self, cur_lane, slot, max_retries: int = 3) -> bool:
        """Retry loading with incremental feeds when initial feed fails."""
        for attempt in range(max_retries):
            self.logger.info(f"Smart load retry {attempt+1}/{max_retries} for {cur_lane.name}")
            try:
                # Small incremental feed
                self._ace.feed_filament(slot, self.sensor_step, self.feed_speed)
                self._wait_for_feed_complete()
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
            self._ace.feed_filament(slot, step, self.feed_speed)
            self._wait_for_feed_complete()
            total_fed += step
            self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.2)

        new_dist = round(total_fed, 2)
        cal_msg = f'\n dist_hub: New: {new_dist} Old: {cur_lane.dist_hub}'
        cur_lane.dist_hub = new_dist
        self.afc.function.ConfigRewrite(cur_lane.fullname, "dist_hub", new_dist, cal_msg)

        # Retract to clear hub
        backoff = getattr(hub, 'hub_clear_move_dis', 50.0)
        self._ace.unwind_filament(slot, backoff, self.retract_speed)
        self._wait_for_retract_complete()

        return True, "dist_hub calibration successful", new_dist


def load_config_prefix(config):
    return afcACE(config)
