# Armored Turtle Automated Filament Control
#
# Copyright (C) 2024-2026 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.

"""AFC unit driver for OpenAMS filament changers with stuck spool,
clog detection, and engagement verification."""

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

try:
    from extras.AFC_OpenAMS_follower import FollowerController
except Exception:
    FollowerController = None

try:
    from extras.AFC_OpenAMS_monitor import OAMSMonitor
except Exception:
    OAMSMonitor = None



class afcAMS(afcUnit):
    """OpenAMS unit type — supports engagement verification, stuck spool
    and clog detection via FPS + encoder monitoring."""

    def __init__(self, config):
        super().__init__(config)
        self.type = config.get('type', 'OpenAMS')

        # Hardware identifier
        self.oams_name = config.get("oams", "oams1")

        # Stuck spool detection
        self.stuck_spool_auto_recovery = config.getboolean("stuck_spool_auto_recovery", False)
        self.stuck_spool_load_grace = config.getfloat("stuck_spool_load_grace", 8.0, minval=0.0)
        self.stuck_spool_pressure_threshold = config.getfloat(
            "stuck_spool_pressure_threshold", 0.08, minval=0.0)

        # Engagement verification
        self._engagement_pressure_max = config.getfloat(
            "engagement_pressure_threshold", 0.6, minval=0.0)
        self._engagement_pressure_min = config.getfloat(
            "engagement_min_pressure", 0.25, minval=0.0)

        # Clog detection
        self.clog_sensitivity = config.get("clog_sensitivity", "medium").lower()

        # Engagement params — unit-level defaults
        self._engagement_length = config.getfloat("engagement_length", 20.0, minval=1.0)
        self._engagement_speed = config.getfloat("engagement_speed", 300.0, minval=10.0)

        # Runtime state
        self.oams = None
        self._follower: Optional[FollowerController] = None
        self._monitor: Optional[OAMSMonitor] = None
        self._spool_map: dict[str, int] = {}

        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command(
            '_OAMS_CUSTOM_LOAD', self._cmd_oams_custom_load,
            desc="OpenAMS internal load command")
        self.gcode.register_command(
            '_OAMS_CUSTOM_UNLOAD', self._cmd_oams_custom_unload,
            desc="OpenAMS internal unload command")
        self.gcode.register_command(
            'AFC_OAMS_CALIBRATE_PTFE', self.cmd_AFC_OAMS_CALIBRATE_PTFE,
            desc="Calibrate OpenAMS PTFE length")
        self.gcode.register_command(
            'AFC_OAMS_CALIBRATE_HUB_HES', self.cmd_AFC_OAMS_CALIBRATE_HUB_HES,
            desc="Calibrate OpenAMS hub HES for a spool")
        self.gcode.register_command(
            'AFC_OAMS_CALIBRATE_HUB_HES_ALL', self.cmd_AFC_OAMS_CALIBRATE_HUB_HES_ALL,
            desc="Calibrate all loaded OpenAMS hub HES sensors")
        self.gcode.register_command(
            'AFC_OAMS_CLEAR_ERRORS', self.cmd_AFC_OAMS_CLEAR_ERRORS,
            desc="Clear OpenAMS errors and resync state")

    def handle_connect(self):
        super().handle_connect()

        try:
            from extras.temperature_oams import _register_sensor_factory
            _register_sensor_factory(self.printer)
        except Exception:
            pass

        self.logo = '<span class=success--text>R  OpenAMS\n'
        self.logo += 'E  ========\n'
        self.logo += 'A  |      |\n'
        self.logo += 'D  | OAMS |\n'
        self.logo += 'Y  ========</span>\n'
        self.logo += '  ' + self.name + '\n'

        self.logo_error = '<span class=error--text>E  OpenAMS\n'
        self.logo_error += 'R  ========\n'
        self.logo_error += 'R  | ERR  |\n'
        self.logo_error += 'O  |  X   |\n'
        self.logo_error += 'R  ========</span>\n'
        self.logo_error += '  ' + self.name + '\n'

        # Resolve OAMS hardware object
        try:
            self.oams = self.printer.lookup_object(f"oams {self.oams_name}", None)
        except Exception:
            self.oams = None

        # Build spool map and set custom load/unload commands
        for lane_name, lane in self.lanes.items():
            slot = getattr(lane, 'index', 0)
            self._spool_map[lane_name] = slot
            lane.custom_load_cmd = f"_OAMS_CUSTOM_LOAD UNIT={self.name} LANE={lane_name}"
            lane.custom_unload_cmd = f"_OAMS_CUSTOM_UNLOAD UNIT={self.name} LANE={lane_name}"

        # Initialize follower and monitor subsystems
        self._init_follower_and_monitor()

    def _init_follower_and_monitor(self):
        """Set up follower motor controller and stuck/clog monitor."""
        if self.oams is None:
            return

        fps_obj = None
        fps_name = None
        # Look for FPS buffer attached to any lane in this unit
        for lane in self.lanes.values():
            buf = getattr(lane, 'buffer_obj', None)
            if buf and hasattr(buf, 'get_fps_value'):
                fps_obj = buf
                fps_name = getattr(buf, 'name', 'fps')
                break

        if FollowerController is not None and self.oams is not None:
            try:
                self._follower = FollowerController(
                    self.oams, self.printer, self.logger)
            except Exception as e:
                self.logger.error(f"Failed to init follower: {e}")

        if OAMSMonitor is not None and fps_obj is not None:
            try:
                self._monitor = OAMSMonitor(
                    fps_name, fps_obj,
                    self.afc.reactor, self.logger,
                    stuck_pressure_low=self.stuck_spool_pressure_threshold,
                    stuck_load_grace=self.stuck_spool_load_grace,
                    clog_sensitivity=self.clog_sensitivity,
                    on_stuck_spool=self._on_stuck_spool_detected,
                    on_clog=self._on_clog_detected,
                    on_stuck_cleared=self._on_stuck_spool_cleared)
            except Exception as e:
                self.logger.error(f"Failed to init monitor: {e}")

    # ── Engagement verification ─────────────────────────────────────

    def get_engagement_params(self, lane_name: str) -> tuple:
        """Return (engagement_length, engagement_speed) for a lane."""
        return (self._engagement_length, self._engagement_speed)

    def _verify_engagement(self, cur_lane) -> bool:
        """Verify filament engagement by extruding and checking encoder/FPS movement."""
        engagement_length, engagement_speed = self.get_engagement_params(cur_lane.name)

        self.logger.info(
            f"Verifying engagement for {cur_lane.name}: "
            f"{engagement_length:.1f}mm at {engagement_speed:.0f}mm/min")

        if self._monitor:
            self._monitor.notify_engagement_start()

        try:
            # Record encoder before
            encoder_before = 0
            if self.oams and hasattr(self.oams, 'encoder_clicks'):
                encoder_before = self.oams.encoder_clicks or 0

            # Wait for FPS pressure to normalize
            self.afc.reactor.pause(self.afc.reactor.monotonic() + 1.5)

            # Two-phase extrude: prime then main
            prime_length = min(5.0, engagement_length)
            remaining = max(0.0, engagement_length - prime_length)

            self._oams_extrude(prime_length, engagement_speed, "engagement_prime")
            if remaining > 0:
                self._oams_extrude(remaining, engagement_speed, "engagement_main")

            # Check encoder movement
            encoder_after = 0
            if self.oams and hasattr(self.oams, 'encoder_clicks'):
                encoder_after = self.oams.encoder_clicks or 0

            encoder_delta = abs(encoder_after - encoder_before)
            min_movement = 2

            if encoder_delta >= min_movement:
                self.logger.info(
                    f"Engagement verified: encoder moved {encoder_delta} clicks")
                return True

            # Brief retry
            self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.3)
            encoder_retry = 0
            if self.oams and hasattr(self.oams, 'encoder_clicks'):
                encoder_retry = abs((self.oams.encoder_clicks or 0) - encoder_before)
            if encoder_retry >= min_movement:
                self.logger.info(
                    f"Engagement verified on retry: encoder moved {encoder_retry} clicks")
                return True

            self.logger.error(
                f"Engagement verification failed: encoder moved only {encoder_delta} clicks")
            return False

        finally:
            if self._monitor:
                self._monitor.notify_engagement_end()

    def _oams_extrude(self, length: float, speed: float, label: str = ""):
        """Extrude filament via gcode for engagement/retraction."""
        self.afc.gcode.run_script_from_command(
            f"G92 E0\nG1 E{length:.3f} F{speed:.0f}\nM400")

    # ── Stuck spool / clog detection callbacks ──────────────────────

    def _on_stuck_spool_detected(self, lane_name: str = None):
        """Called by OAMSMonitor when stuck spool detected during print."""
        msg = f"OpenAMS stuck spool detected"
        if lane_name:
            msg += f" on {lane_name}"

        if self.stuck_spool_auto_recovery:
            self.logger.info(f"{msg} — attempting auto recovery")
            # TODO: implement auto-recovery (unload + reload + resume)
        else:
            msg += ". Print paused — check spool and resume."
            self.afc.error.AFC_error(msg, pause=True)

    def _on_clog_detected(self, lane_name: str = None):
        """Called by OAMSMonitor when clog detected during print."""
        msg = "OpenAMS clog detected"
        if lane_name:
            msg += f" on {lane_name}"
        msg += ". Print paused — check filament path."
        self.afc.error.AFC_error(msg, pause=True)

    def _on_stuck_spool_cleared(self, lane_name: str = None):
        """Called by OAMSMonitor when stuck spool condition clears."""
        self.logger.info(f"Stuck spool cleared{' for ' + lane_name if lane_name else ''}")

    # ── Unit interface overrides ────────────────────────────────────

    def prep_load(self, lane: AFCLane):
        """No-op: OpenAMS firmware drives filament to load sensor directly."""
        pass

    def prep_post_load(self, lane: AFCLane):
        """No-op: OpenAMS handles hub staging internally."""
        pass

    def eject_lane(self, lane: AFCLane):
        """OpenAMS does not support stepper-based eject."""
        self.logger.info(
            f"Eject not supported for OpenAMS lane {lane.name}. "
            "Remove spool physically or use TOOL_UNLOAD.")

    def lane_move(self, cur_lane, distance, speed_mode):
        """OpenAMS has no stepper — log warning."""
        self.logger.info(
            f"Lane move not supported for OpenAMS lane {cur_lane.name}. "
            "OpenAMS firmware controls filament movement.")

    def lane_unload(self, cur_lane):
        """Custom lane unload via OAMS hardware."""
        if self.oams is None:
            return None
        try:
            self.oams.unload_spool_with_retry()
            self._wait_for_idle()
        except Exception as e:
            self.logger.error(f"OpenAMS lane_unload failed: {e}")
        return True

    def abort_load(self, cur_lane):
        """Cancel in-progress OpenAMS load."""
        if self.oams is not None:
            try:
                self.oams.abort_current_action(wait=True)
            except Exception:
                pass

    def get_lane_reset_command(self, lane, dis):
        """OpenAMS lanes don't support distance-based reset."""
        return None

    def get_current_lane_fallback(self, tool_obj):
        """Return the loaded lane name for this unit's extruder when on_shuttle is False."""
        for lane_name, lane in self.lanes.items():
            if getattr(lane, 'tool_loaded', False):
                return lane_name
        return None

    def on_lane_unset_loaded(self, lane, extruder_name):
        """Cleanup after lane is unset — stop follower, update state."""
        if self._follower is not None:
            try:
                fps_state = self._get_monitor_state()
                if fps_state and self.oams:
                    self._follower.set_follower_state(
                        fps_state, self.oams, 0, 0, "lane unset", force=True)
            except Exception:
                pass

    def system_Test(self, cur_lane, delay, assignTcmd, enable_movement):
        """OpenAMS system test — check hardware and lane state."""
        msg = ''
        succeeded = True

        if self.oams is None:
            msg = '<span class=error--text>OAMS NOT CONNECTED</span>'
            succeeded = False
        else:
            slot = self._spool_map.get(cur_lane.name, 0)
            if cur_lane.prep_state:
                self.lane_loaded(cur_lane)
                msg += "<span class=success--text>LOCKED</span>"
                if cur_lane.load_state:
                    cur_lane.status = AFCLaneState.LOADED
                    msg += "<span class=success--text> AND LOADED</span>"
                    self.lane_illuminate_spool(cur_lane)

                    if (cur_lane.tool_loaded
                        and cur_lane.extruder_obj.lane_loaded == cur_lane.name):
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
        self._print_function_not_defined("calibrate_bowden")
        return False, "Bowden calibration not supported for OpenAMS", 0

    def calibrate_lane(self, cur_lane, tol):
        self._print_function_not_defined("calibrate_lane")
        return False, "Lane calibration not supported for OpenAMS", 0

    # ── Custom load/unload gcode handlers ───────────────────────────

    def _cmd_oams_custom_load(self, gcmd):
        """Handle _OAMS_CUSTOM_LOAD gcode command."""
        lane_name = gcmd.get('LANE')
        if lane_name not in self.afc.lanes:
            self.logger.error(f"Unknown lane: {lane_name}")
            return
        cur_lane = self.afc.lanes[lane_name]
        self._oams_load(cur_lane)

    def _cmd_oams_custom_unload(self, gcmd):
        """Handle _OAMS_CUSTOM_UNLOAD gcode command."""
        lane_name = gcmd.get('LANE')
        if lane_name not in self.afc.lanes:
            self.logger.error(f"Unknown lane: {lane_name}")
            return
        cur_lane = self.afc.lanes[lane_name]
        self._oams_unload(cur_lane)

    def _oams_load(self, cur_lane, max_retries: int = 3) -> bool:
        """Load filament via OpenAMS hardware with engagement verification."""
        if self.oams is None:
            self.logger.error("OAMS hardware not available")
            return False

        spool_index = self._spool_map.get(cur_lane.name, 0)

        # Wait for OAMS idle
        self._wait_for_idle()

        # Stop monitor during load to prevent false positives
        if self._monitor:
            self._monitor.stop()

        # Enable follower forward before load
        if self._follower:
            fps_state = self._get_monitor_state()
            if fps_state:
                self._follower.enable_follower(
                    fps_state, self.oams, 1, "before load", force=True)

        for attempt in range(max_retries):
            try:
                success, msg = self.oams.load_spool_with_retry(spool_index)
                if not success:
                    self.logger.error(f"OAMS load attempt {attempt+1} failed: {msg}")
                    continue

                # Verify engagement
                engaged = self._verify_engagement(cur_lane)
                if engaged:
                    # Enable follower and start monitor
                    if self._follower:
                        fps_state = self._get_monitor_state()
                        if fps_state:
                            self._follower.enable_follower(
                                fps_state, self.oams, 1, "load complete", force=True)

                    if self._monitor:
                        self._monitor.notify_load_complete(
                            cur_lane.name, self.oams_name, spool_index)
                        self._monitor.start(self.oams)

                    # Set virtual hub sensor
                    cur_lane.loaded_to_hub = True

                    return True

                # Engagement failed — clean up and retry
                self.logger.info(
                    f"Engagement failed attempt {attempt+1}, cleaning up")

                if self._follower:
                    fps_state = self._get_monitor_state()
                    if fps_state:
                        self._follower.set_follower_state(
                            fps_state, self.oams, 1, 0,
                            "engagement cleanup", force=True)

                # Retract from extruder
                unload_length, _ = self.get_engagement_params(cur_lane.name)
                self._oams_extrude(
                    -(unload_length + 10.0), 1500, "engagement_cleanup_retract")

                # Hardware cleanup
                self.oams.abort_current_action(wait=True)
                self.oams.unload_spool_with_retry()
                self._wait_for_idle()

                if attempt < max_retries - 1:
                    self.afc.reactor.pause(self.afc.reactor.monotonic() + 2.0)

            except Exception as e:
                self.logger.error(f"OAMS load error attempt {attempt+1}: {e}")

        self.logger.error(f"OAMS load failed after {max_retries} attempts")
        return False

    def _oams_unload(self, cur_lane) -> bool:
        """Unload filament via OpenAMS hardware."""
        if self.oams is None:
            self.logger.error("OAMS hardware not available")
            return False

        spool_index = self._spool_map.get(cur_lane.name, 0)

        # Set follower reverse for retraction assist
        if self._follower:
            fps_state = self._get_monitor_state()
            if fps_state:
                self._follower.set_follower_state(
                    fps_state, self.oams, 1, 0,
                    "before unload", force=True)

        # Wait for idle
        self._wait_for_idle()

        # Stop monitor during unload
        if self._monitor:
            self._monitor.stop()

        try:
            # Retract from extruder gears
            unload_length, _ = self.get_engagement_params(cur_lane.name)
            self._oams_extrude(
                -(unload_length + 10.0), 1500, "unload_retract")

            # Queue concurrent extruder retract
            self.afc.gcode.run_script_from_command("G92 E0\nG1 E-20 F1500")

            # Hardware unload
            success, msg = self.oams.unload_spool_with_retry()
            self._wait_for_idle()

            if self._monitor:
                self._monitor.notify_unload_complete()

            # Update hub state from hardware sensor
            if hasattr(self.oams, 'hub_hes_value'):
                hub_values = self.oams.hub_hes_value
                if hub_values and spool_index < len(hub_values):
                    cur_lane.loaded_to_hub = bool(hub_values[spool_index])

                    # Update virtual hub sensor
                    hub = cur_lane.hub_obj
                    if hub and hub.is_virtual_pin():
                        try:
                            eventtime = self.afc.reactor.monotonic()
                            hub.switch_pin_callback(eventtime, cur_lane.loaded_to_hub)
                        except Exception:
                            pass

            return success

        except Exception as e:
            self.logger.error(f"OAMS unload error: {e}")
            return False

    # ── Calibration commands ────────────────────────────────────────

    def cmd_AFC_OAMS_CALIBRATE_PTFE(self, gcmd):
        """Calibrate PTFE length for OpenAMS unit."""
        if self.oams is None:
            gcmd.respond_info("OAMS hardware not available")
            return
        spool = gcmd.get_int('SPOOL', 0)
        try:
            oams_idx = self._get_oams_index()
            self.afc.gcode.run_script_from_command(
                f"OAMS_CALIBRATE_PTFE_LENGTH OAMS={oams_idx} SPOOL={spool}")
            gcmd.respond_info("PTFE calibration complete — check OAMS config for new value")
        except Exception as e:
            gcmd.respond_info(f"PTFE calibration failed: {e}")

    def cmd_AFC_OAMS_CALIBRATE_HUB_HES(self, gcmd):
        """Calibrate hub HES sensor for a specific spool bay."""
        if self.oams is None:
            gcmd.respond_info("OAMS hardware not available")
            return
        spool = gcmd.get_int('SPOOL', None)
        if spool is None:
            gcmd.respond_info("Usage: AFC_OAMS_CALIBRATE_HUB_HES SPOOL=<index>")
            return
        success = self._calibrate_hub_hes_spool(spool)
        gcmd.respond_info(
            f"Hub HES calibration {'successful' if success else 'failed'} for spool {spool}")

    def cmd_AFC_OAMS_CALIBRATE_HUB_HES_ALL(self, gcmd):
        """Calibrate hub HES for all loaded spool bays."""
        if self.oams is None:
            gcmd.respond_info("OAMS hardware not available")
            return
        count = 0
        for lane_name, lane in self.lanes.items():
            if lane.load_state:
                spool_idx = self._spool_map.get(lane_name, 0)
                if self._calibrate_hub_hes_spool(spool_idx):
                    count += 1
        gcmd.respond_info(f"Calibrated {count} hub HES sensor(s)")

    def cmd_AFC_OAMS_CLEAR_ERRORS(self, gcmd):
        """Clear OpenAMS errors and resync state with hardware."""
        if self.oams is None:
            gcmd.respond_info("OAMS hardware not available")
            return

        if self._monitor:
            self._monitor.stop()

        try:
            self.oams.abort_current_action()
            self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.1)
            self.oams.clear_errors()
            self.oams.current_spool = None
        except Exception as e:
            self.logger.error(f"Error clearing OAMS errors: {e}")

        # Clear lane_loaded state for this unit's lanes only
        for lane_name, lane in self.lanes.items():
            try:
                if getattr(lane, "tool_loaded", False):
                    lane.unsync_to_extruder()
                    lane.set_tool_unloaded()
            except Exception as e:
                self.logger.warning(f"Failed to clear lane_loaded for {lane_name}: {e}")

        # Restore LEDs based on current state
        for lane_name, lane in self.lanes.items():
            try:
                if getattr(lane, 'tool_loaded', False):
                    self.lane_tool_loaded(lane)
                elif getattr(lane, 'load_state', False):
                    self.lane_loaded(lane)
                else:
                    self.lane_unloaded(lane)
            except Exception:
                pass

        if self._monitor:
            self._monitor.state.reset()
            self._monitor.start(self.oams)

        gcmd.respond_info("OpenAMS errors cleared and state resynced")

    # ── Hardware event handler ──────────────────────────────────────

    def handle_openams_lane_tool_state(
        self, lane_name: str, loaded: bool, *,
        spool_index: Optional[int] = None,
        eventtime: Optional[float] = None
    ) -> bool:
        """Update lane/tool state from OpenAMS hardware events.

        Scopes lookups to the event lane's own extruder to avoid
        cross-extruder interference in multi-extruder setups.
        """
        lane = self._resolve_lane_reference(lane_name)
        if lane is None:
            self.logger.warning(
                f"OpenAMS reported lane {lane_name} but cannot resolve it")
            return False

        if eventtime is None:
            eventtime = self.afc.reactor.monotonic()

        if loaded:
            lane.loaded_to_hub = True

            # Check if a different lane is loaded on THIS lane's extruder
            lane_extruder_obj = getattr(lane, "extruder_obj", None)
            prev_name = getattr(lane_extruder_obj, "lane_loaded", None) if lane_extruder_obj else None
            if prev_name and prev_name != lane.name and prev_name in self.afc.lanes:
                prev_lane = self.afc.lanes[prev_name]
                try:
                    prev_lane.unsync_to_extruder()
                    prev_lane.set_tool_unloaded()
                except Exception as e:
                    self.logger.error(f"Failed to unset previous lane {prev_name}: {e}")

            try:
                lane.set_tool_loaded()
            except Exception as e:
                self.logger.error(f"Failed to mark lane {lane.name} as loaded: {e}")

            try:
                lane.sync_to_extruder()
            except Exception as e:
                self.logger.error(f"Failed to sync lane {lane.name}: {e}")

            try:
                self.afc.save_vars()
            except Exception:
                pass
            return True

        # Unload path — scope to lane's own extruder
        lane_extruder_obj = getattr(lane, "extruder_obj", None)
        ext_lane_loaded = getattr(lane_extruder_obj, "lane_loaded", None) if lane_extruder_obj else None

        if (ext_lane_loaded == lane.name) or getattr(lane, "tool_loaded", False):
            try:
                lane.unsync_to_extruder()
            except Exception as e:
                self.logger.error(f"Failed to unsync lane {lane.name}: {e}")
            try:
                lane.set_tool_unloaded()
            except Exception as e:
                self.logger.error(f"Failed to unload lane {lane.name}: {e}")
            try:
                self.afc.save_vars()
            except Exception:
                pass

        # Clear extruder tracking if it still points to this lane
        if lane_extruder_obj and getattr(lane_extruder_obj, "lane_loaded", None) == lane.name:
            lane_extruder_obj.lane_loaded = None

        return True

    # ── Runout handling ─────────────────────────────────────────────

    def handle_runout_detected(self, spool_index, monitor=None, lane_name=None):
        """Handle runout event from OpenAMS hardware."""
        lane = self._lane_for_spool_index(spool_index)
        if lane is None and lane_name:
            lane = self._resolve_lane_reference(lane_name)
        if lane is None:
            self.logger.warning(f"Cannot resolve lane for runout spool {spool_index}")
            return

        runout_lane = getattr(lane, 'runout_lane', None)
        if runout_lane:
            self.logger.info(
                f"OpenAMS runout on {lane.name}, switching to {runout_lane}")
        self.afc.error.AFC_error(
            f"Runout detected on {lane.name}", pause=True)

    def check_runout(self, lane=None):
        pass

    # ── Internal helpers ────────────────────────────────────────────

    def _get_oams_index(self) -> int:
        """Return numeric OAMS index for gcode commands."""
        try:
            return int(self.oams_name.replace("oams", ""))
        except (ValueError, AttributeError):
            return 1

    def _get_openams_spool_index(self, lane) -> int:
        return self._spool_map.get(getattr(lane, 'name', ''), 0)

    def _resolve_lane_reference(self, lane_name: Optional[str]):
        """Resolve lane name to lane object, with case-insensitive fallback."""
        if not lane_name:
            return None
        if lane_name in self.afc.lanes:
            return self.afc.lanes[lane_name]
        lower = lane_name.lower()
        for name, lane in self.afc.lanes.items():
            if name.lower() == lower:
                return lane
        return None

    def _lane_for_spool_index(self, spool_index: Optional[int]):
        """Find lane by spool index."""
        if spool_index is None:
            return None
        for name, idx in self._spool_map.items():
            if idx == spool_index and name in self.afc.lanes:
                return self.afc.lanes[name]
        return None

    def _wait_for_idle(self, timeout: float = 30.0) -> bool:
        """Wait for OAMS hardware to become idle."""
        if self.oams is None:
            return False
        deadline = self.afc.reactor.monotonic() + timeout
        while self.afc.reactor.monotonic() < deadline:
            status = getattr(self.oams, 'action_status', None)
            if status is None:
                return True
            self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.5)
        self.logger.error("OAMS idle timeout")
        return False

    def _get_monitor_state(self):
        """Get FPS state for follower/monitor interactions."""
        if self._monitor:
            return self._monitor.state
        return None

    def _calibrate_hub_hes_spool(self, spool_index: int) -> bool:
        """Run hub HES calibration for a single spool bay."""
        try:
            oams_idx = self._get_oams_index()
            self.afc.gcode.run_script_from_command(
                f"OAMS_CALIBRATE_HUB_HES OAMS={oams_idx} SPOOL={spool_index}")
            return True
        except Exception as e:
            self.logger.error(f"Hub HES calibration failed for spool {spool_index}: {e}")
            return False


def load_config_prefix(config):
    return afcAMS(config)
