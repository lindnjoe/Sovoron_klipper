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

        # Engagement params — unit-level defaults, per-lane overrides from lane config
        self._engagement_length = config.getfloat("engagement_length", 20.0, minval=1.0)
        self._engagement_speed = config.getfloat("engagement_speed", 300.0, minval=10.0)
        self._engagement_params: dict[str, tuple[float, float]] = {}

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

        # Sensor polling state
        self._last_f1s = [None] * 4
        self._last_hub = [None] * 4
        self._poll_timer = None

        # Operation guard — prevents polling from corrupting state during load/unload
        self._operation_active = False
        self._prev_states_stale = False
        self._hub_load_suppressed: set[str] = set()

        # Register temperature_oams sensor factory during config parsing
        try:
            from extras.temperature_oams import TemperatureOAMS
            pheaters = self.printer.load_object(config, "heaters")
            pheaters.add_sensor_factory("temperature_oams", TemperatureOAMS)
        except Exception:
            pass

        # Defer hardware resolution until reactor is running
        self.printer.register_event_handler("klippy:ready", self.handle_ready)

    def handle_connect(self):
        super().handle_connect()

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

        # Build spool map, set custom commands, read per-lane engagement params
        for lane_name, lane in self.lanes.items():
            slot = getattr(lane, 'index', 0) - 1
            if slot < 0:
                slot = 0
            self._spool_map[lane_name] = slot
            lane.custom_load_cmd = f"_OAMS_CUSTOM_LOAD UNIT={self.name} LANE={lane_name}"
            lane.custom_unload_cmd = f"_OAMS_CUSTOM_UNLOAD UNIT={self.name} LANE={lane_name}"
            eng_len = getattr(lane, 'engagement_length', None)
            if eng_len is not None:
                eng_speed = getattr(lane, 'engagement_speed', None) or self._engagement_speed
                self._engagement_params[lane_name] = (eng_len, eng_speed)

            # OpenAMS lanes use virtual sensors driven by hardware polling;
            # initialize to False until handle_ready syncs from hardware
            lane.prep_state = False
            lane._load_state = False
            lane.loaded_to_hub = False
            lane.status = AFCLaneState.NONE

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

    def handle_ready(self):
        """Resolve OAMS hardware and start sensor polling once reactor is running."""
        self.oams = self.printer.lookup_object(f"oams {self.oams_name}", None)

        if self.oams is None:
            self.logger.warning(
                f"OpenAMS hardware '[oams {self.oams_name}]' not found for "
                f"'{self.name}'. Sensor state will not update.")
            return

        self._init_follower_and_monitor()

        # Seed initial state from hardware sensors
        self._sync_lanes_from_hardware()

        # Start periodic polling (every 2s) for sensor changes
        self._poll_timer = self.afc.reactor.register_timer(
            self._poll_oams_sensors,
            self.afc.reactor.monotonic() + 1.0)

    def _sync_lanes_from_hardware(self):
        """Read current OAMS sensor values and update all lane states."""
        if self.oams is None:
            return

        eventtime = self.afc.reactor.monotonic()
        f1s_values = getattr(self.oams, 'f1s_hes_value', None) or []
        hub_values = getattr(self.oams, 'hub_hes_value', None) or []

        for lane_name, lane in self.lanes.items():
            slot = self._spool_map.get(lane_name, -1)
            if slot < 0:
                continue

            f1s_present = bool(f1s_values[slot]) if slot < len(f1s_values) else False
            hub_present = bool(hub_values[slot]) if slot < len(hub_values) else False

            lane._load_state = f1s_present
            lane.prep_state = f1s_present
            lane.loaded_to_hub = hub_present

            if slot < len(f1s_values):
                self._last_f1s[slot] = f1s_present
            if slot < len(hub_values):
                self._last_hub[slot] = hub_present

    def _poll_oams_sensors(self, eventtime):
        """Periodic timer callback — detect sensor changes and update lane state."""
        if self.oams is None:
            return self.afc.reactor.NEVER

        if self._operation_active:
            return eventtime + 2.0

        resync_prev = self._prev_states_stale
        self._prev_states_stale = False

        f1s_values = getattr(self.oams, 'f1s_hes_value', None) or []
        hub_values = getattr(self.oams, 'hub_hes_value', None) or []

        for lane_name, lane in self.lanes.items():
            slot = self._spool_map.get(lane_name, -1)
            if slot < 0:
                continue

            # F1S sensor (filament in spool bay)
            if slot < len(f1s_values):
                new_f1s = bool(f1s_values[slot])

                lane._load_state = new_f1s
                lane.prep_state = new_f1s

                if resync_prev:
                    self._last_f1s[slot] = new_f1s
                else:
                    old_f1s = self._last_f1s[slot] if slot < len(self._last_f1s) else None

                    if old_f1s is not None and new_f1s != old_f1s:
                        prep_done = getattr(lane, '_afc_prep_done', False)
                        if prep_done:
                            if new_f1s and not old_f1s:
                                if lane_name in self._hub_load_suppressed:
                                    self._hub_load_suppressed.discard(lane_name)
                                else:
                                    self.logger.info(f"OAMS: {lane.name} filament inserted")
                                    if lane.status == AFCLaneState.NONE:
                                        lane.set_loaded()
                                        self.lane_loaded(lane)
                                        self.afc.save_vars()

                            elif not new_f1s and old_f1s:
                                self.logger.info(f"OAMS: {lane.name} filament removed")
                                if getattr(lane, 'tool_loaded', False):
                                    try:
                                        is_printing = self.afc.function.is_printing()
                                    except Exception:
                                        is_printing = False
                                    if is_printing:
                                        self.afc.error.AFC_error(
                                            f"OAMS runout on {lane.name}", pause=True)
                                else:
                                    lane.loaded_to_hub = False
                                    lane.status = AFCLaneState.NONE
                                    self.lane_unloaded(lane)
                                    self.afc.save_vars()

                    self._last_f1s[slot] = new_f1s

            # Hub HES sensor (filament at hub)
            if slot < len(hub_values):
                new_hub = bool(hub_values[slot])

                if resync_prev:
                    self._last_hub[slot] = new_hub
                    lane.loaded_to_hub = new_hub
                else:
                    old_hub = self._last_hub[slot] if slot < len(self._last_hub) else None

                    if old_hub is not None and new_hub != old_hub:
                        lane.loaded_to_hub = new_hub
                        hub = getattr(lane, 'hub_obj', None)
                        if hub is not None and hasattr(hub, 'switch_pin_callback'):
                            try:
                                hub.switch_pin_callback(eventtime, new_hub)
                            except Exception:
                                pass

                    self._last_hub[slot] = new_hub

        return eventtime + 2.0

    # ── Engagement verification ─────────────────────────────────────

    def get_engagement_params(self, lane_name: str) -> tuple:
        """Return (engagement_length, engagement_speed) for a lane."""
        if lane_name in self._engagement_params:
            return self._engagement_params[lane_name]
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
        """OpenAMS system test — query hardware sensors for lane state."""
        msg = ''
        succeeded = True

        if self.oams is None:
            msg = '<span class=error--text>OAMS NOT CONNECTED</span>'
            succeeded = False
        else:
            slot = self._spool_map.get(cur_lane.name, -1)

            # Read state directly from OAMS hardware sensors
            f1s_values = getattr(self.oams, 'f1s_hes_value', None) or []
            hub_values = getattr(self.oams, 'hub_hes_value', None) or []

            f1s_present = bool(f1s_values[slot]) if 0 <= slot < len(f1s_values) else False
            hub_present = bool(hub_values[slot]) if 0 <= slot < len(hub_values) else False

            # Update lane state from hardware
            cur_lane._load_state = f1s_present
            cur_lane.prep_state = f1s_present
            cur_lane.loaded_to_hub = hub_present

            if f1s_present:
                self.lane_loaded(cur_lane)
                msg += "<span class=success--text>LOCKED</span>"
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
                self.afc.function.afc_led(cur_lane.led_not_ready, cur_lane.led_index)
                msg += 'EMPTY READY FOR SPOOL'

        if assignTcmd:
            self.afc.function.TcmdAssign(cur_lane)
        cur_lane.do_enable(False)
        self.logger.info('{lane_name} tool cmd: {tcmd:3} {msg}'.format(
            lane_name=cur_lane.name, tcmd=cur_lane.map, msg=msg))
        cur_lane.set_afc_prep_done()
        return succeeded

    def calibrate_bowden(self, cur_lane, dis, tol):
        msg = (
            "OpenAMS units do not support standard AFC bowden calibration. "
            "Use OpenAMS-specific calibration commands instead:\n"
            f"  - AFC_OAMS_CALIBRATE_PTFE SPOOL=<spool_index>\n"
            f"  - AFC_OAMS_CALIBRATE_HUB_HES SPOOL=<spool_index>\n"
            f"  - AFC_OAMS_CALIBRATE_HUB_HES_ALL"
        )
        self.logger.info(msg)
        return False, msg, 0

    def calibrate_lane(self, cur_lane, tol):
        msg = (
            "OpenAMS units do not support standard AFC hub calibration. "
            "Use OpenAMS-specific calibration commands instead:\n"
            f"  - AFC_OAMS_CALIBRATE_HUB_HES SPOOL=<spool_index>\n"
            f"  - AFC_OAMS_CALIBRATE_HUB_HES_ALL"
        )
        self.logger.info(msg)
        return False, msg, 0

    def _toolhead_sensor_triggered(self, cur_lane):
        """Check if the toolhead sensor is triggered, using the raw hardware
        button state for U1 motion sensors (which need encoder rotation for
        filament_present but have a physical switch for static detection)."""
        sensor_obj = getattr(cur_lane.extruder_obj, 'filament_sensor_obj', None)
        if sensor_obj is not None and hasattr(sensor_obj, 'runout_buttun_state'):
            return bool(sensor_obj.runout_buttun_state)
        return cur_lane.get_toolhead_pre_sensor_state()

    # ── Custom load/unload gcode handlers ───────────────────────────

    def _cmd_oams_custom_load(self, gcmd):
        """Handle _OAMS_CUSTOM_LOAD — full load sequence with toolhead ops."""
        lane_name = gcmd.get('LANE')
        cur_lane = self.afc.lanes.get(lane_name)
        if cur_lane is None:
            raise gcmd.error(f"Unknown lane: {lane_name}")
        cur_extruder = cur_lane.extruder_obj
        result = self._oams_load_sequence(cur_lane, cur_extruder)
        if not result:
            raise gcmd.error(f"OAMS load failed for {lane_name}")

    def _cmd_oams_custom_unload(self, gcmd):
        """Handle _OAMS_CUSTOM_UNLOAD — full unload sequence with toolhead ops."""
        lane_name = gcmd.get('LANE')
        cur_lane = self.afc.lanes.get(lane_name)
        if cur_lane is None:
            raise gcmd.error(f"Unknown lane: {lane_name}")
        cur_extruder = cur_lane.extruder_obj
        result = self._oams_unload_sequence(cur_lane, cur_extruder)
        if not result:
            raise gcmd.error(f"OAMS unload failed for {lane_name}")

    def _oams_load_sequence(self, cur_lane, cur_extruder) -> bool:
        """Full OAMS load: heat → OAMS hardware load → extruder load."""
        self._operation_active = True
        try:
            return self._oams_load_inner(cur_lane, cur_extruder)
        finally:
            self._operation_active = False
            self._prev_states_stale = True

    def _oams_load_inner(self, cur_lane, cur_extruder) -> bool:
        afc = self.afc

        # Heat extruder
        if afc._check_extruder_temp(cur_lane):
            afc.afcDeltaTime.log_with_time("Done heating toolhead")

        # Clear suppression — this lane is being intentionally loaded
        self._hub_load_suppressed.discard(cur_lane.name)

        # OAMS hardware load
        if not self._oams_load(cur_lane):
            return False

        # Sync to extruder and finalize
        cur_lane.loaded_to_hub = True
        cur_lane.sync_to_extruder()

        # Extruder load (tool_stn)
        if cur_extruder.tool_stn > 0:
            afc.move_e_pos(cur_extruder.tool_stn, cur_extruder.tool_load_speed, "tool stn")
            afc.toolhead.wait_moves()

        # Verify filament reached toolhead sensor (switch, FPS, or motion sensor)
        # Pulse extruder in small increments if sensor not yet triggered
        has_sensor = (cur_extruder.tool_start is not None
                      or getattr(cur_extruder, 'filament_sensor_obj', None) is not None)
        if has_sensor and not self._toolhead_sensor_triggered(cur_lane):
            afc.reactor.pause(afc.reactor.monotonic() + 0.5)
            if not self._toolhead_sensor_triggered(cur_lane):
                pulse_step = cur_lane.short_move_dis
                max_pulse_dist = afc.tool_homing_distance
                total_pulsed = 0.0
                self.logger.info(
                    f"OAMS load: sensor not triggered, pulsing up to "
                    f"{max_pulse_dist:.0f}mm in {pulse_step:.0f}mm steps")
                while total_pulsed < max_pulse_dist:
                    afc.move_e_pos(
                        pulse_step, cur_extruder.tool_load_speed,
                        "load retry pulse", wait_tool=True)
                    total_pulsed += pulse_step
                    afc.reactor.pause(afc.reactor.monotonic() + 0.3)
                    if self._toolhead_sensor_triggered(cur_lane):
                        self.logger.info(
                            f"OAMS load: sensor triggered after "
                            f"{total_pulsed:.0f}mm of extra feed")
                        break
                else:
                    cur_lane.unsync_to_extruder()
                    message = (
                        f"OAMS load: filament did not reach toolhead sensor for "
                        f"{cur_lane.name} after {total_pulsed:.0f}mm of retry pulses. "
                        f"Spool may be stuck or PTFE path blocked.\n"
                        f"To resolve set lane loaded with "
                        f"`SET_LANE_LOADED LANE={cur_lane.name}` macro."
                    )
                    if afc.function.in_print():
                        message += "\nOnce filament is fully loaded click resume to continue printing"
                    afc.error.handle_lane_failure(cur_lane, message)
                    return False

        afc.afcDeltaTime.log_with_time("OAMS load complete")
        return True

    def _oams_unload_sequence(self, cur_lane, cur_extruder) -> bool:
        """Full OAMS unload: heat → cut → park → tip → OAMS hardware unload."""
        self._operation_active = True
        try:
            return self._oams_unload_inner(cur_lane, cur_extruder)
        finally:
            self._operation_active = False
            self._prev_states_stale = True

    def _oams_unload_inner(self, cur_lane, cur_extruder) -> bool:
        afc = self.afc

        # Disable buffer
        cur_lane.disable_buffer()

        # LED animation
        self.lane_unloading(cur_lane)

        # Heat extruder
        if afc._check_extruder_temp(cur_lane):
            afc.afcDeltaTime.log_with_time("Done heating toolhead")

        # Quick pull
        afc.move_e_pos(-2, cur_extruder.tool_unload_speed, "Quick Pull", wait_tool=False)

        # Sync for cut/park/tip
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

        # Retract from extruder
        if cur_extruder.tool_stn_unload > 0:
            afc.move_e_pos(
                cur_extruder.tool_stn_unload * -1,
                cur_extruder.tool_unload_speed,
                "Retract from extruder", wait_tool=True)

        # Unsync
        cur_lane.unsync_to_extruder()

        afc.afcDeltaTime.log_with_time("Toolhead operations complete")

        # OAMS hardware unload
        if not self._oams_unload(cur_lane):
            afc.error.handle_lane_failure(
                cur_lane, f"OAMS unload failed for {cur_lane.name}")
            return False

        # Finalize state
        cur_lane.set_tool_unloaded()
        cur_lane.loaded_to_hub = True
        self.lane_tool_unloaded(cur_lane)
        self._hub_load_suppressed.add(cur_lane.name)

        if afc.post_unload_macro is not None:
            afc.gcode.run_script_from_command(afc.post_unload_macro)

        afc.afcDeltaTime.log_with_time("OAMS unload complete")
        return True

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
