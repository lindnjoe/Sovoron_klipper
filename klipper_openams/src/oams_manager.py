# OpenAMS Manager
#
# Copyright (C) 2025 JR Lomas <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.


import logging
import time
from functools import partial
from collections import deque
from typing import Optional, Tuple, Dict, List, Any, Callable

PAUSE_DISTANCE = 60
ENCODER_SAMPLES = 2
MIN_ENCODER_DIFF = 1
FILAMENT_PATH_LENGTH_FACTOR = 1.14  # Replace magic number with a named constant
MONITOR_ENCODER_LOADING_SPEED_AFTER = 2.0  # in seconds
MONITOR_ENCODER_UNLOADING_SPEED_AFTER = 2.0  # in seconds
MONITOR_ENCODER_PERIOD = 2.0  # in seconds


# enum of states
class OAMSRunoutStateEnum:
    STOPPED = "STOPPED"
    MONITORING = "MONITORING"
    DETECTED = "DETECTED"
    COASTING = "COASTING"
    RELOADING = "RELOADING"
    PAUSED = "PAUSED"


class FPSLoadState:
    UNLOADED = "UNLOADED"
    LOADED = "LOADED"
    LOADING = "LOADING"
    UNLOADING = "UNLOADING"
    
class OAMSRunoutMonitor:
    def __init__(self, 
                 printer,
                 fps_name,
                 fps, 
                 fps_state,
                 oams,
                 reload_callback, 
                 reload_before_toolhead_distance=0.0):
        self.oams = oams
        self.printer = printer
        self.fps_name = fps_name
        self.fps_state = fps_state
        self.fps = fps
        self.state = OAMSRunoutStateEnum.STOPPED
        self.runout_position = None
        self.bldc_clear_position = None
        self.reload_before_toolhead_distance = reload_before_toolhead_distance
        self.reload_callback = reload_callback
        reactor = self.printer.get_reactor()
        
        def _monitor_runout(eventtime):
            idle_timeout = self.printer.lookup_object("idle_timeout")
            is_printing = idle_timeout.get_status(eventtime)["state"] == "Printing"
            
            if self.state == OAMSRunoutStateEnum.STOPPED or self.state == OAMSRunoutStateEnum.PAUSED or self.state == OAMSRunoutStateEnum.RELOADING:
                pass
            elif self.state == OAMSRunoutStateEnum.MONITORING:
                if (
                    is_printing
                    and fps_state.state_name == "LOADED"
                    and fps_state.current_group is not None
                    and fps_state.current_spool_idx is not None
                ):
                    oams = self.oams[fps_state.current_oams]
                    spool_empty = not bool(oams.f1s_hes_value[fps_state.current_spool_idx])
                    hub_empty = not bool(oams.hub_hes_value[fps_state.current_spool_idx])
                    if spool_empty and hub_empty:
                        self.state = OAMSRunoutStateEnum.DETECTED
                        logging.info(
                            f"OAMS: Runout detected on FPS {self.fps_name}, pausing for {PAUSE_DISTANCE} mm before coasting the follower."
                        )
                        self.runout_position = fps.extruder.last_position
            
            elif self.state == OAMSRunoutStateEnum.DETECTED:
                traveled_distance = fps.extruder.last_position - self.runout_position
                if traveled_distance >= PAUSE_DISTANCE:
                    logging.info("OAMS: Pause complete, coasting the follower.")
                    self.oams[fps_state.current_oams].set_oams_follower(0, 1)
                    self.bldc_clear_position = fps.extruder.last_position
                    self.state = OAMSRunoutStateEnum.COASTING
                    
            elif self.state == OAMSRunoutStateEnum.COASTING:
                traveled_distance_after_bldc_clear = fps.extruder.last_position - self.bldc_clear_position
                if traveled_distance_after_bldc_clear + self.reload_before_toolhead_distance > self.oams[fps_state.current_oams].filament_path_length / FILAMENT_PATH_LENGTH_FACTOR:
                    logging.info("OAMS: Loading next spool in the filament group.")
                    self.state = OAMSRunoutStateEnum.RELOADING
                    self.reload_callback()
            else:
                raise ValueError(f"Invalid state: {self.state}")
            return eventtime + MONITOR_ENCODER_PERIOD
        self.timer = reactor.register_timer(_monitor_runout, reactor.NOW)
        
    def start(self):
        self.state = OAMSRunoutStateEnum.MONITORING    
    
    def stop(self):
        self.state = OAMSRunoutStateEnum.STOPPED
        
    def reloading(self):
        self.state = OAMSRunoutStateEnum.RELOADING
        self.runout_position = None
        self.runout_after_position = None
        
    def paused(self):
        self.state = OAMSRunoutStateEnum.PAUSED
        
    def reset(self):
        self.state = OAMSRunoutStateEnum.STOPPED
        self.runout_position = None
        self.runout_after_position = None
        if self.timer is not None:
            self.printer.get_reactor().unregister_timer(self.timer)
            self.timer = None

class OAMSState:
    def __init__(self) -> None:
        self.fps_state: Dict[str, "FPSState"] = {}

    def add_fps_state(self, fps_name: str) -> None:
        self.fps_state[fps_name] = FPSState()


class FPSState:
    def __init__(
        self,
        state_name: str = FPSLoadState.UNLOADED,
        current_group: Optional[str] = None,
        current_oams: Optional[str] = None,
        current_spool_idx: Optional[int] = None,
    ) -> None:

        # Primary state tracking
        self.state_name = state_name
        self.current_group = current_group
        self.current_oams = current_oams
        self.current_spool_idx = current_spool_idx

        # Runout tracking
        self.runout_position: Optional[float] = None
        self.runout_after_position: Optional[float] = None

        # Timer references
        self.monitor_spool_timer = None
        self.monitor_pause_timer = None
        self.monitor_load_next_spool_timer = None

        # Motion monitoring
        self.encoder_samples = deque(maxlen=ENCODER_SAMPLES)

        # Follower state
        self.following: bool = False
        self.direction: int = 0
        self.since: Optional[float] = None

    def reset_runout_positions(self) -> None:
        self.runout_position = None
        self.runout_after_position = None

    def __repr__(self) -> str:
        return (
            f"FPSState(state_name={self.state_name}, current_group={self.current_group}, "
            f"current_oams={self.current_oams}, current_spool_idx={self.current_spool_idx})"
        )

    def __str__(self) -> str:
        return (
            f"State: {self.state_name}, Group: {self.current_group}, "
            f"OAMS: {self.current_oams}, Spool Index: {self.current_spool_idx}"
        )

class OAMSManager:
    def __init__(self, config) -> None:
        self.config = config
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()

        # Hardware object collections
        self.filament_groups: Dict[str, Any] = {}
        self.oams: Dict[str, Any] = {}
        self.fpss: Dict[str, Any] = {}

        # State management
        self.current_state = OAMSState()

        # Monitoring and control
        self.monitor_timers: List[Any] = []
        self.runout_monitor: Optional[OAMSRunoutMonitor] = None
        self.ready: bool = False

        # Configuration parameters
        self.reload_before_toolhead_distance = config.getfloat("reload_before_toolhead_distance", 0.0)

        # External integration
        self.runout_callback: Optional[Callable[[str, str, int], None]] = None

        # Initialize hardware collections
        self._initialize_oams()
        self._initialize_filament_groups()

        # Register with printer and setup event handlers
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.printer.add_object("oams_manager", self)
        self.register_commands()

    def register_runout_callback(self, callback: Callable[[str, str, int], None]) -> None:
        """Register a callback for runout events.

        After the manager attempts to reload from the filament group the
        callback is invoked with ``fps_name``, ``group`` and ``spool_idx``.
        ``spool_idx`` is the index of the newly loaded spool or ``-1`` if no
        spool was available so external handling is required.
        """
        self.runout_callback = callback

    def load_spool_for_lane(
        self, fps_name: str, group_name: str, oams_name: str, bay_index: int
    ) -> bool:
        """Manually load a spool from a specific bay and resume monitoring."""
        fps_state = self.current_state.fps_state.get(fps_name)
        oam = self.oams.get(oams_name)
        if fps_state is None or oam is None:
            return False
        if not oam.is_bay_ready(bay_index):
            return False

        success, _ = oam.load_spool(bay_index)
        if not success:
            return False

        now = self.reactor.monotonic()
        fps_state.state_name = "LOADED"
        fps_state.since = now
        fps_state.current_group = group_name
        fps_state.current_oams = oams_name
        fps_state.current_spool_idx = bay_index
        fps_state.reset_runout_positions()

        if self.runout_monitor is not None:
            self.runout_monitor.reset()
            self.runout_monitor.start()

        return True
        
    def get_status(self, eventtime) -> Dict[str, Dict[str, Any]]:
        attributes = {
            "oams": {
                name: {"action_status_code": oam.action_status_code}
                for name, oam in self.oams.items()
            }
        }
        for fps_name, fps_state in self.current_state.fps_state.items():
            attributes[fps_name] = {
                "current_group": fps_state.current_group,
                "current_oams": fps_state.current_oams,
                "current_spool_idx": fps_state.current_spool_idx,
                "state_name": fps_state.state_name,
                "since": fps_state.since,
            }
        return attributes
    
    def determine_state(self):
        current_oams = None
        for (fps_name, fps_state) in self.current_state.fps_state.items():
            fps_state.current_group, current_oams, fps_state.current_spool_idx = self.determine_current_loaded_group(fps_name)
            if current_oams is not None:
                fps_state.current_oams = current_oams.name
            else:
                fps_state.current_oams = None
            if fps_state.current_oams is not None and fps_state.current_spool_idx is not None:
                fps_state.state_name = "LOADED"
                fps_state.since = self.reactor.monotonic()
        
    def handle_ready(self):
        for (fps_name, fps) in self.printer.lookup_objects(module="fps"):
            self.fpss[fps_name] = fps
            self.current_state.add_fps_state(fps_name)
        if self.fpss == {}:
            raise ValueError("No FPS found in system, this is required for OAMS to work")
        self.determine_state()
        self.start_monitors()
        self.ready = True

    def _initialize_oams(self) -> None:
        for name, oam in self.printer.lookup_objects(module="oams"):
            short_name = name.split()[-1]
            oam.name = short_name
            self.oams[short_name] = oam
        
    def _initialize_filament_groups(self) -> None:
        for name, group in self.printer.lookup_objects(module="filament_group"):
            name = name.split()[-1]
            logging.info(f"OAMS: Adding group {name}")
            self.filament_groups[name] = group
    
    def determine_current_loaded_group(
        self, fps_name: str
    ) -> Tuple[Optional[str], Optional[object], Optional[int]]:
        fps = self.fpss.get(fps_name)
        if fps is None:
            raise ValueError(f"FPS {fps_name} not found")
        for group_name, group in self.filament_groups.items():
            for oam, bay_index in group.bays:
                if oam.is_bay_loaded(bay_index) and oam in fps.oams:
                    return group_name, oam, bay_index
        return None, None, None
        
    def register_commands(self):
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "OAMSM_UNLOAD_FILAMENT",
            self.cmd_UNLOAD_FILAMENT,
            desc=self.cmd_UNLOAD_FILAMENT_help,
        )
        
        gcode.register_command(
            "OAMSM_LOAD_FILAMENT",
            self.cmd_LOAD_FILAMENT,
            desc=self.cmd_LOAD_FILAMENT_help,
        )
        
        gcode.register_command(
            "OAMSM_FOLLOWER",
            self.cmd_FOLLOWER,
            desc=self.cmd_FOLLOWER_help,
        )
        
        # gcode.register_command(
@@ -397,99 +493,120 @@ class OAMSManager:
                    return
                else:
                    gcmd.respond_info(message)
                    return
        gcmd.respond_info(f"No spool available for group {group_name}")
        return
        
    def _pause_printer_message(self, message):
        logging.info(f"OAMS: {message}")
        gcode = self.printer.lookup_object("gcode")
        message = f"Print has been paused: {message}"
        gcode.run_script(f"M118 {message}")
        gcode.run_script(f"M114 {message}")
        gcode.run_script("PAUSE")
        
    def _monitor_unload_speed_for_fps(self, fps_name):
        def _monitor_unload_speed(self, eventtime):
            #logging.info("OAMS: Monitoring unloading speed state: %s" % self.current_state.name)
            fps_state = self.current_state.fps_state[fps_name]
            oams = None
            if fps_state.current_oams is not None:
                oams = self.oams[fps_state.current_oams]
            if fps_state.state_name == "UNLOADING" and self.reactor.monotonic() - fps_state.since > MONITOR_ENCODER_UNLOADING_SPEED_AFTER:
                fps_state.encoder_samples.append(oams.encoder_clicks)
                if len(fps_state.encoder_samples) < ENCODER_SAMPLES:
                    return eventtime + MONITOR_ENCODER_PERIOD
                encoder_diff = abs(fps_state.encoder_samples[-1] - fps_state.encoder_samples[0])
                logging.info("OAMS[%d] Unload Monitor: Encoder diff %d" %(oams.oams_idx, encoder_diff))
                if encoder_diff < MIN_ENCODER_DIFF:              
                    oams.set_led_error(fps_state.current_spool_idx, 1)
                    self._pause_printer_message("Printer paused because the unloading speed of the moving filament was too low")
                    logging.info("after unload speed too low")
                    self.stop_monitors()
                    return self.printer.get_reactor().NEVER
            return eventtime + MONITOR_ENCODER_PERIOD
        return partial(_monitor_unload_speed, self)
    
    def _monitor_load_speed_for_fps(self, fps_name):
        def _monitor_load_speed(self, eventtime):
            #logging.info("OAMS: Monitoring loading speed state: %s" % self.current_state.name)
            fps_state = self.current_state.fps_state[fps_name]
            oams = None
            if fps_state.current_oams is not None:
                oams = self.oams[fps_state.current_oams]
            if fps_state.state_name == "LOADING" and self.reactor.monotonic() - fps_state.since > MONITOR_ENCODER_LOADING_SPEED_AFTER:
                fps_state.encoder_samples.append(oams.encoder_clicks)
                if len(fps_state.encoder_samples) < ENCODER_SAMPLES:
                    return eventtime + MONITOR_ENCODER_PERIOD
                encoder_diff = abs(fps_state.encoder_samples[-1] - fps_state.encoder_samples[0])
                logging.info("OAMS[%d] Load Monitor: Encoder diff %d" % (oams.oams_idx, encoder_diff))
                if encoder_diff < MIN_ENCODER_DIFF:
                    oams.set_led_error(fps_state.current_spool_idx, 1)
                    self._pause_printer_message("Printer paused because the loading speed of the moving filament was too low")
                    self.stop_monitors()
                    return self.printer.get_reactor().NEVER
            return eventtime + MONITOR_ENCODER_PERIOD
        return partial(_monitor_load_speed, self)
    
    def start_monitors(self):
        self.monitor_timers = []
        reactor = self.printer.get_reactor()        
        for fps_name, fps_state in self.current_state.fps_state.items():
            self.monitor_timers.append(
                reactor.register_timer(self._monitor_unload_speed_for_fps(fps_name), reactor.NOW)
            )
            self.monitor_timers.append(
                reactor.register_timer(self._monitor_load_speed_for_fps(fps_name), reactor.NOW)
            )

            def _reload_callback(fps_name=fps_name, fps_state=fps_state):
                for oam, bay_index in self.filament_groups[fps_state.current_group].bays:
                    if oam.is_bay_ready(bay_index):
                        success, message = oam.load_spool(bay_index)
                        if success:
                            logging.info(
                                f"OAMS: Successfully loaded spool in bay {bay_index} of OAM {oam.name}"
                            )
                            fps_state.state_name = "LOADED"
                            fps_state.since = self.reactor.monotonic()
                            fps_state.current_spool_idx = bay_index
                            fps_state.current_oams = oam.name
                            fps_state.reset_runout_positions()
                            self.runout_monitor.reset()
                            self.runout_monitor.start()
                            if self.runout_callback is not None:
                                self.runout_callback(fps_name, fps_state.current_group, bay_index)
                            return
                        else:
                            logging.error(f"OAMS: Failed to load spool: {message}")
                            break
                self._pause_printer_message(
                    "No spool available for group %s" % fps_state.current_group
                )
                self.runout_monitor.paused()
                if self.runout_callback is not None:
                    self.runout_callback(fps_name, fps_state.current_group, -1)
                return

            self.runout_monitor = OAMSRunoutMonitor(
                self.printer,
                fps_name,
                self.fpss[fps_name],
                fps_state,
                self.oams,
                _reload_callback,
                reload_before_toolhead_distance=self.reload_before_toolhead_distance,
            )
            self.monitor_timers.append(self.runout_monitor.timer)
            self.runout_monitor.start()
            
        logging.info("OAMS: All monitors started")
    
    def stop_monitors(self):
        for timer in self.monitor_timers:
            self.printer.get_reactor().unregister_timer(timer)
        self.monitor_timers = []

def load_config(config):
    return OAMSManager(config)