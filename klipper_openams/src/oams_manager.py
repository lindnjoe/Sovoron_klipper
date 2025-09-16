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
from .filament_group import FilamentGroup

# Configuration constants
PAUSE_DISTANCE = 60  # mm to pause before coasting follower
EXTRA_COAST_DISTANCE = 30  # additional mm to coast before loading next spool
ENCODER_SAMPLES = 2  # Number of encoder samples to collect
MIN_ENCODER_DIFF = 1  # Minimum encoder difference to consider movement
FILAMENT_PATH_LENGTH_FACTOR = 1.14  # Factor for calculating filament path traversal
MONITOR_ENCODER_LOADING_SPEED_AFTER = 2.0  # seconds
MONITOR_ENCODER_PERIOD = 2.0 # seconds
MONITOR_ENCODER_UNLOADING_SPEED_AFTER = 2.0  # seconds


class OAMSRunoutState:
    """Enum for runout monitor states."""
    STOPPED = "STOPPED"          # Monitor is disabled
    MONITORING = "MONITORING"    # Actively watching for runout
    DETECTED = "DETECTED"        # Runout detected, pausing before coast
    COASTING = "COASTING"        # Follower coasting, preparing next spool
    RELOADING = "RELOADING"      # Loading next spool in sequence
    PAUSED = "PAUSED"           # Monitor paused due to error/manual intervention


class FPSLoadState:
    """Enum for FPS loading states."""
    UNLOADED = "UNLOADED"    # No filament loaded
    LOADED = "LOADED"        # Filament loaded and ready
    LOADING = "LOADING"      # Currently loading filament
    UNLOADING = "UNLOADING"  # Currently unloading filament
    
class OAMSRunoutMonitor:
    """
    Monitors filament runout for a specific FPS and handles automatic reload.
    
    State Management:
    - Tracks runout detection and follower coasting
    - Manages automatic spool switching within filament groups
    - Coordinates with OAMS hardware for filament loading
    """
    
    def __init__(self, 
                 printer,
                 fps_name: str,
                 fps, 
                 fps_state,
                 oams: Dict[str, Any],
                 reload_callback: Callable, 
                 reload_before_toolhead_distance: float = 0.0):
        # Core references
        self.oams = oams
        self.printer = printer
        self.fps_name = fps_name
        self.fps_state = fps_state
        self.fps = fps
        
        # State tracking
        self.state = OAMSRunoutState.STOPPED
        self.runout_position: Optional[float] = None
        self.bldc_clear_position: Optional[float] = None
        
        # Configuration
        self.reload_before_toolhead_distance = reload_before_toolhead_distance
        self.reload_callback = reload_callback
        
        reactor = self.printer.get_reactor()
        
        def _monitor_runout(eventtime):
            idle_timeout = self.printer.lookup_object("idle_timeout")
            is_printing = idle_timeout.get_status(eventtime)["state"] == "Printing"
            
            if self.state == OAMSRunoutState.STOPPED or self.state == OAMSRunoutState.PAUSED or self.state == OAMSRunoutState.RELOADING:
                pass
            elif self.state == OAMSRunoutState.MONITORING:
                #logging.info("OAMS: Monitoring runout, is_printing: %s, fps_state: %s, fps_state.current_group: %s, fps_state.current_spool_idx: %s, oams: %s" % (is_printing, fps_state.state_name, fps_state.current_group, fps_state.current_spool_idx, fps_state.current_oams))
                if is_printing and \
                fps_state.state_name == "LOADED" and \
                fps_state.current_group is not None and \
                fps_state.current_spool_idx is not None and \
                not bool(self.oams[fps_state.current_oams].hub_hes_value[fps_state.current_spool_idx]):
                    self.state = OAMSRunoutState.DETECTED
                    logging.info(f"OAMS: Runout detected on FPS {self.fps_name}, pausing for {PAUSE_DISTANCE} mm before coasting the follower.")
                    self.runout_position = fps.extruder.last_position
            
            elif self.state == OAMSRunoutState.DETECTED:
                traveled_distance = fps.extruder.last_position - self.runout_position
                if traveled_distance >= PAUSE_DISTANCE:
                    logging.info("OAMS: Pause complete, coasting the follower.")
                    self.oams[fps_state.current_oams].set_oams_follower(0, 1)
                    self.bldc_clear_position = fps.extruder.last_position
                    self.state = OAMSRunoutState.COASTING
                    
            elif self.state == OAMSRunoutState.COASTING:
                traveled_distance_after_bldc_clear = fps.extruder.last_position - self.bldc_clear_position
                if traveled_distance_after_bldc_clear + self.reload_before_toolhead_distance > (
                    self.oams[fps_state.current_oams].filament_path_length / FILAMENT_PATH_LENGTH_FACTOR
                    + EXTRA_COAST_DISTANCE
                ):
                    logging.info("OAMS: Loading next spool in the filament group.")
                    self.state = OAMSRunoutState.RELOADING
                    self.reload_callback()
            else:
                raise ValueError(f"Invalid state: {self.state}")
            return eventtime + MONITOR_ENCODER_PERIOD
        self.timer = reactor.register_timer(_monitor_runout, reactor.NOW)
        
    def start(self) -> None:
        """Start monitoring for filament runout."""
        self.state = OAMSRunoutState.MONITORING    
    
    def stop(self) -> None:
        """Stop monitoring for filament runout."""
        self.state = OAMSRunoutState.STOPPED
        
    def reloading(self) -> None:
        """Set state to reloading and reset positions."""
        self.state = OAMSRunoutState.RELOADING
        self.runout_position = None
        self.runout_after_position = None
        
    def paused(self) -> None:
        """Pause the monitor due to error or manual intervention."""
        self.state = OAMSRunoutState.PAUSED
        
    def reset(self) -> None:
        """Reset monitor to stopped state and clean up."""
        self.state = OAMSRunoutState.STOPPED
        self.runout_position = None
        self.runout_after_position = None
        if self.timer is not None:
            self.printer.get_reactor().unregister_timer(self.timer)
            self.timer = None

class OAMSState:
    """
    Global state container for all FPS units in the system.
    
    Attributes:
    - fps_state: Dictionary mapping FPS names to their FPSState objects
    """
    
    def __init__(self):
        self.fps_state: Dict[str, 'FPSState'] = {}
        
    def add_fps_state(self, fps_name: str) -> None:
        """Add a new FPS state tracker."""
        self.fps_state[fps_name] = FPSState()
        

class FPSState:
    """
    Tracks the state of a single FPS (Filament Pressure Sensor).
    
    Key State Variables:
    - state_name: Current loading state (LOADED, UNLOADED, LOADING, UNLOADING)
    - current_group: Filament group name (e.g., "T0", "T1") if loaded
    - current_oams: Name of the OAMS unit currently loaded
    - current_spool_idx: Index (0-3) of the spool bay currently loaded
    
    Monitoring State:
    - encoder_samples: Recent encoder readings for motion detection
    - following: Whether follower mode is active
    - direction: Follower direction (0=forward, 1=reverse)
    - since: Timestamp when current state began
    """
    
    def __init__(self, 
                 state_name: str = FPSLoadState.UNLOADED, 
                 current_group: Optional[str] = None, 
                 current_oams: Optional[str] = None, 
                 current_spool_idx: Optional[int] = None):
        
        # Primary state tracking
        self.state_name = state_name  # FPSLoadState: LOADED, UNLOADED, LOADING, UNLOADING
        self.current_group = current_group  # Filament group name (T0, T1, etc.)
        self.current_oams = current_oams  # OAMS unit name currently loaded
        self.current_spool_idx = current_spool_idx  # Spool bay index (0-3)
        
        # Runout tracking
        self.runout_position: Optional[float] = None
        self.runout_after_position: Optional[float] = None
        
        # Timer references (for cleanup)
        self.monitor_spool_timer = None
        self.monitor_pause_timer = None
        self.monitor_load_next_spool_timer = None
        
        # Motion monitoring
        self.encoder_samples = deque(maxlen=ENCODER_SAMPLES)  # Recent encoder readings
        
        # Follower state
        self.following: bool = False  # Whether follower mode is active
        self.direction: int = 0  # Follower direction (0=forward, 1=reverse)
        self.since: Optional[float] = None  # Timestamp when current state began
        
    def reset_runout_positions(self) -> None:
        """Clear runout position tracking."""
        self.runout_position = None
        self.runout_after_position = None

    def __repr__(self) -> str:
        return f"FPSState(state_name={self.state_name}, current_group={self.current_group}, current_oams={self.current_oams}, current_spool_idx={self.current_spool_idx})"

    def __str__(self) -> str:
        return f"State: {self.state_name}, Group: {self.current_group}, OAMS: {self.current_oams}, Spool Index: {self.current_spool_idx}"

class OAMSManager:
    """
    Main coordinator for OpenAMS system with multiple FPS units.
    
    Manages:
    - Multiple FPS (Filament Pressure Sensor) units
    - OAMS (OpenAMS) hardware units  
    - Filament groups (T0, T1, etc.) mapping to FPS units
    - Automatic filament runout detection and switching
    
    Key Attributes:
    - fpss: Dictionary of FPS objects {fps_name: fps_object}
    - oams: Dictionary of OAMS objects {oams_name: oams_object}  
    - filament_groups: Dictionary of filament groups {group_name: group_object}
    - current_state: OAMSState tracking all FPS states
    """
    
    def __init__(self, config):
        # Core configuration and printer interface
        self.config = config
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        
        # Hardware object collections
        self.filament_groups: Dict[str, Any] = {}  # Group name -> FilamentGroup object
        self.oams: Dict[str, Any] = {}  # OAMS name -> OAMS object
        self.fpss: Dict[str, Any] = {}  # FPS name -> FPS object
        self.lane_groups: Dict[str, str] = {}  # Lane name -> filament group name
        
        # State management
        self.current_state = OAMSState()  # Tracks state of all FPS units
        
        # Monitoring and control
        self.monitor_timers: List[Any] = []  # Active monitoring timers
        self.ready: bool = False  # System initialization complete
        
        # Configuration parameters
        self.reload_before_toolhead_distance: float = config.getfloat(
            "reload_before_toolhead_distance", 0.0
        )

        # External runout callback (fps_name, new_spool_idx)
        self._runout_callback: Optional[Callable[[str, int], None]] = None

        # Initialize hardware collections
        self._initialize_oams()
        self._initialize_filament_groups()

        # Register with printer and setup event handlers
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.printer.add_object("oams_manager", self)
        self.register_commands()

    def register_runout_callback(self, callback: Callable[[str, int], None]) -> None:
        """Register a callback invoked after a runout reload attempt."""
        self._runout_callback = callback

    def _notify_runout(self, fps_name: str, spool_idx: int) -> None:
        cb = self._runout_callback
        if cb is None:
            return
        try:
            cb(fps_name, spool_idx)
        except Exception:
            logging.exception("OAMS: runout callback failed")

    def load_spool_for_lane(self, fps_name: str, oams_name: str, bay_index: int) -> bool:
        """Load a specific spool bay and restart monitoring.

        Returns ``True`` if the spool was successfully loaded.
        """
        fps_state = self.current_state.fps_state.get(fps_name)
        oam = self.oams.get(oams_name)
        if fps_state is None or oam is None:
            return False
        if not oam.is_bay_ready(bay_index):
            return False
        success, message = oam.load_spool(bay_index)
        if success:
            logging.info(
                f"OAMS: Loaded spool in bay {bay_index} of OAM {oams_name}"
            )
            fps_state.state_name = "LOADED"
            fps_state.since = self.printer.get_reactor().monotonic()
            fps_state.current_spool_idx = bay_index
            fps_state.current_oams = oams_name
            fps_state.reset_runout_positions()
            if hasattr(self, "runout_monitor"):
                self.runout_monitor.reset()
                self.runout_monitor.start()
        else:
            logging.error(
                f"OAMS: Failed to load spool in bay {bay_index} of OAM {oams_name}: {message}"
            )
        return success
        
    def get_status(self, eventtime: float) -> Dict[str, Dict[str, Any]]:
        """
        Return current status of all FPS units for monitoring.
        
        Returns:
            Dictionary with FPS names as keys, each containing:
            - current_group: Active filament group name
            - current_oams: Active OAMS unit name  
            - current_spool_idx: Active spool bay index (0-3)
            - state_name: Loading state (LOADED/UNLOADED/LOADING/UNLOADING)
            - since: Timestamp when current state began
        """
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
    
    def determine_state(self) -> None:
        """
        Analyze hardware state and update FPS state tracking.
        
        For each FPS:
        1. Check which filament group is currently loaded
        2. Identify the active OAMS unit and spool bay
        3. Update state to LOADED if filament is present
        """
        for fps_name, fps_state in self.current_state.fps_state.items():
            fps_state.current_group, current_oams, fps_state.current_spool_idx = self.determine_current_loaded_group(fps_name)
            
            if current_oams is not None:
                fps_state.current_oams = current_oams.name
            else:
                fps_state.current_oams = None
                
            if fps_state.current_oams is not None and fps_state.current_spool_idx is not None:
                fps_state.state_name = FPSLoadState.LOADED
                fps_state.since = self.reactor.monotonic()
        
    def handle_ready(self) -> None:
        """
        Initialize system when printer is ready.
        
        1. Discover and register all FPS units
        2. Create state tracking for each FPS
        3. Determine current hardware state
        4. Start monitoring timers
        """
        # Discover all FPS units in the system
        for fps_name, fps in self.printer.lookup_objects(module="fps"):
            self.fpss[fps_name] = fps
            self.current_state.add_fps_state(fps_name)
            
        if not self.fpss:
            raise ValueError("No FPS found in system, this is required for OAMS to work")
            
        # Initialize system state and start monitoring
        self.determine_state()
        self.start_monitors()
        self.ready = True

    def _initialize_oams(self) -> None:
        """Discover and register all OAMS hardware units."""
        for name, oam in self.printer.lookup_objects(module="oams"):
            # Configuration sections are typically named "oams <name>".
            # Normalize to the short name (e.g. "oams1") for dictionary keys
            # and update the OAMS object's name attribute so any later
            # references use the same identifier.
            short_name = name.split()[-1]
            oam.name = short_name
            self.oams[short_name] = oam
        
    def _initialize_filament_groups(self) -> None:
        """Discover and register all filament group configurations."""
        for name, group in self.printer.lookup_objects(module="filament_group"):
            name = name.split()[-1]  # Extract group name from full object name
            logging.info(f"OAMS: Adding group {name}")
            self.filament_groups[name] = group
    
    def determine_current_loaded_group(self, fps_name: str) -> Tuple[Optional[str], Optional[object], Optional[int]]:
        """
        Determine which filament group is currently loaded in the specified FPS.
        
        Args:
            fps_name: Name of the FPS to check
            
        Returns:
            Tuple of (group_name, oams_object, bay_index) or (None, None, None) if unloaded
            
        Process:
        1. Get the FPS object
        2. Check each filament group for loaded bays
        3. Verify the OAMS is connected to this FPS
        4. Return the first match found
        """
        fps = self.fpss.get(fps_name)
        if fps is None:
            raise ValueError(f"FPS {fps_name} not found")
            
        # Check each filament group for loaded spools
        for group_name, group in self.filament_groups.items():
            for oam, bay_index in group.bays:
                # Check if this bay has filament loaded and the OAMS is connected to this FPS
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
        #     "OAMSM_CURRENT_LOADED_GROUP",
        #     self.cmd_CURRENT_LOADED_GROUP,
        #     desc=self.cmd_CURRENT_LOADED_GROUP_help,
        # )
        
        gcode.register_command(
            "OAMSM_CLEAR_ERRORS",
            self.cmd_CLEAR_ERRORS,
            desc=self.cmd_CLEAR_ERRORS_help,
        )
    
    cmd_CLEAR_ERRORS_help = "Clear the error state of the OAMS"
    def cmd_CLEAR_ERRORS(self, gcmd):
        if len(self.monitor_timers) > 0:
            self.stop_monitors()
        for (fps_name, fps_state) in self.current_state.fps_state.items():
            fps_state.encoder_samples.clear()
        for _, oam in self.oams.items():
            oam.clear_errors()
        self.determine_state()
        self.start_monitors()
        
        return
    
    cmd_FOLLOWER_help = "Enable the follower on whatever OAMS is current loaded"
    def cmd_FOLLOWER(self, gcmd):
        enable = gcmd.get_int('ENABLE')
        if enable is None:
            gcmd.respond_info("Missing ENABLE parameter")
            return
        direction = gcmd.get_int('DIRECTION')
        if direction is None:
            gcmd.respond_info("Missing DIRECTION parameter")
            return
        fps_name = gcmd.get('FPS')
        fps_name = "fps " + fps_name
        if fps_name is None:
            gcmd.respond_info("Missing FPS parameter")
            return
        if fps_name not in self.fpss:
            gcmd.respond_info(f"FPS {fps_name} does not exist")
            return
        fps_state = self.current_state.fps_state[fps_name]
        if fps_state.state_name == "UNLOADED":
            gcmd.respond_info(f"FPS {fps_name} is already unloaded")
            return
        if fps_state.state_name == "LOADING":
            gcmd.respond_info(f"FPS {fps_name} is currently loading a spool")
            return
        if fps_state.state_name == "UNLOADING":
            gcmd.respond_info(f"FPS {fps_name} is currently unloading a spool")
            return
        self.oams[fps_state.current_oams].set_oams_follower(enable, direction)
        fps_state.following = enable
        fps_state.direction = direction
        fps_state.encoder = self.oams[fps_state.current_oams].encoder_clicks
        fps_state.current_spool_idx = self.oams[fps_state.current_oams].current_spool
        return
    
    def group_fps_name(self, group_name):
        for c_group_name, c_group in self.filament_groups.items():
            if c_group_name == group_name:
                for fps_name, fps in self.fpss.items():
                        for fps_oam in fps.oams:
                            if fps_oam in c_group.oams:
                                return fps_name
        return None

    def fps_name_for_oams(self, oams_name):
        logging.info("OAMS manager: resolving FPS for %s", oams_name)
        for fps_name, fps in self.fpss.items():
            if oams_name in fps.oams:
                logging.info(
                    "OAMS manager: %s belongs to %s", oams_name, fps_name
                )
                return fps_name
        logging.info("OAMS manager: %s not found in any FPS", oams_name)
        return None

    def group_for_lane(self, lane_name: Optional[str]) -> Optional[str]:
        """Return the filament group assigned to ``lane_name`` if any."""

        if lane_name is None:
            return None
        return self.lane_groups.get(lane_name)

    def ensure_group_for_lanes(self, group_name, lane_a, lane_b=None):
        """Ensure ``group_name`` reflects the provided AMS lanes.

        When ``lane_b`` is omitted a single-lane group is created for
        ``lane_a``.  If both lanes share the same FPS controller the group is
        updated to contain the pair.  Lane-to-group mappings are refreshed so
        OpenAMS can resolve the active filament group before a reload occurs.
        """

        if group_name is None or lane_a is None:
            return

        lanes = [lane_a]
        if lane_b is not None and lane_b is not lane_a:
            lanes.append(lane_b)

        lane_details = []
        fps_name = None
        for lane in lanes:
            unit_obj = getattr(lane, "unit_obj", None)
            oams_name = getattr(unit_obj, "oams_name", None)
            if oams_name is None:
                return
            oam = self.oams.get(oams_name)
            if oam is None:
                return
            lane_fps = self.fps_name_for_oams(oam.name)
            if lane_fps is None:
                return
            if fps_name is None:
                fps_name = lane_fps
            elif fps_name != lane_fps:
                logging.info(
                    "OAMS manager: lanes %s and %s are on different FPS (%s vs %s)",
                    lanes[0].name,
                    lane.name,
                    fps_name,
                    lane_fps,
                )
                return

            bay_count = len(getattr(oam, "f1s_hes_value", [])) or 4
            bay_index = (lane.index - 1) % bay_count
            lane_details.append((lane, oam, bay_index))

        group = self.filament_groups.get(group_name)
        if group is None:
            group = FilamentGroup.__new__(FilamentGroup)
            group.printer = self.printer
            group.group_name = group_name
            self.filament_groups[group_name] = group

        group.bays = [(oam, bay) for _, oam, bay in lane_details]
        group.oams = []
        for _, oam, _ in lane_details:
            if oam not in group.oams:
                group.oams.append(oam)

        lane_names = {lane.name for lane, _, _ in lane_details}
        for lane_name, mapped_group in list(self.lane_groups.items()):
            if mapped_group == group_name and lane_name not in lane_names:
                self.lane_groups.pop(lane_name, None)

        for lane, _, _ in lane_details:
            self.lane_groups[lane.name] = group_name

        if len(lane_details) == 1:
            oam = lane_details[0][1]
            bay = lane_details[0][2]
            logging.info(
                "OAMS manager: group %s set to %s-%s",
                group_name,
                oam.name,
                bay,
            )
        else:
            (_lane_a, oam_a, bay_a), (_lane_b, oam_b, bay_b) = lane_details
            logging.info(
                "OAMS manager: group %s set to %s-%s, %s-%s",
                group_name,
                oam_a.name,
                bay_a,
                oam_b.name,
                bay_b,
            )

        if fps_name is not None:
            fps_state = self.current_state.fps_state.get(fps_name)
            if fps_state is not None:
                for _, oam, bay in lane_details:
                    if (
                        fps_state.current_oams == oam.name
                        and fps_state.current_spool_idx == bay
                    ):
                        fps_state.current_group = group_name
                        break
    
    cmd_UNLOAD_FILAMENT_help = "Unload a spool from any of the OAMS if any is loaded"
    def cmd_UNLOAD_FILAMENT(self, gcmd):
        fps_name = gcmd.get('FPS')
        if fps_name is None:
            gcmd.respond_info("Missing FPS parameter")
            return
        fps_name = "fps " + fps_name
        if fps_name not in self.fpss:
            gcmd.respond_info(f"FPS {fps_name} does not exist")
            return
        fps_state = self.current_state.fps_state[fps_name]
        if fps_state.state_name == "UNLOADED":
            gcmd.respond_info(f"FPS {fps_name} is already unloaded")
            return
        if fps_state.state_name == "LOADING":
            gcmd.respond_info(f"FPS {fps_name} is currently loading a spool")
            return
        if fps_state.state_name == "UNLOADING":
            gcmd.respond_info(f"FPS {fps_name} is currently unloading a spool")
            return
        if fps_state.state_name == "LOADED":
            oams = self.oams[fps_state.current_oams]
            if oams is None:
                gcmd.respond_info(f"FPS {fps_name} has no OAMS loaded")
                return
            if oams.current_spool is not None:
                fps_state.state_name = "UNLOADING"
                fps_state.encoder = oams.encoder_clicks
                fps_state.since = self.reactor.monotonic()
                fps_state.current_oams = oams.name
                fps_state.current_spool_idx = oams.current_spool
            
                success, message = oams.unload_spool()
                
                if success:
                    fps_state.state_name = "UNLOADED"
                    fps_state.following = False
                    fps_state.direction = 0
                    fps_state.since = self.reactor.monotonic()
                    
                    fps_state.current_group = None
                    fps_state.current_spool_idx = None
                    return
                else:
                    gcmd.respond_info(message)
                    return
        gcmd.respond_info("No spool is loaded in any of the OAMS")
        self.current_group = None
        return
        
    cmd_LOAD_FILAMENT_help = "Load a spool from an specific group"
    def cmd_LOAD_FILAMENT(self, gcmd):
        # determine which fps this group is assigned to
        group_name = gcmd.get('GROUP')
        if group_name not in self.filament_groups:
            gcmd.respond_info(f"Group {group_name} does not exist")
            return
        fps_name = self.group_fps_name(group_name)
        fps_state = self.current_state.fps_state[fps_name]
        if self.current_state.fps_state[fps_name].state_name == "LOADED":
            gcmd.respond_info(f"Group {group_name} is already loaded")
            return
        
        for (oam, bay_index) in self.filament_groups[group_name].bays:
            if oam.is_bay_ready(bay_index):
                fps_state.state_name = "LOADING"
                fps_state.encoder = oam.encoder_clicks
                fps_state.since = self.reactor.monotonic()
                fps_state.current_oams = oam.name
                fps_state.current_spool_idx = bay_index
                
                success, message = oam.load_spool(bay_index)
                
                if success:
                    fps_state.current_group = group_name
                    fps_state.current_oams = oam.name
                    fps_state.current_spool_idx = bay_index
                    
                    
                    fps_state.state_name = "LOADED"
                    fps_state.since = self.reactor.monotonic()
                    fps_state.current_oams = oam.name
                    fps_state.current_spool_idx = bay_index
                    fps_state.following = False
                    fps_state.direction = 1
                    
                    gcmd.respond_info(message)
                    self.current_group = group_name
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
        for (fps_name, fps_state) in self.current_state.fps_state.items():
            self.monitor_timers.append(reactor.register_timer(self._monitor_unload_speed_for_fps(fps_name), reactor.NOW))
            self.monitor_timers.append(reactor.register_timer(self._monitor_load_speed_for_fps(fps_name), reactor.NOW))
            
            def _reload_callback():
                for (oam, bay_index) in self.filament_groups[fps_state.current_group].bays:
                    if oam.is_bay_ready(bay_index):
                        success, message = oam.load_spool(bay_index)
                        if success:
                            logging.info(f"OAMS: Successfully loaded spool in bay {bay_index} of OAM {oam.name}")
                            fps_state.state_name = "LOADED"
                            fps_state.since = self.reactor.monotonic()
                            fps_state.current_spool_idx = bay_index
                            fps_state.current_oams = oam.name
                            fps_state.reset_runout_positions()
                            self.runout_monitor.reset()
                            self.runout_monitor.start()
                            self._notify_runout(fps_name, bay_index)
                            return
                        else:
                            logging.error(f"OAMS: Failed to load spool: {message}")
                            break
                self._notify_runout(fps_name, -1)
                self._pause_printer_message("No spool available for group %s" % fps_state.current_group)
                self.runout_monitor.paused()
                return
            
            self.runout_monitor = OAMSRunoutMonitor(self.printer, fps_name, self.fpss[fps_name], fps_state, self.oams, _reload_callback, reload_before_toolhead_distance=self.reload_before_toolhead_distance)
            self.monitor_timers.append(self.runout_monitor.timer)
            self.runout_monitor.start()
            
        logging.info("OAMS: All monitors started")
    
    def stop_monitors(self):
        for timer in self.monitor_timers:
            self.printer.get_reactor().unregister_timer(timer)
        self.monitor_timers = []

def load_config(config):
    return OAMSManager(config)