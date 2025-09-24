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

from .oams import OAMSOpCode

# Configuration constants

PAUSE_DISTANCE = 60  # mm to pause before coasting follower
ENCODER_SAMPLES = 2  # Number of encoder samples to collect
MIN_ENCODER_DIFF = 1  # Minimum encoder difference to consider movement
FILAMENT_PATH_LENGTH_FACTOR = 1.14  # Factor for calculating filament path traversal
MONITOR_ENCODER_LOADING_SPEED_AFTER = 2.0  # seconds
MONITOR_ENCODER_PERIOD = 2.0  # seconds
MONITOR_ENCODER_UNLOADING_SPEED_AFTER = 2.0  # seconds
AFC_DELEGATION_TIMEOUT = 30.0  # seconds to suppress duplicate AFC runout triggers

UNLOAD_RETRY_NUDGE_TIME = 0.5  # seconds to nudge filament forward before retry
UNLOAD_RETRY_EXTRUDER_DISTANCE_MM = 5.0  # mm to retract with the extruder during unload retries
UNLOAD_RETRY_EXTRUDER_SPEED = 20.0  # mm/s for the unload retry extruder assist


# Clog detection defaults
CLOG_SENSITIVITY_DEFAULT = 5.0
CLOG_SENSITIVITY_MIN = 0.0
CLOG_SENSITIVITY_MAX = 10.0
CLOG_MONITOR_PERIOD = 1.0  # seconds between clog samples while printing
CLOG_WINDOW_MIN_MM = 12.0
CLOG_WINDOW_MAX_MM = 48.0
CLOG_ENCODER_DELTA_MIN = 3.0
CLOG_ENCODER_DELTA_MAX = 15.0
# Pressure tolerance (+/- window) around the target FPS value that still counts as
# "on target" for clog detection. Hardware regulates around ~0.5, so we treat
# sustained readings within this window as nominal load pressure.
CLOG_PRESSURE_OFFSET_MIN = 0.10
CLOG_PRESSURE_OFFSET_MAX = 0.30
CLOG_DWELL_MIN = 4.0
CLOG_DWELL_MAX = 14.0
CLOG_RETRACTION_TOLERANCE_MM = 0.8

# Spool jam detection
STUCK_SPOOL_PRESSURE_TRIGGER = 0.08  # Pressure level indicating the spool is likely stuck



# Default retry behaviour for unload recovery
UNLOAD_RETRY_ERROR_CODE = 3




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
    - Triggers reload once the remaining filament in the tube reaches the
      configured safety margin, independent of the total PTFE length. Each FPS
      can optionally override the safety margin so coasting distance can be
      tuned per extruder lane.
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
        self.runout_after_position: Optional[float] = None
        
        # Configuration
        # Reload is triggered as soon as the follower has <= this much filament
        # left in the tube after the BLDC clear phase, regardless of the OAMS
        # unit's total PTFE length. This ensures AMS2-style lanes swap as soon
        # as the configured safety margin is reached.
        self.reload_before_toolhead_distance = reload_before_toolhead_distance
        self.reload_callback = reload_callback
        
        self.reactor = self.printer.get_reactor()

        def _monitor_runout(eventtime):
            idle_timeout = self.printer.lookup_object("idle_timeout")
            is_printing = idle_timeout.get_status(eventtime)["state"] == "Printing"
            
            if self.state == OAMSRunoutState.STOPPED or self.state == OAMSRunoutState.PAUSED or self.state == OAMSRunoutState.RELOADING:
                pass

            elif self.state == OAMSRunoutState.MONITORING:
                #logging.info("OAMS: Monitoring runout, is_printing: %s, fps_state: %s, fps_state.current_group: %s, fps_state.current_spool_idx: %s, oams: %s" % (is_printing, fps_state.state_name, fps_state.current_group, fps_state.current_spool_idx, fps_state.current_oams))
                if getattr(fps_state, "afc_delegation_active", False):
                    now = self.reactor.monotonic()
                    if now < getattr(fps_state, "afc_delegation_until", 0.0):
                        return eventtime + MONITOR_ENCODER_PERIOD
                    fps_state.afc_delegation_active = False
                    fps_state.afc_delegation_until = 0.0
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
                    self.runout_after_position = 0.0
                    self.state = OAMSRunoutState.COASTING

            elif self.state == OAMSRunoutState.COASTING:
                traveled_distance_after_bldc_clear = max(
                    fps.extruder.last_position - self.bldc_clear_position, 0.0
                )
                self.runout_after_position = traveled_distance_after_bldc_clear
                path_length = getattr(
                    self.oams[fps_state.current_oams], "filament_path_length", 0.0
                )
                effective_path_length = (
                    path_length / FILAMENT_PATH_LENGTH_FACTOR if path_length else 0.0
                )
                consumed_with_margin = (
                    self.runout_after_position
                    + PAUSE_DISTANCE
                    + self.reload_before_toolhead_distance
                )

                if consumed_with_margin >= effective_path_length:
                    logging.info(
                        "OAMS: Loading next spool (%.2f mm consumed + margin %.2f mm >= effective path %.2f mm).",
                        self.runout_after_position + PAUSE_DISTANCE,
                        self.reload_before_toolhead_distance,
                        effective_path_length,
                    )
                    self.state = OAMSRunoutState.RELOADING
                    self.reload_callback()
            else:
                raise ValueError(f"Invalid state: {self.state}")
            return eventtime + MONITOR_ENCODER_PERIOD
        self._timer_callback = _monitor_runout
        self.timer = self.reactor.register_timer(self._timer_callback, self.reactor.NOW)
        

    def start(self) -> None:
        """Start monitoring for filament runout."""
        if self.timer is None:
            self.timer = self.reactor.register_timer(self._timer_callback, self.reactor.NOW)
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
            self.reactor.unregister_timer(self.timer)
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
        self.encoder: Optional[float] = None

        # Follower state
        self.following: bool = False  # Whether follower mode is active
        self.direction: int = 0  # Follower direction (0=forward, 1=reverse)
        self.since: Optional[float] = None  # Timestamp when current state began

        # AFC delegation state
        self.afc_delegation_active: bool = False
        self.afc_delegation_until: float = 0.0

        # Clog detection tracker
        self.clog_extruder_start: Optional[float] = None
        self.clog_encoder_start: Optional[float] = None
        self.clog_last_extruder: Optional[float] = None
        self.clog_last_encoder: Optional[float] = None
        self.clog_extruder_delta: float = 0.0
        self.clog_encoder_delta: float = 0.0
        self.clog_max_pressure: float = 0.0
        self.clog_min_pressure: float = 1.0
        self.clog_start_time: Optional[float] = None

        # Stuck spool detection tracker
        self.stuck_spool_start_time: Optional[float] = None
        self.stuck_spool_active: bool = False
        self.stuck_spool_last_oams: Optional[str] = None
        self.stuck_spool_last_spool_idx: Optional[int] = None
        self.stuck_spool_led_asserted: bool = False
        self.stuck_spool_should_restore_follower: bool = False
        self.stuck_spool_restore_direction: int = 0

        self.reset_stuck_spool_state()
        self.reset_clog_tracker()


    def reset_runout_positions(self) -> None:
        """Clear runout position tracking."""
        self.runout_position = None
        self.runout_after_position = None
        self.reset_clog_tracker()
        self.reset_stuck_spool_state()

    def reset_clog_tracker(self) -> None:
        """Reset clog detection accumulation state."""
        self.clog_extruder_start = None
        self.clog_encoder_start = None
        self.clog_last_extruder = None
        self.clog_last_encoder = None
        self.clog_extruder_delta = 0.0
        self.clog_encoder_delta = 0.0
        self.clog_max_pressure = 0.0
        self.clog_min_pressure = 1.0
        self.clog_start_time = None
        self.stuck_spool_start_time = None

    def reset_stuck_spool_state(self) -> None:
        """Clear stuck spool detection latches and history."""
        self.stuck_spool_start_time = None
        self.stuck_spool_active = False
        self.stuck_spool_last_oams = None
        self.stuck_spool_last_spool_idx = None
        self.stuck_spool_led_asserted = False
        self.stuck_spool_should_restore_follower = False
        self.stuck_spool_restore_direction = 0

    def prime_clog_tracker(
        self,
        extruder_position: float,
        encoder_position: float,
        timestamp: float,
        pressure: float,
    ) -> None:
        """Initialize clog tracking with the current motion sample."""
        self.clog_extruder_start = extruder_position
        self.clog_encoder_start = encoder_position
        self.clog_last_extruder = extruder_position
        self.clog_last_encoder = encoder_position
        self.clog_extruder_delta = 0.0
        self.clog_encoder_delta = 0.0
        clamped_pressure = max(pressure, 0.0)
        self.clog_max_pressure = clamped_pressure
        self.clog_min_pressure = clamped_pressure
        self.clog_start_time = timestamp

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
        self.pause_resume = self.printer.lookup_object("pause_resume")
        self.print_stats = self.printer.lookup_object("print_stats", None)
        self.toolhead = self.printer.lookup_object("toolhead", None)


        # Hardware object collections
        self.filament_groups: Dict[str, Any] = {}  # Group name -> FilamentGroup object
        self.oams: Dict[str, Any] = {}  # OAMS name -> OAMS object
        self.fpss: Dict[str, Any] = {}  # FPS name -> FPS object

        # State management
        self.current_state = OAMSState()  # Tracks state of all FPS units
        self.current_group: Optional[str] = None  # Last group requested via load command
        self.afc = None  # Optional reference to AFC for lane runout mappings
        self._afc_logged = False  # Tracks whether AFC integration has been announced

        

        # Monitoring and control
        self.monitor_timers: List[Any] = []  # Active monitoring timers
        self.runout_monitors: Dict[str, OAMSRunoutMonitor] = {}
        self.ready: bool = False  # System initialization complete

        # Configuration parameters
        self.reload_before_toolhead_distance: float = config.getfloat(
            "reload_before_toolhead_distance",
            0.0,
        )

        # Automatic unload retry configuration
        self.unload_retry_enabled: bool = config.getboolean(
            "unload_retry_enabled",
            True,
        )
        self.unload_retry_push_distance: float = config.getfloat(
            "unload_retry_push_distance",
            2.0,
        )
        self.unload_retry_push_speed: float = config.getfloat(
            "unload_retry_push_speed",
            25.0,
            minval=0.1,
        )

        self.unload_retry_extruder_distance_mm: float = config.getfloat(

            "unload_retry_extruder_distance_mm",
            UNLOAD_RETRY_EXTRUDER_DISTANCE_MM,
            minval=0.0,
        )

        self.unload_retry_extruder_speed: float = config.getfloat(

            "unload_retry_extruder_speed",
            UNLOAD_RETRY_EXTRUDER_SPEED,
            minval=0.0,
        )


        raw_clog_sensitivity = config.getfloat(
            "clog_sensitivity",
            CLOG_SENSITIVITY_DEFAULT,
        )
        clamped_sensitivity = max(
            CLOG_SENSITIVITY_MIN,
            min(raw_clog_sensitivity, CLOG_SENSITIVITY_MAX),
        )
        if clamped_sensitivity != raw_clog_sensitivity:
            logging.warning(
                "OAMS: clog_sensitivity %.2f outside %.2f-%.2f, clamped to %.2f",
                raw_clog_sensitivity,
                CLOG_SENSITIVITY_MIN,
                CLOG_SENSITIVITY_MAX,
                clamped_sensitivity,
            )

        self.clog_sensitivity: float = clamped_sensitivity
        self.clog_detection_enabled: bool = self.clog_sensitivity > CLOG_SENSITIVITY_MIN
        self.clog_monitor_period: float = config.getfloat(
            "clog_monitor_period",
            CLOG_MONITOR_PERIOD,
            minval=0.1,
        )

        if CLOG_SENSITIVITY_MAX > CLOG_SENSITIVITY_MIN:
            sensitivity_scale = (
                (self.clog_sensitivity - CLOG_SENSITIVITY_MIN)
                / (CLOG_SENSITIVITY_MAX - CLOG_SENSITIVITY_MIN)
            )
        else:
            sensitivity_scale = 0.0

        window_span = max(CLOG_WINDOW_MAX_MM - CLOG_WINDOW_MIN_MM, 0.0)
        encoder_span = max(CLOG_ENCODER_DELTA_MAX - CLOG_ENCODER_DELTA_MIN, 0.0)
        pressure_span = max(CLOG_PRESSURE_OFFSET_MAX - CLOG_PRESSURE_OFFSET_MIN, 0.0)
        dwell_span = max(CLOG_DWELL_MAX - CLOG_DWELL_MIN, 0.0)

        self.clog_extruder_window_mm: float = max(
            0.0,
            CLOG_WINDOW_MAX_MM - sensitivity_scale * window_span,
        )
        self.clog_encoder_delta_limit: float = max(
            0.0,
            CLOG_ENCODER_DELTA_MAX - sensitivity_scale * encoder_span,
        )
        self.clog_pressure_offset: float = max(
            0.0,
            CLOG_PRESSURE_OFFSET_MAX - sensitivity_scale * pressure_span,
        )
        self.clog_dwell_time: float = max(
            0.0,
            CLOG_DWELL_MAX - sensitivity_scale * dwell_span,
        )
        self.stuck_spool_dwell_time: float = max(0.0, self.clog_dwell_time * 0.5)
        self.clog_retraction_tolerance_mm: float = max(
            0.0,
            CLOG_RETRACTION_TOLERANCE_MM,
        )

        logging.debug(
            "OAMS: clog detection sensitivity %.2f -> window %.1fmm, encoder slack %.1f, pressure window +/-%.2f, dwell %.1fs",
            self.clog_sensitivity,
            self.clog_extruder_window_mm,
            self.clog_encoder_delta_limit,
            self.clog_pressure_offset,
            self.clog_dwell_time,
        )


        # Cached mappings
        self.group_to_fps: Dict[str, str] = {}
        self._canonical_lane_by_group: Dict[str, str] = {}
        self._canonical_group_by_lane: Dict[str, str] = {}
        self._lane_unit_map: Dict[str, str] = {}
        self._lane_by_location: Dict[Tuple[str, int], str] = {}

        
        # Initialize hardware collections
        self._initialize_oams()
        self._initialize_filament_groups()
        
        # Register with printer and setup event handlers
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.printer.register_event_handler(
            "gcode:command_AFC_RESUME", self._handle_resume_command
        )
        self.printer.register_event_handler(
            "gcode:command_RESUME", self._handle_resume_command
        )
        self.printer.add_object("oams_manager", self)
        self.register_commands()
        

    def get_status(self, eventtime: float) -> Dict[str, Dict[str, Any]]:
        """
        Return current status of all FPS units and OAMS hardware for monitoring.

        Returns:
            Dictionary containing:
            - "oams": Mapping of OAMS identifiers to their latest action status
            - One entry per FPS with its current loading state information
        """
        attributes: Dict[str, Dict[str, Any]] = {"oams": {}}

        for name, oam in self.oams.items():
            status_name = name.split()[-1]
            oam_status = {
                "action_status": oam.action_status,
                "action_status_code": oam.action_status_code,
                "action_status_value": oam.action_status_value,
            }
            attributes["oams"][status_name] = oam_status
            if status_name != name:
                attributes["oams"][name] = oam_status

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
                if fps_state.direction not in (0, 1) or fps_state.direction == 0:
                    fps_state.direction = 1
        
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

        self._rebuild_group_fps_index()

        # Initialize system state and start monitoring
        self.determine_state()
        self.start_monitors()

        self.ready = True

    def _initialize_oams(self) -> None:
        """Discover and register all OAMS hardware units."""
        for name, oam in self.printer.lookup_objects(module="oams"):
            self.oams[name] = oam
        
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

    def _clear_all_errors(self) -> None:
        """Clear error flags on all OAMS units and restart monitors."""
        if self.monitor_timers:
            self.stop_monitors()
        for _, fps_state in self.current_state.fps_state.items():
            fps_state.encoder_samples.clear()
            fps_state.reset_clog_tracker()
            self._clear_stuck_spool_state(fps_state)
        for _, oam in self.oams.items():
            oam.clear_errors()
        self.determine_state()
        self.start_monitors()

    cmd_CLEAR_ERRORS_help = "Clear the error state of the OAMS"
    def cmd_CLEAR_ERRORS(self, gcmd):
        self._clear_all_errors()
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
    

    def _rebuild_group_fps_index(self) -> None:
        """Build a lookup table from filament groups to their owning FPS."""
        mapping: Dict[str, str] = {}
        for group_name, group in self.filament_groups.items():
            for fps_name, fps in self.fpss.items():
                if any(oam in fps.oams for oam in group.oams):
                    mapping[group_name] = fps_name
                    break
        self.group_to_fps = mapping

    def group_fps_name(self, group_name: str) -> Optional[str]:
        if group_name not in self.group_to_fps and self.fpss:
            self._rebuild_group_fps_index()
        return self.group_to_fps.get(group_name)

    def _normalize_group_name(self, group: Optional[str]) -> Optional[str]:
        """Return a trimmed filament group name or None if invalid."""
        if not group or not isinstance(group, str):
            return None
        group = group.strip()
        if not group:
            return None
        if " " in group:
            group = group.split()[-1]
        return group

    def _rebuild_lane_location_index(self) -> None:
        """Map each (OAMS name, bay index) tuple to its canonical AFC lane."""
        mapping: Dict[Tuple[str, int], str] = {}
        for group_name, lane_name in self._canonical_lane_by_group.items():
            group = self.filament_groups.get(group_name)
            if not group:
                continue
            for oam, bay_index in group.bays:
                mapping[(oam.name, bay_index)] = lane_name
        self._lane_by_location = mapping

    def _ensure_afc_lane_cache(self, afc) -> None:
        """Capture the canonical AFC lane mapping when AFC is available."""
        lanes = getattr(afc, "lanes", {})
        updated = False
        for lane_name, lane in lanes.items():
            canonical_group = self._normalize_group_name(getattr(lane, "_map", None))
            if canonical_group is None:
                canonical_group = self._normalize_group_name(getattr(lane, "map", None))
            if canonical_group:
                if lane_name not in self._canonical_group_by_lane:
                    self._canonical_group_by_lane[lane_name] = canonical_group
                    updated = True
                if canonical_group not in self._canonical_lane_by_group:
                    self._canonical_lane_by_group[canonical_group] = lane_name
                    updated = True
            unit_name = getattr(lane, "unit", None)
            if unit_name and lane_name not in self._lane_unit_map:
                self._lane_unit_map[lane_name] = unit_name
        if updated:
            self._rebuild_lane_location_index()

    def _resolve_lane_for_state(
        self,
        fps_state: 'FPSState',
        group_name: Optional[str],
        afc,
    ) -> Tuple[Optional[str], Optional[str]]:
        """Determine the canonical AFC lane and group for the provided FPS state."""

        normalized_group = self._normalize_group_name(group_name)
        lane_name: Optional[str] = None

        # Prefer the physical OAMS location currently tracked by the FPS state.
        if fps_state.current_oams and fps_state.current_spool_idx is not None:
            lane_name = self._lane_by_location.get(
                (fps_state.current_oams, fps_state.current_spool_idx)
            )
            if lane_name:
                lane_group = self._canonical_group_by_lane.get(lane_name)
                if lane_group:
                    normalized_group = lane_group

        # Fall back to the canonical mapping captured from AFC at startup.
        if lane_name is None and normalized_group:
            lane_name = self._canonical_lane_by_group.get(normalized_group)

        lanes = getattr(afc, "lanes", {})

        # As a last resort, inspect the lanes directly using their original map assignments.
        if lane_name is None and normalized_group:
            lane_name = next(
                (
                    name
                    for name, lane in lanes.items()
                    if self._normalize_group_name(getattr(lane, "_map", None))
                    == normalized_group
                ),
                None,
            )

        canonical_group = normalized_group
        if lane_name:
            lane = lanes.get(lane_name)
            if lane is not None:
                canonical_candidate = self._normalize_group_name(
                    getattr(lane, "_map", None)
                )
                if canonical_candidate is None:
                    canonical_candidate = self._normalize_group_name(
                        getattr(lane, "map", None)
                    )

                updated = False
                if canonical_candidate:
                    canonical_group = canonical_candidate
                    if lane_name not in self._canonical_group_by_lane:
                        self._canonical_group_by_lane[lane_name] = canonical_candidate
                        updated = True
                    if canonical_candidate not in self._canonical_lane_by_group:
                        self._canonical_lane_by_group[canonical_candidate] = lane_name
                        updated = True
                unit_name = getattr(lane, "unit", None)
                if unit_name and lane_name not in self._lane_unit_map:
                    self._lane_unit_map[lane_name] = unit_name
                if updated:
                    self._rebuild_lane_location_index()

        return lane_name, canonical_group

    def _get_afc(self):
        """Lazily retrieve the AFC object if it is available."""
        if self.afc is not None:
            return self.afc
        try:
            afc = self.printer.lookup_object('AFC')
        except Exception:
            self.afc = None
            return None
        self.afc = afc
        self._ensure_afc_lane_cache(afc)
        if not self._afc_logged:
            logging.info("OAMS: AFC integration detected; enabling same-FPS infinite runout support.")
            self._afc_logged = True
        return self.afc

    def _get_infinite_runout_target_group(
        self,
        fps_name: str,
        fps_state: 'FPSState',
    ) -> Tuple[Optional[str], Optional[str], bool, Optional[str]]:
        """
        Return the target filament group and lane for infinite runout, if configured.

        The third element of the tuple indicates whether the runout handling should be
        delegated back to AFC (for example when the configured runout lane is not on
        the same FPS and therefore cannot be handled by OAMS directly).
        """
        current_group = fps_state.current_group
        normalized_group = self._normalize_group_name(current_group)
        if normalized_group is None:
            return None, None, False, None

        afc = self._get_afc()
        if afc is None:
            return None, None, False, None

        lane_name, resolved_group = self._resolve_lane_for_state(
            fps_state,
            normalized_group,
            afc,
        )

        if resolved_group and resolved_group != normalized_group:
            normalized_group = resolved_group
            fps_state.current_group = resolved_group

        if not lane_name:
            logging.debug(
                "OAMS: Unable to resolve AFC lane for group %s on %s",
                normalized_group,
                fps_name,
            )
            return None, None, False, None

        lanes = getattr(afc, "lanes", {})
        lane = afc.lanes.get(lane_name)
        if lane is None:
            return None, None, False, lane_name

        runout_lane_name = getattr(lane, "runout_lane", None)
        if not runout_lane_name:
            return None, None, False, lane_name

        target_lane = afc.lanes.get(runout_lane_name)
        if target_lane is None:
            logging.warning(
                "OAMS: Runout lane %s for %s on %s is not available; deferring to AFC",
                runout_lane_name,
                normalized_group,
                fps_name,
            )
            return None, runout_lane_name, True, lane_name

        source_unit = self._lane_unit_map.get(lane_name)
        target_unit = self._lane_unit_map.get(runout_lane_name)
        if source_unit and target_unit and source_unit != target_unit:
            logging.debug(
                "OAMS: Runout lane %s (%s) for %s on %s belongs to different unit %s; deferring to AFC",
                runout_lane_name,
                target_unit,
                normalized_group,
                fps_name,
                source_unit,
            )
            return None, runout_lane_name, True, lane_name

        source_extruder = getattr(lane, "extruder_obj", None)
        target_extruder = getattr(target_lane, "extruder_obj", None)
        if (
            source_extruder is not None
            and target_extruder is not None
            and source_extruder is not target_extruder
        ):
            logging.debug(
                "OAMS: Deferring infinite runout for %s on %s because lane %s (%s) spools to %s (%s)",
                normalized_group,
                fps_name,
                lane_name,
                getattr(source_extruder, "name", "unknown"),
                runout_lane_name,
                getattr(target_extruder, "name", "unknown"),
            )
            return None, runout_lane_name, True, lane_name

        target_group = self._canonical_group_by_lane.get(runout_lane_name)
        if not target_group:
            target_group = self._normalize_group_name(getattr(target_lane, "_map", None))
        if not target_group:
            target_group = self._normalize_group_name(getattr(target_lane, "map", None))

        if not target_group:
            logging.debug(
                "OAMS: Runout lane %s for %s on %s has no canonical group; deferring to AFC",
                runout_lane_name,
                normalized_group,
                fps_name,
            )
            return None, runout_lane_name, True, lane_name

        updated = False
        if runout_lane_name not in self._canonical_group_by_lane:
            self._canonical_group_by_lane[runout_lane_name] = target_group
            updated = True
        if target_group not in self._canonical_lane_by_group:
            self._canonical_lane_by_group[target_group] = runout_lane_name
            updated = True
        if updated:
            self._rebuild_lane_location_index()

        if target_group == normalized_group:
            logging.debug(
                "OAMS: Runout lane %s for %s on %s does not map to a different filament group; deferring to AFC",
                runout_lane_name,
                normalized_group,
                fps_name,
            )
            return None, runout_lane_name, True, lane_name

        if normalized_group not in self.filament_groups:
            logging.debug(
                "OAMS: Source group %s is not managed by OAMS; deferring to AFC",
                normalized_group,
            )
            return None, runout_lane_name, True, lane_name

        if target_group not in self.filament_groups:
            logging.debug(
                "OAMS: Runout mapping %s -> %s is not managed by OAMS; deferring to AFC",
                normalized_group,
                target_group,
            )
            return None, runout_lane_name, True, lane_name

        source_fps = self.group_fps_name(normalized_group)
        target_fps = self.group_fps_name(target_group)
        if source_fps != fps_name or target_fps != fps_name:
            logging.info(
                "OAMS: Deferring infinite runout for %s on %s to AFC lane %s because target group %s loads via %s",
                normalized_group,
                fps_name,
                runout_lane_name,
                target_group,
                target_fps or "unknown FPS",
            )
            return None, runout_lane_name, True, lane_name

        logging.info(
            "OAMS: Infinite runout configured for %s on %s -> %s (lanes %s -> %s)",
            normalized_group,
            fps_name,
            target_group,
            lane_name,
            runout_lane_name,
        )
        return target_group, runout_lane_name, False, lane_name

    def _delegate_runout_to_afc(
        self,
        fps_name: str,
        fps_state: 'FPSState',
        source_lane_name: Optional[str],
        target_lane_name: Optional[str],
    ) -> bool:
        """Ask AFC to perform the infinite runout swap for the provided lane."""

        afc = self._get_afc()
        if afc is None:
            logging.debug(
                "OAMS: Cannot delegate infinite runout for %s; AFC not available",
                fps_name,
            )
            return False

        if not source_lane_name:
            logging.debug(
                "OAMS: Cannot delegate infinite runout for %s; no source lane recorded",
                fps_name,
            )
            return False

        lane = afc.lanes.get(source_lane_name)
        if lane is None:
            logging.warning(
                "OAMS: AFC lane %s not found while delegating infinite runout for %s",
                source_lane_name,
                fps_name,
            )
            return False

        runout_target = getattr(lane, "runout_lane", None)
        if not runout_target:
            logging.warning(
                "OAMS: AFC lane %s has no runout target while delegating infinite runout for %s",
                source_lane_name,
                fps_name,
            )
            return False

        if target_lane_name and target_lane_name != runout_target:
            logging.debug(
                "OAMS: AFC lane %s runout target mismatch (%s != %s) while delegating infinite runout for %s",
                source_lane_name,
                runout_target,
                target_lane_name,
                fps_name,
            )

        now = self.reactor.monotonic()
        if fps_state.afc_delegation_active and now < fps_state.afc_delegation_until:
            logging.debug(
                "OAMS: AFC infinite runout for %s still in progress; skipping duplicate trigger",
                fps_name,
            )
            return True

        if runout_target not in afc.lanes:
            logging.warning(
                "OAMS: AFC runout lane %s referenced by %s is unavailable",
                runout_target,
                source_lane_name,
            )
            return False

        try:
            lane._perform_infinite_runout()
        except Exception:
            logging.exception(
                "OAMS: AFC infinite runout failed for lane %s -> %s",
                source_lane_name,
                runout_target,
            )
            fps_state.afc_delegation_active = False
            fps_state.afc_delegation_until = 0.0
            return False

        fps_state.afc_delegation_active = True
        fps_state.afc_delegation_until = now + AFC_DELEGATION_TIMEOUT
        logging.info(
            "OAMS: Delegated infinite runout for %s via AFC lane %s -> %s",
            fps_name,
            source_lane_name,
            runout_target,
        )
        return True

    def _clear_error_state_for_retry(self, fps_state, oams):
        """Clear error state prior to an automatic unload retry."""
        try:
            oams.clear_errors()
        except Exception:
            logging.exception(
                "OAMS: Failed to clear errors on %s prior to unload retry",
                getattr(oams, "name", "unknown"),
            )
        fps_state.encoder_samples.clear()
        fps_state.reset_clog_tracker()

    def _nudge_filament_before_retry(self, oams, direction: int = 1,

                                     duration: Optional[float] = None) -> None:
        """Briefly move the filament to relieve tension before retrying."""
        if duration is None:
            duration = globals().get("UNLOAD_RETRY_NUDGE_TIME", 0.5)


        if duration <= 0 or not hasattr(oams, "set_oams_follower"):
            return

        enable_sent = False
        try:
            logging.info(
                "OAMS: Nudging filament forward on %s for %.2f seconds before retry",
                getattr(oams, "name", "unknown"),
                duration,
            )
            oams.set_oams_follower(1, direction)
            enable_sent = True
            self.reactor.pause(self.reactor.monotonic() + duration)
        except Exception:
            logging.exception(
                "OAMS: Failed while nudging filament on %s",
                getattr(oams, "name", "unknown"),
            )
        finally:
            if enable_sent:
                try:
                    oams.set_oams_follower(0, direction)
                except Exception:
                    logging.exception(
                        "OAMS: Failed to stop follower on %s after nudge",
                        getattr(oams, "name", "unknown"),
                    )


    def _assist_retry_with_extruder(
        self, fps_name: str, oams
    ) -> Optional[Callable[[], None]]:
        """Retract filament with the extruder prior to an unload retry.

        Returns a callback that will wait for the queued move to finish, or
        ``None`` if no assist move was scheduled.
        """

        distance = float(self.unload_retry_extruder_distance_mm or 0.0)
        speed = float(self.unload_retry_extruder_speed or 0.0)

        if distance <= 0.0 or speed <= 0.0:
            return None


        fps = self.fpss.get(fps_name)
        if fps is None:
            logging.debug(
                "OAMS: Skipping unload retry extruder assist; FPS %s not available",
                fps_name,
            )

            return None


        extruder = getattr(fps, "extruder", None)
        if extruder is None:
            logging.debug(
                "OAMS: Skipping unload retry extruder assist for %s; no extruder bound",
                fps_name,
            )

            return None


        heater = None
        get_heater = getattr(extruder, "get_heater", None)
        if callable(get_heater):
            try:
                heater = get_heater()
            except Exception:
                logging.exception(
                    "OAMS: Unable to query heater for extruder assist on %s",
                    fps_name,
                )

                return None

        if heater is None:
            heater = getattr(extruder, "heater", None)

        if heater is not None and not getattr(heater, "can_extrude", True):
            logging.info(
                "OAMS: Skipping unload retry extruder assist for %s; heater below minimum extrude temp",
                fps_name,
            )

            return None


        try:
            gcode_move = self.printer.lookup_object("gcode_move")
            toolhead = self.printer.lookup_object("toolhead")
        except Exception:
            logging.debug(
                "OAMS: Skipping unload retry extruder assist for %s; gcode_move/toolhead unavailable",
                fps_name,
            )

            return None

        if gcode_move is None or toolhead is None:
            logging.debug(
                "OAMS: Skipping unload retry extruder assist for %s; gcode_move/toolhead unavailable",
                fps_name,
            )

            return None


        last_position = getattr(gcode_move, "last_position", None)
        if not isinstance(last_position, (list, tuple)) or len(last_position) < 4:
            logging.debug(
                "OAMS: Skipping unload retry extruder assist for %s; invalid gcode position",
                fps_name,
            )

            return None

        new_position = list(last_position)
        extrude_factor = getattr(gcode_move, "extrude_factor", 1.0) or 1.0
        new_position[3] -= distance * extrude_factor

        follower_enabled = False
        move_queued = False
        wait_callback: Optional[Callable[[], None]] = None

        def disable_follower():
            nonlocal follower_enabled
            if follower_enabled:
                try:
                    oams.set_oams_follower(0, 0)
                except Exception:
                    logging.exception(
                        "OAMS: Failed to disable follower after extruder assist on %s",
                        getattr(oams, "name", "unknown"),
                    )
                finally:
                    follower_enabled = False

        extruder_name = getattr(extruder, "name", getattr(fps, "extruder_name", "extruder"))
        try:
            logging.info(
                "OAMS: Assisting unload retry for %s by retracting %.3fmm on %s at %.3f mm/s",
                fps_name,
                distance,
                extruder_name,
                speed,
            )
            oams.set_oams_follower(1, 0)
            follower_enabled = True
            gcode_move.move_with_transform(new_position, speed)
            gcode_move.last_position = new_position
            move_queued = True

            maybe_wait = getattr(toolhead, "wait_moves", None)

            def wait_and_sync(
                maybe_wait=maybe_wait, gcode_move=gcode_move, fps_name=fps_name
            ):
                try:
                    if callable(maybe_wait):
                        maybe_wait()
                finally:
                    try:
                        gcode_move.reset_last_position()
                    except Exception:
                        logging.exception(
                            "OAMS: Failed to reset last position after extruder assist on %s",
                            fps_name,
                        )
                    finally:
                        disable_follower()

            wait_callback = wait_and_sync
        finally:
            if not move_queued:
                disable_follower()


        return wait_callback if move_queued else None


    def _recover_unload_failure(
        self,
        fps_name: str,
        fps_state,
        oams,
        failure_message: str,
    ) -> Tuple[bool, str]:
        """Attempt to recover from a failed unload caused by OAMS error code 3."""

        action_code = getattr(oams, "action_status_code", None)
        if action_code != OAMSOpCode.SPOOL_ALREADY_IN_BAY:
            return False, failure_message

        logging.warning(
            "OAMS: Unload failed on %s for %s with code %s; attempting recovery",
            fps_name,
            getattr(oams, "name", "unknown"),
            action_code,
        )

        self._clear_error_state_for_retry(fps_state, oams)
        self._nudge_filament_before_retry(oams)

        wait_for_assist: Optional[Callable[[], None]] = None
        try:
            wait_for_assist = self._assist_retry_with_extruder(fps_name, oams)

        except Exception:
            logging.exception(
                "OAMS: Extruder assist failed prior to unload retry for %s on %s",
                fps_name,
                getattr(oams, "name", "unknown"),
            )

        fps_state.encoder = oams.encoder_clicks
        fps_state.since = self.reactor.monotonic()

        try:
            retry_success, retry_message = oams.unload_spool()
        finally:
            if wait_for_assist is not None:
                try:
                    wait_for_assist()
                except Exception:
                    logging.exception(
                        "OAMS: Error while waiting for extruder assist moves to finish for %s",
                        fps_name,
                    )

        if retry_success:
            logging.info(
                "OAMS: Automatic unload retry succeeded on %s for %s",
                fps_name,
                getattr(oams, "name", "unknown"),
            )
            return True, retry_message

        combined_message = retry_message or failure_message
        logging.warning(
            "OAMS: Automatic unload retry failed on %s for %s with code %s: %s",
            fps_name,
            getattr(oams, "name", "unknown"),
            getattr(oams, "action_status_code", None),
            combined_message,
        )
        return False, combined_message

    def _abort_stalled_load_and_retry(self, fps_name: str, fps_state, oams) -> bool:
        """Abort a stalled load attempt and schedule a retry for the same group."""

        if oams is None:
            logging.error("OAMS: Cannot abort stalled load on %s; no OAMS available", fps_name)
            return False

        spool_idx = fps_state.current_spool_idx
        if spool_idx is None:
            logging.error(
                "OAMS: Cannot abort stalled load on %s; spool index is undefined",
                fps_name,
            )
            return False

        group_name: Optional[str] = None
        for candidate_group, group in self.filament_groups.items():
            for candidate_oam, candidate_bay in getattr(group, "bays", []):
                if candidate_oam is oams and candidate_bay == spool_idx:
                    group_name = candidate_group
                    break
            if group_name:
                break

        if group_name is None:
            logging.error(
                "OAMS: Unable to determine filament group for stalled load on %s spool %s",
                fps_name,
                spool_idx,
            )
            return False

        oams_name = getattr(oams, "name", fps_state.current_oams or "unknown")
        logging.warning(
            "OAMS: Load stalled on %s (%s spool %s); aborting before retry",
            fps_name,
            oams_name,
            spool_idx,
        )

        try:
            unload_success, unload_message = oams.unload_spool()
        except Exception:
            logging.exception(
                "OAMS: Exception while aborting stalled load on %s spool %s",
                oams_name,
                spool_idx,
            )
            return False

        if not unload_success:
            logging.error(
                "OAMS: Failed to abort stalled load on %s spool %s: %s",
                oams_name,
                spool_idx,
                unload_message,
            )
            return False

        if unload_message and unload_message != "Spool unloaded successfully":
            logging.info(
                "OAMS: Abort unload for stalled load on %s spool %s reported: %s",
                oams_name,
                spool_idx,
                unload_message,
            )

        fps_state.state_name = FPSLoadState.UNLOADED
        fps_state.current_group = None
        fps_state.current_spool_idx = None
        fps_state.current_oams = None
        fps_state.following = False
        fps_state.direction = 0
        fps_state.encoder = oams.encoder_clicks
        fps_state.encoder_samples.clear()
        fps_state.reset_clog_tracker()
        fps_state.since = self.reactor.monotonic()

        if fps_state.monitor_load_next_spool_timer is not None:
            try:
                self.reactor.unregister_timer(fps_state.monitor_load_next_spool_timer)
            except Exception:
                logging.exception(
                    "OAMS: Failed to cancel existing load retry timer for %s",
                    fps_name,
                )
            finally:
                fps_state.monitor_load_next_spool_timer = None

        retry_delay = MONITOR_ENCODER_PERIOD

        def _retry_load(
            self,
            eventtime,
            fps_name=fps_name,
            group_name=group_name,
            spool_idx=spool_idx,
            oams=oams,
            fps_state=fps_state,
        ):
            fps_state.monitor_load_next_spool_timer = None
            try:
                success, message = self._load_filament_for_group(group_name)
            except Exception:
                logging.exception(
                    "OAMS: Unexpected error while retrying load for group %s on %s",
                    group_name,
                    fps_name,
                )
                failure_message = (
                    f"Unexpected error while retrying load for {group_name} on {fps_name}"
                )
                self._pause_printer_message(failure_message)
                try:
                    if hasattr(oams, "set_led_error"):
                        oams.set_led_error(spool_idx, 1)
                except Exception:
                    logging.exception(
                        "OAMS: Failed to set error LED after retry exception on %s spool %s",
                        getattr(oams, "name", "unknown"),
                        spool_idx,
                    )
                self.stop_monitors()
                return self.reactor.NEVER

            if success:
                logging.info(
                    "OAMS: Retry load succeeded for group %s on %s",
                    group_name,
                    fps_name,
                )
                return self.reactor.NEVER

            logging.error(
                "OAMS: Retry load failed for group %s on %s: %s",
                group_name,
                fps_name,
                message,
            )
            failure_message = (
                message or f"Retry load failed for group {group_name} on {fps_name}"
            )
            self._pause_printer_message(failure_message)
            try:
                if hasattr(oams, "set_led_error"):
                    oams.set_led_error(spool_idx, 1)
            except Exception:
                logging.exception(
                    "OAMS: Failed to set error LED after retry failure on %s spool %s",
                    getattr(oams, "name", "unknown"),
                    spool_idx,
                )
            self.stop_monitors()
            return self.reactor.NEVER

        next_time = self.reactor.monotonic() + retry_delay
        fps_state.monitor_load_next_spool_timer = self.reactor.register_timer(
            partial(_retry_load, self),
            next_time,
        )

        logging.info(
            "OAMS: Scheduled load retry for group %s on %s in %.1f seconds",
            group_name,
            fps_name,
            retry_delay,
        )
        return True

    def _unload_filament_for_fps(self, fps_name: str) -> Tuple[bool, str]:
        """Unload filament from the specified FPS and update state."""
        if fps_name not in self.fpss:
            return False, f"FPS {fps_name} does not exist"

        fps_state = self.current_state.fps_state[fps_name]
        if fps_state.state_name != FPSLoadState.LOADED:
            return False, f"FPS {fps_name} is not currently loaded"

        if fps_state.current_oams is None:
            return False, f"FPS {fps_name} has no OAMS loaded"

        oams = self.oams.get(fps_state.current_oams)
        if oams is None:
            return False, f"OAMS {fps_state.current_oams} not found for FPS {fps_name}"

        if oams.current_spool is None:
            fps_state.state_name = FPSLoadState.UNLOADED
            fps_state.following = False
            fps_state.direction = 0
            fps_state.current_group = None
            fps_state.current_spool_idx = None
            fps_state.since = self.reactor.monotonic()
            self.current_group = None
            fps_state.reset_clog_tracker()
            return True, "Spool already unloaded"

        fps_state.state_name = FPSLoadState.UNLOADING
        fps_state.encoder = oams.encoder_clicks
        fps_state.since = self.reactor.monotonic()
        fps_state.current_oams = oams.name
        fps_state.current_spool_idx = oams.current_spool

        success, message = oams.unload_spool()

        if not success:

            retry_success, retry_message = self._recover_unload_failure(

                fps_name,
                fps_state,
                oams,
                message,
            )
            if retry_success:

                success = True
                message = retry_message
            elif retry_message is not None:

                message = retry_message

        if success:
            fps_state.state_name = FPSLoadState.UNLOADED
            fps_state.following = False
            fps_state.direction = 0
            fps_state.since = self.reactor.monotonic()
            fps_state.current_group = None
            fps_state.current_spool_idx = None
            self.current_group = None
            fps_state.reset_clog_tracker()
            return True, message

        fps_state.state_name = FPSLoadState.LOADED
        return False, message

    def _attempt_unload_retry(
        self,
        fps_name: str,
        fps_state,
        oams,
        last_message: Optional[str],
    ) -> Tuple[bool, Optional[str]]:
        """Attempt an automatic unload retry after recovering from an error."""
        if not self.unload_retry_enabled:
            return False, last_message

        if oams.action_status_code != UNLOAD_RETRY_ERROR_CODE:
            return False, last_message

        extruder_name = getattr(self.fpss[fps_name], "extruder_name", None)
        if not extruder_name:
            logging.error(
                "OAMS: Unable to perform unload retry for %s, extruder not configured",
                fps_name,
            )
            return False, last_message

        logging.warning(
            "OAMS: Unload on %s failed with code %s, attempting automatic recovery",
            fps_name,
            oams.action_status_code,
        )

        gcode = self.printer.lookup_object("gcode")
        if self.unload_retry_push_distance != 0.0:
            command = (
                f"FORCE_MOVE STEPPER={extruder_name} "
                f"DISTANCE={self.unload_retry_push_distance:.3f} "
                f"VELOCITY={self.unload_retry_push_speed:.3f}"
            )
            try:
                logging.info(
                    "OAMS: Jogging extruder %s by %.3fmm before retry",
                    extruder_name,
                    self.unload_retry_push_distance,
                )
                gcode.run_script(command)
            except Exception:
                logging.exception(
                    "OAMS: Failed to jog extruder %s prior to unload retry",
                    extruder_name,
                )

        self._clear_all_errors()

        fps_state.state_name = FPSLoadState.UNLOADING
        fps_state.encoder = oams.encoder_clicks
        fps_state.since = self.reactor.monotonic()
        fps_state.current_oams = oams.name
        fps_state.current_spool_idx = oams.current_spool

        retry_success, retry_message = oams.unload_spool()
        if retry_success:
            logging.info("OAMS: Automatic unload retry succeeded on %s", fps_name)
            return True, retry_message

        logging.error(
            "OAMS: Automatic unload retry failed on %s: %s",
            fps_name,
            retry_message,
        )
        return False, retry_message

    def _load_filament_for_group(self, group_name: str) -> Tuple[bool, str]:
        """Load filament for the provided filament group."""
        if group_name not in self.filament_groups:
            return False, f"Group {group_name} does not exist"

        fps_name = self.group_fps_name(group_name)
        if fps_name is None:
            return False, f"No FPS associated with group {group_name}"

        fps_state = self.current_state.fps_state[fps_name]

        attempted_locations: List[str] = []
        last_failure_message: Optional[str] = None

        for (oam, bay_index) in self.filament_groups[group_name].bays:
            if not oam.is_bay_ready(bay_index):
                continue

            attempted_locations.append(f"{getattr(oam, 'name', 'unknown')} bay {bay_index}")

            fps_state.state_name = FPSLoadState.LOADING
            fps_state.encoder_samples.clear()
            fps_state.encoder = oam.encoder_clicks
            fps_state.since = self.reactor.monotonic()
            fps_state.current_oams = oam.name
            fps_state.current_spool_idx = bay_index

            success, message = oam.load_spool(bay_index)

            if success:
                fps_state.current_group = group_name
                fps_state.current_oams = oam.name
                fps_state.current_spool_idx = bay_index
                fps_state.state_name = FPSLoadState.LOADED
                fps_state.since = self.reactor.monotonic()
                fps_state.following = False
                fps_state.direction = 1
                self.current_group = group_name
                fps_state.encoder_samples.clear()
                fps_state.reset_clog_tracker()
                return True, message

            failure_reason = message or "Unknown load failure"
            logging.warning(
                "OAMS: Failed to load group %s from %s bay %s: %s",
                group_name,
                getattr(oam, "name", "unknown"),
                bay_index,
                failure_reason,
            )

            retry_success, retry_message = self._attempt_unload_retry(
                fps_name,
                fps_state,
                oam,
                message,
            )
            if retry_success:
                logging.info(
                    "OAMS: Cleared stalled load on %s bay %s before trying next bay",
                    getattr(oam, "name", "unknown"),
                    bay_index,
                )
            elif retry_message:
                logging.warning(
                    "OAMS: Automatic unload retry failed for %s bay %s: %s",
                    getattr(oam, "name", "unknown"),
                    bay_index,
                    retry_message,
                )

            fps_state.state_name = FPSLoadState.UNLOADED
            fps_state.current_group = None
            fps_state.current_spool_idx = None
            fps_state.current_oams = None
            fps_state.following = False
            fps_state.direction = 0
            fps_state.encoder = None
            fps_state.encoder_samples.clear()
            fps_state.reset_clog_tracker()
            fps_state.since = self.reactor.monotonic()
            self.current_group = None

            last_failure_message = failure_reason

        if attempted_locations:
            attempts_summary = ", ".join(attempted_locations)
            detail = last_failure_message or "No detailed error provided"
            final_message = (
                f"All ready bays failed to load for group {group_name} after automatic retries "
                f"(attempted: {attempts_summary}). Last error: {detail}"
            )
            logging.error("OAMS: %s", final_message)
            return False, final_message

        return False, f"No spool available for group {group_name}"

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

        success, message = self._unload_filament_for_fps(fps_name)
        if not success or (message and message != "Spool unloaded successfully"):
            gcmd.respond_info(message)
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
        success, message = self._load_filament_for_group(group_name)
        gcmd.respond_info(message)
        return

        
    def _pause_printer_message(self, message):
        logging.info(f"OAMS: {message}")
        gcode = self.printer.lookup_object("gcode")
        message = f"Print has been paused: {message}"
        gcode.run_script(f"M118 {message}")
        gcode.run_script(f"M114 {message}")
        gcode.run_script("PAUSE")

    def _clear_stuck_spool_state(
        self,
        fps_state: 'FPSState',
        restore_following: bool = True,
    ) -> None:
        """Clear any latched stuck-spool indicators for the provided FPS."""

        oams_name = fps_state.stuck_spool_last_oams
        spool_idx = fps_state.stuck_spool_last_spool_idx
        stored_oams = self.oams.get(oams_name) if oams_name is not None else None

        if fps_state.stuck_spool_led_asserted and stored_oams is not None and spool_idx is not None:
            try:
                stored_oams.set_led_error(spool_idx, 0)
            except Exception:
                logging.exception(
                    "OAMS: Failed to clear stuck spool LED on %s spool %s",
                    getattr(stored_oams, "name", oams_name),
                    spool_idx,
                )

        active_oams_name = fps_state.current_oams or oams_name
        active_oams = (
            self.oams.get(active_oams_name)
            if active_oams_name is not None
            else stored_oams
        )

        cleared_ids = set()
        for unit, unit_name in (
            (active_oams, active_oams_name),
            (stored_oams, oams_name),
        ):
            unit_id = id(unit) if unit is not None else None
            if (
                unit is None
                or unit_id in cleared_ids
                or not hasattr(unit, "clear_errors")
            ):
                continue
            try:
                unit.clear_errors()
            except Exception:
                logging.exception(
                    "OAMS: Failed to clear stuck spool error on %s",
                    getattr(unit, "name", unit_name),
                )
            cleared_ids.add(unit_id)

        should_restore = (
            restore_following
            and fps_state.stuck_spool_should_restore_follower
            and fps_state.state_name == FPSLoadState.LOADED
            and active_oams is not None
            and hasattr(active_oams, "set_oams_follower")
        )

        if should_restore:
            direction = fps_state.stuck_spool_restore_direction
            if direction not in (0, 1):
                direction = fps_state.direction if fps_state.direction in (0, 1) else 1
            if direction == 0:
                direction = 1

            try:
                active_oams.set_oams_follower(1, direction)
                fps_state.following = True
                fps_state.direction = direction
                logging.info(
                    "OAMS: Resumed follower on %s spool %s after stuck spool recovery",
                    getattr(active_oams, "name", active_oams_name),
                    spool_idx if spool_idx is not None else fps_state.current_spool_idx,
                )
            except Exception:
                logging.exception(
                    "OAMS: Failed to resume follower after stuck spool on %s",
                    getattr(active_oams, "name", active_oams_name),
                )

        fps_state.reset_stuck_spool_state()

    def _handle_resume_command(self, *args, **kwargs) -> None:
        """React to resume commands so stuck-spool state clears immediately."""

        if not self.ready:
            return

        self.reactor.register_callback(self._recover_after_resume)

    def _recover_after_resume(self, eventtime):
        for fps_state in self.current_state.fps_state.values():
            if (
                fps_state.stuck_spool_active
                or fps_state.stuck_spool_led_asserted
                or fps_state.stuck_spool_should_restore_follower
            ):
                self._clear_stuck_spool_state(fps_state)
        return self.reactor.NEVER

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
                    spool_idx = fps_state.current_spool_idx
                    if oams is not None and spool_idx is not None:
                        try:
                            oams.set_led_error(spool_idx, 0)
                        except Exception:
                            logging.exception(
                                "OAMS: Failed to clear error LED while aborting load retry on %s spool %s",
                                getattr(oams, "name", fps_state.current_oams),
                                spool_idx,
                            )
                    if not self._abort_stalled_load_and_retry(fps_name, fps_state, oams):
                        if oams is not None and spool_idx is not None:
                            try:
                                oams.set_led_error(spool_idx, 1)
                            except Exception:
                                logging.exception(
                                    "OAMS: Failed to assert error LED after stalled load on %s spool %s",
                                    getattr(oams, "name", fps_state.current_oams),
                                    spool_idx,
                                )
                        self._pause_printer_message("Printer paused because the loading speed of the moving filament was too low")
                        self.stop_monitors()
                        return self.printer.get_reactor().NEVER
                    return eventtime + MONITOR_ENCODER_PERIOD
            return eventtime + MONITOR_ENCODER_PERIOD
        return partial(_monitor_load_speed, self)


    def _monitor_stuck_spool_for_fps(self, fps_name: str):
        idle_timeout = self.printer.lookup_object("idle_timeout")
        pause_resume = self.pause_resume
        print_stats = self.print_stats

        def _monitor_stuck_spool(self, eventtime):
            fps_state = self.current_state.fps_state.get(fps_name)
            fps = self.fpss.get(fps_name)
            if fps_state is None or fps is None:
                return eventtime + self.clog_monitor_period

            try:
                is_paused = bool(pause_resume.get_status(eventtime).get("is_paused"))
            except Exception:
                logging.exception("OAMS: Failed to query pause state for stuck spool monitor")
                is_paused = False

            if fps_state.stuck_spool_active:
                spool_changed = (
                    fps_state.current_oams != fps_state.stuck_spool_last_oams
                    or fps_state.current_spool_idx != fps_state.stuck_spool_last_spool_idx
                    or fps_state.current_oams is None
                    or fps_state.current_spool_idx is None
                )
                if spool_changed:
                    self._clear_stuck_spool_state(fps_state, restore_following=False)
                    return eventtime + self.clog_monitor_period
                if is_paused:
                    return eventtime + self.clog_monitor_period
                self._clear_stuck_spool_state(fps_state)

            status = idle_timeout.get_status(eventtime)
            is_printing = status.get("state") == "Printing"

            all_axes_homed = True
            if self.toolhead is not None:
                try:
                    homed_axes = self.toolhead.get_status(eventtime).get(
                        "homed_axes", ""
                    )
                except Exception:
                    logging.exception(
                        "OAMS: Failed to query homed axes for stuck spool monitor"
                    )
                    homed_axes = ""

                if isinstance(homed_axes, (list, tuple, set)):
                    axes = "".join(homed_axes)
                else:
                    axes = str(homed_axes)

                all_axes_homed = all(axis in axes for axis in "xyz")

            if not all_axes_homed:
                fps_state.stuck_spool_start_time = None
                return eventtime + self.clog_monitor_period

            if is_printing and print_stats is not None:
                try:
                    stats_state = print_stats.get_status(eventtime).get("state")
                except Exception:
                    logging.exception(
                        "OAMS: Failed to query print stats for stuck spool monitor"
                    )
                    stats_state = None
                is_printing = stats_state == "printing"

            if not is_printing or fps_state.state_name != FPSLoadState.LOADED:
                fps_state.stuck_spool_start_time = None
                return eventtime + self.clog_monitor_period

            if fps_state.current_oams is None or fps_state.current_spool_idx is None:
                fps_state.stuck_spool_start_time = None
                return eventtime + self.clog_monitor_period

            oams = self.oams.get(fps_state.current_oams)
            if oams is None:
                fps_state.stuck_spool_start_time = None
                return eventtime + self.clog_monitor_period

            pressure = float(
                getattr(oams, "fps_value", getattr(fps, "fps_value", 0.0)) or 0.0
            )
            now = self.reactor.monotonic()

            if pressure <= STUCK_SPOOL_PRESSURE_TRIGGER:
                if fps_state.stuck_spool_start_time is None:
                    fps_state.stuck_spool_start_time = now
                elif now - (fps_state.stuck_spool_start_time or now) >= self.stuck_spool_dwell_time:
                    fps_state.stuck_spool_active = True
                    fps_state.stuck_spool_last_oams = fps_state.current_oams
                    fps_state.stuck_spool_last_spool_idx = fps_state.current_spool_idx
                    fps_state.stuck_spool_start_time = None
                    fps_state.stuck_spool_should_restore_follower = True
                    direction = (
                        fps_state.direction
                        if fps_state.direction in (0, 1)
                        else 1
                    )
                    if direction == 0:
                        direction = 1
                    fps_state.stuck_spool_restore_direction = direction
                    if hasattr(oams, "set_oams_follower"):
                        try:
                            oams.set_oams_follower(0, direction)
                            fps_state.following = False
                            fps_state.direction = direction
                            logging.info(
                                "OAMS: Disabled follower on %s spool %s due to stuck spool detection",
                                getattr(oams, "name", fps_state.current_oams),
                                fps_state.current_spool_idx,
                            )
                        except Exception:
                            logging.exception(
                                "OAMS: Failed to stop follower after stuck spool on %s spool %s",
                                getattr(oams, "name", fps_state.current_oams),
                                fps_state.current_spool_idx,
                            )
                    if fps_state.current_spool_idx is not None:
                        try:
                            oams.set_led_error(fps_state.current_spool_idx, 1)
                            fps_state.stuck_spool_led_asserted = True
                        except Exception:
                            logging.exception(
                                "OAMS: Failed to set stuck spool LED on %s spool %s",
                                getattr(oams, "name", fps_state.current_oams),
                                fps_state.current_spool_idx,
                            )
                    group = fps_state.current_group or fps_name
                    logging.error(
                        "OAMS: Stuck spool detected on %s (spool %s) pressure %.2f",
                        group,
                        fps_state.current_spool_idx,
                        pressure,
                    )
                    self._pause_printer_message(
                        "Spool appears stuck on %s spool %s (pressure %.2f)"
                        % (
                            group,
                            fps_state.current_spool_idx,
                            pressure,
                        )
                    )
                    return eventtime + self.clog_monitor_period
            else:
                fps_state.stuck_spool_start_time = None

            return eventtime + self.clog_monitor_period

        return partial(_monitor_stuck_spool, self)


    def _monitor_clog_for_fps(self, fps_name: str):
        idle_timeout = self.printer.lookup_object("idle_timeout")
        pause_resume = self.pause_resume
        print_stats = self.print_stats
        toolhead = self.toolhead

        def _monitor_clog(self, eventtime):
            if not self.clog_detection_enabled:
                return self.printer.get_reactor().NEVER

            fps_state = self.current_state.fps_state.get(fps_name)
            fps = self.fpss.get(fps_name)
            if fps_state is None or fps is None:
                return eventtime + self.clog_monitor_period

            is_paused = False
            if pause_resume is not None:
                try:
                    is_paused = bool(pause_resume.get_status(eventtime).get("is_paused"))
                except Exception:
                    logging.exception("OAMS: Failed to query pause state for clog monitor")
                    is_paused = False

            if is_paused or fps_state.state_name != FPSLoadState.LOADED:
                fps_state.reset_clog_tracker()
                return eventtime + self.clog_monitor_period

            status = idle_timeout.get_status(eventtime)
            is_printing = status.get("state") == "Printing"

            if is_printing and print_stats is not None:
                try:
                    stats_state = print_stats.get_status(eventtime).get("state")
                except Exception:
                    logging.exception("OAMS: Failed to query print stats for clog monitor")
                    stats_state = None
                is_printing = stats_state == "printing"

            all_axes_homed = True
            if toolhead is not None:
                try:
                    homed_axes = toolhead.get_status(eventtime).get("homed_axes", "")
                except Exception:
                    logging.exception("OAMS: Failed to query homed axes for clog monitor")
                    homed_axes = ""

                if isinstance(homed_axes, (list, tuple, set)):
                    axes = "".join(homed_axes)
                else:
                    axes = str(homed_axes)

                all_axes_homed = all(axis in axes for axis in "xyz")

            if (
                not is_printing
                or not all_axes_homed
                or fps_state.current_oams is None
                or fps_state.current_spool_idx is None
                or fps_state.stuck_spool_active
            ):
                fps_state.reset_clog_tracker()
                return eventtime + self.clog_monitor_period

            oams = self.oams.get(fps_state.current_oams)
            if oams is None:
                fps_state.reset_clog_tracker()
                return eventtime + self.clog_monitor_period

            extruder = getattr(fps, "extruder", None)
            extruder_position = getattr(extruder, "last_position", None) if extruder else None
            if extruder_position is None:
                fps_state.reset_clog_tracker()
                return eventtime + self.clog_monitor_period

            encoder_position_raw = getattr(oams, "encoder_clicks", None)
            if encoder_position_raw is None:
                fps_state.reset_clog_tracker()
                return eventtime + self.clog_monitor_period

            encoder_position = float(encoder_position_raw)
            now = self.reactor.monotonic()
            pressure = float(
                getattr(oams, "fps_value", getattr(fps, "fps_value", 0.0)) or 0.0
            )

            if fps_state.clog_extruder_start is None:
                fps_state.prime_clog_tracker(
                    float(extruder_position),
                    encoder_position,
                    now,
                    pressure,
                )
                return eventtime + self.clog_monitor_period

            last_extruder = fps_state.clog_last_extruder
            if (
                last_extruder is not None
                and float(extruder_position)
                < last_extruder - self.clog_retraction_tolerance_mm
            ):
                fps_state.reset_clog_tracker()
                fps_state.prime_clog_tracker(
                    float(extruder_position),
                    encoder_position,
                    now,
                    pressure,
                )
                return eventtime + self.clog_monitor_period

            extruder_delta = float(extruder_position) - float(fps_state.clog_extruder_start)
            if extruder_delta < 0.0:
                fps_state.reset_clog_tracker()
                fps_state.prime_clog_tracker(
                    float(extruder_position),
                    encoder_position,
                    now,
                    pressure,
                )
                return eventtime + self.clog_monitor_period

            encoder_delta = encoder_position - float(fps_state.clog_encoder_start)
            if encoder_delta < -self.clog_encoder_delta_limit:
                fps_state.reset_clog_tracker()
                fps_state.prime_clog_tracker(
                    float(extruder_position),
                    encoder_position,
                    now,
                    pressure,
                )
                return eventtime + self.clog_monitor_period

            fps_state.clog_extruder_delta = extruder_delta
            fps_state.clog_encoder_delta = encoder_delta
            fps_state.clog_max_pressure = max(fps_state.clog_max_pressure, pressure)
            fps_state.clog_min_pressure = min(fps_state.clog_min_pressure, pressure)
            fps_state.clog_last_extruder = float(extruder_position)
            fps_state.clog_last_encoder = encoder_position

            if extruder_delta < self.clog_extruder_window_mm:
                return eventtime + self.clog_monitor_period

            start_time = fps_state.clog_start_time or now
            if now - start_time < self.clog_dwell_time:
                return eventtime + self.clog_monitor_period

            target_pressure = getattr(oams, "fps_target", None)
            if target_pressure is None:
                target_pressure = getattr(fps, "fps_target", None)
            if target_pressure is None:
                target_pressure = getattr(fps, "_set_point", 0.5)
            clamped_target = max(0.0, min(1.0, float(target_pressure)))
            pressure_window = max(0.0, self.clog_pressure_offset)
            clamped_min_pressure = max(0.0, min(1.0, float(fps_state.clog_min_pressure)))
            clamped_max_pressure = max(0.0, min(1.0, float(fps_state.clog_max_pressure)))

            max_deviation = max(
                abs(clamped_max_pressure - clamped_target),
                abs(clamped_min_pressure - clamped_target),
            )

            if pressure_window and max_deviation > pressure_window:
                return eventtime + self.clog_monitor_period
            if not pressure_window and max_deviation > 0.0:
                return eventtime + self.clog_monitor_period

            if abs(encoder_delta) > self.clog_encoder_delta_limit:
                return eventtime + self.clog_monitor_period

            logging.error(
                "OAMS: Clog suspected on %s after %.1fmm extruder advance (encoder %.1f, fps window %.2f-%.2f around %.2f)",
                fps_name,
                extruder_delta,
                encoder_delta,
                clamped_min_pressure,
                clamped_max_pressure,
                clamped_target,
            )

            if fps_state.current_spool_idx is not None:
                try:
                    oams.set_led_error(fps_state.current_spool_idx, 1)
                except Exception:
                    logging.exception(
                        "OAMS: Failed to set error LED for clog on %s spool %s",
                        getattr(oams, "name", fps_state.current_oams),
                        fps_state.current_spool_idx,
                    )

            self._pause_printer_message(
                (
                    "Clog suspected on %s: extruder advanced %.1fmm while encoder moved %.1f "
                    "counts with FPS %.2f-%.2f around %.2f"
                )
                % (
                    fps_name,
                    extruder_delta,
                    encoder_delta,
                    clamped_min_pressure,
                    clamped_max_pressure,
                    clamped_target,
                )
            )
            fps_state.reset_clog_tracker()
            self.stop_monitors()
            return self.printer.get_reactor().NEVER

        return partial(_monitor_clog, self)


    def start_monitors(self):
        self.monitor_timers = []
        self.runout_monitors = {}
        reactor = self.printer.get_reactor()
        for (fps_name, fps_state) in self.current_state.fps_state.items():
            fps_state.reset_clog_tracker()
            self.monitor_timers.append(reactor.register_timer(self._monitor_unload_speed_for_fps(fps_name), reactor.NOW))
            self.monitor_timers.append(reactor.register_timer(self._monitor_load_speed_for_fps(fps_name), reactor.NOW))
            self.monitor_timers.append(
                reactor.register_timer(
                    self._monitor_stuck_spool_for_fps(fps_name),
                    reactor.NOW,
                )
            )
            if self.clog_detection_enabled:
                self.monitor_timers.append(
                    reactor.register_timer(
                        self._monitor_clog_for_fps(fps_name),
                        reactor.NOW,
                    )
                )

            def _reload_callback(fps_name=fps_name, fps_state=fps_state):
                monitor = self.runout_monitors.get(fps_name)
                source_group = fps_state.current_group
                target_group, target_lane, delegate_to_afc, source_lane = self._get_infinite_runout_target_group(
                    fps_name,
                    fps_state,
                )
                source_group = fps_state.current_group

                if delegate_to_afc:
                    delegated = self._delegate_runout_to_afc(
                        fps_name,
                        fps_state,
                        source_lane,
                        target_lane,
                    )
                    if delegated:
                        fps_state.reset_runout_positions()
                        if monitor:
                            monitor.reset()
                            monitor.start()
                        return

                    logging.error(
                        "OAMS: Failed to delegate infinite runout for %s on %s via AFC",
                        fps_name,
                        source_group or "<unknown>",
                    )
                    fps_state.reset_runout_positions()
                    self._pause_printer_message(
                        f"Unable to delegate infinite runout for {source_group or fps_name}"
                    )
                    if monitor:
                        monitor.paused()
                    return

                group_to_load = target_group or source_group

                if target_group:
                    logging.info(
                        "OAMS: Infinite runout triggered for %s on %s -> %s",
                        fps_name,
                        source_group,
                        target_group,
                    )
                    unload_success, unload_message = self._unload_filament_for_fps(fps_name)
                    if not unload_success:
                        logging.error(
                            "OAMS: Failed to unload filament during infinite runout on %s: %s",
                            fps_name,
                            unload_message,
                        )
                        failure_message = unload_message or f"Failed to unload current spool on {fps_name}"
                        self._pause_printer_message(failure_message)
                        if monitor:
                            monitor.paused()
                        return

                if group_to_load is None:
                    logging.error("OAMS: No filament group available to reload on %s", fps_name)
                    self._pause_printer_message(f"No filament group available to reload on {fps_name}")
                    if monitor:
                        monitor.paused()
                    return

                load_success, load_message = self._load_filament_for_group(group_to_load)
                if load_success:
                    logging.info(
                        "OAMS: Successfully loaded group %s on %s%s",
                        group_to_load,
                        fps_name,
                        " after infinite runout" if target_group else "",
                    )
                    if target_group:
                        if target_lane:
                            try:
                                gcode = self.printer.lookup_object("gcode")
                                gcode.run_script(f"SET_LANE_LOADED LANE={target_lane}")
                                logging.debug(
                                    "OAMS: Marked lane %s as loaded after infinite runout on %s",
                                    target_lane,
                                    fps_name,
                                )
                            except Exception:
                                logging.exception(
                                    "OAMS: Failed to mark lane %s as loaded after infinite runout on %s",
                                    target_lane,
                                    fps_name,
                                )
                        else:
                            logging.warning(
                                "OAMS: No runout lane recorded for %s on %s when marking lane loaded",
                                target_group,
                                fps_name,
                            )
                    fps_state.reset_runout_positions()
                    if monitor:
                        monitor.reset()
                        monitor.start()
                    return

                logging.error(
                    "OAMS: Failed to load group %s on %s: %s",
                    group_to_load,
                    fps_name,
                    load_message,
                )
                failure_message = load_message or f"No spool available for group {group_to_load}"
                self._pause_printer_message(failure_message)
                if monitor:
                    monitor.paused()
                return

            fps_reload_margin = getattr(
                self.fpss[fps_name],
                "reload_before_toolhead_distance",
                None,
            )
            if fps_reload_margin is None:
                fps_reload_margin = self.reload_before_toolhead_distance
            else:
                logging.debug(
                    "OAMS: Using FPS-specific reload margin %.2f mm for %s",
                    fps_reload_margin,
                    fps_name,
                )

            monitor = OAMSRunoutMonitor(
                self.printer,
                fps_name,
                self.fpss[fps_name],
                fps_state,
                self.oams,
                _reload_callback,
                reload_before_toolhead_distance=fps_reload_margin,
            )
            self.runout_monitors[fps_name] = monitor
            monitor.start()

        logging.info("OAMS: All monitors started")

    def stop_monitors(self):
        for timer in self.monitor_timers:
            self.printer.get_reactor().unregister_timer(timer)
        self.monitor_timers = []
        for fps_state in self.current_state.fps_state.values():
            timer = getattr(fps_state, "monitor_load_next_spool_timer", None)
            if timer is not None:
                try:
                    self.printer.get_reactor().unregister_timer(timer)
                except Exception:
                    logging.exception(
                        "OAMS: Failed to cancel pending load retry timer while stopping monitors",
                    )
                finally:
                    fps_state.monitor_load_next_spool_timer = None
        for monitor in self.runout_monitors.values():
            monitor.reset()
        self.runout_monitors = {}


def load_config(config):
    return OAMSManager(config)