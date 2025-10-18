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

# Configuration constants (kept as module-level constants to mirror oams.py style)
PAUSE_DISTANCE = 60  # mm to pause before coasting follower
ENCODER_SAMPLES = 2  # Number of encoder samples to collect
MIN_ENCODER_DIFF = 1  # Minimum encoder difference to consider movement
FILAMENT_PATH_LENGTH_FACTOR = 1.14  # Factor for calculating filament path traversal
MONITOR_ENCODER_LOADING_SPEED_AFTER = 2.0  # seconds
MONITOR_ENCODER_PERIOD = 2.0  # seconds
MONITOR_ENCODER_UNLOADING_SPEED_AFTER = 2.0  # seconds
AFC_DELEGATION_TIMEOUT = 30.0  # seconds to suppress duplicate AFC runout triggers

STUCK_SPOOL_PRESSURE_THRESHOLD = 0.08  # Pressure indicating the spool is no longer feeding
STUCK_SPOOL_DWELL = 3.5  # Seconds the pressure must remain below the threshold before pausing
STUCK_SPOOL_LOAD_GRACE = 8.0  # Grace period after a swap/load before stuck detection arms

CLOG_PRESSURE_TARGET = 0.50
CLOG_SENSITIVITY_LEVELS = {
    "low": {
        "extrusion_window": 48.0,
        "encoder_slack": 15,
        "pressure_band": 0.08,
        "dwell": 12.0,
    },
    "medium": {
        "extrusion_window": 24.0,
        "encoder_slack": 8,
        "pressure_band": 0.06,
        "dwell": 8.0,
    },
    "high": {
        "extrusion_window": 12.0,
        "encoder_slack": 4,
        "pressure_band": 0.04,
        "dwell": 6.0,
    },
}
CLOG_SENSITIVITY_DEFAULT = "medium"

POST_LOAD_PRESSURE_THRESHOLD = 0.52  # FPS value indicating a possible clog
POST_LOAD_PRESSURE_DWELL = 5.0  # Seconds pressure must remain above threshold
POST_LOAD_PRESSURE_CHECK_PERIOD = 1.0


class OAMSRunoutState:
    """Enum for runout monitor states."""
    STOPPED = "STOPPED"          # Monitor is disabled
    MONITORING = "MONITORING"    # Actively watching for runout
    DETECTED = "DETECTED"        # Runout detected, pausing before coast
    COASTING = "COASTING"        # Follower coasting, preparing next spool
    RELOADING = "RELOADING"      # Loading next spool in sequence
    PAUSED = "PAUSED"            # Monitor paused due to error/manual intervention


class FPSLoadState:
    """Enum for FPS loading states."""
    UNLOADED = "UNLOADED"    # No filament loaded
    LOADED = "LOADED"        # Filament loaded and ready
    LOADING = "LOADING"      # Currently loading filament
    UNLOADING = "UNLOADING"  # Currently unloading filament


class OAMSRunoutMonitor:
    """
    Monitors filament runout for a specific FPS and handles automatic reload.

    This class mirrors the approach used in oams.py for keeping a compact,
    hardware-aware helper that interacts with OAMS objects.
    """

    def __init__(
        self,
        printer,
        fps_name: str,
        fps,
        fps_state,
        oams: Dict[str, Any],
        reload_callback: Callable,
        reload_before_toolhead_distance: float = 0.0,
    ):
        self.printer = printer
        self.reactor = printer.get_reactor()
        self.fps_name = fps_name
        self.fps = fps
        self.fps_state = fps_state
        self.oams = oams
        self.reload_callback = reload_callback
        self.reload_before_toolhead_distance = reload_before_toolhead_distance

        # internal state for runout sequencing
        self.state = OAMSRunoutState.STOPPED
        self.runout_position: Optional[float] = None
        self.bldc_clear_position: Optional[float] = None
        self.runout_after_position: Optional[float] = None

        # register timer
        self._timer_callback = self._monitor_runout
        self.timer = self.reactor.register_timer(self._timer_callback, self.reactor.NOW)

    def _monitor_runout(self, eventtime):
        # Guard when monitor is not active
        if self.state in (OAMSRunoutState.STOPPED, OAMSRunoutState.PAUSED, OAMSRunoutState.RELOADING):
            return eventtime + MONITOR_ENCODER_PERIOD

        try:
            idle_timeout = self.printer.lookup_object("idle_timeout")
            is_printing = idle_timeout.get_status(eventtime)["state"] == "Printing"
        except Exception:
            is_printing = False

        # Helper to safely read hub HES values
        def _hub_spool_empty():
            try:
                if not getattr(self.fps_state, "current_oams", None):
                    return False
                oams_obj = self.oams.get(self.fps_state.current_oams)
                if oams_obj is None:
                    return False
                hes_values = oams_obj.hub_hes_value
                return not bool(hes_values[self.fps_state.current_spool_idx])
            except Exception:
                logging.exception("OAMS: Failed to read HES values for runout detection on %s", self.fps_name)
                return False

        # State machine
        if self.state == OAMSRunoutState.MONITORING:
            # AFC delegation suppression
            if getattr(self.fps_state, "afc_delegation_active", False):
                now = self.reactor.monotonic()
                if now < getattr(self.fps_state, "afc_delegation_until", 0.0):
                    return eventtime + MONITOR_ENCODER_PERIOD
                self.fps_state.afc_delegation_active = False
                self.fps_state.afc_delegation_until = 0.0

            if (
                is_printing
                and getattr(self.fps_state, "state_name", None) == FPSLoadState.LOADED
                and self.fps_state.current_group is not None
                and self.fps_state.current_spool_idx is not None
                and _hub_spool_empty()
            ):
                # Detected runout
                self.state = OAMSRunoutState.DETECTED
                logging.info(
                    "OAMS: Runout detected on FPS %s, pausing for %d mm before coasting the follower.",
                    self.fps_name,
                    PAUSE_DISTANCE,
                )
                self.runout_position = self.fps.extruder.last_position

        elif self.state == OAMSRunoutState.DETECTED:
            if self.runout_position is None:
                self.runout_position = self.fps.extruder.last_position
            traveled_distance = self.fps.extruder.last_position - self.runout_position
            if traveled_distance >= PAUSE_DISTANCE:
                logging.info("OAMS: Pause complete, coasting the follower.")
                try:
                    # stop follower on the relevant OAMS
                    self.oams[self.fps_state.current_oams].set_oams_follower(0, 1)
                except Exception:
                    logging.exception("OAMS: Failed to stop follower while coasting on %s", self.fps_name)
                finally:
                    # ensure internal state is consistent
                    self.fps_state.following = False
                self.bldc_clear_position = self.fps.extruder.last_position
                self.runout_after_position = 0.0
                self.state = OAMSRunoutState.COASTING

        elif self.state == OAMSRunoutState.COASTING:
            if self.bldc_clear_position is None:
                self.bldc_clear_position = self.fps.extruder.last_position
            traveled_distance_after_bldc_clear = max(self.fps.extruder.last_position - self.bldc_clear_position, 0.0)
            self.runout_after_position = traveled_distance_after_bldc_clear
            try:
                path_length = getattr(self.oams[self.fps_state.current_oams], "filament_path_length", 0.0)
            except Exception:
                logging.exception("OAMS: Failed to read filament path length while coasting on %s", self.fps_name)
                return eventtime + MONITOR_ENCODER_PERIOD
            effective_path_length = (path_length / FILAMENT_PATH_LENGTH_FACTOR) if path_length else 0.0
            consumed_with_margin = self.runout_after_position + PAUSE_DISTANCE + self.reload_before_toolhead_distance

            if consumed_with_margin >= effective_path_length:
                logging.info(
                    "OAMS: Loading next spool (%.2f mm consumed + margin %.2f mm >= effective path %.2f mm).",
                    self.runout_after_position + PAUSE_DISTANCE,
                    self.reload_before_toolhead_distance,
                    effective_path_length,
                )
                self.state = OAMSRunoutState.RELOADING
                # invoke reload callback (OAMSManager will handle delegation/actual load)
                try:
                    self.reload_callback()
                except Exception:
                    logging.exception("OAMS: Exception while invoking reload callback for %s", self.fps_name)
        else:
            logging.error("OAMS: Invalid runout monitor state %s for %s", self.state, self.fps_name)

        return eventtime + MONITOR_ENCODER_PERIOD

    def start(self) -> None:
        if self.timer is None:
            self.timer = self.reactor.register_timer(self._timer_callback, self.reactor.NOW)
        self.state = OAMSRunoutState.MONITORING

    def stop(self) -> None:
        self.state = OAMSRunoutState.STOPPED

    def reloading(self) -> None:
        self.state = OAMSRunoutState.RELOADING
        self.runout_position = None
        self.runout_after_position = None

    def paused(self) -> None:
        self.state = OAMSRunoutState.PAUSED

    def reset(self) -> None:
        self.state = OAMSRunoutState.STOPPED
        self.runout_position = None
        self.runout_after_position = None
        if self.timer is not None:
            try:
                self.reactor.unregister_timer(self.timer)
            except Exception:
                logging.exception("OAMS: Failed to unregister runout monitor timer for %s", self.fps_name)
            self.timer = None


class OAMSState:
    """
    Global state container for all FPS units in the system.
    """

    def __init__(self):
        self.fps_state: Dict[str, 'FPSState'] = {}

    def add_fps_state(self, fps_name: str) -> None:
        self.fps_state[fps_name] = FPSState()


class FPSState:
    """
    Tracks the state of a single FPS (Filament Pressure Sensor).

    This class is intentionally compact and mirrors the telemetry/state
    expectations set by the OAMS object.
    """

    def __init__(
        self,
        state_name: str = FPSLoadState.UNLOADED,
        current_group: Optional[str] = None,
        current_oams: Optional[str] = None,
        current_spool_idx: Optional[int] = None,
    ):
        # Primary state
        self.state_name = state_name
        self.current_group = current_group
        self.current_oams = current_oams
        self.current_spool_idx = current_spool_idx

        # Runout tracking
        self.runout_position: Optional[float] = None
        self.runout_after_position: Optional[float] = None

        # Timers
        self.monitor_spool_timer = None
        self.monitor_pause_timer = None
        self.monitor_load_next_spool_timer = None

        # Encoder / follower monitoring
        self.encoder_samples = deque(maxlen=ENCODER_SAMPLES)
        self.following: bool = False
        self.direction: int = 0
        self.since: Optional[float] = None

        # AFC delegation
        self.afc_delegation_active: bool = False
        self.afc_delegation_until: float = 0.0

        # Stuck spool detection
        self.stuck_spool_start_time: Optional[float] = None
        self.stuck_spool_active: bool = False
        self.stuck_spool_restore_follower: bool = False
        self.stuck_spool_restore_direction: int = 1

        # Clog detection
        self.clog_active: bool = False
        self.clog_start_extruder: Optional[float] = None
        self.clog_start_encoder: Optional[int] = None
        self.clog_start_time: Optional[float] = None
        self.clog_min_pressure: Optional[float] = None
        self.clog_max_pressure: Optional[float] = None
        self.clog_last_extruder: Optional[float] = None

        # Post-load pressure validation
        self.post_load_pressure_timer = None
        self.post_load_pressure_start: Optional[float] = None

    def reset_runout_positions(self) -> None:
        self.runout_position = None
        self.runout_after_position = None

    def reset_stuck_spool_state(self, preserve_restore: bool = False) -> None:
        self.stuck_spool_start_time = None
        self.stuck_spool_active = False
        if not preserve_restore:
            self.stuck_spool_restore_follower = False
            self.stuck_spool_restore_direction = 1

    def reset_clog_tracker(self) -> None:
        self.clog_active = False
        self.clog_start_extruder = None
        self.clog_start_encoder = None
        self.clog_start_time = None
        self.clog_min_pressure = None
        self.clog_max_pressure = None
        self.clog_last_extruder = None

    def prime_clog_tracker(self, extruder_pos: float, encoder_clicks: int, pressure: float, timestamp: float) -> None:
        self.clog_start_extruder = extruder_pos
        self.clog_last_extruder = extruder_pos
        self.clog_start_encoder = encoder_clicks
        self.clog_start_time = timestamp
        self.clog_min_pressure = pressure
        self.clog_max_pressure = pressure

    def __repr__(self) -> str:
        return f"FPSState(state_name={self.state_name}, current_group={self.current_group}, current_oams={self.current_oams}, current_spool_idx={self.current_spool_idx})"

    def __str__(self) -> str:
        return f"State: {self.state_name}, Group: {self.current_group}, OAMS: {self.current_oams}, Spool: {self.current_spool_idx}"


class OAMSManager:
    """
    Main coordinator for OpenAMS system with multiple FPS units.

    This refactored manager aims to follow the structural conventions used in oams.py:
    - compact hardware helper classes register with MCU and reactor
    - clear separation of state tracking (OAMSState/FPSState)
    - small helper methods for I/O and error handling
    """

    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()

        self.name = config.get_name().split()[-1]

        # Hardware collections
        self.filament_groups: Dict[str, Any] = {}
        self.oams: Dict[str, Any] = {}
        self.fpss: Dict[str, Any] = {}

        # State
        self.current_state = OAMSState()
        self.current_group: Optional[str] = None
        self.afc = None
        self._afc_logged = False

        # Monitors
        self.monitor_timers: List[Any] = []
        self.runout_monitors: Dict[str, OAMSRunoutMonitor] = {}
        self.ready: bool = False

        # Config options
        self.reload_before_toolhead_distance: float = config.getfloat("reload_before_toolhead_distance", 0.0)

        sensitivity = config.get("clog_sensitivity", CLOG_SENSITIVITY_DEFAULT).lower()
        if sensitivity not in CLOG_SENSITIVITY_LEVELS:
            logging.warning("OAMS: Unknown clog_sensitivity '%s', falling back to %s", sensitivity, CLOG_SENSITIVITY_DEFAULT)
            sensitivity = CLOG_SENSITIVITY_DEFAULT
        self.clog_sensitivity = sensitivity
        self.clog_settings = CLOG_SENSITIVITY_LEVELS[self.clog_sensitivity]

        # Cached mappings for AFC integration and lanes
        self.group_to_fps: Dict[str, str] = {}
        self._canonical_lane_by_group: Dict[str, str] = {}
        self._canonical_group_by_lane: Dict[str, str] = {}
        self._lane_unit_map: Dict[str, str] = {}
        self._lane_by_location: Dict[Tuple[str, int], str] = {}

        # Initialize known hardware & groups
        self._initialize_oams()
        self._initialize_filament_groups()

        # Event handlers & registration
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.printer.register_event_handler("idle_timeout:printing", self._handle_printing_resumed)
        self.printer.register_event_handler("pause:resume", self._handle_printing_resumed)

        self.printer.add_object("oams_manager", self)
        self.register_commands()

    # --- Discovery / initialization ------------------------------------------------

    def _initialize_oams(self) -> None:
        """Discover and register all OAMS hardware units."""
        for name, oam in self.printer.lookup_objects(module="oams"):
            self.oams[name] = oam

    def _initialize_filament_groups(self) -> None:
        """Discover and register all filament group configurations."""
        for name, group in self.printer.lookup_objects(module="filament_group"):
            group_name = name.split()[-1]
            logging.info("OAMS: Adding group %s", group_name)
            self.filament_groups[group_name] = group

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
        if not group or not isinstance(group, str):
            return None
        group = group.strip()
        if not group:
            return None
        if " " in group:
            group = group.split()[-1]
        return group

    # --- Compatibility helpers for bay API ---------------------------------------
    # These provide robust access to bay "ready" / "loaded" state whether the
    # target object exposes is_bay_ready/is_bay_loaded helpers or only low-level
    # arrays (f1s_hes_value, hub_hes_value).
    def _is_bay_ready(self, oam: Any, bay_index: int) -> bool:
        try:
            if hasattr(oam, "is_bay_ready"):
                return bool(oam.is_bay_ready(bay_index))
            f1s = getattr(oam, "f1s_hes_value", None)
            if f1s is not None and len(f1s) > bay_index:
                return bool(f1s[bay_index])
            alt = getattr(oam, "f1s_value", None)
            if alt is not None and len(alt) > bay_index:
                return bool(alt[bay_index])
        except Exception:
            logging.exception("OAMS Manager: error querying is_bay_ready for %s-%d", getattr(oam, "name", "<unknown>"), bay_index)
        return False

    def _is_bay_loaded(self, oam: Any, bay_index: int) -> bool:
        try:
            if hasattr(oam, "is_bay_loaded"):
                return bool(oam.is_bay_loaded(bay_index))
            hub = getattr(oam, "hub_hes_value", None)
            if hub is not None and len(hub) > bay_index:
                return bool(hub[bay_index])
            alt = getattr(oam, "hub_value", None)
            if alt is not None and len(alt) > bay_index:
                return bool(alt[bay_index])
        except Exception:
            logging.exception("OAMS Manager: error querying is_bay_loaded for %s-%d", getattr(oam, "name", "<unknown>"), bay_index)
        return False

    # --- AFC integration / lane mapping -------------------------------------------

    def _rebuild_lane_location_index(self) -> None:
        mapping: Dict[Tuple[str, int], str] = {}
        for group_name, lane_name in self._canonical_lane_by_group.items():
            group = self.filament_groups.get(group_name)
            if not group:
                continue
            for oam, bay_index in group.bays:
                mapping[(oam.name, bay_index)] = lane_name
        self._lane_by_location = mapping

    def _ensure_afc_lane_cache(self, afc) -> None:
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

    def _resolve_lane_for_state(self, fps_state: 'FPSState', group_name: Optional[str], afc) -> Tuple[Optional[str], Optional[str]]:
        """Determine the canonical AFC lane and group for the provided FPS state."""
        normalized_group = self._normalize_group_name(group_name)
        lane_name: Optional[str] = None

        # Prefer physical location recorded on FPS state
        if fps_state.current_oams and fps_state.current_spool_idx is not None:
            lane_name = self._lane_by_location.get((fps_state.current_oams, fps_state.current_spool_idx))
            if lane_name:
                lane_group = self._canonical_group_by_lane.get(lane_name)
                if lane_group:
                    normalized_group = lane_group

        # Fall back to canonical mapping
        if lane_name is None and normalized_group:
            lane_name = self._canonical_lane_by_group.get(normalized_group)

        lanes = getattr(afc, "lanes", {})

        # Inspect lanes directly as last resort
        if lane_name is None and normalized_group:
            lane_name = next(
                (name for name, lane in lanes.items() if self._normalize_group_name(getattr(lane, "_map", None)) == normalized_group),
                None,
            )

        canonical_group = normalized_group
        if lane_name:
            lane = lanes.get(lane_name)
            if lane is not None:
                canonical_candidate = self._normalize_group_name(getattr(lane, "_map", None))
                if canonical_candidate is None:
                    canonical_candidate = self._normalize_group_name(getattr(lane, "map", None))

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

    # --- Helpers for infinite runout delegation ----------------------------------

    def _get_infinite_runout_target_group(self, fps_name: str, fps_state: 'FPSState') -> Tuple[Optional[str], Optional[str], bool, Optional[str]]:
        """
        Return (target_group, runout_lane_name, delegate_to_afc, source_lane_name).
        If delegation is required, delegate_to_afc will be True.
        """
        current_group = fps_state.current_group
        normalized_group = self._normalize_group_name(current_group)
        if normalized_group is None:
            return None, None, False, None

        afc = self._get_afc()
        if afc is None:
            return None, None, False, None

        lane_name, resolved_group = self._resolve_lane_for_state(fps_state, normalized_group, afc)
        if resolved_group and resolved_group != normalized_group:
            normalized_group = resolved_group
            fps_state.current_group = resolved_group

        if not lane_name:
            logging.debug("OAMS: Unable to resolve AFC lane for group %s on %s", normalized_group, fps_name)
            return None, None, False, None

        lane = afc.lanes.get(lane_name)
        if lane is None:
            return None, None, False, lane_name

        runout_lane_name = getattr(lane, "runout_lane", None)
        if not runout_lane_name:
            return None, None, False, lane_name

        target_lane = afc.lanes.get(runout_lane_name)
        if target_lane is None:
            logging.warning("OAMS: Runout lane %s for %s on %s is not available; deferring to AFC", runout_lane_name, normalized_group, fps_name)
            return None, runout_lane_name, True, lane_name

        source_unit = self._lane_unit_map.get(lane_name)
        target_unit = self._lane_unit_map.get(runout_lane_name)
        if source_unit and target_unit and source_unit != target_unit:
            logging.debug("OAMS: Runout lane %s for %s on %s belongs to different unit %s; deferring to AFC", runout_lane_name, normalized_group, fps_name, source_unit)
            return None, runout_lane_name, True, lane_name

        source_extruder = getattr(lane, "extruder_obj", None)
        target_extruder = getattr(target_lane, "extruder_obj", None)
        if source_extruder is not None and target_extruder is not None and source_extruder is not target_extruder:
            logging.debug("OAMS: Deferring infinite runout for %s on %s because lane %s spools to %s", normalized_group, fps_name, lane_name, runout_lane_name)
            return None, runout_lane_name, True, lane_name

        target_group = self._canonical_group_by_lane.get(runout_lane_name)
        if not target_group:
            target_group = self._normalize_group_name(getattr(target_lane, "_map", None))
        if not target_group:
            target_group = self._normalize_group_name(getattr(target_lane, "map", None))
        if not target_group:
            logging.debug("OAMS: Runout lane %s for %s on %s has no canonical group; deferring to AFC", runout_lane_name, normalized_group, fps_name)
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

        # If mapping is to same group or external to OAMS, delegate to AFC
        if target_group == normalized_group:
            logging.debug("OAMS: Runout lane %s for %s on %s does not map to a different filament group; deferring to AFC", runout_lane_name, normalized_group, fps_name)
            return None, runout_lane_name, True, lane_name

        if normalized_group not in self.filament_groups:
            logging.debug("OAMS: Source group %s is not managed by OAMS; deferring to AFC", normalized_group)
            return None, runout_lane_name, True, lane_name

        if target_group not in self.filament_groups:
            logging.debug("OAMS: Runout mapping %s -> %s is not managed by OAMS; deferring to AFC", normalized_group, target_group)
            return None, runout_lane_name, True, lane_name

        source_fps = self.group_fps_name(normalized_group)
        target_fps = self.group_fps_name(target_group)
        if source_fps != fps_name or target_fps != fps_name:
            logging.info("OAMS: Deferring infinite runout for %s on %s to AFC lane %s because target group %s loads via %s", normalized_group, fps_name, runout_lane_name, target_group, target_fps or "unknown")
            return None, runout_lane_name, True, lane_name

        logging.info("OAMS: Infinite runout configured for %s on %s -> %s (lanes %s -> %s)", normalized_group, fps_name, target_group, lane_name, runout_lane_name)
        return target_group, runout_lane_name, False, lane_name

    def _delegate_runout_to_afc(self, fps_name: str, fps_state: 'FPSState', source_lane_name: Optional[str], target_lane_name: Optional[str]) -> bool:
        """Ask AFC to perform the infinite runout swap for the provided lane."""
        afc = self._get_afc()
        if afc is None:
            logging.debug("OAMS: Cannot delegate infinite runout for %s; AFC not available", fps_name)
            return False

        if not source_lane_name:
            logging.debug("OAMS: Cannot delegate infinite runout for %s; no source lane recorded", fps_name)
            return False

        lane = afc.lanes.get(source_lane_name)
        if lane is None:
            logging.warning("OAMS: AFC lane %s not found while delegating infinite runout for %s", source_lane_name, fps_name)
            return False

        runout_target = getattr(lane, "runout_lane", None)
        if not runout_target:
            logging.warning("OAMS: AFC lane %s has no runout target while delegating infinite runout for %s", source_lane_name, fps_name)
            return False

        # Avoid duplicate delegation
        now = self.reactor.monotonic()
        if fps_state.afc_delegation_active and now < fps_state.afc_delegation_until:
            logging.debug("OAMS: AFC infinite runout for %s still in progress; skipping duplicate trigger", fps_name)
            return True

        if runout_target not in afc.lanes:
            logging.warning("OAMS: AFC runout lane %s referenced by %s is unavailable", runout_target, source_lane_name)
            return False

        try:
            lane._perform_infinite_runout()
        except Exception:
            logging.exception("OAMS: AFC infinite runout failed for lane %s -> %s", source_lane_name, runout_target)
            fps_state.afc_delegation_active = False
            fps_state.afc_delegation_until = 0.0
            return False

        fps_state.afc_delegation_active = True
        fps_state.afc_delegation_until = now + AFC_DELEGATION_TIMEOUT
        logging.info("OAMS: Delegated infinite runout for %s via AFC lane %s -> %s", fps_name, source_lane_name, runout_target)
        return True

    # --- Determine current hardware state ----------------------------------------

    def determine_current_loaded_group(self, fps_name: str) -> Tuple[Optional[str], Optional[object], Optional[int]]:
        """
        Determine which filament group is currently loaded in the specified FPS.

        Returns (group_name, oams_object, bay_index) or (None, None, None).
        """
        fps = self.fpss.get(fps_name)
        if fps is None:
            raise ValueError(f"FPS {fps_name} not found")

        for group_name, group in self.filament_groups.items():
            for oam, bay_index in group.bays:
                try:
                    # Use compatibility wrapper instead of direct method call
                    is_loaded = self._is_bay_loaded(oam, bay_index)
                except Exception:
                    logging.exception("OAMS: Failed to query bay %s on %s while determining loaded group", bay_index, getattr(oam, "name", "<unknown>"))
                    continue
                if is_loaded and oam in fps.oams:
                    return group_name, oam, bay_index
        return None, None, None

    def determine_state(self) -> None:
        """
        Analyze hardware state and update FPS state tracking.
        """
        for fps_name, fps_state in self.current_state.fps_state.items():
            fps_state.current_group, current_oams_obj, fps_state.current_spool_idx = self.determine_current_loaded_group(fps_name)
            fps_state.current_oams = current_oams_obj.name if current_oams_obj is not None else None

            if fps_state.current_oams is not None and fps_state.current_spool_idx is not None:
                fps_state.state_name = FPSLoadState.LOADED
                fps_state.since = self.reactor.monotonic()
                fps_state.reset_stuck_spool_state()
                fps_state.reset_clog_tracker()
                self._ensure_forward_follower(fps_name, fps_state, "state detection")
            else:
                fps_state.state_name = FPSLoadState.UNLOADED
                fps_state.reset_stuck_spool_state()
                fps_state.reset_clog_tracker()
                self._cancel_post_load_pressure_check(fps_state)

    # --- ready handler to discover FPS objects and start monitors -----------------

    def handle_ready(self) -> None:
        # Discover FPS modules
        for fps_name, fps in self.printer.lookup_objects(module="fps"):
            self.fpss[fps_name] = fps
            self.current_state.add_fps_state(fps_name)

        if not self.fpss:
            raise ValueError("No FPS found in system, this is required for OAMS to work")

        self._rebuild_group_fps_index()
        self.determine_state()
        self.start_monitors()
        self.ready = True

    # --- G-code command registration ---------------------------------------------

    def register_commands(self):
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command("OAMSM_UNLOAD_FILAMENT", self.cmd_UNLOAD_FILAMENT, desc=self.cmd_UNLOAD_FILAMENT_help)
        gcode.register_command("OAMSM_LOAD_FILAMENT", self.cmd_LOAD_FILAMENT, desc=self.cmd_LOAD_FILAMENT_help)
        gcode.register_command("OAMSM_FOLLOWER", self.cmd_FOLLOWER, desc=self.cmd_FOLLOWER_help)
        gcode.register_command("OAMSM_CLEAR_ERRORS", self.cmd_CLEAR_ERRORS, desc=self.cmd_CLEAR_ERRORS_help)

    cmd_CLEAR_ERRORS_help = "Clear the error state of the OAMS"

    def cmd_CLEAR_ERRORS(self, gcmd):
        if len(self.monitor_timers) > 0:
            self.stop_monitors()
        for fps_name, fps_state in self.current_state.fps_state.items():
            fps_state.encoder_samples.clear()
            fps_state.reset_stuck_spool_state()
            self._cancel_post_load_pressure_check(fps_state)

        for oams_name, oam in self.oams.items():
            try:
                oam.clear_errors()
            except Exception:
                logging.exception("OAMS: Failed to clear errors on %s", oams_name)
        self.determine_state()
        self.start_monitors()

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
        if fps_name is None:
            gcmd.respond_info("Missing FPS parameter")
            return
        fps_full = "fps " + fps_name
        if fps_full not in self.fpss:
            gcmd.respond_info(f"FPS {fps_full} does not exist")
            return
        fps_state = self.current_state.fps_state[fps_full]
        if fps_state.state_name == FPSLoadState.UNLOADED:
            gcmd.respond_info(f"FPS {fps_full} is already unloaded")
            return
        if fps_state.state_name == FPSLoadState.LOADING:
            gcmd.respond_info(f"FPS {fps_full} is currently loading a spool")
            return
        if fps_state.state_name == FPSLoadState.UNLOADING:
            gcmd.respond_info(f"FPS {fps_full} is currently unloading a spool")
            return
        oams_obj = self.oams.get(fps_state.current_oams)
        if oams_obj is None:
            gcmd.respond_info(f"OAMS {fps_state.current_oams} is not available for follower control")
            return

        try:
            oams_obj.set_oams_follower(enable, direction)
            encoder_clicks = oams_obj.encoder_clicks
            current_spool = oams_obj.current_spool
        except Exception:
            logging.exception("OAMS: Failed to set follower %s direction %s on %s", enable, direction, fps_state.current_oams)
            gcmd.respond_info(f"Failed to set follower on {fps_state.current_oams}. Check logs for details.")
            return

        fps_state.following = bool(enable)
        fps_state.direction = direction
        fps_state.encoder = encoder_clicks
        fps_state.current_spool_idx = current_spool

    cmd_UNLOAD_FILAMENT_help = "Unload a spool from any of the OAMS if any is loaded"

    def cmd_UNLOAD_FILAMENT(self, gcmd):
        fps_name = gcmd.get('FPS')
        if fps_name is None:
            gcmd.respond_info("Missing FPS parameter")
            return
        fps_full = "fps " + fps_name
        if fps_full not in self.fpss:
            gcmd.respond_info(f"FPS {fps_full} does not exist")
            return
        fps_state = self.current_state.fps_state[fps_full]
        if fps_state.state_name == FPSLoadState.UNLOADED:
            gcmd.respond_info(f"FPS {fps_full} is already unloaded")
            return
        if fps_state.state_name == FPSLoadState.LOADING:
            gcmd.respond_info(f"FPS {fps_full} is currently loading a spool")
            return
        if fps_state.state_name == FPSLoadState.UNLOADING:
            gcmd.respond_info(f"FPS {fps_full} is currently unloading a spool")
            return

        success, message = self._unload_filament_for_fps(fps_full)
        if not success or (message and message != "Spool unloaded successfully"):
            gcmd.respond_info(message)

    cmd_LOAD_FILAMENT_help = "Load a spool from an specific group"

    def cmd_LOAD_FILAMENT(self, gcmd):
        group_name = gcmd.get('GROUP')
        if group_name not in self.filament_groups:
            gcmd.respond_info(f"Group {group_name} does not exist")
            return
        fps_name = self.group_fps_name(group_name)
        if fps_name is None:
            gcmd.respond_info(f"No FPS associated with group {group_name}")
            return
        fps_state = self.current_state.fps_state[fps_name]
        if fps_state.state_name == FPSLoadState.LOADED:
            gcmd.respond_info(f"Group {group_name} is already loaded")
            return
        success, message = self._load_filament_for_group(group_name)
        gcmd.respond_info(message)

    # --- Pause / messaging helpers -----------------------------------------------

    def _pause_printer_message(self, message):
        logging.info("OAMS: %s", message)
        try:
            gcode = self.printer.lookup_object("gcode")
            pause_message = f"Print has been paused: {message}"
            gcode.run_script(f"M118 {pause_message}")
            gcode.run_script(f"M114 {pause_message}")
        except Exception:
            logging.exception("OAMS: Failed to send pause notification gcode")

        try:
            toolhead = self.printer.lookup_object("toolhead")
            homed_axes = toolhead.get_status(self.reactor.monotonic()).get("homed_axes", "")
        except Exception:
            logging.exception("OAMS: Failed to query toolhead state during pause handling")
            return

        if all(axis in homed_axes for axis in ("x", "y", "z")):
            try:
                gcode.run_script("PAUSE")
            except Exception:
                logging.exception("OAMS: Failed to run PAUSE script for clog handling")
        else:
            logging.warning("OAMS: Skipping PAUSE command because axes are not homed (homed_axes=%s)", homed_axes)

    # --- Post-load pressure check management -------------------------------------

    def _cancel_post_load_pressure_check(self, fps_state: "FPSState") -> None:
        timer = getattr(fps_state, "post_load_pressure_timer", None)
        if timer is not None:
            try:
                self.reactor.unregister_timer(timer)
            except Exception:
                logging.exception("OAMS: Failed to cancel post-load pressure timer")
        fps_state.post_load_pressure_timer = None
        fps_state.post_load_pressure_start = None

    def _schedule_post_load_pressure_check(self, fps_name: str, fps_state: "FPSState") -> None:
        """Verify the FPS pressure settles after a successful load."""

        self._cancel_post_load_pressure_check(fps_state)

        def _monitor_pressure(eventtime, manager=self, fps_name=fps_name):
            tracked_state = manager.current_state.fps_state.get(fps_name)
            fps = manager.fpss.get(fps_name)

            if tracked_state is None or fps is None:
                if tracked_state is not None:
                    manager._cancel_post_load_pressure_check(tracked_state)
                return manager.reactor.NEVER

            if tracked_state.state_name != FPSLoadState.LOADED:
                manager._cancel_post_load_pressure_check(tracked_state)
                return manager.reactor.NEVER

            pressure = float(getattr(fps, "fps_value", 0.0))
            if pressure <= POST_LOAD_PRESSURE_THRESHOLD:
                manager._cancel_post_load_pressure_check(tracked_state)
                return manager.reactor.NEVER

            now = manager.reactor.monotonic()
            if tracked_state.post_load_pressure_start is None:
                tracked_state.post_load_pressure_start = now
                return eventtime + POST_LOAD_PRESSURE_CHECK_PERIOD

            if now - tracked_state.post_load_pressure_start < POST_LOAD_PRESSURE_DWELL:
                return eventtime + POST_LOAD_PRESSURE_CHECK_PERIOD

            oams_obj = None
            if tracked_state.current_oams is not None:
                oams_obj = manager.oams.get(tracked_state.current_oams)
            if oams_obj is not None and tracked_state.current_spool_idx is not None:
                try:
                    oams_obj.set_led_error(tracked_state.current_spool_idx, 1)
                except Exception:
                    logging.exception("OAMS: Failed to set clog LED on %s spool %s after loading", fps_name, tracked_state.current_spool_idx)

            tracked_state.clog_active = True

            message = (
                f"Possible clog detected after loading {tracked_state.current_group or fps_name}: "
                f"FPS pressure {pressure:.2f} remained above {POST_LOAD_PRESSURE_THRESHOLD:.2f}"
            )
            manager._pause_printer_message(message)
            manager._cancel_post_load_pressure_check(tracked_state)
            return manager.reactor.NEVER

        timer = self.reactor.register_timer(_monitor_pressure, self.reactor.NOW)
        fps_state.post_load_pressure_timer = timer
        fps_state.post_load_pressure_start = None

    # --- Follower management helpers ---------------------------------------------

    def _enable_follower(self, fps_name: str, fps_state: "FPSState", oams: Optional[Any], direction: int, context: str) -> None:
        if fps_state.current_spool_idx is None:
            return

        if oams is None and fps_state.current_oams is not None:
            oams = self.oams.get(fps_state.current_oams)
        if oams is None:
            return

        direction = direction if direction in (0, 1) else 1

        try:
            oams.set_oams_follower(1, direction)
            fps_state.following = True
            fps_state.direction = direction
            logging.debug("OAMS: Enabled follower for %s spool %s after %s.", fps_name, fps_state.current_spool_idx, context)
        except Exception:
            logging.exception("OAMS: Failed to enable follower for %s after %s", fps_name, context)

    def _ensure_forward_follower(self, fps_name: str, fps_state: "FPSState", context: str) -> None:
        if (
            fps_state.current_oams is None
            or fps_state.current_spool_idx is None
            or fps_state.stuck_spool_active
            or fps_state.state_name != FPSLoadState.LOADED
        ):
            return

        if fps_state.following and fps_state.direction == 1:
            return

        oams = self.oams.get(fps_state.current_oams)
        if oams is None:
            return

        fps_state.direction = 1
        self._enable_follower(fps_name, fps_state, oams, 1, context)

    def _restore_follower_if_needed(self, fps_name: str, fps_state: "FPSState", oams: Optional[Any], context: str) -> None:
        if not fps_state.stuck_spool_restore_follower:
            return

        if fps_state.current_oams is None:
            fps_state.stuck_spool_restore_follower = False
            return

        if oams is None:
            oams = self.oams.get(fps_state.current_oams)
        if oams is None:
            return

        direction = fps_state.stuck_spool_restore_direction
        self._enable_follower(fps_name, fps_state, oams, direction, context)
        if fps_state.following:
            fps_state.stuck_spool_restore_follower = False
            logging.info("OAMS: Restarted follower for %s spool %s after %s.", fps_name, fps_state.current_spool_idx, context)

    def _handle_printing_resumed(self, _eventtime):
        for fps_name, fps_state in self.current_state.fps_state.items():
            oams = self.oams.get(fps_state.current_oams) if fps_state.current_oams else None
            if fps_state.stuck_spool_restore_follower:
                self._restore_follower_if_needed(fps_name, fps_state, oams, "print resume")
            elif fps_state.current_oams is not None and fps_state.current_spool_idx is not None and not fps_state.following:
                self._ensure_forward_follower(fps_name, fps_state, "print resume")

    # --- Stuck spool handling ----------------------------------------------------

    def _trigger_stuck_spool_pause(self, fps_name: str, fps_state: "FPSState", oams: Optional[Any], message: str) -> None:
        if fps_state.stuck_spool_active:
            return

        spool_idx = fps_state.current_spool_idx
        if oams is None and fps_state.current_oams is not None:
            oams = self.oams.get(fps_state.current_oams)

        if oams is not None and spool_idx is not None:
            try:
                oams.set_led_error(spool_idx, 1)
            except Exception:
                logging.exception("OAMS: Failed to set stuck spool LED on %s spool %s", fps_name, spool_idx)
            direction = fps_state.direction if fps_state.direction in (0, 1) else 1
            fps_state.direction = direction
            fps_state.stuck_spool_restore_follower = True
            fps_state.stuck_spool_restore_direction = direction
            if fps_state.following:
                try:
                    oams.set_oams_follower(0, direction)
                except Exception:
                    logging.exception("OAMS: Failed to stop follower for %s spool %s during stuck spool pause", fps_name, spool_idx)
            fps_state.following = False

        if oams is not None:
            try:
                oams.abort_current_action()
            except Exception:
                logging.exception("OAMS: Failed to abort active action for %s during stuck spool pause", fps_name)

        fps_state.stuck_spool_active = True
        fps_state.stuck_spool_start_time = None
        self._pause_printer_message(message)

    # --- Monitors for load/unload/stuck/clog -------------------------------------

    def _monitor_unload_speed_for_fps(self, fps_name):
        def _monitor_unload_speed(eventtime, manager=self, fps_name=fps_name):
            fps_state = manager.current_state.fps_state[fps_name]
            oams = manager.oams.get(fps_state.current_oams) if fps_state.current_oams else None
            if fps_state.state_name == FPSLoadState.UNLOADING and manager.reactor.monotonic() - fps_state.since > MONITOR_ENCODER_UNLOADING_SPEED_AFTER:
                if oams is None:
                    return eventtime + MONITOR_ENCODER_PERIOD
                try:
                    encoder_value = oams.encoder_clicks
                except Exception:
                    logging.exception("OAMS: Failed to read encoder while monitoring unload on %s", fps_name)
                    return eventtime + MONITOR_ENCODER_PERIOD

                fps_state.encoder_samples.append(encoder_value)
                if len(fps_state.encoder_samples) < ENCODER_SAMPLES:
                    return eventtime + MONITOR_ENCODER_PERIOD
                encoder_diff = abs(fps_state.encoder_samples[-1] - fps_state.encoder_samples[0])
                logging.info("OAMS[%s] Unload Monitor: Encoder diff %d", getattr(oams, "oams_idx", -1), encoder_diff)
                if encoder_diff < MIN_ENCODER_DIFF:
                    try:
                        oams.set_led_error(fps_state.current_spool_idx, 1)
                    except Exception:
                        logging.exception("OAMS: Failed to set unload LED on %s", fps_name)
                    manager._pause_printer_message("Printer paused because the unloading speed of the moving filament was too low")
                    logging.info("after unload speed too low")
                    manager.stop_monitors()
                    return manager.printer.get_reactor().NEVER
            return eventtime + MONITOR_ENCODER_PERIOD
        return _monitor_unload_speed

    def _monitor_load_speed_for_fps(self, fps_name):
        def _monitor_load_speed(eventtime, manager=self, fps_name=fps_name):
            fps_state = manager.current_state.fps_state[fps_name]
            oams = manager.oams.get(fps_state.current_oams) if fps_state.current_oams else None
            if fps_state.stuck_spool_active:
                return eventtime + MONITOR_ENCODER_PERIOD
            if fps_state.state_name == FPSLoadState.LOADING and manager.reactor.monotonic() - fps_state.since > MONITOR_ENCODER_LOADING_SPEED_AFTER:
                if oams is None:
                    return eventtime + MONITOR_ENCODER_PERIOD
                try:
                    encoder_value = oams.encoder_clicks
                except Exception:
                    logging.exception("OAMS: Failed to read encoder while monitoring load on %s", fps_name)
                    return eventtime + MONITOR_ENCODER_PERIOD

                fps_state.encoder_samples.append(encoder_value)
                if len(fps_state.encoder_samples) < ENCODER_SAMPLES:
                    return eventtime + MONITOR_ENCODER_PERIOD
                encoder_diff = abs(fps_state.encoder_samples[-1] - fps_state.encoder_samples[0])
                logging.info("OAMS[%s] Load Monitor: Encoder diff %d", getattr(oams, "oams_idx", -1), encoder_diff)
                if encoder_diff < MIN_ENCODER_DIFF:
                    group_label = fps_state.current_group or fps_name
                    spool_label = str(fps_state.current_spool_idx) if fps_state.current_spool_idx is not None else "unknown"
                    message = ("Spool appears stuck while loading" if fps_state.current_group is None else f"Spool appears stuck while loading {group_label} spool {spool_label}")
                    manager._trigger_stuck_spool_pause(fps_name, fps_state, oams, message)
                    manager.stop_monitors()
                    return manager.printer.get_reactor().NEVER
            return eventtime + MONITOR_ENCODER_PERIOD
        return _monitor_load_speed

    def _monitor_stuck_spool_for_fps(self, fps_name):
        def _monitor_stuck_spool(eventtime, manager=self, fps_name=fps_name):
            fps_state = manager.current_state.fps_state[fps_name]
            fps = manager.fpss.get(fps_name)
            if fps is None:
                if fps_state.stuck_spool_active and fps_state.current_oams is not None:
                    oams_obj = manager.oams.get(fps_state.current_oams)
                    if oams_obj is not None and fps_state.current_spool_idx is not None:
                        try:
                            oams_obj.set_led_error(fps_state.current_spool_idx, 0)
                        except Exception:
                            logging.exception("OAMS: Failed to clear stuck spool LED on %s while fps missing", fps_name)
                fps_state.reset_stuck_spool_state()
                return eventtime + MONITOR_ENCODER_PERIOD

            if fps_state.state_name != FPSLoadState.LOADED:
                if fps_state.stuck_spool_active and fps_state.current_oams is not None and fps_state.current_spool_idx is not None:
                    oams_obj = manager.oams.get(fps_state.current_oams)
                    if oams_obj is not None:
                        try:
                            oams_obj.set_led_error(fps_state.current_spool_idx, 0)
                        except Exception:
                            logging.exception("OAMS: Failed to clear stuck spool LED on %s while not loaded", fps_name)
                fps_state.reset_stuck_spool_state()
                return eventtime + MONITOR_ENCODER_PERIOD

            oams = manager.oams.get(fps_state.current_oams) if fps_state.current_oams else None
            if oams is None or fps_state.current_spool_idx is None:
                if fps_state.stuck_spool_active and oams is not None and fps_state.current_spool_idx is not None:
                    try:
                        oams.set_led_error(fps_state.current_spool_idx, 0)
                    except Exception:
                        logging.exception("OAMS: Failed to clear stuck spool LED on %s without active spool", fps_name)
                fps_state.reset_stuck_spool_state()
                return eventtime + MONITOR_ENCODER_PERIOD

            try:
                idle_timeout = manager.printer.lookup_object("idle_timeout")
                is_printing = idle_timeout.get_status(eventtime)["state"] == "Printing"
            except Exception:
                is_printing = False

            monitor = manager.runout_monitors.get(fps_name)
            if monitor is not None and monitor.state != OAMSRunoutState.MONITORING:
                if fps_state.stuck_spool_active:
                    try:
                        oams.set_led_error(fps_state.current_spool_idx, 0)
                    except Exception:
                        logging.exception("OAMS: Failed to clear stuck spool LED while runout monitor inactive on %s", fps_name)
                fps_state.reset_stuck_spool_state(preserve_restore=fps_state.stuck_spool_restore_follower)
                return eventtime + MONITOR_ENCODER_PERIOD

            if not is_printing:
                if fps_state.stuck_spool_active:
                    try:
                        oams.set_led_error(fps_state.current_spool_idx, 0)
                    except Exception:
                        logging.exception("OAMS: Failed to clear stuck spool LED while idle on %s", fps_name)
                fps_state.reset_stuck_spool_state(preserve_restore=fps_state.stuck_spool_restore_follower)
                return eventtime + MONITOR_ENCODER_PERIOD

            if fps_state.since is not None and manager.reactor.monotonic() - fps_state.since < STUCK_SPOOL_LOAD_GRACE:
                fps_state.stuck_spool_start_time = None
                return eventtime + MONITOR_ENCODER_PERIOD

            if not fps_state.following or fps_state.direction != 1:
                fps_state.stuck_spool_start_time = None

                if fps_state.stuck_spool_restore_follower and is_printing and oams is not None:
                    manager._restore_follower_if_needed(fps_name, fps_state, oams, "stuck spool recovery")
                return eventtime + MONITOR_ENCODER_PERIOD

            pressure = float(getattr(fps, "fps_value", 0.0))
            now = manager.reactor.monotonic()

            if pressure <= STUCK_SPOOL_PRESSURE_THRESHOLD:
                if fps_state.stuck_spool_start_time is None:
                    fps_state.stuck_spool_start_time = now
                elif not fps_state.stuck_spool_active and now - fps_state.stuck_spool_start_time >= STUCK_SPOOL_DWELL:
                    message = "Spool appears stuck"
                    if fps_state.current_group is not None:
                        message = f"Spool appears stuck on {fps_state.current_group} spool {fps_state.current_spool_idx}"
                    manager._trigger_stuck_spool_pause(fps_name, fps_state, oams, message)
            else:
                if fps_state.stuck_spool_active:
                    try:
                        oams.set_led_error(fps_state.current_spool_idx, 0)
                    except Exception:
                        logging.exception("OAMS: Failed to clear stuck spool LED on %s spool %d", fps_name, fps_state.current_spool_idx)
                if fps_state.stuck_spool_restore_follower and is_printing:
                    manager._restore_follower_if_needed(fps_name, fps_state, oams, "stuck spool recovery")
                elif is_printing and not fps_state.following:
                    manager._ensure_forward_follower(fps_name, fps_state, "stuck spool recovery")
                if not fps_state.stuck_spool_restore_follower:
                    fps_state.reset_stuck_spool_state()

            return eventtime + MONITOR_ENCODER_PERIOD
        return _monitor_stuck_spool

    def _monitor_clog_for_fps(self, fps_name):
        def _monitor_clog(eventtime, manager=self, fps_name=fps_name):
            fps_state = manager.current_state.fps_state[fps_name]
            fps = manager.fpss.get(fps_name)

            if fps_state.state_name != FPSLoadState.LOADED:
                if fps_state.clog_active and fps_state.current_oams is not None and fps_state.current_spool_idx is not None:
                    oams_obj = manager.oams.get(fps_state.current_oams)
                    if oams_obj is not None:
                        try:
                            oams_obj.set_led_error(fps_state.current_spool_idx, 0)
                        except Exception:
                            logging.exception("OAMS: Failed to clear clog LED on %s spool %s while idle", fps_name, fps_state.current_spool_idx)
                fps_state.reset_clog_tracker()
                return eventtime + MONITOR_ENCODER_PERIOD

            oams = manager.oams.get(fps_state.current_oams) if fps_state.current_oams else None
            if oams is None or fps_state.current_spool_idx is None or fps is None:
                fps_state.reset_clog_tracker()
                return eventtime + MONITOR_ENCODER_PERIOD

            try:
                hes_values = oams.hub_hes_value
                spool_available = bool(hes_values[fps_state.current_spool_idx])
            except Exception:
                logging.exception("OAMS: Failed to read HES values while monitoring clogs on %s", fps_name)
                return eventtime + MONITOR_ENCODER_PERIOD

            if spool_available:
                if fps_state.clog_active:
                    try:
                        oams.set_led_error(fps_state.current_spool_idx, 0)
                    except Exception:
                        logging.exception("OAMS: Failed to clear clog LED on %s spool %s after runout", fps_name, fps_state.current_spool_idx)
                fps_state.reset_clog_tracker()
                return eventtime + MONITOR_ENCODER_PERIOD

            try:
                idle_timeout = manager.printer.lookup_object("idle_timeout")
                is_printing = idle_timeout.get_status(eventtime)["state"] == "Printing"
            except Exception:
                is_printing = False

            monitor = manager.runout_monitors.get(fps_name)
            if monitor is not None and monitor.state != OAMSRunoutState.MONITORING:
                if fps_state.clog_active:
                    try:
                        oams.set_led_error(fps_state.current_spool_idx, 0)
                    except Exception:
                        logging.exception("OAMS: Failed to clear clog LED on %s spool %s while runout monitor inactive", fps_name, fps_state.current_spool_idx)
                fps_state.reset_clog_tracker()
                return eventtime + MONITOR_ENCODER_PERIOD

            if not is_printing:
                if fps_state.clog_active:
                    try:
                        oams.set_led_error(fps_state.current_spool_idx, 0)
                    except Exception:
                        logging.exception("OAMS: Failed to clear clog LED on %s spool %s while printer idle", fps_name, fps_state.current_spool_idx)
                fps_state.reset_clog_tracker()
                return eventtime + MONITOR_ENCODER_PERIOD

            try:
                extruder_pos = float(getattr(fps.extruder, "last_position", 0.0))
            except Exception:
                logging.exception("OAMS: Failed to read extruder position while monitoring clogs on %s", fps_name)
                return eventtime + MONITOR_ENCODER_PERIOD

            try:
                encoder_clicks = int(getattr(oams, "encoder_clicks", 0))
            except Exception:
                logging.exception("OAMS: Failed to read encoder clicks while monitoring clogs on %s", fps_name)
                return eventtime + MONITOR_ENCODER_PERIOD

            try:
                pressure = float(getattr(fps, "fps_value", 0.0))
            except Exception:
                logging.exception("OAMS: Failed to read FPS pressure while monitoring clogs on %s", fps_name)
                return eventtime + MONITOR_ENCODER_PERIOD

            now = manager.reactor.monotonic()
            if fps_state.clog_start_extruder is None:
                fps_state.prime_clog_tracker(extruder_pos, encoder_clicks, pressure, now)
                return eventtime + MONITOR_ENCODER_PERIOD

            if extruder_pos < (fps_state.clog_last_extruder or extruder_pos):
                fps_state.prime_clog_tracker(extruder_pos, encoder_clicks, pressure, now)
                return eventtime + MONITOR_ENCODER_PERIOD

            fps_state.clog_last_extruder = extruder_pos
            if fps_state.clog_min_pressure is None or pressure < fps_state.clog_min_pressure:
                fps_state.clog_min_pressure = pressure
            if fps_state.clog_max_pressure is None or pressure > fps_state.clog_max_pressure:
                fps_state.clog_max_pressure = pressure

            extrusion_delta = extruder_pos - (fps_state.clog_start_extruder or extruder_pos)
            encoder_delta = abs(encoder_clicks - (fps_state.clog_start_encoder or encoder_clicks))
            pressure_span = (fps_state.clog_max_pressure or pressure) - (fps_state.clog_min_pressure or pressure)

            settings = manager.clog_settings
            if extrusion_delta < settings["extrusion_window"]:
                return eventtime + MONITOR_ENCODER_PERIOD

            if encoder_delta > settings["encoder_slack"] or pressure_span > settings["pressure_band"]:
                fps_state.prime_clog_tracker(extruder_pos, encoder_clicks, pressure, now)
                return eventtime + MONITOR_ENCODER_PERIOD

            if now - (fps_state.clog_start_time or now) < settings["dwell"]:
                return eventtime + MONITOR_ENCODER_PERIOD

            if not fps_state.clog_active:
                try:
                    oams.set_led_error(fps_state.current_spool_idx, 1)
                except Exception:
                    logging.exception("OAMS: Failed to set clog LED on %s spool %s", fps_name, fps_state.current_spool_idx)
                pressure_mid = (fps_state.clog_min_pressure + fps_state.clog_max_pressure) / 2.0
                message = (
                    f"Clog suspected on {fps_state.current_group or fps_name}: "
                    f"extruder advanced {extrusion_delta:.1f}mm while encoder moved {encoder_delta} counts "
                    f"with FPS {pressure_mid:.2f} near {CLOG_PRESSURE_TARGET:.2f}"
                )
                fps_state.clog_active = True
                manager._pause_printer_message(message)

            return eventtime + MONITOR_ENCODER_PERIOD
        return _monitor_clog

    # --- Monitors lifecycle ------------------------------------------------------

    def start_monitors(self):
        self.monitor_timers = []
        self.runout_monitors = {}
        reactor = self.printer.get_reactor()
        for (fps_name, fps_state) in self.current_state.fps_state.items():
            self.monitor_timers.append(reactor.register_timer(self._monitor_unload_speed_for_fps(fps_name), reactor.NOW))
            self.monitor_timers.append(reactor.register_timer(self._monitor_load_speed_for_fps(fps_name), reactor.NOW))
            self.monitor_timers.append(reactor.register_timer(self._monitor_stuck_spool_for_fps(fps_name), reactor.NOW))
            self.monitor_timers.append(reactor.register_timer(self._monitor_clog_for_fps(fps_name), reactor.NOW))

            # prepare reload callback that uses the manager's infinite runout logic
            def _reload_callback(fps_name=fps_name, fps_state=fps_state):
                monitor = self.runout_monitors.get(fps_name)
                source_group = fps_state.current_group
                target_group, target_lane, delegate_to_afc, source_lane = self._get_infinite_runout_target_group(fps_name, fps_state)
                source_group = fps_state.current_group

                if delegate_to_afc:
                    delegated = self._delegate_runout_to_afc(fps_name, fps_state, source_lane, target_lane)
                    if delegated:
                        fps_state.reset_runout_positions()
                        if monitor:
                            monitor.reset()
                            monitor.start()
                        return
                    logging.error("OAMS: Failed to delegate infinite runout for %s on %s via AFC", fps_name, source_group or "<unknown>")
                    fps_state.reset_runout_positions()
                    self._pause_printer_message(f"Unable to delegate infinite runout for {source_group or fps_name}")
                    if monitor:
                        monitor.paused()
                    return

                group_to_load = target_group or source_group

                if target_group:
                    logging.info("OAMS: Infinite runout triggered for %s on %s -> %s", fps_name, source_group, target_group)
                    unload_success, unload_message = self._unload_filament_for_fps(fps_name)
                    if not unload_success:
                        logging.error("OAMS: Failed to unload filament during infinite runout on %s: %s", fps_name, unload_message)
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
                    logging.info("OAMS: Successfully loaded group %s on %s%s", group_to_load, fps_name, " after infinite runout" if target_group else "")
                    if target_group and target_lane:
                        try:
                            gcode = self.printer.lookup_object("gcode")
                            gcode.run_script(f"SET_LANE_LOADED LANE={target_lane}")
                            logging.debug("OAMS: Marked lane %s as loaded after infinite runout on %s", target_lane, fps_name)
                        except Exception:
                            logging.exception("OAMS: Failed to mark lane %s as loaded after infinite runout on %s", target_lane, fps_name)
                    fps_state.reset_runout_positions()
                    if monitor:
                        monitor.reset()
                        monitor.start()
                    return

                logging.error("OAMS: Failed to load group %s on %s: %s", group_to_load, fps_name, load_message)
                failure_message = load_message or f"No spool available for group {group_to_load}"
                self._pause_printer_message(failure_message)
                if monitor:
                    monitor.paused()
                return

            # create and start runout monitor for this FPS
            fps_reload_margin = getattr(self.fpss[fps_name], "reload_before_toolhead_distance", None)
            if fps_reload_margin is None:
                fps_reload_margin = self.reload_before_toolhead_distance
            else:
                logging.debug("OAMS: Using FPS-specific reload margin %.2f mm for %s", fps_reload_margin, fps_name)

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
            try:
                self.printer.get_reactor().unregister_timer(timer)
            except Exception:
                logging.exception("OAMS: Failed to unregister monitor timer")
        self.monitor_timers = []
        for monitor in self.runout_monitors.values():
            try:
                monitor.reset()
            except Exception:
                logging.exception("OAMS: Failed to reset runout monitor")
        self.runout_monitors = {}

    # --- Load / unload helpers ---------------------------------------------------

    def _unload_filament_for_fps(self, fps_name: str) -> Tuple[bool, str]:
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
            fps_state.reset_stuck_spool_state()
            fps_state.reset_clog_tracker()
            self._cancel_post_load_pressure_check(fps_state)
            return True, "Spool already unloaded"

        try:
            fps_state.state_name = FPSLoadState.UNLOADING
            fps_state.encoder = oams.encoder_clicks
            fps_state.since = self.reactor.monotonic()
            fps_state.current_oams = oams.name
            fps_state.current_spool_idx = oams.current_spool
        except Exception:
            logging.exception("OAMS: Failed to capture unload state for %s", fps_name)
            return False, f"Failed to prepare unload on {fps_name}"

        try:
            success, message = oams.unload_spool()
        except Exception:
            logging.exception("OAMS: Exception while unloading filament on %s", fps_name)
            return False, f"Exception unloading filament on {fps_name}"

        if success:
            fps_state.state_name = FPSLoadState.UNLOADED
            fps_state.following = False
            fps_state.direction = 0
            fps_state.since = self.reactor.monotonic()
            fps_state.current_group = None
            fps_state.current_spool_idx = None
            self.current_group = None
            fps_state.reset_stuck_spool_state()
            fps_state.reset_clog_tracker()
            self._cancel_post_load_pressure_check(fps_state)
            return True, message

        fps_state.state_name = FPSLoadState.LOADED
        return False, message

    def _load_filament_for_group(self, group_name: str) -> Tuple[bool, str]:
        if group_name not in self.filament_groups:
            return False, f"Group {group_name} does not exist"

        fps_name = self.group_fps_name(group_name)
        if fps_name is None:
            return False, f"No FPS associated with group {group_name}"

        fps_state = self.current_state.fps_state[fps_name]
        self._cancel_post_load_pressure_check(fps_state)

        for (oam, bay_index) in self.filament_groups[group_name].bays:
            try:
                # Use compatibility wrapper
                is_ready = self._is_bay_ready(oam, bay_index)
            except Exception:
                logging.exception(
                    "OAMS: Failed to query readiness of bay %s on %s",
                    bay_index,
                    getattr(oam, "name", "<unknown>"),
                )
                continue

            if not is_ready:
                continue

            try:
                fps_state.state_name = FPSLoadState.LOADING
                fps_state.encoder = oam.encoder_clicks
                fps_state.since = self.reactor.monotonic()
                fps_state.current_oams = oam.name
                fps_state.current_spool_idx = bay_index
            except Exception:
                logging.exception(
                    "OAMS: Failed to capture load state for group %s bay %s",
                    group_name,
                    bay_index,
                )
                fps_state.state_name = FPSLoadState.UNLOADED
                fps_state.current_group = None
                fps_state.current_spool_idx = None
                fps_state.current_oams = None
                continue

            try:
                success, message = oam.load_spool(bay_index)
            except Exception:
                logging.exception(
                    "OAMS: Exception while loading group %s bay %s",
                    group_name,
                    bay_index,
                )
                success, message = False, f"Exception loading spool {bay_index} on {group_name}"

            if success:
                fps_state.current_group = group_name
                fps_state.current_oams = oam.name
                fps_state.current_spool_idx = bay_index
                fps_state.state_name = FPSLoadState.LOADED
                fps_state.since = self.reactor.monotonic()
                fps_state.direction = 1
                self.current_group = group_name
                fps_state.reset_stuck_spool_state()
                fps_state.reset_clog_tracker()
                self._ensure_forward_follower(fps_name, fps_state, "load filament")
                self._schedule_post_load_pressure_check(fps_name, fps_state)
                return True, message

            # failed for this bay, clear state and continue trying next bay
            fps_state.state_name = FPSLoadState.UNLOADED
            fps_state.current_group = None
            fps_state.current_spool_idx = None
            fps_state.current_oams = None
            fps_state.following = False
            fps_state.reset_stuck_spool_state()
            fps_state.reset_clog_tracker()
            self._cancel_post_load_pressure_check(fps_state)
            return False, message

        return False, f"No spool available for group {group_name}"

    # --- Status RPC --------------------------------------------------------------

    def get_status(self, eventtime: float) -> Dict[str, Dict[str, Any]]:
        attributes: Dict[str, Dict[str, Any]] = {"oams": {}}
        for name, oam in self.oams.items():
            status_name = name.split()[-1]
            try:
                oam_status = {
                    "action_status": oam.action_status,
                    "action_status_code": oam.action_status_code,
                    "action_status_value": oam.action_status_value,
                }
            except Exception:
                logging.exception("OAMS: Failed to fetch status from %s", name)
                oam_status = {"action_status": "error", "action_status_code": None, "action_status_value": None}
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


def load_config(config):
    return OAMSManager(config)
