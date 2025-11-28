# OpenAMS Manager 
#
# Copyright (C) 2025 JR Lomas <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# OPTIMIZATIONS APPLIED:
# 1. Adaptive Polling Intervals: Monitors use 2.0s active / 4.0s idle intervals
#    to reduce CPU usage when printer is idle (15-25% reduction in polling overhead)
# 2. Object Caching: Frequently accessed objects (idle_timeout, gcode, toolhead, AFC)
#    are cached at initialization to avoid repeated lookups
# 3. State Change Tracking: FPSState tracks consecutive idle polls to intelligently
#    switch between active and idle polling intervals
# 4. Pre-checks: Monitor functions skip expensive sensor reads when idle and stable
#
# CONFIGURABLE DETECTION PARAMETERS:
# The following parameters can be tuned in [oams_manager] config section:
# - stuck_spool_load_grace: Grace period (seconds) after load before stuck spool detection (default: 8.0)
# - stuck_spool_pressure_threshold: Pressure below which stuck detection starts (default: 0.08)
# - stuck_spool_pressure_clear_threshold: Pressure above which stuck state clears - hysteresis (default: 0.12)
# - clog_pressure_target: Target FPS pressure for clog detection (default: 0.50)
# - post_load_pressure_dwell: Duration (seconds) to monitor pressure after load (default: 15.0)
# - load_fps_stuck_threshold: FPS pressure above which load is considered failed (default: 0.75)
# - clog_sensitivity: Detection sensitivity level - "low", "medium", "high" (default: "medium")

import logging
import time
import traceback
from contextlib import contextmanager
from functools import partial
from typing import Optional, Tuple, Dict, List, Any, Callable

try:
    from extras.openams_integration import AMSRunoutCoordinator
except Exception:
    AMSRunoutCoordinator = None


def _normalize_extruder_name(name: Optional[str]) -> Optional[str]:
    """Return a lowercase token for comparing extruder identifiers."""
    if not name or not isinstance(name, str):
        return None

    cleaned = name.strip()
    if not cleaned:
        return None

    return cleaned.lower()

# Configuration constants
PAUSE_DISTANCE = 60
MIN_ENCODER_DIFF = 1
FILAMENT_PATH_LENGTH_FACTOR = 1.14
MONITOR_ENCODER_PERIOD = 2.0
MONITOR_ENCODER_PERIOD_IDLE = 4.0  # OPTIMIZATION: Longer interval when idle
MONITOR_ENCODER_SPEED_GRACE = 2.0
AFC_DELEGATION_TIMEOUT = 30.0
COASTING_TIMEOUT = 1800.0  # Max time to wait for hub to clear and filament to coast through PTFE (30 minutes - typical prints take 15-20 min)
IDLE_POLL_THRESHOLD = 3  # OPTIMIZATION: Polls before switching to idle interval

STUCK_SPOOL_PRESSURE_THRESHOLD = 0.08
STUCK_SPOOL_PRESSURE_CLEAR_THRESHOLD = 0.12  # Hysteresis upper threshold
STUCK_SPOOL_DWELL = 3.5
STUCK_SPOOL_LOAD_GRACE = 8.0

CLOG_PRESSURE_TARGET = 0.50
CLOG_SENSITIVITY_LEVELS = {
    "low": {"extrusion_window": 48.0, "encoder_slack": 15, "pressure_band": 0.08, "dwell": 12.0},
    "medium": {"extrusion_window": 24.0, "encoder_slack": 8, "pressure_band": 0.06, "dwell": 8.0},
    "high": {"extrusion_window": 12.0, "encoder_slack": 4, "pressure_band": 0.04, "dwell": 6.0},
}
CLOG_SENSITIVITY_DEFAULT = "medium"

POST_LOAD_PRESSURE_THRESHOLD = 0.65
POST_LOAD_PRESSURE_DWELL = 15.0
POST_LOAD_PRESSURE_CHECK_PERIOD = 0.5

# Threshold for detecting failed load - if FPS stays above this during LOADING, filament isn't engaging
LOAD_FPS_STUCK_THRESHOLD = 0.75


class OAMSRunoutState:
    """Enum for runout monitor states."""
    STOPPED = "STOPPED"
    MONITORING = "MONITORING"
    DETECTED = "DETECTED"
    COASTING = "COASTING"
    RELOADING = "RELOADING"
    PAUSED = "PAUSED"


class FPSLoadState:
    """Enum for FPS loading states"""
    UNLOADED = 0
    LOADED = 1
    LOADING = 2
    UNLOADING = 3
    
class OAMSRunoutMonitor:
    """Monitors filament runout for a specific FPS."""
    
    def __init__(self,
                 printer,
                 fps_name: str,
                 fps,
                 fps_state,
                 oams: Dict[str, Any],
                 reload_callback: Callable,
                 reload_before_toolhead_distance: float = 0.0):
        self.oams = oams
        self.printer = printer
        self.fps_name = fps_name
        self.fps_state = fps_state
        self.fps = fps

        self.state = OAMSRunoutState.STOPPED
        self.runout_position: Optional[float] = None
        self.bldc_clear_position: Optional[float] = None
        self.runout_after_position: Optional[float] = None
        self.is_cross_extruder_runout: bool = False  # Track if runout is cross-extruder

        # Track when hub sensor clears during COASTING
        self.hub_cleared: bool = False
        self.hub_clear_position: Optional[float] = None
        self.coasting_start_time: Optional[float] = None

        self.reload_before_toolhead_distance = reload_before_toolhead_distance
        self.reload_callback = reload_callback

        self.reactor = self.printer.get_reactor()

        self.hardware_service = None
        self.latest_lane_name: Optional[str] = None
        self._logged_f1s_error: bool = False
        if AMSRunoutCoordinator is not None:
            try:
                self.hardware_service = AMSRunoutCoordinator.register_runout_monitor(self)
            except Exception as e:
                logging.getLogger(__name__).error(
                    "CRITICAL: Failed to register OpenAMS monitor with AFC (AMSRunoutCoordinator). "
                    "Infinite runout and AFC integration will not function. Error: %s", e
                )
                self.hardware_service = None
        
        def _monitor_runout(eventtime):
            idle_timeout = self.printer.lookup_object("idle_timeout")
            is_printing = idle_timeout.get_status(eventtime)["state"] == "Printing"
            
            if self.state in (OAMSRunoutState.STOPPED, OAMSRunoutState.PAUSED, OAMSRunoutState.RELOADING):
                return eventtime + MONITOR_ENCODER_PERIOD
            
            if self.state == OAMSRunoutState.MONITORING:
                if getattr(fps_state, "afc_delegation_active", False):
                    now = self.reactor.monotonic()
                    if now < getattr(fps_state, "afc_delegation_until", 0.0):
                        return eventtime + MONITOR_ENCODER_PERIOD
                    fps_state.afc_delegation_active = False
                    fps_state.afc_delegation_until = 0.0
                
                oams_obj = self.oams.get(fps_state.current_oams) if fps_state.current_oams else None
                if oams_obj is None:
                    return eventtime + MONITOR_ENCODER_PERIOD

                spool_idx = fps_state.current_spool_idx
                if spool_idx is None:
                    self.latest_lane_name = None
                    return eventtime + MONITOR_ENCODER_PERIOD

                lane_name = None
                spool_empty = None
                unit_name = getattr(fps_state, "current_oams", None) or self.fps_name

                # Get lane name from hardware service if available
                if self.hardware_service is not None:
                    try:
                        lane_name = self.hardware_service.resolve_lane_for_spool(unit_name, spool_idx)
                    except Exception:
                        pass

                # Fallback to fps_state.current_lane if hardware_service didn't provide a lane name
                if lane_name is None and fps_state.current_lane is not None:
                    lane_name = fps_state.current_lane
                    logging.debug("OAMS: Using fps_state.current_lane '%s' (hardware_service didn't resolve lane name)", lane_name)

                # ALWAYS check F1S sensor for runout detection (not hub sensor)
                # This triggers runout immediately when spool runs out
                try:
                    f1s_values = oams_obj.f1s_hes_value
                    if spool_idx < 0 or spool_idx >= len(f1s_values):
                        return eventtime + MONITOR_ENCODER_PERIOD
                    spool_empty = not bool(f1s_values[spool_idx])
                    if self._logged_f1s_error:
                        logging.debug("OAMS: F1S values recovered for %s", self.fps_name)
                        self._logged_f1s_error = False
                except Exception as e:
                    if not self._logged_f1s_error:
                        logging.error("OAMS: Failed to read F1S values for runout detection on %s: %s", self.fps_name, e)
                        self._logged_f1s_error = True
                    return eventtime + MONITOR_ENCODER_PERIOD

                self.latest_lane_name = lane_name

                if (is_printing and fps_state.state == FPSLoadState.LOADED and
                    fps_state.current_lane is not None and fps_state.current_spool_idx is not None and spool_empty):
                    # Determine if this is a cross-extruder runout by checking the target lane's extruder
                    # Cross-extruder: Target lane is on a different tool/extruder
                    # Same-FPS: Target lane is on the same tool/extruder (e.g., lane8?lane7 on extruder4)
                    try:
                        afc = self.printer.lookup_object('AFC')
                        current_lane_obj = None
                        target_lane_obj = None
                        target_lane_name = None

                        if afc and hasattr(afc, 'lanes'):
                            # Log what lane we're looking for and what lanes exist
                            available_lanes = list(afc.lanes.keys()) if afc.lanes else []
                            logging.info("OAMS: Looking up lane '%s' in AFC, available lanes: %s", lane_name, available_lanes)

                            current_lane_obj = afc.lanes.get(lane_name)
                            if not current_lane_obj:
                                logging.warning("OAMS: Lane '%s' not found in AFC lanes dictionary", lane_name)
                            else:
                                target_lane_name = getattr(current_lane_obj, 'runout_lane', None)
                                logging.info("OAMS: Lane '%s' found, runout_lane attribute: %s", lane_name, target_lane_name or "None")

                                # If no runout lane is configured, pause immediately without reload attempt
                                if target_lane_name is None:
                                    logging.info("OAMS: No runout_lane configured for %s - pausing without reload", lane_name)
                                    self.state = OAMSRunoutState.PAUSED
                                    self.runout_position = fps.extruder.last_position
                                    fps_state.is_cross_extruder_runout = False

                                    # Pause the printer immediately
                                    gcode = self.printer.lookup_object('gcode')
                                    gcode.run_script_from_command(f"PAUSE")
                                    gcode.respond_info(f"Filament runout detected on {lane_name} with no reload lane configured")
                                    return eventtime + MONITOR_ENCODER_PERIOD

                                if target_lane_name:
                                    target_lane_obj = afc.lanes.get(target_lane_name)
                                    if not target_lane_obj:
                                        logging.warning("OAMS: Target lane '%s' not found in AFC lanes dictionary", target_lane_name)
                        else:
                            logging.warning("OAMS: AFC object missing or has no 'lanes' attribute - afc: %s, has lanes: %s",
                                          "found" if afc else "None", hasattr(afc, 'lanes') if afc else False)

                        # Check if target lane is on a different extruder
                        if current_lane_obj and target_lane_obj:
                            current_extruder = getattr(current_lane_obj, 'extruder_name', None)
                            target_extruder = getattr(target_lane_obj, 'extruder_name', None)

                            logging.info("OAMS: Runout detection - current lane: %s (extruder: %s), target lane: %s (extruder: %s)",
                                       lane_name, current_extruder or "None", target_lane_name or "unknown", target_extruder or "None")

                            # Cross-extruder if target is on different extruder
                            # Hub sensor state doesn't matter - by the time we detect the runout,
                            # the hub may have already cleared (F1S goes empty, then hub clears shortly after)
                            if current_extruder and target_extruder and current_extruder != target_extruder:
                                self.is_cross_extruder_runout = True
                                logging.info("OAMS: Detected cross-extruder runout: %s (extruder %s) -> %s (extruder %s)",
                                           lane_name, current_extruder, target_lane_name, target_extruder)
                            else:
                                # Same extruder = same-FPS runout (even if hub has filament from other lanes)
                                self.is_cross_extruder_runout = False
                                if current_extruder == target_extruder:
                                    logging.info("OAMS: Detected same-extruder runout: %s -> %s (both on extruder %s)",
                                               lane_name, target_lane_name, current_extruder)
                                else:
                                    logging.warning("OAMS: Defaulting to same-extruder runout (missing extruder info): %s (extruder: %s) -> %s (extruder: %s)",
                                                  lane_name, current_extruder or "None", target_lane_name or "unknown", target_extruder or "None")
                        else:
                            # Fallback: if we can't determine target, assume same-FPS
                            self.is_cross_extruder_runout = False
                            logging.warning("OAMS: Cannot determine runout type (missing lane objects) - current_lane_obj: %s, target_lane_obj: %s, defaulting to same-extruder",
                                          "found" if current_lane_obj else "None", "found" if target_lane_obj else "None")
                    except Exception as e:
                        logging.error("OAMS: Failed to determine cross-extruder runout status, defaulting to same-FPS: %s", e)
                        self.is_cross_extruder_runout = False

                    # Both cross-extruder and same-extruder runouts start in DETECTED state
                    # Cross-extruder: Skip PAUSE_DISTANCE, go straight to reload with CHANGE_TOOL
                    # Same-extruder: Use PAUSE_DISTANCE ? COASTING ? reload sequence
                    self.state = OAMSRunoutState.DETECTED
                    self.runout_position = fps.extruder.last_position

                    # Store cross-extruder flag in fps_state so reload_callback can access it
                    fps_state.is_cross_extruder_runout = self.is_cross_extruder_runout

                    # Set flag on AFC lane so shared load/prep logic doesn't error on F1S empty + hub loaded
                    if self.is_cross_extruder_runout and lane_name:
                        try:
                            afc = self.printer.lookup_object('AFC')
                            if afc and hasattr(afc, 'lanes'):
                                lane_obj = afc.lanes.get(lane_name)
                                if lane_obj:
                                    lane_obj._oams_cross_extruder_runout = True
                                    logging.info("OAMS: Set cross-extruder runout flag on lane %s to bypass shared load/prep validation", lane_name)
                        except Exception as e:
                            logging.error("OAMS: Failed to set cross-extruder runout flag on lane %s: %s", lane_name, e)

                    if self.is_cross_extruder_runout:
                        logging.info("OAMS: Cross-extruder runout detected on FPS %s (F1S empty, target on different extruder) - will trigger immediate tool change",
                                   self.fps_name)
                    else:
                        logging.info("OAMS: Same-extruder runout detected on FPS %s (F1S empty), pausing for %d mm",
                                   self.fps_name, PAUSE_DISTANCE)

                    if AMSRunoutCoordinator is not None:
                        try:
                            AMSRunoutCoordinator.notify_runout_detected(self, spool_idx, lane_name=lane_name)
                        except Exception:
                            logging.getLogger(__name__).exception("Failed to notify AFC about OpenAMS runout")

            elif self.state == OAMSRunoutState.DETECTED:
                # For cross-extruder runouts, skip PAUSE_DISTANCE and trigger reload immediately
                # For same-extruder runouts, wait for PAUSE_DISTANCE before entering COASTING
                if self.is_cross_extruder_runout:
                    # Skip PAUSE_DISTANCE and COASTING phases - trigger tool change immediately
                    logging.info("OAMS: Cross-extruder runout - triggering immediate reload (no PAUSE_DISTANCE/COASTING)")
                    self.state = OAMSRunoutState.RELOADING
                    self.reload_callback()
                else:
                    # Same-FPS runout: Wait for PAUSE_DISTANCE then enter COASTING
                    # COASTING clears old filament from shared PTFE buffer before loading new lane
                    # Follower STAYS ENABLED - it's in the hub and pushes ALL lanes' filament
                    # All lanes in an AMS unit feed into ONE hub with ONE follower
                    traveled_distance = fps.extruder.last_position - self.runout_position
                    if traveled_distance >= PAUSE_DISTANCE:
                        logging.info("OAMS: Pause complete, entering COASTING (waiting for hub to clear before counting)")
                        self.bldc_clear_position = fps.extruder.last_position
                        self.runout_after_position = 0.0
                        self.hub_cleared = False  # Reset hub clear tracking
                        self.hub_clear_position = None
                        self.coasting_start_time = self.reactor.monotonic()
                        self.state = OAMSRunoutState.COASTING

            elif self.state == OAMSRunoutState.COASTING:
                # Wait for old filament to be pushed out of shared PTFE buffer by hub follower
                # Phase 1: Wait for hub sensor to clear (old lane filament leaving hub)
                # Phase 2: Count distance through shared PTFE after hub clears

                # Check for COASTING timeout (hardware failure, sensor stuck, etc.)
                now = self.reactor.monotonic()
                if self.coasting_start_time and now - self.coasting_start_time > COASTING_TIMEOUT:
                    logging.error("OAMS: COASTING timeout on %s after %.1f seconds (hub never cleared or distance not reached)",
                                self.fps_name, now - self.coasting_start_time)
                    fps_state.reset_runout_positions()
                    self.state = OAMSRunoutState.PAUSED
                    # Pause printer with error message
                    try:
                        gcode = self.printer.lookup_object("gcode")
                        gcode.run_script(f"M118 OAMS COASTING timeout on {self.fps_name}")
                        gcode.run_script("PAUSE")
                    except Exception as e:
                        logging.error("Failed to pause printer after COASTING timeout: %s", e)
                    return eventtime + MONITOR_ENCODER_PERIOD

                # Check hub sensor for current spool
                spool_idx = fps_state.current_spool_idx
                try:
                    oams_obj = self.oams.get(fps_state.current_oams)
                    if oams_obj and spool_idx is not None:
                        hes_values = oams_obj.hub_hes_value
                        hub_has_filament = bool(hes_values[spool_idx]) if spool_idx < len(hes_values) else True

                        # Detect when hub sensor clears
                        if not self.hub_cleared and not hub_has_filament:
                            self.hub_cleared = True
                            self.hub_clear_position = fps.extruder.last_position
                            logging.info("OAMS: Hub sensor cleared at position %.1f, starting shared PTFE countdown",
                                       self.hub_clear_position)
                except Exception as e:
                    logging.error("OAMS: Failed to read hub sensor during COASTING on %s: %s", self.fps_name, e)
                    # If we can't read hub sensor, assume it's cleared and proceed
                    if not self.hub_cleared:
                        self.hub_cleared = True
                        self.hub_clear_position = fps.extruder.last_position

                # Calculate distance traveled through shared PTFE (only after hub clears)
                if self.hub_cleared and self.hub_clear_position is not None:
                    traveled_distance_after_hub_clear = max(fps.extruder.last_position - self.hub_clear_position, 0.0)
                    self.runout_after_position = traveled_distance_after_hub_clear
                else:
                    # Still waiting for hub to clear
                    self.runout_after_position = 0.0

                try:
                    path_length = getattr(self.oams[fps_state.current_oams], "filament_path_length", 0.0)
                except Exception as e:
                    logging.error("OAMS: Failed to read filament path length while coasting on %s: %s", self.fps_name, e)
                    return eventtime + MONITOR_ENCODER_PERIOD

                effective_path_length = (path_length / FILAMENT_PATH_LENGTH_FACTOR if path_length else 0.0)
                # Now consumed only includes distance AFTER hub cleared, plus reload margin
                consumed_with_margin = (self.runout_after_position + self.reload_before_toolhead_distance)

                # Log COASTING progress every 100mm (only after hub clears)
                if not hasattr(self, '_last_coast_log_position'):
                    self._last_coast_log_position = 0.0
                    logging.info("OAMS: COASTING - path_length=%.1f, effective_path_length=%.1f, reload_margin=%.1f",
                               path_length, effective_path_length, self.reload_before_toolhead_distance)

                if self.hub_cleared and self.runout_after_position - self._last_coast_log_position >= 100.0:
                    self._last_coast_log_position = self.runout_after_position
                    remaining = effective_path_length - consumed_with_margin
                    logging.info("OAMS: COASTING progress (after hub clear) - runout_after=%.1f, consumed_with_margin=%.1f, remaining=%.1f",
                               self.runout_after_position, consumed_with_margin, remaining)

                if self.hub_cleared and consumed_with_margin >= effective_path_length:
                    logging.info("OAMS: Old filament cleared shared PTFE (%.2f mm after hub clear, %.2f mm effective path), loading new lane",
                               self.runout_after_position, effective_path_length)
                    self._last_coast_log_position = 0.0  # Reset for next runout
                    self.state = OAMSRunoutState.RELOADING
                    self.reload_callback()

            return eventtime + MONITOR_ENCODER_PERIOD
        
        self._timer_callback = _monitor_runout
        self.timer = None  # Don't register timer until start() is called

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
        self.is_cross_extruder_runout = False
        # Clear COASTING state tracking
        self.hub_cleared = False
        self.hub_clear_position = None
        self.coasting_start_time = None

    def paused(self) -> None:
        self.state = OAMSRunoutState.PAUSED

    def reset(self) -> None:
        self.state = OAMSRunoutState.STOPPED
        self.runout_position = None
        self.runout_after_position = None
        self.is_cross_extruder_runout = False
        # Clear COASTING state tracking
        self.hub_cleared = False
        self.hub_clear_position = None
        self.coasting_start_time = None
        if self.timer is not None:
            self.reactor.unregister_timer(self.timer)
            self.timer = None

class OAMSState:
    """Global state container for all FPS units."""
    def __init__(self):
        self.fps_state: Dict[str, 'FPSState'] = {}
        
    def add_fps_state(self, fps_name: str) -> None:
        self.fps_state[fps_name] = FPSState()
        

class FPSState:
    """Tracks the state of a single FPS"""
    
    def __init__(self, 
                 state: int = FPSLoadState.UNLOADED,
                 current_lane: Optional[str] = None, 
                 current_oams: Optional[str] = None, 
                 current_spool_idx: Optional[int] = None):
        
        self.state = state
        self.current_lane = current_lane
        self.current_oams = current_oams
        self.current_spool_idx = current_spool_idx
        
        self.runout_position: Optional[float] = None
        self.runout_after_position: Optional[float] = None
        
        self.monitor_spool_timer = None
        self.monitor_pause_timer = None
        self.monitor_load_next_spool_timer = None
        
        self.encoder_sample_prev: Optional[int] = None
        self.encoder_sample_current: Optional[int] = None

        self.following: bool = False
        self.direction: int = 0
        self.since: Optional[float] = None

        self.afc_delegation_active: bool = False
        self.afc_delegation_until: float = 0.0

        self.is_cross_extruder_runout: bool = False  # Track if current runout is cross-extruder

        self.stuck_spool_start_time: Optional[float] = None
        self.stuck_spool_active: bool = False
        self.stuck_spool_restore_follower: bool = False
        self.stuck_spool_restore_direction: int = 1

        self.clog_active: bool = False
        self.clog_start_extruder: Optional[float] = None
        self.clog_start_encoder: Optional[int] = None
        self.clog_start_time: Optional[float] = None
        self.clog_min_pressure: Optional[float] = None
        self.clog_max_pressure: Optional[float] = None
        self.clog_last_extruder: Optional[float] = None

        self.post_load_pressure_timer = None
        self.post_load_pressure_start: Optional[float] = None

        # OPTIMIZATION: Adaptive polling state with exponential backoff
        self.consecutive_idle_polls: int = 0
        self.idle_backoff_level: int = 0  # 0-3 for exponential backoff (1x, 2x, 4x, 8x)
        self.last_state_change: Optional[float] = None

    def record_encoder_sample(self, value: int) -> Optional[int]:
        """Record encoder sample and return diff if we have 2 samples."""
        self.encoder_sample_prev = self.encoder_sample_current
        self.encoder_sample_current = value
        
        if self.encoder_sample_prev is not None:
            return abs(self.encoder_sample_current - self.encoder_sample_prev)
        return None
    
    def clear_encoder_samples(self):
        """Reset encoder tracking."""
        self.encoder_sample_prev = None
        self.encoder_sample_current = None

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
        state_names = {0: "UNLOADED", 1: "LOADED", 2: "LOADING", 3: "UNLOADING"}
        return f"FPSState(state={state_names.get(self.state, self.state)}, lane={self.current_lane}, oams={self.current_oams}, spool={self.current_spool_idx})"


class OAMSManager:
    """Main coordinator for OpenAMS system"""
    
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.logger = logging.getLogger(__name__)

        self.oams: Dict[str, Any] = {}
        self.fpss: Dict[str, Any] = {}

        self.current_state = OAMSState()
        self.afc = None
        self._afc_logged = False

        self.monitor_timers: List[Any] = []
        self.runout_monitors: Dict[str, OAMSRunoutMonitor] = {}
        self.ready: bool = False

        # Follower coast tracking: when all hub sensors go empty, coast 50mm before disabling
        self.follower_coast_distance = 50.0  # mm to coast after hub empties
        self.follower_coasting: Dict[str, bool] = {}  # oams_name -> is_coasting
        self.follower_coast_start_pos: Dict[str, float] = {}  # oams_name -> extruder position when coast started
        self.follower_had_filament: Dict[str, bool] = {}  # oams_name -> previous state (had filament)
        self.follower_manual_override: Dict[str, bool] = {}  # oams_name -> manually commanded (skip auto control)
        self.follower_last_state: Dict[str, Tuple[int, int]] = {}  # oams_name -> (enable, direction) to avoid redundant MCU commands

        # LED state tracking: avoid sending redundant LED commands to MCU
        # Key format: "oams_name:spool_idx" -> error_state (0 or 1)
        self.led_error_state: Dict[str, int] = {}  # Track last commanded LED state to avoid redundant MCU commands

        self.reload_before_toolhead_distance: float = config.getfloat("reload_before_toolhead_distance", 0.0)

        sensitivity = config.get("clog_sensitivity", CLOG_SENSITIVITY_DEFAULT).lower()
        if sensitivity not in CLOG_SENSITIVITY_LEVELS:
            self.logger.warning("Unknown clog_sensitivity '%s', using %s", sensitivity, CLOG_SENSITIVITY_DEFAULT)
            sensitivity = CLOG_SENSITIVITY_DEFAULT
        self.clog_sensitivity = sensitivity
        self.clog_settings = CLOG_SENSITIVITY_LEVELS[self.clog_sensitivity]

        # Configurable detection thresholds and timing parameters with validation
        self.stuck_spool_load_grace = config.getfloat("stuck_spool_load_grace", STUCK_SPOOL_LOAD_GRACE, minval=0.0, maxval=60.0)
        self.stuck_spool_pressure_threshold = config.getfloat("stuck_spool_pressure_threshold", STUCK_SPOOL_PRESSURE_THRESHOLD, minval=0.0, maxval=1.0)
        self.stuck_spool_pressure_clear_threshold = config.getfloat("stuck_spool_pressure_clear_threshold", STUCK_SPOOL_PRESSURE_CLEAR_THRESHOLD, minval=0.0, maxval=1.0)
        self.clog_pressure_target = config.getfloat("clog_pressure_target", CLOG_PRESSURE_TARGET, minval=0.0, maxval=1.0)
        self.post_load_pressure_dwell = config.getfloat("post_load_pressure_dwell", POST_LOAD_PRESSURE_DWELL, minval=0.0, maxval=60.0)
        self.load_fps_stuck_threshold = config.getfloat("load_fps_stuck_threshold", LOAD_FPS_STUCK_THRESHOLD, minval=0.0, maxval=1.0)

        # Validate hysteresis: clear threshold must be > trigger threshold
        if self.stuck_spool_pressure_clear_threshold <= self.stuck_spool_pressure_threshold:
            raise config.error(
                f"stuck_spool_pressure_clear_threshold ({self.stuck_spool_pressure_clear_threshold}) "
                f"must be greater than stuck_spool_pressure_threshold ({self.stuck_spool_pressure_threshold})"
            )

        self._lane_unit_map: Dict[str, str] = {}
        self._lane_by_location: Dict[Tuple[str, int], str] = {}
        self._lane_to_fps_cache: Dict[str, str] = {}  # OPTIMIZATION: Lane?FPS direct mapping cache

        # OPTIMIZATION: Cache hardware service lookups
        self._hardware_service_cache: Dict[str, Any] = {}
        self._idle_timeout_obj = None
        self._gcode_obj = None
        self._toolhead_obj = None

        self._initialize_oams()

        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.printer.register_event_handler("idle_timeout:printing", self._handle_printing_resumed)
        self.printer.register_event_handler("pause:resume", self._handle_printing_resumed)

        self.printer.add_object("oams_manager", self)
        self.register_commands()

    def get_status(self, eventtime: float) -> Dict[str, Dict[str, Any]]:
        """Return current status of all FPS units and OAMS hardware."""
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
                self.logger.error("Failed to fetch status from %s", name)
                oam_status = {"action_status": "error", "action_status_code": None, "action_status_value": None}
            attributes["oams"][status_name] = oam_status
            if status_name != name:
                attributes["oams"][name] = oam_status

        state_names = {0: "UNLOADED", 1: "LOADED", 2: "LOADING", 3: "UNLOADING"}
        for fps_name, fps_state in self.current_state.fps_state.items():
            attributes[fps_name] = {
                "current_lane": fps_state.current_lane,
                "current_oams": fps_state.current_oams,
                "current_spool_idx": fps_state.current_spool_idx,
                "state_name": state_names.get(fps_state.state, str(fps_state.state)),
                "since": fps_state.since,
            }

        return attributes
    
    def determine_state(self) -> None:
        """Analyze hardware state and update FPS state tracking."""
        for fps_name, fps_state in self.current_state.fps_state.items():
            # Check if there's an active runout for this FPS
            monitor = self.runout_monitors.get(fps_name)
            is_runout_active = monitor and monitor.state != OAMSRunoutState.MONITORING
            was_loaded = (fps_state.state == FPSLoadState.LOADED)

            # Check F1S sensor state if we have OAMS and spool info
            # This helps detect runout conditions before monitor transitions state
            f1s_empty = False
            hub_has_filament = False
            if fps_state.current_oams and fps_state.current_spool_idx is not None:
                oams_obj = self.oams.get(fps_state.current_oams)
                if oams_obj:
                    try:
                        f1s_values = getattr(oams_obj, 'f1s_hes_value', None)
                        if f1s_values and fps_state.current_spool_idx < len(f1s_values):
                            f1s_empty = not bool(f1s_values[fps_state.current_spool_idx])

                        hub_values = getattr(oams_obj, 'hub_hes_value', None)
                        if hub_values and fps_state.current_spool_idx < len(hub_values):
                            hub_has_filament = bool(hub_values[fps_state.current_spool_idx])
                    except Exception:
                        pass

            # Query what AFC thinks is loaded (don't assign yet!)
            detected_lane, current_oams, detected_spool_idx = self.determine_current_loaded_lane(fps_name)

            if current_oams is not None and detected_spool_idx is not None:
                # Lane is detected as loaded - update state normally
                fps_state.current_lane = detected_lane
                fps_state.current_oams = current_oams.name
                fps_state.current_spool_idx = detected_spool_idx
                fps_state.state = FPSLoadState.LOADED
                fps_state.since = self.reactor.monotonic()
                fps_state.reset_stuck_spool_state()
                fps_state.reset_clog_tracker()
                self._ensure_forward_follower(fps_name, fps_state, "state detection")
            else:
                # AFC says no lane loaded (lane_loaded = None)
                # CRITICAL: If F1S just went empty but hub still has filament, keep state LOADED
                # This allows runout detection to trigger before we clear the lane info
                # Don't wait for monitor.state to change - it won't change until runout is detected
                if was_loaded and f1s_empty and hub_has_filament:
                    self.logger.info("State detection: Keeping %s as LOADED (F1S empty, hub has filament, runout about to be detected)",
                                   fps_name)
                    # Keep all existing state - runout detection needs this info
                    pass
                elif is_runout_active and was_loaded:
                    self.logger.info("State detection: Keeping %s as LOADED during active runout (state=%s, AFC lane_loaded cleared)",
                                   fps_name, monitor.state if monitor else "unknown")
                    # CRITICAL: Don't overwrite current_lane and current_spool_idx!
                    # Keep the existing values so runout detection can complete
                    # Only update current_oams if it was detected (might be None during runout)
                    if current_oams is not None:
                        fps_state.current_oams = current_oams.name
                else:
                    # No active runout and not about to detect one - transition to UNLOADED normally
                    # Save lane info before clearing for AFC notification
                    old_lane_name = fps_state.current_lane
                    old_oams_name = fps_state.current_oams
                    old_spool_idx = fps_state.current_spool_idx

                    fps_state.current_lane = None
                    fps_state.current_oams = None
                    fps_state.current_spool_idx = None
                    fps_state.state = FPSLoadState.UNLOADED
                    fps_state.reset_stuck_spool_state()
                    fps_state.reset_clog_tracker()
                    self._cancel_post_load_pressure_check(fps_state)

                    # Notify AFC that lane is unloaded to update virtual sensors (AMS_Extruder#)
                    # This ensures virtual sensors show correct state when lane becomes empty
                    if was_loaded and old_lane_name and AMSRunoutCoordinator is not None:
                        try:
                            AMSRunoutCoordinator.notify_lane_tool_state(
                                self.printer,
                                old_oams_name,
                                old_lane_name,
                                loaded=False,
                                spool_index=old_spool_idx,
                                eventtime=self.reactor.monotonic()
                            )
                            self.logger.debug("Notified AFC that %s unloaded during state detection (clears virtual sensor)", old_lane_name)
                        except Exception:
                            self.logger.error("Failed to notify AFC about %s unload during state detection", old_lane_name)
        
    def handle_ready(self) -> None:
        """Initialize system when printer is ready."""
        for fps_name, fps in self.printer.lookup_objects(module="fps"):
            self.fpss[fps_name] = fps
            self.current_state.add_fps_state(fps_name)

        if not self.fpss:
            raise ValueError("No FPS found in system, this is required for OAMS to work")

        # OPTIMIZATION: Cache frequently accessed objects
        try:
            self._idle_timeout_obj = self.printer.lookup_object("idle_timeout")
        except Exception:
            self._idle_timeout_obj = None

        try:
            self._gcode_obj = self.printer.lookup_object("gcode")
        except Exception:
            self._gcode_obj = None

        try:
            self._toolhead_obj = self.printer.lookup_object("toolhead")
        except Exception:
            self._toolhead_obj = None

        self.determine_state()
        self.start_monitors()
        self.ready = True

    def _initialize_oams(self) -> None:
        for name, oam in self.printer.lookup_objects(module="oams"):
            self.oams[name] = oam
        
    def determine_current_loaded_lane(self, fps_name: str) -> Tuple[Optional[str], Optional[object], Optional[int]]:
        """Determine which lane is currently loaded in the specified FPS."""
        fps = self.fpss.get(fps_name)
        if fps is None:
            raise ValueError(f"FPS {fps_name} not found")

        # Lane-based detection only
        return self._determine_loaded_lane_for_fps(fps_name, fps)

    def _determine_loaded_lane_for_fps(self, fps_name: str, fps) -> Tuple[Optional[str], Optional[object], Optional[int]]:
        """Determine which AFC lane is loaded by asking AFC which lane is loaded to each extruder.

        With load_to_hub: False configuration, filament bypasses OAMS hub sensors and goes
        directly to toolhead. AFC tracks filament position via its own sensors and is the
        authoritative source for what's loaded.
        """
        afc = self._get_afc()
        if afc is None:
            self.logger.warning("State detection: AFC not found")
            return None, None, None

        if not hasattr(afc, 'tools'):
            self.logger.warning("State detection: AFC has no 'tools' attribute")
            return None, None, None

        # Check each AFC tool/extruder to see which lane is loaded
        for extruder_name, extruder_obj in afc.tools.items():
            loaded_lane_name = getattr(extruder_obj, 'lane_loaded', None)
            if not loaded_lane_name:
                continue

            # Check if this lane is on the current FPS
            lane_fps = self.get_fps_for_afc_lane(loaded_lane_name)
            if lane_fps != fps_name:
                continue  # This lane is on a different FPS

            # Get the lane object
            lane = afc.lanes.get(loaded_lane_name)
            if lane is None:
                self.logger.warning("Lane %s not found in afc.lanes", loaded_lane_name)
                continue

            # Get the OAMS and bay index for this lane
            unit_str = getattr(lane, "unit", None)
            if not unit_str:
                self.logger.warning("Lane %s has no unit", loaded_lane_name)
                continue

            # Parse unit and slot
            if isinstance(unit_str, str) and ':' in unit_str:
                base_unit_name, slot_str = unit_str.split(':', 1)
                try:
                    slot_number = int(slot_str)
                except ValueError:
                    self.logger.warning("Invalid slot number in unit %s", unit_str)
                    continue
            else:
                base_unit_name = str(unit_str)
                slot_number = getattr(lane, "index", None)
                if slot_number is None:
                    self.logger.warning("No index found for lane %s", loaded_lane_name)
                    continue

            # Convert to bay index
            bay_index = slot_number - 1
            if bay_index < 0:
                self.logger.warning("Invalid bay index %d (slot %d)", bay_index, slot_number)
                continue

            # Get OAMS name from AFC unit
            unit_obj = getattr(lane, "unit_obj", None)
            if unit_obj is None:
                units = getattr(afc, "units", {})
                unit_obj = units.get(base_unit_name)
            if unit_obj is None:
                self.logger.warning("Unit %s not found", base_unit_name)
                continue

            oams_name = getattr(unit_obj, "oams_name", None)
            if not oams_name:
                self.logger.warning("Unit %s has no oams_name", base_unit_name)
                continue

            # Find OAMS object - check both short and prefixed names
            oam = self.oams.get(oams_name)
            if oam is None:
                oam = self.oams.get(f"oams {oams_name}")
            if oam is None:
                self.logger.warning("OAMS %s not found", oams_name)
                continue

            # Found loaded lane! Return lane name (e.g., "lane8") not map (e.g., "T4")
            # Map can be retrieved from lane object if needed for display
            self.logger.info("Detected %s loaded to %s (bay %d on %s)",
                           loaded_lane_name, extruder_name, bay_index, oams_name)
            return loaded_lane_name, oam, bay_index

        return None, None, None
        
    def register_commands(self):
        gcode = self.printer.lookup_object("gcode")
        commands = [
            ("OAMSM_UNLOAD_FILAMENT", self.cmd_UNLOAD_FILAMENT, self.cmd_UNLOAD_FILAMENT_help),
            ("OAMSM_LOAD_FILAMENT", self.cmd_LOAD_FILAMENT, self.cmd_LOAD_FILAMENT_help),
            ("OAMSM_FOLLOWER", self.cmd_FOLLOWER, self.cmd_FOLLOWER_help),
            ("OAMSM_FOLLOWER_RESET", self.cmd_FOLLOWER_RESET, self.cmd_FOLLOWER_RESET_help),
            ("OAMSM_CLEAR_ERRORS", self.cmd_CLEAR_ERRORS, self.cmd_CLEAR_ERRORS_help),
            ("OAMSM_CLEAR_LANE_MAPPINGS", self.cmd_CLEAR_LANE_MAPPINGS, self.cmd_CLEAR_LANE_MAPPINGS_help),
            ("OAMSM_STATUS", self.cmd_STATUS, self.cmd_STATUS_help),
        ]
        for cmd_name, handler, help_text in commands:
            gcode.register_command(cmd_name, handler, desc=help_text)

    cmd_CLEAR_ERRORS_help = "Clear the error state of the OAMS"
    def cmd_CLEAR_ERRORS(self, gcmd):
        with self._monitors_suspended("OAMSM_CLEAR_ERRORS"):
            # Reset all runout monitors to clear COASTING and other states
            for fps_name, monitor in list(self.runout_monitors.items()):
                try:
                    monitor.reset()
                except Exception:
                    self.logger.error("Failed to reset runout monitor for %s", fps_name)

            # Clear all FPS state error flags and tracking
            for fps_name, fps_state in self.current_state.fps_state.items():
                fps_state.clear_encoder_samples()
                fps_state.reset_stuck_spool_state()
                fps_state.reset_clog_tracker()
                fps_state.afc_delegation_active = False
                fps_state.afc_delegation_until = 0.0
                self._cancel_post_load_pressure_check(fps_state)

            # Clear OAMS hardware errors (this also clears all LED errors)
            # Do this in a single pass to minimize MCU commands
            for oam in self.oams.values():
                try:
                    oam.clear_errors()
                except Exception:
                    self.logger.error("Failed to clear errors on %s", getattr(oam, "name", "<unknown>"))

            # Preserve lane mappings during error clearing to avoid losing lane state.
            # We still surface any active redirects so an operator can clear them explicitly
            # with OAMSM_CLEAR_LANE_MAPPINGS if desired.
            try:
                afc = self.printer.lookup_object('AFC')
                if afc and hasattr(afc, 'lanes'):
                    retained_maps = []
                    for lane_name, lane in afc.lanes.items():
                        if hasattr(lane, 'map') and lane.map is not None:
                            if isinstance(lane.map, str) and lane.map.startswith('lane') and lane.map != lane_name:
                                retained_maps.append(f"{lane_name}->{lane.map}")
                    if retained_maps:
                        self.logger.info(
                            "Retaining OAMS lane mappings during OAMSM_CLEAR_ERRORS: %s. "
                            "Use OAMSM_CLEAR_LANE_MAPPINGS to clear these redirects if needed.",
                            ", ".join(retained_maps),
                        )
            except Exception:
                self.logger.debug("Could not inspect lane mappings (AFC not available or no mappings set)")

            # Re-detect state from hardware sensors
            try:
                self.determine_state()
            except Exception:
                self.logger.exception("State detection failed during OAMSM_CLEAR_ERRORS")

            # Clear all manual follower overrides and coast state - return to automatic hub sensor control
            # Also clear last state tracking so follower state is refreshed from actual sensors
            for oams_name in self.oams.keys():
                self.follower_manual_override[oams_name] = False
                self.follower_last_state[oams_name] = None  # Force state refresh
                self.follower_coasting[oams_name] = False
                self.follower_coast_start_pos[oams_name] = 0.0
                self.follower_had_filament[oams_name] = False

            # Clear LED state tracking so LEDs are refreshed from actual state
            self.led_error_state.clear()

            self.logger.info(
                "Cleared all manual follower overrides, coast state, LED state, and state tracking - returning to automatic control"
            )

            # After clearing errors and detecting state, ensure followers are enabled for any
            # lanes that have filament loaded to the hub (even if not loaded to toolhead)
            # This keeps filament pressure up for manual operations during troubleshooting
            self._ensure_followers_for_loaded_hubs()

        gcmd.respond_info("OAMS errors cleared and system re-initialized")

    cmd_CLEAR_LANE_MAPPINGS_help = "Clear OAMS cross-extruder lane mappings (call after RESET_AFC_MAPPING)"
    def cmd_CLEAR_LANE_MAPPINGS(self, gcmd):
        """Clear any lane.map redirects created by OAMS cross-extruder runouts.

        This is automatically called by OAMSM_CLEAR_ERRORS, but can also be called
        manually after RESET_AFC_MAPPING to ensure lane mappings are fully reset.

        Useful to add to your PRINT_END or CANCEL_PRINT macros after RESET_AFC_MAPPING:

        Example:
            RESET_AFC_MAPPING
            OAMSM_CLEAR_LANE_MAPPINGS
        """
        cleared_count = 0
        try:
            afc = self.printer.lookup_object('AFC')
            if afc and hasattr(afc, 'lanes'):
                for lane_name, lane in afc.lanes.items():
                    if hasattr(lane, 'map') and lane.map is not None:
                        # Check if map points to another lane (cross-extruder redirect)
                        if isinstance(lane.map, str) and lane.map.startswith('lane') and lane.map != lane_name:
                            self.logger.info("Clearing OAMS lane mapping: %s -> %s", lane_name, lane.map)
                            gcmd.respond_info(f"Clearing OAMS lane mapping: {lane_name} -> {lane.map}")
                            lane.map = None
                            cleared_count += 1
        except Exception:
            gcmd.respond_info("Could not clear lane mappings (AFC not available)")
            return

        if cleared_count > 0:
            gcmd.respond_info(f"Cleared {cleared_count} OAMS lane mapping(s)")
        else:
            gcmd.respond_info("No OAMS lane mappings to clear")

    cmd_STATUS_help = "Show OAMS manager state and run state detection diagnostics"
    def cmd_STATUS(self, gcmd):
        """Diagnostic command to show current state and test state detection."""
        afc = self._get_afc()

        gcmd.respond_info("=== OAMS Manager Status ===")

        # Show AFC info
        if afc is None:
            gcmd.respond_info("AFC: Not found!")
        else:
            gcmd.respond_info(f"AFC: Found, has {len(getattr(afc, 'tools', {}))} tools, {len(getattr(afc, 'lanes', {}))} lanes")

            # Show what AFC thinks is loaded
            if hasattr(afc, 'tools'):
                for tool_name, tool_obj in afc.tools.items():
                    lane_loaded = getattr(tool_obj, 'lane_loaded', None)
                    gcmd.respond_info(f"  Tool {tool_name}: lane_loaded={lane_loaded}")

        # Show FPS states
        for fps_name, fps_state in self.current_state.fps_state.items():
            gcmd.respond_info(f"\n{fps_name}:")
            gcmd.respond_info(f"  State: {fps_state.state}")
            gcmd.respond_info(f"  Current OAMS: {fps_state.current_oams}")
            gcmd.respond_info(f"  Current Lane: {fps_state.current_lane}")
            gcmd.respond_info(f"  Spool Index: {fps_state.current_spool_idx}")
            gcmd.respond_info(f"  Following: {fps_state.following}, Direction: {fps_state.direction}")

        # Run state detection and show results
        gcmd.respond_info("\n=== Running State Detection ===")
        self.determine_state()

        # Show state after detection
        for fps_name, fps_state in self.current_state.fps_state.items():
            gcmd.respond_info(f"\nAfter detection - {fps_name}:")
            gcmd.respond_info(f"  State: {fps_state.state}")
            gcmd.respond_info(f"  Current OAMS: {fps_state.current_oams}")
            gcmd.respond_info(f"  Current Lane: {fps_state.current_lane}")
            gcmd.respond_info(f"  Spool Index: {fps_state.current_spool_idx}")

        gcmd.respond_info("\nCheck klippy.log for detailed state detection logs")

    cmd_FOLLOWER_help = "Enable the follower on whatever OAMS is current loaded"
    def cmd_FOLLOWER(self, gcmd):
        enable = gcmd.get_int('ENABLE')
        # DIRECTION is optional when disabling (ENABLE=0), defaults to 0
        direction = gcmd.get_int('DIRECTION', 0)
        fps_name = "fps " + gcmd.get('FPS')

        if fps_name not in self.fpss:
            gcmd.respond_info(f"FPS {fps_name} does not exist")
            return

        fps_state = self.current_state.fps_state[fps_name]

        # Allow enabling follower when UNLOADED (before load starts) or LOADED
        # Only block during active LOADING/UNLOADING operations
        if fps_state.state in (FPSLoadState.LOADING, FPSLoadState.UNLOADING):
            gcmd.respond_info(f"FPS {fps_name} is currently busy")
            return

        # Prevent manual ENABLE during active error conditions (allow DISABLE for troubleshooting)
        if enable and (fps_state.clog_active or fps_state.stuck_spool_active):
            gcmd.respond_info(
                f"FPS {fps_name} has active error condition "
                f"(clog_active={fps_state.clog_active}, stuck_spool_active={fps_state.stuck_spool_active}). "
                f"Use OAMSM_CLEAR_ERRORS first to clear error state."
            )
            return

        # When disabling (ENABLE=0), disable follower and keep manual override
        # This prevents automatic control from re-enabling it
        if not enable:
            # If already unloaded or no OAMS, just mark as not following and return
            if not fps_state.current_oams:
                fps_state.following = False
                self.logger.info("Follower disable requested on %s but no OAMS loaded, marking as not following", fps_name)
                return

            oams_obj = self.oams.get(fps_state.current_oams)
            if oams_obj:
                try:
                    oams_obj.set_oams_follower(0, direction)
                    fps_state.following = False
                    # Update state tracker to avoid redundant commands
                    self.follower_last_state[fps_state.current_oams] = (0, direction)
                    # Keep manual override so it stays disabled (use OAMSM_FOLLOWER_RESET to return to automatic)
                    self.follower_manual_override[fps_state.current_oams] = True
                    self.logger.info("Disabled follower on %s (manual override - use OAMSM_FOLLOWER_RESET to return to automatic)", fps_name)
                    gcmd.respond_info(f"Follower disabled on {fps_name} (manual control - use OAMSM_FOLLOWER_RESET to return to automatic)")
                except Exception:
                    self.logger.error("Failed to disable follower on %s", fps_state.current_oams)
                    gcmd.respond_info(f"Failed to disable follower. Check logs.")
            else:
                # OAMS not found but mark as not following anyway
                fps_state.following = False
                self.logger.info("Follower disable: OAMS %s not found, marking as not following", fps_state.current_oams)
            return

        # When enabling, we need a valid OAMS
        oams_obj = self.oams.get(fps_state.current_oams)
        if oams_obj is None:
            gcmd.respond_info(f"OAMS {fps_state.current_oams} is not available")
            return

        try:
            self.logger.info("OAMSM_FOLLOWER: enabling follower on %s, direction=%d (manual override - will stay enabled regardless of hub sensors)", fps_name, direction)
            oams_obj.set_oams_follower(enable, direction)
            fps_state.following = bool(enable)
            fps_state.direction = direction
            # Update state tracker to avoid redundant commands
            self.follower_last_state[fps_state.current_oams] = (enable, direction)
            # Set manual override flag - follower stays enabled even if hub sensors are empty
            self.follower_manual_override[fps_state.current_oams] = True
            self.logger.info("OAMSM_FOLLOWER: successfully enabled follower on %s (manual override active)", fps_name)
            gcmd.respond_info(f"Follower enabled on {fps_name} (manual control - use OAMSM_FOLLOWER_RESET to return to automatic)")
        except Exception:
            self.logger.error("Failed to set follower on %s", fps_state.current_oams)
            gcmd.respond_info(f"Failed to set follower. Check logs.")

    cmd_FOLLOWER_RESET_help = "Return follower to automatic control based on hub sensors"
    def cmd_FOLLOWER_RESET(self, gcmd):
        fps_name = "fps " + gcmd.get('FPS')

        if fps_name not in self.fpss:
            gcmd.respond_info(f"FPS {fps_name} does not exist")
            return

        fps_state = self.current_state.fps_state[fps_name]

        if not fps_state.current_oams:
            gcmd.respond_info(f"No OAMS associated with {fps_name}")
            return

        # Clear manual override flag - return to automatic hub sensor control
        self.follower_manual_override[fps_state.current_oams] = False
        self.logger.info("Cleared manual follower override for %s, returning to automatic control", fps_name)
        gcmd.respond_info(f"Follower on {fps_name} returned to automatic control (hub sensor based)")

        # Immediately update follower based on current hub sensor state
        oams_obj = self.oams.get(fps_state.current_oams)
        if oams_obj:
            self._update_follower_for_oams(fps_state.current_oams, oams_obj)
            gcmd.respond_info(f"Follower state updated based on current hub sensors")

    def get_fps_for_afc_lane(self, lane_name: str) -> Optional[str]:
        """Get the FPS name for an AFC lane by querying its unit configuration.

        Returns the FPS name (e.g., "fps fps1") or None if not found.
        Uses cached mapping when available for performance.
        """
        # OPTIMIZATION: Check cache first
        cached = self._lane_to_fps_cache.get(lane_name)
        if cached is not None:
            return cached

        # Cache miss - compute and cache the result
        fps_name = self._compute_fps_for_afc_lane(lane_name)
        if fps_name is not None:
            self._lane_to_fps_cache[lane_name] = fps_name
        return fps_name

    def _compute_fps_for_afc_lane(self, lane_name: str) -> Optional[str]:
        """Compute the FPS name for an AFC lane (internal helper for caching)."""
        afc = self._get_afc()
        if afc is None:
            return None

        lane = afc.lanes.get(lane_name)
        if lane is None:
            return None

        # Get the unit string (e.g., "AMS_1:1")
        unit_str = getattr(lane, "unit", None)
        if not unit_str or not isinstance(unit_str, str):
            return None

        # Extract base unit name (e.g., "AMS_1" from "AMS_1:1")
        if ':' in unit_str:
            base_unit_name = unit_str.split(':')[0]
        else:
            base_unit_name = unit_str

        # Look up the AFC unit object
        unit_obj = getattr(lane, "unit_obj", None)
        if unit_obj is None:
            units = getattr(afc, "units", {})
            unit_obj = units.get(base_unit_name)

        if unit_obj is None:
            return None

        # Get the OAMS name from the unit (e.g., "oams1")
        oams_name = getattr(unit_obj, "oams_name", None)
        if not oams_name:
            return None

        # Find which FPS has this OAMS
        for fps_name, fps in self.fpss.items():
            if hasattr(fps, "oams"):
                fps_oams = fps.oams
                # fps.oams could be a list or a single oams object
                if isinstance(fps_oams, list):
                    for oam in fps_oams:
                        oam_name_full = getattr(oam, "name", None)
                        # OAMS objects can be registered as "oams1", "oams oams1", or "OAMS oams1"
                        if (oam_name_full == oams_name or
                            oam_name_full == f"oams {oams_name}" or
                            oam_name_full == f"OAMS {oams_name}"):
                            return fps_name
                else:
                    oam_name_check = getattr(fps_oams, "name", None)
                    if (oam_name_check == oams_name or
                        oam_name_check == f"oams {oams_name}" or
                        oam_name_check == f"OAMS {oams_name}"):
                        return fps_name

        return None


    def _rebuild_lane_location_index(self) -> None:
        """No longer needed - using lane-based detection only."""
        pass

    def _validate_afc_oams_integration(self, afc) -> None:
        """Validate AFC lane configs match OAMS hardware configuration.

        Checks for common integration issues:
        - Lanes without unit definitions
        - Lanes referencing non-existent AFC units
        - Units without OAMS names
        - OAMS names that don't exist in OAMS manager
        - Lanes that can't be mapped to any FPS
        """
        lanes = getattr(afc, "lanes", {})
        if not lanes:
            return

        issues = []
        valid_lanes = 0

        for lane_name, lane in lanes.items():
            # Check lane has valid unit
            unit_str = getattr(lane, "unit", None)
            if not unit_str:
                issues.append(f"Lane {lane_name} has no unit defined")
                continue

            # Parse unit string to get base unit name
            if isinstance(unit_str, str) and ':' in unit_str:
                base_unit_name = unit_str.split(':')[0]
            else:
                base_unit_name = str(unit_str)

            # Check unit exists in AFC
            unit_obj = getattr(lane, "unit_obj", None)
            if unit_obj is None:
                units = getattr(afc, "units", {})
                unit_obj = units.get(base_unit_name)

            if unit_obj is None:
                issues.append(f"Lane {lane_name} references non-existent AFC unit '{base_unit_name}'")
                continue

            # Check unit has OAMS name
            oams_name = getattr(unit_obj, "oams_name", None)
            if not oams_name:
                issues.append(f"AFC unit {base_unit_name} has no oams_name defined")
                continue

            # Check OAMS exists in OAMS manager
            oam = self.oams.get(oams_name)
            if oam is None:
                oam = self.oams.get(f"oams {oams_name}")
            if oam is None:
                oam = self.oams.get(f"OAMS {oams_name}")
            if oam is None:
                issues.append(f"Lane {lane_name} references OAMS '{oams_name}' which doesn't exist in OAMS manager")
                continue

            # Check FPS can be found (use cache since we just built it)
            fps_name = self._lane_to_fps_cache.get(lane_name)
            if not fps_name:
                issues.append(f"Lane {lane_name} cannot be mapped to any FPS (OAMS {oams_name} not found in any FPS config)")
                continue

            valid_lanes += 1

        # Log results
        if issues:
            self.logger.warning("AFC-OAMS integration validation found %d issue(s):", len(issues))
            for issue in issues:
                self.logger.warning("  - %s", issue)
            if valid_lanes > 0:
                self.logger.info("AFC-OAMS integration: %d lanes configured correctly", valid_lanes)
        else:
            self.logger.info("AFC-OAMS integration validated: %d lanes configured correctly", valid_lanes)

    def _ensure_afc_lane_cache(self, afc) -> None:
        """Build caches for AFC lane metadata and mappings."""
        lanes = getattr(afc, "lanes", {})
        cache_built = False

        for lane_name, lane in lanes.items():
            # Cache unit mapping
            unit_name = getattr(lane, "unit", None)
            if unit_name and lane_name not in self._lane_unit_map:
                self._lane_unit_map[lane_name] = unit_name

            # OPTIMIZATION: Pre-populate lane?FPS cache
            if lane_name not in self._lane_to_fps_cache:
                fps_name = self._compute_fps_for_afc_lane(lane_name)
                if fps_name is not None:
                    self._lane_to_fps_cache[lane_name] = fps_name
                    cache_built = True

        # Validate AFC-OAMS integration after building cache
        if cache_built and not self._afc_logged:
            self._validate_afc_oams_integration(afc)

    def _resolve_afc_lane_name(self, afc, identifier: Optional[str]) -> Optional[str]:
        """Resolve lane identifiers or map aliases (e.g., T#) to AFC lane names."""
        if not identifier:
            return None

        lanes = getattr(afc, "lanes", {})
        lookup = identifier.strip()
        if lookup in lanes:
            return lookup

        lowered = lookup.lower()
        for lane_name, lane in lanes.items():
            if lane_name.lower() == lowered:
                return lane_name

            lane_map = getattr(lane, "map", None)
            if not isinstance(lane_map, str):
                continue

            if lane_map == lookup or lane_map.lower() == lowered:
                return lane_name

            normalized_map = lane_map.replace(" ", "").lower()
            normalized_lookup = lookup.replace(" ", "").lower()
            if normalized_map == normalized_lookup:
                return lane_name

        return None

    def _resolve_lane_for_state(self, fps_state: 'FPSState', lane_name: Optional[str], afc) -> Tuple[Optional[str], Optional[str]]:
        """Resolve lane name from FPS state. Returns (lane_name, None) - group support removed."""
        # If lane_name provided, return it
        if lane_name:
            return lane_name, None

        # Try to get from current state (legacy location-based lookup)
        if fps_state.current_oams and fps_state.current_spool_idx is not None:
            located_lane = self._lane_by_location.get((fps_state.current_oams, fps_state.current_spool_idx))
            if located_lane:
                return located_lane, None

        return None, None

    def _get_afc(self):
        # OPTIMIZATION: Cache AFC object lookup with validation
        if self.afc is not None:
            # Validate cached object is still alive
            try:
                _ = self.afc.lanes  # Quick attribute access test
                return self.afc
            except Exception:
                self.logger.warning("Cached AFC object invalid, re-fetching")
                self.afc = None

        cached_afc = self._hardware_service_cache.get("afc_object")
        if cached_afc is not None:
            # Validate hardware service cache
            try:
                _ = cached_afc.lanes
                self.afc = cached_afc
                return self.afc
            except Exception:
                self.logger.warning("Cached AFC object in hardware service invalid, re-fetching")
                self._hardware_service_cache.pop("afc_object", None)

        try:
            afc = self.printer.lookup_object('AFC')
        except Exception:
            self.afc = None
            return None

        self.afc = afc
        self._hardware_service_cache["afc_object"] = afc
        self._ensure_afc_lane_cache(afc)
        if not self._afc_logged:
            self.logger.info("AFC integration detected; enabling same-FPS infinite runout support.")
            self._afc_logged = True
        return self.afc

    def _get_infinite_runout_target_lane(self, fps_name: str, fps_state: 'FPSState') -> Tuple[Optional[str], Optional[str], bool, Optional[str]]:
        """Get target lane for infinite runout.

        Returns: (target_lane_map, target_lane_name, delegate_to_afc, source_lane_name)
        - target_lane_map: Lane map attribute or lane name (for display)
        - target_lane_name: Actual target lane name to load
        - delegate_to_afc: True if AFC should handle (always True to preserve coasting/timing logic)
        - source_lane_name: Current source lane name
        """
        current_lane = fps_state.current_lane
        if not current_lane:
            return None, None, False, None

        afc = self._get_afc()
        if afc is None:
            return None, None, False, None

        lane_name, _ = self._resolve_lane_for_state(fps_state, current_lane, afc)

        if not lane_name:
            return None, None, False, None

        lanes = getattr(afc, "lanes", {})
        lane = afc.lanes.get(lane_name)
        if lane is None:
            return None, None, False, lane_name

        raw_runout_lane = getattr(lane, "runout_lane", None)
        runout_lane_name = self._resolve_afc_lane_name(afc, raw_runout_lane)
        if not runout_lane_name:
            return None, None, False, lane_name

        if raw_runout_lane and runout_lane_name != raw_runout_lane:
            try:
                lane.runout_lane = runout_lane_name
                self.logger.debug("Normalized runout lane for %s from %s to %s", lane_name, raw_runout_lane, runout_lane_name)
            except Exception:
                self.logger.debug("Could not persist normalized runout lane %s on %s", runout_lane_name, lane_name)

        target_lane = afc.lanes.get(runout_lane_name)
        if target_lane is None:
            self.logger.warning("Runout lane %s for %s on %s is not available; deferring to AFC", runout_lane_name, lane_name, fps_name)
            return None, runout_lane_name, True, lane_name

        # Check if source and target lanes are on the same FPS/extruder
        # If SAME FPS: OpenAMS handles reload internally (coast, PTFE calc, load new spool)
        # If DIFFERENT FPS: AFC handles via CHANGE_TOOL (full runout, then switch tools)
        source_extruder = _normalize_extruder_name(getattr(lane.extruder_obj, "name", None) if hasattr(lane, "extruder_obj") else None)
        target_extruder = _normalize_extruder_name(getattr(target_lane.extruder_obj, "name", None) if hasattr(target_lane, "extruder_obj") else None)

        same_fps = bool(source_extruder and target_extruder and source_extruder == target_extruder)
        delegate_to_afc = not same_fps  # Only delegate if different FPS

        target_lane_map = getattr(target_lane, "map", runout_lane_name)

        if same_fps:
            self.logger.info("Infinite runout: %s -> %s on same FPS %s (OpenAMS handling internally)",
                           lane_name, runout_lane_name, fps_name)
        else:
            self.logger.info("Infinite runout: %s -> %s on different FPS (delegating to AFC for tool change)",
                           lane_name, runout_lane_name)

        return target_lane_map, runout_lane_name, delegate_to_afc, lane_name

    def _delegate_runout_to_afc(self, fps_name: str, fps_state: 'FPSState', source_lane_name: Optional[str], target_lane_name: Optional[str]) -> bool:
        afc = self._get_afc()
        if afc is None:
            return False

        if not source_lane_name:
            return False

        lane = afc.lanes.get(source_lane_name)
        if lane is None:
            self.logger.warning("AFC lane %s not found while delegating infinite runout for %s", source_lane_name, fps_name)
            return False

        raw_runout_target = getattr(lane, "runout_lane", None)
        resolved_request = target_lane_name or raw_runout_target
        runout_target = self._resolve_afc_lane_name(afc, raw_runout_target) or self._resolve_afc_lane_name(afc, target_lane_name)
        if not runout_target:
            self.logger.warning(
                "AFC lane %s has no runout target while delegating infinite runout for %s (raw=%s, requested=%s)",
                source_lane_name,
                fps_name,
                raw_runout_target,
                resolved_request,
            )
            return False

        if raw_runout_target and runout_target != raw_runout_target:
            try:
                lane.runout_lane = runout_target
                self.logger.info("Delegating runout for %s using normalized target %s (from %s)", source_lane_name, runout_target, raw_runout_target)
            except Exception:
                self.logger.debug("Failed to persist normalized runout target %s on %s", runout_target, source_lane_name)
        elif not raw_runout_target:
            try:
                lane.runout_lane = runout_target
                self.logger.debug("Delegating runout for %s using resolved target %s (no raw target set)", source_lane_name, runout_target)
            except Exception:
                self.logger.debug("Failed to persist resolved runout target %s on %s", runout_target, source_lane_name)

        if getattr(lane, "runout_lane", None) != runout_target:
            self.logger.warning(
                "Resolved runout target %s could not be stored on lane %s (current=%s)",
                runout_target,
                source_lane_name,
                getattr(lane, "runout_lane", None),
            )
            return False

        now = self.reactor.monotonic()
        if fps_state.afc_delegation_active and now < fps_state.afc_delegation_until:
            return True

        if runout_target not in afc.lanes:
            self.logger.warning(
                "AFC runout lane %s referenced by %s is unavailable (requested=%s)",
                runout_target,
                source_lane_name,
                resolved_request,
            )
            return False

        try:
            current_lane = getattr(afc, "current", None)
            if current_lane != source_lane_name:
                if current_lane not in afc.lanes:
                    self.logger.debug(
                        "AFC current lane %s invalid during runout for %s; overriding to %s",
                        current_lane,
                        source_lane_name,
                        source_lane_name,
                    )
                try:
                    afc.current = source_lane_name
                except Exception:
                    self.logger.debug("Failed to set AFC current lane to %s before delegation", source_lane_name)

            self.logger.debug(
                "Delegating infinite runout via AFC: fps=%s source=%s target=%s (resolved_request=%s)",
                fps_name,
                source_lane_name,
                runout_target,
                resolved_request,
            )
            lane._perform_infinite_runout()
        except Exception:
            err_trace = traceback.format_exc()
            self.logger.error(
                "AFC infinite runout failed for lane %s -> %s: %s",
                source_lane_name,
                runout_target,
                err_trace,
            )
            fps_state.afc_delegation_active = False
            fps_state.afc_delegation_until = 0.0
            return False

        fps_state.afc_delegation_active = True
        fps_state.afc_delegation_until = now + AFC_DELEGATION_TIMEOUT
        self.logger.info("Delegated infinite runout for %s via AFC lane %s -> %s", fps_name, source_lane_name, runout_target)
        return True

    def _unload_filament_for_fps(self, fps_name: str) -> Tuple[bool, str]:
        if fps_name not in self.fpss:
            return False, f"FPS {fps_name} does not exist"

        fps_state = self.current_state.fps_state[fps_name]
        if fps_state.state == FPSLoadState.UNLOADED:
            return False, f"FPS {fps_name} is already unloaded"
        if fps_state.state in (FPSLoadState.LOADING, FPSLoadState.UNLOADING):
            return False, f"FPS {fps_name} is busy ({fps_state.state.name}), cannot unload"
        if fps_state.state != FPSLoadState.LOADED:
            return False, f"FPS {fps_name} is in unexpected state {fps_state.state.name}"

        if fps_state.current_oams is None:
            return False, f"FPS {fps_name} has no OAMS loaded"

        oams = self.oams.get(fps_state.current_oams)
        if oams is None:
            return False, f"OAMS {fps_state.current_oams} not found for FPS {fps_name}"

        if oams.current_spool is None:
            fps_state.state = FPSLoadState.UNLOADED
            fps_state.following = False
            fps_state.direction = 0
            fps_state.current_lane = None
            fps_state.current_spool_idx = None
            fps_state.since = self.reactor.monotonic()
            fps_state.reset_stuck_spool_state()
            fps_state.reset_clog_tracker()
            self._cancel_post_load_pressure_check(fps_state)
            return True, "Spool already unloaded"

        lane_name: Optional[str] = None
        spool_index = fps_state.current_spool_idx
        if AMSRunoutCoordinator is not None:
            try:
                afc = self._get_afc()
                if afc is not None:
                    lane_name, _ = self._resolve_lane_for_state(fps_state, fps_state.current_lane, afc)
            except Exception:
                self.logger.error("Failed to resolve AFC lane for unload on %s", fps_name)
                lane_name = None

        # Capture state BEFORE changing fps_state.state to avoid getting stuck
        try:
            encoder = oams.encoder_clicks
            current_time = self.reactor.monotonic()
            current_oams_name = oams.name
            current_spool = oams.current_spool
        except Exception:
            self.logger.error("Failed to capture unload state for %s", fps_name)
            return False, f"Failed to prepare unload on {fps_name}"

        # Only set state after all preliminary operations succeed
        fps_state.state = FPSLoadState.UNLOADING
        fps_state.encoder = encoder
        fps_state.since = current_time
        fps_state.current_oams = current_oams_name
        fps_state.current_spool_idx = current_spool
        fps_state.clear_encoder_samples()  # Clear stale encoder samples

        # Cancel post-load pressure check to prevent false positive clog detection during unload
        self._cancel_post_load_pressure_check(fps_state)

        try:
            success, message = oams.unload_spool_with_retry()
        except Exception:
            self.logger.error("Exception while unloading filament on %s", fps_name)
            # Reset state on exception to avoid getting stuck
            fps_state.state = FPSLoadState.LOADED
            return False, f"Exception unloading filament on {fps_name}"

        if success:
            fps_state.state = FPSLoadState.UNLOADED
            fps_state.following = False
            fps_state.direction = 0
            fps_state.since = self.reactor.monotonic()
            if lane_name:
                try:
                    AMSRunoutCoordinator.notify_lane_tool_state(self.printer, fps_state.current_oams or oams.name, lane_name, loaded=False, spool_index=spool_index, eventtime=fps_state.since)
                    # This triggers AFC's _apply_lane_sensor_state() which:
                    # - Handles shared prep/load lanes properly via _update_shared_lane()
                    # - Updates virtual sensor via _mirror_lane_to_virtual_sensor()
                except Exception:
                    self.logger.error("Failed to notify AFC that lane %s unloaded on %s", lane_name, fps_name)

            # Clear LED error state if stuck spool was active before resetting state
            if fps_state.stuck_spool_active and oams is not None and spool_index is not None:
                try:
                    oams.set_led_error(spool_index, 0)
                    self.logger.info("Cleared stuck spool LED for %s spool %d after successful unload", fps_name, spool_index)
                except Exception:
                    self.logger.error("Failed to clear LED on %s spool %d after successful unload", fps_name, spool_index)

            fps_state.current_lane = None
            fps_state.current_spool_idx = None
            fps_state.reset_stuck_spool_state()
            fps_state.reset_clog_tracker()
            self._cancel_post_load_pressure_check(fps_state)
            return True, message

        fps_state.state = FPSLoadState.LOADED
        return False, message

    def _clear_lane_on_runout(self, fps_name: str, fps_state: "FPSState", lane_name: Optional[str]) -> None:
        """
        Clear lane state from toolhead and OAMS when runout occurs without infinite spool target.

        This ensures proper cleanup when a lane runs out and has no runout_lane configured.
        Called after the runout sequence completes:
        1. F1S sensor detects empty spool
        2. 60mm PAUSE_DISTANCE allows hub to clear
        3. Follower motor stopped when entering COASTING (hub already cleared)
        4. PTFE amount coasts to toolhead
        5. This method is called to clear state

        Actions:
        - Clears FPS state in OAMS manager (sets UNLOADED, updates follower state flag)
        - Notifies AFC that lane is unloaded from toolhead
        - Resets stuck spool and clog tracking

        Note: Follower motor is already stopped by this point (stopped after hub cleared
        during COASTING transition). This method just updates the state flags.

        Args:
            fps_name: Name of the FPS (e.g., "fps fps1")
            fps_state: Current FPS state object
            lane_name: Name of the lane that ran out (e.g., "lane7")
        """
        if not lane_name:
            self.logger.warning("Cannot clear lane on runout - no lane name provided for %s", fps_name)
            return

        # Save spool index before clearing for AFC notification
        spool_index = fps_state.current_spool_idx
        oams_name = fps_state.current_oams

        # Clear FPS state (matching _unload_filament_for_fps and cross-extruder clear logic)
        fps_state.state = FPSLoadState.UNLOADED
        fps_state.following = False
        fps_state.direction = 0
        fps_state.since = self.reactor.monotonic()
        fps_state.current_lane = None
        fps_state.current_spool_idx = None
        fps_state.current_oams = None
        fps_state.reset_stuck_spool_state()
        fps_state.reset_clog_tracker()
        fps_state.reset_runout_positions()

        self.logger.info("Cleared FPS state for %s (was lane %s, spool %s)", fps_name, lane_name, spool_index)

        # Notify AFC that lane is unloaded from toolhead
        # This triggers AFC's _apply_lane_sensor_state() which:
        # - Handles shared prep/load lanes properly via _update_shared_lane()
        # - Updates virtual sensor via _mirror_lane_to_virtual_sensor()
        # - Calls lane.unit_obj.lane_unloaded() for proper cleanup
        if AMSRunoutCoordinator is not None and oams_name and lane_name:
            try:
                AMSRunoutCoordinator.notify_lane_tool_state(
                    self.printer,
                    oams_name,
                    lane_name,
                    loaded=False,
                    spool_index=spool_index,
                    eventtime=fps_state.since
                )
                self.logger.info("Notified AFC coordinator that lane %s unloaded from toolhead after runout", lane_name)
            except Exception:
                self.logger.error("Failed to notify AFC coordinator about lane %s unload after runout", lane_name)

    def _load_filament_for_lane(self, lane_name: str) -> Tuple[bool, str]:
        """Load filament for a lane by deriving OAMS and bay from the lane's unit configuration.

        This eliminates the need for [filament_group] configs by directly using:
        - lane.unit (e.g., "AMS_1:1") to get bay number
        - AFC_OpenAMS unit config to get OAMS name
        """
        afc = self._get_afc()
        if afc is None:
            return False, "AFC not available"

        lane = afc.lanes.get(lane_name)
        if lane is None:
            return False, f"Lane {lane_name} does not exist"

        # Get the unit string and slot/index
        # AFC stores "unit: AMS_1:1" as unit="AMS_1" and index stored separately
        unit_str = getattr(lane, "unit", None)
        if not unit_str:
            return False, f"Lane {lane_name} has no unit defined"

        # Try to get slot number from different possible attributes
        slot_number = None

        # Method 1: If unit_str contains ':', parse it directly (e.g., "AMS_1:1")
        if isinstance(unit_str, str) and ':' in unit_str:
            base_unit_name, slot_str = unit_str.split(':', 1)
            try:
                slot_number = int(slot_str)
            except ValueError:
                return False, f"Invalid slot number in unit {unit_str}"
        else:
            # Method 2: unit_str is just the unit name (e.g., "AMS_1"), get slot from lane.index
            base_unit_name = str(unit_str)
            slot_number = getattr(lane, "index", None)

            if slot_number is None:
                # Method 3: Try to get it from the lane's name if it follows a pattern
                # This is a fallback - lane names might not always have indices
                return False, f"Lane {lane_name} unit '{unit_str}' doesn't specify slot number"

        if slot_number is None:
            return False, f"Could not determine slot number for lane {lane_name}"

        # Convert slot number to 0-indexed bay number
        bay_index = slot_number - 1
        if bay_index < 0:
            return False, f"Invalid slot number {slot_number} (must be >= 1)"

        # Look up the AFC unit object to get OAMS name
        unit_obj = getattr(lane, "unit_obj", None)
        if unit_obj is None:
            units = getattr(afc, "units", {})
            unit_obj = units.get(base_unit_name)

        if unit_obj is None:
            return False, f"AFC unit {base_unit_name} not found"

        # Get the OAMS name from the unit (e.g., "oams1")
        oams_name = getattr(unit_obj, "oams_name", None)
        if not oams_name:
            return False, f"Unit {base_unit_name} has no oams_name defined"

        # Find the OAMS object
        # OAMS objects are stored with full name like "oams oams1", not just "oams1"
        oam = self.oams.get(oams_name)
        if oam is None:
            # Try with "oams " prefix
            oam = self.oams.get(f"oams {oams_name}")
        if oam is None:
            return False, f"OAMS {oams_name} not found"

        # Find which FPS has this OAMS
        fps_name = None
        for fps_name_candidate, fps in self.fpss.items():
            if hasattr(fps, "oams"):
                fps_oams = fps.oams
                if isinstance(fps_oams, list):
                    if oam in fps_oams:
                        fps_name = fps_name_candidate
                        break
                else:
                    if fps_oams == oam:
                        fps_name = fps_name_candidate
                        break

        if not fps_name:
            return False, f"No FPS found for OAMS {oams_name}"

        fps_state = self.current_state.fps_state[fps_name]

        # Synchronize with actual loaded lane before deciding how to handle the request
        detected_lane, detected_oams, detected_spool_idx = self.determine_current_loaded_lane(fps_name)
        if detected_lane is not None:
            fps_state.current_lane = detected_lane
            fps_state.current_oams = detected_oams.name if detected_oams else None
            fps_state.current_spool_idx = detected_spool_idx
            fps_state.state = FPSLoadState.LOADED
            fps_state.since = self.reactor.monotonic()
            if detected_lane == lane_name:
                return False, f"Lane {lane_name} is already loaded to {fps_name}"

            unload_success, unload_message = self._unload_filament_for_fps(fps_name)
            if not unload_success:
                return False, f"Failed to unload existing lane {detected_lane} from {fps_name}: {unload_message}"

        if fps_state.state == FPSLoadState.LOADED:
            return False, f"FPS {fps_name} is already loaded"

        self._cancel_post_load_pressure_check(fps_state)

        # Check if the bay is ready
        try:
            is_ready = oam.is_bay_ready(bay_index)
        except Exception:
            self.logger.error("Failed to check bay %s readiness on %s", bay_index, oams_name)
            return False, f"Failed to check bay {bay_index} readiness on {oams_name}"

        if not is_ready:
            return False, f"Bay {bay_index} on {oams_name} is not ready (no spool detected)"

        # Load the filament
        self.logger.info("Loading lane %s: %s bay %s via %s", lane_name, oams_name, bay_index, fps_name)

        # Capture state BEFORE changing fps_state.state to avoid getting stuck
        try:
            encoder = oam.encoder_clicks
            current_time = self.reactor.monotonic()
            oam_name = oam.name
        except Exception:
            self.logger.error("Failed to capture load state for lane %s bay %s", lane_name, bay_index)
            return False, f"Failed to capture load state for lane {lane_name}"

        # Only set state after all preliminary operations succeed
        fps_state.state = FPSLoadState.LOADING
        fps_state.encoder = encoder
        fps_state.current_oams = oam_name
        fps_state.current_spool_idx = bay_index
        # Set since to now for THIS load attempt (will be updated on success)
        fps_state.since = current_time
        fps_state.clear_encoder_samples()

        try:
            success, message = oam.load_spool_with_retry(bay_index)
        except Exception:
            self.logger.error("Failed to load bay %s on %s", bay_index, oams_name)
            fps_state.state = FPSLoadState.UNLOADED
            error_msg = f"Failed to load bay {bay_index} on {oams_name}"

            # CRITICAL: Pause printer if load fails during printing
            # This prevents printing without filament loaded
            self._pause_on_critical_failure(error_msg, oams_name)
            return False, error_msg

        if success:
            fps_state.current_lane = lane_name  # Store lane name (e.g., "lane8") not map (e.g., "T0")
            fps_state.current_oams = oam.name
            fps_state.current_spool_idx = bay_index

            # CRITICAL: Set fps_state.since to the successful load time BEFORE changing state
            successful_load_time = oam.get_last_successful_load_time(bay_index)
            if successful_load_time is not None:
                fps_state.since = successful_load_time
            else:
                fps_state.since = self.reactor.monotonic()

            # Now set state to LOADED after timestamp is correct
            fps_state.state = FPSLoadState.LOADED
            fps_state.direction = 1

            # OPTIMIZATION: Enable follower immediately before cleanup operations
            self._ensure_forward_follower(fps_name, fps_state, "load filament")

            # Clear LED error state if stuck spool was active
            if fps_state.stuck_spool_active:
                try:
                    oam.set_led_error(bay_index, 0)
                    self.logger.info("Cleared stuck spool LED for %s spool %d after successful load", fps_name, bay_index)
                except Exception:
                    self.logger.error("Failed to clear LED on %s spool %d after successful load", fps_name, bay_index)

            fps_state.reset_stuck_spool_state()
            fps_state.reset_clog_tracker()

            # Monitors are already running globally, no need to restart them
            return True, f"Loaded lane {lane_name} ({oam_name} bay {bay_index})"
        else:
            fps_state.state = FPSLoadState.UNLOADED
            error_msg = message if message else f"Failed to load lane {lane_name}"

            # CRITICAL: Pause printer if load fails during printing
            # This prevents printing without filament loaded, which would cause:
            # - No encoder movement
            # - FPS pressure at ~0.01
            # - Print failure and possible nozzle damage
            self._pause_on_critical_failure(error_msg, oams_name)
            return False, error_msg


    cmd_UNLOAD_FILAMENT_help = "Unload a spool from any of the OAMS if any is loaded"
    def cmd_UNLOAD_FILAMENT(self, gcmd):
        fps_name = "fps " + gcmd.get('FPS')
        if fps_name not in self.fpss:
            gcmd.respond_info(f"FPS {fps_name} does not exist")
            return
        fps_state = self.current_state.fps_state[fps_name]
        if fps_state.state == FPSLoadState.UNLOADED:
            gcmd.respond_info(f"FPS {fps_name} is already unloaded")
            return
        if fps_state.state in (FPSLoadState.LOADING, FPSLoadState.UNLOADING):
            gcmd.respond_info(f"FPS {fps_name} is currently busy")
            return

        success, message = self._unload_filament_for_fps(fps_name)
        if not success or (message and message != "Spool unloaded successfully"):
            gcmd.respond_info(message)

    cmd_LOAD_FILAMENT_help = "Load a spool from a specific AFC lane (LANE=name)"
    def cmd_LOAD_FILAMENT(self, gcmd):
        lane_name = gcmd.get('LANE', None)

        if not lane_name:
            gcmd.respond_info("LANE parameter is required (e.g., LANE=lane4)")
            return

        # Load directly from lane configuration
        success, message = self._load_filament_for_lane(lane_name)
        gcmd.respond_info(message)

    def _pause_on_critical_failure(self, error_message: str, oams_name: Optional[str] = None) -> None:
        """
        Pause the printer if a critical failure occurs during printing.

        This prevents catastrophic failures like:
        - Printing without filament loaded
        - Attempting to extrude with no material
        - Nozzle damage from dry printing

        The pause is scheduled asynchronously to avoid deadlocks when called
        from within gcode commands (like custom load macros).

        Only pauses if printer is actively printing. Manual operations during idle
        will just report the error without pausing.

        Args:
            error_message: Description of the failure
            oams_name: Name of the OAMS unit that encountered the error
        """
        # OPTIMIZATION: Use cached idle_timeout object
        idle_timeout = self._idle_timeout_obj
        if idle_timeout is None:
            try:
                idle_timeout = self.printer.lookup_object("idle_timeout")
                self._idle_timeout_obj = idle_timeout
            except Exception:
                self.logger.warning("Cannot check printer state for pause decision")
                # Schedule async pause to avoid deadlock
                self._schedule_async_pause(error_message, oams_name)
                return

        # Check if printer is currently printing
        try:
            eventtime = self.reactor.monotonic()
            status = idle_timeout.get_status(eventtime)
            is_printing = status.get("state") == "Printing"
        except Exception:
            self.logger.error("Failed to get printer state")
            # Schedule async pause to avoid deadlock
            self._schedule_async_pause(error_message, oams_name)
            return

        if is_printing:
            # Critical: printer is trying to print without filament loaded
            self.logger.error("CRITICAL FAILURE during printing: %s - PAUSING PRINTER", error_message)
            # Schedule pause asynchronously to avoid deadlock when called from gcode command
            self._schedule_async_pause(error_message, oams_name)
        else:
            # Not printing, just log the error - user can retry manually
            self.logger.warning("Load failed while idle: %s", error_message)

    def _schedule_async_pause(self, message: str, oams_name: Optional[str] = None) -> None:
        """
        Schedule a pause to happen asynchronously via reactor timer.

        This prevents deadlocks when pause is triggered from within a gcode command.
        The timer will fire after the current command completes, allowing the pause
        to execute in a clean context.
        """
        def _do_pause(eventtime):
            try:
                self._pause_printer_message(message, oams_name)
            except Exception:
                self.logger.error("Failed to execute async pause")
            return self.reactor.NEVER

        # Schedule pause to happen ASAP (0.05s delay to let current command finish)
        self.reactor.register_timer(_do_pause, self.reactor.monotonic() + 0.05)

    def _pause_printer_message(self, message, oams_name: Optional[str] = None):
        self.logger.info(message)

        if AMSRunoutCoordinator is not None and oams_name:
            try:
                AMSRunoutCoordinator.notify_afc_error(self.printer, oams_name, message, pause=False)
            except Exception:
                self.logger.error("Failed to forward OAMS pause message to AFC")

        # OPTIMIZATION: Use cached gcode object
        gcode = self._gcode_obj
        if gcode is None:
            try:
                gcode = self.printer.lookup_object("gcode")
                self._gcode_obj = gcode
            except Exception:
                self.logger.error("Failed to look up gcode object for pause message")
                return

        pause_message = f"Print has been paused: {message}"
        try:
            gcode.run_script(f"M118 {pause_message}")
            gcode.run_script(f"M114 {pause_message}")
        except Exception:
            self.logger.error("Failed to send pause notification gcode")

        # OPTIMIZATION: Use cached toolhead object
        toolhead = self._toolhead_obj
        if toolhead is None:
            try:
                toolhead = self.printer.lookup_object("toolhead")
                self._toolhead_obj = toolhead
            except Exception:
                self.logger.error("Failed to query toolhead state during pause handling")
                return

        try:
            homed_axes = toolhead.get_status(self.reactor.monotonic()).get("homed_axes", "")
        except Exception:
            self.logger.error("Failed to query toolhead state during pause handling")
            return

        try:
            state_message = self.printer.get_state_message()
        except Exception:
            state_message = None

        if isinstance(state_message, (list, tuple)) and state_message:
            printer_state_text = state_message[0]
        else:
            printer_state_text = state_message

        printer_state_text = printer_state_text if isinstance(printer_state_text, str) else None

        if printer_state_text:
            lowered_state = printer_state_text.lower()
            if "lost communication" in lowered_state or "mcu" in lowered_state:
                self.logger.warning(
                    "Printer reported an error state during pause handling: %s",
                    printer_state_text,
                )
                gcode.respond_info(
                    f"Pause notification may fail because printer reported: {printer_state_text}"
                )

        already_paused = False
        try:
            pause_resume = self.printer.lookup_object("pause_resume")
        except Exception:
            pause_resume = None

        if pause_resume is not None:
            try:
                already_paused = bool(getattr(pause_resume, "is_paused", False))
            except Exception:
                already_paused = False

        if already_paused:
            self.logger.debug("Skipping PAUSE command because printer is already paused")
            return

        if all(axis in homed_axes for axis in ("x", "y", "z")):
            pause_attempted = False
            pause_successful = False
            try:
                gcode.run_script("PAUSE")
                pause_attempted = True

                # Verify pause state after attempting to pause
                if pause_resume is not None:
                    try:
                        pause_successful = bool(getattr(pause_resume, "is_paused", False))
                    except Exception:
                        self.logger.error("Failed to verify pause state after PAUSE command")
            except Exception:
                self.logger.error("Failed to run PAUSE script")

            if pause_attempted and not pause_successful:
                self.logger.error(
                    "CRITICAL: Failed to pause printer for critical error: %s. "
                    "Print may continue despite error condition!",
                    message
                )
        else:
            self.logger.warning("Skipping PAUSE command because axes are not homed (homed_axes=%s)", homed_axes)

    def _cancel_post_load_pressure_check(self, fps_state: "FPSState") -> None:
        timer = getattr(fps_state, "post_load_pressure_timer", None)
        if timer is not None:
            try:
                self.reactor.unregister_timer(timer)
            except Exception:
                self.logger.error("Failed to cancel post-load pressure timer")
        fps_state.post_load_pressure_timer = None
        fps_state.post_load_pressure_start = None

    def _schedule_post_load_pressure_check(self, fps_name: str, fps_state: "FPSState") -> None:
        self._cancel_post_load_pressure_check(fps_state)

        def _monitor_pressure(self, eventtime):
            tracked_state = self.current_state.fps_state.get(fps_name)
            fps = self.fpss.get(fps_name)

            if tracked_state is None or fps is None:
                if tracked_state is not None:
                    self._cancel_post_load_pressure_check(tracked_state)
                return self.reactor.NEVER

            if tracked_state.state != FPSLoadState.LOADED:
                self._cancel_post_load_pressure_check(tracked_state)
                return self.reactor.NEVER

            pressure = float(getattr(fps, "fps_value", 0.0))
            if pressure <= POST_LOAD_PRESSURE_THRESHOLD:
                self._cancel_post_load_pressure_check(tracked_state)
                return self.reactor.NEVER

            now = self.reactor.monotonic()
            if tracked_state.post_load_pressure_start is None:
                tracked_state.post_load_pressure_start = now
                return eventtime + POST_LOAD_PRESSURE_CHECK_PERIOD

            if now - tracked_state.post_load_pressure_start < self.post_load_pressure_dwell:
                return eventtime + POST_LOAD_PRESSURE_CHECK_PERIOD

            oams_obj = None
            if tracked_state.current_oams is not None:
                oams_obj = self.oams.get(tracked_state.current_oams)
            if (oams_obj is not None and tracked_state.current_spool_idx is not None):
                try:
                    oams_obj.set_led_error(tracked_state.current_spool_idx, 1)
                except Exception:
                    self.logger.error("Failed to set clog LED on %s spool %s after loading", fps_name, tracked_state.current_spool_idx)

            message = f"Possible clog detected after loading {tracked_state.current_lane or fps_name}: FPS pressure {pressure:.2f} remained above {POST_LOAD_PRESSURE_THRESHOLD:.2f}"

            # Pause printer with error message
            self._pause_printer_message(message, tracked_state.current_oams)

            tracked_state.clog_active = True

            self._cancel_post_load_pressure_check(tracked_state)
            return self.reactor.NEVER

        timer = self.reactor.register_timer(partial(_monitor_pressure, self), self.reactor.NOW)
        fps_state.post_load_pressure_timer = timer
        fps_state.post_load_pressure_start = None

    def _enable_follower(self, fps_name: str, fps_state: "FPSState", oams: Optional[Any], direction: int, context: str) -> None:
        """
        Enable the OAMS follower motor to track filament movement.

        The follower motor maintains proper tension on the filament by following its movement
        through the buffer tube. This is essential for accurate encoder tracking and preventing
        filament tangles.

        Args:
            fps_name: Name of the FPS (Filament Pressure Sensor) being controlled
            fps_state: Current state object for the FPS
            oams: OAMS object controlling the hardware (can be None, will be looked up)
            direction: Follower direction (0=reverse, 1=forward)
            context: Description of why follower is being enabled (for logging)

        State Updates:
            - fps_state.following: Set to True on success
            - fps_state.direction: Updated to match requested direction

        Notes:
            - Direction defaults to forward (1) if invalid value provided
            - Fails silently if no spool is loaded (current_spool_idx is None)
            - Logs exceptions but doesn't raise them to avoid disrupting workflow
            - Uses state-tracked helper to avoid redundant MCU commands
        """
        if fps_state.current_spool_idx is None:
            return

        if oams is None and fps_state.current_oams is not None:
            oams = self.oams.get(fps_state.current_oams)
        if oams is None:
            self.logger.warning("Cannot enable follower: OAMS not found")
            return

        direction = direction if direction in (0, 1) else 1

        # Use state-tracked helper to avoid overwhelming MCU with redundant commands
        self._set_follower_if_changed(fps_state.current_oams, oams, 1, direction, context)

        # Update FPS state to reflect follower is now enabled
        # Note: _set_follower_if_changed updates follower_last_state tracking
        fps_state.following = True
        fps_state.direction = direction

    def _ensure_forward_follower(self, fps_name: str, fps_state: "FPSState", context: str) -> None:
        """Ensure follower is enabled in forward direction after successful load."""
        if (fps_state.current_oams is None or fps_state.current_spool_idx is None or
            fps_state.stuck_spool_active or fps_state.state != FPSLoadState.LOADED):
            return

        if fps_state.following and fps_state.direction == 1:
            return  # Already following in correct direction

        oams = self.oams.get(fps_state.current_oams)
        if oams is None:
            self.logger.warning("Cannot enable follower: OAMS %s not found", fps_state.current_oams)
            return

        fps_state.direction = 1
        self._enable_follower(fps_name, fps_state, oams, 1, context)

    def _set_led_error_if_changed(self, oams: Any, oams_name: str, spool_idx: int, error_state: int, context: str = "") -> None:
        """
        Send LED error command only if state has changed to avoid overwhelming MCU with redundant commands.

        Args:
            oams: OAMS object
            oams_name: Name of the OAMS
            spool_idx: Spool index (0-based)
            error_state: 0 to clear, 1 to set error
            context: Description for logging (optional)
        """
        led_key = f"{oams_name}:{spool_idx}"
        last_state = self.led_error_state.get(led_key, None)

        # Only send command if state changed or this is the first command
        if last_state != error_state:
            try:
                oams.set_led_error(spool_idx, error_state)
                self.led_error_state[led_key] = error_state
                if context and self.logger.isEnabledFor(logging.DEBUG):
                    self.logger.debug("LED error %s for %s spool %d (%s)", "set" if error_state else "cleared", oams_name, spool_idx, context)
            except Exception:
                self.logger.error("Failed to %s LED error for %s spool %d%s", "set" if error_state else "clear",
                                oams_name, spool_idx, f" ({context})" if context else "")

    def _set_follower_if_changed(self, oams_name: str, oams: Any, enable: int, direction: int, context: str = "") -> None:
        """
        Send follower command only if state has changed to avoid overwhelming MCU with redundant commands.

        Args:
            oams_name: Name of the OAMS
            oams: OAMS object
            enable: 1 to enable, 0 to disable
            direction: 0 for reverse, 1 for forward
            context: Description for logging (optional)
        """
        last_state = self.follower_last_state.get(oams_name, None)
        desired_state = (enable, direction)

        # Only send command if state changed or this is the first command
        if last_state != desired_state:
            try:
                oams.set_oams_follower(enable, direction)
                self.follower_last_state[oams_name] = desired_state
                if context:
                    self.logger.debug("Follower %s for %s (%s)", "enabled" if enable else "disabled", oams_name, context)
            except Exception:
                self.logger.error("Failed to %s follower for %s%s", "enable" if enable else "disable",
                                oams_name, f" ({context})" if context else "")

    def _update_follower_for_oams(self, oams_name: str, oams: Any) -> None:
        """
        Update follower state for an OAMS based on hub sensors with coast support.

        RULE: Follower enabled if ANY hub sensor shows filament, disabled after 50mm coast when ALL empty.
        When all hub sensors go empty, we coast 50mm to clear filament from shared PTFE before disabling.

        MANUAL OVERRIDE: If follower was manually enabled via OAMSM_FOLLOWER, skip automatic control.
        """
        try:
            # Skip automatic control if manually commanded
            if self.follower_manual_override.get(oams_name, False):
                self.logger.debug("Skipping automatic follower control for %s (manual override active)", oams_name)
                return

            hub_hes_values = getattr(oams, "hub_hes_value", None)
            if hub_hes_values is None:
                return

            # Check if ANY hub sensor shows filament
            any_filament = any(hub_hes_values)
            was_coasting = self.follower_coasting.get(oams_name, False)
            had_filament = self.follower_had_filament.get(oams_name, False)

            if any_filament:
                # At least one bay has filament - enable follower and clear coast state
                self._set_follower_if_changed(oams_name, oams, 1, 1, f"hub sensors: {hub_hes_values}")

                # Clear coast state
                if was_coasting:
                    self.logger.info("Follower coast cancelled for %s (filament detected)", oams_name)
                self.follower_coasting[oams_name] = False
                self.follower_coast_start_pos[oams_name] = 0.0
                self.follower_had_filament[oams_name] = True
            else:
                # All bays empty - start or continue coasting
                if not was_coasting and had_filament:
                    # Transition from "had filament" to "all empty" - start coast
                    # Find any FPS associated with this OAMS to get extruder position
                    extruder_pos = None
                    for fps_name, fps_state in self.current_state.fps_state.items():
                        if fps_state.current_oams == oams_name:
                            fps = self.fpss.get(fps_name)
                            if fps and hasattr(fps, 'extruder'):
                                extruder_pos = fps.extruder.last_position
                                break

                    if extruder_pos is not None:
                        self.follower_coasting[oams_name] = True
                        self.follower_coast_start_pos[oams_name] = extruder_pos
                        self.logger.info("Follower coast started for %s at position %.1f (all hub sensors empty, will coast %.1f mm)",
                                       oams_name, extruder_pos, self.follower_coast_distance)
                        # Keep follower enabled during coast
                        self._set_follower_if_changed(oams_name, oams, 1, 1, "starting coast")
                    else:
                        # Can't find extruder position, just disable immediately
                        self.logger.warning("Cannot determine extruder position for %s coast, disabling follower immediately", oams_name)
                        self._set_follower_if_changed(oams_name, oams, 0, 1, "no extruder position")
                        self.follower_had_filament[oams_name] = False

                elif was_coasting:
                    # Already coasting - check if we've coasted enough
                    # Find current extruder position
                    current_pos = None
                    for fps_name, fps_state in self.current_state.fps_state.items():
                        if fps_state.current_oams == oams_name:
                            fps = self.fpss.get(fps_name)
                            if fps and hasattr(fps, 'extruder'):
                                current_pos = fps.extruder.last_position
                                break

                    if current_pos is not None:
                        coast_start = self.follower_coast_start_pos.get(oams_name, 0.0)
                        coasted_distance = max(current_pos - coast_start, 0.0)

                        if coasted_distance >= self.follower_coast_distance:
                            # Coast complete - disable follower
                            self.logger.info("Follower coast complete for %s (coasted %.1f mm), disabling follower",
                                           oams_name, coasted_distance)
                            self._set_follower_if_changed(oams_name, oams, 0, 1, "coast complete")
                            self.follower_coasting[oams_name] = False
                            self.follower_coast_start_pos[oams_name] = 0.0
                            self.follower_had_filament[oams_name] = False
                        else:
                            # Still coasting - keep follower enabled
                            self.logger.debug("Follower coasting for %s: %.1f / %.1f mm",
                                            oams_name, coasted_distance, self.follower_coast_distance)
                            self._set_follower_if_changed(oams_name, oams, 1, 1, "still coasting")
                    else:
                        # Can't find position, just keep coasting
                        self._set_follower_if_changed(oams_name, oams, 1, 1, "coasting without position")
                else:
                    # Was already empty and not coasting - keep disabled
                    self._set_follower_if_changed(oams_name, oams, 0, 1, "all hub sensors empty")
                    self.follower_had_filament[oams_name] = False
        except Exception:
            self.logger.error("Failed to update follower for %s", oams_name)

    def _ensure_followers_for_loaded_hubs(self) -> None:
        """
        Ensure followers are enabled for any OAMS that has filament in the hub.
        Called after OAMSM_CLEAR_ERRORS and other state changes.
        Simple: just check hub sensors and enable/disable accordingly.
        """
        for oams_name, oams in self.oams.items():
            self._update_follower_for_oams(oams_name, oams)

    def _find_fps_for_oams_bay(self, oams_name: str, bay_idx: int) -> Optional[str]:
        """Find the FPS name that corresponds to a specific OAMS bay."""
        # Query AFC to find which lane uses this OAMS bay
        afc = self._get_afc()
        if afc is None or not hasattr(afc, 'lanes'):
            return None

        # Look through AFC lanes to find one that matches this OAMS and bay
        for lane_name, lane in afc.lanes.items():
            # Get lane's unit and slot
            unit_str = getattr(lane, "unit", None)
            if not unit_str:
                continue

            # Parse unit name and slot
            if isinstance(unit_str, str) and ':' in unit_str:
                base_unit_name, slot_str = unit_str.split(':', 1)
                try:
                    slot_number = int(slot_str)
                except ValueError:
                    continue
            else:
                base_unit_name = str(unit_str)
                slot_number = getattr(lane, "index", None)
                if slot_number is None:
                    continue

            # Convert slot to bay index
            lane_bay_idx = slot_number - 1
            if lane_bay_idx != bay_idx:
                continue

            # Check if this lane's unit uses this OAMS
            unit_obj = getattr(lane, "unit_obj", None)
            if unit_obj is None:
                units = getattr(afc, "units", {})
                unit_obj = units.get(base_unit_name)
            if unit_obj is None:
                continue

            lane_oams_name = getattr(unit_obj, "oams_name", None)
            if lane_oams_name != oams_name and f"oams {lane_oams_name}" != oams_name:
                continue

            # Found matching lane - return its FPS
            return self.get_fps_for_afc_lane(lane_name)

        return None

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
            self.logger.info("Restarted follower for %s spool %s after %s.", fps_name, fps_state.current_spool_idx, context)

    def _handle_printing_resumed(self, _eventtime):
        # Check if monitors were stopped and need to be restarted
        if not self.monitor_timers:
            self.logger.info("Restarting monitors after pause/intervention")
            self.start_monitors()

        # Clear any error LEDs on resume (error flags already cleared when pause was triggered)
        for fps_name, fps_state in self.current_state.fps_state.items():
            oams = self.oams.get(fps_state.current_oams) if fps_state.current_oams else None

            # Clear stuck_spool_active on resume to allow follower to restart
            if fps_state.stuck_spool_active:
                fps_state.reset_stuck_spool_state(preserve_restore=True)
                self.logger.info("Cleared stuck spool state for %s on print resume", fps_name)

            # Clear clog_active on resume and reset tracker
            if fps_state.clog_active:
                fps_state.reset_clog_tracker()
                self.logger.info("Cleared clog state for %s on print resume", fps_name)
                # Clear the error LED if we have an OAMS and spool index
                if oams is not None and fps_state.current_spool_idx is not None:
                    try:
                        oams.set_led_error(fps_state.current_spool_idx, 0)
                    except Exception:
                        self.logger.error("Failed to clear clog LED on %s after resume", fps_name)

            if fps_state.stuck_spool_restore_follower:
                self._restore_follower_if_needed(fps_name, fps_state, oams, "print resume")
            elif (
                fps_state.current_oams is not None
                and fps_state.current_spool_idx is not None
                and not fps_state.following
            ):
                self._ensure_forward_follower(fps_name, fps_state, "print resume")

        # Update all followers based on hub sensors
        # Simple: if hub has filament, enable follower; if all empty, disable
        self._ensure_followers_for_loaded_hubs()

    def _trigger_stuck_spool_pause(self, fps_name: str, fps_state: "FPSState", oams: Optional[Any], message: str) -> None:
        if fps_state.stuck_spool_active:
            return

        spool_idx = fps_state.current_spool_idx
        if oams is None and fps_state.current_oams is not None:
            oams = self.oams.get(fps_state.current_oams)

        # Set LED to red to indicate error
        if oams is not None and spool_idx is not None:
            try:
                oams.set_led_error(spool_idx, 1)
            except Exception:
                self.logger.error("Failed to set stuck spool LED on %s spool %s", fps_name, spool_idx)

        # Abort current action (unload/load)
        if oams is not None:
            try:
                oams.abort_current_action()
            except Exception:
                self.logger.error("Failed to abort active action for %s during stuck spool pause", fps_name)

        # Pause printer with error message
        self._pause_printer_message(message, fps_state.current_oams)

        # Clear error flag immediately after pausing - system is ready for user to fix
        # LED stays red to indicate the issue, but error flag doesn't block other operations
        fps_state.stuck_spool_active = False
        fps_state.stuck_spool_start_time = None
        self.logger.info("Stuck spool pause triggered for %s, error flag cleared (LED stays red)", fps_name)

    def _unified_monitor_for_fps(self, fps_name):
        """Consolidated monitor handling all FPS checks in a single timer (OPTIMIZED)."""
        def _unified_monitor(self, eventtime):
            fps_state = self.current_state.fps_state.get(fps_name)
            fps = self.fpss.get(fps_name)

            if fps_state is None or fps is None:
                return eventtime + MONITOR_ENCODER_PERIOD_IDLE

            oams = self.oams.get(fps_state.current_oams) if fps_state.current_oams else None

            # OPTIMIZATION: Use cached idle_timeout object
            is_printing = False
            if self._idle_timeout_obj is not None:
                try:
                    is_printing = self._idle_timeout_obj.get_status(eventtime)["state"] == "Printing"
                except Exception:
                    is_printing = False

            # OPTIMIZATION: Skip sensor reads if idle and no state changes
            state = fps_state.state
            if not is_printing and state == FPSLoadState.LOADED:
                fps_state.consecutive_idle_polls += 1
                if fps_state.consecutive_idle_polls > IDLE_POLL_THRESHOLD:
                    # Exponential backoff for idle polling
                    if fps_state.consecutive_idle_polls % 5 == 0:
                        fps_state.idle_backoff_level = min(fps_state.idle_backoff_level + 1, 3)
                    backoff_multiplier = 2 ** fps_state.idle_backoff_level
                    return eventtime + (MONITOR_ENCODER_PERIOD_IDLE * backoff_multiplier)

            # Read sensors
            try:
                if oams:
                    encoder_value = oams.encoder_clicks
                    pressure = float(getattr(fps, "fps_value", 0.0))
                    hes_values = oams.hub_hes_value
                else:
                    return eventtime + MONITOR_ENCODER_PERIOD_IDLE
            except Exception:
                self.logger.error("Failed to read sensors for %s", fps_name)
                return eventtime + MONITOR_ENCODER_PERIOD_IDLE

            now = self.reactor.monotonic()
            state_changed = False

            if state == FPSLoadState.UNLOADING and now - fps_state.since > MONITOR_ENCODER_SPEED_GRACE:
                self._check_unload_speed(fps_name, fps_state, oams, encoder_value, now)
                state_changed = True
            elif state == FPSLoadState.LOADING and now - fps_state.since > MONITOR_ENCODER_SPEED_GRACE:
                self._check_load_speed(fps_name, fps_state, fps, oams, encoder_value, pressure, now)
                state_changed = True
            elif state == FPSLoadState.UNLOADED:
                # When UNLOADED, periodically check if filament was newly inserted
                # AFC updates lane_loaded when filament is detected, so we need to check determine_state()
                # to pick up the new lane_loaded and update current_spool_idx
                # Skip auto-detect if there's an active runout in progress to avoid interference
                monitor = self.runout_monitors.get(fps_name)
                is_runout_active = monitor and monitor.state != OAMSRunoutState.MONITORING
                if not is_runout_active and fps_state.consecutive_idle_polls % 10 == 0:  # Check every 10 polls
                    old_lane = fps_state.current_lane
                    old_spool_idx = fps_state.current_spool_idx
                    (
                        fps_state.current_lane,
                        current_oams,
                        fps_state.current_spool_idx,
                    ) = self.determine_current_loaded_lane(fps_name)

                    if current_oams is not None:
                        fps_state.current_oams = current_oams.name

                    # If lane was newly detected, transition to LOADED
                    if fps_state.current_lane and fps_state.current_spool_idx is not None:
                        fps_state.state = FPSLoadState.LOADED
                        fps_state.since = now
                        fps_state.reset_stuck_spool_state()
                        fps_state.reset_clog_tracker()
                        self._ensure_forward_follower(fps_name, fps_state, "auto-detect new filament")
                        self.logger.info("Auto-detected newly inserted filament: %s (spool %s)",
                                       fps_state.current_lane, fps_state.current_spool_idx)
                        state_changed = True
            elif state == FPSLoadState.LOADED:
                if is_printing:
                    # Always call stuck spool check (it has its own clearing logic for runout states)
                    self._check_stuck_spool(fps_name, fps_state, fps, oams, pressure, hes_values, now)

                    # Skip clog detection during AMS runout DETECTED/COASTING states
                    # DETECTED: Old spool empty, no encoder movement expected
                    # COASTING: New filament traveling through buffer, may not move encoder immediately
                    monitor = self.runout_monitors.get(fps_name)
                    if monitor and monitor.state in (OAMSRunoutState.DETECTED, OAMSRunoutState.COASTING):
                        # Runout in progress - skip clog detection to prevent false positives
                        pass
                    else:
                        self._check_clog(fps_name, fps_state, fps, oams, encoder_value, pressure, now)

                    state_changed = True
            # Update follower based on hub sensors (simple: ANY filament = enabled, ALL empty = disabled)
            # This runs every monitor cycle to keep follower in sync with actual hub state
            if oams and fps_state.current_oams:
                self._update_follower_for_oams(fps_state.current_oams, oams)

            # OPTIMIZATION: Adaptive polling interval with exponential backoff
            if state_changed or is_printing:
                fps_state.consecutive_idle_polls = 0
                fps_state.idle_backoff_level = 0
                fps_state.last_state_change = now
                return eventtime + MONITOR_ENCODER_PERIOD

            fps_state.consecutive_idle_polls += 1
            if fps_state.consecutive_idle_polls > IDLE_POLL_THRESHOLD:
                # Exponential backoff: increase backoff level every 5 idle polls
                if fps_state.consecutive_idle_polls % 5 == 0:
                    fps_state.idle_backoff_level = min(fps_state.idle_backoff_level + 1, 3)

                # Calculate backoff multiplier (1x, 2x, 4x, 8x)
                backoff_multiplier = 2 ** fps_state.idle_backoff_level
                interval = MONITOR_ENCODER_PERIOD_IDLE * backoff_multiplier
                return eventtime + interval

            return eventtime + MONITOR_ENCODER_PERIOD

        return partial(_unified_monitor, self)

    def _check_unload_speed(self, fps_name, fps_state, oams, encoder_value, now):
        """Check unload speed using optimized encoder tracking."""
        # Skip check if already handling a stuck spool
        if fps_state.stuck_spool_active:
            return

        encoder_diff = fps_state.record_encoder_sample(encoder_value)
        if encoder_diff is None:
            return

        if self.logger.isEnabledFor(logging.DEBUG):
            self.logger.debug("OAMS[%d] Unload Monitor: Encoder diff %d", getattr(oams, "oams_idx", -1), encoder_diff)

        if encoder_diff < MIN_ENCODER_DIFF:
            # Check if we're actively printing (only set stuck flag during prints)
            is_printing = False
            if self._idle_timeout_obj is not None:
                try:
                    is_printing = self._idle_timeout_obj.get_status(now)["state"] == "Printing"
                except Exception:
                    is_printing = False

            group_label = fps_state.current_lane or fps_name
            spool_label = str(fps_state.current_spool_idx) if fps_state.current_spool_idx is not None else "unknown"

            # Abort the current unload operation cleanly
            try:
                oams.abort_current_action()
                self.logger.info("Aborted stuck spool unload operation on %s", fps_name)
            except Exception:
                self.logger.error("Failed to abort unload operation on %s", fps_name)

            # Set LED error using state-tracked helper
            if fps_state.current_spool_idx is not None:
                self._set_led_error_if_changed(oams, fps_state.current_oams, fps_state.current_spool_idx, 1, "unload stuck detected")

            # CRITICAL: Keep follower enabled even during stuck unload
            # User needs follower running to manually fix issues or re-attempt unload
            # Follower doesn't interfere with stuck detection (encoder based)
            if fps_state.current_oams:
                self._ensure_forward_follower(fps_name, fps_state, "stuck unload - keep follower active")

            # Only set stuck flag and transition to LOADED during active prints
            # For manual unloads (standby), let OAMS retry logic handle it without setting stuck flag
            if is_printing:
                # Transition to LOADED state cleanly (unload failed during print, so still loaded)
                fps_state.state = FPSLoadState.LOADED
                fps_state.clear_encoder_samples()

                # Set the stuck flag but DON'T pause - let the OAMS retry logic handle it
                # The retry logic will clear this flag if the retry succeeds
                fps_state.stuck_spool_active = True
                fps_state.stuck_spool_start_time = None

                self.logger.info("Spool appears stuck while unloading %s spool %s - letting retry logic handle it", group_label, spool_label)
            else:
                # During standby unload, don't set stuck flag - just let OAMS retry
                # State stays UNLOADING so retry logic can continue
                fps_state.clear_encoder_samples()
                self.logger.info("Spool unload slow/stuck on %s spool %s (standby) - OAMS retry will handle it", group_label, spool_label)

    def _check_load_speed(self, fps_name, fps_state, fps, oams, encoder_value, pressure, now):
        """Check load speed using optimized encoder tracking and FPS pressure monitoring."""
        if fps_state.stuck_spool_active:
            return

        # Skip check if we don't have a valid since timestamp
        if fps_state.since is None:
            return

        # Skip check if we're still in grace period
        if now - fps_state.since <= MONITOR_ENCODER_SPEED_GRACE:
            return

        encoder_diff = fps_state.record_encoder_sample(encoder_value)
        if encoder_diff is None:
            return

        if self.logger.isEnabledFor(logging.DEBUG):
            self.logger.debug("OAMS[%d] Load Monitor: Encoder diff %d, FPS pressure %.2f",
                            getattr(oams, "oams_idx", -1), encoder_diff, pressure)

        # Check for stuck spool conditions:
        # 1. Encoder not moving (original check)
        # 2. FPS pressure staying high (filament not engaging) - NEW CHECK
        stuck_detected = False
        stuck_reason = ""

        if encoder_diff < MIN_ENCODER_DIFF:
            stuck_detected = True
            stuck_reason = "encoder not moving"
        elif pressure >= self.load_fps_stuck_threshold:
            # FPS pressure is high while loading - filament isn't being pulled in
            # This catches cases where extruder is turning but filament missed the drive gear
            stuck_detected = True
            stuck_reason = f"FPS pressure {pressure:.2f} >= {self.load_fps_stuck_threshold:.2f} (filament not engaging)"

        if stuck_detected:
            group_label = fps_state.current_lane or fps_name
            spool_label = str(fps_state.current_spool_idx) if fps_state.current_spool_idx is not None else "unknown"

            # Abort the current load operation cleanly
            try:
                oams.abort_current_action()
                self.logger.info("Aborted stuck spool load operation on %s: %s", fps_name, stuck_reason)
            except Exception:
                self.logger.error("Failed to abort load operation on %s", fps_name)

            # Set LED error using state-tracked helper
            if fps_state.current_spool_idx is not None:
                self._set_led_error_if_changed(oams, fps_state.current_oams, fps_state.current_spool_idx, 1, "load stuck detected")

            # Transition to UNLOADED state cleanly
            fps_state.state = FPSLoadState.UNLOADED
            fps_state.clear_encoder_samples()

            # Set the stuck flag but DON'T pause - let the OAMS retry logic handle it
            # The retry logic will clear this flag if the retry succeeds
            fps_state.stuck_spool_active = True
            fps_state.stuck_spool_start_time = None

            # CRITICAL: Keep follower enabled even during stuck load
            # User needs follower running to manually fix clogs or re-attempt load
            # Follower doesn't interfere with stuck detection (encoder + FPS based)
            if fps_state.current_oams:
                self._ensure_forward_follower(fps_name, fps_state, "stuck load - keep follower active")

            self.logger.info("Spool appears stuck while loading %s spool %s (%s) - letting retry logic handle it",
                           group_label, spool_label, stuck_reason)

    def _check_stuck_spool(self, fps_name, fps_state, fps, oams, pressure, hes_values, now):
        """Check for stuck spool conditions (OPTIMIZED)."""
        # OPTIMIZATION: Use cached idle_timeout object
        is_printing = False
        if self._idle_timeout_obj is not None:
            try:
                is_printing = self._idle_timeout_obj.get_status(now)["state"] == "Printing"
            except Exception:
                is_printing = False

        monitor = self.runout_monitors.get(fps_name)
        if monitor is not None and monitor.state != OAMSRunoutState.MONITORING:
            if fps_state.stuck_spool_active and oams is not None and fps_state.current_spool_idx is not None:
                self._set_led_error_if_changed(oams, fps_state.current_oams, fps_state.current_spool_idx, 0, "runout monitor inactive")
            fps_state.reset_stuck_spool_state(preserve_restore=fps_state.stuck_spool_restore_follower)
            return

        if not is_printing:
            if fps_state.stuck_spool_active and oams is not None and fps_state.current_spool_idx is not None:
                self._set_led_error_if_changed(oams, fps_state.current_oams, fps_state.current_spool_idx, 0, "printer idle")
            fps_state.reset_stuck_spool_state(preserve_restore=fps_state.stuck_spool_restore_follower)
            return

        if fps_state.since is not None and now - fps_state.since < self.stuck_spool_load_grace:
            fps_state.stuck_spool_start_time = None
            # Clear stuck spool flag during grace period after successful load
            if fps_state.stuck_spool_active:
                if oams is not None and fps_state.current_spool_idx is not None:
                    self._set_led_error_if_changed(oams, fps_state.current_oams, fps_state.current_spool_idx, 0, "grace period")
                fps_state.reset_stuck_spool_state(preserve_restore=True)
            return

        if not fps_state.following or fps_state.direction != 1:
            fps_state.stuck_spool_start_time = None
            # Auto-enable follower if we have a spool loaded but follower is disabled
            if is_printing and oams is not None and not fps_state.following:
                self._ensure_forward_follower(fps_name, fps_state, "auto-enable after manual load")
            elif fps_state.stuck_spool_restore_follower and is_printing and oams is not None:
                self._restore_follower_if_needed(fps_name, fps_state, oams, "stuck spool recovery")
            return

        # Hysteresis logic: Use lower threshold to start timer, upper threshold to clear
        if pressure <= self.stuck_spool_pressure_threshold:
            # Pressure is low - start or continue stuck spool timer
            if fps_state.stuck_spool_start_time is None:
                fps_state.stuck_spool_start_time = now
            elif (not fps_state.stuck_spool_active and now - fps_state.stuck_spool_start_time >= STUCK_SPOOL_DWELL):
                message = "Spool appears stuck"
                if fps_state.current_lane is not None:
                    message = f"Spool appears stuck on {fps_state.current_lane} spool {fps_state.current_spool_idx}"
                self._trigger_stuck_spool_pause(fps_name, fps_state, oams, message)
        elif pressure >= self.stuck_spool_pressure_clear_threshold:
            # Pressure is definitively high - clear stuck spool state
            if fps_state.stuck_spool_active and oams is not None and fps_state.current_spool_idx is not None:
                self._set_led_error_if_changed(oams, fps_state.current_oams, fps_state.current_spool_idx, 0, "pressure restored")

                # Clear the stuck_spool_active flag BEFORE trying to restore follower
                fps_state.reset_stuck_spool_state(preserve_restore=True)
                self.logger.info("Cleared stuck spool state for %s, pressure restored to %.2f", fps_name, pressure)

            # Also clear timer if it was running but not yet triggered
            if fps_state.stuck_spool_start_time is not None and not fps_state.stuck_spool_active:
                fps_state.stuck_spool_start_time = None

            # Now restore/enable follower
            if fps_state.stuck_spool_restore_follower and is_printing:
                self._restore_follower_if_needed(fps_name, fps_state, oams, "stuck spool recovery")
            elif is_printing and not fps_state.following:
                self._ensure_forward_follower(fps_name, fps_state, "stuck spool recovery")
        # else: Pressure is in hysteresis band (between thresholds) - maintain current state

    def _check_clog(self, fps_name, fps_state, fps, oams, encoder_value, pressure, now):
        """Check for clog conditions (OPTIMIZED)."""
        # OPTIMIZATION: Use cached idle_timeout object
        is_printing = False
        if self._idle_timeout_obj is not None:
            try:
                is_printing = self._idle_timeout_obj.get_status(now)["state"] == "Printing"
            except Exception:
                is_printing = False

        monitor = self.runout_monitors.get(fps_name)
        if monitor is not None and monitor.state != OAMSRunoutState.MONITORING:
            if fps_state.clog_active and oams is not None and fps_state.current_spool_idx is not None:
                self._set_led_error_if_changed(oams, fps_state.current_oams, fps_state.current_spool_idx, 0, "runout monitor inactive")
            fps_state.reset_clog_tracker()
            return

        if not is_printing:
            if fps_state.clog_active and oams is not None and fps_state.current_spool_idx is not None:
                self._set_led_error_if_changed(oams, fps_state.current_oams, fps_state.current_spool_idx, 0, "printer idle")
            fps_state.reset_clog_tracker()
            return

        # Skip clog detection if FPS pressure is very low - indicates stuck spool, not clog
        # During lane loads, stuck spool should trigger retry logic, not clog pause
        # During normal printing, low pressure also indicates stuck spool (separate detection)
        if pressure <= self.stuck_spool_pressure_threshold:
            # Very low FPS pressure indicates stuck spool, not clog - skip clog detection
            fps_state.reset_clog_tracker()
            return

        # Clog detection now runs normally for all states (including TOOL_LOADING)
        # During load purge: extruder advances + encoder doesn't move = genuine clog, detect it
        # Before purge starts: extruder not advancing = clog won't trigger (extrusion_delta < threshold)
        # The existing clog logic is already smart enough to handle this correctly

        try:
            extruder_pos = float(getattr(fps.extruder, "last_position", 0.0))
        except Exception:
            self.logger.error("Failed to read extruder position while monitoring clogs on %s", fps_name)
            return

        if fps_state.clog_start_extruder is None:
            fps_state.prime_clog_tracker(extruder_pos, encoder_value, pressure, now)
            return

        if extruder_pos < (fps_state.clog_last_extruder or extruder_pos):
            fps_state.prime_clog_tracker(extruder_pos, encoder_value, pressure, now)
            return

        fps_state.clog_last_extruder = extruder_pos
        if fps_state.clog_min_pressure is None or pressure < fps_state.clog_min_pressure:
            fps_state.clog_min_pressure = pressure
        if fps_state.clog_max_pressure is None or pressure > fps_state.clog_max_pressure:
            fps_state.clog_max_pressure = pressure

        extrusion_delta = extruder_pos - (fps_state.clog_start_extruder or extruder_pos)
        encoder_delta = abs(encoder_value - (fps_state.clog_start_encoder or encoder_value))
        pressure_span = (fps_state.clog_max_pressure or pressure) - (fps_state.clog_min_pressure or pressure)

        settings = self.clog_settings
        if extrusion_delta < settings["extrusion_window"]:
            return

        if (encoder_delta > settings["encoder_slack"] or pressure_span > settings["pressure_band"]):
            # Encoder is moving or pressure is varying - filament is flowing
            # If clog was previously active, clear it and ensure follower keeps up
            if fps_state.clog_active:
                self.logger.info(
                    "Clog cleared on %s - encoder moving normally (delta=%d, pressure_span=%.2f)",
                    fps_name,
                    encoder_delta,
                    pressure_span,
                )

                if oams is not None and fps_state.current_spool_idx is not None:
                    self._set_led_error_if_changed(
                        oams, fps_state.current_oams, fps_state.current_spool_idx, 0, "clog auto-recovery"
                    )

                fps_state.reset_clog_tracker()

            fps_state.prime_clog_tracker(extruder_pos, encoder_value, pressure, now)
            return

        if now - (fps_state.clog_start_time or now) < settings["dwell"]:
            return

        if not fps_state.clog_active:
            if oams is not None and fps_state.current_spool_idx is not None:
                self._set_led_error_if_changed(oams, fps_state.current_oams, fps_state.current_spool_idx, 1, "clog detected")

            pressure_mid = (fps_state.clog_min_pressure + fps_state.clog_max_pressure) / 2.0
            message = (
                f"Clog suspected on {fps_state.current_lane or fps_name}: "
                f"extruder advanced {extrusion_delta:.1f}mm while encoder moved {encoder_delta} counts "
                f"with FPS {pressure_mid:.2f} near {self.clog_pressure_target:.2f}"
            )

            fps_state.clog_active = True

            # Pause printer with error message
            self._pause_printer_message(message, fps_state.current_oams)

    def start_monitors(self):
        """Start all monitoring timers"""
        # Stop existing monitors first to prevent timer leaks
        if self.monitor_timers:
            self.stop_monitors()

        self.monitor_timers = []
        self.runout_monitors = {}
        reactor = self.printer.get_reactor()
        
        for fps_name in self.current_state.fps_state.keys():
            self.monitor_timers.append(
                reactor.register_timer(
                    self._unified_monitor_for_fps(fps_name), 
                    reactor.NOW
                )
            )

            def _reload_callback(fps_name=fps_name, fps_state=self.current_state.fps_state[fps_name]):
                monitor = self.runout_monitors.get(fps_name)
                source_lane_name = fps_state.current_lane
                active_oams = fps_state.current_oams

                # Handle cross-extruder runouts using box turtle infinite runout sequence
                # Box turtle sequence: PAUSE -> SAVE_POS -> CHANGE_TOOL -> SET_MAP -> LANE_UNLOAD -> RESTORE_POS -> RESUME
                if getattr(fps_state, 'is_cross_extruder_runout', False):
                    self.logger.info("OAMS: Handling cross-extruder runout for %s (using box turtle sequence)", fps_name)

                    # Get target lane from AFC runout_lane configuration
                    afc = self._get_afc()
                    target_lane_name = None
                    source_lane = None

                    if afc and source_lane_name:
                        source_lane = afc.lanes.get(source_lane_name)
                        if source_lane:
                            target_lane_name = getattr(source_lane, 'runout_lane', None)
                            if target_lane_name:
                                self.logger.info("OAMS: Found runout_lane=%s for lane %s", target_lane_name, source_lane_name)

                    if not target_lane_name:
                        self.logger.error("OAMS: No runout lane configured for cross-extruder runout on %s", source_lane_name or fps_name)
                        fps_state.reset_runout_positions()
                        fps_state.is_cross_extruder_runout = False
                        self._pause_printer_message(f"No runout lane configured for {source_lane_name or fps_name}", active_oams)
                        if monitor:
                            monitor.paused()
                        return

                    # DON'T update hardware service snapshot for cross-extruder runouts
                    # Lane0 might share the same AMS bay as lane8, and updating the snapshot
                    # to "empty" would cause AFC to reject loading lane0 with "filament detected but not loaded"

                    # Execute cross-extruder runout sequence: Heat target, then use CHANGE_TOOL (like box turtle)
                    try:
                        self.logger.info("OAMS: Cross-extruder infinite runout: %s -> %s", source_lane_name, target_lane_name)
                        gcode = self.printer.lookup_object("gcode")

                        # 1. Pause printer
                        self.logger.info("OAMS: Step 1 - Pausing printer")
                        gcode.run_script("PAUSE")

                        # 2. Save position
                        self.logger.info("OAMS: Step 2 - Saving position")
                        afc.save_pos()

                        # 3. Z-hop to lift nozzle off print
                        self.logger.info("OAMS: Step 3 - Z-hop 5mm")
                        gcode.run_script("G91")  # Relative positioning
                        gcode.run_script("G1 Z5 F600")  # Lift 5mm
                        gcode.run_script("G90")  # Absolute positioning

                        # 4. Get target lane and extruder
                        self.logger.info("OAMS: Step 4 - Getting target lane info")
                        target_lane = afc.lanes.get(target_lane_name)
                        if not target_lane:
                            raise Exception(f"Target lane {target_lane_name} not found in AFC")

                        target_extruder_name = getattr(target_lane, 'extruder_name', None)
                        if not target_extruder_name:
                            raise Exception(f"Target lane {target_lane_name} has no extruder_name")

                        # Get source extruder name for turning it off later
                        source_extruder_name = getattr(source_lane, 'extruder_name', None) if source_lane else None

                        # Get current extruder temp to use for target
                        current_extruder = self.printer.lookup_object('toolhead').get_extruder()
                        target_temp = current_extruder.get_heater().target_temp
                        if target_temp <= 0:
                            raise Exception(f"Current extruder has no target temp set")

                        # 5. Set target extruder temp before CHANGE_TOOL (so it knows what temp to heat to)
                        self.logger.info("OAMS: Step 5 - Setting target temp for %s to %.1f", target_extruder_name, target_temp)
                        target_extruder_obj = self.printer.lookup_object(target_extruder_name)
                        target_heater = target_extruder_obj.get_heater()
                        target_heater.set_temp(target_temp)

                        # 6. Use AFC's CHANGE_TOOL Python method (like box turtle does)
                        # This should handle tool switching, heating, and loading
                        self.logger.info("OAMS: Step 6 - Calling afc.CHANGE_TOOL for %s", target_lane_name)
                        afc.CHANGE_TOOL(target_lane, restore_pos=False)

                        # 8. Update FPS state - this FPS is now UNLOADED (new lane is on different FPS/tool)
                        # Cross-extruder runout means lane0 is on a different FPS, so this FPS has nothing loaded
                        self.logger.info("OAMS: Step 8 - Setting FPS state to UNLOADED (lane loaded to different FPS)")
                        fps_state.state = FPSLoadState.UNLOADED
                        fps_state.current_lane = None
                        fps_state.current_spool_idx = None
                        fps_state.current_oams = None

                        # 9. Set mapping so T# references use new lane
                        self.logger.info("OAMS: Step 9 - Setting lane mapping")
                        empty_lane_map = getattr(source_lane, 'map', None) or source_lane_name
                        gcode.run_script(f"SET_MAP LANE={target_lane_name} MAP={empty_lane_map}")
                        self.logger.info("OAMS: Set mapping %s -> %s", target_lane_name, empty_lane_map)

                        # 10. Unload empty lane from unit
                        if not afc.error_state:
                            self.logger.info("OAMS: Step 10 - Unloading empty lane %s", source_lane_name)
                            gcode.run_script(f"LANE_UNLOAD LANE={source_lane_name}")

                            # 11. Restore position (brings Z back down) and resume
                            self.logger.info("OAMS: Step 11 - Restoring position and resuming")
                            afc.restore_pos()
                            gcode.run_script("RESUME")

                            # 12. Turn off the old extruder that ran out (now that we're on the new tool)
                            if source_extruder_name:
                                try:
                                    self.logger.info("OAMS: Step 12 - Turning off old extruder %s", source_extruder_name)
                                    source_extruder_obj = self.printer.lookup_object(source_extruder_name)
                                    source_heater = source_extruder_obj.get_heater()
                                    source_heater.set_temp(0)
                                    self.logger.info("OAMS: Set %s heater target to 0", source_extruder_name)
                                except Exception as e:
                                    self.logger.error("OAMS: Failed to turn off old extruder %s: %s", source_extruder_name, e)
                            else:
                                self.logger.warning("OAMS: Cannot turn off old extruder - source_extruder_name not available")
                        else:
                            self.logger.error("OAMS: AFC error_state is set, skipping LANE_UNLOAD and RESUME")

                        # Clear cross-extruder flag on source lane
                        if source_lane_name:
                            try:
                                source_lane_obj = afc.lanes.get(source_lane_name)
                                if source_lane_obj and hasattr(source_lane_obj, '_oams_cross_extruder_runout'):
                                    source_lane_obj._oams_cross_extruder_runout = False
                                    self.logger.info("OAMS: Cleared cross-extruder runout flag on lane %s", source_lane_name)
                            except Exception as e:
                                self.logger.error("OAMS: Failed to clear cross-extruder runout flag on lane %s: %s", source_lane_name, e)

                        # Reset state and restart monitoring
                        fps_state.reset_runout_positions()
                        fps_state.is_cross_extruder_runout = False
                        if monitor:
                            monitor.reset()
                            monitor.start()
                        self.logger.info("OAMS: Cross-extruder runout sequence completed successfully")
                        return

                    except Exception as e:
                        self.logger.error("OAMS: Failed to execute cross-extruder runout sequence - Exception: %s", str(e))

                        # Clear cross-extruder flag on error too
                        if source_lane_name:
                            try:
                                afc = self.printer.lookup_object('AFC')
                                if afc and hasattr(afc, 'lanes'):
                                    source_lane_obj = afc.lanes.get(source_lane_name)
                                    if source_lane_obj and hasattr(source_lane_obj, '_oams_cross_extruder_runout'):
                                        source_lane_obj._oams_cross_extruder_runout = False
                            except Exception:
                                pass

                        fps_state.reset_runout_positions()
                        fps_state.is_cross_extruder_runout = False
                        self._pause_printer_message(f"Failed to change tool for {source_lane_name or fps_name}: {str(e)}", active_oams)
                        if monitor:
                            monitor.paused()
                        return

                # Standard runout handling for same-extruder runouts
                target_lane_map, target_lane, delegate_to_afc, source_lane = self._get_infinite_runout_target_lane(fps_name, fps_state)
                source_lane_name = fps_state.current_lane

                if delegate_to_afc:
                    delegated = self._delegate_runout_to_afc(fps_name, fps_state, source_lane, target_lane)
                    if delegated:
                        fps_state.reset_runout_positions()
                        if monitor:
                            monitor.reset()
                            monitor.start()
                        return

                    self.logger.error("Failed to delegate infinite runout for %s on %s via AFC", fps_name, source_lane_name or "<unknown>")
                    fps_state.reset_runout_positions()
                    self._pause_printer_message(f"Unable to delegate infinite runout for {source_lane_name or fps_name}", fps_state.current_oams or active_oams)
                    if monitor:
                        monitor.paused()
                    return

                # Load the target lane directly
                if target_lane is None:
                    # No infinite runout target configured - clear the lane and pause
                    self.logger.info("No infinite runout target for %s on %s - clearing lane from toolhead and OAMS",
                                   source_lane_name or fps_name, fps_name)

                    # Clear FPS state and notify AFC (similar to cross-extruder runout handling)
                    self._clear_lane_on_runout(fps_name, fps_state, source_lane_name)

                    self.logger.error("No lane available to reload on %s", fps_name)
                    self._pause_printer_message(f"No lane available to reload on {fps_name}", fps_state.current_oams or active_oams)
                    if monitor:
                        monitor.paused()
                    return

                if target_lane_map:
                    self.logger.info("Infinite runout triggered for %s on %s -> %s", fps_name, source_lane_name, target_lane)
                    unload_success, unload_message = self._unload_filament_for_fps(fps_name)
                    if not unload_success:
                        self.logger.error("Failed to unload filament during infinite runout on %s: %s", fps_name, unload_message)
                        failure_message = unload_message or f"Failed to unload current spool on {fps_name}"
                        self._pause_printer_message(failure_message, fps_state.current_oams or active_oams)
                        if monitor:
                            monitor.paused()
                        return

                load_success, load_message = self._load_filament_for_lane(target_lane)
                if load_success:
                    self.logger.info("Successfully loaded lane %s on %s%s", target_lane, fps_name, " after infinite runout" if target_lane_map else "")

                    # Always notify AFC that target lane is loaded to update virtual sensors
                    # This ensures AMS_Extruder# sensors show correct state after same-FPS runouts
                    if target_lane:
                        handled = False
                        if AMSRunoutCoordinator is not None:
                            try:
                                handled = AMSRunoutCoordinator.notify_lane_tool_state(self.printer, fps_state.current_oams or active_oams, target_lane, loaded=True, spool_index=fps_state.current_spool_idx, eventtime=fps_state.since)
                                if handled:
                                    self.logger.info("Notified AFC that lane %s is loaded via AMSRunoutCoordinator (updates virtual sensor state)", target_lane)
                                else:
                                    self.logger.warning("AMSRunoutCoordinator.notify_lane_tool_state returned False for lane %s, trying fallback", target_lane)
                            except Exception as e:
                                self.logger.error("Failed to notify AFC lane %s after infinite runout on %s: %s", target_lane, fps_name, e)
                                handled = False
                        else:
                            # AMSRunoutCoordinator not available - call AFC methods directly
                            self.logger.info("AMSRunoutCoordinator not available, updating virtual sensor directly for lane %s", target_lane)
                            try:
                                afc = self._get_afc()
                                if afc and hasattr(afc, 'lanes'):
                                    lane_obj = afc.lanes.get(target_lane)
                                    if lane_obj:
                                        # Update virtual sensor using AFC's direct method
                                        if hasattr(afc, '_mirror_lane_to_virtual_sensor'):
                                            afc._mirror_lane_to_virtual_sensor(lane_obj, self.reactor.monotonic())
                                            self.logger.info("Updated virtual sensor for lane %s via AFC._mirror_lane_to_virtual_sensor", target_lane)
                                            handled = True
                                        else:
                                            self.logger.warning("AFC doesn't have _mirror_lane_to_virtual_sensor method")
                                    else:
                                        self.logger.warning("Lane object not found for %s in AFC", target_lane)
                                else:
                                    self.logger.warning("AFC not available or has no lanes attribute")
                            except Exception as e:
                                self.logger.error("Failed to update virtual sensor directly for lane %s: %s", target_lane, e)

                        if not handled:
                            try:
                                gcode = self.printer.lookup_object("gcode")
                                gcode.run_script(f"SET_LANE_LOADED LANE={target_lane}")
                                self.logger.info("Marked lane %s as loaded via SET_LANE_LOADED after infinite runout on %s", target_lane, fps_name)
                            except Exception as e:
                                self.logger.error("Failed to mark lane %s as loaded after infinite runout on %s: %s", target_lane, fps_name, e)

                    # Ensure follower is enabled after successful reload
                    # Follower should stay enabled throughout same-FPS runouts (never disabled)
                    # This is a safety check to ensure follower is active for new lane
                    if fps_state.current_oams and fps_state.current_spool_idx is not None:
                        oams = self.oams.get(fps_state.current_oams)
                        if oams:
                            self._ensure_forward_follower(fps_name, fps_state, "after infinite runout reload")

                    # Clear the source lane's state in AFC so it shows as EMPTY and can detect new filament
                    # FPS state stays LOADED with the new target lane, but old lane needs to be cleared
                    if source_lane_name:
                        try:
                            if AMSRunoutCoordinator is not None:
                                # Notify AFC that source lane is now unloaded
                                AMSRunoutCoordinator.notify_lane_tool_state(
                                    self.printer,
                                    fps_state.current_oams or active_oams,
                                    source_lane_name,
                                    loaded=False,
                                    spool_index=None,
                                    eventtime=self.reactor.monotonic()
                                )
                                self.logger.info("Cleared source lane %s state in AFC after successful reload to %s", source_lane_name, target_lane)
                            else:
                                # Fallback to gcode command if coordinator not available
                                gcode = self.printer.lookup_object("gcode")
                                gcode.run_script(f"SET_LANE_UNLOADED LANE={source_lane_name}")
                                self.logger.info("Cleared source lane %s via SET_LANE_UNLOADED after reload to %s", source_lane_name, target_lane)
                        except Exception:
                            self.logger.error("Failed to clear source lane %s state after reload to %s", source_lane_name, target_lane)

                    fps_state.reset_runout_positions()
                    if monitor:
                        monitor.reset()
                        monitor.start()
                    return

                self.logger.error("Failed to load lane %s on %s: %s", target_lane, fps_name, load_message)
                failure_message = load_message or f"No spool available for lane {target_lane}"
                self._pause_printer_message(failure_message, fps_state.current_oams or active_oams)
                if monitor:
                    monitor.paused()

            fps_reload_margin = getattr(self.fpss[fps_name], "reload_before_toolhead_distance", None)
            if fps_reload_margin is None:
                fps_reload_margin = self.reload_before_toolhead_distance

            monitor = OAMSRunoutMonitor(self.printer, fps_name, self.fpss[fps_name], self.current_state.fps_state[fps_name], self.oams, _reload_callback, reload_before_toolhead_distance=fps_reload_margin)
            self.runout_monitors[fps_name] = monitor
            monitor.start()

        self.logger.info("All monitors started (optimized)")

    # ============================================================================
    # AFC State Change Notifications (Optional Callbacks)
    # ============================================================================
    # These methods can be called by AFC when lane state changes to keep OAMS
    # manager in sync immediately without waiting for state detection polling.
    # AFC integration is optional - these are no-ops if called when AFC is not configured.

    def on_afc_lane_loaded(self, lane_name: str, extruder_name: Optional[str] = None) -> None:
        """Callback for AFC to notify OAMS when a lane is loaded.

        Args:
            lane_name: Name of the lane that was loaded (e.g., "lane4")
            extruder_name: Optional name of the extruder/tool it was loaded to

        This allows OAMS to:
        - Update FPS state immediately (no polling lag)
        - Enable follower motor instantly
        - Sync state for accurate monitoring
        """
        try:
            # Get FPS for this lane (uses cache for speed)
            fps_name = self.get_fps_for_afc_lane(lane_name)
            if not fps_name:
                return  # Lane not on any FPS we manage

            fps_state = self.current_state.fps_state.get(fps_name)
            if fps_state is None:
                return

            # Let state detection handle the full sync
            # We just trigger it to run immediately instead of waiting for next poll
            detected_lane_name, oam, bay_index = self._determine_loaded_lane_for_fps(fps_name, self.fpss[fps_name])

            if detected_lane_name and oam and bay_index is not None:
                # Update FPS state - store lane name (e.g., "lane8") not map (e.g., "T0")
                fps_state.current_lane = detected_lane_name
                fps_state.current_oams = oam.name
                fps_state.current_spool_idx = bay_index
                fps_state.state = FPSLoadState.LOADED
                fps_state.since = self.reactor.monotonic()
                fps_state.direction = 1

                # Enable follower immediately
                self._ensure_forward_follower(fps_name, fps_state, "AFC lane loaded notification")

                self.logger.info("Synced OAMS state from AFC: %s loaded to %s (bay %d on %s)",
                               detected_lane_name, fps_name, bay_index, oam.name)
        except Exception:
            self.logger.error("Error processing AFC lane loaded notification for %s", lane_name)

    def on_afc_lane_unloaded(self, lane_name: str, extruder_name: Optional[str] = None) -> None:
        """Callback for AFC to notify OAMS when a lane is unloaded.

        Args:
            lane_name: Name of the lane that was unloaded (e.g., "lane4")
            extruder_name: Optional name of the extruder/tool it was unloaded from

        This allows OAMS to:
        - Update FPS state immediately
        - Disable follower motor
        - Clear monitoring state
        """
        try:
            # Get FPS for this lane (uses cache for speed)
            fps_name = self.get_fps_for_afc_lane(lane_name)
            if not fps_name:
                return  # Lane not on any FPS we manage

            fps_state = self.current_state.fps_state.get(fps_name)
            if fps_state is None:
                return

            # Only update if this lane was actually loaded on this FPS
            if fps_state.state == FPSLoadState.LOADED:
                # Disable follower
                if fps_state.current_oams and fps_state.following:
                    oam = self.oams.get(fps_state.current_oams)
                    if oam:
                        try:
                            oam.set_oams_follower(0, fps_state.direction)
                            fps_state.following = False
                        except Exception:
                            self.logger.error("Failed to disable follower during AFC unload notification")

                # Update state
                fps_state.state = FPSLoadState.UNLOADED
                fps_state.current_lane = None
                fps_state.current_oams = None
                fps_state.current_spool_idx = None
                fps_state.since = self.reactor.monotonic()

                self.logger.info("Synced OAMS state from AFC: %s unloaded from %s", lane_name, fps_name)
        except Exception:
            self.logger.error("Error processing AFC lane unloaded notification for %s", lane_name)

    def stop_monitors(self):
        for timer in self.monitor_timers:
            self.printer.get_reactor().unregister_timer(timer)
        self.monitor_timers = []
        for monitor in self.runout_monitors.values():
            monitor.reset()
        self.runout_monitors = {}

    @contextmanager
    def _monitors_suspended(self, reason: str = ""):
        """Pause monitors while performing coordinated reset work."""
        monitors_were_running = bool(self.monitor_timers)
        if monitors_were_running:
            try:
                self.stop_monitors()
                if reason:
                    self.logger.debug("Monitors paused for %s", reason)
            except Exception:
                self.logger.exception("Failed to pause monitors%s", f" for {reason}" if reason else "")
                monitors_were_running = False
        try:
            yield
        finally:
            if monitors_were_running:
                try:
                    self.start_monitors()
                    if reason:
                        self.logger.debug("Monitors resumed after %s", reason)
                except Exception:
                    self.logger.exception("Failed to resume monitors%s", f" after {reason}" if reason else "")


def load_config(config):
    return OAMSManager(config)