# OpenAMS Mainboard
#
# Copyright (C) 2025 JR Lomas <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
import mcu
import struct
from math import pi
from typing import Tuple, List, Optional, Any

# OAMS Hardware Status Constants
class OAMSStatus:
    """Hardware status codes reported by OAMS firmware."""
    LOADING = 0              # Currently loading filament
    UNLOADING = 1            # Currently unloading filament  
    FORWARD_FOLLOWING = 2    # Following extruder in forward direction
    REVERSE_FOLLOWING = 3    # Following extruder in reverse direction
    COASTING = 4             # Coasting without active control
    STOPPED = 5              # Motor stopped, idle state
    CALIBRATING = 6          # Running calibration procedure
    ERROR = 7                # Error state requiring intervention

# OAMS Operation Result Codes  
class OAMSOpCode:
    """Operation result codes from OAMS firmware."""
    SUCCESS = 0                    # Operation completed successfully
    ERROR_UNSPECIFIED = 1         # Generic error occurred
    ERROR_BUSY = 2                # OAMS busy with another operation
    SPOOL_ALREADY_IN_BAY = 3      # Attempted to load when bay occupied
    NO_SPOOL_IN_BAY = 4           # Attempted to unload empty bay
    ERROR_KLIPPER_CALL = 5        # Error in Klipper communication


class OAMS:
    """
    OpenAMS hardware controller for managing filament spools.
    
    Hardware Interface:
    - Controls 4 filament bays (indexed 0-3)
    - Monitors Hall Effect Sensors (HES) for spool detection
    - Manages BLDC motor for filament feeding
    - Tracks encoder position for motion feedback
    
    Key State Variables:
    - current_spool: Index (0-3) of currently loaded spool, None if unloaded
    - f1s_hes_value: Array of filament sensor readings [bay0, bay1, bay2, bay3]
    - hub_hes_value: Array of hub sensor readings [bay0, bay1, bay2, bay3] 
    - fps_value: Current pressure sensor reading
    - encoder_clicks: Current encoder position
    """
    
    def __init__(self, config):
        # Core printer interface
        self.printer = config.get_printer()

        # keep the full config section name (e.g. "oams oams1")
        # and also the short unit name (e.g. "oams1")
        self.name = config.get_name()  # full section name
        self.unit_name = self.name.split()[-1]  # short name
        
        self.mcu = mcu.get_printer_mcu(self.printer, config.get("mcu", "mcu"))
        self.reactor = self.printer.get_reactor()
        
        # Hardware configuration - Pressure sensor thresholds
        self.fps_upper_threshold: float = config.getfloat("fps_upper_threshold")
        self.fps_lower_threshold: float = config.getfloat("fps_lower_threshold") 
        self.fps_is_reversed: bool = config.getboolean("fps_is_reversed")
        
        # Current state variables
        self.current_spool: Optional[int] = None  # Currently loaded spool index (0-3)
        self.encoder_clicks: int = 0  # Current encoder position
        self.i_value: float = 0.0  # Current sensor value
        
        # Sensor configuration - Hall Effect Sensor thresholds
        self.f1s_hes_on: List[float] = list(
            map(lambda x: float(x.strip()), config.get("f1s_hes_on").split(","))
        )
        self.f1s_hes_is_above: bool = config.getboolean("f1s_hes_is_above")
        self.hub_hes_on: List[float] = list(
            map(lambda x: float(x.strip()), config.get("hub_hes_on").split(","))
        )
        self.hub_hes_is_above: bool = config.getboolean("hub_hes_is_above")
        
        # Physical configuration
        self.filament_path_length: float = config.getfloat("ptfe_length")
        self.oams_idx: int = config.getint("oams_idx")

        # PID control parameters for pressure
        self.kd: float = config.getfloat("kd", 0.0)
        self.ki: float = config.getfloat("ki", 0.0)
        self.kp: float = config.getfloat("kp", 6.0)

        # PID control parameters for current
        self.current_kp: float = config.getfloat("current_kp", 0.375)
        self.current_ki: float = config.getfloat("current_ki", 0.0)
        self.current_kd: float = config.getfloat("current_kd", 0.0)

        # Target values
        self.fps_target: float = config.getfloat(
            "fps_target",
            0.5,
            minval=0.0,
            maxval=1.0,
            above=self.fps_lower_threshold,
            below=self.fps_upper_threshold,
        )
        self.current_target: float = config.getfloat(
            "current_target", 0.3, minval=0.1, maxval=0.4
        )
        
        # Hardware state arrays (updated by firmware)
        self.fps_value: float = 0  # Current pressure reading
        self.f1s_hes_value: List[int] = [0, 0, 0, 0]  # Filament sensors [bay0, bay1, bay2, bay3]
        self.hub_hes_value: List[int] = [0, 0, 0, 0]  # Hub sensors [bay0, bay1, bay2, bay3]
        
        # Action status tracking
        self.action_status: Optional[int] = None
        self.action_status_code: Optional[int] = None
        self.action_status_value: Optional[int] = None
        
        # NOTE:
        # We need this per-unit instance to be discoverable during config parsing
        # (so modules like filament_group can lookup "oams oams1" or "oams1").
        # The original code relied on Klipper to register the object; to be robust
        # we register both forms now. However registering too early previously
        # caused an AttributeError in some environments because MCU callback
        # wiring happened before the instance was fully ready. To avoid that
        # we:
        #  - register the object name(s) immediately so lookups succeed
        #  - delay registering MCU responses/config callback until handle_ready().
        try:
            try:
                self.printer.add_object(self.name, self)
            except Exception:
                logging.debug("OAMS: printer.add_object('%s') failed or already registered", self.name)
            try:
                self.printer.add_object(self.unit_name, self)
            except Exception:
                logging.debug("OAMS: printer.add_object('%s') failed or already registered", self.unit_name)
            logging.info("OAMS: Registered object names '%s' and '%s' for unit", self.name, self.unit_name)
        except Exception:
            logging.exception("OAMS: Failed to register OAMS object names for %s", getattr(self, "name", "<unknown>"))

        # register commands (gcode mux etc) and schedule handle_ready to finish setup
        self.register_commands(self.unit_name)
        self.printer.register_event_handler("klippy:ready", self.handle_ready)

    def get_status(self, eventtime: float) -> dict:
        """Return current hardware status for monitoring."""
        return {
            "current_spool": self.current_spool,
            "f1s_hes_value": list(self.f1s_hes_value),
            "hub_hes_value": list(self.hub_hes_value),
            "fps_value": self.fps_value
        }
    
    def is_bay_ready(self, bay_index: int) -> bool:
        """Check if a spool bay has filament ready to load (filament sensor active)."""
        return bool(self.f1s_hes_value[bay_index])
    
    def is_bay_loaded(self, bay_index: int) -> bool:
        """Check if a spool bay has filament loaded into the hub (hub sensor active)."""
        return bool(self.hub_hes_value[bay_index])
    
    def stats(self, eventtime):
        return (
            False,
            """
OAMS[%s]: current_spool=%s fps_value=%s f1s_hes_value_0=%d f1s_hes_value_1=%d f1s_hes_value_2=%d f1s_hes_value_3=%d hub_hes_value_0=%d hub_hes_value_1=%d hub_hes_value_2=%d hub_hes_value_3=%d kp=[...]
"""
            % ( self.oams_idx,
                self.current_spool,
                self.fps_value,
                self.f1s_hes_value[0],
                self.f1s_hes_value[1],
                self.f1s_hes_value[2],
                self.f1s_hes_value[3],
                self.hub_hes_value[0],
                self.hub_hes_value[1],
                self.hub_hes_value[2],
                self.hub_hes_value[3],
                self.kp,
                self.ki,
                self.kd,
                self.encoder_clicks,
                self.i_value,
            ),
        )

    def handle_ready(self):
        """
        Finish initialization that requires the reactor/MCU to be ready.
        Register MCU response handlers and lookup commands here so that any
        framework wiring that might invoke callbacks won't happen before the
        class is fully constructed.
        """
        try:
            # Wire MCU response handlers and config callback now (was previously
            # in __init__, which could lead to callbacks being invoked too early).
            self.mcu.register_response(self._oams_action_status, "oams_action_status")
            self.mcu.register_response(self._oams_cmd_stats, "oams_cmd_stats")
            self.mcu.register_response(self._oams_cmd_current_stats, "oams_cmd_current_status")
            self.mcu.register_config_callback(self._build_config)

            # Lookup / prepare MCU commands
            self.oams_load_spool_cmd = self.mcu.lookup_command(
                "oams_cmd_load_spool spool=%c"
            )

            self.oams_unload_spool_cmd = self.mcu.lookup_command(
                "oams_cmd_unload_spool"
            )

            self.oams_follower_cmd = self.mcu.lookup_command(
                "oams_cmd_follower enable=%c direction=%c"
            )

            self.oams_calibrate_ptfe_length_cmd = self.mcu.lookup_command(
                "oams_cmd_calibrate_ptfe_length spool=%c"
            )

            self.oams_calibrate_hub_hes_cmd = self.mcu.lookup_command(
                "oams_cmd_calibrate_hub_hes spool=%c"
            )

            self.oams_pid_cmd = self.mcu.lookup_command(
                "oams_cmd_pid kp=%u ki=%u kd=%u target=%u"
            )

            # LED error command
            self.oams_set_led_error_cmd = self.mcu.lookup_command(
                "oams_set_led_error idx=%c value=%c"
            )

            cmd_queue = self.mcu.alloc_command_queue()

            self.oams_spool_query_spool_cmd = self.mcu.lookup_query_command(
                "oams_cmd_query_spool",
                "oams_query_response_spool spool=%u",
                cq=cmd_queue,
            )
            
            # Clear any stale error states and determine current spool
            self.clear_errors()
        except Exception as e:
            logging.error("Failed to initialize OAMS commands/MCU wiring: %s", e)

    def get_spool_status(self, bay_index):
        return self.f1s_hes_value[bay_index]
            
    def clear_errors(self):
        for i in range(4):
            try:
                self.set_led_error(i, 0)
            except Exception:
                logging.debug("OAMS: set_led_error failed during clear_errors")
        try:
            self.current_spool = self.determine_current_spool()
        except Exception:
            logging.debug("OAMS: determine_current_spool failed during clear_errors")
            
    def set_led_error(self, idx, value):
        logging.info("Setting LED %d to %d", idx, value)
        try:
            self.oams_set_led_error_cmd.send([idx, value])
        except Exception:
            logging.exception("OAMS: Failed to send set_led_error command")
        # TODO: need to restore the actual value of the LED when resetting the error
        
            
    def determine_current_spool(self):
        try:
            params = self.oams_spool_query_spool_cmd.send()
            if params is not None and "spool" in params:
                if params["spool"] >= 0 and params["spool"] <= 3:
                    return params["spool"]
        except Exception:
            logging.exception("OAMS: Failed to query current spool")
        return None
        

    def register_commands(self, name):
        id = str(self.oams_idx)
        # Register commands
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command(
            "OAMS_LOAD_SPOOL",
            "OAMS",
            id,
            self.cmd_OAMS_LOAD_SPOOL,
            desc=self.cmd_OAMS_LOAD_SPOOL_help,
        )
        gcode.register_mux_command(
            "OAMS_UNLOAD_SPOOL",
            "OAMS",
            id,
            self.cmd_OAMS_UNLOAD_SPOOL,
            self.cmd_OAMS_UNLOAD_SPOOL_help,
        )

        gcode.register_mux_command(
            "OAMS_FOLLOWER",
            "OAMS",
            id,
            self.cmd_OAMS_FOLLOWER,
            self.cmd_OAMS_FOLLOWER_help,
        )

        gcode.register_mux_command(
            "OAMS_CALIBRATE_PTFE_LENGTH",
            "OAMS",
            id,
            self.cmd_OAMS_CALIBRATE_PTFE_LENGTH,
            self.cmd_OAMS_CALIBRATE_PTFE_LENGTH_help,
        )

        gcode.register_mux_command(
            "OAMS_CALIBRATE_HUB_HES",
            "OAMS",
            id,
            self.cmd_OAMS_CALIBRATE_HUB_HES,
            self.cmd_OAMS_CALIBRATE_HUB_HES_help,
        )

        gcode.register_mux_command(
            "OAMS_PID_AUTOTUNE",
            "OAMS",
            id,
            self.cmd_OAMS_PID_AUTOTUNE,
            self.cmd_OAMS_PID_AUTOTUNE_help,
        )

        gcode.register_mux_command(
            "OAMS_PID_SET",
            "OAMS",
            id,
            self.cmd_OAMS_PID_SET,
            self.cmd_OAMS_PID_SET_help,
        )

        gcode.register_mux_command(
            "OAMS_CURRENT_PID_SET",
            "OAMS",
            id,
            self.cmd_OAMS_CURRENT_PID_SET,
            self.cmd_OAMS_CURRENT_PID_SET_help,
        )

    cmd_OAMS_CURRENT_PID_SET_help = "Set the PID values for the current sensor"

    def cmd_OAMS_CURRENT_PID_SET(self, gcmd):
        p = gcmd.get_float("P", None)
        i = gcmd.get_float("I", None)
        d = gcmd.get_float("D", None)
        t = gcmd.get_float("TARGET", None)
        if p is None:
            raise gcmd.error("P value is required")
        if i is None:
            raise gcmd.error("I value is required")
        if d is None:
            raise gcmd.error("D value is required")
        if t is None:
            t = self.current_target
        kp = self.float_to_u32(p)
        ki = self.float_to_u32(i)
        kd = self.float_to_u32(d)
        kt = self.float_to_u32(t)
        self.oams_pid_cmd.send([kp, ki, kd, kt])
        self.current_kp = p
        self.current_ki = i
        self.current_kd = d
        self.current_target = t
        gcmd.respond_info(
            "Current PID values set to P=%f I=%f D=%f TARGET=%f" % (p, i, d, t)
        )

    cmd_OAMS_PID_SET_help = "Set the PID values for the OAMS"

    def cmd_OAMS_PID_SET(self, gcmd):
        p = gcmd.get_float("P", None)
        i = gcmd.get_float("I", None)
        d = gcmd.get_float("D", None)
        t = gcmd.get_float("TARGET", None)
        if p is None:
            raise gcmd.error("P value is required")
        if i is None:
            raise gcmd.error("I value is required")
        if d is None:
            raise gcmd.error("D value is required")
        if t is None:
            t = self.fps_target
        kp = self.float_to_u32(p)
        ki = self.float_to_u32(i)
        kd = self.float_to_u32(d)
        kt = self.float_to_u32(t)
        self.oams_pid_cmd.send([kp, ki, kd, kt])
        self.kp = p
        self.ki = i
        self.kd = d
        self.fps_target = t
        gcmd.respond_info("PID values set to P=%f I=%f D=%f TARGET=%f" % (p, i, d, t))

    # rest of file unchanged...
def load_config_prefix(config):
    return OAMS(config)

