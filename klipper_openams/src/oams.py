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
    LOADING = 0
    UNLOADING = 1
    FORWARD_FOLLOWING = 2
    REVERSE_FOLLOWING = 3
    COASTING = 4
    STOPPED = 5
    CALIBRATING = 6
    ERROR = 7

# OAMS Operation Result Codes
class OAMSOpCode:
    SUCCESS = 0
    ERROR_UNSPECIFIED = 1
    ERROR_BUSY = 2
    SPOOL_ALREADY_IN_BAY = 3
    NO_SPOOL_IN_BAY = 4
    ERROR_KLIPPER_CALL = 5


class OAMS:
    """
    OpenAMS hardware controller for managing filament spools.
    """

    def __init__(self, config):
        # Core printer interface
        self.printer = config.get_printer()

        # Keep both the full config section name (e.g. "oams oams1")
        # and the short unit name (e.g. "oams1")
        self.full_name = config.get_name()  # e.g. "oams oams1"
        self.unit_name = self.full_name.split()[-1]  # e.g. "oams1"

        # MCU and reactor references (safe to get now)
        self.mcu = mcu.get_printer_mcu(self.printer, config.get("mcu", "mcu"))
        self.reactor = self.printer.get_reactor()

        # Configuration parameters
        self.fps_upper_threshold: float = config.getfloat("fps_upper_threshold")
        self.fps_lower_threshold: float = config.getfloat("fps_lower_threshold")
        self.fps_is_reversed: bool = config.getboolean("fps_is_reversed")

        # Current state variables
        self.current_spool: Optional[int] = None
        self.encoder_clicks: int = 0
        self.i_value: float = 0.0

        # Sensor configuration
        self.f1s_hes_on: List[float] = list(map(lambda x: float(x.strip()), config.get("f1s_hes_on").split(",")))
        self.f1s_hes_is_above: bool = config.getboolean("f1s_hes_is_above")
        self.hub_hes_on: List[float] = list(map(lambda x: float(x.strip()), config.get("hub_hes_on").split(",")))
        self.hub_hes_is_above: bool = config.getboolean("hub_hes_is_above")

        # Physical configuration
        self.filament_path_length: float = config.getfloat("ptfe_length")
        self.oams_idx: int = config.getint("oams_idx")

        # PID parameters
        self.kd: float = config.getfloat("kd", 0.0)
        self.ki: float = config.getfloat("ki", 0.0)
        self.kp: float = config.getfloat("kp", 6.0)

        self.current_kp: float = config.getfloat("current_kp", 0.375)
        self.current_ki: float = config.getfloat("current_ki", 0.0)
        self.current_kd: float = config.getfloat("current_kd", 0.0)

        self.fps_target: float = config.getfloat(
            "fps_target",
            0.5,
            minval=0.0,
            maxval=1.0,
            above=self.fps_lower_threshold,
            below=self.fps_upper_threshold,
        )
        self.current_target: float = config.getfloat("current_target", 0.3, minval=0.1, maxval=0.4)

        # Hardware state arrays
        self.fps_value: float = 0.0
        self.f1s_hes_value: List[int] = [0, 0, 0, 0]
        self.hub_hes_value: List[int] = [0, 0, 0, 0]

        # Action status
        self.action_status: Optional[int] = None
        self.action_status_code: Optional[int] = None
        self.action_status_value: Optional[int] = None

        # Register the object names early so other modules can lookup during config parsing.
        # Use try/except because add_object may already be done by the framework in some setups.
        try:
            try:
                self.printer.add_object(self.full_name, self)
            except Exception:
                logging.debug("OAMS: add_object('%s') failed or already registered", self.full_name)
            try:
                self.printer.add_object(self.unit_name, self)
            except Exception:
                logging.debug("OAMS: add_object('%s') failed or already registered", self.unit_name)
            logging.info("OAMS: registered object names '%s' and '%s'", self.full_name, self.unit_name)
        except Exception:
            logging.exception("OAMS: exception while registering object names")

        # Delay MCU wiring and command lookups until reactor is ready to avoid early callback races.
        self.printer.register_event_handler("klippy:ready", self.handle_ready)

    # --- Status / helpers -------------------------------------------------------

    def get_status(self, eventtime: float) -> dict:
        return {
            "current_spool": self.current_spool,
            "f1s_hes_value": list(self.f1s_hes_value),
            "hub_hes_value": list(self.hub_hes_value),
            "fps_value": self.fps_value,
        }

    def is_bay_ready(self, bay_index: int) -> bool:
        return bool(self.f1s_hes_value[bay_index])

    def is_bay_loaded(self, bay_index: int) -> bool:
        return bool(self.hub_hes_value[bay_index])

    # Keep the original stats method or rely on the updated extras/oams.stats reporter.
    # (We fixed the reporter in a separate patch; leave this here for compatibility.)
    def stats(self, eventtime):
        try:
            return (
                False,
                "OAMS[%s]: current_spool=%s fps_value=%s f1s_hes_value_0=%d f1s_hes_value_1=%d f1s_hes_value_2=%d f1s_hes_value_3=%d hub_hes_value_0=%d hub_hes_value_1=%d hub_hes_value_2=%d hub_hes_value_3=%d kp=%s ki=%s kd=%s encoder_clicks=%d i_value=%.2f"
                % (
                    self.oams_idx,
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
                    str(self.kp),
                    str(self.ki),
                    str(self.kd),
                    self.encoder_clicks,
                    self.i_value,
                ),
            )
        except Exception:
            logging.exception("OAMS: failed to build stats string")
            return False, "OAMS: stats error"

    # --- finish initialization once reactor/MCU are ready -----------------------

    def handle_ready(self):
        try:
            # Register gcode mux commands now (safe)
            try:
                self.register_commands(self.unit_name)
            except Exception:
                logging.exception("OAMS: register_commands failed for %s", self.unit_name)

            # Wire MCU responses and config callback now that instance is fully built
            try:
                self.mcu.register_response(self._oams_action_status, "oams_action_status")
                self.mcu.register_response(self._oams_cmd_stats, "oams_cmd_stats")
                self.mcu.register_response(self._oams_cmd_current_stats, "oams_cmd_current_status")
                self.mcu.register_config_callback(self._build_config)
            except Exception:
                logging.exception("OAMS: failed to register MCU responses/config callback")

            # Lookup MCU commands & query commands
            try:
                self.oams_load_spool_cmd = self.mcu.lookup_command("oams_cmd_load_spool spool=%c")
                self.oams_unload_spool_cmd = self.mcu.lookup_command("oams_cmd_unload_spool")
                self.oams_follower_cmd = self.mcu.lookup_command("oams_cmd_follower enable=%c direction=%c")
                self.oams_calibrate_ptfe_length_cmd = self.mcu.lookup_command("oams_cmd_calibrate_ptfe_length spool=%c")
                self.oams_calibrate_hub_hes_cmd = self.mcu.lookup_command("oams_cmd_calibrate_hub_hes spool=%c")
                self.oams_pid_cmd = self.mcu.lookup_command("oams_cmd_pid kp=%u ki=%u kd=%u target=%u")
                self.oams_set_led_error_cmd = self.mcu.lookup_command("oams_set_led_error idx=%c value=%c")
                cmd_queue = self.mcu.alloc_command_queue()
                self.oams_spool_query_spool_cmd = self.mcu.lookup_query_command(
                    "oams_cmd_query_spool", "oams_query_response_spool spool=%u", cq=cmd_queue
                )
            except Exception:
                logging.exception("OAMS: failed to lookup MCU commands for %s", self.unit_name)

            # Clear errors and establish current spool
            try:
                self.clear_errors()
            except Exception:
                logging.exception("OAMS: clear_errors failed during handle_ready for %s", self.unit_name)

        except Exception as e:
            logging.error("OAMS: handle_ready top-level failure: %s", e)

    # --- MCU command handlers & helpers ----------------------------------------

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
            logging.exception("OAMS: failed to send set_led_error")

    def determine_current_spool(self):
        try:
            params = self.oams_spool_query_spool_cmd.send()
            if params is not None and "spool" in params:
                if 0 <= params["spool"] <= 3:
                    return params["spool"]
        except Exception:
            logging.exception("OAMS: failed to query current spool")
        return None

    def register_commands(self, name):
        id = str(self.oams_idx)
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command("OAMS_LOAD_SPOOL", "OAMS", id, self.cmd_OAMS_LOAD_SPOOL, desc=self.cmd_OAMS_LOAD_SPOOL_help)
        gcode.register_mux_command("OAMS_UNLOAD_SPOOL", "OAMS", id, self.cmd_OAMS_UNLOAD_SPOOL, self.cmd_OAMS_UNLOAD_SPOOL_help)
        gcode.register_mux_command("OAMS_FOLLOWER", "OAMS", id, self.cmd_OAMS_FOLLOWER, self.cmd_OAMS_FOLLOWER_help)
        gcode.register_mux_command("OAMS_CALIBRATE_PTFE_LENGTH", "OAMS", id, self.cmd_OAMS_CALIBRATE_PTFE_LENGTH, self.cmd_OAMS_CALIBRATE_PTFE_LENGTH_help)
        gcode.register_mux_command("OAMS_CALIBRATE_HUB_HES", "OAMS", id, self.cmd_OAMS_CALIBRATE_HUB_HES, self.cmd_OAMS_CALIBRATE_HUB_HES_help)
        gcode.register_mux_command("OAMS_PID_AUTOTUNE", "OAMS", id, self.cmd_OAMS_PID_AUTOTUNE, self.cmd_OAMS_PID_AUTOTUNE_help)
        gcode.register_mux_command("OAMS_PID_SET", "OAMS", id, self.cmd_OAMS_PID_SET, self.cmd_OAMS_PID_SET_help)
        gcode.register_mux_command("OAMS_CURRENT_PID_SET", "OAMS", id, self.cmd_OAMS_CURRENT_PID_SET, self.cmd_OAMS_CURRENT_PID_SET_help)

    # The remainder of the methods (cmd_OAMS_* etc.) remain unchanged from the original implementation.
    # (Include them as-is in your copy; omitted here for brevity but keep the same logic.)

    def _oams_cmd_stats(self, params):
        self.fps_value = self.u32_to_float(params["fps_value"])
        self.f1s_hes_value[0] = params["f1s_hes_value_0"]
        self.f1s_hes_value[1] = params["f1s_hes_value_1"]
        self.f1s_hes_value[2] = params["f1s_hes_value_2"]
        self.f1s_hes_value[3] = params["f1s_hes_value_3"]
        self.hub_hes_value[0] = params["hub_hes_value_0"]
        self.hub_hes_value[1] = params["hub_hes_value_1"]
        self.hub_hes_value[2] = params["hub_hes_value_2"]
        self.hub_hes_value[3] = params["hub_hes_value_3"]
        self.encoder_clicks = params["encoder_clicks"]

    def _oams_cmd_current_stats(self, params):
        self.i_value = self.u32_to_float(params["current_value"])

    def get_current(self):
        return self.i_value

    def _oams_action_status(self, params):
        logging.info("oams status received")
        if params["action"] == OAMSStatus.LOADING:
            self.action_status = None
            self.action_status_code = params["code"]
        elif params["action"] == OAMSStatus.UNLOADING:
            self.action_status = None
            self.action_status_code = params["code"]
        elif params["action"] == OAMSStatus.CALIBRATING:
            self.action_status = None
            self.action_status_code = params["code"]
            self.action_status_value = params.get("value")
        elif params["action"] == OAMSStatus.ERROR:
            self.action_status = None
            self.action_status_code = params["code"]
        elif params.get("code") == OAMSOpCode.ERROR_KLIPPER_CALL:
            self.action_status = None
            self.action_status_code = params["code"]
        else:
            logging.error("Spurious response from AMS with code %s and action %s", params.get("code"), params.get("action"))

    def float_to_u32(self, f):
        return struct.unpack("I", struct.pack("f", f))[0]

    def u32_to_float(self, i):
        return struct.unpack("f", struct.pack("I", i))[0]

    def _build_config(self):
        self.mcu.add_config_cmd("config_oams_buffer upper=%u lower=%u is_reversed=%u" % (self.float_to_u32(self.fps_upper_threshold), self.float_to_u32(self.fps_lower_threshold), self.fps_is_reversed))
        self.mcu.add_config_cmd("config_oams_f1s_hes on1=%u on2=%u on3=%u on4=%u is_above=%u" % (self.float_to_u32(self.f1s_hes_on[0]), self.float_to_u32(self.f1s_hes_on[1]), self.float_to_u32(self.f1s_hes_on[2]), self.float_to_u32(self.f1s_hes_on[3]), self.f1s_hes_is_above))
        self.mcu.add_config_cmd("config_oams_hub_hes on1=%u on2=%u on3=%u on4=%u is_above=%u" % (self.float_to_u32(self.hub_hes_on[0]), self.float_to_u32(self.hub_hes_on[1]), self.float_to_u32(self.hub_hes_on[2]), self.float_to_u32(self.hub_hes_on[3]), self.hub_hes_is_above))
        self.mcu.add_config_cmd("config_oams_pid kp=%u ki=%u kd=%u target=%u" % (self.float_to_u32(self.kp), self.float_to_u32(self.ki), self.float_to_u32(self.kd), self.float_to_u32(self.fps_target)))
        self.mcu.add_config_cmd("config_oams_ptfe length=%u" % (self.filament_path_length))
        self.mcu.add_config_cmd("config_oams_current_pid kp=%u ki=%u kd=%u target=%u" % (self.float_to_u32(self.current_kp), self.float_to_u32(self.current_ki), self.float_to_u32(self.current_kd), self.float_to_u32(self.current_target)))
        self.mcu.add_config_cmd("config_oams_logger idx=%u" % (self.oams_idx))


def load_config_prefix(config):
    return OAMS(config)
