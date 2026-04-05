# Armored Turtle Automated Filament Control
#
# Copyright (C) 2025 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.

# Toolchanging code is inspired and based off klipper-toolchanger project https://github.com/viesturz/klipper-toolchanger
# Originally authored by Viesturs Zariņš and licensed under the GNU General Public License v3.0.
#
# Full license text available at: https://www.gnu.org/licenses/gpl-3.0.html

from __future__ import annotations

import ast
import bisect
import traceback
from configparser import Error as error

from typing import TYPE_CHECKING, Optional, Dict

if TYPE_CHECKING:
    from configfile import ConfigWrapper
    from gcode import GCodeCommand
    from extras.AFC_lane import AFCLane
    from extras.AFC_functions import afcFunction
    from extras.AFC_extruder import AFCExtruder

try: from extras.AFC_utils import ERROR_STR
except: raise error("Error when trying to import AFC_utils.ERROR_STR\n{trace}".format(trace=traceback.format_exc()))

try: from extras.AFC_unit import afcUnit
except: raise error(ERROR_STR.format(import_lib="AFC_unit", trace=traceback.format_exc()))

try: from extras.AFC import State
except: raise error(ERROR_STR.format(import_lib="AFC", trace=traceback.format_exc()))

try: from extras.AFC_lane import AFCLaneState
except: raise error(ERROR_STR.format(import_lib="AFC_lane", trace=traceback.format_exc()))

# Toolchanger status constants
STATUS_UNINITIALIZED = 'uninitialized'
STATUS_INITIALIZING = 'initializing'
STATUS_READY = 'ready'
STATUS_CHANGING = 'changing'
STATUS_ERROR = 'error'

# Detection pin states
DETECT_UNAVAILABLE = -1
DETECT_ABSENT = 0
DETECT_PRESENT = 1

XYZ_TO_INDEX = {'x': 0, 'X': 0, 'y': 1, 'Y': 1, 'z': 2, 'Z': 2}
INDEX_TO_XYZ = 'XYZ'

class AfcToolchanger(afcUnit):
    def __init__(self, config: ConfigWrapper) -> None:
        super().__init__(config)
        self.config = config
        self.type = config.get("type", "Toolchanger")
        firstLeg = '<span class=warning--text>|</span><span class=success--text>_</span>'
        secondLeg = firstLeg + '<span class=warning--text>|</span>'
        self._logo_base  = '<span class=success--text>'
        self._logo_base += 'R  ___ ___ ____\n'
        self._logo_base += 'E |_T_|_T_|  </span><span class=info--text>o</span><span class=success--text> |\n'
        self._logo_base += 'A |       |/ __/\n'
        self._logo_base += 'D |_________/\n'
        self._logo_base += 'Y {first}{second} {first}{second}\n'.format(first=firstLeg, second=secondLeg)
        self._logo_base += '  ' + self.name + '</span>'

        firstLeg_e = '<span class=warning--text>|</span><span class=error--text>_</span>'
        secondLeg_e = firstLeg_e + '<span class=warning--text>|</span>'
        self.logo_error  = '<span class=error--text>'
        self.logo_error += 'E  ___ ___ ____\n'
        self.logo_error += 'R |_X_|_X_|  </span><span class=secondary--text>X</span><span class=error--text> |\n'
        self.logo_error += 'R |       |/ __/\n'
        self.logo_error += 'O |_________/\n'
        self.logo_error += 'R {first}{second} {first}{second}\n'.format(first=firstLeg_e, second=secondLeg_e)
        self.logo_error += '  ' + self.name + '</span>\n'

        # Flipped turtle for detection pin conflicts (multiple tools on shuttle)
        self._logo_conflict  = '<span class=error--text>'
        self._logo_conflict += 'E {first}{second} {first}{second}\n'.format(first=firstLeg_e, second=secondLeg_e)
        self._logo_conflict += 'R  _________\n'
        self._logo_conflict += 'R |       |\\ __\\\n'
        self._logo_conflict += 'O |_X_|_X_|  </span><span class=secondary--text>X</span><span class=error--text> |\n'
        self._logo_conflict += 'R  --- --- ----\n'
        self._logo_conflict += '! ' + self.name + ' CONFLICT</span>\n'
        self.functions: afcFunction = self.printer.load_object(config, 'AFC_functions')
        self.gcode_move = self.printer.load_object(config, 'gcode_move')
        self.gcode_macro = self.printer.load_object(config, 'gcode_macro')

        # Toolchanger config options
        self.uses_axis: str = config.get('uses_axis', 'xyz').lower()
        self.require_tool_present: bool = config.getboolean('require_tool_present', True)
        self.verify_tool_pickup: bool = config.getboolean('verify_tool_pickup', True)
        self.transfer_fan_speed: bool = config.getboolean('transfer_fan_speed', True)

        # Crash detection — monitors detection pins during printing to detect
        # if a tool falls off the shuttle mid-print.
        self.crash_detection_enabled: bool = config.getboolean('crash_detection', False)
        self.crash_mintime: float = config.getfloat('crash_mintime', 0.5, above=0.)
        self._crash_watchdog_interval: float = config.getfloat('crash_watchdog_interval', 0.5, above=0.)
        self._crash_watchdog_threshold: int = config.getint('crash_watchdog_threshold', 2, minval=1)
        self._crash_enable_grace: float = config.getfloat('crash_enable_grace', 2.0, minval=0.)
        self._crash_active = False
        self._crash_watchdog_timer = None
        self._crash_watchdog_errors = 0

        # Dock cooling — turns on part cooling fan for docked tools above
        # a temperature threshold to prevent oozing while parked.
        self.dock_cooling: bool = config.getboolean('dock_cooling', False)
        self.dock_cooling_temp: float = config.getfloat('dock_cooling_temp', 170.0, minval=0.)
        self.dock_cooling_fan_speed: float = config.getfloat('dock_cooling_fan_speed', 1.0, minval=0., maxval=1.0)
        self._dock_cooled_tools: set = set()
        # Timer always registered — per-extruder dock_cooling can enable
        # even when toolchanger-level is off
        self._dock_cooling_timer = self.printer.get_reactor().register_timer(
            self._dock_cooling_tick)
        self.printer.register_event_handler("klippy:ready", self._start_dock_cooling_timer)
        self._crash_enable_time = 0.0

        # Default gcode templates (can be overridden per-tool in AFC_extruder)
        self.default_before_change_gcode = self.gcode_macro.load_template(
            config, 'before_change_gcode', config.get('before_change_gcode', ''))
        self.default_after_change_gcode = self.gcode_macro.load_template(
            config, 'after_change_gcode', config.get('after_change_gcode', ''))
        self.default_pickup_gcode = self.gcode_macro.load_template(
            config, 'pickup_gcode', config.get('pickup_gcode', ''))
        self.default_dropoff_gcode = self.gcode_macro.load_template(
            config, 'dropoff_gcode', config.get('dropoff_gcode', ''))

        # Default params (inherited by tools that don't specify their own)
        self.default_params = {}
        for option in config.get_prefix_options('params_'):
            try:
                self.default_params[option] = ast.literal_eval(config.get(option))
            except ValueError:
                raise config.error(
                    "Option '%s' in section '%s' is not a valid literal" % (
                        option, config.get_name()))
        self.initialize_gcode = self.gcode_macro.load_template(
            config, 'initialize_gcode', config.get('initialize_gcode', ''))
        self.error_gcode = None
        if config.get('error_gcode', None):
            self.error_gcode = self.gcode_macro.load_template(config, 'error_gcode')

        # Native toolchanger state machine
        self.status: str = STATUS_UNINITIALIZED
        self.active_tool: Optional[AFCExtruder] = None
        self.detected_tool: Optional[AFCExtruder] = None
        self.has_detection: bool = False
        self.tools: Dict[int, AFCExtruder] = {}
        self.tool_numbers: list = []
        self.tool_names: list = []
        self.error_message: str = ''

        # State saved during tool change
        self.last_change_gcode_position = None
        self.last_change_gcode_offset = None
        self.last_change_restore_axis = None
        self.last_change_pickup_tool = None

        # Gcode offset transform
        self.gcode_transform = ToolGcodeTransform()

        # tool_probe_endstop integration (for optotap/probe-based tool detection)
        self.tool_probe_endstop = None

        # Fan switcher (created on demand)
        self.fan_switcher = None

        # Register native toolchanger gcode commands
        self.gcode.register_command("SELECT_TOOL",
                                    self.cmd_SELECT_TOOL,
                                    desc="Select active tool")
        self.gcode.register_command("UNSELECT_TOOL",
                                    self.cmd_UNSELECT_TOOL,
                                    desc="Unselect active tool")
        self.gcode.register_command("INITIALIZE_TOOLCHANGER",
                                    self.cmd_INITIALIZE_TOOLCHANGER,
                                    desc="Initialize the toolchanger")
        self.gcode.register_command("ENTER_DOCKING_MODE",
                                    self.cmd_ENTER_DOCKING_MODE,
                                    desc="Enter docking mode")
        self.gcode.register_command("EXIT_DOCKING_MODE",
                                    self.cmd_EXIT_DOCKING_MODE,
                                    desc="Exit docking mode")
        self.gcode.register_command("SET_TOOL_PARAMETER",
                                    self.cmd_SET_TOOL_PARAMETER,
                                    desc="Set a tool parameter at runtime")
        self.gcode.register_command("SAVE_TOOL_PARAMETER",
                                    self.cmd_SAVE_TOOL_PARAMETER,
                                    desc="Save a tool parameter to config")
        self.gcode.register_command("SAVE_TOOL_Z_OFFSET",
                                    self.cmd_SAVE_TOOL_Z_OFFSET,
                                    desc="Save gcode_z_offset for a tool to config")
        self.gcode.register_command("ADJUST_Z_AFTER_TOOL_NOZZLE_HOME",
                                    self.cmd_ADJUST_Z_AFTER_TOOL_NOZZLE_HOME,
                                    desc="Adjust Z position by active tool's Z offset after homing")
        self.gcode.register_command("VERIFY_TOOL_DETECTED",
                                    self.cmd_VERIFY_TOOL_DETECTED,
                                    desc="Verify expected tool is detected on shuttle")
        self.gcode.register_command("START_TOOL_CRASH_DETECTION",
                                    self.cmd_START_TOOL_CRASH_DETECTION,
                                    desc="Enable tool crash detection")
        self.gcode.register_command("STOP_TOOL_CRASH_DETECTION",
                                    self.cmd_STOP_TOOL_CRASH_DETECTION,
                                    desc="Disable tool crash detection")
        self.gcode.register_command("TOOLCHANGER_PREP",
                                    self.cmd_TOOLCHANGER_PREP,
                                    desc="Scan detection pins and report toolchanger status")

        self.printer.register_event_handler("klippy:connect",
                                            self._handle_tc_connect)

        # AFC-specific commands
        self.functions.register_commands(self.afc.show_macros, "AFC_SELECT_TOOL",
                                         self.cmd_AFC_SELECT_TOOL, self.cmd_AFC_SELECT_TOOL_help,
                                         self.cmd_AFC_SELECT_TOOL_options )

        self.functions.register_commands(self.afc.show_macros, "AFC_UNSELECT_TOOL",
                                         self.cmd_AFC_UNSELECT_TOOL, self.cmd_AFC_UNSELECT_TOOL_help)

        self.functions.register_commands(self.afc.show_macros, "AFC_SET_TOOLHEAD_LED",
                                         self.cmd_AFC_SET_TOOLHEAD_LED,
                                         self.cmd_AFC_SET_TOOLHEAD_LED_help,
                                         self.cmd_AFC_SET_TOOLHEAD_LED_options)

    @property
    def logo(self):
        """Dynamic logo with detection pin status for PREP output."""
        ok, detail = self.prep_check()
        if not ok:
            return self._logo_conflict + '\n ' + detail + '\n'
        return self._logo_base + '\n ' + detail + '\n'

    def _handle_tc_connect(self):
        """Install the gcode offset transform on connect and find tool_probe_endstop."""
        self.gcode_transform.next_transform = self.gcode_move.set_move_transform(
            self.gcode_transform, force=True)
        self.tool_probe_endstop = self.printer.lookup_object(
            'AFC_tool_probe_endstop', None)

    def require_fan_switcher(self):
        """Create fan switcher on demand when a tool has a fan configured."""
        if not self.fan_switcher:
            self.fan_switcher = FanSwitcher(self)

    # --- Tool registration and lookup ---

    def register_tool(self, extruder, number):
        """Register an AFC_extruder as a tool with the given number."""
        if number in self.tools:
            self.logger.warning("Toolchanger: replacing tool_number %d" % number)
            self.tool_numbers.remove(number)
            self.tool_names.remove(self.tools[number].name)
        self.tools[number] = extruder
        position = bisect.bisect_left(self.tool_numbers, number)
        self.tool_numbers.insert(position, number)
        self.tool_names.insert(position, extruder.name)
        # Update detection state — check if any tool has a detection pin
        # configured, not whether it has been sampled yet (detect_state
        # starts at -1/UNAVAILABLE until the first callback fires).
        self.has_detection = any(
            t.detect_pin_name is not None or getattr(t, 'tool_probe', None) is not None
            for t in self.tools.values())
        # Register T<n> gcode command
        self._register_t_command(extruder, number)

    def _register_t_command(self, extruder, number):
        """Register T0, T1, etc. gcode commands for direct tool selection."""
        name = 'T%d' % number
        existing = self.gcode.register_command(name, None)
        if existing:
            self.gcode.register_command(name, existing)
        else:
            axis = extruder.t_command_restore_axis
            func = lambda gcmd, n=number, a=axis: self.select_tool(
                gcmd, self.lookup_tool(n), a)
            self.gcode.register_command(name, func, desc='Select tool %d' % number)

    def lookup_tool(self, number):
        return self.tools.get(number)

    def get_selected_tool(self):
        return self.active_tool

    def note_detect_change(self, extruder, eventtime):
        """Called by AFC_extruder when a detection pin changes state."""
        self.detected_tool = self._require_detected_tool()

    def get_status(self, eventtime=None):
        """Return status for webhooks / gcode template context."""
        offset = self.active_tool.get_offset() if self.active_tool else [0., 0., 0.]
        return {
            'name': self.config.get_name(),
            'status': self.status,
            'tool': self.active_tool.name if self.active_tool else None,
            'tool_number': self.active_tool.tool_number if self.active_tool else -1,
            'tool_numbers': self.tool_numbers,
            'tool_names': self.tool_names,
            'active_tool_gcode_x_offset': offset[0],
            'active_tool_gcode_y_offset': offset[1],
            'active_tool_gcode_z_offset': offset[2],
        }

    # --- Native toolchanger gcode commands ---

    def cmd_INITIALIZE_TOOLCHANGER(self, gcmd):
        tool = self._gcmd_tool(gcmd, self.detected_tool)
        self.initialize(tool)

    def cmd_SELECT_TOOL(self, gcmd):
        tool_name = gcmd.get('TOOL', None)
        if tool_name:
            tool = self.afc.tools.get(tool_name)
            if not tool:
                raise gcmd.error("SELECT_TOOL: TOOL=%s not found" % tool_name)
            restore_axis = gcmd.get('RESTORE_AXIS', tool.t_command_restore_axis)
            self.select_tool(gcmd, tool, restore_axis)
            return
        tool_nr = gcmd.get_int('T', None)
        if tool_nr is not None:
            tool = self.lookup_tool(tool_nr)
            if not tool:
                raise gcmd.error("SELECT_TOOL: T%d not found" % tool_nr)
            restore_axis = gcmd.get('RESTORE_AXIS', tool.t_command_restore_axis)
            self.select_tool(gcmd, tool, restore_axis)
            return
        raise gcmd.error("SELECT_TOOL: Either TOOL or T must be specified")

    def cmd_UNSELECT_TOOL(self, gcmd):
        if not self.active_tool:
            return
        restore_axis = gcmd.get('RESTORE_AXIS',
                                self.active_tool.t_command_restore_axis)
        self.select_tool(gcmd, None, restore_axis)

    def cmd_ENTER_DOCKING_MODE(self, gcmd):
        if self.status == STATUS_UNINITIALIZED:
            self.initialize(self.detected_tool)
        if self.status != STATUS_READY:
            raise gcmd.error(
                "Cannot enter docking mode, status is %s" % self.status)
        self.stop_crash_detection()
        self.status = STATUS_CHANGING
        self._save_state("", None)
        self._set_toolchange_transform()

    def cmd_EXIT_DOCKING_MODE(self, gcmd):
        if self.status != STATUS_CHANGING:
            raise gcmd.error(
                "Cannot exit docking mode, status is %s" % self.status)
        self._restore_state_and_transform(self.active_tool)
        if self.active_tool:
            self.start_crash_detection()
        self.status = STATUS_READY

    def _resolve_tool_from_gcmd(self, gcmd):
        """Look up a tool by T=<number> from a gcode command."""
        tn = gcmd.get_int('T', None)
        if tn is None:
            raise gcmd.error("SET_TOOL_PARAMETER/SAVE_TOOL_PARAMETER requires T=<tool_number>")
        tool = self.tools.get(tn)
        if tool is None:
            raise gcmd.error("Tool T%d not found" % tn)
        return tool

    def cmd_SET_TOOL_PARAMETER(self, gcmd):
        """Set a tool parameter at runtime.
        Usage: SET_TOOL_PARAMETER T=<num> PARAMETER=<name> VALUE=<value>
        """
        tool = self._resolve_tool_from_gcmd(gcmd)
        param = gcmd.get('PARAMETER')
        raw_value = gcmd.get('VALUE')
        try:
            value = ast.literal_eval(raw_value)
        except (ValueError, SyntaxError):
            value = raw_value
        tool.params[param] = value

    def cmd_SAVE_TOOL_PARAMETER(self, gcmd):
        """Save a tool parameter to config file.
        Usage: SAVE_TOOL_PARAMETER T=<num> PARAMETER=<name>
        """
        tool = self._resolve_tool_from_gcmd(gcmd)
        param = gcmd.get('PARAMETER')
        if param not in tool.params:
            raise gcmd.error("Parameter '%s' not found on tool T%d" % (
                param, tool.tool_number))
        self.functions.ConfigRewrite(
            tool.fullname, param, tool.params[param], '')

    def cmd_SAVE_TOOL_Z_OFFSET(self, gcmd):
        """Save gcode_z_offset for a tool to config.
        Usage: SAVE_TOOL_Z_OFFSET T=<num> Z=<offset>
        Returns the old value for display.
        """
        tool = self._resolve_tool_from_gcmd(gcmd)
        new_z = gcmd.get_float('Z')
        old_z = tool.gcode_z_offset
        tool.gcode_z_offset = new_z
        cal_msg = (
            f"\n gcode_z_offset: New: {new_z:.4f} Old: {old_z:.4f}")
        self.functions.ConfigRewrite(
            tool.fullname, "gcode_z_offset", "%.4f" % new_z, cal_msg)
        gcmd.respond_info(
            "%s gcode_z_offset: %.4f (was %.4f)" % (
                tool.name, new_z, old_z))

    def cmd_ADJUST_Z_AFTER_TOOL_NOZZLE_HOME(self, gcmd):
        """Adjust Z position by the active tool's gcode_z_offset after nozzle homing.
        Usage: ADJUST_Z_AFTER_TOOL_NOZZLE_HOME
        """
        tool = self.active_tool
        if not tool:
            raise gcmd.error("ADJUST_Z_AFTER_TOOL_NOZZLE_HOME - no active tool")
        z_offset = tool.gcode_z_offset
        if z_offset != 0.0:
            self.logger.info(
                "Adjusting Z position after homing by %.4f" % z_offset)
            toolhead = self.printer.lookup_object('toolhead')
            pos = list(toolhead.get_position())
            pos[2] += z_offset
            toolhead.set_position(pos)

    # --- Core tool change engine ---

    def initialize(self, select_tool=None):
        """Initialize toolchanger, optionally selecting a tool."""
        if self.status == STATUS_CHANGING:
            raise Exception('Cannot initialize while changing tools')
        should_run_init = self.status != STATUS_INITIALIZING
        extra_context = {
            'dropoff_tool': None,
            'pickup_tool': select_tool.name if select_tool else None,
        }
        try:
            if should_run_init:
                self.status = STATUS_INITIALIZING
                self._run_gcode('initialize_gcode', self.initialize_gcode, extra_context)

            inferred = select_tool
            if inferred is None and self.has_detection:
                # Brief pause to let button callbacks settle.  Docked tools
                # have pins HIGH which triggers a callback (setting ABSENT).
                # The shuttle tool's pin stays LOW — no callback fires, so it
                # keeps the default PRESENT.  The pause ensures docked-tool
                # callbacks have arrived before we check.
                reactor = self.printer.get_reactor()
                pause_end = reactor.monotonic() + 0.5
                reactor.pause(pause_end)
                inferred = self._require_detected_tool()

            # Safety: never trust an inferred tool unless its detection pin
            # positively confirms it is on the shuttle.  This prevents
            # activating a tool from stale state or unsampled pins.
            if inferred and self.has_detection:
                if inferred.detect_state != DETECT_PRESENT:
                    self.logger.info(
                        "Rejecting inferred tool %s during init: "
                        "detection pin is %d (not PRESENT)"
                        % (inferred.name, inferred.detect_state))
                    inferred = None

            if self.require_tool_present and inferred is None and self.has_detection:
                if should_run_init:
                    self.status = STATUS_ERROR
                    self.error_message = 'No tool detected on shuttle during initialization'
                    raise self.gcode.error(
                        '%s: %s' % (self.config.get_name(), self.error_message))

            if inferred or not self.require_tool_present or self.has_detection:
                self._configure_toolhead_for_tool(inferred)
                if inferred:
                    self._run_gcode('after_change_gcode',
                                   inferred.after_change_gcode, extra_context)
                    self.gcode_transform.tool = inferred

            if should_run_init:
                if self.status == STATUS_INITIALIZING:
                    self.status = STATUS_READY
                    self.logger.info(
                        '%s initialized, active %s' % (
                            self.config.get_name(),
                            self.active_tool.name if self.active_tool else None))
                else:
                    raise self.gcode.error(
                        '%s failed to initialize: %s' % (
                            self.config.get_name(), self.error_message))
        except Exception:
            # Ensure status doesn't stay stuck at INITIALIZING on unexpected errors
            if should_run_init and self.status == STATUS_INITIALIZING:
                self.status = STATUS_ERROR
                self.error_message = 'Exception during initialization'
            raise

    def prep_check(self):
        """Scan detection pins and report toolchanger status.

        Returns (ok, message) tuple. Checks:
        - Detection pin conflicts (multiple tools reading PRESENT)
        - Tool probe conflicts (multiple probes triggered)
        - Identifies which tool is on the shuttle (if any)
        - Reports any tools with unavailable detection (-1)
        """
        if not self.has_detection:
            return True, "No detection pins configured — skipping prep check"

        # Let detection pin callbacks settle
        reactor = self.printer.get_reactor()
        reactor.pause(reactor.monotonic() + 0.3)

        present = []
        absent = []
        unavailable = []

        for tool in self.tools.values():
            if tool.detect_state == DETECT_PRESENT:
                present.append(tool)
            elif tool.detect_state == 0:  # ABSENT
                absent.append(tool)
            else:
                unavailable.append(tool)

        parts = []

        if len(present) > 1:
            names = ', '.join(t.name for t in present)
            msg = (
                "CONFLICT: Multiple tools detected on shuttle: %s. "
                "Check detection pins — only one tool should read PRESENT."
                % names)
            self.logger.error(msg)
            return False, msg

        if len(present) == 1:
            parts.append("Shuttle: %s" % present[0].name)
        else:
            parts.append("Shuttle: empty")

        if absent:
            parts.append("Docked: %s" % ', '.join(t.name for t in absent))

        if unavailable:
            names = ', '.join(t.name for t in unavailable)
            parts.append("No detection: %s" % names)

        msg = " | ".join(parts)
        return True, msg

    def cmd_TOOLCHANGER_PREP(self, gcmd):
        """Run toolchanger prep check — scan detection pins and report status."""
        ok, msg = self.prep_check()
        gcmd.respond_info(msg)

    def select_tool(self, gcmd, tool, restore_axis):
        """Core tool change: dropoff current, pickup new, manage offsets."""
        if self.status == STATUS_UNINITIALIZED:
            self.initialize(self.detected_tool)
        if self.status != STATUS_READY:
            raise gcmd.error(
                "Cannot select tool, status is %s: %s" % (
                    self.status, self.error_message))

        if self.active_tool == tool:
            self.logger.info(
                'Tool %s already selected' % (tool.name if tool else None))
            return

        try:
            # Stop crash detection before tool change
            self.stop_crash_detection()
            self._ensure_homed(gcmd)
            self.status = STATUS_CHANGING
            self._save_state(restore_axis, tool)

            start_position = self._position_with_tool_offset(
                self.last_change_gcode_position, tool)
            extra_context = {
                'dropoff_tool': self.active_tool.name if self.active_tool else None,
                'pickup_tool': tool.name if tool else None,
                'start_position': self._position_to_xyz(start_position, 'xyz'),
                'restore_position': self._position_to_xyz(start_position, restore_axis),
            }

            before_gcode = (self.active_tool.before_change_gcode
                           if self.active_tool else self.default_before_change_gcode)
            self._run_gcode('before_change_gcode', before_gcode, extra_context)
            self._set_toolchange_transform()

            if self.active_tool:
                self._run_gcode('tool.dropoff_gcode',
                               self.active_tool.dropoff_gcode, extra_context)

            self._configure_toolhead_for_tool(tool)
            if tool is not None:
                self._run_gcode('tool.pickup_gcode',
                               tool.pickup_gcode, extra_context)
                # Detection is now verified inline by VERIFY_TOOL_DETECTED
                # in the pickup_gcode template (at the 'verify' path step,
                # while still at the dock — before restore moves).
                self._run_gcode('after_change_gcode',
                               tool.after_change_gcode, extra_context)

            self._restore_state_and_transform(tool)
            self.status = STATUS_READY
            if tool:
                self.logger.info('Selected %s (tool_number %d)' % (
                    tool.name, tool.tool_number))
                # Re-enable crash detection after successful pickup
                self.start_crash_detection()
            else:
                self.logger.info('Tool unselected')

        except Exception:
            if self.status == STATUS_ERROR:
                pass  # process_error already handled recovery
            elif self.status == STATUS_CHANGING:
                # Unexpected exception — force status back so the
                # toolchanger doesn't get permanently stuck.
                self.status = STATUS_ERROR
                self.error_message = 'Unexpected error during tool change'
            raise

    def system_Test(self, cur_lane: AFCLane, delay: float, assignTcmd: str, enable_movement: bool):
        msg = ""
        succeeded = True

        # Track on-shuttle count across lanes — reset on first lane
        first_lane = next(iter(self.lanes.values()), None)
        if first_lane is not None and cur_lane.name == first_lane.name:
            self._prep_on_shuttle = []

        # Check detection pin for this lane's extruder
        extruder = cur_lane.extruder_obj
        if extruder is not None and extruder.on_shuttle():
            msg += "<span class=info--text>ON SHUTTLE</span> "
            self._prep_on_shuttle.append(cur_lane.name)

        # After last lane, check for conflicts
        last_lane = list(self.lanes.values())[-1] if self.lanes else None
        if last_lane is not None and cur_lane.name == last_lane.name:
            if len(self._prep_on_shuttle) > 1:
                names = ', '.join(self._prep_on_shuttle)
                self.logger.error(
                    "CONFLICT: Multiple tools detected on shuttle: %s. "
                    "Check detection pins." % names)

        if not cur_lane.prep_state:
            if not cur_lane.load_state:
                self.lane_not_ready(cur_lane)
                msg += 'EMPTY READY FOR SPOOL'
            else:
                self.lane_fault(cur_lane)
                cur_lane.do_enable(False)
                msg = '<span class=error--text>CHECK FILAMENT Prep: False - Load: True</span>'
                succeeded = False
        else:
            self.lane_loaded(cur_lane)
            msg += "<span class=success--text>LOCKED</span>"
            if not cur_lane.load_state:
                msg += "<span class=error--text> NOT LOADED</span>"
                self.lane_not_ready(cur_lane)
                succeeded = False
            else:
                cur_lane.status = AFCLaneState.LOADED
                msg += "<span class=success--text> AND LOADED</span>"
                self.lane_illuminate_spool(cur_lane)

                if (cur_lane.tool_loaded
                    and cur_lane.extruder_obj.lane_loaded == cur_lane.name):
                    # Toolchangers can be direct-fed (no buffer/sensors),
                    # so also trust on_shuttle() detection pin as validation
                    tool_ready = (
                        cur_lane.get_toolhead_pre_sensor_state()
                        or cur_lane.extruder_obj.tool_start == "buffer"
                        or cur_lane.extruder_obj.tool_end_state
                        or cur_lane.extruder_obj.on_shuttle()
                    )
                    if tool_ready:
                        cur_lane.sync_to_extruder()
                        on_shuttle = ""
                        if cur_lane.extruder_obj.tc_unit_obj:
                            on_shuttle = (
                                " and toolhead on shuttle"
                                if cur_lane.extruder_obj.on_shuttle()
                                else ""
                            )
                        msg += f"<span class=primary--text> in ToolHead{on_shuttle}</span>"

                        if (cur_lane.extruder_obj.tool_start == "buffer"
                            and (not self.afc.homing_enabled
                                 or not self.enable_buffer_tool_check)):
                            msg += "<span class=warning--text>\n Ram sensor enabled, confirm tool is loaded</span>"

                        if self.afc.current == cur_lane.name:
                            self.afc.spool.set_active_spool(cur_lane.spool_id)
                            self.lane_tool_loaded(cur_lane)
                            cur_lane.status = AFCLaneState.TOOLED
                        else:
                            self.lane_tool_loaded_idle(cur_lane)

                        cur_lane.enable_buffer()
                    else:
                        self.afc.error.fix('toolhead', cur_lane)

        if assignTcmd:
            self.afc.function.TcmdAssign(cur_lane)
        cur_lane.send_lane_data()
        cur_lane.do_enable(False)
        self.logger.raw('{lane_name} tool cmd: {tcmd:3} {msg}'.format(
            lane_name=cur_lane.name, tcmd=cur_lane.map, msg=msg))
        cur_lane.set_afc_prep_done()
        return succeeded

    # --- Private helper methods for tool change ---

    def _save_state(self, restore_axis, tool):
        """Save gcode state before tool change. Offsets are zeroed during change."""
        gcode_status = self.gcode_move.get_status()
        self.gcode.run_script_from_command("SAVE_GCODE_STATE NAME=_toolchange_state")
        self.last_change_pickup_tool = tool
        self.last_change_gcode_position = list(gcode_status['gcode_position'])
        self.last_change_gcode_offset = gcode_status['homing_origin']
        self.last_change_restore_axis = restore_axis

    def _set_toolchange_transform(self):
        """Zero out gcode offsets during tool change so moves use toolhead coords."""
        self.gcode_transform.tool = None
        self.gcode_move.reset_last_position()
        self.gcode.run_script_from_command("SET_GCODE_OFFSET X=0.0 Y=0.0 Z=0.0")

    def _restore_state_and_transform(self, tool):
        """Restore gcode state and apply new tool's offset transform."""
        self.gcode_transform.tool = tool
        self.gcode_move.reset_last_position()
        self.gcode.run_script_from_command(
            "RESTORE_GCODE_STATE NAME=_toolchange_state MOVE=0")
        self.last_change_gcode_offset = None
        if self.last_change_restore_axis:
            self._restore_axis(
                self.last_change_gcode_position, self.last_change_restore_axis)
            self.gcode.run_script_from_command(
                "RESTORE_GCODE_STATE NAME=_toolchange_state MOVE=0")

    def _restore_axis(self, position, axis):
        """Move to saved position on specified axes after tool change."""
        pos = self._position_with_tool_offset(position, None)
        self.gcode.run_script_from_command("G90")
        self.gcode_move.cmd_G1(
            self.gcode.create_gcode_command(
                "G0", "G0", self._position_to_xyz(pos, axis)))

    def _position_to_xyz(self, position, axis):
        """Convert position list to dict with X/Y/Z keys for given axes."""
        result = {}
        for i in axis:
            index = XYZ_TO_INDEX[i]
            result[INDEX_TO_XYZ[index]] = position[index]
        return result

    def _position_with_tool_offset(self, position, tool):
        """Compute gcode position accounting for saved offset and tool offset."""
        result = []
        for i in range(3):
            v = position[i]
            if self.last_change_gcode_offset is not None:
                v += self.last_change_gcode_offset[i]
            if tool:
                offset = tool.get_offset()
                v += offset[i]
            result.append(v)
        result.extend(position[3:])
        return result

    def _configure_toolhead_for_tool(self, tool):
        """Deactivate old tool, activate new tool's extruder/fan/stepper/probe."""
        if self.active_tool:
            self.active_tool.deactivate_tool()
        self.active_tool = tool
        # Switch active tool probe for optotap/tool_probe setups
        if self.tool_probe_endstop:
            probe = getattr(tool, 'tool_probe', None) if tool else None
            self.tool_probe_endstop.set_active_probe(probe)
        if self.active_tool:
            self.active_tool.activate_tool()

    def _run_gcode(self, name, template, extra_context):
        """Run a gcode template with tool/toolchanger context."""
        curtime = self.printer.get_reactor().monotonic()
        context = {
            **template.create_template_context(),
            'tool': self.active_tool.get_tool_status(curtime) if self.active_tool else {},
            'toolchanger': self.get_status(curtime),
            **extra_context,
        }
        template.run_gcode_from_command(context)

    def _ensure_homed(self, gcmd):
        """Check that required axes are homed before tool change."""
        if not self.uses_axis:
            return
        toolhead = self.printer.lookup_object('toolhead')
        curtime = self.printer.get_reactor().monotonic()
        homed = toolhead.get_kinematics().get_status(curtime)['homed_axes']
        needs_homing = any(a not in homed for a in self.uses_axis)
        if not needs_homing:
            return
        toolhead.wait_moves()
        curtime = self.printer.get_reactor().monotonic()
        homed = toolhead.get_kinematics().get_status(curtime)['homed_axes']
        axis_to_home = [a for a in self.uses_axis if a not in homed]
        if not axis_to_home:
            return
        raise gcmd.error(
            "Cannot perform toolchange, axis not homed. Required: %s, homed: %s" % (
                self.uses_axis, homed))

    def _require_detected_tool(self):
        """Find which tool is detected via detection pins.

        Returns the tool only when exactly one detection pin reads PRESENT.
        Returns None if zero or multiple tools appear detected (the latter
        typically happens when pins have not been sampled yet at startup).
        """
        detected = None
        detected_count = 0
        for tool in self.tools.values():
            if tool.detect_state == DETECT_PRESENT:
                detected = tool
                detected_count += 1
        if detected_count != 1:
            return None
        return detected

    def _validate_detected_tool(self, expected, raise_error):
        """Verify the expected tool is actually detected."""
        actual = self._require_detected_tool()
        if actual != expected:
            expected_name = expected.name if expected else "None"
            actual_name = actual.name if actual else "None"
            message = "Expected tool %s but detected %s" % (expected_name, actual_name)
            self.process_error(raise_error, message)

    def cmd_VERIFY_TOOL_DETECTED(self, gcmd):
        """Inline gcode command to verify tool detection at the dock.

        Called from within pickup_gcode at the 'verify' point in the path,
        BEFORE restore moves.  This matches viesturz/klipper-toolchanger
        behaviour: if pickup fails, error fires while still at the dock.
        """
        if not self.has_detection:
            return
        # During a tool change, verify the tool being picked up
        expected = (self.last_change_pickup_tool
                    if self.status == STATUS_CHANGING
                    else self.active_tool)
        if expected is None:
            return
        toolhead = self.printer.lookup_object('toolhead')
        reactor = self.printer.get_reactor()
        # Wait for queued moves to physically complete
        toolhead.wait_moves()
        # Brief pause to let detection pin edge callbacks settle
        reactor.pause(reactor.monotonic() + 0.2)
        self._validate_detected_tool(expected, gcmd.error)

    def process_error(self, raise_error, message):
        """Handle toolchanger errors with proper state recovery.

        Matches the KTC error handling pattern:
        1. Set ERROR status
        2. Restore gcode state so offsets aren't zeroed
        3. Run error_gcode (e.g. M112, safe Y move) with position context
        4. Raise the error
        """
        is_inside_toolchange = self.status == STATUS_CHANGING
        self.status = STATUS_ERROR
        self.error_message = message

        if self.error_gcode:
            try:
                extra_context = {}
                if self.last_change_gcode_position is not None:
                    start_position = self._position_with_tool_offset(
                        self.last_change_gcode_position,
                        self.last_change_pickup_tool)
                    extra_context = {
                        'start_position': self._position_to_xyz(
                            start_position, 'xyz'),
                        'restore_position': self._position_to_xyz(
                            start_position,
                            self.last_change_restore_axis or 'XYZ'),
                        'pickup_tool': self.last_change_pickup_tool,
                    }
                    # Restore gcode state without moving so error_gcode
                    # can run PAUSE and capture the correct resume position.
                    self.gcode.run_script_from_command(
                        "RESTORE_GCODE_STATE NAME=_toolchange_state MOVE=0")
                self._run_gcode('error_gcode', self.error_gcode, extra_context)
            except Exception as e:
                self.logger.error(
                    "error_gcode failed (original error: %s): %s" % (message, e))

        if raise_error:
            raise raise_error(message)

    # ---- Crash detection ----

    def start_crash_detection(self):
        """Enable crash detection — monitors detection pins for tool loss.

        Called automatically after successful tool pickup. Can also be called
        manually via START_TOOL_CRASH_DETECTION gcode.
        """
        if not self.crash_detection_enabled or not self.has_detection:
            return
        if not self.active_tool:
            return
        self._crash_watchdog_errors = 0
        self._crash_enable_time = self.printer.get_reactor().monotonic()
        self._crash_active = True
        # Start watchdog timer
        if self._crash_watchdog_timer is None:
            self._crash_watchdog_timer = self.printer.get_reactor().register_timer(
                self._crash_watchdog_tick,
                self.printer.get_reactor().monotonic() + self._crash_watchdog_interval)
        self.logger.info("tool_crash: enabled")

    def stop_crash_detection(self):
        """Disable crash detection — called before tool change operations.

        Called automatically when select_tool() begins (STATUS_CHANGING).
        Can also be called manually via STOP_TOOL_CRASH_DETECTION gcode.
        """
        self._crash_active = False
        self._crash_watchdog_errors = 0
        if self._crash_watchdog_timer is not None:
            self.printer.get_reactor().unregister_timer(self._crash_watchdog_timer)
            self._crash_watchdog_timer = None
        self.logger.info("tool_crash: disabled")

    def _crash_watchdog_tick(self, eventtime):
        """Periodic watchdog check — verify active tool is still detected."""
        if not self._crash_active or not self.active_tool:
            self._crash_watchdog_timer = None
            return self.printer.get_reactor().NEVER

        # Grace period after enable
        if eventtime - self._crash_enable_time < self._crash_enable_grace:
            return eventtime + self._crash_watchdog_interval

        # Skip during tool changes or initialization
        if self.status in (STATUS_CHANGING, STATUS_INITIALIZING, STATUS_UNINITIALIZED):
            return eventtime + self._crash_watchdog_interval

        # Check if active tool is still detected.
        # Uses detection pin (Cartographer/Beacon) OR tool probe (Optotap/Tap)
        # depending on what the extruder has configured.
        active = self.active_tool
        tool_present = self._is_tool_present(active)
        if not tool_present:
            self._crash_watchdog_errors += 1
            if self._crash_watchdog_errors >= self._crash_watchdog_threshold:
                self._do_crash(
                    "tool_crash: watchdog detected loss of %s" % active.name,
                    eventtime)
                return self.printer.get_reactor().NEVER
        else:
            self._crash_watchdog_errors = 0

        return eventtime + self._crash_watchdog_interval

    def _is_tool_present(self, tool):
        """Check if a tool is present on the shuttle.

        Uses whichever detection method the tool has configured:
        - detection_pin: checks detect_state (Cartographer/Beacon setups)
        - tool_probe: queries the probe endstop (Optotap/Tap setups)

        :param tool: AFCExtruder tool object
        :return: True if tool is detected on shuttle
        """
        if tool is None:
            return False

        # Check detection pin first (Cartographer/Beacon)
        if tool.detect_pin_name is not None:
            return tool.detect_state == DETECT_PRESENT

        # Check tool probe (Optotap/Tap) — query the endstop directly
        if tool.tool_probe is not None:
            try:
                toolhead = self.printer.lookup_object('toolhead')
                print_time = toolhead.get_last_move_time()
                triggered = tool.tool_probe.mcu_probe.query_endstop(print_time)
                # For tool probes: NOT triggered = tool present (probe is open
                # when tool is on shuttle, triggered when touching surface)
                return not triggered
            except Exception:
                return True  # Assume present on query failure

        # No detection method configured
        return True

    def _do_crash(self, message, eventtime):
        """Handle a detected tool crash — disable detection and error."""
        self._crash_active = False
        if self._crash_watchdog_timer is not None:
            self.printer.get_reactor().unregister_timer(self._crash_watchdog_timer)
            self._crash_watchdog_timer = None
        self.logger.error(message)

        # Use error_gcode if configured, otherwise shutdown
        if self.error_gcode:
            try:
                self.printer.get_reactor().register_callback(
                    lambda _: self._run_gcode('error_gcode', self.error_gcode, {}),
                    eventtime + self.crash_mintime)
            except Exception as e:
                self.logger.error("crash gcode failed: %s" % e)
                self.printer.invoke_shutdown(message)
        else:
            self.printer.invoke_shutdown(message)

    def cmd_START_TOOL_CRASH_DETECTION(self, gcmd):
        """Enable tool crash detection manually."""
        self.start_crash_detection()

    def cmd_STOP_TOOL_CRASH_DETECTION(self, gcmd):
        """Disable tool crash detection manually."""
        self.stop_crash_detection()

    # ---- Dock cooling ----

    def _start_dock_cooling_timer(self):
        """Start the dock cooling polling timer."""
        if self._dock_cooling_timer is not None:
            reactor = self.printer.get_reactor()
            reactor.update_timer(self._dock_cooling_timer, reactor.NOW + 5.0)

    def _dock_cooling_tick(self, eventtime):
        """Periodic check: cool any docked tool above temp threshold."""
        for tool in self.tools.values():
            # Per-extruder override: True/False overrides toolchanger default
            tool_dc = getattr(tool, 'dock_cooling', None)
            if tool_dc is False:
                self._stop_dock_cooling_fan(tool)
                continue
            if tool_dc is None and not self.dock_cooling:
                continue

            if tool == self.active_tool:
                # Tool is on shuttle — stop cooling if it was running
                self._stop_dock_cooling_fan(tool)
                continue

            # Tool is docked — check temperature
            try:
                heater = self.printer.lookup_object(tool.name).get_heater()
                current_temp, _ = heater.get_temp(eventtime)
            except Exception:
                continue

            if current_temp >= self.dock_cooling_temp:
                self._start_dock_cooling_fan(tool, current_temp)
            else:
                self._stop_dock_cooling_fan(tool)

        return eventtime + 5.0

    def _start_dock_cooling_fan(self, tool, current_temp):
        """Turn on part cooling fan for a docked tool."""
        if tool.tool_number in self._dock_cooled_tools:
            return  # Already cooling
        fan_name = getattr(tool, 'fan_name', None)
        if not fan_name:
            return
        try:
            self.gcode.run_script_from_command(
                "SET_FAN_SPEED FAN=%s SPEED=%.2f" % (fan_name, self.dock_cooling_fan_speed))
            self._dock_cooled_tools.add(tool.tool_number)
            self.logger.info(
                "Dock cooling: %s fan on at %.0f%% (temp %.1fC > %.1fC)"
                % (tool.name, self.dock_cooling_fan_speed * 100,
                   current_temp, self.dock_cooling_temp))
        except Exception as e:
            self.logger.warning("Dock cooling: failed to start fan for %s: %s" % (tool.name, e))

    def _stop_dock_cooling_fan(self, tool):
        """Turn off dock cooling fan for a tool."""
        if tool.tool_number not in self._dock_cooled_tools:
            return
        self._dock_cooled_tools.discard(tool.tool_number)
        fan_name = getattr(tool, 'fan_name', None)
        if not fan_name:
            return
        try:
            self.gcode.run_script_from_command(
                "SET_FAN_SPEED FAN=%s SPEED=0" % fan_name)
            self.logger.info("Dock cooling: %s fan off" % tool.name)
        except Exception as e:
            self.logger.warning("Dock cooling: failed to stop fan for %s: %s" % (tool.name, e))

    def _gcmd_tool(self, gcmd, default=None):
        """Resolve tool from TOOL= or T= gcode parameters."""
        tool_name = gcmd.get('TOOL', None)
        tool_number = gcmd.get_int('T', None)
        if tool_name:
            return self.afc.tools.get(tool_name)
        if tool_number is not None:
            tool = self.lookup_tool(tool_number)
            if not tool:
                raise gcmd.error('Tool #%d is not assigned' % tool_number)
            return tool
        return default

    def _increase_unselect(self):
        """
        Helper function to lookup current selected extruder and increase tool unselected count
        """
        current_extruder = self.afc.function.get_current_extruder_obj()
        if current_extruder:
            current_extruder.estats.tool_unselected.increase_count()

    cmd_AFC_SELECT_TOOL_help = "Select specified tool"
    cmd_AFC_SELECT_TOOL_options = {
        "TOOL": {"type": "string", "default": "extruder"}
    }
    def cmd_AFC_SELECT_TOOL(self, gcmd: GCodeCommand):
        """
        Selects tool based on passed in extruder name

        Usage
        -----
        `AFC_SELECT_TOOL TOOL=<extruder_name>`

        Example
        -----
        ```
        AFC_SELECT_TOOL TOOL=extruder1
        ```
        """
        tool_key = gcmd.get("TOOL")
        tool = self.afc.tools.get(tool_key)

        if tool:
            if tool.tc_lane is not None:
                self.tool_swap(tool.tc_lane)
            else:
                self.logger.error(f"Tool '{tool_key}' does not have a valid 'tc_lane' attribute.")
        else:
            self.logger.error(f"Key:{tool_key} invalid for TOOL")

    cmd_AFC_UNSELECT_TOOL_help = "Unselects and docks current tool on shuttle"
    def cmd_AFC_UNSELECT_TOOL(self, gcmd: GCodeCommand):
        """
        Unselects current tool using native AFC toolchanger engine.

        Usage
        -----
        `AFC_UNSELECT_TOOL`

        Example
        -----
        ```
        AFC_UNSELECT_TOOL
        ```
        """
        self._increase_unselect()
        current_extruder = self.afc.function.get_current_extruder_obj()
        if (current_extruder
            and current_extruder.custom_unselect):
            self.logger.info(f"Running custom unselect: {current_extruder.custom_unselect}")
            self.afc.gcode.run_script_from_command(f"{current_extruder.custom_unselect}")
        else:
            # Use native UNSELECT_TOOL
            self.cmd_UNSELECT_TOOL(gcmd)

        lane_obj = self.afc.function.get_current_lane_obj()
        if lane_obj:
            if (lane_obj.prep_state
                and lane_obj.load_state):
                if lane_obj.tool_loaded:
                    lane_obj.unit_obj.lane_tool_loaded_idle(lane_obj)
                else:
                    lane_obj.unit_obj.lane_tool_unloaded(lane_obj)
            else:
                lane_obj.extruder_obj.set_status_led(lane_obj.led_tool_unloaded)

        self.afc.spool.set_active_spool('')

    def tool_swap(self, lane: AFCLane, set_start_time=True):
        """
        Perform a tool swap operation for the specified lane.

        Uses the native AFC toolchanger engine to handle the physical tool
        swap (save state, run dropoff/pickup gcode, apply offsets).

        :param lane: The lane object whose extruder/toolhead should be activated.
        :param set_start_time: Set true to set a starting time for afcDeltaTime.
        """
        # Units can suppress timing during their own load sequences
        if getattr(self.afc, '_suppress_tool_swap_timer', False):
            set_start_time = False
        if set_start_time:
            self.afc.afcDeltaTime.set_start_time()

        self.afc.current_state = State.TOOL_SWAP
        self._increase_unselect()
        self.afc.function.log_toolhead_pos("Before toolswap: ")

        try:
            self.afc.afcDeltaTime.log_with_time("Performing tool swap")

            if lane.extruder_obj.custom_tool_swap:
                self.logger.info(f"Running custom select: {lane.extruder_obj.custom_tool_swap}")
                self.afc.gcode.run_script_from_command(f"{lane.extruder_obj.custom_tool_swap}")
            else:
                # Use native toolchanger engine: SELECT_TOOL targets the physical
                # extruder via its tool_number (dock position), not the lane's T-map.
                tool_index = lane.extruder_obj.tool_number
                if tool_index < 0:
                    raise self.afc.gcode.error(
                        "Cannot swap to %s: extruder %s has no tool_number configured"
                        % (lane.name, lane.extruder_obj.name))

                self.afc.gcode.run_script_from_command(
                    'SELECT_TOOL T={}'.format(tool_index))

            # Sync AFC lane state AFTER tool swap completes and position is restored.
            # Must happen here (not inside select_tool) because _handle_activate_extruder
            # can trigger buffer/stepper/LED changes that interfere with the gcode position
            # restore sequence inside select_tool.
            self.afc.function._handle_activate_extruder(0, lane=lane)

            self.afc.toolhead.wait_moves()
            self.afc.afcDeltaTime.log_with_time("Tool swap done", debug=False)
            if self.afc.afc_stats.average_tool_swap_time:
                self.afc.afc_stats.average_tool_swap_time.average_time(
                    self.afc.afcDeltaTime.delta_time)
            # Re-capture AFC's position state from gcode_move after the swap.
            # SELECT_TOOL already restored the gcode state and applied the new
            # tool's offset transform, so these values are now correct for the
            # new tool's coordinate system.
            self.afc.base_position    = list(self.afc.gcode_move.base_position)
            self.afc.homing_position  = list(self.afc.gcode_move.homing_position)
            self.afc.last_gcode_position = list(self.afc.gcode_move.last_position)

            self.afc.function.log_toolhead_pos("After toolswap: ")
            lane.extruder_obj.estats.tool_selected.increase_count()
        finally:
            self.afc.current_state = State.IDLE

    cmd_AFC_SET_TOOLHEAD_LED_help = "Turns on leds for toolhead specified by mapping, does not affect status led if status_led_idx variable is provided"
    cmd_AFC_SET_TOOLHEAD_LED_options = {
        "MAP": {"type": "string", "default": "T0"},
        "TURN_ON": {"type": "int", "default": 1}
    }
    def cmd_AFC_SET_TOOLHEAD_LED(self, gcmd: GCodeCommand):
        """
        Macro call to set nozzle led in toolhead based on lane/tool mapping. Led config name needs to be
        set to AFC_extruder `led_name` variable. Status led in toolhead will not be affected if `status_led_idx`
        is set in AFC_extruder config. If `nozzle_led_idx` is set in AFC_extruder configuration then just
        those leds will be turned on. If `nozzle_led_inx` is not provided then all leds not in defined in
        `status_led_idx` will be turned on.

        `MAP` - is the mapped toolhead to set. ex(T0,T1,etc)

        `TURN_ON` - set to 1 to turn on leds, set to 0 to turn off leds. If not supplied, defaults to 1

        Usage
        -----
        `AFC_SET_TOOLHEAD_LED MAP=<T(n) mapping> TURN_ON=<0/1>`

        Example
        -----
        ```
        AFC_SET_TOOLHEAD_LED MAP=T2 TURN_ON=1
        ```

        """
        mapping = gcmd.get("MAP")
        state = gcmd.get_int("TURN_ON", 1)

        lane_obj = self.afc.function.get_lane_by_map(mapping)
        if lane_obj is None:
            error_string = f"{mapping} mapping not found when trying to set toolhead led"
            self.logger.error(error_string)
            raise gcmd.error(error_string)

        success, error_string = lane_obj.extruder_obj.set_print_leds(state)

        if not success:
            raise gcmd.error(error_string)

class ToolGcodeTransform:
    """Applies per-tool XYZ gcode offsets during moves."""
    def __init__(self):
        self.next_transform = None
        self.tool = None  # AFCExtruder or None

    def move(self, newpos, speed):
        if not self.tool:
            return self.next_transform.move(newpos, speed)
        offset = self.tool.get_offset()
        transformed = [
            newpos[0] + offset[0],
            newpos[1] + offset[1],
            newpos[2] + offset[2],
        ] + newpos[3:]
        return self.next_transform.move(transformed, speed)

    def get_position(self):
        base_pos = self.next_transform.get_position()
        if not self.tool:
            return base_pos
        offset = self.tool.get_offset()
        return [
            base_pos[0] - offset[0],
            base_pos[1] - offset[1],
            base_pos[2] - offset[2],
        ] + base_pos[3:]


class FanSwitcher:
    """Manages fan switching between tools during tool changes."""
    def __init__(self, toolchanger):
        self.toolchanger = toolchanger
        self.printer = toolchanger.printer
        self.gcode = toolchanger.gcode
        self.active_fan = None
        self.pending_speed = None
        self.transfer_fan_speed = toolchanger.transfer_fan_speed
        # Register M106/M107 for multi-tool fan control
        self.gcode.register_command("M106", self.cmd_M106)
        self.gcode.register_command("M107", self.cmd_M107)

    def activate_fan(self, fan):
        """Switch active fan to the given fan_generic object."""
        if self.active_fan == fan or not self.transfer_fan_speed:
            return
        speed_to_set = self.pending_speed
        if self.active_fan:
            curtime = self.printer.get_reactor().monotonic()
            speed_to_set = self.active_fan.get_status(curtime)['speed']
            self.gcode.run_script_from_command(
                "SET_FAN_SPEED FAN='%s' SPEED=%s" % (
                    self.active_fan.fan_name, 0.0))
        self.active_fan = fan
        if speed_to_set is not None:
            if self.active_fan:
                self.pending_speed = None
                self.gcode.run_script_from_command(
                    "SET_FAN_SPEED FAN='%s' SPEED=%s" % (
                        self.active_fan.fan_name, speed_to_set))
            else:
                self.pending_speed = speed_to_set

    def _resolve_tool(self, gcmd):
        """Resolve tool from M106/M107 P= parameter.

        Uses AFC lane map first (P5 -> T5 -> lane5 -> extruder4), falling
        back to the currently active tool when P is not specified.
        """
        tool_nr = gcmd.get_int('P', None)
        if tool_nr is not None:
            # Resolve through AFC lane map — consistent with M109 T= behavior
            map_name = "T{}".format(tool_nr)
            lane = self.toolchanger.afc.function.get_lane_by_map(map_name)
            if lane is not None:
                return lane.extruder_obj
            # If no lane maps to this P number, try direct tool lookup
            # as a last resort (for setups without full lane mapping)
            tool = self.toolchanger.lookup_tool(tool_nr)
            if tool is not None:
                return tool
        return self.toolchanger.active_tool

    def cmd_M106(self, gcmd):
        speed = gcmd.get_float('S', 255., minval=0.) / 255.
        self._set_speed(speed, self._resolve_tool(gcmd))

    def cmd_M107(self, gcmd):
        self._set_speed(0.0, self._resolve_tool(gcmd))

    def _set_speed(self, speed, tool):
        if tool and tool.fan:
            self.gcode.run_script_from_command(
                "SET_FAN_SPEED FAN='%s' SPEED=%s" % (
                    tool.fan.fan_name, speed))
        else:
            self.pending_speed = speed


def load_config_prefix(config: ConfigWrapper):
    return AfcToolchanger(config)

def load_config(config: ConfigWrapper):
    return load_config_prefix(config)