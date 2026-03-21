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

import bisect
import logging
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
        self.logo       = '<span class=success--text>Toolchanger Ready\n</span>'
        self.logo_error = '<span class=error--text>Toolchanger Not Ready</span>\n'
        self.functions: afcFunction = self.printer.load_object(config, 'AFC_functions')
        self.gcode_move = self.printer.load_object(config, 'gcode_move')
        self.gcode_macro = self.printer.load_object(config, 'gcode_macro')

        # Toolchanger config options
        self.uses_axis: str = config.get('uses_axis', 'xyz').lower()
        self.require_tool_present: bool = config.getboolean('require_tool_present', True)
        self.verify_tool_pickup: bool = config.getboolean('verify_tool_pickup', True)
        self.transfer_fan_speed: bool = config.getboolean('transfer_fan_speed', True)

        # Default gcode templates (can be overridden per-tool in AFC_extruder)
        self.default_before_change_gcode = self.gcode_macro.load_template(
            config, 'before_change_gcode', config.get('before_change_gcode', ''))
        self.default_after_change_gcode = self.gcode_macro.load_template(
            config, 'after_change_gcode', config.get('after_change_gcode', ''))
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

    def _handle_tc_connect(self):
        """Install the gcode offset transform on connect and find tool_probe_endstop."""
        self.gcode_transform.next_transform = self.gcode_move.set_move_transform(
            self.gcode_transform, force=True)
        self.tool_probe_endstop = self.printer.lookup_object(
            'tool_probe_endstop', None)

    def require_fan_switcher(self):
        """Create fan switcher on demand when a tool has a fan configured."""
        if not self.fan_switcher:
            self.fan_switcher = FanSwitcher(self)

    # --- Tool registration and lookup ---

    def register_tool(self, extruder, number):
        """Register an AFC_extruder as a tool with the given number."""
        if number in self.tools:
            logging.warning("Toolchanger: replacing tool T%d" % number)
            self.tool_numbers.remove(number)
            self.tool_names.remove(self.tools[number].name)
        self.tools[number] = extruder
        position = bisect.bisect_left(self.tool_numbers, number)
        self.tool_numbers.insert(position, number)
        self.tool_names.insert(position, extruder.name)
        # Update detection state
        self.has_detection = any(
            t.detect_state != DETECT_UNAVAILABLE for t in self.tools.values())
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
        return self.tools.get(number, None)

    def get_selected_tool(self):
        return self.active_tool

    def note_detect_change(self, extruder, eventtime):
        """Called by AFC_extruder when a detection pin changes state."""
        detected = None
        detected_names = []
        for tool in self.tools.values():
            if tool.detect_state == DETECT_PRESENT:
                detected = tool
                detected_names.append(tool.name)
        if len(detected_names) > 1:
            detected = None
        self.detected_tool = detected

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
        self.status = STATUS_CHANGING
        self._save_state("", None)
        self._set_toolchange_transform()

    def cmd_EXIT_DOCKING_MODE(self, gcmd):
        if self.status != STATUS_CHANGING:
            raise gcmd.error(
                "Cannot exit docking mode, status is %s" % self.status)
        self._restore_state_and_transform(self.active_tool)
        self.status = STATUS_READY

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
        if should_run_init:
            self.status = STATUS_INITIALIZING
            self._run_gcode('initialize_gcode', self.initialize_gcode, extra_context)

        inferred = select_tool
        if inferred is None and self.has_detection:
            inferred = self._require_detected_tool()

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
                self.gcode.respond_info(
                    '%s initialized, active %s' % (
                        self.config.get_name(),
                        self.active_tool.name if self.active_tool else None))
            else:
                raise self.gcode.error(
                    '%s failed to initialize: %s' % (
                        self.config.get_name(), self.error_message))

    def select_tool(self, gcmd, tool, restore_axis):
        """Core tool change: dropoff current, pickup new, manage offsets."""
        if self.status == STATUS_UNINITIALIZED:
            self.initialize(self.detected_tool)
        if self.status != STATUS_READY:
            raise gcmd.error(
                "Cannot select tool, status is %s: %s" % (
                    self.status, self.error_message))

        if self.active_tool == tool:
            gcmd.respond_info(
                'Tool %s already selected' % (tool.name if tool else None))
            return

        try:
            self._ensure_homed(gcmd)
            self.status = STATUS_CHANGING
            self._save_state(restore_axis, tool)

            # Validate active_tool against detection pins before dropoff.
            # If detection says no tool is on shuttle, clear active_tool to
            # prevent slamming into a docked tool during a phantom dropoff.
            if self.active_tool and self.has_detection:
                if self.active_tool.detect_state == DETECT_ABSENT:
                    self.logger.info(
                        "active_tool is %s but detection pin shows ABSENT, "
                        "clearing active_tool to skip dropoff"
                        % self.active_tool.name)
                    gcmd.respond_info(
                        "Warning: %s was marked active but is not detected "
                        "on shuttle, skipping dropoff" % self.active_tool.name)
                    self.active_tool.deactivate_tool()
                    self.active_tool = None

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
                if self.has_detection and self.verify_tool_pickup:
                    self._validate_detected_tool(tool, gcmd.respond_info, gcmd.error)
                self._run_gcode('after_change_gcode',
                               tool.after_change_gcode, extra_context)

            self._restore_state_and_transform(tool)
            self.status = STATUS_READY
            if tool:
                gcmd.respond_info('Selected tool %s (%s)' % (
                    str(tool.tool_number), tool.name))
            else:
                gcmd.respond_info('Tool unselected')

        except gcmd.error:
            if self.status == STATUS_ERROR:
                pass  # Error handling already ran
            else:
                raise

    def system_Test(self, cur_lane: AFCLane, delay: float, assignTcmd: str, enable_movement: bool):
        msg = ""
        succeeded = True

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
                    tool_ready = (
                        cur_lane.get_toolhead_pre_sensor_state()
                        or cur_lane.extruder_obj.tool_start == "buffer"
                        or cur_lane.extruder_obj.tool_end_state
                    )
                    if tool_ready:
                        cur_lane.sync_to_extruder()
                        on_shuttle = ""
                        if (cur_lane.extruder_obj.tool_obj
                            and cur_lane.extruder_obj.tc_unit_name):
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
                        lane_check = self.afc.error.fix('toolhead', cur_lane)

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
            probe = tool.tool_probe if tool else None
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
        """Find which tool is detected via detection pins."""
        detected = None
        for tool in self.tools.values():
            if tool.detect_state == DETECT_PRESENT:
                detected = tool
        return detected

    def _validate_detected_tool(self, expected, respond_info, raise_error):
        """Verify the expected tool is actually detected."""
        actual = self._require_detected_tool()
        if actual != expected:
            expected_name = expected.name if expected else "None"
            actual_name = actual.name if actual else "None"
            message = "Expected tool %s but detected %s" % (expected_name, actual_name)
            if raise_error:
                raise raise_error(message)

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
        Select's tool based off passed in extruder name

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
            if hasattr(tool, 'tc_lane') and tool.tc_lane is not None:
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
        if set_start_time:
            self.afc.afcDeltaTime.set_start_time()

        self.afc.current_state = State.TOOL_SWAP
        self._increase_unselect()
        self.afc.function.log_toolhead_pos("Before toolswap: ")
        # Save the current position before switching tools and subtract offsets
        for i in range(0, 3):
            self.afc.last_gcode_position[i] -= self.afc.gcode_move.base_position[i]

        self.afc.afcDeltaTime.log_with_time("Performing tool swap")

        if lane.extruder_obj.custom_tool_swap:
            self.logger.info(f"Running custom select: {lane.extruder_obj.custom_tool_swap}")
            self.afc.gcode.run_script_from_command(f"{lane.extruder_obj.custom_tool_swap}")
        else:
            # Use native toolchanger engine: SELECT_TOOL handles dropoff/pickup/offsets
            tool_index = lane.extruder_obj.tool_number
            if tool_index < 0:
                name = lane.extruder_obj.name
                tool_index = 0 if name == "extruder" else int(name.replace("extruder", ""))
            self.afc.gcode.run_script_from_command(
                'SELECT_TOOL T={}'.format(tool_index))

        # Activate the correct klipper extruder for this toolhead
        lane.activate_toolhead_extruder()
        # Sync AFC lane state (enable buffer, stepper, etc.)
        self.afc.function._handle_activate_extruder(0)

        self.afc.toolhead.wait_moves()
        self.afc.afcDeltaTime.log_with_time("Tool swap done", debug=False)
        if self.afc.afc_stats.average_tool_swap_time:
            self.afc.afc_stats.average_tool_swap_time.average_time(
                self.afc.afcDeltaTime.delta_time)
        self.afc.current_state = State.IDLE
        # Update the base position and homing position after the tool swap.
        self.afc.base_position   = list(self.afc.gcode_move.base_position)
        self.afc.homing_position = list(self.afc.gcode_move.homing_position)
        # Update the last_gcode_position to reflect the new base position.
        for i in range(0, 3):
            self.afc.last_gcode_position[i] += self.afc.gcode_move.base_position[i]

        self.afc.function.log_toolhead_pos("After toolswap: ")
        lane.extruder_obj.estats.tool_selected.increase_count()

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
            speed_to_set = self.active_fan.get_status(0)['speed']
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

    def cmd_M106(self, gcmd):
        speed = gcmd.get_float('S', 255., minval=0.) / 255.
        tool_nr = gcmd.get_int('P', None)
        tool = None
        if tool_nr is not None:
            tool = self.toolchanger.lookup_tool(tool_nr)
        if tool is None:
            tool = self.toolchanger.active_tool
        self._set_speed(speed, tool)

    def cmd_M107(self, gcmd):
        tool_nr = gcmd.get_int('P', None)
        tool = None
        if tool_nr is not None:
            tool = self.toolchanger.lookup_tool(tool_nr)
        if tool is None:
            tool = self.toolchanger.active_tool
        self._set_speed(0.0, tool)

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