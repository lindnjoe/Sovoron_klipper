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

import traceback
from configparser import Error as error

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from configfile import ConfigWrapper
    from gcode import GCodeCommand
    from extras.AFC_lane import AFCLane, MoveDirection, AFCHomingPoints
    from extras.AFC_functions import afcFunction
    from extras.AFC_stepper import AFCExtruderStepper

try: from extras.AFC_utils import ERROR_STR
except: raise error("Error when trying to import AFC_utils.ERROR_STR\n{trace}".format(trace=traceback.format_exc()))

try: from extras.AFC_unit import afcUnit
except: raise error(ERROR_STR.format(import_lib="AFC_unit", trace=traceback.format_exc()))

try: from extras.AFC import State
except: raise error(ERROR_STR.format(import_lib="AFC", trace=traceback.format_exc()))

try: from extras.AFC_lane import AFCMoveWarning, SpeedMode, AssistActive
except: raise error(ERROR_STR.format(import_lib="AFC_lane", trace=traceback.format_exc()))


class AfcToolchanger(afcUnit):
    def __init__(self, config: ConfigWrapper) -> None:
        super().__init__(config)
        self.type = config.get("type", "Toolchanger")
        self.auto_spoolman_create = config.getboolean("auto_spoolman_create", False)
        self.logo       = '<span class=success--text>Toolchanger Ready\n</span>'
        self.logo_error = '<span class=error--text>Toolchanger Not Ready</span>\n'
        self.functions: afcFunction = self.printer.load_object(config, 'AFC_functions')

        # Dock cooling — turns on part cooling fan for docked tools above
        # a temperature threshold to prevent oozing while parked.
        self.dock_cooling: bool = config.getboolean('dock_cooling', False)
        self.dock_cooling_temp: float = config.getfloat('dock_cooling_temp', 170.0, minval=0.)
        self.dock_cooling_fan_speed: float = config.getfloat('dock_cooling_fan_speed', 1.0, minval=0., maxval=1.0)
        self._dock_cooled_tools: set = set()
        # Always register the timer — per-tool dock_cooling overrides
        # can enable cooling even when the global setting is off.
        self._dock_cooling_timer = self.printer.get_reactor().register_timer(
            self._dock_cooling_tick)
        self.printer.register_event_handler("klippy:ready",
            lambda: self.printer.get_reactor().update_timer(
                self._dock_cooling_timer,
                self.printer.get_reactor().NOW + 5.0))

        self.functions.register_commands(self.afc.show_macros, "AFC_SELECT_TOOL",
                                         self.cmd_AFC_SELECT_TOOL, self.cmd_AFC_SELECT_TOOL_help,
                                         self.cmd_AFC_SELECT_TOOL_options )

        self.functions.register_commands(self.afc.show_macros, "AFC_UNSELECT_TOOL",
                                         self.cmd_AFC_UNSELECT_TOOL, self.cmd_AFC_UNSELECT_TOOL_help)

        self.functions.register_commands(self.afc.show_macros, "AFC_SET_TOOLHEAD_LED",
                                         self.cmd_AFC_SET_TOOLHEAD_LED,
                                         self.cmd_AFC_SET_TOOLHEAD_LED_help,
                                         self.cmd_AFC_SET_TOOLHEAD_LED_options)

    def system_Test(self, cur_lane: AFCLane, delay: float, assignTcmd: str, enable_movement: bool):
        if assignTcmd: self.afc.function.TcmdAssign(cur_lane)
        # Now that a T command is assigned, send lane data to moonraker
        cur_lane.send_lane_data()
        msg = ""
        if( cur_lane.prep_state and cur_lane.load_state ):
            msg = "<span class=success--text>LOADED</span> <span class=primary--text>in ToolHead</span>"
        self.logger.raw( '{lane_name} tool cmd: {tcmd:3} {msg}'.format(lane_name=cur_lane.name, tcmd=cur_lane.map, msg=msg))
        cur_lane.set_afc_prep_done()
        return True

    def _increase_unselect(self):
        """
        Helper function to lookup current selected extruder and increase tool unselected count
        """
        current_extruder = self.afc.function.get_current_extruder_obj()
        if current_extruder:
            current_extruder.estats.tool_unselected.increase_count()

    def move_to_hub(self, lane: AFCLane, dist: float,
                    dir: MoveDirection, use_homing: bool=True,
                    speed_mode: SpeedMode=SpeedMode.HUB,
                    assist_active: AssistActive=AssistActive.DYNAMIC
                ) -> tuple[bool, float|int, AFCMoveWarning]:
        """
        Overriding method from AFC_unit

        return: True, 0, AFCMoveWarning.NONE
        """
        return True, 0, AFCMoveWarning.NONE

    def move_to_load(self, lane: AFCLane, dist: float,
                     dir: MoveDirection, use_homing: bool=True,
                     speed_mode: SpeedMode=SpeedMode.LONG
                ) -> tuple[bool, float|int, AFCMoveWarning]:
        """
        Overriding method from AFC_unit

        return: True, 0, AFCMoveWarning.NONE
        """
        return True, 0, AFCMoveWarning.NONE

    def load_then_home(self, lane: AFCLane|AFCExtruderStepper, distance: float,
                       assist_active: AssistActive, endstop: AFCHomingPoints
                    ) -> tuple[bool, float|int, AFCMoveWarning]:
        """
        Overriding method from AFC_unit

        return: True, 0, AFCMoveWarning.NONE
        """
        return True, 0, AFCMoveWarning.NONE

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
        Unselects current tool loaded in shuttle by calling klipper-toolchanger UNSELECT_TOOL

        TODO: Update text once moved away from KTC

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
            self.afc.gcode.run_script_from_command("UNSELECT_TOOL")

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

        This function handles switching the active extruder/toolhead to the one associated with the given lane.
        It saves the current toolhead position, updates internal state, and issues a SELECT_TOOL command
        to switch to the correct extruder. This is primarily used in multi-extruder/toolchanger setups.

        :param lane: The lane object whose extruder/toolhead should be activated.
        :param set_start_time: Set true to set a starting time for afcDeltaTime.

        :return: None
        """
        if set_start_time:
            self.afc.afcDeltaTime.set_start_time()

        self.afc.current_state = State.TOOL_SWAP
        self._increase_unselect()
        self.afc.function.log_toolhead_pos("Before toolswap: ")
        # Save the current position before switching tools and subtract offsets
        for i in range(0, 3):
            self.afc.last_gcode_position[i] -= self.afc.gcode_move.base_position[i]
        # Perform a tool swap by selecting the appropriate extruder based on the lane's extruder object.
        self.afc.afcDeltaTime.log_with_time("Performing tool swap")
        name = lane.extruder_obj.name
        tool_index = 0 if name == "extruder" else int(name.replace("extruder", ""))

        if lane.extruder_obj.custom_tool_swap:
            self.logger.info(f"Running custom select: {lane.extruder_obj.custom_tool_swap}")
            self.afc.gcode.run_script_from_command(f"{lane.extruder_obj.custom_tool_swap}")
        else:
            self.afc.gcode.run_script_from_command('SELECT_TOOL T={}'.format(tool_index))

        # Switching toolhead extruders, this is mainly for setups with multiple extruders
        lane.activate_toolhead_extruder()
        # Need to call again since KTC activate callback happens before switching to new extruder
        # TODO: Take double call out once transitioned away from KTC
        self.afc.function._handle_activate_extruder(0)

        self.afc.toolhead.wait_moves()
        self.afc.afcDeltaTime.log_with_time("Tool swap done", debug=False)
        if self.afc.afc_stats.average_tool_swap_time:
            self.afc.afc_stats.average_tool_swap_time.average_time(self.afc.afcDeltaTime.delta_time)
        self.afc.current_state = State.IDLE
        # Update the base position and homing position after the tool swap.
        self.afc.base_position   = list(self.afc.gcode_move.base_position)
        self.afc.homing_position = list(self.afc.gcode_move.homing_position)
        # Update the last_gcode_position to reflect the new base position after the tool swap.
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

    # ---- Dock cooling ----

    def _get_tools_for_cooling(self):
        """Return extruder objects that belong to this toolchanger unit."""
        tools = []
        for tool in self.afc.tools.values():
            if getattr(tool, 'tc_unit_obj', None) == self:
                tools.append(tool)
        return tools

    def _dock_cooling_tick(self, eventtime):
        """Periodic check to start/stop dock cooling fans based on temperature.

        Uses fan object directly (not gcode) so it's safe from reactor context.
        Per-tool dock_cooling overrides the global setting.

        Fan speed is re-applied every tick so that external overrides
        (e.g. FanSwitcher queued commands racing with dock cooling) are
        corrected within one polling interval.  Klipper's _apply_speed
        discards redundant set_pwm calls internally, so this is cheap.
        """
        for tool in self._get_tools_for_cooling():
            tool_dc = getattr(tool, 'dock_cooling', None)
            if tool_dc is False:
                continue
            if tool_dc is None and not self.dock_cooling:
                continue
            # Active tool's fan is owned by the slicer — don't touch it.
            if tool.on_shuttle():
                self._dock_cooled_tools.discard(tool.name)
                continue
            try:
                current_temp, _ = tool.get_heater().get_temp(eventtime)
            except Exception:
                continue
            if current_temp >= self.dock_cooling_temp:
                self._start_dock_cooling_fan(tool, current_temp)
            else:
                self._stop_dock_cooling_fan(tool, current_temp)

        return eventtime + 5.0

    def _start_dock_cooling_fan(self, tool, current_temp):
        """Set fan to cooling speed — safe from reactor context."""
        fan_name = getattr(tool, 'fan_name', None)
        if not fan_name:
            return
        try:
            fan_obj = self.printer.lookup_object(
                "fan_generic " + fan_name, None)
            if fan_obj and hasattr(fan_obj, 'fan'):
                fan_obj.fan.set_speed(self.dock_cooling_fan_speed)
                if tool.name not in self._dock_cooled_tools:
                    self._dock_cooled_tools.add(tool.name)
                    self.logger.info(
                        "Dock cooling: %s fan on at %.0f%% "
                        "(temp %.1fC >= %.1fC)"
                        % (tool.name, self.dock_cooling_fan_speed * 100,
                           current_temp, self.dock_cooling_temp))
        except Exception as e:
            self.logger.warning(
                "Dock cooling: failed to start fan for %s: %s"
                % (tool.name, e))

    def _stop_dock_cooling_fan(self, tool, current_temp):
        """Set fan to zero — safe from reactor context."""
        fan_name = getattr(tool, 'fan_name', None)
        if not fan_name:
            return
        try:
            fan_obj = self.printer.lookup_object(
                "fan_generic " + fan_name, None)
            if fan_obj and hasattr(fan_obj, 'fan'):
                fan_obj.fan.set_speed(0.)
                if tool.name in self._dock_cooled_tools:
                    self._dock_cooled_tools.discard(tool.name)
                    self.logger.info(
                        "Dock cooling: %s fan off (temp %.1fC < %.1fC)"
                        % (tool.name, current_temp, self.dock_cooling_temp))
        except Exception as e:
            self.logger.warning(
                "Dock cooling: failed to stop fan for %s: %s"
                % (tool.name, e))

def load_config_prefix(config: ConfigWrapper):
    return AfcToolchanger(config)

def load_config(config: ConfigWrapper):
    return load_config_prefix(config)