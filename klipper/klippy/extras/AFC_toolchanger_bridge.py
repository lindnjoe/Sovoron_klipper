# AFC Toolchanger Bridge
#
# Copyright (C) 2024 - AFC + Toolchanger Integration
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# This module bridges AFC (Armored Filament Changer) with klipper-toolchanger-easy
# to provide seamless integration between filament management and physical tool changes.
#
# Key Features:
# - Auto-discovery of lane ? extruder ? tool mappings
# - Smart tool change detection (only when extruder changes)
# - Optional dock/pickup for same-tool lane swaps (for OpenAMS safety)
# - Tool offset management via toolchanger
# - Physical tool detection verification
# - Simplified lane change commands

import logging

class AFCToolchangerBridge:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.reactor = self.printer.get_reactor()

        self.name = config.get_name()

        # Core objects (loaded later)
        self.toolchanger = None
        self.afc = None

        # Configuration
        self.enabled = config.getboolean('enable', True)
        self.auto_tool_change = config.getboolean('auto_tool_change', True)
        self.verify_tool_mounted = config.getboolean('verify_tool_mounted', True)
        self.verify_timeout = config.getfloat('verify_timeout', 1.0)

        # Dock behavior for same-tool lane swaps (OpenAMS use case)
        # Support both old (global) and new (per-extruder) configuration
        self.dock_when_swap = config.getboolean('dock_when_swap', False)  # Deprecated global
        dock_extruders_str = config.get('dock_when_swap_extruders', '')
        self.dock_when_swap_extruders = set()
        if dock_extruders_str:
            # Parse comma-separated list of extruder names
            self.dock_when_swap_extruders = set(
                name.strip() for name in dock_extruders_str.split(',') if name.strip()
            )

        # Tool cutting behavior
        self.tool_cut = config.getboolean('tool_cut', False)
        self.tool_cut_command = config.get('tool_cut_command', 'AFC_CUT')

        # Post-cut retract and pre-purge extrude amounts (from AFC extruder settings)
        # These will be read from AFC extruder objects at runtime if not specified
        self.tool_stn_unload = config.getfloat('tool_stn_unload', None)  # Retract after cut
        self.tool_stn = config.getfloat('tool_stn', None)  # Extrude before purge

        # Purge behavior
        self.auto_purge = config.getboolean('auto_purge', False)
        self.purge_command = config.get('purge_command', 'LOAD_NOZZLE')
        self.purge_before_pickup = config.getboolean('purge_before_pickup', False)

        # Speed overrides (optional - uses toolchanger defaults if not set)
        self.tool_change_speed = config.getfloat('tool_change_speed', None)
        self.path_speed = config.getfloat('path_speed', None)

        # Mapping data structures
        self.lane_map = {}              # lane_name ? {extruder, tool, tool_number, tool_obj}
        self.extruder_tool_map = {}     # extruder_name ? tool_obj
        self.current_lane = None        # Currently loaded lane name

        # State tracking
        self.in_lane_change = False
        self.pending_pickup_tool = None

        # Register event handlers
        self.printer.register_event_handler("klippy:ready", self._handle_ready)

        # Register GCode commands
        self.gcode.register_command("AFC_CHANGE_LANE",
                                    self.cmd_AFC_CHANGE_LANE,
                                    desc=self.cmd_AFC_CHANGE_LANE_help)
        self.gcode.register_command("AFC_BRIDGE_STATUS",
                                    self.cmd_AFC_BRIDGE_STATUS,
                                    desc=self.cmd_AFC_BRIDGE_STATUS_help)
        self.gcode.register_command("AFC_BRIDGE_GET_TOOL",
                                    self.cmd_AFC_BRIDGE_GET_TOOL,
                                    desc=self.cmd_AFC_BRIDGE_GET_TOOL_help)
        self.gcode.register_command("AFC_BRIDGE_PREPARE_LANE",
                                    self.cmd_AFC_BRIDGE_PREPARE_LANE,
                                    desc=self.cmd_AFC_BRIDGE_PREPARE_LANE_help)
        self.gcode.register_command("AFC_BRIDGE_LANE_LOADED",
                                    self.cmd_AFC_BRIDGE_LANE_LOADED,
                                    desc=self.cmd_AFC_BRIDGE_LANE_LOADED_help)

        logging.info("AFC_toolchanger_bridge initialized (enabled=%s)" % self.enabled)

    def _handle_ready(self):
        """Build mappings after all components are loaded"""
        if not self.enabled:
            logging.info("AFC_toolchanger_bridge: Disabled, skipping initialization")
            return

        # Load AFC and Toolchanger objects
        try:
            self.afc = self.printer.lookup_object('AFC')
        except Exception as e:
            logging.warning("AFC_toolchanger_bridge: AFC not found, bridge disabled (%s)" % str(e))
            self.enabled = False
            return

        try:
            self.toolchanger = self.printer.lookup_object('toolchanger')
        except Exception as e:
            logging.warning("AFC_toolchanger_bridge: Toolchanger not found, bridge disabled (%s)" % str(e))
            self.enabled = False
            return

        # Build the mapping
        self._build_mappings()

        logging.info("AFC_toolchanger_bridge: Ready with %d lanes mapped" % len(self.lane_map))

    def _build_mappings(self):
        """Auto-discover lane ? extruder ? tool relationships"""
        # Find all tools and map them by extruder
        for tool_obj in self.toolchanger.tools.values():
            if hasattr(tool_obj, 'extruder_name') and tool_obj.extruder_name:
                self.extruder_tool_map[tool_obj.extruder_name] = tool_obj
                logging.info("AFC_toolchanger_bridge: Mapped %s ? Tool %s (T%d)" %
                           (tool_obj.extruder_name, tool_obj.name, tool_obj.tool_number))

        # Map each AFC lane to its tool
        for lane_name, lane_obj in self.afc.lanes.items():
            extruder_name = lane_obj.extruder_name

            if not extruder_name:
                logging.warning("AFC_toolchanger_bridge: Lane %s has no extruder defined" % lane_name)
                continue

            # Find the tool for this extruder
            tool_obj = self.extruder_tool_map.get(extruder_name)

            if tool_obj:
                self.lane_map[lane_name] = {
                    'extruder': extruder_name,
                    'tool': tool_obj.name,
                    'tool_number': tool_obj.tool_number,
                    'tool_obj': tool_obj,
                    'lane_obj': lane_obj
                }
                logging.info("AFC_toolchanger_bridge: Lane %s ? %s (T%d)" %
                           (lane_name, tool_obj.name, tool_obj.tool_number))
            else:
                logging.warning("AFC_toolchanger_bridge: Lane %s uses %s, but no tool found for that extruder" %
                              (lane_name, extruder_name))

    def get_lane_info(self, lane_name):
        """Get mapping info for a lane"""
        return self.lane_map.get(lane_name)

    def get_current_tool(self):
        """Get the currently active tool from toolchanger"""
        if not self.toolchanger:
            return None
        return self.toolchanger.active_tool

    def needs_tool_change(self, from_lane, to_lane):
        """Check if switching lanes requires a physical tool change"""
        from_info = self.get_lane_info(from_lane) if from_lane else None
        to_info = self.get_lane_info(to_lane)

        if not to_info:
            return False

        # If no current lane, definitely need to select tool
        if not from_info:
            return True

        # Check if extruders differ
        return from_info['extruder'] != to_info['extruder']

    def should_dock_for_swap(self, from_lane, to_lane):
        """Check if we should dock the tool for a same-tool lane swap"""
        # Only dock if staying on the same extruder/tool
        if self.needs_tool_change(from_lane, to_lane):
            return False

        # Check per-extruder setting first (new method)
        if self.dock_when_swap_extruders:
            to_info = self.get_lane_info(to_lane)
            if to_info:
                extruder = to_info['extruder']
                return extruder in self.dock_when_swap_extruders
            return False

        # Fall back to global setting (deprecated)
        return self.dock_when_swap

    def get_extruder_setting(self, extruder_name, setting_name, default=None):
        """Get a setting from AFC extruder object"""
        if not self.afc or not extruder_name:
            return default

        # Try to find the extruder in AFC's tools/extruders
        extruder_obj = self.afc.tools.get(extruder_name)
        if extruder_obj and hasattr(extruder_obj, setting_name):
            value = getattr(extruder_obj, setting_name)
            if value is not None:
                return value

        return default

    def get_tool_stn_unload(self, extruder_name):
        """Get tool_stn_unload for an extruder (retract after cut)"""
        if self.tool_stn_unload is not None:
            return self.tool_stn_unload
        return self.get_extruder_setting(extruder_name, 'tool_stn_unload', 0)

    def get_tool_stn(self, extruder_name):
        """Get tool_stn for an extruder (extrude before purge)"""
        if self.tool_stn is not None:
            return self.tool_stn
        return self.get_extruder_setting(extruder_name, 'tool_stn', 0)

    def get_lane_temperature(self, lane_name, default_temp=240):
        """
        Get the target temperature for a lane.
        For OpenAMS lanes, this reads from the unit's temperature cache.
        Falls back to AFC's default material temperatures.
        """
        if not self.afc or not lane_name:
            return default_temp

        # Try to get lane object
        lane_key = "AFC_lane %s" % lane_name
        lane = self.printer.lookup_object(lane_key, None)
        if not lane:
            return default_temp

        # Check if lane has explicit temperature set
        lane_temp = getattr(lane, 'extruder_temp', None) or getattr(lane, 'nozzle_temp', None)
        if lane_temp is not None:
            try:
                return int(lane_temp)
            except (TypeError, ValueError):
                pass

        # For OpenAMS units, try to get cached temperature
        unit = getattr(lane, 'unit_obj', None)
        if unit and hasattr(unit, 'get_lane_temperature'):
            try:
                temp = unit.get_lane_temperature(lane_name, default_temp)
                if temp is not None:
                    return int(temp)
            except Exception as e:
                logging.warning("AFC_toolchanger_bridge: Failed to get OpenAMS temperature for %s: %s" % (lane_name, e))

        # Fall back to AFC default material temps
        material = getattr(lane, 'material', None) or getattr(lane, '_material', None)
        if material and self.afc.default_material_temps:
            for temp_entry in self.afc.default_material_temps:
                if isinstance(temp_entry, str) and ':' in temp_entry:
                    mat, temp = temp_entry.split(':', 1)
                    if mat.strip().upper() in str(material).upper():
                        try:
                            return int(temp)
                        except (TypeError, ValueError):
                            pass

        return default_temp

    def select_tool(self, tool_number, restore_axis='ZYX'):
        """Select a tool via toolchanger"""
        if not self.auto_tool_change:
            logging.info("AFC_toolchanger_bridge: auto_tool_change disabled, skipping SELECT_TOOL T=%d" % tool_number)
            return

        cmd = "SELECT_TOOL T=%d" % tool_number
        if restore_axis:
            cmd += " RESTORE_AXIS=%s" % restore_axis

        logging.info("AFC_toolchanger_bridge: Executing %s" % cmd)
        self.gcode.run_script_from_command(cmd)

    def unselect_tool(self):
        """Unselect (dock) the current tool"""
        if not self.auto_tool_change:
            logging.info("AFC_toolchanger_bridge: auto_tool_change disabled, skipping UNSELECT_TOOL")
            return

        logging.info("AFC_toolchanger_bridge: Executing UNSELECT_TOOL")
        self.gcode.run_script_from_command("UNSELECT_TOOL")

    def verify_tool(self, expected_tool_number, timeout=None):
        """Verify the expected tool is mounted via detection pins"""
        if not self.verify_tool_mounted:
            return True

        if timeout is None:
            timeout = self.verify_timeout

        if not self.toolchanger.has_detection:
            logging.warning("AFC_toolchanger_bridge: verify_tool_mounted enabled but toolchanger has no detection pins")
            return True

        # Wait a bit for detection to settle
        self.reactor.pause(self.reactor.monotonic() + timeout)

        detected = self.toolchanger.detected_tool
        if detected and detected.tool_number == expected_tool_number:
            logging.info("AFC_toolchanger_bridge: Verified T%d is mounted" % expected_tool_number)
            return True

        detected_num = detected.tool_number if detected else None
        logging.error("AFC_toolchanger_bridge: Tool verification failed! Expected T%d, detected T%s" %
                     (expected_tool_number, detected_num))
        return False

    def prepare_lane_change(self, to_lane, from_lane=None):
        """
        Prepare for a lane change - handles tool docking/changing if needed.

        IMPORTANT: This does NOT unload filament from the previous tool.
        Filament remains loaded in each tool's extruder when the tool is docked.

        Behavior:
        - Same extruder (lane swap): Optionally dock tool if dock_when_swap=True
        - Different extruder (tool change): Dock current tool, pickup new tool
        - Filament unload/load is handled separately by AFC macros
        """
        to_info = self.get_lane_info(to_lane)
        if not to_info:
            logging.warning("AFC_toolchanger_bridge: Unknown lane %s" % to_lane)
            return

        from_info = self.get_lane_info(from_lane) if from_lane else None

        # Case 1: Same tool lane swap with dock_when_swap enabled
        # Use case: OpenAMS on toolhead needs tool out of the way for safe loading
        if self.should_dock_for_swap(from_lane, to_lane):
            current_tool = self.get_current_tool()
            if current_tool:
                extruder_name = to_info['extruder']
                logging.info("AFC_toolchanger_bridge: Same-tool swap (%s?%s) on %s, docking %s" %
                           (from_lane, to_lane, extruder_name, current_tool.name))

                # Step 1: Cut filament if enabled (BEFORE any unloading)
                if self.tool_cut and self.tool_cut_command:
                    logging.info("AFC_toolchanger_bridge: Cutting filament: %s" % self.tool_cut_command)
                    self.gcode.run_script_from_command(self.tool_cut_command)

                    # Step 2: Retract tool_stn_unload after cut
                    tool_stn_unload = self.get_tool_stn_unload(extruder_name)
                    if tool_stn_unload > 0:
                        logging.info("AFC_toolchanger_bridge: Retracting %.1fmm after cut" % tool_stn_unload)
                        self.gcode.run_script_from_command(
                            "G1 E-%.3f F300" % tool_stn_unload
                        )

                # Step 3: Dock the tool
                self.unselect_tool()
                self.pending_pickup_tool = to_info['tool_number']

        # Case 2: Different tool - need tool change
        # SELECT_TOOL handles: dock current tool ? pickup new tool
        # Filament stays in the previous tool's extruder (not unloaded)
        elif self.needs_tool_change(from_lane, to_lane):
            current_tool = self.get_current_tool()
            logging.info("AFC_toolchanger_bridge: Tool change %s?%s (extruder change)" %
                       (current_tool.name if current_tool else 'None', to_info['tool']))
            logging.info("AFC_toolchanger_bridge: Filament remains in previous tool's extruder")

            # Dock current tool and pickup new tool
            self.select_tool(to_info['tool_number'])

            # Verify tool is mounted
            if self.verify_tool_mounted:
                if not self.verify_tool(to_info['tool_number']):
                    raise self.printer.command_error(
                        "AFC_toolchanger_bridge: Tool verification failed for T%d" % to_info['tool_number'])

    def finalize_lane_change(self, to_lane):
        """Finalize after lane is loaded - pickup tool if needed, trigger purge"""
        to_info = self.get_lane_info(to_lane)
        if not to_info:
            return

        # Handle purge and pickup order based on configuration
        has_pending_pickup = self.pending_pickup_tool is not None

        # Case 1: Purge before pickup (tool still docked, common for OpenAMS)
        if self.purge_before_pickup and has_pending_pickup:
            extruder_name = to_info['extruder']

            # Extrude tool_stn before purging (pushes new filament into position)
            tool_stn = self.get_tool_stn(extruder_name)
            if tool_stn > 0:
                logging.info("AFC_toolchanger_bridge: Extruding %.1fmm before purge" % tool_stn)
                self.gcode.run_script_from_command(
                    "G1 E%.3f F300" % tool_stn
                )

            # Purge while tool is docked
            if self.auto_purge and self.purge_command:
                logging.info("AFC_toolchanger_bridge: Purging before pickup (tool docked)")
                self.gcode.run_script_from_command(self.purge_command)

            # Then pickup tool
            logging.info("AFC_toolchanger_bridge: Picking up T%d after purge" % self.pending_pickup_tool)
            self.select_tool(self.pending_pickup_tool)

            # Verify pickup
            if self.verify_tool_mounted:
                if not self.verify_tool(self.pending_pickup_tool):
                    raise self.printer.command_error(
                        "AFC_toolchanger_bridge: Tool pickup verification failed for T%d" % self.pending_pickup_tool)

            self.pending_pickup_tool = None

        # Case 2: Pickup before purge (standard workflow)
        else:
            extruder_name = to_info['extruder']

            # Pickup tool first if we docked it earlier
            if has_pending_pickup:
                logging.info("AFC_toolchanger_bridge: Picking up T%d before purge" % self.pending_pickup_tool)
                self.select_tool(self.pending_pickup_tool)

                # Verify pickup
                if self.verify_tool_mounted:
                    if not self.verify_tool(self.pending_pickup_tool):
                        raise self.printer.command_error(
                            "AFC_toolchanger_bridge: Tool pickup verification failed for T%d" % self.pending_pickup_tool)

                self.pending_pickup_tool = None

            # Extrude tool_stn before purging (if we had a tool swap)
            if has_pending_pickup or self.tool_cut:
                tool_stn = self.get_tool_stn(extruder_name)
                if tool_stn > 0:
                    logging.info("AFC_toolchanger_bridge: Extruding %.1fmm before purge" % tool_stn)
                    self.gcode.run_script_from_command(
                        "G1 E%.3f F300" % tool_stn
                    )

            # Then purge if enabled
            if self.auto_purge and self.purge_command:
                logging.info("AFC_toolchanger_bridge: Purging after pickup (tool mounted)")
                self.gcode.run_script_from_command(self.purge_command)

        # Update current lane
        self.current_lane = to_lane

    # GCode Commands

    cmd_AFC_CHANGE_LANE_help = "Change AFC lane with automatic tool handling"
    def cmd_AFC_CHANGE_LANE(self, gcmd):
        """
        Main command for lane changes. Handles complete orchestration:
        1. Tool docking (if same-tool swap and dock_when_swap=True)
        2. AFC unload of old lane
        3. Tool change (if different extruder)
        4. Temperature management
        5. AFC load of new lane
        6. Tool pickup (if we docked earlier)
        7. Purge (if auto_purge=True)

        This eliminates the need for custom_load_cmd and custom_unload_cmd macros.
        """
        if not self.enabled:
            raise gcmd.error("AFC_toolchanger_bridge is disabled")

        from_lane = gcmd.get('FROM', self.current_lane)
        to_lane = gcmd.get('TO', None)

        if not to_lane:
            raise gcmd.error("AFC_CHANGE_LANE: TO parameter required")

        to_info = self.get_lane_info(to_lane)
        if not to_info:
            raise gcmd.error("AFC_CHANGE_LANE: Unknown lane %s" % to_lane)

        try:
            self.in_lane_change = True

            gcmd.respond_info("AFC Bridge: Changing lane %s ? %s" % (from_lane or 'None', to_lane))

            # Step 1: Prepare (dock if needed, tool change)
            self.prepare_lane_change(to_lane, from_lane)

            # Step 2: Unload old lane (if exists)
            if from_lane:
                from_info = self.get_lane_info(from_lane)
                if from_info:
                    gcmd.respond_info("AFC Bridge: Unloading %s" % from_lane)
                    # Call AFC's built-in unload command
                    self.gcode.run_script_from_command("TOOL_UNLOAD LANE=%s" % from_lane)

            # Step 3: Heat extruder to target temperature
            target_temp = self.get_lane_temperature(to_lane)
            if target_temp:
                extruder_name = to_info['extruder']
                gcmd.respond_info("AFC Bridge: Heating %s to %dC" % (extruder_name, target_temp))
                # Activate the correct extruder and heat it
                if extruder_name != 'extruder':
                    # Switch to the correct extruder first
                    self.gcode.run_script_from_command("ACTIVATE_EXTRUDER EXTRUDER=%s" % extruder_name)
                # Heat and wait for temperature
                self.gcode.run_script_from_command("M109 S%d" % target_temp)

            # Step 4: Load new lane using AFC's built-in load command
            gcmd.respond_info("AFC Bridge: Loading %s" % to_lane)
            self.gcode.run_script_from_command("TOOL_LOAD LANE=%s" % to_lane)

            # Step 5: Finalize (pickup if needed, purge)
            self.finalize_lane_change(to_lane)

            gcmd.respond_info("AFC Bridge: Lane change complete - %s ready" % to_lane)

        finally:
            self.in_lane_change = False

    cmd_AFC_BRIDGE_PREPARE_LANE_help = "Prepare for lane load (dock tool if needed)"
    def cmd_AFC_BRIDGE_PREPARE_LANE(self, gcmd):
        """
        Called from custom_load_cmd macros to prepare for lane load.
        Handles tool docking if needed.
        """
        if not self.enabled:
            return

        to_lane = gcmd.get('LANE', None)
        if not to_lane:
            raise gcmd.error("AFC_BRIDGE_PREPARE_LANE: LANE parameter required")

        from_lane = gcmd.get('FROM', self.current_lane)

        self.prepare_lane_change(to_lane, from_lane)

    cmd_AFC_BRIDGE_LANE_LOADED_help = "Finalize after lane load (pickup tool if needed)"
    def cmd_AFC_BRIDGE_LANE_LOADED(self, gcmd):
        """
        Called from custom_load_cmd macros after lane is loaded.
        Handles tool pickup if we docked earlier.
        """
        if not self.enabled:
            return

        to_lane = gcmd.get('LANE', None)
        if not to_lane:
            raise gcmd.error("AFC_BRIDGE_LANE_LOADED: LANE parameter required")

        self.finalize_lane_change(to_lane)

    cmd_AFC_BRIDGE_GET_TOOL_help = "Get tool info for a lane"
    def cmd_AFC_BRIDGE_GET_TOOL(self, gcmd):
        """Query which tool is needed for a lane"""
        lane = gcmd.get('LANE', None)
        if not lane:
            raise gcmd.error("AFC_BRIDGE_GET_TOOL: LANE parameter required")

        info = self.get_lane_info(lane)
        if info:
            gcmd.respond_info("Lane %s: Tool=%s (T%d), Extruder=%s" %
                            (lane, info['tool'], info['tool_number'], info['extruder']))
        else:
            gcmd.respond_info("Lane %s: Not found in mapping" % lane)

    cmd_AFC_BRIDGE_STATUS_help = "Show AFC toolchanger bridge status"
    def cmd_AFC_BRIDGE_STATUS(self, gcmd):
        """Display bridge status and mappings"""
        msg = "AFC Toolchanger Bridge Status:\n"
        msg += "  Enabled: %s\n" % self.enabled
        msg += "  Auto Tool Change: %s\n" % self.auto_tool_change

        # Show dock configuration
        if self.dock_when_swap_extruders:
            msg += "  Dock When Swap: Per-extruder (%s)\n" % ', '.join(sorted(self.dock_when_swap_extruders))
        else:
            msg += "  Dock When Swap: %s (global)\n" % self.dock_when_swap

        msg += "  Tool Cut: %s\n" % self.tool_cut
        msg += "  Auto Purge: %s\n" % self.auto_purge
        msg += "  Verify Tool: %s (timeout: %.1fs)\n" % (self.verify_tool_mounted, self.verify_timeout)
        msg += "  Current Lane: %s\n" % (self.current_lane or 'None')

        if self.toolchanger:
            current_tool = self.get_current_tool()
            detected_tool = self.toolchanger.detected_tool
            msg += "\nToolchanger:\n"
            msg += "  Active Tool: %s\n" % (current_tool.name if current_tool else 'None')
            msg += "  Detected Tool: %s\n" % (detected_tool.name if detected_tool else 'None')
            msg += "  Has Detection: %s\n" % self.toolchanger.has_detection

        msg += "\nLane Mappings (%d lanes):\n" % len(self.lane_map)
        for lane_name in sorted(self.lane_map.keys()):
            info = self.lane_map[lane_name]
            msg += "  %s ? %s (T%d) via %s\n" %
                   (lane_name, info['tool'], info['tool_number'], info['extruder'])

        gcmd.respond_info(msg)

    def get_status(self, eventtime):
        """Provide status for Moonraker/Mainsail"""
        current_tool = self.get_current_tool()
        detected_tool = self.toolchanger.detected_tool if self.toolchanger else None

        return {
            'enabled': self.enabled,
            'auto_tool_change': self.auto_tool_change,
            'dock_when_swap': self.dock_when_swap,
            'auto_purge': self.auto_purge,
            'current_lane': self.current_lane,
            'current_tool': current_tool.name if current_tool else None,
            'current_tool_number': current_tool.tool_number if current_tool else -1,
            'detected_tool': detected_tool.name if detected_tool else None,
            'detected_tool_number': detected_tool.tool_number if detected_tool else -1,
            'lanes_mapped': len(self.lane_map),
            'in_lane_change': self.in_lane_change,
        }

def load_config(config):
    return AFCToolchangerBridge(config)