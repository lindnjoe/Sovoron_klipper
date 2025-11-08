# AFC M109 Deadband Override Module
#
# This module provides a toggleable feature to automatically use the deadband
# configured for each AFC extruder when using M109 commands.
#
# Copyright (C) 2025
#
# This file may be distributed under the terms of the GNU GPLv3 license.

class AFC_M109_Deadband:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')

        # Get configuration options
        self.enabled = config.getint('enabled', 1)  # Default: enabled

        # Register event handler to hook in after AFC loads
        self.printer.register_event_handler("klippy:connect",
                                           self.handle_connect)

        # Register gcode commands for enable/disable
        self.gcode.register_command('AFC_M109_DEADBAND_ENABLE',
                                   self.cmd_ENABLE,
                                   desc=self.cmd_ENABLE_help)
        self.gcode.register_command('AFC_M109_DEADBAND_DISABLE',
                                   self.cmd_DISABLE,
                                   desc=self.cmd_DISABLE_help)
        self.gcode.register_command('AFC_M109_DEADBAND_STATUS',
                                   self.cmd_STATUS,
                                   desc=self.cmd_STATUS_help)

    def handle_connect(self):
        """
        This runs after Klipper connects. We need to rename AFC's M109
        and register our own version.
        """
        # Check if AFC is loaded
        try:
            self.afc = self.printer.lookup_object('AFC')
        except:
            raise self.printer.config_error(
                "AFC_M109_deadband requires AFC to be loaded")

        # Rename AFC's M109 command to M109_AFC_ORIGINAL
        try:
            self.gcode.register_command('M109_AFC_ORIGINAL',
                                       self.gcode.commands['M109'].func,
                                       desc="Original AFC M109 command")
            # Register our override
            self.gcode.register_command('M109', self.cmd_M109,
                                       desc="M109 with automatic AFC deadband")
        except Exception as e:
            self.gcode.respond_info(
                "AFC_M109_deadband: Could not override M109: %s" % str(e))

    cmd_ENABLE_help = "Enable automatic AFC deadband usage in M109"
    def cmd_ENABLE(self, gcmd):
        self.enabled = 1
        gcmd.respond_info("AFC M109 Deadband Override: ENABLED - "
                         "M109 will use AFC extruder deadband settings")

    cmd_DISABLE_help = "Disable automatic AFC deadband usage in M109"
    def cmd_DISABLE(self, gcmd):
        self.enabled = 0
        gcmd.respond_info("AFC M109 Deadband Override: DISABLED - "
                         "M109 will use default AFC behavior")

    cmd_STATUS_help = "Show status of AFC M109 deadband override"
    def cmd_STATUS(self, gcmd):
        status = "ENABLED" if self.enabled else "DISABLED"
        gcmd.respond_info("AFC M109 Deadband Override: %s" % status)

    def cmd_M109(self, gcmd):
        """
        Override M109 to automatically add deadband from AFC extruder config
        """
        # Get parameters
        temp = gcmd.get_float('S', 0.0)
        toolnum = gcmd.get_int('T', None, minval=0)
        deadband_override = gcmd.get_float('D', None)

        # Build command to pass to AFC's M109
        cmd_parts = ["M109_AFC_ORIGINAL"]
        if temp > 0:
            cmd_parts.append("S=%.1f" % temp)
        if toolnum is not None:
            cmd_parts.append("T=%d" % toolnum)

        # If override is enabled and no deadband was manually specified
        if self.enabled and deadband_override is None:
            deadband = self._get_deadband_for_tool(toolnum)
            if deadband is not None:
                cmd_parts.append("D=%.1f" % deadband)
                # Determine extruder name for logging
                if toolnum is not None:
                    extruder_name = "extruder" if toolnum == 0 else "extruder%d" % toolnum
                else:
                    toolhead = self.printer.lookup_object('toolhead')
                    extruder_name = toolhead.get_extruder().get_name()
                gcmd.respond_info("M109: Using AFC deadband %.1f°C for %s" %
                                (deadband, extruder_name))
        elif deadband_override is not None:
            # User manually specified deadband, pass it through
            cmd_parts.append("D=%.1f" % deadband_override)

        # Execute AFC's original M109
        self.gcode.run_script_from_command(" ".join(cmd_parts))

    def _get_deadband_for_tool(self, toolnum):
        """
        Get the deadband value from AFC extruder configuration
        """
        # Determine which extruder to look up
        if toolnum is not None:
            extruder_name = "extruder" if toolnum == 0 else "extruder%d" % toolnum
        else:
            # Get current extruder
            toolhead = self.printer.lookup_object('toolhead')
            extruder_name = toolhead.get_extruder().get_name()

        # Look up AFC extruder object
        afc_extruder_name = "AFC_extruder %s" % extruder_name
        try:
            afc_extruder = self.printer.lookup_object(afc_extruder_name)
            # Get deadband attribute
            if hasattr(afc_extruder, 'deadband'):
                return afc_extruder.deadband
        except:
            pass

        return None

def load_config(config):
    return AFC_M109_Deadband(config)