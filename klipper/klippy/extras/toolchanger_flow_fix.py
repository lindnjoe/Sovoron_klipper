# Monkey patch to fix flow rate and filament tracking with toolchangers
#
# This module fixes three issues that occur during toolchanges:
# 1. Volumetric flow (mm³/s) showing 0.0 or not tracking correctly
# 2. Filament used tracking stopping after toolchanges
# 3. Print time estimates blanking out after toolchanges
#
# To use, add this to your printer.cfg:
#   [toolchanger_flow_fix]
#
# Copyright (C) 2025  Joe Lindner <lindnjoe@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license

class ToolchangerFlowFix:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.register_event_handler("klippy:connect",
                                           self._handle_connect)

    def _handle_connect(self):
        # Patch gcode_move to preserve extrude_factor during toolchanges
        gcode_move = self.printer.lookup_object('gcode_move')
        self._patch_gcode_move(gcode_move)

        # Patch motion_report to track active extruder velocity
        motion_report = self.printer.lookup_object('motion_report', None)
        if motion_report is not None:
            self._patch_motion_report(motion_report)

    def _patch_gcode_move(self, gcode_move):
        """Patch GCodeMove to preserve extrude_factor during toolchanges"""
        original_activate = gcode_move._handle_activate_extruder

        def patched_handle_activate_extruder():
            # Reset position but preserve extrude_factor
            gcode_move.reset_last_position()
            # Do NOT reset extrude_factor - preserve for filament tracking
            # The toolchanger system uses SAVE_GCODE_STATE/RESTORE_GCODE_STATE
            # Original code did: gcode_move.extrude_factor = 1.
            gcode_move.base_position[3] = gcode_move.last_position[3]

        gcode_move._handle_activate_extruder = patched_handle_activate_extruder

    def _patch_motion_report(self, motion_report):
        """Patch PrinterMotionReport to track active extruder velocity"""
        original_get_status = motion_report.get_status

        def patched_get_status(eventtime):
            # Call original to get base status
            status = original_get_status(eventtime)

            # If we're past the status refresh time, update extruder velocity
            if eventtime >= motion_report.next_status_time - 0.001:
                toolhead = motion_report.printer.lookup_object('toolhead')
                active_extruder = toolhead.get_extruder()

                if active_extruder is not None:
                    extruder_name = active_extruder.get_name()
                    ehandler = motion_report.dtrapqs.get(extruder_name)

                    if ehandler is not None:
                        mcu = motion_report.printer.lookup_object('mcu')
                        print_time = mcu.estimated_print_time(eventtime)
                        pos, velocity = ehandler.get_trapq_position(print_time)

                        if pos is not None and velocity is not None:
                            # Update the velocity from the active extruder
                            status = dict(status)
                            status['live_extruder_velocity'] = velocity
                            motion_report.last_status = status

            return status

        motion_report.get_status = patched_get_status

def load_config(config):
    return ToolchangerFlowFix(config)