# Monkey patch to fix volumetric flow rate tracking with toolchangers
#
# This module fixes the issue where volumetric flow (mm³/s) shows 0.0
# or stops tracking correctly after toolchanges.
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
        # Patch motion_report to track active extruder velocity
        motion_report = self.printer.lookup_object('motion_report', None)
        if motion_report is not None:
            self._patch_motion_report(motion_report)

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