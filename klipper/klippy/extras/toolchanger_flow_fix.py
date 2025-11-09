# Monkey patch to fix flow rate and filament tracking with toolchangers
#
# This module fixes three issues that occur during toolchanges:
# 1. Volumetric flow (mm³/s) showing 0.0 or not tracking correctly
# 2. Filament used tracking stopping after toolchanges
# 3. Print time estimates blanking out after toolchanges
#
# To use, add this to your printer.cfg:
#   [toolchanger_flow_fix]
#   debug: True  # Optional, for detailed logging
#
# Copyright (C) 2025  Joe Lindner <lindnjoe@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license

import logging

class ToolchangerFlowFix:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = None
        self.debug_enabled = config.getboolean('debug', False)
        self.printer.register_event_handler("klippy:connect",
                                           self._handle_connect)
        self.printer.register_event_handler("klippy:ready",
                                           self._handle_ready)

    def _handle_connect(self):
        self.gcode = self.printer.lookup_object('gcode')
        logging.info("toolchanger_flow_fix: Module loaded and connecting")

        # Patch gcode_move to preserve extrude_factor during toolchanges
        gcode_move = self.printer.lookup_object('gcode_move')
        self._patch_gcode_move(gcode_move)

        # Patch motion_report to track active extruder velocity
        motion_report = self.printer.lookup_object('motion_report', None)
        if motion_report is not None:
            self._patch_motion_report(motion_report)
        else:
            logging.warning("toolchanger_flow_fix: motion_report not found!")

        # Try to patch AFC if it exists
        afc = self.printer.lookup_object('AFC', None)
        if afc is not None:
            self._patch_afc(afc)
            logging.info("toolchanger_flow_fix: AFC patches applied")
        else:
            logging.info("toolchanger_flow_fix: AFC not found, skipping AFC patches")

        logging.info("toolchanger_flow_fix: All patches applied successfully")

    def _patch_gcode_move(self, gcode_move):
        """Patch GCodeMove to preserve extrude_factor during toolchanges"""
        original_activate = gcode_move._handle_activate_extruder
        debug_enabled = self.debug_enabled

        def patched_handle_activate_extruder():
            # Save the current extrude_factor before doing anything
            saved_extrude_factor = gcode_move.extrude_factor

            # Reset position but preserve extrude_factor
            gcode_move.reset_last_position()
            # Do NOT reset extrude_factor - preserve for filament tracking
            # Original code did: gcode_move.extrude_factor = 1.
            gcode_move.base_position[3] = gcode_move.last_position[3]

            # Log what happened
            if debug_enabled:
                logging.info("toolchanger_flow_fix: ACTIVATE_EXTRUDER called - "
                           "preserved extrude_factor=%.3f (would have reset to 1.0)"
                           % (saved_extrude_factor,))
            else:
                logging.info("toolchanger_flow_fix: Extruder activated, "
                           "extrude_factor preserved at %.3f" % (saved_extrude_factor,))

        gcode_move._handle_activate_extruder = patched_handle_activate_extruder
        logging.info("toolchanger_flow_fix: gcode_move patch applied")

    def _patch_motion_report(self, motion_report):
        """Patch PrinterMotionReport to track active extruder velocity"""
        original_get_status = motion_report.get_status
        debug_enabled = self.debug_enabled
        call_count = [0]  # Mutable to allow modification in closure

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
                            old_velocity = status.get('live_extruder_velocity', 0)
                            status = dict(status)
                            status['live_extruder_velocity'] = velocity
                            motion_report.last_status = status

                            # Debug logging - only log every 50th call to avoid spam
                            call_count[0] += 1
                            if debug_enabled and call_count[0] % 50 == 0:
                                logging.info("toolchanger_flow_fix: Active extruder=%s, "
                                           "velocity=%.3f mm/s (was %.3f)"
                                           % (extruder_name, velocity, old_velocity))
                        elif debug_enabled and call_count[0] % 100 == 0:
                            logging.warning("toolchanger_flow_fix: trapq position is None "
                                          "for extruder %s at print_time %.3f"
                                          % (extruder_name, print_time))
                    elif debug_enabled and call_count[0] % 100 == 0:
                        logging.warning("toolchanger_flow_fix: No trapq handler for "
                                      "extruder %s, available: %s"
                                      % (extruder_name, list(motion_report.dtrapqs.keys())))
                elif debug_enabled and call_count[0] % 100 == 0:
                    logging.warning("toolchanger_flow_fix: No active extruder!")

            return status

        motion_report.get_status = patched_get_status
        logging.info("toolchanger_flow_fix: motion_report patch applied")

    def _patch_afc(self, afc):
        """Patch AFC save_pos/restore_pos to add logging"""
        original_save_pos = afc.save_pos
        original_restore_pos = afc.restore_pos
        debug_enabled = self.debug_enabled

        def patched_save_pos():
            original_save_pos()
            extrude_factor = afc.extrude_factor if hasattr(afc, 'extrude_factor') else afc.gcode_move.extrude_factor
            if debug_enabled:
                logging.info("toolchanger_flow_fix: AFC save_pos() called, "
                           "extrude_factor=%.3f saved" % extrude_factor)

        def patched_restore_pos(move_z_first=True):
            saved_extrude_factor = afc.extrude_factor if hasattr(afc, 'extrude_factor') else None
            original_restore_pos(move_z_first)
            current_extrude_factor = afc.gcode_move.extrude_factor
            if debug_enabled:
                logging.info("toolchanger_flow_fix: AFC restore_pos() called, "
                           "extrude_factor restored to %.3f (was saved as %.3f)"
                           % (current_extrude_factor, saved_extrude_factor if saved_extrude_factor else 0))

        afc.save_pos = patched_save_pos
        afc.restore_pos = patched_restore_pos

    def _handle_ready(self):
        self.gcode.register_command('FLOW_FIX_STATUS',
                                   self.cmd_FLOW_FIX_STATUS,
                                   desc="Report toolchanger flow fix status")

    cmd_FLOW_FIX_STATUS_help = "Report current flow fix status and diagnostics"
    def cmd_FLOW_FIX_STATUS(self, gcmd):
        gcode_move = self.printer.lookup_object('gcode_move')
        motion_report = self.printer.lookup_object('motion_report', None)
        toolhead = self.printer.lookup_object('toolhead')
        afc = self.printer.lookup_object('AFC', None)

        # Get current state
        active_extruder = toolhead.get_extruder()
        extruder_name = active_extruder.get_name() if active_extruder else "None"
        extrude_factor = gcode_move.extrude_factor

        msg = "Flow Fix Status:\n"
        msg += "  Active Extruder: %s\n" % extruder_name
        msg += "  Extrude Factor: %.3f (%.1f%%)\n" % (extrude_factor, extrude_factor * 100)

        if motion_report:
            status = motion_report.get_status(self.printer.get_reactor().monotonic())
            msg += "  Live Extruder Velocity: %.3f mm/s\n" % status.get('live_extruder_velocity', 0)
            msg += "  Live XYZ Velocity: %.3f mm/s\n" % status.get('live_velocity', 0)

            # Check if trapq exists for active extruder
            if active_extruder:
                ehandler = motion_report.dtrapqs.get(extruder_name)
                msg += "  Trapq for %s: %s\n" % (extruder_name, "Found" if ehandler else "NOT FOUND")
                msg += "  Available trapqs: %s\n" % ", ".join(motion_report.dtrapqs.keys())

        if afc:
            msg += "\nAFC Info:\n"
            msg += "  Current lane: %s\n" % (afc.current if hasattr(afc, 'current') else "Unknown")
            msg += "  In toolchange: %s\n" % (afc.in_toolchange if hasattr(afc, 'in_toolchange') else "Unknown")
            msg += "  Position saved: %s\n" % (afc.position_saved if hasattr(afc, 'position_saved') else "Unknown")
            if hasattr(afc, 'extrude_factor'):
                msg += "  AFC saved extrude_factor: %.3f\n" % afc.extrude_factor

        gcmd.respond_info(msg)

def load_config(config):
    return ToolchangerFlowFix(config)