# Monkey patch to fix flow rate and filament tracking with toolchangers
#
# This module fixes two issues that occur with toolchangers:
# 1. Volumetric flow (mm³/s) showing 0.0 or not tracking correctly
# 2. AFC lane changes causing negative filament tracking
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

        # Patch motion_report to track active extruder velocity
        motion_report = self.printer.lookup_object('motion_report', None)
        if motion_report is not None:
            self._patch_motion_report(motion_report)
        else:
            logging.warning("toolchanger_flow_fix: motion_report not found!")

        # Patch print_stats to add debugging
        print_stats = self.printer.lookup_object('print_stats', None)
        if print_stats is not None:
            self._patch_print_stats(print_stats)
            logging.info("toolchanger_flow_fix: print_stats patches applied")

        # Try to patch AFC if it exists
        afc = self.printer.lookup_object('AFC', None)
        if afc is not None:
            self._patch_afc(afc)
            logging.info("toolchanger_flow_fix: AFC patches applied")
        else:
            logging.info("toolchanger_flow_fix: AFC not found, skipping AFC patches")

        logging.info("toolchanger_flow_fix: All patches applied successfully")

    def _patch_motion_report(self, motion_report):
        """Patch PrinterMotionReport to track active extruder velocity"""
        original_get_status = motion_report.get_status
        debug_enabled = self.debug_enabled
        call_count = [0]  # Mutable to allow modification in closure

        def patched_get_status(eventtime):
            # Check timing BEFORE calling original (which updates next_status_time)
            should_update = eventtime >= motion_report.next_status_time

            # Call original to get base status
            status = original_get_status(eventtime)

            # Only update extruder velocity when status was actually refreshed
            if should_update and motion_report.dtrapqs:
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
                        else:
                            call_count[0] += 1
                            if debug_enabled and call_count[0] % 100 == 0:
                                logging.warning("toolchanger_flow_fix: trapq position is None "
                                              "for extruder %s at print_time %.3f"
                                              % (extruder_name, print_time))
                    else:
                        call_count[0] += 1
                        if debug_enabled and call_count[0] % 100 == 0:
                            logging.warning("toolchanger_flow_fix: No trapq handler for "
                                          "extruder %s, available: %s"
                                          % (extruder_name, list(motion_report.dtrapqs.keys())))
                else:
                    call_count[0] += 1
                    if debug_enabled and call_count[0] % 100 == 0:
                        logging.warning("toolchanger_flow_fix: No active extruder!")

            return status

        motion_report.get_status = patched_get_status
        logging.info("toolchanger_flow_fix: motion_report patch applied")

    def _patch_print_stats(self, print_stats):
        """Patch print_stats to add filament tracking debugging"""
        original_update_filament = print_stats._update_filament_usage
        original_handle_activate = print_stats._handle_activate_extruder
        debug_enabled = self.debug_enabled

        def patched_update_filament_usage(eventtime):
            old_filament_used = print_stats.filament_used
            old_last_epos = print_stats.last_epos

            original_update_filament(eventtime)

            delta = print_stats.filament_used - old_filament_used
            if debug_enabled and abs(delta) > 0.001:
                logging.info("toolchanger_flow_fix: Filament tracking update - "
                           "used: %.3f (+%.3f), last_epos: %.3f -> %.3f"
                           % (print_stats.filament_used, delta, old_last_epos, print_stats.last_epos))

        def patched_handle_activate_extruder():
            old_last_epos = print_stats.last_epos
            original_handle_activate()
            logging.info("toolchanger_flow_fix: print_stats.handle_activate_extruder() called - "
                       "last_epos reset from %.3f to %.3f"
                       % (old_last_epos, print_stats.last_epos))

        print_stats._update_filament_usage = patched_update_filament_usage
        print_stats._handle_activate_extruder = patched_handle_activate_extruder

    def _patch_afc(self, afc):
        """Patch AFC save_pos/restore_pos to skip toolchange moves in filament tracking"""
        original_save_pos = afc.save_pos
        original_restore_pos = afc.restore_pos
        debug_enabled = self.debug_enabled
        printer = self.printer

        # Store saved E position for calculating delta
        saved_e_pos = [None]  # Use list to make it mutable in closures

        def patched_save_pos():
            # Save current E position before AFC operations
            gcode_move = printer.lookup_object('gcode_move')
            reactor = printer.get_reactor()
            eventtime = reactor.monotonic()
            gc_status = gcode_move.get_status(eventtime)
            saved_e_pos[0] = gc_status['position'].e

            if debug_enabled:
                logging.info("toolchanger_flow_fix: AFC save_pos() - saved E position: %.3f"
                           % saved_e_pos[0])

            original_save_pos()

        def patched_restore_pos(move_z_first=True):
            original_restore_pos(move_z_first)

            # Calculate how much E changed during AFC operations and compensate
            print_stats = printer.lookup_object('print_stats', None)
            gcode_move = printer.lookup_object('gcode_move')
            if print_stats and saved_e_pos[0] is not None:
                reactor = printer.get_reactor()
                eventtime = reactor.monotonic()
                gc_status = gcode_move.get_status(eventtime)
                current_e = gc_status['position'].e
                e_delta = current_e - saved_e_pos[0]

                # Adjust last_epos by the AFC delta to skip toolchange moves
                old_last_epos = print_stats.last_epos
                print_stats.last_epos += e_delta

                if debug_enabled:
                    logging.info("toolchanger_flow_fix: AFC restore_pos() - E changed by %.3f "
                               "(%.3f -> %.3f), adjusted last_epos from %.3f to %.3f"
                               % (e_delta, saved_e_pos[0], current_e, old_last_epos, print_stats.last_epos))
                elif e_delta != 0:
                    logging.info("toolchanger_flow_fix: AFC restore_pos() - compensated for "
                               "%.3fmm E delta from lane change" % e_delta)

                saved_e_pos[0] = None  # Reset for next use

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
        print_stats = self.printer.lookup_object('print_stats', None)

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

        if print_stats:
            stats = print_stats.get_status(self.printer.get_reactor().monotonic())
            msg += "\nPrint Stats:\n"
            msg += "  State: %s\n" % stats.get('state', 'unknown')
            msg += "  Filament used: %.3f mm\n" % stats.get('filament_used', 0)
            msg += "  Total duration: %.1f s\n" % stats.get('total_duration', 0)
            msg += "  Print duration: %.1f s\n" % stats.get('print_duration', 0)
            msg += "  Last E position: %.3f\n" % print_stats.last_epos

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