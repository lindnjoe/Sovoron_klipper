# Armored Turtle Automated Filament Changer
#
# Copyright (C) 2024-2026 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# AFC AutoCal — per-spool flow calibration (K) management.
#
# Decoupled from any RFID reader and from the U1 bridge: it keys purely off a
# lane's spool_id, so it works no matter how the spool was identified (RFID,
# scanner, or manual SET_SPOOL_ID). Responsibilities:
#   * read a spool's stored flow K from Spoolman (comment tag afc_flow_k=...)
#   * cache it per-lane (validated against the spool_id it was stored for)
#   * apply it to the active extruder via the flow_calibrator module
#   * re-apply after events that reset pressure advance (tool load, homing,
#     extruder activation)
#   * optionally run a calibration (FLOW_CALIBRATE) and persist the new K
#
# The heavy calibration *mechanics* (discard-bin extrusion, print_task_config
# sync) live in the flow_calibrator / U1 bridge; this module just invokes
# FLOW_CALIBRATE and stores the result.
#
# ── Configuration ───────────────────────────────────────────────────
#   [AFC_autocal]
#   flow_sync_lanes:  lane0, lane4, lane5   # lanes that sync K to/from Spoolman
#   auto_calibrate_lanes:                   # lanes to auto-run FLOW_CALIBRATE on
#                                           # when loaded with no stored K (opt)

from __future__ import annotations
import logging

from extras.AFC_RFID import SpoolmanClient

SPOOLMAN_FLOW_K_TAG = "afc_flow_k"


class AFC_autocal:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.logger = logging.getLogger('AFC_autocal')
        self.afc = None
        self._lane_flow_k = {}   # lane_name -> (spool_id, k)

        self._flow_sync_lanes = {s.strip() for s in
                                 config.get('flow_sync_lanes', '').split(',')
                                 if s.strip()}
        self._auto_cal_lanes = {s.strip() for s in
                                config.get('auto_calibrate_lanes', '').split(',')
                                if s.strip()}

        self.printer.register_event_handler('klippy:ready', self._handle_ready)
        self.printer.register_event_handler('afc:tool_loaded',
                                            self._handle_tool_loaded)
        self.printer.register_event_handler('homing:home_rails_end',
                                            self._handle_home_rails_end)
        self.printer.register_event_handler('extruder:activate_extruder',
                                            self._handle_activate_extruder)

        self.gcode.register_command(
            'AFC_APPLY_LANE_FLOW_K', self.cmd_APPLY_LANE_FLOW_K,
            desc="Apply stored flow K for the current lane")
        self.gcode.register_command(
            'AFC_CALIBRATE_LANE_FLOW_K', self.cmd_CALIBRATE_LANE_FLOW_K,
            desc="Run flow calibration on the current lane and store K")

    # ── Lifecycle ───────────────────────────────────────────────────

    def _handle_ready(self):
        self.afc = self.printer.lookup_object('AFC', None)
        if self.afc is None:
            self.logger.warning("AFC_autocal: AFC not loaded; disabled")
            return
        self.logger = self.afc.logger

    def _spoolman(self):
        mr = getattr(self.afc, 'moonraker', None) if self.afc else None
        if mr is None or getattr(self.afc, 'spoolman', None) is None:
            return None
        return SpoolmanClient(mr)

    # ── K cache (spool-validated) ───────────────────────────────────

    @staticmethod
    def _norm_spool_id(sid):
        if sid in (None, "", 0, "0"):
            return None
        try:
            return int(sid)
        except (ValueError, TypeError):
            return None

    def _flow_sync_enabled(self, lane) -> bool:
        if lane.name in self._flow_sync_lanes:
            return True
        unit = getattr(lane, 'unit_obj', None)
        return bool(getattr(unit, 'spoolman_flow_sync', False)) if unit else False

    def _set_lane_k(self, lane, k):
        self._lane_flow_k[lane.name] = (
            self._norm_spool_id(getattr(lane, 'spool_id', None)), k)

    def _get_lane_k(self, lane):
        entry = self._lane_flow_k.get(lane.name)
        if entry is None:
            return None
        stored, k = entry
        if stored != self._norm_spool_id(getattr(lane, 'spool_id', None)):
            del self._lane_flow_k[lane.name]
            return None
        return k

    # ── Spoolman read/write (via the shared client) ─────────────────

    def _read_k_from_spoolman(self, lane):
        sid = self._norm_spool_id(getattr(lane, 'spool_id', None))
        if sid is None:
            return None
        client = self._spoolman()
        return client.read_flow_k(sid) if client else None

    def _write_k_to_spoolman(self, lane, k):
        sid = self._norm_spool_id(getattr(lane, 'spool_id', None))
        if sid is None:
            return
        client = self._spoolman()
        if client is not None:
            client.write_flow_k(sid, k)

    # ── Apply via flow_calibrator ───────────────────────────────────

    def _apply_lane_k(self, lane_name):
        entry = self._lane_flow_k.get(lane_name)
        if entry is None:
            return None
        _, k = entry
        flow_cal = self.printer.lookup_object('flow_calibrator', None)
        if flow_cal is None:
            self.logger.warning("AFC_autocal: flow_calibrator not found")
            return None
        ext = self.printer.lookup_object('toolhead').get_extruder()
        ext_name = ext.get_name()
        flow_cal._set_pressure_advance(ext, k)
        flow_cal._current_k[ext_name] = k
        msg = ("AFC autocal: applied K=%.6f for %s on %s"
               % (k, lane_name, ext_name))
        self.logger.info(msg)
        return msg

    def _ensure_k_loaded(self, lane):
        """Return K for a lane: cached, else from Spoolman (if sync on)."""
        k = self._get_lane_k(lane)
        if k is None and self._flow_sync_enabled(lane):
            k = self._read_k_from_spoolman(lane)
            if k is not None:
                self._set_lane_k(lane, k)
        return k

    def _current_lane(self):
        if self.afc is None:
            return None
        try:
            return self.afc.function.get_current_lane_obj()
        except Exception:
            return None

    # ── Event handlers ──────────────────────────────────────────────

    def _handle_tool_loaded(self, cur_lane):
        try:
            if self.afc is None or cur_lane is None:
                return
            if self._ensure_k_loaded(cur_lane) is not None:
                self._apply_lane_k(cur_lane.name)
        except Exception as e:
            self.logger.warning("AFC_autocal: tool_loaded error: %s" % e)

    def _reapply_current_k(self):
        if self.afc is None or not getattr(self.afc, 'prep_done', False):
            return
        state = getattr(self.afc, 'current_state', None)
        if state is not None and str(state).split('.')[-1].lower() != 'idle':
            return
        cur_lane = self._current_lane()
        if cur_lane is None:
            return
        if self._ensure_k_loaded(cur_lane) is not None:
            self._apply_lane_k(cur_lane.name)

    def _handle_home_rails_end(self, homing_state, rails):
        try:
            self._reapply_current_k()
        except Exception as e:
            self.logger.warning("AFC_autocal: home reapply error: %s" % e)

    def _handle_activate_extruder(self):
        try:
            self._reapply_current_k()
        except Exception as e:
            self.logger.warning("AFC_autocal: activate reapply error: %s" % e)

    # ── GCode commands ──────────────────────────────────────────────

    def cmd_APPLY_LANE_FLOW_K(self, gcmd):
        cur_lane = self._current_lane()
        if cur_lane is None:
            gcmd.respond_info("AFC_autocal: no current lane")
            return
        if self._ensure_k_loaded(cur_lane) is None:
            gcmd.respond_info("AFC_autocal: no stored K for %s" % cur_lane.name)
            return
        gcmd.respond_info(self._apply_lane_k(cur_lane.name) or "applied")

    def cmd_CALIBRATE_LANE_FLOW_K(self, gcmd):
        cur_lane = self._current_lane()
        if cur_lane is None:
            gcmd.respond_info("AFC_autocal: no current lane")
            return
        flow_cal = self.printer.lookup_object('flow_calibrator', None)
        if flow_cal is None:
            raise gcmd.error("AFC_autocal: flow_calibrator not found")
        ext_name = cur_lane.extruder_obj.name
        k_before = flow_cal._current_k.get(ext_name)
        self.gcode.run_script_from_command("FLOW_CALIBRATE")
        k_after = flow_cal._current_k.get(ext_name)
        if k_after is not None and k_after != k_before:
            self._set_lane_k(cur_lane, k_after)
            self._apply_lane_k(cur_lane.name)
            if self._flow_sync_enabled(cur_lane):
                self._write_k_to_spoolman(cur_lane, k_after)
            gcmd.respond_info(
                "AFC_autocal: stored K=%.6f for %s" % (k_after, cur_lane.name))
        else:
            gcmd.respond_info("AFC_autocal: calibration produced no new K")


def load_config(config):
    return AFC_autocal(config)
