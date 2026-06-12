# Armored Turtle Automated Filament Changer
#
# Copyright (C) 2024-2026 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# AFC AutoCal — per-spool flow calibration (K), all-or-nothing.
#
# Decoupled from any RFID reader and from the U1 bridge: it keys purely off a
# lane's spool_id, so it works no matter how the spool was identified (RFID,
# scanner, or manual SET_SPOOL_ID).
#
# Behaviour when ``enabled``: on every tool load, for ANY lane —
#   * if the spool already has a stored K in Spoolman  -> apply it
#   * otherwise                                        -> run a calibration,
#                                                         then store the K on
#                                                         the Spoolman spool
# It also re-applies the current lane's K after events that reset pressure
# advance (homing, extruder activation). When disabled it does nothing.
#
# K is persisted per-spool in Spoolman's comment field (afc_flow_k=<value>),
# read/written via the shared SpoolmanClient. The actual measurement is the
# ``calibrate_gcode`` command (default FLOW_CALIBRATE) — point it at a macro
# that handles any purge/discard-bin mechanics if needed.
#
# ── Configuration ───────────────────────────────────────────────────
#   [AFC_autocal]
#   enabled: True                 # master on/off for all lanes
#   calibrate_gcode: FLOW_CALIBRATE   # command run to measure K (default)

from __future__ import annotations
import logging

from extras.AFC_RFID import SpoolmanClient


class AFC_autocal:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.logger = logging.getLogger('AFC_autocal')
        self.afc = None
        self._lane_flow_k = {}   # lane_name -> (spool_id, k)

        self.enabled = config.getboolean('enabled', False)
        self.calibrate_gcode = config.get('calibrate_gcode', 'FLOW_CALIBRATE')

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
        """Return K for a lane: cached, else from Spoolman. None if neither."""
        k = self._get_lane_k(lane)
        if k is None:
            k = self._read_k_from_spoolman(lane)
            if k is not None:
                self._set_lane_k(lane, k)
        return k

    def _calibrate(self, cur_lane, gcmd=None):
        """Run the calibration command, then store/apply/persist the new K.

        :return float or None: the measured K, or None if none produced.
        """
        flow_cal = self.printer.lookup_object('flow_calibrator', None)
        if flow_cal is None:
            (gcmd.error if gcmd else RuntimeError)(
                "AFC_autocal: flow_calibrator not found")
            return None
        ext_name = cur_lane.extruder_obj.name
        k_before = flow_cal._current_k.get(ext_name)
        self.gcode.run_script_from_command(self.calibrate_gcode)
        k_after = flow_cal._current_k.get(ext_name)
        if k_after is None or k_after == k_before:
            self.logger.info(
                "AFC autocal: calibration produced no new K for %s"
                % cur_lane.name)
            return None
        self._set_lane_k(cur_lane, k_after)
        self._apply_lane_k(cur_lane.name)
        self._write_k_to_spoolman(cur_lane, k_after)
        self.logger.info(
            "AFC autocal: calibrated and stored K=%.6f for %s"
            % (k_after, cur_lane.name))
        return k_after

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
            if not self.enabled or self.afc is None or cur_lane is None:
                return
            # Already calibrated on the spool? apply it. Otherwise calibrate.
            if self._ensure_k_loaded(cur_lane) is not None:
                self._apply_lane_k(cur_lane.name)
            else:
                self._calibrate(cur_lane)
        except Exception as e:
            self.logger.warning("AFC_autocal: tool_loaded error: %s" % e)

    def _reapply_current_k(self):
        if not self.enabled or self.afc is None:
            return
        if not getattr(self.afc, 'prep_done', False):
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

    # ── GCode commands (manual overrides, work regardless of enabled) ─

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
        k = self._calibrate(cur_lane, gcmd=gcmd)
        if k is not None:
            gcmd.respond_info(
                "AFC_autocal: stored K=%.6f for %s" % (k, cur_lane.name))
        else:
            gcmd.respond_info("AFC_autocal: calibration produced no new K")


def load_config(config):
    return AFC_autocal(config)
