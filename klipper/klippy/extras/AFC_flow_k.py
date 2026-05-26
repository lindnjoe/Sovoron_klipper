# Armored Turtle Automated Filament Changer
#
# Copyright (C) 2024-2026 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# AFC Flow K — Reads per-spool pressure advance (K) values from Spoolman
# and applies them on tool load / extruder activation. Works on any AFC
# printer that shares Spoolman with a machine that calibrates K values
# (stored as afc_flow_k=<value> in the spool comment field).
#
# Loaded automatically by AFC_Toolchanger; no config section needed.
# Requires spoolman_flow_sync: True on the unit or lane for K to be read.
#
# Applies K directly on the extruder stepper (same approach as U1
# flow_calibrator) and wraps per-extruder SET_PRESSURE_ADVANCE handlers
# to block slicer overrides during printing.

from __future__ import annotations
import re
from typing import TYPE_CHECKING, Optional, Dict, Tuple

from extras.AFC import State

if TYPE_CHECKING:
    from extras.AFC_lane import AFCLane


SPOOLMAN_FLOW_K_TAG = "afc_flow_k"
_K_PATTERN = re.compile(r'\bafc_flow_k=([\d.]+)')


class AFCFlowK:
    """Read per-spool flow K from Spoolman and apply as pressure advance."""

    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")
        self.afc = None
        self.logger = None
        self._lane_flow_k: Dict[str, Tuple[Optional[str], float]] = {}
        self._managed_extruders: set = set()
        self._wrapped_extruders: set = set()

        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.printer.register_event_handler("afc:tool_loaded", self._handle_tool_loaded)
        self.printer.register_event_handler("extruder:activate_extruder",
                                            self._handle_activate_extruder)

    def _handle_connect(self):
        self.afc = self.printer.lookup_object("AFC")
        self.logger = self.afc.logger

    def _handle_ready(self):
        self._wrap_pa_handlers()
        self._load_all_spoolman_k()

    def _load_all_spoolman_k(self):
        """At startup, load K from Spoolman for all lanes with spoolman_flow_sync."""
        if self.afc.moonraker is None:
            return
        for lane in self.afc.lanes.values():
            if self._get_lane_k(lane) is not None:
                continue
            if not self._spoolman_flow_sync_enabled(lane):
                continue
            k = self._read_flow_k_from_spoolman(lane)
            if k is not None:
                self._set_lane_k(lane, k)

    def _handle_tool_loaded(self, cur_lane):
        """On tool load: read K from Spoolman if needed, apply."""
        try:
            if self.logger:
                self.logger.info("AFC flow K: tool_loaded event for %s" % cur_lane.name)
            self._do_handle_tool_loaded(cur_lane)
        except Exception as e:
            if self.logger:
                self.logger.error("AFC_flow_k tool_loaded error: %s" % e)

    def _do_handle_tool_loaded(self, cur_lane):
        k = self._get_lane_k(cur_lane)
        if k is not None:
            self._apply_k(cur_lane, k)
            return

        if not self._spoolman_flow_sync_enabled(cur_lane):
            if self.logger:
                self.logger.info(
                    "AFC flow K: spoolman_flow_sync not enabled for %s"
                    % cur_lane.name)
            return

        k = self._read_flow_k_from_spoolman(cur_lane)
        if k is not None:
            self._set_lane_k(cur_lane, k)
            self._apply_k(cur_lane, k)
        else:
            if self.logger:
                self.logger.info(
                    "AFC flow K: no K found in Spoolman for %s (spool_id=%s)"
                    % (cur_lane.name, getattr(cur_lane, 'spool_id', None)))

    def _handle_activate_extruder(self):
        """Re-apply K when extruder is activated (e.g. after G28)."""
        if not self.afc.prep_done:
            return
        if self.afc.current_state != State.IDLE:
            return
        cur_lane = self.afc.function.get_current_lane_obj()
        if cur_lane is None:
            return

        k = self._get_lane_k(cur_lane)
        if k is None and self._spoolman_flow_sync_enabled(cur_lane):
            k = self._read_flow_k_from_spoolman(cur_lane)
            if k is not None:
                self._set_lane_k(cur_lane, k)
        if k is not None:
            self._apply_k(cur_lane, k)

    # ── K storage (in-memory, keyed by spool_id) ──────────────────

    def _set_lane_k(self, lane, k: float):
        spool_id = self._norm_spool_id(getattr(lane, 'spool_id', None))
        self._lane_flow_k[lane.name] = (spool_id, k)

    def _get_lane_k(self, lane) -> Optional[float]:
        entry = self._lane_flow_k.get(lane.name)
        if entry is None:
            return None
        stored_spool, k = entry
        current_spool = self._norm_spool_id(getattr(lane, 'spool_id', None))
        if stored_spool != current_spool:
            del self._lane_flow_k[lane.name]
            return None
        return k

    @staticmethod
    def _norm_spool_id(val) -> Optional[str]:
        if val is None or val == "" or val == 0:
            return None
        return str(val)

    # ── Apply K directly on extruder stepper (same as U1) ─────────

    def _apply_k(self, lane, k: float):
        ext_obj = getattr(lane, 'extruder_obj', None)
        ext_name = ext_obj.name if ext_obj is not None else "extruder"
        printer_ext = self.printer.lookup_object(ext_name, None)
        if printer_ext is None:
            if self.logger:
                self.logger.error(
                    "AFC flow K: extruder %s not found" % ext_name)
            return
        estepper = getattr(printer_ext, 'extruder_stepper', None)
        if estepper is None:
            if self.logger:
                self.logger.error(
                    "AFC flow K: extruder %s has no extruder_stepper" % ext_name)
            return
        smooth_time = getattr(estepper, 'config_smooth_time',
                              estepper.pressure_advance_smooth_time)
        estepper._set_pressure_advance(k, smooth_time)
        self._managed_extruders.add(ext_name)
        if self.logger:
            self.logger.info(
                "AFC flow K: applied K=%.6f for %s on %s"
                % (k, lane.name, ext_name))

    # ── Block slicer PA overrides during printing ─────────────────

    def _wrap_pa_handlers(self):
        """Wrap per-extruder SET_PRESSURE_ADVANCE mux handlers to block
        slicer changes during printing when AFC manages K."""
        mux_entry = self.gcode.mux_commands.get("SET_PRESSURE_ADVANCE")
        if mux_entry is None:
            return
        _key, values = mux_entry
        for ext_name, orig_func in list(values.items()):
            if ext_name in self._wrapped_extruders:
                continue
            flow_k = self
            def make_wrapper(original, name):
                def wrapper(gcmd):
                    if (name in flow_k._managed_extruders
                            and flow_k.afc is not None
                            and flow_k.afc.function.is_printing()):
                        if flow_k.logger:
                            flow_k.logger.info(
                                "AFC flow K enabled for %s, slicer PA "
                                "change will not take effect" % name)
                        gcmd.respond_info(
                            "AFC flow K enabled, so not take effect.")
                        return
                    original(gcmd)
                return wrapper
            values[ext_name] = make_wrapper(orig_func, ext_name)
            self._wrapped_extruders.add(ext_name)

    # ── Spoolman read ─────────────────────────────────────────────

    def _spoolman_flow_sync_enabled(self, lane) -> bool:
        val = getattr(lane, 'spoolman_flow_sync', None)
        if val is not None:
            return bool(val)
        unit = getattr(lane, 'unit_obj', None)
        if unit is not None:
            return bool(getattr(unit, 'spoolman_flow_sync', False))
        return False

    def _read_flow_k_from_spoolman(self, lane) -> Optional[float]:
        spool_id = getattr(lane, 'spool_id', None)
        if not spool_id:
            return None
        if self.afc.moonraker is None:
            return None
        try:
            spool = self.afc.moonraker.get_spool(int(spool_id))
            if spool is None:
                return None
            comment = spool.get("comment") or ""
            k = self._parse_k_from_comment(comment)
            if k is not None and self.logger:
                self.logger.info(
                    "AFC flow K: read K=%.6f from Spoolman spool %s for %s"
                    % (k, spool_id, lane.name))
            return k
        except Exception as e:
            if self.logger:
                self.logger.error(
                    "AFC flow K: failed to read from Spoolman spool %s: %s"
                    % (spool_id, e))
        return None

    @staticmethod
    def _parse_k_from_comment(comment: str) -> Optional[float]:
        m = _K_PATTERN.search(comment)
        if m:
            return float(m.group(1))
        return None
