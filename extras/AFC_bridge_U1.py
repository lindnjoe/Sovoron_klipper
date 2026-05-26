# Armored Turtle Automated Filament Changer
#
# Copyright (C) 2024-2026 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# AFC U1 Bridge — integrates AFC with the Snapmaker U1 state machine.
# Loaded automatically by AFC_Toolchanger; no-ops on non-U1 printers.
#
# ── GCode Commands ──────────────────────────────────────────────────
#
#   AFC_PRINT_SETUP_U1 [BED_MESH=0|1] [FLOW_CALIBRATE=0|1]
#                       [SHAPER_CALIBRATE=0|1] [Z_OFFSET=<mm>]
#       Full pre-print orchestration:
#       1. Parses gcode file metadata to determine which tools are used
#       2. Builds extruder_map_table (logical T-index → physical extruder)
#       3. Syncs AFC filament info (type, color, vendor) to print_task_config
#       4. Configures calibration flags (flow, bed mesh, shaper)
#       5. Runs bed foreign-object detection (DEFECT_DETECTION_DETECT_BED)
#       6. Preheats all used extruders to 150°C
#       7. Homes Z (with optional Z_OFFSET) and cleans nozzles
#       8. Optionally runs bed mesh and/or input shaper calibration
#       9. Enters PRINTING state
#      10. Verifies dock/undock switching for all used extruders
#      11. Runs per-lane flow calibration (skips lanes with existing K)
#      12. Loads Spoolman flow K for any remaining uncalibrated lanes
#      13. Pre-extrudes filament for single-lane extruders
#      14. Saves extruder target temperatures for later restoration
#
#   AFC_SYNC_FILAMENT_U1
#       Push AFC lane filament data (type, color, vendor, soft flag) to
#       print_task_config without running calibrations. Useful as a
#       standalone resync after RFID clear events or manual changes.
#
#   AFC_APPLY_LANE_FLOW_K_U1
#       Apply the stored per-lane flow calibration K value for the
#       currently loading or loaded lane. Checks current_loading first
#       (set before poop_cmd runs) then falls back to lane_loaded.
#       Intended for use in poop/purge macros.
#
#   AFC_CALIBRATE_LANE_FLOW_K_U1 [LANE=<name>]
#       Run U1 flow calibration on a specific lane (or the current
#       lane if omitted). Loads the lane if needed (via T-command for
#       shared extruders, AFC_SELECT_TOOL for standalone), syncs
#       filament info, starts feed assist for ACE lanes, runs
#       FLOW_CALIBRATE, and stores K for auto-application on future
#       tool loads. Writes K to Spoolman if spoolman_flow_sync is on.
#
#   AFC_RESUME_RESTORE_TEMPS_U1
#       Pre-resume hook (pre_resume_cmd): re-syncs filament_exist and
#       filament_type to print_task_config so INNER_RESUME won't throw
#       error 523, then restores extruder target temps from the last
#       saved snapshot.
#
#   AFC_RESTORE_TEMPS_U1
#       Manual recovery: restore extruder temps from the last saved
#       snapshot. Reports the restored values.
#
# ── Automatic Behaviors ─────────────────────────────────────────────
#
#   filament_exist patch:
#       Wraps print_task_config.update_filament_exist_flag so the U1's
#       periodic hardware sensor polling cannot reset filament_exist to
#       False for AFC-managed extruders (AFC filament bypasses the
#       native feed-port sensors).
#
#   afc:tool_loaded event:
#       On every tool load: re-syncs filament info to print_task_config
#       (counteracts asynchronous RFID clear events from dock/undock),
#       saves extruder target temps, and applies per-lane flow K.
#       Loads K from Spoolman on-demand if not already in memory.
#
#   afc:lane_inserted event:
#       When filament is inserted into a lane with auto_insert_flow_cal
#       enabled: checks for existing K (memory or Spoolman), homes the
#       printer if needed, loads filament to toolhead via TOOL_LOAD,
#       runs flow calibration, and stores K (to Spoolman if enabled).
#       Skipped during printing or before prep completes.
#
#   extruder:activate_extruder event:
#       Re-applies the current lane's flow K after any extruder
#       activation (e.g. after G28 resets pressure advance).
#
#   homing:home_rails_end event:
#       Re-applies the current lane's flow K after homing completes,
#       since homing can reset extruder state and pressure advance.
#
#   pre_resume_cmd auto-configuration:
#       On U1 printers, automatically sets pre_resume_cmd to
#       AFC_RESUME_RESTORE_TEMPS_U1 if not already configured.
#
#   Deferred Spoolman flow K loading:
#       On klippy:ready, waits for AFC prep to complete, then loads
#       flow K from Spoolman for all lanes in two passes: immediately
#       after prep (for lanes with saved spool_id from lane_data),
#       and again after 10s (for RFID-detected lanes).
#
# ── Spoolman Flow K Sync ────────────────────────────────────────────
#
#   When spoolman_flow_sync is enabled (per-lane or per-unit config):
#   - Flow K is stored in the spool's Spoolman comment as an
#     "afc_flow_k=<value>" tag
#   - K is read from Spoolman on startup, tool load, and lane insert
#   - K is written to Spoolman after calibration (with retry + verify)
#   - K is tagged with spool_id; if the spool changes, stale K is
#     discarded and recalibration is needed

from __future__ import annotations
import copy
import logging
import os
from typing import TYPE_CHECKING, Optional, Dict

if TYPE_CHECKING:
    from extras.AFC_lane import AFCLane
    from gcode import GCodeCommand

PHYSICAL_EXTRUDER_NUM = 4
LOGICAL_EXTRUDER_NUM = 32

SOFT_MATERIALS = {"TPU", "TPE", "FLEX"}


class AFCU1Bridge:
    """Bridge between AFC and the Snapmaker U1 state machine.

    Keeps print_task_config in sync with AFC's filament data so the U1's
    INNER_PAUSE/INNER_RESUME, filament_exist polling, and flow calibrator
    all work correctly with AFC-managed filament that bypasses the native
    feed-port sensors and RFID system.

    Key state:
        _lane_flow_k:           {lane_name: (spool_id, k)} — per-lane flow K cache
        _saved_temps:           {extruder_name: target_temp} — snapshot for restore
        _afc_managed_extruders: set of physical indices with AFC filament loaded
    """

    def __init__(self, config) -> None:
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")
        self._init_logger = logging.getLogger("AFC_bridge_U1")
        self.logger = self._init_logger
        self._afc = None
        self._lane_flow_k = {}  # {lane_name: (spool_id, k_value)}
        self._saved_temps = {}
        self._afc_managed_extruders = set()
        self.physical_extruder_num = PHYSICAL_EXTRUDER_NUM
        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.printer.register_event_handler("afc:tool_loaded", self._handle_tool_loaded)
        self.printer.register_event_handler("afc:lane_inserted", self._handle_lane_inserted)
        self.printer.register_event_handler("extruder:activate_extruder",
                                            self._handle_activate_extruder)
        self.printer.register_event_handler("homing:home_rails_end",
                                            self._handle_home_rails_end)

    def _handle_connect(self):
        self.functions = self.printer.lookup_object('AFC_functions')
        afc = self.printer.lookup_object("AFC")
        self.logger = afc.logger
        self.functions.register_commands(
            afc.show_macros,
            "AFC_PRINT_SETUP_U1",
            self.cmd_AFC_PRINT_SETUP_U1,
            "Configure U1 print_task_config from AFC tool mapping "
            "and run pre-print calibrations",
            self.cmd_AFC_PRINT_SETUP_U1_options,
        )
        self.functions.register_commands(
            afc.show_macros,
            "AFC_SYNC_FILAMENT_U1",
            self.cmd_AFC_SYNC_FILAMENT_U1,
            "Sync AFC lane filament info to U1 print_task_config "
            "without running calibrations",
        )
        self.functions.register_commands(
            afc.show_macros,
            "AFC_APPLY_LANE_FLOW_K_U1",
            self.cmd_AFC_APPLY_LANE_FLOW_K_U1,
            "Apply per-lane flow calibration K for the current lane",
        )
        self.functions.register_commands(
            afc.show_macros,
            "AFC_RESUME_RESTORE_TEMPS_U1",
            self.cmd_AFC_RESUME_RESTORE_TEMPS_U1,
            "Pre-resume: sync AFC filament to U1 and restore extruder temps",
        )
        self.functions.register_commands(
            afc.show_macros,
            "AFC_RESTORE_TEMPS_U1",
            self.cmd_AFC_RESTORE_TEMPS_U1,
            "Restore extruder temps from last saved state",
        )
        self.functions.register_commands(
            afc.show_macros,
            "AFC_CALIBRATE_LANE_FLOW_K_U1",
            self.cmd_AFC_CALIBRATE_LANE_FLOW_K_U1,
            "Run U1 flow calibration on a specific lane and store K",
            self.cmd_AFC_CALIBRATE_LANE_FLOW_K_U1_options,
        )

        msm = self.printer.lookup_object("machine_state_manager", None)
        if msm is not None and afc.pre_resume_cmd is None:
            afc.pre_resume_cmd = "AFC_RESUME_RESTORE_TEMPS_U1"
            self.logger.info(
                "Auto-configured pre_resume_cmd = AFC_RESUME_RESTORE_TEMPS_U1",
                console_only=True)

        self._patch_filament_exist_update()
        self.logger.info("AFC_bridge_U1 initialized")

    def _handle_ready(self):
        """
        Defer Spoolman flow K loading until after prep and RFID are done.
        Registers a reactor callback for ``_deferred_load_flow_k``.
        """
        self.printer.get_reactor().register_callback(self._deferred_load_flow_k)

    def _deferred_load_flow_k(self, eventtime):
        """
        Load Spoolman flow K for all lanes after startup settles.

        Two passes: first picks up lanes whose spool_id was restored from
        saved lane_data, second picks up RFID-detected lanes after they
        have time to scan.

        :param eventtime: Reactor event timestamp (unused, required by callback API)
        """
        reactor = self.printer.get_reactor()
        for _ in range(60):
            if self.afc.prep_done:
                break
            reactor.pause(reactor.monotonic() + 1.0)
        self._apply_spoolman_flow_k()
        reactor.pause(reactor.monotonic() + 10.0)
        self._apply_spoolman_flow_k()

    @property
    def afc(self):
        if self._afc is None:
            self._afc = self.printer.lookup_object("AFC")
        return self._afc

    # ── helpers ──────────────────────────────────────────────────────

    def _get_used_tool_indices(self) -> Optional[set]:
        """
        Parse the current gcode file for filament usage metadata.

        Reads ``; filament used [mm]`` from the gcode header and returns
        indices with non-zero usage.

        :return set or None: Set of tool indices (0-based) that have non-zero
            filament usage, or None if the metadata cannot be found
        """
        sdcard = self.printer.lookup_object("virtual_sdcard", None)
        if sdcard is None:
            return None
        filepath = sdcard.file_path()
        if not filepath or not os.path.exists(filepath):
            return None
        try:
            with open(filepath, "r") as f:
                for line in f:
                    if line.startswith("; filament used [mm]"):
                        parts = line.split("=", 1)
                        if len(parts) < 2:
                            return None
                        values = parts[1].strip().split(",")
                        used = set()
                        for i, val in enumerate(values):
                            try:
                                if float(val.strip()) > 0:
                                    used.add(i)
                            except ValueError:
                                continue
                        return used if used else None
        except (IOError, OSError):
            return None
        return None

    def _get_physical_index(self, extruder_name: str) -> Optional[int]:
        """
        Convert a Klipper extruder name to a 0-based physical index.

        :param extruder_name: Klipper extruder name (e.g. ``extruder``, ``extruder1``)
        :return int or None: Physical extruder index, or None if name is invalid
        """
        if extruder_name == "extruder":
            return 0
        try:
            return int(extruder_name.replace("extruder", ""))
        except (ValueError, AttributeError):
            return None

    def _build_extruder_map(self, used_tools: Optional[set] = None) -> tuple[list, list, Dict[int, "AFCLane"], list]:
        """
        Build U1 extruder_map_table entries from AFC's tool_cmds.

        Maps each logical T-index to the physical extruder its AFC lane
        is assigned to, filtering by ``used_tools`` when provided.

        :param used_tools: If provided, only include these tool indices (0-based).
            Obtained from ``_get_used_tool_indices()``.
        :return tuple: (map_entries, used_physical, phys_to_lane, logical_indices)
            where map_entries is list of [logical, physical] pairs,
            used_physical is sorted list of physical indices,
            phys_to_lane maps physical index to first AFCLane on that extruder,
            logical_indices is sorted list of logical T-indices
        """
        map_entries = []
        used_physical = set()
        logical_indices = []
        phys_to_lane: Dict[int, "AFCLane"] = {}

        for tcmd, lane_name in self.afc.tool_cmds.items():
            if not tcmd.startswith("T"):
                continue
            try:
                logical_index = int(tcmd[1:])
            except ValueError:
                continue
            if logical_index >= LOGICAL_EXTRUDER_NUM:
                continue

            if used_tools is not None and logical_index not in used_tools:
                continue

            lane = self.afc.lanes.get(lane_name)
            if lane is None:
                continue

            phys = self._get_physical_index(lane.extruder_obj.name)
            if phys is None or phys >= self.physical_extruder_num:
                continue

            map_entries.append([logical_index, phys])
            used_physical.add(phys)
            logical_indices.append(logical_index)
            if phys not in phys_to_lane:
                phys_to_lane[phys] = lane

        return map_entries, sorted(used_physical), phys_to_lane, sorted(logical_indices)

    def _build_lanes_per_extruder(self, used_tools: Optional[set] = None) -> Dict[int, list]:
        """
        Map each physical extruder to its lanes used in this print.

        Used by flow calibration to iterate per-lane on shared extruders.

        :param used_tools: If provided, only include these tool indices (0-based)
        :return dict: {phys_index: [(logical_index, lane_obj), ...]} filtered
            by used_tools when available
        """
        result: Dict[int, list] = {}
        for tcmd, lane_name in self.afc.tool_cmds.items():
            if not tcmd.startswith("T"):
                continue
            try:
                logical_index = int(tcmd[1:])
            except ValueError:
                continue
            if logical_index >= LOGICAL_EXTRUDER_NUM:
                continue
            if used_tools is not None and logical_index not in used_tools:
                continue
            lane = self.afc.lanes.get(lane_name)
            if lane is None:
                continue
            phys = self._get_physical_index(lane.extruder_obj.name)
            if phys is None or phys >= self.physical_extruder_num:
                continue
            if phys not in result:
                result[phys] = []
            result[phys].append((logical_index, lane))
        return result

    def _afc_color_to_u1(self, color: Optional[str]) -> tuple[int, str]:
        """
        Convert AFC hex color to U1 ARGB int and RRGGBBAA string.

        :param color: Hex color string (e.g. ``#FF0000`` or ``FF0000FF``), or None
        :return tuple: (argb_int, rgba_string) — defaults to ``(0xFFFFFFFF, "FFFFFFFF")``
            if color is missing or invalid
        """
        if not color:
            return 0xFFFFFFFF, "FFFFFFFF"
        color = color.lstrip("#").upper()
        if len(color) == 6:
            color += "FF"
        if len(color) != 8:
            return 0xFFFFFFFF, "FFFFFFFF"
        try:
            rgb = int(color[:6], 16)
            alpha = int(color[6:8], 16)
            argb = (alpha << 24) | rgb
            return argb, color
        except ValueError:
            return 0xFFFFFFFF, "FFFFFFFF"

    def _sync_filament_to_ptc(self, ptc, phys_to_lane: Dict[int, "AFCLane"]):
        """
        Push AFC lane filament info into U1 print_task_config.

        U1 RFID clear events (triggered by tool dock/undock) asynchronously
        reset filament_type to NONE.  This method re-pushes the correct
        values, backs them up, and saves to disk so the data survives clears.

        Sets filament_vendor, filament_type, filament_sub_type, filament_soft,
        filament_exist, filament_color, filament_color_rgba, filament_color_multi,
        filament_official, filament_sku, and filament_edit for each physical extruder.

        :param ptc: print_task_config Klipper object
        :param phys_to_lane: Dict mapping physical extruder index to AFCLane object
        """
        cfg = ptc.print_task_config

        for phys, lane in phys_to_lane.items():
            material = lane.material or cfg["filament_type"][phys]
            color = lane.color

            cfg["filament_vendor"][phys] = "AFC"
            cfg["filament_type"][phys] = material.upper()
            cfg["filament_sub_type"][phys] = "Basic"
            cfg["filament_soft"][phys] = material.upper() in SOFT_MATERIALS
            cfg["filament_exist"][phys] = True

            argb, rgba_str = self._afc_color_to_u1(color)
            cfg["filament_color"][phys] = argb
            cfg["filament_color_rgba"][phys] = rgba_str
            cfg["filament_color_multi"][phys] = {
                "nums": 1,
                "alpha": (argb >> 24) & 0xFF,
                "mode": 0,
                "colors": [rgba_str[:6]],
            }

            cfg["filament_official"][phys] = False
            cfg["filament_sku"][phys] = 0
            cfg["filament_edit"][phys] = True

        ptc.backup_filament_info()
        if hasattr(ptc, "config_path"):
            self.printer.update_snapmaker_config_file(
                ptc.config_path, cfg, None)

    def _save_extruder_temps(self):
        """
        Snapshot current target temps for all physical extruders.

        Stores into ``_saved_temps`` dict for later restoration via
        ``_restore_extruder_temps()``.
        """
        for i in range(self.physical_extruder_num):
            name = "extruder" if i == 0 else "extruder{}".format(i)
            ext = self.printer.lookup_object(name, None)
            if ext is None:
                continue
            heater = ext.get_heater()
            if heater is not None:
                self._saved_temps[name] = heater.target_temp

    def _restore_extruder_temps(self):
        """
        Restore saved extruder temps via SET_HEATER_TEMPERATURE.

        Uses temps captured by ``_save_extruder_temps()``. Skips extruders
        whose saved target was 0.
        """
        if not self._saved_temps:
            self.logger.info("No saved temps to restore")
            return
        for name, temp in self._saved_temps.items():
            if temp > 0:
                self.gcode.run_script_from_command(
                    "SET_HEATER_TEMPERATURE HEATER={name} TARGET={temp}".format(
                        name=name, temp=temp))
                self.logger.info("Restored %s to %.1f°C" % (name, temp))

    def _patch_filament_exist_update(self):
        """
        Wrap print_task_config.update_filament_exist_flag to force
        filament_exist=True for AFC-managed extruders.

        The U1 polls hardware sensors on every get_status() and resets
        filament_exist to False when AFC filament doesn't trigger the
        native feed-port sensors.  This patch re-asserts True for any
        extruder tracked in ``_afc_managed_extruders``.
        """
        ptc = self.printer.lookup_object("print_task_config", None)
        if ptc is None or not hasattr(ptc, 'update_filament_exist_flag'):
            return
        original_fn = ptc.update_filament_exist_flag
        bridge = self

        def patched_update():
            original_fn()
            cfg = ptc.print_task_config
            filament_exist = cfg.get("filament_exist")
            if filament_exist is None:
                return
            for phys in bridge._afc_managed_extruders:
                if phys < len(filament_exist):
                    filament_exist[phys] = True

        ptc.update_filament_exist_flag = patched_update
        self.logger.info("Patched update_filament_exist_flag for AFC extruders")

    def _apply_print_config(self, ptc, map_entries, used_physical,
                            flow_calibrate, bed_mesh, shaper_calibrate,
                            flow_calib_phys=None):
        """
        Write extruder map + calibration flags directly into print_task_config.

        Updates extruder_map_table, extruders_used, flow_calibrate,
        flow_calib_extruders, auto_bed_leveling, shaper_calibrate, and
        the corresponding reprint_info entries. Saves config to disk.

        :param ptc: print_task_config Klipper object
        :param map_entries: List of [logical_index, physical_index] pairs
        :param used_physical: Sorted list of physical extruder indices in use
        :param flow_calibrate: Whether to enable flow calibration (0 or 1)
        :param bed_mesh: Whether to enable bed mesh calibration (0 or 1)
        :param shaper_calibrate: Whether to enable input shaper calibration (0 or 1)
        :param flow_calib_phys: Set of physical indices to flow-calibrate, or
            None to calibrate all used_physical
        """
        cfg = ptc.print_task_config

        for logical, physical in map_entries:
            cfg["extruder_map_table"][logical] = physical

        cfg["extruders_used"] = [False] * self.physical_extruder_num
        for phys in used_physical:
            cfg["extruders_used"][phys] = True

        calib_set = flow_calib_phys if flow_calib_phys is not None else set(used_physical)
        cfg["flow_calibrate"] = bool(flow_calibrate) and len(calib_set) > 0
        cfg["flow_calib_extruders"] = [False] * self.physical_extruder_num
        if cfg["flow_calibrate"]:
            for phys in calib_set:
                cfg["flow_calib_extruders"][phys] = True

        cfg["auto_bed_leveling"] = bool(bed_mesh)
        cfg["shaper_calibrate"] = bool(shaper_calibrate)

        cfg["reprint_info"]["extruder_map_table"] = copy.deepcopy(
            cfg["extruder_map_table"]
        )
        cfg["reprint_info"]["extruders_used"] = copy.deepcopy(
            cfg["extruders_used"]
        )
        cfg["reprint_info"]["flow_calibrate"] = cfg["flow_calibrate"]
        cfg["reprint_info"]["flow_calib_extruders"] = copy.deepcopy(
            cfg["flow_calib_extruders"]
        )
        cfg["reprint_info"]["auto_bed_leveling"] = cfg["auto_bed_leveling"]
        cfg["reprint_info"]["time_lapse_camera"] = cfg.get(
            "time_lapse_camera", False
        )

        if hasattr(ptc, "config_path"):
            self.printer.update_snapmaker_config_file(
                ptc.config_path, cfg, None
            )

    # ── commands ─────────────────────────────────────────────────────

    cmd_AFC_PRINT_SETUP_U1_options = {
        "BED_MESH":          {"type": "int", "default": 0},
        "FLOW_CALIBRATE":    {"type": "int", "default": 0},
        "SHAPER_CALIBRATE":  {"type": "int", "default": 0},
        "Z_OFFSET":          {"type": "float", "default": 0.0},
    }

    def cmd_AFC_PRINT_SETUP_U1(self, gcmd: "GCodeCommand"):
        """
        Full pre-print orchestration for the Snapmaker U1 with AFC.

        Builds the extruder map from AFC tool assignments, syncs filament info,
        configures calibration flags, runs bed defect detection, preheats, homes Z,
        cleans nozzles, optionally runs bed mesh and shaper calibration, enters
        PRINTING state, verifies extruder switching, runs per-lane flow calibration,
        pre-extrudes filament, and saves extruder temps.

        Usage
        -----
        `AFC_PRINT_SETUP_U1 BED_MESH=<0|1>(optional) FLOW_CALIBRATE=<0|1>(optional) SHAPER_CALIBRATE=<0|1>(optional) Z_OFFSET=<mm>(optional)`

        Example
        ------
        ```
        AFC_PRINT_SETUP_U1 BED_MESH=1 FLOW_CALIBRATE=1
        ```
        """
        msm = self.printer.lookup_object("machine_state_manager", None)
        ptc = self.printer.lookup_object("print_task_config", None)

        if msm is None or ptc is None:
            self.logger.info(
                "AFC_PRINT_SETUP_U1: machine_state_manager or "
                "print_task_config not found — skipping"
            )
            return

        bed_mesh = gcmd.get_int("BED_MESH", 0, minval=0, maxval=1)
        flow_calibrate = gcmd.get_int("FLOW_CALIBRATE", 0, minval=0, maxval=1)
        shaper_calibrate = gcmd.get_int("SHAPER_CALIBRATE", 0, minval=0, maxval=1)
        z_offset = gcmd.get_float("Z_OFFSET", 0.0)

        # ── 1. Build extruder map from AFC tool assignments ──────
        used_tools = self._get_used_tool_indices()
        if used_tools is not None:
            self.logger.info(
                "AFC_PRINT_SETUP_U1: gcode metadata says tools used: %s"
                % sorted(used_tools))
        map_entries, used_physical, phys_to_lane, logical_indices = (
            self._build_extruder_map(used_tools)
        )
        if not map_entries:
            self.logger.info(
                "AFC_PRINT_SETUP_U1: No AFC tool mappings found, "
                "cannot configure print_task_config"
            )
            return

        self.logger.info(
            "AFC_PRINT_SETUP_U1: map=%s used_extruders=%s"
            % (map_entries, used_physical))

        # Track which physical extruders AFC manages
        self._afc_managed_extruders = set(used_physical)

        # ── 1b. Build per-extruder lane usage for flow cal ──────
        lanes_per_ext = self._build_lanes_per_extruder(used_tools)
        flow_calib_phys = set(used_physical) if flow_calibrate else set()

        # ── 2. Sync filament info from AFC lanes to U1 ──────────
        self._sync_filament_to_ptc(ptc, phys_to_lane)

        # ── 3. Configure extruder map + calibration flags ────────
        self._apply_print_config(
            ptc, map_entries, used_physical,
            flow_calibrate, bed_mesh, shaper_calibrate,
            flow_calib_phys,
        )

        # ── 4. Detect bed foreign objects ────────────────────────
        self._run_defect_detection()

        # ── 5. Preheat all used extruders ────────────────────────
        self._run_preheat(used_physical)

        # ── 5a. Home Z before nozzle clean ───────────────────────
        self._run_home_z(z_offset)

        # ── 5b. Clean nozzles after preheat ──────────────────────
        self._run_nozzle_clean()

        # ── 5c. Precise Z home after nozzle clean ─────────────
        if bed_mesh:
            self._run_home_z(z_offset)

        # ── 6. Exit PRINTING for calibrations that need IDLE ───
        if bed_mesh or shaper_calibrate:
            self.gcode.run_script_from_command(
                "EXIT_TO_IDLE REQ_FROM_STATE=PRINTING"
            )

        if bed_mesh:
            self._run_bed_mesh(z_offset)

        if shaper_calibrate:
            self._run_shaper_calibrate()

        # ── 7. Enter PRINTING state ──────────────────────────────
        self.gcode.run_script_from_command(
            "SET_MAIN_STATE MAIN_STATE=PRINTING"
        )

        # ── 8. Verify extruder switching (dock/undock check) ─────
        self._run_switch_check(used_physical)

        # ── 9. Flow calibration (runs inside PRINTING state) ────
        if flow_calibrate and flow_calib_phys:
            self._run_flow_calibrate(ptc, flow_calib_phys, phys_to_lane,
                                     lanes_per_ext)

        # ── 9b. Apply stored Spoolman flow K for all lanes/units ──
        self._apply_spoolman_flow_k()

        # ── 10. Pre-extrude filament for single-lane extruders ────
        # Skip pre-extrude on extruders with multiple lanes in this print —
        # we don't know which lane the slicer uses first, and loading the
        # wrong one just to purge and unload wastes time. The first tool
        # change will prime the correct lane via the normal purge sequence.
        multi_lane_phys = {
            phys for phys, ext_lanes in lanes_per_ext.items()
            if len(ext_lanes) > 1
        }
        for logical, phys in map_entries:
            if phys not in multi_lane_phys:
                self.gcode.run_script_from_command(
                    "SM_PRINT_PREEXTRUDE_FILAMENT INDEX={idx}".format(idx=logical)
                )

        self._save_extruder_temps()
        self.logger.info("AFC_PRINT_SETUP_U1: complete")

    def cmd_AFC_SYNC_FILAMENT_U1(self, gcmd: "GCodeCommand"):
        """
        Push current AFC lane filament data (type, color, vendor) to U1
        print_task_config without running calibrations. Useful as a standalone
        resync after RFID clear events or manual changes.

        Usage
        -----
        `AFC_SYNC_FILAMENT_U1`

        Example
        ------
        ```
        AFC_SYNC_FILAMENT_U1
        ```
        """
        ptc = self.printer.lookup_object("print_task_config", None)
        if ptc is None:
            return
        _, _, phys_to_lane, _ = self._build_extruder_map()
        if phys_to_lane:
            self._sync_filament_to_ptc(ptc, phys_to_lane)

    def cmd_AFC_RESUME_RESTORE_TEMPS_U1(self, gcmd: "GCodeCommand"):
        """
        Pre-resume hook: sync AFC filament data to U1 print_task_config and
        restore extruder target temps from the last saved snapshot. Runs before
        _AFC_RENAMED_RESUME_ (U1's INNER_RESUME) to ensure filament_exist and
        filament_type are correct (prevents error 523).

        Auto-configured as ``pre_resume_cmd`` on U1 printers.

        Usage
        -----
        `AFC_RESUME_RESTORE_TEMPS_U1`

        Example
        ------
        ```
        AFC_RESUME_RESTORE_TEMPS_U1
        ```
        """
        ptc = self.printer.lookup_object("print_task_config", None)
        if ptc is not None:
            _, _, phys_to_lane, _ = self._build_extruder_map()
            if phys_to_lane:
                self._sync_filament_to_ptc(ptc, phys_to_lane)
                self.logger.info(
                    "Pre-resume: synced filament for extruders %s"
                    % sorted(phys_to_lane.keys()))

        self._restore_extruder_temps()

    def cmd_AFC_RESTORE_TEMPS_U1(self, gcmd: "GCodeCommand"):
        """
        Manual recovery: restore extruder target temps from the last saved
        snapshot. Reports the restored values.

        Usage
        -----
        `AFC_RESTORE_TEMPS_U1`

        Example
        ------
        ```
        AFC_RESTORE_TEMPS_U1
        ```
        """
        if not self._saved_temps:
            gcmd.respond_info("AFC_RESTORE_TEMPS_U1: no saved temps available")
            return
        self._restore_extruder_temps()
        gcmd.respond_info(
            "AFC_RESTORE_TEMPS_U1: restored %s" %
            {k: "%.1f" % v for k, v in self._saved_temps.items()})

    cmd_AFC_CALIBRATE_LANE_FLOW_K_U1_options = {
        "LANE": {"type": "string", "default": ""},
    }

    def cmd_AFC_CALIBRATE_LANE_FLOW_K_U1(self, gcmd: "GCodeCommand"):
        """
        Run U1 flow calibration on a specific lane and store the K value.

        Loads the lane if needed (via T-command for shared extruders,
        AFC_SELECT_TOOL for standalone), syncs filament info, starts feed
        assist for ACE lanes, runs FLOW_CALIBRATE, and stores the resulting
        K so it gets applied on future tool loads. Writes K to Spoolman
        if ``spoolman_flow_sync`` is enabled.

        Usage
        -----
        `AFC_CALIBRATE_LANE_FLOW_K_U1 LANE=<lane_name>(optional)`

        Example
        ------
        ```
        AFC_CALIBRATE_LANE_FLOW_K_U1 LANE=lane1
        ```
        """
        lane_name = gcmd.get("LANE", "")
        if not lane_name:
            toolhead = self.printer.lookup_object("toolhead")
            ext_name = toolhead.get_extruder().get_name()
            ext_obj = self.afc.tools.get(ext_name)
            if ext_obj is not None:
                lane_name = ext_obj.lane_loaded
            if not lane_name:
                raise gcmd.error(
                    "AFC_CALIBRATE_LANE_FLOW_K_U1: no LANE specified and "
                    "no lane loaded on active extruder")

        lane = self.afc.lanes.get(lane_name)
        if lane is None:
            raise gcmd.error(
                f"AFC_CALIBRATE_LANE_FLOW_K_U1: lane '{lane_name}' not found")

        ptc = self.printer.lookup_object("print_task_config", None)
        flow_cal = self.printer.lookup_object("flow_calibrator", None)
        if flow_cal is None:
            raise gcmd.error(
                "AFC_CALIBRATE_LANE_FLOW_K_U1: flow_calibrator not found")

        phys = self._get_physical_index(lane.extruder_obj.name)
        if phys is None:
            raise gcmd.error(
                f"AFC_CALIBRATE_LANE_FLOW_K_U1: cannot determine physical "
                f"extruder for {lane_name}")

        ext_name = lane.extruder_obj.name

        # Load the lane's filament into the extruder.
        # For standalone toolheads (1 lane = 1 extruder), just select the tool.
        # For shared extruders (multiple lanes), use the T-command to trigger
        # a full filament swap — unless this lane is already loaded.
        if lane.extruder_obj.is_standalone():
            self.gcode.run_script_from_command(
                "AFC_SELECT_TOOL TOOL={}".format(ext_name))
        elif lane.extruder_obj.lane_loaded == lane_name:
            self.logger.info(
                "Flow cal: %s already loaded on %s, selecting tool"
                % (lane_name, ext_name))
            self.gcode.run_script_from_command(
                "AFC_SELECT_TOOL TOOL={}".format(ext_name))
        else:
            tool_cmd = lane.map
            if tool_cmd:
                self.logger.info(
                    "Flow cal: loading %s via %s on %s"
                    % (lane_name, tool_cmd, ext_name))
                self.gcode.run_script_from_command(tool_cmd)
            else:
                raise gcmd.error(
                    f"AFC_CALIBRATE_LANE_FLOW_K_U1: lane '{lane_name}' has no "
                    f"tool mapping — cannot load for calibration")

        self._exit_discard_bin()

        # Sync filament info so flow_calibrator knows the material
        if ptc is not None:
            self._sync_filament_to_ptc(ptc, {phys: lane})

        # Ensure feed assist is running for ACE lanes during calibration
        self._ensure_feed_assist(lane)

        # Snapshot current K so we can detect a new result
        k_before = flow_cal._current_k.get(ext_name)

        # Reset calibrated flag so flow_calibrator allows a fresh run
        flow_cal._calibrated_in_printing[ext_name] = False

        # Run the calibration
        self.gcode.run_script_from_command("FLOW_CALIBRATE")

        # Check result — _current_k updates regardless of print state
        k_after = flow_cal._current_k.get(ext_name)
        if k_after is not None and k_after != k_before:
            self._set_lane_k(lane, k_after)
            gcmd.respond_info(
                "AFC_CALIBRATE_LANE_FLOW_K_U1: stored K=%.6f for %s on %s"
                % (k_after, lane_name, ext_name))
            self.logger.info(
                "Flow cal complete: K=%.6f for %s on %s"
                % (k_after, lane_name, ext_name))
            if self._spoolman_flow_sync_enabled(lane):
                self._write_flow_k_to_spoolman(lane, k_after)
        else:
            gcmd.respond_info(
                "AFC_CALIBRATE_LANE_FLOW_K_U1: flow calibration did not "
                "produce a new K for %s on %s" % (lane_name, ext_name))

        self._exit_discard_bin()

    # ── calibration helpers ──────────────────────────────────────

    def _run_home_z(self, z_offset=0.0):
        """
        Home Z axis with optional offset.

        :param z_offset: Z offset to apply during homing (default 0.0)
        """
        if z_offset:
            self.gcode.run_script_from_command(
                "G28 Z Z_OFFSET {:.4f}".format(z_offset))
        else:
            self.gcode.run_script_from_command("G28 Z")

    def _run_bed_mesh(self, z_offset=0.0):
        """
        Run adaptive bed mesh calibration.

        :param z_offset: Z offset to apply during probing (default 0.0)
        """
        cmd = "BED_MESH_CALIBRATE PROBE_COUNT=11,11 ADAPTIVE=1 ADAPTIVE_MARGIN=50"
        if z_offset:
            cmd += " Z_OFFSET={:.4f}".format(z_offset)
        self.gcode.run_script_from_command(cmd)

    def _run_shaper_calibrate(self):
        self.gcode.run_script_from_command("SM_FAST_SHAPER_CALIBRATE")

    def _run_defect_detection(self):
        self.gcode.run_script_from_command("DEFECT_DETECTION_DETECT_BED")

    def _ensure_feed_assist(self, lane):
        """
        Start feed assist for an ACE lane if not already active.

        Checks the lane's unit for ACE feed assist capability and starts
        it if the lane's slot is not already running feed assist.

        :param lane: AFCLane object to start feed assist for
        """
        unit = getattr(lane, 'unit_obj', None)
        if unit is None:
            return
        get_slot = getattr(unit, '_get_slot', None)
        use_fa = getattr(unit, '_use_feed_assist', None)
        active = getattr(unit, '_feed_assist_active', None)
        start = getattr(unit, '_start_feed_assist', None)
        if get_slot is None or start is None:
            return
        if use_fa is not None and not use_fa(lane):
            return
        slot = get_slot(lane.name)
        if slot < 0:
            return
        if active is not None and slot in active:
            return
        try:
            start(slot)
        except Exception:
            pass

    def _exit_discard_bin(self):
        self.gcode.run_script_from_command("SNAPMAKER_EXIT_DISCARD_BIN")

    def _run_switch_check(self, used_physical):
        """
        Verify dock/undock switching for all used extruders.

        Selects each extruder tool and exits the discard bin to confirm
        the toolchanger mechanism works for all extruders in this print.

        :param used_physical: Sorted list of physical extruder indices to check
        """
        self.gcode.run_script_from_command(
            "SET_ACTION_CODE ACTION=PRINT_SWITCH_CHECKING"
        )
        for ext in used_physical:
            name = "extruder" if ext == 0 else "extruder{}".format(ext)
            self.gcode.run_script_from_command(
                "AFC_SELECT_TOOL TOOL={}".format(name)
            )
            self._exit_discard_bin()
        self.gcode.run_script_from_command("SET_ACTION_CODE ACTION=IDLE")

    def _run_nozzle_clean(self):
        self.gcode.run_script_from_command("MOVE_TO_DISCARD_FILAMENT_POSITION")
        self.gcode.run_script_from_command("ROUGHLY_CLEAN_NOZZLE")
        self._exit_discard_bin()

    def _run_preheat(self, used_physical, preheat_temp=150):
        """
        Preheat all used extruders to the specified temperature.

        :param used_physical: List of physical extruder indices to preheat
        :param preheat_temp: Target preheat temperature in °C (default 150)
        """
        for ext in used_physical:
            self.gcode.run_script_from_command(
                "SM_PRINT_EXTRUDER_PREHEAT EXTRUDER={ext} TEMP={temp}".format(
                    ext=ext, temp=preheat_temp
                )
            )

    def _run_flow_calibrate(self, ptc, flow_calib_phys, phys_to_lane,
                            lanes_per_ext):
        """
        Run per-lane flow calibration and store K values.

        Skips calibration for lanes that already have a stored K value
        (in-memory from this session, or from Spoolman when
        spoolman_flow_sync is enabled). Calibrates remaining lanes
        individually on their physical extruder.

        Bypasses SM_PRINT_FLOW_CALIBRATE to avoid its ``T{ext} A0`` command
        which triggers U1 RFID clear events that wipe filament_type.

        :param ptc: print_task_config Klipper object
        :param flow_calib_phys: Set of physical extruder indices to calibrate
        :param phys_to_lane: Dict mapping physical index to first AFCLane on that extruder
        :param lanes_per_ext: Dict mapping physical index to list of (logical_index, lane) tuples
        """
        flow_cal = self.printer.lookup_object("flow_calibrator", None)

        for ext in sorted(flow_calib_phys):
            name = "extruder" if ext == 0 else "extruder{}".format(ext)
            ext_lanes = lanes_per_ext.get(ext, [])

            if not ext_lanes:
                lane = phys_to_lane.get(ext)
                if lane is None:
                    continue
                ext_lanes = [(None, lane)]

            for logical_idx, lane in ext_lanes:
                # Skip if we already have K for this spool (memory or Spoolman)
                existing_k = self._get_lane_k(lane)
                if existing_k is None and self._spoolman_flow_sync_enabled(lane):
                    existing_k = self._read_flow_k_from_spoolman(lane)
                    if existing_k is not None:
                        self._set_lane_k(lane, existing_k)
                if existing_k is not None:
                    self.logger.info(
                        "Skipping flow calibration for %s — already has "
                        "K=%.6f" % (lane.name, existing_k))
                    continue

                if (not lane.extruder_obj.is_standalone()
                        and logical_idx is not None):
                    self.logger.info(
                        "Loading %s (T%d) for flow calibration on %s"
                        % (lane.name, logical_idx, name))
                    self.gcode.run_script_from_command(
                        "T{}".format(logical_idx))
                else:
                    self.gcode.run_script_from_command(
                        "AFC_SELECT_TOOL TOOL={}".format(name))

                self._exit_discard_bin()
                self._sync_filament_to_ptc(ptc, {ext: lane})

                self._ensure_feed_assist(lane)

                k_before = (flow_cal._current_k.get(name)
                            if flow_cal else None)

                if flow_cal:
                    flow_cal._calibrated_in_printing[name] = False

                self.gcode.run_script_from_command("FLOW_CALIBRATE")

                k_after = (flow_cal._current_k.get(name)
                           if flow_cal else None)
                if k_after is not None and k_after != k_before:
                    self._set_lane_k(lane, k_after)
                    self.logger.info(
                        "Stored flow K=%.6f for %s (T%s) on %s"
                        % (k_after, lane.name,
                           logical_idx if logical_idx is not None
                           else "?", name))
                    if self._spoolman_flow_sync_enabled(lane):
                        self._write_flow_k_to_spoolman(lane, k_after)
                else:
                    self.logger.info(
                        "Flow calibration did not produce a new K for "
                        "%s on %s (k_before=%s, k_after=%s)"
                        % (lane.name, name, k_before, k_after))

        if self._lane_flow_k:
            self.logger.info(
                "Per-lane flow K after calibration: %s"
                % {name: "%.6f" % k for name, (_, k) in self._lane_flow_k.items()})
        else:
            self.logger.info("No per-lane flow K values stored after calibration")

    @staticmethod
    def _norm_spool_id(sid):
        """
        Normalize spool_id to int or None for consistent comparison.

        :param sid: Raw spool_id (str, int, None, or empty string)
        :return int or None: Normalized spool_id
        """
        if sid is None or sid == "" or sid == 0:
            return None
        try:
            return int(sid)
        except (TypeError, ValueError):
            return None

    def _set_lane_k(self, lane, k: float):
        """
        Store K for a lane, tagged with the current spool_id.

        :param lane: AFCLane object
        :param k: Flow calibration K value to store
        """
        spool_id = self._norm_spool_id(getattr(lane, 'spool_id', None))
        self._lane_flow_k[lane.name] = (spool_id, k)

    def _get_lane_k(self, lane) -> "Optional[float]":
        """
        Get K for a lane, only if the spool_id still matches.

        For identified spools: validates stored spool_id matches current.
        For session-only K (both stored and current are None): returns K
        as long as auto_insert placed it there for this load session.
        Returns None if the lane had a spool and now doesn't (or vice versa).

        :param lane: AFCLane object to look up
        :return float or None: Stored K value, or None if not found or spool changed
        """
        entry = self._lane_flow_k.get(lane.name)
        if entry is None:
            return None
        stored_spool, k = entry
        current_spool = self._norm_spool_id(getattr(lane, 'spool_id', None))
        if stored_spool != current_spool:
            del self._lane_flow_k[lane.name]
            return None
        return k

    def _apply_lane_flow_k(self, lane_name):
        """
        Apply per-lane flow K for the given lane via flow_calibrator.

        Sets pressure_advance on the active extruder and updates
        flow_calibrator's _current_k tracking.

        :param lane_name: Name of the AFC lane to apply K for
        :return str or None: Status message, or None if nothing was applied
        """
        if not self._lane_flow_k:
            return None

        entry = self._lane_flow_k.get(lane_name)
        if entry is None:
            return None
        _, k = entry

        toolhead = self.printer.lookup_object("toolhead")
        printer_ext = toolhead.get_extruder()
        ext_name = printer_ext.get_name()

        flow_cal = self.printer.lookup_object("flow_calibrator", None)
        if flow_cal:
            flow_cal._set_pressure_advance(printer_ext, k)
            flow_cal._current_k[ext_name] = k
            msg = "AFC flow calibration: applied K=%.6f for lane %s on %s" % (
                k, lane_name, ext_name)
            self.logger.info(msg)
            return msg
        return None

    def _apply_spoolman_flow_k(self):
        """
        Load and store flow K from Spoolman for all lanes across all units.

        Called during PREP so that previously calibrated K values are ready
        before printing starts.  Lanes that already have K in memory (e.g.
        from flow calibration in step 9) are skipped. Only queries Spoolman
        for lanes with ``spoolman_flow_sync`` enabled.
        """
        for lane in self.afc.lanes.values():
            if self._get_lane_k(lane) is not None:
                continue
            if not self._spoolman_flow_sync_enabled(lane):
                continue
            k = self._read_flow_k_from_spoolman(lane)
            if k is not None:
                self._set_lane_k(lane, k)

    # ── Spoolman flow K sync ───────────────────────────────────────

    SPOOLMAN_FLOW_K_TAG = "afc_flow_k"

    def _spoolman_flow_sync_enabled(self, lane) -> bool:
        """
        Check if spoolman_flow_sync is enabled for this lane.

        Checks the lane-level config first, then falls back to the unit-level config.

        :param lane: AFCLane object to check
        :return bool: True if spoolman_flow_sync is enabled
        """
        val = getattr(lane, 'spoolman_flow_sync', None)
        if val is not None:
            return bool(val)
        unit = getattr(lane, 'unit_obj', None)
        if unit is not None:
            return bool(getattr(unit, 'spoolman_flow_sync', False))
        return False

    def _read_flow_k_from_spoolman(self, lane) -> "Optional[float]":
        """
        Read flow K from the spool's comment in Spoolman.

        Parses the ``afc_flow_k=<value>`` tag from the spool's comment field.

        :param lane: AFCLane object with a valid spool_id
        :return float or None: Flow K value, or None if not found or on error
        """
        spool_id = getattr(lane, 'spool_id', None)
        if not spool_id:
            return None
        try:
            spool = self.afc.moonraker.get_spool(int(spool_id))
            if spool is None:
                return None
            comment = spool.get("comment") or ""
            k = self._parse_k_from_comment(comment)
            if k is not None:
                self.logger.info(
                    "Read flow K=%.6f from Spoolman spool %s for %s"
                    % (k, spool_id, lane.name))
            return k
        except Exception as e:
            self.logger.error(
                "Failed to read flow K from Spoolman spool %s: %s"
                % (spool_id, e))
        return None

    _SPOOLMAN_WRITE_RETRIES = 3
    _SPOOLMAN_RETRY_DELAY = 2.0

    def _write_flow_k_to_spoolman(self, lane, k: float):
        """
        Write flow K to the spool's comment in Spoolman with retry + verify.

        Updates the ``afc_flow_k=<value>`` tag in the spool's comment field.
        Retries up to ``_SPOOLMAN_WRITE_RETRIES`` times with verify-after-write.

        :param lane: AFCLane object with a valid spool_id
        :param k: Flow calibration K value to write
        """
        spool_id = getattr(lane, 'spool_id', None)
        if not spool_id:
            self.logger.info(
                "Cannot save flow K to Spoolman: no spool_id on %s"
                % lane.name)
            return
        k_str = str(round(k, 6))
        reactor = self.printer.get_reactor()

        for attempt in range(self._SPOOLMAN_WRITE_RETRIES):
            try:
                result = self.afc.moonraker.update_spool_comment_tag(
                    int(spool_id), self.SPOOLMAN_FLOW_K_TAG, k_str)
                if result is None:
                    self.logger.warning(
                        "Spoolman write returned no result for spool %s "
                        "(attempt %d/%d)"
                        % (spool_id, attempt + 1, self._SPOOLMAN_WRITE_RETRIES))
                    reactor.pause(reactor.monotonic() + self._SPOOLMAN_RETRY_DELAY)
                    continue

                verify = self.afc.moonraker.get_spool(int(spool_id))
                if verify is not None:
                    saved_k = self._parse_k_from_comment(
                        verify.get("comment") or "")
                    if saved_k is not None:
                        self.logger.info(
                            "Saved flow K=%.6f to Spoolman spool %s for %s"
                            % (k, spool_id, lane.name))
                        return
                    self.logger.warning(
                        "Write succeeded but verify shows no K in spool %s "
                        "comment (attempt %d/%d)"
                        % (spool_id, attempt + 1, self._SPOOLMAN_WRITE_RETRIES))
                else:
                    self.logger.warning(
                        "Write succeeded but verify read failed for spool %s "
                        "(attempt %d/%d)"
                        % (spool_id, attempt + 1, self._SPOOLMAN_WRITE_RETRIES))
            except Exception as e:
                self.logger.warning(
                    "Spoolman write error for spool %s (attempt %d/%d): %s"
                    % (spool_id, attempt + 1, self._SPOOLMAN_WRITE_RETRIES, e))

            reactor.pause(reactor.monotonic() + self._SPOOLMAN_RETRY_DELAY)

        self.logger.error(
            "Failed to save flow K=%.6f to Spoolman spool %s for %s "
            "after %d attempts"
            % (k, spool_id, lane.name, self._SPOOLMAN_WRITE_RETRIES))

    def _parse_k_from_comment(self, comment: str) -> "Optional[float]":
        """
        Parse ``afc_flow_k=<value>`` from a spool comment string.

        :param comment: Raw comment string from Spoolman spool
        :return float or None: Parsed K value, or None if tag not found
        """
        import re
        m = re.search(r'\bafc_flow_k=([\d.]+)', comment)
        if m:
            return float(m.group(1))
        return None

    def _auto_insert_flow_enabled(self, lane) -> bool:
        """
        Check if auto_insert_flow_cal is enabled for this lane.

        Checks the lane-level config first, then falls back to the unit-level config.

        :param lane: AFCLane object to check
        :return bool: True if auto_insert_flow_cal is enabled
        """
        val = getattr(lane, 'auto_insert_flow_cal', None)
        if val is not None:
            return bool(val)
        unit = getattr(lane, 'unit_obj', None)
        if unit is not None:
            return bool(getattr(unit, 'auto_insert_flow_cal', False))
        return False

    def _auto_calibrate_lane(self, cur_lane):
        """
        Run flow calibration automatically after tool load or lane insert.

        Syncs filament info, starts feed assist, runs FLOW_CALIBRATE,
        stores K, and applies it immediately. For spools with spool_id:
        writes K to Spoolman if ``spoolman_flow_sync`` is enabled.
        For unidentified spools: stores K in memory only for this session.

        :param cur_lane: AFCLane object to calibrate
        """
        lane_name = cur_lane.name
        ext_name = cur_lane.extruder_obj.name
        spool_id = getattr(cur_lane, 'spool_id', None)

        self.logger.info(
            "Auto-insert flow calibration: running for %s (spool_id=%s)"
            % (lane_name, spool_id))

        ptc = self.printer.lookup_object("print_task_config", None)
        if ptc is not None:
            phys = self._get_physical_index(ext_name)
            if phys is not None:
                self._sync_filament_to_ptc(ptc, {phys: cur_lane})

        self._ensure_feed_assist(cur_lane)

        flow_cal = self.printer.lookup_object("flow_calibrator", None)
        if flow_cal is None:
            self.logger.error("Auto-insert flow: flow_calibrator not found")
            return

        k_before = flow_cal._current_k.get(ext_name)
        flow_cal._calibrated_in_printing[ext_name] = False

        self.gcode.run_script_from_command("FLOW_CALIBRATE")

        k_after = flow_cal._current_k.get(ext_name)
        if k_after is not None and k_after != k_before:
            self._set_lane_k(cur_lane, k_after)
            self._apply_lane_flow_k(lane_name)
            self.logger.info(
                "Auto-insert flow: stored K=%.6f for %s (spool_id=%s)"
                % (k_after, lane_name, spool_id))
            if spool_id and self._spoolman_flow_sync_enabled(cur_lane):
                self._write_flow_k_to_spoolman(cur_lane, k_after)
        else:
            self.logger.info(
                "Auto-insert flow: calibration did not produce K for %s"
                % lane_name)

        self._exit_discard_bin()

    def _handle_activate_extruder(self):
        """
        Event handler for ``extruder:activate_extruder``.
        Re-applies flow K when an extruder is activated (e.g. after G28
        resets pressure advance to the extruder's default).
        """
        self._reapply_current_k()

    def _handle_home_rails_end(self, homing_state, rails):
        """
        Event handler for ``homing:home_rails_end``.
        Re-applies flow K after homing completes, since homing can
        reset extruder state and pressure advance.

        :param homing_state: Klipper homing state object
        :param rails: List of rails that were homed
        """
        self._reapply_current_k()

    def _reapply_current_k(self):
        """
        Re-apply flow K for the currently loaded lane on the active extruder.

        Guards against running before prep completes or during non-Idle states.
        Falls back to Spoolman if K is not in memory.
        """
        if not self.afc.prep_done:
            return
        if self.afc.current_state != "Idle":
            return
        cur_lane = self.afc.function.get_current_lane_obj()
        if cur_lane is None:
            return
        lane_name = cur_lane.name
        k = self._get_lane_k(cur_lane)
        if k is None and self._spoolman_flow_sync_enabled(cur_lane):
            k = self._read_flow_k_from_spoolman(cur_lane)
            if k is not None:
                self._set_lane_k(cur_lane, k)
        if k is not None:
            self._apply_lane_flow_k(lane_name)

    def _handle_lane_inserted(self, cur_lane):
        """
        Event handler for ``afc:lane_inserted``.
        Runs auto flow calibration if ``auto_insert_flow_cal`` is enabled for
        the lane. Checks for existing K (memory or Spoolman) before calibrating.
        Loads filament to the toolhead via TOOL_LOAD if not already loaded.

        :param cur_lane: AFCLane object that had filament inserted
        """
        if not self.afc.prep_done:
            return
        if self.afc.function.is_printing():
            return
        if not self._auto_insert_flow_enabled(cur_lane):
            return
        k = self._get_lane_k(cur_lane)
        if k is not None:
            self.logger.info(
                "Lane inserted: %s already has K=%.6f, skipping auto cal"
                % (cur_lane.name, k))
            return
        if self._spoolman_flow_sync_enabled(cur_lane):
            k = self._read_flow_k_from_spoolman(cur_lane)
            if k is not None:
                self._set_lane_k(cur_lane, k)
                self.logger.info(
                    "Lane inserted: %s loaded K=%.6f from Spoolman, skipping auto cal"
                    % (cur_lane.name, k))
                return
        if not cur_lane.tool_loaded:
            if not self.afc.function.check_homed():
                self.logger.info(
                    "Lane inserted: %s — printer not homed, skipping auto cal"
                    % cur_lane.name)
                return
            self.logger.info(
                "Lane inserted: %s — loading to toolhead for auto flow calibration"
                % cur_lane.name)
            if not self.afc.TOOL_LOAD(cur_lane, set_start_time=True):
                self.logger.error(
                    "Lane inserted: %s — TOOL_LOAD failed, skipping auto cal"
                    % cur_lane.name)
                return
        k = self._get_lane_k(cur_lane)
        if k is not None:
            self.logger.info(
                "Lane inserted: %s already calibrated K=%.6f during load"
                % (cur_lane.name, k))
            self._exit_discard_bin()
            return
        self.logger.info(
            "Lane inserted: %s — running auto flow calibration" % cur_lane.name)
        self._auto_calibrate_lane(cur_lane)

    def _handle_tool_loaded(self, cur_lane):
        """
        Event handler for ``afc:tool_loaded`` — sync filament, save temps, apply flow K.

        Wraps ``_do_handle_tool_loaded`` with exception handling so event handler
        errors don't crash the print.

        :param cur_lane: AFCLane object that was just loaded into the toolhead
        """
        try:
            self._do_handle_tool_loaded(cur_lane)
        except Exception as e:
            import traceback
            self.logger.error(
                "tool_loaded handler error: %s" % e,
                traceback=traceback.format_exc())

    def _do_handle_tool_loaded(self, cur_lane):
        """
        Inner implementation of tool_loaded handling.

        Re-syncs filament info to print_task_config (counteracts RFID clear
        events), saves extruder target temps, and applies per-lane flow K
        (from memory or Spoolman).

        :param cur_lane: AFCLane object that was just loaded into the toolhead
        """
        ptc = self.printer.lookup_object("print_task_config", None)
        if ptc is not None:
            phys = self._get_physical_index(cur_lane.extruder_obj.name)
            if phys is not None:
                self._afc_managed_extruders.add(phys)
                self._sync_filament_to_ptc(ptc, {phys: cur_lane})

        self._save_extruder_temps()

        lane_name = cur_lane.name
        spool_id = getattr(cur_lane, 'spool_id', None)
        self.logger.info(
            "tool_loaded: %s spool_id=%s sync=%s memory_k=%s"
            % (lane_name, spool_id,
               self._spoolman_flow_sync_enabled(cur_lane),
               self._lane_flow_k.get(lane_name)))

        k = self._get_lane_k(cur_lane)
        if k is not None:
            self._apply_lane_flow_k(lane_name)
            return
        if self._spoolman_flow_sync_enabled(cur_lane):
            k = self._read_flow_k_from_spoolman(cur_lane)
            if k is not None:
                self._set_lane_k(cur_lane, k)
                self._apply_lane_flow_k(lane_name)
                return


    def cmd_AFC_APPLY_LANE_FLOW_K_U1(self, gcmd: "GCodeCommand"):
        """
        Apply the calibrated flow K for the currently loading/loaded lane.

        Uses AFC's ``current_loading`` (set before poop_cmd runs) since
        ``lane_loaded`` isn't updated until after purge completes. Falls
        back to ``lane_loaded`` for calls outside the load sequence.

        Usage
        -----
        `AFC_APPLY_LANE_FLOW_K_U1`

        Example
        ------
        ```
        AFC_APPLY_LANE_FLOW_K_U1
        ```
        """
        if not self._lane_flow_k:
            gcmd.respond_info(
                "AFC flow calibration: no per-lane K stored "
                "(calibration may not have run)")
            return

        lane_name = self.afc.current_loading
        if not lane_name:
            toolhead = self.printer.lookup_object("toolhead")
            ext_name = toolhead.get_extruder().get_name()
            ext_obj = self.afc.tools.get(ext_name)
            if ext_obj is not None:
                lane_name = ext_obj.lane_loaded

        if not lane_name:
            gcmd.respond_info(
                "AFC flow calibration: no lane currently loading/loaded")
            return

        msg = self._apply_lane_flow_k(lane_name)
        if msg:
            gcmd.respond_info(msg)
        else:
            gcmd.respond_info(
                "AFC flow calibration: no K stored for lane %s "
                "(stored lanes: %s)" % (lane_name, list(self._lane_flow_k.keys())))


def load_config(config):
    return AFCU1Bridge(config)
