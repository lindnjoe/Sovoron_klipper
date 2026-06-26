# Armored Turtle Automated Filament Changer
#
# Copyright (C) 2024-2026 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# AFC U1 Print Setup — Snapmaker U1 pre-print orchestration for AFC.
#
# A focused module that does only the U1 print-start setup: build the U1
# extruder map from AFC's tool assignments, push filament info to
# print_task_config, and run the pre-print calibration sequence. Related
# concerns live in their own modules:
#   - flow-K / auto-calibration .......... AFC_autocal.py (+ SpoolmanClient)
#   - RFID -> Spoolman sync / scanner .... AFC_U1_rfid.py / AFC_RFID.py
#   - snapmaker filament colour .......... AFC_spool.set_snapmaker_filament_params
#   - steppermless feed / unload phase ... AFC_OpenAMS / AFC_ACE (native core)
#   - pause/resume + temp restore ........ upstream
#
# This module delegates all flow-K storage/apply to [AFC_autocal]: it runs the
# U1 FLOW_CALIBRATE orchestration but calls AFC_autocal to read/store/persist K
# so there is a single K cache. If [AFC_autocal] isn't configured, flow
# calibration still runs but the measured K isn't persisted.
#
# No-ops cleanly on non-U1 printers (machine_state_manager / print_task_config
# absent).
#
# ── GCode Commands ──────────────────────────────────────────────────
#   AFC_PRINT_SETUP_U1 [BED_MESH=0|1] [FLOW_CALIBRATE=0|1]
#                      [SHAPER_CALIBRATE=0|1] [Z_OFFSET=<mm>]
#       Full pre-print orchestration (see cmd_AFC_PRINT_SETUP_U1).
#   AFC_SYNC_FILAMENT_U1
#       Push AFC lane filament (type/color/vendor) to print_task_config
#       without running any calibrations.
#
# ── Configuration ───────────────────────────────────────────────────
#   [AFC_U1_print_setup]
#   physical_extruders: 4    # number of physical extruders on the U1 (default 4)

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


class AFCU1PrintSetup:
    """Snapmaker U1 pre-print setup for AFC-managed filament."""

    def __init__(self, config) -> None:
        """Set up the U1 print-setup module.

        :param config: Klipper config wrapper for ``[AFC_U1_print_setup]``;
            reads ``physical_extruders`` (number of physical extruders on the
            U1, default 4).
        """
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")
        self.logger = logging.getLogger("AFC_U1_print_setup")
        self._afc = None
        self._afc_managed_extruders = set()
        self.physical_extruder_num = config.getint(
            "physical_extruders", PHYSICAL_EXTRUDER_NUM, minval=1)
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)

    @property
    def afc(self):
        """The AFC object, looked up and cached on first access.

        :return: the main AFC printer object.
        """
        if self._afc is None:
            self._afc = self.printer.lookup_object("AFC")
        return self._afc

    def _handle_connect(self):
        """Register the print-setup G-code commands and patch filament_exist."""
        self.functions = self.printer.lookup_object("AFC_functions")
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
        self._patch_filament_exist_update()
        self.logger.info("AFC_U1_print_setup initialized")

    # ── Extruder map building ────────────────────────────────────────

    def _get_used_tool_indices(self) -> Optional[set]:
        """Parse the sliced gcode for ``; filament used [mm]`` and return the
        set of 0-based tool indices with non-zero usage (None if unknown)."""
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
        """Convert a Klipper extruder name to its 0-based physical index.

        :param extruder_name: Klipper extruder name (``extruder``,
            ``extruder1`` …).
        :return: the 0-based physical index, or None if it can't be parsed.
        """
        if extruder_name == "extruder":
            return 0
        try:
            return int(extruder_name.replace("extruder", ""))
        except (ValueError, AttributeError):
            return None

    def _build_extruder_map(self, used_tools: Optional[set] = None):
        """Build U1 extruder_map_table entries from AFC's tool_cmds.

        :param used_tools: optional set of 0-based logical tool indices to
            include (filters to tools actually used this print); None = all.
        :return: (map_entries, used_physical, phys_to_lane, logical_indices)
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

        return (map_entries, sorted(used_physical), phys_to_lane,
                sorted(logical_indices))

    def _build_lanes_per_extruder(
            self, used_tools: Optional[set] = None) -> Dict[int, list]:
        """Map each physical extruder to its [(logical_index, lane), ...] used
        in this print (for per-lane flow calibration on shared extruders).

        :param used_tools: optional set of logical tool indices to include;
            None = all mapped tools.
        :return dict: physical extruder index -> list of (logical_index, lane).
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
            result.setdefault(phys, []).append((logical_index, lane))
        return result

    # ── print_task_config writing ────────────────────────────────────

    def _afc_color_to_u1(self, color: Optional[str]):
        """Convert an AFC hex colour to (U1 ARGB int, RRGGBBAA string).
        Defaults to white (0xFFFFFFFF) when missing/invalid.

        :param color: AFC colour as a hex string (``RRGGBB`` or ``RRGGBBAA``,
            optional leading ``#``), or None.
        :return tuple: (ARGB integer, 8-char RRGGBBAA hex string).
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
        """Push AFC lane filament info into U1 print_task_config so the display
        and flow calibrator see AFC-managed filament. Backs up and saves to disk
        so it survives the U1's async RFID-clear events.

        :param ptc: the print_task_config object to write into.
        :param phys_to_lane: physical extruder index -> AFC lane providing its
            filament.
        """
        cfg = ptc.print_task_config

        for phys, lane in phys_to_lane.items():
            material = lane.material or cfg["filament_type"][phys]
            color = lane.color

            cfg["filament_vendor"][phys] = "AFC"
            cfg["filament_type"][phys] = material.upper()
            # Use the spool's real sub-type/variant (stashed on the lane from the
            # RFID tag / Spoolman 'variant' field); fall back to "Basic".
            cfg["filament_sub_type"][phys] = getattr(lane, "sub_type", None) or "Basic"
            cfg["filament_soft"][phys] = material.upper() in SOFT_MATERIALS
            cfg["filament_exist"][phys] = True

            argb, rgba_str = self._afc_color_to_u1(color)
            cfg["filament_color"][phys] = argb
            cfg["filament_color_rgba"][phys] = rgba_str
            # filament_color / _rgba carry the primary colour; filament_color_multi
            # carries the full list for dual/gradient spools. lane.multi_color is
            # the spool's complete colour list (from RFID/Spoolman) — show all of
            # them on the U1 when there's more than one, else just the primary.
            multi = [c.lstrip("#").upper()
                     for c in (getattr(lane, "multi_color", None) or []) if c]
            if len(multi) > 1:
                cfg["filament_color_multi"][phys] = {
                    "nums": len(multi),
                    "alpha": (argb >> 24) & 0xFF,
                    "mode": 1,
                    "colors": multi,
                }
            else:
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

    def _apply_print_config(self, ptc, map_entries, used_physical,
                            flow_calibrate, bed_mesh, shaper_calibrate,
                            flow_calib_phys=None):
        """Write extruder map + calibration flags into print_task_config and
        mirror them into reprint_info, then save to disk.

        :param ptc: the print_task_config object to write into.
        :param map_entries: list of ``[logical_index, physical_index]`` pairs.
        :param used_physical: iterable of physical extruder indices used.
        :param flow_calibrate: whether flow calibration is requested.
        :param bed_mesh: whether auto bed levelling is requested.
        :param shaper_calibrate: whether input-shaper calibration is requested.
        :param flow_calib_phys: optional set of physical extruders to flow
            calibrate (defaults to all ``used_physical``).
        """
        cfg = ptc.print_task_config

        for logical, physical in map_entries:
            cfg["extruder_map_table"][logical] = physical

        cfg["extruders_used"] = [False] * self.physical_extruder_num
        for phys in used_physical:
            cfg["extruders_used"][phys] = True

        calib_set = (flow_calib_phys if flow_calib_phys is not None
                     else set(used_physical))
        cfg["flow_calibrate"] = bool(flow_calibrate) and len(calib_set) > 0
        cfg["flow_calib_extruders"] = [False] * self.physical_extruder_num
        if cfg["flow_calibrate"]:
            for phys in calib_set:
                cfg["flow_calib_extruders"][phys] = True

        cfg["auto_bed_leveling"] = bool(bed_mesh)
        cfg["shaper_calibrate"] = bool(shaper_calibrate)

        cfg["reprint_info"]["extruder_map_table"] = copy.deepcopy(
            cfg["extruder_map_table"])
        cfg["reprint_info"]["extruders_used"] = copy.deepcopy(
            cfg["extruders_used"])
        cfg["reprint_info"]["flow_calibrate"] = cfg["flow_calibrate"]
        cfg["reprint_info"]["flow_calib_extruders"] = copy.deepcopy(
            cfg["flow_calib_extruders"])
        cfg["reprint_info"]["auto_bed_leveling"] = cfg["auto_bed_leveling"]
        cfg["reprint_info"]["time_lapse_camera"] = cfg.get(
            "time_lapse_camera", False)

        if hasattr(ptc, "config_path"):
            self.printer.update_snapmaker_config_file(
                ptc.config_path, cfg, None)

    def _patch_filament_exist_update(self):
        """Force filament_exist=True for AFC-managed extruders. The U1 polls
        hardware feed-port sensors on every get_status() and resets
        filament_exist to False when AFC filament (which bypasses those sensors)
        is loaded; re-assert True for extruders AFC manages."""
        ptc = self.printer.lookup_object("print_task_config", None)
        if ptc is None or not hasattr(ptc, "update_filament_exist_flag"):
            return
        original_fn = ptc.update_filament_exist_flag
        setup = self

        def patched_update():
            """Re-assert filament_exist=True for AFC-managed extruders after the
            U1's own update resets them from the hardware feed sensors."""
            original_fn()
            cfg = ptc.print_task_config
            filament_exist = cfg.get("filament_exist")
            if filament_exist is None:
                return
            for phys in setup._afc_managed_extruders:
                if phys < len(filament_exist):
                    filament_exist[phys] = True

        ptc.update_filament_exist_flag = patched_update
        self.logger.info("Patched update_filament_exist_flag for AFC extruders")

    # ── Commands ─────────────────────────────────────────────────────

    cmd_AFC_PRINT_SETUP_U1_options = {
        "BED_MESH":         {"type": "int", "default": 0},
        "FLOW_CALIBRATE":   {"type": "int", "default": 0},
        "SHAPER_CALIBRATE": {"type": "int", "default": 0},
        "Z_OFFSET":         {"type": "float", "default": 0.0},
    }

    def cmd_AFC_PRINT_SETUP_U1(self, gcmd: "GCodeCommand"):
        """Full pre-print orchestration for the Snapmaker U1 with AFC.

        Builds the extruder map from AFC tool assignments, syncs filament info,
        configures calibration flags, runs bed defect detection, preheats, homes
        Z, cleans nozzles, optionally runs bed mesh / shaper calibration, enters
        PRINTING, verifies extruder switching, runs per-lane flow calibration
        (delegating K storage to AFC_autocal), and pre-extrudes filament.

        Usage
        -----
        `AFC_PRINT_SETUP_U1 BED_MESH=<0|1> FLOW_CALIBRATE=<0|1> SHAPER_CALIBRATE=<0|1> Z_OFFSET=<mm>`

        :param gcmd: the G-code command, providing BED_MESH / FLOW_CALIBRATE /
            SHAPER_CALIBRATE / Z_OFFSET parameters.
        """
        msm = self.printer.lookup_object("machine_state_manager", None)
        ptc = self.printer.lookup_object("print_task_config", None)
        if msm is None or ptc is None:
            self.logger.info(
                "AFC_PRINT_SETUP_U1: machine_state_manager or "
                "print_task_config not found — skipping")
            return

        bed_mesh = gcmd.get_int("BED_MESH", 0, minval=0, maxval=1)
        flow_calibrate = gcmd.get_int("FLOW_CALIBRATE", 0, minval=0, maxval=1)
        shaper_calibrate = gcmd.get_int("SHAPER_CALIBRATE", 0, minval=0, maxval=1)
        z_offset = gcmd.get_float("Z_OFFSET", 0.0)

        # 1. Build extruder map from AFC tool assignments (filtered by usage).
        used_tools = self._get_used_tool_indices()
        if used_tools is not None:
            self.logger.info(
                "AFC_PRINT_SETUP_U1: gcode metadata says tools used: %s"
                % sorted(used_tools))
        map_entries, used_physical, phys_to_lane, logical_indices = (
            self._build_extruder_map(used_tools))
        if not map_entries:
            self.logger.info(
                "AFC_PRINT_SETUP_U1: No AFC tool mappings found, "
                "cannot configure print_task_config")
            return

        self.logger.info(
            "AFC_PRINT_SETUP_U1: map=%s used_extruders=%s"
            % (map_entries, used_physical))

        self._afc_managed_extruders = set(used_physical)

        # M104/M109 use this to block tools not used in this print.
        if used_tools is not None:
            self.afc.print_used_tools = {"T{}".format(i) for i in used_tools}
        else:
            self.afc.print_used_tools = set(self.afc.tool_cmds.keys())

        lanes_per_ext = self._build_lanes_per_extruder(used_tools)
        flow_calib_phys = set(used_physical) if flow_calibrate else set()

        # 2. Sync filament info to the U1 + 3. write the map / calib flags.
        self._sync_filament_to_ptc(ptc, phys_to_lane)
        self._apply_print_config(
            ptc, map_entries, used_physical,
            flow_calibrate, bed_mesh, shaper_calibrate, flow_calib_phys)

        # 4-5. Bed defect scan, preheat, home Z, nozzle clean.
        self._run_defect_detection()
        self._run_preheat(used_physical)
        self._run_home_z(z_offset)
        self._run_nozzle_clean()
        if bed_mesh:
            self._run_nozzle_clean()
            self._run_home_z(z_offset)

        # 6. Calibrations that need IDLE.
        if bed_mesh or shaper_calibrate:
            self.gcode.run_script_from_command(
                "EXIT_TO_IDLE REQ_FROM_STATE=PRINTING")
        if bed_mesh:
            self._run_bed_mesh(z_offset)
        if shaper_calibrate:
            self._run_shaper_calibrate()

        # 7. Enter PRINTING.
        self.gcode.run_script_from_command("SET_MAIN_STATE MAIN_STATE=PRINTING")

        # 8. Verify dock/undock switching.
        self._run_switch_check(used_physical)

        # 9. Per-lane flow calibration (K storage delegated to AFC_autocal).
        if flow_calibrate and flow_calib_phys:
            self._run_flow_calibrate(ptc, flow_calib_phys, phys_to_lane,
                                     lanes_per_ext)

        # 10. Pre-extrude single-lane extruders. Skip extruders with several
        # lanes this print — we don't know which the slicer uses first; the
        # first tool change primes the correct lane via the normal purge.
        multi_lane_phys = {
            phys for phys, ext_lanes in lanes_per_ext.items()
            if len(ext_lanes) > 1}
        for logical, phys in map_entries:
            if phys not in multi_lane_phys:
                self.gcode.run_script_from_command(
                    "SM_PRINT_PREEXTRUDE_FILAMENT INDEX={idx}".format(
                        idx=logical))

        self.logger.info("AFC_PRINT_SETUP_U1: complete")

    def cmd_AFC_SYNC_FILAMENT_U1(self, gcmd: "GCodeCommand"):
        """Push current AFC lane filament data to U1 print_task_config without
        running any calibrations (resync after RFID clears / manual changes).

        :param gcmd: the G-code command (takes no parameters).
        """
        ptc = self.printer.lookup_object("print_task_config", None)
        if ptc is None:
            return
        _, _, phys_to_lane, _ = self._build_extruder_map()
        if phys_to_lane:
            self._sync_filament_to_ptc(ptc, phys_to_lane)

    # ── Pre-print step helpers ───────────────────────────────────────

    def _run_home_z(self, z_offset=0.0):
        """Home the Z axis, applying an optional Z offset.

        :param z_offset: extra Z offset in mm to pass to G28 (0 = none).
        """
        if z_offset:
            self.gcode.run_script_from_command(
                "G28 Z Z_OFFSET {:.4f}".format(z_offset))
        else:
            self.gcode.run_script_from_command("G28 Z")

    def _run_bed_mesh(self, z_offset=0.0):
        """Run an adaptive bed-mesh calibration.

        :param z_offset: optional Z offset in mm applied during the mesh.
        """
        cmd = "BED_MESH_CALIBRATE PROBE_COUNT=11,11 ADAPTIVE=1 ADAPTIVE_MARGIN=50"
        if z_offset:
            cmd += " Z_OFFSET={:.4f}".format(z_offset)
        self.gcode.run_script_from_command(cmd)

    def _run_shaper_calibrate(self):
        """Run the U1 fast input-shaper calibration macro."""
        self.gcode.run_script_from_command("SM_FAST_SHAPER_CALIBRATE")

    def _run_defect_detection(self):
        """Run the U1 bed defect-detection scan."""
        self.gcode.run_script_from_command("DEFECT_DETECTION_DETECT_BED")

    def _exit_discard_bin(self):
        """Exit the U1 discard bin (SNAPMAKER_EXIT_DISCARD_BIN)."""
        self.gcode.run_script_from_command("SNAPMAKER_EXIT_DISCARD_BIN")

    def _ensure_feed_assist(self, lane):
        """Start an ACE/OpenAMS lane's feed assist if its unit supports it and
        it isn't already running (used during flow calibration).

        :param lane: the AFC lane whose unit feed assist to start.
        """
        unit = getattr(lane, "unit_obj", None)
        if unit is None:
            return
        get_slot = getattr(unit, "_get_slot", None)
        use_fa = getattr(unit, "_use_feed_assist", None)
        active = getattr(unit, "_feed_assist_active", None)
        start = getattr(unit, "_start_feed_assist", None)
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

    def _run_switch_check(self, used_physical):
        """Select each used extruder and clear the discard bin to confirm the
        toolchanger docks/undocks for every extruder in this print.

        :param used_physical: iterable of physical extruder indices to check.
        """
        self.gcode.run_script_from_command(
            "SET_ACTION_CODE ACTION=PRINT_SWITCH_CHECKING")
        for ext in used_physical:
            name = "extruder" if ext == 0 else "extruder{}".format(ext)
            self.gcode.run_script_from_command(
                "AFC_SELECT_TOOL TOOL={}".format(name))
            self._exit_discard_bin()
        self.gcode.run_script_from_command("SET_ACTION_CODE ACTION=IDLE")

    def _run_nozzle_clean(self):
        """Move to the discard position, roughly clean the nozzle, exit the bin."""
        self.gcode.run_script_from_command("MOVE_TO_DISCARD_FILAMENT_POSITION")
        self.gcode.run_script_from_command("ROUGHLY_CLEAN_NOZZLE")
        self._exit_discard_bin()

    def _run_preheat(self, used_physical, preheat_temp=150):
        """Preheat each used extruder to a holding temperature.

        :param used_physical: iterable of physical extruder indices to preheat.
        :param preheat_temp: holding temperature in C (default 150).
        """
        for ext in used_physical:
            self.gcode.run_script_from_command(
                "SM_PRINT_EXTRUDER_PREHEAT EXTRUDER={ext} TEMP={temp}".format(
                    ext=ext, temp=preheat_temp))

    def _run_flow_calibrate(self, ptc, flow_calib_phys, phys_to_lane,
                            lanes_per_ext):
        """Per-lane flow calibration, delegating K storage to AFC_autocal.

        For each lane: skip if AFC_autocal already has a K (memory or Spoolman),
        otherwise select the lane's tool, clear the bin, sync filament, ensure
        feed assist, and run the measurement — AFC_autocal._calibrate() runs the
        calibrate command and stores/applies/persists the result. Bypasses the
        U1's SM_PRINT_FLOW_CALIBRATE (its ``T{ext} A0`` triggers RFID clears).

        :param ptc: the print_task_config object (for per-lane filament sync).
        :param flow_calib_phys: set of physical extruder indices to calibrate.
        :param phys_to_lane: physical extruder index -> AFC lane fallback.
        :param lanes_per_ext: physical extruder index -> [(logical_index, lane)]
            for per-lane calibration on shared extruders.
        """
        autocal = self.printer.lookup_object("AFC_autocal", None)
        flow_cal = self.printer.lookup_object("flow_calibrator", None)
        if autocal is None:
            self.logger.info(
                "AFC_PRINT_SETUP_U1: [AFC_autocal] not configured — flow "
                "calibration will run but K will not be persisted")

        for ext in sorted(flow_calib_phys):
            name = "extruder" if ext == 0 else "extruder{}".format(ext)
            ext_lanes = lanes_per_ext.get(ext, [])
            if not ext_lanes:
                lane = phys_to_lane.get(ext)
                if lane is None:
                    continue
                ext_lanes = [(None, lane)]

            for logical_idx, lane in ext_lanes:
                # Already have K for this spool? (cache or Spoolman) — skip.
                if (autocal is not None
                        and autocal._ensure_k_loaded(lane) is not None):
                    self.logger.info(
                        "Skipping flow calibration for %s — already has K"
                        % lane.name)
                    continue

                # Select the lane's tool (loads it for steppered lanes) and
                # clear the discard bin before measuring.
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

                if flow_cal is not None:
                    flow_cal._calibrated_in_printing[name] = False

                if autocal is not None:
                    # Measures (autocal.calibrate_gcode), stores, applies, and
                    # persists to Spoolman — the single K source of truth.
                    k = autocal._calibrate(lane)
                    if k is not None:
                        self.logger.info(
                            "Stored flow K=%.6f for %s (T%s) on %s"
                            % (k, lane.name,
                               logical_idx if logical_idx is not None else "?",
                               name))
                else:
                    # No autocal — run the raw calibration without persistence.
                    self.gcode.run_script_from_command("FLOW_CALIBRATE")
                    self.logger.info(
                        "Ran FLOW_CALIBRATE for %s on %s (K not persisted)"
                        % (lane.name, name))


def load_config(config):
    """Klipper module entry point for [AFC_U1_print_setup].

    :param config: Klipper config wrapper for the section.
    :return: the AFCU1PrintSetup instance.
    """
    return AFCU1PrintSetup(config)
