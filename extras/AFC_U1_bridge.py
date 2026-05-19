# Armored Turtle Automated Filament Changer
#
# Copyright (C) 2024-2026 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# U1 State Machine Bridge: integrates AFC with the Snapmaker U1's
# machine_state_manager and print_task_config so that calibrations
# (bed mesh, flow calibration, input shaper) and extruder mapping
# are handled automatically during PRINT_START.
#
# Enable by adding [AFC_U1_bridge] to your printer config.

from __future__ import annotations
import copy
import logging
from typing import TYPE_CHECKING, Optional, Dict

if TYPE_CHECKING:
    from extras.AFC_lane import AFCLane
    from gcode import GCodeCommand

PHYSICAL_EXTRUDER_NUM = 4
LOGICAL_EXTRUDER_NUM = 32

SOFT_MATERIALS = {"TPU", "TPE", "FLEX"}


class AFCU1Bridge:
    """Bridge between AFC tool mapping and the U1 state machine.

    Provides AFC_PRINT_SETUP_U1 which:
      1. Builds the U1 extruder_map_table from AFC's tool_cmds
      2. Syncs AFC lane filament info to U1 print_task_config
      3. Configures calibration flags and used extruders
      4. Runs bed defect detection
      5. Preheats all used extruders
      6. Runs pre-print calibrations (bed mesh, shaper) while IDLE
      7. Enters PRINTING state
      8. Verifies extruder switching (dock/undock)
      9. Runs flow calibration for each used physical extruder
     10. Pre-extrudes filament for each used logical index
    """

    def __init__(self, config) -> None:
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")
        self.logger = logging.getLogger("AFC_U1_bridge")
        self._afc = None

        self.gcode.register_command(
            "AFC_PRINT_SETUP_U1",
            self.cmd_AFC_PRINT_SETUP_U1,
            desc="Configure U1 print_task_config from AFC tool mapping "
                 "and run pre-print calibrations",
        )
        self.gcode.register_command(
            "AFC_SYNC_FILAMENT_U1",
            self.cmd_AFC_SYNC_FILAMENT_U1,
            desc="Sync AFC lane filament info to U1 print_task_config "
                 "without running calibrations",
        )

    @property
    def afc(self):
        if self._afc is None:
            self._afc = self.printer.lookup_object("AFC")
        return self._afc

    # ── helpers ──────────────────────────────────────────────────────

    def _get_physical_index(self, extruder_name: str) -> Optional[int]:
        if extruder_name == "extruder":
            return 0
        try:
            return int(extruder_name.replace("extruder", ""))
        except (ValueError, AttributeError):
            return None

    def _build_extruder_map(self) -> tuple[list, list, Dict[int, "AFCLane"], list]:
        """Build U1 extruder_map_table entries from AFC's tool_cmds.

        Returns (map_entries, used_physical, phys_to_lane, logical_indices).
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

            lane = self.afc.lanes.get(lane_name)
            if lane is None:
                continue

            phys = self._get_physical_index(lane.extruder_obj.name)
            if phys is None or phys >= PHYSICAL_EXTRUDER_NUM:
                continue

            map_entries.append([logical_index, phys])
            used_physical.add(phys)
            logical_indices.append(logical_index)
            if phys not in phys_to_lane:
                phys_to_lane[phys] = lane

        return map_entries, sorted(used_physical), phys_to_lane, sorted(logical_indices)

    def _afc_color_to_u1(self, color: Optional[str]) -> tuple[int, str]:
        """Convert AFC hex color to U1 ARGB int and RRGGBBAA string."""
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
        """Push AFC lane filament info into U1 print_task_config."""
        cfg = ptc.print_task_config

        for phys, lane in phys_to_lane.items():
            material = lane.material or "NONE"
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

            self.logger.info(
                "AFC_PRINT_SETUP_U1: extruder%d filament=%s color=%s",
                phys, material, rgba_str[:6]
            )

        ptc.backup_filament_info()

    def _apply_print_config(self, ptc, map_entries, used_physical,
                            flow_calibrate, bed_mesh, shaper_calibrate):
        """Write extruder map + calibration flags directly into print_task_config."""
        cfg = ptc.print_task_config

        for logical, physical in map_entries:
            cfg["extruder_map_table"][logical] = physical

        cfg["extruders_used"] = [False] * PHYSICAL_EXTRUDER_NUM
        for phys in used_physical:
            cfg["extruders_used"][phys] = True

        cfg["flow_calibrate"] = bool(flow_calibrate)
        cfg["flow_calib_extruders"] = [False] * PHYSICAL_EXTRUDER_NUM
        if flow_calibrate:
            for phys in used_physical:
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

    def cmd_AFC_PRINT_SETUP_U1(self, gcmd: "GCodeCommand"):
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

        # ── 1. Build extruder map from AFC tool assignments ──────
        map_entries, used_physical, phys_to_lane, logical_indices = (
            self._build_extruder_map()
        )
        if not map_entries:
            self.logger.info(
                "AFC_PRINT_SETUP_U1: No AFC tool mappings found, "
                "cannot configure print_task_config"
            )
            return

        self.logger.info(
            "AFC_PRINT_SETUP_U1: map=%s used_extruders=%s",
            map_entries, used_physical
        )

        # ── 2. Sync filament info from AFC lanes to U1 ──────────
        self._sync_filament_to_ptc(ptc, phys_to_lane)

        # ── 3. Configure extruder map + calibration flags ────────
        self._apply_print_config(
            ptc, map_entries, used_physical,
            flow_calibrate, bed_mesh, shaper_calibrate,
        )

        # ── 4. Detect bed foreign objects ────────────────────────
        self._run_defect_detection()

        # ── 5. Preheat all used extruders ────────────────────────
        self._run_preheat(used_physical)

        # ── 6. Run calibrations while still in IDLE state ────────
        if bed_mesh:
            self._run_bed_mesh()

        if shaper_calibrate:
            self._run_shaper_calibrate()

        # ── 7. Enter PRINTING state ──────────────────────────────
        self.gcode.run_script_from_command(
            "SET_MAIN_STATE MAIN_STATE=PRINTING"
        )

        # ── 8. Verify extruder switching (dock/undock check) ─────
        self._run_switch_check()

        # ── 9. Flow calibration (runs inside PRINTING state) ────
        if flow_calibrate:
            for ext in used_physical:
                self.logger.info(
                    "AFC_PRINT_SETUP_U1: flow calibrating extruder %d", ext
                )
                self.gcode.run_script_from_command(
                    "SM_PRINT_FLOW_CALIBRATE EXTRUDER={ext}".format(ext=ext)
                )

        # ── 10. Pre-extrude filament for each used logical index ─
        for idx in logical_indices:
            self.logger.info("AFC_PRINT_SETUP_U1: pre-extruding T%d", idx)
            self.gcode.run_script_from_command(
                "SM_PRINT_PREEXTRUDE_FILAMENT INDEX={idx}".format(idx=idx)
            )

        self.logger.info("AFC_PRINT_SETUP_U1: complete")

    def cmd_AFC_SYNC_FILAMENT_U1(self, gcmd: "GCodeCommand"):
        """Push current AFC filament data to U1 outside of print setup."""
        ptc = self.printer.lookup_object("print_task_config", None)
        if ptc is None:
            return
        _, _, phys_to_lane, _ = self._build_extruder_map()
        if phys_to_lane:
            self._sync_filament_to_ptc(ptc, phys_to_lane)
            self.logger.info("AFC_SYNC_FILAMENT_U1: complete")

    # ── calibration helpers ──────────────────────────────────────

    def _run_bed_mesh(self):
        self.logger.info("AFC_PRINT_SETUP_U1: precise Z home before bed mesh")
        self.gcode.run_script_from_command("G28 Z")
        self.logger.info("AFC_PRINT_SETUP_U1: running bed mesh calibration")
        self.gcode.run_script_from_command(
            "SET_MAIN_STATE MAIN_STATE=BED_LEVELING ACTION=BED_LEVELING"
        )
        self.gcode.run_script_from_command("BED_MESH_CALIBRATE")
        self.gcode.run_script_from_command(
            "EXIT_TO_IDLE REQ_FROM_STATE=BED_LEVELING"
        )

    def _run_shaper_calibrate(self):
        self.logger.info("AFC_PRINT_SETUP_U1: running fast shaper calibration")
        self.gcode.run_script_from_command(
            "SET_MAIN_STATE MAIN_STATE=SHAPER_CALIBRATE ACTION=SHAPER_CALIBRATING"
        )
        self.gcode.run_script_from_command("SM_FAST_SHAPER_CALIBRATE")
        self.gcode.run_script_from_command(
            "EXIT_TO_IDLE REQ_FROM_STATE=SHAPER_CALIBRATE"
        )

    def _run_defect_detection(self):
        self.logger.info("AFC_PRINT_SETUP_U1: running bed defect detection")
        self.gcode.run_script_from_command("DEFECT_DETECTION_DETECT_BED")

    def _run_switch_check(self):
        self.logger.info("AFC_PRINT_SETUP_U1: verifying extruder switching")
        self.gcode.run_script_from_command("SM_PRINT_CHECK_SWITCH_EXTRUDER")

    def _run_preheat(self, used_physical, preheat_temp=150):
        for ext in used_physical:
            self.logger.info(
                "AFC_PRINT_SETUP_U1: preheating extruder %d to %dC",
                ext, preheat_temp
            )
            self.gcode.run_script_from_command(
                "SM_PRINT_EXTRUDER_PREHEAT EXTRUDER={ext} TEMP={temp}".format(
                    ext=ext, temp=preheat_temp
                )
            )


def load_config(config):
    return AFCU1Bridge(config)
