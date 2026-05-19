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
# This module is instantiated by AFC_Toolchanger when it detects
# the U1 firmware (machine_state_manager config section present).
# It does NOT have a load_config — it is never loaded directly.

from __future__ import annotations
import logging
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    from extras.AFC import afc as AFCMain
    from extras.AFC_Toolchanger import AfcToolchanger
    from gcode import GCodeCommand

PHYSICAL_EXTRUDER_NUM = 4
LOGICAL_EXTRUDER_NUM = 32


class AFCU1Bridge:
    """Bridge between AFC tool mapping and the U1 state machine.

    Provides AFC_U1_PRINT_SETUP which:
      1. Builds the U1 extruder_map_table from AFC's tool_cmds
      2. Configures print_task_config (calibration flags, used extruders)
      3. Runs pre-print calibrations (bed mesh, shaper) while IDLE
      4. Enters PRINTING state
      5. Runs flow calibration for each used physical extruder
    """

    def __init__(self, toolchanger: AfcToolchanger) -> None:
        self.printer = toolchanger.printer
        self.afc: AFCMain = toolchanger.afc
        self.logger = toolchanger.logger
        self.functions = toolchanger.functions
        self.config = toolchanger.printer.lookup_object('configfile')

        self.functions.register_commands(
            self.afc.show_macros,
            "AFC_U1_PRINT_SETUP",
            self.cmd_AFC_U1_PRINT_SETUP,
            self.cmd_AFC_U1_PRINT_SETUP_help,
            self.cmd_AFC_U1_PRINT_SETUP_options,
        )

    def _get_physical_index(self, extruder_name: str) -> Optional[int]:
        """Convert extruder name to physical index (0-3)."""
        if extruder_name == "extruder":
            return 0
        try:
            return int(extruder_name.replace("extruder", ""))
        except (ValueError, AttributeError):
            return None

    def _build_extruder_map(self) -> tuple[list, list]:
        """Build U1 extruder_map_table entries from AFC's tool_cmds.

        Returns (map_entries, used_physical) where:
          map_entries: list of [logical, physical] pairs
          used_physical: sorted list of physical extruder indices in use
        """
        map_entries = []
        used_physical = set()

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

        return map_entries, sorted(used_physical)

    # ── command ──────────────────────────────────────────────────────

    cmd_AFC_U1_PRINT_SETUP_help = (
        "Configure U1 print_task_config from AFC tool mapping "
        "and run pre-print calibrations"
    )
    cmd_AFC_U1_PRINT_SETUP_options = {
        "BED_MESH":          {"type": "int", "default": 0},
        "FLOW_CALIBRATE":    {"type": "int", "default": 0},
        "SHAPER_CALIBRATE":  {"type": "int", "default": 0},
    }

    def cmd_AFC_U1_PRINT_SETUP(self, gcmd: GCodeCommand):
        gcode = self.printer.lookup_object("gcode")
        msm = self.printer.lookup_object("machine_state_manager", None)
        ptc = self.printer.lookup_object("print_task_config", None)

        if msm is None or ptc is None:
            self.logger.info(
                "AFC_U1_PRINT_SETUP: machine_state_manager or "
                "print_task_config not found — skipping"
            )
            return

        bed_mesh = gcmd.get_int("BED_MESH", 0, minval=0, maxval=1)
        flow_calibrate = gcmd.get_int("FLOW_CALIBRATE", 0, minval=0, maxval=1)
        shaper_calibrate = gcmd.get_int("SHAPER_CALIBRATE", 0, minval=0, maxval=1)

        # ── 1. Build extruder map from AFC tool assignments ──────
        map_entries, used_physical = self._build_extruder_map()
        if not map_entries:
            self.logger.info(
                "AFC_U1_PRINT_SETUP: No AFC tool mappings found, "
                "cannot configure print_task_config"
            )
            return

        self.logger.info(
            "AFC_U1_PRINT_SETUP: map={map} used_extruders={used}".format(
                map=map_entries, used=used_physical
            )
        )

        # ── 2. Configure print_task_config via SET_PRINT_TASK_PARAMETERS ─
        flow_calib_ext_str = str(used_physical) if flow_calibrate else "[]"

        gcode.run_script_from_command(
            "SET_PRINT_TASK_PARAMETERS"
            " MAP_TABLE={map_table}"
            " FLOW_CALIBRATE={flow_cal}"
            " FLOW_CALIBRATE_EXTRUDERS={flow_ext}"
            " BED_LEVEL={bed}"
            " SHAPER_CALIBRATE={shaper}".format(
                map_table=str(map_entries),
                flow_cal=1 if flow_calibrate else 0,
                flow_ext=flow_calib_ext_str,
                bed=1 if bed_mesh else 0,
                shaper=1 if shaper_calibrate else 0,
            )
        )

        # ── 3. Run calibrations while still in IDLE state ────────
        if bed_mesh:
            self._run_bed_mesh(gcode)

        if shaper_calibrate:
            self._run_shaper_calibrate(gcode)

        # ── 4. Enter PRINTING state ──────────────────────────────
        gcode.run_script_from_command(
            "SET_MAIN_STATE MAIN_STATE=PRINTING"
        )

        # ── 5. Flow calibration (runs inside PRINTING state) ────
        if flow_calibrate:
            for ext in used_physical:
                self.logger.info(
                    "AFC_U1_PRINT_SETUP: flow calibrating extruder {ext}".format(
                        ext=ext
                    )
                )
                gcode.run_script_from_command(
                    "SM_PRINT_FLOW_CALIBRATE EXTRUDER={ext}".format(ext=ext)
                )

        self.logger.info("AFC_U1_PRINT_SETUP: complete")

    # ── calibration helpers ──────────────────────────────────────

    def _run_bed_mesh(self, gcode):
        self.logger.info("AFC_U1_PRINT_SETUP: running bed mesh calibration")
        gcode.run_script_from_command(
            "SET_MAIN_STATE MAIN_STATE=BED_LEVELING ACTION=BED_LEVELING"
        )
        gcode.run_script_from_command("BED_MESH_CALIBRATE")
        gcode.run_script_from_command(
            "EXIT_TO_IDLE REQ_FROM_STATE=BED_LEVELING"
        )

    def _run_shaper_calibrate(self, gcode):
        self.logger.info("AFC_U1_PRINT_SETUP: running fast shaper calibration")
        gcode.run_script_from_command(
            "SET_MAIN_STATE MAIN_STATE=SHAPER_CALIBRATE ACTION=SHAPER_CALIBRATING"
        )
        gcode.run_script_from_command("SM_FAST_SHAPER_CALIBRATE")
        gcode.run_script_from_command(
            "EXIT_TO_IDLE REQ_FROM_STATE=SHAPER_CALIBRATE"
        )
