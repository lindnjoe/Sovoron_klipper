# Armored Turtle Automated Filament Changer
#
# Copyright (C) 2024-2026 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.
"""ACE unit support for AFC.

This unit integrates with DuckACE (`[ace <name>]`) directly and does not depend
on OpenAMS/FPS objects.

Config section: [AFC_ACE <name>]
"""
from __future__ import annotations

from extras.AFC_unit import afcUnit
from extras.AFC_lane import AFCLaneState


class afcACE(afcUnit):
    """AFC unit implementation for Anycubic ACE (DuckACE)."""

    def __init__(self, config):
        super().__init__(config)
        self.type = "ACE"

        self.ace_name = config.get("ace", "ace")
        self.toolchange_cmd = config.get("ace_toolchange_cmd", "ACE_CHANGE_TOOL")
        self.unload_tool_index = config.getint("ace_unload_tool", -1)
        self.feed_len = config.getfloat("ace_feed_length", 120.0, minval=1.0)
        self.retract_len = config.getfloat("ace_retract_length", 120.0, minval=1.0)
        self.feed_speed = config.getfloat("ace_feed_speed", 25.0, minval=1.0)
        self.retract_speed = config.getfloat("ace_retract_speed", 25.0, minval=1.0)
        self.auto_set_prep_state = config.getboolean("ace_auto_set_prep_state", True)

        self.ace = None
        self.printer.register_event_handler("klippy:ready", self.handle_ready)

    def handle_ready(self):
        self.ace = self.printer.lookup_object(f"ace {self.ace_name}", None)
        if self.ace is None:
            self.logger.warning(
                "[AFC_ACE %s] could not find [ace %s]. "
                "ACE commands will fail until the object exists.",
                self.name,
                self.ace_name,
            )

    def _lane_to_ace_index(self, lane) -> int:
        """Resolve ACE slot index from lane.unit suffix (`<unit>:<slot>`)."""
        raw = getattr(lane, "unit", "")
        if ":" in raw:
            suffix = raw.rsplit(":", 1)[1]
            if suffix.isdigit():
                slot = int(suffix)
                if 1 <= slot <= 4:
                    return slot - 1

        configured = getattr(lane, "map", "")
        if configured and configured.upper().startswith("T") and configured[1:].isdigit():
            tool = int(configured[1:])
            if 0 <= tool <= 3:
                return tool

        raise ValueError(
            f"Unable to map lane '{lane.name}' to ACE index. "
            "Use unit suffix ':1..:4' (recommended) or map: T0..T3."
        )

    def _send_ace_gcode(self, cmd: str) -> bool:
        try:
            self.gcode.run_script_from_command(cmd)
            return True
        except Exception as err:
            self.logger.error("ACE gcode failed: %s (%s)", cmd, err)
            return False

    def _ensure_ace(self) -> bool:
        if self.ace is None:
            self.ace = self.printer.lookup_object(f"ace {self.ace_name}", None)
        return self.ace is not None

    def load_sequence(self, cur_lane, _cur_hub, _cur_extruder):
        """Load filament by commanding DuckACE directly."""
        if not self._ensure_ace():
            self.afc.error.handle_lane_failure(cur_lane, f"ACE object '{self.ace_name}' is not available")
            return False

        try:
            idx = self._lane_to_ace_index(cur_lane)
        except ValueError as err:
            self.afc.error.handle_lane_failure(cur_lane, str(err))
            return False

        # Preferred path: full toolchange managed by DuckACE.
        if self.toolchange_cmd:
            if not self._send_ace_gcode(f"{self.toolchange_cmd} TOOL={idx}"):
                self.afc.error.handle_lane_failure(cur_lane, f"ACE toolchange failed for {cur_lane.name}")
                return False
        else:
            if not self._send_ace_gcode(
                f"ACE_FEED INDEX={idx} LENGTH={self.feed_len:.1f} SPEED={self.feed_speed:.1f}"
            ):
                self.afc.error.handle_lane_failure(cur_lane, f"ACE feed failed for {cur_lane.name}")
                return False

        cur_lane.load_state = True
        cur_lane.loaded_to_hub = True
        if self.auto_set_prep_state:
            cur_lane.prep_state = True

        cur_lane.set_tool_loaded()
        self.afc.current = cur_lane.name
        self.afc.save_vars()
        return True

    def unload_sequence(self, cur_lane, _cur_hub, _cur_extruder):
        """Unload filament by commanding DuckACE directly."""
        if not self._ensure_ace():
            self.afc.error.handle_lane_failure(cur_lane, f"ACE object '{self.ace_name}' is not available")
            return False

        try:
            idx = self._lane_to_ace_index(cur_lane)
        except ValueError as err:
            self.afc.error.handle_lane_failure(cur_lane, str(err))
            return False

        if self.toolchange_cmd:
            # TOOL=-1 is DuckACE's "unload current tool" behavior.
            if not self._send_ace_gcode(f"{self.toolchange_cmd} TOOL={self.unload_tool_index}"):
                self.afc.error.handle_lane_failure(cur_lane, f"ACE unload failed for {cur_lane.name}")
                return False
        else:
            if not self._send_ace_gcode(
                f"ACE_RETRACT INDEX={idx} LENGTH={self.retract_len:.1f} SPEED={self.retract_speed:.1f}"
            ):
                self.afc.error.handle_lane_failure(cur_lane, f"ACE retract failed for {cur_lane.name}")
                return False

        cur_lane.load_state = False
        cur_lane.loaded_to_hub = False
        cur_lane.set_tool_unloaded()
        cur_lane.set_unloaded()
        self.afc.current = None
        self.afc.save_vars()
        return True

    def prep_load(self, lane):
        """ACE has no AFC prep stepper path; assume ready once lane is selected."""
        if self.auto_set_prep_state:
            lane.prep_state = True

    def prep_post_load(self, lane):
        if self.auto_set_prep_state:
            lane.prep_state = True

    def eject_lane(self, lane):
        """Eject lane filament back into ACE using retract command."""
        try:
            idx = self._lane_to_ace_index(lane)
        except ValueError as err:
            self.afc.error.AFC_error(str(err), pause=False)
            return
        self._send_ace_gcode(
            f"ACE_RETRACT INDEX={idx} LENGTH={self.retract_len:.1f} SPEED={self.retract_speed:.1f}"
        )

    def system_Test(self, cur_lane, _delay, assignTcmd, _enable_movement):
        """Basic startup state presentation for ACE lanes."""
        if assignTcmd:
            self.afc.function.TcmdAssign(cur_lane)

        if self.auto_set_prep_state:
            cur_lane.prep_state = True

        if cur_lane.tool_loaded:
            cur_lane.status = AFCLaneState.TOOL_LOADED
            self.lane_tool_loaded_idle(cur_lane)
        elif cur_lane.load_state:
            cur_lane.set_loaded()
        else:
            cur_lane.set_unloaded()
        cur_lane.send_lane_data()
        return True


def load_config_prefix(config):
    return afcACE(config)
