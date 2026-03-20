# Armored Turtle Automated Filament Changer
#
# Copyright (C) 2024-2026 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.

# ACE Internal Buffer Driver
#
# Uses the ACE unit's built-in feed_assist_count (0-12) as a virtual buffer.
# The ACE hardware manages its own buffer correction via feed assist — this
# driver only tracks state for detection purposes (tool_start, stuck spool)
# and satisfies the buffer interface so AFC's lane/load logic works without
# an external FPS or TurtleNeck sensor.
#
# feed_assist_count semantics (higher = more pressure against extruder):
#   0       -> no filament at extruder / empty / stuck
#   1       -> filament is at the extruder (minimal pressure)
#   2-5     -> trailing zone (light pressure, spool feeding slowly)
#   6       -> neutral (balanced)
#   7-11    -> advancing zone (buffer compressed, significant pressure)
#   12      -> fully compressed against extruder gears
#
# Config example:
#   [AFC_ACE_buffer ACE_buffer1]
#
# Then on the ACE unit:
#   buffer: ACE_buffer1
#
# This eliminates the need for an external filament sensor at the toolhead.

from __future__ import annotations

from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    from extras.AFC_lane import AFCLane
    from extras.AFC_stepper import AFCExtruderStepper

TRAILING_STATE_NAME = "Trailing"
ADVANCING_STATE_NAME = "Advancing"
NEUTRAL_STATE_NAME = "Neutral"


class AFCACEBuffer:
    """Virtual buffer backed by the ACE's internal feed_assist_count."""

    def __init__(self, config):
        self.printer = config.get_printer()
        self.afc = self.printer.load_object(config, 'AFC')
        self.reactor = self.afc.reactor
        self.gcode = self.afc.gcode
        self.logger = self.afc.logger

        self.name = config.get_name().split(' ')[-1]
        self.lanes = {}
        self.last_state = "Unknown"
        self.enable = False
        self.current_lane: Optional[AFCLane | AFCExtruderStepper] = None
        self.advance_state = False
        self.trailing_state = False

        # No hardware pins — ACE buffer is purely virtual
        self.advance_pin = None
        self.trailing_pin = None

        # Thresholds for mapping feed_assist_count (0-12) to buffer state
        self.advance_threshold = config.getint(
            'advance_threshold', 7, minval=1, maxval=12
        )
        self.trailing_threshold = config.getint(
            'trailing_threshold', 2, minval=0, maxval=11
        )
        # Count at or above which we consider filament is at the toolhead.
        # Any non-zero count means filament is at the extruder — default 1.
        self.tool_start_threshold = config.getint(
            'tool_start_threshold', 1, minval=1, maxval=12
        )

        # Current raw count from ACE hardware
        self._feed_assist_count = 0

        # Fault detection — not used for ACE buffer (ACE handles its own),
        # but interface must exist for compatibility
        self.error_sensitivity = 0
        self.fault_sensitivity = 0
        self.filament_error_pos = None
        self.past_extruder_position = None
        self.fault_timer = None
        self.extruder_pos_timer = None

        # Register with AFC buffer registry
        self.afc.buffers[self.name] = self

        # Also register under the AFC_buffer namespace so lane/unit lookup
        # (printer.lookup_object('AFC_buffer <name>')) finds us.
        self.printer.add_object('AFC_buffer {}'.format(self.name), self)

    def __str__(self):
        return self.name

    # ------------------------------------------------------------------
    # feed_assist_count update (called from ACE status polling)
    # ------------------------------------------------------------------
    def update_count(self, count):
        """Update the buffer state from ACE's feed_assist_count.

        Called by the ACE unit each time it polls hardware status.

        :param count: feed_assist_count value (0-12)
        """
        self._feed_assist_count = count

        if count >= self.advance_threshold:
            self.advance_state = True
            self.trailing_state = False
            self.last_state = ADVANCING_STATE_NAME
        elif count <= self.trailing_threshold:
            self.advance_state = False
            self.trailing_state = True
            self.last_state = TRAILING_STATE_NAME
        else:
            self.advance_state = False
            self.trailing_state = False
            self.last_state = NEUTRAL_STATE_NAME

    @property
    def feed_assist_count(self):
        """Current raw feed_assist_count from ACE hardware."""
        return self._feed_assist_count

    @property
    def tool_start_triggered(self):
        """True when feed_assist_count indicates filament is at the toolhead."""
        return self._feed_assist_count >= self.tool_start_threshold

    @property
    def buffer_trailing_triggered(self):
        """True when buffer is in trailing state (spool not feeding)."""
        return self.trailing_state

    def get_fps_value(self):
        """Return normalized pressure value (0.0-1.0) from feed_assist_count.

        Maps 0-12 range to 0.0-1.0 for compatibility with FPS-based code.
        """
        return self._feed_assist_count / 12.0

    # ------------------------------------------------------------------
    # Buffer enable / disable (interface expected by AFCLane)
    # ------------------------------------------------------------------
    def enable_buffer(self, lane):
        """Enable the ACE buffer for the given lane."""
        self.current_lane = lane
        self.enable = True
        self.logger.debug(
            f"{self.name} ACE buffer enabled for {lane.name}"
        )

    def disable_buffer(self):
        """Disable the ACE buffer."""
        self.enable = False
        if self.current_lane is not None:
            self.logger.debug(
                f"{self.name} ACE buffer disabled for "
                f"{self.current_lane.name}"
            )
        self.current_lane = None

    # ------------------------------------------------------------------
    # Multiplier control — no-ops, ACE handles correction internally
    # ------------------------------------------------------------------
    def set_multiplier(self, multiplier):
        pass

    def reset_multiplier(self):
        pass

    # ------------------------------------------------------------------
    # Fault detection — no-ops, ACE has its own detection
    # ------------------------------------------------------------------
    def get_fault_sensitivity(self, sensitivity):
        return 0

    def disable_fault_sensitivity(self):
        pass

    def restore_fault_sensitivity(self):
        pass

    def update_filament_error_pos(self):
        pass

    def fault_detection_enabled(self):
        return False

    def start_fault_detection(self, eventtime, multiplier):
        pass

    def register_lane_endstops(self, lane, query_endstops):
        """No hardware endstops to register for ACE buffer."""
        pass

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------
    def buffer_status(self):
        return self.last_state

    def get_status(self, eventtime=None):
        return {
            'state': self.last_state,
            'lanes': [lane.name for lane in self.lanes.values()],
            'enabled': self.enable,
            'feed_assist_count': self._feed_assist_count,
            'fps_value': round(self.get_fps_value(), 3),
            'advance_state': self.advance_state,
            'trailing_state': self.trailing_state,
            'tool_start_triggered': self.tool_start_triggered,
            'active_lane': (
                self.current_lane.name if self.current_lane else None
            ),
        }


def load_config_prefix(config):
    return AFCACEBuffer(config)
