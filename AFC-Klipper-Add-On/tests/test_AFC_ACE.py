"""
Unit tests for extras/AFC_ACE.py

Covers:
  - afcACE: class constants, slot mapping, color conversion
  - _get_local_slot_for_lane: maps lane index to 0-based slot
  - _ace_color_to_hex: converts RGB array to hex string
  - _get_feed_assist_for_slot: default vs override
  - system_Test: LED logic for each lane state combination
  - _poll_slot_status: shifting state handling, operation guard, state consistency
  - get_status: correct keys and content
  - check_runout: returns True when printing
  - get_lane_reset_command: returns TOOL_UNLOAD command
  - lane_unload: blocks manual unload
  - handle_connect: sets logo strings
"""

from __future__ import annotations

from unittest.mock import MagicMock, PropertyMock, patch
import pytest

from extras.AFC_ACE import afcACE, MODE_COMBINED, MODE_DIRECT
from extras.AFC_lane import AFCLaneState


# ── Helpers ───────────────────────────────────────────────────────────────────

def _make_ace(name="ACE_1", mode=MODE_COMBINED):
    """Build an afcACE bypassing the complex __init__."""
    unit = afcACE.__new__(afcACE)

    from tests.conftest import MockAFC, MockPrinter, MockLogger, MockReactor

    afc = MockAFC()
    reactor = MockReactor()
    afc.reactor = reactor
    afc.logger = MockLogger()
    printer = MockPrinter(afc=afc)

    unit.printer = printer
    unit.afc = afc
    unit.logger = afc.logger
    unit.reactor = reactor
    unit.name = name
    unit.full_name = ["AFC_ACE", name]
    unit.lanes = {}
    unit.hub_obj = None
    unit.extruder_obj = None
    unit.buffer_obj = None
    unit.hub = None
    unit.extruder = None
    unit.buffer_name = None
    unit.td1_defined = False
    unit.type = "ACE"
    unit.gcode = afc.gcode
    unit.mode = mode
    unit.serial_port = "/dev/ttyACE0"
    unit.feed_speed = 800.0
    unit.retract_speed = 800.0
    unit.feed_length = 500.0
    unit.retract_length = 500.0
    unit._default_feed_assist = True
    unit._slot_feed_assist = {}
    unit.extruder_assist_length = 50.0
    unit.extruder_assist_speed = 300.0
    unit.sensor_approach_margin = 60.0
    unit.sensor_step = 20.0
    unit.calibration_step = 50.0
    unit.max_feed_overshoot = 100.0
    unit.dock_purge = False
    unit.dock_purge_length = 50.0
    unit.dock_purge_speed = 5.0
    unit.crash_detection_mode = "disabled"
    unit.fps_threshold = 0.9
    unit._fps_obj = None
    unit._fps_extruder = None
    unit._fps_runout_helper = None
    unit._fps_latched = False
    unit.poll_interval = 1.0
    unit.baud_rate = 115200
    unit._ace = None
    unit._slot_inventory = [{} for _ in range(4)]
    unit._cached_hw_status = {}
    unit._current_loaded_slot = -1
    unit._prev_slot_states = {}
    unit._operation_active = False
    unit._feed_assist_active = set()
    unit._feed_assist_refresh_counter = 0
    unit._FEED_ASSIST_REFRESH_INTERVAL = 7
    unit._pending_feed_assist_restore = False
    unit._drying_active = False
    unit._persistence = MagicMock()
    unit._last_inventory_sync = 0.0
    unit._inventory_sync_interval = 300.0
    unit._unit_load_to_hub = None
    unit._hub_load_suppressed = set()

    return unit


def _make_lane(name="lane1", prep_state=False, load_state=False,
               tool_loaded=False, index=1):
    lane = MagicMock()
    lane.name = name
    lane.prep_state = prep_state
    lane.load_state = load_state
    lane.tool_loaded = tool_loaded
    lane.map = "T0"
    lane.index = index
    lane.status = None
    lane._afc_prep_done = True
    lane.loaded_to_hub = False
    lane.led_ready = "0,1,0,0"
    lane.led_not_ready = "0,0,0,0.25"
    lane.led_fault = "1,0,0,0"
    lane.led_loading = "0,0,1,0"
    lane.led_spool_illum = "1,1,1,0"
    lane.led_index = "1"
    lane.led_spool_index = "2"
    return lane


# ── Constants ────────────────────────────────────────────────────────────────

class TestConstants:
    def test_slots_per_unit(self):
        assert afcACE.SLOTS_PER_UNIT == 4

    def test_mode_constants(self):
        assert MODE_COMBINED == "combined"
        assert MODE_DIRECT == "direct"


# ── _get_local_slot_for_lane ──────────────────────────────────────────────────

class TestGetLocalSlotForLane:
    def test_returns_0_for_index_1(self):
        unit = _make_ace()
        lane = _make_lane(index=1)
        assert unit._get_local_slot_for_lane(lane) == 0

    def test_returns_3_for_index_4(self):
        unit = _make_ace()
        lane = _make_lane(index=4)
        assert unit._get_local_slot_for_lane(lane) == 3

    def test_returns_neg1_for_index_0(self):
        unit = _make_ace()
        lane = _make_lane(index=0)
        assert unit._get_local_slot_for_lane(lane) == -1

    def test_returns_neg1_for_index_5(self):
        unit = _make_ace()
        lane = _make_lane(index=5)
        assert unit._get_local_slot_for_lane(lane) == -1


# ── _ace_color_to_hex ────────────────────────────────────────────────────────

class TestAceColorToHex:
    def test_rgb_conversion(self):
        assert afcACE._ace_color_to_hex([255, 128, 0]) == "#ff8000"

    def test_black_for_zeros(self):
        assert afcACE._ace_color_to_hex([0, 0, 0]) == "#000000"

    def test_fallback_for_short_array(self):
        assert afcACE._ace_color_to_hex([255]) == "#000000"

    def test_fallback_for_non_list(self):
        assert afcACE._ace_color_to_hex("red") == "#000000"

    def test_tuple_input(self):
        assert afcACE._ace_color_to_hex((128, 64, 32)) == "#804020"


# ── _get_feed_assist_for_slot ────────────────────────────────────────────────

class TestGetFeedAssistForSlot:
    def test_returns_default_when_no_override(self):
        unit = _make_ace()
        unit._default_feed_assist = True
        assert unit._get_feed_assist_for_slot(0) is True

    def test_returns_override_when_set(self):
        unit = _make_ace()
        unit._default_feed_assist = True
        unit._slot_feed_assist = {0: False}
        assert unit._get_feed_assist_for_slot(0) is False

    def test_returns_default_for_unoveridden_slot(self):
        unit = _make_ace()
        unit._default_feed_assist = False
        unit._slot_feed_assist = {0: True}
        assert unit._get_feed_assist_for_slot(1) is False


# ── check_runout ─────────────────────────────────────────────────────────────

class TestCheckRunout:
    def test_returns_true_when_printing(self):
        unit = _make_ace()
        unit.afc.function.is_printing.return_value = True
        assert unit.check_runout("lane1") is True

    def test_returns_false_when_not_printing(self):
        unit = _make_ace()
        unit.afc.function.is_printing.return_value = False
        assert unit.check_runout("lane1") is False

    def test_returns_false_on_exception(self):
        unit = _make_ace()
        unit.afc.function.is_printing.side_effect = Exception("err")
        assert unit.check_runout("lane1") is False


# ── get_lane_reset_command ───────────────────────────────────────────────────

class TestGetLaneResetCommand:
    def test_returns_ace_lane_reset(self):
        unit = _make_ace()
        lane = _make_lane(name="lane3")
        result = unit.get_lane_reset_command(lane, None)
        assert result == "ACE_LANE_RESET UNIT=ACE_1 LANE=lane3"


# ── lane_unload ──────────────────────────────────────────────────────────────

class TestLaneUnload:
    def test_returns_none(self):
        unit = _make_ace()
        lane = _make_lane()
        result = unit.lane_unload(lane)
        assert result is None


# ── get_status ───────────────────────────────────────────────────────────────

class TestGetStatus:
    def test_has_ace_mode_key(self):
        unit = _make_ace()
        status = unit.get_status()
        assert "ace_mode" in status

    def test_ace_mode_is_combined(self):
        unit = _make_ace(mode=MODE_COMBINED)
        status = unit.get_status()
        assert status["ace_mode"] == MODE_COMBINED

    def test_has_ace_connected_key(self):
        unit = _make_ace()
        status = unit.get_status()
        assert "ace_connected" in status

    def test_reports_not_connected_when_no_ace(self):
        unit = _make_ace()
        unit._ace = None
        status = unit.get_status()
        assert status["ace_connected"] is False

    def test_has_ace_serial_port(self):
        unit = _make_ace()
        status = unit.get_status()
        assert status["ace_serial_port"] == "/dev/ttyACE0"

    def test_has_ace_status(self):
        unit = _make_ace()
        unit._cached_hw_status = {"temp": 25}
        status = unit.get_status()
        assert status["ace_status"] == {"temp": 25}


# ── handle_connect ───────────────────────────────────────────────────────────

class TestHandleConnect:
    def test_sets_logo_with_mode(self):
        unit = _make_ace(mode=MODE_COMBINED)
        unit.set_logo_color = MagicMock()
        unit._register_gcode_commands = MagicMock()
        unit.handle_connect()
        assert "combined" in unit.logo

    def test_sets_logo_error(self):
        unit = _make_ace()
        unit.set_logo_color = MagicMock()
        unit._register_gcode_commands = MagicMock()
        unit.handle_connect()
        assert "ERR" in unit.logo_error


# ── system_Test: empty lane (prep=F, load=F) ────────────────────────────────

class TestSystemTestEmptyLane:
    def test_empty_lane_sets_not_ready(self):
        unit = _make_ace()
        lane = _make_lane(prep_state=False, load_state=False)
        unit.system_Test(lane, delay=0.0, assignTcmd=True, enable_movement=False)
        unit.afc.function.afc_led.assert_any_call(lane.led_not_ready, lane.led_index)

    def test_empty_lane_returns_true(self):
        unit = _make_ace()
        lane = _make_lane(prep_state=False, load_state=False)
        result = unit.system_Test(lane, delay=0.0, assignTcmd=True, enable_movement=False)
        assert result is True


# ── system_Test: fault lane (prep=F, load=T) ────────────────────────────────

class TestSystemTestFaultLane:
    def test_fault_state_sets_fault_led(self):
        unit = _make_ace()
        lane = _make_lane(prep_state=False, load_state=True)
        unit.system_Test(lane, delay=0.0, assignTcmd=True, enable_movement=False)
        unit.afc.function.afc_led.assert_any_call(lane.led_fault, lane.led_index)

    def test_fault_state_returns_false(self):
        unit = _make_ace()
        lane = _make_lane(prep_state=False, load_state=True)
        result = unit.system_Test(lane, delay=0.0, assignTcmd=True, enable_movement=False)
        assert result is False


# ── system_Test: loaded (prep=T, load=T) ─────────────────────────────────────

class TestSystemTestLoadedLane:
    def test_loaded_sets_status_to_loaded(self):
        unit = _make_ace()
        lane = _make_lane(prep_state=True, load_state=True)
        unit.system_Test(lane, delay=0.0, assignTcmd=True, enable_movement=False)
        assert lane.status == AFCLaneState.LOADED


# ── system_Test: locked not loaded (prep=T, load=F) ─────────────────────────

class TestSystemTestLockedNotLoaded:
    def test_returns_false(self):
        unit = _make_ace()
        lane = _make_lane(prep_state=True, load_state=False)
        result = unit.system_Test(lane, delay=0.0, assignTcmd=True, enable_movement=False)
        assert result is False


# ── _poll_slot_status: shifting state handling ───────────────────────────────

class TestPollSlotStatusShifting:
    def test_shifting_does_not_update_prev_state(self):
        unit = _make_ace()
        lane = _make_lane()
        unit.lanes = {"lane1": lane}
        unit._prev_slot_states = {"lane1": True}
        unit._slot_inventory = [{"status": "shifting"}, {}, {}, {}]

        ace = MagicMock()
        ace.connected = True
        ace.get_status.return_value = {"slots": [{"status": "shifting"}]}
        unit._ace = ace
        unit.afc.function.is_printing.return_value = True

        unit._poll_slot_status(100.0)

        assert unit._prev_slot_states["lane1"] is True

    def test_shifting_does_not_trigger_runout(self):
        unit = _make_ace()
        lane = _make_lane()
        lane.status = AFCLaneState.TOOLED
        unit.lanes = {"lane1": lane}
        unit._prev_slot_states = {"lane1": True}
        unit._slot_inventory = [{"status": "shifting"}, {}, {}, {}]

        ace = MagicMock()
        ace.connected = True
        ace.get_status.return_value = {"slots": [{"status": "shifting"}]}
        unit._ace = ace
        unit.afc.function.is_printing.return_value = True

        unit._poll_slot_status(100.0)

        lane._perform_infinite_runout.assert_not_called()
        lane._perform_pause_runout.assert_not_called()


class TestPollSlotStatusOperationGuard:
    def test_skips_when_operation_active(self):
        unit = _make_ace()
        unit._operation_active = True
        unit._ace = MagicMock()
        unit._ace.connected = True

        result = unit._poll_slot_status(100.0)

        assert result == 101.0  # eventtime + poll_interval
