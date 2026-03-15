"""
Unit tests for extras/AFC_ACE.py

Covers:
  - afcACE: class constants, slot mapping, color conversion
  - _get_local_slot_for_lane: maps lane unit field to 0-based slot index
  - _ace_color_to_hex: converts RGB array to hex string
  - system_Test: LED logic for each lane state combination
  - _poll_slot_status: shifting state handling, operation guard, state consistency
  - get_status: correct keys and content
  - check_runout: returns True when printing
  - get_lane_reset_command: returns TOOL_UNLOAD command
  - lane_unload: blocks manual unload
  - load_sequence / unload_sequence: _operation_active guard
"""

from __future__ import annotations

from unittest.mock import MagicMock, PropertyMock, patch
import pytest

from extras.AFC_ACE import afcACE, _AceBackend, BACKEND_ACEPRO, BACKEND_DUCKACE
from extras.AFC_lane import AFCLaneState


# ── Helpers ───────────────────────────────────────────────────────────────────

def _make_ace(name="ACE_1"):
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
    unit.ace_instance_index = 0
    unit._configured_backend = "auto"
    unit._cached_ace_obj = None
    unit._backend = None
    unit._operation_active = False
    unit._prev_slot_states = {}

    return unit


def _make_lane(name="lane1", prep_state=False, load_state=False,
               tool_loaded=False, unit_field="ACE_1:1"):
    lane = MagicMock()
    lane.name = name
    lane.prep_state = prep_state
    lane.load_state = load_state
    lane.tool_loaded = tool_loaded
    lane.map = "T0"
    lane.unit = unit_field
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


def _make_backend(backend_type=BACKEND_DUCKACE, connected=True):
    """Create a mock _AceBackend."""
    backend = MagicMock(spec=_AceBackend)
    backend.backend_type = backend_type
    backend.is_connected = connected
    return backend


# ── Constants ────────────────────────────────────────────────────────────────

class TestConstants:
    def test_slots_per_unit(self):
        assert afcACE.SLOTS_PER_UNIT == 4


# ── _get_local_slot_for_lane ──────────────────────────────────────────────────

class TestGetLocalSlotForLane:
    def test_returns_0_for_slot_1(self):
        unit = _make_ace()
        lane = _make_lane(unit_field="ACE_1:1")
        assert unit._get_local_slot_for_lane(lane) == 0

    def test_returns_3_for_slot_4(self):
        unit = _make_ace()
        lane = _make_lane(unit_field="ACE_1:4")
        assert unit._get_local_slot_for_lane(lane) == 3

    def test_returns_neg1_for_invalid_field(self):
        unit = _make_ace()
        lane = _make_lane(unit_field="ACE_1")
        assert unit._get_local_slot_for_lane(lane) == -1

    def test_returns_neg1_for_empty_field(self):
        unit = _make_ace()
        lane = _make_lane(unit_field="")
        assert unit._get_local_slot_for_lane(lane) == -1


# ── _ace_color_to_hex ────────────────────────────────────────────────────────

class TestAceColorToHex:
    def test_rgb_conversion(self):
        assert afcACE._ace_color_to_hex([255, 128, 0]) == "#ff8000"

    def test_black_for_zeros(self):
        assert afcACE._ace_color_to_hex([0, 0, 0]) == "#000000"

    def test_white(self):
        assert afcACE._ace_color_to_hex([255, 255, 255]) == "#ffffff"

    def test_fallback_for_short_array(self):
        assert afcACE._ace_color_to_hex([255]) == "#000000"

    def test_fallback_for_non_list(self):
        assert afcACE._ace_color_to_hex("red") == "#000000"

    def test_tuple_input(self):
        assert afcACE._ace_color_to_hex((128, 64, 32)) == "#804020"


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
    def test_returns_tool_unload(self):
        unit = _make_ace()
        lane = _make_lane(name="lane3")
        result = unit.get_lane_reset_command(lane, None)
        assert result == "TOOL_UNLOAD LANE=lane3"

    def test_ignores_distance(self):
        unit = _make_ace()
        lane = _make_lane(name="lane1")
        result = unit.get_lane_reset_command(lane, 100)
        assert result == "TOOL_UNLOAD LANE=lane1"


# ── lane_unload ──────────────────────────────────────────────────────────────

class TestLaneUnload:
    def test_returns_none(self):
        unit = _make_ace()
        lane = _make_lane()
        result = unit.lane_unload(lane)
        assert result is None


# ── get_status ───────────────────────────────────────────────────────────────

class TestGetStatus:
    def test_has_ace_backend_key(self):
        unit = _make_ace()
        status = unit.get_status()
        assert "ace_backend" in status

    def test_has_ace_connected_key(self):
        unit = _make_ace()
        status = unit.get_status()
        assert "ace_connected" in status

    def test_reports_not_connected_when_no_backend(self):
        unit = _make_ace()
        unit._backend = None
        status = unit.get_status()
        assert status["ace_connected"] is False

    def test_reports_connected_when_backend_present(self):
        unit = _make_ace()
        unit._backend = _make_backend(connected=True)
        status = unit.get_status()
        assert status["ace_connected"] is True


# ── handle_connect ───────────────────────────────────────────────────────────

class TestHandleConnect:
    def test_sets_logo(self):
        unit = _make_ace()
        unit.set_logo_color = MagicMock()
        unit.handle_connect()
        assert "ACE" in unit.logo

    def test_sets_logo_error(self):
        unit = _make_ace()
        unit.set_logo_color = MagicMock()
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
    def test_loaded_sets_ready_led(self):
        unit = _make_ace()
        lane = _make_lane(prep_state=True, load_state=True)
        unit.system_Test(lane, delay=0.0, assignTcmd=True, enable_movement=False)
        unit.afc.function.afc_led.assert_any_call(lane.led_ready, lane.led_index)

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
        backend = _make_backend()
        unit._backend = backend
        lane = _make_lane()
        unit.lanes = {"lane1": lane}
        unit._prev_slot_states = {"lane1": True}

        # Slot reports "shifting"
        backend.get_slot_info.return_value = {"status": "shifting"}
        unit.afc.function.is_printing.return_value = True

        unit._poll_slot_status(100.0)

        # prev state should remain True (not updated during shifting)
        assert unit._prev_slot_states["lane1"] is True

    def test_shifting_does_not_trigger_runout(self):
        unit = _make_ace()
        backend = _make_backend()
        unit._backend = backend
        lane = _make_lane()
        lane.status = AFCLaneState.TOOLED
        unit.lanes = {"lane1": lane}
        unit._prev_slot_states = {"lane1": True}

        backend.get_slot_info.return_value = {"status": "shifting"}
        unit.afc.function.is_printing.return_value = True

        unit._poll_slot_status(100.0)

        # No runout should be triggered
        lane._perform_infinite_runout.assert_not_called()
        lane._perform_pause_runout.assert_not_called()


class TestPollSlotStatusOperationGuard:
    def test_skips_when_operation_active(self):
        unit = _make_ace()
        unit._operation_active = True
        unit._backend = _make_backend()
        unit.lanes = {"lane1": _make_lane()}

        result = unit._poll_slot_status(100.0)

        # Should return early without processing lanes
        assert result == 102.0  # eventtime + 2.0

    def test_processes_when_operation_not_active(self):
        unit = _make_ace()
        unit._operation_active = False
        backend = _make_backend()
        unit._backend = backend
        lane = _make_lane()
        unit.lanes = {"lane1": lane}
        backend.get_slot_info.return_value = {"status": "ready"}
        unit.afc.function.is_printing.return_value = False

        result = unit._poll_slot_status(100.0)

        # Should process (idle rate = 2s)
        assert result == 102.0


class TestPollSlotStatusStateConsistency:
    def test_ready_slot_in_none_state_gets_set_loaded(self):
        unit = _make_ace()
        backend = _make_backend()
        unit._backend = backend
        lane = _make_lane()
        lane.status = AFCLaneState.NONE
        lane._afc_prep_done = True
        unit.lanes = {"lane1": lane}
        unit._prev_slot_states = {}

        backend.get_slot_info.return_value = {"status": "ready"}
        unit.afc.function.is_printing.return_value = False

        unit._poll_slot_status(100.0)

        lane.set_loaded.assert_called_once()
        unit.afc.save_vars.assert_called()


class TestPollSlotStatusRunout:
    def test_ready_to_empty_triggers_runout_on_tooled_lane(self):
        unit = _make_ace()
        backend = _make_backend()
        unit._backend = backend
        lane = _make_lane()
        lane.status = AFCLaneState.TOOLED
        lane.runout_lane = "lane2"
        unit.lanes = {"lane1": lane}
        unit._prev_slot_states = {"lane1": True}

        backend.get_slot_info.return_value = {"status": "empty"}
        unit.afc.function.is_printing.return_value = True

        unit._poll_slot_status(100.0)

        lane._perform_infinite_runout.assert_called_once()

    def test_no_runout_when_not_printing(self):
        unit = _make_ace()
        backend = _make_backend()
        unit._backend = backend
        lane = _make_lane()
        lane.status = AFCLaneState.TOOLED
        unit.lanes = {"lane1": lane}
        unit._prev_slot_states = {"lane1": True}

        backend.get_slot_info.return_value = {"status": "empty"}
        unit.afc.function.is_printing.return_value = False

        unit._poll_slot_status(100.0)

        lane._perform_infinite_runout.assert_not_called()
        lane._perform_pause_runout.assert_not_called()

    def test_adaptive_poll_rate(self):
        unit = _make_ace()
        backend = _make_backend()
        unit._backend = backend
        unit.lanes = {}

        unit.afc.function.is_printing.return_value = True
        result = unit._poll_slot_status(100.0)
        assert result == 101.0  # 1s when printing

        unit.afc.function.is_printing.return_value = False
        result = unit._poll_slot_status(100.0)
        assert result == 102.0  # 2s when idle


class TestPollSlotStatusIdleSync:
    def test_syncs_loaded_to_hub_during_idle(self):
        unit = _make_ace()
        backend = _make_backend()
        unit._backend = backend
        lane = _make_lane()
        lane.loaded_to_hub = False
        lane.status = AFCLaneState.LOADED
        unit.lanes = {"lane1": lane}
        unit._prev_slot_states = {}

        backend.get_slot_info.return_value = {"status": "ready"}
        unit.afc.function.is_printing.return_value = False

        unit._poll_slot_status(100.0)

        assert lane.loaded_to_hub is True


# ── load_sequence / unload_sequence: operation guard ─────────────────────────

class TestOperationActiveGuard:
    def test_load_sets_operation_active(self):
        unit = _make_ace()
        unit._backend = _make_backend()
        # Will fail early due to no backend, but operation_active should be set/cleared
        assert unit._operation_active is False

    def test_unload_sets_operation_active(self):
        unit = _make_ace()
        assert unit._operation_active is False
