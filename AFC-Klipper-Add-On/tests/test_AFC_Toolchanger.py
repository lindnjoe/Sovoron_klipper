"""Tests for AFC_Toolchanger – native toolchanger engine."""

import pytest
from unittest.mock import MagicMock

from extras.AFC_Toolchanger import (
    AfcToolchanger,
    ToolGcodeTransform,
    FanSwitcher,
    STATUS_UNINITIALIZED,
    STATUS_INITIALIZING,
    STATUS_READY,
    STATUS_CHANGING,
    STATUS_ERROR,
    DETECT_ABSENT,
    DETECT_PRESENT,
    DETECT_UNAVAILABLE,
    XYZ_TO_INDEX,
)
from extras.AFC import State
from extras.AFC_extruder import AFCExtruder
from tests.conftest import MockAFC, MockPrinter, MockLogger, MockReactor, MockGcode


# ── Helpers ──────────────────────────────────────────────────────────────────

def _make_toolchanger():
    """Build an AfcToolchanger bypassing __init__."""
    tc = AfcToolchanger.__new__(AfcToolchanger)

    afc = MockAFC()
    reactor = MockReactor()
    printer = MockPrinter(afc=afc)
    gcode = MockGcode()

    tc.printer = printer
    tc.afc = afc
    tc.logger = MockLogger()
    tc.reactor = reactor
    tc.gcode = gcode
    tc.config = MagicMock()
    tc.config.get_name.return_value = "AFC_Toolchanger Tools"
    tc.type = "Toolchanger"
    tc.functions = MagicMock()
    tc.gcode_move = MagicMock()
    tc.gcode_macro = MagicMock()

    tc.uses_axis = "xyz"
    tc.require_tool_present = False
    tc.verify_tool_pickup = True
    tc.transfer_fan_speed = True

    tc.default_before_change_gcode = MagicMock()
    tc.default_after_change_gcode = MagicMock()
    tc.initialize_gcode = MagicMock()
    tc.error_gcode = None

    tc.status = STATUS_UNINITIALIZED
    tc.active_tool = None
    tc.detected_tool = None
    tc.has_detection = False
    tc.tools = {}
    tc.tool_numbers = []
    tc.tool_names = []
    tc.error_message = ""

    tc.last_change_gcode_position = None
    tc.last_change_gcode_offset = None
    tc.last_change_restore_axis = None
    tc.last_change_pickup_tool = None

    tc.gcode_transform = ToolGcodeTransform()
    tc.tool_probe_endstop = None
    tc.fan_switcher = None

    return tc


def _make_tool(name="extruder", tool_number=0, detect_state=DETECT_UNAVAILABLE):
    """Build a mock AFCExtruder tool."""
    tool = MagicMock(spec=AFCExtruder)
    tool.name = name
    tool.tool_number = tool_number
    tool.detect_state = detect_state
    tool.detect_pin_name = None if detect_state == DETECT_UNAVAILABLE else "detect_pin"
    tool.t_command_restore_axis = "XYZ"
    tool.fan = None
    tool.tool_probe = None
    tool.resonance_chip = None
    tool.get_offset.return_value = [0.0, 0.0, 0.0]
    tool.before_change_gcode = MagicMock()
    tool.after_change_gcode = MagicMock()
    tool.dropoff_gcode = MagicMock()
    tool.pickup_gcode = MagicMock()
    return tool


# ── ToolGcodeTransform ───────────────────────────────────────────────────────

class TestToolGcodeTransform:
    def test_no_tool_passthrough(self):
        t = ToolGcodeTransform()
        t.next_transform = MagicMock()
        t.next_transform.get_position.return_value = [10.0, 20.0, 30.0, 0.0]
        t.tool = None
        pos = t.get_position()
        assert pos == [10.0, 20.0, 30.0, 0.0]

    def test_with_tool_offset_get_position(self):
        t = ToolGcodeTransform()
        t.next_transform = MagicMock()
        t.next_transform.get_position.return_value = [10.0, 20.0, 30.0, 0.0]
        tool = MagicMock()
        tool.get_offset.return_value = [1.0, 2.0, 3.0]
        t.tool = tool
        pos = t.get_position()
        assert pos == [9.0, 18.0, 27.0, 0.0]

    def test_with_tool_offset_move(self):
        t = ToolGcodeTransform()
        t.next_transform = MagicMock()
        tool = MagicMock()
        tool.get_offset.return_value = [1.0, 2.0, 0.5]
        t.tool = tool
        t.move([10.0, 20.0, 30.0, 0.0], 100.0)
        t.next_transform.move.assert_called_once_with(
            [11.0, 22.0, 30.5, 0.0], 100.0)

    def test_no_tool_move_passthrough(self):
        t = ToolGcodeTransform()
        t.next_transform = MagicMock()
        t.tool = None
        t.move([5.0, 10.0, 15.0, 0.0], 50.0)
        t.next_transform.move.assert_called_once_with(
            [5.0, 10.0, 15.0, 0.0], 50.0)


# ── Tool registration ───────────────────────────────────────────────────────

class TestToolRegistration:
    def test_register_tool(self):
        tc = _make_toolchanger()
        tool = _make_tool("extruder", 0)
        tc.register_tool(tool, 0)
        assert tc.tools[0] is tool
        assert 0 in tc.tool_numbers
        assert "extruder" in tc.tool_names

    def test_register_multiple_tools_sorted(self):
        tc = _make_toolchanger()
        t2 = _make_tool("extruder2", 2)
        t0 = _make_tool("extruder", 0)
        t1 = _make_tool("extruder1", 1)
        tc.register_tool(t2, 2)
        tc.register_tool(t0, 0)
        tc.register_tool(t1, 1)
        assert tc.tool_numbers == [0, 1, 2]
        assert tc.tool_names == ["extruder", "extruder1", "extruder2"]

    def test_lookup_tool(self):
        tc = _make_toolchanger()
        tool = _make_tool("extruder", 0)
        tc.register_tool(tool, 0)
        assert tc.lookup_tool(0) is tool
        assert tc.lookup_tool(99) is None

    def test_register_tool_updates_detection(self):
        tc = _make_toolchanger()
        tool = _make_tool("extruder", 0, detect_state=DETECT_PRESENT)
        tc.register_tool(tool, 0)
        assert tc.has_detection is True

    def test_register_tool_no_detection(self):
        tc = _make_toolchanger()
        tool = _make_tool("extruder", 0, detect_state=DETECT_UNAVAILABLE)
        tc.register_tool(tool, 0)
        assert tc.has_detection is False


# ── get_status ───────────────────────────────────────────────────────────────

class TestGetStatus:
    def test_no_active_tool(self):
        tc = _make_toolchanger()
        status = tc.get_status()
        assert status["tool"] is None
        assert status["tool_number"] == -1
        assert status["status"] == STATUS_UNINITIALIZED

    def test_with_active_tool(self):
        tc = _make_toolchanger()
        tool = _make_tool("extruder", 0)
        tool.get_offset.return_value = [1.5, -2.0, 0.3]
        tc.active_tool = tool
        tc.status = STATUS_READY
        status = tc.get_status()
        assert status["tool"] == "extruder"
        assert status["tool_number"] == 0
        assert status["status"] == STATUS_READY
        assert status["active_tool_gcode_x_offset"] == 1.5
        assert status["active_tool_gcode_y_offset"] == -2.0
        assert status["active_tool_gcode_z_offset"] == 0.3


# ── Detection ────────────────────────────────────────────────────────────────

class TestDetection:
    def test_note_detect_change_single(self):
        tc = _make_toolchanger()
        t0 = _make_tool("extruder", 0, detect_state=DETECT_PRESENT)
        t1 = _make_tool("extruder1", 1, detect_state=DETECT_ABSENT)
        tc.tools = {0: t0, 1: t1}
        tc.note_detect_change(t0, 100.0)
        assert tc.detected_tool is t0

    def test_note_detect_change_none(self):
        tc = _make_toolchanger()
        t0 = _make_tool("extruder", 0, detect_state=DETECT_ABSENT)
        t1 = _make_tool("extruder1", 1, detect_state=DETECT_ABSENT)
        tc.tools = {0: t0, 1: t1}
        tc.note_detect_change(t0, 100.0)
        assert tc.detected_tool is None

    def test_note_detect_change_multiple_resets(self):
        tc = _make_toolchanger()
        t0 = _make_tool("extruder", 0, detect_state=DETECT_PRESENT)
        t1 = _make_tool("extruder1", 1, detect_state=DETECT_PRESENT)
        tc.tools = {0: t0, 1: t1}
        tc.note_detect_change(t0, 100.0)
        assert tc.detected_tool is None


# ── Position helpers ─────────────────────────────────────────────────────────

class TestPositionHelpers:
    def test_position_to_xyz(self):
        tc = _make_toolchanger()
        pos = [10.0, 20.0, 30.0, 0.0]
        result = tc._position_to_xyz(pos, "xyz")
        assert result == {"X": 10.0, "Y": 20.0, "Z": 30.0}

    def test_position_to_xyz_partial(self):
        tc = _make_toolchanger()
        pos = [10.0, 20.0, 30.0, 0.0]
        result = tc._position_to_xyz(pos, "xz")
        assert result == {"X": 10.0, "Z": 30.0}
        assert "Y" not in result

    def test_position_with_tool_offset_no_offset(self):
        tc = _make_toolchanger()
        tc.last_change_gcode_offset = None
        pos = [10.0, 20.0, 30.0, 0.0]
        result = tc._position_with_tool_offset(pos, None)
        assert result[:3] == [10.0, 20.0, 30.0]

    def test_position_with_tool_offset(self):
        tc = _make_toolchanger()
        tc.last_change_gcode_offset = [1.0, 2.0, 3.0, 0.0]
        tool = _make_tool()
        tool.get_offset.return_value = [0.5, 0.5, 0.5]
        pos = [10.0, 20.0, 30.0, 0.0]
        result = tc._position_with_tool_offset(pos, tool)
        assert result[:3] == [11.5, 22.5, 33.5]


# ── Homing check ─────────────────────────────────────────────────────────────

class TestEnsureHomed:
    def test_no_axis_required(self):
        tc = _make_toolchanger()
        tc.uses_axis = ""
        gcmd = MagicMock()
        tc._ensure_homed(gcmd)  # Should not raise

    def test_all_homed(self):
        tc = _make_toolchanger()
        tc.uses_axis = "xyz"
        toolhead = MagicMock()
        toolhead.get_kinematics.return_value.get_status.return_value = {
            "homed_axes": "xyz"
        }
        tc.printer = MagicMock()
        tc.printer.lookup_object.return_value = toolhead
        tc.printer.get_reactor.return_value = MockReactor()
        gcmd = MagicMock()
        tc._ensure_homed(gcmd)  # Should not raise


# ── FanSwitcher ──────────────────────────────────────────────────────────────

class TestFanSwitcher:
    def test_activate_fan_none(self):
        tc = _make_toolchanger()
        fs = FanSwitcher(tc)
        fs.activate_fan(None)
        assert fs.active_fan is None

    def test_activate_fan_sets_active(self):
        tc = _make_toolchanger()
        fs = FanSwitcher(tc)
        fan = MagicMock()
        fan.fan_name = "T0_partfan"
        fs.activate_fan(fan)
        assert fs.active_fan is fan


# ── Constants ────────────────────────────────────────────────────────────────

class TestConstants:
    def test_xyz_to_index(self):
        assert XYZ_TO_INDEX["x"] == 0
        assert XYZ_TO_INDEX["Y"] == 1
        assert XYZ_TO_INDEX["z"] == 2

    def test_status_constants(self):
        assert STATUS_UNINITIALIZED == "uninitialized"
        assert STATUS_READY == "ready"
        assert STATUS_CHANGING == "changing"
        assert STATUS_ERROR == "error"

    def test_detect_constants(self):
        assert DETECT_UNAVAILABLE == -1
        assert DETECT_ABSENT == 0
        assert DETECT_PRESENT == 1


# ── Initialize ───────────────────────────────────────────────────────────────

class TestInitialize:
    def test_initialize_no_detection_no_tool(self):
        tc = _make_toolchanger()
        tc.has_detection = False
        tc.require_tool_present = False
        tc.initialize()
        assert tc.status == STATUS_READY

    def test_initialize_with_tool(self):
        tc = _make_toolchanger()
        tc.has_detection = False
        tc.require_tool_present = False
        tool = _make_tool()
        tc.initialize(select_tool=tool)
        assert tc.status == STATUS_READY
        assert tc.active_tool is tool

    def test_cannot_initialize_while_changing(self):
        tc = _make_toolchanger()
        tc.status = STATUS_CHANGING
        try:
            tc.initialize()
            assert False, "Should have raised"
        except Exception as e:
            assert "Cannot initialize" in str(e)


# ── Configure toolhead for tool ──────────────────────────────────────────────

class TestConfigureToolhead:
    def test_activates_new_tool(self):
        tc = _make_toolchanger()
        tool = _make_tool()
        tc._configure_toolhead_for_tool(tool)
        assert tc.active_tool is tool
        tool.activate_tool.assert_called_once()

    def test_deactivates_old_tool(self):
        tc = _make_toolchanger()
        old_tool = _make_tool("old", 0)
        new_tool = _make_tool("new", 1)
        tc.active_tool = old_tool
        tc._configure_toolhead_for_tool(new_tool)
        old_tool.deactivate_tool.assert_called_once()
        assert tc.active_tool is new_tool

    def test_sets_none_tool(self):
        tc = _make_toolchanger()
        old_tool = _make_tool("old", 0)
        tc.active_tool = old_tool
        tc._configure_toolhead_for_tool(None)
        old_tool.deactivate_tool.assert_called_once()
        assert tc.active_tool is None

    def test_sets_active_probe(self):
        tc = _make_toolchanger()
        tc.tool_probe_endstop = MagicMock()
        tool = _make_tool()
        tool.tool_probe = MagicMock()
        tc._configure_toolhead_for_tool(tool)
        tc.tool_probe_endstop.set_active_probe.assert_called_once_with(
            tool.tool_probe)

    def test_no_tool_probe_attribute_uses_getattr(self):
        """Extruder without tool_probe attribute should not raise AttributeError."""
        tc = _make_toolchanger()
        tc.tool_probe_endstop = MagicMock()
        tool = _make_tool()
        # Remove tool_probe attribute to simulate extruder without probe config
        del tool.tool_probe
        tc._configure_toolhead_for_tool(tool)
        tc.tool_probe_endstop.set_active_probe.assert_called_once_with(None)


# ── select_tool state machine ────────────────────────────────────────────────

def _make_ready_toolchanger():
    """Build a toolchanger in READY state with mocked internals for select_tool."""
    tc = _make_toolchanger()
    tc.status = STATUS_READY
    # Mock _ensure_homed to not raise
    tc._ensure_homed = MagicMock()
    # Mock _save_state to record gcode position
    def fake_save_state(restore_axis, tool):
        tc.last_change_gcode_position = [100.0, 200.0, 10.0, 0.0]
        tc.last_change_gcode_offset = [0.0, 0.0, 0.0, 0.0]
        tc.last_change_restore_axis = restore_axis
        tc.last_change_pickup_tool = tool
    tc._save_state = MagicMock(side_effect=fake_save_state)
    tc._set_toolchange_transform = MagicMock()
    tc._run_gcode = MagicMock()
    tc._restore_state_and_transform = MagicMock()
    return tc


class TestSelectTool:
    def test_select_tool_happy_path(self):
        tc = _make_ready_toolchanger()
        tool = _make_tool("extruder1", 1)
        gcmd = MagicMock()
        tc.select_tool(gcmd, tool, "XYZ")
        assert tc.status == STATUS_READY
        assert tc.active_tool is tool

    def test_select_same_tool_is_noop(self):
        tc = _make_ready_toolchanger()
        tool = _make_tool("extruder0", 0)
        tc.active_tool = tool
        gcmd = MagicMock()
        tc.select_tool(gcmd, tool, "XYZ")
        # _save_state should not have been called
        tc._save_state.assert_not_called()
        assert tc.status == STATUS_READY

    def test_select_none_unselects(self):
        tc = _make_ready_toolchanger()
        old_tool = _make_tool("extruder0", 0)
        tc.active_tool = old_tool
        gcmd = MagicMock()
        tc.select_tool(gcmd, None, "XYZ")
        assert tc.status == STATUS_READY

    def test_cannot_select_when_not_ready(self):
        tc = _make_toolchanger()
        tc.status = STATUS_ERROR
        tc.error_message = "previous error"
        # Bypass auto-initialize
        tc.has_detection = False
        tc.require_tool_present = False
        tool = _make_tool()
        gcmd = MagicMock()
        gcmd.error = type("GcmdError", (Exception,), {})
        with pytest.raises(gcmd.error):
            tc.select_tool(gcmd, tool, "XYZ")

    def test_gcmd_error_during_change_sets_error_status(self):
        """If _run_gcode raises gcmd.error during change, status should be ERROR."""
        tc = _make_ready_toolchanger()
        tool = _make_tool("extruder1", 1)
        gcmd = MagicMock()
        gcmd.error = type("GcmdError", (Exception,), {})
        # Make _run_gcode raise after status is set to CHANGING
        tc._run_gcode = MagicMock(side_effect=gcmd.error("gcode failed"))
        with pytest.raises(gcmd.error):
            tc.select_tool(gcmd, tool, "XYZ")
        assert tc.status == STATUS_ERROR

    def test_unexpected_exception_sets_error_not_changing(self):
        """Non-gcode exception must not leave status stuck at CHANGING."""
        tc = _make_ready_toolchanger()
        tool = _make_tool("extruder1", 1)
        gcmd = MagicMock()
        gcmd.error = type("GcmdError", (Exception,), {})
        # Simulate unexpected AttributeError during tool change
        tc._run_gcode = MagicMock(side_effect=AttributeError("bad attr"))
        with pytest.raises(AttributeError):
            tc.select_tool(gcmd, tool, "XYZ")
        # Must NOT be stuck at CHANGING
        assert tc.status != STATUS_CHANGING
        assert tc.status == STATUS_ERROR

    def test_runtime_error_sets_error_not_changing(self):
        """RuntimeError during tool change must not leave status stuck."""
        tc = _make_ready_toolchanger()
        tool = _make_tool("extruder1", 1)
        gcmd = MagicMock()
        gcmd.error = type("GcmdError", (Exception,), {})
        tc._run_gcode = MagicMock(side_effect=RuntimeError("move failed"))
        with pytest.raises(RuntimeError):
            tc.select_tool(gcmd, tool, "XYZ")
        assert tc.status == STATUS_ERROR


# ── tool_swap try/finally ────────────────────────────────────────────────────

def _make_lane_for_swap(extruder_name="extruder0", tool_number=0):
    """Build a mock lane suitable for tool_swap."""
    lane = MagicMock()
    lane.extruder_obj = MagicMock()
    lane.extruder_obj.name = extruder_name
    lane.extruder_obj.tool_number = tool_number
    lane.extruder_obj.custom_tool_swap = None
    lane.extruder_obj.estats = MagicMock()
    return lane


class TestToolSwap:
    def _make_swap_ready_tc(self):
        tc = _make_toolchanger()
        tc.status = STATUS_READY
        tc.afc.current_state = State.IDLE
        tc.afc.afcDeltaTime = MagicMock()
        tc.afc.afcDeltaTime.delta_time = 1.0
        tc.afc.afc_stats = MagicMock()
        tc.afc.afc_stats.average_tool_swap_time = None
        tc.afc.toolhead = MagicMock()
        tc.afc.gcode_move = MagicMock()
        tc.afc.gcode_move.base_position = [0, 0, 0, 0]
        tc.afc.gcode_move.homing_position = [0, 0, 0, 0]
        tc.afc.gcode_move.last_position = [0, 0, 0, 0]
        tc.afc._oams_suppress_tool_swap_timer = False
        return tc

    def test_tool_swap_happy_path(self):
        tc = self._make_swap_ready_tc()
        lane = _make_lane_for_swap()
        tc.tool_swap(lane)
        assert tc.afc.current_state == State.IDLE
        tc.afc.gcode.run_script_from_command.assert_called()

    def test_tool_swap_resets_state_on_error(self):
        """State must be IDLE even if SELECT_TOOL throws."""
        tc = self._make_swap_ready_tc()
        lane = _make_lane_for_swap()
        tc.afc.gcode.run_script_from_command = MagicMock(
            side_effect=Exception("SELECT_TOOL failed"))
        with pytest.raises(Exception, match="SELECT_TOOL failed"):
            tc.tool_swap(lane)
        # State must NOT be stuck at TOOL_SWAP
        assert tc.afc.current_state == State.IDLE

    def test_tool_swap_resets_state_on_activate_error(self):
        """State must be IDLE even if _handle_activate_extruder throws."""
        tc = self._make_swap_ready_tc()
        lane = _make_lane_for_swap()
        tc.afc.function._handle_activate_extruder = MagicMock(
            side_effect=RuntimeError("stepper sync failed"))
        with pytest.raises(RuntimeError):
            tc.tool_swap(lane)
        assert tc.afc.current_state == State.IDLE

    def test_tool_swap_custom_tool_swap(self):
        tc = self._make_swap_ready_tc()
        lane = _make_lane_for_swap()
        lane.extruder_obj.custom_tool_swap = "MY_CUSTOM_SWAP"
        tc.tool_swap(lane)
        tc.afc.gcode.run_script_from_command.assert_any_call("MY_CUSTOM_SWAP")

    def test_tool_swap_derives_tool_index_from_name(self):
        """When tool_number is -1, derive index from extruder name."""
        tc = self._make_swap_ready_tc()
        lane = _make_lane_for_swap("extruder3", tool_number=-1)
        tc.tool_swap(lane)
        tc.afc.gcode.run_script_from_command.assert_any_call(
            'SELECT_TOOL T=3')

    def test_tool_swap_base_extruder_index_zero(self):
        """Base 'extruder' (no number) should resolve to T=0."""
        tc = self._make_swap_ready_tc()
        lane = _make_lane_for_swap("extruder", tool_number=-1)
        tc.tool_swap(lane)
        tc.afc.gcode.run_script_from_command.assert_any_call(
            'SELECT_TOOL T=0')


# ── process_error ────────────────────────────────────────────────────────────

class TestProcessError:
    def test_sets_error_status_and_message(self):
        tc = _make_toolchanger()
        tc.status = STATUS_CHANGING
        tc.error_gcode = None
        with pytest.raises(Exception, match="pickup failed"):
            tc.process_error(Exception, "pickup failed")
        assert tc.status == STATUS_ERROR
        assert tc.error_message == "pickup failed"

    def test_no_raise_when_raise_error_is_none(self):
        tc = _make_toolchanger()
        tc.status = STATUS_CHANGING
        tc.error_gcode = None
        # Should not raise
        tc.process_error(None, "just a warning")
        assert tc.status == STATUS_ERROR

    def test_error_gcode_runs_with_position_context(self):
        tc = _make_toolchanger()
        tc.status = STATUS_CHANGING
        tc.last_change_gcode_position = [100.0, 200.0, 10.0, 0.0]
        tc.last_change_gcode_offset = [0.0, 0.0, 0.0, 0.0]
        tc.last_change_restore_axis = "XYZ"
        tc.last_change_pickup_tool = _make_tool("extruder1", 1)
        tc.error_gcode = MagicMock()
        tc._run_gcode = MagicMock()
        with pytest.raises(Exception):
            tc.process_error(Exception, "detect failed")
        tc._run_gcode.assert_called_once()
        call_args = tc._run_gcode.call_args
        assert call_args[0][0] == 'error_gcode'
        assert 'start_position' in call_args[0][2]

    def test_error_gcode_failure_preserves_original_error(self):
        """If error_gcode itself throws, the original error must still be raised."""
        tc = _make_toolchanger()
        tc.status = STATUS_CHANGING
        tc.last_change_gcode_position = None
        tc.error_gcode = MagicMock()
        tc._run_gcode = MagicMock(side_effect=RuntimeError("error_gcode crashed"))
        with pytest.raises(Exception, match="tool broke"):
            tc.process_error(Exception, "tool broke")
        # The original error is raised, not the error_gcode error
        assert tc.status == STATUS_ERROR
        assert tc.error_message == "tool broke"

    def test_error_gcode_failure_is_logged(self):
        """error_gcode failure should be logged."""
        tc = _make_toolchanger()
        tc.status = STATUS_CHANGING
        tc.last_change_gcode_position = None
        tc.error_gcode = MagicMock()
        tc._run_gcode = MagicMock(side_effect=RuntimeError("boom"))
        with pytest.raises(Exception):
            tc.process_error(Exception, "original")
        # Check that the error was logged
        errors = [m for level, m in tc.logger.messages if level == "error"]
        assert any("original" in e and "boom" in e for e in errors)


# ── initialize error paths ───────────────────────────────────────────────────

class TestInitializeErrors:
    def test_gcode_error_sets_error_status(self):
        """If initialize_gcode raises, status must not stay at INITIALIZING."""
        tc = _make_toolchanger()
        tc.has_detection = False
        tc.require_tool_present = False
        tc._run_gcode = MagicMock(side_effect=RuntimeError("macro crashed"))
        with pytest.raises(RuntimeError):
            tc.initialize()
        assert tc.status == STATUS_ERROR
        assert tc.status != STATUS_INITIALIZING

    def test_detection_failure_sets_error(self):
        """When require_tool_present=True and no tool detected, status=ERROR."""
        tc = _make_toolchanger()
        tc.has_detection = True
        tc.require_tool_present = True
        # All tools ABSENT
        t0 = _make_tool("extruder", 0, detect_state=DETECT_ABSENT)
        tc.tools = {0: t0}
        tc._run_gcode = MagicMock()
        with pytest.raises(Exception, match="No tool detected"):
            tc.initialize()
        assert tc.status == STATUS_ERROR

    def test_initialize_rejects_unsampled_tool(self):
        """Tool with detect_state != PRESENT should be rejected during init."""
        tc = _make_toolchanger()
        tc.has_detection = True
        tc.require_tool_present = False
        tool = _make_tool("extruder", 0, detect_state=DETECT_UNAVAILABLE)
        tc.tools = {0: tool}
        tc._run_gcode = MagicMock()
        tc.initialize(select_tool=tool)
        assert tc.status == STATUS_READY
        # Tool should have been rejected — active_tool should be None
        assert tc.active_tool is None

    def test_double_initialize_skips_gcode(self):
        """Calling initialize when already INITIALIZING skips init gcode."""
        tc = _make_toolchanger()
        tc.status = STATUS_INITIALIZING
        tc.has_detection = False
        tc.require_tool_present = False
        tc._run_gcode = MagicMock()
        # should_run_init = False, so init gcode won't run
        tc.initialize()
        # _run_gcode may be called for after_change_gcode but not for initialize_gcode
        # status should still be INITIALIZING (no transition in non-should_run_init path)
        # Actually with our code, should_run_init=False means the final block doesn't run
        # So status stays at INITIALIZING
        assert tc.status == STATUS_INITIALIZING


# ── FanSwitcher extended ─────────────────────────────────────────────────────

class TestFanSwitcherExtended:
    def test_transfer_speed_from_old_fan(self):
        """Switching fans should transfer speed from old to new fan."""
        tc = _make_toolchanger()
        fs = FanSwitcher(tc)
        old_fan = MagicMock()
        old_fan.fan_name = "T0_partfan"
        old_fan.get_status.return_value = {'speed': 0.75}
        new_fan = MagicMock()
        new_fan.fan_name = "T1_partfan"
        fs.active_fan = old_fan
        fs.activate_fan(new_fan)
        assert fs.active_fan is new_fan
        # Old fan should be turned off, new fan set to 0.75
        assert any("T0_partfan" in str(c) and "0.0" in str(c)
                    for c in tc.gcode.run_script_from_command.call_args_list)
        assert any("T1_partfan" in str(c) and "0.75" in str(c)
                    for c in tc.gcode.run_script_from_command.call_args_list)

    def test_no_transfer_when_disabled(self):
        """transfer_fan_speed=False should skip fan switching."""
        tc = _make_toolchanger()
        tc.transfer_fan_speed = False
        fs = FanSwitcher(tc)
        old_fan = MagicMock()
        old_fan.fan_name = "T0_partfan"
        new_fan = MagicMock()
        new_fan.fan_name = "T1_partfan"
        fs.active_fan = old_fan
        fs.activate_fan(new_fan)
        # Should not have switched — still old fan
        assert fs.active_fan is old_fan

    def test_pending_speed_applied_to_new_fan(self):
        """Pending speed (set before any fan exists) should apply to first fan."""
        tc = _make_toolchanger()
        fs = FanSwitcher(tc)
        fs.pending_speed = 0.5
        fan = MagicMock()
        fan.fan_name = "T0_partfan"
        fs.activate_fan(fan)
        assert fs.active_fan is fan
        assert fs.pending_speed is None
        assert any("T0_partfan" in str(c) and "0.5" in str(c)
                    for c in tc.gcode.run_script_from_command.call_args_list)

    def test_deactivating_stores_pending_speed(self):
        """Switching to None should store current speed as pending."""
        tc = _make_toolchanger()
        fs = FanSwitcher(tc)
        old_fan = MagicMock()
        old_fan.fan_name = "T0_partfan"
        old_fan.get_status.return_value = {'speed': 0.8}
        fs.active_fan = old_fan
        fs.activate_fan(None)
        assert fs.active_fan is None
        assert fs.pending_speed == 0.8

    def test_same_fan_is_noop(self):
        """Activating the same fan should be a no-op."""
        tc = _make_toolchanger()
        fs = FanSwitcher(tc)
        fan = MagicMock()
        fan.fan_name = "T0_partfan"
        fs.active_fan = fan
        fs.activate_fan(fan)
        tc.gcode.run_script_from_command.assert_not_called()

    def test_m106_sets_speed(self):
        """M106 should set fan speed on active tool's fan."""
        tc = _make_toolchanger()
        fs = FanSwitcher(tc)
        tool = _make_tool("extruder0", 0)
        tool.fan = MagicMock()
        tool.fan.fan_name = "T0_partfan"
        tc.active_tool = tool
        tc.tools = {0: tool}
        gcmd = MagicMock()
        gcmd.get_float.return_value = 127.5
        gcmd.get_int.return_value = None
        fs.cmd_M106(gcmd)
        assert any("T0_partfan" in str(c)
                    for c in tc.gcode.run_script_from_command.call_args_list)

    def test_m107_turns_off_fan(self):
        """M107 should set fan speed to 0."""
        tc = _make_toolchanger()
        fs = FanSwitcher(tc)
        tool = _make_tool("extruder0", 0)
        tool.fan = MagicMock()
        tool.fan.fan_name = "T0_partfan"
        tc.active_tool = tool
        tc.tools = {0: tool}
        gcmd = MagicMock()
        gcmd.get_int.return_value = None
        fs.cmd_M107(gcmd)
        assert any("T0_partfan" in str(c) and "0.0" in str(c)
                    for c in tc.gcode.run_script_from_command.call_args_list)
