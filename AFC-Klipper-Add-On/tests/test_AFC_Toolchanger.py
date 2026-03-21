"""Tests for AFC_Toolchanger – native toolchanger engine."""

from unittest.mock import MagicMock

from extras.AFC_Toolchanger import (
    AfcToolchanger,
    ToolGcodeTransform,
    FanSwitcher,
    STATUS_UNINITIALIZED,
    STATUS_READY,
    STATUS_CHANGING,
    STATUS_ERROR,
    DETECT_ABSENT,
    DETECT_PRESENT,
    DETECT_UNAVAILABLE,
    XYZ_TO_INDEX,
)
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
