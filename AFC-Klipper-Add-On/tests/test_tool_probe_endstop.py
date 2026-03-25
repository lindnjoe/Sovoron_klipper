"""Tests for tool_probe_endstop.py — probe routing, detection, crash detection."""

import sys
import types
from unittest.mock import MagicMock, patch

# Mock klipper's probe module before importing tool_probe_endstop
_probe_mock = types.ModuleType("extras.probe")
_probe_mock.ProbeCommandHelper = MagicMock
_probe_mock.ProbeOffsetsHelper = MagicMock
_probe_mock.ProbeParameterHelper = MagicMock
_probe_mock.ProbeEndstopWrapper = MagicMock
_probe_mock.HomingViaProbeHelper = MagicMock
_probe_mock.ProbeSessionHelper = MagicMock
sys.modules.setdefault("extras.probe", _probe_mock)

from extras.tool_probe_endstop import (
    ToolProbeEndstop,
    ProbeRouter,
    EndstopRouter,
)
from tests.conftest import MockLogger, MockReactor


# ── Helpers ──────────────────────────────────────────────────────────────────

def _make_tool_probe(name="tool_probe T0", tool_number=0, triggered=False):
    """Build a mock tool_probe object."""
    tp = MagicMock()
    tp.name = name
    tp.tool_number = tool_number
    tp.mcu_probe = MagicMock()
    tp.mcu_probe.query_endstop.return_value = triggered
    tp.probe_offsets = MagicMock()
    tp.probe_offsets.get_offsets.return_value = (0.0, 0.0, -1.5)
    tp.param_helper = MagicMock()
    return tp


def _make_endstop(standalone=False):
    """Build a ToolProbeEndstop bypassing __init__."""
    obj = ToolProbeEndstop.__new__(ToolProbeEndstop)
    obj.printer = MagicMock()
    obj.reactor = MockReactor()
    obj.name = "tool_probe_endstop"
    obj.probes = []
    obj.tool_number_to_probe = {}
    obj.last_query = {}
    obj.active_probe = None
    obj.active_tool_number = -1
    obj.gcode_macro = MagicMock()
    obj.logger = MockLogger()
    obj.crash_detection_active = False
    obj.crash_lasttime = 0.0
    obj.crash_mintime = 0.5
    obj.crash_gcode = MagicMock()
    obj.mcu_probe = EndstopRouter(obj.printer)
    obj.probe = ProbeRouter(obj.printer)
    obj.probe_offsets = obj.probe
    obj.param_helper = obj.probe
    obj.cmd_helper = MagicMock()
    obj.cmd_helper.get_status.return_value = {}
    obj.toolhead = MagicMock()
    obj.gcode = MagicMock()
    return obj


# ── _describe_tool_detection_issue ───────────────────────────────────────────

class TestDescribeToolDetectionIssue:
    def test_single_candidate_is_ok(self):
        obj = _make_endstop()
        tp = _make_tool_probe()
        result = obj._describe_tool_detection_issue([tp])
        assert result == "OK"

    def test_no_candidates_all_triggered(self):
        obj = _make_endstop()
        result = obj._describe_tool_detection_issue([])
        assert "All probes triggered" in result

    def test_multiple_candidates_lists_names(self):
        """Multiple untriggered probes should list their names (not map object)."""
        obj = _make_endstop()
        tp0 = _make_tool_probe("probe_T0", 0)
        tp1 = _make_tool_probe("probe_T1", 1)
        result = obj._describe_tool_detection_issue([tp0, tp1])
        # Must contain actual names, not <map object>
        assert "probe_T0" in result
        assert "probe_T1" in result
        assert "map object" not in result


# ── set_active_probe ─────────────────────────────────────────────────────────

class TestSetActiveProbe:
    def test_set_active_probe(self):
        obj = _make_endstop()
        tp = _make_tool_probe("probe_T0", 0)
        obj.set_active_probe(tp)
        assert obj.active_probe is tp
        assert obj.active_tool_number == 0

    def test_set_active_probe_none(self):
        obj = _make_endstop()
        tp = _make_tool_probe("probe_T0", 0)
        obj.set_active_probe(tp)
        obj.set_active_probe(None)
        assert obj.active_probe is None
        assert obj.active_tool_number == -1

    def test_set_same_probe_is_noop(self):
        obj = _make_endstop()
        tp = _make_tool_probe("probe_T0", 0)
        obj.set_active_probe(tp)
        obj.probe.set_active_probe = MagicMock()
        obj.set_active_probe(tp)
        # Should not re-set since it's the same probe
        obj.probe.set_active_probe.assert_not_called()


# ── add_probe ────────────────────────────────────────────────────────────────

class TestAddProbe:
    def test_add_probe_registers(self):
        obj = _make_endstop()
        tp = _make_tool_probe("probe_T0", 0)
        config = MagicMock()
        obj.add_probe(config, tp)
        assert tp in obj.probes
        assert obj.tool_number_to_probe[0] is tp

    def test_add_probe_no_tool_number(self):
        """Probe without tool_number should still be added to probes list."""
        obj = _make_endstop()
        tp = _make_tool_probe("probe_generic", tool_number=None)
        config = MagicMock()
        obj.add_probe(config, tp)
        assert tp in obj.probes
        assert None not in obj.tool_number_to_probe

    def test_duplicate_tool_number_raises(self):
        obj = _make_endstop()
        tp1 = _make_tool_probe("probe_T0_a", 0)
        tp2 = _make_tool_probe("probe_T0_b", 0)
        config = MagicMock()
        config.error = lambda msg: Exception(msg)
        obj.add_probe(config, tp1)
        try:
            obj.add_probe(config, tp2)
            assert False, "Should have raised"
        except Exception as e:
            assert "Duplicate" in str(e)


# ── _query_open_tools ────────────────────────────────────────────────────────

class TestQueryOpenTools:
    def test_single_open_tool(self):
        obj = _make_endstop()
        tp0 = _make_tool_probe("probe_T0", 0, triggered=True)   # docked
        tp1 = _make_tool_probe("probe_T1", 1, triggered=False)  # on shuttle
        obj.probes = [tp0, tp1]
        candidates = obj._query_open_tools()
        assert len(candidates) == 1
        assert candidates[0] is tp1

    def test_all_triggered(self):
        obj = _make_endstop()
        tp0 = _make_tool_probe("probe_T0", 0, triggered=True)
        tp1 = _make_tool_probe("probe_T1", 1, triggered=True)
        obj.probes = [tp0, tp1]
        candidates = obj._query_open_tools()
        assert len(candidates) == 0

    def test_multiple_open(self):
        obj = _make_endstop()
        tp0 = _make_tool_probe("probe_T0", 0, triggered=False)
        tp1 = _make_tool_probe("probe_T1", 1, triggered=False)
        obj.probes = [tp0, tp1]
        candidates = obj._query_open_tools()
        assert len(candidates) == 2

    def test_updates_last_query(self):
        obj = _make_endstop()
        tp0 = _make_tool_probe("probe_T0", 0, triggered=True)
        tp1 = _make_tool_probe("probe_T1", 1, triggered=False)
        obj.probes = [tp0, tp1]
        obj._query_open_tools()
        assert obj.last_query[0] is True
        assert obj.last_query[1] is False


# ── _detect_active_tool ──────────────────────────────────────────────────────

class TestDetectActiveTool:
    def test_single_open_sets_active(self):
        obj = _make_endstop()
        tp0 = _make_tool_probe("probe_T0", 0, triggered=True)
        tp1 = _make_tool_probe("probe_T1", 1, triggered=False)
        obj.probes = [tp0, tp1]
        obj._detect_active_tool()
        assert obj.active_probe is tp1

    def test_multiple_open_sets_none(self):
        obj = _make_endstop()
        tp0 = _make_tool_probe("probe_T0", 0, triggered=False)
        tp1 = _make_tool_probe("probe_T1", 1, triggered=False)
        obj.probes = [tp0, tp1]
        obj._detect_active_tool()
        assert obj.active_probe is None


# ── Crash detection ──────────────────────────────────────────────────────────

class TestCrashDetection:
    def test_note_probe_triggered_ignores_when_inactive(self):
        obj = _make_endstop()
        obj.crash_detection_active = False
        tp = _make_tool_probe()
        obj.active_probe = tp
        obj.note_probe_triggered(tp, 100.0, True)
        assert obj.crash_lasttime == 0.0

    def test_note_probe_triggered_ignores_wrong_probe(self):
        obj = _make_endstop()
        obj.crash_detection_active = True
        tp0 = _make_tool_probe("probe_T0", 0)
        tp1 = _make_tool_probe("probe_T1", 1)
        obj.active_probe = tp0
        obj.note_probe_triggered(tp1, 100.0, True)
        assert obj.crash_lasttime == 0.0

    def test_note_probe_triggered_sets_lasttime(self):
        obj = _make_endstop()
        obj.crash_detection_active = True
        tp = _make_tool_probe()
        obj.active_probe = tp
        obj.note_probe_triggered(tp, 100.0, True)
        assert obj.crash_lasttime == 100.0

    def test_note_probe_untriggered_clears_lasttime(self):
        obj = _make_endstop()
        obj.crash_detection_active = True
        tp = _make_tool_probe()
        obj.active_probe = tp
        obj.crash_lasttime = 100.0
        obj.note_probe_triggered(tp, 101.0, False)
        assert obj.crash_lasttime == 0.0

    def test_delayed_callback_fires_crash(self):
        """If crash_lasttime matches, delayed callback should fire crash gcode."""
        obj = _make_endstop()
        obj.crash_detection_active = True
        obj.crash_lasttime = 100.0
        obj._probe_triggered_delayed(100.0)
        obj.crash_gcode.run_gcode_from_command.assert_called_once()
        assert obj.crash_detection_active is False

    def test_delayed_callback_cancelled(self):
        """If crash_lasttime changed, delayed callback should not fire."""
        obj = _make_endstop()
        obj.crash_detection_active = True
        obj.crash_lasttime = 0.0  # Reset (probe was un-triggered)
        obj._probe_triggered_delayed(100.0)
        obj.crash_gcode.run_gcode_from_command.assert_not_called()
        assert obj.crash_detection_active is True

    def test_stop_crash_detection(self):
        obj = _make_endstop()
        obj.crash_detection_active = True
        obj.crash_lasttime = 50.0
        obj.stop_crash_detection()
        assert obj.crash_detection_active is False
        assert obj.crash_lasttime == 0.0


# ── get_status ───────────────────────────────────────────────────────────────

class TestGetStatus:
    def test_status_no_active_probe(self):
        obj = _make_endstop()
        status = obj.get_status(0)
        assert status['active_tool_probe'] is None
        assert status['active_tool_number'] == -1

    def test_status_with_active_probe(self):
        obj = _make_endstop()
        tp = _make_tool_probe("probe_T0", 0)
        obj.set_active_probe(tp)
        status = obj.get_status(0)
        assert status['active_tool_probe'] == "probe_T0"
        assert status['active_tool_number'] == 0


# ── ProbeRouter ──────────────────────────────────────────────────────────────

class TestProbeRouter:
    def test_no_active_probe_returns_zero_offsets(self):
        router = ProbeRouter(MagicMock())
        offsets = router.get_offsets()
        assert offsets == (0.0, 0.0, 0.0)

    def test_active_probe_returns_probe_offsets(self):
        router = ProbeRouter(MagicMock())
        tp = _make_tool_probe()
        tp.probe_offsets.get_offsets.return_value = (1.0, 2.0, -1.5)
        router.set_active_probe(tp)
        offsets = router.get_offsets()
        assert offsets == (1.0, 2.0, -1.5)

    def test_no_probe_get_probe_params_raises(self):
        router = ProbeRouter(MagicMock())
        try:
            router.get_probe_params()
            assert False, "Should have raised"
        except Exception:
            pass


# ── EndstopRouter ────────────────────────────────────────────────────────────

class TestEndstopRouter:
    def test_no_active_mcu_query_raises(self):
        router = EndstopRouter(MagicMock())
        try:
            router.query_endstop(0.0)
            assert False, "Should have raised"
        except Exception:
            pass

    def test_no_active_mcu_position_returns_zero(self):
        router = EndstopRouter(MagicMock())
        assert router.get_position_endstop() == 0.0

    def test_active_mcu_delegates_query(self):
        router = EndstopRouter(MagicMock())
        mcu = MagicMock()
        mcu.query_endstop.return_value = True
        router.set_active_mcu(mcu)
        assert router.query_endstop(0.0) is True

    def test_add_stepper_propagates_to_all_mcus(self):
        router = EndstopRouter(MagicMock())
        mcu1 = MagicMock()
        mcu2 = MagicMock()
        router.add_mcu(mcu1)
        router.add_mcu(mcu2)
        stepper = MagicMock()
        router.add_stepper(stepper)
        mcu1.add_stepper.assert_called_with(stepper)
        mcu2.add_stepper.assert_called_with(stepper)
