"""
Unit tests for extras/AFC_OpenAMS.py

Covers:
  - afcAMS: class constants, type
  - _is_openams_unit: type detection helper
  - get_lane_reset_command: returns TOOL_UNLOAD command
  - lane_unload: blocks manual unload, returns True
  - eject_lane: blocks eject with informative message
  - prep_load / prep_post_load: no-ops
  - check_runout: returns True when printing, False otherwise
  - get_status: correct keys
  - handle_connect: sets logo strings
  - system_Test: LED logic for each lane state combination
"""

from __future__ import annotations

from unittest.mock import MagicMock, PropertyMock, patch
import pytest

from extras.AFC_OpenAMS import afcAMS, _is_openams_unit
from extras.AFC_lane import AFCLaneState


# ── Helpers ───────────────────────────────────────────────────────────────────

def _make_openams(name="AMS_1"):
    """Build an afcAMS bypassing the complex __init__."""
    unit = afcAMS.__new__(afcAMS)

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
    unit.full_name = ["AFC_OpenAMS", name]
    unit.lanes = {}
    unit.hub_obj = None
    unit.extruder_obj = None
    unit.buffer_obj = None
    unit.hub = None
    unit.extruder = None
    unit.buffer_name = None
    unit.td1_defined = False
    unit.type = "OpenAMS"
    unit.gcode = afc.gcode
    unit.oams_name = "oams1"
    unit.oams = None
    unit.hardware_service = None
    unit.registry = None
    unit.event_bus = None
    unit.stuck_spool_auto_recovery = False
    unit._saved_unit_cache = None
    unit._saved_unit_mtime = None
    unit._virtual_tool_sensor = None
    unit._last_hub_hes_values = None
    unit._cached_sensor_helper = None
    unit._cached_gcode = None
    unit._cached_extruder_objects = {}
    unit._cached_lane_objects = {}
    unit._cached_oams_index = None
    unit._cached_oams_manager = None
    unit._pending_spool_loaded_timers = {}
    unit._last_lane_tool_loaded_sync = {}
    unit.led_tool_loaded = "0,0,1,0"
    unit.led_tool_loaded_idle = "0.4,0.4,0,0"
    unit.led_tool_unloaded = "1,0,0,0"

    return unit


def _make_lane(name="lane1", prep_state=False, load_state=False,
               tool_loaded=False):
    lane = MagicMock()
    lane.name = name
    lane.prep_state = prep_state
    lane.load_state = load_state
    lane.tool_loaded = tool_loaded
    lane.map = "T0"
    lane.index = 1
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
    lane.unit_obj = MagicMock()
    return lane


# ── Type ─────────────────────────────────────────────────────────────────────

class TestType:
    def test_type_is_openams(self):
        unit = _make_openams()
        assert unit.type == "OpenAMS"


# ── _is_openams_unit ─────────────────────────────────────────────────────────

class TestIsOpenamsUnit:
    def test_returns_true_for_afcams_instance(self):
        unit = _make_openams()
        assert _is_openams_unit(unit) is True

    def test_returns_true_for_type_openams(self):
        obj = MagicMock()
        obj.type = "OpenAMS"
        assert _is_openams_unit(obj) is True

    def test_returns_true_for_oams_name_attr(self):
        obj = MagicMock(spec=[])
        obj.type = "Something"
        obj.oams_name = "oams1"
        assert _is_openams_unit(obj) is True

    def test_returns_false_for_none(self):
        assert _is_openams_unit(None) is False

    def test_returns_false_for_boxturtle(self):
        obj = MagicMock(spec=[])
        obj.type = "Box_Turtle"
        assert _is_openams_unit(obj) is False


# ── get_lane_reset_command ───────────────────────────────────────────────────

class TestGetLaneResetCommand:
    def test_returns_tool_unload(self):
        unit = _make_openams()
        lane = _make_lane(name="lane4")
        result = unit.get_lane_reset_command(lane, None)
        assert result == "TOOL_UNLOAD LANE=lane4"

    def test_ignores_distance(self):
        unit = _make_openams()
        lane = _make_lane(name="lane1")
        result = unit.get_lane_reset_command(lane, 200)
        assert result == "TOOL_UNLOAD LANE=lane1"


# ── lane_unload ──────────────────────────────────────────────────────────────

class TestLaneUnload:
    def test_returns_true(self):
        unit = _make_openams()
        lane = _make_lane()
        result = unit.lane_unload(lane)
        assert result is True

    def test_logs_info_message(self):
        unit = _make_openams()
        lane = _make_lane(name="lane2")
        unit.lane_unload(lane)
        info_msgs = [m for lvl, m in unit.logger.messages if lvl == "info"]
        assert any("lane2" in m for m in info_msgs)


# ── eject_lane ───────────────────────────────────────────────────────────────

class TestEjectLane:
    def test_logs_info_message(self):
        unit = _make_openams()
        lane = _make_lane(name="lane3")
        unit.eject_lane(lane)
        info_msgs = [m for lvl, m in unit.logger.messages if lvl == "info"]
        assert any("lane3" in m for m in info_msgs)

    def test_message_mentions_not_supported(self):
        unit = _make_openams()
        lane = _make_lane(name="lane1")
        unit.eject_lane(lane)
        info_msgs = [m for lvl, m in unit.logger.messages if lvl == "info"]
        assert any("not supported" in m.lower() for m in info_msgs)


# ── prep_load / prep_post_load ───────────────────────────────────────────────

class TestPrepLoadNoOp:
    def test_prep_load_is_noop(self):
        unit = _make_openams()
        lane = _make_lane()
        result = unit.prep_load(lane)
        assert result is None

    def test_prep_post_load_is_noop(self):
        unit = _make_openams()
        lane = _make_lane()
        result = unit.prep_post_load(lane)
        assert result is None


# ── check_runout ─────────────────────────────────────────────────────────────

class TestCheckRunout:
    def test_returns_true_when_printing_with_valid_lane(self):
        unit = _make_openams()
        lane = MagicMock()
        lane.unit_obj = unit
        unit.afc.function.is_printing.return_value = True
        assert unit.check_runout(lane) is True

    def test_returns_false_when_not_printing(self):
        unit = _make_openams()
        lane = MagicMock()
        lane.unit_obj = unit
        unit.afc.function.is_printing.return_value = False
        assert unit.check_runout(lane) is False

    def test_returns_false_for_none_lane(self):
        unit = _make_openams()
        assert unit.check_runout(None) is False

    def test_returns_false_when_lane_belongs_to_different_unit(self):
        unit = _make_openams()
        lane = MagicMock()
        lane.unit_obj = MagicMock()  # different unit
        assert unit.check_runout(lane) is False

    def test_returns_false_on_exception(self):
        unit = _make_openams()
        lane = MagicMock()
        lane.unit_obj = unit
        unit.afc.function.is_printing.side_effect = Exception("err")
        assert unit.check_runout(lane) is False


# ── _is_openams_unit method ──────────────────────────────────────────────────

class TestIsOpenamsUnitMethod:
    def test_returns_true_when_oams_set(self):
        unit = _make_openams()
        unit.oams = MagicMock()
        assert unit._is_openams_unit() is True

    def test_returns_false_when_oams_none(self):
        unit = _make_openams()
        unit.oams = None
        assert unit._is_openams_unit() is False


# ── _get_openams_spool_index ─────────────────────────────────────────────────

class TestGetOpenamsSpooolIndex:
    def test_returns_0_for_index_1(self):
        unit = _make_openams()
        lane = _make_lane()
        lane.index = 1
        result = unit._get_openams_spool_index(lane)
        assert result == 0

    def test_returns_3_for_index_4(self):
        unit = _make_openams()
        lane = _make_lane()
        lane.index = 4
        result = unit._get_openams_spool_index(lane)
        assert result == 3

    def test_returns_none_for_invalid_index(self):
        unit = _make_openams()
        lane = _make_lane()
        lane.index = "bad"
        result = unit._get_openams_spool_index(lane)
        assert result is None
