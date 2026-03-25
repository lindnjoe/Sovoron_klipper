"""Tests for M109/M104 lane-to-extruder mapping in AFC.py.

Verifies that temperature commands heat the correct extruder when lanes
map to different physical extruders than the T-number might suggest.

Example scenario (toolchanger + AFC):
  - extruder4 has lanes: lane4, lane5, lane6, lane7
  - lane5 maps to T5
  - extruder5 exists with tool_number=5
  - M109 T5 should heat extruder4 (via lane5), NOT extruder5
"""

from unittest.mock import MagicMock

from extras.AFC import afc, State


# ── Helpers ──────────────────────────────────────────────────────────────────

def _make_afc_for_m109():
    """Build an afc instance wired for M109/M104 tests."""
    obj = afc.__new__(afc)

    from tests.conftest import MockAFC, MockLogger, MockPrinter, MockReactor

    inner = MockAFC()
    printer = MockPrinter(afc=inner)
    obj.printer = printer
    obj.logger = MockLogger()
    obj.reactor = inner.reactor
    obj.function = MagicMock()
    obj.toolhead = MagicMock()
    obj.lanes = {}
    obj.tools = {}
    obj.tool_cmds = {}
    obj.disable_ooze_check = True
    obj.deadband_auto = False
    obj.temp_wait_tolerance = 5
    # Track which heater was passed to set_temperature
    obj._heated_heaters = []
    return obj


def _make_extruder(name, tool_number=-1):
    """Build a mock extruder with heater.  Each heater has a ._name for easy assertions."""
    ext = MagicMock()
    ext.name = name
    ext.tool_number = tool_number
    ext.lane_loaded = None
    ext.lanes = []
    heater = MagicMock()
    heater._name = f"heater_{name}"  # easy identification in assertions
    heater.get_temp = MagicMock(return_value=(200.0, 0.0))
    ext.get_heater.return_value = heater
    return ext, heater


def _assert_heater_was_set(pheaters, expected_heater, temp):
    """Assert that set_temperature was called with the expected heater (at least once)."""
    calls = pheaters.set_temperature.call_args_list
    assert len(calls) > 0, "set_temperature was never called"
    # Check that at least one call used the expected heater
    matched = any(c[0][0] is expected_heater for c in calls)
    assert matched, (
        "Expected heater %s but got calls: %s" % (
            expected_heater._name,
            [(c[0][0]._name, c[0][1]) for c in calls if hasattr(c[0][0], '_name')])
    )
    # Check no OTHER heater was called
    wrong = [c for c in calls if c[0][0] is not expected_heater]
    assert len(wrong) == 0, (
        "Unexpected heater was also heated: %s" % (
            [(c[0][0]._name, c[0][1]) for c in wrong if hasattr(c[0][0], '_name')])
    )


def _make_lane(name, map_name, extruder_obj):
    """Build a mock lane mapped to T<n>."""
    lane = MagicMock()
    lane.name = name
    lane.map = map_name
    lane.extruder_obj = extruder_obj
    return lane


def _make_gcmd(toolnum=None, temp=250.0, deadband=None, extruder_name=None):
    """Build a mock gcode command with T/S/D/EXTRUDER params."""
    gcmd = MagicMock()
    gcmd.get = MagicMock(side_effect=lambda key, default=None, **kw:
                         extruder_name if key == 'EXTRUDER' else default)
    gcmd.get_int = MagicMock(side_effect=lambda key, default=None, **kw:
                             toolnum if key == 'T' else default)
    gcmd.get_float = MagicMock(side_effect=lambda key, default=0.0, **kw:
                               temp if key == 'S' else
                               (deadband if key == 'D' else default))
    return gcmd


# ── EXTRUDER= direct lookup ─────────────────────────────────────────────────

class TestM109ExtruderParam:
    """EXTRUDER= parameter bypasses lane map and targets physical extruder."""

    def test_extruder_param_bypasses_lane_map(self):
        """M109 EXTRUDER=extruder5 heats extruder5 even though T5 would map to extruder4."""
        obj = _make_afc_for_m109()

        ext4, heater4 = _make_extruder("extruder4", tool_number=4)
        ext5, heater5 = _make_extruder("extruder5", tool_number=5)
        lane5 = _make_lane("lane5", "T5", ext4)
        obj.lanes = {"lane5": lane5}
        obj.tools = {"extruder4": ext4, "extruder5": ext5}
        obj.function.get_lane_by_map = MagicMock(return_value=lane5)

        pheaters = MagicMock()
        obj.printer.lookup_object = MagicMock(return_value=pheaters)

        gcmd = _make_gcmd(temp=220.0, extruder_name="extruder5")
        obj._cmd_AFC_M109(gcmd, wait=False)

        # Must heat extruder5, NOT extruder4 (which T5 lane map would give)
        _assert_heater_was_set(pheaters, heater5, 220.0)

    def test_extruder_param_unknown_name(self):
        """M109 EXTRUDER=nonexistent should log error and not heat."""
        obj = _make_afc_for_m109()
        obj.tools = {}

        pheaters = MagicMock()
        obj.printer.lookup_object = MagicMock(return_value=pheaters)

        gcmd = _make_gcmd(temp=200.0, extruder_name="nonexistent")
        obj._cmd_AFC_M109(gcmd, wait=False)

        pheaters.set_temperature.assert_not_called()
        errors = [m for level, m in obj.logger.messages if level == "error"]
        assert any("nonexistent" in e for e in errors)

    def test_extruder_param_strips_equals_prefix(self):
        """Klipper M-code parsing adds '=' prefix — should be stripped."""
        obj = _make_afc_for_m109()
        ext5, heater5 = _make_extruder("extruder5", tool_number=5)
        obj.tools = {"extruder5": ext5}

        pheaters = MagicMock()
        obj.printer.lookup_object = MagicMock(return_value=pheaters)

        gcmd = _make_gcmd(temp=220.0, extruder_name="=extruder5")
        obj._cmd_AFC_M109(gcmd, wait=False)

        _assert_heater_was_set(pheaters, heater5, 220.0)

    def test_extruder_param_number_converts_to_name(self):
        """EXTRUDER=5 should resolve to extruder5."""
        obj = _make_afc_for_m109()
        ext5, heater5 = _make_extruder("extruder5", tool_number=5)
        obj.tools = {"extruder5": ext5}

        pheaters = MagicMock()
        obj.printer.lookup_object = MagicMock(return_value=pheaters)

        gcmd = _make_gcmd(temp=220.0, extruder_name="5")
        obj._cmd_AFC_M109(gcmd, wait=False)

        _assert_heater_was_set(pheaters, heater5, 220.0)

    def test_extruder_param_zero_converts_to_extruder(self):
        """EXTRUDER=0 should resolve to 'extruder' (base extruder)."""
        obj = _make_afc_for_m109()
        ext0, heater0 = _make_extruder("extruder", tool_number=0)
        obj.tools = {"extruder": ext0}

        pheaters = MagicMock()
        obj.printer.lookup_object = MagicMock(return_value=pheaters)

        gcmd = _make_gcmd(temp=200.0, extruder_name="0")
        obj._cmd_AFC_M109(gcmd, wait=False)

        _assert_heater_was_set(pheaters, heater0, 200.0)

    def test_extruder_param_equals_plus_number(self):
        """EXTRUDER==5 (klipper M-code double-equals) should resolve to extruder5."""
        obj = _make_afc_for_m109()
        ext5, heater5 = _make_extruder("extruder5", tool_number=5)
        obj.tools = {"extruder5": ext5}

        pheaters = MagicMock()
        obj.printer.lookup_object = MagicMock(return_value=pheaters)

        gcmd = _make_gcmd(temp=220.0, extruder_name="=5")
        obj._cmd_AFC_M109(gcmd, wait=False)

        _assert_heater_was_set(pheaters, heater5, 220.0)

    def test_extruder_param_takes_priority_over_t(self):
        """When both EXTRUDER= and T= are provided, EXTRUDER= wins."""
        obj = _make_afc_for_m109()

        ext4, heater4 = _make_extruder("extruder4", tool_number=4)
        ext5, heater5 = _make_extruder("extruder5", tool_number=5)
        lane5 = _make_lane("lane5", "T5", ext4)
        obj.lanes = {"lane5": lane5}
        obj.tools = {"extruder4": ext4, "extruder5": ext5}
        obj.function.get_lane_by_map = MagicMock(return_value=lane5)

        pheaters = MagicMock()
        obj.printer.lookup_object = MagicMock(return_value=pheaters)

        # Both EXTRUDER=extruder5 and T=5 — EXTRUDER should win
        gcmd = _make_gcmd(toolnum=5, temp=230.0, extruder_name="extruder5")
        obj._cmd_AFC_M109(gcmd, wait=False)

        _assert_heater_was_set(pheaters, heater5, 230.0)


# ── Lane map priority ───────────────────────────────────────────────────────

class TestM109LaneMapPriority:
    """Lane map lookup must take priority over tool_number lookup."""

    def test_lane_map_takes_priority_over_tool_number(self):
        """M109 T5 heats extruder4 (via lane5 map) not extruder5 (tool_number=5)."""
        obj = _make_afc_for_m109()

        # extruder4 owns lane5, which maps to T5
        ext4, heater4 = _make_extruder("extruder4", tool_number=4)
        # extruder5 has tool_number=5 — the WRONG one to heat
        ext5, heater5 = _make_extruder("extruder5", tool_number=5)

        lane5 = _make_lane("lane5", "T5", ext4)
        obj.lanes = {"lane5": lane5}
        obj.tools = {"extruder4": ext4, "extruder5": ext5}
        obj.function.get_lane_by_map = MagicMock(return_value=lane5)

        pheaters = MagicMock()
        obj.printer.lookup_object = MagicMock(return_value=pheaters)

        gcmd = _make_gcmd(toolnum=5, temp=250.0)
        obj._cmd_AFC_M109(gcmd, wait=False)

        # Must heat extruder4's heater, NOT extruder5's
        _assert_heater_was_set(pheaters, heater4, 250.0)

    def test_tool_number_fallback_when_no_lane_map(self):
        """When no lane maps to T5, fall back to extruder with tool_number=5."""
        obj = _make_afc_for_m109()

        ext5, heater5 = _make_extruder("extruder5", tool_number=5)
        obj.tools = {"extruder5": ext5}
        obj.function.get_lane_by_map = MagicMock(return_value=None)

        pheaters = MagicMock()
        obj.printer.lookup_object = MagicMock(return_value=pheaters)

        gcmd = _make_gcmd(toolnum=5, temp=220.0)
        obj._cmd_AFC_M109(gcmd, wait=False)

        _assert_heater_was_set(pheaters, heater5, 220.0)

    def test_no_t_param_uses_current_extruder(self):
        """M109 without T parameter heats the currently active extruder."""
        obj = _make_afc_for_m109()

        heater = MagicMock()
        heater._name = "heater_current"
        heater.get_temp = MagicMock(return_value=(200.0, 0.0))
        current_ext = MagicMock()
        current_ext.get_heater.return_value = heater
        obj.toolhead.get_extruder.return_value = current_ext

        pheaters = MagicMock()
        obj.printer.lookup_object = MagicMock(return_value=pheaters)

        gcmd = _make_gcmd(toolnum=None, temp=200.0)
        obj._cmd_AFC_M109(gcmd, wait=False)

        _assert_heater_was_set(pheaters, heater, 200.0)

    def test_unconfigured_tool_returns_early(self):
        """M109 T99 with no lane map and no tool_number match should log error."""
        obj = _make_afc_for_m109()
        obj.tools = {}
        obj.function.get_lane_by_map = MagicMock(return_value=None)

        pheaters = MagicMock()
        obj.printer.lookup_object = MagicMock(return_value=pheaters)

        gcmd = _make_gcmd(toolnum=99, temp=200.0)
        obj._cmd_AFC_M109(gcmd, wait=False)

        # Should not call set_temperature at all
        pheaters.set_temperature.assert_not_called()
        # Should have logged error
        errors = [m for level, m in obj.logger.messages if level == "error"]
        assert any("T99" in e for e in errors)


# ── Multi-extruder mapping scenarios ─────────────────────────────────────────

class TestMultiExtruderMapping:
    """Test correct extruder resolution in multi-extruder toolchanger setups."""

    def _setup_8_extruder_4_lanes_each(self):
        """Simulate 8 extruders with 4 lanes each = 32 lanes total.

        Mapping: extruder0 has lane0-3 (T0-T3),
                 extruder1 has lane4-7 (T4-T7), etc.
        """
        obj = _make_afc_for_m109()
        extruders = {}
        heaters = {}
        lanes = {}
        lane_map = {}

        for ext_idx in range(8):
            ext, heater = _make_extruder(
                f"extruder{ext_idx}" if ext_idx > 0 else "extruder",
                tool_number=ext_idx)
            extruders[ext.name] = ext
            heaters[ext.name] = heater
            ext_lanes = []
            for lane_offset in range(4):
                lane_idx = ext_idx * 4 + lane_offset
                lane_name = f"lane{lane_idx}"
                map_name = f"T{lane_idx}"
                lane = _make_lane(lane_name, map_name, ext)
                lanes[lane_name] = lane
                lane_map[map_name] = lane
                ext_lanes.append(lane_name)
            ext.lanes = ext_lanes

        obj.lanes = lanes
        obj.tools = extruders

        def get_lane_by_map(m):
            return lane_map.get(m)
        obj.function.get_lane_by_map = MagicMock(side_effect=get_lane_by_map)

        return obj, extruders, heaters

    def test_t5_heats_extruder1_not_extruder5(self):
        """T5 = lane5 on extruder1 (lanes 4-7), not extruder5."""
        obj, extruders, heaters = self._setup_8_extruder_4_lanes_each()
        pheaters = MagicMock()
        obj.printer.lookup_object = MagicMock(return_value=pheaters)

        gcmd = _make_gcmd(toolnum=5, temp=250.0)
        obj._cmd_AFC_M109(gcmd, wait=False)

        _assert_heater_was_set(pheaters, heaters["extruder1"], 250.0)

    def test_t12_heats_extruder3(self):
        """T12 = lane12 on extruder3 (lanes 12-15)."""
        obj, extruders, heaters = self._setup_8_extruder_4_lanes_each()
        pheaters = MagicMock()
        obj.printer.lookup_object = MagicMock(return_value=pheaters)

        gcmd = _make_gcmd(toolnum=12, temp=210.0)
        obj._cmd_AFC_M109(gcmd, wait=False)

        _assert_heater_was_set(pheaters, heaters["extruder3"], 210.0)

    def test_t0_heats_extruder0(self):
        """T0 = lane0 on extruder (base extruder)."""
        obj, extruders, heaters = self._setup_8_extruder_4_lanes_each()
        pheaters = MagicMock()
        obj.printer.lookup_object = MagicMock(return_value=pheaters)

        gcmd = _make_gcmd(toolnum=0, temp=200.0)
        obj._cmd_AFC_M109(gcmd, wait=False)

        _assert_heater_was_set(pheaters, heaters["extruder"], 200.0)

    def test_t31_heats_extruder7(self):
        """T31 = lane31 on extruder7 (lanes 28-31)."""
        obj, extruders, heaters = self._setup_8_extruder_4_lanes_each()
        pheaters = MagicMock()
        obj.printer.lookup_object = MagicMock(return_value=pheaters)

        gcmd = _make_gcmd(toolnum=31, temp=230.0)
        obj._cmd_AFC_M109(gcmd, wait=False)

        _assert_heater_was_set(pheaters, heaters["extruder7"], 230.0)

    def test_each_lane_heats_correct_extruder(self):
        """Every lane T0-T31 should heat the correct physical extruder."""
        obj, extruders, heaters = self._setup_8_extruder_4_lanes_each()

        for lane_idx in range(32):
            expected_ext_idx = lane_idx // 4
            expected_ext_name = (
                "extruder" if expected_ext_idx == 0
                else f"extruder{expected_ext_idx}")

            pheaters = MagicMock()
            obj.printer.lookup_object = MagicMock(return_value=pheaters)

            gcmd = _make_gcmd(toolnum=lane_idx, temp=250.0)
            obj._cmd_AFC_M109(gcmd, wait=False)

            _assert_heater_was_set(pheaters, heaters[expected_ext_name], 250.0)


# ── M104 delegates to M109 ──────────────────────────────────────────────────

class TestM104DelegatesToM109:
    """M104 should use the same lane mapping logic as M109."""

    def test_m104_uses_lane_map(self):
        """M104 T5 should resolve via lane map, same as M109."""
        obj = _make_afc_for_m109()

        ext4, heater4 = _make_extruder("extruder4", tool_number=4)
        ext5, heater5 = _make_extruder("extruder5", tool_number=5)
        lane5 = _make_lane("lane5", "T5", ext4)
        obj.lanes = {"lane5": lane5}
        obj.tools = {"extruder4": ext4, "extruder5": ext5}
        obj.function.get_lane_by_map = MagicMock(return_value=lane5)

        pheaters = MagicMock()
        obj.printer.lookup_object = MagicMock(return_value=pheaters)

        gcmd = _make_gcmd(toolnum=5, temp=250.0)
        # M104 calls M109 with wait=False
        obj._cmd_AFC_M109(gcmd, wait=False)

        _assert_heater_was_set(pheaters, heater4, 250.0)


# ── Ooze prevention check ───────────────────────────────────────────────────

class TestOozePrevention:
    """Verify ooze prevention logic blocks heating wrong lanes on same extruder."""

    def test_ooze_check_blocks_other_lane_on_same_extruder(self):
        """M109 T1 when lane0 is loaded on same extruder should be blocked."""
        obj = _make_afc_for_m109()
        obj.disable_ooze_check = False

        ext, heater = _make_extruder("extruder", tool_number=0)
        ext.lanes = ["lane0", "lane1"]
        ext.lane_loaded = "lane0"

        lane0 = _make_lane("lane0", "T0", ext)
        lane1 = _make_lane("lane1", "T1", ext)
        obj.lanes = {"lane0": lane0, "lane1": lane1}
        obj.tools = {"extruder": ext}

        obj.function.get_lane_by_map = MagicMock(return_value=lane1)
        obj.function.get_current_extruder_obj = MagicMock(return_value=ext)

        pheaters = MagicMock()
        obj.printer.lookup_object = MagicMock(return_value=pheaters)

        gcmd = _make_gcmd(toolnum=1, temp=250.0)
        obj._cmd_AFC_M109(gcmd, wait=False)

        # Should NOT set temperature — ooze check blocks it
        pheaters.set_temperature.assert_not_called()
        # Should have logged warning
        raw_msgs = [m for level, m in obj.logger.messages if level == "raw"]
        assert any("WARNING" in m and "T1" in m for m in raw_msgs)

    def test_ooze_check_allows_loaded_lane(self):
        """M109 T0 when lane0 is the loaded lane should proceed."""
        obj = _make_afc_for_m109()
        obj.disable_ooze_check = False

        ext, heater = _make_extruder("extruder", tool_number=0)
        ext.lanes = ["lane0", "lane1"]
        ext.lane_loaded = "lane0"

        lane0 = _make_lane("lane0", "T0", ext)
        lane1 = _make_lane("lane1", "T1", ext)
        obj.lanes = {"lane0": lane0, "lane1": lane1}
        obj.tools = {"extruder": ext}

        obj.function.get_lane_by_map = MagicMock(return_value=lane0)
        obj.function.get_current_extruder_obj = MagicMock(return_value=ext)

        pheaters = MagicMock()
        obj.printer.lookup_object = MagicMock(return_value=pheaters)

        gcmd = _make_gcmd(toolnum=0, temp=250.0)
        obj._cmd_AFC_M109(gcmd, wait=False)

        _assert_heater_was_set(pheaters, heater, 250.0)

    def test_ooze_check_bypassed_when_no_lane_loaded(self):
        """M109 before any lane is loaded should proceed (start macro)."""
        obj = _make_afc_for_m109()
        obj.disable_ooze_check = False

        ext, heater = _make_extruder("extruder", tool_number=0)
        ext.lanes = ["lane0", "lane1"]
        ext.lane_loaded = None  # No lane loaded yet

        lane1 = _make_lane("lane1", "T1", ext)
        obj.lanes = {"lane1": lane1}
        obj.tools = {"extruder": ext}

        obj.function.get_lane_by_map = MagicMock(return_value=lane1)
        obj.function.get_current_extruder_obj = MagicMock(return_value=ext)

        pheaters = MagicMock()
        obj.printer.lookup_object = MagicMock(return_value=pheaters)

        gcmd = _make_gcmd(toolnum=1, temp=250.0)
        obj._cmd_AFC_M109(gcmd, wait=False)

        _assert_heater_was_set(pheaters, heater, 250.0)

    def test_ooze_check_disabled_allows_all(self):
        """disable_ooze_check=True should never block."""
        obj = _make_afc_for_m109()
        obj.disable_ooze_check = True

        ext, heater = _make_extruder("extruder", tool_number=0)
        ext.lanes = ["lane0", "lane1"]
        ext.lane_loaded = "lane0"

        lane1 = _make_lane("lane1", "T1", ext)
        obj.lanes = {"lane0": MagicMock(), "lane1": lane1}
        obj.tools = {"extruder": ext}

        obj.function.get_lane_by_map = MagicMock(return_value=lane1)
        obj.function.get_current_extruder_obj = MagicMock(return_value=ext)

        pheaters = MagicMock()
        obj.printer.lookup_object = MagicMock(return_value=pheaters)

        gcmd = _make_gcmd(toolnum=1, temp=250.0)
        obj._cmd_AFC_M109(gcmd, wait=False)

        _assert_heater_was_set(pheaters, heater, 250.0)
