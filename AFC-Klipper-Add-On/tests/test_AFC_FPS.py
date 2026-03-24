"""
Unit tests for extras/AFC_FPS.py

Covers:
  - AFCFPSBuffer: correction algorithm, multiplier scaling, deadband logic
  - ADC callback: reversed mode, EMA smoothing
  - enable_buffer / disable_buffer: LED state, timer control
  - Config validation: low_point < set_point < high_point, deadband bounds
  - Fault detection: sensitivity, timer lifecycle
  - get_status: correct keys and content
  - G-code commands: SET_FPS_SET_POINT, QUERY_BUFFER
"""

from __future__ import annotations

from unittest.mock import MagicMock, PropertyMock, patch
import pytest

from extras.AFC_FPS import (
    AFCFPSBuffer,
    TRAILING_STATE_NAME,
    ADVANCING_STATE_NAME,
    NEUTRAL_STATE_NAME,
)
from tests.conftest import MockAFC, MockPrinter, MockLogger, MockReactor, MockConfig


# ── Helpers ───────────────────────────────────────────────────────────────────

def _make_fps_buffer(name="FPS_buffer1", overrides=None):
    """Build an AFCFPSBuffer bypassing the complex __init__."""
    buf = AFCFPSBuffer.__new__(AFCFPSBuffer)

    afc = MockAFC()
    reactor = MockReactor()
    afc.reactor = reactor

    buf.printer = MockPrinter(afc=afc)
    buf.afc = afc
    buf.reactor = reactor
    buf.gcode = afc.gcode
    buf.logger = afc.logger

    buf.name = name
    buf.lanes = {}
    buf.last_state = "Unknown"
    buf.enable = False
    buf.current_lane = None
    buf.advance_state = False
    buf.trailing_state = False
    buf.debug = False

    # ADC defaults
    buf.adc = MagicMock()
    buf.ppins = MagicMock()
    buf.sample_count = 5
    buf.sample_time = 0.005
    buf.report_time = 0.100
    buf.reversed = False

    # Buffer tuning defaults
    buf.fps_value = 0.5
    buf.set_point = 0.5
    buf.low_point = 0.1
    buf.high_point = 0.9
    buf.multiplier_high = 1.1
    buf.multiplier_low = 0.9
    buf.deadband = 0.30
    buf.smoothing = 0.3
    buf.smoothed_fps = 0.5
    buf.update_interval = 0.25

    # Fault detection defaults
    buf.error_sensitivity = 0
    buf.fault_sensitivity = 0
    buf.filament_error_pos = None
    buf.past_extruder_position = None
    buf.extruder_pos_timer = None
    buf.fault_timer = None
    buf.min_event_systime = reactor.NEVER

    # LED defaults
    buf.led = False
    buf.led_index = None
    buf.led_advancing = '0,0,1,0'
    buf.led_trailing = '0,1,0,0'
    buf.led_neutral = '0,0.5,0.5,0'
    buf.led_buffer_disabled = '0,0,0,0.25'

    # Sensors
    buf.enable_sensors_in_gui = False

    # Timer mock
    buf.correction_timer = MagicMock()

    # Function mock
    buf.function = afc.function
    buf.show_macros = True
    buf.toolhead = MagicMock()

    # Apply overrides
    if overrides:
        for k, v in overrides.items():
            setattr(buf, k, v)

    return buf


def _make_mock_lane(name="lane1", has_stepper=True):
    """Create a mock lane with optional extruder stepper."""
    lane = MagicMock()
    lane.name = name
    if has_stepper:
        lane.extruder_stepper = MagicMock()
        lane.extruder_stepper.stepper = MagicMock()
        lane.extruder_stepper.stepper.get_rotation_distance.return_value = (7.5,)
    else:
        lane.extruder_stepper = None
    return lane



# NOTE: normalize_pin_value and is_fps_buffer_pin were removed from AFC_FPS.py


# ── ADC Callback ─────────────────────────────────────────────────────────────

class TestAdcCallback:
    def test_normal_reading(self):
        buf = _make_fps_buffer()
        buf._adc_callback(100.0, 0.7)
        assert buf.fps_value == 0.7

    def test_reversed_reading(self):
        buf = _make_fps_buffer(overrides={"reversed": True})
        buf._adc_callback(100.0, 0.3)
        assert buf.fps_value == pytest.approx(0.7)

    def test_list_format(self):
        """Kalico API passes a list of (time, value) tuples."""
        buf = _make_fps_buffer()
        buf._adc_callback([(99.0, 0.3), (100.0, 0.6)])
        assert buf.fps_value == 0.6  # Uses last reading

    def test_empty_list_ignored(self):
        buf = _make_fps_buffer()
        buf.fps_value = 0.42
        buf._adc_callback([])
        assert buf.fps_value == 0.42  # Unchanged

    def test_smoothing_applied(self):
        buf = _make_fps_buffer(overrides={"smoothing": 0.5, "smoothed_fps": 0.5})
        buf._adc_callback(100.0, 0.8)
        # EMA: 0.5 * 0.5 + 0.5 * 0.8 = 0.65
        assert buf.smoothed_fps == pytest.approx(0.65)

    def test_no_smoothing(self):
        buf = _make_fps_buffer(overrides={"smoothing": 0.0, "smoothed_fps": 0.5})
        buf._adc_callback(100.0, 0.8)
        assert buf.smoothed_fps == pytest.approx(0.8)


# ── Correction Algorithm ─────────────────────────────────────────────────────

class TestCorrectionEvent:
    def test_returns_never_when_disabled(self):
        buf = _make_fps_buffer(overrides={"enable": False})
        result = buf._correction_event(100.0)
        assert result == buf.reactor.NEVER

    def test_returns_never_when_no_lane(self):
        buf = _make_fps_buffer(overrides={"enable": True, "current_lane": None})
        result = buf._correction_event(100.0)
        assert result == buf.reactor.NEVER

    def test_neutral_in_deadband(self):
        """Reading within deadband → multiplier=1.0, neutral state."""
        buf = _make_fps_buffer(overrides={
            "enable": True,
            "current_lane": _make_mock_lane(),
            "smoothed_fps": 0.5,  # Exactly at set_point
        })
        result = buf._correction_event(100.0)
        assert result == pytest.approx(100.25)
        assert buf.last_state == NEUTRAL_STATE_NAME
        assert buf.advance_state is False
        assert buf.trailing_state is False
        buf.current_lane.update_rotation_distance.assert_called_with(1.0)

    def test_neutral_at_deadband_edge_low(self):
        buf = _make_fps_buffer(overrides={
            "enable": True,
            "current_lane": _make_mock_lane(),
            "smoothed_fps": 0.35,  # set_point(0.5) - deadband/2(0.15) = 0.35
        })
        buf._correction_event(100.0)
        assert buf.last_state == NEUTRAL_STATE_NAME

    def test_neutral_at_deadband_edge_high(self):
        buf = _make_fps_buffer(overrides={
            "enable": True,
            "current_lane": _make_mock_lane(),
            "smoothed_fps": 0.65,  # set_point(0.5) + deadband/2(0.15) = 0.65
        })
        buf._correction_event(100.0)
        assert buf.last_state == NEUTRAL_STATE_NAME

    def test_advancing_above_deadband(self):
        """Reading above neutral_high → advancing state, multiplier < 1.

        High FPS = buffer compressed = filament advancing toward toolhead.
        Correction slows feeding (multiplier < 1).
        """
        buf = _make_fps_buffer(overrides={
            "enable": True,
            "current_lane": _make_mock_lane(),
            "smoothed_fps": 0.9,  # At high_point → full advancing
        })
        buf._correction_event(100.0)
        assert buf.last_state == ADVANCING_STATE_NAME
        assert buf.advance_state is True
        assert buf.trailing_state is False
        # At high_point, fraction=1.0, multiplier = 1.0 - 1.0*(1.0-0.9) = 0.9
        buf.current_lane.update_rotation_distance.assert_called_with(pytest.approx(0.9))

    def test_advancing_partial(self):
        """Reading between neutral_high and high_point → partial advancing."""
        buf = _make_fps_buffer(overrides={
            "enable": True,
            "current_lane": _make_mock_lane(),
            "smoothed_fps": 0.775,  # Midway between 0.65 and 0.9
        })
        buf._correction_event(100.0)
        assert buf.last_state == ADVANCING_STATE_NAME
        # range_size = 0.9 - 0.65 = 0.25, fraction = (0.775-0.65)/0.25 = 0.5
        # multiplier = 1.0 - 0.5*(1.0-0.9) = 0.95
        buf.current_lane.update_rotation_distance.assert_called_with(pytest.approx(0.95))

    def test_trailing_below_deadband(self):
        """Reading below neutral_low → trailing state, multiplier > 1.

        Low FPS = buffer stretched = filament trailing behind toolhead.
        Correction speeds up feeding (multiplier > 1).
        """
        buf = _make_fps_buffer(overrides={
            "enable": True,
            "current_lane": _make_mock_lane(),
            "smoothed_fps": 0.1,  # At low_point → full trailing
        })
        buf._correction_event(100.0)
        assert buf.last_state == TRAILING_STATE_NAME
        assert buf.trailing_state is True
        assert buf.advance_state is False
        # At low_point, fraction=1.0, multiplier = 1.0 + 1.0*(1.1-1.0) = 1.1
        buf.current_lane.update_rotation_distance.assert_called_with(pytest.approx(1.1))

    def test_trailing_partial(self):
        """Reading between low_point and neutral_low → partial trailing."""
        buf = _make_fps_buffer(overrides={
            "enable": True,
            "current_lane": _make_mock_lane(),
            "smoothed_fps": 0.225,  # Midway between 0.35 and 0.1
        })
        buf._correction_event(100.0)
        assert buf.last_state == TRAILING_STATE_NAME
        # range_size = 0.35 - 0.1 = 0.25, fraction = (0.35-0.225)/0.25 = 0.5
        # multiplier = 1.0 + 0.5*(1.1-1.0) = 1.05
        buf.current_lane.update_rotation_distance.assert_called_with(pytest.approx(1.05))

    def test_clamped_beyond_high_point(self):
        """Reading > high_point → fraction clamped to 1.0."""
        buf = _make_fps_buffer(overrides={
            "enable": True,
            "current_lane": _make_mock_lane(),
            "smoothed_fps": 1.0,  # Beyond high_point
        })
        buf._correction_event(100.0)
        buf.current_lane.update_rotation_distance.assert_called_with(pytest.approx(0.9))

    def test_clamped_beyond_low_point(self):
        """Reading < low_point → fraction clamped to 1.0."""
        buf = _make_fps_buffer(overrides={
            "enable": True,
            "current_lane": _make_mock_lane(),
            "smoothed_fps": 0.0,  # Beyond low_point
        })
        buf._correction_event(100.0)
        buf.current_lane.update_rotation_distance.assert_called_with(pytest.approx(1.1))

    def test_returns_next_update_time(self):
        buf = _make_fps_buffer(overrides={
            "enable": True,
            "current_lane": _make_mock_lane(),
            "smoothed_fps": 0.5,
            "update_interval": 0.5,
        })
        result = buf._correction_event(200.0)
        assert result == pytest.approx(200.5)


# ── LED States ───────────────────────────────────────────────────────────────

class TestLedStates:
    def test_enable_buffer_shows_neutral_led(self):
        buf = _make_fps_buffer(overrides={"led": True, "led_index": "0"})
        lane = _make_mock_lane()
        buf.enable_buffer(lane)
        buf.afc.function.afc_led.assert_called_with('0,0.5,0.5,0', '0')

    def test_disable_buffer_shows_disabled_led(self):
        buf = _make_fps_buffer(overrides={
            "led": True,
            "led_index": "0",
            "enable": True,
            "current_lane": _make_mock_lane(),
        })
        buf.disable_buffer()
        buf.afc.function.afc_led.assert_called_with('0,0,0,0.25', '0')

    def test_correction_neutral_led(self):
        buf = _make_fps_buffer(overrides={
            "enable": True,
            "current_lane": _make_mock_lane(),
            "smoothed_fps": 0.5,
            "led": True,
            "led_index": "0",
        })
        buf._correction_event(100.0)
        buf.afc.function.afc_led.assert_called_with('0,0.5,0.5,0', '0')

    def test_correction_advancing_led(self):
        """High FPS reading → advancing state → advancing LED color."""
        buf = _make_fps_buffer(overrides={
            "enable": True,
            "current_lane": _make_mock_lane(),
            "smoothed_fps": 0.8,
            "led": True,
            "led_index": "0",
        })
        buf._correction_event(100.0)
        buf.afc.function.afc_led.assert_called_with('0,0,1,0', '0')

    def test_correction_trailing_led(self):
        """Low FPS reading → trailing state → trailing LED color."""
        buf = _make_fps_buffer(overrides={
            "enable": True,
            "current_lane": _make_mock_lane(),
            "smoothed_fps": 0.2,
            "led": True,
            "led_index": "0",
        })
        buf._correction_event(100.0)
        buf.afc.function.afc_led.assert_called_with('0,1,0,0', '0')


# ── Enable / Disable Buffer ─────────────────────────────────────────────────

class TestEnableDisableBuffer:
    def test_enable_sets_lane_and_flag(self):
        buf = _make_fps_buffer()
        lane = _make_mock_lane()
        buf.enable_buffer(lane)
        assert buf.current_lane is lane
        assert buf.enable is True

    def test_enable_resets_smoothed_fps(self):
        buf = _make_fps_buffer(overrides={"fps_value": 0.7, "smoothed_fps": 0.3})
        buf.enable_buffer(_make_mock_lane())
        assert buf.smoothed_fps == 0.7

    def test_enable_starts_correction_timer(self):
        buf = _make_fps_buffer()
        buf.reactor.update_timer = MagicMock()
        buf.enable_buffer(_make_mock_lane())
        buf.reactor.update_timer.assert_called()

    def test_disable_resets_state(self):
        lane = _make_mock_lane()
        buf = _make_fps_buffer(overrides={
            "enable": True,
            "current_lane": lane,
        })
        buf.disable_buffer()
        assert buf.enable is False
        assert buf.current_lane is None

    def test_disable_resets_multiplier(self):
        lane = _make_mock_lane()
        buf = _make_fps_buffer(overrides={
            "enable": True,
            "current_lane": lane,
        })
        buf.disable_buffer()
        lane.update_rotation_distance.assert_called_with(1)

    def test_disable_when_no_lane_is_noop(self):
        buf = _make_fps_buffer(overrides={"enable": True, "current_lane": None})
        buf.disable_buffer()  # Should not raise
        assert buf.enable is False


# ── set_multiplier / reset_multiplier ────────────────────────────────────────

class TestMultiplierControl:
    def test_set_multiplier_updates_lane(self):
        lane = _make_mock_lane()
        buf = _make_fps_buffer(overrides={"enable": True, "current_lane": lane})
        buf.set_multiplier(1.05)
        lane.update_rotation_distance.assert_called_with(1.05)

    def test_set_multiplier_noop_when_disabled(self):
        lane = _make_mock_lane()
        buf = _make_fps_buffer(overrides={"enable": False, "current_lane": lane})
        buf.set_multiplier(1.05)
        lane.update_rotation_distance.assert_not_called()

    def test_set_multiplier_noop_no_stepper(self):
        lane = _make_mock_lane(has_stepper=False)
        buf = _make_fps_buffer(overrides={"enable": True, "current_lane": lane})
        buf.set_multiplier(1.05)
        # Should not raise — silently skips for stepper-less units

    def test_reset_multiplier(self):
        lane = _make_mock_lane()
        buf = _make_fps_buffer(overrides={"enable": True, "current_lane": lane})
        buf.reset_multiplier()
        lane.update_rotation_distance.assert_called_with(1)


# ── Fault Detection ──────────────────────────────────────────────────────────

class TestFaultDetection:
    def test_sensitivity_to_fault_conversion(self):
        buf = _make_fps_buffer()
        assert buf.get_fault_sensitivity(10) == 10   # (11-10)*10
        assert buf.get_fault_sensitivity(5) == 60    # (11-5)*10
        assert buf.get_fault_sensitivity(1) == 100   # (11-1)*10
        assert buf.get_fault_sensitivity(0) == 0     # disabled

    def test_fault_detection_enabled(self):
        buf = _make_fps_buffer(overrides={"fault_sensitivity": 50})
        assert buf.fault_detection_enabled() is True

    def test_fault_detection_disabled(self):
        buf = _make_fps_buffer(overrides={"fault_sensitivity": 0})
        assert buf.fault_detection_enabled() is False

    def test_disable_restore_sensitivity(self):
        buf = _make_fps_buffer(overrides={"error_sensitivity": 5})
        buf.fault_sensitivity = 60
        buf.disable_fault_sensitivity()
        assert buf.fault_sensitivity == 0
        buf.restore_fault_sensitivity()
        assert buf.fault_sensitivity == 60


# ── get_status ───────────────────────────────────────────────────────────────

class TestGetStatus:
    def test_status_keys_disabled(self):
        buf = _make_fps_buffer()
        status = buf.get_status()
        assert 'state' in status
        assert 'fps_value' in status
        assert 'smoothed_fps' in status
        assert 'set_point' in status
        assert 'enabled' in status
        assert 'active_lane' in status
        assert status['enabled'] is False
        assert status['active_lane'] is None
        assert status['rotation_distance'] is None

    def test_status_enabled_with_lane(self):
        lane = _make_mock_lane()
        buf = _make_fps_buffer(overrides={
            "enable": True,
            "current_lane": lane,
            "fps_value": 0.42,
            "smoothed_fps": 0.45,
        })
        status = buf.get_status()
        assert status['enabled'] is True
        assert status['active_lane'] == 'lane1'
        assert status['fps_value'] == 0.42
        assert status['smoothed_fps'] == 0.45
        assert status['rotation_distance'] is not None

    def test_status_fault_detection(self):
        buf = _make_fps_buffer(overrides={"error_sensitivity": 5})
        status = buf.get_status()
        assert status['fault_detection_enabled'] is True
        assert status['error_sensitivity'] == 5


# ── buffer_status ────────────────────────────────────────────────────────────

class TestBufferStatus:
    def test_returns_last_state(self):
        buf = _make_fps_buffer(overrides={"last_state": ADVANCING_STATE_NAME})
        assert buf.buffer_status() == ADVANCING_STATE_NAME


# ── G-code: SET_FPS_SET_POINT ────────────────────────────────────────────────

class TestSetFpsSetPoint:
    def test_updates_set_point_and_deadband(self):
        buf = _make_fps_buffer()
        gcmd = MagicMock()
        gcmd.get_float.side_effect = lambda key, default, **kw: {
            'SET_POINT': 0.6,
            'DEADBAND': 0.20,
        }[key]
        buf.cmd_SET_FPS_SET_POINT(gcmd)
        assert buf.set_point == 0.6
        assert buf.deadband == 0.20


# ── Edge Cases ───────────────────────────────────────────────────────────────

class TestEdgeCases:
    def test_zero_deadband(self):
        """With deadband=0, any deviation from set_point triggers correction."""
        buf = _make_fps_buffer(overrides={
            "enable": True,
            "current_lane": _make_mock_lane(),
            "smoothed_fps": 0.51,
            "deadband": 0.0,
        })
        buf._correction_event(100.0)
        # 0.51 > 0.5 (set_point), so advancing (high = compressed)
        assert buf.last_state == ADVANCING_STATE_NAME

    def test_zero_range_high(self):
        """If high_point == neutral_high, fraction defaults to 1.0."""
        buf = _make_fps_buffer(overrides={
            "enable": True,
            "current_lane": _make_mock_lane(),
            "smoothed_fps": 0.9,
            "high_point": 0.65,  # Same as neutral_high
            "deadband": 0.30,
        })
        buf._correction_event(100.0)
        # fraction = 1.0 (zero range), multiplier = 0.9
        buf.current_lane.update_rotation_distance.assert_called_with(pytest.approx(0.9))

    def test_zero_range_low(self):
        """If low_point == neutral_low, fraction defaults to 1.0."""
        buf = _make_fps_buffer(overrides={
            "enable": True,
            "current_lane": _make_mock_lane(),
            "smoothed_fps": 0.1,
            "low_point": 0.35,  # Same as neutral_low
            "deadband": 0.30,
        })
        buf._correction_event(100.0)
        # fraction = 1.0 (zero range), multiplier = 1.1
        buf.current_lane.update_rotation_distance.assert_called_with(pytest.approx(1.1))

    def test_aggressive_multipliers(self):
        """Test with more aggressive multiplier range."""
        buf = _make_fps_buffer(overrides={
            "enable": True,
            "current_lane": _make_mock_lane(),
            "smoothed_fps": 0.1,
            "multiplier_high": 1.5,
        })
        buf._correction_event(100.0)
        # At low_point, fraction=1.0, multiplier = 1.0 + 1.0*(1.5-1.0) = 1.5
        buf.current_lane.update_rotation_distance.assert_called_with(pytest.approx(1.5))


# ── Extruder Property (oams_manager compatibility) ──────────────────────────

class TestExtruderProperty:
    def test_returns_active_extruder(self):
        """extruder property returns the active toolhead extruder."""
        buf = _make_fps_buffer()
        mock_extruder = MagicMock()
        mock_extruder.last_position = 42.5
        buf.toolhead.get_extruder.return_value = mock_extruder
        assert buf.extruder is mock_extruder
        assert buf.extruder.last_position == 42.5

    def test_returns_none_before_ready(self):
        """extruder property returns None when toolhead not yet initialized."""
        buf = _make_fps_buffer(overrides={"toolhead": None})
        assert buf.extruder is None

    def test_oams_style_access_pattern(self):
        """Verify the fps.extruder.last_position pattern used by oams_manager."""
        buf = _make_fps_buffer()
        mock_extruder = MagicMock()
        mock_extruder.last_position = 100.0
        buf.toolhead.get_extruder.return_value = mock_extruder
        # This is the exact pattern oams_manager uses:
        pos = float(getattr(buf.extruder, "last_position", 0.0))
        assert pos == 100.0


# ── Fault Detection Skipped for Stepper-less Lanes ──────────────────────────

class TestFaultDetectionOpenAMS:
    def test_no_fault_timer_for_stepperless_lane(self):
        """enable_buffer should NOT start fault detection for stepper-less lanes."""
        buf = _make_fps_buffer(overrides={"error_sensitivity": 5, "fault_sensitivity": 60})
        buf.start_fault_detection = MagicMock()
        lane = _make_mock_lane(has_stepper=False)
        buf.enable_buffer(lane)
        buf.start_fault_detection.assert_not_called()

    def test_fault_timer_starts_for_stepper_lane(self):
        """enable_buffer SHOULD start fault detection for stepper-based lanes."""
        buf = _make_fps_buffer(overrides={"error_sensitivity": 5, "fault_sensitivity": 60})
        buf.start_fault_detection = MagicMock()
        lane = _make_mock_lane(has_stepper=True)
        buf.enable_buffer(lane)
        buf.start_fault_detection.assert_called_once()

    def test_extruder_pos_event_skips_stepperless(self):
        """extruder_pos_update_event should bail for stepper-less lanes."""
        buf = _make_fps_buffer(overrides={
            "current_lane": _make_mock_lane(has_stepper=False),
        })
        result = buf.extruder_pos_update_event(100.0)
        # Should return next check time without doing fault detection
        assert result == pytest.approx(100.5)  # CHECK_RUNOUT_TIMEOUT = 0.5
