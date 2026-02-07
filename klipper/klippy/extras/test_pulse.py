"""Test pulsing helper for OpenAMS TD-1 capture workflows."""

from __future__ import annotations
import struct


def float_to_u32(value):
    """Convert float to u32 for MCU commands."""
    return struct.unpack('I', struct.pack('f', float(value)))[0]


class TestPulse:
    """Provides a TEST_PULSE gcode command for OpenAMS TD-1 capture testing."""

    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")
        self.reactor = self.printer.get_reactor()
        self.gcode.register_command(
            "TEST_PULSE",
            self.cmd_TEST_PULSE,
            desc=(
                "Test OpenAMS with short ptfe_length and low fps_target"
            ),
        )

    def cmd_TEST_PULSE(self, gcmd):
        lane_name = gcmd.get("LANE", None)
        if not lane_name:
            raise gcmd.error("LANE is required for TEST_PULSE")

        test_ptfe_length = gcmd.get_int("PTFE", 500)  # Default to 500 for testing
        test_fps_target = gcmd.get_float("FPS_TARGET", 0.001)  # Very low target
        hub_timeout = gcmd.get_float("HUB_TIMEOUT", 30.0)

        afc = self.printer.lookup_object("AFC", None)
        if afc is None or not hasattr(afc, "lanes"):
            raise gcmd.error("AFC not available for TEST_PULSE")

        lane = afc.lanes.get(lane_name)
        if lane is None:
            raise gcmd.error(f"Lane {lane_name} not found")

        unit_obj = getattr(lane, "unit_obj", None)
        if unit_obj is None or getattr(unit_obj, "type", None) != "OpenAMS":
            raise gcmd.error(f"Lane {lane_name} is not an OpenAMS lane")

        oams_manager = self.printer.lookup_object("oams_manager", None)
        if oams_manager is None:
            raise gcmd.error("oams_manager not available for TEST_PULSE")

        oams_name = getattr(unit_obj, "oams_name", None)
        if not oams_name:
            raise gcmd.error(f"OpenAMS unit {unit_obj.name} has no oams_name")

        oams_obj = oams_manager.oams.get(oams_name) or oams_manager.oams.get(
            f"oams {oams_name}"
        )
        if oams_obj is None:
            raise gcmd.error(f"OAMS {oams_name} not found")

        spool_index = getattr(lane, "index", None)
        if spool_index is None:
            raise gcmd.error(f"Lane {lane_name} has no index configured")
        spool_index -= 1
        if spool_index < 0:
            raise gcmd.error(f"Lane {lane_name} has invalid spool index")

        # Save original values
        original_ptfe = getattr(oams_obj, "filament_path_length", 2087)
        original_kp = getattr(oams_obj, "kp", 6.0)
        original_ki = getattr(oams_obj, "ki", 0.0)
        original_kd = getattr(oams_obj, "kd", 0.0)
        original_fps_target = getattr(oams_obj, "fps_target", 0.5)

        gcmd.respond_info(f"TEST_PULSE: Original ptfe_length: {original_ptfe}")
        gcmd.respond_info(f"TEST_PULSE: Original fps_target: {original_fps_target}")
        gcmd.respond_info(f"TEST_PULSE: Setting temporary ptfe_length: {test_ptfe_length}")
        gcmd.respond_info(f"TEST_PULSE: Setting temporary fps_target: {test_fps_target}")

        # Try to send config command to MCU with shorter ptfe_length
        try:
            mcu = oams_obj.mcu
            config_cmd = mcu.lookup_command("config_oams_ptfe length=%u")
            config_cmd.send([test_ptfe_length])
            gcmd.respond_info(f"TEST_PULSE: Sent config_oams_ptfe with length={test_ptfe_length}")
        except Exception as exc:
            gcmd.respond_info(f"TEST_PULSE: Could not send ptfe config: {exc}")

        # Set low fps_target using the PID command
        try:
            kp_u32 = float_to_u32(original_kp)
            ki_u32 = float_to_u32(original_ki)
            kd_u32 = float_to_u32(original_kd)
            target_u32 = float_to_u32(test_fps_target)
            oams_obj.oams_pid_cmd.send([kp_u32, ki_u32, kd_u32, target_u32])
            gcmd.respond_info(f"TEST_PULSE: Sent PID command with fps_target={test_fps_target}")
        except Exception as exc:
            gcmd.respond_info(f"TEST_PULSE: Could not send PID command: {exc}")

        # Step 1: Start loading the spool
        gcmd.respond_info(
            f"TEST_PULSE: Starting load for lane {lane_name} (spool {spool_index})"
        )

        try:
            oams_obj.oams_load_spool_cmd.send([spool_index])
        except Exception as exc:
            raise gcmd.error(f"Failed to start spool load: {exc}") from exc

        # Step 2: Wait for hub sensor to trigger
        gcmd.respond_info(f"TEST_PULSE: Waiting for hub sensor to trigger...")

        hub_triggered = False
        hub_deadline = self.reactor.monotonic() + hub_timeout
        while self.reactor.monotonic() < hub_deadline:
            try:
                hub_values = getattr(oams_obj, "hub_hes_value", None)
                if hub_values and spool_index < len(hub_values):
                    if bool(hub_values[spool_index]):
                        hub_triggered = True
                        break
            except Exception:
                pass
            self.reactor.pause(self.reactor.monotonic() + 0.1)

        if not hub_triggered:
            gcmd.respond_info(f"TEST_PULSE: Hub sensor did not trigger within timeout")
            self._restore_settings(gcmd, oams_obj, original_ptfe, original_kp, original_ki, original_kd, original_fps_target)
            raise gcmd.error("Hub sensor did not trigger - aborting test")

        gcmd.respond_info(f"TEST_PULSE: Hub sensor triggered!")

        # Wait a moment to see if it stops on its own (fps_target reached)
        gcmd.respond_info(f"TEST_PULSE: Waiting to see if AMS stops at fps_target...")
        self.reactor.pause(self.reactor.monotonic() + 3.0)

        # Step 3: Disable follower
        gcmd.respond_info(f"TEST_PULSE: Disabling follower...")
        try:
            oams_obj.set_oams_follower(0, 0)
        except Exception as exc:
            gcmd.respond_info(f"TEST_PULSE: Warning - disable follower failed: {exc}")

        # Step 4: Send unload command
        gcmd.respond_info(f"TEST_PULSE: Sending unload command...")
        try:
            oams_obj.oams_unload_spool_cmd.send([])
        except Exception as exc:
            gcmd.respond_info(f"TEST_PULSE: Warning - unload command failed: {exc}")

        # Step 5: Enable follower in reverse
        gcmd.respond_info(f"TEST_PULSE: Enabling follower reverse...")
        try:
            oams_obj.set_oams_follower(1, 0)
        except Exception as exc:
            gcmd.respond_info(f"TEST_PULSE: Warning - follower reverse failed: {exc}")

        # Wait for retraction
        self.reactor.pause(self.reactor.monotonic() + 2.0)

        # Step 6: Disable follower
        try:
            oams_obj.set_oams_follower(0, 0)
        except Exception:
            pass

        # Restore original settings
        self._restore_settings(gcmd, oams_obj, original_ptfe, original_kp, original_ki, original_kd, original_fps_target)

        gcmd.respond_info(
            f"TEST_PULSE: DONE - Check if filament stopped after ptfe_length or continued."
        )

    def _restore_settings(self, gcmd, oams_obj, original_ptfe, original_kp, original_ki, original_kd, original_fps_target):
        """Restore original OAMS settings."""
        gcmd.respond_info(f"TEST_PULSE: Restoring original settings...")

        # Restore ptfe_length
        try:
            config_cmd = oams_obj.mcu.lookup_command("config_oams_ptfe length=%u")
            config_cmd.send([int(original_ptfe)])
            gcmd.respond_info(f"TEST_PULSE: Restored ptfe_length to {original_ptfe}")
        except Exception as exc:
            gcmd.respond_info(f"TEST_PULSE: Could not restore ptfe_length: {exc}")

        # Restore PID settings
        try:
            kp_u32 = float_to_u32(original_kp)
            ki_u32 = float_to_u32(original_ki)
            kd_u32 = float_to_u32(original_kd)
            target_u32 = float_to_u32(original_fps_target)
            oams_obj.oams_pid_cmd.send([kp_u32, ki_u32, kd_u32, target_u32])
            gcmd.respond_info(f"TEST_PULSE: Restored fps_target to {original_fps_target}")
        except Exception as exc:
            gcmd.respond_info(f"TEST_PULSE: Could not restore PID settings: {exc}")


def load_config_prefix(config):
    return TestPulse(config)


def load_config(config):
    return TestPulse(config)