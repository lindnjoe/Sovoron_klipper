"""Test pulsing helper for OpenAMS TD-1 capture workflows."""

from __future__ import annotations


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
                "Test OpenAMS with short ptfe_length to see if commands work"
            ),
        )

    def cmd_TEST_PULSE(self, gcmd):
        lane_name = gcmd.get("LANE", None)
        if not lane_name:
            raise gcmd.error("LANE is required for TEST_PULSE")

        test_ptfe_length = gcmd.get_int("PTFE", 500)  # Default to 500 for testing
        hub_timeout = gcmd.get_float("HUB_TIMEOUT", 30.0)
        test_fps_target = gcmd.get_float("FPS_TARGET", 0.0)
        command_delay = gcmd.get_float("CMD_DELAY", 1.0)
        busy_timeout = gcmd.get_float("BUSY_TIMEOUT", 15.0)
        force_clear_busy = gcmd.get_int("FORCE_CLEAR_BUSY", 1)
        unload_retries = gcmd.get_int("UNLOAD_RETRIES", 1)
        unload_retry_delay = gcmd.get_float("UNLOAD_RETRY_DELAY", 3.0)

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

        # Save original ptfe_length and fps_target
        original_ptfe = getattr(oams_obj, "filament_path_length", 2087)
        original_fps_target = getattr(oams_obj, "fps_target", 0.5)
        gcmd.respond_info(f"TEST_PULSE: Original ptfe_length: {original_ptfe}")
        gcmd.respond_info(f"TEST_PULSE: Setting temporary ptfe_length: {test_ptfe_length}")
        gcmd.respond_info(f"TEST_PULSE: Original fps_target: {original_fps_target}")
        gcmd.respond_info(f"TEST_PULSE: Setting temporary fps_target: {test_fps_target}")

        def restore_settings():
            # Restore original ptfe_length
            gcmd.respond_info(
                f"TEST_PULSE: Restoring original ptfe_length: {original_ptfe}"
            )
            try:
                config_cmd = oams_obj.mcu.lookup_command("config_oams_ptfe length=%u")
                config_cmd.send([int(original_ptfe)])
                gcmd.respond_info(
                    f"TEST_PULSE: Restored ptfe_length to {original_ptfe}"
                )
            except Exception as exc:
                gcmd.respond_info(f"TEST_PULSE: Could not restore ptfe_length: {exc}")

            # Restore original fps_target
            gcmd.respond_info(
                f"TEST_PULSE: Restoring original fps_target: {original_fps_target}"
            )
            try:
                kp = oams_obj.float_to_u32(oams_obj.kp)
                ki = oams_obj.float_to_u32(oams_obj.ki)
                kd = oams_obj.float_to_u32(oams_obj.kd)
                kt = oams_obj.float_to_u32(original_fps_target)
                oams_obj.oams_pid_cmd.send([kp, ki, kd, kt])
                oams_obj.fps_target = original_fps_target
                gcmd.respond_info(
                    f"TEST_PULSE: Restored fps_target to {original_fps_target}"
                )
            except Exception as exc:
                gcmd.respond_info(f"TEST_PULSE: Could not restore fps_target: {exc}")

        def wait_for_idle(label, timeout=5.0):
            deadline = self.reactor.monotonic() + timeout
            while self.reactor.monotonic() < deadline:
                if getattr(oams_obj, "action_status", None) is None:
                    return True
                self.reactor.pause(self.reactor.monotonic() + 0.2)
            gcmd.respond_info(f"TEST_PULSE: Timed out waiting for idle after {label}")
            return False

        # Try to send config command to MCU with shorter ptfe_length
        try:
            mcu = oams_obj.mcu
            config_cmd = mcu.lookup_command("config_oams_ptfe length=%u")
            config_cmd.send([test_ptfe_length])
            gcmd.respond_info(f"TEST_PULSE: Sent config_oams_ptfe command with length={test_ptfe_length}")
            self.reactor.pause(self.reactor.monotonic() + command_delay)
        except Exception as exc:
            gcmd.respond_info(f"TEST_PULSE: Could not send config command: {exc}")
            gcmd.respond_info("TEST_PULSE: Continuing with original ptfe_length...")

        try:
            try:
                kp = oams_obj.float_to_u32(oams_obj.kp)
                ki = oams_obj.float_to_u32(oams_obj.ki)
                kd = oams_obj.float_to_u32(oams_obj.kd)
                kt = oams_obj.float_to_u32(test_fps_target)
                oams_obj.oams_pid_cmd.send([kp, ki, kd, kt])
                oams_obj.fps_target = test_fps_target
                gcmd.respond_info(
                    f"TEST_PULSE: Sent oams_pid_cmd with fps_target={test_fps_target}"
                )
                self.reactor.pause(self.reactor.monotonic() + command_delay)
            except Exception as exc:
                gcmd.respond_info(f"TEST_PULSE: Could not update fps_target: {exc}")

            # Step 1: Start loading the spool
            gcmd.respond_info(
                f"TEST_PULSE: Starting load for lane {lane_name} (spool {spool_index})"
            )

            try:
                oams_obj.oams_load_spool_cmd.send([spool_index])
                self.reactor.pause(self.reactor.monotonic() + command_delay)
            except Exception as exc:
                raise gcmd.error(f"Failed to start spool load: {exc}") from exc

            # Step 2: Wait for hub sensor to trigger
            gcmd.respond_info("TEST_PULSE: Waiting for hub sensor to trigger...")

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
                gcmd.respond_info(
                    "TEST_PULSE: Hub sensor did not trigger within timeout"
                )
                try:
                    oams_obj.abort_current_action(wait=True)
                except Exception:
                    pass
                raise gcmd.error("Hub sensor did not trigger - aborting test")

            gcmd.respond_info("TEST_PULSE: Hub sensor triggered!")
            self.reactor.pause(self.reactor.monotonic() + command_delay)

            # Step 3: Abort current action and force reverse unload
            gcmd.respond_info("TEST_PULSE: Aborting current action before unload...")
            try:
                oams_obj.abort_current_action(wait=True)
                self.reactor.pause(self.reactor.monotonic() + command_delay)
                idle = wait_for_idle("abort", timeout=busy_timeout)
                if not idle:
                    gcmd.respond_info("TEST_PULSE: Retrying abort without wait...")
                    try:
                        oams_obj.abort_current_action(wait=False)
                        self.reactor.pause(self.reactor.monotonic() + command_delay)
                    except Exception as exc:
                        gcmd.respond_info(
                            f"TEST_PULSE: Abort retry failed: {exc}"
                        )
                    idle = wait_for_idle("abort retry", timeout=busy_timeout)
                if not idle and force_clear_busy:
                    gcmd.respond_info(
                        "TEST_PULSE: Forcing clear of busy state before unload..."
                    )
                    oams_obj.action_status = None
            except Exception as exc:
                gcmd.respond_info(f"TEST_PULSE: Abort current action failed: {exc}")

            gcmd.respond_info("TEST_PULSE: Enabling follower reverse...")
            try:
                oams_obj.set_oams_follower(1, 0)
                self.reactor.pause(self.reactor.monotonic() + command_delay)
            except Exception as exc:
                gcmd.respond_info(
                    f"TEST_PULSE: Warning - follower reverse failed: {exc}"
                )

            gcmd.respond_info("TEST_PULSE: Sending unload command...")
            unload_success = False
            unload_message = "Unload not attempted"
            try:
                wait_for_idle("follower reverse", timeout=busy_timeout)
                for attempt in range(unload_retries):
                    if attempt > 0:
                        gcmd.respond_info(
                            f"TEST_PULSE: Unload retry {attempt + 1}/{unload_retries}"
                        )
                        self.reactor.pause(
                            self.reactor.monotonic() + unload_retry_delay
                        )
                    unload_success, unload_message = oams_obj.unload_spool()
                    if unload_success:
                        break
                gcmd.respond_info(f"TEST_PULSE: Unload result: {unload_message}")
                if not unload_success:
                    gcmd.respond_info("TEST_PULSE: Unload command reported failure.")
                self.reactor.pause(self.reactor.monotonic() + command_delay)
            except Exception as exc:
                gcmd.respond_info(f"TEST_PULSE: Unload command failed: {exc}")

            gcmd.respond_info("TEST_PULSE: Disabling follower after unload...")
            try:
                oams_obj.set_oams_follower(0, 0)
                self.reactor.pause(self.reactor.monotonic() + command_delay)
            except Exception as exc:
                gcmd.respond_info(
                    f"TEST_PULSE: Warning - disable follower failed: {exc}"
                )
        finally:
            restore_settings()


def load_config_prefix(config):
    return TestPulse(config)


def load_config(config):
    return TestPulse(config)