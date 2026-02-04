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
                "Test OpenAMS hub load, TD-1 data read, and immediate unload - "
                "stops after sending unload to test if filament retracts"
            ),
        )

    def cmd_TEST_PULSE(self, gcmd):
        lane_name = gcmd.get("LANE", None)
        if not lane_name:
            raise gcmd.error("LANE is required for TEST_PULSE")

        hub_timeout = gcmd.get_float("HUB_TIMEOUT", 30.0)
        td1_timeout = gcmd.get_float("TD1_TIMEOUT", 10.0)

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

        td1_device_id = getattr(lane, "td1_device_id", None) or getattr(unit_obj, "td1_device_id", None)
        if not td1_device_id:
            gcmd.respond_info(f"WARNING: No TD-1 device ID configured for {lane_name}")

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
            # Abort and return
            try:
                oams_obj.abort_current_action(wait=True, code=0)
            except Exception:
                pass
            raise gcmd.error("Hub sensor did not trigger - aborting test")

        gcmd.respond_info(f"TEST_PULSE: Hub sensor triggered!")

        # Step 3: Abort the load action to take manual control
        gcmd.respond_info(f"TEST_PULSE: Aborting load to take manual control...")
        try:
            oams_obj.abort_current_action(wait=True, code=0)
        except Exception as exc:
            gcmd.respond_info(f"TEST_PULSE: Warning - abort failed: {exc}")

        # Step 4: Read TD-1 data
        gcmd.respond_info(f"TEST_PULSE: Reading TD-1 data...")

        td1_data = None
        if td1_device_id and hasattr(afc, "moonraker"):
            td1_deadline = self.reactor.monotonic() + td1_timeout
            while self.reactor.monotonic() < td1_deadline:
                try:
                    data = afc.moonraker.get_td1_data()
                    if data is not None and td1_device_id in data:
                        device_data = data[td1_device_id]
                        if "error" not in device_data or device_data.get("error") is None:
                            td1_data = device_data
                            break
                except Exception as exc:
                    gcmd.respond_info(f"TEST_PULSE: TD-1 read error: {exc}")
                self.reactor.pause(self.reactor.monotonic() + 0.2)

        if td1_data:
            color = td1_data.get("color", "unknown")
            td_value = td1_data.get("td", "unknown")
            gcmd.respond_info(f"TEST_PULSE: TD-1 data - color={color}, td={td_value}")
        else:
            gcmd.respond_info(f"TEST_PULSE: Could not read TD-1 data")

        # Step 5: Immediately send unload command to reverse AMS motor
        gcmd.respond_info(f"TEST_PULSE: Sending unload command to reverse AMS...")
        try:
            oams_obj.oams_unload_spool_cmd.send([])
        except Exception as exc:
            gcmd.respond_info(f"TEST_PULSE: Warning - unload command failed: {exc}")

        # Step 6: Enable follower in reverse to help retract
        gcmd.respond_info(f"TEST_PULSE: Enabling follower reverse...")
        try:
            oams_obj.set_oams_follower(1, 0)  # enable=1, direction=0 (reverse)
        except Exception as exc:
            gcmd.respond_info(f"TEST_PULSE: Warning - follower reverse failed: {exc}")

        gcmd.respond_info(
            f"TEST_PULSE: STOPPED - Unload sent, follower reversed. "
            f"Check if filament retracts or continues forward."
        )


def load_config_prefix(config):
    return TestPulse(config)


def load_config(config):
    return TestPulse(config)