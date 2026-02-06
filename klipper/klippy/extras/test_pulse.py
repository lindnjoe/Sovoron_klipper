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
                "Load until hub detects filament, read TD-1 data, then unload"
            ),
        )

    def cmd_TEST_PULSE(self, gcmd):
        lane_name = gcmd.get("LANE", None)
        if not lane_name:
            raise gcmd.error("LANE is required for TEST_PULSE")

        hub_timeout = gcmd.get_float("HUB_TIMEOUT", 30.0)
        td1_timeout = gcmd.get_float("TD1_TIMEOUT", 30.0)
        load_timeout = gcmd.get_float("LOAD_TIMEOUT", 30.0)
        command_delay = gcmd.get_float("CMD_DELAY", 0.5)

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

        td1_bowden_length = getattr(lane, "td1_bowden_length", None)
        if td1_bowden_length is None:
            td1_bowden_length = getattr(unit_obj, "td1_bowden_length", None)
        if td1_bowden_length is None:
            td1_bowden_length = gcmd.get_int("TD1_BOWDEN_LENGTH", 0)
        try:
            td1_bowden_length = int(td1_bowden_length)
        except Exception as exc:
            raise gcmd.error(f"Invalid td1_bowden_length for {lane_name}: {exc}") from exc
        if td1_bowden_length < 0:
            raise gcmd.error("TD1 bowden length must be >= 0")

        gcmd.respond_info(
            f"TEST_PULSE: Start load for lane {lane_name} (spool {spool_index})"
        )
        try:
            oams_obj.oams_load_spool_cmd.send([spool_index])
        except Exception as exc:
            raise gcmd.error(f"Failed to start spool load: {exc}") from exc

        hub_triggered = False
        hub_deadline = self.reactor.monotonic() + hub_timeout
        while self.reactor.monotonic() < hub_deadline:
            hub_values = getattr(oams_obj, "hub_hes_value", None)
            if hub_values and spool_index < len(hub_values):
                if bool(hub_values[spool_index]):
                    hub_triggered = True
                    break
            self.reactor.pause(self.reactor.monotonic() + 0.1)

        if not hub_triggered:
            try:
                oams_obj.abort_current_action(wait=True)
            except Exception:
                pass
            raise gcmd.error("Hub sensor did not trigger - aborting test")

        gcmd.respond_info("TEST_PULSE: Hub sensor triggered")

        try:
            encoder_start = int(getattr(oams_obj, "encoder_clicks", 0))
        except Exception:
            encoder_start = 0
        gcmd.respond_info(
            f"TEST_PULSE: Tracking encoder from {encoder_start} toward td1_bowden_length={td1_bowden_length}"
        )

        td1_deadline = self.reactor.monotonic() + td1_timeout
        while self.reactor.monotonic() < td1_deadline:
            try:
                encoder_now = int(getattr(oams_obj, "encoder_clicks", encoder_start))
            except Exception:
                encoder_now = encoder_start
            encoder_delta = abs(encoder_now - encoder_start)
            if encoder_delta >= td1_bowden_length:
                break
            self.reactor.pause(self.reactor.monotonic() + 0.1)
        else:
            gcmd.respond_info(
                "TEST_PULSE: Timed out waiting for encoder to reach td1_bowden_length"
            )

        td1_data = None
        try:
            moonraker = getattr(afc, "moonraker", None)
            if moonraker is not None and hasattr(moonraker, "get_td1_data"):
                td1_data = moonraker.get_td1_data()
        except Exception as exc:
            gcmd.respond_info(f"TEST_PULSE: TD-1 read failed: {exc}")

        if td1_data:
            td1_device_id = getattr(lane, "td1_device_id", None)
            if td1_device_id and td1_device_id in td1_data:
                gcmd.respond_info(
                    f"TEST_PULSE: TD-1 data for {td1_device_id}: {td1_data[td1_device_id]}"
                )
            else:
                gcmd.respond_info(
                    f"TEST_PULSE: TD-1 data read ({len(td1_data)} devices)"
                )
        else:
            gcmd.respond_info("TEST_PULSE: No TD-1 data available")

        gcmd.respond_info("TEST_PULSE: Waiting for load to register before unload")
        loaded_deadline = self.reactor.monotonic() + load_timeout
        last_log = 0.0
        while self.reactor.monotonic() < loaded_deadline:
            current_spool = getattr(oams_obj, "current_spool", None)
            action_status = getattr(oams_obj, "action_status", None)
            action_status_code = getattr(oams_obj, "action_status_code", None)

            queried_spool = None
            try:
                if hasattr(oams_obj, "determine_current_spool"):
                    queried_spool = oams_obj.determine_current_spool()
            except Exception:
                queried_spool = None

            load_registered = (
                action_status is None
                and (current_spool == spool_index or queried_spool == spool_index)
            )
            if load_registered:
                if current_spool != spool_index:
                    oams_obj.current_spool = spool_index
                gcmd.respond_info(
                    "TEST_PULSE: Load registered "
                    f"(current_spool={current_spool}, queried_spool={queried_spool}, "
                    f"action_status_code={action_status_code})"
                )
                break

            now = self.reactor.monotonic()
            if now - last_log >= 1.0:
                gcmd.respond_info(
                    "TEST_PULSE: Waiting load register "
                    f"(current_spool={current_spool}, queried_spool={queried_spool}, "
                    f"action_status={action_status}, action_status_code={action_status_code})"
                )
                last_log = now
            self.reactor.pause(now + 0.1)
        else:
            raise gcmd.error(
                "Load did not register before timeout; refusing unload while OAMS may still be busy"
            )

        self.reactor.pause(self.reactor.monotonic() + command_delay)

        gcmd.respond_info("TEST_PULSE: Unloading spool")
        try:
            unload_success, unload_message = oams_obj.unload_spool()
            gcmd.respond_info(f"TEST_PULSE: Unload result: {unload_message}")
            if not unload_success:
                raise gcmd.error(f"Unload failed: {unload_message}")
        except Exception as exc:
            raise gcmd.error(f"Unload command failed: {exc}") from exc


def load_config_prefix(config):
    return TestPulse(config)


def load_config(config):
    return TestPulse(config)