"""Test pulsing helper for OpenAMS TD-1 capture workflows."""

from __future__ import annotations


class TestPulse:
    """Provides a TEST_PULSE gcode command for OpenAMS pulsing tests."""

    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")
        self.reactor = self.printer.get_reactor()
        self.gcode.register_command(
            "TEST_PULSE",
            self.cmd_TEST_PULSE,
            desc=(
                "Test OpenAMS hub load and follower pulsing without TD-1 data "
                "capture"
            ),
        )

    def cmd_TEST_PULSE(self, gcmd):
        lane_name = gcmd.get("LANE", None)
        if not lane_name:
            raise gcmd.error("LANE is required for TEST_PULSE")

        load_timeout = gcmd.get_float("LOAD_TIMEOUT", 30.0)
        unload_timeout = gcmd.get_float("UNLOAD_TIMEOUT", 60.0)
        fake_fps = gcmd.get_float("FAKE_FPS", None)
        fake_ptfe = gcmd.get_float("FAKE_PTFE", 100.0)

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

        original_fps = None
        original_ptfe = None
        try:
            if fake_fps is not None:
                original_fps = getattr(oams_obj, "fps_value", None)
                oams_obj.fps_value = float(fake_fps)
            if fake_ptfe is not None:
                original_ptfe = getattr(oams_obj, "filament_path_length", None)
                oams_obj.filament_path_length = float(fake_ptfe)

            gcmd.respond_info(
                "TEST_PULSE: loading lane {} (spool {}) to extruder".format(
                    lane_name, spool_index
                )
            )

            try:
                oams_obj.oams_load_spool_cmd.send([spool_index])
            except Exception as exc:
                raise gcmd.error(f"Failed to start spool load: {exc}") from exc

            if hasattr(oams_obj, "action_status"):
                oams_obj.action_status = 0

            load_deadline = self.reactor.monotonic() + load_timeout
            while self.reactor.monotonic() < load_deadline:
                self.reactor.pause(self.reactor.monotonic() + 0.1)
                if getattr(oams_obj, "action_status", None) is None:
                    break
            else:
                raise gcmd.error(
                    "Load did not complete before timeout for lane {}".format(
                        lane_name
                    )
                )

            gcmd.respond_info(
                "TEST_PULSE: load complete; unloading lane {}".format(lane_name)
            )

            try:
                oams_obj.oams_unload_spool_cmd.send()
            except Exception as exc:
                raise gcmd.error(f"Failed to start unload: {exc}") from exc

            if hasattr(oams_obj, "action_status"):
                oams_obj.action_status = 1

            unload_deadline = self.reactor.monotonic() + unload_timeout
            while self.reactor.monotonic() < unload_deadline:
                self.reactor.pause(self.reactor.monotonic() + 0.1)
                if getattr(oams_obj, "action_status", None) is None:
                    break
            else:
                raise gcmd.error(
                    "Unload did not complete before timeout for lane {}".format(
                        lane_name
                    )
                )
        finally:
            if original_fps is not None:
                oams_obj.fps_value = original_fps
            if original_ptfe is not None:
                oams_obj.filament_path_length = original_ptfe

        gcmd.respond_info(f"TEST_PULSE complete for lane {lane_name}")


def load_config_prefix(config):
    return TestPulse(config)


def load_config(config):
    return TestPulse(config)