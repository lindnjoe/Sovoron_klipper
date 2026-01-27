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

        pulse_on = gcmd.get_float("PULSE_ON", 1.0)
        pulse_off = gcmd.get_float("PULSE_OFF", 1.0)
        timeout = gcmd.get_float("HUB_TIMEOUT", 10.0)
        target_clicks = gcmd.get_int("TARGET_CLICKS", None)
        target_mm = gcmd.get_float("TARGET_MM", None)

        if pulse_on <= 0.0:
            raise gcmd.error("PULSE_ON must be greater than 0")
        if pulse_off < 0.0:
            raise gcmd.error("PULSE_OFF must be zero or greater")

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

        if target_clicks is None:
            if target_mm is None:
                target_mm = (
                    float(getattr(oams_obj, "filament_path_length", 0.0)) / 2.0
                )
            target_clicks = max(0, int(round(target_mm)))

        message = "TEST_PULSE: loading lane {} (spool {}) to hub,".format(
            lane_name, spool_index
        )
        gcmd.respond_info(
            "{} pulsing follower to {} clicks".format(message, target_clicks)
        )

        follower_enabled = False
        reached_target = False
        try:
            try:
                oams_obj.oams_load_spool_cmd.send([spool_index])
            except Exception as exc:
                raise gcmd.error(f"Failed to start spool load: {exc}") from exc

            hub_timeout = self.reactor.monotonic() + timeout
            hub_detected = False
            while self.reactor.monotonic() < hub_timeout:
                self.reactor.pause(self.reactor.monotonic() + 0.1)
                try:
                    hub_values = oams_obj.hub_hes_value
                    hub_detected = bool(hub_values[spool_index])
                except Exception:
                    hub_detected = False
                if hub_detected:
                    break

            if not hub_detected:
                raise gcmd.error(
                    "Hub sensor did not trigger for lane {}".format(lane_name)
                )

            try:
                oams_obj.set_oams_follower(0, 0)
            except Exception:
                pass

            try:
                oams_obj.abort_current_action(wait=False, code=0)
            except Exception:
                pass

            try:
                encoder_before = int(oams_obj.encoder_clicks)
            except Exception:
                encoder_before = None

            if encoder_before is None:
                raise gcmd.error("Unable to read encoder clicks before pulsing")

            encoder_target = encoder_before + target_clicks
            while True:
                try:
                    oams_obj.set_oams_follower(1, 1)
                    follower_enabled = True
                except Exception as exc:
                    raise gcmd.error(
                        f"Failed to enable follower: {exc}"
                    ) from exc
                self.reactor.pause(self.reactor.monotonic() + pulse_on)

                try:
                    oams_obj.set_oams_follower(0, 0)
                except Exception:
                    pass
                follower_enabled = False
                self.reactor.pause(self.reactor.monotonic() + pulse_off)

                try:
                    encoder_now = int(oams_obj.encoder_clicks)
                except Exception:
                    encoder_now = encoder_before

                if encoder_now >= encoder_target:
                    reached_target = True
                    break
        finally:
            if follower_enabled:
                try:
                    oams_obj.set_oams_follower(0, 0)
                except Exception:
                    pass
            try:
                if reached_target:
                    gcmd.respond_info(
                        "TEST_PULSE: target reached; unloading spool"
                    )
                oams_obj.oams_unload_spool_cmd.send()
                hub_clear_timeout = self.reactor.monotonic() + timeout
                while self.reactor.monotonic() < hub_clear_timeout:
                    self.reactor.pause(self.reactor.monotonic() + 0.1)
                    try:
                        hub_values = oams_obj.hub_hes_value
                        if not bool(hub_values[spool_index]):
                            break
                    except Exception:
                        break
            except Exception:
                pass

        gcmd.respond_info(f"TEST_PULSE complete for lane {lane_name}")


def load_config_prefix(config):
    return TestPulse(config)


def load_config(config):
    return TestPulse(config)