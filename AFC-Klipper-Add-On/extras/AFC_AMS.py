"""AMS integration helpers for Armored Turtle AFC."""

from __future__ import annotations

import traceback
from textwrap import dedent
from typing import Dict, Optional

from configparser import Error as ConfigError

try:  # pragma: no cover - defensive guard for runtime import errors
    from extras.AFC_unit import afcUnit
except Exception as exc:  # pragma: no cover - defensive guard
    raise ConfigError(
        "Error when trying to import AFC_unit\n{trace}".format(
            trace=traceback.format_exc()
        )
    ) from exc

try:  # pragma: no cover - defensive guard for runtime import errors
    from extras.AFC_lane import AFCLaneState
except Exception as exc:  # pragma: no cover - defensive guard
    raise ConfigError(
        "Error when trying to import AFC_lane\n{trace}".format(
            trace=traceback.format_exc()
        )
    ) from exc


SYNC_INTERVAL = 2.0


class afcAMS(afcUnit):
    """AFC unit subclass that synchronises state with OpenAMS."""

    def __init__(self, config):
        super().__init__(config)
        self.type = "AMS"

        # OpenAMS specific options
        self.oams_name = config.get("oams", "oams1")
        self.interval = config.getfloat("interval", SYNC_INTERVAL, above=0.0)

        self.reactor = self.printer.get_reactor()
        self.timer = self.reactor.register_timer(self._sync_event)
        self.printer.register_event_handler("klippy:ready", self.handle_ready)

        # Track previous sensor state to only forward changes
        self._last_lane_states: Dict[str, bool] = {}
        self._last_hub_states: Dict[str, bool] = {}
        self._virtual_tool_sensor = None
        self._last_virtual_tool_state: Optional[bool] = None
        self.oams = None

        self.gcode.register_mux_command(
            "AFC_AMS_SYNC_TOOL_SENSOR",
            "UNIT",
            self.name,
            self.cmd_SYNC_TOOL_SENSOR,
            desc=self.cmd_SYNC_TOOL_SENSOR_help,
        )

    def handle_connect(self):
        """Initialise the AMS unit and configure custom logos."""
        super().handle_connect()

        self._ensure_virtual_tool_sensor()

        # AMS lanes report their state via OpenAMS so default them until the
        # first poll comes back.
        for lane in self.lanes.values():
            lane.prep_state = False
            lane.load_state = False
            lane.status = AFCLaneState.NONE
            lane.ams_share_prep_load = getattr(lane, "load", None) is None

        first_leg = (
            "<span class=warning--text>|</span>"
            "<span class=error--text>_</span>"
        )
        second_leg = f"{first_leg}<span class=warning--text>|</span>"
        self.logo = dedent(
            """\
            <span class=success--text>R  _____     ____
            E /      \\  |  </span><span class=info--text>o</span><span class=success--text> |
            A |       |/ ___/
            D |_________/
            Y {first}{second} {first}{second}
              {name}
            </span>
            """
        ).format(first=first_leg, second=second_leg, name=self.name)

        self.logo_error = dedent(
            """\
            <span class=error--text>E  _ _   _ _
            R |_|_|_|_|_|
            R |         \\____
            O |              \\
            R |          |\\ <span class=secondary--text>X</span> |
            ! \\_________/ |___|
              {name}
            </span>
            """
        ).format(name=self.name)

    def _ensure_virtual_tool_sensor(self) -> bool:
        """Resolve the virtual tool-start sensor for AMS-tracked extruders."""

        if self._virtual_tool_sensor is not None:
            return True

        extruder = getattr(self, "extruder_obj", None)
        if extruder is None:
            return False

        tool_pin = getattr(extruder, "tool_start", None)
        if not isinstance(tool_pin, str):
            return False

        normalized = tool_pin.strip()
        if not normalized or normalized.lower() == "buffer":
            return False

        while normalized and normalized[0] in "!^":
            normalized = normalized[1:]

        if not normalized.upper().startswith("AMS_"):
            return False

        sensor = getattr(extruder, "fila_tool_start", None)
        if sensor is None:
            return False

        helper = getattr(sensor, "runout_helper", None)
        if helper is None:
            return False

        filament_present = getattr(helper, "filament_present", None)
        if filament_present is not None:
            self._last_virtual_tool_state = bool(filament_present)

        self._virtual_tool_sensor = sensor
        return True

    def _lane_matches_extruder(self, lane) -> bool:
        """Return True if the lane is mapped to this AMS unit's extruder."""

        extruder_name = getattr(self, "extruder", None)
        if not extruder_name:
            return False

        lane_extruder = getattr(lane, "extruder_name", None)
        if lane_extruder is None:
            extruder_obj = getattr(lane, "extruder_obj", None)
            lane_extruder = getattr(extruder_obj, "name", None)

        return lane_extruder == extruder_name

    def _set_virtual_tool_sensor_state(
        self, filament_present: bool, eventtime: float
    ) -> None:
        """Update the cached virtual sensor and extruder state."""

        if not self._ensure_virtual_tool_sensor():
            return

        sensor = self._virtual_tool_sensor
        helper = getattr(sensor, "runout_helper", None)
        if helper is None:
            return

        try:
            helper.note_filament_present(eventtime, filament_present)
        except TypeError:
            helper.note_filament_present(is_filament_present=filament_present)

        setattr(sensor, "filament_present", filament_present)

        extruder = getattr(self, "extruder_obj", None)
        if extruder is not None:
            extruder.tool_start_state = filament_present

        self._last_virtual_tool_state = filament_present

    def _mirror_lane_to_virtual_sensor(self, lane, eventtime: float) -> None:
        """Mirror a lane's load state into the AMS virtual tool sensor."""

        if not self._lane_matches_extruder(lane):
            return

        desired_state = bool(getattr(lane, "load_state", False))
        if desired_state == self._last_virtual_tool_state:
            return

        self._set_virtual_tool_sensor_state(desired_state, eventtime)

    def _sync_virtual_tool_sensor(
        self, eventtime: float, lane_name: Optional[str] = None
    ) -> None:
        """Align the AMS virtual tool sensor with the mapped lane state."""

        if not self._ensure_virtual_tool_sensor():
            return

        desired_state: Optional[bool] = None

        if lane_name:
            lane = self.lanes.get(lane_name)
            if lane is not None and self._lane_matches_extruder(lane):
                desired_state = bool(getattr(lane, "load_state", False))

        if desired_state is None:
            for lane in self.lanes.values():
                if self._lane_matches_extruder(lane):
                    desired_state = bool(getattr(lane, "load_state", False))
                    break

        if desired_state is None or desired_state == self._last_virtual_tool_state:
            return

        self._set_virtual_tool_sensor_state(desired_state, eventtime)

    cmd_SYNC_TOOL_SENSOR_help = (
        "Synchronise the AMS virtual tool-start sensor with the assigned lane."
    )

    def cmd_SYNC_TOOL_SENSOR(self, gcmd):
        lane_name = gcmd.get("LANE", None)
        eventtime = self.reactor.monotonic()
        self._sync_virtual_tool_sensor(eventtime, lane_name)

    def system_Test(self, cur_lane, delay, assignTcmd, enable_movement):
        """Validate AMS lane state without attempting any motion."""

        msg = ""
        succeeded = True

        cur_lane.unsync_to_extruder(False)
        self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.7)

        if not cur_lane.prep_state:
            if not cur_lane.load_state:
                self.afc.function.afc_led(cur_lane.led_not_ready, cur_lane.led_index)
                msg += '<span class=success--text>EMPTY READY FOR SPOOL</span>'
            else:
                self.afc.function.afc_led(cur_lane.led_fault, cur_lane.led_index)
                msg += '<span class=error--text> NOT READY</span>'
                cur_lane.do_enable(False)
                msg = '<span class=error--text>CHECK FILAMENT Prep: False - Load: True</span>'
                succeeded = False
        else:
            self.afc.function.afc_led(cur_lane.led_ready, cur_lane.led_index)
            msg += '<span class=success--text>LOCKED</span>'
            if not cur_lane.load_state:
                msg += '<span class=error--text> NOT LOADED</span>'
                self.afc.function.afc_led(cur_lane.led_not_ready, cur_lane.led_index)
                succeeded = False
            else:
                cur_lane.status = AFCLaneState.LOADED
                msg += '<span class=success--text> AND LOADED</span>'
                self.afc.function.afc_led(cur_lane.led_spool_illum, cur_lane.led_spool_index)

                if cur_lane.tool_loaded:
                    tool_ready = (
                        cur_lane.get_toolhead_pre_sensor_state()
                        or cur_lane.extruder_obj.tool_start == "buffer"
                        or cur_lane.extruder_obj.tool_end_state
                    )
                    if tool_ready and cur_lane.extruder_obj.lane_loaded == cur_lane.name:
                        cur_lane.sync_to_extruder()
                        msg += '<span class=primary--text> in ToolHead</span>'
                        if cur_lane.extruder_obj.tool_start == "buffer":
                            msg += (
                                '<span class=warning--text>'
                                ' Ram sensor enabled, confirm tool is loaded</span>'
                            )
                        if self.afc.function.get_current_lane() == cur_lane.name:
                            self.afc.spool.set_active_spool(cur_lane.spool_id)
                            cur_lane.unit_obj.lane_tool_loaded(cur_lane)
                            cur_lane.status = AFCLaneState.TOOLED
                        cur_lane.enable_buffer()
                    elif tool_ready:
                        msg += (
                            '<span class=error--text> error in ToolHead. '
                            'Lane identified as loaded '
                            'but not identified as loaded in extruder</span>'
                        )
                        succeeded = False

        if assignTcmd:
            self.afc.function.TcmdAssign(cur_lane)
        cur_lane.do_enable(False)
        self.logger.info(
            '{lane_name} tool cmd: {tcmd:3} {msg}'.format(
                lane_name=cur_lane.name, tcmd=cur_lane.map, msg=msg
            )
        )
        cur_lane.set_afc_prep_done()
        return succeeded

    def handle_ready(self):
        """Resolve the OpenAMS object once Klippy is ready."""

        self.oams = self.printer.lookup_object("oams " + self.oams_name, None)
        self.reactor.update_timer(self.timer, self.reactor.NOW)

    def _update_shared_lane(self, lane, lane_val, eventtime):
        """Synchronise shared prep/load sensor lanes without triggering errors."""

        if lane_val == self._last_lane_states.get(lane.name):
            return

        if lane_val:
            lane.load_state = False
            try:
                lane.prep_callback(eventtime, True)
            finally:
                lane.load_callback(eventtime, True)

            self._mirror_lane_to_virtual_sensor(lane, eventtime)

            if (
                lane.prep_state
                and lane.load_state
                and lane.printer.state_message == "Printer is ready"
                and getattr(lane, "_afc_prep_done", False)
            ):
                lane.status = AFCLaneState.LOADED
                lane.unit_obj.lane_loaded(lane)
                lane.afc.spool._set_values(lane)
                lane._prep_capture_td1()
        else:
            lane.load_callback(eventtime, False)
            lane.prep_callback(eventtime, False)

            self._mirror_lane_to_virtual_sensor(lane, eventtime)

            lane.tool_loaded = False
            lane.loaded_to_hub = False
            lane.status = AFCLaneState.NONE
            lane.unit_obj.lane_unloaded(lane)
            lane.td1_data = {}
            lane.afc.spool.clear_values(lane)

        lane.afc.save_vars()
        self._last_lane_states[lane.name] = lane_val

    def _sync_event(self, eventtime):
        """Poll OpenAMS for state updates and propagate to lanes/hubs."""

        try:
            if self.oams is None:
                return eventtime + self.interval

            lane_values = getattr(self.oams, "f1s_hes_value", None)
            hub_values = getattr(self.oams, "hub_hes_value", None)

            for lane in list(self.lanes.values()):
                idx = getattr(lane, "index", 0) - 1
                if idx < 0:
                    continue

                if lane_values is None or idx >= len(lane_values):
                    continue

                lane_val = bool(lane_values[idx])
                if getattr(lane, "ams_share_prep_load", False):
                    self._update_shared_lane(lane, lane_val, eventtime)
                elif lane_val != self._last_lane_states.get(lane.name):
                    lane.load_callback(eventtime, lane_val)
                    lane.prep_callback(eventtime, lane_val)
                    self._mirror_lane_to_virtual_sensor(lane, eventtime)
                    self._last_lane_states[lane.name] = lane_val

                hub = getattr(lane, "hub_obj", None)
                if hub is None or hub_values is None or idx >= len(hub_values):
                    continue

                hub_val = bool(hub_values[idx])
                if hub_val != self._last_hub_states.get(hub.name):
                    hub.switch_pin_callback(eventtime, hub_val)
                    fila = getattr(hub, "fila", None)
                    if fila is not None:
                        fila.runout_helper.note_filament_present(eventtime, hub_val)
                    self._last_hub_states[hub.name] = hub_val
            self._sync_virtual_tool_sensor(eventtime)

        except Exception:
            # Avoid stopping the reactor loop if OpenAMS query fails.
            pass

        return eventtime + self.interval


def load_config_prefix(config):
    return afcAMS(config)
