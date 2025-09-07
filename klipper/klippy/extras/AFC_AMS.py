# Armored Turtle Automated Filament Control - OpenAMS integration
#
# Copyright (C) 2025 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import traceback

from configparser import Error as error

try:
    from extras.AFC_utils import ERROR_STR
except Exception:
    raise error(
        "Error when trying to import AFC_utils.ERROR_STR\n{trace}".format(
            trace=traceback.format_exc()
        )
    )

try:
    from extras.AFC_BoxTurtle import afcBoxTurtle
except Exception:
    raise error(
        ERROR_STR.format(import_lib="AFC_BoxTurtle", trace=traceback.format_exc())
    )


SYNC_INTERVAL = 2.0


class afcAMS(afcBoxTurtle):
    """AFC unit that syncs lane and hub sensors directly from OpenAMS."""

    def __init__(self, config):
        super().__init__(config)
        self.type = config.get("type", "AMS")
        self.oams_name = config.get("oams", "oams1")
        self.interval = config.getfloat("interval", SYNC_INTERVAL, above=0.0)
        self.reactor = self.printer.get_reactor()
        self.oams = None
        self.timer = self.reactor.register_timer(self._sync_event)
        self.last_lane_vals = [None] * 4
        self.last_hub_vals = [None] * 4
        self.printer.register_event_handler("klippy:ready", self.handle_ready)

    def handle_ready(self):
        self.oams = self.printer.lookup_object("oams " + self.oams_name, None)
        self.reactor.update_timer(self.timer, self.reactor.NOW)

    def _sync_event(self, eventtime):
        try:
            if self.oams is None:
                self.oams = self.printer.lookup_object("oams " + self.oams_name, None)
                if self.oams is None:
                    return eventtime + self.interval
            f1s = getattr(self.oams, "f1s_hes_value", [0, 0, 0, 0])
            hubs = getattr(self.oams, "hub_hes_value", [0, 0, 0, 0])
            for lane in self.lanes.values():
                idx = getattr(lane, "index", 0)
                lane_state = bool(f1s[idx])
                if self.last_lane_vals[idx] != lane_state:
                    lane.prep_callback(eventtime, lane_state)
                    lane.load_callback(eventtime, lane_state)
                    self.last_lane_vals[idx] = lane_state
                hub_state = bool(hubs[idx])
                hub = lane.hub_obj
                if hub is not None and self.last_hub_vals[idx] != hub_state:
                    hub.switch_pin_callback(eventtime, hub_state)
                    self.last_hub_vals[idx] = hub_state
        except Exception:
            self.logger.exception("AFC_AMS update error")
        return eventtime + self.interval


def load_config_prefix(config):
    return afcAMS(config)