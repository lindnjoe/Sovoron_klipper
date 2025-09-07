# Armored Turtle Automated Filament Control - AMS integration
#
# Copyright (C) 2025
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import traceback

from configparser import Error as error

try:
    from extras.AFC_unit import afcUnit
except Exception:
    raise error("Error when trying to import AFC_unit\n{trace}".format(trace=traceback.format_exc()))
try:
    from extras.AFC_lane import AFCLaneState
except Exception:
    raise error("Error when trying to import AFC_lane\n{trace}".format(trace=traceback.format_exc()))

SYNC_INTERVAL = 2.0


class afcAMS(afcUnit):
    """AFK unit that synchronizes lane and hub states with OpenAMS."""

    def __init__(self, config):
        super().__init__(config)
        self.type = "AMS"
        self.oams_name = config.get("oams", "oams1")
        self.interval = config.getfloat("interval", SYNC_INTERVAL, above=0.0)

        self.reactor = self.printer.get_reactor()
        self.timer = self.reactor.register_timer(self._sync_event)
        self.printer.register_event_handler("klippy:ready", self.handle_ready)

        # Track last sensor states so callbacks only trigger on changes
        self._last_lane_states = {}
        self._last_hub_states = {}

    def handle_connect(self):
        """Ensure base AFC connectivity."""
        super().handle_connect()

    def system_Test(self, cur_lane, delay, assignTcmd, enable_movement):
        msg = ''
        succeeded = True

        cur_lane.unsync_to_extruder(False)

        if not cur_lane.prep_state:
            if not cur_lane.load_state:
                self.afc.function.afc_led(cur_lane.led_not_ready, cur_lane.led_index)
                msg += 'EMPTY READY FOR SPOOL'
            else:
                self.afc.function.afc_led(cur_lane.led_fault, cur_lane.led_index)
                msg += '<span class=error--text> NOT READY</span>'
                cur_lane.do_enable(False)
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

        hub = getattr(cur_lane, 'hub_obj', None)
        if hub is not None:
            if hub.state:
                msg += '<span class=primary--text> HUB ACTIVE</span>'
            else:
                msg += '<span class=error--text> HUB INACTIVE</span>'
                succeeded = False

        if assignTcmd:
            self.afc.function.TcmdAssign(cur_lane)
        cur_lane.do_enable(False)
        self.logger.info('{lane_name} {msg}'.format(lane_name=cur_lane.name, msg=msg))
        cur_lane.set_afc_prep_done()
        return succeeded

    def handle_ready(self):
        # Resolve OpenAMS object and start periodic polling
        self.oams = self.printer.lookup_object("oams " + self.oams_name, None)
        self.reactor.update_timer(self.timer, self.reactor.NOW)

    def _sync_event(self, eventtime):
        try:
            if self.oams is None:
                return eventtime + self.interval

            # Iterate through lanes belonging to this unit
            for lane in list(self.lanes.values()):
                idx = getattr(lane, "index", 0) - 1
                if idx < 0:
                    continue

                lane_val = bool(self.oams.f1s_hes_value[idx])
                last_lane = self._last_lane_states.get(lane.name)
                if lane_val != last_lane:
                    lane.load_callback(eventtime, lane_val)
                    lane.prep_callback(eventtime, lane_val)
                    self._last_lane_states[lane.name] = lane_val

                hub = getattr(lane, "hub_obj", None)
                if hub is None:
                    continue

                hub_val = bool(self.oams.hub_hes_value[idx])
                last_hub = self._last_hub_states.get(hub.name)
                if hub_val != last_hub:
                    hub.switch_pin_callback(eventtime, hub_val)
                    self._last_hub_states[hub.name] = hub_val

        except Exception:
            # Avoid breaking the reactor loop if OpenAMS query fails
            pass

        return eventtime + self.interval


def load_config_prefix(config):
    return afcAMS(config)
