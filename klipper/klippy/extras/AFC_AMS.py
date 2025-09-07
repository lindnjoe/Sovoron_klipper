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

# -----------------------------------------------------------------------------
# Monkey patch afc_hub to allow virtual switch pins for AMS hubs
# -----------------------------------------------------------------------------
try:
    import extras.AFC_hub as _AFC_HUB

    def _patched_hub_init(self, config):
        self.printer = config.get_printer()
        self.printer.register_event_handler("klippy:connect", self.handle_connect)
        self.afc = self.printer.lookup_object('AFC')
        self.fullname = config.get_name()
        self.name = self.fullname.split()[-1]

        self.unit = None
        self.lanes = {}
        self.state = False

        # HUB Cut variables
        # Next two variables are used in AFC
        self.switch_pin = config.get('switch_pin', None)
        self.hub_clear_move_dis = config.getfloat("hub_clear_move_dis", 25)
        self.afc_bowden_length = config.getfloat("afc_bowden_length", 900)
        self.afc_unload_bowden_length = config.getfloat(
            "afc_unload_bowden_length", self.afc_bowden_length)
        self.assisted_retract = config.getboolean("assisted_retract", False)
        self.move_dis = config.getfloat("move_dis", 50)
        # Servo settings
        self.cut = config.getboolean("cut", False)
        self.cut_cmd = config.get('cut_cmd', None)
        self.cut_servo_name = config.get('cut_servo_name', 'cut')
        self.cut_dist = config.getfloat("cut_dist", 50)
        self.cut_clear = config.getfloat("cut_clear", 120)
        self.cut_min_length = config.getfloat("cut_min_length", 200)
        self.cut_servo_pass_angle = config.getfloat("cut_servo_pass_angle", 0)
        self.cut_servo_clip_angle = config.getfloat("cut_servo_clip_angle", 160)
        self.cut_servo_prep_angle = config.getfloat("cut_servo_prep_angle", 75)
        self.cut_confirm = config.getboolean("cut_confirm", 0)

        self.config_bowden_length = self.afc_bowden_length
        self.config_unload_bowden_length = self.afc_unload_bowden_length
        self.enable_sensors_in_gui = config.getboolean(
            "enable_sensors_in_gui", self.afc.enable_sensors_in_gui)

        buttons = self.printer.load_object(config, "buttons")
        if self.switch_pin in (None, "None", ""):
            # Create a virtual pin so the hub can be referenced without
            # requiring a physical switch pin in the config.
            self.switch_pin = f"afc_virtual_bypass:hub_{self.name}"
        self.state = False
        buttons.register_buttons([self.switch_pin], self.switch_pin_callback)

        if self.enable_sensors_in_gui:
            self.filament_switch_name = (
                "filament_switch_sensor {}_Hub".format(self.name))
            self.fila = _AFC_HUB.add_filament_switch(
                self.filament_switch_name, self.switch_pin, self.printer)

        # Adding self to AFC hubs
        self.afc.hubs[self.name] = self

    _AFC_HUB.afc_hub.__init__ = _patched_hub_init
except Exception:
    # If the hub module isn't present we silently ignore the patch
    pass

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
        """Ensure base AFC connectivity and set up logos."""
        super().handle_connect()

        firstLeg = '<span class=warning--text>|</span><span class=error--text>_</span>'
        secondLeg = firstLeg + '<span class=warning--text>|</span>'
        self.logo = '<span class=success--text>R  _____     ____\n'
        self.logo += 'E /      \\  |  </span><span class=info--text>o</span><span class=success--text> | \n'
        self.logo += 'A |       |/ ___/ \n'
        self.logo += 'D |_________/     \n'
        self.logo += 'Y {first}{second} {first}{second}\n'.format(first=firstLeg, second=secondLeg)
        self.logo += '  ' + self.name + '\n'

        self.logo_error = '<span class=error--text>E  _ _   _ _\n'
        self.logo_error += 'R |_|_|_|_|_|\n'
        self.logo_error += 'R |         \\____\n'
        self.logo_error += 'O |              \\ \n'
        self.logo_error += 'R |          |\\ <span class=secondary--text>X</span> |\\n'
        self.logo_error += '! \\_________/ |___|</span>\n'
        self.logo_error += '  ' + self.name + '\n'

    def system_Test(self, cur_lane, delay, assignTcmd, enable_movement):
        msg = ''
        succeeded = True

        # For AMS units we only need to verify sensor states and do not
        # attempt any filament movements or toolhead synchronization.
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
                    if (cur_lane.get_toolhead_pre_sensor_state() or
                            cur_lane.extruder_obj.tool_start == "buffer" or
                            cur_lane.extruder_obj.tool_end_state):
                        if cur_lane.extruder_obj.lane_loaded == cur_lane.name:
                            cur_lane.sync_to_extruder()
                            msg += '<span class=primary--text> in ToolHead</span>'
                            if cur_lane.extruder_obj.tool_start == "buffer":
                                msg += ('<span class=warning--text>\n Ram sensor enabled, '
                                        'confirm tool is loaded</span>')
                            if self.afc.current == cur_lane.name:
                                self.afc.spool.set_active_spool(cur_lane.spool_id)
                                self.afc.function.afc_led(cur_lane.led_tool_loaded,
                                                          cur_lane.led_index)
                                cur_lane.status = AFCLaneState.TOOLED
                            cur_lane.enable_buffer()
                        else:
                            if (cur_lane.get_toolhead_pre_sensor_state() or
                                    cur_lane.extruder_obj.tool_end_state):
                                msg += (
                                    '<span class=error--text> error in ToolHead. '
                                    '\nLane identified as loaded '
                                    '\n but not identified as loaded in extruder</span>'
                                )
                                succeeded = False
                    else:
                        lane_check = self.afc.error.fix('toolhead', cur_lane)
                        if not lane_check:
                            return False

        if assignTcmd:
            self.afc.function.TcmdAssign(cur_lane)
        cur_lane.do_enable(False)
        self.logger.info('{lane_name} tool cmd: {tcmd:3} {msg}'.format(
            lane_name=cur_lane.name, tcmd=cur_lane.map, msg=msg))
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
                    if hasattr(hub, "fila"):
                        hub.fila.runout_helper.note_filament_present(
                            eventtime, hub_val)
                    self._last_hub_states[hub.name] = hub_val

        except Exception:
            # Avoid breaking the reactor loop if OpenAMS query fails
            pass

        return eventtime + self.interval


def load_config_prefix(config):
    return afcAMS(config)
