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
        self.oams_manager = None

        self.reactor = self.printer.get_reactor()
        self.timer = self.reactor.register_timer(self._sync_event)
        self.printer.register_event_handler("klippy:ready", self.handle_ready)

        # Track last sensor states so callbacks only trigger on changes
        self._last_prep_states = {}
        self._last_load_states = {}
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

    def check_runout(self, cur_lane):
        """Determine if AFC should handle runout for the current lane."""

        # Runout handling is suppressed when both the current lane and its
        # configured runout lane are AMS lanes that share the same FPS
        # (extruder). In that scenario the OpenAMS manager will perform the
        # spool hand-off automatically.
        if (cur_lane.runout_lane is not None
                and getattr(cur_lane.unit_obj, "type", "") == "AMS"):
            ro_lane = self.afc.lanes.get(cur_lane.runout_lane)
            if (ro_lane is not None
                    and getattr(ro_lane.unit_obj, "type", "") == "AMS"
                    and ro_lane.extruder_obj == cur_lane.extruder_obj):
                return False

        return (cur_lane.name == self.afc.function.get_current_lane()
                and self.afc.function.is_printing()
                and cur_lane.status not in (AFCLaneState.EJECTING,
                                            AFCLaneState.CALIBRATING))

    def _trigger_runout(self, lane, force=False):
        """Handle runout for lanes without dedicated load sensors.

        The ``force`` parameter bypasses ``check_runout`` and executes the
        appropriate AFC runout routine unconditionally. This is used when
        OpenAMS reports a runout but cannot reload a backup spool.
        """
        if force or self.check_runout(lane):
            if lane.runout_lane is not None:
                lane._perform_infinite_runout()
            else:
                lane._perform_pause_runout()
        elif lane.status != "calibrating":
            self.afc.function.afc_led(lane.led_not_ready, lane.led_index)
            lane.status = AFCLaneState.NONE
            lane.loaded_to_hub = False
            self.afc.spool._clear_values(lane)
            self.afc.function.afc_led(self.afc.led_not_ready, lane.led_index)
        self.afc.save_vars()

    def _on_oams_runout(self, fps_name, group, spool_idx):
        """Callback from OAMSManager when a runout occurs.

        ``spool_idx`` is the index of the newly loaded spool or ``-1`` if the
        manager could not load another spool and external runout handling is
        required.
        """
        lane = self.lanes.get(group)
        if lane is None:
            return
        eventtime = self.reactor.monotonic()
        self._last_load_states[lane.name] = False
        lane.load_callback(eventtime, False)
        hub = getattr(lane, "hub_obj", None)
        if hub is not None:
            self._last_hub_states[hub.name] = False
            hub.switch_pin_callback(eventtime, False)
            if hasattr(hub, "fila"):
                hub.fila.runout_helper.note_filament_present(eventtime, False)
        if spool_idx < 0:
            ro_lane_name = lane.runout_lane
            if ro_lane_name:
                ro_lane = self.afc.lanes.get(ro_lane_name)
                idx = getattr(ro_lane, "index", 0) - 1 if ro_lane else -1
                ro_unit = getattr(ro_lane, "unit_obj", None) if ro_lane else None
                if (ro_lane is not None and idx >= 0
                        and ro_unit is not None
                        and getattr(ro_unit, "type", "") == "AMS"
                        and ro_lane.extruder_obj == lane.extruder_obj
                        and self.oams_manager is not None
                        and self.oams_manager.load_spool_for_lane(
                            fps_name, ro_lane.name, ro_unit.oams_name, idx)):
                    cur_ext = self.afc.function.get_current_extruder()
                    if cur_ext in self.afc.tools:
                        self.afc.tools[cur_ext].lane_loaded = ro_lane.name
                    ro_lane.unit_obj.lane_loaded(ro_lane)
                    self.afc.spool._clear_values(lane)
                    self.afc.save_vars()
                    return
            self._trigger_runout(lane, force=True)
        else:
            self.afc.spool._clear_values(lane)
            self.afc.save_vars()

    def handle_ready(self):
        # Resolve OpenAMS object and start periodic polling
        self.oams = self.printer.lookup_object("oams " + self.oams_name, None)
        self.oams_manager = self.printer.lookup_object("oams_manager", None)
        if self.oams_manager is not None:
            self.oams_manager.register_runout_callback(self._on_oams_runout)
        self.reactor.update_timer(self.timer, self.reactor.NOW)

    def _sync_event(self, eventtime):
        if self.oams is None:
            return eventtime + self.interval

        # Request updated sensor values from the OpenAMS controller so
        # new spools inserted into empty bays are detected.
        try:
            self.oams.determine_current_spool()
        except Exception:
            pass

        # Iterate through lanes belonging to this unit
        for lane in list(self.lanes.values()):
            idx = getattr(lane, "index", 0) - 1
            if idx < 0:
                continue

            try:
                # OpenAMS exposes separate sensors for spool presence
                # (f1s_hes_value) and the hub path (hub_hes_value). The
                # spool sensor should drive both the prep and load states so
                # that inserting filament reflects immediately in AFC, while
                # the hub sensor is reported separately for informational
                # purposes.
                prep_val = bool(self.oams.f1s_hes_value[idx])
                hub_val = bool(self.oams.hub_hes_value[idx])

                last_prep = self._last_prep_states.get(lane.name)
                if prep_val != last_prep:
                    lane.prep_callback(eventtime, prep_val)
                    self._last_prep_states[lane.name] = prep_val

                load_val = prep_val

                last_load = self._last_load_states.get(lane.name)
                if load_val != last_load:
                    self._last_load_states[lane.name] = load_val
                    if hasattr(lane, "load_debounce_button"):
                        lane.handle_load_runout(eventtime, load_val)
                    else:
                        lane.load_callback(eventtime, load_val)
            except (IndexError, KeyError):
                # Skip lanes that aren't reported by OpenAMS
                continue

            hub = getattr(lane, "hub_obj", None)
            if hub is None:
                continue

            last_hub = self._last_hub_states.get(hub.name)
            if hub_val != last_hub:
                hub.switch_pin_callback(eventtime, hub_val)
                if hasattr(hub, "fila"):
                    hub.fila.runout_helper.note_filament_present(
                        eventtime, hub_val)
                self._last_hub_states[hub.name] = hub_val
                if (not hub_val and not load_val and
                        self.check_runout(lane)):
                    self._trigger_runout(lane)

        return eventtime + self.interval


def load_config_prefix(config):
    return afcAMS(config)
