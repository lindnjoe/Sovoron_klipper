# Filament Motion Sensor Module
#
# Copyright (C) 2021 Joshua Wherrett <thejoshw.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import filament_switch_sensor

CHECK_RUNOUT_TIMEOUT = .100

class EncoderSensor:
    def __init__(self, config):
        # Read config
        self.printer = config.get_printer()
        switch_pin = config.get('switch_pin')
        self.extruder_name = config.get('extruder')
        self.detection_length = config.getfloat(
                'detection_length', 7., above=0.)
        # Configure pins
        buttons = self.printer.load_object(config, 'buttons')
        if config.get('analog_range', None) is None:
            buttons.register_buttons([switch_pin], self.encoder_event)
        else:
            amin, amax = config.getfloatlist('analog_range', count=2)
            pullup = config.getfloat('analog_pullup_resistor', 4700., above=0.)
            buttons.register_adc_button(switch_pin, amin, amax, pullup, self.encoder_event)
        # buttons.register_buttons([switch_pin], self.encoder_event)
        # Get printer objects
        self.reactor = self.printer.get_reactor()
        self.runout_helper = filament_switch_sensor.RunoutHelper(config)
        self.get_status = self.runout_helper.get_status
        self.extruder = None
        self.estimated_print_time = None
        # Initialise internal state
        self.filament_runout_pos = None
        self.runout_buttun_state = False
        self._extruder_pos_update_timer = None
        # Register commands and event handlers
        self.printer.register_event_handler('klippy:ready',
                self._handle_ready)
        # self.printer.register_event_handler('idle_timeout:printing',
        #         self._handle_printing)
        self.printer.register_event_handler('idle_timeout:ready',
                self._handle_not_printing)
        self.printer.register_event_handler('idle_timeout:idle',
                self._handle_not_printing)
        self.printer.register_event_handler('print_stats:start',
                self._handle_start_print_job)
        self.printer.register_event_handler('print_stats:stop',
                self._handle_stop_print_job)
    def _update_filament_runout_pos(self, eventtime=None, fast_runout=False):
        if eventtime is None:
            eventtime = self.reactor.monotonic()
        if fast_runout:
            self.filament_runout_pos = (self._get_extruder_pos(eventtime) + 0.2)
        else:
            self.filament_runout_pos = (self._get_extruder_pos(eventtime) + self.detection_length)
    def _handle_ready(self):
        self.extruder = self.printer.lookup_object(self.extruder_name)
        self.estimated_print_time = (
                self.printer.lookup_object('mcu').estimated_print_time)
        self._update_filament_runout_pos()
        self._extruder_pos_update_timer = self.reactor.register_timer(
                self._extruder_pos_update_event)
    # def _handle_printing(self, print_time):
    #     self.reactor.update_timer(self._extruder_pos_update_timer,
    #             self.reactor.NOW)
    def _handle_not_printing(self, print_time):
        self.reactor.update_timer(self._extruder_pos_update_timer,
                self.reactor.NEVER)
    def _handle_start_print_job(self):
        if self.runout_buttun_state == False:
            self.reactor.update_timer(self._extruder_pos_update_timer, self.reactor.NOW)
            self._update_filament_runout_pos(fast_runout=True)
    def _handle_stop_print_job(self):
        if self.runout_buttun_state == False:
            self.runout_helper.note_filament_present(self.runout_buttun_state, True)
    def _get_extruder_pos(self, eventtime=None):
        if eventtime is None:
            eventtime = self.reactor.monotonic()
        print_time = self.estimated_print_time(eventtime)
        return self.extruder.find_past_position(print_time)
    def _extruder_pos_update_event(self, eventtime):
        extruder_pos = self._get_extruder_pos(eventtime)
        # Check for filament runout
        is_runout = (extruder_pos >= self.filament_runout_pos)
        if (is_runout):
            self.runout_helper.note_filament_present(False, True)
            return self.reactor.NEVER
        return eventtime + CHECK_RUNOUT_TIMEOUT
    def encoder_event(self, eventtime, state):
        self.runout_buttun_state = state
        print_stats = self.printer.lookup_object('print_stats')
        if print_stats.state == "printing":
            if self.extruder is not None:
                if state == True:
                    self.reactor.update_timer(self._extruder_pos_update_timer,
                            self.reactor.NEVER)
                    self.runout_helper.note_filament_present(True)
                else:
                    self.reactor.update_timer(self._extruder_pos_update_timer,
                            self.reactor.NOW)
                    self._update_filament_runout_pos(eventtime)
        else:
            self.runout_helper.note_filament_present(state)
            if self._extruder_pos_update_timer != None:
                self.reactor.update_timer(self._extruder_pos_update_timer, self.reactor.NEVER)

def load_config_prefix(config):
    return EncoderSensor(config)
