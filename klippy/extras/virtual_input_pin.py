"""Virtual input pin module for Klipper.

This module provides a software-based input pin that can be referenced
using the ``ams_pin:`` prefix.  The pin behaves like a standard endstop
and exposes gcode commands to set and query its state.
"""

import logging

class VirtualEndstop:
    """Simple endstop object backed by a virtual pin."""
    def __init__(self, vpin, invert):
        self._vpin = vpin
        self._invert = invert
        self._reactor = vpin.printer.get_reactor()

    def get_mcu(self):
        return None

    def add_stepper(self, stepper):
        pass

    def get_steppers(self):
        return []

    def home_start(self, print_time, sample_time, sample_count, rest_time,
                   triggered=True):
        comp = self._reactor.completion()
        comp.complete(self.query_endstop(print_time))
        return comp

    def home_wait(self, home_end_time):
        if self.query_endstop(home_end_time):
            return home_end_time
        return 0.

    def query_endstop(self, print_time):
        return bool(self._vpin.state) ^ bool(self._invert)

class VirtualInputPin:
    """Manage a single virtual input pin."""
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.state = config.getboolean('initial_value', False)
        self._watchers = set()

        ppins = self.printer.lookup_object('pins')
        try:
            ppins.register_chip('ams_pin', self)
        except ppins.error:
            pass

        gcode = self.printer.lookup_object('gcode')
        cname = self.name
        gcode.register_mux_command('SET_AMS_PIN', 'PIN', cname,
                                   self.cmd_SET_AMS_PIN,
                                   desc=self.cmd_SET_AMS_PIN_help)
        gcode.register_mux_command('QUERY_AMS_PIN', 'PIN', cname,
                                   self.cmd_QUERY_AMS_PIN,
                                   desc=self.cmd_QUERY_AMS_PIN_help)

    # called by the pins framework
    def setup_pin(self, pin_type, pin_params):
        ppins = self.printer.lookup_object('pins')
        pin_name = pin_params['pin']
        if pin_name != self.name:
            obj = self.printer.lookup_object('ams_pin ' + pin_name, None)
            if obj is None:
                raise ppins.error('ams_pin %s not configured' % (pin_name,))
            return obj.setup_pin(pin_type, pin_params)
        if pin_type != 'endstop':
            raise ppins.error('ams_pin pins only support endstop type')
        return VirtualEndstop(self, pin_params['invert'])

    def register_watcher(self, callback):
