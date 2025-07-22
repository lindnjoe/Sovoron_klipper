# Combined virtual pin and filament sensor module for Klipper
#
# Provides a software-defined input pin that can be attached anywhere an
# endstop would normally be used, and optionally emulates a filament
# switch sensor using that pin.
#
# Copyright (C) 2024  The Klipper Project
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging

class VirtualEndstop:
    """Simple endstop-like object representing a virtual input."""
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
    """Configure and manage a virtual input pin."""
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.state = config.getboolean('initial_value', False)
        # Use a set to avoid duplicate callbacks
        self._watchers = set()
        self._button_handlers = []
        self._ack_count = 0

        ppins = self.printer.lookup_object('pins')
        try:
            ppins.register_chip('ams_pin', self)
        except ppins.error:
            pass

        gcode = self.printer.lookup_object('gcode')
        cname = self.name
