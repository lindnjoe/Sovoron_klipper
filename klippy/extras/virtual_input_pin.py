"""Virtual input pin module for Klipper.

This module provides a software-based **input** pin that can be
referenced using the ``ams_pin:`` prefix.  The pin is implemented as an
endstop-style input and cannot be configured for output.  G-code
commands allow software to update or query the stored state.
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
