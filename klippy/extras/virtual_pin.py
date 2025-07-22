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
        gcode.register_mux_command('SET_VIRTUAL_PIN', 'PIN', cname,
                                   self.cmd_SET_VIRTUAL_PIN,
                                   desc=self.cmd_SET_VIRTUAL_PIN_help)
        gcode.register_mux_command('QUERY_VIRTUAL_PIN', 'PIN', cname,
                                   self.cmd_QUERY_VIRTUAL_PIN,
                                   desc=self.cmd_QUERY_VIRTUAL_PIN_help)

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

    def get_status(self, eventtime):
        return {'value': int(self.state)}

    def set_value(self, val):
        val = bool(val)
        if self.state == val:
            return
        self.state = val
        for cb in list(self._watchers):
            try:
                cb(val)
            except Exception:
                logging.exception('Virtual pin callback error')
        if self._button_handlers:
            params = {
                'ack_count': self._ack_count & 0xff,
                'state': bytes([int(val)]),
                '#receive_time': self.printer.get_reactor().monotonic(),
            }
            self._ack_count += 1
            for handler in list(self._button_handlers):
                try:
                    handler(params)
                except Exception:
                    logging.exception('Virtual button handler error')

    def register_watcher(self, callback):
        self._watchers.add(callback)

    # ------------------------------------------------------------------
    # Minimal MCU interface for buttons.py compatibility
    # ------------------------------------------------------------------
    def register_config_callback(self, cb):
        # Virtual pins have no config stage; invoke callback immediately
        try:
            cb()
        except Exception:
            logging.exception('Virtual pin config callback error')

    def create_oid(self):
        self._ack_count = 0
        return 0

    def add_config_cmd(self, cmd, is_init=False, on_restart=False):
        pass

    class _DummyCmd:
        def send(self, params):
            pass

    def alloc_command_queue(self):
        return None

    def lookup_command(self, template, cq=None):
        return self._DummyCmd()

    def get_query_slot(self, oid):
        return 0

    def seconds_to_clock(self, time):
        return 0

    def register_response(self, handler, resp_name=None, oid=None):
        if resp_name == 'buttons_state':
            self._button_handlers.append(handler)
