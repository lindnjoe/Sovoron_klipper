# Virtual input pins for Klipper
#
# Provides software-defined input pins that behave like endstop pins.
# Add sections like:
#   [virtual_input_pin my_pin]
#   initial_value: 0
# The pin can then be referenced as virtual_pin:my_pin.
# State can be updated at runtime using SET_VIRTUAL_PIN and queried with
# QUERY_VIRTUAL_PIN.
#
# Copyright (C) 2024
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging

CHIP_NAME = 'virtual_pin'


def _norm(name):
    return str(name).strip().lower()


class VirtualEndstop:
    def __init__(self, pin, invert):
        self._pin = pin
        self._invert = invert
        self._reactor = pin.chip.printer.get_reactor()

    def get_mcu(self):
        return None

    def add_stepper(self, stepper):
        pass

    def get_steppers(self):
        return []

    def home_start(self, print_time, *args, **kw):
        comp = self._reactor.completion()
        comp.complete(self.query_endstop(print_time))
        return comp

    def home_wait(self, home_end_time):
        return home_end_time if self.query_endstop(home_end_time) else 0.

    def query_endstop(self, print_time):
        return bool(self._pin.state) ^ bool(self._invert)


class VirtualInputPin:
    def __init__(self, chip, name, initial=False):
        self.chip = chip
        self.name = name
        self.state = bool(initial)
        self._watchers = set()

    def setup_pin(self, pin_type, pin_params):
        ppins = self.chip.printer.lookup_object('pins')
        if pin_type != 'endstop':
            raise ppins.error('virtual pins only support endstop type')
        invert = pin_params.get('invert', False)
        return VirtualEndstop(self, invert)

    # watcher helpers -------------------------------------------------
    def _invoke(self, cb, state):
        et = self.chip.printer.get_reactor().monotonic()
        try:
            cb(et, state)
            return
        except TypeError:
            pass
        try:
            cb(state)
            return
        except TypeError:
            pass
        try:
            cb(et)
        except Exception:
            logging.exception('Virtual pin callback error')

    def register_watcher(self, cb):
        if cb in self._watchers:
            return
        self._watchers.add(cb)
        self._invoke(cb, self.state)

    def set_value(self, val):
        val = bool(val)
        if val == self.state:
            return
        self.state = val
        for cb in list(self._watchers):
            self._invoke(cb, val)

    def get_status(self, eventtime):
        return {'value': int(self.state)}


class VirtualPinChip:
    def __init__(self, printer):
        self.printer = printer
        self.pins = {}
        self._config_cbs = []
        self._oid = 0

    # minimal MCU stubs ----------------------------------------------
    def register_config_callback(self, cb):
        self._config_cbs.append(cb)

    def run_config_callbacks(self, eventtime=None):
        cbs = self._config_cbs
        self._config_cbs = []
        for cb in cbs:
            try:
                cb(eventtime) if eventtime is not None else cb()
            except TypeError:
                cb()

    def register_response(self, cb, *args):
        pass

    def add_config_cmd(self, cmd, is_init=False):
        pass

    def alloc_command_queue(self):
        class Dummy:
            def send(self, *a, **k):
                pass
        return Dummy()

    class _DummyCmd:
        def send(self, *a, **k):
            pass

    def lookup_command(self, *a, **k):
        return self._DummyCmd()

    def get_query_slot(self, oid):
        return 0

    def seconds_to_clock(self, seconds):
        return 0

    def create_oid(self):
        self._oid += 1
        return self._oid

    # pin management --------------------------------------------------
    def register_pin(self, pin):
        key = _norm(pin.name)
        if key in self.pins:
            logging.warning('Duplicate virtual pin %s', pin.name)
        self.pins[key] = pin
        objs = getattr(self.printer, 'objects', None)
        if isinstance(objs, dict):
            objs[f'{CHIP_NAME} {pin.name}'] = pin

    def setup_pin(self, pin_type, pin_params):
        name = _norm(pin_params['pin'])
        pin = self.pins.get(name)
        if pin is None:
            ppins = self.printer.lookup_object('pins')
            raise ppins.error('virtual pin %s not configured' % name)
        return pin.setup_pin(pin_type, pin_params)

    # gcode helpers ---------------------------------------------------
    def _parse_pin_arg(self, gcmd):
        name = _norm(gcmd.get('PIN'))
        if name.startswith(CHIP_NAME + ':'):
            name = name.split(':', 1)[1]
        return name

    def cmd_SET_VIRTUAL_PIN(self, gcmd):
        name = self._parse_pin_arg(gcmd)
        val = gcmd.get_int('VALUE', 1)
        pin = self.pins.get(name)
        if pin is None:
            gcmd.respond_info('Unknown virtual pin %s' % name)
            return
        pin.set_value(val)

    def cmd_QUERY_VIRTUAL_PIN(self, gcmd):
        name = self._parse_pin_arg(gcmd)
        pin = self.pins.get(name)
        if pin is None:
            gcmd.respond_info('Unknown virtual pin %s' % name)
            return
        gcmd.respond_info(f'{CHIP_NAME}:{name}={int(pin.state)}')


def _ensure_chip(printer):
    chip = getattr(printer, '_virtual_pin_chip', None)
    if chip is None:
        chip = VirtualPinChip(printer)
        setattr(printer, '_virtual_pin_chip', chip)
        ppins = printer.lookup_object('pins')
        try:
            ppins.register_chip(CHIP_NAME, chip)
        except ppins.error:
            pass
        gcode = printer.lookup_object('gcode')
        gcode.register_command('SET_VIRTUAL_PIN', chip.cmd_SET_VIRTUAL_PIN,
                               desc='Set virtual pin value')
        gcode.register_command('QUERY_VIRTUAL_PIN', chip.cmd_QUERY_VIRTUAL_PIN,
                               desc='Query virtual pin value')
        printer.register_event_handler('klippy:connect', chip.run_config_callbacks)
    return chip


def load_config_prefix(config):
    printer = config.get_printer()
    chip = _ensure_chip(printer)
    name = config.get_name().split()[-1]
    init_val = config.getboolean('initial_value', False)
    pin = VirtualInputPin(chip, name, init_val)
    chip.register_pin(pin)
    return pin
