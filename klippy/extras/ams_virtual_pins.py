# Software-defined AMS pins for Klipper
#
# Creates eight virtual input pins named pin1-pin8 on a fake MCU named 'ams'.
# Pins act like endstop inputs and can be referenced elsewhere in the
# configuration using `ams:pin1`, `ams:pin2`, etc.  Their state can be
# changed at runtime with SET_AMS_PIN and queried with QUERY_AMS_PIN.
#
# Copyright (C) 2024
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging

CHIP_NAME = "ams"
# Allow either "ams" or "virtual_pin" as the chip name so existing
# configurations using the virtual_pin prefix continue to work.
CHIP_ALIASES = (CHIP_NAME, "virtual_pin")

def _norm(name: str) -> str:
    """Normalize a pin name for lookups."""
    return str(name).strip().lower()

class VirtualEndstop:
    def __init__(self, vpin, invert):
        self._vpin = vpin
        self._invert = invert
        self._reactor = vpin.chip.printer.get_reactor()

    def get_mcu(self):
        return None

    def add_stepper(self, stepper):
        pass

    def get_steppers(self):
        return []

    def home_start(self, print_time, *args, **kwargs):
        c = self._reactor.completion()
        c.complete(self.query_endstop(print_time))
        return c

    def home_wait(self, home_end_time):
        if self.query_endstop(home_end_time):
            return home_end_time
        return 0.

    def query_endstop(self, print_time):
        return bool(self._vpin.state) ^ bool(self._invert)

class VirtualInputPin:
    def __init__(self, chip, name):
        self.chip = chip
        self.name = name
        self.state = False
        self._watchers = []

    # MCU helpers forwarded from chip
    def register_config_callback(self, cb):
        self.chip.register_config_callback(cb)
    def register_response(self, cb, *args):
        self.chip.register_response(cb, *args)
    def add_config_cmd(self, cmd, is_init=False):
        self.chip.add_config_cmd(cmd, is_init)
    def alloc_command_queue(self):
        return self.chip.alloc_command_queue()
    def lookup_command(self, *args, **kw):
        return self.chip.lookup_command(*args, **kw)
    def get_query_slot(self, oid):
        return self.chip.get_query_slot(oid)
    def seconds_to_clock(self, seconds):
        return self.chip.seconds_to_clock(seconds)
    def create_oid(self):
        return self.chip.create_oid()

    def setup_pin(self, pin_type, pin_params):
        ppins = self.chip.printer.lookup_object('pins')
        if pin_type != 'endstop':
            raise ppins.error('ams pins only support endstop type')
        invert = pin_params.get('invert', False)
        return VirtualEndstop(self, invert)

    def register_watcher(self, cb):
        if cb not in self._watchers:
            self._watchers.append(cb)
        self._invoke(cb, self.state)

    def _invoke(self, cb, state):
        et = self.chip.printer.get_reactor().monotonic()
        try:
            cb(et, state)
        except TypeError:
            try:
                cb(state)
            except Exception:
                logging.exception('Virtual pin callback error')

    def set_value(self, val):
        val = bool(val)
        if self.state == val:
            return
        self.state = val
        for cb in list(self._watchers):
            self._invoke(cb, val)

    def get_status(self, eventtime):
        return { 'value': int(self.state) }

class VirtualPinChip:
    def __init__(self, printer):
        self.printer = printer
        self.pins = {}
        self._config_cbs = []
        self._oid = 0

    # MCU interface stubs
    def register_config_callback(self, cb):
        self._config_cbs.append(cb)
    def run_config_callbacks(self, eventtime=None):
        cbs = self._config_cbs
        self._config_cbs = []
        for cb in cbs:
            try:
                if eventtime is None:
                    cb()
                else:
                    cb(eventtime)
            except TypeError:
                cb()
    def register_response(self, cb, *args):
        pass
    def add_config_cmd(self, cmd, is_init=False):
        pass
    def alloc_command_queue(self):
        class Dummy: 
            def send(self,*a,**k):
                pass
        return Dummy()
    class _DummyCmd:
        def send(self,*a,**k):
            pass
    def lookup_command(self,*a,**k):
        return self._DummyCmd()
    def get_query_slot(self, oid):
        return 0
    def seconds_to_clock(self, seconds):
        return 0
    def create_oid(self):
        self._oid += 1
        return self._oid

    def setup_pin(self, pin_type, pin_params):
        name = pin_params['pin']
        vp = self.pins.get(name)
        if vp is None:
            ppins = self.printer.lookup_object('pins')
            raise ppins.error('ams pin %s not configured' % name)
        return vp.setup_pin(pin_type, pin_params)

# G-code handlers -------------------------------------------------------

    def cmd_SET_AMS_PIN(self, gcmd):
        name = _norm(gcmd.get('PIN'))
        val = gcmd.get_int('VALUE', 1)
        pin = self.pins.get(name)
        if pin is None:
            gcmd.respond_error('Unknown ams pin %s' % name)
            return
        pin.set_value(val)

    def cmd_QUERY_AMS_PIN(self, gcmd):
        name = _norm(gcmd.get('PIN'))
        pin = self.pins.get(name)
        if pin is None:
            gcmd.respond_error('Unknown ams pin %s' % name)
            return
        gcmd.respond_info('ams:%s=%d' % (name, pin.state))

# Configuration entry point --------------------------------------------

def load_config(config):
    printer = config.get_printer()
    chip = VirtualPinChip(printer)
    ppins = printer.lookup_object('pins')
    for name in CHIP_ALIASES:
        try:
            ppins.register_chip(name, chip)
        except ppins.error:
            pass
    # create eight pins pin1..pin8
    for i in range(1,9):
        name = f'pin{i}'
        vp = VirtualInputPin(chip, name)
        chip.pins[_norm(name)] = vp
        # expose for macros via printer.objects
        objs = getattr(printer, 'objects', None)
        if isinstance(objs, dict):
            objs[f'ams_pin {name}'] = vp
    # register gcode commands
    gcode = printer.lookup_object('gcode')
    gcode.register_command('SET_AMS_PIN', chip.cmd_SET_AMS_PIN,
                           desc='Set ams pin value')
    gcode.register_command('QUERY_AMS_PIN', chip.cmd_QUERY_AMS_PIN,
                           desc='Query ams pin value')
    printer.register_event_handler('klippy:connect', chip.run_config_callbacks)
    printer.ams = chip
    return chip

# allow [ams_virtual_pins] with optional suffix

def load_config_prefix(config):
    return load_config(config)

