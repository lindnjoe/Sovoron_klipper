"""Software-defined input pins for Klipper.

This module allows configuration sections like `[virtual_input_pin foo]`
to create pins that behave like MCU endstop inputs.  Each pin can be
queried or updated at runtime using the `SET_VIRTUAL_PIN` and
`QUERY_VIRTUAL_PIN` gcode commands.
"""

import logging


class VirtualEndstop:
    """Minimal endstop object wrapping a virtual pin."""

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

    def home_start(self, print_time, *args, **kwargs):
        comp = self._reactor.completion()
        comp.complete(self.query_endstop(print_time))
        return comp

    def home_wait(self, home_end_time):
        if self.query_endstop(home_end_time):
            return home_end_time
        return 0.

    def query_endstop(self, print_time):
        return bool(self._vpin.state) ^ bool(self._invert)


class _VirtualPinChip:
    """Chip object that delegates pin setup to registered pins."""

    def __init__(self, printer):
        self.printer = printer
        self._config_callbacks = []
        self._response_callbacks = []

    def register_config_callback(self, cb):
        """Immediately invoke configuration callbacks."""
        self._config_callbacks.append(cb)
        try:
            cb(self.printer.get_reactor().monotonic())
        except TypeError:
            cb()

    def register_response(self, cb):
        self._response_callbacks.append(cb)

    def setup_pin(self, pin_type, pin_params):
        ppins = self.printer.lookup_object('pins')
        if pin_type != 'endstop':
            raise ppins.error('virtual_pin only supports endstop type')
        name = pin_params['pin']
        obj = self.printer.lookup_object('virtual_input_pin ' + name, None)
        if obj is None:
            obj = self.printer.lookup_object('virtual_pin ' + name, None)
        if obj is None:
            raise ppins.error('virtual_pin %s not configured' % (name,))
        return obj.setup_pin(pin_type, pin_params)


_CHIPS = {}


def _ensure_chip(printer):
    chip = _CHIPS.get(printer)
    if chip is not None:
        return chip
    chip = _VirtualPinChip(printer)
    ppins = printer.lookup_object('pins')
    for cname in ('virtual_pin', 'ams_pin', 'ams'):
        try:
            ppins.register_chip(cname, chip)
        except ppins.error:
            pass
    _CHIPS[printer] = chip
    return chip


class VirtualInputPin:
    """Representation of a virtual input pin."""

    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.state = config.getboolean('initial_value', False)
        self._watchers = []

        _ensure_chip(self.printer)

        gcode = self.printer.lookup_object('gcode')
        cname = self.name
        gcode.register_mux_command('SET_VIRTUAL_PIN', 'PIN', cname,
                                   self._cmd_set,
                                   desc=self._cmd_set_help)
        gcode.register_mux_command('QUERY_VIRTUAL_PIN', 'PIN', cname,
                                   self._cmd_query,
                                   desc=self._cmd_query_help)

    def setup_pin(self, pin_type, pin_params):
        ppins = self.printer.lookup_object('pins')
        if pin_type != 'endstop':
            raise ppins.error('virtual_pin only supports endstop type')
        return VirtualEndstop(self, pin_params['invert'])

    # watcher helpers -------------------------------------------------
    def register_watcher(self, cb):
        self._watchers.append(cb)
        try:
            cb(self.printer.get_reactor().monotonic(), self.state)
        except Exception:
            logging.exception('virtual pin callback error')

    def set_value(self, val):
        val = bool(val)
        if self.state == val:
            return
        self.state = val
        et = self.printer.get_reactor().monotonic()
        for cb in list(self._watchers):
            try:
                cb(et, val)
            except Exception:
                logging.exception('virtual pin callback error')

    # gcode commands --------------------------------------------------
    _cmd_set_help = 'Set the value of a virtual input pin'

    def _cmd_set(self, gcmd):
        val = gcmd.get_int('VALUE', 1)
        self.set_value(val)

    _cmd_query_help = 'Report the value of a virtual input pin'

    def _cmd_query(self, gcmd):
        gcmd.respond_info('virtual_pin %s: %d' % (self.name, self.state))


# Configuration entry point

def load_config_prefix(config):
    return VirtualInputPin(config)
