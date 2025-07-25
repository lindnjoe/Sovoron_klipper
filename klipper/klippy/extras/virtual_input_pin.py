"""Global virtual input pins for Klipper.

This module provides eight software-based input pins accessible via the
``ams_pin:`` prefix.  Pins are named ``pin1`` through ``pin8`` and may be
used anywhere an endstop style input is expected.  G-code commands allow
software to update or query the stored state.
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


class VirtualPin:
    """Store state and callbacks for a single virtual pin."""
    def __init__(self, printer, name):
        self.printer = printer
        self.name = name
        self.state = False
        self._watchers = set()
        self._button_handlers = []
        self._ack_count = 0

    def register_watcher(self, callback):
        self._watchers.add(callback)
        try:
            callback(self.state)
        except Exception:
            logging.exception('Virtual pin callback error')

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

    def get_status(self, eventtime):
        return {'value': int(self.state)}

    # --------------------------------------------------------------
    # Minimal MCU interface for compatibility with modules such as
    # buttons.py that expect MCU objects
    # --------------------------------------------------------------
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
            params = {
                'ack_count': self._ack_count & 0xff,
                'state': bytes([int(self.state)]),
                '#receive_time': self.printer.get_reactor().monotonic(),
            }
            self._ack_count += 1
            try:
                handler(params)
            except Exception:
                logging.exception('Virtual button handler error')


class VirtualPins:
    """Manage eight global virtual pins."""
    PIN_NAMES = [f'pin{i}' for i in range(1, 9)]

    def __init__(self, config):
        self.printer = config.get_printer()
        self.pins = {name: VirtualPin(self.printer, name)
                     for name in self.PIN_NAMES}

        ppins = self.printer.lookup_object('pins')
        ppins.register_chip('ams_pin', self)

        gcode = self.printer.lookup_object('gcode')
        for name in self.PIN_NAMES:
            gcode.register_mux_command('SET_AMS_PIN', 'PIN', name,
                                       self._set_factory(name),
                                       desc=self.cmd_SET_AMS_PIN_help)
            gcode.register_mux_command('QUERY_AMS_PIN', 'PIN', name,
                                       self._query_factory(name),
                                       desc=self.cmd_QUERY_AMS_PIN_help)

    def _set_factory(self, name):
        def handler(gcmd, pin=name):
            val = gcmd.get_int('VALUE', 1)
            self.pins[pin].set_value(val)
        return handler

    def _query_factory(self, name):
        def handler(gcmd, pin=name):
            state = self.pins[pin].state
            gcmd.respond_info('ams_pin %s: %d' % (pin, state))
        return handler

    def setup_pin(self, pin_type, pin_params):
        ppins = self.printer.lookup_object('pins')
        if pin_type != 'endstop':
            raise ppins.error('ams_pin pins only support endstop type')
        pin_name = pin_params['pin']
        pin = self.pins.get(pin_name)
        if pin is None:
            raise ppins.error('ams_pin %s not configured' % (pin_name,))
        return VirtualEndstop(pin, pin_params['invert'])

    cmd_SET_AMS_PIN_help = 'Set the value of a virtual input pin'
    cmd_QUERY_AMS_PIN_help = 'Report the value of a virtual input pin'


_chip = None


def load_config_prefix(config):
    """Config handler for [ams_pin] sections."""
    global _chip
    if _chip is None:
        _chip = VirtualPins(config)
    return _chip
