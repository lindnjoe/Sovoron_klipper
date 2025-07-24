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
        # Track button handlers for compatibility with modules that
        # expect MCU-style callbacks (eg, buttons.py)
        self._button_handlers = []
        self._ack_count = 0
        self._config_callbacks = []

        # Run deferred config callbacks after Klipper is ready
        self.printer.register_event_handler('klippy:ready',
                                            self._run_config_callbacks)

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
        """Register a callback for state changes and invoke it with the
        current state."""
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
    def register_config_callback(self, cb):
        """Store configuration callbacks to run when Klipper is ready."""
        self._config_callbacks.append(cb)

    def _run_config_callbacks(self, eventtime=None):
        for cb in self._config_callbacks:
            try:
                cb()
            except Exception:
                logging.exception('Virtual pin config callback error')
        self._config_callbacks = []

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

    cmd_SET_AMS_PIN_help = 'Set the value of a virtual input pin'
    def cmd_SET_AMS_PIN(self, gcmd):
        val = gcmd.get_int('VALUE', 1)
        self.set_value(val)

    cmd_QUERY_AMS_PIN_help = 'Report the value of a virtual input pin'
    def cmd_QUERY_AMS_PIN(self, gcmd):
        gcmd.respond_info('ams_pin %s: %d' % (self.name, self.state))


def load_config_prefix(config):
    """Config handler for [ams_pin] sections."""
    prefix = config.get_name().split()[0]
    if prefix != 'ams_pin':
        raise config.error('Unknown prefix %s' % prefix)
    return VirtualInputPin(config)
